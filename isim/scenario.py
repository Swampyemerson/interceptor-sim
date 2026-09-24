"""Named engagement scenarios for the isim Monte-Carlo runner (ADR-0102 isim).

`Scenario` is a small, picklable description of one engagement's geometry and
pre-flight error injection; `build()` turns it into the five objects
`isim.engine.run_engagement` needs, wiring the UNMODIFIED real flight code
(`flight.deploy.real_flight.RealFlightSM` via `isim.flight_adapter`) so every
scenario flies the same state machine that will fly on the vehicle.

GEOMETRY CONVENTION (a fresh design, not reused from `isim/cli.py`'s ad hoc
one -- that module's `cross_range` is a STARTING east offset with the
perpendicular miss distance fixed by a separate constant, which does not match
the "cross_range_m abeam" wording this task specifies):

  * The interceptor launches from the NED origin (0, 0, -standby_alt_m).
  * Downrange axis = NORTH, cross-track axis = EAST.
  * CROSSING (`head_on=False`): the target flies a straight line parallel to
    NORTH, at a constant EAST offset of `cross_range_m` -- so `cross_range_m`
    *is* the perpendicular ("abeam") distance from the launch point to the
    target's track, by construction (the launch-to-track segment runs along
    EAST, i.e. exactly perpendicular to the target's NORTH-axis velocity --
    which is what "moving perpendicular to the initial line" means: the
    initial line is launch -> abeam point). `direction` is the sign of the
    NORTH velocity (+1 = south-to-north, -1 = north-to-south); the target
    starts `lead_dist_m` before it reaches the abeam point (NORTH = 0) along
    whichever way it is travelling.
  * HEAD-ON (`head_on=True`): the target starts `lead_dist_m` due
    NORTH (`direction=+1`) or SOUTH (`direction=-1`) of the launch point and
    flies straight at it -- no `cross_range_m` term.
  * Altitude: a nominal shared altitude (`MissionConfig()`'s own
    `standby_alt_m` default, reused rather than inventing a second magic
    number) is the reference both sides fly relative to.
    `target_alt_offset_m` sets the TRUE target altitude relative to that
    nominal (0 = same height); `height_guess_error_m` sets what the OPERATOR
    programmed as the vehicle's standby altitude relative to the same nominal
    -- i.e. it perturbs `MissionConfig.standby_alt_m` itself, modelling a
    pre-flight height mis-estimate, never a live read.
  * `aim_error_deg` is added, post-solve, to the pre-flight collision-lead
    heading `flight.deploy.real_flight.resolve_preflight_heading` computes --
    an error in that pre-flight solve, not a perception error.
  * `sprint_scale` multiplies `MissionConfig.dash_speed_ms` (the SAME scaled
    value also feeds the heading solve, so the aim stays self-consistent with
    the speed actually flown). `dash_accel_ms2` is left at its
    `MissionConfig` default -- scaling it is not asked for and would change a
    second, independent thing.

    KNOWN LIMITATION (read, not fixed, per the task brief -- `flight/` is not
    editable from here): `sprint_scale=0.0` does NOT skip CODED_DASH. There is
    no external hook to jump STANDBY -> ENGAGE; `RealFlightSM._step_coded_dash`
    can only leave CODED_DASH via the acquire-streak gate (>= `acquire_streak`
    fresh detections) or a failsafe abort to SAFE. At `sprint_scale=0` the
    vehicle instead HOVERS in CODED_DASH at the dash heading/altitude
    (`dash_forward_speed(0, accel_cap, elapsed) == 0` for every `elapsed`)
    and either acquires the target from a standstill or times out at
    `dash_max_s` to SAFE. That is a real, honestly-reported behaviour of the
    unmodified flight code, not a bug in this module.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Optional, Tuple

import numpy as np

from flight.deploy.real_flight import MissionConfig, resolve_preflight_heading, wrap_deg

from isim.concepts import (
    DualTagSeeker,
    HybridGuidance,
    HybridSprintConfig,
    PursuitConfig,
    PursuitRendezvousGuidance,
)
from isim.engine import EngagementConfig
from isim.flight_adapter import RealFlightGuidance, standby_init_state
from isim.ownstate import OwnStateNoise, OwnStateNoiseConfig
from isim.seeker import AprilTagSeeker, CameraParams, DecodeParams, GlareParams, TagParams
from isim.target_attitude import AttitudeTarget, TargetAttitudeParams
from isim.targets import ConstantVelocityTarget, SpeedChangeTarget, WeaveTarget
from isim.types import Guidance, SeekerModel, TargetState, VehicleModel, VehicleState
from isim.vehicle import QuadVelocityModel, VehicleParams

# The nominal shared operating altitude (m). Reused from MissionConfig's own
# default rather than a second magic number -- see the module docstring.
NOMINAL_ALT_M: float = MissionConfig().standby_alt_m
_BASE_DASH_SPEED_MS: float = MissionConfig().dash_speed_ms
_BASE_DASH_ACCEL_MS2: float = MissionConfig().dash_accel_ms2

# Buffer (s) past `standby_settle_s` before the scripted GO fires -- must be
# strictly positive so the arm gate's `not_settled` check has already cleared
# (see `arm_gate`: it fails only while `obs.t - t_state < standby_settle_s`).
GO_SETTLE_BUFFER_S: float = 0.1

# Extra sim time to hold past CPA before the engine's stop rule ends the run.
STOP_AFTER_GO_BUFFER_S: float = 1.0

# Salt mixed into `[scn.seed, _SCATTER_RNG_SALT]` to build the scatter RNG's
# `SeedSequence` -- an arbitrary fixed constant, not itself a source of
# randomness, just so `Scatter`'s draws don't accidentally coincide with any
# other module that happens to seed straight off `scn.seed`.
_SCATTER_RNG_SALT: int = 7919

# The 6 VehicleParams fields Scatter.vehicle_param_sigma_frac perturbs,
# independently and multiplicatively, each clipped to this fraction so a rare
# tail draw cannot hand the model a negative/unstable gain.
_VEHICLE_SCATTER_FIELDS: Tuple[str, ...] = (
    "kp_vel_horiz", "max_accel_horiz", "max_tilt_deg",
    "thrust_max", "drag_linear_horiz", "tilt_time_constant_s",
)
_VEHICLE_SCATTER_CLIP_FRAC: float = 0.25


@dataclass
class Scatter:
    """Run-to-run scatter applied inside `build()`. Every draw comes from one
    `np.random.default_rng([scn.seed, _SCATTER_RNG_SALT])` -- never global
    state -- so a given seed always draws the same numbers and a different
    seed draws different ones (see `test_scenario.py`).

    Fields are 1-SIGMA (Gaussian) unless the name ends `_max` (uniform 0..max).
    HONESTY: every value here is an `estimate` (bench-plausible, not
    bench-measured) -- there is no logged run this project can point a
    `heading_sigma_deg` at yet; treat every headline number derived under a
    non-default `Scatter` accordingly.

    `Scenario.scatter = None` (the default) means EXACTLY today's
    behaviour, bit-identical -- pinned by
    `test_scenario.test_scatter_none_is_bit_identical_to_no_scatter`.
    """

    # --- pre-flight solve errors: these change what the SOLVE believes, not
    # (necessarily) what is true -- see `build()` for which is which ---------
    heading_sigma_deg: float = 2.0
    # deg. Added to the solved pre-flight dash heading -- compass/EKF yaw
    # error live at the GO edge. estimate.
    height_guess_sigma_m: float = 0.15
    # m. Added to MissionConfig.standby_alt_m, stacking on `height_guess_error_m`
    # -- the operator's own altimetry/height-guess error. estimate.
    target_speed_belief_sigma_frac: float = 0.10
    # frac. The pre-flight solve is handed target speed * (1 + eps); the TRUE
    # target (the trajectory it actually flies) is unchanged. estimate.

    # --- true-world scatter the pre-flight solve is NEVER told ---------------
    target_start_sigma_m: float = 0.5
    # m. Jitter on the TRUE target's start position, along its track (the
    # track runs parallel to north -- module docstring -- so this is a
    # north-axis shift only). The solve's belief start position is untouched.
    # estimate.

    # --- wind: constant per run + a per-step gust on top ----------------------
    wind_mean_max_ms: float = 3.0
    # m/s. Uniform 0..max magnitude, uniform random horizontal direction, HELD
    # CONSTANT for the whole engagement -> VehicleParams.wind_ned. estimate.
    gust_std_ms: float = 0.5
    # m/s. 1-sigma per-step-per-axis Gaussian gust -> VehicleParams.gust_std
    # (QuadVelocityModel already consumes this every step.step()). estimate.

    # --- vehicle-to-vehicle build variation -----------------------------------
    vehicle_param_sigma_frac: float = 0.08
    # frac. 1-sigma multiplicative scatter applied INDEPENDENTLY to each of
    # `_VEHICLE_SCATTER_FIELDS`, each clipped to +-25%. estimate.
    latency_extra_s_max: float = 0.04
    # s. Uniform 0..max, ADDED to VehicleParams.latency_s -- build-to-build
    # command-path latency variation on top of the fitted value. estimate.

    # --- trigger timing --------------------------------------------------------
    go_jitter_s_max: float = 0.05
    # s. Uniform 0..max added to the GO time (the trigger edge the flight code
    # sees) relative to when the target's own motion starts -- i.e. it jitters
    # the GAP between our GO edge and the target's schedule, not the target's
    # schedule itself. estimate.

    # --- camera calibration error (TRUE camera only; see `build()`) ----------
    cam_fx_fy_sigma_frac: float = 0.01
    # frac. 1-sigma SHARED scale error on fx and fy, applied ONLY to the TRUE
    # camera the seeker renders through -- the flight code keeps the nominal
    # intrinsics, so this models an uncalibrated lens, not a live read.
    # estimate.
    cam_tilt_sigma_deg: float = 1.0
    # deg. 1-sigma mount-tilt error, TRUE-camera-only, same rationale as
    # `cam_fx_fy_sigma_frac`. estimate.

    # --- v5 (pursuit_hardening_v5.md): own-state noise, GUIDANCE-visible
    # only (isim.ownstate.OwnStateNoise; concept="pursuit" only -- see
    # build()). Each is independently zeroable to isolate its cost. --------
    ownstate_attitude_bias_rp_sigma_deg: float = 1.0
    # deg. Slowly-varying (Ornstein-Uhlenbeck, tau=ownstate_attitude_bias_tau_s)
    # roll/pitch bias GUIDANCE sees on its own attitude. estimate.
    ownstate_attitude_bias_yaw_sigma_deg: float = 3.0
    # deg. Same OU process, yaw axis -- typically the noisiest axis on a
    # real EKF (no magnetometer/GPS-course fix as good as accel-derived
    # roll/pitch). estimate.
    ownstate_attitude_white_sigma_deg: float = 0.3
    # deg. White noise, per axis, per guidance tick, on top of the OU bias
    # above. estimate.
    ownstate_attitude_bias_tau_s: float = 20.0
    # s. OU correlation time shared by both attitude biases above -- long
    # relative to one engagement (25 s), so "slowly varying" in practice
    # means "close to a per-run constant, with a slow drift on top". estimate.
    ownstate_vel_bias_sigma_ms: float = 0.15
    # m/s per axis (NED). ONE draw, held constant for the whole run --
    # GUIDANCE's own-velocity bias (accelerometer/EKF bias). estimate.
    ownstate_vel_white_sigma_ms: float = 0.1
    # m/s per axis (NED), white noise per guidance tick. estimate.
    ownstate_pos_randomwalk_sigma_ms_sqrt_s: float = 0.05
    # m/sqrt(s) per axis (NED) -- a random-WALK rate (variance grows with
    # time), not a bias: GUIDANCE's own-position estimate slowly wanders
    # from truth. The task's brief claims this "should mostly cancel
    # because estimate and control share the frame" -- see the report for
    # the measured check. estimate.
    ownstate_frame_ts_bias_max_s: float = 0.020
    # s. ONE draw, uniform +/- this, held constant for the whole run --
    # GUIDANCE's belief about WHEN a frame was captured (`Detection.t_capture`)
    # is offset from the true value by a per-run camera/driver clock bias.
    # estimate.
    ownstate_frame_ts_jitter_sigma_s: float = 0.005
    # s, 1-sigma, drawn fresh for every DECODED frame -- timestamp jitter on
    # top of the per-run bias above. estimate.

    # --- v5: tag decode realism (TRUE seeker parameters, not a
    # guidance-visible term -- these widen how good/bad the REAL tag
    # decode can be, per run; guidance's OWN assumed noise model
    # (isim.concepts.PursuitConfig.sigma_px, fixed) is deliberately left
    # untouched, since a real system calibrates once and does not know a
    # given run drew a worse camera/lighting day). ---------------------------
    decode_p_max_min: float = 0.6
    # `isim.seeker.DecodeParams.p_max` (default 0.98) drawn UNIFORM
    # [this, 0.98] per run -- lighting/exposure/decoder-version variation.
    # Set to 0.98 to disable (collapses the draw to today's fixed value).
    # estimate.
    decode_pixel_noise_min_px: float = 0.3
    decode_pixel_noise_max_px: float = 1.0
    # `isim.seeker.DecodeParams.pixel_noise_px` (default 0.3) drawn UNIFORM
    # [min, max] per run. Set min=max=0.3 to disable. estimate.

    # --- v5: target motion realism (only drawn when
    # `Scenario.target_motion != "straight"`; requires `scatter is not
    # None`, since these draws come from the same per-run Scatter rng). ----
    weave_amp_min_m: float = 1.0
    weave_amp_max_m: float = 3.0
    # `isim.targets.WeaveTarget.amp_m` drawn UNIFORM [min, max] per run. estimate.
    weave_period_min_s: float = 4.0
    weave_period_max_s: float = 8.0
    # `isim.targets.WeaveTarget.period_s` drawn UNIFORM [min, max] per run. estimate.
    speed_change_delta_max_ms: float = 2.0
    # `isim.targets.SpeedChangeTarget.delta_ms` drawn UNIFORM
    # [-this, +this] per run (sign random). estimate.
    speed_change_time_max_s: float = 15.0
    # `isim.targets.SpeedChangeTarget.change_t` drawn UNIFORM [2.0, this]
    # per run (relative to the TARGET's own t=0, i.e. its GO edge). estimate.

    # --- tag_realism_v1 (isim/specs/tag_realism_v1.md): TRUE-world tag
    # realism. Every draw below is GATED on a Scenario flag (or, for
    # `own_vib_rate_rms_max_dps`, on its own nonzero value) and happens
    # after every older draw, from per-group children of the per-run rng
    # (`build()`) -- so with the flags at their defaults `Scatter()` draws
    # exactly what it drew before these fields existed, and turning a flag
    # on never reshuffles the older draws or another group's draws (paired
    # seeds stay paired across the realism ladder). Each is independently
    # zeroable. All `estimate`. -----------------------------------------------
    tgt_drag_tilt_sigma_deg: float = 3.0
    # deg, 1-sigma on `TargetAttitudeParams.drag_tilt_at_9ms_deg` (nominal
    # 12), clipped >= 0. Drawn only when `Scenario.target_attitude`. estimate.
    tgt_shake_rms_min_deg: float = 0.5
    tgt_shake_rms_max_deg: float = 2.5
    # deg. `DecodeParams.tgt_shake_rms_deg` drawn UNIFORM [min, max] per run.
    # Drawn/applied only when `Scenario.target_attitude`. Set both to 0 for
    # "attitude, no wobble". estimate.
    own_vib_rate_rms_max_dps: float = 0.0
    # deg/s. `CameraParams.vib_rate_rms_dps` drawn UNIFORM [0, this] per run
    # when > 0. DEFAULT 0.0 = OFF (spec EXPECTED tier: 40.0): unlike the
    # fields around it nothing else gates it, so an EXPECTED default would
    # silently change every existing Scatter() run. estimate.
    sun_azimuth_uniform: bool = True
    # True: sun azimuth UNIFORM [0, 360) per run and elevation per the range
    # below; False: a PINNED sun -- `Scenario.sun_azimuth_deg` /
    # `sun_elevation_deg` used as given. Only when `Scenario.glare`.
    sun_elevation_min_deg: float = 15.0
    sun_elevation_max_deg: float = 60.0
    # deg above the horizon, UNIFORM [min, max] per run (uniform-sun mode).
    # Only when `Scenario.glare`. estimate.
    specular_strength_min: float = 0.2
    specular_strength_max: float = 0.6
    # `GlareParams.specular_strength` UNIFORM [min, max] -- MATTE print
    # assumed. NOTE FOR THE BENCH: print matte; a laminated/glossy tag is the
    # 0.6/0.95 tier. Only when `Scenario.glare`. estimate.
    backlight_kill_max_deg: float = 25.0
    # deg. `GlareParams.backlight_kill_deg` UNIFORM [0, this]. Only when
    # `Scenario.glare`. estimate.


def _scatter_vehicle_params(vp: VehicleParams, scat: Scatter,
                            rng: np.random.Generator) -> VehicleParams:
    """A fresh `VehicleParams` (never mutates `vp`, which may be shared across
    an `mc.py` batch's worker processes) with `_VEHICLE_SCATTER_FIELDS`
    multiplicatively scattered, plus this run's constant wind + gust_std and
    extra command latency."""
    changes = {}
    for name in _VEHICLE_SCATTER_FIELDS:
        eps = float(np.clip(rng.normal(0.0, scat.vehicle_param_sigma_frac),
                            -_VEHICLE_SCATTER_CLIP_FRAC, _VEHICLE_SCATTER_CLIP_FRAC))
        changes[name] = getattr(vp, name) * (1.0 + eps)
    changes["latency_s"] = vp.latency_s + float(rng.uniform(0.0, scat.latency_extra_s_max))
    wind_mag = float(rng.uniform(0.0, scat.wind_mean_max_ms))
    wind_dir = float(rng.uniform(0.0, 2.0 * math.pi))
    changes["wind_ned"] = (wind_mag * math.cos(wind_dir), wind_mag * math.sin(wind_dir), 0.0)
    changes["gust_std"] = scat.gust_std_ms
    return replace(vp, **changes)


@dataclass
class Scenario:
    """One engagement's geometry + pre-flight error injection. Every field is
    a plain picklable value (no numpy arrays) so a list of `Scenario`s can be
    fanned out across `multiprocessing` workers."""

    target_speed_ms: float = 9.0
    # Crossing geometry (ignored when `head_on=True`).
    cross_range_m: float = 6.5           # perpendicular ("abeam") miss distance, m
    lead_dist_m: float = 16.2            # target starts this far before abeam, m
    direction: float = 1.0               # sign only: +1 or -1 (see module docstring)
    # Pre-flight error injection.
    target_alt_offset_m: float = 0.0     # true target alt - NOMINAL_ALT_M
    aim_error_deg: float = 0.0           # added to the solved pre-flight heading
    sprint_scale: float = 1.0            # 1.0 = full dash speed, 0.0 = none (see docstring)
    height_guess_error_m: float = 0.0    # added to the vehicle's own standby_alt_m
    head_on: bool = False                # target flies straight at the launch point
    # AprilTag mount orientation on the target: "camera" = always faces the
    # camera (today's best case, incidence pinned to 0); "rear" = tag normal
    # is minus the target's velocity direction (faces back at a chaser
    # approaching from behind -- the "pursuit" concept's own story); "side" =
    # a fixed horizontal normal perpendicular to the track, facing the launch
    # side (see `build()`). Supersedes the old `faces_camera: bool` field
    # (no other module referenced it; "camera" reproduces its True path).
    tag_facing: str = "camera"
    seed: int = 0
    # Run-to-run scatter (honest noise; see `Scatter`). None = today's exact,
    # bit-identical, deterministic-given-seed behaviour.
    scatter: Optional[Scatter] = None
    # deg, NOMINAL camera mount tilt, given to BOTH the TRUE camera (seeker)
    # and the flight code's own camera model -- a `Scatter.cam_tilt_sigma_deg`
    # error is layered on top of this for the true camera only.
    cam_tilt_up_deg: float = 0.0
    # Passed to `RealFlightGuidance(..., terminal=...)` ONLY when != "stock"
    # (see `build()`); "stock" is the only value guaranteed to work today.
    # Ignored when `concept="pursuit"`.
    terminal: str = "stock"
    # v7 LENS CORRECTION (isim/specs/hybrid_v7.md): the actual ordered part
    # is an innomaker OV9281 at 118deg HFOV, not the earlier 540/933px
    # placeholders -- fx = (width/2) / tan(HFOV/2) = (1280/2)/tan(59deg)
    # ~= 385px. This is the NOMINAL focal length from v7 on (both the TRUE
    # camera and the flight code's nominal belief); reproduced against the
    # builder's own re-measured reference numbers (rear tag, all v5 errors
    # on: 82% nominal / 59% aim20deg / 75% target+2m -- see the task's
    # final report) before anything else in this round was built.
    cam_fx_px: float = 385.0
    tag_side_m: float = 0.30             # printed tag side, m
    # "flyby" = today's open-loop-dash concept (RealFlightGuidance driving
    # the unmodified RealFlightSM), bit-identical to before this field
    # existed. "pursuit" = isim.concepts.PursuitRendezvousGuidance instead --
    # see isim/specs/pursuit_concept.md. Both start the vehicle hovering at
    # the same standby point and release GO at the same time.
    pursuit_window_s: float = 25.0       # pursuit only: engagement window, s
    concept: str = "flyby"
    # PursuitConfig.v_max_ms override, concept="pursuit" only. None = that
    # config's own default (16.0). Exists so isim.mc's generic
    # `dataclasses.replace(base, **{axis_name: value})` sweep machinery can
    # sweep pursuit's "sprint quality" axis the same way it sweeps every
    # other Scenario field (see the task's measurement script).
    pursuit_v_max_ms: Optional[float] = None
    # Generic PursuitConfig kwarg overrides, concept="pursuit" only (v4 ad
    # hoc tuning sweeps -- e.g. {"kf_q_accel_ms2": 0.5}). None/{} = no
    # override, PursuitConfig()'s own defaults. Kept SEPARATE from
    # `pursuit_v_max_ms` because that one field must stay individually
    # sweepable by isim.mc's `dataclasses.replace(base, **{axis_name:
    # value})` machinery (a dict can't be a sweep axis value there); this
    # one is for one-off tuning scripts, not the four required axes.
    pursuit_overrides: Optional[dict] = None
    # PORT-arm analogue of `pursuit_overrides` (2026-09-23, adaptive-speed
    # A/B): kwarg overrides for flight.pursuit_terminal.PursuitTerminalConfig,
    # applied ONLY on concept="flyby", terminal="pursuit" (the real-flight-
    # code chase port). None/{} = PursuitTerminalConfig()'s own defaults,
    # byte-identical to before this field existed.
    port_pursuit_overrides: Optional[dict] = None
    # v4 #1d (MEASURE-only hardware idea, never a new default): a second,
    # SMALLER AprilTag co-located with the main one (same mount/orientation,
    # different side_m), decodable at close range after the main tag has
    # left frame/blurred out. None = today's single-tag seeker, bit-
    # identical. See isim.concepts.DualTagSeeker.
    inner_tag_side_m: Optional[float] = None
    # v4 #3 (MEASURE-only hardware idea, never a new default): a SECOND tag,
    # same size as the main one, at a DIFFERENT `tag_facing` mount
    # orientation ("two-tag target"). None = today's single-tag seeker.
    # Mutually exclusive with `inner_tag_side_m` (DualTagSeeker only
    # combines two seekers, not three) -- `build()` raises if both are set.
    second_tag_facing: Optional[str] = None
    # v5 (pursuit_hardening_v5.md #6): the TRUE target's motion.  "straight"
    # (default) = today's ConstantVelocityTarget, bit-identical.  "weave" =
    # isim.targets.WeaveTarget with amp_m/period_s drawn per run from
    # Scatter's weave_* fields.  "speed_change" = isim.targets.
    # SpeedChangeTarget with delta_ms/change_t drawn per run from Scatter's
    # speed_change_* fields.  Requires `scatter is not None` for anything
    # but "straight" (the draw needs a per-run rng; `build()` raises
    # otherwise).
    target_motion: str = "straight"
    # Wind A/B (isim/specs/wind_chase_prereg_2026-09-23.md): generic
    # VehicleParams kwarg overrides, applied LAST in build() (after
    # Scatter's own vehicle-param draws, so a deliberate sweep cell -- e.g.
    # a controlled wind_ned/gust_ou_std -- wins over Scatter's random wind
    # draw). None/{} = no override, byte-identical to before this field
    # existed. This perturbs the TRUE world only; nothing guidance-visible.
    vehicle_overrides: Optional[dict] = None
    # v7 (hybrid_v7.md): "hybrid" concept only -- generic HybridSprintConfig
    # kwarg overrides, mirroring `pursuit_overrides`'s pattern exactly (kept
    # SEPARATE because that field is PursuitConfig-specific and this one is
    # HybridSprintConfig-specific; both travel through the same mc.py sweep
    # machinery via a plain dict, never a monkeypatch).
    hybrid_overrides: Optional[dict] = None
    # tag_realism_v1 A/D: the TRUE target derives a quad attitude from its
    # trajectory (isim.target_attitude.AttitudeTarget: nose-down cruise
    # pitch, banked turns) and "rear"/"side"/"rear_dual35" tags are BOLTED to
    # its body (TagParams.body_normal_frd). "camera" facing ignores attitude
    # by design (the idealized best case). With `scatter`, also draws the
    # drag-tilt and wobble terms (see Scatter). False (default) = today's
    # upright world-fixed tag, byte-identical.
    target_attitude: bool = False
    # tag_realism_v1 C: sun-geometry glare (isim.seeker.GlareParams), its
    # strengths drawn per run from Scatter -- so `glare=True` needs
    # `scatter is not None` (build() raises otherwise, same convention as
    # `target_motion`). False (default) = no glare, byte-identical.
    glare: bool = False
    # Sun direction passthrough (NED azimuth, 0 = north; elevation above the
    # horizon), used only for a pinned-sun study (glare=True with
    # Scatter.sun_azimuth_uniform=False); defaults = GlareParams' own.
    sun_azimuth_deg: float = 180.0
    sun_elevation_deg: float = 35.0
    # tag_realism_v1 follow-up (2026-09-23 ladder finding): cant the REAR
    # tag's face DOWN by this many degrees IN THE TARGET BODY frame -- the
    # mount-angle countermeasure to the pitch coupling (a nose-down cruiser
    # tilts an un-canted rear tag's face up-and-back, which the ladder
    # measured as the terminal decode loss at height-error cells). A real
    # print-time choice on OUR target's placard mount (the index disc records
    # it). Only meaningful with target_attitude=True and tag_facing="rear";
    # 0.0 (default) = today's straight-back mount, byte-identical.
    tag_mount_pitch_deg: float = 0.0


def _sign(x: float) -> float:
    return 1.0 if x >= 0.0 else -1.0


def _unit_or(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return (v / n) if n > 1e-9 else fallback


def _level_body_normal(normal_ned: np.ndarray, vel_ned: np.ndarray
                       ) -> Tuple[float, float, float]:
    """A world-frame tag normal re-expressed in the target's body FRD, for a
    LEVEL target yawed along the horizontal velocity (north when it is
    slower than AttitudeTarget's yaw threshold) -- i.e. the body mount that
    reproduces today's world-frame normal when the target flies level along
    its nominal track (tag_realism_v1 A3, "sign-matched")."""
    n = np.asarray(normal_ned, float)
    speed_h = math.hypot(float(vel_ned[0]), float(vel_ned[1]))
    yaw = math.atan2(float(vel_ned[1]), float(vel_ned[0])) if speed_h > 0.5 else 0.0
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (float(n[0] * cy + n[1] * sy), float(-n[0] * sy + n[1] * cy), float(n[2]))


def _tag_for(scn: "Scenario", vel_ned: np.ndarray) -> TagParams:
    """Build the target's true AprilTag mount from `scn.tag_facing` -- a fact
    about the simulated world (how the tag is bolted to the target drone),
    never something guidance reads. `scn.target_attitude` additionally bolts
    "rear"/"side" to the target BODY (`body_normal_frd`); the world-frame
    `normal_ned` is kept as the legacy fallback for a state with no quat."""
    tag = _tag_for_world(scn, vel_ned)
    if scn.target_attitude and not tag.faces_camera:
        if scn.tag_facing == "rear":
            # tag_mount_pitch_deg: face canted DOWN in body FRD (+z is down),
            # cancelling the cruise nose-down pitch at level view. 0 -> the
            # exact legacy (-1, 0, 0).
            mp = math.radians(scn.tag_mount_pitch_deg)
            body_n = (-math.cos(mp), 0.0, math.sin(mp))
        else:   # "side": (0, +-1, 0), sign-matched to the world-frame choice
            y = _level_body_normal(np.asarray(tag.normal_ned), vel_ned)[1]
            body_n = (0.0, 1.0 if y >= 0.0 else -1.0, 0.0)
        tag = replace(tag, body_normal_frd=body_n)
    return tag


def _tag_for_world(scn: "Scenario", vel_ned: np.ndarray) -> TagParams:
    """`_tag_for`'s legacy (world-frame) mount, unchanged."""
    if scn.tag_facing == "camera":
        return TagParams(faces_camera=True, side_m=scn.tag_side_m)
    if scn.tag_facing == "rear":
        # Faces backward relative to travel -- toward a chaser approaching
        # from behind, the "pursuit" concept's own geometry.
        rear_normal = -_unit_or(vel_ned, np.array([1.0, 0.0, 0.0]))
        return TagParams(faces_camera=False, faces_velocity=False,
                         normal_ned=tuple(float(c) for c in rear_normal),
                         side_m=scn.tag_side_m)
    if scn.tag_facing == "side":
        # A fixed horizontal normal perpendicular to the track, facing the
        # launch side. Only meaningful for crossing geometry (the module
        # docstring: cross_range_m is the launch-to-track perpendicular
        # offset along east); head-on has no such axis, so a fixed,
        # arbitrary-but-documented east-facing normal is used instead.
        if scn.head_on:
            side_normal = np.array([0.0, 1.0, 0.0])
        else:
            sign = -1.0 if scn.cross_range_m >= 0.0 else 1.0
            side_normal = np.array([0.0, sign, 0.0])
        return TagParams(faces_camera=False, faces_velocity=False,
                         normal_ned=tuple(float(c) for c in side_normal),
                         side_m=scn.tag_side_m)
    if scn.tag_facing == "rear_dual35":
        # v6 #D: not a single TagParams -- see `_rear_dual35_tags` and
        # `build()`'s special-case (a DualTagSeeker of two angled "rear"
        # tags), raised here only if someone calls `_tag_for` on it directly.
        raise ValueError("scenario._tag_for: tag_facing='rear_dual35' needs "
                         "build()'s DualTagSeeker special-case, not a single TagParams")
    raise ValueError(f"scenario.build: tag_facing={scn.tag_facing!r}, want "
                     f"'camera', 'rear', 'side', or 'rear_dual35'")


def _rear_dual35_tags(vel_ned: np.ndarray, tag_side_m: float,
                      angle_deg: float = 35.0,
                      body_mounted: bool = False) -> Tuple[TagParams, TagParams]:
    """v6 #D: two "rear"-like tags, their normals rotated +/-`angle_deg`
    from the pure rear normal (-unit(vel)) in the HORIZONTAL plane (about
    the down axis) -- "mounting two rear tags angled +/-35 deg" to relax
    the single rear tag's narrow low-incidence viewing cone. Rotation
    reuses the SAME (E,N) clockwise-azimuth formula `build()` uses for
    `aim_error_deg` (rotating only the north/east components; a "rear"
    normal's down component is always 0, per `_tag_for`)."""
    rear_normal = -_unit_or(vel_ned, np.array([1.0, 0.0, 0.0]))
    n0, e0 = float(rear_normal[0]), float(rear_normal[1])
    tags = []
    for sign in (+1.0, -1.0):
        theta = math.radians(sign * angle_deg)
        ct, st = math.cos(theta), math.sin(theta)
        e_r, n_r = e0 * ct + n0 * st, n0 * ct - e0 * st
        tag = TagParams(faces_camera=False, faces_velocity=False,
                        normal_ned=(n_r, e_r, 0.0), side_m=tag_side_m)
        if body_mounted:
            # tag_realism_v1: same two tags, bolted to the target body
            # (the level-flight body image of each world normal).
            tag = replace(tag, body_normal_frd=_level_body_normal(
                np.array([n_r, e_r, 0.0]), vel_ned))
        tags.append(tag)
    return tags[0], tags[1]


def _target_geometry(scn: Scenario) -> Tuple[np.ndarray, np.ndarray, Tuple[float, float],
                                             Tuple[float, float]]:
    """Returns (pos0_ned, vel_ned, target_start_en, target_vel_en) -- the NED
    vectors for `ConstantVelocityTarget` and the (east, north) tuples
    `resolve_preflight_heading` wants (its `target_pos`/`target_vel` args are
    documented (east, north), matching `flight.guidance.collision_lead_heading`)."""
    d = _sign(scn.direction)
    target_alt_m = NOMINAL_ALT_M + scn.target_alt_offset_m
    if scn.head_on:
        north0 = d * scn.lead_dist_m
        east0 = 0.0
        vel_north = -d * scn.target_speed_ms
        vel_east = 0.0
    else:
        east0 = scn.cross_range_m
        north0 = -d * scn.lead_dist_m
        vel_north = d * scn.target_speed_ms
        vel_east = 0.0
    pos0_ned = np.array([north0, east0, -target_alt_m], dtype=np.float64)
    vel_ned = np.array([vel_north, vel_east, 0.0], dtype=np.float64)
    return pos0_ned, vel_ned, (east0, north0), (vel_east, vel_north)


@dataclass
class _DelayedTarget:
    """TargetModel wrapper: holds `inner`'s t=0 position/zero-velocity until
    `go_at_s`, then behaves as `inner` with time shifted so
    `state(go_at_s) == inner.state(0.0)`. The GO edge is when the real vehicle
    would release the coded dash, so the target is scripted to start moving at
    the same instant -- honest for a Monte-Carlo geometry study, not a claim
    about a real target's behaviour. `inner` is any TargetModel (v5 #6 adds
    WeaveTarget/SpeedChangeTarget alongside the default ConstantVelocityTarget)."""

    inner: object   # isim.types.TargetModel (duck-typed; any of the three isim.targets classes)
    go_at_s: float

    def state(self, t: float) -> TargetState:
        if t <= self.go_at_s:
            s0 = self.inner.state(0.0)
            if s0.quat_wxyz is None:
                return TargetState(t=t, pos_ned=s0.pos_ned.copy(), vel_ned=np.zeros(3))
            # tag_realism_v1: an attitude-carrying inner (AttitudeTarget) is
            # frozen at its t=0 attitude, not rate -- the same "scripted
            # freeze" as the zero velocity (avoids a fake hover->cruise pitch
            # pulse at the GO edge). Only reachable with target_attitude=True.
            return TargetState(t=t, pos_ned=s0.pos_ned.copy(), vel_ned=np.zeros(3),
                               quat_wxyz=s0.quat_wxyz, ang_vel_body=np.zeros(3))
        return self.inner.state(t - self.go_at_s)


def build(
    scn: Scenario, vehicle_params: VehicleParams
) -> Tuple[EngagementConfig, VehicleModel, _DelayedTarget, SeekerModel, Guidance, VehicleState]:
    """Build one engagement's (cfg, vehicle, target, seeker, guidance,
    init_state) from a `Scenario`, per the module docstring's geometry.

    SCATTER (honesty note): when `scn.scatter` is not None, every draw comes
    from one `np.random.default_rng([scn.seed, _SCATTER_RNG_SALT])` created
    here and nowhere else. Two things are perturbed in a BELIEF-only way (the
    pre-flight solve sees a wrong number; the true world is unaffected) and
    one is perturbed in a TRUTH-only way (the world is wrong; the solve is
    never told) -- see `Scatter`'s docstring for which is which. `scn.scatter
    is None` reproduces EXACTLY the pre-scatter code path, bit-identical."""
    pos0_ned, vel_ned, start_en, vel_en = _target_geometry(scn)
    belief_start_en, belief_vel_en = start_en, vel_en

    scat = scn.scatter
    rng: Optional[np.random.Generator] = None
    if scat is not None:
        rng = np.random.default_rng([scn.seed, _SCATTER_RNG_SALT])

        # TRUTH-only: the true target's start position jitters along its
        # track (a north-axis shift -- see the module docstring). The solve's
        # belief start position (`belief_start_en`) is NOT touched: the
        # pre-flight solve is never told about this.
        start_jitter_m = float(rng.normal(0.0, scat.target_start_sigma_m))
        pos0_ned[0] += start_jitter_m

        # BELIEF-only: the pre-flight solve is handed target speed*(1+eps);
        # `vel_ned` (the TRUE target's velocity, used below to build its
        # actual trajectory) is left untouched.
        speed_eps = float(rng.normal(0.0, scat.target_speed_belief_sigma_frac))
        belief_vel_en = (vel_en[0] * (1.0 + speed_eps), vel_en[1] * (1.0 + speed_eps))

    dash_speed = _BASE_DASH_SPEED_MS * max(0.0, scn.sprint_scale)
    dash_accel = _BASE_DASH_ACCEL_MS2

    heading0, _t_lead = resolve_preflight_heading(
        belief_start_en, belief_vel_en, dash_speed, dash_accel_ms2=dash_accel,
        accel_aware=True, origin=(0.0, 0.0))
    heading_noise_deg = 0.0 if scat is None else float(rng.normal(0.0, scat.heading_sigma_deg))
    heading = wrap_deg(heading0 + scn.aim_error_deg + heading_noise_deg)

    height_noise_m = 0.0 if scat is None else float(rng.normal(0.0, scat.height_guess_sigma_m))
    cfg = MissionConfig(
        preflight_heading_deg=heading,
        standby_alt_m=NOMINAL_ALT_M + scn.height_guess_error_m + height_noise_m,
        dash_speed_ms=dash_speed,
        dash_accel_ms2=dash_accel,
    )

    # `go_at_s` is when the TARGET's own scripted motion starts (unjittered --
    # it runs to its own schedule). `trigger_go_at_s` is when the flight
    # code's GO trigger actually fires; `Scatter.go_jitter_s_max` jitters the
    # GAP between the two, never the target's schedule itself.
    go_at_s = cfg.standby_settle_s + GO_SETTLE_BUFFER_S
    trigger_go_at_s = go_at_s
    if scat is not None:
        trigger_go_at_s = go_at_s + float(rng.uniform(0.0, scat.go_jitter_s_max))
    stop_not_before_s = max(go_at_s, trigger_go_at_s) + STOP_AFTER_GO_BUFFER_S

    vp = vehicle_params if scat is None else _scatter_vehicle_params(vehicle_params, scat, rng)
    if scn.vehicle_overrides:
        vp = replace(vp, **scn.vehicle_overrides)
    vehicle = QuadVelocityModel(vp)

    # Camera calibration error: the TRUE camera (what the seeker renders
    # through) may differ from the NOMINAL camera the flight code believes in
    # -- an uncalibrated lens, not a live read. `cam_tilt_up_deg` is the
    # shared NOMINAL mount tilt; only the true camera also carries the
    # Scatter error terms.
    cam_true = CameraParams(fx=scn.cam_fx_px, fy=scn.cam_fx_px,
                            mount_tilt_up_deg=scn.cam_tilt_up_deg)
    cam_nominal = CameraParams(fx=scn.cam_fx_px, fy=scn.cam_fx_px,
                               mount_tilt_up_deg=scn.cam_tilt_up_deg)
    if scat is not None:
        fx_fy_eps = float(rng.normal(0.0, scat.cam_fx_fy_sigma_frac))
        tilt_err_deg = float(rng.normal(0.0, scat.cam_tilt_sigma_deg))
        cam_true = replace(cam_true,
                           fx=cam_true.fx * (1.0 + fx_fy_eps),
                           fy=cam_true.fy * (1.0 + fx_fy_eps),
                           mount_tilt_up_deg=scn.cam_tilt_up_deg + tilt_err_deg)

    # v5 #5: tag decode realism -- TRUE seeker parameters only (guidance's
    # own assumed noise model is deliberately left at its fixed, pre-
    # calibrated values; see the module's new Scatter fields' docstrings).
    dec_params = DecodeParams()
    if scat is not None:
        p_max = float(rng.uniform(scat.decode_p_max_min, 0.98))
        pixel_noise_px = float(rng.uniform(scat.decode_pixel_noise_min_px,
                                           scat.decode_pixel_noise_max_px))
        dec_params = replace(dec_params, p_max=p_max, pixel_noise_px=pixel_noise_px)

    # v4 #1d/#3 / v6 #D (MEASURE-only hardware ideas -- see
    # isim.concepts.DualTagSeeker): a second, smaller co-located tag, a
    # second tag at a different mount orientation, or (v6 #D) two "rear"
    # tags angled +/-35deg. `inner_tag_side_m`/`second_tag_facing` are
    # mutually exclusive with EACH OTHER and with `tag_facing="rear_dual35"`
    # (DualTagSeeker only ever combines two seekers). The TAG mounts are
    # chosen here (guidance below needs `tag.side_m`); the seekers are built
    # near the end of build(), after tag_realism_v1's draws (which must come
    # after every older draw -- see Scatter). Seeker construction itself
    # draws nothing, so moving it changed no rng stream (pinned by
    # test_realism_defaults_are_byte_identical_to_pre_realism_runs).
    if scn.tag_facing == "rear_dual35":
        if scn.inner_tag_side_m is not None or scn.second_tag_facing is not None:
            raise ValueError("scenario.build: tag_facing='rear_dual35' is mutually "
                             "exclusive with inner_tag_side_m/second_tag_facing")
        tag_a, tag_b = _rear_dual35_tags(vel_ned, scn.tag_side_m,
                                         body_mounted=scn.target_attitude)
        tag = tag_a   # for span_m/RealFlightGuidance below (flyby path only)
    else:
        tag = _tag_for(scn, vel_ned)
        if scn.inner_tag_side_m is not None and scn.second_tag_facing is not None:
            raise ValueError("scenario.build: inner_tag_side_m and second_tag_facing "
                             "are mutually exclusive (DualTagSeeker combines only two)")

    # Pursuit-family belief geometry (concept="pursuit"/"hybrid" Phase A/S/T,
    # AND flyby's terminal="pursuit" port below -- hoisted so both reuse ONE
    # computation): the SAME belief_start_en/belief_vel_en the heading solve
    # above used (already carries the Scatter speed-belief error, honesty-
    # gated -- see Scatter's docstring), rotated about the launch origin by
    # the SAME total angle (aim_error_deg + heading_noise_deg) the flyby
    # concept adds directly to its solved heading -- so a compass/EKF
    # or deliberate aim error distorts the belief exactly as it
    # distorts flyby's aim, for an apples-to-apples comparison across
    # concepts. Rotation (E,N) -> (E',N') by theta clockwise (azimuth
    # convention, atan2(east, north)): E'=E*cos(t)+N*sin(t),
    # N'=N*cos(t)-E*sin(t).
    theta = math.radians(scn.aim_error_deg + heading_noise_deg)
    ct, st = math.cos(theta), math.sin(theta)
    be, bn = belief_start_en
    bve, bvn = belief_vel_en
    be_r, bn_r = be * ct + bn * st, bn * ct - be * st
    bve_r, bvn_r = bve * ct + bvn * st, bvn * ct - bve * st
    belief_pos0_ned = np.array([bn_r, be_r, -NOMINAL_ALT_M], dtype=np.float64)
    belief_vel_ned = np.array([bvn_r, bve_r, 0.0], dtype=np.float64)

    if scn.concept == "flyby":
        guidance_kwargs = {} if scn.terminal == "stock" else {"terminal": scn.terminal}
        if scn.terminal == "pursuit":
            # ADR-0103 "chase only" through the REAL flight code
            # (flight.pursuit_terminal via RealFlightGuidance). Two cfg
            # changes, made ONLY on this path so every other terminal's
            # MissionConfig stays byte-identical: `pursuit_mode=True`
            # (suppresses the fly-by-tuned past-CPA recession trigger and
            # makes GO enter ENGAGE directly -- real_flight._step_standby),
            # and `engage_max_s` lengthened to the pursuit window (the
            # concept arrives slowly, 6-13 s to contact; the fly-by default
            # 12 s would truncate the tail of the chase).
            cfg = replace(cfg, pursuit_mode=True, engage_max_s=scn.pursuit_window_s)
            # Seed convention (mirrors isim/tests/test_pursuit_terminal_isim.py):
            # belief_r0_ned = belief_pos0_ned - own_pos0, where own_pos0 is the
            # scripted standby hover position `standby_init_state(cfg)` returns
            # -- a PRE-FLIGHT belief derived from the same solve inputs, never
            # a live read (its down component is the height-guess error itself:
            # believed target alt is NOMINAL, believed own alt is the
            # operator-programmed standby_alt_m).
            own_pos0 = standby_init_state(cfg).pos_ned
            guidance_kwargs["belief_r0_ned"] = tuple(
                float(c) for c in (belief_pos0_ned - own_pos0))
            guidance_kwargs["belief_vel0_ned"] = tuple(float(c) for c in belief_vel_ned)
            if scn.port_pursuit_overrides:
                from flight.pursuit_terminal import PursuitTerminalConfig
                guidance_kwargs["pursuit_cfg"] = PursuitTerminalConfig(
                    **scn.port_pursuit_overrides)
        guidance: Guidance = RealFlightGuidance(
            cfg, cam_params=cam_nominal, span_m=tag.side_m,
            go_at_s=trigger_go_at_s, home_alt_m=0.0, **guidance_kwargs)
    elif scn.concept in ("pursuit", "hybrid"):
        pcfg_kwargs = {} if scn.pursuit_v_max_ms is None else {"v_max_ms": scn.pursuit_v_max_ms}
        if scn.pursuit_overrides:
            pcfg_kwargs = {**pcfg_kwargs, **scn.pursuit_overrides}
        pcfg = PursuitConfig(fx_px=scn.cam_fx_px, tag_side_m=scn.tag_side_m, **pcfg_kwargs)

        if scn.concept == "pursuit":
            guidance = PursuitRendezvousGuidance(
                pcfg, belief_pos0_ned, belief_vel_ned, go_at_s=trigger_go_at_s,
                initial_yaw_deg=heading, cam_mount_tilt_up_deg=scn.cam_tilt_up_deg)
        else:
            # v7 hybrid: Phase S flies the SAME open-loop sprint the flyby
            # flies (same heading, dash_speed*sprint_scale, dash_accel,
            # believed altitude = cfg.standby_alt_m -- the same belief the
            # flyby concept's coded dash holds).
            scfg_kwargs = dict(scn.hybrid_overrides) if scn.hybrid_overrides else {}
            scfg = HybridSprintConfig(**scfg_kwargs)
            guidance = HybridGuidance(
                scfg, pcfg, heading_deg=heading, dash_speed_ms=dash_speed,
                dash_accel_ms2=dash_accel, believed_alt_m=cfg.standby_alt_m,
                go_at_s=trigger_go_at_s, belief_pos0_ned=belief_pos0_ned,
                belief_vel_ned=belief_vel_ned, cam_mount_tilt_up_deg=scn.cam_tilt_up_deg)

        # v5 #1-4: own-state noise, GUIDANCE-visible only -- "flyby" is
        # untouched (its own, separate Scatter story), so it stays exactly
        # bit-identical to before this field existed regardless of these
        # new fields' values. Gated on `scat is not None`, same convention
        # as every other Scatter-driven perturbation in this function --
        # `scat is None` reproduces the exact pre-v5 code path for either
        # concept. Applies to "hybrid" too (both its Phase S filter AND the
        # Phase T hand-over guidance see the SAME perturbed own-state, by
        # construction -- this wrapper is OUTSIDE HybridGuidance, so
        # whatever it hands to HybridGuidance.step() is what HybridGuidance
        # then hands onward to `self._pursuit.step()` at Phase T).
        if scat is not None:
            ownstate_cfg = OwnStateNoiseConfig(
                attitude_bias_rp_sigma_deg=scat.ownstate_attitude_bias_rp_sigma_deg,
                attitude_bias_yaw_sigma_deg=scat.ownstate_attitude_bias_yaw_sigma_deg,
                attitude_white_sigma_deg=scat.ownstate_attitude_white_sigma_deg,
                attitude_bias_tau_s=scat.ownstate_attitude_bias_tau_s,
                vel_bias_sigma_ms=scat.ownstate_vel_bias_sigma_ms,
                vel_white_sigma_ms=scat.ownstate_vel_white_sigma_ms,
                pos_randomwalk_sigma_ms_sqrt_s=scat.ownstate_pos_randomwalk_sigma_ms_sqrt_s,
                frame_ts_bias_max_s=scat.ownstate_frame_ts_bias_max_s,
                frame_ts_jitter_sigma_s=scat.ownstate_frame_ts_jitter_sigma_s,
            )
            ownstate_rng = rng.spawn(1)[0]
            guidance = OwnStateNoise(guidance, ownstate_cfg, ownstate_rng)
    else:
        raise ValueError(f"scenario.build: concept={scn.concept!r}, want "
                         f"'flyby', 'pursuit', or 'hybrid'")

    # v5 #6: TRUE target motion realism. "straight" (default) is exactly
    # today's ConstantVelocityTarget; the other two need a per-run rng draw
    # (from the SAME scatter rng, so `scatter is not None` is required).
    if scn.target_motion == "straight":
        inner_target = ConstantVelocityTarget(pos0_ned, vel_ned)
    elif scn.target_motion in ("weave", "speed_change"):
        if scat is None:
            raise ValueError(f"scenario.build: target_motion={scn.target_motion!r} "
                             "needs scatter is not None (its parameters are drawn "
                             "from the per-run Scatter rng)")
        if scn.target_motion == "weave":
            amp_m = float(rng.uniform(scat.weave_amp_min_m, scat.weave_amp_max_m))
            period_s = float(rng.uniform(scat.weave_period_min_s, scat.weave_period_max_s))
            inner_target = WeaveTarget(pos0_ned, vel_ned, amp_m=amp_m, period_s=period_s)
        else:
            delta_ms = float(rng.uniform(-scat.speed_change_delta_max_ms,
                                         scat.speed_change_delta_max_ms))
            change_t = float(rng.uniform(2.0, scat.speed_change_time_max_s))
            inner_target = SpeedChangeTarget(pos0_ned, vel_ned, delta_ms=delta_ms,
                                             change_t=change_t)
    else:
        raise ValueError(f"scenario.build: target_motion={scn.target_motion!r}, want "
                         "'straight', 'weave', or 'speed_change'")

    # tag_realism_v1: every draw below comes AFTER all the older draws, each
    # gated (flag off / knob zero -> no draw, no spawn). The three groups
    # (target attitude, own vibration, glare) each get their OWN child of the
    # per-run rng (`spawn` consumes no draws from the parent stream), so a
    # group's values do not depend on which OTHER groups are switched on --
    # the realism ladder's paired seeds stay paired rung to rung.
    if scn.glare and scat is None:
        raise ValueError("scenario.build: glare=True needs scatter is not None "
                         "(its strengths are drawn from the per-run Scatter rng)")
    att_prm = TargetAttitudeParams()
    glare_prm: Optional[GlareParams] = None
    if scat is not None and (scn.target_attitude or scn.glare
                             or scat.own_vib_rate_rms_max_dps > 0.0):
        att_rng, vib_rng, glare_rng = rng.spawn(3)
        if scn.target_attitude:
            drag_tilt = max(0.0, att_prm.drag_tilt_at_9ms_deg
                            + float(att_rng.normal(0.0, scat.tgt_drag_tilt_sigma_deg)))
            shake_rms = float(att_rng.uniform(scat.tgt_shake_rms_min_deg,
                                              scat.tgt_shake_rms_max_deg))
            att_prm = replace(att_prm, drag_tilt_at_9ms_deg=drag_tilt)
            dec_params = replace(dec_params, tgt_shake_rms_deg=shake_rms)
        if scat.own_vib_rate_rms_max_dps > 0.0:
            cam_true = replace(cam_true, vib_rate_rms_dps=float(
                vib_rng.uniform(0.0, scat.own_vib_rate_rms_max_dps)))
        if scn.glare:
            if scat.sun_azimuth_uniform:
                az = float(glare_rng.uniform(0.0, 360.0))
                el = float(glare_rng.uniform(scat.sun_elevation_min_deg,
                                             scat.sun_elevation_max_deg))
            else:                       # pinned-sun study: the Scenario's sun
                az, el = scn.sun_azimuth_deg, scn.sun_elevation_deg
            spec = float(glare_rng.uniform(scat.specular_strength_min,
                                           scat.specular_strength_max))
            kill = float(glare_rng.uniform(0.0, scat.backlight_kill_max_deg))
            glare_prm = GlareParams(sun_azimuth_deg=az, sun_elevation_deg=el,
                                    specular_strength=spec, backlight_kill_deg=kill)

    def _seeker(tp: TagParams) -> AprilTagSeeker:
        return AprilTagSeeker(cam=cam_true, tag=tp, dec=dec_params, glare=glare_prm)

    if scn.tag_facing == "rear_dual35":
        seeker: SeekerModel = DualTagSeeker(first=_seeker(tag_a), second=_seeker(tag_b))
    else:
        seeker = _seeker(tag)
        if scn.inner_tag_side_m is not None:
            seeker = DualTagSeeker(first=seeker,
                                   second=_seeker(replace(tag, side_m=scn.inner_tag_side_m)))
        elif scn.second_tag_facing is not None:
            second_scn = replace(scn, tag_facing=scn.second_tag_facing)
            seeker = DualTagSeeker(first=seeker, second=_seeker(_tag_for(second_scn, vel_ned)))

    if scn.target_attitude:
        # A2: the attitude wraps the MOTION model, inside the GO delay, so
        # `_DelayedTarget` stays the outermost object (its `.inner` is the
        # AttitudeTarget; the motion model is `.inner.inner`).
        inner_target = AttitudeTarget(inner_target, att_prm)
    target = _DelayedTarget(inner=inner_target, go_at_s=go_at_s)
    init_state = standby_init_state(cfg)

    if scn.concept in ("pursuit", "hybrid") or (
            scn.concept == "flyby" and scn.terminal == "pursuit"):
        # All three are scored on closest approach over the WHOLE window (v7:
        # "as pursuit") -- the first fly-past is not the end of the
        # engagement, the chase is. flyby+terminal="pursuit" (the flight-code
        # port of the same concept) gets the SAME window so its numbers are
        # comparable to the native prototype's, cell for cell.
        ecfg = EngagementConfig(stop_not_before_s=stop_not_before_s, seed=scn.seed,
                                max_t=scn.pursuit_window_s,
                                stop_after_cpa_s=scn.pursuit_window_s)
    else:
        ecfg = EngagementConfig(stop_not_before_s=stop_not_before_s, seed=scn.seed)

    return ecfg, vehicle, target, seeker, guidance, init_state
