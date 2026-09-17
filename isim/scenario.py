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

from isim.engine import EngagementConfig
from isim.flight_adapter import RealFlightGuidance, standby_init_state
from isim.seeker import AprilTagSeeker, CameraParams, DecodeParams, TagParams
from isim.targets import ConstantVelocityTarget
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
    faces_camera: bool = True            # AprilTag orientation: best-case decode
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
    terminal: str = "stock"


def _sign(x: float) -> float:
    return 1.0 if x >= 0.0 else -1.0


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
    about a real target's behaviour."""

    inner: ConstantVelocityTarget
    go_at_s: float

    def state(self, t: float) -> TargetState:
        if t <= self.go_at_s:
            s0 = self.inner.state(0.0)
            return TargetState(t=t, pos_ned=s0.pos_ned.copy(), vel_ned=np.zeros(3))
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
    vehicle = QuadVelocityModel(vp)

    # Camera calibration error: the TRUE camera (what the seeker renders
    # through) may differ from the NOMINAL camera the flight code believes in
    # -- an uncalibrated lens, not a live read. `cam_tilt_up_deg` is the
    # shared NOMINAL mount tilt; only the true camera also carries the
    # Scatter error terms.
    cam_true = CameraParams(mount_tilt_up_deg=scn.cam_tilt_up_deg)
    cam_nominal = CameraParams(mount_tilt_up_deg=scn.cam_tilt_up_deg)
    if scat is not None:
        fx_fy_eps = float(rng.normal(0.0, scat.cam_fx_fy_sigma_frac))
        tilt_err_deg = float(rng.normal(0.0, scat.cam_tilt_sigma_deg))
        cam_true = replace(cam_true,
                           fx=cam_true.fx * (1.0 + fx_fy_eps),
                           fy=cam_true.fy * (1.0 + fx_fy_eps),
                           mount_tilt_up_deg=scn.cam_tilt_up_deg + tilt_err_deg)

    tag = TagParams(faces_camera=scn.faces_camera)
    seeker = AprilTagSeeker(cam=cam_true, tag=tag, dec=DecodeParams())

    guidance_kwargs = {} if scn.terminal == "stock" else {"terminal": scn.terminal}
    guidance = RealFlightGuidance(cfg, cam_params=cam_nominal, span_m=tag.side_m,
                                  go_at_s=trigger_go_at_s, home_alt_m=0.0,
                                  **guidance_kwargs)

    target = _DelayedTarget(inner=ConstantVelocityTarget(pos0_ned, vel_ned), go_at_s=go_at_s)
    init_state = standby_init_state(cfg)

    ecfg = EngagementConfig(stop_not_before_s=stop_not_before_s, seed=scn.seed)

    return ecfg, vehicle, target, seeker, guidance, init_state
