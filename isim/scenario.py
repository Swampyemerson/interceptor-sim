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

from dataclasses import dataclass
from typing import Tuple

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
    init_state) from a `Scenario`, per the module docstring's geometry."""
    pos0_ned, vel_ned, start_en, vel_en = _target_geometry(scn)

    dash_speed = _BASE_DASH_SPEED_MS * max(0.0, scn.sprint_scale)
    dash_accel = _BASE_DASH_ACCEL_MS2

    heading0, _t_lead = resolve_preflight_heading(
        start_en, vel_en, dash_speed, dash_accel_ms2=dash_accel,
        accel_aware=True, origin=(0.0, 0.0))
    heading = wrap_deg(heading0 + scn.aim_error_deg)

    cfg = MissionConfig(
        preflight_heading_deg=heading,
        standby_alt_m=NOMINAL_ALT_M + scn.height_guess_error_m,
        dash_speed_ms=dash_speed,
        dash_accel_ms2=dash_accel,
    )

    go_at_s = cfg.standby_settle_s + GO_SETTLE_BUFFER_S
    stop_not_before_s = go_at_s + STOP_AFTER_GO_BUFFER_S

    cam = CameraParams()
    tag = TagParams(faces_camera=scn.faces_camera)
    seeker = AprilTagSeeker(cam=cam, tag=tag, dec=DecodeParams())

    guidance = RealFlightGuidance(cfg, cam_params=cam, span_m=tag.side_m,
                                  go_at_s=go_at_s, home_alt_m=0.0)

    target = _DelayedTarget(inner=ConstantVelocityTarget(pos0_ned, vel_ned), go_at_s=go_at_s)
    vehicle = QuadVelocityModel(vehicle_params)
    init_state = standby_init_state(cfg)

    ecfg = EngagementConfig(stop_not_before_s=stop_not_before_s, seed=scn.seed)

    return ecfg, vehicle, target, seeker, guidance, init_state
