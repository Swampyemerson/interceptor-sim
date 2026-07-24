#!/usr/bin/env python3
"""flight/deploy/real_flight.py -- the REAL interceptor's ONBOARD FLIGHT STATE MACHINE.

    *** SKELETON FOR BUILDER REVIEW -- NOT FLIGHT-READY. ***
    Every constant marked TODO-BUILDER below is a GUESS or a sim-derived number
    that must be re-measured on the physical aircraft before props ever spin.
    The SITL-testable half (state machine, guards, failsafes, handoff) is real
    and unit-tested; the hardware half (RC channel map, PWM endpoints, link
    timeout, geofence, battery budget) is deliberately unfinished.

WHAT THIS IS
------------
`flight/deploy/seeker_loop.py` implements the TERMINAL seeker only -- on the
vehicle path it deliberately does not arm, take off, or land ("props-spinning
autonomy is the dash's job"). THIS file is that missing dash: the launch/standby
logic and the coded open-loop dash that DELIVERS the airborne interceptor to the
terminal, per `docs/launch_mechanism_plan.md` Sec 5 (the plan's one real build
item, L1).

    ground takeoff  ->  STANDBY   armed hover at the loft altitude, holding the
                                  commanded aim yaw, streaming hold setpoints so
                                  PX4 stays in OFFBOARD
                          | human TRIGGER (ELRS aux channel, arm/kill class --
                          | carries NO target information; Sec 4)
                          v
                        CODED_DASH open-loop NED velocity at the pre-flight
                                  collision-lead heading, accel ramp + loft dive
                                  from flight.guidance. NO datalink, NO cue, NO gt.
                          | 5 consecutive fresh camera detections (the streak IS
                          | the handoff -- mirrors scripts/m4_intercept.py)
                          v
                        ENGAGE    seeker_loop.SeekerGuidance -- the SAME camera-only
                                  pro-nav terminal the sim validates (composed, not
                                  re-implemented)
                          | past-CPA / timeout / target-lost
                          v
                        BREAKOFF -> SAFE (hover, land requested)

Every failsafe is a GUARDED, LOGGED transition into SAFE. There is no path that
"continues blindly".

COMPOSITION (this file adds a state machine and NOTHING else)
------------------------------------------------------------
  * the terminal            -> flight.deploy.seeker_loop.SeekerGuidance (imported)
  * the dash aim            -> flight.guidance.collision_lead_heading_accel (ADR-0080)
  * the dash speed ramp     -> flight.guidance.dash_forward_speed
  * the loft-then-dive ref  -> flight.guidance.dash_loft_alt_ref
  * the dash dead-reckoned distance -> flight.guidance.dash_ramp_distance
  * the MAVSDK connect/own-state/OFFBOARD pattern -> mirrors seeker_loop.run_mavsdk
    (including the connect() timeout + _kill_server trap measured 2026-07-24)

HONESTY / NO-CHEAT AUDIT (CLAUDE.md honesty boundary; re-earned here because the
dash is a NEW command path)
--------------------------------------------------------------------------------
The state machine's ONLY inputs are:
  1. CAMERA PIXELS  -- and only in ENGAGE, through the seeker (a detection box,
     plus its implied range, which is what the handoff streak counts).
  2. OWN-STATE EKF  -- attitude quaternion, yaw, relative altitude, armed/offboard
     status, as MAVSDK reports them. Actuation (arm/takeoff/land) is not an input.
  3. PRE-FLIGHT CONSTANTS -- dash heading, standby altitude, standby yaw, dash
     speed/accel, trigger channel + thresholds. Computed or entered BEFORE launch.
  4. ONE BIT over the RF link -- the GO edge. It is arm/kill class: a single edge
     carrying zero target information, and the link may be killed immediately
     after with no effect on the intercept (constraint `no-datalink`; the
     link-denied flight arm L11 is the deliverable that PROVES it).

There is no ground truth on real hardware to read and none is read here: no
identifier in this module is a ground-truth symbol, and `--audit` proves it from
the syntax tree (not the text) along with the LATCH-ONCE property below.

LATCH-ONCE (docs/launch_mechanism_plan.md L2 / ADR-lite #5 "nothing may read a
sensor after the GO edge"): the dash heading lives in a `LatchOnce` cell that
RAISES on a second write, and `--audit` proves the source contains exactly ONE
`.set(` call site for it -- the GO edge. So "the dash cannot be re-steered
mid-flight" is machine-checked, not promised.

SIM-CLOCK vs WALL-CLOCK: there is no sim clock on the vehicle. Every duration
here is `time.monotonic()` seconds -- monotonic, NEVER `time.time()` (a GPS time
sync can step the wall clock mid-flight and would corrupt the dash ramp). The
state machine takes `t` from the caller, so tests inject a fake clock.

RUN IT
------
  # pure-logic self-test + honesty audit (no MAVSDK, no sim, no camera)
  .venv/bin/python -m flight.deploy.real_flight --self-test
  .venv/bin/python -m flight.deploy.real_flight --audit

  # offline dry-run: fake vehicle, scripted trigger, PRINTS every transition
  .venv/bin/python -m flight.deploy.real_flight --dry-run

  # against a live PX4 SITL (the head runs this; needs a booted sim)
  .venv/bin/python -m flight.deploy.real_flight --sitl-smoke \
      --mavsdk-url udpin://0.0.0.0:14540

DEPS: pure stdlib + numpy (via seeker_loop). `mavsdk` is a GUARDED import used
only by the live driver, so every offline path runs on this x86 desk.
"""
from __future__ import annotations

import argparse
import ast
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from flight.guidance import (  # noqa: E402
    collision_lead_heading,
    collision_lead_heading_accel,
    dash_forward_speed,
    dash_loft_alt_ref,
    dash_ramp_distance,
)
from flight.deploy.seeker_loop import (  # noqa: E402
    GuidanceConfig,
    OwnState,
    SeekerGuidance,
    Setpoint,
    StepTelemetry,
    _clamp,
    load_camera,
)

# ---------------------------------------------------------------- states


class State:
    """The five flight states. SAFE is ABSORBING: nothing leaves it."""

    STANDBY = "STANDBY"
    CODED_DASH = "CODED_DASH"
    ENGAGE = "ENGAGE"
    BREAKOFF = "BREAKOFF"
    SAFE = "SAFE"


ALL_STATES = (State.STANDBY, State.CODED_DASH, State.ENGAGE,
              State.BREAKOFF, State.SAFE)


# ---------------------------------------------------------------- latch-once


class LatchViolation(RuntimeError):
    """Raised when a latch-once cell is written a second time. This is the
    machine-checkable form of `docs/launch_mechanism_plan.md` ADR-lite #5:
    nothing in the launch stack may re-steer the dash after the GO edge."""


class LatchOnce:
    """A value writable EXACTLY ONCE (at the GO edge), readable forever after.

    Why a class instead of a plain attribute: a plain attribute makes
    "assigned once" a code-review promise. This makes it a runtime error AND a
    statically countable call site (`--audit` counts `.set(` in the AST)."""

    __slots__ = ("_name", "_value", "_set", "_t_set")

    def __init__(self, name: str):
        self._name = name
        self._value = None
        self._set = False
        self._t_set: Optional[float] = None

    def set(self, value, t: Optional[float] = None):
        if self._set:
            raise LatchViolation(
                f"{self._name} was already latched to {self._value!r} at "
                f"t={self._t_set}; a second write ({value!r}) is a honesty-boundary "
                "violation (no post-GO re-steer)")
        self._value = value
        self._set = True
        self._t_set = t
        return value

    @property
    def value(self):
        return self._value

    @property
    def is_set(self) -> bool:
        return self._set

    @property
    def t_set(self) -> Optional[float]:
        return self._t_set

    def __repr__(self):
        return (f"LatchOnce({self._name}={self._value!r}, "
                f"set={self._set}, t={self._t_set})")


# ---------------------------------------------------------------- config


@dataclass
class MissionConfig:
    """Every constant the state machine needs. Grouped by honesty class and
    tagged TODO-BUILDER where the number is NOT yet measured on the aircraft.

    THE THREE PRE-FLIGHT AIM CONSTANTS (docs/launch_mechanism_plan.md Sec 2):
      C1 dash heading   -> `preflight_heading_deg` (or the optional bearing latch)
      C2 lead correction-> `latch_lead_correction_deg` (bearing-latch path only;
                           the accel-aware lead DERIVES it on the solve path)
      C3 standby alt    -> `standby_alt_m` = target altitude + loft
    """

    # --- pre-flight aim (honesty class: pre-flight constants) ------------------
    preflight_heading_deg: float = 0.0     # C1, NED compass azimuth (0=N, 90=E)
    standby_alt_m: float = 7.0             # C3 = target alt + loft. TODO-BUILDER
    standby_yaw_deg: Optional[float] = None  # None -> = preflight_heading_deg
    # OPTIONAL trigger-instant camera bearing latch (Sec 2 C1-primary). DEFAULT
    # OFF: it needs the OV9281 intrinsics (bench L4) and a measured 1-sigma
    # bearing error (bench L5) before it may drive a ram. TODO-BUILDER.
    bearing_latch_enabled: bool = False
    latch_lead_correction_deg: float = 0.0  # C2, signed on the known crossing dir

    # --- coded dash ------------------------------------------------------------
    dash_speed_ms: float = 16.0            # TODO-BUILDER: measure on the airframe
    # The acceleration the dash ACTUALLY flies. Feeds BOTH the accel-aware lead
    # solve and the dead-reckoned distance bound. Sim-fitted; the FIRST real dash
    # ULog replaces it (launch_mechanism_plan L9). TODO-BUILDER.
    dash_accel_ms2: float = 10.0
    # Commanded ramp cap (`--dash-accel-cap` in sim). None = uncapped. Measured to
    # cost ~0.34 m of CPA even at correct aim (project_state coded_dash 2026-07-24)
    # -- but Sec 3.3 recommends the SLOWER, aim-forgiving profile for FIRST kills.
    dash_accel_cap_ms2: Optional[float] = None
    dash_loft_m: float = 0.0               # loft height above the target's altitude
    dash_loft_dive_s: float = 2.0
    dash_max_s: float = 8.0                # hard dash bound (sim --coded-dash-max-s)
    dash_max_dist_m: Optional[float] = None  # optional dead-reckoned distance bound

    # --- handoff ---------------------------------------------------------------
    acquire_streak: int = 5                # mirrors m4 CODED_DASH_ACQUIRE_STREAK
    # The pre-flight range-plausibility window is RETIRED (ADR-0077 NULL, and the
    # nose-cantilever mount designs the phantom out of FoV). Kept as inert knobs
    # so the contract matches m4's; leave them None.
    acquire_range_min_m: Optional[float] = None
    acquire_range_max_m: Optional[float] = None

    # --- terminal / breakoff (all TODO-BUILDER: sim-derived) -------------------
    engage_max_s: float = 12.0
    engage_lost_target_s: float = 2.0      # no fresh detection this long -> BREAKOFF
    breakoff_arm_range_m: float = 4.0      # arms past-CPA logic (m4 FPV profile)
    breakoff_range_increases: int = 3
    breakoff_range_deadband_m: float = 0.05
    breakoff_hard_floor_m: float = 0.5
    breakoff_s: float = 1.5
    breakoff_climb_ms: float = 1.0

    # --- safety / failsafe (ALL TODO-BUILDER) ----------------------------------
    standby_settle_s: float = 2.0          # min hold before a GO may be accepted
    standby_max_s: float = 60.0            # battery budget, Sec 1 "cost"
    mission_max_s: float = 120.0           # absolute backstop, any state
    link_timeout_s: float = 1.0            # RC frame staleness -> link DENIED
    link_acquire_grace_s: float = 8.0      # allow the RC stream to start up
    alt_tol_m: float = 0.3                 # arm gate, Sec 4
    yaw_tol_deg: float = 2.0               # arm gate, Sec 4
    require_armed: bool = True
    require_offboard: bool = True
    require_low_before_go: bool = True     # a switch HIGH at boot is NEVER a GO
    safe_hold_s: float = 2.0               # hold in SAFE before the driver lands

    # --- vertical / hold control ----------------------------------------------
    kp_alt: float = 1.0
    v_vert_max_ms: float = 2.0

    @property
    def aim_yaw_deg(self) -> float:
        """The yaw STANDBY holds. Defaults to the pre-flight dash heading, so the
        aircraft is already pointed down the dash line before the trigger."""
        return (self.preflight_heading_deg if self.standby_yaw_deg is None
                else self.standby_yaw_deg)

    @property
    def dash_base_alt_m(self) -> float:
        """Altitude the loft dive ENDS at = the target's altitude (Sec 1 reason 3:
        standby altitude = target altitude + loft, so the dash begins lofted)."""
        return self.standby_alt_m - self.dash_loft_m


# ---------------------------------------------------------------- observations


@dataclass
class TriggerState:
    """What the RF link says THIS tick. Arm/kill class only -- there is no field
    here that could carry target information, by construction."""

    go: bool = False                    # switch currently commanding GO
    link_ok: bool = False               # a fresh RC frame arrived recently
    age_s: Optional[float] = None       # seconds since the last RC frame
    raw_us: Optional[int] = None        # last PWM (logging only)


@dataclass
class VehicleObs:
    """Everything the state machine may read on one tick. Own-state EKF + own
    status + the camera + the one trigger bit. No ground truth, by construction."""

    t: float                                    # monotonic seconds
    armed: Optional[bool] = None
    offboard_active: Optional[bool] = None
    alt_m: Optional[float] = None               # relative altitude (own EKF)
    yaw_deg: Optional[float] = None             # own EKF yaw
    quat: Optional[Tuple[float, float, float, float]] = None  # (w,x,y,z) body->NED
    trigger: TriggerState = field(default_factory=TriggerState)
    # Camera, exactly as the detector reports it (finetuned_seeker.SeekerDetection)
    det_new: bool = False                       # a NEW detector result this tick
    det_box_xywh: Optional[Sequence[float]] = None
    det_range_m: Optional[float] = None
    # Bearing for the OPTIONAL pre-launch bearing latch (degrees, camera-relative).
    # Read ONLY at the GO edge and only when cfg.bearing_latch_enabled.
    det_bearing_deg: Optional[float] = None

    def own_state(self) -> OwnState:
        return OwnState(quat=self.quat,
                        psi_rad=(None if self.yaw_deg is None
                                 else math.radians(self.yaw_deg)),
                        alt_m=self.alt_m)


@dataclass
class Transition:
    t: float
    frm: str
    to: str
    reason: str


@dataclass
class Decision:
    """One tick's output. `setpoint` is ALWAYS present: PX4 drops OFFBOARD if the
    setpoint stream stops for ~0.5 s, so every state -- including SAFE -- must
    emit one."""

    state: str
    setpoint: Setpoint
    events: List[str] = field(default_factory=list)
    transition: Optional[Transition] = None
    streak: int = 0
    land_requested: bool = False
    terminated: bool = False
    safe_reason: Optional[str] = None
    telemetry: Optional[StepTelemetry] = None


# ---------------------------------------------------------------- pure helpers


def wrap_deg(a: float) -> float:
    """Wrap an angle (degrees) into [-180, 180) -- the degree twin of
    flight.geometry.wrap_pi, same formula, so the two agree at every boundary
    (note both map exactly +180 to -180)."""
    return (float(a) + 180.0) % 360.0 - 180.0


def update_acquire_streak(streak: int, new_meas: bool,
                          meas_range_m: Optional[float],
                          acquire_range_min: Optional[float] = None,
                          acquire_range_max: Optional[float] = None) -> int:
    """The HANDOFF streak, contract-identical to
    `scripts/m4_intercept.update_coded_dash_streak` (the flown, sim-validated
    logic; a parity test pins the two to agree case-for-case):

      * no NEW detector result this tick -> HOLD (the control loop outpaces the
        detector, so "nothing new" is normal cadence, not a miss),
      * NEW result with a range inside the (optional) window -> streak + 1,
      * NEW result with no target, or out of window -> RESET to 0.

    KNOWN RESIDUAL (inherited, ADR-0076 add #18g): a self-consistent own-prop
    PHANTOM advances this streak exactly like a real target. That hazard is
    designed out in HARDWARE (nose-cantilever mount puts the prop out of FoV,
    constraint `prop-clearance`), NOT here."""
    if not new_meas:
        return streak
    range_ok = meas_range_m is not None and (
        acquire_range_min is None or meas_range_m >= acquire_range_min) and (
        acquire_range_max is None or meas_range_m <= acquire_range_max)
    return streak + 1 if range_ok else 0


def update_recede_streak(streak: int, last_range: Optional[float],
                         meas_range: float, deadband: float):
    """Past-CPA range-INCREASE streak with a noise deadband. Mirrors
    `scripts/m4_intercept.update_range_increase_streak` minus its
    streak_start_range magnitude check (TODO-BUILDER: port that check before a
    real terminal flies -- it is what stops a slow drift from firing breakoff).

    Returns (streak, last_range)."""
    if last_range is None:
        return 0, meas_range
    if meas_range > last_range + deadband:
        return streak + 1, meas_range
    if meas_range > last_range:
        return streak, last_range        # within-deadband creep: hold
    return 0, meas_range


def resolve_preflight_heading(target_start, target_vel, dash_speed,
                              dash_accel_ms2=None, accel_aware=True,
                              origin=(0.0, 0.0)):
    """C1 FALLBACK / solve path: the pre-flight collision-lead azimuth.

    `accel_aware` (default, ADR-0080) solves the triangle against the dash's
    ACTUAL acceleration ramp instead of a constant speed -- the interceptor
    starts at rest, so the constant-speed solve systematically UNDER-LEADS, and
    the old hand-tuned crossing bias was really an acceleration constant in
    disguise. Sim-validated on disjoint seeds (project_state launch_aim).

    Honesty: both inputs are pre-flight -- the operator-entered AUTO-leg geometry
    (we program that leg) and one own-vehicle spec (the dash accel). No live read."""
    if accel_aware and dash_accel_ms2:
        return collision_lead_heading_accel(target_start, target_vel, dash_speed,
                                            dash_accel_ms2, origin=origin)
    return collision_lead_heading(target_start, target_vel, dash_speed,
                                  origin=origin)


def latch_heading_from_bearing(bearing_deg: float, own_yaw_deg: float,
                               lead_correction_deg: float = 0.0) -> float:
    """C1 PRIMARY (docs/launch_mechanism_plan.md Sec 2): turn the trigger-instant
    camera bearing into the dash heading.

        heading = own EKF yaw + camera bearing + C2 lead correction

    Why it beats a pre-computed heading (Sec 3.2): at ~16.7 m and 9 m/s the LOS
    sweeps ~31 deg/s, so 100 ms of human trigger jitter is ~3 deg of aim error --
    the WHOLE 0.5 m ram budget. Computing at the trigger instant converts that
    timing error into a benign range error. It also cancels compass/EKF yaw error
    to first order, because the same yaw estimate computes AND flies the aim.

    TODO-BUILDER: this is only trustworthy once (a) the OV9281 intrinsics are
    calibrated (bench L4 -- the ~118 deg M12 barrel-distorts >1 deg at the edge,
    so the caller MUST pass an UNDISTORTED bearing from
    flight.camera.CameraModel.pixel_to_bearing) and (b) the latched-bearing
    1-sigma error is measured against tape truth (bench L5, needs << 3 deg).
    Default OFF until both land."""
    return wrap_deg(float(own_yaw_deg) + float(bearing_deg)
                    + float(lead_correction_deg))


# ---------------------------------------------------------------- the machine


class RealFlightSM:
    """The onboard flight state machine, as a PURE step function.

    No MAVSDK, no camera, no clock of its own: `step(obs)` takes one
    observation (which carries `t`) and returns one `Decision`. That is what
    makes the whole thing unit-testable off-hardware -- inject a fake vehicle,
    a fake clock and a fake RC source and the transitions are deterministic.

    `guidance` is the seeker_loop terminal (SeekerGuidance). It may be None, in
    which case ENGAGE flies the open-loop coast fallback -- used by pure-logic
    tests that do not want a camera model."""

    def __init__(self, cfg: MissionConfig,
                 guidance: Optional[SeekerGuidance] = None,
                 on_event: Optional[Callable[[str], None]] = None):
        self.cfg = cfg
        self.guidance = guidance
        self._on_event = on_event
        self.state = State.STANDBY
        # THE latch. Written at exactly one call site (the GO edge); `--audit`
        # proves that statically and LatchOnce enforces it at runtime.
        self.dash_heading = LatchOnce("dash_heading_deg")
        self.transitions: List[Transition] = []
        self.safe_reason: Optional[str] = None
        self.streak = 0
        # timing (all monotonic seconds supplied by the caller)
        self.t0: Optional[float] = None
        self.t_state: Optional[float] = None
        self.t_dash_start: Optional[float] = None
        self.t_engage_start: Optional[float] = None
        # trigger edge tracking
        self._trig_prev: Optional[bool] = None
        self._trig_seen_low = not cfg.require_low_before_go
        self._logged_high_at_boot = False
        self._link_seen = False
        # gate bookkeeping
        self._gate_ok_since: Optional[float] = None
        # terminal bookkeeping
        self._last_dash_sp: Optional[Setpoint] = None
        self._last_yaw_cmd_deg: float = cfg.aim_yaw_deg
        self._last_det_t: Optional[float] = None
        self._breakoff_armed = False
        self._recede_streak = 0
        self._recede_last_range: Optional[float] = None

    # ---------------------------------------------------------------- logging

    def _emit(self, msg: str, events: List[str]):
        events.append(msg)
        if self._on_event is not None:
            self._on_event(msg)

    def _transition(self, to: str, reason: str, obs: VehicleObs,
                    events: List[str]) -> Transition:
        tr = Transition(t=obs.t, frm=self.state, to=to, reason=reason)
        self.transitions.append(tr)
        self._emit(f"[{obs.t:7.2f}s] {tr.frm} -> {to}  ({reason})", events)
        self.state = to
        self.t_state = obs.t
        return tr

    # ---------------------------------------------------------------- guards

    def link_status(self, obs: VehicleObs) -> Tuple[bool, str]:
        """LINK-DENIED failsafe input. Returns (ok, reason).

        POLICY (a real design decision, see the ADR-lite in the report): the RF
        link is monitored ONLY in STANDBY. After the GO edge, link loss is
        EXPECTED and IGNORED -- that is the entire jam-resistance claim
        (constraint `no-datalink`, flight arm L11: kill the TX right after the
        GO edge and the intercept must complete unaided). A failsafe that
        aborted the dash on link loss would DELETE the headline capability."""
        trig = obs.trigger
        if trig.link_ok:
            return True, "ok"
        if not self._link_seen:
            elapsed = 0.0 if self.t_state is None else obs.t - self.t_state
            if elapsed < self.cfg.link_acquire_grace_s:
                return True, "grace"       # RC stream has not started yet
            return False, "link_never_acquired"
        return False, "link_lost"

    def arm_gate(self, obs: VehicleObs) -> Tuple[bool, List[str]]:
        """The Sec 4 arm gate: the Pi REFUSES to dash unless the aircraft is in a
        fit state. Pure + public so the driver (and the tests) can ask."""
        cfg = self.cfg
        fails: List[str] = []
        if cfg.require_armed and obs.armed is not True:
            fails.append(f"not_armed(armed={obs.armed})")
        if cfg.require_offboard and obs.offboard_active is not True:
            fails.append(f"not_offboard(offboard={obs.offboard_active})")
        if obs.alt_m is None:
            fails.append("no_altitude")
        elif abs(obs.alt_m - cfg.standby_alt_m) > cfg.alt_tol_m:
            fails.append(f"alt_off({obs.alt_m:.2f} vs {cfg.standby_alt_m:.2f} "
                         f"+-{cfg.alt_tol_m})")
        if obs.yaw_deg is None:
            fails.append("no_yaw")
        else:
            err = abs(wrap_deg(obs.yaw_deg - cfg.aim_yaw_deg))
            if err > cfg.yaw_tol_deg:
                fails.append(f"yaw_off({err:.1f} deg > {cfg.yaw_tol_deg})")
        if self.t_state is not None and (obs.t - self.t_state) < cfg.standby_settle_s:
            fails.append("not_settled")
        return (not fails), fails

    def _go_edge(self, obs: VehicleObs, events: List[str]) -> bool:
        """RISING-edge detection on the trigger, with two safety rules:
          * a GO is only honoured while the link is alive (a stale link cannot
            fire the dash), and
          * a switch found ALREADY HIGH at boot is never a GO -- it must be seen
            LOW first. (Otherwise powering up with the switch left on dashes the
            moment the state machine starts.)"""
        trig = obs.trigger
        raw = bool(trig.go) and bool(trig.link_ok)
        prev = self._trig_prev
        self._trig_prev = raw
        if not raw:
            self._trig_seen_low = True
            return False
        if not self._trig_seen_low:
            if not self._logged_high_at_boot:
                self._logged_high_at_boot = True
                self._emit("[trigger] REFUSED: aux channel was already HIGH at "
                           "startup -- flip it LOW then HIGH to arm the GO edge",
                           events)
            return False
        return prev is False

    # ---------------------------------------------------------------- setpoints

    def _v_down(self, obs: VehicleObs, alt_ref: float) -> float:
        if obs.alt_m is None:
            return 0.0
        return _clamp(self.cfg.kp_alt * (obs.alt_m - alt_ref),
                      -self.cfg.v_vert_max_ms, self.cfg.v_vert_max_ms)

    def _standby_setpoint(self, obs: VehicleObs) -> Setpoint:
        """Hold: zero horizontal velocity, altitude-hold at the standby (lofted)
        altitude, absolute yaw at the aim heading. Streamed >= 2 Hz so PX4 stays
        in OFFBOARD -- the standby hover MUST be a Pi setpoint stream, not a PX4
        LOITER the Pi later takes over (plan Sec 1)."""
        self._last_yaw_cmd_deg = self.cfg.aim_yaw_deg
        return Setpoint(0.0, 0.0, self._v_down(obs, self.cfg.standby_alt_m),
                        self.cfg.aim_yaw_deg)

    def _dash_setpoint(self, obs: VehicleObs) -> Setpoint:
        """The open-loop dash command -- structurally identical to the flown sim
        CODED_DASH block (scripts/m4_intercept.py), calling the SAME portable
        flight.guidance functions:

            v(t)     = dash_forward_speed(speed, accel_cap, t_since_dash)
            alt_ref  = dash_loft_alt_ref(base_alt, loft, t_since_dash, dive_s)
            v_N, v_E = v*cos(heading), v*sin(heading)     (compass azimuth)
            yaw      = heading   (nose on the dash line, so the camera looks
                                  where the aircraft is going)

        Own-state only: altitude for the dive reference. No camera, no cue, no gt."""
        cfg = self.cfg
        elapsed = 0.0 if self.t_dash_start is None else obs.t - self.t_dash_start
        v = dash_forward_speed(cfg.dash_speed_ms, cfg.dash_accel_cap_ms2, elapsed)
        alt_ref = dash_loft_alt_ref(cfg.dash_base_alt_m, cfg.dash_loft_m,
                                    elapsed, cfg.dash_loft_dive_s)
        h_deg = self.dash_heading.value
        h = math.radians(h_deg)
        self._last_yaw_cmd_deg = h_deg
        sp = Setpoint(v * math.cos(h), v * math.sin(h),
                      self._v_down(obs, alt_ref), h_deg)
        self._last_dash_sp = sp
        return sp

    def _coast_setpoint(self, obs: VehicleObs) -> Setpoint:
        """ENGAGE fallback while the seeker has no lock yet: keep flying the last
        dash velocity (open-loop coast) with altitude hold at the target's
        altitude. Rationale: the handoff only fires on 5 fresh detections, so the
        tracker locks within a tick or two; hovering instead would throw away the
        closing speed the dash just built. It also guarantees the OFFBOARD
        setpoint stream never gaps."""
        if self._last_dash_sp is None:
            return Setpoint(0.0, 0.0, self._v_down(obs, self.cfg.standby_alt_m),
                            self._last_yaw_cmd_deg)
        d = self._last_dash_sp
        return Setpoint(d.v_north, d.v_east,
                        self._v_down(obs, self.cfg.dash_base_alt_m),
                        self._last_yaw_cmd_deg)

    # ---------------------------------------------------------------- step

    def step(self, obs: VehicleObs) -> Decision:
        events: List[str] = []
        if self.t0 is None:
            self.t0 = obs.t
            self.t_state = obs.t
            self._emit(f"[{obs.t:7.2f}s] START in {self.state} "
                       f"(aim yaw {self.cfg.aim_yaw_deg:.1f} deg, standby alt "
                       f"{self.cfg.standby_alt_m:.1f} m)", events)
        before = self.state
        transition_before = len(self.transitions)

        # ABSOLUTE BACKSTOP: no state except SAFE may outlive the mission bound.
        if self.state != State.SAFE and (obs.t - self.t0) > self.cfg.mission_max_s:
            self._transition(State.SAFE,
                             f"mission_timeout({self.cfg.mission_max_s:.0f}s)",
                             obs, events)
            self.safe_reason = "mission_timeout"

        if self.state == State.STANDBY:
            sp = self._step_standby(obs, events)
        elif self.state == State.CODED_DASH:
            sp = self._step_coded_dash(obs, events)
        elif self.state == State.ENGAGE:
            sp, tel = self._step_engage(obs, events)
            return self._decide(before, transition_before, sp, events, tel)
        elif self.state == State.BREAKOFF:
            sp = self._step_breakoff(obs, events)
        else:
            sp = self._step_safe(obs, events)
        return self._decide(before, transition_before, sp, events, None)

    def _decide(self, before, transition_before, sp, events, tel) -> Decision:
        tr = (self.transitions[-1]
              if len(self.transitions) > transition_before else None)
        terminated = False
        if self.state == State.SAFE and self.t_state is not None:
            terminated = (self._last_obs_t - self.t_state) >= self.cfg.safe_hold_s
        return Decision(state=self.state, setpoint=sp, events=events,
                        transition=tr, streak=self.streak,
                        land_requested=(self.state == State.SAFE),
                        terminated=terminated, safe_reason=self.safe_reason,
                        telemetry=tel)

    # -- STANDBY ---------------------------------------------------------------

    def _step_standby(self, obs: VehicleObs, events: List[str]) -> Setpoint:
        cfg = self.cfg
        self._last_obs_t = obs.t
        if obs.trigger.link_ok:
            self._link_seen = True

        # Edge detector runs EVERY tick so it sees the lows (and so a link drop
        # forces a fresh rising edge after recovery).
        go = self._go_edge(obs, events)

        # FAILSAFE 1 -- LINK DENIED in STANDBY: do NOT dash. (Post-GO link loss is
        # ignored on purpose; see link_status().)
        ok, why = self.link_status(obs)
        if not ok:
            self._transition(State.SAFE, f"link_denied({why})", obs, events)
            self.safe_reason = f"link_denied:{why}"
            return self._safe_setpoint(obs)

        # FAILSAFE 2 -- standby battery/time budget.
        if (obs.t - self.t_state) > cfg.standby_max_s:
            self._transition(State.SAFE,
                             f"standby_timeout({cfg.standby_max_s:.0f}s)",
                             obs, events)
            self.safe_reason = "standby_timeout"
            return self._safe_setpoint(obs)

        gate_ok, fails = self.arm_gate(obs)
        self._gate_ok_since = (obs.t if (gate_ok and self._gate_ok_since is None)
                               else (self._gate_ok_since if gate_ok else None))

        if go:
            if not gate_ok:
                # Sec 4: a GO with a failed arm gate ABORTS -- it does not dash and
                # it does not silently ignore the operator. Loud, logged, SAFE.
                self._transition(State.SAFE,
                                 "arm_gate_failed_at_go[" + ",".join(fails) + "]",
                                 obs, events)
                self.safe_reason = "arm_gate_failed_at_go"
                return self._safe_setpoint(obs)
            # ---- THE GO EDGE: latch C1 here and NOWHERE ELSE. -----------------
            heading = cfg.preflight_heading_deg
            src = "preflight lead solve"
            if cfg.bearing_latch_enabled and obs.det_bearing_deg is not None \
                    and obs.yaw_deg is not None:
                heading = latch_heading_from_bearing(
                    obs.det_bearing_deg, obs.yaw_deg, cfg.latch_lead_correction_deg)
                src = "trigger-instant camera bearing latch"
            self.dash_heading.set(wrap_deg(heading), t=obs.t)
            self._emit(f"[{obs.t:7.2f}s] GO edge: dash heading LATCHED "
                       f"{self.dash_heading.value:+.2f} deg ({src}) -- this value "
                       "is now immutable for the rest of the flight", events)
            self.t_dash_start = obs.t
            self._transition(State.CODED_DASH, "go_edge", obs, events)
            return self._dash_setpoint(obs)

        return self._standby_setpoint(obs)

    # -- CODED_DASH ------------------------------------------------------------

    def _step_coded_dash(self, obs: VehicleObs, events: List[str]) -> Setpoint:
        cfg = self.cfg
        self._last_obs_t = obs.t
        # The command for THIS tick is computed BEFORE the transition checks, so
        # the handoff tick still flies the dash and ENGAGE takes over next tick --
        # byte-for-byte the sim's ordering (m4_intercept.py CODED_DASH).
        sp = self._dash_setpoint(obs)
        elapsed = obs.t - self.t_dash_start

        self.streak = update_acquire_streak(
            self.streak, obs.det_new, obs.det_range_m,
            cfg.acquire_range_min_m, cfg.acquire_range_max_m)
        if obs.det_new and obs.det_range_m is not None:
            self._last_det_t = obs.t

        if self.streak >= cfg.acquire_streak:
            self._emit(f"[{obs.t:7.2f}s] camera ACQUIRED at dash t={elapsed:.2f}s "
                       f"({self.streak} consecutive fresh detections)", events)
            self.t_engage_start = obs.t
            self._transition(State.ENGAGE, f"acquire_streak>={cfg.acquire_streak}",
                             obs, events)
            return sp

        # FAILSAFE 3 -- dash time bound with NO acquire: SAFE, never a blind
        # continue. (A blind continue is a spinning-prop aircraft flying open-loop
        # past its own intercept point.)
        if elapsed > cfg.dash_max_s:
            self._transition(State.SAFE,
                             f"dash_timeout_no_acquire({cfg.dash_max_s:.1f}s)",
                             obs, events)
            self.safe_reason = "dash_timeout_no_acquire"
            return self._safe_setpoint(obs)

        # FAILSAFE 4 -- optional dead-reckoned DISTANCE bound (own dash model, no
        # position read needed). Off by default; size it to the range box.
        if cfg.dash_max_dist_m is not None:
            flown = dash_ramp_distance(cfg.dash_speed_ms,
                                       cfg.dash_accel_cap_ms2 or cfg.dash_accel_ms2,
                                       elapsed)
            if flown > cfg.dash_max_dist_m:
                self._transition(State.SAFE,
                                 f"dash_distance_bound({flown:.1f} m > "
                                 f"{cfg.dash_max_dist_m:.1f} m)", obs, events)
                self.safe_reason = "dash_distance_bound"
                return self._safe_setpoint(obs)
        return sp

    # -- ENGAGE ----------------------------------------------------------------

    def _step_engage(self, obs: VehicleObs, events: List[str]):
        cfg = self.cfg
        self._last_obs_t = obs.t
        elapsed = obs.t - self.t_engage_start
        tel = None
        sp = None

        if self.guidance is not None:
            box = obs.det_box_xywh if obs.det_range_m is not None else None
            sp, tel = self.guidance.step(box, obs.own_state(), obs.t)
        if sp is not None:
            self._last_yaw_cmd_deg = sp.yaw_deg

        if obs.det_new and obs.det_range_m is not None:
            self._last_det_t = obs.t
            r = obs.det_range_m
            if r < cfg.breakoff_arm_range_m:
                self._breakoff_armed = True
            self._recede_streak, self._recede_last_range = update_recede_streak(
                self._recede_streak, self._recede_last_range, r,
                cfg.breakoff_range_deadband_m)
            if r <= cfg.breakoff_hard_floor_m:
                self._transition(State.BREAKOFF, f"hard_floor({r:.2f} m)",
                                 obs, events)
                return self._breakoff_setpoint(obs), tel
            if self._breakoff_armed and \
                    self._recede_streak >= cfg.breakoff_range_increases:
                self._transition(State.BREAKOFF,
                                 f"past_cpa({self._recede_streak} rising)",
                                 obs, events)
                return self._breakoff_setpoint(obs), tel

        # FAILSAFE 5 -- the terminal lost the target and did not recover.
        if self._last_det_t is not None and \
                (obs.t - self._last_det_t) > cfg.engage_lost_target_s:
            self._transition(State.BREAKOFF,
                             f"target_lost({cfg.engage_lost_target_s:.1f}s)",
                             obs, events)
            return self._breakoff_setpoint(obs), tel

        # FAILSAFE 6 -- terminal time bound.
        if elapsed > cfg.engage_max_s:
            self._transition(State.BREAKOFF,
                             f"engage_timeout({cfg.engage_max_s:.1f}s)",
                             obs, events)
            return self._breakoff_setpoint(obs), tel

        return (sp if sp is not None else self._coast_setpoint(obs)), tel

    # -- BREAKOFF / SAFE -------------------------------------------------------

    def _breakoff_setpoint(self, obs: VehicleObs) -> Setpoint:
        return Setpoint(0.0, 0.0, -self.cfg.breakoff_climb_ms,
                        self._last_yaw_cmd_deg)

    def _step_breakoff(self, obs: VehicleObs, events: List[str]) -> Setpoint:
        self._last_obs_t = obs.t
        if (obs.t - self.t_state) > self.cfg.breakoff_s:
            self._transition(State.SAFE, "breakoff_complete", obs, events)
            self.safe_reason = self.safe_reason or "breakoff_complete"
            return self._safe_setpoint(obs)
        return self._breakoff_setpoint(obs)

    def _safe_setpoint(self, obs: VehicleObs) -> Setpoint:
        """SAFE: stop. Zero horizontal velocity, no commanded climb/descent, hold
        the last yaw. `land_requested` tells the driver to hand back to PX4 LAND
        (the driver owns actuation; the state machine never lands by itself)."""
        return Setpoint(0.0, 0.0, 0.0, self._last_yaw_cmd_deg)

    def _step_safe(self, obs: VehicleObs, events: List[str]) -> Setpoint:
        self._last_obs_t = obs.t
        return self._safe_setpoint(obs)

    # convenience for the drivers/tests
    _last_obs_t: float = 0.0

    @property
    def visited(self) -> List[str]:
        seen = [State.STANDBY]
        for tr in self.transitions:
            seen.append(tr.to)
        return seen


# ---------------------------------------------------------------- trigger sources


class PwmSwitch:
    """Two-threshold (hysteresis) reading of one RC channel's PWM.

    TODO-BUILDER (hardware-specific, bench L3/L6 before any props-on test):
      * `high_us` / `low_us` must be measured against the ACTUAL RadioMaster
        Pocket M2 + RP3 ELRS endpoints (they are NOT guaranteed 1000/2000), with
        the switch's centre position landing firmly in the dead band.
      * confirm the channel index against the PX4 RC map -- MAVLink RC_CHANNELS
        is 1-based and reports the RAW (pre-mapping) channel order.
      * decide whether a mid-band reading during a brownout should be treated as
        GO-off (it is here) or as link-denied."""

    def __init__(self, high_us: int = 1700, low_us: int = 1300):
        if low_us >= high_us:
            raise ValueError("low_us must be < high_us")
        self.high_us = int(high_us)
        self.low_us = int(low_us)
        self.state = False

    def update(self, pwm: Optional[int]) -> bool:
        if pwm is None:
            return self.state
        if pwm >= self.high_us:
            self.state = True
        elif pwm <= self.low_us:
            self.state = False
        return self.state


class RcChannelTrigger:
    """The REAL trigger: an ELRS aux switch read out of MAVLink `RC_CHANNELS`.

    PATH (docs/launch_mechanism_plan.md Sec 4): switch -> ELRS -> RP3 RX -> SBUS
    -> Pixhawk 6C Mini RC-IN -> PX4 -> MAVLink RC_CHANNELS over TELEM2 -> the Pi.
    No new hardware, no second link.

    WHY IT IS NOT A DATALINK CUE (constraint `no-datalink`): the GO bit is a
    COMMAND, not a CUE. It carries zero target information, it is a single edge,
    and the RF link may be jammed/killed the instant after it fires with no
    effect on the intercept. This class CANNOT carry target data -- the only
    thing it exports is a boolean and a staleness age.

    MECHANISM NOTE: MAVSDK-Python's telemetry plugin exposes only `rc_status()`
    (signal strength / availability), NOT per-channel values, so the driver feeds
    this object from the `mavlink_direct` plugin's raw RC_CHANNELS subscription.
    TODO-BUILDER: verify on the bench (L3) that PX4 actually STREAMS RC_CHANNELS
    on TELEM2 at a useful rate -- it may need SR / MAV_x_RATE tuning, and if it
    does not, fall back to a pymavlink reader on the same port."""

    def __init__(self, channel: int = 7, high_us: int = 1700, low_us: int = 1300,
                 link_timeout_s: float = 1.0):
        self.channel = int(channel)
        self.switch = PwmSwitch(high_us, low_us)
        self.link_timeout_s = float(link_timeout_s)
        self.last_pwm: Optional[int] = None
        self.last_rx_t: Optional[float] = None

    def ingest(self, fields: dict, t: float) -> None:
        """Fold one decoded RC_CHANNELS message in. `fields` is the JSON body
        MAVSDK's mavlink_direct hands back (chan1_raw .. chan18_raw, rssi, ...)."""
        pwm = fields.get(f"chan{self.channel}_raw")
        if pwm is None:
            return
        self.last_pwm = int(pwm)
        self.last_rx_t = t
        self.switch.update(self.last_pwm)

    def poll(self, t: float) -> TriggerState:
        age = None if self.last_rx_t is None else (t - self.last_rx_t)
        link_ok = age is not None and age <= self.link_timeout_s
        return TriggerState(go=self.switch.state and link_ok, link_ok=link_ok,
                            age_s=age, raw_us=self.last_pwm)


class ScriptedTrigger:
    """OFFLINE ONLY (tests, --dry-run, --sitl-smoke). Fires GO at `go_at_s`, can
    simulate a link that dies at `link_dies_at_s`, and can start HIGH (to prove
    the switch-high-at-boot refusal)."""

    def __init__(self, go_at_s: Optional[float] = None,
                 link_dies_at_s: Optional[float] = None,
                 start_high: bool = False, link_alive: bool = True):
        self.go_at_s = go_at_s
        self.link_dies_at_s = link_dies_at_s
        self.start_high = start_high
        self.link_alive = link_alive
        self.forced_go = False

    def poll(self, t: float) -> TriggerState:
        link_ok = self.link_alive and (self.link_dies_at_s is None
                                       or t < self.link_dies_at_s)
        go = self.forced_go or self.start_high or (
            self.go_at_s is not None and t >= self.go_at_s)
        return TriggerState(go=bool(go), link_ok=link_ok,
                            age_s=(0.0 if link_ok else 999.0),
                            raw_us=(1900 if go else 1100))


class GateReadyTrigger(ScriptedTrigger):
    """SMOKE ONLY: synthesises the human GO edge as soon as the state machine's
    OWN arm gate has been satisfied for `hold_ticks` consecutive ticks.

    Why not a fixed time: after takeoff PX4 must still slew to the commanded aim
    yaw, and the gate's +-2 deg yaw tolerance would otherwise refuse a fixed-time
    GO and (correctly) abort the smoke to SAFE. Waiting on the gate makes the
    SITL check exercise the ARM GATE POSITIVELY and keeps it deterministic. It is
    NOT how the real aircraft fires -- a human does."""

    def __init__(self, hold_ticks: int = 4):
        super().__init__()
        self.hold_ticks = int(hold_ticks)
        self._ok_ticks = 0

    def observe_gate(self, gate_ok: bool) -> None:
        self._ok_ticks = self._ok_ticks + 1 if gate_ok else 0
        if self._ok_ticks >= self.hold_ticks:
            self.forced_go = True


# ---------------------------------------------------------------- offline vehicle


class FakeVehicle:
    """A crude kinematic stand-in for --dry-run and the unit tests: altitude
    integrates the commanded v_down, yaw slews toward the commanded yaw at a
    bounded rate. It is NOT a flight-dynamics model and is not used for any
    number that gets claimed anywhere -- only to walk the state machine."""

    def __init__(self, alt_m: float = 0.0, yaw_deg: float = 0.0,
                 yaw_rate_deg_s: float = 45.0, armed: bool = True,
                 offboard: bool = True):
        self.alt_m = float(alt_m)
        self.yaw_deg = float(yaw_deg)
        self.yaw_rate = float(yaw_rate_deg_s)
        self.armed = armed
        self.offboard = offboard

    def apply(self, sp: Setpoint, dt: float) -> None:
        self.alt_m = max(0.0, self.alt_m - sp.v_down * dt)   # NED: down positive
        err = wrap_deg(sp.yaw_deg - self.yaw_deg)
        step = _clamp(err, -self.yaw_rate * dt, self.yaw_rate * dt)
        self.yaw_deg = wrap_deg(self.yaw_deg + step)

    def obs(self, t: float, trigger: TriggerState, **kw) -> VehicleObs:
        return VehicleObs(t=t, armed=self.armed, offboard_active=self.offboard,
                          alt_m=self.alt_m, yaw_deg=self.yaw_deg,
                          quat=(1.0, 0.0, 0.0, 0.0), trigger=trigger, **kw)


# ---------------------------------------------------------------- honesty audit


_LATCH_ATTR = "dash_heading"
# Identifier prefixes that would mean a ground-truth read. There is no ground
# truth on real hardware; this asserts none crept in from a sim-side copy/paste.
_FORBIDDEN_ID_PREFIXES = ("gt_",)
_FORBIDDEN_IMPORT_ROOTS = ("gz", "gz_transport", "gz.transport13")


def honesty_audit(path: Optional[str] = None, verbose: bool = True) -> int:
    """Prove -- from the SYNTAX TREE, not the text -- that this module:

      1. reads NO ground-truth symbol (no identifier with a forbidden prefix),
      2. imports no simulator/ground-truth module,
      3. writes the dash heading latch at EXACTLY ONE call site (the GO edge),
         i.e. `docs/launch_mechanism_plan.md` L2's "assigned once, never again".

    A comment or a docstring cannot fool it (the walk ignores string constants),
    which is why the module's prose may freely discuss the boundary. Mirrors the
    existing AST-audit pattern in scripts/field/fc_link_check.py.

    Exit 0 = clean, 1 = violation."""
    path = path or os.path.abspath(__file__)
    with open(path) as fh:
        tree = ast.parse(fh.read(), filename=path)
    problems: List[str] = []

    def bad(name: str) -> bool:
        return any(name.startswith(p) for p in _FORBIDDEN_ID_PREFIXES)

    latch_sets = 0
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and bad(node.id):
            problems.append(f"line {node.lineno}: ground-truth identifier {node.id}")
        elif isinstance(node, ast.Attribute):
            if bad(node.attr):
                problems.append(
                    f"line {node.lineno}: ground-truth attribute .{node.attr}")
        elif isinstance(node, ast.arg) and bad(node.arg):
            problems.append(f"line {node.lineno}: ground-truth argument {node.arg}")
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            mod = getattr(node, "module", None) or ""
            names = [a.name for a in node.names]
            for m in [mod] + names:
                root = (m or "").split(".")[0]
                if root and root in _FORBIDDEN_IMPORT_ROOTS:
                    problems.append(f"line {node.lineno}: simulator import {m}")
        if isinstance(node, ast.Call):
            f = node.func
            if isinstance(f, ast.Attribute) and f.attr == "set" \
                    and isinstance(f.value, ast.Attribute) \
                    and f.value.attr == _LATCH_ATTR:
                latch_sets += 1

    if latch_sets != 1:
        problems.append(
            f"the dash-heading latch is written at {latch_sets} call sites "
            "(must be EXACTLY 1 -- the GO edge)")

    if verbose:
        print(f"[audit] {os.path.relpath(path, _REPO_ROOT)}")
        print(f"  [{'PASS' if latch_sets == 1 else 'FAIL'}] dash heading latched at "
              f"exactly one call site (found {latch_sets})")
        n_gt = sum(1 for p in problems if "ground-truth" in p)
        print(f"  [{'PASS' if n_gt == 0 else 'FAIL'}] no ground-truth identifier "
              f"is read anywhere (found {n_gt})")
        n_imp = sum(1 for p in problems if "simulator import" in p)
        print(f"  [{'PASS' if n_imp == 0 else 'FAIL'}] no simulator/ground-truth "
              f"import (found {n_imp})")
        print("  [INFO] inputs by construction: camera pixels (ENGAGE only), "
              "own-state EKF, pre-flight constants, and ONE arm/kill trigger bit")
        for p in problems:
            print(f"  VIOLATION: {p}")
        print(f"[audit] {'PASS' if not problems else 'FAIL'}")
    return 0 if not problems else 1


# ---------------------------------------------------------------- CSV logging


_CSV_HEADER = ("t_s,state,streak,armed,offboard,alt_m,yaw_deg,trig_go,trig_link,"
               "trig_age_s,trig_us,det_new,det_range_m,v_north,v_east,v_down,"
               "yaw_cmd_deg,safe_reason\n")


def _csv_row(obs: VehicleObs, dec: Decision) -> str:
    sp = dec.setpoint
    def f(x, nd=4):
        return "" if x is None else f"{x:.{nd}f}"
    return (f"{obs.t:.3f},{dec.state},{dec.streak},{obs.armed},"
            f"{obs.offboard_active},{f(obs.alt_m,3)},{f(obs.yaw_deg,2)},"
            f"{int(bool(obs.trigger.go))},{int(bool(obs.trigger.link_ok))},"
            f"{f(obs.trigger.age_s,2)},{obs.trigger.raw_us or ''},"
            f"{int(bool(obs.det_new))},{f(obs.det_range_m,3)},"
            f"{sp.v_north:.4f},{sp.v_east:.4f},{sp.v_down:.4f},"
            f"{sp.yaw_deg:.2f},{dec.safe_reason or ''}\n")


# ---------------------------------------------------------------- offline driver


def run_offline(cfg: MissionConfig, trigger, guidance: Optional[SeekerGuidance] = None,
                fps: float = 20.0, max_s: float = 90.0,
                detect_after_dash_s: float = 1.0, detect_every: int = 2,
                stub_r0: float = 8.0, stub_shrink: float = 0.25,
                verbose: bool = True, csv_path: Optional[str] = None):
    """--dry-run: walk the WHOLE state machine with a fake vehicle, a scripted
    trigger and a stub seeker, PRINTING every transition and a setpoint sample.
    No MAVSDK, no camera, no sim. Returns (sm, rows)."""
    from flight.deploy.seeker_loop import StubSeeker

    sm = RealFlightSM(cfg, guidance=guidance,
                      on_event=(print if verbose else None))
    veh = FakeVehicle(alt_m=cfg.standby_alt_m, yaw_deg=0.0)
    stub = StubSeeker(r0=stub_r0, r_shrink=stub_shrink)
    dt = 1.0 / fps
    rows: List[str] = []
    n = 0
    t = 0.0
    while t <= max_s:
        trig = trigger.poll(t)
        det_new = det_range = det_box = None
        if sm.t_dash_start is not None and \
                t >= sm.t_dash_start + detect_after_dash_s and (n % detect_every == 0):
            d = stub.detect(None, t)
            det_new, det_range, det_box = True, d.range_m, d.box_xywh
        obs = veh.obs(t, trig, det_new=bool(det_new), det_range_m=det_range,
                      det_box_xywh=det_box)
        if isinstance(trigger, GateReadyTrigger):
            trigger.observe_gate(sm.arm_gate(obs)[0])
        dec = sm.step(obs)
        rows.append(_csv_row(obs, dec))
        if verbose and n % 10 == 0:
            sp = dec.setpoint
            print(f"  t={t:6.2f}  {dec.state:<10} streak={dec.streak} "
                  f"alt={veh.alt_m:5.2f} yaw={veh.yaw_deg:+7.2f} "
                  f"| SP vN={sp.v_north:+6.2f} vE={sp.v_east:+6.2f} "
                  f"vD={sp.v_down:+5.2f} yaw={sp.yaw_deg:+7.2f}")
        veh.apply(dec.setpoint, dt)
        if dec.terminated:
            if verbose:
                print(f"  t={t:6.2f}  TERMINATED in SAFE "
                      f"(reason={dec.safe_reason}); driver would now LAND")
            break
        t += dt
        n += 1
    if csv_path:
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        with open(csv_path, "w") as fh:
            fh.write(_CSV_HEADER)
            fh.writelines(rows)
        if verbose:
            print(f"[log] {len(rows)} ticks -> {csv_path}")
    return sm, rows


# ---------------------------------------------------------------- MAVSDK driver

# Timeouts mirror flight/deploy/seeker_loop.py (same PX4 SITL boot, same traps).
_CONNECT_TIMEOUT_S = 30.0
_HEALTH_TIMEOUT_S = 60.0
_TAKEOFF_TIMEOUT_S = 60.0
_TAKEOFF_ALT_FRAC = 0.8
_LAND_TIMEOUT_S = 60.0


async def run_mavsdk_mission(args, cfg: MissionConfig, sm: RealFlightSM,
                             trigger, detector=None,
                             smoke: bool = False):  # pragma: no cover -- needs a vehicle
    """Live driver: connect over MAVLink, stream own-state EKF + RC_CHANNELS, and
    stream the state machine's NED velocity+yaw setpoints through PX4 OFFBOARD.

    Unlike seeker_loop, this driver OWNS the takeoff in BOTH modes -- the standby
    hover is the whole point, so arm/takeoff/land are part of the real sequence,
    not a smoke-only synthesis. `smoke=True` changes only two things:
      * the trigger is synthetic (GateReadyTrigger / a timed edge) instead of a
        human on an ELRS switch, and the detector is synthetic (no camera), and
      * the run asserts a PASS/FAIL contract at the end (states visited, setpoint
        count, heading latched) so the SITL check can exit 0/1.

    TODO-BUILDER before this ever flies: PX4 geofence sized to the range box,
    the RC kill switch verified live (RC_MAP_KILL_SW), battery failsafe, and a
    props-off bench pass of the whole sequence (launch_mechanism_plan L3)."""
    import asyncio

    from mavsdk import System
    from mavsdk.offboard import OffboardError, VelocityNedYaw
    from mavsdk.telemetry import LandedState

    drone = System()

    def _kill_server():
        # MAVSDK-Python spawns a mavsdk_server child per System(); kill it if we
        # bail out, so a failed connect never leaks a server (the trap measured
        # 2026-07-24 in seeker_loop.py / fc_link_check.py).
        proc = getattr(drone, "_server_process", None)
        if proc is not None and proc.poll() is None:
            proc.kill()

    print(f"[mavsdk] connecting {args.mavsdk_url} ...")
    # connect() itself blocks forever on a dead link (mavsdk_server does not open
    # its gRPC port until it discovers a vehicle), so the timeout MUST wrap it.
    try:
        await asyncio.wait_for(drone.connect(system_address=args.mavsdk_url),
                               timeout=_CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        print(f"[mavsdk] FAIL: connect() did not return within {_CONNECT_TIMEOUT_S}s")
        _kill_server()
        return 1

    async def _wait_connected():
        async for st in drone.core.connection_state():
            if st.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_connected(), timeout=_CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        print(f"[mavsdk] FAIL: no connection within {_CONNECT_TIMEOUT_S}s")
        _kill_server()
        return 1
    print("[mavsdk] connected")

    # OWN-STATE EKF + own status. The ONLY non-camera, non-constant inputs.
    state = {"quat": None, "psi": None, "alt": None, "armed": None,
             "landed": None, "offboard": None}

    async def _att_q():
        async for q in drone.telemetry.attitude_quaternion():
            state["quat"] = (q.w, q.x, q.y, q.z)

    async def _att_e():
        async for e in drone.telemetry.attitude_euler():
            state["psi"] = e.yaw_deg

    async def _pos():
        async for p in drone.telemetry.position():
            state["alt"] = p.relative_altitude_m

    async def _armed():
        async for a in drone.telemetry.armed():
            state["armed"] = a

    async def _landed():
        async for ls in drone.telemetry.landed_state():
            state["landed"] = ls

    async def _rc():
        """RC_CHANNELS -> the ELRS aux switch. MAVSDK's telemetry plugin has no
        per-channel accessor, so this uses the mavlink_direct raw subscription."""
        async for msg in drone.mavlink_direct.message("RC_CHANNELS"):
            try:
                trigger.ingest(json.loads(msg.fields_json), time.monotonic())
            except Exception as e:  # noqa: BLE001 -- never let a bad frame kill the loop
                print(f"[rc] skipped a malformed RC_CHANNELS: {e}")

    coros = [_att_q, _att_e, _pos, _armed, _landed]
    if isinstance(trigger, RcChannelTrigger):
        coros.append(_rc)
    tasks = [asyncio.ensure_future(c()) for c in coros]

    rc_exit = 0
    offboard_ok = False
    n_tick = n_sp = 0
    csv_rows: List[str] = []
    try:
        print(f"[mavsdk] waiting for health (global+home position ok), timeout "
              f"{_HEALTH_TIMEOUT_S}s ...")

        async def _wait_health():
            async for h in drone.telemetry.health():
                if h.is_global_position_ok and h.is_home_position_ok:
                    return

        try:
            await asyncio.wait_for(_wait_health(), timeout=_HEALTH_TIMEOUT_S)
        except asyncio.TimeoutError:
            print(f"[mavsdk] FAIL: health not OK within {_HEALTH_TIMEOUT_S}s")
            return 1
        print("[mavsdk] health OK")

        alt = cfg.standby_alt_m
        print(f"[mavsdk] set takeoff altitude {alt:.1f} m (standby/loft); arm; takeoff ...")
        await drone.action.set_takeoff_altitude(alt)
        await drone.action.arm()
        await drone.action.takeoff()
        deadline = time.monotonic() + _TAKEOFF_TIMEOUT_S
        while time.monotonic() < deadline:
            if state["alt"] is not None and state["alt"] >= _TAKEOFF_ALT_FRAC * alt:
                break
            await asyncio.sleep(0.2)
        a_now = state["alt"]
        print(f"[mavsdk] airborne at rel_alt="
              f"{'None' if a_now is None else f'{a_now:.2f}'} m")

        # OFFBOARD needs an established setpoint stream before the mode switch.
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0,
                                                             cfg.aim_yaw_deg))
        try:
            await drone.offboard.start()
            offboard_ok = True
            state["offboard"] = True
            print("[mavsdk] OFFBOARD active; STANDBY hold stream begins")
        except OffboardError as e:
            print(f"[mavsdk] FAIL: offboard start failed: {e}")
            rc_exit = 1

        if offboard_ok:
            t0 = time.monotonic()
            frame = None
            if detector is not None:
                import numpy as _np
                frame = _np.zeros((960, 1280, 3), dtype=_np.uint8)
            while True:
                t = time.monotonic() - t0
                if t > args.smoke_duration:
                    print(f"[mavsdk] outer smoke bound {args.smoke_duration:.0f}s "
                          f"reached in state {sm.state} -- forcing shutdown")
                    rc_exit = 1
                    break
                trig = trigger.poll(t)
                det_new = det_range = det_box = None
                if detector is not None and sm.t_dash_start is not None \
                        and t >= sm.t_dash_start + args.smoke_acquire_after_s \
                        and (n_tick % 2 == 0):
                    # SYNTHETIC detection (no camera / weights / onnx): the smoke
                    # validates the MAVLink + state-machine plumbing, not perception.
                    # Every OTHER tick, so the "no new result -> HOLD" streak
                    # branch is exercised too.
                    d = detector.detect(frame, t)
                    det_new, det_range, det_box = True, d.range_m, d.box_xywh
                obs = VehicleObs(t=t, armed=state["armed"],
                                 offboard_active=state["offboard"],
                                 alt_m=state["alt"], yaw_deg=state["psi"],
                                 quat=state["quat"], trigger=trig,
                                 det_new=bool(det_new), det_range_m=det_range,
                                 det_box_xywh=det_box)
                if isinstance(trigger, GateReadyTrigger):
                    trigger.observe_gate(sm.arm_gate(obs)[0])
                dec = sm.step(obs)
                csv_rows.append(_csv_row(obs, dec))
                sp = dec.setpoint
                if args.dry_run:
                    print(f"[dry-run] {dec.state} SETPOINT {sp.as_tuple()}")
                else:
                    await drone.offboard.set_velocity_ned(VelocityNedYaw(
                        sp.v_north, sp.v_east, sp.v_down, sp.yaw_deg))
                n_sp += 1
                n_tick += 1
                if n_tick % 20 == 0:
                    _alt = state["alt"]
                    _alt_s = "None" if _alt is None else f"{_alt:.2f}"
                    _yaw = state["psi"]
                    _yaw_s = "None" if _yaw is None else f"{_yaw:+.1f}"
                    print(f"[mavsdk] t={t:5.1f}s {dec.state:<10} "
                          f"streak={dec.streak} alt={_alt_s} yaw={_yaw_s} "
                          f"| SP vN={sp.v_north:+.2f} vE={sp.v_east:+.2f} "
                          f"vD={sp.v_down:+.2f} yaw={sp.yaw_deg:+.1f}")
                if dec.terminated:
                    print(f"[mavsdk] state machine TERMINATED in SAFE "
                          f"(reason={dec.safe_reason})")
                    break
                await asyncio.sleep(1.0 / args.fps)
    finally:
        if offboard_ok:
            try:
                await drone.offboard.stop()
                print("[mavsdk] offboard stopped")
            except Exception as e:  # noqa: BLE001 -- best-effort teardown
                print(f"[mavsdk] offboard.stop skipped: {e}")
        try:
            await drone.action.land()
            print(f"[mavsdk] landing; waiting up to {_LAND_TIMEOUT_S}s for "
                  f"touchdown + disarm ...")
            deadline = time.monotonic() + _LAND_TIMEOUT_S
            touch_t = None
            forced = False
            while time.monotonic() < deadline:
                if state["armed"] is False:
                    break
                if state["landed"] == LandedState.ON_GROUND and touch_t is None:
                    touch_t = time.monotonic()
                if touch_t is not None and not forced and \
                        time.monotonic() - touch_t > 5.0:
                    forced = True
                    try:
                        await drone.action.disarm()
                        print("[mavsdk] forced disarm (auto-disarm lagged)")
                    except Exception as e:  # noqa: BLE001
                        print(f"[mavsdk] disarm request skipped: {e}")
                await asyncio.sleep(0.5)
            print(f"[mavsdk] landed_state={state['landed']} armed={state['armed']}")
        except Exception as e:  # noqa: BLE001 -- best-effort teardown
            print(f"[mavsdk] land skipped: {e}")
        for tk in tasks:
            tk.cancel()
        if getattr(args, "log_csv", None):
            os.makedirs(os.path.dirname(args.log_csv), exist_ok=True)
            with open(args.log_csv, "w") as fh:
                fh.write(_CSV_HEADER)
                fh.writelines(csv_rows)
            print(f"[log] {len(csv_rows)} ticks -> {args.log_csv}")

    # ---- PASS/FAIL for the SITL gate ------------------------------------------
    visited = sm.visited
    print(f"[mission] states visited: {' -> '.join(visited)}")
    print("[mission] transitions:")
    for tr in sm.transitions:
        print(f"    {tr.t:7.2f}s  {tr.frm} -> {tr.to}  ({tr.reason})")
    if smoke:
        need = (State.CODED_DASH, State.ENGAGE, State.SAFE)
        for st in need:
            if st not in visited:
                print(f"[sitl-smoke] FAIL: never reached {st}")
                rc_exit = 1
        if not offboard_ok:
            rc_exit = 1
        if n_sp < 20:
            print(f"[sitl-smoke] FAIL: only {n_sp} setpoints streamed")
            rc_exit = 1
        if not sm.dash_heading.is_set:
            print("[sitl-smoke] FAIL: dash heading was never latched")
            rc_exit = 1
        print(f"[sitl-smoke] {'PASS' if rc_exit == 0 else 'FAIL'} "
              f"({n_sp} setpoints, dash heading "
              f"{sm.dash_heading.value if sm.dash_heading.is_set else 'unset'})")
    return rc_exit


# ---------------------------------------------------------------- self-test


def _sm(cfg=None, **kw) -> RealFlightSM:
    return RealFlightSM(cfg or MissionConfig(**kw))


def _obs(t, **kw):
    # yaw defaults to the scenarios' aim heading (40 deg): the arm gate REQUIRES
    # the aircraft to be holding C1 +-yaw_tol before a GO is accepted.
    base = dict(armed=True, offboard_active=True, alt_m=7.0, yaw_deg=40.0,
                quat=(1.0, 0.0, 0.0, 0.0))
    base.update(kw)
    trig = base.pop("trigger", TriggerState(go=False, link_ok=True, age_s=0.05))
    return VehicleObs(t=t, trigger=trig, **base)


def self_test() -> int:
    """A handful of end-to-end scenario assertions with NO MAVSDK, NO sim and NO
    camera. The exhaustive version lives in flight/tests/test_real_flight.py;
    this is the exit-0/1 smoke a gate script can call."""
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        ok = ok and cond

    cfg = MissionConfig(preflight_heading_deg=40.0, standby_settle_s=0.0,
                        dash_speed_ms=10.0, dash_accel_ms2=10.0)

    # 1) happy path: GO -> dash -> 5 fresh detections -> ENGAGE
    sm = _sm(cfg)
    go = TriggerState(go=True, link_ok=True, age_s=0.05)
    no = TriggerState(go=False, link_ok=True, age_s=0.05)
    sm.step(_obs(0.0, trigger=no))
    sm.step(_obs(0.05, trigger=go))
    check(sm.state == State.CODED_DASH, "a GO edge with all guards satisfied dashes")
    check(abs(sm.dash_heading.value - 40.0) < 1e-9,
          f"the dash heading latched to the pre-flight constant "
          f"({sm.dash_heading.value:.1f} deg)")
    for i in range(5):
        sm.step(_obs(0.10 + 0.05 * i, trigger=no, det_new=True, det_range_m=9.0))
    check(sm.state == State.ENGAGE, "5 consecutive fresh detections hand off to ENGAGE")

    # 2) LINK DENIED in STANDBY -> do NOT dash
    sm = _sm(cfg)
    sm.step(_obs(0.0, trigger=no))
    sm.step(_obs(0.05, trigger=TriggerState(go=False, link_ok=False, age_s=3.0)))
    check(sm.state == State.SAFE and sm.safe_reason.startswith("link_denied"),
          f"link loss in STANDBY -> SAFE, no dash ({sm.safe_reason})")

    # 3) link loss AFTER the GO edge is IGNORED (the jam-resistance deliverable)
    sm = _sm(cfg)
    sm.step(_obs(0.0, trigger=no))
    sm.step(_obs(0.05, trigger=go))
    dead = TriggerState(go=False, link_ok=False, age_s=9.0)
    for i in range(20):
        sm.step(_obs(0.10 + 0.05 * i, trigger=dead))
    check(sm.state == State.CODED_DASH,
          "link loss AFTER the GO edge does NOT abort the dash (constraint no-datalink)")

    # 4) dash timeout with no acquire -> SAFE, not a blind continue
    sm = _sm(MissionConfig(preflight_heading_deg=40.0, standby_settle_s=0.0,
                           dash_max_s=1.0))
    sm.step(_obs(0.0, trigger=no))
    sm.step(_obs(0.05, trigger=go))
    for i in range(40):
        sm.step(_obs(0.10 + 0.05 * i, trigger=no))
    check(sm.state == State.SAFE and sm.safe_reason == "dash_timeout_no_acquire",
          f"dash timeout with no acquire -> SAFE ({sm.safe_reason})")

    # 5) the latch fires exactly AT the GO edge and the audit is clean.
    #    (The "a second write RAISES" negative test deliberately lives in
    #    flight/tests/test_real_flight.py, NOT here: a `.dash_heading.set(` call
    #    anywhere in THIS file would be a second static write site and would --
    #    correctly -- fail --audit. The audit is not weakened for a test.)
    sm = _sm(cfg)
    sm.step(_obs(0.0, trigger=no))
    sm.step(_obs(0.05, trigger=go))
    check(sm.dash_heading.is_set and abs(sm.dash_heading.t_set - 0.05) < 1e-9,
          f"the dash heading is latched AT the GO edge (t={sm.dash_heading.t_set})")
    check(honesty_audit(verbose=False) == 0,
          "the AST honesty audit passes (one latch site, no ground-truth read)")

    # 6) every state emits a setpoint (PX4 drops OFFBOARD without a stream)
    sm = _sm(cfg)
    seen = set()
    dec = sm.step(_obs(0.0, trigger=no))
    seen.add(dec.state)
    all_have_sp = dec.setpoint is not None
    t = 0.05
    trig = go
    for i in range(400):
        d = sm.step(_obs(t, trigger=trig, det_new=(i % 2 == 0),
                         det_range_m=max(0.3, 9.0 - 0.15 * i)))
        seen.add(d.state)
        all_have_sp = all_have_sp and isinstance(d.setpoint, Setpoint)
        trig = no
        t += 0.05
        if d.terminated:
            break
    check(all_have_sp, "every tick in every state emits a setpoint")
    check(seen >= {State.STANDBY, State.CODED_DASH, State.ENGAGE,
                   State.BREAKOFF, State.SAFE},
          f"the full path STANDBY->DASH->ENGAGE->BREAKOFF->SAFE is reachable "
          f"({sorted(seen)})")

    print(f"\n[self-test] {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# ---------------------------------------------------------------- CLI


def build_config(args) -> MissionConfig:
    """Resolve the pre-flight constants ONCE, before anything flies."""
    heading = args.dash_heading_deg
    t_lead = None
    if heading is None:
        tstart = tuple(float(v) for v in args.target_start.split(",")[:2])
        tvel = tuple(float(v) for v in args.target_vel.split(",")[:2])
        heading, t_lead = resolve_preflight_heading(
            tstart, tvel, args.dash_speed, args.dash_accel_ms2,
            accel_aware=not args.no_accel_aware_lead)
        print(f"[aim] pre-flight collision lead "
              f"({'accel-aware, ADR-0080' if not args.no_accel_aware_lead else 'constant-speed'}"
              f", a={args.dash_accel_ms2:.1f} m/s^2, v={args.dash_speed:.1f} m/s) "
              f"-> C1 = {heading:.2f} deg"
              + (f", t_lead={t_lead:.2f} s" if t_lead is not None else
                 ", t_lead=None (uncatchable by pure lead)"))
    else:
        print(f"[aim] C1 = {heading:.2f} deg (operator-entered --dash-heading-deg)")
    return MissionConfig(
        preflight_heading_deg=heading,
        standby_alt_m=args.standby_alt_m,
        standby_yaw_deg=args.standby_yaw_deg,
        dash_speed_ms=args.dash_speed,
        dash_accel_ms2=args.dash_accel_ms2,
        dash_accel_cap_ms2=args.dash_accel_cap,
        dash_loft_m=args.dash_loft_m,
        dash_loft_dive_s=args.dash_loft_dive_s,
        dash_max_s=args.dash_max_s,
        dash_max_dist_m=args.dash_max_dist_m,
        acquire_streak=args.acquire_streak,
        engage_max_s=args.engage_max_s,
        standby_settle_s=args.standby_settle_s,
        standby_max_s=args.standby_max_s,
        mission_max_s=args.mission_max_s,
        link_timeout_s=args.link_timeout_s,
        alt_tol_m=args.alt_tol_m,
        yaw_tol_deg=args.yaw_tol_deg,
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Real interceptor onboard flight state machine "
                    "(STANDBY -> CODED_DASH -> ENGAGE -> BREAKOFF -> SAFE). "
                    "SKELETON FOR REVIEW -- NOT FLIGHT-READY.")
    mode = ap.add_argument_group("modes")
    mode.add_argument("--self-test", action="store_true",
                      help="pure-logic scenario checks (no MAVSDK/sim/camera); exits 0/1")
    mode.add_argument("--audit", action="store_true",
                      help="AST honesty audit: no ground-truth read, no sim import, "
                           "dash heading latched at exactly one call site; exits 0/1")
    mode.add_argument("--dry-run", action="store_true",
                      help="offline: fake vehicle + scripted trigger + stub seeker; "
                           "PRINTS every transition and setpoint. No MAVSDK.")
    mode.add_argument("--sitl-smoke", action="store_true",
                      help="drive the LIVE MAVSDK OFFBOARD path against a local PX4 "
                           "SITL with a synthetic trigger + synthetic detector: "
                           "arm, takeoff, STANDBY hold, GO, dash, acquire, ENGAGE, "
                           "breakoff, SAFE, land+disarm. Needs --mavsdk-url. Exits 0/1.")
    mode.add_argument("--mavsdk-url", default=None,
                      help="MAVSDK connect URL (e.g. udpin://0.0.0.0:14540)")

    aim = ap.add_argument_group("pre-flight aim constants (C1/C2/C3)")
    aim.add_argument("--dash-heading-deg", type=float, default=None,
                     help="C1 override (NED compass deg). Omit to solve the "
                          "collision lead from --target-start/--target-vel.")
    aim.add_argument("--target-start", default="0,16.7",
                     help="operator-entered target position at launch, 'east,north' m")
    aim.add_argument("--target-vel", default="9,0",
                     help="operator-entered target velocity, 'east,north' m/s")
    aim.add_argument("--no-accel-aware-lead", action="store_true",
                     help="use the old CONSTANT-SPEED lead solve (under-leads: the "
                          "dash starts at rest -- see ADR-0080)")
    aim.add_argument("--standby-alt-m", type=float, default=7.0,
                     help="C3 = target altitude + loft (TODO-BUILDER)")
    aim.add_argument("--standby-yaw-deg", type=float, default=None,
                     help="standby aim yaw; default = C1")

    dash = ap.add_argument_group("dash")
    dash.add_argument("--dash-speed", type=float, default=16.0)
    dash.add_argument("--dash-accel-ms2", type=float, default=10.0,
                      help="the acceleration the dash actually flies (feeds the "
                           "lead solve + the distance bound). TODO-BUILDER: from "
                           "the first real dash ULog.")
    dash.add_argument("--dash-accel-cap", type=float, default=None,
                      help="commanded accel ramp cap (slower, aim-forgiving profile)")
    dash.add_argument("--dash-loft-m", type=float, default=0.0)
    dash.add_argument("--dash-loft-dive-s", type=float, default=2.0)
    dash.add_argument("--dash-max-s", type=float, default=8.0)
    dash.add_argument("--dash-max-dist-m", type=float, default=None)
    dash.add_argument("--acquire-streak", type=int, default=5)
    dash.add_argument("--engage-max-s", type=float, default=12.0)

    saf = ap.add_argument_group("safety / failsafe (ALL TODO-BUILDER)")
    saf.add_argument("--standby-settle-s", type=float, default=2.0)
    saf.add_argument("--standby-max-s", type=float, default=60.0)
    saf.add_argument("--mission-max-s", type=float, default=120.0)
    saf.add_argument("--link-timeout-s", type=float, default=1.0)
    saf.add_argument("--alt-tol-m", type=float, default=0.3)
    saf.add_argument("--yaw-tol-deg", type=float, default=2.0)

    trg = ap.add_argument_group("trigger")
    trg.add_argument("--trigger", choices=("rc", "timer", "gate-ready"),
                     default="gate-ready",
                     help="'rc' = the REAL ELRS aux channel via MAVLink RC_CHANNELS; "
                          "'timer'/'gate-ready' are OFFLINE/SMOKE synthetics")
    trg.add_argument("--rc-channel", type=int, default=7,
                     help="RC_CHANNELS index of the GO switch (TODO-BUILDER: verify "
                          "against the PX4 RC map)")
    trg.add_argument("--rc-go-us", type=int, default=1700)
    trg.add_argument("--rc-low-us", type=int, default=1300)
    trg.add_argument("--go-after-s", type=float, default=8.0,
                     help="--trigger timer: synthesise the GO edge at this time")

    trm = ap.add_argument_group("terminal seeker (composed from seeker_loop)")
    trm.add_argument("--intrinsics",
                     default=os.path.join(_REPO_ROOT, "camera_intrinsics.json"))
    trm.add_argument("--n-pronav", type=float, default=5.0)
    trm.add_argument("--mount-fwd-m", type=float, default=0.10)
    trm.add_argument("--mount-left-m", type=float, default=0.0)
    trm.add_argument("--mount-up-m", type=float, default=0.05)
    trm.add_argument("--mount-tilt-deg", type=float, default=0.0)

    run = ap.add_argument_group("run")
    run.add_argument("--fps", type=float, default=20.0)
    run.add_argument("--smoke-duration", type=float, default=90.0,
                     help="outer wall bound on the setpoint stream (forces shutdown)")
    run.add_argument("--smoke-acquire-after-s", type=float, default=2.0,
                     help="--sitl-smoke: synthetic detections begin this many "
                          "seconds into the dash")
    run.add_argument("--log-csv", default=None,
                     help="per-tick CSV (default: auto-named under logs/)")
    args = ap.parse_args(argv)

    if args.audit:
        return honesty_audit()
    if args.self_test:
        return self_test()

    if not (args.dry_run or args.sitl_smoke):
        ap.error("pick a mode: --self-test | --audit | --dry-run | --sitl-smoke")

    cfg = build_config(args)
    print(f"[config] standby alt {cfg.standby_alt_m:.1f} m, aim yaw "
          f"{cfg.aim_yaw_deg:.1f} deg, dash {cfg.dash_speed_ms:.1f} m/s "
          f"(cap {cfg.dash_accel_cap_ms2}), loft {cfg.dash_loft_m:.1f} m, "
          f"streak {cfg.acquire_streak}, dash bound {cfg.dash_max_s:.1f} s")
    print("[config] *** SKELETON -- every failsafe constant above is "
          "TODO-BUILDER (unmeasured on the airframe) ***")

    # The terminal, composed (NOT re-implemented): the same SeekerGuidance the
    # sim validates and the bench runs.
    gcfg = GuidanceConfig(n_pronav=args.n_pronav, mount_fwd_m=args.mount_fwd_m,
                          mount_left_m=args.mount_left_m,
                          mount_up_m=args.mount_up_m,
                          mount_up_rad=math.radians(args.mount_tilt_deg),
                          alt_ref_m=cfg.dash_base_alt_m)
    from flight.camera import CameraModel
    cam = (load_camera(args.intrinsics) if os.path.exists(args.intrinsics)
           else CameraModel(539.936, 539.936, 640.0, 480.0))
    guidance = SeekerGuidance(gcfg, cam, gcfg.target_span_m)

    ts = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    if args.log_csv is None:
        args.log_csv = os.path.join(_REPO_ROOT, "logs", f"real_flight_{ts}.csv")

    if args.dry_run:
        trigger = (ScriptedTrigger(go_at_s=args.go_after_s)
                   if args.trigger == "timer" else GateReadyTrigger())
        sm, rows = run_offline(cfg, trigger, guidance=guidance, fps=args.fps,
                               max_s=args.mission_max_s, csv_path=args.log_csv)
        print(f"[mission] states visited: {' -> '.join(sm.visited)}")
        for tr in sm.transitions:
            print(f"    {tr.t:7.2f}s  {tr.frm} -> {tr.to}  ({tr.reason})")
        reached = State.SAFE in sm.visited and State.ENGAGE in sm.visited
        print(f"[dry-run] {'PASS' if reached else 'FAIL'} "
              f"(heading latched: {sm.dash_heading})")
        return 0 if reached else 1

    # --sitl-smoke
    if not args.mavsdk_url:
        ap.error("--sitl-smoke requires --mavsdk-url (e.g. udpin://0.0.0.0:14540)")
    if args.trigger == "rc":
        print("[trigger] REAL RC_CHANNELS path selected -- SITL has no RC input "
              "by default, so this will (correctly) never fire a GO.")
        trigger = RcChannelTrigger(args.rc_channel, args.rc_go_us, args.rc_low_us,
                                   cfg.link_timeout_s)
    elif args.trigger == "timer":
        trigger = ScriptedTrigger(go_at_s=args.go_after_s)
    else:
        trigger = GateReadyTrigger()
    from flight.deploy.seeker_loop import SmokeSeeker
    detector = SmokeSeeker(cam.fx, gcfg.target_span_m, cx=cam.cx, cy=cam.cy)
    sm = RealFlightSM(cfg, guidance=guidance, on_event=print)
    import asyncio
    return asyncio.run(run_mavsdk_mission(args, cfg, sm, trigger, detector,
                                          smoke=True))


if __name__ == "__main__":
    raise SystemExit(main())
