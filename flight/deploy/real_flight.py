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
                                  from flight.guidance. No LIVE datalink, no gt;
                                  optional PRE-FLIGHT-LATCHED cue (--cue-json,
                                  flight.cue_relay) -- latched before GO, link
                                  then dies.
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

POST-GO OFFBOARD LOSS (FAILSAFE 7, builder decision #22): after the GO edge the
RF link may die freely (that is the whole no-datalink claim), but the OFFBOARD
CONTROL PATH may not. If PX4 leaves OFFBOARD -- or the flight-mode sample that
claims it is IN OFFBOARD goes stale, i.e. we no longer KNOW -- during
CODED_DASH/ENGAGE/BREAKOFF, the machine goes SAFE. Two reasons, both about
props-on hazard: (1) while PX4 is out of OFFBOARD our setpoints are not being
flown, so every guard downstream of them (the dash bound, breakoff, the terminal)
is scoring a flight that is not happening; (2) SAFE is ABSORBING, so when the
pilot hands OFFBOARD back the vehicle receives a ZERO-velocity hold + a land
request -- NOT a live, half-ramped dash command that would fire the moment
control returns. Link-loss and OFFBOARD-loss are deliberately opposite policies:
one is expected and ignored, the other is a control-path failure and aborts.

COMPOSITION (this file adds a state machine and NOTHING else)
------------------------------------------------------------
  * the terminal            -> flight.deploy.seeker_loop.SeekerGuidance (imported)
  * the dash aim            -> flight.guidance.collision_lead_heading_accel (ADR-0080)
  * the dash speed ramp     -> flight.guidance.dash_forward_speed
  * the loft-then-dive ref  -> flight.guidance.dash_loft_alt_ref
  * the alt trim + AGL floor -> flight.guidance.apply_alt_ref_trim
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

  # PROPS-OFF REHEARSAL on the real Pi + Pixhawk (cannot arm; no flight battery)
  .venv-pi/bin/python -m flight.deploy.real_flight --bench-props-off \
      --mavsdk-url serial:///dev/ttyAMA10:921600 --trigger rc --rc-channel 7

  # THE REAL FLIGHT (the only mode that may arm over a serial link)
  .venv-pi/bin/python -m flight.deploy.real_flight --fly \
      --mavsdk-url serial:///dev/ttyAMA10:921600 \
      --mavsdk-backup-url udpout://127.0.0.1:14550 \
      --trigger rc --rc-channel 7 \
      --arm-permit-channel 8 --source picamera --camera-mount upside-down \
      --target-span-m 0.30 \
      --cue-json cue.jsonl --own-lat ... --own-lon ... --own-alt-m-msl ...
  # --mavsdk-backup-url: the Pixhawk's USB port through the flight recorder; a
  #   failed serial cable then no longer ends the flight (link_redundancy.py).
  # --fly also suspends the Pixhawk's low-battery action and GPS quality checks
  #   for the flight and restores them after landing (board_overrides.py);
  #   --board-failsafes keep leaves the board alone.

THE THREE LIVE MODES (2026-11-18, interface audit): `--sitl-smoke` arms and
takes off on its own with a synthetic GO and a synthetic target, so it refuses
a serial link. `--bench-props-off` runs the same control loop on the real board
and structurally cannot arm. `--fly` is the real flight: it refuses synthetic
triggers and detectors and a belief nobody entered, reads the board's limits,
prints a pre-arm summary, and arms only on the operator's RC arm permit. Every
live path runs under flight/deploy/loop_supervisor.py, which ends the setpoint
stream if the control loop stalls or this program dies.

DEPS: pure stdlib + numpy (via seeker_loop). `mavsdk` is a GUARDED import used
only by the live driver, so every offline path runs on this x86 desk.
"""
from __future__ import annotations

import argparse
import ast
import json
import math
import weakref
import os
import sys
import time
from dataclasses import dataclass, field, replace
from typing import Any, Callable, List, Optional, Sequence, Tuple

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from flight.guidance import (  # noqa: E402
    collision_lead_heading,
    collision_lead_heading_accel,
    dash_forward_speed,
    apply_alt_ref_trim,
    dash_loft_alt_ref,
    floor_v_down,
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
    # LOST-TARGET CLOCK SWITCHES (2026-11-18; all default OFF = the clock above,
    # byte-identical). Why they exist: the 2.0 s clock was earned at 9 m/s and
    # 16 m range, where a loss happens inside 3 m and is final. On the 500 m
    # approach dash it ends engagements the terminal could still win
    # (docs/guidance_review_2026-11-18.md F1, section 1; isim/specs/
    # approach_dash_drag_prereg_2026-11-18.md stages 3 and 5). Builder ruling
    # the same day: "whatever gets highest intercept rate, I'll be supervising
    # so I can disengage if it gets ugly". The flying values live in ONE place,
    # FLY_LOST_TARGET_PRESET below; every other mode keeps these defaults.
    #  (a) needs_track: in pursuit mode a decode does not start the clock until
    #      the terminal has had a track (its Phase B). One stray decode during
    #      the belief transit can then no longer end the engagement 2 s later.
    pursuit_lost_clock_needs_track: bool = False
    #  (c) off_in_search: the clock does not run on any tick the terminal
    #      reports it is in its arrival search (still looking, by design).
    pursuit_lost_clock_off_in_search: bool = False
    #  (b) far-loss clock: used INSTEAD of engage_lost_target_s when the last
    #      range estimate is beyond pursuit_miss_range_m, so the terminal's own
    #      return-to-belief logic (fallback_s, 3.0 s) gets to run. None = off.
    #      The close-miss rule is untouched: inside the miss range the short
    #      clock still sends the aircraft straight to SAFE.
    engage_far_lost_target_s: Optional[float] = None
    # THE CUE IS THE GO COMMAND (--trigger cue, builder ruling 2026-11-18). When
    # True the GO edge must arrive WITH a belief latched from the live cue link
    # that same tick (VehicleObs.cue_belief) and the terminal must be able to
    # take it; otherwise GO goes SAFE instead of flying on a stale belief.
    cue_go: bool = False
    # ADR-0103 "chase only": the pursuit terminal deliberately does NOT fly a
    # fly-by -- Phase A matches the target's speed at a standoff, so range can
    # legitimately hold flat or wobble for seconds before Phase B ever starts
    # closing. The past-CPA RECESSION trigger below (count-and-magnitude on a
    # RISING range) was tuned for a fast crossing dash that flies THROUGH the
    # target and must detect having passed it; against pursuit's slow, non-
    # monotonic approach it is a false-abort hazard, not a real recession.
    # `pursuit_mode=True` suppresses ONLY that trigger -- `breakoff_hard_floor_m`
    # (contact, the 0.35 m ram radius, ADR-0084), `engage_lost_target_s`, and
    # `engage_max_s` all stay active; a pursuit engagement still ends on
    # contact or a real failsafe, never on an untested "it must be a fly-by"
    # assumption. docs/pursuit_port_2026-09-17.md work item 3.
    pursuit_mode: bool = False
    # PURSUIT-MODE MISS POLICY (builder ruling 2026-09-22, ADR-0105 open item
    # (a)). `isim/specs/parity_flightcode_2026-09-21.md`'s PRE-REGISTERED
    # abort-timer A/B (n=50/cell, identical seeds, engage_lost_target_s
    # 2.0/4.0/10.0/30.0 s) came back a byte-identical NULL at every window: at
    # close range a decode dropout is PERMANENT and pursuit's own Phase-B->A
    # fallback never produces a second converging pass through this state
    # machine, so lengthening the lost-target clock buys nothing -- the
    # engagement's scored CPA is already made on the FIRST pass. So instead
    # of raising the timer, the miss is handled explicitly: if the last KF
    # range estimate (`StepTelemetry.r_hat_m`, tracked continuously through
    # Phase B whether or not this tick decoded) was inside
    # `pursuit_miss_range_m` when `engage_lost_target_s` expires, the miss has
    # already happened at close quarters and FAILSAFE 5 (`_step_engage`)
    # transitions ENGAGE -> SAFE DIRECTLY (reason `pursuit_miss`), skipping
    # BREAKOFF's climb-and-search -- which assumes a fly-by pursuit never
    # flies -- so the vehicle gets OUT of the target's airspace and
    # hovers/RTLs/lands per `safe_behavior` instead. Outside that range the
    # ORIGINAL target_lost -> BREAKOFF path stands (a belief re-acquire may
    # still help at range). Default matches
    # `flight.pursuit_terminal.PursuitTerminalConfig.no_fallback_range_m`
    # (3.0 m) -- the same range pursuit's own internal fallback uses to judge
    # "close enough that a re-approach would not help". Inert outside
    # pursuit_mode (the past-CPA/BREAKOFF path there is unaffected).
    pursuit_miss_range_m: float = 3.0
    breakoff_arm_range_m: float = 4.0      # arms past-CPA logic (m4 FPV profile)
    breakoff_range_increases: int = 3
    breakoff_range_deadband_m: float = 0.05
    # MAGNITUDE guard on the past-CPA breakoff -- the missing half of the count
    # guard, ported from scripts/m4_intercept.py (`--breakoff-min-rise-m`). The
    # count alone fires on N tiny rises, i.e. on a SLOW DRIFT of the range
    # estimate; this requires the total rise from the BASE of the rising run to
    # clear a real distance before "past CPA" is believed.
    #
    # DERIVATION of the 0.30 m default (sim-derived; TODO-BUILDER, see below).
    # Range comes from the known-size span: r = fx*span/w_px, so one pixel of
    # box-width noise moves the estimate by dr = r/w = r^2/(fx*span). At the
    # breakoff ARM range (4 m) with fx=540 px and span~0.36 m, w~49 px and
    # dr~0.08 m/px, so a 2-3 px box wobble can drift ~0.16-0.25 m -- which is
    # exactly the "slow drift fires breakoff" hazard. 0.30 m sits above that and
    # far below a true post-CPA recession (>=0.1 m per fresh detection even at
    # the 2 m/s bottom rung of the kill-day ladder, i.e. >=0.3 m over the 3
    # required rises, and metres at the 9 m/s rung). NOTE the count guard alone
    # already implies deadband*increases = 0.15 m, so anything <=0.15 would be
    # VACUOUS -- 0.30 m is the first value that actually bites.
    # TODO-BUILDER: re-derive from the OV9281 intrinsics (bench L4) and the
    # MEASURED box-width noise of the deployed detector (bench L5); until then
    # this is a sim-geometry estimate, not a measured one.
    breakoff_min_rise_m: float = 0.30
    # m4's `--breakoff-max-range-m` twin. INERT by default (None), kept so this
    # config matches the sim's contract knob-for-knob.
    breakoff_max_range_m: Optional[float] = None
    breakoff_hard_floor_m: float = 0.5
    breakoff_s: float = 1.5
    breakoff_climb_ms: float = 1.0

    # --- ADR-0101 PASSAGE WINDOW (ported from scripts/m4_intercept.py's
    # --breakoff-min-flown-frac / --breakoff-force-flown-frac -- that ADR's OWN
    # disclosed open item: "real_flight.py still carries the old rule ... port
    # deliberately, not silently"). An own-motion FLOOR and CEILING on the
    # past-CPA `past_cpa` trigger in `_step_engage`:
    #   FLOOR   (breakoff_min_flown_frac) the range-increase breakoff may not
    #           fire until the vehicle's own displacement since the dash began
    #           reaches this fraction of the pre-flight PLAN distance -- you
    #           cannot have PASSED a target you have not yet flown far enough
    #           to REACH (m4 offline replay: a 0.8 gate blocks 9/9 premature
    #           breakoffs and only 3/38 legitimate ones, all three already
    #           past a recorded CPA).
    #   CEILING (breakoff_force_flown_frac) the terminal is FORCED to
    #           breakoff once that same displacement reaches this fraction,
    #           whatever the camera says -- m4 measured true closest approach
    #           always arriving by 1.05x planned over 64 flown flights.
    # Both read OWN-STATE (integrated ground speed, `_flown_since_dash_m`) and
    # ONE pre-flight constant (`dash_plan_m`, below) -- no live target read,
    # no gt_*. Defaults None = OFF, byte-identical (matches m4's own inert
    # default). APPLIES ONLY to the stock dash+terminal path: pursuit_mode has
    # no coded dash and already suppresses the past-CPA trigger entirely
    # (ADR-0103), so both knobs are inert there regardless of value.
    breakoff_min_flown_frac: Optional[float] = None
    breakoff_force_flown_frac: Optional[float] = None
    # The pre-flight lead solve's OWN intercept distance (m) -- set by
    # `build_config()` from the SOLVED heading's t_lead (m4's
    # `coded_dash_plan_m`). None when the heading is an explicit
    # --dash-heading-deg override: no solved distance exists then, so per
    # ADR-0101 "Portability" the floor fails CLOSED (breakoff never fires
    # early) and the ceiling never fires, by design -- not a bug to chase.
    # Honesty class: pre-flight constant, same as preflight_heading_deg --
    # not a live read.
    dash_plan_m: Optional[float] = None

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
    require_attitude_quat: bool = True     # arm gate: no dash without a live quat
    # ENGAGE: consecutive ticks on which the terminal reports `range_track_broken`
    # (r_hat <= 0 or a non-physical range rate) before BREAKOFF. A diverged range
    # channel must abort loudly, never silently select the coast branch (review2).
    engage_track_broken_ticks: int = 3
    require_low_before_go: bool = True     # a switch HIGH at boot is NEVER a GO
    safe_hold_s: float = 2.0               # hold in SAFE before the driver lands

    # --- SAFE behavior (builder ruling 2026-09-22: "as soon as an intercept
    # is MISSED, abort immediately and either hover in place or return home --
    # for real flights in a large open field"). The STATE MACHINE keeps
    # emitting SAFE's hold setpoint every tick regardless of this value (PX4
    # drops OFFBOARD on a stream gap of ~0.5 s) -- only WHEN a land/RTL
    # request is issued, and by whom, changes.
    #   'land'  (default, UNCHANGED byte-for-byte): hold safe_hold_s, then
    #           `Decision.land_requested` goes True and the driver requests
    #           PX4 LAND, exactly as before this option existed.
    #   'hover' hold position INDEFINITELY: `land_requested` stays False and
    #           `Decision.terminated` does not fire at safe_hold_s. The
    #           `mission_max_s` ABSOLUTE BACKSTOP still applies and THEN
    #           lands (a hover that can never end is not a safe default).
    #   'rtl'   DRIVER-LEVEL ONLY. The state machine's own behaviour is
    #           BYTE-IDENTICAL to 'land' (terminates at safe_hold_s,
    #           land_requested=True) -- it does not know what RTL is, by
    #           design. Only the LIVE MAVSDK driver reads this value, and at
    #           that same instant commands PX4 RETURN-TO-LAUNCH instead of
    #           LAND. RTL is PX4's OWN GPS-based safety behaviour (fly home,
    #           then land) -- it is a SAFETY fallback, NOT a guidance
    #           decision, and must never be read as one.
    safe_behavior: str = "land"
    # BUILDER RULING 2026-09-22 ("let's go to hover after miss for now, I can
    # add that second attempt again later" -- ADR-0107): a MISS-class SAFE
    # (the ENGAGEMENT ended without contact: `pursuit_miss`,
    # `breakoff_complete`) hovers by default instead of landing, holding
    # position for the operator while the option of a future re-approach
    # feature stays open. SYSTEM-HEALTH aborts (offboard lost, link denied,
    # timeouts, gate failures) deliberately keep `safe_behavior` -- when the
    # control path itself is suspect, coming down is safer than loitering.
    # Same three values as `safe_behavior`; the `mission_max_s` backstop
    # still lands a hover (a hover that can never end is not a safe default).
    miss_safe_behavior: str = "hover"

    # --- FAILSAFE 7: POST-GO OFFBOARD LOSS (builder decision #22) --------------
    # The RF LINK may die after GO (expected, ignored -- constraint no-datalink).
    # The OFFBOARD CONTROL PATH may not: if PX4 is not flying our setpoints, the
    # dash is not happening, and a live mid-ramp command must not be waiting to
    # execute when the pilot hands control back. Both arms -> SAFE.
    require_offboard_after_go: bool = True   # NEVER set False for a props-on flight
    # ARM 1 (mode loss): how long `offboard_active is not True` may PERSIST after
    # the GO edge before aborting. Not zero, because one dropped/late telemetry
    # sample must not end an engagement.
    # SIM-DERIVED DEFAULT, TODO-BUILDER: 0.5 s is PX4's own OFFBOARD setpoint
    # timeout (COM_OF_LOSS_T family / the ~0.5 s stream gap that drops the mode),
    # so it is the shortest bound that cannot fire on a healthy vehicle. At the
    # 16 m/s dash that is ~8 m of un-flown travel -- size it DOWN once the real
    # flight_mode cadence is measured on the Pi (bench L3/L6).
    offboard_lost_s: float = 0.5
    # ARM 2 (stale knowledge): `offboard_active` is a value in a dict fed by a
    # telemetry subscriber. If that subscriber dies the value FREEZES at True and
    # arm 1 can never fire -- the exact silent-failure shape this project keeps
    # hitting (the driver already prints "an own-state stream is now FROZEN").
    # So an offboard sample older than this is treated as LOSS, not as health.
    # None disables the arm (arm 1 still stands).
    # MEASURED (not assumed): the MAVSDK flight_mode stream arrives at the ~1 Hz
    # heartbeat rate -- max observed sample age 1.00 s over a full 328-tick
    # SITL mission (logs/real_flight_sitl_20260725T200929Z.csv, offb_age_s
    # column; scripts/check_real_flight_sitl.sh PASS 2026-07-25). 3.0 s is 3x
    # that. RE-MEASURED ON THE REAL Pi5<-Pixhawk 6C Mini TELEM2 LINK
    # (bench L3/L6, 2026-09-22, /dev/serial0 @ 921600, props off, one 30 s
    # window; logs/bench_telem2_cadence_20260922.json): HEARTBEAT (what
    # carries flight_mode) max inter-arrival 1.002 s -- the SAME ~1 Hz
    # cadence SITL showed, so the 3x-heartbeat sizing HOLDS on real hardware.
    # (Same bench window: ATTITUDE 100 Hz, ATTITUDE_QUATERNION 50 Hz,
    # LOCAL_POSITION_NED 30 Hz -- not this arm's concern, noted for the other
    # own-state consumers.) NOT YET RE-CONFIRMED: this was a raw pymavlink
    # listen, not MAVSDK's own `telemetry.flight_mode()` subscription --
    # worth a quick check in the first SITL/bench driver run (the driver
    # PRINTS the max observed sample age every run) before treating the
    # MAVSDK-side cadence as identical.
    offboard_stale_s: Optional[float] = 3.0

    # --- FAILSAFE 9: STALE OWN-STATE (interface audit 2026-11-18, gap G7) ------
    # Attitude, yaw, height and velocity each arrive on their own telemetry
    # stream. If one stops, its last value stays in the driver's dict FOREVER
    # and every consumer keeps computing on it: a frozen height makes the
    # STANDBY height loop command a constant climb or descent, a frozen
    # attitude rotates every camera bearing by the wrong amount. The terminal
    # already refuses to STEER on own-state older than
    # GuidanceConfig.own_state_max_age_s (0.25 s) -- but until 2026-11-18 this
    # driver never told it the age, so that check never ran here.
    # This bound is the state machine's own: own-state older than this, in ANY
    # state but SAFE, goes SAFE. It is deliberately looser than the terminal's
    # 0.25 s (a short gap should cost a few held ticks, not the flight) and
    # tighter than FAILSAFE 7's 3.0 s mode-sample bound, which until now was
    # the only thing that could notice a dead link after GO -- 3 s at dash
    # speed is up to 100 m.
    # None disables it. `VehicleObs.own_state_age_s` None (a driver that does
    # not stamp its streams: the offline vehicle, the desk mode, isim's
    # adapter) leaves it inactive, byte-identical to before.
    # TODO-BUILDER: 1.0 s is 30 missed samples of the slowest consumed stream
    # as measured on the real TELEM2 link (30 Hz, runs/fc_readonly_20261118/);
    # tighten it once the bench props-off run has printed the worst age seen.
    own_state_stale_s: Optional[float] = 1.0

    # --- BATTERY: before GO and after the intercept, never between (builder
    # ruling 2026-11-26: "ensure low battery triggers won't activate once
    # intercept go is activated"). The Pixhawk's own low-battery action is
    # switched to "warning only" for a program-flown flight
    # (flight/deploy/board_overrides.py), because a full-power dash sags the
    # pack voltage and PX4's voltage-only gauge reads that as empty. This is
    # what stands in for it:
    #   STANDBY   charge below `battery_low_frac` for `battery_low_hold_s`
    #             -> SAFE (`battery_low_before_go`), which lands per
    #             `safe_behavior`. No GO on a low pack.
    #   CODED_DASH / ENGAGE / BREAKOFF   IGNORED. One event says so.
    #   SAFE      a hover (`miss_safe_behavior='hover'`) ends in a landing as
    #             soon as the charge is low for the hold time.
    # `battery_low_frac` is a fraction 0..1 of charge left (the same number
    # PX4's BAT_CRIT_THR is compared with); None = this rule is inactive and
    # the state machine is byte-identical to before. A missing reading
    # (VehicleObs.battery_frac None) never triggers anything.
    # The hold time keeps a moment of sag in the hover from counting as low.
    battery_low_frac: Optional[float] = None
    battery_low_hold_s: float = 3.0

    # --- FAILSAFE 10: DEAD CAMERA (review 2026-11-26) ---------------------------
    # After GO the camera is the only sensor that can find the target. If its
    # capture thread dies or the sensor stalls, the detector simply reports "no
    # detection" for ever -- and with no engage time limit, and the battery
    # ignored after GO, the aircraft would search blind until the pilot or the
    # geofence stopped it. So: in ENGAGE, a newest frame older than this goes
    # SAFE (`camera_dead`), which ends per `safe_behavior`. This is total loss
    # of the sensor, not a degraded one: frames that arrive but show no target
    # never trip it. None = off (byte-identical); VehicleObs.camera_age_s None
    # (a detector with no live feed: the synthetic one, a replay) = inactive.
    # 3.0 s is 180 missed frames at the camera's 60 fps.
    camera_dead_s: Optional[float] = None

    # --- vertical / hold control ----------------------------------------------
    kp_alt: float = 1.0
    v_vert_max_ms: float = 2.0
    # ADR-0085 decided a camera-driven terminal vertical channel with a hard AGL
    # minimum, and NEITHER HALF was ever written (contradiction
    # `terminal-vertical-channel-decided-not-built`). These two land the half the
    # measurements support plus the safety floor. Both DEFAULT-INERT: at 0.0/None
    # `_v_down` is byte-identical to the pre-2026-09-10 behaviour.
    #
    # dash_alt_trim_m -- pre-flight constant from
    #   flight.guidance.derive_dash_alt_trim_m(measured drift). The vertical error
    #   is DELIVERED BY THE DASH (vertical_miss_anatomy.py: off-altitude at CPA on
    #   24/24 committed flights, and dash drift is 66% of it) and the terminal has
    #   0.02-0.17 m of authority left at measured closing speeds
    #   (handoff_closing_speed.py) -- so it is corrected here, as ADR-0080/0083 did
    #   for horizontal aim, not in the terminal.
    #   TODO-BUILDER: derive from THIS airframe. The sign is
    #   configuration-dependent (cue-era arms end HIGH, ADR-0095's fleet ended LOW),
    #   so the mechanism transfers and the number does not. Leave 0.0 until measured.
    # min_agl_m -- ADR-0085's hard floor. The reference never goes below it,
    #   whatever a trim or a future seeker elevation asks for. TODO-BUILDER: set
    #   from the site's own geometry before any props-on flight; None disables it,
    #   which is the pre-existing (unfloored) behaviour and NOT a safe default.
    dash_alt_trim_m: float = 0.0
    min_agl_m: Optional[float] = None

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


# THE FLYING PRESET (one place, on purpose). `--fly` applies these over the
# defaults above unless the operator typed the matching flag; no other mode
# does, so every existing gate and test keeps the old behaviour. Builder
# rulings 2026-11-18: on the lost-target rule, "whatever gets highest intercept
# rate, I'll be supervising so I can disengage if it gets ugly"; on the engage
# time, "no limit at least for now". THE HEAD SETS THESE VALUES from the
# flight-hardware sim results; change them HERE and nowhere else.
FLY_PRESET = {
    # lost-target clock (MissionConfig docstrings (a), (c), (b))
    "pursuit_lost_clock_needs_track": True,
    "pursuit_lost_clock_off_in_search": True,
    "engage_far_lost_target_s": 6.0,   # fast sim: 91 -> 102 kills of 400, none lost
    # time limits in pursuit mode: none. What then ends an engagement that never
    # finds the target is listed in `what_ends_an_unbounded_engagement()`.
    "engage_max_s": math.inf,
    "mission_max_s": math.inf,
    # ARRIVAL SEARCH ON (builder ruling 2026-11-18, "whatever gets highest
    # intercept rate"). Flight-hardware rerun, flying lens at 20 deg of camera
    # tilt, all errors on, straight target: 117 kills of 192 without the
    # search, 190 with `believed`, 187 with `stern` (within 3 of 192; `believed`
    # is the registered tie-break because it costs no time when the cue is
    # good). Fast-sim evidence only. Cost: engagements run to about 40 s with a
    # tail past 60 s, and `believed` adds blind passes close to the target.
    # isim/specs/flight_hardware_rerun_prereg_2026-11-18.md,
    # isim/specs/arrival_search_prereg_2026-11-18.md.
    "pursuit_arrival_search": "believed",
    # The search re-uses the acquisition list after a dropout; the guard stops
    # a stale decode restarting the camera phase every tick.
    "pursuit_stale_decode_guard": True,
    # APPROACH-DASH BRAKE 3 m/s^2, with the vertical arrival-sync it was
    # simulated with. Same rerun: brakes 2, 3 and 4 all gave 16 kills of 16 at
    # 20 deg of tilt; 3 costs 0.8 s against 4 and leaves more margin on a
    # braking authority that has never been measured on the real aircraft.
    # Applies only when the approach dash is on.
    "pursuit_dash_brake_accel": 3.0,
    "pursuit_brake_vert_sync": True,
    # FAILSAFE 10: a camera that has delivered no frame for this long ends the
    # engagement (MissionConfig.camera_dead_s).
    "camera_dead_s": 3.0,
}

# Horizon used where a finite number is needed and the engage window is not
# (the believed-rendezvous arithmetic of the pre-arm limit check).
_UNBOUNDED_HORIZON_S = 300.0


def what_ends_an_unbounded_engagement() -> List[str]:
    """With no engage time limit, the things that still end an engagement that
    never finds its target. Printed in the pre-arm summary. Pure."""
    return [
        "the pilot: the mode switch takes the aircraft (this program then sends "
        "nothing), and the kill switch always works",
        "the board's geofence: on a breach PX4 leaves OFFBOARD and this program "
        "goes SAFE",
        "the battery: before GO a low pack lands the aircraft; after GO it "
        "is ignored until the engagement ends (then a low pack lands the "
        "hover). With --board-failsafes keep, PX4's own low-battery action "
        "stays live instead and ends the intercept",
        "a stale own-state stream or a hung program (the watchdog ends the "
        "setpoint stream)",
        "a dead camera: no frame at all for 3 s (frames with no target in them "
        "do not count)",
        "a target that was found and then lost for good (the lost-target rule)",
    ]


# ---------------------------------------------------------------- observations


@dataclass
class TriggerState:
    """What the RF link says THIS tick. Arm/kill class only -- there is no field
    here that could carry target information, by construction."""

    go: bool = False                    # switch currently commanding GO
    link_ok: bool = False               # a fresh RC frame arrived recently
    age_s: Optional[float] = None       # seconds since the last RC frame
    raw_us: Optional[int] = None        # last PWM (logging only)
    # A NEGATIVE age is physically impossible: it means ingest() and poll() were
    # stamped from different clock bases, which silently pins link_ok True
    # forever (review2 BLOCKER). Surfaced so the log says so instead of "healthy".
    clock_fault: bool = False


@dataclass
class VehicleObs:
    """Everything the state machine may read on one tick. Own-state EKF + own
    status + the camera + the one trigger bit. No ground truth, by construction."""

    t: float                                    # monotonic seconds
    armed: Optional[bool] = None
    # LIVE flight-mode-derived (see make_mode_subscriber). None = no sample yet,
    # which the arm gate treats as FAIL (it tests `is not True`). It is NOT
    # "offboard.start() returned OK" -- those are different facts.
    offboard_active: Optional[bool] = None
    # Age (s) of the flight-mode sample that produced `offboard_active`. FAILSAFE
    # 7 arm 2 reads it: a frozen subscriber leaves `offboard_active` pinned at its
    # last value forever, and only its AGE can say so. None = the driver did not
    # supply it (arm 2 inactive); a NEGATIVE value is a clock-base fault and is
    # treated as loss, exactly like TriggerState.clock_fault.
    offboard_sample_age_s: Optional[float] = None
    mode: Optional[str] = None                  # PX4 flight mode name (log only)
    alt_m: Optional[float] = None               # relative altitude (own EKF)
    yaw_deg: Optional[float] = None             # own EKF yaw
    quat: Optional[Tuple[float, float, float, float]] = None  # (w,x,y,z) body->NED
    # Horizontal ground speed from the own-state EKF (same honesty class as
    # alt_m). When present FAILSAFE 4 INTEGRATES it instead of dead-reckoning off
    # `dash_accel_ms2` -- that constant is a MISS-MAE aim fit, not a kinematic
    # acceleration, and ADR-0083 measured it over-predicting travel by ~2.7x in
    # the CPA window (22.2 m predicted vs 8.2 m flown).
    ground_speed_ms: Optional[float] = None
    trigger: TriggerState = field(default_factory=TriggerState)
    # Camera, exactly as the detector reports it (finetuned_seeker.SeekerDetection)
    det_new: bool = False                       # a NEW detector result this tick
    det_box_xywh: Optional[Sequence[float]] = None
    det_range_m: Optional[float] = None
    # OPTIONAL detector-PnP slant range for the SAME detection (2026-09-23,
    # tick-trace S2 fix -- isim/specs/xcheck_tick_trace_2026-09-23.md): the
    # AprilTag pose solver's |t|, camera origin to tag centre. Same honesty
    # class as the box (camera pixels + known tag size, no ground truth).
    # Default None so every existing caller/driver/test is byte-unaffected;
    # consumed ONLY by a terminal that advertises SUPPORTS_POSE_RANGE
    # (flight.pursuit_terminal) -- the stock/tag terminals' AABB range
    # channel is under the separate ADR-0105 ruling and is not changed.
    det_range_pose_m: Optional[float] = None
    # STAMPED measurement age (s): t minus the detection's own capture stamp,
    # filled by the driver ONLY under --pursuit-stamped-age and only when the
    # detector publishes t_capture on the same clock (FeedDetector does; see
    # runs/gz_latency_20261022 LJ90 for why jitter makes this worth having).
    # None = the terminal keeps its fixed meas_latency_s assumption.
    det_age_s: Optional[float] = None
    # Bearing for the OPTIONAL pre-launch bearing latch (degrees, camera-relative).
    # Read ONLY at the GO edge and only when cfg.bearing_latch_enabled.
    det_bearing_deg: Optional[float] = None
    # Own-state EKF ground velocity, NED (m/s). Optional (default None, so every
    # existing caller/test is unaffected): only flight/tag_terminal.py's 3-D
    # relative-velocity estimator reads it via own_state() below -- own-state,
    # not ground truth, the vector twin of `ground_speed_ms` above.
    vel_ned: Optional[Tuple[float, float, float]] = None
    # Age (s) of the OLDEST own-state stream the guidance consumes (attitude
    # quaternion, yaw, height, velocity), as stamped by the driver on arrival.
    # None = the driver does not stamp (offline vehicle, desk, isim adapter):
    # the terminal's staleness check and FAILSAFE 9 both stay inactive, exactly
    # as before. See MissionConfig.own_state_stale_s.
    own_state_age_s: Optional[float] = None
    # Battery charge left as the Pixhawk reports it, 0..1. None = no reading
    # (or the driver does not supply one): the battery rule then does nothing.
    # Read in STANDBY and SAFE only; see MissionConfig.battery_low_frac.
    battery_frac: Optional[float] = None
    # Seconds since the camera's newest frame was captured (the live feed's own
    # clock). None = no live feed to ask. See MissionConfig.camera_dead_s.
    camera_age_s: Optional[float] = None
    # THE CUE, LATCHED (--trigger cue): ((north, east, down) m, (vn, ve, vd)
    # m/s), the target relative to THIS vehicle and its velocity, advanced to
    # this instant. The driver sets it on exactly ONE tick, the tick the cue
    # trigger fires GO, and it is None on every other tick of the flight. A
    # pre-flight given (graded given-noisy), read at the GO edge and never
    # after -- the same class as the bearing latch above.
    cue_belief: Optional[Tuple[Tuple[float, float, float],
                               Tuple[float, float, float]]] = None

    def own_state(self) -> OwnState:
        return OwnState(quat=self.quat,
                        psi_rad=(None if self.yaw_deg is None
                                 else math.radians(self.yaw_deg)),
                        alt_m=self.alt_m, vel_ned=self.vel_ned,
                        age_s=self.own_state_age_s)


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
    # The EFFECTIVE vertical reference this tick's command was built from, and
    # whether the AGL floor clamped it. Carried on the Decision so `_csv_row` can
    # log what was APPLIED rather than re-deriving what it thinks should have been
    # (review F11; the ADR-0096 lesson about actuators that log intent).
    alt_ref_m: Optional[float] = None
    floor_active: bool = False


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


def update_range_increase_streak(streak: int, last_fresh_range: Optional[float],
                                 streak_start_range: Optional[float],
                                 meas_range: float, deadband: float):
    """Past-CPA range-INCREASE streak with a noise deadband AND the streak BASE.

    CONTRACT-IDENTICAL to `scripts/m4_intercept.update_range_increase_streak` --
    same name, same signature, same branches, so the two can be fuzzed against
    each other case-for-case (the executor may not IMPORT the sim module: that is
    a honesty-audit violation, `_FORBIDDEN_IMPORT_ROOTS`).

    Returns (streak, last_fresh_range, streak_start_range).

      * a RISE (meas > last_fresh + deadband) advances the streak, and on the
        FIRST rise of a run records `streak_start_range` = the range at the BASE
        of that run -- which is what makes a MAGNITUDE test possible at all,
      * a within-deadband CREEP holds the streak WITHOUT advancing the baseline
        (quantization jitter cannot ratchet a false streak),
      * a genuine DROP resets both the streak and the base.

    PORTED 2026-07-25 (audit reader 1 / completeness): the executor previously
    carried the COUNT half only, so N tiny rises -- a slow drift of the range
    estimate -- could fire breakoff on the real aircraft while the sim-validated
    twin would not. The base returned here feeds `cfg.breakoff_min_rise_m` in
    `_step_engage`; that pair IS the ported guard."""
    if last_fresh_range is None:
        return 0, meas_range, None
    if meas_range > last_fresh_range + deadband:
        if streak == 0:
            streak_start_range = last_fresh_range
        return streak + 1, meas_range, streak_start_range
    if meas_range > last_fresh_range:
        # within-deadband creep: hold streak, keep the last SIGNIFICANT range
        return streak, last_fresh_range, streak_start_range
    return 0, meas_range, None


def update_recede_streak(streak: int, last_range: Optional[float],
                         meas_range: float, deadband: float):
    """BACK-COMPAT 2-tuple view of `update_range_increase_streak` (streak,
    last_range) -- the streak/baseline halves are element-for-element identical;
    only the streak BASE is dropped. Kept because it is the published signature
    (flight/tests/test_real_flight.py); the state machine calls the 3-tuple form,
    because the base is what the magnitude guard needs."""
    s, last, _ = update_range_increase_streak(streak, last_range, None,
                                              meas_range, deadband)
    return s, last


def passage_gate_ok(flown_m: Optional[float], planned_m: Optional[float],
                    min_frac: Optional[float]) -> bool:
    """ADR-0101 PASSAGE GATE (the FLOOR) on the past-CPA breakoff: you cannot
    have PASSED a target you have not yet flown far enough to REACH.

    CONTRACT-IDENTICAL to `scripts/m4_intercept.passage_gate_ok` -- same
    name, same signature, same branches, so the two can be fuzzed against
    each other case-for-case (the executor may not IMPORT the sim module:
    `_FORBIDDEN_IMPORT_ROOTS`).

      flown_m   own displacement since the dash began (own-state only)
      planned_m the PRE-FLIGHT lead solve's own intercept distance
      min_frac  None = gate OFF (always True, byte-identical)

    True = the breakoff may fire. FAIL-CLOSED on a missing input while the
    gate is ON: an unmeasured distance must not read as 'far enough' (the
    lost-target/engage-timeout failsafes remain the backstop). Reads nothing
    about the target in flight and nothing from gt_*."""
    if min_frac is None:
        return True
    if flown_m is None or planned_m is None or planned_m <= 0.0:
        return False
    return flown_m >= min_frac * planned_m


def passage_ceiling_hit(flown_m: Optional[float], planned_m: Optional[float],
                        force_frac: Optional[float]) -> bool:
    """ADR-0101 PASSAGE CEILING, the mirror of `passage_gate_ok`: once the
    vehicle's OWN displacement since the dash began exceeds
    `force_frac * planned_m` it MUST have passed the target, whatever the
    camera says (m4 measured true closest approach always arriving by 1.05x
    planned, over 64 flown camera flights).

    CONTRACT-IDENTICAL to `scripts/m4_intercept.passage_ceiling_hit`.
    None/missing input -> never forces (False): a missing input must not
    force a breakoff (the engage-timeout/lost-target failsafes stay the
    backstop)."""
    if force_frac is None or flown_m is None or planned_m is None or planned_m <= 0.0:
        return False
    return flown_m >= force_frac * planned_m


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
        # The belief latched from the live cue at the GO edge (cfg.cue_go).
        # Same rule, same audit: exactly one call site writes it.
        self.cue_belief = LatchOnce("cue_belief")
        self._had_track = False            # the terminal has reached Phase B
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
        self._recede_start_range: Optional[float] = None   # base of the rising run
        self._breakoff_held_logged = False   # "count met, magnitude short" said once
        # FAILSAFE 7: when `offboard_active` first stopped reading True (None =
        # it is reading True right now). Persistence, not a single bad tick.
        self._offboard_bad_since: Optional[float] = None
        self.max_offboard_sample_age_s: float = 0.0        # evidence for the log
        self.max_own_state_age_s: float = 0.0              # FAILSAFE 9 evidence
        self._broken_streak = 0            # FAILSAFE 8 persistence counter
        # Own displacement since the dash began -- ONE running integrator that
        # feeds BOTH FAILSAFE 4's optional distance bound (CODED_DASH) and the
        # ADR-0101 passage window (ENGAGE); see `_flown_since_dash_m`.
        self._flown_m = 0.0
        self._gs_last_t: Optional[float] = None
        self._gs_last_v: float = 0.0
        self._last_r_hat_m: Optional[float] = None  # pursuit miss-policy input
        self._rehearsal_logged = False     # rehearsal trigger said once
        self._rehearsal_too_late_logged = False   # too-late floor said once
        self._warned_own_state = False
        self.last_telemetry: Optional[StepTelemetry] = None  # for the CSV row
        # AGL-floor bookkeeping (ADR-0085's half, review F3). `_last_alt_m` is the
        # altitude the CURRENT tick reported, kept so the floor can be applied at
        # `_decide` -- the one exit every state funnels through, including the
        # states that never build a command from a reference.
        self._last_obs_t: Optional[float] = None
        self._last_alt_m: Optional[float] = None
        self._last_alt_ref_m: Optional[float] = None
        self._floor_warned = False         # the clamp is announced once, not per tick
        self._floor_active = False         # exposed so a caller/CSV can log that it bit
        # battery rule (MissionConfig.battery_low_frac)
        self._batt_low_since: Optional[float] = None
        self.battery_low = False           # low for the hold time, this tick
        self._batt_ignored_noted = False
        self._batt_hover_noted = False
        self._validate_vertical_config()
        self._validate_passage_config()
        self._validate_safe_behavior()

    def _validate_passage_config(self) -> None:
        """ADR-0101 passage window: refuse an incoherent floor/ceiling at
        CONSTRUCTION, `_validate_vertical_config`-style -- a guidance constant
        that cannot be justified from a measurement is the failure this
        project has a rule about."""
        c = self.cfg
        for name, frac in (("breakoff_min_flown_frac", c.breakoff_min_flown_frac),
                           ("breakoff_force_flown_frac", c.breakoff_force_flown_frac)):
            if frac is None:
                continue
            if frac != frac:
                raise ValueError(f"{name} is NaN")
            if frac <= 0.0:
                raise ValueError(
                    f"{name}={frac} must be > 0 -- a non-positive fraction of "
                    "the plan distance is never satisfiable (the floor) or "
                    "fires on the very first tick (the ceiling), neither of "
                    "which is a coherent passage window.")
        if c.breakoff_min_flown_frac is not None \
                and c.breakoff_force_flown_frac is not None \
                and c.breakoff_min_flown_frac >= c.breakoff_force_flown_frac:
            raise ValueError(
                f"breakoff_min_flown_frac={c.breakoff_min_flown_frac} >= "
                f"breakoff_force_flown_frac={c.breakoff_force_flown_frac}: the "
                "floor that ARMS the past-CPA breakoff cannot sit at or past "
                "the ceiling that FORCES it -- the window would be empty or "
                "inverted (the breakoff could never legally fire).")
        p = c.pursuit_miss_range_m
        if p != p or p < 0.0:
            raise ValueError(f"pursuit_miss_range_m={p} must be a "
                             "non-negative number")

    def _validate_safe_behavior(self) -> None:
        """`MissionConfig.safe_behavior`/`miss_safe_behavior` are read by
        string in both the state machine and the live driver (which command it
        issues after SAFE) -- an unrecognised value must fail LOUD at
        construction, not silently fall through to whichever branch a stale
        if/elif happens to match."""
        for name in ("safe_behavior", "miss_safe_behavior"):
            v = getattr(self.cfg, name)
            if v not in ("land", "hover", "rtl"):
                raise ValueError(
                    f"{name}={v!r} must be one of 'land' / 'hover' / 'rtl'")

    # SAFE entries whose CAUSE is a missed/ended engagement, not a sick
    # system -- these take `miss_safe_behavior` (ADR-0107: hover by default).
    # `rehearsal_breakoff` (2026-09-24, isim/specs/rehearsal_breakoff_prereg_
    # 2026-09-24.md) is a deliberate practice break-off -- an ended
    # engagement, so it hovers ready for the next pass like a miss.
    MISS_SAFE_REASONS = ("pursuit_miss", "breakoff_complete", "rehearsal_breakoff")

    def effective_safe_behavior(self) -> str:
        """Which of the two configured SAFE behaviours applies RIGHT NOW.
        Public + pure so the state machine (`_decide`), both drivers, and the
        tests all ask the same one question of the same one place."""
        if self.safe_reason in self.MISS_SAFE_REASONS:
            return self.cfg.miss_safe_behavior
        return self.cfg.safe_behavior

    def _validate_vertical_config(self) -> None:
        """Refuse an incoherent vertical config at CONSTRUCTION, not at 200 m.

        `MissionConfig` had no validator at all (review finding F11), so three
        incoherent combinations were silently accepted. Each raises here with the
        arithmetic in the message, because a guidance constant that cannot be
        justified from a measurement is the failure this project has a rule about.
        """
        c = self.cfg
        t = c.dash_alt_trim_m
        if t != t:
            raise ValueError("dash_alt_trim_m is NaN")
        # DELIBERATELY NOT a rule about `alt_tol_m`. The first version of this
        # validator refused |trim| >= alt_tol_m, which reads plausibly and is
        # wrong: it ties the trim to the ARM GATE's tolerance, two unrelated
        # quantities, and it rejected -0.320 m -- the exact value ADR-0099 derives.
        # A validator that refuses the project's own documented experiment is a
        # worse defect than the one it was guarding. The gate coupling was a real
        # bug (review F1) and it was fixed STRUCTURALLY: the trim no longer touches
        # the standby hold at all, so there is nothing left for a magnitude rule to
        # protect. What remains is genuine incoherence only.
        if abs(t) >= c.dash_base_alt_m:
            raise ValueError(
                f"dash_alt_trim_m={t:+.3f} m is >= dash_base_alt_m="
                f"{c.dash_base_alt_m} m, which would drive the dash reference to "
                f"or below zero. That is a mis-derived drift, not a trim.")
        if c.min_agl_m is not None:
            f = c.min_agl_m
            if f != f:
                raise ValueError("min_agl_m is NaN -- a floor that cannot be "
                                 "compared is not a floor")
            if f >= c.standby_alt_m:
                raise ValueError(
                    f"min_agl_m={f} m is at or above standby_alt_m="
                    f"{c.standby_alt_m} m, so the vehicle can never satisfy the "
                    f"arm gate and hold above the floor at the same time.")
            if f >= c.dash_base_alt_m:
                raise ValueError(
                    f"min_agl_m={f} m is at or above dash_base_alt_m="
                    f"{c.dash_base_alt_m} m: the floor would silently truncate the "
                    f"dash profile the whole engagement is aimed with.")

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

    def offboard_status(self, obs: VehicleObs) -> Tuple[bool, str]:
        """FAILSAFE 7 input (builder decision #22). Returns (ok, reason).

        POLICY -- and note it is the OPPOSITE of link_status() on purpose:

          * the RF LINK is monitored only BEFORE the GO edge (post-GO loss is the
            jam-resistance deliverable, so it is ignored);
          * the OFFBOARD CONTROL PATH is monitored only AFTER it (before the GO
            edge the arm gate already refuses to dash without it).

        Two arms, both -> SAFE, both conservative:
          ARM 1  the vehicle is NOT in OFFBOARD (`offboard_active is not True`)
                 for `offboard_lost_s` continuously. Our setpoints are not being
                 flown; and because SAFE absorbs, handing OFFBOARD back cannot
                 resume a half-ramped dash.
          ARM 2  the flight-mode SAMPLE is older than `offboard_stale_s` (or has
                 a physically impossible NEGATIVE age -> clock-base fault). A
                 dead subscriber freezes `offboard_active` at True and would make
                 arm 1 structurally unreachable, which is precisely the
                 silent-failure shape this project keeps retracting results over.

        Pure + public so the driver, the tests and a log reader can all ask it."""
        cfg = self.cfg
        if not cfg.require_offboard_after_go:
            return True, "disabled"

        age = obs.offboard_sample_age_s
        if age is not None:
            self.max_offboard_sample_age_s = max(self.max_offboard_sample_age_s,
                                                 float(age))

        # ARM 1 -- not in OFFBOARD, with persistence.
        if obs.offboard_active is not True:
            if self._offboard_bad_since is None:
                self._offboard_bad_since = obs.t
            held = obs.t - self._offboard_bad_since
            if held >= cfg.offboard_lost_s:
                return False, (f"mode={obs.mode or obs.offboard_active} for "
                               f"{held:.2f}s >= {cfg.offboard_lost_s:.2f}s")
            return True, "transient"
        self._offboard_bad_since = None

        # ARM 2 -- the sample that says "OFFBOARD" is stale (or impossible).
        if age is not None:
            if age < 0.0:
                return False, (f"mode_sample_clock_fault(age={age:.2f}s < 0 -- "
                               "producer and consumer are on different clocks)")
            if cfg.offboard_stale_s is not None and age > cfg.offboard_stale_s:
                return False, (f"mode_sample_stale({age:.2f}s > "
                               f"{cfg.offboard_stale_s:.2f}s -- offboard_active "
                               "is a FROZEN value, not a live one)")
        return True, "ok"

    def own_state_status_sm(self, obs: VehicleObs) -> Tuple[bool, str]:
        """FAILSAFE 9 input. Returns (ok, reason). Pure + public, like
        offboard_status(). Inactive (always ok) when the driver supplies no age
        or the bound is None."""
        age = obs.own_state_age_s
        if age is None:
            return True, "not_instrumented"
        self.max_own_state_age_s = max(self.max_own_state_age_s, float(age))
        bound = self.cfg.own_state_stale_s
        if bound is None:
            return True, "disabled"
        if age < 0.0:
            return False, (f"age={age:.2f}s < 0 -- stamp and reader are on "
                           "different clocks")
        if age > bound:
            return False, f"oldest stream {age:.2f}s > {bound:.2f}s"
        return True, "ok"

    def _update_battery(self, obs: VehicleObs) -> bool:
        """True when the charge has been below `battery_low_frac` for
        `battery_low_hold_s`. Inactive (False) with no threshold or no reading."""
        thr = self.cfg.battery_low_frac
        frac = obs.battery_frac
        if thr is None or frac is None or not math.isfinite(float(frac)) \
                or float(frac) >= float(thr):
            self._batt_low_since = None
            self.battery_low = False
            return False
        if self._batt_low_since is None:
            self._batt_low_since = obs.t
        self.battery_low = (obs.t - self._batt_low_since) \
            >= self.cfg.battery_low_hold_s
        return self.battery_low

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
        # The terminal's whole LOS chain hangs on the attitude QUATERNION, and
        # with it absent seeker_loop reverts to the pre-fix yaw-only formula on an
        # assumed-level vehicle. The gate already refuses to dash on `no_yaw`;
        # quat is the SAME own-state EKF (review2 HIGH). One line converts a
        # silent revert-to-pre-fix into a loud pre-dash refusal on the ground.
        if cfg.require_attitude_quat and obs.quat is None:
            fails.append("no_attitude_quat")
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
        """Altitude-hold velocity toward `alt_ref`. Byte-identical to the
        pre-2026-09-10 form: the caller passes whatever reference it wants and
        NOTHING is applied here.

        The trim used to be applied here and that was WRONG in two ways (review
        findings F1/F7): it biased the STANDBY hold, which the arm gate then
        compared against the UNTRIMMED config value, so a trim larger than
        `alt_tol_m` = 0.3 m failed the gate at GO and dropped the mission into
        absorbing SAFE -- at the very value ADR-0099 derives, 0.320 m. The trim is
        a DASH reference correction and now only the dash and terminal references
        carry it. The FLOOR moved to the emitted command instead (`_decide`),
        because three states never build a command from a reference at all."""
        self._last_alt_ref_m = alt_ref        # for the CSV column
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
            alt_ref  = apply_alt_ref_trim(alt_ref, cfg.dash_alt_trim_m, None)
            v_N, v_E = v*cos(heading), v*sin(heading)     (compass azimuth)
            yaw      = heading   (nose on the dash line, so the camera looks
                                  where the aircraft is going)

        Own-state only: altitude for the dive reference. No camera, no cue, no gt."""
        cfg = self.cfg
        elapsed = 0.0 if self.t_dash_start is None else obs.t - self.t_dash_start
        v = dash_forward_speed(cfg.dash_speed_ms, cfg.dash_accel_cap_ms2, elapsed)
        alt_ref = dash_loft_alt_ref(cfg.dash_base_alt_m, cfg.dash_loft_m,
                                    elapsed, cfg.dash_loft_dive_s)
        alt_ref = apply_alt_ref_trim(alt_ref, cfg.dash_alt_trim_m, None)
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
                        self._v_down(obs, apply_alt_ref_trim(
                            self.cfg.dash_base_alt_m,
                            self.cfg.dash_alt_trim_m, None)),
                        self._last_yaw_cmd_deg)

    def _flown_since_dash_m(self, obs: VehicleObs, elapsed_since_dash: float
                            ) -> Tuple[float, str]:
        """Own displacement (m) since the dash began. ONE running integrator
        that feeds BOTH FAILSAFE 4's optional distance bound (`_step_coded_
        dash`) and the ADR-0101 passage window (`_step_engage`), so the two
        measurements can never disagree or double-count -- call this AT MOST
        ONCE per tick.

        PREFERS THE MEASUREMENT, exactly like FAILSAFE 4 already did:
        trapezoid-integrates the EKF ground speed when the vehicle reports
        it, and only dead-reckons off the dash's own kinematic model when it
        does not -- and SAYS so in the returned source tag ("EKF"/
        "MODELLED"), so no log reader can mistake a modelled metre for a
        flown one.

        DIVERGENCE FROM THE SIM TWIN, disclosed (not hidden): m4's passage
        functions (`scripts/m4_intercept.py`) read the own EKF's HORIZONTAL
        POSITION and take a straight-line hypot from the dash-entry point --
        exact through a turn. This state machine's `VehicleObs` carries no
        horizontal position (own-state EKF exposes only a ground-speed
        SCALAR here), so this integrates SPEED instead, which is exact on
        the straight coded-dash leg and an OVERESTIMATE on any curved path (a
        turn keeps adding speed*dt while displacement grows more slowly).
        Acceptable because both consumers are gated `not cfg.pursuit_mode` --
        the one path that actually turns (the pursuit chase) never reaches
        either. TODO-BUILDER if that gate is ever loosened for a turning
        stock/tag profile: wire `VehicleObs.vel_ned` into a real 2-D running
        displacement instead of a speed integral."""
        if obs.ground_speed_ms is not None:
            if self._gs_last_t is not None:
                self._flown_m += 0.5 * (obs.ground_speed_ms + self._gs_last_v) \
                    * max(0.0, obs.t - self._gs_last_t)
            self._gs_last_t, self._gs_last_v = obs.t, obs.ground_speed_ms
            return self._flown_m, "EKF"
        cfg = self.cfg
        return dash_ramp_distance(
            cfg.dash_speed_ms, cfg.dash_accel_cap_ms2 or cfg.dash_accel_ms2,
            elapsed_since_dash), "MODELLED"

    # ---------------------------------------------------------------- step

    def step(self, obs: VehicleObs) -> Decision:
        events: List[str] = []
        self._last_alt_m = obs.alt_m          # the floor at `_decide` reads this
        if self.t0 is None:
            self.t0 = obs.t
            self.t_state = obs.t
            self._emit(f"[{obs.t:7.2f}s] START in {self.state} "
                       f"(aim yaw {self.cfg.aim_yaw_deg:.1f} deg, standby alt "
                       f"{self.cfg.standby_alt_m:.1f} m)", events)
        before = self.state
        transition_before = len(self.transitions)

        # BATTERY (MissionConfig.battery_low_frac): acted on in STANDBY and in
        # SAFE, and deliberately NOT between GO and the end of the engagement.
        if self._update_battery(obs) and not self._batt_ignored_noted \
                and self.state in (State.CODED_DASH, State.ENGAGE,
                                   State.BREAKOFF):
            self._batt_ignored_noted = True
            self._emit(f"[{obs.t:7.2f}s] battery reads low "
                       f"({obs.battery_frac:.0%} left) in {self.state} -- "
                       f"IGNORED after GO by the builder's ruling; the "
                       f"intercept goes on", events)

        # ABSOLUTE BACKSTOP: no state except SAFE may outlive the mission bound.
        if self.state != State.SAFE and (obs.t - self.t0) > self.cfg.mission_max_s:
            self._transition(State.SAFE,
                             f"mission_timeout({self.cfg.mission_max_s:.0f}s)",
                             obs, events)
            self.safe_reason = "mission_timeout"

        # FAILSAFE 7 -- POST-GO OFFBOARD LOSS (builder decision #22). Checked for
        # every post-GO state, BEFORE the state body, so no tick of dash/terminal
        # logic runs on a flight the vehicle is not actually flying. STANDBY is
        # excluded on purpose: there the arm gate already refuses to dash without
        # OFFBOARD, and a pre-GO blip should cost a GO, not the aircraft.
        if self.state in (State.CODED_DASH, State.ENGAGE, State.BREAKOFF):
            offb_ok, offb_why = self.offboard_status(obs)
            if not offb_ok:
                self._last_obs_t = obs.t
                self._transition(State.SAFE, f"offboard_lost({offb_why})",
                                 obs, events)
                self.safe_reason = "offboard_lost"
                self._emit(f"[{obs.t:7.2f}s] FAILSAFE 7: the OFFBOARD control "
                           f"path is gone after the GO edge -- SAFE is absorbing, "
                           f"so the vehicle gets a ZERO-velocity hold + land "
                           f"request if control returns, NOT the dash command",
                           events)
                return self._decide(before, transition_before,
                                    self._safe_setpoint(obs), events, None)

        # FAILSAFE 9 -- STALE OWN-STATE (MissionConfig.own_state_stale_s). Every
        # state but SAFE, STANDBY included: a frozen height in STANDBY drives
        # the height loop, which is as bad as a frozen attitude in ENGAGE.
        # Checked before the state body so no tick steers on numbers known to
        # be old. A NEGATIVE age is a clock-base fault and is treated as loss,
        # the same rule as the RC and flight-mode ages.
        if self.state != State.SAFE:
            own_ok, own_why = self.own_state_status_sm(obs)
            if not own_ok:
                self._last_obs_t = obs.t
                self._transition(State.SAFE, f"own_state_stale({own_why})",
                                 obs, events)
                self.safe_reason = "own_state_stale"
                self._emit(f"[{obs.t:7.2f}s] FAILSAFE 9: own-state telemetry is "
                           f"stale ({own_why}) -- an attitude, height or "
                           f"velocity stream has stopped and its last value is "
                           f"frozen. SAFE: zero-velocity hold + land request.",
                           events)
                return self._decide(before, transition_before,
                                    self._safe_setpoint(obs), events, None)

        # FAILSAFE 10 -- DEAD CAMERA in ENGAGE (MissionConfig.camera_dead_s).
        if self.state == State.ENGAGE and self.cfg.camera_dead_s is not None \
                and obs.camera_age_s is not None \
                and obs.camera_age_s > self.cfg.camera_dead_s:
            self._last_obs_t = obs.t
            self._transition(
                State.SAFE, f"camera_dead(no frame for {obs.camera_age_s:.1f}s "
                            f"> {self.cfg.camera_dead_s:.1f}s)", obs, events)
            self.safe_reason = "camera_dead"
            self._emit(f"[{obs.t:7.2f}s] FAILSAFE 10: the camera has delivered "
                       f"no frame for {obs.camera_age_s:.1f} s -- a camera-only "
                       f"chase cannot go on without it. SAFE.", events)
            return self._decide(before, transition_before,
                                self._safe_setpoint(obs), events, None)

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
        # ADR-0085's HARD FLOOR, applied to the EMITTED command (review F3/F4).
        # It used to live in `_v_down`, which three states never call: ENGAGE
        # returns the SEEKER's own setpoint, and BREAKOFF and SAFE emit hardcoded
        # vertical rates. So the floor was missing from exactly the state ADR-0085
        # wrote it for -- "below which the vehicle will not descend regardless of
        # the seeker" -- and a probe showed ENGAGE commanding +2.0 m/s down while
        # 2 m UNDER its own floor. Here it covers every state, including a setpoint
        # this module never built. Byte-identical when `min_agl_m is None`.
        obs_t_for_log = self._last_obs_t if self._last_obs_t is not None else 0.0
        # RESET FIRST, so the CSV column cannot report a STALE True. It was set
        # only inside the branch below, so after a tick where the floor bit, any
        # later tick with `sp is None` (or with the floor disabled) kept
        # reporting floor_active=1 -- in the very column that was added for
        # observability. A flag that lies about the current tick is worse than no
        # flag (review finding S11).
        self._floor_active = False
        # LAST GUARD (review 2026-11-26): no non-finite number may leave this
        # function as a velocity command. There was no such check anywhere
        # between the terminal and the Pixhawk. A NaN command is a fault in
        # this program, not a degraded sensor, so it goes SAFE.
        if sp is not None and not _all_finite(sp.as_tuple()):
            bad = sp.as_tuple()
            if not math.isfinite(float(self._last_yaw_cmd_deg)):
                self._last_yaw_cmd_deg = self.cfg.aim_yaw_deg
            sp = Setpoint(0.0, 0.0, 0.0, self._last_yaw_cmd_deg)
            if self.state != State.SAFE:
                tr_obs = VehicleObs(t=obs_t_for_log)
                self._transition(State.SAFE, f"nonfinite_setpoint{bad}",
                                 tr_obs, events)
                self.safe_reason = "nonfinite_setpoint"
            tel = None
        if sp is not None and self.cfg.min_agl_m is not None:
            v_down, floored = floor_v_down(
                sp.v_down, self._last_alt_m, self.cfg.min_agl_m,
                self.cfg.kp_alt, self.cfg.v_vert_max_ms)
            if floored:
                sp = Setpoint(sp.v_north, sp.v_east, v_down, sp.yaw_deg)
                if not self._floor_warned:
                    self._floor_warned = True
                    self._emit(f"[{obs_t_for_log:7.2f}s] AGL FLOOR: refusing "
                               f"descent at {self._last_alt_m} m (floor "
                               f"{self.cfg.min_agl_m} m) -- clamping v_down in "
                               f"{self.state}. Logged once.", events)
            self._floor_active = floored
        tr = (self.transitions[-1]
              if len(self.transitions) > transition_before else None)
        # SAFE behavior (builder ruling 2026-09-22). 'land'/'rtl' are
        # BYTE-IDENTICAL at this layer to the pre-existing behaviour --
        # land_requested goes True the INSTANT SAFE is entered, terminated
        # follows safe_hold_s later; 'rtl' differs only in which PX4 action
        # the LIVE DRIVER issues once those fire (see run_mavsdk_mission).
        # 'hover' withholds BOTH indefinitely -- the setpoint stream above
        # still emits SAFE's zero-velocity hold every tick regardless -- until
        # the mission_max_s ABSOLUTE BACKSTOP (measured from mission start,
        # t0, not from SAFE entry) fires, at which point it lands like any
        # other SAFE.
        terminated = False
        land_requested = False
        if self.state == State.SAFE and self.t_state is not None:
            if self.effective_safe_behavior() == "hover":
                past_backstop = (self.t0 is not None and
                                 (self._last_obs_t - self.t0) >= self.cfg.mission_max_s)
                # A hover ends in a landing on a low pack (the Pixhawk's own
                # low-battery action is "warning only" on a program flight).
                if self.battery_low and not self._batt_hover_noted:
                    self._batt_hover_noted = True
                    self._emit(f"[{self._last_obs_t:7.2f}s] battery low in "
                               f"the hover -- landing", events)
                terminated = past_backstop or self.battery_low
                land_requested = terminated
            else:
                land_requested = True
                terminated = (self._last_obs_t - self.t_state) >= self.cfg.safe_hold_s
        return Decision(state=self.state, setpoint=sp, events=events,
                        transition=tr, streak=self.streak,
                        land_requested=land_requested,
                        terminated=terminated, safe_reason=self.safe_reason,
                        telemetry=tel, alt_ref_m=self._last_alt_ref_m,
                        floor_active=self._floor_active)

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

        # FAILSAFE 2b -- a low pack in the standby hover: land, never GO.
        if self.battery_low:
            self._transition(
                State.SAFE,
                f"battery_low_before_go({obs.battery_frac:.0%} left, below "
                f"{cfg.battery_low_frac:.0%} for {cfg.battery_low_hold_s:.0f}s)",
                obs, events)
            self.safe_reason = "battery_low_before_go"
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
            if cfg.cue_go:
                # THE CUE IS THE GO COMMAND: the belief arrives on this tick or
                # there is no GO. Fail closed, loudly -- never a dash on the
                # placeholder belief the terminal was built with.
                hook = getattr(self.guidance, "set_belief_once", None)
                why = None
                if obs.cue_belief is None:
                    why = "cue_go_without_belief"
                elif hook is None:
                    why = "cue_go_terminal_cannot_take_belief"
                if why is not None:
                    self._transition(State.SAFE, why, obs, events)
                    self.safe_reason = why
                    return self._safe_setpoint(obs)
                self.cue_belief.set(obs.cue_belief, t=obs.t)
                r0, v0 = self.cue_belief.value
                hook(r0, v0, obs.t)
                self._emit(f"[{obs.t:7.2f}s] GO edge: cue belief LATCHED -- target "
                           f"{math.hypot(r0[0], r0[1]):.1f} m away (north "
                           f"{r0[0]:+.1f}, east {r0[1]:+.1f}, down {r0[2]:+.1f}) "
                           f"moving {math.hypot(v0[0], v0[1]):.2f} m/s -- "
                           "immutable for the rest of the flight; the link is "
                           "now closed", events)
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
            # ADR-0103 "chase only": pursuit flies the WHOLE approach itself
            # (Phase A belief rendezvous needs no detection), so GO enters
            # ENGAGE directly -- the acquire-streak DASH gate is a sprint-era
            # mechanism a rear-aspect chase can never satisfy from standby.
            # t_dash_start is still latched above: run_offline's stub seeker
            # and the log tooling key off it.
            if cfg.pursuit_mode:
                self.t_engage_start = obs.t
                self._transition(State.ENGAGE, "go_edge_pursuit(chase_only)",
                                 obs, events)
                # The GO tick itself flies the standby hold; ENGAGE takes over
                # next tick -- the same transition-tick ordering the coded dash
                # uses (see _step_coded_dash's handoff comment).
                return self._standby_setpoint(obs)
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

        # Own displacement since the dash began -- ONE running integrator
        # (PREFERS THE MEASUREMENT: trapezoid-integrates EKF ground speed when
        # available, else dead-reckons off the dash model and SAYS so in the
        # source tag) that feeds BOTH FAILSAFE 4 below AND the ADR-0101
        # passage window evaluated in `_step_engage`. Updated every
        # CODED_DASH tick regardless of whether the distance bound is armed,
        # so the integrator/dead-reckon state is already current the moment
        # ENGAGE starts.
        flown, src = self._flown_since_dash_m(obs, elapsed)

        # FAILSAFE 4 -- optional dead-reckoned DISTANCE bound (own dash model, no
        # position read needed). Off by default; size it to the range box.
        if cfg.dash_max_dist_m is not None and flown > cfg.dash_max_dist_m:
            self._transition(State.SAFE,
                             f"dash_distance_bound({src} {flown:.1f} m > "
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
            if getattr(self.guidance, "SUPPORTS_POSE_RANGE", False):
                # Pursuit-scoped pose-range channel (tick-trace S2 fix): the
                # kwarg is passed ONLY to a terminal that declares it, so the
                # duck-typed 3-argument contract of SeekerGuidance/
                # TagInterceptGuidance is untouched. The pose range travels
                # only alongside a box the terminal will actually consume.
                kw = {}
                if getattr(self.guidance, "SUPPORTS_STAMPED_AGE", False):
                    kw["det_age_s"] = (obs.det_age_s if box is not None
                                       else None)
                sp, tel = self.guidance.step(
                    box, obs.own_state(), obs.t,
                    det_range_pose_m=(obs.det_range_pose_m
                                      if box is not None else None), **kw)
            else:
                sp, tel = self.guidance.step(box, obs.own_state(), obs.t)
        if sp is not None:
            self._last_yaw_cmd_deg = sp.yaw_deg
        # PURSUIT-MODE MISS POLICY input (builder ruling 2026-09-22,
        # MissionConfig.pursuit_miss_range_m's docstring): the last KF range
        # estimate. `pursuit_terminal.PursuitTerminalGuidance.step` sets
        # `tel.r_hat_m` every tick Phase B is active -- predicted/coasted,
        # not just on a fresh decode -- so this tracks the CURRENT best
        # estimate continuously, not a value frozen at the last detection.
        # Read by FAILSAFE 5 below.
        if tel is not None and tel.r_hat_m is not None:
            self._last_r_hat_m = tel.r_hat_m

        # REHEARSAL BREAK-OFF (2026-09-24, isim/specs/rehearsal_breakoff_
        # prereg_2026-09-24.md; flight.pursuit_terminal's
        # PursuitTerminalConfig.rehearsal_breakoff docstring). Duck-typed:
        # only a terminal that carries `rehearsal_trigger_t` can report one,
        # so every other terminal -- and pursuit with the flag off, where it
        # stays None -- is untouched. The trigger instant, range and the
        # onboard would-have ZEM go into the mission log once; when the
        # terminal's timed evade is done the engagement ends through SAFE
        # with its own reason (MISS-class: hover, ADR-0107).
        rehearsal_t = getattr(self.guidance, "rehearsal_trigger_t", None) \
            if self.guidance is not None else None
        if rehearsal_t is not None:
            g = self.guidance
            if not self._rehearsal_logged:
                self._rehearsal_logged = True
                self._emit(
                    f"[{obs.t:7.2f}s] REHEARSAL break-off triggered at "
                    f"t={rehearsal_t:.2f}s (r_hat={g.rehearsal_trigger_range_m:.2f} m, "
                    f"onboard would-have ZEM={g.rehearsal_zem_m:.2f} m, "
                    f"t_go={g.rehearsal_t_go_s:.2f} s, evade {g.rehearsal_side}) "
                    f"-- evading", events)
            if getattr(g, "rehearsal_complete", False) is True:
                self._transition(
                    State.SAFE,
                    f"rehearsal_breakoff(trigger t={rehearsal_t:.2f}s "
                    f"r_hat={g.rehearsal_trigger_range_m:.2f} m "
                    f"zem={g.rehearsal_zem_m:.2f} m, evade "
                    f"{g.cfg.rehearsal_evade_s:.1f}s {g.rehearsal_side})",
                    obs, events)
                self.safe_reason = "rehearsal_breakoff"
                return self._safe_setpoint(obs), tel
        elif self.guidance is not None and \
                getattr(self.guidance, "rehearsal_too_late", False) is True \
                and not self._rehearsal_too_late_logged:
            # Amendment #1 TOO-LATE FLOOR (same prereg doc): the terminal
            # latched `rehearsal_too_late` -- its first eligible t_go was
            # already inside rehearsal_t_late_s -- and will not break off this
            # flight; the normal engagement end decides. Logged once; no
            # state change here.
            g = self.guidance
            self._rehearsal_too_late_logged = True
            self._emit(
                f"[{obs.t:7.2f}s] REHEARSAL too late at "
                f"t={g.rehearsal_too_late_t:.2f}s (t_go="
                f"{g.rehearsal_too_late_t_go_s:.2f} s < "
                f"{g.cfg.rehearsal_t_late_s:.2f} s, r_hat="
                f"{g.rehearsal_too_late_range_m:.2f} m) -- NOT breaking off; "
                f"this pass ends through the normal engagement end", events)

        # FAILSAFE 8 -- the terminal's RANGE CHANNEL DIVERGED (review2 BLOCKER).
        # r_hat <= 0 or a non-physical |rdot_hat| is proof the estimate is not the
        # target any more. Pre-fix that state silently selected the terminal-coast
        # branch and flew a frozen vector to the end of the engagement.
        #   * PERSISTENCE: one bad tick may not end an engagement.
        #   * NOT DURING AN ARMED COAST: inside the freeze the range estimate is
        #     SUPPOSED to run through zero -- that is the coast flying through CPA
        #     (ADR-0023), and the coast can only arm off a sane track anyway.
        #     Aborting there would delete the validated terminal mechanism.
        broken_now = (tel is not None and getattr(tel, "track_broken", False)
                      and not getattr(tel, "terminal_coast", False))
        if broken_now:
            self._broken_streak += 1
        elif tel is not None:
            self._broken_streak = 0
        if self._broken_streak >= max(1, cfg.engage_track_broken_ticks):
            self._transition(State.BREAKOFF,
                             f"range_track_broken({self._broken_streak} ticks)",
                             obs, events)
            return self._breakoff_setpoint(obs), tel
        # Own-state loss inside the terminal: the seeker refuses to steer (returns
        # None) and we coast the last dash velocity. Say so once -- an unremarked
        # coast is indistinguishable from guidance in the log.
        if tel is not None and not getattr(tel, "own_state_ok", True) \
                and not self._warned_own_state:
            self._warned_own_state = True
            self._emit(f"[{obs.t:7.2f}s] WARNING: own-state EKF unavailable in "
                       f"ENGAGE ({','.join(tel.health)}) -- terminal is HOLDING, "
                       f"not guiding", events)

        # ADR-0101 PASSAGE CEILING (ported from scripts/m4_intercept.py
        # `passage_ceiling_hit`; that ADR's own disclosed open item --
        # "real_flight.py still carries the old rule ... port deliberately,
        # not silently"). Evaluated EVERY ENGAGE tick, detected or not, on
        # OWN-STATE ONLY -- a camera dropout must not be able to hide it.
        # Once the vehicle's own displacement since the dash began reaches
        # breakoff_force_flown_frac x the pre-flight PLAN distance it MUST
        # have passed the target, whatever the camera says. Inert in
        # pursuit_mode (no coded dash, no plan distance -- and the trigger
        # this window governs is already suppressed there, ADR-0103), and
        # inert with no `t_dash_start` (should not happen outside
        # pursuit_mode, but fails closed rather than crashing if it ever
        # does).
        flown_since_dash: Optional[float] = None
        flown_src: Optional[str] = None
        if not cfg.pursuit_mode and self.t_dash_start is not None:
            flown_since_dash, flown_src = self._flown_since_dash_m(
                obs, obs.t - self.t_dash_start)
            if cfg.breakoff_force_flown_frac is not None and passage_ceiling_hit(
                    flown_since_dash, cfg.dash_plan_m, cfg.breakoff_force_flown_frac):
                self._transition(
                    State.BREAKOFF,
                    f"passage_ceiling(flown {flown_since_dash:.2f} m "
                    f"[{flown_src}] >= {cfg.breakoff_force_flown_frac} x "
                    f"planned {cfg.dash_plan_m:.2f} m -- must be past "
                    "closest approach)", obs, events)
                return self._breakoff_setpoint(obs), tel

        if obs.det_new and obs.det_range_m is not None:
            self._last_det_t = obs.t
            r = obs.det_range_m
            if r < cfg.breakoff_arm_range_m:
                self._breakoff_armed = True
            (self._recede_streak, self._recede_last_range,
             self._recede_start_range) = update_range_increase_streak(
                self._recede_streak, self._recede_last_range,
                self._recede_start_range, r, cfg.breakoff_range_deadband_m)
            if r <= cfg.breakoff_hard_floor_m:
                self._transition(State.BREAKOFF, f"hard_floor({r:.2f} m)",
                                 obs, events)
                return self._breakoff_setpoint(obs), tel
            # PAST-CPA TRIGGER = COUNT **and** MAGNITUDE (ported from
            # scripts/m4_intercept.py, audit reader 1). The count alone fires on
            # N tiny rises -- a slow drift of the range estimate reads exactly
            # like a real recession. `rise` is the total climb from the BASE of
            # the current rising run and must clear `breakoff_min_rise_m`.
            rise = (None if self._recede_start_range is None
                    else r - self._recede_start_range)
            rise_ok = (rise is None or rise >= cfg.breakoff_min_rise_m)
            range_ok = (cfg.breakoff_max_range_m is None
                        or r <= cfg.breakoff_max_range_m)
            # ADR-0101 PASSAGE GATE (the FLOOR half of the passage window --
            # the ceiling was already checked above, every tick). Ported from
            # scripts/m4_intercept.py `passage_gate_ok`: the breakoff may not
            # fire until the vehicle's own displacement since the dash began
            # reaches breakoff_min_flown_frac x the plan distance. None =
            # gate OFF (byte-identical).
            passage_ok = passage_gate_ok(flown_since_dash, cfg.dash_plan_m,
                                         cfg.breakoff_min_flown_frac)
            # ADR-0103 "chase only": pursuit_mode suppresses ONLY this
            # recession trigger (see MissionConfig.pursuit_mode's docstring)
            # -- hard_floor above and the failsafes below stay active.
            if not cfg.pursuit_mode and self._breakoff_armed and \
                    self._recede_streak >= cfg.breakoff_range_increases:
                if rise_ok and range_ok and passage_ok:
                    self._transition(
                        State.BREAKOFF,
                        f"past_cpa({self._recede_streak} rising, "
                        f"+{0.0 if rise is None else rise:.2f} m from "
                        f"{'?' if self._recede_start_range is None else f'{self._recede_start_range:.2f}'} m"
                        f" >= min_rise {cfg.breakoff_min_rise_m:.2f} m)",
                        obs, events)
                    return self._breakoff_setpoint(obs), tel
                # HELD, and SAID SO: a suppressed breakoff that is invisible in
                # the log is indistinguishable from one that never armed. Once
                # per rising run (reset below when the run breaks), not per tick.
                if not self._breakoff_held_logged:
                    self._breakoff_held_logged = True
                    if not rise_ok:
                        why = (
                            f"rise +{0.0 if rise is None else rise:.2f} m < "
                            f"min_rise {cfg.breakoff_min_rise_m:.2f} m (drift, "
                            "not recession)")
                    elif not range_ok:
                        why = (f"range {r:.2f} m > breakoff_max_range "
                              f"{cfg.breakoff_max_range_m}")
                    else:
                        why = (
                            f"flown "
                            f"{'?' if flown_since_dash is None else f'{flown_since_dash:.2f}'}"
                            f" m < {cfg.breakoff_min_flown_frac} x planned "
                            f"{'?' if cfg.dash_plan_m is None else f'{cfg.dash_plan_m:.2f}'}"
                            " m (ADR-0101 passage gate: not yet flown far "
                            "enough to have passed it)")
                    self._emit(f"[{obs.t:7.2f}s] past-CPA count met "
                               f"({self._recede_streak} rising) but HELD: {why}",
                               events)
            elif self._recede_streak == 0:
                self._breakoff_held_logged = False   # the rising run broke

        # LOST-TARGET CLOCK SWITCHES (MissionConfig; all default off). Applied
        # after this tick's detection bookkeeping and before the clock is read.
        if tel is not None and getattr(tel, "phase", None) == "B":
            self._had_track = True
        if cfg.pursuit_mode and self._last_det_t is not None:
            if cfg.pursuit_lost_clock_needs_track and not self._had_track:
                self._last_det_t = None    # nothing to lose yet: no track
            elif cfg.pursuit_lost_clock_off_in_search \
                    and getattr(self.guidance, "searching", False):
                self._last_det_t = None    # the terminal is still looking
        lost_s = cfg.engage_lost_target_s
        if cfg.pursuit_mode and cfg.engage_far_lost_target_s is not None \
                and self._last_r_hat_m is not None \
                and self._last_r_hat_m > cfg.pursuit_miss_range_m:
            lost_s = cfg.engage_far_lost_target_s      # far loss: the long clock

        # FAILSAFE 5 -- the terminal lost the target and did not recover.
        # NOT during a rehearsal evade: the evade is itself the timed exit
        # from the target's airspace (bounded by rehearsal_evade_s, sim
        # time) and ends in SAFE on its own; losing the tag while climbing
        # away is expected and must not relabel the pass `pursuit_miss`.
        if self._last_det_t is not None and rehearsal_t is None and \
                (obs.t - self._last_det_t) > lost_s:
            # PURSUIT-MODE MISS POLICY (builder ruling 2026-09-22, builder's
            # words: "as soon as an intercept is MISSED, abort immediately
            # and either hover in place or return home"). isim/specs/
            # parity_flightcode_2026-09-21.md's pre-registered abort-timer
            # A/B (n=50/cell, engage_lost_target_s 2.0/4.0/10.0/30.0 s) came
            # back a byte-identical NULL: at close range the dropout is
            # PERMANENT and lengthening this clock buys nothing (pursuit's
            # own Phase-B->A fallback never produces a second converging
            # pass through this wrapper). So a CLOSE miss does not wait
            # longer or search via BREAKOFF's climb (which assumes a fly-by
            # pursuit never flies) -- it goes straight to SAFE and gets out
            # of the target's airspace. Outside pursuit_miss_range_m the
            # original BREAKOFF path stands: a belief re-acquire may still
            # help at range.
            if cfg.pursuit_mode and self._last_r_hat_m is not None \
                    and self._last_r_hat_m <= cfg.pursuit_miss_range_m:
                self._transition(
                    State.SAFE,
                    f"pursuit_miss(r_hat={self._last_r_hat_m:.2f} m <= "
                    f"{cfg.pursuit_miss_range_m:.2f} m, target lost "
                    f"{lost_s:.1f}s)", obs, events)
                self.safe_reason = "pursuit_miss"
                return self._safe_setpoint(obs), tel
            self._transition(State.BREAKOFF,
                             f"target_lost({lost_s:.1f}s)",
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
    does not, fall back to a pymavlink reader on the same port.

    CLOCK BASE (review2 BLOCKER, fixed 2026-07-25): `ingest()` is fed from an
    async MAVLink callback that has no notion of "mission elapsed", so BOTH
    ingest and poll take ABSOLUTE `time.monotonic()`. The live driver used to
    stamp ingest with time.monotonic() but poll with `time.monotonic() - t0`,
    which made `age = t - last_rx_t` a large NEGATIVE number: `link_ok` was
    pinned True for the whole flight, FAILSAFE 1 (link-denied -> SAFE) was
    structurally unreachable, and the CSV asserted `trig_link=1` with
    `trig_age_s=-24580.70`. `clock_base` declares which clock poll() wants, so
    the driver can never guess wrong again (`trigger_poll_time`), and a negative
    age is now itself a hard link failure."""

    #: poll() expects ABSOLUTE time.monotonic() (the offline triggers want
    #: mission-elapsed seconds -- see trigger_poll_time()).
    clock_base = "absolute"

    def __init__(self, channel: int = 7, high_us: int = 1700, low_us: int = 1300,
                 link_timeout_s: float = 1.0):
        self.channel = int(channel)
        self.switch = PwmSwitch(high_us, low_us)
        self.link_timeout_s = float(link_timeout_s)
        self.last_pwm: Optional[int] = None
        self.last_rx_t: Optional[float] = None
        self.n_clock_faults = 0

    def ingest(self, fields: dict, t: float) -> None:
        """Fold one decoded RC_CHANNELS message in. `fields` is the JSON body
        MAVSDK's mavlink_direct hands back (chan1_raw .. chan18_raw, rssi, ...).
        `t` MUST be absolute time.monotonic() -- the same base poll() is given."""
        pwm = fields.get(f"chan{self.channel}_raw")
        if pwm is None:
            return
        self.last_pwm = int(pwm)
        self.last_rx_t = t
        self.switch.update(self.last_pwm)

    def poll(self, t: float) -> TriggerState:
        age = None if self.last_rx_t is None else (t - self.last_rx_t)
        clock_fault = age is not None and age < 0.0
        if clock_fault:
            self.n_clock_faults += 1
            if self.n_clock_faults == 1:
                print(f"[trigger] FAULT clock_base_mismatch: RC frame age is "
                      f"{age:.2f} s (negative) -- ingest() and poll() are on "
                      f"different clocks. Treating the link as DENIED.")
        # Fail CLOSED on a negative age: a link we cannot age is not a link.
        link_ok = age is not None and 0.0 <= age <= self.link_timeout_s
        return TriggerState(go=self.switch.state and link_ok, link_ok=link_ok,
                            age_s=age, raw_us=self.last_pwm,
                            clock_fault=clock_fault)


def trigger_poll_time(trigger, t_mission: float, t_abs: float) -> float:
    """WHICH CLOCK a trigger's `poll()` expects. The driver MUST route through
    this -- the whole link-denied failsafe was dead because the two call sites
    disagreed and nothing declared the contract (review2 BLOCKER).

      * `clock_base == "absolute"` (RcChannelTrigger): absolute time.monotonic(),
        because its ingest() runs in an async MAVLink callback that has no
        mission clock.
      * otherwise (ScriptedTrigger/GateReadyTrigger): mission-elapsed seconds,
        because `go_at_s` / `link_dies_at_s` are defined against mission start.

    Pure, so the test drives the SAME expression the driver does."""
    return t_abs if getattr(trigger, "clock_base", "mission") == "absolute" \
        else t_mission


class ScriptedTrigger:
    """OFFLINE ONLY (tests, --dry-run, --sitl-smoke). Fires GO at `go_at_s`, can
    simulate a link that dies at `link_dies_at_s`, and can start HIGH (to prove
    the switch-high-at-boot refusal). poll() takes MISSION-ELAPSED seconds."""

    clock_base = "mission"

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


class CueTrigger:
    """`--trigger cue`: THE CUE IS THE GO COMMAND (builder ruling 2026-11-18:
    "the cue receiving and estimate is the go command").

    GO fires on the first tick where ALL of these hold:
      * the state machine's own arm gate has been satisfied for `hold_ticks`
        ticks running (armed, OFFBOARD, at standby height and yaw, settled) --
        so a stream that was already valid before take-off still waits;
      * the human ENABLE switch, if one is configured, is HIGH with a live RC
        link, and was seen LOW first (a switch left on at start-up never
        counts). Lowering it is the veto: no GO while it is LOW;
      * the live cue link holds a quality-gated fit over the capture window
        that is fresh, spans most of that window (`min_span_s`: a stream that
        has only just started, or just come back, does not fire GO on its
        first second of data -- the 5 s window is what buys the speed
        estimate), and whose time base is trusted (`flight.cue_link`);
      * `go_check`, if given, has no objection to that fit (range, geofence).
    At that instant the belief is LATCHED ONCE, advanced to that instant, and
    the link is closed. `take_belief()` hands it to the driver for exactly one
    tick; the state machine writes it into the terminal at the GO edge.

    WHAT IT DOES WHEN THINGS GO WRONG (every case refuses, none guesses):
      * no records yet, too few, stale, a hole in the window, a clock that
        cannot be trusted, a refused fit: no GO. STANDBY continues and the
        reason is printed when it changes. The standby time budget
        (standby_max_s) is what ends a wait that never resolves.
      * the stream dies in STANDBY: the same -- the fit goes stale, no GO. It
        does not abort the sortie, so a Wi-Fi dropout costs time, not the
        flight; a stream that returns must refill a clean window first.
      * after GO nothing here touches the link again.

    THE RC LINK: with an enable switch, `link_ok` is that RC link's, so losing
    the transmitter in STANDBY is still FAILSAFE 1 (no dash, SAFE). Without
    one, `link_ok` is always True and there is no human veto after the arm
    permit; `--fly` refuses that configuration.

    HONESTY: the cue is a pre-flight given (graded given-noisy). `poll()`
    reads the link only before the latch; `take_belief()` returns the belief
    once and None ever after."""

    clock_base = "absolute"

    def __init__(self, receiver, own_lla: Callable[[], Optional[Tuple[float, float, float]]],
                 enable: Optional["RcChannelTrigger"] = None, *,
                 hold_ticks: int = 4, window_s: Optional[float] = None,
                 min_span_s: Optional[float] = None,
                 max_staleness_s: float = 1.0, max_gap_s: Optional[float] = 1.0,
                 go_check: Optional[Callable[[Any], Optional[str]]] = None,
                 log: Callable[[str], None] = print):
        self.receiver = receiver
        self.own_lla = own_lla
        self.enable = enable
        self.hold_ticks = int(hold_ticks)
        self.window_s = window_s
        if min_span_s is None:
            from flight.cue_relay import DEFAULT_CAPTURE_DURATION_S
            w = DEFAULT_CAPTURE_DURATION_S if window_s is None else float(window_s)
            min_span_s = max(1.0, 0.8 * w)       # 4.4 s of the 5.5 s window
        self.min_span_s = float(min_span_s)
        self.max_staleness_s = float(max_staleness_s)
        self.max_gap_s = max_gap_s
        self.go_check = go_check
        self._log = log
        self._ok_ticks = 0
        self._enable_seen_low = enable is None
        self.fired = False
        self.fired_t: Optional[float] = None
        self.solution = None
        self._belief_out = None
        self.status = "waiting"
        self._status_key: Optional[str] = None

    def observe_gate(self, gate_ok: bool) -> None:
        self._ok_ticks = self._ok_ticks + 1 if gate_ok else 0

    def _say(self, key: str, msg: str) -> None:
        """Print a wait reason when its KIND changes, not every tick (numbers
        are stripped from the key: "span 1.8 s" and "span 2.0 s" are one kind)."""
        self.status = msg
        key = "".join("#" if c.isdigit() else c for c in key)
        if key != self._status_key:
            self._status_key = key
            self._log(f"[cue] {msg}")

    def _fit_kwargs(self, t: float) -> dict:
        kw = dict(now_t=t, max_staleness_s=self.max_staleness_s,
                  max_gap_s=self.max_gap_s, min_span_s=self.min_span_s)
        if self.window_s is not None:
            kw["window_s"] = self.window_s
        return kw

    def poll(self, t: float) -> TriggerState:
        if self.fired:
            # After the latch the link is closed and is not consulted again.
            return TriggerState(go=True, link_ok=True, age_s=0.0, raw_us=None)
        en = (self.enable.poll(t) if self.enable is not None
              else TriggerState(go=True, link_ok=True, age_s=0.0))
        idle = TriggerState(go=False, link_ok=en.link_ok, age_s=en.age_s,
                            raw_us=en.raw_us, clock_fault=en.clock_fault)
        if self.enable is not None:
            if en.link_ok and not en.go:
                self._enable_seen_low = True
            if not en.link_ok:
                self._say("rc", "waiting: no live RC link for the enable switch")
                return idle
            if not self._enable_seen_low:
                self._say("high", "REFUSED so far: the enable switch was already "
                                  "HIGH at start -- lower it, then raise it")
                return idle
            if not en.go:
                self._say("veto", "holding: the enable switch is LOW (your veto); "
                                  "raise it to let the cue fire GO")
                return idle
        if self._ok_ticks < self.hold_ticks:
            self._say("gate", "waiting: the aircraft is not yet steady at its "
                              "standby point (arm gate)")
            return idle
        own = self.own_lla()
        if own is None:
            self._say("own", "waiting: no own GPS position (with a fresh height from the "
                             "GPS receiver) to place the target against")
            return idle
        from flight.cue_link import CueLinkError
        from flight.cue_relay import CueQualityError
        try:
            sol = self.receiver.peek(own[0], own[1], own[2], **self._fit_kwargs(t))
        except (CueQualityError, CueLinkError) as e:
            self._say("fit:" + str(e)[:40], f"waiting: no usable cue yet ({e})")
            return idle
        if self.go_check is not None:
            why = self.go_check(sol)
            if why:
                self._say("check:" + why[:24], f"holding: {why}")
                return idle
        # ---- THE LATCH: the one place the link becomes the belief. ----------
        try:
            sol = self.receiver.latch(own[0], own[1], own[2], **self._fit_kwargs(t))
        except (CueQualityError, CueLinkError) as e:   # cannot differ from peek
            self._say("latch", f"REFUSED at the latch ({e}); the link is closed "
                               f"and no GO will fire")
            return idle
        self.fired, self.fired_t, self.solution = True, t, sol
        self._belief_out = (tuple(float(x) for x in sol.belief_r0_ned),
                            tuple(float(x) for x in sol.belief_vel0_ned))
        info = getattr(self.receiver, "latch_info", None) or {}
        base = info.get("time_base", "?")
        delay = ""
        if base == "utc":
            delay = (f", link delay measured {info['delay_min_s']*1e3:.0f} to "
                     f"{info['delay_max_s']*1e3:.0f} ms (median "
                     f"{info['delay_median_s']*1e3:.0f}) and removed")
        elif base == "min_delay":
            delay = (", link delay NOT measured (no shared clock): the belief "
                     "lags by the link's minimum delay")
        elif base == "receive":
            delay = ", link delay NOT measured and not reduced (receive time only)"
        self._log(f"[cue] GO: fit of {sol.n_points} records over {sol.span_s:.1f} s "
                  f"({sol.model}, velocity {sol.vel_source}, residual "
                  f"{sol.residual_rms_m:.2f} m), advanced "
                  f"{info.get('advanced_by_s', float('nan'))*1e3:.0f} ms to now on "
                  f"the '{base}' time base{delay}")
        return TriggerState(go=True, link_ok=True, age_s=0.0, raw_us=en.raw_us)

    def take_belief(self):
        """The latched belief, ONCE (the GO tick); None before and ever after."""
        out, self._belief_out = self._belief_out, None
        return out


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
# The second latch (2026-11-18): the belief taken from the live cue at the GO
# edge. Same rule -- exactly one call site may write it.
_CUE_LATCH_ATTR = "cue_belief"
# Identifier prefixes that would mean a ground-truth read. There is no ground
# truth on real hardware; this asserts none crept in from a sim-side copy/paste.
#
# `ground_truth` is NOT optional here (review2 HIGH, 2026-07-25): the repo's
# ACTUAL ground-truth accessors are `m3_static_intercept.ground_truth_world_points`
# and `PoseTracker.ground_truth_rel_optical` -- neither starts with `gt_`, so the
# one-prefix guard PASSED a planted read of the real accessor. This list is kept
# equal to tests/test_honesty_seekers.GT_PREFIXES on purpose; the drift between
# two guards that were meant to say the same thing IS the bug.
_FORBIDDEN_ID_PREFIXES = ("gt_", "ground_truth")
# Import roots that would drag ground truth in under a clean local name (a
# wrapper renaming the return value defeats the identifier check alone).
_FORBIDDEN_IMPORT_ROOTS = ("gz", "gz_transport", "gz.transport13",
                           "m2_detect", "m3_static_intercept", "m4_intercept",
                           "guidance_lab", "pose_tracker")

# The REAL-BUILD IMPORT CLOSURE: every module that runs on the vehicle. The
# audit's printed claim is only true of what it actually walks, so the boundary
# of the proof is now explicit and machine-checked instead of asserted. The
# latch-count arm stays scoped to real_flight.py (it is the only file with a
# latch); the gt/import arms run over all of these.
_AUDITED_MODULES = (
    os.path.join("flight", "deploy", "real_flight.py"),
    os.path.join("flight", "deploy", "seeker_loop.py"),
    os.path.join("flight", "camera.py"),
    os.path.join("flight", "geometry.py"),
    os.path.join("flight", "estimator.py"),
    os.path.join("flight", "guidance.py"),
    os.path.join("flight", "range_fusion.py"),
    os.path.join("flight", "terminal_coast.py"),
    os.path.join("flight", "fov_guidance.py"),
    os.path.join("flight", "tag_terminal.py"),
    os.path.join("flight", "pursuit_terminal.py"),
    os.path.join("flight", "deploy", "frame_feed.py"),
    # Added 2026-10-22 (review 3): cue_relay seeds the terminal's belief and had
    # escaped the audit -- the same rot that once left seeker_loop uncovered.
    os.path.join("flight", "cue_relay.py"),
    # Added 2026-11-18: the live cue link feeds the belief latched at GO.
    os.path.join("flight", "cue_link.py"),
    # frame_recorder is log-only, but it runs on the vehicle, so it is audited
    # rather than exempted: cheaper to walk it than to argue about it.
    os.path.join("flight", "deploy", "frame_recorder.py"),
    # The backup-link wrapper and the board-settings handler (2026-11-26). Neither
    # reads a sensor the guidance uses, but both run on the vehicle.
    os.path.join("flight", "deploy", "link_redundancy.py"),
    os.path.join("flight", "deploy", "board_overrides.py"),
    # The setpoint-stream watchdog (2026-11-18). A separate process that reads
    # no sensor, but it runs on the vehicle: same rule as the recorder.
    os.path.join("flight", "deploy", "loop_supervisor.py"),
)

# Modules REACHABLE from the audited closure that the audit deliberately does
# not walk, each with the reason it is safe to skip. Anything reachable from an
# audited module and in NEITHER mapping fails
# flight/tests/test_real_flight.py::test_the_audited_closure_is_import_complete,
# so a new `from flight.X import ...` cannot silently escape the audit again.
_AUDIT_IMPORT_EXEMPT = {
    os.path.join("flight", "__init__.py"):
        "docstring-only package marker; no imports, no executable code",
    os.path.join("flight", "deploy", "__init__.py"):
        "docstring-only package marker; no imports, no executable code",
}


def audited_module_paths() -> List[str]:
    """Absolute paths of the real-build import closure the honesty audit walks."""
    return [os.path.join(_REPO_ROOT, m) for m in _AUDITED_MODULES]


def _audit_one(path: str, check_latch: bool) -> Tuple[List[str], int]:
    """AST arms for ONE file. Returns (problems, latch_set_count)."""
    with open(path) as fh:
        tree = ast.parse(fh.read(), filename=path)
    problems: List[str] = []

    def bad(name: str) -> bool:
        return any(name.startswith(p) for p in _FORBIDDEN_ID_PREFIXES)

    latch_sets = 0
    cue_latch_sets = 0
    cue_link_latches = 0
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
            if isinstance(f, ast.Attribute) and f.attr == "set" \
                    and isinstance(f.value, ast.Attribute) \
                    and f.value.attr == _CUE_LATCH_ATTR:
                cue_latch_sets += 1
            # `<anything>.receiver.latch(...)`: the call that turns the live
            # link into the belief and closes it.
            if isinstance(f, ast.Attribute) and f.attr == "latch" \
                    and isinstance(f.value, ast.Attribute) \
                    and f.value.attr == "receiver":
                cue_link_latches += 1

    if check_latch and latch_sets != 1:
        problems.append(
            f"the dash-heading latch is written at {latch_sets} call sites "
            "(must be EXACTLY 1 -- the GO edge)")
    if check_latch and cue_latch_sets != 1:
        problems.append(
            f"the cue belief latch is written at {cue_latch_sets} call sites "
            "(must be EXACTLY 1 -- the GO edge)")
    if check_latch and cue_link_latches != 1:
        problems.append(
            f"the live cue link is latched at {cue_link_latches} call sites "
            "(must be EXACTLY 1 -- the cue trigger's GO)")
    return problems, latch_sets


def is_offboard_mode(mode) -> Optional[bool]:
    """True/False from one MAVSDK `FlightMode` sample; None if there is no sample
    yet. Compared by NAME so this is testable with no mavsdk installed, and so a
    MAVSDK enum-repr change ("FlightMode.OFFBOARD" vs "OFFBOARD") cannot flip it.

    DO NOT substitute `drone.offboard.is_active()`: that reports MAVSDK's own
    setpoint-mode flag, not the vehicle's flight mode, and would reproduce the
    exact permanently-True silent failure this replaces."""
    if mode is None:
        return None
    name = getattr(mode, "name", None) or str(mode)
    return str(name).rsplit(".", 1)[-1].strip().upper() == "OFFBOARD"


def make_mode_subscriber(drone, state: dict):
    """The live OFFBOARD producer, as a factory so a test can drive it with a
    fake drone (the seam that had no test -- the consumer was parametrised, the
    producer did not exist). Sets state["offboard"] from TELEMETRY, never from
    "we called offboard.start() once".

    It also STAMPS each sample (`state["t_mode"]`, monotonic). FAILSAFE 7 arm 2
    reads the age of that stamp: without it, a subscriber that dies leaves
    `state["offboard"]` frozen at True and the mode-loss arm becomes structurally
    unreachable -- the same freeze the driver's `_task_died` warning exists for."""
    async def _mode():
        async for m in drone.telemetry.flight_mode():
            state["offboard"] = is_offboard_mode(m)
            state["mode"] = str(getattr(m, "name", None) or m)
            state["t_mode"] = time.monotonic()
    return _mode


def honesty_audit(path: Optional[str] = None, verbose: bool = True,
                  paths: Optional[Sequence[str]] = None) -> int:
    """Prove -- from the SYNTAX TREE, not the text -- that the real-build code:

      1. reads NO ground-truth symbol (no identifier with a forbidden prefix),
      2. imports no simulator/ground-truth module,
      3. writes the dash heading latch at EXACTLY ONE call site (the GO edge),
         i.e. `docs/launch_mechanism_plan.md` L2's "assigned once, never again".

    A comment or a docstring cannot fool it (the walk ignores string constants),
    which is why the module's prose may freely discuss the boundary. Mirrors the
    existing AST-audit pattern in scripts/field/fc_link_check.py.

    SCOPE (be precise about what this proves): with no arguments it walks the
    whole real-build import closure (`_AUDITED_MODULES`) -- not just this file,
    which was the pre-2026-07-25 behaviour and left seeker_loop.py (the module
    that actually produces det_range_m) covered by no AST check at all. Pass
    `path=` to audit a single file (used by the mutation-calibration test).

    It remains a NAMING-CONVENTION check, not an information-flow proof: it
    cannot see ground truth handed in by a caller under a clean name or read from
    a topic at runtime. It says "no module on the vehicle NAMES a ground-truth
    symbol or imports a simulator module", which is what it should be quoted as.

    Exit 0 = clean, 1 = violation."""
    if path is not None:
        targets = [os.path.abspath(path)]
    elif paths is not None:
        targets = [os.path.abspath(p) for p in paths]
    else:
        targets = audited_module_paths()
    real_flight_path = os.path.join(_REPO_ROOT, _AUDITED_MODULES[0])

    all_problems: List[str] = []
    latch_total = 0
    latch_checked = False
    per_file = []
    for tgt in targets:
        # The latch arm applies to real_flight.py only -- and to a single-file
        # audit of a mutated COPY of it (the calibration test's temp file), which
        # is detected by the module's own latch attribute being present.
        check_latch = (os.path.abspath(tgt) == os.path.abspath(real_flight_path)
                       or (len(targets) == 1 and _has_latch_site(tgt)))
        probs, n_latch = _audit_one(tgt, check_latch)
        if check_latch:
            latch_total += n_latch
            latch_checked = True
        per_file.append((tgt, probs))
        all_problems += [f"{os.path.relpath(tgt, _REPO_ROOT)}: {p}" for p in probs]

    if verbose:
        print("[audit] real-build import closure "
              f"({len(targets)} module{'s' if len(targets) != 1 else ''}):")
        for tgt, probs in per_file:
            rel = os.path.relpath(tgt, _REPO_ROOT)
            print(f"  [{'PASS' if not probs else 'FAIL'}] {rel}")
        if latch_checked:
            print(f"  [{'PASS' if latch_total == 1 else 'FAIL'}] dash heading "
                  f"latched at exactly one call site (found {latch_total})")
            n_cue = sum(1 for p in all_problems if "cue " in p and "latch" in p)
            print(f"  [{'PASS' if n_cue == 0 else 'FAIL'}] cue belief written at "
                  f"exactly one call site, and the live link latched at exactly "
                  f"one (the GO edge)")
        n_gt = sum(1 for p in all_problems if "ground-truth" in p)
        print(f"  [{'PASS' if n_gt == 0 else 'FAIL'}] no ground-truth identifier "
              f"({'/'.join(_FORBIDDEN_ID_PREFIXES)}) is read in ANY audited "
              f"module (found {n_gt})")
        n_imp = sum(1 for p in all_problems if "simulator import" in p)
        print(f"  [{'PASS' if n_imp == 0 else 'FAIL'}] no simulator/ground-truth "
              f"import (found {n_imp})")
        print("  [INFO] inputs by construction, IN THE MODULES LISTED ABOVE: "
              "camera pixels (ENGAGE only), own-state EKF, pre-flight constants, "
              "and ONE arm/kill trigger bit")
        print("  [INFO] scope: AST naming/import check -- it cannot see ground "
              "truth passed in by a caller under a clean name")
        for p in all_problems:
            print(f"  VIOLATION: {p}")
        print(f"[audit] {'PASS' if not all_problems else 'FAIL'}")
    return 0 if not all_problems else 1


def _has_latch_site(path: str) -> bool:
    """Does this file define the LatchOnce cell (i.e. is it a copy of the state
    machine)? Used so the calibration test's temp copy still gets the latch arm."""
    try:
        with open(path) as fh:
            src = fh.read()
    except OSError:
        return False
    return f"self.{_LATCH_ATTR} = LatchOnce" in src


# ---------------------------------------------------------------- CSV logging


# PER-TICK MISSION LOG. The guidance block (bearing.. onward) exists because the
# pre-2026-07-25 schema logged only the COMMAND: a flight that latched the coast
# freeze, spun on a stale bearing or ran on a missing attitude produced a CSV of
# smooth, plausible setpoints with nothing to tell any of them apart from correct
# guidance (review2). Decision.telemetry was already carried and thrown away.
# `_CSV_FIELDS` is the single source of truth -- header and row are built from it,
# so the two halves cannot drift (the seam that broke elsewhere today).
_CSV_FIELDS = (
    # `offb_age_s` is FAILSAFE 7 arm 2's input: without it a log cannot tell
    # "offboard=True, freshly sampled" from "offboard=True, frozen 40 s ago".
    "t_s", "state", "streak", "armed", "offboard", "offb_age_s", "mode",
    # `alt_ref_m` / `floor_active` added 2026-09-10 (review F11): without them a
    # trimmed flight's log is byte-indistinguishable from an untrimmed one, and the
    # documented procedure is "re-derive the drift from the logs" -- so a second
    # iteration would re-derive on already-trimmed data and double-count.
    "alt_m", "alt_ref_m", "floor_active", "yaw_deg",
    "trig_go", "trig_link", "trig_age_s", "trig_clock_fault", "trig_us",
    "det_new", "det_range_m",
    "v_north", "v_east", "v_down", "yaw_cmd_deg",
    # --- guidance state (dec.telemetry; empty outside ENGAGE) ---
    "sp_source", "bearing_deg", "lambda_deg", "lambda_dot_deg_s",
    "r_hat_m", "rdot_hat_m_s", "v_perp", "terminal_coast",
    "own_state_ok", "own_age_s", "meas_age_s", "stale", "yaw_hold",
    "track_broken", "coast_age_s", "health",
    "safe_reason",
)
_CSV_HEADER = ",".join(_CSV_FIELDS) + "\n"


def _csv_row(obs: VehicleObs, dec: Decision) -> str:
    sp = dec.setpoint
    tel = dec.telemetry

    def f(x, nd=4):
        return "" if x is None else f"{x:.{nd}f}"

    def b(x):
        return "" if x is None else str(int(bool(x)))

    def g(attr, nd=4):
        return "" if tel is None else f(getattr(tel, attr, None), nd)

    if tel is None:
        sp_source = "safe" if dec.state == State.SAFE else dec.state.lower()
    elif tel.setpoint is None:
        sp_source = "hold"            # seeker declined -> caller's coast setpoint
    elif tel.terminal_coast:
        sp_source = "coast"
    else:
        sp_source = "guided"
    health = "" if tel is None else "|".join(getattr(tel, "health", []) or [])
    # Effective vertical reference + floor state, for the two new columns. Read
    # off the machine rather than recomputed, so the log records what was APPLIED
    # rather than what a reader thinks should have been (the ADR-0096 lesson: an
    # actuator that logs intent rather than effect can diverge silently).
    alt_ref_eff = dec.alt_ref_m
    floor_active = bool(dec.floor_active)
    vals = [
        f"{obs.t:.3f}", dec.state, str(dec.streak), str(obs.armed),
        str(obs.offboard_active), f(obs.offboard_sample_age_s, 2),
        str(obs.mode or ""), f(obs.alt_m, 3),
        # The EFFECTIVE reference the vertical command was built from, and whether
        # the AGL floor clamped it. Without these a trimmed flight's log is
        # byte-indistinguishable from an untrimmed one (review F11), and the
        # documented procedure re-derives the drift FROM these logs -- so a second
        # iteration would silently re-derive on already-trimmed data.
        f(alt_ref_eff, 3), b(floor_active),
        f(obs.yaw_deg, 2),
        str(int(bool(obs.trigger.go))), str(int(bool(obs.trigger.link_ok))),
        f(obs.trigger.age_s, 2), str(int(bool(obs.trigger.clock_fault))),
        str(obs.trigger.raw_us or ""),
        str(int(bool(obs.det_new))), f(obs.det_range_m, 3),
        f"{sp.v_north:.4f}", f"{sp.v_east:.4f}", f"{sp.v_down:.4f}",
        f"{sp.yaw_deg:.2f}",
        sp_source, g("bearing_deg", 3), g("lambda_deg", 3),
        g("lambda_dot_deg_s", 3), g("r_hat_m", 3), g("rdot_hat_m_s", 3),
        g("v_perp", 3), "" if tel is None else b(tel.terminal_coast),
        "" if tel is None else b(tel.own_state_ok), g("own_age_s", 3),
        g("meas_age_s", 3), "" if tel is None else b(tel.stale),
        "" if tel is None else b(tel.yaw_hold),
        "" if tel is None else b(tel.track_broken), g("coast_age_s", 3),
        health,
        dec.safe_reason or "",
    ]
    assert len(vals) == len(_CSV_FIELDS), (
        f"CSV row/header drift: {len(vals)} values vs {len(_CSV_FIELDS)} columns")
    return ",".join(vals) + "\n"


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
                _eff = (cfg.miss_safe_behavior
                        if dec.safe_reason in RealFlightSM.MISS_SAFE_REASONS
                        else cfg.safe_behavior)
                _action = {"land": "LAND", "rtl": "RETURN-TO-LAUNCH",
                          "hover": "LAND (mission_max_s backstop reached)"
                          }[_eff]
                print(f"  t={t:6.2f}  TERMINATED in SAFE "
                      f"(reason={dec.safe_reason}); driver would now {_action}")
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


def run_desk(cfg: MissionConfig, sm: "RealFlightSM", trigger, detector,
             fps: float = 20.0, max_s: float = 90.0,
             acquire_after_s: float = 0.0, realtime: bool = True,
             csv_path: Optional[str] = None, frame_recorder=None,
             verbose: bool = True):
    """--desk: the mission loop over REAL frames + the REAL detector with NO
    vehicle link. Cannot arm by construction: this function imports no MAVSDK,
    holds no vehicle handle, and own-state comes from FakeVehicle (its
    armed/offboard flags are synthetic inputs to the arm gate). Every setpoint
    is computed and logged, never sent. Detection timing mirrors
    run_mavsdk_mission (from t_dash_start + acquire_after_s, every tick).

    realtime=True paces ticks on time.monotonic (live camera); False steps
    t = n/fps with no sleep (replay, deterministic). Returns (sm, rows)."""
    veh = FakeVehicle(alt_m=cfg.standby_alt_m, yaw_deg=0.0)
    dt = 1.0 / fps
    rows: List[str] = []
    n = 0
    t = 0.0
    t0 = time.monotonic()
    try:
        while t <= max_s:
            if realtime:
                t = time.monotonic() - t0
            trig = trigger.poll(t)
            det_new = det_range = det_box = det_pose = None
            if sm.t_dash_start is not None and \
                    t >= sm.t_dash_start + acquire_after_s:
                d = detector.detect(None, t)
                _f = getattr(d, "frame", None)
                if frame_recorder is not None and _f is not None:
                    frame_recorder.offer(_f, getattr(d, "t_capture", t))
                det_new, det_range, det_box = (getattr(d, "is_new", True),
                                               d.range_m, d.box_xywh)
                det_pose = getattr(d, "tag_range_m", None)
            obs = veh.obs(t, trig, det_new=bool(det_new), det_range_m=det_range,
                          det_box_xywh=det_box, det_range_pose_m=det_pose)
            if isinstance(trigger, GateReadyTrigger):
                trigger.observe_gate(sm.arm_gate(obs)[0])
            dec = sm.step(obs)
            rows.append(_csv_row(obs, dec))
            if verbose and n % 20 == 0:
                sp = dec.setpoint
                print(f"[desk] t={t:6.2f} {dec.state:<10} streak={dec.streak} "
                      f"det={'-' if det_range is None else f'{det_range:.2f}'} "
                      f"| SP (NOT SENT) vN={sp.v_north:+.2f} "
                      f"vE={sp.v_east:+.2f} vD={sp.v_down:+.2f} "
                      f"yaw={sp.yaw_deg:+.1f}")
            veh.apply(dec.setpoint, dt)
            if dec.terminated:
                if verbose:
                    print(f"[desk] t={t:6.2f} TERMINATED in SAFE "
                          f"(reason={dec.safe_reason})")
                break
            n += 1
            if realtime:
                time.sleep(max(0.0, t0 + n * dt - time.monotonic()))
            else:
                t = n * dt
    finally:
        if csv_path:
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            with open(csv_path, "w") as fh:
                fh.write(_CSV_HEADER)
                fh.writelines(rows)
            if verbose:
                print(f"[log] {len(rows)} ticks -> {csv_path}")
        if frame_recorder is not None:
            fr_meta = frame_recorder.close()
            print(f"[frame_recorder] {fr_meta['n_written']} written, "
                  f"{fr_meta['n_dropped']} dropped -> {fr_meta['out_dir']}")
    return sm, rows


# ---------------------------------------------------------------- live-driver safety
#
# Built 2026-11-18 from the interface audit of the same day. The audit read the
# live driver and found that the only way to fly the real aircraft was the SITL
# smoke path with its simulator defaults (G1), that a hung Pi program keeps the
# aircraft flying its last command (G2), that the board's limits are below what
# the guidance commands and nothing checks (G4), that teardown lands the aircraft
# out from under a pilot who has taken over (G6), that a frozen own-state stream
# goes unnoticed (G7), and that the props-off rehearsal the field protocol asks
# for could not be run at all (G11).
#
# Everything in this section is PURE, or a thin wrapper round one child process,
# so it is tested on the desk (flight/tests/test_real_flight_driver.py). The
# driver below only wires it together.


def is_serial_url(url) -> bool:
    """True for a MAVSDK URL that addresses a REAL serial link (the Pi's UART to
    TELEM2, or the Pixhawk's USB port). A simulator is always UDP or TCP."""
    return bool(url) and str(url).strip().lower().startswith("serial")


class _ExplicitStore(argparse.Action):
    """argparse `store`, plus a record that the operator TYPED this flag.

    `--target-start` and `--target-vel` have simulator defaults (a 9 m/s target
    16.7 m north). A real flight must not be able to inherit a belief nobody
    entered, and a value equal to the default is still a decision if it was
    typed -- so "explicit" is recorded, not inferred by comparing values."""

    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, values)
        seen = set(getattr(namespace, "explicit_flags", None) or ())
        seen.add(self.dest)
        namespace.explicit_flags = seen


def validate_fly_args(args) -> List[str]:
    """Every reason `--fly` must refuse to start, as plain sentences. Empty list
    = allowed. Pure, so the refusals are unit-tested without a vehicle.

    `--fly` is the ONLY mode that may arm over a serial link. It exists so a
    real flight cannot be started with a synthetic trigger, a synthetic target
    or a belief nobody entered -- all three were the DEFAULTS of the only live
    path before 2026-11-18."""
    why: List[str] = []
    if not getattr(args, "mavsdk_url", None):
        why.append("--fly needs --mavsdk-url (on the Pi: "
                   "serial:///dev/ttyAMA10:921600)")
    trig = getattr(args, "trigger", None)
    if trig not in ("rc", "cue"):
        why.append(f"--fly needs --trigger rc or --trigger cue: '{trig}' "
                   f"is a synthetic GO that fires with no human")
    if trig == "cue":
        if getattr(args, "terminal", "stock") != "pursuit":
            why.append("--trigger cue needs --terminal pursuit (the only "
                       "terminal that can take a belief latched at GO)")
        if getattr(args, "cue_no_enable", False):
            why.append("--fly refuses --cue-no-enable: with the cue as the GO "
                       "command, the enable switch is the only human veto "
                       "between the arm permit and the dash")
        if getattr(args, "cue_utc_source", "vehicle") == "none" \
                and not getattr(args, "cue_allow_unsynced", False):
            why.append("--cue-utc-source none can never fire GO unless "
                       "--cue-allow-unsynced is also given")
        for flag, name in (("standby_alt_m", "--standby-alt-m"),
                           ("standby_yaw_deg", "--standby-yaw-deg")):
            if getattr(args, flag, None) is None:
                why.append(f"--trigger cue needs {name} typed: the belief does "
                           f"not exist until GO, so the standby hold cannot be "
                           f"derived from it")
    src = getattr(args, "source", None)
    if not src:
        why.append("--fly needs a real camera (--source picamera): without "
                   "--source the detector is a synthetic target")
    elif str(src).startswith("dir:"):
        why.append(f"--fly refuses --source {src}: replayed images are not a "
                   f"camera")
    elif getattr(args, "camera_mount", None) not in ("upright", "upside-down"):
        why.append("--fly needs --camera-mount upright|upside-down: a camera "
                   "mounted upside down and read unrotated mirrors every "
                   "bearing, and the aircraft steers away from the target. "
                   "The interceptor's nose camera is upside-down.")
    explicit = set(getattr(args, "explicit_flags", None) or ())
    if trig == "cue":
        pass    # the belief is the cue, latched at GO; nothing to type
    elif not getattr(args, "cue_json", None):
        missing = [f for f in ("target_start", "target_vel") if f not in explicit]
        if missing:
            why.append("--fly needs an explicit belief: --cue-json FILE, or BOTH "
                       "--target-start and --target-vel typed (with "
                       "--manual-belief). The defaults are a simulator's.")
        elif not getattr(args, "manual_belief", False):
            why.append("a typed --target-start/--target-vel belief needs "
                       "--manual-belief, to say on the record that the aim came "
                       "from a person and not from the GPS cue")
    ch = getattr(args, "arm_permit_channel", None)
    if ch is None:
        why.append("--fly needs --arm-permit-channel N: the RC switch whose "
                   "LOW-to-HIGH flip permits the program to arm and take off")
    elif ch == getattr(args, "rc_channel", None):
        why.append(f"--arm-permit-channel {ch} is the GO channel: the permit "
                   f"must be a different switch")
    if not (getattr(args, "loop_watchdog_s", 0.0) or 0.0) > 0.0:
        why.append("--fly refuses --loop-watchdog-s 0: a real flight does not "
                   "run without the setpoint-stream watchdog")
    return why


def validate_bench_args(args) -> List[str]:
    """Reasons `--bench-props-off` must refuse to start. Empty list = allowed."""
    why: List[str] = []
    if not getattr(args, "mavsdk_url", None):
        why.append("--bench-props-off needs --mavsdk-url")
    if getattr(args, "trigger", None) not in ("rc", "timer", "cue"):
        why.append("--bench-props-off needs --trigger rc (the real GO switch), "
                   "--trigger cue (the live cue link) or --trigger timer "
                   "(unattended)")
    if getattr(args, "trigger", None) == "cue" \
            and getattr(args, "terminal", "stock") != "pursuit":
        why.append("--trigger cue needs --terminal pursuit")
    return why


class ArmPermit:
    """`--fly`'s deliberate human step, as a pure latch.

    Launching the program must NOT be the arm command (it was, before
    2026-11-18: the driver armed the moment GPS health went good). Under
    `--fly` the program connects, checks, prints what it is about to do, and
    then WAITS for this permit before it arms:

        1. the RC link is alive and the GO switch is seen LOW;
        2. the PERMIT switch (a different channel) is seen LOW;
        3. the permit switch then goes HIGH while the GO switch is still LOW.

    Fail-closed at every step: a permit switch found already HIGH never counts
    (it must be seen LOW first, the same rule as the GO edge); a rise while the
    GO switch is HIGH is refused and the permit must go LOW again; no RC link
    means no progress. The permit is a one-way latch for this run -- lowering
    the switch afterwards does nothing (stopping the aircraft is the kill
    switch's job, and PX4's, not this program's).

    Feed it the two `TriggerState`s from two `RcChannelTrigger`s every poll."""

    def __init__(self, high_us: int = 1700, low_us: int = 1300):
        if low_us >= high_us:
            raise ValueError("low_us must be < high_us")
        self.high_us, self.low_us = int(high_us), int(low_us)
        self.go_seen_low = False
        self.permit_seen_low = False
        self.granted = False
        self.refused_go_high = False
        self._permit_prev_high: Optional[bool] = None

    def _level(self, ts: TriggerState) -> Optional[str]:
        if not ts.link_ok or ts.raw_us is None:
            return None
        if ts.raw_us >= self.high_us:
            return "high"
        if ts.raw_us <= self.low_us:
            return "low"
        return "mid"

    def update(self, go: TriggerState, permit: TriggerState) -> bool:
        if self.granted:
            return True
        g, p = self._level(go), self._level(permit)
        if g is None or p is None:
            # No link, no progress -- and a stale switch is not a seen switch.
            self._permit_prev_high = None
            return False
        if g == "low":
            self.go_seen_low = True
        if p == "low":
            self.permit_seen_low = True
        rose = (p == "high" and self._permit_prev_high is False)
        self._permit_prev_high = (p == "high") if p != "mid" else self._permit_prev_high
        if rose and self.permit_seen_low:
            if g == "low" and self.go_seen_low:
                self.granted = True
            else:
                # GO is up (or was never seen down): refuse, and make the
                # operator lower the permit switch before trying again.
                self.refused_go_high = True
                self.permit_seen_low = False
        return self.granted

    def status(self) -> str:
        if self.granted:
            return "GRANTED"
        if not self.go_seen_low:
            return "waiting: GO switch not yet seen LOW with a live RC link"
        if not self.permit_seen_low:
            return "waiting: permit switch not yet seen LOW"
        return "ready: raise the permit switch (GO must stay LOW)"


# Flight modes in which THIS PROGRAM owns the aircraft. In any other mode
# either the pilot has taken it (manual, stabilized, altitude, position, acro)
# or PX4 is already running its own failsafe (return, land).
_PROGRAM_OWNED_MODES = ("OFFBOARD", "HOLD", "TAKEOFF")


def teardown_plan(mode_name: Optional[str], mode_age_s: Optional[float],
                  stale_s: Optional[float], safe_behavior: str
                  ) -> Tuple[bool, Optional[str], str]:
    """What the driver may send when it shuts down. Returns
    (stop_offboard, action, why) with action in ('land', 'rtl', None).

    THE FAULT THIS FIXES (audit gap G6): teardown used to send `offboard.stop()`
    and then LAND unconditionally. MAVSDK's `offboard.stop()` itself commands
    HOLD. So when the pilot flipped the mode switch to take the aircraft back --
    the thing the field protocol tells him is safe -- the program answered about
    2.5 s later by pulling it into HOLD and then LAND under his hands.

    RULE: commands go out only while the aircraft is in a mode this program put
    it in (OFFBOARD, HOLD, or TAKEOFF before OFFBOARD was ever reached). In a
    pilot mode, or a PX4 failsafe mode, the program sends NOTHING -- not even
    the offboard stop.

    UNKNOWN is the one judgment call: with no flight-mode sample at all, or a
    stale one, nobody can say who has the aircraft. This keeps the old
    behaviour there (stop and land), because an aircraft left hovering with no
    program and no known owner is the worse of the two errors, and a pilot who
    does have it can cancel a LAND with his mode switch. Pure."""
    action = "rtl" if safe_behavior == "rtl" else "land"
    name = None if mode_name is None else \
        str(mode_name).rsplit(".", 1)[-1].strip().upper()
    stale = (mode_age_s is not None
             and (mode_age_s < 0.0
                  or (stale_s is not None and mode_age_s > stale_s)))
    if name is None or name in ("", "UNKNOWN", "NONE") or stale:
        return True, action, ("flight mode unknown or stale -- cannot tell who "
                              "has the aircraft; stopping and landing")
    if name in _PROGRAM_OWNED_MODES:
        return True, action, f"aircraft is in {name}, which this program commanded"
    return False, None, (f"aircraft is in {name}: the pilot or a PX4 failsafe "
                         f"has it -- this program sends NOTHING")


def own_state_age(stamps: dict, now: float) -> Optional[float]:
    """Age of the OLDEST own-state stream, given each stream's last arrival
    stamp (None = that stream has not delivered yet, which the arm gate already
    refuses on -- so it is not counted as old). None when nothing has arrived.
    `now` and the stamps MUST share one clock."""
    ages = [now - t for t in stamps.values() if t is not None]
    return max(ages) if ages else None


# The PX4 settings the guidance silently depends on, read ONCE before arming.
# READ-ONLY: this module never writes a parameter (a test walks the syntax tree
# for any set_param call). The one write on the live path is MAVSDK's own
# `set_takeoff_altitude`, which sets MIS_TAKEOFF_ALT; the pre-arm summary says so.
# THE ONE EXCEPTION, and it lives in its own file on purpose: under
# `--board-failsafes suspend` flight/deploy/board_overrides.py changes a fixed,
# test-pinned list of three settings on the DISARMED aircraft and restores them
# after landing (builder ruling 2026-11-26). The pre-arm summary lists them.
PLATFORM_PARAMS: Tuple[Tuple[str, str, str], ...] = (
    ("MPC_XY_VEL_MAX", "float", "horizontal speed limit, m/s"),
    ("MPC_TILTMAX_AIR", "float", "lean limit, deg"),
    ("MPC_ACC_HOR_MAX", "float", "horizontal acceleration limit, m/s^2"),
    ("GF_ACTION", "int", "geofence action (0 none, 1 warn, 2 hold, 3 return, "
                         "4 terminate, 5 land)"),
    ("GF_MAX_HOR_DIST", "float", "geofence radius, m (0 = off)"),
    ("GF_MAX_VER_DIST", "float", "geofence height, m (0 = off)"),
    ("COM_RCL_EXCEPT", "int", "modes exempt from RC-loss failsafe (bit 2 = offboard)"),
    ("COM_OF_LOSS_T", "float", "offboard stream loss timeout, s"),
    ("COM_OBL_RC_ACT", "int", "action on offboard loss (0 position, 1 altitude, "
                              "2 manual, 3 return, 4 land, 5 hold)"),
    # The board's own thresholds, which the program applies itself before
    # takeoff when it suspends the board's battery action and GPS checks.
    ("BAT_LOW_THR", "float", "battery 'low' level, fraction of charge"),
    ("BAT_CRIT_THR", "float", "battery 'critical' level, fraction of charge"),
    ("EKF2_REQ_NSATS", "int", "fewest satellites the estimator accepts"),
    ("EKF2_REQ_EPH", "float", "worst horizontal GPS accuracy accepted, m"),
    ("EKF2_REQ_SACC", "float", "worst GPS speed accuracy accepted, m/s"),
)


def battery_prearm_refusal(batt_frac: Optional[float], limits: dict
                           ) -> Optional[str]:
    """Why a program flight with the board's battery action suspended must not
    arm, or None. Fails closed: no reading, or no threshold, is a refusal.
    Pure."""
    low = limits.get("BAT_LOW_THR")
    if low is None:
        return "the board's battery 'low' level (BAT_LOW_THR) was not read"
    if batt_frac is None:
        return ("no battery reading from the Pixhawk: with the board's "
                "low-battery action suspended the program must see the charge")
    if float(batt_frac) < float(low):
        return (f"battery at {float(batt_frac):.0%}, below the board's 'low' "
                f"level of {float(low):.0%}: charge or change the pack")
    return None


def gps_prearm_refusals(samples: Sequence[dict], limits: dict) -> List[str]:
    """The board's OWN GPS quality thresholds, applied by the program before
    takeoff -- the check `--board-failsafes suspend` then switches off on the
    board so that a dip in flight cannot end the intercept. `samples` are the
    readings of the last few seconds ({sats, fix, hacc, sacc}); EVERY one must
    pass. Fails closed on no samples or an unread threshold. Pure."""
    out: List[str] = []
    need = {k: limits.get(k) for k in ("EKF2_REQ_NSATS", "EKF2_REQ_EPH",
                                       "EKF2_REQ_SACC")}
    missing = [k for k, v in need.items() if v is None]
    if missing:
        return [f"GPS thresholds not read from the board: {', '.join(missing)}"]
    if not samples:
        return ["no GPS quality reading arrived from the Pixhawk"]
    sats = [s.get("sats") for s in samples]
    fixes = [s.get("fix") for s in samples]
    haccs = [s.get("hacc") for s in samples]
    saccs = [s.get("sacc") for s in samples]
    if any(v is None for v in sats + fixes + haccs + saccs):
        return ["a GPS quality reading is missing (satellites, fix, or accuracy)"]
    if min(fixes) < 3:
        out.append("no 3D GPS fix")
    if min(sats) < int(need["EKF2_REQ_NSATS"]):
        out.append(f"{min(sats)} satellites, fewer than the {int(need['EKF2_REQ_NSATS'])} "
                   f"the board requires (EKF2_REQ_NSATS)")
    if max(haccs) > float(need["EKF2_REQ_EPH"]):
        out.append(f"horizontal accuracy {max(haccs):.1f} m, worse than "
                   f"{float(need['EKF2_REQ_EPH']):.1f} m (EKF2_REQ_EPH)")
    if max(saccs) > float(need["EKF2_REQ_SACC"]):
        out.append(f"speed accuracy {max(saccs):.2f} m/s, worse than "
                   f"{float(need['EKF2_REQ_SACC']):.2f} m/s (EKF2_REQ_SACC)")
    return out


def fresh_battery_frac(state: dict, now: float, max_age_s: float = 5.0
                       ) -> Optional[float]:
    """The battery charge (0..1) if its reading is recent, else None. A stale
    reading never triggers the battery rule."""
    t = state.get("t_batt")
    rem = state.get("batt_rem")
    if t is None or rem is None or (now - t) > max_age_s:
        return None
    return float(rem)


async def read_platform_limits(drone, timeout_s: float = 5.0
                               ) -> Tuple[dict, List[str]]:  # pragma: no cover -- needs a link; driven by the fake in tests
    """Read `PLATFORM_PARAMS` from the vehicle. Returns (values, unread) where
    `unread` lists every parameter that did not come back and why. Never
    substitutes a default for a value it could not read."""
    import asyncio
    vals: dict = {}
    unread: List[str] = []
    for name, kind, _what in PLATFORM_PARAMS:
        getter = (drone.param.get_param_float if kind == "float"
                  else drone.param.get_param_int)
        try:
            vals[name] = await asyncio.wait_for(getter(name), timeout=timeout_s)
        except Exception as e:  # noqa: BLE001 -- any failure is "not read"
            unread.append(f"{name} ({type(e).__name__}: {e})")
    return vals, unread


def approach_transit_lower_bound_s(belief_range_m: float, v_cap_ms: float,
                                   brake_accel_ms2: float) -> float:
    """Shortest time an approach dash to a belief `belief_range_m` away can
    take: the transit at the speed ceiling, the braking tail, and a fixed 8 s
    terminal allowance. (The expression build_terminal has always printed.)"""
    return (belief_range_m / max(v_cap_ms, 1e-6)
            + v_cap_ms / max(brake_accel_ms2, 1e-6) / 2.0
            + 8.0)


def believed_rendezvous(r0_ne: Tuple[float, float], v_ne: Tuple[float, float],
                        v_own_ms: float, horizon_s: float
                        ) -> Tuple[Tuple[float, float], float, bool]:
    """Where and when a vehicle flying straight at `v_own_ms` from the launch
    point meets a target believed at `r0_ne` moving at `v_ne` (north, east).
    Returns (point_ne, t_s, catchable). When there is no meeting inside
    `horizon_s` (the target is as fast as the vehicle, or faster), returns the
    believed target position AT the horizon and catchable=False -- the farthest
    the engagement can drag the vehicle."""
    rn, re_ = float(r0_ne[0]), float(r0_ne[1])
    vn, ve = float(v_ne[0]), float(v_ne[1])
    a = vn * vn + ve * ve - v_own_ms * v_own_ms
    b = 2.0 * (rn * vn + re_ * ve)
    c = rn * rn + re_ * re_
    t_hit: Optional[float] = None
    if abs(a) < 1e-9:
        if abs(b) > 1e-9 and -c / b > 0.0:
            t_hit = -c / b
    else:
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            roots = [(-b - math.sqrt(disc)) / (2.0 * a),
                     (-b + math.sqrt(disc)) / (2.0 * a)]
            pos = [r for r in roots if r > 0.0]
            if pos:
                t_hit = min(pos)
    if t_hit is None or t_hit > horizon_s:
        t = float(horizon_s)
        return (rn + vn * t, re_ + ve * t), t, False
    return (rn + vn * t_hit, re_ + ve * t_hit), t_hit, True


def commanded_caps(args, cfg: MissionConfig, guidance) -> dict:
    """What the guidance may COMMAND, gathered in one place so it can be printed
    beside what the board will actually fly. Pure."""
    which = getattr(args, "terminal", "stock")
    t_e, t_n = (float(v) for v in str(args.target_start).split(",")[:2])
    v_e, v_n = (float(v) for v in str(args.target_vel).split(",")[:2])
    caps = {"terminal": which, "belief_r0_ne": (t_n, t_e), "belief_v_ne": (v_n, v_e),
            "approach_dash": False, "brake_accel_ms2": None}
    p = getattr(guidance, "cfg", None)
    if which == "pursuit" and p is not None:
        v = float(p.v_max_ms)
        accel = max(float(p.accel_max_ms2), float(p.accel_max_horiz_b_ms2))
        if getattr(p, "adaptive_speed", False):
            v = max(v, float(p.v_hw_max_ms))
        if getattr(p, "approach_dash", False):
            v = max(v, float(p.dash_v_max_ms))
            accel = max(accel, float(p.dash_accel_max_ms2))
            caps["approach_dash"] = True
            caps["brake_accel_ms2"] = float(p.brake_accel_ms2)
            caps["brake_vert_sync"] = bool(getattr(p, "brake_vert_sync", False))
        if getattr(p, "arrival_search", False):
            caps["arrival_search"] = str(p.search_start)
            caps["search_sweep_ms"] = float(p.search_sweep_ms)
            caps["search_drop_ms"] = float(p.search_drop_ms)
            caps["search_sigma_speed_frac"] = float(p.search_sigma_speed_frac)
        caps["stale_decode_guard"] = bool(getattr(p, "stale_decode_guard", False))
        caps.update(v_cap_ms=v, accel_ms2=accel)
    else:
        caps.update(v_cap_ms=float(cfg.dash_speed_ms),
                    accel_ms2=float(cfg.dash_accel_cap_ms2 or cfg.dash_accel_ms2))
    return caps


def check_platform_limits(limits: dict, unread: Sequence[str], caps: dict,
                          cfg: MissionConfig) -> Tuple[List[str], List[str], List[str]]:
    """Compare what the guidance commands with what the board will fly.
    Returns (hard, refusals, notes):

      hard      -- cannot be overridden: a limit that was not READ. A number
                   that should have been measured is never replaced by a guess.
      refusals  -- `--fly` stops on these unless --override-platform-limits:
                   a commanded speed above the board's, a believed rendezvous
                   outside the geofence, a standby height above its ceiling, a
                   transit that cannot fit the engage window at the speed the
                   board allows.
      notes     -- printed, never blocking.
    Pure."""
    hard = [f"not read from the vehicle: {u}" for u in unread]
    refusals: List[str] = []
    notes: List[str] = []
    v_cmd = float(caps["v_cap_ms"])
    v_plat = limits.get("MPC_XY_VEL_MAX")
    v_eff = v_cmd
    if v_plat is not None:
        v_eff = min(v_cmd, float(v_plat))
        if v_cmd > float(v_plat) + 1e-6:
            refusals.append(
                f"the guidance commands up to {v_cmd:.0f} m/s but the board "
                f"flies at most {float(v_plat):.0f} m/s (MPC_XY_VEL_MAX): it "
                f"would silently fly slower than every timing assumes")
    r0, v0 = caps["belief_r0_ne"], caps["belief_v_ne"]
    tgt_speed = math.hypot(v0[0], v0[1])
    belief_range = math.hypot(r0[0], r0[1])
    horizon = (cfg.engage_max_s if math.isfinite(cfg.engage_max_s)
               else _UNBOUNDED_HORIZON_S)
    point, t_meet, catchable = believed_rendezvous(r0, v0, v_eff, horizon)
    d_meet = math.hypot(point[0], point[1])
    if not catchable:
        notes.append(
            f"at {v_eff:.0f} m/s the vehicle does not reach the believed target "
            f"({tgt_speed:.1f} m/s, {belief_range:.0f} m away) inside "
            + (f"the {cfg.engage_max_s:.0f} s engage window"
               if math.isfinite(cfg.engage_max_s)
               else f"{horizon:.0f} s (there is no engage time limit)")
            + f"; the chase can carry it {d_meet:.0f} m from launch")
    gf_act = limits.get("GF_ACTION")
    gf_h = limits.get("GF_MAX_HOR_DIST")
    gf_v = limits.get("GF_MAX_VER_DIST")
    if gf_act is not None and int(gf_act) != 0:
        if gf_h is not None and float(gf_h) > 0.0 and d_meet > float(gf_h):
            refusals.append(
                f"the believed rendezvous is {d_meet:.0f} m from launch, "
                f"outside the {float(gf_h):.0f} m geofence (GF_MAX_HOR_DIST, "
                f"action {int(gf_act)}): PX4 would leave OFFBOARD on the way")
        if gf_v is not None and float(gf_v) > 0.0 \
                and cfg.standby_alt_m > float(gf_v):
            refusals.append(
                f"standby height {cfg.standby_alt_m:.1f} m is above the "
                f"{float(gf_v):.1f} m geofence ceiling (GF_MAX_VER_DIST)")
    elif gf_act is not None:
        notes.append("geofence action is 0 (none): nothing on the board bounds "
                     "how far the chase can go")
    if caps.get("approach_dash") and caps.get("brake_accel_ms2"):
        t_min = approach_transit_lower_bound_s(belief_range, v_eff,
                                               caps["brake_accel_ms2"])
        if cfg.engage_max_s < t_min:
            refusals.append(
                f"the approach dash needs at least {t_min:.0f} s to cover the "
                f"{belief_range:.0f} m cue at the {v_eff:.0f} m/s the board "
                f"allows, but the engage window is {cfg.engage_max_s:.0f} s "
                f"(--engage-max-s): it would time out mid-dash")
    acc_plat = limits.get("MPC_ACC_HOR_MAX")
    if acc_plat is not None and caps["accel_ms2"] > float(acc_plat) + 1e-6:
        notes.append(f"the guidance slews its command at up to "
                     f"{caps['accel_ms2']:.0f} m/s^2; the board accelerates at "
                     f"most {float(acc_plat):.0f} (MPC_ACC_HOR_MAX)")
    rcl = limits.get("COM_RCL_EXCEPT")
    if rcl is not None and not (int(rcl) & 0b100):
        notes.append("RC loss in OFFBOARD triggers PX4's RC-loss failsafe "
                     "(COM_RCL_EXCEPT has no offboard bit): switching the "
                     "transmitter off after GO ends the intercept")
    of_t, of_act = limits.get("COM_OF_LOSS_T"), limits.get("COM_OBL_RC_ACT")
    if of_t is not None and of_act is not None:
        notes.append(f"if the setpoint stream stops, PX4 waits {float(of_t):.1f} s "
                     f"(COM_OF_LOSS_T) and then takes action {int(of_act)} "
                     f"(COM_OBL_RC_ACT; 0 = Position mode)")
    return hard, refusals, notes


def format_platform_table(limits: dict, caps: dict) -> List[str]:
    """The board's limits beside the guidance's caps, one line each."""
    lines = [f"[limits] guidance: terminal {caps['terminal']}, commands up to "
             f"{caps['v_cap_ms']:.0f} m/s, slews at up to {caps['accel_ms2']:.0f} "
             f"m/s^2" + (", approach dash ON" if caps.get("approach_dash") else "")]
    for name, _kind, what in PLATFORM_PARAMS:
        val = limits.get(name)
        lines.append(f"[limits] board:    {name:<16} = "
                     f"{'NOT READ' if val is None else val}   ({what})")
    return lines


# What the supervisor must find in a process's command line before it will kill
# it. A module constant so a test can point it at a stand-in process.
_SERVER_MATCH = "mavsdk_server"
_SUPERVISOR_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "loop_supervisor.py")


class LoopSupervisor:
    """This program's end of flight/deploy/loop_supervisor.py (read that file's
    header for the fault it exists for). Usage: `start(server_pid)` once the
    MAVSDK server exists; `watch()` when the control loop begins; `beat()` every
    tick; `unwatch()` before any deliberate pause (teardown); `close()` at exit.

    The pipe is NON-BLOCKING: a watchdog that could stall the loop it watches
    would be its own fault. A write that fails marks the supervisor LOST, and
    the driver stops flying on a lost supervisor rather than carry on unwatched."""

    def __init__(self, bound_s: float, log_path: Optional[str] = None,
                 match: Optional[str] = None):
        self.bound_s = float(bound_s)
        self.log_path = log_path
        self.match = _SERVER_MATCH if match is None else match
        self._proc = None
        self.lost = False

    def start(self, server_pid) -> bool:
        """`server_pid`: one pid, or a list of them (primary + backup link
        servers). Every one is killed when the watchdog trips."""
        import subprocess
        pids = ([int(server_pid)] if isinstance(server_pid, int)
                else [int(p) for p in server_pid])
        cmd = [sys.executable, _SUPERVISOR_SCRIPT]
        for p in pids:
            cmd += ["--server-pid", str(p)]
        cmd += ["--bound-s", f"{self.bound_s:.3f}", "--match", self.match]
        if self.log_path:
            cmd += ["--log", self.log_path]
        try:
            # Its own session: a terminal Ctrl-C or a dropped SSH session must
            # reach this program, not the thing watching it.
            self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                          stdout=subprocess.DEVNULL,
                                          start_new_session=True, close_fds=True)
            os.set_blocking(self._proc.stdin.fileno(), False)
        except OSError as e:
            print(f"[watchdog] FAULT: could not start the loop supervisor: {e}")
            self._proc = None
            self.lost = True
            return False
        return True

    def _send(self, b: bytes) -> bool:
        if self._proc is None or self.lost:
            return False
        try:
            os.write(self._proc.stdin.fileno(), b)
            return True
        except (BlockingIOError, BrokenPipeError, OSError, ValueError):
            self.lost = True
            return False

    def watch(self) -> bool:
        """Start timing the gap between beats (the control loop has begun)."""
        return self._send(b"W")

    def beat(self) -> bool:
        return self._send(b".")

    def unwatch(self) -> bool:
        """Stop timing: a deliberate pause follows (teardown, a clean exit)."""
        return self._send(b"U")

    def alive(self) -> bool:
        return (self._proc is not None and not self.lost
                and self._proc.poll() is None)

    def exit_code(self) -> Optional[int]:
        return None if self._proc is None else self._proc.poll()

    def close(self, wait_s: float = 2.0) -> Optional[int]:
        """Clean goodbye: disarm, close the pipe, wait for it to go."""
        if self._proc is None:
            return None
        self.unwatch()
        try:
            self._proc.stdin.close()
        except OSError:
            pass
        try:
            return self._proc.wait(timeout=wait_s)
        except Exception:  # noqa: BLE001 -- it ignores SIGTERM by design
            self._proc.kill()
            return self._proc.poll()


# ---------------------------------------------------------------- MAVSDK driver

# Timeouts mirror flight/deploy/seeker_loop.py (same PX4 SITL boot, same traps).
_CONNECT_TIMEOUT_S = 30.0
_HEALTH_TIMEOUT_S = 60.0
_TAKEOFF_TIMEOUT_S = 60.0
_TAKEOFF_ALT_FRAC = 0.8
_LAND_TIMEOUT_S = 60.0
# Bounded wait for the FIRST telemetry.flight_mode() sample after offboard.start().
# Only affects how loudly we report; the gate fails closed on None regardless.
_MODE_FIRST_SAMPLE_TIMEOUT_S = 10.0



def _stamped_det_age(d, t_mission, now_monotonic):
    """Stamped measurement age for --pursuit-stamped-age, or None.

    Two publisher conventions, by attribute name (review F1, 2026-10-22):
    - `t_capture`: MISSION-RELATIVE capture stamp (FeedDetector's convention:
      t_mission - (clock - t_cap_clock)); aged against mission time.
    - `t_capture_monotonic`: ABSOLUTE time.monotonic()-axis stamp (the cinema
      driver under --det-stamp-capture; the SimClockShim makes monotonic()
      absolute sim time); aged against now_monotonic.
    Publishing an absolute stamp as `t_capture` clamps every age to 0.0 and
    silently un-ages the measurements -- the F1 bug; the split consumption
    here plus tests/test_det_stamp_capture.py pin the convention from both
    sides. The Gazebo harness's own `t_capture_sim` is a third attribute
    name and is deliberately never read.
    """
    t_cap = getattr(d, "t_capture", None)
    if t_cap is not None:
        return max(0.0, t_mission - float(t_cap))
    t_cap_mono = getattr(d, "t_capture_monotonic", None)
    if t_cap_mono is not None:
        return max(0.0, now_monotonic - float(t_cap_mono))
    return None


# Consecutive control ticks on which the state machine may raise, or a setpoint
# send may fail, before the loop stops and teardown takes the aircraft. At the
# 20 Hz loop rate 5 ticks is 0.25 s: longer than a one-off, far shorter than
# PX4's own 1.0 s offboard-loss time (COM_OF_LOSS_T on this board).
_STEP_FAULTS_TO_ABORT = 5
_SEND_FAULTS_TO_ABORT = 5


class _DetMiss:
    """What the control loop uses in place of a detector result that raised."""
    is_new = False
    range_m = None
    box_xywh = None
    frame = None
    tag_range_m = None


_DET_MISS = _DetMiss()


def _camera_age(detector) -> Optional[float]:
    """Age of the live camera feed's newest frame, or None when the detector
    has no live feed (synthetic, replay). Never raises."""
    feed = getattr(detector, "feed", None)
    fn = getattr(feed, "frame_age_s", None)
    if fn is None:
        return None
    try:
        age = fn()
        return None if age is None else float(age)
    except Exception:  # noqa: BLE001 -- a health read must not end the loop
        return None


def _det_is_finite(d) -> bool:
    """False when a detection carries a non-finite number. One NaN box seeds
    the terminal's range filter with NaN for the rest of the flight (review
    2026-11-26), so it is stopped at the door and counted as a miss."""
    r = getattr(d, "range_m", None)
    if r is not None and not math.isfinite(float(r)):
        return False
    box = getattr(d, "box_xywh", None)
    if box is not None:
        try:
            if len(box) != 4 or not all(math.isfinite(float(x)) for x in box):
                return False
        except (TypeError, ValueError):
            return False
    pr = getattr(d, "tag_range_m", None)
    if pr is not None and not math.isfinite(float(pr)):
        return False
    return True


_BENCH_STREAM_WAIT_S = 10.0      # bench: wait this long for armed + battery samples
_ARM_PERMIT_POLL_S = 0.05

# The own-state streams the guidance consumes, by the key their arrival stamp
# is stored under. FAILSAFE 9 ages the oldest of these.
_OWN_STATE_STAMPS = ("t_quat", "t_euler", "t_pos", "t_vel")


def _all_finite(values) -> bool:
    try:
        return all(v is not None and math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False


def _new_link_state() -> dict:
    """The driver's one mutable dict of own-state + own status. `offboard` and
    `mode` start None and are owned by the flight_mode SUBSCRIBER -- never by
    "offboard.start() returned". arm_gate uses `is not True`, so None fails
    CLOSED (no GO can be accepted), and standby_settle_s=2.0 means no GO is
    possible inside the first-sample window."""
    return {"quat": None, "psi": None, "alt": None, "armed": None, "vel_ned": None,
            "landed": None, "offboard": None, "mode": None,
            "t_quat": None, "t_euler": None, "t_pos": None, "t_vel": None,
            "t_mode": None, "gs": None,
            "batt_v": None, "t_batt": None,
            "rc_n": 0, "t_rc": None, "rc_max_gap_s": 0.0, "t_rc_first": None,
            # own GPS position (lat deg, lon deg, alt m AMSL): what the cue's
            # target position is placed against at the GO edge (--trigger cue)
            "lla": None, "systime_n": 0,
            # the GPS RECEIVER's own height above sea level and when it came.
            # The Pixhawk's reported height is the barometer's when the board's
            # height reference is the barometer (measured 2026-11-26: 97 m
            # below the receiver's), so it must not be differenced with the
            # target's GPS height.
            "gps_amsl": None, "t_gps_amsl": None,
            # own-state samples dropped for carrying a non-finite number
            "n_nonfinite": 0,
            # battery charge left, 0..1 as PX4 reports it (None = no reading)
            "batt_rem": None,
            # GPS quality (pre-arm gate only; nothing reads it after GO)
            "gps_sats": None, "gps_fix": None, "gps_hacc": None,
            "gps_sacc": None, "t_gps": None}


# MAVSDK's `Battery.remaining_percent` is taken as 0..100. DERIVED, NOT YET
# MEASURED WITH A PACK CONNECTED: with no battery on the power module the Pi's
# mavsdk 3.15.3 reports -1.0 (runs/interceptor_redundancy_20261126/), which is
# MAVLink's own "unknown" value for the 0..100 percent field passed through
# unscaled; a 0..1 scale would have shown -0.01. `--fly` prints the figure it
# reads before the arm permit ("battery NN%"): check that line against the
# pack on the first battery-on run. If it ever read 100 times too small the
# program would refuse to arm (fails closed), not fly on a wrong number.
_BATTERY_PERCENT_SCALE = 100.0


def _start_telemetry(drone, state: dict, rc_sinks=(), want_battery: bool = False,
                     utc_sink=None, want_gps: bool = False,
                     want_gps_height: bool = False):  # pragma: no cover -- driven by the fake in tests
    """Subscribe to every stream the driver reads and return the tasks. Each
    own-state stream STAMPS its arrival (`t_quat`, `t_euler`, `t_pos`, `t_vel`):
    a generator that dies leaves its slice of `state` frozen at the last value,
    and only the age of the stamp can say so (FAILSAFE 9).

    Read-only: subscriptions only. `rc_sinks` are the `RcChannelTrigger`s fed
    from the one RC_CHANNELS subscription (the GO switch, and under --fly the
    arm-permit switch)."""
    import asyncio

    async def _att_q():
        async for q in drone.telemetry.attitude_quaternion():
            v = (q.w, q.x, q.y, q.z)
            if not _all_finite(v):
                state["n_nonfinite"] += 1      # dropped: the stamp ages instead
                continue
            state["quat"] = v
            state["t_quat"] = time.monotonic()

    async def _att_e():
        async for e in drone.telemetry.attitude_euler():
            if not _all_finite((e.yaw_deg,)):
                state["n_nonfinite"] += 1
                continue
            state["psi"] = e.yaw_deg
            state["t_euler"] = time.monotonic()

    async def _pos():
        async for p in drone.telemetry.position():
            if not _all_finite((p.relative_altitude_m,)):
                state["n_nonfinite"] += 1
                continue
            state["alt"] = p.relative_altitude_m
            state["t_pos"] = time.monotonic()
            lat = getattr(p, "latitude_deg", None)
            lon = getattr(p, "longitude_deg", None)
            amsl = getattr(p, "absolute_altitude_m", None)
            if lat is not None and lon is not None and amsl is not None \
                    and all(math.isfinite(float(x)) for x in (lat, lon, amsl)):
                state["lla"] = (float(lat), float(lon), float(amsl))

    async def _vel():
        """Own-state EKF horizontal ground speed -- FAILSAFE 4 integrates this
        instead of dead-reckoning off the aim-fitted dash_accel_ms2."""
        async for v in drone.telemetry.velocity_ned():
            if not _all_finite((v.north_m_s, v.east_m_s, v.down_m_s)):
                # PX4 sends NaN for a velocity it holds invalid. Stored, it
                # made the pursuit terminal's belief NaN for the rest of the
                # flight (review 2026-11-26). Dropped, the stamp ages and the
                # terminal falls back to its last commanded velocity.
                state["n_nonfinite"] += 1
                continue
            state["gs"] = math.hypot(v.north_m_s, v.east_m_s)
            # ADR-0117 (endgame-estimator diagnosis): the FULL EKF velocity
            # vector, not just the scalar. Without it VehicleObs.vel_ned
            # stayed None and the pursuit KF fell back to the last COMMANDED
            # velocity on every tick of every campaign flight (16/16 FAULT
            # lines) -- measured 3.6-4.2 m/s off actual, the driver of the
            # -2.6 m endgame range collapse and the ~3 m early coast latch.
            state["vel_ned"] = (v.north_m_s, v.east_m_s, v.down_m_s)
            state["t_vel"] = time.monotonic()

    async def _armed():
        async for a in drone.telemetry.armed():
            state["armed"] = a

    async def _landed():
        async for ls in drone.telemetry.landed_state():
            state["landed"] = ls

    async def _batt():
        async for b in drone.telemetry.battery():
            state["batt_v"] = b.voltage_v
            state["t_batt"] = time.monotonic()
            rem = getattr(b, "remaining_percent", None)
            # A negative figure is PX4's "not known" (no battery monitor
            # reading); it is no reading, never "empty".
            state["batt_rem"] = (rem / _BATTERY_PERCENT_SCALE
                                 if rem is not None and _all_finite((rem,))
                                 and rem >= 0.0 else None)

    async def _gps_info():
        async for g in drone.telemetry.gps_info():
            state["gps_sats"] = int(g.num_satellites)
            fix = g.fix_type
            state["gps_fix"] = int(getattr(fix, "value", fix))
            state["t_gps"] = time.monotonic()
            # A short history, so the pre-arm check judges the last few
            # seconds without making the operator wait for them.
            hist = state.setdefault("gps_hist", [])
            hist.append({"t": state["t_gps"], "sats": state["gps_sats"],
                         "fix": state["gps_fix"], "hacc": state["gps_hacc"],
                         "sacc": state["gps_sacc"]})
            del hist[:-60]

    async def _raw_gps():
        async for r in drone.telemetry.raw_gps():
            state["gps_hacc"] = float(r.horizontal_uncertainty_m)
            state["gps_sacc"] = float(r.velocity_uncertainty_m_s)
            amsl = getattr(r, "absolute_altitude_m", None)
            if amsl is not None and math.isfinite(float(amsl)):
                state["gps_amsl"] = float(amsl)
                state["t_gps_amsl"] = time.monotonic()

    # THE OFFBOARD INPUT. Without this subscriber `state["offboard"]` was a
    # latched constant and the arm gate's OFFBOARD check was structurally
    # unreachable in flight (review2 BLOCKER).
    _mode = make_mode_subscriber(drone, state)

    async def _rc():
        """RC_CHANNELS -> the ELRS aux switch(es). MAVSDK's telemetry plugin has
        no per-channel accessor, so this uses the mavlink_direct raw subscription.

        CLOCK: absolute time.monotonic(), matching RcChannelTrigger.clock_base
        (the driver polls it through `trigger_poll_time`)."""
        async for msg in drone.mavlink_direct.message("RC_CHANNELS"):
            # NARROW: a programming error in here must not masquerade as "a
            # malformed RC_CHANNELS" forever (that swallow hid the clock defect).
            try:
                now = time.monotonic()
                fields = json.loads(msg.fields_json)
                for sink in rc_sinks:
                    sink.ingest(fields, now)
                if state["t_rc"] is not None:
                    state["rc_max_gap_s"] = max(state["rc_max_gap_s"],
                                                now - state["t_rc"])
                else:
                    state["t_rc_first"] = now
                state["t_rc"] = now
                state["rc_n"] += 1
            except (json.JSONDecodeError, KeyError, TypeError, ValueError,
                    AttributeError) as e:
                print(f"[rc] skipped a malformed RC_CHANNELS: {e}")

    async def _systime():
        """SYSTEM_TIME -> this machine's monotonic-to-UTC mapping
        (flight.cue_link.MonoToUtc), so the cue link can measure each record's
        delay against the vehicle's own GPS-disciplined clock."""
        async for msg in drone.mavlink_direct.message("SYSTEM_TIME"):
            try:
                fields = json.loads(msg.fields_json)
                utc_sink.ingest(float(fields["time_unix_usec"]), time.monotonic())
                state["systime_n"] += 1
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
                print(f"[cue] skipped a malformed SYSTEM_TIME: {e}")

    coros = [_att_q, _att_e, _pos, _vel, _armed, _landed, _mode]
    if want_battery:
        coros.append(_batt)
    if want_gps:
        coros += [_gps_info, _raw_gps]
    elif want_gps_height:
        coros.append(_raw_gps)
    if rc_sinks:
        coros.append(_rc)
    if utc_sink is not None:
        coros.append(_systime)
    tasks = [asyncio.ensure_future(c()) for c in coros]

    def _task_died(task):
        """A telemetry generator that raises dies SILENTLY and freezes its slice
        of own-state at the last value forever. Say so."""
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            print(f"[mavsdk] FAULT telemetry_task_died: {type(exc).__name__}: "
                  f"{exc} -- an own-state stream is now FROZEN")

    for tk in tasks:
        tk.add_done_callback(_task_died)
    return tasks


async def _connect_vehicle(drone, url) -> bool:  # pragma: no cover -- driven by the fake in tests
    """Connect and wait for the link. False (after killing the server it
    spawned) on a dead link."""
    import asyncio

    def _kill_server():
        # MAVSDK-Python spawns a mavsdk_server child per System(); kill it if we
        # bail out, so a failed connect never leaks a server (the trap measured
        # 2026-07-24 in seeker_loop.py / fc_link_check.py).
        proc = getattr(drone, "_server_process", None)
        if proc is not None and proc.poll() is None:
            proc.kill()

    print(f"[mavsdk] connecting {url} ...")
    # connect() itself blocks forever on a dead link (mavsdk_server does not open
    # its gRPC port until it discovers a vehicle), so the timeout MUST wrap it.
    try:
        await asyncio.wait_for(drone.connect(system_address=url),
                               timeout=_CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        print(f"[mavsdk] FAIL: connect() did not return within {_CONNECT_TIMEOUT_S}s")
        _kill_server()
        return False

    async def _wait_connected():
        async for st in drone.core.connection_state():
            if st.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_connected(), timeout=_CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        print(f"[mavsdk] FAIL: no connection within {_CONNECT_TIMEOUT_S}s")
        _kill_server()
        return False
    print("[mavsdk] connected")
    return True


# gRPC ports for the two MAVSDK servers of a flight with a backup link (each
# `System` starts its own server; the default port is 50051).
_GRPC_PORT_PRIMARY = 50051
_GRPC_PORT_BACKUP = 50052
# MAVLink component id of the backup link's server, so the Pixhawk (which
# forwards traffic between its ports) can tell the two servers apart.
_BACKUP_COMPID = 191


_GPS_PREARM_WINDOW_S = 3.0
_GPS_PREARM_MIN_SAMPLES = 3


def board_failsafes_mode(args) -> str:
    """'suspend' or 'keep'. `--fly` defaults to suspend (builder ruling
    2026-11-26); typing the flag always wins."""
    v = getattr(args, "board_failsafes", None)
    if v in ("keep", "suspend"):
        return v
    return "suspend" if getattr(args, "fly", False) else "keep"


def _board_links(drone):
    """(the link to write board settings on, the link to read them back on).
    With two live links the read-back uses the OTHER one, so it cannot be
    answered from the writing server's memory."""
    links = getattr(drone, "links", None)
    if links and len(links) == 2 and all(ln.alive for ln in links):
        return links[0].system, links[1].system
    return drone, drone


async def _sample_gps(state: dict, window_s: float) -> List[dict]:  # pragma: no cover -- driven by the fake in tests
    """The GPS quality readings of the last `window_s`, from the history the
    telemetry keeps. Waits (at most `window_s`) only until a few complete
    readings exist; it never pauses a program that already has them."""
    import asyncio

    def recent() -> List[dict]:
        now = time.monotonic()
        return [h for h in state.get("gps_hist", [])
                if now - h["t"] <= window_s and h["hacc"] is not None
                and h["sacc"] is not None]

    end = time.monotonic() + float(window_s)
    while len(recent()) < _GPS_PREARM_MIN_SAMPLES and time.monotonic() < end:
        await asyncio.sleep(0.1)
    return recent()


# Every MAVSDK `System` this process opened. The library reads each server's
# output on a NON-daemon thread and kills the server only from an atexit hook,
# and Python joins non-daemon threads BEFORE it runs atexit hooks: a run that
# ended with a server still alive never exited (bench runs 1 and 4, 2026-11-26:
# the report printed, the process stayed, and it kept the serial port). So the
# program ends its own servers when it is done (`end_opened_servers`).
_OPENED_SYSTEMS: list = []


def end_opened_servers() -> int:
    """End every MAVSDK server this process started. Called once, last, when
    the run is over and its teardown has been done. Returns how many it ended."""
    n = 0
    while _OPENED_SYSTEMS:
        drone = _OPENED_SYSTEMS.pop()()      # weak: holds no vehicle object alive
        if drone is not None and _end_setpoint_stream(drone):
            n += 1
    return n


async def _open_vehicle(args):  # pragma: no cover -- driven by the fake in tests
    """Connect the MAVLink link(s) and return the object the driver talks to,
    or None (after ending any server it started) when a link will not connect.

    One link (`--mavsdk-url` alone): a plain MAVSDK `System`, exactly as
    before. Two (`--mavsdk-backup-url` as well): a
    `flight.deploy.link_redundancy.RedundantSystem` over both -- read that
    module's header for what it does. A backup that was asked for and does not
    connect is a REFUSAL, not a quiet single-link flight."""
    from mavsdk import System
    backup_url = getattr(args, "mavsdk_backup_url", None)
    if not backup_url:
        drone = System()
        _OPENED_SYSTEMS.append(weakref.ref(drone))
        return drone if await _connect_vehicle(drone, args.mavsdk_url) else None
    from flight.deploy.link_redundancy import RedundantSystem
    primary = System(port=_GRPC_PORT_PRIMARY)
    _OPENED_SYSTEMS.append(weakref.ref(primary))
    if not await _connect_vehicle(primary, args.mavsdk_url):
        return None
    backup = System(port=_GRPC_PORT_BACKUP, compid=_BACKUP_COMPID)
    _OPENED_SYSTEMS.append(weakref.ref(backup))
    if not await _connect_vehicle(backup, backup_url):
        print(f"[link] REFUSED: the backup link {backup_url} did not connect. "
              f"Fix it (is the flight recorder running?) or drop "
              f"--mavsdk-backup-url to fly on one link knowingly.")
        _end_setpoint_stream(primary)
        return None
    print(f"[link] TWO LINKS: primary {args.mavsdk_url}, backup {backup_url}. "
          f"Setpoints go out on both; telemetry is read from the primary and "
          f"from the backup only when the primary is silent.")
    # The driver's own clock (the one its telemetry stamps use), so the quiet
    # times are measured on the same base as FAILSAFE 9's ages.
    return RedundantSystem(primary, backup, clock=lambda: time.monotonic())


def _server_pids(drone) -> List[int]:
    """Process ids of every MAVSDK server behind `drone` (one, or two with a
    backup link)."""
    procs = getattr(drone, "_server_processes", None)
    if procs is None:
        one = getattr(drone, "_server_process", None)
        procs = [] if one is None else [one]
    return [p.pid for p in procs if getattr(p, "pid", None) is not None]


def _start_supervisor(drone, args, required: bool):
    """Start the out-of-process loop watchdog on this run's MAVSDK server.
    Returns (supervisor_or_None, ok). `ok` is False only when the watchdog is
    `required` (a real flight, or the bench rehearsal of one) and could not be
    started -- the caller then refuses to go on."""
    bound = float(getattr(args, "loop_watchdog_s", 0.0) or 0.0)
    if bound <= 0.0:
        print("[watchdog] OFF (--loop-watchdog-s 0): a hung program would keep "
              "the aircraft flying its last command")
        return None, not required
    pids = _server_pids(drone)
    n_links = len(getattr(drone, "links", [None]))
    if not pids or len(pids) < n_links:
        print(f"[watchdog] {len(pids)} of {n_links} MAVSDK server process(es) "
              f"can be watched (an external server was supplied?) -- the "
              f"setpoint stream is UNWATCHED")
        return None, not required
    log_csv = getattr(args, "log_csv", None)
    log_path = None
    if log_csv:
        # Beside the tick log, and created NOW: the watchdog's one line of
        # evidence is written at the moment the tick log may never be.
        log_path = log_csv + ".watchdog.log"
        try:
            os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
        except OSError:
            log_path = None
    sup = LoopSupervisor(bound, log_path=log_path)
    if not sup.start(pids):
        return None, not required
    print(f"[watchdog] loop supervisor watching server pid(s) "
          f"{', '.join(map(str, pids))}: no control "
          f"tick for {bound:.2f} s, or this program dying, ends the setpoint "
          f"stream")
    return sup, True


async def _enter_offboard(drone, cfg: MissionConfig, state: dict) -> bool:  # pragma: no cover -- driven by the fake in tests
    """Prime one setpoint, start OFFBOARD, and wait (bounded, LOUD) for the
    first flight-mode sample. Returns whether `offboard.start()` succeeded --
    a different fact from "the vehicle IS in OFFBOARD", which only telemetry
    may answer."""
    import asyncio
    from mavsdk.offboard import OffboardError, VelocityNedYaw

    # OFFBOARD needs an established setpoint stream before the mode switch.
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0,
                                                         cfg.aim_yaw_deg))
    try:
        await drone.offboard.start()
        # "start() SUCCEEDED" -- a different fact from "the vehicle IS in
        # OFFBOARD right now", which only telemetry may answer. Do NOT latch
        # state["offboard"] here (review2 BLOCKER).
        print("[mavsdk] OFFBOARD start accepted; STANDBY hold stream begins")
    except OffboardError as e:
        print(f"[mavsdk] FAIL: offboard start failed: {e}")
        return False

    # Wait (bounded, LOUD) for the first flight_mode sample so the arm gate is
    # not merely failing closed on a warm-up None.
    _deadline = time.monotonic() + _MODE_FIRST_SAMPLE_TIMEOUT_S
    while time.monotonic() < _deadline and state["offboard"] is None:
        await asyncio.sleep(0.1)
    if state["offboard"] is None:
        print(f"[mavsdk] WARNING: no telemetry.flight_mode() sample within "
              f"{_MODE_FIRST_SAMPLE_TIMEOUT_S:.0f}s -- offboard_active "
              f"stays None and the arm gate will (correctly) REFUSE every "
              f"GO. This is fail-closed, not a silent pass.")
    else:
        print(f"[mavsdk] flight mode = {state['mode']} "
              f"(offboard_active={state['offboard']}) from TELEMETRY")
    return True


def _rc_readers(*triggers) -> list:
    """Every RC_CHANNELS reader among `triggers`: the RC triggers themselves and
    a cue trigger's enable switch."""
    out = []
    for tr in triggers:
        if isinstance(tr, RcChannelTrigger):
            out.append(tr)
        elif isinstance(tr, CueTrigger) and tr.enable is not None:
            out.append(tr.enable)
    return out


# The GPS receiver's height must be this fresh to place the target against.
CUE_OWN_GPS_HEIGHT_MAX_AGE_S = 2.0


def cue_own_lla(state: dict, now: Optional[float] = None
                ) -> Optional[Tuple[float, float, float]]:
    """Own position for placing the cued target: the Pixhawk's latitude and
    longitude with the GPS RECEIVER's height above sea level. The target
    reports GPS height; the Pixhawk's own reported height is the barometer's
    when the board's height reference is the barometer, and the two differed
    by 97 m on the ground on 2026-11-26, which latched the target 101 m
    overhead. No fresh receiver height means no answer (GO waits): the
    Pixhawk's height is never substituted."""
    lla = state.get("lla")
    amsl, t = state.get("gps_amsl"), state.get("t_gps_amsl")
    if lla is None or amsl is None or t is None:
        return None
    now = time.monotonic() if now is None else now
    if now - t > CUE_OWN_GPS_HEIGHT_MAX_AGE_S:
        return None
    return (lla[0], lla[1], float(amsl))


def _wire_cue_trigger(trigger, state: dict) -> bool:
    """Give a cue trigger the driver's own GPS position (own-state). Returns
    whether the telemetry must carry the GPS receiver's height."""
    if isinstance(trigger, CueTrigger):
        trigger.own_lla = lambda: cue_own_lla(state)
        return True
    return False


def make_cue_go_check(limits: dict, caps: dict, max_range_m: Optional[float]):
    """The GO-time half of the pre-arm limit check, for `--trigger cue`: at arm
    time the real belief does not exist yet, so the questions that need it are
    asked when it does. Returns f(solution) -> refusal text or None. Pure."""
    v_cmd = float(caps["v_cap_ms"])
    v_plat = limits.get("MPC_XY_VEL_MAX")
    v_eff = min(v_cmd, float(v_plat)) if v_plat is not None else v_cmd
    gf_act, gf_h = limits.get("GF_ACTION"), limits.get("GF_MAX_HOR_DIST")

    def check(sol) -> Optional[str]:
        r0 = (float(sol.belief_r0_ned[0]), float(sol.belief_r0_ned[1]))
        v0 = (float(sol.belief_vel0_ned[0]), float(sol.belief_vel0_ned[1]))
        rng = math.hypot(*r0)
        if max_range_m is not None and rng > float(max_range_m):
            return (f"the cued target is {rng:.0f} m away, beyond "
                    f"--cue-max-range-m {float(max_range_m):.0f}")
        point, _t, catchable = believed_rendezvous(r0, v0, v_eff,
                                                   _UNBOUNDED_HORIZON_S)
        if not catchable:
            return (f"at the {v_eff:.0f} m/s this aircraft can fly it does not "
                    f"reach the cued target ({math.hypot(*v0):.1f} m/s, "
                    f"{rng:.0f} m away) within {_UNBOUNDED_HORIZON_S:.0f} s")
        d_meet = math.hypot(point[0], point[1])
        if gf_act is not None and int(gf_act) != 0 and gf_h is not None \
                and float(gf_h) > 0.0 and d_meet > float(gf_h):
            return (f"the believed rendezvous is {d_meet:.0f} m from here, "
                    f"outside the {float(gf_h):.0f} m geofence")
        return None

    return check


async def _control_loop(drone, args, cfg: MissionConfig, sm: RealFlightSM,
                        trigger, detector, state: dict, frame_recorder,
                        supervisor, *, outer_bound_s: float,
                        acquire_after_s: float, bench: bool = False,
                        bench_max_batt_v: Optional[float] = None,
                        out: Optional[dict] = None,
                        tick_log: Optional["TickLog"] = None,
                        handover: Optional["HandoverLimits"] = None) -> dict:  # pragma: no cover -- driven by the fake in tests
    """THE control loop: observe, step the state machine, send one setpoint,
    feed the watchdog. Shared by the flight driver and the props-off bench
    rehearsal ON PURPOSE -- a rehearsal that ran different loop code would
    rehearse nothing.

    It sends velocity setpoints and NOTHING else: no arm, no takeoff, no land,
    no mode change (a test walks the syntax tree to prove it).

    `bench=True` (props-off rehearsal) changes exactly two things: the height
    fed to the state machine is the standby height (a board on a desk is not at
    7 m, and the height gate is not what is being rehearsed), and the loop
    ABORTS the instant the vehicle reports armed or a flight battery appears."""
    import asyncio
    from mavsdk.offboard import VelocityNedYaw

    # `out` belongs to the CALLER when it passes one: if this loop is left by an
    # exception, the rows and counters gathered so far are still in the
    # caller's hands (they used to be lost with this frame).
    if out is None:
        out = {}
    out.update({"rc_exit": 0, "n_tick": 0, "n_sp": 0, "n_off_true": 0,
                "offboard_lost_after_go": False, "abort": None,
                "max_tick_gap_s": 0.0, "bound_reached": False,
                "n_step_faults": 0, "n_det_faults": 0})
    out.setdefault("csv_rows", [])
    csv_rows: List[str] = out["csv_rows"]
    last_sp = None                 # the last setpoint actually sent
    step_faults_running = 0
    send_faults_running = 0
    n_tick = n_sp = n_off_true = 0
    offboard_lost_after_go = False
    t0 = time.monotonic()
    frame = None
    if detector is not None:
        import numpy as _np
        frame = _np.zeros((960, 1280, 3), dtype=_np.uint8)
    if supervisor is not None:
        supervisor.watch()
    t_prev_tick: Optional[float] = None
    while True:
        t_abs = time.monotonic()
        t = t_abs - t0
        if t_prev_tick is not None:
            out["max_tick_gap_s"] = max(out["max_tick_gap_s"], t_abs - t_prev_tick)
        t_prev_tick = t_abs
        if t > outer_bound_s:
            out["bound_reached"] = True
            if bench:
                print(f"[bench] run bound {outer_bound_s:.0f}s reached in state "
                      f"{sm.state} -- ending the rehearsal")
            else:
                print(f"[mavsdk] outer smoke bound {outer_bound_s:.0f}s "
                      f"reached in state {sm.state} -- forcing shutdown")
                out["rc_exit"] = 1
            break
        if bench:
            # THE BENCH INTERLOCKS, every tick. Props are off and no flight
            # battery is connected, or this stops.
            if state["armed"] is not False:
                out["abort"] = (f"vehicle reports armed={state['armed']} during "
                                f"a props-off rehearsal")
            elif (bench_max_batt_v is not None and state["batt_v"] is not None
                  and state["batt_v"] > bench_max_batt_v):
                out["abort"] = (f"battery reads {state['batt_v']:.1f} V (> "
                                f"{bench_max_batt_v:.1f} V): a flight battery "
                                f"is connected")
            if out["abort"]:
                print(f"[bench] ABORT: {out['abort']} -- stopping the setpoint "
                      f"stream NOW")
                out["rc_exit"] = 1
                break
        # ONE clock contract, declared by the trigger (review2 BLOCKER):
        # RcChannelTrigger.ingest() stamps absolute time.monotonic(), so
        # its poll() must get the same base -- polling it with mission
        # time made the RC age negative and pinned link_ok True forever.
        trig = trigger.poll(trigger_poll_time(trigger, t, time.monotonic()))
        # --trigger cue: the belief latched from the live link, on the GO tick
        # only (None on every other tick; see VehicleObs.cue_belief).
        cue_belief = (trigger.take_belief() if isinstance(trigger, CueTrigger)
                      else None)
        det_new = det_range = det_box = det_pose = det_age = None
        if detector is not None and sm.t_dash_start is not None \
                and t >= sm.t_dash_start + acquire_after_s:
            # Detection runs EVERY tick (2026-09-23, tick-trace S3 fix:
            # the old `n_tick % 2 == 0` gate capped a live detector --
            # the Gazebo cross-check -- at ~9.6 Hz against a 30 fps
            # camera, 5.2 consumed det/s vs isim's 25.5; the camera was
            # never the limit). CPU caveat: the decode now runs in the
            # control loop every tick -- the tick trace measured detect
            # ticks only +4 ms (0.056 vs 0.052 s), but the re-fly must
            # confirm tick dt stays ~0.05 s with the doubled decode
            # rate. The "no new result -> HOLD" streak branch this
            # gate once exercised in the synthetic smoke is still
            # covered by the offline self-test drive (det_new every
            # other tick there) and by real decode misses/dropouts.
            try:
                d = detector.detect(frame, t)
                if not _det_is_finite(d):
                    raise ValueError("non-finite box or range from the detector")
            except Exception as e:  # noqa: BLE001 -- a decode fault is a MISS
                # One bad frame (a cv2 / AprilTag / PnP error, a NaN box) used
                # to escape this loop, land the aircraft where it was and lose
                # the tick log (review 2026-11-26). It is a missed detection.
                out["n_det_faults"] += 1
                if out["n_det_faults"] in (1, 10, 100, 1000):
                    print(f"[detector] FAULT #{out['n_det_faults']}: "
                          f"{type(e).__name__}: {e} -- counted as a missed "
                          f"detection, the flight goes on")
                d = _DET_MISS
            # A frame-fed detector (frame_feed.FeedDetector, --source)
            # returns the frame it actually decoded + its capture time,
            # or frame=None on a poll with no new frame (nothing to
            # record). Detectors without these attributes keep the
            # placeholder path unchanged.
            _rec_frame = getattr(d, "frame", frame)
            if frame_recorder is not None and _rec_frame is not None:
                # AFTER the detector consumed the frame, never before
                # -- the seeker's latency budget comes first. offer()
                # is O(1)/non-blocking (flight/deploy/frame_recorder.py).
                frame_recorder.offer(_rec_frame,
                                     getattr(d, "t_capture", t))
            det_new, det_range, det_box = (getattr(d, "is_new", True),
                                           d.range_m, d.box_xywh)
            # Detector-PnP slant range, if this detector publishes one
            # (the Gazebo cross-check driver's tag_range_m; None from
            # the synthetic smoke detector) -- see VehicleObs.
            det_pose = getattr(d, "tag_range_m", None)
            # STAMPED measurement age (--pursuit-stamped-age):
            # extracted to _stamped_det_age so the clock-base
            # convention is pinned by a unit test (review R2-F2 --
            # the F1 bug lived exactly in this block with no test
            # between the driver and the terminal).
            det_age = None
            if (getattr(args, "pursuit_stamped_age", False)
                    and det_range is not None):
                det_age = _stamped_det_age(d, t, time.monotonic())
                if det_age is not None and os.environ.get("DEBUG_DET_AGE"):
                    print(f"[det-age] stamped age = "
                          f"{det_age*1000:.0f} ms")
        # FAILSAFE 7 arm 2 input: how old the flight-mode sample is, on
        # the SAME clock base it was stamped with (absolute monotonic --
        # mission time here would reproduce the RC clock-base defect, and
        # offboard_status() treats a negative age as a hard fault).
        _now = time.monotonic()
        _t_mode = state["t_mode"]
        _offb_age = (None if _t_mode is None else _now - _t_mode)
        # FAILSAFE 9 input: the age of the OLDEST own-state stream, same clock.
        _own_age = own_state_age({k: state[k] for k in _OWN_STATE_STAMPS}, _now)
        obs = VehicleObs(t=t, armed=state["armed"],
                         offboard_active=state["offboard"],
                         offboard_sample_age_s=_offb_age,
                         mode=state["mode"],
                         alt_m=(cfg.standby_alt_m if (bench or handover is not None)
                                else state["alt"]),
                         yaw_deg=state["psi"],
                         quat=state["quat"],
                         ground_speed_ms=state["gs"],
                         vel_ned=state.get("vel_ned"), trigger=trig,
                         det_new=bool(det_new), det_range_m=det_range,
                         det_box_xywh=det_box, det_range_pose_m=det_pose,
                         det_age_s=det_age, own_state_age_s=_own_age,
                         battery_frac=fresh_battery_frac(state, _now),
                         camera_age_s=_camera_age(detector),
                         cue_belief=cue_belief)
        if isinstance(trigger, (GateReadyTrigger, CueTrigger)):
            trigger.observe_gate(sm.arm_gate(obs)[0])
        try:
            dec = sm.step(obs)
            step_faults_running = 0
        except Exception as e:  # noqa: BLE001 -- the stream must not die on a bug
            # The state machine raised. Ending the loop here used to mean the
            # setpoint stream stopped mid-flight. Hold the last command for a
            # few ticks; if it keeps raising, stop and hand over to teardown
            # (which lands or returns per the plan).
            out["n_step_faults"] += 1
            step_faults_running += 1
            print(f"[mavsdk] FAULT step_raised #{out['n_step_faults']} at "
                  f"t={t:.2f}s in {sm.state}: {type(e).__name__}: {e}")
            if last_sp is None or step_faults_running > _STEP_FAULTS_TO_ABORT:
                out["abort"] = (f"the state machine raised "
                                f"{step_faults_running} ticks running "
                                f"({type(e).__name__}: {e})")
                out["rc_exit"] = 1
                break
            if not args.dry_run:
                try:
                    await drone.offboard.set_velocity_ned(last_sp)
                except Exception:  # noqa: BLE001 -- counted on the next tick
                    pass
            if supervisor is not None:
                supervisor.beat()
            await asyncio.sleep(1.0 / args.fps)
            continue
        if handover is not None:
            # PILOT HAND-OVER: what leaves this loop is the LIMITED command, and
            # the limited command is what the tick log records.
            dec.setpoint = handover.limit(dec.setpoint, dec.state, state["psi"])
            out["handover_max_h_ms"] = max(
                out.get("handover_max_h_ms", 0.0),
                math.hypot(dec.setpoint.v_north, dec.setpoint.v_east))
        n_off_true += 1 if state["offboard"] is True else 0
        if state["offboard"] is not True and sm.dash_heading.is_set:
            if not offboard_lost_after_go:
                offboard_lost_after_go = True
                print(f"[mavsdk] FAULT offboard_lost_after_go: PX4 flight "
                      f"mode is {state['mode']} at t={t:.2f}s in state "
                      f"{dec.state} -- the setpoints below this line were "
                      f"NOT flown by the vehicle. POLICY (FAILSAFE 7, "
                      f"builder decision #22): the STATE MACHINE aborts to "
                      f"SAFE once this persists {cfg.offboard_lost_s:.2f}s "
                      f"(or the mode sample goes stale past "
                      f"{cfg.offboard_stale_s}s), and SAFE is absorbing, so "
                      f"restoring OFFBOARD hands the vehicle a zero-velocity "
                      f"hold + land request rather than a live dash. This "
                      f"is NOT the link failsafe: post-GO LINK loss is "
                      f"still expected and ignored (constraint no-datalink).")
        _row = _csv_row(obs, dec)
        csv_rows.append(_row)
        if tick_log is not None:
            tick_log.offer(_row)
        sp = dec.setpoint
        if args.dry_run:
            print(f"[dry-run] {dec.state} SETPOINT {sp.as_tuple()}")
        else:
            last_sp = VelocityNedYaw(sp.v_north, sp.v_east, sp.v_down,
                                     sp.yaw_deg)
            try:
                await drone.offboard.set_velocity_ned(last_sp)
                send_faults_running = 0
            except Exception as e:  # noqa: BLE001 -- one failed call is not the flight
                # The MAVSDK server re-sends the previous setpoint on its own
                # timer, so one refused call costs one tick of freshness. A
                # run of them is a dead control path: stop and tear down.
                send_faults_running += 1
                out["n_send_faults"] = out.get("n_send_faults", 0) + 1
                print(f"[mavsdk] FAULT setpoint_send_failed "
                      f"#{send_faults_running} running at t={t:.2f}s: "
                      f"{type(e).__name__}: {e}")
                if send_faults_running > _SEND_FAULTS_TO_ABORT:
                    out["abort"] = (f"{send_faults_running} setpoint sends "
                                    f"failed running ({type(e).__name__}: {e})")
                    out["rc_exit"] = 1
                    break
        n_sp += 1
        n_tick += 1
        if supervisor is not None:
            supervisor.beat()
            if not supervisor.alive():
                # The watchdog is gone (or has already ended the stream).
                # Flying on unwatched is the one thing it exists to prevent.
                out["abort"] = (f"loop supervisor is gone (exit "
                                f"{supervisor.exit_code()})")
                print(f"[watchdog] FAULT: {out['abort']} -- the setpoint "
                      f"stream is no longer watched; stopping")
                out["rc_exit"] = 1
                break
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
            if handover is not None and not args.dry_run:
                # The program is finished but the aircraft may still be in
                # OFFBOARD with the pilot's switch on. Hold still (zero
                # velocity, the yaw it has now) until the pilot takes it back
                # or the run bound is reached; never end the stream under an
                # aircraft that is still flying it.
                hold = VelocityNedYaw(0.0, 0.0, 0.0,
                                      float(state["psi"] or dec.setpoint.yaw_deg))
                print("[handover] finished: holding still until the GO switch "
                      "is lowered (you then have it in the mode-switch mode)")
                while state["offboard"] is True \
                        and time.monotonic() - t0 <= outer_bound_s:
                    try:
                        await drone.offboard.set_velocity_ned(hold)
                    except Exception:  # noqa: BLE001 -- keep trying to hold
                        pass
                    n_sp += 1
                    out["n_hold"] = out.get("n_hold", 0) + 1
                    if supervisor is not None:
                        supervisor.beat()
                    await asyncio.sleep(1.0 / args.fps)
            break
        if handover is not None and dec.state == State.SAFE \
                and state["offboard"] is not True:
            # Finished, and the pilot has the aircraft (or never gave it up):
            # nothing is flying these commands, so stop sending them.
            print(f"[handover] finished in SAFE (reason={dec.safe_reason}) and "
                  f"the vehicle is in {state['mode']}, not OFFBOARD: ending "
                  f"the command stream")
            break
        await asyncio.sleep(1.0 / args.fps)
    out.update(n_tick=n_tick, n_sp=n_sp, n_off_true=n_off_true,
               offboard_lost_after_go=offboard_lost_after_go)
    return out


class TickLog:
    """The tick log, written WHILE the flight runs.

    Why: the rows used to live in memory and reach the card only after landing,
    so a crash, a kill or a power cut lost the one record of what the program
    commanded (review 2026-11-26). The control loop hands each row to a queue
    (constant time, never blocks); a thread writes them and flushes about once
    a second. `path` is the final log's name plus `.live`; the complete log is
    still written at the end, and the `.live` file is then removed. If the
    program dies, the `.live` file IS the log, short by at most a second."""

    def __init__(self, final_path: Optional[str], flush_s: float = 1.0):
        import queue
        import threading
        self.path = None if not final_path else final_path + ".live"
        self._q = queue.SimpleQueue()
        self._flush_s = float(flush_s)
        self._thread = None
        self.error: Optional[str] = None
        self.n_written = 0
        if self.path is None:
            return
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            self._fh = open(self.path, "w")
            self._fh.write(_CSV_HEADER)
            self._fh.flush()
        except OSError as e:
            self.error = f"{type(e).__name__}: {e}"
            self.path = None
            return
        self._thread = threading.Thread(target=self._run, name="ticklog",
                                        daemon=True)
        self._thread.start()

    def offer(self, row: str) -> None:
        if self._thread is not None:
            self._q.put(row)

    def _run(self) -> None:
        import queue
        last_flush = time.monotonic()
        while True:
            try:
                row = self._q.get(timeout=self._flush_s)
            except queue.Empty:
                row = ""
            try:
                if row is None:
                    self._fh.flush()
                    self._fh.close()
                    return
                if row:
                    self._fh.write(row)
                    self.n_written += 1
                if time.monotonic() - last_flush >= self._flush_s:
                    self._fh.flush()
                    last_flush = time.monotonic()
            except (OSError, ValueError) as e:
                self.error = f"{type(e).__name__}: {e}"
                return

    def close(self, remove: bool = False, wait_s: float = 2.0) -> None:
        """Flush and stop. `remove=True` once the complete log is on the card."""
        if self._thread is not None:
            self._q.put(None)
            self._thread.join(timeout=wait_s)
            self._thread = None
        if remove and self.path is not None:
            try:
                os.remove(self.path)
            except OSError:
                pass


def _write_mission_logs(args, csv_rows, frame_recorder) -> None:
    if getattr(args, "log_csv", None):
        os.makedirs(os.path.dirname(args.log_csv), exist_ok=True)
        with open(args.log_csv, "w") as fh:
            fh.write(_CSV_HEADER)
            fh.writelines(csv_rows)
        print(f"[log] {len(csv_rows)} ticks -> {args.log_csv}")
    # MISSION END: drain + finalize the onboard footage recorder (--record-
    # frames), unconditionally -- this runs on every exit path (clean
    # shutdown, the outer bound, an exception), so a recorder attached to a
    # mission that fails partway still gets its meta.json written and its
    # queued frames drained within the deadline.
    if frame_recorder is not None:
        fr_meta = frame_recorder.close()
        print(f"[frame_recorder] {fr_meta['n_written']} written, "
              f"{fr_meta['n_dropped']} dropped "
              f"(rate={fr_meta['drop_rate']:.1%}), target_fps="
              f"{fr_meta['config']['target_fps']} -> {fr_meta['out_dir']}")


def _board_summary(args) -> List[str]:
    if board_failsafes_mode(args) != "suspend":
        return ["[fly] board failsafes: KEPT. The Pixhawk's low-battery action "
                "and GPS quality checks stay live and can end the intercept."]
    from flight.deploy import board_overrides
    return board_overrides.summary_lines()


def prearm_summary(args, cfg: MissionConfig, caps: dict) -> List[str]:
    """What `--fly` is about to do, in plain lines, printed before the arm
    permit is asked for. Pure."""
    belief = ("from the GPS cue file " + str(args.cue_json)
              if getattr(args, "cue_json", None)
              else "TYPED by the operator (--manual-belief)")
    r0, v0 = caps["belief_r0_ne"], caps["belief_v_ne"]
    cue = getattr(args, "trigger", None) == "cue"
    if cue:
        belief_line = (
            "[fly] belief: NONE YET. The cue is the GO command: the target's GPS "
            f"arrives on UDP port {args.cue_udp_port}, and at GO the freshest "
            "fit is advanced to that instant (each record's link delay "
            + ("measured against the Pixhawk's GPS clock" if
               args.cue_utc_source == "vehicle" else
               "measured against this computer's clock" if
               args.cue_utc_source == "system" else "NOT measured")
            + ") and latched once.")
        go_line = (
            f"[fly] it then waits up to {cfg.standby_max_s:.0f} s. GO fires BY "
            f"ITSELF on the first usable cue while the enable switch (RC channel "
            f"{args.rc_channel}) is HIGH; keep it LOW to hold fire. ")
    else:
        belief_line = (
            f"[fly] belief: target {math.hypot(r0[0], r0[1]):.0f} m away (north "
            f"{r0[0]:+.1f}, east {r0[1]:+.1f}) moving {math.hypot(v0[0], v0[1]):.1f} "
            f"m/s -- {belief}. It is used AS IF TRUE AT THE GO EDGE: nothing "
            f"advances it for the time between when it was taken and GO.")
        go_line = (
            f"[fly] it then waits up to {cfg.standby_max_s:.0f} s for GO on RC "
            f"channel {args.rc_channel} (LOW then HIGH). ")
    if math.isfinite(cfg.engage_max_s):
        limit_lines = [go_line + f"GO starts the engagement: up to "
                       f"{caps['v_cap_ms']:.0f} m/s commanded, for at most "
                       f"{cfg.engage_max_s:.0f} s."]
    else:
        limit_lines = [go_line + f"GO starts the engagement: up to "
                       f"{caps['v_cap_ms']:.0f} m/s commanded, with NO TIME LIMIT "
                       f"(--engage-max-s sets one). What still ends an "
                       f"engagement that never finds the target:"]
        limit_lines += [f"[fly]    - {x}" for x in what_ends_an_unbounded_engagement()]
    if math.isfinite(cfg.mission_max_s):
        limit_lines.append(f"[fly] whole-run backstop: {cfg.mission_max_s:.0f} s "
                           f"from program start, any state.")
    if caps.get("approach_dash"):
        limit_lines.append(
            f"[fly] approach dash: it brakes into the believed rendezvous at "
            f"{caps['brake_accel_ms2']:.1f} m/s^2"
            + (" with vertical arrival-sync." if caps.get("brake_vert_sync")
               else " (vertical arrival-sync OFF)."))
    if caps.get("arrival_search"):
        limit_lines.append(
            f"[fly] ARRIVAL SEARCH is ON ('{caps['arrival_search']}'): if the "
            f"target is not where the cue says, the aircraft sweeps along the "
            f"believed track, up to {caps['search_sweep_ms']:.0f} m/s faster and "
            f"{caps['search_drop_ms']:.0f} m/s slower than the believed target, "
            f"until the camera reads the tag. It may pass close to the target "
            f"without having seen it, and it keeps searching until one of the "
            f"things above stops it. Declared cue speed error: "
            f"{caps['search_sigma_speed_frac']*100:.0f} % (one sigma).")
    elif caps.get("terminal") == "pursuit":
        limit_lines.append("[fly] arrival search is OFF: the aircraft parks behind "
                           "where it believes the target is and waits to see it.")
    return [
        "[fly] ===== PRE-ARM SUMMARY: read this before raising the permit switch =====",
        f"[fly] link: {args.mavsdk_url}   terminal: {caps['terminal']}   "
        f"camera: {args.source} ({args.detector})",
        belief_line,
        f"[fly] on the arm permit this program will: set MIS_TAKEOFF_ALT to "
        f"{cfg.standby_alt_m:.1f} m (a parameter WRITE, by MAVSDK's "
        f"set_takeoff_altitude"
        + (", plus the three board settings listed below" if
           board_failsafes_mode(args) == "suspend" else " -- the only one")
        + "), ARM, TAKE OFF to "
        f"{cfg.standby_alt_m:.1f} m, enter OFFBOARD and hold, yawed to "
        f"{cfg.aim_yaw_deg:.1f} deg.",
    ] + limit_lines + [
        f"[fly] it ends by: {cfg.miss_safe_behavior} after a miss, "
        f"{cfg.safe_behavior} after a fault. If you take the aircraft with the "
        f"mode switch, this program sends nothing further.",
        f"[fly] watchdog: no control tick for {args.loop_watchdog_s:.2f} s ends "
        f"the setpoint stream; own-state older than {cfg.own_state_stale_s} s "
        f"goes SAFE.",
        "[fly] THE KILL SWITCH stays yours for the whole flight.",
    ] + ([f"[fly] links: primary {args.mavsdk_url}, backup "
          f"{args.mavsdk_backup_url}; a failed primary no longer ends the "
          f"flight."]
         if getattr(args, "mavsdk_backup_url", None) else
         [f"[fly] links: ONE ({args.mavsdk_url}), no backup: if this cable "
          f"fails the intercept ends (--mavsdk-backup-url adds the second)."]
    ) + (_board_summary(args)) + ([
        "[fly] WARNING: output is going to a terminal. If that terminal is an "
        "SSH session and it stalls, print() blocks the control loop and the "
        "watchdog ends the flight (PX4 takes over). Run detached (tmux, or "
        "systemd-run) with output to a file.",
    ] if sys.stdout.isatty() else [])


async def _wait_arm_permit(args, trigger, permit_trigger, state: dict) -> bool:  # pragma: no cover -- driven by the fake in tests
    """Block until the operator gives the RC arm permit (see `ArmPermit`), or
    the timeout passes. True only on a granted permit with the vehicle still
    disarmed. Sends nothing."""
    import asyncio
    permit = ArmPermit(args.rc_go_us, args.rc_low_us)
    timeout_s = float(args.arm_permit_timeout_s)
    _sw = "ENABLE" if isinstance(trigger, CueTrigger) else "GO"
    print(f"[arm-permit] WAITING up to {timeout_s:.0f} s. With the {_sw} switch "
          f"(ch {args.rc_channel}) LOW, flip the permit switch (ch "
          f"{args.arm_permit_channel}) LOW then HIGH to let this program arm "
          f"and take off. Ctrl-C, or doing nothing, leaves the aircraft disarmed.")
    deadline = time.monotonic() + timeout_s
    last = None
    said_refused = False
    # Under --trigger cue the "GO switch" of the permit rule is the cue's
    # ENABLE switch: it must be LOW (no automatic GO possible) while you arm.
    go_switch = trigger.enable if isinstance(trigger, CueTrigger) else trigger
    while time.monotonic() < deadline and not permit.granted:
        now = time.monotonic()
        permit.update(go_switch.poll(now), permit_trigger.poll(now))
        if permit.refused_go_high and not said_refused:
            said_refused = True
            print("[arm-permit] REFUSED once: the permit switch rose while the "
                  "GO switch was not LOW. Lower both, then raise the permit "
                  "switch alone.")
        st = permit.status()
        if st != last:
            last = st
            print(f"[arm-permit] {st}")
        if not permit.granted:
            await asyncio.sleep(_ARM_PERMIT_POLL_S)
    if not permit.granted:
        print(f"[arm-permit] NOT GRANTED within {timeout_s:.0f} s -- NOT arming")
        return False
    if state["armed"] is not False:
        print(f"[arm-permit] the vehicle reports armed={state['armed']} before "
              f"this program armed it -- NOT taking it over")
        return False
    return True


def _end_setpoint_stream(drone) -> bool:
    """Stop the setpoint stream WITHOUT commanding a mode: end the MAVSDK server
    that is re-sending it. (`offboard.stop()` is not that -- it commands HOLD.)"""
    procs = getattr(drone, "_server_processes", None)
    if procs is None:
        one = getattr(drone, "_server_process", None)
        procs = [] if one is None else [one]
    ended = False
    for proc in procs:          # EVERY server: one survivor keeps re-sending
        if proc is not None and proc.poll() is None:
            proc.kill()
            ended = True
    return ended


async def _teardown_flight(drone, cfg: MissionConfig, sm: RealFlightSM,
                           state: dict, offboard_ok: bool,
                           armed_by_program: bool, LandedState) -> None:  # pragma: no cover -- driven by the fake in tests
    """Hand the aircraft back at the end of a run. With `_wait_arm_permit`'s
    caller this is the ONLY code in the module that touches the action plugin.

    What it may send is decided by `teardown_plan` from the flight mode the
    vehicle is in RIGHT NOW: land (or return) only while the aircraft is still
    in a mode this program commanded; NOTHING when the pilot has taken it."""
    import asyncio
    if not armed_by_program:
        print("[mavsdk] teardown: this program never armed the vehicle -- "
              "sending nothing")
        return
    _t_mode = state["t_mode"]
    _age = None if _t_mode is None else time.monotonic() - _t_mode
    stop_offboard, action, why = teardown_plan(
        state["mode"], _age, cfg.offboard_stale_s, sm.effective_safe_behavior())
    print(f"[mavsdk] teardown: {why}")
    if action is None:
        ended = _end_setpoint_stream(drone)
        print(f"[mavsdk] no offboard stop, no land, no return sent"
              f"{'; setpoint stream ended' if ended else ''}")
        return
    if offboard_ok and stop_offboard:
        try:
            await drone.offboard.stop()
            print("[mavsdk] offboard stopped")
        except Exception as e:  # noqa: BLE001 -- best-effort teardown
            print(f"[mavsdk] offboard.stop skipped: {e}")
    try:
        # SAFE behavior (builder ruling 2026-09-22): the STATE MACHINE
        # never knows what RTL is -- it stays exactly as pure for
        # safe_behavior='rtl' as for the 'land' default (same
        # land_requested/terminated timing, see RealFlightSM._decide).
        # Only the LIVE DRIVER, here, picks which PX4 ACTION to issue.
        # RETURN-TO-LAUNCH is PX4's OWN GPS-based safety behaviour (fly
        # home at a safe altitude, then auto-land) -- a SAFETY fallback,
        # NOT a guidance decision, and must never be read as one. This is
        # the teardown path for every program-owned exit (also reached on
        # the outer forced-shutdown bound), so 'hover' still comes down
        # here rather than being stranded airborne forever.
        if action == "rtl":
            await drone.action.return_to_launch()
            print(f"[mavsdk] RETURN-TO-LAUNCH commanded (effective safe "
                  f"behavior 'rtl', reason={sm.safe_reason}); waiting up "
                  f"to {_LAND_TIMEOUT_S}s for touchdown + disarm (PX4 "
                  f"flies home first -- may not complete within this "
                  f"bound; that is a teardown-wait timeout, not a "
                  f"failsafe) ...")
        else:
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


async def run_mavsdk_mission(args, cfg: MissionConfig, sm: RealFlightSM,
                             trigger, detector=None,
                             smoke: bool = False,
                             frame_recorder=None, *,
                             fly: bool = False,
                             permit_trigger=None):  # pragma: no cover -- needs a vehicle; driven by the fake in tests
    """Live driver: connect over MAVLink, stream own-state EKF + RC_CHANNELS, and
    stream the state machine's NED velocity+yaw setpoints through PX4 OFFBOARD.

    Unlike seeker_loop, this driver OWNS the takeoff in BOTH modes -- the standby
    hover is the whole point, so arm/takeoff/land are part of the real sequence,
    not a smoke-only synthesis. `smoke=True` changes only two things:
      * the trigger is synthetic (GateReadyTrigger / a timed edge) instead of a
        human on an ELRS switch, and the detector is synthetic (no camera), and
      * the run asserts a PASS/FAIL contract at the end (states visited, setpoint
        count, heading latched) so the SITL check can exit 0/1.

    `fly=True` is the REAL FLIGHT (`--fly`, 2026-11-18). It is the only way this
    function will arm over a serial link, and before it arms it (1) reads the
    board's limits and refuses if the guidance commands more than the board
    flies or the believed rendezvous is outside the geofence, (2) prints a
    plain pre-arm summary, and (3) waits for the operator's RC arm permit.

    `frame_recorder` (flight.deploy.frame_recorder.FrameRecorder, optional):
    dependency-injected via `build_frame_recorder`/`--record-frames`; None by
    default and this function never imports the recorder module itself. When
    attached, every raw frame is `offer()`ed right after `detector.detect()`
    consumes it (never before), and `close()` runs at mission end with one
    summary line printed.

    TODO-BUILDER before this ever flies: the RC kill switch verified live
    (RC_MAP_KILL_SW), battery failsafe, and a props-off bench pass of the whole
    sequence (`--bench-props-off`, launch_mechanism_plan L3)."""
    import asyncio

    # Defence in depth behind main()'s own refusal: a serial link is a real
    # aircraft, and only `--fly` (with its permit and its checks) may arm one.
    if is_serial_url(args.mavsdk_url) and not fly:
        print(f"[mavsdk] REFUSED: {args.mavsdk_url} is a serial link to a real "
              f"flight controller, and this path would arm and take off with "
              f"no human step. Use --fly (real flight) or --bench-props-off "
              f"(rehearsal, cannot arm).")
        return 1

    from mavsdk.telemetry import LandedState
    from flight.deploy import board_overrides

    drone = await _open_vehicle(args)
    if drone is None:
        return 1
    suspend = fly and board_failsafes_mode(args) == "suspend"
    board_state_path = (getattr(args, "board_state_path", None)
                        or board_overrides.DEFAULT_STATE_PATH)
    board_changed = False

    # OWN-STATE EKF + own status. The ONLY non-camera, non-constant inputs.
    state = _new_link_state()
    rc_sinks = _rc_readers(trigger, permit_trigger)
    cue_height = _wire_cue_trigger(trigger, state)
    tasks = _start_telemetry(drone, state, rc_sinks=rc_sinks, want_battery=fly,
                             utc_sink=getattr(trigger, "vehicle_utc", None),
                             want_gps=suspend, want_gps_height=cue_height)
    tick_log = TickLog(getattr(args, "log_csv", None))
    if tick_log.error:
        print(f"[log] WARNING: the live tick log could not be opened "
              f"({tick_log.error}); the log is written at the end only")

    supervisor, sup_ok = _start_supervisor(drone, args, required=fly)

    rc_exit = 0
    offboard_ok = False
    armed_by_program = False
    loop_out = {"n_tick": 0, "n_sp": 0, "n_off_true": 0, "csv_rows": [],
                "offboard_lost_after_go": False, "abort": None, "rc_exit": 0}
    try:
        if not sup_ok:
            print("[fly] REFUSED: the setpoint-stream watchdog could not be "
                  "started -- NOT arming")
            return 1
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

        if fly:
            limits, unread = await read_platform_limits(drone)
            caps = commanded_caps(args, cfg, sm.guidance)
            for line in format_platform_table(limits, caps):
                print(line)
            hard, refusals, notes = check_platform_limits(limits, unread, caps, cfg)
            for n in notes:
                print(f"[limits] note: {n}")
            for h in hard:
                print(f"[limits] REFUSED (cannot be overridden): {h}")
            override = bool(getattr(args, "override_platform_limits", False))
            for r in refusals:
                print(f"[limits] {'OVERRIDDEN by --override-platform-limits' if override else 'REFUSED'}: {r}")
            if hard or (refusals and not override):
                print("[fly] NOT ARMING: fix the mismatch above (change the "
                      "guidance cap, the belief or the board), or pass "
                      "--override-platform-limits to fly it knowingly.")
                return 1
            if isinstance(trigger, CueTrigger):
                trigger.go_check = make_cue_go_check(
                    limits, caps, getattr(args, "cue_max_range_m", None))
            if suspend:
                # The program takes over the two checks it is about to switch
                # off on the board: the battery level and the GPS quality.
                samples = await _sample_gps(state, _GPS_PREARM_WINDOW_S)
                stops = gps_prearm_refusals(samples, limits)
                b = battery_prearm_refusal(
                    fresh_battery_frac(state, time.monotonic()), limits)
                if b:
                    stops.append(b)
                for st in stops:
                    print(f"[fly] REFUSED: {st}")
                if stops:
                    print("[fly] NOT ARMING. (--board-failsafes keep leaves "
                          "these checks to the Pixhawk instead.)")
                    return 1
                cfg.battery_low_frac = float(limits["BAT_CRIT_THR"])
                print(f"[fly] GPS and battery pass the board's own thresholds "
                      f"({samples[-1]['sats']} satellites, speed accuracy "
                      f"{samples[-1]['sacc']:.2f} m/s, battery "
                      f"{state['batt_rem']:.0%}). Battery rule from here: "
                      f"below {cfg.battery_low_frac:.0%} lands before GO and "
                      f"after the intercept, ignored in between.")
            for line in prearm_summary(args, cfg, caps):
                print(line)
            if permit_trigger is None or not await _wait_arm_permit(
                    args, trigger, permit_trigger, state):
                return 1
            if suspend:
                w, v = _board_links(drone)
                board_changed = True     # from here a restore is owed
                if not await board_overrides.apply(
                        w, state["armed"], verify_drone=v,
                        state_path=board_state_path):
                    print("[fly] NOT ARMING: the board settings for this "
                          "flight could not be confirmed.")
                    return 1

        alt = cfg.standby_alt_m
        print(f"[mavsdk] set takeoff altitude {alt:.1f} m (standby/loft); arm; takeoff ...")
        await drone.action.set_takeoff_altitude(alt)
        armed_by_program = True
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

        offboard_ok = await _enter_offboard(drone, cfg, state)
        if not offboard_ok:
            rc_exit = 1

        if offboard_ok:
            loop_out = await _control_loop(
                drone, args, cfg, sm, trigger, detector, state, frame_recorder,
                supervisor,
                # A real flight is bounded by the state machine's own mission
                # backstop; the smoke bound (90 s, shorter than mission_max_s)
                # and the 2 s synthetic-detection delay are SITL conveniences.
                outer_bound_s=(cfg.mission_max_s + cfg.safe_hold_s + 15.0 if fly
                               else args.smoke_duration),
                acquire_after_s=(0.0 if fly else args.smoke_acquire_after_s),
                out=loop_out, tick_log=tick_log)
            rc_exit = rc_exit or loop_out["rc_exit"]
    finally:
        rc_exit = rc_exit or loop_out.get("rc_exit", 0)
        # The loop is over: tell the watchdog the silence that follows (the
        # landing wait) is deliberate.
        if supervisor is not None:
            supervisor.unwatch()
        try:
            await _teardown_flight(drone, cfg, sm, state, offboard_ok,
                                   armed_by_program, LandedState)
        finally:
            if supervisor is not None:
                supervisor.close()
        if board_changed:
            try:
                w, v = _board_links(drone)
                await board_overrides.restore(w, state["armed"], verify_drone=v,
                                              state_path=board_state_path)
            except Exception as e:  # noqa: BLE001 -- the saved file keeps it owed
                print(f"[board] restore did not run ({type(e).__name__}: {e}); "
                      f"the originals stay saved in {board_state_path}")
        for line in getattr(drone, "report_lines", lambda: [])():
            print(line)
        for tk in tasks:
            tk.cancel()
        try:
            _write_mission_logs(args, loop_out["csv_rows"], frame_recorder)
            tick_log.close(remove=True)
        except Exception as e:  # noqa: BLE001 -- keep the live copy
            tick_log.close(remove=False)
            print(f"[log] the final log could not be written "
                  f"({type(e).__name__}: {e}); the live copy is {tick_log.path}")

    n_tick, n_sp, n_off_true = (loop_out["n_tick"], loop_out["n_sp"],
                                loop_out["n_off_true"])
    csv_rows = loop_out["csv_rows"]
    offboard_lost_after_go = loop_out["offboard_lost_after_go"]

    # ---- PASS/FAIL for the SITL gate ------------------------------------------
    visited = sm.visited
    print(f"[mission] states visited: {' -> '.join(visited)}")
    print("[mission] transitions:")
    for tr in sm.transitions:
        print(f"    {tr.t:7.2f}s  {tr.frm} -> {tr.to}  ({tr.reason})")
    print(f"[mission] oldest own-state sample seen: {sm.max_own_state_age_s:.2f}s "
          f"(FAILSAFE 9 bound = {cfg.own_state_stale_s}s)")
    if smoke:
        # ADR-0105/0118: in pursuit_mode the GO edge enters ENGAGE directly --
        # CODED_DASH is structurally unreachable, so requiring it would fail
        # the flying default forever (it did, 2026-10-24, the first run of
        # this gate after the ADR-0118 default flip). The dash remains
        # required for the stock/tag paths, where skipping it IS a defect.
        need = ((State.ENGAGE, State.SAFE) if sm.cfg.pursuit_mode
                else (State.CODED_DASH, State.ENGAGE, State.SAFE))
        for st in need:
            if st not in visited:
                print(f"[sitl-smoke] FAIL: never reached {st}")
                rc_exit = 1
        if not offboard_ok:
            print("[sitl-smoke] FAIL: offboard.start() never succeeded")
            rc_exit = 1
        if n_sp < 20:
            print(f"[sitl-smoke] FAIL: only {n_sp} setpoints streamed")
            rc_exit = 1
        if not sm.dash_heading.is_set:
            print("[sitl-smoke] FAIL: dash heading was never latched")
            rc_exit = 1
        # A DEAD SEEKER MUST FAIL THIS GATE (review2): "states visited" +
        # "n_sp > 0" both pass with a terminal that never steers -- which is
        # exactly how a frozen/zero-command terminal survived elsewhere. Assert
        # the terminal ACTUATED: some ENGAGE tick was `guided`, and the commanded
        # horizontal velocity actually moved.
        eng = [r.split(",") for r in csv_rows if f",{State.ENGAGE}," in r]
        i_src, i_vn, i_ve = (_CSV_FIELDS.index("sp_source"),
                             _CSV_FIELDS.index("v_north"),
                             _CSV_FIELDS.index("v_east"))
        n_guided = sum(1 for r in eng if r[i_src] == "guided")
        n_distinct = len({(r[i_vn], r[i_ve]) for r in eng})
        print(f"[sitl-smoke] terminal: {len(eng)} ENGAGE ticks, {n_guided} "
              f"camera-GUIDED, {n_distinct} distinct horizontal commands")
        if eng and n_guided == 0:
            print("[sitl-smoke] FAIL: ZERO ENGAGE ticks were camera-guided -- the "
                  "terminal never steered (states visited is not evidence)")
            rc_exit = 1
        if eng and n_distinct < 2:
            print("[sitl-smoke] FAIL: the commanded horizontal velocity took "
                  f"{n_distinct} distinct value(s) across ENGAGE -- a frozen or "
                  "zero terminal, not guidance")
            rc_exit = 1
        # OFFBOARD must have been held WHILE the terminal was actually steering --
        # NOT at the final tick. CORRECTED 2026-07-25 (review 2, SITL-observed): a
        # nominal mission ends in SAFE with offboard cleanly STOPPED before landing
        # (mode=HOLD), so "offboard active at the end" false-FAILs a perfect run. The
        # real fact — "were the guided setpoints flown in OFFBOARD?" — is checked here
        # over the ENGAGE ticks, and the mid-flight loss is caught by
        # offboard_lost_after_go below. (This is why offboard is telemetry-sourced now:
        # a latched constant could not have told the truth at either point.)
        i_mode = _CSV_FIELDS.index("mode")
        eng_guided = [r for r in eng if r[i_src] == "guided"]
        n_guided_offb = sum(1 for r in eng_guided if r[i_mode] == "OFFBOARD")
        if eng_guided and n_guided_offb < len(eng_guided):
            print(f"[sitl-smoke] FAIL: {len(eng_guided) - n_guided_offb} of "
                  f"{len(eng_guided)} camera-GUIDED ENGAGE ticks were NOT in OFFBOARD "
                  f"-- the commanded dash was not flown by the vehicle "
                  f"(offboard.start() succeeding is not the same fact)")
            rc_exit = 1
        if offboard_lost_after_go:
            print("[sitl-smoke] FAIL: PX4 left OFFBOARD after the GO edge -- the "
                  "logged dash was not flown by the vehicle")
            rc_exit = 1
        print(f"[sitl-smoke] offboard_active was True on {n_off_true}/{n_tick} "
              f"ticks (telemetry-sourced, not latched)")
        # THE MEASUREMENT that sizes FAILSAFE 7 arm 2 (cfg.offboard_stale_s is a
        # TODO-BUILDER guess until this number exists on the real Pi). Printed
        # every run so the bound is set from data, not from an assumed rate.
        _max_age = sm.max_offboard_sample_age_s
        print(f"[sitl-smoke] max flight_mode sample age observed: {_max_age:.2f}s "
              f"(FAILSAFE 7 arm 2 bound = {cfg.offboard_stale_s}s; set the bound "
              f"to ~5x the observed cadence, or None if the stream is "
              f"on-change-only)")
        if cfg.offboard_stale_s is not None and _max_age > 0.5 * cfg.offboard_stale_s:
            print(f"[sitl-smoke] WARNING: the observed mode-sample age is within "
                  f"2x of the FAILSAFE 7 staleness bound -- raise "
                  f"--offboard-stale-s or a healthy flight will abort")
        print(f"[sitl-smoke] {'PASS' if rc_exit == 0 else 'FAIL'} "
              f"({n_sp} setpoints, dash heading "
              f"{sm.dash_heading.value if sm.dash_heading.is_set else 'unset'})")
    return rc_exit


def bench_config(cfg: MissionConfig) -> MissionConfig:
    """The mission config for a PROPS-OFF rehearsal: the three arm-gate checks
    a board on a desk cannot meet are stubbed, and nothing else.

      * `require_armed=False` -- the vehicle must stay DISARMED the whole run;
      * the height gate is met by the driver feeding the standby height (see
        `_control_loop(bench=True)`);
      * `yaw_tol_deg=180` -- a disarmed vehicle cannot turn to the aim yaw.

    The OFFBOARD gate, the attitude-quaternion gate, the settle time, the GO
    edge rules, every failsafe and the whole terminal run as they will fly."""
    return replace(cfg, require_armed=False, yaw_tol_deg=180.0)


async def run_bench_props_off(args, cfg: MissionConfig, sm: RealFlightSM,
                              trigger, detector=None,
                              frame_recorder=None):  # pragma: no cover -- needs the board; driven by the fake in tests
    """PROPS-OFF REHEARSAL of the real program on the real board
    (`--bench-props-off`). The field protocol's L0 rung ("start the state
    machine, watch STANDBY, flip GO") could not be run before 2026-11-18,
    because the only live path always armed and took off.

    It connects, PROVES the vehicle is disarmed and that no flight battery is
    connected, reads the board's limits, enters OFFBOARD while disarmed, and
    runs the SAME control loop the flight runs: STANDBY -> GO (the real RC
    switch, or a timer) -> ENGAGE, with the real setpoint stream, the real
    own-state streams and the real watchdog. It ends by stopping OFFBOARD.

    IT CANNOT ARM, AND THAT IS STRUCTURAL: neither this coroutine nor anything
    it calls names the vehicle's `action` plugin (arm, takeoff, land, disarm
    all live there). flight/tests/test_real_flight_driver.py walks the syntax
    tree of this function's whole call closure to prove it. The physical
    interlock is still PROPS OFF."""
    import asyncio

    drone = await _open_vehicle(args)
    if drone is None:
        return 1
    state = _new_link_state()
    rc_sinks = _rc_readers(trigger)
    cue_height = _wire_cue_trigger(trigger, state)
    tasks = _start_telemetry(drone, state, rc_sinks=rc_sinks, want_battery=True,
                             utc_sink=getattr(trigger, "vehicle_utc", None),
                             want_gps_height=cue_height)
    supervisor, _sup_ok = _start_supervisor(drone, args, required=False)

    max_batt = float(args.bench_max_batt_v)
    rc_exit = 0
    offboard_ok = False
    unread: List[str] = []
    loop_out = {"n_tick": 0, "n_sp": 0, "n_off_true": 0, "csv_rows": [],
                "offboard_lost_after_go": False, "abort": None,
                "max_tick_gap_s": 0.0}
    try:
        # INTERLOCKS. Both need a REAL sample: "no reading" is not "disarmed"
        # and it is not "no battery".
        deadline = time.monotonic() + _BENCH_STREAM_WAIT_S
        while time.monotonic() < deadline and (state["armed"] is None
                                               or state["batt_v"] is None):
            await asyncio.sleep(0.1)
        if state["armed"] is not False:
            print(f"[bench] REFUSED: the vehicle reports armed={state['armed']} "
                  f"-- a props-off rehearsal needs a DISARMED vehicle")
            return 1
        if state["batt_v"] is None:
            print(f"[bench] REFUSED: no battery reading within "
                  f"{_BENCH_STREAM_WAIT_S:.0f} s -- cannot show that no flight "
                  f"battery is connected")
            return 1
        if state["batt_v"] > max_batt:
            print(f"[bench] REFUSED: battery reads {state['batt_v']:.1f} V (> "
                  f"{max_batt:.1f} V, --bench-max-batt-v): a flight battery is "
                  f"connected. Unplug it; this rehearsal runs on USB power.")
            return 1
        print(f"[bench] interlocks OK: disarmed, battery {state['batt_v']:.2f} V "
              f"(<= {max_batt:.1f} V). PROPS MUST BE OFF. This mode cannot arm.")
        print("[bench] stubs: armed gate off, height gate fed the standby "
              "height, yaw tolerance 180 deg. Everything else runs as it flies.")

        limits, unread = await read_platform_limits(drone)
        caps = commanded_caps(args, cfg, sm.guidance)
        for line in format_platform_table(limits, caps):
            print(line)
        hard, refusals, notes = check_platform_limits(limits, unread, caps, cfg)
        for n in notes:
            print(f"[limits] note: {n}")
        for h in hard:
            print(f"[limits] {h}")
        for r in refusals:
            print(f"[limits] --fly WOULD REFUSE: {r}")

        offboard_ok = await _enter_offboard(drone, cfg, state)
        if not offboard_ok:
            rc_exit = 1
        else:
            loop_out = await _control_loop(
                drone, args, cfg, sm, trigger, detector, state, frame_recorder,
                supervisor, outer_bound_s=args.smoke_duration,
                acquire_after_s=args.smoke_acquire_after_s,
                bench=True, bench_max_batt_v=max_batt)
            rc_exit = rc_exit or loop_out["rc_exit"]
    finally:
        if supervisor is not None:
            supervisor.unwatch()
        if loop_out.get("abort"):
            # Something is wrong with the bench itself (armed, or a battery).
            # Send NO mode command; just end the stream.
            _end_setpoint_stream(drone)
            print("[bench] setpoint stream ended; no mode command sent")
        elif offboard_ok:
            try:
                await drone.offboard.stop()
                print("[bench] offboard stopped (no land, no disarm: nothing "
                      "was ever armed)")
            except Exception as e:  # noqa: BLE001 -- best-effort teardown
                print(f"[bench] offboard.stop skipped: {e}")
        sup_exit = None
        if supervisor is not None:
            sup_exit = supervisor.close()
        for tk in tasks:
            tk.cancel()
        _write_mission_logs(args, loop_out["csv_rows"], frame_recorder)

    # ---- the rehearsal's own PASS/FAIL ----------------------------------------
    visited = sm.visited
    csv_rows = loop_out["csv_rows"]
    print(f"[mission] states visited: {' -> '.join(visited)}")
    print("[mission] transitions:")
    for tr in sm.transitions:
        print(f"    {tr.t:7.2f}s  {tr.frm} -> {tr.to}  ({tr.reason})")
    eng = [r.split(",") for r in csv_rows if f",{State.ENGAGE}," in r]
    i_mode = _CSV_FIELDS.index("mode")
    n_eng_offb = sum(1 for r in eng if r[i_mode] == "OFFBOARD")
    checks: List[Tuple[str, bool]] = [
        ("the vehicle stayed disarmed and no battery appeared",
         loop_out.get("abort") is None and state["armed"] is False),
        ("OFFBOARD was entered while disarmed (flight mode from telemetry)",
         loop_out["n_off_true"] > 0),
        (f"setpoints streamed ({loop_out['n_sp']})", loop_out["n_sp"] >= 20),
        ("the GO edge was accepted and ENGAGE ran", State.ENGAGE in visited),
        (f"every ENGAGE tick was in OFFBOARD ({n_eng_offb}/{len(eng)})",
         bool(eng) and n_eng_offb == len(eng)),
        ("every platform limit was read from the board", not unread),
    ]
    if isinstance(trigger, RcChannelTrigger):
        span = (0.0 if state["t_rc"] is None or state["t_rc_first"] is None
                else state["t_rc"] - state["t_rc_first"])
        rate = (state["rc_n"] - 1) / span if span > 0 else 0.0
        checks.append((f"RC_CHANNELS reached the Pi ({state['rc_n']} frames, "
                       f"{rate:.1f} Hz, worst gap {state['rc_max_gap_s']:.2f} s; "
                       f"link bound {cfg.link_timeout_s:.1f} s)",
                       state["rc_n"] > 1
                       and state["rc_max_gap_s"] <= cfg.link_timeout_s))
    if supervisor is not None:
        checks.append((f"the loop watchdog ran and exited clean (exit {sup_exit})",
                       sup_exit == 0))
    print("[bench] ---- props-off rehearsal ----")
    for what, ok in checks:
        print(f"[bench] {'ok  ' if ok else 'FAIL'} {what}")
        if not ok:
            rc_exit = 1
    print(f"[bench] measured: oldest own-state sample {sm.max_own_state_age_s:.2f} s "
          f"(FAILSAFE 9 bound {cfg.own_state_stale_s} s); worst control-tick gap "
          f"{loop_out.get('max_tick_gap_s', 0.0):.2f} s (watchdog bound "
          f"{getattr(args, 'loop_watchdog_s', 0.0)} s); oldest flight-mode sample "
          f"{sm.max_offboard_sample_age_s:.2f} s (FAILSAFE 7 bound "
          f"{cfg.offboard_stale_s} s)")
    print(f"[bench] {'PASS' if rc_exit == 0 else 'FAIL'}")
    return rc_exit


# ---------------------------------------------------------------- pilot hand-over

HANDOVER_DEFAULT_MAX_SPEED_MS = 0.9      # 2 mph


@dataclass(frozen=True)
class HandoverLimits:
    """What `--pilot-handover` lets out of the control loop, whatever the
    guidance asked for:

      * horizontal speed at most `max_speed_ms` (direction kept);
      * NO vertical command, ever: the aircraft holds the height the pilot left
        it at. The belief's height is the difference of two GPS heights, which
        disagreed by 2.4 m with both aircraft on the ground (2026-11-26);
      * in STANDBY and SAFE: zero velocity and the yaw the aircraft has now, so
        taking over or finishing never swings the nose.

    A non-finite command becomes a hold."""
    max_speed_ms: float = HANDOVER_DEFAULT_MAX_SPEED_MS

    def limit(self, sp: Setpoint, state_name: str,
              yaw_now_deg: Optional[float]) -> Setpoint:
        yaw_now = (float(yaw_now_deg) if yaw_now_deg is not None
                   and math.isfinite(float(yaw_now_deg)) else None)
        vals = (sp.v_north, sp.v_east, sp.yaw_deg)
        if state_name in (State.STANDBY, State.SAFE) \
                or not all(math.isfinite(float(x)) for x in vals):
            return Setpoint(0.0, 0.0, 0.0,
                            yaw_now if yaw_now is not None else
                            (float(sp.yaw_deg) if math.isfinite(float(sp.yaw_deg))
                             else 0.0))
        vn, ve = float(sp.v_north), float(sp.v_east)
        h = math.hypot(vn, ve)
        cap = max(0.0, float(self.max_speed_ms))
        if h > cap:
            k = cap / h if h > 0 else 0.0
            vn, ve = vn * k, ve * k
        return Setpoint(vn, ve, 0.0, float(sp.yaw_deg))


def handover_config(cfg: MissionConfig) -> MissionConfig:
    """The mission config for a pilot hand-over: the pilot chose the height and
    the heading, so the height gate is fed the standby height (as on the bench)
    and the yaw gate is open. Armed and OFFBOARD are still required for GO."""
    return replace(cfg, yaw_tol_deg=180.0)


def validate_handover_args(args) -> List[str]:
    """Reasons `--pilot-handover` must refuse to start. Empty list = allowed."""
    why: List[str] = []
    if not getattr(args, "mavsdk_url", None):
        why.append("--pilot-handover needs --mavsdk-url")
    if getattr(args, "trigger", None) != "cue":
        why.append("--pilot-handover needs --trigger cue (the target's live "
                   "position is what it flies toward)")
    if getattr(args, "terminal", "stock") != "pursuit":
        why.append("--pilot-handover needs --terminal pursuit")
    if getattr(args, "cue_no_enable", False):
        why.append("--pilot-handover needs the enable switch (drop "
                   "--cue-no-enable): the same switch is the Pixhawk's "
                   "offboard switch and your take-back")
    if getattr(args, "source", None):
        why.append("--pilot-handover flies on the cue alone: drop --source "
                   "(no camera calibration exists for the nose camera)")
    v = getattr(args, "handover_max_speed_ms", HANDOVER_DEFAULT_MAX_SPEED_MS)
    if not (0.0 < float(v) <= 3.0):
        why.append(f"--handover-max-speed-ms {v} is outside 0 to 3 m/s: this "
                   f"mode is for slow first flights")
    return why


async def run_pilot_handover(args, cfg: MissionConfig, sm: RealFlightSM,
                             trigger):  # pragma: no cover -- needs the board; driven by the fake in tests
    """PILOT HAND-OVER (`--pilot-handover`, 2026-11-26, builder's design). The
    pilot arms, takes off and hovers by hand. This program never arms, never
    takes off, never lands and never commands a flight mode: it only streams
    velocity setpoints. The Pixhawk's own offboard switch (RC_MAP_OFFB_SW, the
    same channel as the cue's enable switch) puts the aircraft in OFFBOARD when
    the pilot raises it and gives it back to the mode switch when he lowers it,
    with no help from this program.

    Raised: the Pixhawk enters OFFBOARD, the cue's estimate is latched (GO),
    and the chase flies toward the believed target under `HandoverLimits`.
    Lowered: the state machine sees OFFBOARD gone and ends. No camera."""
    import asyncio

    drone = await _open_vehicle(args)
    if drone is None:
        return 1
    limits_out = HandoverLimits(float(args.handover_max_speed_ms))
    state = _new_link_state()
    rc_sinks = _rc_readers(trigger)
    cue_height = _wire_cue_trigger(trigger, state)
    tasks = _start_telemetry(drone, state, rc_sinks=rc_sinks, want_battery=True,
                             utc_sink=getattr(trigger, "vehicle_utc", None),
                             want_gps_height=cue_height)
    supervisor, sup_ok = _start_supervisor(drone, args, required=True)
    rc_exit = 0
    loop_out = {"n_tick": 0, "n_sp": 0, "n_off_true": 0, "csv_rows": [],
                "abort": None, "max_tick_gap_s": 0.0}
    tick_log = TickLog(getattr(args, "log_csv", None))
    try:
        if not sup_ok:
            print("[handover] REFUSED: the setpoint-stream watchdog could not "
                  "be started")
            return 1
        deadline = time.monotonic() + _BENCH_STREAM_WAIT_S
        while time.monotonic() < deadline and (state["armed"] is None
                                               or state["mode"] is None):
            await asyncio.sleep(0.1)
        if state["armed"] is None or state["mode"] is None:
            print("[handover] REFUSED: no armed / flight-mode telemetry from "
                  "the vehicle")
            return 1
        if state["offboard"] is True:
            print("[handover] REFUSED: the vehicle is ALREADY in OFFBOARD. "
                  "Lower the GO switch and start again.")
            return 1
        print(f"[handover] PILOT HAND-OVER. This program cannot arm, take off, "
              f"land or change the flight mode. Vehicle is "
              f"{'ARMED' if state['armed'] else 'disarmed'}, mode "
              f"{state['mode']}.")
        print(f"[handover] limits on every command: horizontal speed <= "
              f"{limits_out.max_speed_ms:.2f} m/s, NO climb or descent, no "
              f"camera. GO switch raised = the Pixhawk's offboard switch hands "
              f"over; lowered = you have it back.")
        limits, unread = await read_platform_limits(drone)
        for line in format_platform_table(limits, commanded_caps(args, cfg, sm.guidance)):
            print(line)
        for line in what_ends_an_unbounded_engagement():
            print(f"[handover] also ends it: {line}")
        loop_out = await _control_loop(
            drone, args, cfg, sm, trigger, None, state, None, supervisor,
            outer_bound_s=args.smoke_duration, acquire_after_s=0.0,
            out=loop_out, tick_log=tick_log, handover=limits_out)
        # The loop's "bound reached" is a normal end here, not a failure.
        rc_exit = 0 if loop_out.get("abort") is None else 1
    finally:
        if supervisor is not None:
            supervisor.unwatch()
        # NO mode command. End the stream; if the aircraft were still in
        # OFFBOARD the Pixhawk's own offboard-loss rule takes it (Position).
        _end_setpoint_stream(drone)
        if supervisor is not None:
            supervisor.close()
        for tk in tasks:
            tk.cancel()
        try:
            _write_mission_logs(args, loop_out.get("csv_rows", []), None)
            tick_log.close(remove=True)
        except Exception as e:  # noqa: BLE001 -- keep the live copy
            tick_log.close(remove=False)
            print(f"[log] the final tick log could not be written "
                  f"({type(e).__name__}: {e}); the live copy is {tick_log.path}")
    print(f"[mission] states visited: {' -> '.join(sm.visited)}")
    for tr in sm.transitions:
        print(f"    {tr.t:7.2f}s  {tr.frm} -> {tr.to}  ({tr.reason})")
    print(f"[handover] largest horizontal command sent: "
          f"{loop_out.get('handover_max_h_ms', 0.0):.2f} m/s (limit "
          f"{limits_out.max_speed_ms:.2f}); ticks {loop_out.get('n_tick', 0)}, "
          f"of them in OFFBOARD {loop_out.get('n_off_true', 0)}; worst tick gap "
          f"{loop_out.get('max_tick_gap_s', 0.0):.2f} s")
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

    # 7) FAILSAFE 7 arm 1 -- PX4 leaves OFFBOARD mid-dash -> SAFE, and SAFE
    #    ABSORBS, so restoring OFFBOARD cannot resume the dash.
    sm = _sm(MissionConfig(preflight_heading_deg=40.0, standby_settle_s=0.0,
                           dash_max_s=60.0, offboard_lost_s=0.5))
    sm.step(_obs(0.0, trigger=no))
    sm.step(_obs(0.05, trigger=go))
    sm.step(_obs(0.10, trigger=no, offboard_active=False, mode="POSCTL"))
    still_dashing = sm.state == State.CODED_DASH        # transient, not yet a loss
    for i in range(20):
        sm.step(_obs(0.15 + 0.05 * i, trigger=no, offboard_active=False,
                     mode="POSCTL"))
    check(still_dashing and sm.state == State.SAFE
          and sm.safe_reason == "offboard_lost",
          f"FAILSAFE 7 arm 1: OFFBOARD lost after GO -> SAFE after the "
          f"persistence window, not on one tick ({sm.safe_reason})")
    d = sm.step(_obs(2.0, trigger=go, offboard_active=True, det_new=True,
                     det_range_m=9.0))
    check(sm.state == State.SAFE and d.setpoint.v_north == 0.0
          and d.setpoint.v_east == 0.0 and d.land_requested,
          "...and restoring OFFBOARD gets a ZERO-velocity hold + land request, "
          "never the half-ramped dash command")

    # 8) FAILSAFE 7 arm 2 -- offboard_active is still True but its SAMPLE is
    #    stale (a dead subscriber). Arm 1 can never fire on a frozen True.
    sm = _sm(MissionConfig(preflight_heading_deg=40.0, standby_settle_s=0.0,
                           dash_max_s=60.0, offboard_stale_s=3.0))
    sm.step(_obs(0.0, trigger=no))
    sm.step(_obs(0.05, trigger=go))
    sm.step(_obs(1.0, trigger=no, offboard_sample_age_s=2.0))
    fresh_ok = sm.state == State.CODED_DASH
    sm.step(_obs(5.0, trigger=no, offboard_sample_age_s=4.5))
    check(fresh_ok and sm.state == State.SAFE
          and sm.safe_reason == "offboard_lost",
          "FAILSAFE 7 arm 2: a FROZEN offboard sample (age 4.5s > 3.0s) aborts "
          "even though offboard_active still reads True")
    #    ...and a NEGATIVE age (two clock bases) is a hard fault, not health.
    sm = _sm(MissionConfig(preflight_heading_deg=40.0, standby_settle_s=0.0,
                           dash_max_s=60.0))
    sm.step(_obs(0.0, trigger=no))
    sm.step(_obs(0.05, trigger=go))
    sm.step(_obs(0.10, trigger=no, offboard_sample_age_s=-24580.0))
    check(sm.state == State.SAFE and "clock_fault" in sm.transitions[-1].reason,
          "a NEGATIVE offboard sample age (clock-base mismatch) reads as LOSS")

    # 9) BREAKOFF MAGNITUDE parity with the sim (ported guard): the COUNT alone
    #    must not fire breakoff on a slow drift, but a real recession still does.
    def _engage_then(ranges, min_rise):
        s = _sm(MissionConfig(preflight_heading_deg=40.0, standby_settle_s=0.0,
                              dash_max_s=60.0, engage_max_s=60.0,
                              breakoff_arm_range_m=4.0,
                              breakoff_range_deadband_m=0.0,
                              breakoff_min_rise_m=min_rise))
        s.step(_obs(0.0, trigger=no))
        s.step(_obs(0.05, trigger=go))
        t = 0.10
        for _ in range(5):
            s.step(_obs(t, trigger=no, det_new=True, det_range_m=3.0))
            t += 0.05
        assert s.state == State.ENGAGE, s.transitions
        for r in ranges:
            s.step(_obs(t, trigger=no, det_new=True, det_range_m=r))
            t += 0.05
        return s
    drift = [3.02, 3.04, 3.06, 3.08, 3.10]      # +0.10 m total: estimator drift
    # NB both lists start with the SEED detection (the first fresh range in
    # ENGAGE only sets the baseline; rises are counted from there).
    real = [3.0, 3.6, 4.4, 5.3]                 # +2.3 m from the base: past CPA
    check(_engage_then(drift, 0.30).state == State.ENGAGE,
          "breakoff MAGNITUDE guard: 5 rising detections totalling +0.10 m "
          "(drift) do NOT fire breakoff")
    check(_engage_then(real, 0.30).state == State.BREAKOFF,
          "...and a true post-CPA recession (+2.3 m) still does")
    check(_engage_then(drift, 0.0).state == State.BREAKOFF,
          "...and with min_rise=0 the same drift DOES fire it -- proof the guard "
          "is what changed the outcome, not the fixture")

    # 10) FAILSAFE 9 -- stale own-state -> SAFE, in STANDBY and after GO; and a
    #     driver that supplies no age leaves it inactive (the offline paths).
    sm = _sm(cfg)
    sm.step(_obs(0.0, trigger=no, own_state_age_s=0.02))
    sm.step(_obs(0.05, trigger=no, own_state_age_s=1.5))
    check(sm.state == State.SAFE and sm.safe_reason == "own_state_stale",
          "own-state 1.5 s old in STANDBY -> SAFE (a frozen height stream "
          "would otherwise drive the height loop)")
    sm = _sm(cfg)
    sm.step(_obs(0.0, trigger=no, own_state_age_s=0.02))
    sm.step(_obs(0.05, trigger=go, own_state_age_s=0.02))
    sm.step(_obs(0.10, trigger=no, own_state_age_s=1.5))
    check(sm.state == State.SAFE and sm.safe_reason == "own_state_stale",
          "...and after the GO edge too")
    sm = _sm(cfg)
    for i in range(60):
        sm.step(_obs(0.05 * i, trigger=no))
    check(sm.state == State.STANDBY,
          "...and with no age supplied (offline vehicle, desk) it never fires")
    stop, action, _why = teardown_plan("POSCTL", 0.2, 3.0, "land")
    check(stop is False and action is None,
          "teardown sends NOTHING once the pilot has taken a manual mode")
    stop, action, _why = teardown_plan("OFFBOARD", 0.2, 3.0, "land")
    check(stop is True and action == "land",
          "...and still lands an aircraft this program is flying")

    print(f"\n[self-test] {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# ---------------------------------------------------------------- CLI


def _negative_disables(v: Optional[float]) -> Optional[float]:
    """A NEGATIVE bound on the command line is the explicit "disable" spelling
    (argparse has no clean optional-float). It must never be read as a
    0-second bound, which would abort every flight instantly."""
    return None if v is not None and v < 0 else v


def build_config(args) -> MissionConfig:
    """Resolve the pre-flight constants ONCE, before anything flies."""
    # OPTIONAL GPS-fitted cue (docs/cue_relay_plan.md): overrides the
    # operator-typed --target-start/--target-vel with a belief fitted from
    # ~3 s of relayed target GPS (2 s before ADR-0130), and since ADR-0130
    # also sizes the ALTITUDE PLAN from the cue (standby = target height +
    # loft, so the dash arrives co-altitude). Honesty class: PRE-FLIGHT
    # constant, graded
    # given-noisy (real measured error, see the plan doc's budget), read at
    # EXACTLY this one call site before the state machine exists -- nothing
    # downstream re-reads the file, the same read-once mechanism that covers
    # the solved heading. FAIL-CLOSED: a cue that does not pass the quality
    # gate aborts the launch setup; it never silently degrades to a default.
    if getattr(args, "cue_json", None):
        from flight.cue_relay import CueQualityError, load_cue_for_real_flight
        if None in (args.own_lat, args.own_lon, args.own_alt_m_msl):
            raise SystemExit(
                "[cue] --cue-json requires --own-lat/--own-lon/--own-alt-m-msl "
                "(the interceptor's own pre-launch GPS fix)")
        # DELIBERATELY no now_t: record timestamps come from the GROUND
        # producer's clock, and comparing them against this Pi's clock is a
        # cross-clock-base staleness check -- the exact fault class
        # TriggerState.clock_fault exists for. Staleness is enforced where
        # the clock matches the records: the ground-side relay refuses to
        # ship a file older than the plan doc's T budget (§8).
        try:
            args.target_start, args.target_vel, cue_sol = load_cue_for_real_flight(
                args.cue_json, args.own_lat, args.own_lon, args.own_alt_m_msl)
        except CueQualityError as e:
            raise SystemExit(f"[cue] REFUSED (fail-closed): {e} -- retype "
                             f"--target-start/--target-vel by hand instead")
        _turn = (f", turn={math.degrees(cue_sol.omega_rad_s):+.1f} deg/s "
                 f"(1s {math.degrees(cue_sol.omega_sigma_rad_s):.1f})"
                 if getattr(cue_sol, "model", "cv") == "ct" else "")
        print(f"[cue] GPS-fitted belief: target-start={args.target_start} "
              f"target-vel={args.target_vel} (n={cue_sol.n_points} pts, "
              f"span={cue_sol.span_s:.2f}s, residual={cue_sol.residual_rms_m:.2f} m, "
              f"vel_source={cue_sol.vel_source}, "
              f"model={getattr(cue_sol, 'model', 'cv')}{_turn})")
        # ADR-0130: the cue's VERTICAL channel is consumed too. Datum (named,
        # per the ADR-0095/0100 rule): GPS-MSL minus GPS-MSL = height above
        # the interceptor's pre-launch antenna position, read against the
        # EKF's above-arm altitude -- the ~0.1-0.2 m antenna-above-arm offset
        # is accepted inside GPS vertical noise (1.5-2.5 m), which is the
        # dominant term of the +/-2 m target-height budget.
        from flight.cue_relay import cue_target_climb_ms, cue_target_height_m
        args.cue_target_h_m = cue_target_height_m(cue_sol)
        args.cue_target_vd_ms = -cue_target_climb_ms(cue_sol)  # NED down
        _sb = ("" if args.standby_alt_m is None else
               f" -- NOT driving the altitude plan: explicit --standby-alt-m="
               f"{args.standby_alt_m:.1f} wins")
        print(f"[cue] target altitude: {args.cue_target_h_m:+.1f} m above the "
              f"launch point (GPS-vertical datum, +/-2 m budget), climb "
              f"{cue_target_climb_ms(cue_sol):+.2f} m/s{_sb}")
    heading = args.dash_heading_deg
    t_lead = None
    dash_plan_m = None
    if heading is None:
        tstart = tuple(float(v) for v in args.target_start.split(",")[:2])
        tvel = tuple(float(v) for v in args.target_vel.split(",")[:2])
        heading, t_lead = resolve_preflight_heading(
            tstart, tvel, args.dash_speed, args.dash_accel_ms2,
            accel_aware=not args.no_accel_aware_lead)
        if t_lead is not None:
            # ADR-0101 passage window's plan distance: the along-heading
            # distance the pre-flight lead solve itself assumed the dash
            # would cover by t_lead. Mirrors scripts/m4_intercept.py's
            # `coded_dash_plan_m` exactly -- the accel-aware ramp INTEGRAL
            # (dash_ramp_distance, the same function collision_lead_heading_
            # accel roots against to find t_lead) when that solve was used,
            # the constant-speed product otherwise.
            dash_plan_m = (
                dash_ramp_distance(args.dash_speed, args.dash_accel_ms2, t_lead)
                if not args.no_accel_aware_lead
                else args.dash_speed * t_lead)
        print(f"[aim] pre-flight collision lead "
              f"({'accel-aware, ADR-0080' if not args.no_accel_aware_lead else 'constant-speed'}"
              f", a={args.dash_accel_ms2:.1f} m/s^2, v={args.dash_speed:.1f} m/s) "
              f"-> C1 = {heading:.2f} deg"
              + (f", t_lead={t_lead:.2f} s, plan distance="
                 f"{dash_plan_m:.2f} m (ADR-0101 passage window)"
                 if t_lead is not None else
                 ", t_lead=None (uncatchable by pure lead; ADR-0101 passage "
                 "window has no plan distance and stays OFF)"))
    else:
        print(f"[aim] C1 = {heading:.2f} deg (operator-entered --dash-heading-deg)"
              " -- ADR-0101 passage window has no plan distance here: the "
              "floor fails CLOSED, the ceiling never fires, by design "
              "(Portability note, ADR-0101)")
    if (args.breakoff_min_flown_frac is not None
            or args.breakoff_force_flown_frac is not None) and dash_plan_m is None:
        print("[aim] WARNING: --breakoff-min-flown-frac/--breakoff-force-flown-frac "
              "set but no plan distance was solved (explicit --dash-heading-deg) -- "
              "the floor will fail closed (breakoff never gated early) and the "
              "ceiling will never fire. This is by design (ADR-0101 Portability), "
              "not a bug -- pass a solved heading if the passage window matters.")
    is_pursuit = getattr(args, "terminal", "stock") == "pursuit"
    fly_pursuit = (bool(getattr(args, "fly", False))
                   or bool(getattr(args, "pilot_handover", False))) and is_pursuit
    explicit = set(getattr(args, "explicit_flags", None) or ())

    def _preset(name, typed):
        """A typed value wins; else FLY_PRESET under --fly in pursuit mode;
        else None (the caller's own default)."""
        if typed is not None:
            return typed
        return FLY_PRESET[name] if fly_pursuit else None

    def _onoff(v):
        return None if v is None else (v == "on")

    far_typed = getattr(args, "engage_far_lost_target_s", None)
    far_lost = _preset("engage_far_lost_target_s", far_typed)
    if far_lost is not None and far_lost < 0:
        far_lost = None                       # negative = off, as elsewhere
    engage_max = _preset("engage_max_s", args.engage_max_s)
    if engage_max is None:
        engage_max = 30.0 if is_pursuit else 12.0
    mission_max = _preset("mission_max_s", (args.mission_max_s
                                            if "mission_max_s" in explicit else None))
    if mission_max is None:
        mission_max = args.mission_max_s
    return MissionConfig(
        preflight_heading_deg=heading,
        # ADR-0130: C3 is computed per its own definition (standby = target
        # altitude + loft) from the MEASURED cue when one is loaded, instead
        # of a typed constant -- so dash_base_alt_m == the cue's target
        # height and the dash ends co-altitude. An explicit --standby-alt-m
        # wins; with neither, the as-flown 7.0 m default stands. The
        # min_agl/loft validators below still run on the resolved value and
        # fail closed on a geometry they can't fly.
        standby_alt_m=(args.standby_alt_m if args.standby_alt_m is not None
                       else (args.cue_target_h_m + args.dash_loft_m
                             if getattr(args, "cue_target_h_m", None) is not None
                             else 7.0)),
        standby_yaw_deg=args.standby_yaw_deg,
        dash_speed_ms=args.dash_speed,
        dash_accel_ms2=args.dash_accel_ms2,
        dash_accel_cap_ms2=args.dash_accel_cap,
        dash_loft_m=args.dash_loft_m,
        dash_loft_dive_s=args.dash_loft_dive_s,
        dash_max_s=args.dash_max_s,
        dash_max_dist_m=args.dash_max_dist_m,
        acquire_streak=args.acquire_streak,
        # ADR-0129 (2026-10-22): the engage-window default is terminal-mode-
        # aware. 12 s was sized for the sprint-era fly-by; the pursuit chase
        # measured 6-13 s to contact from ENGAGE entry (ADR-0103 campaign) and
        # its approach dash burns the same clock, so a 12 s default can time a
        # real chase out MID-ENGAGEMENT (ADR-0105 open item (b), closed here).
        # 30 s = max measured contact time + transit margin; the t_transit_min
        # launch check still warns when a long cue needs more. Stock/tag keep
        # the as-flown 12 s. An explicit --engage-max-s wins in any mode.
        engage_max_s=engage_max,
        standby_settle_s=args.standby_settle_s,
        standby_max_s=args.standby_max_s,
        mission_max_s=mission_max,
        pursuit_lost_clock_needs_track=bool(_preset(
            "pursuit_lost_clock_needs_track",
            _onoff(getattr(args, "lost_clock_needs_track", None)))),
        pursuit_lost_clock_off_in_search=bool(_preset(
            "pursuit_lost_clock_off_in_search",
            _onoff(getattr(args, "lost_clock_off_in_search", None)))),
        engage_far_lost_target_s=far_lost,
        camera_dead_s=_negative_disables(_preset(
            "camera_dead_s", getattr(args, "camera_dead_s", None))),
        cue_go=(getattr(args, "trigger", None) == "cue"),
        link_timeout_s=args.link_timeout_s,
        alt_tol_m=args.alt_tol_m,
        dash_alt_trim_m=args.dash_alt_trim_m,
        min_agl_m=args.min_agl_m,
        yaw_tol_deg=args.yaw_tol_deg,
        offboard_lost_s=args.offboard_lost_s,
        # A NEGATIVE bound is the explicit "disable arm 2" spelling (argparse has
        # no clean optional-float); it must never be read as a 0-second bound,
        # which would abort every flight instantly.
        offboard_stale_s=(None if args.offboard_stale_s is not None
                          and args.offboard_stale_s < 0 else args.offboard_stale_s),
        # Same spelling for FAILSAFE 9: negative = disabled. getattr, because
        # callers that build their own Namespace predate the flag.
        own_state_stale_s=_negative_disables(
            getattr(args, "own_state_stale_s", 1.0)),
        breakoff_min_rise_m=args.breakoff_min_rise_m,
        breakoff_max_range_m=args.breakoff_max_range_m,
        breakoff_min_flown_frac=args.breakoff_min_flown_frac,
        breakoff_force_flown_frac=args.breakoff_force_flown_frac,
        dash_plan_m=dash_plan_m,
        pursuit_miss_range_m=args.pursuit_miss_range_m,
        safe_behavior=args.safe_behavior,
        miss_safe_behavior=args.miss_safe_behavior,
        # ADR-0103 "chase only": --terminal pursuit implies pursuit_mode (the
        # receding-range breakoff suppression AND the GO->ENGAGE direct
        # transition). Never settable independently from this CLI -- a pursuit
        # terminal under the fly-by breakoff rule is a false-abort machine.
        pursuit_mode=(getattr(args, "terminal", "stock") == "pursuit"),
    )


def lost_clock_lines(cfg: MissionConfig, pcfg) -> List[str]:
    """The lost-target rule as it will fly, in plain lines, with a WARNING when
    the wrapper's clock is not longer than the terminal's own fallback time (the
    terminal's return-to-belief logic can then never run). Pure."""
    far = cfg.engage_far_lost_target_s
    eff = cfg.engage_lost_target_s if far is None else far
    lines = [
        f"[terminal] lost-target rule: {cfg.engage_lost_target_s:.1f} s without a "
        f"decode ends the engagement"
        + ("" if far is None else
           f" inside {cfg.pursuit_miss_range_m:.0f} m, {far:.1f} s beyond it")
        + ("; the clock starts only once the terminal has a track"
           if cfg.pursuit_lost_clock_needs_track else "")
        + ("; it does not run during the arrival search"
           if cfg.pursuit_lost_clock_off_in_search else "") + "."]
    if eff <= float(pcfg.fallback_s):
        lines.append(
            f"[terminal] WARNING: the lost-target clock ({eff:.1f} s) is not "
            f"longer than the terminal's own fallback time "
            f"({float(pcfg.fallback_s):.1f} s), so a target lost while still "
            f"ahead ends the engagement before the terminal can return to its "
            f"belief chase (--engage-far-lost-target-s).")
    return lines


def build_cue_trigger(args, cfg: MissionConfig) -> "CueTrigger":
    """`--trigger cue`: start the live cue link and wrap it as the GO trigger.
    The driver supplies own position and, under --fly, the GO-time checks."""
    from flight.cue_link import CueReceiver, MonoToUtc, SystemClockUtc
    src = args.cue_utc_source
    # ONE clock: the link, the UTC mapping and the driver all read this
    # module's `time.monotonic` (the trigger is polled with it, and the latch
    # advances the belief to it).
    def mono() -> float:
        return time.monotonic()

    vehicle_utc = MonoToUtc(clock=mono) if src == "vehicle" else None
    utc = vehicle_utc if src == "vehicle" else (
        SystemClockUtc(clock=mono, wall=lambda: time.time())
        if src == "system" else None)
    rx = CueReceiver(port=args.cue_udp_port, utc=utc, clock=mono,
                     allow_unsynced=bool(args.cue_allow_unsynced))
    rx.start()
    enable = (None if args.cue_no_enable else
              RcChannelTrigger(args.rc_channel, args.rc_go_us, args.rc_low_us,
                               cfg.link_timeout_s))
    trig = CueTrigger(rx, own_lla=lambda: None, enable=enable,
                      window_s=args.cue_window_s,
                      min_span_s=args.cue_min_span_s,
                      max_staleness_s=args.cue_max_staleness_s,
                      max_gap_s=args.cue_max_gap_s)
    trig.vehicle_utc = vehicle_utc
    print(f"[cue] THE CUE IS THE GO COMMAND. Listening on UDP {rx.port}; UTC "
          f"source '{src}'"
          + (" (delay is measured per record and removed)" if utc is not None
             else " (no shared clock)")
          + ("; unsynced records ALLOWED (delay then not measured)"
             if args.cue_allow_unsynced else "; unsynced records REFUSE GO")
          + (f"; enable switch on RC channel {args.rc_channel} (LOW = your veto)"
             if enable is not None else "; NO enable switch (no human veto)"))
    return trig


def build_terminal(args, cfg: MissionConfig, gcfg, cam):
    """Construct the ENGAGE terminal `main()` hands to `RealFlightSM` --
    factored out of `main()` so tests can assert the class and the belief
    seed without running a mission. 'stock' is byte-for-byte the historical
    path (SeekerGuidance)."""
    which = getattr(args, "terminal", "stock")
    if which == "stock":
        return SeekerGuidance(gcfg, cam, gcfg.target_span_m)
    if which == "tag":
        from flight.tag_terminal import TagInterceptGuidance, TagTerminalConfig
        return TagInterceptGuidance(TagTerminalConfig(), cam, gcfg.target_span_m,
                                    gcfg)
    assert which == "pursuit", which
    from flight.pursuit_terminal import (PursuitTerminalConfig,
                                         PursuitTerminalGuidance)
    # Phase-A belief seed -- the SAME pre-flight constants the C1 lead solve
    # reads (--target-start/--target-vel, east-north metres about the launch
    # point, latched at trigger; honesty-clean, never a live/gt read). NED:
    # [north, east, down]. Vertical (ADR-0130): with a cue loaded the down
    # component is the TRUE relative geometry at GO -- vehicle at
    # standby_alt_m, target at the cue's measured height -- and the vertical
    # velocity is the cue's fitted climb/descent. When standby was itself
    # auto-sized from the cue (standby = height + loft) this reduces to the
    # historical +dash_loft_m; it differs only when the operator overrides
    # --standby-alt-m, where the old constant was simply wrong. Without a
    # cue: byte-identical to the pre-ADR-0130 seed (+loft, 0.0).
    t_e, t_n = (float(v) for v in args.target_start.split(",")[:2])
    v_e, v_n = (float(v) for v in args.target_vel.split(",")[:2])
    cue_h = getattr(args, "cue_target_h_m", None)
    if cue_h is not None:
        belief_r0_ned = (t_n, t_e, cfg.standby_alt_m - float(cue_h))
        belief_vel0_ned = (v_n, v_e, float(getattr(args, "cue_target_vd_ms", 0.0)))
    else:
        belief_r0_ned = (t_n, t_e, cfg.dash_loft_m)
        belief_vel0_ned = (v_n, v_e, 0.0)
    print(f"[terminal] pursuit (ADR-0103 chase only): belief seed rel NED "
          f"({belief_r0_ned[0]:+.1f}, {belief_r0_ned[1]:+.1f}, "
          f"{belief_r0_ned[2]:+.1f}) m at {belief_vel0_ned[0]:+.1f}/"
          f"{belief_vel0_ned[1]:+.1f} m/s -- receding-range breakoff "
          f"SUPPRESSED (contact floor / lost-target / engage-max-s active, "
          f"engage_max_s={cfg.engage_max_s:.0f}s)")
    # go_at_s=0.0: the state machine already gates the terminal -- step() is
    # first called the tick ENGAGE begins (= the GO edge in pursuit_mode), so
    # the class's own pre-GO zero-velocity hold must never re-trigger here.
    # For a non-timer trigger (rc/gate) the GO instant is not a pre-flight
    # constant anyway; the SM's gating is what makes this seedable at all.
    pcfg = PursuitTerminalConfig()
    if getattr(args, "pursuit_meas_latency_s", None) is not None:
        pcfg = replace(pcfg, meas_latency_s=float(args.pursuit_meas_latency_s))
        print(f"[terminal] pursuit KF assumed measurement age set to "
              f"{pcfg.meas_latency_s * 1e3:.0f} ms (--pursuit-meas-latency-s)")
    if getattr(args, "pursuit_brake", False):
        # The ADOPTED package exactly as registered/measured (a3-horizontal +
        # vert-sync); constants are the prereg's, not per-flight tunables.
        pcfg = replace(pcfg, brake_shaping=True,
                       brake_accel_ms2=3.0, brake_vert_sync=True)
        print("[terminal] pursuit BRAKE PACKAGE ON (ADR-0115: a=3 m/s^2, "
              "lead 0.45 s, horizontal-only cap + vertical arrival-sync)")
    # THE FLYING PRESET (FLY_PRESET): --fly takes the search, the guard and the
    # approach-dash brake from it unless the operator typed the matching flag.
    fly_preset = (bool(getattr(args, "fly", False))
                  or bool(getattr(args, "pilot_handover", False)))
    if getattr(args, "pursuit_approach_dash", False):
        pcfg = replace(pcfg, approach_dash=True,
                       dash_v_max_ms=float(args.pursuit_dash_vmax),
                       dash_accel_max_ms2=float(args.pursuit_dash_accel))
        _brake = getattr(args, "pursuit_dash_brake_accel", None)
        if _brake is not None:
            pcfg = replace(pcfg, brake_accel_ms2=float(_brake))
        elif fly_preset:
            pcfg = replace(pcfg,
                           brake_accel_ms2=float(FLY_PRESET["pursuit_dash_brake_accel"]),
                           brake_vert_sync=bool(FLY_PRESET["pursuit_brake_vert_sync"]))
            print(f"[terminal] --fly preset: approach-dash brake "
                  f"{pcfg.brake_accel_ms2:.1f} m/s^2 with vertical arrival-sync "
                  f"(--pursuit-dash-brake-accel to change)")
        print(f"[terminal] pursuit APPROACH DASH ON (transit cap "
              f"{pcfg.dash_v_max_ms:.0f} m/s, slew {pcfg.dash_accel_max_ms2:.0f} "
              f"m/s^2, decel schedule a={pcfg.brake_accel_ms2:.1f} m/s^2 into "
              f"the believed rendezvous; Phase-A brake cap forced on)")
        # Fail-LOUD window check (not fatal: the belief is a cue, not truth):
        # a long-range transit that cannot fit the engage window would end in
        # engage_timeout mid-dash -- say so at launch, not in the debrief.
        # Transit lower bound: straight-line belief range at the dash ceiling
        # plus the braking tail and a terminal allowance.
        belief_range = math.hypot(belief_r0_ned[0], belief_r0_ned[1])
        # (This is the COMMANDED ceiling: no link is open yet, so the board's
        # own limit is unknown here. --fly repeats the check with
        # min(commanded, board) once it has read MPC_XY_VEL_MAX.)
        t_transit_min = approach_transit_lower_bound_s(
            belief_range, pcfg.dash_v_max_ms, pcfg.brake_accel_ms2)
        if cfg.engage_max_s < t_transit_min:
            print(f"[terminal] WARNING: engage_max_s={cfg.engage_max_s:.0f}s is "
                  f"shorter than the approach-dash transit lower bound "
                  f"(~{t_transit_min:.0f}s for the {belief_range:.0f} m cue at "
                  f"{pcfg.dash_v_max_ms:.0f} m/s) -- the engagement would time "
                  f"out mid-dash; raise --engage-max-s. ALSO NOTE: offboard-"
                  f"loss failsafe distances scale with speed (0.5 s at "
                  f"{pcfg.dash_v_max_ms:.0f} m/s = {0.5*pcfg.dash_v_max_ms:.0f} m "
                  f"unflown -- TODO-BUILDER constants, bench L3/L6).")
    _search = getattr(args, "pursuit_arrival_search", None)
    if _search is None:                       # not typed
        _search = FLY_PRESET["pursuit_arrival_search"] if fly_preset else "off"
    _guard = getattr(args, "pursuit_stale_decode_guard", None)
    _guard = (FLY_PRESET["pursuit_stale_decode_guard"] if fly_preset else False) \
        if _guard is None else (_guard == "on")
    if _guard:
        pcfg = replace(pcfg, stale_decode_guard=True)
        print("[terminal] pursuit STALE-DECODE GUARD ON (a dropout cannot restart "
              "the camera phase from decodes older than the acquisition window)")
    if _search != "off":
        pcfg = replace(pcfg, arrival_search=True, search_start=_search)
        if getattr(args, "pursuit_search_speed_sigma", None) is not None:
            pcfg = replace(pcfg, search_sigma_speed_frac=float(
                args.pursuit_search_speed_sigma))
        print(f"[terminal] pursuit ARRIVAL SEARCH ON (start '{_search}', declared "
              f"speed error {pcfg.search_sigma_speed_frac*100:.0f} % one sigma, "
              f"sweep +{pcfg.search_sweep_ms:.0f} / drop -{pcfg.search_drop_ms:.0f} "
              f"m/s about the believed target; fast-sim result only, "
              f"isim/specs/arrival_search_prereg_2026-11-18.md)")
    if getattr(args, "pursuit_rehearsal", False):
        # PRACTICE mode: ONE switch applies the WHOLE registered practice
        # profile (rehearsal + brake package + reduced 10 m/s cap) -- the
        # round-2 sweep's P1 margins are CONDITIONAL on that profile, so a
        # bare rehearsal trigger at full speed would fly with unproven
        # margins (isim/specs/rehearsal_breakoff_prereg_2026-09-24.md,
        # amendment #1 RESULT; decide-and-log per ADR-0118 discipline).
        pcfg = replace(pcfg, rehearsal_breakoff=True, brake_shaping=True,
                       brake_accel_ms2=3.0, brake_vert_sync=True,
                       v_max_ms=10.0)
        print(f"[terminal] pursuit REHEARSAL practice profile ON (no contact: "
              f"brake a=3 + vert-sync, v_max 10 m/s; evade at t_go <= "
              f"{pcfg.rehearsal_t_react_s:.1f} s (r_hat <= "
              f"{pcfg.rehearsal_range_m:.1f} m) on a fed track, too late "
              f"below t_go {pcfg.rehearsal_t_late_s:.1f} s, then SAFE "
              f"'rehearsal_breakoff'; isim-derived margins -- Gazebo "
              f"spot-check pending)")
    if cfg.cue_go:
        print("[terminal] --trigger cue: the belief above is a PLACEHOLDER. The "
              "real one is latched from the live cue at the GO edge.")
    for line in lost_clock_lines(cfg, pcfg):
        print(line)
    return PursuitTerminalGuidance(
        pcfg, cam, gcfg.target_span_m, gcfg,
        belief_r0_ned=belief_r0_ned, belief_vel0_ned=belief_vel0_ned,
        go_at_s=0.0, initial_yaw_deg=cfg.preflight_heading_deg)


def build_arg_parser() -> argparse.ArgumentParser:
    """The real CLI parser, factored out of `main()` so tests can parse the
    same flags `main()` does (`--terminal` construction tests) without
    running a mission."""
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
    mode.add_argument("--desk", action="store_true",
                      help="DESK verification: the mission state machine over "
                           "REAL frames (--source) + the REAL detector, with NO "
                           "vehicle link -- no MAVSDK, cannot arm; setpoints "
                           "computed + logged, never sent. Own-state is the "
                           "FakeVehicle stand-in.")
    mode.add_argument("--fly", action="store_true",
                      help="THE REAL FLIGHT, and the only mode that may arm "
                           "over a serial link. Needs --mavsdk-url, --trigger "
                           "rc, a real camera (--source picamera), an explicit "
                           "belief (--cue-json, or typed --target-start and "
                           "--target-vel with --manual-belief) and "
                           "--arm-permit-channel. It reads the board's limits, "
                           "prints what it is about to do, and arms only on "
                           "the operator's RC arm permit.")
    mode.add_argument("--bench-props-off", action="store_true",
                      help="PROPS-OFF REHEARSAL on the real board: requires a "
                           "disarmed vehicle with no flight battery, never "
                           "arms or takes off, and runs STANDBY -> GO -> "
                           "ENGAGE with the real setpoint stream in OFFBOARD. "
                           "Needs --mavsdk-url and --trigger rc|timer. "
                           "Without --source the detector is synthetic. "
                           "Exits 0/1.")
    mode.add_argument("--pilot-handover", action="store_true",
                      help="PILOT HAND-OVER: you arm, take off and hover; the "
                           "program only streams velocity commands. The "
                           "Pixhawk's offboard switch (RC_MAP_OFFB_SW = the "
                           "--rc-channel switch) hands the aircraft over when "
                           "raised and back when lowered. Cue only, no camera, "
                           "speed limited, height held. Never arms, takes off, "
                           "lands or changes mode. Needs --mavsdk-url and "
                           "--trigger cue.")
    mode.add_argument("--handover-max-speed-ms", type=float,
                      default=HANDOVER_DEFAULT_MAX_SPEED_MS,
                      help="--pilot-handover: horizontal speed limit on every "
                           "command, m/s (default 0.9 = 2 mph; at most 3)")
    mode.add_argument("--mavsdk-url", default=None,
                      help="MAVSDK connect URL (e.g. udpin://0.0.0.0:14540)")

    src = ap.add_argument_group("camera source (flight/deploy/frame_feed.py)")
    src.add_argument("--source", default=None,
                     help="'picamera' (live OV9281) or 'dir:PATH' (image replay). "
                          "Absent = the synthetic detector (unchanged). Valid "
                          "with --desk or --sitl-smoke.")
    src.add_argument("--camera-mount", choices=("upright", "upside-down"),
                     default=None,
                     help="how the camera is mounted. 'upside-down' rotates "
                          "the image 180 deg at the sensor and requires a "
                          "calibration captured the same way (its "
                          "image_flip_deg must be 180). Required by --fly.")
    src.add_argument("--detector", choices=("tag", "onnx"), default="tag",
                     help="with --source: 'tag' = AprilTag tag36h11 (needs "
                          "--target-span-m = the tag's black-square size), "
                          "'onnx' = seeker_loop.build_detector (--weights)")
    src.add_argument("--tag-id", type=int, default=0)
    src.add_argument("--quad-decimate", type=float, default=2.0,
                     help="AprilTag decimate (2.0 = the rate measured in the "
                          "Pi fps soak, ADR-0090)")
    src.add_argument("--weights", default=None,
                     help="--detector onnx weights (default: seeker_loop's)")
    src.add_argument("--conf", type=float, default=0.25)
    src.add_argument("--gray-input", choices=("auto", "on", "off"), default="auto")
    src.add_argument("--camera-fps", type=float, default=None,
                     help="--source picamera sensor rate (default: seeker_loop "
                          "CAMERA_FPS_DEFAULT)")
    src.add_argument("--exposure-us", type=float, default=None,
                     help="--source picamera exposure override, us (default: "
                          "the <=1 ms flight spec). BENCH knob: a monitor "
                          "target needs ~8000; never fly an override")
    src.add_argument("--gain", type=float, default=None,
                     help="--source picamera analogue gain override "
                          "(default: seeker_loop's auto-gain behavior)")
    src.add_argument("--require-span-calib", action="store_true",
                     help="--detector onnx: refuse an uncalibrated span")

    aim = ap.add_argument_group("pre-flight aim constants (C1/C2/C3)")
    aim.add_argument("--dash-heading-deg", type=float, default=None,
                     help="C1 override (NED compass deg). Omit to solve the "
                          "collision lead from --target-start/--target-vel.")
    aim.add_argument("--target-start", default="0,16.7", action=_ExplicitStore,
                     help="operator-entered target position at launch, 'east,north' m")
    aim.add_argument("--target-vel", default="9,0", action=_ExplicitStore,
                     help="operator-entered target velocity, 'east,north' m/s")
    aim.add_argument("--manual-belief", action="store_true",
                     help="--fly: acknowledge that the belief was TYPED "
                          "(--target-start/--target-vel) and did not come "
                          "from the GPS cue (--cue-json)")
    aim.add_argument("--no-accel-aware-lead", action="store_true",
                     help="use the old CONSTANT-SPEED lead solve (under-leads: the "
                          "dash starts at rest -- see ADR-0080)")
    aim.add_argument("--standby-alt-m", type=float, default=None,
                     help="C3 = target altitude + loft. Default: computed "
                          "from the cue's measured target height when "
                          "--cue-json is given (ADR-0130), else the as-flown "
                          "7.0 m. An explicit value always wins.")
    aim.add_argument("--standby-yaw-deg", type=float, default=None,
                     help="standby aim yaw; default = C1")
    aim.add_argument("--cue-json", default=None,
                     help="flight.cue_relay JSON-lines cue file "
                          "(docs/cue_relay_plan.md) -- overrides "
                          "--target-start/--target-vel with a GPS-fitted, "
                          "FAIL-CLOSED belief. Requires --own-lat/--own-lon/"
                          "--own-alt-m-msl (the interceptor's own pre-launch "
                          "fix). Read exactly once, in build_config, before "
                          "anything flies -- same read-once discipline as the "
                          "solved heading.")
    aim.add_argument("--own-lat", type=float, default=None,
                     help="interceptor pre-launch latitude (with --cue-json)")
    aim.add_argument("--own-lon", type=float, default=None,
                     help="interceptor pre-launch longitude (with --cue-json)")
    aim.add_argument("--own-alt-m-msl", type=float, default=None,
                     help="interceptor pre-launch altitude MSL (with --cue-json)")

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
    dash.add_argument("--engage-max-s", type=float, default=None,
                      help="hard cap on the ENGAGE phase. Default is TERMINAL-"
                           "MODE-AWARE (ADR-0129): 30 s in pursuit mode (chases "
                           "measured 6-13 s to contact PLUS the approach-dash "
                           "transit share the same window), 12 s as-flown for "
                           "stock/tag (fly-by-sized). An explicit value wins in "
                           "any mode.")

    saf = ap.add_argument_group("safety / failsafe (ALL TODO-BUILDER)")
    saf.add_argument("--standby-settle-s", type=float, default=2.0)
    saf.add_argument("--standby-max-s", type=float, default=60.0)
    saf.add_argument("--mission-max-s", type=float, default=120.0,
                     action=_ExplicitStore,
                     help="absolute backstop on the whole run, any state. Under "
                          "--fly in pursuit mode there is NO limit unless you "
                          "type one (FLY_PRESET, builder ruling 2026-11-18).")
    saf.add_argument("--lost-clock-needs-track", choices=("on", "off"), default=None,
                     help="pursuit: a decode does not start the lost-target "
                          "clock until the terminal has had a track. Default "
                          "off; --fly takes FLY_PRESET unless typed.")
    saf.add_argument("--lost-clock-off-in-search", choices=("on", "off"),
                     default=None,
                     help="pursuit: the lost-target clock does not run while the "
                          "terminal is in its arrival search. Default off; "
                          "--fly takes FLY_PRESET unless typed.")
    saf.add_argument("--engage-far-lost-target-s", type=float, default=None,
                     help="pursuit: lost-target clock used when the target was "
                          "last estimated beyond the miss range (negative = "
                          "off). Default off; --fly takes FLY_PRESET unless typed.")
    saf.add_argument("--link-timeout-s", type=float, default=1.0)
    saf.add_argument("--alt-tol-m", type=float, default=0.3)
    # ADR-0099. Both were settable ONLY by editing source or building a
    # MissionConfig in Python -- `build_config()` never set them, so every
    # documented run path (--dry-run, --sitl-smoke, the real driver) ran with the
    # lever hard-wired off while the pre-registration said "only the flag values
    # change". That is the phantom-flag class tests/test_inert_flag_guards.py
    # exists to prevent. (Review finding F2.)
    saf.add_argument("--dash-alt-trim-m", type=float, default=0.0,
                     help="ADR-0099 pre-flight altitude trim on the DASH and "
                          "terminal references, metres, DERIVED from a measured "
                          "dash altitude drift via "
                          "flight.guidance.derive_dash_alt_trim_m -- NOT tuned. "
                          "Positive raises the reference. Default 0.0 = inert. "
                          "Does NOT touch the standby hold or the arm gate. "
                          "Refused if |trim| >= --alt-tol-m.")
    saf.add_argument("--min-agl-m", type=float, default=None,
                     help="ADR-0085 hard floor: the vehicle is never commanded to "
                          "descend at or below this altitude, in ANY state, "
                          "including on a setpoint the seeker produced. Measured "
                          "against the EKF altitude relative to the ARM POINT, so "
                          "it is only true AGL on level ground -- set it from the "
                          "site's own geometry. Default None = no floor, which is "
                          "the pre-existing behaviour and NOT a safe default for "
                          "props-on flight.")
    saf.add_argument("--yaw-tol-deg", type=float, default=2.0)
    saf.add_argument("--offboard-lost-s", type=float, default=0.5,
                     help="FAILSAFE 7 arm 1: how long PX4 may be OUT of OFFBOARD "
                          "after the GO edge before aborting to SAFE")
    saf.add_argument("--offboard-stale-s", type=float, default=3.0,
                     help="FAILSAFE 7 arm 2: a flight-mode sample older than this "
                          "is treated as LOSS (a frozen subscriber pins "
                          "offboard_active True). Negative disables the arm.")
    saf.add_argument("--own-state-stale-s", type=float, default=1.0,
                     help="FAILSAFE 9: own-state (attitude, yaw, height, "
                          "velocity) whose OLDEST stream is older than this "
                          "goes SAFE, in any state. Negative disables it.")
    saf.add_argument("--camera-dead-s", type=float, default=None,
                     help="in ENGAGE, a camera that has delivered no frame at "
                          "all for this long goes SAFE (default: 3.0 under "
                          "--fly in pursuit mode, off elsewhere; a negative "
                          "value turns it off)")
    saf.add_argument("--mavsdk-backup-url", default=None,
                     help="a SECOND MAVLink link to the same Pixhawk, e.g. "
                          "udpout://127.0.0.1:14550 (the flight recorder's "
                          "bridge to the Pixhawk's USB port). Setpoints then "
                          "go out on both links and telemetry falls back to "
                          "this one when the primary is silent "
                          "(flight/deploy/link_redundancy.py). If given and it "
                          "does not connect, the program refuses to go on.")
    saf.add_argument("--board-failsafes", choices=("keep", "suspend"),
                     default=None,
                     help="'suspend' (the --fly default, builder ruling "
                          "2026-11-26): on the disarmed aircraft switch the "
                          "Pixhawk's low-battery action and GPS quality checks "
                          "off for this flight, apply both checks in this "
                          "program before takeoff, and restore the board after "
                          "landing (flight/deploy/board_overrides.py). 'keep': "
                          "change nothing on the board; its low-battery action "
                          "or a GPS dip can then end the intercept.")
    saf.add_argument("--board-state-path", default=None,
                     help="where the board's original settings are saved "
                          "during a flight (default ~/.interceptor/"
                          "board_overrides.json)")
    saf.add_argument("--loop-watchdog-s", type=float, default=0.5,
                     help="live paths: an out-of-process watchdog "
                          "(flight/deploy/loop_supervisor.py) ends the "
                          "setpoint stream if the control loop has not ticked "
                          "for this long, or if this program dies, so PX4's "
                          "own offboard-loss failsafe acts. 0 = off (refused "
                          "by --fly).")
    saf.add_argument("--override-platform-limits", action="store_true",
                     help="--fly: go ahead although the guidance commands "
                          "more than the board flies, or the believed "
                          "rendezvous is outside the geofence, or the transit "
                          "cannot fit the engage window. Never overrides a "
                          "limit that could not be READ.")
    saf.add_argument("--bench-max-batt-v", type=float, default=6.0,
                     help="--bench-props-off: refuse (and abort) above this "
                          "battery voltage. USB power reads about 0 V; any "
                          "flight battery reads far more.")
    saf.add_argument("--breakoff-min-rise-m", type=float, default=0.30,
                     help="past-CPA breakoff MAGNITUDE guard: total range rise "
                          "from the base of the rising run (mirrors m4's "
                          "--breakoff-min-rise-m). 0 = count-only (unguarded).")
    saf.add_argument("--breakoff-max-range-m", type=float, default=None,
                     help="past-CPA breakoff is ignored beyond this range "
                          "(mirrors m4's --breakoff-max-range-m; default inert)")
    saf.add_argument("--breakoff-min-flown-frac", type=float, default=None,
                     help="ADR-0101 passage window FLOOR: the past-CPA "
                          "breakoff may not fire until the vehicle's own "
                          "displacement since the dash began reaches this "
                          "fraction of the pre-flight PLAN distance (mirrors "
                          "m4's --breakoff-min-flown-frac). Needs a SOLVED "
                          "dash heading (not --dash-heading-deg) -- see the "
                          "[aim] log line. Default None = OFF (byte-identical).")
    saf.add_argument("--breakoff-force-flown-frac", type=float, default=None,
                     help="ADR-0101 passage window CEILING: force the "
                          "breakoff once that same displacement reaches this "
                          "fraction of the plan distance, whatever the "
                          "camera says (mirrors m4's "
                          "--breakoff-force-flown-frac). Default None = OFF "
                          "(byte-identical).")
    saf.add_argument("--safe-behavior", choices=("land", "hover", "rtl"),
                     default="land",
                     help="what happens after the safe_hold_s hold in SAFE: "
                          "'land' (default, unchanged) -- the driver requests "
                          "PX4 LAND; 'hover' -- hold position indefinitely, no "
                          "land request, until the mission_max_s backstop; "
                          "'rtl' -- the DRIVER commands PX4 RETURN-TO-LAUNCH "
                          "instead of LAND (PX4's own GPS-based safety "
                          "behavior, not a guidance decision). The state "
                          "machine keeps emitting SAFE's hold setpoint every "
                          "tick regardless of this choice. Applies to "
                          "SYSTEM-HEALTH aborts; a MISS takes "
                          "--miss-safe-behavior instead.")
    saf.add_argument("--miss-safe-behavior", choices=("land", "hover", "rtl"),
                     default="hover",
                     help="SAFE behavior when the ENGAGEMENT ended without "
                          "contact (reasons pursuit_miss / breakoff_complete "
                          "/ rehearsal_breakoff) "
                          "-- builder ruling 2026-09-22 (ADR-0107): HOVER by "
                          "default, holding position for the operator; a "
                          "future re-approach feature slots in here. The "
                          "mission_max_s backstop still lands a hover.")

    trg = ap.add_argument_group("trigger")
    trg.add_argument("--trigger", choices=("rc", "cue", "timer", "gate-ready"),
                     default="gate-ready",
                     help="'rc' = the REAL ELRS aux channel via MAVLink RC_CHANNELS; "
                          "'cue' = THE CUE IS THE GO COMMAND: GO fires when the "
                          "live cue link holds a fresh, quality-gated fit, and "
                          "the belief is latched at that instant (pursuit "
                          "terminal only; --rc-channel becomes the ENABLE "
                          "switch, your veto); "
                          "'timer'/'gate-ready' are OFFLINE/SMOKE synthetics")
    trg.add_argument("--cue-udp-port", type=int, default=47810,
                     help="--trigger cue: UDP port the target's GPS records "
                          "arrive on (scripts/field/cue_send.py)")
    trg.add_argument("--cue-window-s", type=float, default=None,
                     help="--trigger cue: seconds of records fitted (default: "
                          "flight.cue_relay.DEFAULT_CAPTURE_DURATION_S, 5.5)")
    trg.add_argument("--cue-min-span-s", type=float, default=None,
                     help="--trigger cue: the fitted records must span at least "
                          "this long before GO can fire (default: 0.8 x the "
                          "window, 4.4 s)")
    trg.add_argument("--cue-max-staleness-s", type=float, default=1.0,
                     help="--trigger cue: the newest record must be this fresh "
                          "for GO to fire")
    trg.add_argument("--cue-max-gap-s", type=float, default=1.0,
                     help="--trigger cue: largest hole allowed inside the "
                          "fitted window")
    trg.add_argument("--cue-utc-source", choices=("vehicle", "system", "none"),
                     default="vehicle",
                     help="--trigger cue: this side's UTC clock, used to measure "
                          "each record's link delay. 'vehicle' = the Pixhawk's "
                          "GPS time (SYSTEM_TIME over MAVLink); 'system' = this "
                          "computer's clock, right only if it is disciplined "
                          "(desk/NTP); 'none' = no shared clock")
    trg.add_argument("--cue-allow-unsynced", action="store_true",
                     help="--trigger cue: when the shared clock is missing or "
                          "implausible, fall back to the minimum-delay anchor "
                          "instead of refusing GO (delay is then NOT measured)")
    trg.add_argument("--cue-no-enable", action="store_true",
                     help="--trigger cue: no RC enable switch; the cue fires GO "
                          "with no human veto after the gates. Refused by --fly.")
    trg.add_argument("--cue-max-range-m", type=float, default=None,
                     help="--trigger cue: hold GO while the cued target is "
                          "farther than this")
    trg.add_argument("--rc-channel", type=int, default=7,
                     help="RC_CHANNELS index of the GO switch (TODO-BUILDER: verify "
                          "against the PX4 RC map)")
    trg.add_argument("--rc-go-us", type=int, default=1700)
    trg.add_argument("--rc-low-us", type=int, default=1300)
    trg.add_argument("--go-after-s", type=float, default=8.0,
                     help="--trigger timer: synthesise the GO edge at this time")
    trg.add_argument("--arm-permit-channel", type=int, default=None,
                     help="--fly (required): RC_CHANNELS index of the ARM "
                          "PERMIT switch. The program arms and takes off only "
                          "after this switch goes LOW then HIGH while the GO "
                          "switch is LOW. Must differ from --rc-channel. "
                          "Uses the --rc-go-us / --rc-low-us thresholds.")
    trg.add_argument("--arm-permit-timeout-s", type=float, default=120.0,
                     help="--fly: give up (without arming) if no arm permit "
                          "arrives within this long")

    trm = ap.add_argument_group("terminal seeker (composed from seeker_loop)")
    trm.add_argument("--terminal", choices=("stock", "tag", "pursuit"),
                     default="pursuit",
                     help="which terminal law flies ENGAGE. DEFAULT = 'pursuit' "
                          "(ADR-0118, 2026-09-24: the ruled chase-only "
                          "engagement became the flying default when its "
                          "standing gate -- the Gazebo cross-check -- passed, "
                          "prereg4 median 0.343 m). 'stock' = SeekerGuidance "
                          "(LOS-rate pro-nav, the sprint-era default, kept "
                          "as-flown), 'tag' = flight.tag_terminal (3-D "
                          "predicted-intercept-point), 'pursuit' = flight."
                          "pursuit_terminal (ADR-0103 'chase only': GO enters "
                          "ENGAGE directly -- no coded dash -- and the receding-"
                          "range breakoff is suppressed; contact floor, lost-"
                          "target and engage-max-s stay active)")
    trm.add_argument("--pursuit-miss-range-m", type=float, default=3.0,
                     help="pursuit-mode MISS POLICY (builder ruling "
                          "2026-09-22): if the last KF range estimate at a "
                          "lost-target abort was inside this range, "
                          "transition ENGAGE -> SAFE DIRECTLY (reason "
                          "pursuit_miss) instead of BREAKOFF's climb-and-"
                          "search -- isim/specs/parity_flightcode_2026-09-21"
                          ".md's abort-timer A/B measured that a close-range "
                          "dropout is permanent, so lengthening the timer "
                          "buys nothing. Default matches "
                          "PursuitTerminalConfig.no_fallback_range_m. Inert "
                          "outside --terminal pursuit.")
    trm.add_argument("--pursuit-brake", action="store_true",
                     help="pursuit-mode BRAKE PACKAGE (builder ruling "
                          "2026-09-24, ADR-0115): stopping-distance cap on "
                          "the horizontal relative command (a=3 m/s^2, lead "
                          "0.45 s) + vertical arrival-sync. The registered "
                          "package, one switch, no tuning surface -- "
                          "isim/specs/brake_shaping_prereg_2026-09-24.md. "
                          "Default OFF = legacy behaviour, byte-identical.")
    trm.add_argument("--pursuit-approach-dash", action="store_true",
                     help="pursuit-mode LONG-RANGE APPROACH DASH (builder "
                          "directive 2026-10-21): for a cue far beyond the "
                          "camera envelope, Phase A flies the transit at up "
                          "to --pursuit-dash-vmax and sheds the speed on the "
                          "stopping-distance schedule into the believed "
                          "rendezvous (the Phase-A brake cap is applied "
                          "unconditionally while on). Belief + own-EKF "
                          "inputs only. Default OFF = legacy, byte-identical.")
    trm.add_argument("--pursuit-dash-vmax", type=float, default=20.0,
                     help="approach-dash transit speed ceiling, m/s (the "
                          "PLANT's own ceiling still applies -- PX4 "
                          "MPC_XY_VEL_MAX tops out at 20 in velocity-mode "
                          "OFFBOARD; the real airframe's top speed is an "
                          "unmeasured assumption)")
    trm.add_argument("--pursuit-dash-accel", type=float, default=8.0,
                     help="approach-dash Phase-A accel slew budget, m/s^2")
    trm.add_argument("--pursuit-stamped-age", action="store_true",
                     help="feed the pursuit KF each detection's TRUE pipeline "
                          "age (t - t_capture, FeedDetector sources only) "
                          "instead of the fixed meas_latency_s assumption -- "
                          "the jitter immunity measured in "
                          "runs/gz_latency_20261022 (LJ90)")
    trm.add_argument("--pursuit-meas-latency-s", type=float, default=None,
                     help="override PursuitTerminalConfig.meas_latency_s (the "
                          "KF's assumed measurement age; set it to the "
                          "MEASURED pipeline latency when the detector is "
                          "slower than the 45 ms default -- see "
                          "runs/gz_latency_20261022)")
    trm.add_argument("--pursuit-dash-brake-accel", type=float, default=None,
                     help="override brake_accel_ms2 for the approach-dash "
                          "deceleration schedule (default: whatever the "
                          "brake package / config already set)")
    trm.add_argument("--pursuit-arrival-search", choices=("off", "stern", "believed"),
                     default=None,
                     help="pursuit-mode ARRIVAL SEARCH (flight.pursuit_terminal "
                          "`arrival_search`, 2026-11-18): search the along-track "
                          "interval the target can be in instead of parking "
                          "behind where it is believed to be. 'stern' starts "
                          "deep astern and sweeps forward; 'believed' goes to "
                          "the believed point first. Needs ground speed of the "
                          "target's plus the sweep rate. Default off; --fly "
                          "takes FLY_PRESET ('believed') unless typed.")
    trm.add_argument("--pursuit-stale-decode-guard", choices=("on", "off"),
                     default=None,
                     help="pursuit-mode: prune the acquisition list on ticks "
                          "with no decode, so a dropout cannot restart the "
                          "camera phase from stale decodes "
                          "(flight.pursuit_terminal `stale_decode_guard`). "
                          "Default off; --fly takes FLY_PRESET (on) unless typed.")
    trm.add_argument("--pursuit-search-speed-sigma", type=float, default=None,
                     help="--pursuit-arrival-search: the DECLARED one-sigma "
                          "error of the believed target speed, as a fraction "
                          "(a given describing the cue; default 0.10)")
    trm.add_argument("--pursuit-rehearsal", action="store_true",
                     help="pursuit-mode REHEARSAL BREAK-OFF (builder "
                          "directive 2026-09-24): practice passes without "
                          "contact -- on a freshly-fed KF track whose "
                          "time-to-go is inside rehearsal_t_react_s (not "
                          "already inside rehearsal_t_late_s: too late, "
                          "logged, no break-off), climb and push "
                          "away from the target's path, then end the pass in "
                          "SAFE (reason rehearsal_breakoff, hovers like a "
                          "miss). The trigger instant, range and onboard "
                          "would-have ZEM land in the mission log. One "
                          "switch, config-default range -- "
                          "isim/specs/rehearsal_breakoff_prereg_2026-09-24.md. "
                          "Default OFF = legacy behaviour, byte-identical.")
    trm.add_argument("--intrinsics",
                     default=os.path.join(_REPO_ROOT, "configs/camera_intrinsics.json"))
    trm.add_argument("--n-pronav", type=float, default=5.0)
    trm.add_argument("--mount-fwd-m", type=float, default=0.10)
    trm.add_argument("--mount-left-m", type=float, default=0.0)
    trm.add_argument("--mount-up-m", type=float, default=0.05)
    trm.add_argument("--mount-tilt-deg", type=float, default=0.0)
    trm.add_argument("--target-span-m", type=float, default=None,
                     help="known-size span (m) the terminal converts a box width "
                          "into range with. NO nn_tier detector ships a range "
                          "calibration yet, so on real imagery the default is a "
                          "SIM-fitted constant and every range-keyed threshold "
                          "(freeze 3.5 m, breakoff arm 4.0 m, hard floor 0.5 m) "
                          "fires at the wrong true distance. TODO-BUILDER.")

    run = ap.add_argument_group("run")
    run.add_argument("--fps", type=float, default=20.0)
    run.add_argument("--smoke-duration", type=float, default=90.0,
                     help="outer wall bound on the setpoint stream (forces shutdown)")
    run.add_argument("--smoke-acquire-after-s", type=float, default=2.0,
                     help="--sitl-smoke: synthetic detections begin this many "
                          "seconds into the dash")
    run.add_argument("--log-csv", default=None,
                     help="per-tick CSV (default: auto-named under logs/)")
    run.add_argument("--record-frames", default=None, metavar="DIR",
                     help="onboard footage recorder (flight.deploy."
                          "frame_recorder.FrameRecorder): save raw camera "
                          "frames as reduced-rate/quality JPEGs to DIR during "
                          "the LIVE/--sitl-smoke run, for engagement evidence, "
                          "real-data retrain frames, and field debugging. "
                          "Default OFF (None) -- no import of the recorder "
                          "module happens at all unless this is set. Has no "
                          "effect on --dry-run (that path has no camera "
                          "frames to record).")
    return ap


def build_frame_recorder(record_frames_dir: Optional[str]):
    """Construct the onboard footage recorder for `--record-frames`, or
    return None unchanged. Kept as its own function so the default-off path
    is trivially provable byte-identical (no import of
    `flight.deploy.frame_recorder` happens at all when `record_frames_dir`
    is falsy) and so construction is unit-testable without a vehicle."""
    if not record_frames_dir:
        return None
    from flight.deploy.frame_recorder import FrameRecorder
    return FrameRecorder(record_frames_dir)


def build_source_detector(args, gcfg, cam):
    """--source: span -> source -> first-frame contract -> detector, composed
    from seeker_loop via flight.deploy.frame_feed. Returns (FeedDetector, None)
    or (None, refusal message). Sets gcfg.target_span_m to the detector's span
    so the terminal and the detector use one value."""
    from flight.deploy import frame_feed as ff
    from flight.deploy.seeker_loop import (CAMERA_FPS_DEFAULT, build_argparser,
                                           resolve_span, warn_uncalibrated_span)
    if args.detector == "tag":
        if args.target_span_m is None:
            return None, ("--detector tag needs --target-span-m (the tag's "
                          "measured black-square edge, m) -- no default")
        span, span_src = float(args.target_span_m), "--target-span-m"
    else:
        if args.weights is None:
            args.weights = build_argparser().get_default("weights")
        span, span_src, _ok = resolve_span(args.weights, gcfg, args.target_span_m)
        if warn_uncalibrated_span(span, span_src, args.weights, gcfg, live=True,
                                  require=args.require_span_calib):
            return None, "uncalibrated span (--require-span-calib)"
    gcfg.target_span_m = span
    print(f"[terminal] target span {span:.3f} m ({span_src}); detector "
          f"{args.detector}")
    fps = CAMERA_FPS_DEFAULT if args.camera_fps is None else args.camera_fps
    try:
        kind, source = ff.open_source(
            args.source, cam, fps, exposure_us=args.exposure_us, gain=args.gain,
            flip180=(getattr(args, "camera_mount", None) == "upside-down"))
    except (ff.RefusedSource, ValueError, OSError, RuntimeError) as e:
        return None, str(e)
    feed = ff.FrameFeed(source, threaded=(kind == "picamera")).start()
    fault = ff.preflight(feed, cam)
    if fault:
        feed.stop()
        return None, f"frame/intrinsics contract: {fault}"
    h, w = feed.first_frame().shape[:2]
    print(f"[source] {kind} first frame {w}x{h} matches the intrinsics "
          f"({'unstated' if cam.width is None else f'{cam.width}x{cam.height}'})")
    det = ff.FeedDetector(feed, ff.build_inner_detector(args, cam, span))
    return det, None


def run_desk_mode(args, cfg, guidance, detector) -> int:
    """--desk entry: no MAVSDK anywhere on this path (see run_desk)."""
    trigger = (ScriptedTrigger(go_at_s=args.go_after_s)
               if args.trigger == "timer" else GateReadyTrigger())
    sm = RealFlightSM(cfg, guidance=guidance, on_event=print)
    frame_recorder = build_frame_recorder(args.record_frames)
    print("[desk] NO vehicle link: nothing connects, nothing arms; setpoints "
          "are computed and logged, NOT sent. Own-state = FakeVehicle stand-in.")
    try:
        _sm, rows = run_desk(cfg, sm, trigger, detector, fps=args.fps,
                             max_s=args.mission_max_s,
                             acquire_after_s=args.smoke_acquire_after_s,
                             realtime=detector.feed.threaded,
                             csv_path=args.log_csv,
                             frame_recorder=frame_recorder)
    finally:
        detector.feed.stop()
    i_state, i_src = _CSV_FIELDS.index("state"), _CSV_FIELDS.index("sp_source")
    eng = [r.split(",") for r in rows]
    eng = [r for r in eng if r[i_state] == State.ENGAGE]
    n_guided = sum(1 for r in eng if r[i_src] == "guided")
    f = detector.feed
    print(f"[mission] states visited: {' -> '.join(sm.visited)}")
    print(f"[desk] frames: captured={f.n_captured} consumed={f.n_new} "
          f"stale_polls={f.n_stale} | detector runs={detector.n_detect} "
          f"hits={detector.n_hit} | ENGAGE ticks={len(eng)} "
          f"camera-guided={n_guided}")
    if detector.n_detect == 0:
        print("[desk] UNCERTAIN: zero frames reached the detector -- the "
              "plumbing was not exercised")
        return 1
    print("[desk] DONE: real frames reached the detector "
          f"({detector.n_detect}); target hits are reported above, not graded")
    return 0


def main(argv=None) -> int:
    ap = build_arg_parser()
    args = ap.parse_args(argv)

    if args.audit:
        return honesty_audit()
    if args.self_test:
        return self_test()

    if not (args.dry_run or args.sitl_smoke or args.desk or args.fly
            or args.bench_props_off or args.pilot_handover):
        ap.error("pick a mode: --self-test | --audit | --dry-run | --sitl-smoke "
                 "| --desk | --fly | --bench-props-off | --pilot-handover")
    if args.pilot_handover and (args.dry_run or args.sitl_smoke or args.desk
                                or args.fly or args.bench_props_off):
        ap.error("--pilot-handover is its own mode: drop the other mode flag")
    # The two modes that touch a REAL board are each their own mode.
    for flag, name in ((args.fly, "--fly"), (args.bench_props_off, "--bench-props-off")):
        if flag and (args.dry_run or args.sitl_smoke or args.desk
                     or (args.fly and args.bench_props_off)):
            ap.error(f"{name} is its own mode: drop the other mode flag")
    if args.desk:
        if args.dry_run or args.sitl_smoke:
            ap.error("--desk is its own mode (no --dry-run / --sitl-smoke)")
        if args.mavsdk_url:
            ap.error("--desk has no vehicle link: drop --mavsdk-url")
        if args.trigger in ("rc", "cue"):
            ap.error("--desk has no RC or vehicle link: use --trigger "
                     "gate-ready|timer")
        if not args.source:
            ap.error("--desk needs --source picamera | dir:PATH")
    if args.source and args.dry_run:
        ap.error("--source applies to --desk / --sitl-smoke, not --dry-run")
    if args.trigger == "cue":
        if args.dry_run:
            ap.error("--trigger cue needs a live mode (--sitl-smoke, "
                     "--bench-props-off or --fly): the cue is placed against "
                     "the vehicle's own GPS position")
        if args.terminal != "pursuit" and not (args.fly or args.bench_props_off
                                               or args.pilot_handover):
            ap.error("--trigger cue needs --terminal pursuit (the only terminal "
                     "that can take a belief latched at GO)")
    # A serial link is a real flight controller. The SITL smoke path arms and
    # takes off on its own with a synthetic trigger and a synthetic target, so
    # it may never be pointed at one (interface audit 2026-11-18, gap G1).
    if args.sitl_smoke and is_serial_url(args.mavsdk_url):
        ap.error(f"--sitl-smoke refuses {args.mavsdk_url}: that is a serial link "
                 f"to a real flight controller, and this mode would arm and "
                 f"take off with no human step. Use --fly (real flight) or "
                 f"--bench-props-off (rehearsal, cannot arm).")
    # Each real-board mode states ALL its refusals before anything is opened.
    if args.fly or args.bench_props_off:
        why = validate_fly_args(args) if args.fly else validate_bench_args(args)
        if why:
            tag = "fly" if args.fly else "bench"
            for w in why:
                print(f"[{tag}] REFUSED: {w}")
            return 2

    if args.pilot_handover:
        why = validate_handover_args(args)
        if why:
            for w in why:
                print(f"[handover] REFUSED: {w}")
            return 2

    cfg = build_config(args)
    if args.bench_props_off:
        cfg = bench_config(cfg)
    if args.pilot_handover:
        cfg = handover_config(cfg)
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
                          # The TERMINAL must hold the SAME trimmed reference as
                          # the coast fallback. While these disagreed, ENGAGE
                          # alternated between the seeker's setpoint and the coast
                          # setpoint tick by tick, so the commanded reference jumped
                          # by exactly the trim at up to 20 Hz -- a square wave on
                          # the vertical channel through the terminal, and the
                          # seeker pulled the vehicle back to the untrimmed altitude
                          # the moment it locked (review finding F4).
                          alt_ref_m=apply_alt_ref_trim(
                              cfg.dash_base_alt_m, cfg.dash_alt_trim_m, None))
    from flight.camera import CameraModel
    cam = (load_camera(args.intrinsics) if os.path.exists(args.intrinsics)
           else CameraModel(539.936, 539.936, 640.0, 480.0))
    # KNOWN-SIZE SPAN: range = fx*span/box_width_px, and span multiplies Vc into
    # the pro-nav command -- it is NOT a display constant. --dry-run/--sitl-smoke
    # use a SYNTHETIC detector built from this same value, so the default is
    # self-consistent there; a real camera mode must pass --target-span-m or ship
    # a fitted <weights>.calib.json (review2 HIGH). TODO-BUILDER.
    src_detector = None
    if args.source:
        # Real source: span, source, first-frame contract and detector are all
        # settled HERE, before build_terminal and before any link/arm.
        src_detector, refusal = build_source_detector(args, gcfg, cam)
        if refusal:
            print(f"[source] REFUSED: {refusal}")
            return 1
    elif args.target_span_m is not None:
        gcfg.target_span_m = float(args.target_span_m)
        print(f"[terminal] target span {gcfg.target_span_m:.3f} m "
              f"(--target-span-m)")
    else:
        print(f"[terminal] target span {gcfg.target_span_m:.3f} m (config "
              f"default -- NOT range-calibrated; synthetic-detector modes only)")
    guidance = build_terminal(args, cfg, gcfg, cam)

    ts = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    if args.log_csv is None:
        args.log_csv = os.path.join(_REPO_ROOT, "logs", f"real_flight_{ts}.csv")

    if args.dry_run:
        if args.record_frames:
            # --dry-run has no camera frames at all (run_offline drives the
            # state machine off a StubSeeker with `frame=None`) -- --record-
            # frames only applies to the live/--sitl-smoke path. Say so rather
            # than silently doing nothing.
            print(f"[frame_recorder] --record-frames {args.record_frames} has "
                  f"NO EFFECT on --dry-run (no camera frames exist on this "
                  f"path) -- use --sitl-smoke or the live vehicle.")
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

    if args.desk:
        return run_desk_mode(args, cfg, guidance, src_detector)

    # --sitl-smoke | --fly | --bench-props-off: the three live modes.
    if not args.mavsdk_url:
        ap.error("--sitl-smoke requires --mavsdk-url (e.g. udpin://0.0.0.0:14540)")
    if args.trigger == "rc":
        if args.sitl_smoke:
            print("[trigger] REAL RC_CHANNELS path selected -- SITL has no RC "
                  "input by default, so this will (correctly) never fire a GO.")
        trigger = RcChannelTrigger(args.rc_channel, args.rc_go_us, args.rc_low_us,
                                   cfg.link_timeout_s)
    elif args.trigger == "cue":
        trigger = build_cue_trigger(args, cfg)
    elif args.trigger == "timer":
        trigger = ScriptedTrigger(go_at_s=args.go_after_s)
    else:
        trigger = GateReadyTrigger()
    if args.pilot_handover:
        detector = None             # cue only: no camera, no synthetic target
    elif src_detector is not None:
        detector = src_detector
    else:
        # Unreachable under --fly: validate_fly_args refused a missing --source.
        from flight.deploy.seeker_loop import SmokeSeeker
        detector = SmokeSeeker(cam.fx, gcfg.target_span_m, cx=cam.cx, cy=cam.cy)
        if args.bench_props_off:
            print("[bench] no --source: the detector is SYNTHETIC (a fake "
                  "target). Add --source picamera to rehearse with the camera.")
    sm = RealFlightSM(cfg, guidance=guidance, on_event=print)
    frame_recorder = build_frame_recorder(args.record_frames)
    if frame_recorder is not None:
        print(f"[frame_recorder] recording raw frames to {args.record_frames} "
              f"(target_fps={frame_recorder.target_fps}, "
              f"jpeg_quality={frame_recorder.jpeg_quality})")
    import asyncio
    try:
        if args.pilot_handover:
            return asyncio.run(run_pilot_handover(args, cfg, sm, trigger))
        if args.bench_props_off:
            return asyncio.run(run_bench_props_off(args, cfg, sm, trigger, detector,
                                                   frame_recorder=frame_recorder))
        if args.fly:
            # The arm-permit switch: a second reader on the same RC_CHANNELS
            # stream, a different channel (validate_fly_args checked both).
            permit_trigger = RcChannelTrigger(args.arm_permit_channel, args.rc_go_us,
                                              args.rc_low_us, cfg.link_timeout_s)
            return asyncio.run(run_mavsdk_mission(args, cfg, sm, trigger, detector,
                                                  smoke=False,
                                                  frame_recorder=frame_recorder,
                                                  fly=True,
                                                  permit_trigger=permit_trigger))
        return asyncio.run(run_mavsdk_mission(args, cfg, sm, trigger, detector,
                                              smoke=True,
                                              frame_recorder=frame_recorder))
    finally:
        if isinstance(trigger, CueTrigger):
            # Idempotent: the latch already closed it on a GO.
            trigger.receiver.close()
            st = trigger.receiver.stats()
            print(f"[cue] link closed: {st.n_rx} records received, {st.n_bad} "
                  f"malformed, {st.n_after_latch} dropped after the latch; GO "
                  f"{'fired' if trigger.fired else 'never fired'} "
                  f"(last status: {trigger.status})")
        # LAST: the run and its teardown are over. Without this the process
        # never exits while a server is alive (see _OPENED_SYSTEMS).
        if end_opened_servers():
            print("[mavsdk] server process(es) ended; the vehicle link is free")


if __name__ == "__main__":
    raise SystemExit(main())
