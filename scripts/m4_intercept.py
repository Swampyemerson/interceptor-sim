#!/usr/bin/env python3
"""M4 gate script: intercept a MOVING AprilTag using ONLY the live camera as
target feedback, running either a pursuit or a proportional-navigation
(pro-nav) guidance law, and log the miss distance. See GOALS.md milestone
M4, scripts/m3_static_intercept.py (reused wholesale for setup/detection/
ground-truth plumbing), and scripts/m4_target_mover.py (the separate
process that drives the tag along a straight line).

THE STRAPDOWN-SEEKER PROBLEM (why yaw compensation is not optional here):
M3's camera boresight pointed at a STATIONARY target, so "how fast is the
tag drifting across the image" and "how fast is the true line of sight
(LOS) rotating in the world" were the same number. M4's camera is mounted
on a vehicle whose YAW ACTIVELY TRACKS the tag (same yaw-centering law as
M3), so the two are no longer the same thing at all: if the vehicle turns
in lockstep with the target, the tag can sit almost still in the image
(bearing beta roughly constant) while the true LOS is sweeping through the
sky just as fast as the vehicle is turning. A pro-nav law built naively off
d(beta)/dt would then see a near-zero rate and command almost no lateral
turn -- an inert, gutted pro-nav that looks like it's "running" but isn't
reacting to anything. The fix is to reconstruct the INERTIAL LOS azimuth,

    lambda = psi + beta

where psi is the vehicle's own yaw (from PX4's own EKF, MAVSDK
`attitude_euler()` -- this is the vehicle's OWN state, not target
information, so it is fair game under the "own-state from PX4, target from
the camera" split from ADR-0008/M3) and beta is the camera bearing (still
purely a camera measurement). lambda_dot -- the derivative of THIS
quantity, not of beta alone -- is the real LOS rate pro-nav needs. (psi and
beta share the same rotational-sign convention -- MAVSDK's yaw_deg is
"positive turning right/clockwise seen from above", and beta is "positive
when the tag is right of the nose" -- so adding them is dimensionally and
directionally sound; see the guidance-law section below and --bench, which
exists purely to prove this sign compensation is really working.)

ALPHA-BETA FILTER, IN TWO SENTENCES: lambda and range both come from a
noisy, occasionally-dropped-out camera measurement, so each is tracked with
a small constant-velocity (g-h / alpha-beta) filter that PREDICTS every
control tick (x_hat += xdot_hat * dt) and only CORRECTS -- nudging the
estimate and its rate toward the measurement -- on ticks where a genuinely
new detection arrived. This gives both a smoothed instantaneous value
(lambda_hat, R_hat) and, just as important for pro-nav, a rate estimate
(lambda_dot_hat) that keeps predicting sanely through a brief dropout
instead of going to zero or undefined the instant a frame is missed.

PURSUIT VS PRO-NAV -- SHARED EVERYTHING BUT THE LATERAL TERM: both laws use
the exact same closing-speed control, the exact same yaw-centering
sensor-pointing loop, the exact same altitude loop, the exact same filters,
and the exact same NED command path. The ONLY difference is the lateral
("perpendicular to LOS") velocity term: pursuit always holds it at zero
(aim straight up the LOS -- lag against a mover, by design, is the point of
running this baseline); pro-nav integrates `a_cmd = N * Vc * lambda_dot_hat`
into it. Sharing every other line of code is what makes the miss-distance
comparison between the two an honest one -- any difference in outcome
traces to that one term, not to some other incidental gain difference.

CAMERA-ONLY BREAKOFF, GROUND-TRUTH-ONLY MISS (ADR-0008 lineage): exactly
like M3, every control decision -- when to acquire, when to break off, what
to command -- is made from the camera's own measurements and the vehicle's
own EKF state. Ground truth (from `/world/apriltag/pose/info`, same
ADR-0006 transform chain) is sampled every tick purely to SCORE the run
(the miss-distance number in the gate) -- never read by the control law.
grep for "gt_" and you will only find it downstream of a command that was
already sent.

WHY NED VELOCITY + ABSOLUTE YAW SETPOINTS (changed from body-frame +
yawspeed mid-M4; ADR-0009 second addendum): the guidance law already
produces a world-frame velocity vector, PX4's body-frame velocity
tracking measurably degrades at speed, and commanding yaw as an ABSOLUTE
angle (psi + beta, the tag's azimuth) hands the pointing problem to
PX4's attitude loop -- no LOS-rate feedforward or yaw-rate tuning needed.
The camera is still the only target sensor.

TERMINAL-PHASE RULES, AND WHY: as range R -> 0, lambda_dot is mathematically
singular (a fixed absolute angular wobble corresponds to an ever-larger
apparent rate as you get close), so v_perp integration FREEZES inside
TERMINAL_FREEZE_RANGE_M -- we stop trusting lambda_dot near the end and just
coast on the last commanded lateral rate. Likewise, a brief loss of the tag
from the frame in the last few meters is EXPECTED, not a bug: pursuit in
particular can spin hard enough chasing a fast LOS rate that the tag
briefly leaves the field of view right as it matters most -- that is
precisely the failure mode this milestone exists to demonstrate, so instead
of instantly panicking to a hover, the terminal-range dropout rule holds
the last command for up to TERMINAL_HOLD_MAX_S and lets the geometry (and
the log) speak for itself.

Run manually (with PX4 SITL + gz_x500_mono_cam + the apriltag world already
running, and the tag pre-placed at the engagement start position -- see
scripts/check_m4.sh's launch block for the exact env and pre-placement
command):

    .venv/bin/python scripts/m4_intercept.py --law pronav
    .venv/bin/python scripts/m4_intercept.py --law pursuit
    .venv/bin/python scripts/m4_intercept.py --law pronav --bench

Normally this is invoked by scripts/check_m4.sh, which boots a fresh sim
per law, pre-places the tag, and runs both.
"""

import argparse
import asyncio
import csv
import math
import os
import queue
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Optional

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, VelocityNedYaw
from mavsdk.telemetry import LandedState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from m2_detect import (  # noqa: E402
    get_camera_intrinsics,
    PoseTracker,
    IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    POSE_TOPIC,
)
from m0_takeoff import (  # noqa: E402
    TelemetryState,
    track_position,
    track_flight_mode,
    track_armed,
    track_landed_state,
    wait_for_connection,
    wait_for_health,
    SYSTEM_ADDRESS,
)
from m3_static_intercept import (  # noqa: E402
    Measurement,
    LatestFrame,
    MeasurementHolder,
    detection_loop,
    sample_measurement,
    ground_truth_world_points,
    _clamp,
)

# --- Guidance targets / gains (GOALS.md M4; pro-nav mechanization + gain
# council-decided per CLAUDE.md -- do not retune without an ADR). See the
# module docstring for the guidance law itself. ---
# Camera sits ~0.25 m above the model origin (base_link ~0 m + mount offset,
# see ADR-0006) -> roughly ~0.75 m up; tag center is at 0.5 m, so a LOWER
# reference altitude than M3's 1.0 m keeps the vertical component small so
# it doesn't eat into the <1 m 3D miss-distance budget.
ALT_REF_M = 0.5
N_PRONAV = 4.0  # pro-nav gain (typical range 3-5, GOALS.md guidance arc)
ALPHA = 0.5  # alpha-beta filter position gain (both lambda and range channels)
# Rate gains split per channel after dev run ...T012819Z: the lambda
# channel's 0.15 rate gain lagged a building LOS rate by ~0.5 s (est -3
# deg/s while truth was -16), making the yaw feedforward useless exactly
# when it mattered. 0.30 halves that lag; range keeps the calmer 0.15
# (Vc doesn't need to be fast, it needs to be smooth).
BETA_GAIN_LAMBDA = 0.30  # alpha-beta rate gain, lambda (LOS) channel
BETA_GAIN_RANGE = 0.15  # alpha-beta rate gain, range channel
# Closing speed is CONSTANT during ENGAGE (changed after the first official
# gate run, m4_intercept_pronav_...T025458Z: the original v_close =
# 0.8*R_hat law -- inherited from M3's gentle standoff approach -- throttled
# the interceptor to ~1.1 m/s inside 1.5 m while the target cruised at
# 2.0 m/s, so the target simply outran it at CPA and the terminal LOS rate
# blew up as the symptom. An interceptor does not slow down at the target;
# rendezvous-style braking is a standoff behavior, not an intercept one.)
V_CLOSE_MAX = 3.0  # m/s, commanded along-LOS speed for the whole ENGAGE phase
# V_PERP_MAX raised 2.0 -> 3.0 after dev run ...T011913Z: the lead demand at
# the council geometry saturated a 2.0 m/s cap for the whole engagement,
# capping how fast pro-nav could null the LOS rate.
V_PERP_MAX = 3.0  # m/s, lateral (pro-nav) velocity clamp
V_TOTAL_MAX = 4.0  # m/s, horizontal command safety clamp after combining close+perp
# Yaw sensor-pointing loop: retuned after dev run ...T011913Z, where a pure
# P-loop (gain 1.5) lagged the rotating LOS by a growing margin (commanded
# -37 deg/s, LOS moving ~-25 deg/s, bearing diverged to -40 deg -> detection
# lost: off-axis + oblique board view kills the detector well inside the
# nominal +-50 deg half-FOV). ENGAGE now adds the filtered LOS rate as a
# FEEDFORWARD term (yawspeed = lambda_dot_hat + KYAW*beta) -- zero
# steady-state lag against a constantly rotating LOS, still built purely
# from camera + own-state. Both laws share it (fairness).
KYAW_DEG_PER_DEG = 3.0  # yaw-centering P-gain (sensor pointing)
YAWSPEED_MAX_DEG_S = 60.0  # raised from M3's 30 deg/s -- endgame LOS rates demand it
KP_ALT = 1.0  # 1/s
V_VERT_MAX = 0.5  # m/s
CONTROL_RATE_HZ = 20.0
MEAS_STALE_S = 0.4  # ignore (treat as no detection) measurements older than this
ACQUIRE_MIN_DETECTIONS = 10  # consecutive FRESH detections required to leave ACQUIRE
ACQUIRE_MIN_S = 1.0  # ... and at least this many seconds must also have elapsed
# ... and the tag must be CENTERED before engaging (added after dev run
# ...T012356Z, which engaged 1.02 s in -- mid-yaw-slew, 20 deg off-boresight,
# with the lambda filter still polluted by the initial turn; the engagement
# then started from a 20 deg hole it never climbed out of).
ACQUIRE_MAX_BEARING_DEG = 5.0
ACQUIRE_CENTERED_STREAK = 6  # ~0.4 s of consecutive centered detections
ACQUIRE_TIMEOUT_S = 20.0
ENGAGE_TIMEOUT_S = 30.0
TERMINAL_RANGE_M = 3.0  # inside this: a dropout holds the last command instead of hovering
TERMINAL_HOLD_MAX_S = 1.0  # max time to hold-last-command through a terminal dropout
# Terminal coast (raised 1.5 -> 2.0 and extended from freezing just v_perp to
# freezing the WHOLE commanded velocity vector, after official gate run
# ...T030007Z): as R->0 lambda_dot is singular -- by 1.5 m the estimate had
# already blown up to -53 deg/s (vs -13 at 2.0 m), whipping the commanded
# velocity DIRECTION and the yaw feedforward so fast that PX4's actual
# velocity collapsed to ~1.75 m/s sideways and the 2.0 m/s target pulled
# 1.0 m ahead by CPA (a pure tail-chase miss; cross-track was 0.10 m). A
# real interceptor's terminal phase does the same thing this fix does:
# stop chasing the singular LOS rate and fly the established collision
# course straight through the intercept.
TERMINAL_FREEZE_RANGE_M = 2.0  # inside this: coast -- hold the frozen velocity vector
BREAKOFF_ARM_RANGE_M = 2.5  # past-CPA breakoff logic arms only after R_hat dips below this
BREAKOFF_RANGE_INCREASES = 3  # consecutive FRESH detections w/ increasing range -> breakoff
BREAKOFF_HARD_FLOOR_M = 0.5  # measured range below this -> immediate breakoff
LOST_TAG_ABORT_S = 5.0  # continuous no-detection outside terminal range -> abort
CLIMB_S = 1.5  # breakoff climb duration
CLIMB_V = 1.0  # m/s up during breakoff climb
POST_BREAKOFF_LOG_S = 2.0  # total time spent (logging) in the BREAKOFF phase

# --- FPV profile (M4.5 sub-step S1, ADR-0010). Activated by --fpv; the
# module defaults above stay exactly at the M4-validated values so
# check_m4.sh is untouched. This profile OVERRIDES a bundle of constants
# for the higher-speed regime and is applied once in apply_fpv_profile()
# before the guidance loop runs. Design (ADR-0010 decision #2): DECOUPLE a
# fast run-in closing speed (to catch a ~6 m/s target) from a throttled
# terminal closing speed (to keep the terminal LOS-rate math at ~M4's
# validated ~3-4 m/s difficulty), and rescale every terminal RANGE so its
# TIME semantics survive the faster terminal speed. Numbers are the
# council's starting proposals -- validated/retuned by S1 dev runs, logged
# in ADR-0010 as they settle.
FPV = {
    # Two-speed closing law (see compute_v_close): fast run-in above the
    # throttle range, throttled terminal speed below it.
    "V_CLOSE_RUNIN": 9.0,      # m/s, closing speed while still far (catch the mover)
    "V_CLOSE_TERMINAL": 5.5,   # m/s, throttled closing speed in the terminal band
    "THROTTLE_RANGE_M": 5.0,   # r_hat at/below which closing throttles toward terminal
    # Lateral / total clamps scaled up for the faster regime.
    "V_PERP_MAX": 8.0,         # m/s, lateral (pro-nav) velocity clamp (S1 dev: 6 saturated, starving the Y-lead)
    "V_TOTAL_MAX": 13.0,       # m/s, horizontal command safety clamp
    # Terminal ranges rescaled ~1.7x (terminal speed 3.0 -> 5.5 m/s) so coast
    # DURATION and dropout-hold DURATION match M4's proven time semantics.
    "TERMINAL_RANGE_M": 5.0,
    "TERMINAL_FREEZE_RANGE_M": 3.5,
    "BREAKOFF_ARM_RANGE_M": 4.0,
    "VC_FLOOR_M_S": 4.5,       # range-rate floor (S1 dev: Vc pinned at 3.0 under-powered a=N*Vc*lam_dot; true closing ~4.5)
    # Range-rate filter must track a faster-swinging Rdot (the ADR-0009
    # lambda-lag pathology, now on the range channel per council seat A).
    "BETA_GAIN_RANGE": 0.45,
    # PX4 params set via MAVSDK before arming (ADR-0010 decision #3).
    "N_PRONAV": 5.0,          # lab trade study: N=5 minimized pure-PN miss (ADR-0011)
    "PX4_PARAMS": {
        "MPC_XY_VEL_MAX": 20.0,
        "MPC_ACC_HOR_MAX": 12.0,
        "MPC_TILTMAX_AIR": 60.0,
        "MPC_JERK_MAX": 30.0,
    },
}


def apply_fpv_profile():
    """Overwrite the module-level guidance constants with the FPV bundle
    (S1, ADR-0010). Called once from main() when --fpv is set, BEFORE the
    guidance loop reads any of them. Leaves the PX4 params for main() to
    push over MAVSDK (they aren't module constants)."""
    global V_PERP_MAX, V_TOTAL_MAX, TERMINAL_RANGE_M, TERMINAL_FREEZE_RANGE_M
    global BREAKOFF_ARM_RANGE_M, BETA_GAIN_RANGE, N_PRONAV
    V_PERP_MAX = FPV["V_PERP_MAX"]
    V_TOTAL_MAX = FPV["V_TOTAL_MAX"]
    TERMINAL_RANGE_M = FPV["TERMINAL_RANGE_M"]
    TERMINAL_FREEZE_RANGE_M = FPV["TERMINAL_FREEZE_RANGE_M"]
    BREAKOFF_ARM_RANGE_M = FPV["BREAKOFF_ARM_RANGE_M"]
    BETA_GAIN_RANGE = FPV["BETA_GAIN_RANGE"]
    N_PRONAV = FPV["N_PRONAV"]


def compute_v_close(r_hat, fpv_on):
    """Commanded along-LOS closing speed. M4 default: constant V_CLOSE_MAX.
    FPV (S1): fast run-in far out, linearly throttled to V_CLOSE_TERMINAL as
    r_hat falls through THROTTLE_RANGE_M down to TERMINAL_FREEZE_RANGE_M --
    catch the fast target, then slow the terminal geometry so lambda_dot
    doesn't blow up (ADR-0010 decision #2, seat A's detection-window warning)."""
    if not fpv_on:
        return V_CLOSE_MAX
    runin = FPV["V_CLOSE_RUNIN"]
    term = FPV["V_CLOSE_TERMINAL"]
    hi = FPV["THROTTLE_RANGE_M"]
    lo = FPV["TERMINAL_FREEZE_RANGE_M"]
    if r_hat is None or r_hat >= hi:
        return runin
    if r_hat <= lo:
        return term
    frac = (r_hat - lo) / (hi - lo)  # 1 at hi -> 0 at lo
    return term + frac * (runin - term)

# --- MAVSDK / takeoff constants (mirrors scripts/m3_static_intercept.py's
# own constants -- not reused via import, since M3 doesn't export these as
# module-level names meant for reuse; they're the same numbers by design). ---
CONNECT_TIMEOUT_S = 60
HEALTH_TIMEOUT_S = 90
ALTITUDE_SUCCESS_FRACTION = 0.8
ALTITUDE_TIMEOUT_S = 60
LAND_TIMEOUT_S = 90
LOG_POLL_HZ = 5.0
POSE_WAIT_TIMEOUT_S = 30.0
CAMERA_INFO_TIMEOUT_S = 30.0

# --- --bench mode constants (sign-convention validation; see run_bench()). ---
BENCH_WAVE_DEG_S = 20.0
BENCH_SEG_S = 2.0
BENCH_CYCLES = 2
BENCH_RECENTER_S = 3.0
BENCH_PASS_MEAN_DEG_S = 3.0

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")
MOVER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "m4_target_mover.py")

CSV_HEADER = [
    "t", "phase", "law", "detected",
    "meas_x", "meas_y", "meas_z", "meas_range", "bearing_rad",
    "psi_deg", "lambda_deg", "lambda_dot_deg_s",
    "r_hat_m", "rdot_hat_m_s", "vc_m_s", "a_cmd_m_s2", "v_perp_m_s",
    "cmd_vn", "cmd_ve", "cmd_vd", "cmd_yaw_deg",
    "alt_m",
    "gt_cam_x", "gt_cam_y", "gt_cam_z",
    "gt_tag_x", "gt_tag_y", "gt_tag_z", "gt_range",
]


def wrap_pi(angle: float) -> float:
    """Wrap an angle (radians) into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class AlphaBetaFilter:
    """Constant-velocity (g-h / alpha-beta) tracker for one scalar channel
    (either lambda, the inertial LOS azimuth, or R, the range). See the
    module docstring's "alpha-beta filter, in two sentences" for the idea.

    `predict()` is meant to be called every control tick; `correct()` only
    on ticks where a genuinely new (fresh) measurement is available. Set
    `angular=True` for the lambda channel: the RESIDUAL (measurement minus
    current estimate) is wrapped to [-pi, pi] before use, but x_hat itself
    is never wrapped -- that keeps lambda continuous (able to accumulate
    past +-180 deg as the vehicle spins) while still comparing correctly
    against a measurement that only makes sense mod 2*pi.
    """

    def __init__(self, alpha: float, beta: float, angular: bool = False):
        self.alpha = alpha
        self.beta = beta
        self.angular = angular
        self.x_hat: Optional[float] = None
        self.xdot_hat: float = 0.0
        self.last_innovation: float = 0.0
        self._last_correction_t: Optional[float] = None

    @property
    def initialized(self) -> bool:
        return self.x_hat is not None

    def predict(self, dt: float) -> None:
        if self.x_hat is None:
            return
        self.x_hat += self.xdot_hat * dt

    def correct(self, meas: float, t: float) -> None:
        if self.x_hat is None:
            # First measurement: initialize position, zero rate (spec).
            self.x_hat = meas
            self.xdot_hat = 0.0
            self.last_innovation = 0.0
            self._last_correction_t = t
            return
        residual = meas - self.x_hat
        if self.angular:
            residual = wrap_pi(residual)
        dt_since = max(1e-3, t - self._last_correction_t)
        self.x_hat += self.alpha * residual
        self.xdot_hat += self.beta * residual / dt_since
        self.last_innovation = residual
        self._last_correction_t = t


# --- Predicted Intercept Point (PIP) machinery (ADR-0011). Ported from the
# guidance_lab.py trade study, where PIP roughly halved the miss vs pure PN
# by aiming where the target WILL be, not where it is. Reuses AlphaBetaFilter
# on the target's ABSOLUTE north/east position (from own-EKF-position + the
# camera-measured relative vector -- target info still camera-only). ---
class TargetTracker:
    """Alpha-beta filters on the target's absolute (north, east) position,
    yielding a position+velocity estimate from a stream of possibly-noisy,
    intermittent absolute-position measurements. The velocity estimate is
    what PIP needs to solve the intercept triangle and lead the target."""

    def __init__(self, alpha=0.6, beta=0.2):
        self.fn = AlphaBetaFilter(alpha, beta)
        self.fe = AlphaBetaFilter(alpha, beta)

    def predict(self, dt):
        self.fn.predict(dt)
        self.fe.predict(dt)

    def correct(self, abs_ne, t):
        self.fn.correct(abs_ne[0], t)
        self.fe.correct(abs_ne[1], t)

    @property
    def initialized(self):
        return self.fn.x_hat is not None

    @property
    def pos_hat(self):
        if self.fn.x_hat is None:
            return None
        return (self.fn.x_hat, self.fe.x_hat)

    @property
    def vel_hat(self):
        if self.fn.x_hat is None:
            return None
        return (self.fn.xdot_hat, self.fe.xdot_hat)


def solve_intercept_time(rel, vt, v_close, max_lead_s):
    """Classic lead-pursuit / PIP intercept-triangle solve: smallest positive
    t such that |rel + vt*t| = v_close*t (target at `rel` relative to us,
    moving at constant `vt`; we close at constant speed `v_close`). Returns 0
    (aim at the target's current estimated position -- plain pursuit) when the
    target outruns v_close and diverges, or the quadratic degenerates.
    Ported verbatim from guidance_lab.py (ADR-0011)."""
    a = vt[0] * vt[0] + vt[1] * vt[1] - v_close * v_close
    b = 2.0 * (rel[0] * vt[0] + rel[1] * vt[1])
    c = rel[0] * rel[0] + rel[1] * rel[1]
    t_go = 0.0
    if abs(a) < 1e-9:
        if abs(b) > 1e-9:
            t = -c / b
            if t > 0:
                t_go = t
    else:
        disc = b * b - 4 * a * c
        if disc >= 0:
            sq = math.sqrt(disc)
            candidates = [tt for tt in ((-b + sq) / (2 * a), (-b - sq) / (2 * a)) if tt > 0]
            if candidates:
                t_go = min(candidates)
    return min(max(t_go, 0.0), max_lead_s)


# PIP tuning (ADR-0011 / guidance_lab.py winner: V_CLOSE=6 minimized miss).
PIP_TRACK_ALPHA = 0.6
PIP_TRACK_BETA = 0.2
PIP_MAX_LEAD_S = 3.0


class M4TelemetryState(TelemetryState):
    """TelemetryState plus the vehicle's own yaw (to reconstruct the inertial
    LOS azimuth lambda = psi + beta) and its own north/east position (for
    PIP's absolute target track). Both are OWN state (PX4 EKF), not target
    info -- the camera stays the only target sensor."""

    def __init__(self):
        super().__init__()
        self.yaw_deg: Optional[float] = None
        self.pos_n: Optional[float] = None
        self.pos_e: Optional[float] = None


async def track_attitude(drone, state: "M4TelemetryState") -> None:
    async for euler in drone.telemetry.attitude_euler():
        state.yaw_deg = euler.yaw_deg


async def track_local_position(drone, state: "M4TelemetryState") -> None:
    async for pv in drone.telemetry.position_velocity_ned():
        state.pos_n = pv.position.north_m
        state.pos_e = pv.position.east_m


def acquire_command(detected: bool, meas: "Optional[Measurement]", alt_m, psi_deg):
    """ACQUIRE-phase command: hover + point the nose at the tag, NO closing
    (the tag is still stationary at this point -- see check_m4.sh, which
    pre-places it before this script even starts). Commands are NED velocity
    + ABSOLUTE yaw angle (see the "why NED" note in the module docstring):
    when the tag is visible, yaw setpoint = psi + beta (the tag's absolute
    azimuth); when it isn't, hold the current heading."""
    v_down = (
        _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
        if alt_m is not None else 0.0
    )
    psi = psi_deg if psi_deg is not None else 0.0
    if not detected:
        return 0.0, 0.0, v_down, psi
    yaw_deg = psi + math.degrees(meas.bearing_rad)
    return 0.0, 0.0, v_down, yaw_deg


def write_row_m4(
    writer, log_file, t: float, phase: str, law: str,
    detected: Optional[bool], meas: "Optional[Measurement]",
    psi_deg, lambda_rad, lambda_dot_rad_s, r_hat_m, rdot_hat_m_s,
    vc_m_s, a_cmd_m_s2, v_perp_m_s, cmd, alt_m, gt_cam, gt_tag, gt_range,
):
    def fmt(value, spec="{:.4f}"):
        return "" if value is None else spec.format(value)

    meas_xyz = meas.meas_xyz if (meas is not None and detected) else None
    meas_range = meas.range_m if (meas is not None and detected) else None
    bearing_rad = meas.bearing_rad if (meas is not None and detected) else None

    row = [
        f"{t:.3f}",
        phase,
        law,
        "" if detected is None else int(detected),
        fmt(meas_xyz[0]) if meas_xyz is not None else "",
        fmt(meas_xyz[1]) if meas_xyz is not None else "",
        fmt(meas_xyz[2]) if meas_xyz is not None else "",
        fmt(meas_range),
        fmt(bearing_rad, "{:.5f}"),
        fmt(psi_deg, "{:.3f}"),
        fmt(math.degrees(lambda_rad), "{:.3f}") if lambda_rad is not None else "",
        fmt(math.degrees(lambda_dot_rad_s), "{:.3f}") if lambda_dot_rad_s is not None else "",
        fmt(r_hat_m),
        fmt(rdot_hat_m_s),
        fmt(vc_m_s),
        fmt(a_cmd_m_s2),
        fmt(v_perp_m_s),
        fmt(cmd[0]) if cmd is not None else "",
        fmt(cmd[1]) if cmd is not None else "",
        fmt(cmd[2]) if cmd is not None else "",
        fmt(cmd[3]) if cmd is not None else "",
        fmt(alt_m),
        fmt(gt_cam[0]) if gt_cam is not None else "",
        fmt(gt_cam[1]) if gt_cam is not None else "",
        fmt(gt_cam[2]) if gt_cam is not None else "",
        fmt(gt_tag[0]) if gt_tag is not None else "",
        fmt(gt_tag[1]) if gt_tag is not None else "",
        fmt(gt_tag[2]) if gt_tag is not None else "",
        fmt(gt_range),
    ]
    writer.writerow(row)
    log_file.flush()


def recompute_min_gt_range_from_csv(log_path: str) -> float:
    """Independent recomputation of the miss distance directly from the
    just-written CSV (CLAUDE.md: "analyze from logs"). Should match the
    running-min tracked during flight -- see the cross-check in main()."""
    min_val = None
    with open(log_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            value = row.get("gt_range", "")
            if value:
                v = float(value)
                if min_val is None or v < min_val:
                    min_val = v
    return min_val if min_val is not None else float("nan")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--law", choices=["pursuit", "pronav", "pip"], required=True,
        help="guidance law to run (pip = predicted intercept point, ADR-0011)",
    )
    parser.add_argument(
        "--target-start", default="6.5,-4,0.5",
        help="tag start position 'x,y,z' (m) -- forwarded to m4_target_mover.py",
    )
    parser.add_argument(
        "--target-vel", default="0,2.0",
        help="tag velocity 'vx,vy' (m/s) -- forwarded to m4_target_mover.py",
    )
    parser.add_argument(
        "--require-miss", type=float, default=None,
        help="exit nonzero if the final miss distance is >= this (meters)",
    )
    parser.add_argument(
        "--bench", action="store_true",
        help="run the sign-convention bench check instead of an intercept",
    )
    parser.add_argument(
        "--mover-duration", type=float, default=12.0,
        help="how long the target mover streams motion for, in seconds",
    )
    parser.add_argument(
        "--fpv", action="store_true",
        help="FPV profile (S1, ADR-0010): bump PX4 params, two-speed closing "
             "law, rescaled terminal ranges for a faster (~6 m/s) target",
    )
    return parser.parse_args()


async def run_acquire_and_engage(drone, state, meas_holder, tracker, writer, log_file, started, args):
    """ACQUIRE (hover + yaw-center on the static tag) -> ENGAGE (spawn the
    mover, run the chosen guidance law) -> BREAKOFF (climb off, keep
    logging) or an abort. See the module docstring for the guidance law
    and the CLI spec for the exact phase/trigger semantics.

    Returns a dict of results consumed by main() to print the summary and
    decide the process exit code.
    """
    dt = 1.0 / CONTROL_RATE_HZ

    lambda_filter = AlphaBetaFilter(ALPHA, BETA_GAIN_LAMBDA, angular=True)
    range_filter = AlphaBetaFilter(ALPHA, BETA_GAIN_RANGE, angular=False)
    # PIP (ADR-0011): absolute target position+velocity track for the lead
    # solve. Only used when args.law == "pip"; predicts every tick, corrects
    # on fresh detections with own_ned + camera relative vector.
    target_tracker = TargetTracker(PIP_TRACK_ALPHA, PIP_TRACK_BETA)

    phase = "ACQUIRE"
    acquire_start_mono = time.monotonic()
    consecutive_fresh = 0
    centered_streak = 0
    last_meas_t_mono = None

    v_perp = 0.0
    vh0 = vh1 = 0.0
    frozen_vworld = None
    last_cmd = (0.0, 0.0, 0.0, 0.0)
    mover_proc = None
    engage_t0 = None

    breakoff_reason = ""
    breakoff_entry_mono = None
    breakoff_armed = False
    range_increase_streak = 0
    last_fresh_range = None
    dropout_start_mono = None
    lost_since_mono = None

    aborted = False
    abort_reason = ""
    n_ticks = 0
    n_detected_ticks = 0
    min_gt_range_running = None

    while True:
        tick_start = time.monotonic()
        detected, meas = sample_measurement(meas_holder)
        # new_meas: the detection thread produced a NEW result since our last
        # tick (whether or not it contains a detection). fresh: that new
        # result IS a detection (usable for filter correction). The two are
        # distinct on purpose: detection takes ~70 ms/frame here (measured
        # 2026-07-05, dev run m4_intercept_pronav_...T011633Z), i.e. ~14 Hz
        # against this 20 Hz loop, so "no new result this tick" is NORMAL
        # cadence, not a miss -- only a new result that failed to detect
        # counts against acquisition.
        new_meas = last_meas_t_mono is None or meas.t_mono != last_meas_t_mono
        fresh = detected and new_meas
        if new_meas:
            last_meas_t_mono = meas.t_mono

        psi_deg = state.yaw_deg
        psi_rad = math.radians(psi_deg) if psi_deg is not None else 0.0
        alt_m = state.relative_altitude_m

        # --- filters: predict every tick, correct only on FRESH ticks ---
        lambda_filter.predict(dt)
        range_filter.predict(dt)
        target_tracker.predict(dt)
        if fresh:
            lambda_meas = psi_rad + meas.bearing_rad
            lambda_filter.correct(lambda_meas, tick_start)
            range_filter.correct(meas.range_m, tick_start)
            # PIP absolute target track (ADR-0011): own NED position (EKF,
            # own-state) + the camera relative vector in NED. The relative
            # vector uses the SAME bench-validated LOS azimuth psi+beta the
            # pronav command frame uses: north = range*cos(lambda),
            # east = range*sin(lambda). Target info stays camera-only.
            if state.pos_n is not None and state.pos_e is not None:
                rel_n = meas.range_m * math.cos(lambda_meas)
                rel_e = meas.range_m * math.sin(lambda_meas)
                target_tracker.correct(
                    (state.pos_n + rel_n, state.pos_e + rel_e), tick_start
                )

        lambda_hat = lambda_filter.x_hat
        lambda_dot_hat = lambda_filter.xdot_hat if lambda_filter.initialized else None
        r_hat = range_filter.x_hat
        rdot_hat = range_filter.xdot_hat if range_filter.initialized else None
        # Vc floor raised 0.3 -> 1.5 after dev run ...T012356Z: the range-rate
        # filter starts at ~0 from hover (and lags under ~50% detection
        # coverage), so a 0.3 m/s floor let a_cmd = N*Vc*lambda_dot build the
        # lead at one-fifth strength for the first ~3 s -- the target's LOS
        # walked out of the FOV before the lead caught up. 1.5 m/s is a
        # defensible prior: the closing law commands a constant V_CLOSE_MAX
        # from engage onward, so true closing speed is never near zero once
        # engaged. Measured -Rdot_hat takes over the
        # moment it exceeds the floor.
        VC_FLOOR_M_S = FPV["VC_FLOOR_M_S"] if args.fpv else 1.5
        vc = max(VC_FLOOR_M_S, -rdot_hat) if rdot_hat is not None else VC_FLOOR_M_S
        a_cmd = (
            N_PRONAV * vc * lambda_dot_hat
            if lambda_dot_hat is not None else 0.0
        )

        if r_hat is not None and r_hat < BREAKOFF_ARM_RANGE_M:
            breakoff_armed = True

        if phase == "ACQUIRE":
            cmd = acquire_command(detected, meas, alt_m, psi_deg)
            # Streak counts consecutive DETECTIONS in the detection stream:
            # a tick with no new detector result leaves it untouched (see
            # new_meas comment above); only a new result WITHOUT a tag
            # resets it. (First version counted per control tick and could
            # never reach 10 -- the 20 Hz loop outpaces ~14 Hz detection.)
            if new_meas:
                consecutive_fresh = consecutive_fresh + 1 if meas.range_m is not None else 0
            acquire_elapsed = tick_start - acquire_start_mono

            # Centered must be SETTLED (a streak of fresh detections), not a
            # momentary zero-crossing mid-slew: dev run ...T012819Z engaged
            # with beta ~4 deg but ~+16 deg/s of residual yaw rate, and the
            # engagement started in a hole. A stable streak of centered
            # detections implies both small beta AND ~zero yaw rate (static
            # tag: beta steady <=> nose steady on the LOS).
            if fresh:
                if abs(math.degrees(meas.bearing_rad)) <= ACQUIRE_MAX_BEARING_DEG:
                    centered_streak += 1
                else:
                    centered_streak = 0
            if (
                consecutive_fresh >= ACQUIRE_MIN_DETECTIONS
                and acquire_elapsed >= ACQUIRE_MIN_S
                and centered_streak >= ACQUIRE_CENTERED_STREAK
            ):
                print(
                    f"[m4] Acquired at t={acquire_elapsed:.2f}s "
                    f"({consecutive_fresh} consecutive fresh detections). "
                    "Spawning mover and engaging..."
                )
                phase = "ENGAGE"
                engage_t0 = tick_start
                mover_args = [
                    sys.executable, MOVER_SCRIPT,
                    "--start", args.target_start,
                    "--vel", args.target_vel,
                    "--duration", str(args.mover_duration),
                ]
                print(f"[m4] mover: {' '.join(mover_args)}")
                mover_proc = subprocess.Popen(mover_args, stdout=None, stderr=None)
            elif acquire_elapsed > ACQUIRE_TIMEOUT_S:
                aborted = True
                abort_reason = f"failed to acquire tag within {ACQUIRE_TIMEOUT_S}s"

        elif phase == "ENGAGE":
            engage_elapsed = tick_start - engage_t0

            if detected:
                in_terminal_coast = (
                    frozen_vworld is not None
                    or (r_hat is not None and r_hat < TERMINAL_FREEZE_RANGE_M)
                )
                if not in_terminal_coast:
                    v_close = compute_v_close(r_hat, args.fpv)  # two-speed under --fpv

                    if args.law == "pip":
                        # Predicted Intercept Point (ADR-0011): aim the whole
                        # closing velocity at the LEAD point where we and the
                        # target's estimated constant-velocity track would
                        # meet -- the direct cure for arriving where the target
                        # WAS. Falls back to aiming at the current estimate
                        # (pursuit) when the tracker isn't ready or the target
                        # outruns v_close (solve_intercept_time -> 0).
                        tpos = target_tracker.pos_hat
                        if tpos is not None and state.pos_n is not None:
                            own = (state.pos_n, state.pos_e)
                            rel = (tpos[0] - own[0], tpos[1] - own[1])
                            vt = target_tracker.vel_hat or (0.0, 0.0)
                            t_go = solve_intercept_time(rel, vt, v_close, PIP_MAX_LEAD_S)
                            aim = (tpos[0] + vt[0] * t_go, tpos[1] + vt[1] * t_go)
                            dirn = (aim[0] - own[0], aim[1] - own[1])
                            dn = math.hypot(dirn[0], dirn[1])
                            if dn > 1e-6:
                                vh0 = v_close * dirn[0] / dn
                                vh1 = v_close * dirn[1] / dn
                            else:
                                vh0, vh1 = last_cmd[0], last_cmd[1]
                        else:
                            # Tracker not ready -> aim up the LOS at v_close.
                            vh0 = v_close * math.cos(lambda_hat)
                            vh1 = v_close * math.sin(lambda_hat)
                        v_perp = 0.0  # not used by PIP; kept for the CSV column
                    else:
                        u = (math.cos(lambda_hat), math.sin(lambda_hat))
                        p = (-math.sin(lambda_hat), math.cos(lambda_hat))
                        if args.law == "pronav":
                            v_perp = _clamp(v_perp + a_cmd * dt, -V_PERP_MAX, V_PERP_MAX)
                        else:
                            v_perp = 0.0  # pursuit: no lateral term, ever.
                        vh0 = v_close * u[0] + v_perp * p[0]
                        vh1 = v_close * u[1] + v_perp * p[1]

                    norm = math.hypot(vh0, vh1)
                    if norm > V_TOTAL_MAX:
                        scale = V_TOTAL_MAX / norm
                        vh0 *= scale
                        vh1 *= scale
                else:
                    # TERMINAL COAST (see TERMINAL_FREEZE_RANGE_M comment):
                    # lock the collision-course velocity vector on entry and
                    # fly it straight through CPA -- lambda_dot is singular
                    # here and chasing it un-flies the intercept.
                    if frozen_vworld is None:
                        frozen_vworld = (vh0, vh1)
                        print(
                            f"[m4] Terminal coast: velocity vector frozen at "
                            f"r_hat={r_hat:.2f} m (({vh0:.2f}, {vh1:.2f}) world)"
                        )
                    vh0, vh1 = frozen_vworld

                v_down = (
                    _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
                    if alt_m is not None else 0.0
                )
                # Absolute yaw setpoint at the tag's azimuth (psi + beta) --
                # PX4's attitude controller does the slewing; no rate loop or
                # LOS-rate feedforward needed with yaw-angle setpoints.
                yaw_deg = psi_deg + math.degrees(meas.bearing_rad) if psi_deg is not None else 0.0
                cmd = (vh0, vh1, v_down, yaw_deg)
                last_cmd = cmd
                dropout_start_mono = None
                lost_since_mono = None

                if fresh:
                    if meas.range_m < BREAKOFF_HARD_FLOOR_M:
                        breakoff_reason = (
                            f"measured range {meas.range_m:.2f} m < hard floor "
                            f"{BREAKOFF_HARD_FLOOR_M} m"
                        )
                    if last_fresh_range is not None and meas.range_m > last_fresh_range:
                        range_increase_streak += 1
                    else:
                        range_increase_streak = 0
                    last_fresh_range = meas.range_m
                    if (
                        not breakoff_reason
                        and breakoff_armed
                        and range_increase_streak >= BREAKOFF_RANGE_INCREASES
                    ):
                        breakoff_reason = (
                            f"measured range increased for {range_increase_streak} "
                            "consecutive fresh detections (past closest approach)"
                        )
            else:
                # Dropout: terminal-range holds the last command through a
                # brief loss (endgame FOV loss is EXPECTED, see module
                # docstring); far from the target, a dropout means we
                # genuinely lost the target and should just hover/abort.
                if r_hat is not None and r_hat < TERMINAL_RANGE_M:
                    if dropout_start_mono is None:
                        dropout_start_mono = tick_start
                    cmd = last_cmd
                    if tick_start - dropout_start_mono > TERMINAL_HOLD_MAX_S:
                        breakoff_reason = (
                            f"lost detection for >{TERMINAL_HOLD_MAX_S}s inside "
                            f"terminal range ({TERMINAL_RANGE_M} m)"
                        )
                else:
                    cmd = (0.0, 0.0, 0.0, psi_deg if psi_deg is not None else 0.0)
                    last_cmd = cmd
                    if lost_since_mono is None:
                        lost_since_mono = tick_start
                    if tick_start - lost_since_mono > LOST_TAG_ABORT_S:
                        aborted = True
                        abort_reason = (
                            f"lost tag for more than {LOST_TAG_ABORT_S}s (far from target)"
                        )

            if not breakoff_reason and engage_elapsed > ENGAGE_TIMEOUT_S:
                breakoff_reason = f"engage phase exceeded {ENGAGE_TIMEOUT_S}s timeout"

            if breakoff_reason:
                print(f"[m4] BREAKOFF: {breakoff_reason}")
                phase = "BREAKOFF"
                breakoff_entry_mono = tick_start

        else:  # phase == "BREAKOFF"
            breakoff_elapsed = tick_start - breakoff_entry_mono
            hold_yaw = last_cmd[3]
            if breakoff_elapsed < CLIMB_S:
                cmd = (0.0, 0.0, -CLIMB_V, hold_yaw)
            else:
                cmd = (0.0, 0.0, 0.0, hold_yaw)

        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(*cmd))
        except OffboardError as error:
            aborted = True
            abort_reason = f"set_velocity_ned failed: {error}"

        gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
        if gt_range is not None and (min_gt_range_running is None or gt_range < min_gt_range_running):
            min_gt_range_running = gt_range

        n_ticks += 1
        if detected:
            n_detected_ticks += 1

        write_row_m4(
            writer, log_file, time.monotonic() - started, phase, args.law,
            detected, meas, psi_deg, lambda_hat, lambda_dot_hat, r_hat, rdot_hat,
            vc, a_cmd, v_perp, cmd, alt_m, gt_cam, gt_tag, gt_range,
        )

        if aborted:
            print(f"[m4] ABORT: {abort_reason}")
            break
        if phase == "BREAKOFF" and (tick_start - breakoff_entry_mono) >= POST_BREAKOFF_LOG_S:
            break

        elapsed = time.monotonic() - tick_start
        await asyncio.sleep(max(0.0, dt - elapsed))

    return {
        "aborted": aborted,
        "abort_reason": abort_reason,
        "breakoff_reason": breakoff_reason,
        "n_ticks": n_ticks,
        "n_detected_ticks": n_detected_ticks,
        "min_gt_range_running": min_gt_range_running,
        "mover_proc": mover_proc,
    }


async def run_bench(drone, state, meas_holder, tracker, writer, log_file, started, args):
    """--bench: prove the lambda = psi + beta compensation actually works.
    Spins the vehicle in place (yawspeed square wave) over a STATIC tag
    (no mover) while running the lambda filter normally; if the
    compensation is correct, lambda_dot_hat should stay near zero (the tag
    genuinely isn't moving) even though the raw bearing is sweeping hard as
    the vehicle spins under it. See the module docstring's "strapdown
    seeker" section for why this is exactly the failure mode a naive
    d(bearing)/dt would fall into.
    """
    dt = 1.0 / CONTROL_RATE_HZ
    lambda_filter = AlphaBetaFilter(ALPHA, BETA_GAIN_LAMBDA, angular=True)

    wave_total_s = BENCH_SEG_S * 2 * BENCH_CYCLES
    lambda_dot_samples = []
    last_meas_t_mono = None
    phase = "BENCH_WAVE"
    bench_start = time.monotonic()
    recenter_start = None

    while True:
        tick_start = time.monotonic()
        detected, meas = sample_measurement(meas_holder)
        fresh = detected and (last_meas_t_mono is None or meas.t_mono != last_meas_t_mono)
        if fresh:
            last_meas_t_mono = meas.t_mono

        psi_deg = state.yaw_deg
        psi_rad = math.radians(psi_deg) if psi_deg is not None else 0.0
        alt_m = state.relative_altitude_m

        lambda_filter.predict(dt)
        if fresh:
            lambda_filter.correct(psi_rad + meas.bearing_rad, tick_start)

        v_down = (
            _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
            if alt_m is not None else 0.0
        )

        elapsed = tick_start - bench_start
        if phase == "BENCH_WAVE":
            if elapsed >= wave_total_s:
                phase = "BENCH_RECENTER"
                recenter_start = tick_start
                yawspeed_cmd = 0.0
                cmd = (0.0, 0.0, v_down, yawspeed_cmd)
            else:
                cycle_pos = elapsed % (2 * BENCH_SEG_S)
                yawspeed_cmd = BENCH_WAVE_DEG_S if cycle_pos < BENCH_SEG_S else -BENCH_WAVE_DEG_S
                cmd = (0.0, 0.0, v_down, yawspeed_cmd)
                if fresh and lambda_filter.initialized:
                    lambda_dot_samples.append(abs(math.degrees(lambda_filter.xdot_hat)))
        else:  # BENCH_RECENTER
            recenter_elapsed = tick_start - recenter_start
            if recenter_elapsed >= BENCH_RECENTER_S:
                gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
                write_row_m4(
                    writer, log_file, time.monotonic() - started, phase, args.law,
                    detected, meas, psi_deg, lambda_filter.x_hat, lambda_filter.xdot_hat,
                    None, None, None, None, None, (0.0, 0.0, 0.0, 0.0), alt_m,
                    gt_cam, gt_tag, gt_range,
                )
                break
            if detected:
                yawspeed_cmd = _clamp(
                    KYAW_DEG_PER_DEG * math.degrees(meas.bearing_rad),
                    -YAWSPEED_MAX_DEG_S, YAWSPEED_MAX_DEG_S,
                )
            else:
                yawspeed_cmd = 0.0
            cmd = (0.0, 0.0, v_down, yawspeed_cmd)

        try:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(*cmd))
        except OffboardError as error:
            print(f"[m4] BENCH FAILED: set_velocity_body error: {error}")
            break

        gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
        write_row_m4(
            writer, log_file, time.monotonic() - started, phase, args.law,
            detected, meas, psi_deg, lambda_filter.x_hat, lambda_filter.xdot_hat,
            None, None, None, None, None, cmd, alt_m, gt_cam, gt_tag, gt_range,
        )

        tick_elapsed = time.monotonic() - tick_start
        await asyncio.sleep(max(0.0, dt - tick_elapsed))

    mean_val = sum(lambda_dot_samples) / len(lambda_dot_samples) if lambda_dot_samples else float("nan")
    max_val = max(lambda_dot_samples) if lambda_dot_samples else float("nan")
    print(f"BENCH mean_abs_lambda_dot={mean_val:.3f} max={max_val:.3f}")
    passed = bool(lambda_dot_samples) and mean_val < BENCH_PASS_MEAN_DEG_S
    return passed, mean_val, max_val


async def main():
    started = time.monotonic()
    args = parse_args()

    if args.fpv:
        apply_fpv_profile()
        print(
            "[m4] FPV profile ON (S1, ADR-0010): two-speed closing "
            f"(run-in {FPV['V_CLOSE_RUNIN']}, terminal {FPV['V_CLOSE_TERMINAL']} m/s), "
            f"terminal-freeze {TERMINAL_FREEZE_RANGE_M} m, V_PERP_MAX {V_PERP_MAX}, "
            f"BETA_GAIN_RANGE {BETA_GAIN_RANGE}"
        )

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    suffix = "bench" if args.bench else args.law
    log_path = os.path.join(LOGS_DIR, f"m4_intercept_{suffix}_{timestamp}.csv")

    node = Node()

    print(f"[m4] Reading camera intrinsics from {CAMERA_INFO_TOPIC} ...")
    try:
        fx, fy, cx, cy, width, height = get_camera_intrinsics(node, CAMERA_INFO_TIMEOUT_S)
    except (RuntimeError, queue.Empty) as exc:
        print(f"[m4] FAILED: could not get camera intrinsics: {exc}")
        return 1
    print(
        f"[m4] Intrinsics: fx={fx:.3f} fy={fy:.3f} cx={cx:.1f} cy={cy:.1f} "
        f"({width}x{height})"
    )

    tracker = PoseTracker()
    pose_sub = node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v)
    if not pose_sub:
        print(f"[m4] FAILED: could not subscribe to {POSE_TOPIC}")
        return 1

    frame_holder = LatestFrame()
    image_sub = node.subscribe(Image, IMAGE_TOPIC, frame_holder.on_image)
    if not image_sub:
        print(f"[m4] FAILED: could not subscribe to {IMAGE_TOPIC}")
        node.unsubscribe(POSE_TOPIC)
        return 1

    print(f"[m4] Subscribed to {IMAGE_TOPIC} and {POSE_TOPIC}")
    print(f"[m4] Logging to {log_path}")
    print("[m4] Waiting for the pose topic to populate...")
    deadline = time.monotonic() + POSE_WAIT_TIMEOUT_S
    while not tracker.latest and time.monotonic() < deadline:
        time.sleep(0.1)
    if not tracker.latest:
        print(f"[m4] FAILED: no pose data received within {POSE_WAIT_TIMEOUT_S}s")
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        return 1

    meas_holder = MeasurementHolder()
    stop_event = threading.Event()
    detector_thread = threading.Thread(
        target=detection_loop,
        args=(frame_holder, meas_holder, fx, fy, cx, cy, stop_event),
        kwargs={"detector_kwargs": {"quad_decimate": 2.0}},
        daemon=True,
    )
    detector_thread.start()
    print("[m4] Detection thread started.")

    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(CSV_HEADER)
    log_file.flush()

    drone = System()
    state = M4TelemetryState()
    tracker_tasks = []

    result_code = 1
    mover_proc = None

    try:
        print(f"[m4] Connecting to {SYSTEM_ADDRESS} (timeout {CONNECT_TIMEOUT_S}s)...")
        await drone.connect(system_address=SYSTEM_ADDRESS)
        await wait_for_connection(drone, CONNECT_TIMEOUT_S)
        print("[m4] Connected.")

        tracker_tasks = [
            asyncio.create_task(track_position(drone, state)),
            asyncio.create_task(track_flight_mode(drone, state)),
            asyncio.create_task(track_armed(drone, state)),
            asyncio.create_task(track_landed_state(drone, state)),
            asyncio.create_task(track_attitude(drone, state)),
            asyncio.create_task(track_local_position(drone, state)),
        ]

        print(
            "[m4] Waiting for health (is_global_position_ok, is_home_position_ok), "
            f"timeout {HEALTH_TIMEOUT_S}s..."
        )
        await wait_for_health(drone, HEALTH_TIMEOUT_S)
        print("[m4] Health OK.")

        if args.fpv:
            # Push the FPV envelope params BEFORE arming (ADR-0010 decision #3):
            # runtime MAVSDK param API, never an airframe-file edit. Read back
            # each one so a stale value can't silently contaminate a run (and
            # so the run's own log records exactly what flew).
            print("[m4] FPV: setting PX4 params (read back for the log header)...")
            for name, value in FPV["PX4_PARAMS"].items():
                await drone.param.set_param_float(name, value)
                got = await drone.param.get_param_float(name)
                print(f"[m4]   {name} = {got} (requested {value})")
                if abs(got - value) > 1e-3:
                    raise RuntimeError(
                        f"PX4 param {name} did not take (got {got}, wanted {value})"
                    )

        print(f"[m4] Setting takeoff altitude to {ALT_REF_M} m...")
        await drone.action.set_takeoff_altitude(ALT_REF_M)

        print("[m4] Arming...")
        await drone.action.arm()

        print("[m4] Commanding takeoff...")
        await drone.action.takeoff()

        # NOTE: PX4 may floor very low takeoff altitudes / the relative
        # altitude may wobble a bit above target during takeoff itself --
        # that's fine, the altitude P-loop takes over once we're in
        # OFFBOARD (same note as M3).
        success_altitude = ALT_REF_M * ALTITUDE_SUCCESS_FRACTION
        print(
            f"[m4] Waiting for relative_altitude_m >= {success_altitude:.2f} m, "
            f"timeout {ALTITUDE_TIMEOUT_S}s..."
        )
        altitude_reached = False
        deadline = time.monotonic() + ALTITUDE_TIMEOUT_S
        while time.monotonic() < deadline:
            detected, meas = sample_measurement(meas_holder)
            gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
            write_row_m4(
                writer, log_file, time.monotonic() - started, "TAKEOFF", args.law,
                detected, meas, state.yaw_deg, None, None, None, None, None, None,
                0.0, None, state.relative_altitude_m, gt_cam, gt_tag, gt_range,
            )
            if (
                state.relative_altitude_m is not None
                and state.relative_altitude_m >= success_altitude
            ):
                altitude_reached = True
                break
            await asyncio.sleep(1.0 / LOG_POLL_HZ)

        if not altitude_reached:
            raise RuntimeError(
                f"altitude target not reached within {ALTITUDE_TIMEOUT_S}s "
                f"(last relative_altitude_m={state.relative_altitude_m})"
            )
        print(f"[m4] Altitude reached: {state.relative_altitude_m:.2f} m")

        print("[m4] Entering OFFBOARD (streaming a zero setpoint first)...")
        offboard_ok = True
        try:
            if args.bench:
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            else:
                yaw0 = state.yaw_deg if state.yaw_deg is not None else 0.0
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw0))
            await drone.offboard.start()
        except OffboardError as error:
            print(f"[m4] FAILED: offboard start failed: {error}")
            offboard_ok = False

        if offboard_ok:
            print(f"[m4] OFFBOARD active. Mode={'BENCH' if args.bench else args.law}.")
            if args.bench:
                passed, mean_val, max_val = await run_bench(
                    drone, state, meas_holder, tracker, writer, log_file, started, args
                )
                result_code = 0 if passed else 1
                print(
                    f"[m4] BENCH {'PASS' if passed else 'FAIL'} "
                    f"(mean {mean_val:.3f} deg/s, bar < {BENCH_PASS_MEAN_DEG_S} deg/s)"
                )
            else:
                result = await run_acquire_and_engage(
                    drone, state, meas_holder, tracker, writer, log_file, started, args
                )
                mover_proc = result["mover_proc"]

                min_gt_running = result["min_gt_range_running"]
                miss_from_csv = recompute_min_gt_range_from_csv(log_path)
                if min_gt_running is not None and not math.isnan(miss_from_csv):
                    if abs(miss_from_csv - min_gt_running) > 1e-2:
                        print(
                            f"[m4] WARNING: running-min miss ({min_gt_running:.3f} m) "
                            f"and CSV-recomputed miss ({miss_from_csv:.3f} m) disagree "
                            "by more than 0.01 m"
                        )
                miss_distance = (
                    miss_from_csv if not math.isnan(miss_from_csv)
                    else (min_gt_running if min_gt_running is not None else float("nan"))
                )

                coverage = (
                    result["n_detected_ticks"] / result["n_ticks"]
                    if result["n_ticks"] else 0.0
                )
                engaged_close = (
                    min_gt_running is not None and min_gt_running < BREAKOFF_ARM_RANGE_M
                )
                clean = bool((not result["aborted"]) and engaged_close)

                print(
                    f"[m4] law={args.law} miss_distance_m={miss_distance:.3f} "
                    f"n_ticks={result['n_ticks']} coverage={coverage:.3f} "
                    f"breakoff_reason={result['breakoff_reason'] or 'n/a'} "
                    f"aborted={result['aborted']} log={log_path}"
                )
                engaged = mover_proc is not None
                print(
                    f"M4_RESULT law={args.law} miss={miss_distance:.3f} "
                    f"clean={int(clean)} engaged={int(engaged)}"
                )

                exit_ok = clean
                if exit_ok and args.require_miss is not None:
                    exit_ok = miss_distance < args.require_miss
                result_code = 0 if exit_ok else 1
        else:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
            result_code = 1

        if mover_proc is not None and mover_proc.poll() is None:
            print("[m4] Terminating mover subprocess...")
            mover_proc.terminate()
            try:
                mover_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                print("[m4] Mover did not exit in time, killing...")
                mover_proc.kill()
                mover_proc.wait(timeout=3.0)

        print("[m4] Guidance phase complete. Stopping offboard and landing...")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"[m4] WARNING: offboard.stop() failed (may not have been active): {error}")

        await drone.action.land()

        print(f"[m4] Waiting for landed/disarmed, timeout {LAND_TIMEOUT_S}s...")
        landed = False
        deadline = time.monotonic() + LAND_TIMEOUT_S
        while time.monotonic() < deadline:
            if state.landed_state == LandedState.ON_GROUND or state.armed is False:
                landed = True
                break
            await asyncio.sleep(1.0 / LOG_POLL_HZ)
        if not landed:
            print(
                f"[m4] WARNING: vehicle did not report landed/disarmed within "
                f"{LAND_TIMEOUT_S}s (last landed_state={state.landed_state}, "
                f"armed={state.armed})"
            )
        else:
            print("[m4] Landed / disarmed.")

        return result_code

    except asyncio.TimeoutError as exc:
        print(f"[m4] FAILED: timed out waiting for a stage: {exc}")
        if not args.bench:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
        return 1
    except ActionError as exc:
        print(f"[m4] FAILED: MAVSDK action error: {exc}")
        if not args.bench:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
        return 1
    except RuntimeError as exc:
        print(f"[m4] FAILED: {exc}")
        if not args.bench:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
        return 1
    except Exception as exc:  # noqa: BLE001 - top-level gate script, log and exit
        print(f"[m4] FAILED: unexpected error: {exc!r}")
        if not args.bench:
            print(f"M4_RESULT law={args.law} miss=nan clean=0 engaged=0")
        return 1
    finally:
        if mover_proc is not None and mover_proc.poll() is None:
            mover_proc.kill()
        for task in tracker_tasks:
            task.cancel()
        for task in tracker_tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception:
                pass
        stop_event.set()
        detector_thread.join(timeout=2.0)
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        log_file.close()
        print(f"[m4] Log written to {log_path}")


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
