#!/usr/bin/env python3
"""Guidance-method trade-study lab: a FAST, pure-Python, point-mass kinematic
Monte-Carlo harness for comparing intercept guidance laws. See GOALS.md's
"guidance arc" and scripts/m4_intercept.py (the real Gazebo mechanization).

WHAT THIS IS: a design-time SURROGATE for the full PX4/Gazebo sim. It has NO
Gazebo, NO MAVSDK, NO camera rendering, NO AprilTag detector -- just a 2D
horizontal point-mass interceptor, a parameterized target path, a sensor
model that mimics the camera's range/dropout/rate limits, and a small
library of pluggable guidance laws. Because it is pure numeric integration
at a fixed 50 Hz tick with tiny per-tick cost, it runs THOUSANDS of
intercepts in seconds, which makes it practical to sweep methods x target
paths x speeds x seeds x gains -- something the real Gazebo sim (which
takes real wall-clock minutes per single run) cannot do at any useful scale.

MODELING ASSUMPTIONS (the honesty boundary of this tool):
  - Interceptor is a point mass in the horizontal plane (altitude ignored;
    targets stay level, matching M4's fixed-altitude engagements). Commanded
    velocity is tracked through a first-order lag (TAU=0.3s) with a hard
    per-step acceleration clamp (A_MAX=12 m/s^2) and a top-speed clamp
    (V_MAX=16 m/s) -- these three numbers are the MEASURED PX4 velocity-
    tracking envelope from the M4/M4.5 dev runs (ADR-0009/0010), not
    invented.
  - The "camera" sensor yields a relative-position measurement (target minus
    interceptor) only inside DET_RANGE, at ~14 Hz (matching the detector's
    measured framerate in m4_intercept.py), with Gaussian bearing/range
    noise and an EDGE-WEIGHTED dropout probability -- it does NOT do the
    strapdown lambda=psi+beta reconstruction from m4_intercept.py's module
    docstring; here the returned bearing IS the inertial LOS angle directly
    (own vehicle yaw is not separately exposed to the guidance laws). That
    reconstruction is validated in Gazebo (ADR-0009); this lab's job is
    comparing LATERAL GUIDANCE LAWS, not re-deriving the strapdown-frame
    math.
      A SEPARATE, internal-to-the-sensor own-heading model (own_heading)
    DOES exist now (ADR-0011 2nd addendum calibration pass): the boresight
    slews toward the last commanded absolute yaw (== the last fresh
    detection's world-frame bearing, mirroring m4_intercept.py's
    "yaw_deg = psi + beta" absolute-yaw-setpoint law) at a limited rate
    (YAW_RATE_MAX_DEG_S_DEFAULT, mirrors m4_intercept.py's
    YAWSPEED_MAX_DEG_S). A detection only fires when the TRUE bearing is
    within FOV_HALF_DEG_DEFAULT of that boresight AND inside DET_RANGE, and
    the dropout probability itself rises toward the FOV edge. This is what
    lets a fast crosser's LOS rate outrun the vehicle's yaw and walk the
    tag out of frame -- the real Gazebo dropout/coverage mechanism (see the
    bearing trace in m4_intercept_pronav_...T044945Z.csv) that the
    ORIGINAL (ADR-0011) version of this lab did not model at all.
  - An optional "external cue" (degraded ground-truth position, ADR-0010's
    mocked ground-sensor handoff) is available only beyond HANDOFF_RANGE, at
    10 Hz, with its own position noise and fixed latency, and NO field-of-
    view limit (it is an external/ground sensor watching from outside, not
    the interceptor's own onboard camera). It is a toggle (--two-stage): the
    main trade study defaults to CAMERA-ONLY, since that is this project's
    headline capability (GOALS.md: "when the datalink is denied, the
    interceptor's own camera locks the target"). A new `two_stage_dash`
    guidance method (S2 design, ADR-0011 2nd addendum) uses the cue to DASH
    at up to DASH_SPEED while beyond HANDOFF_RANGE, then hands off to a
    normal camera-only terminal law (default pure_pn) once inside
    HANDOFF_RANGE with a fresh camera lock -- the "running start" the
    addendum's S1<->S2 coupling finding calls for.

WHY THIS EXISTS: to answer "which guidance law (and gain) should we port
into the real Gazebo sim" cheaply, before spending Gazebo wall-clock time.
THE WINNER HERE IS A HYPOTHESIS, NOT A RESULT. It still has to be validated
by an actual scripted Gazebo gate (check_m4.sh-style) before anything in
docs/decisions.md can cite it as a real number -- this script's CSV is a
design aid, not a milestone gate.

Run:
    .venv/bin/python scripts/guidance_lab.py --quick     # smoke test, seconds
    .venv/bin/python scripts/guidance_lab.py              # full trade study
"""

import argparse
import csv
import functools
import math
import os
import sys
import time
from datetime import datetime, timezone

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")

# --- Simulation constants -----------------------------------------------
DT = 1.0 / 50.0            # 50 Hz sim step
T_MAX_S = 15.0              # cap per run
HIT_RANGE_M = 0.5           # "intercept" range for intercept_time purposes

# --- Vehicle envelope (measured PX4 velocity-tracking behavior, ADR-0009/
# 0010 dev runs -- see module docstring) ---
TAU_S = 0.3                 # first-order velocity-tracking lag
V_MAX_M_S = 16.0            # top speed clamp
A_MAX_M_S2 = 12.0           # per-step acceleration envelope

# --- Sensor model defaults ---
DET_RANGE_DEFAULT_M = 8.0        # camera valid only inside this range
HANDOFF_RANGE_DEFAULT_M = 8.0    # external cue valid only beyond this range
CAM_HZ = 14.0                     # measured detector framerate (m4_intercept.py)
CUE_HZ = 10.0
SIGMA_BEARING_DEG = 0.5
RANGE_NOISE_FRAC = 0.02
# Dropout is EDGE-WEIGHTED (ADR-0011 2nd-addendum calibration pass): DROPOUT_P
# is the rate right at boresight center, DROPOUT_P_EDGE is the rate once the
# true bearing sits right at the FOV edge, ramped by DROPOUT_EDGE_EXPONENT.
# Tuned so a 3-4 m/s crosser's engagement detection_coverage lands ~0.6-0.7
# and a 6 m/s crosser's lands much lower -- both match the real Gazebo CSVs
# (m4_intercept_pronav_...T044945Z.csv: 0.67; ...T044458Z.csv (6 m/s): 0.36).
DROPOUT_P = 0.05
DROPOUT_P_EDGE = 0.65
DROPOUT_EDGE_EXPONENT = 2.0
# Camera half field-of-view (mirrors m4_intercept.py's own comment: "nominal
# +-50 deg half-FOV"). A detection can only fire when the TRUE bearing is
# within this of the vehicle's own (slewing) boresight -- see FOV/heading
# model in the Sensor class docstring.
FOV_HALF_DEG_DEFAULT = 50.0
# Max yaw slew rate the vehicle's own boresight can track at (mirrors
# m4_intercept.py's YAWSPEED_MAX_DEG_S=60, raised for FPV's faster LOS
# rates). This is THE mechanism that lets a fast crosser outrun the yaw and
# walk off-boresight -- not a flat dropout percentage.
YAW_RATE_MAX_DEG_S_DEFAULT = 60.0
CUE_SIGMA_M = 0.5
CUE_LATENCY_S = 0.1

# "Lock" bookkeeping for detection_coverage: a measurement is considered to
# still be held (not lost) for this long after it arrives -- mirrors
# m4_intercept.py's MEAS_STALE_S idea (control loop runs faster than the
# sensor, so "no new frame this tick" is normal cadence, not a loss).
MEAS_STALE_S = 0.3
# Range inside which a genuine camera dropout counts as "tag lost in
# terminal" (mirrors m4_intercept.py's FPV TERMINAL_RANGE_M=5.0 -- the FPV
# profile's rescaled value, since this lab is now calibrated to the FPV
# engagement, not M4's original 2 m/s baseline).
TERMINAL_LOST_TAG_RANGE_M = 5.0
# Default terminal-freeze radius for LOS-rate-driven laws (mirrors
# m4_intercept.py's FPV TERMINAL_FREEZE_RANGE_M=3.5: lambda_dot is singular
# as R->0, so guidance coasts on the last computed command instead of
# chasing a blown-up rate estimate; FPV rescaled this from M4's 2.0 up to
# 3.5 to match its faster terminal closing speed's time semantics).
TERMINAL_FREEZE_RANGE_M = 3.5


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def wrap_pi(angle):
    """Wrap an angle (radians) into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# =========================================================================
# Alpha-beta (g-h) filter -- same mechanization as m4_intercept.py's
# AlphaBetaFilter. Copied (not imported) so this lab has ZERO dependency on
# gz-transport/MAVSDK-importing modules -- see the module docstring's "pure
# Python, NO-Gazebo, NO-MAVSDK" requirement.
# =========================================================================
class AlphaBetaFilter:
    """Constant-velocity tracker for one scalar channel. predict() every
    tick; correct() only on ticks with a genuinely fresh measurement.
    angular=True wraps the residual (not x_hat itself) to [-pi, pi] -- lets
    lambda accumulate continuously while still comparing correctly to a
    measurement that only makes sense mod 2*pi."""

    def __init__(self, alpha, beta, angular=False):
        self.alpha = alpha
        self.beta = beta
        self.angular = angular
        self.x_hat = None
        self.xdot_hat = 0.0
        self._last_t = None

    @property
    def initialized(self):
        return self.x_hat is not None

    def predict(self, dt):
        if self.x_hat is None:
            return
        self.x_hat += self.xdot_hat * dt

    def correct(self, meas, t):
        if self.x_hat is None:
            self.x_hat = meas
            self.xdot_hat = 0.0
            self._last_t = t
            return
        residual = meas - self.x_hat
        if self.angular:
            residual = wrap_pi(residual)
        dt_since = max(1e-3, t - self._last_t)
        self.x_hat += self.alpha * residual
        self.xdot_hat += self.beta * residual / dt_since
        self._last_t = t


class TargetTracker:
    """Alpha-beta filters on the target's ABSOLUTE (x, y) position, giving
    a position+velocity estimate from a stream of possibly-noisy,
    possibly-intermittent absolute-position measurements (own_pos + a
    measured relative vector, from EITHER sensor channel). Also keeps a
    smoothed (EMA) finite-difference of the velocity estimate as an
    acceleration estimate, for methods that need it (apn). This is the
    "estimate target velocity from the sequence of absolute target position
    deltas" piece described in the task brief.
    """

    def __init__(self, alpha=0.6, beta=0.2, accel_ema=0.3, accel_warmup_corrections=6):
        self.fx = AlphaBetaFilter(alpha, beta)
        self.fy = AlphaBetaFilter(alpha, beta)
        self.accel_ema_gain = accel_ema
        # WARMUP GUARD (found empirically, see guidance_lab.py's own dev
        # trace): the velocity estimate itself is still RAMPING from 0 up
        # to the true value for the first few corrections after track
        # start (the alpha-beta filter's own convergence transient) -- that
        # ramp looks exactly like acceleration to a naive finite-difference,
        # even against a truly zero-acceleration (constant-velocity)
        # target. A real tracker would not trust an acceleration estimate
        # before its own velocity estimate has settled, so accel_hat stays
        # at zero for the first `accel_warmup_corrections` corrections.
        self.accel_warmup_corrections = accel_warmup_corrections
        self._n_corrections = 0
        self.accel_hat = (0.0, 0.0)
        self._prev_vel = None
        self._prev_t = None

    def predict(self, dt):
        self.fx.predict(dt)
        self.fy.predict(dt)

    def correct(self, abs_pos, t):
        self.fx.correct(abs_pos[0], t)
        self.fy.correct(abs_pos[1], t)
        self._n_corrections += 1
        vel = (self.fx.xdot_hat, self.fy.xdot_hat)
        if self._prev_vel is not None and self._n_corrections > self.accel_warmup_corrections:
            dt_since = max(1e-3, t - self._prev_t)
            ax = (vel[0] - self._prev_vel[0]) / dt_since
            ay = (vel[1] - self._prev_vel[1]) / dt_since
            g = self.accel_ema_gain
            self.accel_hat = (
                self.accel_hat[0] + g * (ax - self.accel_hat[0]),
                self.accel_hat[1] + g * (ay - self.accel_hat[1]),
            )
        self._prev_vel = vel
        self._prev_t = t

    @property
    def pos_hat(self):
        if self.fx.x_hat is None:
            return None
        return (self.fx.x_hat, self.fy.x_hat)

    @property
    def vel_hat(self):
        if self.fx.x_hat is None:
            return None
        return (self.fx.xdot_hat, self.fy.xdot_hat)


# =========================================================================
# Vehicle model
# =========================================================================
def step_vehicle(vx, vy, vcx, vcy, dt=DT, tau=TAU_S, v_max=V_MAX_M_S, a_max=A_MAX_M_S2):
    """First-order lag toward the commanded velocity, clamped to a per-step
    acceleration envelope and a top-speed cap. See module docstring for
    where these three numbers come from."""
    vx_new = vx + (vcx - vx) * (dt / tau)
    vy_new = vy + (vcy - vy) * (dt / tau)
    dvx, dvy = vx_new - vx, vy_new - vy
    dv_norm = math.hypot(dvx, dvy)
    max_dv = a_max * dt
    if dv_norm > max_dv and dv_norm > 0.0:
        scale = max_dv / dv_norm
        dvx *= scale
        dvy *= scale
    vx_new, vy_new = vx + dvx, vy + dvy
    speed = math.hypot(vx_new, vy_new)
    if speed > v_max:
        s = v_max / speed
        vx_new *= s
        vy_new *= s
    return vx_new, vy_new


# =========================================================================
# Sensor model
# =========================================================================
class Measurement:
    __slots__ = ("t", "rel_xy", "range_m", "bearing_rad", "source")

    def __init__(self, t, rel_xy, range_m, bearing_rad, source):
        self.t = t
        self.rel_xy = rel_xy
        self.range_m = range_m
        self.bearing_rad = bearing_rad
        self.source = source  # "camera" | "cue"


class Sensor:
    """The interceptor's ONLY window onto the target -- mirrors the camera
    honesty boundary (GOALS.md / ADR-0008 lineage): guidance never sees
    ground truth, only these measurements.

    Camera: valid inside det_range, ~cam_hz update rate, Gaussian
    bearing/range noise, random per-attempt dropout.
    External cue (optional, --two-stage): valid beyond handoff_range,
    ~cue_hz, Gaussian ABSOLUTE-position noise, fixed latency (delivered
    `cue_latency_s` after the sample was taken).
    """

    def __init__(
        self, rng, det_range=DET_RANGE_DEFAULT_M, handoff_range=HANDOFF_RANGE_DEFAULT_M,
        cam_hz=CAM_HZ, cue_hz=CUE_HZ, sigma_bearing_deg=SIGMA_BEARING_DEG,
        range_noise_frac=RANGE_NOISE_FRAC, dropout_p=DROPOUT_P,
        cue_sigma_m=CUE_SIGMA_M, cue_latency_s=CUE_LATENCY_S,
    ):
        self.rng = rng
        self.det_range = det_range
        self.handoff_range = handoff_range
        self.cam_period = 1.0 / cam_hz
        self.cue_period = 1.0 / cue_hz
        self.sigma_bearing = math.radians(sigma_bearing_deg)
        self.range_noise_frac = range_noise_frac
        self.dropout_p = dropout_p
        self.cue_sigma = cue_sigma_m
        self.cue_latency = cue_latency_s
        self._cam_timer = 0.0
        self._cue_timer = 0.0
        self._cue_pending = []  # list of (deliver_t, x, y)

    def tick(self, t, dt, own_pos, target_pos, two_stage):
        rel = (target_pos[0] - own_pos[0], target_pos[1] - own_pos[1])
        true_range = math.hypot(rel[0], rel[1])

        self._cam_timer += dt
        cam_meas = None
        if self._cam_timer >= self.cam_period:
            self._cam_timer -= self.cam_period
            if true_range < self.det_range and self.rng.random() >= self.dropout_p:
                true_bearing = math.atan2(rel[1], rel[0])
                range_n = true_range * (1.0 + self.rng.normal(0.0, self.range_noise_frac))
                bearing_n = true_bearing + self.rng.normal(0.0, self.sigma_bearing)
                rel_n = (range_n * math.cos(bearing_n), range_n * math.sin(bearing_n))
                cam_meas = Measurement(t, rel_n, range_n, bearing_n, "camera")

        cue_meas = None
        if two_stage:
            self._cue_timer += dt
            if self._cue_timer >= self.cue_period:
                self._cue_timer -= self.cue_period
                if true_range > self.handoff_range:
                    nx = target_pos[0] + self.rng.normal(0.0, self.cue_sigma)
                    ny = target_pos[1] + self.rng.normal(0.0, self.cue_sigma)
                    self._cue_pending.append((t + self.cue_latency, nx, ny))
            while self._cue_pending and self._cue_pending[0][0] <= t:
                _, nx, ny = self._cue_pending.pop(0)
                rel_n = (nx - own_pos[0], ny - own_pos[1])
                r = math.hypot(*rel_n)
                b = math.atan2(rel_n[1], rel_n[0])
                cue_meas = Measurement(t, rel_n, r, b, "cue")

        # Camera takes priority in the (rare, boundary-tick) case both fire
        # this tick -- guidance uses the cue only in the CUE phase and the
        # camera only in TERMINAL (ADR-0010 decision #5); with the default
        # equal det_range/handoff_range this collision is a non-event.
        return cam_meas if cam_meas is not None else cue_meas


# =========================================================================
# Guidance law library
# =========================================================================
DEFAULT_PN_PARAMS = dict(
    N=4.0, V_CLOSE=8.0, VC_FLOOR=1.5, V_PERP_MAX=6.0, V_TOTAL_MAX=14.0,
    ALPHA=0.5, BETA_LAMBDA=0.30, BETA_RANGE=0.15,
    TERMINAL_FREEZE_RANGE=TERMINAL_FREEZE_RANGE_M,
)
DEFAULT_PURSUIT_PARAMS = dict(DEFAULT_PN_PARAMS)  # shares gains w/ pure_pn (fairness, ADR-0009 style)
DEFAULT_APN_PARAMS = dict(DEFAULT_PN_PARAMS, TRACK_ALPHA=0.6, TRACK_BETA=0.2, ACCEL_EMA=0.3)
DEFAULT_PN_PLUS_LEAD_PARAMS = dict(DEFAULT_PN_PARAMS, TRACK_ALPHA=0.6, TRACK_BETA=0.2)
DEFAULT_PIP_PARAMS = dict(
    V_CLOSE=8.0, MAX_LEAD_S=6.0, TERMINAL_FREEZE_RANGE=0.5,
    TRACK_ALPHA=0.6, TRACK_BETA=0.2,
)


class Pursuit:
    """Baseline: aim velocity along the filtered LOS bearing at a constant
    closing speed -- no lead, no LOS-rate nulling. Shares its lambda/range
    alpha-beta filters, gains, V_CLOSE and terminal-freeze with pure_pn (the
    ONLY difference is the lateral term is always zero) so a miss-distance
    comparison between the two isolates that one term (mirrors
    m4_intercept.py's fairness design)."""

    NAME = "pursuit"

    def __init__(self, params):
        p = {**DEFAULT_PURSUIT_PARAMS, **(params or {})}
        self.v_close = p["V_CLOSE"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.lam = AlphaBetaFilter(p["ALPHA"], p["BETA_LAMBDA"], angular=True)
        self.rng_f = AlphaBetaFilter(p["ALPHA"], p["BETA_RANGE"])
        self.frozen = None
        self.last_cmd = (0.0, 0.0)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        if meas is not None and meas.source == "camera":
            self.lam.correct(meas.bearing_rad, t)
            self.rng_f.correct(meas.range_m, t)
        if not self.lam.initialized:
            return (0.0, 0.0)  # no lock yet: hold position

        r_hat = self.rng_f.x_hat
        if self.frozen is not None or (r_hat is not None and r_hat < self.freeze_range):
            if self.frozen is None:
                self.frozen = self.last_cmd
            return self.frozen

        lambda_hat = self.lam.x_hat
        vx = self.v_close * math.cos(lambda_hat)
        vy = self.v_close * math.sin(lambda_hat)
        self.last_cmd = (vx, vy)
        return (vx, vy)


class PurePN:
    """a_cmd = N * Vc * lambda_dot, mechanized exactly like
    scripts/m4_intercept.py: alpha-beta filters on the LOS angle (lambda)
    and range; Vc for the LATERAL-ACCEL FORMULA is a floored closing-rate
    estimate (-range_rate_hat), while the FORWARD (along-LOS) speed command
    is a separate constant V_CLOSE -- these are two different numbers in
    the real mechanization, not a naming accident (see m4_intercept.py's
    compute_v_close vs its VC_FLOOR_M_S comment)."""

    NAME = "pure_pn"

    def __init__(self, params):
        p = {**DEFAULT_PN_PARAMS, **(params or {})}
        self.N = p["N"]
        self.v_close = p["V_CLOSE"]
        self.vc_floor = p["VC_FLOOR"]
        self.v_perp_max = p["V_PERP_MAX"]
        self.v_total_max = p["V_TOTAL_MAX"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.lam = AlphaBetaFilter(p["ALPHA"], p["BETA_LAMBDA"], angular=True)
        self.rng_f = AlphaBetaFilter(p["ALPHA"], p["BETA_RANGE"])
        self.v_perp = 0.0
        self.frozen = None
        self.last_cmd = (0.0, 0.0)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        if meas is not None and meas.source == "camera":
            self.lam.correct(meas.bearing_rad, t)
            self.rng_f.correct(meas.range_m, t)
        if not self.lam.initialized:
            return (0.0, 0.0)

        r_hat = self.rng_f.x_hat
        if self.frozen is not None or (r_hat is not None and r_hat < self.freeze_range):
            if self.frozen is None:
                self.frozen = self.last_cmd
            return self.frozen

        lambda_hat = self.lam.x_hat
        lambda_dot = self.lam.xdot_hat
        rdot_hat = self.rng_f.xdot_hat if self.rng_f.initialized else None
        vc = max(self.vc_floor, -rdot_hat) if rdot_hat is not None else self.vc_floor
        a_cmd = self.N * vc * lambda_dot
        self.v_perp = clamp(self.v_perp + a_cmd * dt, -self.v_perp_max, self.v_perp_max)

        u = (math.cos(lambda_hat), math.sin(lambda_hat))
        pvec = (-math.sin(lambda_hat), math.cos(lambda_hat))
        vx = self.v_close * u[0] + self.v_perp * pvec[0]
        vy = self.v_close * u[1] + self.v_perp * pvec[1]
        norm = math.hypot(vx, vy)
        if norm > self.v_total_max:
            s = self.v_total_max / norm
            vx *= s
            vy *= s
        self.last_cmd = (vx, vy)
        return (vx, vy)


class APN:
    """Augmented pro-nav: pure_pn's a_cmd PLUS (N/2)*a_target_perp, where
    a_target is the target's estimated inertial acceleration (from a
    TargetTracker: alpha-beta position/velocity filter on the absolute
    target position, then an EMA-smoothed finite difference of the
    velocity estimate), projected onto the perpendicular-to-LOS direction.
    This is the textbook maneuvering-target upgrade to PN -- it needs a
    SECOND derivative of an already-noisy signal, which is exactly why
    ADR-0010 rejected it for the real Gazebo mechanization (a new failure
    mode class); it is included HERE because this lab's whole purpose is
    to measure whether that extra complexity is worth it in principle
    before ever deciding to port it."""

    NAME = "apn"

    def __init__(self, params):
        p = {**DEFAULT_APN_PARAMS, **(params or {})}
        self.N = p["N"]
        self.v_close = p["V_CLOSE"]
        self.vc_floor = p["VC_FLOOR"]
        self.v_perp_max = p["V_PERP_MAX"]
        self.v_total_max = p["V_TOTAL_MAX"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.lam = AlphaBetaFilter(p["ALPHA"], p["BETA_LAMBDA"], angular=True)
        self.rng_f = AlphaBetaFilter(p["ALPHA"], p["BETA_RANGE"])
        self.tracker = TargetTracker(p["TRACK_ALPHA"], p["TRACK_BETA"], p["ACCEL_EMA"])
        self.v_perp = 0.0
        self.frozen = None
        self.last_cmd = (0.0, 0.0)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        self.tracker.predict(dt)
        if meas is not None:
            abs_pos = (own_pos[0] + meas.rel_xy[0], own_pos[1] + meas.rel_xy[1])
            self.tracker.correct(abs_pos, t)
            if meas.source == "camera":
                self.lam.correct(meas.bearing_rad, t)
                self.rng_f.correct(meas.range_m, t)
        if not self.lam.initialized:
            return (0.0, 0.0)

        r_hat = self.rng_f.x_hat
        if self.frozen is not None or (r_hat is not None and r_hat < self.freeze_range):
            if self.frozen is None:
                self.frozen = self.last_cmd
            return self.frozen

        lambda_hat = self.lam.x_hat
        lambda_dot = self.lam.xdot_hat
        rdot_hat = self.rng_f.xdot_hat if self.rng_f.initialized else None
        vc = max(self.vc_floor, -rdot_hat) if rdot_hat is not None else self.vc_floor

        pvec = (-math.sin(lambda_hat), math.cos(lambda_hat))
        a_t = self.tracker.accel_hat
        a_target_perp = a_t[0] * pvec[0] + a_t[1] * pvec[1]
        a_cmd = self.N * vc * lambda_dot + 0.5 * self.N * a_target_perp
        self.v_perp = clamp(self.v_perp + a_cmd * dt, -self.v_perp_max, self.v_perp_max)

        u = (math.cos(lambda_hat), math.sin(lambda_hat))
        vx = self.v_close * u[0] + self.v_perp * pvec[0]
        vy = self.v_close * u[1] + self.v_perp * pvec[1]
        norm = math.hypot(vx, vy)
        if norm > self.v_total_max:
            s = self.v_total_max / norm
            vx *= s
            vy *= s
        self.last_cmd = (vx, vy)
        return (vx, vy)


def solve_intercept_time(rel, vt, v_close, max_lead_s):
    """Classic lead-pursuit / PIP intercept-triangle solve: smallest
    positive t such that |rel + vt*t| = v_close*t (target at `rel`, moving
    at constant `vt`; interceptor closing at constant speed `v_close`).
    Falls back to t=0 (aim at the target's CURRENT estimated position --
    plain pursuit) when the target outruns v_close and is diverging, or
    the quadratic degenerates."""
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


class PIP:
    """Predicted intercept point / lead guidance: track the target's
    absolute position+velocity (TargetTracker), solve the intercept
    triangle for the lead point where a constant-speed (V_CLOSE)
    interceptor and the constant-velocity target estimate would meet, aim
    straight at that point. THE direct fix for pure pursuit's "always
    lagging behind a mover" failure mode."""

    NAME = "pip"

    def __init__(self, params):
        p = {**DEFAULT_PIP_PARAMS, **(params or {})}
        self.v_close = p["V_CLOSE"]
        self.max_lead_s = p["MAX_LEAD_S"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.tracker = TargetTracker(p["TRACK_ALPHA"], p["TRACK_BETA"])
        self.frozen = None
        self.last_cmd = (0.0, 0.0)
        self._have_meas = False

    def step(self, t, dt, own_pos, own_vel, meas):
        self.tracker.predict(dt)
        if meas is not None:
            abs_pos = (own_pos[0] + meas.rel_xy[0], own_pos[1] + meas.rel_xy[1])
            self.tracker.correct(abs_pos, t)
            self._have_meas = True
        if not self._have_meas:
            return (0.0, 0.0)

        target_pos = self.tracker.pos_hat
        rel = (target_pos[0] - own_pos[0], target_pos[1] - own_pos[1])
        r_hat = math.hypot(rel[0], rel[1])
        if self.frozen is not None or r_hat < self.freeze_range:
            if self.frozen is None:
                self.frozen = self.last_cmd
            return self.frozen

        vt = self.tracker.vel_hat or (0.0, 0.0)
        t_go = solve_intercept_time(rel, vt, self.v_close, self.max_lead_s)
        aim = (target_pos[0] + vt[0] * t_go, target_pos[1] + vt[1] * t_go)
        direction = (aim[0] - own_pos[0], aim[1] - own_pos[1])
        dnorm = math.hypot(direction[0], direction[1])
        if dnorm < 1e-6:
            vx, vy = self.last_cmd
        else:
            vx = self.v_close * direction[0] / dnorm
            vy = self.v_close * direction[1] / dnorm
        self.last_cmd = (vx, vy)
        return (vx, vy)


class PNPlusLead:
    """Simpler hybrid: pure PN's LOS-rate-nulling vector (same lambda/range
    filters, same v_perp integrator as pure_pn) PLUS a raw feedforward of
    the estimated target velocity vector (TargetTracker). No intercept-
    triangle solve -- just addition -- which is what makes this the
    "simpler" alternative to PIP."""

    NAME = "pn_plus_lead"

    def __init__(self, params):
        p = {**DEFAULT_PN_PLUS_LEAD_PARAMS, **(params or {})}
        self.N = p["N"]
        self.v_close = p["V_CLOSE"]
        self.vc_floor = p["VC_FLOOR"]
        self.v_perp_max = p["V_PERP_MAX"]
        self.v_total_max = p["V_TOTAL_MAX"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.lam = AlphaBetaFilter(p["ALPHA"], p["BETA_LAMBDA"], angular=True)
        self.rng_f = AlphaBetaFilter(p["ALPHA"], p["BETA_RANGE"])
        self.tracker = TargetTracker(p["TRACK_ALPHA"], p["TRACK_BETA"])
        self.v_perp = 0.0
        self.frozen = None
        self.last_cmd = (0.0, 0.0)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        self.tracker.predict(dt)
        if meas is not None:
            abs_pos = (own_pos[0] + meas.rel_xy[0], own_pos[1] + meas.rel_xy[1])
            self.tracker.correct(abs_pos, t)
            if meas.source == "camera":
                self.lam.correct(meas.bearing_rad, t)
                self.rng_f.correct(meas.range_m, t)
        if not self.lam.initialized:
            return (0.0, 0.0)

        r_hat = self.rng_f.x_hat
        if self.frozen is not None or (r_hat is not None and r_hat < self.freeze_range):
            if self.frozen is None:
                self.frozen = self.last_cmd
            return self.frozen

        lambda_hat = self.lam.x_hat
        lambda_dot = self.lam.xdot_hat
        rdot_hat = self.rng_f.xdot_hat if self.rng_f.initialized else None
        vc = max(self.vc_floor, -rdot_hat) if rdot_hat is not None else self.vc_floor
        a_cmd = self.N * vc * lambda_dot
        self.v_perp = clamp(self.v_perp + a_cmd * dt, -self.v_perp_max, self.v_perp_max)

        u = (math.cos(lambda_hat), math.sin(lambda_hat))
        pvec = (-math.sin(lambda_hat), math.cos(lambda_hat))
        vx = self.v_close * u[0] + self.v_perp * pvec[0]
        vy = self.v_close * u[1] + self.v_perp * pvec[1]
        vt = self.tracker.vel_hat
        if vt is not None:
            vx += vt[0]
            vy += vt[1]
        norm = math.hypot(vx, vy)
        if norm > self.v_total_max:
            s = self.v_total_max / norm
            vx *= s
            vy *= s
        self.last_cmd = (vx, vy)
        return (vx, vy)


METHODS = {
    "pursuit": (Pursuit, DEFAULT_PURSUIT_PARAMS),
    "pure_pn": (PurePN, DEFAULT_PN_PARAMS),
    "apn": (APN, DEFAULT_APN_PARAMS),
    "pip": (PIP, DEFAULT_PIP_PARAMS),
    "pn_plus_lead": (PNPlusLead, DEFAULT_PN_PLUS_LEAD_PARAMS),
}


# =========================================================================
# Target path library -- each builder returns (start_pos, vel_fn(t)) and is
# deterministic given the shared per-run RNG (small geometry jitter is
# drawn from it so a Monte-Carlo sweep varies genuine geometry, not just
# sensor noise). Speeds parameterizable ~2-10 m/s. Geometry mirrors
# m4_intercept.py's own engagement setup (target start ~6.5 m downrange,
# crossing at a lateral offset) so initial range sits just outside the
# default 8 m detection envelope -- by design, matching ADR-0009's choice.
# =========================================================================
def build_crossing(speed, rng, direction=1.0, x0=6.5, y0_mag=6.0, jitter=0.5):
    """direction=+1: L->R (starts at y=-y0_mag, moves toward +y, crossing
    through the interceptor's vicinity near y=0). direction=-1: R->L,
    mirrored (starts at y=+y0_mag, moves toward -y). The start Y sign MUST
    flip with direction -- a target starting at y=-6 moving further in -y
    never crosses anything, it just flies away (a real bug caught by the
    crossing_r2l path showing 0.0 detection_coverage and identical
    miss_distance for every method during dev testing: an unreachable
    target makes every guidance law look identical because none of them
    ever get a single measurement)."""
    x = x0 + rng.uniform(-jitter, jitter)
    y0 = -direction * y0_mag
    y = y0 + rng.uniform(-jitter, jitter)

    def vel_fn(t):
        return (0.0, direction * speed)

    return (x, y), vel_fn


def build_head_on(speed, rng, x0=10.0, y0=0.0, jitter=0.5):
    x = x0 + rng.uniform(-jitter, jitter)
    y = y0 + rng.uniform(-0.3, 0.3)

    def vel_fn(t):
        return (-speed, 0.0)

    return (x, y), vel_fn


def build_oblique(speed, rng, x0=8.0, y0=-5.0, jitter=0.5):
    x = x0 + rng.uniform(-jitter, jitter)
    y = y0 + rng.uniform(-jitter, jitter)
    angle = math.radians(135.0) + rng.uniform(-0.1, 0.1)  # closing in x, crossing in +y

    def vel_fn(t):
        return (speed * math.cos(angle), speed * math.sin(angle))

    return (x, y), vel_fn


def build_s_weave(speed, rng, x0=6.5, y0=-6.0, jitter=0.5, amp_deg=35.0, period_s=3.0):
    x = x0 + rng.uniform(-jitter, jitter)
    y = y0 + rng.uniform(-jitter, jitter)
    heading0 = math.pi / 2.0
    amp = math.radians(amp_deg)
    omega = 2.0 * math.pi / (period_s + rng.uniform(-0.3, 0.3))
    phase = rng.uniform(0.0, 2.0 * math.pi)

    def vel_fn(t):
        heading = heading0 + amp * math.sin(omega * t + phase)
        return (speed * math.cos(heading), speed * math.sin(heading))

    return (x, y), vel_fn


def build_jink(speed, rng, x0=6.5, y0=-6.0, jitter=0.5, t_jink_nominal=1.2, step_deg=90.0):
    x = x0 + rng.uniform(-jitter, jitter)
    y = y0 + rng.uniform(-jitter, jitter)
    t_jink = t_jink_nominal + rng.uniform(-0.2, 0.2)
    heading0 = math.pi / 2.0
    step = math.radians(step_deg)

    def vel_fn(t):
        heading = heading0 if t < t_jink else heading0 + step
        return (speed * math.cos(heading), speed * math.sin(heading))

    return (x, y), vel_fn


def build_variable_speed(speed, rng, x0=6.5, y0=-6.0, jitter=0.5, ramp_s=5.0):
    x = x0 + rng.uniform(-jitter, jitter)
    y = y0 + rng.uniform(-jitter, jitter)
    heading0 = math.pi / 2.0

    def vel_fn(t):
        frac = min(1.0, t / ramp_s)
        s = speed * (0.5 + frac)  # ramps 0.5x -> 1.5x speed over ramp_s, then holds
        return (s * math.cos(heading0), s * math.sin(heading0))

    return (x, y), vel_fn


PATHS = {
    "crossing_l2r": functools.partial(build_crossing, direction=1.0),
    "crossing_r2l": functools.partial(build_crossing, direction=-1.0),
    "head_on": build_head_on,
    "oblique": build_oblique,
    "s_weave": build_s_weave,
    "jink": build_jink,
    "variable_speed": build_variable_speed,
}


# =========================================================================
# Single-run simulator
# =========================================================================
def simulate(method_name, method_params, path_name, path_params, seed, two_stage=False,
             t_max=T_MAX_S, dt=DT):
    """Run one deterministic (given `seed`) intercept and score it.

    Returns a dict: miss_distance (min interceptor-target range over the
    run), intercept_time (time of that min, or first time range<0.5 m if
    ever reached), control_effort (sum |delta-v| = integral |accel| dt),
    detection_coverage (fraction of ticks WHILE THE TARGET WAS WITHIN
    DETECTION RANGE that had a not-yet-stale measurement -- deliberately
    NOT divided by the full 15s cap, since most of that cap is dead time
    before acquisition / after breakoff and would dilute the number into
    meaninglessness), tag_lost_in_terminal (bool: any camera dropout while
    inside TERMINAL_LOST_TAG_RANGE_M).
    """
    rng = np.random.default_rng(seed)
    path_params = dict(path_params or {})
    speed = path_params.pop("speed", 5.0)
    det_range = path_params.pop("det_range", DET_RANGE_DEFAULT_M)
    handoff_range = path_params.pop("handoff_range", HANDOFF_RANGE_DEFAULT_M)

    builder = PATHS[path_name]
    (tx, ty), vel_fn = builder(speed, rng, **path_params)

    cls, defaults = METHODS[method_name]
    params = {**defaults, **(method_params or {})}
    law = cls(params)

    sensor = Sensor(rng, det_range=det_range, handoff_range=handoff_range)

    ix = iy = ivx = ivy = 0.0
    min_range = math.hypot(tx - ix, ty - iy)
    t_min_range = 0.0
    first_hit_time = None
    control_effort = 0.0
    n_ticks = 0
    n_locked_ticks = 0
    n_in_range_ticks = 0
    last_meas_t = None
    tag_lost_in_terminal = False

    n_steps = int(round(t_max / dt))
    t = 0.0
    for i in range(n_steps):
        t = i * dt
        true_range = math.hypot(tx - ix, ty - iy)
        if true_range < min_range:
            min_range = true_range
            t_min_range = t
        if first_hit_time is None and true_range < HIT_RANGE_M:
            first_hit_time = t

        meas = sensor.tick(t, dt, (ix, iy), (tx, ty), two_stage)
        if meas is not None:
            last_meas_t = t
        has_lock = last_meas_t is not None and (t - last_meas_t) <= MEAS_STALE_S
        if true_range < det_range:
            n_in_range_ticks += 1
            if has_lock:
                n_locked_ticks += 1
        if true_range < TERMINAL_LOST_TAG_RANGE_M and not (
            meas is not None and meas.source == "camera"
        ) and not has_lock:
            tag_lost_in_terminal = True

        vcx, vcy = law.step(t, dt, (ix, iy), (ivx, ivy), meas)

        nvx, nvy = step_vehicle(ivx, ivy, vcx, vcy, dt)
        control_effort += math.hypot(nvx - ivx, nvy - ivy)
        ivx, ivy = nvx, nvy
        ix += ivx * dt
        iy += ivy * dt

        vtx, vty = vel_fn(t)
        tx += vtx * dt
        ty += vty * dt

        n_ticks += 1

    true_range = math.hypot(tx - ix, ty - iy)
    if true_range < min_range:
        min_range = true_range
        t_min_range = t_max

    intercept_time = first_hit_time if first_hit_time is not None else t_min_range
    detection_coverage = n_locked_ticks / n_in_range_ticks if n_in_range_ticks else 0.0

    return dict(
        method=method_name, path=path_name, speed=speed, seed=seed, two_stage=two_stage,
        N=params.get("N"), V_CLOSE=params.get("V_CLOSE"),
        miss_distance=min_range, intercept_time=intercept_time,
        control_effort=control_effort, detection_coverage=detection_coverage,
        tag_lost_in_terminal=tag_lost_in_terminal,
    )


# =========================================================================
# Trade-study driver
# =========================================================================
ALL_METHODS = list(METHODS.keys())
ALL_PATHS = list(PATHS.keys())
DEFAULT_SPEEDS = (3.0, 5.0, 7.0)
DEFAULT_SEEDS = 20


def run_sweep(methods, paths, speeds, n_seeds, two_stage=False, method_params_by_name=None):
    method_params_by_name = method_params_by_name or {}
    rows = []
    for method in methods:
        mp = method_params_by_name.get(method)
        for path in paths:
            for speed in speeds:
                for seed in range(n_seeds):
                    rows.append(
                        simulate(method, mp, path, {"speed": speed}, seed, two_stage=two_stage)
                    )
    return rows


def aggregate_by(rows, keys):
    groups = {}
    for r in rows:
        k = tuple(r[key] for key in keys)
        groups.setdefault(k, []).append(r)
    out = []
    for k, grp in groups.items():
        miss = np.array([g["miss_distance"] for g in grp])
        itime = np.array([g["intercept_time"] for g in grp])
        cov = np.array([g["detection_coverage"] for g in grp])
        effort = np.array([g["control_effort"] for g in grp])
        lost = np.array([g["tag_lost_in_terminal"] for g in grp])
        entry = dict(zip(keys, k))
        entry.update(dict(
            n=len(grp),
            miss_mean=float(miss.mean()), miss_median=float(np.median(miss)),
            miss_p90=float(np.percentile(miss, 90)),
            itime_mean=float(itime.mean()), itime_median=float(np.median(itime)),
            itime_p90=float(np.percentile(itime, 90)),
            coverage_mean=float(cov.mean()),
            effort_mean=float(effort.mean()),
            tag_lost_frac=float(lost.mean()),
        ))
        out.append(entry)
    return out


def print_table(rows, headers, keyfmt):
    widths = [len(h) for h in headers]
    lines = []
    for r in rows:
        vals = keyfmt(r)
        for i, v in enumerate(vals):
            widths[i] = max(widths[i], len(v))
        lines.append(vals)
    fmt = "  ".join("{:<" + str(w) + "}" for w in widths)
    print(fmt.format(*headers))
    print(fmt.format(*["-" * w for w in widths]))
    for vals in lines:
        print(fmt.format(*vals))


def write_csv(rows, path):
    fieldnames = [
        "method", "path", "speed", "seed", "two_stage", "N", "V_CLOSE",
        "miss_distance", "intercept_time", "control_effort",
        "detection_coverage", "tag_lost_in_terminal",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k) for k in fieldnames})


def make_plots(rows, methods, paths, out_prefix):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[guidance_lab] matplotlib not available -- skipping plots")
        return []

    by_method_path = aggregate_by(rows, ("method", "path"))
    lookup = {(e["method"], e["path"]): e for e in by_method_path}

    saved = []
    ncols = min(4, len(paths))
    nrows = int(math.ceil(len(paths) / ncols))

    for metric, ylabel, suffix in (
        ("miss_mean", "mean miss distance (m)", "miss_by_method"),
        ("itime_mean", "mean intercept time (s)", "itime_by_method"),
    ):
        fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 3.2 * nrows), squeeze=False)
        for idx, path in enumerate(paths):
            ax = axes[idx // ncols][idx % ncols]
            vals = [lookup.get((m, path), {}).get(metric, np.nan) for m in methods]
            ax.bar(range(len(methods)), vals, color="tab:blue")
            ax.set_xticks(range(len(methods)))
            ax.set_xticklabels(methods, rotation=45, ha="right", fontsize=8)
            ax.set_title(path, fontsize=9)
            ax.set_ylabel(ylabel, fontsize=8)
        for idx in range(len(paths), nrows * ncols):
            axes[idx // ncols][idx % ncols].axis("off")
        fig.tight_layout()
        out_path = f"{out_prefix}_{suffix}.png"
        fig.savefig(out_path, dpi=110)
        plt.close(fig)
        saved.append(out_path)
        print(f"[guidance_lab] plot saved: {out_path}")
    return saved


GAIN_SWEEP_GRID = {
    "pure_pn": [{"N": n, "V_CLOSE": vc} for n in (3.0, 4.0, 5.0) for vc in (6.0, 8.0, 10.0)],
    "apn": [{"N": n, "V_CLOSE": vc} for n in (3.0, 4.0, 5.0) for vc in (6.0, 8.0, 10.0)],
    "pn_plus_lead": [{"N": n, "V_CLOSE": vc} for n in (3.0, 4.0, 5.0) for vc in (6.0, 8.0, 10.0)],
    "pursuit": [{"V_CLOSE": vc} for vc in (6.0, 8.0, 10.0)],
    "pip": [{"V_CLOSE": vc} for vc in (6.0, 8.0, 10.0)],
}


def run_gain_sweep(top_methods, paths, n_seeds, two_stage=False):
    rows = []
    for method in top_methods:
        for params in GAIN_SWEEP_GRID.get(method, []):
            for path in paths:
                for seed in range(n_seeds):
                    r = simulate(method, params, path, {"speed": 5.0}, seed, two_stage=two_stage)
                    rows.append(r)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--quick", action="store_true", help="small fast sweep (smoke test)")
    parser.add_argument("--two-stage", action="store_true",
                         help="enable the external-cue CUE phase (default: camera-only)")
    parser.add_argument("--no-plots", action="store_true", help="skip matplotlib plots")
    parser.add_argument("--seeds", type=int, default=None, help="override seed count")
    args = parser.parse_args()

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    csv_path = os.path.join(LOGS_DIR, f"guidance_lab_{timestamp}.csv")
    plot_prefix = os.path.join(LOGS_DIR, f"guidance_lab_{timestamp}")

    if args.quick:
        methods = ALL_METHODS
        paths = ["crossing_l2r", "jink"]
        speeds = [5.0]
        n_seeds = args.seeds or 5
    else:
        methods = ALL_METHODS
        paths = ALL_PATHS
        speeds = list(DEFAULT_SPEEDS)
        n_seeds = args.seeds or DEFAULT_SEEDS

    n_runs = len(methods) * len(paths) * len(speeds) * n_seeds
    print(
        f"[guidance_lab] sweep: {len(methods)} methods x {len(paths)} paths x "
        f"{len(speeds)} speeds x {n_seeds} seeds = {n_runs} runs "
        f"(two_stage={args.two_stage})"
    )

    t0 = time.perf_counter()
    rows = run_sweep(methods, paths, speeds, n_seeds, two_stage=args.two_stage)
    elapsed = time.perf_counter() - t0
    print(f"[guidance_lab] main sweep done: {len(rows)} runs in {elapsed:.2f}s "
          f"({len(rows) / max(elapsed, 1e-6):.0f} runs/s)")

    # --- per (method, path) ranking -------------------------------------
    by_mp = aggregate_by(rows, ("method", "path"))
    by_mp.sort(key=lambda r: (r["path"], r["miss_mean"]))

    print("\n=== per (method, path) aggregate (all speeds+seeds pooled) ===")
    print_table(
        by_mp,
        ["path", "method", "n", "miss_mean", "miss_median", "miss_p90",
         "itime_mean", "itime_p90", "coverage", "tag_lost_frac"],
        lambda r: (
            r["path"], r["method"], str(r["n"]),
            f"{r['miss_mean']:.3f}", f"{r['miss_median']:.3f}", f"{r['miss_p90']:.3f}",
            f"{r['itime_mean']:.2f}", f"{r['itime_p90']:.2f}",
            f"{r['coverage_mean']:.2f}", f"{r['tag_lost_frac']:.2f}",
        ),
    )

    # per (method, path, speed) breakdown -- finer-grained than the spec's
    # minimum ask, useful for spotting speed-dependent crossover
    by_mps = aggregate_by(rows, ("method", "path", "speed"))
    by_mps.sort(key=lambda r: (r["path"], r["speed"], r["miss_mean"]))
    print("\n=== per (method, path, speed) breakdown ===")
    print_table(
        by_mps,
        ["path", "speed", "method", "n", "miss_mean", "itime_mean", "coverage"],
        lambda r: (
            r["path"], f"{r['speed']:.0f}", r["method"], str(r["n"]),
            f"{r['miss_mean']:.3f}", f"{r['itime_mean']:.2f}", f"{r['coverage_mean']:.2f}",
        ),
    )

    # --- winners per path ------------------------------------------------
    print("\n=== WINNER per path ===")
    paths_seen = sorted({r["path"] for r in by_mp})
    winner_counts = {}
    for path in paths_seen:
        candidates = [r for r in by_mp if r["path"] == path]
        best_miss = min(candidates, key=lambda r: r["miss_mean"])
        best_time = min(candidates, key=lambda r: r["itime_mean"])
        print(
            f"  {path:16s} best-miss: {best_miss['method']:14s} "
            f"({best_miss['miss_mean']:.3f} m)   "
            f"best-time: {best_time['method']:14s} ({best_time['itime_mean']:.2f} s)"
        )
        winner_counts[best_miss["method"]] = winner_counts.get(best_miss["method"], 0) + 1

    # --- overall (combine every path/speed/seed) -------------------------
    by_method = aggregate_by(rows, ("method",))
    by_method.sort(key=lambda r: r["miss_mean"])
    print("\n=== OVERALL (all paths+speeds+seeds pooled) ===")
    print_table(
        by_method,
        ["method", "n", "miss_mean", "miss_median", "miss_p90", "itime_mean", "coverage"],
        lambda r: (
            r["method"], str(r["n"]), f"{r['miss_mean']:.3f}", f"{r['miss_median']:.3f}",
            f"{r['miss_p90']:.3f}", f"{r['itime_mean']:.2f}", f"{r['coverage_mean']:.2f}",
        ),
    )
    overall_best_miss = by_method[0]["method"]
    overall_best_time = min(by_method, key=lambda r: r["itime_mean"])["method"]

    # --- small gain sweep on the top methods ------------------------------
    top2 = [r["method"] for r in by_method[:2]]
    print(f"\n[guidance_lab] gain-sweeping top methods by overall miss: {top2}")
    t0 = time.perf_counter()
    gain_rows = run_gain_sweep(top2, paths, n_seeds=min(n_seeds, 10), two_stage=args.two_stage)
    elapsed = time.perf_counter() - t0
    print(f"[guidance_lab] gain sweep done: {len(gain_rows)} runs in {elapsed:.2f}s")

    by_gain = aggregate_by(gain_rows, ("method", "N", "V_CLOSE"))
    print("\n=== gain sweep (pooled across paths, speed=5 m/s) ===")
    for method in top2:
        cands = [r for r in by_gain if r["method"] == method]
        cands.sort(key=lambda r: r["miss_mean"])
        best = cands[0]
        print(
            f"  {method:14s} best gains: N={best.get('N')} V_CLOSE={best.get('V_CLOSE')} "
            f"-> miss_mean={best['miss_mean']:.3f} m, itime_mean={best['itime_mean']:.2f} s"
        )
        print_table(
            cands[:5],
            ["method", "N", "V_CLOSE", "n", "miss_mean", "itime_mean"],
            lambda r: (
                r["method"], str(r.get("N")), str(r.get("V_CLOSE")), str(r["n"]),
                f"{r['miss_mean']:.3f}", f"{r['itime_mean']:.2f}",
            ),
        )

    # --- CSV + plots -------------------------------------------------------
    all_rows = rows + gain_rows
    write_csv(all_rows, csv_path)
    print(f"\n[guidance_lab] CSV written: {csv_path} ({len(all_rows)} rows)")

    if not args.no_plots:
        make_plots(rows, methods, paths, plot_prefix)

    # --- final recommendation ----------------------------------------------
    best_method_entry = by_method[0]
    best_gain_entry = None
    if overall_best_miss in top2:
        cands = [r for r in by_gain if r["method"] == overall_best_miss]
        if cands:
            best_gain_entry = min(cands, key=lambda r: r["miss_mean"])

    print("\n" + "=" * 72)
    print("GUIDANCE LAB RECOMMENDATION (design-time hypothesis -- NOT a Gazebo gate)")
    print("=" * 72)
    print(f"  Minimizes miss distance overall : {overall_best_miss} "
          f"(mean {best_method_entry['miss_mean']:.3f} m, "
          f"median {best_method_entry['miss_median']:.3f} m)")
    print(f"  Minimizes intercept time overall: {overall_best_time}")
    if best_gain_entry is not None:
        print(
            f"  Recommended gains for {overall_best_miss}: "
            f"N={best_gain_entry.get('N')} V_CLOSE={best_gain_entry.get('V_CLOSE')} "
            f"(gain-sweep mean miss {best_gain_entry['miss_mean']:.3f} m)"
        )
    print(f"  Per-path winners (by miss): " + ", ".join(
        f"{p}->{r['method']}" for p in paths_seen
        for r in [min([x for x in by_mp if x['path'] == p], key=lambda x: x['miss_mean'])]
    ))
    print(f"  CSV: {csv_path}")
    print(
        "  REMINDER: this is a kinematic surrogate (see module docstring's "
        "modeling assumptions) -- the recommended method+gains must still be "
        "validated by an actual scripted Gazebo gate before it goes in "
        "docs/decisions.md as a real result."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
