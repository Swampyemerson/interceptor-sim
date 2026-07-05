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
# Bearing/range noise raised from the ORIGINAL (ADR-0011) lab's 0.5 deg /
# 2% (empirically found, during the 2nd-addendum calibration pass, to be
# far too clean: a monocular AprilTag pose estimate genuinely degrades with
# oblique incidence -- ADR-0003's own risk note, and confirmed by the real
# Gazebo lambda_dot/rdot_hat traces, e.g. m4_intercept_pronav_...T044945Z.csv,
# which show much larger tick-to-tick variance than a 0.5 deg model would
# produce). These larger numbers are what let the LOS-rate/closing-rate
# estimate stay noisy enough to reproduce Gazebo's actual convergence
# behavior (see the module docstring's calibration note).
SIGMA_BEARING_DEG = 6.0
RANGE_NOISE_FRAC = 0.10
# Dropout is EDGE-WEIGHTED (ADR-0011 2nd-addendum calibration pass): DROPOUT_P
# is the rate right at boresight center, DROPOUT_P_EDGE is the rate once the
# true bearing sits right at the FOV edge, ramped by DROPOUT_EDGE_EXPONENT
# (an exponent < 1 makes the ramp STEEP even at modest angles -- matches the
# real bearing trace, where dropouts start well before the FOV edge, not
# only right at it). Tuned (empirical search, ADR-0011 2nd addendum) so a
# 3-4 m/s crosser's engagement detection_coverage lands ~0.7-0.8 and a 6 m/s
# crosser's drops further (~0.55-0.65) -- in the ballpark of the real
# Gazebo CSVs (m4_intercept_pronav_...T044945Z.csv (4 m/s): 0.67;
# ...T044458Z.csv (6 m/s): 0.36 -- the lab does not get as low as the real
# 6 m/s number; see the module docstring's calibration honesty note).
DROPOUT_P = 0.05
DROPOUT_P_EDGE = 0.99
DROPOUT_EDGE_EXPONENT = 0.4
# Camera half field-of-view (mirrors m4_intercept.py's own comment: "nominal
# +-50 deg half-FOV"; narrowed to 40 deg here -- empirical calibration
# search found the real detector effectively loses the tag somewhat inside
# the nominal hardware FOV, consistent with ADR-0003's oblique-incidence
# risk). A detection can only fire when the TRUE bearing is within this of
# the vehicle's own (slewing) boresight -- see FOV/heading model in the
# Sensor class docstring.
FOV_HALF_DEG_DEFAULT = 40.0
# Max yaw slew rate the vehicle's own boresight can track at (mirrors
# m4_intercept.py's YAWSPEED_MAX_DEG_S=60, tuned down slightly to 45 in this
# calibration pass -- the real PX4 attitude loop's ACHIEVED yaw rate while
# also translating fast is plausibly below the commanded cap). This is THE
# mechanism that lets a fast crosser outrun the yaw and walk off-boresight.
YAW_RATE_MAX_DEG_S_DEFAULT = 45.0
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
# BREAKOFF: mirrors m4_intercept.py's BREAKOFF_ARM_RANGE_M / RANGE_INCREASES
# -- once inside BREAKOFF_ARM_RANGE_M, `BREAKOFF_RANGE_INCREASES` consecutive
# ticks of TRUE range increasing means closest-point-of-approach has passed;
# the simulate() loop stops there. THIS MATTERS MORE THAN IT LOOKS: every
# guidance law's "frozen" terminal coast (see TERMINAL_FREEZE_RANGE_M) holds
# its last command FOREVER once triggered (no un-freeze) -- without a
# breakoff stop, that permanent ballistic coast can fly back through a
# close pass PURELY BY GEOMETRIC LUCK long after the real intercept window
# closed, which is exactly what silently inflated the ORIGINAL (pre-FOV)
# lab's fast-crosser miss numbers (a 6 m/s "uncatchable" run was scoring
# ~0.3-0.6 m from a lucky post-freeze flyby, not a real intercept -- caught
# empirically during the ADR-0011 2nd-addendum calibration pass). Using
# TRUE range for this stopping decision is a harness/SCORING construct only
# (mirrors min_range/t_min_range, both already ground-truth) -- it is never
# fed to any guidance law, so the honesty boundary is unchanged.
BREAKOFF_ARM_RANGE_M = 4.0
BREAKOFF_RANGE_INCREASES = 3


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def wrap_pi(angle):
    """Wrap an angle (radians) into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# =========================================================================
# TRACKING-REFINEMENT candidates (research task, see docs/decisions.md ADR
# addendum for citations). Every helper/param introduced below is OPT-IN --
# the baseline (all-defaults) numeric behavior is unchanged; see the
# baseline-reproduction check in this task's report. Do NOT resurrect
# ADR-0010's rejected APN / in-flight-adaptive-N as "tracking" changes -- the
# additions here are FILTER gain/gating/latency mechanics, not guidance-law
# augmentations, and are scoped to the target TRACK, not the pro-nav law.
# =========================================================================
def kalata_alpha_beta(sigma_process, sigma_meas, dt):
    """Kalata's tracking-index-derived steady-state alpha-beta gains (T.P.
    Kalata, 1984, "The Tracking Index: A Generalized Parameter for
    alpha-beta and alpha-beta-gamma Target Trackers", IEEE Trans. Aerosp.
    Electron. Syst. AES-20(2):174-182 -- see also the "Alpha beta filter"
    Wikipedia summary of the closed-form relation, which independently
    matches the same formulas). Given an assumed target-maneuver-noise std
    `sigma_process` (units = the tracked quantity's 2nd derivative, e.g.
    rad/s^2 for an angle channel or m/s^2 for a range/position channel), a
    measurement-noise std `sigma_meas` (units = the tracked quantity itself),
    and the ACTUAL sample interval `dt` since the last correction (this is
    the point: recompute the gains at the REAL interval, which is why this
    filter needs no separate "dropout handling" -- a longer gap after a
    dropout produces a larger tracking index and hence LARGER alpha/beta,
    correctly trusting the next real measurement more because the
    prediction has had longer to drift; a principled, non-estimated
    fading-memory response, not a new adaptive-estimator failure mode),
    returns the (alpha, beta) pair a steady-state Kalman filter for a
    constant-velocity model would converge to.

    lambda_idx = sigma_process * dt^2 / sigma_meas
    r          = (4 + lambda_idx - sqrt(8*lambda_idx + lambda_idx^2)) / 4
    alpha      = 1 - r^2
    beta       = 2*(2 - alpha) - 4*sqrt(1 - alpha)
    """
    dt = max(dt, 1e-3)
    sigma_meas = max(sigma_meas, 1e-9)
    lambda_idx = sigma_process * dt * dt / sigma_meas
    r = (4.0 + lambda_idx - math.sqrt(8.0 * lambda_idx + lambda_idx * lambda_idx)) / 4.0
    alpha = clamp(1.0 - r * r, 0.0, 0.999)
    beta = 2.0 * (2.0 - alpha) - 4.0 * math.sqrt(max(0.0, 1.0 - alpha))
    return alpha, beta


def range_gate_ok(predicted_range, meas_range, k, sigma_frac=RANGE_NOISE_FRAC, sigma_floor_m=0.3):
    """Candidate-4 camera range-outlier gate: is `meas_range` within `k`
    standard deviations of `predicted_range`? Sigma is modeled as scaling
    with the current range estimate (`sigma_frac * predicted_range`,
    floored at `sigma_floor_m` so it doesn't collapse to ~0 near closest
    approach) -- the same range-noise model the sensor itself uses
    (RANGE_NOISE_FRAC), so this is "know your own sensor spec", not reading
    ground truth. Returns True (pass) whenever there is nothing to gate
    against yet (`predicted_range is None`) or gating is disabled (`k is
    None`) -- both are baseline-preserving no-ops."""
    if k is None or predicted_range is None:
        return True
    sigma = max(sigma_floor_m, sigma_frac * predicted_range)
    return abs(meas_range - predicted_range) <= k * sigma


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
    measurement that only makes sense mod 2*pi.

    KALATA MODE (candidate 2, opt-in): pass `kalata_sigma_process` (not
    None) to IGNORE the fixed `alpha`/`beta` and instead recompute them at
    every correct() from `kalata_alpha_beta(kalata_sigma_process,
    kalata_sigma_meas, dt_since)` using the ACTUAL elapsed time since the
    last correction -- see that function's docstring for why this is the
    principled fix for irregular update intervals/dropouts, not a new
    adaptive-estimator failure mode. Default (`kalata_sigma_process=None`)
    is the untouched, baseline fixed-gain behavior."""

    def __init__(self, alpha, beta, angular=False, kalata_sigma_process=None, kalata_sigma_meas=None):
        self.alpha = alpha
        self.beta = beta
        self.angular = angular
        self.kalata_sigma_process = kalata_sigma_process
        self.kalata_sigma_meas = kalata_sigma_meas
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
        if self.kalata_sigma_process is not None:
            alpha, beta = kalata_alpha_beta(self.kalata_sigma_process, self.kalata_sigma_meas, dt_since)
        else:
            alpha, beta = self.alpha, self.beta
        self.x_hat += alpha * residual
        self.xdot_hat += beta * residual / dt_since
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

    def __init__(self, alpha=0.6, beta=0.2, accel_ema=0.3, accel_warmup_corrections=6,
                 kalata_sigma_process=None, kalata_sigma_meas=None,
                 range_gate_k=None, range_gate_sigma_frac=RANGE_NOISE_FRAC):
        self.fx = AlphaBetaFilter(alpha, beta, kalata_sigma_process=kalata_sigma_process,
                                   kalata_sigma_meas=kalata_sigma_meas)
        self.fy = AlphaBetaFilter(alpha, beta, kalata_sigma_process=kalata_sigma_process,
                                   kalata_sigma_meas=kalata_sigma_meas)
        # Candidate 4 (opt-in, range_gate_k=None -> disabled): reject a
        # camera measurement whose implied range differs too much from this
        # tracker's own current range estimate before letting it correct
        # fx/fy at all -- see range_gate_ok().
        self.range_gate_k = range_gate_k
        self.range_gate_sigma_frac = range_gate_sigma_frac
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

    def correct_full(self, meas, own_pos, t, latency_compensate=False):
        """Unified entry point used by PIP/APN/PNPlusLead/TwoStageDash so
        the caller does not need to branch on tracker kind (TargetTracker
        vs KalmanTracker share this call signature). With
        latency_compensate=False and no range gate configured (both
        defaults), this reduces to EXACTLY the original inline
        'abs_pos = own_pos + meas.rel_xy; tracker.correct(abs_pos, t)' --
        baseline-preserving by construction.

        latency_compensate=True (candidate 3): a cue measurement is
        delivered `meas.age_s` seconds after it was actually sampled (fixed,
        known latency -- see Measurement.age_s). Advance its implied
        absolute position forward along the CURRENT velocity estimate by
        that known latency before correcting -- the standard fix for a
        measurement with known, bounded out-of-sequence delay when you are
        not maintaining a full state history buffer for exact retrodiction.
        """
        if meas.source == "camera" and self.range_gate_k is not None and self.pos_hat is not None:
            predicted_range = math.hypot(self.pos_hat[0] - own_pos[0], self.pos_hat[1] - own_pos[1])
            if not range_gate_ok(predicted_range, meas.range_m, self.range_gate_k, self.range_gate_sigma_frac):
                return  # reject outlier -- treat this tick as if no detection arrived
        abs_pos = (own_pos[0] + meas.rel_xy[0], own_pos[1] + meas.rel_xy[1])
        if latency_compensate and meas.source == "cue" and meas.age_s > 0.0:
            vel = self.vel_hat
            if vel is not None:
                abs_pos = (abs_pos[0] + vel[0] * meas.age_s, abs_pos[1] + vel[1] * meas.age_s)
        self.correct(abs_pos, t)

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


# Standard chi-square critical values for a 2-DoF innovation gate (position
# innovation is 2D: x,y). From any chi-square table, P(chi2_2 <= v) = p:
#   p=0.95 -> 5.991, p=0.99 -> 9.210, p=0.997 -> 11.83 (~3-sigma equivalent).
# Default gate uses the 99% value (Bar-Shalom, Li & Kirubarajan, "Estimation
# with Applications to Tracking and Navigation", Ch. 6 -- validation
# gating/NIS test); see also the "gating" note in this task's research pass.
CHI2_99_DOF2 = 9.210


class KalmanTracker:
    """Candidate 1: hand-rolled 4-state (x, y, vx, vy) constant-velocity
    Kalman filter on the target's ABSOLUTE position, with:
      (a) RANGE-SCALED measurement noise for camera detections -- bearing
          noise (a fixed ANGLE std) becomes a CROSS-RANGE POSITION std that
          scales with range (sigma_cross = range * sigma_bearing_rad), while
          range noise stays a along-LOS position std (sigma_range = range *
          range_noise_frac); both are rotated from the (range, cross-range)
          frame into world (x, y) by the measured bearing angle to build a
          proper (non-isotropic, non-constant) 2x2 measurement covariance R
          -- the "converted measurement" approach standard in bearing-only/
          range-bearing tracking (Lerro & Bar-Shalom-style conversion; see
          this task's research notes). Cue measurements get an isotropic R
          (the mocked ground cue's noise IS isotropic by construction, ADR-
          0010 decision #4).
      (b) a chi-square (NIS) innovation gate (candidate 4's principled
          multivariate generalization): reject any measurement whose
          Mahalanobis distance from the predicted position exceeds
          `chi2_gate` (default the 2-DoF 99% critical value) -- this
          SUPERSEDES a separate scalar range gate for Kalman-tracked laws
          (a bad pose solve corrupts range AND bearing jointly; the 2D
          Mahalanobis test catches both at once, not just range).
    Uses ONLY numpy (already a dependency) -- no scipy.

    Interface-compatible with TargetTracker's predict()/pos_hat/vel_hat/
    correct_full() so callers (PIP, TwoStageDash) do not need to branch on
    tracker kind.
    """

    H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])

    def __init__(self, q_accel_sigma=2.0, sigma_bearing_rad=math.radians(SIGMA_BEARING_DEG),
                 range_noise_frac=RANGE_NOISE_FRAC, cue_sigma_m=CUE_SIGMA_M,
                 chi2_gate=CHI2_99_DOF2, range_sigma_floor_m=0.3, cross_sigma_floor_m=0.1,
                 init_vel_var=25.0):
        self.q_accel_var = q_accel_sigma * q_accel_sigma
        self.sigma_bearing_rad = sigma_bearing_rad
        self.range_noise_frac = range_noise_frac
        self.cue_sigma_m = cue_sigma_m
        self.chi2_gate = chi2_gate
        self.range_sigma_floor_m = range_sigma_floor_m
        self.cross_sigma_floor_m = cross_sigma_floor_m
        self.init_vel_var = init_vel_var
        self.x = None       # np.array shape (4,): [x, y, vx, vy]
        self.P = None        # np.array shape (4, 4)

    @property
    def initialized(self):
        return self.x is not None

    def predict(self, dt):
        if self.x is None:
            return
        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        # Discrete white-noise-acceleration process model (Bar-Shalom et
        # al., "Estimation with Applications to Tracking and Navigation",
        # Eq. 6.3.2-2), one q per axis, axes uncorrelated.
        q = self.q_accel_var
        Qb = q * np.array([[dt ** 4 / 4.0, dt ** 3 / 2.0], [dt ** 3 / 2.0, dt ** 2]])
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = Qb
        Q[np.ix_([1, 3], [1, 3])] = Qb
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def _measurement_R_and_z(self, meas, own_pos):
        z = np.array([own_pos[0] + meas.rel_xy[0], own_pos[1] + meas.rel_xy[1]])
        if meas.source == "camera":
            r = max(meas.range_m, 1e-3)
            sigma_r = max(self.range_sigma_floor_m, self.range_noise_frac * r)
            sigma_c = max(self.cross_sigma_floor_m, r * self.sigma_bearing_rad)
            R_local = np.diag([sigma_r ** 2, sigma_c ** 2])
            b = meas.bearing_rad
            c, s = math.cos(b), math.sin(b)
            Rot = np.array([[c, -s], [s, c]])
            R = Rot @ R_local @ Rot.T
        else:  # "cue" -- isotropic mocked-ground-sensor noise (ADR-0010 #4)
            R = np.diag([self.cue_sigma_m ** 2, self.cue_sigma_m ** 2])
        return z, R

    def correct_full(self, meas, own_pos, t, latency_compensate=False):
        """See TargetTracker.correct_full's docstring for the shared
        latency_compensate semantics (candidate 3)."""
        z, R = self._measurement_R_and_z(meas, own_pos)
        if latency_compensate and meas.source == "cue" and meas.age_s > 0.0 and self.x is not None:
            z = z + self.x[2:4] * meas.age_s

        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            self.P = np.diag([R[0, 0], R[1, 1], self.init_vel_var, self.init_vel_var])
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        d2 = float(y @ np.linalg.solve(S, y))
        if self.chi2_gate is not None and d2 > self.chi2_gate:
            return  # reject outlier (candidate 4, multivariate form)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def pos_hat(self):
        return None if self.x is None else (float(self.x[0]), float(self.x[1]))

    @property
    def vel_hat(self):
        return None if self.x is None else (float(self.x[2]), float(self.x[3]))


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
    __slots__ = ("t", "rel_xy", "range_m", "bearing_rad", "source", "age_s")

    def __init__(self, t, rel_xy, range_m, bearing_rad, source, age_s=0.0):
        self.t = t
        self.rel_xy = rel_xy
        self.range_m = range_m
        self.bearing_rad = bearing_rad
        self.source = source  # "camera" | "cue"
        # age_s: KNOWN elapsed time between when this measurement was
        # SAMPLED and when it was delivered (candidate-3 latency
        # compensation input). 0.0 for the camera (no modeled latency);
        # cue_latency_s for the cue (fixed, known by construction -- see
        # Sensor.tick). NOT an estimate -- this is the same "known sensor
        # spec" honesty boundary as range_gate_ok's sigma_frac.
        self.age_s = age_s


class Sensor:
    """The interceptor's ONLY window onto the target -- mirrors the camera
    honesty boundary (GOALS.md / ADR-0008 lineage): guidance never sees
    ground truth, only these measurements.

    Camera: valid inside det_range AND inside the FOV (see the own_heading
    model below), ~cam_hz update rate, Gaussian bearing/range noise, and an
    EDGE-WEIGHTED dropout probability that rises toward the FOV edge.
    External cue (optional, --two-stage): valid beyond handoff_range,
    ~cue_hz, Gaussian ABSOLUTE-position noise, fixed latency (delivered
    `cue_latency_s` after the sample was taken), and -- unlike the camera --
    NO field-of-view limit at all (it is an external/ground sensor watching
    the engagement from outside, not the interceptor's own onboard camera).

    OWN-HEADING / FOV MODEL (ADR-0011 2nd-addendum calibration pass -- THE
    piece the original lab lacked): own_heading is a simple proxy for where
    the interceptor's camera boresight points. It slews toward
    `desired_heading` at up to `yaw_rate_max` (mirrors m4_intercept.py's
    YAWSPEED_MAX_DEG_S rate-limited yaw-centering law). `desired_heading`
    itself is only updated on a FRESH camera detection -- set to that
    detection's (noisy) world-frame bearing, mirroring the real controller's
    absolute-yaw-setpoint law "yaw_deg = psi + beta" (own yaw + measured
    bearing IS the tag's world-frame bearing, up to sensor noise). Between
    detections the boresight just keeps flying toward the LAST thing it
    saw -- it does not clairvoyantly re-aim at the target's current true
    position. This is exactly what lets a fast crosser's true LOS run away
    from the boresight faster than `yaw_rate_max` can chase it, walking the
    tag out past `fov_half` -- the real dropout mechanism confirmed in
    m4_intercept_pronav_...T044945Z.csv's bearing trace (bearing climbs
    from ~0 to ~-40 deg over ~10 detections as coverage collapses, right as
    the FOV edge is approached).
    """

    def __init__(
        self, rng, det_range=DET_RANGE_DEFAULT_M, handoff_range=HANDOFF_RANGE_DEFAULT_M,
        cam_hz=CAM_HZ, cue_hz=CUE_HZ, sigma_bearing_deg=SIGMA_BEARING_DEG,
        range_noise_frac=RANGE_NOISE_FRAC, dropout_p=DROPOUT_P,
        dropout_p_edge=DROPOUT_P_EDGE, dropout_edge_exponent=DROPOUT_EDGE_EXPONENT,
        fov_half_deg=FOV_HALF_DEG_DEFAULT, yaw_rate_max_deg_s=YAW_RATE_MAX_DEG_S_DEFAULT,
        cue_sigma_m=CUE_SIGMA_M, cue_latency_s=CUE_LATENCY_S, initial_heading=0.0,
    ):
        self.rng = rng
        self.det_range = det_range
        self.handoff_range = handoff_range
        self.cam_period = 1.0 / cam_hz
        self.cue_period = 1.0 / cue_hz
        self.sigma_bearing = math.radians(sigma_bearing_deg)
        self.range_noise_frac = range_noise_frac
        self.dropout_p = dropout_p
        self.dropout_p_edge = dropout_p_edge
        self.dropout_edge_exponent = dropout_edge_exponent
        self.fov_half = math.radians(fov_half_deg)
        self.yaw_rate_max = math.radians(yaw_rate_max_deg_s)
        self.cue_sigma = cue_sigma_m
        self.cue_latency = cue_latency_s
        self._cam_timer = 0.0
        self._cue_timer = 0.0
        self._cue_pending = []  # list of (deliver_t, x, y)
        # own_heading/desired_heading start centered on the target -- mirrors
        # the real engagement's ACQUIRE phase (hover + yaw-center on the
        # tag) already having completed BEFORE the ENGAGE clock this lab
        # simulates starts ticking (see simulate()'s initial_heading calc).
        self.own_heading = initial_heading
        self.desired_heading = initial_heading

    def tick(self, t, dt, own_pos, target_pos, two_stage):
        rel = (target_pos[0] - own_pos[0], target_pos[1] - own_pos[1])
        true_range = math.hypot(rel[0], rel[1])
        true_bearing = math.atan2(rel[1], rel[0])

        # Slew the boresight toward the last commanded heading at the yaw
        # rate limit -- see class docstring.
        max_step = self.yaw_rate_max * dt
        dh = clamp(wrap_pi(self.desired_heading - self.own_heading), -max_step, max_step)
        self.own_heading = wrap_pi(self.own_heading + dh)
        boresight_err = wrap_pi(true_bearing - self.own_heading)

        self._cam_timer += dt
        cam_meas = None
        if self._cam_timer >= self.cam_period:
            self._cam_timer -= self.cam_period
            in_fov = abs(boresight_err) < self.fov_half
            if true_range < self.det_range and in_fov:
                edge_frac = clamp(abs(boresight_err) / self.fov_half, 0.0, 1.0)
                p_drop = self.dropout_p + (self.dropout_p_edge - self.dropout_p) * (
                    edge_frac ** self.dropout_edge_exponent
                )
                if self.rng.random() >= p_drop:
                    range_n = true_range * (1.0 + self.rng.normal(0.0, self.range_noise_frac))
                    bearing_n = true_bearing + self.rng.normal(0.0, self.sigma_bearing)
                    rel_n = (range_n * math.cos(bearing_n), range_n * math.sin(bearing_n))
                    cam_meas = Measurement(t, rel_n, range_n, bearing_n, "camera")
                    # Commanded absolute yaw = psi + measured beta, which IS
                    # (up to sensor noise) the tag's world-frame bearing --
                    # only updates here, on a fresh detection (see docstring).
                    self.desired_heading = bearing_n

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
                cue_meas = Measurement(t, rel_n, r, b, "cue", age_s=self.cue_latency)

        # Camera takes priority in the (rare, boundary-tick) case both fire
        # this tick -- guidance uses the cue only in the CUE phase and the
        # camera only in TERMINAL (ADR-0010 decision #5); with the default
        # equal det_range/handoff_range this collision is a non-event.
        return cam_meas if cam_meas is not None else cue_meas


# =========================================================================
# Guidance law library
# =========================================================================
def compute_v_close(r_hat, v_close_runin, v_close_term, throttle_range, freeze_range):
    """Two-speed along-LOS closing law: fast run-in (`v_close_runin`) while
    still far, linearly throttled down to `v_close_term` as r_hat falls
    through `throttle_range` to `freeze_range`, held at `v_close_term`
    inside that. Mirrors m4_intercept.py's compute_v_close() under --fpv
    EXACTLY (same two numbers, same throttle band) -- this is one of the
    two changes (the other being the FOV/dropout sensor model) needed for
    this lab to reproduce the FPV-profile Gazebo miss numbers (ADR-0011 2nd
    addendum): the original (ADR-0011) lab used one flat V_CLOSE, which
    under-modeled both the aggressive run-in AND the tamed terminal phase
    that the real gain search actually validated in Gazebo."""
    if r_hat is None or r_hat >= throttle_range:
        return v_close_runin
    if r_hat <= freeze_range:
        return v_close_term
    frac = (r_hat - freeze_range) / (throttle_range - freeze_range)
    return v_close_term + frac * (v_close_runin - v_close_term)


# FPV-profile-calibrated defaults (ADR-0011 2nd addendum / m4_intercept.py's
# FPV dict): N=5 (N_PRONAV), two-speed closing (9.0 run-in / 5.5 terminal,
# throttled from r_hat=5.0 down to the 3.5 m freeze range), VC_FLOOR=4.5,
# V_PERP_MAX=8, V_TOTAL_MAX=13, BETA_RANGE=0.45 -- these are the SAME
# numbers that were actually flown and validated in Gazebo, not the
# original lab's generic guesses.
DEFAULT_PN_PARAMS = dict(
    N=5.0, V_CLOSE=5.5, V_CLOSE_RUNIN=9.0, THROTTLE_RANGE=5.0,
    VC_FLOOR=4.5, V_PERP_MAX=8.0, V_TOTAL_MAX=13.0,
    ALPHA=0.5, BETA_LAMBDA=0.30, BETA_RANGE=0.45,
    TERMINAL_FREEZE_RANGE=TERMINAL_FREEZE_RANGE_M,
    # --- tracking-refinement opt-ins (all None = baseline-preserving; see
    # module's tracking-candidates block and _build_lambda_range_filters/
    # _gate_camera_meas) ---
    RANGE_GATE_K=None, RANGE_GATE_SIGMA_FRAC=RANGE_NOISE_FRAC,
    KALATA_LAMBDA_SIGMA_PROCESS_DEG=None, KALATA_LAMBDA_SIGMA_MEAS_DEG=SIGMA_BEARING_DEG,
    KALATA_RANGE_SIGMA_PROCESS=None, KALATA_RANGE_SIGMA_MEAS_M=0.5,
)
DEFAULT_PURSUIT_PARAMS = dict(DEFAULT_PN_PARAMS)  # shares gains w/ pure_pn (fairness, ADR-0009 style)
DEFAULT_APN_PARAMS = dict(DEFAULT_PN_PARAMS, TRACK_ALPHA=0.6, TRACK_BETA=0.2, ACCEL_EMA=0.3)
DEFAULT_PN_PLUS_LEAD_PARAMS = dict(DEFAULT_PN_PARAMS, TRACK_ALPHA=0.6, TRACK_BETA=0.2)
DEFAULT_PIP_PARAMS = dict(
    V_CLOSE=5.5, V_CLOSE_RUNIN=9.0, THROTTLE_RANGE=5.0,
    MAX_LEAD_S=3.0,  # FPV PIP_MAX_LEAD_S (m4_intercept.py) -- was 6.0
    TERMINAL_FREEZE_RANGE=TERMINAL_FREEZE_RANGE_M,  # shares the FPV freeze range w/ pure_pn (was hardcoded 0.5)
    TRACK_ALPHA=0.6, TRACK_BETA=0.2,
    # --- tracking-refinement opt-ins (all None/False/"alphabeta" =
    # baseline-preserving defaults; see module's tracking-candidates block) ---
    TRACKER_KIND="alphabeta",       # "alphabeta" (baseline TargetTracker) | "kalman" (candidate 1)
    LATENCY_COMPENSATE=False,       # candidate 3 (only affects cue-sourced meas, i.e. two_stage)
    RANGE_GATE_K=None,               # candidate 4, only used when TRACKER_KIND="alphabeta"
    RANGE_GATE_SIGMA_FRAC=RANGE_NOISE_FRAC,
    KALMAN_Q_ACCEL=2.0, KALMAN_CHI2_GATE=CHI2_99_DOF2,
    KALMAN_SIGMA_BEARING_DEG=SIGMA_BEARING_DEG, KALMAN_RANGE_NOISE_FRAC=RANGE_NOISE_FRAC,
    KALMAN_CUE_SIGMA_M=CUE_SIGMA_M,
    # candidate 2 for the TargetTracker's own x/y channels (only used when
    # TRACKER_KIND="alphabeta"; None -> fixed TRACK_ALPHA/TRACK_BETA, baseline)
    KALATA_POS_SIGMA_PROCESS=None, KALATA_POS_SIGMA_MEAS_M=1.0,
)


def _build_target_tracker(p):
    """Shared TargetTracker/KalmanTracker construction for PIP/TwoStageDash
    (candidate 1's TRACKER_KIND switch). `p["TRACKER_KIND"]` default
    "alphabeta" reproduces the exact original TargetTracker(TRACK_ALPHA,
    TRACK_BETA) construction -- baseline-preserving."""
    kind = p.get("TRACKER_KIND", "alphabeta")
    if kind == "kalman":
        return KalmanTracker(
            q_accel_sigma=p.get("KALMAN_Q_ACCEL", 2.0),
            sigma_bearing_rad=math.radians(p.get("KALMAN_SIGMA_BEARING_DEG", SIGMA_BEARING_DEG)),
            range_noise_frac=p.get("KALMAN_RANGE_NOISE_FRAC", RANGE_NOISE_FRAC),
            cue_sigma_m=p.get("KALMAN_CUE_SIGMA_M", CUE_SIGMA_M),
            chi2_gate=p.get("KALMAN_CHI2_GATE", CHI2_99_DOF2),
        )
    if kind != "alphabeta":
        raise ValueError(f"unknown TRACKER_KIND {kind!r}")
    return TargetTracker(
        p.get("TRACK_ALPHA", 0.6), p.get("TRACK_BETA", 0.2),
        kalata_sigma_process=p.get("KALATA_POS_SIGMA_PROCESS"),
        kalata_sigma_meas=p.get("KALATA_POS_SIGMA_MEAS_M", 1.0),
        range_gate_k=p.get("RANGE_GATE_K"),
        range_gate_sigma_frac=p.get("RANGE_GATE_SIGMA_FRAC", RANGE_NOISE_FRAC),
    )


def _build_lambda_range_filters(p):
    """Shared lambda/range alpha-beta filter construction for
    pursuit/pure_pn/apn/pn_plus_lead. Wires in candidate 2 (Kalata-derived
    gains) when `KALATA_LAMBDA_SIGMA_PROCESS_DEG` / `KALATA_RANGE_SIGMA_PROCESS`
    are provided (both default None -> the original fixed ALPHA/BETA_LAMBDA/
    BETA_RANGE gains, baseline-preserving)."""
    lam_sp_deg = p.get("KALATA_LAMBDA_SIGMA_PROCESS_DEG")
    lam = AlphaBetaFilter(
        p["ALPHA"], p["BETA_LAMBDA"], angular=True,
        kalata_sigma_process=math.radians(lam_sp_deg) if lam_sp_deg is not None else None,
        kalata_sigma_meas=math.radians(p.get("KALATA_LAMBDA_SIGMA_MEAS_DEG", SIGMA_BEARING_DEG)),
    )
    rng_sp = p.get("KALATA_RANGE_SIGMA_PROCESS")
    rng_f = AlphaBetaFilter(
        p["ALPHA"], p["BETA_RANGE"],
        kalata_sigma_process=rng_sp,
        kalata_sigma_meas=p.get("KALATA_RANGE_SIGMA_MEAS_M", 0.5),
    )
    return lam, rng_f


def _gate_camera_meas(meas, rng_f, range_gate_k, range_gate_sigma_frac):
    """Candidate 4 for the scalar (non-Kalman) lambda/range filter path:
    reject (treat as no detection this tick) a fresh camera measurement
    whose range differs from `rng_f`'s current estimate by more than
    `range_gate_k` standard deviations. No-op (returns `meas` unchanged)
    when gating is disabled (`range_gate_k is None`, the default) or there
    is no prior estimate yet to gate against."""
    if (meas is None or meas.source != "camera" or range_gate_k is None
            or not rng_f.initialized):
        return meas
    if range_gate_ok(rng_f.x_hat, meas.range_m, range_gate_k, range_gate_sigma_frac):
        return meas
    return None


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
        self.v_close_runin = p.get("V_CLOSE_RUNIN", self.v_close)
        self.throttle_range = p.get("THROTTLE_RANGE", p["TERMINAL_FREEZE_RANGE"])
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.lam, self.rng_f = _build_lambda_range_filters(p)
        self.range_gate_k = p.get("RANGE_GATE_K")
        self.range_gate_sigma_frac = p.get("RANGE_GATE_SIGMA_FRAC", RANGE_NOISE_FRAC)
        self.frozen = None
        self.last_cmd = (0.0, 0.0)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        meas = _gate_camera_meas(meas, self.rng_f, self.range_gate_k, self.range_gate_sigma_frac)
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
        v_close = compute_v_close(
            r_hat, self.v_close_runin, self.v_close, self.throttle_range, self.freeze_range
        )
        vx = v_close * math.cos(lambda_hat)
        vy = v_close * math.sin(lambda_hat)
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
        self.v_close_runin = p.get("V_CLOSE_RUNIN", self.v_close)
        self.throttle_range = p.get("THROTTLE_RANGE", p["TERMINAL_FREEZE_RANGE"])
        self.vc_floor = p["VC_FLOOR"]
        self.v_perp_max = p["V_PERP_MAX"]
        self.v_total_max = p["V_TOTAL_MAX"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.lam, self.rng_f = _build_lambda_range_filters(p)
        self.range_gate_k = p.get("RANGE_GATE_K")
        self.range_gate_sigma_frac = p.get("RANGE_GATE_SIGMA_FRAC", RANGE_NOISE_FRAC)
        self.v_perp = 0.0
        self.frozen = None
        self.last_cmd = (0.0, 0.0)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        meas = _gate_camera_meas(meas, self.rng_f, self.range_gate_k, self.range_gate_sigma_frac)
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

        v_close = compute_v_close(
            r_hat, self.v_close_runin, self.v_close, self.throttle_range, self.freeze_range
        )
        u = (math.cos(lambda_hat), math.sin(lambda_hat))
        pvec = (-math.sin(lambda_hat), math.cos(lambda_hat))
        vx = v_close * u[0] + self.v_perp * pvec[0]
        vy = v_close * u[1] + self.v_perp * pvec[1]
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
        self.v_close_runin = p.get("V_CLOSE_RUNIN", self.v_close)
        self.throttle_range = p.get("THROTTLE_RANGE", p["TERMINAL_FREEZE_RANGE"])
        self.vc_floor = p["VC_FLOOR"]
        self.v_perp_max = p["V_PERP_MAX"]
        self.v_total_max = p["V_TOTAL_MAX"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.lam, self.rng_f = _build_lambda_range_filters(p)
        self.range_gate_k = p.get("RANGE_GATE_K")
        self.range_gate_sigma_frac = p.get("RANGE_GATE_SIGMA_FRAC", RANGE_NOISE_FRAC)
        self.tracker = TargetTracker(p["TRACK_ALPHA"], p["TRACK_BETA"], p["ACCEL_EMA"])
        self.v_perp = 0.0
        self.frozen = None
        self.last_cmd = (0.0, 0.0)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        self.tracker.predict(dt)
        meas = _gate_camera_meas(meas, self.rng_f, self.range_gate_k, self.range_gate_sigma_frac)
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

        v_close = compute_v_close(
            r_hat, self.v_close_runin, self.v_close, self.throttle_range, self.freeze_range
        )
        u = (math.cos(lambda_hat), math.sin(lambda_hat))
        vx = v_close * u[0] + self.v_perp * pvec[0]
        vy = v_close * u[1] + self.v_perp * pvec[1]
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
        self.v_close_runin = p.get("V_CLOSE_RUNIN", self.v_close)
        self.throttle_range = p.get("THROTTLE_RANGE", p["TERMINAL_FREEZE_RANGE"])
        self.max_lead_s = p["MAX_LEAD_S"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.tracker = _build_target_tracker(p)
        self.latency_compensate = p.get("LATENCY_COMPENSATE", False)
        self.frozen = None
        self.last_cmd = (0.0, 0.0)
        self._have_meas = False

    def step(self, t, dt, own_pos, own_vel, meas):
        self.tracker.predict(dt)
        if meas is not None:
            self.tracker.correct_full(meas, own_pos, t, latency_compensate=self.latency_compensate)
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

        v_close = compute_v_close(
            r_hat, self.v_close_runin, self.v_close, self.throttle_range, self.freeze_range
        )
        vt = self.tracker.vel_hat or (0.0, 0.0)
        t_go = solve_intercept_time(rel, vt, v_close, self.max_lead_s)
        aim = (target_pos[0] + vt[0] * t_go, target_pos[1] + vt[1] * t_go)
        direction = (aim[0] - own_pos[0], aim[1] - own_pos[1])
        dnorm = math.hypot(direction[0], direction[1])
        if dnorm < 1e-6:
            vx, vy = self.last_cmd
        else:
            vx = v_close * direction[0] / dnorm
            vy = v_close * direction[1] / dnorm
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
        self.v_close_runin = p.get("V_CLOSE_RUNIN", self.v_close)
        self.throttle_range = p.get("THROTTLE_RANGE", p["TERMINAL_FREEZE_RANGE"])
        self.vc_floor = p["VC_FLOOR"]
        self.v_perp_max = p["V_PERP_MAX"]
        self.v_total_max = p["V_TOTAL_MAX"]
        self.freeze_range = p["TERMINAL_FREEZE_RANGE"]
        self.lam, self.rng_f = _build_lambda_range_filters(p)
        self.range_gate_k = p.get("RANGE_GATE_K")
        self.range_gate_sigma_frac = p.get("RANGE_GATE_SIGMA_FRAC", RANGE_NOISE_FRAC)
        self.tracker = TargetTracker(p["TRACK_ALPHA"], p["TRACK_BETA"])
        self.v_perp = 0.0
        self.frozen = None
        self.last_cmd = (0.0, 0.0)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        self.tracker.predict(dt)
        meas = _gate_camera_meas(meas, self.rng_f, self.range_gate_k, self.range_gate_sigma_frac)
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

        v_close = compute_v_close(
            r_hat, self.v_close_runin, self.v_close, self.throttle_range, self.freeze_range
        )
        u = (math.cos(lambda_hat), math.sin(lambda_hat))
        pvec = (-math.sin(lambda_hat), math.cos(lambda_hat))
        vx = v_close * u[0] + self.v_perp * pvec[0]
        vy = v_close * u[1] + self.v_perp * pvec[1]
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


DEFAULT_DASH_PARAMS = dict(
    DASH_SPEED=12.0,       # m/s, external-cue-guided closing speed during CUE phase (S2 "running start")
    DASH_MAX_LEAD_S=4.0,
    HANDOFF_RANGE=HANDOFF_RANGE_DEFAULT_M,
    TRACK_ALPHA=0.6, TRACK_BETA=0.2,
    TERMINAL_METHOD="pure_pn",   # which METHODS entry flies the terminal (camera-only) phase
    TERMINAL_PARAMS=None,        # optional override dict for the terminal law's own params/gains
    # --- tracking-refinement opt-ins for the DASH-PHASE tracker (baseline-
    # preserving defaults; the terminal law's own tracker is configured
    # separately via TERMINAL_PARAMS, e.g. {"TRACKER_KIND": "kalman"}) ---
    TRACKER_KIND="alphabeta", LATENCY_COMPENSATE=False,
    RANGE_GATE_K=None, RANGE_GATE_SIGMA_FRAC=RANGE_NOISE_FRAC,
    KALMAN_Q_ACCEL=2.0, KALMAN_CHI2_GATE=CHI2_99_DOF2,
    KALMAN_SIGMA_BEARING_DEG=SIGMA_BEARING_DEG, KALMAN_RANGE_NOISE_FRAC=RANGE_NOISE_FRAC,
    KALMAN_CUE_SIGMA_M=CUE_SIGMA_M,
    KALATA_POS_SIGMA_PROCESS=None, KALATA_POS_SIGMA_MEAS_M=1.0,
)


class TwoStageDash:
    """S2 design (ADR-0011 2nd addendum's S1<->S2 coupling finding): DASH
    toward the external cue's tracked target position+velocity at up to
    `DASH_SPEED` (~12 m/s) while beyond `HANDOFF_RANGE` / before the camera
    has a fresh lock, then HAND OFF to a normal camera-only terminal law
    (default pure_pn -- the Gazebo-validated robust choice) once inside
    `HANDOFF_RANGE` WITH a fresh camera detection. This gives the
    interceptor a running start into the terminal phase instead of
    building closing speed from hover, which is what the hover-start
    (camera_only) config's ~4.7 m 'uncatchable' 6 m/s result says it
    structurally cannot do on its own.

    The dash itself reuses the PIP lead-pursuit solve (solve_intercept_time)
    against the SAME TargetTracker fed by either sensor channel -- the cue's
    10 Hz/0.5 m-sigma/100 ms-latency track is clean enough that a lead
    solve is worth it even at long range; the terminal law gets its own
    independent camera-only lambda/range filters (fed continuously, even
    during the dash, so they are warm -- not cold-started -- the instant
    handoff happens)."""

    NAME = "two_stage_dash"

    def __init__(self, params):
        p = {**DEFAULT_DASH_PARAMS, **(params or {})}
        self.dash_speed = p["DASH_SPEED"]
        self.dash_max_lead_s = p["DASH_MAX_LEAD_S"]
        self.handoff_range = p["HANDOFF_RANGE"]
        terminal_name = p["TERMINAL_METHOD"]
        cls, defaults = METHODS[terminal_name]
        terminal_params = {**defaults, **(p.get("TERMINAL_PARAMS") or {})}
        self.terminal = cls(terminal_params)
        self.tracker = _build_target_tracker(p)
        self.latency_compensate = p.get("LATENCY_COMPENSATE", False)
        self.last_camera_t = None
        self.last_cmd = (0.0, 0.0)
        # Publicly observable (self-derived, NOT ground truth) dash->terminal
        # state -- lets simulate()'s track-quality diagnostic snapshot the
        # tracker's error at the exact tick handoff occurs, without the
        # diagnostic needing to reimplement this law's own handoff logic.
        self.in_terminal = False

    def step(self, t, dt, own_pos, own_vel, meas):
        self.tracker.predict(dt)
        if meas is not None:
            self.tracker.correct_full(meas, own_pos, t, latency_compensate=self.latency_compensate)
            if meas.source == "camera":
                self.last_camera_t = t

        # Always step the terminal law (fed the same `meas`) so ITS filters
        # stay warm through the dash phase -- avoids a cold-start filter the
        # instant handoff happens.
        terminal_cmd = self.terminal.step(t, dt, own_pos, own_vel, meas)

        camera_has_tag = (
            self.last_camera_t is not None and (t - self.last_camera_t) <= MEAS_STALE_S
        )
        pos_hat = self.tracker.pos_hat
        r_hat = None
        if pos_hat is not None:
            r_hat = math.hypot(pos_hat[0] - own_pos[0], pos_hat[1] - own_pos[1])

        in_terminal = camera_has_tag and r_hat is not None and r_hat <= self.handoff_range
        self.in_terminal = in_terminal
        if in_terminal:
            self.last_cmd = terminal_cmd
            return terminal_cmd

        if pos_hat is None:
            return (0.0, 0.0)  # no track yet at all (before the cue's first sample)

        vt = self.tracker.vel_hat or (0.0, 0.0)
        rel = (pos_hat[0] - own_pos[0], pos_hat[1] - own_pos[1])
        t_go = solve_intercept_time(rel, vt, self.dash_speed, self.dash_max_lead_s)
        aim = (pos_hat[0] + vt[0] * t_go, pos_hat[1] + vt[1] * t_go)
        direction = (aim[0] - own_pos[0], aim[1] - own_pos[1])
        dnorm = math.hypot(direction[0], direction[1])
        if dnorm < 1e-6:
            vx, vy = self.last_cmd
        else:
            vx = self.dash_speed * direction[0] / dnorm
            vy = self.dash_speed * direction[1] / dnorm
        self.last_cmd = (vx, vy)
        return (vx, vy)


METHODS = {
    "pursuit": (Pursuit, DEFAULT_PURSUIT_PARAMS),
    "pure_pn": (PurePN, DEFAULT_PN_PARAMS),
    "apn": (APN, DEFAULT_APN_PARAMS),
    "pip": (PIP, DEFAULT_PIP_PARAMS),
    "pn_plus_lead": (PNPlusLead, DEFAULT_PN_PLUS_LEAD_PARAMS),
    "two_stage_dash": (TwoStageDash, DEFAULT_DASH_PARAMS),  # only meaningful with two_stage=True (needs the cue)
}
# Methods that only make sense/exist to be run under the cue channel
# (two_stage=True); excluded from the default camera_only trade study.
TWO_STAGE_ONLY_METHODS = {"two_stage_dash"}


# =========================================================================
# Target path library -- each builder returns (start_pos, vel_fn(t)) and is
# deterministic given the shared per-run RNG (small geometry jitter is
# drawn from it so a Monte-Carlo sweep varies genuine geometry, not just
# sensor noise). Speeds parameterizable ~2-10 m/s. crossing_l2r/r2l's
# geometry now matches m4_intercept.py's/check_s1.sh's ACTUAL FPV
# engagement setup EXACTLY (target start (6.5, -4), tag pre-centered by the
# ACQUIRE phase before this lab's t=0 -- ADR-0011 2nd addendum calibration
# pass), not just "mirrors" it approximately as the original (ADR-0011)
# version did with y0_mag=6.0.
# =========================================================================
def build_crossing(speed, rng, direction=1.0, x0=6.5, y0_mag=4.0, jitter=0.5):
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

    ix = iy = ivx = ivy = 0.0
    # ACQUIRE already happened before this lab's clock starts (m4_intercept.py:
    # hover + yaw-center on the tag BEFORE spawning the mover/engaging) -- so
    # the boresight starts centered on the target's initial position, not at
    # a fixed world heading. See Sensor's own_heading/FOV docstring.
    initial_heading = math.atan2(ty - iy, tx - ix)
    sensor = Sensor(
        rng, det_range=det_range, handoff_range=handoff_range, initial_heading=initial_heading
    )

    min_range = math.hypot(tx - ix, ty - iy)
    t_min_range = 0.0
    first_hit_time = None
    control_effort = 0.0
    n_ticks = 0
    n_locked_ticks = 0
    n_in_range_ticks = 0
    last_meas_t = None
    tag_lost_in_terminal = False
    breakoff_armed = False
    breakoff_increase_streak = 0
    last_true_range = None

    # --- track-quality diagnostics (SCORING-ONLY, same honesty boundary as
    # min_range/breakoff/tag_lost_in_terminal above -- reads tx,ty/vel_fn(t)
    # ground truth ONLY to grade the tracker's OWN internal pos_hat/vel_hat
    # estimate; never fed back into law.step() or any guidance decision).
    # Lets a tracking-candidate comparison show WHY a winner wins (e.g. a
    # lower tracker_vel_err_rms for PIP's TargetTracker/KalmanTracker),
    # for methods that expose a `.tracker` (pip/apn/pn_plus_lead/
    # two_stage_dash); None for methods that don't (pursuit/pure_pn, which
    # track lambda/range directly, not an absolute-position track).
    _pos_err_sq_sum = 0.0
    _vel_err_sq_sum = 0.0
    _track_diag_n = 0
    _handoff_pos_err = None
    _handoff_vel_err = None
    _prev_in_terminal = False

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

        # BREAKOFF (see BREAKOFF_ARM_RANGE_M docstring): CPA has passed --
        # stop here rather than let a permanently-frozen command coast on
        # into a geometrically-lucky (or unlucky) later flyby.
        if true_range < BREAKOFF_ARM_RANGE_M:
            breakoff_armed = True
        if breakoff_armed and last_true_range is not None:
            if true_range > last_true_range:
                breakoff_increase_streak += 1
            else:
                breakoff_increase_streak = 0
            if breakoff_increase_streak >= BREAKOFF_RANGE_INCREASES:
                break
        last_true_range = true_range

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

        tracker = getattr(law, "tracker", None)
        if tracker is not None and tracker.pos_hat is not None:
            true_vt_now = vel_fn(t)
            pos_err = math.hypot(tracker.pos_hat[0] - tx, tracker.pos_hat[1] - ty)
            _pos_err_sq_sum += pos_err * pos_err
            vel_hat_now = tracker.vel_hat
            vel_err = None
            if vel_hat_now is not None:
                vel_err = math.hypot(vel_hat_now[0] - true_vt_now[0], vel_hat_now[1] - true_vt_now[1])
                _vel_err_sq_sum += vel_err * vel_err
            _track_diag_n += 1
            in_terminal_now = getattr(law, "in_terminal", None)
            if in_terminal_now and not _prev_in_terminal:
                _handoff_pos_err = pos_err
                _handoff_vel_err = vel_err
            if in_terminal_now is not None:
                _prev_in_terminal = in_terminal_now

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
    tracker_pos_err_rms = math.sqrt(_pos_err_sq_sum / _track_diag_n) if _track_diag_n else None
    tracker_vel_err_rms = math.sqrt(_vel_err_sq_sum / _track_diag_n) if _track_diag_n else None

    return dict(
        method=method_name, path=path_name, speed=speed, seed=seed, two_stage=two_stage,
        N=params.get("N"), V_CLOSE=params.get("V_CLOSE"),
        dash_speed=params.get("DASH_SPEED"), handoff_range_cfg=params.get("HANDOFF_RANGE"),
        terminal_method=params.get("TERMINAL_METHOD"),
        miss_distance=min_range, intercept_time=intercept_time,
        control_effort=control_effort, detection_coverage=detection_coverage,
        tag_lost_in_terminal=tag_lost_in_terminal,
        # Track-quality diagnostics (candidate research task, ground-truth
        # SCORING-ONLY -- see comment above the accumulators): None for
        # methods without a `.tracker` (pursuit/pure_pn).
        tracker_pos_err_rms=tracker_pos_err_rms, tracker_vel_err_rms=tracker_vel_err_rms,
        handoff_pos_err=_handoff_pos_err, handoff_vel_err=_handoff_vel_err,
    )


# =========================================================================
# Trade-study driver
# =========================================================================
# camera_only methods -- excludes two_stage_dash (only meaningful with the
# cue channel enabled; see TWO_STAGE_ONLY_METHODS).
ALL_METHODS = [m for m in METHODS.keys() if m not in TWO_STAGE_ONLY_METHODS]
ALL_PATHS = list(PATHS.keys())
# Calibration band (ADR-0011 2nd addendum): 3/4/6 m/s are the exact speeds
# validated in Gazebo (0.94 m / 1.6 m / uncatchable-from-hover).
DEFAULT_SPEEDS = (3.0, 4.0, 6.0)
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
        "dash_speed", "handoff_range_cfg", "terminal_method",
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


# =========================================================================
# Calibration check (ADR-0011 2nd addendum): does the upgraded lab
# reproduce the 5 real Gazebo data points at the EXACT validated engagement
# geometry (crossing_l2r, x0=6.5/y0=-4, FPV profile)? This is a direct
# head-to-head against numbers that were actually flown, not a hypothesis.
# =========================================================================
CALIBRATION_TARGETS = [
    # (method, speed, gazebo target description)
    ("pure_pn", 3.0, "0.94 m (clean intercept)"),
    ("pure_pn", 4.0, "~1.6 m"),
    ("pure_pn", 6.0, "uncatchable from hover, min range ~4.7 m"),
    ("pip", 4.0, "~3.0 m (WORSE than pure_pn)"),
]


def run_calibration(n_seeds):
    rows = []
    for method in ("pure_pn", "pip"):
        for speed in (3.0, 4.0, 6.0):
            for seed in range(n_seeds):
                rows.append(
                    simulate(method, None, "crossing_l2r", {"speed": speed}, seed, two_stage=False)
                )
    return rows


def print_calibration(cal_rows):
    agg = {(a["method"], a["speed"]): a for a in aggregate_by(cal_rows, ("method", "speed"))}
    print("\n" + "=" * 78)
    print("CALIBRATION vs Gazebo ground truth (ADR-0011 2nd addendum) -- camera_only, hover start")
    print("=" * 78)
    print(f"{'method':10s} {'speed':>6s} {'lab mean':>9s} {'lab median':>11s} {'lab p90':>8s} "
          f"{'coverage':>9s}   gazebo target")
    for method, speed, target in CALIBRATION_TARGETS:
        a = agg.get((method, speed))
        if a is None:
            print(f"{method:10s} {speed:6.1f}   <no data>")
            continue
        print(
            f"{method:10s} {speed:6.1f} {a['miss_mean']:9.3f} {a['miss_median']:11.3f} "
            f"{a['miss_p90']:8.3f} {a['coverage_mean']:9.2f}   {target}"
        )
    print(
        "(Gazebo coverage during real FPV engagements ranged ~0.36 (6 m/s) to "
        "~0.70 (3-4 m/s); see m4_intercept_pronav/pip_*.csv.)"
    )


# =========================================================================
# S2 dash-config sweep (ADR-0011 2nd addendum's S1<->S2 coupling finding):
# sweep dash_speed x handoff_range x terminal_method at the FPV target band
# (6/8/10 m/s) on a LONGER-standoff geometry so the external cue has real
# dash room before handoff (the calibration geometry's ~7.6 m start range
# is already inside the camera's own det_range -- there is no cue runway
# there by construction). Answers: what speeds become catchable, and what's
# the best S2 config.
# =========================================================================
S2_SPEEDS = (6.0, 8.0, 10.0)
S2_DASH_SPEED_GRID = (10.0, 12.0, 14.0)
S2_HANDOFF_RANGE_GRID = (6.0, 8.0, 10.0)
S2_TERMINAL_METHODS = ("pure_pn", "pip")
S2_PATH_PARAMS = dict(x0=6.5, y0_mag=14.0)  # ~15.4 m initial range -- real dash runway


def run_s2_dash_sweep(n_seeds):
    rows = []
    for speed in S2_SPEEDS:
        for dash_speed in S2_DASH_SPEED_GRID:
            for handoff_range in S2_HANDOFF_RANGE_GRID:
                for terminal_method in S2_TERMINAL_METHODS:
                    method_params = dict(
                        DASH_SPEED=dash_speed, HANDOFF_RANGE=handoff_range,
                        TERMINAL_METHOD=terminal_method,
                    )
                    path_params = dict(S2_PATH_PARAMS, speed=speed, handoff_range=handoff_range)
                    for seed in range(n_seeds):
                        rows.append(
                            simulate(
                                "two_stage_dash", method_params, "crossing_l2r", path_params,
                                seed, two_stage=True,
                            )
                        )
    return rows


def run_s2_baseline(n_seeds):
    """Camera-only, hover-start pure_pn at the SAME long-standoff geometry
    and the S2 target speeds -- the "no dash" comparison point."""
    rows = []
    for speed in S2_SPEEDS:
        for seed in range(n_seeds):
            path_params = dict(S2_PATH_PARAMS, speed=speed)
            rows.append(
                simulate("pure_pn", None, "crossing_l2r", path_params, seed, two_stage=False)
            )
    return rows


def print_s2_results(dash_rows, baseline_rows):
    print("\n" + "=" * 78)
    print("S2 DASH STUDY: two_stage_dash vs camera_only (hover start), long-standoff geometry")
    print("=" * 78)
    base_agg = {a["speed"]: a for a in aggregate_by(baseline_rows, ("speed",))}
    dash_agg = aggregate_by(
        dash_rows, ("speed", "dash_speed", "handoff_range_cfg", "terminal_method")
    )
    best_overall = []
    for speed in S2_SPEEDS:
        cands = [r for r in dash_agg if r["speed"] == speed]
        cands.sort(key=lambda r: r["miss_mean"])
        best = cands[0]
        base = base_agg.get(speed)
        base_miss = base["miss_mean"] if base else float("nan")
        print(
            f"  speed={speed:4.1f} m/s  camera_only(hover) miss={base_miss:6.3f} m   "
            f"BEST two_stage_dash miss={best['miss_mean']:.3f} m  "
            f"(dash_speed={best['dash_speed']}, handoff_range={best['handoff_range_cfg']}, "
            f"terminal={best['terminal_method']}, coverage={best['coverage_mean']:.2f})"
        )
        print("    top 3 configs:")
        print_table(
            cands[:3],
            ["dash_speed", "handoff_range_cfg", "terminal_method", "n", "miss_mean", "coverage"],
            lambda r: (
                str(r["dash_speed"]), str(r["handoff_range_cfg"]), r["terminal_method"],
                str(r["n"]), f"{r['miss_mean']:.3f}", f"{r['coverage_mean']:.2f}",
            ),
        )
        best_overall.append(best)
    return best_overall


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--quick", action="store_true", help="small fast sweep (smoke test)")
    parser.add_argument("--two-stage", action="store_true",
                         help="enable the external-cue CUE phase for the MAIN trade study "
                              "(default: camera-only). The calibration and S2 dash sections "
                              "always run regardless of this flag (camera_only / two_stage "
                              "respectively, per ADR-0011 2nd addendum's 'run both configs' ask).")
    parser.add_argument("--no-plots", action="store_true", help="skip matplotlib plots")
    parser.add_argument("--seeds", type=int, default=None, help="override seed count (main sweep)")
    parser.add_argument("--skip-calibration", action="store_true",
                         help="skip the pure_pn/pip vs Gazebo calibration check")
    parser.add_argument("--skip-s2", action="store_true", help="skip the S2 dash-config sweep")
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
        cal_seeds = 30
        s2_seeds = 5
    else:
        methods = ALL_METHODS
        paths = ALL_PATHS
        speeds = list(DEFAULT_SPEEDS)
        n_seeds = args.seeds or DEFAULT_SEEDS
        cal_seeds = 300
        s2_seeds = 40

    extra_rows = []

    # --- (a) CALIBRATION: does the lab reproduce the 5 Gazebo data points? --
    if not args.skip_calibration:
        print(f"\n[guidance_lab] calibration: 2 methods x 3 speeds x {cal_seeds} seeds "
              f"on the exact Gazebo-validated crossing_l2r geometry...")
        t0 = time.perf_counter()
        cal_rows = run_calibration(cal_seeds)
        print(f"[guidance_lab] calibration done: {len(cal_rows)} runs in "
              f"{time.perf_counter() - t0:.2f}s")
        print_calibration(cal_rows)
        extra_rows += cal_rows

    n_runs = len(methods) * len(paths) * len(speeds) * n_seeds
    print(
        f"\n[guidance_lab] sweep: {len(methods)} methods x {len(paths)} paths x "
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

    # --- (c) S2 DASH STUDY: does an external-cue running start make ---------
    # 6/8/10 m/s catchable, and what's the best dash config? -----------------
    s2_best = None
    if not args.skip_s2:
        n_s2_runs = (
            len(S2_SPEEDS) * len(S2_DASH_SPEED_GRID) * len(S2_HANDOFF_RANGE_GRID)
            * len(S2_TERMINAL_METHODS) * s2_seeds
        )
        print(
            f"\n[guidance_lab] S2 dash sweep: {len(S2_SPEEDS)} speeds x "
            f"{len(S2_DASH_SPEED_GRID)} dash speeds x {len(S2_HANDOFF_RANGE_GRID)} "
            f"handoff ranges x {len(S2_TERMINAL_METHODS)} terminal methods x "
            f"{s2_seeds} seeds = {n_s2_runs} runs, plus a camera_only baseline..."
        )
        t0 = time.perf_counter()
        s2_dash_rows = run_s2_dash_sweep(s2_seeds)
        s2_baseline_rows = run_s2_baseline(s2_seeds)
        print(f"[guidance_lab] S2 sweep done: {len(s2_dash_rows) + len(s2_baseline_rows)} "
              f"runs in {time.perf_counter() - t0:.2f}s")
        s2_best = print_s2_results(s2_dash_rows, s2_baseline_rows)
        extra_rows += s2_dash_rows + s2_baseline_rows

    # --- CSV + plots -------------------------------------------------------
    all_rows = rows + gain_rows + extra_rows
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
    if s2_best:
        print("  S2 dash recommendation (per target speed):")
        for best in s2_best:
            print(
                f"    {best['speed']:.0f} m/s -> dash_speed={best['dash_speed']}, "
                f"handoff_range={best['handoff_range_cfg']}, terminal={best['terminal_method']} "
                f"-> miss_mean={best['miss_mean']:.3f} m"
            )
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
