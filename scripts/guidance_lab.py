#!/usr/bin/env python3
"""Guidance-method trade-study lab: a FAST, pure-Python, point-mass kinematic
Monte-Carlo harness for comparing intercept guidance laws. See docs/goals.md's
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
    headline capability (docs/goals.md: "when the datalink is denied, the
    interceptor's own camera locks the target"). A new `two_stage_dash`
    guidance method (S2 design, ADR-0011 2nd addendum) uses the cue to DASH
    at up to DASH_SPEED while beyond HANDOFF_RANGE, then hands off to a
    normal camera-only terminal law (default pure_pn) once inside
    HANDOFF_RANGE with a fresh camera lock -- the "running start" the
    addendum's S1<->S2 coupling finding calls for.

  - P-6 MID-COURSE FUSION + WARM HANDOFF (ADR-0015 fusion mechanization;
    opt-in via TwoStageDash params FUSE_MIDCOURSE / WARM_HANDOFF /
    FUSE_GAZEBO_LATCH, all default-off = byte-identical baseline): during
    the window where BOTH sources exist (camera detecting AND cue alive,
    i.e. first detection -> the handoff latch), a FusedTrack blends the
    camera bearing (angle-strong; the cue NEVER touches the angle) with the
    cue's range/velocity (range-holding); the cue stays deliverable until
    the one-way LATCH exactly like the real m4_intercept.py --handoff
    pipeline (the legacy lab model killed the cue at handoff_range -- a
    modeling gap); and the latch can WARM-initialize the terminal law's
    filter/rate states from the fused track instead of letting them
    converge from cold. Evaluated by the --p6-fusion driver (fusion x warm
    A/B across perception tiers). See FusedTrack's docstring for the
    mechanization rationale.

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
    is the untouched, baseline fixed-gain behavior.

    RATE CAP (ADR-0014 lever 2, opt-in): pass `rate_cap` (rad/s, not None)
    to clamp `xdot_hat` to +-rate_cap immediately after every correct().
    Because predict() forward-integrates `x_hat += xdot_hat * dt` off this
    SAME stored `xdot_hat`, clamping it here caps BOTH the rate any caller
    reads (e.g. a_cmd = N*Vc*lambda_dot) AND the filter's own forward
    integration in one place -- this is the "cap in both places" half of
    ADR-0014's lever 2 (the singular-as-R->0 lambda_dot fix). Guidance laws
    that instead want to demonstrate the REJECTED "a_cmd-only" variant (cap
    the value read for a_cmd but leave predict()'s integration on the raw,
    uncapped rate -- the ADR-0009-whipsaw-recreating mistake) must NOT set
    this and instead clamp their own local read of `xdot_hat` -- see
    PurePN's `LAMBDA_DOT_CAP_A_CMD_ONLY_DEG_S`. Default None is the
    untouched, baseline uncapped behavior."""

    def __init__(self, alpha, beta, angular=False, kalata_sigma_process=None, kalata_sigma_meas=None,
                 rate_cap=None):
        self.alpha = alpha
        self.beta = beta
        self.angular = angular
        self.kalata_sigma_process = kalata_sigma_process
        self.kalata_sigma_meas = kalata_sigma_meas
        self.rate_cap = rate_cap
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

    def correct(self, meas, t, gain_scale=1.0):
        """`gain_scale` (P-6 fusion, default 1.0 = byte-identical baseline --
        x*1.0 is IEEE-exact) scales BOTH gains for this one correction: how
        FusedTrack folds a LOWER-confidence source (the cue's range) into a
        filter whose nominal gains are tuned for the camera, without
        touching the filter's stored gains. The weight is inverse-variance,
        computed by the caller from KNOWN sensor specs (FusedTrack.update)."""
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
        self.x_hat += alpha * gain_scale * residual
        self.xdot_hat += beta * gain_scale * residual / dt_since
        if self.rate_cap is not None:
            self.xdot_hat = clamp(self.xdot_hat, -self.rate_cap, self.rate_cap)
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
            # Prefer the EMITTED velocity for the OOSM advance when present
            # (it is a cleaner, un-differentiated velocity); else fall back
            # to the filter's own velocity estimate (the original behavior).
            vel = meas.vel_xy if meas.vel_xy is not None else self.vel_hat
            if vel is not None:
                abs_pos = (abs_pos[0] + vel[0] * meas.age_s, abs_pos[1] + vel[1] * meas.age_s)
        self.correct(abs_pos, t)
        # ADR-0015 constraint #2: a cue that EMITS a filtered velocity feeds
        # the velocity channel DIRECTLY (baseline-preserving: meas.vel_xy is
        # None unless CUE_EMIT_VELOCITY is on, so this block never fires in
        # the baseline differentiate-the-position path). This is the whole
        # point of the constraint-2 test -- PIP's lead solve gets a clean
        # velocity instead of one differentiated from a noisy/biased cue
        # position (which the ADR-0015 table calls a ~7 m/s-noise PIP-killer).
        if meas.vel_xy is not None:
            self.fx.xdot_hat = meas.vel_xy[0]
            self.fy.xdot_hat = meas.vel_xy[1]

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

    H_VEL = np.array([[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    def __init__(self, q_accel_sigma=2.0, sigma_bearing_rad=math.radians(SIGMA_BEARING_DEG),
                 range_noise_frac=RANGE_NOISE_FRAC, cue_sigma_m=CUE_SIGMA_M,
                 chi2_gate=CHI2_99_DOF2, range_sigma_floor_m=0.3, cross_sigma_floor_m=0.1,
                 init_vel_var=25.0, cue_vel_sigma_m_s=0.5):
        self.q_accel_var = q_accel_sigma * q_accel_sigma
        self.sigma_bearing_rad = sigma_bearing_rad
        self.range_noise_frac = range_noise_frac
        self.cue_sigma_m = cue_sigma_m
        self.chi2_gate = chi2_gate
        self.range_sigma_floor_m = range_sigma_floor_m
        self.cross_sigma_floor_m = cross_sigma_floor_m
        self.init_vel_var = init_vel_var
        # ADR-0015 constraint #2: measurement std for a cue-EMITTED velocity
        # (only used when meas.vel_xy is present, i.e. CUE_EMIT_VELOCITY on).
        self.cue_vel_sigma_m_s = cue_vel_sigma_m_s
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
            v0 = meas.vel_xy if meas.vel_xy is not None else (0.0, 0.0)
            vvar = (self.cue_vel_sigma_m_s ** 2) if meas.vel_xy is not None else self.init_vel_var
            self.x = np.array([z[0], z[1], v0[0], v0[1]])
            self.P = np.diag([R[0, 0], R[1, 1], vvar, vvar])
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        d2 = float(y @ np.linalg.solve(S, y))
        if self.chi2_gate is not None and d2 > self.chi2_gate:
            return  # reject outlier (candidate 4, multivariate form)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        # ADR-0015 constraint #2: fold in the cue's EMITTED velocity as a
        # direct velocity measurement (H_VEL picks out vx,vy). Baseline-
        # preserving: meas.vel_xy is None unless CUE_EMIT_VELOCITY is on.
        if meas.vel_xy is not None:
            zv = np.array([meas.vel_xy[0], meas.vel_xy[1]])
            Rv = np.diag([self.cue_vel_sigma_m_s ** 2, self.cue_vel_sigma_m_s ** 2])
            yv = zv - self.H_VEL @ self.x
            Sv = self.H_VEL @ self.P @ self.H_VEL.T + Rv
            Kv = self.P @ self.H_VEL.T @ np.linalg.inv(Sv)
            self.x = self.x + Kv @ yv
            self.P = (np.eye(4) - Kv @ self.H_VEL) @ self.P

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
    __slots__ = ("t", "rel_xy", "range_m", "bearing_rad", "source", "age_s", "vel_xy")

    def __init__(self, t, rel_xy, range_m, bearing_rad, source, age_s=0.0, vel_xy=None):
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
        # vel_xy: an EMITTED absolute-velocity estimate carried alongside the
        # position (ADR-0015 data-constraint #2, opt-in). None on the camera
        # (onboard bearing-only, no velocity channel) and on a baseline cue
        # (which sends position only, forcing the drone to DIFFERENTIATE).
        # When the cue emits a filtered velocity (CUE_EMIT_VELOCITY), the
        # target tracker ingests it DIRECTLY instead of differentiating a
        # noisy position -- the ADR-0015 "does PIP survive without an emitted
        # velocity" test.
        self.vel_xy = vel_xy


# =========================================================================
# ADR-0015 PERCEPTION DATA-CONSTRAINTS (docs/decisions.md ADR-0015's
# "DATA CONSTRAINTS BACK TO THE GUIDANCE MATH" table). Every knob below is
# OPT-IN: PERCEPTION_DEFAULTS is the byte-identical baseline (a perception
# config == None, or all these values, reproduces the pre-ADR-0015 sensor
# EXACTLY -- no extra RNG draw fires unless its feature is switched on).
# These model how a REAL cue (cross-platform ground track) and a REAL
# terminal seeker (ML box on a small FPV drone) degrade vs the sim's clean
# AprilTag + flat-0.5 m-cue stand-ins, so the guidance re-tuning sees the
# real track quality BEFORE any hardware dollar is spent ("lab ranks, Gazebo
# decides", ADR-0015 build-plan step 3).
# =========================================================================
PERCEPTION_DEFAULTS = dict(
    # --- constraint 1: range-dependent cue position sigma + per-run datum bias ---
    # sigma_R(R) = CUE_SIGMA_BASE_M + CUE_SIGMA_QUAD * R^2 (ground-stereo range
    # error grows ~R^2). None -> the flat CUE_SIGMA_M baseline (0.5 m).
    CUE_SIGMA_BASE_M=None,
    CUE_SIGMA_QUAD=0.0,
    # per-run CONSTANT common-mode offset vector magnitude (RTK-vs-standard-GPS
    # datum bias, ADR-0015 table rows 1 & 5). Drawn once per run: fixed
    # magnitude, random direction. 0.0 -> no bias (baseline).
    CUE_DATUM_BIAS_MAG_M=0.0,
    # --- constraint 2: cue velocity channel ---
    # False -> cue sends POSITION ONLY (drone must DIFFERENTIATE it: baseline).
    # True  -> cue EMITS a filtered velocity (true target vel + N(0,sigma_v)),
    #          ingested directly by the tracker (see TargetTracker.correct_full).
    CUE_EMIT_VELOCITY=False,
    CUE_VEL_SIGMA_M_S=0.5,
    # --- constraint 3: latency mean (Sensor.cue_latency) + JITTER ---
    # CUE_LATENCY_MEAN_S: override the fixed delivery-latency MEAN (None ->
    # the CUE_LATENCY_S=0.1 baseline). ADR-0015's recommended realistic mean
    # is 0.12 s (matches the Gazebo mock's --latency-s default); added for
    # the P-6 tier presets. Purely a schedule constant -- no RNG involved.
    CUE_LATENCY_MEAN_S=None,
    # extra +-uniform jitter on each delivery, ON TOP of the fixed mean. The
    # delivered age_s stays the NOMINAL mean (what a timestamp-at-source system
    # "knows"), so the jitter is the UNCOMPENSATED residual. 0.0 -> baseline.
    CUE_LATENCY_JITTER_S=0.0,
    # --- constraint 4: bursty Markov cue dropout ---
    # a 2-state (up/down) chain on cue delivery: mean outage burst
    # CUE_DROPOUT_BURST_S long, steady-state outage fraction CUE_DROPOUT_P_OUT.
    CUE_DROPOUT_MARKOV=False,
    CUE_DROPOUT_P_OUT=0.10,
    CUE_DROPOUT_BURST_S=1.5,
    # --- constraint 5: jammer link cutoff BEFORE lock (+ dead-reckon coast) ---
    # the cue link dies PERMANENTLY once the true range falls to <= this (a
    # jammer engaging in the terminal approach), or once t >= the time cutoff.
    # After cutoff the two-stage law flies the last cued intercept solution
    # ballistically (the tracker predict()s forward on its frozen velocity)
    # and relies on the camera acquiring before CPA -- else a miss. None ->
    # cue alive until handoff (baseline).
    CUE_LINK_CUTOFF_RANGE_M=None,
    CUE_LINK_CUTOFF_TIME_S=None,
    # --- constraint 6: LOS-rate-dependent terminal camera dropout ---
    # the terminal detector's dropout probability rises with the TRUE |lambda
    # dot| (models the ADR-0014 yaw-rate-deficit / motion-blur CPA loss that
    # caused 20/20 terminal dropouts), ADDED on top of the edge-weighted term.
    # Ramps linearly to TERM_LOS_RATE_DROPOUT_MAX as |lambda_dot| reaches
    # TERM_LOS_RATE_REF_DEG_S. False -> baseline (edge-weighted dropout only).
    TERM_LOS_RATE_DROPOUT=False,
    TERM_LOS_RATE_REF_DEG_S=90.0,
    TERM_LOS_RATE_DROPOUT_MAX=0.7,
    # --- constraint 7: ML-grade terminal (camera) sensor noise ---
    # override the camera's bearing/range noise with ML-detector-grade values
    # (box centroid ~1-2 deg bearing; box-size-to-range 15-30%). None -> the
    # AprilTag-calibrated SIGMA_BEARING_DEG (6.0) / RANGE_NOISE_FRAC (0.10).
    TERM_BEARING_SIGMA_DEG=None,
    TERM_RANGE_NOISE_FRAC=None,
)


def resolve_perception(perception):
    """Merge a partial perception override onto PERCEPTION_DEFAULTS. None ->
    the baseline dict (every feature off)."""
    if perception is None:
        return dict(PERCEPTION_DEFAULTS)
    return {**PERCEPTION_DEFAULTS, **perception}


def draw_datum_bias(rng, mag_m):
    """One per-run constant common-mode offset vector: fixed magnitude,
    random direction (RTK-vs-standard-GPS datum bias). Returns (0,0) with NO
    rng draw when mag<=0 so the baseline RNG sequence is untouched."""
    if mag_m is None or mag_m <= 0.0:
        return (0.0, 0.0)
    theta = rng.uniform(0.0, 2.0 * math.pi)
    return (mag_m * math.cos(theta), mag_m * math.sin(theta))


class Sensor:
    """The interceptor's ONLY window onto the target -- mirrors the camera
    honesty boundary (docs/goals.md / ADR-0008 lineage): guidance never sees
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
        perception=None, datum_bias=(0.0, 0.0), cue_until_latch=False,
    ):
        self.rng = rng
        self.det_range = det_range
        self.handoff_range = handoff_range
        self.cam_period = 1.0 / cam_hz
        self.cue_period = 1.0 / cue_hz
        # ADR-0015 constraint #7: ML-grade terminal (camera) noise overrides.
        p = resolve_perception(perception)
        self.perc = p
        if p["TERM_BEARING_SIGMA_DEG"] is not None:
            sigma_bearing_deg = p["TERM_BEARING_SIGMA_DEG"]
        if p["TERM_RANGE_NOISE_FRAC"] is not None:
            range_noise_frac = p["TERM_RANGE_NOISE_FRAC"]
        # ADR-0015 constraint #3 mean (P-6 tiers): pure schedule constant,
        # no RNG; None -> the CUE_LATENCY_S baseline passed in above.
        if p["CUE_LATENCY_MEAN_S"] is not None:
            cue_latency_s = p["CUE_LATENCY_MEAN_S"]
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
        self._cue_pending = []  # list of (deliver_t, x, y, vel_xy)
        # own_heading/desired_heading start centered on the target -- mirrors
        # the real engagement's ACQUIRE phase (hover + yaw-center on the
        # tag) already having completed BEFORE the ENGAGE clock this lab
        # simulates starts ticking (see simulate()'s initial_heading calc).
        self.own_heading = initial_heading
        self.desired_heading = initial_heading

        # --- ADR-0015 derived perception state (all inert at baseline) ---
        self.datum_bias = datum_bias
        # P-6 (FUSE_MIDCOURSE): when True, cue delivery is gated by the
        # LAW's one-way handoff latch (simulate() calls close_cue() the tick
        # the latch fires) instead of by the handoff_range geometry -- the
        # real m4_intercept.py semantics, where the cue socket stays open
        # until HANDOFF closes it. False (default) -> the legacy geometric
        # gate, byte-identical.
        self.cue_until_latch = bool(cue_until_latch)
        self.cue_gate_closed = False
        # constraint 1: range-dependent cue sigma
        self._cue_sigma_base = p["CUE_SIGMA_BASE_M"]
        self._cue_sigma_quad = p["CUE_SIGMA_QUAD"]
        # constraint 2: velocity emission
        self._cue_emit_velocity = bool(p["CUE_EMIT_VELOCITY"])
        self._cue_vel_sigma = p["CUE_VEL_SIGMA_M_S"]
        # constraint 3: latency jitter
        self._cue_latency_jitter = p["CUE_LATENCY_JITTER_S"]
        # constraint 4: bursty Markov cue dropout -- precompute per-sample
        # transition probabilities from (p_out, mean burst length).
        self._cue_markov = bool(p["CUE_DROPOUT_MARKOV"])
        self._cue_link_up = True  # Markov up/down state (True = delivering)
        p_out = clamp(p["CUE_DROPOUT_P_OUT"], 0.0, 0.999)
        burst_s = max(1e-3, p["CUE_DROPOUT_BURST_S"])
        # P(down->up) per cue sample; P(up->down) from steady-state p_out.
        self._p_down_to_up = clamp(self.cue_period / burst_s, 0.0, 1.0)
        self._p_up_to_down = clamp(
            self._p_down_to_up * p_out / max(1e-6, 1.0 - p_out), 0.0, 1.0
        )
        # constraint 5: jammer link cutoff (permanent latch once tripped)
        self._cue_cutoff_range = p["CUE_LINK_CUTOFF_RANGE_M"]
        self._cue_cutoff_time = p["CUE_LINK_CUTOFF_TIME_S"]
        self._cue_link_cut = False
        self._cue_link_cut_t = None  # scoring diagnostic (jam-envelope study)
        # constraint 6: LOS-rate-dependent terminal camera dropout
        self._term_los_rate_dropout = bool(p["TERM_LOS_RATE_DROPOUT"])
        self._term_los_rate_ref = math.radians(p["TERM_LOS_RATE_REF_DEG_S"])
        self._term_los_rate_dropout_max = p["TERM_LOS_RATE_DROPOUT_MAX"]
        self._prev_true_bearing = None
        self._prev_bearing_t = None

    def _cue_sigma_at(self, true_range):
        """Range-dependent cue position sigma (constraint 1): base + quad*R^2,
        falling back to the flat baseline cue_sigma when not configured."""
        if self._cue_sigma_base is None:
            return self.cue_sigma
        return self._cue_sigma_base + self._cue_sigma_quad * true_range * true_range

    def close_cue(self):
        """P-6 one-way cue-gate close (mirrors m4_intercept.py's HANDOFF
        closing the UDP socket): no further deliveries, and the in-flight
        (pending) datagrams are lost with the socket. Only ever called by
        simulate() when the law's latch fires under cue_until_latch."""
        self.cue_gate_closed = True
        self._cue_pending = []

    def tick(self, t, dt, own_pos, target_pos, two_stage, target_vel=None):
        rel = (target_pos[0] - own_pos[0], target_pos[1] - own_pos[1])
        true_range = math.hypot(rel[0], rel[1])
        true_bearing = math.atan2(rel[1], rel[0])

        # TRUE LOS rate (constraint 6 input; also purely deterministic -- no
        # rng, so tracking it every tick is baseline-preserving). This reads
        # ground truth to decide a SENSOR dropout (a physical blur/tracking
        # effect), never fed to guidance -- same honesty boundary as the
        # boresight/FOV geometry the sensor already uses.
        los_rate = 0.0
        if self._prev_true_bearing is not None and self._prev_bearing_t is not None:
            los_rate = wrap_pi(true_bearing - self._prev_true_bearing) / max(
                1e-6, t - self._prev_bearing_t
            )
        self._prev_true_bearing = true_bearing
        self._prev_bearing_t = t

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
                # constraint 6: extra dropout that rises with the TRUE LOS
                # rate (yaw-rate-deficit / motion-blur CPA loss). Added to
                # p_drop BEFORE the single rng.random() draw -- so the baseline
                # (feature off) draw count and p_drop are byte-identical.
                if self._term_los_rate_dropout:
                    extra = self._term_los_rate_dropout_max * clamp(
                        abs(los_rate) / max(1e-6, self._term_los_rate_ref), 0.0, 1.0
                    )
                    p_drop = clamp(p_drop + extra, 0.0, 0.999)
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
            # constraint 5: permanent jammer link cutoff (latch + flush the
            # in-flight buffer so the dead-reckon coast starts immediately).
            if not self._cue_link_cut and (
                (self._cue_cutoff_range is not None and true_range <= self._cue_cutoff_range)
                or (self._cue_cutoff_time is not None and t >= self._cue_cutoff_time)
            ):
                self._cue_link_cut = True
                self._cue_link_cut_t = t
                self._cue_pending = []

            self._cue_timer += dt
            if self._cue_timer >= self.cue_period:
                self._cue_timer -= self.cue_period
                # constraint 4: bursty Markov up/down transition (one draw per
                # cue sample, only when the feature is on).
                if self._cue_markov:
                    r = self.rng.random()
                    if self._cue_link_up:
                        if r < self._p_up_to_down:
                            self._cue_link_up = False
                    else:
                        if r < self._p_down_to_up:
                            self._cue_link_up = True
                # P-6 (FUSE_MIDCOURSE): under cue_until_latch the delivery
                # window is "until the law's latch closes the gate" (any
                # range) instead of "beyond handoff_range". The default path
                # (cue_until_latch=False) evaluates the exact original
                # geometric condition.
                if self.cue_until_latch:
                    in_cue_window = not self.cue_gate_closed
                else:
                    in_cue_window = true_range > self.handoff_range
                deliver_ok = (
                    not self._cue_link_cut
                    and (self._cue_link_up or not self._cue_markov)
                    and in_cue_window
                )
                if deliver_ok:
                    sigma = self._cue_sigma_at(true_range)
                    nx = target_pos[0] + self.datum_bias[0] + self.rng.normal(0.0, sigma)
                    ny = target_pos[1] + self.datum_bias[1] + self.rng.normal(0.0, sigma)
                    vel = None
                    if self._cue_emit_velocity and target_vel is not None:
                        # constraint 2: cue emits a FILTERED velocity (true +
                        # noise, NO datum bias -- a constant offset does not
                        # corrupt a derivative, which is exactly why emitting
                        # velocity sidesteps the biased-position PIP-killer).
                        vel = (
                            target_vel[0] + self.rng.normal(0.0, self._cue_vel_sigma),
                            target_vel[1] + self.rng.normal(0.0, self._cue_vel_sigma),
                        )
                    # constraint 3: mean latency + uncompensated jitter. The
                    # delivered age_s stays the NOMINAL mean (source timestamp
                    # is known); the jitter is the residual staleness.
                    delay = self.cue_latency
                    if self._cue_latency_jitter > 0.0:
                        delay += self.rng.uniform(-self._cue_latency_jitter, self._cue_latency_jitter)
                    self._cue_pending.append((t + delay, nx, ny, vel))
            while self._cue_pending and self._cue_pending[0][0] <= t:
                _, nx, ny, vel = self._cue_pending.pop(0)
                rel_n = (nx - own_pos[0], ny - own_pos[1])
                r = math.hypot(*rel_n)
                b = math.atan2(rel_n[1], rel_n[0])
                cue_meas = Measurement(t, rel_n, r, b, "cue", age_s=self.cue_latency, vel_xy=vel)

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
    # --- ADR-0014 lever 2 opt-ins (all False/None = baseline-preserving:
    # the ORIGINAL whole-vector freeze, uncapped lambda_dot). See PurePN's
    # docstring for the full mechanics. ---
    # SPLIT_FREEZE: freeze only the v_perp MAGNITUDE inside
    # TERMINAL_FREEZE_RANGE; keep v_close and lambda_hat LIVE off fresh
    # detections (reconstructed each tick), instead of freezing vx,vy whole.
    SPLIT_FREEZE=False,
    # LAMBDA_DOT_CAP_DEG_S: cap-BOTH variant (routed into the lambda filter
    # itself via _build_lambda_range_filters -- affects predict() AND a_cmd).
    LAMBDA_DOT_CAP_DEG_S=None,
    # LAMBDA_DOT_CAP_A_CMD_ONLY_DEG_S: the REJECTED cap-a_cmd-ONLY variant,
    # kept implemented so a run can empirically confirm it is worse/unstable
    # (ADR-0014 lever 2's explicit ask) -- clamps only the value read for
    # a_cmd; the filter's own xdot_hat (and hence predict()'s forward
    # integration of lambda_hat) stays raw/uncapped.
    LAMBDA_DOT_CAP_A_CMD_ONLY_DEG_S=None,
    # LEAD_EXTRAP_MAX_S: optional sub-variant -- through a genuinely BLIND
    # tail (no fresh camera correction for longer than MEAS_STALE_S, up to
    # this many seconds), aim at a Cartesian dead-reckoned lead point from a
    # parallel-fed TargetTracker instead of coasting the polar lambda/range
    # filters' own (singular-prone) forward integration. None disables it
    # (baseline-preserving; no extra tracker is even constructed).
    LEAD_EXTRAP_MAX_S=None,
    LEAD_EXTRAP_MIN_CORRECTIONS=6,
    LEAD_EXTRAP_TRACK_ALPHA=0.6, LEAD_EXTRAP_TRACK_BETA=0.2,
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
    BETA_RANGE gains, baseline-preserving). ADR-0014 lever 2's "cap-both"
    variant wires `LAMBDA_DOT_CAP_DEG_S` (default None -> uncapped,
    baseline-preserving) into the lambda filter's `rate_cap` -- this affects
    BOTH predict()'s forward integration and any a_cmd read of xdot_hat, in
    one place, for every law that shares this builder."""
    lam_sp_deg = p.get("KALATA_LAMBDA_SIGMA_PROCESS_DEG")
    lam_rate_cap_deg = p.get("LAMBDA_DOT_CAP_DEG_S")
    lam = AlphaBetaFilter(
        p["ALPHA"], p["BETA_LAMBDA"], angular=True,
        kalata_sigma_process=math.radians(lam_sp_deg) if lam_sp_deg is not None else None,
        kalata_sigma_meas=math.radians(p.get("KALATA_LAMBDA_SIGMA_MEAS_DEG", SIGMA_BEARING_DEG)),
        rate_cap=math.radians(lam_rate_cap_deg) if lam_rate_cap_deg is not None else None,
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
    compute_v_close vs its VC_FLOOR_M_S comment).

    ADR-0014 LEVER 2 (opt-in, all default-off -- see DEFAULT_PN_PARAMS):
    the diagnosed real-Gazebo mechanism is a yaw-rate deficit that walks the
    tag off-boresight WELL inside the nominal FOV during ENGAGE; the terminal
    response to that today is `m4_intercept.py`'s whole-vector freeze once
    r_hat < TERMINAL_FREEZE_RANGE_M (lambda_dot is singular as R->0). Three
    independent knobs replace/augment that:

      SPLIT_FREEZE=True: freeze ONLY the v_perp MAGNITUDE at the tick the
      vehicle first enters the freeze range (a one-way latch, same
      permanent-coast philosophy as the original -- no un-freeze). v_close
      and lambda_hat stay LIVE off fresh detections every tick (both were
      already being corrected above regardless of freeze status -- only the
      OUTPUT command was frozen before); the command is reconstructed each
      tick as v_close*(cos,sin) + v_perp_frozen*(-sin,cos). v_close is safe
      to keep live because compute_v_close is a function of r_hat, which is
      monotone and never singular (unlike lambda_dot); only the LOS-rate
      term is what actually blows up near R->0, so only IT gets frozen.

      LAMBDA_DOT_CAP_DEG_S vs LAMBDA_DOT_CAP_A_CMD_ONLY_DEG_S: see
      AlphaBetaFilter's rate_cap docstring for the cap-both mechanism and
      this class's own docstring note above -- exactly one of these two
      should be set per run, never both; they exist side by side so a study
      can prove the a_cmd-only variant is worse (the ADR-0009 whipsaw:
      predict() keeps forward-integrating lambda_hat off a raw, unbounded
      lambda_dot even though a_cmd's OWN read was capped, so the commanded
      DIRECTION still corrupts).

      LEAD_EXTRAP_MAX_S (optional sub-variant, only meaningful alongside
      SPLIT_FREEZE): through a genuinely BLIND tail -- no fresh camera
      correction for longer than MEAS_STALE_S, for up to this many seconds,
      gated on having had >= LEAD_EXTRAP_MIN_CORRECTIONS prior camera fixes
      -- aim directly at a parallel-fed TargetTracker's Cartesian
      dead-reckoned position instead of the split-freeze's v_close/v_perp
      reconstruction. This sidesteps the polar (lambda, range) singularity
      entirely for that tail by working in Cartesian space, where a
      constant-velocity target's extrapolated position is well-behaved even
      as R->0."""

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

        # --- ADR-0014 lever 2 opt-ins ---
        self.split_freeze = bool(p.get("SPLIT_FREEZE", False))
        cap_a_cmd_deg = p.get("LAMBDA_DOT_CAP_A_CMD_ONLY_DEG_S")
        self.lambda_dot_cap_a_cmd_only_rad = (
            math.radians(cap_a_cmd_deg) if cap_a_cmd_deg is not None else None
        )
        self._v_perp_frozen = False
        self._v_perp_frozen_value = 0.0

        self.lead_extrap_max_s = p.get("LEAD_EXTRAP_MAX_S")
        self.lead_extrap_min_corrections = p.get("LEAD_EXTRAP_MIN_CORRECTIONS", 6)
        if self.lead_extrap_max_s is not None:
            self._lead_tracker = TargetTracker(
                p.get("LEAD_EXTRAP_TRACK_ALPHA", 0.6), p.get("LEAD_EXTRAP_TRACK_BETA", 0.2)
            )
        else:
            self._lead_tracker = None
        self._lead_corrections = 0
        self._last_camera_t = None

    def _capped_lambda_dot(self, lambda_dot):
        if self.lambda_dot_cap_a_cmd_only_rad is None:
            return lambda_dot
        return clamp(lambda_dot, -self.lambda_dot_cap_a_cmd_only_rad, self.lambda_dot_cap_a_cmd_only_rad)

    def step(self, t, dt, own_pos, own_vel, meas):
        self.lam.predict(dt)
        self.rng_f.predict(dt)
        if self._lead_tracker is not None:
            self._lead_tracker.predict(dt)
        meas = _gate_camera_meas(meas, self.rng_f, self.range_gate_k, self.range_gate_sigma_frac)
        if meas is not None and meas.source == "camera":
            self.lam.correct(meas.bearing_rad, t)
            self.rng_f.correct(meas.range_m, t)
            if self._lead_tracker is not None:
                abs_pos = (own_pos[0] + meas.rel_xy[0], own_pos[1] + meas.rel_xy[1])
                self._lead_tracker.correct(abs_pos, t)
                self._lead_corrections += 1
                self._last_camera_t = t
        if not self.lam.initialized:
            return (0.0, 0.0)

        r_hat = self.rng_f.x_hat
        in_freeze = r_hat is not None and r_hat < self.freeze_range

        if not self.split_freeze:
            # ORIGINAL whole-vector freeze (baseline-preserving path,
            # unchanged from the pre-ADR-0014 implementation).
            if self.frozen is not None or in_freeze:
                if self.frozen is None:
                    self.frozen = self.last_cmd
                return self.frozen

            lambda_hat = self.lam.x_hat
            lambda_dot = self.lam.xdot_hat
            rdot_hat = self.rng_f.xdot_hat if self.rng_f.initialized else None
            vc = max(self.vc_floor, -rdot_hat) if rdot_hat is not None else self.vc_floor
            a_cmd = self.N * vc * self._capped_lambda_dot(lambda_dot)
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

        # --- SPLIT_FREEZE path (ADR-0014 lever 2) ---
        if in_freeze and not self._v_perp_frozen:
            self._v_perp_frozen = True
            self._v_perp_frozen_value = self.v_perp

        if not self._v_perp_frozen:
            lambda_dot = self.lam.xdot_hat
            rdot_hat = self.rng_f.xdot_hat if self.rng_f.initialized else None
            vc = max(self.vc_floor, -rdot_hat) if rdot_hat is not None else self.vc_floor
            a_cmd = self.N * vc * self._capped_lambda_dot(lambda_dot)
            self.v_perp = clamp(self.v_perp + a_cmd * dt, -self.v_perp_max, self.v_perp_max)
            v_perp_use = self.v_perp
        else:
            v_perp_use = self._v_perp_frozen_value

        # LIVE: both corrected unconditionally above, unlike v_perp.
        lambda_hat = self.lam.x_hat
        v_close = compute_v_close(
            r_hat, self.v_close_runin, self.v_close, self.throttle_range, self.freeze_range
        )

        # Optional lead-extrapolation through a genuinely BLIND tail.
        if (
            self.lead_extrap_max_s is not None and self._lead_tracker is not None
            and self._lead_tracker.pos_hat is not None
            and self._last_camera_t is not None
            and self._lead_corrections >= self.lead_extrap_min_corrections
        ):
            blind_s = t - self._last_camera_t
            if MEAS_STALE_S < blind_s <= self.lead_extrap_max_s:
                aim = self._lead_tracker.pos_hat
                direction = (aim[0] - own_pos[0], aim[1] - own_pos[1])
                dnorm = math.hypot(direction[0], direction[1])
                if dnorm > 1e-6:
                    vx = v_close * direction[0] / dnorm
                    vy = v_close * direction[1] / dnorm
                    norm = math.hypot(vx, vy)
                    if norm > self.v_total_max:
                        s = self.v_total_max / norm
                        vx *= s
                        vy *= s
                    self.last_cmd = (vx, vy)
                    return (vx, vy)

        u = (math.cos(lambda_hat), math.sin(lambda_hat))
        pvec = (-math.sin(lambda_hat), math.cos(lambda_hat))
        vx = v_close * u[0] + v_perp_use * pvec[0]
        vy = v_close * u[1] + v_perp_use * pvec[1]
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


# Floor for the assumed camera range sigma in the fused-range weight (same
# floor range_gate_ok uses -- "know your own sensor spec").
FUSE_RANGE_SIGMA_FLOOR_M = 0.3


class FusedTrack:
    """P-6 mid-course fusion (ADR-0015 fusion mechanization, lab prototype;
    ported to m4_intercept.py's --fuse-midcourse -- keep the two in sync): a
    POLAR fused target estimate maintained through the window where BOTH
    sources exist (camera detecting AND cue still alive, i.e. from first
    camera detection until the handoff latch).

    Mechanization -- BEARING-WEIGHTED BY DESIGN:
      - lambda channel: alpha-beta on the CAMERA bearing ONLY. The cue never
        touches the angle. ADR-0015 #3's own ruling is that bearing-only
        suffices for pro-nav (it consumes the LOS *rate*, an angle), so
        fusion must HELP the lambda_dot estimate, not fight it -- and a
        datum-biased cue position at 8 m range is ~18 deg of bearing error
        (2.5 m standard-GPS bias), which would poison the seeker's clean
        few-degree angle. Camera wins the angle, always. (The alternative --
        cue corrects the angle too -- is exactly what the Cartesian dash
        tracker already does, so the A/B against the off/off baseline
        covers it.)
      - range channel: alpha-beta corrected by BOTH the camera's monocular
        range (full gain) AND the cue-implied range |cue_pos - own_pos|
        (gain-scaled), the scale being an inverse-variance weight from KNOWN
        sensor specs (the same "know your own sensor spec" boundary as
        range_gate_ok): camera sigma = cam_range_frac * R_hat (own seeker
        bench spec); cue sigma = the assumed transmitted-covariance model
        sigma_R(R) = base + quad*R^2 (ADR-0015 #3: the real track message
        carries covariance) RSS'd with a FIXED datum-bias budget -- the
        cross-platform datum offset is unknowable on the drone, so it is
        budgeted at the shared-RTK design value; an off-spec day (the WORST
        tier's 2.5 m) tests exactly the fragility question.
      - velocity: the cue's EMITTED velocity when fresh (ADR-0015's #1
        lever), else the caller's fallback (the Cartesian dash tracker's
        estimate).
    Everything here is per-tick scalar math -- trivially implementable on
    the real Pi (it is LESS work than the KalmanTracker this lab already
    carries as candidate 1)."""

    def __init__(self, alpha, beta_lambda, beta_range, cam_range_frac,
                 cue_sigma_base, cue_sigma_quad, datum_budget_m, vel_stale_s):
        self.lam = AlphaBetaFilter(alpha, beta_lambda, angular=True)
        self.rng_f = AlphaBetaFilter(alpha, beta_range)
        self.cam_range_frac = cam_range_frac
        self.cue_sigma_base = cue_sigma_base
        self.cue_sigma_quad = cue_sigma_quad
        self.datum_budget = datum_budget_m
        self.vel_stale_s = vel_stale_s
        self.vel = None        # latest cue-EMITTED velocity (None if never sent)
        self.vel_t = None
        self.last_camera_t = None

    def predict(self, dt):
        self.lam.predict(dt)
        self.rng_f.predict(dt)

    def update(self, meas, t):
        if meas.source == "camera":
            self.lam.correct(meas.bearing_rad, t)
            self.rng_f.correct(meas.range_m, t)
            self.last_camera_t = t
            return
        # cue
        if meas.vel_xy is not None:
            self.vel = meas.vel_xy
            self.vel_t = t
        # Range-only fusion, and only once the camera side exists: the
        # fusion window starts at first detection. Before that the cue
        # already steers the dash through the Cartesian tracker; it must not
        # pre-seed the CAMERA-anchored polar track (a biased cue defining
        # the initial state would be exactly the angle-poisoning this
        # mechanization exists to prevent).
        if not (self.rng_f.initialized and self.lam.initialized):
            return
        r_hat = max(self.rng_f.x_hat, 1e-3)
        sigma_cam = max(FUSE_RANGE_SIGMA_FLOOR_M, self.cam_range_frac * r_hat)
        cue_r = meas.range_m
        sigma_cue_stat = self.cue_sigma_base + self.cue_sigma_quad * cue_r * cue_r
        sigma_cue = math.sqrt(sigma_cue_stat * sigma_cue_stat + self.datum_budget * self.datum_budget)
        w = min(1.0, (sigma_cam * sigma_cam) / max(1e-6, sigma_cue * sigma_cue))
        self.rng_f.correct(cue_r, t, gain_scale=w)

    def camera_fresh(self, t):
        return self.last_camera_t is not None and (t - self.last_camera_t) <= MEAS_STALE_S

    def target_vel(self, t, fallback):
        if (self.vel is not None and self.vel_t is not None
                and (t - self.vel_t) <= self.vel_stale_s):
            return self.vel
        return fallback

    def state(self, t, own_pos, own_vel, fallback_vel):
        """Fused snapshot dict (pos, vel, lam, R, rdot, lamdot), or None if
        the camera side hasn't initialized the polar track yet. The rates
        are GEOMETRIC -- derived from the fused target velocity (cue-emitted
        when fresh, ADR-0015's #1 lever) minus own velocity, projected
        along/across the LOS:
            rdot   = (vt - vo) . u
            lamdot = cross(u, vt - vo) / R
        rather than the alpha-beta's own still-converging rate states; the
        whole point of the warm handoff is that ~1-2 s of noisy camera
        corrections cannot match a transmitted filtered velocity."""
        if not (self.lam.initialized and self.rng_f.initialized):
            return None
        lam = self.lam.x_hat
        R = max(self.rng_f.x_hat, 1e-3)
        u = (math.cos(lam), math.sin(lam))
        vt = self.target_vel(t, fallback_vel) or (0.0, 0.0)
        vrel = (vt[0] - own_vel[0], vt[1] - own_vel[1])
        rdot = vrel[0] * u[0] + vrel[1] * u[1]
        lamdot = (u[0] * vrel[1] - u[1] * vrel[0]) / R
        pos = (own_pos[0] + R * u[0], own_pos[1] + R * u[1])
        return dict(pos=pos, vel=vt, lam=lam, R=R, rdot=rdot, lamdot=lamdot)


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
    # --- P-6 mid-course fusion + warm handoff (ADR-0015 fusion decision;
    # all default-off = byte-identical baseline; see FusedTrack + the
    # --p6-fusion study driver) ---
    # FUSE_MIDCOURSE: maintain a FusedTrack (camera bearing + weighted cue
    # range + cue velocity) through the both-sources window, steer the dash
    # off it while the camera is fresh, and keep the cue DELIVERABLE until
    # the one-way latch (real-pipeline semantics) instead of killing it at
    # handoff_range.
    FUSE_MIDCOURSE=False,
    # WARM_HANDOFF: at the latch tick, initialize the terminal law's
    # filter/rate states (lambda_dot, R, Rdot, v_perp, target pos/vel) from
    # the fused track (or, without FUSE_MIDCOURSE, from the dash tracker)
    # instead of their cold/ramping values. See _warm_init_terminal.
    WARM_HANDOFF=False,
    # FUSE_GAZEBO_LATCH: attribution control -- ONLY the Gazebo-faithful
    # one-way latch (streak >= FUSE_LATCH_STREAK), no fusion, no warm init.
    # Lets the A/B separate "the latch semantics changed" from "fusion/warm
    # helped". Implied True whenever either flag above is on.
    FUSE_GAZEBO_LATCH=False,
    FUSE_LATCH_STREAK=3,       # mirrors m4_intercept.py's S2 HANDOFF_STREAK_MIN
    # Drone-side spec constants for the fused-range weight (FusedTrack):
    # assumed cue sigma model (the ADR-0015 recommended realistic
    # sigma_R(R)) + a FIXED datum-bias budget at the shared-RTK design
    # value. Deliberately NOT read from the per-run perception config: a
    # fielded fusion is tuned to its design spec, and the WORST tier tests
    # exactly what happens when reality runs off-spec.
    FUSE_CUE_SIGMA_BASE_M=0.4,
    FUSE_CUE_SIGMA_QUAD=0.008,
    FUSE_DATUM_BUDGET_M=0.5,
    FUSE_VEL_STALE_S=0.6,      # emitted-velocity freshness horizon (~6 cue periods)
    # --- ADR-0015 "Handoff continuity" coast-search, lab port (jam-envelope
    # study; all default-off = byte-identical baseline). Mirrors
    # m4_intercept.py --coast-search semantics EXACTLY (constants below are
    # m4's COAST_* values verbatim): if the cue goes silent > COAST_STALE_S
    # during the dash, latch a dead-reckon COAST toward the tracker's
    # predicted point (the tracker already predict()s forward each tick on
    # the last cue velocity -- which is exactly why ADR-0015 says velocity
    # EMISSION is what makes coasting possible: the frozen vel_hat was
    # ingested, not differentiated); once the PREDICTED range falls to
    # COAST_ACQ_RANGE_M, HOLD position and run a bounded +/- yaw sweep of
    # the seeker boresight around the predicted LOS (the "bounded seeker
    # search" at the basket); if the camera has not acquired within
    # COAST_SEARCH_BUDGET_S of link loss, BREAK OFF rather than fly blind.
    # A cue that comes back (Markov burst, not a jammer) un-latches the
    # coast, exactly like m4's "cue link recovered" branch. Acquisition
    # itself stays 100% the existing Sensor det_range/FOV/dropout model --
    # the sweep only points the boresight; if the dead-reckon error walks
    # the true target outside the swept FOV cone or the target never comes
    # inside det_range of the held position, the run stays lost (honest
    # break-off/miss, no omniscient reacquisition). Everything here is
    # per-tick scalar math on the drone's OWN track state -- same Pi-class
    # implementability argument as FusedTrack (and it is already
    # implemented on the real pipeline side in m4_intercept.py).
    COAST_SEARCH=False,
    COAST_STALE_S=1.0,             # m4 COAST_STALE_S
    COAST_ACQ_RANGE_M=10.0,        # m4 COAST_ACQ_RANGE_M
    COAST_SEARCH_YAW_AMP_DEG=20.0,  # m4 COAST_SEARCH_YAW_AMP_DEG
    COAST_SEARCH_PERIOD_S=4.0,     # m4 COAST_SEARCH_PERIOD_S (~31 deg/s peak)
    COAST_SEARCH_BUDGET_S=8.0,     # m4 COAST_SEARCH_BUDGET_S
    # DASH_YAW_TRACKER (default False = byte-identical): m4's actual DASH
    # yaw law -- "Yaw points at the TRACKER's azimuth, not a camera bearing
    # -- there may be no detection yet at DASH's longer ranges" -- i.e. the
    # boresight follows the cue-track azimuth all through the dash. The
    # legacy lab boresight instead stays at its INITIAL heading until the
    # first camera detection (fine on the compressed ~15 m standoff the lab
    # was calibrated on, where the initial heading already points at the
    # engagement; wrong for the jam-envelope's ~45 m runway, where cue-
    # driven yaw is what delivers the seeker onto the target at all). The
    # jam-envelope study turns this on for EVERY cell so the COAST_SEARCH
    # off/on axis isolates the coast/search/break-off branch itself, not
    # the boresight-steering difference.
    DASH_YAW_TRACKER=False,
    # CUE_UNTIL_LATCH (default False = byte-identical): extend ADR-0018's
    # "cue stays deliverable until the one-way latch" real-pipeline
    # semantics (m4's cue socket is closed BY the HANDOFF latch, never by a
    # range gate) to NON-fused runs too. Only meaningful together with the
    # Gazebo-faithful latch (FUSE_GAZEBO_LATCH or any flag implying it);
    # the jam-envelope study sets it for every cell so a 9-11.5 m link
    # cutoff means the same thing in fuse=off and fuse=on cells.
    CUE_UNTIL_LATCH=False,
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

        # --- P-6 fusion opt-ins (all default-off; see DEFAULT_DASH_PARAMS
        # and _step_fused's docstring). gazebo_latch is implied by either
        # feature flag: fusion extends the cue window and warm init must
        # fire exactly once -- both need the real one-way latch to stay
        # honest.
        self.fuse_midcourse = bool(p.get("FUSE_MIDCOURSE", False))
        self.warm_handoff = bool(p.get("WARM_HANDOFF", False))
        # ADR-0015 coast-search lab port (jam-envelope study; see the
        # DEFAULT_DASH_PARAMS block for the full mechanization rationale).
        # Both flags imply the Gazebo-faithful one-way latch below: the
        # coast/search/break-off branch and the tracker-azimuth dash yaw
        # are properties of the REAL (latched) S2 pipeline, and the legacy
        # re-evaluated handoff would let them interleave dishonestly.
        self.coast_search = bool(p.get("COAST_SEARCH", False))
        self.dash_yaw_tracker = bool(p.get("DASH_YAW_TRACKER", False))
        self.gazebo_latch = (
            bool(p.get("FUSE_GAZEBO_LATCH", False))
            or self.fuse_midcourse or self.warm_handoff
            or self.coast_search or self.dash_yaw_tracker
        )
        # Cue deliverable until the latch even without fusion (real m4
        # socket semantics; see DEFAULT_DASH_PARAMS). simulate() reads this.
        self.cue_until_latch_flag = self.fuse_midcourse or bool(
            p.get("CUE_UNTIL_LATCH", False)
        )
        self.coast_stale_s = p.get("COAST_STALE_S", 1.0)
        self.coast_acq_range = p.get("COAST_ACQ_RANGE_M", 10.0)
        self.coast_yaw_amp = math.radians(p.get("COAST_SEARCH_YAW_AMP_DEG", 20.0))
        self.coast_period = p.get("COAST_SEARCH_PERIOD_S", 4.0)
        self.coast_budget_s = p.get("COAST_SEARCH_BUDGET_S", 8.0)
        self.coast_active = False      # currently coasting on a stale/dead link
        self.coast_ever = False        # scoring: coast latched at least once
        self.coast_searched = False    # scoring: reached the basket search phase
        self.coast_breakoff = False    # budget expired w/o acquisition -> break off
        self.coast_phase = None        # None | "coast" | "search" (diagnostic)
        # Law-commanded absolute boresight heading for the Sensor (None =
        # the legacy camera-detection-driven boresight). Only ever set when
        # DASH_YAW_TRACKER/COAST_SEARCH is on -- simulate() ignores None, so
        # the default path is untouched.
        self.boresight_cmd = None
        self._last_cue_t = None
        self._coast_loss_t = None
        self.latch_streak = int(p.get("FUSE_LATCH_STREAK", 3))
        self.latched = False
        self._cam_streak = 0
        self._warm_done = False
        if self.fuse_midcourse:
            self.fused = FusedTrack(
                DEFAULT_PN_PARAMS["ALPHA"], DEFAULT_PN_PARAMS["BETA_LAMBDA"],
                DEFAULT_PN_PARAMS["BETA_RANGE"],
                cam_range_frac=RANGE_NOISE_FRAC,
                cue_sigma_base=p.get("FUSE_CUE_SIGMA_BASE_M", 0.4),
                cue_sigma_quad=p.get("FUSE_CUE_SIGMA_QUAD", 0.008),
                datum_budget_m=p.get("FUSE_DATUM_BUDGET_M", 0.5),
                vel_stale_s=p.get("FUSE_VEL_STALE_S", 0.6),
            )
        else:
            self.fused = None

    def step(self, t, dt, own_pos, own_vel, meas):
        if self.gazebo_latch:
            # P-6 path (any fusion-family flag on). The all-defaults
            # baseline never enters here -- the body below is untouched.
            return self._step_fused(t, dt, own_pos, own_vel, meas)
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

    def _coast_update(self, t, meas):
        """m4 --coast-search staleness bookkeeping (only called when
        COAST_SEARCH is on). Mirrors m4_intercept.py exactly:
          - the staleness clock only starts after the FIRST cue receipt
            (m4 guards on last_cue_recv_sim_t is not None -- a cue that
            never existed is a dash-timeout problem, not a coast);
          - cue silent > COAST_STALE_S latches coast_active;
          - a recovered cue (Markov burst over -- a jammer cutoff never
            recovers) un-latches it and clears the loss clock."""
        if meas is not None and meas.source == "cue":
            if self.coast_active:
                self.coast_active = False
                self._coast_loss_t = None
            self._last_cue_t = t
        if (
            not self.coast_active
            and self._last_cue_t is not None
            and (t - self._last_cue_t) > self.coast_stale_s
        ):
            self.coast_active = True
            self.coast_ever = True
            self._coast_loss_t = t

    def _step_fused(self, t, dt, own_pos, own_vel, meas):
        """P-6 path (any of FUSE_MIDCOURSE / WARM_HANDOFF / FUSE_GAZEBO_LATCH
        on; the all-defaults baseline never enters here). Differences vs
        step(), each mirroring the REAL S2 pipeline (m4_intercept.py
        --handoff) rather than the legacy lab approximation:
          1. ONE-WAY LATCH (ADR-0010 #5): handoff = >= FUSE_LATCH_STREAK
             consecutive fresh camera detections with the range estimate
             <= handoff_range, latched forever. The legacy lab handoff (a
             single fresh frame, re-evaluated every tick, able to drop back
             to DASH) both under-modeled the real latch and would -- with
             the fusion window's extended cue -- let a post-CPA camera loss
             resume cue reads inside the terminal, an honesty violation the
             one-way latch makes structurally impossible (post-latch cue
             measurements are nulled at the top of this method, the lab
             mirror of m4_intercept.py's cue_reader=None).
          2. FUSE_MIDCOURSE: the cue keeps DELIVERING below handoff_range
             until the latch (the real pipeline's semantics -- the socket
             stays open until HANDOFF closes it; the legacy lab killed the
             cue at handoff_range, a modeling gap), and through the window
             where both sources exist the DASH aims off the FusedTrack's
             camera-bearing + fused-range polar state instead of the
             Cartesian tracker (removes the cue's cross-LOS noise/datum
             error from the final dash stretch).
          3. WARM_HANDOFF: at the latch tick, the terminal law's filter and
             rate states are initialized from the fused (or, without
             fusion, the dash tracker's) track -- see _warm_init_terminal.
        """
        # One-way honesty latch: post-latch cue data is structurally dropped
        # BEFORE it can touch any tracker/filter.
        if self.latched and meas is not None and meas.source == "cue":
            meas = None

        # ADR-0015 coast-search staleness clock (feature-guarded; no RNG).
        if self.coast_search and not self.latched:
            self._coast_update(t, meas)

        self.tracker.predict(dt)
        if self.fused is not None:
            self.fused.predict(dt)
        if meas is not None:
            self.tracker.correct_full(meas, own_pos, t, latency_compensate=self.latency_compensate)
            if self.fused is not None:
                self.fused.update(meas, t)
            if meas.source == "camera":
                # Consecutive-detection streak; a gap longer than the lock
                # staleness resets it (mirrors m4_intercept.py's
                # HANDOFF_STREAK_MIN new_meas/reset semantics).
                if self.last_camera_t is not None and (t - self.last_camera_t) <= MEAS_STALE_S:
                    self._cam_streak += 1
                else:
                    self._cam_streak = 1
                self.last_camera_t = t

        camera_has_tag = (
            self.last_camera_t is not None and (t - self.last_camera_t) <= MEAS_STALE_S
        )

        # Latch decision BEFORE stepping the terminal law, so a warm init is
        # in place for the very first terminal-owned command.
        if not self.latched and camera_has_tag and self._cam_streak >= self.latch_streak:
            if self.fused is not None and self.fused.rng_f.initialized:
                r_est = self.fused.rng_f.x_hat
            else:
                pos_hat = self.tracker.pos_hat
                r_est = (
                    math.hypot(pos_hat[0] - own_pos[0], pos_hat[1] - own_pos[1])
                    if pos_hat is not None else None
                )
            if r_est is not None and r_est <= self.handoff_range:
                self.latched = True
                if self.warm_handoff and not self._warm_done:
                    self._warm_init_terminal(t, own_pos, own_vel)
                    self._warm_done = True

        # Terminal law stepped every tick (same warm-keeping as step()).
        terminal_cmd = self.terminal.step(t, dt, own_pos, own_vel, meas)
        self.in_terminal = self.latched
        if self.latched:
            # Past the latch the boresight is camera-driven again (m4's
            # ENGAGE yaw law) -- stop commanding it. No-ops at baseline
            # (boresight_cmd is only ever non-None under the new flags).
            self.boresight_cmd = None
            self.coast_phase = None
            self.last_cmd = terminal_cmd
            return terminal_cmd

        # --- DASH (mirrors step()'s dash block; aim source may be fused) ---
        aim_pos = None
        aim_vel = None
        if self.fuse_midcourse and self.fused is not None and self.fused.camera_fresh(t):
            st = self.fused.state(t, own_pos, own_vel, self.tracker.vel_hat)
            if st is not None:
                aim_pos = st["pos"]
                aim_vel = st["vel"]
        if aim_pos is None:
            aim_pos = self.tracker.pos_hat
            aim_vel = self.tracker.vel_hat
        if aim_pos is None:
            return (0.0, 0.0)  # no track yet at all (before the cue's first sample)

        vt = aim_vel or (0.0, 0.0)
        rel = (aim_pos[0] - own_pos[0], aim_pos[1] - own_pos[1])

        # --- ADR-0015 coast-search + m4 dash-yaw boresight (all feature-
        # guarded; the baseline never sets boresight_cmd / coast state). ---
        self.boresight_cmd = None
        self.coast_phase = None
        if self.dash_yaw_tracker:
            # m4 DASH yaw law: boresight at the TRACK azimuth (fused when
            # camera-fresh, else the cue tracker -- same source as the aim).
            self.boresight_cmd = math.atan2(rel[1], rel[0])
        if self.coast_search and self.coast_active:
            pred_range = math.hypot(rel[0], rel[1])
            los_pred = math.atan2(rel[1], rel[0])
            # Break-off budget (checked while un-latched only -- a latch
            # this same tick already returned above, m4's phase priority):
            # no camera acquisition within the budget of link loss -> break
            # off rather than fly blind. simulate() ends the run.
            if (
                self._coast_loss_t is not None
                and (t - self._coast_loss_t) > self.coast_budget_s
            ):
                self.coast_breakoff = True
            if pred_range <= self.coast_acq_range:
                # Bounded seeker search at the predicted basket: HOLD
                # position (velocity 0, first-order lag decelerates) and
                # sweep the boresight +/- amp around the PREDICTED LOS.
                self.coast_phase = "search"
                self.coast_searched = True
                t_since = t - (self._coast_loss_t if self._coast_loss_t is not None else t)
                self.boresight_cmd = wrap_pi(
                    los_pred
                    + self.coast_yaw_amp
                    * math.sin(2.0 * math.pi * t_since / self.coast_period)
                )
                self.last_cmd = (0.0, 0.0)
                return (0.0, 0.0)
            # Far out: dead-reckon COAST -- keep flying the same lead solve
            # below (the tracker's predict() is the dead-reckoner), yaw at
            # the predicted LOS so the seeker is looking down-track.
            self.coast_phase = "coast"
            self.boresight_cmd = los_pred

        t_go = solve_intercept_time(rel, vt, self.dash_speed, self.dash_max_lead_s)
        aim = (aim_pos[0] + vt[0] * t_go, aim_pos[1] + vt[1] * t_go)
        direction = (aim[0] - own_pos[0], aim[1] - own_pos[1])
        dnorm = math.hypot(direction[0], direction[1])
        if dnorm < 1e-6:
            vx, vy = self.last_cmd
        else:
            vx = self.dash_speed * direction[0] / dnorm
            vy = self.dash_speed * direction[1] / dnorm
        self.last_cmd = (vx, vy)
        return (vx, vy)

    def _warm_init_terminal(self, t, own_pos, own_vel):
        """WARM_HANDOFF (ADR-0015 "handoff should be a WARM lock-TRANSFER,
        never a cold acquisition"): initialize the terminal law's estimator
        state from the best track available at the latch instant, instead of
        letting the terminal fly its first seconds on still-converging rate
        states -- the terminal's own alpha-beta rates ramp from 0 through
        exactly the window where the ADR-0009 pathology showed a starved Vc
        gutting a_cmd = N*Vc*lambda_dot.

        Angle states stay CAMERA-owned where they already exist (the
        camera's angle is the best angle in the system -- ADR-0015); what
        gets initialized is the RATE/velocity state, derived geometrically
        from the fused target velocity (cue-emitted when available -- the
        ADR-0015 #1 lever -- else the dash tracker's estimate) and own
        velocity:  Rdot = (vt - vo).u,  lamdot = cross(u, vt - vo)/R."""
        st = None
        if self.fused is not None:
            st = self.fused.state(t, own_pos, own_vel, self.tracker.vel_hat)
        if st is None:
            pos_hat = self.tracker.pos_hat
            if pos_hat is None:
                return  # nothing to warm from (cue never arrived) -- no-op
            rel = (pos_hat[0] - own_pos[0], pos_hat[1] - own_pos[1])
            R = max(math.hypot(rel[0], rel[1]), 1e-3)
            lam = math.atan2(rel[1], rel[0])
            u = (rel[0] / R, rel[1] / R)
            vt = self.tracker.vel_hat or (0.0, 0.0)
            vrel = (vt[0] - own_vel[0], vt[1] - own_vel[1])
            st = dict(
                pos=pos_hat, vel=vt, lam=lam, R=R,
                rdot=vrel[0] * u[0] + vrel[1] * u[1],
                lamdot=(u[0] * vrel[1] - u[1] * vrel[0]) / R,
            )

        term = self.terminal
        lam_f = getattr(term, "lam", None)
        rng_f = getattr(term, "rng_f", None)
        if lam_f is not None and rng_f is not None:
            # PurePN / Pursuit / APN / PNPlusLead: polar lambda/range filters.
            # lambda position stays camera-owned when already initialized
            # (it is, by the latch's own >=3-detection requirement -- the
            # branch below is defensive).
            if not lam_f.initialized:
                lam_f.x_hat = st["lam"]
            lam_f.xdot_hat = st["lamdot"]
            rng_f.x_hat = st["R"]
            rng_f.xdot_hat = st["rdot"]
            if lam_f._last_t is None:
                lam_f._last_t = t
            if rng_f._last_t is None:
                rng_f._last_t = t
            # v_perp: start the lateral integrator at the vehicle's ACTUAL
            # cross-LOS velocity instead of whatever it happened to
            # integrate off dash-phase detections -- the terminal's first
            # commands then continue the established dash course rather
            # than jumping.
            if hasattr(term, "v_perp"):
                u = (math.cos(st["lam"]), math.sin(st["lam"]))
                pvec = (-u[1], u[0])
                v_perp0 = own_vel[0] * pvec[0] + own_vel[1] * pvec[1]
                cap = getattr(term, "v_perp_max", None)
                term.v_perp = clamp(v_perp0, -cap, cap) if cap else v_perp0

        tracker = getattr(term, "tracker", None)
        if tracker is not None:
            # PIP (and APN/PNPlusLead's velocity feedforward): re-anchor the
            # Cartesian track to the fused polar state -- position on the
            # camera LOS, velocity from the fused (cue-emitted when fresh)
            # estimate. Removes any residual cue datum bias from the lead
            # solve at the moment the terminal takes over.
            if isinstance(tracker, TargetTracker):
                for f, pos_c, vel_c in (
                    (tracker.fx, st["pos"][0], st["vel"][0]),
                    (tracker.fy, st["pos"][1], st["vel"][1]),
                ):
                    f.x_hat = pos_c
                    f.xdot_hat = vel_c
                    if f._last_t is None:
                        f._last_t = t
                tracker._prev_vel = (st["vel"][0], st["vel"][1])
                tracker._prev_t = t
            elif isinstance(tracker, KalmanTracker) and tracker.x is not None:
                tracker.x[0] = st["pos"][0]
                tracker.x[1] = st["pos"][1]
                tracker.x[2] = st["vel"][0]
                tracker.x[3] = st["vel"][1]
        if hasattr(term, "_have_meas"):
            term._have_meas = True


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
             t_max=T_MAX_S, dt=DT, perception=None):
    """Run one deterministic (given `seed`) intercept and score it.

    `perception` (ADR-0015, default None -> byte-identical pre-ADR-0015
    baseline) is a partial override of PERCEPTION_DEFAULTS applying the
    real-world cue/seeker data-constraints (range-dependent + datum-biased
    cue, emitted velocity, latency jitter, bursty dropout, jammer link
    cutoff, LOS-rate-driven + ML-grade terminal noise). See that dict.

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
    # ADR-0014 lever 1 (yaw-rate authority): the boresight's own slew-rate
    # cap is this lab's proxy for PX4's MPC_YAWRAUTO_MAX. Sweepable via
    # path_params so a study can raise it without touching the default
    # (YAW_RATE_MAX_DEG_S_DEFAULT=45, unchanged -- baseline-preserving).
    yaw_rate_max_deg_s = path_params.pop("yaw_rate_max_deg_s", YAW_RATE_MAX_DEG_S_DEFAULT)

    builder = PATHS[path_name]
    (tx, ty), vel_fn = builder(speed, rng, **path_params)

    cls, defaults = METHODS[method_name]
    params = {**defaults, **(method_params or {})}
    law = cls(params)

    # P-6 (FUSE_MIDCOURSE) / jam-envelope (CUE_UNTIL_LATCH): the cue channel
    # stays deliverable below handoff_range until the law's one-way latch
    # fires (the real m4_intercept.py semantics -- the cue socket is closed
    # BY the HANDOFF latch, not by a range gate). cue_until_latch_flag covers
    # both FUSE_MIDCOURSE and the explicit CUE_UNTIL_LATCH param; False for
    # every default config, leaving the baseline Sensor delivery condition
    # byte-identical.
    cue_until_latch = bool(
        getattr(law, "cue_until_latch_flag", False)
        or getattr(law, "fuse_midcourse", False)
    )

    # ADR-0015: draw the per-run constant cue datum bias AFTER the path
    # builder (so its rng draw never perturbs geometry) and ONLY when
    # configured (guarded in draw_datum_bias -> zero draws at baseline).
    perc = resolve_perception(perception)
    datum_bias = draw_datum_bias(rng, perc["CUE_DATUM_BIAS_MAG_M"])

    ix = iy = ivx = ivy = 0.0
    # ACQUIRE already happened before this lab's clock starts (m4_intercept.py:
    # hover + yaw-center on the tag BEFORE spawning the mover/engaging) -- so
    # the boresight starts centered on the target's initial position, not at
    # a fixed world heading. See Sensor's own_heading/FOV docstring.
    initial_heading = math.atan2(ty - iy, tx - ix)
    sensor = Sensor(
        rng, det_range=det_range, handoff_range=handoff_range, initial_heading=initial_heading,
        yaw_rate_max_deg_s=yaw_rate_max_deg_s, perception=perception, datum_bias=datum_bias,
        cue_until_latch=cue_until_latch,
    )

    min_range = math.hypot(tx - ix, ty - iy)
    t_min_range = 0.0
    first_hit_time = None
    control_effort = 0.0
    n_ticks = 0
    n_locked_ticks = 0
    n_in_range_ticks = 0
    # ADR-0014 diagnostic: coverage restricted to the TERMINAL band (same
    # TERMINAL_LOST_TAG_RANGE_M=5.0 boundary tag_lost_in_terminal already
    # uses) -- the whole-run detection_coverage below dilutes exactly the
    # phase this study cares about (yaw-rate-limited boresight losing the
    # tag inside the FOV during the endgame), so this is reported alongside
    # it, not in place of it.
    n_in_terminal_ticks = 0
    n_terminal_locked_ticks = 0
    last_meas_t = None
    tag_lost_in_terminal = False
    breakoff_armed = False
    breakoff_increase_streak = 0
    last_true_range = None
    # ADR-0015 constraint-5 diagnostics (scoring-only): did the onboard camera
    # ever get a lock, and when. Combined post-loop with the sensor's own
    # link-cut latch to flag "link cut before the seeker acquired".
    _any_camera_meas = False
    _first_camera_lock_t = None
    _first_camera_lock_range = None  # TRUE range at first camera lock (scoring only)
    _coast_broke_off = False

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

    # --- ADR-0023 Tier-1 latch/ZEM diagnostics (SCORING-ONLY, no RNG --
    # same honesty boundary as min_range/breakoff above; the default paths
    # are numerically untouched). ZEM at the handoff latch = the miss if
    # both vehicles flew ballistically from the latch tick (the
    # docs/terminal_diagnosis.md kinematic-lock metric, item 5):
    # |rel + vrel*t_cpa| at t_cpa = -(rel.vrel)/|vrel|^2 (clamped >= 0).
    # realized_correction (returned below) = zem_at_latch - final miss: how
    # much of the delivered ZEM the terminal actually corrected -- the
    # ~20-25% recoverable mechanization-loss channel Tier-1 targets.
    _t1_prev_in_terminal = False
    _t1_latch_t = None
    _t1_latch_true_range = None
    _t1_zem_at_latch = None

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

        meas = sensor.tick(t, dt, (ix, iy), (tx, ty), two_stage, target_vel=vel_fn(t))
        if meas is not None:
            last_meas_t = t
        if meas is not None and meas.source == "camera":
            _any_camera_meas = True
            if _first_camera_lock_t is None:
                _first_camera_lock_t = t
                _first_camera_lock_range = true_range
        has_lock = last_meas_t is not None and (t - last_meas_t) <= MEAS_STALE_S
        if true_range < det_range:
            n_in_range_ticks += 1
            if has_lock:
                n_locked_ticks += 1
        if true_range < TERMINAL_LOST_TAG_RANGE_M:
            n_in_terminal_ticks += 1
            if has_lock:
                n_terminal_locked_ticks += 1
        if true_range < TERMINAL_LOST_TAG_RANGE_M and not (
            meas is not None and meas.source == "camera"
        ) and not has_lock:
            tag_lost_in_terminal = True

        vcx, vcy = law.step(t, dt, (ix, iy), (ivx, ivy), meas)

        # ADR-0023 Tier-1 diagnostic (scoring-only; see the accumulator
        # comment above): snapshot true-state ZEM the tick the law's
        # dash->terminal latch fires (in_terminal False -> True). Uses the
        # PRE-step vehicle velocity (ivx, ivy) -- the state the terminal
        # inherits at the latch instant.
        _t1_in_term = bool(getattr(law, "in_terminal", False))
        if _t1_in_term and not _t1_prev_in_terminal:
            _t1_latch_t = t
            _t1_latch_true_range = true_range
            _vt_now = vel_fn(t)
            _relx, _rely = tx - ix, ty - iy
            _vrx, _vry = _vt_now[0] - ivx, _vt_now[1] - ivy
            _v2 = _vrx * _vrx + _vry * _vry
            _tcpa = max(0.0, -(_relx * _vrx + _rely * _vry) / _v2) if _v2 > 1e-9 else 0.0
            _t1_zem_at_latch = math.hypot(_relx + _vrx * _tcpa, _rely + _vry * _tcpa)
        _t1_prev_in_terminal = _t1_in_term

        # P-6: the tick the fused law's one-way latch fires, close the cue
        # gate (no further deliveries; in-flight datagrams lost) -- the lab
        # mirror of m4_intercept.py closing the UDP socket at HANDOFF. The
        # law ALSO structurally drops any post-latch cue measurement at the
        # top of _step_fused (belt and suspenders, matching the real
        # pipeline's socket-close + reader-nulling pair).
        if cue_until_latch and not sensor.cue_gate_closed and getattr(law, "latched", False):
            sensor.close_cue()

        # ADR-0015 coast-search hooks (inert at baseline: boresight_cmd is
        # only ever non-None, and coast_breakoff only ever True, under the
        # DASH_YAW_TRACKER/COAST_SEARCH feature flags -- no RNG either way).
        # The law's absolute boresight command (m4's DASH tracker-azimuth
        # yaw / the bounded search sweep) becomes the sensor's desired
        # heading; the sensor still slews it at its own yaw-rate cap, same
        # as the real PX4 yaw setpoint path.
        _bs_cmd = getattr(law, "boresight_cmd", None)
        if _bs_cmd is not None:
            sensor.desired_heading = _bs_cmd
        if getattr(law, "coast_breakoff", False):
            # No camera acquisition within the coast-search budget of link
            # loss -> break off rather than fly blind (m4's
            # link_lost_no_acq outcome). The run scores its min_range so
            # far -- honestly a miss unless the dash already got close.
            _coast_broke_off = True
            break

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
    terminal_coverage = (
        n_terminal_locked_ticks / n_in_terminal_ticks if n_in_terminal_ticks else None
    )
    tracker_pos_err_rms = math.sqrt(_pos_err_sq_sum / _track_diag_n) if _track_diag_n else None
    tracker_vel_err_rms = math.sqrt(_vel_err_sq_sum / _track_diag_n) if _track_diag_n else None

    return dict(
        method=method_name, path=path_name, speed=speed, seed=seed, two_stage=two_stage,
        N=params.get("N"), V_CLOSE=params.get("V_CLOSE"),
        dash_speed=params.get("DASH_SPEED"), handoff_range_cfg=params.get("HANDOFF_RANGE"),
        terminal_method=params.get("TERMINAL_METHOD"),
        miss_distance=min_range, intercept_time=intercept_time,
        control_effort=control_effort, detection_coverage=detection_coverage,
        terminal_coverage=terminal_coverage,
        tag_lost_in_terminal=tag_lost_in_terminal,
        # Track-quality diagnostics (candidate research task, ground-truth
        # SCORING-ONLY -- see comment above the accumulators): None for
        # methods without a `.tracker` (pursuit/pure_pn).
        tracker_pos_err_rms=tracker_pos_err_rms, tracker_vel_err_rms=tracker_vel_err_rms,
        handoff_pos_err=_handoff_pos_err, handoff_vel_err=_handoff_vel_err,
        # ADR-0015 constraint-5 diagnostics (scoring-only).
        link_cut=bool(getattr(sensor, "_cue_link_cut", False)),
        camera_locked=_any_camera_meas,
        first_camera_lock_t=_first_camera_lock_t,
        # link cut by the jammer AND the seeker never acquired -> a
        # dead-reckon-coast miss (the ADR-0015 link-loss-before-lock gap).
        link_cut_no_acquire=bool(getattr(sensor, "_cue_link_cut", False)) and not _any_camera_meas,
        # --- jam-envelope diagnostics (scoring-only; all inert/False/None at
        # baseline -- no feature draws RNG) ---
        first_camera_lock_range=_first_camera_lock_range,
        coast_engaged=bool(getattr(law, "coast_ever", False)),
        coast_searched=bool(getattr(law, "coast_searched", False)),
        coast_breakoff=_coast_broke_off,
        # camera acquired strictly AFTER the jammer cut the link = a
        # successful dead-reckon-coast reacquisition.
        acquired_after_cut=bool(
            getattr(sensor, "_cue_link_cut_t", None) is not None
            and _first_camera_lock_t is not None
            and _first_camera_lock_t > sensor._cue_link_cut_t
        ),
        # --- ADR-0023 Tier-1 latch/ZEM diagnostics (scoring-only; see the
        # accumulator comment). None for laws without an in_terminal latch
        # (everything except two_stage_dash) or runs that never latched. ---
        latch_t=_t1_latch_t,
        latch_true_range=_t1_latch_true_range,
        zem_at_latch=_t1_zem_at_latch,
        realized_correction=(
            _t1_zem_at_latch - min_range if _t1_zem_at_latch is not None else None
        ),
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
        "detection_coverage", "terminal_coverage", "tag_lost_in_terminal",
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


# =========================================================================
# ADR-0014 TERMINAL-GUIDANCE LEVER STUDY ("lab ranks, Gazebo decides", BEFORE
# any Gazebo flight time is spent). Re-diagnosis (seat C, adopted): the S2
# terminal tag loss is NOT FOV overflow -- it's a YAW-RATE deficit (achieved
# ~21-24 deg/s vs required ~32-58 deg/s LOS rotation) walking the boresight
# off a tag that is still well inside the FOV and 100+ px wide. Three levers,
# all opt-in (baseline defaults verified byte-identical above this section):
#   1. yaw-rate authority   -- Sensor's yaw_rate_max_deg_s (the lab's proxy
#      for PX4's MPC_YAWRAUTO_MAX).
#   2. split terminal freeze + lambda_dot rate-cap -- PurePN's SPLIT_FREEZE /
#      LAMBDA_DOT_CAP_DEG_S / LAMBDA_DOT_CAP_A_CMD_ONLY_DEG_S (+ optional
#      LEAD_EXTRAP_MAX_S sub-variant).
#   3. slower terminal closing -- PurePN's V_CLOSE (the V_CLOSE_TERMINAL
#      analog), already a plain sweepable param, no new code needed.
# Evaluated on: (a) the S2 two-stage dash config (dash_speed=10, handoff=10
# -- the lab's own validated best dash/handoff pair, ADR-0011 3rd addendum)
# with TERMINAL_METHOD="pure_pn" (the law lever 2 actually targets -- PIP's
# terminal freeze is a structurally different whole-command lead-pursuit
# latch with no lambda_dot/v_perp at all, so lever 2 does not apply to it);
# (b) a camera-only pure_pn crossing regression check (3/4/6 m/s, both
# directions) so a terminal-only change is confirmed to not quietly break
# the already-validated camera-only numbers.
# =========================================================================
ADR14_S2_SPEEDS = S2_SPEEDS               # 6/8/10 m/s
ADR14_DASH_SPEED = 10.0                   # lab's validated best (ADR-0011 3rd addendum)
ADR14_HANDOFF_RANGE = 10.0
ADR14_REGRESSION_SPEEDS = (3.0, 4.0, 6.0)  # calibration band (ADR-0011 2nd addendum)
ADR14_PK_RANGES = (0.5, 1.0, 1.5, 2.0, 2.5)  # ADR-0014 decision #1's headline set

ADR14_YAW_RATES_DEG_S = (45.0, 60.0, 75.0, 90.0)  # 45.0 = baseline (YAW_RATE_MAX_DEG_S_DEFAULT)

ADR14_LEVER2_VARIANTS = [
    ("baseline (whole-vector freeze)", {}),
    ("split_freeze only", {"SPLIT_FREEZE": True}),
    ("split_freeze + cap_both 60deg/s", {"SPLIT_FREEZE": True, "LAMBDA_DOT_CAP_DEG_S": 60.0}),
    ("split_freeze + cap_both 75deg/s", {"SPLIT_FREEZE": True, "LAMBDA_DOT_CAP_DEG_S": 75.0}),
    # REJECTED-ON-PURPOSE control: cap only the a_cmd read, leave the
    # filter's own forward integration (predict()) uncapped -- included to
    # empirically CONFIRM this recreates the ADR-0009 whipsaw, not to
    # recommend it.
    ("split_freeze + cap_a_cmd_ONLY 60deg/s (expect WORSE)",
     {"SPLIT_FREEZE": True, "LAMBDA_DOT_CAP_A_CMD_ONLY_DEG_S": 60.0}),
    ("split_freeze + cap_both 60 + lead_extrap 0.3s",
     {"SPLIT_FREEZE": True, "LAMBDA_DOT_CAP_DEG_S": 60.0, "LEAD_EXTRAP_MAX_S": 0.3}),
    ("split_freeze + cap_both 60 + lead_extrap 0.5s",
     {"SPLIT_FREEZE": True, "LAMBDA_DOT_CAP_DEG_S": 60.0, "LEAD_EXTRAP_MAX_S": 0.5}),
]

ADR14_V_CLOSE_TERMINAL_GRID = (5.5, 4.5, 4.0)  # 5.5 = baseline (FPV V_CLOSE_TERMINAL)


def _adr14_camera_only_regression(pn_overrides, n_seeds, yaw_rate=None, speeds=ADR14_REGRESSION_SPEEDS):
    """Camera-only pure_pn crossing (both directions) regression rows for a
    given lever config -- the "does this terminal change break the
    already-validated camera-only numbers" check."""
    rows = []
    for path in ("crossing_l2r", "crossing_r2l"):
        for speed in speeds:
            path_params = {"speed": speed}
            if yaw_rate is not None:
                path_params["yaw_rate_max_deg_s"] = yaw_rate
            for seed in range(n_seeds):
                rows.append(simulate("pure_pn", pn_overrides, path, path_params, seed, two_stage=False))
    return rows


def _adr14_s2_dash_config(terminal_params, n_seeds, terminal_method="pure_pn", yaw_rate=None,
                           speeds=ADR14_S2_SPEEDS, dash_speed=ADR14_DASH_SPEED,
                           handoff_range=ADR14_HANDOFF_RANGE):
    """S2 two_stage_dash rows (dash10/handoff10, the lab's validated best
    dash pair) at the FPV target band, with `terminal_params` overriding the
    terminal law's own params (ADR-0014 levers live here)."""
    rows = []
    for speed in speeds:
        method_params = dict(
            DASH_SPEED=dash_speed, HANDOFF_RANGE=handoff_range,
            TERMINAL_METHOD=terminal_method, TERMINAL_PARAMS=terminal_params,
        )
        path_params = dict(S2_PATH_PARAMS, speed=speed, handoff_range=handoff_range)
        if yaw_rate is not None:
            path_params["yaw_rate_max_deg_s"] = yaw_rate
        for seed in range(n_seeds):
            rows.append(
                simulate("two_stage_dash", method_params, "crossing_l2r", path_params, seed, two_stage=True)
            )
    return rows


def _adr14_stats(rows, pk_ranges=None):
    miss = np.array([r["miss_distance"] for r in rows])
    tc_vals = [r["terminal_coverage"] for r in rows if r["terminal_coverage"] is not None]
    tc = np.array(tc_vals) if tc_vals else None
    out = dict(
        n=len(rows),
        miss_mean=float(miss.mean()),
        miss_p90=float(np.percentile(miss, 90)),
        miss_p95=float(np.percentile(miss, 95)),
        terminal_coverage_mean=float(tc.mean()) if tc is not None else None,
    )
    if pk_ranges:
        out["pk"] = {r: float(np.mean(miss <= r)) for r in pk_ranges}
    return out


def _adr14_print_row(label, per_speed_stats, speeds):
    for speed in speeds:
        s = per_speed_stats[speed]
        tc = f"{s['terminal_coverage_mean']:.2f}" if s["terminal_coverage_mean"] is not None else "n/a"
        print(
            f"    {label:46s} {speed:5.1f}  n={s['n']:<4d} "
            f"mean={s['miss_mean']:6.3f}  p90={s['miss_p90']:6.3f}  p95={s['miss_p95']:6.3f}  "
            f"term_cov={tc}"
        )


def run_adr14_lever1_yawrate(n_seeds):
    print("\n" + "=" * 90)
    print("ADR-0014 LEVER 1: yaw-rate authority (Sensor.yaw_rate_max_deg_s, "
          "lab proxy for MPC_YAWRAUTO_MAX)")
    print("=" * 90)
    print(f"  [regression] camera-only pure_pn crossing (l2r+r2l), speeds={ADR14_REGRESSION_SPEEDS}, "
          f"n_seeds={n_seeds}")
    reg_results = {}
    for yaw_rate in ADR14_YAW_RATES_DEG_S:
        rows = _adr14_camera_only_regression(None, n_seeds, yaw_rate=yaw_rate)
        reg_results[yaw_rate] = _adr14_stats(rows)
        r = reg_results[yaw_rate]
        tc = f"{r['terminal_coverage_mean']:.2f}" if r["terminal_coverage_mean"] is not None else "n/a"
        tag = " (baseline)" if yaw_rate == 45.0 else ""
        print(f"    yaw_rate={yaw_rate:5.1f} deg/s{tag:12s} n={r['n']:<5d} "
              f"mean={r['miss_mean']:.3f}  p90={r['miss_p90']:.3f}  term_cov={tc}")

    print(f"\n  [S2 dash config] dash={ADR14_DASH_SPEED}, handoff={ADR14_HANDOFF_RANGE}, "
          f"terminal=pure_pn, speeds={ADR14_S2_SPEEDS}, n_seeds={n_seeds}")
    s2_results = {}
    for yaw_rate in ADR14_YAW_RATES_DEG_S:
        per_speed = {}
        for speed in ADR14_S2_SPEEDS:
            rows = _adr14_s2_dash_config({}, n_seeds, yaw_rate=yaw_rate, speeds=(speed,))
            per_speed[speed] = _adr14_stats(rows)
        s2_results[yaw_rate] = per_speed
        tag = " (baseline)" if yaw_rate == 45.0 else ""
        print(f"    yaw_rate={yaw_rate:5.1f} deg/s{tag}")
        _adr14_print_row("", per_speed, ADR14_S2_SPEEDS)
    return reg_results, s2_results


def run_adr14_lever2_splitfreeze(n_seeds):
    print("\n" + "=" * 90)
    print("ADR-0014 LEVER 2: split terminal freeze + lambda_dot rate-cap (PurePN)")
    print("=" * 90)
    print(f"  [regression] camera-only pure_pn crossing (l2r+r2l), speeds={ADR14_REGRESSION_SPEEDS}, "
          f"n_seeds={n_seeds}")
    reg_results = {}
    for label, overrides in ADR14_LEVER2_VARIANTS:
        rows = _adr14_camera_only_regression(overrides, n_seeds)
        reg_results[label] = _adr14_stats(rows)
        r = reg_results[label]
        tc = f"{r['terminal_coverage_mean']:.2f}" if r["terminal_coverage_mean"] is not None else "n/a"
        print(f"    {label:52s} n={r['n']:<5d} mean={r['miss_mean']:.3f}  p90={r['miss_p90']:.3f}  term_cov={tc}")

    print(f"\n  [S2 dash config] dash={ADR14_DASH_SPEED}, handoff={ADR14_HANDOFF_RANGE}, "
          f"terminal=pure_pn, speeds={ADR14_S2_SPEEDS}, n_seeds={n_seeds}")
    s2_results = {}
    for label, overrides in ADR14_LEVER2_VARIANTS:
        per_speed = {}
        for speed in ADR14_S2_SPEEDS:
            rows = _adr14_s2_dash_config(overrides, n_seeds, speeds=(speed,))
            per_speed[speed] = _adr14_stats(rows)
        s2_results[label] = per_speed
        print(f"    {label}")
        _adr14_print_row("", per_speed, ADR14_S2_SPEEDS)
    return reg_results, s2_results


def run_adr14_lever3_vclose(n_seeds):
    print("\n" + "=" * 90)
    print("ADR-0014 LEVER 3: slower terminal closing speed (PurePN's V_CLOSE)")
    print("=" * 90)
    print(f"  [regression] camera-only pure_pn crossing (l2r+r2l), speeds={ADR14_REGRESSION_SPEEDS}, "
          f"n_seeds={n_seeds}")
    reg_results = {}
    for vc in ADR14_V_CLOSE_TERMINAL_GRID:
        rows = _adr14_camera_only_regression({"V_CLOSE": vc}, n_seeds)
        reg_results[vc] = _adr14_stats(rows)
        r = reg_results[vc]
        tag = " (baseline)" if vc == 5.5 else ""
        print(f"    V_CLOSE={vc:4.1f} m/s{tag:12s} n={r['n']:<5d} mean={r['miss_mean']:.3f}  p90={r['miss_p90']:.3f}")

    print(f"\n  [S2 dash config] dash={ADR14_DASH_SPEED}, handoff={ADR14_HANDOFF_RANGE}, "
          f"terminal=pure_pn, speeds={ADR14_S2_SPEEDS}, n_seeds={n_seeds}")
    s2_results = {}
    for vc in ADR14_V_CLOSE_TERMINAL_GRID:
        per_speed = {}
        for speed in ADR14_S2_SPEEDS:
            rows = _adr14_s2_dash_config({"V_CLOSE": vc}, n_seeds, speeds=(speed,))
            per_speed[speed] = _adr14_stats(rows)
        s2_results[vc] = per_speed
        tag = " (baseline)" if vc == 5.5 else ""
        print(f"    V_CLOSE={vc:4.1f} m/s{tag}")
        _adr14_print_row("", per_speed, ADR14_S2_SPEEDS)
    return reg_results, s2_results


def run_adr14_best_combo(best_terminal_overrides, best_yaw_rate, n_seeds, pk_ranges=ADR14_PK_RANGES):
    """Final evaluation: best-combo (pure_pn terminal + winning levers) vs
    the no-lever pure_pn-terminal baseline vs the previously-best PIP-terminal
    S2 config (ADR-0011 3rd addendum) -- all at dash10/handoff10, n_seeds>=40,
    plus a camera-only pure_pn regression check and a Pk-vs-R preview."""
    print("\n" + "=" * 90)
    print("ADR-0014 BEST-COMBINATION EVALUATION")
    print("=" * 90)
    print(f"  best-combo terminal overrides: {best_terminal_overrides}")
    print(f"  best-combo yaw_rate_max_deg_s: {best_yaw_rate}")
    print(f"  n_seeds={n_seeds}, speeds={ADR14_S2_SPEEDS}, dash={ADR14_DASH_SPEED}, "
          f"handoff={ADR14_HANDOFF_RANGE}")

    configs = [
        ("S2 baseline (pure_pn terminal, no levers)", {}, None, "pure_pn"),
        ("S2 PIP terminal (ADR-0011 3rd addendum winner, no levers)", {}, None, "pip"),
        ("S2 BEST-COMBO (pure_pn terminal + levers)", best_terminal_overrides, best_yaw_rate, "pure_pn"),
    ]

    all_results = {}
    all_rows = {}
    for label, overrides, yaw_rate, terminal_method in configs:
        per_speed = {}
        per_speed_rows = {}
        for speed in ADR14_S2_SPEEDS:
            rows = _adr14_s2_dash_config(
                overrides, n_seeds, terminal_method=terminal_method, yaw_rate=yaw_rate, speeds=(speed,)
            )
            per_speed[speed] = _adr14_stats(rows, pk_ranges=pk_ranges)
            per_speed_rows[speed] = rows
        all_results[label] = per_speed
        all_rows[label] = per_speed_rows
        print(f"\n  {label}")
        for speed in ADR14_S2_SPEEDS:
            s = per_speed[speed]
            tc = f"{s['terminal_coverage_mean']:.2f}" if s["terminal_coverage_mean"] is not None else "n/a"
            pk_str = ", ".join(f"Pk@{r}={s['pk'][r]*100:.0f}%" for r in pk_ranges)
            print(
                f"    speed={speed:4.1f}  n={s['n']:<4d} mean={s['miss_mean']:.3f}  "
                f"p90={s['miss_p90']:.3f}  p95={s['miss_p95']:.3f}  term_cov={tc}"
            )
            print(f"      {pk_str}")

    # regression check on the best combo -- compared APPLES-TO-APPLES against
    # a no-lever run on the SAME crossing_l2r/r2l-only path subset (NOT the
    # ADR-0011 3rd addendum's 1.786 m, which pools 7 different paths -- that
    # number is not a valid baseline for this narrower check).
    print(f"\n  [regression] camera-only pure_pn crossing (l2r+r2l) w/ best-combo params, "
          f"speeds={ADR14_REGRESSION_SPEEDS}, n_seeds={n_seeds}")
    reg_rows = _adr14_camera_only_regression(best_terminal_overrides, n_seeds, yaw_rate=best_yaw_rate)
    reg_stats = _adr14_stats(reg_rows)
    reg_baseline_rows = _adr14_camera_only_regression(None, n_seeds)
    reg_baseline_stats = _adr14_stats(reg_baseline_rows)
    print(f"    no-lever baseline (same path subset): mean={reg_baseline_stats['miss_mean']:.3f}  "
          f"p90={reg_baseline_stats['miss_p90']:.3f}")
    print(f"    best-combo:                            mean={reg_stats['miss_mean']:.3f}  "
          f"p90={reg_stats['miss_p90']:.3f}")

    return all_results, all_rows


def run_adr14_study(n_seeds=120, combo_n_seeds=150):
    """Top-level ADR-0014 lever study driver -- CLI-invokable via --adr0014.
    Runs levers 1/2/3 independently (rank them), assembles the best
    combination from the winners, then evaluates the combo against the S2
    baseline and the prior PIP-terminal winner with a Pk-vs-R preview.
    Returns (all lever-study rows, best-combo overrides, best yaw rate) so
    main() can fold the rows into the run's CSV."""
    t0 = time.perf_counter()
    reg1, s2_1 = run_adr14_lever1_yawrate(n_seeds)
    reg2, s2_2 = run_adr14_lever2_splitfreeze(n_seeds)
    reg3, s2_3 = run_adr14_lever3_vclose(n_seeds)
    print(f"\n[guidance_lab] ADR-0014 levers 1-3 done in {time.perf_counter() - t0:.2f}s")

    # Pick winners by pooled (mean over speeds) S2 dash-config miss.
    def _pooled_mean(per_speed_by_key):
        return {
            k: float(np.mean([per_speed_by_key[k][sp]["miss_mean"] for sp in ADR14_S2_SPEEDS]))
            for k in per_speed_by_key
        }

    yaw_pooled = _pooled_mean(s2_1)
    best_yaw = min(yaw_pooled, key=yaw_pooled.get)
    lever2_pooled = _pooled_mean(s2_2)
    best_lever2_label = min(lever2_pooled, key=lever2_pooled.get)
    best_lever2_overrides = dict(next(o for lbl, o in ADR14_LEVER2_VARIANTS if lbl == best_lever2_label))
    vclose_pooled = _pooled_mean(s2_3)
    best_vclose = min(vclose_pooled, key=vclose_pooled.get)

    print("\n" + "=" * 90)
    print("ADR-0014 LEVER WINNERS (by pooled S2 dash-config mean miss, 6/8/10 m/s)")
    print("=" * 90)
    print(f"  Lever 1 (yaw rate):   best={best_yaw} deg/s   pooled_means={yaw_pooled}")
    print(f"  Lever 2 (split/cap):  best='{best_lever2_label}'   pooled_means={lever2_pooled}")
    print(f"  Lever 3 (V_CLOSE):    best={best_vclose} m/s   pooled_means={vclose_pooled}")

    best_combo_overrides = dict(best_lever2_overrides)
    best_combo_overrides["V_CLOSE"] = best_vclose
    best_yaw_rate = best_yaw if best_yaw != 45.0 else None  # None = baseline default, no override needed

    combo_results, combo_rows = run_adr14_best_combo(best_combo_overrides, best_yaw_rate, combo_n_seeds)

    all_rows = []
    for src in (
        _adr14_camera_only_regression(None, n_seeds),  # re-collected for CSV completeness (cheap)
    ):
        all_rows.extend(src)
    for per_speed_rows in combo_rows.values():
        for rows in per_speed_rows.values():
            all_rows.extend(rows)

    return all_rows, best_combo_overrides, best_yaw_rate


# =========================================================================
# ADR-0015 PERCEPTION-CONSTRAINT COST STUDY ("lab ranks, Gazebo decides",
# ADR-0015 build-plan step 3). Measures how much each real-world cue/seeker
# data-constraint (docs/decisions.md ADR-0015's data-constraints table) costs
# intercept success (Pk), on the S2 two-stage dash config (dash10/handoff10,
# the ADR-0011-3rd-addendum validated best) at the FPV target band 6/8/10 m/s.
# Sections:
#   (A) idealized baseline (perception off) -- terminal PIP (S2 default) + pure_pn.
#   (B) EACH constraint toggled ON ALONE -> a SENSITIVITY RANKING of what
#       hurts Pk most (tells the hardware where to spend effort).
#   (C) the COMBINED "realistic" preset -- how far realism drops the honest
#       Pk-vs-R curve vs the idealized baseline.
#   (D) PN-vs-PIP under realistic velocity quality (constraint #2): does PIP
#       still win when the cue EMITS a filtered velocity, and does it collapse
#       when the drone must DIFFERENTIATE the noisy/biased cue position?
# baseline defaults verified byte-identical above this section (perception
# defaults to None everywhere the main/S2/ADR-0014 studies call simulate()).
# =========================================================================
ADR15_SPEEDS = (6.0, 8.0, 10.0)
ADR15_DASH_SPEED = 10.0
ADR15_HANDOFF_RANGE = 10.0
ADR15_PATH = "crossing_l2r"
ADR15_PK_RANGES = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)

# The COMBINED "realistic" cue/seeker model (un-mitigated cue: standard-GPS
# datum bias, no shared RTK). CUE_EMIT_VELOCITY is toggled per-variant by the
# driver (the honest headline uses emit=True -- a real ground node runs a
# tracker anyway; emit=False is the "cue sends position only" degradation).
# Magnitudes chosen from the ADR-0015 table, scaled to this lab's compressed
# ~15 m standoff (see the report's "recommended default realistic values").
ADR15_REALISTIC = dict(
    CUE_SIGMA_BASE_M=0.4, CUE_SIGMA_QUAD=0.008,   # sigma_R ~0.9 m @8m, ~2.2 m @15m
    CUE_DATUM_BIAS_MAG_M=2.5,                       # standard-GPS common-mode (2-3 m)
    CUE_VEL_SIGMA_M_S=0.5,                          # emitted-velocity noise (used iff emit on)
    CUE_LATENCY_JITTER_S=0.05,                      # on top of the 0.12 s mean
    CUE_DROPOUT_MARKOV=True, CUE_DROPOUT_P_OUT=0.12, CUE_DROPOUT_BURST_S=1.5,
    CUE_LINK_CUTOFF_RANGE_M=11.5,                   # jammer cuts mid-dash, before camera lock
    TERM_LOS_RATE_DROPOUT=True, TERM_LOS_RATE_REF_DEG_S=90.0, TERM_LOS_RATE_DROPOUT_MAX=0.7,
    TERM_BEARING_SIGMA_DEG=1.5, TERM_RANGE_NOISE_FRAC=0.22,   # ML box vs AprilTag
)

# The ADR-0015-DESIGN "realistic + mitigations" model: the recommended
# perception architecture actually gets built (shared RTK base + PPS ->
# ~0.5 m datum bias, ground node EMITS filtered velocity, OOSM latency comp).
# Terminal seeker physics (constraints 6,7) and link jamming (4,5) are NOT
# fixable by ground infra, so they stay. Shows how much of the realism hit
# the ADR-0015 design recovers.
ADR15_REALISTIC_MITIGATED = dict(
    ADR15_REALISTIC,
    CUE_DATUM_BIAS_MAG_M=0.5,      # shared RTK base
    CUE_EMIT_VELOCITY=True,        # ground node emits velocity
    CUE_LINK_CUTOFF_RANGE_M=None,  # assume NOT jammed in this variant
)

# Each single constraint at its honest un-mitigated realistic magnitude,
# toggled onto the idealized baseline (constraint 2 is a MITIGATION, analyzed
# in section D, not a standalone degradation).
ADR15_SINGLE = [
    ("c1a cue sigma_R(R)",        dict(CUE_SIGMA_BASE_M=0.4, CUE_SIGMA_QUAD=0.008)),
    ("c1b datum bias 2.5 m",      dict(CUE_DATUM_BIAS_MAG_M=2.5)),
    ("c1  sigma_R + datum bias",  dict(CUE_SIGMA_BASE_M=0.4, CUE_SIGMA_QUAD=0.008, CUE_DATUM_BIAS_MAG_M=2.5)),
    ("c3  latency jitter 0.05 s", dict(CUE_LATENCY_JITTER_S=0.05)),
    ("c4  bursty dropout",        dict(CUE_DROPOUT_MARKOV=True, CUE_DROPOUT_P_OUT=0.12, CUE_DROPOUT_BURST_S=1.5)),
    ("c5  link cutoff 11.5 m",    dict(CUE_LINK_CUTOFF_RANGE_M=11.5)),
    ("c6  LOS-rate term dropout", dict(TERM_LOS_RATE_DROPOUT=True)),
    ("c7  ML terminal noise",     dict(TERM_BEARING_SIGMA_DEG=1.5, TERM_RANGE_NOISE_FRAC=0.22)),
]


def _adr15_rows(perception, terminal_method, speed, n_seeds, emit_velocity=None):
    """n_seeds two_stage_dash runs at one speed on the S2 dash config
    (dash10/handoff10), with the given perception override + terminal law."""
    perc = None if perception is None else dict(perception)
    if emit_velocity is not None:
        perc = dict(perc or {})
        perc["CUE_EMIT_VELOCITY"] = emit_velocity
    method_params = dict(
        DASH_SPEED=ADR15_DASH_SPEED, HANDOFF_RANGE=ADR15_HANDOFF_RANGE,
        TERMINAL_METHOD=terminal_method,
    )
    path_params = dict(S2_PATH_PARAMS, speed=speed, handoff_range=ADR15_HANDOFF_RANGE)
    rows = []
    for seed in range(n_seeds):
        rows.append(
            simulate("two_stage_dash", method_params, ADR15_PATH, path_params, seed,
                     two_stage=True, perception=perc)
        )
    return rows


def _adr15_all_speeds(perception, terminal_method, n_seeds, emit_velocity=None, speeds=ADR15_SPEEDS):
    out = {}
    for sp in speeds:
        out[sp] = _adr15_rows(perception, terminal_method, sp, n_seeds, emit_velocity=emit_velocity)
    return out


def _adr15_pk(rows, pk_ranges=ADR15_PK_RANGES):
    miss = np.array([r["miss_distance"] for r in rows])
    tc = [r["terminal_coverage"] for r in rows if r["terminal_coverage"] is not None]
    return dict(
        n=len(rows), mean=float(miss.mean()), p90=float(np.percentile(miss, 90)),
        pk={R: float(np.mean(miss <= R)) for R in pk_ranges},
        term_cov=float(np.mean(tc)) if tc else None,
        link_cut_no_acq=float(np.mean([r["link_cut_no_acquire"] for r in rows])),
    )


def _adr15_print_speed_block(label, per_speed):
    print(f"\n  {label}")
    for sp in ADR15_SPEEDS:
        s = _adr15_pk(per_speed[sp])
        tc = f"{s['term_cov']:.2f}" if s["term_cov"] is not None else "n/a"
        print(
            f"    speed={sp:4.1f}  n={s['n']:<3d}  mean={s['mean']:5.2f}  p90={s['p90']:5.2f}  "
            f"Pk@1={s['pk'][1.0]*100:3.0f}%  Pk@2={s['pk'][2.0]*100:3.0f}%  "
            f"term_cov={tc}  cut_no_acq={s['link_cut_no_acq']*100:.0f}%"
        )


def _adr15_pk_curve_line(per_speed):
    """One Pk-vs-R row per speed, all R in ADR15_PK_RANGES."""
    for sp in ADR15_SPEEDS:
        s = _adr15_pk(per_speed[sp])
        pk = "  ".join(f"R{R:>3.1f}={s['pk'][R]*100:3.0f}%" for R in ADR15_PK_RANGES)
        print(f"    speed={sp:4.1f}  mean={s['mean']:5.2f}  p90={s['p90']:5.2f}   {pk}")


def run_adr0015_study(n_seeds=60):
    all_rows = []
    t0 = time.perf_counter()

    # --- (A) idealized baseline -------------------------------------------
    print("\n" + "=" * 92)
    print("ADR-0015 (A) IDEALIZED BASELINE (perception OFF) -- S2 dash10/handoff10, Pk-vs-R")
    print("=" * 92)
    base = {}
    for term in ("pip", "pure_pn"):
        base[term] = _adr15_all_speeds(None, term, n_seeds)
        for ps in base[term].values():
            all_rows.extend(ps)
        print(f"\n  terminal={term}")
        _adr15_pk_curve_line(base[term])
    base_pip = base["pip"]

    # --- (B) single-constraint sensitivity (PIP terminal = S2 default) -----
    print("\n" + "=" * 92)
    print("ADR-0015 (B) SENSITIVITY: each constraint toggled ON ALONE (PIP terminal)")
    print("=" * 92)
    single = {}
    for label, perc in ADR15_SINGLE:
        single[label] = _adr15_all_speeds(perc, "pip", n_seeds)
        for ps in single[label].values():
            all_rows.extend(ps)
        _adr15_print_speed_block(label, single[label])

    # sensitivity ranking: pooled-over-speed Pk drop vs idealized baseline
    def pooled(per_speed):
        return _adr15_pk([r for sp in ADR15_SPEEDS for r in per_speed[sp]])
    base_pooled = pooled(base_pip)
    print("\n" + "-" * 92)
    print("  SENSITIVITY RANKING (pooled 6/8/10 m/s, PIP terminal) -- Pk drop vs idealized baseline")
    print(f"    baseline pooled: mean={base_pooled['mean']:.2f}  "
          f"Pk@1={base_pooled['pk'][1.0]*100:.0f}%  Pk@2={base_pooled['pk'][2.0]*100:.0f}%")
    print("-" * 92)
    ranking = []
    for label, _ in ADR15_SINGLE:
        s = pooled(single[label])
        d1 = base_pooled["pk"][1.0] - s["pk"][1.0]
        d2 = base_pooled["pk"][2.0] - s["pk"][2.0]
        dmean = s["mean"] - base_pooled["mean"]
        ranking.append((label, s, d1, d2, dmean))
    ranking.sort(key=lambda x: (x[3], x[2]), reverse=True)  # worst Pk@2 damage first
    print(f"    {'constraint':28s} {'mean':>6s} {'dMean':>7s} {'Pk@1':>6s} {'dPk@1':>7s} "
          f"{'Pk@2':>6s} {'dPk@2':>7s}")
    for label, s, d1, d2, dmean in ranking:
        print(f"    {label:28s} {s['mean']:6.2f} {dmean:+7.2f} "
              f"{s['pk'][1.0]*100:5.0f}% {-d1*100:+6.0f}% {s['pk'][2.0]*100:5.0f}% {-d2*100:+6.0f}%")

    # --- (C) combined realistic (emit=True headline + differentiate) -------
    print("\n" + "=" * 92)
    print("ADR-0015 (C) COMBINED REALISTIC vs idealized baseline -- Pk-vs-R, both terminals")
    print("=" * 92)
    combined = {}
    for term in ("pip", "pure_pn"):
        for emit in (True, False):
            key = (term, "emit" if emit else "differentiate")
            combined[key] = _adr15_all_speeds(ADR15_REALISTIC, term, n_seeds, emit_velocity=emit)
            for ps in combined[key].values():
                all_rows.extend(ps)
    # mitigated design variant (shared RTK + emit + no jam), PIP terminal
    mitig_pip = _adr15_all_speeds(ADR15_REALISTIC_MITIGATED, "pip", n_seeds)
    for ps in mitig_pip.values():
        all_rows.extend(ps)

    print("\n  --- idealized baseline (PIP terminal) ---")
    _adr15_pk_curve_line(base_pip)
    print("\n  --- COMBINED REALISTIC, PIP terminal, cue EMITS velocity (honest headline) ---")
    _adr15_pk_curve_line(combined[("pip", "emit")])
    print("\n  --- COMBINED REALISTIC, PIP terminal, drone DIFFERENTIATES cue ---")
    _adr15_pk_curve_line(combined[("pip", "differentiate")])
    print("\n  --- REALISTIC + ADR-0015 MITIGATIONS (RTK 0.5 m + emit vel + no jam), PIP terminal ---")
    _adr15_pk_curve_line(mitig_pip)

    # --- (D) PN vs PIP under realistic velocity quality --------------------
    print("\n" + "=" * 92)
    print("ADR-0015 (D) PN-vs-PIP under REALISTIC velocity quality (constraint #2)")
    print("=" * 92)
    for term in ("pip", "pure_pn"):
        _adr15_print_speed_block(f"terminal={term}, cue EMITS velocity", combined[(term, "emit")])
        _adr15_print_speed_block(f"terminal={term}, drone DIFFERENTIATES", combined[(term, "differentiate")])

    print(f"\n[guidance_lab] ADR-0015 study done in {time.perf_counter() - t0:.2f}s "
          f"({len(all_rows)} runs)")
    return all_rows


# =========================================================================
# P-6 MID-COURSE FUSION + WARM HANDOFF STUDY (ADR-0015 fusion decision;
# docs/next.md P-6). "lab ranks, Gazebo decides" -- this ranks the four
# fusion x warm variants; the winner still owes a Gazebo A/B (do NOT put a
# lab number in an ADR as a result). Builder methodology mandate (docs/next.md):
# evaluate under THREE perception tiers and treat any variant that only wins
# under BEST as FRAGILE.
#
# 2x2 variant grid (fusion off/on x warm off/on). All four share the SAME
# Gazebo-faithful one-way latch (FUSE_GAZEBO_LATCH), so the off/off cell is
# NOT the legacy TwoStageDash -- it is "real latch, no fusion, no warm",
# which is the honest baseline for isolating fusion's/warm's effect (the
# legacy-latch S2 numbers live in the ADR-0011/0014/0015 sections). Run on
# the S2 dash config (dash10/handoff10) at the FPV band 6/8/10 m/s.
# =========================================================================
P6_SPEEDS = (6.0, 8.0, 10.0)
P6_DASH_SPEED = 10.0
P6_HANDOFF_RANGE = 10.0
P6_PATH = "crossing_l2r"
P6_PK_RANGES = (1.0, 2.0)

# Terminal laws for the P-6 A/B, evaluated separately:
#   pip     -- the ADR-0011-3rd-addendum / ADR-0015 two-stage winner (its lead
#              solve is exactly what a warm-initialized clean velocity feeds),
#              and the S2 default terminal. NB: PIP uses NO lambda_dot, so warm
#              handoff's rate init mostly re-anchors its Cartesian track.
#   pure_pn -- where warm handoff's lambda_dot/Rdot init actually bites: its
#              a_cmd = N*Vc*lambda_dot rides camera-only lambda/range filters
#              that start cold at the latch, exactly the ADR-0009 starved-Vc
#              window the warm init targets.
P6_TERMINALS = ("pip", "pure_pn")

# Three perception tiers (docs/next.md worse-than-ideal mandate). Magnitudes are
# the ADR-0015 recommended realistic defaults, scaled to this lab's
# compressed ~15 m standoff (same basis as ADR15_REALISTIC).
P6_TIERS = {
    # BEST: idealized -- perception off entirely (the ADR-0015 (A) baseline).
    "BEST": None,
    # EXPECTED: ADR-0015's recommended realistic + designed-mitigations cue
    # (shared-RTK 0.5 m datum, EMIT velocity sigma_v=0.5, latency 0.12+0.05
    # jitter, bursty Markov dropout p 0.12/burst 1.5 s) + realistic terminal
    # seeker (LOS-rate dropout + ML-grade noise). Link NOT jammed (a designed
    # comms link that stays up to lock).
    "EXPECTED": dict(
        CUE_SIGMA_BASE_M=0.4, CUE_SIGMA_QUAD=0.008,
        CUE_DATUM_BIAS_MAG_M=0.5,
        CUE_EMIT_VELOCITY=True, CUE_VEL_SIGMA_M_S=0.5,
        CUE_LATENCY_MEAN_S=0.12, CUE_LATENCY_JITTER_S=0.05,
        CUE_DROPOUT_MARKOV=True, CUE_DROPOUT_P_OUT=0.12, CUE_DROPOUT_BURST_S=1.5,
        TERM_LOS_RATE_DROPOUT=True, TERM_LOS_RATE_REF_DEG_S=90.0, TERM_LOS_RATE_DROPOUT_MAX=0.7,
        TERM_BEARING_SIGMA_DEG=1.5, TERM_RANGE_NOISE_FRAC=0.22,
    ),
    # WORST-CREDIBLE: standard-GPS 2.5 m datum (poisons a naive cue-angle
    # fusion -- the fragility test), sigma_v=1.0, latency jitter 0.08,
    # dropout p 0.20 / burst 3 s, jammer link cutoff ~11.5 m (before lock),
    # terminal LOS-rate dropout ON, ML terminal noise ON.
    "WORST": dict(
        CUE_SIGMA_BASE_M=0.4, CUE_SIGMA_QUAD=0.008,
        CUE_DATUM_BIAS_MAG_M=2.5,
        CUE_EMIT_VELOCITY=True, CUE_VEL_SIGMA_M_S=1.0,
        CUE_LATENCY_MEAN_S=0.12, CUE_LATENCY_JITTER_S=0.08,
        CUE_DROPOUT_MARKOV=True, CUE_DROPOUT_P_OUT=0.20, CUE_DROPOUT_BURST_S=3.0,
        CUE_LINK_CUTOFF_RANGE_M=11.5,
        TERM_LOS_RATE_DROPOUT=True, TERM_LOS_RATE_REF_DEG_S=90.0, TERM_LOS_RATE_DROPOUT_MAX=0.7,
        TERM_BEARING_SIGMA_DEG=1.5, TERM_RANGE_NOISE_FRAC=0.22,
    ),
}
P6_TIER_ORDER = ("BEST", "EXPECTED", "WORST")

# fusion off/on x warm off/on -- all four carry the Gazebo-faithful latch.
P6_VARIANTS = [
    ("fuse=off warm=off", dict(FUSE_GAZEBO_LATCH=True)),
    ("fuse=on  warm=off", dict(FUSE_MIDCOURSE=True)),
    ("fuse=off warm=on ", dict(WARM_HANDOFF=True)),
    ("fuse=on  warm=on ", dict(FUSE_MIDCOURSE=True, WARM_HANDOFF=True)),
]
# Bearing-weighted-fusion is the mechanization FusedTrack already implements
# (the cue never touches the angle). The WORST tier's 2.5 m datum is the
# direct test of the ADR-0015 warning that a biased cue can poison a clean
# camera bearing -- if fusion still holds up at WORST, that IS the answer to
# "does bearing-weighting fix the poisoning". Reported in the verdict.


def _p6_rows(perception, variant_overrides, speed, n_seeds, terminal):
    """n_seeds two_stage_dash runs at one speed on the S2 dash config with a
    fusion variant + a perception tier + a terminal law."""
    method_params = dict(
        DASH_SPEED=P6_DASH_SPEED, HANDOFF_RANGE=P6_HANDOFF_RANGE,
        TERMINAL_METHOD=terminal, **variant_overrides,
    )
    path_params = dict(S2_PATH_PARAMS, speed=speed, handoff_range=P6_HANDOFF_RANGE)
    perc = None if perception is None else dict(perception)
    rows = []
    for seed in range(n_seeds):
        rows.append(
            simulate("two_stage_dash", method_params, P6_PATH, path_params, seed,
                     two_stage=True, perception=perc)
        )
    return rows


def _p6_stats(rows, pk_ranges=P6_PK_RANGES):
    miss = np.array([r["miss_distance"] for r in rows])
    tc = [r["terminal_coverage"] for r in rows if r["terminal_coverage"] is not None]
    return dict(
        n=len(rows), mean=float(miss.mean()), median=float(np.median(miss)),
        p90=float(np.percentile(miss, 90)),
        pk={R: float(np.mean(miss <= R)) for R in pk_ranges},
        term_cov=float(np.mean(tc)) if tc else None,
    )


# Datum-poisoning probe: the WORST tier with the JAMMER LINK CUTOFF REMOVED
# (so the 2.5 m standard-GPS datum-biased cue survives INTO the fusion window,
# where the camera is also tracking) -- WORST's own 11.5 m cutoff otherwise
# kills the cue before the camera acquires (~8 m), leaving no both-sources
# window and masking exactly the datum-poisoning question the task asks. If
# fuse=on still doesn't LOSE here vs fuse=off, that IS the evidence that the
# bearing-weighted mechanization (camera owns the angle; cue range folded in
# only inverse-variance-weighted) prevents a biased cue from poisoning the
# clean seeker bearing.
P6_POISON_PROBE = {k: v for k, v in P6_TIERS["WORST"].items() if k != "CUE_LINK_CUTOFF_RANGE_M"}


def _p6_tier_block(results, terminal, n_seeds, all_rows):
    """Run + print all tiers x variants x speeds for one terminal, filling
    results[tier][label][speed] and extending all_rows."""
    print("\n" + "#" * 100)
    print(f"#  TERMINAL = {terminal}")
    print("#" * 100)
    for tier in P6_TIER_ORDER:
        perception = P6_TIERS[tier]
        results[tier] = {}
        print("\n" + "-" * 100)
        print(f"  TIER: {tier}"
              + ("  (idealized -- perception OFF)" if perception is None else ""))
        print("-" * 100)
        print(f"    {'variant':18s} {'speed':>5s}  {'n':>3s}  {'mean':>5s} "
              f"{'p90':>5s}  {'Pk@1':>5s} {'Pk@2':>5s}  {'term_cov':>8s}")
        for label, overrides in P6_VARIANTS:
            results[tier][label] = {}
            for sp in P6_SPEEDS:
                rows = _p6_rows(perception, overrides, sp, n_seeds, terminal)
                all_rows.extend(rows)
                s = _p6_stats(rows)
                results[tier][label][sp] = s
                tc = f"{s['term_cov']:.2f}" if s["term_cov"] is not None else "n/a"
                print(f"    {label:18s} {sp:5.1f}  {s['n']:>3d}  {s['mean']:5.2f} "
                      f"{s['p90']:5.2f}  {s['pk'][1.0]*100:4.0f}% {s['pk'][2.0]*100:4.0f}%  "
                      f"{tc:>8s}")


def _p6_verdict_block(results, terminal):
    """Pooled-over-speed verdict + fragility summary for one terminal.
    Returns the pooled verdict dict for the caller's cross-terminal summary."""
    print("\n" + "=" * 100)
    print(f"P-6 VERDICT (terminal={terminal}): pooled-over-speed mean miss + Pk@2 "
          "per (tier x variant), delta vs fuse=off/warm=off baseline of the SAME tier")
    print("=" * 100)
    print(f"    {'tier':9s} {'variant':18s} {'mean':>6s} {'dMean':>7s}  "
          f"{'Pk@1':>5s} {'Pk@2':>5s} {'dPk@2':>7s}")
    verdict = {}
    for tier in P6_TIER_ORDER:
        base_label = P6_VARIANTS[0][0]
        base_mean = float(np.mean([results[tier][base_label][sp]["mean"] for sp in P6_SPEEDS]))
        base_pk2 = float(np.mean([results[tier][base_label][sp]["pk"][2.0] for sp in P6_SPEEDS]))
        verdict[tier] = {}
        for label, _ in P6_VARIANTS:
            mean = float(np.mean([results[tier][label][sp]["mean"] for sp in P6_SPEEDS]))
            pk1 = float(np.mean([results[tier][label][sp]["pk"][1.0] for sp in P6_SPEEDS]))
            pk2 = float(np.mean([results[tier][label][sp]["pk"][2.0] for sp in P6_SPEEDS]))
            verdict[tier][label] = dict(mean=mean, pk1=pk1, pk2=pk2)
            print(f"    {tier:9s} {label:18s} {mean:6.2f} {mean - base_mean:+7.2f}  "
                  f"{pk1*100:4.0f}% {pk2*100:4.0f}% {(pk2 - base_pk2)*100:+6.0f}%")
        print()
    print("-" * 100)
    print(f"  FRAGILITY (terminal={terminal}; a variant winning ONLY under BEST "
          "is fragile; a WORST-tier loss vs baseline is a real negative result):")
    for label, _ in P6_VARIANTS[1:]:
        parts = []
        for tier in P6_TIER_ORDER:
            base = verdict[tier][P6_VARIANTS[0][0]]["mean"]
            m = verdict[tier][label]["mean"]
            rel = (base - m) / base * 100.0 if base > 1e-6 else 0.0
            parts.append(f"{tier} {rel:+5.1f}%")
        print(f"    {label}:  " + "   ".join(parts)
              + "   (mean-miss improvement vs same-tier baseline; >~15% real, <15% noise)")
    return verdict


def run_p6_study(n_seeds=80):
    """P-6 fusion + warm-handoff A/B across three perception tiers, for BOTH
    the pip and pure_pn terminals, plus a datum-poisoning probe. Returns all
    rows (for the CSV) so main() can write them."""
    all_rows = []
    t0 = time.perf_counter()
    print("\n" + "=" * 100)
    print("P-6 MID-COURSE FUSION + WARM HANDOFF (ADR-0015 fusion decision) -- "
          f"S2 dash{P6_DASH_SPEED:.0f}/handoff{P6_HANDOFF_RANGE:.0f}, "
          f"terminals={P6_TERMINALS}, {n_seeds} seeds/cell")
    print("=" * 100)
    print("  All four variants carry the SAME Gazebo-faithful one-way latch "
          "(streak>=3, cue nulled post-latch);")
    print("  the fuse=off/warm=off cell is the honest baseline for isolating "
          "fusion's/warm's effect,")
    print("  NOT the legacy TwoStageDash (whose numbers are in the "
          "ADR-0011/0014/0015 sections).")

    verdicts = {}
    for terminal in P6_TERMINALS:
        results = {}
        _p6_tier_block(results, terminal, n_seeds, all_rows)
        verdicts[terminal] = _p6_verdict_block(results, terminal)

    # --- datum-poisoning probe (pip terminal, WORST minus link cutoff): does a
    # LIVE 2.5 m-datum cue poison the fused bearing? fuse=off vs fuse=on. ---
    print("\n" + "=" * 100)
    print("P-6 DATUM-POISONING PROBE (terminal=pip, WORST tier MINUS the 11.5 m "
          "link cutoff, so the 2.5 m-datum cue is ALIVE through the fusion window)")
    print("=" * 100)
    print("  Answers ADR-0015's fragility warning directly: if fuse=on does NOT lose "
          "vs fuse=off here,")
    print("  the bearing-weighted mechanization (camera owns the angle) is what "
          "prevents datum poisoning.")
    print(f"    {'variant':18s} {'speed':>5s}  {'n':>3s}  {'mean':>5s} {'p90':>5s}  "
          f"{'Pk@1':>5s} {'Pk@2':>5s}  {'term_cov':>8s}")
    probe = {}
    for label, overrides in (P6_VARIANTS[0], P6_VARIANTS[1]):  # fuse=off/off, fuse=on/off
        probe[label] = {}
        for sp in P6_SPEEDS:
            rows = _p6_rows(P6_POISON_PROBE, overrides, sp, n_seeds, "pip")
            all_rows.extend(rows)
            s = _p6_stats(rows)
            probe[label][sp] = s
            tc = f"{s['term_cov']:.2f}" if s["term_cov"] is not None else "n/a"
            print(f"    {label:18s} {sp:5.1f}  {s['n']:>3d}  {s['mean']:5.2f} "
                  f"{s['p90']:5.2f}  {s['pk'][1.0]*100:4.0f}% {s['pk'][2.0]*100:4.0f}%  {tc:>8s}")
    off_mean = float(np.mean([probe[P6_VARIANTS[0][0]][sp]["mean"] for sp in P6_SPEEDS]))
    on_mean = float(np.mean([probe[P6_VARIANTS[1][0]][sp]["mean"] for sp in P6_SPEEDS]))
    rel = (off_mean - on_mean) / off_mean * 100.0 if off_mean > 1e-6 else 0.0
    print(f"  POISON-PROBE POOLED: fuse=off mean={off_mean:.2f} m, fuse=on mean={on_mean:.2f} m "
          f"-> fusion {rel:+.1f}% (positive = fusion HELPS even with a live 2.5 m datum; "
          "negative = the datum poisons -> a real negative result)")

    print(f"\n[guidance_lab] P-6 study done in {time.perf_counter() - t0:.2f}s "
          f"({len(all_rows)} runs)")
    return all_rows


# =========================================================================
# JAMMER LINK-CUTOFF ENVELOPE STUDY (--jam-envelope): the number the
# comms-denied headline hangs on. ADR-0015 established ONE point (an 11.5 m
# cutoff on the compressed ~15 m lab standoff cost -26% Pk@2 -- the worst
# single degradation) and the design constraint R_acquire >= R_cutoff +
# coast_margin; ADR-0018's honest limit is that at WORST the cutoff precedes
# camera acquisition so the fusion window never opens. This study maps the
# WHOLE cliff: Pk vs R_cutoff x coast-search{off,on} x fusion{off,on} at the
# EXPECTED tier (+ a WORST column), and extracts (a) the cliff-edge cutoff,
# (b) the tolerated cue-denied coast distance, (c) whether fusion moves the
# cliff, (d) the data-backed form of the R_acquire/R_cutoff requirement.
#
# GEOMETRY -- extended ~45 m runway, NOT the compressed 15.4 m S2 standoff:
# cutoffs at 20-40 m are meaningless on a 15.4 m start (the cue would die
# at t=0, i.e. "never had a cue", collapsing every such cell into one), so
# the envelope needs a runway longer than its largest cutoff. 45 m is also
# more faithful to the real engagement (ADR-0017's ground rig tracks to
# ~150 m; a jammer engages in the tens of meters). Consequence: these cells
# are NOT comparable to the ADR-0015/P-6 tables (different geometry) -- the
# no-cut column IS the study's own baseline. NB the 11.5 m ADR-0015 point
# lands differently here: on a 45 m runway the track is mature by 11.5 m,
# so its -26% was substantially a SHORT-RUNWAY artifact (cut at 11.5 m of
# 15.4 m = an infant track), which this study quantifies explicitly.
#
# CUE SIGMA -- uses ADR-0017's CORRECTED noise/datum split, not the hand-set
# sigma_R = 0.4 + 0.008 R^2 the compressed-geometry studies used: at 40+ m
# that curve gives a 13+ m sigma (ADR-0017 measured it ~180x too steep as
# stereo noise -- physics says ~0.45 m at 100 m), which would turn the
# envelope into a cue-noise study instead of a link-cutoff study. ADR-0017
# says adopt (a,c) EXPECTED=(0.000, 4.45e-05) / WORST=(0.214, 1.94e-04)
# with datum bias 0.5 / 2.5 m "at the next cue-model change" -- this is it.
# Everything else matches the P-6 tier presets (emit-velocity ON per
# ADR-0015's #1 lever -- also exactly what makes dead-reckoning coast work).
#
# All cells run the Gazebo-faithful one-way latch + m4 cue-socket semantics
# (CUE_UNTIL_LATCH) + m4 dash yaw (DASH_YAW_TRACKER -- see that param's
# comment; an attribution probe below shows its isolated effect), so the
# coast{off,on} axis isolates ONLY the ADR-0015 coast/search/break-off
# branch, and fuse{off,on} only the FusedTrack aim.
# =========================================================================
JAM_SPEEDS = (6.0, 8.0, 10.0)
JAM_DASH_SPEED = 10.0
JAM_HANDOFF_RANGE = 10.0
JAM_PATH = "crossing_l2r"
JAM_PATH_PARAMS = dict(x0=6.5, y0_mag=44.5)   # sqrt(6.5^2+44.5^2) ~= 45.0 m start
JAM_CUTOFFS = (None, 40.0, 30.0, 25.0, 20.0, 15.0, 11.5, 9.0)
JAM_WORST_CUTOFFS = (None, 20.0, 15.0, 11.5, 9.0)   # where the cliff threatens
JAM_WEAVE_CUTOFFS = (None, 30.0, 20.0, 9.0)
JAM_PK2_DROP = 0.10   # ">10 pts Pk@2 drop" = the cliff / coast-margin criterion

JAM_TIERS = dict(
    # EXPECTED: the P-6 EXPECTED preset with ADR-0017's corrected stereo
    # noise + datum split (see the header comment). Emit-velocity ON.
    EXPECTED=dict(
        CUE_SIGMA_BASE_M=0.0, CUE_SIGMA_QUAD=4.45e-05,
        CUE_DATUM_BIAS_MAG_M=0.5,
        CUE_EMIT_VELOCITY=True, CUE_VEL_SIGMA_M_S=0.5,
        CUE_LATENCY_MEAN_S=0.12, CUE_LATENCY_JITTER_S=0.05,
        CUE_DROPOUT_MARKOV=True, CUE_DROPOUT_P_OUT=0.12, CUE_DROPOUT_BURST_S=1.5,
        TERM_LOS_RATE_DROPOUT=True, TERM_LOS_RATE_REF_DEG_S=90.0,
        TERM_LOS_RATE_DROPOUT_MAX=0.7,
        TERM_BEARING_SIGMA_DEG=1.5, TERM_RANGE_NOISE_FRAC=0.22,
    ),
    # WORST-CREDIBLE: ADR-0017 WORST stereo curve + 2.5 m standard-GPS
    # datum, sigma_v=1.0, worse latency jitter, p_out 0.20 / 3 s bursts.
    WORST=dict(
        CUE_SIGMA_BASE_M=0.214, CUE_SIGMA_QUAD=1.94e-04,
        CUE_DATUM_BIAS_MAG_M=2.5,
        CUE_EMIT_VELOCITY=True, CUE_VEL_SIGMA_M_S=1.0,
        CUE_LATENCY_MEAN_S=0.12, CUE_LATENCY_JITTER_S=0.08,
        CUE_DROPOUT_MARKOV=True, CUE_DROPOUT_P_OUT=0.20, CUE_DROPOUT_BURST_S=3.0,
        TERM_LOS_RATE_DROPOUT=True, TERM_LOS_RATE_REF_DEG_S=90.0,
        TERM_LOS_RATE_DROPOUT_MAX=0.7,
        TERM_BEARING_SIGMA_DEG=1.5, TERM_RANGE_NOISE_FRAC=0.22,
    ),
)


def _jam_rows(tier_name, cutoff, coast, fuse, speed, n_seeds, terminal,
              path=JAM_PATH, path_params=None, dash_yaw=True):
    """n_seeds two_stage_dash runs for one envelope cell, rows stamped with
    the cell coordinates for the CSV. Seeds are always range(n_seeds)."""
    perc = dict(JAM_TIERS[tier_name])
    if cutoff is not None:
        perc["CUE_LINK_CUTOFF_RANGE_M"] = cutoff
    mp = dict(
        DASH_SPEED=JAM_DASH_SPEED, HANDOFF_RANGE=JAM_HANDOFF_RANGE,
        TERMINAL_METHOD=terminal,
        FUSE_GAZEBO_LATCH=True,        # honest S2 one-way latch, all cells
        CUE_UNTIL_LATCH=True,          # m4 cue-socket semantics, all cells
        DASH_YAW_TRACKER=dash_yaw,     # m4 dash yaw semantics (see probe)
    )
    if fuse:
        mp["FUSE_MIDCOURSE"] = True
    if coast:
        mp["COAST_SEARCH"] = True
    pp = dict(path_params if path_params is not None else JAM_PATH_PARAMS,
              speed=speed, handoff_range=JAM_HANDOFF_RANGE)
    rows = []
    for seed in range(n_seeds):
        r = simulate("two_stage_dash", mp, path, pp, seed, two_stage=True,
                     perception=perc)
        r.update(
            jam_tier=tier_name, jam_cutoff=cutoff,
            jam_coast=int(coast), jam_fuse=int(fuse),
            jam_dash_yaw=int(dash_yaw), jam_path=path,
        )
        rows.append(r)
    return rows


def _jam_stats(rows):
    miss = np.array([r["miss_distance"] for r in rows])
    tc = [r["terminal_coverage"] for r in rows if r["terminal_coverage"] is not None]
    acq = [r["first_camera_lock_range"] for r in rows
           if r["first_camera_lock_range"] is not None]
    return dict(
        n=len(rows), mean=float(miss.mean()), p90=float(np.percentile(miss, 90)),
        pk1=float(np.mean(miss <= 1.0)), pk2=float(np.mean(miss <= 2.0)),
        term_cov=float(np.mean(tc)) if tc else None,
        lock=float(np.mean([r["camera_locked"] for r in rows])),
        reacq=float(np.mean([r["acquired_after_cut"] for r in rows])),
        brk=float(np.mean([r["coast_breakoff"] for r in rows])),
        acq_range=float(np.mean(acq)) if acq else None,
    )


def _jam_cut_label(cutoff):
    return "no-cut" if cutoff is None else f"{cutoff:g} m"


def _jam_print_header():
    print(f"    {'cutoff':>7s}  {'n':>3s}  {'mean':>5s} {'p90':>5s}  "
          f"{'Pk@1':>5s} {'Pk@2':>5s}  {'Pk@2 @6/8/10':>14s}  "
          f"{'t_cov':>5s} {'lock':>5s} {'reacq':>5s} {'brkoff':>6s} {'acq_R':>5s}")


def _jam_print_cell(cutoff, pooled, per_speed):
    tc = f"{pooled['term_cov']:.2f}" if pooled["term_cov"] is not None else "  n/a"
    aq = f"{pooled['acq_range']:5.2f}" if pooled["acq_range"] is not None else "  n/a"
    pspd = "/".join(f"{per_speed[sp]['pk2'] * 100:3.0f}" for sp in JAM_SPEEDS)
    print(f"    {_jam_cut_label(cutoff):>7s}  {pooled['n']:>3d}  "
          f"{pooled['mean']:5.2f} {pooled['p90']:5.2f}  "
          f"{pooled['pk1'] * 100:4.0f}% {pooled['pk2'] * 100:4.0f}%  "
          f"{pspd:>14s}  {tc:>5s} {pooled['lock'] * 100:4.0f}% "
          f"{pooled['reacq'] * 100:4.0f}% {pooled['brk'] * 100:5.0f}% {aq}")


def _jam_config_sweep(tier, cutoffs, coast, fuse, terminal, n_seeds, all_rows,
                      speeds=JAM_SPEEDS):
    """Run cutoffs x speeds for one (tier, coast, fuse, terminal) config.
    Returns {cutoff: pooled_stats} and prints the table block."""
    out = {}
    print(f"\n  --- tier={tier}  terminal={terminal}  "
          f"coast-search={'ON ' if coast else 'off'}  "
          f"fusion={'ON ' if fuse else 'off'} ---")
    _jam_print_header()
    for cutoff in cutoffs:
        per_speed = {}
        cell_rows = []
        for sp in speeds:
            rows = _jam_rows(tier, cutoff, coast, fuse, sp, n_seeds, terminal)
            all_rows.extend(rows)
            cell_rows.extend(rows)
            per_speed[sp] = _jam_stats(rows)
        pooled = _jam_stats(cell_rows)
        out[cutoff] = pooled
        _jam_print_cell(cutoff, pooled, per_speed)
    return out


def _jam_cliff(stats_by_cutoff, cutoffs):
    """Cliff extraction for one config: largest cutoff still tolerated
    (pooled Pk@2 within JAM_PK2_DROP of the no-cut baseline, with every
    smaller tested cutoff also tolerated -- a prefix in ascending order),
    plus ALL failing cutoffs (an isolated small-cutoff fail with larger
    ones passing is visible seed noise, not hidden by the prefix rule).
    Returns (base_pk2, edge, fails)."""
    base = stats_by_cutoff[None]["pk2"]
    finite = sorted(c for c in cutoffs if c is not None)
    fails = [c for c in finite if base - stats_by_cutoff[c]["pk2"] > JAM_PK2_DROP]
    edge = None
    for c in finite:
        if c in fails:
            break
        edge = c
    return base, edge, fails


def run_jam_envelope(n_seeds=80):
    """The jammer link-cutoff envelope study. Returns all rows for the CSV."""
    all_rows = []
    t0 = time.perf_counter()
    print("\n" + "=" * 100)
    print(f"JAMMER LINK-CUTOFF ENVELOPE (ADR-0015 'Handoff continuity') -- "
          f"~45 m runway, S2 dash{JAM_DASH_SPEED:.0f}/handoff{JAM_HANDOFF_RANGE:.0f}, "
          f"{n_seeds} seeds/cell (seeds 0..{n_seeds - 1})")
    print("=" * 100)
    print("  All cells: Gazebo-faithful one-way latch + m4 cue-until-latch + m4 dash yaw;")
    print("  EXPECTED tier = ADR-0015 realistic defaults WITH emit-velocity + ADR-0017")
    print("  corrected sigma_R split. Geometry differs from the ADR-0015/P-6 tables --")
    print("  compare cells to THIS study's no-cut column only.")

    # --- (A) EXPECTED envelope: cutoff x coast x fuse, PIP terminal --------
    print("\n" + "#" * 100)
    print("#  (A) EXPECTED tier, terminal=pip: cutoff x coast-search x fusion")
    print("#" * 100)
    exp_pip = {}
    for coast in (False, True):
        for fuse in (False, True):
            exp_pip[(coast, fuse)] = _jam_config_sweep(
                "EXPECTED", JAM_CUTOFFS, coast, fuse, "pip", n_seeds, all_rows)

    # --- (B) EXPECTED, pure_pn terminal (fuse=off only -- cheapness cut:
    # the fusion axis is already covered on PIP, and ADR-0018 found fusion
    # tier-consistent across both terminals) ---------------------------------
    print("\n" + "#" * 100)
    print("#  (B) EXPECTED tier, terminal=pure_pn (fusion=off only): cutoff x coast-search")
    print("#" * 100)
    exp_pn = {}
    for coast in (False, True):
        exp_pn[(coast, False)] = _jam_config_sweep(
            "EXPECTED", JAM_CUTOFFS, coast, False, "pure_pn", n_seeds, all_rows)

    # --- (C) WORST column at the threatening cutoffs, PIP -------------------
    print("\n" + "#" * 100)
    print("#  (C) WORST tier, terminal=pip: cutoff x coast-search x fusion")
    print("#" * 100)
    worst_pip = {}
    for coast in (False, True):
        for fuse in (False, True):
            worst_pip[(coast, fuse)] = _jam_config_sweep(
                "WORST", JAM_WORST_CUTOFFS, coast, fuse, "pip", n_seeds, all_rows)

    # --- (D) maneuvering-target honesty probe: the crossing target is
    # constant-velocity, which is the BEST case for a dead-reckon coast (the
    # frozen velocity stays true). s_weave breaks that assumption -- the
    # coast tolerance below is an upper bound and this probe shows how much
    # a weaving target erodes it. ------------------------------------------
    print("\n" + "#" * 100)
    print("#  (D) MANEUVER PROBE: s_weave target @8 m/s, EXPECTED, pip, fusion=off")
    print("#      (CV-target coast tolerance is an UPPER BOUND; weave breaks dead-reckoning)")
    print("#" * 100)
    weave_pp = dict(x0=6.5, y0=-44.5)   # same ~45 m start as the crossing cells
    for coast in (False, True):
        print(f"\n  --- s_weave  coast-search={'ON ' if coast else 'off'} ---")
        _jam_print_header()
        for cutoff in JAM_WEAVE_CUTOFFS:
            rows = _jam_rows("EXPECTED", cutoff, coast, False, 8.0, n_seeds,
                             "pip", path="s_weave", path_params=weave_pp)
            all_rows.extend(rows)
            s = _jam_stats(rows)
            _jam_print_cell(cutoff, s, {sp: s for sp in JAM_SPEEDS})

    # --- (E) attribution probe: DASH_YAW_TRACKER off vs on (coast off,
    # fuse off) -- shows how much of any coast-cell gain is just "the
    # boresight follows the track" (the m4 semantics every cell above uses)
    # rather than the coast/search branch itself. ---------------------------
    print("\n" + "#" * 100)
    print("#  (E) ATTRIBUTION PROBE: m4 dash yaw (DASH_YAW_TRACKER) off vs on,")
    print("#      EXPECTED, pip, coast=off, fuse=off, speeds 6+10 pooled")
    print("#" * 100)
    print(f"    {'cutoff':>7s}  {'yaw':>3s}  {'n':>3s}  {'mean':>5s} {'p90':>5s}  "
          f"{'Pk@1':>5s} {'Pk@2':>5s}  {'lock':>5s} {'acq_R':>5s}")
    for cutoff in (None, 40.0):
        for dash_yaw in (False, True):
            rows = []
            for sp in (6.0, 10.0):
                rows.extend(_jam_rows("EXPECTED", cutoff, False, False, sp,
                                      n_seeds, "pip", dash_yaw=dash_yaw))
            all_rows.extend(rows)
            s = _jam_stats(rows)
            aq = f"{s['acq_range']:5.2f}" if s["acq_range"] is not None else "  n/a"
            print(f"    {_jam_cut_label(cutoff):>7s}  {'ON ' if dash_yaw else 'off'}  "
                  f"{s['n']:>3d}  {s['mean']:5.2f} {s['p90']:5.2f}  "
                  f"{s['pk1'] * 100:4.0f}% {s['pk2'] * 100:4.0f}%  "
                  f"{s['lock'] * 100:4.0f}% {aq}")

    # --- VERDICT ------------------------------------------------------------
    print("\n" + "=" * 100)
    print("ENVELOPE VERDICT (pooled 6/8/10 m/s; cliff criterion: pooled Pk@2 "
          f"drop > {JAM_PK2_DROP * 100:.0f} pts vs the SAME config's no-cut)")
    print("=" * 100)
    verdicts = {}
    for label, table, cutoffs in (
        ("EXPECTED pip", exp_pip, JAM_CUTOFFS),
        ("EXPECTED pure_pn", exp_pn, JAM_CUTOFFS),
        ("WORST pip", worst_pip, JAM_WORST_CUTOFFS),
    ):
        for (coast, fuse), stats in table.items():
            base, edge, fails = _jam_cliff(stats, cutoffs)
            acq = stats[None]["acq_range"]
            margin = (edge - acq) if (edge is not None and acq is not None) else None
            key = (label, coast, fuse)
            verdicts[key] = dict(base=base, edge=edge, fails=fails, acq=acq, margin=margin)
            fail_txt = ("none tested" if not fails
                        else ",".join(_jam_cut_label(c) for c in fails))
            print(f"  {label:18s} coast={'ON ' if coast else 'off'} "
                  f"fuse={'ON ' if fuse else 'off'}:  no-cut Pk@2={base * 100:3.0f}%  "
                  f"tolerated cutoff <= {_jam_cut_label(edge) if edge is not None else 'NONE':>7s}"
                  f"  fails: {fail_txt}"
                  + (f"  R_acq~{acq:.1f} m  coast margin~{margin:+.1f} m"
                     if margin is not None else ""))
    print("\n  Reading the margin: 'coast margin' = tolerated_cutoff - measured "
          "acquisition range, i.e. how many")
    print("  meters of cue-denied dead-reckon coast the system absorbs before "
          "the >10-pt Pk@2 cliff. In")
    print("  ADR-0015's constraint form R_acquire >= R_cutoff + coast_margin, "
          "the data supports R_acquire >=")
    print("  R_cutoff - X with X = the printed margin (a NEGATIVE required "
          "margin: acquisition may happen X m")
    print("  INSIDE the cutoff radius because the coast covers the gap).")

    print(f"\n[guidance_lab] jam-envelope study done in "
          f"{time.perf_counter() - t0:.2f}s ({len(all_rows)} runs)")
    return all_rows


def write_jam_csv(rows, path):
    fieldnames = [
        "method", "path", "speed", "seed", "two_stage",
        "dash_speed", "handoff_range_cfg", "terminal_method",
        "jam_tier", "jam_cutoff", "jam_coast", "jam_fuse", "jam_dash_yaw", "jam_path",
        "miss_distance", "intercept_time", "control_effort",
        "detection_coverage", "terminal_coverage", "tag_lost_in_terminal",
        "camera_locked", "first_camera_lock_t", "first_camera_lock_range",
        "link_cut", "link_cut_no_acquire", "acquired_after_cut",
        "coast_engaged", "coast_searched", "coast_breakoff",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k) for k in fieldnames})


# =========================================================================
# ADR-0023 TIER-1 GUIDANCE RECLAIM STUDY (--tier1): the free-software
# reclaim of the ~20-25% RECOVERABLE mechanization loss the terminal
# diagnosis quantified (docs/terminal_diagnosis.md items 5-6: the freeze
# latch discards the last ~0.20 s = 0.17 m of correction capacity, and the
# alpha-beta filters realize only 0.28 m of an available 0.72 m). Three
# levers, A/B'd individually AND in every combination (2^3), each one
# already implementable on a real Pi (per-tick scalar math only):
#
#   A  EARLY HANDOFF -- engage the camera-only terminal law at the FIRST
#      solid detection streak instead of the current 3-streak latch.
#      Capacity scales t_go^2 (diagnosis item 6), so latching ~1-1.5 m
#      earlier is free correction capacity. "Solid" reuses the EXISTING
#      latch streak logic (FUSE_LATCH_STREAK, mirroring m4_intercept.py's
#      HANDOFF_STREAK_MIN) at a reduced count chosen by the pre-sweep below
#      -- an AprilTag/fiducial detection has a near-zero false-positive
#      rate (ADR-0003), so the 3-streak's job was never spoof rejection,
#      only settling; the pre-sweep measures what that settling is worth.
#      HONESTY: the ADR-0010 #5 one-way cue-socket latch is UNTOUCHED --
#      an earlier latch closes the cue SOONER (reads strictly LESS cue
#      data), never longer.
#   B  SPLIT-FREEZE + LATER FREEZE -- ADR-0014 lever 2: freeze only the
#      singular part (the v_perp magnitude) instead of the whole command
#      vector; v_close (monotone in r_hat, never singular) and lambda_hat
#      stay live; |lambda_dot| capped in BOTH a_cmd and the filter's own
#      predict() (AlphaBetaFilter.rate_cap -- one clamp covers both, the
#      anti-whipsaw requirement from ADR-0009/0014). "Later" = a smaller
#      TERMINAL_FREEZE_RANGE, reclaiming the discarded pre-CPA correction
#      window; cap + freeze range chosen by the pre-sweep.
#   C  WARM-SETTLED TERMINAL FILTERS -- ADR-0018's already-built
#      WARM_HANDOFF seeding (geometric lambda_dot/Rdot + v_perp + track
#      re-anchor from the pre-terminal dash track at the latch instant).
#      Alone it measured sub-noise at the 3-streak latch (ADR-0018); the
#      Tier-1 question is whether it matters MORE under lever A, where the
#      terminal starts one detection into the streak instead of three.
#
# Grid: 8 variants x BEST/EXPECTED/WORST perception tiers (reusing the P-6
# presets -- the EXPECTED tier IS "ADR-0015 realistic defaults WITH
# emit-velocity"; the cue model is deliberately HELD CONSTANT at the same
# hand-set sigma_R the p6/EMIT batches used, NOT ADR-0017's corrected
# constants -- changing two things would confound the A/B) x 6/8/10 m/s
# x >= 80 seeds, terminal pure_pn (the Gazebo-validated robust law,
# ADR-0011 addendum -- the mc_batch arms this study gates fly pronav).
# Per-cell metrics add the diagnosis's own kinematic bookkeeping:
# latch range, ZEM@latch, and realized terminal correction
# (ZEM@latch - miss). Verdict rule: a lab win < ~15% is noise; a lever
# that wins at BEST/EXPECTED but hurts at WORST is fragile (worse-than-
# ideal mandate) and is called out as such.
# =========================================================================
T1_SPEEDS = (6.0, 8.0, 10.0)
T1_PK_RANGES = (1.0, 2.0)
T1_TERMINAL = "pure_pn"
T1_PRESWEEP_SPEED = 6.0        # the Gazebo-batch speed (mc_p6/EMIT arms)
# Pre-sweep candidate grids (EXPECTED tier, 6 m/s). Values bracket the
# ADR-0014 proposals: cap 60-90 deg/s ("~60-75, tune"), freeze range from
# the FPV 3.5 m baseline down to 1.5 m (the diagnosis's 0.20-s-to-go latch
# happened at ~3 m ground truth; 2.0 m at |v_rel|~12 m/s is ~0.17 s).
T1_A_STREAKS = (1, 2)
T1_B_FREEZES = (3.5, 2.5, 2.0, 1.5)
T1_B_CAPS = (60.0, 75.0, 90.0)


def _t1_method_params(overrides, terminal_overrides):
    """Assemble two_stage_dash params for one Tier-1 variant. Every variant
    (including the base) carries the FULL Gazebo-faithful latch chain:
      - FUSE_GAZEBO_LATCH: the real one-way streak latch;
      - DASH_YAW_TRACKER: m4's actual DASH yaw law (boresight at the track
        azimuth all through the dash -- without it the lab boresight holds
        its INITIAL heading until first detection, and on a 6-10 m/s
        crosser roughly half the runs never even acquire the streak, which
        would corrupt a latch-TIMING lever like A at the source);
      - CUE_UNTIL_LATCH: m4's cue-socket semantics (closed BY the latch,
        not by a range gate) -- under lever A an earlier latch then closes
        the cue STRICTLY SOONER, the honest direction (ADR-0010 #5).
    All three are constant across the 8 variants, so the A/B isolates the
    levers. NB: this makes the tier-1 base cell NOT number-comparable to
    the P-6 study's base cells (which lacked the last two flags)."""
    mp = dict(
        DASH_SPEED=P6_DASH_SPEED, HANDOFF_RANGE=P6_HANDOFF_RANGE,
        TERMINAL_METHOD=T1_TERMINAL, FUSE_GAZEBO_LATCH=True,
        DASH_YAW_TRACKER=True, CUE_UNTIL_LATCH=True,
    )
    mp.update(overrides or {})
    if terminal_overrides:
        mp["TERMINAL_PARAMS"] = dict(terminal_overrides)
    return mp


def _t1_rows(perception, overrides, terminal_overrides, speed, n_seeds):
    path_params = dict(S2_PATH_PARAMS, speed=speed, handoff_range=P6_HANDOFF_RANGE)
    mp = _t1_method_params(overrides, terminal_overrides)
    perc = None if perception is None else dict(perception)
    return [
        simulate("two_stage_dash", mp, P6_PATH, path_params, seed,
                 two_stage=True, perception=perc)
        for seed in range(n_seeds)
    ]


def _t1_stats(rows):
    s = _p6_stats(rows, pk_ranges=T1_PK_RANGES)
    zems = [r["zem_at_latch"] for r in rows if r.get("zem_at_latch") is not None]
    rcs = [r["realized_correction"] for r in rows if r.get("realized_correction") is not None]
    lrs = [r["latch_true_range"] for r in rows if r.get("latch_true_range") is not None]
    s["zem"] = float(np.mean(zems)) if zems else None
    s["realized"] = float(np.mean(rcs)) if rcs else None
    s["latch_r"] = float(np.mean(lrs)) if lrs else None
    s["n_latched"] = len(lrs)
    return s


def _t1_cell_line(label, speed, s):
    def fmt(v, w=5, d=2):
        return ("%*.*f" % (w, d, v)) if v is not None else " " * (w - 3) + "n/a"
    tc = f"{s['term_cov']:.2f}" if s["term_cov"] is not None else " n/a"
    return (f"    {label:22s} {speed:5.1f}  {s['n']:>3d} {s['n_latched']:>4d}  "
            f"{s['mean']:5.2f} {s['p90']:5.2f}  {s['pk'][1.0]*100:4.0f}% "
            f"{s['pk'][2.0]*100:4.0f}%  {tc:>5s}  {fmt(s['latch_r'])} "
            f"{fmt(s['zem'])} {fmt(s['realized'])}")


def _t1_header():
    return (f"    {'variant':22s} {'speed':>5s}  {'n':>3s} {'nlat':>4s}  "
            f"{'mean':>5s} {'p90':>5s}  {'Pk@1':>5s} {'Pk@2':>5s}  "
            f"{'tcov':>5s}  {'latchR':>5s} {'ZEM@l':>5s} {'realzd':>5s}")


def run_tier1_presweep(n_seeds):
    """Pick lever A's streak and lever B's (cap, freeze) at the EXPECTED
    tier, 6 m/s (the Gazebo-batch cell), before the main 2^3 table. Returns
    (a_streak, b_cap, b_freeze). Selection = mean miss (Pk@1 tiebreak)."""
    perception = P6_TIERS["EXPECTED"]
    print("\n" + "-" * 100)
    print(f"  TIER-1 PRE-SWEEP (EXPECTED tier, {T1_PRESWEEP_SPEED:.0f} m/s, "
          f"{n_seeds} seeds/cell): pick lever A streak + lever B cap/freeze")
    print("-" * 100)
    print(_t1_header())

    base_rows = _t1_rows(perception, None, None, T1_PRESWEEP_SPEED, n_seeds)
    base = _t1_stats(base_rows)
    print(_t1_cell_line("base (streak=3)", T1_PRESWEEP_SPEED, base))

    a_results = {}
    for streak in T1_A_STREAKS:
        rows = _t1_rows(perception, dict(FUSE_LATCH_STREAK=streak), None,
                        T1_PRESWEEP_SPEED, n_seeds)
        s = _t1_stats(rows)
        a_results[streak] = s
        print(_t1_cell_line(f"A: latch streak={streak}", T1_PRESWEEP_SPEED, s))
    a_streak = min(a_results, key=lambda k: (a_results[k]["mean"], -a_results[k]["pk"][1.0]))

    print()
    b_results = {}
    for freeze in T1_B_FREEZES:
        for cap in T1_B_CAPS:
            tp = dict(SPLIT_FREEZE=True, LAMBDA_DOT_CAP_DEG_S=cap,
                      TERMINAL_FREEZE_RANGE=freeze)
            rows = _t1_rows(perception, None, tp, T1_PRESWEEP_SPEED, n_seeds)
            s = _t1_stats(rows)
            b_results[(cap, freeze)] = s
            print(_t1_cell_line(f"B: split fr={freeze} cap={cap:.0f}",
                                T1_PRESWEEP_SPEED, s))
    b_cap, b_freeze = min(
        b_results, key=lambda k: (b_results[k]["mean"], -b_results[k]["pk"][1.0])
    )
    print(f"\n  PRE-SWEEP PICKS: A streak={a_streak} "
          f"(mean {a_results[a_streak]['mean']:.2f} vs base {base['mean']:.2f}); "
          f"B cap={b_cap:.0f} deg/s freeze={b_freeze} m "
          f"(mean {b_results[(b_cap, b_freeze)]['mean']:.2f})")
    return a_streak, b_cap, b_freeze


def run_tier1_study(n_seeds=80, presweep_seeds=80):
    """The full Tier-1 A/B: 8 lever combos x 3 tiers x 3 speeds x n_seeds,
    pure_pn terminal on the S2 dash config. Returns (all_rows, picks) so
    main() can write the CSV."""
    all_rows = []
    t0 = time.perf_counter()
    print("\n" + "=" * 100)
    print("ADR-0023 TIER-1 GUIDANCE RECLAIM (--tier1): early-handoff x "
          "split-freeze x warm-terminal, 2^3 combos")
    print(f"  S2 dash{P6_DASH_SPEED:.0f}/handoff{P6_HANDOFF_RANGE:.0f}, "
          f"terminal={T1_TERMINAL}, {n_seeds} seeds/cell, tiers BEST/EXPECTED/WORST "
          "(P-6 presets; cue model held constant -- NOT ADR-0017's corrected sigma)")
    print("=" * 100)

    a_streak, b_cap, b_freeze = run_tier1_presweep(presweep_seeds)

    lever_a = dict(FUSE_LATCH_STREAK=a_streak)
    lever_b_tp = dict(SPLIT_FREEZE=True, LAMBDA_DOT_CAP_DEG_S=b_cap,
                      TERMINAL_FREEZE_RANGE=b_freeze)
    lever_c = dict(WARM_HANDOFF=True)

    variants = []
    for combo in range(8):
        use_a, use_b, use_c = bool(combo & 1), bool(combo & 2), bool(combo & 4)
        label = "base" if combo == 0 else "".join(
            ch for ch, on in (("A", use_a), ("B", use_b), ("C", use_c)) if on
        )
        overrides = {}
        if use_a:
            overrides.update(lever_a)
        if use_c:
            overrides.update(lever_c)
        variants.append((label, overrides, dict(lever_b_tp) if use_b else None))
    # Order: base, A, B, C, AB, AC, BC, ABC (readability)
    order = {"base": 0, "A": 1, "B": 2, "C": 3, "AB": 4, "AC": 5, "BC": 6, "ABC": 7}
    variants.sort(key=lambda v: order[v[0]])

    results = {}
    for tier in P6_TIER_ORDER:
        perception = P6_TIERS[tier]
        results[tier] = {}
        print("\n" + "-" * 100)
        print(f"  TIER: {tier}"
              + ("  (idealized -- perception OFF)" if perception is None else ""))
        print("-" * 100)
        print(_t1_header())
        for label, overrides, term_over in variants:
            results[tier][label] = {}
            for sp in T1_SPEEDS:
                rows = _t1_rows(perception, overrides, term_over, sp, n_seeds)
                for r in rows:
                    r["tier1_variant"] = label
                    r["tier"] = tier
                    r["a_streak"] = a_streak if (label != "base" and "A" in label) else 3
                    r["b_cap"] = b_cap if (label != "base" and "B" in label) else None
                    r["b_freeze"] = b_freeze if (label != "base" and "B" in label) else None
                all_rows.extend(rows)
                s = _t1_stats(rows)
                results[tier][label][sp] = s
                print(_t1_cell_line(label, sp, s))

    # --- pooled verdict + fragility (same rules as the P-6 study) ---
    print("\n" + "=" * 100)
    print("TIER-1 VERDICT: pooled-over-speed mean miss / Pk / realized correction "
          "per (tier x variant), delta vs the SAME tier's base")
    print("=" * 100)
    print(f"    {'tier':9s} {'variant':8s} {'mean':>6s} {'dMean':>7s} {'rel%':>6s}  "
          f"{'Pk@1':>5s} {'Pk@2':>5s}  {'realzd':>6s} {'latchR':>6s}")
    for tier in P6_TIER_ORDER:
        base_mean = float(np.mean([results[tier]["base"][sp]["mean"] for sp in T1_SPEEDS]))
        for label, _, _ in variants:
            mean = float(np.mean([results[tier][label][sp]["mean"] for sp in T1_SPEEDS]))
            pk1 = float(np.mean([results[tier][label][sp]["pk"][1.0] for sp in T1_SPEEDS]))
            pk2 = float(np.mean([results[tier][label][sp]["pk"][2.0] for sp in T1_SPEEDS]))
            rl = [results[tier][label][sp]["realized"] for sp in T1_SPEEDS]
            rl = [x for x in rl if x is not None]
            lr = [results[tier][label][sp]["latch_r"] for sp in T1_SPEEDS]
            lr = [x for x in lr if x is not None]
            rel = (base_mean - mean) / base_mean * 100.0 if base_mean > 1e-6 else 0.0
            print(f"    {tier:9s} {label:8s} {mean:6.2f} {mean - base_mean:+7.2f} "
                  f"{rel:+5.1f}%  {pk1*100:4.0f}% {pk2*100:4.0f}%  "
                  f"{(np.mean(rl) if rl else float('nan')):6.2f} "
                  f"{(np.mean(lr) if lr else float('nan')):6.2f}")
        print()
    print("  RULES: <~15% mean-miss delta = lab noise (trust ranking, not size); "
          "a lever that wins at BEST/EXPECTED\n  but hurts at WORST is FRAGILE "
          "(worse-than-ideal mandate). Winner still owes a Gazebo mc_batch A/B "
          "-- lab ranks, Gazebo decides.")
    print(f"\n[guidance_lab] Tier-1 study done in {time.perf_counter() - t0:.2f}s "
          f"({len(all_rows)} runs)")
    return all_rows, dict(a_streak=a_streak, b_cap=b_cap, b_freeze=b_freeze)


def write_tier1_csv(rows, path):
    fieldnames = [
        "tier1_variant", "tier", "a_streak", "b_cap", "b_freeze",
        "method", "path", "speed", "seed", "two_stage",
        "dash_speed", "handoff_range_cfg", "terminal_method",
        "miss_distance", "intercept_time", "control_effort",
        "detection_coverage", "terminal_coverage", "tag_lost_in_terminal",
        "latch_t", "latch_true_range", "zem_at_latch", "realized_correction",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k) for k in fieldnames})


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
    parser.add_argument("--adr0014", action="store_true",
                         help="run ONLY the ADR-0014 terminal-guidance lever study (yaw-rate "
                              "authority, split terminal-freeze + lambda_dot rate-cap, slower "
                              "terminal closing) instead of the main trade study, and write its "
                              "own CSV. See the ADR-0014 section of this file.")
    parser.add_argument("--adr0014-seeds", type=int, default=120,
                         help="seed count for the per-lever ranking sweeps (default 120)")
    parser.add_argument("--adr0014-combo-seeds", type=int, default=150,
                         help="seed count for the final best-combo/Pk-preview evaluation (default 150)")
    parser.add_argument("--adr0015", action="store_true",
                         help="run ONLY the ADR-0015 perception-constraint COST study (how much "
                              "each real-world cue/seeker data-constraint costs Pk; sensitivity "
                              "ranking + combined-realistic Pk-vs-R + PN-vs-PIP under realistic "
                              "velocity) instead of the main trade study, writing its own CSV. "
                              "See the ADR-0015 section of this file.")
    parser.add_argument("--adr0015-seeds", type=int, default=60,
                         help="seed count per cell for the ADR-0015 study (default 60, >=40 asked)")
    parser.add_argument("--p6-fusion", action="store_true",
                         help="run ONLY the P-6 mid-course-fusion + warm-handoff study "
                              "(ADR-0015 fusion decision; fusion off/on x warm off/on across "
                              "BEST/EXPECTED/WORST perception tiers) instead of the main trade "
                              "study, writing its own CSV. See the P-6 section of this file.")
    parser.add_argument("--p6-seeds", type=int, default=80,
                         help="seed count per cell for the P-6 fusion study (default 80, >=80 asked)")
    parser.add_argument("--jam-envelope", action="store_true",
                         help="run ONLY the jammer link-cutoff ENVELOPE study (ADR-0015 "
                              "'Handoff continuity': Pk vs cutoff range x coast-search x "
                              "fusion on a ~45 m runway, EXPECTED + WORST tiers, cliff-edge "
                              "and coast-margin extraction) instead of the main trade study, "
                              "writing its own CSV. See the JAM section of this file.")
    parser.add_argument("--jam-seeds", type=int, default=80,
                         help="seed count per cell for the jam-envelope study (default 80)")
    parser.add_argument("--tier1", action="store_true",
                         help="run ONLY the ADR-0023 Tier-1 guidance-reclaim study "
                              "(early-handoff x split-freeze x warm-terminal, 2^3 combos "
                              "x BEST/EXPECTED/WORST tiers x 6/8/10 m/s, pure_pn terminal) "
                              "instead of the main trade study, writing its own CSV. "
                              "See the Tier-1 section of this file.")
    parser.add_argument("--tier1-seeds", type=int, default=80,
                         help="seed count per cell for the Tier-1 main table (default 80)")
    parser.add_argument("--tier1-presweep-seeds", type=int, default=80,
                         help="seed count per cell for the Tier-1 lever-parameter "
                              "pre-sweep (default 80)")
    args = parser.parse_args()

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    csv_path = os.path.join(LOGS_DIR, f"guidance_lab_{timestamp}.csv")
    plot_prefix = os.path.join(LOGS_DIR, f"guidance_lab_{timestamp}")

    if args.tier1:
        print(f"\n[guidance_lab] ADR-0023 Tier-1 guidance-reclaim study "
              f"(seeds={args.tier1_seeds}/cell, presweep={args.tier1_presweep_seeds})")
        t1_csv_path = os.path.join(LOGS_DIR, f"guidance_lab_tier1_{timestamp}.csv")
        all_rows, picks = run_tier1_study(
            n_seeds=args.tier1_seeds, presweep_seeds=args.tier1_presweep_seeds
        )
        write_tier1_csv(all_rows, t1_csv_path)
        print(f"\n[guidance_lab] Tier-1 CSV written: {t1_csv_path} ({len(all_rows)} rows)")
        print(f"[guidance_lab] Tier-1 lever picks: {picks}")
        print(
            "  REMINDER: kinematic surrogate -- this RANKS the three Tier-1 levers; "
            "the winner(s) still owe a paired Gazebo mc_batch A/B (--early-handoff / "
            "--split-freeze / --warm-handoff on m4_intercept.py) before anything is a "
            "result. The lab CANNOT model the ADR-0009 whipsaw (no commanded-direction"
            " -> achieved-velocity collapse coupling), so split-freeze risk is "
            "under-priced here by construction -- Gazebo decides."
        )
        return 0

    if args.jam_envelope:
        print(f"\n[guidance_lab] jammer link-cutoff envelope study "
              f"(seeds={args.jam_seeds}/cell)")
        jam_csv_path = os.path.join(LOGS_DIR, f"guidance_lab_jamenv_{timestamp}.csv")
        all_rows = run_jam_envelope(n_seeds=args.jam_seeds)
        write_jam_csv(all_rows, jam_csv_path)
        print(f"\n[guidance_lab] jam-envelope CSV written: {jam_csv_path} "
              f"({len(all_rows)} rows)")
        print(
            "  REMINDER: kinematic surrogate -- this RANKS and SIZES the link-cutoff "
            "envelope (relative cliff position, coast tolerance, fusion interaction); "
            "the lab under-prices terminal perception (ADR-0015 caveat) and the "
            "crossing target is constant-velocity (dead-reckon best case -- see the "
            "s_weave probe). The cliff numbers gate a Gazebo --coast-search A/B; "
            "Gazebo decides."
        )
        return 0

    if args.p6_fusion:
        print(f"\n[guidance_lab] P-6 mid-course-fusion + warm-handoff study "
              f"(seeds={args.p6_seeds}/cell)")
        p6_csv_path = os.path.join(LOGS_DIR, f"guidance_lab_p6fusion_{timestamp}.csv")
        all_rows = run_p6_study(n_seeds=args.p6_seeds)
        write_csv(all_rows, p6_csv_path)
        print(f"\n[guidance_lab] P-6 CSV written: {p6_csv_path} ({len(all_rows)} rows)")
        print(
            "  REMINDER: kinematic surrogate -- the P-6 ranking is a HYPOTHESIS gating "
            "the Gazebo A/B of --fuse-midcourse/--warm-handoff; a lab win < ~15% is noise, "
            "and the lab UNDER-prices terminal perception (ADR-0015 caveat), so trust the "
            "RELATIVE ranking across tiers, not the absolute Pk."
        )
        return 0

    if args.adr0015:
        print(f"\n[guidance_lab] ADR-0015 perception-constraint cost study "
              f"(seeds={args.adr0015_seeds}/cell)")
        adr15_csv_path = os.path.join(LOGS_DIR, f"guidance_lab_adr0015_{timestamp}.csv")
        all_rows = run_adr0015_study(n_seeds=args.adr0015_seeds)
        write_csv(all_rows, adr15_csv_path)
        print(f"\n[guidance_lab] ADR-0015 CSV written: {adr15_csv_path} ({len(all_rows)} rows)")
        print(
            "  REMINDER: kinematic surrogate -- these perception-cost numbers RANK which "
            "data-constraints the hardware/perception design must get right; the winning "
            "config still has to re-earn its Pk in a Gazebo/mc_batch run before it is a result."
        )
        return 0

    if args.adr0014:
        print(
            "\n[guidance_lab] ADR-0014 terminal-guidance lever study "
            f"(seeds={args.adr0014_seeds}, combo_seeds={args.adr0014_combo_seeds})"
        )
        adr14_csv_path = os.path.join(LOGS_DIR, f"guidance_lab_adr0014_{timestamp}.csv")
        all_rows, best_combo_overrides, best_yaw_rate = run_adr14_study(
            n_seeds=args.adr0014_seeds, combo_n_seeds=args.adr0014_combo_seeds
        )
        write_csv(all_rows, adr14_csv_path)
        print(f"\n[guidance_lab] ADR-0014 CSV written: {adr14_csv_path} ({len(all_rows)} rows)")
        print(f"[guidance_lab] ADR-0014 recommended combo overrides: {best_combo_overrides}")
        print(f"[guidance_lab] ADR-0014 recommended yaw_rate_max_deg_s: {best_yaw_rate}")
        print(
            "  REMINDER: this is a kinematic surrogate -- these lever rankings are a HYPOTHESIS "
            "gating which levers get an actual Gazebo A/B, not a Gazebo result themselves."
        )
        return 0

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
