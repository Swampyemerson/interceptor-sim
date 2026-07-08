#!/usr/bin/env python3
"""Cartesian constant-velocity Extended Kalman Filter (EKF) target tracker --
the Option A estimator from docs/ekf_design_brief.md (post-M5 queue item 3,
ADR-0033). SELF-CONTAINED and OFFLINE: this module boots no sim, reads no
gt_*, and is a DROP-IN for the pair of scalar alpha-beta filters
(lambda_filter + range_filter) that scripts/m4_intercept.py's pro-nav loop
consumes.

WHAT THIS IS (one paragraph). We currently estimate the LOS azimuth lambda and
the range R with two independent fixed-gain alpha-beta (g-h) filters
(AlphaBetaFilter in m4_intercept.py). An alpha-beta filter is exactly the
STEADY-STATE Kalman filter for a constant-velocity target at a FIXED sample
rate and FIXED measurement noise, with the covariance thrown away and the gain
frozen (brief section 1.3). A real Kalman filter keeps the covariance P and
recomputes the gain K = P- H^T (H P- H^T + R)^-1 every step, so it stays
correct when (1) the sample interval varies -- the bimodal ~14 Hz-burst +
multi-second-gap cadence that made Kalata's closed-form adaptive gain go
degenerate and lose every Gazebo flight (ADR-0013), and (2) the measurement
noise varies -- our camera range noise is range-dependent (sigma ~ frac*R).
Those are precisely the two assumptions the frozen alpha-beta gain cannot
honor. Because the camera measures POLAR (bearing, range) quantities of a
CARTESIAN target, the measurement map h(x) is nonlinear -> EKF (linearize h
via its Jacobian each step; brief section 1.4).

STATE PARAMETERIZATION (honest deviation from the brief, made to satisfy the
drop-in / "no loop change beyond construction" constraint). The brief's
Option A is the ABSOLUTE target Cartesian state [n, e, vn, ve] with own NED
position folded into h(x). That formulation needs own-state at every camera
update. m4_intercept.py's guidance loop hands the tracker ONLY the scalar
measurements (lambda_meas, range_m) through AlphaBetaFilter.correct(meas, t)
-- it does NOT pass own position through that call, and the task mandates the
loop stay byte-identical except for the construction branch. So this EKF
tracks the RELATIVE Cartesian state

    x = [dn, de, dvn, dve]   (target - own, in PX4 local NED; N, E, then rates)

which is a pure function of the same (lambda, R) the two alpha-beta channels
already see -- no own-state needed in the update:

    h(x) = [ lambda, R ] = [ atan2(de, dn), sqrt(dn^2 + de^2) ]

This is still a Cartesian constant-velocity EKF (Option A's geometry and its
covariance-aware derived LOS rate); the one cost is that OWN-vehicle
maneuvering, which the absolute formulation would subtract deterministically,
here enters as process noise Q instead. That is a documented, defensible
trade, and Q is sized (section "Process noise") to cover it. The pro-nav
read-outs are DERIVED, covariance-aware, from the relative Cartesian state --
exactly the geometric forms FusedTrack.state() already uses
(m4_intercept.py:876-895):

    lambda_hat     = atan2(de, dn)
    lambda_dot_hat = (dn*dve - de*dvn) / R^2      # d/dt atan2(de, dn)
    r_hat          = sqrt(dn^2 + de^2)
    rdot_hat       = (dn*dvn + de*dve) / R

A Cartesian velocity state gives a CLEANER lambda_dot than differentiating a
filtered angle, because the target-velocity state absorbs measurement noise
before it reaches the rate (brief section 2.1) -- and pro-nav consumes the
rate (a_cmd = N*Vc*lambda_dot).

DROP-IN CONTRACT. EKFTracker exposes two thin channel VIEWS,
`.lambda_filter` and `.range_filter`, each mirroring the AlphaBetaFilter
public surface the guidance loop touches: `.predict(dt)`, `.correct(meas, t)`,
`.x_hat`, `.xdot_hat`, `.initialized`, `._last_correction_t` (read + write --
the warm-handoff seeding at m4_intercept.py:1995-2017 writes these). Both
views are backed by ONE shared EKF:
  * lambda_filter.predict(dt) advances the EKF once; range_filter.predict(dt)
    is a no-op (the loop always calls both back-to-back with the same dt, so
    the state is propagated exactly once per tick).
  * lambda_filter.correct(lambda, t) BUFFERS the bearing; range_filter.
    correct(R, t) FLUSHES the joint 2-D camera update (init on the first, a
    sequential-measurement EKF update after). The loop always corrects the
    pair together on a fresh detection (m4_intercept.py:1591-1592), so the
    camera update is always the full [lambda, R].
The alpha-beta-specific knobs (--kalata gains, --split-freeze rate_cap) do NOT
apply to the EKF and are ignored under --tracker ekf; the EKF's own Q is the
principled replacement for the Kalata dt-adaptation (brief section 2.4).

MEASUREMENT NOISE R (where the sensor physics live, brief section 2.3):
  * Camera bearing sigma: sub-degree for the AprilTag in-sim (1-2 deg for the
    real ML seeker). CAM_BEARING_SIGMA_DEG.
  * Camera range sigma is RANGE-DEPENDENT: sigma_R = frac * R_hat, rebuilt
    every update (AprilTag ~5-8% in-sim; FUSE_CAM_RANGE_FRAC=0.10 conservative
    match). A KF does this natively; a fixed-gain filter structurally cannot.
  * Cue channel (correct_cue, interface-complete for the gated fusion arm):
    sigma_R(R) = a + c*R^2 with the ADR-0017-corrected constants
    (a, c) = (0.0, 4.45e-05), RSS'd with the datum-bias budget -- because the
    cue error is BIAS-dominated (a per-run constant GPS/clock offset), which a
    zero-mean KF cannot model, so we inflate R to swallow the bias budget
    (brief section 2.3, the crude-but-honest treatment; the bias-augmented
    Option D is a future arm).

INNOVATION (chi-square) GATING (brief section 2.6). With S in hand, gating is
free: reject a camera/cue measurement whose NIS = y^T S^-1 y exceeds a
chi-square threshold. Targets the Markov dropout-burst re-acquire glitches and
any real-seeker fat-tail outlier. HONESTY (pre-registered): ADR-0013 found
range-outlier gating negative in-sim ("nothing to catch in Gaussian noise" --
the AprilTag is clean), so in the current sim gating is LIKELY INERT (a null);
it earns its keep only under the real seeker's fatter tails or the cue's
re-acquire glitches. Default threshold is loose enough to be inert on clean
Gaussian data.

WARM-START COVARIANCE SEEDING (brief section 2.5, the cleanest EKF win). At
the one-way cue handoff, m4_intercept.py's --warm-handoff seeds the terminal
filters' STATE from the fused/dash track but cannot seed CONFIDENCE (alpha-beta
has no covariance). This EKF seeds x_hat AND P: the fused velocity came from an
emitted cue velocity with a KNOWN sigma_v ~ 0.5 m/s, so the seeded velocity
covariance is SMALL and the terminal does not spend its first corrections
re-earning confidence it was handed. seed_from_polar() does this; the view
setters used by the existing warm-handoff code route through it.

>>> NEXT STEP IS GATED (do NOT run here). The paired-seed n>=8 Gazebo A/B
    pre-registered in docs/ekf_design_brief.md section 4 -- --tracker
    {alphabeta, ekf} on matched seeds, expecting a NULL on end-to-end miss
    (the miss is KINEMATIC, r^2(ZEM,miss)=0.990, ADR-0023) and a possible
    real SIGNAL on track RMSE / LOS-rate error -- is the next task. It runs
    ONE sim at a time at idle machine load (CLAUDE.md batch hygiene;
    mc_batch self-kill hazard). This module is OFFLINE-built and
    unit-tested only; it must not perturb the frozen M5 default path
    (--tracker defaults to alphabeta; the alphabeta construction in
    m4_intercept.py is byte-identical to HEAD). <<<
"""

import math
from typing import Optional

import numpy as np

# --- Measurement-noise model (brief section 2.3). Camera is the clean angle
# authority; range is range-dependent. Kept as module constants so an A/B arm
# or a real-seeker profile can override them explicitly. ---
CAM_BEARING_SIGMA_DEG = 0.5      # AprilTag sub-degree in-sim (1-2 deg real ML seeker)
CAM_RANGE_FRAC = 0.10            # sigma_R = frac * R_hat; AprilTag ~5-8%, 0.10 conservative
CAM_RANGE_SIGMA_FLOOR_M = 0.05   # keep R positive-definite as R -> 0

# Cue channel (correct_cue): ADR-0017-corrected sigma_R(R) = a + c*R^2, RSS'd
# with the datum-bias budget (the cue error is BIAS-dominated; inflate R to
# swallow it -- brief section 2.3 crude-but-honest treatment).
CUE_RANGE_SIGMA_A = 0.0          # a (m), ADR-0017 corrected
CUE_RANGE_SIGMA_C = 4.45e-05     # c (m / m^2), ADR-0017 corrected (was 0.008, ~180x too steep)
CUE_DATUM_BUDGET_M = 0.5         # shared-RTK datum/clock offset budget (ADR-0015 row 1)
CUE_VEL_SIGMA_M_S = 0.5          # emitted-cue velocity sigma (ADR-0015 #1 lever)
# Fusion capstone design D2 (ADR-0041): the caller converts the cue's world
# position into a RELATIVE position by subtracting OWN EKF2 position
# (m4_intercept.py's state.pos_n/pos_e) -- itself an unmeasured error source,
# never bench-validated against gt in this design pass ("or measure EKF2-vs-
# gt for one flight first -- ten minutes -- and size it properly" was the
# design doc's other option; this fixed term is the cheaper one taken now).
CUE_OWN_POS_INSURANCE_M = 0.15   # 1-sigma, RSS'd into the cue position R

# --- Handoff boundary (design D3, ADR-0041): the one-way latch re-inflation
# floor and the adaptive gate-recovery floors. Both sized relative to the
# council's own WORST-tier finding (F1: an untuned worst-credible 2.5 m
# datum bias made a sensibly-tuned gate reject 81-100% of VALID post-latch
# camera corrections) -- generously above that 2.5 m so a first post-latch
# camera correction is never chi-square-rejected purely from carried-over
# cue-position confidence, even at the worst case the council measured. ---
LATCH_POS_SIGMA_FLOOR_M = 3.0            # D3.1: unconditional latch re-inflation, position block only
GATE_RECOVERY_STREAK_N = 3               # D3.2: consecutive post-latch camera rejections that trigger recovery
GATE_RECOVERY_POS_SIGMA_FLOOR_M = 3.0    # D3.2 position-block floor (same scale as the latch floor)
GATE_RECOVERY_VEL_SIGMA_FLOOR_M_S = 8.0  # D3.2 velocity-block floor (looser than warm, tighter than cold-init's 12.0 -- still worth keeping some carried state, just not confidently)

# --- Process noise Q (brief section 2.4). Continuous white-noise-acceleration
# (CWNA) per-axis block  Q_axis = q * [[T^3/3, T^2/2],[T^2/2, T]], q = PSD of
# the acceleration noise (m^2/s^3). Sized from the worst credible maneuver in
# the M5 path suite AND (in this RELATIVE formulation) the own-vehicle dash
# accel that the absolute formulation would have subtracted: weave lateral
# WEAVE_LAT_SPEED=3.0 m/s over period tau gives a_max ~ (2*pi/tau)*v_lat; the
# dash agility bundle allows ~12-20 m/s^2. We take sigma_a ~ 8 m/s^2 as a
# worst-credible combined accel and q ~ sigma_a^2 (correlation time ~1 s), a
# single documented default that the NIS/NEES consistency harness
# (tests/test_ekf_tracker.py) then VALIDATES rather than eyeballs. Override via
# EKFTracker(q_accel_psd=...). ---
Q_ACCEL_PSD_DEFAULT = 64.0       # (m^2/s^3) ~ (8 m/s^2)^2; see derivation above

# --- Initialization (brief section 2.5). Cold init: position from the first
# camera (lambda, R); velocity UNKNOWN -> large variance (~ full target speed
# band). ---
INIT_POS_SIGMA_M = 1.0           # first-fix relative-position sigma
INIT_VEL_SIGMA_M_S = 12.0        # target speed band (brief: ~12 m/s), velocity unknown at cold init

# Warm-start (seed_from_polar): the handed velocity came from an emitted cue
# velocity (sigma_v ~ 0.5) -> SMALL velocity covariance so the terminal does
# not re-earn confidence.
WARM_POS_SIGMA_M = 0.5
WARM_VEL_SIGMA_M_S = 0.7         # ~ emitted-cue sigma_v with a little slack

# --- Innovation gating (brief section 2.6). chi-square 0.99 upper tail.
# dof=2 for the joint camera update. Loose enough to be INERT on clean
# Gaussian data (the pre-registered null); catches gross re-acquire glitches /
# real-seeker fat tails. Default ON but effectively a no-op in the clean sim. ---
GATE_CHI2_DOF2_P99 = 9.210       # chi2.ppf(0.99, 2)
GATE_CHI2_DOF1_P99 = 6.635       # chi2.ppf(0.99, 1) (single-channel cue update)


def wrap_pi(angle: float) -> float:
    """Wrap an angle (radians) into [-pi, pi]. Same convention as
    m4_intercept.wrap_pi -- used on the bearing innovation."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# =====================================================================
# Self-contained chi-square / normal quantiles (no scipy in this venv).
# Used by the tests' NIS/NEES consistency bands and by gating thresholds.
# =====================================================================
def norm_ppf(p: float) -> float:
    """Inverse standard-normal CDF (Acklam's rational approximation).
    Absolute error < 1.15e-9 -- ample for consistency bands."""
    if p <= 0.0:
        return -math.inf
    if p >= 1.0:
        return math.inf
    a = [-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
         1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00]
    b = [-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
         6.680131188771972e+01, -1.328068155288572e+01]
    c = [-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
         -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00]
    d = [7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
         3.754408661907416e+00]
    plow = 0.02425
    phigh = 1.0 - plow
    if p < plow:
        q = math.sqrt(-2.0 * math.log(p))
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    if p > phigh:
        q = math.sqrt(-2.0 * math.log(1.0 - p))
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
                ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    q = p - 0.5
    r = q * q
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / \
           (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0)


def chi2_ppf(p: float, k: float) -> float:
    """Chi-square quantile via the Wilson-Hilferty (cube-root normal)
    approximation: excellent for the moderate/large dof (2K, 4K) of the
    time-averaged NIS/NEES consistency bands."""
    z = norm_ppf(p)
    t = 1.0 - 2.0 / (9.0 * k) + z * math.sqrt(2.0 / (9.0 * k))
    return k * t * t * t


class _ChannelView:
    """Thin adapter presenting the AlphaBetaFilter surface the guidance loop
    touches, backed by the shared EKFTracker. `kind` is 'lambda' or 'range'.
    All state lives in the parent EKF; this object is a typed accessor so the
    loop's `lambda_filter.x_hat` / `range_filter.correct(...)` calls work
    unchanged."""

    def __init__(self, ekf: "EKFTracker", kind: str):
        self._ekf = ekf
        self._kind = kind

    # --- read-outs (None when uninitialized, exactly like AlphaBetaFilter) ---
    @property
    def initialized(self) -> bool:
        return self._ekf.initialized

    @property
    def x_hat(self):
        if not self._ekf.initialized:
            return None
        return self._ekf.lambda_hat if self._kind == "lambda" else self._ekf.r_hat

    @x_hat.setter
    def x_hat(self, value):
        # Warm-handoff seeding (m4_intercept.py:1995-2017 writes .x_hat).
        self._ekf._seed_scalar(self._kind, "pos", value)

    @property
    def xdot_hat(self):
        if not self._ekf.initialized:
            return 0.0
        return self._ekf.lambda_dot_hat if self._kind == "lambda" else self._ekf.rdot_hat

    @xdot_hat.setter
    def xdot_hat(self, value):
        self._ekf._seed_scalar(self._kind, "rate", value)

    @property
    def _last_correction_t(self):
        return self._ekf._last_correction_t

    @_last_correction_t.setter
    def _last_correction_t(self, value):
        self._ekf._last_correction_t = value

    # --- steps ---
    def predict(self, dt: float) -> None:
        # Only the lambda view advances the shared EKF; the range view is a
        # no-op (the loop calls both per tick with the same dt).
        if self._kind == "lambda":
            self._ekf.predict(dt)

    def correct(self, meas: float, t: float, gain_scale: float = 1.0) -> None:
        # gain_scale is accepted for signature-compatibility with
        # AlphaBetaFilter.correct (FusedTrack passes it); the EKF weights by
        # its own live covariance, so it is ignored here.
        if self._kind == "lambda":
            self._ekf._buffer_bearing(meas, t)
        else:
            self._ekf._camera_update(meas, t)


class EKFTracker:
    """Cartesian constant-velocity EKF on the RELATIVE target state
    x = [dn, de, dvn, dve] (target - own, PX4 local NED). Drop-in for the
    (lambda_filter, range_filter) alpha-beta pair via `.lambda_filter` /
    `.range_filter` views. See the module docstring for the full rationale."""

    def __init__(self, q_accel_psd: float = Q_ACCEL_PSD_DEFAULT,
                 cam_bearing_sigma_deg: float = CAM_BEARING_SIGMA_DEG,
                 cam_range_frac: float = CAM_RANGE_FRAC,
                 gating: bool = True):
        self.q = float(q_accel_psd)
        self.cam_bearing_sigma = math.radians(cam_bearing_sigma_deg)
        self.cam_range_frac = cam_range_frac
        self.gating = gating

        self.x: Optional[np.ndarray] = None   # [dn, de, dvn, dve]
        self.P: Optional[np.ndarray] = None   # 4x4 covariance
        self._last_correction_t: Optional[float] = None
        self._pending_bearing = None          # (t, lambda) buffered by the lambda view
        self.n_corrections = 0

        # Diagnostics (scoring/consistency only -- never feeds a command).
        self.last_nis: Optional[float] = None
        self.last_S = None
        self.last_innovation: float = 0.0
        self.n_gated = 0
        # Camera-channel-only freshness (design D2 mid-course aim gate): set
        # by _camera_update on every attempted (not just accepted) camera
        # correction, distinct from _last_correction_t, which correct_cue
        # also advances -- a caller wanting "did the CAMERA (not the cue)
        # correct recently" needs this, mirroring FusedTrack.last_camera_t /
        # camera_fresh() (m4_intercept.py:848,877).
        self.last_camera_t: Optional[float] = None

        # Handoff boundary (design D3, ADR-0041). _cue_called is TRUE the
        # instant correct_cue is first invoked PRE-latch (belt: latch() reads
        # it to decide whether P needs re-inflation) -- it is a "was the cue
        # ever used" flag, not a per-call counter. cue_updates_post_handoff
        # is the reverse: a call COUNT of illegal post-latch attempts, which
        # the m4_intercept.py wiring proves stays 0 (the cue_reader=None
        # belt already makes such a call structurally unreachable in normal
        # flight; this counter is the suspenders for a synthetic/injected
        # leak test).
        self._latched = False
        self._cue_called = False
        self.cue_updates_post_handoff = 0
        self.camera_gated_post_handoff = 0
        self._post_latch_gate_streak = 0
        self.p_diag_at_latch: Optional[tuple] = None

        self.lambda_filter = _ChannelView(self, "lambda")
        self.range_filter = _ChannelView(self, "range")

    # ---------------------------------------------------------------
    # Derived, covariance-aware read-outs (the geometric forms
    # FusedTrack.state() uses, m4_intercept.py:876-895).
    # ---------------------------------------------------------------
    @property
    def initialized(self) -> bool:
        return self.x is not None

    @property
    def _R2(self) -> float:
        dn, de = self.x[0], self.x[1]
        return max(dn * dn + de * de, 1e-9)

    @property
    def r_hat(self) -> float:
        return math.sqrt(self._R2)

    @property
    def lambda_hat(self) -> float:
        return math.atan2(self.x[1], self.x[0])

    @property
    def lambda_dot_hat(self) -> float:
        dn, de, dvn, dve = self.x
        return (dn * dve - de * dvn) / self._R2

    @property
    def rdot_hat(self) -> float:
        dn, de, dvn, dve = self.x
        return (dn * dvn + de * dve) / self.r_hat

    @property
    def pos_hat(self):
        """Relative target position (dn, de) in NED, or None. (Absolute
        target position needs own NED, which this relative EKF does not
        carry; the pro-nav loop consumes the polar read-outs, not pos_hat.)"""
        if self.x is None:
            return None
        return (float(self.x[0]), float(self.x[1]))

    @property
    def vel_hat(self):
        """Relative target velocity (dvn, dve) in NED, or None."""
        if self.x is None:
            return None
        return (float(self.x[2]), float(self.x[3]))

    # ---------------------------------------------------------------
    # Predict (brief section 1.2). x- = F x ; P- = F P F^T + Q.
    # ---------------------------------------------------------------
    def predict(self, dt: float) -> None:
        if self.x is None or dt <= 0.0:
            return
        F = np.array([[1.0, 0.0, dt, 0.0],
                      [0.0, 1.0, 0.0, dt],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)

    def _Q(self, dt: float) -> np.ndarray:
        """CWNA per-axis process-noise block (brief section 2.4)."""
        q = self.q
        t2 = dt * dt
        t3 = t2 * dt
        a = q * t3 / 3.0
        b = q * t2 / 2.0
        c = q * dt
        return np.array([[a, 0.0, b, 0.0],
                         [0.0, a, 0.0, b],
                         [b, 0.0, c, 0.0],
                         [0.0, b, 0.0, c]])

    # ---------------------------------------------------------------
    # Camera update: buffered bearing (lambda view) + range (range view)
    # -> one joint 2-D EKF update z = [lambda, R]. Sequential-measurement
    # form (diagonal R) is identical to the batch update.
    # ---------------------------------------------------------------
    def _buffer_bearing(self, lambda_meas: float, t: float) -> None:
        self._pending_bearing = (t, lambda_meas)

    def _camera_update(self, range_m: float, t: float) -> None:
        # Recover the bearing buffered by the lambda view this same tick.
        lam = None
        if self._pending_bearing is not None and abs(self._pending_bearing[0] - t) < 1e-9:
            lam = self._pending_bearing[1]
        self._pending_bearing = None
        # design D2 mid-course aim gate: mark "a camera update was attempted
        # this tick" unconditionally (matches FusedTrack.camera_fresh()'s
        # ungated semantics -- FusedTrack has no gate to consider at all, so
        # this is the closest analogous meaning for the EKF: "fresh" =
        # processed, not "fresh" = accepted).
        self.last_camera_t = t

        if self.x is None:
            # Cold init (brief section 2.5): position from the first
            # (lambda, R); velocity unknown -> large variance.
            if lam is None:
                return  # need both to place the relative position
            dn = range_m * math.cos(lam)
            de = range_m * math.sin(lam)
            self.x = np.array([dn, de, 0.0, 0.0])
            self.P = np.diag([INIT_POS_SIGMA_M ** 2, INIT_POS_SIGMA_M ** 2,
                              INIT_VEL_SIGMA_M_S ** 2, INIT_VEL_SIGMA_M_S ** 2])
            self._last_correction_t = t
            return

        if lam is None:
            # Range-only update (rare: lambda view not called this tick).
            self._scalar_range_update(range_m, t)
            self._last_correction_t = t
            return

        # Joint [lambda, R] EKF update.
        dn, de = self.x[0], self.x[1]
        R2 = max(dn * dn + de * de, 1e-9)
        R = math.sqrt(R2)
        h = np.array([math.atan2(de, dn), R])
        H = np.array([[-de / R2, dn / R2, 0.0, 0.0],
                      [dn / R, de / R, 0.0, 0.0]])
        sigma_R = max(CAM_RANGE_SIGMA_FLOOR_M, self.cam_range_frac * R)
        Rmat = np.diag([self.cam_bearing_sigma ** 2, sigma_R ** 2])
        z = np.array([lam, range_m])
        y = z - h
        y[0] = wrap_pi(y[0])       # angular innovation
        self._apply_update(H, Rmat, y, t, dof=2)

    def _scalar_range_update(self, range_m: float, t: float) -> None:
        dn, de = self.x[0], self.x[1]
        R = math.sqrt(max(dn * dn + de * de, 1e-9))
        H = np.array([[dn / R, de / R, 0.0, 0.0]])
        sigma_R = max(CAM_RANGE_SIGMA_FLOOR_M, self.cam_range_frac * R)
        Rmat = np.array([[sigma_R ** 2]])
        y = np.array([range_m - R])
        self._apply_update(H, Rmat, y, t, dof=1)

    # ---------------------------------------------------------------
    # Cue update (interface-complete for the gated fusion arm, brief
    # section 3.3; POLAR-SPLIT per design D2 / ADR-0041 F2). Takes the cue
    # as a RELATIVE position (caller subtracts own NED, mirroring
    # m4_intercept's existing cue path) plus an optional RELATIVE velocity.
    # PRE-latch only -- see latch() below; a post-latch call is structurally
    # illegal and raises.
    # ---------------------------------------------------------------
    def correct_cue(self, rel_pos_ned, rel_vel_ned, t: float) -> None:
        if self._latched:
            # Belt-and-suspenders (design D3): m4_intercept.py nulls
            # cue_reader at the exact HANDOFF instant this latch() call is
            # made, so this call site is structurally unreachable in normal
            # flight (the belt). This raise + counter is the suspenders --
            # tests/test_ekf_tracker.py and the injected-leak honesty-static
            # variant both exercise it directly.
            self.cue_updates_post_handoff += 1
            raise RuntimeError(
                "EKFTracker.correct_cue() called after latch() -- the cue "
                "channel is structurally closed post-handoff (design D3, "
                "ADR-0041); m4_intercept.py must never reach this call site "
                "once cue_reader is nulled."
            )
        self._cue_called = True

        if self.x is None:
            # Cold init straight from the cue (interface-complete for a
            # cue-only cold start): no lambda_hat exists yet to polar-split
            # against. In the real m4_intercept.py wiring this branch is
            # never reached -- correct_cue is only called AFTER the EKF is
            # camera-initialized (mirrors FusedTrack.update_cue's own
            # initialized-gate: "a biased cue must never define the
            # CAMERA-anchored track's initial state").
            dn, de = float(rel_pos_ned[0]), float(rel_pos_ned[1])
            vn = 0.0 if rel_vel_ned is None else float(rel_vel_ned[0])
            ve = 0.0 if rel_vel_ned is None else float(rel_vel_ned[1])
            self.x = np.array([dn, de, vn, ve])
            pv = INIT_VEL_SIGMA_M_S if rel_vel_ned is None else CUE_VEL_SIGMA_M_S
            self.P = np.diag([INIT_POS_SIGMA_M ** 2, INIT_POS_SIGMA_M ** 2, pv ** 2, pv ** 2])
            self._last_correction_t = t
            return

        # --- Polar split (F2 fix, design D2): decompose the position
        # innovation into along-LOS (range) and cross-LOS components using
        # the CURRENT lambda_hat, then feed ONLY the along-LOS scalar into
        # the filter -- the cross-LOS component is DROPPED OUTRIGHT (not
        # merely inflated). Chosen over a large-but-finite R inflation
        # because (a) it exactly matches FusedTrack's stated discipline
        # ("the cue NEVER touches the angle") with no residual leak at any
        # finite inflation factor, and (b) a pure cross-LOS cue offset is
        # then EXACTLY orthogonal to the measurement direction u (u . offset
        # = 0 when offset = k*p, since u perp p by construction) -- so the
        # innovation itself is zero rather than merely down-weighted:
        # provably inert on lambda_hat, not just "small" (see
        # test_cue_polar_split_cross_range_never_touches_bearing). Reuses
        # the existing dof=1 gate threshold (GATE_CHI2_DOF1_P99), the same
        # one _scalar_range_update already uses for a range-only camera
        # correction.
        R_hat = self.r_hat
        lam = self.lambda_hat
        u = (math.cos(lam), math.sin(lam))     # along-LOS unit vector
        y_pos_n = rel_pos_ned[0] - self.x[0]
        y_pos_e = rel_pos_ned[1] - self.x[1]
        y_along = y_pos_n * u[0] + y_pos_e * u[1]

        sigma_stat = CUE_RANGE_SIGMA_A + CUE_RANGE_SIGMA_C * R_hat * R_hat
        # +CUE_OWN_POS_INSURANCE_M (design D2): the world->NED->relative
        # conversion at the call site subtracts OWN EKF2 position, which is
        # itself imperfect and unmeasured here -- RSS in a fixed budget so
        # the cue R does not understate the true position uncertainty.
        sigma_pos = math.sqrt(
            sigma_stat * sigma_stat + CUE_DATUM_BUDGET_M ** 2 + CUE_OWN_POS_INSURANCE_M ** 2
        )
        H_along = np.array([[u[0], u[1], 0.0, 0.0]])
        Rmat_along = np.array([[sigma_pos ** 2]])
        self._apply_update(H_along, Rmat_along, np.array([y_along]), t, dof=1)

        # Velocity (design D2: "the cue contributes range/velocity only" --
        # the angle restriction is POSITION-specific; the cue's emitted
        # velocity is a trusted lever (ADR-0015 #1) and updates dvn/dve
        # directly, a separate sequential-measurement update from the
        # along-LOS position update above -- same pattern the joint camera
        # update already documents as "identical to the batch update").
        if rel_vel_ned is not None:
            H_vel = np.array([[0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]])
            Rmat_vel = np.diag([CUE_VEL_SIGMA_M_S ** 2, CUE_VEL_SIGMA_M_S ** 2])
            y_vel = np.array([rel_vel_ned[0] - self.x[2], rel_vel_ned[1] - self.x[3]])
            self._apply_update(H_vel, Rmat_vel, y_vel, t, dof=2)

    # ---------------------------------------------------------------
    # Shared update core: gain, gate, Joseph-form covariance, diagnostics.
    # ---------------------------------------------------------------
    def _apply_update(self, H, Rmat, y, t, dof) -> None:
        S = H @ self.P @ H.T + Rmat
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        nis = float(y @ Sinv @ y)
        self.last_nis = nis
        self.last_S = S
        self.last_innovation = float(y[0])

        # Innovation gate (brief section 2.6): reject gross outliers. Inert on
        # clean Gaussian data (the pre-registered null).
        if self.gating:
            thresh = GATE_CHI2_DOF2_P99 if dof >= 2 else GATE_CHI2_DOF1_P99
            if dof == 4:
                thresh = 13.277  # chi2.ppf(0.99, 4)
            if nis > thresh:
                self.n_gated += 1
                if self._latched:
                    # F1 escape hatch (design D3.2, ADR-0041): post-latch,
                    # this can only be a camera-channel rejection --
                    # correct_cue raises before ever reaching _apply_update
                    # once latched, so every post-latch caller here is the
                    # bearing/range channel. Track CONSECUTIVE rejections;
                    # N=3 running is the signature of a bias-locked filter
                    # (an unbiased camera measurement should not keep
                    # failing the gate on clean Gaussian data -- the
                    # pre-registered gating null) and triggers recovery.
                    self.camera_gated_post_handoff += 1
                    self._post_latch_gate_streak += 1
                    if self._post_latch_gate_streak >= GATE_RECOVERY_STREAK_N:
                        self._gate_recovery_reinflate()
                        self._post_latch_gate_streak = 0
                return
        if self._latched:
            # An ACCEPTED post-latch camera correction breaks any streak of
            # rejections -- the recovery trigger is CONSECUTIVE, not total.
            self._post_latch_gate_streak = 0

        K = self.P @ H.T @ Sinv
        self.x = self.x + K @ y
        # Joseph form -- keeps P symmetric positive-definite through the
        # long-gap / high-gain regime (brief Q6 numerical-robustness note).
        I = np.eye(4)
        A = I - K @ H
        self.P = A @ self.P @ A.T + K @ Rmat @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self._last_correction_t = t
        self.n_corrections += 1

    def _gate_recovery_reinflate(self) -> None:
        """F1 fix (design D3.2, ADR-0041): N consecutive post-latch camera
        rejections is the signature of a bias-locked filter (a correctly-
        modeled, unbiased camera measurement should not fail the gate N
        times running on clean Gaussian data -- the pre-registered gating
        null from brief section 2.6). Widen P's position AND velocity
        blocks (track-loss-recovery pattern) so the next camera correction
        is accepted and can pull the state back toward truth, rather than
        the filter's own tight-but-wrong P shielding the error forever.

        M1 (ADR-0041 adversarial review): floor via max(), never shrink.
        Assignment could DECREASE a diagonal entry that already grew past
        the floor (e.g. a long dropout under Q=64), and shrinking one
        diagonal entry while leaving cross terms (P[0,2], P[1,3], ...)
        untouched can leave P non-PSD. Increasing a PSD matrix's diagonal
        always preserves PSD regardless of the cross terms -- so this is
        simultaneously the correctness fix and the reason no off-diagonal
        zeroing is needed (or wanted) here."""
        if self.P is None:
            return
        pos_floor2 = GATE_RECOVERY_POS_SIGMA_FLOOR_M ** 2
        vel_floor2 = GATE_RECOVERY_VEL_SIGMA_FLOOR_M_S ** 2
        self.P[0, 0] = max(self.P[0, 0], pos_floor2)
        self.P[1, 1] = max(self.P[1, 1], pos_floor2)
        self.P[2, 2] = max(self.P[2, 2], vel_floor2)
        self.P[3, 3] = max(self.P[3, 3], vel_floor2)

    # ---------------------------------------------------------------
    # Handoff boundary (design D3, ADR-0041; the "re-initialize, or provably
    # decay" standard from docs/ekf_design_brief.md). One-way: called
    # exactly once, at the same instant m4_intercept.py nulls cue_reader --
    # AFTER any warm-handoff seeding completes (H1, ADR-0041 adversarial
    # review: state warm-transfer is legitimate, but latch() must be the
    # LAST word on P, or a warm-handoff reseed immediately after latch()
    # would silently undo the D3.1 floor -- see seed_from_polar()'s own
    # defense-in-depth for the reverse ordering mistake).
    # ---------------------------------------------------------------
    def latch(self) -> None:
        """Close the cue channel structurally (correct_cue raises after
        this). If correct_cue was EVER called pre-latch, the position block
        of P may carry cue-shaped confidence that would otherwise survive
        the boundary UNBOUNDED (F3: "the natural config ... leaves P
        completely unbounded at latch") -- unconditionally re-inflate it to
        AT LEAST LATCH_POS_SIGMA_FLOOR_M (max(), never shrink -- M1, see
        _gate_recovery_reinflate's docstring for the PSD argument) so the
        first post-latch camera corrections are accepted even if the state
        itself is still cue-biased (the F1 confident-bias-lock escape
        hatch). Left UNTOUCHED (state AND P) when the cue was never used
        pre-latch -- a camera-only warm handoff needs no re-inflation; its P
        already reflects camera-only confidence, and forcing it wider would
        only make the terminal re-earn confidence it legitimately already
        has. p_diag_at_latch records P's diagonal AFTER this decision
        either way, for post-flight audit. Idempotent: a second call just
        re-latches and re-records the diagnostic (and, since it's max(),
        can only ever hold P steady or widen it further, never shrink it
        back toward the floor)."""
        if self._cue_called and self.P is not None:
            floor2 = LATCH_POS_SIGMA_FLOOR_M ** 2
            self.P[0, 0] = max(self.P[0, 0], floor2)
            self.P[1, 1] = max(self.P[1, 1], floor2)
        self._latched = True
        if self.P is not None:
            self.p_diag_at_latch = tuple(float(v) for v in np.diag(self.P))

    # ---------------------------------------------------------------
    # Warm-start (brief section 2.5). Seed x_hat AND P from a polar handoff
    # state (lambda, lambda_dot, R, rdot). The existing warm-handoff code
    # writes the scalar views (.x_hat/.xdot_hat); those route here.
    # ---------------------------------------------------------------
    def seed_from_polar(self, lam, lamdot, R, rdot, t=None) -> None:
        R = max(R, 1e-3)
        u = (math.cos(lam), math.sin(lam))
        p = (-math.sin(lam), math.cos(lam))   # unit perp
        # rel pos
        dn = R * u[0]
        de = R * u[1]
        # rel vel from polar rates: vrel = rdot*u + R*lamdot*perp
        dvn = rdot * u[0] + R * lamdot * p[0]
        dve = rdot * u[1] + R * lamdot * p[1]
        self.x = np.array([dn, de, dvn, dve])
        self.P = np.diag([WARM_POS_SIGMA_M ** 2, WARM_POS_SIGMA_M ** 2,
                          WARM_VEL_SIGMA_M_S ** 2, WARM_VEL_SIGMA_M_S ** 2])
        if self._latched:
            # H1 (ADR-0041 adversarial review) defense-in-depth: the real
            # m4_intercept.py wiring calls latch() AFTER this warm-start
            # completes, so latch() is normally the genuine last word on P
            # (see the "Handoff boundary" section comment above). This
            # branch protects any OTHER call ordering: state warm-transfer
            # is legitimate even if it somehow runs post-latch, but
            # confidence must never regress below the D3.1 floor once
            # latched -- max(), never shrink (same PSD argument as
            # _gate_recovery_reinflate).
            floor2 = LATCH_POS_SIGMA_FLOOR_M ** 2
            self.P[0, 0] = max(self.P[0, 0], floor2)
            self.P[1, 1] = max(self.P[1, 1], floor2)
        if t is not None:
            self._last_correction_t = t

    def _seed_scalar(self, kind: str, which: str, value: float) -> None:
        """Route a warm-handoff scalar write (.x_hat / .xdot_hat on a view)
        into the Cartesian state, holding the other polar components at their
        current best estimate. Applied incrementally as the four writes arrive
        (lam, lamdot, R, rdot) so the final state matches seed_from_polar."""
        lam = self.lambda_hat if self.x is not None else 0.0
        lamdot = self.lambda_dot_hat if self.x is not None else 0.0
        R = self.r_hat if self.x is not None else 1.0
        rdot = self.rdot_hat if self.x is not None else 0.0
        if kind == "lambda" and which == "pos":
            lam = value
        elif kind == "lambda" and which == "rate":
            lamdot = value
        elif kind == "range" and which == "pos":
            R = value
        elif kind == "range" and which == "rate":
            rdot = value
        self.seed_from_polar(lam, lamdot, R, rdot)

    def nees(self, true_rel_state) -> Optional[float]:
        """Normalized Estimation Error Squared vs a TRUE relative state
        [dn, de, dvn, dve] (scoring/consistency ONLY -- reconstructed from
        gt_* offline; never a command input). NEES = e^T P^-1 e."""
        if self.x is None:
            return None
        e = self.x - np.asarray(true_rel_state, dtype=float)
        try:
            Pinv = np.linalg.inv(self.P)
        except np.linalg.LinAlgError:
            return None
        return float(e @ Pinv @ e)
