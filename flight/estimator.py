"""flight.estimator -- the scalar alpha-beta (g-h) tracker that runs the LOS
azimuth and range channels of the camera-only terminal, plus the optional
Kalata adaptive-gain mode. Extracted VERBATIM from scripts/m4_intercept.py
(itself ported from guidance_lab.py) so behavior transfers bit-for-bit to the
real Pixhawk/Pi build (ADR-0076 add #18, P0.3). Pure Python + math + wrap_pi;
no gz/gt/cue/loop deps. m4 keeps its own audited copy; flight/tests/
test_estimator.py pins this mirror to agree with it.

Two-sentence idea: a constant-velocity tracker for one noisy, intermittent
scalar. `predict(dt)` coasts x_hat forward on the current rate every control
tick; `correct(meas, t)` folds in a fresh measurement, nudging both position
(alpha) and rate (beta) toward it.
"""

import math
from typing import Optional

from flight.geometry import wrap_pi

KALATA_LOG_EVERY = 20  # print sampled (alpha, beta) at correction #1 and every Nth after


def _clamp(x, lo, hi):
    """Clamp x into [lo, hi] (mirror of m3_static_intercept._clamp)."""
    return max(lo, min(hi, x))


def kalata_alpha_beta(sigma_process: float, sigma_meas: float, dt: float):
    """Steady-state alpha-beta gains from the Kalata tracking index (T.P.
    Kalata, 1984), given an assumed process (target-maneuver) noise std
    `sigma_process` (units = the tracked quantity's 2nd derivative), a
    measurement-noise std `sigma_meas` (units = the tracked quantity itself),
    and the ACTUAL sample interval `dt` since the last correction -- recomputing
    at the real interval is the point: a longer gap (e.g. after a dropout)
    produces a larger tracking index and hence larger alpha/beta, correctly
    trusting the next real measurement more because the prediction has drifted
    longer. Ported verbatim from guidance_lab.py / m4_intercept.py."""
    dt = max(dt, 1e-3)
    sigma_meas = max(sigma_meas, 1e-9)
    lambda_idx = sigma_process * dt * dt / sigma_meas
    r = (4.0 + lambda_idx - math.sqrt(8.0 * lambda_idx + lambda_idx * lambda_idx)) / 4.0
    alpha = _clamp(1.0 - r * r, 0.0, 0.999)
    beta = 2.0 * (2.0 - alpha) - 4.0 * math.sqrt(max(0.0, 1.0 - alpha))
    return alpha, beta


# Lab-recommended Kalata params (tracking-refinement study winner): lambda
# channel assumes 150 deg/s^2 process noise vs 6 deg measurement sigma; range
# channel 0.1 m/s^2 vs 0.5 m sigma (matches the camera range-noise expectation).
KALATA_LAMBDA_SIGMA_PROCESS_DEG_S2 = 150.0
KALATA_LAMBDA_SIGMA_MEAS_DEG = 6.0
KALATA_RANGE_SIGMA_PROCESS_M_S2 = 0.1
KALATA_RANGE_SIGMA_MEAS_M = 0.5


class AlphaBetaFilter:
    """Constant-velocity (g-h / alpha-beta) tracker for one scalar channel
    (either lambda, the inertial LOS azimuth, or R, the range).

    `predict()` is meant to be called every control tick; `correct()` only on
    ticks where a genuinely new (fresh) measurement is available. Set
    `angular=True` for the lambda channel: the RESIDUAL (measurement minus
    current estimate) is wrapped to [-pi, pi] before use, but x_hat itself is
    never wrapped -- that keeps lambda continuous (able to accumulate past
    +-180 deg as the vehicle spins) while still comparing correctly against a
    measurement that only makes sense mod 2*pi.

    KALATA MODE (opt-in): pass `kalata_sigma_process` (not None) to IGNORE the
    fixed alpha/beta and recompute them at every correct() from
    kalata_alpha_beta(...) using the ACTUAL elapsed time since the last
    correction. Default (None) is the untouched fixed-gain baseline.

    RATE CAP: clamp xdot_hat to +-rate_cap immediately after every correct().
    Because predict() forward-integrates x_hat off this SAME stored xdot_hat,
    one clamp here caps BOTH the rate a caller reads (a_cmd = N*Vc*lambda_dot)
    AND the filter's own forward integration. Default None = untouched baseline.
    """

    def __init__(
        self, alpha: float, beta: float, angular: bool = False,
        kalata_sigma_process=None, kalata_sigma_meas=None, label: Optional[str] = None,
        rate_cap: Optional[float] = None,
    ):
        self.alpha = alpha
        self.beta = beta
        self.angular = angular
        self.kalata_sigma_process = kalata_sigma_process
        self.kalata_sigma_meas = kalata_sigma_meas
        self.label = label
        self.rate_cap = rate_cap
        self.n_corrections = 0
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

    def correct(self, meas: float, t: float, gain_scale: float = 1.0) -> None:
        """`gain_scale` (default 1.0 = byte-identical baseline) scales BOTH
        gains for this one correction: how a fused lower-confidence source folds
        into a filter whose nominal gains are tuned for the camera, without
        touching the stored gains. Inverse-variance weight computed by the
        caller from known sensor specs."""
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
        if self.kalata_sigma_process is not None:
            alpha, beta = kalata_alpha_beta(self.kalata_sigma_process, self.kalata_sigma_meas, dt_since)
        else:
            alpha, beta = self.alpha, self.beta
        self.x_hat += alpha * gain_scale * residual
        self.xdot_hat += beta * gain_scale * residual / dt_since
        if self.rate_cap is not None:
            self.xdot_hat = _clamp(self.xdot_hat, -self.rate_cap, self.rate_cap)
        self.last_innovation = residual
        self._last_correction_t = t
        self.n_corrections += 1
