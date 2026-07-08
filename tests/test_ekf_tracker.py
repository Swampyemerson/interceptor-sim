#!/usr/bin/env python3
"""OFFLINE unit tests for scripts/ekf_tracker.py (ADR-0033 item 3, the
Cartesian constant-velocity EKF target tracker). NO sim / PX4 / Gazebo: pure
synthetic-data checks that run in milliseconds on every commit, exactly like
tests/test_honesty_static.py.

Three checks, mapping to the task's requirements:

  (a) test_nis_nees_consistency -- on a synthetic constant-velocity relative
      target with KNOWN process and measurement noise (matched to the filter's
      Q and R), the EKF's time-/ensemble-averaged NIS and NEES sit INSIDE
      their chi-square consistency bands (brief section 2.4 tuning check).
      This is the disciplined replacement for hand-tuning gains.

  (b) test_long_gap_gain_stays_sane / test_burst_cadence_not_deaf -- the
      degenerate-cadence cases that killed Kalata's closed-form adaptive gain
      (ADR-0013: alpha=0.031/beta~0 DEAF at burst cadence; alpha=0.999/
      beta=1.876 BLOW-UP after multi-second gaps). The EKF's P-carried gain
      must instead (i) grow its uncertainty sanely through a multi-second gap,
      stay positive-definite, and incorporate the next measurement with a
      BOUNDED gain (no blow-up), and (ii) NOT go deaf at fast burst cadence
      (it still tracks the target).

  (c) test_tracker_defaults_to_alphabeta / test_alphabeta_construction_...
      -- the byte-identical-default guarantees: --tracker defaults to
      'alphabeta', '--tracker ekf' parses, the EKF views satisfy the
      AlphaBetaFilter drop-in contract, and the alpha-beta construction lines
      + gains in m4_intercept.py are unchanged from HEAD.

Run:  .venv/bin/python -m pytest tests/test_ekf_tracker.py -v
"""

import math
import os
import sys

import numpy as np
import pytest

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts")
sys.path.insert(0, SCRIPTS)

import ekf_tracker as ekf_mod  # noqa: E402
from ekf_tracker import EKFTracker, chi2_ppf, norm_ppf  # noqa: E402


# =====================================================================
# (a) NIS / NEES filter-consistency (brief section 2.4)
# =====================================================================
def _simulate_cv(ekf, q, sigma_beta, frac, dt, steps, rng, start, vel):
    """Drive `ekf` with a synthetic constant-velocity RELATIVE target whose
    process noise EXACTLY matches the filter's CWNA Q (so NEES is consistent)
    and whose camera measurements match the filter's R. Returns per-step
    (nis, nees) after a burn-in, plus the final relative-position RMSE."""
    # Discrete CWNA process-noise covariance per axis (matches EKFTracker._Q).
    t2, t3 = dt * dt, dt * dt * dt
    Qax = q * np.array([[t3 / 3.0, t2 / 2.0], [t2 / 2.0, dt]])
    Lax = np.linalg.cholesky(Qax)  # 2x2, drives [pos, vel] per axis

    x_true = np.array([start[0], start[1], vel[0], vel[1]], dtype=float)
    F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)

    nis_samples, nees_samples = [], []
    burn = max(40, steps // 5)
    for k in range(steps):
        # Truth propagate + matched process noise.
        x_true = F @ x_true
        for ax in (0, 1):  # north, east: correlated (pos,vel) noise
            w = Lax @ rng.standard_normal(2)
            x_true[ax] += w[0]
            x_true[ax + 2] += w[1]

        dn, de = x_true[0], x_true[1]
        R_true = math.hypot(dn, de)
        lam_true = math.atan2(de, dn)
        lam_meas = lam_true + sigma_beta * rng.standard_normal()
        R_meas = R_true + (frac * R_true) * rng.standard_normal()

        t = (k + 1) * dt
        ekf.lambda_filter.predict(dt)
        ekf.range_filter.predict(dt)
        ekf.lambda_filter.correct(lam_meas, t)
        ekf.range_filter.correct(R_meas, t)

        if k >= burn and ekf.n_corrections > 0 and ekf.last_nis is not None:
            nis_samples.append(ekf.last_nis)
            nees_samples.append(ekf.nees(x_true))
    rmse = math.hypot(ekf.x[0] - x_true[0], ekf.x[1] - x_true[1])
    return nis_samples, nees_samples, rmse


def test_nis_nees_consistency():
    dt = 0.05
    steps = 260
    q = 0.5          # PSD kept modest so the test trajectory stays away from CPA;
    sigma_beta = math.radians(0.5)
    frac = 0.10
    n_runs = 40

    all_nis, all_nees = [], []
    for run in range(n_runs):
        rng = np.random.default_rng(1000 + run)
        # gating OFF for the clean consistency check (gating is a separate,
        # deliberately-inert-on-clean-data mechanism, tested below).
        f = EKFTracker(q_accel_psd=q, cam_bearing_sigma_deg=0.5,
                       cam_range_frac=frac, gating=False)
        nis, nees, _ = _simulate_cv(
            f, q, sigma_beta, frac, dt, steps, rng, start=(28.0, 4.0), vel=(-1.0, 0.3))
        all_nis += nis
        all_nees += [v for v in nees if v is not None]

    mean_nis = float(np.mean(all_nis))
    mean_nees = float(np.mean(all_nees))
    n1, n2 = len(all_nis), len(all_nees)

    # Two-sided 99% consistency bands on the SAMPLE MEAN: for m i.i.d.
    # chi2(dof) samples, m*mean ~ chi2(dof*m).
    nis_lo = chi2_ppf(0.005, 2 * n1) / n1
    nis_hi = chi2_ppf(0.995, 2 * n1) / n1
    nees_lo = chi2_ppf(0.005, 4 * n2) / n2
    nees_hi = chi2_ppf(0.995, 4 * n2) / n2

    print(f"\n[NIS]  mean={mean_nis:.3f}  band=[{nis_lo:.3f}, {nis_hi:.3f}]  (dof=2, n={n1})")
    print(f"[NEES] mean={mean_nees:.3f}  band=[{nees_lo:.3f}, {nees_hi:.3f}]  (dof=4, n={n2})")

    assert nis_lo <= mean_nis <= nis_hi, (
        f"NIS {mean_nis:.3f} outside 99% band [{nis_lo:.3f},{nis_hi:.3f}] -> Q/R mistuned")
    assert nees_lo <= mean_nees <= nees_hi, (
        f"NEES {mean_nees:.3f} outside 99% band [{nees_lo:.3f},{nees_hi:.3f}] -> filter inconsistent")


# =====================================================================
# (b) Degenerate cadence -- the Kalata failure modes (ADR-0013)
# =====================================================================
def _converge(f, dt=0.05, n=40, rng=None, start=(20.0, 3.0), vel=(-1.0, 0.2),
              sigma_beta=math.radians(0.5), frac=0.10):
    """Bring the EKF to a settled state with n clean camera updates; return
    the true relative state at the end."""
    rng = rng or np.random.default_rng(7)
    x = np.array([start[0], start[1], vel[0], vel[1]], dtype=float)
    for _ in range(n):
        x[0] += x[2] * dt
        x[1] += x[3] * dt
        R = math.hypot(x[0], x[1])
        lam = math.atan2(x[1], x[0])
        f.lambda_filter.predict(dt)
        f.range_filter.predict(dt)
        f.lambda_filter.correct(lam + sigma_beta * rng.standard_normal(), _ * dt)
        f.range_filter.correct(R + frac * R * rng.standard_normal(), _ * dt)
    return x


def test_long_gap_gain_stays_sane():
    """Multi-second measurement gap: P must GROW (uncertainty rises), stay
    positive-definite, and the next correction must be incorporated with a
    BOUNDED gain -- the opposite of Kalata's alpha=0.999/beta=1.876 blow-up
    after multi-second gaps (ADR-0013)."""
    dt = 0.05
    f = EKFTracker(q_accel_psd=1.0)
    x_true = _converge(f, dt=dt, n=40)

    tr_before = float(np.trace(f.P))
    velvar_before = float(f.P[2, 2])

    # ~3 s gap: predict-only, no corrections.
    n_gap = 60
    for _ in range(n_gap):
        f.predict(dt)
        x_true[0] += x_true[2] * dt
        x_true[1] += x_true[3] * dt

    tr_after = float(np.trace(f.P))
    velvar_after = float(f.P[2, 2])

    # Uncertainty grew (gain will rise) but the filter is still well-posed.
    assert tr_after > tr_before, "P did not grow across the gap"
    assert velvar_after > velvar_before, "velocity variance did not grow across the gap"
    assert np.all(np.isfinite(f.x)) and np.all(np.isfinite(f.P))
    np.linalg.cholesky(f.P)  # raises if not positive-definite

    # Gain on the NEXT correction, computed explicitly, must be BOUNDED.
    dn, de = f.x[0], f.x[1]
    R = math.hypot(dn, de)
    R2 = R * R
    H = np.array([[-de / R2, dn / R2, 0, 0], [dn / R, de / R, 0, 0]])
    sigma_R = max(ekf_mod.CAM_RANGE_SIGMA_FLOOR_M, f.cam_range_frac * R)
    Rmat = np.diag([f.cam_bearing_sigma ** 2, sigma_R ** 2])
    S = H @ f.P @ H.T + Rmat
    K = f.P @ H.T @ np.linalg.inv(S)
    assert np.all(np.isfinite(K)), "Kalman gain went non-finite after the gap"
    # The UNIT-FREE sane-gain check (raw |K| entries carry units -- the
    # bearing->cross-range gain is ~R m/rad, correctly O(R), not a blow-up).
    # The dimensionless effective gain is K@H: the FRACTION of the state the
    # measurement corrects. For a stable blend its spectral radius stays < 1
    # (it approaches, never exceeds, "trust the measurement"). Kalata's
    # closed-form gain went DEGENERATE here -- beta = 1.876 > 1 after
    # multi-second gaps (ADR-0013). The P-carried EKF gain does not.
    rho = float(max(abs(np.linalg.eigvals(K @ H))))
    assert rho < 1.0 + 1e-6, f"effective gain spectral radius {rho:.4f} >= 1 (Kalata blow-up)"
    kmax = float(np.max(np.abs(K)))

    # Deliver the correction; posterior uncertainty must SHRINK and the state
    # must not diverge from truth.
    R_true = math.hypot(x_true[0], x_true[1])
    lam_true = math.atan2(x_true[1], x_true[0])
    t_now = 100.0
    f.lambda_filter.correct(lam_true, t_now)
    f.range_filter.correct(R_true, t_now)
    assert float(np.trace(f.P)) < tr_after, "P did not shrink after the post-gap correction"
    perr = math.hypot(f.x[0] - x_true[0], f.x[1] - x_true[1])
    assert perr < 5.0, f"post-gap position error {perr:.2f} m diverged"
    print(f"\n[gap] trace P {tr_before:.2f} -> {tr_after:.2f} (gap) -> "
          f"{float(np.trace(f.P)):.2f} (corr); max|K|={kmax:.3f}; post-gap perr={perr:.3f} m")


def test_burst_cadence_not_deaf():
    """Fast burst cadence (dt=0.02, ~50 Hz): the filter must still INCORPORATE
    measurements and track a target -- Kalata went DEAF here (alpha=0.031/
    beta~0, Vc pinned at floor). We verify the EKF tracks a constant-velocity
    target to low RMSE and its velocity estimate matches truth."""
    dt = 0.02
    rng = np.random.default_rng(11)
    f = EKFTracker(q_accel_psd=1.0)
    sigma_beta = math.radians(0.5)
    frac = 0.10
    x = np.array([18.0, 2.0, -2.5, 0.4], dtype=float)
    perrs = []
    for k in range(400):
        x[0] += x[2] * dt
        x[1] += x[3] * dt
        R = math.hypot(x[0], x[1])
        lam = math.atan2(x[1], x[0])
        f.lambda_filter.predict(dt)
        f.range_filter.predict(dt)
        f.lambda_filter.correct(lam + sigma_beta * rng.standard_normal(), k * dt)
        f.range_filter.correct(R + frac * R * rng.standard_normal(), k * dt)
        if k > 100:
            perrs.append(math.hypot(f.x[0] - x[0], f.x[1] - x[1]))

    pos_rmse = float(np.sqrt(np.mean(np.square(perrs))))
    verr = math.hypot(f.x[2] - x[2], f.x[3] - x[3])
    # Not deaf: it actually tracks. (A deaf filter would drift unboundedly.)
    assert pos_rmse < 1.0, f"burst-cadence position RMSE {pos_rmse:.3f} m -> filter not tracking"
    assert verr < 1.5, f"burst-cadence velocity error {verr:.3f} m/s -> velocity channel deaf"
    print(f"\n[burst] pos_rmse={pos_rmse:.3f} m  vel_err={verr:.3f} m/s  "
          f"(n_corr={f.n_corrections})")


def test_gating_inert_on_clean_data_but_catches_outlier():
    """Brief section 2.6 pre-registered null: gating is INERT on clean
    Gaussian data (near-zero rejections) yet DOES catch a gross outlier."""
    dt = 0.05
    rng = np.random.default_rng(3)
    f = EKFTracker(q_accel_psd=1.0, gating=True)
    x = np.array([20.0, 3.0, -1.5, 0.3], dtype=float)
    for k in range(200):
        x[0] += x[2] * dt
        x[1] += x[3] * dt
        R = math.hypot(x[0], x[1])
        lam = math.atan2(x[1], x[0])
        f.lambda_filter.predict(dt)
        f.range_filter.predict(dt)
        f.lambda_filter.correct(lam + math.radians(0.5) * rng.standard_normal(), k * dt)
        f.range_filter.correct(R + 0.10 * R * rng.standard_normal(), k * dt)
    # Inert on clean data: at most a tiny handful of the ~200 updates gated.
    assert f.n_gated <= 4, f"gating not inert on clean data ({f.n_gated} rejected)"
    gated_before = f.n_gated
    # A gross 10 m range outlier at ~15 m range should be rejected.
    f.lambda_filter.predict(dt)
    f.range_filter.predict(dt)
    f.lambda_filter.correct(math.atan2(x[1], x[0]), 100.0)
    f.range_filter.correct(math.hypot(x[0], x[1]) + 12.0, 100.0)
    assert f.n_gated == gated_before + 1, "gross outlier was not gated"
    print(f"\n[gate] clean rejections={gated_before}/200; outlier rejected=OK")


# =====================================================================
# (c) Byte-identical-default guarantees + drop-in contract
# =====================================================================
def _load_m4():
    import importlib
    import m4_intercept
    importlib.reload(m4_intercept)
    return m4_intercept


def test_tracker_defaults_to_alphabeta(monkeypatch):
    m4 = _load_m4()
    monkeypatch.setattr(sys, "argv", ["m4_intercept.py", "--law", "pronav"])
    args = m4.parse_args()
    assert args.tracker == "alphabeta", "default --tracker must be alphabeta (byte-identical path)"


def test_tracker_ekf_parses(monkeypatch):
    m4 = _load_m4()
    monkeypatch.setattr(sys, "argv", ["m4_intercept.py", "--law", "pronav", "--tracker", "ekf"])
    args = m4.parse_args()
    assert args.tracker == "ekf"


def test_alphabeta_gains_unchanged():
    """The frozen M4/M5 g-h gains must not have moved."""
    m4 = _load_m4()
    assert m4.ALPHA == 0.5
    assert m4.BETA_GAIN_LAMBDA == 0.30
    assert m4.BETA_GAIN_RANGE == 0.15


def test_alphabeta_construction_source_is_byte_identical():
    """Guard that the two original alpha-beta construction statements remain
    verbatim in m4_intercept.py and the EKF override is strictly gated behind
    `if args.tracker == "ekf":` (so the default path is unchanged)."""
    src = open(os.path.join(SCRIPTS, "m4_intercept.py")).read()
    assert (
        'lambda_filter = AlphaBetaFilter(\n'
        '        ALPHA, BETA_GAIN_LAMBDA, angular=True, rate_cap=lambda_rate_cap,\n'
        '        **lambda_kalata_kwargs,\n'
        '    )' in src
    ), "lambda_filter construction changed -- alpha-beta path not byte-identical"
    assert (
        'range_filter = AlphaBetaFilter(ALPHA, BETA_GAIN_RANGE, angular=False, '
        '**range_kalata_kwargs)' in src
    ), "range_filter construction changed -- alpha-beta path not byte-identical"
    assert 'if args.tracker == "ekf":' in src, "EKF branch is not gated on --tracker ekf"
    # The EKF import lives INSIDE the branch (not imported on the default path).
    idx = src.index('if args.tracker == "ekf":')
    assert 'from ekf_tracker import EKFTracker' in src[idx:idx + 400], \
        "EKF import is not confined to the --tracker ekf branch"


def test_ekf_views_satisfy_alphabeta_dropin_contract():
    """The two views must expose exactly the AlphaBetaFilter surface the
    guidance loop touches, so they are a genuine drop-in."""
    f = EKFTracker()
    for view in (f.lambda_filter, f.range_filter):
        for attr in ("predict", "correct", "x_hat", "xdot_hat",
                     "initialized", "_last_correction_t"):
            assert hasattr(view, attr), f"view missing {attr}"
        # uninitialized read-outs mirror AlphaBetaFilter (None / 0.0).
        assert view.x_hat is None
        assert view.xdot_hat == 0.0
        assert view.initialized is False

    # After a fresh camera correction pair the read-outs are live and sane.
    f.lambda_filter.predict(0.05)
    f.range_filter.predict(0.05)
    f.lambda_filter.correct(math.radians(20.0), 0.0)   # lambda ~ 20 deg
    f.range_filter.correct(15.0, 0.0)                  # R = 15 m
    assert f.lambda_filter.initialized
    assert abs(f.range_filter.x_hat - 15.0) < 1e-6
    assert abs(math.degrees(f.lambda_filter.x_hat) - 20.0) < 1e-6


def test_quantile_helpers_sane():
    # norm_ppf sanity, and chi2_ppf (Wilson-Hilferty) accuracy where it is
    # actually USED: the LARGE-dof consistency bands (2n/4n, n in the hundreds
    # to thousands). WH is a cube-root-normal approximation, weakest at tiny
    # dof, so those get looser tolerances; the large-dof case is tight.
    assert abs(norm_ppf(0.975) - 1.959964) < 1e-3
    assert abs(chi2_ppf(0.95, 1) - 3.8415) < 0.15    # dof=1: WH weakest
    assert abs(chi2_ppf(0.95, 2) - 5.9915) < 0.10
    assert abs(chi2_ppf(0.5, 4) - 3.3567) < 0.1
    # Large dof (as used by the NIS/NEES bands): near-exact.
    assert abs(chi2_ppf(0.975, 1000) - 1089.531) < 2.0
    assert abs(chi2_ppf(0.025, 1000) - 914.257) < 2.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
