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

  (d) fusion capstone P1/P2 (design D2/D3, ADR-0041) --
      test_cue_polar_split_cross_range_never_touches_bearing (F2 fix: a pure
      cross-LOS cue offset must leave the state bit-identical; a pure
      along-LOS offset DOES move range);
      test_cue_polar_split_cross_range_accumulation_never_moves_bearing (the
      REAL F2 regression guard -- F2 is an ACCUMULATION effect a single shot
      cannot catch, see the ADR-0041 adversarial-review note below);
      test_latch_reinflates_position_block_only_if_cue_used (D3.1:
      unconditional latch re-inflation via max()/floor semantics, gated on
      whether the cue was ever used pre-latch);
      test_correct_cue_after_latch_raises_and_counts (the post-latch
      structural-illegality belt-and-suspenders);
      test_gate_recovery_after_n_consecutive_post_latch_rejections (D3.2, the
      F1 confident-bias-lock escape hatch); test_seed_from_polar_after_latch_
      respects_the_floor and test_seed_from_polar_before_latch_then_latch_
      still_reaches_the_floor (H1: latch() must be the LAST word on P
      regardless of call order relative to warm-handoff seeding).

ADR-0041 ADVERSARIAL REVIEW (2026-07-08, post-P1/P2 merge review) fixed two
defects here: M1 -- latch()/gate-recovery originally used ASSIGNMENT, which
could DECREASE a diagonal entry that had already grown past the floor (e.g.
a long dropout under Q=64) and leave P non-PSD; fixed to max()-based floor
semantics (never shrink). M2 -- the original single-shot polar-split test
used a loose <0.5 deg epsilon and was VACUOUS against the exact regression
it was meant to guard: reverting the polar split to a full-Cartesian dof=2
[dn,de] update still passed it, because F2 is an ACCUMULATION effect
invisible to one shot (measured via a real mutation probe against THIS
file: a Cartesian mutant drifts lambda_hat 2.86 deg after 60 corrections of
a persistent 1 m cross bias vs the fixed code's exact 0.0 deg) -- fixed by
strengthening the single-shot check to EXACT bit-identity and adding the
accumulation test as the real discriminator. See H1/M1/M2 in
docs/decisions.md's ADR-0041 addendum for the full review trace.

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
from ekf_tracker import EKFTracker, chi2_ppf, norm_ppf, wrap_pi  # noqa: E402


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


# =====================================================================
# (d) Fusion capstone P1/P2 (design D2/D3, ADR-0041)
# =====================================================================
def _camera_init(f, lam_deg=20.0, r=20.0, t=0.0):
    """Bring a fresh EKFTracker to its post-cold-init state with one joint
    camera correction (lambda, R) -- the state every correct_cue call in the
    real pipeline assumes (correct_cue is only ever reached after the
    camera has anchored the track)."""
    f.lambda_filter.predict(0.05)
    f.range_filter.predict(0.05)
    f.lambda_filter.correct(math.radians(lam_deg), t)
    f.range_filter.correct(r, t)
    return f


def test_cue_polar_split_cross_range_never_touches_bearing():
    """F2 fix (design D2): a PURE cross-LOS cue offset (perpendicular to the
    current lambda_hat) must leave the state BIT-IDENTICAL -- the door
    FusedTrack's polar split was built to close (ADR-0018: "the cue NEVER
    touches the angle"), now closed for correct_cue too. The same-magnitude
    offset applied ALONG the LOS instead DOES move range measurably -- the
    polar split discards only the cross component, not the whole channel.

    M2 (ADR-0041 adversarial review): this SINGLE-SHOT check is EXACT (not a
    degree-scale epsilon -- a loose epsilon here was proven vacuous against
    the exact regression it's meant to guard, see the module docstring), but
    it is not, by itself, a sufficient regression guard: a 5 m cross offset
    gets chi-square-gated by a full-Cartesian [dn,de] update too (rejected
    ==> "no state change" for a completely different, non-discriminating
    reason), and F2 is fundamentally an ACCUMULATION effect a single shot
    can't expose either way. The REAL guard is
    test_cue_polar_split_cross_range_accumulation_never_moves_bearing below
    -- keep this test for the single-shot orthogonality property (documents
    the exact mechanism: u . (k*p) == 0 for any k since u perp p) but never
    rely on it alone."""
    f = _camera_init(EKFTracker())
    lam_before, r_before = f.lambda_hat, f.r_hat
    x_before = f.x.copy()
    p_perp = (-math.sin(lam_before), math.cos(lam_before))  # unit cross-LOS
    cross_offset = 5.0
    cue_pos = (f.x[0] + cross_offset * p_perp[0], f.x[1] + cross_offset * p_perp[1])
    f.correct_cue(cue_pos, None, 0.1)

    assert np.array_equal(f.x, x_before), (
        "a pure cross-range cue offset must leave the state BIT-IDENTICAL "
        f"(x changed by {f.x - x_before}) -- the along-LOS innovation must "
        "be (numerically) exactly zero, not merely small"
    )
    assert f.lambda_hat == lam_before and f.r_hat == r_before

    # Same magnitude, but ALONG the LOS: must move range.
    f2 = _camera_init(EKFTracker())
    lam2_before, r2_before = f2.lambda_hat, f2.r_hat
    u = (math.cos(lam2_before), math.sin(lam2_before))      # unit along-LOS
    along_offset = 1.5   # inside the gate (a 5 m along offset gets legitimately gated, see below)
    cue_pos2 = (f2.x[0] + along_offset * u[0], f2.x[1] + along_offset * u[1])
    f2.correct_cue(cue_pos2, None, 0.1)
    r2_shift = f2.r_hat - r2_before
    assert r2_shift > 0.5, (
        f"along-LOS cue offset only moved range by {r2_shift:.4f} m -- "
        "expected a measurable correction toward the cue"
    )
    assert f2.n_gated == 0, "a moderate along-LOS offset should not trip the chi-square gate"


def test_cue_polar_split_cross_range_accumulation_never_moves_bearing():
    """M2 (ADR-0041 adversarial review) -- the REAL F2 regression guard.

    F2 is an ACCUMULATION effect: a single shot cannot distinguish the fixed
    polar-split code from a full-Cartesian [dn,de] regression (see the test
    above's docstring). This test drives 60 consecutive correct_cue calls
    (interleaved with the SAME predict() calls the real m4 tick loop makes
    every tick, cue or no cue), each a PERSISTENT 1 m cross-LOS bias -- fixed
    in WORLD frame relative to the TRUE (stationary) target position and a
    FIXED offset direction computed once up front, modeling a persistent
    datum/GPS-style bias rather than one that (unphysically) chases the
    tracker's own possibly-drifted estimate.

    Mutation probe performed by hand against this file (temporarily
    reverting correct_cue's polar split to the pre-F2-fix full-Cartesian
    dof=2 [dn,de] update, keeping everything else, incl. sigma_pos, R
    inflation and gating, identical): the CARTESIAN mutant drifts
    lambda_hat by 2.86 deg over these same 60 corrections (n_gated=0 the
    whole time -- a 1 m bias at ~20 m range is comfortably inside the
    gate); the FIXED polar-split code drifts by EXACTLY 0.0 deg (bit-exact:
    raw radian difference == 0.0, not merely below a printed-degrees
    rounding threshold) -- confirmed by direct inspection, not just the
    wrap_pi-rounded degrees display. This test asserts the exact-zero
    property that discriminates the two."""
    f = _camera_init(EKFTracker())
    lam0, r0 = f.lambda_hat, f.r_hat
    true_dn, true_de = f.x[0], f.x[1]           # fixed truth (stationary target)
    p_perp = (-math.sin(lam0), math.cos(lam0))  # fixed cross-LOS direction, computed once
    t = 0.1
    dt = 0.05
    n_shots = 60
    cross_bias_m = 1.0
    for _ in range(n_shots):
        f.lambda_filter.predict(dt)   # mirrors the m4 tick loop: predict every tick
        f.range_filter.predict(dt)
        cue_pos = (true_dn + cross_bias_m * p_perp[0], true_de + cross_bias_m * p_perp[1])
        f.correct_cue(cue_pos, None, t)
        t += dt

    assert f.lambda_hat == lam0, (
        f"a persistent PURE cross-range cue bias accumulated a lambda_hat "
        f"drift of {math.degrees(wrap_pi(f.lambda_hat - lam0)):.6f} deg over "
        f"{n_shots} corrections -- expected EXACTLY 0.0 (the mutation-probed "
        "full-Cartesian regression drifts 2.86 deg under the identical setup)"
    )
    assert f.r_hat == r0, "a persistent PURE cross-range cue bias must never move range either"


def test_cue_gross_along_los_offset_is_gated_not_silently_eaten():
    """Sanity companion to the split test above: the along-LOS channel still
    goes through the SAME chi-square gate as everything else (dof=1,
    GATE_CHI2_DOF1_P99) -- a gross along-LOS outlier is rejected, not
    silently absorbed, exactly like _scalar_range_update's existing gate."""
    f = _camera_init(EKFTracker())
    r_before = f.r_hat
    u = (math.cos(f.lambda_hat), math.sin(f.lambda_hat))
    gross_offset = 5.0
    cue_pos = (f.x[0] + gross_offset * u[0], f.x[1] + gross_offset * u[1])
    f.correct_cue(cue_pos, None, 0.1)
    assert f.n_gated == 1, "gross along-LOS cue offset should be gated"
    assert abs(f.r_hat - r_before) < 1e-9, "a gated update must not move the state"


def test_correct_cue_before_camera_init_uses_raw_cartesian_cold_start():
    """Interface-completeness edge case: correct_cue on a COLD (x is None)
    tracker still works (cue-only cold start), using the raw rel_pos_ned --
    there is no lambda_hat yet to polar-split against. The real
    m4_intercept.py wiring never reaches this branch (it only calls
    correct_cue once the EKF is camera-initialized), but the interface must
    not crash if it's ever driven this way directly."""
    f = EKFTracker()
    f.correct_cue((18.0, 4.0), (1.0, -0.5), 0.0)
    assert f.initialized
    assert abs(f.x[0] - 18.0) < 1e-9 and abs(f.x[1] - 4.0) < 1e-9
    assert abs(f.x[2] - 1.0) < 1e-9 and abs(f.x[3] - (-0.5)) < 1e-9


def test_latch_reinflates_position_block_only_if_cue_used():
    """D3.1: unconditional latch re-inflation of P's position block to AT
    LEAST LATCH_POS_SIGMA_FLOOR_M -- but ONLY when correct_cue was ever
    called pre-latch. A camera-only track's P is left completely untouched
    (it already reflects legitimate camera-only confidence; forcing it
    wider would make the terminal re-earn confidence it already has).

    M1 (ADR-0041 adversarial review): floor semantics are max()-based, so
    this asserts P >= floor (not ==) -- see the "grown wide" case below,
    which is the actual regression M1 guards: assignment could DECREASE a
    diagonal entry that had already grown past the floor (e.g. a long
    dropout under Q=64), and shrinking one diagonal entry while leaving
    cross terms untouched can leave P non-PSD."""
    # Cue was used pre-latch, P still small (typical case): floor RAISES it.
    f1 = _camera_init(EKFTracker())
    f1.correct_cue((f1.x[0] - 1.0, f1.x[1] + 0.5), (0.5, 0.0), 0.1)
    assert f1._cue_called
    f1.latch()
    floor2 = ekf_mod.LATCH_POS_SIGMA_FLOOR_M ** 2
    assert f1.P[0, 0] >= floor2 - 1e-9
    assert f1.P[1, 1] >= floor2 - 1e-9
    assert f1.p_diag_at_latch is not None
    assert f1.p_diag_at_latch[0] >= floor2 - 1e-9

    # Cue never used pre-latch -- P must be byte-identical before/after.
    f2 = _camera_init(EKFTracker())
    p_before = f2.P.copy()
    assert not f2._cue_called
    f2.latch()
    assert np.array_equal(f2.P, p_before), (
        "P must be completely untouched by latch() when the cue was never "
        "called pre-latch"
    )
    assert f2.p_diag_at_latch == tuple(float(v) for v in np.diag(p_before))

    # M1: cue used pre-latch, but P had ALREADY grown past the floor (e.g. a
    # long dropout under Q=64) -- latch() must NOT shrink it back down; the
    # variance stays exactly what it was (max() is a no-op when already
    # above the floor).
    f3 = _camera_init(EKFTracker())
    f3.correct_cue((f3.x[0] - 1.0, f3.x[1] + 0.5), (0.5, 0.0), 0.1)
    assert f3._cue_called
    grown_var = floor2 * 5.0   # well above the floor
    f3.P[0, 0] = grown_var
    f3.P[1, 1] = grown_var
    f3.latch()
    assert f3.P[0, 0] == grown_var, (
        "latch() must not SHRINK a position variance that was already "
        "above the floor (M1: max()-based floor, never assignment)"
    )
    assert f3.P[1, 1] == grown_var


def test_seed_from_polar_before_latch_then_latch_still_reaches_the_floor():
    """H1 (ADR-0041 adversarial review) regression test, exactly the ordering
    now used in m4_intercept.py: correct_cue used pre-latch (so _cue_called
    is True), THEN a warm-handoff-style seed_from_polar() call (which resets
    P to the SMALL WARM_POS/VEL_SIGMA values -- legitimate state transfer,
    but P-shrinking), THEN latch() (matching the fixed call order). latch()
    must be the LAST word on P: the position block must satisfy the D3.1
    floor AFTERWARD, undoing the warm-seed's P reset rather than leaving it
    silently in place (the exact bug: latch() used to run BEFORE seeding,
    so the seed's P reset silently won regardless of what latch() had just
    done)."""
    f = _camera_init(EKFTracker())
    f.correct_cue((f.x[0] - 1.0, f.x[1] + 0.5), (0.5, 0.0), 0.1)
    assert f._cue_called

    # Simulate the warm-handoff seeding block -- resets P to the SMALL warm
    # values, well below the D3.1 floor.
    f.seed_from_polar(f.lambda_hat, f.lambda_dot_hat, f.r_hat, f.rdot_hat, t=0.2)
    warm_var = ekf_mod.WARM_POS_SIGMA_M ** 2
    assert abs(f.P[0, 0] - warm_var) < 1e-9, "seed_from_polar should reset P to the warm values"
    assert warm_var < ekf_mod.LATCH_POS_SIGMA_FLOOR_M ** 2, (
        "test setup assumption broken: WARM_POS_SIGMA_M must be smaller "
        "than LATCH_POS_SIGMA_FLOOR_M for this to be a meaningful regression check"
    )

    # NEW call order (H1 fix): latch() runs AFTER seeding.
    f.latch()
    floor2 = ekf_mod.LATCH_POS_SIGMA_FLOOR_M ** 2
    assert f.P[0, 0] >= floor2 - 1e-9, (
        f"P[0,0]={f.P[0, 0]:.4f} is below the D3.1 floor {floor2:.4f} after "
        "latch() -- the warm-handoff reseed silently undid the re-inflation "
        "(H1 regression)"
    )
    assert f.P[1, 1] >= floor2 - 1e-9
    assert f.p_diag_at_latch[0] >= floor2 - 1e-9


def test_seed_from_polar_after_latch_respects_the_floor():
    """H1 (ADR-0041 adversarial review) defense-in-depth: if seed_from_polar
    is ever called AFTER latch() (the real m4_intercept.py wiring never does
    this -- see test_ekf_latch_runs_after_warm_handoff_seeding in
    test_honesty_static.py, which proves the actual call order statically --
    but the API itself should not silently regress P below the floor for
    ANY caller/ordering). State warm-transfer is legitimate even post-latch;
    confidence must never regress below the D3.1 floor once latched."""
    f = _camera_init(EKFTracker())
    f.correct_cue((f.x[0] - 1.0, f.x[1] + 0.5), (0.5, 0.0), 0.1)
    f.latch()
    floor2 = ekf_mod.LATCH_POS_SIGMA_FLOOR_M ** 2
    assert f.P[0, 0] >= floor2 - 1e-9   # sanity: floor already applied

    # A hypothetical post-latch seed_from_polar call (state warm-transfer is
    # legal; P must not regress below the floor because of it).
    f.seed_from_polar(f.lambda_hat, f.lambda_dot_hat, f.r_hat, f.rdot_hat, t=0.3)
    assert f.P[0, 0] >= floor2 - 1e-9, (
        "seed_from_polar() called AFTER latch() must still respect the "
        "D3.1 floor (H1 defense-in-depth)"
    )
    assert f.P[1, 1] >= floor2 - 1e-9


def test_correct_cue_after_latch_raises_and_counts():
    """D3.1 belt-and-suspenders: correct_cue is structurally illegal after
    latch() -- it must raise AND increment cue_updates_post_handoff (the
    counter the S2_RESULT line reports, which must read 0 on every real
    flight since m4_intercept.py nulls cue_reader at the same instant)."""
    f = _camera_init(EKFTracker())
    f.latch()
    assert f.cue_updates_post_handoff == 0
    with pytest.raises(RuntimeError):
        f.correct_cue((f.x[0] + 1.0, f.x[1]), None, 0.2)
    assert f.cue_updates_post_handoff == 1
    with pytest.raises(RuntimeError):
        f.correct_cue((f.x[0] + 1.0, f.x[1]), (0.1, 0.1), 0.3)
    assert f.cue_updates_post_handoff == 2


def test_gate_recovery_after_n_consecutive_post_latch_rejections():
    """D3.2 (F1 fix): after GATE_RECOVERY_STREAK_N consecutive post-latch
    camera rejections, P's position+velocity blocks are re-inflated to AT
    LEAST the declared floors and the streak resets -- so the NEXT (4th,
    normal) camera correction is accepted, escaping the confident-bias-lock
    F1 demonstrated at the project's own constants.

    M1 (ADR-0041 adversarial review): floor semantics are max()-based, so
    this asserts P >= floor (not ==)."""
    dt = 0.05
    rng = np.random.default_rng(11)
    f = EKFTracker(q_accel_psd=1.0)
    x = np.array([20.0, 3.0, -1.5, 0.3], dtype=float)
    for k in range(40):
        x[0] += x[2] * dt
        x[1] += x[3] * dt
        R = math.hypot(x[0], x[1])
        lam = math.atan2(x[1], x[0])
        f.lambda_filter.predict(dt)
        f.range_filter.predict(dt)
        f.lambda_filter.correct(lam + math.radians(0.5) * rng.standard_normal(), k * dt)
        f.range_filter.correct(R + 0.10 * R * rng.standard_normal(), k * dt)

    f.latch()
    assert f.camera_gated_post_handoff == 0

    t = 100.0
    for i in range(ekf_mod.GATE_RECOVERY_STREAK_N):
        t += dt
        f.lambda_filter.predict(dt)
        f.range_filter.predict(dt)
        # Gross outlier: 60 deg bearing error + 50 m range error -- reliably
        # gated (nis far above GATE_CHI2_DOF2_P99=9.21) on a converged track.
        f.lambda_filter.correct(math.atan2(x[1], x[0]) + math.radians(60), t)
        f.range_filter.correct(math.hypot(x[0], x[1]) + 50.0, t)

    assert f.camera_gated_post_handoff == ekf_mod.GATE_RECOVERY_STREAK_N
    assert f._post_latch_gate_streak == 0, "streak must reset once recovery fires"
    pos_floor2 = ekf_mod.GATE_RECOVERY_POS_SIGMA_FLOOR_M ** 2
    vel_floor2 = ekf_mod.GATE_RECOVERY_VEL_SIGMA_FLOOR_M_S ** 2
    assert f.P[0, 0] >= pos_floor2 - 1e-9
    assert f.P[1, 1] >= pos_floor2 - 1e-9
    assert f.P[2, 2] >= vel_floor2 - 1e-9
    assert f.P[3, 3] >= vel_floor2 - 1e-9

    # 4th innovation: a NORMAL (small) camera correction -- must be accepted.
    n_corr_before = f.n_corrections
    gated_before = f.camera_gated_post_handoff
    t += dt
    f.lambda_filter.predict(dt)
    f.range_filter.predict(dt)
    f.lambda_filter.correct(math.atan2(x[1], x[0]), t)
    f.range_filter.correct(math.hypot(x[0], x[1]), t)
    assert f.n_corrections == n_corr_before + 1, "the 4th (normal) correction must be accepted"
    assert f.camera_gated_post_handoff == gated_before, "the 4th correction must not be gated"


def test_gate_recovery_never_shrinks_p_already_above_the_floor():
    """M1 (ADR-0041 adversarial review) companion to the latch() case: if P
    has already grown past the gate-recovery floor (e.g. a long dropout
    under Q=64) when the recovery trigger fires, _gate_recovery_reinflate
    must NOT shrink it back down -- max()-based floor, never assignment
    (the PSD argument: increasing a diagonal entry always preserves PSD;
    decreasing one while leaving cross terms untouched need not)."""
    f = _camera_init(EKFTracker())
    f.latch()
    pos_floor2 = ekf_mod.GATE_RECOVERY_POS_SIGMA_FLOOR_M ** 2
    vel_floor2 = ekf_mod.GATE_RECOVERY_VEL_SIGMA_FLOOR_M_S ** 2
    grown_pos = pos_floor2 * 4.0
    grown_vel = vel_floor2 * 4.0
    f.P[0, 0] = grown_pos
    f.P[1, 1] = grown_pos
    f.P[2, 2] = grown_vel
    f.P[3, 3] = grown_vel
    f._gate_recovery_reinflate()
    assert f.P[0, 0] == grown_pos, "gate-recovery must not shrink an already-wide position variance"
    assert f.P[1, 1] == grown_pos
    assert f.P[2, 2] == grown_vel, "gate-recovery must not shrink an already-wide velocity variance"
    assert f.P[3, 3] == grown_vel


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
