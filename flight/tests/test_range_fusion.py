"""Pin flight.range_fusion (Phase C: bbox size-range prior + closing rate +
tau, fused as a coarse prior into the existing alpha-beta range channel).
Pure-math behavioral assertions on synthetic closing geometry -- no sim.
Run standalone (exit 0/1) or under pytest.
"""

import math

from flight.estimator import AlphaBetaFilter
from flight.range_fusion import (
    SizeRangeEstimator,
    fusion_weight,
    size_range_from_width,
    size_range_sigma,
)

FX = 600.0
W = 0.30


def _feed_closing(est, r0, vc, t0=0.0, t1=1.5, hz=30.0):
    """Feed the estimator detector widths from a constant-Vc closing
    geometry R(t) = r0 - vc*t, at the detection rate. Returns final t."""
    dt = 1.0 / hz
    t = t0
    while t <= t1:
        est.predict(dt)
        r = r0 - vc * t
        est.correct(FX * W / r, t)
        t += dt
    return t - dt


def test_size_range_recovers_known_r():
    """The pinhole inverse recovers a known range exactly: a target of span
    W at range R images at w = fx*W/R px -> R back out."""
    for r_true in (2.0, 6.0, 12.0, 20.0):
        w_px = FX * W / r_true
        r = size_range_from_width(w_px, FX, W)
        assert abs(r - r_true) < 1e-9, (r_true, r)


def test_degenerate_width_is_none():
    assert size_range_from_width(None, FX) is None
    assert size_range_from_width(0.0, FX) is None
    assert size_range_from_width(-3.0, FX) is None
    assert size_range_sigma(None, 10.0) is None
    assert size_range_sigma(6.0, 0.0) is None


def test_sigma_model_honest():
    """The declared noise: sigma_R/R = sigma_w/w RSS'd with the foreshorten
    fraction. A +-1 px error on a 10 px box alone is 10% of range (the
    levers-doc claim); the 30-deg-bank foreshortening term adds on top."""
    r = 12.0
    s_clean = size_range_sigma(r, 10.0, px_sigma=1.0, foreshorten_frac=0.0)
    assert abs(s_clean - 0.10 * r) < 1e-9
    s_bank = size_range_sigma(r, 10.0, px_sigma=1.0, foreshorten_frac=0.13)
    assert abs(s_bank - r * math.hypot(0.10, 0.13)) < 1e-9
    assert s_bank > s_clean
    # A FEW-px box is proportionally worse: 1 px on 3 px = 33% of range.
    s_tiny = size_range_sigma(r, 3.0, px_sigma=1.0, foreshorten_frac=0.0)
    assert abs(s_tiny - r / 3.0) < 1e-9


def test_estimator_recovers_range_and_closing_rate():
    """Constant-Vc closing (12 m -> 6 m at 4 m/s, 30 Hz widths): the
    width-channel filter's derived range tracks truth and the derived
    closing rate converges near -Vc. (The width grows hyperbolically while
    the filter is constant-velocity, so a lag bias is expected -- the
    tolerance is honest, not tight.)"""
    est = SizeRangeEstimator(FX, W)
    t_end = _feed_closing(est, r0=12.0, vc=4.0, t1=1.5)
    r_true = 12.0 - 4.0 * t_end
    assert est.initialized
    assert abs(est.range_m - r_true) < 0.6, (est.range_m, r_true)
    assert est.rdot_m_s < -2.5, est.rdot_m_s          # clearly closing, near -4
    assert abs(est.rdot_m_s + 4.0) < 1.5, est.rdot_m_s


def test_tau_counts_down():
    """Tau = w/wdot must track the true time-to-contact R/Vc and COUNT DOWN
    ~1 s per s: sampled 0.5 s apart, the tau estimate drops by ~0.5 s."""
    est = SizeRangeEstimator(FX, W)
    _feed_closing(est, r0=12.0, vc=4.0, t1=1.0)
    tau_a = est.tau_s
    true_a = (12.0 - 4.0 * 1.0) / 4.0                 # 2.0 s
    _feed_closing(est, r0=12.0, vc=4.0, t0=1.0 + 1.0 / 30.0, t1=1.5)
    tau_b = est.tau_s
    true_b = (12.0 - 4.0 * 1.5) / 4.0                 # 1.5 s
    assert tau_a is not None and tau_b is not None
    assert abs(tau_a - true_a) < 0.5, (tau_a, true_a)
    assert abs(tau_b - true_b) < 0.4, (tau_b, true_b)
    assert 0.2 < (tau_a - tau_b) < 0.8                # counted down ~0.5 s


def test_tau_none_when_opening():
    """A shrinking box (opening geometry) has no time-to-contact."""
    est = SizeRangeEstimator(FX, W)
    dt = 1.0 / 30.0
    for i in range(30):
        t = i * dt
        est.predict(dt)
        r = 8.0 + 3.0 * t                              # opening at 3 m/s
        est.correct(FX * W / r, t)
    assert est.tau_s is None
    assert est.rdot_m_s > 0.0                          # and reads as opening


def test_fusion_weight_bounds():
    assert fusion_weight(0.5, 2.0) == 0.0625           # (0.5/2)^2
    assert fusion_weight(2.0, 0.5) == 1.0              # capped at full gain
    assert fusion_weight(1.0, 1.0) == 1.0


def test_fuse_is_coarse_prior_not_primary():
    """fuse_into folds the size range into the CALLER'S filter at reduced
    gain: the state moves toward the prior, but by LESS than the same
    correction at full gain -- and the filter's stored gains are untouched
    (compose, don't replace)."""
    est = SizeRangeEstimator(FX, W)
    _feed_closing(est, r0=12.0, vc=4.0, t1=1.0)        # prior ~8 m, sigma ~16%
    primary = AlphaBetaFilter(0.5, 0.15)
    primary.correct(10.0, 0.0)
    primary.correct(10.0, 1.0)                         # primary track at 10 m
    full = AlphaBetaFilter(0.5, 0.15)
    full.correct(10.0, 0.0)
    full.correct(10.0, 1.0)
    x_before = primary.x_hat
    w = est.fuse_into(primary, 1.05)
    full.correct(est.range_m, 1.05)                    # same meas, full gain
    assert 0.0 < w < 1.0, w
    d_fused = primary.x_hat - x_before
    d_full = full.x_hat - x_before
    assert d_fused != 0.0 and math.copysign(1, d_fused) == math.copysign(1, d_full)
    assert abs(d_fused) < abs(d_full)                  # coarse prior, weaker pull
    assert primary.alpha == 0.5 and primary.beta == 0.15   # gains untouched


def test_fuse_same_tick_guard_protects_rate():
    """SAFE-BY-CONSTRUCTION (review finding 1): folding the coarse prior at
    the SAME sim time as a fresh camera correction would collapse dt_since to
    the α-β 1e-3 floor and inject a huge Rdot into a DEFAULT fixed-gain range
    filter (which does NOT self-protect like Kalata mode). fuse_into must
    SKIP that fold (min_dt_s guard) -- the primary's rate is untouched -- and
    still fuse a legitimate gap-fill on a later tick."""
    est = SizeRangeEstimator(FX, W)
    _feed_closing(est, r0=12.0, vc=4.0, t1=1.0)
    primary = AlphaBetaFilter(0.5, 0.15)       # fixed-gain, as in the adopted config
    primary.correct(10.0, 0.9)
    primary.correct(10.0, 1.0)                 # camera just corrected at t=1.0
    xdot_before = primary.xdot_hat
    # same-tick fold: MUST be skipped (would be ~150*w*residual m/s otherwise).
    assert est.fuse_into(primary, 1.0) == 0.0
    assert primary.xdot_hat == xdot_before     # rate channel untouched
    # a fold within the guard window is also skipped...
    assert est.fuse_into(primary, 1.0 + 0.01) == 0.0
    # ...but a genuine gap-fill one control tick later DOES fuse, and the
    # resulting Rdot stays physically sane (not blown up by a tiny dt).
    w = est.fuse_into(primary, 1.0 + 0.05)
    assert w > 0.0
    assert abs(primary.xdot_hat) < 5.0, primary.xdot_hat   # m/s, sane closing


def test_fuse_skips_uninitialized_primary():
    """A coarse prior must not DEFINE the track (update_cue precedent): with
    the primary uninitialized, fuse_into is a no-op unless allow_init=True
    is deliberately passed."""
    est = SizeRangeEstimator(FX, W)
    _feed_closing(est, r0=12.0, vc=4.0, t1=1.0)
    primary = AlphaBetaFilter(0.5, 0.15)
    assert est.fuse_into(primary, 2.0) == 0.0
    assert not primary.initialized
    assert est.fuse_into(primary, 2.0, allow_init=True) == 1.0
    assert primary.initialized and abs(primary.x_hat - est.range_m) < 1e-9


def test_fuse_noop_before_first_detection():
    """No widths seen -> no prior -> fuse_into applies nothing."""
    est = SizeRangeEstimator(FX, W)
    primary = AlphaBetaFilter(0.5, 0.15)
    primary.correct(10.0, 0.0)
    x = primary.x_hat
    assert est.fuse_into(primary, 1.0) == 0.0
    assert primary.x_hat == x


def test_predict_keeps_width_physical():
    """A long coast on a shrinking-width state clamps at the floor instead
    of crossing zero (range stays finite and positive)."""
    est = SizeRangeEstimator(FX, W)
    est.correct(10.0, 0.0)
    est.correct(7.0, 0.5)                              # wdot < 0
    for _ in range(200):
        est.predict(0.1)                               # 20 s coast
    assert est.width_px >= 1e-3
    assert est.range_m > 0.0


ALL = [test_size_range_recovers_known_r, test_degenerate_width_is_none,
       test_sigma_model_honest, test_estimator_recovers_range_and_closing_rate,
       test_tau_counts_down, test_tau_none_when_opening,
       test_fusion_weight_bounds, test_fuse_is_coarse_prior_not_primary,
       test_fuse_same_tick_guard_protects_rate,
       test_fuse_skips_uninitialized_primary, test_fuse_noop_before_first_detection,
       test_predict_keeps_width_physical]

if __name__ == "__main__":
    import sys
    failed = 0
    for t in ALL:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
    print(f"{len(ALL) - failed}/{len(ALL)} range_fusion tests passed")
    sys.exit(1 if failed else 0)
