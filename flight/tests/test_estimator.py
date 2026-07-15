"""Pin flight.estimator (alpha-beta tracker + Kalata gains). Behavioral
assertions on the g-h math -- extracted verbatim from m4_intercept.py, so these
green means the mirror matches. Run standalone (exit 0/1) or under pytest.
"""

import math

from flight.estimator import AlphaBetaFilter, kalata_alpha_beta


def test_first_correct_initializes():
    """First measurement sets x_hat=meas, xdot=0 (spec)."""
    f = AlphaBetaFilter(0.5, 0.5)
    assert not f.initialized
    f.correct(5.0, 0.0)
    assert f.initialized and f.x_hat == 5.0 and f.xdot_hat == 0.0


def test_constant_velocity_predict():
    """Two corrects establish a rate; predict() coasts x_hat on it.
    alpha=beta=0.5: correct(0@0)->init; correct(1@1)->x=0.5,xd=0.5;
    predict(1)->x=1.0."""
    f = AlphaBetaFilter(0.5, 0.5)
    f.correct(0.0, 0.0)
    f.correct(1.0, 1.0)
    assert abs(f.x_hat - 0.5) < 1e-12 and abs(f.xdot_hat - 0.5) < 1e-12
    f.predict(1.0)
    assert abs(f.x_hat - 1.0) < 1e-12


def test_angular_wraps_residual():
    """angular=True wraps the residual: from x=3.0, a meas of -3.0 is the SHORT
    way toward +pi (residual +0.283), so x_hat moves UP toward pi -- not the
    non-angular filter's plunge toward 0 on a raw -6 residual."""
    fa = AlphaBetaFilter(0.5, 0.5, angular=True)
    fa.correct(3.0, 0.0)
    fa.correct(-3.0, 1.0)
    fn = AlphaBetaFilter(0.5, 0.5, angular=False)
    fn.correct(3.0, 0.0)
    fn.correct(-3.0, 1.0)
    assert fa.x_hat > 3.0            # wrapped: toward +pi
    assert abs(fa.x_hat - math.pi) < 0.02
    assert fn.x_hat < 1.0            # unwrapped: plunged toward 0
    assert fa.x_hat != fn.x_hat


def test_rate_cap_clamps_xdot():
    """rate_cap bounds xdot_hat (caps both the read rate and predict's own
    forward integration)."""
    f = AlphaBetaFilter(0.5, 0.9, rate_cap=0.1)
    f.correct(0.0, 0.0)
    f.correct(10.0, 1.0)            # would drive a huge xdot without the cap
    assert abs(f.xdot_hat) <= 0.1 + 1e-12


def test_kalata_gain_grows_with_gap():
    """A longer sample gap -> larger tracking index -> larger alpha (trust the
    next real measurement more after a dropout). Gains stay in [0, 0.999]."""
    a_short, b_short = kalata_alpha_beta(150.0, 6.0, 0.05)
    a_long, b_long = kalata_alpha_beta(150.0, 6.0, 0.2)
    assert a_long > a_short
    for a in (a_short, a_long):
        assert 0.0 <= a <= 0.999


def test_kalata_mode_recomputes():
    """A Kalata-mode filter ignores fixed gains and adapts per-interval."""
    f = AlphaBetaFilter(0.01, 0.01, kalata_sigma_process=150.0, kalata_sigma_meas=6.0)
    f.correct(0.0, 0.0)
    f.correct(1.0, 0.1)
    # kalata alpha for dt=0.1 is far larger than the nominal 0.01 -> x_hat moved
    # much more than 0.01 toward the measurement.
    assert f.x_hat > 0.1


ALL = [test_first_correct_initializes, test_constant_velocity_predict,
       test_angular_wraps_residual, test_rate_cap_clamps_xdot,
       test_kalata_gain_grows_with_gap, test_kalata_mode_recomputes]

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
    print(f"{len(ALL) - failed}/{len(ALL)} estimator tests passed")
    sys.exit(1 if failed else 0)
