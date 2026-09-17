import numpy as np

from isim.targets import ConstantVelocityTarget, HoverTarget, SpeedChangeTarget, WeaveTarget


def test_constant_velocity_target():
    pos0 = np.array([1.0, 2.0, -3.0])
    vel = np.array([5.0, -1.0, 0.5])
    tgt = ConstantVelocityTarget(pos0, vel)
    for t in (0.0, 1.0, 3.7):
        st = tgt.state(t)
        np.testing.assert_allclose(st.pos_ned, pos0 + vel * t)
        np.testing.assert_allclose(st.vel_ned, vel)
        assert st.t == t


def test_hover_target_is_stationary():
    pos = np.array([10.0, -4.0, -2.0])
    tgt = HoverTarget(pos)
    for t in (0.0, 5.0, 12.3):
        st = tgt.state(t)
        np.testing.assert_allclose(st.pos_ned, pos)
        np.testing.assert_allclose(st.vel_ned, np.zeros(3))


def test_weave_velocity_matches_numerical_derivative():
    pos0 = np.array([0.0, 0.0, -10.0])
    vel = np.array([8.0, 0.0, 0.0])
    tgt = WeaveTarget(pos0, vel, amp_m=3.0, period_s=4.0)
    h = 1e-6
    for t in (0.3, 1.1, 2.9, 5.5):
        p_plus = tgt.state(t + h).pos_ned
        p_minus = tgt.state(t - h).pos_ned
        numeric_vel = (p_plus - p_minus) / (2 * h)
        analytic_vel = tgt.state(t).vel_ned
        np.testing.assert_allclose(numeric_vel, analytic_vel, atol=1e-4)


def test_weave_perpendicular_to_cruise_velocity():
    # Cruise north; weave should offset purely in east (perpendicular, horizontal).
    pos0 = np.array([0.0, 0.0, 0.0])
    vel = np.array([10.0, 0.0, 0.0])
    tgt = WeaveTarget(pos0, vel, amp_m=2.0, period_s=5.0)
    st = tgt.state(1.25)  # quarter period -> sin(2*pi*0.25)=1 -> max lateral offset
    lateral = st.pos_ned - (pos0 + vel * 1.25)
    assert abs(lateral[0]) < 1e-9  # no residual north component
    assert abs(abs(lateral[1]) - 2.0) < 1e-6


# --------------------------------------------------------- v5: SpeedChangeTarget

def test_speed_change_target_is_straight_before_and_after_the_change():
    pos0 = np.array([0.0, 0.0, -10.0])
    vel = np.array([9.0, 0.0, 0.0])
    tgt = SpeedChangeTarget(pos0, vel, delta_ms=2.0, change_t=5.0)

    before = tgt.state(2.0)
    np.testing.assert_allclose(before.pos_ned, pos0 + vel * 2.0)
    np.testing.assert_allclose(before.vel_ned, vel)

    at = tgt.state(5.0)
    np.testing.assert_allclose(at.vel_ned, vel)   # still the OLD speed exactly at change_t

    after = tgt.state(7.0)
    expected_vel_after = np.array([11.0, 0.0, 0.0])   # 9 + 2 m/s, same direction
    np.testing.assert_allclose(after.vel_ned, expected_vel_after)
    pos_at_change = pos0 + vel * 5.0
    np.testing.assert_allclose(after.pos_ned, pos_at_change + expected_vel_after * 2.0)


def test_speed_change_target_position_is_continuous_at_the_change():
    pos0 = np.array([0.0, 0.0, -10.0])
    vel = np.array([9.0, 0.0, 0.0])
    tgt = SpeedChangeTarget(pos0, vel, delta_ms=-2.0, change_t=5.0)
    just_before = tgt.state(5.0 - 1e-9).pos_ned
    just_after = tgt.state(5.0 + 1e-9).pos_ned
    np.testing.assert_allclose(just_before, just_after, atol=1e-6)
    # ...but velocity has a real step (that IS the point).
    assert not np.allclose(tgt.state(4.999).vel_ned, tgt.state(5.001).vel_ned, atol=1e-3)


def test_speed_change_target_never_goes_negative_speed():
    pos0 = np.array([0.0, 0.0, -10.0])
    vel = np.array([1.0, 0.0, 0.0])   # slow target
    tgt = SpeedChangeTarget(pos0, vel, delta_ms=-5.0, change_t=1.0)   # would go to -4 m/s
    after = tgt.state(3.0)
    assert float(np.linalg.norm(after.vel_ned)) == 0.0
