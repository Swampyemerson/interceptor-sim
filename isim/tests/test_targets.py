import numpy as np

from isim.targets import ConstantVelocityTarget, HoverTarget, WeaveTarget


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
