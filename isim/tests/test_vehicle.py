"""Tests for isim.vehicle.QuadVelocityModel: hover holding, step tracking,
saturation limits, the tilt/thrust-saturation -> altitude coupling, yaw wrap,
command latency, determinism, wind tracking, and quaternion consistency."""
import math
import time

import numpy as np
import pytest

from isim.types import VehicleState, VelCmd
from isim.vehicle import QuadVelocityModel, VehicleParams


def _hover_state(pos=(0.0, 0.0, 0.0)) -> VehicleState:
    return VehicleState(
        t=0.0,
        pos_ned=np.array(pos, dtype=float),
        vel_ned=np.zeros(3),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        yaw_rad=0.0,
    )


def _run(model: QuadVelocityModel, cmd: VelCmd, n_steps: int, dt: float, seed: int = 0):
    """Reset from hover at the origin and run n_steps of a constant command;
    return the list of VehicleState after each step."""
    rng = np.random.default_rng(seed)
    model.reset(_hover_state(), rng)
    states = []
    for _ in range(n_steps):
        states.append(model.step(cmd, dt))
    return states


HOVER_CMD = VelCmd(v_north=0.0, v_east=0.0, v_down=0.0, yaw_deg=0.0)


def test_hover_holds_position():
    """No command deviation, zero wind: the vehicle should not drift."""
    model = QuadVelocityModel()
    states = _run(model, HOVER_CMD, n_steps=1000, dt=0.005)  # 5 s
    final = states[-1]
    assert np.linalg.norm(final.pos_ned) < 0.01, "hover drifted > 1 cm in 5 s"
    assert np.linalg.norm(final.vel_ned) < 0.01
    assert all(not s.saturated for s in states), "hover should never saturate"


def test_step_to_5ms_reaches_target_with_bounded_overshoot():
    """A modest step should converge close to the setpoint without wild
    overshoot -- a basic sanity check on the velocity-loop tuning."""
    model = QuadVelocityModel()
    cmd = VelCmd(v_north=5.0, v_east=0.0, v_down=0.0, yaw_deg=0.0)
    states = _run(model, cmd, n_steps=1600, dt=0.005)  # 8 s
    vn = np.array([s.vel_ned[0] for s in states])

    # reaches 95% of the commanded velocity within a plausible time
    idx_95 = np.argmax(vn >= 0.95 * 5.0)
    assert vn[idx_95] >= 0.95 * 5.0, "never reached 95% of the commanded step"
    t_95 = idx_95 * 0.005
    assert t_95 < 4.0, f"took {t_95:.2f}s to reach 95% of a 5 m/s step -- too slow"

    # bounded overshoot: should not exceed the setpoint by more than 20%
    assert vn.max() <= 5.0 * 1.2, f"overshoot too large: peak {vn.max():.2f} m/s"
    # and it should have settled near the setpoint by the end
    assert abs(vn[-1] - 5.0) < 0.5


def test_max_tilt_never_exceeded():
    """Even a very aggressive step must respect the tilt ceiling."""
    model = QuadVelocityModel()
    cmd = VelCmd(v_north=15.0, v_east=15.0, v_down=0.0, yaw_deg=0.0)
    states = _run(model, cmd, n_steps=800, dt=0.005)  # 4 s
    max_tilt_seen = 0.0
    for s in states:
        # tilt = angle between body -z axis (rotated by quat) and vertical;
        # decoupled from yaw (see fit_vehicle._tilt_deg_from_quat).
        _, qx, qy, _ = s.quat_wxyz
        cos_tilt = max(-1.0, min(1.0, 1.0 - 2.0 * (qx**2 + qy**2)))
        max_tilt_seen = max(max_tilt_seen, math.degrees(math.acos(cos_tilt)))
    assert max_tilt_seen <= QuadVelocityModel().p.max_tilt_deg + 0.5


def test_saturated_flag_false_in_hover_true_in_hard_step():
    model = QuadVelocityModel()
    hover_states = _run(model, HOVER_CMD, n_steps=200, dt=0.005)
    assert not any(s.saturated for s in hover_states)

    model2 = QuadVelocityModel()
    hard_cmd = VelCmd(v_north=12.0, v_east=0.0, v_down=0.0, yaw_deg=0.0)
    hard_states = _run(model2, hard_cmd, n_steps=200, dt=0.005)
    assert any(s.saturated for s in hard_states), "0->12 m/s step should saturate something"


def test_hard_step_couples_into_altitude_via_thrust_limit():
    """The core modelling requirement: a tight thrust ceiling forces a
    measurable altitude excursion during a hard horizontal accel; a
    generous ceiling leaves altitude almost untouched. This is what lets a
    fit against real logs (which show a ~0.35 m climb) identify thrust_max
    and the tilt/thrust time constants."""
    cmd = VelCmd(v_north=12.0, v_east=0.0, v_down=0.0, yaw_deg=0.0)

    tight = QuadVelocityModel(VehicleParams(thrust_max=10.5))
    tight_states = _run(tight, cmd, n_steps=600, dt=0.005)  # 3 s
    tight_excursion = max(abs(s.pos_ned[2]) for s in tight_states)

    generous = QuadVelocityModel(VehicleParams(thrust_max=19.6))
    generous_states = _run(generous, cmd, n_steps=600, dt=0.005)
    generous_excursion = max(abs(s.pos_ned[2]) for s in generous_states)

    assert tight_excursion > 0.5, "tight thrust ceiling should sag/climb noticeably"
    assert generous_excursion < 0.2, "generous thrust ceiling should barely move altitude"
    assert tight_excursion > 3.0 * generous_excursion


def test_yaw_rate_limit_respected_through_wrap():
    """A near-180-degree yaw command must ramp at <= the rate limit even
    when it has to wrap through +-180."""
    model = QuadVelocityModel()
    rng = np.random.default_rng(0)
    start = VehicleState(
        t=0.0, pos_ned=np.zeros(3), vel_ned=np.zeros(3),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0), yaw_rad=math.radians(170.0),
    )
    model.reset(start, rng)
    cmd = VelCmd(v_north=0.0, v_east=0.0, v_down=0.0, yaw_deg=-170.0)  # 20 deg the short way
    dt = 0.005
    max_rate = model.p.yaw_rate_limit_deg_s
    prev_yaw = math.degrees(start.yaw_rad)
    for _ in range(400):
        s = model.step(cmd, dt)
        yaw = math.degrees(s.yaw_rad)
        dyaw = (yaw - prev_yaw + 180.0) % 360.0 - 180.0
        assert abs(dyaw) <= max_rate * dt + 1e-6, "yaw rate limit violated across the wrap"
        prev_yaw = yaw
    # should have made real progress towards -170 deg (the short way, 20 deg of travel)
    assert abs(((yaw - (-170.0) + 180.0) % 360.0) - 180.0) < 5.0


def test_command_latency_delays_response_by_expected_steps():
    """Warm up on hover first (so the delay line is full of hover commands,
    as a real command stream would be), then switch to a step command and
    compare against a zero-latency model: the delayed trace should be the
    no-delay trace shifted by exactly n_delay = round(latency_s/dt) steps."""
    dt = 0.01
    latency_s = 0.05
    n_delay_steps = round(latency_s / dt)
    step_cmd = VelCmd(v_north=5.0, v_east=0.0, v_down=0.0, yaw_deg=0.0)

    def run_with_warmup(latency_s):
        model = QuadVelocityModel(VehicleParams(latency_s=latency_s))
        rng = np.random.default_rng(0)
        model.reset(_hover_state(), rng)
        for _ in range(20):
            model.step(HOVER_CMD, dt)
        return [model.step(step_cmd, dt).vel_ned[0] for _ in range(20)]

    vn_delay = np.array(run_with_warmup(latency_s))
    vn_nodelay = np.array(run_with_warmup(0.0))

    assert np.all(vn_delay[:n_delay_steps] < 1e-9), "should still be at hover during the delay"
    assert vn_nodelay[0] > 1e-9, "no-delay model should respond on the very next step"
    # shifting the no-delay trace by n_delay_steps should reproduce the
    # delayed trace almost exactly (both replay the identical dynamics)
    assert np.allclose(
        vn_delay[n_delay_steps:], vn_nodelay[: len(vn_nodelay) - n_delay_steps], atol=1e-9
    )


def test_determinism_same_seed_same_trajectory():
    p = VehicleParams(gust_std=0.5)  # exercise the RNG path
    cmd = VelCmd(v_north=6.0, v_east=2.0, v_down=-1.0, yaw_deg=30.0)

    m1 = QuadVelocityModel(p)
    m2 = QuadVelocityModel(p)
    states1 = _run(m1, cmd, n_steps=300, dt=0.005, seed=123)
    states2 = _run(m2, cmd, n_steps=300, dt=0.005, seed=123)

    for s1, s2 in zip(states1, states2):
        assert np.allclose(s1.pos_ned, s2.pos_ned)
        assert np.allclose(s1.vel_ned, s2.vel_ned)
        assert s1.quat_wxyz == s2.quat_wxyz


def test_tracks_ground_velocity_under_constant_wind_with_small_ss_error():
    """The velocity loop commands GROUND velocity (own-state EKF style), so
    under a steady headwind it should still converge close to the commanded
    ground speed thanks to the integral term, not drift off by the wind
    speed."""
    wind = (-3.0, 0.0, 0.0)  # 3 m/s wind blowing south (opposing a north cmd)
    model = QuadVelocityModel(VehicleParams(wind_ned=wind))
    cmd = VelCmd(v_north=5.0, v_east=0.0, v_down=0.0, yaw_deg=0.0)
    states = _run(model, cmd, n_steps=2000, dt=0.005)  # 10 s to settle
    final_vn = states[-1].vel_ned[0]
    assert abs(final_vn - 5.0) < 0.3, f"steady-state ground-speed error too large: {final_vn}"


def test_quaternion_unit_norm_and_matches_thrust_direction():
    model = QuadVelocityModel()
    cmd = VelCmd(v_north=6.0, v_east=-4.0, v_down=0.0, yaw_deg=45.0)
    states = _run(model, cmd, n_steps=300, dt=0.005)
    for s in states:
        w, x, y, z = s.quat_wxyz
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        assert abs(norm - 1.0) < 1e-6

    # body -z axis rotated by the final quaternion should match the model's
    # internal thrust direction (third rotation-matrix column, negated).
    w, x, y, z = states[-1].quat_wxyz
    body_minus_z_world = np.array(
        [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)]
    ) * -1.0
    # this should equal the model's tilt-derived thrust direction to within
    # numerical tolerance; recompute it independently from the stored tilt.
    from isim.vehicle import _tilt_to_dir

    expected_dir = _tilt_to_dir(model._tilt)
    assert np.allclose(body_minus_z_world, expected_dir, atol=1e-6)


def test_step_time_budget_microseconds():
    """Report measured per-step cost; not a hard perf gate (CI hardware
    varies) but flags a gross regression, and the number is printed for the
    task report."""
    model = QuadVelocityModel()
    rng = np.random.default_rng(0)
    model.reset(_hover_state(), rng)
    cmd = VelCmd(v_north=3.0, v_east=1.0, v_down=0.0, yaw_deg=10.0)
    n = 20000
    t0 = time.perf_counter()
    for _ in range(n):
        model.step(cmd, 0.005)
    elapsed = time.perf_counter() - t0
    us_per_step = 1e6 * elapsed / n
    print(f"\nmeasured QuadVelocityModel.step: {us_per_step:.2f} us/step")
    assert us_per_step < 300.0, f"step() far slower than expected: {us_per_step:.1f} us"
