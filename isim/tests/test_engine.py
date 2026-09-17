import math
from typing import Optional

import numpy as np
import pytest

from isim.engine import EngagementConfig, run_engagement
from isim.stubs import FirstOrderVehicle, PerfectSeeker, PursuitGuidance, yaw_to_quat_wxyz
from isim.targets import ConstantVelocityTarget, HoverTarget
from isim.types import Detection, TargetState, VehicleState, VelCmd


# ---------------------------------------------------------------------------
# Minimal fixtures used only by these tests.
# ---------------------------------------------------------------------------


class LinearVehicle:
    """Ignores every command and flies in a straight line at a fixed
    velocity -- lets a test know the exact analytic trajectory."""

    def __init__(self, vel_ned: np.ndarray) -> None:
        self._vel = np.asarray(vel_ned, dtype=np.float64)

    def reset(self, state: VehicleState, rng: np.random.Generator) -> None:
        self._pos = np.asarray(state.pos_ned, dtype=np.float64).copy()
        self._t = float(state.t)

    def step(self, cmd: VelCmd, dt: float) -> VehicleState:
        self._pos = self._pos + self._vel * dt
        self._t += dt
        return VehicleState(
            t=self._t,
            pos_ned=self._pos.copy(),
            vel_ned=self._vel.copy(),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            yaw_rad=0.0,
        )


class NullSeeker:
    def reset(self, rng: np.random.Generator) -> None:
        pass

    def observe(self, t, own, tgt):
        return None, None


class NullGuidance:
    def reset(self) -> None:
        pass

    def step(self, t, own, det) -> VelCmd:
        return VelCmd(0.0, 0.0, 0.0, 0.0)


class NoisyVehicle:
    """Like FirstOrderVehicle but perturbs velocity with its own rng each
    step, to exercise seeding/determinism end to end."""

    def __init__(self, tau_s: float = 0.4, vmax: float = 15.0, noise_std: float = 0.05) -> None:
        self.tau_s = tau_s
        self.vmax = vmax
        self.noise_std = noise_std

    def reset(self, state: VehicleState, rng: np.random.Generator) -> None:
        self._rng = rng
        self._pos = np.asarray(state.pos_ned, dtype=np.float64).copy()
        self._vel = np.asarray(state.vel_ned, dtype=np.float64).copy()
        self._yaw = float(state.yaw_rad)
        self._t = float(state.t)

    def step(self, cmd: VelCmd, dt: float) -> VehicleState:
        v_cmd = np.array([cmd.v_north, cmd.v_east, cmd.v_down])
        speed = float(np.linalg.norm(v_cmd))
        saturated = speed > self.vmax
        if saturated and speed > 1e-12:
            v_cmd = v_cmd * (self.vmax / speed)
        noise = self._rng.normal(scale=self.noise_std, size=3)
        self._vel = self._vel + (v_cmd - self._vel) * (dt / self.tau_s) + noise * dt
        self._pos = self._pos + self._vel * dt
        self._yaw = math.radians(cmd.yaw_deg)
        self._t += dt
        return VehicleState(
            t=self._t,
            pos_ned=self._pos.copy(),
            vel_ned=self._vel.copy(),
            quat_wxyz=yaw_to_quat_wxyz(self._yaw),
            yaw_rad=self._yaw,
            saturated=saturated,
        )


class SpyGuidance:
    """Records the type of every argument it is handed."""

    def __init__(self) -> None:
        self.own_types = []
        self.det_types = []

    def reset(self) -> None:
        pass

    def step(self, t, own, det) -> VelCmd:
        self.own_types.append(type(own))
        self.det_types.append(type(det))
        return VelCmd(0.0, 0.0, 0.0, 0.0)


def _init_state(pos, yaw=0.0):
    return VehicleState(
        t=0.0,
        pos_ned=np.asarray(pos, dtype=np.float64),
        vel_ned=np.zeros(3),
        quat_wxyz=yaw_to_quat_wxyz(yaw),
        yaw_rad=yaw,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_determinism_same_seed_bit_identical():
    def make():
        cfg = EngagementConfig(max_t=6.0, seed=42)
        vehicle = NoisyVehicle()
        target = ConstantVelocityTarget(np.array([80.0, 5.0, 0.0]), np.array([0.0, 3.0, 0.0]))
        seeker = PerfectSeeker()
        guidance = PursuitGuidance(speed=10.0)
        init = _init_state([0.0, 0.0, 0.0])
        return cfg, vehicle, target, seeker, guidance, init

    r1 = run_engagement(*make())
    r2 = run_engagement(*make())

    assert r1.miss_m == r2.miss_m
    assert r1.t_cpa == r2.t_cpa
    np.testing.assert_array_equal(r1.miss_vec_ned, r2.miss_vec_ned)
    assert r1.n_frames == r2.n_frames
    assert r1.n_decoded == r2.n_decoded

    # a different seed is *allowed* to differ (not required to) -- just must not crash
    cfg3 = EngagementConfig(max_t=6.0, seed=43)
    r3 = run_engagement(
        cfg3,
        NoisyVehicle(),
        ConstantVelocityTarget(np.array([80.0, 5.0, 0.0]), np.array([0.0, 3.0, 0.0])),
        PerfectSeeker(),
        PursuitGuidance(speed=10.0),
        _init_state([0.0, 0.0, 0.0]),
    )
    # with real noise in the loop, different seeds should (almost surely) diverge
    assert r3.miss_m != r1.miss_m


def test_cpa_interpolation_analytic_flyby():
    # Vehicle flies straight north at 10 m/s from the origin; target hovers
    # at (50, 5, 0). Closest approach analytically at t*=5s, miss=5m.
    own_vel = np.array([10.0, 0.0, 0.0])
    tgt_pos0 = np.array([50.0, 5.0, 0.0])

    cfg = EngagementConfig(dt=0.005, guidance_dt=0.02, max_t=10.0, stop_after_cpa_s=2.0, seed=0)
    vehicle = LinearVehicle(own_vel)
    target = HoverTarget(tgt_pos0)
    seeker = NullSeeker()
    guidance = NullGuidance()
    init = _init_state([0.0, 0.0, 0.0])

    result = run_engagement(cfg, vehicle, target, seeker, guidance, init, record_trace=False)

    assert result.miss_m == pytest.approx(5.0, abs=1e-6)
    assert result.t_cpa == pytest.approx(5.0, abs=1e-6)
    assert result.miss_horiz_m == pytest.approx(5.0, abs=1e-6)
    assert result.miss_vert_m == pytest.approx(0.0, abs=1e-6)


def test_hover_target_pursuit_close_intercept():
    init = _init_state([0.0, 0.0, 0.0], yaw=0.0)
    target = HoverTarget(np.array([60.0, 0.0, 0.0]))
    cfg = EngagementConfig(max_t=15.0, seed=7)
    vehicle = FirstOrderVehicle()
    seeker = PerfectSeeker()
    guidance = PursuitGuidance(speed=12.0)

    result = run_engagement(cfg, vehicle, target, seeker, guidance, init)
    assert result.n_decoded > 0
    assert result.miss_m < 0.2


def test_guidance_never_receives_target_state_or_frame_report():
    init = _init_state([0.0, 0.0, 0.0], yaw=0.0)
    target = ConstantVelocityTarget(np.array([60.0, 10.0, 0.0]), np.array([0.0, 2.0, 0.0]))
    cfg = EngagementConfig(max_t=8.0, seed=3)
    vehicle = FirstOrderVehicle()
    seeker = PerfectSeeker()
    spy = SpyGuidance()

    run_engagement(cfg, vehicle, target, seeker, spy, init)

    assert len(spy.own_types) > 0
    for ot in spy.own_types:
        assert ot is VehicleState
        assert ot is not TargetState
    for dt_ in spy.det_types:
        assert dt_ in (Detection, type(None))


def test_stop_after_cpa_terminates_early():
    # Target flies away fast; engagement should end well before max_t once
    # range has been increasing for stop_after_cpa_s.
    init = _init_state([0.0, 0.0, 0.0], yaw=0.0)
    target = ConstantVelocityTarget(np.array([20.0, 0.0, 0.0]), np.array([50.0, 0.0, 0.0]))
    cfg = EngagementConfig(max_t=30.0, stop_after_cpa_s=1.0, seed=1)
    vehicle = FirstOrderVehicle()
    seeker = PerfectSeeker()
    guidance = PursuitGuidance(speed=5.0)

    result = run_engagement(cfg, vehicle, target, seeker, guidance, init, record_trace=True)

    full_run_steps = round(cfg.max_t / cfg.dt)
    assert len(result.trace["t"]) < full_run_steps
    assert result.trace["t"][-1] < cfg.max_t - 1.0
