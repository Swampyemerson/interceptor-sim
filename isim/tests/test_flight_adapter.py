"""isim/flight_adapter.py tests (ADR-0102 isim). Drives the UNMODIFIED
flight/deploy/real_flight.RealFlightSM through the isim engagement loop."""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import pytest

from flight.deploy.real_flight import MissionConfig, RealFlightSM
from flight.guidance import dash_forward_speed

from isim.engine import EngagementConfig, run_engagement
from isim.flight_adapter import RealFlightGuidance, standby_init_state
from isim.stubs import FirstOrderVehicle, PerfectSeeker
from isim.seeker import AprilTagSeeker, CameraParams, TagParams
from isim.targets import ConstantVelocityTarget, HoverTarget
from isim.types import Detection, FrameReport, TargetState, VehicleState, VelCmd


def _base_cfg(**overrides) -> MissionConfig:
    """A MissionConfig tuned to make the state machine transition FAST in a
    short offline test. Every change from the dataclass default is explained
    where it is set (below), never silently."""
    kwargs = dict(
        preflight_heading_deg=0.0,     # dash due north -> v_north/v_east easy to check
        standby_alt_m=5.0,
        standby_settle_s=1.0,          # default 2.0 s; shortened so tests tick fewer steps
        dash_speed_ms=8.0,
        acquire_streak=3,              # default 5; shortened, same reason
    )
    kwargs.update(overrides)
    return MissionConfig(**kwargs)


def _hold_own_state(cfg: MissionConfig) -> VehicleState:
    return standby_init_state(cfg)


# --------------------------------------------------------------------- (f)


def test_holds_real_flight_sm_instance():
    cfg = _base_cfg()
    g = RealFlightGuidance(cfg, go_at_s=2.0)
    assert isinstance(g._sm, RealFlightSM)


# --------------------------------------------------------------------- (a)


def test_dash_speed_ramps_toward_dash_speed_ms():
    """Once CODED_DASH starts, the commanded horizontal velocity follows
    dash_forward_speed(dash_speed_ms, dash_accel_cap_ms2, elapsed) along the
    latched preflight heading -- checked against the SAME function
    flight.guidance exports, at every tick, not just at the end."""
    cfg = _base_cfg(dash_accel_cap_ms2=4.0, dash_speed_ms=10.0,
                    preflight_heading_deg=30.0)
    g = RealFlightGuidance(cfg, go_at_s=1.0)
    own = _hold_own_state(cfg)

    dt = 0.05
    saw_dash = False
    saw_ramped_below_max = False
    for i in range(160):  # 8 s
        t = i * dt
        cmd = g.step(t, own, None)
        if g._sm.state != "CODED_DASH":
            continue
        saw_dash = True
        elapsed = t - g._sm.t_dash_start
        expect_speed = dash_forward_speed(cfg.dash_speed_ms, cfg.dash_accel_cap_ms2, elapsed)
        h = math.radians(cfg.preflight_heading_deg)
        expect_vn, expect_ve = expect_speed * math.cos(h), expect_speed * math.sin(h)
        assert cmd.v_north == pytest.approx(expect_vn, abs=1e-9)
        assert cmd.v_east == pytest.approx(expect_ve, abs=1e-9)
        assert cmd.yaw_deg == pytest.approx(cfg.preflight_heading_deg, abs=1e-9)
        if 0.0 < expect_speed < cfg.dash_speed_ms:
            saw_ramped_below_max = True

    assert saw_dash, "never reached CODED_DASH"
    assert saw_ramped_below_max, "never observed the ramp strictly below dash_speed_ms"


# --------------------------------------------------------------------- (c)


def test_premature_go_aborts_to_safe_not_coded_dash():
    """A GO edge before standby_settle_s fails the arm gate. real_flight's
    actual policy (Sec 4) is an ABORT to SAFE (`arm_gate_failed_at_go`), not a
    silent refusal that leaves the vehicle sitting in STANDBY -- CODED_DASH
    must never be reached either way."""
    cfg = _base_cfg(standby_settle_s=1.0)
    g = RealFlightGuidance(cfg, go_at_s=0.10)   # well before the 1.0 s settle
    own = _hold_own_state(cfg)

    dt = 0.02
    for i in range(100):  # 2 s
        g.step(i * dt, own, None)

    states_seen = [s for _, s in g.state_log]
    assert "CODED_DASH" not in states_seen
    assert g._sm.state == "SAFE"
    assert g.last_decision.safe_reason is not None
    assert g.last_decision.safe_reason.startswith("arm_gate_failed_at_go")


# --------------------------------------------------------------------- (b)


def test_reaches_engage_after_acquire_streak():
    """A stationary in-view target -> CODED_DASH -> (acquire_streak fresh
    detections) -> ENGAGE, driven through the real isim engagement loop with
    PerfectSeeker (deterministic, no decode-probability roll -- explicitly
    sanctioned by the task brief as simpler than AprilTagSeeker for this
    check)."""
    cfg = _base_cfg(dash_max_s=8.0)
    guidance = RealFlightGuidance(cfg, go_at_s=cfg.standby_settle_s)
    vehicle = FirstOrderVehicle(tau_s=0.3, vmax=20.0)
    target = HoverTarget(pos_ned=np.array([30.0, 0.0, -cfg.standby_alt_m]))
    seeker = PerfectSeeker(fps=30.0, latency_s=0.0)
    init = standby_init_state(cfg)

    # stop_after_cpa_s must clear go_at_s: during STANDBY the range is flat (the
    # vehicle is holding, not closing), so a short stop_after_cpa_s reads that
    # flat range as "already past CPA" and ends the run before GO ever fires.
    ecfg = EngagementConfig(dt=0.005, guidance_dt=0.02, max_t=10.0,
                            stop_after_cpa_s=4.0, seed=1)
    run_engagement(ecfg, vehicle, target, seeker, guidance, init)

    states_seen = [s for _, s in guidance.state_log]
    assert "CODED_DASH" in states_seen
    assert "ENGAGE" in states_seen, f"never reached ENGAGE; states={states_seen}, " \
        f"last safe_reason={guidance.last_decision.safe_reason}"


# --------------------------------------------------------------------- (d)


def test_two_engagements_are_bit_identical_for_the_same_seed():
    """reset() must clear EVERYTHING -- the SM latch/streak AND the
    SeekerGuidance alpha-beta filters (which have no reset() of their own) --
    or a second run_engagement() on the same seed would silently start from
    the first run's terminal filter state and diverge."""
    cfg = _base_cfg(dash_max_s=8.0)
    cam = CameraParams()
    guidance = RealFlightGuidance(cfg, cam_params=cam, go_at_s=cfg.standby_settle_s)
    tag = TagParams(faces_camera=True)   # best-case decode -- keep this test robust to the seed

    def _fresh_scenario():
        vehicle = FirstOrderVehicle(tau_s=0.3, vmax=20.0)
        target = ConstantVelocityTarget(pos0_ned=np.array([30.0, 0.0, -cfg.standby_alt_m]),
                                        vel_ned=np.array([0.0, 1.5, 0.0]))
        seeker = AprilTagSeeker(cam=cam, tag=tag)
        init = standby_init_state(cfg)
        return vehicle, target, seeker, init

    ecfg = EngagementConfig(dt=0.005, guidance_dt=0.02, max_t=10.0,
                            stop_after_cpa_s=4.0, seed=7)

    v1, tg1, sk1, i1 = _fresh_scenario()
    r1 = run_engagement(ecfg, v1, tg1, sk1, guidance, i1, record_trace=True)
    log1 = list(guidance.state_log)

    v2, tg2, sk2, i2 = _fresh_scenario()
    r2 = run_engagement(ecfg, v2, tg2, sk2, guidance, i2, record_trace=True)
    log2 = list(guidance.state_log)

    assert log1 == log2
    assert log1, "the scenario never left STANDBY -- not exercising reset()"
    assert r1.miss_m == r2.miss_m
    assert r1.n_decoded == r2.n_decoded
    for key in r1.trace:
        assert np.array_equal(r1.trace[key], r2.trace[key], equal_nan=True), key


# --------------------------------------------------------------------- (e)


class _TypeSpyGuidance:
    """Wraps RealFlightGuidance and records the exact type of every argument
    the ENGINE hands to `step()` -- proves the honesty boundary from the
    caller's side: isim's engine (and this adapter) must never pass a
    TargetState or a FrameReport into guidance."""

    def __init__(self, inner: RealFlightGuidance) -> None:
        self.inner = inner
        self.seen_own_types: List[type] = []
        self.seen_det_types: List[type] = []

    def reset(self) -> None:
        self.inner.reset()

    def step(self, t: float, own, det) -> VelCmd:
        self.seen_own_types.append(type(own))
        self.seen_det_types.append(type(det))
        assert isinstance(own, VehicleState)
        assert not isinstance(own, TargetState)
        assert det is None or isinstance(det, Detection)
        assert not isinstance(det, FrameReport)
        return self.inner.step(t, own, det)


def test_engine_never_hands_guidance_a_target_state_or_frame_report():
    cfg = _base_cfg(dash_max_s=8.0)
    inner = RealFlightGuidance(cfg, go_at_s=cfg.standby_settle_s)
    spy = _TypeSpyGuidance(inner)
    vehicle = FirstOrderVehicle(tau_s=0.3, vmax=20.0)
    target = HoverTarget(pos_ned=np.array([30.0, 0.0, -cfg.standby_alt_m]))
    seeker = PerfectSeeker(fps=30.0, latency_s=0.0)
    init = standby_init_state(cfg)
    ecfg = EngagementConfig(dt=0.005, guidance_dt=0.02, max_t=5.0,
                            stop_after_cpa_s=1.0, seed=1)

    run_engagement(ecfg, vehicle, target, seeker, spy, init)

    assert len(spy.seen_own_types) > 0
    assert set(spy.seen_own_types) == {VehicleState}
    assert set(spy.seen_det_types) <= {Detection, type(None)}
