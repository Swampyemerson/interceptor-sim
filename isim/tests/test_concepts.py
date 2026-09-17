"""isim/concepts.py tests (isim/specs/pursuit_concept.md). Loose bounds only
-- these pin BEHAVIOUR (converges, stays honest, is deterministic, doesn't
change the pre-existing "flyby" concept), never a tuned miss-distance number.
Measured values are printed, not asserted, per the spec.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from isim.concepts import PursuitConfig, PursuitRendezvousGuidance
from isim.engine import EngagementConfig, run_engagement
from isim.flight_adapter import RealFlightGuidance
from isim.replay_a0 import load_params
from isim.scenario import Scenario, build
from isim.stubs import FirstOrderVehicle, yaw_to_quat_wxyz
from isim.targets import ConstantVelocityTarget
from isim.types import Detection, FrameReport, TargetState, VehicleState, VelCmd


@pytest.fixture(scope="module")
def vp():
    return load_params()


class _NeverDecodesSeeker:
    """A SeekerModel that never produces a Detection -- Phase A never gets a
    chance to acquire, so `PursuitRendezvousGuidance` stays in Phase A for
    the whole run."""

    def reset(self, rng: np.random.Generator) -> None:
        pass

    def observe(self, t, own, tgt):
        return None, None


def _init_state(pos):
    return VehicleState(t=0.0, pos_ned=np.asarray(pos, dtype=np.float64),
                        vel_ned=np.zeros(3), quat_wxyz=yaw_to_quat_wxyz(0.0), yaw_rad=0.0)


# --------------------------------------------------------------------- Phase A

def test_phase_a_alone_converges_to_the_aim_point_and_target_velocity():
    """No tag ever decodes (_NeverDecodesSeeker): Phase A alone must steer
    the vehicle to within 1 m of the moving aim point (d_behind_m behind the
    EXACTLY-KNOWN belief target) and within 1 m/s of its velocity, per the
    spec's own bound."""
    pos0 = np.array([20.0, 0.0, -10.0])
    vel = np.array([6.0, 0.0, 0.0])
    cfg = PursuitConfig()
    guidance = PursuitRendezvousGuidance(cfg, belief_pos0_ned=pos0, belief_vel_ned=vel,
                                         go_at_s=0.0, initial_yaw_deg=0.0)
    vehicle = FirstOrderVehicle(tau_s=0.3, vmax=20.0)
    target = ConstantVelocityTarget(pos0, vel)
    seeker = _NeverDecodesSeeker()
    init = _init_state([0.0, 0.0, -10.0])

    # stop_not_before_s far beyond max_t disables the engine's own past-CPA
    # stop rule, so the run always reaches max_t and the trace's last row is
    # the converged steady state, not an early stop mid-approach.
    max_t = 30.0
    ecfg = EngagementConfig(dt=0.01, guidance_dt=0.02, max_t=max_t,
                            stop_not_before_s=1e9, seed=0)
    result = run_engagement(ecfg, vehicle, target, seeker, guidance, init, record_trace=True)

    t_end = float(result.trace["t"][-1])
    tgt_end = target.state(t_end)
    aim_end = tgt_end.pos_ned - cfg.d_behind_m * (vel / np.linalg.norm(vel))

    own_pos_end = result.trace["own_pos"][-1]
    own_vel_end = result.trace["own_vel"][-1]

    pos_err = float(np.linalg.norm(own_pos_end - aim_end))
    vel_err = float(np.linalg.norm(own_vel_end - vel))
    print(f"phase-A-alone: pos_err={pos_err:.3f} m, vel_err={vel_err:.3f} m/s at t={t_end:.1f}s")
    assert pos_err < 1.0
    assert vel_err < 1.0
    # And it really did stay in Phase A the whole time -- no ENGAGE.
    assert all(s != "ENGAGE" for _, s in guidance.state_log)


# ------------------------------------------------------------------- honesty

class _TypeSpyGuidance:
    """Wraps PursuitRendezvousGuidance and records the exact type of every
    argument the ENGINE hands to step() -- the honesty boundary from the
    caller's side, mirroring test_flight_adapter.py's `_TypeSpyGuidance`."""

    def __init__(self, inner: PursuitRendezvousGuidance) -> None:
        self.inner = inner
        self.seen_own_types = []
        self.seen_det_types = []

    def reset(self) -> None:
        self.inner.reset()

    def step(self, t, own, det) -> VelCmd:
        self.seen_own_types.append(type(own))
        self.seen_det_types.append(type(det))
        assert isinstance(own, VehicleState)
        assert not isinstance(own, TargetState)
        assert det is None or isinstance(det, Detection)
        assert not isinstance(det, FrameReport)
        return self.inner.step(t, own, det)


def test_guidance_never_receives_target_state_or_frame_report(vp):
    scn = Scenario(concept="pursuit", target_speed_ms=9.0, cross_range_m=6.5,
                   lead_dist_m=16.2, cam_fx_px=933.0, tag_facing="rear", seed=1)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    assert isinstance(guidance, PursuitRendezvousGuidance)
    spy = _TypeSpyGuidance(guidance)

    run_engagement(ecfg, vehicle, target, seeker, spy, init)

    assert len(spy.seen_own_types) > 0
    assert set(spy.seen_own_types) == {VehicleState}
    assert set(spy.seen_det_types) <= {Detection, type(None)}


# ------------------------------------------------------------------ determinism

def test_pursuit_determinism_per_seed(vp):
    scn = Scenario(concept="pursuit", target_speed_ms=9.0, cross_range_m=6.5,
                   lead_dist_m=16.2, cam_fx_px=933.0, tag_facing="rear", seed=4)

    def _run():
        ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
        return run_engagement(ecfg, vehicle, target, seeker, guidance, init)

    r1, r2 = _run(), _run()
    assert r1.miss_m == r2.miss_m
    assert r1.t_cpa == r2.t_cpa
    assert r1.n_decoded == r2.n_decoded


def test_pursuit_scattered_determinism_per_seed(vp):
    from isim.scenario import Scatter
    scn = Scenario(concept="pursuit", target_speed_ms=9.0, cross_range_m=6.5,
                   lead_dist_m=16.2, cam_fx_px=933.0, tag_facing="rear",
                   scatter=Scatter(), seed=8)

    def _run():
        ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
        return run_engagement(ecfg, vehicle, target, seeker, guidance, init)

    r1, r2 = _run(), _run()
    assert r1.miss_m == r2.miss_m
    assert r1.n_decoded == r2.n_decoded


# --------------------------------------------------------------- flyby unchanged

def test_concept_flyby_is_the_default_and_matches_explicit_flyby(vp):
    """`concept="flyby"` (and the implicit default) must build the exact
    same RealFlightGuidance-driven engagement as before this field existed
    -- a bit-identical no-op for anyone who never sets `concept`."""
    scn_default = Scenario(target_speed_ms=9.0, cross_range_m=6.5, lead_dist_m=16.2, seed=2)
    scn_explicit = Scenario(target_speed_ms=9.0, cross_range_m=6.5, lead_dist_m=16.2,
                            seed=2, concept="flyby")
    assert scn_default.concept == "flyby"

    _, _, _, _, g_default, init_default = build(scn_default, vp)
    _, _, _, _, g_explicit, init_explicit = build(scn_explicit, vp)

    assert isinstance(g_default, RealFlightGuidance)
    assert isinstance(g_explicit, RealFlightGuidance)
    assert g_default.cfg.preflight_heading_deg == pytest.approx(
        g_explicit.cfg.preflight_heading_deg, abs=1e-12)
    assert g_default.cfg.standby_alt_m == pytest.approx(g_explicit.cfg.standby_alt_m, abs=1e-12)
    assert np.array_equal(init_default.pos_ned, init_explicit.pos_ned)


def test_concept_flyby_ignores_tag_facing_camera_equals_old_faces_camera_true(vp):
    """`tag_facing="camera"` (the default) must reproduce the old
    `faces_camera=True` best-case path exactly: incidence pinned to 0."""
    scn = Scenario(tag_facing="camera")
    _, _, target, seeker, _, _ = build(scn, vp)
    own = _init_state([0.0, 0.0, -7.0])
    tgt = target.state(50.0)
    normal = seeker.tag.normal(tgt, own.pos_ned)
    to_cam = own.pos_ned - tgt.pos_ned
    cos_i = float(np.dot(normal, to_cam)) / max(float(np.linalg.norm(to_cam)), 1e-12)
    assert cos_i == pytest.approx(1.0, abs=1e-9)   # incidence == 0 deg, always


# --------------------------------------------------------------------- closed loop

def test_pursuit_closed_loop_apriltag_seeker_rear_facing_prints_miss(vp):
    """Full closed loop through the real AprilTagSeeker, fx=933, 0.30 m tag,
    tag_facing="rear" (the pursuit concept's own approach-from-behind
    geometry), on the nominal crossing case. Smoke test: prints the miss,
    asserts only that the run completed and produced a finite, non-negative
    number and at least one camera frame -- no tuned bound, per the spec."""
    scn = Scenario(concept="pursuit", target_speed_ms=9.0, cross_range_m=6.5,
                   lead_dist_m=16.2, cam_fx_px=933.0, tag_side_m=0.30,
                   tag_facing="rear", seed=0)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    result = run_engagement(ecfg, vehicle, target, seeker, guidance, init)

    print(f"pursuit/rear closed-loop: miss={result.miss_m:.3f} m at t={result.t_cpa:.2f}s, "
          f"n_frames={result.n_frames}, n_decoded={result.n_decoded}, "
          f"final_state={guidance.last_decision.state if guidance.last_decision else None}")
    assert math.isfinite(result.miss_m)
    assert result.miss_m >= 0.0
    assert result.n_frames > 0
