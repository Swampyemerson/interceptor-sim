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
    spec's own bound. v4's Phase-A search (#2/#3) is disabled here -- this
    test is about the underlying rendezvous law converging, not the search
    sweep that deliberately moves off the aim point once arrived-with-no-
    decode (that behaviour has its own test, below)."""
    pos0 = np.array([20.0, 0.0, -10.0])
    vel = np.array([6.0, 0.0, 0.0])
    cfg = PursuitConfig(vsearch_amplitude_m=0.0, yaw_search_amplitude_deg=0.0)
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


def test_phase_a_search_sweeps_vertically_and_in_yaw_once_arrived_with_no_decode():
    """v4 #2/#3: once Phase A has arrived at its aim point with no decode
    for vsearch_delay_s, it must stop sitting still -- own altitude command
    and yaw command must both develop real variation (not converge to a
    single fixed value) over the following sweep periods."""
    pos0 = np.array([10.0, 0.0, -10.0])
    vel = np.array([3.0, 0.0, 0.0])
    cfg = PursuitConfig(vsearch_arrival_range_m=2.0, vsearch_delay_s=1.0,
                        vsearch_amplitude_m=2.5, vsearch_period_s=4.0,
                        yaw_search_amplitude_deg=60.0, yaw_search_period_s=3.5)
    guidance = PursuitRendezvousGuidance(cfg, belief_pos0_ned=pos0, belief_vel_ned=vel,
                                         go_at_s=0.0, initial_yaw_deg=0.0)
    vehicle = FirstOrderVehicle(tau_s=0.2, vmax=20.0)
    target = ConstantVelocityTarget(pos0, vel)
    seeker = _NeverDecodesSeeker()
    init = _init_state([0.0, 0.0, -10.0])

    ecfg = EngagementConfig(dt=0.01, guidance_dt=0.02, max_t=20.0, stop_not_before_s=1e9, seed=0)
    result = run_engagement(ecfg, vehicle, target, seeker, guidance, init, record_trace=True)

    # Only look at the tail (well past the arrival + delay), where the
    # sweep should be in full swing.
    t = result.trace["t"]
    tail = t > 12.0
    down = result.trace["own_pos"][:, 2][tail]
    yaw_deg = np.degrees(result.trace["yaw"][tail])
    print(f"phase-A-search tail: down range=[{down.min():.2f},{down.max():.2f}], "
         f"yaw range=[{yaw_deg.min():.1f},{yaw_deg.max():.1f}]")
    assert (down.max() - down.min()) > 1.0     # real vertical excursion, not a point
    assert (yaw_deg.max() - yaw_deg.min()) > 10.0   # real yaw excursion


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


# --------------------------------------------------------- v2: honesty (no ref)

def _walk_no_target_or_trace_ref(obj, target, forbidden_types, seen=None, depth=0):
    """Recursively walk `obj`'s attributes (bounded depth, cycle-safe via
    `id()`) asserting no attribute IS the `target` object and no attribute is
    an instance of any type in `forbidden_types` -- pursuit_concept_v2.md #5:
    "add a test that guidance has no reference to the trace or target"."""
    if seen is None:
        seen = set()
    if depth > 4 or id(obj) in seen:
        return
    seen.add(id(obj))
    assert obj is not target
    assert not isinstance(obj, forbidden_types)
    if isinstance(obj, dict):
        for v in obj.values():
            _walk_no_target_or_trace_ref(v, target, forbidden_types, seen, depth + 1)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            _walk_no_target_or_trace_ref(v, target, forbidden_types, seen, depth + 1)
    elif hasattr(obj, "__dict__"):
        for v in vars(obj).values():
            _walk_no_target_or_trace_ref(v, target, forbidden_types, seen, depth + 1)


def test_guidance_holds_no_reference_to_target_or_trace(vp):
    """Structural honesty check, complementing the type-spy test: after a
    full closed-loop run (including a B->A fallback, which is where a truth
    leak would be easiest to introduce by accident), no attribute anywhere
    on the guidance object is the `target` object itself or a `TargetState`
    -- and it never held a trace/`EngagementResult` reference either."""
    from isim.engine import EngagementResult
    scn = Scenario(concept="pursuit", target_speed_ms=9.0, cross_range_m=6.5,
                   lead_dist_m=16.2, cam_fx_px=933.0, tag_facing="camera",
                   cam_tilt_up_deg=12.0, seed=0)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    run_engagement(ecfg, vehicle, target, seeker, guidance, init, record_trace=True)

    _walk_no_target_or_trace_ref(guidance, target, (TargetState, FrameReport, EngagementResult))


# --------------------------------------------------------------- v2: bug fixes

def test_start_phase_b_seeds_velocity_from_belief_not_finite_difference():
    """v2 Bug A fix: two decode positions a tiny (0.01 s) time apart, several
    metres apart in position (as noisy-but-plausible position measurements
    at real acquisition ranges can be) -- a 2-point finite difference would
    give a velocity error of hundreds of m/s; seeding from the Phase-A
    belief instead must give exactly the belief velocity."""
    belief_vel = np.array([5.0, 0.0, 0.0])
    cfg = PursuitConfig()
    guidance = PursuitRendezvousGuidance(
        cfg, belief_pos0_ned=np.array([10.0, 0.0, -5.0]), belief_vel_ned=belief_vel,
        go_at_s=0.0, initial_yaw_deg=0.0)
    guidance._phase = "A"
    guidance._decode_positions = [
        (1.00, np.array([10.0, 0.0, -5.0])),
        (1.01, np.array([9.0, 3.0, -5.0])),   # a wild 300 m/s-equivalent jump if diffed
    ]
    guidance._start_phase_b(1.01)
    assert guidance._phase == "B"
    np.testing.assert_allclose(guidance._kf.vel, belief_vel, atol=1e-9)


def test_fallback_velocity_is_speed_capped():
    """v2 Bug B fix: a KF velocity of 200 m/s (the kind of runaway the task's
    final report traces) must be clamped, relative to the ORIGINAL believed
    target speed, before `_sane_fallback_vel` hands it back as a usable
    belief velocity."""
    belief_vel = np.array([9.0, 0.0, 0.0])   # believed speed 9 m/s
    cfg = PursuitConfig(fallback_speed_cap_mult=2.0, fallback_speed_floor_ms=3.0)
    guidance = PursuitRendezvousGuidance(
        cfg, belief_pos0_ned=np.zeros(3), belief_vel_ned=belief_vel,
        go_at_s=0.0, initial_yaw_deg=0.0)
    runaway = np.array([200.0, -50.0, 30.0])
    safe = guidance._sane_fallback_vel(runaway)
    cap = 9.0 * cfg.fallback_speed_cap_mult
    assert float(np.linalg.norm(safe)) == pytest.approx(cap, rel=1e-9)
    # direction is preserved -- only the magnitude is capped.
    np.testing.assert_allclose(safe / np.linalg.norm(safe), runaway / np.linalg.norm(runaway))

    # And the floor applies when the believed target speed is ~0 (the
    # target_speed_ms=0 test case): cap must not collapse to 0.
    cfg0 = PursuitConfig(fallback_speed_cap_mult=2.0, fallback_speed_floor_ms=3.0)
    g0 = PursuitRendezvousGuidance(cfg0, belief_pos0_ned=np.zeros(3),
                                   belief_vel_ned=np.zeros(3), go_at_s=0.0,
                                   initial_yaw_deg=0.0)
    safe0 = g0._sane_fallback_vel(np.array([50.0, 0.0, 0.0]))
    assert float(np.linalg.norm(safe0)) == pytest.approx(cfg0.fallback_speed_floor_ms, rel=1e-9)


def test_no_fallback_inside_no_fallback_range_m():
    """v2 #4: a long dropout (well past `fallback_s`) at an estimated range
    INSIDE `no_fallback_range_m` must NOT revert Phase B to Phase A -- the
    tag legitimately fills/leaves the frame at close range; that is not a
    real track loss."""
    cfg = PursuitConfig(fallback_s=3.0, no_fallback_range_m=3.0)
    guidance = PursuitRendezvousGuidance(
        cfg, belief_pos0_ned=np.array([5.0, 0.0, -5.0]), belief_vel_ned=np.array([1.0, 0.0, 0.0]),
        go_at_s=0.0, initial_yaw_deg=0.0)
    own = _init_state([4.0, 0.0, -5.0])   # 1 m from the KF's eventual estimate
    guidance._phase = "A"
    guidance._decode_positions = [(0.0, np.array([5.0, 0.0, -5.0])),
                                  (0.05, np.array([5.0, 0.0, -5.0]))]
    guidance._start_phase_b(0.05)
    assert guidance._phase == "B"
    guidance._kf.x[3:6] = 0.0   # zero the KF's velocity: no drift during the dropout

    t = 0.05
    for _ in range(400):   # 8 s of dropout (>> fallback_s), no fresh detections
        t += 0.02
        guidance.step(t, own, None)
    assert guidance._phase == "B", "fell back to Phase A despite range < no_fallback_range_m"


def test_closing_speed_schedule_matches_clamp_formula():
    """v2 #2: v_close = clamp(k_close*range_est, v_close_min_ms, v_close_max_ms),
    checked at a range where the schedule is BELOW the ceiling (so the
    along-LOS speed component is exactly k_close*range) and at a range where
    it is clamped to the ceiling."""
    # cam_frame_range_m=0.0 -- disables v6 #A's camera-frame steering, so
    # this test stays purely about the NED-frame law's closing schedule.
    cfg = PursuitConfig(k_close=0.6, v_close_min_ms=1.5, v_close_max_ms=6.0, kp_lat=0.0,
                        cam_frame_range_m=0.0)
    guidance = PursuitRendezvousGuidance(
        cfg, belief_pos0_ned=np.zeros(3), belief_vel_ned=np.array([9.0, 0.0, 0.0]),
        go_at_s=0.0, initial_yaw_deg=0.0)
    guidance._phase = "B"
    guidance._kf.x = np.array([5.0, 0.0, 0.0, 9.0, 0.0, 0.0])   # range 5 m, vel (9,0,0)
    own = VehicleState(t=0.0, pos_ned=np.zeros(3), vel_ned=np.array([9.0, 0.0, 0.0]),
                      quat_wxyz=yaw_to_quat_wxyz(0.0), yaw_rad=0.0)
    cmd_v, _yaw = guidance._phase_b_cmd(0.02, own, np.zeros(3), None)
    # kp_lat=0 and own_vel==kf_vel (v_rel~0, degenerate) -> ref_dir falls back
    # to unit(kf_vel)==los_dir here (target due north), so r_perp==0 exactly;
    # the along-LOS speed is v_target_est[0] + v_close.
    expected_v_close = np.clip(cfg.k_close * 5.0, cfg.v_close_min_ms, cfg.v_close_max_ms)
    assert expected_v_close == pytest.approx(3.0)   # 0.6*5 = 3.0, inside [1.5, 6.0]
    assert cmd_v[0] == pytest.approx(9.0 + expected_v_close, abs=1e-6)

    # Far range: schedule clamps to v_close_max_ms.
    guidance._kf.x = np.array([50.0, 0.0, 0.0, 9.0, 0.0, 0.0])
    cmd_v_far, _ = guidance._phase_b_cmd(0.02, own, np.zeros(3), None)
    assert cmd_v_far[0] == pytest.approx(9.0 + cfg.v_close_max_ms, abs=1e-6)


# ----------------------------------------------------------- v3: capture-time fix

def test_interp_own_state_linear_between_two_samples():
    from isim.concepts import _interp_own_state
    from collections import deque
    hist = deque([
        (1.00, np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), (1.0, 0.0, 0.0, 0.0)),
        (1.10, np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), (1.0, 0.0, 0.0, 0.0)),
    ])
    t_q, pos, vel, quat = _interp_own_state(hist, 1.05)
    assert t_q == pytest.approx(1.05)
    np.testing.assert_allclose(pos, [0.5, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(vel, [1.0, 0.0, 0.0], atol=1e-9)
    # Clamped to the ends, never extrapolated.
    _, pos_before, _, _ = _interp_own_state(hist, 0.5)
    np.testing.assert_allclose(pos_before, [0.0, 0.0, 0.0])
    _, pos_after, _, _ = _interp_own_state(hist, 5.0)
    np.testing.assert_allclose(pos_after, [1.0, 0.0, 0.0])


def test_measurement_uses_own_state_at_capture_not_at_arrival():
    """v3 #1: a Detection with `t_capture` well before the tick it is HANDED
    to `step()` must be converted to NED using the own position/attitude AT
    `t_capture` (from the ring buffer), not the `own` argument's (arrival-
    time) state -- reproduced directly: the vehicle moves 5 m north between
    capture and arrival; using the arrival-time position would put the
    measured target 5 m further north than using the (correct) capture-time
    position."""
    cfg = PursuitConfig()
    guidance = PursuitRendezvousGuidance(
        cfg, belief_pos0_ned=np.zeros(3), belief_vel_ned=np.array([1.0, 0.0, 0.0]),
        go_at_s=0.0, initial_yaw_deg=0.0)

    # Own flies north at 50 m/s (exaggerated on purpose -- makes the effect
    # large and unambiguous) for 0.1 s between two step() calls; a Detection
    # captured at the EARLIER tick arrives (is handed to step()) at the LATER
    # one, with a bearing/elevation of dead-ahead (0, 0) and range 10 m.
    quat0 = yaw_to_quat_wxyz(0.0)
    own_t0 = VehicleState(t=0.0, pos_ned=np.array([0.0, 0.0, -5.0]),
                         vel_ned=np.array([50.0, 0.0, 0.0]), quat_wxyz=quat0, yaw_rad=0.0)
    guidance.step(0.0, own_t0, None)   # records history sample at t=0

    own_t1 = VehicleState(t=0.1, pos_ned=np.array([5.0, 0.0, -5.0]),
                         vel_ned=np.array([50.0, 0.0, 0.0]), quat_wxyz=quat0, yaw_rad=0.0)
    det = Detection(t_capture=0.0, t_available=0.1, u_px=640.0, v_px=400.0, side_px=20.0,
                   range_m=10.0, bearing_deg=0.0, elevation_deg=0.0)
    guidance.step(0.1, own_t1, det)   # hands the STALE-by-0.1s detection here

    # dead-ahead, yaw=0 -> dir_ned = (1,0,0); z = pos_at_capture + 10*(1,0,0).
    # Correct (capture-time pos [0,0,-5]): z = (10, 0, -5).
    # Wrong (arrival-time pos [5,0,-5]):    z = (15, 0, -5).
    got = guidance._decode_positions[-1][1]
    np.testing.assert_allclose(got, [10.0, 0.0, -5.0], atol=1e-6)
    assert not np.allclose(got, [15.0, 0.0, -5.0], atol=1e-6)


# ------------------------------------------------------- v4: dual tag seeker

def test_dual_tag_seeker_prefers_first_and_falls_back_to_second():
    from isim.concepts import DualTagSeeker

    class _FixedSeeker:
        def __init__(self, det, rep):
            self._det, self._rep = det, rep

        def reset(self, rng):
            pass

        def observe(self, t, own, tgt):
            return self._det, self._rep

    class _NoneSeeker:
        def reset(self, rng):
            pass

        def observe(self, t, own, tgt):
            return None, None

    det_a = Detection(t_capture=0.0, t_available=0.0, u_px=1, v_px=1, side_px=20.0,
                     range_m=5.0, bearing_deg=0.0, elevation_deg=0.0)
    rep_a = FrameReport(t_capture=0.0, in_fov=True, side_px=20.0, incidence_deg=0.0,
                        blur_px=0.0, p_decode=1.0, decoded=True)
    det_b = Detection(t_capture=0.0, t_available=0.0, u_px=2, v_px=2, side_px=5.0,
                     range_m=1.0, bearing_deg=1.0, elevation_deg=1.0)
    rep_b = FrameReport(t_capture=0.0, in_fov=True, side_px=5.0, incidence_deg=0.0,
                        blur_px=0.0, p_decode=1.0, decoded=True)

    # Both decode -> first wins.
    dual = DualTagSeeker(_FixedSeeker(det_a, rep_a), _FixedSeeker(det_b, rep_b))
    dual.reset(np.random.default_rng(0))
    det, rep = dual.observe(0.0, None, None)
    assert det is det_a and rep is rep_a

    # First silent -> falls back to second.
    dual2 = DualTagSeeker(_NoneSeeker(), _FixedSeeker(det_b, rep_b))
    dual2.reset(np.random.default_rng(0))
    det2, rep2 = dual2.observe(0.0, None, None)
    assert det2 is det_b and rep2 is rep_b

    # Neither -> (None, None).
    dual3 = DualTagSeeker(_NoneSeeker(), _NoneSeeker())
    dual3.reset(np.random.default_rng(0))
    assert dual3.observe(0.0, None, None) == (None, None)


def test_scenario_inner_tag_and_second_tag_facing_are_mutually_exclusive(vp):
    with pytest.raises(ValueError):
        build(Scenario(concept="pursuit", inner_tag_side_m=0.08, second_tag_facing="side"), vp)


def test_scenario_inner_tag_side_m_builds_a_dual_tag_seeker(vp):
    from isim.concepts import DualTagSeeker
    scn = Scenario(concept="pursuit", inner_tag_side_m=0.08, seed=0)
    _, _, _, seeker, _, _ = build(scn, vp)
    assert isinstance(seeker, DualTagSeeker)
    assert seeker.second.tag.side_m == pytest.approx(0.08)
    assert seeker.first.tag.side_m == pytest.approx(scn.tag_side_m)


def test_scenario_second_tag_facing_builds_a_dual_tag_seeker(vp):
    from isim.concepts import DualTagSeeker
    scn = Scenario(concept="pursuit", tag_facing="rear", second_tag_facing="side", seed=0)
    _, _, _, seeker, _, _ = build(scn, vp)
    assert isinstance(seeker, DualTagSeeker)
    assert seeker.first.tag.side_m == pytest.approx(seeker.second.tag.side_m)


def test_pursuit_overrides_reach_the_config(vp):
    scn = Scenario(concept="pursuit", pursuit_overrides={"d_behind_m": 12.0}, seed=0)
    _, _, _, _, guidance, _ = build(scn, vp)
    assert guidance.cfg.d_behind_m == pytest.approx(12.0)


# ------------------------------------------------------------- v6: #A camera-frame

def test_dir_body_boresight_is_independent_of_attitude():
    """v6 #A's whole premise: `_dir_body_boresight` takes no quaternion at
    all -- it is a pure function of bearing/elevation/mount-tilt."""
    cfg = PursuitConfig()
    g = PursuitRendezvousGuidance(cfg, belief_pos0_ned=np.zeros(3),
                                  belief_vel_ned=np.array([1.0, 0.0, 0.0]),
                                  go_at_s=0.0, initial_yaw_deg=0.0, cam_mount_tilt_up_deg=5.0)
    det = Detection(t_capture=0.0, t_available=0.0, u_px=1, v_px=1, side_px=20.0,
                   range_m=3.0, bearing_deg=10.0, elevation_deg=-5.0)
    dir1 = g._dir_body_boresight(det)
    dir2 = g._dir_body_boresight(det)   # nothing attitude-related to vary it by
    np.testing.assert_array_equal(dir1, dir2)
    # Sanity: bearing>0 (right) -> body Y (right) component > 0.
    assert dir1[1] > 0.0


def test_camera_frame_command_shift_under_attitude_error_does_not_grow_with_range():
    """v6 #A's claim, checked directly on the camera-frame law alone: the
    SAME detection (bearing=elevation=0, i.e. dead-on) and the SAME
    attitude error, at two DIFFERENT ranges (both inside cam_frame_range_m)
    -- the resulting NED command shift must be driven by the command's own
    magnitude (bounded, ~sin(attitude error) x |cmd|), NOT by range, unlike
    the old absolute-frame law where a fixed attitude error's induced
    POSITION error is range x bias (grows linearly with range)."""
    def _cmd(range_m, yaw_err_deg):
        cfg = PursuitConfig(cam_frame_range_m=100.0, kp_cam=1.5)
        g = PursuitRendezvousGuidance(cfg, belief_pos0_ned=np.zeros(3),
                                      belief_vel_ned=np.array([9.0, 0.0, 0.0]),
                                      go_at_s=0.0, initial_yaw_deg=0.0)
        g._phase = "B"
        g._kf.x = np.array([range_m, 0.0, 0.0, 9.0, 0.0, 0.0])   # dead ahead, north
        yaw = math.radians(yaw_err_deg)
        quat = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        own = VehicleState(t=0.0, pos_ned=np.zeros(3), vel_ned=np.array([9.0, 0.0, 0.0]),
                          quat_wxyz=quat, yaw_rad=yaw)
        det = Detection(t_capture=0.0, t_available=0.0, u_px=1, v_px=1, side_px=20.0,
                       range_m=range_m, bearing_deg=0.0, elevation_deg=0.0)
        cmd_v, _yaw = g._phase_b_cmd(0.02, own, np.zeros(3), det)
        return cmd_v

    shift_close = float(np.linalg.norm(_cmd(1.0, 10.0) - _cmd(1.0, 0.0)))
    shift_far = float(np.linalg.norm(_cmd(4.0, 10.0) - _cmd(4.0, 0.0)))
    print(f"10deg yaw error, camera-frame law: cmd shift at range=1m -> {shift_close:.3f} m/s, "
         f"at range=4m -> {shift_far:.3f} m/s (4x range)")
    # NOT proportional to the 4x range increase (would be ~4x if it were
    # inheriting the old law's range x bias behaviour); allow some growth
    # from the feed-forward/closing-speed terms but nowhere near 4x.
    assert shift_far < 2.0 * shift_close


# --------------------------------------------------------------- v6: #B2 bias state

def test_kf_bias_state_does_not_diverge_and_residual_shrinks():
    """v6 #B2's own spec text warns t_bias "may be weakly observable --
    report honestly" -- and an isolated KF-only test (target at exactly
    constant velocity, `own` never appearing in the measurement equation
    at all, unlike the real simulation) is close to the WORST case for
    observability: `h(x) = pos - vel*(age+t_bias)` lets a `pos` shift of
    `+vel*t_bias` mimic `t_bias_hat=0` for any `age_s`. This test only pins
    that the 7-state filter stays NUMERICALLY WELL-BEHAVED under a real
    (if partly degenerate) bias -- it does not assert `t_bias` converges
    to the true value (the honest full-simulation measurement, where own
    genuinely moves/rotates and can break this degeneracy, is in the
    task's final report)."""
    from isim.concepts import _ConstVelKF
    true_vel = np.array([9.0, 0.0, 0.0])
    true_pos0 = np.array([10.0, 0.0, -5.0])
    true_bias_s = 0.030
    kf = _ConstVelKF(q_accel_ms2=0.5, estimate_ts_bias=True)
    kf.init(pos=true_pos0, vel=true_vel, p0_pos=3.0, p0_vel=5.0, p0_bias_s=0.05)
    pos_true_now = true_pos0.copy()
    rng = np.random.default_rng(0)
    residuals = []
    for _ in range(200):
        kf.predict(0.02)
        pos_true_now = pos_true_now + true_vel * 0.02
        believed_age_s = float(rng.uniform(0.01, 0.08))
        true_age_s = believed_age_s + true_bias_s
        z = pos_true_now - true_vel * true_age_s + rng.normal(0.0, 0.02, size=3)
        h = kf.pos - kf.vel * (believed_age_s + kf.bias_s)
        residuals.append(float(np.linalg.norm(z - h)))
        kf.update(z, np.eye(3) * (0.02 ** 2), age_s=believed_age_s)
    print(f"final bias_hat={kf.bias_s * 1000:.1f} ms (true {true_bias_s * 1000:.0f} ms), "
         f"vel_err={float(np.linalg.norm(kf.vel - true_vel)):.3f} m/s, "
         f"early residual={np.mean(residuals[:10]):.3f} m, late residual={np.mean(residuals[-10:]):.3f} m")
    assert np.all(np.isfinite(kf.x)) and np.all(np.isfinite(kf.P))
    assert abs(kf.bias_s) < 1.0   # stays in a physically sane range, doesn't blow up
    assert np.mean(residuals[-10:]) <= np.mean(residuals[:10]) + 1e-6   # fit doesn't worsen


def test_kf_without_bias_state_is_unaffected_shape_wise():
    from isim.concepts import _ConstVelKF
    kf = _ConstVelKF(q_accel_ms2=1.0)   # estimate_ts_bias=False, the default
    assert kf.n == 6
    assert kf.bias_s == pytest.approx(0.0)
    kf.init(pos=np.zeros(3), vel=np.zeros(3), p0_pos=1.0, p0_vel=1.0)
    assert kf.x.shape == (6,)


# ------------------------------------------------------------------- v6: #D dual rear

def test_rear_dual35_builds_two_tags_angled_from_pure_rear(vp):
    from isim.concepts import DualTagSeeker
    scn = Scenario(concept="pursuit", tag_facing="rear_dual35", target_speed_ms=9.0,
                   cross_range_m=6.5, lead_dist_m=16.2, seed=0)
    _, _, target, seeker, _, _ = build(scn, vp)
    assert isinstance(seeker, DualTagSeeker)
    tgt = target.state(20.0)
    n_a = np.asarray(seeker.first.tag.normal_ned)
    n_b = np.asarray(seeker.second.tag.normal_ned)
    pure_rear = -tgt.vel_ned / np.linalg.norm(tgt.vel_ned)
    ang_a = math.degrees(math.acos(np.clip(np.dot(n_a, pure_rear), -1.0, 1.0)))
    ang_b = math.degrees(math.acos(np.clip(np.dot(n_b, pure_rear), -1.0, 1.0)))
    assert ang_a == pytest.approx(35.0, abs=1e-6)
    assert ang_b == pytest.approx(35.0, abs=1e-6)
    # opposite sides
    assert np.dot(n_a, n_b) < np.dot(n_a, pure_rear)


def test_rear_dual35_mutually_exclusive_with_other_dual_tag_options(vp):
    with pytest.raises(ValueError):
        build(Scenario(concept="pursuit", tag_facing="rear_dual35", inner_tag_side_m=0.08), vp)


# --------------------------------------------------------------- v7: hybrid

def test_hybrid_phase_s_flies_the_open_loop_sprint_speed_schedule():
    """Phase S's forward speed must match `dash_forward_speed` exactly (the
    SAME accel-limited ramp the flyby's coded dash uses), along the given
    heading, with no lateral/vertical correction while the background
    filter has never acquired (no detections at all in this test)."""
    from isim.concepts import HybridGuidance, HybridSprintConfig
    from flight.guidance import dash_forward_speed

    scfg = HybridSprintConfig()
    pcfg = PursuitConfig()
    g = HybridGuidance(scfg, pcfg, heading_deg=30.0, dash_speed_ms=16.0, dash_accel_ms2=10.0,
                       believed_alt_m=7.0, go_at_s=0.0,
                       belief_pos0_ned=np.array([50.0, 0.0, -7.0]),
                       belief_vel_ned=np.array([9.0, 0.0, 0.0]))
    own = VehicleState(t=0.0, pos_ned=np.array([0.0, 0.0, -7.0]), vel_ned=np.zeros(3),
                      quat_wxyz=yaw_to_quat_wxyz(0.0), yaw_rad=0.0)
    for i, t in enumerate((0.0, 0.5, 1.0, 1.5)):
        cmd = g.step(t, own, None)
        expected_speed = dash_forward_speed(16.0, 10.0, t)   # go_at_s=0 -> dash starts at t
        expected_vn = expected_speed * math.cos(math.radians(30.0))
        expected_ve = expected_speed * math.sin(math.radians(30.0))
        assert cmd.v_north == pytest.approx(expected_vn, abs=1e-6)
        assert cmd.v_east == pytest.approx(expected_ve, abs=1e-6)
        assert cmd.yaw_deg == pytest.approx(30.0, abs=1e-9)
    assert all(s in ("STANDBY", "SPRINT") for _, s in g.state_log)


def test_hybrid_turns_around_on_sprint_timer_expiry_when_never_acquired():
    from isim.concepts import HybridGuidance, HybridSprintConfig
    scfg = HybridSprintConfig(sprint_max_s=2.0)
    pcfg = PursuitConfig()
    g = HybridGuidance(scfg, pcfg, heading_deg=0.0, dash_speed_ms=16.0, dash_accel_ms2=10.0,
                       believed_alt_m=7.0, go_at_s=0.0,
                       belief_pos0_ned=np.array([50.0, 0.0, -7.0]),
                       belief_vel_ned=np.array([9.0, 0.0, 0.0]))
    own = VehicleState(t=0.0, pos_ned=np.array([0.0, 0.0, -7.0]), vel_ned=np.array([16.0, 0.0, 0.0]),
                      quat_wxyz=yaw_to_quat_wxyz(0.0), yaw_rad=0.0)
    t = 0.0
    for _ in range(160):   # 3.2 s at dt=0.02, past sprint_max_s=2.0
        g.step(t, own, None)
        t += 0.02
    assert g.turn_reason == "timer"
    assert g.t_turn_start is not None and g.t_turn_start == pytest.approx(2.02, abs=0.03)
    assert g.range_est_at_turn is None   # never acquired -- honestly None, not a guess
    assert any(s not in ("STANDBY", "SPRINT") for _, s in g.state_log)   # handed to pursuit


def test_hybrid_handover_seeds_pursuit_with_the_real_current_velocity_and_yaw():
    """The 'brake': at the Phase S->T handover, `self._pursuit`'s own
    slew-limiter must start from the REAL current (fast sprint) velocity/
    yaw, not from zero/its own reset() default."""
    from isim.concepts import HybridGuidance, HybridSprintConfig
    scfg = HybridSprintConfig(sprint_max_s=1.0)
    pcfg = PursuitConfig()
    g = HybridGuidance(scfg, pcfg, heading_deg=0.0, dash_speed_ms=16.0, dash_accel_ms2=10.0,
                       believed_alt_m=7.0, go_at_s=0.0,
                       belief_pos0_ned=np.array([50.0, 0.0, -7.0]),
                       belief_vel_ned=np.array([9.0, 0.0, 0.0]))
    fast_vel = np.array([15.5, 1.2, -0.3])
    own = VehicleState(t=0.0, pos_ned=np.array([0.0, 0.0, -7.0]), vel_ned=fast_vel,
                      quat_wxyz=yaw_to_quat_wxyz(0.3), yaw_rad=0.3)
    t = 0.0
    handover_cmd = None
    dt = 0.02
    for _ in range(60):   # 1.2 s, past sprint_max_s=1.0
        was_turned = g.t_turn_start is not None
        cmd = g.step(t, own, None)
        if handover_cmd is None and not was_turned and g.t_turn_start is not None:
            handover_cmd = cmd   # the FIRST command issued on the turn tick itself
        t += dt
    assert handover_cmd is not None, "never turned -- test setup problem"
    # Pursuit's own Phase-B slew limiter (accel_max_horiz_b_ms2/vert) bounds
    # how far ONE tick can move away from the seeded `fast_vel` -- a real
    # "brake" (bounded deceleration), not a jump to/from zero.
    cmd_v = np.array([handover_cmd.v_north, handover_cmd.v_east, handover_cmd.v_down])
    one_tick_bound = max(PursuitConfig().accel_max_horiz_b_ms2,
                         PursuitConfig().accel_max_vert_b_ms2) * dt * 3.0   # generous margin
    shift = float(np.linalg.norm(cmd_v - fast_vel))
    print(f"handover tick command shift from fast_vel: {shift:.3f} m/s (bound {one_tick_bound:.3f})")
    assert shift < one_tick_bound
    assert shift < float(np.linalg.norm(fast_vel)) * 0.5   # nowhere near "jumped to/from zero"


def test_hybrid_never_receives_target_state_or_frame_report(vp):
    """Honesty spy, mirroring the pursuit/flyby versions."""
    from isim.concepts import HybridGuidance

    class _Spy:
        def __init__(self, inner):
            self.inner = inner
            self.seen_own_types = []
            self.seen_det_types = []

        def reset(self):
            self.inner.reset()

        def step(self, t, own, det):
            self.seen_own_types.append(type(own))
            self.seen_det_types.append(type(det))
            assert isinstance(own, VehicleState) and not isinstance(own, TargetState)
            assert det is None or isinstance(det, Detection)
            assert not isinstance(det, FrameReport)
            return self.inner.step(t, own, det)

    scn = Scenario(concept="hybrid", target_speed_ms=9.0, cross_range_m=6.5, lead_dist_m=16.2,
                   cam_fx_px=385.0, tag_facing="rear", cam_tilt_up_deg=12.0, seed=0)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    assert isinstance(guidance, HybridGuidance)
    spy = _Spy(guidance)
    run_engagement(ecfg, vehicle, target, seeker, spy, init)
    assert len(spy.seen_own_types) > 0
    assert set(spy.seen_own_types) == {VehicleState}
    assert set(spy.seen_det_types) <= {Detection, type(None)}


def test_hybrid_scoring_window_matches_pursuit(vp):
    scn_h = Scenario(concept="hybrid", seed=0)
    scn_p = Scenario(concept="pursuit", seed=0)
    ecfg_h, *_ = build(scn_h, vp)
    ecfg_p, *_ = build(scn_p, vp)
    assert ecfg_h.max_t == pytest.approx(ecfg_p.max_t)
    assert ecfg_h.stop_after_cpa_s == pytest.approx(ecfg_p.stop_after_cpa_s)


def test_hybrid_overrides_reach_the_sprint_config(vp):
    scn = Scenario(concept="hybrid", hybrid_overrides={"sprint_max_s": 3.5}, seed=0)
    _, _, _, _, guidance, _ = build(scn, vp)
    assert guidance.scfg.sprint_max_s == pytest.approx(3.5)
