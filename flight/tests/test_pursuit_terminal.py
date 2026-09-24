"""flight/tests/test_pursuit_terminal.py -- unit tests for
flight.pursuit_terminal, the "chase only" pursuit-rendezvous camera terminal
(ADR-0103; docs/pursuit_port_2026-09-17.md).

Pure unit tests: no sim, no MAVSDK, no weights. `PursuitTerminalGuidance` is a
pure step function exactly like `TagInterceptGuidance`, so it is driven the
same way `test_tag_terminal.py` drives that class.
"""
import math
import os

import numpy as np
import pytest

from flight.camera import CameraModel
from flight.deploy.seeker_loop import GuidanceConfig, OwnState
from flight.pursuit_terminal import PursuitTerminalConfig, PursuitTerminalGuidance

FX = FY = 539.936
CX, CY = 640.0, 480.0
DT = 0.05
IDENT = (1.0, 0.0, 0.0, 0.0)   # level attitude, nose north


def cam():
    return CameraModel(FX, FY, CX, CY)


def own(n_e_d_vel=(0.0, 0.0, 0.0), quat=IDENT, alt_m=5.0, age_s=0.0):
    return OwnState(quat=quat, psi_rad=0.0, alt_m=alt_m, age_s=age_s,
                    vel_ned=n_e_d_vel)


def box_for(n, e, d, side_m=1.0, fx=FX, fy=FY, cx=CX, cy=CY):
    """A detector box that, under IDENTITY attitude, zero mount tilt and zero
    camera offset, reconstructs EXACTLY the NED relative position (n, e, d)
    (same derivation as test_tag_terminal.py's box_for)."""
    slant = math.sqrt(n * n + e * e + d * d)
    xn, yn = e / n, d / n
    u, v = cx + fx * xn, cy + fy * yn
    bw = fx * side_m / slant
    return (u - bw / 2.0, v - bw / 2.0, bw, bw)


def guidance(belief_r0_ned=(20.0, 0.0, 0.0), belief_vel0_ned=(-9.0, 0.0, 0.0),
             go_at_s=0.0, **cfgkw):
    # Zero camera mount offset/tilt so box_for()'s exact-reconstruction
    # derivation holds exactly in these unit tests.
    gcfg = GuidanceConfig(mount_fwd_m=0.0, mount_left_m=0.0, mount_up_m=0.0,
                          mount_up_rad=0.0)
    cfg = PursuitTerminalConfig(**cfgkw)
    g = PursuitTerminalGuidance(cfg, cam(), 1.0, gcfg,
                                belief_r0_ned=np.array(belief_r0_ned),
                                belief_vel0_ned=np.array(belief_vel0_ned),
                                go_at_s=go_at_s)
    return g, cfg, gcfg


# ==================================================================== Phase A


def test_phase_a_flies_toward_a_point_behind_the_moving_belief():
    """Target believed 20 m ahead (north), moving north (receding) at 9 m/s
    -- the aim point trails it by d_behind_m, so Phase A should command a
    northward-closing velocity that (once the slew limiter has ramped up
    over a couple of seconds) approaches the target's own speed plus a
    position-correction term, clipped at v_max_ms."""
    g, cfg, _ = guidance(belief_r0_ned=(20.0, 0.0, 0.0), belief_vel0_ned=(9.0, 0.0, 0.0),
                         go_at_s=0.0, d_behind_m=8.0)
    t = 0.0
    for _ in range(3):
        t += DT
        sp, tel = g.step(None, own(), t=t)
    assert tel.phase == "A"
    assert sp is not None
    # First tick: only accel_max_ms2*dt = 6.0*0.05 = 0.3 m/s of slew is
    # available yet, so direction (not magnitude) is what a single tick can
    # prove; forward progress should still be underway (v_north > 0).
    assert sp.v_north > 0.0
    assert sp.v_east == pytest.approx(0.0, abs=1e-6)

    # After ramping (aim point is (12,0,0) relative -> cmd_v settles at
    # v_track + kp_pos*aim = (9,0,0) + 0.8*(12,0,0) = (18.6,0,0), clipped to
    # v_max_ms=16), the slewed command should approach the ceiling.
    for _ in range(60):
        t += DT
        sp, tel = g.step(None, own(), t=t)
    assert sp.v_north == pytest.approx(cfg.v_max_ms, abs=0.05)


def test_phase_a_yaws_toward_the_believed_target():
    # Target 20 m east, moving east at 5 m/s: the believed target stays due
    # east of own the whole time, so yaw should converge to ~90 deg. (This
    # geometry cannot distinguish target-yaw from aim-point-yaw -- both sit
    # due east; the discriminating case is the station-keeping test below.)
    g, _, _ = guidance(belief_r0_ned=(0.0, 20.0, 0.0), belief_vel0_ned=(0.0, 5.0, 0.0))
    t = 0.0
    sp = None
    for _ in range(30):   # yaw slew is capped at 180 deg/s -> ramps in <1s
        t += DT
        sp, tel = g.step(None, own(), t=t)
    assert tel.phase == "A"
    # target east of own -> yaw ~ 90 deg (atan2(east, north))
    assert sp.yaw_deg == pytest.approx(90.0, abs=1.0)


def test_phase_a_station_keeping_still_points_camera_at_target():
    """REGRESSION PIN (Gazebo cross-check flight 1, 2026-09-23): own is AT the
    rendezvous point, so r_aim ~ 0 -- the old law (yaw toward r_aim) fell back
    to the arrival bearing and FROZE the camera off-target for the whole
    engagement (zero decodes at 12 m dead astern). The prototype (and now the
    port) yaws at the believed TARGET, d_behind_m ahead along the track."""
    d = float(PursuitTerminalConfig().d_behind_m)
    # Believed target d_behind_m EAST, both flying east at 5 m/s, own velocity
    # matching: the KF's relative state stays (0, d, 0) and r_aim stays ~0 --
    # exact station-keeping. own() faces NORTH (psi=0), so the old law's
    # fallback held yaw at ~0 while the target sat due EAST (90 deg).
    g, cfg, _ = guidance(belief_r0_ned=(0.0, d, 0.0),
                         belief_vel0_ned=(0.0, 5.0, 0.0))
    t = 0.0
    sp = None
    for _ in range(30):
        t += DT
        sp, tel = g.step(None, own(n_e_d_vel=(0.0, 5.0, 0.0)), t=t)
    assert tel.phase == "A"
    assert abs(_wrap(sp.yaw_deg - 90.0)) < 15.0


def _wrap(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def test_before_go_at_s_commands_zero_and_reports_standby():
    g, _, _ = guidance(go_at_s=5.0)
    sp, tel = g.step(None, own(), t=1.0)
    assert tel.phase == "STANDBY"
    assert sp.v_north == 0.0 and sp.v_east == 0.0 and sp.v_down == 0.0


# ============================================================ acquire gate


def test_enough_fresh_detections_transition_to_phase_b():
    g, cfg, _ = guidance(belief_r0_ned=(15.0, 0.0, 0.0), belief_vel0_ned=(0.0, 0.0, 0.0),
                         acquire_n=2, acquire_window_s=0.3)
    t = 0.0
    box = box_for(15.0, 0.0, 0.0)
    _, tel1 = g.step(box, own(), t=t)
    assert tel1.phase == "A"
    t += DT
    _, tel2 = g.step(box, own(), t=t)
    assert tel2.phase == "B"


def test_no_detections_stays_in_phase_a():
    g, _, _ = guidance(belief_r0_ned=(15.0, 0.0, 0.0), belief_vel0_ned=(0.0, 0.0, 0.0))
    t = 0.0
    for _ in range(10):
        t += DT
        _, tel = g.step(None, own(), t=t)
    assert tel.phase == "A"


# ================================================================= Phase B


def test_phase_b_closes_range_over_repeated_ticks_on_a_stationary_target():
    g, cfg, _ = guidance(belief_r0_ned=(15.0, 0.0, 0.0), belief_vel0_ned=(0.0, 0.0, 0.0),
                         acquire_n=2, acquire_window_s=0.3, hold_range_m=0.5)
    t = 0.0
    # Acquire.
    box = box_for(15.0, 0.0, 0.0)
    g.step(box, own(), t=t)
    t += DT
    _, tel = g.step(box, own(), t=t)
    assert tel.phase == "B"
    r0 = tel.r_hat_m

    # Fly closer for a couple of seconds, feeding fresh (moving-own-frame)
    # detections consistent with the vehicle's own commanded closing motion
    # -- own stays at rest (own vel 0) but the TARGET is stationary in NED,
    # so as the vehicle would actually move the box should shrink; since we
    # don't integrate a real vehicle here, feed the box as if range is
    # closing at v_close to check the estimator/law drives r_hat_m down when
    # given consistent measurements.
    r_true = 15.0
    for _ in range(40):
        t += DT
        r_true = max(0.5, r_true - cfg.v_close_min_ms * DT)
        box = box_for(r_true, 0.0, 0.0)
        _, tel = g.step(box, own(), t=t)
        assert tel.phase == "B"

    assert tel.r_hat_m < r0
    assert tel.setpoint.v_north > 0.0   # still closing forward


def test_phase_b_holds_inside_hold_range_m_and_reports_terminal_coast():
    g, cfg, _ = guidance(belief_r0_ned=(2.0, 0.0, 0.0), belief_vel0_ned=(0.0, 0.0, 0.0),
                         acquire_n=2, acquire_window_s=0.3, hold_range_m=1.0)
    t = 0.0
    box = box_for(0.5, 0.0, 0.0)   # already inside hold_range_m at acquisition
    g.step(box, own(), t=t)
    t += DT
    sp1, tel1 = g.step(box, own(), t=t)
    assert tel1.phase == "B"
    assert tel1.terminal_coast is True
    prev_cmd = (sp1.v_north, sp1.v_east, sp1.v_down)

    t += DT
    sp2, tel2 = g.step(box, own(), t=t)
    assert tel2.terminal_coast is True
    assert (sp2.v_north, sp2.v_east, sp2.v_down) == pytest.approx(prev_cmd, abs=1e-9)


def test_long_dropout_falls_back_to_phase_a_with_bounded_velocity():
    g, cfg, _ = guidance(belief_r0_ned=(15.0, 0.0, 0.0), belief_vel0_ned=(9.0, 0.0, 0.0),
                         acquire_n=2, acquire_window_s=0.3,
                         fallback_s=1.0, no_fallback_range_m=3.0,
                         fallback_speed_cap_mult=2.0, fallback_speed_floor_ms=3.0)
    t = 0.0
    box = box_for(15.0, 0.0, 0.0)
    g.step(box, own(), t=t)
    t += DT
    _, tel = g.step(box, own(), t=t)
    assert tel.phase == "B"

    # Force a bogus, huge KF velocity to simulate a corrupted acquisition,
    # then let it dropout past fallback_s with the range still far outside
    # no_fallback_range_m.
    g._kf.x[3:6] = np.array([80.0, -30.0, 5.0])
    for _ in range(30):
        t += DT
        _, tel = g.step(None, own(), t=t)
    assert tel.phase == "A"
    cap = max(9.0 * cfg.fallback_speed_cap_mult, cfg.fallback_speed_floor_ms)
    assert np.linalg.norm(g._v_track) <= cap + 1e-6


# =============================================================== own-state


def test_stale_own_state_refuses_to_steer():
    gcfg_kwargs = dict(mount_fwd_m=0.0, mount_left_m=0.0, mount_up_m=0.0,
                       mount_up_rad=0.0, own_state_max_age_s=0.1)
    gcfg = GuidanceConfig(**gcfg_kwargs)
    cfg = PursuitTerminalConfig()
    g = PursuitTerminalGuidance(cfg, cam(), 1.0, gcfg,
                                belief_r0_ned=np.array([15.0, 0.0, 0.0]),
                                belief_vel0_ned=np.array([0.0, 0.0, 0.0]),
                                go_at_s=0.0)
    stale = own(age_s=5.0)
    sp, tel = g.step(None, stale, t=1.0)
    assert sp is None
    assert tel.own_state_ok is False


def test_missing_own_attitude_refuses_to_steer():
    g, _, _ = guidance()
    bad = OwnState(quat=None, psi_rad=None, alt_m=5.0, age_s=0.0, vel_ned=(0.0, 0.0, 0.0))
    sp, tel = g.step(None, bad, t=0.05)
    assert sp is None
    assert tel.own_state_ok is False


# ================================================== adaptive speed governor


def test_adaptive_off_cap_is_the_legacy_v_max():
    """Default config (adaptive_speed=False): the cap is the fixed v_max_ms
    in both phases, whatever the believed target speed -- the governor must
    be inert unless asked for."""
    g, cfg, _ = guidance(belief_vel0_ned=(15.0, 0.0, 0.0))
    assert cfg.adaptive_speed is False
    assert g._speed_cap() == pytest.approx(cfg.v_max_ms)


def test_adaptive_phase_a_cap_is_believed_speed_plus_margin():
    """Adaptive on, target believed receding north at 15 m/s: the Phase-A
    command should ramp to 15 + overtake_margin (20), ABOVE the legacy 16
    cap -- the overtake-margin finding (docs/fast_intercept_limits.md #1)
    made adaptive, end to end through step()."""
    g, cfg, _ = guidance(belief_r0_ned=(20.0, 0.0, 0.0),
                         belief_vel0_ned=(15.0, 0.0, 0.0),
                         adaptive_speed=True, overtake_margin_ms=5.0,
                         v_hw_max_ms=24.0)
    assert g._speed_cap() == pytest.approx(20.0)
    t, sp = 0.0, None
    for _ in range(100):   # accel_max 6 m/s^2 -> ~3.4 s to reach 20 m/s
        t += DT
        sp, tel = g.step(None, own(), t=t)
    assert tel.phase == "A"
    # cmd wants v_track + kp_pos*aim = 15 + 0.8*12 = 24.6 -> clipped to 20.
    assert sp.v_north == pytest.approx(20.0, abs=0.05)


def test_adaptive_cap_respects_floor_and_hardware_ceiling():
    g_slow, cfg, _ = guidance(belief_vel0_ned=(1.0, 0.0, 0.0),
                              adaptive_speed=True)
    assert g_slow._speed_cap() == pytest.approx(cfg.adaptive_v_floor_ms)
    g_fast, cfg2, _ = guidance(belief_vel0_ned=(30.0, 0.0, 0.0),
                               adaptive_speed=True)
    assert g_fast._speed_cap() == pytest.approx(cfg2.v_hw_max_ms)


def test_adaptive_does_not_slow_closure_on_stale_lock():
    """The lock-quality closure modulation (slow down when decodes go
    stale) was built, A/B'd and REJECTED 2026-09-23 (null on aim-error
    cells, 12-14 point contact cost at 18 m/s -- timidity feedback; see
    docs/adaptive_speed_prereg.md amendments 1-2). This pins its ABSENCE:
    2 s into a dropout (short of the 3 s Phase-A fallback), an adaptive
    Phase B must still command full closure toward a stationary target."""
    g, cfg, _ = guidance(belief_r0_ned=(10.0, 0.0, 0.0),
                         belief_vel0_ned=(0.0, 0.0, 0.0),
                         adaptive_speed=True)
    t = 0.0
    for _ in range(22):
        t += DT
        sp, tel = g.step(box_for(10.0, 0.0, 0.0), own(), t=t)
    assert tel.phase == "B"
    fresh_speed = math.sqrt(sp.v_north**2 + sp.v_east**2 + sp.v_down**2)
    assert fresh_speed > 4.0    # closing near v_close_max
    for _ in range(40):         # 2.0 s dropout
        t += DT
        sp, tel = g.step(None, own(), t=t)
    assert tel.phase == "B"     # no Phase-A fallback yet
    stale_speed = math.sqrt(sp.v_north**2 + sp.v_east**2 + sp.v_down**2)
    assert stale_speed > 4.0    # still closing -- no timidity decay


# ============================================ keep-in-frame vertical assist
#
# isim/specs/keepframe_prereg_2026-09-23.md. The assist is config-gated and
# DEFAULT OFF; the byte-identity test below locks "flag off == the code
# before the feature existed" against a fixture generated from the
# PRE-keepframe module (git f123197) by the exact drive sequence
# `_keepframe_identity_drive` replays.

_FIXTURE_PATH = os.path.join(os.path.dirname(__file__),
                             "keepframe_identity_fixture.npz")


def _keepframe_det_for_tick(i):
    """Deterministic detection schedule: nothing, centered closes, TOP-EDGE
    boxes (target ~37 deg above boresight -> box center v ~ 76 px of 960),
    a dropout, then centered again. VERBATIM the fixture generator's."""
    if i < 20:
        return None                                  # Phase A, no camera
    if i < 40:
        rng_n = 12.0 - 0.1 * (i - 20)
        return box_for(rng_n, 0.3, -0.2)             # centered-ish decodes
    if i < 60:
        return box_for(6.0, 0.0, -4.5)               # TOP-EDGE (v ~ 76 px)
    if i < 80:
        return None                                  # dropout
    if i < 120:
        return box_for(5.0, 0.0, 0.0)                # centered
    return None


def _keepframe_identity_drive(cfg_kwargs=None, det_for_tick=_keepframe_det_for_tick):
    """-> (N, 4) float64 array of (v_north, v_east, v_down, yaw_deg); NaN row
    where step() returned None. Must stay VERBATIM in sync with the fixture
    generator that produced keepframe_identity_fixture.npz."""
    gcfg = GuidanceConfig(mount_fwd_m=0.0, mount_left_m=0.0, mount_up_m=0.0,
                          mount_up_rad=0.0)
    cfg = PursuitTerminalConfig(**(cfg_kwargs or {}))
    g = PursuitTerminalGuidance(
        cfg, cam(), 1.0, gcfg,
        belief_r0_ned=np.array([15.0, 0.0, 0.0]),
        belief_vel0_ned=np.array([3.0, 0.0, 0.0]), go_at_s=0.2)
    rows = []
    t = 0.0
    for i in range(140):
        t += DT
        o = OwnState(quat=IDENT, psi_rad=0.0, alt_m=5.0, age_s=0.0,
                     vel_ned=(1.0, 0.0, 0.0))
        sp, _tel = g.step(det_for_tick(i), o, t)
        if sp is None:
            rows.append([math.nan] * 4)
        else:
            rows.append([sp.v_north, sp.v_east, sp.v_down, sp.yaw_deg])
    return np.array(rows, dtype=np.float64)


def _assert_matches_golden(got, golden):
    """Exact equality where the runner's numpy kernels match the fixture's
    provenance; else a 1e-9 absolute band. RATIONALE (2026-09-24, CI runs
    35963711767/35964354924): GitHub-hosted runners span CPU generations and
    numpy selects SIMD kernels at import, so transcendental results can drift
    by ~1 ulp (~1e-16) between runners -- two doc-only pushes flipped these
    "byte-identical" asserts fail/pass with IDENTICAL flight code. 1e-9 is
    ~7 orders above kernel drift and ~6 below any real behavioural change
    (a mutated constant moves commands by >=1e-3 m/s); the failure message
    quantifies the divergence so a real regression is never mistaken for
    kernel noise."""
    got = np.asarray(got, dtype=np.float64)
    if np.array_equal(got, golden, equal_nan=True):
        return
    nan_got, nan_gold = np.isnan(got), np.isnan(golden)
    assert np.array_equal(nan_got, nan_gold), "NaN pattern differs from the fixture"
    diff = np.abs(np.where(nan_got, 0.0, got - np.where(nan_gold, 0.0, golden)))
    assert float(diff.max()) < 1e-9, \
        f"golden mismatch beyond kernel-drift band: max |diff| = {float(diff.max()):.3e}"


def test_keepframe_off_is_byte_identical_to_prefeature_code():
    """REPO INVARIANT: with keepframe_assist=False (the default) the emitted
    commands are BYTE-IDENTICAL to the module as it was before the feature
    existed -- the fixture was generated by running this exact drive (top-edge
    boxes, dropout and all) against the pre-change code."""
    golden = np.load(_FIXTURE_PATH)["cmds"]
    got = _keepframe_identity_drive()
    assert got.shape == golden.shape
    _assert_matches_golden(got, golden)   # exact-or-kernel-drift band
    # And the default really is off.
    assert PursuitTerminalConfig().keepframe_assist is False


def test_keepframe_engages_on_top_edge_detection_and_commands_climb():
    """Flag ON in Phase B: a persistent top-edge target must produce a
    STEADY-STATE command with more climb (more negative NED v_down) than the
    OFF arm -- compared late, after the slew has converged, because during the
    initial ramp both arms ride the same vertical slew limit (the assist
    respects the budget, so it cannot show up there)."""
    def top_edge_long(i):
        return box_for(6.0, 0.0, -4.5) if i >= 20 else None
    off = _keepframe_identity_drive(det_for_tick=top_edge_long)
    on = _keepframe_identity_drive({"keepframe_assist": True},
                                   det_for_tick=top_edge_long)
    window = slice(100, 140)                   # steady state, decodes fresh
    delta = np.nanmean(on[window, 2] - off[window, 2])
    assert delta < -0.5                        # a real added climb component
    cfg = PursuitTerminalConfig()
    assert delta >= -cfg.keepframe_vz_max_ms - 1e-6   # bounded by the clamp
    # Before any detection at all, the arms are identical (no decode -> no
    # assist state, term 0).
    assert np.array_equal(on[:20], off[:20], equal_nan=True)


def test_keepframe_no_assist_on_centered_detection():
    """A centered detection must never engage the assist: with only-centered
    boxes the ON arm is byte-identical to the OFF arm."""
    def centered_only(i):
        return box_for(10.0, 0.0, 0.0) if 20 <= i < 120 else None
    off = _keepframe_identity_drive(det_for_tick=centered_only)
    on = _keepframe_identity_drive({"keepframe_assist": True},
                                   det_for_tick=centered_only)
    assert np.array_equal(on, off, equal_nan=True)


def test_keepframe_assist_active_in_phase_a_acquisition():
    """Phase A (pre-KF) consumes decodes too: with the acquire gate held shut
    (acquire_n huge) a top-edge decode must still climb the Phase-A command."""
    def top_edge(i):
        return box_for(6.0, 0.0, -4.5) if i >= 20 else None
    kw = {"acquire_n": 999, "keepframe_assist": True}
    on = _keepframe_identity_drive(kw, det_for_tick=top_edge)
    off = _keepframe_identity_drive({"acquire_n": 999}, det_for_tick=top_edge)
    window = slice(24, 60)
    assert np.nanmin(on[window, 2] - off[window, 2]) < -0.1
    # Sanity: still Phase A throughout (acquire gate never opened) -- drive a
    # fresh instance to inspect telemetry.
    gcfg = GuidanceConfig(mount_fwd_m=0.0, mount_left_m=0.0, mount_up_m=0.0,
                          mount_up_rad=0.0)
    g = PursuitTerminalGuidance(
        PursuitTerminalConfig(**kw), cam(), 1.0, gcfg,
        belief_r0_ned=np.array([15.0, 0.0, 0.0]),
        belief_vel0_ned=np.array([3.0, 0.0, 0.0]), go_at_s=0.2)
    t = 0.0
    for i in range(60):
        t += DT
        _, tel = g.step(top_edge(i), OwnState(quat=IDENT, psi_rad=0.0, alt_m=5.0,
                                              age_s=0.0, vel_ned=(1.0, 0.0, 0.0)), t)
    assert tel.phase == "A"


def test_keepframe_term_clamped_and_hold_expires():
    """The added term never exceeds keepframe_vz_max_ms (even for a box AT the
    top edge), and after a dropout longer than keepframe_hold_s it drops to
    zero -- no open-loop climb."""
    g, cfg, _ = guidance(belief_r0_ned=(15.0, 0.0, 0.0),
                         belief_vel0_ned=(0.0, 0.0, 0.0),
                         keepframe_assist=True, acquire_n=999)
    t = DT
    # Box hard against the top edge: center v ~ 27 px (deepest engagement).
    box = box_for(4.0, 0.0, -3.35)
    x0, y0, bw, bh = box
    assert (y0 + bh / 2.0) < 30.0
    g.step(box, own(), t=t)
    assert 0.0 < g._keepframe_vz <= cfg.keepframe_vz_max_ms
    assert g._keepframe_term(t) == g._keepframe_vz
    # Deeper than the margin is CLAMPED at the max, never beyond.
    assert g._keepframe_term(t) <= cfg.keepframe_vz_max_ms
    # Hold: still active within keepframe_hold_s of the decode...
    assert g._keepframe_term(t + cfg.keepframe_hold_s) == g._keepframe_vz
    # ...and exactly zero beyond it.
    assert g._keepframe_term(t + cfg.keepframe_hold_s + 0.01) == 0.0


def test_keepframe_respects_vertical_slew_budget():
    """The assist shapes the command BEFORE the slew: per-tick v_down change
    stays within the Phase-A combined budget accel_max_ms2 * dt."""
    on = _keepframe_identity_drive({"keepframe_assist": True, "acquire_n": 999})
    cfg = PursuitTerminalConfig()
    dv = np.diff(on[:, 2])
    dv = dv[np.isfinite(dv)]
    assert np.max(np.abs(dv)) <= cfg.accel_max_ms2 * DT + 1e-9


# ================================================ pose-range channel (S2 fix)
#
# isim/specs/xcheck_tick_trace_2026-09-23.md: the Gazebo cross-check's tick
# trace measured the AABB box-width range channel 15-25% short on a rotated/
# perspective tag while the same detector's PnP pose range was near-clean
# (-0.03 m bias, 0.109 m spread, n=415). step() therefore accepts an OPTIONAL
# det_range_pose_m; absent, the code path is byte-identical to before.


def test_pose_range_absent_is_byte_identical_to_prefeature_code():
    """The keepframe golden fixture was generated from the PRE-pose-range
    module; the default drive (no det_range_pose_m anywhere) must still
    reproduce it exactly -- and an explicit det_range_pose_m=None must be
    byte-identical to not passing the kwarg at all."""
    golden = np.load(_FIXTURE_PATH)["cmds"]
    got = _keepframe_identity_drive()
    _assert_matches_golden(got, golden)

    # Same drive, kwarg explicitly None on every tick.
    gcfg = GuidanceConfig(mount_fwd_m=0.0, mount_left_m=0.0, mount_up_m=0.0,
                          mount_up_rad=0.0)
    g = PursuitTerminalGuidance(
        PursuitTerminalConfig(), cam(), 1.0, gcfg,
        belief_r0_ned=np.array([15.0, 0.0, 0.0]),
        belief_vel0_ned=np.array([3.0, 0.0, 0.0]), go_at_s=0.2)
    rows = []
    t = 0.0
    for i in range(140):
        t += DT
        o = OwnState(quat=IDENT, psi_rad=0.0, alt_m=5.0, age_s=0.0,
                     vel_ned=(1.0, 0.0, 0.0))
        sp, _tel = g.step(_keepframe_det_for_tick(i), o, t, det_range_pose_m=None)
        rows.append([math.nan] * 4 if sp is None
                    else [sp.v_north, sp.v_east, sp.v_down, sp.yaw_deg])
    _assert_matches_golden(np.array(rows), golden)


def test_pose_range_corrects_a_widened_aabb_box():
    """The Gazebo defect in miniature: an off-axis detection whose box width
    is inflated 25% (the AABB of a rotated tag) under-ranges the box channel;
    supplying the TRUE slant as det_range_pose_m must reconstruct the exact
    NED vector (direction still from the box centre)."""
    n, e, d = 6.0, 2.5, -1.0
    slant = math.sqrt(n * n + e * e + d * d)
    # PHYSICAL pinhole box (width encodes DEPTH n: side_px = fx*side/n) -- the
    # geometry a real fronto-parallel tag projects to, which the slant-
    # corrected box channel reconstructs exactly. (`box_for` above is a
    # different, test-local convention that encodes the slant instead.)
    u, v = CX + FX * e / n, CY + FY * d / n
    bw = FX * 1.0 / n
    # AABB inflation: widen the box about its own centre (centre unchanged).
    box_wide = (u - 1.25 * bw / 2.0, v - 1.25 * bw / 2.0, 1.25 * bw, 1.25 * bw)

    g, _, _ = guidance(belief_r0_ned=(10.0, 0.0, 0.0),
                       belief_vel0_ned=(0.0, 0.0, 0.0), acquire_n=999)
    meas_box, _dir, rng_box = g._measured_r_ned(box_wide, IDENT)
    meas_pose, _dir2, rng_pose = g._measured_r_ned(box_wide, IDENT,
                                                   range_override_m=slant)
    # Box channel: ~25% short (1/1.25), the defect.
    assert rng_box == pytest.approx(slant / 1.25, rel=1e-6)
    # Pose channel: exact range and exact vector reconstruction.
    assert rng_pose == pytest.approx(slant, rel=1e-9)
    assert meas_pose == pytest.approx(np.array([n, e, d]), abs=1e-6)
    assert np.linalg.norm(meas_box - np.array([n, e, d])) > 1.0


def test_pose_range_nonpositive_or_nonfinite_falls_back_to_box():
    g, _, _ = guidance(acquire_n=999)
    box = box_for(8.0, 1.0, -0.5)
    ref, _, rref = g._measured_r_ned(box, IDENT)
    for bad in (0.0, -3.0, float("nan"), float("inf")):
        got, _, rgot = g._measured_r_ned(box, IDENT, range_override_m=bad)
        assert rgot == pytest.approx(rref, rel=1e-12)
        assert got == pytest.approx(ref, rel=1e-12)


def test_pose_range_steers_the_kf_through_step():
    """End-to-end through step(): identical box streams, one arm with a pose
    range 20% LONGER than the box range -- the KF range estimate must come out
    correspondingly longer (the channel is live, not decorative)."""
    def drive(pose_scale):
        g, _, _ = guidance(belief_r0_ned=(12.0, 0.0, 0.0),
                           belief_vel0_ned=(0.0, 0.0, 0.0),
                           acquire_n=2, acquire_window_s=0.3)
        t, tel = 0.0, None
        for _ in range(20):
            t += DT
            box = box_for(10.0, 0.0, 0.0)
            pose = 10.0 * pose_scale if pose_scale else None
            _, tel = g.step(box, own(), t, det_range_pose_m=pose)
        return tel.r_hat_m
    r_box = drive(None)
    r_pose = drive(1.2)
    assert r_pose > r_box * 1.1
    assert r_box == pytest.approx(10.0, abs=0.5)
    assert r_pose == pytest.approx(12.0, abs=0.6)


def test_sm_passes_pose_range_only_to_a_supporting_terminal():
    """RealFlightSM._step_engage must pass det_range_pose_m= ONLY when the
    terminal advertises SUPPORTS_POSE_RANGE -- the duck-typed 3-argument
    contract of the other terminals is untouched."""
    from flight.deploy.real_flight import (MissionConfig, RealFlightSM,
                                           State, TriggerState, VehicleObs)

    class RecordingTerminal:
        SUPPORTS_POSE_RANGE = True

        def __init__(self):
            self.calls = []

        def step(self, box, own_state, t, det_range_pose_m=None):
            self.calls.append((box, det_range_pose_m))
            from flight.deploy.seeker_loop import Setpoint, StepTelemetry
            tel = StepTelemetry(t=t, detected=box is not None)
            return Setpoint(0.0, 0.0, 0.0, 0.0), tel

    class LegacyTerminal:
        def __init__(self):
            self.calls = []

        def step(self, box, own_state, t):   # NO kwarg -- must never get one
            self.calls.append(box)
            from flight.deploy.seeker_loop import Setpoint, StepTelemetry
            tel = StepTelemetry(t=t, detected=box is not None)
            return Setpoint(0.0, 0.0, 0.0, 0.0), tel

    def drive(terminal):
        cfg = MissionConfig(preflight_heading_deg=0.0, standby_alt_m=7.0,
                            standby_settle_s=0.0, pursuit_mode=True)
        sm = RealFlightSM(cfg, guidance=terminal)
        no = TriggerState(go=False, link_ok=True, age_s=0.05, raw_us=1100)
        go = TriggerState(go=True, link_ok=True, age_s=0.05, raw_us=1900)

        def o(t, trig, **kw):
            base = dict(armed=True, offboard_active=True, alt_m=7.0,
                        yaw_deg=0.0, quat=(1.0, 0.0, 0.0, 0.0))
            base.update(kw)
            return VehicleObs(t=t, trigger=trig, **base)
        sm.step(o(0.0, no))
        sm.step(o(0.05, go))
        assert sm.state == State.ENGAGE     # pursuit_mode: GO -> ENGAGE direct
        sm.step(o(0.10, no, det_new=True, det_range_m=9.0,
                  det_box_xywh=(600.0, 450.0, 30.0, 30.0),
                  det_range_pose_m=9.4))
        # A tick whose detection is a MISS (range None): box gated to None,
        # so the pose range must be gated to None too.
        sm.step(o(0.15, no, det_new=True, det_range_m=None,
                  det_box_xywh=None, det_range_pose_m=7.7))
        return terminal.calls

    rec = drive(RecordingTerminal())
    assert rec[-2] == ((600.0, 450.0, 30.0, 30.0), 9.4)
    assert rec[-1] == (None, None)
    legacy = drive(LegacyTerminal())     # would TypeError on an extra kwarg
    assert legacy[-2] == (600.0, 450.0, 30.0, 30.0)


# ========================================== stopping-distance brake cap
#
# isim/specs/brake_shaping_prereg_2026-09-24.md. Config-gated, DEFAULT OFF:
# the relative command |cmd - v_t| is capped at
# sqrt(v_close_min^2 + 2*brake_accel*max(range - brake_lead*closing, 0)).
# The slew budgets are opened wide in the arithmetic tests below so step()'s
# output IS the (capped) command, not a ramp toward it.

_NO_SLEW = dict(accel_max_ms2=1e6, accel_max_horiz_b_ms2=1e6,
                accel_max_vert_b_ms2=1e6)


def test_brake_off_is_byte_identical_to_prefeature_code():
    """REPO INVARIANT: brake_shaping=False (the default) reproduces the golden
    fixture (generated from the pre-keepframe module, so it predates this
    feature too) exactly -- explicitly passed and defaulted alike."""
    golden = np.load(_FIXTURE_PATH)["cmds"]
    _assert_matches_golden(_keepframe_identity_drive(), golden)
    got = _keepframe_identity_drive({"brake_shaping": False})
    _assert_matches_golden(got, golden)   # exact-or-kernel-drift band
    assert PursuitTerminalConfig().brake_shaping is False


def _brake_phase_b_drive(own_vel_n, rng_n=6.0, **cfgkw):
    """Stationary target dead ahead at `rng_n`, own closing at `own_vel_n`:
    acquire on two on-axis boxes -> (setpoint, guidance) after the Phase-B
    tick."""
    g, cfg, _ = guidance(belief_r0_ned=(rng_n, 0.0, 0.0),
                         belief_vel0_ned=(0.0, 0.0, 0.0),
                         acquire_n=2, acquire_window_s=0.3, **_NO_SLEW, **cfgkw)
    o = own(n_e_d_vel=(own_vel_n, 0.0, 0.0))
    g.step(box_for(rng_n, 0.0, 0.0), o, t=DT)
    sp, tel = g.step(box_for(rng_n, 0.0, 0.0), o, t=2 * DT)
    assert tel.phase == "B"
    return sp, g, cfg


def test_brake_cap_arithmetic_through_step_phase_b():
    """Hot approach (14 m/s at ~6 m): the cap binds, and the emitted relative
    command equals the formula evaluated on the guidance's own KF state,
    direction preserved (the flag-off arm's direction)."""
    sp_off, g_off, _ = _brake_phase_b_drive(14.0)
    sp_on, g, cfg = _brake_phase_b_drive(14.0, brake_shaping=True)
    r, v_t = g._kf.r, g._kf.v_t
    rng = float(np.linalg.norm(r))
    closing = max(float(np.dot(np.array([14.0, 0.0, 0.0]) - v_t, r / rng)), 0.0)
    d_eff = max(rng - cfg.brake_lead_s * closing, 0.0)
    cap = math.sqrt(cfg.v_close_min_ms ** 2 + 2.0 * cfg.brake_accel_ms2 * d_eff)
    rel_on = np.array([sp_on.v_north, sp_on.v_east, sp_on.v_down]) - v_t
    rel_off = np.array([sp_off.v_north, sp_off.v_east, sp_off.v_down]) - g_off._kf.v_t
    # Same KF state in both arms (the cap never feeds back within one tick).
    assert g_off._kf.x == pytest.approx(g._kf.x, abs=1e-12)
    assert np.linalg.norm(rel_off) > cap + 0.5          # the cap really binds
    assert np.linalg.norm(rel_on) == pytest.approx(cap, abs=1e-9)
    assert rel_on / np.linalg.norm(rel_on) == pytest.approx(
        rel_off / np.linalg.norm(rel_off), abs=1e-9)
    # At 14 m/s the lead eats the whole ~6 m: the floor is v_close_min.
    assert d_eff == 0.0 and cap == pytest.approx(cfg.v_close_min_ms)


def test_brake_cap_tightens_monotonically_as_range_shrinks():
    g, cfg, _ = guidance(brake_shaping=True)
    v_t = np.array([2.0, 0.0, 0.0])
    own_vel = np.array([10.0, 0.0, 0.0])          # closing 8 m/s
    big = v_t + np.array([50.0, 0.0, 0.0])        # far above any cap
    rels = []
    for rng in (30.0, 20.0, 12.0, 8.0, 5.0, 3.6, 2.0):
        cmd = g._brake_cap(big, v_t, np.array([rng, 0.0, 0.0]), own_vel)
        rels.append(float(np.linalg.norm(cmd - v_t)))
    assert all(a > b for a, b in zip(rels[:5], rels[1:6]))   # strictly tighter
    # Inside brake_lead_s*closing (3.6 m) the cap sits on the floor.
    assert rels[-2] == pytest.approx(cfg.v_close_min_ms)
    assert rels[-1] == pytest.approx(cfg.v_close_min_ms)
    # A command already inside the cap passes through untouched.
    small = v_t + np.array([1.0, 0.0, 0.0])
    assert np.array_equal(
        g._brake_cap(small, v_t, np.array([30.0, 0.0, 0.0]), own_vel), small)


def test_brake_lead_zero_is_looser_than_lead_positive():
    """Closing geometry: the lead term shrinks the effective range, so
    lead>0 caps TIGHTER than lead=0 (through step()); on an opening geometry
    the closing speed floors at 0 and the two caps are equal."""
    sp_lead, g_lead, _ = _brake_phase_b_drive(12.0, rng_n=7.0, brake_shaping=True)
    sp_zero, g_zero, _ = _brake_phase_b_drive(12.0, rng_n=7.0, brake_shaping=True,
                                             brake_lead_s=0.0)
    rel_lead = np.linalg.norm(np.array([sp_lead.v_north, sp_lead.v_east,
                                        sp_lead.v_down]) - g_lead._kf.v_t)
    rel_zero = np.linalg.norm(np.array([sp_zero.v_north, sp_zero.v_east,
                                        sp_zero.v_down]) - g_zero._kf.v_t)
    assert rel_lead < rel_zero - 0.2     # a real margin, not a rounding tie
    g, _, _ = guidance(brake_shaping=True)
    g0, _, _ = guidance(brake_shaping=True, brake_lead_s=0.0)
    r = np.array([6.0, 0.0, 0.0])
    opening = np.array([-3.0, 0.0, 0.0])
    assert g._brake_rel_cap(6.0, g._brake_closing(r, np.zeros(3), opening)) == \
        g0._brake_rel_cap(6.0, g0._brake_closing(r, np.zeros(3), opening))


def test_brake_phase_a_caps_only_inside_the_stopping_envelope():
    """Phase A, belief 20 m ahead and stationary, rendezvous point d_behind_m
    short of it. At rest (closing 0 -> envelope 0) the command is untouched;
    closing at 15 m/s the rendezvous (~11.25 m) is inside 2x the envelope and
    the relative command is capped by the formula on r_aim."""
    kw = dict(belief_r0_ned=(20.0, 0.0, 0.0), belief_vel0_ned=(0.0, 0.0, 0.0))
    g_off, _, _ = guidance(**kw, **_NO_SLEW)
    g_on, cfg, _ = guidance(**kw, brake_shaping=True, **_NO_SLEW)
    sp_off, _ = g_off.step(None, own(), t=DT)
    sp_on, tel = g_on.step(None, own(), t=DT)
    assert tel.phase == "A"
    assert (sp_on.v_north, sp_on.v_east, sp_on.v_down) == \
        (sp_off.v_north, sp_off.v_east, sp_off.v_down)

    g_off, _, _ = guidance(**kw, **_NO_SLEW)
    g_on, cfg, _ = guidance(**kw, brake_shaping=True, **_NO_SLEW)
    hot = own(n_e_d_vel=(15.0, 0.0, 0.0))
    sp_off, _ = g_off.step(None, hot, t=DT)
    sp_on, _ = g_on.step(None, hot, t=DT)
    r_aim = 20.0 - 15.0 * DT - cfg.d_behind_m               # 11.25 m
    assert sp_off.v_north == pytest.approx(cfg.kp_pos * r_aim)   # 9.0, uncapped
    d_eff = r_aim - cfg.brake_lead_s * 15.0
    cap = math.sqrt(cfg.v_close_min_ms ** 2 + 2.0 * cfg.brake_accel_ms2 * d_eff)
    assert sp_on.v_north == pytest.approx(cap, abs=1e-9)     # ~6.18 m/s
    assert sp_on.v_east == 0.0 and sp_on.v_down == 0.0


def test_brake_horizontal_only_passes_the_climb_through():
    """Amendment #1 (brake_shaping_prereg_2026-09-24.md): with the default
    brake_horizontal_only, a climbing relative command keeps its ENTIRE
    vertical component while the horizontal part is capped on horizontal
    range/closing; brake_horizontal_only=False reproduces the flown 3-D cap
    (whole vector scaled, climb included)."""
    g_h, cfg, _ = guidance(brake_shaping=True)
    g_3, cfg3, _ = guidance(brake_shaping=True, brake_horizontal_only=False)
    r = np.array([6.0, 0.0, -3.0])          # target ahead and 3 m ABOVE
    v_t = np.zeros(3)
    own_vel = np.array([10.0, 0.0, -2.0])   # closing hot, climbing
    cmd = np.array([8.0, 0.0, -2.5])

    out_h = g_h._brake_cap(cmd, v_t, r, own_vel)
    cap_h = g_h._brake_rel_cap(6.0, 10.0)   # horizontal range/closing only
    assert out_h[2] == pytest.approx(-2.5)              # climb untouched
    assert math.hypot(out_h[0], out_h[1]) == pytest.approx(cap_h, abs=1e-9)
    assert out_h[1] == pytest.approx(0.0)

    out_3 = g_3._brake_cap(cmd, v_t, r, own_vel)
    rng3 = float(np.linalg.norm(r))
    closing3 = max(float(np.dot(own_vel - v_t, r / rng3)), 0.0)
    cap_3 = g_3._brake_rel_cap(rng3, closing3)
    assert np.linalg.norm(out_3) == pytest.approx(cap_3, abs=1e-9)
    assert out_3[2] != pytest.approx(-2.5)              # 3-D cap scales climb
    # Same direction as the raw command (3-D mode preserves the full vector).
    assert out_3 / np.linalg.norm(out_3) == pytest.approx(cmd / np.linalg.norm(cmd))


def test_brake_vert_sync_schedules_the_climb_to_the_horizontal_closure():
    """Amendment #2: with sync on, the vertical relative command equals
    dz / t_go_h (both gaps zero together); sign = climb toward a target
    ABOVE; default off = amendment-#1 vertical passthrough exactly."""
    g, cfg, _ = guidance(brake_shaping=True, brake_vert_sync=True)
    v_t = np.zeros(3)
    own_vel = np.array([8.0, 0.0, 0.0])
    r = np.array([6.0, 0.0, -3.0])          # target ahead, 3 m ABOVE
    cmd = np.array([5.0, 0.0, -2.5])
    out = g._brake_sync_vertical(cmd, v_t, r, own_vel)
    t_go = 6.0 / 8.0                         # d_h / closing_h (> v_close_min)
    assert out[2] == pytest.approx(-3.0 / t_go)          # climb, dz/t_go
    assert out[0] == pytest.approx(cmd[0]) and out[1] == pytest.approx(cmd[1])
    # Arrival-sync property: commanded vertical rate / gap == closing / d_h.
    assert out[2] / r[2] == pytest.approx(8.0 / 6.0)
    # Slow closure floors at v_close_min in the schedule (no divide blowup).
    out2 = g._brake_sync_vertical(cmd, v_t, r, np.zeros(3))
    assert out2[2] == pytest.approx(-3.0 / (6.0 / cfg.v_close_min_ms))
    # Magnitude clamp at v_max.
    out3 = g._brake_sync_vertical(cmd, v_t, np.array([0.01, 0.0, -3.0]), own_vel)
    assert abs(out3[2]) <= cfg.v_max_ms + 1e-9
    # Default OFF: config default is False and the Phase-B path then matches
    # the amendment-#1 drive exactly.
    assert PursuitTerminalConfig().brake_vert_sync is False
    a = _keepframe_identity_drive({"brake_shaping": True, "brake_accel_ms2": 3.0})
    b = _keepframe_identity_drive({"brake_shaping": True, "brake_accel_ms2": 3.0,
                                   "brake_vert_sync": False})
    assert np.array_equal(a, b, equal_nan=True)
