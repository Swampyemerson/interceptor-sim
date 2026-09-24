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


# ================================================== rehearsal break-off
#
# isim/specs/rehearsal_breakoff_prereg_2026-09-24.md. Config-gated, DEFAULT
# OFF: on a freshly-fed (>= rehearsal_min_updates KF updates inside
# rehearsal_fresh_s), CLOSING Phase-B track whose horizontal time-to-go
# (AMENDMENT #1: d_h / max(closing_h, v_close_min_ms)) is inside
# rehearsal_t_react_s and whose |KF r| is inside the rehearsal_range_m upper
# bound, record the onboard would-have ZEM and fly the latched evade (climb +
# lateral away from the target's path, yaw held) for rehearsal_evade_s, then
# raise rehearsal_complete for RealFlightSM's SAFE 'rehearsal_breakoff'. If
# the first eligible tick is already inside rehearsal_t_late_s: latch
# rehearsal_too_late instead and never trigger.


def test_rehearsal_off_is_byte_identical_to_prefeature_code():
    """REPO INVARIANT: rehearsal_breakoff=False (the default) reproduces the
    golden fixture (generated from the pre-keepframe module, so it predates
    this feature too) exactly -- explicitly passed and defaulted alike."""
    golden = np.load(_FIXTURE_PATH)["cmds"]
    _assert_matches_golden(_keepframe_identity_drive(), golden)
    got = _keepframe_identity_drive({"rehearsal_breakoff": False})
    _assert_matches_golden(got, golden)   # exact-or-kernel-drift band
    assert PursuitTerminalConfig().rehearsal_breakoff is False


def test_rehearsal_on_never_triggering_is_identical_to_off():
    """Flag ON but never eligible -- two independent routes: (a) the target
    never inside the rehearsal_range_m upper bound (the fixture drive's
    closest box is 5 m), (b) t_go never inside a tiny rehearsal_t_react_s.
    The commands are identical to the OFF arm -- the extra update
    bookkeeping never feeds a command -- and no too-late latch fires."""
    golden = np.load(_FIXTURE_PATH)["cmds"]
    for kw in ({"rehearsal_breakoff": True, "rehearsal_range_m": 4.0},
               {"rehearsal_breakoff": True, "rehearsal_t_react_s": 0.05,
                "rehearsal_t_late_s": 0.0}):
        on = _keepframe_identity_drive(kw)
        _assert_matches_golden(on, golden)
        assert np.array_equal(on, _keepframe_identity_drive(), equal_nan=True)


def _rehearsal_drive(boxes, own_vel=(3.0, 0.0, 0.0), belief_vel=(0.0, 0.0, 0.0),
                     belief_r0=(2.0, 0.0, 0.0), **cfgkw):
    """Step once per entry of `boxes` (None = no decode); -> (guidance,
    [(setpoint, tel), ...])."""
    g, cfg, _ = guidance(belief_r0_ned=belief_r0, belief_vel0_ned=belief_vel,
                         acquire_n=2, acquire_window_s=0.3, **cfgkw)
    out, t = [], 0.0
    for b in boxes:
        t += DT
        out.append(g.step(b, own(n_e_d_vel=own_vel), t=t))
    return g, cfg, out


def test_rehearsal_gate_needs_n_fresh_updates_and_records_the_zem():
    """Acquire on tick 1-2 (the first KF update lands on tick 2), so with
    rehearsal_min_updates=3 the trigger cannot fire before tick 4 even though
    range/closing/t_go qualify from the start (t_go ~1.2 s, inside the 2.0 s
    default and above the 0.6 s floor); on tick 4 it fires, the recorded
    t_go is the amendment-#1 horizontal t_go and the would-have estimate
    equals the ZEM formula on the KF state at that tick."""
    boxes = [box_for(4.0 - 0.15 * i, 0.2, 0.0) for i in range(4)]
    g, cfg, out = _rehearsal_drive(boxes[:3], belief_r0=(4.0, 0.2, 0.0),
                                   rehearsal_breakoff=True)
    assert out[-1][1].phase == "B"
    assert g.rehearsal_trigger_t is None          # only 2 updates so far
    g, cfg, out = _rehearsal_drive(boxes, belief_r0=(4.0, 0.2, 0.0),
                                   rehearsal_breakoff=True)
    assert g.rehearsal_trigger_t == pytest.approx(4 * DT)
    assert not g.rehearsal_too_late
    own_v = np.array([3.0, 0.0, 0.0])
    r, v_rel = g._kf.r, g._kf.v_t - own_v
    # Trigger clock: horizontal range over horizontal closing (> v_close_min).
    r_h = np.array([r[0], r[1], 0.0])
    d_h = float(np.linalg.norm(r_h))
    closing_h = float(np.dot(own_v - g._kf.v_t, r_h / d_h))
    assert closing_h > cfg.v_close_min_ms
    assert g.rehearsal_t_go_s == pytest.approx(d_h / closing_h, abs=1e-12)
    assert cfg.rehearsal_t_late_s < g.rehearsal_t_go_s <= cfg.rehearsal_t_react_s
    t_cpa = -float(np.dot(r, v_rel)) / float(np.dot(v_rel, v_rel))
    assert t_cpa > 0.0
    assert g.rehearsal_zem_m == pytest.approx(
        float(np.linalg.norm(r + v_rel * t_cpa)), abs=1e-12)
    assert g.rehearsal_trigger_range_m == pytest.approx(float(np.linalg.norm(r)))
    assert g.rehearsal_trigger_range_m <= cfg.rehearsal_range_m
    # A higher N delays it: same stream, 4 updates needed -> not yet.
    g4, _, _ = _rehearsal_drive(boxes, belief_r0=(4.0, 0.2, 0.0),
                                rehearsal_breakoff=True, rehearsal_min_updates=4)
    assert g4.rehearsal_trigger_t is None


def test_rehearsal_coasting_track_does_not_trigger():
    """Decodes stop at ~8.5 m (t_go ~2.8 s, outside the 2.0 s window); the KF
    then COASTS (predicts) the range down to ~0 -- through the whole t_go
    window -- while the last update is > rehearsal_fresh_s old: a coasting
    track must NOT trigger (and, the gate never opening, must not latch
    too-late either). The control arm (decodes continue) does trigger."""
    fed = [box_for(9.0 - 0.15 * i, 0.0, 0.0) for i in range(4)]
    g, cfg, out = _rehearsal_drive(fed + [None] * 56, belief_r0=(9.0, 0.0, 0.0),
                                   rehearsal_breakoff=True)
    assert all(tel.phase == "B" for _sp, tel in out[1:])
    assert min(tel.r_hat_m for _sp, tel in out[1:]) < cfg.hold_range_m
    assert g.rehearsal_trigger_t is None
    assert not g.rehearsal_too_late
    assert not g.rehearsal_complete
    ctrl = [box_for(max(9.0 - 0.15 * i, 0.3), 0.0, 0.0) for i in range(60)]
    g2, _, _ = _rehearsal_drive(ctrl, belief_r0=(9.0, 0.0, 0.0),
                                rehearsal_breakoff=True)
    assert g2.rehearsal_trigger_t is not None
    assert g2.rehearsal_t_go_s <= cfg.rehearsal_t_react_s


def test_rehearsal_opening_track_does_not_trigger():
    """Inside range, fed, but OPENING (own receding): closing <= 0 -> no
    trigger (the floored t_go alone, 2 m / v_close_min, would be inside the
    window -- the closing test is what stops it), and no too-late latch."""
    boxes = [box_for(2.0 + 0.15 * i, 0.0, 0.0) for i in range(6)]
    g, _, _ = _rehearsal_drive(boxes, own_vel=(-3.0, 0.0, 0.0),
                               rehearsal_breakoff=True, rehearsal_range_m=3.0)
    assert g.rehearsal_trigger_t is None
    assert not g.rehearsal_too_late


def test_rehearsal_t_go_arithmetic_through_step():
    """Known geometry: stationary target, own closing at 3 m/s, target 1 m
    ABOVE and 0.5 m east. The trigger fires on the first tick whose
    amendment-#1 t_go = d_h / max(closing_h, v_close_min_ms) -- recomputed
    here from the guidance's own KF state -- is inside rehearsal_t_react_s,
    and NOT on the tick before (where the same formula is outside it). The
    vertical offset is excluded from the clock (horizontal only)."""
    own_v = np.array([3.0, 0.0, 0.0])
    boxes = [box_for(9.0 - 0.15 * i, 0.5, -1.0) for i in range(40)]
    g, cfg, _ = guidance(belief_r0_ned=(9.0, 0.5, -1.0),
                         belief_vel0_ned=(0.0, 0.0, 0.0),
                         acquire_n=2, acquire_window_s=0.3,
                         rehearsal_breakoff=True)

    def formula():
        r, v_t = g._kf.r, g._kf.v_t
        r_h = np.array([r[0], r[1], 0.0])
        d_h = float(np.linalg.norm(r_h))
        closing_h = max(float(np.dot(own_v - v_t, r_h / d_h)), 0.0)
        return d_h / max(closing_h, cfg.v_close_min_ms)

    t, fired_at, prev = 0.0, None, None
    for b in boxes:
        t += DT
        g.step(b, own(n_e_d_vel=tuple(own_v)), t=t)
        if g._phase == "B" and g.rehearsal_trigger_t is None:
            prev = formula()
        if g.rehearsal_trigger_t is not None:
            fired_at = t
            break
    assert fired_at is not None and g.rehearsal_trigger_t == pytest.approx(fired_at)
    assert prev is not None and prev > cfg.rehearsal_t_react_s   # tick before: outside
    assert g.rehearsal_t_go_s == pytest.approx(formula(), abs=1e-12)
    assert g.rehearsal_t_go_s <= cfg.rehearsal_t_react_s
    # ~ d_h / 3 m/s: the trigger range is ~ t_react * closing (horizontal).
    assert g.rehearsal_t_go_s * 3.0 == pytest.approx(
        float(np.hypot(g._kf.r[0], g._kf.r[1])), rel=0.05)
    # A larger t_react fires EARLIER (farther out) on the same stream.
    g3, _, _ = _rehearsal_drive(boxes, belief_r0=(9.0, 0.5, -1.0),
                                rehearsal_breakoff=True, rehearsal_t_react_s=2.5)
    assert g3.rehearsal_trigger_t < g.rehearsal_trigger_t
    assert g3.rehearsal_trigger_range_m > g.rehearsal_trigger_range_m


def test_rehearsal_slow_closing_inside_range_does_not_trigger():
    """Fed, closing, well INSIDE the rehearsal_range_m upper bound -- but
    closing slowly (0.5 m/s, floored to v_close_min_ms in the clock), so t_go =
    d_h / 1.5 m/s stays above rehearsal_t_react_s while d_h > 3 m: no
    trigger. The same start at a hot closing speed does trigger: reaction
    time, not range, is what gates it now."""
    slow = [box_for(4.0 - 0.025 * i, 0.0, 0.0) for i in range(30)]
    g, cfg, out = _rehearsal_drive(slow, own_vel=(0.5, 0.0, 0.0),
                                   belief_r0=(4.0, 0.0, 0.0),
                                   rehearsal_breakoff=True)
    assert all(tel.r_hat_m < cfg.rehearsal_range_m for _sp, tel in out[2:])
    assert min(tel.r_hat_m for _sp, tel in out[2:]) > \
        cfg.rehearsal_t_react_s * cfg.v_close_min_ms
    assert g.rehearsal_trigger_t is None
    assert not g.rehearsal_too_late
    hot = [box_for(4.0 - 0.15 * i, 0.0, 0.0) for i in range(6)]
    g2, _, _ = _rehearsal_drive(hot, own_vel=(3.0, 0.0, 0.0),
                                belief_r0=(4.0, 0.0, 0.0), rehearsal_breakoff=True)
    assert g2.rehearsal_trigger_t is not None
    # The v_close_min_ms floor is live: the same 0.5 m/s crawl from 2.0 m has
    # a floored t_go of ~1.3 s (inside the adopted 1.5 s window, above the
    # 0.6 s too-late floor) -- an unfloored clock (2.0 / 0.5 = 4.0 s) would
    # never fire it. (Geometry re-sized when the round-2 sweep adopted
    # t_react 1.5 s; the old 2.6 m case floored to ~1.7 s and only fired at
    # the retired 2.0 s default.)
    crawl = [box_for(2.0 - 0.025 * i, 0.0, 0.0) for i in range(4)]   # fires on tick 4
    g3, _, _ = _rehearsal_drive(crawl, own_vel=(0.5, 0.0, 0.0),
                                belief_r0=(2.0, 0.0, 0.0), rehearsal_breakoff=True)
    assert g3.rehearsal_trigger_t is not None
    assert g3.rehearsal_t_go_s == pytest.approx(
        float(np.hypot(g3._kf.r[0], g3._kf.r[1])) / cfg.v_close_min_ms, abs=1e-12)


def test_rehearsal_too_late_floor_latches_and_never_triggers(capsys):
    """The gate first becomes satisfied (tick 4, the 3rd KF update) with t_go
    already under rehearsal_t_late_s: the floor latches `rehearsal_too_late`
    with its telemetry, prints ONE FAULT-style line, never triggers for the
    rest of the flight (even though t_go stays in the window and the track
    stays fed), and the commands are the flag-off chase exactly -- the pass
    is not relabelled. Mutation control: a lower floor on the same stream
    triggers instead."""
    boxes = [box_for(max(1.6 - 0.15 * i, 0.05), 0.2, 0.0) for i in range(12)]
    kw = dict(own_vel=(3.0, 0.0, 0.0), belief_r0=(1.6, 0.2, 0.0))
    g, cfg, out = _rehearsal_drive(boxes, rehearsal_breakoff=True, **kw)
    assert g.rehearsal_too_late
    assert g.rehearsal_too_late_t == pytest.approx(4 * DT)
    assert g.rehearsal_too_late_t_go_s < cfg.rehearsal_t_late_s
    assert g.rehearsal_too_late_range_m < 1.5
    assert g.rehearsal_trigger_t is None and not g.rehearsal_complete
    assert g.rehearsal_zem_m is None
    lines = [ln for ln in capsys.readouterr().out.splitlines()
             if "FAULT rehearsal_too_late" in ln]
    assert len(lines) == 1
    assert "rehearsal_too_late" in out[3][1].health            # the latch tick
    assert all("rehearsal_too_late" not in tel.health for _sp, tel in out[4:])
    _g0, _, out_off = _rehearsal_drive(boxes, **kw)            # flag off
    for (sp_on, _t1), (sp_off, _t2) in zip(out, out_off):
        assert (sp_on.v_north, sp_on.v_east, sp_on.v_down, sp_on.yaw_deg) == \
            (sp_off.v_north, sp_off.v_east, sp_off.v_down, sp_off.yaw_deg)
    g_ctl, _, _ = _rehearsal_drive(boxes, rehearsal_breakoff=True,
                                   rehearsal_t_late_s=0.1, **kw)
    assert g_ctl.rehearsal_trigger_t == pytest.approx(4 * DT)
    assert not g_ctl.rehearsal_too_late


def test_rehearsal_too_late_is_first_eligibility_not_first_gate_opening():
    """The aim20 hole of round 1: the gate is open FAR out (t_go outside the
    window -- not eligible), the track then drops out through the whole
    t_react..t_late window, and the gate re-opens with t_go already under
    the floor. The floor is judged at the first ELIGIBLE tick, so this latches
    too-late; the continuously-fed control arm triggers normally."""
    fed = [box_for(9.0 - 0.15 * i, 0.0, 0.0) for i in range(6)]
    gap = [None] * 46                                    # coast 9 -> ~1.3 m
    late = [box_for(max(9.0 - 0.15 * i, 0.3), 0.0, 0.0) for i in range(52, 60)]
    g, cfg, _ = _rehearsal_drive(fed + gap + late, belief_r0=(9.0, 0.0, 0.0),
                                 rehearsal_breakoff=True)
    assert g.rehearsal_trigger_t is None
    assert g.rehearsal_too_late
    assert g.rehearsal_too_late_t_go_s < cfg.rehearsal_t_late_s
    ctrl = [box_for(max(9.0 - 0.15 * i, 0.3), 0.0, 0.0) for i in range(60)]
    g2, _, _ = _rehearsal_drive(ctrl, belief_r0=(9.0, 0.0, 0.0),
                                rehearsal_breakoff=True)
    assert g2.rehearsal_trigger_t is not None and not g2.rehearsal_too_late


_NO_SLEW_R = dict(accel_max_ms2=1e6, accel_max_horiz_b_ms2=1e6,
                  accel_max_vert_b_ms2=1e6)


def _moving_target_drive(east_offset, n_ticks=4, **cfgkw):
    """Target moving NORTH at 3 m/s, own north at 6 m/s (closing 3, t_go
    ~1.4 s at the tick-4 trigger), target `east_offset` metres east of own ->
    the vehicle sits WEST (left) of the target's path for east_offset > 0."""
    boxes = [box_for(4.5 - 0.15 * i, east_offset, 0.0) for i in range(n_ticks)]
    return _rehearsal_drive(boxes, own_vel=(6.0, 0.0, 0.0),
                            belief_vel=(3.0, 0.0, 0.0),
                            belief_r0=(4.5, east_offset, 0.0),
                            rehearsal_breakoff=True, **cfgkw)


def test_rehearsal_evade_climbs_and_pushes_away_from_the_path_side():
    g_l, cfg, out_l = _moving_target_drive(0.5, **_NO_SLEW_R)
    assert g_l.rehearsal_trigger_t == pytest.approx(4 * DT)
    assert g_l.rehearsal_side == "left"
    sp_prev, _ = out_l[-2]
    sp, tel = out_l[-1]
    assert sp.v_down < -1.0                         # climbing (NED down < 0)
    assert sp.v_east < -1.0                         # pushed WEST, away from the path
    assert math.sqrt(sp.v_north ** 2 + sp.v_east ** 2 + sp.v_down ** 2) \
        <= cfg.v_max_ms + 1e-9                      # the norm cap still applies
    assert sp.yaw_deg == pytest.approx(sp_prev.yaw_deg)   # yaw held

    g_r, _, out_r = _moving_target_drive(-0.5, **_NO_SLEW_R)
    assert g_r.rehearsal_side == "right"
    assert out_r[-1][0].v_east > 1.0                # mirrored: pushed EAST

    # On the path (no lateral offset): fallback = right of the target's course.
    g_c, _, out_c = _moving_target_drive(0.0, **_NO_SLEW_R)
    assert g_c.rehearsal_side == "right_fallback"
    assert out_c[-1][0].v_east > 1.0


def test_rehearsal_evade_respects_slew_budgets_and_completes():
    """Default budgets: the evade ramps within the Phase-B split budgets; after
    rehearsal_evade_s of SIM time the terminal raises rehearsal_complete."""
    g, cfg, out = _moving_target_drive(0.5, n_ticks=4)
    t_trig = g.rehearsal_trigger_t
    assert t_trig is not None and not g.rehearsal_complete
    prev = np.array([out[-1][0].v_north, out[-1][0].v_east, out[-1][0].v_down])
    t = 4 * DT
    while t - t_trig < cfg.rehearsal_evade_s - 1e-9:
        t += DT
        assert not g.rehearsal_complete
        sp, _tel = g.step(None, own(n_e_d_vel=(6.0, 0.0, 0.0)), t=t)
        cur = np.array([sp.v_north, sp.v_east, sp.v_down])
        assert np.linalg.norm(cur[0:2] - prev[0:2]) <= cfg.accel_max_horiz_b_ms2 * DT + 1e-9
        assert abs(cur[2] - prev[2]) <= cfg.accel_max_vert_b_ms2 * DT + 1e-9
        prev = cur
    assert g.rehearsal_complete
    assert prev[2] < -3.0                           # well into the climb


def test_sm_ends_a_rehearsal_through_safe_with_its_own_reason():
    """RealFlightSM + the real terminal: the trigger lands in the mission log,
    the lost-target failsafe does NOT relabel the pass while the evade runs
    (engage_lost_target_s set SHORTER than the evade), and the engagement ends
    in SAFE 'rehearsal_breakoff', a MISS-class reason (hover)."""
    from flight.deploy.real_flight import (MissionConfig, RealFlightSM,
                                           State, TriggerState, VehicleObs)
    g, _cfg, _ = guidance(belief_r0_ned=(4.5, 0.5, 0.0),
                          belief_vel0_ned=(3.0, 0.0, 0.0),
                          acquire_n=2, acquire_window_s=0.3,
                          rehearsal_breakoff=True)
    mcfg = MissionConfig(preflight_heading_deg=0.0, standby_alt_m=7.0,
                         standby_settle_s=0.0, pursuit_mode=True,
                         engage_lost_target_s=1.0, engage_max_s=60.0)
    sm = RealFlightSM(mcfg, guidance=g)
    no = TriggerState(go=False, link_ok=True, age_s=0.05, raw_us=1100)
    go = TriggerState(go=True, link_ok=True, age_s=0.05, raw_us=1900)

    def o(t, trig, **kw):
        base = dict(armed=True, offboard_active=True, alt_m=7.0, yaw_deg=0.0,
                    quat=(1.0, 0.0, 0.0, 0.0), vel_ned=(6.0, 0.0, 0.0))
        base.update(kw)
        return VehicleObs(t=t, trigger=trig, **base)
    sm.step(o(0.0, no))
    sm.step(o(0.05, go))
    assert sm.state == State.ENGAGE
    t = 0.05
    for i in range(4):
        t += DT
        rng_n = 4.5 - 0.15 * i
        sm.step(o(t, no, det_new=True, det_range_m=rng_n,
                  det_box_xywh=box_for(rng_n, 0.5, 0.0)))
    assert g.rehearsal_trigger_t is not None
    while sm.state == State.ENGAGE and t < 10.0:
        t += DT
        sm.step(o(t, no))
    assert sm.state == State.SAFE
    assert sm.safe_reason == "rehearsal_breakoff"
    assert sm.safe_reason in RealFlightSM.MISS_SAFE_REASONS
    assert sm.effective_safe_behavior() == mcfg.miss_safe_behavior
    assert "rehearsal_breakoff" in sm.transitions[-1].reason
    assert (t - g.rehearsal_trigger_t) == pytest.approx(g.cfg.rehearsal_evade_s,
                                                        abs=DT + 1e-9)


def test_sm_without_rehearsal_still_reports_pursuit_miss():
    """Control for the test above: flag OFF, same stream -> the lost-target
    clock ends it as the pre-existing pursuit_miss."""
    from flight.deploy.real_flight import (MissionConfig, RealFlightSM,
                                           State, TriggerState, VehicleObs)
    g, _cfg, _ = guidance(belief_r0_ned=(4.5, 0.5, 0.0),
                          belief_vel0_ned=(3.0, 0.0, 0.0),
                          acquire_n=2, acquire_window_s=0.3)
    mcfg = MissionConfig(preflight_heading_deg=0.0, standby_alt_m=7.0,
                         standby_settle_s=0.0, pursuit_mode=True,
                         engage_lost_target_s=1.0, engage_max_s=60.0)
    sm = RealFlightSM(mcfg, guidance=g)
    no = TriggerState(go=False, link_ok=True, age_s=0.05, raw_us=1100)
    go = TriggerState(go=True, link_ok=True, age_s=0.05, raw_us=1900)

    def o(t, trig, **kw):
        base = dict(armed=True, offboard_active=True, alt_m=7.0, yaw_deg=0.0,
                    quat=(1.0, 0.0, 0.0, 0.0), vel_ned=(6.0, 0.0, 0.0))
        base.update(kw)
        return VehicleObs(t=t, trigger=trig, **base)
    sm.step(o(0.0, no))
    sm.step(o(0.05, go))
    t = 0.05
    for i in range(4):
        t += DT
        rng_n = 4.5 - 0.15 * i
        sm.step(o(t, no, det_new=True, det_range_m=rng_n,
                  det_box_xywh=box_for(rng_n, 0.5, 0.0)))
    while sm.state == State.ENGAGE and t < 10.0:
        t += DT
        sm.step(o(t, no))
    assert g.rehearsal_trigger_t is None
    assert sm.safe_reason == "pursuit_miss"


def test_sm_logs_a_too_late_rehearsal_once_and_does_not_relabel_the_end():
    """RealFlightSM + the real terminal, too-late geometry (first eligible
    t_go under rehearsal_t_late_s): the mission log carries ONE 'REHEARSAL
    too late' line, no break-off line, and the engagement ends through the
    pre-existing pursuit_miss path -- not rehearsal_breakoff."""
    from flight.deploy.real_flight import (MissionConfig, RealFlightSM,
                                           State, TriggerState, VehicleObs)
    g, _cfg, _ = guidance(belief_r0_ned=(1.6, 0.5, 0.0),
                          belief_vel0_ned=(3.0, 0.0, 0.0),
                          acquire_n=2, acquire_window_s=0.3,
                          rehearsal_breakoff=True)
    mcfg = MissionConfig(preflight_heading_deg=0.0, standby_alt_m=7.0,
                         standby_settle_s=0.0, pursuit_mode=True,
                         engage_lost_target_s=1.0, engage_max_s=60.0)
    sm = RealFlightSM(mcfg, guidance=g)
    no = TriggerState(go=False, link_ok=True, age_s=0.05, raw_us=1100)
    go = TriggerState(go=True, link_ok=True, age_s=0.05, raw_us=1900)
    msgs = []
    sm._emit = lambda msg, events: msgs.append(msg)

    def o(t, trig, **kw):
        base = dict(armed=True, offboard_active=True, alt_m=7.0, yaw_deg=0.0,
                    quat=(1.0, 0.0, 0.0, 0.0), vel_ned=(6.0, 0.0, 0.0))
        base.update(kw)
        return VehicleObs(t=t, trigger=trig, **base)
    sm.step(o(0.0, no))
    sm.step(o(0.05, go))
    t = 0.05
    for i in range(6):
        t += DT
        rng_n = 1.6 - 0.15 * i
        sm.step(o(t, no, det_new=True, det_range_m=rng_n,
                  det_box_xywh=box_for(rng_n, 0.5, 0.0)))
    assert g.rehearsal_too_late and g.rehearsal_trigger_t is None
    while sm.state == State.ENGAGE and t < 10.0:
        t += DT
        sm.step(o(t, no))
    late = [m for m in msgs if "REHEARSAL too late" in m]
    assert len(late) == 1
    assert not any("REHEARSAL break-off" in m for m in msgs)
    assert sm.safe_reason == "pursuit_miss"
