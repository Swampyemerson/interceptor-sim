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


def test_keepframe_off_is_byte_identical_to_prefeature_code():
    """REPO INVARIANT: with keepframe_assist=False (the default) the emitted
    commands are BYTE-IDENTICAL to the module as it was before the feature
    existed -- the fixture was generated by running this exact drive (top-edge
    boxes, dropout and all) against the pre-change code."""
    golden = np.load(_FIXTURE_PATH)["cmds"]
    got = _keepframe_identity_drive()
    assert got.shape == golden.shape
    assert np.array_equal(got, golden, equal_nan=True)   # exact, not approx
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
