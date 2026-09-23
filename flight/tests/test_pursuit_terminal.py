"""flight/tests/test_pursuit_terminal.py -- unit tests for
flight.pursuit_terminal, the "chase only" pursuit-rendezvous camera terminal
(ADR-0103; docs/pursuit_port_2026-09-17.md).

Pure unit tests: no sim, no MAVSDK, no weights. `PursuitTerminalGuidance` is a
pure step function exactly like `TagInterceptGuidance`, so it is driven the
same way `test_tag_terminal.py` drives that class.
"""
import math

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
