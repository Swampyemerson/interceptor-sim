"""flight/tests/test_tag_terminal.py -- unit tests for flight.tag_terminal, the
3-D predicted-intercept-point (PIP) camera terminal.

Pure unit tests: no sim, no MAVSDK, no weights. `TagInterceptGuidance` is a
pure step function exactly like `SeekerGuidance`, so it is driven the same way
`test_seeker_loop_coast.py` drives that class.
"""
import math

import numpy as np
import pytest

from flight.camera import CameraModel
from flight.deploy.seeker_loop import GuidanceConfig, OwnState
from flight.fov_guidance import FovHoldConfig, half_fov_from_intrinsics
from flight.geometry import derotate_bearing_lambda, euler_to_quat_body_to_ned
from flight.tag_terminal import (
    TagInterceptGuidance,
    TagTerminalConfig,
    _cam_offset_ned,
    _look_angle_accel_bounds,
    _ned_vec_to_optical,
    _optical_vec_to_ned,
    _pitch_rad_from_quat,
    range_measurement_noise,
    solve_intercept_t_go,
)

FX = FY = 539.936
CX, CY = 640.0, 480.0
DT = 0.05
IDENT = (1.0, 0.0, 0.0, 0.0)   # level attitude, nose north


def cam():
    return CameraModel(FX, FY, CX, CY)


def own(n_e_d_vel=(0.0, 0.0, 0.0), quat=IDENT, alt_m=5.0, age_s=0.0):
    return OwnState(quat=quat, psi_rad=0.0, alt_m=alt_m, age_s=age_s,
                    vel_ned=n_e_d_vel)


def box_for(n, e, d, side_m=1.0, fx=FX, fy=FY, cx=CX, cy=CY, noise_px=0.0, rng=None):
    """A detector box that, under IDENTITY attitude, zero mount tilt and zero
    camera offset, reconstructs EXACTLY the NED relative position (n, e, d)
    through measurement_from_box + tag_terminal's optical->NED transform (see
    the derivation in this module's docstring companion: xn=e/n, yn=d/n,
    bw = fx*side/slant_range gives meas_xyz = (e, d, n) exactly)."""
    slant = math.sqrt(n * n + e * e + d * d)
    xn, yn = e / n, d / n
    u, v = cx + fx * xn, cy + fy * yn
    bw = fx * side_m / slant
    if noise_px and rng is not None:
        u += rng.normal(0.0, noise_px)
        v += rng.normal(0.0, noise_px)
        bw += rng.normal(0.0, noise_px)
    return (u - bw / 2.0, v - bw / 2.0, bw, bw)


def guidance(**cfgkw):
    # Zero camera mount offset/tilt (GuidanceConfig's own defaults are NOT
    # zero -- 0.10 m fwd, 0.05 m up, ADR-0011 FPV profile) so box_for()'s
    # "identity attitude, zero offset -> exact NED reconstruction" derivation
    # holds exactly in these unit tests.
    gcfg = GuidanceConfig(mount_fwd_m=0.0, mount_left_m=0.0, mount_up_m=0.0,
                          mount_up_rad=0.0)
    cfg = TagTerminalConfig(**cfgkw)
    return TagInterceptGuidance(cfg, cam(), 1.0, gcfg), cfg, gcfg


# ============================================================ intercept solve


def test_solve_intercept_stationary_target():
    r = np.array([10.0, 0.0, 0.0])
    v_t = np.zeros(3)
    t_go = solve_intercept_t_go(r, v_t, 5.0)
    assert t_go == pytest.approx(2.0, abs=1e-9)


def test_solve_intercept_crossing_target_matches_hand_solved_case():
    """r=(10,0,0), v_t=(0,3,0), v=5 -> analytically t_go=2.5 (checked by hand:
    |r + v_t*t| = |(10, 7.5, 0)| = 12.5 = 5*2.5)."""
    r = np.array([10.0, 0.0, 0.0])
    v_t = np.array([0.0, 3.0, 0.0])
    t_go = solve_intercept_t_go(r, v_t, 5.0)
    assert t_go == pytest.approx(2.5, abs=1e-6)
    lead = r + v_t * t_go
    assert np.linalg.norm(lead) == pytest.approx(5.0 * t_go, abs=1e-6)


def test_solve_intercept_no_solution_when_target_outruns_interceptor():
    """A target already receding FASTER than the commanded speed can never be
    caught -- both quadratic roots are negative, so the solve returns None."""
    r = np.array([10.0, 0.0, 0.0])
    v_t = np.array([20.0, 0.0, 0.0])
    assert solve_intercept_t_go(r, v_t, 5.0) is None


def test_solve_intercept_no_solution_at_exactly_matched_speed():
    """Target receding at EXACTLY the commanded speed (a == 0 branch): the gap
    never closes -> None."""
    r = np.array([10.0, 0.0, 0.0])
    v_t = np.array([5.0, 0.0, 0.0])
    assert solve_intercept_t_go(r, v_t, 5.0) is None


# ================================================================ frame math


def test_optical_to_ned_azimuth_matches_derotate_bearing_lambda():
    """`_optical_vec_to_ned`'s horizontal direction must agree EXACTLY with the
    audited `derotate_bearing_lambda` -- they run the same transform chain, so
    if they ever diverge it is a real bug, not a design choice."""
    quat = (0.9238795, 0.0, 0.0, 0.3826834)   # 45 deg yaw
    meas_xyz = (0.3, -0.1, 1.0)               # some off-axis camera ray
    bearing = math.atan2(meas_xyz[0], meas_xyz[2])
    for mount_up in (0.0, math.radians(15.0)):
        lam = derotate_bearing_lambda(meas_xyz, bearing, quat, 0.0, mount_up_rad=mount_up)
        ned = _optical_vec_to_ned(meas_xyz, quat, mount_up)
        assert math.atan2(ned[1], ned[0]) == pytest.approx(lam, abs=1e-9)


def test_box_for_reconstructs_exact_ned_under_identity_attitude():
    gcfg = GuidanceConfig()
    from flight.deploy.seeker_loop import measurement_from_box
    b = box_for(12.0, 3.0, -1.5, side_m=1.0)
    _bearing, _range, meas_xyz = measurement_from_box(b, gcfg, cam(), 1.0)
    ned = _optical_vec_to_ned(meas_xyz, IDENT, 0.0)
    assert ned == pytest.approx([12.0, 3.0, -1.5], abs=1e-6)


def test_cam_offset_ned_matches_camera_to_cg_los_horizontal_lever_arm():
    from flight.geometry import camera_to_cg_los
    quat = (0.9659258, 0.0, 0.0, 0.258819)   # 30 deg yaw
    offset_body = (0.10, 0.0, 0.05)          # fwd, left, up
    range_m, lam = 8.0, math.radians(20.0)
    range_cg, lam_cg = camera_to_cg_los(range_m, lam, quat, offset_body)
    tn_cg, te_cg = range_cg * math.cos(lam_cg), range_cg * math.sin(lam_cg)

    off_ned = _cam_offset_ned(quat, offset_body)
    tn2 = range_m * math.cos(lam) + off_ned[0]
    te2 = range_m * math.sin(lam) + off_ned[1]
    assert (tn2, te2) == pytest.approx((tn_cg, te_cg), abs=1e-9)


# ============================================================ closed-loop step


def test_no_lock_before_first_detection_returns_none():
    g, _, _ = guidance()
    sp, tel = g.step(None, own(), 0.0)
    assert sp is None
    assert tel.detected is False


def test_refuses_to_steer_without_own_attitude():
    """Same own-state precondition as SeekerGuidance, reused via the shared
    GuidanceConfig/own_state_status -- a missing quat must refuse, not steer
    on a fabricated level/yaw-0 stand-in."""
    g, _, _ = guidance()
    bad_own = OwnState(quat=None, psi_rad=None, alt_m=5.0, vel_ned=(0.0, 0.0, 0.0))
    sp, tel = g.step(box_for(10.0, 0.0, 0.0), bad_own, 0.0)
    assert sp is None
    assert tel.own_state_ok is False
    assert "no_own_attitude_quat" in tel.health


def test_vertical_command_sign_target_above_climbs():
    """Target ABOVE own (NED down NEGATIVE) -> commanded v_down must be
    negative (a climb), and vice versa for a target below."""
    g, _, _ = guidance()
    sp, _ = g.step(box_for(10.0, 0.0, -3.0), own(), 0.0)
    assert sp is not None
    assert sp.v_down < 0.0

    g2, _, _ = guidance()
    sp2, _ = g2.step(box_for(10.0, 0.0, 3.0), own(), 0.0)
    assert sp2 is not None
    assert sp2.v_down > 0.0


def test_telemetry_contract_fields_present_for_real_flight():
    """Every field flight.deploy.real_flight._step_engage/_csv_row reads off
    StepTelemetry must exist and be sane on a nominal locked tick."""
    g, _, _ = guidance()
    sp, tel = g.step(box_for(10.0, 0.0, 0.0), own(), 0.0)
    assert sp is not None
    for attr in ("track_broken", "terminal_coast", "own_state_ok", "health",
                "r_hat_m", "setpoint", "stale", "yaw_hold", "coast_age_s"):
        assert hasattr(tel, attr)
    assert tel.track_broken is False
    assert tel.health == []
    assert tel.r_hat_m == pytest.approx(10.0, abs=1e-6)


def test_freeze_inside_freeze_range_locks_the_command():
    g, cfg, _ = guidance(freeze_range_m=1.5)
    sp1, tel1 = g.step(box_for(1.0, 0.0, 0.0), own(), 0.0)
    assert tel1.terminal_coast is True
    assert sp1 is not None
    # A wildly different NEXT detection must NOT move the frozen command.
    sp2, tel2 = g.step(box_for(1.0, 5.0, 2.0), own(), DT)
    assert tel2.terminal_coast is True
    assert sp2.as_tuple() == sp1.as_tuple()


def test_dropout_coasts_then_reports_track_lost():
    g, cfg, _ = guidance(coast_max_s=0.3, freeze_range_m=0.5)
    sp0, _ = g.step(box_for(10.0, 0.0, 0.0), own(), 0.0)
    assert sp0 is not None
    # Dark ticks inside coast_max_s: still commanding.
    sp1, tel1 = g.step(None, own(), DT)
    assert sp1 is not None
    assert "track_lost" not in tel1.health
    sp2, tel2 = g.step(None, own(), 2 * DT)
    assert sp2 is not None
    # Past coast_max_s with no detection: refuses to command.
    sp3, tel3 = g.step(None, own(), 0.5)
    assert sp3 is None
    assert "track_lost" in tel3.health


def test_yaw_command_is_rate_limited():
    g, cfg, _ = guidance(yaw_rate_max_deg_s=30.0)
    sp1, _ = g.step(box_for(10.0, 0.0, 0.0), own(), 0.0)   # target due north
    assert sp1 is not None
    yaw0 = sp1.yaw_deg
    # Next tick: target jumps to due EAST (a 90 deg swing) one control tick
    # later -- the commanded yaw may move by at most yaw_rate_max_deg_s*DT.
    sp2, _ = g.step(box_for(0.1, 10.0, 0.0), own(), DT)
    assert sp2 is not None
    step_deg = abs(((sp2.yaw_deg - yaw0) + 180.0) % 360.0 - 180.0)
    assert step_deg <= cfg.yaw_rate_max_deg_s * DT + 1e-6


def test_estimator_converges_on_noisy_constant_velocity_target():
    """A target crossing at (3, 0) m/s NED (north/east), starting at (15, 0, 0)
    relative to a stationary own vehicle, observed through NOISY synthetic
    boxes (0.1 px 1-sigma on the pixel centre AND the box width) at 20 Hz for
    4 s: the KF's target-velocity estimate must converge close to the true
    (3, 0, 0) m/s. Checks the TRAILING AVERAGE of the last 10 ticks, not one
    sample -- a KF's single-tick gain still responds to instantaneous
    residuals; the quantity a caller trusts is what it settles to."""
    rng = np.random.default_rng(0)
    g, _, _ = guidance(meas_latency_s=0.0)
    o = own((0.0, 0.0, 0.0))
    t = 0.0
    r0 = np.array([15.0, 0.0, 0.0])
    v_true = np.array([3.0, 0.0, 0.0])
    v_hats = []
    sp = None
    for k in range(80):
        r_true = r0 + v_true * t
        b = box_for(float(r_true[0]), float(r_true[1]), float(r_true[2]),
                   noise_px=0.1, rng=rng)
        sp, tel = g.step(b, o, t)
        v_hats.append(g.v_t_hat.copy())
        t += DT
    assert sp is not None
    trailing_mean = np.mean(v_hats[-10:], axis=0)
    assert trailing_mean[:2] == pytest.approx([3.0, 0.0], abs=0.5)
    r_true_final = r0 + v_true * (t - DT)
    assert np.linalg.norm(g.r_hat - r_true_final) < 1.5


# ==================================================== round 2: KF internals


def test_kf_first_correction_is_exact_init():
    """First correction (no predict history) must set r_hat = the raw
    measurement exactly and v_t_hat = the prior -- same contract as round 1's
    alpha-beta init, so single-tick tests (freeze/vertical-sign/etc.) do not
    silently change behaviour under the new estimator."""
    g, _, _ = guidance()
    sp, tel = g.step(box_for(10.0, 2.0, -1.0), own(), 0.0)
    assert sp is not None
    assert g.r_hat == pytest.approx([10.0, 2.0, -1.0], abs=1e-6)
    assert g.v_t_hat == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)


def test_range_measurement_noise_grows_along_los_not_cross_los():
    """Along-LOS (range) noise must grow as range^2; cross-LOS (bearing)
    noise must grow only linearly with range -- so at long range the
    along-LOS variance dominates by far more than the range ratio alone."""
    cfg = TagTerminalConfig()
    c = cam()
    near = range_measurement_noise(4.0, np.array([4.0, 0.0, 0.0]), cfg, c, 0.3)
    far = range_measurement_noise(16.0, np.array([16.0, 0.0, 0.0]), cfg, c, 0.3)
    # direction is along north, so the ALONG-LOS VARIANCE is at [0,0]: sigma
    # ~ range^2 -> variance ~ range^4 -> a 4x range ratio is 4^4=256x. The
    # CROSS-LOS variance is at [1,1]/[2,2]: sigma ~ range -> variance ~
    # range^2 -> 4x range is 4^2=16x.
    assert far[0, 0] / near[0, 0] == pytest.approx(256.0, rel=0.05)
    assert far[1, 1] / near[1, 1] == pytest.approx(16.0, rel=0.05)


# ============================================== round 2: look-angle accel cap


def test_look_angle_bounds_none_without_pitch_or_eps():
    fov = FovHoldConfig(half_vfov_rad=0.6, edge_margin_rad=0.1)
    assert _look_angle_accel_bounds(None, 0.0, fov) == (None, None)
    assert _look_angle_accel_bounds(0.0, None, fov) == (None, None)


def test_look_angle_bounds_symmetric_at_frame_centre():
    """Target dead-centre (eps_v=0), level pitch: the forward (nose-down) and
    braking (nose-up) headroom must be numerically equal by symmetry."""
    fov = FovHoldConfig(half_vfov_rad=0.6, edge_margin_rad=0.1)
    a_min, a_max = _look_angle_accel_bounds(0.0, 0.0, fov)
    assert a_max == pytest.approx(-a_min, rel=1e-9)
    assert a_max > 0.0


def test_look_angle_bounds_shrink_as_target_nears_the_edge_it_would_worsen():
    """Target already pushed toward the TOP edge (eps_v negative, i.e. MORE
    forward accel makes it worse): a_max (forward headroom) must shrink, and
    a_min (braking headroom, which HELPS here) must not shrink the same way."""
    fov = FovHoldConfig(half_vfov_rad=0.6, edge_margin_rad=0.1)
    a_min_c, a_max_c = _look_angle_accel_bounds(0.0, 0.0, fov)
    a_min_top, a_max_top = _look_angle_accel_bounds(-0.3, 0.0, fov)
    assert a_max_top < a_max_c
    assert a_min_top <= a_min_c   # braking headroom does not shrink toward top


def test_pitch_rad_from_quat_matches_euler_convention_nose_down_negative():
    """Pinned against flight.geometry.euler_to_quat_body_to_ned: positive
    pitch there is nose-UP (matches fov_guidance's 'theta negative = nose
    down' convention that pitch_headroom_accel_cap assumes)."""
    for pitch in (0.2, -0.2, 0.0):
        q = euler_to_quat_body_to_ned(0.0, pitch, 0.0)
        assert _pitch_rad_from_quat(q) == pytest.approx(pitch, abs=1e-9)


def test_ned_vec_to_optical_round_trips_with_optical_to_ned():
    quat = (0.8446, 0.1913, 0.4619, 0.1913)   # some non-trivial attitude
    n = math.sqrt(sum(c * c for c in quat))
    quat = tuple(c / n for c in quat)
    for mount_up in (0.0, math.radians(-10.0), math.radians(12.0)):
        optical_in = (0.2, -0.15, 1.0)
        ned = _optical_vec_to_ned(optical_in, quat, mount_up)
        optical_out = _ned_vec_to_optical(ned, quat, mount_up)
        assert optical_out == pytest.approx(optical_in, abs=1e-9)


def test_look_angle_cap_limits_forward_acceleration_when_camera_has_a_height():
    """End-to-end: with a camera that HAS a height (so the cap is active) and
    a narrow FOV, commanding a big forward step from rest must be visibly
    throttled versus the SAME setup with the cap disabled (no height)."""
    gcfg = GuidanceConfig(mount_fwd_m=0.0, mount_left_m=0.0, mount_up_m=0.0,
                          mount_up_rad=0.0)
    narrow_cam = CameraModel(1400.0, 1400.0, 640.0, 400.0, width=1280, height=800)
    cfg = TagTerminalConfig(v_engage_ms=14.0, accel_floor_ms2=0.5)
    g_capped = TagInterceptGuidance(cfg, narrow_cam, 0.3, gcfg)
    g_free = TagInterceptGuidance(cfg, CameraModel(1400.0, 1400.0, 640.0, 400.0),
                                  0.3, gcfg)   # no height -> cap inactive

    def box_for_cam(n, e, d, c, side_m=0.3):
        slant = math.sqrt(n * n + e * e + d * d)
        xn, yn = e / n, d / n
        bw = c.fx * side_m / slant
        u, v = c.cx + c.fx * xn, c.cy + c.fy * yn
        return (u - bw / 2.0, v - bw / 2.0, bw, bw)

    o = own()
    sp_capped, _ = g_capped.step(box_for_cam(20.0, 0.0, 0.0, narrow_cam), o, 0.0)
    sp_free, _ = g_free.step(box_for_cam(20.0, 0.0, 0.0, CameraModel(1400.0, 1400.0, 640.0, 400.0)), o, 0.0)
    assert sp_capped is not None and sp_free is not None
    assert abs(sp_capped.v_north) < abs(sp_free.v_north)
