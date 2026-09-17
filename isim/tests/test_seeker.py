"""Tests for isim.seeker -- geometry signs, the decode surface, the frame
scheduler, latency and determinism.

Frame conventions under test (world NED, body FRD, camera OpenCV z-fwd/x-right/
y-down): a vehicle with the identity quaternion faces NORTH and is level, so
north is camera +z, east is camera +x (right) and DOWN is camera +y.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from isim.seeker import (AprilTagSeeker, CameraParams, DecodeParams, TagParams,
                         UNMEASURED, p_decode, project, quat_to_rot,
                         tag_corners_ned, world_to_camera)
from isim.types import TargetState, VehicleState


def vs(pos, vel=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0), t=0.0) -> VehicleState:
    return VehicleState(t=t, pos_ned=np.array(pos, float),
                        vel_ned=np.array(vel, float), quat_wxyz=q, yaw_rad=0.0)


def ts(pos, vel=(0.0, 0.0, 0.0), t=0.0) -> TargetState:
    return TargetState(t=t, pos_ned=np.array(pos, float),
                       vel_ned=np.array(vel, float))


def q_pitch(deg: float):
    """Body->NED quaternion for a pure pitch. Positive pitch = NOSE UP (a
    positive rotation about body y sends body x to (cos, 0, -sin) in NED, and
    NED z is down, so the nose rises)."""
    h = math.radians(deg) / 2.0
    return (math.cos(h), 0.0, math.sin(h), 0.0)


def q_yaw(deg: float):
    h = math.radians(deg) / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def certain_dec(**kw) -> DecodeParams:
    """Decode params that always decode, with no measurement noise, so the
    geometry can be asserted exactly."""
    base = dict(p_max=1.0, size50_px=0.0, size_k_px=0.5, min_side_px=0.0,
                inc50_deg=1.0e6, inc_k_deg=1.0, blur50_cells=1.0e6,
                pixel_noise_px=0.0)
    base.update(kw)
    return DecodeParams(**base)


def certain_cam(**kw) -> CameraParams:
    base = dict(latency_s=0.0, latency_jitter_s=0.0)
    base.update(kw)
    return CameraParams(**base)


def one_shot(target, own=None, cam=None, tag=None, dec=None, seed=0):
    """Expose a single frame at t=0 and return (detection, report)."""
    skr = AprilTagSeeker(cam or certain_cam(), tag or TagParams(faces_camera=True),
                         dec or certain_dec())
    skr.reset(np.random.default_rng(seed))
    return skr.observe(0.0, own or vs((0.0, 0.0, 0.0)), target)


# --------------------------------------------------------------- geometry

def test_dead_ahead_projects_to_principal_point():
    cam = certain_cam()
    tag = TagParams(faces_camera=True)
    det, rep = one_shot(ts((10.0, 0.0, 0.0)), cam=cam, tag=tag)
    p_cam = world_to_camera((10.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1, 0, 0, 0), cam)
    u, v, in_front = project(p_cam, cam)
    assert in_front
    assert u == pytest.approx(cam.cx, abs=1e-9)
    assert v == pytest.approx(cam.cy, abs=1e-9)
    assert rep.side_px == pytest.approx(cam.fx * tag.side_m / 10.0, rel=1e-12)
    assert det is not None
    assert det.bearing_deg == pytest.approx(0.0, abs=1e-9)
    assert det.elevation_deg == pytest.approx(0.0, abs=1e-9)


def test_target_to_the_right_gives_u_gt_cx_and_positive_bearing():
    # East of a north-facing vehicle -> camera +x -> right of the image.
    cam = certain_cam()
    det, rep = one_shot(ts((10.0, 2.0, 0.0)), cam=cam)
    assert det.u_px > cam.cx
    assert det.bearing_deg > 0.0
    assert det.bearing_deg == pytest.approx(math.degrees(math.atan2(2.0, 10.0)), abs=1e-9)


def test_target_above_gives_v_lt_cy_and_positive_elevation():
    # NED down is negative for "above"; camera +y is DOWN, so the row falls.
    cam = certain_cam()
    det, rep = one_shot(ts((10.0, 0.0, -2.0)), cam=cam)
    assert det.v_px < cam.cy
    assert det.elevation_deg > 0.0
    assert det.elevation_deg == pytest.approx(math.degrees(math.atan2(2.0, 10.0)), abs=1e-9)


def test_mount_tilt_up_moves_a_level_target_down_in_the_image():
    # Pitching the camera up puts the horizon lower in the frame: v grows by
    # fy*tan(theta) for a target at the camera's own altitude.
    theta = 10.0
    flat = certain_cam()
    tilted = certain_cam(mount_tilt_up_deg=theta)
    _, r0 = one_shot(ts((10.0, 0.0, 0.0)), cam=flat)
    d1, _ = one_shot(ts((10.0, 0.0, 0.0)), cam=tilted)
    d0, _ = one_shot(ts((10.0, 0.0, 0.0)), cam=flat)
    assert d1.v_px > d0.v_px
    assert d1.v_px - d0.v_px == pytest.approx(
        tilted.fy * math.tan(math.radians(theta)), rel=1e-9)
    assert d1.elevation_deg == pytest.approx(-theta, abs=1e-9)  # below the axis


def test_vehicle_pitch_nose_down_moves_the_target_up_in_the_image():
    # Nose-down = NEGATIVE pitch (see q_pitch). The camera then looks at the
    # ground, so a level target ahead appears HIGHER in the frame: v shrinks.
    cam = certain_cam()
    level, _ = one_shot(ts((10.0, 0.0, 0.0)), own=vs((0, 0, 0)), cam=cam)
    nose_dn, _ = one_shot(ts((10.0, 0.0, 0.0)),
                          own=vs((0, 0, 0), q=q_pitch(-10.0)), cam=cam)
    assert nose_dn.v_px < level.v_px
    assert nose_dn.v_px - cam.cy == pytest.approx(
        cam.fy * math.tan(math.radians(-10.0)), rel=1e-9)
    assert nose_dn.elevation_deg > 0.0


def test_target_behind_camera_is_not_in_fov_and_yields_no_detection():
    det, rep = one_shot(ts((-10.0, 0.0, 0.0)))
    assert rep.in_fov is False
    assert rep.decoded is False
    assert rep.p_decode == 0.0
    assert det is None


def test_quat_to_rot_is_orthonormal_and_yaw_is_right_handed_about_down():
    r = quat_to_rot(q_yaw(90.0))
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
    # Yaw +90 deg: body forward now points EAST.
    assert np.allclose(r @ np.array([1.0, 0, 0]), np.array([0.0, 1.0, 0.0]), atol=1e-9)


def test_tag_corners_are_a_square_of_the_right_side():
    c = tag_corners_ned((10.0, 0.0, 0.0), (-1.0, 0.0, 0.0), 0.30)
    assert c.shape == (4, 3)
    for i in range(4):
        d = np.linalg.norm(c[i] - c[(i + 1) % 4])
        assert d == pytest.approx(0.30, rel=1e-12)
    assert np.allclose(c.mean(axis=0), np.array([10.0, 0.0, 0.0]), atol=1e-12)


# ---------------------------------------------------------- decode surface

def test_tag_seen_from_behind_never_decodes():
    # Tag normal points north, camera is south of it -> incidence 180 deg.
    tag = TagParams(normal_ned=(1.0, 0.0, 0.0))
    det, rep = one_shot(ts((10.0, 0.0, 0.0)), tag=tag)
    assert rep.incidence_deg == pytest.approx(180.0, abs=1e-6)
    assert rep.p_decode == 0.0
    assert rep.decoded is False
    assert det is None
    assert p_decode(200.0, 180.0, 0.0, DecodeParams()) == 0.0
    assert p_decode(200.0, 90.0, 0.0, DecodeParams()) == 0.0


def test_p_decode_is_monotonic_in_each_argument():
    dp = DecodeParams()
    sizes = [10, 14, 18, 22, 26, 30, 35, 40, 60, 100]
    vals = [p_decode(s, 0.0, 0.0, dp) for s in sizes]
    assert all(b > a for a, b in zip(vals, vals[1:])), vals
    incs = [0, 10, 20, 30, 45, 55, 65, 75, 85]
    vals = [p_decode(120.0, float(i), 0.0, dp) for i in incs]
    assert all(b < a for a, b in zip(vals, vals[1:])), vals
    blurs = [0.0, 1.0, 2.0, 4.0, 8.0, 16.0]
    vals = [p_decode(120.0, 0.0, b, dp) for b in blurs]
    assert all(b < a for a, b in zip(vals, vals[1:])), vals


def test_p_decode_hits_its_design_points():
    dp = DecodeParams()
    assert p_decode(22.0, 0.0, 0.0, dp) == pytest.approx(0.5 * dp.p_max, abs=0.02)
    assert p_decode(35.0, 0.0, 0.0, dp) > 0.92 * dp.p_max
    assert p_decode(120.0, 45.0, 0.0, dp) > 0.9 * dp.p_max     # flat to 45 deg
    assert p_decode(120.0, 78.0, 0.0, dp) < 0.05 * dp.p_max    # dead by ~78 deg
    # 50% loss when the smear is one cell wide.
    one_cell = 120.0 / dp.cells_across
    assert p_decode(120.0, 0.0, one_cell, dp) == pytest.approx(0.5 * dp.p_max, rel=0.02)


def test_corner_outside_the_image_blocks_decode():
    cam = certain_cam()
    x = 10.0
    y = x * (cam.width - 3.0 - cam.cx) / cam.fx     # centre 3 px from the edge
    det, rep = one_shot(ts((x, y, 0.0)), cam=cam)
    assert rep.in_fov is True                        # the CENTRE is on the sensor
    assert rep.p_decode == 0.0                       # but a corner is not
    assert rep.decoded is False
    assert det is None
    # Move it back inside and it decodes with the same params.
    det2, rep2 = one_shot(ts((x, y * 0.9, 0.0)), cam=cam)
    assert rep2.decoded is True


def test_fast_own_yaw_rotation_creates_blur_and_lowers_p_decode():
    cam = CameraParams(fps=100.0, latency_s=0.0, latency_jitter_s=0.0)
    tag = TagParams(faces_camera=True)
    tgt = ts((10.0, 0.0, 0.0))
    rate = 4.0                                        # rad/s of own yaw

    static = AprilTagSeeker(cam, tag, DecodeParams())
    static.reset(np.random.default_rng(1))
    static.observe(0.00, vs((0, 0, 0)), tgt)
    _, r_static = static.observe(0.01, vs((0, 0, 0)), tgt)

    spin = AprilTagSeeker(cam, tag, DecodeParams())
    spin.reset(np.random.default_rng(1))
    spin.observe(0.00, vs((0, 0, 0), q=q_yaw(0.0)), tgt)
    _, r_spin = spin.observe(
        0.01, vs((0, 0, 0), q=q_yaw(math.degrees(rate * 0.01))), tgt)

    assert r_static.blur_px == pytest.approx(0.0, abs=1e-9)
    # ~ fx * omega * exposure for an on-axis target.
    assert r_spin.blur_px == pytest.approx(cam.fx * rate * cam.exposure_s, rel=0.05)
    assert 0.0 < r_spin.p_decode < r_static.p_decode


# ------------------------------------------------------- clock and latency

def test_frame_count_over_ten_seconds_matches_fps():
    cam = CameraParams()
    skr = AprilTagSeeker(cam, TagParams(faces_camera=True), DecodeParams())
    skr.reset(np.random.default_rng(0))
    own, tgt = vs((0, 0, 0)), ts((10.0, 0.0, 0.0))
    n = 0
    for i in range(5001):                              # dt = 2 ms, t: 0 -> 10 s
        _, rep = skr.observe(i * 0.002, own, tgt)
        if rep is not None:
            n += 1
    assert abs(n - round(10.0 * cam.fps)) <= 1, n
    assert n == skr.frames


def test_latency_is_respected_exactly():
    cam = certain_cam(latency_s=0.045, fps=38.2)
    skr = AprilTagSeeker(cam, TagParams(faces_camera=True), certain_dec())
    skr.reset(np.random.default_rng(3))
    own, tgt = vs((0, 0, 0)), ts((10.0, 0.0, 0.0))
    dt, released = 0.005, []
    for i in range(400):
        t = i * dt
        det, _ = skr.observe(t, own, tgt)
        if det is not None:
            released.append((t, det))
    assert released
    for t, det in released:
        # Released on the FIRST step at or after t_available (1 ns of sim-clock
        # slop, because a step time built as i*dt lands a few ULP off).
        assert det.t_available <= t + 1e-9                # never early
        assert det.t_available > t - dt + 1e-9            # first step at/after
        assert det.t_available == pytest.approx(det.t_capture + 0.045, abs=1e-12)


def test_no_detection_is_released_before_its_latency_elapses():
    cam = certain_cam(latency_s=0.10, fps=38.2)
    skr = AprilTagSeeker(cam, TagParams(faces_camera=True), certain_dec())
    skr.reset(np.random.default_rng(4))
    own, tgt = vs((0, 0, 0)), ts((10.0, 0.0, 0.0))
    first = None
    for i in range(200):
        t = i * 0.005
        det, _ = skr.observe(t, own, tgt)
        if det is not None:
            first = t
            break
    assert first is not None and first >= 0.10 - 1e-12


def test_determinism_with_the_same_seed():
    def run(seed):
        cam = CameraParams(fps=50.0)
        skr = AprilTagSeeker(cam, TagParams(faces_camera=True), DecodeParams())
        skr.reset(np.random.default_rng(seed))
        out = []
        for i in range(600):
            t = i * 0.005
            # 7.4 m -> side_px ~ 21.9 px, i.e. right at the 50% decode point,
            # so the draw genuinely matters.
            det, rep = skr.observe(t, vs((0, 0, 0)), ts((7.4, 0.0, 0.0)))
            out.append((rep.decoded if rep else None,
                        None if det is None else (det.u_px, det.range_m)))
        return out, skr.decodes

    a, na = run(11)
    b, nb = run(11)
    c, _ = run(12)
    assert a == b
    assert 0 < na < 300           # genuinely stochastic, not all-or-nothing
    assert a != c


def test_zero_noise_detection_reproduces_truth():
    cam = certain_cam()
    tag = TagParams(faces_camera=True)
    det, rep = one_shot(ts((10.0, 2.0, -3.0)), cam=cam, tag=tag,
                        dec=certain_dec(pixel_noise_px=0.0))
    assert det is not None
    # Camera frame of a north-facing level vehicle: x=east, y=down, z=north.
    assert det.range_m == pytest.approx(math.sqrt(100.0 + 4.0 + 9.0), rel=1e-6)
    assert det.bearing_deg == pytest.approx(math.degrees(math.atan2(2.0, 10.0)), abs=1e-6)
    assert det.elevation_deg == pytest.approx(math.degrees(math.atan2(3.0, 10.0)), abs=1e-6)
    assert det.side_px == pytest.approx(cam.fx * tag.side_m / 10.0, rel=1e-9)


def test_pixel_noise_shows_up_in_the_measurement():
    cam = certain_cam()
    skr = AprilTagSeeker(cam, TagParams(faces_camera=True),
                         certain_dec(pixel_noise_px=0.3))
    skr.reset(np.random.default_rng(7))
    us = []
    for i in range(200):
        det, _ = skr.observe(i * 0.01, vs((0, 0, 0)), ts((10.0, 0.0, 0.0)))
        if det is not None:
            us.append(det.u_px)
    assert len(us) > 50
    assert np.std(us) == pytest.approx(0.3, rel=0.35)
    assert np.mean(us) == pytest.approx(cam.cx, abs=0.15)


def test_range_error_grows_as_range_squared():
    """sigma_range = range^2 * sigma_side / (fx * side): doubling the range
    should quadruple the range scatter."""
    def scatter(rng_m):
        cam = certain_cam()
        skr = AprilTagSeeker(cam, TagParams(faces_camera=True),
                             certain_dec(pixel_noise_px=0.3))
        skr.reset(np.random.default_rng(5))
        r = []
        for i in range(400):
            det, _ = skr.observe(i * 0.01, vs((0, 0, 0)), ts((rng_m, 0.0, 0.0)))
            if det is not None:
                r.append(det.range_m)
        return float(np.std(r))

    s10, s20 = scatter(10.0), scatter(20.0)
    assert s20 / s10 == pytest.approx(4.0, rel=0.3)


def test_unmeasured_names_are_real_fields():
    fields = set(CameraParams.__dataclass_fields__) | set(
        TagParams.__dataclass_fields__) | set(DecodeParams.__dataclass_fields__)
    assert set(UNMEASURED) <= fields
    assert "fps" not in UNMEASURED and "exposure_s" not in UNMEASURED
