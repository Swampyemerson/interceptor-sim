"""Pin flight.geometry to the SAME assertions as m4_intercept's
bench_derotation_selfcheck: identity at level, pitch inflation, roll cross-
coupling, and #40 mount compose. If this passes, the extracted module is
behaviorally identical to the audited original. Run standalone (exit 0/1) or
under pytest.
"""

import math

from flight.geometry import (
    wrap_pi,
    quat_rotate,
    euler_to_quat_body_to_ned,
    derotate_bearing_lambda,
    camera_to_cg_los,
    parse_cam_fwd_from_sdf,
    check_cam_offset_consistency,
    STOCK_CAM_FWD_M,
)

TOL = math.radians(0.05)

# --- P0.4 camera lever-arm guard fixtures (mirror the real model.sdf format,
#     incl. the encoding=UTF-8 declaration + a comment before <sdf>). ---
_STOCK_SDF = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_mono_cam'>
    <include merge='true'><uri>x500</uri></include>
    <include merge='true'>
      <uri>model://mono_cam</uri>
      <pose>.12 .03 .242 0 0 0</pose>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
    </joint>
  </model>
</sdf>"""
_MOVED_SDF = _STOCK_SDF.replace(".12 .03", ".40 .03")   # props-clearance variant


def test_parse_cam_fwd_from_sdf():
    """The active camera forward-x is read from the CameraJoint pose, through the
    encoding declaration + comment (ElementTree rejects a str with an encoding
    decl -- the parser must bytes-encode). Garbage -> None (guard then skips)."""
    assert abs(parse_cam_fwd_from_sdf(_STOCK_SDF) - 0.12) < 1e-9
    assert abs(parse_cam_fwd_from_sdf(_MOVED_SDF) - 0.40) < 1e-9
    assert abs(parse_cam_fwd_from_sdf(_STOCK_SDF.encode("utf-8")) - 0.12) < 1e-9
    assert parse_cam_fwd_from_sdf("not xml at all") is None
    assert parse_cam_fwd_from_sdf("<sdf><model/></sdf>") is None  # no CameraJoint


def test_cam_offset_guard_matrix():
    """The mismatch matrix (P0.4): stock+0 OK, moved+matching OK, moved+0 FAIL,
    stock+offset FAIL, and the escape hatch forces OK while still reporting the
    raw mismatch."""
    # stock camera (0.12) + no offset -> the sanctioned negligible default -> OK
    c = check_cam_offset_consistency(STOCK_CAM_FWD_M, 0.0)
    assert c.ok and not c.mismatch and c.variant == "stock"
    # moved camera (0.40) + matching offset -> compensated -> OK
    c = check_cam_offset_consistency(0.40, 0.40)
    assert c.ok and not c.mismatch and "moved" in c.variant
    # moved camera (0.40) flown with offset 0 -> uncompensated parallax -> FAIL
    c = check_cam_offset_consistency(0.40, 0.0)
    assert (not c.ok) and c.mismatch and c.detail
    # stock camera (0.12) flown with a large offset -> over-compensation -> FAIL
    c = check_cam_offset_consistency(STOCK_CAM_FWD_M, 0.40)
    assert (not c.ok) and c.mismatch and c.detail
    # escape hatch: the same moved+0 mismatch is ALLOWED but still flagged
    c = check_cam_offset_consistency(0.40, 0.0, allow_mismatch=True)
    assert c.ok and c.mismatch  # ok forced True, mismatch still visible


def test_cam_offset_guard_intermediate_variant():
    """A 0.28 props-clearance variant: matched offset OK, a 0.40 offset (meant
    for the 0.40 model) is a FAIL -- the guard is not stock-vs-one-model only."""
    assert check_cam_offset_consistency(0.28, 0.28).ok
    assert not check_cam_offset_consistency(0.28, 0.40).ok
    assert not check_cam_offset_consistency(0.28, 0.0).ok


def test_lever_arm_zero_offset_identity():
    """No camera offset -> range/LOS returned unchanged (a CG-mounted camera is
    byte-identical; guards every gated M3/M4/S1/S2 path that sets no offset)."""
    q = euler_to_quat_body_to_ned(0.0, math.radians(25.0), math.radians(40.0))
    for rng, lam in ((5.0, 0.2), (12.0, -0.5), (2.0, 1.1)):
        assert camera_to_cg_los(rng, lam, q, (0.0, 0.0, 0.0)) == (rng, lam)
    # missing range/quat also passes through
    assert camera_to_cg_los(None, 0.3, q, (0.4, 0, 0)) == (None, 0.3)
    assert camera_to_cg_los(5.0, 0.3, None, (0.4, 0, 0)) == (5.0, 0.3)


def test_lever_arm_level_geometry():
    """Level camera 0.4 m forward: a target 5 m dead-ahead of the CAMERA is
    5.4 m ahead of the CG; a target due-east of the camera reads slightly
    forward-of-east from the CG (which sits 0.4 m behind)."""
    q = euler_to_quat_body_to_ned(0.0, 0.0, 0.0)  # level, yaw 0 -> body==NED
    r, lam = camera_to_cg_los(5.0, 0.0, q, (0.4, 0.0, 0.0))
    assert abs(r - 5.4) < 1e-9 and abs(lam) < 1e-12
    r2, lam2 = camera_to_cg_los(5.0, math.radians(90.0), q, (0.4, 0.0, 0.0))
    assert abs(r2 - math.hypot(5.0, 0.4)) < 1e-9
    assert math.radians(84.0) < lam2 < math.radians(90.0)  # pulled forward of due-east


def test_lever_arm_pitch_shrinks_horizontal_offset():
    """Under a 30 deg dash pitch the forward body offset tilts down, so its
    NORTH (horizontal) contribution shrinks by cos(pitch) -- the exact reason a
    yaw-only correction is wrong through the dash."""
    pitch = math.radians(30.0)
    q = euler_to_quat_body_to_ned(0.0, pitch, 0.0)
    r, _ = camera_to_cg_los(5.0, 0.0, q, (0.4, 0.0, 0.0))
    assert abs(r - (5.0 + 0.4 * math.cos(pitch))) < 1e-9


def test_identity_level():
    """roll=pitch=0 -> derotated lambda == psi + beta for a grid of
    bearings/elevations/yaws (preserves the validated level M3/M4/M5 runs)."""
    for yaw_d in (0.0, 37.0, -80.0):
        for bearing_d in (0.0, 12.0, -25.0):
            for elev_d in (0.0, 15.0, -20.0):
                psi = math.radians(yaw_d)
                b = math.radians(bearing_d)
                el = math.radians(elev_d)
                mx = (math.sin(b) * math.cos(el), math.sin(el),
                      math.cos(b) * math.cos(el))
                q = euler_to_quat_body_to_ned(0.0, 0.0, psi)
                got = derotate_bearing_lambda(mx, b, q, psi)
                want = wrap_pi(psi + b)
                assert abs(wrap_pi(got - want)) <= TOL, (yaw_d, bearing_d, elev_d)


def test_pitch_inflates_azimuth():
    """A 30 deg nose pitch inflates the azimuth of an off-boresight target;
    the pre-fix yaw-only answer is provably wrong, the derotation is right."""
    pitch = math.radians(30.0)
    b = math.radians(20.0)
    mx = (math.sin(b), 0.0, math.cos(b))
    q = euler_to_quat_body_to_ned(0.0, pitch, 0.0)
    got = derotate_bearing_lambda(mx, b, q, 0.0)
    body = (mx[2], mx[0], mx[1])
    ned = quat_rotate(q, body)
    want = math.atan2(ned[1], ned[0])
    assert abs(wrap_pi(got - want)) <= TOL
    assert abs(wrap_pi(got - b)) > math.radians(1.0)  # differs from yaw-only


def test_roll_cross_couples_vertical():
    """A 40 deg roll mixes a purely-VERTICAL target offset into the azimuth
    (pre-fix lambda == 0; derotated lambda != 0), matching an independent
    rotation-matrix computation."""
    roll = math.radians(40.0)
    mx = (0.0, math.tan(math.radians(15.0)), 1.0)
    q = euler_to_quat_body_to_ned(roll, 0.0, 0.0)
    got = derotate_bearing_lambda(mx, 0.0, q, 0.0)
    body = (mx[2], mx[0], mx[1])
    ned = quat_rotate(q, body)
    want = math.atan2(ned[1], ned[0])
    assert abs(wrap_pi(got - want)) <= TOL
    assert abs(got) > math.radians(1.0)


def test_mount_compose():
    """#40/ADR-0067: a 15 deg up-tilt mount, forward-generated at a rolled/
    pitched dash attitude, is recovered exactly by the mount-composed derotation
    while the zero-mount call is provably wrong."""
    mount = math.radians(15.0)
    alpha = math.radians(20.0)
    roll = math.radians(30.0)
    pitch = math.radians(-30.0)
    q = euler_to_quat_body_to_ned(roll, pitch, 0.0)
    d_ned = (math.cos(alpha), math.sin(alpha), 0.0)
    qc = (q[0], -q[1], -q[2], -q[3])
    b_frd = quat_rotate(qc, d_ned)
    cm, sm = math.cos(-mount), math.sin(-mount)
    cam = (cm * b_frd[0] + sm * b_frd[2], b_frd[1],
           -sm * b_frd[0] + cm * b_frd[2])
    mx = (cam[1], cam[2], cam[0])
    beta = math.atan2(mx[0], mx[2])
    got_c = derotate_bearing_lambda(mx, beta, q, 0.0, mount_up_rad=mount)
    got_u = derotate_bearing_lambda(mx, beta, q, 0.0)
    assert abs(wrap_pi(got_c - alpha)) <= TOL
    assert abs(wrap_pi(got_u - alpha)) > math.radians(2.0)


def test_fallback_no_quat():
    """No attitude yet -> yaw-only fallback lambda = psi + beta (nothing worse
    than the pre-fix path during stream warmup)."""
    got = derotate_bearing_lambda(None, math.radians(10.0), None, math.radians(5.0))
    assert abs(got - math.radians(15.0)) <= 1e-12


ALL = [test_identity_level, test_pitch_inflates_azimuth,
       test_roll_cross_couples_vertical, test_mount_compose, test_fallback_no_quat,
       test_parse_cam_fwd_from_sdf, test_cam_offset_guard_matrix,
       test_cam_offset_guard_intermediate_variant]

if __name__ == "__main__":
    import sys
    failed = 0
    for t in ALL:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
    print(f"{len(ALL) - failed}/{len(ALL)} geometry tests passed")
    sys.exit(1 if failed else 0)
