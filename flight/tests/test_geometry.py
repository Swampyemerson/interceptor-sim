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
)

TOL = math.radians(0.05)


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
       test_roll_cross_couples_vertical, test_mount_compose, test_fallback_no_quat]

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
