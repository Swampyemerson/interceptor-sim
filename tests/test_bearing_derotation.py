#!/usr/bin/env python3
"""FIX A (audit-3 H1): FULL-attitude derotation of the camera LOS bearing.

THE BUG. The pre-fix inertial LOS azimuth was `lambda = psi + beta` -- the
camera bearing beta (a HORIZONTAL azimuth in the optical frame) plus the vehicle
YAW psi. That is correct only when the camera boresight is LEVEL. At the measured
27-36 deg dash pitch (ADR-0060) it inflates the azimuth ~1/cos(pitch) and, under
roll, mixes the target's VERTICAL off-boresight component into the azimuth. lambda
and lambda_dot are the pro-nav input, so this corrupts guidance exactly in the
maneuvering/dash regime.

THE FIX (derotate_bearing_lambda). Rotate the camera ray camera(optical)->body
(FRD)->NED with the full own-state attitude quaternion, then lambda = atan2(e, n).

WHAT THESE TESTS PROVE.
  * IDENTITY: at roll=pitch=0 the derotated lambda == psi + beta to machine
    precision, for any bearing/elevation/yaw -> the validated LEVEL M3/M4/M5
    results are preserved to the bit.
  * FRAME CHAIN: over a grid of (roll, pitch, yaw, ray) the quaternion path
    matches an INDEPENDENT rotation-matrix (Rz@Ry@Rx) computation -> the
    optical->body->NED chain and the quaternion rotation are both correct.
  * PITCH: a known nose pitch inflates the azimuth of an off-boresight target by
    the exact 1/cos law, and provably differs from the pre-fix yaw-only answer.
  * ROLL: a known roll couples a purely-VERTICAL target offset into the azimuth
    (pre-fix lambda == 0), matching the independent computation.
  * FALLBACK: no quaternion (startup) or no bearing -> the pre-fix value, so
    nothing regresses below the old behaviour during the attitude-stream warmup.
  * The module's own --bench derotation self-check passes.

Run: `.venv/bin/python -m pytest tests/test_bearing_derotation.py -v`
"""
import math
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO_ROOT, "scripts")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(SCRIPTS, "seeker"))

from m4_intercept import (                                    # noqa: E402
    derotate_bearing_lambda, _quat_rotate, wrap_pi,
    _euler_to_quat_body_to_ned, bench_derotation_selfcheck,
)


# --------------------------------------------------------------- independent refs
def rot_matrix_body_to_ned(roll, pitch, yaw):
    """Independent body(FRD)->NED rotation matrix, ZYX = Rz(yaw)@Ry(pitch)@Rx(roll).
    Deliberately NOT the quaternion path -- this is the cross-check oracle."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def matvec(M, v):
    return tuple(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2] for i in range(3))


def optical_to_body(ray):
    """optical (x right, y down, z fwd) -> body FRD (fwd, right, down)."""
    rx, ry, rz = ray
    return (rz, rx, ry)


def lambda_via_matrix(ray, roll, pitch, yaw):
    body = optical_to_body(ray)
    ned = matvec(rot_matrix_body_to_ned(roll, pitch, yaw), body)
    return math.atan2(ned[1], ned[0])


def ray_from_bearing_elev(bearing, elev):
    """Camera optical-frame unit ray with horizontal bearing + vertical elevation
    (down positive, matching y_opt)."""
    return (math.sin(bearing) * math.cos(elev),
            math.sin(elev),
            math.cos(bearing) * math.cos(elev))


# ------------------------------------------------------------------------- tests
def test_quat_matches_rotation_matrix():
    """_quat_rotate with a quaternion built from euler == the ZYX matrix."""
    for roll in (-0.7, 0.0, 0.5, 1.0):
        for pitch in (-0.6, 0.0, 0.4, 0.9):
            for yaw in (-2.0, -0.3, 0.0, 1.5, 3.0):
                q = _euler_to_quat_body_to_ned(roll, pitch, yaw)
                M = rot_matrix_body_to_ned(roll, pitch, yaw)
                for v in [(1, 0, 0), (0, 1, 0), (0, 0, 1), (0.3, -0.7, 0.5)]:
                    got = _quat_rotate(q, v)
                    want = matvec(M, v)
                    for g, w in zip(got, want):
                        assert abs(g - w) < 1e-12


def test_identity_at_zero_roll_pitch_preserves_level_results():
    """The load-bearing guarantee: at roll=pitch=0, derotated lambda == psi+beta
    to machine precision (M3/M4/M5 level runs preserved to the bit)."""
    worst = 0.0
    for yaw_d in (-170, -80, -13, 0, 25, 90, 179):
        for bearing_d in (-58, -20, -3, 0, 7, 30, 55):
            for elev_d in (-40, -15, 0, 10, 33):   # elevation must NOT leak into azimuth
                psi = math.radians(yaw_d)
                b = math.radians(bearing_d)
                ray = ray_from_bearing_elev(b, math.radians(elev_d))
                q = _euler_to_quat_body_to_ned(0.0, 0.0, psi)
                got = derotate_bearing_lambda(ray, b, q, psi)
                want = wrap_pi(psi + b)
                worst = max(worst, abs(wrap_pi(got - want)))
    assert worst < 1e-9, f"identity broken, worst err {math.degrees(worst)} deg"


def test_matches_independent_matrix_under_full_attitude():
    """Under arbitrary roll+pitch+yaw the derotation == the independent matrix
    computation (proves the frame chain, not just the level identity)."""
    for roll_d in (-40, -10, 0, 25, 55):
        for pitch_d in (-35, -12, 0, 20, 36):
            for yaw_d in (-120, -30, 0, 45, 160):
                for bearing_d in (-30, -5, 0, 15, 40):
                    for elev_d in (-25, 0, 20):
                        roll, pitch, yaw = map(math.radians, (roll_d, pitch_d, yaw_d))
                        b = math.radians(bearing_d)
                        ray = ray_from_bearing_elev(b, math.radians(elev_d))
                        q = _euler_to_quat_body_to_ned(roll, pitch, yaw)
                        got = derotate_bearing_lambda(ray, b, q, yaw)
                        want = lambda_via_matrix(ray, roll, pitch, yaw)
                        assert abs(wrap_pi(got - want)) < 1e-9


def test_pitch_inflates_azimuth_by_the_one_over_cos_law():
    """Nose-up pitch theta inflates an off-boresight target's azimuth to
    atan(tan(beta)/cos(theta)); the pre-fix yaw-only value is provably wrong."""
    theta = math.radians(30.0)
    b = math.radians(20.0)
    ray = (math.sin(b), 0.0, math.cos(b))     # horizontal off-boresight, elevation 0
    q = _euler_to_quat_body_to_ned(0.0, theta, 0.0)
    got = derotate_bearing_lambda(ray, b, q, 0.0)
    expected = math.atan(math.tan(b) / math.cos(theta))
    assert abs(wrap_pi(got - expected)) < 1e-9
    # and it is a real correction vs the pre-fix yaw-only answer (== beta here)
    assert abs(wrap_pi(got - b)) > math.radians(1.0)


def test_roll_couples_vertical_offset_into_azimuth():
    """A target on the horizontal boresight (beta=0) but BELOW it, seen under a
    known roll, acquires a nonzero azimuth (pre-fix lambda == 0)."""
    roll = math.radians(40.0)
    ray = (0.0, math.tan(math.radians(15.0)), 1.0)   # beta 0, 15 deg down
    q = _euler_to_quat_body_to_ned(roll, 0.0, 0.0)
    got = derotate_bearing_lambda(ray, 0.0, q, 0.0)
    want = lambda_via_matrix(ray, roll, 0.0, 0.0)
    assert abs(wrap_pi(got - want)) < 1e-9
    assert abs(got) > math.radians(1.0)          # pre-fix would have said 0


def test_fallback_no_quaternion_is_prefix_behaviour():
    """Before the attitude stream is up (quat None) the result is exactly the
    old yaw-only lambda = psi + beta -- never worse than before."""
    for psi in (-1.0, 0.0, 0.7):
        for b in (-0.3, 0.0, 0.4):
            assert derotate_bearing_lambda((0.2, 0.1, 1.0), b, None, psi) == psi + b
    # bearing None -> falls back to psi only (no target angle available)
    assert derotate_bearing_lambda(None, None, None, 0.9) == 0.9


def test_meas_xyz_none_uses_bearing_ray_and_is_identity_at_level():
    """When meas_xyz is missing, the ray is synthesized from the bearing (zero
    elevation); still identity at level."""
    psi, b = math.radians(20.0), math.radians(15.0)
    q = _euler_to_quat_body_to_ned(0.0, 0.0, psi)
    got = derotate_bearing_lambda(None, b, q, psi)
    assert abs(wrap_pi(got - (psi + b))) < 1e-9


def test_bench_selfcheck_passes():
    ok, lines = bench_derotation_selfcheck()
    assert ok, "\n".join(lines)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
