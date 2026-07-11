#!/usr/bin/env python3
"""#target-realism: velocity-derived target orientation — unit tests.

WHAT THIS PINS. The maneuvering-drone orientation the mover streams under
`--orient-to-velocity` must (1) YAW to face the travel direction, (2) BANK INTO
turns (coordinated-turn roll toward the turn center), (3) pitch nose-down with
speed, and (4) fall back to a valid IDENTITY quaternion when nearly stationary.
The rigid-pose default (flag off) is covered structurally: send_pose leaves the
orientation field untouched when quat is None, byte-identical to the pre-fix
behavior (see scripts/m4_target_mover.py send_pose docstring).

Loaded straight from the module file so no live Gazebo / gz-transport is needed.
"""
import importlib.util
import math
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load():
    spec = importlib.util.spec_from_file_location(
        "m4_target_mover", os.path.join(REPO, "scripts", "m4_target_mover.py"))
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except SystemExit:      # module has no argv-free side effects, but be safe
        pass
    return mod


MV = _load()
VO = MV.velocity_orientation


def _yaw(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _roll(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))


def _pitch(q):
    w, x, y, z = q
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    return math.degrees(math.asin(s))


def test_yaw_faces_travel_direction():
    assert abs(_yaw(VO(lambda t: (5.0, 0.0), 0.0)) - 0.0) < 1e-6      # +X -> 0
    assert abs(_yaw(VO(lambda t: (0.0, 5.0), 0.0)) - 90.0) < 1e-6     # +Y -> 90
    assert abs(abs(_yaw(VO(lambda t: (-5.0, 0.0), 0.0))) - 180.0) < 1e-6  # -X


def test_banks_into_the_turn():
    sp = 8.0

    def left(t):        # heading rotating CCW (turning left)
        th = math.radians(30.0) * t
        return (sp * math.cos(th), sp * math.sin(th))

    def right(t):
        th = -math.radians(30.0) * t
        return (sp * math.cos(th), sp * math.sin(th))

    assert _roll(VO(left, 0.0)) > 5.0        # bank left into a left turn
    assert _roll(VO(right, 0.0)) < -5.0      # bank right into a right turn


def test_pitch_is_forward_lean_growing_with_speed():
    slow = _pitch(VO(lambda t: (3.0, 0.0), 0.0))
    fast = _pitch(VO(lambda t: (10.0, 0.0), 0.0))
    assert slow < 0.0 and fast < 0.0         # nose-down (negative pitch)
    assert fast < slow                       # faster => more forward lean


def test_pitch_capped():
    q = VO(lambda t: (100.0, 0.0), 0.0, max_pitch_deg=25.0)
    assert _pitch(q) >= -25.0 - 1e-6         # never exceeds the cap


def test_bank_capped():
    sp = 40.0

    def hard_left(t):
        th = math.radians(400.0) * t         # absurd turn rate -> would exceed cap
        return (sp * math.cos(th), sp * math.sin(th))

    assert _roll(VO(hard_left, 0.0, max_bank_deg=55.0)) <= 55.0 + 1e-6


def test_stationary_is_identity():
    q = VO(lambda t: (0.0, 0.0), 0.0)
    assert q == (1.0, 0.0, 0.0, 0.0)


def test_quat_is_unit_norm():
    for vf in (lambda t: (5.0, 0.0), lambda t: (7.0, 3.0), lambda t: (0.05, 0.0)):
        w, x, y, z = VO(vf, 0.0)
        assert abs(math.sqrt(w * w + x * x + y * y + z * z) - 1.0) < 1e-9


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
