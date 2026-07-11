#!/usr/bin/env python3
"""#target-realism: velocity-derived target orientation — unit tests.

WHAT THIS PINS. The maneuvering-drone orientation the mover streams under
`--orient-to-velocity` must (1) point the model's NOSE (baked local +Y forward)
along the travel direction — the offset-composition correctness check, (2) BANK
opposite ways for left vs right turns (coordinated-turn roll), (3) pitch the nose
DOWN while moving, and (4) fall back to a valid IDENTITY quaternion when nearly
stationary. The ABSOLUTE bank/pitch sign convention (into-turn, nose-down) is
convention-dependent after the model-forward composition and is verified visually
in a GUI/onboard frame before the demo relies on it (ADR-0072); here we pin the
convention-independent invariants.

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
    except SystemExit:
        pass
    return mod


MV = _load()
VO = MV.velocity_orientation
FWD = MV.quat_rotate_forward       # model local +Y (nose) -> world
# model local up (+Z) and right (+X) rotated to world, for bank/pitch checks:
UP = lambda q: MV.quat_rotate_forward(q, (0.0, 0.0, 1.0))


def _nose_heading_deg(q):
    fx, fy, _ = FWD(q)
    return math.degrees(math.atan2(fy, fx))


def _nose_pitch_component(q):
    """z of the world nose vector; < 0 == nose pitched DOWN."""
    return FWD(q)[2]


def test_nose_faces_travel_direction():
    # +Y line (the known-good default pose): nose must point +Y (heading 90)
    assert abs(_nose_heading_deg(VO(lambda t: (0.0, 5.0), 0.0)) - 90.0) < 1.0
    # +X travel: nose points +X (heading 0)
    assert abs(_nose_heading_deg(VO(lambda t: (5.0, 0.0), 0.0)) - 0.0) < 1.0
    # -X travel: nose points -X (heading 180)
    assert abs(abs(_nose_heading_deg(VO(lambda t: (-5.0, 0.0), 0.0))) - 180.0) < 1.0
    # diagonal +X+Y: heading 45
    assert abs(_nose_heading_deg(VO(lambda t: (5.0, 5.0), 0.0)) - 45.0) < 1.0


def test_default_plus_y_line_is_near_identity_pose():
    """The +Y straight line reproduces the KNOWN-GOOD baked pose: nose +Y, level
    (only the small nose-down pitch differs from pure identity)."""
    q = VO(lambda t: (0.0, 5.0), 0.0)
    fx, fy, fz = FWD(q)
    assert fy > 0.9                 # nose essentially +Y
    assert abs(fx) < 0.05           # no sideways crab
    assert fz < 0.0                 # slight nose-down from the speed pitch


def test_banks_into_the_turn():
    """Traveling +Y: a LEFT turn curves toward -X and must roll the lift (up
    vector) toward -X (up.x < 0, INTO the turn); a RIGHT turn toward +X."""
    sp = 8.0

    def left(t):        # heading rotates CCW from +Y toward -X
        th = math.radians(30.0) * t
        return (sp * math.cos(th + math.pi / 2), sp * math.sin(th + math.pi / 2))

    def right(t):       # heading rotates CW from +Y toward +X
        th = -math.radians(30.0) * t
        return (sp * math.cos(th + math.pi / 2), sp * math.sin(th + math.pi / 2))

    up_l = UP(VO(left, 0.0))
    up_r = UP(VO(right, 0.0))
    assert up_l[0] < -0.05      # left turn: lift tilts toward -X (into the turn)
    assert up_r[0] > 0.05       # right turn: lift tilts toward +X (into the turn)


def test_pitch_is_nose_down_and_grows_with_speed():
    slow = _nose_pitch_component(VO(lambda t: (3.0, 0.0), 0.0))
    fast = _nose_pitch_component(VO(lambda t: (10.0, 0.0), 0.0))
    assert slow < 0.0 and fast < 0.0        # nose-down
    assert fast < slow                      # faster => more nose-down


def test_pitch_capped():
    q = VO(lambda t: (100.0, 0.0), 0.0, max_pitch_deg=25.0)
    # nose z component corresponds to sin(pitch); cap at 25 deg
    assert _nose_pitch_component(q) >= -math.sin(math.radians(25.0)) - 1e-6


def test_stationary_is_identity():
    assert VO(lambda t: (0.0, 0.0), 0.0) == (1.0, 0.0, 0.0, 0.0)


def test_quat_is_unit_norm():
    for vf in (lambda t: (5.0, 0.0), lambda t: (7.0, 3.0), lambda t: (0.05, 0.0)):
        w, x, y, z = VO(vf, 0.0)
        assert abs(math.sqrt(w * w + x * x + y * y + z * z) - 1.0) < 1e-9


def test_yaw_offset_is_model_specific():
    """A reskin model with a +X-forward mesh (offset 0) points its nose the OTHER
    way for the same velocity — proving the offset is honoured."""
    q_default = VO(lambda t: (0.0, 5.0), 0.0, model_forward_offset_deg=-90.0)
    q_xfwd = VO(lambda t: (0.0, 5.0), 0.0, model_forward_offset_deg=0.0)
    assert abs(_nose_heading_deg(q_default) - 90.0) < 1.0      # +Y nose faces +Y travel
    # with no offset the +Y-baked-nose mesh would be 90 deg off
    assert abs(_nose_heading_deg(q_default) - _nose_heading_deg(q_xfwd)) > 45.0


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
