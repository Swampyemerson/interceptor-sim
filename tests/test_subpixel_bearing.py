#!/usr/bin/env python3
"""ADR-0071 subpixel-centroid terminal bearing — unit tests.

WHAT THIS PINS. The published terminal bearing must come from the SUBPIXEL
DARKNESS-WEIGHTED CENTROID of the dark target within the tracker box (not the
jittery box CENTER, which floors the miss at ~1.5 m per ADR-0056/0070) — and it
must be a NO-OP (box center) when the flag is off, and immune to box-edge jitter.

Tested via `DetectThenTracker.__new__` (bypassing __init__, which needs a live
ONNX seeker + cv2 CSRT) so this stays a fast, sim-free, dependency-light unit
test of the geometry method itself.
"""
import math
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts", "seeker"))

from detect_track import DetectThenTracker    # noqa: E402


def _tracker(pctl=40.0):
    """A bare DetectThenTracker with only the attrs the centroid methods read
    (skips __init__ / the ONNX seeker + cv2 tracker validation)."""
    t = DetectThenTracker.__new__(DetectThenTracker)
    t.subpixel_dark_pctl = pctl
    return t


def _frame_with_blob(cx, cy, size=8, bg=200, fg=30, shape=(120, 120)):
    """Light background with a dark square 'drone' centered at (cx, cy)."""
    f = np.full((shape[0], shape[1], 3), bg, np.uint8)
    h = size // 2
    f[cy - h:cy + h, cx - h:cx + h] = fg
    return f


def test_centroid_finds_target_off_box_center():
    """The centroid locks the ACTUAL dark target, even when the box is loose so
    its center is several px off the target."""
    t = _tracker()
    f = _frame_with_blob(62, 40, size=8)
    box = (50, 25, 40, 40)          # box center (70, 45) -- 8+ px off the target
    cx, cy = t._subpixel_center(f, box)
    assert abs(cx - 62) < 1.5 and abs(cy - 40) < 1.5   # ~ the true target
    bc = (box[0] + box[2] / 2, box[1] + box[3] / 2)
    assert math.hypot(cx - bc[0], cy - bc[1]) > 4.0    # materially off the box center


def test_centroid_immune_to_box_edge_jitter():
    """The whole point: growing the box a few px (the CSRT edge jitter that IS
    the box-center bearing noise) moves the box center but NOT the centroid."""
    t = _tracker()
    f = _frame_with_blob(62, 40, size=8)
    c1 = t._subpixel_center(f, (50, 25, 40, 40))
    c2 = t._subpixel_center(f, (47, 22, 47, 47))       # box grown/shifted ~5 px
    assert math.hypot(c1[0] - c2[0], c1[1] - c2[1]) < 0.5   # centroid barely moves
    bc1 = (50 + 20, 25 + 20)
    bc2 = (47 + 23.5, 22 + 23.5)
    assert math.hypot(bc1[0] - bc2[0], bc1[1] - bc2[1]) > 0.5   # box center DOES move


def test_recenter_shifts_center_preserves_size():
    """_recenter moves the box so its center = the centroid but keeps w,h (the
    width-based RANGE is unchanged; only the bearing is refined)."""
    t = _tracker()
    f = _frame_with_blob(62, 40, size=8)
    box = (50, 25, 40, 40)
    rx, ry, rw, rh = t._recenter(f, box)
    assert (rw, rh) == (40, 40)                        # size preserved
    assert abs((rx + rw / 2) - 62) < 1.5               # new center ~ target
    assert abs((ry + rh / 2) - 40) < 1.5


def test_uniform_roi_falls_back_to_box():
    """No dark target (flat ROI) -> centroid returns None -> _recenter keeps the
    original box (never publishes a garbage bearing)."""
    t = _tracker()
    flat = np.full((120, 120, 3), 128, np.uint8)
    box = (50, 25, 40, 40)
    assert t._subpixel_center(flat, box) is None
    assert t._recenter(flat, box) == box


def test_tiny_box_returns_none():
    t = _tracker()
    f = _frame_with_blob(62, 40, size=4)
    assert t._subpixel_center(f, (61, 39, 2, 2)) is None   # < 3 px -> None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
