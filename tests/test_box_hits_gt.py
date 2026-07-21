#!/usr/bin/env python3
"""Unit tests for the offline hit gate box_scoring.box_hits_gt (constraint
`box-hits-gt-scoring-debt`). PURE geometry -- no cv2 / no onnxruntime -- so it
runs in the main .venv suite (scripts/run_tests.sh stage 1).

Pins the three fixes and, critically, that they only ever RECOVER false-rejects
(never inflate an obvious miss and never break the centred 100%s). The
"measured rows" are the real v2 outputs on data/frametop_sweep/frametop
(conf 0.25) that the scoring debt false-rejected -- reproduce with
scripts/seeker/rescore_box_hits_gt.py.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "seeker"))
from box_scoring import (  # noqa: E402
    box_hits_gt, gt_scale_for, centre_lag_tol_px, TARGET_EXTENT_M, FY, CY,
)

SWEEP_SCALE = gt_scale_for(0.35)      # frame-top sweep labelled at 0.35 m
CX = 640.0

# (range, gt_w=gt_h px @0.35, det box (score,u,v,bw,bh)) -- v2, frame-top,
# target dead-centred horizontally at the +38deg frame-top row (gcy=58).
FRAMETOP_ROWS = [
    (8,  30.0, (0.80, 640, 58, 87.5, 89.7)),   # was a hit even OLD
    (12, 20.0, (0.70, 640, 58, 62.0, 59.7)),   # conf 0.70, centred -> REAL
    (16, 15.0, (0.53, 640, 58, 48.2, 47.4)),   # conf 0.53, centred -> REAL
    (20, 12.0, (0.53, 640, 58, 48.2, 47.4)),   # conf 0.53, centred -> REAL
]
GCY_TOP = 58.0


def _old_hit(boxes, gcx, gcy, gw, gh):
    """The pre-fix gate: fixed 3x, no off-axis, no extent unification."""
    return box_hits_gt(boxes, gcx, gcy, gw, gh, gt_scale=1.0, offaxis_aware=False)


# --------------------------------------------------------------------------
# Defect 1 + 2: the documented false-rejections flip to HITS under the fix.
# --------------------------------------------------------------------------
@pytest.mark.parametrize("rng,gw,det", FRAMETOP_ROWS)
def test_frametop_falsereject_flips_to_hit(rng, gw, det):
    boxes = [det]
    new = box_hits_gt(boxes, CX, GCY_TOP, gw, gw, gt_scale=SWEEP_SCALE,
                      offaxis_aware=True)
    assert new is True, f"r={rng} should be a HIT under the fixed gate"


def test_frametop_12_16_20_were_false_rejected_before():
    """r12/r16/r20 (real, centred, confident) were scored MISS by the OLD gate
    purely on the size ratio -- the near-6th-mirage this fix removes."""
    for rng, gw, det in FRAMETOP_ROWS:
        old = _old_hit([det], CX, GCY_TOP, gw, gw)
        if rng == 8:
            assert old is True          # 8 m passed even before (ratio 2.9)
        else:
            assert old is False, f"r={rng} was NOT a false-reject before?"


def test_r8_was_and_stays_a_hit():
    rng, gw, det = FRAMETOP_ROWS[0]
    assert _old_hit([det], CX, GCY_TOP, gw, gw) is True
    assert box_hits_gt([det], CX, GCY_TOP, gw, gw, gt_scale=SWEEP_SCALE) is True


# --------------------------------------------------------------------------
# No inflation: genuine misses stay misses under the (more lenient) fix.
# --------------------------------------------------------------------------
def test_no_boxes_is_a_miss():
    assert box_hits_gt([], CX, GCY_TOP, 20, 20, gt_scale=SWEEP_SCALE) is False


def test_far_offset_phantom_is_a_miss():
    """A confident box 300 px away from the target is not a hit, fix or no."""
    boxes = [(0.9, 940, 58, 60, 60)]
    assert box_hits_gt(boxes, CX, GCY_TOP, 20, 20, gt_scale=SWEEP_SCALE) is False
    assert _old_hit(boxes, CX, GCY_TOP, 20, 20) is False


def test_blob_guard_survives_the_fix():
    """The Fable round-3 case: a 376x368 blob merely CONTAINING a 60x60 gt is
    still rejected -- even with off-axis widening AND extent unification."""
    blob = [(0.9, 640, 480, 376, 368)]
    assert box_hits_gt(blob, 640, 480, 60, 60) is False               # on-axis
    assert box_hits_gt(blob, 640, 480, 60, 60, gt_scale=gt_scale_for(0.35)) is False
    # even placed at the +38deg frame-top row (max sec^2 widening here)
    assert box_hits_gt([(0.9, 640, 58, 376, 368)], 640, GCY_TOP, 60, 60,
                       gt_scale=gt_scale_for(0.35)) is False


def test_tiny_fragment_is_a_miss():
    """A 2 px fragment on a 30 px gt fails the lower size bound."""
    assert box_hits_gt([(0.9, 640, 480, 2, 2)], 640, 480, 30, 30) is False


# --------------------------------------------------------------------------
# Off-axis correction is a NO-OP on a centred target (never breaks the 100%s).
# --------------------------------------------------------------------------
def test_offaxis_is_noop_at_boresight():
    boxes = [(0.8, 640, 480, 50, 50)]
    on = box_hits_gt(boxes, 640, 480, 30, 30, offaxis_aware=True)
    off = box_hits_gt(boxes, 640, 480, 30, 30, offaxis_aware=False)
    assert on == off is True


def test_centred_hit_never_becomes_a_miss_from_widening():
    """Widening is monotone for a real centred detection: turning the fix on
    can only keep or add a hit, never remove one."""
    boxes = [(0.8, 645, 483, 55, 52)]
    assert box_hits_gt(boxes, 640, 480, 30, 30, offaxis_aware=False) is True
    assert box_hits_gt(boxes, 640, 480, 30, 30, offaxis_aware=True) is True


# --------------------------------------------------------------------------
# Defect 1 geometry: sec^2(theta) magnitude and gt_scale linearity.
# --------------------------------------------------------------------------
def test_sec2_matches_frametop_elevation():
    # gcy=58 at the +38deg row -> sec^2 ~ 1.61
    tan = (GCY_TOP - CY) / FY
    sec2 = 1.0 + tan * tan
    assert 1.58 < sec2 < 1.64


def test_gt_scale_linear():
    assert gt_scale_for(0.35) == pytest.approx(TARGET_EXTENT_M / 0.35)
    assert gt_scale_for(0.9) == pytest.approx(TARGET_EXTENT_M / 0.9)
    assert gt_scale_for(TARGET_EXTENT_M) == pytest.approx(1.0)
    assert gt_scale_for(0.0) == 1.0     # degenerate guard


# --------------------------------------------------------------------------
# Defect 3: moving-capture centre lag rescues a one-frame-stale crosser.
# --------------------------------------------------------------------------
def test_centre_lag_formula():
    # 12 m target crossing at 9 m/s, 20 Hz frames (dt=0.05 s)
    lag = centre_lag_tol_px(9.0, 12.0, 0.05)
    assert lag == pytest.approx(539.936 * (9.0 / 12.0) * 0.05, rel=1e-6)
    assert centre_lag_tol_px(9.0, 0.0, 0.05) == 0.0      # degenerate range
    assert centre_lag_tol_px(9.0, 12.0, 0.0) == 0.0      # degenerate dt


def test_moving_lag_recovers_a_stale_crosser():
    """A detection 40 px behind a fast crosser (target width 30, so base centre
    tolerance = 15 half + 15 tol = 30 px) is a MISS with the static tol but a
    HIT once the measured ~20 px centre-lag is allowed."""
    boxes = [(0.7, 640 - 40, 480, 45, 45)]
    assert box_hits_gt(boxes, 640, 480, 30, 30, centre_lag_px=0.0) is False
    lag = centre_lag_tol_px(9.0, 12.0, 0.05)             # ~20 px
    assert box_hits_gt(boxes, 640, 480, 30, 30, centre_lag_px=lag) is True


def test_static_default_lag_changes_nothing():
    """Default centre_lag_px=0 -> static set-pose numbers are untouched."""
    boxes = [(0.8, 655, 490, 50, 50)]
    a = box_hits_gt(boxes, 640, 480, 30, 30)
    b = box_hits_gt(boxes, 640, 480, 30, 30, centre_lag_px=0.0)
    assert a == b


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
