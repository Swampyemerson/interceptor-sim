#!/usr/bin/env python3
"""Measure the fpv_quad_enemy silhouette extent from its meshes -- the
reproducible DERIVATION behind box_scoring.TARGET_EXTENT_M (0.52 m).

WHY: the box_hits_gt scoring gate compares a detected box to a ground-truth
box whose size is `--extent-m` * intrinsics / range. That extent was 0.35 in
the frame-top sweep and 0.9 in quad_approach -- not comparable, and neither
measured for THIS model (the 0.9 came from the old fpv_target_markerless
billboard). This script pins the number to the actual model geometry so the
scorer has one cited source of truth.

Reads the binary prop STL bounding boxes (reliable) and composes them with the
SDF prop/motor poses to get the silhouette AABB. Pure stdlib, no sim, no cv2.
Exit 0 iff the measured mean-axis span matches box_scoring.TARGET_EXTENT_M
within 0.02 m (a self-test that the cited constant still traces to the mesh).
"""
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MDIR = os.path.join(HERE, "..", "..", "models", "fpv_quad_enemy", "meshes")
PROP_SCALE = 0.8461538461538461            # <scale> on the 1345_prop meshes
# Rotor arm-tip (motor) positions and the flattened prop-mesh origins, copied
# verbatim from models/fpv_quad_enemy/model.sdf (the header's own arithmetic).
MOTOR_XY = [(0.174, 0.174), (-0.174, 0.174), (0.174, -0.174), (-0.174, -0.174)]
PROP_ORIGIN_XY = [(0.152, -0.32038461538461536), (-0.196, 0.02761538461538464),
                  (0.152, 0.02761538461538464), (-0.196, -0.32038461538461536)]
MOTOR_BELL_R = 0.028                        # 5010 bell ~56 mm dia -> r ~0.028 m


def stl_bbox(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        mnx = mny = 1e9
        mxx = mxy = -1e9
        for _ in range(n):
            vals = struct.unpack("<12f", f.read(50)[:48])
            for k in range(1, 4):
                x, y = vals[k * 3], vals[k * 3 + 1]
                mnx, mny = min(mnx, x), min(mny, y)
                mxx, mxy = max(mxx, x), max(mxy, y)
    return mnx, mxx, mny, mxy


def measure():
    mnx, mxx, mny, mxy = stl_bbox(os.path.join(MDIR, "1345_prop_ccw.stl"))
    # prop mesh has no rotation in-SDF (0 0 0), so scale + translate only.
    pmnx, pmxx = mnx * PROP_SCALE, mxx * PROP_SCALE
    pmny, pmxy = mny * PROP_SCALE, mxy * PROP_SCALE
    xs, ys = [], []
    for px, py in PROP_ORIGIN_XY:
        xs += [px + pmnx, px + pmxx]
        ys += [py + pmny, py + pmxy]
    for mx, my in MOTOR_XY:
        xs += [mx - MOTOR_BELL_R, mx + MOTOR_BELL_R]
        ys += [my - MOTOR_BELL_R, my + MOTOR_BELL_R]
    x_span = max(xs) - min(xs)
    y_span = max(ys) - min(ys)
    motor_diag = 0.174 * math.sqrt(2) * 2
    return {
        "x_span_m": x_span,
        "y_span_m": y_span,
        "mean_axis_span_m": (x_span + y_span) / 2,
        "max_axis_span_m": max(x_span, y_span),
        "aabb_diag_m": math.hypot(x_span, y_span),
        "motor_to_motor_diag_m": motor_diag,
    }


def main():
    m = measure()
    print("fpv_quad_enemy silhouette (measured from meshes + SDF poses):")
    print(f"  roll-axis span (X)       = {m['x_span_m']:.3f} m")
    print(f"  prop-blade-axis span (Y) = {m['y_span_m']:.3f} m")
    print(f"  motor-to-motor diagonal  = {m['motor_to_motor_diag_m']:.3f} m")
    print(f"  AABB diagonal            = {m['aabb_diag_m']:.3f} m")
    print(f"  max axis span            = {m['max_axis_span_m']:.3f} m")
    print(f"  MEAN axis span (chosen)  = {m['mean_axis_span_m']:.3f} m")
    sys.path.insert(0, HERE)
    from box_scoring import TARGET_EXTENT_M
    dv = abs(m["mean_axis_span_m"] - TARGET_EXTENT_M)
    ok = dv <= 0.02
    print(f"  box_scoring.TARGET_EXTENT_M = {TARGET_EXTENT_M} m  "
          f"(|mean - const| = {dv:.3f} m, tol 0.02)  {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
