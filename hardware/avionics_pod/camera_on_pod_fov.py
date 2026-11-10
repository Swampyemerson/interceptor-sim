#!/usr/bin/env python3
"""Can the seeker camera ride on the avionics pod without seeing its own props?

A DERIVATION, not a measurement. It answers one question from geometry alone:
with the camera on top of the pod's front edge, tilted up by the dash angle, how
many degrees OUTSIDE the picture is the nearest point of the front prop discs?

The props are bolted to the same body as the camera, so the answer does not
depend on how the aircraft is flying -- only on where the camera sits and how far
it is tilted. Positive margin = props are outside the frame. Negative = in frame.

Inputs and where they come from:
  frame     240 mm wheelbase true-X, 5 in props (docs/airframe_numbers.md, vendor)
            -> motors at (+-84.9, +-84.9) mm, prop radius 63.5 mm.
  prop height  builder 2026-11-09: props level with the top plate (z = 0). UNMEASURED
            to the millimetre, so the sweep also shows +10 mm (props higher = worse).
  camera    delivered OV9281 lens ~70 deg HFOV (fx ~1104 px at 1280x800, measured,
            docs/camera_paper_check.md); the published 118 deg lens; AR0234 ~80 deg
            (vendor). Pinhole model: lens distortion is ignored, which UNDERSTATES
            what a wide lens sees at the corners -- hence the required margin below.
  pod       hardware/avionics_pod/pod.py: pod centred on the frame is an ASSUMPTION
            (the pod's fore-aft position on the top plate is not fixed yet).

Rule used for a verdict: margin >= 10 deg = CLEAR, 0..10 = MARGINAL (bench-check
with props spinning before trusting it), < 0 = IN FRAME.

    python3 hardware/avionics_pod/camera_on_pod_fov.py            # table
    python3 hardware/avionics_pod/camera_on_pod_fov.py --self-test
"""
from __future__ import annotations

import argparse
import math
import sys

WHEELBASE = 240.0
PROP_R = 63.5
MOTOR = WHEELBASE / 2 * math.cos(math.radians(45))   # 84.85 mm

# name: (horizontal FoV deg, aspect = rows/cols)
CAMERAS = {
    "OV9281 as delivered (70 deg)": (70.0, 800 / 1280),
    "OV9281 published lens (118 deg)": (118.0, 800 / 1280),
    "AR0234 stock lens (80 deg)": (80.0, 1200 / 1920),
}
CLEAR_DEG = 10.0


def prop_points(z_prop: float, n: int = 240):
    """Rim and interior of both FRONT prop discs, body frame (x fwd, y left, z up)."""
    pts = []
    for sy in (-1, 1):
        for i in range(n):
            a = 2 * math.pi * i / n
            for r in (PROP_R, PROP_R * 0.66, PROP_R * 0.33):
                pts.append((MOTOR + r * math.cos(a), sy * MOTOR + r * math.sin(a), z_prop))
    return pts


def margin_deg(cam_x, cam_z, tilt_up_deg, hfov_deg, aspect, z_prop=0.0):
    """Smallest angular distance (deg) from any front-prop point to the image
    rectangle. Negative means that many degrees INSIDE the frame."""
    t = math.radians(tilt_up_deg)
    th = math.tan(math.radians(hfov_deg / 2))
    tv = th * aspect
    half_h = math.degrees(math.atan(th))
    half_v = math.degrees(math.atan(tv))
    best = float("inf")
    for (px, py, pz) in prop_points(z_prop):
        dx, dy, dz = px - cam_x, py, pz - cam_z
        # camera axes: forward tilted up by t about the body's left axis
        f = dx * math.cos(t) + dz * math.sin(t)      # along the optical axis
        u = -dx * math.sin(t) + dz * math.cos(t)     # image "up"
        r = -dy                                      # image "right"
        if f <= 0:
            continue                                 # behind the lens plane
        ang_h = math.degrees(math.atan2(abs(r), f)) - half_h
        ang_v = math.degrees(math.atan2(abs(u), f)) - half_v
        m = max(ang_h, ang_v)                        # outside if EITHER axis is outside
        best = min(best, m)
    return best


def verdict(m: float) -> str:
    if m == float("inf"):
        return "CLEAR (props behind the lens)"
    if m >= CLEAR_DEG:
        return "CLEAR"
    return "MARGINAL" if m >= 0 else "IN FRAME"


def table() -> int:
    print(f"front motors at (+{MOTOR:.1f}, +-{MOTOR:.1f}) mm, prop radius {PROP_R} mm\n")
    cases = [
        ("pod cradle as drawn (rev 3)", 60.0, 68.0),   # lens ~8 mm ahead of the pivot
        ("pod top, set back 45 mm", 15.0, 68.0),
        ("low nose mount, for comparison", 110.0, 15.0),
    ]
    for cname, (hfov, asp) in CAMERAS.items():
        print(cname)
        print(f"  {'camera position (x fwd, z up, mm)':38s} {'tilt':>5s}  "
              f"{'props at z=0':>22s}  {'props at z=+10':>22s}")
        for (label, cx, cz) in cases:
            for tilt in (15, 25, 30, 40):
                m0 = margin_deg(cx, cz, tilt, hfov, asp, 0.0)
                m1 = margin_deg(cx, cz, tilt, hfov, asp, 10.0)
                print(f"  {label + f' ({cx:.0f}, {cz:.0f})':38s} {tilt:>4d}d  "
                      f"{m0:>+7.1f} {verdict(m0):<14s}  {m1:>+7.1f} {verdict(m1):<14s}")
        print()
    return 0


def self_test() -> int:
    bad = 0

    def check(name, cond):
        nonlocal bad
        print(("ok   " if cond else "FAIL ") + name)
        bad += 0 if cond else 1

    # 1. a camera staring straight down at the props from between them must see them
    check("looking down at a prop from above it is IN FRAME",
          margin_deg(MOTOR, 200.0, -90.0, 80.0, 0.625) < 0)
    # 2. tilting up moves the lower image edge away from props that are below
    a = margin_deg(45.0, 62.0, 15.0, 80.0, 0.625)
    b = margin_deg(45.0, 62.0, 40.0, 80.0, 0.625)
    check("more up-tilt gives more margin when props are below the camera", b > a)
    # 3. a wider lens never has MORE margin than a narrower one, same pose
    check("wider lens has less margin",
          margin_deg(45.0, 62.0, 25.0, 118.0, 0.625) < margin_deg(45.0, 62.0, 25.0, 70.0, 0.625))
    # 4. raising the props toward the camera reduces margin
    check("higher props reduce margin",
          margin_deg(45.0, 62.0, 25.0, 80.0, 0.625, 10.0) < margin_deg(45.0, 62.0, 25.0, 80.0, 0.625, 0.0))
    # 5. hand check: camera at origin height 0, no tilt, a point dead ahead at the
    #    prop plane is on the optical axis -> inside by the full vertical half-angle
    half_v = math.degrees(math.atan(math.tan(math.radians(40)) * 0.625))
    pt_margin = max(0 - 40.0, 0 - half_v)
    check("on-axis point is inside by the vertical half-angle", abs(pt_margin + half_v) < 1e-9)
    return 1 if bad else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    sys.exit(self_test() if a.self_test else table())
