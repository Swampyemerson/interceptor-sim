#!/usr/bin/env python3
"""Avionics pod for the interceptor: Pixhawk 6C Mini + Raspberry Pi 5 + AI HAT+.

Three printed parts, generated from the numbers in the PARAMETERS block:

  dock      stays on the frame's top plate (4 x M3). Front wedge hook, two
            locating pins, rear boss with an M3 heat-set insert.
  pod_base  the removable module. Toe it under the hook, hinge it down onto the
            pins, one M3 screw at the rear. Flight controller on the floor,
            Pi 5 on four M2.5 bosses above it, HAT on the Pi's own standoffs.
  pod_lid   vented roof over the HAT (on 4 x M2.5 standoffs).
  dock_coupon   0.8 mm drilling/fit template of the dock -- PRINT THIS FIRST.

Axes: +x = nose, +y = left, +z = up. z = 0 is the top face of the frame's top
plate. x = 0 is the centre of the Pi board.

    <venv with cadquery>/bin/python hardware/avionics_pod/pod.py [--out DIR]

UNVERIFIED INPUTS are marked  # MEASURE  -- they are datasheet-memory or
guesses, not caliper readings. See README.md.
"""
from __future__ import annotations

import argparse
import json
import os

import cadquery as cq

# ============================== PARAMETERS ==================================
# --- frame interface (TBS Source One V6 top plate) -------------------------
DOCK_W = 38.0            # MEASURE  dock width  (<= flat width of the top plate)
DOCK_HOLE_DX = 30.5      # MEASURE  fore-aft spacing of the 4 M3 holes you will use
DOCK_HOLE_DY = 30.5      # MEASURE  left-right spacing of those holes
DOCK_HOLE_X0 = -8.0      # x of the hole pattern's centre (pod datum, see report)
DOCK_SLOT = 4.0          # each hole is a slot this long (fore-aft) for adjustment
DOCK_T = 3.0
TIE_XS = (-40.0, 18.0)   # zip-tie stations (clear of pins, bolt slots, boss, hook)
TIE_W, TIE_GROOVE = 5.0, 1.4   # fits ties up to 4.8 mm wide, 1.3 mm thick

# --- Pixhawk 6C Mini --------------------------------------------------------
FC_L, FC_W, FC_H = 53.3, 38.4, 16.5   # MEASURE (vendor sheet from memory)
FC_X0 = -8.0             # FC centre, fore-aft
FOAM_T = 2.0             # foam tape under the FC
FC_PLUG_ROOM = 12.0      # height kept free above the FC for plugs + cable bend

# --- Raspberry Pi 5 (official mechanical drawing) --------------------------
PI_L, PI_W, PI_T = 85.0, 56.0, 1.6
PI_HOLE_DX, PI_HOLE_DY = 58.0, 49.0
PI_HOLE_FROM_SD_END = 3.5            # first hole row, measured from the SD end
HAT_STANDOFF = 16.0                  # AI HAT+ ships with 16 mm standoffs
HAT_L = 65.0
LID_STANDOFF = 8.0       # MEASURE  M2.5 standoff above the HAT (clears the module)

# --- printed structure ------------------------------------------------------
FLOOR_T = 2.4
PANEL_T = 2.4
BOSS_D = 7.0
INSERT_M25_D, INSERT_M25_DEPTH = 3.6, 5.0   # M2.5 heat-set insert pocket
INSERT_M3_D, INSERT_M3_DEPTH = 4.0, 5.5     # M3 heat-set insert pocket
PIN_D, PIN_CLEAR = 4.0, 0.25
FLOOR_X_FRONT, FLOOR_X_REAR = 30.0, -44.0
TAB_X_REAR, TAB_HALF_W = -57.0, 9.0
SCREW_X = -50.0
HOOK_HALF_W = 12.0
LID_T = 1.6

# --- masses for the centre-of-mass report (grams) ---------------------------
PETG_G_PER_MM3 = 1.27e-3
PRINT_FILL = 0.9         # walls are mostly solid at these thicknesses
MASS = {"fc": 39.0, "pi": 46.0, "cooler": 25.0, "hat": 20.0, "hardware": 8.0}  # MEASURE
# ============================================================================

Z_DOCK = DOCK_T                               # top of dock = underside of pod floor
Z_FLOOR = Z_DOCK + FLOOR_T                    # top of pod floor
Z_FC0 = Z_FLOOR + FOAM_T
Z_PI = Z_FC0 + FC_H + FC_PLUG_ROOM            # underside of the Pi board
Z_HAT = Z_PI + PI_T + HAT_STANDOFF
Z_LID = Z_HAT + PI_T + LID_STANDOFF
PI_X_SD = -PI_L / 2                           # SD-card end of the Pi points AFT
HOLE_XS = (PI_X_SD + PI_HOLE_FROM_SD_END, PI_X_SD + PI_HOLE_FROM_SD_END + PI_HOLE_DX)
HOLE_YS = (-PI_HOLE_DY / 2, PI_HOLE_DY / 2)
PANEL_Y_IN = PI_W / 2 - PANEL_T               # inner face of each side panel
PINS = [(-30.0, 13.0), (-30.0, -13.0)]


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def cyl(x, y, z0, z1, d):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def xz_prism(pts, y0, y1):
    """Extrude an x-z polygon along y."""
    return (cq.Workplane("XZ").polyline(pts).close().extrude(-(y1 - y0))
            .translate((0, y0, 0)))


def dock(coupon: bool = False):
    t = 0.8 if coupon else DOCK_T
    d = box(TAB_X_REAR, FLOOR_X_FRONT + 6.0, -DOCK_W / 2, DOCK_W / 2, 0, t)
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = DOCK_HOLE_X0 + sx * DOCK_HOLE_DX / 2
            y = sy * DOCK_HOLE_DY / 2
            slot = (cq.Workplane("XY").slot2D(DOCK_SLOT + 3.4, 3.4).extrude(t + 1)
                    .translate((x, y, -0.5)))
            d = d.cut(slot)
            if not coupon:   # countersink for an M3 flat-head, flush under the pod
                cs = (cq.Workplane("XY").slot2D(DOCK_SLOT + 6.4, 6.4).extrude(1.6)
                      .translate((x, y, t - 1.6)))
                d = d.cut(cs)
    if coupon:
        d = d.cut(cyl(0, 0, -1, 2, 2.0))          # x = 0 datum mark
        return d
    # NO-DRILL OPTION: foam tape under the dock + two zip ties round the top plate.
    # Each tie lies in a groove across the dock's top face (so the pod still sits
    # flat) and drops through a notch at each edge.
    for gx in TIE_XS:
        d = d.cut(box(gx - TIE_W / 2, gx + TIE_W / 2, -DOCK_W / 2 - 1, DOCK_W / 2 + 1,
                      DOCK_T - TIE_GROOVE, DOCK_T + 1))
        for sy in (-1, 1):
            ya, yb = sorted((sy * (DOCK_W / 2 - 2.0), sy * (DOCK_W / 2 + 1)))
            d = d.cut(box(gx - TIE_W / 2, gx + TIE_W / 2, ya, yb, -1, DOCK_T + 1))
    # front wedge hook: 45-degree face, prints without support
    xf = FLOOR_X_FRONT + 0.2
    hook = xz_prism([(xf, Z_DOCK), (xf + 5.8, Z_DOCK), (xf + 5.8, Z_DOCK + 5.5),
                     (xf - 4.0, Z_DOCK + 5.5), (xf - 4.0, Z_DOCK + 4.0)],
                    -HOOK_HALF_W, HOOK_HALF_W)
    d = d.union(hook)
    for (x, y) in PINS:
        d = d.union(cyl(x, y, Z_DOCK, Z_DOCK + FLOOR_T - 0.2, PIN_D))
    boss_top = Z_DOCK + 5.0
    d = d.union(cyl(SCREW_X, 0, Z_DOCK, boss_top, 9.0))
    d = d.cut(cyl(SCREW_X, 0, boss_top - INSERT_M3_DEPTH, boss_top + 1, INSERT_M3_D))
    return d


def pod_base():
    b = box(FLOOR_X_REAR, FLOOR_X_FRONT, -PI_W / 2, PI_W / 2, Z_DOCK, Z_FLOOR)
    b = b.union(box(TAB_X_REAR, FLOOR_X_REAR, -TAB_HALF_W, TAB_HALF_W, Z_DOCK, Z_FLOOR))
    # 45-degree toe on the front edge, mates with the dock hook
    toe = xz_prism([(FLOOR_X_FRONT - FLOOR_T, Z_FLOOR), (FLOOR_X_FRONT + 1, Z_FLOOR),
                    (FLOOR_X_FRONT + 1, Z_DOCK - 1), (FLOOR_X_FRONT, Z_DOCK)],
                   -PI_W / 2 - 1, PI_W / 2 + 1)
    b = b.cut(toe)
    # side panels with a low, wide cable window each (chamfered top = no supports)
    px0, px1 = HOLE_XS[0] - BOSS_D / 2, HOLE_XS[1] + BOSS_D / 2
    wx0, wx1 = FC_X0 - FC_L / 2 + 3.0, FC_X0 + FC_L / 2 - 3.0
    wz_flat, wz_top = Z_FLOOR + 13.0, Z_PI - 8.0
    rise = wz_top - wz_flat
    for sy in (-1, 1):
        y0, y1 = sorted((sy * PANEL_Y_IN, sy * PI_W / 2))
        panel = box(px0, px1, y0, y1, Z_FLOOR, Z_PI)
        win = xz_prism([(wx0, Z_FLOOR), (wx1, Z_FLOOR), (wx1, wz_flat),
                        (wx1 - rise, wz_top), (wx0 + rise, wz_top), (wx0, wz_flat)],
                       y0 - 1, y1 + 1)
        b = b.union(panel.cut(win))
        for hx in HOLE_XS:   # Pi bosses, full height, blended into the panel
            b = b.union(cyl(hx, sy * PI_HOLE_DY / 2, Z_FLOOR, Z_PI, BOSS_D))
    for hx in HOLE_XS:
        for hy in HOLE_YS:
            b = b.cut(cyl(hx, hy, Z_PI - INSERT_M25_DEPTH, Z_PI + 1, INSERT_M25_D))
    # FC corner fences (locate the foam-taped FC; open everywhere else)
    for sx in (-1, 1):
        fx = FC_X0 + sx * (FC_L / 2 + 0.6)
        fx0, fx1 = sorted((fx, fx + sx * 1.6))
        for sy in (-1, 1):
            ya, yb = sorted((sy * (FC_W / 2 - 9.0), sy * (FC_W / 2 - 1.0)))
            b = b.union(box(fx0, fx1, ya, yb, Z_FLOOR, Z_FLOOR + 5.0))
    # sockets for the dock's locating pins: one round, one slotted (no over-constraint)
    (x, y) = PINS[0]
    b = b.cut(cyl(x, y, Z_DOCK - 1, Z_FLOOR + 1, PIN_D + 2 * PIN_CLEAR))
    (x, y) = PINS[1]
    b = b.cut(cq.Workplane("XY").slot2D(PIN_D + 2 * PIN_CLEAR + 1.5, PIN_D + 2 * PIN_CLEAR, 90)
              .extrude(FLOOR_T + 2).translate((x, y, Z_DOCK - 1)))
    # rear screw "hat": sits over the dock boss, M3 clamps it down
    boss_top = Z_DOCK + 5.0
    hat = cyl(SCREW_X, 0, Z_DOCK, boss_top + 0.3 + 2.6, 14.0)
    hat = hat.cut(cyl(SCREW_X, 0, Z_DOCK - 1, boss_top + 0.3, 9.6))
    b = b.cut(cyl(SCREW_X, 0, Z_DOCK - 1, Z_FLOOR + 1, 9.6)).union(hat)
    b = b.cut(cyl(SCREW_X, 0, boss_top, boss_top + 5, 3.4))
    # floor: cooling/lightening slots beside the FC pad, and two strap slots
    for sy in (-1, 1):
        b = b.cut(cq.Workplane("XY").slot2D(30, 5).extrude(FLOOR_T + 2)
                  .translate((FC_X0, sy * (FC_W / 2 + 2.6), Z_DOCK - 1)))
    for sx in (-14.0, 14.0):
        b = b.cut(cq.Workplane("XY").slot2D(22, 4, 90).extrude(FLOOR_T + 2)
                  .translate((FC_X0 + sx, 0, Z_DOCK - 1)))
    return b


def pod_lid():
    x0, x1 = PI_X_SD, PI_X_SD + HAT_L
    lid = box(x0, x1, -PI_W / 2, PI_W / 2, Z_LID, Z_LID + LID_T)
    for sy in (-1, 1):                      # two stiffening / roll-over rails on top
        lid = lid.union(box(x0, x1, sy * 14 - 1.2, sy * 14 + 1.2,
                            Z_LID + LID_T, Z_LID + LID_T + 3.0))
    n = 7
    for i in range(n):                      # vent slots across the roof
        x = x0 + 9.0 + i * (HAT_L - 18.0) / (n - 1)
        for (yc, ln) in ((0.0, 20.0), (21.0, 7.0), (-21.0, 7.0)):
            lid = lid.cut(cq.Workplane("XY").slot2D(ln, 3.6, 90).extrude(LID_T + 8)
                          .translate((x, yc, Z_LID - 1)))
    for hx in HOLE_XS:
        for hy in HOLE_YS:
            lid = lid.union(cyl(hx, hy, Z_LID, Z_LID + LID_T, 7.0))
            lid = lid.cut(cyl(hx, hy, Z_LID - 1, Z_LID + LID_T + 1, 2.8))
    return lid


def mass_report(parts):
    items = []
    for name, wp in parts.items():
        s = wp.val()
        c = cq.Shape.centerOfMass(s)
        items.append((name, s.Volume() * PETG_G_PER_MM3 * PRINT_FILL, (c.x, c.y, c.z)))
    comps = [
        ("fc", MASS["fc"], (FC_X0, 0.0, Z_FC0 + FC_H / 2)),
        ("pi", MASS["pi"], (0.0, 0.0, Z_PI + 3.0)),
        ("cooler", MASS["cooler"], (-4.0, 0.0, Z_PI + 9.0)),
        ("hat", MASS["hat"], (PI_X_SD + HAT_L / 2, 0.0, Z_HAT + 2.0)),
        ("hardware", MASS["hardware"], (-10.0, 0.0, Z_PI)),
    ]
    removable = [i for i in items if i[0] in ("pod_base", "pod_lid")] + comps
    m = sum(i[1] for i in removable)
    cg = [sum(i[1] * i[2][k] for i in removable) / m for k in range(3)]
    return {
        "printed_g": {n: round(g, 1) for n, g, _ in items},
        "removable_module_g": round(m, 1),
        "removable_module_cg_mm": [round(v, 1) for v in cg],
        "total_height_above_top_plate_mm": round(Z_LID + LID_T + 3.0, 1),
        "footprint_mm": [round(FLOOR_X_FRONT + 6.2 - TAB_X_REAR, 1), PI_W],
        "levels_mm": {"fc_underside": Z_FC0, "pi_underside": Z_PI,
                      "hat_underside": Z_HAT, "lid_underside": Z_LID},
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()
    parts = {"dock": dock(), "pod_base": pod_base(), "pod_lid": pod_lid(),
             "dock_coupon": dock(coupon=True)}
    for name, wp in parts.items():
        solids = wp.val().Solids()
        if len(solids) != 1 or not wp.val().isValid():
            print(f"FAIL: {name} is {len(solids)} solid(s), valid={wp.val().isValid()}")
            return 1
        cq.exporters.export(wp, os.path.join(a.out, f"{name}.stl"), tolerance=0.02,
                            angularTolerance=0.2)
        cq.exporters.export(wp, os.path.join(a.out, f"{name}.step"))
    rep = mass_report({k: v for k, v in parts.items() if k != "dock_coupon"})
    with open(os.path.join(a.out, "mass_report.json"), "w") as f:
        json.dump(rep, f, indent=1)
    print(json.dumps(rep, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
