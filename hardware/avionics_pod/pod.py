#!/usr/bin/env python3
"""Avionics pod for the interceptor: Pixhawk 6C Mini + Raspberry Pi 5 + AI HAT+.

Three printed parts, generated from the numbers in the PARAMETERS block:

  dock      stays on the frame's top plate (4 x M3). Front wedge hook, two
            locating pins, rear boss with an M3 heat-set insert.
  pod_base  the removable module. Toe it under the hook, hinge it down onto the
            pins, one M3 screw at the rear. Flight controller on the floor,
            Pi 5 on four M2.5 bosses above it, HAT on the Pi's own standoffs.
  pod_lid   vented roof over the HAT (on 4 x M2.5 standoffs).
  dock_cam + cam_plate_nose   the dock with a camera post off its nose, and the
            tilting plate it carries (chosen 2026-11-10; dock_cam_coupon is its template).

STL files are written in PRINT pose (as the part sits on the bed, see print_pose);
STEP files and the assembly are in flight position.
ONLY the parts named in EXPORT below are written to disk. The plain dock and the
lid-cradle camera option are still built in memory (the mass report and the vibration
model compare against them) but no file of them is ever exported, so a superseded part
cannot be picked up and printed by mistake.

Axes: +x = nose, +y = left, +z = up. z = 0 is the top face of the frame's top
plate. x = 0 is the centre of the Pi board.

    <venv with cadquery>/bin/python hardware/avionics_pod/pod.py [--out DIR]

UNVERIFIED INPUTS are marked  # MEASURE  -- they are datasheet-memory or
guesses, not caliper readings. See README.md.
"""
from __future__ import annotations

import argparse
import json
import math
import os

import cadquery as cq

# The CURRENT design: the only part files this script writes. tests/ checks that
# the folder holds exactly these (plus ASSEMBLY) and nothing older.
EXPORT = ("dock_cam", "cam_plate_nose", "pod_base", "pod_lid", "dock_cam_coupon")
ASSEMBLY = "avionics_pod_assembly_nose_camera"

# ============================== PARAMETERS ==================================
# --- frame interface (TBS Source One V6 top plate) -------------------------
# From TBS's published V6 drawing (So1-V6-2025-AUG-19.dxf, github.com/tbs-trappy/
# source_one; read by the 2026-11-09 gap review, NOT yet checked against the real
# plate): top plate 160.3 x 42.5 mm, but 42.5 wide for only ~40 mm and 29 mm wide
# elsewhere. It has NO 30.5 mm pattern; its only holes are the standoff screws at
# y = +-14.6. So the dock bolts to four of those screws, and overhangs the narrow
# parts of the plate.
DOCK_W = 38.0            # MEASURE  dock width
DOCK_HOLE_DX = 55.77     # fore-aft spacing of the four standoff screws used (drawing;
                         # calipers 2026-11-11 on the target's identical frame: 56.0)
DOCK_HOLE_DY = 29.2      # left-right spacing of those screws (drawing; calipers: 29.3)
DOCK_NARROW_AFT_X = -44.0   # aft of here the dock is only +-10 wide (rear prop tips)
CABLE_SLOT = None        # was (17, 25, 7). The plate's own cut-out turned out to be at the
                         # REAR (pod x -42 to -32, under the FC's tail), so wires come up
                         # beside the 29 mm plate and in through the side windows instead.
DOCK_HOLE_X0 = -8.0      # x of the hole pattern's centre. In TBS's assembled view the four
                         # screws straddle the FRAME CENTRE, so the frame centre is here too.
DOCK_SLOT = 1.0          # each hole is a slot this long (fore-aft) for adjustment
DOCK_T = 3.0
TIE_XS = (-20.0, 6.0)     # zip-tie stations (clear of pins, bolt slots, boss, hook)
TIE_W, TIE_GROOVE = 5.0, 1.4   # fits ties up to 4.8 mm wide, 1.3 mm thick

# --- Pixhawk 6C Mini --------------------------------------------------------
# Model A, current case: bounding box of Holybro's own STEP (6C-MINI-A_rev.stp).
# NOTHING plugs into the top of a Model A. Arrow end: POWER/TELEM1/TELEM2/GPS1/CAN.
# Tail end: servo pin header, AUX5, AUX6, RC IN. Sides: USB-C, debug, I2C, GPS2 /
# DSM, RSSI, microSD. So the plug room this pod must provide is at the ENDS.
FC_L, FC_W, FC_H = 54.3, 39.0, 17.45
FC_X0 = -12.0            # FC centre, fore-aft
FC_ARROW_AFT = True      # as flown since maiden day (SENS_BOARD_ROT=4, yaw 180)
FOAM_T = 2.0             # foam tape under the FC
FC_PLUG_ROOM = 4.0       # air gap over the FC lid (Model A: no top plugs); a
                         # Model B has its servo pins on top and needs ~14 here

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
BOSS_D = 6.6             # 0.2 mm inside the panel's outer face (7.0 was exactly tangent)
INSERT_M25_D, INSERT_M25_DEPTH = 3.2, 5.0   # M2.5 heat-set insert pocket: fits the
#   3.5 mm OD x 4 mm inserts the builder ordered (2026-11-10)
INSERT_M3_D, INSERT_M3_DEPTH = 4.0, 5.5     # M3 heat-set insert pocket
PIN_D, PIN_CLEAR = 4.0, 0.25
FLOOR_X_FRONT, FLOOR_X_REAR = 30.0, -44.0
TAB_X_REAR, TAB_HALF_W = -71.0, 9.0
SCREW_X = -64.0         # ~18 mm of plug-and-bend room behind the FC
HOOK_HALF_W = 12.0
LID_T = 1.6

# --- camera option (rev 3 draft): seeker camera on a tilting cradle on the lid --
# Arducam B0353 (AR0234), read from Arducam's own model 2026-11-10
# (uctronics.com/download/Mechanical_Drawing/B0353.STEP): board 38 x 38, four
# 2.5 mm holes on a 34.04 mm square with 4 mm pads (so M2 screws), lens centred,
# barrel 14 mm across and 19.2 mm tall. On the back: ribbon connector along one
# edge (17.6 wide, 2.05 tall) and the lens holder's two screws 10 mm either side
# of centre. Not yet checked against the delivered board.
CAM_BOARD = 38.0
CAM_HOLE = 34.0
CAM_BOSS_R = 2.0         # posts match the board's 4 mm pads
CAM_WINDOW_HALF = 13.0   # clears the connector and the holder's screw heads
CAM_LENS_D, CAM_LENS_L = 14.0, 19.2   # for the stand-in only
CAM_PIVOT_X, CAM_PIVOT_ABOVE_LID = 52.0, 10.3   # pivot ahead of the USB ports
CAM_TILT_MIN, CAM_TILT_MAX = 10.0, 45.0         # up-tilt range the lock slot allows
CAM_TILT_SHOWN = 25.0    # tilt drawn in the assembly
CAM_LOCK_R = 10.0        # lock screw sits this far behind the pivot
# Cheeks sit OUTBOARD of the lid screws and run the lid's full length as deep
# beams, tied by a bridge bar. The first draft (2.5 mm cheeks off the lid's front
# edge) had its first shake modes at 110-120 Hz in fea_modal.py -- too soft.
CHEEK_T = 3.5
CHEEK_Y_IN = 27.0        # inner face of each cheek (lid screws are at y = 24.5)
CAM_PLATE_HALF_W = CHEEK_Y_IN - 0.5
CORNER_R = 3.0           # rounded plan-view corners on dock, floor and lid

# --- camera option chosen 2026-11-10: a short post off the dock's nose -------
# The dock grows a tongue along the top plate to the frame's two nose standoff
# screws, and two cheeks rise from it to carry a tilting camera plate. Low and
# bolted to the frame, so it does not ride on the Pi/HAT tower.
NOSE_PIVOT_X, NOSE_PIVOT_Z = 65.0, 36.0   # tilt axis (z above the top plate)
NOSE_DX = NOSE_PIVOT_X - 60.0            # the post was first drawn 5 mm further aft
NOSE_CHEEK_Y_IN, NOSE_CHEEK_T = 16.5, 3.5
NOSE_PLATE_HALF_W = 19.7     # camera plate half-width (board is 38 wide)
NOSE_PLATE_X = 5.0           # plate sits this far ahead of the tilt axis
# The plate's NOSE has a full-width screw row 25.0 mm ahead of the dock's front row
# (calipers 2026-11-11: 25.05). The narrow 22 mm pair is at the REAR, by the GPS;
# revision 4 had the plate the wrong way round.
NOSE_SCREW_X = DOCK_HOLE_X0 + DOCK_HOLE_DX / 2 + 25.0     # 44.885
NOSE_SCREW_DY = 29.2         # their left-right spacing (drawing; calipers 29.3)
NOSE_TONGUE_END = 66.9       # top plate's nose edge is at 67.4 (drawing)
NOSE_TONGUE_T = 6.0          # tongue thickness ahead of the pod's nose (dock is 3.0)
NOSE_TONGUE_THICK_X = 40.0   # ... from here forward (full width starts here; the
                             # pod's hook ends at 35.8)
NOSE_LOCK_AT_DOWN = 45.0     # lock screw is straight below the axis at this tilt

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
# Front pins (rev 6, 2026-11-13). The two rear pins sit on one line across the pod, 26 mm
# apart, so their 0.25 mm clearance lets the pod's front edge, 60 mm ahead of them, swing
# about 1 mm each way (geometry; the builder felt the play on the PLA fit print). These two hold the nose
# sideways. Their sockets are slots running FORE-AFT, so they do not fight the rear pins
# and the pod can still hinge down from under the hook. Between the front dock screws.
PINS_FRONT = [(20.0, 6.0), (20.0, -6.0)]
PIN_FRONT_SLOT = 2.5     # extra socket length fore-aft


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def rbox(x0, x1, y0, y1, z0, z1, r=None):
    """Box with its four vertical edges rounded (nicer to hold, no sharp corners)."""
    r = CORNER_R if r is None else r
    return box(x0, x1, y0, y1, z0, z1).edges("|Z").fillet(r)


def nose_arrow(x_tip, z_top, size=7.0, depth=0.6):
    """Shallow arrow sunk into a top face, pointing to the nose (+x)."""
    pts = [(x_tip, 0), (x_tip - size, size * 0.6), (x_tip - size, -size * 0.6)]
    return (cq.Workplane("XY").polyline(pts).close().extrude(depth + 0.5)
            .translate((0, 0, z_top - depth)))


def cyl(x, y, z0, z1, d):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def xz_prism(pts, y0, y1):
    """Extrude an x-z polygon along y."""
    return (cq.Workplane("XZ").polyline(pts).close().extrude(-(y1 - y0))
            .translate((0, y0, 0)))


def nose_lock_local():
    """Lock-screw position on the camera plate, in the plate's own frame."""
    a = math.radians(NOSE_LOCK_AT_DOWN)
    return (-CAM_LOCK_R * math.sin(a), -CAM_LOCK_R * math.cos(a))


def _tilt(x, z, tilt_deg):
    t = math.radians(tilt_deg)
    return (x * math.cos(t) - z * math.sin(t), x * math.sin(t) + z * math.cos(t))


def dock(coupon: bool = False, nose_post: bool = False):
    t = 0.8 if coupon else DOCK_T
    d = rbox(TAB_X_REAR, FLOOR_X_FRONT + 6.0, -DOCK_W / 2, DOCK_W / 2, 0, t)
    if nose_post:
        # tongue: full width under the cheeks, then as narrow as the top plate.
        # Every low edge stays 7 mm or more outside the front prop circles in
        # top view (same gap the frame's own plate leaves).
        w0, w1 = DOCK_W / 2, NOSE_CHEEK_Y_IN + NOSE_CHEEK_T
        half = [(33.0, w0), (40.0, w1), (58.5 + NOSE_DX, w1), (NOSE_TONGUE_END, w1 - 3.0)]
        pts = half + [(x, -y) for (x, y) in reversed(half)]
        d = d.union(cq.Workplane("XY").polyline(pts).close().extrude(t))
        tn = t
        if not coupon:
            # the post stands AHEAD of its two screws, so the tongue between the pod's
            # nose and the post is the spring it nods on: make that part thick
            tn = NOSE_TONGUE_T
            thick = [(NOSE_TONGUE_THICK_X, w1)] + half[2:]
            tp = thick + [(x, -y) for (x, y) in reversed(thick)]
            d = d.union(cq.Workplane("XY").polyline(tp).close().extrude(tn))
        for sy in (-1, 1):
            x, y = NOSE_SCREW_X, sy * NOSE_SCREW_DY / 2
            d = d.cut(cq.Workplane("XY").slot2D(DOCK_SLOT + 3.4, 3.4).extrude(tn + 1)
                      .translate((x, y, -0.5)))
            if not coupon:
                d = d.cut(cq.Workplane("XY").slot2D(DOCK_SLOT + 6.4, 6.4).extrude(1.6 + 1)
                          .translate((x, y, tn - 1.6)))
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
    for sy in (-1, 1):       # narrow tail: keeps the corners away from the rear props
        ya, yb = sorted((sy * 10.0, sy * (DOCK_W / 2 + 1)))
        d = d.cut(box(TAB_X_REAR - 1, DOCK_NARROW_AFT_X, ya, yb, -1, t + 1))
    if CABLE_SLOT:
        d = d.cut(box(CABLE_SLOT[0], CABLE_SLOT[1], -CABLE_SLOT[2], CABLE_SLOT[2], -1, t + 1))
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
            ya, yb = sorted((sy * 16.0, sy * (DOCK_W / 2 + 1)))   # over the plate's strap slots
            d = d.cut(box(gx - TIE_W / 2, gx + TIE_W / 2, ya, yb, -1, DOCK_T + 1))
    d = d.cut(nose_arrow(6.0, DOCK_T))            # which way round the dock goes
    # front wedge hook: 45-degree face, prints without support
    xf = FLOOR_X_FRONT + 0.2
    hook = xz_prism([(xf, Z_DOCK), (xf + 5.8, Z_DOCK), (xf + 5.8, Z_DOCK + 5.5),
                     (xf - 4.0, Z_DOCK + 5.5), (xf - 4.0, Z_DOCK + 4.0)],
                    -HOOK_HALF_W, HOOK_HALF_W)
    d = d.union(hook)
    for (x, y) in PINS + PINS_FRONT:
        d = d.union(cyl(x, y, Z_DOCK, Z_DOCK + FLOOR_T - 0.2, PIN_D))
    boss_top = Z_DOCK + 5.0
    d = d.union(cyl(SCREW_X, 0, Z_DOCK, boss_top, 9.0))
    d = d.cut(cyl(SCREW_X, 0, boss_top - INSERT_M3_DEPTH, boss_top + 1, INSERT_M3_D))
    if nose_post:
        d = d.union(_nose_cheeks())
    return d


def _nose_cheeks():
    """Two upright cheeks + two low cross walls = an open-topped box on the tongue."""
    px, pz = NOSE_PIVOT_X, NOSE_PIVOT_Z
    yi, yo = NOSE_CHEEK_Y_IN, NOSE_CHEEK_Y_IN + NOSE_CHEEK_T
    # rear edge leans forward as it rises (room for the pod to hinge in); front
    # edge leans forward too, under 45 degrees, so it prints without support
    prof = [(44.0 + NOSE_DX, DOCK_T - 0.5), (58.5 + NOSE_DX, DOCK_T - 0.5), (px + 4.0, 12.0),
            (px + 4.0, pz + 1.5), (px + 1.5, pz + 4.0), (px - 1.5, pz + 4.0),
            (px - 4.0, pz + 1.5), (px - 11.0, pz - 6.0)]
    out = None
    lx, lz = nose_lock_local()
    for sy in (-1, 1):
        ya, yb = sorted((sy * yi, sy * yo))
        c = xz_prism(prof, ya, yb)
        c = c.cut(cq.Workplane("XZ").center(px, pz).circle(1.7).extrude(-80)
                  .translate((0, -40, 0)))
        n = 24
        for i in range(n + 1):              # arc slot for the tilt-lock screw
            dx, dz = _tilt(lx, lz, CAM_TILT_MIN + (CAM_TILT_MAX - CAM_TILT_MIN) * i / n)
            c = c.cut(cq.Workplane("XZ").center(px + dx, pz + dz).circle(1.7).extrude(-80)
                      .translate((0, -40, 0)))
        out = c if out is None else out.union(c)
    # rear wall: leans with the cheeks' rear edge and ties them together up to
    # 2 mm under the tilting plate's ears (a low wall left the cheeks free to sway)
    out = out.union(xz_prism([(44.0 + NOSE_DX, DOCK_T - 0.5), (46.8 + NOSE_DX, DOCK_T - 0.5),
                              (px - 9.3, pz - 12.0), (px - 12.1, pz - 12.0)],
                             -yi - 0.5, yi + 0.5))
    out = out.union(box(56.0 + NOSE_DX, 58.5 + NOSE_DX, -yi - 0.5, yi + 0.5, DOCK_T - 0.5, 10.0))   # front wall
    return out


def pod_base():
    b = box(FLOOR_X_REAR, FLOOR_X_FRONT, -PI_W / 2, PI_W / 2, Z_DOCK, Z_FLOOR)
    b = b.union(rbox(TAB_X_REAR, FLOOR_X_REAR + 4.0, -TAB_HALF_W, TAB_HALF_W,
                     Z_DOCK, Z_FLOOR))
    for sy in (-1, 1):   # zip-tie slots: strain relief for the harness leaving aft
        b = b.cut(box(FLOOR_X_REAR - 6.0, FLOOR_X_REAR - 3.0, sy * 6.5 - 1.0,
                      sy * 6.5 + 1.0, Z_DOCK - 1, Z_FLOOR + 1))
    # 45-degree toe on the front edge, mates with the dock hook
    toe = xz_prism([(FLOOR_X_FRONT - FLOOR_T, Z_FLOOR), (FLOOR_X_FRONT + 1, Z_FLOOR),
                    (FLOOR_X_FRONT + 1, Z_DOCK - 1), (FLOOR_X_FRONT, Z_DOCK)],
                   -PI_W / 2 - 1, PI_W / 2 + 1)
    b = b.cut(toe)
    # side panels with a low, wide cable window each (chamfered top = no supports)
    # panels run 1.2 mm past the end bosses: ending level with them left a 0.2 mm
    # sliver face that SOLIDWORKS could not mesh (2026-11-10)
    px0, px1 = HOLE_XS[0] - BOSS_D / 2 - 1.2, HOLE_XS[1] + BOSS_D / 2 + 1.2
    wx0, wx1 = FC_X0 - FC_L / 2 + 3.0, FC_X0 + FC_L / 2 - 3.0
    wz_top = Z_PI - 6.0
    wz_flat = wz_top - 1.5
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
    # wires up from the stack below (ESC signal, receiver) pass through the floor
    if CABLE_SLOT:
        b = b.cut(box(CABLE_SLOT[0], CABLE_SLOT[1], -CABLE_SLOT[2], CABLE_SLOT[2],
                      Z_DOCK - 1, Z_FLOOR + 1))
    # the Pixhawk's USB-C is high on its right side: a plug body reaches z ~24.4,
    # above the normal window top, so the right panel gets a taller notch there
    b = b.cut(box(FC_X0 - 20.0, FC_X0 - 1.0, -PI_W / 2 - 1, -PANEL_Y_IN + 1,
                  Z_FLOOR + 6.0, Z_PI - 2.5))
    # FC corner fences (locate the foam-taped FC; open everywhere else)
    for sx in (-1, 1):
        fx = FC_X0 + sx * (FC_L / 2 + 0.6)
        fx0, fx1 = sorted((fx, fx + sx * 1.6))
        for sy in (-1, 1):
            ya, yb = sorted((sy * (FC_W / 2 - 9.0), sy * (FC_W / 2 - 1.0)))
            b = b.union(box(fx0, fx1, ya, yb, Z_FLOOR, Z_FC0 + 1.5))   # low: clear of the end plugs
    # sockets for the dock's locating pins: one round, one slotted (no over-constraint)
    (x, y) = PINS[0]
    b = b.cut(cyl(x, y, Z_DOCK - 1, Z_FLOOR + 1, PIN_D + 2 * PIN_CLEAR))
    (x, y) = PINS[1]
    b = b.cut(cq.Workplane("XY").slot2D(PIN_D + 2 * PIN_CLEAR + 1.5, PIN_D + 2 * PIN_CLEAR, 90)
              .extrude(FLOOR_T + 2).translate((x, y, Z_DOCK - 1)))
    for (x, y) in PINS_FRONT:       # fore-aft slots: hold the nose sideways only
        b = b.cut(cq.Workplane("XY").slot2D(PIN_D + 2 * PIN_CLEAR + PIN_FRONT_SLOT,
                                            PIN_D + 2 * PIN_CLEAR, 0)
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


def cam_pivot():
    return (CAM_PIVOT_X, Z_LID + LID_T + CAM_PIVOT_ABOVE_LID)


def pod_lid(camera: bool = False):
    x0, x1 = PI_X_SD, PI_X_SD + HAT_L
    lid_hw = (CHEEK_Y_IN + CHEEK_T) if camera else PI_W / 2
    lid = rbox(x0, x1, -lid_hw, lid_hw, Z_LID, Z_LID + LID_T)
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
    lid = lid.cut(nose_arrow(x1 - 1.0, Z_LID + LID_T, size=5.0))
    if not camera:
        return lid
    # --- camera cradle: two cheeks reaching forward over the USB ports ---------
    zt = Z_LID + LID_T
    px, pz = cam_pivot()
    prof = [(x0 + 2.0, zt), (px + 6.0, zt), (px + 8.0, zt + 4.0), (px + 8.0, pz + 5.0),
            (px + 3.0, pz + 8.0), (px - 18.0, pz + 8.0), (x0 + 2.0, zt + 5.0)]
    for sy in (-1, 1):
        ya, yb = sorted((sy * CHEEK_Y_IN, sy * (CHEEK_Y_IN + CHEEK_T)))
        cheek = xz_prism(prof, ya, yb)
        # pivot hole + arc slot for the tilt lock (M3 clearance)
        cheek = cheek.cut(cq.Workplane("XZ").circle(1.7).extrude(-60)
                          .translate((px, -30, pz)))
        n = 24
        for i in range(n + 1):
            th = math.radians(CAM_TILT_MIN + (CAM_TILT_MAX - CAM_TILT_MIN) * i / n)
            cheek = cheek.cut(cq.Workplane("XZ").circle(1.7).extrude(-60).translate(
                (px - CAM_LOCK_R * math.cos(th), -30, pz - CAM_LOCK_R * math.sin(th))))
        lid = lid.union(cheek)
    # cross tie under the cheeks, just ahead of the lid's front edge
    lid = lid.union(box(x1 - 2.0, x1 + 5.0, -CHEEK_Y_IN - CHEEK_T, CHEEK_Y_IN + CHEEK_T,
                        Z_LID, zt))
    # bridge bar between the cheek tops, behind the plate's swing
    lid = lid.union(box(px - 17.0, px - 13.5, -CHEEK_Y_IN - 0.5, CHEEK_Y_IN + 0.5,
                        pz + 2.0, pz + 8.0))
    return lid


def _cam_local_to_body(wp, tilt_deg):
    """Local frame: +x = optical axis, origin = pivot. Tilt up, then move to the pivot."""
    px, pz = cam_pivot()
    return wp.rotate((0, 0, 0), (0, 1, 0), -tilt_deg).translate((px, 0, pz))


def cam_plate_local():
    """Tilting plate the camera board screws to (local frame, see above)."""
    half = CAM_PLATE_HALF_W
    bh = CAM_BOARD / 2 + 2.0
    pl = box(2.0, 4.4, -half, half, -bh, bh)
    pl = pl.cut(box(1.0, 5.4, -CAM_WINDOW_HALF, CAM_WINDOW_HALF, -bh - 1.0, 12.0))   # window, open at the bottom
    #                                   edge for the board's ribbon connector
    for sy in (-1, 1):
        for sz in (-1, 1):
            y, z = sy * CAM_HOLE / 2, sz * CAM_HOLE / 2
            boss = cq.Workplane("YZ").circle(CAM_BOSS_R).extrude(2.5).translate((4.4, y, z))
            pl = pl.union(boss)
            pl = pl.cut(cq.Workplane("YZ").circle(0.9).extrude(8).translate((1.0, y, z)))
        ya, yb = sorted((sy * (half - 7.0), sy * half))
        ear = box(-CAM_LOCK_R - 4.0, 2.0, ya, yb, -4.0, 4.0)
        for ax in (0.0, -CAM_LOCK_R):                             # M3 heat-set inserts
            ys = sy * half
            ear = ear.cut(cq.Workplane("XZ").circle(INSERT_M3_D / 2).extrude(-sy * INSERT_M3_DEPTH)
                          .translate((ax, ys, 0)) if sy > 0 else
                          cq.Workplane("XZ").circle(INSERT_M3_D / 2).extrude(INSERT_M3_DEPTH)
                          .translate((ax, ys + INSERT_M3_DEPTH, 0)))
        pl = pl.union(ear)
    return pl


def cam_plate(tilt_deg=None):
    return _cam_local_to_body(cam_plate_local(),
                              CAM_TILT_SHOWN if tilt_deg is None else tilt_deg)


def cam_ghost(tilt_deg=None):
    """Camera board + lens stand-in, for the assembly only."""
    b = box(6.9, 8.5, -CAM_BOARD / 2, CAM_BOARD / 2, -CAM_BOARD / 2, CAM_BOARD / 2)
    lens = cq.Workplane("YZ").circle(CAM_LENS_D / 2).extrude(CAM_LENS_L).translate((8.5, 0, 0))
    return _cam_local_to_body(b.union(lens),
                              CAM_TILT_SHOWN if tilt_deg is None else tilt_deg)


def _nose_local_to_body(wp, tilt_deg):
    return (wp.rotate((0, 0, 0), (0, 1, 0), -tilt_deg)
            .translate((NOSE_PIVOT_X, 0, NOSE_PIVOT_Z)))


NOSE_EAR_HALF = NOSE_CHEEK_Y_IN - 0.5


def cam_plate_nose_local():
    """Tilting plate for the nose post (local frame: +x optical axis, origin on
    the tilt axis). The window is open at the TOP: mount the board with its
    ribbon connector up, which is the short way to the Pi."""
    half = NOSE_PLATE_HALF_W
    xp = NOSE_PLATE_X
    bh = CAM_BOARD / 2 + 2.0
    pl = box(xp, xp + 2.4, -half, half, -bh, bh)
    pl = pl.cut(box(xp - 1, xp + 3.4, -CAM_WINDOW_HALF, CAM_WINDOW_HALF, -14.0, bh + 1.0))
    for sy in (-1, 1):
        for sz in (-1, 1):
            y, z = sy * CAM_HOLE / 2, sz * CAM_HOLE / 2
            pl = pl.union(cq.Workplane("YZ").circle(CAM_BOSS_R).extrude(2.5)
                          .translate((xp + 2.4, y, z)))
            pl = pl.cut(cq.Workplane("YZ").circle(0.9).extrude(8).translate((xp - 1, y, z)))
    lx, lz = nose_lock_local()
    r = 4.5
    ear_half = NOSE_EAR_HALF
    k = r * math.sqrt(0.5)
    prof = [(xp, r), (0.0, r), (-k, k), (lx - k, lz + k), (lx - k, lz - k),
            (lx + k, lz - r), (xp, lz - r)]
    for sy in (-1, 1):
        ya, yb = sorted((sy * (ear_half - 6.5), sy * ear_half))
        ear = xz_prism(prof, ya, yb)
        for (ax, az) in ((0.0, 0.0), (lx, lz)):               # M3 heat-set inserts
            pocket = (cq.Workplane("XZ").center(ax, az).circle(INSERT_M3_D / 2)
                      .extrude(INSERT_M3_DEPTH))
            ear = ear.cut(pocket.translate((0, ear_half if sy > 0
                                            else -ear_half + INSERT_M3_DEPTH, 0)))
        pl = pl.union(ear)
    return pl


def cam_plate_nose(tilt_deg=None):
    return _nose_local_to_body(cam_plate_nose_local(),
                               CAM_TILT_SHOWN if tilt_deg is None else tilt_deg)


def cam_ghost_nose(tilt_deg=None):
    xb = NOSE_PLATE_X + 4.9
    b = box(xb, xb + 1.6, -CAM_BOARD / 2, CAM_BOARD / 2, -CAM_BOARD / 2, CAM_BOARD / 2)
    lens = (cq.Workplane("YZ").circle(CAM_LENS_D / 2).extrude(CAM_LENS_L)
            .translate((xb + 1.6, 0, 0)))
    return _nose_local_to_body(b.union(lens),
                               CAM_TILT_SHOWN if tilt_deg is None else tilt_deg)


def nose_lens_xz(tilt_deg):
    """Front of the lens stand-in, body frame: the viewpoint used for prop margin."""
    dx, dz = _tilt(NOSE_PLATE_X + 6.5 + CAM_LENS_L, 0.0, tilt_deg)
    return (NOSE_PIVOT_X + dx, NOSE_PIVOT_Z + dz)


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


def assembly(parts, vendor_fc=None, camera=False, nose=False):
    """One STEP for CAD inspection: the three printed parts in flight position,
    plus plain blocks standing in for the boards (envelopes, not real models)."""
    hw = PI_W / 2
    asm = cq.Assembly(name="avionics_pod")
    if nose:
        asm.add(parts["dock_cam"], name="dock_cam", color=cq.Color(0.35, 0.35, 0.38))
        asm.add(parts["cam_plate_nose"], name="cam_plate_nose",
                color=cq.Color(0.85, 0.35, 0.10))
        asm.add(cam_ghost_nose(), name="REF_camera_board_and_lens",
                color=cq.Color(0.2, 0.2, 0.25))
    else:
        asm.add(parts["dock"], name="dock", color=cq.Color(0.35, 0.35, 0.38))
    asm.add(parts["pod_base"], name="pod_base", color=cq.Color(0.95, 0.55, 0.15))
    if camera:
        asm.add(parts["pod_lid_cam"], name="pod_lid_cam", color=cq.Color(0.95, 0.75, 0.30))
        asm.add(parts["cam_plate"], name="cam_plate", color=cq.Color(0.85, 0.35, 0.10))
        asm.add(cam_ghost(), name="REF_camera_board_and_lens",
                color=cq.Color(0.2, 0.2, 0.25))
    else:
        asm.add(parts["pod_lid"], name="pod_lid", color=cq.Color(0.95, 0.75, 0.30))
    fc = None
    if vendor_fc and os.path.exists(vendor_fc):
        # Holybro's model: -x = ARROW end (six small openings), +x = PIN end (one
        # big opening); checked by slicing each end wall. Underside at z = -1.
        raw = cq.importers.importStep(vendor_fc)
        fc = cq.Workplane(obj=cq.Compound.makeCompound(list(raw.vals())))
        fc = fc.translate((-1.95, 0, 1.0))
        if not FC_ARROW_AFT:
            fc = fc.rotate((0, 0, 0), (0, 0, 1), 180)
        fc = fc.translate((FC_X0, 0, Z_FC0))
    if fc is not None:
        asm.add(fc, name="REF_pixhawk_6c_mini_modelA_vendor",
                color=cq.Color(0.15, 0.15, 0.15))
    else:
        asm.add(box(FC_X0 - FC_L / 2, FC_X0 + FC_L / 2, -FC_W / 2, FC_W / 2,
                    Z_FC0, Z_FC0 + FC_H),
                name="REF_pixhawk_6c_mini_envelope", color=cq.Color(0.15, 0.15, 0.15))
    # keep-clear volumes for plugs and cable bends at both ends of the FC
    for nm, xa, xb in (("REF_plug_room_front", FC_X0 + FC_L / 2, FC_X0 + FC_L / 2 + 15.0),
                       ("REF_plug_room_rear", FC_X0 - FC_L / 2 - 15.0, FC_X0 - FC_L / 2)):
        asm.add(box(xa, xb, -FC_W / 2 + 1, FC_W / 2 - 1, Z_FC0 + 2.0, Z_FC0 + FC_H - 1.0),
                name=nm, color=cq.Color(0.9, 0.2, 0.2, 0.35))
    asm.add(box(-PI_L / 2, PI_L / 2, -hw, hw, Z_PI, Z_PI + PI_T),
            name="REF_pi5_board", color=cq.Color(0.10, 0.60, 0.20))
    asm.add(box(PI_L / 2 - 21, PI_L / 2 + 2, -hw + 2, hw - 2, Z_PI + PI_T, Z_PI + 16),
            name="REF_pi5_usb_ethernet_block", color=cq.Color(0.75, 0.75, 0.78))
    asm.add(box(PI_X_SD, PI_X_SD + HAT_L, -hw, hw, Z_HAT, Z_HAT + PI_T),
            name="REF_ai_hat_board", color=cq.Color(0.10, 0.35, 0.70))
    return asm


def print_pose(name, wp):
    """The part as it should sit on the printer bed, lowest face at z = 0. The STL
    files are written in THIS pose so nobody has to re-orient one; the STEP files
    and the assembly stay in flight position for CAD inspection."""
    # pod_lid needs no turning: its flat underside is already down and its two rails
    # are on top. (Earlier notes said "print it upside down"; that was wrong and would
    # have put it on the rails.)
    if name == "cam_plate_nose":               # plate level, screw bosses down, ears up
        wp = cam_plate_nose_local().rotate((0, 0, 0), (0, 1, 0), 90)
    zmin = wp.val().BoundingBox().zmin
    return wp if abs(zmin) < 1e-6 else wp.translate((0, 0, -zmin))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--vendor-fc", default=None,
                    help="path to Holybro's 6C-MINI-A_rev.stp (not committed; see README)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    parts = {"dock": dock(), "pod_base": pod_base(), "pod_lid": pod_lid(),
             "dock_coupon": dock(coupon=True),
             "pod_lid_cam": pod_lid(camera=True), "cam_plate": cam_plate(),
             "dock_cam": dock(nose_post=True), "cam_plate_nose": cam_plate_nose(),
             "dock_cam_coupon": dock(coupon=True, nose_post=True)}
    for name, wp in parts.items():
        solids = wp.val().Solids()
        if len(solids) != 1 or not wp.val().isValid():
            print(f"FAIL: {name} is {len(solids)} solid(s), valid={wp.val().isValid()}")
            return 1
    for name in EXPORT:
        cq.exporters.export(print_pose(name, parts[name]), os.path.join(a.out, f"{name}.stl"),
                            tolerance=0.02, angularTolerance=0.2)
        cq.exporters.export(parts[name], os.path.join(a.out, f"{name}.step"))
    assembly(parts, a.vendor_fc, nose=True).save(os.path.join(a.out, ASSEMBLY + ".step"))
    rep = mass_report({k: v for k, v in parts.items() if not k.endswith("coupon")})
    px, pz = cam_pivot()
    rep["camera_option"] = {"pivot_x_mm": px, "pivot_z_above_top_plate_mm": round(pz, 1),
                            "tilt_range_deg": [CAM_TILT_MIN, CAM_TILT_MAX]}
    rep["nose_camera_option"] = {
        "pivot_x_mm": NOSE_PIVOT_X, "pivot_z_above_top_plate_mm": NOSE_PIVOT_Z,
        "tilt_range_deg": [CAM_TILT_MIN, CAM_TILT_MAX],
        "lens_front_xz_mm": {str(int(t)): [round(v, 1) for v in nose_lens_xz(t)]
                             for t in (CAM_TILT_MIN, CAM_TILT_SHOWN, CAM_TILT_MAX)}}
    with open(os.path.join(a.out, "mass_report.json"), "w") as f:
        json.dump(rep, f, indent=1)
    print(json.dumps(rep, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
