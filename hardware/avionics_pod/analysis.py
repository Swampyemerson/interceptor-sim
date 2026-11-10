#!/usr/bin/env python3
"""Whole-aircraft effect of the avionics pod: mass, balance, turning inertia,
vibration bands and heat. Plain arithmetic on stated inputs -- every input is
listed with where it came from, and most are ESTIMATES, so every output is too.

Frame: x forward, y left, z up; z = 0 is the top face of the top plate, which the
builder reports is level with the props (2026-11-09). x = 0 is the pod datum,
ASSUMED to sit at the frame centre.

    python3 hardware/avionics_pod/analysis.py            # prints the report numbers
    python3 hardware/avionics_pod/analysis.py --self-test
Reads mass_report.json and fea_results/*.json from this folder (no CAD needed).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MOTOR_XY = 240.0 / 2 * math.cos(math.radians(45))      # 84.85 mm (240 mm true-X, vendor)

# ---- vertical layout of the frame: ASSUMED from the vendor's standoff/arm sizes
Z_FRAME_MID = -20.0      # centre of the 30 mm standoff bay under the top plate
Z_FRAME_BOTTOM = -40.0   # underside of the lower plate + arms (2 + 30 + 6 + 2 mm)
Z_BATTERY = Z_FRAME_BOTTOM - 3.0 - 19.0   # pad + half of a ~38 mm thick pack
Z_MOTOR = -12.0          # motor + prop centre of mass, a little under the prop plane


def airframe_items():
    """(name, grams, x, y, z) for everything that is NOT the pod or its boards.
    Masses: docs/airframe_numbers.md section 3 (mostly estimates)."""
    it = [("frame", 136.5, 0, 0, Z_FRAME_MID)]
    for sx in (-1, 1):
        for sy in (-1, 1):
            it.append(("motor+prop", 146 / 4, sx * MOTOR_XY, sy * MOTOR_XY, Z_MOTOR))
    it += [("esc", 15, 0, 0, -28), ("pm02", 30, -20, 0, Z_FRAME_MID),
           ("bec", 10, 20, 0, Z_FRAME_MID), ("wiring", 30, 0, 0, -15),
           ("receiver", 5, -40, 0, Z_FRAME_MID), ("buzzer", 3, -40, 0, Z_FRAME_MID),
           ("gps", 15, -70, 0, 10),                    # location NOT designed yet
           ("battery", 254, 0, 0, Z_BATTERY)]
    return it


def configs():
    rep = json.load(open(os.path.join(HERE, "mass_report.json")))
    pg = rep["printed_g"]
    cg = rep["removable_module_cg_mm"]
    pod_removable = ("pod (boards + printed)", rep["removable_module_g"], cg[0], cg[1], cg[2])
    dock = ("dock", pg["dock"], -17, 0, 1.5)
    cam = 15.0
    base = airframe_items()
    # A: the old plan -- boards low on the top plate on a printed tray, camera on a nose boom
    a = base + [("fc", 39, 0, 0, 12), ("pi+cooler+hat", 91, 0, 0, 22),
                ("tray+boom", 35, 40, 0, 10), ("camera on nose boom", cam, 110, 0, 15)]
    # B: pod, camera still on a (lighter) nose boom
    b = base + [pod_removable, dock, ("boom", 15, 80, 0, 10),
                ("camera on nose boom", cam, 110, 0, 15)]
    # C: pod with the camera cradle on its lid
    extra_lid = pg.get("pod_lid_cam", 19.0) - pg["pod_lid"]
    c = base + [pod_removable, dock, ("cradle lid extra", extra_lid, 10, 0, 62),
                ("cam plate", pg.get("cam_plate", 6.0), 54, 0, 68),
                ("camera on pod", cam, 62, 0, 70)]
    # D: pod, camera on the short post off the dock's nose (chosen 2026-11-10)
    post_extra = pg.get("dock_cam", 24.4) - pg["dock"]
    d = base + [pod_removable, dock, ("nose post + tongue", post_extra, 55, 0, 14),
                ("nose cam plate", pg.get("cam_plate_nose", 5.1), 64, 0, 37),
                ("camera on nose post", cam, 72, 0, 40)]
    return {"A  old plan: boards low, camera on nose boom": a,
            "B  pod, camera on nose boom": b,
            "C  pod with camera cradle": c,
            "D  pod, camera on the dock's nose post": d}


def mass_props(items):
    m = sum(i[1] for i in items)
    cg = [sum(i[1] * i[k] for i in items) / m for k in (2, 3, 4)]
    ixx = sum(i[1] * ((i[3] - cg[1]) ** 2 + (i[4] - cg[2]) ** 2) for i in items)   # roll
    iyy = sum(i[1] * ((i[2] - cg[0]) ** 2 + (i[4] - cg[2]) ** 2) for i in items)   # pitch
    izz = sum(i[1] * ((i[2] - cg[0]) ** 2 + (i[3] - cg[1]) ** 2) for i in items)   # yaw
    # g mm^2 -> kg m^2
    return {"mass_g": m, "cg_mm": cg, "ixx": ixx * 1e-9, "iyy": iyy * 1e-9, "izz": izz * 1e-9}


def tape_rocking_hz(mass_g, cg_z_mm, g_mpa, tape_t_mm, length_mm=107.0, width_mm=38.0,
                    body_w_mm=56.0):
    """Side-to-side rocking of the whole pod, taken as rigid, on a foam-tape layer.
    Tape stiffness in squeeze taken as 3 x its shear modulus G."""
    k_theta = 3.0 * g_mpa * (length_mm * width_mm ** 3 / 12.0) / tape_t_mm     # N mm / rad
    inertia = mass_g * 1e-3 * ((cg_z_mm * 1e-3) ** 2 + (body_w_mm * 1e-3) ** 2 / 12.0)
    return math.sqrt(k_theta * 1e-3 / inertia) / (2 * math.pi)


def motor_band():
    """Shaft rate in Hz: docs/airframe_numbers.md section 7 (estimates)."""
    return {"hover": 12800 / 60.0, "full": 35000 / 60.0}


def thermal():
    """Heat in the pod and the air needed to carry it away. Power figures are
    typical values from memory, NOT measured on this build."""
    lo = {"pi5": 6.0, "hailo8": 2.5, "camera": 0.8, "pixhawk": 1.0}
    hi = {"pi5": 9.0, "hailo8": 5.0, "camera": 1.2, "pixhawk": 1.5}
    cp, rho = 1005.0, 1.16            # J/kg/K, kg/m^3 (warm air)
    out = {"watts_lo": sum(lo.values()), "watts_hi": sum(hi.values())}
    for k in ("lo", "hi"):
        w = out["watts_" + k]
        out["litres_per_s_for_10K_rise_" + k] = w / (cp * 10.0) / rho * 1000.0
    # the gap between the Pi and the HAT is the main air path: 56 mm wide x 16 mm
    out["gap_area_cm2"] = 5.6 * 1.6
    out["air_speed_m_s_needed_hi"] = (out["litres_per_s_for_10K_rise_hi"] * 1e-3
                                      / (out["gap_area_cm2"] * 1e-4))
    # still-air conduction from the Pi's warm underside into the Pixhawk's lid
    area = 54.3e-3 * 39.0e-3
    gap = 2.55e-3                     # measured in CAD by the gap review
    out["pi_to_pixhawk_watts_at_20K"] = 0.026 * area * 20.0 / gap
    return out


def report() -> int:
    print("== MASS, BALANCE, TURNING INERTIA (estimates) ==")
    ref = None
    for name, items in configs().items():
        r = mass_props(items)
        if ref is None:
            ref = r
        print(f"{name}\n   mass {r['mass_g']:.0f} g   centre of mass x {r['cg_mm'][0]:+.1f}  "
              f"z {r['cg_mm'][2]:+.1f} mm (z = 0 is the prop plane)\n"
              f"   roll inertia {r['ixx'] * 1e3:.2f}  pitch {r['iyy'] * 1e3:.2f}  "
              f"yaw {r['izz'] * 1e3:.2f}  g m^2   "
              f"(vs A: roll {100 * (r['ixx'] / ref['ixx'] - 1):+.0f}%, "
              f"pitch {100 * (r['iyy'] / ref['iyy'] - 1):+.0f}%, "
              f"yaw {100 * (r['izz'] / ref['izz'] - 1):+.0f}%)")
    print("\n== VIBRATION ==")
    mb = motor_band()
    print(f"motor shaft rate: hover {mb['hover']:.0f} Hz, full throttle {mb['full']:.0f} Hz "
          f"(blade pass = 3x)")
    d = os.path.join(HERE, "fea_results")
    for f in sorted(os.listdir(d)):
        r = json.load(open(os.path.join(d, f)))
        print(f"  {f[:-5]:28s} first modes {r['freqs_hz'][:4]} Hz  "
              f"{[m[0] for m in r['modes'][:4]]}")
    print("  whole pod rocking on foam tape (rigid pod, hand formula):")
    for (m, z, lab) in ((176.0, 27.0, "no camera"), (210.0, 33.0, "with camera")):
        row = [f"G={g} MPa,t={t} mm: {tape_rocking_hz(m, z, g, t):.0f} Hz"
               for (g, t) in ((0.2, 1.0), (0.5, 1.0), (1.0, 1.0), (0.5, 3.0))]
        print(f"    {lab:12s} " + "   ".join(row))
    print("\n== HEAT ==")
    t = thermal()
    print(f"heat in the pod: {t['watts_lo']:.1f} to {t['watts_hi']:.1f} W (typical figures, unmeasured)")
    print(f"air needed for a 10 C rise: {t['litres_per_s_for_10K_rise_lo']:.2f} to "
          f"{t['litres_per_s_for_10K_rise_hi']:.2f} litres/s "
          f"= {t['air_speed_m_s_needed_hi']:.1f} m/s through the {t['gap_area_cm2']:.0f} cm^2 "
          f"Pi-to-HAT gap at the high end")
    print(f"Pi underside -> Pixhawk lid through 2.55 mm of still air, 20 C warmer: "
          f"{t['pi_to_pixhawk_watts_at_20K']:.2f} W")
    return 0


def self_test() -> int:
    bad = 0

    def check(name, cond):
        nonlocal bad
        print(("ok   " if cond else "FAIL ") + name)
        bad += 0 if cond else 1

    two = [("a", 100, 0, 0, 0), ("b", 100, 0, 100, 0)]
    r = mass_props(two)
    check("two equal masses 100 mm apart: centre midway", abs(r["cg_mm"][1] - 50) < 1e-9)
    check("... roll inertia = 2 * 0.1 kg * (0.05 m)^2", abs(r["ixx"] - 2 * 0.1 * 0.05 ** 2) < 1e-12)
    check("stiffer tape rocks faster",
          tape_rocking_hz(180, 30, 1.0, 1.0) > tape_rocking_hz(180, 30, 0.2, 1.0))
    check("thicker tape rocks slower",
          tape_rocking_hz(180, 30, 0.5, 3.0) < tape_rocking_hz(180, 30, 0.5, 1.0))
    t = thermal()
    check("10 W needs ~0.86 L/s for a 10 C rise",
          abs(10.0 / (1005 * 10) / 1.16 * 1000 - 0.858) < 0.01 and t["watts_hi"] > t["watts_lo"])
    for name, items in configs().items():
        check(f"config '{name[:1]}' has four motors and a battery",
              sum(1 for i in items if i[0] == "motor+prop") == 4
              and any(i[0] == "battery" for i in items))
    return 1 if bad else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    sys.exit(self_test() if a.self_test else report())
