"""A0 (part 1): replay logged OPEN-LOOP sprint flights through the fitted vehicle
model and score each replay against the LOGGED target track. If isim's vehicle
is good enough, model miss ~= logged miss, arm by arm. Dash-only arms only: their
commands do not depend on what the vehicle did, so a command replay is valid.

Usage: python -m isim.replay_a0 ARM [ARM ...]   (e.g. TOLm5 TOL0 TOL5 TOL10)
"""
from __future__ import annotations

import csv
import glob
import json
import os
import sys

import numpy as np

from isim.fit_vehicle import simulate_segment
from isim.fitdata import load_flight
from isim.vehicle import VehicleParams


def load_params(path="isim/fits/vehicle_gazebo_x500.json") -> VehicleParams:
    p = json.load(open(path))["params"]
    p["wind_ned"] = tuple(p["wind_ned"])
    return VehicleParams(**p)


def cpa(t, pn, pe, pd, seg):
    """Closest approach to the logged target, linear-interpolated per interval."""
    r = np.stack([seg["tgt_n"] - pn, seg["tgt_e"] - pe, seg["tgt_d"] - pd], 1)
    best = (np.inf, 0.0, 0.0)
    for i in range(len(t) - 1):
        a, b = r[i], r[i + 1]
        d = b - a
        tau = float(np.clip(-(a @ d) / max(d @ d, 1e-12), 0.0, 1.0))
        m = a + tau * d
        n = float(np.linalg.norm(m))
        if n < best[0]:
            best = (n, float(np.hypot(m[0], m[1])), float(m[2]))
    return best     # (miss, horizontal, vertical: + = vehicle ABOVE target)


def main(arms) -> int:
    params = load_params()
    print(f"{'arm':10s} {'n':>3s} {'logged med':>10s} {'model med':>10s} "
          f"{'|diff| med':>10s} {'logged vert':>11s} {'model vert':>10s}")
    for arm in arms:
        rows = []
        for batch in sorted(glob.glob(f"logs/mc_fp_arm{arm}_line9_s*.csv")):
            for r in csv.DictReader(open(batch)):
                p = r["flight_csv_path"]
                seg = load_flight(p) if os.path.exists(p) else None
                if seg is None:
                    continue
                m = simulate_segment(params, seg)
                lo = cpa(seg["t"], seg["pos_n"], seg["pos_e"], seg["pos_d"], seg)
                mo = cpa(seg["t"], m["pos_n"], m["pos_e"], m["pos_d"], seg)
                rows.append((lo, mo))
        if not rows:
            print(f"{arm:10s}   0  -- no usable flights (UNCERTAIN, not a pass)")
            continue
        L = np.array([r[0] for r in rows]); M = np.array([r[1] for r in rows])
        print(f"{arm:10s} {len(rows):3d} {np.median(L[:,0]):10.2f} {np.median(M[:,0]):10.2f} "
              f"{np.median(np.abs(L[:,0]-M[:,0])):10.2f} {np.median(L[:,2]):+11.2f} "
              f"{np.median(M[:,2]):+10.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:] or ["TOLm5", "TOL0", "TOL2", "TOL5", "TOL7", "TOL10", "TOL15"]))
