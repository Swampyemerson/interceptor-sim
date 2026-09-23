#!/usr/bin/env python3
"""isim prediction for the Gazebo pursuit cross-check (pre-registered arm).

Same canonical crossing geometry as the parity grid, but with the GAZEBO
camera and tag substituted for the real-hardware model: fx = 540.3 px
(mono_cam HFOV 1.74 rad at 1280 px) and tag_side_m = 0.5 (the apriltag
world's placard, models/apriltag_target/model.sdf). Rear-facing tag, port
arm (flyby concept + pursuit terminal), no scatter, seeds 0..49.

The isim vehicle model is itself fitted to Gazebo x500 flights
(isim/fits/vehicle_gazebo_x500.json), so this run IS the prediction of what
the same flight code should do in Gazebo if the port transfers.
"""
import csv
import dataclasses

import numpy as np

from isim.scenario import Scenario
from isim import mc

OUT = "logs/xcheck_isim_prediction_20260923.csv"


def main():
    scens = [dataclasses.replace(
        Scenario(concept="flyby", terminal="pursuit", tag_facing="rear",
                 cam_fx_px=540.3, tag_side_m=0.5),
        seed=s) for s in range(50)]
    rows = mc.run_many(scens, workers=10)
    miss = np.array([float(r["miss_m"]) for r in rows])
    print(f"n {len(miss)}  median {np.median(miss):.3f}  "
          f"p10 {np.percentile(miss, 10):.3f}  p90 {np.percentile(miss, 90):.3f}  "
          f"pct<=0.35 {100 * np.mean(miss <= 0.35):.1f}%  "
          f"pct<=1.0 {100 * np.mean(miss <= 1.0):.1f}%")
    with open(OUT, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
