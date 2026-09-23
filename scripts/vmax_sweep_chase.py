#!/usr/bin/env python3
"""Chase speed-ceiling sweep: pursuit v_max vs target speed (native concept).

The 16 m/s pursuit v_max is a CONFIG constant, not physics (the sim vehicle
model is fitted with margin, and a real 5-inch quad has more top speed than
16 m/s). This sweep asks whether the chase's speed ceiling is simply the
overtake margin v_max - v_target, by raising v_max on the PROTOTYPE concept
(pursuit_v_max_ms is a native-concept-only hook) across target 15/18/21 m/s.
Vehicle dynamics still come from the fitted Gazebo x500 model, so a v_max
above what the fitted accel/drag can sustain will show up honestly as the
vehicle failing to reach it.
"""
import dataclasses

import numpy as np

from isim.scenario import Scenario
from isim import mc


def main():
    print(f"{'v_max':>6} {'tgt':>5} {'med':>7} {'p90':>7} {'<=0.35':>7} "
          f"{'<=1.0':>6} {'med_dec':>8}")
    for vmax in (16.0, 20.0, 24.0):
        for spd in (15.0, 18.0, 21.0):
            scens = [dataclasses.replace(
                Scenario(concept="pursuit", tag_facing="rear",
                         target_speed_ms=spd, pursuit_v_max_ms=vmax),
                seed=s) for s in range(50)]
            rows = mc.run_many(scens, workers=10)
            miss = np.array([float(r["miss_m"]) for r in rows])
            dec = np.median([float(r["n_decoded"]) for r in rows])
            print(f"{vmax:6.0f} {spd:5.0f} {np.median(miss):7.3f} "
                  f"{np.percentile(miss, 90):7.3f} "
                  f"{100 * np.mean(miss <= 0.35):6.0f}% "
                  f"{100 * np.mean(miss <= 1.0):5.0f}% {dec:8.0f}")


if __name__ == "__main__":
    main()
