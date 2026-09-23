#!/usr/bin/env python3
"""Adaptive speed governor A/B (2026-09-23). PRE-REGISTERED:
docs/adaptive_speed_prereg.md -- read it before interpreting output.

Port arm (real flight code), rear tag, paired seeds 0..49. Three arms:
A legacy (defaults), B fixed-high (v_max_ms=24), C adaptive
(adaptive_speed=True). Five cells: 9/15/18 m/s nominal, 9+aim20, 15+aim10.
"""
import dataclasses

import numpy as np

from isim.scenario import Scenario
from isim import mc

ARMS = {
    "A_legacy": None,
    "B_vmax24": {"v_max_ms": 24.0},
    "C_adaptive": {"adaptive_speed": True},
}
CELLS = [
    ("9_nom", dict(target_speed_ms=9.0)),
    ("15_nom", dict(target_speed_ms=15.0)),
    ("18_nom", dict(target_speed_ms=18.0)),
    ("9_aim20", dict(target_speed_ms=9.0, aim_error_deg=20.0)),
    ("15_aim10", dict(target_speed_ms=15.0, aim_error_deg=10.0)),
]


def main():
    print(f"{'cell':>9} {'arm':>10} {'med':>7} {'p90':>7} {'<=0.35':>7} "
          f"{'<=1.0':>6} {'med_dec':>8}")
    for cell_name, cell_kw in CELLS:
        for arm_name, overrides in ARMS.items():
            base = Scenario(concept="flyby", terminal="pursuit",
                            tag_facing="rear",
                            port_pursuit_overrides=overrides, **cell_kw)
            scens = [dataclasses.replace(base, seed=s) for s in range(50)]
            rows = mc.run_many(scens, workers=10)
            miss = np.array([float(r["miss_m"]) for r in rows])
            dec = np.median([float(r["n_decoded"]) for r in rows])
            print(f"{cell_name:>9} {arm_name:>10} {np.median(miss):7.3f} "
                  f"{np.percentile(miss, 90):7.3f} "
                  f"{100 * np.mean(miss <= 0.35):6.0f}% "
                  f"{100 * np.mean(miss <= 1.0):5.0f}% {dec:8.0f}")
        print()


if __name__ == "__main__":
    main()
