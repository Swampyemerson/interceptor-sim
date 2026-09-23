#!/usr/bin/env python3
"""Keep-in-frame vertical assist A/B (2026-09-23). PRE-REGISTERED:
isim/specs/keepframe_prereg_2026-09-23.md -- read it before interpreting
output (predictions, adopt/reject criterion, and what a NULL means were
written down BEFORE this script first ran).

Port arm (real flight code, concept="flyby" terminal="pursuit"), rear tag,
ALL realistic errors on (Scatter() defaults), paired seeds 0..49. Two arms:
OFF (defaults) vs ON (keepframe_assist=True). Twelve cells:
target_alt_offset_m {-2, 0, +1, +2, +3, +4} x aim_error_deg {0, 20}.
Mechanism metric per cell: median decode count and median decode fraction
(n_decoded / n_frames) -- a win must come with more lock.

Mechanics copied from scripts/adaptive_speed_ab.py.
"""
import dataclasses

import numpy as np

from isim.scenario import Scatter, Scenario
from isim import mc

ARMS = {
    "OFF": None,
    "ON": {"keepframe_assist": True},
}
ALT_CELLS = [-2.0, 0.0, 1.0, 2.0, 3.0, 4.0]
AIM_CELLS = [0.0, 20.0]
N_SEEDS = 50


def main():
    print(f"{'cell':>14} {'arm':>4} {'med':>7} {'p90':>7} {'<=0.35':>7} "
          f"{'<=1.0':>6} {'med_dec':>8} {'med_decfrac':>12}")
    for aim in AIM_CELLS:
        for alt in ALT_CELLS:
            cell_name = f"alt{alt:+.0f}_aim{aim:.0f}"
            for arm_name, overrides in ARMS.items():
                base = Scenario(concept="flyby", terminal="pursuit",
                                tag_facing="rear", scatter=Scatter(),
                                target_alt_offset_m=alt, aim_error_deg=aim,
                                port_pursuit_overrides=overrides)
                scens = [dataclasses.replace(base, seed=s) for s in range(N_SEEDS)]
                rows = mc.run_many(scens, workers=10)
                miss = np.array([float(r["miss_m"]) for r in rows])
                dec = np.median([float(r["n_decoded"]) for r in rows])
                decfrac = np.median([float(r["n_decoded"]) / max(1.0, float(r["n_frames"]))
                                     for r in rows])
                print(f"{cell_name:>14} {arm_name:>4} {np.median(miss):7.3f} "
                      f"{np.percentile(miss, 90):7.3f} "
                      f"{100 * np.mean(miss <= 0.35):6.0f}% "
                      f"{100 * np.mean(miss <= 1.0):5.0f}% {dec:8.0f} "
                      f"{decfrac:12.3f}", flush=True)
            print()


if __name__ == "__main__":
    main()
