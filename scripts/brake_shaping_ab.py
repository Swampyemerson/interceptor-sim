#!/usr/bin/env python3
"""Stopping-distance brake cap A/B (2026-09-24 overnight). PRE-REGISTERED:
isim/specs/brake_shaping_prereg_2026-09-24.md -- read it before interpreting
output.

Port arm (real flight code, terminal="pursuit"), rear tag, all v5 errors on
(scatter=Scatter()), paired seeds 0..49. Five arms (baseline OFF, a=3/4/5 with
lead 0.45 s, a=4 with lead 0) x two vehicle fits (the honest ENGAGE fit =
primary, the dash fit = no-regression control) x four cells x two realism
rungs (tag_realism_v1 §F: R0 upright tag, R2 attitude + wobble).

`mc.run_many` hardcodes the default (dash) fit, so this script loads each fit
itself and fans `mc._run_one` out under its own spawn-context pool.

Mechanism column: `mc._run_one`'s rows carry no own-speed-at-range series for
the flyby/port arm (no trace is recorded there), so `med_vcl` is the median
CLOSING SPEED AT CPA (`closing_speed` row field) -- the nearest cheap proxy
for "arrived hot".
"""
import dataclasses
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from isim.replay_a0 import load_params
from isim.scenario import Scatter, Scenario
from isim import mc

N_SEEDS = 50
WORKERS = 10
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PLANTS = [
    ("honest", os.path.join(_ROOT, "isim/fits/vehicle_gazebo_x500_engage.json")),
    ("dash", os.path.join(_ROOT, "isim/fits/vehicle_gazebo_x500.json")),
]

ARMS = {
    "base": None,
    "a3": {"brake_shaping": True, "brake_accel_ms2": 3.0},
    "a4": {"brake_shaping": True, "brake_accel_ms2": 4.0},
    "a5": {"brake_shaping": True, "brake_accel_ms2": 5.0},
    "a4_lead0": {"brake_shaping": True, "brake_accel_ms2": 4.0, "brake_lead_s": 0.0},
}

# Rungs: (name, scenario overrides) -- tag_realism_v1 §F; scatter is the
# default Scatter() on both (R2 = target attitude + default wobble).
RUNGS = [
    ("R0", dict()),
    ("R2", dict(target_attitude=True)),
]

# Cells: (name, scenario overrides)
CELLS = [
    ("nominal", dict()),
    ("aim20", dict(aim_error_deg=20.0)),
    ("alt+3", dict(target_alt_offset_m=3.0, cam_tilt_up_deg=10.0,
                   tag_mount_pitch_deg=12.0)),
    ("weave", dict(target_motion="weave")),
]


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    params = {name: load_params(path) for name, path in PLANTS}
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=WORKERS, mp_context=ctx) as ex:
        # Submit everything up front (keeps all workers busy across group
        # boundaries), then print groups in registered order.
        groups = []
        for cell_name, cell_kw in CELLS:
            if only and only not in cell_name:
                continue
            for plant_name, _path in PLANTS:
                for rung_name, rung_kw in RUNGS:
                    for arm_name, overrides in ARMS.items():
                        base = Scenario(concept="flyby", terminal="pursuit",
                                        tag_facing="rear", scatter=Scatter(),
                                        port_pursuit_overrides=overrides,
                                        **cell_kw, **rung_kw)
                        futs = [ex.submit(mc._run_one,
                                          dataclasses.replace(base, seed=s),
                                          params[plant_name])
                                for s in range(N_SEEDS)]
                        groups.append((cell_name, plant_name, rung_name,
                                       arm_name, futs))

        print(f"{'cell':>8} {'plant':>6} {'rung':>4} {'arm':>9} {'med':>7} "
              f"{'p90':>7} {'<=0.35':>7} {'<=1.0':>6} {'med_dec':>8} "
              f"{'med_vcl':>8}", flush=True)
        prev_key = None
        for cell_name, plant_name, rung_name, arm_name, futs in groups:
            key = (cell_name, plant_name, rung_name)
            if prev_key is not None and key != prev_key:
                print(flush=True)
            prev_key = key
            rows = [f.result() for f in futs]
            if len(rows) != N_SEEDS:
                print(f"{cell_name:>8} {plant_name:>6} {rung_name:>4} "
                      f"{arm_name:>9}  UNCERTAIN -- {len(rows)} rows", flush=True)
                continue
            miss = np.array([float(r["miss_m"]) for r in rows])
            dec = np.median([float(r["n_decoded"]) for r in rows])
            vcl = np.median([float(r["closing_speed"]) for r in rows])
            print(f"{cell_name:>8} {plant_name:>6} {rung_name:>4} {arm_name:>9} "
                  f"{np.median(miss):7.3f} {np.percentile(miss, 90):7.3f} "
                  f"{100 * np.mean(miss <= 0.35):6.0f}% "
                  f"{100 * np.mean(miss <= 1.0):5.0f}% {dec:8.0f} {vcl:8.2f}",
                  flush=True)


if __name__ == "__main__":
    main()
