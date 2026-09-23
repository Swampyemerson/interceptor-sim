#!/usr/bin/env python3
"""Wind vs. the closed-loop chase (2026-09-23). PRE-REGISTERED:
isim/specs/wind_chase_prereg_2026-09-23.md -- read it before interpreting
output (predictions, adopt criterion, and what a null means are frozen there).

Port arm (real flight code chase: concept="flyby", terminal="pursuit"), rear
tag, all realistic errors on (scatter), paired seeds 0..49 per cell. Cells:
wind {0, 1.5, 4.5, 8} m/s x direction {head, cross, tail} relative to the
target's track (track = north; head = wind FROM the north, opposing the
chaser's overtake). Steady wind + horizontal OU gusts at 40% of steady
(vertical gusts deliberately 0 -- see the prereg amendment). The TARGET is
NOT wind-affected (scripted kinematic track -- existing sim assumption).

Coefficient disclosure: wind couples through the vehicle model's fitted
drag_linear_horiz = 0.1586 s^-1 (fitted on ZERO-WIND Gazebo flights; same
order as PX4's default MCOEF 0.15 s^-1) -- a PLACEHOLDER as a wind
coefficient. This measures the MECHANISM, not the magnitude.

Metrics per cell: % closing inside 0.35 m, median CPA, median achieved
overtake speed over the last 3 s before CPA (chaser's along-track ground
speed minus the target's 9 m/s -- needs the trace, so this script runs
engagements itself instead of going through isim.mc.run_many).
"""
import argparse
import csv
import dataclasses
import itertools
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

import numpy as np

from isim.engine import run_engagement
from isim.replay_a0 import load_params
from isim.scenario import Scatter, Scenario, build

TARGET_SPEED = 9.0
GUST_FRAC = 0.4          # OU gust sigma as a fraction of the steady speed
N_SEEDS = 50

# (cell name, wind_ned) -- track = north; head = wind blowing FROM north.
def _cells():
    cells = [("0_none", 0.0, (0.0, 0.0, 0.0))]
    for w in (1.5, 4.5, 8.0):
        cells.append((f"{w:g}_head", w, (-w, 0.0, 0.0)))
        cells.append((f"{w:g}_cross", w, (0.0, w, 0.0)))
        cells.append((f"{w:g}_tail", w, (+w, 0.0, 0.0)))
    return cells


def _run_one(args):
    """One engagement -> plain scalar dict (picklable for spawn workers)."""
    cell, seed, wind_ned, gust_ou = args
    scn = Scenario(concept="flyby", terminal="pursuit", tag_facing="rear",
                   target_speed_ms=TARGET_SPEED, scatter=Scatter(), seed=seed,
                   vehicle_overrides={
                       # Controlled wind: override Scatter's own random
                       # wind/gust draws so the cell IS the wind condition.
                       "wind_ned": wind_ned,
                       "gust_std": 0.0,
                       "gust_ou_std": gust_ou,
                   })
    ecfg, vehicle, target, seeker, guidance, init = build(scn, load_params())
    res = run_engagement(ecfg, vehicle, target, seeker, guidance, init,
                         record_trace=True)
    # Achieved overtake speed: median along-track (north) own ground speed
    # over the last 3 s before CPA, minus the target's speed.
    t = res.trace["t"]
    win = (t >= res.t_cpa - 3.0) & (t <= res.t_cpa)
    ovt = float(np.median(res.trace["own_vel"][win, 0])) - TARGET_SPEED \
        if np.any(win) else float("nan")
    return {"cell": cell, "seed": seed, "miss_m": res.miss_m,
            "t_cpa": res.t_cpa, "overtake_last3s_ms": ovt,
            "n_decoded": res.n_decoded}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--out", type=str, default=None, help="optional CSV path")
    a = ap.parse_args()

    jobs = [(cell, seed, wind, GUST_FRAC * w)
            for (cell, w, wind) in _cells() for seed in range(N_SEEDS)]
    if a.workers <= 1:
        rows = [_run_one(j) for j in jobs]
    else:
        ctx = get_context("spawn")
        with ProcessPoolExecutor(max_workers=a.workers, mp_context=ctx) as ex:
            rows = list(ex.map(_run_one, jobs))

    print(f"{'cell':>10} {'n':>3} {'<=0.35':>7} {'med_cpa':>8} {'p90_cpa':>8} "
          f"{'med_ovt3s':>9} {'med_dec':>8}")
    for (cell, _w, _wind) in _cells():
        g = [r for r in rows if r["cell"] == cell]
        if not g:
            print(f"{cell:>10}   0  UNCERTAIN -- zero runs")
            continue
        miss = np.array([r["miss_m"] for r in g])
        ovt = np.array([r["overtake_last3s_ms"] for r in g])
        dec = np.median([r["n_decoded"] for r in g])
        print(f"{cell:>10} {len(g):>3} {100*np.mean(miss <= 0.35):6.0f}% "
              f"{np.median(miss):8.3f} {np.percentile(miss, 90):8.3f} "
              f"{np.nanmedian(ovt):9.2f} {dec:8.0f}")

    if a.out:
        with open(a.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {len(rows)} rows to {a.out}")


if __name__ == "__main__":
    main()
