#!/usr/bin/env python3
"""Chase pursuit vs blind dash under injected aim error -- pre-registered in
docs/pursuit_vs_dash_prereg_2026-10-22.md (registered before this ran).
Canon base = the stern/target-height studies' (rear tag, fx 933 ~= the
delivered ~69-deg lens, all v5 errors on), paired seeds 0..15."""
import csv
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from isim.mc import run_many, _summarize  # noqa: E402
from isim.scenario import Scenario, Scatter  # noqa: E402

N = 16
AIMS = [0.0, 5.0, 10.0, 20.0]
HIT_M = 0.35
ARMS = {
    "blind":   dict(concept="flyby", terminal="stock",
                    decode_overrides={"p_max": 0.0}),
    "pursuit": dict(concept="flyby", terminal="pursuit"),
    "hybrid":  dict(concept="hybrid"),
}
BASE = dict(target_speed_ms=9.0, cross_range_m=6.5, lead_dist_m=16.2,
            cam_fx_px=933.0, tag_facing="rear")


def cell(arm_kw, aim):
    return [Scenario(**BASE, **arm_kw, scatter=Scatter(), seed=s,
                     aim_error_deg=aim) for s in range(N)]


def main():
    here = os.path.dirname(__file__)
    ckpt = os.path.join(here, "cells.jsonl")
    done = {}
    if os.path.exists(ckpt):
        with open(ckpt) as fh:
            done = {(j["arm"], j["aim"]): j
                    for j in (json.loads(l) for l in fh)}
    cells, perrun = [], []
    for arm, kw in ARMS.items():
        for aim in AIMS:
            key = (arm, aim)
            if key in done:
                cells.append(done[key])
                perrun.extend(done[key]["runs"])
                continue
            rows = run_many(cell(kw, aim), workers=os.cpu_count())
            runs = [{"arm": arm, "aim": aim, "seed": r["seed"],
                     "miss_m": r["miss_m"], "hit": r["miss_m"] <= HIT_M,
                     "n_decoded": r["n_decoded"], "t_cpa": r["t_cpa"]}
                    for r in rows]
            s = _summarize(rows)
            rec = {"arm": arm, "aim": aim, "n": len(rows),
                   "med_miss_m": s["med_miss_m"], "p90_miss_m": s["p90_miss_m"],
                   "pct_hit": 100.0 * sum(r["hit"] for r in runs) / len(runs),
                   "pct_engage": s["pct_engage"], "runs": runs}
            cells.append(rec)
            perrun.extend(runs)
            with open(ckpt, "a") as fh:
                fh.write(json.dumps(rec) + "\n")
            print(f"{arm:8s} aim={aim:4.1f}: med={rec['med_miss_m']:6.3f} "
                  f"p90={rec['p90_miss_m']:6.3f} hit={rec['pct_hit']:5.1f}% "
                  f"engage={rec['pct_engage']:5.1f}%", flush=True)
    with open(os.path.join(here, "perrun.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(perrun[0].keys()))
        w.writeheader()
        w.writerows(perrun)
    # paired-seed comparison, pursuit vs blind, per aim cell
    paired = []
    byk = {(r["arm"], r["aim"], r["seed"]): r for r in perrun}
    for aim in AIMS:
        wins = sum(1 for s in range(N)
                   if byk[("pursuit", aim, s)]["miss_m"]
                   < byk[("blind", aim, s)]["miss_m"])
        dhit = (sum(byk[("pursuit", aim, s)]["hit"] for s in range(N))
                - sum(byk[("blind", aim, s)]["hit"] for s in range(N)))
        paired.append({"aim": aim, "pursuit_wins_of_16": wins,
                       "hit_delta_pursuit_minus_blind": dhit})
        print(f"paired aim={aim:4.1f}: pursuit wins {wins}/16 seeds, "
              f"hit delta {dhit:+d}", flush=True)
    with open(os.path.join(here, "results.json"), "w") as fh:
        json.dump({"spec": "docs/pursuit_vs_dash_prereg_2026-10-22.md",
                   "n_seeds": N, "hit_m": HIT_M,
                   "cells": [{k: v for k, v in c.items() if k != "runs"}
                             for c in cells],
                   "paired": paired}, fh, indent=1)
    print("wrote results.json")


if __name__ == "__main__":
    main()
