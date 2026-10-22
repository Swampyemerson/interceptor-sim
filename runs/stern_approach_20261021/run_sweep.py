#!/usr/bin/env python3
"""Stern-approach geometry sweep -- pre-registered in
isim/specs/stern_approach_2026-10-21.md (amendments logged there, no data
taken before them). Pattern copied from runs/nn_requirements_20261021.

Run:  .venv/bin/python runs/stern_approach_20261021/run_sweep.py
Writes results.json next to itself. Pure isim (no Gazebo), multiprocessing.
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from isim.mc import run_many, _summarize  # noqa: E402
from isim.scenario import Scenario, Scatter  # noqa: E402

N_SEEDS = 8
SPEED = 9.0    # the isim-earned pursuit/hybrid operating point (spec amendment)
D_BEHIND = [4.0, 8.0, 16.0, 24.0]     # PursuitConfig trail distance (8 = today)
DASH_AIM_BEHIND = [0.0, 8.0, 16.0, 32.0]   # dash solve astern shift (0 = today)
AIM_ERR_DEG = [0.0, 4.0, 8.0]     # deliberate cue error on top of Scatter


def cell_scenarios(d_behind: float, dash_behind: float, aim_err: float):
    # Canonical pursuit/hybrid geometry (the isim-validated config used by
    # the concept tests): fx 933, 6.5 m cross range, 16.2 m lead.
    return [Scenario(concept="hybrid", target_speed_ms=SPEED,
                     cross_range_m=6.5, lead_dist_m=16.2, cam_fx_px=933.0,
                     tag_facing="rear", scatter=Scatter(), seed=s,
                     pursuit_d_behind_m=d_behind,
                     dash_aim_behind_m=dash_behind,
                     aim_error_deg=aim_err)
            for s in range(N_SEEDS)]


def main():
    """Checkpointed: one JSONL line per finished cell (daemons on this box
    die silently -- the standing pattern is checkpoint + relaunch-until-done),
    skip-on-resume, final results.json assembled from the JSONL."""
    here = os.path.dirname(__file__)
    ckpt = os.path.join(here, "cells.jsonl")
    done = {}
    if os.path.exists(ckpt):
        with open(ckpt) as fh:
            for line in fh:
                rec = json.loads(line)
                done[(rec["aim_error_deg"], rec["d_behind_m"],
                      rec["dash_aim_behind_m"])] = rec
    results = []
    for aim_err in AIM_ERR_DEG:
        for d_behind in D_BEHIND:
            for dash_behind in DASH_AIM_BEHIND:
                key = (aim_err, d_behind, dash_behind)
                if key in done:
                    results.append(done[key])
                    continue
                rows = run_many(cell_scenarios(d_behind, dash_behind, aim_err),
                                workers=os.cpu_count())
                s = _summarize(rows)
                rec = {"aim_error_deg": aim_err, "d_behind_m": d_behind,
                       "dash_aim_behind_m": dash_behind, **s}
                results.append(rec)
                with open(ckpt, "a") as fh:
                    fh.write(json.dumps(rec) + "\n")
                print(f"aim_err={aim_err:3.0f} d_behind={d_behind:4.1f} "
                      f"dash_behind={dash_behind:4.1f} "
                      f"med_miss={s.get('med_miss_m', -1):6.3f} "
                      f"engage={s.get('pct_engage', -1):5.1f}% "
                      f"decoded={s.get('med_decoded', -1):5.1f}", flush=True)
    out_path = os.path.join(here, "results.json")
    with open(out_path, "w") as fh:
        json.dump({"spec": "isim/specs/stern_approach_2026-10-21.md",
                   "n_seeds": N_SEEDS, "speed_ms": SPEED, "cells": results},
                  fh, indent=1)
    print(f"wrote {out_path} ({len(results)} cells)")


if __name__ == "__main__":
    main()
