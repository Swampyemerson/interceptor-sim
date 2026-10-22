#!/usr/bin/env python3
"""Registered d_behind {4,8} follow-up at n=32 -- see the spec's
'Registered follow-up' section. Reuses the sweep's cell builder."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from isim.mc import run_many, _summarize  # noqa: E402
SPEED = 9.0  # the canon point (run_sweep.py)
from isim.scenario import Scenario, Scatter  # noqa: E402

N = 32

def cell(d_behind):
    return [Scenario(concept="hybrid", target_speed_ms=SPEED,
                     cross_range_m=6.5, lead_dist_m=16.2, cam_fx_px=933.0,
                     tag_facing="rear", scatter=Scatter(), seed=s,
                     pursuit_d_behind_m=d_behind, dash_aim_behind_m=0.0,
                     aim_error_deg=0.0)
            for s in range(N)]

def main():
    out = {}
    for db in (4.0, 8.0):
        s = _summarize(run_many(cell(db), workers=os.cpu_count()))
        out[str(db)] = s
        print(f"d_behind={db}: med_miss={s['med_miss_m']:.3f} "
              f"p90={s['p90_miss_m']:.3f} engage={s['pct_engage']:.1f}% "
              f"decoded={s['med_decoded']:.1f}", flush=True)
    path = os.path.join(os.path.dirname(__file__), "followup_dbehind.json")
    with open(path, "w") as fh:
        json.dump({"n_seeds": N, "cells": out}, fh, indent=1)
    print("wrote", path)

if __name__ == "__main__":
    main()
