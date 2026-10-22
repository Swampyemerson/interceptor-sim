#!/usr/bin/env python3
"""Target-height cue-error tolerance sweep -- pre-registered in
isim/specs/target_height_error_2026-10-22.md. Canon hybrid base (the stern
study's), vertical axis only."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from isim.mc import run_many, _summarize  # noqa: E402
from isim.scenario import Scenario, Scatter  # noqa: E402

N = 16
OFFSETS = [0.0, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0]


def cell(off):
    return [Scenario(concept="hybrid", target_speed_ms=9.0,
                     cross_range_m=6.5, lead_dist_m=16.2, cam_fx_px=933.0,
                     tag_facing="rear", scatter=Scatter(), seed=s,
                     aim_error_deg=0.0, target_alt_offset_m=off)
            for s in range(N)]


def main():
    here = os.path.dirname(__file__)
    ckpt = os.path.join(here, "cells.jsonl")
    done = {}
    if os.path.exists(ckpt):
        with open(ckpt) as fh:
            done = {json.loads(l)["target_alt_offset_m"]: json.loads(l) for l in fh}
    out = []
    for off in OFFSETS:
        if off in done:
            out.append(done[off]); continue
        s = _summarize(run_many(cell(off), workers=os.cpu_count()))
        rec = {"target_alt_offset_m": off, **s}
        out.append(rec)
        with open(ckpt, "a") as fh:
            fh.write(json.dumps(rec) + "\n")
        print(f"offset={off:+4.1f}: engage={s['pct_engage']:5.1f}% "
              f"med_miss={s['med_miss_m']:6.3f} p90={s['p90_miss_m']:6.3f} "
              f"vert={s.get('med_vert_m', float('nan')):+6.3f} "
              f"decoded={s['med_decoded']:5.1f}", flush=True)
    with open(os.path.join(here, "results.json"), "w") as fh:
        json.dump({"spec": "isim/specs/target_height_error_2026-10-22.md",
                   "n_seeds": N, "cells": out}, fh, indent=1)
    print("wrote results.json")


if __name__ == "__main__":
    main()
