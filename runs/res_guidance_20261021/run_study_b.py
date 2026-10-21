#!/usr/bin/env python3
"""Variant B: same-FOV resolution scaling -- see spec.md AMENDMENT."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from isim.mc import run_many, _summarize  # noqa: E402
from isim.scenario import Scenario, Scatter  # noqa: E402

BASE_FX = 385.0
SCALES = (0.5, 1.0, 2.0, 3.0)
LAWS = ("pursuit", "tag")
SPEEDS = (9.0, 12.0)
N_SEEDS = 8


def main():
    results = []
    for speed in SPEEDS:
        for s in SCALES:
            for law in LAWS:
                scns = [Scenario(target_speed_ms=speed, terminal=law,
                                 tag_facing="camera", scatter=Scatter(),
                                 cam_fx_px=BASE_FX * s,
                                 cam_width_px=1280.0 * s,
                                 cam_height_px=800.0 * s, seed=k)
                        for k in range(N_SEEDS)]
                rows = run_many(scns, workers=os.cpu_count())
                summ = _summarize(rows)
                results.append({"speed_ms": speed, "res_scale": s, "law": law, **summ})
                print(f"v={speed:4.1f} s={s:3.1f} {law:8s} "
                      f"med_miss={summ.get('med_miss_m', -1):6.3f} "
                      f"engage={summ.get('pct_engage', -1):5.1f}% "
                      f"decoded={summ.get('med_decoded', -1):6.1f}", flush=True)
    out = os.path.join(os.path.dirname(__file__), "results_b2.json")
    with open(out, "w") as fh:
        json.dump({"spec": "spec.md#amendment", "n_seeds": N_SEEDS, "cells": results}, fh, indent=1)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
