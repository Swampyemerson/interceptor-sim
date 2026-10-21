#!/usr/bin/env python3
"""Candidate-B study -- see spec.md."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from isim.mc import run_many, _summarize  # noqa: E402
from isim.scenario import Scenario, Scatter  # noqa: E402

ARMS = {
    "baseline": {},
    "B30": {"pip_aim": True, "look_angle_max_deg": 30.0},
    "B20": {"pip_aim": True, "look_angle_max_deg": 20.0},
    "PIPonly": {"pip_aim": True},
}
GEOMS = {"canon": dict(cross_range_m=6.5, lead_dist_m=16.2),
         "far": dict(cross_range_m=150.0, lead_dist_m=477.0)}
N = 8


def main():
    results = []
    for speed in (9.0, 12.0):
        for s in (1.0, 2.0):
            cam = dict(cam_fx_px=385.0 * s, cam_width_px=1280.0 * s,
                       cam_height_px=800.0 * s)
            for gname, geom in GEOMS.items():
                for arm, po in ARMS.items():
                    scns = [Scenario(target_speed_ms=speed, concept="pursuit",
                                     tag_facing="camera", scatter=Scatter(),
                                     pursuit_overrides=(po or None), seed=k,
                                     **cam, **geom) for k in range(N)]
                    summ = _summarize(run_many(scns, workers=os.cpu_count()))
                    results.append({"speed": speed, "s": s, "geom": gname,
                                    "arm": arm, **summ})
                    print(f"v={speed:4.1f} s={s:3.1f} {gname:5s} {arm:8s} "
                          f"med={summ.get('med_miss_m', -1):6.3f} "
                          f"eng={summ.get('pct_engage', -1):5.1f}% "
                          f"dec={summ.get('med_decoded', -1):6.1f}", flush=True)
    out = os.path.join(os.path.dirname(__file__), "results.json")
    json.dump({"spec": "spec.md", "n": N, "cells": results}, open(out, "w"), indent=1)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
