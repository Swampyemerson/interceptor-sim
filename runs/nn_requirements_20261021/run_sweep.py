#!/usr/bin/env python3
"""NN detector requirement sweep -- see spec.md (pre-registered) in this dir.

Run:  .venv/bin/python runs/nn_requirements_20261021/run_sweep.py
Writes results.json next to itself. Pure isim (no Gazebo), multiprocessing.
"""
import json
import os
import sys
from dataclasses import replace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from isim.mc import run_many, _summarize  # noqa: E402
from isim.scenario import Scenario, Scatter  # noqa: E402

FX = 385.0
SPAN_M = 0.30
N_SEEDS = 8
SPEEDS = (9.0, 12.0)

ANCHOR = dict(r50_m=16.0, fps=24.0, latency_s=0.06, p_max=0.9)

AXES = {
    "r50_m": [6.0, 8.0, 12.0, 16.0, 24.0, 32.0, 48.0],
    "fps": [5.0, 10.0, 15.0, 24.0, 30.0, 60.0],
    "latency_s": [0.02, 0.06, 0.12, 0.20],
    "p_max": [0.5, 0.7, 0.9],
}
CROSS = [(r, f) for r in (8.0, 12.0, 16.0) for f in (5.0, 10.0, 24.0)]


def detector_overrides(r50_m: float, fps: float, latency_s: float, p_max: float):
    s50 = SPAN_M * FX / r50_m
    dec = {"size50_px": s50, "size_k_px": s50 / 5.0, "min_side_px": 0.5 * s50,
           "p_max": p_max}
    cam = {"fps": fps, "latency_s": latency_s}
    return cam, dec


def cell_scenarios(speed: float, **knobs):
    cam, dec = detector_overrides(**knobs)
    return [Scenario(target_speed_ms=speed, terminal="pursuit",
                     tag_facing="camera", scatter=Scatter(), seed=s,
                     camera_overrides=cam, decode_overrides=dec)
            for s in range(N_SEEDS)]


def run_cell(label: str, speed: float, knobs: dict, out: list):
    rows = run_many(cell_scenarios(speed, **knobs), workers=os.cpu_count())
    s = _summarize(rows)
    ok = (s["verdict"] == "ok" and s["med_miss_m"] <= 0.35
          and s["pct_engage"] >= 87.5)
    rec = {"label": label, "speed_ms": speed, **knobs, **s, "pass": bool(ok)}
    out.append(rec)
    print(f"{label:28s} v={speed:4.1f} med_miss={s.get('med_miss_m', -1):6.3f} "
          f"engage={s.get('pct_engage', -1):5.1f}% decoded={s.get('med_decoded', -1):5.1f} "
          f"{'PASS' if ok else 'fail'}", flush=True)


def main():
    results = []
    for speed in SPEEDS:
        for axis, values in AXES.items():
            for v in values:
                knobs = dict(ANCHOR)
                knobs[axis] = v
                run_cell(f"{axis}={v}", speed, knobs, results)
        for r, f in CROSS:
            knobs = dict(ANCHOR, r50_m=r, fps=f)
            run_cell(f"cross r50={r} fps={f}", speed, knobs, results)
    out_path = os.path.join(os.path.dirname(__file__), "results.json")
    with open(out_path, "w") as fh:
        json.dump({"spec": "spec.md", "anchor": ANCHOR, "n_seeds": N_SEEDS,
                   "cells": results}, fh, indent=1)
    print(f"wrote {out_path} ({len(results)} cells)")


if __name__ == "__main__":
    main()
