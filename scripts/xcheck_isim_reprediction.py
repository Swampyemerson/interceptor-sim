#!/usr/bin/env python3
"""Honest isim re-prediction for the Gazebo pursuit cross-check (tick-trace
follow-up -- isim/specs/xcheck_tick_trace_2026-09-23.md, "recommended next
action" S4; registered in docs/xcheck_gazebo_pursuit_prereg2.md).

Same matched-optics scenario as scripts/xcheck_isim_prediction.py (canonical
crossing, Gazebo camera fx=540.3, tag 0.5 m, rear facing, port arm, no
scatter, seeds 0..N-1), run FOUR ways:

  (a) old fit, old cadence      -- sanity pin: must reproduce ~0.122 m median
                                   (logs/xcheck_isim_prediction_20260923.csv).
  (b) NEW ENGAGE-regime fit     -- does the honest plant alone move the
      (vehicle_gazebo_x500_engage) prediction toward the measured 1.51 m?
  (c) new fit + realistic detection cadence (~5 det/s, the driver's measured
      consumed rate; DecodeParams.min_decode_interval_s=0.192) -- honest
      plant + honest cadence. (b)/(c) answer: was the 12x "transfer gap"
      isim flattering the plant/pipeline?
  (d) (c) + pose-range supply (RealFlightGuidance.supply_pose_range=True,
      matching the FIXED driver, which now feeds the detector's PnP slant
      range to the pursuit terminal) -- the REGISTERED PREDICTION for the
      Gazebo re-fly with all three fixes. NOTE: isim's synthetic box is an
      ideal pinhole side (no AABB defect), so (d) ~= (c) is EXPECTED here;
      the pose-range fix removes a Gazebo-only defect isim structurally
      cannot see (tick-trace S2).

Uses scenario.build() + engine.run_engagement directly (not isim.mc.run_many,
whose fit path is hardcoded to the dash fit). Deterministic per seed.

Usage: .venv/bin/python scripts/xcheck_isim_reprediction.py [--n 50]
       [--arms a,b,c,d] [--workers 10] [--out logs/...csv]
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import dataclasses
import io
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _REPO)

from isim.engine import run_engagement          # noqa: E402
from isim.replay_a0 import load_params          # noqa: E402
from isim.scenario import Scenario, build       # noqa: E402

OLD_FIT = os.path.join(_REPO, "isim", "fits", "vehicle_gazebo_x500.json")
NEW_FIT = os.path.join(_REPO, "isim", "fits", "vehicle_gazebo_x500_engage.json")
# Driver-measured consumed-detection cadence in ENGAGE (tick trace S3:
# 5.2 det/s); interval = 1/5.2.
THIN_S = 0.192

ARMS = {
    #        fit_path, thin_s, supply_pose_range
    "a": (OLD_FIT, 0.0, False),
    "b": (NEW_FIT, 0.0, False),
    "c": (NEW_FIT, THIN_S, False),
    "d": (NEW_FIT, THIN_S, True),
    # Cadence-sensitivity arms for the prereg2 band (the FIXED driver detects
    # every tick, so the re-fly will consume ~2x the old 5.2 det/s -- the
    # registered prediction must cover the delivered-cadence uncertainty):
    "d10": (NEW_FIT, 0.096, True),    # ~10.4 det/s (every-tick ceiling)
    "dfull": (NEW_FIT, 0.0, True),    # no thinning (isim's own ~20+/s)
}


def _run_one(job):
    arm, seed = job
    fit_path, thin_s, pose = ARMS[arm]
    vp = load_params(fit_path)
    scn = Scenario(concept="flyby", terminal="pursuit", tag_facing="rear",
                   cam_fx_px=540.3, tag_side_m=0.5, seed=seed)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    if thin_s > 0.0:
        seeker.dec = dataclasses.replace(seeker.dec, min_decode_interval_s=thin_s)
    if pose:
        guidance.supply_pose_range = True
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        r = run_engagement(ecfg, vehicle, target, seeker, guidance, init)
    n_fault = sum(1 for ln in buf.getvalue().splitlines() if "FAULT" in ln)
    return {"arm": arm, "seed": seed, "miss_m": r.miss_m, "t_cpa": r.t_cpa,
            "miss_horiz_m": r.miss_horiz_m, "miss_vert_m": r.miss_vert_m,
            "n_decoded": r.n_decoded, "n_fault_lines": n_fault}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--arms", default="a,b,c,d")
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--out", default=os.path.join(
        _REPO, "logs", "xcheck_isim_reprediction_20260923.csv"))
    args = ap.parse_args()

    arms = [a.strip() for a in args.arms.split(",") if a.strip()]
    for a in arms:
        fit_path = ARMS[a][0]
        if not os.path.exists(fit_path):
            raise SystemExit(f"arm {a}: fit file missing: {fit_path}")
    jobs = [(a, s) for a in arms for s in range(args.n)]
    if args.workers <= 1:
        rows = [_run_one(j) for j in jobs]
    else:
        ctx = get_context("spawn")
        with ProcessPoolExecutor(max_workers=args.workers, mp_context=ctx) as ex:
            rows = list(ex.map(_run_one, jobs))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        if fh.tell() == 0:
            w.writeheader()
        w.writerows(rows)

    for a in arms:
        miss = np.array([r["miss_m"] for r in rows if r["arm"] == a])
        dec = np.array([r["n_decoded"] for r in rows if r["arm"] == a])
        print(f"arm {a}: n {len(miss)}  median {np.median(miss):.3f} m  "
              f"p10 {np.percentile(miss, 10):.3f}  "
              f"p90 {np.percentile(miss, 90):.3f}  "
              f"%<=0.35 {100 * np.mean(miss <= 0.35):.0f}%  "
              f"%<=1.0 {100 * np.mean(miss <= 1.0):.0f}%  "
              f"med_decoded {np.median(dec):.0f}")
    print("appended ->", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
