#!/usr/bin/env python3
"""Chase-regime camera up-tilt A/B (2026-09-23). PRE-REGISTERED:
isim/specs/chase_tilt_prereg_2026-09-23.md -- read it before interpreting
output (predictions, adopt/reject criterion and null meaning were written
BEFORE this script first ran).

CONFIG-ONLY lever: no flight-code change. `Scenario.cam_tilt_up_deg` feeds
BOTH the true seeker camera and the flight code's GuidanceConfig.mount_up_rad
via isim.flight_adapter (the same capture-instant attitude/mount path
guidance uses -- never a scoring-only tilt); Scatter's cam_tilt_sigma_deg
error stays true-camera-only on top, as always.

Part 1 (main grid, mc.run_many): tilt {0, 10, 20} deg x alt-err
{-2, 0, +1, +2, +3, +4} m, aim 0, rear tag, scatter on, n=50 paired seeds.
Part 2 (mechanism subset, traced): 20 seeds, cells {-2, 0, +3} x all tilts:
mean decoded box-center-v (px), median approach-window length, and the
same per-frame attribution instrument as the keepframe follow-up (TOP-exit
share must shrink for a win to be attributed to the raised ceiling).
"""
import contextlib
import dataclasses
import io
import math
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

import numpy as np

from isim.engine import run_engagement
from isim.replay_a0 import load_params
from isim.scenario import Scatter, Scenario, build
from isim.seeker import world_to_camera, project
from isim import mc

TILTS = [0.0, 10.0, 20.0]
ALT_CELLS = [-2.0, 0.0, 1.0, 2.0, 3.0, 4.0]
N_SEEDS = 50
MECH_CELLS = [-2.0, 0.0, 3.0]
MECH_SEEDS = 20


def _scenario(alt, tilt, seed):
    return Scenario(concept="flyby", terminal="pursuit", tag_facing="rear",
                    scatter=Scatter(), aim_error_deg=0.0,
                    target_alt_offset_m=alt, cam_tilt_up_deg=tilt, seed=seed)


# ------------------------------------------------------------- part 1: grid

def main_grid():
    print("== PART 1: main grid (n=50 paired seeds/cell-arm) ==")
    print(f"{'cell':>8} {'tilt':>5} {'med':>7} {'p90':>7} {'<=0.35':>7} "
          f"{'<=1.0':>6} {'med_dec':>8}")
    for alt in ALT_CELLS:
        for tilt in TILTS:
            scens = [_scenario(alt, tilt, s) for s in range(N_SEEDS)]
            rows = mc.run_many(scens, workers=10)
            miss = np.array([float(r["miss_m"]) for r in rows])
            dec = np.median([float(r["n_decoded"]) for r in rows])
            print(f"{alt:+8.0f} {tilt:5.0f} {np.median(miss):7.3f} "
                  f"{np.percentile(miss, 90):7.3f} "
                  f"{100 * np.mean(miss <= 0.35):6.0f}% "
                  f"{100 * np.mean(miss <= 1.0):5.0f}% {dec:8.0f}", flush=True)
        print()


# -------------------------------------------------- part 2: mechanism subset

def _nearest(ts, t):
    return int(np.clip(np.searchsorted(ts, t), 0, len(ts) - 1))


def _frame_bucket(rep, trace, seeker):
    """TOP/BOTTOM/HORIZ/in-frame bucket for one no-decode frame -- the same
    ground-truth instrument as the keepframe follow-up (diagnosis only)."""
    cam = seeker.cam
    k = _nearest(trace["t"], rep.t_capture)
    p_cam = world_to_camera(trace["tgt_pos"][k], trace["own_pos"][k],
                            trace["quat"][k], cam)
    u, v, in_front = project(p_cam, cam)
    if not in_front:
        return "behind"
    vert_out = (v < 0.0) or (v >= cam.height)
    horiz_out = (u < 0.0) or (u >= cam.width)
    if vert_out and horiz_out:
        return "out-both"
    if vert_out:
        return "out-TOP" if v < 0.0 else "out-BOTTOM"
    if horiz_out:
        return "out-horiz"
    return "inframe-nodecode"


def _mech_one(args):
    alt, tilt, seed = args
    scn = _scenario(alt, tilt, seed)
    ecfg, vehicle, target, seeker, guidance, init_state = build(scn, load_params())
    with contextlib.redirect_stdout(io.StringIO()):
        result = run_engagement(ecfg, vehicle, target, seeker, guidance,
                                init_state, record_trace=True)
    trace = result.trace
    det_mask = trace["det_new"] > 0.5
    mean_det_v = float(np.mean(trace["det_v"][det_mask])) if np.any(det_mask) else math.nan
    t_eng = next((t for t, s in guidance.state_log if s == "ENGAGE"), None)
    window_s = math.nan
    buckets = Counter()
    if t_eng is not None:
        first_dec = next((r.t_capture for r in result.frame_reports
                          if r.decoded and r.t_capture >= t_eng), None)
        t_end = first_dec if first_dec is not None else float(trace["t"][-1])
        window_s = t_end - t_eng
        for rep in result.frame_reports:
            if t_eng <= rep.t_capture <= t_end and not rep.decoded:
                buckets[_frame_bucket(rep, trace, seeker)] += 1
    return mean_det_v, window_s, buckets


def mechanism():
    print("== PART 2: mechanism subset (n=20 seeds, traced) ==")
    print(f"{'cell':>8} {'tilt':>5} {'mean_det_v':>11} {'med_window_s':>13} "
          f"{'%TOP':>6} {'%BOT':>6} {'%horiz':>7} {'%inframe':>9}")
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=10, mp_context=ctx) as ex:
        for alt in MECH_CELLS:
            for tilt in TILTS:
                jobs = [(alt, tilt, s) for s in range(MECH_SEEDS)]
                out = list(ex.map(_mech_one, jobs))
                det_v = [o[0] for o in out if math.isfinite(o[0])]
                windows = [o[1] for o in out if math.isfinite(o[1])]
                total = Counter()
                for _dv, _w, b in out:
                    total.update(b)
                n = max(1, sum(total.values()))
                pct = lambda k: 100.0 * total.get(k, 0) / n
                print(f"{alt:+8.0f} {tilt:5.0f} {np.mean(det_v):11.1f} "
                      f"{np.median(windows):13.2f} {pct('out-TOP'):5.1f}% "
                      f"{pct('out-BOTTOM'):5.1f}% "
                      f"{pct('out-horiz') + pct('out-both'):6.1f}% "
                      f"{pct('inframe-nodecode'):8.1f}%", flush=True)
            print()


if __name__ == "__main__":
    import sys
    if "--fine" in sys.argv:
        # PREREG #2 (chase_tilt_prereg_2026-09-23.md): fine angle sweep.
        # 0/10 deg columns are REUSED from the part-1 run, not reflown.
        TILTS = [8.0, 12.0, 15.0]
        ALT_CELLS = [-2.0, 0.0, 1.0, 2.0, 3.0]
        main_grid()
        mechanism()
    else:
        main_grid()
        mechanism()
