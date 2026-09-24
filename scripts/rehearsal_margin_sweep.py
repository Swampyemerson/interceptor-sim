#!/usr/bin/env python3
"""Rehearsal break-off margin sweep (2026-09-24). PRE-REGISTERED:
isim/specs/rehearsal_breakoff_prereg_2026-09-24.md -- read it before
interpreting output.

Port arm (real flight code, terminal="pursuit"), rear tag, all v5 errors on
(scatter=Scatter()), tag-realism rung R2 (target attitude + default wobble),
honest ENGAGE vehicle fit, pose-range channel supplied (the fixed Gazebo
driver's PnP range; RealFlightGuidance.supply_pose_range=True), paired seeds
0..49. Arms: rehearsal OFF plus rehearsal_breakoff ON at rehearsal_range_m
in {1.5, 2.0, 2.5, 3.0} (gate/evade at the config defaults). Cells: nominal,
aim20, alt+3 pair (target 3 m above + camera up-tilt 10 deg + tag mount
pitch 12 deg).

`mc._run_one` cannot set `supply_pose_range` and records no truth trace on
the flyby/port arm, while the post-trigger separation needs the per-tick
truth range -- so this script runs its own worker (build -> pose on ->
`run_engagement(record_trace=True)`), otherwise the same recipe, under its
own spawn-context pool. Truth (the trace) is read HERE, for scoring only;
the guidance never sees it.

Columns (per cell x arm):
  trig%     flights whose rehearsal trigger fired
  recall    triggered / flights whose PAIRED OFF twin passed inside 0.35 m
  sep_min   worst (min over triggered flights) TRUE separation from the
            trigger tick to the end of the run
  sep_p05   5th percentile of the same per-flight minimum
  sep_med   median of the same
  hit<.35   triggered flights whose post-trigger separation still went
            inside 0.35 m (the evade failed to prevent contact)
  untrig_hit  UNtriggered ON flights that passed inside 0.35 m (practice
            mode did not engage and the pass was a real contact)
  err_med   median |onboard would-have ZEM - paired OFF CPA| (triggered)
  err_sgn   median signed (ZEM - OFF CPA)
  ident     untriggered ON flights whose CPA differs from the OFF twin
            (must be 0: flag-on-but-silent is identical to flag-off)
"""
import contextlib
import dataclasses
import io
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from isim.scenario import Scatter, Scenario

N_SEEDS = 50
WORKERS = 10
HIT_M = 0.35
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIT = os.path.join(_ROOT, "isim/fits/vehicle_gazebo_x500_engage.json")
OUT_DIR = os.path.join(_ROOT, "logs/rehearsal_20260924")

RANGES = (1.5, 2.0, 2.5, 3.0)
ARMS = {"off": None}
for _r in RANGES:
    ARMS[f"r{_r:.1f}"] = {"rehearsal_breakoff": True, "rehearsal_range_m": _r}

# Rung R2 (tag_realism_v1 §F): target attitude + default wobble.
RUNG = dict(target_attitude=True)

CELLS = [
    ("nominal", dict()),
    ("aim20", dict(aim_error_deg=20.0)),
    ("alt+3", dict(target_alt_offset_m=3.0, cam_tilt_up_deg=10.0,
                   tag_mount_pitch_deg=12.0)),
]


def run_one(job):
    """One engagement -> a plain dict (picklable, module-scope for spawn)."""
    cell_name, arm_name, seed = job
    from isim.engine import run_engagement
    from isim.replay_a0 import load_params
    from isim.scenario import build
    cell_kw = dict(CELLS)[cell_name]
    scn = Scenario(concept="flyby", terminal="pursuit", tag_facing="rear",
                   scatter=Scatter(), port_pursuit_overrides=ARMS[arm_name],
                   seed=seed, **cell_kw, **RUNG)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, load_params(FIT))
    guidance.supply_pose_range = True
    with contextlib.redirect_stdout(io.StringIO()):
        res = run_engagement(ecfg, vehicle, target, seeker, guidance, init,
                             record_trace=True)
    term = guidance._sm.guidance          # the terminal of THIS run (reset())
    sm = guidance._sm
    out = dict(cell=cell_name, arm=arm_name, seed=seed, miss_m=float(res.miss_m),
               t_cpa=float(res.t_cpa), safe_reason=sm.safe_reason,
               final_state=sm.state, triggered=False)
    t_trig = getattr(term, "rehearsal_trigger_t", None)
    if t_trig is not None:
        tr = res.trace
        k0 = int(np.searchsorted(tr["t"], t_trig - 1e-9))
        post = tr["range"][k0:]
        out.update(triggered=True, t_trig=float(t_trig),
                   r_hat_trig=float(term.rehearsal_trigger_range_m),
                   zem=float(term.rehearsal_zem_m),
                   t_go=float(term.rehearsal_t_go_s),
                   side=term.rehearsal_side,
                   r_true_trig=float(tr["range"][k0]) if k0 < len(tr["range"]) else math.nan,
                   sep_post=float(post.min()) if post.size else math.nan,
                   evade_done=bool(term.rehearsal_complete))
    return out


def _pct(x):
    return f"{100.0 * x:5.0f}%"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    jobs = [(c, a, s) for c, _ in CELLS for a in ARMS for s in range(N_SEEDS)]
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=WORKERS, mp_context=ctx) as ex:
        rows = list(ex.map(run_one, jobs, chunksize=2))
    with open(os.path.join(OUT_DIR, "flights.json"), "w") as f:
        json.dump(rows, f, indent=1, default=str)

    by = {(r["cell"], r["arm"], r["seed"]): r for r in rows}
    print(f"rehearsal margin sweep -- R2, ENGAGE fit, pose range on, n={N_SEEDS} "
          f"paired seeds/cell; gate N=3 in 0.5 s, evade 1.5 s (config defaults)",
          flush=True)
    print(f"{'cell':>8} {'arm':>5} {'med':>6} {'<=.35':>6} {'trig%':>6} "
          f"{'recall':>9} {'sep_min':>8} {'sep_p05':>8} {'sep_med':>8} "
          f"{'hit<.35':>8} {'untrig_hit':>10} {'err_med':>8} {'err_sgn':>8} "
          f"{'ident':>6}", flush=True)
    summary = {}
    for cell_name, _kw in CELLS:
        off = [by[(cell_name, "off", s)] for s in range(N_SEEDS)]
        if len(off) != N_SEEDS:
            print(f"{cell_name:>8} UNCERTAIN -- {len(off)} OFF rows", flush=True)
            continue
        off_miss = np.array([r["miss_m"] for r in off])
        contact_seeds = [s for s in range(N_SEEDS) if off_miss[s] <= HIT_M]
        for arm_name in ARMS:
            arm = [by[(cell_name, arm_name, s)] for s in range(N_SEEDS)]
            miss = np.array([r["miss_m"] for r in arm])
            head = (f"{cell_name:>8} {arm_name:>5} {np.median(miss):6.3f} "
                    f"{_pct(np.mean(miss <= HIT_M))}")
            if arm_name == "off":
                print(head + f"   (reference: {len(contact_seeds)}/{N_SEEDS} "
                      f"inside {HIT_M} m)", flush=True)
                continue
            trig = [r for r in arm if r["triggered"]]
            n_trig = len(trig)
            recall_n = sum(1 for s in contact_seeds
                           if by[(cell_name, arm_name, s)]["triggered"])
            recall = (f"{recall_n}/{len(contact_seeds)}" if contact_seeds
                      else "0/0")
            untrig = [r for r in arm if not r["triggered"]]
            untrig_hit = sum(1 for r in untrig if r["miss_m"] <= HIT_M)
            ident_bad = sum(1 for r in untrig
                            if r["miss_m"] != by[(cell_name, "off", r["seed"])]["miss_m"])
            if n_trig == 0:
                # NO VACUOUS VERDICTS: zero triggered flights -> no separation
                # or estimate statistic exists; print UNCERTAIN, not a number.
                print(head + f" {_pct(0.0)} {recall:>9}   UNCERTAIN -- 0 "
                      f"triggered flights {'':>24} {untrig_hit:10d} "
                      f"{'--':>8} {'--':>8} {ident_bad:6d}", flush=True)
                summary[(cell_name, arm_name)] = dict(
                    n_trig=0, recall_n=recall_n, n_contact=len(contact_seeds))
                continue
            sep = np.array([r["sep_post"] for r in trig])
            err = np.array([r["zem"] - by[(cell_name, "off", r["seed"])]["miss_m"]
                            for r in trig])
            print(head + f" {_pct(n_trig / N_SEEDS)} {recall:>9} "
                  f"{sep.min():8.3f} {np.percentile(sep, 5):8.3f} "
                  f"{np.median(sep):8.3f} {int(np.sum(sep <= HIT_M)):8d} "
                  f"{untrig_hit:10d} {np.median(np.abs(err)):8.3f} "
                  f"{np.median(err):+8.3f} {ident_bad:6d}", flush=True)
            summary[(cell_name, arm_name)] = dict(
                n_trig=n_trig, recall_n=recall_n, n_contact=len(contact_seeds),
                sep_min=float(sep.min()), err_med=float(np.median(np.abs(err))))
        print(flush=True)

    # Registered bars (prereg): P1 = worst post-trigger sep >= 0.7 m on ALL
    # cells AND recall >= 80% on ALL cells, smallest such range adopted.
    # P2 = median |ZEM - paired OFF CPA| <= 0.15 m on triggered flights.
    print("registered bars:", flush=True)
    p1_ranges = []
    for arm_name in ARMS:
        if arm_name == "off":
            continue
        cells_ok, why = True, []
        for cell_name, _kw in CELLS:
            s = summary.get((cell_name, arm_name))
            if s is None or s["n_trig"] == 0 or s["n_contact"] == 0:
                cells_ok = False
                why.append(f"{cell_name}: UNCERTAIN (no triggered/contact flights)")
                continue
            rec = s["recall_n"] / s["n_contact"]
            ok = s["sep_min"] >= 0.7 and rec >= 0.8
            cells_ok &= ok
            why.append(f"{cell_name}: sep_min {s['sep_min']:.3f} "
                       f"{'>=' if s['sep_min'] >= 0.7 else '<'} 0.7, recall "
                       f"{100 * rec:.0f}% {'>=' if rec >= 0.8 else '<'} 80%")
        p2 = [summary[(c, arm_name)]["err_med"] for c, _ in CELLS
              if summary.get((c, arm_name), {}).get("n_trig", 0) > 0]
        print(f"  {arm_name}: P1 {'PASS' if cells_ok else 'FAIL'} ({'; '.join(why)})"
              f"  | P2 median|err| per cell "
              f"{', '.join(f'{e:.3f}' for e in p2)} -> "
              f"{'PASS' if p2 and len(p2) == len(CELLS) and max(p2) <= 0.15 else 'FAIL'}",
              flush=True)
        if cells_ok:
            p1_ranges.append(arm_name)
    print(f"  smallest range meeting P1: {p1_ranges[0] if p1_ranges else 'NONE (null branch)'}",
          flush=True)


if __name__ == "__main__":
    main()
