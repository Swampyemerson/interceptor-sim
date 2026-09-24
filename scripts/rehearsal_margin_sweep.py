#!/usr/bin/env python3
"""Rehearsal break-off margin sweep (2026-09-24). PRE-REGISTERED:
isim/specs/rehearsal_breakoff_prereg_2026-09-24.md -- read it before
interpreting output. This is the ROUND 2 sweep (that doc's AMENDMENT #1).
Round 1 (raw range trigger, rehearsal_range_m in {1.5, 2.0, 2.5, 3.0}, no
practice profile; logs/rehearsal_20260924/sweep.txt) is reproducible only
from the git history -- the round-1 trigger no longer exists in the code.

ROUND 2 arms: OFF (the plain default -- the "real pass" twin the would-have
is scored against) and rehearsal ON at rehearsal_t_react_s in {1.5, 2.0,
2.5} s, every ON arm flying the PRACTICE PROFILE (brake package a=3 m/s^2 +
vertical arrival-sync, v_max_ms 10); range upper bound / t_late / gate /
evade at the config defaults. One UNREGISTERED diagnostic arm, `prac_off`
(practice profile, rehearsal off), is flown and printed in its own section
-- never in the registered bars -- because the registered reference (plain
OFF) is not the pass the practice-profile ON arms would actually have flown.

Port arm (real flight code, terminal="pursuit"), rear tag, all v5 errors on
(scatter=Scatter()), tag-realism rung R2 (target attitude + default wobble),
honest ENGAGE vehicle fit, pose-range channel supplied (the fixed Gazebo
driver's PnP range; RealFlightGuidance.supply_pose_range=True), paired seeds
0..49. Cells: nominal, aim20, alt+3 pair (target 3 m above + camera up-tilt 10 deg + tag mount
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
            (a too-late flight is NOT triggered, so it counts AGAINST recall
            -- amendment #1: no gaming the floor)
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
            (round 1's identity check; in round 2 the ON arms fly the
            practice profile, so ON flights legitimately differ from the
            plain OFF twin -- reported as a count, NOT a pass/fail; the
            identity property is pinned by the unit tests instead)
  too_late  flights whose too-late floor latched (never triggered)
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

from flight.pursuit_terminal import PursuitTerminalConfig
from isim.scenario import Scatter, Scenario

N_SEEDS = 50
WORKERS = 10
HIT_M = 0.35
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIT = os.path.join(_ROOT, "isim/fits/vehicle_gazebo_x500_engage.json")
OUT_DIR = os.path.join(_ROOT, "logs/rehearsal_20260924")

T_REACTS = (1.5, 2.0, 2.5)
# Amendment #1 item 3: practice passes fly with the brake package on (the
# --pursuit-brake constants, ADR-0115) and a 10 m/s speed cap.
PRACTICE = {"brake_shaping": True, "brake_accel_ms2": 3.0,
            "brake_vert_sync": True, "v_max_ms": 10.0}
ARMS = {"off": None}
for _tr in T_REACTS:
    ARMS[f"t{_tr:.1f}"] = {"rehearsal_breakoff": True,
                           "rehearsal_t_react_s": _tr, **PRACTICE}
# UNREGISTERED diagnostic arm (flown, printed separately, never in the bars):
# the practice profile with rehearsal OFF -- the pass the ON arms would
# really have flown.
DIAG_ARMS = {"prac_off": dict(PRACTICE)}

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
    overrides = ARMS[arm_name] if arm_name in ARMS else DIAG_ARMS[arm_name]
    scn = Scenario(concept="flyby", terminal="pursuit", tag_facing="rear",
                   scatter=Scatter(), port_pursuit_overrides=overrides,
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
               final_state=sm.state, triggered=False,
               too_late=bool(getattr(term, "rehearsal_too_late", False)))
    if out["too_late"]:
        out.update(t_go_late=float(term.rehearsal_too_late_t_go_s),
                   r_hat_late=float(term.rehearsal_too_late_range_m))
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


def _arm_rows(by, cell_name, arm_name):
    return [by[(cell_name, arm_name, s)] for s in range(N_SEEDS)
            if (cell_name, arm_name, s) in by]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    all_arms = list(ARMS) + list(DIAG_ARMS)
    jobs = [(c, a, s) for c, _ in CELLS for a in all_arms for s in range(N_SEEDS)]
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=WORKERS, mp_context=ctx) as ex:
        rows = list(ex.map(run_one, jobs, chunksize=2))
    with open(os.path.join(OUT_DIR, "flights_round2.json"), "w") as f:
        json.dump(rows, f, indent=1, default=str)

    by = {(r["cell"], r["arm"], r["seed"]): r for r in rows}
    d = PursuitTerminalConfig()
    print(f"rehearsal margin sweep ROUND 2 (amendment #1) -- R2, ENGAGE fit, pose "
          f"range on, n={N_SEEDS} paired seeds/cell; ON arms = t_go trigger + "
          f"practice profile {PRACTICE}; config defaults: gate "
          f"N={d.rehearsal_min_updates} in {d.rehearsal_fresh_s} s, evade "
          f"{d.rehearsal_evade_s} s, t_late {d.rehearsal_t_late_s} s, range "
          f"upper bound {d.rehearsal_range_m} m; reference = plain-default OFF",
          flush=True)
    print(f"{'cell':>8} {'arm':>5} {'med':>6} {'<=.35':>6} {'trig%':>6} "
          f"{'recall':>9} {'sep_min':>8} {'sep_p05':>8} {'sep_med':>8} "
          f"{'hit<.35':>8} {'untrig_hit':>10} {'err_med':>8} {'err_sgn':>8} "
          f"{'ident':>6} {'too_late':>8}", flush=True)
    summary = {}
    for cell_name, _kw in CELLS:
        off = _arm_rows(by, cell_name, "off")
        if len(off) != N_SEEDS:
            print(f"{cell_name:>8} UNCERTAIN -- {len(off)} OFF rows", flush=True)
            continue
        off_miss = np.array([r["miss_m"] for r in off])
        contact_seeds = [s for s in range(N_SEEDS) if off_miss[s] <= HIT_M]
        for arm_name in ARMS:
            arm = _arm_rows(by, cell_name, arm_name)
            if len(arm) != N_SEEDS:
                print(f"{cell_name:>8} {arm_name:>5} UNCERTAIN -- {len(arm)} rows",
                      flush=True)
                continue
            miss = np.array([r["miss_m"] for r in arm])
            head = (f"{cell_name:>8} {arm_name:>5} {np.median(miss):6.3f} "
                    f"{_pct(np.mean(miss <= HIT_M))}")
            if arm_name == "off":
                print(head + f"   (reference: {len(contact_seeds)}/{N_SEEDS} "
                      f"inside {HIT_M} m)", flush=True)
                continue
            trig = [r for r in arm if r["triggered"]]
            n_trig = len(trig)
            n_late = sum(1 for r in arm if r["too_late"])
            # Too-late flights are untriggered, so they count AGAINST recall.
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
                      f"{'--':>8} {'--':>8} {ident_bad:6d} {n_late:8d}", flush=True)
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
                  f"{np.median(err):+8.3f} {ident_bad:6d} {n_late:8d}", flush=True)
            summary[(cell_name, arm_name)] = dict(
                n_trig=n_trig, recall_n=recall_n, n_contact=len(contact_seeds),
                sep_min=float(sep.min()), err_med=float(np.median(np.abs(err))))
        print(flush=True)

    # Registered bars (prereg, unchanged by amendment #1): P1 = worst post-
    # trigger sep >= 0.7 m on ALL cells AND recall >= 80% on ALL cells
    # (too-late flights count against recall), smallest such t_react adopted.
    # P2 = median |ZEM - paired OFF CPA| <= 0.15 m on triggered flights.
    print("registered bars:", flush=True)
    p1_arms = []
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
            p1_arms.append(arm_name)
    print(f"  smallest t_react meeting P1: "
          f"{p1_arms[0] if p1_arms else 'NONE (null branch)'}", flush=True)

    # UNREGISTERED diagnostics (not bars; direction only): where the trigger
    # fired, the too-late t_go, and the would-have error / recall against the
    # PRACTICE-profile OFF twin (`prac_off`: same profile, rehearsal off) --
    # the pass the ON arms would actually have flown, vs the registered
    # plain-default reference above.
    print("\nUNREGISTERED diagnostics (direction only, not bars):", flush=True)
    print(f"{'cell':>8} {'arm':>8} {'med':>6} {'<=.35':>6} {'rhat_med':>8} "
          f"{'rhat_max':>8} {'tgo_med':>7} {'late_tgo':>9} {'rec_prac':>9} "
          f"{'errp_med':>8} {'errp_sgn':>8}", flush=True)
    for cell_name, _kw in CELLS:
        poff = _arm_rows(by, cell_name, "prac_off")
        if len(poff) != N_SEEDS:
            print(f"{cell_name:>8} UNCERTAIN -- {len(poff)} prac_off rows", flush=True)
            continue
        pmiss = np.array([r["miss_m"] for r in poff])
        print(f"{cell_name:>8} {'prac_off':>8} {np.median(pmiss):6.3f} "
              f"{_pct(np.mean(pmiss <= HIT_M))}", flush=True)
        pcontact = [s for s in range(N_SEEDS) if pmiss[s] <= HIT_M]
        for arm_name in ARMS:
            if arm_name == "off":
                continue
            arm = _arm_rows(by, cell_name, arm_name)
            trig = [r for r in arm if r["triggered"]]
            late = [r["t_go_late"] for r in arm if r["too_late"]]
            rec_n = sum(1 for s in pcontact if by[(cell_name, arm_name, s)]["triggered"])
            late_s = f"{np.median(late):9.2f}" if late else f"{'--':>9}"
            if not trig:
                print(f"{cell_name:>8} {arm_name:>8} {'':>6} {'':>6} UNCERTAIN -- 0 "
                      f"triggered {late_s} {rec_n:>4}/{len(pcontact):<4}", flush=True)
                continue
            rh = np.array([r["r_hat_trig"] for r in trig])
            tg = np.array([r["t_go"] for r in trig])
            errp = np.array([r["zem"] - by[(cell_name, "prac_off", r["seed"])]["miss_m"]
                             for r in trig])
            print(f"{cell_name:>8} {arm_name:>8} {'':>6} {'':>6} {np.median(rh):8.2f} "
                  f"{rh.max():8.2f} {np.median(tg):7.2f} {late_s} "
                  f"{f'{rec_n}/{len(pcontact)}':>9} {np.median(np.abs(errp)):8.3f} "
                  f"{np.median(errp):+8.3f}", flush=True)
        print(flush=True)


if __name__ == "__main__":
    main()
