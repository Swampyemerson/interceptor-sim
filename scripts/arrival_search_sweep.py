#!/usr/bin/env python3
"""Arrival search at a long cue (isim, real flight code port arm).
PRE-REGISTERED: isim/specs/arrival_search_prereg_2026-11-18.md -- read it
before interpreting output.

Same setup as Stage 2 / 2b of scripts/approach_dash_drag_sweep.py (imported,
not copied): 500 m cue, profile 36 / 4, plant R with the tilt-saturation repair
on, no body drag (D0), all errors on, rear 0.5 m tag at fx 540.3. The only
things varied are the guidance arm (`ARMS`: PursuitTerminalConfig overrides)
and the TRUE believed-speed error (`SPEED_ERR`: a Scatter override; guidance
is never told which level is flying).

  --stage check   reproduce Stage 2b / Stage 2 baseline rows (must match)
  --stage dev     development probes on seeds 1000+ (never the test seeds)
  --stage main    the registered matrix, seeds 0..49
Rows land in runs/arrival_search_20261118/.
"""
import argparse
import contextlib
import dataclasses
import io
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import numpy as np

import approach_dash_drag_sweep as base
from isim.engine import run_engagement
from isim.replay_a0 import load_params
from isim.scenario import build

OUT = os.path.join(ROOT, "runs/arrival_search_20261118")
# Baseline rows this runner must reproduce. Read from this checkout when
# present, else from the main checkout (the worktree shares its history).
REF_DIRS = [os.path.join(ROOT, "runs/approach_dash_drag_20261118"),
            "/home/emerson/interceptor-sim/runs/approach_dash_drag_20261118"]

KILL_M = 0.35
BLIND_PASS_M = 3.0      # "passed within a few metres without having decoded"
NEAR_M = 5.0            # "time near the target"

# TRUE 1-sigma error of the believed target speed (Scatter override).
SPEED_ERR = {"e10": 0.10, "e04": 0.04, "e00": 0.0}

# The wrapper's lost-target clock (MissionConfig.engage_lost_target_s, 2.0 s as
# flown) ends the engagement that long after the LAST decode, whatever the
# terminal is doing. Development probes (seeds 1000+) showed one or two stray
# decodes at 25 to 35 m during the approach arm that clock and abort the search
# before it starts; at 30 s it still cut a search 2 s before it re-acquired.
# A search treats "lost" as a normal state, so the search arms run with the
# clock set to the whole window (it never fires before the engagement's own
# time bound); `base_T` is the baseline under the same clock, so the search's
# own effect is not confused with the clock's. The wrapper itself
# (flight/deploy/real_flight.py) is not changed here.
LONG_CLOCK_S = 120.0

# Guidance arms: `port` = PursuitTerminalConfig overrides on top of the Stage 2
# profile; `mission` = MissionConfig overrides. The declared speed sigma is
# 10 % in every search arm except the *_decl04 arms, whatever error is really
# flying -- the declared value is a GIVEN, not a readout.
_TL = dict(engage_lost_target_s=LONG_CLOCK_S)
ARMS = {
    "base": dict(port={}, mission=None),
    "base_T": dict(port={}, mission=_TL),
    "stern": dict(port=dict(arrival_search=True, search_start="stern"), mission=_TL),
    "believed": dict(port=dict(arrival_search=True, search_start="believed"),
                     mission=_TL),
    "stern_decl04": dict(port=dict(arrival_search=True, search_start="stern",
                                   search_sigma_speed_frac=0.04), mission=_TL),
    # Search ON under the flying 2.0 s clock: shows what the clock costs it.
    "believed_T2": dict(port=dict(arrival_search=True, search_start="believed"),
                        mission=None),
}


def _scenario(geom, motion, err, arm, seed):
    over = dict(base.PLANTS["R"])
    over["drag_quad_horiz"] = base.DRAG["D0"]
    return base.scenario(
        geom, 36, 4, seed, veh_over=over, scatter=True,
        extra_port=dict(ARMS[arm]["port"]),
        extra_scn=dict(target_motion=motion),
        scatter_over={"target_speed_belief_sigma_frac": SPEED_ERR[err]})


def run_one(job):
    """One engagement -> one flat row. Top-level so a spawn worker can import it."""
    tag, scn, mission_over = job
    vp = load_params(base.FIT)
    ecfg, vehicle, target, seeker, guidance, init_state = build(scn, vp)
    if mission_over:
        # MissionConfig overrides: rebuild the state machine on the changed
        # config before anything runs (same mechanism as the drag sweep).
        guidance.cfg = dataclasses.replace(guidance.cfg, **mission_over)
        guidance.reset()
    guidance.supply_pose_range = True
    with contextlib.redirect_stdout(io.StringIO()):
        res = run_engagement(ecfg, vehicle, target, seeker, guidance, init_state,
                             record_trace=True)
    tr = res.trace
    n = int(np.count_nonzero(tr["t"])) + 1
    t = tr["t"][:n]
    rng = tr["range"][:n]
    det = tr["det_new"][:n] > 0.5
    v = np.linalg.norm(tr["own_vel"][:n], axis=1)
    dt = float(np.median(np.diff(t))) if n > 2 else 0.0
    row = dict(tag=tag, seed=int(scn.seed), miss=float(res.miss_m),
               t_cpa=float(res.t_cpa), miss_h=float(res.miss_horiz_m),
               miss_v=float(res.miss_vert_m), n_dec=int(res.n_decoded),
               n_in_fov=int(res.n_in_fov), peak_v=float(v.max()),
               closing=float(res.closing_speed_ms), t_end=float(t[-1]))
    k = int(np.argmin(np.abs(t - res.t_cpa)))
    tv = tr["tgt_vel"][k, :2]
    tvn = float(np.linalg.norm(tv))
    if tvn > 1e-6:
        u = tv / tvn
        mv = np.asarray(res.miss_vec_ned, dtype=float)[:2]
        row["miss_along"] = float(mv @ u)
        row["miss_cross"] = float(u[0] * mv[1] - u[1] * mv[0])
    else:
        row["miss_along"] = row["miss_cross"] = None
    if det.any():
        i = int(np.argmax(det))
        row.update(first_det=float(t[i]), range_first=float(rng[i]),
                   v_first=float(v[i]))
        pre = rng[:i] if i > 0 else rng[:1]
    else:
        row.update(first_det=None, range_first=None, v_first=None)
        pre = rng
    # Closest the vehicle came to the target BEFORE it had decoded anything
    # (the whole run when it never decoded): the blind-pass instrument.
    j = int(np.argmin(pre))
    row["blind_min_range"] = float(pre[j])
    row["blind_min_t"] = float(t[j])
    row["blind_pass"] = bool(pre[j] <= BLIND_PASS_M)
    # First contact: the first sample inside the kill radius (the closest
    # approach itself when only the interpolated miss is inside it). A real
    # engagement ends there; the sim flies on, so "time near the target" is
    # counted up to first contact, or over the whole run when there is none.
    hit = np.flatnonzero(rng <= KILL_M)
    if hit.size:
        row["t_kill"] = float(t[hit[0]])
        stop = int(hit[0])
    elif res.miss_m <= KILL_M:
        row["t_kill"] = float(res.t_cpa)
        stop = k
    else:
        row["t_kill"] = None
        stop = n
    row["near_s"] = float(np.count_nonzero(rng[:stop] <= NEAR_M) * dt)
    sm = getattr(guidance, "_sm", None)
    term = getattr(sm, "guidance", None)
    row["search_leg_end"] = getattr(term, "search_leg", None)
    row["search_n_legs"] = int(getattr(term, "search_n_legs", 0))
    row["reasons"] = " || ".join(f"{tr_.to}@{tr_.t:.1f}: {tr_.reason}"
                                 for tr_ in getattr(sm, "transitions", []))[:600]
    return row


def run_jobs(jobs, workers):
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as ex:
        return list(ex.map(run_one, jobs, chunksize=1))


def _ref_rows(name):
    for d in REF_DIRS:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return json.load(open(p))
    raise SystemExit(f"reference rows {name} not found in {REF_DIRS}")


def stage_check(workers):
    """The baseline arm IS Stage 2b's (straight) and Stage 2's (weave) cells:
    same seeds, same config. Any difference means this runner is not the
    instrument that produced the result it is trying to improve."""
    ref2b = {(r["tag"], r["seed"]): r for r in _ref_rows("stage2b.json")}
    ref2 = {(r["tag"], r["seed"]): r for r in _ref_rows("stage2.json")}
    pairs = []
    for geom in ("near", "demo"):
        for err, pkg in (("e10", "full"), ("e04", "speed_belief_4pct"),
                         ("e00", "no_speed_belief")):
            for s in range(6):
                pairs.append((("straight", geom, err, s), ref2b[(f"{geom}|{pkg}", s)]))
        for s in range(6):
            pairs.append((("weave", geom, "e10", s), ref2[(f"we|{geom}|v36a4|D0", s)]))
    jobs = [(f"{m}|{g}|{e}|{s}", _scenario(g, m, e, "base", s), None)
            for (m, g, e, s), _ in pairs]
    rows = run_jobs(jobs, workers)
    bad = 0
    for row, (_key, ref) in zip(rows, pairs):
        same = (abs(row["miss"] - ref["miss"]) < 1e-9
                and abs(row["t_cpa"] - ref["t_cpa"]) < 1e-9
                and row["n_dec"] == ref["n_dec"])
        bad += 0 if same else 1
        if not same:
            print(f"DIFFERS {row['tag']}: miss {row['miss']:.6f} vs {ref['miss']:.6f}, "
                  f"t_cpa {row['t_cpa']:.4f} vs {ref['t_cpa']:.4f}, "
                  f"n_dec {row['n_dec']} vs {ref['n_dec']}")
    print(f"INSTRUMENT CHECK {'PASS' if not bad else 'FAIL'}: "
          f"{len(rows) - bad} of {len(rows)} baseline rows match the stored "
          f"Stage 2 / 2b rows exactly")
    return 1 if bad or len(rows) != len(pairs) else 0


def summarize(rows, expect_n, base_arm="base_T"):
    """One line per cell, with paired kill discordance (lost / gained / net)
    against `base_arm` in the same (motion, geometry, error) cell. `k45` =
    kills whose first contact came within 45 s. A cell with the wrong row
    count is UNCERTAIN, never a pass."""
    cells = {}
    for r in rows:
        cells.setdefault(r["tag"], {})[r["seed"]] = r
    print(f"{'cell':>34} {'n':>3} {'kill':>6} {'k45':>3} {'nodec':>5} {'med':>7} {'p90':>7} "
          f"{'t_kill':>6} {'t_det':>6} {'blind':>5} {'near_s':>6} "
          f"{'lost':>4} {'gain':>4} {'net':>4}")
    bad = 0
    for tag in cells:
        rs = cells[tag]
        if len(rs) != expect_n:
            print(f"{tag:>34} UNCERTAIN -- {len(rs)} rows, expected {expect_n}")
            bad += 1
            continue
        arm, cell = tag.split("|", 1)
        m = np.array([r["miss"] for r in rs.values()])
        det = [r for r in rs.values() if r["first_det"] is not None]
        kills = [r for r in rs.values() if r["miss"] <= KILL_M]
        ref = cells.get(f"{base_arm}|{cell}")
        if ref is not None and len(ref) == expect_n and arm != base_arm:
            lost = sum(1 for s in rs if ref[s]["miss"] <= KILL_M < rs[s]["miss"])
            gain = sum(1 for s in rs if rs[s]["miss"] <= KILL_M < ref[s]["miss"])
            disc = f"{lost:4d} {gain:4d} {gain - lost:+4d}"
        else:
            disc = f"{'':>4} {'':>4} {'':>4}"
        med = lambda k, src: (float(np.median([r[k] for r in src])) if src else float("nan"))
        k45 = sum(1 for r in kills if r["t_kill"] is not None and r["t_kill"] <= 45.0)
        print(f"{tag:>34} {len(rs):3d} {len(kills):3d}/{len(rs):<2d} {k45:3d} "
              f"{len(rs) - len(det):5d} {np.median(m):7.3f} {np.percentile(m, 90):7.3f} "
              f"{med('t_kill', kills):6.1f} {med('first_det', det):6.1f} "
              f"{sum(r['blind_pass'] for r in rs.values()):5d} "
              f"{med('near_s', list(rs.values())):6.1f} {disc}")
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, choices=["check", "dev", "main"])
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--n", type=int, default=None)
    ap.add_argument("--ref", default="base_T",
                    help="arm the kill discordance is paired against")
    args = ap.parse_args()
    if args.stage == "check":
        return stage_check(args.workers)
    arms = [a for a in args.arms.split(",") if a]
    for a in arms:
        if a not in ARMS:
            raise SystemExit(f"unknown arm {a!r}; have {sorted(ARMS)}")
    if args.stage == "dev":
        # Development only: DISJOINT seeds, so nothing chosen here was tuned
        # on the registered seeds 0..49.
        n = args.n or 12
        seeds = range(1000, 1000 + n)
        cells = [("straight", "near", "e10"), ("straight", "demo", "e10"),
                 ("straight", "near", "e00")]
    else:
        n = args.n or 50
        seeds = range(n)
        cells = [(m, g, e) for m in ("straight", "weave") for g in ("near", "demo")
                 for e in SPEED_ERR]
    jobs = [(f"{arm}|{m[:2]}|{g}|{e}", _scenario(g, m, e, arm, s), ARMS[arm]["mission"])
            for (m, g, e) in cells for arm in arms for s in seeds]
    rows = run_jobs(jobs, args.workers)
    os.makedirs(OUT, exist_ok=True)
    name = f"{args.stage}.json" if args.stage == "main" else f"dev_{'_'.join(arms)}.json"
    json.dump(rows, open(os.path.join(OUT, name), "w"))
    bad = summarize(rows, n, base_arm=args.ref)
    if len(rows) != len(jobs) or bad:
        print(f"UNCERTAIN: {len(rows)} rows for {len(jobs)} jobs, {bad} short cells")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
