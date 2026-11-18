#!/usr/bin/env python3
"""Amendment 1 stages of isim/specs/flight_hardware_rerun_prereg_2026-11-18.md:
the lean-limit x camera-tilt grid with picture-loss accounting, the lost-target
rule arms, and the arrival-search arms, all on the flying optics.

  --stage grid     no errors: lean 45/52/60 x tilt 10..45 x 3 geometries, 16 seeds
  --stage gridb    all errors, 32 seeds: the grid's top three pairs + (45, 10)
  --stage lost     all errors, 32 seeds: clock 20 s, 120 s, 120 s + close rule off
  --stage search   all errors, 32 seeds: arrival search off / stern / believed
  --report         print every table from the saved rows
Rows land in runs/flight_hardware_rerun_20261118/<stage>.json.

Picture-loss numbers come from the sim's truth-side frame reports. They are
scoring only; nothing here reaches guidance.
"""
import collections
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import numpy as np                                                     # noqa: E402
import approach_dash_drag_sweep as sw                                  # noqa: E402
from approach_dash_drag_sweep import run_jobs, scenario                # noqa: E402
import flight_hardware_rerun as fh                                     # noqa: E402
from flight_hardware_rerun import OUT, OPTICS, blind_kill, camera_kill, go_time  # noqa: E402

DECODE_RANGE_M = fh.FX_CAL * fh.TAG_M / 8.0      # 8 px floor: 48 m
LEANS = {45: dict(fh._BASE, max_tilt_deg=45.0), 52: dict(fh._BASE, max_tilt_deg=52.0),
         60: dict(fh._BASE, max_tilt_deg=60.0)}
TILTS = (10, 15, 20, 25, 30, 35, 45)

# ---- capture the engagement result so the unchanged sweep.run_one can be reused
_LAST = {}
_orig_run_engagement = sw.run_engagement


def _capturing_run_engagement(*a, **k):
    res = _orig_run_engagement(*a, **k)
    _LAST["res"] = res
    return res


sw.run_engagement = _capturing_run_engagement


def _gap_stats(flags, ts):
    """flags: True where the tag is OUT of the picture. -> (share, longest gap s)."""
    if not len(flags):
        return None, None
    longest, start = 0.0, None
    for f, t in zip(flags, ts):
        if f and start is None:
            start = t
        if not f and start is not None:
            longest = max(longest, t - start)
            start = None
    if start is not None:
        longest = max(longest, ts[-1] - start)
    return float(np.mean(flags)), float(longest)


def run_one_fov(job):
    row = sw.run_one(job)
    res = _LAST.pop("res")
    reps = res.frame_reports
    tr = res.trace
    n = int(np.count_nonzero(tr["t"])) + 1
    tt = tr["t"][:n]
    dt = float(tt[1] - tt[0]) if n > 1 else 0.005
    q = tr["quat"][:n]
    pitch = np.degrees(np.arcsin(np.clip(2.0 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))
    rng = tr["range"][:n]
    t_rep = np.array([r.t_capture for r in reps])
    idx = np.clip(np.round(t_rep / dt).astype(int), 0, n - 1)
    out = np.array([not r.in_fov for r in reps])
    dec = np.array([r.decoded for r in reps])
    t_fd = float(t_rep[np.argmax(dec)]) if dec.any() else None
    # (a) the chase: first decode -> closest approach
    if t_fd is not None and res.t_cpa > t_fd:
        m = (t_rep >= t_fd) & (t_rep <= res.t_cpa)
        share, gap = _gap_stats(out[m], t_rep[m])
        row.update(chase_frames=int(m.sum()), chase_out=share, chase_gap_s=gap,
                   chase_out_pitch=(float(np.median(pitch[idx][m & out])) if (m & out).any() else None))
    else:
        row.update(chase_frames=0, chase_out=None, chase_gap_s=None, chase_out_pitch=None)
    # (b) arrival: target inside decode range, not yet decoded
    m = (rng[idx] <= DECODE_RANGE_M) & ((t_rep < t_fd) if t_fd is not None else np.ones(len(t_rep), bool))
    share, gap = _gap_stats(out[m], t_rep[m])
    row.update(arr_frames=int(m.sum()), arr_out=share, arr_gap_s=gap,
               arr_out_pitch=(float(np.median(pitch[idx][m & out])) if (m & out).any() else None))
    return row


def run_jobs_fov(jobs, workers):
    from concurrent.futures import ProcessPoolExecutor
    from multiprocessing import get_context
    with ProcessPoolExecutor(max_workers=workers, mp_context=get_context("spawn")) as ex:
        return list(ex.map(run_one_fov, jobs, chunksize=1))


def jobs_for(tag, geom, seeds, lean, tilt, *, scatter=False, speed_err=None,
             motion="straight", clock=None, extra_port=None, a=3, drag="D1"):
    over = dict(LEANS[lean])
    over["drag_quad_horiz"] = sw.DRAG[drag]
    extra = dict(OPTICS)
    extra["cam_tilt_up_deg"] = float(tilt)
    extra["target_motion"] = motion
    so = None if speed_err is None else {"target_speed_belief_sigma_frac": float(speed_err)}
    out = []
    for s in seeds:
        scn = scenario(geom, fh.DASH_CAP, a, s, veh_over=over, scatter=scatter,
                       extra_port=extra_port, extra_scn=extra, scatter_over=so)
        out.append((tag, scn) if clock is None
                   else (tag, scn, dict(engage_lost_target_s=float(clock))))
    return out


def cells_of(stage):
    p = os.path.join(OUT, f"{stage}.json")
    if not os.path.exists(p):
        return None
    c = collections.defaultdict(dict)
    for r in json.load(open(p)):
        c[r["tag"]][r["seed"]] = r
    return c


def kills(c):
    return sum(r["miss"] <= 0.35 for r in c.values())


def late(c, s):
    return sum(1 for r in c.values() if r["miss"] <= 0.35 and r["t_cpa"] - go_time(r) > s)


def mean_of(c, k):
    v = [r[k] for r in c.values() if r.get(k) is not None]
    return float(np.mean(v)) if v else float("nan")


def grid_ranking(grid):
    rank = []
    for lean in LEANS:
        for tilt in TILTS:
            cs = [grid[f"{g}|L{lean}|t{tilt}"] for g in fh.GEOMS]
            k = sum(kills(c) for c in cs)
            co = float(np.nanmean([mean_of(c, "chase_out") for c in cs]))
            rank.append((-k, co if not math.isnan(co) else 9.0, tilt, lean))
    rank.sort()
    return [(lean, tilt, -nk, co) for nk, co, tilt, lean in rank]


def gridb_pairs(grid):
    pairs = [(l, t) for l, t, _, _ in grid_ranking(grid)[:3]]
    if (45, 10) not in pairs:
        pairs.append((45, 10))
    else:
        pairs.append(next((l, t) for l, t, _, _ in grid_ranking(grid) if (l, t) not in pairs))
    return pairs


def paired(ref, cur):
    seeds = [s for s in cur if s in ref]
    lost = sum(1 for s in seeds if ref[s]["miss"] <= 0.35 and cur[s]["miss"] > 0.35)
    gained = sum(1 for s in seeds if ref[s]["miss"] > 0.35 and cur[s]["miss"] <= 0.35)
    return lost, gained


def report():
    grid, gridb, lost, search, b = (cells_of(s) for s in ("grid", "gridb", "lost", "search", "b"))
    if grid:
        print("== GRID (no errors, D1, brake 3): kills of 48 over three geometries; "
              "share of frames with the tag out of the picture ==")
        print(f"{'lean':>4} {'tilt':>4} | {'kills':>5} {'near':>4} {'demo':>4} {'tail':>4} | "
              f"{'chase out':>9} {'gap s':>6} {'pitch':>6} | {'arrival out':>11} {'pitch':>6} | {'t_cpa':>6} | late 30/45/60")
        for lean in LEANS:
            for tilt in TILTS:
                cs = [grid[f"{g}|L{lean}|t{tilt}"] for g in fh.GEOMS]
                allr = {i: r for i, r in enumerate(r for c in cs for r in c.values())}
                print(f"{lean:4d} {tilt:4d} | {sum(kills(c) for c in cs):5d} "
                      + " ".join(f"{kills(c):4d}" for c in cs)
                      + f" | {100 * mean_of(allr, 'chase_out'):8.1f}% {mean_of(allr, 'chase_gap_s'):6.2f} "
                      f"{mean_of(allr, 'chase_out_pitch'):6.1f} | {100 * mean_of(allr, 'arr_out'):10.1f}% "
                      f"{mean_of(allr, 'arr_out_pitch'):6.1f} | {np.median([r['t_cpa'] for r in allr.values()]):6.1f} | "
                      f"{late(allr, 30)}/{late(allr, 45)}/{late(allr, 60)}")
        print("ranking (kills, then chase out share, tilt, lean):",
              [(l, t, k, round(100 * co, 1)) for l, t, k, co in grid_ranking(grid)[:6]])
        print("pairs for gridb:", gridb_pairs(grid))
    if gridb:
        print("\n== GRIDB (all errors, speed error 4 %, clock 120 s, 32 seeds): kills of 32 ==")
        pairs = sorted({tuple(int(x[1:]) for x in t.split("|")[2:4]) for t in gridb})
        for lean, tilt in pairs:
            cs = {k: gridb[f"{m}|{g}|L{lean}|t{tilt}"] for m in ("st", "we") for g in ("near", "demo")
                  for k in [f"{m}|{g}"]}
            allr = {i: r for i, r in enumerate(r for c in cs.values() for r in c.values())}
            print(f"  lean {lean} tilt {tilt}: total {sum(kills(c) for c in cs.values())} of 128 | "
                  + " ".join(f"{k} {kills(c)}" for k, c in cs.items())
                  + f" | camera kills {sum(camera_kill(r) for r in allr.values())}, blind "
                  f"{sum(blind_kill(r) for r in allr.values())}, never decoded "
                  f"{sum(r['first_det'] is None for r in allr.values())} | chase out "
                  f"{100 * mean_of(allr, 'chase_out'):.1f}% | late 30/45/60 {late(allr, 30)}/{late(allr, 45)}/{late(allr, 60)}")
    if lost and b:
        print("\n== LOST-TARGET RULE (all errors, envelope now, tilt 10, first 32 seeds): kills of 32 ==")
        print(f"{'cell':>16} | T2 | T6 | T20 | T120 | T120 close-rule off | paired T120 vs T2 lost/gained")
        tot = collections.Counter()
        for m in ("st", "we"):
            for g in ("near", "demo"):
                for e in ("4", "1"):
                    ref2 = {s: r for s, r in b[f"{m}|{g}|e{e}|T2"].items() if s < 32}
                    ref6 = {s: r for s, r in b[f"{m}|{g}|e{e}|T6"].items() if s < 32}
                    c20, c120, cx = (lost[f"{m}|{g}|e{e}|{x}"] for x in ("T20", "T120", "T120x"))
                    for k, c in (("T2", ref2), ("T6", ref6), ("T20", c20), ("T120", c120), ("T120x", cx)):
                        tot[k] += kills(c)
                    lo, ga = paired(ref2, c120)
                    print(f"{m + '|' + g + '|e' + e:>16} | {kills(ref2):2d} | {kills(ref6):2d} | {kills(c20):3d} | "
                          f"{kills(c120):4d} | {kills(cx):4d} | {lo}/{ga}")
        print("  totals of 256:", dict(tot))
        allr = {i: r for i, r in enumerate(r for t, c in lost.items() if t.endswith("T120") for r in c.values())}
        print(f"  T120: late kills 30/45/60 s: {late(allr, 30)}/{late(allr, 45)}/{late(allr, 60)}; "
              f"median time to closest approach of kills "
              f"{np.median([r['t_cpa'] - go_time(r) for r in allr.values() if r['miss'] <= 0.35]):.1f} s after GO")
    if search:
        print("\n== ARRIVAL SEARCH (all errors, tilt 10, clock 120 s, 32 seeds): kills of 32 "
              "(camera kills / never decoded) ==")
        print(f"{'cell':>22} | {'off':>12} | {'stern':>12} | {'believed':>12} | stern vs off lost/gained | t_cpa off/stern")
        tot = collections.Counter()
        for lean in (45, 60):
            for m in ("st", "we"):
                for g in ("near", "demo"):
                    for e in ("10", "4", "1"):
                        key = f"{m}|{g}|e{e}|L{lean}"
                        if f"{key}|off" not in search:
                            continue
                        cs = {x: search[f"{key}|{x}"] for x in ("off", "stern", "believed")}
                        def cell(c):
                            return (f"{kills(c):2d} ({sum(camera_kill(r) for r in c.values()):2d}/"
                                    f"{sum(r['first_det'] is None for r in c.values()):2d})")
                        for x, c in cs.items():
                            tot[(lean, e, x)] += kills(c)
                        tk = lambda c: np.median([r["t_cpa"] - go_time(r) for r in c.values() if r["miss"] <= 0.35] or [float("nan")])  # noqa: E731
                        lo, ga = paired(cs["off"], cs["stern"])
                        print(f"{key:>22} | {cell(cs['off']):>12} | {cell(cs['stern']):>12} | {cell(cs['believed']):>12} | "
                              f"{lo}/{ga} | {tk(cs['off']):.0f}/{tk(cs['stern']):.0f}")
        print("  totals:", {f"L{k[0]} e{k[1]} {k[2]}": v for k, v in sorted(tot.items())})
        allr = {i: r for i, r in enumerate(r for t, c in search.items() if t.endswith("stern") for r in c.values())}
        print(f"  stern arms: late kills 30/45/60 s: {late(allr, 30)}/{late(allr, 45)}/{late(allr, 60)} of "
              f"{sum(r['miss'] <= 0.35 for r in allr.values())}; blind kills {sum(blind_kill(r) for r in allr.values())}")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", choices=["grid", "gridb", "lost", "search"])
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--workers", type=int, default=12)
    args = ap.parse_args()
    if args.report:
        report()
        return 0
    os.makedirs(OUT, exist_ok=True)
    jobs, fov = [], False
    if args.stage == "grid":
        n, fov = 16, True
        for lean in LEANS:
            for tilt in TILTS:
                for g in fh.GEOMS:
                    jobs += jobs_for(f"{g}|L{lean}|t{tilt}", g, range(n), lean, tilt)
    elif args.stage == "gridb":
        n, fov = 32, True
        for lean, tilt in gridb_pairs(cells_of("grid")):
            for m in ("straight", "weave"):
                for g in ("near", "demo"):
                    jobs += jobs_for(f"{m[:2]}|{g}|L{lean}|t{tilt}", g, range(n), lean, tilt,
                                     scatter=True, speed_err=0.04, motion=m, clock=120.0)
    elif args.stage == "lost":
        n = 32
        for m in ("straight", "weave"):
            for g in ("near", "demo"):
                for e in (0.04, 0.01):
                    base = f"{m[:2]}|{g}|e{e * 100:g}"
                    kw = dict(scatter=True, speed_err=e, motion=m)
                    jobs += jobs_for(f"{base}|T20", g, range(n), 45, 10, clock=20.0, **kw)
                    jobs += jobs_for(f"{base}|T120", g, range(n), 45, 10, clock=120.0, **kw)
                    jobs += jobs_for(f"{base}|T120x", g, range(n), 45, 10, clock=120.0,
                                     extra_port=dict(no_fallback_range_m=0.0), **kw)
    else:  # search
        n = 32
        modes = {"off": None, "stern": dict(arrival_search=True, search_start="stern"),
                 "believed": dict(arrival_search=True, search_start="believed")}
        for lean in (45, 60):
            for m in (("straight", "weave") if lean == 45 else ("straight",)):
                for g in ("near", "demo"):
                    for e in (0.10, 0.04, 0.01):
                        for name, port in modes.items():
                            jobs += jobs_for(f"{m[:2]}|{g}|e{e * 100:g}|L{lean}|{name}", g, range(n),
                                             lean, 10, scatter=True, speed_err=e, motion=m,
                                             clock=120.0, extra_port=port)
    rows = run_jobs_fov(jobs, args.workers) if fov else run_jobs(jobs, args.workers)
    json.dump(rows, open(os.path.join(OUT, f"{args.stage}.json"), "w"))
    cells = collections.Counter(r["tag"] for r in rows)
    short = [t for t, c in cells.items() if c != n]
    print(f"{args.stage}: {len(rows)} rows for {len(jobs)} jobs, {len(cells)} cells, short cells: {short}")
    return 1 if (short or len(rows) != len(jobs)) else 0


if __name__ == "__main__":
    sys.exit(main())
