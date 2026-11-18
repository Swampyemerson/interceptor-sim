#!/usr/bin/env python3
"""Tables for isim/specs/flight_hardware_rerun_prereg_2026-11-18.md from the rows
scripts/flight_hardware_rerun.py wrote. Reads only; prints only."""
import collections
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from flight_hardware_rerun import OUT, blind_kill, camera_kill, go_time  # noqa: E402


def load(stage):
    p = os.path.join(OUT, f"{stage}.json")
    if not os.path.exists(p):
        return None
    cells = collections.defaultdict(dict)
    for r in json.load(open(p)):
        cells[r["tag"]][r["seed"]] = r
    return cells


def kills(c):
    return sum(r["miss"] <= 0.35 for r in c.values())


def med(c, k, only_det=False):
    v = [r[k] for r in c.values() if r[k] is not None and (not only_det or r["first_det"] is not None)]
    return float(np.median(v)) if v else float("nan")


def paired(ref, cur):
    lost = sum(1 for s in ref if ref[s]["miss"] <= 0.35 and cur[s]["miss"] > 0.35)
    gained = sum(1 for s in ref if ref[s]["miss"] > 0.35 and cur[s]["miss"] <= 0.35)
    return lost, gained


def why(c):
    out = collections.Counter()
    for r in c.values():
        if r["miss"] <= 0.35:
            continue
        if r["first_det"] is None:
            out["never decoded"] += 1
        elif "target_lost" in r["reasons"]:
            out["target lost"] += 1
        elif "pursuit_miss" in r["reasons"]:
            out["close miss"] += 1
        else:
            out["other"] += 1
    return dict(out)


def stage_a(a):
    print("\n== STAGE a: kills of 16 / median time to closest approach, by brake ==")
    print(f"{'geometry|envelope|drag':>26} | {'a2':>10} | {'a3':>10} | {'a4':>10} | peak v a2/a3/a4 | v at first decode | lean at first decode | late kills(>30 s)")
    for geom in ("near", "demo", "tail"):
        for env in ("now", "lean60"):
            for drag in ("D0", "D1", "D2"):
                cs = [a[f"{geom}|{env}|{drag}|a{b}|t10"] for b in (2, 3, 4)]
                late = [sum(1 for r in c.values() if r["miss"] <= 0.35 and r["t_cpa"] - go_time(r) > 30) for c in cs]
                print(f"{geom + '|' + env + '|' + drag:>26} | "
                      + " | ".join(f"{kills(c):2d}, {med(c, 't_cpa'):5.1f}" for c in cs)
                      + " | " + "/".join(f"{med(c, 'peak_v'):.0f}" for c in cs)
                      + " | " + "/".join(f"{med(c, 'v_first', True):.1f}" for c in cs)
                      + " | " + "/".join(f"{med(c, 'tilt_first', True):.0f}" for c in cs)
                      + " | " + "/".join(str(x) for x in late))
    print("\nmisses in stage a, by reason:")
    for tag, c in a.items():
        w = why(c)
        if w:
            blind = sum(blind_kill(r) for r in c.values())
            print(f"  {tag}: {w}" + (f", blind kills {blind}" if blind else ""))
    print("\nbrake rule (softest brake with the best brake's kills in every D0/D1 cell, within 2.0 s):")
    for b in (2, 3, 4):
        worst_dk, dts = 0, []
        for geom in ("near", "demo", "tail"):
            for env in ("now", "lean60"):
                for drag in ("D0", "D1"):
                    ks = {x: kills(a[f"{geom}|{env}|{drag}|a{x}|t10"]) for x in (2, 3, 4)}
                    ts = {x: med(a[f"{geom}|{env}|{drag}|a{x}|t10"], "t_cpa") for x in (2, 3, 4)}
                    worst_dk = max(worst_dk, max(ks.values()) - ks[b])
                    dts.append(ts[b] - min(ts.values()))
        print(f"  brake {b}: worst kill deficit {worst_dk} of 16; time penalty median {np.median(dts):.1f} s, max {max(dts):.1f} s")


def stage_c(a, c):
    print("\n== STAGE c: kills of 16 by camera tilt (brake 3) ==")
    print(f"{'geometry|envelope|drag':>26} | t10 | t15 | t20 | t25 | paired vs t10 (lost/gained) t15, t20, t25")
    for geom in ("near", "demo", "tail"):
        for env in ("now", "lean60"):
            for drag in ("D1", "D2"):
                ref = a[f"{geom}|{env}|{drag}|a3|t10"]
                cs = [c[f"{geom}|{env}|{drag}|a3|t{t}"] for t in (15, 20, 25)]
                print(f"{geom + '|' + env + '|' + drag:>26} | {kills(ref):3d} | "
                      + " | ".join(f"{kills(x):3d}" for x in cs) + " | "
                      + ", ".join("%d/%d" % paired(ref, x) for x in cs))
    tot = {t: sum(kills(c[f"{g}|{e}|{d}|a3|t{t}"]) for g in ("near", "demo", "tail")
                  for e in ("now", "lean60") for d in ("D1", "D2")) for t in (15, 20, 25)}
    tot[10] = sum(kills(a[f"{g}|{e}|{d}|a3|t10"]) for g in ("near", "demo", "tail")
                  for e in ("now", "lean60") for d in ("D1", "D2"))
    print("  totals of 192:", {t: tot[t] for t in (10, 15, 20, 25)})
    print("misses by reason:")
    for tag, cell in c.items():
        w = why(cell)
        if w:
            print(f"  {tag}: {w}")


def stage_brackets(a, br):
    print("\n== BRACKETS (D1, brake 3, tilt 10): kills of 16 ==")
    for geom in ("near", "demo", "tail"):
        for env in ("now", "lean60"):
            ref = a[f"{geom}|{env}|D1|a3|t10"]
            f = br[f"fx933|{geom}|{env}|D1|a3|t10"]
            s = br[f"sidetag|{geom}|{env}|D1|a3|t10"]
            print(f"  {geom}|{env}: fx 1104 rear {kills(ref)} | fx 933 {kills(f)} (lost/gained %d/%d) | "
                  % paired(ref, f)
                  + f"side-facing tag {kills(s)} (camera kills {sum(camera_kill(r) for r in s.values())}, "
                  f"never decoded {sum(r['first_det'] is None for r in s.values())})")


def stage_b(b, bt):
    print("\n== STAGE b: all errors on, kills of 50 (camera kills / blind / never decoded) ==")
    print(f"{'target|geometry|speed error':>26} | clock 2.0 s          | clock 6.0 s          | paired 6 s vs 2 s lost/gained | late kills 2 s")
    tot = collections.Counter()
    for motion in ("st", "we"):
        for geom in ("near", "demo"):
            for err in ("10", "4", "2", "1", "0"):
                c2 = b[f"{motion}|{geom}|e{err}|T2"]
                c6 = b[f"{motion}|{geom}|e{err}|T6"]
                def cell(c):
                    return (f"{kills(c):2d} ({sum(camera_kill(r) for r in c.values()):2d}/"
                            f"{sum(blind_kill(r) for r in c.values())}/"
                            f"{sum(r['first_det'] is None for r in c.values()):2d})")
                lost, gained = paired(c2, c6)
                late = sum(1 for r in c2.values() if r["miss"] <= 0.35 and r["t_cpa"] - go_time(r) > 30)
                tot[(motion, "k2")] += kills(c2); tot[(motion, "k6")] += kills(c6)
                tot["lost"] += lost; tot["gained"] += gained
                print(f"{motion + '|' + geom + '|' + err + ' %':>26} | {cell(c2):20s} | {cell(c6):20s} | {lost}/{gained} | {late}")
    print("  totals:", dict(tot))
    print("misses at clock 2.0 s by reason:")
    for tag, c in b.items():
        if tag.endswith("T2"):
            print(f"  {tag}: {why(c)}; median time to closest approach {med(c, 't_cpa'):.1f} s; "
                  f"median first decode {med(c, 'first_det', True):.1f} s")
    if bt:
        print("\n== btilt: tilt 20 against tilt 10, straight target, clock 2.0 s ==")
        for geom in ("near", "demo"):
            for err in ("4", "1"):
                ref = b[f"st|{geom}|e{err}|T2"]
                cur = bt[f"st|{geom}|e{err}|T2|t20"]
                print(f"  {geom} {err} %: tilt 10 {kills(ref)} | tilt 20 {kills(cur)} | lost/gained %d/%d" % paired(ref, cur))


if __name__ == "__main__":
    a, c, br, b, bt = (load(s) for s in ("a", "c", "brackets", "b", "btilt"))
    if a:
        stage_a(a)
    if a and c:
        stage_c(a, c)
    if a and br:
        stage_brackets(a, br)
    if b:
        stage_b(b, bt)
