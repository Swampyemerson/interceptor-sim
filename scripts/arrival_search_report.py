#!/usr/bin/env python3
"""Tables for isim/specs/arrival_search_prereg_2026-11-18.md from the rows
scripts/arrival_search_sweep.py --stage main wrote. Reads only; no sim.

    python scripts/arrival_search_report.py [rows.json]

Every table counts its denominator. A cell whose row count is not the expected
50 makes the whole report UNCERTAIN (exit 1): a shrinking cell is reported,
never absorbed.
"""
import collections
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT = os.path.join(ROOT, "runs/arrival_search_20261118/main.json")
KILL_M = 0.35
N = 50
ARMS = ["base", "base_T", "base_T_guard", "stern", "believed", "stern_decl04",
        "believed_T2"]
CELLS = [f"{m}|{g}|{e}" for m in ("st", "we") for g in ("near", "demo")
         for e in ("e10", "e04", "e00")]


def kill(r):
    return r["miss"] <= KILL_M


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    rows = json.load(open(path))
    by = collections.defaultdict(dict)
    for r in rows:
        arm, cell = r["tag"].split("|", 1)
        by[(arm, cell)][r["seed"]] = r
    arms = [a for a in ARMS if any((a, c) in by for c in CELLS)]
    short = [(a, c, len(by.get((a, c), {}))) for a in arms for c in CELLS
             if len(by.get((a, c), {})) != N]
    print(f"rows: {len(rows)}; arms: {arms}; cells: {len(CELLS)}; "
          f"cells with a wrong row count: {len(short)}")
    for a, c, n in short:
        print(f"  UNCERTAIN {a}|{c}: {n} rows, expected {N}")

    def cell(a, c):
        return by.get((a, c), {})

    print("\nTABLE 1  kills of 50 (first contact within 45 s in brackets)")
    print(f"{'cell':>14} " + " ".join(f"{a:>13}" for a in arms))
    for c in CELLS:
        out = []
        for a in arms:
            rs = cell(a, c).values()
            k = sum(kill(r) for r in rs)
            k45 = sum(1 for r in rs if kill(r) and r["t_kill"] is not None and r["t_kill"] <= 45.0)
            out.append(f"{k:3d} [{k45:2d}]".rjust(13))
        print(f"{c:>14} " + " ".join(out))

    print("\nTABLE 2  runs that never decoded, of 50")
    print(f"{'cell':>14} " + " ".join(f"{a:>13}" for a in arms))
    for c in CELLS:
        print(f"{c:>14} " + " ".join(
            f"{sum(r['first_det'] is None for r in cell(a, c).values()):13d}" for a in arms))

    print("\nTABLE 3  paired against base_T: kills lost / gained / net (same seeds)")
    others = [a for a in arms if a != "base_T"]
    print(f"{'cell':>14} " + " ".join(f"{a:>13}" for a in others))
    for c in CELLS:
        ref = cell("base_T", c)
        out = []
        for a in others:
            cur = cell(a, c)
            common = [s for s in cur if s in ref]
            lost = sum(1 for s in common if kill(ref[s]) and not kill(cur[s]))
            gain = sum(1 for s in common if kill(cur[s]) and not kill(ref[s]))
            out.append(f"{lost}/{gain}/{gain - lost:+d}".rjust(13))
        print(f"{c:>14} " + " ".join(out))

    print("\nTABLE 4  median time to first contact, s (kills only; n in brackets)")
    print(f"{'cell':>14} " + " ".join(f"{a:>13}" for a in arms))
    for c in CELLS:
        out = []
        for a in arms:
            ts = [r["t_kill"] for r in cell(a, c).values() if kill(r) and r["t_kill"] is not None]
            out.append((f"{np.median(ts):5.1f} [{len(ts):2d}]" if ts else "    - [ 0]").rjust(13))
        print(f"{c:>14} " + " ".join(out))

    print("\nTABLE 5  blind passes (within 3 m before any decode), of 50; "
          "then how many of those runs still ended a kill")
    print(f"{'cell':>14} " + " ".join(f"{a:>13}" for a in arms))
    for c in CELLS:
        out = []
        for a in arms:
            bp = [r for r in cell(a, c).values() if r["blind_pass"]]
            out.append(f"{len(bp):2d} ({sum(kill(r) for r in bp):2d})".rjust(13))
        print(f"{c:>14} " + " ".join(out))

    print("\nTABLE 6  seconds within 5 m before first contact (median / p90)")
    print(f"{'cell':>14} " + " ".join(f"{a:>13}" for a in arms))
    for c in CELLS:
        out = []
        for a in arms:
            v = [r["near_s"] for r in cell(a, c).values()]
            out.append((f"{np.median(v):4.1f}/{np.percentile(v, 90):5.1f}" if v else "-").rjust(13))
        print(f"{c:>14} " + " ".join(out))

    print("\nTABLE 7  what the misses are, per arm, all 12 cells pooled")
    print(f"{'arm':>14} {'runs':>5} {'kills':>5} {'no decode':>9} {'cross>12m':>9} "
          f"{'<=1 m':>6} {'other':>6}")
    for a in arms:
        rs = [r for c in CELLS for r in cell(a, c).values()]
        miss = [r for r in rs if not kill(r)]
        nodec = [r for r in miss if r["first_det"] is None]
        seen = [r for r in miss if r["first_det"] is not None]
        wide = [r for r in seen if r["miss_cross"] is not None and abs(r["miss_cross"]) > 12.0]
        narrow = [r for r in seen if r not in wide and r["miss"] <= 1.0]
        other = len(seen) - len(wide) - len(narrow)
        print(f"{a:>14} {len(rs):5d} {len(rs) - len(miss):5d} {len(nodec):9d} "
              f"{len(wide):9d} {len(narrow):6d} {other:6d}")

    print("\nTABLE 8  search legs flown before the end (median / max), search arms")
    for a in arms:
        legs = [r["search_n_legs"] for c in CELLS for r in cell(a, c).values()]
        if legs and max(legs) > 0:
            print(f"{a:>14} median {np.median(legs):.0f}  max {max(legs)}")

    print("\nTABLE 9  the flying 2.0 s lost-target clock: runs that decoded, did not "
          "kill, and ended target_lost / pursuit_miss on that clock (all 12 cells pooled); "
          "'sparse' = 5 or fewer decodes in the whole run")
    for a in ("base", "believed_T2"):
        if a not in arms:
            continue
        rs = [r for c in CELLS for r in cell(a, c).values()]
        died = [r for r in rs if not kill(r) and r["first_det"] is not None
                and ("target_lost(2.0s)" in r["reasons"] or "target lost 2.0s" in r["reasons"])]
        sparse = [r for r in died if r["n_dec"] <= 5]
        print(f"{a:>14} runs {len(rs)}; decoded-but-died-on-the-clock {len(died)}; "
              f"of those sparse {len(sparse)}")

    print("\nREGISTERED CRITERION (straight target, against base_T): "
          "+15 net at 10 % and +8 net at 4 % in BOTH geometries, "
          "and no worse than -2 net in either 0 % cell")
    for a in ("stern", "believed", "stern_decl04", "believed_T2", "base_T_guard"):
        if a not in arms:
            continue
        net = {}
        for c in ("st|near|e10", "st|demo|e10", "st|near|e04", "st|demo|e04",
                  "st|near|e00", "st|demo|e00"):
            ref, cur = cell("base_T", c), cell(a, c)
            common = [s for s in cur if s in ref]
            net[c] = (sum(1 for s in common if kill(cur[s]) and not kill(ref[s]))
                      - sum(1 for s in common if kill(ref[s]) and not kill(cur[s])))
        ok = (net["st|near|e10"] >= 15 and net["st|demo|e10"] >= 15
              and net["st|near|e04"] >= 8 and net["st|demo|e04"] >= 8
              and net["st|near|e00"] >= -2 and net["st|demo|e00"] >= -2)
        print(f"{a:>14} net: " + ", ".join(f"{k.split('|', 1)[1]} {v:+d}" for k, v in net.items())
              + f"  -> {'MEETS' if ok else 'does not meet'}")
    return 1 if short else 0


if __name__ == "__main__":
    sys.exit(main())
