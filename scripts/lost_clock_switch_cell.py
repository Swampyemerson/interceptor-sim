#!/usr/bin/env python3
"""Lost-target clock switches: one paired fast-sim cell.
PRE-REGISTERED: isim/specs/lost_clock_switches_prereg_2026-11-18.md -- read it
before interpreting output.

Stage 2's cell "st|near|v36a4|D0" (straight target, near geometry, profile
36 / 4, plant R, no drag, all errors on, seeds 0..49) through the real flight
code, with the arrival search off or on and the lost-target rule varied through
MissionConfig. Rows land in runs/lost_clock_switches_20261118/cell.json.
"""
import collections
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import approach_dash_drag_sweep as base  # noqa: E402

OUT = os.path.join(base.ROOT, "runs/lost_clock_switches_20261118")
N = 50
STERN = dict(arrival_search=True, search_start="stern")
A = dict(pursuit_lost_clock_needs_track=True)
C = dict(pursuit_lost_clock_off_in_search=True)
B = dict(engage_far_lost_target_s=6.0)
ARMS = {
    "A0 base, 2 s clock": (None, None),
    "A1 base, (a)+(b)": (None, {**A, **B}),
    "A2 search, 2 s clock": (STERN, None),
    "A3 search, 120 s clock": (STERN, dict(engage_lost_target_s=120.0)),
    "A4 search, preset (a)+(c)+(b)": (STERN, {**A, **C, **B}),
    "A5 search, (a)+(c)": (STERN, {**A, **C}),
}


def jobs_for(arm):
    port, mission = ARMS[arm]
    over = dict(base.PLANTS["R"])
    over["drag_quad_horiz"] = base.DRAG["D0"]
    out = []
    for s in range(N):
        scn = base.scenario("near", 36, 4, s, veh_over=over, scatter=True,
                            extra_port=(dict(port) if port else None),
                            extra_scn=dict(target_motion="straight"))
        out.append((arm, scn, dict(mission)) if mission else (arm, scn))
    return out


def end_kind(r):
    if r["miss"] <= 0.35:
        return "kill"
    if r["first_det"] is None:
        return "never decoded"
    why = r["reasons"]
    for key in ("target_lost", "pursuit_miss", "engage_timeout", "mission_timeout"):
        if key in why.split(" || ")[-1] or key in why:
            return key
    return "still engaged at the window's end"


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    jobs = [j for arm in ARMS for j in jobs_for(arm)]
    rows = base.run_jobs(jobs, 12)
    json.dump(rows, open(os.path.join(OUT, "cell.json"), "w"))
    by = collections.defaultdict(dict)
    for r in rows:
        by[r["tag"]][r["seed"]] = r
    ok = True
    # instrument check against the stored Stage 2 rows
    stored = {r["seed"]: r for r in json.load(open(os.path.join(
        base.OUT, "stage2.json"))) if r["tag"] == "st|near|v36a4|D0"}
    a0 = by["A0 base, 2 s clock"]
    same = len(stored) == N and all(
        abs(a0[s]["miss"] - stored[s]["miss"]) < 1e-9 for s in range(N))
    print(f"A0 reproduces Stage 2's stored cell: {'YES' if same else 'NO'}")
    ok &= same
    ref = by["A2 search, 2 s clock"]
    print(f"{'arm':32s} {'n':>3} {'kills':>5} {'no decode':>9} {'med t_cpa':>9}  "
          f"vs A0 (won/lost)  vs A2 (won/lost)  ends")
    for arm in ARMS:
        rs = by[arm]
        if len(rs) != N:
            print(f"{arm:32s} UNCERTAIN -- {len(rs)} rows, expected {N}")
            ok = False
            continue
        kills = [s for s in rs if rs[s]["miss"] <= 0.35]
        nod = sum(1 for s in rs if rs[s]["first_det"] is None)
        tc = sorted(rs[s]["t_cpa"] for s in kills)
        med = tc[len(tc) // 2] if tc else float("nan")

        def pair(other):
            won = sum(1 for s in rs if rs[s]["miss"] <= 0.35 < other[s]["miss"])
            lost = sum(1 for s in rs if other[s]["miss"] <= 0.35 < rs[s]["miss"])
            return f"{won:>3}/{lost:<3}"
        ends = collections.Counter(end_kind(rs[s]) for s in rs)
        print(f"{arm:32s} {N:3d} {len(kills):5d} {nod:9d} {med:9.1f}  "
              f"{pair(a0):>15s}  {pair(ref):>15s}  {dict(ends)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
