#!/usr/bin/env python3
"""Paired analysis of the Gazebo latency arm (spec.md). Reads the
CINEMA_RESULT lines out of logs/gz_latency/run_L*.log.

Usage: .venv/bin/python runs/gz_latency_20261022/analyze.py
"""
import glob
import os
import re
import statistics as st
import sys

LOGDIR = "logs/gz_latency"
PAT = re.compile(r"cpa_m=([0-9.]+) t_cpa=([0-9.]+) n_det=(\d+) "
                 r"final_state=(\S+) aborted=(\d+)")


def main():
    takes = {}
    for path in sorted(glob.glob(os.path.join(LOGDIR, "run_L*.log"))):
        name = os.path.basename(path)[4:-4]          # L060_c3
        m = None
        with open(path) as f:
            for line in f:
                mm = PAT.search(line)
                if mm:
                    m = mm
        if m is None:
            takes[name] = None
            continue
        takes[name] = {"cpa": float(m.group(1)), "n_det": int(m.group(3)),
                       "state": m.group(4), "aborted": int(m.group(5))}

    arms = {}
    for name, r in sorted(takes.items()):
        arm, cue = name.split("_")
        arms.setdefault(arm, {})[cue] = r
        print(f"{name}: " + ("NO RESULT (boot/param fail?)" if r is None else
              f"cpa={r['cpa']:.3f} n_det={r['n_det']} {r['state']} ab={r['aborted']}"))

    base = arms.get("L000", {})
    print()
    for arm in sorted(a for a in arms if a != "L000"):
        rows = arms[arm]
        cues = [c for c in base if c in rows and base[c] and rows[c]]
        if not cues:
            print(f"{arm}: UNCERTAIN -- no complete pairs (no vacuous verdicts)")
            continue
        deltas = [rows[c]["cpa"] - base[c]["cpa"] for c in cues]
        worse = sum(d > 0.2 for d in deltas)        # run noise ~0.2 m here
        med_b = st.median(base[c]["cpa"] for c in cues)
        med_a = st.median(rows[c]["cpa"] for c in cues)
        print(f"{arm}: n_pairs={len(cues)} med {med_b:.3f} -> {med_a:.3f} m "
              f"({med_a / max(med_b, 1e-9):.1f}x); pairs degraded >0.2 m: "
              f"{worse}/{len(cues)}; deltas {[round(d, 2) for d in deltas]}")
    print("\nverdict rule (spec.md): 120 ms CONFIRMS the cliff iff >=5/6 pairs "
          "degrade and median >=2x; 60 ms expected within noise.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
