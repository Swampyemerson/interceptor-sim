#!/usr/bin/env python3
"""Final-second (CPA-1s .. CPA) endgame forensics behind ADR-0042's mechanism.

Reproduces the two tables the ADR cites: per-seed detection content
(ticks / detected / polluted false-locks) and guidance dynamics
(median |lambda_dot|, |a_cmd|, vc) across arms, in the last second before
closest approach. gt_range is SCORING-ONLY here (offline analysis).

Run:  .venv/bin/python scripts/seeker/endgame_forensics.py
      [--seeds 229259,631263,26226,776647]
"""
from __future__ import annotations

import argparse
import csv
import statistics as st

ARMS = {
    "tag": "logs/mc_ab_apriltag.csv",
    "v1": "logs/mc_ab_markerless_tuned.csv",
    "v2": "logs/mc_ab_markerless_tuned_v2.csv",
}
DEFAULT_SEEDS = "229259,631263,26226,776647"  # the previously-tight seeds


def flight_paths(arm_csv):
    return {r["cue_seed"]: r["flight_csv_path"] for r in csv.DictReader(open(arm_csv))}


def window(rows):
    gt = [(i, float(r["gt_range"])) for i, r in enumerate(rows) if r.get("gt_range")]
    cpa_i = min(gt, key=lambda x: x[1])[0]
    t_cpa = float(rows[cpa_i]["t_sim"])
    return [r for r in rows if r.get("t_sim")
            and t_cpa - 1.0 <= float(r["t_sim"]) <= t_cpa]


def content(win):
    det = [r for r in win if r["detected"] == "1"]
    poll = [r for r in det if r.get("gt_range") and float(r["gt_range"]) > 0.5
            and float(r["meas_range"]) / float(r["gt_range"]) < 0.2]
    return len(win), len(det), len(poll)


def dynamics(win):
    def med(col, absolute=True):
        vals = [float(r[col]) for r in win if r.get(col)]
        if absolute:
            vals = [abs(v) for v in vals]
        return st.median(vals) if vals else 0.0
    return med("lambda_dot_deg_s"), med("a_cmd_m_s2"), med("vc_m_s", absolute=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", default=DEFAULT_SEEDS)
    args = ap.parse_args()
    seeds = [s.strip() for s in args.seeds.split(",") if s.strip()]

    wins = {}
    for arm, arm_csv in ARMS.items():
        paths = flight_paths(arm_csv)
        for s in seeds:
            wins[(arm, s)] = window(list(csv.DictReader(open(paths[s]))))

    print("Table 1 — final-second detection content: ticks / detected / polluted(false-lock)")
    print(f"{'seed':>7s} | " + " | ".join(f"{a:>15s}" for a in ARMS))
    for s in seeds:
        cells = [f"{'/'.join(str(x) for x in content(wins[(a, s)])):>15s}" for a in ARMS]
        print(f"{s:>7s} | " + " | ".join(cells))

    print("\nTable 2 — final-second dynamics: median |lambda_dot| deg/s ; |a_cmd| m/s2 ; vc m/s")
    print(f"{'seed':>7s} | " + " | ".join(f"{a:>22s}" for a in ARMS))
    for s in seeds:
        cells = []
        for a in ARMS:
            ld, ac, vc = dynamics(wins[(a, s)])
            cells.append(f"{ld:6.1f} ; {ac:6.1f} ; {vc:4.1f}")
        print(f"{s:>7s} | " + " | ".join(f"{c:>22s}" for c in cells))

    print("\nReading (ADR-0042): tag = near-ballistic endgame (a_cmd~0, vc rides the "
          "4.5 floor); v1 = saturated thrash off polluted estimates (accidentally "
          "straight -> tight); v2 = moderate ACHIEVABLE commands off honest but "
          "noisier bearings -> follows the noise (~+1 m dispersion).")


if __name__ == "__main__":
    main()
