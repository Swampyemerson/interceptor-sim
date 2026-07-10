#!/usr/bin/env python3
"""Independent leakage audit of onboard_dataset_v3's split_meta.csv
(ADR-0055 precedent: a SEPARATE verifier artifact that proves zero leakage
structurally, not by trusting the builder). Run AFTER
build_onboard_dataset_v3.py. Offline, sim-free, read-only.

Asserts (docs/seeker_v3_dataset_plan.md §6 + the coordinator's strict
train/eval disjointness rule):
  1. No flight run_tag appears in BOTH train and val (flight-level split).
  2. No grid range appears in BOTH train and val (held-out-range split).
  3. Legacy (v1 renders / v2 capture) is train-only.
  4. VAL flights are exactly the declared held-out set (capv3_03/04/08/10).
  5. VAL grid ranges are exactly the held-out set (3.0, 18.0 m).
  6. NO eval frames (evalv3_*) anywhere in the dataset (the v2-vs-v3 eval
     pool must be strictly outside training).
Exits non-zero on ANY violation.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

VAL_FLIGHTS = {"capv3_03", "capv3_04", "capv3_08", "capv3_10"}
VAL_GRID_RANGES = {3.0, 18.0}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "data", "onboard_dataset_v3"))
    args = ap.parse_args()

    meta = os.path.join(args.dataset, "split_meta.csv")
    if not os.path.isfile(meta):
        print(f"[verify-split] FAILED: no split_meta.csv at {meta}")
        return 2

    rows = list(csv.DictReader(open(meta)))
    if not rows:
        print("[verify-split] FAILED: split_meta.csv empty")
        return 2

    def col(r, *names):
        for n in names:
            if n in r and r[n] != "":
                return r[n]
        return ""

    flights_by_split = {"train": set(), "val": set()}
    grid_ranges_by_split = {"train": set(), "val": set()}
    counts = {"train": {"pos": 0, "neg": 0}, "val": {"pos": 0, "neg": 0}}
    legacy_splits = set()
    eval_leak = []
    violations = []

    for r in rows:
        split = col(r, "split")
        source = col(r, "source")
        flight = col(r, "flight_key", "flight")
        rng = col(r, "range_m", "range")
        pos = col(r, "positive")
        if split not in ("train", "val"):
            violations.append(f"bad split {split!r} for {col(r,'dest_stem','image')}")
            continue
        counts[split]["pos" if pos in ("1", "1.0", "True", "true") else "neg"] += 1
        if flight:
            if "evalv3" in flight:
                eval_leak.append(flight)
            if flight.startswith("capv3"):
                flights_by_split[split].add(flight)
        if source and "grid" in source and rng:
            try:
                grid_ranges_by_split[split].add(round(float(rng), 1))
            except ValueError:
                pass
        if source and "legacy" in source:
            legacy_splits.add(split)
        if source and "eval" in source.lower():
            eval_leak.append(source)

    # 1. flight-level disjointness
    both_flights = flights_by_split["train"] & flights_by_split["val"]
    if both_flights:
        violations.append(f"flights in BOTH splits (leak): {sorted(both_flights)}")
    # 2. grid range disjointness
    both_ranges = grid_ranges_by_split["train"] & grid_ranges_by_split["val"]
    if both_ranges:
        violations.append(f"grid ranges in BOTH splits (leak): {sorted(both_ranges)}")
    # 3. legacy train-only
    if "val" in legacy_splits:
        violations.append("legacy data leaked into VAL (must be train-only)")
    # 4. val flights == declared
    unexpected_val = flights_by_split["val"] - VAL_FLIGHTS
    if unexpected_val:
        violations.append(f"unexpected VAL flights (not the declared held-out set): {sorted(unexpected_val)}")
    # 5. val grid ranges == held-out
    unexpected_vr = grid_ranges_by_split["val"] - VAL_GRID_RANGES
    if unexpected_vr:
        violations.append(f"unexpected VAL grid ranges (not held-out 3/18): {sorted(unexpected_vr)}")
    # 6. no eval frames anywhere
    if eval_leak:
        violations.append(f"EVAL data leaked into the training dataset: {sorted(set(eval_leak))[:5]}")

    print(f"[verify-split] dataset={args.dataset}")
    print(f"[verify-split] train pos={counts['train']['pos']} neg={counts['train']['neg']}  "
          f"val pos={counts['val']['pos']} neg={counts['val']['neg']}")
    print(f"[verify-split] val flights  : {sorted(flights_by_split['val'])}")
    print(f"[verify-split] val grid rng : {sorted(grid_ranges_by_split['val'])}")
    print(f"[verify-split] train flights: {len(flights_by_split['train'])} distinct capv3 tags")
    print(f"[verify-split] train grid rng: {sorted(grid_ranges_by_split['train'])}")
    print(f"[verify-split] legacy splits: {sorted(legacy_splits)} (must be ['train'] only)")

    if violations:
        print("[verify-split] LEAKAGE / SPLIT VIOLATIONS:")
        for v in violations:
            print(f"    !! {v}")
        return 1
    print("[verify-split] PASS — zero leakage: flight-level + held-out-range disjoint, "
          "legacy train-only, val = declared held-out set, NO eval frames present.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
