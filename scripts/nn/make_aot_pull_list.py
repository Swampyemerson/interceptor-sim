#!/usr/bin/env python3
"""Stage 2 of the AOT slice: stratified-sample the small-target manifest
into a pull list sized for the builder's go-ahead (docs/nn_seeker_program.md).

Strata: pixel-size bins x range bins, sampled evenly so tiny/far frames are
not swamped by the easy close ones; at most --per-flight frames per flight
so no single sortie dominates.

Usage: .venv/bin/python scripts/nn/make_aot_pull_list.py --n 120000
Writes data/aot/pull_list.json + prints the size estimate. Downloads nothing.
"""
import argparse
import json
import os
import random
from collections import defaultdict

OUT_DIR = "data/aot"
SIZE_BINS = ((3, 6), (6, 10), (10, 16), (16, 24), (24, 40))
RANGE_BINS = ((0, 300), (300, 700), (700, 10_000), (None, None))  # last = unknown
BYTES_PER_FRAME = 450_000


def bin_of(v, bins):
    for i, (lo, hi) in enumerate(bins):
        if lo is None:
            if v is None:
                return i
        elif v is not None and lo <= v < hi:
            return i
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=120_000)
    ap.add_argument("--per-flight", type=int, default=120)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    rng = random.Random(args.seed)

    with open(os.path.join(OUT_DIR, "small_target_manifest.json")) as f:
        manifest = json.load(f)

    # one record per unique frame; keep its smallest box + range
    frames = {}
    for m in manifest:
        k = (m["part"], m["flight"], m["img"])
        side = max(m["bb"][2], m["bb"][3])
        if k not in frames or side < frames[k]["side"]:
            frames[k] = {"side": side, "range_m": m.get("range_m")}

    strata = defaultdict(list)
    for k, v in frames.items():
        sb = bin_of(v["side"], SIZE_BINS)
        rb = bin_of(v["range_m"], RANGE_BINS)
        if sb is not None:
            strata[(sb, rb)].append(k)

    quota = max(1, args.n // max(1, len(strata)))
    per_flight = defaultdict(int)
    picked = []
    for key in sorted(strata, key=lambda t: (t[0], -1 if t[1] is None else t[1])):
        pool = strata[key]
        rng.shuffle(pool)
        taken = 0
        for k in pool:
            if taken >= quota:
                break
            if per_flight[k[1]] >= args.per_flight:
                continue
            picked.append(k)
            per_flight[k[1]] += 1
            taken += 1

    out = [{"part": p, "flight": fl, "img": im} for p, fl, im in picked]
    with open(os.path.join(OUT_DIR, "pull_list.json"), "w") as f:
        json.dump(out, f)
    gb = len(out) * BYTES_PER_FRAME / 1e9
    print(f"strata: {len(strata)}; quota/stratum: {quota}; picked {len(out)} "
          f"frames from {len(per_flight)} flights -> est {gb:.1f} GB. "
          f"Pull list: {OUT_DIR}/pull_list.json (bulk pull needs builder go-ahead)")


if __name__ == "__main__":
    main()
