#!/usr/bin/env python3
"""Measure the markerless detector's REAL-TARGET recall vs RANGE from coded-dash
flight logs — the make-or-break number (ADR-0076 add #18h: the wall is recall on
the small, fast, APPROACHING target, ~0 in sim beyond ~8 m). This quantifies that
wall so the real-data detector (R6) has a baseline to beat, and lets us read any
sim lever (lower conf, foveated crop, camera-forward) by whether it shifts the
recall curve to LONGER range.

A detection is the REAL target (not the own-prop phantom) iff its implied range
agrees with gt_range (|meas_range - gt_range| < 50% of gt_range) — gt is
scoring-legal (labeling only, never guidance). For each gt-range bin we report:
  real-detection RATE  = ticks with a real detection / all ticks in that bin
  phantom RATE         = ticks with a phantom detection / all ticks in that bin
The real rate falling off with range IS the acquisition wall.

Usage:
    scripts/seeker/approach_recall.py logs/mc_coded_dash_qv2_line9_s123.csv [more.csv ...]
    scripts/seeker/approach_recall.py --bin 2 --max 30 A.csv B.csv   # compare two arms
"""
import argparse
import csv
import sys


def _is_real(mr, gr):
    return gr > 0 and abs(mr - gr) / gr < 0.5


def recall_curve(mc_csv, bin_m, max_m):
    """Aggregate real/phantom detection rate per gt-range bin across all flights
    in an mc_batch CSV, counting ONLY the APPROACHING (pre-CPA, closing) portion
    of each flight. Returns {bin_lo: (real, phantom, total)}.

    Why closing-only (Fable round-2 check): the detector sees a RECEDING target
    (post-CPA, target flying away) far better than an approaching one, so counting
    post-CPA ticks inflates "approach" recall and makes the metric a mirage
    generator (any lever touching post-CPA geometry moves it). We find each
    flight's CPA (min gt_range tick) and count only ticks up to it."""
    bins = {}
    for r in csv.DictReader(open(mc_csv)):
        fp = r.get("flight_csv_path", "")
        try:
            frows = list(csv.DictReader(open(fp)))
        except (FileNotFoundError, OSError):
            continue
        # CPA index = the tick of minimum gt_range; keep only ticks up to it.
        grs = []
        for x in frows:
            try:
                grs.append(float(x["gt_range"]))
            except (KeyError, TypeError, ValueError):
                grs.append(float("inf"))
        if not grs or min(grs) == float("inf"):
            continue
        cpa_i = grs.index(min(grs))
        for i, x in enumerate(frows):
            if i > cpa_i:
                break  # receding after CPA -- excluded
            gr = grs[i]
            if gr <= 0 or gr == float("inf") or gr > max_m:
                continue
            b = int(gr // bin_m) * bin_m
            real = phantom = 0
            if x.get("detected") in ("1", "True", "true"):
                try:
                    mr = float(x["meas_range"])
                    if _is_real(mr, gr):
                        real = 1
                    else:
                        phantom = 1
                except (KeyError, TypeError, ValueError):
                    pass
            t = bins.get(b, [0, 0, 0])
            t[0] += real
            t[1] += phantom
            t[2] += 1
            bins[b] = t
    return bins


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+", help="mc_batch result CSV(s)")
    ap.add_argument("--bin", type=float, default=2.0, help="range bin size m (default 2)")
    ap.add_argument("--max", type=float, default=30.0, help="max range m (default 30)")
    args = ap.parse_args()
    for path in args.csvs:
        bins = recall_curve(path, args.bin, args.max)
        print(f"\n=== {path} — real-target detection rate vs range ===")
        print(f"  {'range(m)':<10}{'real%':>8}{'phantom%':>10}{'ticks':>8}")
        for b in sorted(bins):
            real, phantom, tot = bins[b]
            print(f"  {f'{b:.0f}-{b+args.bin:.0f}':<10}{100*real/tot:>7.0f}%"
                  f"{100*phantom/tot:>9.0f}%{tot:>8}")
        # the acquisition-relevant summary: real recall in the >8 m approach band
        far = [v for b, v in bins.items() if b >= 8]
        if far:
            fr = sum(v[0] for v in far); ft = sum(v[2] for v in far)
            print(f"  APPROACH (>=8 m): real detection rate {100*fr/max(1,ft):.1f}% "
                  f"({fr}/{ft} ticks)  <- the wall")


if __name__ == "__main__":
    sys.exit(main())
