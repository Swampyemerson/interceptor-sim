#!/usr/bin/env python3
"""Summarize coded-dash (or any mc_batch) result CSVs by direction, with Pk
gates -- the real-build metric that matters (miss inside the lethal radius).

Usage:
    scripts/coded_dash_summary.py CSV [CSV ...]      # one table per CSV
    scripts/coded_dash_summary.py --pk 2.5 A.csv B.csv

Reports, per direction (l2r / r2l): engaged (valid miss), clean flag, Pk@gate,
miss mean / median / min-max. When exactly TWO CSVs are given, also prints the
per-direction delta (B - A) so a paired A/B (e.g. cue-guided vs coded-dash, or
base vs error-sweep) reads at a glance. Numbers only -- every claim traces here.
"""
import argparse
import csv
import statistics


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def summarize(path, gate):
    rows = list(csv.DictReader(open(path)))
    out = {}
    for d in ("l2r", "r2l"):
        sub = [r for r in rows if r.get("direction") == d]
        ms = [num(r.get("miss_m")) for r in sub if num(r.get("miss_m")) is not None]
        clean = sum(1 for r in sub if r.get("clean") == "1")
        out[d] = {
            "n": len(sub),
            "eng": len(ms),
            "clean": clean,
            "pk": sum(1 for m in ms if m < gate),
            "mean": statistics.mean(ms) if ms else None,
            "median": statistics.median(ms) if ms else None,
            "min": min(ms) if ms else None,
            "max": max(ms) if ms else None,
        }
    return out


def fmt(s, gate):
    if not s["eng"]:
        return f"n={s['n']:<3} NO valid miss (all abort/null)"
    return (f"n={s['n']:<3} eng={s['eng']}/{s['n']}  clean={s['clean']}/{s['n']}  "
            f"Pk@{gate}={s['pk']}/{s['eng']}  "
            f"miss mean={s['mean']:.2f} med={s['median']:.2f} "
            f"[{s['min']:.2f}-{s['max']:.2f}]")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+")
    ap.add_argument("--pk", type=float, default=2.5, help="Pk lethal-radius gate m (default 2.5)")
    args = ap.parse_args()

    summaries = []
    for p in args.csvs:
        s = summarize(p, args.pk)
        summaries.append(s)
        print(f"\n=== {p} ===")
        for d in ("l2r", "r2l"):
            print(f"  {d}: {fmt(s[d], args.pk)}")

    if len(summaries) == 2:
        a, b = summaries
        print(f"\n=== DELTA (B - A), Pk@{args.pk} ===")
        for d in ("l2r", "r2l"):
            if a[d]["eng"] and b[d]["eng"]:
                dmean = b[d]["mean"] - a[d]["mean"]
                dpk = b[d]["pk"] - a[d]["pk"]
                print(f"  {d}: miss mean {a[d]['mean']:.2f} -> {b[d]['mean']:.2f} "
                      f"({dmean:+.2f} m)   Pk {a[d]['pk']}/{a[d]['eng']} -> "
                      f"{b[d]['pk']}/{b[d]['eng']} ({dpk:+d})")
            else:
                print(f"  {d}: (missing engagements in one arm)")
    print()


if __name__ == "__main__":
    main()
