#!/usr/bin/env python3
r"""T17-v2 headline comparison: ground_v1 vs ground_v2 on the SAME held-out
multi-range capture. Reads the two `scripts/multirange_nn_detection.py`
output directories (one run with --weights ground_v1.pt, one with
--weights ground_v2.pt, both against the SAME --root, per-run --out-dir so
neither clobbers the other -- see that script's T17-v2 --out-dir addition)
and answers the ADR-0053 question directly: is the -6.3 m @ 50 m
out-of-domain bias GONE, and is the box-size head monotonic now?

PURE OFFLINE -- no sim, no detector inference (both are already run by the
time this executes; this script only reads their CSV/JSON outputs).

FOUR THINGS REPORTED (task-ordered):
  (a) per-standoff bias + sigma, v1 vs v2, side by side.
  (b) headline bias-at-50m before/after (the specific ADR-0053 number).
  (c) box-size (mean_box_diag_px) MONOTONICITY vs range, v1 vs v2 -- ADR-0053
      found v1's box-size regression head non-monotonic out-of-domain (50 m
      boxes SMALLER than 90 m boxes despite being closer/larger in reality).
  (d) REAL-detector sigma_R-vs-R power-law fit for v2, using
      real_centroid_triangulation.csv (the REAL detected+triangulated
      residuals, not perfect-centroid synthetic jitter): one fit across ALL
      captured standoffs (more points, but includes TRAIN-range residuals --
      optimistic, disclosed), and one restricted to ONLY the held-out
      standoffs (thin -- n is small, honestly labeled, but this is the
      genuinely held-out number T20 needs). Range values are binned to the
      nearest 5 m (the X/Z-jitter poses spread true range within a standoff
      bucket) so the fit has more than one x-value per standoff.

Prints a machine-parseable SUMMARY line for scripts/check_t17v2.sh to parse.

RUN:
    .venv-seeker-train/bin/python scripts/eval_t17v2_comparison.py \
        --v1-dir logs/t17v2/nn_detection_v1 --v2-dir logs/t17v2/nn_detection_v2 \
        --held-out-standoffs 70 130 --out logs/t17v2/comparison_summary.json
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from multirange_sigma_fit import power_law_fit  # noqa: E402 -- reuse the log-log OLS, not re-derive


def load_summary(run_dir):
    with open(os.path.join(run_dir, "nn_detection_summary.json")) as fh:
        return json.load(fh)


def load_real_sigma(run_dir):
    with open(os.path.join(run_dir, "real_centroid_sigma_by_standoff.json")) as fh:
        return json.load(fh)


def load_tri_rows(run_dir):
    path = os.path.join(run_dir, "real_centroid_triangulation.csv")
    if not os.path.exists(path):
        return []
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        r["true_R_m"] = float(r["true_R_m"])
        r["triangulated_R_m"] = float(r["triangulated_R_m"])
        r["residual_m"] = float(r["residual_m"])
    return rows


def standoff_tag_to_m(tag):
    return float(tag.replace("standoff_", "").replace("m", ""))


def bin_and_fit(tri_rows, bin_width_m=5.0, label=""):
    """Bins tri_rows by rounded true_R_m, computes pooled residual std per
    bin, power-law-fits sigma_R = c*R^n across bins with >=2 residuals
    (needed for a variance estimate). Returns (fit_dict_or_None, bin_rows)."""
    bins = {}
    for r in tri_rows:
        key = round(r["true_R_m"] / bin_width_m) * bin_width_m
        bins.setdefault(key, []).append(r["residual_m"])
    bin_rows = []
    for R_bin, resids in sorted(bins.items()):
        if len(resids) < 2:
            continue
        sigma = statistics.pstdev(resids)
        bin_rows.append({"R_bin_m": R_bin, "n": len(resids), "sigma_R_m": sigma,
                          "mean_bias_m": statistics.mean(resids)})
    print(f"[t17v2-cmp] {label}: {len(bin_rows)} usable range bins "
          f"(bin_width={bin_width_m} m, >=2 residuals each)")
    for b in bin_rows:
        print(f"    R~{b['R_bin_m']:6.1f} m  n={b['n']:2d}  sigma_R={b['sigma_R_m']:.3f} m  "
              f"bias={b['mean_bias_m']:+.3f} m")
    if len(bin_rows) < 2:
        print(f"[t17v2-cmp] {label}: FEWER THAN 2 usable bins -- power-law fit not computable, "
              f"reporting bin table only.")
        return None, bin_rows
    Rs = [b["R_bin_m"] for b in bin_rows]
    sigs = [max(b["sigma_R_m"], 1e-6) for b in bin_rows]  # power_law_fit needs y>0 for log()
    n_exp, c_fit, r2 = power_law_fit(Rs, sigs)
    fit = {"exponent": n_exp, "c_fit": c_fit, "r_squared": r2, "n_bins": len(bin_rows)}
    print(f"[t17v2-cmp] {label} FIT: sigma_R = {c_fit:.4e} * R^{n_exp:.3f}  (log-log R^2={r2:.4f})")
    return fit, bin_rows


def monotonic_nonincreasing(sorted_by_R_vals, tol=0.0):
    """True if the sequence is non-increasing (allowing `tol` slack per
    step) -- box size should SHRINK (or hold) as range grows."""
    for a, b in zip(sorted_by_R_vals, sorted_by_R_vals[1:]):
        if b > a + tol:
            return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--v1-dir", required=True)
    ap.add_argument("--v2-dir", required=True)
    ap.add_argument("--held-out-standoffs", type=float, nargs="+", default=[70.0, 130.0])
    ap.add_argument("--bin-width-m", type=float, default=5.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    v1_summary = load_summary(args.v1_dir)
    v2_summary = load_summary(args.v2_dir)
    v1_sigma = load_real_sigma(args.v1_dir)
    v2_sigma = load_real_sigma(args.v2_dir)

    tags = sorted(set(v1_summary) | set(v2_summary), key=standoff_tag_to_m)

    print("=" * 100)
    print("(a)+(c) PER-STANDOFF HEAD-TO-HEAD: hit-rate, box-size, real-centroid bias/sigma")
    print("=" * 100)
    hdr = (f"{'standoff':>16} {'R_m':>7} | {'v1 hit':>7} {'v1 box_px':>10} {'v1 bias_m':>10} "
           f"{'v1 sig_m':>9} | {'v2 hit':>7} {'v2 box_px':>10} {'v2 bias_m':>10} {'v2 sig_m':>9}")
    print(hdr)
    rows_out = []
    for tag in tags:
        R = standoff_tag_to_m(tag)
        s1, s2 = v1_summary.get(tag, {}), v2_summary.get(tag, {})
        b1, b2 = v1_sigma.get(tag, {}), v2_sigma.get(tag, {})
        v1_box = s1.get("mean_box_diag_px")
        v2_box = s2.get("mean_box_diag_px")
        v1_bias = b1.get("mean_residual_bias_m")
        v2_bias = b2.get("mean_residual_bias_m")
        v1_sig = b1.get("sigma_R_m")
        v2_sig = b2.get("sigma_R_m")

        def fmt(x, f="{:.2f}"):
            return f.format(x) if isinstance(x, (int, float)) else "n/a"

        print(f"{tag:>16} {R:>7.1f} | {s1.get('hit_rate', float('nan')):>7.0%} "
              f"{fmt(v1_box):>10} {fmt(v1_bias, '{:+.2f}'):>10} {fmt(v1_sig):>9} | "
              f"{s2.get('hit_rate', float('nan')):>7.0%} {fmt(v2_box):>10} "
              f"{fmt(v2_bias, '{:+.2f}'):>10} {fmt(v2_sig):>9}")
        rows_out.append({"standoff_tag": tag, "R_m": R,
                          "v1_hit_rate": s1.get("hit_rate"), "v1_box_diag_px": v1_box,
                          "v1_bias_m": v1_bias, "v1_sigma_m": v1_sig,
                          "v2_hit_rate": s2.get("hit_rate"), "v2_box_diag_px": v2_box,
                          "v2_bias_m": v2_bias, "v2_sigma_m": v2_sig,
                          "held_out": R in args.held_out_standoffs})

    print("\n" + "=" * 100)
    print("(b) HEADLINE: bias @ 50 m, before/after")
    print("=" * 100)
    tag_50 = "standoff_050m"
    v1_bias_50 = v1_sigma.get(tag_50, {}).get("mean_residual_bias_m")
    v2_bias_50 = v2_sigma.get(tag_50, {}).get("mean_residual_bias_m")
    print(f"[t17v2-cmp] ground_v1 bias @ 50 m: {v1_bias_50!r} m  (ADR-0053 measured -6.3 m)")
    print(f"[t17v2-cmp] ground_v2 bias @ 50 m: {v2_bias_50!r} m")

    print("\n" + "=" * 100)
    print("(c) BOX-SIZE MONOTONICITY (should shrink as range grows)")
    print("=" * 100)
    def box_seq(summary):
        pts = [(standoff_tag_to_m(t), summary[t].get("mean_box_diag_px")) for t in tags
               if t in summary and summary[t].get("mean_box_diag_px") is not None]
        pts.sort()
        return pts
    v1_pts = box_seq(v1_summary)
    v2_pts = box_seq(v2_summary)
    v1_mono = monotonic_nonincreasing([p[1] for p in v1_pts])
    v2_mono = monotonic_nonincreasing([p[1] for p in v2_pts])
    print(f"[t17v2-cmp] v1 box-size vs R: {v1_pts}  monotonic_nonincreasing={v1_mono}")
    print(f"[t17v2-cmp] v2 box-size vs R: {v2_pts}  monotonic_nonincreasing={v2_mono}")

    print("\n" + "=" * 100)
    print("(d) REAL-DETECTOR sigma_R vs R power-law fit (ground_v2 only -- the T20 deliverable)")
    print("=" * 100)
    v2_tri = load_tri_rows(args.v2_dir)
    print(f"[t17v2-cmp] v2 real-centroid triangulated pairs: {len(v2_tri)}")
    fit_all, bins_all = bin_and_fit(v2_tri, args.bin_width_m, label="ALL standoffs (incl. train-range, optimistic)")
    held_out_tags = {f"standoff_{int(round(s)):03d}m" for s in args.held_out_standoffs}
    v2_tri_heldout = [r for r in v2_tri if r["standoff"] in held_out_tags]
    print(f"[t17v2-cmp] v2 held-out-only triangulated pairs: {len(v2_tri_heldout)} "
          f"(standoffs {sorted(args.held_out_standoffs)})")
    fit_heldout, bins_heldout = bin_and_fit(v2_tri_heldout, args.bin_width_m,
                                             label="HELD-OUT ONLY (the genuinely generalization-tested number)")

    print("\n" + "=" * 100)
    print("VERDICT")
    print("=" * 100)
    bias_fixed = (isinstance(v2_bias_50, (int, float)) and abs(v2_bias_50) < 2.0)
    print(f"[t17v2-cmp] |bias @ 50 m| < 2.0 m: v2={fmt2(v2_bias_50)} -> "
          f"{'PASS' if bias_fixed else 'FAIL'}")
    print(f"[t17v2-cmp] box-size monotonic (v2): {'PASS' if v2_mono else 'FAIL'} "
          f"(v1 was {'monotonic' if v1_mono else 'NON-monotonic'})")

    summary = {
        "held_out_standoffs": args.held_out_standoffs,
        "per_standoff": rows_out,
        "bias_at_50m": {"v1": v1_bias_50, "v2": v2_bias_50},
        "box_size_monotonic": {"v1": v1_mono, "v2": v2_mono},
        "box_size_points": {"v1": v1_pts, "v2": v2_pts},
        "sigma_R_fit_v2_all_standoffs": fit_all,
        "sigma_R_fit_v2_held_out_only": fit_heldout,
        "sigma_R_bins_v2_all_standoffs": bins_all,
        "sigma_R_bins_v2_held_out_only": bins_heldout,
    }
    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, "w") as fh:
            json.dump(summary, fh, indent=2, default=str)
        print(f"[t17v2-cmp] wrote {args.out}")

    # machine-parseable summary line for check_t17v2.sh
    v1b = v1_bias_50 if isinstance(v1_bias_50, (int, float)) else float("nan")
    v2b = v2_bias_50 if isinstance(v2_bias_50, (int, float)) else float("nan")
    print(f"SUMMARY bias50_v1={v1b:.4f} bias50_v2={v2b:.4f} "
          f"monotonic_v1={v1_mono} monotonic_v2={v2_mono} "
          f"n_heldout_tri_pairs={len(v2_tri_heldout)}")
    return 0


def fmt2(x):
    return f"{x:+.3f}" if isinstance(x, (int, float)) else "n/a"


if __name__ == "__main__":
    raise SystemExit(main())
