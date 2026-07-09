#!/usr/bin/env python3
r"""ADR-0050 consolidated follow-up -- THE PIVOTAL ANALYSIS: does measured
sigma_R actually scale as R^2 across a real RANGE sweep (50/90/130/160 m),
not just at one fixed ~160 m band (T18's untested caveat, ADR-0049/0050)?

PURE MATH, no Gazebo/PX4 -- runs against the multi-range capture directories
written by scripts/multirange_capture.py (logs/rig_captures/multirange_*/
standoff_{050,090,130,160}m/). Reuses scripts/t18_sigma_validation.py's
PERFECT-CENTROID machinery UNCHANGED (import, not reimplement): for every
captured (standoff, y-offset) pose, project the GT world position into both
TRUE cameras, jitter by the BEST/EXPECTED/WORST sigma_d tiers
(stereo_model.PARAMS["sigma_match_px"]), triangulate, and measure the
resulting range-error std -- exactly t18_sigma_validation.run_capture_sigma_R,
just called once per standoff directory and pooled.

THE FIT: log(measured_sigma_R) vs log(R) linear regression per tier gives
(exponent, c) for measured_sigma_R = c * R^exponent. Compared against:
  - the physics: exponent should be ~2.0 (docs/stereo_design.md's
    sigma_R = R^2/(b*f_px) * sigma_d)
  - the resolution-scaled EXPECTED model c=3.67e-05 m/m^2
    (docs/rig_geometry_analysis.md Sec 1, matching-noise-only, this rig's
    own f_px)
  - the mock's assumed EXPECTED c=4.45e-05 m/m^2 (ADR-0017, full 4-term
    budget, s2_cue_mock.py)

Run:
    .venv/bin/python scripts/multirange_sigma_fit.py --root logs/rig_captures/multirange_run1
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

import stereo_model as sm  # noqa: E402  (reuse constants, not re-derive)
from ground_station.triangulate import RigConfig  # noqa: E402
from t18_sigma_validation import (  # noqa: E402  (reuse the perfect-centroid
    # harness's own functions UNCHANGED -- only the input directory differs)
    load_capture,
    run_capture_sigma_R,
    model_c_matching,
    print_header,
)

OUT_DIR = os.path.join(REPO_ROOT, "logs", "multirange_sigma_fit")
MOCK_C_EXPECTED_M_PER_M2 = 4.45e-05  # ADR-0017 EXPECTED, s2_cue_mock.py SIGMA_RANGE_B_M_PER_M2


def discover_standoff_dirs(root):
    dirs = sorted(glob.glob(os.path.join(root, "standoff_*m")))
    if not dirs:
        raise SystemExit(f"[multirange-fit] no standoff_*m/ directories found under {root}")
    return dirs


def power_law_fit(xs, ys):
    """log(y) = log(c) + n*log(x) -- ordinary least squares on log-log data.
    Returns (n, c, r_squared)."""
    lx = np.log(np.array(xs, dtype=float))
    ly = np.log(np.array(ys, dtype=float))
    n, log_c = np.polyfit(lx, ly, 1)
    c = math.exp(log_c)
    pred = n * lx + log_c
    ss_res = float(np.sum((ly - pred) ** 2))
    ss_tot = float(np.sum((ly - np.mean(ly)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return float(n), c, r2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, help="multirange_capture.py output root "
                     "(contains standoff_*m/ subdirs)")
    ap.add_argument("--n-trials-capture", type=int, default=2000,
                     help="Monte-Carlo trials per captured row (default 2000, higher than "
                          "t18_sigma_validation.py's 500 default since we only have 5 rows/bin)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    standoff_dirs = discover_standoff_dirs(args.root)
    print(f"[multirange-fit] root={args.root}")
    print(f"[multirange-fit] standoff dirs: {[os.path.basename(d) for d in standoff_dirs]}")

    os.makedirs(OUT_DIR, exist_ok=True)
    all_bin_rows = []
    all_per_row = []
    for d in standoff_dirs:
        meta, index_rows = load_capture(d)
        rig = RigConfig.from_capture_meta(os.path.join(d, "capture_meta.json"))
        per_row, bin_rows = run_capture_sigma_R(rig, index_rows, n_trials=args.n_trials_capture,
                                                 seed=args.seed, bin_width_m=1.0)
        for r in bin_rows:
            r["standoff_dir"] = os.path.basename(d)
            r["rig_pose_xyz_yaw"] = rig.true_pose
        for r in per_row:
            r["standoff_dir"] = os.path.basename(d)
        all_bin_rows.extend(bin_rows)
        all_per_row.extend(per_row)
        print(f"[multirange-fit] {os.path.basename(d):>16}: f_px={rig.f_px:.2f} "
              f"true_pose={tuple(round(v, 2) for v in rig.true_pose)} "
              f"n_rows={len(index_rows)}")

    # write raw pooled CSVs
    def _write_csv(path, rows):
        if not rows:
            return
        fieldnames = list(rows[0].keys())
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print(f"[multirange-fit] wrote {path} ({len(rows)} rows)")

    _write_csv(os.path.join(OUT_DIR, "sigma_R_per_row_all_standoffs.csv"), all_per_row)
    _write_csv(os.path.join(OUT_DIR, "sigma_R_bins_all_standoffs.csv"), all_bin_rows)

    # ---- per-tier: pool bins, fit power law ----
    print_header("SIGMA_R vs RANGE -- per-standoff table, all tiers")
    tiers = ("best", "expected", "worst")
    print(f"{'tier':>9} {'R_bin(m)':>9} {'n_rows':>7} {'meas_sigR':>10} {'model_sigR':>11} {'rel_err':>9}")
    for r in sorted(all_bin_rows, key=lambda r: (r["tier"], r["R_bin_center_m"])):
        print(f"{r['tier']:>9} {r['R_bin_center_m']:>9.1f} {r['n_rows_in_bin']:>7} "
              f"{r['measured_sigma_R_m']:>10.5f} {r['model_sigma_R_m']:>11.5f} {r['rel_err']:>+8.2%}")

    print_header("POWER-LAW FIT: measured_sigma_R = c * R^n  (log-log OLS across the 4 standoff bins)")
    fits = {}
    for tier in tiers:
        rows = sorted([r for r in all_bin_rows if r["tier"] == tier], key=lambda r: r["R_actual_mean_m"])
        Rs = [r["R_actual_mean_m"] for r in rows]
        sigs = [r["measured_sigma_R_m"] for r in rows]
        n_exp, c_fit, r2 = power_law_fit(Rs, sigs)
        model_c = model_c_matching(sm.PARAMS["sigma_match_px"][tier],
                                    sm.CHOSEN_BASELINE_M,
                                    RigConfig.from_capture_meta(
                                        os.path.join(standoff_dirs[0], "capture_meta.json")).f_px)
        fits[tier] = {"exponent": n_exp, "c_fit": c_fit, "r_squared": r2,
                      "model_c_matching_only": model_c, "n_points": len(Rs),
                      "R_values": Rs, "sigma_R_values": sigs}
        print(f"[{tier:>9}] fit: sigma_R = {c_fit:.4e} * R^{n_exp:.3f}   (R^2 of log-log fit = {r2:.4f})")
        print(f"            exponent vs physics-expected 2.0: {n_exp:.3f} "
              f"({'CONSISTENT' if abs(n_exp - 2.0) <= 0.3 else 'DIVERGES'} within +/-0.3)")
        print(f"            c_fit vs resolution-scaled matching-only model "
              f"({model_c:.3e}): ratio = {c_fit / model_c:.2f}x")
        if tier == "expected":
            print(f"            c_fit vs mock's assumed EXPECTED ({MOCK_C_EXPECTED_M_PER_M2:.2e}, "
                  f"full 4-term budget): ratio = {c_fit / MOCK_C_EXPECTED_M_PER_M2:.2f}x")

    print_header("SUMMARY")
    exp_expected = fits["expected"]["exponent"]
    verdict = ("HOLDS" if abs(exp_expected - 2.0) <= 0.3 else
               "DOES NOT CLEANLY HOLD (diverges from R^2 by more than +/-0.3 in the fitted exponent)")
    print(f"EXPECTED-tier fitted exponent = {exp_expected:.3f} (physics predicts 2.0) -> {verdict}")
    print(f"EXPECTED-tier fitted c = {fits['expected']['c_fit']:.4e} m/m^2 "
          f"vs resolution-scaled model {fits['expected']['model_c_matching_only']:.4e} "
          f"(ratio {fits['expected']['c_fit']/fits['expected']['model_c_matching_only']:.2f}x) "
          f"vs mock's {MOCK_C_EXPECTED_M_PER_M2:.4e} "
          f"(ratio {fits['expected']['c_fit']/MOCK_C_EXPECTED_M_PER_M2:.2f}x)")

    summary_path = os.path.join(OUT_DIR, "sigma_R_power_law_fit.json")
    with open(summary_path, "w") as fh:
        json.dump({
            "root": args.root, "standoff_dirs": [os.path.basename(d) for d in standoff_dirs],
            "n_trials_capture": args.n_trials_capture, "seed": args.seed,
            "fits": fits, "mock_c_expected_m_per_m2": MOCK_C_EXPECTED_M_PER_M2,
        }, fh, indent=2, default=str)
    print(f"\n[multirange-fit] wrote {summary_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
