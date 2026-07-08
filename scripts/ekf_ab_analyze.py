#!/usr/bin/env python3
"""Paired A/B analysis for the EKF-vs-alpha-beta tracker comparison.

Post-M5 queue item (docs/ekf_design_brief.md section 4). This script is the
OFFLINE analyzer for the pre-registered EKF A/B: it takes the two mc_batch arm
CSVs that differ ONLY by --tracker (alpha-beta baseline vs EKF candidate),
which are PER-SEED PAIRED by run_idx (identical cue_seed per row because both
arms share --master-seed 42), and prints the pre-registered PRIMARY / SECONDARY
metrics and a SUCCESS / PARTIAL / NULL verdict.

It NEVER boots a sim or flies anything -- it only reads existing CSVs.

WHAT IT COMPUTES (all criteria are pre-registered in the brief, section 4.4):

  PRIMARY (expect NULL -- the miss is kinematic, ADR-0023 r^2(ZEM,miss)=0.990):
    * paired per-seed miss delta (ekf - alphabeta): mean, sd, 95% CI
      (mean +/- 1.96*sd/sqrt(n)); "tied" if the CI straddles 0 OR |mean| is
      below the ~1 m run-to-run terminal-dropout noise floor (project rule,
      CLAUDE.md "Statistics before verdicts").
    * handoff-reach rate and clean rate per arm.

  SECONDARY (where an EKF edge would show, brief section 4.3):
    * TRACK RMSE vs ground truth (gt_* used for SCORING/ANALYSIS ONLY, honesty
      boundary ADR-0008):
        - range track RMSE   = RMS(r_hat_m - gt_range) over detected ticks
        - position track RMSE = RMS( ||(tgt_n_hat,tgt_e_hat) - (gt_tag_y,gt_tag_x)|| )
          over pre-CPA ticks with an estimate present (world->NED: north=world_y,
          east=world_x -- ADR-0013).
    * terminal COVERAGE = fraction of terminal-range ticks (gt_range <= terminal
      range) carrying a fresh detection.
    * LOS-rate error proxy = RMS( lambda_dot_deg_s - true LOS rate ), where the
      true inertial LOS azimuth is reconstructed from the ground-truth geometry
      lambda_true = atan2(east_rel, north_rel), rel = gt_tag - gt_cam, and
      differenced in sim time. THIN + singularity-adjacent -> reported as a
      low-confidence proxy only.
    Each is reported per arm and as a paired per-seed delta with a 95% CI.

  VERDICT (pre-registered, brief section 4.4):
    SUCCESS  -> miss delta 95% CI excludes 0 AND favors EKF, no regression.
    PARTIAL  -> track RMSE and/or coverage improve significantly but miss tied
                (the MOST LIKELY outcome -- ADR-0023 kinematic floor holds).
    NULL     -> both tied (also REJECT if EKF is significantly worse anywhere).
    Honest "not significant at n=8" language is printed wherever a CI straddles 0.

HONESTY: gt_cam / gt_tag / gt_range are ground truth, used here for SCORING /
ANALYSIS ONLY. The trackers never read them in flight.

USAGE:
    python3 scripts/ekf_ab_analyze.py [alphabeta_arm.csv] [ekf_arm.csv]
    (defaults: logs/mc_ekfab_alphabeta.csv  logs/mc_ekfab_ekf.csv)

    # dev / plumbing check against a prior paired pair (NOT a tracker A/B):
    python3 scripts/ekf_ab_analyze.py logs/mc_final_line6.csv logs/mc_final_line9.csv

Handles a missing EKF arm and missing columns gracefully (prints what is
available). Standard library only -- no numpy/scipy.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys

# --- Constants pinned to the project's paired-test convention -----------------
# (mirrors mc_analyze.py:115/125/126 so the two analyzers agree.)
Z_95 = 1.959963984540054   # two-sided 95% normal quantile (brief uses 1.96)
NOISE_FLOOR_M = 1.0        # run-to-run terminal-dropout noise (CLAUDE.md/ADR-0013)
PAIRED_MIN_N = 8           # ADR-0033 item 3: paired seeds, n >= 8 per cell

# Deployment/FPV terminal geometry these arms fly (mc_ekfab_* = pronav /
# standard / deployment profile). TERMINAL_RANGE_M 5.0 and TERMINAL_FREEZE
# 3.5 are the FPV-dict values in m4_intercept.py:327-328; the breakoff_reason
# text in the arm CSVs confirms "terminal range (5.0 m)".
DEFAULT_TERMINAL_RANGE_M = 5.0
DEFAULT_FREEZE_RANGE_M = 3.5


# --- small numeric helpers ----------------------------------------------------

def fnum(s):
    """Parse a CSV cell to a finite float, or None (blank / non-finite)."""
    if s is None:
        return None
    s = s.strip()
    if s == "":
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def mean_sd(values):
    if not values:
        return None, None
    m = statistics.mean(values)
    sd = statistics.stdev(values) if len(values) > 1 else float("nan")
    return m, sd


# --- arm CSV parsing ----------------------------------------------------------

def load_arm(path, law):
    """Read one mc_batch arm CSV -> list of per-run dicts (filtered to `law`).

    Returns (rows, warnings). Each row dict carries run_idx, cue_seed, miss,
    clean, handoff, coverage_csv, flight_csv_path (+ raw). The A/B arms are
    pronav-only; filtering to `law` also makes run_idx unique so we can pair
    on it. Rows with a non-finite miss are kept but flagged (failed flights).
    """
    rows = []
    warns = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames or []
        for raw in reader:
            if law and (raw.get("law", "").strip() != law):
                continue
            miss = fnum(raw.get("miss_m"))
            rows.append({
                "run_idx": (raw.get("run_idx") or "").strip(),
                "cue_seed": (raw.get("cue_seed") or "").strip(),
                "miss": miss,
                "clean": fnum(raw.get("clean")),
                "handoff": fnum(raw.get("handoff")),
                "coverage_csv": fnum(raw.get("coverage")),
                "py_exit_code": (raw.get("py_exit_code") or "").strip(),
                "flight_csv_path": (raw.get("flight_csv_path") or "").strip(),
                "raw": raw,
            })
    if "miss_m" not in cols:
        warns.append(f"{os.path.basename(path)}: no 'miss_m' column")
    return rows, warns


# --- per-flight (per-tick) metrics -------------------------------------------

# Exact per-tick columns consumed (m4_intercept.py CSV schema, m3_static_intercept.write_row):
#   t_sim               -- sim clock (never wall time; CLAUDE.md rule)
#   detected            -- 1 on a fresh camera detection this tick
#   r_hat_m             -- tracker RANGE estimate (alpha-beta or EKF read-out)
#   lambda_dot_deg_s    -- tracker LOS-rate estimate (lambda_dot_hat), pro-nav's input
#   tgt_n_hat,tgt_e_hat -- tracker TARGET position estimate, PX4 local NED
#   gt_range            -- GT range ||gt_tag - gt_cam||          (scoring only)
#   gt_cam_x,gt_cam_y   -- GT interceptor camera world pos (ENU) (scoring only)
#   gt_tag_x,gt_tag_y   -- GT target world pos (ENU)             (scoring only)
# world->NED: north = world_y, east = world_x (ADR-0013), so GT target NED =
# (gt_tag_y, gt_tag_x) and inertial LOS azimuth = atan2(east_rel, north_rel).

REQUIRED_TICK_COLS = ["t_sim", "detected", "gt_range"]


def flight_metrics(flight_csv, terminal_range, freeze_range, los_upper):
    """Compute the per-flight secondary metrics from one m4_intercept_*.csv.

    Returns (metrics_dict, note). metrics_dict values are None where a column
    is missing or no valid tick exists. `note` records missing columns.
    """
    if not flight_csv or not os.path.exists(flight_csv):
        return None, "flight CSV missing"
    try:
        with open(flight_csv, newline="") as f:
            reader = csv.DictReader(f)
            cols = set(reader.fieldnames or [])
            rows = list(reader)
    except OSError as e:
        return None, f"read error: {e}"

    missing = [c for c in REQUIRED_TICK_COLS if c not in cols]
    if missing:
        return None, "missing cols: " + ",".join(missing)

    have_rhat = "r_hat_m" in cols
    have_pos = "tgt_n_hat" in cols and "tgt_e_hat" in cols and \
               "gt_tag_x" in cols and "gt_tag_y" in cols
    have_los = "lambda_dot_deg_s" in cols and \
               all(c in cols for c in ("gt_cam_x", "gt_cam_y", "gt_tag_x", "gt_tag_y"))

    # Closest-point-of-approach tick (minimum gt_range) -> the intercept moment.
    # Pre-CPA is the meaningful tracking window; post-CPA the target has flown
    # past and the coasting range estimate diverges (r_hat goes negative), which
    # is geometry, not tracker error -- excluded.
    t_cpa = None
    min_gt = None
    for r in rows:
        g = fnum(r.get("gt_range"))
        ts = fnum(r.get("t_sim"))
        if g is None or ts is None:
            continue
        if min_gt is None or g < min_gt:
            min_gt, t_cpa = g, ts

    m = {
        "min_gt": min_gt, "t_cpa": t_cpa,
        "range_rmse_det": None, "n_range_det": 0,
        "range_rmse_precpa": None, "n_range_precpa": 0,
        "pos_rmse": None, "n_pos": 0,
        "term_cov": None, "n_term": 0,
        "los_rmse": None, "n_los": 0,
    }

    # ---- range track RMSE: detected ticks (measurement-anchored) & pre-CPA coast
    if have_rhat and t_cpa is not None:
        det, precpa = [], []
        for r in rows:
            ts = fnum(r.get("t_sim"))
            rh = fnum(r.get("r_hat_m"))
            g = fnum(r.get("gt_range"))
            if ts is None or rh is None or g is None or ts > t_cpa:
                continue
            if (r.get("detected") or "").strip() == "1":
                det.append(rh - g)
            if rh > 0:                       # drop coast that has crossed through zero
                precpa.append(rh - g)
        m["range_rmse_det"], m["n_range_det"] = rms(det), len(det)
        m["range_rmse_precpa"], m["n_range_precpa"] = rms(precpa), len(precpa)

    # ---- position track RMSE: mid-course target track vs GT target NED, pre-CPA
    if have_pos and t_cpa is not None:
        errs = []
        for r in rows:
            ts = fnum(r.get("t_sim"))
            nh = fnum(r.get("tgt_n_hat"))
            eh = fnum(r.get("tgt_e_hat"))
            ty = fnum(r.get("gt_tag_y"))     # GT north
            tx = fnum(r.get("gt_tag_x"))     # GT east
            if None in (ts, nh, eh, ty, tx) or ts > t_cpa:
                continue
            errs.append(math.hypot(nh - ty, eh - tx))
        m["pos_rmse"], m["n_pos"] = rms(errs), len(errs)

    # ---- terminal coverage: fraction of terminal-range ticks with a detection
    term = [r for r in rows
            if (fnum(r.get("gt_range")) is not None
                and fnum(r.get("gt_range")) <= terminal_range)]
    if term:
        det_n = sum(1 for r in term if (r.get("detected") or "").strip() == "1")
        m["term_cov"], m["n_term"] = det_n / len(term), len(term)

    # ---- LOS-rate error proxy: lambda_dot_hat vs finite-diff true LOS rate.
    # THIN and singularity-adjacent (lambda_dot only logged in the terminal;
    # true LOS rate blows up as gt_range->0). Gated to detected ticks with
    # freeze_range < gt_range <= los_upper. Low confidence by construction.
    if have_los:
        seq = []
        for r in rows:
            ts = fnum(r.get("t_sim"))
            cx, cy = fnum(r.get("gt_cam_x")), fnum(r.get("gt_cam_y"))
            tx, ty = fnum(r.get("gt_tag_x")), fnum(r.get("gt_tag_y"))
            if None in (ts, cx, cy, tx, ty):
                continue
            dn, de = ty - cy, tx - cx        # rel in NED (north, east)
            lam = math.degrees(math.atan2(de, dn))
            seq.append((ts, lam, r))
        errs = []
        for k in range(1, len(seq)):
            t0, l0, _ = seq[k - 1]
            t1, l1, r1 = seq[k]
            dt = t1 - t0
            if dt <= 0:
                continue
            dl = l1 - l0
            while dl > 180:
                dl -= 360
            while dl < -180:
                dl += 360
            ld_true = dl / dt
            ld_hat = fnum(r1.get("lambda_dot_deg_s"))
            g = fnum(r1.get("gt_range"))
            if ld_hat is None or g is None:
                continue
            if (r1.get("detected") or "").strip() != "1":
                continue
            if not (freeze_range < g <= los_upper):
                continue
            errs.append(ld_hat - ld_true)
        m["los_rmse"], m["n_los"] = rms(errs), len(errs)

    note_bits = []
    if not have_rhat:
        note_bits.append("no r_hat_m")
    if not have_pos:
        note_bits.append("no tgt_/gt_tag pos cols")
    if not have_los:
        note_bits.append("no LOS cols")
    return m, ("; ".join(note_bits) if note_bits else "")


# --- paired stats -------------------------------------------------------------

def paired_delta(a_by_key, b_by_key, higher_is_better=False,
                 noise_floor=0.0, min_n=PAIRED_MIN_N):
    """Paired delta (candidate B - baseline A) over shared keys with both
    values finite. Returns a dict with n, mean, sd, ci, and a verdict flag.

    `favors` in {"ekf","alphabeta","tied"}. Significant requires n>=min_n AND
    the 95% CI to clear 0 entirely AND |mean| > noise_floor (noise_floor is the
    ~1 m miss floor; 0 for unitless/coverage metrics). Lower-is-better metrics
    (miss, RMSE, LOS error): a NEGATIVE mean favors EKF. Higher-is-better
    (coverage, handoff-reach): a POSITIVE mean favors EKF.
    """
    diffs = []
    for k in a_by_key:
        a, b = a_by_key[k], b_by_key.get(k)
        if a is None or b is None:
            continue
        diffs.append(b - a)
    n = len(diffs)
    if n == 0:
        return {"n": 0, "mean": None, "sd": None, "ci": (None, None),
                "favors": "n/a", "signif": False}
    mean = statistics.mean(diffs)
    sd = statistics.stdev(diffs) if n > 1 else float("nan")
    margin = Z_95 * sd / math.sqrt(n) if (n > 1 and sd > 0) else 0.0
    ci = (mean - margin, mean + margin)
    ci_clears_0 = (ci[0] > 0) or (ci[1] < 0)
    beats_floor = abs(mean) > noise_floor
    signif = (n >= min_n) and ci_clears_0 and beats_floor
    if not signif:
        favors = "tied"
    else:
        improved = (mean > 0) if higher_is_better else (mean < 0)
        favors = "ekf" if improved else "alphabeta"
    return {"n": n, "mean": mean, "sd": sd, "ci": ci,
            "favors": favors, "signif": signif,
            "ci_clears_0": ci_clears_0, "beats_floor": beats_floor}


def fmt_delta(d, unit="m", higher_is_better=False):
    if d["n"] == 0:
        return "no paired data"
    lo, hi = d["ci"]
    sd = d["sd"]
    sd_s = f"{sd:.3f}" if (sd is not None and math.isfinite(sd)) else "n/a"
    base = (f"delta(ekf-ab)={d['mean']:+.3f} {unit}  sd={sd_s}  "
            f"95%CI=[{lo:+.3f}, {hi:+.3f}]  n={d['n']}")
    if d["favors"] == "ekf":
        tag = "FAVORS EKF"
    elif d["favors"] == "alphabeta":
        tag = "FAVORS ALPHA-BETA (regression)"
    else:
        if d["n"] < PAIRED_MIN_N:
            tag = f"not significant (n={d['n']}<{PAIRED_MIN_N})"
        else:
            tag = "tied (CI straddles 0)"
    return base + "  -> " + tag


# --- per-arm aggregation & printing ------------------------------------------

def arm_metric_maps(arm_rows, terminal_range, freeze_range, los_upper):
    """For one arm, compute the per-flight secondary metrics keyed by run_idx.
    Returns (maps, flight_notes, n_flight_csv_ok)."""
    keys = ["range_rmse_det", "range_rmse_precpa", "pos_rmse", "term_cov", "los_rmse"]
    maps = {k: {} for k in keys}
    tick_counts = {k: 0 for k in ["n_range_det", "n_pos", "n_term", "n_los"]}
    notes = {}
    ok = 0
    for row in arm_rows:
        m, note = flight_metrics(row["flight_csv_path"], terminal_range,
                                 freeze_range, los_upper)
        if m is None:
            notes[row["run_idx"]] = note
            continue
        ok += 1
        rk = row["run_idx"]
        for k in keys:
            if m.get(k) is not None:
                maps[k][rk] = m[k]
        for tk in tick_counts:
            tick_counts[tk] += m.get(tk, 0) or 0
        if note:
            notes[rk] = note
    return maps, tick_counts, notes, ok


def summarize_map(mp):
    vals = list(mp.values())
    if not vals:
        return "n=0 (no valid flights)"
    m, sd = mean_sd(vals)
    sd_s = f"{sd:.3f}" if (sd is not None and math.isfinite(sd)) else "n/a"
    return f"mean={m:.3f}  sd={sd_s}  n_flights={len(vals)}"


def rate(rows, field):
    vals = [r[field] for r in rows if r[field] is not None]
    if not vals:
        return None, 0
    return sum(1 for v in vals if v >= 0.5) / len(vals), len(vals)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("alphabeta_csv", nargs="?",
                   default="logs/mc_ekfab_alphabeta.csv",
                   help="baseline (alpha-beta) arm CSV")
    p.add_argument("ekf_csv", nargs="?",
                   default="logs/mc_ekfab_ekf.csv",
                   help="candidate (EKF) arm CSV")
    p.add_argument("--law", default="pronav",
                   help="guidance law to filter each arm to (default pronav; "
                        "the A/B arms are pronav-only)")
    p.add_argument("--terminal-range", type=float, default=DEFAULT_TERMINAL_RANGE_M)
    p.add_argument("--freeze-range", type=float, default=DEFAULT_FREEZE_RANGE_M)
    p.add_argument("--los-upper", type=float, default=8.0,
                   help="upper gt_range gate for the LOS-rate proxy (m)")
    args = p.parse_args(argv)

    # Resolve paths relative to the repo root (so it runs from anywhere).
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    def resolve(pth):
        return pth if os.path.isabs(pth) else os.path.join(root, pth)

    ab_path = resolve(args.alphabeta_csv)
    ekf_path = resolve(args.ekf_csv)

    print("=" * 78)
    print("EKF vs ALPHA-BETA -- paired A/B analysis (docs/ekf_design_brief.md sec.4)")
    print("=" * 78)
    print(f"baseline  arm (alpha-beta): {ab_path}")
    print(f"candidate arm (EKF)       : {ekf_path}")
    print(f"law filter={args.law}   terminal_range={args.terminal_range} m   "
          f"freeze_range={args.freeze_range} m")
    print("PROVENANCE / honesty: gt_cam=GT interceptor cam world pos, "
          "gt_tag=GT target world pos, gt_range=||gt_tag-gt_cam||.")
    print("  gt_* is SCORING/ANALYSIS ONLY (ADR-0008). Track RMSE uses r_hat_m, "
          "tgt_n_hat/tgt_e_hat vs gt_*;")
    print("  LOS-rate proxy uses lambda_dot_deg_s vs atan2(east_rel,north_rel) "
          "(world->NED north=world_y,east=world_x).")
    print("  Sim-time (t_sim) throughout; pre-CPA window = ticks up to min(gt_range).")
    print()

    if not os.path.exists(ab_path):
        print(f"FATAL: baseline arm not found: {ab_path}")
        print("Nothing to analyze. Fly the alpha-beta arm first.")
        return 2

    ab_rows, ab_warn = load_arm(ab_path, args.law)
    for w in ab_warn:
        print("  [warn]", w)
    print(f"[baseline] {os.path.basename(ab_path)}: "
          f"{len(ab_rows)} '{args.law}' rows, run_idx="
          f"{[r['run_idx'] for r in ab_rows]}")

    ekf_present = os.path.exists(ekf_path)
    ekf_rows = []
    if ekf_present:
        ekf_rows, ekf_warn = load_arm(ekf_path, args.law)
        for w in ekf_warn:
            print("  [warn]", w)
        print(f"[candidate] {os.path.basename(ekf_path)}: "
              f"{len(ekf_rows)} '{args.law}' rows, run_idx="
              f"{[r['run_idx'] for r in ekf_rows]}")
    else:
        print(f"[candidate] {os.path.basename(ekf_path)}: NOT FOUND "
              "-- EKF arm not flown yet.")
    print()

    # ---- per-flight secondary metrics for each arm ----
    ab_maps, ab_ticks, ab_notes, ab_ok = arm_metric_maps(
        ab_rows, args.terminal_range, args.freeze_range, args.los_upper)
    ekf_maps, ekf_ticks, ekf_notes, ekf_ok = ({}, {}, {}, 0)
    if ekf_present:
        ekf_maps, ekf_ticks, ekf_notes, ekf_ok = arm_metric_maps(
            ekf_rows, args.terminal_range, args.freeze_range, args.los_upper)

    # ---- 1. PRIMARY -----------------------------------------------------------
    print("-" * 78)
    print("1. PRIMARY -- end-to-end miss (expect NULL; ADR-0023 miss is kinematic)")
    print("-" * 78)

    def rate_line(label, rows):
        r_ho, n_ho = rate(rows, "handoff")
        r_cl, n_cl = rate(rows, "clean")
        miss_vals = [r["miss"] for r in rows if r["miss"] is not None]
        mm, msd = mean_sd(miss_vals)
        mm_s = f"{mm:.3f}" if mm is not None else "n/a"
        msd_s = f"{msd:.3f}" if (msd is not None and math.isfinite(msd)) else "n/a"
        ho_s = f"{r_ho*100:.0f}% ({n_ho})" if r_ho is not None else "n/a"
        cl_s = f"{r_cl*100:.0f}% ({n_cl})" if r_cl is not None else "n/a"
        print(f"  {label:11s}  miss mean={mm_s} m sd={msd_s} (n={len(miss_vals)})   "
              f"handoff-reach={ho_s}   clean={cl_s}")

    rate_line("alpha-beta", ab_rows)
    if ekf_present:
        rate_line("ekf", ekf_rows)

    miss_delta = None
    if ekf_present:
        ab_miss = {r["run_idx"]: r["miss"] for r in ab_rows}
        ekf_miss = {r["run_idx"]: r["miss"] for r in ekf_rows}
        # sanity: paired rows must share cue_seed
        ab_seed = {r["run_idx"]: r["cue_seed"] for r in ab_rows}
        ekf_seed = {r["run_idx"]: r["cue_seed"] for r in ekf_rows}
        mism = [k for k in ab_seed if k in ekf_seed and ab_seed[k] != ekf_seed[k]]
        if mism:
            print(f"  [WARN] cue_seed mismatch on run_idx {mism} -- pairing may be "
                  "invalid (arms should share --master-seed).")
        miss_delta = paired_delta(ab_miss, ekf_miss, higher_is_better=False,
                                   noise_floor=NOISE_FLOOR_M)
        print("  paired miss  ", fmt_delta(miss_delta, "m"))
        if miss_delta["favors"] == "tied" and miss_delta["n"] >= PAIRED_MIN_N:
            print("               (tied is the PRE-REGISTERED expectation -- "
                  "the miss is kinematic, not estimator-limited.)")
    else:
        print("  paired miss   -- pending EKF arm --")
    print()

    # ---- 2. SECONDARY ---------------------------------------------------------
    print("-" * 78)
    print("2. SECONDARY -- track quality (where an EKF edge would show)")
    print("-" * 78)
    print(f"  per-arm central values (mean +/- sd over flights); pooled ticks in [].")
    print()

    sec = [
        ("range track RMSE (detected)",   "range_rmse_det",   "m",  False, "n_range_det"),
        ("range track RMSE (pre-CPA)",    "range_rmse_precpa","m",  False, None),
        ("position track RMSE (pre-CPA)", "pos_rmse",         "m",  False, "n_pos"),
        ("terminal coverage",             "term_cov",         "",   True,  "n_term"),
        ("LOS-rate error proxy [thin]",   "los_rmse",         "d/s",False, "n_los"),
    ]
    sec_deltas = {}
    for label, key, unit, hib, tickkey in sec:
        ab_summary = summarize_map(ab_maps.get(key, {}))
        tick_note = ""
        if tickkey:
            ab_t = ab_ticks.get(tickkey, 0)
            ek_t = ekf_ticks.get(tickkey, 0) if ekf_present else 0
            tick_note = f"  [ticks ab={ab_t}" + (f", ekf={ek_t}]" if ekf_present else "]")
        print(f"  {label}")
        print(f"      alpha-beta: {ab_summary}{tick_note}")
        if ekf_present:
            print(f"      ekf       : {summarize_map(ekf_maps.get(key, {}))}")
            d = paired_delta(ab_maps.get(key, {}), ekf_maps.get(key, {}),
                             higher_is_better=hib, noise_floor=0.0)
            sec_deltas[key] = d
            print(f"      paired    : {fmt_delta(d, unit, higher_is_better=hib)}")
        print()

    # flight-CSV read notes
    all_notes = {}
    for rk, nt in ab_notes.items():
        all_notes.setdefault(("ab", rk), nt)
    for rk, nt in ekf_notes.items():
        all_notes.setdefault(("ekf", rk), nt)
    if all_notes:
        print("  flight-CSV notes:")
        for (arm, rk), nt in sorted(all_notes.items()):
            print(f"      [{arm} run {rk}] {nt}")
        print()

    # ---- 3. VERDICT -----------------------------------------------------------
    print("-" * 78)
    print("3. VERDICT (pre-registered, brief sec.4.4)")
    print("-" * 78)

    if not ekf_present:
        print("  INCOMPLETE -- EKF arm not found. Baseline (alpha-beta) metrics")
        print("  are printed above for reference. Fly the EKF arm")
        print(f"  ({os.path.basename(ekf_path)}, --tracker ekf, same seeds) to")
        print("  complete the paired A/B, then re-run this analyzer.")
        print("=" * 78)
        return 0

    n_pairs = miss_delta["n"] if miss_delta else 0
    underpowered = n_pairs < PAIRED_MIN_N

    miss_favors_ekf = miss_delta and miss_delta["favors"] == "ekf"
    miss_favors_ab = miss_delta and miss_delta["favors"] == "alphabeta"

    # secondary "improves significantly" = paired CI clears 0 in the EKF
    # direction with n>=min_n; "regresses" = clears 0 the other way.
    sec_improves = [k for k, d in sec_deltas.items() if d.get("favors") == "ekf"]
    sec_regress = [k for k, d in sec_deltas.items() if d.get("favors") == "alphabeta"]

    if miss_favors_ekf and not sec_regress:
        verdict = "SUCCESS"
        why = ("miss paired-delta 95% CI excludes 0 and favors EKF, no "
               "regression -> adopt EKF as default (--tracker ekf).")
    elif miss_favors_ab or sec_regress:
        verdict = "NULL / REJECT"
        why = ("EKF is significantly WORSE on "
               + (", ".join(["miss"] * bool(miss_favors_ab) + sec_regress))
               + " -> keep alpha-beta default; document EKF as tested-and-rejected "
                 "(like Kalata, ADR-0013).")
    elif sec_improves:
        verdict = "PARTIAL"
        why = ("track quality improves ("
               + ", ".join(sec_improves)
               + ") but the end-to-end miss delta straddles 0 -> keep EKF "
                 "opt-in (--tracker ekf, documented). This is the MOST LIKELY "
                 "outcome and confirms the ADR-0023 kinematic floor.")
    else:
        verdict = "NULL"
        why = ("no significant improvement on any metric (miss tied, track "
               "quality tied) -> keep alpha-beta default; a reproducible "
               "negative result is a result (like Kalata, ADR-0013).")

    prefix = "PRELIMINARY " if underpowered else ""
    print(f"  >>> {prefix}{verdict} <<<")
    print(f"  {why}")
    if underpowered:
        print(f"  NOTE: n={n_pairs} paired < {PAIRED_MIN_N} -- NOT SIGNIFICANT at this n; "
              "verdict is provisional until the arms reach n>=8.")
    if miss_delta and miss_delta["favors"] == "tied" and not underpowered:
        print("  Honest read: the miss delta is NOT significant at this n "
              "(CI straddles 0) -- the expected, pre-registered answer.")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
