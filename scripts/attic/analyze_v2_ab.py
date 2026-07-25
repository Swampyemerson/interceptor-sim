#!/usr/bin/env python3
"""Seeker v2 paired A/B analyzer (docs/seeker_v2_plan.md gates 2-4).

Reads the mc_batch-style aggregate arm CSVs (one row per flight: miss_m,
clean, handoff, coverage, cue_seed, flight_csv_path, ...) that
scripts/mc_analyze.py / scripts/ekf_ab_analyze.py already know how to parse,
PLUS the per-run flight CSVs each row points at (flight_csv_path ->
m4_intercept_*.csv, one row per sim tick: detected, meas_range, bearing_rad,
gt_range, ...) to compute the MECHANISM metrics (pollution signature, range
bias, terminal coverage) that back gates 2a and 4.

Default arms (docs/seeker_v2_plan.md, ADR-0040):
  apriltag  = logs/mc_ab_apriltag.csv             (baseline, n=8, master-seed 42)
  markerless= logs/mc_ab_markerless.csv           (off-the-shelf two-stage, ADR-0038)
  tuned_v1  = logs/mc_ab_markerless_tuned.csv     (in-domain fine-tune, ADR-0040)
  tuned_v2  = logs/mc_ab_markerless_tuned_v2.csv  (hard-negatives fine-tune, not
              flown yet as of this script's authoring -- absent CSV degrades to
              PENDING gate verdicts, not a crash).

REUSE, NOT REINVENT: fnum()/rms()/mean_sd()/paired_delta() come from
scripts/ekf_ab_analyze.py (the project's existing paired-stats helpers);
percentile() comes from scripts/mc_analyze.py. Both are imported as modules
(sys.path trick, same pattern scripts/seeker/calibrate_range.py already uses
to import a sibling script) -- their own main()/argparse never runs because
each is guarded by `if __name__ == "__main__":`.

MECHANISM METRICS (the load-bearing part, gates 2a/4): for each
MARKERLESS-FAMILY arm (any arm whose label does not contain "apriltag"),
pool every per-tick row across that arm's 8 flight CSVs where the seeker
produced a fresh detection (detected == "1") and compute:
  (a) pollution fraction  = fraction of NEAR-BORESIGHT detections
      (|bearing_rad| <= --near-boresight-deg, default 30 deg -- the exact
      self-mask bearing gate FinetunedNNSeeker enforces at inference,
      scripts/seeker/finetuned_seeker.py:58/71) whose meas_range/gt_range
      ratio is < --pollution-ratio-thresh (default 0.2). This is what
      ADR-0040's addendum called "the pollution signature" (v1: ~0.75 of
      near-boresight detections are >5x too close -- own-airframe / oversized
      boxes still slipping through the self-mask).
  (b) median meas/gt ratio among those same near-boresight detections (v1:
      ~0.06 -- the "16x collapse" the addendum names).
  (c) detection coverage INSIDE the terminal window (gt_range <=
      --terminal-range-m, default 5.0 m): fraction of terminal-range ticks
      (pooled across the arm's 8 flights) carrying a fresh detection --
      mirrors ekf_ab_analyze.py's term_cov, restricted to the terminal band
      rather than the whole flight (that whole-flight number is the
      'coverage' column already logged in the aggregate CSV, printed
      separately in the per-arm table).

Note the off-the-shelf 'markerless' arm's two-stage seeker does not enforce
an explicit bearing-degree self-mask (it gates on box-touches-frame-edge
instead, scripts/seeker/two_stage_seeker.py:125) -- applying the SAME
bearing threshold to both arms here is a deliberate apples-to-apples choice
for comparability, not a claim that they share one mechanism.

GATE VERDICTS (docs/seeker_v2_plan.md, gates 2-4 only -- gate 1 is a static
probe this script has no CSV for, gates 5/6 are honesty-audit /
qualitative and are NOT scored here): PASS / FAIL / PENDING (v2 arm CSV
absent). The plan states gate 2a qualitatively ("the pollution signature
collapses ... toward zero") without a numeric bar; POLLUTION_COLLAPSE_MAX
below is THIS SCRIPT's operationalization of that language (documented, not
itself pre-registered) -- treat it as a reviewable default, not gospel.

HONESTY: reads gt_range / gt_cam / gt_tag columns, which are SCORING-ONLY
ground truth (CLAUDE.md honesty boundary) -- never fed to a guidance path.
This script never boots a sim; it is a read-only analyzer over existing logs.

Usage:
  .venv/bin/python scripts/seeker/analyze_v2_ab.py
  .venv/bin/python scripts/seeker/analyze_v2_ab.py \\
      --arms apriltag=logs/mc_ab_apriltag.csv \\
      --arms markerless=logs/mc_ab_markerless.csv \\
      --arms tuned_v1=logs/mc_ab_markerless_tuned.csv \\
      --arms tuned_v2=logs/mc_ab_markerless_tuned_v2.csv \\
      --out-prefix docs/images/seeker_v2_ab
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")  # headless by default, GOALS.md
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))        # scripts/seeker
SCRIPTS_DIR = os.path.dirname(HERE)                        # scripts
REPO = os.path.dirname(SCRIPTS_DIR)                        # repo root
sys.path.insert(0, SCRIPTS_DIR)

from ekf_ab_analyze import fnum, mean_sd, paired_delta, Z_95, NOISE_FLOOR_M, PAIRED_MIN_N  # noqa: E402
from mc_analyze import percentile  # noqa: E402

# --- defaults -------------------------------------------------------------
DEFAULT_ARMS = [
    ("apriltag", "logs/mc_ab_apriltag.csv"),
    ("markerless", "logs/mc_ab_markerless.csv"),
    ("tuned_v1", "logs/mc_ab_markerless_tuned.csv"),
    ("tuned_v2", "logs/mc_ab_markerless_tuned_v2.csv"),
]
DEFAULT_OUT_PREFIX = "docs/images/seeker_v2_ab"

NEAR_BORESIGHT_DEG = 30.0        # FinetunedNNSeeker self-mask bearing gate
POLLUTION_RATIO_THRESH = 0.2     # ADR-0040 addendum's "pollution" cut
TERMINAL_RANGE_M = 5.0           # m4_intercept.py BREAKOFF_ARM/terminal window

# Gate 2/3-specific seeds NAMED IN THE PLAN TEXT (docs/seeker_v2_plan.md
# lines 40, 42) -- these two pairs are not derived from data, they ARE the
# pre-registered criteria ("the two v1 dropout-abort seeds", "the two
# flybys v1 fixed"). Everything else (which 8 seeds exist at all, their
# order, their miss values) comes from the CSVs, never hardcoded.
GATE2B_DROPOUT_SEEDS = ("772247", "33327")     # must go CLEAN in v2
GATE3_FLYBY_FIXED_SEEDS = ("256788", "776647")  # must STAY clean in v2

# This script's operationalization of gate 2a's qualitative "toward zero"
# bar (v1's measured near-boresight pollution fraction is ~0.75, a dominant
# mode) -- not itself in the pre-registered plan text.
POLLUTION_COLLAPSE_MAX = 0.15
RANGE_OK_LO, RANGE_OK_HI = 0.7, 1.3   # gate 4 post-calibration band


def resolve(path):
    return path if os.path.isabs(path) else os.path.join(REPO, path)


def load_arm_csv(path):
    """Read one aggregate arm CSV -> list of row dicts, or None if absent."""
    if not os.path.isfile(path):
        return None
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_flight_ticks(flight_csv):
    if not flight_csv or not os.path.isfile(flight_csv):
        return None
    try:
        with open(flight_csv, newline="") as f:
            return list(csv.DictReader(f))
    except OSError:
        return None


def arm_mechanism_metrics(rows, near_boresight_deg, pollution_thresh, terminal_range_m):
    """Pool per-tick data across every flight_csv_path in this arm's rows.

    Returns (metrics_dict_or_None, n_flight_csv_missing). metrics is None
    only if NOT ONE flight CSV was readable (arm not flown / logs pruned).
    """
    ratios_all = []   # meas/gt on every detected tick, any bearing
    ratios_nb = []    # meas/gt on detected ticks within the near-boresight gate
    n_term_ticks = 0
    n_term_detected = 0
    n_missing = 0
    for r in rows:
        frows = load_flight_ticks(r.get("flight_csv_path", ""))
        if frows is None:
            n_missing += 1
            continue
        for fr in frows:
            g = fnum(fr.get("gt_range"))
            if g is None or g <= 0:
                continue
            if g <= terminal_range_m:
                n_term_ticks += 1
                if (fr.get("detected") or "").strip() == "1":
                    n_term_detected += 1
            if (fr.get("detected") or "").strip() != "1":
                continue
            m = fnum(fr.get("meas_range"))
            if m is None:
                continue
            ratio = m / g
            ratios_all.append(ratio)
            b = fnum(fr.get("bearing_rad"))
            if b is not None and abs(math.degrees(b)) <= near_boresight_deg:
                ratios_nb.append(ratio)
    if n_missing == len(rows):
        return None, n_missing
    n_pollute_nb = sum(1 for x in ratios_nb if x < pollution_thresh)
    metrics = {
        "n_det_all": len(ratios_all),
        "n_det_nb": len(ratios_nb),
        "frac_pollution_nb": (n_pollute_nb / len(ratios_nb)) if ratios_nb else float("nan"),
        "median_ratio_nb": statistics.median(ratios_nb) if ratios_nb else float("nan"),
        "median_ratio_all": statistics.median(ratios_all) if ratios_all else float("nan"),
        "term_coverage": (n_term_detected / n_term_ticks) if n_term_ticks else float("nan"),
        "n_term_ticks": n_term_ticks,
        "n_flight_csv_missing": n_missing,
    }
    return metrics, n_missing


def build_arm(label, path, args):
    """Load + summarize one arm. Returns a dict, or None if the CSV is absent."""
    apath = resolve(path)
    rows = load_arm_csv(apath)
    if rows is None:
        return None
    n = len(rows)
    seeds = [(r.get("cue_seed") or "").strip() for r in rows]
    miss_map = {}
    clean_map = {}
    handoff_vals = []
    coverage_vals = []
    for r, seed in zip(rows, seeds):
        m = fnum(r.get("miss_m"))
        if m is not None:
            miss_map[seed] = m
        c = r.get("clean", "").strip()
        if c in ("0", "1"):
            clean_map[seed] = int(c)
        h = r.get("handoff", "").strip()
        if h in ("0", "1"):
            handoff_vals.append(int(h))
        cov = fnum(r.get("coverage"))
        if cov is not None:
            coverage_vals.append(cov)

    n_clean = sum(clean_map.values())
    n_clean_reported = len(clean_map)
    handoff_rate = (sum(handoff_vals) / len(handoff_vals)) if handoff_vals else float("nan")
    miss_vals = sorted(miss_map.values())

    mech, n_missing_flights = (None, 0)
    is_markerless_family = "apriltag" not in label.lower()
    if is_markerless_family:
        mech, n_missing_flights = arm_mechanism_metrics(
            rows, args.near_boresight_deg, args.pollution_ratio_thresh,
            args.terminal_range_m)

    return {
        "label": label, "path": apath, "rows": rows, "n": n,
        "seed_order": seeds, "miss_map": miss_map, "clean_map": clean_map,
        "n_clean": n_clean, "n_clean_reported": n_clean_reported,
        "handoff_rate": handoff_rate,
        "median_miss": percentile(miss_vals, 50) if miss_vals else float("nan"),
        "mean_miss": statistics.mean(miss_vals) if miss_vals else float("nan"),
        "median_coverage": (statistics.median(coverage_vals)
                             if coverage_vals else float("nan")),
        "is_markerless_family": is_markerless_family,
        "mechanism": mech,
        "n_flight_csv_missing": n_missing_flights,
    }


# --- paired stats -----------------------------------------------------------

def paired_seed_diffs(a_map, b_map):
    """b - a over shared, finite-valued seed keys. Returns (seeds, diffs)."""
    seeds, diffs = [], []
    for k in a_map:
        if k in b_map:
            seeds.append(k)
            diffs.append(b_map[k] - a_map[k])
    return seeds, diffs


def print_paired_miss_delta(ref_label, ref_arm, arm):
    seeds, diffs = paired_seed_diffs(ref_arm["miss_map"], arm["miss_map"])
    if not diffs:
        print(f"    vs {ref_label:10s}: no shared seeds with valid miss values")
        return
    med = statistics.median(diffs)
    d = paired_delta(ref_arm["miss_map"], arm["miss_map"],
                      higher_is_better=False, noise_floor=NOISE_FLOOR_M)
    mean_s = f"{d['mean']:+.3f}" if d["mean"] is not None else "n/a"
    print(f"    vs {ref_label:10s}: median miss delta={med:+.3f} m  "
          f"mean={mean_s} m  n={len(diffs)} paired seeds")
    if abs(med) < NOISE_FLOOR_M:
        print(f"        (|delta| < {NOISE_FLOOR_M:g} m run-to-run terminal-dropout "
              f"noise floor -- not significant at this n, CLAUDE.md)")
    elif len(diffs) < PAIRED_MIN_N:
        print(f"        (n={len(diffs)} < {PAIRED_MIN_N} paired seeds -- "
              f"not significant at this n)")
    # clean-rate delta over the SAME paired seeds
    ref_c = ref_arm["clean_map"]
    arm_c = arm["clean_map"]
    common_c = [s for s in seeds if s in ref_c and s in arm_c]
    if common_c:
        ref_rate = sum(ref_c[s] for s in common_c) / len(common_c)
        arm_rate = sum(arm_c[s] for s in common_c) / len(common_c)
        print(f"    vs {ref_label:10s}: clean-rate delta="
              f"{arm_rate - ref_rate:+.3f} "
              f"({sum(arm_c[s] for s in common_c)}/{len(common_c)} vs "
              f"{sum(ref_c[s] for s in common_c)}/{len(common_c)})")


# --- gate verdicts ------------------------------------------------------------

def print_gate_verdicts(arms):
    v1 = arms.get("tuned_v1")
    v2 = arms.get("tuned_v2")
    print("\n--- Gate verdicts (docs/seeker_v2_plan.md gates 2-4; gate 1/5/6 out of "
          "scope for this CSV-only analyzer) ---")
    if v1 is None:
        print("  PENDING: no tuned_v1 arm loaded -- gates are defined relative to it, "
              "nothing to compare against.")
        return
    v1_mech = v1["mechanism"]
    print(f"  v1 baseline (tuned_v1, {v1['path']}): "
          f"clean={v1['n_clean']}/{v1['n_clean_reported']}  "
          f"median_coverage={v1['median_coverage']:.3f}")
    if v1_mech:
        print(f"    v1 mechanism: pollution_frac_nb={v1_mech['frac_pollution_nb']:.3f}  "
              f"median_ratio_nb={v1_mech['median_ratio_nb']:.3f}  "
              f"(n_nb={v1_mech['n_det_nb']})  term_coverage(<= "
              f"{TERMINAL_RANGE_M:g} m)={v1_mech['term_coverage']:.3f}")

    if v2 is None:
        print("  GATE 2 (PRIMARY clean-rate):  PENDING -- tuned_v2 arm CSV not found "
              "(fly the v2 A/B first, then re-run this analyzer).")
        print("  GATE 3 (no regression):       PENDING -- same reason.")
        print("  GATE 4 (range, secondary):    PENDING -- same reason.")
        return

    v2_mech = v2["mechanism"]
    ap = arms.get("apriltag")

    # ---- Gate 2: PRIMARY clean-rate ----
    clean_rate_ok = v2["n_clean"] >= 7
    pollution_ok = (v2_mech is not None and not math.isnan(v2_mech["frac_pollution_nb"])
                     and v2_mech["frac_pollution_nb"] <= POLLUTION_COLLAPSE_MAX)
    dropout_seeds_ok = all(v2["clean_map"].get(s) == 1 for s in GATE2B_DROPOUT_SEEDS
                            if s in v2["clean_map"])
    dropout_seeds_present = all(s in v2["clean_map"] for s in GATE2B_DROPOUT_SEEDS)
    gate2 = clean_rate_ok and pollution_ok and dropout_seeds_ok and dropout_seeds_present
    print(f"  GATE 2 (PRIMARY clean-rate):  {'PASS' if gate2 else 'FAIL'}")
    print(f"    clean-rate {v2['n_clean']}/8 >= 7           : "
          f"{'ok' if clean_rate_ok else 'FAIL'}")
    if v2_mech:
        print(f"    2a pollution collapse (<= {POLLUTION_COLLAPSE_MAX:g}, "
              f"v1 was {v1_mech['frac_pollution_nb']:.3f}): "
              f"v2={v2_mech['frac_pollution_nb']:.3f}  "
              f"{'ok' if pollution_ok else 'FAIL'}")
    else:
        print("    2a pollution collapse: FAIL (no readable v2 flight-tick data)")
    print(f"    2b dropout seeds {GATE2B_DROPOUT_SEEDS} now clean: "
          f"{'ok' if (dropout_seeds_ok and dropout_seeds_present) else 'FAIL'}"
          f"{'' if dropout_seeds_present else ' (seed missing from v2 CSV)'}")

    # ---- Gate 3: no regression ----
    coverage_ok = (not math.isnan(v2["median_coverage"])
                   and v2["median_coverage"] >= v1["median_coverage"])
    gap_v1 = gap_v2 = None
    gap_ok = True
    if ap is not None:
        _, gd_v1 = paired_seed_diffs(ap["miss_map"], v1["miss_map"])
        _, gd_v2 = paired_seed_diffs(ap["miss_map"], v2["miss_map"])
        gap_v1 = statistics.median(gd_v1) if gd_v1 else float("nan")
        gap_v2 = statistics.median(gd_v2) if gd_v2 else float("nan")
        gap_ok = (not math.isnan(gap_v2)) and gap_v2 <= gap_v1
    flyby_ok = all(v2["clean_map"].get(s) == 1 for s in GATE3_FLYBY_FIXED_SEEDS
                   if s in v2["clean_map"])
    flyby_present = all(s in v2["clean_map"] for s in GATE3_FLYBY_FIXED_SEEDS)
    gate3 = coverage_ok and gap_ok and flyby_ok and flyby_present
    print(f"  GATE 3 (no regression):       {'PASS' if gate3 else 'FAIL'}")
    print(f"    coverage v2={v2['median_coverage']:.3f} >= v1={v1['median_coverage']:.3f}: "
          f"{'ok' if coverage_ok else 'FAIL'}")
    if gap_v1 is not None:
        print(f"    median gap-to-tag v2={gap_v2:+.3f} m <= v1={gap_v1:+.3f} m: "
              f"{'ok' if gap_ok else 'FAIL'}")
    else:
        print("    median gap-to-tag: FAIL (no apriltag arm loaded to compute the gap)")
    print(f"    flyby-fixed seeds {GATE3_FLYBY_FIXED_SEEDS} stay clean: "
          f"{'ok' if (flyby_ok and flyby_present) else 'FAIL'}"
          f"{'' if flyby_present else ' (seed missing from v2 CSV)'}")

    # ---- Gate 4: range, secondary ----
    range_ok = (v2_mech is not None and not math.isnan(v2_mech["median_ratio_nb"])
                and RANGE_OK_LO <= v2_mech["median_ratio_nb"] <= RANGE_OK_HI)
    print(f"  GATE 4 (range, secondary):    {'PASS' if range_ok else 'FAIL'}")
    if v2_mech:
        print(f"    median meas/gt ratio (near-boresight) v2={v2_mech['median_ratio_nb']:.3f} "
              f"in [{RANGE_OK_LO}, {RANGE_OK_HI}] (v1 was {v1_mech['median_ratio_nb']:.3f}): "
              f"{'ok' if range_ok else 'FAIL'}")
    else:
        print("    median meas/gt ratio: FAIL (no readable v2 flight-tick data)")


# --- plotting -----------------------------------------------------------------

TAB10 = plt.get_cmap("tab10").colors
WONG_BLUE, WONG_ORANGE, WONG_GREEN = "#0072B2", "#E69F00", "#009E73"


def plot_panels(arms, seed_order, out_path, args):
    present = [a for a in arms.values() if a is not None]
    if not present:
        print("[plot] no arms loaded -- skipping figure")
        return

    fig, (axl, axr) = plt.subplots(1, 2, figsize=(13.5, 5.6))

    # ---- LEFT: per-seed paired miss, one line per seed across arms ----
    arm_labels = [a["label"] for a in present]
    xs = list(range(len(arm_labels)))
    for si, seed in enumerate(seed_order):
        color = TAB10[si % len(TAB10)]
        xs_seed, ys_seed = [], []
        for xi, arm in zip(xs, present):
            if seed in arm["miss_map"]:
                xs_seed.append(xi)
                ys_seed.append(arm["miss_map"][seed])
        if not xs_seed:
            continue
        axl.plot(xs_seed, ys_seed, color=color, linewidth=1.4, alpha=0.85,
                  marker="o", markersize=6, zorder=3)
        for xi, arm in zip(xs, present):
            if seed in arm["clean_map"] and arm["clean_map"][seed] == 0 and seed in arm["miss_map"]:
                axl.scatter([xi], [arm["miss_map"][seed]], marker="x", s=90,
                            color=color, zorder=4)
        axl.annotate(seed, (xs_seed[-1], ys_seed[-1]), color=color, fontsize=7.5,
                      xytext=(5, 0), textcoords="offset points", va="center")
    axl.set_xticks(xs)
    axl.set_xticklabels(arm_labels, fontsize=9)
    axl.set_ylabel("miss distance (m)")
    axl.set_title("Per-seed paired miss by arm\n(line = same cue_seed; x = abort/dirty)",
                   fontsize=10)
    axl.grid(alpha=0.25)
    axl.axhline(0.5, color="#d03b3b", ls=":", lw=1, alpha=0.7)
    axl.text(xs[-1], 0.5, " ram 0.5 m", color="#d03b3b", fontsize=7, va="bottom")

    # ---- RIGHT: pollution-signature fraction + terminal coverage by arm ----
    mech_arms = [a for a in present if a["is_markerless_family"] and a["mechanism"]]
    if mech_arms:
        names = [a["label"] for a in mech_arms]
        pollution = [a["mechanism"]["frac_pollution_nb"] for a in mech_arms]
        term_cov = [a["mechanism"]["term_coverage"] for a in mech_arms]
        xr = range(len(names))
        w = 0.38
        axr.bar([x - w / 2 for x in xr], pollution, w, color=WONG_ORANGE,
                label=f"pollution frac (ratio<{args.pollution_ratio_thresh:g}, "
                      f"|bearing|<={args.near_boresight_deg:g} deg)")
        axr.bar([x + w / 2 for x in xr], term_cov, w, color=WONG_BLUE,
                label=f"coverage inside {args.terminal_range_m:g} m")
        for x, p, c in zip(xr, pollution, term_cov):
            if not math.isnan(p):
                axr.text(x - w / 2, p + 0.02, f"{p:.2f}", ha="center", fontsize=8)
            if not math.isnan(c):
                axr.text(x + w / 2, c + 0.02, f"{c:.2f}", ha="center", fontsize=8)
        axr.set_xticks(list(xr))
        axr.set_xticklabels(names, fontsize=9)
        axr.set_ylim(0, 1.08)
        axr.set_ylabel("fraction")
        axr.set_title("Mechanism: detection pollution vs terminal coverage\n"
                       "(markerless-family arms only)", fontsize=10)
        axr.legend(loc="upper right", fontsize=7.5, frameon=False)
        axr.grid(alpha=0.2, axis="y")
    else:
        axr.text(0.5, 0.5, "no markerless-family mechanism data", ha="center",
                  va="center", transform=axr.transAxes)
        axr.axis("off")

    fig.suptitle("Seeker v2 paired A/B (master-seed 42, n=8) -- "
                  "docs/seeker_v2_plan.md gates 2-4", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"\nSaved: {out_path}")


# --- main ---------------------------------------------------------------------

def parse_arms_arg(arms_arg):
    if not arms_arg:
        return DEFAULT_ARMS
    out = []
    for entry in arms_arg:
        if "=" not in entry:
            raise SystemExit(f"--arms entry must be label=csvpath, got: {entry!r}")
        label, path = entry.split("=", 1)
        out.append((label.strip(), path.strip()))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arms", action="append", default=None,
                    help="repeatable label=csvpath; default: apriltag/markerless/"
                         "tuned_v1/tuned_v2 (see DEFAULT_ARMS)")
    p.add_argument("--out-prefix", default=DEFAULT_OUT_PREFIX,
                    help="plot written to <out-prefix>.png")
    p.add_argument("--near-boresight-deg", type=float, default=NEAR_BORESIGHT_DEG)
    p.add_argument("--pollution-ratio-thresh", type=float, default=POLLUTION_RATIO_THRESH)
    p.add_argument("--terminal-range-m", type=float, default=TERMINAL_RANGE_M)
    args = p.parse_args(argv)

    arm_specs = parse_arms_arg(args.arms)

    print("=" * 78)
    print("Seeker v2 paired A/B analysis (docs/seeker_v2_plan.md)")
    print("=" * 78)
    arms = {}
    for label, path in arm_specs:
        arm = build_arm(label, path, args)
        arms[label] = arm
        status = f"n={arm['n']} rows" if arm else "NOT FOUND"
        print(f"  [{label:10s}] {resolve(path)}  -- {status}")
    print()

    loaded = {lbl: a for lbl, a in arms.items() if a is not None}
    if not loaded:
        print("FATAL: no arm CSVs found. Nothing to analyze.")
        return 2

    # canonical seed order: first loaded arm's row order (all plan arms share
    # master-seed 42, so this should be identical across arms).
    seed_order = next(iter(loaded.values()))["seed_order"]

    # ---- 1. per-arm table ----
    print("-" * 78)
    print("1. Per-arm summary")
    print("-" * 78)
    for label, _ in arm_specs:
        arm = arms.get(label)
        if arm is None:
            print(f"  [{label}] NOT FOUND -- skipped")
            continue
        print(f"  [{label}]  n={arm['n']}  "
              f"clean={arm['n_clean']}/{arm['n_clean_reported']} "
              f"({100.0 * arm['n_clean'] / arm['n_clean_reported']:.0f}%)  "
              f"handoff={arm['handoff_rate']:.2f}  "
              f"median_miss={arm['median_miss']:.3f} m  "
              f"mean_miss={arm['mean_miss']:.3f} m  "
              f"median_coverage={arm['median_coverage']:.3f}")
        seed_bits = []
        for seed in seed_order:
            m = arm["miss_map"].get(seed)
            c = arm["clean_map"].get(seed)
            flag = "" if c is None else ("" if c == 1 else "*")
            m_s = f"{m:.2f}{flag}" if m is not None else "n/a"
            seed_bits.append(f"{seed}:{m_s}")
        print(f"      per-seed miss (*=dirty/abort): " + "  ".join(seed_bits))
    print()

    # ---- 2. paired deltas vs apriltag and vs tuned_v1 ----
    print("-" * 78)
    print(f"2. Paired deltas vs apriltag / vs tuned_v1 "
          f"(noise floor {NOISE_FLOOR_M:g} m, min n={PAIRED_MIN_N} for significance)")
    print("-" * 78)
    for label, _ in arm_specs:
        arm = arms.get(label)
        if arm is None:
            continue
        print(f"  [{label}]")
        for ref_label in ("apriltag", "tuned_v1"):
            if ref_label == label:
                continue
            ref = arms.get(ref_label)
            if ref is None:
                print(f"    vs {ref_label:10s}: reference arm not loaded")
                continue
            print_paired_miss_delta(ref_label, ref, arm)
    print()

    # ---- 3. mechanism metrics ----
    print("-" * 78)
    print(f"3. Mechanism metrics (markerless-family arms only; near-boresight = "
          f"|bearing|<={args.near_boresight_deg:g} deg, pollution = meas/gt "
          f"ratio<{args.pollution_ratio_thresh:g}, terminal window="
          f"{args.terminal_range_m:g} m)")
    print("-" * 78)
    for label, _ in arm_specs:
        arm = arms.get(label)
        if arm is None or not arm["is_markerless_family"]:
            continue
        mech = arm["mechanism"]
        if mech is None:
            print(f"  [{label}] no readable per-tick flight data "
                  f"({arm['n_flight_csv_missing']}/{arm['n']} flight CSVs missing)")
            continue
        print(f"  [{label}]  n_detected_ticks(near-boresight)={mech['n_det_nb']}  "
              f"(all-bearing n={mech['n_det_all']}, "
              f"{mech['n_flight_csv_missing']} flight CSV(s) unreadable)")
        print(f"      (a) pollution fraction (ratio<{args.pollution_ratio_thresh:g}): "
              f"{mech['frac_pollution_nb']:.4f}")
        print(f"      (b) median meas/gt ratio (near-boresight): "
              f"{mech['median_ratio_nb']:.4f}")
        print(f"      (c) detection coverage inside {args.terminal_range_m:g} m: "
              f"{mech['term_coverage']:.4f}  (n_term_ticks={mech['n_term_ticks']})")
    print()

    # ---- ADR-0040 cross-check ----
    print("-" * 78)
    print("Cross-check vs ADR-0040 published numbers")
    print("-" * 78)
    checks = [
        ("apriltag", "n_clean", 8, lambda a: a["n_clean"]),
        ("apriltag", "median_miss", 0.92, lambda a: a["median_miss"]),
        ("markerless", "n_clean", 6, lambda a: a["n_clean"]),
        ("markerless", "mean_miss", 3.39, lambda a: a["mean_miss"]),
        ("tuned_v1", "n_clean", 6, lambda a: a["n_clean"]),
        ("tuned_v1", "mean_miss", 2.45, lambda a: a["mean_miss"]),
    ]
    for label, name, expect, fn in checks:
        arm = arms.get(label)
        if arm is None:
            continue
        got = fn(arm)
        ok = abs(got - expect) < 0.02 if isinstance(expect, float) else got == expect
        print(f"  [{label}] {name}: got={got}  ADR-0040={expect}  "
              f"{'MATCH' if ok else '*** DISAGREES ***'}")
    if arms.get("tuned_v1") is not None:
        ap = arms.get("apriltag")
        v1 = arms["tuned_v1"]
        if ap is not None:
            _, gd = paired_seed_diffs(ap["miss_map"], v1["miss_map"])
            gap = statistics.median(gd) if gd else float("nan")
            expect = 0.66
            ok = abs(gap - expect) < 0.02
            print(f"  [tuned_v1] median gap-to-tag: got={gap:.3f}  ADR-0040={expect}  "
                  f"{'MATCH' if ok else '*** DISAGREES ***'}")
        v1_mech = v1["mechanism"]
        if v1_mech is not None:
            print(f"  [tuned_v1] near-boresight n_det: got={v1_mech['n_det_nb']}  "
                  f"ADR-0040 addendum='n=506'  "
                  f"{'MATCH' if v1_mech['n_det_nb'] == 506 else '(differs -- see note below)'}")
            print(f"  [tuned_v1] median meas/gt ratio (near-boresight): "
                  f"got={v1_mech['median_ratio_nb']:.4f}  ADR-0040 addendum=0.06  "
                  f"{'MATCH' if abs(v1_mech['median_ratio_nb'] - 0.06) < 0.01 else '*** DISAGREES ***'}")
    print()

    # ---- 4. gate verdicts ----
    print_gate_verdicts(arms)

    # ---- plot ----
    out_path = resolve(args.out_prefix + ".png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plot_panels(arms, seed_order, out_path, args)

    return 0


if __name__ == "__main__":
    sys.exit(main())
