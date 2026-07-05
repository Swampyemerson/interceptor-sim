#!/usr/bin/env python3
"""Monte-Carlo analysis: reads one scripts/mc_batch.sh aggregate CSV and
computes the headline number for this project (GOALS.md M5): **Pk**, the
probability that a flight's closest approach ("miss distance") beats a
given lethal radius R_lethal, as a CURVE over several plausible radii, each
with a Wilson 95% confidence interval -- so a "95%" claim always carries an
honest uncertainty band instead of a bare point estimate (critical at the
flight counts this project can afford, e.g. n=20).

WHAT A WILSON INTERVAL IS AND WHY (not the more familiar "p +/- 1.96*sqrt(p
(1-p)/n)" normal-approximation interval): that normal approximation gets
badly overconfident exactly where Monte-Carlo batches like this one often
land -- p near 0 or 1, or n small (both true here: a handful of flights,
and a Pk that may be near 1 at large R_lethal or near 0 at small
R_lethal). Wilson's interval (E.B. Wilson, 1927) inverts the normal
approximation to the binomial test statistic instead of the estimate
itself, which keeps it well-behaved (never goes below 0 or above 1, stays
sane at p=0 or p=1) in exactly those regimes. Standard reference: any
intro to categorical data analysis (e.g. Agresti, "Categorical Data
Analysis"); this is the textbook formula, not a novel derivation.

WHY A NAN MISS COUNTS AS A MISS AT EVERY RADIUS: a flight that crashed,
timed out, or never reached HANDOFF produced no closest-approach number at
all (scripts/mc_batch.sh logs miss_m=nan for it) -- but "the interceptor's
software fell over" is not a kill by any honest definition, so it must
never silently drop out of the denominator. Pk here is computed over ALL
attempted flights (including nan ones as failures), not just the flights
that completed cleanly -- see main() for exactly how this is enforced (nan
< anything is False in Python, which does the right thing automatically as
long as nan rows stay IN the flight count).

Run:
    .venv/bin/python scripts/mc_analyze.py                     # newest logs/mc_batch_*.csv
    .venv/bin/python scripts/mc_analyze.py logs/mc_batch_....csv

Output: a printed table (overall clean/handoff rates, miss-distance
distribution over the VALID -- non-nan -- flights, the nan-failure
breakdown by breakoff_reason, and the Pk-vs-radius table with Wilson CIs),
plus two matplotlib figures (Agg backend, no GUI needed) saved to plots/:
  plots/pk_vs_radius_<UTCstamp>.png  -- Pk vs R_lethal with a Wilson CI band
  plots/miss_cdf_<UTCstamp>.png      -- miss-distance CDF over valid flights
"""

import csv
import glob
import math
import os
import statistics
import sys
from collections import Counter
from datetime import datetime, timezone

import matplotlib
matplotlib.use("Agg")  # headless by default, GOALS.md
import matplotlib.pyplot as plt

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")
PLOTS_DIR = os.path.join(REPO_ROOT, "plots")

# Lethal-radius sweep (task spec, meters).
RADII_M = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0]
Z_95 = 1.959963984540054  # two-sided 95% normal quantile


def find_csv(argv):
    if len(argv) > 1:
        path = argv[1]
        if not os.path.isfile(path):
            raise SystemExit(f"mc_analyze: no such file: {path}")
        return path
    candidates = sorted(glob.glob(os.path.join(LOGS_DIR, "mc_batch_*.csv")))
    if not candidates:
        raise SystemExit(f"mc_analyze: no logs/mc_batch_*.csv files found in {LOGS_DIR}")
    return candidates[-1]


def wilson_ci(k, n, z=Z_95):
    """Wilson score interval for a binomial proportion. Returns (p_hat, lo,
    hi). n=0 is degenerate (no data) -- returns all zeros rather than
    raising, since a batch with zero rows should be caught earlier."""
    if n <= 0:
        return 0.0, 0.0, 0.0
    p = k / n
    denom = 1.0 + z * z / n
    center = p + z * z / (2.0 * n)
    margin = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n))
    lo = (center - margin) / denom
    hi = (center + margin) / denom
    return p, max(0.0, lo), min(1.0, hi)


def percentile(sorted_vals, pct):
    """Linear-interpolation percentile (no numpy -- GOALS.md minimal deps,
    same convention as numpy.percentile's default 'linear' method)."""
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * (pct / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_vals[int(k)]
    return sorted_vals[int(f)] * (c - k) + sorted_vals[int(c)] * (k - f)


def parse_miss(row):
    try:
        return float(row.get("miss_m", ""))
    except (TypeError, ValueError):
        return float("nan")


def main():
    path = find_csv(sys.argv)
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))

    n_total = len(rows)
    if n_total == 0:
        raise SystemExit(f"mc_analyze: {path} has a header but zero data rows")

    misses = [parse_miss(r) for r in rows]
    n_nan = sum(1 for m in misses if math.isnan(m))
    valid = sorted(m for m in misses if not math.isnan(m))
    n_valid = len(valid)

    n_clean = sum(1 for r in rows if r.get("clean", "") == "1")
    handoff_vals = [r.get("handoff", "") for r in rows]
    n_handoff_reported = sum(1 for v in handoff_vals if v in ("0", "1"))
    n_handoff = sum(1 for v in handoff_vals if v == "1")

    print(f"=== Monte-Carlo analysis: {path} ===")
    print(f"Total flights attempted: {n_total}")
    print(f"Clean flights:           {n_clean}/{n_total} ({100.0 * n_clean / n_total:.1f}%)")
    if n_handoff_reported:
        print(
            f"Handoff reached:         {n_handoff}/{n_handoff_reported} "
            f"({100.0 * n_handoff / n_handoff_reported:.1f}% of flights with a handoff value)"
        )
    else:
        print("Handoff reached:         n/a (no 'handoff' values in this CSV -- non-handoff mode?)")
    print(f"Failed flights (miss=nan): {n_nan}/{n_total} ({100.0 * n_nan / n_total:.1f}%)")

    if n_nan:
        print("\n--- Failure breakdown (nan flights, by breakoff_reason) ---")
        reasons = Counter(
            (r.get("breakoff_reason", "") or "(blank)")
            for r, m in zip(rows, misses) if math.isnan(m)
        )
        for reason, cnt in reasons.most_common():
            print(f"  {cnt:>3d}x  {reason}")

    if n_valid:
        mean_v = statistics.mean(valid)
        median_v = statistics.median(valid)
        std_v = statistics.stdev(valid) if n_valid > 1 else 0.0
        p50 = percentile(valid, 50)
        p90 = percentile(valid, 90)
        p95 = percentile(valid, 95)
        print(f"\n--- Miss-distance distribution (valid flights only, n={n_valid}) ---")
        print(f"  mean={mean_v:.3f} m   median={median_v:.3f} m   std={std_v:.3f} m")
        print(f"  min={valid[0]:.3f} m   max={valid[-1]:.3f} m")
        print(f"  p50={p50:.3f} m   p90={p90:.3f} m   p95={p95:.3f} m")
    else:
        print("\nNo valid (non-nan) miss values -- cannot compute a distribution.")

    print(
        f"\n--- Pk (probability of kill) vs lethal radius, Wilson 95% CI "
        f"(n={n_total}, nan flights counted as a miss at every radius) ---"
    )
    print(f"{'R_lethal_m':>10} {'kills':>6} {'Pk':>8} {'CI_low':>8} {'CI_high':>8}")
    pk_rows = []
    for r_lethal in RADII_M:
        k = sum(1 for m in misses if (not math.isnan(m)) and m < r_lethal)
        p, lo, hi = wilson_ci(k, n_total)
        pk_rows.append((r_lethal, k, p, lo, hi))
        print(f"{r_lethal:>10.2f} {k:>6d} {p:>8.3f} {lo:>8.3f} {hi:>8.3f}")

    os.makedirs(PLOTS_DIR, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    basename = os.path.basename(path)

    # --- Pk vs radius, with Wilson CI band ---
    radii = [r[0] for r in pk_rows]
    pk = [r[2] for r in pk_rows]
    lo = [r[3] for r in pk_rows]
    hi = [r[4] for r in pk_rows]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(radii, pk, marker="o", color="C0", label="Pk (fraction with miss < R)")
    ax.fill_between(radii, lo, hi, color="C0", alpha=0.2, label="Wilson 95% CI")
    ax.axhline(0.95, color="red", linestyle="--", linewidth=1, label="95% target")
    ax.set_xlabel("Lethal radius R_lethal (m)")
    ax.set_ylabel("Pk (probability of kill)")
    ax.set_ylim(-0.02, 1.05)
    ax.set_title(f"Pk vs lethal radius (n={n_total} flights)\n{basename}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    pk_plot_path = os.path.join(PLOTS_DIR, f"pk_vs_radius_{stamp}.png")
    fig.savefig(pk_plot_path, dpi=150)
    plt.close(fig)

    # --- miss-distance CDF (valid flights only; nan count annotated) ---
    fig2, ax2 = plt.subplots(figsize=(7, 5))
    if n_valid:
        ys = [(i + 1) / n_valid for i in range(n_valid)]
        ax2.step(valid, ys, where="post", color="C1")
        ax2.set_xlabel("Miss distance (m)")
        ax2.set_ylabel("CDF (fraction of valid flights <= x)")
        ax2.grid(True, alpha=0.3)
    else:
        ax2.text(0.5, 0.5, "No valid miss values", ha="center", va="center", transform=ax2.transAxes)
    ax2.set_title(f"Miss-distance CDF (n_valid={n_valid}, n_nan={n_nan})\n{basename}")
    fig2.tight_layout()
    cdf_plot_path = os.path.join(PLOTS_DIR, f"miss_cdf_{stamp}.png")
    fig2.savefig(cdf_plot_path, dpi=150)
    plt.close(fig2)

    print(f"\nSaved: {pk_plot_path}")
    print(f"Saved: {cdf_plot_path}")


if __name__ == "__main__":
    main()
