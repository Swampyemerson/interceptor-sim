#!/usr/bin/env python3
"""Monte-Carlo analysis for the M5 final batch (ADR-0029 / ADR-0033).

Reads one scripts/mc_batch.sh aggregate CSV (plus, for the trajectory plot,
the per-run flight CSVs it points at) and produces the project's headline
number and the supporting figures:

  * **Pk vs lethal radius, PER ARM (speed x law)** -- the actual ADR-0025
    headline: ADR-0025 (docs/decisions.md L813) and ADR-0014's unanimous
    decision #1 (L246) mandate reporting Pk "per-speed (6/8/10 m/s, never
    pooled)" because a single pooled curve can hide a worse number at the
    hard, high-speed end (ADR-0014 honesty boundary (c), L258). This is a
    small-multiples figure: one subplot per target speed, one Pk-vs-radius
    line (+ Wilson 95% CI band) per guidance law. compute_arm_pk() groups
    rows by (round(speed), law) and reuses compute_pk_rows() per group --
    never a single n=all-flights curve.
  * **Pk vs lethal radius, POOLED (context only, not the headline)** -- the
    same curve collapsed across every speed and law into one n=all-flights
    line. Kept because it is a useful sanity check, but the plot title and
    legend say "POOLED -- masks the hard high-speed end" explicitly so it is
    never mistaken for the per-arm deliverable above. The two mechanism-
    anchored radii from ADR-0025 are drawn as reference lines on both
    figures: kinetic **ram ~= 0.5 m** and **net ~= 1.5 m**.
  * **Miss-distance histogram + CDF per arm** -- the shape of the miss
    distribution for each guidance law (pursuit / pro-nav), not just a mean.
  * **Per-path pursuit-vs-pro-nav comparison** -- one row per engagement path
    (speed x direction, or the explicit path/geometry labels the final batch
    adds), PAIRED-SEED AWARE: when both laws were flown on the same cue_seed
    the comparison pairs them; otherwise it falls back to an unpaired two-sample
    read. Either way it shows n and uses honest small-n language -- the project
    rule is that a single-flight delta under ~1 m is run-to-run noise, and an
    A/B claim needs n>=8 paired seeds plus a mechanism (CLAUDE.md methodology).
  * **Trajectory overlay** -- interceptor and target ground-truth paths per run
    (top-down East-North), CPA marked, laid out as small multiples by path.
    NOTE: these are gt_* positions, which are scoring/visualization ONLY and
    are never read by guidance (the honesty boundary, CLAUDE.md) -- drawing the
    flown geometry from ground truth is legitimate; it is not a guidance input.

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
that completed cleanly -- see compute_pk_rows() for exactly how this is
enforced (nan < anything is False in Python, which does the right thing
automatically as long as nan rows stay IN the flight count).

GRACEFUL WITH NEW COLUMNS: the final ADR-0033 batch adds path/geometry label
columns (e.g. an explicit path name, a west-approach geometry). Everything
here reads columns BY NAME and treats new label columns as optional -- if
they are absent (today's ADR-0029 CSVs), the path key is synthesized from
target speed + crossing direction, so old and new CSVs both analyze.
path_key() ALWAYS folds the (rounded) target speed into both the grouping
key and the display label, even when an explicit label column is present --
two rows sharing a label (e.g. both "oblique_close") but flown at different
speeds must land in different per-path/per-trajectory cells, or they'd
silently pool exactly the way ADR-0025 forbids for the headline Pk figure.

PAIRED-DIFF SIGNIFICANCE USES THE SPREAD, NOT JUST THE MEAN: path_verdict()'s
paired branch computes the paired differences' sample SD (sd_d) and requires
the difference's ~95% CI (mean_d +/- Z_95*sd_d/sqrt(n)) to clear the noise
floor entirely (not overlap +-NOISE_FLOOR_M) before it will print "favours"
-- a big mean difference riding on high-variance paired diffs prints "tied"
even past the n>=8 gate, honest-small-n language otherwise unchanged.

Run:
    .venv/bin/python scripts/mc_analyze.py                     # newest logs/mc_batch_*.csv
    .venv/bin/python scripts/mc_analyze.py logs/mc_batch_....csv
    .venv/bin/python scripts/mc_analyze.py --no-traj           # skip the trajectory plot (fast)

Output: a printed table (overall clean/handoff rates, miss-distance
distribution, nan-failure breakdown, the Pk-vs-radius table with Wilson CIs,
and the per-path pursuit-vs-pro-nav read with its honesty verdict), plus
matplotlib figures (Agg backend, no GUI needed) saved to plots/ with the
repo's timestamped naming:
  plots/pk_vs_radius_by_arm_<UTCstamp>.png -- HEADLINE: Pk vs R_lethal, per speed x law
  plots/pk_vs_radius_<UTCstamp>.png    -- pooled Pk vs R_lethal (context only, labelled)
  plots/miss_hist_cdf_<UTCstamp>.png   -- miss histogram + CDF per law
  plots/per_path_<UTCstamp>.png        -- per-path pursuit vs pro-nav (paired-aware)
  plots/traj_overlay_<UTCstamp>.png    -- interceptor+target paths per run, CPA marked
"""

import csv
import glob
import math
import os
import statistics
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone

import matplotlib
matplotlib.use("Agg")  # headless by default, docs/goals.md
import matplotlib.pyplot as plt

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")
PLOTS_DIR = os.path.join(REPO_ROOT, "plots")

# Lethal-radius sweep (task spec, meters).
RADII_M = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0]
Z_95 = 1.959963984540054  # two-sided 95% normal quantile

# ADR-0025 mechanism-anchored lethal radii (drawn as reference lines, not fit).
RAM_RADIUS_M = 0.5   # kinetic ram (headline)
NET_RADIUS_M = 1.5   # net (fast/FPV forgiveness upgrade)
PK_TARGET = 0.95     # >=95% Pk goal (ADR-0025 / MEMORY)

# Project methodology thresholds for the honest A/B verdict (CLAUDE.md):
#   - a single-flight (or pooled) miss delta under ~1 m is run-to-run noise;
#   - an A/B claim needs at least this many PAIRED seeds plus a mechanism.
NOISE_FLOOR_M = 1.0
PAIRED_MIN_N = 8

# --- dataviz palette (references/palette.md, validated for light + dark) ------
# Categorical hues, FIXED ORDER: pursuit -> slot 1 (blue), pro-nav -> slot 2
# (aqua). Validated adjacent CVD deltaE 73.6 (deutan); light-mode aqua sits below
# 3:1 contrast, so the relief rule is met with distinct MARKERS + direct labels +
# a legend (never colour alone). Any further laws fall through to the remaining
# categorical slots in fixed order.
CATEGORICAL = ["#2a78d6", "#1baf7a", "#eda100", "#008300",
               "#4a3aa7", "#e34948", "#e87ba4", "#eb6834"]
LAW_PIN = {"pursuit": 0, "pronav": 1}
LAW_MARKER = {"pursuit": "o", "pronav": "s"}

INK = "#0b0b0b"          # primary ink (text, CPA marks)
INK2 = "#52514e"         # secondary ink (reference-line labels)
MUTED = "#898781"        # axis / muted labels / target context path
GRID = "#e1e0d9"         # hairline gridline
AXIS = "#c3c2b7"         # baseline / axis spine
SURFACE = "#fcfcfb"      # light chart surface
CRITICAL = "#d03b3b"     # status: the 95% target line (reserved status hue)

DEFAULT_MARKER = "^"


def law_color_map(laws):
    """Assign a categorical hue to each law in FIXED order (pursuit=blue,
    pro-nav=aqua pinned; any others take the next free slots), so colour
    follows the entity and never its rank (dataviz non-negotiable)."""
    used = set()
    out = {}
    for law in laws:
        if law in LAW_PIN and LAW_PIN[law] < len(CATEGORICAL):
            idx = LAW_PIN[law]
        else:
            idx = next(i for i in range(len(CATEGORICAL)) if i not in used and i not in LAW_PIN.values())
        out[law] = CATEGORICAL[idx % len(CATEGORICAL)]
        used.add(idx)
    return out


def style_axes(ax):
    """Recessive grid + axes, thin marks (dataviz marks-and-anatomy)."""
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=1.0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(1.0)
    ax.tick_params(colors=MUTED, labelcolor=INK2, length=3)


def find_csv(args):
    positional = [a for a in args if not a.startswith("-")]
    if positional:
        path = positional[0]
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
    """Linear-interpolation percentile (no numpy -- docs/goals.md minimal deps,
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


def parse_float(row, key):
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return float("nan")


def parse_miss(row):
    return parse_float(row, "miss_m")


# --- path / geometry labelling (graceful with the final batch's new columns) --
# Candidate explicit label columns the ADR-0033 final batch may add. Read by
# name; use whichever are present and non-empty, else synthesize from motion.
PATH_LABEL_COLS = ("path", "path_label", "path_name", "profile", "geometry", "approach")


def speed_of(row):
    """Target crossing speed (m/s) from the velocity components -- robust to
    which axis carries the motion (today target_vx=0, target_vy=+-speed)."""
    vx = parse_float(row, "target_vx")
    vy = parse_float(row, "target_vy")
    vx = 0.0 if math.isnan(vx) else vx
    vy = 0.0 if math.isnan(vy) else vy
    return math.hypot(vx, vy)


def direction_of(row):
    d = (row.get("direction", "") or "").strip()
    if d:
        return d
    # Fall back to the sign of the crossing velocity if no explicit column.
    vy = parse_float(row, "target_vy")
    if not math.isnan(vy) and vy != 0.0:
        return "l2r" if vy > 0 else "r2l"
    return "?"


def path_key(row):
    """A stable, human-readable path label. Prefers the final batch's explicit
    label columns; otherwise synthesizes 'speed dir' from motion. Returns
    (sort_key, label).

    ALWAYS folds the (rounded) target speed into the label/key, even when an
    explicit label column (geometry/path/profile/...) is present. Reviewer
    finding: without this, two rows sharing a label but flown at different
    speeds (e.g. the ADR-0033 final batch's --geometry oblique_close arm at
    two different speeds) silently collapse into ONE per-path/trajectory
    cell, re-triggering the "never pool across speed" problem ADR-0025 flags
    for the headline Pk figure. Rounding to the nearest whole m/s matches the
    nominal 6/9/12 m/s speed grid used across this project's batches."""
    labels = [(row.get(c, "") or "").strip() for c in PATH_LABEL_COLS]
    labels = [x for x in labels if x]
    spd = speed_of(row)
    spd_r = round(spd)
    direction = direction_of(row)
    if labels:
        # Explicit label(s) present -- keep them, still annotate direction
        # AND speed so distinct speeds under one label never pool together.
        text = " / ".join(dict.fromkeys(labels))  # de-dup, keep order
        if direction != "?" and direction not in text.lower():
            text = f"{text} {direction.upper()}"
        text = f"{text} @{spd_r:g}m/s"
        return (spd_r, direction, text), text
    # Synthesized: "6 m/s L2R"
    text = f"{spd:.0f} m/s {direction.upper()}" if direction != "?" else f"{spd:.0f} m/s"
    return (spd_r, direction, text), text


# ----------------------------------------------------------------------------
def compute_pk_rows(misses, n_total):
    rows = []
    for r_lethal in RADII_M:
        k = sum(1 for m in misses if (not math.isnan(m)) and m < r_lethal)
        p, lo, hi = wilson_ci(k, n_total)
        rows.append((r_lethal, k, p, lo, hi))
    return rows


def compute_arm_pk(rows):
    """Group rows by (round(speed), law) and compute a full Pk-vs-radius
    curve PER ARM -- this is the ADR-0025 headline requirement ("report
    per-speed, never pooled", docs/decisions.md L246/L813): a single pooled
    Pk-vs-radius curve can hide a worse number at the hard, high-speed end
    (ADR-0014 honesty boundary (c), L258). nan misses still count in that
    arm's n (same "nan = a miss, never dropped from the denominator" rule as
    compute_pk_rows / the module docstring).

    Returns (arms, speeds, laws): arms maps (speed_bucket, law) -> {"speed":
    float, "law": str, "n": int, "pk_rows": [...]}; speeds/laws are the
    sorted distinct buckets/laws seen, for stable plot/table ordering."""
    grouped = defaultdict(lambda: {"speed": None, "law": None, "misses": []})
    for r in rows:
        spd = speed_of(r)
        law = (r.get("law", "") or "?").strip()
        key = (round(spd), law)
        g = grouped[key]
        g["speed"] = spd
        g["law"] = law
        g["misses"].append(parse_miss(r))
    arms = {}
    for key, g in grouped.items():
        n = len(g["misses"])
        arms[key] = {
            "speed": g["speed"], "law": g["law"], "n": n,
            "pk_rows": compute_pk_rows(g["misses"], n),
        }
    speeds = sorted({k[0] for k in arms})
    laws = sorted({k[1] for k in arms})
    return arms, speeds, laws


def plot_pk_vs_radius_by_arm(arms, speeds, laws, basename, out_path):
    """HEADLINE figure: small multiples, one subplot per target speed, one
    Pk-vs-radius line (+ Wilson 95% CI band) per guidance law. Never mixes
    speeds into a single curve -- this is what ADR-0025's 'report per-speed,
    never pooled' actually requires (see compute_arm_pk)."""
    if not speeds:
        return False
    colors = law_color_map(laws)
    ncols = min(3, len(speeds))
    nrows = math.ceil(len(speeds) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.8 * ncols, 4.6 * nrows),
                             squeeze=False)
    for si, spd in enumerate(speeds):
        ax = axes[si // ncols][si % ncols]
        style_axes(ax)
        for law in laws:
            g = arms.get((spd, law))
            if not g or g["n"] == 0:
                continue
            radii = [r[0] for r in g["pk_rows"]]
            pk = [r[2] for r in g["pk_rows"]]
            lo = [r[3] for r in g["pk_rows"]]
            hi = [r[4] for r in g["pk_rows"]]
            ax.fill_between(radii, lo, hi, color=colors.get(law, INK), alpha=0.16,
                            linewidth=0)
            ax.plot(radii, pk, marker=LAW_MARKER.get(law, DEFAULT_MARKER), markersize=6,
                    color=colors.get(law, INK), linewidth=2, label=f"{law} (n={g['n']})")
        for r_m in (RAM_RADIUS_M, NET_RADIUS_M):
            ax.axvline(r_m, color=INK2, linestyle=":", linewidth=1.0)
        ax.axhline(PK_TARGET, color=CRITICAL, linestyle="--", linewidth=1.0)
        ax.set_ylim(-0.02, 1.08)
        ax.set_title(f"{spd:g} m/s", color=INK, fontsize=11)
        ax.set_xlabel("R_lethal (m)", color=INK2, fontsize=9)
        ax.set_ylabel("Pk (miss < R)", color=INK2, fontsize=9)
        ax.legend(loc="lower right", frameon=False, fontsize=8, labelcolor=INK2)
    for k in range(len(speeds), nrows * ncols):
        axes[k // ncols][k % ncols].axis("off")
    fig.suptitle("Pk vs lethal radius, PER SPEED x LAW (ADR-0025 headline -- never "
                 "pooled; dashed line = 95% target, dotted = ram/net radii)",
                 color=INK, fontsize=12)
    fig.text(0.005, 0.005, basename, fontsize=7.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.01, 1, 0.94))
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return True


def plot_pk_vs_radius(pk_rows, n_total, basename, out_path):
    radii = [r[0] for r in pk_rows]
    pk = [r[2] for r in pk_rows]
    lo = [r[3] for r in pk_rows]
    hi = [r[4] for r in pk_rows]
    fig, ax = plt.subplots(figsize=(8.0, 5.4))
    style_axes(ax)
    ax.fill_between(radii, lo, hi, color=CATEGORICAL[0], alpha=0.18, linewidth=0,
                    label="Wilson 95% CI")
    ax.plot(radii, pk, marker="o", markersize=7, color=CATEGORICAL[0], linewidth=2,
            label=f"Pk pooled, ALL speeds+laws (n={n_total})")
    # ADR-0025 mechanism-anchored radii + the 95% target (status hue, dashed).
    for r_m, name in ((RAM_RADIUS_M, "ram"), (NET_RADIUS_M, "net")):
        ax.axvline(r_m, color=INK2, linestyle=":", linewidth=1.2)
        ax.annotate(f"{name} {r_m:g} m", xy=(r_m, 0.55), xycoords=("data", "axes fraction"),
                    xytext=(-4, 0), textcoords="offset points", rotation=90,
                    ha="right", va="center", fontsize=8, color=INK2)
    ax.axhline(PK_TARGET, color=CRITICAL, linestyle="--", linewidth=1.2)
    ax.annotate(f"{PK_TARGET:.0%} target", xy=(radii[-1], PK_TARGET), xycoords="data",
                ha="right", va="bottom", fontsize=8, color=CRITICAL)
    ax.set_xlabel("Lethal radius R_lethal (m)", color=INK2)
    ax.set_ylabel("Pk (probability of kill)", color=INK2)
    ax.set_ylim(-0.02, 1.08)
    ax.set_title(f"Pk vs lethal radius -- POOLED (n={n_total}), NOT the headline",
                 color=INK, fontsize=12)
    fig.text(0.5, 0.925, "masks the hard high-speed end -- see pk_vs_radius_by_arm "
              "for the ADR-0025 per-speed figure", ha="center", fontsize=8.5, color=INK2)
    ax.legend(loc="center right", frameon=False, fontsize=9, labelcolor=INK2)
    fig.text(0.005, 0.005, basename, fontsize=7.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.02, 1, 0.9))
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def plot_miss_hist_cdf(by_law_valid, n_nan, basename, out_path):
    """Left: grouped histogram of miss per law. Right: empirical CDF per law."""
    laws = sorted(by_law_valid)
    colors = law_color_map(laws)
    all_vals = [m for vals in by_law_valid.values() for m in vals]
    if not all_vals:
        return
    lo_v, hi_v = min(all_vals), max(all_vals)
    if hi_v <= lo_v:
        hi_v = lo_v + 1.0
    n_bins = min(10, max(4, int(round(math.sqrt(len(all_vals))))))
    edges = [lo_v + (hi_v - lo_v) * i / n_bins for i in range(n_bins + 1)]

    fig, (axh, axc) = plt.subplots(1, 2, figsize=(12, 5))
    for ax in (axh, axc):
        style_axes(ax)

    # --- grouped histogram (2px surface gap between the paired bars) ---
    group_w = (edges[1] - edges[0]) * 0.86
    bar_w = group_w / max(1, len(laws))
    for li, law in enumerate(laws):
        vals = by_law_valid[law]
        counts = [0] * n_bins
        for m in vals:
            b = min(n_bins - 1, max(0, int((m - lo_v) / (hi_v - lo_v) * n_bins)))
            counts[b] += 1
        centers = [edges[i] + (edges[i + 1] - edges[i]) / 2.0 for i in range(n_bins)]
        offs = [c - group_w / 2.0 + bar_w * (li + 0.5) for c in centers]
        axh.bar(offs, counts, width=bar_w * 0.92, color=colors[law],
                edgecolor=SURFACE, linewidth=1.5,
                label=f"{law} (n={len(vals)})")
    for r_m in (RAM_RADIUS_M, NET_RADIUS_M):
        axh.axvline(r_m, color=INK2, linestyle=":", linewidth=1.0)
    axh.set_xlabel("Miss distance (m)", color=INK2)
    axh.set_ylabel("Flights", color=INK2)
    ttl = "Miss-distance histogram by law"
    if n_nan:
        ttl += f"   ({n_nan} nan flight(s) excluded from these panels)"
    axh.set_title(ttl, color=INK, fontsize=12)
    axh.legend(loc="upper right", frameon=False, fontsize=9, labelcolor=INK2)

    # --- empirical CDF per law (step, direct-labelled at the top) ---
    for law in laws:
        vals = sorted(by_law_valid[law])
        n = len(vals)
        ys = [(i + 1) / n for i in range(n)]
        xs = [vals[0]] + vals
        ys = [0.0] + ys
        axc.step(xs, ys, where="post", color=colors[law], linewidth=2,
                 marker=LAW_MARKER.get(law, DEFAULT_MARKER), markersize=5,
                 label=f"{law} (n={n})")
    for r_m, name in ((RAM_RADIUS_M, "ram"), (NET_RADIUS_M, "net")):
        axc.axvline(r_m, color=INK2, linestyle=":", linewidth=1.0)
        axc.annotate(f"{name} {r_m:g} m", xy=(r_m, 0.5), xycoords=("data", "axes fraction"),
                     xytext=(-4, 0), textcoords="offset points", rotation=90,
                     ha="right", va="center", fontsize=8, color=INK2)
    axc.set_xlabel("Miss distance (m)", color=INK2)
    axc.set_ylabel("CDF (fraction of valid flights <= x)", color=INK2)
    axc.set_ylim(0, 1.06)
    axc.set_title("Miss-distance CDF by law", color=INK, fontsize=12)
    axc.legend(loc="lower right", frameon=False, fontsize=9, labelcolor=INK2)

    fig.text(0.005, 0.005, basename, fontsize=7.5, color=MUTED)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def per_path_stats(rows):
    """Group by (path, law) -> valid misses and by seed for pairing. Returns a
    list of per-path dicts sorted by the path sort-key."""
    grouped = defaultdict(lambda: {"label": "", "sortk": None,
                                    "by_law": defaultdict(list),
                                    "seed": defaultdict(dict)})
    for r in rows:
        sortk, label = path_key(r)
        m = parse_miss(r)
        law = (r.get("law", "") or "?").strip()
        seed = (r.get("cue_seed", "") or "").strip()
        g = grouped[sortk[2]]  # key on the readable label text
        g["label"] = label
        g["sortk"] = sortk
        if not math.isnan(m):
            g["by_law"][law].append(m)
            if seed:
                g["seed"][seed][law] = m
    out = list(grouped.values())
    out.sort(key=lambda g: g["sortk"])
    return out


def path_verdict(g):
    """Compute the honest pursuit-vs-pro-nav read for one path. Returns
    (mode, n_desc, delta, verdict_text). Paired when the two laws share
    seeds; otherwise unpaired two-sample."""
    laws = sorted(g["by_law"])
    if set(laws) != {"pronav", "pursuit"}:
        return ("single", f"{len(next(iter(g['by_law'].values()), []))}", float("nan"),
                "only one law on this path -- no A/B")
    # Paired seeds (both laws flown on the same cue_seed)?
    pairs = [(v["pursuit"], v["pronav"]) for v in g["seed"].values()
             if "pursuit" in v and "pronav" in v]
    if pairs:
        diffs = [pu - pr for pu, pr in pairs]  # >0 => pro-nav closer
        n = len(diffs)
        mean_d = statistics.mean(diffs)
        sd_d = statistics.stdev(diffs) if n > 1 else float("nan")
        # Honest gate: use the SPREAD of the paired diffs, not just their
        # mean. Approximate 95% CI on mean_d (normal approx on the paired
        # diffs, same Z_95 convention as the Wilson interval elsewhere in
        # this module -- n>=8 is required below before this CI is trusted).
        # "Favours" only fires when that CI clears the noise floor ENTIRELY
        # (doesn't overlap +-NOISE_FLOOR_M) -- a big mean riding on
        # high-variance diffs now prints "tied", not "favours".
        if n > 1 and sd_d > 0:
            margin = Z_95 * sd_d / math.sqrt(n)
        else:
            margin = 0.0
        ci_lo, ci_hi = mean_d - margin, mean_d + margin
        signif = (n >= PAIRED_MIN_N) and (ci_lo > NOISE_FLOOR_M or ci_hi < -NOISE_FLOOR_M)
        if signif:
            better = "pro-nav" if mean_d > 0 else "pursuit"
            v = (f"paired Delta={mean_d:+.2f} m [{ci_lo:+.2f},{ci_hi:+.2f}] "
                 f"favours {better} (n={n})")
        elif n < PAIRED_MIN_N:
            v = f"paired Delta={mean_d:+.2f} m +-{sd_d:.2f} sd, n={n}<8: not significant"
        else:
            v = (f"paired Delta={mean_d:+.2f} m [{ci_lo:+.2f},{ci_hi:+.2f}] "
                 f"overlaps +-{NOISE_FLOOR_M:g}m noise: tied")
        return ("paired", f"{n} pairs", mean_d, v)
    # Unpaired two-sample fallback.
    pu = g["by_law"]["pursuit"]
    pr = g["by_law"]["pronav"]
    mean_d = statistics.mean(pu) - statistics.mean(pr)
    n_min = min(len(pu), len(pr))
    if abs(mean_d) < NOISE_FLOOR_M:
        v = f"unpaired Delta={mean_d:+.2f} m < {NOISE_FLOOR_M:g}m noise: tied (n={len(pu)}/{len(pr)})"
    elif n_min < PAIRED_MIN_N:
        v = f"unpaired Delta={mean_d:+.2f} m, n={len(pu)}/{len(pr)}<8: not a claim"
    else:
        v = f"unpaired Delta={mean_d:+.2f} m (n={len(pu)}/{len(pr)}): needs paired seeds"
    return ("unpaired", f"{len(pu)}/{len(pr)}", mean_d, v)


def plot_per_path(path_groups, basename, out_path):
    """Horizontal dot plot: mean miss per (path, law) with +-1 SD whiskers,
    grouped by path, plus the honest per-path verdict text."""
    laws_all = sorted({law for g in path_groups for law in g["by_law"]})
    colors = law_color_map(laws_all)
    n_paths = len(path_groups)
    fig, ax = plt.subplots(figsize=(11, max(3.2, 0.95 * n_paths + 1.6)))
    style_axes(ax)
    ax.grid(True, axis="x", color=GRID, linewidth=0.8)
    ax.grid(False, axis="y")

    yticks, ylabels = [], []
    offset = 0.16
    for pi, g in enumerate(path_groups):
        base_y = n_paths - 1 - pi  # top-to-bottom in sorted order
        yticks.append(base_y)
        ylabels.append(g["label"])
        laws = sorted(g["by_law"])
        for li, law in enumerate(laws):
            vals = g["by_law"][law]
            if not vals:
                continue
            mean_v = statistics.mean(vals)
            sd_v = statistics.stdev(vals) if len(vals) > 1 else 0.0
            y = base_y + (offset if law == "pursuit" else -offset if law == "pronav"
                          else offset * (1 - li))
            ax.errorbar(mean_v, y, xerr=sd_v, fmt=LAW_MARKER.get(law, DEFAULT_MARKER),
                        color=colors[law], markersize=9, elinewidth=2, capsize=4,
                        markeredgecolor=SURFACE, markeredgewidth=1.2,
                        label=law if pi == 0 else None)
            ax.annotate(f"n={len(vals)}", xy=(mean_v, y), xytext=(0, 9 if law == "pursuit" else -13),
                        textcoords="offset points", ha="center", fontsize=7, color=MUTED)
        # verdict text in the reserved right margin
        _, _, _, verdict = path_verdict(g)
        ax.annotate(verdict, xy=(1.02, base_y), xycoords=("axes fraction", "data"),
                    va="center", ha="left", fontsize=7, color=INK2)

    for r_m, name in ((RAM_RADIUS_M, "ram"), (NET_RADIUS_M, "net")):
        ax.axvline(r_m, color=INK2, linestyle=":", linewidth=1.0)
        ax.annotate(f"{name} {r_m:g} m", xy=(r_m, 0.5), xycoords=("data", "axes fraction"),
                    xytext=(-4, 0), textcoords="offset points", rotation=90,
                    ha="right", va="center", fontsize=8, color=INK2)
    ax.set_yticks(yticks)
    ax.set_yticklabels(ylabels, color=INK2)
    ax.set_ylim(-0.7, n_paths - 0.3)
    ax.set_xlabel("Miss distance (m)  --  mean +/- 1 SD per arm", color=INK2)
    ax.set_title("Per-path pursuit vs pro-nav (paired-seed aware; small-n = tied)",
                 color=INK, fontsize=12)
    ax.legend(loc="lower left", frameon=False, fontsize=9, labelcolor=INK2)
    fig.text(0.005, 0.01, basename, fontsize=7.5, color=MUTED)
    # reserve the right ~36% of the figure for the per-path verdict text
    fig.tight_layout(rect=(0, 0.02, 0.64, 1))
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def load_trajectory(flight_path):
    """Read one per-run flight CSV -> (interceptor_xy, target_xy, cpa_idx) using
    the ground-truth position columns. Returns None if unreadable / missing the
    gt columns. gt_* are scoring/viz only (honesty boundary)."""
    if not flight_path or not os.path.isfile(flight_path):
        return None
    try:
        with open(flight_path, newline="") as f:
            frows = list(csv.DictReader(f))
    except OSError:
        return None
    ix, iy, tx, ty, gr = [], [], [], [], []
    for r in frows:
        cx = parse_float(r, "gt_cam_x")
        cy = parse_float(r, "gt_cam_y")
        gx = parse_float(r, "gt_tag_x")
        gy = parse_float(r, "gt_tag_y")
        if math.isnan(cx) or math.isnan(cy) or math.isnan(gx) or math.isnan(gy):
            continue
        ix.append(cx); iy.append(cy); tx.append(gx); ty.append(gy)
        rng = parse_float(r, "gt_range")
        if math.isnan(rng):
            rng = math.hypot(gx - cx, gy - cy)
        gr.append(rng)
    if len(ix) < 2:
        return None
    cpa_idx = min(range(len(gr)), key=lambda i: gr[i])
    # Trim to the approach + a short tail past CPA: post-CPA is BREAKOFF, where
    # a fast target zooms off to tens of metres and would dominate an
    # equal-aspect view. Keeping ~CPA plus a margin shows the engagement
    # geometry and where closest approach happened without the runaway tail.
    end = min(len(ix), cpa_idx + max(8, int(0.15 * len(ix))) + 1)
    ix, iy, tx, ty = ix[:end], iy[:end], tx[:end], ty[:end]
    return (ix, iy), (tx, ty), cpa_idx


def plot_trajectory_overlay(rows, basename, out_path):
    """Small multiples by path: interceptor + target ground-truth paths per run,
    CPA marked. Returns False (and saves nothing) if no flight CSVs are
    readable, so the rest of the analysis still completes."""
    # Group flight CSV paths by path label, tagged with law.
    groups = defaultdict(list)  # label -> list of (sortk, law, traj)
    laws_seen = set()
    for r in rows:
        sortk, label = path_key(r)
        traj = load_trajectory(r.get("flight_csv_path", ""))
        if traj is None:
            continue
        law = (r.get("law", "") or "?").strip()
        laws_seen.add(law)
        groups[label].append((sortk, law, traj))
    if not groups:
        return False
    labels = sorted(groups, key=lambda lb: groups[lb][0][0])
    colors = law_color_map(sorted(laws_seen))

    n = len(labels)
    ncols = min(3, n)
    nrows = math.ceil(n / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.6 * ncols, 4.4 * nrows),
                             squeeze=False)
    for k, label in enumerate(labels):
        ax = axes[k // ncols][k % ncols]
        style_axes(ax)
        law_drawn = set()
        for _, law, traj in groups[label]:
            (ix, iy), (tx, ty), cpa = traj
            # target path -- muted context, drawn thin
            ax.plot(tx, ty, color=MUTED, linewidth=1.0, alpha=0.6,
                    label="target" if "target" not in law_drawn else None)
            ax.plot(tx[0], ty[0], marker=".", color=MUTED, markersize=6)
            # interceptor path -- coloured by law
            lbl = law if law not in law_drawn else None
            ax.plot(ix, iy, color=colors.get(law, INK), linewidth=1.6, alpha=0.9,
                    marker=LAW_MARKER.get(law, DEFAULT_MARKER), markevery=[0],
                    markersize=6, label=lbl)
            # CPA marker (closest point of approach) on the interceptor path
            ax.plot(ix[cpa], iy[cpa], marker="x", color=INK, markersize=8,
                    markeredgewidth=2)
            law_drawn.add(law)
            if "target" not in law_drawn:
                law_drawn.add("target")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(label, color=INK, fontsize=11)
        ax.set_xlabel("East (m)", color=INK2, fontsize=9)
        ax.set_ylabel("North (m)", color=INK2, fontsize=9)
    # blank any unused subplots
    for k in range(n, nrows * ncols):
        axes[k // ncols][k % ncols].axis("off")

    handles, labs = axes[0][0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labs, loc="upper right", frameon=False, fontsize=9,
                   labelcolor=INK2)
    fig.suptitle("Interceptor + target ground-truth paths per run  (x = CPA)  -- gt is "
                 "scoring/viz only, not a guidance input", color=INK, fontsize=12)
    fig.text(0.005, 0.005, basename, fontsize=7.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.01, 1, 0.96))
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return True


def main():
    args = sys.argv[1:]
    no_traj = "--no-traj" in args
    path = find_csv(args)
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
    pk_rows = compute_pk_rows(misses, n_total)
    for r_lethal, k, p, lo, hi in pk_rows:
        print(f"{r_lethal:>10.2f} {k:>6d} {p:>8.3f} {lo:>8.3f} {hi:>8.3f}")
    print("  ^ POOLED across every speed and law -- context only. ADR-0025 forbids")
    print("    headlining this: it can hide a worse number at the high-speed end.")

    # --- Pk vs radius PER ARM (speed x law) -- the actual ADR-0025 headline ---
    arms, arm_speeds, arm_laws = compute_arm_pk(rows)
    print(
        f"\n--- Pk vs lethal radius, PER SPEED x LAW, Wilson 95% CI "
        f"(ADR-0025 headline: never pooled) ---"
    )
    for spd in arm_speeds:
        for law in arm_laws:
            g = arms.get((spd, law))
            if not g or g["n"] == 0:
                continue
            print(f"  {spd:g} m/s, {law} (n={g['n']}):")
            print(f"    {'R_lethal_m':>10} {'kills':>6} {'Pk':>8} {'CI_low':>8} {'CI_high':>8}")
            for r_lethal, k, p, lo, hi in g["pk_rows"]:
                print(f"    {r_lethal:>10.2f} {k:>6d} {p:>8.3f} {lo:>8.3f} {hi:>8.3f}")

    # --- per-path pursuit-vs-pro-nav read (paired-seed aware) ---
    path_groups = per_path_stats(rows)
    print("\n--- Per-path pursuit vs pro-nav (paired-seed aware; honest small-n) ---")
    for g in path_groups:
        mode, ndesc, _, verdict = path_verdict(g)
        parts = []
        for law in sorted(g["by_law"]):
            vals = g["by_law"][law]
            mean_v = statistics.mean(vals)
            sd_v = statistics.stdev(vals) if len(vals) > 1 else 0.0
            parts.append(f"{law} {mean_v:.2f}+-{sd_v:.2f} m (n={len(vals)})")
        print(f"  {g['label']:<16}  " + "  ".join(parts))
        print(f"  {'':<16}  -> [{mode}, {ndesc}] {verdict}")

    os.makedirs(PLOTS_DIR, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    basename = os.path.basename(path)

    arm_path = os.path.join(PLOTS_DIR, f"pk_vs_radius_by_arm_{stamp}.png")
    pk_path = os.path.join(PLOTS_DIR, f"pk_vs_radius_{stamp}.png")
    hist_path = os.path.join(PLOTS_DIR, f"miss_hist_cdf_{stamp}.png")
    perpath_path = os.path.join(PLOTS_DIR, f"per_path_{stamp}.png")
    traj_path = os.path.join(PLOTS_DIR, f"traj_overlay_{stamp}.png")

    if plot_pk_vs_radius_by_arm(arms, arm_speeds, arm_laws, basename, arm_path):
        print(f"\nSaved: {arm_path}  (HEADLINE: Pk vs radius, per speed x law)")
    else:
        print("\nPer-arm Pk-vs-radius plot skipped: no speed/law data found")

    plot_pk_vs_radius(pk_rows, n_total, basename, pk_path)
    print(f"Saved: {pk_path}  (pooled, context only)")

    by_law_valid = defaultdict(list)
    for r, m in zip(rows, misses):
        if not math.isnan(m):
            by_law_valid[(r.get("law", "") or "?").strip()].append(m)
    if by_law_valid:
        plot_miss_hist_cdf(by_law_valid, n_nan, basename, hist_path)
        print(f"Saved: {hist_path}")

    if path_groups:
        plot_per_path(path_groups, basename, perpath_path)
        print(f"Saved: {perpath_path}")

    if no_traj:
        print("Skipped trajectory overlay (--no-traj)")
    else:
        if plot_trajectory_overlay(rows, basename, traj_path):
            print(f"Saved: {traj_path}")
        else:
            print("Trajectory overlay skipped: no readable per-run flight CSVs "
                  "(flight_csv_path missing or gt_* columns absent)")


if __name__ == "__main__":
    main()
