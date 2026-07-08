#!/usr/bin/env python3
"""Portfolio plots for the markerless seeker A/B + ground-rig sweep (ADR-0038/0039).

Reads the arm CSVs from logs/ and writes three figures to docs/images/:
  1. seeker_ab_pk_by_arm.png     -- Pk-vs-radius, ONE LINE PER ARM (never pooled,
                                    ADR-0025), ram 0.5 m / net 1.5 m reference lines.
  2. seeker_ab_paired_scatter.png -- apriltag vs markerless miss per shared cue_seed
                                    (paired), log axes, the 2 late-acquisition flybys
                                    annotated -- the whole "cost of killing the tag" story.
  3. seeker_c_sweep_bars.png     -- clean-rate + Pk@2.5 m + dash-aborts per C-sweep arm.

Design: colorblind-safe (Wong palette), direct-labeled where it beats a legend,
status colors reserved for pass/fail semantics only. No sim, no gt -- reads CSVs.
"""
from __future__ import annotations
import csv, math, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
IMG = os.path.join(REPO, "docs", "images")
os.makedirs(IMG, exist_ok=True)

# Wong colorblind-safe palette
BLUE, ORANGE, GREEN, RED, PURPLE, BROWN, GREY = (
    "#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#8C6D31", "#999999")

def load(path):
    try:
        with open(path) as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []

def miss(r):
    try: return float(r["miss_m"])
    except: return float("nan")

ARMS = [
    ("AprilTag (baseline)",         "logs/mc_ab_apriltag.csv",                        BLUE),
    ("Markerless (default)",        "logs/mc_ab_markerless.csv",                      ORANGE),
    ("Markerless late-handoff",     "logs/mc_grsweep_armB_markerless_lateHandoff.csv", GREEN),
    ("Markerless wide ceiling 20m", "logs/mc_grsweep_armC_markerless_wideCeiling.csv", PURPLE),
    ("Markerless tight ceiling 2.5m","logs/mc_grsweep_armD_markerless_tightCeiling.csv", BROWN),
    ("AprilTag tight ceiling 2.5m", "logs/mc_grsweep_armE_apriltag_tightCeiling.csv",  GREY),
]

def pk_curve(rows, radii):
    n = len(rows); ms = [miss(r) for r in rows]
    return [sum(1 for m in ms if not math.isnan(m) and m <= x) / n for x in radii]

# ---- Figure 1: Pk-vs-radius, one line per arm ----------------------------
radii = [i * 0.1 for i in range(1, 61)]  # 0.1 .. 6.0 m
fig, ax = plt.subplots(figsize=(8.2, 5.4))
for label, path, color in ARMS:
    rows = load(path)
    if not rows: continue
    y = pk_curve(rows, radii)
    ax.plot(radii, y, color=color, lw=2.2, label=label)
    ax.annotate(f" {label}", (radii[-1], y[-1]), color=color, fontsize=8,
                va="center", ha="left")
ax.axvline(0.5, color=RED, ls=":", lw=1, alpha=0.7)
ax.axvline(1.5, color="#444", ls=":", lw=1, alpha=0.7)
ax.text(0.5, 1.02, "ram 0.5 m", color=RED, fontsize=8, ha="center")
ax.text(1.5, 1.02, "net 1.5 m", color="#444", fontsize=8, ha="center")
ax.set_xlabel("lethal radius (m)"); ax.set_ylabel("Pk = fraction of flights within radius")
ax.set_title("Pk vs radius by arm — markerless A/B + ground-rig sweep (n=8/arm, ADR-0038/0039)")
ax.set_xlim(0, 7.6); ax.set_ylim(0, 1.08); ax.grid(alpha=0.25)
fig.tight_layout(); fig.savefig(os.path.join(IMG, "seeker_ab_pk_by_arm.png"), dpi=130)
plt.close(fig)

# ---- Figure 2: paired scatter apriltag vs markerless ---------------------
ap = {r["cue_seed"]: r for r in load("logs/mc_ab_apriltag.csv")}
mk = {r["cue_seed"]: r for r in load("logs/mc_ab_markerless.csv")}
common = [s for s in mk if s in ap]
fig, ax = plt.subplots(figsize=(6.4, 6.0))
for s in common:
    x, y = miss(ap[s]), miss(mk[s])
    clean = mk[s].get("clean") == "1"
    ax.scatter(x, y, s=60, color=(ORANGE if clean else RED),
               edgecolor="k", lw=0.5, zorder=3)
    if y > 4:  # annotate the flybys
        fd = mk[s].get("first_det_range_m", "?")
        ax.annotate(f"late acq {float(fd):.1f} m", (x, y), fontsize=8, color=RED,
                    xytext=(6, -2), textcoords="offset points")
lim = [0.4, 12]
ax.plot(lim, lim, color=GREY, ls="--", lw=1, label="equal miss")
ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlim(*lim); ax.set_ylim(*lim)
ax.set_xlabel("AprilTag miss (m)"); ax.set_ylabel("Markerless miss (m)")
ax.set_title("Paired A/B by cue-seed — markerless matches the tag\nexcept on late-acquisition flybys (red)")
ax.scatter([], [], color=ORANGE, edgecolor="k", label="markerless clean")
ax.scatter([], [], color=RED, edgecolor="k", label="markerless flyby")
ax.legend(loc="lower right", fontsize=8); ax.grid(alpha=0.25, which="both")
fig.tight_layout(); fig.savefig(os.path.join(IMG, "seeker_ab_paired_scatter.png"), dpi=130)
plt.close(fig)

# ---- Figure 3: C-sweep bars (clean-rate, Pk@2.5, dash-aborts) -------------
fig, ax = plt.subplots(figsize=(9.0, 5.0))
names, cleans, pk25, aborts = [], [], [], []
for label, path, _ in ARMS:
    rows = load(path)
    if not rows: continue
    n = len(rows)
    names.append(label.replace(" ", "\n", 1))
    cleans.append(sum(1 for r in rows if r.get("clean") == "1") / n)
    pk25.append(sum(1 for r in rows if not math.isnan(miss(r)) and miss(r) <= 2.5) / n)
    aborts.append(sum(1 for r in rows if r.get("handoff") == "0"))
xs = range(len(names)); w = 0.38
ax.bar([x - w/2 for x in xs], cleans, w, color=GREEN, label="clean-rate")
ax.bar([x + w/2 for x in xs], pk25, w, color=BLUE, label="Pk@2.5 m")
for x, a in zip(xs, aborts):
    if a: ax.text(x, 0.02, f"{a} dash-abort", rotation=90, fontsize=7, color=RED, va="bottom", ha="center")
ax.set_xticks(list(xs)); ax.set_xticklabels(names, fontsize=7)
ax.set_ylabel("fraction of 8 flights"); ax.set_ylim(0, 1.05)
ax.set_title("Clean-rate & Pk@2.5 m per arm — handoff-timing levers don't recover markerless (ADR-0039)")
ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.2, axis="y")
fig.tight_layout(); fig.savefig(os.path.join(IMG, "seeker_c_sweep_bars.png"), dpi=130)
plt.close(fig)

print("wrote docs/images/seeker_ab_pk_by_arm.png, seeker_ab_paired_scatter.png, seeker_c_sweep_bars.png")
