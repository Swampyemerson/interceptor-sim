#!/usr/bin/env python3
"""Render the LinkedIn media figures from the 2026-09-22 isim parity re-run.

Data provenance (all simulation, isim flight-code-in-the-loop):
  logs/linkedin_media_20260922/{native,port}_{aim,alt}.csv
    -- fresh n=50/cell re-run of the registered parity grid
       (isim/specs/parity_trace_2026-09-22.md), rear-aspect tag,
       vehicle model fitted to 345 logged Gazebo flights.
  logs/parity_trace_2026-09-22/seed2_port.csv
    -- post-fix per-tick trace of one flight-code engagement.

Palette: dataviz reference instance (series-1 blue / series-2 orange),
light surface. One hue per arm, fixed assignment.
"""
import csv, os, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = os.path.join(ROOT, "logs", "linkedin_media_20260922")
OUT = os.path.join(ROOT, "docs", "images", "linkedin")
os.makedirs(OUT, exist_ok=True)

BLUE, ORANGE = "#2a78d6", "#eb6834"
SURFACE, INK, MUTED, GRIDC = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
RAM = 0.35  # m, binary-kill ram radius (ADR-0025 lineage)

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "font.family": "DejaVu Sans", "text.color": INK,
    "axes.edgecolor": GRIDC, "axes.labelcolor": MUTED,
    "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRIDC, "grid.linewidth": 0.8,
    "axes.axisbelow": True, "svg.fonttype": "none",
})

def load(path):
    with open(path) as f:
        return [r for r in csv.DictReader(f) if not r["target_speed_ms"].startswith("#")]

def cells(arm):
    """Return dict cell_name -> list of miss_m for the four registered cells."""
    aim = load(os.path.join(D, f"{arm}_aim.csv"))
    alt = load(os.path.join(D, f"{arm}_alt.csv"))
    out = {}
    for label, val in [("nominal", 0.0), ("aim error 10°", 10.0), ("aim error 20°", 20.0)]:
        out[label] = [float(r["miss_m"]) for r in aim if float(r["aim_error_deg"]) == val]
    out["target +2 m alt"] = [float(r["miss_m"]) for r in alt if float(r["target_alt_offset_m"]) == 2.0]
    for k, v in out.items():
        assert len(v) == 50, (k, len(v))  # no vacuous verdicts
    return out

native, port = cells("native"), cells("port")
names = list(native.keys())
prov = ("Simulation (isim, flight code in the loop) · n = 50 seeded runs per condition · "
        "rear-aspect 0.30 m AprilTag · vehicle model fitted to 345 logged flights · 2026-09-22")

# ---- Fig 1: robustness grid, grouped bars --------------------------------
fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=200)
x = np.arange(len(names)); w = 0.34
def pct(v): return 100.0 * np.mean(np.array(v) <= RAM)
pn = [pct(native[k]) for k in names]; pp = [pct(port[k]) for k in names]
b1 = ax.bar(x - w/2 - 0.01, pn, w, color=BLUE, label="Guidance prototype", zorder=3)
b2 = ax.bar(x + w/2 + 0.01, pp, w, color=ORANGE, label="Flight code (ported, on-vehicle stack)", zorder=3)
for bars, vals, misses in [(b1, pn, native), (b2, pp, port)]:
    for rect, v, k in zip(bars, vals, names):
        med = np.median(misses[k])
        ax.annotate(f"{v:.0f}%", (rect.get_x() + rect.get_width()/2, v), xytext=(0, 4),
                    textcoords="offset points", ha="center", fontsize=11, color=INK, fontweight="bold")
        ax.annotate(f"med {med:.2f} m", (rect.get_x() + rect.get_width()/2, 2),
                    xytext=(0, 2), textcoords="offset points", ha="center",
                    fontsize=7.5, color="#ffffff", rotation=90, va="bottom")
ax.set_xticks(x); ax.set_xticklabels(names, fontsize=11, color=INK)
ax.set_ylim(0, 112); ax.set_yticks([0, 25, 50, 75, 100])
ax.set_ylabel("runs ending inside 0.35 m intercept radius (%)")
ax.xaxis.grid(False)
ax.set_title("Camera-only terminal chase: robustness to launch-cue error",
             fontsize=15, color=INK, pad=44, loc="left", fontweight="bold")
ax.text(0, 1.075, "Monte-Carlo miss distance vs a 9 m/s crossing target; median miss printed inside each bar",
        transform=ax.transAxes, fontsize=10.5, color=MUTED)
ax.legend(loc="upper left", bbox_to_anchor=(0.0, 1.065), frameon=False, fontsize=10, ncols=2)
fig.text(0.01, 0.012, prov, fontsize=7.5, color=MUTED)
fig.tight_layout(rect=(0, 0.03, 1, 1))
fig.savefig(os.path.join(OUT, "fig1_chase_robustness_grid.png"))
plt.close(fig)

# ---- Fig 2: miss-distance CDF, nominal cell ------------------------------
fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=200)
for vals, c, lab in [(native["nominal"], BLUE, "Guidance prototype"),
                     (port["nominal"], ORANGE, "Flight code (ported)")]:
    v = np.sort(vals); y = np.arange(1, len(v) + 1) / len(v) * 100
    ax.step(v, y, where="post", color=c, lw=2, label=lab, zorder=3)
ax.axvline(RAM, color=MUTED, lw=1.2, ls="--", zorder=2)
ax.annotate("0.35 m intercept radius", (RAM, 6), xytext=(-8, 0), textcoords="offset points",
            fontsize=9.5, color=MUTED, ha="right")
ax.set_xscale("log"); ax.set_xlim(0.02, 0.6)
ax.set_xticks([0.02, 0.05, 0.1, 0.2, 0.35, 0.5])
ax.set_xticklabels(["0.02", "0.05", "0.1", "0.2", "0.35", "0.5"])
ax.set_ylim(0, 102)
ax.set_xlabel("closest approach to target (m, log scale)")
ax.set_ylabel("cumulative share of runs (%)")
ax.set_title("Miss-distance distribution, nominal launch cue (50 seeded runs per arm)",
             fontsize=15, color=INK, pad=30, loc="left", fontweight="bold")
ax.text(0, 1.04, "Every run in both arms ends inside the intercept radius; flight-code median 0.068 m",
        transform=ax.transAxes, fontsize=10.5, color=MUTED)
ax.legend(loc="lower right", frameon=False, fontsize=10)
fig.text(0.01, 0.012, prov, fontsize=7.5, color=MUTED)
fig.tight_layout(rect=(0, 0.03, 1, 1))
fig.savefig(os.path.join(OUT, "fig2_miss_cdf_nominal.png"))
plt.close(fig)

# ---- Fig 3: engagement anatomy from the post-fix trace -------------------
with open(os.path.join(ROOT, "logs", "parity_trace_2026-09-22", "seed2_port.csv")) as f:
    tr = list(csv.DictReader(f))
t = np.array([float(r["t"]) for r in tr])
rng = np.array([float(r["range_true_m"]) for r in tr])
est = np.array([float(r["est_err_m"]) if r["est_err_m"] != "nan" else np.nan for r in tr])
det = np.array([int(r["det"]) for r in tr]) > 0
cpa_i = int(np.nanargmin(rng)); cut = min(len(t), cpa_i + 60)
t, rng, est, det = t[:cut], rng[:cut], est[:cut], det[:cut]

fig, (a1, a2) = plt.subplots(2, 1, figsize=(9.6, 6.4), dpi=200, sharex=True,
                             height_ratios=[2.1, 1])
a1.plot(t, rng, color=BLUE, lw=2, zorder=3)
a1.scatter(t[det], rng[det], s=9, color=ORANGE, zorder=4, label="tag detection consumed")
a1.axhline(RAM, color=MUTED, lw=1.2, ls="--")
a1.annotate("0.35 m intercept radius", (t[2], RAM), xytext=(0, 5), textcoords="offset points",
            fontsize=9, color=MUTED)
a1.annotate(f"closest approach {np.nanmin(rng):.3f} m", (t[cpa_i], rng[cpa_i]),
            xytext=(-10, 14), textcoords="offset points", ha="right", fontsize=10,
            color=INK, fontweight="bold")
a1.set_yscale("log"); a1.set_ylim(0.05, 25)
a1.set_yticks([0.1, 0.35, 1, 3, 10]); a1.set_yticklabels(["0.1", "0.35", "1", "3", "10"])
a1.set_ylabel("range to target (m, log)")
a1.legend(loc="upper right", frameon=False, fontsize=9.5)
a1.set_title("Anatomy of one flight-code engagement (17 m tail-chase to contact)",
             fontsize=15, color=INK, pad=28, loc="left", fontweight="bold")
a1.text(0, 1.07, "Camera detections drive a Kalman track of the target; guidance closes on the estimate, never on ground truth",
        transform=a1.transAxes, fontsize=10, color=MUTED)
a2.plot(t, est, color=BLUE, lw=1.6, zorder=3)
a2.set_ylabel("estimator error (m)")
a2.set_xlabel("engagement time (s)")
a2.set_ylim(0, max(1.0, np.nanmax(est) * 1.1))
fig.text(0.01, 0.012,
         "Simulation trace, seed 2 of the registered parity set · truth used for scoring only · "
         "logs/parity_trace_2026-09-22/seed2_port.csv · 2026-09-22", fontsize=7.5, color=MUTED)
fig.tight_layout(rect=(0, 0.03, 1, 1))
fig.savefig(os.path.join(OUT, "fig3_engagement_anatomy.png"))
plt.close(fig)

print("wrote 3 figures to", OUT)
