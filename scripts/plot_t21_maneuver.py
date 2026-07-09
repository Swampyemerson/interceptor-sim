#!/usr/bin/env python3
"""T21 maneuvering-arms comparison plot (ADR-0056): at 12 m/s + maneuver, the
markerless seeker's terminal miss blows up while the clean AprilTag seeker
holds — isolating a PERCEPTION (bearing-quality) limit, not a kinematic one.

Reads the three mc_batch arm CSVs, draws a per-flight miss strip plot grouped
by arm, coloured clean (<=2.5 m Pk radius, ADR-0025) vs dirty, with the arm
clean-rate + median annotated. Saves docs/images/t21_maneuver_miss.png.
Run: .venv/bin/python scripts/plot_t21_maneuver.py
"""
import csv, os, statistics
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PK_R = 2.5  # ratified Pk radius (ADR-0025/0036)
ARMS = [
    ("logs/mc_t21_markerless_weave12.csv", "markerless\nweave"),
    ("logs/mc_t21_markerless_jink12.csv",  "markerless\njink"),
    ("logs/mc_t21_apriltag_weave12.csv",   "AprilTag\nweave (ctrl)"),
]
# colourblind-safe: clean = blue, dirty = orange
C_CLEAN, C_DIRTY = "#0072B2", "#E69F00"

fig, ax = plt.subplots(figsize=(7.2, 4.6))
xticks, xlabels = [], []
for i, (rel, label) in enumerate(ARMS):
    rows = list(csv.DictReader(open(os.path.join(REPO, rel))))
    miss = [float(r["miss_m"]) for r in rows]
    clean = sum(m <= PK_R for m in miss)
    # jittered strip
    import math
    for j, m in enumerate(miss):
        jx = i + ((j % 5) - 2) * 0.055
        ax.scatter(jx, m, s=64, color=(C_CLEAN if m <= PK_R else C_DIRTY),
                   edgecolor="white", linewidth=0.8, zorder=3)
    med = statistics.median(miss)
    ax.plot([i - 0.22, i + 0.22], [med, med], color="black", lw=2, zorder=4)
    ax.annotate(f"{clean}/{len(miss)} clean\nmed {med:.1f} m",
                (i, max(miss) + 0.5), ha="center", va="bottom", fontsize=9,
                fontweight="bold")
    xticks.append(i); xlabels.append(label)

ax.axhline(PK_R, color="#888", ls="--", lw=1.2, zorder=1)
ax.annotate(f"{PK_R} m Pk radius", (2.35, PK_R + 0.12), ha="right", fontsize=8,
            color="#555")
ax.set_xticks(xticks); ax.set_xticklabels(xlabels)
ax.set_ylabel("terminal miss distance (m)")
ax.set_ylim(0, 11)
ax.set_title("T21: 12 m/s maneuvering intercept — perception, not kinematics\n"
             "clean AprilTag holds; markerless bearing-quality fails under sparse "
             "maneuver detection", fontsize=10.5)
# legend
from matplotlib.lines import Line2D
ax.legend(handles=[
    Line2D([0], [0], marker="o", color="w", markerfacecolor=C_CLEAN, markersize=9,
           label=f"clean (miss ≤ {PK_R} m)"),
    Line2D([0], [0], marker="o", color="w", markerfacecolor=C_DIRTY, markersize=9,
           label=f"dirty (miss > {PK_R} m)"),
    Line2D([0], [0], color="black", lw=2, label="arm median"),
], loc="upper left", fontsize=8, framealpha=0.9)
ax.grid(axis="y", alpha=0.25)
fig.tight_layout()
out = os.path.join(REPO, "docs/images/t21_maneuver_miss.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=130)
print(f"wrote {out}")
