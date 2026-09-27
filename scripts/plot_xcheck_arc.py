#!/usr/bin/env python3
"""Render the Gazebo chase cross-check campaign arc (README hero figure).

Data provenance: the per-flight closest-approach (CPA) values are parsed
straight from the registered RESULT lines ("CPAs sorted (m): ...") of the five
pre-registration docs, docs/xcheck_gazebo_pursuit_prereg{,2,3,4,5}.md, so the
figure cannot drift from the written record. Each campaign is n=8 fresh-boot
PX4 SITL + Gazebo flights, camera-in-the-loop, scored against ground truth
(scoring only). Same registered bar throughout.

Output: docs/images/xcheck_campaign_arc.png
"""
import os
import re
import statistics

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(ROOT, "docs")
OUT = os.path.join(DOCS, "images", "xcheck_campaign_arc.png")

BLUE, ORANGE = "#2a78d6", "#eb6834"
SURFACE, INK, MUTED, GRIDC = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
RAM = 0.35  # m, ratified contact radius (ADR-0084)

# (prereg suffix, x-label, what changed) -- ADRs per docs/decisions.md
CAMPAIGNS = [
    ("", "#1", "first port\n(ADR-0105)"),
    ("2", "#2", "range-bias +\ncadence fixes\n(ADR-0113)"),
    ("3", "#3", "brake cap\n(ADR-0116)"),
    ("4", "#4", "own-velocity\nwiring fix\n(ADR-0117)"),
    ("5", "#5", "fix + brake\n(ADR-0120)"),
]


def cpas(suffix):
    path = os.path.join(DOCS, f"xcheck_gazebo_pursuit_prereg{suffix}.md")
    text = open(path, encoding="utf-8").read()
    m = re.search(r"CPAs,? sorted \(m\):(.*?)\.\s*(?:\*\*)?Median", text, re.S)
    if not m:
        raise SystemExit(f"no registered CPA line in {path}")
    vals = [float(v) for v in re.findall(r"\d+\.\d+", m.group(1))]
    if len(vals) != 8:  # fail closed: every campaign is registered at n=8
        raise SystemExit(f"{path}: expected 8 CPAs, parsed {len(vals)}")
    return vals


def main():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
        "font.family": "DejaVu Sans", "text.color": INK,
        "axes.edgecolor": GRIDC, "axes.labelcolor": MUTED,
        "xtick.color": MUTED, "ytick.color": MUTED,
        "axes.grid": True, "grid.color": GRIDC, "grid.linewidth": 0.8,
        "axes.axisbelow": True,
    })
    data = [cpas(s) for s, _, _ in CAMPAIGNS]
    meds = [statistics.median(d) for d in data]

    fig, ax = plt.subplots(figsize=(12, 6.75), dpi=160)
    fig.subplots_adjust(left=0.08, right=0.97, top=0.80, bottom=0.22)
    offs = [-0.21, -0.15, -0.09, -0.03, 0.03, 0.09, 0.15, 0.21]
    for i, d in enumerate(data):
        for j, v in enumerate(d):
            hit = v <= RAM
            ax.scatter(i + offs[j], v, s=70, zorder=3,
                       color=ORANGE if hit else BLUE,
                       edgecolor="white", linewidth=0.8)
        ax.plot([i - 0.3, i + 0.3], [meds[i]] * 2, color=INK, lw=2.2, zorder=4)
        n_in = sum(v <= RAM for v in d)
        ax.text(i + 0.33, meds[i], f"median {meds[i]:.2f} m\n{n_in}/8 touch",
                va="center", ha="left", fontsize=10, color=INK)
    ax.axhline(RAM, color=MUTED, ls="--", lw=1.4, zorder=2)
    ax.text(-0.45, RAM * 1.07, "0.35 m contact radius (airframes touch)",
            fontsize=10, color=MUTED, va="bottom")
    ax.set_yscale("log")
    ax.set_ylim(0.08, 3.2)
    ax.set_yticks([0.1, 0.2, 0.35, 0.5, 1, 2, 3])
    ax.set_yticklabels(["0.1", "0.2", "0.35", "0.5", "1", "2", "3"])
    ax.set_xlim(-0.55, len(data) - 0.1)
    ax.set_xticks(range(len(data)))
    ax.set_xticklabels([f"{lab}\n{what}" for _, lab, what in CAMPAIGNS],
                       fontsize=10)
    ax.set_ylabel("closest approach to target (m, log)")
    ax.grid(axis="x", visible=False)
    fig.text(0.08, 0.93, "Camera-guided chase in full-physics sim: five campaigns, one bar",
             fontsize=17, weight="bold")
    fig.text(0.08, 0.875,
             "Each dot is one PX4 SITL + Gazebo flight (n=8 per campaign); "
             "orange = inside the contact radius. Every fix pre-registered before flying.",
             fontsize=11, color=MUTED)
    fig.text(0.01, 0.015,
             "Simulation, 9 m/s crossing target, AprilTag seeker, perfect launch cue, "
             "no wind - not a field result · source: docs/xcheck_gazebo_pursuit_prereg{,2..5}.md "
             "· 2026-09-24",
             fontsize=8.5, color=MUTED)
    fig.savefig(OUT)
    print(f"wrote {os.path.relpath(OUT, ROOT)}; medians "
          + " -> ".join(f"{m:.3f}" for m in meds))


if __name__ == "__main__":
    main()
