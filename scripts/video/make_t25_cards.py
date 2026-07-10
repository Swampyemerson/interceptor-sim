#!/usr/bin/env python3
"""T25 static cards: shot 0 (title + architecture strip) and shot 8 (results
end card). Offline, no sim -- these two storyboard shots are fully buildable
today (docs/t25_storyboard.md rows 0 and 8).

HONESTY (checked at review):
  * Every end-card NUMBER is recomputed here from the committed validation arm
    logs/mc_t21_trackgate_weave12_r2.csv and ASSERTED byte-equal to the
    ADR-0058 claims (the storyboard's "end-card numbers byte-match" audit
    item, done mechanically). If the CSV and the expected strings ever
    disagree, this script exits 1 rather than render a wrong card.
    The two per-tick counts that cannot be recomputed from the aggregate CSV
    (zero phantom handoffs; 0/155 false terminal detections) are printed with
    an explicit ADR-0058 citation instead.
  * The two disclosure lines are the storyboard's honesty constraints #2 and
    #3, near-verbatim. The jam-denial claim is HELD (ADR-0059 in progress) --
    the card says so; do not strengthen the wording.

USAGE:  .venv/bin/python scripts/video/make_t25_cards.py [--outdir demo_out/t25/cards]
Writes shot0_title.png and shot8_endcard.png (1920x1080).
"""
import argparse
import csv
import os
import statistics
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import cv2

SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(SCRIPTS_DIR)
sys.path.insert(0, SCRIPTS_DIR)
import render_hud as rh  # noqa: E402

ARM_CSV = os.path.join(REPO, "logs", "mc_t21_trackgate_weave12_r2.csv")
W, H = 1920, 1080

# ADR-0058 expected claims (docs/decisions.md; docs/t25_storyboard.md shot 8).
EXPECT = dict(n=16, pooled_pk25=16, pooled_median="2.11", pooled_max="2.48",
              handoff_n=14, handoff_pk25=14, handoff_median="2.03")

DISCLOSURE_2 = ("camera-only terminal (structural latch); "
                "mid-dash jam-denial validation in progress (ADR-0059)")
DISCLOSURE_3 = ("ground-track shots: real stereo pipeline (validated straight-line, ADR-0053); "
                "flown cue: calibrated mock (maneuvering stereo validation pending, T20/T21)")


def arm_stats():
    with open(ARM_CSV, newline="") as f:
        rows = list(csv.DictReader(f))
    miss = [float(r["miss_m"]) for r in rows]
    ho = [float(r["miss_m"]) for r in rows
          if r.get("handoff") == "1" and (r.get("handoff_range_m") or "").strip()
          and float(r["handoff_range_m"]) > 0]
    got = dict(
        n=len(rows),
        pooled_pk25=sum(m <= 2.5 for m in miss),
        pooled_median=f"{statistics.median(miss):.2f}",
        pooled_max=f"{max(miss):.2f}",
        handoff_n=len(ho),
        handoff_pk25=sum(m <= 2.5 for m in ho),
        handoff_median=f"{statistics.median(ho):.2f}",
    )
    bad = {k: (got[k], EXPECT[k]) for k in EXPECT if got[k] != EXPECT[k]}
    if bad:
        print(f"[t25_cards] FAIL: recomputed r2 stats disagree with ADR-0058: {bad}",
              file=sys.stderr)
        sys.exit(1)
    print(f"[t25_cards] r2 stats recomputed from {os.path.basename(ARM_CSV)} and "
          f"byte-match ADR-0058: {got}")
    return got


def _canvas():
    fig = plt.figure(figsize=(W / rh.FIG_DPI, H / rh.FIG_DPI), dpi=rh.FIG_DPI)
    fig.patch.set_facecolor(rh.COLOR_BG)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(rh.COLOR_BG)
    return fig, ax


def _save(fig, path):
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())
    plt.close(fig)
    if not cv2.imwrite(path, cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)):
        raise RuntimeError(f"t25_cards: imwrite failed for {path}")
    print(f"[t25_cards] wrote {path}")


def _box(ax, x, y, w, h, text, color, fs=13):
    ax.add_patch(plt.Rectangle((x, y), w, h, facecolor=rh.COLOR_BG, alpha=0.9,
                               edgecolor=color, linewidth=1.8, zorder=2))
    ax.text(x + w / 2, y + h / 2, text, color=color, fontsize=fs, family=rh.MONO,
            ha="center", va="center", weight="bold", zorder=3, linespacing=1.5)


def _arrow(ax, x0, x1, y, color=rh.COLOR_GRAY):
    ax.annotate("", xy=(x1, y), xytext=(x0, y),
                arrowprops=dict(color=color, width=1.2, headwidth=7, headlength=8))


def title_card(path):
    fig, ax = _canvas()
    ax.text(0.5, 0.80, "COUNTER-UAS  INTERCEPTOR", color=rh.COLOR_GREEN,
            fontsize=34, family=rh.MONO, ha="center", weight="bold")
    ax.text(0.5, 0.715, "GROUND STEREO CUE  →  DASH  →  CAMERA-ONLY TERMINAL",
            color=rh.COLOR_WHITE, fontsize=20, family=rh.MONO, ha="center", weight="bold")
    ax.text(0.5, 0.645,
            "PX4 SITL + Gazebo Harmonic  --  simulation only; every readout traces to a logged CSV column",
            color=rh.COLOR_GT_REF, fontsize=12.5, family=rh.MONO, ha="center")

    # mini architecture strip (storyboard shot 0)
    bw, bh, y = 0.155, 0.135, 0.36
    xs = [0.045, 0.245, 0.445, 0.645, 0.83]
    _box(ax, xs[0], y, bw, bh, "GROUND STEREO RIG\n2 cams, NN detect", rh.COLOR_AMBER, 11)
    _box(ax, xs[1], y, bw, bh, "TRIANGULATED TRACK\n10 Hz cue msg", rh.COLOR_AMBER, 11)
    _box(ax, xs[2], y, bw, bh, "INTERCEPTOR DASH\n16 m/s, pro-nav", rh.COLOR_WHITE, 11)
    _box(ax, xs[3], y, bw, bh, "HANDOFF\nlink closed (one-way)", rh.COLOR_GREEN, 11)
    _box(ax, xs[4], y, 0.125, bh, "CAMERA-ONLY\nNN+CSRT terminal", rh.COLOR_GREEN, 11)
    for a, b in zip(xs[:-1], xs[1:]):
        _arrow(ax, a + bw + 0.004, b - 0.006, y + bh / 2)

    ax.text(0.5, 0.24, "target: 12 m/s WEAVING drone  --  fastest regime with a full validation arm behind it (ADR-0058)",
            color=rh.COLOR_GRAY, fontsize=12, family=rh.MONO, ha="center")
    _save(fig, path)


def end_card(path, s):
    fig, ax = _canvas()
    ax.text(0.5, 0.875, "RESULTS  --  VALIDATED ARM, NOT THE HERO FLIGHT ALONE",
            color=rh.COLOR_WHITE, fontsize=19, family=rh.MONO, ha="center", weight="bold")
    ax.text(0.5, 0.80,
            f"12 m/s weaving target  --  n={s['n']} paired-seed validation arm (ADR-0058)",
            color=rh.COLOR_GRAY, fontsize=14, family=rh.MONO, ha="center")

    lines = [
        (f"pooled Pk@2.5 m      {s['pooled_pk25']}/{s['n']}   "
         f"(median {s['pooled_median']} m, max {s['pooled_max']} m)", rh.COLOR_GREEN),
        (f"camera-only terminal {s['handoff_pk25']}/{s['handoff_n']}   "
         f"(post-handoff, median {s['handoff_median']} m)", rh.COLOR_GREEN),
        ("zero phantom handoffs  --  0/155 false terminal detections (ADR-0058)",
         rh.COLOR_WHITE),
    ]
    y = 0.665
    for text, col in lines:
        ax.text(0.5, y, text, color=col, fontsize=17, family=rh.MONO,
                ha="center", weight="bold")
        y -= 0.075
    ax.text(0.5, y + 0.01,
            "numbers recomputed from logs/mc_t21_trackgate_weave12_r2.csv at card-build time",
            color=rh.COLOR_GT_REF, fontsize=10.5, family=rh.MONO, ha="center")

    # the two non-negotiable disclosure lines (honesty constraints #2 and #3)
    ax.add_patch(plt.Rectangle((0.06, 0.13), 0.88, 0.16, facecolor="none",
                               edgecolor=rh.COLOR_PANEL_LINE, linewidth=1.2))
    ax.text(0.5, 0.245, DISCLOSURE_2, color=rh.COLOR_AMBER, fontsize=12.5,
            family=rh.MONO, ha="center")
    ax.text(0.5, 0.185, DISCLOSURE_3, color=rh.COLOR_AMBER, fontsize=11.0,
            family=rh.MONO, ha="center")
    ax.text(0.985, 0.025, "miss = closest approach vs a CHOSEN lethal radius -- no collision volume in sim (ADR-0014/0025)",
            color=rh.COLOR_GT_REF, fontsize=9.5, family=rh.MONO, ha="right")
    _save(fig, path)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--outdir", default=os.path.join(REPO, "demo_out", "t25", "cards"))
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    s = arm_stats()
    title_card(os.path.join(args.outdir, "shot0_title.png"))
    end_card(os.path.join(args.outdir, "shot8_endcard.png"), s)


if __name__ == "__main__":
    main()
