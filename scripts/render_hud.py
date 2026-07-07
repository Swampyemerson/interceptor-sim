#!/usr/bin/env python3
"""Offline, deterministic HUD-frame generator (docs/demo_plan.md, Part 1).

WHAT: reads a per-tick flight CSV written by scripts/m4_intercept.py (or
scripts/m3_static_intercept.py -- same style, a strict subset of columns)
and renders one RGBA PNG "glass-cockpit" HUD frame per CSV row (or, with
--fps, per a fixed time-grid sample of the flight). This is NOT rendered
in-sim -- it is a pure function of logged columns, so every number on
screen traces to the CSV. That is the whole point of "data-driven, not
screen-only" (docs/demo_plan.md Part 1: composite this panel beside a
Gazebo GUI flight capture with ffmpeg, later/gated -- see scripts/
compose_demo.sh, not built yet).

WHY THIS EXISTS NOW, UNGATED: docs/demo_plan.md's TODO list marks this
tool "safe to build against validated numbers/tooling" -- it reads ANY
per-tick CSV and draws whatever is in it; it has no opinion about which
guidance law produced the numbers, so it does not need Tier-2/M5 to be
finalized to be built and tested.

HONESTY / ROBUSTNESS RULES (read before touching -- these are the
project's non-negotiables, not style preferences):
  - Blank CSV cells (e.g. r_hat_m/lambda_dot_deg_s during TAKEOFF/CUE_WAIT,
    before the tracker has anything to report) are parsed as NaN, never
    0 -- see parse_float(). A 0-fill would draw a false "0 m range" or a
    false "LOCK" during a phase where the interceptor has no track at
    all. NaN readouts render as "--" in a dim/dark color, not a number.
  - The SENSOR/mode lamp ("EXTERNAL CUE (mocked)" -> "CAMERA-ONLY") is
    keyed off `ext_fresh`, NOT off a "HANDOFF" phase value -- there is no
    such phase (m4_intercept.py's phases are TAKEOFF / CUE_WAIT / DASH /
    ENGAGE / BREAKOFF, plus ACQUIRE/BENCH_* for non-S2 runs; grep
    m4_intercept.py for `phase = ` to confirm). Specifically: the flip
    tick is the LAST row where ext_fresh == 1. That is safe to treat as a
    one-way latch because m4_intercept.py's HANDOFF transition closes and
    nulls the CueReader (ADR-0010 #5) -- ext_fresh is written from a
    per-tick local that is structurally impossible to become 1 again
    after that (see compute_handoff_idx()'s docstring for the source
    trace). A run that never received any cue (no --handoff, or a
    --handoff run that aborted before its first cue) has no such row --
    those flights render CAMERA-ONLY for their entire length, which is
    the correct reading (no external cue was ever used).
  - The mini-map's lethal-radius ring / closest-approach marker are keyed
    off `gt_range` (ground truth). That is the legitimate SCORING use of
    gt_* per CLAUDE.md's honesty boundary: gt_* never feeds guidance, it
    only tells the story of what actually happened, same as a range
    safety officer's radar would. It is drawn visually distinct (dashed,
    labeled "GT (reference only)") and never appears before the tick it
    actually happens on -- no foreknowledge.
  - The APRILTAG LOCK lamp is keyed off `detected` (0/1), parsed through
    parse_flag01() so a genuinely blank cell renders as "--" (pre-
    acquisition / unknown), never silently coerced to 0/"SEARCHING". This
    is the demo's core honesty disclosure: the target model is dressed as
    an FPV racing drone for the video, but the lock is a classical/
    fiducial AprilTag detection (GOALS.md's "same architecture, one
    honest simplification"), not an EO/IR seeker actually recognizing an
    FPV airframe. The lamp text says "APRILTAG", not "TARGET LOCK", so
    that distinction reads on-screen even out of context.

WIDGETS (each -> the exact CSV column driving it):
  PHASE banner              <- phase
  SENSOR / mode lamp        <- ext_fresh (see compute_handoff_idx())
  APRILTAG LOCK lamp        <- detected (see parse_flag01())
  INTERCEPT SOLUTION        <- tgt_n_hat / tgt_e_hat presence
  CLOSING SPEED             <- vc_m_s
  RANGE                     <- r_hat_m
  LOS RATE                  <- lambda_dot_deg_s
  LAW                       <- law
  Top-down East-North map   <- gt_cam_x/y (own ship, reference position),
                               psi_deg (own heading), tgt_n_hat/tgt_e_hat
                               (the guidance-legitimate target track),
                               gt_tag_x/y (target ground truth, reference
                               only), gt_range (closest-approach ring).

CLI:
    .venv/bin/python scripts/render_hud.py <flight_csv> --out <dir> \\
        [--fps N] [--radius M] [--transparent]

--transparent: writes frames with the empty canvas at alpha=0 (only the
drawn HUD panels/text/lines are opaque or semi-opaque), for compositing
as an overlay LAYER on top of a Gazebo render in a video editor (e.g.
Premiere Pro) instead of the default side-by-side sidebar look. The
lamp/readout panels keep a semi-opaque dark backing plate for legibility
over moving video; the default (no --transparent) opaque near-black
background is unchanged. See render_frame()'s `transparent` branches.

Output: <dir>/frame_000000.png, frame_000001.png, ... (RGBA PNG, one per
CSV row by default; one per 1/--fps-second time-grid sample if --fps is
given -- see build_time_grid()).
"""
import argparse
import bisect
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless by default, GOALS.md -- this script never opens a window
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import cv2

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ADR-0025 lethal-radius candidates (ratified 2026-07-06): kinetic ram
# ~0.5 m (the headline kill mechanism) and net ~1.5 m (the FPV-speed
# forgiveness upgrade). --radius overrides; default here is the net
# figure since it is the more forgiving/likely demo-reel number, but
# either is a legitimate, disclosed "narrative assumption" per ADR-0014 --
# there is no collision volume in the sim.
LETHAL_RADIUS_NET_M = 1.5
LETHAL_RADIUS_RAM_M = 0.5
DEFAULT_RADIUS_M = LETHAL_RADIUS_NET_M

# Canvas geometry: a tall narrow "sidebar" panel meant to sit beside a
# widescreen Gazebo GUI capture (docs/demo_plan.md's ffmpeg hstack, not
# built yet). Top band = text readouts; bottom band = the mini-map.
FIG_W_IN, FIG_H_IN = 6.4, 10.8
FIG_DPI = 100  # -> 640x1080 px RGBA frames

# Aerospace glass-cockpit palette (near-black bg, monospace, green/amber/red).
COLOR_BG = "#050708"
COLOR_PANEL_LINE = "#123018"
COLOR_GREEN = "#39ff6a"
COLOR_AMBER = "#ffb000"
COLOR_RED = "#ff3b30"
COLOR_DIM = "#3a4a42"      # blank/NaN readouts -- dark, never a false number
COLOR_WHITE = "#d8f5e0"
COLOR_GT_REF = "#6a7a72"   # ground-truth reference elements -- visually distinct/dim

# --transparent panel backing: widget panels (lamp boxes, the readout
# block, the mini-map) keep this semi-opaque dark fill for legibility when
# composited over moving video; everything NOT inside a panel/widget stays
# alpha=0 (see render_frame()'s `transparent` branches).
PANEL_ALPHA = 0.55

PHASE_COLORS = {
    "TAKEOFF": COLOR_WHITE,
    "ACQUIRE": COLOR_WHITE,
    "CUE_WAIT": COLOR_AMBER,
    "DASH": COLOR_AMBER,
    "ENGAGE": COLOR_GREEN,
    "BREAKOFF": COLOR_RED,
    "BENCH_WAVE": COLOR_WHITE,
    "BENCH_RECENTER": COLOR_WHITE,
}


def parse_float(value):
    """Blank/None/unparseable -> NaN, never 0.0. This is the honesty-
    critical function in this file: every numeric readout in the CSV goes
    through this, so a missing measurement renders as a dim "--", not a
    false zero (docs/demo_plan.md's "gotcha" -- see module docstring)."""
    if value is None:
        return float("nan")
    value = value.strip()
    if value == "":
        return float("nan")
    try:
        return float(value)
    except ValueError:
        return float("nan")


def parse_flag01(value):
    """CSV 0/1/blank flag column (e.g. ext_fresh, detected) -> Optional[int].
    None (not 0) when the cell is blank -- a blank flag is "unknown/not
    applicable this tick", never silently treated as False."""
    if value is None:
        return None
    value = value.strip()
    if value == "":
        return None
    try:
        return int(float(value))
    except ValueError:
        return None


def load_rows(csv_path):
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise SystemExit(f"render_hud: {csv_path} has no data rows")
    return rows


def col_floats(rows, name):
    """A numpy float array for column `name`, NaN for blank/missing (the
    column may not even exist in an older CSV -- e.g. t_sim predates this
    tool; row.get() returns None uniformly for "column absent" and "cell
    blank", which parse_float() both turn into NaN)."""
    return np.array([parse_float(r.get(name)) for r in rows], dtype=float)


def compute_handoff_idx(rows):
    """Index (0-based) of the LAST row where ext_fresh == 1 -- the tick the
    external-cue channel was still alive, immediately before the one-way
    HANDOFF latch (m4_intercept.py ADR-0010 #5) closes and nulls the
    CueReader. Source trace: ext_fresh is reset to 0 at the top of every
    tick (m4_intercept.py, run_acquire_and_engage) and only set to 1 inside
    `if args.handoff and cue_reader is not None: ...`; once HANDOFF fires,
    cue_reader is closed and the guidance loop sets its own reference to
    None, so that branch can never execute again for the rest of the run --
    ext_fresh is therefore STRUCTURALLY stuck at 0 from the next tick on
    (see CueReader's docstring, "LATCH ONE-WAY"). Returns None if the CSV
    never has an ext_fresh==1 row at all (no --handoff, or a --handoff run
    that never received a cue before aborting) -- those flights are
    CAMERA-ONLY for their whole length."""
    last = None
    for i, r in enumerate(rows):
        if parse_flag01(r.get("ext_fresh")) == 1:
            last = i
    return last


def mode_label(i, handoff_idx):
    if handoff_idx is None:
        return "CAMERA-ONLY"
    return "EXTERNAL CUE (mocked)" if i <= handoff_idx else "CAMERA-ONLY"


def solution_label(tgt_n_hat_i):
    return "CONVERGED" if not math.isnan(tgt_n_hat_i) else "SOLVING"


def compute_cpa_idx(gt_range):
    """Index of the closest-approach tick by ground-truth range (gt_range)
    -- this is the legitimate SCORING use of gt_* (see module docstring's
    honesty section): it never touches guidance, it only reports what
    actually happened. None if gt_range is all-NaN (pose topic never
    populated -- shouldn't happen on a real run, but don't crash)."""
    valid = ~np.isnan(gt_range)
    if not valid.any():
        return None
    idx = np.where(valid)[0]
    return int(idx[np.argmin(gt_range[idx])])


def fmt_num(value, unit, spec="{:.2f}"):
    if math.isnan(value):
        return "--"
    return spec.format(value) + unit


def build_time_grid(rows, fps):
    """A fixed dt=1/fps time grid over [t0, t1], preferring SIM time
    (t_sim) since wall time sags under RTF load (ADR-0009) and is not a
    faithful clock for a played-back video. Falls back to wall time `t`
    (with a printed warning -- this is exactly the kind of thing
    docs/demo_plan.md's honest-depiction rules say must be disclosed, not
    silently substituted) when t_sim is missing/mostly blank -- e.g. any
    CSV recorded before this column existed, or any non---handoff run
    (t_sim is only populated when --handoff subscribes to /clock; see
    m4_intercept.py's CSV_HEADER comment). Returns (grid_times, sample_idx)
    where sample_idx[k] is the row index to hold (ZOH) for grid_times[k]."""
    t_sim = col_floats(rows, "t_sim")
    frac_valid = np.mean(~np.isnan(t_sim)) if len(t_sim) else 0.0
    if frac_valid > 0.5:
        clock = t_sim
    else:
        print(
            "[render_hud] WARNING: t_sim missing/mostly blank (pre-dates "
            "this column, or a non---handoff run) -- falling back to WALL "
            "time for --fps sampling. Wall time is NOT a faithful sim "
            "clock under RTF sag (ADR-0009); disclose any timing claim "
            "made from this in the demo video.",
            file=sys.stderr,
        )
        clock = col_floats(rows, "t")

    # Any row with a NaN clock value can't anchor the grid -- drop it from
    # the search array (keep original row indices via `valid_idx`).
    valid_idx = np.where(~np.isnan(clock))[0]
    if len(valid_idx) == 0:
        raise SystemExit("render_hud: no usable timestamp column (t or t_sim) found")
    clock_valid = clock[valid_idx]
    t0, t1 = float(clock_valid[0]), float(clock_valid[-1])
    if t1 <= t0:
        raise SystemExit("render_hud: non-increasing time column, cannot build --fps grid")

    dt = 1.0 / fps
    n = int(math.floor((t1 - t0) / dt)) + 1
    grid_times = [t0 + k * dt for k in range(n)]
    clock_list = list(clock_valid)
    sample_idx = []
    for gt in grid_times:
        # Zero-order hold: the latest row at/before this grid time.
        pos = bisect.bisect_right(clock_list, gt) - 1
        pos = max(pos, 0)
        sample_idx.append(int(valid_idx[pos]))
    return grid_times, sample_idx


class HudContext:
    """Precomputed, whole-flight state shared by every frame: fixed map
    extent (so the mini-map doesn't jitter/rescale tick to tick -- a real
    HUD wouldn't), the handoff tick, and the closest-approach tick. All
    derived once from the full CSV, all NaN-safe."""

    def __init__(self, rows, radius_m, source_name=""):
        self.rows = rows
        self.n = len(rows)
        self.radius_m = radius_m
        self.source_name = source_name

        self.gt_cam_x = col_floats(rows, "gt_cam_x")
        self.gt_cam_y = col_floats(rows, "gt_cam_y")
        self.gt_tag_x = col_floats(rows, "gt_tag_x")
        self.gt_tag_y = col_floats(rows, "gt_tag_y")
        self.tgt_n_hat = col_floats(rows, "tgt_n_hat")
        self.tgt_e_hat = col_floats(rows, "tgt_e_hat")
        self.gt_range = col_floats(rows, "gt_range")
        self.psi_deg = col_floats(rows, "psi_deg")
        self.vc_m_s = col_floats(rows, "vc_m_s")
        self.r_hat_m = col_floats(rows, "r_hat_m")
        self.lambda_dot_deg_s = col_floats(rows, "lambda_dot_deg_s")
        # Optional[int] per row, NOT a float array -- None (blank cell) must
        # stay distinguishable from 0 ("SEARCHING") for the APRILTAG lamp's
        # third "--" (pre-acquisition/unknown) state. See parse_flag01().
        self.detected_flags = [parse_flag01(r.get("detected")) for r in rows]

        self.handoff_idx = compute_handoff_idx(rows)
        self.cpa_idx = compute_cpa_idx(self.gt_range)

        # Own-ship world frame is ENU (x=East, y=North; GOALS.md + the
        # cue's world->NED mapping comment in m4_intercept.py both confirm
        # world_x=East/world_y=North). tgt_n_hat/e_hat are already named
        # North/East in the same frame (no translation term needed --
        # GOALS.md: "World: ENU, origin at the interceptor's start").
        east_pts = [self.gt_cam_x, self.gt_tag_x, self.tgt_e_hat]
        north_pts = [self.gt_cam_y, self.gt_tag_y, self.tgt_n_hat]
        all_east = np.concatenate([a[~np.isnan(a)] for a in east_pts])
        all_north = np.concatenate([a[~np.isnan(a)] for a in north_pts])
        if len(all_east) == 0:
            # Degenerate CSV (no ground truth ever populated) -- still
            # render, just with an arbitrary unit box instead of crashing.
            all_east = np.array([0.0, 1.0])
            all_north = np.array([0.0, 1.0])
        margin = max(radius_m * 1.5, 1.0)
        self.xlim = (float(all_east.min()) - margin, float(all_east.max()) + margin)
        self.ylim = (float(all_north.min()) - margin, float(all_north.max()) + margin)


def _monospace_family():
    """Prefer the real monospace TTF matplotlib ships (DejaVu Sans Mono) --
    guarantees the aerospace glass-cockpit "monospace" look is an ACTUAL
    monospace font, not just a font family hint that silently falls back
    to a proportional font if the named family isn't installed."""
    names = {f.name for f in fm.fontManager.ttflist}
    return "DejaVu Sans Mono" if "DejaVu Sans Mono" in names else "monospace"


MONO = _monospace_family()


def render_frame(ctx, i, out_path, transparent=False):
    row = ctx.rows[i]
    fig = plt.figure(figsize=(FIG_W_IN, FIG_H_IN), dpi=FIG_DPI)
    if transparent:
        # The whole canvas starts fully transparent (alpha=0). Widget
        # panels below opt back IN to a semi-opaque dark backing
        # (PANEL_ALPHA) for legibility; everything else -- the gaps
        # between panels, the map's side margins -- stays alpha=0, per
        # the --transparent contract in the module docstring.
        fig.patch.set_alpha(0.0)
    else:
        fig.patch.set_facecolor(COLOR_BG)

    # facecolor/alpha kwargs for the three bordered lamp boxes (PHASE,
    # SENSOR, APRILTAG): opaque mode relies on txt_ax's own opaque
    # background so the boxes stay outline-only (unchanged look);
    # transparent mode fills each box with a semi-opaque backing plate
    # since the canvas behind it is now alpha=0.
    box_fill = dict(facecolor=COLOR_BG, alpha=PANEL_ALPHA) if transparent else dict(facecolor="none")

    # --- Text/readout panel (top ~42% of the canvas) ---
    txt_ax = fig.add_axes((0.0, 0.58, 1.0, 0.42))
    txt_ax.set_xlim(0, 1)
    txt_ax.set_ylim(0, 1)
    txt_ax.axis("off")
    if transparent:
        txt_ax.patch.set_alpha(0.0)
        # One backing plate behind the whole lower readout block
        # (INTERCEPT SOLUTION through the honesty footnotes) so that
        # block reads as a single legible instrument cluster over video,
        # flush against the APRILTAG box's bottom edge (y=0.66) -- see
        # the APRILTAG panel below.
        txt_ax.add_patch(plt.Rectangle((0.0, 0.0), 1.0, 0.66, facecolor=COLOR_BG,
                                        alpha=PANEL_ALPHA, edgecolor="none", zorder=0))
    else:
        txt_ax.set_facecolor(COLOR_BG)

    phase = row.get("phase", "") or "--"
    phase_color = PHASE_COLORS.get(phase, COLOR_WHITE)
    txt_ax.add_patch(plt.Rectangle((0.03, 0.90), 0.94, 0.09,
                                    edgecolor=phase_color, linewidth=2, **box_fill))
    txt_ax.text(0.5, 0.945, f"PHASE: {phase}", color=phase_color, fontsize=20,
                family=MONO, ha="center", va="center", weight="bold")

    label = mode_label(i, ctx.handoff_idx)
    lamp_color = COLOR_GREEN if label == "CAMERA-ONLY" else COLOR_AMBER
    txt_ax.add_patch(plt.Rectangle((0.03, 0.78), 0.94, 0.09,
                                    edgecolor=lamp_color, linewidth=2, **box_fill))
    txt_ax.text(0.5, 0.825, f"SENSOR: {label}", color=lamp_color, fontsize=15,
                family=MONO, ha="center", va="center", weight="bold")

    # APRILTAG LOCK lamp <- `detected` (see parse_flag01() / module
    # docstring honesty note). Three states, never a 0-fill on blank:
    #   None (blank cell)  -> "--"        dim/unknown (pre-acquisition)
    #   0                  -> "SEARCHING" amber (mid-flight, no lock this tick)
    #   1                  -> "LOCKED"    green
    det_i = ctx.detected_flags[i]
    if det_i is None:
        tag_text, tag_color = "APRILTAG: --", COLOR_DIM
    elif det_i == 1:
        tag_text, tag_color = "APRILTAG: LOCKED", COLOR_GREEN
    else:
        tag_text, tag_color = "APRILTAG: SEARCHING", COLOR_AMBER
    txt_ax.add_patch(plt.Rectangle((0.03, 0.66), 0.94, 0.09,
                                    edgecolor=tag_color, linewidth=2, **box_fill))
    txt_ax.text(0.5, 0.705, tag_text, color=tag_color, fontsize=15,
                family=MONO, ha="center", va="center", weight="bold")

    sol = solution_label(ctx.tgt_n_hat[i])
    sol_color = COLOR_GREEN if sol == "CONVERGED" else COLOR_AMBER
    txt_ax.text(0.5, 0.58, f"INTERCEPT SOLUTION: {sol}", color=sol_color, fontsize=13,
                family=MONO, ha="center", va="center")

    def readout_line(y, label_text, value_text, good):
        color = COLOR_GREEN if good else COLOR_DIM
        txt_ax.text(0.06, y, label_text, color=COLOR_WHITE, fontsize=13,
                    family=MONO, ha="left", va="center")
        txt_ax.text(0.94, y, value_text, color=color, fontsize=13,
                    family=MONO, ha="right", va="center", weight="bold")

    vc = ctx.vc_m_s[i]
    readout_line(0.49, "CLOSING SPEED", fmt_num(vc, " m/s"), not math.isnan(vc))
    r_hat = ctx.r_hat_m[i]
    readout_line(0.415, "RANGE", fmt_num(r_hat, " m"), not math.isnan(r_hat))
    ldot = ctx.lambda_dot_deg_s[i]
    readout_line(0.34, "LOS RATE", fmt_num(ldot, " deg/s", "{:+.1f}"), not math.isnan(ldot))
    law = (row.get("law") or "--").upper()
    readout_line(0.265, "LAW", law, True)

    t_display = row.get("t_sim") or ""
    t_display = t_display.strip() if t_display else ""
    if not t_display:
        t_display = row.get("t", "") + "s (wall)"
    else:
        t_display = t_display + "s (sim)"
    txt_ax.text(0.5, 0.15, f"t = {t_display}", color=COLOR_GT_REF, fontsize=10,
                family=MONO, ha="center", va="center")

    # Honest-depiction footnotes (docs/demo_plan.md "Honest-depiction
    # rules" -- static every frame, cheap, and non-negotiable per CLAUDE.md).
    txt_ax.text(0.5, 0.075,
                "EXTERNAL CUE = mocked ground-sensor stand-in, not live hardware",
                color=COLOR_GT_REF, fontsize=8, family=MONO, ha="center", va="center")
    txt_ax.text(0.5, 0.03,
                "GT markers = scoring reference only, never fed to guidance",
                color=COLOR_GT_REF, fontsize=8, family=MONO, ha="center", va="center")

    # --- Mini-map (bottom ~55% of the canvas) ---
    map_ax = fig.add_axes((0.08, 0.04, 0.84, 0.50))
    map_ax.set_facecolor(COLOR_BG)
    if transparent:
        # The map is itself a widget panel -- keep it as a legible
        # semi-opaque plate (its own facecolor/alpha), not fully
        # transparent; the area OUTSIDE this axes' rectangle (side
        # margins, the gap above it) is still alpha=0 via fig.patch.
        map_ax.patch.set_alpha(PANEL_ALPHA)
    for spine in map_ax.spines.values():
        spine.set_color(COLOR_PANEL_LINE)
    map_ax.tick_params(colors=COLOR_GT_REF, labelsize=7)
    map_ax.set_xlim(*ctx.xlim)
    map_ax.set_ylim(*ctx.ylim)
    map_ax.set_aspect("equal")
    map_ax.grid(color=COLOR_PANEL_LINE, linewidth=0.5, alpha=0.6)
    map_ax.set_xlabel("EAST (m)", color=COLOR_GT_REF, fontsize=8, family=MONO)
    map_ax.set_ylabel("NORTH (m)", color=COLOR_GT_REF, fontsize=8, family=MONO)

    upto = i + 1

    # Ground-truth reference target track (dashed, dim, labeled).
    gt_e, gt_n = ctx.gt_tag_x[:upto], ctx.gt_tag_y[:upto]
    m = ~np.isnan(gt_e) & ~np.isnan(gt_n)
    if m.any():
        map_ax.plot(gt_e[m], gt_n[m], "--", color=COLOR_GT_REF, linewidth=1.0,
                    label="target GT (reference only)")

    # Guidance-legitimate target track estimate (solid, amber).
    tgt_e, tgt_n = ctx.tgt_e_hat[:upto], ctx.tgt_n_hat[:upto]
    m = ~np.isnan(tgt_e) & ~np.isnan(tgt_n)
    if m.any():
        map_ax.plot(tgt_e[m], tgt_n[m], "-", color=COLOR_AMBER, linewidth=1.5,
                    label="target track (tracker estimate)")
        map_ax.plot(tgt_e[m][-1], tgt_n[m][-1], "o", color=COLOR_AMBER, markersize=6)

    # Own-ship trail + heading arrow (reference position -- not fed back
    # into guidance, see module docstring).
    own_e, own_n = ctx.gt_cam_x[:upto], ctx.gt_cam_y[:upto]
    m = ~np.isnan(own_e) & ~np.isnan(own_n)
    if m.any():
        map_ax.plot(own_e[m], own_n[m], "-", color=COLOR_GREEN, linewidth=1.5,
                    label="own ship (GT position)")
        cx, cy = own_e[m][-1], own_n[m][-1]
        psi = ctx.psi_deg[i]
        if not math.isnan(psi):
            rad = math.radians(psi)
            dx, dy = math.sin(rad), math.cos(rad)  # heading 0=North(+y), 90=East(+x)
            scale = 0.06 * (ctx.xlim[1] - ctx.xlim[0])
            map_ax.annotate("", xy=(cx + dx * scale, cy + dy * scale), xytext=(cx, cy),
                            arrowprops=dict(color=COLOR_GREEN, width=1.5, headwidth=6))
        else:
            map_ax.plot(cx, cy, "^", color=COLOR_GREEN, markersize=8)

    # Lethal-radius ring + closest-approach marker: only once it has
    # actually happened (i >= cpa_idx) -- no foreknowledge.
    if ctx.cpa_idx is not None and i >= ctx.cpa_idx:
        cx, cy = ctx.gt_tag_x[ctx.cpa_idx], ctx.gt_tag_y[ctx.cpa_idx]
        if not (math.isnan(cx) or math.isnan(cy)):
            ring = plt.Circle((cx, cy), ctx.radius_m, fill=False,
                              edgecolor=COLOR_RED, linewidth=1.5, linestyle=":")
            map_ax.add_patch(ring)
            map_ax.plot(cx, cy, "x", color=COLOR_RED, markersize=9, markeredgewidth=2)
            miss = ctx.gt_range[ctx.cpa_idx]
            map_ax.text(
                cx, cy - ctx.radius_m * 1.3,
                f"CPA {fmt_num(miss, ' m')} (R_lethal={ctx.radius_m:.1f} m,\n"
                "criterion, not a modeled collision)",
                color=COLOR_RED, fontsize=7, family=MONO, ha="center", va="top",
            )

    legend = map_ax.legend(loc="upper left", fontsize=6, facecolor=COLOR_BG,
                            edgecolor=COLOR_PANEL_LINE, labelcolor=COLOR_WHITE)
    if legend:
        legend.get_frame().set_alpha(0.7)

    # --- Footer strip: subtle, small, run-traceability caption (CLAUDE.md
    # "numbers trace to a run") -- not a widget readout, just a source tag
    # so a frame pulled out of context still says where its numbers came
    # from. Sits in the margin below the map; alpha=0 background under
    # --transparent (bare small text), opaque COLOR_BG otherwise.
    footer_ax = fig.add_axes((0.0, 0.0, 1.0, 0.035))
    footer_ax.set_xlim(0, 1)
    footer_ax.set_ylim(0, 1)
    footer_ax.axis("off")
    if transparent:
        footer_ax.patch.set_alpha(0.0)
    else:
        footer_ax.set_facecolor(COLOR_BG)
    footer_ax.text(
        0.5, 0.5,
        f"render_hud.py (offline overlay, not in-sim capture) -- source: {ctx.source_name or '--'}",
        color=COLOR_DIM, fontsize=6, family=MONO, ha="center", va="center",
    )

    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())  # (H, W, 4) uint8
    plt.close(fig)

    bgra = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA)
    ok = cv2.imwrite(out_path, bgra)
    if not ok:
        raise RuntimeError(f"render_hud: cv2.imwrite failed for {out_path}")
    return rgba.shape


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("csv_path", help="a scripts/m4_intercept.py (or m3_static_intercept.py) per-tick CSV")
    ap.add_argument("--out", required=True, help="output directory for frame_*.png")
    ap.add_argument("--fps", type=float, default=None,
                    help="resample to a fixed-fps time grid (ZOH; sim-time preferred, "
                         "see build_time_grid()) instead of one frame per CSV row")
    ap.add_argument("--radius", type=float, default=DEFAULT_RADIUS_M,
                    help=f"lethal-radius ring, meters (ADR-0025: net={LETHAL_RADIUS_NET_M}, "
                         f"ram={LETHAL_RADIUS_RAM_M}; default {DEFAULT_RADIUS_M})")
    ap.add_argument("--transparent", action="store_true",
                    help="write frames with the empty canvas at alpha=0 (widget panels keep a "
                         "semi-opaque dark backing) for compositing as an overlay layer on top "
                         "of a Gazebo render, instead of the default opaque near-black sidebar")
    args = ap.parse_args()

    rows = load_rows(args.csv_path)
    os.makedirs(args.out, exist_ok=True)
    ctx = HudContext(rows, args.radius, source_name=os.path.basename(args.csv_path))

    if args.fps:
        _grid_times, sample_idx = build_time_grid(rows, args.fps)
        indices = sample_idx
    else:
        indices = list(range(len(rows)))

    shape = None
    for frame_num, row_idx in enumerate(indices):
        out_path = os.path.join(args.out, f"frame_{frame_num:06d}.png")
        shape = render_frame(ctx, row_idx, out_path, transparent=args.transparent)

    print(
        f"[render_hud] wrote {len(indices)} frames to {args.out} "
        f"(source rows={len(rows)}, frame shape={shape}, "
        f"handoff_idx={ctx.handoff_idx}, cpa_idx={ctx.cpa_idx}, "
        f"transparent={args.transparent})"
    )


if __name__ == "__main__":
    main()
