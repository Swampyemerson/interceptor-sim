#!/usr/bin/env python3
"""Offline, deterministic HUD/OSD-frame generator (docs/demo_plan.md, Part 1).

WHAT: reads a per-tick flight CSV written by scripts/m4_intercept.py (or
scripts/m3_static_intercept.py -- same style, a strict subset of columns)
and renders one RGBA PNG HUD frame per CSV row (or per a fixed time-grid
sample, or per an external capture manifest's sim_t list). This is NOT
rendered in-sim -- it is a pure function of logged columns, so every number
on screen traces to the CSV. That is the whole point of "data-driven, not
screen-only" (docs/demo_plan.md Part 1).

TWO LAYOUTS (--layout):
  sidebar  (default, back-compat) -- a tall 640x1080 near-black panel meant
           to sit BESIDE a widescreen render (ffmpeg hstack). Unchanged
           behaviour for existing callers (scripts/compose_demo.sh).
  overlay  -- a real FPV/interceptor OSD sized to the VIDEO frame (default
           1280x960, --width/--height) with instruments in the MARGINS and
           the centre kept clear for the seeker view, for compositing as a
           transparent LAYER on top of the onboard-camera footage (the
           primary portfolio deliverable, docs/demo_plan.md TODO). Every
           field traces to a CSV column; unsourced fields are omitted.

OSD FIELDS (overlay) and the exact CSV column driving each:
  PHASE lamp                 <- phase
  SENSOR / mode lamp         <- ext_fresh          (see compute_handoff_idx())
  APRILTAG LOCK lamp         <- detected           (see parse_flag01())
  HEADING tape (attitude)    <- psi_deg
  T_GO (time-to-intercept)   <- r_hat_m / vc_m_s   (camera firing solution)
  RANGE bar + readout        <- r_hat_m            (R_net/R_ram tick = --radius)
  CLOSING SPEED readout       <- vc_m_s
  LOS RATE readout            <- lambda_dot_deg_s   (the live pro-nav signal)
  GND SPD gauge               <- d/dt of gt_cam_x/y (GT-derived, scoring ref;
                                 0.3 s smoothed -- see HudContext.ground_speed)
  ALT gauge/readout           <- alt_m

  The top-down mini-map, the INTERCEPT-SOLUTION status line, and the "LAW"
  label were REMOVED from the OVERLAY OSD in the 2026-07-07 de-cringe pass
  (builder feedback: the OSD should read like a clean instrument panel, not a
  moving-map video game -- err toward LESS). The mini-map lives on in the
  `sidebar` layout only; draw_minimap() is retained for that caller.

WHY NO PITCH/ROLL HORIZON: the CSV has psi_deg (yaw/heading) but no pitch or
roll column. A flight-path angle derived from d(alt)/dt over ground speed is
near-zero and noise-dominated in this low-altitude, level engagement (it
climbs on takeoff, then dashes and engages essentially level), and roll has
no honest source at all -- so an artificial horizon would be fabricated. Per
docs/demo_plan.md's "cut anything decorative that doesn't inform" and "if a
field has no honest source, omit it", the attitude instrument here is the
HEADING tape (psi_deg) only. Overlaying a wings-level artificial horizon on
the onboard footage -- whose real horizon banks hard in the terminal -- would
also visibly contradict the scene.

HONESTY / ROBUSTNESS RULES (read before touching -- project non-negotiables):
  - Blank CSV cells (r_hat_m/lambda_dot_deg_s during TAKEOFF/CUE_WAIT/DASH,
    before the camera has a track) parse as NaN, never 0 -- see parse_float().
    A 0-fill would draw a false "0 m range" or "LOCK". NaN readouts render as
    "--" in a dim colour, never a number, and gauges/bars draw empty.
  - The SENSOR/mode lamp ("EXTERNAL CUE (mocked)" -> "CAMERA-ONLY") is keyed
    off `ext_fresh`, NOT a "HANDOFF" phase value -- there is no such phase.
    The flip tick is the LAST row where ext_fresh == 1, a one-way latch
    (ADR-0010 #5; see compute_handoff_idx()).
  - The mini-map lethal-radius ring / closest-approach marker are keyed off
    `gt_range` (ground truth). That is the legitimate SCORING use of gt_* per
    CLAUDE.md's honesty boundary: gt_* never feeds guidance, it only tells the
    story of what happened. Drawn visually distinct and never before the tick
    it happens on (no foreknowledge). GND SPD is likewise GT-derived and
    display-only (labelled), consistent with the map's own-ship GT position.
  - The APRILTAG lamp is keyed off `detected` (0/1/blank) through
    parse_flag01(): blank -> "--" (pre-acquisition), 0 -> "SEARCHING",
    1 -> "LOCKED". The lamp says "APRILTAG", not "TARGET LOCK", so the honest
    simplification (fiducial stand-in for a real seeker, docs/goals.md) reads
    on-screen even out of context.

DATAVIZ DESIGN RULES (docs/demo_plan.md TODO, "dataviz skill", 2026-07-07):
  - Status colours (green/amber/red) are RESERVED for status lamps and are
    always paired with TEXT, never colour-alone.
  - Readout/gauge VALUES render in white/gray ink, never a status accent.
  - Gauge/bar fills use ONE sequential magnitude hue (COLOR_FILL), distinct
    from the status palette; one scale per gauge, no dual axes.
  - The mini-map's two series are CVD-safe by construction: cool cyan SOLID
    "true path" vs warm amber BROKEN "cue estimate" -- warm/cool + line-style
    + direct labels, never colour alone.
  - Recessive grid/axes; thin marks; monospace throughout.

CLI:
    .venv/bin/python scripts/render_hud.py <flight_csv> --out <dir> \\
        [--layout sidebar|overlay] [--width W --height H] \\
        [--fps N | --times-from <manifest.csv>] [--radius M] [--transparent]

--transparent: empty canvas at alpha=0; only drawn instrument plates/text
are opaque/semi-opaque, for compositing as an overlay LAYER over a render.
--times-from <manifest.csv>: render one frame per row of an external capture
manifest (must have a `sim_t` column), zero-order-hold sampling the flight
CSV at each manifest sim_t -- gives HUD frames INDEX-ALIGNED 1:1 with a
captured onboard/chase frame sequence for a clean overlay composite.

Output: <dir>/frame_000000.png, frame_000001.png, ... (RGBA PNG).
"""
import argparse
import bisect
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless by default, docs/goals.md -- this script never opens a window
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import cv2

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Lethal-radius candidates: kinetic ram 0.35 m (headline kill, RATIFIED
# 2026-07-25 ADR-0084 from the ordered 5-inch pair's contact envelope -- was
# 0.5 m, the retired 7-inch number) and net ~1.5 m (FPV-speed forgiveness).
# --radius overrides; default is the net figure (the more forgiving/likely
# demo-reel number), a disclosed narrative assumption per ADR-0014 -- there is
# no collision volume in the sim.
LETHAL_RADIUS_NET_M = 1.5
LETHAL_RADIUS_RAM_M = 0.35
DEFAULT_RADIUS_M = LETHAL_RADIUS_NET_M


def radius_label(r):
    """Short on-screen NAME for a criterion radius, so a HUD tick/ring can never
    be read as the WRONG kill mechanism (ADR-0084 claims-scrub, 2026-07-25: the
    ram radius is 0.35 m from the ordered 5-inch pair; the 1.5 m net-class
    radius is a different, more forgiving mechanism and is unchanged)."""
    if abs(r - LETHAL_RADIUS_RAM_M) < 1e-6:
        return "R_ram"
    if abs(r - LETHAL_RADIUS_NET_M) < 1e-6:
        return "R_net"
    return "R_crit"


def fmt_radius(r):
    """Radius as text with enough precision to distinguish 0.35 from 1.5."""
    return f"{r:.2f}" if r < 1.0 else f"{r:.1f}"

# Sidebar canvas geometry (the default tall panel; unchanged).
FIG_W_IN, FIG_H_IN = 6.4, 10.8
FIG_DPI = 100  # -> 640x1080 px RGBA

# Overlay canvas default (matches the onboard camera capture, 4:3).
OVERLAY_W, OVERLAY_H = 1280, 960

# Gauge full-scale references (one scale per gauge, dataviz rule).
GND_SPD_MAX = 16.0   # m/s -- the --dash-unclamp dash ceiling (ADR-0030)
ALT_MAX = 2.0        # m   -- the M0 takeoff target; low, level engagement

# Aerospace glass-cockpit palette (near-black bg, monospace, green/amber/red).
COLOR_BG = "#050708"
COLOR_PANEL_LINE = "#123018"
COLOR_GREEN = "#39ff6a"
COLOR_AMBER = "#ffb000"
COLOR_RED = "#ff3b30"
COLOR_DIM = "#3a4a42"      # blank/NaN readouts -- dark, never a false number
COLOR_WHITE = "#d8f5e0"    # primary readout ink
COLOR_GRAY = "#8fa39a"     # labels / units / recessive ink
COLOR_GT_REF = "#6a7a72"   # dim ground-truth text/footnotes
COLOR_FILL = "#37b0c4"     # ONE sequential magnitude hue for gauge/bar fills

# Mini-map TWO-SERIES track pair (CVD-safe: cool vs warm hue + line-style +
# direct labels, never colour alone). Deliberately distinct from COLOR_AMBER.
MAP_TRUE_PATH_COLOR = "#4fc3d6"   # cool cyan -- true target path (reference only)
MAP_CUE_EST_COLOR = "#e8912e"     # warm amber -- degraded ground-cue estimate

PANEL_ALPHA = 0.55   # semi-opaque dark backing for instrument plates over video

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
    """Blank/None/unparseable -> NaN, never 0.0. The honesty-critical parser:
    every numeric readout goes through this, so a missing measurement renders
    as a dim "--", not a false zero (docs/demo_plan.md gotcha)."""
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
    """CSV 0/1/blank flag (ext_fresh, detected) -> Optional[int]. None (not 0)
    when blank -- a blank flag is "unknown/not applicable", never False."""
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
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"render_hud: {csv_path} has no data rows")
    return rows


def col_floats(rows, name):
    """numpy float array for column `name`, NaN for blank/missing (an absent
    column and a blank cell both -> NaN via row.get() + parse_float())."""
    return np.array([parse_float(r.get(name)) for r in rows], dtype=float)


def compute_handoff_idx(rows):
    """Index of the LAST row where ext_fresh == 1 -- the tick the external-cue
    channel was still alive, immediately before the one-way HANDOFF latch
    (ADR-0010 #5) nulls the CueReader. ext_fresh is reset to 0 each tick and
    only set to 1 inside the cue branch; once HANDOFF fires that branch can
    never run again, so ext_fresh is structurally stuck at 0 afterward.
    Returns None if the CSV never has an ext_fresh==1 row (no --handoff / an
    aborted run) -- those render CAMERA-ONLY for their whole length."""
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
    """Index of closest approach by ground-truth range (legitimate SCORING use
    of gt_*: reports what happened, never touches guidance). None if all-NaN."""
    valid = ~np.isnan(gt_range)
    if not valid.any():
        return None
    idx = np.where(valid)[0]
    return int(idx[np.argmin(gt_range[idx])])


def fmt_num(value, unit, spec="{:.2f}"):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "--"
    return spec.format(value) + unit


def build_time_grid(rows, fps):
    """Fixed dt=1/fps grid over [t0,t1], preferring SIM time (t_sim) since wall
    time sags under RTF load (ADR-0009). Falls back to wall time `t` (printed
    warning -- disclose any timing claim from it) when t_sim is missing/blank.
    Returns (grid_times, sample_idx) with ZOH row indices."""
    t_sim = col_floats(rows, "t_sim")
    frac_valid = np.mean(~np.isnan(t_sim)) if len(t_sim) else 0.0
    if frac_valid > 0.5:
        clock = t_sim
    else:
        print(
            "[render_hud] WARNING: t_sim missing/mostly blank -- falling back "
            "to WALL time for --fps sampling; wall time is NOT a faithful sim "
            "clock under RTF sag (ADR-0009). Disclose any timing claim.",
            file=sys.stderr,
        )
        clock = col_floats(rows, "t")
    valid_idx = np.where(~np.isnan(clock))[0]
    if len(valid_idx) == 0:
        raise SystemExit("render_hud: no usable timestamp column (t or t_sim)")
    clock_valid = clock[valid_idx]
    t0, t1 = float(clock_valid[0]), float(clock_valid[-1])
    if t1 <= t0:
        raise SystemExit("render_hud: non-increasing time column, cannot grid")
    dt = 1.0 / fps
    n = int(math.floor((t1 - t0) / dt)) + 1
    grid_times = [t0 + k * dt for k in range(n)]
    clock_list = list(clock_valid)
    sample_idx = []
    for gt in grid_times:
        pos = max(bisect.bisect_right(clock_list, gt) - 1, 0)
        sample_idx.append(int(valid_idx[pos]))
    return grid_times, sample_idx


def build_manifest_grid(rows, manifest_csv):
    """Render one frame per row of an external capture manifest, zero-order-
    hold sampling the flight CSV at each manifest `sim_t`. Gives HUD frames
    index-aligned 1:1 with a captured onboard/chase frame sequence, so the
    overlay composite is a plain per-index alpha blend (no time drift)."""
    with open(manifest_csv, newline="") as f:
        man = list(csv.DictReader(f))
    if not man or "sim_t" not in man[0]:
        raise SystemExit(f"render_hud: {manifest_csv} needs a 'sim_t' column")
    clock = col_floats(rows, "t_sim")
    valid_idx = np.where(~np.isnan(clock))[0]
    if len(valid_idx) == 0:
        raise SystemExit("render_hud: flight CSV has no t_sim for manifest sync")
    clock_list = list(clock[valid_idx])
    sample_idx = []
    for m in man:
        try:
            st = float(m["sim_t"])
        except (ValueError, KeyError):
            st = clock_list[0]
        pos = max(bisect.bisect_right(clock_list, st) - 1, 0)
        sample_idx.append(int(valid_idx[pos]))
    return sample_idx


class HudContext:
    """Precomputed, whole-flight state shared by every frame: fixed map extent
    (no tick-to-tick jitter), handoff tick, closest-approach tick, a smoothed
    GT-derived ground speed, and the range-bar full scale. All NaN-safe."""

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
        self.alt_m = col_floats(rows, "alt_m")
        self.t_sim = col_floats(rows, "t_sim")
        # Optional[int] per row (blank cell -> None, distinct from 0) for the
        # APRILTAG lamp's three states. See parse_flag01().
        self.detected_flags = [parse_flag01(r.get("detected")) for r in rows]

        self.handoff_idx = compute_handoff_idx(rows)
        self.cpa_idx = compute_cpa_idx(self.gt_range)
        self.ground_speed = self._ground_speed(win_s=0.3)

        # Range-bar full scale = the largest valid own-ship range estimate
        # (the camera acquisition/handoff range). Fallback keeps a sane axis
        # if r_hat is never positive.
        pos_r = self.r_hat_m[self.r_hat_m > 0]
        self.r0_range = float(np.nanmax(pos_r)) if pos_r.size else 6.0

        # Own-ship world frame is ENU (x=East, y=North; docs/goals.md). tgt_n/e_hat
        # are already North/East in the same frame (no translation).
        #
        # MAP EXTENT is GROUND TRUTH ONLY (gt_cam/gt_tag) -- the degraded cue
        # estimate (tgt_e/n_hat) is DELIBERATELY EXCLUDED so its honest 5-15 m
        # swings don't drag the zoom out and squash the smooth ground-truth
        # path (builder-feedback fix). The estimate still plots every tick; it
        # simply gets clipped when it swings outside the frame.
        all_east = np.concatenate([a[~np.isnan(a)] for a in (self.gt_cam_x, self.gt_tag_x)])
        all_north = np.concatenate([a[~np.isnan(a)] for a in (self.gt_cam_y, self.gt_tag_y)])
        if len(all_east) == 0:
            all_east = np.array([0.0, 1.0])
            all_north = np.array([0.0, 1.0])
        margin = max(radius_m * 1.5, 1.0)
        self.xlim = (float(all_east.min()) - margin, float(all_east.max()) + margin)
        self.ylim = (float(all_north.min()) - margin, float(all_north.max()) + margin)

    def _ground_speed(self, win_s=0.3):
        """Horizontal ground speed from the GT own-ship position (gt_cam_x/y)
        differentiated over a +/- win_s/2 sim-time window. GT-derived and
        DISPLAY-ONLY (never fed to guidance) -- the same legitimate scoring use
        as the map's own-ship marker. Windowed (not tick-to-tick) because the
        raw finite difference explodes on tiny position jitter when consecutive
        t_sim are close; a ~0.3 s window gives a clean 0 (hover) -> ramp (dash)
        trace validated against the flight CSV."""
        t, cx, cy = self.t_sim, self.gt_cam_x, self.gt_cam_y
        gs = np.full(self.n, np.nan)
        for i in range(self.n):
            if math.isnan(t[i]):
                continue
            j0, j1 = i, i
            while j0 > 0 and not math.isnan(t[j0 - 1]) and t[i] - t[j0 - 1] < win_s / 2:
                j0 -= 1
            while j1 < self.n - 1 and not math.isnan(t[j1 + 1]) and t[j1 + 1] - t[i] < win_s / 2:
                j1 += 1
            dt = t[j1] - t[j0]
            if j1 > j0 and dt > 0 and not (math.isnan(cx[j0]) or math.isnan(cx[j1])):
                gs[i] = math.hypot(cx[j1] - cx[j0], cy[j1] - cy[j0]) / dt
        return gs

    def t_go(self, i):
        """Time-to-intercept estimate = range / closing speed, the standard
        first-order t_go. Only meaningful once the CAMERA has a range solution
        (r_hat_m live in ENGAGE) and closing; NaN otherwise -> "--"."""
        r, vc = self.r_hat_m[i], self.vc_m_s[i]
        if math.isnan(r) or math.isnan(vc) or vc <= 0.1 or r <= 0:
            return float("nan")
        return r / vc


def _monospace_family():
    """Prefer the real monospace TTF matplotlib ships (DejaVu Sans Mono) so the
    glass-cockpit look is an ACTUAL monospace, not a silent proportional
    fallback."""
    names = {f.name for f in fm.fontManager.ttflist}
    return "DejaVu Sans Mono" if "DejaVu Sans Mono" in names else "monospace"


MONO = _monospace_family()


# ----------------------------------------------------------------------------
# Shared mini-map (used by both layouts)
# ----------------------------------------------------------------------------
def draw_minimap(map_ax, ctx, i, transparent=False, compact=False):
    """Draw the top-down East-North mini-map into `map_ax` up to tick i.
    compact=True shrinks fonts / shortens labels for the small overlay inset."""
    lab_fs = 5 if compact else 8
    leg_fs = 5 if compact else 6
    map_ax.set_facecolor(COLOR_BG)
    if transparent:
        map_ax.patch.set_alpha(PANEL_ALPHA)
    for spine in map_ax.spines.values():
        spine.set_color(COLOR_PANEL_LINE)
    map_ax.tick_params(colors=COLOR_GT_REF, labelsize=(5 if compact else 7))
    map_ax.set_xlim(*ctx.xlim)
    map_ax.set_ylim(*ctx.ylim)
    map_ax.set_aspect("equal")
    map_ax.grid(color=COLOR_PANEL_LINE, linewidth=0.5, alpha=0.6)
    if not compact:
        map_ax.set_xlabel("EAST (m)", color=COLOR_GT_REF, fontsize=8, family=MONO)
        map_ax.set_ylabel("NORTH (m)", color=COLOR_GT_REF, fontsize=8, family=MONO)

    upto = i + 1

    # Ground-truth target track: SMOOTH solid cool-cyan line (reference, NOT
    # seen by guidance). Full-length array so any gap renders as a break, never
    # a false interpolation.
    gt_e, gt_n = ctx.gt_tag_x[:upto], ctx.gt_tag_y[:upto]
    if (~np.isnan(gt_e) & ~np.isnan(gt_n)).any():
        map_ax.plot(gt_e, gt_n, "-", color=MAP_TRUE_PATH_COLOR, linewidth=1.6,
                    solid_capstyle="round", zorder=2,
                    label=("TRUE PATH" if compact else "true path (reference, not seen by guidance)"))

    # Degraded ground-cue estimate: warm-amber BROKEN line + per-sample markers
    # (NaNs left in place so matplotlib breaks the line at each dropout).
    tgt_e, tgt_n = ctx.tgt_e_hat[:upto], ctx.tgt_n_hat[:upto]
    m = ~np.isnan(tgt_e) & ~np.isnan(tgt_n)
    if m.any():
        map_ax.plot(tgt_e, tgt_n, "-", marker="o", markersize=(1.8 if compact else 2.8),
                    linewidth=1.1, color=MAP_CUE_EST_COLOR, zorder=3,
                    label=("CUE EST" if compact else "degraded ground-cue estimate"))
        map_ax.plot(tgt_e[m][-1], tgt_n[m][-1], "o", color=MAP_CUE_EST_COLOR,
                    markersize=(4.5 if compact else 6), zorder=4)

    # Own-ship trail + heading arrow (GT position, display only).
    own_e, own_n = ctx.gt_cam_x[:upto], ctx.gt_cam_y[:upto]
    m = ~np.isnan(own_e) & ~np.isnan(own_n)
    if m.any():
        map_ax.plot(own_e[m], own_n[m], "-", color=COLOR_GREEN, linewidth=1.4,
                    label=("OWN SHIP" if compact else "own ship (GT position)"))
        cx, cy = own_e[m][-1], own_n[m][-1]
        psi = ctx.psi_deg[i]
        if not math.isnan(psi):
            rad = math.radians(psi)
            dx, dy = math.sin(rad), math.cos(rad)  # heading 0=North(+y), 90=East(+x)
            scale = 0.06 * (ctx.xlim[1] - ctx.xlim[0])
            map_ax.annotate("", xy=(cx + dx * scale, cy + dy * scale), xytext=(cx, cy),
                            arrowprops=dict(color=COLOR_GREEN, width=1.4, headwidth=(5 if compact else 6)))
        else:
            map_ax.plot(cx, cy, "^", color=COLOR_GREEN, markersize=7)

    # Lethal-radius ring + CPA marker: only once it has happened (no foreknowledge).
    if ctx.cpa_idx is not None and i >= ctx.cpa_idx:
        cx, cy = ctx.gt_tag_x[ctx.cpa_idx], ctx.gt_tag_y[ctx.cpa_idx]
        if not (math.isnan(cx) or math.isnan(cy)):
            map_ax.add_patch(plt.Circle((cx, cy), ctx.radius_m, fill=False,
                                        edgecolor=COLOR_RED, linewidth=1.4, linestyle=":"))
            map_ax.plot(cx, cy, "x", color=COLOR_RED, markersize=(7 if compact else 9),
                        markeredgewidth=2)
            miss = ctx.gt_range[ctx.cpa_idx]
            if compact:
                txt = f"CPA {fmt_num(miss, ' m')}"
            else:
                txt = (f"CPA {fmt_num(miss, ' m')} "
                       f"({radius_label(ctx.radius_m)}={fmt_radius(ctx.radius_m)} m,\n"
                       "criterion, not a modeled collision)")
            map_ax.text(cx, cy - ctx.radius_m * 1.35, txt, color=COLOR_RED,
                        fontsize=(6 if compact else 7), family=MONO, ha="center", va="top")

    legend = map_ax.legend(loc="upper left", fontsize=leg_fs, facecolor=COLOR_BG,
                           edgecolor=COLOR_PANEL_LINE, labelcolor=COLOR_WHITE,
                           handlelength=1.4, borderpad=0.3, labelspacing=0.25)
    if legend:
        legend.get_frame().set_alpha(0.7)


# ----------------------------------------------------------------------------
# Overlay-layout instrument primitives (normalised 0-1 coords on one axes)
# ----------------------------------------------------------------------------
def _plate(ax, x, y, w, h, transparent, edge=COLOR_PANEL_LINE, lw=1.2, z=1):
    """Instrument backing plate: semi-opaque dark fill + thin edge over video
    (transparent mode); outline-only over the already-dark opaque canvas."""
    face = COLOR_BG if transparent else "none"
    alpha = PANEL_ALPHA if transparent else 1.0
    ax.add_patch(plt.Rectangle((x, y), w, h, facecolor=face, alpha=alpha,
                               edgecolor=edge, linewidth=lw, zorder=z))


def _lamp(ax, x, y, w, h, text, color, transparent, fs):
    _plate(ax, x, y, w, h, transparent, edge=color, lw=1.8, z=2)
    ax.text(x + w / 2, y + h / 2, text, color=color, fontsize=fs, family=MONO,
            ha="center", va="center", weight="bold", zorder=3)


def _vgauge(ax, x, y, w, h, value, vmax, label, valtext, transparent, fs, tick_val=None):
    """Vertical fill gauge (one 0..vmax scale). Fill = COLOR_FILL magnitude
    hue; label above in gray; value below in white ink. Optional red tick."""
    _plate(ax, x, y, w, h, transparent, z=2)
    if value is not None and not math.isnan(value):
        frac = max(0.0, min(1.0, value / vmax))
        ax.add_patch(plt.Rectangle((x, y), w, h * frac, facecolor=COLOR_FILL,
                                   alpha=0.85, edgecolor="none", zorder=3))
    if tick_val is not None:
        ty = y + h * max(0.0, min(1.0, tick_val / vmax))
        ax.plot([x, x + w], [ty, ty], color=COLOR_RED, linewidth=1.0, zorder=4)
    ax.text(x + w / 2, y + h + 0.012, label, color=COLOR_GRAY, fontsize=fs * 0.85,
            family=MONO, ha="center", va="bottom", zorder=3)
    ax.text(x + w / 2, y - 0.014, valtext, color=COLOR_WHITE, fontsize=fs,
            family=MONO, ha="center", va="top", weight="bold", zorder=3)


def _range_bar(ax, x, y, w, h, r, r0, r_lethal, transparent, fs):
    """Horizontal RANGE bar that DEPLETES as range closes (full=r0, empty=0),
    with a red criterion-radius tick, NAMED by mechanism (R_net / R_ram / R_crit
    -- ADR-0084). Value in white ink; label in gray."""
    _plate(ax, x, y, w, h, transparent, z=2)
    if r is not None and not math.isnan(r) and r > 0:
        frac = max(0.0, min(1.0, r / r0))
        ax.add_patch(plt.Rectangle((x, y), w * frac, h, facecolor=COLOR_FILL,
                                   alpha=0.85, edgecolor="none", zorder=3))
        valtext = fmt_num(r, " m")
    else:
        valtext = "ACQUIRING" if (r is None or math.isnan(r)) else "--"
    if r0 > 0:
        tx = x + w * max(0.0, min(1.0, r_lethal / r0))
        ax.plot([tx, tx], [y, y + h], color=COLOR_RED, linewidth=1.4, zorder=4)
        ax.text(tx, y - 0.008,
                f"{radius_label(r_lethal)} {fmt_radius(r_lethal)}", color=COLOR_RED,
                fontsize=fs * 0.62, family=MONO, ha="center", va="top", zorder=4)
    ax.text(x, y + h + 0.008, "RANGE", color=COLOR_GRAY, fontsize=fs * 0.8,
            family=MONO, ha="left", va="bottom", zorder=3)
    ax.text(x + w, y + h + 0.008, valtext, color=COLOR_WHITE, fontsize=fs * 0.95,
            family=MONO, ha="right", va="bottom", weight="bold", zorder=3)


def _heading_tape(ax, cx, cy, w, h, psi, transparent, fs):
    """HUD heading tape (the attitude/heading instrument, <- psi_deg). Moving
    compass scale under a fixed centre caret; numeric HDG below. This is the
    only attitude element -- no pitch/roll (unsourced); see module docstring."""
    _plate(ax, cx - w / 2, cy - h / 2, w, h, transparent, z=2)
    baseline = cy - h * 0.10
    half = (w / 2) * 0.92
    ax.plot([cx - half, cx + half], [baseline, baseline], color=COLOR_GRAY,
            linewidth=0.9, zorder=3)
    if psi is None or math.isnan(psi):
        ax.text(cx, cy + h * 0.22, "HDG --", color=COLOR_DIM, fontsize=fs,
                family=MONO, ha="center", va="center", weight="bold", zorder=3)
        return
    p = psi % 360.0
    span = 50.0
    d = math.ceil((p - span) / 10.0) * 10.0
    while d <= p + span:
        off = ((d - p + 180.0) % 360.0) - 180.0
        if abs(off) <= span:
            xx = cx + (off / span) * half
            major = (int(round(d)) % 30 == 0)
            th = h * 0.18 if major else h * 0.10
            ax.plot([xx, xx], [baseline, baseline + th],
                    color=(COLOR_WHITE if major else COLOR_GRAY),
                    linewidth=(1.2 if major else 0.8), zorder=3)
            if major:
                hd = int(round(d)) % 360
                lbl = {0: "N", 90: "E", 180: "S", 270: "W"}.get(hd, f"{hd:03d}")
                ax.text(xx, baseline + th + h * 0.02, lbl, color=COLOR_GRAY,
                        fontsize=fs * 0.72, family=MONO, ha="center", va="bottom", zorder=3)
        d += 10.0
    # fixed centre caret (white reference, not a status colour)
    cw = w * 0.014
    ax.plot([cx - cw, cx, cx + cw], [baseline + h * 0.30, baseline + h * 0.10, baseline + h * 0.30],
            color=COLOR_WHITE, linewidth=1.6, zorder=4)
    ax.text(cx, cy - h * 0.36, f"HDG {int(round(p)) % 360:03d}", color=COLOR_WHITE,
            fontsize=fs, family=MONO, ha="center", va="center", weight="bold", zorder=4)


def _boresight(ax, cx, cy, transparent):
    """Fixed seeker boresight reticle (thin cross with a centre gap). Honest:
    marks the fixed camera axis -- the tag drifting relative to it IS the LOS
    geometry the pro-nav law works on. Dim ink, never a status colour."""
    g, a = 0.018, 0.055   # gap, arm length (normalised)
    col = COLOR_GRAY
    for (x0, y0, x1, y1) in [(cx - g - a, cy, cx - g, cy), (cx + g, cy, cx + g + a, cy),
                             (cx, cy - g - a, cx, cy - g), (cx, cy + g, cx, cy + g + a)]:
        ax.plot([x0, x1], [y0, y1], color=col, linewidth=1.1, alpha=0.9, zorder=3)
    ax.plot([cx], [cy], marker=".", color=col, markersize=2, zorder=3)


def _readout(ax, x, y, label, valtext, fs, ha_val="right", x_val=None):
    """Compact label/value readout: label gray, value white ink (never accent)."""
    ax.text(x, y, label, color=COLOR_GRAY, fontsize=fs * 0.85, family=MONO,
            ha="left", va="center", zorder=3)
    ax.text(x_val if x_val is not None else x, y, valtext, color=COLOR_WHITE,
            fontsize=fs, family=MONO, ha=ha_val, va="center", weight="bold", zorder=3)


# ----------------------------------------------------------------------------
# Sidebar layout (default; back-compat with compose_demo.sh)
# ----------------------------------------------------------------------------
def render_sidebar_frame(ctx, i, out_path, transparent=False):
    row = ctx.rows[i]
    fig = plt.figure(figsize=(FIG_W_IN, FIG_H_IN), dpi=FIG_DPI)
    if transparent:
        fig.patch.set_alpha(0.0)
    else:
        fig.patch.set_facecolor(COLOR_BG)

    box_fill = dict(facecolor=COLOR_BG, alpha=PANEL_ALPHA) if transparent else dict(facecolor="none")

    txt_ax = fig.add_axes((0.0, 0.58, 1.0, 0.42))
    txt_ax.set_xlim(0, 1)
    txt_ax.set_ylim(0, 1)
    txt_ax.axis("off")
    if transparent:
        txt_ax.patch.set_alpha(0.0)
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

    # Readout VALUES in white ink (dataviz rule) -- status accents are reserved
    # for the lamps above; a NaN value renders dim.
    def readout_line(y, label_text, value_text, valid):
        vcol = COLOR_WHITE if valid else COLOR_DIM
        txt_ax.text(0.06, y, label_text, color=COLOR_GRAY, fontsize=13,
                    family=MONO, ha="left", va="center")
        txt_ax.text(0.94, y, value_text, color=vcol, fontsize=13,
                    family=MONO, ha="right", va="center", weight="bold")

    vc = ctx.vc_m_s[i]
    readout_line(0.49, "CLOSING SPEED", fmt_num(vc, " m/s"), not math.isnan(vc))
    r_hat = ctx.r_hat_m[i]
    readout_line(0.415, "RANGE", fmt_num(r_hat, " m"), not math.isnan(r_hat))
    tgo = ctx.t_go(i)
    readout_line(0.34, "T-GO", fmt_num(tgo, " s", "{:.2f}"), not math.isnan(tgo))
    ldot = ctx.lambda_dot_deg_s[i]
    readout_line(0.265, "LOS RATE", fmt_num(ldot, " deg/s", "{:+.1f}"), not math.isnan(ldot))
    law = (row.get("law") or "--").upper()
    readout_line(0.19, "LAW", law, True)

    t_display = (row.get("t_sim") or "").strip()
    t_display = (t_display + "s (sim)") if t_display else (row.get("t", "") + "s (wall)")
    txt_ax.text(0.5, 0.115, f"t = {t_display}", color=COLOR_GT_REF, fontsize=10,
                family=MONO, ha="center", va="center")
    txt_ax.text(0.5, 0.06,
                "EXTERNAL CUE = mocked ground-sensor stand-in, not live hardware",
                color=COLOR_GT_REF, fontsize=8, family=MONO, ha="center", va="center")
    txt_ax.text(0.5, 0.02,
                "GT markers = scoring reference only, never fed to guidance",
                color=COLOR_GT_REF, fontsize=8, family=MONO, ha="center", va="center")

    map_ax = fig.add_axes((0.08, 0.04, 0.84, 0.50))
    draw_minimap(map_ax, ctx, i, transparent=transparent, compact=False)

    footer_ax = fig.add_axes((0.0, 0.0, 1.0, 0.035))
    footer_ax.set_xlim(0, 1)
    footer_ax.set_ylim(0, 1)
    footer_ax.axis("off")
    if transparent:
        footer_ax.patch.set_alpha(0.0)
    else:
        footer_ax.set_facecolor(COLOR_BG)
    footer_ax.text(0.5, 0.5,
                   f"render_hud.py (offline overlay, not in-sim capture) -- source: {ctx.source_name or '--'}",
                   color=COLOR_DIM, fontsize=6, family=MONO, ha="center", va="center")

    return _save(fig, out_path)


# ----------------------------------------------------------------------------
# Overlay layout (FPV/interceptor OSD, edge-mounted, centre kept clear)
# ----------------------------------------------------------------------------
def render_overlay_frame(ctx, i, out_path, transparent=False, W=OVERLAY_W, H=OVERLAY_H):
    row = ctx.rows[i]
    fig = plt.figure(figsize=(W / FIG_DPI, H / FIG_DPI), dpi=FIG_DPI)
    if transparent:
        fig.patch.set_alpha(0.0)
    else:
        fig.patch.set_facecolor(COLOR_BG)

    # Font scale relative to a 960 px-tall reference frame.
    fs = 13.0 * (H / 960.0)

    ax = fig.add_axes((0.0, 0.0, 1.0, 1.0))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    if transparent:
        ax.patch.set_alpha(0.0)
    else:
        ax.set_facecolor(COLOR_BG)

    # --- centre boresight reticle (subtle, keeps the tag readable) ---
    _boresight(ax, 0.5, 0.52, transparent)

    # --- top-left status lamp stack (PHASE / SENSOR / APRILTAG) ---
    lx, lw, lh = 0.015, 0.285, 0.043
    phase = row.get("phase", "") or "--"
    _lamp(ax, lx, 0.942, lw, lh, f"PHASE  {phase}", PHASE_COLORS.get(phase, COLOR_WHITE),
          transparent, fs * 0.92)
    label = mode_label(i, ctx.handoff_idx)
    lamp_color = COLOR_GREEN if label == "CAMERA-ONLY" else COLOR_AMBER
    sensor_txt = "SENSOR  CAMERA-ONLY" if label == "CAMERA-ONLY" else "SENSOR  EXT CUE (mock)"
    _lamp(ax, lx, 0.895, lw, lh, sensor_txt, lamp_color, transparent, fs * 0.82)
    det_i = ctx.detected_flags[i]
    if det_i is None:
        tag_text, tag_color = "APRILTAG  --", COLOR_DIM
    elif det_i == 1:
        tag_text, tag_color = "APRILTAG  LOCKED", COLOR_GREEN
    else:
        tag_text, tag_color = "APRILTAG  SEARCHING", COLOR_AMBER
    _lamp(ax, lx, 0.848, lw, lh, tag_text, tag_color, transparent, fs * 0.82)

    # --- top-centre heading tape (attitude/heading, <- psi_deg) ---
    _heading_tape(ax, 0.5, 0.945, 0.30, 0.075, ctx.psi_deg[i], transparent, fs * 0.9)

    # --- top-right T-GO hero number (time-to-intercept, camera firing solution) ---
    # The gamey "SOLUTION: CONVERGED/SOLVING" status line was removed in the
    # 2026-07-07 de-cringe pass -- T-GO ("--" until the camera has a solution)
    # already carries the "do we have a firing solution yet" story honestly.
    trx, trw, trh = 0.70, 0.285, 0.052
    _plate(ax, trx, 0.933, trw, trh, transparent, z=2)
    tgo = ctx.t_go(i)
    ax.text(trx + 0.02, 0.959, "T-GO", color=COLOR_GRAY, fontsize=fs * 0.85,
            family=MONO, ha="left", va="center", zorder=3)
    ax.text(trx + trw - 0.02, 0.957, ("--" if math.isnan(tgo) else f"{tgo:0.2f}s"),
            color=COLOR_WHITE, fontsize=fs * 1.5, family=MONO, ha="right", va="center",
            weight="bold", zorder=3)

    # --- left GND SPD gauge / right ALT gauge (flight state) ---
    gs = ctx.ground_speed[i]
    _vgauge(ax, 0.022, 0.42, 0.028, 0.30, gs, GND_SPD_MAX, "GS",
            fmt_num(gs, "", "{:.1f}") + "\nm/s", transparent, fs * 0.85)
    alt = ctx.alt_m[i]
    _vgauge(ax, 0.950, 0.42, 0.028, 0.30, alt, ALT_MAX, "ALT",
            fmt_num(alt, "", "{:.2f}") + "\nm", transparent, fs * 0.85)

    # --- bottom-left mini-map inset: REMOVED (2026-07-07 de-cringe pass) ---
    # The two-series moving-map read as a video-game minimap, not an instrument
    # (builder feedback #3a). draw_minimap() is retained for the sidebar layout.

    # --- bottom-centre firing-solution cluster: RANGE bar + readouts ---
    bx, bw = 0.335, 0.33
    _plate(ax, bx - 0.012, 0.045, bw + 0.024, 0.175, transparent, z=1)
    _range_bar(ax, bx, 0.075, bw, 0.028, ctx.r_hat_m[i], ctx.r0_range,
               ctx.radius_m, transparent, fs * 0.9)
    # three readouts in a row above the bar
    vc = ctx.vc_m_s[i]
    ldot = ctx.lambda_dot_deg_s[i]
    ax.text(bx, 0.185, "CLOSING", color=COLOR_GRAY, fontsize=fs * 0.72, family=MONO,
            ha="left", va="center", zorder=3)
    ax.text(bx, 0.160, fmt_num(vc, " m/s"), color=(COLOR_WHITE if not math.isnan(vc) else COLOR_DIM),
            fontsize=fs * 0.92, family=MONO, ha="left", va="center", weight="bold", zorder=3)
    ax.text(bx + bw, 0.185, "LOS RATE", color=COLOR_GRAY, fontsize=fs * 0.72, family=MONO,
            ha="right", va="center", zorder=3)
    ax.text(bx + bw, 0.160, fmt_num(ldot, " deg/s", "{:+.1f}"),
            color=(COLOR_WHITE if not math.isnan(ldot) else COLOR_DIM),
            fontsize=fs * 0.92, family=MONO, ha="right", va="center", weight="bold", zorder=3)
    # (The "LAW PRONAV" label was removed in the 2026-07-07 de-cringe pass --
    #  builder feedback #3c; the guidance law is stated on the title card.)

    # --- bottom-right time + honesty footnotes (understated, but non-negotiable) ---
    t_display = (row.get("t_sim") or "").strip()
    t_display = (t_display + "s sim") if t_display else ((row.get("t", "") or "--") + "s wall")
    ax.text(0.985, 0.115, f"t = {t_display}", color=COLOR_GT_REF, fontsize=fs * 0.72,
            family=MONO, ha="right", va="center", zorder=3)
    ax.text(0.985, 0.033, "EXTERNAL CUE = mocked ground sensor, not live hardware",
            color=COLOR_GT_REF, fontsize=fs * 0.55, family=MONO, ha="right", va="center", zorder=3)
    ax.text(0.985, 0.013, "GND SPD / CPA = ground truth, scoring only, never fed to guidance",
            color=COLOR_GT_REF, fontsize=fs * 0.55, family=MONO, ha="right", va="center", zorder=3)

    return _save(fig, out_path)


def _save(fig, out_path):
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())  # (H, W, 4) uint8
    plt.close(fig)
    bgra = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA)
    if not cv2.imwrite(out_path, bgra):
        raise RuntimeError(f"render_hud: cv2.imwrite failed for {out_path}")
    return rgba.shape


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("csv_path", help="a scripts/m4_intercept.py (or m3_static_intercept.py) per-tick CSV")
    ap.add_argument("--out", required=True, help="output directory for frame_*.png")
    ap.add_argument("--layout", choices=("sidebar", "overlay"), default="sidebar",
                    help="sidebar (default, beside a render) or overlay (FPV OSD over the video)")
    ap.add_argument("--width", type=int, default=OVERLAY_W, help="overlay canvas width px")
    ap.add_argument("--height", type=int, default=OVERLAY_H, help="overlay canvas height px")
    ap.add_argument("--fps", type=float, default=None,
                    help="resample to a fixed-fps sim-time grid (ZOH) instead of one frame per CSV row")
    ap.add_argument("--times-from", default=None,
                    help="render one frame per row of this capture manifest CSV (needs a sim_t "
                         "column), ZOH-sampled -- gives HUD frames index-aligned 1:1 with the capture")
    ap.add_argument("--radius", type=float, default=DEFAULT_RADIUS_M,
                    help=f"lethal-radius ring/tick, meters (ADR-0025 net={LETHAL_RADIUS_NET_M}, "
                         f"ram={LETHAL_RADIUS_RAM_M}; default {DEFAULT_RADIUS_M})")
    ap.add_argument("--transparent", action="store_true",
                    help="alpha=0 canvas (instrument plates keep a semi-opaque backing) for "
                         "compositing as an overlay layer on top of a render")
    args = ap.parse_args()

    rows = load_rows(args.csv_path)
    os.makedirs(args.out, exist_ok=True)
    ctx = HudContext(rows, args.radius, source_name=os.path.basename(args.csv_path))

    if args.times_from:
        indices = build_manifest_grid(rows, args.times_from)
    elif args.fps:
        _grid, indices = build_time_grid(rows, args.fps)
    else:
        indices = list(range(len(rows)))

    render = render_overlay_frame if args.layout == "overlay" else render_sidebar_frame
    shape = None
    for frame_num, row_idx in enumerate(indices):
        out_path = os.path.join(args.out, f"frame_{frame_num:06d}.png")
        if args.layout == "overlay":
            shape = render(ctx, row_idx, out_path, transparent=args.transparent,
                           W=args.width, H=args.height)
        else:
            shape = render(ctx, row_idx, out_path, transparent=args.transparent)

    print(
        f"[render_hud] layout={args.layout} wrote {len(indices)} frames to {args.out} "
        f"(source rows={len(rows)}, frame shape={shape}, handoff_idx={ctx.handoff_idx}, "
        f"cpa_idx={ctx.cpa_idx}, r0_range={ctx.r0_range:.2f}, transparent={args.transparent})"
    )


if __name__ == "__main__":
    main()
