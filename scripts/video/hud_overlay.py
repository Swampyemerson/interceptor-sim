#!/usr/bin/env python3
"""T25 demo-video HUD overlay renderer (docs/t25_storyboard.md, shots 3-6/7).

WHAT: reads a per-tick flight CSV written by scripts/m4_intercept.py (the
ADR-0058 markerless detect-then-track schema, e.g. the committed r2 hero
flights logs/m4_intercept_pronav_20260709T232447Z.csv / ...233852Z.csv) and
renders one annotated HUD frame per tick / per captured video frame. Like
scripts/render_hud.py (which this module IMPORTS for its palette, parsers and
instrument primitives), it is a pure OFFLINE function of logged CSV columns --
every readout traces to a column, nothing is invented, no sim required.

WHAT'S NEW vs render_hud.py's overlay layout (the T25 storyboard needs):
  * FOUR-lamp sensor-contribution stack, the storyboard's core ask:
      PHASE     <- phase
      GND CUE   <- the ADR-0010 #5 one-way handoff latch: LIVE (amber) up to
                   the latch tick, then DARK ("LINK CLOSED", dim) forever.
                   Computed STRUCTURALLY from the latch index (last
                   ext_fresh==1 row, render_hud.compute_handoff_idx), never
                   from per-row ext_* values -- so no cue element can ever
                   re-animate after handoff (honesty constraint #1).
      CAMERA    <- detected + meas_source: SEARCHING before first detection,
                   NN ACQUIRE (first NN hit) / NN RE-VAL ('drone' rows after
                   acquisition, the every-8-frames re-validation) / TRACK CSRT
                   ('track' rows) while detected==1, and an honest COAST
                   (amber) during any post-acquisition detection gap -- never
                   "TRACKING" through a gap (storyboard frame-audit rule).
      TERMINAL  <- dim "--" pre-handoff, green "CAMERA-ONLY" after the latch.
  * HANDOFF callout, shown only for a sim-time window after the latch tick:
    "GROUND LINK CLOSED -- structurally unreadable from this tick". This is
    the storyboard's honesty beat (shot 5). Deliberately NOT worded as
    "jam-proof"/"jamming-proven": mid-dash jam-denial is a HELD claim
    (ADR-0059 MC FLEW 2026-07-10 -- the config fails closed under a
    pre-acquisition jam and the staleness fix is fail-SAFE, not recovery; the
    claim stays HELD. Honesty constraint #2).
  * Seeker track box: the CSRT/NN measurement (meas_x/y/z, camera frame,
    OpenCV convention: x right, y down, z forward) projected through the
    logged camera intrinsics (fx=fy=539.936, cx=640, cy=480 @1280x960 -- the
    "[m4] Intrinsics:" line in every run log) onto the frame, with a
    track-age counter and NN confidence. Drawn ONLY on detected==1 rows.
  * SLOW-MO watermark: --slowmo T0:T1 (repeatable, sim seconds) stamps
    "SLOW MOTION (retimed)" on every frame whose sim-time falls in a window
    (honesty constraint #4: disclose retiming on screen, on EVERY retimed
    frame). --rt-label stamps "REAL-TIME 1x" on the others.
  * CPA close-out (>= the CPA tick only, no foreknowledge): "CLOSEST APPROACH
    X.XX m" + `cpa_criterion_line()`, which NAMES the kill mechanism the
    criterion radius belongs to (NET-CLASS 1.5 m, ADR-0025) and states the
    other (ram/contact 0.35 m, ADR-0084) so a net-class CPA can never read as
    a ram kill -- labeled GT/scoring-only (honesty constraints #4/#5).
  * Standing footnote every frame (honesty constraint #3): the cue that
    steered these validated flights is the calibrated MOCK ground-stereo
    stand-in (real maneuvering stereo caches not built; T20/T21 pending).
  * hud_audit.csv beside the frames: one row per emitted frame (tick, lamps,
    slow-mo flag, key readouts) so the storyboard's pre-publish frame audit
    ("no cue element lit after the latch on ANY frame") is a grep, not an
    eyeball pass.

HONESTY GUARANTEES (checked at review against docs/t25_storyboard.md):
  - After the latch tick no cue-derived value is rendered anywhere: the GND
    CUE panel content is only drawn inside the `i <= handoff_idx` branch.
  - gt_* is used ONLY for the CPA close-out, the labeled dim "GT RNG
    (scoring)" reference readout, and the GS gauge (same GT-derived,
    display-only precedent as render_hud.py) -- all labeled on screen.
  - NaN/blank cells render as "--"/empty, never 0 (render_hud.parse_float).
  - No caption anywhere says or implies "jam-proof" -- do not add one.

THREE FRAME-SELECTION MODES (mutually exclusive):
  --frames MANIFEST   composite the HUD over captured video frames: one output
                      frame per manifest row (idx,wall_t,sim_t,file -- the
                      scripts/demo_capture_frames.py format), flight CSV
                      zero-order-hold-sampled at each sim_t. THE frames and
                      THE CSV must be from the SAME flight (build_demo.py
                      discipline) -- rendering committed r2 CSVs over a fresh
                      re-fly's frames would be a silent lie; use the re-fly's
                      own CSV.
  --fps N             HUD-only frames on a fixed sim-time grid (or composite
                      base --fps with --transparent overlay layers for ffmpeg).
  --times "a,b,c"     just those sim-times (quick sample frames / frame audit).
  (default: one frame per CSV row)

USAGE (offline, main .venv):
  .venv/bin/python scripts/video/hud_overlay.py logs/m4_intercept_pronav_20260709T232447Z.csv \
      --out demo_out/t25/hud_run1 --fps 30 --slowmo 22.9:23.3
  # over captured frames, as a transparent layer or composited:
  .venv/bin/python scripts/video/hud_overlay.py <refly_flight.csv> \
      --out demo_out/t25/shot36_frames --frames demo_out/t25/onboard_frames/manifest.csv \
      --slowmo <t_cpa-0.35>:<t_cpa+0.1>
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")  # offline/headless always (docs/goals.md)
import matplotlib.pyplot as plt
import cv2

SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(SCRIPTS_DIR)
sys.path.insert(0, SCRIPTS_DIR)
import render_hud as rh  # noqa: E402  palette, parsers, instrument primitives

# Camera intrinsics for the track-box projection: the values every markerless
# run logs at boot ("[m4] Intrinsics: fx=539.936 fy=539.936 cx=640.0 cy=480.0
# (1280x960)", e.g. logs/mc_batch_run_1_20260709T232429Z.log). CLI-overridable
# if a future capture uses a different camera.
FX_DEFAULT = 539.936
FY_DEFAULT = 539.936
CX_DEFAULT = 640.0
CY_DEFAULT = 480.0

TARGET_HALF_SPAN_M = 0.35   # FPV-target half-span for the presentational box
                            # size (box CENTER is the logged measurement; the
                            # box EXTENT is a fixed-metric visual aid, stated
                            # here, not a measured bounding box).

# Storyboard-mandated wording (docs/t25_storyboard.md). Do not "improve" these
# into stronger claims -- constraint #2 explicitly HOLDS the jam-denial claim.
HANDOFF_LINE_1 = "HANDOFF -- GROUND LINK CLOSED (one-way latch)"
HANDOFF_LINE_2 = "cue structurally unreadable from this tick -- terminal is camera-only"
FOOTNOTE_CUE = "flown cue = calibrated MOCK ground-stereo stand-in (maneuvering stereo validation pending, T20/T21)"
FOOTNOTE_GT = "GT RNG / GS / CPA = ground truth, scoring only, never fed to guidance"
SLOWMO_TEXT = "SLOW MOTION (retimed)"
RT_TEXT = "REAL-TIME 1x"


def cpa_criterion_line(radius_m):
    """The CPA close-out sub-caption. NAMES the kill mechanism the criterion
    radius belongs to, and states the OTHER one, so a viewer can never read a
    net-class CPA as a ram/contact kill (ADR-0066 claims-scrub applied to the
    reel, ADR-0084: the ram radius is 0.35 m from the ordered 5-inch pair's
    contact envelope -- it RETIRES the old 7-inch 0.5 m figure; the 1.5 m
    net-class radius is a different mechanism and is unchanged)."""
    net = f"{rh.LETHAL_RADIUS_NET_M:.1f}"
    ram = f"{rh.LETHAL_RADIUS_RAM_M:.2f}"
    if abs(radius_m - rh.LETHAL_RADIUS_RAM_M) < 1e-6:
        shown = f"RAM/CONTACT criterion R={ram} m (ADR-0084); net-class radius {net} m"
    elif abs(radius_m - rh.LETHAL_RADIUS_NET_M) < 1e-6:
        shown = f"NET-CLASS criterion R={net} m (ADR-0025); ram/contact radius {ram} m (ADR-0084)"
    else:
        shown = (f"criterion R={rh.fmt_radius(radius_m)} m -- net-class {net} m / "
                 f"ram {ram} m (ADR-0025/0084)")
    return shown + " -- not a modeled collision; GT, scoring only"


# ---------------------------------------------------------------------------
# context: everything per-flight, precomputed once
# ---------------------------------------------------------------------------
class T25Context(rh.HudContext):
    """render_hud.HudContext + the detect-then-track / T25 extras."""

    def __init__(self, rows, radius_m, source_name=""):
        super().__init__(rows, radius_m, source_name)
        self.meas_source = [(r.get("meas_source") or "").strip() for r in rows]
        self.meas_conf = rh.col_floats(rows, "meas_conf")
        self.meas_x = rh.col_floats(rows, "meas_x")
        self.meas_y = rh.col_floats(rows, "meas_y")
        self.meas_z = rh.col_floats(rows, "meas_z")
        self.ext_age_s = rh.col_floats(rows, "ext_age_s")

        # first NN acquisition tick (detected flips to 1 the first time)
        self.first_det_idx = None
        for i, d in enumerate(self.detected_flags):
            if d == 1:
                self.first_det_idx = i
                break

        # track age: sim-seconds since the start of the current detected==1
        # streak (0 when not detected). An honest per-streak counter -- it
        # resets to 0 after every gap rather than pretending continuity.
        self.track_age_s = np.zeros(self.n)
        streak_t0 = None
        for i in range(self.n):
            if self.detected_flags[i] == 1:
                if streak_t0 is None:
                    streak_t0 = self.t_sim[i]
                if not math.isnan(self.t_sim[i]) and not math.isnan(streak_t0):
                    self.track_age_s[i] = self.t_sim[i] - streak_t0
            else:
                streak_t0 = None

        self.handoff_t = (float(self.t_sim[self.handoff_idx])
                          if self.handoff_idx is not None
                          and not math.isnan(self.t_sim[self.handoff_idx]) else None)
        self.cpa_t = (float(self.t_sim[self.cpa_idx])
                      if self.cpa_idx is not None
                      and not math.isnan(self.t_sim[self.cpa_idx]) else None)
        self.cpa_m = (float(self.gt_range[self.cpa_idx])
                      if self.cpa_idx is not None else float("nan"))

    # -- lamp state functions: each returns (text, color) ------------------
    def cue_lamp(self, i):
        """One-way by construction: a pure function of i vs handoff_idx."""
        if self.handoff_idx is None:
            return "GND CUE  --", rh.COLOR_DIM
        if i <= self.handoff_idx:
            return "GND CUE  LIVE", rh.COLOR_AMBER
        return "GND CUE  LINK CLOSED", rh.COLOR_DIM

    def camera_lamp(self, i):
        det = self.detected_flags[i]
        if det is None:
            return "CAMERA  --", rh.COLOR_DIM
        if self.first_det_idx is None or i < self.first_det_idx:
            return "CAMERA  SEARCHING", rh.COLOR_AMBER
        if det == 1:
            src = self.meas_source[i]
            if src == "drone":
                return ("CAMERA  NN ACQUIRE" if i == self.first_det_idx
                        else "CAMERA  NN RE-VAL"), rh.COLOR_GREEN
            if src == "track":
                return "CAMERA  TRACK CSRT", rh.COLOR_GREEN
            return "CAMERA  LOCKED", rh.COLOR_GREEN   # apriltag-schema flights
        return "CAMERA  COAST", rh.COLOR_AMBER        # honest gap, never "TRACKING"

    def terminal_lamp(self, i):
        if self.handoff_idx is not None and i > self.handoff_idx:
            return "TERMINAL  CAMERA-ONLY", rh.COLOR_GREEN
        return "TERMINAL  --", rh.COLOR_DIM


# ---------------------------------------------------------------------------
# frame renderer
# ---------------------------------------------------------------------------
def render_frame(ctx, i, out_path, W, H, transparent, slowmo, rt_label,
                 intr, handoff_window_s, base_img=None):
    """Render tick i. base_img (BGR) -> HUD composited over it; else opaque
    dark canvas (or alpha=0 with --transparent). Returns the audit record."""
    row = ctx.rows[i]
    fig = plt.figure(figsize=(W / rh.FIG_DPI, H / rh.FIG_DPI), dpi=rh.FIG_DPI)
    fig.patch.set_alpha(0.0)
    ax = fig.add_axes((0.0, 0.0, 1.0, 1.0))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.patch.set_alpha(0.0)
    fs = 13.0 * (H / 960.0)
    # instruments always draw their semi-opaque plates (we composite ourselves)
    tp = True

    rh._boresight(ax, 0.5, 0.52, tp)

    # --- top-left: the four-lamp sensor-contribution stack ---
    lx, lw_, lh_ = 0.015, 0.285, 0.040
    phase = row.get("phase", "") or "--"
    lamps = [
        (f"PHASE  {phase}", rh.PHASE_COLORS.get(phase, rh.COLOR_WHITE)),
        ctx.cue_lamp(i),
        ctx.camera_lamp(i),
        ctx.terminal_lamp(i),
    ]
    y = 0.945
    for text, color in lamps:
        rh._lamp(ax, lx, y, lw_, lh_, text, color, tp, fs * 0.80)
        y -= 0.044
    # cue age readout: PRE-HANDOFF ONLY (structurally inside the latch branch;
    # honesty constraint #1 -- nothing cue-derived may render after the latch)
    if ctx.handoff_idx is not None and i <= ctx.handoff_idx:
        age = ctx.ext_age_s[i]
        if not math.isnan(age):
            ax.text(lx + 0.004, y + 0.030, f"cue age {age:0.2f}s",
                    color=rh.COLOR_GRAY, fontsize=fs * 0.62, family=rh.MONO,
                    ha="left", va="center", zorder=3)

    # --- top-centre heading tape / top-right T-GO (render_hud instruments) ---
    rh._heading_tape(ax, 0.5, 0.945, 0.30, 0.075, ctx.psi_deg[i], tp, fs * 0.9)
    trx, trw, trh = 0.70, 0.285, 0.052
    rh._plate(ax, trx, 0.933, trw, trh, tp, z=2)
    tgo = ctx.t_go(i)
    ax.text(trx + 0.02, 0.959, "T-GO", color=rh.COLOR_GRAY, fontsize=fs * 0.85,
            family=rh.MONO, ha="left", va="center", zorder=3)
    ax.text(trx + trw - 0.02, 0.957, ("--" if math.isnan(tgo) else f"{tgo:0.2f}s"),
            color=rh.COLOR_WHITE, fontsize=fs * 1.5, family=rh.MONO,
            ha="right", va="center", weight="bold", zorder=3)

    # --- side gauges: GS (GT-derived, labeled) and ALT ---
    gs = ctx.ground_speed[i]
    rh._vgauge(ax, 0.022, 0.42, 0.028, 0.30, gs, rh.GND_SPD_MAX, "GS",
               rh.fmt_num(gs, "", "{:.1f}") + "\nm/s", tp, fs * 0.85)
    alt = ctx.alt_m[i]
    rh._vgauge(ax, 0.950, 0.42, 0.028, 0.30, alt, rh.ALT_MAX, "ALT",
               rh.fmt_num(alt, "", "{:.2f}") + "\nm", tp, fs * 0.85)

    # --- seeker track box (detected rows only; centre = logged measurement) ---
    det = ctx.detected_flags[i]
    if det == 1 and not math.isnan(ctx.meas_z[i]) and ctx.meas_z[i] > 0.1:
        fx, fy, cx_i, cy_i = intr
        u = cx_i + fx * ctx.meas_x[i] / ctx.meas_z[i]
        v = cy_i + fy * ctx.meas_y[i] / ctx.meas_z[i]
        # normalised (matplotlib y-up); intrinsics are for 1280x960 -- scale
        xn, yn = (u / 1280.0), 1.0 - (v / 960.0)
        half_px = max(fx * TARGET_HALF_SPAN_M / ctx.meas_z[i], 12.0)
        hxn, hyn = half_px / 1280.0, half_px / 960.0
        if -0.2 < xn < 1.2 and -0.2 < yn < 1.2:
            src = ctx.meas_source[i]
            box_col = rh.COLOR_GREEN if src == "track" else rh.COLOR_WHITE
            ax.add_patch(plt.Rectangle((xn - hxn, yn - hyn), 2 * hxn, 2 * hyn,
                                       fill=False, edgecolor=box_col,
                                       linewidth=1.6, zorder=5))
            conf = ctx.meas_conf[i]
            tag = {"track": f"TRK {ctx.track_age_s[i]:0.2f}s",
                   "drone": ("NN ACQUIRE" if i == ctx.first_det_idx else "NN RE-VAL")
                   }.get(src, "LOCK")
            if not math.isnan(conf):
                tag += f"  c={conf:0.2f}"
            # label above the box, unless the box is high in frame (would
            # collide with the heading tape / retiming pill) -- then below
            if yn + hyn > 0.80:
                ty, tva = yn - hyn - 0.018, "top"
            else:
                ty, tva = yn + hyn + 0.018, "bottom"
            ax.text(xn, ty, tag, color=box_col,
                    fontsize=fs * 0.72, family=rh.MONO, ha="center",
                    va=tva, weight="bold", zorder=5)

    # --- bottom-centre firing-solution cluster (camera-sourced only) ---
    bx, bw = 0.335, 0.33
    rh._plate(ax, bx - 0.012, 0.045, bw + 0.024, 0.175, tp, z=1)
    rh._range_bar(ax, bx, 0.075, bw, 0.028, ctx.r_hat_m[i], ctx.r0_range,
                  ctx.radius_m, tp, fs * 0.9)
    vc = ctx.vc_m_s[i]
    ldot = ctx.lambda_dot_deg_s[i]
    ax.text(bx, 0.185, "CLOSING", color=rh.COLOR_GRAY, fontsize=fs * 0.72,
            family=rh.MONO, ha="left", va="center", zorder=3)
    ax.text(bx, 0.160, rh.fmt_num(vc, " m/s"),
            color=(rh.COLOR_WHITE if not math.isnan(vc) else rh.COLOR_DIM),
            fontsize=fs * 0.92, family=rh.MONO, ha="left", va="center",
            weight="bold", zorder=3)
    ax.text(bx + bw, 0.185, "LOS RATE", color=rh.COLOR_GRAY, fontsize=fs * 0.72,
            family=rh.MONO, ha="right", va="center", zorder=3)
    ax.text(bx + bw, 0.160, rh.fmt_num(ldot, " deg/s", "{:+.1f}"),
            color=(rh.COLOR_WHITE if not math.isnan(ldot) else rh.COLOR_DIM),
            fontsize=fs * 0.92, family=rh.MONO, ha="right", va="center",
            weight="bold", zorder=3)

    # --- HANDOFF callout: latch tick .. latch + window (sim time) ---
    show_handoff = (ctx.handoff_t is not None
                    and not math.isnan(ctx.t_sim[i])
                    and ctx.handoff_t <= ctx.t_sim[i] <= ctx.handoff_t + handoff_window_s
                    and i > ctx.handoff_idx)
    if show_handoff:
        rh._plate(ax, 0.16, 0.60, 0.68, 0.085, tp, edge=rh.COLOR_GREEN, lw=1.8, z=4)
        ax.text(0.5, 0.662, HANDOFF_LINE_1, color=rh.COLOR_GREEN, fontsize=fs * 1.05,
                family=rh.MONO, ha="center", va="center", weight="bold", zorder=5)
        ax.text(0.5, 0.628, HANDOFF_LINE_2, color=rh.COLOR_WHITE, fontsize=fs * 0.75,
                family=rh.MONO, ha="center", va="center", zorder=5)

    # --- CPA close-out: from the CPA tick onward only (no foreknowledge) ---
    show_cpa = ctx.cpa_idx is not None and i >= ctx.cpa_idx and not math.isnan(ctx.cpa_m)
    if show_cpa:
        rh._plate(ax, 0.14, 0.70, 0.72, 0.105, tp, edge=rh.COLOR_RED, lw=1.8, z=4)
        ax.text(0.5, 0.772, f"CLOSEST APPROACH  {ctx.cpa_m:0.2f} m",
                color=rh.COLOR_RED, fontsize=fs * 1.25, family=rh.MONO,
                ha="center", va="center", weight="bold", zorder=5)
        ax.text(0.5, 0.735, cpa_criterion_line(ctx.radius_m),
                color=rh.COLOR_GRAY, fontsize=fs * 0.68, family=rh.MONO,
                ha="center", va="center", zorder=5)

    # --- retiming disclosure pill (constraint #4) ---
    pill = None
    if slowmo:
        pill = (SLOWMO_TEXT, rh.COLOR_AMBER)
    elif rt_label:
        pill = (RT_TEXT, rh.COLOR_GRAY)
    if pill:
        rh._plate(ax, 0.375, 0.856, 0.25, 0.040, tp, z=4)
        ax.text(0.5, 0.876, pill[0], color=pill[1], fontsize=fs * 0.80,
                family=rh.MONO, ha="center", va="center", weight="bold", zorder=5)

    # --- bottom-right: sim clock, labeled GT range reference, footnotes ---
    t_disp = (row.get("t_sim") or "").strip()
    t_disp = (t_disp + "s sim") if t_disp else ((row.get("t", "") or "--") + "s wall")
    ax.text(0.985, 0.135, f"t = {t_disp}", color=rh.COLOR_GT_REF, fontsize=fs * 0.72,
            family=rh.MONO, ha="right", va="center", zorder=3)
    gtr = ctx.gt_range[i]
    ax.text(0.985, 0.105, f"GT RNG {rh.fmt_num(gtr, ' m')} (scoring)",
            color=rh.COLOR_GT_REF, fontsize=fs * 0.62, family=rh.MONO,
            ha="right", va="center", zorder=3)
    ax.text(0.985, 0.033, FOOTNOTE_CUE, color=rh.COLOR_GT_REF, fontsize=fs * 0.55,
            family=rh.MONO, ha="right", va="center", zorder=3)
    ax.text(0.985, 0.013, FOOTNOTE_GT, color=rh.COLOR_GT_REF, fontsize=fs * 0.55,
            family=rh.MONO, ha="right", va="center", zorder=3)

    # rasterise the overlay, then composite
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())
    plt.close(fig)
    ov_bgra = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA)

    if base_img is not None:
        if base_img.shape[:2] != (H, W):
            base_img = cv2.resize(base_img, (W, H))
        out = _alpha_over(base_img, ov_bgra)
    elif transparent:
        out = ov_bgra
    else:
        canvas = np.zeros((H, W, 3), np.uint8)
        canvas[:] = _hex_bgr(rh.COLOR_BG)
        out = _alpha_over(canvas, ov_bgra)
    if not cv2.imwrite(out_path, out):
        raise RuntimeError(f"hud_overlay: cv2.imwrite failed for {out_path}")

    return {  # audit record (see module docstring)
        "row_idx": i, "t_sim": row.get("t_sim", ""), "phase": phase,
        "cue_lamp": lamps[1][0], "camera_lamp": lamps[2][0],
        "terminal_lamp": lamps[3][0], "slowmo": int(bool(slowmo)),
        "handoff_callout": int(show_handoff), "cpa_overlay": int(show_cpa),
        "detected": "" if det is None else det,
        "meas_source": ctx.meas_source[i],
        "r_hat_m": row.get("r_hat_m", ""), "vc_m_s": row.get("vc_m_s", ""),
        "gt_range": row.get("gt_range", ""),
    }


def _hex_bgr(h):
    h = h.lstrip("#")
    return (int(h[4:6], 16), int(h[2:4], 16), int(h[0:2], 16))


def _alpha_over(bg_bgr, ov_bgra):
    if ov_bgra.shape[:2] != bg_bgr.shape[:2]:
        ov_bgra = cv2.resize(ov_bgra, (bg_bgr.shape[1], bg_bgr.shape[0]))
    a = ov_bgra[:, :, 3:4].astype(np.float32) / 255.0
    out = ov_bgra[:, :, :3].astype(np.float32) * a + bg_bgr.astype(np.float32) * (1 - a)
    return out.astype(np.uint8)


# ---------------------------------------------------------------------------
# frame selection
# ---------------------------------------------------------------------------
def parse_windows(specs):
    wins = []
    for s in specs or []:
        try:
            a, b = s.split(":")
            wins.append((float(a), float(b)))
        except ValueError:
            raise SystemExit(f"hud_overlay: bad --slowmo window '{s}' (want T0:T1)")
    return wins


def in_windows(t, wins):
    return (not math.isnan(t)) and any(a <= t <= b for a, b in wins)


def select_times(ctx, times_str):
    clock = ctx.t_sim
    valid = np.where(~np.isnan(clock))[0]
    if len(valid) == 0:
        raise SystemExit("hud_overlay: flight CSV has no t_sim")
    cl = list(clock[valid])
    import bisect
    out = []
    for tok in times_str.split(","):
        t = float(tok)
        pos = max(bisect.bisect_right(cl, t) - 1, 0)
        out.append(int(valid[pos]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path", help="per-tick flight CSV (scripts/m4_intercept.py schema)")
    ap.add_argument("--out", required=True, help="output dir for frame_*.png + hud_audit.csv")
    ap.add_argument("--frames", default=None,
                    help="capture manifest.csv (idx,wall_t,sim_t,file) from the SAME flight; "
                         "composites the HUD over each captured frame, index-aligned")
    ap.add_argument("--fps", type=float, default=None, help="fixed sim-time grid instead of per-row")
    ap.add_argument("--times", default=None, help="comma-separated sim-times, one frame each")
    ap.add_argument("--width", type=int, default=rh.OVERLAY_W)
    ap.add_argument("--height", type=int, default=rh.OVERLAY_H)
    ap.add_argument("--radius", type=float, default=rh.DEFAULT_RADIUS_M,
                    help=f"lethal-radius criterion, m (default {rh.DEFAULT_RADIUS_M}, ADR-0025 net)")
    ap.add_argument("--transparent", action="store_true",
                    help="no --frames: emit alpha-0 overlay layers for an ffmpeg composite")
    ap.add_argument("--slowmo", action="append", default=[],
                    help="sim-time window T0:T1 stamped 'SLOW MOTION (retimed)'; repeatable")
    ap.add_argument("--rt-label", action="store_true",
                    help="stamp 'REAL-TIME 1x' on frames outside every --slowmo window")
    ap.add_argument("--handoff-window-s", type=float, default=0.8,
                    help="sim-seconds the HANDOFF callout stays up after the latch (default 0.8)")
    ap.add_argument("--fx", type=float, default=FX_DEFAULT)
    ap.add_argument("--fy", type=float, default=FY_DEFAULT)
    ap.add_argument("--cx", type=float, default=CX_DEFAULT)
    ap.add_argument("--cy", type=float, default=CY_DEFAULT)
    args = ap.parse_args()

    if sum(x is not None for x in (args.frames, args.fps, args.times)) > 1:
        raise SystemExit("hud_overlay: --frames / --fps / --times are mutually exclusive")

    rows = rh.load_rows(args.csv_path)
    ctx = T25Context(rows, args.radius, source_name=os.path.basename(args.csv_path))
    os.makedirs(args.out, exist_ok=True)
    wins = parse_windows(args.slowmo)
    intr = (args.fx, args.fy, args.cx, args.cy)

    # ---- pick (row_idx, base_image_path or None) per output frame ----
    plan = []
    if args.frames:
        man_dir = os.path.dirname(os.path.abspath(args.frames))
        with open(args.frames, newline="") as f:
            man = list(csv.DictReader(f))
        if not man or "sim_t" not in man[0] or "file" not in man[0]:
            raise SystemExit("hud_overlay: manifest needs sim_t and file columns "
                             "(scripts/demo_capture_frames.py format)")
        idxs = rh.build_manifest_grid(rows, args.frames)
        for m, ri in zip(man, idxs):
            plan.append((ri, os.path.join(man_dir, m["file"])))
    elif args.fps:
        _grid, idxs = rh.build_time_grid(rows, args.fps)
        plan = [(ri, None) for ri in idxs]
    elif args.times:
        plan = [(ri, None) for ri in select_times(ctx, args.times)]
    else:
        plan = [(ri, None) for ri in range(len(rows))]

    W, H = args.width, args.height
    audit_path = os.path.join(args.out, "hud_audit.csv")
    audit_cols = ["frame", "row_idx", "t_sim", "phase", "cue_lamp", "camera_lamp",
                  "terminal_lamp", "slowmo", "handoff_callout", "cpa_overlay",
                  "detected", "meas_source", "r_hat_m", "vc_m_s", "gt_range"]
    n_cue_after_latch = 0
    with open(audit_path, "w", newline="") as af:
        aw = csv.DictWriter(af, fieldnames=audit_cols)
        aw.writeheader()
        for k, (ri, img_path) in enumerate(plan):
            base = None
            if img_path is not None:
                base = cv2.imread(img_path)
                if base is None:
                    raise SystemExit(f"hud_overlay: cannot read capture frame {img_path}")
            t_i = ctx.t_sim[ri]
            rec = render_frame(ctx, ri, os.path.join(args.out, f"frame_{k:06d}.png"),
                               W, H, args.transparent, in_windows(t_i, wins),
                               args.rt_label, intr, args.handoff_window_s, base)
            rec["frame"] = k
            aw.writerow(rec)
            if (ctx.handoff_idx is not None and ri > ctx.handoff_idx
                    and "LIVE" in rec["cue_lamp"]):
                n_cue_after_latch += 1  # structurally impossible; belt+braces

    # ---- beats summary (for picking shot boundaries / slow-mo windows) ----
    def _fmt_t(idx):
        return "--" if idx is None else f"t_sim={ctx.t_sim[idx]:.3f}s (row {idx})"
    print(f"[hud_overlay] source: {ctx.source_name}  frames: {len(plan)} -> {args.out}")
    print(f"[hud_overlay] first NN acquisition: {_fmt_t(ctx.first_det_idx)}"
          + (f"  range={rows[ctx.first_det_idx].get('meas_range','--')} m"
             if ctx.first_det_idx is not None else ""))
    print(f"[hud_overlay] handoff latch (last ext_fresh==1): {_fmt_t(ctx.handoff_idx)}")
    print(f"[hud_overlay] CPA: {_fmt_t(ctx.cpa_idx)}  gt_range={ctx.cpa_m:.3f} m "
          f"(criterion radius {ctx.radius_m:.1f} m)")
    print(f"[hud_overlay] audit CSV: {audit_path}")
    if n_cue_after_latch:
        print(f"[hud_overlay] HONESTY VIOLATION: {n_cue_after_latch} frames drew a live "
              "cue lamp after the latch -- DO NOT SHIP", file=sys.stderr)
        return 1
    print("[hud_overlay] honesty check: 0 frames with a live cue element after the latch")
    return 0


if __name__ == "__main__":
    sys.exit(main())
