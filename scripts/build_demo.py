#!/usr/bin/env python3
"""Assemble the portfolio DEMO VIDEO from the captured hero-flight frames.

WHAT: builds the finished ONBOARD hero cut (the seeker POV + the FPV OSD HUD
overlaid, ending on the proximity-fuse close-out) as the PRIMARY video, plus a
brief CHASE/wide secondary cut, opened by a short title card (no outro). OFFLINE --
no Gazebo/GPU; the 700+ frames are already captured (demo_out/onboard_frames,
chase_frames + their manifest.csv) and the flight is the ADR-0032/0033 hero CSV
(logs/m4_intercept_pronav_20260707T211601Z.csv, miss 0.632 m).

HOW IT STAYS HONEST (project non-negotiables, docs/demo_plan.md rules):
  - The HUD is scripts/render_hud.py's overlay layout -- a pure function of the
    logged CSV columns; every readout traces to data (see that file).
  - The onboard footage is retimed for watchability (the whole engagement is
    ~0.3 s of sim time). Every speed change is DISCLOSED with an on-screen
    "REAL-TIME" / "SLOW-MO" tag, and slow-mo in-betweens are honest cross-
    dissolves of adjacent captured frames, never invented motion.
  - The proximity-fuse close-out cites the REAL trigger number: CPA (minimum
    ground-truth range) 0.632 m < the 1.5 m net lethal radius (ADR-0025), and
    is explicitly labelled a CRITERION, not a modeled blast -- the sim has no
    collision/blast volume (ADR-0014). The ring/flash is a stylised graphic.
  - gt_* (used only for the CPA number + the map's reference markers) is
    scoring-only and never fed to guidance (CLAUDE.md honesty boundary).

LAYERS KEPT FOR RE-EDIT (Premiere): demo_out/onboard_frames (seeker POV),
demo_out/chase_frames (wide), demo_out/hud_onboard_transparent +
hud_chase_transparent (the alpha HUD overlay layers), and the assembled
frame dirs. The final encode is ffmpeg.

USAGE:  .venv/bin/python scripts/build_demo.py [--fast]
  --fast: skip the GIF and the chase cut (quicker iteration on the onboard look).
"""
import argparse
import csv
import math
import os
import shutil
import subprocess
import sys

import cv2
import numpy as np

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)
import render_hud as rh  # noqa: E402  (palette, MONO, HudContext, overlay renderer)

HERO_CSV = os.path.join(REPO, "logs", "m4_intercept_pronav_20260707T211601Z.csv")
DEMO = os.path.join(REPO, "demo_out")
ONBOARD_DIR = os.path.join(DEMO, "onboard_frames")
CHASE_DIR = os.path.join(DEMO, "chase_frames")
FPS = 30
RADIUS = rh.LETHAL_RADIUS_NET_M  # 1.5 m net (ADR-0025) -- the CPA criterion radius

# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def _hex_bgr(h):
    h = h.lstrip("#")
    return (int(h[4:6], 16), int(h[2:4], 16), int(h[0:2], 16))


def load_manifest(path):
    """idx -> sim_t and idx -> filepath from a capture manifest.csv."""
    man = list(csv.DictReader(open(path)))
    simt = {int(r["idx"]): float(r["sim_t"]) for r in man}
    files = {int(r["idx"]): os.path.join(os.path.dirname(path), r["file"]) for r in man}
    return simt, files


def nearest_idx(simt, t):
    return min(simt, key=lambda k: abs(simt[k] - t))


def alpha_over(bg_bgr, ov_bgra):
    """Composite a BGRA overlay onto a BGR frame (both same HxW)."""
    if ov_bgra.shape[:2] != bg_bgr.shape[:2]:
        ov_bgra = cv2.resize(ov_bgra, (bg_bgr.shape[1], bg_bgr.shape[0]))
    a = ov_bgra[:, :, 3:4].astype(np.float32) / 255.0
    out = ov_bgra[:, :, :3].astype(np.float32) * a + bg_bgr.astype(np.float32) * (1 - a)
    return out.astype(np.uint8)


def blend(a_bgr, b_bgr, f):
    """Linear cross-dissolve a->b, f in [0,1]."""
    return cv2.addWeighted(a_bgr, 1.0 - f, b_bgr, f, 0.0)


# ---------------------------------------------------------------------------
# matplotlib text/graphic layers (monospace, matches the HUD aesthetic)
# ---------------------------------------------------------------------------
def text_layer(items, W, H, bg=None):
    """Render text items onto a W×H canvas. items = list of dicts with keys
    x,y (normalised, y up), text, color, size, ha (default center), weight
    (default normal). bg=None -> transparent; else a hex fill. Returns BGRA."""
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(W / rh.FIG_DPI, H / rh.FIG_DPI), dpi=rh.FIG_DPI)
    if bg is None:
        fig.patch.set_alpha(0.0)
    else:
        fig.patch.set_facecolor(bg)
    ax = fig.add_axes((0, 0, 1, 1)); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    if bg is None:
        ax.patch.set_alpha(0.0)
    else:
        ax.set_facecolor(bg)
    for it in items:
        if it.get("plate"):
            x, y, w, h = it["plate"]
            ax.add_patch(plt.Rectangle((x, y), w, h, facecolor=it.get("plate_fc", rh.COLOR_BG),
                                       alpha=it.get("plate_alpha", 0.6),
                                       edgecolor=it.get("plate_ec", rh.COLOR_PANEL_LINE),
                                       linewidth=it.get("plate_lw", 1.2), zorder=1))
        if "text" in it:
            ax.text(it["x"], it["y"], it["text"], color=it["color"],
                    fontsize=it["size"] * (H / 960.0), family=rh.MONO,
                    ha=it.get("ha", "center"), va=it.get("va", "center"),
                    weight=it.get("weight", "normal"), zorder=2,
                    linespacing=it.get("linespacing", 1.4))
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())
    plt.close(fig)
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA)


def title_card(W, H):
    # Short: a one-line headline (the intercept name) + one understated honesty
    # line -- nothing lecture-y (builder feedback #4). The criterion-not-collision
    # honesty is carried in-scene by the proximity-fuse banner at the CPA hold.
    items = [
        {"x": 0.5, "y": 0.575, "text": "CAMERA-ONLY  PROPORTIONAL-NAVIGATION",
         "color": rh.COLOR_WHITE, "size": 20, "weight": "bold"},
        {"x": 0.5, "y": 0.50, "text": "COUNTER-UAS  INTERCEPT",
         "color": rh.COLOR_GREEN, "size": 26, "weight": "bold"},
        {"x": 0.5, "y": 0.395,
         "text": "PX4 / Gazebo SITL    -    simulation only; every readout traces to a logged CSV column",
         "color": rh.COLOR_GT_REF, "size": 10},
    ]
    layer = text_layer(items, W, H, bg=rh.COLOR_BG)
    return layer[:, :, :3]


# NOTE: the outro/metrics "RESULTS" card was REMOVED (2026-07-07, builder
# feedback #4 -- "I don't really need the screen at the end explaining the
# interceptor"). The video now ends on the proximity-fuse CPA hold and cuts to
# black. The results narrative lives in the README / docs, not the demo reel.


def fuse_banner(W, H, cpa_m):
    """The DETONATE text banner (transparent BGRA) -- composited over the CPA
    frame with a fade. Honest: cites CPA vs lethal radius as a CRITERION."""
    items = [
        {"plate": (0.14, 0.70, 0.72, 0.145), "plate_fc": "#2a0806", "plate_alpha": 0.62,
         "plate_ec": rh.COLOR_RED, "plate_lw": 2.0},
        {"x": 0.5, "y": 0.792, "text": "PROXIMITY FUSE  —  DETONATE",
         "color": rh.COLOR_RED, "size": 22, "weight": "bold"},
        {"x": 0.5, "y": 0.737, "text": f"CPA {cpa_m:.2f} m  <  {RADIUS:.1f} m lethal radius",
         "color": rh.COLOR_AMBER, "size": 14, "weight": "bold"},
        {"x": 0.5, "y": 0.710, "text": "ADR-0025 criterion  --  not a modeled blast (no collision volume in sim)",
         "color": rh.COLOR_GRAY, "size": 9.5},
    ]
    return text_layer(items, W, H)


def draw_fuse(base_bgr, banner_bgra, progress, center):
    """De-cringed proximity-fuse close-out (2026-07-07, builder feedback #3):
    ONE thin static red criterion ring on the aimpoint + the fading DETONATE
    banner. Removed the gamey stuff: the white trigger flash, the expanding
    shockwave ring, and the amber second concentric ring. Reads like an
    instrument, not a video game. progress in [0,1] across the hold; center=
    (cx,cy) px (the tag centroid / criterion-ring centre)."""
    H, W = base_bgr.shape[:2]
    out = base_bgr.copy()
    cx, cy = center
    red = _hex_bgr(rh.COLOR_RED)

    # single thin criterion ring -- quick fade-in, then steady. Marks the
    # lethal-radius CRITERION around the tag (stylised; there is no modeled
    # blast volume in the sim -- the banner states this explicitly).
    rf = min(1.0, progress / 0.15)
    if rf > 0:
        r = int(0.34 * min(W, H))
        ring = out.copy()
        cv2.circle(ring, (cx, cy), r, red, thickness=2, lineType=cv2.LINE_AA)
        out = cv2.addWeighted(ring, rf, out, 1.0 - rf, 0.0)

    # banner fades in over the first ~0.2 then holds
    bf = min(1.0, progress / 0.2)
    if bf > 0:
        faded = banner_bgra.copy()
        faded[:, :, 3] = (faded[:, :, 3].astype(np.float32) * bf).astype(np.uint8)
        out = alpha_over(out, faded)
    return out


def disclosure_pill(W, H, text, color):
    """A small centred playback-rate disclosure pill, clear below the heading tape."""
    items = [{"plate": (0.375, 0.856, 0.25, 0.040), "plate_alpha": 0.6},
             {"x": 0.5, "y": 0.876, "text": text, "color": color, "size": 10.5, "weight": "bold"}]
    return text_layer(items, W, H)


def callout(W, H, text, color):
    items = [{"plate": (0.20, 0.585, 0.60, 0.075), "plate_alpha": 0.62,
              "plate_ec": color, "plate_lw": 1.6},
             {"x": 0.5, "y": 0.622, "text": text, "color": color, "size": 16, "weight": "bold"}]
    return text_layer(items, W, H)


# ---------------------------------------------------------------------------
# HUD layer + composite
# ---------------------------------------------------------------------------
def render_hud_layer(ctx, manifest_simt, idx_range, hud_dir, W, H):
    """Render transparent overlay HUD frames for a range of capture indices,
    named frame_<idx>.png (index-aligned with the capture). Returns dict
    idx -> BGRA. Also writes them to hud_dir (the Premiere overlay layer)."""
    os.makedirs(hud_dir, exist_ok=True)
    # map capture index -> nearest CSV row (ZOH on t_sim)
    clock = ctx.t_sim
    valid = np.where(~np.isnan(clock))[0]
    cl = list(clock[valid])
    import bisect
    hud = {}
    for idx in idx_range:
        t = manifest_simt[idx]
        pos = max(bisect.bisect_right(cl, t) - 1, 0)
        row = int(valid[pos])
        p = os.path.join(hud_dir, f"frame_{idx:06d}.png")
        rh.render_overlay_frame(ctx, row, p, transparent=True, W=W, H=H)
        hud[idx] = cv2.imread(p, cv2.IMREAD_UNCHANGED)
    return hud


# ---------------------------------------------------------------------------
# encode
# ---------------------------------------------------------------------------
def encode(frame_dir, mp4, fps=FPS):
    cmd = ["ffmpeg", "-y", "-framerate", str(fps), "-i",
           os.path.join(frame_dir, "frame_%06d.png"),
           "-vf", "format=yuv420p", "-c:v", "libx264", "-crf", "19", mp4]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"  wrote {mp4}")


def make_gif_range(frame_dir, start, end, gif, fps=12, height=464):
    """README-friendly GIF from a frame subrange (a punchy highlight loop, not
    the whole cut with its static cards). Copies the subrange to a temp dir
    renumbered from 0, then palette-encodes -- robust vs ffmpeg start_number
    interacting with the fps filter."""
    tmp = frame_dir + "_gifsrc"
    if os.path.isdir(tmp):
        shutil.rmtree(tmp)
    os.makedirs(tmp)
    for k, idx in enumerate(range(start, end)):
        shutil.copyfile(os.path.join(frame_dir, f"frame_{idx:06d}.png"),
                        os.path.join(tmp, f"frame_{k:06d}.png"))
    subprocess.run(
        ["ffmpeg", "-y", "-framerate", str(FPS), "-i", os.path.join(tmp, "frame_%06d.png"),
         "-vf", f"fps={fps},scale=-1:{height}:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse",
         gif], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    shutil.rmtree(tmp)
    print(f"  wrote {gif} (highlight loop, frames {start}-{end})")


# ---------------------------------------------------------------------------
# ONBOARD hero cut (PRIMARY)
# ---------------------------------------------------------------------------
def build_onboard(ctx, make_gif=True):
    W, H = rh.OVERLAY_W, rh.OVERLAY_H  # 1280x960
    simt, files = load_manifest(os.path.join(ONBOARD_DIR, "manifest.csv"))

    # key capture indices (from the manifest / CSV markers)
    EST0 = nearest_idx(simt, 14.0)       # ~300  establish start: target is a
                                         #       distant shape near the horizon
                                         #       while the interceptor holds on
                                         #       the external cue (comes in from
                                         #       far -- builder feedback #1)
    DASH0 = nearest_idx(simt, 18.42)     # 432  dash / buildup begins (range closes)
    HANDOFF = nearest_idx(simt, 20.164)  # 485  ext_fresh 1->0 (datalink denied)
    FIRSTDET = nearest_idx(simt, 20.264) # 488  first AprilTag detection
    CPA = ctx.cpa_idx and nearest_idx(simt, float(ctx.t_sim[ctx.cpa_idx]))  # 496
    cpa_m = float(ctx.gt_range[ctx.cpa_idx])

    need = range(EST0 - 1, CPA + 3)
    hud = render_hud_layer(ctx, simt, need, os.path.join(DEMO, "hud_onboard_transparent"), W, H)
    vid = {i: cv2.imread(files[i]) for i in need}   # raw seeker frames (blended for slow-mo)

    def comp_of(i):
        """HUD-over-video for a single (crisp) capture index."""
        return alpha_over(vid[i], hud[i])

    # disclosure pills + handoff callout (cached transparent layers). Three
    # honestly-labelled playback rates in a clean progression: real-time
    # establish -> gentle ~4x buildup (watch it grow) -> ~7x terminal.
    pill_rt = disclosure_pill(W, H, "REAL-TIME  1x", rh.COLOR_GRAY)
    pill_bu = disclosure_pill(W, H, "SLOW MOTION  ~4x", rh.COLOR_AMBER)
    pill_sm = disclosure_pill(W, H, "SLOW MOTION  ~7x", rh.COLOR_AMBER)
    ho_call = callout(W, H, "DATALINK DENIED  ->  CAMERA-ONLY TERMINAL", rh.COLOR_GREEN)
    banner = fuse_banner(W, H, cpa_m)

    out_dir = os.path.join(DEMO, "onboard_final_frames")
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    n = 0

    def emit(img):
        nonlocal n
        cv2.imwrite(os.path.join(out_dir, f"frame_{n:06d}.png"), img)
        n += 1

    def emit_hold(img, secs):
        for _ in range(int(round(secs * FPS))):
            emit(img)

    def emit_fade(img_from, img_to, secs):
        k = max(1, int(round(secs * FPS)))
        for j in range(k):
            emit(blend(img_from, img_to, (j + 1) / k))

    # Slow-mo emitter: cross-dissolve only the VIDEO between adjacent captured
    # frames (smooth), and overlay a CRISP (unblended) HUD from the nearer
    # source tick on top -- blending the HUD too would ghost its moving ticks/
    # text into a double image. `layers` are constant transparent overlays.
    def emit_slow(i0, i1, sub, layers):
        for i in range(i0, i1):
            for j in range(sub):
                f = j / sub
                frame = blend(vid[i], vid[i + 1], f)
                frame = alpha_over(frame, hud[i] if f < 0.5 else hud[i + 1])
                for lay in layers:
                    frame = alpha_over(frame, lay)
                emit(frame)

    # 1) TITLE (short: fade in from black, brief hold, fade to first frame)
    title = title_card(W, H)
    black = np.zeros((H, W, 3), np.uint8)
    emit_fade(black, title, 0.5)
    emit_hold(title, 2.4)
    first = alpha_over(comp_of(EST0), pill_rt)
    emit_fade(title, first, 0.5)

    # 2) ESTABLISH -- REAL-TIME 1x, coming in from far away (builder feedback #1):
    #    the target is a distant shape near the horizon while the interceptor
    #    holds on the external cue. Source ~30 fps ≈ 1x at 30 fps out.
    for i in range(EST0, DASH0):
        emit(alpha_over(comp_of(i), pill_rt))

    # 3) BUILDUP / DASH-IN -- gently paced ~4x slow-mo so the target visibly
    #    GROWS from a distant shape to an AprilTag speck as range closes
    #    (30 m -> ~5 m). Honest cross-dissolve of adjacent captured frames with
    #    a crisp HUD; the APRILTAG lamp holds SEARCHING here (flips LOCKED in
    #    the terminal beat below -- good drama, builder feedback #1).
    emit_slow(DASH0, HANDOFF, 4, [pill_bu])

    hl_start = n  # README-GIF highlight loop starts at the handoff beat

    # 4) HANDOFF beat -- slow, with the CAMERA-ONLY callout (datalink denied)
    emit_slow(HANDOFF, FIRSTDET, 8, [pill_sm, ho_call])

    # 5) TERMINAL slow-mo (~7x) -- the tag loom + the HUD firing solution
    #    collapsing. Kept EXACTLY as the builder liked it (feedback #2).
    emit_slow(FIRSTDET, CPA, 7, [pill_sm])

    # 6) CPA / PROXIMITY FUSE hold -- one thin criterion ring + DETONATE banner
    #    (de-cringed: no flash / no shockwave / no double rings, feedback #3).
    cpa_comp = alpha_over(comp_of(CPA), pill_sm)
    center = (int(W * 0.60), int(H * 0.67))  # the AprilTag centroid at CPA
    holdN = int(round(1.9 * FPS))
    for k in range(holdN):
        emit(draw_fuse(cpa_comp, banner, k / (holdN - 1), center))
    hl_end = n  # highlight loop ends after the fuse hold

    # 7) End on the fuse hold and cut to black -- NO outro/metrics card
    #    (builder feedback #4: don't explain the interceptor at the end).
    last_fuse = draw_fuse(cpa_comp, banner, 1.0, center)
    emit_fade(last_fuse, black, 0.5)

    mp4 = os.path.join(DEMO, "interceptor_onboard.mp4")
    print(f"[onboard] {n} frames ({n / FPS:.1f}s), CPA={cpa_m:.3f} m at capture idx {CPA}")
    encode(out_dir, mp4)
    if make_gif:
        make_gif_range(out_dir, hl_start, hl_end, os.path.join(DEMO, "interceptor_onboard.gif"))

    # refresh the committed sample still (the proximity-fuse frame)
    fuse_still = draw_fuse(cpa_comp, banner, 1.0, center)
    cv2.imwrite(os.path.join(REPO, "docs", "images", "demo_onboard_final.png"), fuse_still)
    return mp4, CPA


# ---------------------------------------------------------------------------
# CHASE cut (SECONDARY, brief wide view)
# ---------------------------------------------------------------------------
def build_chase(ctx):
    W, H = 960, 540
    simt, files = load_manifest(os.path.join(CHASE_DIR, "manifest.csv"))
    START = nearest_idx(simt, 19.6)
    CPA = nearest_idx(simt, float(ctx.t_sim[ctx.cpa_idx]))
    END = CPA + 2
    need = range(START, END + 1)
    hud = render_hud_layer(ctx, simt, need, os.path.join(DEMO, "hud_chase_transparent"), W, H)
    vid = {i: cv2.imread(files[i]) for i in need}
    label = text_layer([{"plate": (0.335, 0.792, 0.33, 0.052), "plate_alpha": 0.6},
                        {"x": 0.5, "y": 0.818, "text": "WIDE / CHASE VIEW  (same intercept)",
                         "color": rh.COLOR_GRAY, "size": 10, "weight": "bold"}], W, H)

    out_dir = os.path.join(DEMO, "chase_final_frames")
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    n = 0

    def emit(img):
        nonlocal n
        cv2.imwrite(os.path.join(out_dir, f"frame_{n:06d}.png"), img)
        n += 1

    # slight slow-mo (3x): blend video only, crisp HUD from the nearer tick
    sub = 3
    for i in range(START, END):
        for j in range(sub):
            f = j / sub
            frame = blend(vid[i], vid[i + 1], f)
            frame = alpha_over(frame, hud[i] if f < 0.5 else hud[i + 1])
            emit(alpha_over(frame, label))
    tail = alpha_over(alpha_over(vid[END], hud[END]), label)
    for _ in range(int(0.8 * FPS)):
        emit(tail)

    mp4 = os.path.join(DEMO, "interceptor_chase.mp4")
    print(f"[chase] {n} frames ({n / FPS:.1f}s)")
    encode(out_dir, mp4)
    cv2.imwrite(os.path.join(REPO, "docs", "images", "demo_chase_final.png"),
                alpha_over(alpha_over(vid[CPA], hud[CPA]), label))
    return mp4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true", help="skip GIF + chase cut")
    args = ap.parse_args()
    rows = rh.load_rows(HERO_CSV)
    ctx = rh.HudContext(rows, RADIUS, source_name=os.path.basename(HERO_CSV))
    print(f"hero: {os.path.basename(HERO_CSV)}  cpa_idx={ctx.cpa_idx}  "
          f"gt_range@cpa={ctx.gt_range[ctx.cpa_idx]:.3f} m  handoff_idx={ctx.handoff_idx}")
    build_onboard(ctx, make_gif=not args.fast)
    if not args.fast:
        build_chase(ctx)


if __name__ == "__main__":
    main()
