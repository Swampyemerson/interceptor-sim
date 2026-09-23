#!/usr/bin/env python3
"""T25 tilt-shot A/B card (#40 / ADR-0067) — a side-by-side of the LEVEL vs
UP-TILT onboard camera at a MATCHED target range, showing the nominal
perception-availability win (the up-tilt lowers the horizon so the horizon-level
target gains top-edge margin during the nose-down dash).

HONESTY (bound by docs/t25_storyboard.md + ADR-0068):
  * Frames are selected by MATCHED gt_range (not by look) — the caller passes the
    two frames the manifest maps to the same range; this script only composites.
  * The caption cites the ARM-LEVEL ADR-0067 result (dash-above-FoV 32% -> 0%,
    n=8 paired), NOT the single demo flight's numbers — one flight is an
    illustration, the arm is the evidence.
  * It is a NOMINAL-AVAILABILITY claim ONLY. The #40 Stage-1 nominal-terminal
    re-fly FAILED strict parity and Stage-2 (comms-jam recovery) was a NULL
    (ADR-0068) — so NO fixed up-tilt mount is adopted (the fixed-tilt adoption
    is graveyarded in docs/project_state.json; adaptive tilt #46/ADR-0065 is the
    live pointing lever). The card states both explicitly and must NOT imply
    either an adopted mount or comms-denied recovery.

Usage:
  .venv/bin/python scripts/video/make_tilt_ab_card.py \
      --level demo_out/t25/onboard_frames/frame_000678.png \
      --tilt  demo_out/t25/onboard_frames_up15/frame_000669.png \
      --range-m 15.0 --out demo_out/t25/shot3b_tilt_ab.png
"""
import argparse
import os

from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 960
BG = (18, 20, 24)
FG = (232, 236, 240)
DIM = (150, 156, 164)
ACCENT = (120, 200, 255)
FONT_DIR = "/usr/share/fonts/truetype/dejavu"


def font(name, size):
    return ImageFont.truetype(os.path.join(FONT_DIR, name), size)


def centered(d, cx, y, text, f, fill):
    l, t, r, b = d.textbbox((0, 0), text, font=f)
    d.text((cx - (r - l) / 2, y), text, font=f, fill=fill)
    return b - t


def panel(src, pw, ph):
    im = Image.open(src).convert("RGB")
    im = im.resize((pw, ph), Image.LANCZOS)
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", required=True, help="LEVEL-camera frame (up00)")
    ap.add_argument("--tilt", required=True, help="UP-TILT frame (up15)")
    ap.add_argument("--range-m", type=float, required=True,
                    help="matched target gt_range for both frames (m)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    canvas = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(canvas)
    f_title = font("DejaVuSans-Bold.ttf", 40)
    f_sub = font("DejaVuSans.ttf", 22)
    f_label = font("DejaVuSans-Bold.ttf", 30)
    f_lsub = font("DejaVuSans.ttf", 22)
    f_cap = font("DejaVuSans.ttf", 24)

    centered(d, W / 2, 26, "PERCEPTION AVAILABILITY — up-tilt camera mount", f_title, FG)
    centered(d, W / 2, 76, f"onboard seeker · 12 m/s weave target · matched {a.range_m:.0f} m range",
             f_sub, DIM)

    pw, ph = 588, 441          # 4:3 panels
    gap = 24
    x0 = (W - (2 * pw + gap)) // 2
    y0 = 130
    for i, (src, label, sub) in enumerate([
            (a.level, "LEVEL CAMERA", "horizon & target jam the top edge"),
            (a.tilt, "UP-TILT +15°", "horizon lowered — target framed, +margin")]):
        x = x0 + i * (pw + gap)
        canvas.paste(panel(src, pw, ph), (x, y0))
        d.rectangle([x, y0, x + pw, y0 + ph], outline=(70, 76, 84), width=2)
        col = ACCENT if i == 1 else DIM
        centered(d, x + pw / 2, y0 + ph + 14, label, f_label, col)
        centered(d, x + pw / 2, y0 + ph + 52, sub, f_lsub, DIM)

    cap_y = y0 + ph + 118
    d.line([(x0, cap_y - 10), (x0 + 2 * pw + gap, cap_y - 10)], fill=(60, 66, 74), width=1)
    centered(d, W / 2, cap_y,
             "Up-tilt keeps the horizon-level target in frame during the nose-down dash:",
             f_cap, FG)
    centered(d, W / 2, cap_y + 34,
             "target above field-of-view 32% → 0% of dash  (ADR-0067, n=8 paired)",
             f_cap, ACCENT)
    centered(d, W / 2, cap_y + 72,
             "compensated arm restores terminal Pk@2.5 to 8/8 — but strict parity was NOT met:",
             f_sub, DIM)
    centered(d, W / 2, cap_y + 100,
             "NO fixed up-tilt mount is adopted, and the tilt does NOT enable comms-denied "
             "recovery (ADR-0068)",
             f_sub, DIM)

    canvas.save(a.out)
    print(f"[tilt-ab] wrote {a.out} ({W}x{H})")


if __name__ == "__main__":
    main()
