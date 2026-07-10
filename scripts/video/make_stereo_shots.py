#!/usr/bin/env python3
"""T25 shots 1-2 — the GROUND-STEREO opener (§B). HUD-forward: the story is the
CUE (detection state + triangulated range), not the sparse 135 m rig image.

Renders an animated stereo clip over the rig capture + ground-station audit:
  * L | R rig frames (real capture) as backdrop, per-cam detection confidence,
  * a SEARCHING -> TRACKING state lamp keyed off the ground-station event
    (miss/emit) — the real SEARCHING(0-15)->TRACKING(16-39) transition,
  * the triangulated range read-out vs gt (real triangulation), disparity,
  * a range-vs-seq trace that builds up across the 133-138 m corridor.

HONESTY (docs/t25_storyboard.md constraints):
  * Every number is from the real ground-station audit CSV / rig gt — no invented
    detections, no drawn NN box (rig->pixel projection is not asserted here; the
    detection is conveyed by the real det_conf + state, not a fabricated box).
  * The cue is a REPLAY of a genuinely-computed stereo triangulation (ADR-0048/
    0050), disclosed on the caption + the end card (mock-cue disclosure line).
  * Do NOT caption the rig image as "visibly weaving" — from this staging the
    weave lies along the rig LoS (render plan §B gap 1); the trace shows it.

Usage:
  .venv/bin/python scripts/video/make_stereo_shots.py \
    --rig  logs/rig_captures/t25_weave_replay \
    --audit logs/ground_station_20260710T234921Z.csv \
    --out-dir demo_out/t25 --fps 10
"""
import argparse
import csv
import glob
import json
import math
import os
import subprocess

from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 960
BG = (18, 20, 24)
FG = (232, 236, 240)
DIM = (150, 156, 164)
GREEN = (90, 220, 130)
AMBER = (240, 190, 90)
ACCENT = (120, 200, 255)
FONT_DIR = "/usr/share/fonts/truetype/dejavu"


def font(name, size):
    return ImageFont.truetype(os.path.join(FONT_DIR, name), size)


def centered(d, cx, y, text, f, fill):
    l, t, r, b = d.textbbox((0, 0), text, font=f)
    d.text((cx - (r - l) / 2, y), text, font=f, fill=fill)


def load_rows(rig, audit):
    meta = json.load(open(os.path.join(rig, "capture_meta.json")))
    rx, ry, rz, _ = meta["rig_pose_xyz_yaw"]
    idx = {int(r["seq"]): r for r in csv.DictReader(open(os.path.join(rig, "index.csv")))}
    aud = {int(r["seq"]): r for r in csv.DictReader(open(audit))}
    rows = []
    for seq in sorted(idx):
        ir, ar = idx[seq], aud.get(seq, {})
        gx, gy, gz = float(ir["gt_target_x"]), float(ir["gt_target_y"]), float(ir["gt_target_z"])
        gt_range = math.sqrt((gx - rx) ** 2 + (gy - ry) ** 2 + (gz - rz) ** 2)
        emit = ar.get("event") == "emit"
        rows.append({
            "seq": seq, "emit": emit, "gt_range": gt_range,
            "range_m": float(ar["range_m"]) if emit and ar.get("range_m") else None,
            "cl": float(ar["det_conf_left"]) if emit and ar.get("det_conf_left") else 0.0,
            "cr": float(ar["det_conf_right"]) if emit and ar.get("det_conf_right") else 0.0,
            "disp": float(ar["disparity_px"]) if emit and ar.get("disparity_px") else None,
        })
    return rows, meta


def panel(path, pw, ph):
    im = Image.open(path).convert("RGB").resize((pw, ph), Image.LANCZOS)
    return im


def render(rows, meta, rig, out_dir, fps):
    f_title = font("DejaVuSans-Bold.ttf", 36)
    f_lbl = font("DejaVuSans-Bold.ttf", 24)
    f_big = font("DejaVuSans-Bold.ttf", 46)
    f_med = font("DejaVuSans.ttf", 24)
    f_sm = font("DejaVuSans.ttf", 20)

    rng_lo = min(r["gt_range"] for r in rows) - 3
    rng_hi = max(r["gt_range"] for r in rows) + 3
    tracked = [r for r in rows if r["range_m"] is not None]
    mae = (sum(abs(r["range_m"] - r["gt_range"]) for r in tracked) / len(tracked)) if tracked else 0.0
    first = next((r for r in rows if r["emit"]), None)

    pw, ph = 600, 375
    x0, x1 = 20, 660
    py = 92
    # range-trace plot box
    px, pyt, pw2, ph2 = 90, 590, W - 180, 250

    frames_dir = os.path.join(out_dir, "_stereo_frames")
    os.makedirs(frames_dir, exist_ok=True)
    for old in glob.glob(os.path.join(frames_dir, "*.png")):
        os.remove(old)

    def xy(seq, rng):
        fx = px + pw2 * (seq / (len(rows) - 1))
        fy = pyt + ph2 * (1 - (rng - rng_lo) / (rng_hi - rng_lo))
        return fx, fy

    for i, r in enumerate(rows):
        c = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(c)
        centered(d, W / 2, 20, "GROUND STEREO — triangulated cue (2 m baseline)", f_title, FG)

        lf = os.path.join(rig, "left", f"{r['seq']:05d}.png")
        rf = os.path.join(rig, "right", f"{r['seq']:05d}.png")
        c.paste(panel(lf, pw, ph), (x0, py))
        c.paste(panel(rf, pw, ph), (x1, py))
        for x, cam, conf in ((x0, "LEFT", r["cl"]), (x1, "RIGHT", r["cr"])):
            d.rectangle([x, py, x + pw, py + ph], outline=(70, 76, 84), width=2)
            d.text((x + 8, py + 6), cam, font=f_lbl, fill=FG)
            if r["emit"]:
                d.text((x + pw - 150, py + 6), f"conf {conf:.2f}", font=f_sm, fill=GREEN)

        # state lamp + range read-out
        ly = py + ph + 16
        if r["emit"]:
            d.ellipse([x0, ly + 6, x0 + 26, ly + 32], fill=GREEN)
            d.text((x0 + 38, ly), "TRACKING", font=f_lbl, fill=GREEN)
            centered(d, W / 2, ly - 4, f"RANGE  {r['range_m']:.1f} m", f_big, ACCENT)
            d.text((W - 300, ly), f"gt {r['gt_range']:.1f} m   err {abs(r['range_m']-r['gt_range']):.2f} m",
                   font=f_sm, fill=DIM)
            if r["disp"]:
                d.text((W - 300, ly + 26), f"disparity {r['disp']:.0f} px", font=f_sm, fill=DIM)
        else:
            d.ellipse([x0, ly + 6, x0 + 26, ly + 32], fill=AMBER)
            d.text((x0 + 38, ly), "SEARCHING", font=f_lbl, fill=AMBER)
            centered(d, W / 2, ly + 2, "acquiring — target entering field of view",
                     f_med, DIM)

        # range trace
        d.rectangle([px, pyt, px + pw2, pyt + ph2], outline=(60, 66, 74), width=1)
        d.text((px, pyt - 26), "triangulated range vs ground truth", font=f_sm, fill=DIM)
        d.text((px - 78, pyt - 6), f"{rng_hi:.0f} m", font=f_sm, fill=DIM)
        d.text((px - 78, pyt + ph2 - 14), f"{rng_lo:.0f} m", font=f_sm, fill=DIM)
        # gt line up to current
        gtpts = [xy(rr["seq"], rr["gt_range"]) for rr in rows[:i + 1]]
        if len(gtpts) > 1:
            d.line(gtpts, fill=(110, 116, 124), width=2)
        # triangulated dots up to current
        for rr in rows[:i + 1]:
            if rr["range_m"] is not None:
                fx, fy = xy(rr["seq"], rr["range_m"])
                d.ellipse([fx - 4, fy - 4, fx + 4, fy + 4], fill=GREEN)
        if first:
            fx, _ = xy(first["seq"], first["gt_range"])
            d.line([(fx, pyt), (fx, pyt + ph2)], fill=(80, 120, 90), width=1)
            d.text((fx + 4, pyt + 2), f"first firm track {first['gt_range']:.0f} m",
                   font=f_sm, fill=GREEN)

        centered(d, W / 2, H - 46,
                 f"replayed stereo triangulation · mean range error {mae:.2f} m across the corridor",
                 f_sm, DIM)
        c.save(os.path.join(frames_dir, f"f{i:04d}.png"))

    # encode shot 1 (full detect->track clip)
    shot1 = os.path.join(out_dir, "shot1_ground_stereo.mp4")
    subprocess.run(["ffmpeg", "-y", "-framerate", str(fps), "-i",
                    os.path.join(frames_dir, "f%04d.png"),
                    "-vf", "format=yuv420p", "-c:v", "libx264", "-crf", "19", shot1],
                   check=True, capture_output=True)
    # shot 2 = the converged-track tail held (last frame) as a still card
    shot2 = os.path.join(out_dir, "shot2_track_cue_up.png")
    Image.open(os.path.join(frames_dir, f"f{len(rows)-1:04d}.png")).save(shot2)
    print(f"[stereo] wrote {shot1} ({len(rows)} frames @ {fps}fps) + {shot2}")
    print(f"[stereo] first firm track {first['gt_range']:.1f} m, mean range err {mae:.2f} m, "
          f"tracked {len(tracked)}/{len(rows)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", required=True)
    ap.add_argument("--audit", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--fps", type=int, default=10)
    a = ap.parse_args()
    rows, meta = load_rows(a.rig, a.audit)
    render(rows, meta, a.rig, a.out_dir, a.fps)


if __name__ == "__main__":
    main()
