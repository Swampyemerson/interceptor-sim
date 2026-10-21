#!/usr/bin/env python3
"""Composite the technical HUD over rendered shots and assemble the video.

Inputs: renders/<take>/manifest.json (from blender_scene.py; per-frame sim
times per shot), the take's `*_run.json` (telemetry + events + seeker-frame
index + offline detection boxes) and `*_frames/` (real seeker JPEGs).
Output: <out>/frames/%05d.png + <out>/<name>.mp4 (ffmpeg libx264).

HUD (configurable via --hud-config JSON):
  top-left   title + sim clock + SM phase
  bottom-left telemetry panel: own/target speed, closing speed, range,
             guidance source line -- INERTIAL PREDICT (cue) before the first
             sustained tag decode, CAMERA TRACK -- TAG LOCK while decodes are
             fresh (<lock_hold_s old), SEARCHING between
  bottom-right picture-in-picture: the REAL onboard seeker frame nearest in
             sim time (live Gazebo camera recording), detection corners drawn
             when that frame decoded, crosshair + label
  slow-mo badge when a shot's speed < 1; end card with the scored numbers.

Every number shown is read from the run logs -- nothing is typed in.
"""
import argparse
import json
import math
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
MONO_B = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"


class Tele:
    def __init__(self, run):
        self.run = run
        self.t0, self.hz, self.n = run["t"][0], run["hz"], len(run["t"])

    def at(self, key, ts):
        x = (ts - self.t0) * self.hz
        i = max(0, min(self.n - 2, int(x)))
        f = max(0.0, min(1.0, x - i))
        a = self.run[key]
        return a[i] * (1 - f) + a[i + 1] * f

    def phase(self, ts):
        ph = "STANDBY"
        for t, name in self.run["events"]["sm_states"]:
            if ts >= t:
                ph = name
        return ph

    def guidance(self, ts, lock_hold=0.6):
        ev = self.run["events"]
        if ts < ev["go_t"]:
            return "standby"
        last = None
        for t, _r in ev["det_ticks"]:
            if t <= ts:
                last = t
            else:
                break
        if last is not None and ts - last <= lock_hold:
            return "lock"
        if last is None:
            return "predict"
        return "coast" if ts <= ev["last_det_t"] + lock_hold else "post"


def nearest_frame(frames_index, ts, tol=0.15):
    best, bd = None, tol
    for n, t in frames_index:
        d = abs(t - ts)
        if d < bd:
            best, bd = n, d
    return best


def rounded_panel(draw, box, fill=(8, 12, 10, 185), outline=(90, 220, 140, 160)):
    draw.rounded_rectangle(box, radius=8, fill=fill, outline=outline, width=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--renders", required=True, help="renders/<take> dir")
    ap.add_argument("--run", required=True, help="<stem>_run.json")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="intercept_demo")
    ap.add_argument("--title", default="INTERCEPTOR-SIM · CAMERA-ONLY PURSUIT · 12 m/s TARGET")
    ap.add_argument("--subtitle", default="PX4 SITL + Gazebo camera-in-the-loop flight — offline cinematic re-render from pose logs")
    ap.add_argument("--freeze-s", type=float, default=3.0)
    ap.add_argument("--fps", type=int, default=None, help="override manifest fps")
    args = ap.parse_args()

    man = json.load(open(os.path.join(args.renders, "manifest.json")))
    run = json.load(open(args.run))
    tele = Tele(run)
    ev = run["events"]
    meta = run["meta"]
    stem = args.run[:-len("_run.json")]
    fdir = stem + "_frames"
    fps = args.fps or man["fps"]
    boxes = {int(k): v for k, v in run["det_boxes"].items()}
    frames_index = run["frames_index"]

    fr_dir = os.path.join(args.out, "frames")
    os.makedirs(fr_dir, exist_ok=True)
    f_sm = ImageFont.truetype(MONO, 22)
    f_md = ImageFont.truetype(MONO_B, 26)
    f_lg = ImageFont.truetype(MONO_B, 40)
    f_xs = ImageFont.truetype(MONO, 17)

    cue = meta["config"]
    cue_err_m = math.hypot(cue["cue_offset_e"], cue["cue_offset_n"])
    cue_err_deg = abs(cue["cue_vel_rot_deg"])
    W = H = None
    k = 0
    video_t = 0.0

    def put(img):
        nonlocal k
        img.convert("RGB").save(os.path.join(fr_dir, f"{k:05d}.png"))
        k += 1

    for shot in man["shots"]:
        for fr, ts in shot["frames"]:
            base = Image.open(os.path.join(shot["dir"], f"{fr:04d}.png")).convert("RGBA")
            if W is None:
                W, H = base.size
            ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
            d = ImageDraw.Draw(ov)

            # ---- top-left: title + clock + phase (panel sized to the text)
            tw = d.textlength(args.title, font=f_md)
            rounded_panel(d, (24, 20, max(560, 40 + tw + 24), 96))
            d.text((40, 30), args.title, font=f_md, fill=(235, 245, 238, 255))
            ph = tele.phase(ts)
            tplus = ts - ev["go_t"]
            d.text((40, 64), f"T{tplus:+06.2f} s   PHASE {ph}", font=f_sm,
                   fill=(150, 230, 170, 255))
            if video_t < 3.5:
                a = 255 if video_t < 2.8 else int(255 * (3.5 - video_t) / 0.7)
                d.text((40, 104), args.subtitle, font=f_xs,
                       fill=(220, 230, 225, a))

            # ---- bottom-left: telemetry + guidance source
            own = tele.at("own_speed", ts)
            tgt = tele.at("tgt_speed", ts)
            cls = tele.at("closing_ms", ts)
            rng = tele.at("range_m", ts)
            y0 = H - 196
            rounded_panel(d, (24, y0, 560, H - 24))
            pre_go = ts < ev["go_t"]   # target still parked: range/closing not meaningful
            rows = [("OWN SPEED", f"{own:5.1f} m/s"),
                    ("TARGET SPEED", "  --" if pre_go else f"{tgt:5.1f} m/s"),
                    ("CLOSING RATE", "  --" if pre_go else f"{cls:+5.1f} m/s"),
                    ("RANGE (truth)", "  --" if pre_go else f"{rng:6.2f} m")]
            for i, (lab, val) in enumerate(rows):
                d.text((44, y0 + 14 + 26 * i), f"{lab:<14}", font=f_sm,
                       fill=(150, 175, 160, 255))
                d.text((268, y0 + 14 + 26 * i), val, font=f_sm,
                       fill=(235, 245, 238, 255))
            g = tele.guidance(ts)
            gy = y0 + 14 + 26 * 4 + 6
            styles = {
                "standby": ((255, 190, 60), "GUIDANCE  ARMED — AWAITING GO"),
                "predict": ((255, 190, 60), f"GUIDANCE  INERTIAL PREDICT (cue err {cue_err_m:.1f} m/{cue_err_deg:.0f}°)"),
                "coast":   ((255, 120, 60), "GUIDANCE  COAST — REACQUIRING"),
                "lock":    ((70, 255, 140), "GUIDANCE  CAMERA TRACK — TAG LOCK"),
                "post":    ((150, 175, 160), "GUIDANCE  PASS COMPLETE"),
            }
            col, txt = styles[g]
            blink = g == "lock" or (int(ts * 2.5) % 2 == 0)
            if blink:
                d.ellipse((44, gy + 7, 58, gy + 21), fill=col + (255,))
            d.text((70, gy), txt, font=f_sm, fill=col + (255,))

            # ---- bottom-right: real seeker PiP
            pw, ph_ = 420, 315
            px, py = W - pw - 24, H - ph_ - 24
            fn = nearest_frame(frames_index, ts)
            rounded_panel(d, (px - 6, py - 34, px + pw + 6, py + ph_ + 6),
                          fill=(8, 12, 10, 205))
            d.text((px + 4, py - 28), "ONBOARD SEEKER — LIVE SIM FEED",
                   font=f_xs, fill=(150, 230, 170, 255))
            if fn is not None:
                try:
                    pip = Image.open(os.path.join(fdir, f"{fn:05d}.jpg"))
                    sx, sy = pw / pip.width, ph_ / pip.height
                    pip = pip.resize((pw, ph_))
                    ov.paste(pip, (px, py))
                    d = ImageDraw.Draw(ov)
                    if fn in boxes:
                        c = [(px + x * sx, py + y * sy) for x, y in boxes[fn]]
                        d.polygon(c, outline=(70, 255, 140, 255), width=3)
                        cxm = sum(p[0] for p in c) / 4
                        cym = sum(p[1] for p in c) / 4
                        d.text((min(cxm + 14, px + pw - 110), cym - 10), "TAG 0",
                               font=f_xs, fill=(70, 255, 140, 255))
                except Exception:
                    pass
                ch_x, ch_y = px + pw / 2, py + ph_ / 2
                d.line((ch_x - 14, ch_y, ch_x + 14, ch_y),
                       fill=(140, 255, 180, 120), width=1)
                d.line((ch_x, ch_y - 14, ch_x, ch_y + 14),
                       fill=(140, 255, 180, 120), width=1)
                d.text((px + 4, py + ph_ - 22),
                       f"{meta['intrinsics']['w']}x{meta['intrinsics']['h']} "
                       f"{meta['hfov_deg']:.0f}°HFOV", font=f_xs,
                       fill=(200, 220, 210, 230))

            # ---- slow-mo badge
            if shot["speed"] < 0.999:
                rounded_panel(d, (W - 250, 20, W - 24, 64),
                              fill=(30, 8, 8, 190), outline=(255, 120, 80, 200))
                d.text((W - 234, 30), f"REPLAY {shot['speed']:.1f}x",
                       font=f_md, fill=(255, 160, 110, 255))

            put(Image.alpha_composite(base, ov))
            video_t += 1.0 / fps

    # ---- freeze + end card
    last = Image.open(os.path.join(fr_dir, f"{k-1:05d}.png")).convert("RGBA")
    ov = Image.new("RGBA", last.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    W, H = last.size
    bx0, by0 = W // 2 - 530, H // 2 - 170
    rounded_panel(d, (bx0, by0, bx0 + 1060, by0 + 340),
                  fill=(8, 12, 10, 215))
    lines = [
        ("INTERCEPT", f_lg, (70, 255, 140, 255)),
        (f"closest approach  {ev['cpa_m']:.2f} m   (contact radius 0.35 m)",
         f_md, (235, 245, 238, 255)),
        (f"target speed 12 m/s - camera decodes {len(ev['det_ticks'])} "
         f"({ev['last_det_t'] - ev['first_det_t']:.1f} s of lock)",
         f_sm, (200, 220, 210, 255)),
        (f"cue error injected {cue_err_m:.1f} m / {cue_err_deg:.0f}° — "
         "terminal correction flown on camera alone", f_sm, (200, 220, 210, 255)),
        ("guidance saw ONLY camera pixels + own-state EKF",
         f_sm, (150, 230, 170, 255)),
        ("PX4 SITL + Gazebo | flight code: flight/pursuit_terminal.py",
         f_xs, (150, 175, 160, 255)),
    ]
    yy = by0 + 26
    for txt, f, col in lines:
        d.text((bx0 + 36, yy), txt, font=f, fill=col)
        yy += f.size + 18
    frozen = Image.alpha_composite(last, ov).convert("RGB")
    for _ in range(int(args.freeze_s * fps)):
        frozen.save(os.path.join(fr_dir, f"{k:05d}.png"))
        k += 1

    mp4 = os.path.join(args.out, f"{args.name}.mp4")
    subprocess.run([
        "ffmpeg", "-y", "-framerate", str(fps),
        "-i", os.path.join(fr_dir, "%05d.png"),
        "-c:v", "libx264", "-preset", "slow", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", mp4], check=True,
        capture_output=True)
    print(f"[hud] {k} frames -> {mp4} ({k/fps:.1f} s at {fps} fps)")


if __name__ == "__main__":
    main()
