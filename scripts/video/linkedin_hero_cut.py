#!/usr/bin/env python3
"""Assemble the LinkedIn hero cut from the 2026-09-23 M4 pro-nav capture.

Source flight: logs/m4_intercept_pronav_20260923T025539Z.csv
  (check_m4-class scenario, law=pronav, miss 0.413 m, clean=1, engaged=1).
Frames: demo_out/linkedin_hero/onboard_frames/ (+ manifest.csv, sim-time
  stamped), captured live from the onboard camera topic during that flight.

HONESTY RULES (docs/demo_plan.md lineage):
  - Every overlay element is a pure function of the captured frames
    themselves (offline AprilTag re-decode: box corners, center, side px)
    or of the manifest (sim_t). No flight-CSV sync exists for this schema
    (t_sim populates only under --handoff/--coded-dash), so nothing from
    the CSV is drawn per-frame; the CSV contributes ONLY the end-card
    closest-approach number, quoted with its log path.
  - Retiming is disclosed on-screen (REAL-TIME / SLOW MOTION pills).
  - A SIMULATION tag is on screen for every flight frame.
  - No kill/contact claim: 0.413 m does NOT clear the 0.35 m ram radius
    (ADR-0084) and the end card does not claim it.

USAGE: .venv/bin/python scripts/video/linkedin_hero_cut.py
"""
import csv
import json
import os
import subprocess

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except ImportError:
    from pyapriltags import Detector

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HERO = os.path.join(REPO, "demo_out", "linkedin_hero")
FRAMES = os.path.join(HERO, "onboard_frames")
OUT_FRAMES = os.path.join(HERO, "cut_frames")
MP4 = os.path.join(HERO, "linkedin_intercept.mp4")
FPS = 30
W, H = 1280, 960

# palette: match the LinkedIn figure set (dataviz reference instance)
ORANGE = (52, 104, 235)   # BGR of #eb6834
BLUE = (214, 120, 42)     # BGR of #2a78d6
INK = (245, 245, 245)
DIM = (170, 170, 170)
BG = (24, 27, 25)         # near the dashboard dark bg

MISS_M = "0.413"          # M4_RESULT of the source flight (end card only)
LOG_NAME = "m4_intercept_pronav_20260923T025539Z.csv"

FONT = cv2.FONT_HERSHEY_SIMPLEX


def scan_all():
    """Re-decode every frame (cached to tag_scan_full.json)."""
    cache = os.path.join(HERO, "tag_scan_full.json")
    if os.path.exists(cache):
        return {int(k): v for k, v in json.load(open(cache)).items()}
    det = Detector(families="tag36h11", nthreads=4, quad_decimate=2.0)
    rows = list(csv.DictReader(open(os.path.join(FRAMES, "manifest.csv"))))
    out = {}
    for r in rows:
        img = cv2.imread(os.path.join(FRAMES, r["file"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        ds = det.detect(img)
        if ds:
            d = max(ds, key=lambda x: cv2.contourArea(x.corners.astype(np.float32)))
            out[int(r["idx"])] = {
                "sim_t": float(r["sim_t"]),
                "corners": d.corners.tolist(),
                "center": [float(d.center[0]), float(d.center[1])],
                "side": float(np.sqrt(cv2.contourArea(d.corners.astype(np.float32)))),
            }
    json.dump(out, open(cache, "w"))
    return out


def manifest():
    return {int(r["idx"]): r for r in csv.DictReader(open(os.path.join(FRAMES, "manifest.csv")))}


def pill(img, x, y, text, fg, border):
    (tw, th), _ = cv2.getTextSize(text, FONT, 0.62, 2)
    cv2.rectangle(img, (x - 10, y - th - 12), (x + tw + 10, y + 10), BG, -1)
    cv2.rectangle(img, (x - 10, y - th - 12), (x + tw + 10, y + 10), border, 2)
    cv2.putText(img, text, (x, y), FONT, 0.62, fg, 2, cv2.LINE_AA)


def overlay(img, scan_row, sim_t, speed_label):
    """Draw the honest frame-derived overlay."""
    if scan_row is not None:
        c = np.array(scan_row["corners"], dtype=np.int32)
        x0, y0 = c[:, 0].min(), c[:, 1].min()
        x1, y1 = c[:, 0].max(), c[:, 1].max()
        pad = max(10, (x1 - x0) // 8)
        x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
        L = max(14, (x1 - x0) // 5)
        for (cx, cy, dx, dy) in [(x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)]:
            cv2.line(img, (cx, cy), (cx + dx * L, cy), ORANGE, 2, cv2.LINE_AA)
            cv2.line(img, (cx, cy), (cx, cy + dy * L), ORANGE, 2, cv2.LINE_AA)
        cv2.putText(img, "TARGET LOCK", (x0, max(24, y0 - 12)), FONT, 0.55, ORANGE, 2, cv2.LINE_AA)
        lock = True
    else:
        lock = False

    # persistent honesty tags
    pill(img, 24, 42, "SIMULATION  PX4 SITL + GAZEBO", INK, DIM)
    pill(img, 24, 92, speed_label, INK, ORANGE if "SLOW" in speed_label else BLUE)
    pill(img, W - 260, 42, f"SIM T +{sim_t:6.2f} s", INK, DIM)
    st = "SEEKER: LOCK" if lock else "SEEKER: COAST"
    pill(img, W - 260, 92, st, ORANGE if lock else DIM, ORANGE if lock else DIM)
    cv2.putText(img, "onboard camera, guidance sees only this view + own-state estimator",
                (24, H - 20), FONT, 0.52, DIM, 1, cv2.LINE_AA)
    return img


def card(lines, sub_lines, n_frames, fname_prefix, k0):
    img = np.full((H, W, 3), BG, dtype=np.uint8)
    y = H // 2 - 40 * len(lines)
    for i, (txt, scale, color) in enumerate(lines):
        (tw, _), _ = cv2.getTextSize(txt, FONT, scale, 2)
        cv2.putText(img, txt, ((W - tw) // 2, y + i * 70), FONT, scale, color, 2, cv2.LINE_AA)
    yy = y + len(lines) * 70 + 20
    for j, txt in enumerate(sub_lines):
        (tw, _), _ = cv2.getTextSize(txt, FONT, 0.55, 1)
        cv2.putText(img, txt, ((W - tw) // 2, yy + j * 36), FONT, 0.55, DIM, 1, cv2.LINE_AA)
    k = k0
    for _ in range(n_frames):
        cv2.imwrite(os.path.join(OUT_FRAMES, f"cut_{k:06d}.png"), img)
        k += 1
    return k


def main():
    os.makedirs(OUT_FRAMES, exist_ok=True)
    for f in os.listdir(OUT_FRAMES):
        os.remove(os.path.join(OUT_FRAMES, f))
    scan = scan_all()
    man = manifest()

    # motion onset: first scanned frame whose tag center-x moved > 8 px from start
    idxs = sorted(scan.keys())
    cx0 = scan[idxs[0]]["center"][0]
    onset = next(i for i in idxs if abs(scan[i]["center"][0] - cx0) > 8)
    peak = max(idxs, key=lambda i: scan[i]["side"])
    last = idxs[-1]
    print(f"motion onset idx {onset}, peak idx {peak}, last decode {last}")

    k = 0
    # title card, 2.5 s
    k = card([("Autonomous camera-guided intercept", 1.05, INK),
              ("simulation, onboard seeker view", 0.8, DIM)],
             ["proportional navigation terminal, AprilTag practice target crossing at 2 m/s",
              "real time and slow motion, retiming labeled on screen"],
             int(2.5 * FPS), "title", k)

    slow_start = peak - 52  # ~1.7 s before the pass
    # segment A: approach, real time
    for i in range(max(onset - 30, 0), slow_start):
        if i not in man:
            continue
        img = cv2.imread(os.path.join(FRAMES, man[i]["file"]))
        img = overlay(img, scan.get(i), float(man[i]["sim_t"]), "REAL-TIME")
        cv2.imwrite(os.path.join(OUT_FRAMES, f"cut_{k:06d}.png"), img)
        k += 1
    # segment B: terminal, 5x slow motion (each frame written 5x)
    for i in range(slow_start, min(peak + 8, last + 1)):
        if i not in man:
            continue
        img = cv2.imread(os.path.join(FRAMES, man[i]["file"]))
        img = overlay(img, scan.get(i), float(man[i]["sim_t"]), "SLOW MOTION 5x")
        for _ in range(5):
            cv2.imwrite(os.path.join(OUT_FRAMES, f"cut_{k:06d}.png"), img)
            k += 1
    # end card, 5 s
    k = card([("Closest approach 0.413 m", 1.15, ORANGE),
              ("against a target crossing at 2 m/s", 0.8, INK)],
             ["proportional navigation, camera-only terminal, PX4 SITL + Gazebo Harmonic",
              "guidance never reads simulator ground truth, enforced by automated tests",
              f"number from the flight log: logs/{LOG_NAME}",
              "not a contact: the project's physical contact bar is 0.35 m, still open"],
             int(5.0 * FPS), "end", k)

    subprocess.run(["ffmpeg", "-y", "-framerate", str(FPS), "-i",
                    os.path.join(OUT_FRAMES, "cut_%06d.png"),
                    "-vf", "format=yuv420p", "-c:v", "libx264", "-crf", "19", MP4],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    dur = k / FPS
    print(f"wrote {MP4}  ({k} frames, {dur:.1f} s)")


if __name__ == "__main__":
    main()
