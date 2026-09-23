#!/usr/bin/env python3
"""Assemble the FAST LinkedIn cut (v2): two acts from 2026-09-23 captures.

ACT 1 — the sprint: the ADOPTED open-loop dash config (16 m/s commanded,
dash-unclamped, camera acquisition gated shut) against the canonical line-9
crosser (9 m/s). Single flight, CPA 0.334 m (gt scoring), pass at sim
t=23.57. Frames: logs/sprint_capture_20260923b/onboard_frames (sim-time
manifest aligns with the flight CSV's t_sim; both are gz /clock).
  logs/m4_intercept_pronav_20260923T060459Z.csv

ACT 2 — the camera-guided intercept: classic tag scenario, target at 3 m/s
(a 50% faster target than the v1 video), pro-nav terminal, CPA 0.238 m
clean. Frames: logs/speed_scout_20260923/frames_v3; pass located by offline
tag re-decode (largest-tag frame), same method as linkedin_hero_cut.py.
  logs/m4_intercept_pronav_20260923T055719Z.csv

HONESTY: overlays derive from the frames/manifest only; CSVs contribute the
end-card numbers, quoted with log paths. Both flights are n=1 under the
best-case givens (perfect launch cue from known kinematics, no wind, known
target height); the sprint's 0.334 m is inside the 0.35 m bar but a single
flight — the adopted config's measured rate is 13/16 best-case (ADR-0100).
Retiming disclosed on-screen; SIMULATION tag on every flight frame.

USAGE: .venv/bin/python scripts/video/linkedin_fast_cut.py
"""
import csv
import json
import os
import subprocess
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from linkedin_hero_cut import BG, DIM, INK, ORANGE, BLUE, FONT, pill  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SPRINT_FRAMES = os.path.join(REPO, "logs", "sprint_capture_20260923b", "onboard_frames")
CHASE_FRAMES = os.path.join(REPO, "logs", "speed_scout_20260923", "frames_v3")
OUT_FRAMES = os.path.join(REPO, "demo_out", "linkedin_hero", "fast_cut_frames")
MP4 = os.path.join(REPO, "demo_out", "linkedin_hero", "linkedin_fast_intercept.mp4")
FPS = 30
W, H = 1280, 960
SPRINT_CPA_SIM_T = 23.57


def manifest(d):
    return list(csv.DictReader(open(os.path.join(d, "manifest.csv"))))


def overlay(img, sim_t, speed_label, act_label, lock_box=None):
    if lock_box is not None:
        c = np.array(lock_box, dtype=np.int32)
        x0, y0 = c[:, 0].min(), c[:, 1].min()
        x1, y1 = c[:, 0].max(), c[:, 1].max()
        pad = max(10, (x1 - x0) // 8)
        x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
        L = max(14, (x1 - x0) // 5)
        for (cx, cy, dx, dy) in [(x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)]:
            cv2.line(img, (cx, cy), (cx + dx * L, cy), ORANGE, 2, cv2.LINE_AA)
            cv2.line(img, (cx, cy), (cx, cy + dy * L), ORANGE, 2, cv2.LINE_AA)
        cv2.putText(img, "TARGET LOCK", (x0, max(24, y0 - 12)), FONT, 0.55, ORANGE, 2, cv2.LINE_AA)
    pill(img, 24, 42, "SIMULATION  PX4 SITL + GAZEBO", INK, DIM)
    pill(img, 24, 92, speed_label, INK, ORANGE if "SLOW" in speed_label else BLUE)
    pill(img, W - 300, 42, f"SIM T +{sim_t:6.2f} s", INK, DIM)
    pill(img, 24, H - 30, act_label, INK, DIM)
    return img


def card(lines, sub_lines, n_frames, k0):
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


def scan_chase():
    cache = os.path.join(REPO, "logs", "speed_scout_20260923", "tag_scan_v3.json")
    if os.path.exists(cache):
        return {int(k): v for k, v in json.load(open(cache)).items()}
    try:
        from pupil_apriltags import Detector
    except ImportError:
        from pyapriltags import Detector
    det = Detector(families="tag36h11", nthreads=4, quad_decimate=2.0)
    out = {}
    for r in manifest(CHASE_FRAMES):
        img = cv2.imread(os.path.join(CHASE_FRAMES, r["file"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        ds = det.detect(img)
        if ds:
            d = max(ds, key=lambda x: cv2.contourArea(x.corners.astype(np.float32)))
            out[int(r["idx"])] = {"sim_t": float(r["sim_t"]), "corners": d.corners.tolist(),
                                  "side": float(np.sqrt(cv2.contourArea(d.corners.astype(np.float32))))}
    json.dump(out, open(cache, "w"))
    return out


def main():
    os.makedirs(OUT_FRAMES, exist_ok=True)
    for f in os.listdir(OUT_FRAMES):
        os.remove(os.path.join(OUT_FRAMES, f))
    k = 0
    k = card([("Two regimes, one interceptor", 1.05, INK),
              ("simulation, onboard seeker views", 0.8, DIM)],
             ["act 1: the open-loop sprint, 16 m/s commanded against a 9 m/s crossing drone",
              "act 2: the camera-guided intercept of a 3 m/s target",
              "real time and slow motion, retiming labeled on screen"], int(3.0 * FPS), k)

    # ---- ACT 1: sprint. Real-time from CPA-3.2s, slow-mo 6x through the pass.
    man = manifest(SPRINT_FRAMES)
    idx_of = lambda t: min(range(len(man)), key=lambda i: abs(float(man[i]["sim_t"]) - t))
    a, b, c = idx_of(SPRINT_CPA_SIM_T - 3.2), idx_of(SPRINT_CPA_SIM_T - 0.55), idx_of(SPRINT_CPA_SIM_T + 0.35)
    act = "ACT 1  OPEN-LOOP SPRINT  16 m/s cmd vs 9 m/s crosser (no camera steering)"
    for i in range(a, b):
        img = cv2.imread(os.path.join(SPRINT_FRAMES, man[i]["file"]))
        cv2.imwrite(os.path.join(OUT_FRAMES, f"cut_{k:06d}.png"),
                    overlay(img, float(man[i]["sim_t"]), "REAL-TIME", act))
        k += 1
    for i in range(b, c):
        img = cv2.imread(os.path.join(SPRINT_FRAMES, man[i]["file"]))
        img = overlay(img, float(man[i]["sim_t"]), "SLOW MOTION 6x", act)
        for _ in range(6):
            cv2.imwrite(os.path.join(OUT_FRAMES, f"cut_{k:06d}.png"), img)
            k += 1
    k = card([("Closest approach 0.334 m", 1.1, ORANGE),
              ("open-loop, 16 m/s commanded sprint", 0.75, INK)],
             ["single flight, best-case givens (perfect cue, no wind, known height)",
              "the adopted config measures 13/16 inside 0.35 m under the same givens (ADR-0100)",
              "log: logs/m4_intercept_pronav_20260923T060459Z.csv"], int(3.5 * FPS), k)

    # ---- ACT 2: camera-guided 3 m/s intercept.
    scan = scan_chase()
    cman = manifest(CHASE_FRAMES)
    idxs = sorted(scan.keys())
    peak = max(idxs, key=lambda i: scan[i]["side"])
    cx0 = scan[idxs[0]]["corners"][0][0]
    onset = next((i for i in idxs if abs(scan[i]["corners"][0][0] - cx0) > 8), idxs[0])
    act2 = "ACT 2  CAMERA-GUIDED INTERCEPT  pro-nav terminal, 3 m/s target"
    slow_start = peak - 45
    rowby = {int(r["idx"]): r for r in cman}
    for i in range(max(onset - 20, 0), slow_start):
        if i not in rowby:
            continue
        img = cv2.imread(os.path.join(CHASE_FRAMES, rowby[i]["file"]))
        box = scan[i]["corners"] if i in scan else None
        cv2.imwrite(os.path.join(OUT_FRAMES, f"cut_{k:06d}.png"),
                    overlay(img, float(rowby[i]["sim_t"]), "REAL-TIME", act2, box))
        k += 1
    for i in range(slow_start, min(peak + 8, idxs[-1] + 1)):
        if i not in rowby:
            continue
        img = cv2.imread(os.path.join(CHASE_FRAMES, rowby[i]["file"]))
        box = scan[i]["corners"] if i in scan else None
        img = overlay(img, float(rowby[i]["sim_t"]), "SLOW MOTION 5x", act2, box)
        for _ in range(5):
            cv2.imwrite(os.path.join(OUT_FRAMES, f"cut_{k:06d}.png"), img)
            k += 1
    k = card([("Closest approach 0.238 m", 1.1, ORANGE),
              ("camera-guided, 3 m/s crossing target", 0.75, INK)],
             ["proportional navigation, camera-only terminal; guidance never reads",
              "simulator ground truth (enforced by automated tests)",
              "single flights shown; the 0.35 m physical contact bar remains open",
              "log: logs/m4_intercept_pronav_20260923T055719Z.csv"], int(4.5 * FPS), k)

    subprocess.run(["ffmpeg", "-y", "-framerate", str(FPS), "-i",
                    os.path.join(OUT_FRAMES, "cut_%06d.png"),
                    "-vf", "format=yuv420p", "-c:v", "libx264", "-crf", "19", MP4],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"wrote {MP4}  ({k} frames, {k / FPS:.1f} s)")


if __name__ == "__main__":
    main()
