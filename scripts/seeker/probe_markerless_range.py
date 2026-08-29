#!/usr/bin/env python3
"""Static acquisition-range probe for the markerless seeker (ADR-0033 item 2, A2).

WHY: before spending a paired-seed flight A/B on the markerless seeker, we need
one decisive number -- does the two-stage markerless seeker (the EXACT pipeline
scripts/seeker/markerless_loop.py flies) fire on the *tag-less* body
`models/fpv_target_markerless`, and out to what range? The offline prototype
(docs/seeker_prototype_results.md) measured ~1.6-3 m acquisition but on TAGGED
demo footage whose tag contrast HELPED detection; the tag-less body has no such
help, so its P_detect(R) must be measured directly. This is the "lab ranks,
Gazebo decides" step for the seeker.

WHAT IT DOES: with PX4 SITL + gz_x500_mono_cam already running on the MARKERLESS
world (see below), this teleports the tag-less target to a sequence of pure
forward ranges (x = r, y = 0), grabs one onboard camera frame at each, saves the
PNG, then runs the two-stage seeker on each frame and reports detect / bearing /
range-estimate / confidence vs the true range. No guidance, no flight, no gt fed
to the detector -- gt range here is the *teleport commanded* range, used only to
label the probe (SCORING-only, exactly like the gt_* boundary, CLAUDE.md).

HONESTY: the seeker reads ONLY camera pixels + the fixed intrinsics. The commanded
range is never handed to the detector. This probe is a design-time surrogate; the
live flight A/B is what concludes.

SET-POSE QUIRK: this process holds a gz-transport SUBSCRIPTION (the camera), and
a subscriber never receives gz service RESPONSES (the SimClockHolder quirk noted
throughout m4_intercept.py). So the teleport is done via a SHORT-LIVED
`gz service` CLI SUBPROCESS, never in-process -- the same pattern preplace_target
and the mover use.

RUN (two terminals / background the sim):
  # terminal 1 -- boot the markerless world (drone sits disarmed; camera still
  # publishes at 30 Hz and /clock advances in headless -r):
  ln -sf $PWD/worlds/markerless.sdf \
     ~/PX4-Autopilot/Tools/simulation/gz/worlds/markerless.sdf
  cd ~/PX4-Autopilot && PX4_GZ_WORLD=markerless \
     GZ_SIM_RESOURCE_PATH=$HOME/interceptor-sim/models HEADLESS=1 \
     make px4_sitl gz_x500_mono_cam
  # terminal 2 -- once "Startup script returned successfully" appears:
  INTERCEPTOR_WORLD_NAME=markerless \
     .venv-seeker/bin/python scripts/seeker/probe_markerless_range.py \
     --ranges 2,3,4,5,7,9,12 --out logs/markerless_probe
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))       # .../scripts/seeker
SCRIPTS = os.path.dirname(HERE)                          # .../scripts
REPO = os.path.dirname(SCRIPTS)                          # repo root
# import the camera topic exactly as the flight stack builds it (from
# INTERCEPTOR_WORLD_NAME) -- set the env BEFORE importing m2_detect so its
# module-level IMAGE_TOPIC keys off the markerless world.
os.environ.setdefault("INTERCEPTOR_WORLD_NAME", "markerless")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, HERE)

import cv2  # noqa: E402

from m2_detect import IMAGE_TOPIC, WORLD_NAME  # noqa: E402
from two_stage_seeker import TwoStageSeeker  # noqa: E402

from gz.transport13 import Node  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402

# gz_x500_mono_cam intrinsics (configs/camera_intrinsics.json / docs/next.md Key facts).
FX = FY = 539.936
CX, CY = 640.0, 480.0

TARGET_MODEL = os.environ.get("INTERCEPTOR_TARGET_MODEL", "fpv_target_markerless")
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"


class FrameHolder:
    """Latest-wins camera frame, decoded RGB->BGR for the seeker."""

    def __init__(self) -> None:
        self.count = 0
        self.bgr = None
        self.w = 0
        self.h = 0

    def on_image(self, msg: Image) -> None:
        try:
            w, h = msg.width, msg.height
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if arr.size < w * h * 3:
                return
            rgb = arr[: w * h * 3].reshape(h, w, 3)
            self.bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            self.w, self.h = w, h
            self.count += 1
        except Exception as exc:  # pragma: no cover - diagnostic robustness
            print(f"[probe] frame decode error: {exc}")


def teleport(x: float, y: float, z: float, timeout_s: float = 4.0) -> bool:
    """Teleport the target via a gz service CLI SUBPROCESS (subscription quirk)."""
    req = f'name: "{TARGET_MODEL}" position {{ x: {x} y: {y} z: {z} }}'
    cmd = [
        "gz", "service", "-s", SET_POSE_SERVICE,
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", req,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"[probe] teleport failed ({exc}) for range x={x}")
        return False
    if r.returncode != 0:
        print(f"[probe] teleport rc={r.returncode}: {r.stderr.strip()}")
        return False
    return True


def wait_new_frames(holder: FrameHolder, n: int, timeout_s: float = 8.0) -> bool:
    """Block until n new frames arrive after a teleport (settle transients)."""
    start_count = holder.count
    t0 = time.time()
    while holder.count < start_count + n:
        if time.time() - t0 > timeout_s:
            return False
        time.sleep(0.05)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ranges", default="2,3,4,5,7,9,12",
                    help="comma list of forward ranges (m) to probe")
    ap.add_argument("--y", type=float, default=0.0, help="target lateral offset (m)")
    ap.add_argument("--z", type=float, default=0.3, help="target height (m)")
    ap.add_argument("--out", default=os.path.join(REPO, "logs", "markerless_probe"),
                    help="output dir for frames + CSV")
    ap.add_argument("--settle-frames", type=int, default=8,
                    help="new frames to wait after each teleport before grabbing")
    ap.add_argument("--drone-weights", default=None,
                    help="optional path to a fine-tuned single-class ONNX to ALSO "
                         "test (e.g. weights/drone_yolo11x_1280.onnx). Best-effort; "
                         "decode differences are caught and reported, not fatal.")
    args = ap.parse_args()

    ranges = [float(v) for v in args.ranges.split(",") if v.strip()]
    os.makedirs(args.out, exist_ok=True)
    frames_dir = os.path.join(args.out, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    print(f"[probe] world={WORLD_NAME} target={TARGET_MODEL}")
    print(f"[probe] camera topic: {IMAGE_TOPIC}")
    print(f"[probe] set_pose service: {SET_POSE_SERVICE}")

    node = Node()
    holder = FrameHolder()
    if not node.subscribe(Image, IMAGE_TOPIC, holder.on_image):
        print(f"[probe] FAILED: could not subscribe to {IMAGE_TOPIC}")
        print("[probe] is PX4+gz running on the markerless world? (gz topic -l)")
        return 2

    # wait for the first frame so we know the camera is live
    print("[probe] waiting for first camera frame...")
    t0 = time.time()
    while holder.count == 0:
        if time.time() - t0 > 20.0:
            print("[probe] FAILED: no camera frames in 20 s.")
            return 2
        time.sleep(0.1)
    print(f"[probe] camera live: {holder.w}x{holder.h}")

    captures = []  # (range, png_path)
    for r in ranges:
        if not teleport(r, args.y, args.z):
            continue
        if not wait_new_frames(holder, args.settle_frames):
            print(f"[probe] WARNING: no fresh frames after teleport to r={r}")
            continue
        bgr = holder.bgr.copy()
        png = os.path.join(frames_dir, f"r{r:04.1f}.png")
        cv2.imwrite(png, bgr)
        captures.append((r, png))
        print(f"[probe] captured r={r:.1f} m -> {png}")

    node.unsubscribe(IMAGE_TOPIC)
    if not captures:
        print("[probe] FAILED: captured 0 frames.")
        return 2

    # ---- offline detection on the captured frames -------------------------
    # Primary: the EXACT TwoStageSeeker markerless_loop.py flies (default COCO
    # yolov8n NN verifier). This is the number that decides Option A.
    print("\n[probe] === TwoStageSeeker (default COCO nn -- the flight path) ===")
    seeker = TwoStageSeeker(FX, FY, CX, CY)
    rows = []
    for r, png in captures:
        bgr = cv2.imread(png)
        det = seeker.detect(bgr)
        hit = det.range_m is not None
        rows.append({
            "range_true_m": r,
            "detect": int(hit),
            "bearing_deg": (f"{det.bearing_rad*180/np.pi:.2f}" if hit else ""),
            "range_est_m": (f"{det.range_m:.2f}" if hit else ""),
            "conf": (f"{det.decision_margin:.3f}" if hit and det.decision_margin else ""),
        })
        status = (f"DETECT bearing={det.bearing_rad*180/np.pi:+.1f} deg "
                  f"range_est={det.range_m:.2f} m conf={det.decision_margin:.2f}"
                  if hit else "no detect (filter would coast)")
        print(f"  r={r:5.1f} m -> {status}")

    csv_path = os.path.join(args.out, "probe_results.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    n_hit = sum(rw["detect"] for rw in rows)
    hits = [rw["range_true_m"] for rw in rows if rw["detect"]]
    print(f"[probe] COCO two-stage: {n_hit}/{len(rows)} ranges fired; "
          f"max acquire range = {max(hits) if hits else 0:.1f} m")
    print(f"[probe] wrote {csv_path}")

    # Optional: a fine-tuned single-class ONNX (best-effort; see --drone-weights).
    if args.drone_weights:
        print(f"\n[probe] === best-effort: {os.path.basename(args.drone_weights)} ===")
        try:
            from nn_seeker import NNSeeker
            nn = NNSeeker(weights=args.drone_weights, conf_thres=0.08)
            seeker2 = TwoStageSeeker(FX, FY, CX, CY, nn=nn)
            for r, png in captures:
                bgr = cv2.imread(png)
                det = seeker2.detect(bgr)
                hit = det.range_m is not None
                print(f"  r={r:5.1f} m -> "
                      + (f"DETECT range_est={det.range_m:.2f} conf={det.decision_margin:.2f}"
                         if hit else "no detect"))
        except Exception as exc:
            print(f"[probe] drone-weights path errored (decode mismatch expected "
                  f"for non-yolov8 output): {exc}")

    print("\n[probe] DONE. Interpretation: if COCO two-stage fires only at very "
          "short range (<~3 m) or 0/N, the tag-less body needs an in-domain "
          "fine-tune (Option B) before the flight A/B is meaningful; the ground-"
          "rig mid-course cue (Option C) is what covers the acquisition gap.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
