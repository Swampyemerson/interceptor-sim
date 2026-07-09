#!/usr/bin/env python3
"""T17 item 3: capture EMPTY-WEDGE negative frames from the stereo rig.

Minimal variant of scripts/rig_snapshot_capture.py -- that harness's
build_samples()/dash-path loop assumes a MOVING target converging on a
boundary (infinite loop / runaway guard trips at dash_speed=0), so it can't
directly produce a "hold still far outside the wedge" negative burst. This
script reuses its exact subscribe/teleport/settle/decode/write pattern
(same FrameHolder, same `gz service` CLI-subprocess teleport -- this
process holds camera subscriptions so a same-process node.request() would
never see a service RESPONSE, the documented gz-transport13 quirk) but
loops over a small list of STATIC target positions, all outside the rig's
~10 deg half-angle wedge at the corridor row (see worlds/stereo_intercept.sdf
header), and grabs N frames per camera at each.

WHY 3 far positions x ~10 frames instead of 1x30: the whole point of a
negative set is "no drone anywhere in frame" scene diversity (sky/horizon/
ground clutter) -- one static position gives just ONE clutter scene, so more
positions vary what fraction of frame is sky vs. ground, without editing the
world's sun (no cheap sun-toggle service available from this harness; the
task's "else just capture the static empty scene frames" fallback explicitly
allows this simplification).

Positions (deliberately far outside the wedge that covers the y in
[-28.2, 28.2] corridor at x=6.5, ~160 m downrange, per
worlds/stereo_intercept.sdf's wedge-coverage comment):
  - (0, 200, 0.5)   -- far off to +Y, well past the wedge edge
  - (0, -200, 0.5)  -- far off to -Y, mirror
  - (6.5, 0, 60.0)  -- straight up the corridor bearing but way above the
                        camera's ~10 deg vertical half-angle at this range
                        (60 m altitude vs. camera at 1.8 m over a ~160 m
                        range is >> 10 deg elevation), so it's off-frame too

RUN (sim already up on stereo_intercept, GALLIUM_DRIVER=d3d12 recommended,
see the boot block in the companion capture_rig_negatives_boot.sh):
    INTERCEPTOR_WORLD_NAME=stereo_intercept \\
    INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \\
    .venv-seeker/bin/python capture_rig_negatives.py \\
        --out logs/rig_captures/negatives_<runstamp> --n-per-pos 10
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np

REPO_ROOT = "/home/emerson/interceptor-sim"
SCRIPTS_DIR = os.path.join(REPO_ROOT, "scripts")
os.environ.setdefault("INTERCEPTOR_WORLD_NAME", "stereo_intercept")
os.environ.setdefault("INTERCEPTOR_TARGET_MODEL", "fpv_target_markerless")
sys.path.insert(0, SCRIPTS_DIR)

import cv2  # noqa: E402

from m1_capture import image_msg_to_bgr  # noqa: E402
from gz.transport13 import Node  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402

WORLD_NAME = os.environ["INTERCEPTOR_WORLD_NAME"]
TARGET_MODEL_NAME = os.environ["INTERCEPTOR_TARGET_MODEL"]
RIG_MODEL_NAME = "ground_stereo_rig"
RIG_LINK_NAME = "link"

LEFT_TOPIC = f"/world/{WORLD_NAME}/model/{RIG_MODEL_NAME}/link/{RIG_LINK_NAME}/sensor/rig_left/image"
RIGHT_TOPIC = f"/world/{WORLD_NAME}/model/{RIG_MODEL_NAME}/link/{RIG_LINK_NAME}/sensor/rig_right/image"
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"

NEG_POSITIONS = [
    ("far_plus_y", 0.0, 200.0, 0.5),
    ("far_minus_y", 0.0, -200.0, 0.5),
    ("far_up", 6.5, 0.0, 60.0),
]

# FOUND DURING DEV (report this): `make px4_sitl gz_x500_mono_cam` spawns the
# INTERCEPTOR airframe near world origin (~0,0,~) independent of the world
# SDF's own model list -- that puts it at bearing ~0deg, ~153.5 m range from
# the rig, i.e. dead center of the rig's 10deg half-angle wedge, REGARDLESS
# of where the target is teleported. First negative-capture attempt (target
# only moved) showed a small drone-shaped blob at the horizon in every
# frame -- traced to the parked interceptor, not the target. Per ADR-0047
# the interceptor airframe must NOT be in the training set at all (neither
# positive nor hard negative), so it must be kept OUT of frame here. Fix:
# yaw the (static) rig itself 90 deg away from the corridor for the
# negative burst -- same "teleport a static model via `gz service`" pattern
# already used for the target throughout T16/T17 (ground_stereo_rig is
# static=true, exactly like fpv_target_markerless) -- then restore it.
RIG_MODEL = "ground_stereo_rig"
RIG_HOME_XYZ_YAW = (-153.5, 0.0, 1.8, 0.0)
RIG_NEG_YAW = 1.5707963267948966  # +90 deg: boresight along world +Y, clear of the corridor

TELEPORT_TIMEOUT_S = 4.0
SETTLE_TIMEOUT_S = 8.0
DEFAULT_SETTLE_FRAMES = 2


class FrameHolder:
    def __init__(self, label: str) -> None:
        self.label = label
        self.count = 0
        self.bgr = None

    def on_image(self, msg: Image) -> None:
        try:
            self.bgr = image_msg_to_bgr(msg)
            self.count += 1
        except Exception as exc:  # pragma: no cover
            print(f"[neg-capture] {self.label} frame decode error: {exc}")


def teleport(x, y, z, timeout_s=TELEPORT_TIMEOUT_S) -> bool:
    req = f'name: "{TARGET_MODEL_NAME}" position {{ x: {x} y: {y} z: {z} }}'
    cmd = ["gz", "service", "-s", SET_POSE_SERVICE, "--reqtype", "gz.msgs.Pose",
           "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"[neg-capture] teleport failed ({exc})")
        return False
    return r.returncode == 0


def teleport_rig(x, y, z, yaw, timeout_s=TELEPORT_TIMEOUT_S) -> bool:
    """Teleport the (static) ground_stereo_rig model itself -- keeps the
    parked interceptor (spawned at ~world origin by `make px4_sitl
    gz_x500_mono_cam`, see the module-level comment) out of the wedge for
    negative captures. Same CLI-subprocess set_pose pattern as teleport()."""
    qz, qw = np.sin(yaw / 2.0), np.cos(yaw / 2.0)
    req = (f'name: "{RIG_MODEL}" position {{ x: {x} y: {y} z: {z} }} '
           f'orientation {{ x: 0 y: 0 z: {qz} w: {qw} }}')
    cmd = ["gz", "service", "-s", SET_POSE_SERVICE, "--reqtype", "gz.msgs.Pose",
           "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"[neg-capture] rig teleport failed ({exc})")
        return False
    return r.returncode == 0


def wait_new_frames(holder: FrameHolder, n: int, timeout_s=SETTLE_TIMEOUT_S) -> bool:
    start = holder.count
    t0 = time.time()
    while holder.count < start + n:
        if time.time() - t0 > timeout_s:
            return False
        time.sleep(0.03)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None)
    ap.add_argument("--n-per-pos", type=int, default=10)
    ap.add_argument("--settle-frames", type=int, default=DEFAULT_SETTLE_FRAMES)
    args = ap.parse_args()

    runstamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.out or os.path.join(REPO_ROOT, "logs", "rig_captures", f"negatives_{runstamp}")
    left_dir = os.path.join(out_dir, "left")
    right_dir = os.path.join(out_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    print(f"[neg-capture] world={WORLD_NAME} target={TARGET_MODEL_NAME} rig={RIG_MODEL_NAME}")
    print(f"[neg-capture] out={out_dir} n_per_pos={args.n_per_pos} positions={NEG_POSITIONS}")

    node = Node()
    left_holder = FrameHolder("rig_left")
    right_holder = FrameHolder("rig_right")
    if not node.subscribe(Image, LEFT_TOPIC, left_holder.on_image):
        print(f"[neg-capture] FAILED: subscribe {LEFT_TOPIC}"); return 2
    if not node.subscribe(Image, RIGHT_TOPIC, right_holder.on_image):
        print(f"[neg-capture] FAILED: subscribe {RIGHT_TOPIC}"); return 2

    print("[neg-capture] waiting for first frame on both rig cameras...")
    t0 = time.time()
    while left_holder.count == 0 or right_holder.count == 0:
        if time.time() - t0 > 20:
            print("[neg-capture] FAILED: no frames within 20s"); return 2
        time.sleep(0.1)

    # Yaw the rig itself 90 deg away from the corridor so the parked
    # interceptor (world origin, dead center of the home-pose wedge) is
    # NOT in view -- see the RIG_NEG_YAW comment above.
    rx, ry, rz, _home_yaw = RIG_HOME_XYZ_YAW
    print(f"[neg-capture] yawing rig to {RIG_NEG_YAW:.4f} rad "
          f"(clear of the parked interceptor) for the negative burst...")
    if not teleport_rig(rx, ry, rz, RIG_NEG_YAW):
        print("[neg-capture] FAILED: rig teleport did not succeed"); return 2
    if not (wait_new_frames(left_holder, args.settle_frames)
            and wait_new_frames(right_holder, args.settle_frames)):
        print("[neg-capture] FAILED: settle timeout after rig teleport"); return 2

    index_rows = []
    seq = 0
    n_ok = n_skipped = 0
    for pos_name, x, y, z in NEG_POSITIONS:
        if not teleport(x, y, z):
            print(f"[neg-capture] {pos_name}: teleport FAILED @ ({x},{y},{z}) -- skipping whole burst")
            n_skipped += args.n_per_pos
            continue
        # settle once after the teleport, then grab n_per_pos successive frames
        if not (wait_new_frames(left_holder, args.settle_frames)
                and wait_new_frames(right_holder, args.settle_frames)):
            print(f"[neg-capture] {pos_name}: settle timeout -- skipping whole burst")
            n_skipped += args.n_per_pos
            continue
        for i in range(args.n_per_pos):
            settle_ok = (wait_new_frames(left_holder, 1) and wait_new_frames(right_holder, 1))
            if not settle_ok:
                print(f"[neg-capture] {pos_name} frame {i}: settle timeout -- skipping")
                n_skipped += 1
                seq += 1
                continue
            left_bgr, right_bgr = left_holder.bgr, right_holder.bgr
            if left_bgr is None or right_bgr is None:
                n_skipped += 1
                seq += 1
                continue
            left_path = os.path.join(left_dir, f"{seq:05d}.png")
            right_path = os.path.join(right_dir, f"{seq:05d}.png")
            cv2.imwrite(left_path, left_bgr)
            cv2.imwrite(right_path, right_bgr)
            index_rows.append({
                "seq": seq, "pos_name": pos_name,
                "gt_target_x": f"{x:.4f}", "gt_target_y": f"{y:.4f}", "gt_target_z": f"{z:.4f}",
            })
            n_ok += 1
            seq += 1

    print(f"[neg-capture] restoring rig to home pose {RIG_HOME_XYZ_YAW}...")
    teleport_rig(rx, ry, rz, _home_yaw)

    node.unsubscribe(LEFT_TOPIC)
    node.unsubscribe(RIGHT_TOPIC)

    index_path = os.path.join(out_dir, "index.csv")
    with open(index_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=["seq", "pos_name", "gt_target_x", "gt_target_y", "gt_target_z"])
        writer.writeheader()
        writer.writerows(index_rows)

    meta = {
        "world": WORLD_NAME, "target_model": TARGET_MODEL_NAME, "rig_model": RIG_MODEL_NAME,
        "positions": NEG_POSITIONS, "n_per_pos": args.n_per_pos,
        "n_captured": n_ok, "n_skipped": n_skipped, "runstamp": runstamp,
        "note": "target teleported OUTSIDE the rig's wedge coverage for every "
                "position -- these frames should contain no drone (empty-wedge negatives).",
    }
    with open(os.path.join(out_dir, "capture_meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)

    print(f"[neg-capture] wrote {n_ok} L/R pairs ({n_skipped} skipped) -> {out_dir}")
    return 0 if n_ok > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
