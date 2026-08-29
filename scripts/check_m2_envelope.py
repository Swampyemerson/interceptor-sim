#!/usr/bin/env python3
"""M2 detection-ENVELOPE sweep: measure the max reliable AprilTag detection
range with the current camera lens, instead of assuming one. See docs/goals.md
milestone M2, ADR-0007 (the old "~6 m" figure this replaces -- it was ad
hoc, never measured with a script), and the ADR-0024 addendum (Tier-2 60
deg lens change this re-baselines: fx 539.9 -> ~1108.5 px, predicted
detection range ~6 m -> ~12 m).

WHAT: with PX4 SITL + gz_x500_mono_cam + the "apriltag" world already
running (scripts/check_m2.sh's launch recipe -- the tag sits fixed at
world (5, 0, 0.5) facing -X, see worlds/apriltag.sdf), reposition the
INTERCEPTOR (not the tag) along the tag's boresight to a series of
standoff ranges and, at each one, sample live camera frames through the
SAME detector + ground-truth chain as scripts/m2_detect.py (ADR-0006):
pupil-apriltags/pyapriltags (scripts/apriltag_detector.py) for the
measured pose, and the world pose topic for ground truth. This is a pure
perception-envelope check -- no MAVSDK, no flight, no PX4 arming needed
(mirrors check_m2.sh's own approach: the drone just sits on the ground
plane at whatever height it spawns at, teleported to a new x each step).

WHY A SEPARATE MOVER PROCESS (do not "simplify" this away): the SAME
gz-transport13 quirk documented in scripts/m4_target_mover.py bites here --
a process holding ANY active topic subscription (this script subscribes to
Image/CameraInfo/Pose_V) never receives `node.request(...)` service
responses again, even though the request is still applied server-side. So
the `set_pose` calls that reposition the drone are sent from a small,
subscription-free CHILD process (MOVER_HELPER_SRC below), talked to over a
stdin/stdout line protocol ("x,y,z\\n" in, "OK"/"FAIL" out).

WHY WALL TIME HERE IS FINE (contrast with the mover/movers-use-sim-time
rule, ADR-0009): that rule is about scheduling a MOVING target's velocity
profile or measuring guidance latencies, where RTF sag silently rescales a
commanded speed or a miss-distance timing window. This script's "how long
to sample at a static range" duration is not doing either -- it is just
"wait for some frames" -- and m2_detect.py itself already uses wall time
for its own steady-state sampling window. Still, per the audit gate, this
must ONLY be run at IDLE machine load (checked by the caller) so RTF stays
~1.0 and the sample-count/settle-time budget below stays valid.

Run manually (with the check_m2.sh sim already up):

    .venv/bin/python scripts/check_m2_envelope.py

Writes logs/m2_envelope_<UTC timestamp>.csv (one row per sampled frame,
same schema as m2_detect.py's CSV plus a `range_cmd_m` column) and prints a
per-range summary table plus the max reliable range found.
"""

import argparse
import csv
import os
import queue
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m2_detect import (  # noqa: E402  (path insert must come first)
    CAMERA_INFO_TIMEOUT_S,
    DETECTION_RATE_THRESHOLD,
    DRONE_MODEL,
    EXPECTED_HEIGHT,
    EXPECTED_WIDTH,
    IMAGE_TOPIC,
    MEAN_ERR_NORM_THRESHOLD_M,
    POSE_TOPIC,
    TAG_FAMILY,
    TAG_SIZE_M,
    PoseTracker,
    get_camera_intrinsics,
    image_msg_to_gray,
)
from apriltag_detector import Detector  # ADR-0012 portable import

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V

# Tag world pose, from worlds/apriltag.sdf: <pose>5 0 0.5 0 0 0</pose>,
# facing -X. The drone's own default spawn orientation is identity (facing
# +X, per m2_detect.py's transform-chain docstring), so parking it at
# (TAG_X - range, y0, z0) with identity orientation points its camera
# straight down the tag's boresight -- no orientation field needs setting
# on the set_pose request (same reasoning m4_target_mover.py uses for the
# tag itself).
TAG_X = 5.0
TAG_Y = 0.0

# The predicted envelope (ADR-0024 addendum) is ~12 m; bracket well past it
# so a FAIL point is actually observed, not just assumed.
DEFAULT_SWEEP_RANGES_M = [4.0, 6.0, 8.0, 10.0, 12.0, 14.0]

SETTLE_S = 1.5          # wall-clock settle time after each teleport
SAMPLE_DURATION_S = 3.0  # wall-clock sampling window per range (idle machine, RTF~1)
MOVE_TIMEOUT_S = 5.0

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")

WORLD_NAME = "apriltag"
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"

# Subscription-free child process (see module docstring for why). Reads
# "x,y,z\n" lines from stdin, issues one set_pose request per line, prints
# "OK"/"FAIL". Deliberately standalone (no shared imports with the parent)
# -- same isolation philosophy as m4_target_mover.py's CLOCK_HELPER_SRC.
MOVER_HELPER_SRC = f"""
import sys
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

SET_POSE_SERVICE = {SET_POSE_SERVICE!r}
DRONE_MODEL = {DRONE_MODEL!r}
TIMEOUT_MS = 3000

node = Node()
for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    x, y, z = (float(v) for v in line.split(","))
    req = Pose()
    req.name = DRONE_MODEL
    req.position.x = x
    req.position.y = y
    req.position.z = z
    ok, resp = node.request(SET_POSE_SERVICE, req, Pose, Boolean, TIMEOUT_MS)
    print("OK" if (ok and resp.data) else "FAIL", flush=True)
"""


class DroneMover:
    """Thin wrapper around the subscription-free mover child process."""

    def __init__(self):
        self.proc = subprocess.Popen(
            [sys.executable, "-c", MOVER_HELPER_SRC],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1,
        )

    def move(self, x: float, y: float, z: float, timeout_s: float) -> bool:
        self.proc.stdin.write(f"{x},{y},{z}\n")
        self.proc.stdin.flush()
        # Local IPC to a same-host gz service; the child's own request has a
        # 3s internal timeout, so a generous wall deadline here just guards
        # against a wedged child, not normal latency.
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = self.proc.stdout.readline()
            if line:
                return line.strip() == "OK"
        return False

    def close(self):
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        self.proc.terminate()
        try:
            self.proc.wait(timeout=3)
        except Exception:
            self.proc.kill()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ranges", default=",".join(str(r) for r in DEFAULT_SWEEP_RANGES_M),
        help="comma-separated list of standoff ranges (m) to sweep",
    )
    args = parser.parse_args()
    sweep_ranges = [float(r) for r in args.ranges.split(",")]

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = os.path.join(LOGS_DIR, f"m2_envelope_{timestamp}.csv")

    node = Node()

    print(f"[m2-envelope] Reading camera intrinsics from {IMAGE_TOPIC} sibling CameraInfo ...")
    fx, fy, cx, cy, width, height = get_camera_intrinsics(node, CAMERA_INFO_TIMEOUT_S)
    print(f"[m2-envelope] Intrinsics: fx={fx:.3f} fy={fy:.3f} cx={cx:.1f} cy={cy:.1f} ({width}x{height})")
    if width != EXPECTED_WIDTH or height != EXPECTED_HEIGHT:
        print(f"[m2-envelope] FAILED: resolution {width}x{height} != expected {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}")
        return 1

    tracker = PoseTracker()
    if not node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v):
        print(f"[m2-envelope] FAILED: could not subscribe to {POSE_TOPIC}")
        return 1

    frame_queue: "queue.Queue[Image]" = queue.Queue()

    def on_image(msg: Image) -> None:
        # Keep only the freshest frame -- avoids sampling a backlog of
        # stale frames from before a teleport settled.
        try:
            while True:
                frame_queue.get_nowait()
        except queue.Empty:
            pass
        frame_queue.put(msg)

    if not node.subscribe(Image, IMAGE_TOPIC, on_image):
        print(f"[m2-envelope] FAILED: could not subscribe to {IMAGE_TOPIC}")
        node.unsubscribe(POSE_TOPIC)
        return 1

    print(f"[m2-envelope] Waiting for the pose topic to populate...")
    deadline = time.monotonic() + 30.0
    while not tracker.latest and time.monotonic() < deadline:
        time.sleep(0.1)
    if DRONE_MODEL not in tracker.latest:
        print(f"[m2-envelope] FAILED: no pose data for {DRONE_MODEL} within 30s")
        return 1

    # Baseline y/z from wherever the drone actually spawned (same height
    # check_m2.sh's own run uses) -- only x (range along the tag's
    # boresight) is varied, so height/lateral offset stay a constant,
    # controlled quantity across the sweep.
    p0, _ = tracker.latest[DRONE_MODEL]
    y0, z0 = float(p0[1]), float(p0[2])
    print(f"[m2-envelope] Baseline spawn pose: x={p0[0]:.3f} y={y0:.3f} z={z0:.3f}")

    mover = DroneMover()
    detector = Detector(families=TAG_FAMILY)

    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(
        ["range_cmd_m", "t", "detected", "meas_x", "meas_y", "meas_z",
         "gt_x", "gt_y", "gt_z", "err_norm", "decision_margin"]
    )

    results = []
    try:
        for r in sweep_ranges:
            x = TAG_X - r
            print(f"[m2-envelope] --- range={r:.1f} m -> drone pose ({x:.2f}, {y0:.2f}, {z0:.2f}) ---")
            ok = mover.move(x, y0, z0, MOVE_TIMEOUT_S)
            if not ok:
                print(f"[m2-envelope] WARNING: set_pose request did not confirm OK for range={r} m")

            time.sleep(SETTLE_S)
            # Drain one more time right before sampling so the settle
            # window's frames don't leak into the sample window.
            try:
                while True:
                    frame_queue.get_nowait()
            except queue.Empty:
                pass

            n_frames = 0
            n_detected = 0
            err_norms = []
            sample_deadline = time.monotonic() + SAMPLE_DURATION_S
            while time.monotonic() < sample_deadline:
                remaining = sample_deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    msg = frame_queue.get(timeout=min(remaining, 0.5))
                except queue.Empty:
                    continue

                t = time.monotonic() - (sample_deadline - SAMPLE_DURATION_S)
                gt_rel, gt_ok, gt_reason = tracker.ground_truth_rel_optical()
                if not gt_ok:
                    print(f"[m2-envelope] FAILED: {gt_reason}")
                    return 1
                if msg.width != EXPECTED_WIDTH or msg.height != EXPECTED_HEIGHT:
                    continue
                gray = image_msg_to_gray(msg)
                detections = detector.detect(
                    gray, estimate_tag_pose=True,
                    camera_params=(fx, fy, cx, cy), tag_size=TAG_SIZE_M,
                )
                n_frames += 1
                if detections:
                    det = detections[0]
                    meas = det.pose_t.flatten()
                    err_norm = float(np.linalg.norm(meas - gt_rel))
                    n_detected += 1
                    err_norms.append(err_norm)
                    writer.writerow(
                        [f"{r:.1f}", f"{t:.3f}", 1,
                         f"{meas[0]:.4f}", f"{meas[1]:.4f}", f"{meas[2]:.4f}",
                         f"{gt_rel[0]:.4f}", f"{gt_rel[1]:.4f}", f"{gt_rel[2]:.4f}",
                         f"{err_norm:.4f}", f"{det.decision_margin:.3f}"]
                    )
                else:
                    writer.writerow(
                        [f"{r:.1f}", f"{t:.3f}", 0, "", "", "",
                         f"{gt_rel[0]:.4f}", f"{gt_rel[1]:.4f}", f"{gt_rel[2]:.4f}", "", ""]
                    )
                log_file.flush()

            detection_rate = n_detected / n_frames if n_frames else 0.0
            mean_err = float(np.mean(err_norms)) if err_norms else float("nan")
            reliable = (
                n_frames > 0
                and detection_rate >= DETECTION_RATE_THRESHOLD
                and err_norms
                and mean_err <= MEAN_ERR_NORM_THRESHOLD_M
            )
            results.append((r, n_frames, n_detected, detection_rate, mean_err, reliable))
            print(
                f"[m2-envelope] range={r:.1f} m n_frames={n_frames} "
                f"detection_rate={detection_rate:.3f} mean_err_norm={mean_err:.4f} m "
                f"reliable={reliable}"
            )
    finally:
        mover.close()
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        log_file.close()

    print("\n[m2-envelope] === Summary ===")
    print(f"{'range_m':>8} {'n_frames':>9} {'n_det':>6} {'det_rate':>9} {'mean_err_m':>11} {'reliable':>9}")
    max_reliable = None
    for r, n_frames, n_detected, detection_rate, mean_err, reliable in results:
        print(f"{r:8.1f} {n_frames:9d} {n_detected:6d} {detection_rate:9.3f} {mean_err:11.4f} {str(reliable):>9}")
        if reliable:
            max_reliable = r
    if max_reliable is not None:
        print(f"[m2-envelope] Max reliable detection range in this sweep: {max_reliable:.1f} m "
              f"(thresholds: detection_rate >= {DETECTION_RATE_THRESHOLD}, "
              f"mean_err_norm <= {MEAN_ERR_NORM_THRESHOLD_M} m)")
    else:
        print("[m2-envelope] No sweep point met the reliability thresholds.")
    print(f"[m2-envelope] Log written to {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
