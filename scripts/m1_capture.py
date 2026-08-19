#!/usr/bin/env python3
"""M1 gate script: capture real camera frames from gz_x500_mono_cam via gz-transport.

What: subscribes to the mono camera's Gazebo Transport image topic (no ROS —
see docs/goals.md "IS NOT: No ROS 2"), decodes each Image protobuf into a numpy
array, and saves the first N frames to disk as PNGs. This is the smallest
possible proof that the camera-rendering pipeline (Gazebo -> gz-transport ->
Python/OpenCV) works end to end before we build AprilTag detection on top of
it. See docs/goals.md milestone M1.

Why gz-transport directly: the project deliberately avoids ROS 2 to keep the
dependency surface small (docs/goals.md). `gz.transport13` is Gazebo Harmonic's own
pub/sub Python binding; `gz.msgs10` provides the matching protobuf message
types (Image, CameraInfo, ...).

Threading note: gz-transport subscribe() callbacks fire on a background
C++ thread, not the Python main thread. We hand each received frame to the
main thread via a thread-safe queue.Queue and just wait on it here, matching
the pattern MAVSDK scripts in this repo use with asyncio tasks — one
producer, one consumer, no shared mutable state without synchronization.

Exit code 0 only if: N frames captured within the timeout, every frame's
(width, height) matches the SDF-declared resolution for mono_cam
(EXPECTED_WIDTH x EXPECTED_HEIGHT, see
~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf), and every
frame has real image content (pixel stddev > MIN_STDDEV — rules out an
all-black or all-one-color frame, which would mean the renderer produced
nothing useful even though messages are flowing). Exit 1 otherwise, with a
clear reason printed.

Run manually (with PX4 SITL + gz_x500_mono_cam already running):

    .venv/bin/python scripts/m1_capture.py --frames 10

Normally this is invoked by scripts/check_m1.sh, which also launches and
tears down the simulator.
"""

import argparse
import os
import queue
import sys
import time
from datetime import datetime, timezone

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image, RGB_INT8

# From ~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf:
#   <camera><horizontal_fov>1.74</horizontal_fov><image><width>1280</width>
#   <height>960</height></image></camera>, <update_rate>30</update_rate>.
# See docs/decisions.md for how this feeds camera-intrinsics work in M2.
EXPECTED_WIDTH = 1280
EXPECTED_HEIGHT = 960

IMAGE_TOPIC = (
    "/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image"
)
DEFAULT_FRAMES = 10
DEFAULT_TIMEOUT_S = 60.0
# Below this stddev, a frame is effectively flat (all-black / all-one-color)
# -- i.e. messages are flowing but the renderer produced no real content.
MIN_STDDEV = 1.0

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Decode a gz.msgs10 Image into an OpenCV-ready BGR numpy array.

    The mono_cam sensor is declared with no <format> override, and the wire
    message we observed (gz topic -e) reports pixel_format_type: RGB_INT8,
    i.e. 3 interleaved bytes per pixel, row-major, no padding beyond
    step == width * 3. We only handle that format here; anything else is a
    configuration change worth failing loudly on rather than silently
    mis-decoding.
    """
    if msg.pixel_format_type != RGB_INT8:
        raise ValueError(
            f"unexpected pixel_format_type {msg.pixel_format_type} "
            f"(expected RGB_INT8={RGB_INT8}); update image_msg_to_bgr if the "
            "sensor config changed"
        )

    expected_step = msg.width * 3
    if msg.step != expected_step:
        raise ValueError(
            f"unexpected step {msg.step} for width {msg.width} "
            f"(expected {expected_step}, i.e. no row padding)"
        )

    rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        (msg.height, msg.width, 3)
    )
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--frames", type=int, default=DEFAULT_FRAMES, help="number of frames to capture"
    )
    parser.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="overall timeout in seconds"
    )
    parser.add_argument(
        "--topic", type=str, default=IMAGE_TOPIC, help="gz-transport image topic to subscribe to"
    )
    args = parser.parse_args()

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = os.path.join(LOGS_DIR, f"m1_frames_{timestamp}")
    os.makedirs(out_dir, exist_ok=True)

    print(f"[m1] Subscribing to {args.topic}")
    print(f"[m1] Saving frames to {out_dir}")

    frame_queue: "queue.Queue[Image]" = queue.Queue()

    def on_image(msg: Image) -> None:
        # Called on a gz-transport background thread -- just hand the
        # message off to the main thread via the thread-safe queue.
        frame_queue.put(msg)

    node = Node()
    subscribed = node.subscribe(Image, args.topic, on_image)
    if not subscribed:
        print(f"[m1] FAILED: could not subscribe to topic {args.topic}")
        return 1

    saved = 0
    deadline = time.monotonic() + args.timeout
    try:
        while saved < args.frames:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                print(
                    f"[m1] FAILED: timed out after {args.timeout}s with only "
                    f"{saved}/{args.frames} frames captured"
                )
                return 1

            try:
                msg = frame_queue.get(timeout=min(remaining, 1.0))
            except queue.Empty:
                continue

            if msg.width != EXPECTED_WIDTH or msg.height != EXPECTED_HEIGHT:
                print(
                    f"[m1] FAILED: frame {saved} resolution {msg.width}x{msg.height} "
                    f"!= expected {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}"
                )
                return 1

            try:
                bgr = image_msg_to_bgr(msg)
            except ValueError as exc:
                print(f"[m1] FAILED: {exc}")
                return 1

            stddev = float(bgr.astype(np.float64).std())
            if stddev <= MIN_STDDEV:
                print(
                    f"[m1] FAILED: frame {saved} pixel stddev {stddev:.3f} <= "
                    f"{MIN_STDDEV} (looks like a flat / blank render, not a "
                    "real scene)"
                )
                return 1

            frame_path = os.path.join(out_dir, f"frame_{saved:03d}.png")
            cv2.imwrite(frame_path, bgr)
            print(
                f"[m1] Frame {saved}: {msg.width}x{msg.height}, "
                f"stddev={stddev:.2f} -> {frame_path}"
            )
            saved += 1

        print(
            f"[m1] Captured {saved}/{args.frames} frames at "
            f"{EXPECTED_WIDTH}x{EXPECTED_HEIGHT} with real content. "
            f"M1 camera capture check PASSED. Frames in {out_dir}"
        )
        return 0
    finally:
        node.unsubscribe(args.topic)


if __name__ == "__main__":
    sys.exit(main())
