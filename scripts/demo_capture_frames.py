#!/usr/bin/env python3
"""Demo-video frame-sequence capture: subscribe to a gz-transport camera
Image topic and (optionally) /clock, and save every received frame as
<out>/frame_%06d.png plus a manifest CSV (idx, wall_t, sim_t) so frames can
be aligned to an scripts/m4_intercept.py flight CSV's `t_sim` column.

NOT part of any gated milestone -- built for the portfolio demo video (see
scripts/compose_demo.sh, docs/demo_plan.md). Reuses
scripts/m1_capture.py's image_msg_to_bgr decode (same RGB_INT8,
no-row-padding wire-format assumption) rather than duplicating it.

WHY A SEPARATE PROCESS PER CAMERA (not folded into the flight script): this
mirrors scripts/m4_target_mover.py's "why its own process" reasoning, one
level down -- gz-transport13's "a subscribing process never gets service
RESPONSES" quirk only bites a process that also CALLS a gz service. This
script never calls one (image/clock are topics, subscribe-only), so it is
safe to run concurrently with scripts/m4_intercept.py (which itself
subscribes to the interceptor's own camera/pose) and with the mover/cue
subprocesses -- it just adds more passive listeners on the same running
sim, capturing the SAME flight the CSV is being logged from (no need to
re-fly for a second, possibly-diverging take).

Runs until SIGTERM/SIGINT (send it the same way scripts/m4_intercept.py
tears down its mover/cue subprocesses) or --max-frames/--timeout, whichever
comes first; flushes the manifest CSV and exits 0 on a clean stop.

Usage:
    .venv/bin/python scripts/demo_capture_frames.py \\
        --topic /world/apriltag_demo/model/x500_mono_cam_0/link/camera_link/sensor/imager/image \\
        --out demo_out/onboard_frames
"""
import argparse
import csv
import os
import queue
import signal
import sys
import time
from datetime import datetime, timezone

import cv2

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.clock_pb2 import Clock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m1_capture import image_msg_to_bgr  # noqa: E402  (reuse the decode, don't duplicate)

DEFAULT_CLOCK_TOPIC = "/clock"
DEFAULT_TIMEOUT_S = 180.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True, help="gz-transport camera Image topic")
    parser.add_argument("--out", required=True, help="output directory for frame_%%06d.png + manifest.csv")
    parser.add_argument("--clock-topic", default=DEFAULT_CLOCK_TOPIC,
                         help="gz-transport Clock topic for sim-time stamping (default %s)" % DEFAULT_CLOCK_TOPIC)
    parser.add_argument("--max-frames", type=int, default=None, help="stop after this many frames")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                         help="overall wall-clock safety timeout in seconds (default %.0f)" % DEFAULT_TIMEOUT_S)
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    manifest_path = os.path.join(args.out, "manifest.csv")

    frame_queue: "queue.Queue[Image]" = queue.Queue()
    sim_clock = {"t": None}

    def on_image(msg: Image) -> None:
        frame_queue.put(msg)

    def on_clock(msg: Clock) -> None:
        sim_clock["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    node = Node()
    subscribed = node.subscribe(Image, args.topic, on_image)
    if not subscribed:
        print(f"[demo_capture] FAILED: could not subscribe to {args.topic}")
        return 1
    node.subscribe(Clock, args.clock_topic, on_clock)  # best-effort; sim_t stays blank if this fails

    stop = {"flag": False}

    def _handle_signal(signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    print(f"[demo_capture] Subscribed to {args.topic}")
    print(f"[demo_capture] Saving frames + manifest to {args.out}")

    manifest = open(manifest_path, "w", newline="")
    writer = csv.writer(manifest)
    writer.writerow(["idx", "wall_t", "sim_t", "file"])

    saved = 0
    deadline = time.monotonic() + args.timeout
    t0_wall = time.monotonic()
    try:
        while not stop["flag"]:
            if args.max_frames is not None and saved >= args.max_frames:
                print(f"[demo_capture] Reached --max-frames={args.max_frames}, stopping.")
                break
            if time.monotonic() >= deadline:
                print(f"[demo_capture] Reached --timeout={args.timeout}s, stopping.")
                break
            try:
                msg = frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                bgr = image_msg_to_bgr(msg)
            except ValueError as exc:
                print(f"[demo_capture] WARNING: skipping undecodable frame: {exc}")
                continue

            fname = f"frame_{saved:06d}.png"
            fpath = os.path.join(args.out, fname)
            cv2.imwrite(fpath, bgr)
            wall_t = time.monotonic() - t0_wall
            sim_t = sim_clock["t"]
            writer.writerow([saved, f"{wall_t:.4f}", "" if sim_t is None else f"{sim_t:.4f}", fname])
            if saved % 30 == 0:
                manifest.flush()
            saved += 1
    finally:
        manifest.flush()
        manifest.close()
        node.unsubscribe(args.topic)
        try:
            node.unsubscribe(args.clock_topic)
        except Exception:
            pass

    print(f"[demo_capture] Done: {saved} frames -> {args.out} (manifest: {manifest_path})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
