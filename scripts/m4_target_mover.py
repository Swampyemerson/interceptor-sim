#!/usr/bin/env python3
"""M4 target mover: streams the AprilTag along a straight-line path by
repeatedly calling Gazebo's `/world/apriltag/set_pose` service. See GOALS.md
milestone M4 (moving-target intercept) and scripts/m4_intercept.py, which
spawns this script as a SEPARATE OS process during its ENGAGE phase.

WHY THIS IS ITS OWN PROCESS (the one thing not to "simplify" away): while
building M4 we found -- empirically, the hard way -- a real quirk in the
gz-transport13 Python bindings: a process that holds ANY active topic
SUBSCRIPTION (like m4_intercept.py's camera/pose subscriptions) never
receives service RESPONSES again -- `node.request(...)` just times out,
even though the request is still applied server-side (the tag really does
move; you just never hear back "yes, I moved it"). Since we need to know
the request succeeded (to count failures and fail loudly if the service
goes away), the mover has to run with ZERO subscriptions of its own -- so
it lives in this small, separate, subscription-free script instead of
being folded into m4_intercept.py's asyncio loop.

WHAT: parses a start position and a constant 2D velocity, then streams
`set_pose` requests at a fixed rate for a fixed duration. Positions are
computed from ELAPSED WALL-CLOCK TIME since the stream started (position =
start + velocity * t), not accumulated tick-by-tick -- so a slow tick (GC
pause, scheduler jitter) shifts WHEN a position is sent, never WHAT
position is sent for a given elapsed time. That keeps the path itself
exact even if the loop can't hit its target rate perfectly.

Every commanded position is logged to logs/m4_mover_<UTC timestamp>.csv
(t, x, y, z) so the guidance side's ground truth can be cross-checked
against exactly what this process asked Gazebo to do.

Run manually (with PX4 SITL + gz_x500_mono_cam + the apriltag world already
running, tag already placed near --start so the pre-warm request has
something sane to confirm against):

    .venv/bin/python scripts/m4_target_mover.py --start "6.5,-4,0.5" --vel "0,2.0" --duration 12

Normally this is spawned automatically by scripts/m4_intercept.py's ENGAGE
phase (see that file), using the same venv interpreter (sys.executable).
"""

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime, timezone

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

# Mirrors scripts/m2_detect.py's WORLD_NAME / TAG_MODEL_NAME -- not imported
# from there on purpose: this script is meant to stay small and completely
# standalone (no shared subscription state, no shared anything), since its
# entire reason for existing is to be a minimal, subscription-free process.
WORLD_NAME = "apriltag"
TARGET_MODEL_NAME = "apriltag_target"
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"

PREWARM_TIMEOUT_MS = 2000
STREAM_TIMEOUT_MS = 500
# ">5 consecutive failures" per the M4 task spec -- the 6th consecutive
# failure trips the abort.
MAX_CONSECUTIVE_FAILURES = 5
CSV_FLUSH_EVERY = 25

DEFAULT_START = "6.5,-4,0.5"
DEFAULT_VEL = "0,2.0"
DEFAULT_RATE_HZ = 50.0
DEFAULT_DURATION_S = 12.0

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")


def _parse_floats(value: str, n: int, flag_name: str):
    """Parse a comma-separated string of exactly n floats, e.g. "6.5,-4,0.5"."""
    parts = value.split(",")
    if len(parts) != n:
        raise ValueError(
            f"--{flag_name} needs {n} comma-separated numbers, got {value!r}"
        )
    try:
        return tuple(float(p) for p in parts)
    except ValueError as exc:
        raise ValueError(f"--{flag_name} value {value!r} is not all numbers") from exc


def send_pose(node: Node, x: float, y: float, z: float, timeout_ms: int) -> bool:
    """One `set_pose` request. Orientation is deliberately omitted (left at
    the message default, i.e. identity) -- the model's world orientation
    for apriltag_target is identity and its facing is baked into the model
    geometry itself (see models/apriltag_target/model.sdf), so there is
    nothing to set here besides position. Returns True iff the service
    replied AND its Boolean payload was true."""
    req = Pose()
    req.name = TARGET_MODEL_NAME
    req.position.x = x
    req.position.y = y
    req.position.z = z
    ok, response = node.request(SET_POSE_SERVICE, req, Pose, Boolean, timeout_ms)
    return bool(ok and response.data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--start", default=DEFAULT_START,
        help="start position 'x,y,z' in meters (world/ENU)",
    )
    parser.add_argument(
        "--vel", default=DEFAULT_VEL,
        help="constant horizontal velocity 'vx,vy' in m/s",
    )
    parser.add_argument(
        "--rate", type=float, default=DEFAULT_RATE_HZ,
        help="set_pose stream rate in Hz",
    )
    parser.add_argument(
        "--duration", type=float, default=DEFAULT_DURATION_S,
        help="how long to stream, in seconds",
    )
    args = parser.parse_args()

    try:
        x0, y0, z0 = _parse_floats(args.start, 3, "start")
        vx, vy = _parse_floats(args.vel, 2, "vel")
    except ValueError as exc:
        print(f"[m4-mover] FAILED: {exc}")
        return 2

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = os.path.join(LOGS_DIR, f"m4_mover_{timestamp}.csv")

    # Install signal handlers before anything blocking (the pre-warm
    # request) so a SIGTERM/SIGINT arriving even that early still gets
    # picked up by the streaming loop below rather than killing us with
    # Python's default disposition.
    stop = {"flag": False, "signaled": False}

    def _handle_signal(signum, _frame):
        stop["flag"] = True
        stop["signaled"] = True

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    node = Node()

    print(
        f"[m4-mover] Pre-warming {SET_POSE_SERVICE} (placing tag at "
        f"start=({x0}, {y0}, {z0}))..."
    )
    if not send_pose(node, x0, y0, z0, PREWARM_TIMEOUT_MS):
        print(
            f"[m4-mover] FAILED: pre-warm request to {SET_POSE_SERVICE} did "
            "not succeed (service not up, or model name wrong?)"
        )
        return 2

    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["t", "x", "y", "z"])
    log_file.flush()

    # This exact line is a handshake: m4_intercept.py's ENGAGE phase spawns
    # this process and may watch stdout for it before treating the mover as
    # "really" streaming -- so it must print ONLY after the pre-warm
    # request above has actually succeeded, never before.
    print("[m4-mover] streaming started")

    n_requests = 0
    n_failures = 0
    consecutive_failures = 0
    final_y = y0
    dt = 1.0 / args.rate
    stream_start = time.monotonic()
    exit_code = 0

    try:
        while not stop["flag"]:
            elapsed = time.monotonic() - stream_start
            if elapsed >= args.duration:
                break

            # Position from ELAPSED TIME, not accumulated per-tick -- see
            # the module docstring for why this matters.
            x = x0 + vx * elapsed
            y = y0 + vy * elapsed
            z = z0

            ok = send_pose(node, x, y, z, STREAM_TIMEOUT_MS)
            n_requests += 1
            final_y = y

            if ok:
                consecutive_failures = 0
            else:
                n_failures += 1
                consecutive_failures += 1
                if consecutive_failures > MAX_CONSECUTIVE_FAILURES:
                    print(
                        f"[m4-mover] FAILED: {consecutive_failures} "
                        "consecutive set_pose failures, aborting"
                    )
                    exit_code = 2
                    break

            writer.writerow([f"{elapsed:.4f}", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"])
            if n_requests % CSV_FLUSH_EVERY == 0:
                log_file.flush()

            # Wall-clock pacing: aim for the Nth tick to land at
            # stream_start + N*dt, so a slow tick doesn't push every
            # future tick later by the same amount (no drift accumulation).
            next_tick = stream_start + n_requests * dt
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        log_file.flush()
        log_file.close()

    # A SIGTERM/SIGINT is a normal, expected shutdown request from
    # m4_intercept.py (which owns the overall run lifecycle) -- always
    # exit 0 for that, regardless of any set_pose failures counted above.
    if stop["signaled"]:
        exit_code = 0

    print(
        f"[m4-mover] done: n_requests={n_requests} n_failures={n_failures} "
        f"final_y={final_y:.3f} log={log_path}"
    )
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
