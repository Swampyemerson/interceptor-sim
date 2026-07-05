#!/usr/bin/env python3
"""S2 external-cue mock: streams a DEGRADED, ground-sensor-style position
report of the AprilTag target over UDP, standing in for the parent
project's ground stereo rig (GOALS.md IS-NOT list: "no ground stereo rig
... mocked away here"). See docs/decisions.md ADR-0010 decision #4 (exactly
3 degradation knobs: Gaussian noise, fixed latency, coarse update rate) and
ADR-0010 decision #5 (hard handoff -- this process is the "external cue"
half of S2's two-stage CUE_WAIT -> DASH -> camera-only-ENGAGE architecture;
scripts/m4_intercept.py is the other half, and reads this process's UDP
datagrams during CUE_WAIT/DASH only).

WHY THIS PROCESS HOLDS SUBSCRIPTIONS AND THE MOVER DOES NOT (read this
before "simplifying" it to match m4_target_mover.py's subscription-free
shape): scripts/m4_target_mover.py MUST stay subscription-free because it
calls the `/world/apriltag/set_pose` gz SERVICE and needs to see the
service's RESPONSE -- and we found, the hard way (ADR-0009), that a
gz-transport13 process holding ANY topic subscription never receives
service responses again (the request still applies, you just never hear
back). This script makes ZERO gz service calls -- it only SUBSCRIBES to
two read-only topics (`/world/apriltag/pose/info` for the target's true
world position, `/clock` for sim time) and sends its own UDP datagrams
(nothing to do with gz-transport at all) -- so that quirk simply does not
apply here, and there is no reason to split this into a subscription-free
process the way the mover was.

WHAT: on a fixed **SIM-time** schedule (`--rate` Hz, default 10), samples
the AprilTag target's true world position from the pose topic, adds
per-axis Gaussian noise (`--sigma`, default 0.5 m), and schedules delivery
`--latency-s` (default 0.12 s) of SIM time after the sample was taken --
sim time, not wall time, for the same reason scripts/m4_target_mover.py
schedules target motion on sim time (ADR-0009 second addendum): this
sim's real-time factor sags under load, and a cue paced by wall-clock
timers would silently drift out of its intended 10 Hz / 100 ms-latency
spec relative to the vehicle's own (sim-time) physics and guidance loop.
Delivery is a UDP JSON datagram to 127.0.0.1:<--port> containing ONLY the
noisy position -- `{"seq", "t_sim", "x", "y", "z"}` -- the world-frame
position AT SAMPLE TIME, degraded; the true position is written to this
process's own CSV log for audit ONLY, never sent over the wire (mirrors
the camera-only / no-cheat honesty boundary elsewhere in this project:
the CONSUMER of this cue must never have a back door to ground truth).

Every sample (delivered or not) that had truth available is logged to
logs/s2_cue_<UTC timestamp>.csv: t_sim_sample, t_sim_emit, x_noisy,
y_noisy, z_noisy, x_true, y_true, z_true -- so a run can be audited after
the fact for exactly what was (and wasn't) available to the interceptor,
and when.

Run manually (with PX4 SITL + gz_x500_mono_cam + the apriltag world already
running, tag placed somewhere so the pose topic has something to report):

    .venv/bin/python scripts/s2_cue_mock.py --seed 0 --duration 30

Normally this is spawned automatically by scripts/m4_intercept.py's
CUE_WAIT phase (see that file's --handoff mode), using the same venv
interpreter (sys.executable), and left running (not killed) across the
HANDOFF transition -- ADR-0010 #5's "illegal-state-unrepresentable" latch
closes the RECEIVING side in m4_intercept.py, not this sender; this
process's own CSV log staying complete through and past handoff is the
audit evidence that the data was available-but-deliberately-unread.
"""

import argparse
import csv
import json
import os
import random
import signal
import socket
import sys
import time
from datetime import datetime, timezone

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.clock_pb2 import Clock

# Mirrors scripts/m2_detect.py's WORLD_NAME / TAG_MODEL_NAME -- not imported
# from there on purpose (task requirement: this script stays standalone,
# no imports from the detection/guidance modules).
WORLD_NAME = "apriltag"
TAG_MODEL_NAME = "apriltag_target"
POSE_TOPIC = f"/world/{WORLD_NAME}/pose/info"
CLOCK_TOPIC = "/clock"

DEFAULT_PORT = 47800
DEFAULT_RATE_HZ = 10.0
DEFAULT_SIGMA_M = 0.5
DEFAULT_LATENCY_S = 0.12
DEFAULT_DURATION_S = 60.0  # SIM seconds -- generous; the consumer decides how long it actually listens
SETUP_WAIT_TIMEOUT_S = 15.0  # wall seconds to wait for the first pose_v + clock sample
CSV_FLUSH_EVERY = 10
WALL_POLL_S = 0.01  # tight poll so sim-time schedule crossings are caught promptly

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")


class LatestTagPose:
    """Latest known TRUE world position of the apriltag_target model, fed by
    /world/apriltag/pose/info. Same lock-free "one attribute, replaced
    atomically under the GIL" pattern as m2_detect.PoseTracker -- the
    callback fires on a background gz-transport thread, this process's main
    loop just reads whatever is freshest. Model origin (top-level pose, no
    parent -- already in world frame, same as m2_detect.py's ground-truth
    chain) IS the tag center (the mover streams z=0.5)."""

    def __init__(self):
        self.xyz = None  # (x, y, z) world/ENU, or None until the first message

    def on_pose_v(self, msg: Pose_V) -> None:
        for p in msg.pose:
            if p.name == TAG_MODEL_NAME:
                self.xyz = (p.position.x, p.position.y, p.position.z)
                return


class LatestSimClock:
    """Latest sim time from /clock, same atomic-attribute pattern. Unlike
    scripts/m4_target_mover.py's clock helper, this can subscribe directly
    in-process -- see the module docstring for why that split doesn't apply
    here (no gz service calls in this script at all)."""

    def __init__(self):
        self.t = None

    def on_clock(self, msg: Clock) -> None:
        self.t = msg.sim.sec + msg.sim.nsec * 1e-9


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help="UDP port on 127.0.0.1 to send cue datagrams to",
    )
    parser.add_argument(
        "--rate", type=float, default=DEFAULT_RATE_HZ,
        help="cue sample rate in Hz, scheduled on SIM time",
    )
    parser.add_argument(
        "--sigma", type=float, default=DEFAULT_SIGMA_M,
        help="Gaussian position noise, meters, applied independently per axis",
    )
    parser.add_argument(
        "--latency-s", type=float, default=DEFAULT_LATENCY_S,
        help="fixed delivery latency after the sample was taken, in SIM seconds",
    )
    parser.add_argument(
        "--seed", type=int, default=0,
        help="RNG seed (reproducibility -- a fresh Random instance, not the global one)",
    )
    parser.add_argument(
        "--duration", type=float, default=DEFAULT_DURATION_S,
        help="how long to sample/stream, in SIM seconds",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = os.path.join(LOGS_DIR, f"s2_cue_{timestamp}.csv")

    # Signal handlers installed before anything blocking, same reasoning as
    # scripts/m4_target_mover.py: a SIGTERM/SIGINT arriving during setup
    # should still be honored by the streaming loop below, not kill us with
    # Python's default disposition.
    stop = {"flag": False, "signaled": False}

    def _handle_signal(signum, _frame):
        stop["flag"] = True
        stop["signaled"] = True

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    node = Node()
    tag_pose = LatestTagPose()
    if not node.subscribe(Pose_V, POSE_TOPIC, tag_pose.on_pose_v):
        print(f"[s2-cue] FAILED: could not subscribe to {POSE_TOPIC}")
        return 2
    sim_clock = LatestSimClock()
    if not node.subscribe(Clock, CLOCK_TOPIC, sim_clock.on_clock):
        print(f"[s2-cue] FAILED: could not subscribe to {CLOCK_TOPIC}")
        node.unsubscribe(POSE_TOPIC)
        return 2

    print(f"[s2-cue] Subscribed to {POSE_TOPIC} and {CLOCK_TOPIC}")
    print(f"[s2-cue] Logging to {log_path}")

    deadline = time.monotonic() + SETUP_WAIT_TIMEOUT_S
    while (tag_pose.xyz is None or sim_clock.t is None) and time.monotonic() < deadline:
        time.sleep(0.05)
    if tag_pose.xyz is None or sim_clock.t is None:
        print(
            f"[s2-cue] FAILED: no pose/clock data within {SETUP_WAIT_TIMEOUT_S}s "
            f"(tag_pose={tag_pose.xyz}, sim_t={sim_clock.t})"
        )
        node.unsubscribe(POSE_TOPIC)
        node.unsubscribe(CLOCK_TOPIC)
        return 2

    rng = random.Random(args.seed)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", args.port)

    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(
        ["t_sim_sample", "t_sim_emit", "x_noisy", "y_noisy", "z_noisy", "x_true", "y_true", "z_true"]
    )
    log_file.flush()

    # This exact line is a handshake, mirroring m4_target_mover.py's own
    # "streaming started" print -- m4_intercept.py's CUE_WAIT phase may
    # watch stdout for it, and it must only print after subscriptions +
    # first pose/clock samples are confirmed good above.
    print("[s2-cue] streaming started")

    period = 1.0 / args.rate
    sim_t0 = sim_clock.t
    next_sample_t = sim_t0
    pending = []  # (deliver_sim_t, seq, t_sample, xn, yn, zn, xt, yt, zt)
    seq = 0
    n_sampled = 0
    n_emitted = 0
    n_skipped_no_truth = 0

    try:
        while not stop["flag"]:
            t_now = sim_clock.t
            if t_now is None:
                time.sleep(WALL_POLL_S)
                continue
            sim_elapsed = t_now - sim_t0
            if sim_elapsed >= args.duration:
                break

            if t_now >= next_sample_t:
                if tag_pose.xyz is not None:
                    xt, yt, zt = tag_pose.xyz
                    xn = xt + rng.gauss(0.0, args.sigma)
                    yn = yt + rng.gauss(0.0, args.sigma)
                    zn = zt + rng.gauss(0.0, args.sigma)
                    deliver_t = t_now + args.latency_s
                    pending.append((deliver_t, seq, t_now, xn, yn, zn, xt, yt, zt))
                    seq += 1
                    n_sampled += 1
                else:
                    n_skipped_no_truth += 1
                next_sample_t += period

            while pending and sim_clock.t is not None and pending[0][0] <= sim_clock.t:
                deliver_t, s, t_sample, xn, yn, zn, xt, yt, zt = pending.pop(0)
                payload = {"seq": s, "t_sim": t_sample, "x": xn, "y": yn, "z": zn}
                sock.sendto(json.dumps(payload).encode("utf-8"), dest)
                t_emit = sim_clock.t
                writer.writerow(
                    [
                        f"{t_sample:.4f}", f"{t_emit:.4f}",
                        f"{xn:.4f}", f"{yn:.4f}", f"{zn:.4f}",
                        f"{xt:.4f}", f"{yt:.4f}", f"{zt:.4f}",
                    ]
                )
                n_emitted += 1
                if n_emitted % CSV_FLUSH_EVERY == 0:
                    log_file.flush()

            time.sleep(WALL_POLL_S)
    finally:
        log_file.flush()
        log_file.close()
        node.unsubscribe(POSE_TOPIC)
        node.unsubscribe(CLOCK_TOPIC)
        sock.close()

    # A SIGTERM/SIGINT is a normal, expected shutdown request from
    # m4_intercept.py (which owns the overall run lifecycle, same as the
    # mover) -- always exit 0 for that.
    exit_code = 0
    print(
        f"[s2-cue] done: n_sampled={n_sampled} n_emitted={n_emitted} "
        f"n_skipped_no_truth={n_skipped_no_truth} log={log_path}"
    )
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
