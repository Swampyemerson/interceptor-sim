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

WHAT: parses a start position and a base 2D velocity, then streams
`set_pose` requests at a fixed wall rate for a fixed duration. Positions
are computed from ELAPSED **SIM TIME** (position = f(sim_elapsed)), not
wall time and not accumulated tick-by-tick.

MANEUVERING PATHS (--path, M5/ADR-0010 #6): three target paths are
supported, all POSITION-only (the tag board's face is rigidly world -X in
the SDF; ADR-0010 #6 says maneuvers are velocity-schedule changes, never
yaw/aspect changes, so this mover sets position and never orientation):
  * line  (default) -- straight line, position = start + base_vel *
    sim_elapsed. BYTE-IDENTICAL to the pre-M5 behavior.
  * weave -- an S-curve: a sinusoidal lateral offset (perpendicular to the
    base velocity) added on top of the straight line. Knobs --weave-period-s
    and --weave-lat-speed (peak lateral speed). Against a 6-12 m/s crosser
    the defaults (4 s period, 3 m/s lateral) give ~1.9 m lateral amplitude.
    A startup check (main(), matching jink's explicit X_FLOOR_M clamp below)
    fails fast if x0 - amplitude <= X_FLOOR_M, i.e. an over-large weave
    config would swing world-X at/below the safety floor -- rather than
    silently violating the same ADR-0010 #6 invariant jink enforces.
  * jink -- one or more SHARP, discontinuous velocity-direction changes at
    sim times scheduled DETERMINISTICALLY from --path-seed (same seed =>
    identical schedule; different seeds => different, so paired-seed A/B
    stays valid). Heading changes are constrained to keep world-X
    non-decreasing (vx >= 0) so the -X-facing tag stays ahead of the
    interceptor (ADR-0010 #6).

ENV-VAR PLUMBING (so scripts/mc_batch.sh can select a path without editing
scripts/m4_intercept.py, which spawns this mover as a subprocess that
inherits the parent env -- the same INTERCEPTOR_WORLD_NAME pattern used for
the world name below): every path flag also reads an INTERCEPTOR_TARGET_PATH
/ _WEAVE_PERIOD_S / _WEAVE_LAT_SPEED / _JINK_COUNT / _JINK_MIN_DEG /
_PATH_SEED env var as its argparse DEFAULT, so setting the env var once
before launching m4_intercept.py retargets the mover too. An explicit CLI
flag always wins over the env var.

WHY SIM TIME (found 2026-07-05, the hard way): under full mission load
(camera rendering + detection + PX4 + guidance) this sim's real-time
factor sags to ~0.5. PX4 flies in SIM time, so a target scheduled on WALL
time moves ~2x its nominal speed relative to the vehicle's physics -- our
"2.0 m/s" target was effectively a 4.3 m/s target, faster than the
interceptor's own 4.0 m/s ceiling, and every engagement degenerated into
a matched-speed tail chase (PX4's ulog vs our wall-clock CSV disagreed on
the duration of the fast phase by exactly the RTF ratio; that was the
tell). Sim time comes from a tiny CHILD process subscribing to /clock and
piping it here -- this process itself must stay subscription-free (see
the service-response quirk above), and --duration is in SIM seconds.

Every commanded position is logged to logs/m4_mover_<UTC timestamp>.csv
(t, x, y, z) so the guidance side's ground truth can be cross-checked
against exactly what this process asked Gazebo to do.

Run manually (with PX4 SITL + gz_x500_mono_cam + the apriltag world already
running, tag already placed near --start so the pre-warm request has
something sane to confirm against):

    .venv/bin/python scripts/m4_target_mover.py --start "6.5,-4,0.5" --vel "0,2.0" --duration 12
    .venv/bin/python scripts/m4_target_mover.py --start "6.5,-14,0.5" --vel "0,6.0" --duration 20 --path weave
    .venv/bin/python scripts/m4_target_mover.py --start "6.5,-14,0.5" --vel "0,6.0" --duration 20 --path jink --path-seed 7

Offline (NO simulator) -- print the generated velocity schedule + integrated
positions over sim time and exit, for verifying a path before a batch:

    .venv/bin/python scripts/m4_target_mover.py --vel "0,6.0" --duration 20 --path jink --path-seed 7 --dry-run

Normally this is spawned automatically by scripts/m4_intercept.py's ENGAGE
phase (see that file), using the same venv interpreter (sys.executable).
"""

import argparse
import csv
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

# Mirrors scripts/m2_detect.py's WORLD_NAME / TAG_MODEL_NAME -- not imported
# from there on purpose: this script is meant to stay small and completely
# standalone (no shared subscription state, no shared anything), since its
# entire reason for existing is to be a minimal, subscription-free process.
# INTERCEPTOR_WORLD_NAME env var override (default "apriltag", byte-
# identical for every gated caller): see scripts/m2_detect.py's matching
# comment -- m4_intercept.py spawns this script as a subprocess, which
# inherits the parent's environment, so setting the var once before
# launching m4_intercept.py retargets the mover too (demo-video tooling).
WORLD_NAME = os.environ.get("INTERCEPTOR_WORLD_NAME", "apriltag")
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
DEFAULT_DURATION_S = 12.0  # SIM seconds (see module docstring)
SIM_CLOCK_TIMEOUT_S = 10.0  # wall seconds to wait for the first /clock sample

# --- maneuvering-path defaults (env-var-overridable so mc_batch.sh can plumb
# a path through the UNCHANGED m4_intercept.py via the inherited environment;
# see the module docstring's ENV-VAR PLUMBING note). An explicit CLI flag
# always overrides the env var (argparse uses these only as the default). ---
DEFAULT_PATH = os.environ.get("INTERCEPTOR_TARGET_PATH", "line")
DEFAULT_WEAVE_PERIOD_S = float(os.environ.get("INTERCEPTOR_WEAVE_PERIOD_S", "4.0"))
DEFAULT_WEAVE_LAT_SPEED = float(os.environ.get("INTERCEPTOR_WEAVE_LAT_SPEED", "3.0"))
DEFAULT_JINK_COUNT = int(os.environ.get("INTERCEPTOR_JINK_COUNT", "2"))
DEFAULT_JINK_MIN_DEG = float(os.environ.get("INTERCEPTOR_JINK_MIN_DEG", "40.0"))
DEFAULT_PATH_SEED = int(os.environ.get("INTERCEPTOR_PATH_SEED", "0"))
VALID_PATHS = ("line", "weave", "jink")

# ADR-0010 #6 safety floor: the -X-facing tag board must stay ahead of the
# interceptor (which spawns at world X=0), so no path is allowed to drive the
# target's world-X below this. line/weave never approach it at the batch
# geometries; jink additionally constrains its headings to vx >= 0.
X_FLOOR_M = 0.5


def _perp_unit(vx, vy):
    """Left-hand (+90 deg) horizontal unit vector perpendicular to the base
    velocity (vx, vy). For a +Y ('north') crosser this is -X ('west'); the
    weave alternates the offset's sign so which way perp points is
    immaterial. A ~zero base velocity has no travel direction, so default the
    lateral axis to world +X ('east')."""
    speed = math.hypot(vx, vy)
    if speed < 1e-9:
        return (1.0, 0.0)
    return (-vy / speed, vx / speed)


def _weave_amplitude(weave_period_s, weave_lat_speed):
    """Peak lateral displacement of --path weave's S-curve. offset(t) =
    amp*sin(w t) with w = 2*pi/period, and PEAK lateral speed (the knob) is
    amp*w by construction, so amp = weave_lat_speed*period/(2*pi). Shared by
    build_path() (to actually generate the path) and main()'s startup safety
    check (so the two can never drift apart)."""
    return weave_lat_speed * weave_period_s / (2.0 * math.pi)


def _build_jink_schedule(vx, vy, duration, seed, jink_count, jink_min_deg):
    """Deterministic piecewise-constant velocity schedule for --path jink: a
    list of (t_start_s, vx, vy) segments. Segment 0 is the UNCHANGED base
    velocity, so motion before the first jink is byte-identical to a straight
    line. Each scheduled jink (times drawn from random.Random(seed) in the
    middle 70% of the flight) rotates the heading to a fresh value that (a)
    differs from the previous heading by at least jink_min_deg -- a genuinely
    SHARP change -- and (b) keeps vx >= 0, so the target's world-X never
    decreases and the -X-facing tag stays ahead of the interceptor
    (ADR-0010 #6). Same seed => identical schedule; different seeds =>
    different (paired-seed A/B stays valid)."""
    speed = math.hypot(vx, vy)
    segments = [(0.0, vx, vy)]
    if speed < 1e-9 or jink_count <= 0:
        return segments
    rng = random.Random(seed)
    lo, hi = 0.15 * duration, 0.85 * duration
    times = sorted(rng.uniform(lo, hi) for _ in range(jink_count))
    # Bound the heading cone strictly inside +-90 deg so cos(heading) > 0
    # (vx > 0) for every jinked segment.
    hmax = math.pi / 2.0 - math.radians(1.0)
    min_step = math.radians(jink_min_deg)
    prev = math.atan2(vy, vx)
    for tj in times:
        cand = prev
        for _ in range(50):
            cand = rng.uniform(-hmax, hmax)
            if abs(cand - prev) >= min_step:
                break
        prev = cand
        segments.append((tj, speed * math.cos(cand), speed * math.sin(cand)))
    return segments


def build_path(path, x0, y0, z0, vx, vy, duration, weave_period_s,
               weave_lat_speed, jink_count, jink_min_deg, path_seed):
    """Return (pos_fn, vel_fn): pos_fn(sim_elapsed) -> (x, y, z) is what the
    mover streams via set_pose; vel_fn(sim_elapsed) -> (vx, vy) is the
    instantaneous commanded velocity (for logs / the --dry-run schedule). All
    three paths are closed-form / piecewise-closed-form in sim_elapsed (no
    tick-by-tick accumulation), matching the line path's original property."""
    if path == "line":
        def pos_fn(t):
            return (x0 + vx * t, y0 + vy * t, z0)

        def vel_fn(t):
            return (vx, vy)

        return pos_fn, vel_fn

    if path == "weave":
        px, py = _perp_unit(vx, vy)
        w = 2.0 * math.pi / weave_period_s
        # Displacement offset(t) = amp*sin(w t) so the PEAK lateral speed
        # (d/dt) is exactly weave_lat_speed = amp*w -> amp = weave_lat_speed/w.
        amp = _weave_amplitude(weave_period_s, weave_lat_speed)

        def pos_fn(t):
            off = amp * math.sin(w * t)
            return (x0 + vx * t + px * off, y0 + vy * t + py * off, z0)

        def vel_fn(t):
            lat = weave_lat_speed * math.cos(w * t)
            return (vx + px * lat, vy + py * lat)

        return pos_fn, vel_fn

    if path == "jink":
        segs = _build_jink_schedule(
            vx, vy, duration, path_seed, jink_count, jink_min_deg
        )

        def pos_fn(t):
            x, y = x0, y0
            for i, (ts, svx, svy) in enumerate(segs):
                te = segs[i + 1][0] if i + 1 < len(segs) else float("inf")
                dt = min(t, te) - ts
                if dt <= 0.0:
                    break
                x += svx * dt
                y += svy * dt
            return (max(x, X_FLOOR_M), y, z0)

        def vel_fn(t):
            cur = segs[0]
            for seg in segs:
                if seg[0] <= t:
                    cur = seg
                else:
                    break
            return (cur[1], cur[2])

        return pos_fn, vel_fn

    raise ValueError(f"unknown --path {path!r} (expected one of {VALID_PATHS})")

# Child process source: subscribes to /clock (gz.msgs Clock, sim field) and
# prints sim-time seconds, one per line. Runs in its OWN process because a
# subscription in THIS process would kill set_pose service responses.
CLOCK_HELPER_SRC = r"""
import sys, time
from gz.transport13 import Node
from gz.msgs10.clock_pb2 import Clock
last = [0.0]
def cb(m):
    t = m.sim.sec + m.sim.nsec * 1e-9
    if t - last[0] >= 0.02:  # throttle to ~50 Hz
        last[0] = t
        print(f"{t:.4f}", flush=True)
n = Node()
if not n.subscribe(Clock, "/clock", cb):
    sys.exit(2)
while True:
    time.sleep(1.0)
"""


class SimClock:
    """Latest sim time, fed by the clock-helper child's stdout on a reader
    thread (same atomic-attribute pattern as the flight scripts)."""

    def __init__(self, proc):
        self.t = None
        self._thread = threading.Thread(target=self._read, args=(proc,), daemon=True)
        self._thread.start()

    def _read(self, proc):
        for line in proc.stdout:
            try:
                self.t = float(line)
            except ValueError:
                pass

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
    parser.add_argument(
        "--path", default=DEFAULT_PATH, choices=VALID_PATHS,
        help="target path: line (default, straight), weave (S-curve), or "
             "jink (sharp seed-scheduled turns). Default is INTERCEPTOR_TARGET_"
             f"PATH env or 'line'. (env default: {DEFAULT_PATH!r})",
    )
    parser.add_argument(
        "--weave-period-s", type=float, default=DEFAULT_WEAVE_PERIOD_S,
        help=f"--path weave S-curve period in SIM seconds (default {DEFAULT_WEAVE_PERIOD_S})",
    )
    parser.add_argument(
        "--weave-lat-speed", type=float, default=DEFAULT_WEAVE_LAT_SPEED,
        help="--path weave PEAK lateral speed (m/s), perpendicular to the base "
             f"velocity (default {DEFAULT_WEAVE_LAT_SPEED})",
    )
    parser.add_argument(
        "--jink-count", type=int, default=DEFAULT_JINK_COUNT,
        help=f"--path jink number of sharp turns (default {DEFAULT_JINK_COUNT})",
    )
    parser.add_argument(
        "--jink-min-deg", type=float, default=DEFAULT_JINK_MIN_DEG,
        help="--path jink minimum heading change per jink, degrees "
             f"(default {DEFAULT_JINK_MIN_DEG})",
    )
    parser.add_argument(
        "--path-seed", type=int, default=DEFAULT_PATH_SEED,
        help="--path jink schedule seed -- same seed gives an identical "
             f"schedule (default {DEFAULT_PATH_SEED})",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="print the generated velocity schedule + integrated positions "
             "over sim time and exit 0, WITHOUT touching Gazebo (offline check)",
    )
    args = parser.parse_args()

    try:
        x0, y0, z0 = _parse_floats(args.start, 3, "start")
        vx, vy = _parse_floats(args.vel, 2, "vel")
    except ValueError as exc:
        print(f"[m4-mover] FAILED: {exc}")
        return 2

    # Reviewer correction: weave had no X_FLOOR_M guard (only jink applied
    # one), so an over-large --weave-period-s/--weave-lat-speed could swing
    # world-X at/below X_FLOOR_M -- driving the target behind the
    # interceptor with the rigidly -X-facing tag turned away, silently
    # breaking the ADR-0010 #6 invariant. Fail fast (both --dry-run and a
    # live run) rather than clamp: clamping position without also reshaping
    # vel_fn would desync the logged velocity from the actual (clamped)
    # motion, so an explicit, loud precondition is the honest fix. Uses the
    # SAME amplitude formula build_path() uses (_weave_amplitude), so this
    # can't drift out of sync with the actual path.
    if args.path == "weave":
        amp = _weave_amplitude(args.weave_period_s, args.weave_lat_speed)
        min_x = x0 - amp
        if min_x <= X_FLOOR_M:
            print(
                f"[m4-mover] FAILED: --path weave with weave_period_s="
                f"{args.weave_period_s} weave_lat_speed={args.weave_lat_speed} "
                f"gives amplitude {amp:.3f}m, swinging world-X as low as "
                f"{min_x:.3f}m from start_x={x0} -- at/below the X_FLOOR_M="
                f"{X_FLOOR_M} safety floor (ADR-0010 #6: the -X-facing tag "
                "must stay ahead of the interceptor). Raise --start's x, or "
                "lower --weave-lat-speed / raise --weave-period-s."
            )
            return 2

    # Build the sim-time position/velocity functions for the chosen path.
    # line is byte-identical to the pre-M5 straight line; weave/jink layer a
    # sim-time-scheduled maneuver on top of the SAME base velocity.
    pos_fn, vel_fn = build_path(
        args.path, x0, y0, z0, vx, vy, args.duration,
        args.weave_period_s, args.weave_lat_speed,
        args.jink_count, args.jink_min_deg, args.path_seed,
    )

    # Offline schedule dump (no Gazebo, no /clock, no set_pose): print the
    # commanded velocity + integrated position over sim time and exit. Used to
    # verify a path deterministically before committing a batch to it.
    if args.dry_run:
        print(
            f"[m4-mover] DRY RUN path={args.path} start=({x0},{y0},{z0}) "
            f"base_vel=({vx},{vy}) duration={args.duration}s "
            f"weave_period_s={args.weave_period_s} weave_lat_speed={args.weave_lat_speed} "
            f"jink_count={args.jink_count} jink_min_deg={args.jink_min_deg} "
            f"path_seed={args.path_seed}"
        )
        print("t,x,y,z,vx,vy")
        step = 0.25
        n_steps = int(round(args.duration / step))
        for k in range(n_steps + 1):
            t = k * step
            px_, py_, pz_ = pos_fn(t)
            pvx, pvy = vel_fn(t)
            print(
                f"{t:.2f},{px_:.4f},{py_:.4f},{pz_:.4f},{pvx:.4f},{pvy:.4f}"
            )
        return 0

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

    clock_proc = subprocess.Popen(
        [sys.executable, "-c", CLOCK_HELPER_SRC],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
    )
    sim_clock = SimClock(clock_proc)
    deadline = time.monotonic() + SIM_CLOCK_TIMEOUT_S
    while sim_clock.t is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if sim_clock.t is None:
        print(f"[m4-mover] FAILED: no /clock data within {SIM_CLOCK_TIMEOUT_S}s")
        clock_proc.kill()
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
    sim_t0 = sim_clock.t
    exit_code = 0

    try:
        while not stop["flag"]:
            sim_elapsed = (sim_clock.t or sim_t0) - sim_t0
            if sim_elapsed >= args.duration:
                break

            # Position from elapsed SIM time (see module docstring: the
            # target must move in the same clock the vehicle's physics
            # uses, or a sagging real-time factor silently scales its
            # speed). pos_fn is line/weave/jink per --path; line is exactly
            # start + base_vel * sim_elapsed as before.
            x, y, z = pos_fn(sim_elapsed)

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

            writer.writerow([f"{sim_elapsed:.4f}", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"])
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
        clock_proc.kill()

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
