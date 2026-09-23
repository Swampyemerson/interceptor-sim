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

The 3 core knobs (--sigma, --latency-s, --rate) are joined by OPT-IN
ADR-0015 realism flags (range-dependent sigma, per-run datum bias, emitted
velocity, latency jitter, bursty Markov dropout, link cutoff). Per ADR-0017
(P-8), the range-dependent-sigma model's DEFAULT constants are now the
physics-grounded EXPECTED tier (sigma_R = 0.000 + 4.45e-05*R^2), NOT the old
hand-set 0.4 + 0.008*R^2 (which was ~180x too steep as stereo noise); that
old pessimistic curve is reachable via --sigma-range-a/--sigma-range-b, and
the long-range 2-4 m cue error it used to lump into the R^2 term is now the
SEPARATE per-run --datum-bias-m offset. A WORST-tier 0.20 s latency stress
(ADR-0016) is opt-in via --latency-worst-tier. Also present is the P-6
--stereo-config flag, which derives the range sigma from stereo GEOMETRY
(sigma_R(R) = R^2 * sigma_d / (b * f_px)) rather than hand-set constants --
the physical model the P-2 stereo-rig worker is calibrating (its defaults
are placeholders; see the STEREO_* constants). Every one of these defaults
OFF so the pre-ADR-0015 datagram + RNG stream stay byte-identical and
scripts/check_s2.sh at defaults is unaffected.

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
import math
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
# INTERCEPTOR_WORLD_NAME env var override (default "apriltag", byte-
# identical for every gated caller): see scripts/m2_detect.py's matching
# comment -- m4_intercept.py spawns this script as a subprocess, which
# inherits the parent's environment, so setting the var once before
# launching m4_intercept.py retargets the cue mock too (demo-video tooling).
WORLD_NAME = os.environ.get("INTERCEPTOR_WORLD_NAME", "apriltag")
# INTERCEPTOR_TARGET_MODEL retargets the true-pose entity this cue mock reads to
# SYNTHESIZE the ground-rig cue (default "apriltag_target"; ADR-0033 item 2 sets
# "fpv_target_markerless" for the markerless A/B). The cue mock legitimately reads
# gt to mock the ground SENSOR's output -- that cue is structurally MID-COURSE and
# unreadable post-handoff (honesty boundary), unchanged by which body it tracks.
TAG_MODEL_NAME = os.environ.get("INTERCEPTOR_TARGET_MODEL", "apriltag_target")
# Interceptor model (mirrors m2_detect.DRONE_MODEL -- not imported, this script
# stays standalone). Read from the SAME pose/info subscription as the tag so
# the ADR-0015 range-dependent sigma / link-cutoff can use the true
# interceptor<->target distance R with zero new topics or gz service calls.
DRONE_MODEL = "x500_mono_cam_0"
POSE_TOPIC = f"/world/{WORLD_NAME}/pose/info"
CLOCK_TOPIC = "/clock"

DEFAULT_PORT = 47800
DEFAULT_RATE_HZ = 10.0
DEFAULT_SIGMA_M = 0.5
DEFAULT_LATENCY_S = 0.12    # ADR-0016 EXPECTED cue-delivery latency tier
WORST_LATENCY_S = 0.20     # ADR-0016 WORST stress tier (opt-in via --latency-worst-tier)
DEFAULT_DURATION_S = 60.0  # SIM seconds -- generous; the consumer decides how long it actually listens

# --- ADR-0015 perception-realism knobs (all OPT-IN; every default below keeps
# the datagram byte-identical and the RNG stream unchanged vs. the pre-ADR-0015
# 3-knob mock, so scripts/check_s2.sh at defaults is unaffected -- see the
# per-flag argparse defaults and the guarded draws in main()). Table refs are
# to docs/decisions.md ADR-0015's "DATA CONSTRAINTS BACK TO THE GUIDANCE MATH".
DEFAULT_VEL_SIGMA_M_S = 0.5      # per-axis velocity noise when --emit-velocity (table #2)
DEFAULT_LATENCY_JITTER_S = 0.0   # DEVIATION from ADR-0015's suggested 0.05: 0.0 keeps
                                 # the DEFAULT path byte-identical for the gate; dev/real
                                 # runs pass --latency-jitter-s 0.05 explicitly (table #3)
DEFAULT_DROPOUT_P = 0.12         # steady-state P(link out) for --dropout-markov (table #4)
DEFAULT_DROPOUT_BURST_S = 1.5    # mean outage (burst) length, sim seconds (table #4)
# Range-dependent sigma model (ADR-0015 table #1, CORRECTED per ADR-0017 P-8):
# sigma_R(R) = A + C*R^2, A in meters, C in m/m^2 (i.e. 1/m). ADR-0017 grounded
# this in stereo-rig physics (scripts/stereo_model.py) and found the ORIGINAL
# hand-set curve (A=0.4, C=0.008) was ~180x too STEEP as stereo NOISE -- it
# extrapolates to ~80 m sigma at R=100 m where the physics gives ~0.45 m. The
# 2-4 m handoff-range error that old curve modeled is REAL, but it is the GPS
# DATUM/CLOCK offset -- a per-RUN CONSTANT that belongs in --datum-bias-m, NOT
# in the R^2 noise term. So the R^2 NOISE (this A+C*R^2) and the per-run DATUM
# BIAS (--datum-bias-m) are now SEPARATE knobs. Corrected three-tier constants
# (ADR-0017; consistency-checked to sigma_disparity ~0.48 px at EXPECTED):
#     BEST     (A,C) = (0.000, 1.08e-05), datum-bias 0.3 m
#   * EXPECTED (A,C) = (0.000, 4.45e-05), datum-bias 0.5 m   <- the DEFAULTS below
#     WORST    (A,C) = (0.214, 1.94e-04), datum-bias 2.5 m
# The DEFAULTS below are the EXPECTED tier. The OLD pre-ADR-0017 pessimistic
# curve is still reachable for regression/comparison via explicit overrides:
#     --sigma-range --sigma-range-a 0.4 --sigma-range-b 0.008
# (see --sigma-range-a / --sigma-range-b). datum-bias tiers pass via
# --datum-bias-m; the WORST latency tier via --latency-worst-tier (ADR-0016).
SIGMA_RANGE_A_M = 0.000            # ADR-0017 EXPECTED tier 'a' (was 0.4 pessimistic)
SIGMA_RANGE_B_M_PER_M2 = 4.45e-05  # ADR-0017 EXPECTED tier 'c' (was 0.008 pessimistic)
# OLD pre-ADR-0017 pessimistic curve, kept as named constants so the override
# example above and any regression harness can cite the exact retired values.
SIGMA_RANGE_A_OLD_M = 0.4
SIGMA_RANGE_B_OLD_M_PER_M2 = 0.008
# --- ADR-0015 table #1 / P-2 stereo geometry (--stereo-config): derive the
# range sigma from FIRST PRINCIPLES instead of the hand-set A + B*R^2 above.
# Stereo range R = b*f_px/d (d = disparity, px); propagating a disparity
# noise sigma_d through it gives
#     sigma_R(R) = R^2 * sigma_d / (b * f_px)
# (baseline b [m], focal f_px [px], sigma_d [px]). This is the physical model
# the P-2 stereo-rig worker (scripts/stereo_model.py, in flight) is
# calibrating; the defaults here are PLACEHOLDERS -- a parallel agent is
# producing the calibrated (baseline, focal, sigma_disparity) for the real
# 50-150 m engagement. NB: at this sim's COMPRESSED ~15 m standoff these
# placeholders yield an optimistic ~0.1 m sigma (225*0.5/1080); the real rig's
# calibrated constants + real range are what make it representative -- swap
# them in from P-2, do not tune these to hit a number here.
STEREO_DEFAULT_BASELINE_M = 2.0
STEREO_DEFAULT_FOCAL_PX = 540.0
STEREO_DEFAULT_SIGMA_DISPARITY_PX = 0.5
# Velocity is a light EMA of the finite-difference of TRUE positions (models a
# ground tracker's own smoothing) before per-axis noise is added.
VEL_EMA_ALPHA = 0.5
# Derived-seed offsets so the per-RUN datum-bias direction and the per-TICK
# dropout chain draw from their OWN Random instances -- enabling either knob
# leaves the per-sample position-noise stream (the main rng) untouched.
BIAS_SEED_OFFSET = 1000003
DROPOUT_SEED_OFFSET = 2000003
SETUP_WAIT_TIMEOUT_S = 15.0  # wall seconds to wait for the first pose_v + clock sample
CSV_FLUSH_EVERY = 10
WALL_POLL_S = 0.01  # tight poll so sim-time schedule crossings are caught promptly

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")


class LatestTagPose:
    """Latest known TRUE world positions of the apriltag_target model AND the
    interceptor model (x500_mono_cam_0), both fed by the ONE
    /world/apriltag/pose/info subscription. Same lock-free "one attribute,
    replaced atomically under the GIL" pattern as m2_detect.PoseTracker -- the
    callback fires on a background gz-transport thread, this process's main
    loop just reads whatever is freshest. Model origins (top-level poses, no
    parent -- already in world frame, same as m2_detect.py's ground-truth
    chain): the tag origin IS the tag center (the mover streams z=0.5); the
    interceptor origin is used ONLY to compute the true interceptor<->target
    range R for the ADR-0015 range-dependent sigma / link-cutoff models (never
    sent over the wire)."""

    def __init__(self):
        self.xyz = None  # tag (x, y, z) world/ENU, or None until first message
        self.interceptor_xyz = None  # interceptor (x, y, z) world/ENU, or None

    def on_pose_v(self, msg: Pose_V) -> None:
        for p in msg.pose:
            if p.name == TAG_MODEL_NAME:
                self.xyz = (p.position.x, p.position.y, p.position.z)
            elif p.name == DRONE_MODEL:
                self.interceptor_xyz = (p.position.x, p.position.y, p.position.z)

    def range_m(self):
        """True interceptor<->target distance R (m), or None if either pose is
        not yet available. Uses full 3D distance (the S2 geometry keeps both at
        z~0.5, so this is essentially the horizontal range)."""
        if self.xyz is None or self.interceptor_xyz is None:
            return None
        dx = self.xyz[0] - self.interceptor_xyz[0]
        dy = self.xyz[1] - self.interceptor_xyz[1]
        dz = self.xyz[2] - self.interceptor_xyz[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)


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
    # --- ADR-0015 perception-realism knobs (all OPT-IN; see the module
    # docstring / the constants block. Every default here reproduces the
    # pre-ADR-0015 3-knob behavior byte-for-byte.) ---
    parser.add_argument(
        "--sigma-range", action="store_true",
        help="ADR-0015 table #1 (CORRECTED per ADR-0017 P-8): replace the flat "
             "--sigma with a range-dependent sigma_R = a + c*R^2 (R = true "
             "interceptor<->target distance from the pose topic). Defaults are "
             "the ADR-0017 EXPECTED tier a=%.3g, c=%.3g; override a/c with "
             "--sigma-range-a/--sigma-range-b (e.g. the OLD pre-ADR-0017 "
             "pessimistic curve: --sigma-range-a %.1f --sigma-range-b %.3f). "
             "Off by default (flat --sigma preserved)." % (
                 SIGMA_RANGE_A_M, SIGMA_RANGE_B_M_PER_M2,
                 SIGMA_RANGE_A_OLD_M, SIGMA_RANGE_B_OLD_M_PER_M2),
    )
    parser.add_argument(
        "--sigma-range-a", type=float, default=None,
        help="ADR-0017: override the range-sigma constant term a (m) used by "
             "--sigma-range. Default None => EXPECTED tier %.3g. Reach the OLD "
             "pessimistic curve with --sigma-range-a %.1f (+ --sigma-range-b "
             "%.3f); BEST/WORST 'a' are 0.000/0.214." % (
                 SIGMA_RANGE_A_M, SIGMA_RANGE_A_OLD_M, SIGMA_RANGE_B_OLD_M_PER_M2),
    )
    parser.add_argument(
        "--sigma-range-b", type=float, default=None,
        help="ADR-0017: override the range-sigma R^2 coefficient c (m/m^2) "
             "used by --sigma-range. Default None => EXPECTED tier %.3g. OLD "
             "pessimistic %.3f; BEST/WORST 'c' are 1.08e-05/1.94e-04." % (
                 SIGMA_RANGE_B_M_PER_M2, SIGMA_RANGE_B_OLD_M_PER_M2),
    )
    parser.add_argument(
        "--datum-bias-m", type=float, default=0.0,
        help="ADR-0015 tables #1/#5 (semantics clarified by ADR-0017): per-RUN "
             "constant common-mode offset vector added to every emitted "
             "position -- a seeded random 3D direction (from --seed) at this "
             "fixed magnitude in m. This is the GPS DATUM/CLOCK offset, kept "
             "SEPARATE from the R^2 stereo noise (--sigma-range). ADR-0017 "
             "tiers: 0.3 BEST / 0.5 EXPECTED (shared-RTK design target) / 2.5 "
             "WORST (standard GPS). Default 0.0 (no bias, no draw).",
    )
    parser.add_argument(
        "--emit-velocity", action="store_true",
        help="ADR-0015 table #2: also send a filtered target velocity "
             "(vx, vy, vz world) in each datagram (finite-difference of TRUE "
             "positions, EMA-smoothed, plus per-axis --vel-sigma noise). "
             "Backward compatible: extra keys only. Off by default.",
    )
    parser.add_argument(
        "--vel-sigma", type=float, default=DEFAULT_VEL_SIGMA_M_S,
        help="per-axis velocity noise std (m/s) used with --emit-velocity "
             "(default %.2f)" % DEFAULT_VEL_SIGMA_M_S,
    )
    parser.add_argument(
        "--latency-jitter-s", type=float, default=DEFAULT_LATENCY_JITTER_S,
        help="ADR-0015 table #3: per-datagram delivery latency = --latency-s "
             "+/- uniform(this) SIM seconds (seeded, clamped >= 0). Default "
             "0.0 keeps the gate byte-identical; pass 0.05 to enable jitter.",
    )
    parser.add_argument(
        "--latency-worst-tier", action="store_true",
        help="ADR-0016 three-tier latency budget: force the WORST stress tier, "
             "overriding --latency-s to %.2f s (sim) -- the pessimistic SiK-link "
             "cue-delivery latency the sim never runs at default. Opt-in stress "
             "preset, NOT the default (EXPECTED = --latency-s %.2f s). Combine "
             "with --latency-jitter-s for jitter on top." % (
                 WORST_LATENCY_S, DEFAULT_LATENCY_S),
    )
    parser.add_argument(
        "--dropout-markov", action="store_true",
        help="ADR-0015 table #4: 2-state (in/out) Markov link dropout; while "
             "'out', emit nothing. Off by default.",
    )
    parser.add_argument(
        "--dropout-p", type=float, default=DEFAULT_DROPOUT_P,
        help="steady-state P(link out) for --dropout-markov (default %.2f)" % DEFAULT_DROPOUT_P,
    )
    parser.add_argument(
        "--dropout-burst-s", type=float, default=DEFAULT_DROPOUT_BURST_S,
        help="mean outage (burst) length in SIM seconds for --dropout-markov "
             "(default %.2f)" % DEFAULT_DROPOUT_BURST_S,
    )
    parser.add_argument(
        "--stereo-config", type=str, default=None,
        metavar="B,F_PX,SIGMA_D",
        help="ADR-0015 table #1 (P-2 stereo geometry): derive the range sigma "
             "from stereo geometry, sigma_R(R) = R^2*sigma_d/(b*f_px), instead "
             "of the hand-set A+B*R^2 (--sigma-range) or flat --sigma. Give "
             "'baseline_m,focal_px,sigma_disparity_px' (default placeholders "
             "%.1f,%.0f,%.1f -- a parallel agent is calibrating the real "
             "values; see the STEREO_* constants comment). Overrides "
             "--sigma-range/--sigma when set." % (
                 STEREO_DEFAULT_BASELINE_M, STEREO_DEFAULT_FOCAL_PX,
                 STEREO_DEFAULT_SIGMA_DISPARITY_PX),
    )
    parser.add_argument(
        "--link-cutoff-range-m", type=float, default=None,
        help="ADR-0015 table #6: once the true interceptor<->target range "
             "first falls below this (m), STOP emitting permanently (models a "
             "jammer killing the link, possibly before camera lock). Default "
             "off.",
    )
    parser.add_argument(
        "--link-cutoff-s", type=float, default=None,
        help="ADR-0015 table #6: once SIM time exceeds this many seconds "
             "after the first emitted datagram, STOP emitting permanently. "
             "Default off.",
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

    rng = random.Random(args.seed)  # per-sample MEASUREMENT noise (position, velocity, jitter)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", args.port)

    # --- ADR-0015 table #1 (P-2 stereo geometry, --stereo-config): parse
    # 'baseline_m,focal_px,sigma_disparity_px' once. None (default) => this
    # whole feature is inert and the sigma model falls back to --sigma-range /
    # flat --sigma exactly as before (byte-identical). Precompute the constant
    # k = sigma_d/(b*f_px) so the per-sample cost is just k*R^2. ---
    stereo_k = None
    if args.stereo_config is not None:
        try:
            parts = [float(v) for v in args.stereo_config.split(",")]
            if len(parts) != 3:
                raise ValueError("expected exactly 3 comma-separated values")
            b_m, f_px, sigma_d = parts
            if b_m <= 0.0 or f_px <= 0.0:
                raise ValueError("baseline and focal must be positive")
        except ValueError as exc:
            print(f"[s2-cue] FAILED: bad --stereo-config {args.stereo_config!r}: {exc} "
                  f"(want 'baseline_m,focal_px,sigma_disparity_px', e.g. "
                  f"{STEREO_DEFAULT_BASELINE_M},{STEREO_DEFAULT_FOCAL_PX:.0f},"
                  f"{STEREO_DEFAULT_SIGMA_DISPARITY_PX})")
            node.unsubscribe(POSE_TOPIC)
            node.unsubscribe(CLOCK_TOPIC)
            return 2
        stereo_k = sigma_d / (b_m * f_px)   # sigma_R(R) = stereo_k * R^2
        print(
            f"[s2-cue] stereo-config ON: baseline={b_m:.2f} m focal={f_px:.0f} px "
            f"sigma_disparity={sigma_d:.2f} px -> sigma_R(R) = {stereo_k:.5g}*R^2 "
            f"(overrides --sigma-range/--sigma; PLACEHOLDER constants pending P-2 "
            f"calibration -- see STEREO_* comment)"
        )

    # --- ADR-0015 per-RUN datum bias (guarded: NO draw unless --datum-bias-m
    # > 0, so the default path leaves both this AND the main rng untouched).
    # Drawn from its OWN Random instance so enabling bias never perturbs the
    # per-sample position-noise stream. A random 3D unit direction scaled by
    # the requested magnitude -> a fixed common-mode offset on every emit. ---
    bias = (0.0, 0.0, 0.0)
    if args.datum_bias_m > 0.0:
        bias_rng = random.Random(args.seed + BIAS_SEED_OFFSET)
        bdx = bias_rng.gauss(0.0, 1.0)
        bdy = bias_rng.gauss(0.0, 1.0)
        bdz = bias_rng.gauss(0.0, 1.0)
        bnorm = math.sqrt(bdx * bdx + bdy * bdy + bdz * bdz) or 1.0
        bias = (
            args.datum_bias_m * bdx / bnorm,
            args.datum_bias_m * bdy / bnorm,
            args.datum_bias_m * bdz / bnorm,
        )
        print(
            f"[s2-cue] datum bias (per-run constant): "
            f"({bias[0]:.3f}, {bias[1]:.3f}, {bias[2]:.3f}) m, |b|={args.datum_bias_m:.2f}"
        )

    # --- ADR-0015 Markov dropout transition probabilities (per 10 Hz sample
    # tick). Steady-state P(out)=dropout_p, mean OUT (burst) length =
    # dropout_burst_s: P(out->in)=period/burst, P(in->out)=P(out->in)*p/(1-p).
    # Chain (and its own Random) only exists under --dropout-markov. ---
    period = 1.0 / args.rate
    dropout_rng = None
    p_in_to_out = p_out_to_in = 0.0
    dropout_state = ""  # "" when disabled; "in"/"out" when --dropout-markov
    if args.dropout_markov:
        dropout_rng = random.Random(args.seed + DROPOUT_SEED_OFFSET)
        p = min(max(args.dropout_p, 0.0), 0.999)
        p_out_to_in = min(1.0, period / max(args.dropout_burst_s, 1e-6))
        p_in_to_out = min(1.0, p_out_to_in * (p / (1.0 - p))) if p > 0.0 else 0.0
        dropout_state = "in"
        print(
            f"[s2-cue] Markov dropout ON: p_out={p:.2f} burst={args.dropout_burst_s:.2f}s "
            f"-> P(in->out)={p_in_to_out:.4f} P(out->in)={p_out_to_in:.4f} per tick"
        )

    # ADR-0017 P-8: resolve the range-noise coefficients. Defaults are the
    # corrected EXPECTED tier (module constants); --sigma-range-a/-b override
    # per-run (e.g. to reach the OLD pessimistic curve). Only consulted when
    # --sigma-range is active (stereo-config still takes precedence over both).
    sr_a = SIGMA_RANGE_A_M if args.sigma_range_a is None else args.sigma_range_a
    sr_b = SIGMA_RANGE_B_M_PER_M2 if args.sigma_range_b is None else args.sigma_range_b
    # ADR-0016 latency tiers: EXPECTED --latency-s (default 0.12); the opt-in
    # --latency-worst-tier forces the 0.20 s WORST stress tier (overrides
    # --latency-s). All jitter/delivery math below uses latency_base.
    latency_base = WORST_LATENCY_S if args.latency_worst_tier else args.latency_s
    if args.latency_worst_tier:
        print(
            f"[s2-cue] WORST latency tier ON (ADR-0016): --latency-s forced to "
            f"{WORST_LATENCY_S:.2f}s sim (EXPECTED default was {args.latency_s:.2f}s)"
        )
    if args.sigma_range:
        old = (sr_a == SIGMA_RANGE_A_OLD_M and sr_b == SIGMA_RANGE_B_OLD_M_PER_M2)
        tier = " [OLD pre-ADR-0017 pessimistic curve]" if old else " [ADR-0017 corrected]"
        print(
            f"[s2-cue] range-dependent sigma ON: sigma_R = {sr_a:.3g} + "
            f"{sr_b:.3g}*R^2{tier} (flat --sigma {args.sigma} ignored)"
        )
    if args.emit_velocity:
        print(f"[s2-cue] velocity emission ON: vel_sigma={args.vel_sigma:.2f} m/s per axis")
    if args.latency_jitter_s > 0.0:
        print(f"[s2-cue] latency jitter ON: +/- {args.latency_jitter_s:.3f}s (sim)")
    if args.link_cutoff_range_m is not None:
        print(f"[s2-cue] link cutoff ON: range < {args.link_cutoff_range_m:.2f} m -> permanent link death")
    if args.link_cutoff_s is not None:
        print(f"[s2-cue] link cutoff ON: t_sim > {args.link_cutoff_s:.2f}s after first emit -> permanent link death")

    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    # First 8 columns are IDENTICAL to the pre-ADR-0015 schema (anything that
    # read them positionally still works); ADR-0015 quantities are appended.
    writer.writerow(
        [
            "t_sim_sample", "t_sim_emit", "x_noisy", "y_noisy", "z_noisy",
            "x_true", "y_true", "z_true",
            "R", "sigma_used", "bias_x", "bias_y", "bias_z",
            "vx", "vy", "vz", "latency_used", "dropout_state", "event", "cutoff",
        ]
    )
    log_file.flush()

    def _f(v, spec="{:.4f}"):
        return "" if v is None else spec.format(v)

    def write_cue_row(event, t_sample, t_emit, xn, yn, zn, xt, yt, zt,
                      R, sigma_used, vel, latency_used, drop_state, cutoff):
        writer.writerow([
            _f(t_sample), _f(t_emit), _f(xn), _f(yn), _f(zn),
            _f(xt), _f(yt), _f(zt),
            _f(R), _f(sigma_used), _f(bias[0]), _f(bias[1]), _f(bias[2]),
            _f(vel[0]) if vel is not None else "",
            _f(vel[1]) if vel is not None else "",
            _f(vel[2]) if vel is not None else "",
            _f(latency_used), drop_state, event, "1" if cutoff else "",
        ])

    # This exact line is a handshake, mirroring m4_target_mover.py's own
    # "streaming started" print -- m4_intercept.py's CUE_WAIT phase may
    # watch stdout for it, and it must only print after subscriptions +
    # first pose/clock samples are confirmed good above.
    print("[s2-cue] streaming started")

    sim_t0 = sim_clock.t
    next_sample_t = sim_t0
    # pending item: dict with everything needed to send + log at delivery.
    pending = []
    seq = 0
    n_sampled = 0
    n_emitted = 0
    n_skipped_no_truth = 0
    n_dropped = 0
    link_dead = False
    first_emit_sim_t = None
    last_true_xyz = None      # for velocity finite-difference
    last_sample_sim_t = None
    vel_ema = None

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
                R = tag_pose.range_m()

                if link_dead:
                    # Link permanently killed (jammer): stop emitting, keep
                    # looping/logging nothing until duration ends (audit).
                    next_sample_t += period
                    time.sleep(WALL_POLL_S)
                    continue

                # ADR-0015 table #6: link cutoff (range- or time-triggered).
                cutoff_now = False
                if args.link_cutoff_range_m is not None and R is not None and R < args.link_cutoff_range_m:
                    cutoff_now = True
                if (
                    args.link_cutoff_s is not None and first_emit_sim_t is not None
                    and (t_now - first_emit_sim_t) > args.link_cutoff_s
                ):
                    cutoff_now = True
                if cutoff_now:
                    link_dead = True
                    xt = yt = zt = None
                    if tag_pose.xyz is not None:
                        xt, yt, zt = tag_pose.xyz
                    write_cue_row(
                        "drop_cutoff", t_now, None, None, None, None,
                        xt, yt, zt, R, None, None, None, dropout_state, True,
                    )
                    log_file.flush()
                    pending.clear()  # jammer kills in-flight datagrams too
                    print(
                        f"[s2-cue] LINK CUTOFF at t_sim={t_now:.3f} "
                        f"(R={'n/a' if R is None else f'{R:.2f}'} m) -- link dead, "
                        "emitting nothing further (ADR-0015 table #6)"
                    )
                    next_sample_t += period
                    time.sleep(WALL_POLL_S)
                    continue

                # ADR-0015 table #4: Markov dropout step (advance chain, then
                # honor an "out" state by emitting nothing this tick).
                if dropout_rng is not None:
                    u = dropout_rng.random()
                    if dropout_state == "in":
                        if u < p_in_to_out:
                            dropout_state = "out"
                    else:
                        if u < p_out_to_in:
                            dropout_state = "in"
                    if dropout_state == "out":
                        xt = yt = zt = None
                        if tag_pose.xyz is not None:
                            xt, yt, zt = tag_pose.xyz
                        write_cue_row(
                            "drop_markov", t_now, None, None, None, None,
                            xt, yt, zt, R, None, None, None, dropout_state, False,
                        )
                        n_dropped += 1
                        next_sample_t += period
                        time.sleep(WALL_POLL_S)
                        continue

                if tag_pose.xyz is not None:
                    xt, yt, zt = tag_pose.xyz
                    # sigma: stereo-geometry-derived (--stereo-config, highest
                    # precedence), else range-dependent (--sigma-range), else
                    # flat --sigma (default).
                    if stereo_k is not None and R is not None:
                        sigma_used = stereo_k * R * R
                    elif args.sigma_range and R is not None:
                        sigma_used = sr_a + sr_b * R * R
                    else:
                        sigma_used = args.sigma
                    # Position noise (SAME 3 rng.gauss draws, SAME order as the
                    # pre-ADR-0015 mock); bias adds a constant 0.0 by default.
                    xn = xt + rng.gauss(0.0, sigma_used) + bias[0]
                    yn = yt + rng.gauss(0.0, sigma_used) + bias[1]
                    zn = zt + rng.gauss(0.0, sigma_used) + bias[2]
                    # Velocity (only under --emit-velocity -> no extra draws by
                    # default): EMA-filtered finite-difference of TRUE positions
                    # over the sim-time sample interval, plus per-axis noise.
                    vel = None
                    if args.emit_velocity:
                        if last_true_xyz is not None and last_sample_sim_t is not None \
                                and (t_now - last_sample_sim_t) > 1e-6:
                            dts = t_now - last_sample_sim_t
                            raw = (
                                (xt - last_true_xyz[0]) / dts,
                                (yt - last_true_xyz[1]) / dts,
                                (zt - last_true_xyz[2]) / dts,
                            )
                        else:
                            raw = (0.0, 0.0, 0.0)
                        if vel_ema is None:
                            vel_ema = raw
                        else:
                            a = VEL_EMA_ALPHA
                            vel_ema = (
                                a * raw[0] + (1.0 - a) * vel_ema[0],
                                a * raw[1] + (1.0 - a) * vel_ema[1],
                                a * raw[2] + (1.0 - a) * vel_ema[2],
                            )
                        vel = (
                            vel_ema[0] + rng.gauss(0.0, args.vel_sigma),
                            vel_ema[1] + rng.gauss(0.0, args.vel_sigma),
                            vel_ema[2] + rng.gauss(0.0, args.vel_sigma),
                        )
                    # Latency: fixed base +/- uniform jitter (only draws when
                    # jitter > 0 -> default path byte-identical), clamped >= 0.
                    latency_used = latency_base
                    if args.latency_jitter_s > 0.0:
                        latency_used = max(
                            0.0, latency_base + rng.uniform(-args.latency_jitter_s, args.latency_jitter_s)
                        )
                    deliver_t = t_now + latency_used
                    pending.append({
                        "deliver_t": deliver_t, "seq": seq, "t_sample": t_now,
                        "xn": xn, "yn": yn, "zn": zn, "xt": xt, "yt": yt, "zt": zt,
                        "R": R, "sigma_used": sigma_used, "vel": vel,
                        "latency_used": latency_used, "drop_state": dropout_state,
                    })
                    seq += 1
                    n_sampled += 1
                    last_true_xyz = (xt, yt, zt)
                    last_sample_sim_t = t_now
                else:
                    n_skipped_no_truth += 1
                next_sample_t += period

            while pending and sim_clock.t is not None and pending[0]["deliver_t"] <= sim_clock.t:
                item = pending.pop(0)
                payload = {
                    "seq": item["seq"], "t_sim": item["t_sample"],
                    "x": item["xn"], "y": item["yn"], "z": item["zn"],
                }
                # Backward compatible: velocity keys appended only when present
                # (old consumers / default path see the exact same 5-key JSON).
                if item["vel"] is not None:
                    payload["vx"] = item["vel"][0]
                    payload["vy"] = item["vel"][1]
                    payload["vz"] = item["vel"][2]
                sock.sendto(json.dumps(payload).encode("utf-8"), dest)
                t_emit = sim_clock.t
                if first_emit_sim_t is None:
                    first_emit_sim_t = t_emit
                write_cue_row(
                    "emit", item["t_sample"], t_emit,
                    item["xn"], item["yn"], item["zn"],
                    item["xt"], item["yt"], item["zt"],
                    item["R"], item["sigma_used"], item["vel"],
                    item["latency_used"], item["drop_state"], False,
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
        f"n_skipped_no_truth={n_skipped_no_truth} n_dropped={n_dropped} "
        f"link_dead={int(link_dead)} log={log_path}"
    )
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
