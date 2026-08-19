#!/usr/bin/env python3
"""T16 (ADR-0046 F1 decision item 1): trajectory-matched OFFLINE stereo-rig
capture harness. See docs/decisions.md ADR-0046 and
docs/phase2_sim_to_real_plan.md Sec 5.5-5.6.

VENV: this needs cv2 (OpenCV) -- run it with the markerless/seeker venv:
    .venv-seeker/bin/python scripts/rig_snapshot_capture.py ...
(matches scripts/seeker/render_sim_dataset.py, which this harness extends;
the main .venv stays untouched/gated per docs/next.md's venv policy.)

WHAT: with PX4 SITL + gz_x500_mono_cam already running on the
`stereo_intercept` world (worlds/stereo_intercept.sdf, which places
models/ground_stereo_rig/ at the `broadside_160m` pose), this script
reproduces the deterministic target-mover dash path from
scripts/m4_target_mover.py's `line` path (READ that file first -- this is
NOT a live mover: it teleports directly via `gz service` subprocess calls,
the exact pattern scripts/seeker/render_sim_dataset.py uses, because this
process holds camera SUBSCRIPTIONS and therefore -- the documented
gz-transport13 quirk -- never receives `node.request(...)` service
RESPONSES, only CLI subprocess calls avoid that). For each sampled pose it
teleports `fpv_target_markerless`, waits for the render to settle (the
render_sim_dataset.py wait_new_frames pattern, applied to BOTH rig
cameras), and grabs one frame from EACH of the rig's two camera topics.

WHY OFFLINE, TARGET-FROZEN CAPTURE (disclosed, for T23's sim-vs-real audit):
frames per pose are captured with the target FROZEN in place while both
cameras settle and are read back -- so left/right are PERFECTLY
synchronized by construction. A real rig's hardware GPIO trigger (ADR-0017)
still has a nonzero left/right sync error (the "sync" term in
docs/stereo_design.md's error budget, ~0.1% of variance at the design
point); this offline harness has EXACTLY ZERO sync error, because there is
no motion between the two shutters. This is a genuine sim-vs-real gap,
carried forward as a T23 audit item -- do not read this harness's frames as
proof the rig has zero sync error in hardware.

THE DASH PATH (matches scripts/m4_target_mover.py's `line` path AND
scripts/rig_geometry_analysis.py's Analysis 2, byte-for-byte in geometry):
target holds world-X fixed at --x0 (default 6.5 m, confirmed against a real
flown log -- see docs/rig_geometry_analysis.md Sec 2) and sweeps world-Y at
constant --dash-speed (default 9.0 m/s -- a harness-local default, NOT
m4_intercept.py's S2["DASH_SPEED"]=10.0 or mc_batch.sh's batch-arm
--dash-speed 16; this is an offline capture-rate choice, not a flown
guidance parameter) from a start of +/- --y0-mag (default 29.3 m, the
ratified docs/next.md dash profile) toward Y=0, STOPPING once |y| <=
PATH_BOUNDARY_ABS_Y_M (5.0 m -- "stop just short of CPA (closest point of
approach)", the exact boundary scripts/rig_geometry_analysis.py's Analysis
2 uses, so this harness's captured span matches the geometry the wedge
analysis already validated). --y0-sign selects the start sign: '+' is the
"r2l" direction (start Y=+y0_mag, moving toward -Y, matching
docs/rig_geometry_analysis.md's r2l naming), '-' is "l2r" (the Y-mirror),
'both' runs r2l then l2r as two separate capture runs (separate output
subdirs/index rows, same --out tree).

SAMPLING CADENCE: --rate (default 10 Hz, matching the rig sensor's own
update_rate in models/ground_stereo_rig/model.sdf) sets a NOMINAL sim-time
spacing dt = 1/rate between consecutive samples; since the target moves at
a constant --dash-speed, the resulting along-track POSE spacing is exactly
dash_speed / rate meters (e.g. 9.0/10 = 0.9 m at the defaults) -- this is
what "pose spacing = speed/rate" means in the task spec. t_sim_nominal
(logged per row) is this synthetic per-sample sim-time value (k * dt from
the start of that direction's sweep), NOT a real /clock reading -- there is
no live mover process here, so there is no sim clock to sample (see
scripts/m4_target_mover.py's module docstring for why a REAL mover needs
one and this offline harness does not).

GROUND TRUTH HONESTY (same boundary as render_sim_dataset.py's gt labels,
per CLAUDE.md's gt_* scoring-only rule): gt_target_x/y/z in index.csv is
exactly the pose THIS SCRIPT COMMANDED via set_pose for that sample -- not
read back from a live pose topic. That is legitimate here because the
target is a teleported, frozen prop for dataset/validation-label purposes
only (T18's triangulation validation), never fed into a guidance decision.
It is the capture-time-only ground truth the task calls for: valid as an
auto-generated dataset LABEL, exactly as a human labeler's measured
ground-truth box would be, and structurally unable to leak into any live
guidance path (this script has no guidance loop at all).

WEAVE REPLAY MODE (T25 shots 1-2; scripts/video/t25_render_plan.md SectB
gap 1, option (a)): `--poses-from <flight_csv>` swaps the built-in line
path for a teleport schedule built from an m4_intercept.py flight CSV's
`t_sim` + `gt_tag_x/y/z` columns -- i.e. the rig re-renders the EXACT
target track a hero flight flew (weave included). Design points:

  * NEAREST-ROW resampling at --rate, NO interpolation: every teleported
    pose is one the flight actually logged (then shifted by the staging
    offsets) -- no invented in-between positions, the same honesty rule
    the video tooling applies to frames (no minterpolate).
  * The schedule window defaults to [mover-start - --pre-roll-s, last row]
    (mover start auto-detected as the first row the target moved
    > 0.05 m from its initial pose; the pre-roll gives the rig honest
    empty/"SEARCHING" frames before the threat starts its run).
    Override with --t-start/--t-end (source-CSV sim-time).
  * --x-shift/--y-shift/--z-shift stage the replayed track relative to the
    rig (worlds/stereo_intercept.sdf pins the rig at broadside_160m,
    (-153.5, 0, 1.8, yaw 0)). The hero corridor at zero shift sits at
    ~160-163 m from the rig (the T16 native band); `--x-shift -25` puts it
    at ~135-139 m, inside the ADR-0053-validated 50-160 m band and the
    storyboard's "first firm track ~120-150 m" staging note. The shift is
    recorded in capture_meta.json and index.csv carries the SHIFTED
    commanded pose (= the pose actually rendered), plus a `t_sim_source`
    column tracing each sample to its source-CSV row.
  * index.csv keeps `t_sim_nominal` = k*dt from schedule start (0-based),
    exactly the line path's semantics -- station.py's ADR-0052 epoch
    rebase (t0 + t_sim_nominal) keeps working unchanged. dash_direction
    is "weave_replay"; dash_speed is the schedule's measured median
    ground speed.
  * `--dry-run` builds and prints the schedule + staging metrics (rig
    range span, first in-frame sample) and exits WITHOUT touching gz --
    validates a replay entirely offline, no sim needed.

    # offline validation (no sim):
    .venv-seeker/bin/python scripts/rig_snapshot_capture.py \\
        --poses-from logs/m4_intercept_pronav_20260709T232447Z.csv \\
        --x-shift -25 --dry-run

    # real capture (sim up on stereo_intercept, ONE sim, idle machine):
    .venv-seeker/bin/python scripts/rig_snapshot_capture.py \\
        --poses-from <hero flight csv> --x-shift -25 \\
        --out logs/rig_captures/t25_weave_replay

RUN (sim already up on stereo_intercept, see scripts/check_t16.sh for the
boot line):

    INTERCEPTOR_WORLD_NAME=stereo_intercept \\
    INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \\
    .venv-seeker/bin/python scripts/rig_snapshot_capture.py \\
        --y0-sign both --out logs/rig_captures/my_run

Smoke check (3 poses, one direction, no full sweep -- scripts/check_t16.sh
uses this):

    .venv-seeker/bin/python scripts/rig_snapshot_capture.py --smoke
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
os.environ.setdefault("INTERCEPTOR_WORLD_NAME", "stereo_intercept")
os.environ.setdefault("INTERCEPTOR_TARGET_MODEL", "fpv_target_markerless")
sys.path.insert(0, HERE)

import cv2  # noqa: E402

from m1_capture import image_msg_to_bgr  # noqa: E402 -- the repo's one RGB_INT8 decoder
from gz.transport13 import Node  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402

WORLD_NAME = os.environ["INTERCEPTOR_WORLD_NAME"]
TARGET_MODEL_NAME = os.environ["INTERCEPTOR_TARGET_MODEL"]
RIG_MODEL_NAME = "ground_stereo_rig"
RIG_LINK_NAME = "link"

LEFT_TOPIC = (
    f"/world/{WORLD_NAME}/model/{RIG_MODEL_NAME}/link/{RIG_LINK_NAME}/sensor/rig_left/image"
)
RIGHT_TOPIC = (
    f"/world/{WORLD_NAME}/model/{RIG_MODEL_NAME}/link/{RIG_LINK_NAME}/sensor/rig_right/image"
)
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"

RIG_HFOV_RAD = 0.349
RIG_WIDTH, RIG_HEIGHT = 1920, 1200
RIG_BASELINE_M = 2.0
# broadside_160m -- docs/rig_geometry_analysis.md Sec 2 / logs/rig_geometry/coverage.csv
RIG_POSE_XYZ_YAW = (-153.5, 0.0, 1.8, 0.0)

# --- Dash-path constants (mirrors scripts/m4_target_mover.py's `line` path
# and scripts/rig_geometry_analysis.py's Analysis 2 EXACTLY, so this
# harness's captured span matches the already-validated wedge geometry). ---
DEFAULT_X0_M = 6.5
DEFAULT_Y0_MAG_M = 29.3
DEFAULT_Z0_M = 0.5
DEFAULT_DASH_SPEED_M_S = 9.0
DEFAULT_RATE_HZ = 10.0
# "from start to y=+5" per the task -- stop just short of CPA (y=0). Same
# name/value/comment as scripts/rig_geometry_analysis.py's
# PATH_BOUNDARY_ABS_Y_M, on purpose: both files describe the same span.
PATH_BOUNDARY_ABS_Y_M = 5.0

TELEPORT_TIMEOUT_S = 4.0
SETTLE_TIMEOUT_S = 8.0
DEFAULT_SETTLE_FRAMES = 3

# --- Weave-replay mode (T25 shots 1-2, see module docstring) ---
MOVER_START_EPS_M = 0.05     # "target has moved" threshold for auto t-start
DEFAULT_PRE_ROLL_S = 1.0     # empty/"SEARCHING" seconds kept before mover start
REPLAY_DIRECTION_LABEL = "weave_replay"


class FrameHolder:
    """Latest decoded BGR frame + a monotonic new-frame counter, one per
    camera. Mirrors scripts/seeker/render_sim_dataset.py's FrameHolder
    (same settle-wait pattern), but decodes via m1_capture.image_msg_to_bgr
    so a pixel_format_type/step drift fails loudly instead of silently
    mis-decoding (the task's explicit RGB_INT8 requirement)."""

    def __init__(self, label: str) -> None:
        self.label = label
        self.count = 0
        self.bgr: "np.ndarray | None" = None
        self.w = 0
        self.h = 0

    def on_image(self, msg: Image) -> None:
        try:
            self.bgr = image_msg_to_bgr(msg)
            self.w, self.h = msg.width, msg.height
            self.count += 1
        except Exception as exc:  # pragma: no cover
            print(f"[rig-capture] {self.label} frame decode error: {exc}")


def teleport(x: float, y: float, z: float, timeout_s: float = TELEPORT_TIMEOUT_S) -> bool:
    """Teleport TARGET_MODEL_NAME via a `gz service` CLI SUBPROCESS -- this
    process holds camera subscriptions, so a same-process node.request(...)
    would never see the response (gz-transport13 quirk; see
    scripts/seeker/render_sim_dataset.py's teleport() and
    scripts/m4_target_mover.py's module docstring for the full story).
    Orientation is left at the message default (identity), matching
    scripts/m4_target_mover.py's send_pose -- the target's facing is baked
    into its geometry (ADR-0010 #6: maneuvers are position-only)."""
    req = f'name: "{TARGET_MODEL_NAME}" position {{ x: {x} y: {y} z: {z} }}'
    cmd = [
        "gz", "service", "-s", SET_POSE_SERVICE,
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", req,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"[rig-capture] teleport failed ({exc})")
        return False
    return r.returncode == 0


def wait_new_frames(holder: FrameHolder, n: int, timeout_s: float = SETTLE_TIMEOUT_S) -> bool:
    start = holder.count
    t0 = time.time()
    while holder.count < start + n:
        if time.time() - t0 > timeout_s:
            return False
        time.sleep(0.03)
    return True


def build_samples(x0, y0_mag, z0, dash_speed, rate_hz, sign, boundary=PATH_BOUNDARY_ABS_Y_M):
    """Return (samples, dash_direction). samples is a list of
    (t_sim_nominal, x, y, z). '+' start is y0=+y0_mag moving toward -Y
    ("r2l" per docs/rig_geometry_analysis.md); '-' start is y0=-y0_mag
    moving toward +Y ("l2r", the mirror). Stops once |y| <= boundary
    (inclusive of that final sample) -- "stop just short of CPA"."""
    if sign == "+":
        y0, vy, direction = y0_mag, -dash_speed, "r2l"
    elif sign == "-":
        y0, vy, direction = -y0_mag, dash_speed, "l2r"
    else:
        raise ValueError(f"sign must be '+' or '-', got {sign!r}")

    dt = 1.0 / rate_hz
    samples = []
    k = 0
    while True:
        t = k * dt
        y = y0 + vy * t
        samples.append((t, x0, y, z0))
        if abs(y) <= boundary:
            break
        k += 1
        if k > 100000:  # runaway guard -- should never trip at sane inputs
            raise RuntimeError("build_samples did not converge on the boundary")
    return samples, direction


def load_flight_gt_poses(csv_path):
    """Read (t_sim, gt_tag_x, gt_tag_y, gt_tag_z) from an m4_intercept.py
    flight CSV, sorted by t_sim. gt_tag_* here is the flight's SCORING-ONLY
    ground truth being reused OFFLINE as a teleport/label schedule -- the
    same boundary as this harness's own gt_target_* columns (see GROUND
    TRUTH HONESTY in the module docstring): it feeds a rendered dataset,
    never a live guidance decision."""
    with open(csv_path, newline="") as fh:
        reader = csv.DictReader(fh)
        need = ("t_sim", "gt_tag_x", "gt_tag_y", "gt_tag_z")
        missing = [c for c in need if c not in (reader.fieldnames or [])]
        if missing:
            raise SystemExit(
                f"[rig-capture] --poses-from {csv_path}: missing column(s) {missing}")
        rows = []
        for r in reader:
            try:
                rows.append((float(r["t_sim"]), float(r["gt_tag_x"]),
                             float(r["gt_tag_y"]), float(r["gt_tag_z"])))
            except (TypeError, ValueError):
                continue  # rows logged before gt columns fill in
    if len(rows) < 2:
        raise SystemExit(
            f"[rig-capture] --poses-from {csv_path}: fewer than 2 usable gt rows")
    rows.sort(key=lambda r: r[0])
    return rows


def detect_mover_start(rows, eps_m=MOVER_START_EPS_M):
    """First source t_sim at which the target has moved > eps_m from its
    initial pose (the mover's start); falls back to the first row's t_sim
    if the target never moves (degenerate, but don't crash a dry-run)."""
    x0, y0, z0 = rows[0][1:4]
    for (t, x, y, z) in rows:
        if math.dist((x, y, z), (x0, y0, z0)) > eps_m:
            return t
    return rows[0][0]


def build_replay_samples(rows, rate_hz, t_start, t_end,
                         x_shift=0.0, y_shift=0.0, z_shift=0.0):
    """Resample the flight's gt_tag schedule at the nominal capture cadence.
    NEAREST-ROW selection, no interpolation (module docstring: every
    teleported pose is one the flight actually logged, plus the staging
    shift). Returns samples of (t_sim_nominal, x, y, z, t_sim_source) --
    t_sim_nominal is k*dt from the schedule start (0-based, the line path's
    exact semantics; station.py's ADR-0052 rebase depends on it),
    t_sim_source is the source-CSV row's own t_sim."""
    dt = 1.0 / rate_hz
    ts = [r[0] for r in rows]
    samples = []
    i = 0
    k = 0
    while True:
        t_k = t_start + k * dt
        if t_k > t_end + 1e-9:
            break
        while i + 1 < len(rows) and abs(ts[i + 1] - t_k) <= abs(ts[i] - t_k):
            i += 1
        t_src, x, y, z = rows[i]
        samples.append((k * dt, x + x_shift, y + y_shift, z + z_shift, t_src))
        k += 1
        if k > 100000:
            raise RuntimeError("build_replay_samples runaway (check --rate/--t-end)")
    return samples


def replay_staging_metrics(samples):
    """Offline staging metrics against the pinned broadside_160m rig pose:
    median ground speed of the schedule, min/max horizontal range from the
    rig origin, and the first sample index whose bearing sits inside the
    rig's hfov wedge (single-point rig approximation -- ignores the 2 m
    stereo baseline; a per-camera frame-entry check is what the render
    itself shows). Used by --dry-run and stamped into capture_meta.json."""
    speeds = []
    for a, b in zip(samples, samples[1:]):
        dt = b[0] - a[0]
        if dt > 0:
            speeds.append(math.dist(a[1:4], b[1:4]) / dt)
    speed_med = float(np.median(speeds)) if speeds else 0.0
    rig_x, rig_y, _rig_z, rig_yaw = RIG_POSE_XYZ_YAW
    half = RIG_HFOV_RAD / 2.0
    ranges = []
    first_in_frame = None
    for idx, s in enumerate(samples):
        dx, dy = s[1] - rig_x, s[2] - rig_y
        ranges.append(math.hypot(dx, dy))
        bearing = math.atan2(dy, dx) - rig_yaw
        bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
        if first_in_frame is None and abs(bearing) <= half:
            first_in_frame = idx
    return speed_med, min(ranges), max(ranges), first_in_frame


def capture_direction(node, holders, samples, direction, args, out_dir, index_rows, seq_start):
    left_dir = os.path.join(out_dir, "left")
    right_dir = os.path.join(out_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    left_holder, right_holder = holders
    seq = seq_start
    n_ok = n_skipped = 0
    for item in samples:
        # line-path samples are (t, x, y, z); replay samples append the
        # source-CSV t_sim as a 5th element (see build_replay_samples).
        t_sim_nominal, x, y, z = item[:4]
        t_sim_source = item[4] if len(item) > 4 else None
        if not teleport(x, y, z):
            print(f"[rig-capture] {direction} seq={seq}: teleport FAILED @ ({x},{y},{z}) -- skipping")
            n_skipped += 1
            seq += 1
            continue
        settle_ok = (
            wait_new_frames(left_holder, args.settle_frames)
            and wait_new_frames(right_holder, args.settle_frames)
        )
        if not settle_ok:
            print(f"[rig-capture] {direction} seq={seq}: settle timeout @ ({x},{y},{z}) -- skipping")
            n_skipped += 1
            seq += 1
            continue

        left_bgr, right_bgr = left_holder.bgr, right_holder.bgr
        if left_bgr is None or right_bgr is None:
            print(f"[rig-capture] {direction} seq={seq}: no decoded frame yet -- skipping")
            n_skipped += 1
            seq += 1
            continue

        left_path = os.path.join(left_dir, f"{seq:05d}.png")
        right_path = os.path.join(right_dir, f"{seq:05d}.png")
        cv2.imwrite(left_path, left_bgr)
        cv2.imwrite(right_path, right_bgr)
        row = {
            "seq": seq,
            "t_sim_nominal": f"{t_sim_nominal:.4f}",
            "gt_target_x": f"{x:.4f}",
            "gt_target_y": f"{y:.4f}",
            "gt_target_z": f"{z:.4f}",
            "dash_direction": direction,
            "dash_speed": f"{args.dash_speed:.3f}",
        }
        if t_sim_source is not None:
            row["t_sim_source"] = f"{t_sim_source:.4f}"
        index_rows.append(row)
        n_ok += 1
        seq += 1

    return seq, n_ok, n_skipped


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--x0", type=float, default=DEFAULT_X0_M,
                    help=f"target start world-X, held fixed (default {DEFAULT_X0_M})")
    ap.add_argument("--y0-mag", type=float, default=DEFAULT_Y0_MAG_M,
                    help=f"|target start world-Y| before the dash (default {DEFAULT_Y0_MAG_M})")
    ap.add_argument("--z0", type=float, default=DEFAULT_Z0_M,
                    help=f"target world-Z / altitude, held fixed (default {DEFAULT_Z0_M})")
    ap.add_argument("--dash-speed", type=float, default=DEFAULT_DASH_SPEED_M_S,
                    help=f"target along-track speed, m/s (default {DEFAULT_DASH_SPEED_M_S})")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                    help="nominal sim-time sampling cadence, Hz -- pose spacing = "
                         f"dash-speed/rate (default {DEFAULT_RATE_HZ})")
    ap.add_argument("--y0-sign", choices=["+", "-", "both"], default="both",
                    help="dash start side: '+' = r2l (start +y0-mag), '-' = l2r "
                         "(start -y0-mag), 'both' = run both directions (default both)")
    ap.add_argument("--settle-frames", type=int, default=DEFAULT_SETTLE_FRAMES,
                    help=f"new frames to wait for per camera after each teleport "
                         f"before capturing (default {DEFAULT_SETTLE_FRAMES})")
    ap.add_argument("--out", default=None,
                    help="output directory (default logs/rig_captures/<UTC runstamp>/)")
    ap.add_argument("--smoke", action="store_true",
                    help="smoke-test mode: 3 poses, one direction only (ignores --y0-sign "
                         "'both' and truncates the sweep) -- used by scripts/check_t16.sh")
    # --- weave-replay mode (T25 shots 1-2; module docstring "WEAVE REPLAY MODE") ---
    ap.add_argument("--poses-from", default=None, metavar="FLIGHT_CSV",
                    help="REPLAY MODE: build the teleport schedule from this "
                         "m4_intercept.py flight CSV's t_sim + gt_tag_x/y/z columns "
                         "instead of the built-in line path (--x0/--y0-mag/"
                         "--dash-speed/--y0-sign are ignored). Nearest-row resample "
                         "at --rate, no interpolation.")
    ap.add_argument("--t-start", type=float, default=None,
                    help="replay window start, SOURCE-CSV sim-time seconds "
                         "(default: auto = mover start - --pre-roll-s)")
    ap.add_argument("--t-end", type=float, default=None,
                    help="replay window end, SOURCE-CSV sim-time seconds "
                         "(default: last usable row)")
    ap.add_argument("--pre-roll-s", type=float, default=DEFAULT_PRE_ROLL_S,
                    help="seconds of pre-mover-start hold kept in the auto window "
                         f"(empty/'SEARCHING' rig frames; default {DEFAULT_PRE_ROLL_S})")
    ap.add_argument("--x-shift", type=float, default=0.0,
                    help="world-X staging offset added to every replayed pose, m "
                         "(-25 puts the hero corridor ~135-139 m from the "
                         "broadside_160m rig, inside the validated 50-160 m band)")
    ap.add_argument("--y-shift", type=float, default=0.0,
                    help="world-Y staging offset, m (default 0)")
    ap.add_argument("--z-shift", type=float, default=0.0,
                    help="world-Z staging offset, m (default 0)")
    ap.add_argument("--dry-run", action="store_true",
                    help="build + print the teleport schedule and staging metrics, "
                         "then exit without touching gz (offline, no sim needed)")
    args = ap.parse_args()

    runstamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.out or os.path.join(REPO_ROOT, "logs", "rig_captures", runstamp)

    # Build every teleport schedule BEFORE touching gz (pure functions), so
    # --dry-run can validate a plan fully offline. plans: (samples, direction,
    # sign) -- sign is None in replay mode.
    replay_meta = None
    if args.poses_from:
        # --- weave-replay mode (module docstring "WEAVE REPLAY MODE") ---
        rows = load_flight_gt_poses(args.poses_from)
        t_move = detect_mover_start(rows)
        t_start = (args.t_start if args.t_start is not None
                   else max(rows[0][0], t_move - args.pre_roll_s))
        t_end = args.t_end if args.t_end is not None else rows[-1][0]
        if t_end <= t_start:
            raise SystemExit(f"[rig-capture] empty replay window: "
                             f"t_start={t_start:.3f} >= t_end={t_end:.3f}")
        samples = build_replay_samples(rows, args.rate, t_start, t_end,
                                       args.x_shift, args.y_shift, args.z_shift)
        if args.smoke:
            samples = samples[:3]
        speed_med, r_min, r_max, first_in = replay_staging_metrics(samples)
        # index rows' dash_speed column = the schedule's measured median
        # ground speed (replay mode has no commanded line speed).
        args.dash_speed = speed_med
        plans = [(samples, REPLAY_DIRECTION_LABEL, None)]
        signs = [REPLAY_DIRECTION_LABEL]  # meta's signs_captured
        replay_meta = {
            "mode": "weave_replay",
            "poses_from": os.path.abspath(args.poses_from),
            "poses_source_rows": len(rows),
            "mover_start_t_sim_source": round(t_move, 4),
            "t_start_source": round(t_start, 4),
            "t_end_source": round(t_end, 4),
            "pre_roll_s": args.pre_roll_s,
            "shift_xyz_m": [args.x_shift, args.y_shift, args.z_shift],
            "replay_speed_med_m_s": round(speed_med, 3),
            "rig_range_min_m": round(r_min, 2),
            "rig_range_max_m": round(r_max, 2),
            "first_in_frame_seq": first_in,
        }
        print(f"[rig-capture] REPLAY schedule from {args.poses_from}")
        print(f"[rig-capture]   source rows={len(rows)} mover_start@t_sim={t_move:.3f} "
              f"window=[{t_start:.3f},{t_end:.3f}]s rate={args.rate} Hz -> {len(samples)} poses")
        print(f"[rig-capture]   shift=({args.x_shift},{args.y_shift},{args.z_shift}) m  "
              f"median ground speed={speed_med:.2f} m/s  "
              f"rig range {r_min:.1f}-{r_max:.1f} m")
        if first_in is None:
            print("[rig-capture]   WARNING: NO sample inside the rig hfov wedge -- "
                  "the whole replay is out of frame; adjust --x-shift/--y-shift")
        else:
            s = samples[first_in]
            print(f"[rig-capture]   first in-frame sample: seq {first_in} @ "
                  f"t_nominal={s[0]:.1f}s pose=({s[1]:.1f},{s[2]:.1f},{s[3]:.2f})")
    else:
        if args.y0_sign in ("+", "-"):
            signs = [args.y0_sign]
        else:  # "both"
            # --smoke wants exactly one direction (the task spec: "3 poses, one
            # direction") -- pick '+' (r2l) as the smoke direction even if the
            # caller left --y0-sign at its 'both' default.
            signs = ["+"] if args.smoke else ["+", "-"]
        plans = []
        for sign in signs:
            samples, direction = build_samples(
                args.x0, args.y0_mag, args.z0, args.dash_speed, args.rate, sign,
            )
            if args.smoke:
                samples = samples[:3]
            plans.append((samples, direction, sign))

    print(f"[rig-capture] world={WORLD_NAME} target={TARGET_MODEL_NAME} rig={RIG_MODEL_NAME}")
    print(f"[rig-capture] left={LEFT_TOPIC}")
    print(f"[rig-capture] right={RIGHT_TOPIC}")
    print(f"[rig-capture] signs={signs} smoke={args.smoke} out={out_dir}")

    if args.dry_run:
        for samples, direction, _sign in plans:
            print(f"[rig-capture] DRY-RUN {direction}: {len(samples)} poses; "
                  f"first/last 3:")
            preview = samples[:3] + (samples[-3:] if len(samples) > 6 else [])
            for s in preview:
                extra = f"  t_src={s[4]:.3f}" if len(s) > 4 else ""
                print(f"    t_nom={s[0]:7.3f}  pose=({s[1]:9.3f},{s[2]:9.3f},{s[3]:6.3f}){extra}")
        print("[rig-capture] DRY-RUN OK: schedule built, nothing captured, gz untouched")
        return 0

    os.makedirs(out_dir, exist_ok=True)

    node = Node()
    left_holder = FrameHolder("rig_left")
    right_holder = FrameHolder("rig_right")
    if not node.subscribe(Image, LEFT_TOPIC, left_holder.on_image):
        print(f"[rig-capture] FAILED: subscribe {LEFT_TOPIC}"); return 2
    if not node.subscribe(Image, RIGHT_TOPIC, right_holder.on_image):
        print(f"[rig-capture] FAILED: subscribe {RIGHT_TOPIC}"); return 2

    print("[rig-capture] waiting for first frame on both rig cameras...")
    t0 = time.time()
    while left_holder.count == 0 or right_holder.count == 0:
        if time.time() - t0 > 20:
            print("[rig-capture] FAILED: no frames within 20s "
                  "(is the stereo_intercept sim up with the rig topics live?)")
            return 2
        time.sleep(0.1)

    index_rows = []
    seq = 0
    total_ok = total_skipped = 0
    for samples, direction, sign in plans:
        if sign is not None:
            print(f"[rig-capture] direction={direction} (sign={sign!r}): {len(samples)} poses")
        else:
            print(f"[rig-capture] direction={direction}: {len(samples)} poses")
        seq, n_ok, n_skipped = capture_direction(
            node, (left_holder, right_holder), samples, direction, args, out_dir,
            index_rows, seq,
        )
        total_ok += n_ok
        total_skipped += n_skipped

    node.unsubscribe(LEFT_TOPIC)
    node.unsubscribe(RIGHT_TOPIC)

    index_path = os.path.join(out_dir, "index.csv")
    fieldnames = [
        "seq", "t_sim_nominal", "gt_target_x", "gt_target_y", "gt_target_z",
        "dash_direction", "dash_speed",
    ]
    if replay_meta is not None:
        # replay rows carry the source-CSV t_sim; appended column so every
        # existing DictReader consumer keeps working unchanged.
        fieldnames.append("t_sim_source")
    with open(index_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(index_rows)

    try:
        git_rev = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT,
            capture_output=True, text=True, timeout=5,
        ).stdout.strip() or "unknown"
    except Exception:
        git_rev = "unknown"

    meta = {
        "world": WORLD_NAME,
        "target_model": TARGET_MODEL_NAME,
        "rig_model": RIG_MODEL_NAME,
        "rig_pose_xyz_yaw": list(RIG_POSE_XYZ_YAW),
        "rig_baseline_m": RIG_BASELINE_M,
        "rig_hfov_rad": RIG_HFOV_RAD,
        "resolution": [RIG_WIDTH, RIG_HEIGHT],
        "left_topic": LEFT_TOPIC,
        "right_topic": RIGHT_TOPIC,
        "dash_x0_m": args.x0,
        "dash_y0_mag_m": args.y0_mag,
        "dash_z0_m": args.z0,
        "dash_speed_m_s": args.dash_speed,
        "rate_hz": args.rate,
        "path_boundary_abs_y_m": PATH_BOUNDARY_ABS_Y_M,
        "signs_captured": signs,
        "smoke": args.smoke,
        "n_captured": total_ok,
        "n_skipped": total_skipped,
        "git_rev": git_rev,
        "runstamp": runstamp,
    }
    meta["mode"] = "line" if replay_meta is None else "weave_replay"
    if replay_meta is not None:
        # replay provenance (appended keys only -- existing consumers read
        # meta by key and are unaffected). NOTE: in replay mode the dash_*
        # keys above describe the UNUSED line-path defaults; "mode" +
        # these keys are authoritative.
        meta.update(replay_meta)
    meta_path = os.path.join(out_dir, "capture_meta.json")
    with open(meta_path, "w") as fh:
        json.dump(meta, fh, indent=2)

    print(f"[rig-capture] wrote {total_ok} L/R pairs ({total_skipped} skipped) -> {out_dir}")
    print(f"[rig-capture] index: {index_path}")
    print(f"[rig-capture] meta:  {meta_path}")
    return 0 if total_ok > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
