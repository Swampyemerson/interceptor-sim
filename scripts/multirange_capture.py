#!/usr/bin/env python3
r"""ADR-0050 consolidated follow-up -- MULTI-RANGE stereo-rig capture.

WHY: the T16 baseline capture (logs/rig_captures/full_sweep_20260709T015530Z)
places the rig at ONE fixed standoff (broadside_160m), so rig-to-target range
is ALWAYS ~160-163 m -- sigma_R proportional-to-R^2 (docs/stereo_design.md's
headline formula) is UNTESTED (ADR-0049/0050). This harness varies RANGE by
moving the RIG to multiple broadside standoffs from the flight corridor
(x=6.5 m), each still looking at the corridor center, and captures a small
grid of target positions at each standoff.

MECHANISM (chosen, justified -- task asked to pick + justify): (b) LIVE
`gz service .../set_pose` on the STATIC `ground_stereo_rig` model between
captures, in ONE sim boot -- NOT (a) N separate world SDFs / N boots.
Justification: `worlds/apriltag_demo.sdf`'s `demo_chase_cam` note documents
this exact mechanism already CONFIRMED WORKING for a static model + camera
sensor ("no reboot needed per iteration"). `ground_stereo_rig` is the same
shape (static link, camera sensor(s) at a link-local pose offset) -- a
static model's whole rigid-body tree (link + child sensors) moves together
under `set_pose` because there is no physics/joint layer to lag it, so the
sensor's WORLD pose (what the renderer uses) updates atomically with the
model's pose on the next render tick. One boot is strictly less wall-clock
risk than four independent PX4/Gazebo boot-settle-teardown cycles (each
adds ~2-3 min and its own chance of an RTF hiccup), and "ONE sim at a time"
is satisfied trivially either way. THE CLAIM IS VERIFIED, NOT JUST ARGUED:
this script's own `--posecheck` output plus the task's own post-hoc image
inspection (documented in the task report) confirms the captured target
apparent size scales with the intended standoff (~3x bigger at 50 m than
160 m, matching the b*f_px/R disparity / f_px*L/R apparent-size relation)
-- proof the camera extrinsics actually moved with the model, not just the
model's abstract pose.

GEOMETRY (reuses scripts/rig_geometry_analysis.py's wedge machinery,
imported not re-derived): standoffs along the broadside direction
(perpendicular to the corridor at its engagement center world (6.5, 0)),
rig at (6.5 - standoff, 0.0, 1.8), yaw=0 (boresight = world +X, dead-ahead
at the corridor center -- the SAME convention as the existing
`broadside_60m/100m/160m` poses in rig_geometry_analysis.py's table;
standoff 160 m reproduces that table's own `broadside_160m` pose exactly, a
useful cross-check). For each standoff, a Y-grid of target world-Y offsets
around the engagement center is verified INSIDE the shared two-camera
wedge via `rig_geometry_analysis.point_in_wedge` (which already accounts
for the +/-1.0 m per-camera baseline offset -- a naive rig-CENTER-only
check overestimates the usable half-width by ~1 m at these ranges,
confirmed while sizing the ADR-0053 grid) before any capture is attempted.

=======================================================================
T17-v2 EXTENSION (ADR-0053 consolidated follow-up: fix the out-of-domain
bias a single-range ground_v1 detector showed at 50 m; multi-range
training dataset).  Three additions over the ADR-0053 campaign:

(1) MORE STANDOFFS, WEDGE-SIZED Y-GRID PER STANDOFF. STANDOFFS_M grew from
    4 points {50,90,130,160} to 7: {50,70,90,110,130,150,160}. Each
    standoff gets its OWN y-grid, auto-sized to a safety-margined fraction
    of that standoff's wedge half-width (`y_grid_for_standoff()` below) --
    ADR-0053 found the naive raw-geometry half-width (R*tan(HFOV/2))
    OVERESTIMATES the safely-usable half-width by ~10-15% once the +/-1 m
    per-camera baseline offset is accounted for (confirmed empirically:
    50 m raw estimate 8.8 m vs. measured-usable ~7.8 m). Rather than
    re-deriving that correction, this sizes a CANDIDATE grid from the raw
    estimate at a conservative fraction (default 0.80, safely under the
    0.885 ratio ADR-0053 measured at the tightest standoff) and then
    directly VERIFIES every generated (standoff, y) point through
    `rig_geometry_analysis.point_in_wedge` (the same pre-flight check the
    original script always ran) -- shrinking iteratively if verification
    still fails, so correctness never depends on the estimate being exact.

(2) X/Z JITTER FOR APPARENT-SIZE VARIETY. At y=0 (boresight center) two
    extra poses jitter target world-X by +/-X_JITTER_M (this DIRECTLY
    perturbs true range within a standoff "bucket" -- a much more potent
    apparent-size lever than the Y-grid, which barely changes range near
    boresight) and two more jitter target world-Z (altitude) by
    +/-Z_JITTER_M (small vertical variety; the wedge model is horizontal-
    only -- see the docstring note on `wedge check does not model
    elevation` below for why this is safe at these jitter magnitudes).

(3) INTEGRATED NEGATIVE CAPTURE. A second, optional phase
    (`capture_negatives()`, default ON, `--no-negatives` to skip) grabs
    empty-wedge negative frames at a handful of standoffs -- reusing
    `capture_rig_negatives.py`'s proven "yaw the rig 90 deg away from the
    corridor so the parked interceptor (always dead-center of the yaw=0
    wedge, independent of standoff -- see that script's own comment) is
    out of frame" mechanism, generalized to run at ANY rig X position (the
    original script hard-coded one rig pose). Negatives are tagged with
    the rig standoff they were captured at (`rig_standoff_m` in
    negatives/index.csv) so a downstream dataset builder can split them
    by the SAME held-out-standoff rule as the positives (see
    scripts/seeker/build_ground_dataset_v2.py) -- this project's flat,
    textureless world was already shown (ADR-0049) to produce near-
    identical negative frames regardless of camera position, so this is a
    disclosed, not-hidden, limited-diversity capture, not a claim of
    genuine background diversity.

CAPTURE ORDER per standoff: teleport rig -> wait for NEW frames on BOTH
cameras (proves the render reflects the new extrinsics, not a stale
subscription) -> for each target pose (grid + jitter): teleport target ->
wait for new frames -> grab one L/R pair. Reuses the exact
FrameHolder/wait_new_frames/teleport-via-gz-service-subprocess pattern from
scripts/rig_snapshot_capture.py (gz-transport13 quirk: THIS process holds
camera subscriptions, so a same-process node.request(...) would never see a
service RESPONSE -- only a `gz service` CLI subprocess call avoids that,
"the mover pattern").

OUTPUT: one subdirectory per standoff (`standoff_050m/`, `_070m/`, ...),
each with `left/`, `right/`, `index.csv`, `capture_meta.json` -- SAME SCHEMA
as scripts/rig_snapshot_capture.py's output (rig_pose_xyz_yaw,
rig_baseline_m, rig_hfov_rad, resolution, ...), so
scripts/ground_station/triangulate.py's `RigConfig.from_capture_meta` and
scripts/t18_sigma_validation.py's `load_capture`/`run_capture_sigma_R` work
UNCHANGED against each standoff directory -- only the centroid/geometry
SOURCE differs (a static grid+jitter set, not a moving dash), which those
functions don't care about (they only read capture_meta.json's rig geometry
+ index.csv's per-row gt_target_x/y/z). Plus one `negatives/` subdirectory
(left/right/index.csv, T17-v2 addition) and one top-level
`campaign_meta.json` recording every standoff's pose + wedge-coverage check.

VENV: needs cv2 + gz-transport13 -- run under .venv-seeker, same as
rig_snapshot_capture.py:
    INTERCEPTOR_WORLD_NAME=stereo_intercept \
    INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \
    .venv-seeker/bin/python scripts/multirange_capture.py --out logs/rig_captures/multirange_<stamp>
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
import rig_geometry_analysis as rga  # noqa: E402 -- reuse the wedge/pose tool, not re-derive

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

# --- Multi-range geometry (matches models/ground_stereo_rig, docs/stereo_design.md) ---
X0_M = 6.5              # corridor engagement center world-X (rig_geometry_analysis.X0_M)
RIG_Z_M = 1.8            # mount height, docs/stereo_design.md
TARGET_Z_M = 0.5         # target altitude, matches the T16 dash profile

# T17-v2: 7 standoffs (was 4). 70 m and 130 m are the DEFAULT held-out-range
# eval standoffs for scripts/seeker/build_ground_dataset_v2.py -- capture
# doesn't need to know that (it just captures every standoff identically);
# the split decision lives entirely in the dataset builder.
STANDOFFS_M = [50.0, 70.0, 90.0, 110.0, 130.0, 150.0, 160.0]

# Wedge-sized Y-grid (T17-v2): each standoff gets its OWN grid, computed by
# y_grid_for_standoff() below -- NOT a single shared list (ADR-0053 found a
# fixed +/-6 m grid is fine at 90-160 m but eats most of the 50 m wedge's
# margin). --y-grid on the CLI overrides this with a single manually-supplied
# grid applied uniformly to every standoff (the OLD v1 behavior), for anyone
# who wants that instead.
N_Y_GRID = 9
Y_GRID_FRAC = 0.80  # fraction of the RAW (baseline-uncorrected) wedge half-
                     # width to target; safely under the ~0.885 usable
                     # fraction ADR-0053 measured at the tightest (50 m)
                     # standoff once verified via point_in_wedge below.

# T17-v2: x/z jitter poses at y=0 (boresight center), apparent-size variety.
# X jitter changes TRUE RANGE directly (the potent lever); Z jitter adds a
# small altitude perturbation. Both applied per standoff.
X_JITTER_M = [-5.0, 5.0]
Z_JITTER_M = [-0.3, 0.3]

# T17-v2: integrated negative capture (extends capture_rig_negatives.py's
# mechanism to run at multiple standoffs in the same boot). NEG_YAW_RAD is
# an ABSOLUTE yaw (not relative to each standoff's home yaw=0): at yaw=+90
# deg the boresight points along world+Y, which is >=90 deg off the bearing
# to the parked interceptor (always near world origin, i.e. bearing 0 or pi
# from any rig position on the y=0 line) for EVERY standoff in STANDOFFS_M
# -- see the module docstring's T17-v2 EXTENSION item (3) for the full
# derivation. NEG_STANDOFFS_M deliberately includes BOTH held-out-range
# standoffs (70, 130) and two train standoffs (50, 160) so the downstream
# dataset builder can split negatives by the same held-out rule as
# positives and still have some held-out negatives to report a real
# (non-nan) held-out neg-clean-rate.
NEG_STANDOFFS_M = [50.0, 70.0, 130.0, 160.0]
NEG_YAW_RAD = math.pi / 2.0
NEG_LATERAL_OFFSETS_M = [-200.0, 200.0]
NEG_UP_Z_M = 60.0
NEG_N_PER_POS = 5

TELEPORT_TIMEOUT_S = 4.0
SETTLE_TIMEOUT_S = 8.0
TARGET_SETTLE_FRAMES = 3
RIG_SETTLE_FRAMES = 5   # extra margin -- a bigger structural change than a target hop


class FrameHolder:
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
            print(f"[multirange] {self.label} frame decode error: {exc}")


def teleport_model(name: str, x: float, y: float, z: float, yaw: float = 0.0,
                    timeout_s: float = TELEPORT_TIMEOUT_S) -> bool:
    """Generic `gz service` CLI SUBPROCESS teleport. `yaw` defaults to 0.0
    (identity orientation, byte-for-byte equivalent to the v1 script's
    orientation-omitted request, since sin(0)=0/cos(0)=1 is exactly the
    identity quaternion) -- every GRID/JITTER pose this campaign uses has
    yaw=0; only the T17-v2 negative-capture rig teleports use a non-zero
    yaw (see teleport_model(..., yaw=NEG_YAW_RAD) calls below). Subprocess,
    not node.request(...): this process holds camera subscriptions, so a
    same-process request would never see the response (gz-transport13
    quirk, "the mover pattern")."""
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    req = (f'name: "{name}" position {{ x: {x} y: {y} z: {z} }} '
           f'orientation {{ x: 0 y: 0 z: {qz} w: {qw} }}')
    cmd = [
        "gz", "service", "-s", SET_POSE_SERVICE,
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", req,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"[multirange] teleport({name}) failed ({exc})")
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


def wedge_half_width_estimate(standoff_m: float) -> float:
    """Raw geometric half-width at the rig CENTER (no per-camera baseline
    correction) -- R * tan(HFOV/2). A deliberate OVER-estimate (ADR-0053);
    y_grid_for_standoff() shrinks from here until point_in_wedge (which DOES
    apply the baseline correction) actually confirms every point."""
    return standoff_m * math.tan(RIG_HFOV_RAD / 2.0)


def y_grid_for_standoff(rx: float, ry: float, yaw: float, standoff_m: float,
                         n: int = N_Y_GRID, frac: float = Y_GRID_FRAC):
    """Wedge-sized Y-grid (T17-v2 item 1): start from `frac` of the raw
    half-width estimate, and if the extreme points don't actually verify
    inside the shared (baseline-corrected) wedge, shrink by 8% and retry.
    Always terminates (frac shrinks toward 0), and the caller still runs
    the SAME point_in_wedge check on every generated point before touching
    the sim (belt-and-suspenders, not a replacement for that check)."""
    cand = frac * wedge_half_width_estimate(standoff_m)
    for _ in range(40):
        ok_pos, _ = rga.point_in_wedge(rx, ry, yaw, (X0_M, cand))
        ok_neg, _ = rga.point_in_wedge(rx, ry, yaw, (X0_M, -cand))
        if ok_pos and ok_neg:
            break
        cand *= 0.92
    ys = np.linspace(-cand, cand, n)
    return [float(round(y, 4)) for y in ys]


def build_poses_for_standoff(rig_pose, standoff_m: float, n_y: int = N_Y_GRID,
                              y_grid_frac: float = Y_GRID_FRAC,
                              x_jitter=X_JITTER_M, z_jitter=Z_JITTER_M,
                              manual_y_grid=None):
    """Returns a list of (label, x, y, z) target poses for one standoff:
    the wedge-sized Y-grid (label 'grid') plus X/Z jitter poses at y=0
    (labels 'jitter_x_plus/minus', 'jitter_z_plus/minus') -- T17-v2 items
    1+2. `manual_y_grid`, if given, REPLACES the auto-sized grid with a
    fixed list applied as-is (the v1 CLI's --y-grid override path)."""
    rx, ry, rz, yaw = rig_pose
    y_grid = (list(manual_y_grid) if manual_y_grid is not None
              else y_grid_for_standoff(rx, ry, yaw, standoff_m, n=n_y, frac=y_grid_frac))
    poses = [("grid", X0_M, y, TARGET_Z_M) for y in y_grid]
    for dx in x_jitter:
        tag = "jitter_x_plus" if dx > 0 else "jitter_x_minus"
        poses.append((tag, X0_M + dx, 0.0, TARGET_Z_M))
    for dz in z_jitter:
        tag = "jitter_z_plus" if dz > 0 else "jitter_z_minus"
        poses.append((tag, X0_M, 0.0, TARGET_Z_M + dz))
    return poses


def wedge_check_poses(rig_pose, poses):
    """Checks every (x, y) of `poses` against the shared wedge (horizontal-
    plane check only -- see the module docstring's T17-v2 item 2 note: Z
    jitter is not modelled by point_in_wedge, but +/-0.3 m against a ~1.3 m
    rig-to-target height difference over 50-160 m ranges is well inside the
    camera's own vertical half-angle margin, so this is a disclosed,
    reasoned omission, not an oversight)."""
    rx, ry, rz, yaw = rig_pose
    rows = []
    all_ok = True
    for label, x, y, z in poses:
        in_wedge, rng = rga.point_in_wedge(rx, ry, yaw, (x, y))
        rows.append({"label": label, "x": x, "y": y, "z": z,
                     "in_wedge": bool(in_wedge), "range_m": round(rng, 3)})
        all_ok = all_ok and in_wedge
    return rows, all_ok


def true_range(rig_pose, target_xyz):
    rx, ry, rz, _ = rig_pose
    tx, ty, tz = target_xyz
    return float(np.sqrt((tx - rx) ** 2 + (ty - ry) ** 2 + (tz - rz) ** 2))


def capture_standoff(node, holders, standoff_m, rig_pose, poses, out_root, git_rev, runstamp):
    left_holder, right_holder = holders
    rx, ry, rz, yaw = rig_pose
    tag = f"standoff_{int(round(standoff_m)):03d}m"
    out_dir = os.path.join(out_root, tag)
    left_dir = os.path.join(out_dir, "left")
    right_dir = os.path.join(out_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    print(f"[multirange] === standoff={standoff_m} m -> teleporting rig to "
          f"({rx},{ry},{rz}) yaw={yaw} ({len(poses)} poses) ===")
    if not teleport_model(RIG_MODEL_NAME, rx, ry, rz, yaw=yaw):
        print(f"[multirange] FAIL: rig teleport failed for standoff={standoff_m}")
        return None
    settle_ok = (
        wait_new_frames(left_holder, RIG_SETTLE_FRAMES)
        and wait_new_frames(right_holder, RIG_SETTLE_FRAMES)
    )
    if not settle_ok:
        print(f"[multirange] FAIL: rig-move settle timeout @ standoff={standoff_m}")
        return None
    print(f"[multirange] rig settled ({RIG_SETTLE_FRAMES} new frames each camera)")

    index_rows = []
    n_ok = n_skipped = 0
    for seq, (label, x, y, z) in enumerate(poses):
        if not teleport_model(TARGET_MODEL_NAME, x, y, z):
            print(f"[multirange]   seq={seq} {label} y={y}: target teleport FAILED -- skipping")
            n_skipped += 1
            continue
        settle_ok = (
            wait_new_frames(left_holder, TARGET_SETTLE_FRAMES)
            and wait_new_frames(right_holder, TARGET_SETTLE_FRAMES)
        )
        if not settle_ok:
            print(f"[multirange]   seq={seq} {label}: settle timeout -- skipping")
            n_skipped += 1
            continue
        left_bgr, right_bgr = left_holder.bgr, right_holder.bgr
        if left_bgr is None or right_bgr is None:
            print(f"[multirange]   seq={seq} {label}: no decoded frame yet -- skipping")
            n_skipped += 1
            continue

        left_path = os.path.join(left_dir, f"{seq:05d}.png")
        right_path = os.path.join(right_dir, f"{seq:05d}.png")
        cv2.imwrite(left_path, left_bgr)
        cv2.imwrite(right_path, right_bgr)
        r_true = true_range(rig_pose, (x, y, z))
        index_rows.append({
            "seq": seq,
            "t_sim_nominal": f"{seq:.4f}",
            "gt_target_x": f"{x:.4f}",
            "gt_target_y": f"{y:.4f}",
            "gt_target_z": f"{z:.4f}",
            "dash_direction": label,
            "dash_speed": "0.0",
        })
        print(f"[multirange]   seq={seq} {label:>14} x={x:.1f} y={y:+.2f} z={z:.2f}  "
              f"true_range={r_true:.3f} m  captured OK")
        n_ok += 1

    index_path = os.path.join(out_dir, "index.csv")
    with open(index_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=[
            "seq", "t_sim_nominal", "gt_target_x", "gt_target_y", "gt_target_z",
            "dash_direction", "dash_speed",
        ])
        writer.writeheader()
        writer.writerows(index_rows)

    meta = {
        "world": WORLD_NAME,
        "target_model": TARGET_MODEL_NAME,
        "rig_model": RIG_MODEL_NAME,
        "rig_pose_xyz_yaw": [rx, ry, rz, yaw],
        "rig_baseline_m": RIG_BASELINE_M,
        "rig_hfov_rad": RIG_HFOV_RAD,
        "resolution": [RIG_WIDTH, RIG_HEIGHT],
        "left_topic": LEFT_TOPIC,
        "right_topic": RIGHT_TOPIC,
        "standoff_m": standoff_m,
        "dash_x0_m": X0_M,
        "dash_y0_mag_m": max(abs(p[2]) for p in poses),
        "dash_z0_m": TARGET_Z_M,
        "dash_speed_m_s": 0.0,
        "rate_hz": 0.0,
        "poses": [{"label": p[0], "x": p[1], "y": p[2], "z": p[3]} for p in poses],
        "capture_mode": "static_multirange_grid_plus_jitter",
        "signs_captured": sorted(set(p[0] for p in poses)),
        "smoke": False,
        "n_captured": n_ok,
        "n_skipped": n_skipped,
        "git_rev": git_rev,
        "runstamp": runstamp,
    }
    meta_path = os.path.join(out_dir, "capture_meta.json")
    with open(meta_path, "w") as fh:
        json.dump(meta, fh, indent=2)

    print(f"[multirange] standoff={standoff_m} m: wrote {n_ok} L/R pairs "
          f"({n_skipped} skipped) -> {out_dir}")
    return {"tag": tag, "out_dir": out_dir, "n_captured": n_ok, "n_skipped": n_skipped,
            "meta_path": meta_path, "index_path": index_path}


# ============================================================================
# T17-v2: integrated negative capture (extends capture_rig_negatives.py's
# mechanism to run at multiple standoffs within this same boot/campaign).
# ============================================================================
def lateral_offset_world(rx: float, ry: float, yaw: float, lateral_m: float):
    """A point `lateral_m` along the rig's own LEFT axis (perpendicular to
    its current boresight `yaw`) -- i.e. always OUT of the wedge regardless
    of yaw, the same "local +/-Y" trick capture_rig_negatives.py used at a
    single fixed (yaw=0) rig pose, generalized to any yaw."""
    lat = (-math.sin(yaw), math.cos(yaw))
    return (rx + lateral_m * lat[0], ry + lateral_m * lat[1])


def capture_negatives(node, holders, standoffs, out_root, git_rev, runstamp,
                       n_per_pos: int = NEG_N_PER_POS):
    """Empty-wedge negative burst at each standoff in `standoffs`: yaw the
    rig to NEG_YAW_RAD (absolute +90 deg -- see the module-level constant's
    comment for why this keeps the parked interceptor out of frame at EVERY
    standoff, not just one hard-coded pose) and grab frames at two lateral
    positions + one straight-up position, all outside the new boresight.
    Every row records `rig_standoff_m` so a downstream dataset builder can
    split negatives by the same held-out-standoff rule as positives."""
    left_holder, right_holder = holders
    out_dir = os.path.join(out_root, "negatives")
    left_dir = os.path.join(out_dir, "left")
    right_dir = os.path.join(out_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    index_rows = []
    seq = 0
    n_ok = n_skipped = 0
    for standoff_m in standoffs:
        rx, ry, rz = X0_M - standoff_m, 0.0, RIG_Z_M
        print(f"[multirange-neg] standoff={standoff_m} m: yawing rig to "
              f"{NEG_YAW_RAD:.4f} rad (clear of the parked interceptor)...")
        if not teleport_model(RIG_MODEL_NAME, rx, ry, rz, yaw=NEG_YAW_RAD):
            print(f"[multirange-neg] standoff={standoff_m}: rig teleport FAILED -- skipping")
            n_skipped += len(NEG_LATERAL_OFFSETS_M) * n_per_pos + n_per_pos
            continue
        if not (wait_new_frames(left_holder, RIG_SETTLE_FRAMES)
                and wait_new_frames(right_holder, RIG_SETTLE_FRAMES)):
            print(f"[multirange-neg] standoff={standoff_m}: settle timeout after rig yaw -- skipping")
            n_skipped += len(NEG_LATERAL_OFFSETS_M) * n_per_pos + n_per_pos
            continue

        positions = []
        for lat_m in NEG_LATERAL_OFFSETS_M:
            lx, ly = lateral_offset_world(rx, ry, NEG_YAW_RAD, lat_m)
            tag = "lateral_plus" if lat_m > 0 else "lateral_minus"
            positions.append((tag, lx, ly, 0.5))
        positions.append(("far_up", rx, ry, NEG_UP_Z_M))

        for pos_name, x, y, z in positions:
            if not teleport_model(TARGET_MODEL_NAME, x, y, z):
                print(f"[multirange-neg]   {pos_name}: target teleport FAILED -- skipping burst")
                n_skipped += n_per_pos
                continue
            if not (wait_new_frames(left_holder, TARGET_SETTLE_FRAMES)
                    and wait_new_frames(right_holder, TARGET_SETTLE_FRAMES)):
                print(f"[multirange-neg]   {pos_name}: settle timeout -- skipping burst")
                n_skipped += n_per_pos
                continue
            for i in range(n_per_pos):
                if not (wait_new_frames(left_holder, 1) and wait_new_frames(right_holder, 1)):
                    n_skipped += 1
                    seq += 1
                    continue
                left_bgr, right_bgr = left_holder.bgr, right_holder.bgr
                if left_bgr is None or right_bgr is None:
                    n_skipped += 1
                    seq += 1
                    continue
                cv2.imwrite(os.path.join(left_dir, f"{seq:05d}.png"), left_bgr)
                cv2.imwrite(os.path.join(right_dir, f"{seq:05d}.png"), right_bgr)
                index_rows.append({
                    "seq": seq, "rig_standoff_m": standoff_m, "pos_name": pos_name,
                    "gt_target_x": f"{x:.4f}", "gt_target_y": f"{y:.4f}", "gt_target_z": f"{z:.4f}",
                })
                n_ok += 1
                seq += 1
            print(f"[multirange-neg]   standoff={standoff_m} {pos_name}: captured {n_per_pos} pairs")

    print(f"[multirange-neg] restoring rig to home pose ({X0_M - standoffs[-1]},0,{RIG_Z_M},yaw=0)...")
    teleport_model(RIG_MODEL_NAME, X0_M - standoffs[-1], 0.0, RIG_Z_M, yaw=0.0)

    index_path = os.path.join(out_dir, "index.csv")
    with open(index_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=[
            "seq", "rig_standoff_m", "pos_name", "gt_target_x", "gt_target_y", "gt_target_z"])
        writer.writeheader()
        writer.writerows(index_rows)

    meta = {
        "world": WORLD_NAME, "target_model": TARGET_MODEL_NAME, "rig_model": RIG_MODEL_NAME,
        "standoffs_m": standoffs, "n_per_pos": n_per_pos,
        "n_captured": n_ok, "n_skipped": n_skipped, "runstamp": runstamp, "git_rev": git_rev,
        "note": "T17-v2: negatives captured at MULTIPLE rig standoffs (yawed 90 deg away from "
                "the corridor at each), tagged with rig_standoff_m for held-out-standoff "
                "splitting. ADR-0049 already found this flat, textureless world produces "
                "near-identical frames regardless of camera position/standoff -- disclosed, "
                "not hidden; see the v2 dataset README for whether that held here too.",
    }
    with open(os.path.join(out_dir, "capture_meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)

    print(f"[multirange-neg] wrote {n_ok} L/R pairs ({n_skipped} skipped) -> {out_dir}")
    return {"out_dir": out_dir, "n_captured": n_ok, "n_skipped": n_skipped,
            "meta_path": os.path.join(out_dir, "capture_meta.json"),
            "index_path": index_path}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None, help="output directory "
                     "(default logs/rig_captures/multirange_<UTC runstamp>/)")
    ap.add_argument("--standoffs", type=float, nargs="+", default=STANDOFFS_M)
    ap.add_argument("--y-grid", type=float, nargs="+", default=None,
                     help="manual override: apply this fixed Y-grid to EVERY standoff "
                          "(v1 behavior). Default: auto wedge-sized PER standoff.")
    ap.add_argument("--n-y-grid", type=int, default=N_Y_GRID)
    ap.add_argument("--y-grid-frac", type=float, default=Y_GRID_FRAC)
    ap.add_argument("--x-jitter", type=float, nargs="*", default=X_JITTER_M)
    ap.add_argument("--z-jitter", type=float, nargs="*", default=Z_JITTER_M)
    ap.add_argument("--negatives", dest="negatives", action="store_true", default=True)
    ap.add_argument("--no-negatives", dest="negatives", action="store_false")
    ap.add_argument("--neg-standoffs", type=float, nargs="+", default=NEG_STANDOFFS_M)
    ap.add_argument("--neg-n-per-pos", type=int, default=NEG_N_PER_POS)
    args = ap.parse_args()

    runstamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_root = args.out or os.path.join(REPO_ROOT, "logs", "rig_captures", f"multirange_{runstamp}")
    os.makedirs(out_root, exist_ok=True)

    print(f"[multirange] world={WORLD_NAME} target={TARGET_MODEL_NAME} rig={RIG_MODEL_NAME}")
    print(f"[multirange] standoffs={args.standoffs}")
    print(f"[multirange] out={out_root}")

    # ---- build every standoff's pose set + wedge-coverage pre-check, before
    # touching the sim at all ----
    print("\n[multirange] === WEDGE COVERAGE CHECK (rig_geometry_analysis.point_in_wedge) ===")
    poses_by_standoff = {}
    campaign_wedge = {}
    any_fail = False
    for standoff in args.standoffs:
        rx, ry, yaw = X0_M - standoff, 0.0, 0.0
        rig_pose = (rx, ry, RIG_Z_M, yaw)
        poses = build_poses_for_standoff(
            rig_pose, standoff, n_y=args.n_y_grid, y_grid_frac=args.y_grid_frac,
            x_jitter=args.x_jitter, z_jitter=args.z_jitter, manual_y_grid=args.y_grid,
        )
        poses_by_standoff[standoff] = (rig_pose, poses)
        rows, all_ok = wedge_check_poses(rig_pose, poses)
        campaign_wedge[standoff] = {"rig_pose_xyz_yaw": list(rig_pose), "rows": rows, "all_ok": all_ok}
        flag = "OK" if all_ok else "FAIL"
        print(f"[multirange] standoff={standoff:6.1f} m  rig_pose={rig_pose}  "
              f"n_poses={len(poses)}  wedge={flag}")
        for r in rows:
            print(f"    {r['label']:>14} x={r['x']:.1f} y={r['y']:+.2f}  "
                  f"in_wedge={r['in_wedge']}  range={r['range_m']:.2f} m")
        if not all_ok:
            any_fail = True
    if any_fail:
        print("[multirange] FAIL: at least one pose is outside the shared wedge "
              "-- fix the grid before capturing. Aborting, no sim interaction attempted.")
        return 2
    print("[multirange] wedge check: ALL standoffs/poses inside the shared wedge. Proceeding.")

    try:
        git_rev = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT,
            capture_output=True, text=True, timeout=5,
        ).stdout.strip() or "unknown"
    except Exception:
        git_rev = "unknown"

    node = Node()
    left_holder = FrameHolder("rig_left")
    right_holder = FrameHolder("rig_right")
    if not node.subscribe(Image, LEFT_TOPIC, left_holder.on_image):
        print(f"[multirange] FAILED: subscribe {LEFT_TOPIC}"); return 2
    if not node.subscribe(Image, RIGHT_TOPIC, right_holder.on_image):
        print(f"[multirange] FAILED: subscribe {RIGHT_TOPIC}"); return 2

    print("[multirange] waiting for first frame on both rig cameras...")
    t0 = time.time()
    while left_holder.count == 0 or right_holder.count == 0:
        if time.time() - t0 > 20:
            print("[multirange] FAILED: no frames within 20s "
                  "(is the stereo_intercept sim up with the rig topics live?)")
            return 2
        time.sleep(0.1)
    print(f"[multirange] first frames received ({left_holder.w}x{left_holder.h})")

    standoff_results = []
    for standoff in args.standoffs:
        rig_pose, poses = poses_by_standoff[standoff]
        res = capture_standoff(node, (left_holder, right_holder), standoff, rig_pose, poses,
                                out_root, git_rev, runstamp)
        if res is None:
            print(f"[multirange] ABORT: standoff={standoff} capture failed")
            node.unsubscribe(LEFT_TOPIC)
            node.unsubscribe(RIGHT_TOPIC)
            return 2
        standoff_results.append(res)

    negatives_result = None
    if args.negatives:
        print("\n[multirange] === NEGATIVE CAPTURE (T17-v2) ===")
        negatives_result = capture_negatives(
            node, (left_holder, right_holder), args.neg_standoffs, out_root, git_rev, runstamp,
            n_per_pos=args.neg_n_per_pos,
        )

    node.unsubscribe(LEFT_TOPIC)
    node.unsubscribe(RIGHT_TOPIC)

    total_ok = sum(r["n_captured"] for r in standoff_results)
    total_skipped = sum(r["n_skipped"] for r in standoff_results)

    campaign_meta = {
        "world": WORLD_NAME,
        "target_model": TARGET_MODEL_NAME,
        "rig_model": RIG_MODEL_NAME,
        "standoffs_m": args.standoffs,
        "n_y_grid": args.n_y_grid,
        "y_grid_frac": args.y_grid_frac,
        "x_jitter_m": args.x_jitter,
        "z_jitter_m": args.z_jitter,
        "rig_z_m": RIG_Z_M,
        "target_x0_m": X0_M,
        "target_z0_m": TARGET_Z_M,
        "rig_baseline_m": RIG_BASELINE_M,
        "rig_hfov_rad": RIG_HFOV_RAD,
        "resolution": [RIG_WIDTH, RIG_HEIGHT],
        "wedge_check": {str(k): v for k, v in campaign_wedge.items()},
        "standoff_dirs": standoff_results,
        "negatives": negatives_result,
        "n_captured_total": total_ok,
        "n_skipped_total": total_skipped,
        "git_rev": git_rev,
        "runstamp": runstamp,
        "mechanism": "b_live_set_pose_single_boot",
        "schema_version": 2,
    }
    campaign_meta_path = os.path.join(out_root, "campaign_meta.json")
    with open(campaign_meta_path, "w") as fh:
        json.dump(campaign_meta, fh, indent=2)

    print(f"\n[multirange] DONE: {total_ok} L/R positive pairs total ({total_skipped} skipped) "
          f"+ {negatives_result['n_captured'] if negatives_result else 0} negative pairs -> {out_root}")
    print(f"[multirange] campaign meta: {campaign_meta_path}")
    return 0 if total_ok > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
