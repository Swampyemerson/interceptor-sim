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
imported not re-derived): four standoffs along the broadside direction
(perpendicular to the corridor at its engagement center world (6.5, 0)),
rig at (6.5 - standoff, 0.0, 1.8), yaw=0 (boresight = world +X, dead-ahead
at the corridor center -- the SAME convention as the existing
`broadside_60m/100m/160m` poses in that script's RIG_POSES table; standoff
160 m reproduces that table's own `broadside_160m` pose exactly, a useful
cross-check). For each standoff, a small grid of target world-Y offsets
around the engagement center (Y_GRID_M) is verified INSIDE the shared
two-camera wedge via `rig_geometry_analysis.point_in_wedge` (which already
accounts for the +/-1.0 m per-camera baseline offset -- a naive
rig-CENTER-only check overestimates the usable half-width by ~1 m at these
ranges, confirmed while sizing this grid) before any capture is attempted.

CAPTURE ORDER per standoff: teleport rig -> wait for NEW frames on BOTH
cameras (proves the render reflects the new extrinsics, not a stale
subscription) -> for each target Y offset: teleport target -> wait for new
frames -> grab one L/R pair. Reuses the exact
FrameHolder/wait_new_frames/teleport-via-gz-service-subprocess pattern from
scripts/rig_snapshot_capture.py (gz-transport13 quirk: THIS process holds
camera subscriptions, so a same-process node.request(...) would never see a
service RESPONSE -- only a `gz service` CLI subprocess call avoids that,
"the mover pattern").

OUTPUT: one subdirectory per standoff (`standoff_050m/`, `_090m/`, `_130m/`,
`_160m/`), each with `left/`, `right/`, `index.csv`, `capture_meta.json` --
SAME SCHEMA as scripts/rig_snapshot_capture.py's output (rig_pose_xyz_yaw,
rig_baseline_m, rig_hfov_rad, resolution, ...), so
scripts/ground_station/triangulate.py's `RigConfig.from_capture_meta` and
scripts/t18_sigma_validation.py's `load_capture`/`run_capture_sigma_R` work
UNCHANGED against each standoff directory -- only the centroid/geometry
SOURCE differs (a static y-grid, not a moving dash), which those functions
don't care about (they only read capture_meta.json's rig geometry +
index.csv's per-row gt_target_x/y/z). Plus one top-level
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
STANDOFFS_M = [50.0, 90.0, 130.0, 160.0]
# +/-6 m, verified INSIDE the shared (per-camera, +/-1.0 m baseline
# corrected) wedge at EVERY standoff incl. the tightest (50 m: usable
# half-width ~7.8 m after the baseline correction -- +/-8 m alone is
# borderline/fails, confirmed by direct point_in_wedge() check while
# sizing this grid, see task report).
Y_GRID_M = [-6.0, -3.0, 0.0, 3.0, 6.0]

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


def teleport_model(name: str, x: float, y: float, z: float,
                    timeout_s: float = TELEPORT_TIMEOUT_S) -> bool:
    """Generic `gz service` CLI SUBPROCESS teleport (position only, identity
    orientation -- every pose this campaign uses has yaw=0, matching
    rig_geometry_analysis.py's broadside-pose convention, so this
    deliberately does NOT do quaternion math -- see module docstring).
    Subprocess, not node.request(...): this process holds camera
    subscriptions, so a same-process request would never see the response
    (gz-transport13 quirk, "the mover pattern")."""
    req = f'name: "{name}" position {{ x: {x} y: {y} z: {z} }}'
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


def wedge_check_for_standoff(standoff_m: float, y_grid=Y_GRID_M):
    """Reuses rig_geometry_analysis.point_in_wedge -- returns (rig_pose,
    [{y, in_wedge, range_m}, ...], all_ok)."""
    rx = X0_M - standoff_m
    ry = 0.0
    yaw = 0.0  # aim=(X0_M, 0.0), rig at (rx, 0.0) -> atan2(0,.)=0, matches broadside_* convention
    rows = []
    all_ok = True
    for y in y_grid:
        in_wedge, rng = rga.point_in_wedge(rx, ry, yaw, (X0_M, float(y)))
        rows.append({"y": y, "in_wedge": bool(in_wedge), "range_m": round(rng, 3)})
        all_ok = all_ok and in_wedge
    return (rx, ry, RIG_Z_M, yaw), rows, all_ok


def true_range(rig_pose, target_xyz):
    rx, ry, rz, _ = rig_pose
    tx, ty, tz = target_xyz
    return float(np.sqrt((tx - rx) ** 2 + (ty - ry) ** 2 + (tz - rz) ** 2))


def capture_standoff(node, holders, standoff_m, rig_pose, y_grid, out_root, git_rev, runstamp):
    left_holder, right_holder = holders
    rx, ry, rz, yaw = rig_pose
    tag = f"standoff_{int(round(standoff_m)):03d}m"
    out_dir = os.path.join(out_root, tag)
    left_dir = os.path.join(out_dir, "left")
    right_dir = os.path.join(out_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    print(f"[multirange] === standoff={standoff_m} m -> teleporting rig to "
          f"({rx},{ry},{rz}) yaw={yaw} ===")
    if not teleport_model(RIG_MODEL_NAME, rx, ry, rz):
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
    for seq, y in enumerate(y_grid):
        x, z = X0_M, TARGET_Z_M
        if not teleport_model(TARGET_MODEL_NAME, x, y, z):
            print(f"[multirange]   seq={seq} y={y}: target teleport FAILED -- skipping")
            n_skipped += 1
            continue
        settle_ok = (
            wait_new_frames(left_holder, TARGET_SETTLE_FRAMES)
            and wait_new_frames(right_holder, TARGET_SETTLE_FRAMES)
        )
        if not settle_ok:
            print(f"[multirange]   seq={seq} y={y}: settle timeout -- skipping")
            n_skipped += 1
            continue
        left_bgr, right_bgr = left_holder.bgr, right_holder.bgr
        if left_bgr is None or right_bgr is None:
            print(f"[multirange]   seq={seq} y={y}: no decoded frame yet -- skipping")
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
            "dash_direction": "static_grid",
            "dash_speed": "0.0",
        })
        print(f"[multirange]   seq={seq} y={y:+.1f}  true_range={r_true:.3f} m  captured OK")
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
        "dash_y0_mag_m": max(abs(y) for y in y_grid),
        "dash_z0_m": TARGET_Z_M,
        "dash_speed_m_s": 0.0,
        "rate_hz": 0.0,
        "y_grid_m": y_grid,
        "capture_mode": "static_multirange_grid",
        "signs_captured": ["static_grid"],
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None, help="output directory "
                     "(default logs/rig_captures/multirange_<UTC runstamp>/)")
    ap.add_argument("--standoffs", type=float, nargs="+", default=STANDOFFS_M)
    ap.add_argument("--y-grid", type=float, nargs="+", default=Y_GRID_M)
    args = ap.parse_args()

    runstamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_root = args.out or os.path.join(REPO_ROOT, "logs", "rig_captures", f"multirange_{runstamp}")
    os.makedirs(out_root, exist_ok=True)

    print(f"[multirange] world={WORLD_NAME} target={TARGET_MODEL_NAME} rig={RIG_MODEL_NAME}")
    print(f"[multirange] standoffs={args.standoffs}  y_grid={args.y_grid}")
    print(f"[multirange] out={out_root}")

    # ---- wedge-coverage pre-check, EVERY standoff, before touching the sim ----
    print("\n[multirange] === WEDGE COVERAGE CHECK (rig_geometry_analysis.point_in_wedge) ===")
    campaign_wedge = {}
    any_fail = False
    for standoff in args.standoffs:
        rig_pose, rows, all_ok = wedge_check_for_standoff(standoff, args.y_grid)
        campaign_wedge[standoff] = {"rig_pose_xyz_yaw": list(rig_pose), "rows": rows, "all_ok": all_ok}
        flag = "OK" if all_ok else "FAIL"
        print(f"[multirange] standoff={standoff:6.1f} m  rig_pose={rig_pose}  wedge={flag}")
        for r in rows:
            print(f"    y={r['y']:+.1f}  in_wedge={r['in_wedge']}  range={r['range_m']:.2f} m")
        if not all_ok:
            any_fail = True
    if any_fail:
        print("[multirange] FAIL: at least one (standoff, y) sample is outside the shared wedge "
              "-- fix the grid before capturing. Aborting, no sim interaction attempted.")
        return 2
    print("[multirange] wedge check: ALL standoffs/grid points inside the shared wedge. Proceeding.")

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
        rig_pose = campaign_wedge[standoff]["rig_pose_xyz_yaw"]
        res = capture_standoff(node, (left_holder, right_holder), standoff, tuple(rig_pose),
                                args.y_grid, out_root, git_rev, runstamp)
        if res is None:
            print(f"[multirange] ABORT: standoff={standoff} capture failed")
            node.unsubscribe(LEFT_TOPIC)
            node.unsubscribe(RIGHT_TOPIC)
            return 2
        standoff_results.append(res)

    node.unsubscribe(LEFT_TOPIC)
    node.unsubscribe(RIGHT_TOPIC)

    total_ok = sum(r["n_captured"] for r in standoff_results)
    total_skipped = sum(r["n_skipped"] for r in standoff_results)

    campaign_meta = {
        "world": WORLD_NAME,
        "target_model": TARGET_MODEL_NAME,
        "rig_model": RIG_MODEL_NAME,
        "standoffs_m": args.standoffs,
        "y_grid_m": args.y_grid,
        "rig_z_m": RIG_Z_M,
        "target_x0_m": X0_M,
        "target_z0_m": TARGET_Z_M,
        "rig_baseline_m": RIG_BASELINE_M,
        "rig_hfov_rad": RIG_HFOV_RAD,
        "resolution": [RIG_WIDTH, RIG_HEIGHT],
        "wedge_check": {str(k): v for k, v in campaign_wedge.items()},
        "standoff_dirs": standoff_results,
        "n_captured_total": total_ok,
        "n_skipped_total": total_skipped,
        "git_rev": git_rev,
        "runstamp": runstamp,
        "mechanism": "b_live_set_pose_single_boot",
    }
    campaign_meta_path = os.path.join(out_root, "campaign_meta.json")
    with open(campaign_meta_path, "w") as fh:
        json.dump(campaign_meta, fh, indent=2)

    print(f"\n[multirange] DONE: {total_ok} L/R pairs total ({total_skipped} skipped) -> {out_root}")
    print(f"[multirange] campaign meta: {campaign_meta_path}")
    return 0 if total_ok > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
