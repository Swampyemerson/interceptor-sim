#!/usr/bin/env python3
"""Auto-label onboard Gazebo frames into a YOLO single-class 'drone' dataset,
using logged ground-truth target pose to place the boxes. Feeds
scripts/seeker/train_drone_finetune.py's Stage-2 fine-tune (see that file's
module docstring, "THE DATA STRATEGY" -- this script IS that label-generator).

HONESTY BOUNDARY (CLAUDE.md / ADR-0010) -- READ BEFORE RUNNING
----------------------------------------------------------------------------
`gt_*` (ground truth) is used HERE, OFFLINE, ONLY to auto-generate training
labels for a dataset that trains a detector. This is dataset MANUFACTURE, not
a runtime guidance input -- exactly the distinction train_drone_finetune.py's
own docstring draws ("using gt_ to MANUFACTURE TRAINING LABELS offline is
legal; the trained detector still reads ONLY pixels at inference"). The
resulting .pt/.onnx weights, once trained, are read by
scripts/seeker/nn_seeker.py, which never imports gt_ or this file. This
script itself is never imported by any seeker/guidance module and never runs
in the control loop, so it does NOT touch the no-cheat audit (CLAUDE.md's
"Honesty boundary" governs the LIVE guidance path, not offline label
generation). If a new guidance path is ever built to consume weights trained
from this data, it re-earns that audit the same as any other seeker change.

WHAT THIS DOES
----------------------------------------------------------------------------
For each onboard frame, look up the target's ground-truth position at that
instant, project it through the FIXED camera intrinsics (pinhole model) into
a single 2D bounding box, and emit a YOLO-format label ("0 cx cy w h",
normalized). No pixel is ever inspected by this script -- only projected
geometry.

KNOWN APPROXIMATION -- m4_intercept.py's CSV logs no camera ORIENTATION
----------------------------------------------------------------------------
m4_intercept.py logs `gt_cam_x/y/z` (the CAMERA's own WORLD position) and
`gt_tag_x/y/z` (the TARGET's WORLD position) -- see
scripts/m3_static_intercept.py's ground_truth_world_points(). Neither is a
camera-frame vector: getting one needs the camera's WORLD ROTATION, which
scripts/m2_detect.py's PoseTracker computes internally
(compose() -> R_cam_world, see PoseTracker.ground_truth_rel_optical) but
m4_intercept.py's CSV writer discards -- only positions are logged (see its
CSV_HEADER). This script therefore reconstructs an APPROXIMATE camera-frame
vector using ONLY the vehicle's logged yaw (`psi_deg`, PX4's own EKF
heading) and assumes zero camera roll/pitch (a level, forward-facing
camera whose azimuth equals the vehicle yaw). This is the SAME
simplification m4_intercept.py's own guidance law already relies on for its
inertial LOS reconstruction (`lambda = psi + beta`, see that file's
"STRAPDOWN-SEEKER" docstring section) -- consistent with, not weaker than,
the existing project convention. It IS still an approximation: real vehicle
pitch/roll during a maneuvering intercept will skew the vertical (v)
placement, and to a lesser extent the horizontal (u) placement, of generated
boxes. A future exact fix is cheap (log
PoseTracker.ground_truth_rel_optical()'s vector, or attitude_euler(),
into new CSV columns) -- out of scope here, which only reads what already
exists. If a --poses CSV carries real camera-frame columns (schema 1 below),
this script uses those directly and skips the approximation entirely.

COLUMN SCHEMAS SUPPORTED (--poses CSV), tried in this priority order
----------------------------------------------------------------------------
  1. CAMERA-FRAME xyz (exact, preferred) -- any of these column trios,
     already in the camera OPTICAL frame (x right, y down, z forward,
     OpenCV convention, docs/goals.md):
       cam_x,cam_y,cam_z | rel_cam_x,rel_cam_y,rel_cam_z |
       target_cam_x,target_cam_y,target_cam_z | gt_cam_frame_x,_y,_z
     A range column (gt_range / gt_range_m / range_m) is used for box
     sizing if present, else the vector norm.
  2. RANGE + BEARING (exact, no yaw needed): a range column
     (gt_range/gt_range_m/range_m) + gt_bearing_rad (or gt_bearing_h_rad)
     [+ optional gt_bearing_vert_rad/gt_bearing_v_rad, else v-centered].
  3. m4_intercept.py's REAL columns (approximate, see above):
     gt_cam_x, gt_cam_y, gt_cam_z, gt_tag_x, gt_tag_y, gt_tag_z, psi_deg
     [+ gt_range if present, else recomputed from the reconstructed vector].

FRAME <-> POSE-ROW ALIGNMENT
----------------------------------------------------------------------------
m4_intercept.py's control loop and scripts/demo_capture_frames.py's frame
capture are two INDEPENDENT, asynchronously-clocked gz-transport
subscribers (see that file's "WHY A SEPARATE PROCESS PER CAMERA" docstring)
-- there is no guaranteed 1:1 row<->frame correspondence. This script
resolves the join, in priority order:
  (a) an explicit frame-index column in --poses (frame_idx / frame_index /
      frame / idx) -> exact join against the frame filename's parsed
      integer index. (No logger writes this today; supported for a future
      combined single-process logger.)
  (b) --align sim-time (default via --align auto): nearest-neighbour join
      on SIM time -- needs BOTH the frames dir's manifest.csv `sim_t`
      column (written by demo_capture_frames.py) AND the poses CSV's
      `t_sim` column populated (only non-blank when m4_intercept.py was
      run with --handoff, the only mode that subscribes to /clock -- see
      its CSV_HEADER comment). Per CLAUDE.md's "sim time, never wall time"
      rule, this is the only alignment this script trusts for a real,
      defensible dataset.
  (c) --align index: assume frame k already corresponds to poses-CSV row k
      (sorted order). ONLY valid if frames/rows were captured in lockstep
      (e.g. a future combined single-process logger); for today's
      two-process capture, frame rate and control-tick rate differ, so
      this is a documented, approximate DEBUG fallback -- prefer (a)/(b).
  `--align auto` (default) tries (a), then (b), then falls back to (c)
  ONLY if frame count exactly equals pose-row count (otherwise it refuses
  and asks you to pick explicitly, rather than silently mis-joining).

USAGE
----------------------------------------------------------------------------
  # validate without writing anything:
  .venv-seeker/bin/python scripts/seeker/gen_sim_dataset.py \\
      --frames demo_out/onboard_frames \\
      --poses logs/m4_intercept_pronav_<timestamp>Z.csv \\
      --out scripts/seeker/data/sim_dataset --dry-run

  # then drop --dry-run to actually write the dataset, and feed the trainer:
  .venv-seeker-train/bin/python scripts/seeker/train_drone_finetune.py \\
      --go --data scripts/seeker/data/sim_dataset/data.yaml
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import random
import re
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from classical_seeker import load_intrinsics  # noqa: E402  (repo convention, see two_stage_seeker.py)

DEFAULT_OUT = os.path.join(HERE, "data", "sim_dataset")
DEFAULT_INTRINSICS = os.path.join(REPO, "configs/camera_intrinsics.json")

# Physical target extent (m) used for bbox sizing -- deliberately independent
# of scripts/seeker/nn_seeker.py's TARGET_SPAN_M=1.0 (that constant covers
# the full prop-arm span for a bearing/range ESTIMATE from an apparent-size
# measurement; --target-size-m here sizes an auto-labeled TRAINING box from
# known geometry, and a tighter body-only figure is a defensible default for
# what we want the detector to learn to bound). Override per --help.
DEFAULT_TARGET_SIZE_M = 0.35

CAMERA_XYZ_ALIASES: Tuple[Tuple[str, str, str], ...] = (
    ("cam_x", "cam_y", "cam_z"),
    ("rel_cam_x", "rel_cam_y", "rel_cam_z"),
    ("target_cam_x", "target_cam_y", "target_cam_z"),
    ("gt_cam_frame_x", "gt_cam_frame_y", "gt_cam_frame_z"),
)
RANGE_ALIASES: Tuple[str, ...] = ("gt_range", "gt_range_m", "range_m")
BEARING_H_ALIASES: Tuple[str, ...] = ("gt_bearing_rad", "gt_bearing_h_rad")
BEARING_V_ALIASES: Tuple[str, ...] = ("gt_bearing_vert_rad", "gt_bearing_v_rad")
FRAME_IDX_ALIASES: Tuple[str, ...] = ("frame_idx", "frame_index", "frame", "idx")
# m4_intercept.py's real CSV_HEADER columns (schema 3, the approximation path).
M4_WORLD_COLS: Tuple[str, ...] = (
    "gt_cam_x", "gt_cam_y", "gt_cam_z", "gt_tag_x", "gt_tag_y", "gt_tag_z",
)

FRAME_EXTS = (".png", ".jpg", ".jpeg")

DATA_YAML_TEMPLATE = """\
# Single-class {cls} detector -- auto-generated by gen_sim_dataset.py from
# in-domain Gazebo renders + logged ground-truth target pose (label-gen
# only, see this script's module docstring's honesty-boundary section: the
# trained detector reads pixels only at inference).
path: {root}          # dataset root
train: images/train   # relative to path
val:   images/val
names:
  0: {cls}
"""


# ===========================================================================
#  Small helpers
# ===========================================================================
def _pf(s) -> Optional[float]:
    """Parse a CSV cell to float, treating blank/missing as None (m4's
    logger writes "" for any tick where ground truth hadn't populated yet
    -- see m3_static_intercept.write_row_m4's `fmt`)."""
    if s is None:
        return None
    if isinstance(s, str):
        s = s.strip()
        if s == "":
            return None
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _load_resolution(intrinsics_path: str) -> Tuple[int, int]:
    d = json.loads(Path(intrinsics_path).read_text())
    res = d.get("resolution", {})
    return int(res.get("width", 1280)), int(res.get("height", 960))


def _parse_frame_index(filename: str) -> Optional[int]:
    m = re.search(r"(\d+)", filename)
    return int(m.group(1)) if m else None


# ===========================================================================
#  Frames + manifest + poses I/O
# ===========================================================================
@dataclass
class FrameEntry:
    idx: int
    path: str
    filename: str


def list_frames(frames_dir: str) -> List[FrameEntry]:
    if not os.path.isdir(frames_dir):
        raise FileNotFoundError(f"--frames dir not found: {frames_dir}")
    out: List[FrameEntry] = []
    for name in sorted(os.listdir(frames_dir)):
        if not name.lower().endswith(FRAME_EXTS):
            continue
        idx = _parse_frame_index(name)
        if idx is None:
            continue
        out.append(FrameEntry(idx, os.path.join(frames_dir, name), name))
    out.sort(key=lambda f: f.idx)
    return out


def load_manifest(frames_dir: str, manifest_path: Optional[str]) -> Optional[Dict[int, float]]:
    """idx -> sim_t from demo_capture_frames.py's manifest.csv. Returns None
    if the manifest is missing or has no usable (non-blank) sim_t at all."""
    path = manifest_path or os.path.join(frames_dir, "manifest.csv")
    if not os.path.exists(path):
        return None
    out: Dict[int, float] = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            idx_val = _pf(row.get("idx"))
            sim_t = _pf(row.get("sim_t"))
            if idx_val is None or sim_t is None:
                continue
            out[int(round(idx_val))] = sim_t
    return out or None


def read_poses(path: str) -> Tuple[List[str], List[Dict[str, str]]]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"--poses CSV not found: {path}")
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fields = reader.fieldnames or []
    return fields, rows


# ===========================================================================
#  Ground-truth column schema resolution
# ===========================================================================
@dataclass
class ColumnSchema:
    mode: str                 # "camera_xyz" | "range_bearing" | "world_yaw"
    cols: Dict[str, str] = field(default_factory=dict)   # role -> actual column name


def resolve_schema(fieldnames: List[str]) -> ColumnSchema:
    fs = set(fieldnames)

    for trio in CAMERA_XYZ_ALIASES:
        if all(c in fs for c in trio):
            cols = {"x": trio[0], "y": trio[1], "z": trio[2]}
            r = next((c for c in RANGE_ALIASES if c in fs), None)
            if r:
                cols["range"] = r
            return ColumnSchema("camera_xyz", cols)

    range_col = next((c for c in RANGE_ALIASES if c in fs), None)
    bearing_col = next((c for c in BEARING_H_ALIASES if c in fs), None)
    if range_col and bearing_col:
        cols = {"range": range_col, "bearing": bearing_col}
        vcol = next((c for c in BEARING_V_ALIASES if c in fs), None)
        if vcol:
            cols["bearing_vert"] = vcol
        return ColumnSchema("range_bearing", cols)

    if all(c in fs for c in M4_WORLD_COLS) and "psi_deg" in fs:
        cols = {
            "cam_x": "gt_cam_x", "cam_y": "gt_cam_y", "cam_z": "gt_cam_z",
            "tag_x": "gt_tag_x", "tag_y": "gt_tag_y", "tag_z": "gt_tag_z",
            "psi_deg": "psi_deg",
        }
        if "gt_range" in fs:
            cols["range"] = "gt_range"
        return ColumnSchema("world_yaw", cols)

    raise ValueError(
        "--poses CSV has none of the supported gt-pose column schemas.\n"
        f"  header seen: {sorted(fieldnames)}\n"
        "  supported, in priority order (see this script's module docstring):\n"
        f"    1. camera-frame xyz trio, one of {CAMERA_XYZ_ALIASES}\n"
        f"    2. range + bearing: one of {RANGE_ALIASES} + one of {BEARING_H_ALIASES}\n"
        f"    3. m4_intercept.py's own columns: {M4_WORLD_COLS} + psi_deg"
    )


def target_cam_xyz(row: Dict[str, str], schema: ColumnSchema) -> Optional[Tuple[float, float, float, float]]:
    """Return (x_right, y_down, z_forward, range_m) in the camera OPTICAL
    frame for one poses-CSV row, or None if this row's gt is incomplete
    (e.g. the pose topic hadn't populated all entities yet this tick)."""
    if schema.mode == "camera_xyz":
        x = _pf(row.get(schema.cols["x"]))
        y = _pf(row.get(schema.cols["y"]))
        z = _pf(row.get(schema.cols["z"]))
        if x is None or y is None or z is None:
            return None
        range_m = _pf(row.get(schema.cols["range"])) if "range" in schema.cols else None
        if range_m is None:
            range_m = math.sqrt(x * x + y * y + z * z)
        return x, y, z, range_m

    if schema.mode == "range_bearing":
        r = _pf(row.get(schema.cols["range"]))
        b = _pf(row.get(schema.cols["bearing"]))
        if r is None or b is None:
            return None
        bv = 0.0
        if "bearing_vert" in schema.cols:
            bv_val = _pf(row.get(schema.cols["bearing_vert"]))
            bv = bv_val if bv_val is not None else 0.0
        # OpenCV optical convention: x right, y down, z forward;
        # b>0 = target right of boresight, bv>0 = target below boresight.
        z = r * math.cos(b) * math.cos(bv)
        x = r * math.sin(b) * math.cos(bv)
        y = r * math.sin(bv)
        return x, y, z, r

    if schema.mode == "world_yaw":
        cam_vals = [_pf(row.get(schema.cols[k])) for k in ("cam_x", "cam_y", "cam_z")]
        tag_vals = [_pf(row.get(schema.cols[k])) for k in ("tag_x", "tag_y", "tag_z")]
        psi_deg = _pf(row.get(schema.cols["psi_deg"]))
        if any(v is None for v in cam_vals) or any(v is None for v in tag_vals) or psi_deg is None:
            return None
        d_east = tag_vals[0] - cam_vals[0]
        d_north = tag_vals[1] - cam_vals[1]
        d_up = tag_vals[2] - cam_vals[2]
        psi = math.radians(psi_deg)
        # YAW-ONLY camera-azimuth approximation (module docstring "KNOWN
        # APPROXIMATION"): assumes zero camera roll/pitch, camera azimuth
        # == vehicle yaw. Same simplification as m4_intercept.py's own
        # lambda = psi + beta LOS reconstruction.
        x = d_east * math.cos(psi) - d_north * math.sin(psi)   # right
        z = d_east * math.sin(psi) + d_north * math.cos(psi)   # forward
        y = -d_up                                              # down
        range_m = _pf(row.get(schema.cols["range"])) if "range" in schema.cols else None
        if range_m is None:
            range_m = math.sqrt(x * x + y * y + z * z)
        return x, y, z, range_m

    raise AssertionError(f"unknown schema mode {schema.mode!r}")


# ===========================================================================
#  Pinhole projection: gt camera-frame point -> pixel bbox
# ===========================================================================
def project_to_bbox(
    x: float, y: float, z: float, range_m: float,
    fx: float, fy: float, cx: float, cy: float,
    target_size_m: float, width: int, height: int,
) -> Optional[Tuple[float, float, float, float]]:
    """Pinhole-project a camera-optical-frame point (x right, y down, z
    forward) into a pixel-space bbox (cx_px, cy_px, w_px, h_px), sized from
    the known physical target extent at the known range -- REQUIREMENTS #2.
    Returns None to SKIP this frame: target behind the camera, or the
    projected box lands fully outside the image."""
    if z <= 1e-3 or range_m <= 1e-3:
        return None  # behind (or coincident with) the camera -- undefined
    u = cx + fx * x / z
    v = cy + fy * y / z
    half_w = fx * (target_size_m / 2.0) / range_m
    half_h = fy * (target_size_m / 2.0) / range_m
    left, right = u - half_w, u + half_w
    top, bottom = v - half_h, v + half_h
    if right <= 0.0 or left >= width or bottom <= 0.0 or top >= height:
        return None  # fully outside the frame
    left = max(0.0, left)
    right = min(float(width), right)
    top = max(0.0, top)
    bottom = min(float(height), bottom)
    w_px = right - left
    h_px = bottom - top
    if w_px <= 1.0 or h_px <= 1.0:
        return None  # degenerate sliver after clamping
    return (left + right) / 2.0, (top + bottom) / 2.0, w_px, h_px


# ===========================================================================
#  Frame <-> pose-row alignment
# ===========================================================================
@dataclass
class AlignedFrame:
    frame: FrameEntry
    row: Dict[str, str]


def align_by_frame_index_column(
    frames: List[FrameEntry], pose_rows: List[Dict[str, str]], idx_col: str,
) -> Tuple[List[AlignedFrame], int]:
    by_idx: Dict[int, Dict[str, str]] = {}
    for row in pose_rows:
        v = _pf(row.get(idx_col))
        if v is not None:
            by_idx[int(round(v))] = row
    out: List[AlignedFrame] = []
    misses = 0
    for f in frames:
        row = by_idx.get(f.idx)
        if row is None:
            misses += 1
            continue
        out.append(AlignedFrame(f, row))
    return out, misses


def align_by_sim_time(
    frames: List[FrameEntry], manifest: Dict[int, float],
    pose_rows: List[Dict[str, str]], t_sim_col: str, max_gap_s: float,
) -> Tuple[List[AlignedFrame], int]:
    timed = sorted(
        ((t, row) for row, t in ((r, _pf(r.get(t_sim_col))) for r in pose_rows) if t is not None),
        key=lambda p: p[0],
    )
    if not timed:
        return [], len(frames)
    times = [t for t, _ in timed]
    out: List[AlignedFrame] = []
    misses = 0
    for f in frames:
        ft = manifest.get(f.idx)
        if ft is None:
            misses += 1
            continue
        j = bisect.bisect_left(times, ft)
        best_gap: Optional[float] = None
        best_k = -1
        for k in (j - 1, j):
            if 0 <= k < len(times):
                gap = abs(times[k] - ft)
                if best_gap is None or gap < best_gap:
                    best_gap, best_k = gap, k
        if best_gap is None or best_gap > max_gap_s:
            misses += 1
            continue
        out.append(AlignedFrame(f, timed[best_k][1]))
    return out, misses


def align_by_index(
    frames: List[FrameEntry], pose_rows: List[Dict[str, str]],
) -> Tuple[List[AlignedFrame], int]:
    n = min(len(frames), len(pose_rows))
    out = [AlignedFrame(frames[i], pose_rows[i]) for i in range(n)]
    return out, len(frames) - n


def align(
    frames: List[FrameEntry], frames_dir: str, manifest_path: Optional[str],
    pose_fields: List[str], pose_rows: List[Dict[str, str]],
    align_mode: str, max_gap_s: float,
) -> Tuple[List[AlignedFrame], int, str]:
    idx_col = next((c for c in FRAME_IDX_ALIASES if c in pose_fields), None)

    if align_mode == "auto":
        if idx_col:
            aligned, misses = align_by_frame_index_column(frames, pose_rows, idx_col)
            return aligned, misses, f"frame-index column '{idx_col}'"
        manifest = load_manifest(frames_dir, manifest_path)
        if manifest and "t_sim" in pose_fields:
            aligned, misses = align_by_sim_time(frames, manifest, pose_rows, "t_sim", max_gap_s)
            if aligned:
                return aligned, misses, "sim-time (auto)"
        if len(frames) == len(pose_rows):
            aligned, misses = align_by_index(frames, pose_rows)
            return aligned, misses, "row-index (auto fallback, frame/row counts matched)"
        raise ValueError(
            "auto-alignment failed: --poses has no frame-index column, no "
            "usable sim-time join (need the frames dir's manifest.csv "
            "'sim_t' AND the poses CSV's 't_sim' both populated -- 't_sim' "
            "is blank unless m4_intercept.py was run with --handoff), and "
            f"frame count ({len(frames)}) != poses row count "
            f"({len(pose_rows)}) so an index fallback isn't safe either. "
            "Pick --align explicitly, or re-capture with --handoff so "
            "t_sim populates (see this script's module docstring)."
        )

    if align_mode == "sim-time":
        manifest = load_manifest(frames_dir, manifest_path)
        if manifest is None:
            raise ValueError(
                f"--align sim-time needs a manifest.csv (idx,wall_t,sim_t,file) "
                f"in {frames_dir} (or --manifest PATH) with non-blank sim_t -- "
                "see scripts/demo_capture_frames.py."
            )
        if "t_sim" not in pose_fields:
            raise ValueError(
                "--align sim-time needs a 't_sim' column in --poses; none "
                "found. m4_intercept.py only populates it when run with "
                "--handoff (the only mode that subscribes to /clock)."
            )
        aligned, misses = align_by_sim_time(frames, manifest, pose_rows, "t_sim", max_gap_s)
        return aligned, misses, "sim-time"

    if align_mode == "index":
        aligned, misses = align_by_index(frames, pose_rows)
        return aligned, misses, "row-index (--align index, approximate -- see module docstring)"

    raise AssertionError(f"unknown --align mode {align_mode!r}")


# ===========================================================================
#  Deterministic train/val split (no wall-clock/random-seed nondeterminism)
# ===========================================================================
def deterministic_split(
    aligned: List[AlignedFrame], val_frac: float, seed: int,
) -> Tuple[List[AlignedFrame], List[AlignedFrame]]:
    items = sorted(aligned, key=lambda a: a.frame.idx)  # stable order before shuffling
    order = list(range(len(items)))
    random.Random(seed).shuffle(order)
    n_val = int(round(len(items) * val_frac))
    val_positions = set(order[:n_val])
    train, val = [], []
    for i, item in enumerate(items):
        (val if i in val_positions else train).append(item)
    return train, val


# ===========================================================================
#  Label / dataset writing
# ===========================================================================
def make_label_line(box: Tuple[float, float, float, float], width: int, height: int, class_id: int) -> str:
    cx_px, cy_px, w_px, h_px = box
    return (
        f"{class_id} {cx_px / width:.6f} {cy_px / height:.6f} "
        f"{w_px / width:.6f} {h_px / height:.6f}"
    )


def write_split(
    items: List[AlignedFrame], split_name: str, out_dir: str, schema: ColumnSchema,
    fx: float, fy: float, cx: float, cy: float, width: int, height: int,
    target_size_m: float, class_id: int, dry_run: bool,
) -> Tuple[int, int, int]:
    img_dir = os.path.join(out_dir, "images", split_name)
    lbl_dir = os.path.join(out_dir, "labels", split_name)
    if not dry_run:
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)

    n_written = n_skipped_gt = n_skipped_proj = 0
    for item in items:
        cam_xyz = target_cam_xyz(item.row, schema)
        if cam_xyz is None:
            n_skipped_gt += 1
            continue
        x, y, z, range_m = cam_xyz
        box = project_to_bbox(x, y, z, range_m, fx, fy, cx, cy, target_size_m, width, height)
        if box is None:
            n_skipped_proj += 1
            continue
        n_written += 1
        if dry_run:
            continue
        stem, _ = os.path.splitext(item.frame.filename)
        shutil.copy2(item.frame.path, os.path.join(img_dir, item.frame.filename))
        with open(os.path.join(lbl_dir, stem + ".txt"), "w") as lf:
            lf.write(make_label_line(box, width, height, class_id) + "\n")
    return n_written, n_skipped_gt, n_skipped_proj


def write_data_yaml(out_dir: str, class_name: str) -> str:
    path = os.path.join(out_dir, "data.yaml")
    os.makedirs(out_dir, exist_ok=True)
    with open(path, "w") as f:
        f.write(DATA_YAML_TEMPLATE.format(root=os.path.abspath(out_dir), cls=class_name))
    return path


# ===========================================================================
#  CLI
# ===========================================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--frames", required=True,
                     help="dir of onboard frames (frame_NNNNNN.png/.jpg), e.g. demo_out/onboard_frames "
                          "-- an optional manifest.csv (idx,wall_t,sim_t,file) alongside enables sim-time alignment")
    ap.add_argument("--poses", required=True,
                     help="per-frame ground-truth CSV (an m4_intercept.py flight log, or compatible -- "
                          "see module docstring's COLUMN SCHEMAS SUPPORTED)")
    ap.add_argument("--out", default=DEFAULT_OUT,
                     help=f"dataset root to write (default {DEFAULT_OUT})")
    ap.add_argument("--target-size-m", type=float, default=DEFAULT_TARGET_SIZE_M,
                     help=f"physical target extent (m) used for bbox sizing (default {DEFAULT_TARGET_SIZE_M})")
    ap.add_argument("--val-frac", type=float, default=0.2,
                     help="fraction of labeled frames held out for validation (default 0.2)")
    ap.add_argument("--seed", type=int, default=0,
                     help="deterministic train/val split seed (default 0; no wall-clock/random-seed nondeterminism)")
    ap.add_argument("--align", choices=["auto", "sim-time", "index"], default="auto",
                     help="frame<->pose-row join strategy (default auto -- see module docstring "
                          "'FRAME <-> POSE-ROW ALIGNMENT')")
    ap.add_argument("--manifest", default=None,
                     help="override path to the frames manifest.csv (default <frames>/manifest.csv)")
    ap.add_argument("--max-time-gap-s", type=float, default=0.2,
                     help="max |dt_sim| accepted for a sim-time join (default 0.2s)")
    ap.add_argument("--intrinsics", default=DEFAULT_INTRINSICS,
                     help=f"configs/camera_intrinsics.json path (default {DEFAULT_INTRINSICS})")
    ap.add_argument("--class-name", default="drone",
                     help="YOLO class-0 name written to data.yaml (default drone)")
    ap.add_argument("--dry-run", action="store_true",
                     help="report how many frames would be labeled/skipped; write nothing")
    args = ap.parse_args()

    fx, fy, cx, cy = load_intrinsics(args.intrinsics)
    width, height = _load_resolution(args.intrinsics)

    frames = list_frames(args.frames)
    if not frames:
        print(f"[gen_sim_dataset] no frame images found in {args.frames}", file=sys.stderr)
        return 1

    pose_fields, pose_rows = read_poses(args.poses)
    if not pose_rows:
        print(f"[gen_sim_dataset] no rows in {args.poses}", file=sys.stderr)
        return 1

    try:
        schema = resolve_schema(pose_fields)
    except ValueError as exc:
        print(f"[gen_sim_dataset] {exc}", file=sys.stderr)
        return 1

    try:
        aligned, misses, method = align(
            frames, args.frames, args.manifest, pose_fields, pose_rows,
            args.align, args.max_time_gap_s,
        )
    except ValueError as exc:
        print(f"[gen_sim_dataset] {exc}", file=sys.stderr)
        return 1

    print("=" * 72)
    print("SIM DATASET LABEL GENERATION (offline; gt used ONLY to manufacture training labels)")
    print("=" * 72)
    print(f"frames dir     : {args.frames}  ({len(frames)} images)")
    print(f"poses CSV      : {args.poses}  ({len(pose_rows)} rows)")
    print(f"intrinsics     : {args.intrinsics}  ({width}x{height}, fx={fx:.3f} fy={fy:.3f} cx={cx:.1f} cy={cy:.1f})")
    print(f"gt column mode : {schema.mode}  cols={schema.cols}")
    print(f"alignment      : {method}  matched={len(aligned)}/{len(frames)}  unmatched={misses}")
    if schema.mode == "world_yaw":
        print("NOTE: using the yaw-only camera-orientation APPROXIMATION (m4_intercept.py "
              "logs no camera attitude) -- see this script's module docstring 'KNOWN APPROXIMATION'.")

    train, val = deterministic_split(aligned, args.val_frac, args.seed)
    print(f"split          : {len(train)} train / {len(val)} val "
          f"(val_frac={args.val_frac}, seed={args.seed}, deterministic)")

    class_id = 0
    tw, tg, tp = write_split(
        train, "train", args.out, schema, fx, fy, cx, cy, width, height,
        args.target_size_m, class_id, args.dry_run,
    )
    vw, vg, vp = write_split(
        val, "val", args.out, schema, fx, fy, cx, cy, width, height,
        args.target_size_m, class_id, args.dry_run,
    )

    n_labeled = tw + vw
    n_skip_gt = tg + vg
    n_skip_proj = tp + vp
    print("-" * 72)
    print(f"train: {tw} labeled, {tg} skipped (missing/incomplete gt), {tp} skipped (behind camera / fully off-frame)")
    print(f"val  : {vw} labeled, {vg} skipped (missing/incomplete gt), {vp} skipped (behind camera / fully off-frame)")
    print(f"TOTAL: {n_labeled} labeled / {len(aligned)} aligned / {len(frames)} frames "
          f"({n_skip_gt} gt-missing, {n_skip_proj} off-frame, {misses} unaligned)")

    if args.dry_run:
        print("\n[dry-run] nothing written.")
        return 0

    yaml_path = write_data_yaml(args.out, args.class_name)
    print(f"\nwrote {n_labeled} images+labels under {args.out}")
    print(f"wrote {yaml_path}")
    print(f"next: train_drone_finetune.py --go --data {yaml_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
