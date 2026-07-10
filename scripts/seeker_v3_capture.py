#!/usr/bin/env python3
r"""Seeker v3 dataset capture — Phase-1 static teleport grid for the ONBOARD
drone detector (docs/seeker_v3_dataset_plan.md §3, Phase 1), plus the
Phase-2/eval flight-plan table printer.

WHY (one paragraph): drone_finetuned_v2 was trained on a 2-12 m static grid
with target yaw 0/±25° plus 3 near-straight-line 9 m/s capture flights
(ADR-0042). At 12 m/s weave/jink it throws large, HIGH-confidence interior
phantoms (~18 m off-target; 12/16 false handoffs) and barely detects the
real fast-crossing target (195/217 ENGAGE detections false — ADR-0056/0057).
This script captures the systematic pose coverage v2 never had: full aspect
ring, extended ranges out to 50 m (the real target sits 15-100 m away at the
false-handoff moment, ADR-0057), bearings inside the deployed ±30° self-mask,
and a small empty-frame negative phase. The domain-matched maneuvering
frames come from Phase-2 ride-along flights (capture_flight_frames.py, not
this script) — `--print-flight-plan` emits that arm table so plan and
tooling cannot drift apart.

STRUCTURE is modeled on scripts/multirange_capture.py:
  * gz-transport camera subscription held by THIS process (FrameHolder),
  * teleports via a `gz service` CLI SUBPROCESS ("the mover pattern": a
    process holding subscriptions never receives service RESPONSES —
    gz-transport13 quirk documented in multirange_capture.py /
    m4_target_mover.py),
  * settle-by-new-frames between poses,
  * one boot, index.csv + capture_meta.json outputs, pre-flight matrix
    check BEFORE touching the sim.
Ground truth comes from m2_detect.PoseTracker (the M2-validated transform
chain), same as render_sim_dataset.py / capture_flight_frames.py.

LABEL HONESTY (CLAUDE.md / ADR-0010, same boundary as every dataset
producer in scripts/seeker/): gt_* is used HERE, OFFLINE, only to
manufacture training labels. The trained detector reads pixels only at
inference; this script never runs in the control loop.

OFFLINE-SAFE REVIEW: `--dry-run` prints the full capture matrix (pose
table, projected box sizes, FOV checks, counts) and exits WITHOUT importing
gz/cv2 or touching any simulator. `--self-test` exercises the local
projection/classification math against synthetic cases. `--print-flight-plan`
prints the Phase-2 training-capture and eval-pool arm tables from the plan
doc. Only a bare invocation (no mode flag) attempts a live capture, and it
fails loudly if the sim is not up.

RUN (live capture; sim ALREADY up on the markerless world — see
probe_markerless_range.py's docstring for the boot line; ONE sim at a time):
    INTERCEPTOR_WORLD_NAME=markerless INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \
      .venv-seeker/bin/python scripts/seeker_v3_capture.py \
      --out scripts/seeker/data/v3_grid

Offline (no sim, safe anywhere):
    python3 scripts/seeker_v3_capture.py --dry-run
    python3 scripts/seeker_v3_capture.py --self-test
    python3 scripts/seeker_v3_capture.py --print-flight-plan
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

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)

# ---------------------------------------------------------------------------
# Fixed camera model — matches camera_intrinsics.json (recorded 2026-07-04
# from the live camera_info topic) and the mono_cam SDF (1280x960, HFOV
# 1.74 rad, 30 Hz, NO motion-blur/exposure model — plan doc §2).
# Same constants render_sim_dataset.py / capture_flight_frames.py hard-code.
# ---------------------------------------------------------------------------
FX = FY = 539.936
CX, CY = 640.0, 480.0
IMG_W, IMG_H = 1280, 960
HFOV_RAD = 1.74

# Target silhouette extent for the projected bbox — the same 0.9 m default
# render_sim_dataset.py / capture_flight_frames.py use (--extent-m).
DEFAULT_EXTENT_M = 0.9
# Label floor: both projected box dims must be >= 4 px, matching
# capture_flight_frames.MIN_BOX_PX (smaller = too ambiguous to train on).
MIN_BOX_PX = 4.0

# ---------------------------------------------------------------------------
# THE PHASE-1 CAPTURE MATRIX (plan doc §3 Phase 1 — every value cited there)
# ---------------------------------------------------------------------------
# Ranges: v1 grid was 2-12 m (render_sim_dataset.py --ranges default);
# extension to 50 m per ADR-0057 (true target 15-100 m away at the
# false-handoff moment, currently an unsupervised ~8 px low-conf blob).
RANGES_M = [1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 18.0, 25.0, 35.0, 50.0]
# Ranges with the FULL height axis; beyond these, height is pinned to the
# deployment corridor 0.5 m (±1 m at 50 m is only ~11 px of elevation —
# plan §3 volume trim).
FULL_HEIGHT_MAX_RANGE_M = 18.0
# Full aspect ring (v1 was 0/±25°). In flight the mover is POSITION-only
# (m4_target_mover.py, ADR-0010 #6 — orientation never set), so deployed
# aspect variation comes from viewing geometry; this yaw ring is the static
# proxy for it.
YAWS_DEG = [0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0]
# Bearings inside the deployed self-mask (finetuned_seeker.py
# max_bearing_deg=30): lateral offset = range * tan(bearing).
BEARINGS_DEG = [-25.0, 0.0, 25.0]
# Heights: 0.5 m is the deployment corridor (m4_target_mover DEFAULT_START
# z, mc_batch z=0.5); 0.25/1.5 add modest vertical variety at close range.
HEIGHTS_M = [0.25, 0.5, 1.5]
CORRIDOR_Z_M = 0.5

# Held-out grid ranges (plan §6): entire range buckets to val, the
# build_ground_dataset_v2.py structural-split pattern (ADR-0055). Capture
# does not act on this — it only RECORDS the bucket in index.csv; the split
# decision lives in the dataset builder. Listed here so the two files agree.
HELD_OUT_RANGES_M = [3.0, 18.0]

# Negative phase (small; the real hard negatives are Phase-2 ride-along
# frames + offline phantom mining — plan §4): park the target far behind
# the camera and grab a few empty frames for volume/regularization.
NEG_TARGET_POSE = (-60.0, 0.0, 0.5)   # world XYZ, far behind the camera
NEG_N_FRAMES = 30

# Settle: wait for N NEW frames after each teleport before grabbing. Set to
# 8 (up from render_sim_dataset.py's 6) to match probe_markerless_range.py's
# proven long-teleport default and give margin for the 35/50 m hops [A4].
# Frame-COUNTED (not wall-timed), so RTF sag cannot shorten it; set_pose is
# atomic server-side, so the render reflects the new pose within 1-2 ticks.
# The live posecheck below VERIFIES this at a 50 m boresight hop before the
# batch, and the per-cell live-classify guard skips any stale render.
SETTLE_FRAMES = 8
TELEPORT_TIMEOUT_S = 4.0
SETTLE_TIMEOUT_S = 12.0
STARTUP_TIMEOUT_S = 20.0

# FOV margin: reject any pose whose projected box center would fall within
# this many px of the frame edge (we want clean interior positives from the
# grid; edge/periphery appearance is Phase-2's job).
EDGE_MARGIN_PX = 8.0


# ===========================================================================
# Pure math (offline-safe: no gz, no cv2) — dry-run/self-test path
# ===========================================================================
def project_box(rel_optical, extent_m: float = DEFAULT_EXTENT_M,
                W: int = IMG_W, H: int = IMG_H, min_px: float = MIN_BOX_PX):
    """Classify one camera-optical-frame target vector (x right, y down,
    z forward — GOALS.md OpenCV convention) into
        ("positive", (cx, cy, w, h) normalized) | ("negative", None) |
        ("drop", None).
    Same math and three-way semantics as capture_flight_frames.classify(),
    reimplemented locally (documented copy, not a fork of behavior) so that
    --dry-run/--self-test work with zero sim-stack imports. If the two ever
    need to diverge, diverge THERE and re-copy — this one must stay
    byte-equivalent in behavior."""
    x, y, z = float(rel_optical[0]), float(rel_optical[1]), float(rel_optical[2])
    if z <= 0.05:
        return "negative", None  # behind the camera
    u = CX + FX * (x / z)
    v = CY + FY * (y / z)
    half_w = FX * (extent_m / 2.0) / z
    half_h = FY * (extent_m / 2.0) / z
    x0, x1 = u - half_w, u + half_w
    y0, y1 = v - half_h, v + half_h
    if x1 < 0 or x0 > W or y1 < 0 or y0 > H:
        return "negative", None  # fully outside the frame
    cx0, cx1 = max(0.0, x0), min(float(W), x1)
    cy0, cy1 = max(0.0, y0), min(float(H), y1)
    bw, bh = cx1 - cx0, cy1 - cy0
    if bw < min_px or bh < min_px:
        return "drop", None
    bcx, bcy = (cx0 + cx1) / 2.0, (cy0 + cy1) / 2.0
    return "positive", (bcx / W, bcy / H, bw / W, bh / H)


def build_grid_poses(ranges=None, yaws_deg=None, bearings_deg=None,
                     heights_m=None,
                     full_height_max_range=FULL_HEIGHT_MAX_RANGE_M):
    """Build the Phase-1 pose list: (label, x, y, z, yaw_rad, range_m,
    bearing_deg, height_m, yaw_deg, held_out).

    Geometry: interceptor parked at world origin, camera boresight = world
    +X (the render_sim_dataset.py v1 grid geometry — TODO(sim): confirm
    with a --posecheck first frame before the batch, plan assumption [A8]).
    Target world pose for (range r, bearing b): x = r, y = r*tan(b)
    (matches v1's forward-range + lateral convention; bearing here is the
    HORIZONTAL bearing the deployed seeker computes, atan2((u-cx)/fx, 1)).
    """
    ranges = RANGES_M if ranges is None else ranges
    yaws_deg = YAWS_DEG if yaws_deg is None else yaws_deg
    bearings_deg = BEARINGS_DEG if bearings_deg is None else bearings_deg
    heights_m = HEIGHTS_M if heights_m is None else heights_m

    poses = []
    for r in ranges:
        hs = heights_m if r <= full_height_max_range else [CORRIDOR_Z_M]
        for b in bearings_deg:
            lat = r * math.tan(math.radians(b))
            for h in hs:
                for yaw in yaws_deg:
                    label = (f"grid_r{r:05.1f}_b{b:+05.1f}_h{h:.2f}"
                             f"_yaw{yaw:+06.1f}")
                    poses.append({
                        "label": label,
                        "x": round(r, 4), "y": round(lat, 4), "z": h,
                        "yaw_rad": math.radians(yaw),
                        "range_m": r, "bearing_deg": b,
                        "height_m": h, "yaw_deg": yaw,
                        "held_out_range": r in HELD_OUT_RANGES_M,
                    })
    return poses


def precheck_pose(pose, extent_m=DEFAULT_EXTENT_M):
    """Offline feasibility check for one grid pose (the multirange_capture
    wedge-check analogue, run BEFORE touching the sim): predict the target's
    camera-frame vector from the parked-interceptor geometry and require a
    'positive' classification with an interior box center.

    APPROXIMATION (fine for a feasibility gate, exact labels come from the
    live PoseTracker at capture): camera at the parked hub height, level,
    boresight +X. TODO(sim): replace CAM_Z with the measured parked camera
    height from the first live gt sample (v1's grid worked with target z
    0-0.5 from the parked vehicle, so 0.0-0.3 m error here is not
    load-bearing).
    """
    CAM_Z = 0.24  # TODO(sim): parked x500 camera height — VERIFY live [A8]
    x_fwd = pose["x"]           # optical z (forward)
    y_right = pose["y"]         # optical x (right): world +Y is right when
    #                             boresight is +X and camera is level.
    #                             TODO(sim): confirm sign with one live
    #                             posecheck frame (a wrong sign flips
    #                             bearings symmetrically; the grid is
    #                             symmetric in bearing so capture would
    #                             still be valid, but labels/meta must
    #                             record the true convention).
    z_down = -(pose["z"] - CAM_Z)  # optical y (down)
    kind, box = project_box((y_right, z_down, x_fwd), extent_m)
    ok = kind == "positive"
    interior = False
    bw_px = bh_px = 0.0
    if box is not None:
        bcx, bcy, bw, bh = box
        bw_px, bh_px = bw * IMG_W, bh * IMG_H
        interior = (EDGE_MARGIN_PX < bcx * IMG_W < IMG_W - EDGE_MARGIN_PX
                    and EDGE_MARGIN_PX < bcy * IMG_H < IMG_H - EDGE_MARGIN_PX)
    return ok and interior, kind, (bw_px, bh_px)


# ===========================================================================
# Phase-2 / eval flight-plan tables (plan doc §3 Phase 2 and §8) — printed,
# not executed, so the plan and the capture tooling share one source.
# ===========================================================================
# Training-capture arms: seeds 1101-1114, DISJOINT from the master-seed-42
# eval family (ADR-0042 capture/eval seed-disjointness discipline).
# TODO(sim): [A1] verify the 12/15 m/s arm geometry (y0_mag, duration)
# against a real `mc_batch.sh --dry-run`-style plan before flying.
# TODO(sim): [A2] verify `--handoff-cue-gate` exists under that name on the
# current m4_intercept.py (file mid-edit); it is a CAPTURE AID only.
TRAIN_FLIGHTS = [
    # (run_tag, path, speed_m_s, direction, seed, note)
    ("capv3_01", "weave", 12.0, "l2r", 1101, "failing regime (ADR-0056)"),
    ("capv3_02", "weave", 12.0, "r2l", 1102, "r2l asymmetry (ADR-0056)"),
    ("capv3_03", "weave", 12.0, "l2r", 1103, "2nd seed -> VAL flight"),
    ("capv3_04", "weave", 12.0, "r2l", 1104, "2nd seed -> VAL flight"),
    ("capv3_05", "jink", 12.0, "l2r", 1105, "second maneuver family"),
    ("capv3_06", "jink", 12.0, "r2l", 1106, ""),
    ("capv3_07", "jink", 12.0, "l2r", 1107, "2nd seed"),
    ("capv3_08", "jink", 12.0, "r2l", 1108, "2nd seed -> VAL flight"),
    ("capv3_09", "weave", 9.0, "l2r", 1109, "bridge speed (ADR-0038..44 era)"),
    ("capv3_10", "weave", 9.0, "r2l", 1110, "-> VAL flight"),
    ("capv3_11", "line", 9.0, "l2r", 1111, "anti-forgetting anchor"),
    ("capv3_12", "line", 6.0, "r2l", 1112, "M5 default speed anchor"),
    ("capv3_13", "weave", 15.0, "l2r", 1113, "WORST-credible margin"),
    ("capv3_14", "weave", 15.0, "r2l", 1114, "WORST-credible margin"),
]
# Eval pool (plan §8): master-seed-42 family, flown once, scored offline by
# BOTH v2 and v3. Includes the 2 Phase-0 forensics flights (evalv3_01/02).
EVAL_FLIGHTS = [
    ("evalv3_01", "weave", 12.0, "l2r", "42-family", "Phase-0 forensics too"),
    ("evalv3_02", "weave", 12.0, "r2l", "42-family", "Phase-0 forensics too"),
    ("evalv3_03", "weave", 12.0, "l2r", "42-family", "2nd seed"),
    ("evalv3_04", "weave", 12.0, "r2l", "42-family", "2nd seed"),
    ("evalv3_05", "jink", 12.0, "l2r", "42-family", ""),
    ("evalv3_06", "jink", 12.0, "r2l", "42-family", ""),
    ("evalv3_07", "weave", 9.0, "l2r", "42-family", ""),
    ("evalv3_08", "weave", 9.0, "r2l", "42-family", ""),
    ("evalv3_09", "line", 9.0, "l2r", "42-family", "G3 no-regression"),
    ("evalv3_10", "line", 6.0, "r2l", "42-family", "G3 no-regression"),
]
# Weave/jink knobs for ALL flights: the T21/defaults set — period 4 s,
# lat-speed 3 m/s (amplitude 1.91 m), jink-count 2, jink-min-deg 40
# (m4_target_mover.py DEFAULT_*; ADR-0056 arms).
MOVER_KNOBS = dict(weave_period_s=4.0, weave_lat_speed=3.0,
                   jink_count=2, jink_min_deg=40.0)
# Recorder settings per flight (plan §3 Phase 2): capture_flight_frames.py
RECORDER_ARGS = "--every 3 --max-frames 300"


def print_flight_plan() -> int:
    print("=" * 78)
    print("PHASE-2 TRAINING-CAPTURE FLIGHTS (seeds 1101-1114, disjoint from "
          "eval 42-family)")
    print("=" * 78)
    hdr = f"{'run_tag':<10} {'path':<6} {'m/s':>5} {'dir':<4} {'seed':>10}  note"
    print(hdr)
    for tag, path, spd, d, seed, note in TRAIN_FLIGHTS:
        print(f"{tag:<10} {path:<6} {spd:>5.1f} {d:<4} {str(seed):>10}  {note}")
    print()
    print("EVAL-POOL FLIGHTS (master-seed-42 family; NEVER in training)")
    print(hdr)
    for tag, path, spd, d, seed, note in EVAL_FLIGHTS:
        print(f"{tag:<10} {path:<6} {spd:>5.1f} {d:<4} {str(seed):>10}  {note}")
    print()
    print(f"mover knobs : {MOVER_KNOBS}")
    print(f"recorder    : capture_flight_frames.py {RECORDER_ARGS} "
          f"--run-tag <run_tag>")
    print("orchestrate : scripts/seeker/capture_pass.sh pattern (fresh boot "
          "per flight,\n              sequential, ONE sim at a time). "
          "Capture flights add --handoff-cue-gate 8 [A2].")
    print("TODO(sim)   : [A1] verify 12/15 m/s arm geometry (y0_mag, "
          "duration) before flying.")
    return 0


# ===========================================================================
# Self-test (offline)
# ===========================================================================
def run_self_test() -> int:
    cases = [
        # (name, rel_optical (x right, y down, z fwd), expected kind)
        ("boresight 5 m", (0.0, 0.0, 5.0), "positive"),
        ("behind camera", (0.0, 0.0, -3.0), "negative"),
        ("far off-axis (off-frame)", (50.0, 0.0, 5.0), "negative"),
        ("too far (sub-4px @640-native... 500 m)", (0.0, 0.0, 500.0), "drop"),
        ("50 m grid cell (must stay positive)", (0.0, -0.2, 50.0), "positive"),
    ]
    all_ok = True
    for name, rel, expect in cases:
        kind, box = project_box(rel)
        ok = kind == expect
        all_ok &= ok
        print(f"[self-test] {name}: {rel} -> {kind!r} (expect {expect!r}) "
              f"box={box} {'PASS' if ok else 'FAIL'}")

    # Matrix sanity: expected counts + every pose passes the pre-check.
    poses = build_grid_poses()
    n_full = sum(1 for r in RANGES_M if r <= FULL_HEIGHT_MAX_RANGE_M)
    n_far = len(RANGES_M) - n_full
    expect_n = (n_full * len(BEARINGS_DEG) * len(HEIGHTS_M) * len(YAWS_DEG)
                + n_far * len(BEARINGS_DEG) * 1 * len(YAWS_DEG))
    ok = len(poses) == expect_n
    all_ok &= ok
    print(f"[self-test] matrix count: {len(poses)} (expect {expect_n}) "
          f"{'PASS' if ok else 'FAIL'}")
    n_bad = sum(1 for p in poses if not precheck_pose(p)[0])
    all_ok &= (n_bad == 0)
    print(f"[self-test] pose pre-check: {len(poses) - n_bad}/{len(poses)} "
          f"feasible {'PASS' if n_bad == 0 else 'FAIL'}")
    print(f"[self-test] {'ALL PASS' if all_ok else 'SOME FAILED'}")
    return 0 if all_ok else 1


# ===========================================================================
# Dry-run (offline): print the matrix, nothing else
# ===========================================================================
def run_dry_run(args) -> int:
    poses = build_grid_poses()
    print("=" * 78)
    print("SEEKER v3 PHASE-1 GRID — DRY RUN (no sim contact, nothing written)")
    print("=" * 78)
    print(f"ranges (m)     : {RANGES_M}  (held-out buckets -> val: "
          f"{HELD_OUT_RANGES_M}, split applied by the dataset builder)")
    print(f"yaws (deg)     : {YAWS_DEG}")
    print(f"bearings (deg) : {BEARINGS_DEG}  (inside the deployed +/-30deg "
          "self-mask)")
    print(f"heights (m)    : {HEIGHTS_M} at r<={FULL_HEIGHT_MAX_RANGE_M}; "
          f"{CORRIDOR_Z_M} only beyond")
    print(f"extent (m)     : {args.extent_m}   min box px: {MIN_BOX_PX}")
    print(f"TOTAL POSES    : {len(poses)}  (+{NEG_N_FRAMES} empty-frame "
          "negatives)")
    print()
    print(f"{'label':<38}{'x':>7}{'y':>8}{'z':>6}  {'boxWxH px':>12}  check")
    n_bad = 0
    for p in poses:
        ok, kind, (bw, bh) = precheck_pose(p, args.extent_m)
        flag = "OK" if ok else f"REJECT({kind})"
        n_bad += 0 if ok else 1
        if args.verbose or not ok or p["yaw_deg"] == 0.0 and p["height_m"] == CORRIDOR_Z_M:
            # default: print one representative row per (range, bearing)
            print(f"{p['label']:<38}{p['x']:>7.1f}{p['y']:>8.2f}"
                  f"{p['z']:>6.2f}  {bw:>5.1f}x{bh:<5.1f}  {flag}")
    print()
    print(f"pre-check: {len(poses) - n_bad}/{len(poses)} poses feasible"
          + ("" if n_bad == 0 else f"  <<< {n_bad} REJECTED — fix the matrix "
             "before any live capture"))
    est_s = len(poses) * 2.2  # ~0.5 s teleport + 6 frames @30 Hz + margin
    print(f"rough capture estimate: ~{est_s / 60:.0f} min at RTF 1.0 "
          "(sim-paced; RTF sag stretches wall time, never sim correctness)")
    print("[dry-run] OK — nothing captured.")
    return 0 if n_bad == 0 else 2


# ===========================================================================
# LIVE capture (sim required; gz/cv2 imported lazily HERE only)
# ===========================================================================
def run_live(args) -> int:
    # Lazy imports so --dry-run/--self-test/--print-flight-plan never touch
    # the sim stack (offline-review safety).
    import cv2  # noqa: F401
    sys.path.insert(0, HERE)
    sys.path.insert(0, os.path.join(HERE, "seeker"))
    from m1_capture import image_msg_to_bgr  # the repo's one RGB decoder
    from m2_detect import (IMAGE_TOPIC, POSE_TOPIC, WORLD_NAME,
                           TAG_MODEL_NAME, PoseTracker)
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V

    set_pose_service = f"/world/{WORLD_NAME}/set_pose"

    class FrameHolder:
        def __init__(self):
            self.count, self.bgr, self.w, self.h = 0, None, 0, 0

        def on_image(self, msg):
            try:
                self.bgr = image_msg_to_bgr(msg)
                self.w, self.h = msg.width, msg.height
                self.count += 1
            except Exception as exc:  # pragma: no cover
                print(f"[v3-grid] frame decode error: {exc}")

    def teleport(x, y, z, yaw=0.0):
        # `gz service` CLI SUBPROCESS, never node.request() — this process
        # holds subscriptions (the documented gz-transport13 quirk; see
        # module docstring / multirange_capture.teleport_model).
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        req = (f'name: "{TAG_MODEL_NAME}" position {{ x: {x} y: {y} z: {z} }} '
               f'orientation {{ x: 0 y: 0 z: {qz} w: {qw} }}')
        cmd = ["gz", "service", "-s", set_pose_service,
               "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
               "--timeout", "2000", "--req", req]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True,
                               timeout=TELEPORT_TIMEOUT_S)
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            print(f"[v3-grid] teleport failed ({exc})")
            return False
        return r.returncode == 0

    def wait_new_frames(holder, n, timeout_s=SETTLE_TIMEOUT_S):
        start, t0 = holder.count, time.time()
        while holder.count < start + n:
            if time.time() - t0 > timeout_s:
                return False
            time.sleep(0.03)
        return True

    # ---- pre-flight: build + check the whole matrix BEFORE touching sim ----
    poses = build_grid_poses()
    bad = [p["label"] for p in poses if not precheck_pose(p, args.extent_m)[0]]
    if bad:
        print(f"[v3-grid] FAIL: {len(bad)} poses rejected by the offline "
              f"pre-check (first: {bad[0]}). Fix the matrix; no sim contact "
              "attempted.")
        return 2

    runstamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_root = args.out or os.path.join(REPO_ROOT, "scripts", "seeker",
                                        "data", f"v3_grid_{runstamp}")
    img_dir = os.path.join(out_root, "images")
    lbl_dir = os.path.join(out_root, "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)

    try:
        git_rev = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT,
                                 capture_output=True, text=True,
                                 timeout=5).stdout.strip() or "unknown"
    except Exception:
        git_rev = "unknown"

    node = Node()
    holder = FrameHolder()
    tracker = PoseTracker()
    if not node.subscribe(Image, IMAGE_TOPIC, holder.on_image):
        print(f"[v3-grid] FAILED: subscribe {IMAGE_TOPIC}")
        return 2
    if not node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v):
        print(f"[v3-grid] FAILED: subscribe {POSE_TOPIC}")
        return 2

    print(f"[v3-grid] world={WORLD_NAME} target={TAG_MODEL_NAME} "
          f"out={out_root}")
    t0 = time.time()
    while holder.count == 0:
        if time.time() - t0 > STARTUP_TIMEOUT_S:
            print("[v3-grid] FAILED: no camera frames within "
                  f"{STARTUP_TIMEOUT_S:.0f}s (is the markerless sim up?)")
            return 2
        time.sleep(0.1)
    print(f"[v3-grid] first frame received ({holder.w}x{holder.h})")
    if (holder.w, holder.h) != (IMG_W, IMG_H):
        print(f"[v3-grid] FAILED: resolution {holder.w}x{holder.h} != "
              f"expected {IMG_W}x{IMG_H} (camera_intrinsics.json)")
        return 2

    # ---- posecheck [A8] + long-teleport settle check [A4]: verify the
    # parked-interceptor camera geometry against LIVE ground truth at two
    # known boresight poses (near 6 m and far 50 m) BEFORE trusting the
    # 648-frame batch. A boresight target (world y=0) must land near image
    # center with its optical FORWARD component ≈ the commanded range; a
    # gross miss means a wrong world/target/boresight/sign convention or a
    # stale render after a long hop. Abort loudly rather than capture
    # mislabeled frames. (multirange_capture.py --posecheck spirit; the far
    # 50 m probe doubles as the [A4] long-teleport settle verification.)
    posecheck_rows = []
    for pc_x, pc_tag in [(6.0, "near_6m"), (50.0, "far_50m")]:
        pc_z = CORRIDOR_Z_M
        if not teleport(pc_x, 0.0, pc_z):
            print(f"[v3-grid] FAILED: posecheck {pc_tag}: teleport failed")
            node.unsubscribe(IMAGE_TOPIC); node.unsubscribe(POSE_TOPIC)
            return 2
        if not wait_new_frames(holder, args.settle_frames):
            print(f"[v3-grid] FAILED: posecheck {pc_tag}: settle timeout after "
                  f"teleport to {pc_x} m — long-hop render did not update [A4]")
            node.unsubscribe(IMAGE_TOPIC); node.unsubscribe(POSE_TOPIC)
            return 2
        rel, ok, reason = tracker.ground_truth_rel_optical()
        if not ok:
            print(f"[v3-grid] FAILED: posecheck {pc_tag}: gt unavailable ({reason})")
            node.unsubscribe(IMAGE_TOPIC); node.unsubscribe(POSE_TOPIC)
            return 2
        kind, box = project_box(rel, args.extent_m, holder.w, holder.h)
        fwd = float(rel[2])
        if box is None:
            print(f"[v3-grid] FAILED: posecheck {pc_tag}: classify={kind} "
                  f"(expected positive); rel_optical={[round(float(v),3) for v in rel]}")
            node.unsubscribe(IMAGE_TOPIC); node.unsubscribe(POSE_TOPIC)
            return 2
        bcx_px, bcy_px = box[0] * holder.w, box[1] * holder.h
        du, dv = bcx_px - CX, bcy_px - CY
        row = {"tag": pc_tag, "commanded_range_m": pc_x, "kind": kind,
               "center_px": [round(bcx_px, 1), round(bcy_px, 1)],
               "dx_px": round(du, 1), "dy_px": round(dv, 1),
               "fwd_m": round(fwd, 3),
               "rel_optical": [round(float(v), 3) for v in rel]}
        posecheck_rows.append(row)
        print(f"[v3-grid] posecheck {pc_tag}: kind={kind} "
              f"center=({bcx_px:.0f},{bcy_px:.0f}) dx={du:+.0f}px dy={dv:+.0f}px "
              f"fwd={fwd:.2f}m (commanded range {pc_x}m)")
        # Boresight (y=0) target: horizontal center within ~40 px of image
        # center; vertical offset can reach ~-25 px near range from the
        # camera/target height difference, so allow ~70 px; forward optical
        # component must match the commanded range within 1 m (catches a
        # stale/mis-scaled render or wrong axis convention).
        if kind != "positive" or abs(du) > 40.0 or abs(dv) > 70.0 or abs(fwd - pc_x) > 1.0:
            print(f"[v3-grid] FAILED: posecheck {pc_tag} OUT OF TOLERANCE — a "
                  "boresight target must sit near image center with fwd≈range. "
                  "Wrong world/target/boresight/sign convention or a settle "
                  "problem; aborting before capturing mislabeled frames.")
            node.unsubscribe(IMAGE_TOPIC); node.unsubscribe(POSE_TOPIC)
            return 2
    print("[v3-grid] posecheck [A8] PASSED (near+far boresight geometry + "
          "50 m long-teleport settle verified [A4]).")

    index_rows = []
    n_ok = n_skip = 0
    for seq, p in enumerate(poses):
        if not teleport(p["x"], p["y"], p["z"], p["yaw_rad"]):
            n_skip += 1
            continue
        if not wait_new_frames(holder, args.settle_frames):
            print(f"[v3-grid] seq={seq} {p['label']}: settle timeout — skip")
            n_skip += 1
            continue
        rel, ok, reason = tracker.ground_truth_rel_optical()
        if not ok:
            print(f"[v3-grid] seq={seq} {p['label']}: gt miss ({reason}) — skip")
            n_skip += 1
            continue
        kind, box = project_box(rel, args.extent_m,
                                holder.w, holder.h)
        if kind != "positive":
            # A grid pose should always be a clean positive (the pre-check
            # guaranteed it geometrically) — a live non-positive means a
            # stale render or a settle problem [A4]: log loudly, skip.
            print(f"[v3-grid] seq={seq} {p['label']}: live classify={kind} "
                  "(expected positive) — SKIP + investigate settle")
            n_skip += 1
            continue
        stem = f"{seq:05d}_{p['label']}"
        cv2.imwrite(os.path.join(img_dir, stem + ".png"), holder.bgr)
        with open(os.path.join(lbl_dir, stem + ".txt"), "w") as fh:
            fh.write(f"0 {box[0]:.6f} {box[1]:.6f} {box[2]:.6f} {box[3]:.6f}\n")
        index_rows.append({
            "seq": seq, "stem": stem, "kind": "positive",
            "gt_target_x": f"{p['x']:.4f}", "gt_target_y": f"{p['y']:.4f}",
            "gt_target_z": f"{p['z']:.4f}",
            "range_m": p["range_m"], "bearing_deg": p["bearing_deg"],
            "height_m": p["height_m"], "yaw_deg": p["yaw_deg"],
            "held_out_range": int(p["held_out_range"]),
        })
        n_ok += 1

    # ---- negative phase (small; plan §4.3) ----
    n_neg = 0
    nx, ny, nz = NEG_TARGET_POSE
    if not args.no_negatives and teleport(nx, ny, nz):
        if wait_new_frames(holder, args.settle_frames):
            for k in range(NEG_N_FRAMES):
                if not wait_new_frames(holder, 1):
                    break
                stem = f"neg_{k:05d}_empty"
                cv2.imwrite(os.path.join(img_dir, stem + ".png"), holder.bgr)
                open(os.path.join(lbl_dir, stem + ".txt"), "w").close()
                index_rows.append({
                    "seq": len(poses) + k, "stem": stem, "kind": "negative",
                    "gt_target_x": nx, "gt_target_y": ny, "gt_target_z": nz,
                    "range_m": "", "bearing_deg": "", "height_m": "",
                    "yaw_deg": "", "held_out_range": 0,
                })
                n_neg += 1

    node.unsubscribe(IMAGE_TOPIC)
    node.unsubscribe(POSE_TOPIC)

    with open(os.path.join(out_root, "index.csv"), "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(index_rows[0].keys()))
        writer.writeheader()
        writer.writerows(index_rows)
    meta = {
        "phase": "v3_grid", "world": WORLD_NAME, "target_model": TAG_MODEL_NAME,
        "ranges_m": RANGES_M, "held_out_ranges_m": HELD_OUT_RANGES_M,
        "yaws_deg": YAWS_DEG, "bearings_deg": BEARINGS_DEG,
        "heights_m": HEIGHTS_M,
        "full_height_max_range_m": FULL_HEIGHT_MAX_RANGE_M,
        "extent_m": args.extent_m, "settle_frames": args.settle_frames,
        "resolution": [IMG_W, IMG_H],
        "intrinsics": {"fx": FX, "fy": FY, "cx": CX, "cy": CY},
        "n_positive": n_ok, "n_negative": n_neg, "n_skipped": n_skip,
        "posecheck": posecheck_rows,  # [A8]/[A4] live geometry+settle verification
        "git_rev": git_rev, "runstamp": runstamp,
        "plan": "docs/seeker_v3_dataset_plan.md",
    }
    with open(os.path.join(out_root, "capture_meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)

    print(f"[v3-grid] DONE: {n_ok} positives + {n_neg} negatives "
          f"({n_skip} skipped) -> {out_root}")
    return 0 if n_ok > 0 else 2


# ===========================================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true",
                      help="print the capture matrix + feasibility checks; "
                           "NO sim contact, nothing written (offline-safe)")
    mode.add_argument("--self-test", action="store_true",
                      help="run the local projection/matrix self-tests and "
                           "exit (offline-safe)")
    mode.add_argument("--print-flight-plan", action="store_true",
                      help="print the Phase-2 training-capture and eval-pool "
                           "flight tables (offline-safe)")
    ap.add_argument("--out", default=None,
                    help="output root (default scripts/seeker/data/"
                         "v3_grid_<runstamp>)")
    ap.add_argument("--extent-m", type=float, default=DEFAULT_EXTENT_M,
                    help=f"bbox silhouette extent (default {DEFAULT_EXTENT_M},"
                         " matches render_sim_dataset/capture_flight_frames)")
    ap.add_argument("--settle-frames", type=int, default=SETTLE_FRAMES,
                    help=f"new frames to wait per teleport (default "
                         f"{SETTLE_FRAMES}; TODO(sim) verify at 35/50 m [A4])")
    ap.add_argument("--no-negatives", action="store_true",
                    help="skip the small empty-frame negative phase")
    ap.add_argument("--verbose", action="store_true",
                    help="--dry-run: print every pose row, not one per cell")
    args = ap.parse_args()

    if args.self_test:
        return run_self_test()
    if args.print_flight_plan:
        return print_flight_plan()
    if args.dry_run:
        return run_dry_run(args)
    return run_live(args)


if __name__ == "__main__":
    raise SystemExit(main())
