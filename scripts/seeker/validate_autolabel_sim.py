#!/usr/bin/env python3
"""Validate the AprilTag auto-labeler IN THE apriltag SIM WORLD, and produce the
placard-sizing curve (build_plan P0, docs/real_data_pipeline.md "Validate the
auto-labeler itself" -- flagged "NOT yet run").

WHAT THIS ANSWERS (docs/real_data_pipeline.md, ADR-0076 add #18h):
  (a) LABEL RATE vs range   -- fraction of frames where the tag decodes and the
                               auto-labeler emits a box. This is the "label
                               ceiling" the far-band strategy + placard size hang
                               on.
  (b) IoU tag-box vs gt-box -- does the tag-pose-projected auto-label box land on
                               the SAME pixels the gt-projected box (what
                               gen_sim_dataset.py would write) does? This is the
                               HONEST version of the check the doc flagged as
                               previously circular: the two boxes use the SAME
                               projector + SAME drone extent, so the ONLY thing
                               that moves the IoU off 1.0 is the difference between
                               the *decoded* tag pose (from real pixels via
                               pupil-apriltags) and the *ground-truth* pose (from
                               the world pose topic, the M2 transform chain).
  (c) range error vs range  -- decoded tag range minus gt range.

The apriltag world has BOTH the tag AND gt, so this is measurable now, before any
hardware flies. gt is SCORING/label-source ONLY (CLAUDE.md honesty boundary).

HOW (--sweep, needs a running sim): with PX4 SITL + gz_x500_mono_cam + the
"apriltag" world already up (validate_autolabel_sim.sh boots it), reposition the
INTERCEPTOR (not the static tag) along the tag's boresight to a range ladder plus
a few lateral off-boresight placements, sampling live camera frames at each. The
relative camera->tag geometry is identical to teleporting the tag, and moving the
interceptor is the proven set_pose pattern (scripts/check_m2_envelope.py). gt is
read LIVE from the pose topic every frame (m2_detect PoseTracker), so it is exact
regardless of which body moved. A subscription-free child issues the set_pose
calls (the gz-transport13 subscribe-vs-service quirk, m4_target_mover.py).

At each sampled frame we save the PNG (so autolabel_from_apriltag.py can run over
the captures as an independent cross-check) and record, per frame: gt optical
vector + range, the gt box, the auto-label box (via the SHIPPED autolabel_frame),
the decoded range + decision_margin, and their IoU -> logs/<stamp>.csv.

HOW (--analyze CSV, no sim): bin by range, compute label-rate / mean-IoU /
range-error curves, render PNGs to docs/images/, and run the placard-sizing
transfer math (sim 0.5 m tag -> candidate real placard edges under the OV9281
px/deg penalty) against the tripod_score.py money-gate arithmetic. Prints a JSON
summary the doc quotes.

Run:
    # sweep (sim up):
    .venv/bin/python scripts/seeker/validate_autolabel_sim.py --sweep \
        --out scripts/seeker/data/autolabel_sim --csv logs/autolabel_sim_<stamp>.csv
    # analyze (no sim):
    .venv/bin/python scripts/seeker/validate_autolabel_sim.py --analyze logs/autolabel_sim_<stamp>.csv \
        --images-dir docs/images

Requires cv2 + pupil-apriltags + numpy (+ matplotlib for --analyze). Run under .venv.
"""
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
SCRIPTS = os.path.dirname(HERE)
REPO = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)   # scripts/  (m2_detect, gen_sim_dataset via seeker)
sys.path.insert(0, HERE)      # scripts/seeker/

# --- sim-world tag geometry + camera (traced numbers) ----------------------
# models/apriltag_target/model.sdf: 0.625 m plane * 0.8 = 0.5 m black square,
# which is what pupil-apriltags' tag_size measures (make_tag_texture.py).
TAG_SIZE_M = 0.5
# 5" quad extent; the box gen_sim_dataset / autolabel project (both curves use
# the SAME extent so IoU isolates pose error, not size).
DRONE_SIZE_M = 0.35
TAG_X = 5.0            # tag world x (worlds/apriltag.sdf)

# Range ladder (m, forward standoff) + a few off-boresight lateral placements.
# Brackets past the expected fail edge so a FAIL point is observed, not assumed
# (check_m2_envelope.py rationale).
DEFAULT_RANGES = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]
# (forward_m, lateral_y_m): move the interceptor sideways so the tag lands off
# boresight (bearing = atan(|y|/fwd)); tests frame-edge decode + projection.
DEFAULT_OFFAXIS = [(8, 3.0), (8, -3.0), (8, 5.0), (12, 4.0), (12, -4.0), (16, 5.0)]

FRAMES_PER_PLACEMENT = 12
SETTLE_S = 1.8

# --- placard-sizing transfer (docs/camera_paper_check.md + tripod_score.py) --
# Sim camera: fx measured live from CameraInfo; nominal 539.936 px @ 1280x960,
# HFoV ~100 deg -> 12.8 px/deg (avg). OV9281: 1280 px / 118 deg = 10.85 px/deg.
OV9281_PXDEG_H = 1280.0 / 118.0          # 10.847 px/deg (camera_paper_check §2)
CANDIDATE_EDGES_M = [0.20, 0.25, 0.30, 0.35]   # 0.35 = carry limit (hw_order §E)
CARRY_LIMIT_M = 0.35
# tripod_score.py money-gate constants (import-checked at analyze time).
TGO_MIN_S = 0.5
V_CLOSING_MPS = 9.0        # conservative (the GATE scenario)
V_CLOSING_HI_MPS = 20.0    # aggressive (context only)
HANDOFF_STREAK = 5
# STREAM_FPS: the capture rate the gate is evaluated at. CORRECTED 2026-07-24
# (ADR-0082): this was a bare 30.0 with no source, while the deployed flight loop
# runs 20 Hz and the only MEASURED AprilTag cadence in the repo is ~14 Hz. The
# rate is load-bearing -- it moves the R_decode90 threshold by ~1.6 m between 30
# and 20 Hz at p=0.9 -- so the sizing report now prints a BAND over these rates
# instead of a single optimistic number. The authority is the rate MEASURED on the
# Pi bench (pi_capture records stream_fps + a p10 slow tail); tripod_score.py
# refuses to invent one.
STREAM_FPS = 30.0                      # legacy single-point (kept for provenance)
STREAM_FPS_BAND = (30.0, 20.0, 14.0)   # 30 = as-published, 20 = deployed loop, 14 = measured tag rate


# ==========================================================================
# geometry helpers
# ==========================================================================
def iou_centerbox(a, b):
    """IoU of two (cx, cy, w, h) pixel boxes."""
    if a is None or b is None:
        return None
    ax0, ay0 = a[0] - a[2] / 2, a[1] - a[3] / 2
    ax1, ay1 = a[0] + a[2] / 2, a[1] + a[3] / 2
    bx0, by0 = b[0] - b[2] / 2, b[1] - b[3] / 2
    bx1, by1 = b[0] + b[2] / 2, b[1] + b[3] / 2
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 0 else 0.0


# ==========================================================================
# --sweep  (needs a running sim)
# ==========================================================================
# Subscription-free child that issues set_pose (the drone body). Kept isolated
# from any subscription so it can actually receive the gz service response
# (gz-transport13 quirk, m4_target_mover.py / check_m2_envelope.py).
_MOVER_SRC = """
import sys
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
SVC = "/world/apriltag/set_pose"
node = Node()
for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    name, x, y, z = line.split(",")
    req = Pose()
    req.name = name
    req.position.x = float(x); req.position.y = float(y); req.position.z = float(z)
    req.orientation.x = 0.0; req.orientation.y = 0.0; req.orientation.z = 0.0
    req.orientation.w = 1.0
    ok, resp = node.request(SVC, req, Pose, Boolean, 3000)
    print("OK" if (ok and resp.data) else "FAIL", flush=True)
"""


class Mover:
    def __init__(self):
        self.p = subprocess.Popen([sys.executable, "-c", _MOVER_SRC],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True, bufsize=1)

    def move(self, name, x, y, z, timeout=6.0):
        self.p.stdin.write(f"{name},{x},{y},{z}\n")
        self.p.stdin.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.p.stdout.readline()
            if line:
                return line.strip() == "OK"
        return False

    def close(self):
        try:
            self.p.stdin.close()
        except Exception:
            pass
        self.p.terminate()
        try:
            self.p.wait(timeout=3)
        except Exception:
            self.p.kill()


def run_sweep(args):
    import cv2
    import queue
    from apriltag_detector import Detector
    from m2_detect import (
        CAMERA_INFO_TIMEOUT_S, DRONE_MODEL, EXPECTED_WIDTH, EXPECTED_HEIGHT,
        IMAGE_TOPIC, POSE_TOPIC, TAG_FAMILY, PoseTracker,
        get_camera_intrinsics, image_msg_to_gray,
    )
    from gen_sim_dataset import project_to_bbox
    from autolabel_from_apriltag import autolabel_frame
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V

    ranges = args.ranges or DEFAULT_RANGES
    out_dir = args.out
    img_dir = os.path.join(out_dir, "images")
    os.makedirs(img_dir, exist_ok=True)

    node = Node()
    print(f"[sweep] reading intrinsics from {IMAGE_TOPIC} sibling CameraInfo ...")
    fx, fy, cx, cy, w, h = get_camera_intrinsics(node, CAMERA_INFO_TIMEOUT_S)
    print(f"[sweep] intrinsics fx={fx:.3f} fy={fy:.3f} cx={cx:.1f} cy={cy:.1f} ({w}x{h})")
    if (w, h) != (EXPECTED_WIDTH, EXPECTED_HEIGHT):
        print(f"[sweep] FAIL: resolution {w}x{h} != {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}")
        return 1

    # write calib.json for the autolabel CLI cross-check (sim = zero distortion)
    with open(os.path.join(out_dir, "calib.json"), "w") as f:
        json.dump(dict(fx=fx, fy=fy, cx=cx, cy=cy,
                       resolution=dict(width=w, height=h),
                       dist_coeffs=[0.0] * 5), f, indent=2)

    tracker = PoseTracker()
    if not node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v):
        print(f"[sweep] FAIL: subscribe {POSE_TOPIC}")
        return 1

    fq: "queue.Queue[Image]" = queue.Queue()

    def on_image(msg):
        try:
            while True:
                fq.get_nowait()
        except queue.Empty:
            pass
        fq.put(msg)

    if not node.subscribe(Image, IMAGE_TOPIC, on_image):
        print(f"[sweep] FAIL: subscribe {IMAGE_TOPIC}")
        return 1

    print("[sweep] waiting for pose topic ...")
    deadline = time.monotonic() + 30.0
    while DRONE_MODEL not in tracker.latest and time.monotonic() < deadline:
        time.sleep(0.1)
    if DRONE_MODEL not in tracker.latest:
        print(f"[sweep] FAIL: no pose for {DRONE_MODEL}")
        return 1
    p0, _ = tracker.latest[DRONE_MODEL]
    y0, z0 = float(p0[1]), float(p0[2])
    print(f"[sweep] baseline spawn y={y0:.3f} z={z0:.3f}")

    mover = Mover()
    detector = Detector(families=TAG_FAMILY)

    os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
    cf = open(args.csv, "w", newline="")
    wr = csv.writer(cf)
    wr.writerow(["fwd_cmd_m", "y_off_cmd_m", "gt_x", "gt_y", "gt_z", "gt_range_m",
                 "bearing_deg", "decoded", "dec_range_m", "decision_margin", "iou",
                 "u_px", "gt_cx", "gt_cy", "gt_w", "gt_h",
                 "tag_cx", "tag_cy", "tag_w", "tag_h", "frame"])

    placements = [(r, 0.0) for r in ranges] + [(fwd, y) for (fwd, y) in DEFAULT_OFFAXIS]
    seq = 0
    try:
        for (fwd, yoff) in placements:
            dx = TAG_X - fwd
            dy = y0 + yoff
            ok = mover.move(DRONE_MODEL, dx, dy, z0)
            tag = f"fwd{fwd:05.1f}_y{yoff:+04.1f}"
            if not ok:
                print(f"[sweep] WARN set_pose not confirmed for {tag}")
            time.sleep(SETTLE_S)
            # drain stale frames from the settle window
            try:
                while True:
                    fq.get_nowait()
            except queue.Empty:
                pass

            n_saved = 0
            n_dec = 0
            t_end = time.monotonic() + 6.0
            while n_saved < FRAMES_PER_PLACEMENT and time.monotonic() < t_end:
                try:
                    msg = fq.get(timeout=1.0)
                except queue.Empty:
                    continue
                if msg.width != w or msg.height != h:
                    continue
                gt_rel, gt_ok, reason = tracker.ground_truth_rel_optical()
                if not gt_ok:
                    continue
                gx, gy, gz = float(gt_rel[0]), float(gt_rel[1]), float(gt_rel[2])
                gt_range = float(np.linalg.norm(gt_rel))
                bearing = math.degrees(math.atan2(math.hypot(gx, gy), gz))
                gray = image_msg_to_gray(msg)

                gt_box = project_to_bbox(gx, gy, gz, gt_range, fx, fy, cx, cy,
                                         DRONE_SIZE_M, w, h)
                # THE SHIPPED auto-labeler box (tag pose -> drone box):
                tag_box = autolabel_frame(gray, detector, (fx, fy, cx, cy),
                                          TAG_SIZE_M, DRONE_SIZE_M, (0.0, 0.0, 0.0), w, h)
                # decoded pose diagnostics (same detector settings)
                dets = detector.detect(gray, estimate_tag_pose=True,
                                       camera_params=(fx, fy, cx, cy), tag_size=TAG_SIZE_M)
                decoded = 1 if dets else 0
                if dets:
                    d = min(dets, key=lambda t: float(t.pose_t.reshape(3)[2]))
                    dec_range = float(np.linalg.norm(d.pose_t.reshape(3)))
                    margin = float(d.decision_margin)
                    n_dec += 1
                else:
                    dec_range = float("nan")
                    margin = float("nan")
                iou = iou_centerbox(tag_box, gt_box)
                u_px = tag_box[0] if tag_box else (gt_box[0] if gt_box else float("nan"))

                fname = f"{tag}_{seq:04d}_r{gt_range:05.1f}.png"
                cv2.imwrite(os.path.join(img_dir, fname), cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
                wr.writerow([
                    f"{fwd:.1f}", f"{yoff:+.1f}", f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
                    f"{gt_range:.4f}", f"{bearing:.2f}", decoded,
                    ("" if math.isnan(dec_range) else f"{dec_range:.4f}"),
                    ("" if math.isnan(margin) else f"{margin:.3f}"),
                    ("" if iou is None else f"{iou:.4f}"),
                    f"{u_px:.1f}",
                    *(f"{v:.2f}" for v in (gt_box if gt_box else (float('nan'),) * 4)),
                    *(f"{v:.2f}" for v in (tag_box if tag_box else (float('nan'),) * 4)),
                    fname,
                ])
                cf.flush()
                seq += 1
                n_saved += 1
            print(f"[sweep] {tag}: gt_range~{gt_range:.1f}m saved={n_saved} decoded={n_dec}/{n_saved}")
    finally:
        mover.close()
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        cf.close()
    print(f"[sweep] DONE -> {args.csv} ; frames -> {img_dir}")
    return 0


# ==========================================================================
# --analyze  (no sim)
# ==========================================================================
def _load_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def _fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def analyze(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = _load_csv(args.analyze)
    # boresight (y_off == 0) drives the decode-envelope curve; off-axis reported separately
    on = [r for r in rows if abs(_fnum(r["y_off_cmd_m"])) < 0.05]
    off = [r for r in rows if abs(_fnum(r["y_off_cmd_m"])) >= 0.05]

    bin_m = 2.0
    bins = {}   # lo -> dict
    for r in on:
        gr = _fnum(r["gt_range_m"])
        if math.isnan(gr):
            continue
        lo = math.floor(gr / bin_m) * bin_m
        b = bins.setdefault(lo, dict(n=0, dec=0, ious=[], rerr=[]))
        b["n"] += 1
        if int(r["decoded"]):
            b["dec"] += 1
            iou = _fnum(r["iou"])
            if not math.isnan(iou):
                b["ious"].append(iou)
            dr = _fnum(r["dec_range_m"])
            if not math.isnan(dr):
                b["rerr"].append(dr - gr)
    los = sorted(bins)

    # --- curve (a): R_decode_any and R_decode90 (sustained >=90% inward) ------
    r_any = 0.0
    for lo in los:
        b = bins[lo]
        if b["dec"] > 0:
            r_any = max(r_any, lo + bin_m)   # farthest bin's upper edge with ANY decode
    # sustained-90 walk from nearest bin outward
    r90 = 0.0
    for lo in los:
        b = bins[lo]
        rate = b["dec"] / b["n"] if b["n"] else 0.0
        if rate >= 0.9:
            r90 = lo + bin_m
        else:
            break
    # decode rate sustained in the R_decode90 boundary bin (for streak burn)
    r90_lo = r90 - bin_m
    rate_at_r90 = (bins[r90_lo]["dec"] / bins[r90_lo]["n"]) if r90_lo in bins and bins[r90_lo]["n"] else 0.0

    # aggregate IoU / range-error (decoded frames only)
    all_ious = [_fnum(r["iou"]) for r in on if int(r["decoded"]) and not math.isnan(_fnum(r["iou"]))]
    all_rerr = [(_fnum(r["dec_range_m"]) - _fnum(r["gt_range_m"])) for r in on
                if int(r["decoded"]) and not math.isnan(_fnum(r["dec_range_m"]))]

    summary = {
        "n_frames_boresight": len(on),
        "n_frames_offaxis": len(off),
        "bin_m": bin_m,
        "R_decode_any_sim_m": r_any,
        "R_decode90_sim_m": r90,
        "decode_rate_at_R90": round(rate_at_r90, 3),
        "tag_size_sim_m": TAG_SIZE_M,
        "mean_iou_decoded": round(float(np.mean(all_ious)), 4) if all_ious else None,
        "median_iou_decoded": round(float(np.median(all_ious)), 4) if all_ious else None,
        "min_iou_decoded": round(float(np.min(all_ious)), 4) if all_ious else None,
        "mean_range_err_m": round(float(np.mean(all_rerr)), 4) if all_rerr else None,
        "max_abs_range_err_m": round(float(np.max(np.abs(all_rerr))), 4) if all_rerr else None,
        "per_bin": {},
    }
    for lo in los:
        b = bins[lo]
        summary["per_bin"][f"{lo:.0f}-{lo+bin_m:.0f}"] = dict(
            n=b["n"], decoded=b["dec"],
            rate=round(b["dec"] / b["n"], 3) if b["n"] else 0.0,
            mean_iou=round(float(np.mean(b["ious"])), 3) if b["ious"] else None,
            mean_range_err=round(float(np.mean(b["rerr"])), 3) if b["rerr"] else None,
        )

    # off-axis roll-up (by |bearing| band)
    off_rollup = {}
    for r in off:
        band = f"{round(abs(_fnum(r['bearing_deg'])))}deg"
        d = off_rollup.setdefault(band, dict(n=0, dec=0, ious=[]))
        d["n"] += 1
        if int(r["decoded"]):
            d["dec"] += 1
            iou = _fnum(r["iou"])
            if not math.isnan(iou):
                d["ious"].append(iou)
    summary["offaxis"] = {k: dict(n=v["n"], decoded=v["dec"],
                                  rate=round(v["dec"] / v["n"], 3) if v["n"] else 0,
                                  mean_iou=round(float(np.mean(v["ious"])), 3) if v["ious"] else None)
                          for k, v in sorted(off_rollup.items())}

    # ---------------- placard sizing math ------------------------------------
    summary["placard"] = placard_sizing(r90, r_any, rate_at_r90, args.sim_fx,
                                        r90_hires=args.r90_hires)

    # ---------------- plots ---------------------------------------------------
    os.makedirs(args.images_dir, exist_ok=True)
    centers = [lo + bin_m / 2 for lo in los]
    rates = [bins[lo]["dec"] / bins[lo]["n"] if bins[lo]["n"] else 0 for lo in los]
    mean_ious = [float(np.mean(bins[lo]["ious"])) if bins[lo]["ious"] else np.nan for lo in los]
    mean_rerr = [float(np.mean(bins[lo]["rerr"])) if bins[lo]["rerr"] else np.nan for lo in los]

    # (a) label rate
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.plot(centers, rates, "o-", color="#1f77b4", lw=2)
    ax.axhline(0.9, ls="--", color="#888", lw=1, label="90% sustain threshold")
    ax.axvline(r90, ls=":", color="#2ca02c", lw=1.5, label=f"R_decode90 = {r90:.0f} m")
    ax.axvline(r_any, ls=":", color="#d62728", lw=1.5, label=f"R_decode_any = {r_any:.0f} m")
    ax.set_xlabel("ground-truth range (m)")
    ax.set_ylabel("auto-label rate (tag decoded -> box)")
    ax.set_ylim(-0.05, 1.08)
    ax.set_title(f"AprilTag auto-label rate vs range  (sim {TAG_SIZE_M} m tag, fx={args.sim_fx:.0f} px)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    p_a = os.path.join(args.images_dir, "autolabel_label_rate_vs_range.png")
    fig.savefig(p_a, dpi=120)
    plt.close(fig)

    # (b) IoU
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.plot(centers, mean_ious, "s-", color="#9467bd", lw=2)
    ax.axhline(0.5, ls="--", color="#888", lw=1, label="IoU 0.5")
    ax.set_xlabel("ground-truth range (m)")
    ax.set_ylabel("mean IoU (auto-label box vs gt box)")
    ax.set_ylim(0, 1.02)
    ax.set_title("Auto-label box agreement with gt box vs range")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    p_b = os.path.join(args.images_dir, "autolabel_iou_vs_range.png")
    fig.savefig(p_b, dpi=120)
    plt.close(fig)

    # (c) range error
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.plot(centers, mean_rerr, "^-", color="#ff7f0e", lw=2)
    ax.axhline(0.0, ls="--", color="#888", lw=1)
    ax.set_xlabel("ground-truth range (m)")
    ax.set_ylabel("decoded range - gt range (m)")
    ax.set_title("AprilTag pose range error vs range")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    p_c = os.path.join(args.images_dir, "autolabel_range_error_vs_range.png")
    fig.savefig(p_c, dpi=120)
    plt.close(fig)

    # (d) placard transfer
    p_d = plot_placard(summary["placard"], args.images_dir, plt)

    summary["plots"] = [os.path.relpath(p, REPO) for p in (p_a, p_b, p_c, p_d)]
    print(json.dumps(summary, indent=2))
    if args.summary_out:
        with open(args.summary_out, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"[analyze] summary -> {args.summary_out}")
    return 0


def placard_sizing(r90_sim, r_any_sim, rate_at_r90, sim_fx, r90_hires=None):
    """Scale the measured sim decode envelope to the real OV9281 for each candidate
    placard edge, and run the tripod_score.py money-gate arithmetic.

    Physics: a small centered tag decodes when its black-square pixel span
    P = f_px * edge / R exceeds a decoder threshold P_min. So R_decode = f_px *
    edge / P_min  ->  R scales LINEARLY with focal length (px) and with tag edge.

    Two camera scale factors are reported (the honest bracket):
      * fx-ratio (pinhole center resolution, CONSERVATIVE, used for the GATE):
          fx_real/fx_sim, fx_real = (W/2)/tan(HFoV_real/2) for the OV9281 118 deg.
      * avg-px/deg (camera_paper_check.md's ~0.85, MIDDLE estimate):
          (1280/118)/(1280/HFoV_sim).
    The real barrel-distorted wide lens typically resolves the center BETTER than
    the pinhole fx-ratio implies, so the true transfer sits at/above the
    conservative bound -- we gate on the conservative one on purpose.
    """
    W = 1280.0
    # sim HFoV + px/deg from the measured fx
    hfov_sim = 2 * math.degrees(math.atan((W / 2) / sim_fx))
    pxdeg_sim = W / hfov_sim
    # real OV9281 pinhole focal length (px) from 118 deg HFoV
    hfov_real = 118.0
    fx_real = (W / 2) / math.tan(math.radians(hfov_real / 2))
    scale_fx = fx_real / sim_fx               # conservative
    scale_pxdeg = OV9281_PXDEG_H / pxdeg_sim  # middle (camera_paper_check ~0.85)

    # per-edge real decode ranges (linear in edge, ref sim tag = 0.5 m)
    edges = CANDIDATE_EDGES_M
    out = dict(
        sim_fx=round(sim_fx, 3), sim_hfov_deg=round(hfov_sim, 2),
        sim_pxdeg=round(pxdeg_sim, 3), ov9281_pxdeg=round(OV9281_PXDEG_H, 3),
        ov9281_fx_px=round(fx_real, 2), scale_fx_conservative=round(scale_fx, 4),
        scale_pxdeg_middle=round(scale_pxdeg, 4),
        R_decode90_sim_m=r90_sim, R_decode_any_sim_m=r_any_sim,
        candidates=[],
    )
    for edge in edges:
        edge_ratio = edge / TAG_SIZE_M
        r90_cons = r90_sim * scale_fx * edge_ratio
        r90_mid = r90_sim * scale_pxdeg * edge_ratio
        rany_cons = r_any_sim * scale_fx * edge_ratio
        gate = tripod_gate(r90_cons, rate_at_r90)
        gate_mid = tripod_gate(r90_mid, rate_at_r90)
        cand = dict(
            edge_m=edge, edge_ratio=round(edge_ratio, 3),
            R_decode90_real_conservative_m=round(r90_cons, 2),
            R_decode90_real_middle_m=round(r90_mid, 2),
            R_decode_any_real_conservative_m=round(rany_cons, 2),
            t_go_conservative_s=gate["t_go"], gate_conservative=gate["verdict"],
            t_go_middle_s=gate_mid["t_go"], gate_middle=gate_mid["verdict"],
            R_streak_burn_m=gate["r_streak_burn"],
        )
        # optional full-res (quad_decimate=1.0) envelope sensitivity
        if r90_hires:
            r90h_cons = r90_hires * scale_fx * edge_ratio
            r90h_mid = r90_hires * scale_pxdeg * edge_ratio
            cand["R_decode90_real_hires_conservative_m"] = round(r90h_cons, 2)
            cand["R_decode90_real_hires_middle_m"] = round(r90h_mid, 2)
            cand["gate_hires_conservative"] = tripod_gate(r90h_cons, rate_at_r90)["verdict"]
        out["candidates"].append(cand)

    # gate threshold on R90: t_go>=0.5 @9 m/s  ->  R90 >= 0.5*V + R_streak_burn
    burn = out["candidates"][0]["R_streak_burn_m"] or 0.0
    out["gate_R90_threshold_m"] = round(TGO_MIN_S * V_CLOSING_MPS + burn, 2)
    # ADR-0082 COMPLETION (2026-07-24, review 2): the single headline threshold above
    # is computed at STREAM_FPS and was being published ALONE -- the frame-rate BAND
    # introduced by ADR-0082 was declared but never emitted, so the "one measured fps"
    # fix was INERT and every published verdict still rode the unmeasured 30 fps. The
    # band is now materialised, and the report carries the fps each number belongs to.
    band = {}
    for _fps in STREAM_FPS_BAND:
        _g = tripod_gate(out["candidates"][0]["R_decode90_real_conservative_m"],
                         rate_at_r90, stream_fps=_fps)
        band[f"{_fps:g}fps"] = {
            "R90_threshold_m": round(TGO_MIN_S * V_CLOSING_MPS
                                     + (_g["r_streak_burn"] or 0.0), 2),
            "R_streak_burn_m": _g["r_streak_burn"],
            "E_streak_frames": _g["e_streak_frames"],
        }
    out["gate_R90_threshold_band"] = band
    out["gate_threshold_fps_assumed"] = STREAM_FPS
    out["gate_threshold_note"] = (
        "gate_R90_threshold_m is computed at gate_threshold_fps_assumed. That rate is "
        "NOT measured -- the deployed loop runs 20 Hz and the only measured tag cadence "
        "is ~14 Hz. Read gate_R90_threshold_band and use the row matching the rate "
        "MEASURED on the Pi bench (pi_capture records stream_fps). ADR-0082.")
    if r90_hires:
        out["R_decode90_sim_hires_m"] = r90_hires
        out["hires_note"] = ("quad_decimate=1.0 (full-res) sim envelope; the deployed "
                             "detector uses the library default 2.0 -- full-res buys "
                             "~1.5x range at a Pi-CPU/fps cost (bench §7.3).")

    # Decode range scales LINEARLY with placard edge and the envelope is SHORT
    # (nothing clears the aggressive 20 m/s scenario in sim), so the prudent
    # recommendation is the LARGEST carriable placard -- there is no room to go
    # smaller once the un-modeled real penalties (motion blur, mono sensor,
    # outdoor light, lens MTF -- all raise the min-decode px, all shorten range)
    # stack on. Only recommend smaller if a smaller edge clears the CONSERVATIVE
    # gate with a real safety margin at the DEPLOYED (qd=2.0) envelope.
    SAFETY = 1.3
    comfy = [c for c in out["candidates"]
             if c["t_go_conservative_s"] is not None
             and c["t_go_conservative_s"] >= TGO_MIN_S * SAFETY]
    if comfy:
        out["recommended_edge_m"] = min(c["edge_m"] for c in comfy)
        out["recommended_reason"] = (
            f"smallest edge clearing the conservative (fx-ratio, 9 m/s) gate with a "
            f">={SAFETY:.1f}x t_go safety margin at the deployed quad_decimate=2.0 envelope")
    else:
        out["recommended_edge_m"] = CARRY_LIMIT_M
        out["recommended_reason"] = (
            "sim decode envelope is SHORT -- no candidate clears the conservative gate "
            "with margin at the deployed quad_decimate=2.0 config, so size to the CARRY "
            "LIMIT (0.35 m): decode range scales linearly with edge, 0.35 m clears the "
            "gate under the realistic (px/deg) camera scaling and comfortably at "
            "quad_decimate=1.0, and every un-modeled real penalty pushes range shorter. "
            "Tripod day measures the real curve.")
    return out


def _streak_frames(p, k=HANDOFF_STREAK):
    """Expected frames to the FIRST RUN of k consecutive decodes (ADR-0079).

    E[T] = (1 - p^k) / (p^k * (1 - p)). This REPLACED the mean-rate model (k/p)
    on 2026-07-24: the handoff needs k CONSECUTIVE detections, and mean-rate
    always UNDERSTATES the burn -- the optimistic, i.e. dangerous, direction for
    a purchase gate (up to 9.25x at p=0.44; ~1.2x in this gate's p>=0.9 regime).
    Must stay identical to tripod_score._streak_burn_frames.
    """
    if p >= 1.0:
        return float(k)
    if p <= 0.0:
        return float("inf")
    pk = p ** k
    return (1.0 - pk) / (pk * (1.0 - p))


def tripod_gate(r90_real, decode_rate, v=V_CLOSING_MPS, stream_fps=STREAM_FPS):
    """tripod_score.py gate_verdict arithmetic (ADR-0079 run-length burn).

    CORRECTED 2026-07-24 (ADR-0082): this used the mean-rate burn
    (HANDOFF_STREAK / decode_Hz) at a hardcoded 30 fps -- the exact model
    ADR-0079 rejected in tripod_score.py. Two different burn models in one
    codebase is the drift that manufactures mirages, and THIS function produced
    the published placard-sizing threshold. Now it mirrors tripod_score exactly
    and takes the frame rate as a parameter.
    """
    e_frames = _streak_frames(decode_rate)
    decode_hz = decode_rate * stream_fps
    r_streak_burn = ((e_frames / stream_fps) * v
                     if (stream_fps > 0 and math.isfinite(e_frames)) else float("inf"))
    t_go = (r90_real - r_streak_burn) / v if v > 0 else float("-inf")
    verdict = "PASS" if (r90_real > 0 and t_go >= TGO_MIN_S) else "FAIL"
    return dict(verdict=verdict,
                t_go=round(t_go, 3) if math.isfinite(t_go) else None,
                r_streak_burn=round(r_streak_burn, 3) if math.isfinite(r_streak_burn) else None,
                decode_hz=round(decode_hz, 2),
                stream_fps=stream_fps,
                e_streak_frames=round(e_frames, 2) if math.isfinite(e_frames) else None,
                burn_model="run-length (ADR-0079)")


def plot_placard(pl, images_dir, plt):
    edges = [c["edge_m"] for c in pl["candidates"]]
    r_cons = [c["R_decode90_real_conservative_m"] for c in pl["candidates"]]
    r_mid = [c["R_decode90_real_middle_m"] for c in pl["candidates"]]
    r_gate = pl["gate_R90_threshold_m"]
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    ax.plot(edges, r_cons, "o-", color="#d62728", lw=2,
            label="qd=2.0 conservative (fx-ratio ×0.71)")
    ax.plot(edges, r_mid, "s--", color="#1f77b4", lw=2,
            label="qd=2.0 middle (avg px/deg ×0.85, camera_paper_check)")
    if "R_decode90_sim_hires_m" in pl:
        r_h_cons = [c.get("R_decode90_real_hires_conservative_m") for c in pl["candidates"]]
        r_h_mid = [c.get("R_decode90_real_hires_middle_m") for c in pl["candidates"]]
        ax.plot(edges, r_h_cons, "o:", color="#ff7f0e", lw=1.6, alpha=0.9,
                label="qd=1.0 conservative (full-res Pi decode)")
        ax.plot(edges, r_h_mid, "s:", color="#17becf", lw=1.6, alpha=0.9,
                label="qd=1.0 middle")
    ax.axhline(r_gate, ls="--", color="#2ca02c", lw=1.6,
               label=f"money gate: R90 >= {r_gate:.1f} m (t_go>=0.5s @9m/s)")
    ax.axvline(CARRY_LIMIT_M, ls=":", color="#888", lw=1, label="carry limit 0.35 m")
    rec = pl["recommended_edge_m"]
    rec_c = next(c for c in pl["candidates"] if c["edge_m"] == rec)
    ax.scatter([rec], [rec_c["R_decode90_real_middle_m"]], s=200, facecolors="none",
               edgecolors="#000", lw=2.2, zorder=5, label=f"RECOMMENDED {rec:.2f} m")
    ax.set_xlabel("real placard edge (AprilTag black-square, m)")
    ax.set_ylabel("predicted real decode range  R_decode90 (m)")
    ax.set_title("Placard sizing: sim decode envelope scaled to the OV9281")
    ax.set_xticks(edges)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7.5, loc="upper left")
    fig.tight_layout()
    p = os.path.join(images_dir, "placard_sizing.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep", action="store_true", help="capture (needs a running apriltag-world sim)")
    ap.add_argument("--analyze", metavar="CSV", help="offline: curves + placard math from a sweep CSV")
    ap.add_argument("--out", default=os.path.join(HERE, "data", "autolabel_sim"),
                    help="sweep frame output dir")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    ap.add_argument("--csv", default=os.path.join(REPO, "logs", f"autolabel_sim_{stamp}.csv"))
    ap.add_argument("--ranges", type=float, nargs="+", default=None)
    ap.add_argument("--images-dir", default=os.path.join(REPO, "docs", "images"))
    ap.add_argument("--summary-out", default=None)
    ap.add_argument("--sim-fx", type=float, default=539.9363327026367,
                    help="sim camera fx (px) for the placard transfer; sweep overrides via CameraInfo")
    ap.add_argument("--r90-hires", type=float, default=None,
                    help="sim R_decode90 (m) measured at quad_decimate=1.0 (full-res) for the "
                         "detector-config sensitivity overlay; deployed default is 2.0")
    args = ap.parse_args()

    if args.sweep:
        return run_sweep(args)
    if args.analyze:
        return analyze(args)
    ap.error("need --sweep or --analyze")


if __name__ == "__main__":
    sys.exit(main())
