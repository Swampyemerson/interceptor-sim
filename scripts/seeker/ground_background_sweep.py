#!/usr/bin/env python3
"""PROBE 1 -- ground-clutter-background recall sweep (headline next_probe, the
first of the TWO un-eliminated in-view detection mechanisms; detector stage note
+ docs/project_state.json headline.next_probe).

WHY THIS EXISTS
---------------
The frame-top sweep (P0.1, setpose_frametop_sweep.sh) proved the deployed detector
localizes the nose-on / banked target at ~100% at the frame-TOP edge, 8-22 m --
BUT it viewed the target's belly against a clean gray SKY. The fixed scorer
(box_scoring.py) then showed most of the "frame-top blindness" was a scoring
artifact. That leaves TWO in-view mechanisms un-eliminated; this probe attacks the
first: GROUND-CLUTTER BACKGROUND. The deployed detector read 0% on the horizon
line vs 76-100% on sky at the SAME range in flight, and under the ~35-40 deg
nose-down dash pitch the real terminal scene puts the nose-on target against
GROUND (with its cast shadow), not sky.

THE GEOMETRY (why a stable ELEVATED hover, not a ground set-pose)
----------------------------------------------------------------
To silhouette a 6-20 m target against GROUND you need a DOWNWARD line of sight --
the camera must sit ABOVE the target. A disarmed interceptor on the ground
(z~0.229 m, the setpose/frametop sweeps' pose) can only place a distant target
within ~1 deg of the horizon before it goes underground, and set_pose-HOLDING the
dynamic airframe at altitude oscillates it +/-1.6-3 m (measured) -- unusable. So
Probe 1 flies the interceptor to a real PX4 HOLD hover (stable to cm, low velocity)
and teleports the STATIC target through a range x background x azimuth x attitude
grid BELOW / AT / ABOVE the level boresight:

  background  camera-frame elevation phi   what's behind the target
  ----------  ---------------------------  ---------------------------------
  ground      phi < 0 (target below cam)   the gray ground plane + cast shadow
  horizon     phi = 0 (co-altitude)        the horizon line (the note's "0%")
  sky         phi > 0 (target above cam)   the gray sky  (reproduces frametop)

A level camera viewing a target at depression |phi| is optically equivalent -- for
the detector's appearance / background / top-down-aspect -- to a camera PITCHED
DOWN |phi| viewing a co-elevation target; they differ ONLY in the target's frame
POSITION, which the frame-top sweep already proved does not impair detection. This
sidesteps the SDF-sensor-tilt gt chain (INTERCEPTOR_SENSOR_PITCH_RAD), whose
env-injected correction showed a residual gt-vs-render misalignment in bench tests
here; the LEVEL-camera native gt chain (m2_detect, env unset) is the exact,
per-frame-synced chain the validated setpose/frametop 'centered' control uses, so
the gt boxes land on the rendered target and box_scoring can be trusted.

HONESTY / CAVEATS (state them in the doc)
-----------------------------------------
* gt (`gt_*`) is SCORING/label-source ONLY (CLAUDE.md boundary); the mover issues
  set_pose on the STATIC target, never reads gt into any guidance path.
* The sim "ground" is a near-uniform gray plane (0.8) vs a gray sky (0.7) plus the
  target's hard cast SHADOW and the horizon seam -- NOT real outdoor terrain
  clutter. So this probe tests the sim's ground/shadow/horizon-seam background
  sensitivity, which BOUNDS but does not equal the real outdoor-appearance gap
  (that stays a real-data-capture question, R5/R6).
* GPU render is ON (sim_gpu_render.sh). The known GPU-render caveat corrupts
  FIDUCIAL (AprilTag) decode only; this probe is markerless NN inference with NO
  tags, so GPU render is fine.
* Static set-pose captures are not timing-sensitive, so the ADR-0015 machine-load
  confound does not bind (validate_autolabel_sim's disclosure) -- but rerun at idle
  if render anomalies appear.

MODES
-----
  --takeoff H          arm + takeoff to H m, then HOLD until SIGTERM (needs sim).
                       Disables the low-battery action so a multi-minute hover
                       doesn't failsafe-land mid-sweep. Flushed prints.
  --sweep --background {sky,horizon,ground} --attitude {level,banked}
                       teleport the target through the range x azimuth grid at the
                       given background/attitude, riding along a
                       capture_flight_frames.py recorder (needs sim + a hover).
  --analyze ROOT       score every background subdir with box_scoring (ALL boxes
                       AND top-1), emit recall vs range x background CSV + a plot,
                       and print the verdict comparison vs the frame-top SKY sweep
                       (no sim).
  --self-test          exercise the placement geometry + background classification
                       + quaternion, no sim.

Run under .venv for --takeoff/--sweep (mavsdk / gz-transport) and --analyze glue;
--analyze/-scoring imports box_scoring (pure). The detector re-score inside
--analyze needs onnxruntime -> run --analyze under .venv-seeker.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
REPO = os.path.dirname(SCRIPTS)
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, HERE)

FX = FY = 539.9363327026367
CX, CY = 640.0, 480.0
W_FULL, H_FULL = 1280, 960

# background -> camera-frame elevation phi (deg, +up). ground is a DEPRESSED LOS.
BACKGROUND_PHI = {"sky": +15.0, "horizon": 0.0, "ground": -22.0}
# two azimuths per range: on-boresight and off-axis (deg, camera-frame +left).
AZIMUTHS = {"on": 0.0, "off": 16.0}
DEFAULT_RANGES = [6.0, 8.0, 10.0, 12.0, 16.0, 20.0]
BANK_ROLL_DEG = 30.0            # matches setpose_frametop_sweep 'frametop_banked'
HOLD_S = 6.0                    # sim-seconds per placement (~180 frames @30fps)


# ==========================================================================
# geometry (pure -- exercised by --self-test)
# ==========================================================================
def cam_dir(phi_deg: float, az_deg: float) -> np.ndarray:
    """Unit direction in the gz SENSOR/camera_link frame (x-fwd, y-left, z-up)
    at bearing az (>0 left) and elevation phi (>0 up)."""
    phi, az = math.radians(phi_deg), math.radians(az_deg)
    return np.array([math.cos(phi) * math.cos(az),
                     math.cos(phi) * math.sin(az),
                     math.sin(phi)])


def target_world_pos(C, R_cam, range_m, phi_deg, az_deg):
    """Target world position at range `range_m` (scalar, m), camera-frame
    (phi, az), given the camera world position C and its world rotation R_cam
    (columns = sensor axes in world)."""
    return np.asarray(C) + range_m * (np.asarray(R_cam) @ cam_dir(phi_deg, az_deg))


def face_camera_quat(T, C, roll_deg=0.0):
    """Quaternion (x,y,z,w) that yaws the target's +x nose toward the camera
    (nose-on approach aspect) with an optional roll (bank) about that nose axis.
    World is ENU (z up)."""
    dx, dy = float(C[0] - T[0]), float(C[1] - T[1])
    yaw = math.atan2(dy, dx)          # nose points from target to camera
    roll = math.radians(roll_deg)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    # yaw about world +z, then roll about the (yawed) body +x:  q = qz * qx
    # qz = (0,0,sy,cy); qx = (sr,0,0,cr)
    w = cy * cr
    x = cy * sr
    y = sy * sr
    z = sy * cr
    return (x, y, z, w)


def _deployed_top1(boxes, max_bearing_rad=math.radians(30.0),
                   edge_margin_px=40.0, W=W_FULL):
    """The (score,u,v,bw,bh) box finetuned_seeker.detect() would return: highest
    score among self-mask survivors (|bearing|<=30 deg, no L/R edge touch), else
    None. Kept here so --analyze infers ONCE per frame instead of calling detect()
    (which re-runs _infer_boxes)."""
    best = None
    for bx in boxes:
        _s, u, _v, bw, _bh = bx
        if abs(math.atan2((u - CX) / FX, 1.0)) > max_bearing_rad:
            continue
        if (u - bw / 2) < edge_margin_px or (u + bw / 2) > (W - edge_margin_px):
            continue
        if best is None or bx[0] > best[0]:
            best = bx
    return best


def classify_background(phi_deg: float) -> str:
    if phi_deg > 3.0:
        return "sky"
    if phi_deg < -3.0:
        return "ground"
    return "horizon"


def _self_test() -> int:
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and cond
        print(f"[self-test] {name}: {'PASS' if cond else 'FAIL'}")

    # cam_dir: on-boresight level -> +x forward
    d = cam_dir(0.0, 0.0)
    check("cam_dir on-boresight = +x", np.allclose(d, [1, 0, 0], atol=1e-9))
    # elevation up -> +z component; depression -> -z
    check("cam_dir sky phi>0 -> +z up", cam_dir(15, 0)[2] > 0)
    check("cam_dir ground phi<0 -> -z down", cam_dir(-22, 0)[2] < 0)
    check("cam_dir off-axis az>0 -> +y left", cam_dir(0, 16)[1] > 0)
    check("cam_dir is unit", abs(np.linalg.norm(cam_dir(-22, 16)) - 1) < 1e-9)

    # target placement: level camera at origin looking +x, ground target is BELOW
    C = np.array([0.0, 0.0, 9.0]); R_cam = np.eye(3)
    Tg = target_world_pos(C, R_cam, 1.0, -22.0, 0.0)
    check("ground target below camera (z<9)", bool(Tg[2] < 9.0))
    check("ground target range ~= R", abs(np.linalg.norm(Tg - C) - 1.0) < 1e-9)
    Ts = target_world_pos(C, R_cam, 1.0, +15.0, 0.0)
    check("sky target above camera (z>9)", bool(Ts[2] > 9.0))

    # ground target renders BELOW image centre (optical y down), sky ABOVE
    def opt_v(T):
        rel = T - C  # camera at origin looking +x, sensor->optical permutation
        x, y, z = -rel[1], -rel[2], rel[0]
        return CY + FY * y / z
    check("ground target v > CY (lower frame)", bool(opt_v(Tg) > CY))
    check("sky target v < CY (upper frame)", bool(opt_v(Ts) < CY))

    # background classification round-trips
    check("classify sky", classify_background(15) == "sky")
    check("classify horizon", classify_background(0) == "horizon")
    check("classify ground", classify_background(-22) == "ground")

    # face_camera_quat: unit; on-axis target ahead (+x) faces -x (yaw 180)
    q = face_camera_quat(np.array([12.0, 0, 5]), np.array([0.0, 0, 9]), 0.0)
    check("quat is unit", abs(math.sqrt(sum(c * c for c in q)) - 1) < 1e-9)
    # banked quat differs from level
    qb = face_camera_quat(np.array([12.0, 0, 5]), np.array([0.0, 0, 9]), 30.0)
    check("banked quat != level quat", not np.allclose(q, qb))

    print(f"[self-test] {'ALL PASS' if ok else 'SOME FAILED'}")
    return 0 if ok else 1


# ==========================================================================
# --takeoff H  (needs sim; holds until SIGTERM)
# ==========================================================================
def run_takeoff(H: float) -> int:
    import asyncio
    from mavsdk import System

    async def _main():
        d = System()
        print(f"[takeoff] connecting udpin://0.0.0.0:14540 ...", flush=True)
        await d.connect(system_address="udpin://0.0.0.0:14540")

        async def _conn():
            async for c in d.core.connection_state():
                if c.is_connected:
                    return
        await asyncio.wait_for(_conn(), 90)
        print("[takeoff] connected", flush=True)

        async def _health():
            async for h in d.telemetry.health():
                if h.is_global_position_ok and h.is_home_position_ok:
                    return
        await asyncio.wait_for(_health(), 120)
        print("[takeoff] health ok", flush=True)

        # keep a multi-minute hover from failsafe-landing on the SITL battery
        for pname in ("COM_LOW_BAT_ACT", "COM_LOW_BAT_ACT"):
            try:
                await d.param.set_param_int(pname, 0)
            except Exception as exc:  # noqa: BLE001
                print(f"[takeoff] warn: param {pname}: {exc}", flush=True)

        st = {"alt": None}

        async def _pos():
            async for p in d.telemetry.position():
                st["alt"] = p.relative_altitude_m
        asyncio.create_task(_pos())

        await d.action.set_takeoff_altitude(H)
        await d.action.arm()
        print(f"[takeoff] armed; taking off to {H} m", flush=True)
        await d.action.takeoff()
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if st["alt"] is not None and st["alt"] >= 0.85 * H:
                break
            await asyncio.sleep(0.5)
        print(f"[takeoff] HOLDING at alt={st['alt']} m (SIGTERM to land)", flush=True)
        # hold forever; the orchestrator SIGTERMs us when the sweep is done
        while True:
            await asyncio.sleep(1.0)

    try:
        return asyncio.run(_main())
    except KeyboardInterrupt:
        return 0


# ==========================================================================
# --sweep  (needs sim + a running hover + a capture_flight_frames recorder)
# ==========================================================================
_MOVER_SRC = r"""
import sys
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
node = Node()
for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    name, x, y, z, qx, qy, qz, qw = line.split(",")
    req = Pose(); req.name = name
    req.position.x = float(x); req.position.y = float(y); req.position.z = float(z)
    req.orientation.x = float(qx); req.orientation.y = float(qy)
    req.orientation.z = float(qz); req.orientation.w = float(qw)
    ok, resp = node.request("/world/%WORLD%/set_pose", req, Pose, Boolean, 3000)
    print("OK" if (ok and resp.data) else "FAIL", flush=True)
"""


def run_sweep(args) -> int:
    import subprocess
    os.environ.setdefault("INTERCEPTOR_WORLD_NAME", args.world)
    os.environ.setdefault("INTERCEPTOR_TARGET_MODEL", args.target)
    from m2_detect import (POSE_TOPIC, PoseTracker, DRONE_MODEL,
                           CAMERA_LINK_NAME, compose)
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V

    phi = BACKGROUND_PHI[args.background]
    roll = BANK_ROLL_DEG if args.attitude == "banked" else 0.0
    ranges = args.ranges or DEFAULT_RANGES

    node = Node()
    tr = PoseTracker()
    if not node.subscribe(Pose_V, POSE_TOPIC, tr.on_pose_v):
        print("[sweep] FAIL subscribe pose"); return 2
    # wait for the hovering interceptor pose
    t0 = time.time()
    while (DRONE_MODEL not in tr.latest or CAMERA_LINK_NAME not in tr.latest) \
            and time.time() - t0 < 30:
        time.sleep(0.2)
    if DRONE_MODEL not in tr.latest:
        print("[sweep] FAIL: no interceptor pose"); return 2

    mover = subprocess.Popen(
        [sys.executable, "-c", _MOVER_SRC.replace("%WORLD%", args.world)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, bufsize=1)

    def set_pose(name, T, q):
        mover.stdin.write(f"{name},{T[0]:.4f},{T[1]:.4f},{T[2]:.4f},"
                          f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}\n")
        mover.stdin.flush()
        dl = time.monotonic() + 5
        while time.monotonic() < dl:
            ln = mover.stdout.readline()
            if ln:
                return ln.strip() == "OK"
        return False

    def cam_pose():
        p_model, R_model = tr.latest[DRONE_MODEL]
        p_cr, R_cr = tr.latest[CAMERA_LINK_NAME]
        return compose(p_model, R_model, p_cr, R_cr)  # -> (p_cam_world, R_cam_world)

    print(f"[sweep] background={args.background} phi={phi} attitude={args.attitude} "
          f"roll={roll} ranges={ranges}")
    try:
        for R in ranges:
            for axis, az in AZIMUTHS.items():
                C, R_cam = cam_pose()
                T = np.asarray(C) + R * (R_cam @ cam_dir(phi, az))
                q = face_camera_quat(T, C, roll)
                ok = set_pose(args.target, T, q)
                gtr = float(np.linalg.norm(T - np.asarray(C)))
                print(f"[sweep]   R={R:>4} {axis:>3}(az={az:+.0f}) -> "
                      f"T=({T[0]:.2f},{T[1]:.2f},{T[2]:.2f}) gtr~{gtr:.1f} "
                      f"{'ok' if ok else 'SETFAIL'}")
                time.sleep(HOLD_S)
    finally:
        try:
            mover.stdin.close()
        except Exception:
            pass
        mover.terminate()
        node.unsubscribe(POSE_TOPIC)
    return 0


# ==========================================================================
# --analyze ROOT  (no sim; scores with box_scoring, ALL boxes + top-1)
# ==========================================================================
def run_analyze(args) -> int:
    import csv
    import glob
    import json
    import re
    from collections import defaultdict
    import cv2
    from finetuned_seeker import FinetunedNNSeeker
    from box_scoring import box_hits_gt, gt_scale_for, TARGET_EXTENT_M

    V2 = os.path.join(HERE, "weights", "drone_finetuned_quad_v2.onnx")
    if not os.path.exists(V2):
        print(f"[analyze] detector weights missing: {V2}"); return 1
    det = FinetunedNNSeeker(FX, FY, CX, CY, V2, conf_thres=args.conf)
    RANGE_RE = re.compile(r"_r(\d+\.?\d*)_")

    def gt_range_of(name):
        m = RANGE_RE.search(name)
        return float(m.group(1)) if m else None

    def load_gt_box(lp):
        try:
            parts = open(lp).readline().split()
        except OSError:
            return None
        if len(parts) < 5:
            return None
        _, cx, cy, w, h = (float(v) for v in parts[:5])
        return cx * W_FULL, cy * H_FULL, w * W_FULL, h * H_FULL

    backgrounds = [b for b in ("sky", "horizon", "ground")
                   if os.path.isdir(os.path.join(args.analyze, b, "images"))]
    if not backgrounds:
        print(f"[analyze] no sky/horizon/ground subdirs under {args.analyze}")
        return 1

    os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
    pf = open(args.csv, "w", newline="")
    wr = csv.writer(pf)
    wr.writerow(["background", "attitude_axis", "frame", "gt_range", "gcx", "gcy",
                 "gbearing_deg", "n_boxes", "any_hit", "top1_hit",
                 "target_hit_rank", "top1_is_phantom"])

    # nested: bg -> range_bin -> {"any":[h,t], "top1":[h,t]}, split on/off axis
    agg = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: [0, 0])))
    gscale = gt_scale_for(args.capture_extent_m)

    for bg in backgrounds:
        bdir = os.path.join(args.analyze, bg)
        imgs = sorted(glob.glob(os.path.join(bdir, "images", "*.png")))
        lbl_dir = os.path.join(bdir, "labels")
        for ip in imgs:
            base = os.path.splitext(os.path.basename(ip))[0]
            gr = gt_range_of(base)
            if gr is None or not (args.rmin <= gr <= args.rmax):
                continue
            gt = load_gt_box(os.path.join(lbl_dir, base + ".txt"))
            if gt is None:
                continue  # negative frame (target not in view) -> skip
            frame = cv2.imread(ip)
            if frame is None:
                continue
            gcx, gcy, gw, gh = gt
            gbearing = math.degrees(math.atan2((gcx - CX) / FX, 1.0))
            boxes = det._infer_boxes(frame)
            sk = dict(gt_scale=gscale, offaxis_aware=True)
            # ALL boxes: any box that hits gt
            any_hit = box_hits_gt(boxes, gcx, gcy, gw, gh, **sk)
            # rank of the best-scoring target-hitting box among all boxes
            ordered = sorted(boxes, key=lambda r: -r[0])
            rank = None
            for i, bx in enumerate(ordered):
                if box_hits_gt([bx], gcx, gcy, gw, gh, **sk):
                    rank = i + 1
                    break
            # DEPLOYED top-1: replicate detect()'s self-mask (|bearing|<=30 deg,
            # no L/R frame-edge touch) then top score, over the SAME boxes -- so
            # we infer ONCE per frame (byte-equivalent selection to detect()).
            dep = _deployed_top1(boxes)
            top1_hit = dep is not None and box_hits_gt([dep], gcx, gcy, gw, gh, **sk)
            top1_phantom = dep is not None and not top1_hit
            axis = "off" if abs(gbearing) > 8.0 else "on"
            b = int(gr // 2) * 2
            agg[bg][b]["any"][0] += int(any_hit); agg[bg][b]["any"][1] += 1
            agg[bg][b]["top1"][0] += int(top1_hit); agg[bg][b]["top1"][1] += 1
            agg[bg][b][f"any_{axis}"][0] += int(any_hit)
            agg[bg][b][f"any_{axis}"][1] += 1
            wr.writerow([bg, axis, base, f"{gr:.1f}", f"{gcx:.1f}", f"{gcy:.1f}",
                         f"{gbearing:+.1f}", len(boxes), int(any_hit),
                         int(top1_hit), rank if rank else "",
                         int(top1_phantom)])
    pf.close()

    # ---- print recall vs range x background (ALL boxes + top-1) --------------
    print(f"\n=== PROBE 1: recall vs range x background "
          f"(conf {args.conf}, unified extent {TARGET_EXTENT_M} m) ===")
    print("ALL-boxes recall (any box hits gt) | top-1 deployed recall (detect())")
    summary = {"conf": args.conf, "backgrounds": {}}
    for bg in backgrounds:
        print(f"\n  --- {bg.upper()} ---")
        print(f"    {'range':<9}{'ALL':>14}{'top-1':>14}")
        aA = [0, 0]; aT = [0, 0]
        for b in sorted(agg[bg]):
            ah, at_ = agg[bg][b]["any"]
            th, tt = agg[bg][b]["top1"]
            aA[0] += ah; aA[1] += at_; aT[0] += th; aT[1] += tt
            print(f"    {f'{b}-{b+2} m':<9}"
                  f"{f'{100*ah/at_:.0f}% ({ah}/{at_})':>14}"
                  f"{f'{100*th/tt:.0f}% ({th}/{tt})':>14}")
        aall = 100 * aA[0] / aA[1] if aA[1] else 0
        tall = 100 * aT[0] / aT[1] if aT[1] else 0
        print(f"    {'ALL':<9}{f'{aall:.0f}% ({aA[0]}/{aA[1]})':>14}"
              f"{f'{tall:.0f}% ({aT[0]}/{aT[1]})':>14}")
        summary["backgrounds"][bg] = dict(
            all_boxes_recall=round(aall, 1), top1_recall=round(tall, 1),
            n=aA[1], all_hits=aA[0], top1_hits=aT[0])

    if args.summary_out:
        with open(args.summary_out, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"\n[analyze] summary -> {args.summary_out}")
    print(f"[analyze] per-frame -> {args.csv}")
    print("[analyze] (plot: run under .venv with --plot --level-csv <csv> "
          "[--banked-csv <csv>] -- matplotlib lives in .venv, onnx in .venv-seeker)")
    return 0


# ==========================================================================
# --plot  (no sim, no onnx; matplotlib -> run under .venv)
# ==========================================================================
def _recall_from_csv(path):
    """per-frame CSV -> {background: {range_bin: {'any':[h,t],'top1':[h,t]}}}."""
    import csv
    from collections import defaultdict
    agg = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: [0, 0])))
    if not path or not os.path.exists(path):
        return agg
    for r in csv.DictReader(open(path)):
        bg = r["background"]
        b = int(float(r["gt_range"]) // 2) * 2
        agg[bg][b]["any"][0] += int(r["any_hit"]); agg[bg][b]["any"][1] += 1
        agg[bg][b]["top1"][0] += int(r["top1_hit"]); agg[bg][b]["top1"][1] += 1
    return agg


def run_plot(args) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(args.images_dir, exist_ok=True)
    colors = {"sky": "#1f77b4", "horizon": "#ff7f0e", "ground": "#d62728"}
    panels = [("level", args.level_csv)]
    if args.banked_csv and os.path.exists(args.banked_csv):
        panels.append(("banked", args.banked_csv))
    fig, axes = plt.subplots(len(panels), 2, figsize=(12, 4.6 * len(panels)),
                             squeeze=False)
    for row, (att, csvp) in enumerate(panels):
        agg = _recall_from_csv(csvp)
        axA, axT = axes[row][0], axes[row][1]
        for bg in ("sky", "horizon", "ground"):
            if bg not in agg:
                continue
            bins = sorted(agg[bg])
            centers = [b + 1 for b in bins]
            rA = [100 * agg[bg][b]["any"][0] / agg[bg][b]["any"][1]
                  if agg[bg][b]["any"][1] else 0 for b in bins]
            rT = [100 * agg[bg][b]["top1"][0] / agg[bg][b]["top1"][1]
                  if agg[bg][b]["top1"][1] else 0 for b in bins]
            axA.plot(centers, rA, "o-", color=colors[bg], lw=2, label=bg)
            axT.plot(centers, rT, "s--", color=colors[bg], lw=2, label=bg)
        for ax, ttl in ((axA, f"{att}: ALL boxes (any hits gt)"),
                        (axT, f"{att}: top-1 deployed (detect())")):
            ax.set_xlabel("ground-truth range (m)")
            ax.set_ylim(-5, 105)
            ax.set_title(ttl)
            ax.grid(alpha=0.3)
            ax.legend(fontsize=9)
        axA.set_ylabel("recall (%)")
    fig.suptitle(f"Probe 1: markerless recall vs range x background (v2, conf {args.conf})")
    fig.tight_layout()
    p = os.path.join(args.images_dir, "probe1_ground_background_recall.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"[plot] -> {p}")
    return 0


# ==========================================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--takeoff", type=float, metavar="H",
                    help="arm+takeoff to H m and HOLD until SIGTERM (needs sim)")
    ap.add_argument("--sweep", action="store_true",
                    help="teleport the target through the grid (needs sim+hover)")
    ap.add_argument("--analyze", metavar="ROOT",
                    help="score sky/horizon/ground subdirs under ROOT (no sim)")
    ap.add_argument("--plot", action="store_true",
                    help="plot recall-vs-range x background from per-frame CSVs "
                         "(no sim/onnx; run under .venv for matplotlib)")
    ap.add_argument("--level-csv", default=None)
    ap.add_argument("--banked-csv", default=None)
    ap.add_argument("--background", choices=list(BACKGROUND_PHI))
    ap.add_argument("--attitude", choices=["level", "banked"], default="level")
    ap.add_argument("--ranges", type=float, nargs="+", default=None)
    ap.add_argument("--world", default="quad_enemy")
    ap.add_argument("--target", default="fpv_quad_enemy")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--rmin", type=float, default=5.0)
    ap.add_argument("--rmax", type=float, default=22.0)
    ap.add_argument("--capture-extent-m", type=float, default=0.9,
                    help="the --extent-m capture_flight_frames labelled at")
    ap.add_argument("--csv", default=os.path.join(REPO, "logs",
                    "ground_background_sweep.csv"))
    ap.add_argument("--images-dir", default=os.path.join(REPO, "docs", "images"))
    ap.add_argument("--summary-out", default=None)
    args = ap.parse_args()

    if args.self_test:
        return _self_test()
    if args.takeoff is not None:
        return run_takeoff(args.takeoff)
    if args.sweep:
        if not args.background:
            ap.error("--sweep needs --background")
        return run_sweep(args)
    if args.analyze:
        return run_analyze(args)
    if args.plot:
        if not args.level_csv:
            ap.error("--plot needs --level-csv")
        return run_plot(args)
    ap.error("need --self-test / --takeoff / --sweep / --analyze / --plot")


if __name__ == "__main__":
    sys.exit(main())
