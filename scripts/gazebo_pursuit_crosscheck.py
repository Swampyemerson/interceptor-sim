#!/usr/bin/env python3
"""Gazebo camera-in-the-loop cross-check of the ADR-0103 chase-only PURSUIT terminal.

WHAT: drives the UNMODIFIED flight code (`flight.deploy.real_flight.run_mavsdk_mission`,
`--terminal pursuit`) against a live PX4 SITL + Gazebo, with detections from the REAL
simulated mono camera (AprilTag tag36h11, 0.5 m black square). Precedent for driving the
flight code from outside: isim/flight_adapter.py. Nothing under flight/ is edited or
imported into it from here (its AST audit forbids gz imports in its own closure).

BOOT SEQUENCE the head runs (one sim at a time, idle machine):
  1. boot the apriltag world headless (px4-gazebo skill), e.g. in ~/PX4-Autopilot:
       HEADLESS=1 PX4_GZ_WORLD=apriltag make px4_sitl gz_x500_mono_cam
     (with scripts/sim_gpu_render.sh sourced, as every boot script does)
  2. nothing to place by hand: the mover's pre-warm request places the tag at the
     track start, and its board is always identity-oriented (see GEOMETRY)
  3. .venv/bin/python scripts/gazebo_pursuit_crosscheck.py --out logs/xcheck_01.csv
     (extra unknown flags are forwarded verbatim to real_flight's own parser,
      e.g. --standby-alt-m 7 --engage-max-s 12)
Offline: .venv/bin/python scripts/gazebo_pursuit_crosscheck.py --self-test

GEOMETRY: the isim crossing scenario (isim/scenario.py: target starts --lead m before the
abeam point on a track --cross-range m from launch, --speed m/s) ROTATED 90 deg so the
target flies world +X (east). Why rotated: scripts/m4_target_mover.py sends position-only
set_pose requests, which reset the board to IDENTITY every tick, and the identity board
faces world -X (models/apriltag_target/model.sdf). So a +X track is the only rear-facing
(face normal anti-parallel to velocity) track the unmodified mover can fly; a pre-run
set_pose yaw would be overwritten on the mover's first tick -> --tag-yaw-deg !=0 refused.
World ENU <-> PX4 NED: north=world_y, east=world_x, origin = vehicle spawn (m4 docstring).

CLOCKS: `t_capture_sim` = the gz Image header stamp (sim time the frame was rendered) --
logged only, because the driver stamps every detection with its own loop time (the
interface carries no capture time). By default the driver's `time` module is swapped for a
/clock-backed shim (--wall-clock-driver disables) so the terminal's filter dt, its engage
bounds and the mover's schedule share SIM time (standing rule: sim time, never wall time).

HONESTY: the pose/info subscription (vehicle + tag world poses) is SCORING/LOGGING ONLY; it
never reaches the driver. Guidance sees camera boxes + the MAVSDK own-state EKF.
"""
import argparse
import asyncio
import csv
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
import types

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _REPO)
sys.path.insert(0, os.path.join(_REPO, "scripts"))

import flight.deploy.real_flight as rf  # noqa: E402

TAG_SIZE_M = 0.5            # black square, models/apriltag_target/model.sdf
TAG_FAMILY = "tag36h11"
TAG_MODEL = "apriltag_target"
VEHICLE_MODEL = "x500_mono_cam_0"
TAG_PNG = os.path.join(_REPO, "models", "apriltag_target", "tag36h11_00000.png")
MOVER = os.path.join(_REPO, "scripts", "m4_target_mover.py")
POSE_SAMPLE_MIN_DT = 0.005  # s sim; CPA interpolates linearly between kept samples


def image_msg_to_gray(msg):
    """RGB_INT8, no row padding -> uint8 gray. Same contract as scripts/m2_detect.py
    (copied, not imported: m2_detect pulls its ground-truth tracker in with it)."""
    import cv2
    from gz.msgs10.image_pb2 import RGB_INT8
    if msg.pixel_format_type != RGB_INT8:
        raise ValueError(f"unexpected pixel_format_type {msg.pixel_format_type}")
    if msg.step != msg.width * 3:
        raise ValueError(f"unexpected step {msg.step} for width {msg.width} (row padding?)")
    rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


def tag_to_detection(dets, fx, span_m, t_capture_sim):
    """AprilTag results -> the object `run_mavsdk_mission` consumes. The driver reads
    ONLY `.range_m` and `.box_xywh` (SmokeSeeker's SimpleNamespace shape); the SM passes
    the box to the terminal only when range_m is not None, so a miss is range=None.
    range_m = fx*span/box_w -- the SAME known-size conversion the terminal applies to the
    box, so the SM's range-keyed guards and the terminal agree. NOTE it is DEPTH along the
    optical axis, not slant range: ~14% short of |pose_t| at ~30 deg off-axis (self-test
    measures it). Extra attrs (t_capture_sim, tag_range_m = slant |pose_t|) are logs."""
    hits = [d for d in dets if d.tag_id == 0]
    if not hits:
        return types.SimpleNamespace(range_m=None, box_xywh=None,
                                     t_capture_sim=t_capture_sim, tag_range_m=None)
    d = hits[0]
    xs, ys = d.corners[:, 0], d.corners[:, 1]
    x0, y0, w, h = float(xs.min()), float(ys.min()), float(np.ptp(xs)), float(np.ptp(ys))
    pose_r = (float(np.linalg.norm(d.pose_t)) if getattr(d, "pose_t", None) is not None
              else None)
    return types.SimpleNamespace(range_m=fx * span_m / max(w, 1e-6),
                                 box_xywh=(x0, y0, w, h),
                                 t_capture_sim=t_capture_sim, tag_range_m=pose_r)


class GazeboTagDetector:
    """`detect(frame, t)` duck-type for run_mavsdk_mission. The driver hands in a blank
    array (its smoke frame); we ignore it and decode the NEWEST gz frame. A frame already
    consumed is a MISS, never re-reported (a repeat would feed the filter one measurement
    twice at two different times)."""

    def __init__(self, fx, fy, cx, cy, span_m=TAG_SIZE_M):
        from apriltag_detector import Detector
        self.det = Detector(families=TAG_FAMILY)
        self.cam = (fx, fy, cx, cy)
        self.span = span_m
        self.latest = None          # newest gz Image msg (callback thread assigns)
        self._last_stamp = None
        self.n_frames = self.n_consumed = self.n_stale = 0
        self.last = None

    def on_image(self, msg):
        self.latest = msg
        self.n_frames += 1

    def detect_msg(self, msg):
        gray = image_msg_to_gray(msg)
        dets = self.det.detect(gray, estimate_tag_pose=True, camera_params=self.cam,
                               tag_size=self.span)
        return tag_to_detection(dets, self.cam[0], self.span, stamp_s(msg))

    def detect(self, _frame, _t=None):
        msg = self.latest
        if msg is None or stamp_s(msg) == self._last_stamp:
            self.n_stale += 1
            self.last = types.SimpleNamespace(range_m=None, box_xywh=None,
                                              t_capture_sim=None, tag_range_m=None)
            return self.last
        self._last_stamp = stamp_s(msg)
        self.last = self.detect_msg(msg)
        self.n_consumed += self.last.range_m is not None
        return self.last


class SimWorld:
    """/clock + /world/<w>/pose/info. `samples` = (sim_t, veh_xyz, tag_xyz) from ONE
    Pose_V message each (same stamp, no cross-clock alignment). SCORING ONLY."""

    def __init__(self):
        self.t = None
        self.samples = []
        self.veh = self.tag = None
        self._lock = threading.Lock()

    def on_clock(self, msg):
        self.t = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(self, msg):
        v = g = None
        for p in msg.pose:
            if p.name == VEHICLE_MODEL:
                v = (p.position.x, p.position.y, p.position.z)
            elif p.name == TAG_MODEL:
                g = (p.position.x, p.position.y, p.position.z)
        if v is None or g is None:
            return
        t = stamp_s(msg)
        self.veh, self.tag = v, g
        with self._lock:
            if not self.samples or t - self.samples[-1][0] >= POSE_SAMPLE_MIN_DT:
                self.samples.append((t, v, g))


class SimClockShim:
    """Stands in for the `time` module inside real_flight: monotonic() = sim seconds."""

    def __init__(self, world):
        self._w = world

    def monotonic(self):
        return self._w.t

    def __getattr__(self, name):
        return getattr(time, name)


class MoverGoTrigger(rf.GateReadyTrigger):
    """GateReadyTrigger whose GO waits for the TARGET: when the SM's own arm gate has held
    `hold_ticks`, spawn the mover; GO fires the first poll after its 'streaming started'
    handshake, so the pre-flight belief seed (target at start, t=0 at GO) is true to within
    one tick. Also the per-tick logging hook (poll() is called once per driver tick)."""

    def __init__(self, mover_cmd, world, detector, world_name, hold_ticks=4):
        super().__init__(hold_ticks)
        self.mover_cmd, self.world, self.detector = mover_cmd, world, detector
        self.world_name = world_name
        self.proc = None
        self._started = threading.Event()
        self.go_sim_t = None
        self.sm = None
        self.rows = []

    def observe_gate(self, gate_ok):
        self._ok_ticks = self._ok_ticks + 1 if gate_ok else 0
        if self._ok_ticks >= self.hold_ticks and self.proc is None:
            print(f"[xcheck] arm gate held; spawning mover: {' '.join(self.mover_cmd)}")
            # UNBUFFERED: a piped child block-buffers print(), which would hold the
            # handshake line back and GO would never fire.
            self.proc = subprocess.Popen(self.mover_cmd, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True,
                                         env={**os.environ, "PYTHONUNBUFFERED": "1",
                                              "INTERCEPTOR_WORLD_NAME": self.world_name})
            threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.proc.stdout:
            print(line.rstrip())
            if "streaming started" in line:
                self._started.set()

    def poll(self, t):
        if self._started.is_set() and not self.forced_go:
            self.forced_go = True
            self.go_sim_t = self.world.t
            print(f"[xcheck] mover streaming -> GO at sim t={self.go_sim_t:.3f}")
        v, g, d = self.world.veh, self.world.tag, self.detector.last
        self.rows.append([
            f"{t:.3f}", "" if self.world.t is None else f"{self.world.t:.4f}",
            self.sm.state if self.sm else "",
            *(("", "", "") if v is None else (f"{v[1]:.3f}", f"{v[0]:.3f}", f"{-v[2]:.3f}")),
            *(("", "", "") if g is None else (f"{g[1]:.3f}", f"{g[0]:.3f}", f"{-g[2]:.3f}")),
            "" if d is None or d.range_m is None else f"{d.range_m:.3f}",
            "" if d is None or d.tag_range_m is None else f"{d.tag_range_m:.3f}",
            "" if d is None or d.t_capture_sim is None else f"{d.t_capture_sim:.4f}",
            self.detector.n_consumed])
        return super().poll(t)


CSV_HEADER = ["mission_t", "sim_t", "sm_state_pre_step",
              "own_n_gz", "own_e_gz", "own_d_gz", "gt_tgt_n", "gt_tgt_e", "gt_tgt_d",
              "det_range_m", "det_tagpose_range_m", "det_t_capture_sim", "n_det_consumed"]


def geometry(cross_range, lead, speed):
    """-> (target_start_en, target_vel_en): isim crossing rotated to a world +X track."""
    return (-lead, -cross_range), (speed, 0.0)


def cpa(samples, t0, t1):
    """Min 3-D separation over [t0, t1] with linear interpolation between samples.
    CENTRE-TO-CENTRE ruler: vehicle model origin to tag-board centre (not a contact
    distance, not the ram radius)."""
    best, t_best = math.inf, None
    seg = [s for s in samples if t0 <= s[0] <= t1]
    for (ta, va, ga), (tb, vb, gb) in zip(seg, seg[1:]):
        d0 = np.subtract(ga, va)
        dd = np.subtract(gb, vb) - d0
        den = float(dd @ dd)
        s = 0.0 if den < 1e-12 else min(1.0, max(0.0, -float(d0 @ dd) / den))
        r = float(np.linalg.norm(d0 + s * dd))
        if r < best:
            best, t_best = r, ta + s * (tb - ta)
    if len(seg) == 1:
        best, t_best = float(np.linalg.norm(np.subtract(seg[0][2], seg[0][1]))), seg[0][0]
    return best, t_best


def self_test():
    """3 synthetic rendered-tag frames through decode + Detection conversion. No sim."""
    import cv2
    from gz.msgs10.image_pb2 import Image, RGB_INT8
    ok = True

    def check(cond, what):
        nonlocal ok
        ok &= bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {what}")

    fx, cx, cy = 539.936, 640.0, 480.0
    det = GazeboTagDetector(fx, fx, cx, cy)
    tag = cv2.imread(TAG_PNG, cv2.IMREAD_GRAYSCALE)
    check(tag is not None, f"tag texture loads ({TAG_PNG})")
    for k, (cell, u, v) in enumerate([(8, 400, 300), (12, 640, 480), (20, 900, 600)]):
        side = 10 * cell                              # 10-cell grid, black square = 8 cells
        patch = cv2.resize(tag, (side, side), interpolation=cv2.INTER_NEAREST)
        gray = np.full((960, 1280), 128, np.uint8)
        gray[v - side // 2:v - side // 2 + side, u - side // 2:u - side // 2 + side] = patch
        rgb = np.dstack([gray] * 3)
        msg = Image(width=1280, height=960, step=1280 * 3, pixel_format_type=RGB_INT8,
                    data=rgb.tobytes())
        msg.header.stamp.sec, msg.header.stamp.nsec = 100 + k, 250_000_000
        det.on_image(msg)
        d = det.detect(np.zeros((960, 1280, 3), np.uint8), 0.0)
        bw_true = 8 * cell
        print(f" frame {k}: black square {bw_true}px at ({u},{v}) -> box={d.box_xywh} "
              f"range={d.range_m} tagpose={d.tag_range_m}")
        check(d.range_m is not None, f"frame {k}: tag detected")
        if d.range_m is None:
            continue
        x, y, w, h = d.box_xywh
        check(abs(x + w / 2 - u) < 2 and abs(y + h / 2 - v) < 2, f"frame {k}: box centre")
        check(abs(w - bw_true) <= 2 and abs(h - bw_true) <= 2, f"frame {k}: box size")
        check(abs(d.range_m - fx * TAG_SIZE_M / w) < 1e-9, f"frame {k}: range=fx*span/w")
        # A pasted square = a fronto-parallel tag at depth Z = fx*span/w; the pose range
        # is the SLANT range |t| = Z*sqrt(1 + off-axis^2), NOT the box-derived depth.
        r_exp = (fx * TAG_SIZE_M / bw_true) * math.sqrt(1 + ((u - cx) ** 2 + (v - cy) ** 2)
                                                        / fx ** 2)
        check(abs(d.tag_range_m - r_exp) / r_exp < 0.03, f"frame {k}: pose slant range "
              f"~{r_exp:.2f} m")
        check(abs(d.t_capture_sim - (100 + k + 0.25)) < 1e-9, f"frame {k}: t_capture = stamp")
        again = det.detect(None, 0.0)
        check(again.range_m is None and again.box_xywh is None,
              f"frame {k}: re-polling a consumed frame is a MISS")
    check(det.n_consumed == 3, "3 detections consumed")
    blank = Image(width=1280, height=960, step=1280 * 3, pixel_format_type=RGB_INT8,
                  data=bytes(1280 * 960 * 3))
    blank.header.stamp.sec = 200
    det.on_image(blank)
    check(det.detect(None).range_m is None, "blank frame -> miss (range None)")
    bad = Image(width=1280, height=960, step=1280 * 3 + 4, pixel_format_type=RGB_INT8)
    try:
        image_msg_to_gray(bad)
        check(False, "row padding refused")
    except ValueError:
        check(True, "row padding refused (fail loud)")
    (e, n), (ve, vn) = geometry(6.5, 16.2, 9.0)
    check((ve, vn) == (9.0, 0.0), "track along world +X: identity board (-X face) is rear")
    check(math.isclose(math.hypot(e, n), math.hypot(6.5, 16.2)), "rotation preserves range")
    s = [(0.0, (0, 0, 0), (-3, 1, 0)), (1.0, (0, 0, 0), (3, 1, 0))]
    r, tc = cpa(s, 0.0, 1.0)
    check(abs(r - 1.0) < 1e-9 and abs(tc - 0.5) < 1e-9, "CPA interpolates between samples")
    print(f"[self-test] {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def camera_info(node, world, timeout_s=30.0):
    from gz.msgs10.camera_info_pb2 import CameraInfo
    topic = (f"/world/{world}/model/{VEHICLE_MODEL}/link/camera_link"
             "/sensor/imager/camera_info")
    got = threading.Event()
    box = {}

    def cb(m):
        box["m"] = m
        got.set()

    node.subscribe(CameraInfo, topic, cb)
    if not got.wait(timeout_s):
        raise SystemExit(f"[xcheck] FAIL: no CameraInfo on {topic} in {timeout_s}s")
    node.unsubscribe(topic)
    k = box["m"].intrinsics.k
    return k[0], k[4], k[2], k[5], box["m"].width, box["m"].height


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--mavsdk-url", default="udpin://0.0.0.0:14540")
    ap.add_argument("--world", default="apriltag")
    ap.add_argument("--cross-range", type=float, default=6.5)
    ap.add_argument("--lead", type=float, default=16.2)
    ap.add_argument("--speed", type=float, default=9.0)
    ap.add_argument("--tag-yaw-deg", type=float, default=0.0)
    ap.add_argument("--window-s", type=float, default=25.0, help="sim s after GO: mover "
                    "duration AND the CPA scoring window")
    ap.add_argument("--standby-budget-s", type=float, default=60.0)
    ap.add_argument("--wall-clock-driver", action="store_true",
                    help="leave real_flight on wall time.monotonic (default: sim clock)")
    ap.add_argument("--out", default=None)
    args, rf_extra = ap.parse_known_args(argv)
    if args.self_test:
        return self_test()
    if args.tag_yaw_deg != 0.0:
        ap.error("--tag-yaw-deg must be 0: m4_target_mover.py resets the board to identity "
                 "every tick (see GEOMETRY); the +X track already makes the face rear-facing")
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V

    ts = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    out = args.out or os.path.join(_REPO, "logs", f"xcheck_pursuit_{ts}.csv")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    node, world = Node(), SimWorld()
    fx, fy, cx, cy, w, h = camera_info(node, args.world)
    print(f"[xcheck] CameraInfo fx={fx:.3f} fy={fy:.3f} cx={cx} cy={cy} {w}x{h}")
    intr = os.path.join(tempfile.mkdtemp(prefix="xcheck_"), "intrinsics.json")
    with open(intr, "w") as fh:
        json.dump({"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                   "resolution": {"width": w, "height": h},
                   "source": f"gz CameraInfo /world/{args.world} (live)"}, fh)
    det = GazeboTagDetector(fx, fy, cx, cy)
    node.subscribe(Clock, "/clock", world.on_clock)
    node.subscribe(Pose_V, f"/world/{args.world}/pose/info", world.on_pose)
    node.subscribe(Image, f"/world/{args.world}/model/{VEHICLE_MODEL}/link/camera_link"
                   "/sensor/imager/image", det.on_image)
    t_wait = time.monotonic() + 15.0
    while (world.t is None or det.latest is None) and time.monotonic() < t_wait:
        time.sleep(0.05)
    if world.t is None or det.latest is None:
        print("[xcheck] FAIL: no /clock or camera frames within 15 s -- is the sim up?")
        return 2

    (te, tn), (ve, vn) = geometry(args.cross_range, args.lead, args.speed)
    rf_args = rf.build_arg_parser().parse_args([
        "--sitl-smoke", "--terminal", "pursuit", "--mavsdk-url", args.mavsdk_url,
        f"--target-start={te},{tn}", f"--target-vel={ve},{vn}",
        "--target-span-m", str(TAG_SIZE_M), "--intrinsics", intr,
        "--smoke-acquire-after-s", "0", "--miss-safe-behavior", "land",
        "--smoke-duration", str(args.standby_budget_s + args.window_s),
        "--log-csv", out.replace(".csv", "_rf.csv"), *rf_extra])
    # Mirrors real_flight.main()'s assembly (cfg -> gcfg -> cam -> terminal) so the
    # terminal is built by real_flight's OWN build_config/build_terminal.
    cfg = rf.build_config(rf_args)
    gcfg = rf.GuidanceConfig(
        n_pronav=rf_args.n_pronav, mount_fwd_m=rf_args.mount_fwd_m,
        mount_left_m=rf_args.mount_left_m, mount_up_m=rf_args.mount_up_m,
        mount_up_rad=math.radians(rf_args.mount_tilt_deg),
        alt_ref_m=rf.apply_alt_ref_trim(cfg.dash_base_alt_m, cfg.dash_alt_trim_m, None))
    gcfg.target_span_m = float(rf_args.target_span_m)
    cam = rf.load_camera(rf_args.intrinsics)
    guidance = rf.build_terminal(rf_args, cfg, gcfg, cam)
    tag_z = cfg.dash_base_alt_m      # the belief's own altitude: target at standby-loft
    mover_cmd = [sys.executable, MOVER, f"--start={te},{tn},{tag_z}", f"--vel={ve},{vn}",
                 "--duration", str(args.window_s)]
    trigger = MoverGoTrigger(mover_cmd, world, det, args.world)
    sm = rf.RealFlightSM(cfg, guidance=guidance, on_event=print)
    trigger.sm = sm
    if not args.wall_clock_driver:
        rf.time = SimClockShim(world)
    rc, aborted = 1, 1
    try:
        rc = asyncio.run(asyncio.wait_for(
            rf.run_mavsdk_mission(rf_args, cfg, sm, trigger, det, smoke=False),
            timeout=4.0 * (args.standby_budget_s + args.window_s) + 120.0))
        aborted = int(rc != 0 or trigger.go_sim_t is None)
    except asyncio.TimeoutError:
        print("[xcheck] FAIL: wall-clock watchdog fired (sim stalled?)")
    finally:
        rf.time = time
        if trigger.proc is not None and trigger.proc.poll() is None:
            trigger.proc.terminate()
    with open(out, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(CSV_HEADER)
        wr.writerows(trigger.rows)
    with world._lock:
        samples = list(world.samples)
    with open(out.replace(".csv", "_gtpose.csv"), "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["sim_t", "gt_veh_x", "gt_veh_y", "gt_veh_z", "gt_tag_x", "gt_tag_y",
                     "gt_tag_z"])
        wr.writerows([f"{t:.4f}", *v, *g] for t, v, g in samples)
    if trigger.go_sim_t is None:
        d_cpa, t_cpa = math.inf, None
    else:
        d_cpa, t_cpa = cpa(samples, trigger.go_sim_t, trigger.go_sim_t + args.window_s)
    print(f"[xcheck] frames={det.n_frames} consumed={det.n_consumed} stale_polls="
          f"{det.n_stale} safe_reason={sm.safe_reason} per-tick -> {out}")
    print(f"XCHECK_RESULT cpa_m={d_cpa:.3f} t_cpa="
          f"{'nan' if t_cpa is None else f'{t_cpa:.3f}'} n_det={det.n_consumed} "
          f"final_state={sm.state} aborted={aborted}")
    return 0 if not aborted else 1


if __name__ == "__main__":
    sys.exit(main())
