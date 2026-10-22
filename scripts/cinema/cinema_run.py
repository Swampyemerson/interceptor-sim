#!/usr/bin/env python3
"""Cinema demo run: one camera-in-the-loop pursuit engagement in Gazebo,
logged richly enough to RE-RENDER offline (scripts/cinema/blender_scene.py).

Derived from scripts/gazebo_pursuit_crosscheck.py (the validated harness —
its classes are IMPORTED, not copied) with four demo-specific additions:

  1. WORLD/TARGET: flies `worlds/cinema.sdf`, whose target entity is the
     quad-bodied `fpv_target` under its own name, so m4_target_mover.py
     `--orient-to-velocity` may stream a realistic attitude (yaw to travel,
     bank into turns, nose-down cruise pitch). The tag is BOLTED to that
     body, so the detector earns every decode under the canted/pitched tag
     the render later shows — render-truth equals camera-truth.
  2. CUE ERROR: `--cue-offset-e/n`, `--cue-vel-rot-deg`, `--cue-speed-err`
     perturb ONLY the pre-flight belief handed to real_flight
     (`--target-start/--target-vel`); the mover flies the TRUE track. A
     perfect cue is the harness default and stays available (zero offsets).
  3. 6-DoF POSE LOG: full position+quaternion of vehicle AND target at
     ~50 Hz sim time (`*_pose6.csv`) — the renderer's input.
  4. FRAME RECORDING: every camera frame to JPEG + an index CSV
     (`frames/`, `frames_index.csv`) for the picture-in-picture seeker
     view. Detection boxes are re-derived OFFLINE by export_run.py with
     the same detector, so the hot loop stays light.

HONESTY: unchanged from the parent harness — pose/info and frame recording
are SCORING/LOGGING ONLY; guidance sees camera boxes + own-state EKF. The
injected cue error makes the demo's given cue WORSE than the validated
perfect-cue scope, never better. Flight code under flight/ is not modified.

Boot (one sim at a time, idle machine):
  bash scripts/sim_kill.sh && bash scripts/demo_boot_sim.sh /tmp/sim.log cinema
Run:
  .venv/bin/python scripts/cinema/cinema_run.py --out logs/cinema/take1.csv \
      --lead -3 --cross-range 4 --speed 12 --cue-offset-n 2.0 \
      --engage-max-s 40
"""
import argparse
import asyncio
import csv
import json
import math
import os
import queue
import sys
import threading
import time
import types

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _REPO)
sys.path.insert(0, os.path.join(_REPO, "scripts"))

import gazebo_pursuit_crosscheck as xck  # noqa: E402  (the validated harness)
import flight.deploy.real_flight as rf   # noqa: E402

TARGET_ENTITY = "fpv_target"             # worlds/cinema.sdf <include><name>


class DelayedDetector:
    """--inject-det-latency-s: hold each decoded detection until SIM time
    (the newest camera frame's stamp) reaches its capture time + delay, then
    release it. Models a slow detector pipeline on the real aircraft: the
    driver stamps detections at ARRIVAL (crosscheck line-31 note), so the
    guidance believes a delay-old bearing is current -- the same mechanism
    isim's seeker `latency_s` models (runs/nn_requirements_20261021).
    SCORING-HONEST: this only makes the camera pipeline WORSE; delay<=0
    never installs the wrapper, keeping the validated path byte-identical."""

    _MISS = dict(range_m=None, box_xywh=None, t_capture_sim=None, tag_range_m=None)

    def __init__(self, inner, delay_s: float, jitter_s: float = 0.0,
                 seed: int = 0, stamp_capture: bool = False):
        self._inner = inner
        self._delay = float(delay_s)
        self._jitter = float(jitter_s)      # per-detection uniform(+/-jitter)
        # --det-stamp-capture (LJ90-S arm): republish each RELEASED
        # detection's capture stamp as `t_capture`, the attribute the flight
        # code's --pursuit-stamped-age path reads. Valid ONLY here, because
        # cinema runs the whole mission on the SimClockShim, so t_capture_sim
        # IS the mission clock; the flight code's own attribute-name
        # exclusion of gz stamps (real_flight det_age block) stays intact
        # for every other driver. Default off = byte-identical.
        self._stamp_capture = bool(stamp_capture)
        import numpy as _np
        self._rng = _np.random.default_rng([seed, 0x1A7E])
        self._q = []                        # [(release_at_sim, det), ...]
        self.last = types.SimpleNamespace(**self._MISS)

    def on_image(self, msg):
        self._inner.on_image(msg)

    def __getattr__(self, name):          # latest / n_frames / n_consumed ...
        return getattr(self._inner, name)

    def detect(self, frame, t=None):
        d = self._inner.detect(frame, t)
        if d.range_m is not None:
            dly = self._delay
            if self._jitter > 0.0:
                dly = max(0.0, dly + float(self._rng.uniform(-self._jitter,
                                                             self._jitter)))
            self._q.append((d.t_capture_sim + dly, d))
        latest = self._inner.latest
        now = xck.stamp_s(latest) if latest is not None else None
        if self._q and now is not None and now >= self._q[0][0]:
            self.last = self._q.pop(0)[1]  # one release per poll (one meas/tick)
            if self._stamp_capture and self.last.t_capture_sim is not None:
                # Review F1 (2026-10-22): publish on the MONOTONIC axis, not
                # as `t_capture` (which real_flight ages against MISSION-
                # relative t -- an absolute stamp there clamps every age to
                # 0.0 and silently un-ages the measurements). Under the
                # SimClockShim, time.monotonic() == absolute sim time ==
                # this stamp's base.
                self.last.t_capture_monotonic = self.last.t_capture_sim
        else:
            self.last = types.SimpleNamespace(**self._MISS)
        return self.last


class Pose6Recorder:
    """Full 6-DoF (pos + wxyz quaternion) of both models from pose/info.
    SCORING/LOGGING ONLY — nothing here reaches the driver or guidance."""

    def __init__(self, vehicle_name, target_name, min_dt=0.02):
        self.names = (vehicle_name, target_name)
        self.min_dt = min_dt
        self.rows = []
        self._lock = threading.Lock()
        self._last_t = -1e9

    def on_pose(self, msg):
        v = g = None
        for p in msg.pose:
            if p.name == self.names[0]:
                v = p
            elif p.name == self.names[1]:
                g = p
        if v is None or g is None:
            return
        t = xck.stamp_s(msg)
        with self._lock:
            if t - self._last_t < self.min_dt:
                return
            self._last_t = t
            self.rows.append([
                f"{t:.4f}",
                *(f"{x:.4f}" for x in (v.position.x, v.position.y, v.position.z,
                                       v.orientation.w, v.orientation.x,
                                       v.orientation.y, v.orientation.z)),
                *(f"{x:.4f}" for x in (g.position.x, g.position.y, g.position.z,
                                       g.orientation.w, g.orientation.x,
                                       g.orientation.y, g.orientation.z))])

    def write(self, path):
        with open(path, "w", newline="") as fh:
            wr = csv.writer(fh)
            wr.writerow(["sim_t",
                         "veh_x", "veh_y", "veh_z",
                         "veh_qw", "veh_qx", "veh_qy", "veh_qz",
                         "tgt_x", "tgt_y", "tgt_z",
                         "tgt_qw", "tgt_qx", "tgt_qy", "tgt_qz"])
            with self._lock:
                wr.writerows(self.rows)
        return len(self.rows)


class FrameRecorder:
    """Every camera frame -> JPEG on a writer thread. Index CSV carries the
    gz sim stamp so frames align with the pose log exactly."""

    def __init__(self, frames_dir, quality=92):
        import cv2  # lazy, like the parent harness
        self.cv2 = cv2
        self.dir = frames_dir
        self.quality = quality
        os.makedirs(frames_dir, exist_ok=True)
        self.q = queue.Queue(maxsize=64)
        self.index = []
        self.n_dropped = 0
        self._n = 0
        self._last_stamp = None
        self._thread = threading.Thread(target=self._writer, daemon=True)
        self._thread.start()

    def on_image(self, msg):
        t = xck.stamp_s(msg)
        if t == self._last_stamp:
            return
        self._last_stamp = t
        try:
            # copy the bytes now: gz may reuse the buffer after the callback
            self.q.put_nowait((self._n, t, bytes(msg.data), msg.width, msg.height))
            self._n += 1
        except queue.Full:
            self.n_dropped += 1

    def _writer(self):
        import numpy as np
        while True:
            item = self.q.get()
            if item is None:
                return
            n, t, data, w, h = item
            rgb = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
            path = os.path.join(self.dir, f"{n:05d}.jpg")
            self.cv2.imwrite(path, self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2BGR),
                             [self.cv2.IMWRITE_JPEG_QUALITY, self.quality])
            self.index.append((n, t))

    def close(self, index_path):
        self.q.put(None)
        self._thread.join(timeout=30)
        with open(index_path, "w", newline="") as fh:
            wr = csv.writer(fh)
            wr.writerow(["frame", "sim_t"])
            wr.writerows([(n, f"{t:.4f}") for n, t in sorted(self.index)])
        return len(self.index), self.n_dropped


def rotate_en(e, n, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (c * e - s * n, s * e + c * n)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--mavsdk-url", default="udpin://0.0.0.0:14540")
    ap.add_argument("--world", default="cinema")
    ap.add_argument("--target-entity", default=TARGET_ENTITY)
    ap.add_argument("--cross-range", type=float, default=4.0)
    ap.add_argument("--lead", type=float, default=-3.0,
                    help="m before abeam (NEGATIVE = tail-chase: target starts "
                         "past abeam and the vehicle overtakes)")
    ap.add_argument("--speed", type=float, default=12.0)
    ap.add_argument("--window-s", type=float, default=40.0)
    ap.add_argument("--standby-budget-s", type=float, default=60.0)
    # --- cue error (belief-only; the mover flies truth) ---
    ap.add_argument("--cue-offset-e", type=float, default=0.0)
    ap.add_argument("--cue-offset-n", type=float, default=0.0)
    ap.add_argument("--cue-vel-rot-deg", type=float, default=0.0)
    ap.add_argument("--inject-det-latency-s", type=float, default=0.0,
                    help="hold each detection this many SIM seconds before the "
                         "driver sees it (slow-NN-pipeline model; 0 = off, "
                         "validated path untouched)")
    ap.add_argument("--inject-det-jitter-s", type=float, default=0.0,
                    help="per-detection uniform(+/-this) added to the injected "
                         "latency (pipeline-jitter model; needs latency > 0)")
    ap.add_argument("--inject-det-seed", type=int, default=0)
    ap.add_argument("--det-stamp-capture", action="store_true",
                    help="republish each RELEASED detection's capture stamp as "
                         "t_capture (the --pursuit-stamped-age input). Valid "
                         "only in cinema (mission clock IS sim time via the "
                         "shim); requires the latency injector (LJ90-S arm)")
    ap.add_argument("--cue-speed-err", type=float, default=0.0,
                    help="fractional: 0.1 = belief thinks 10%% faster")
    # --- target realism (mover-side) ---
    ap.add_argument("--no-orient", action="store_true",
                    help="disable --orient-to-velocity (upright world-fixed tag)")
    ap.add_argument("--orient-max-pitch-deg", type=float, default=18.0)
    ap.add_argument("--path", default="line", choices=("line", "weave"))
    ap.add_argument("--weave-period-s", type=float, default=4.0)
    ap.add_argument("--weave-lat-speed", type=float, default=2.0)
    # --- outputs ---
    ap.add_argument("--out", required=True, help="per-tick CSV; siblings "
                    "*_rf.csv, *_pose6.csv, *_gtpose.csv, *_meta.json, frames/")
    ap.add_argument("--no-frames", action="store_true")
    args, rf_extra = ap.parse_known_args(argv)

    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    stem = out[:-4] if out.endswith(".csv") else out

    # the mover must address the cinema world + entity (env-var plumbing it
    # already supports; MoverGoTrigger passes {**os.environ} through)
    os.environ["INTERCEPTOR_TARGET_MODEL"] = args.target_entity
    xck.TAG_MODEL = args.target_entity   # SimWorld + CPA ruler follow the entity

    # recorders live on their OWN Node: gz-transport13 may not accept two
    # callbacks on one topic from a single node, and the recorders must
    # never interfere with the driver's subscriptions
    node, rec_node, world = Node(), Node(), xck.SimWorld()
    pose6 = Pose6Recorder(xck.VEHICLE_MODEL, args.target_entity)
    fx, fy, cx, cy, w, h = xck.camera_info(node, args.world)
    print(f"[cinema] CameraInfo fx={fx:.3f} fy={fy:.3f} cx={cx} cy={cy} {w}x{h}")
    intr_path = stem + "_intrinsics.json"
    with open(intr_path, "w") as fh:
        json.dump({"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                   "resolution": {"width": w, "height": h},
                   "source": f"gz CameraInfo /world/{args.world} (live)"}, fh)
    det = xck.GazeboTagDetector(fx, fy, cx, cy)
    if args.inject_det_latency_s > 0.0:
        det = DelayedDetector(det, args.inject_det_latency_s,
                              jitter_s=args.inject_det_jitter_s,
                              seed=args.inject_det_seed,
                              stamp_capture=args.det_stamp_capture)
        print(f"[cinema] DETECTION LATENCY INJECTED: +{args.inject_det_latency_s:.3f} s "
              f"+/-{args.inject_det_jitter_s:.3f} s (sim)"
              + (" + capture stamps republished (t_capture)"
                 if args.det_stamp_capture else ""))
    elif args.det_stamp_capture:
        sys.exit("[cinema] --det-stamp-capture requires the latency injector "
                 "(--inject-det-latency-s > 0) -- refusing a silent no-op")
    frames = None if args.no_frames else FrameRecorder(stem + "_frames")
    img_topic = (f"/world/{args.world}/model/{xck.VEHICLE_MODEL}/link/camera_link"
                 "/sensor/imager/image")
    node.subscribe(Clock, "/clock", world.on_clock)
    node.subscribe(Pose_V, f"/world/{args.world}/pose/info", world.on_pose)
    rec_node.subscribe(Pose_V, f"/world/{args.world}/pose/info", pose6.on_pose)
    node.subscribe(Image, img_topic, det.on_image)
    if frames is not None:
        rec_node.subscribe(Image, img_topic, frames.on_image)
    t_wait = time.monotonic() + 15.0
    while (world.t is None or det.latest is None) and time.monotonic() < t_wait:
        time.sleep(0.05)
    if world.t is None or det.latest is None:
        print("[cinema] FAIL: no /clock or camera frames within 15 s -- sim up?")
        return 2

    (te, tn), (ve, vn) = xck.geometry(args.cross_range, args.lead, args.speed)
    be, bn = te + args.cue_offset_e, tn + args.cue_offset_n
    bve, bvn = rotate_en(ve, vn, args.cue_vel_rot_deg)
    bve, bvn = bve * (1 + args.cue_speed_err), bvn * (1 + args.cue_speed_err)

    rf_args = rf.build_arg_parser().parse_args([
        "--sitl-smoke", "--terminal", "pursuit", "--mavsdk-url", args.mavsdk_url,
        f"--target-start={be},{bn}", f"--target-vel={bve},{bvn}",
        "--target-span-m", str(xck.TAG_SIZE_M), "--intrinsics", intr_path,
        "--smoke-acquire-after-s", "0", "--miss-safe-behavior", "land",
        "--smoke-duration", str(args.standby_budget_s + args.window_s),
        "--pursuit-brake", "--engage-max-s", str(args.window_s),
        "--log-csv", stem + "_rf.csv", *rf_extra])
    cfg = rf.build_config(rf_args)
    gcfg = rf.GuidanceConfig(
        n_pronav=rf_args.n_pronav, mount_fwd_m=rf_args.mount_fwd_m,
        mount_left_m=rf_args.mount_left_m, mount_up_m=rf_args.mount_up_m,
        mount_up_rad=math.radians(rf_args.mount_tilt_deg),
        alt_ref_m=rf.apply_alt_ref_trim(cfg.dash_base_alt_m, cfg.dash_alt_trim_m,
                                        None))
    gcfg.target_span_m = float(rf_args.target_span_m)
    cam = rf.load_camera(rf_args.intrinsics)
    guidance = rf.build_terminal(rf_args, cfg, gcfg, cam)
    tag_z = cfg.dash_base_alt_m
    mover_cmd = [sys.executable, xck.MOVER, f"--start={te},{tn},{tag_z}",
                 f"--vel={ve},{vn}", "--duration", str(args.window_s),
                 "--path", args.path,
                 "--weave-period-s", str(args.weave_period_s),
                 "--weave-lat-speed", str(args.weave_lat_speed)]
    if not args.no_orient:
        mover_cmd += ["--orient-to-velocity", "--orient-yaw-offset-deg", "0",
                      "--orient-max-pitch-deg", str(args.orient_max_pitch_deg)]
    trigger = xck.MoverGoTrigger(mover_cmd, world, det, args.world)
    sm = rf.RealFlightSM(cfg, guidance=guidance, on_event=print)
    trigger.sm = sm
    rf.time = xck.SimClockShim(world)
    rc, aborted = 1, 1
    try:
        rc = asyncio.run(asyncio.wait_for(
            rf.run_mavsdk_mission(rf_args, cfg, sm, trigger, det, smoke=False),
            timeout=4.0 * (args.standby_budget_s + args.window_s) + 120.0))
        aborted = int(rc != 0 or trigger.go_sim_t is None)
    except asyncio.TimeoutError:
        print("[cinema] FAIL: wall-clock watchdog fired (sim stalled?)")
    finally:
        rf.time = time
        if trigger.proc is not None and trigger.proc.poll() is None:
            trigger.proc.terminate()

    with open(out, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(xck.CSV_HEADER)
        wr.writerows(trigger.rows)
    with world._lock:
        samples = list(world.samples)
    with open(stem + "_gtpose.csv", "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["sim_t", "gt_veh_x", "gt_veh_y", "gt_veh_z",
                     "gt_tag_x", "gt_tag_y", "gt_tag_z"])
        wr.writerows([f"{t:.4f}", *v, *g] for t, v, g in samples)
    n_pose6 = pose6.write(stem + "_pose6.csv")
    n_frames = frames_dropped = 0
    if frames is not None:
        n_frames, frames_dropped = frames.close(stem + "_frames_index.csv")
    if trigger.go_sim_t is None:
        d_cpa, t_cpa = math.inf, None
    else:
        d_cpa, t_cpa = xck.cpa(samples, trigger.go_sim_t,
                               trigger.go_sim_t + args.window_s)
    meta = {
        "config": vars(args), "rf_extra": rf_extra,
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "w": w, "h": h},
        "hfov_deg": 2 * math.degrees(math.atan(0.5 * w / fx)),
        "vfov_deg": 2 * math.degrees(math.atan(0.5 * h / fy)),
        "true_start_en": [te, tn], "true_vel_en": [ve, vn], "tag_alt_m": tag_z,
        "belief_start_en": [be, bn], "belief_vel_en": [bve, bvn],
        "cue_error_note": "belief-only injection; mover flies truth",
        "go_sim_t": trigger.go_sim_t, "cpa_m": d_cpa, "t_cpa": t_cpa,
        "n_det_consumed": det.n_consumed, "n_frames_gz": det.n_frames,
        "inject_det_latency_s": args.inject_det_latency_s,
        "n_frames_saved": n_frames, "frames_dropped": frames_dropped,
        "n_pose6": n_pose6, "final_state": sm.state, "aborted": aborted,
        "safe_reason": sm.safe_reason,
        "scope": "DEMO run (cinema pipeline): camera-in-the-loop Gazebo, "
                 "injected cue error as configured, windless, sim camera; "
                 "NOT a gate — gates live in docs/xcheck_gazebo_pursuit_*.md",
    }
    with open(stem + "_meta.json", "w") as fh:
        json.dump(meta, fh, indent=1)
    print(f"[cinema] frames={det.n_frames} consumed={det.n_consumed} "
          f"saved={n_frames} dropped={frames_dropped} pose6={n_pose6}")
    print(f"CINEMA_RESULT cpa_m={d_cpa:.3f} t_cpa="
          f"{'nan' if t_cpa is None else f'{t_cpa:.3f}'} n_det={det.n_consumed} "
          f"final_state={sm.state} aborted={aborted}")
    return 0 if not aborted else 1


if __name__ == "__main__":
    sys.exit(main())
