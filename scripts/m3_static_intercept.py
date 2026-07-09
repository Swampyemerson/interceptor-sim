#!/usr/bin/env python3
"""M3 gate script: close on the static AprilTag using ONLY the live camera
detection as target feedback, and hold a 2.0 m standoff. See GOALS.md
milestone M3, scripts/m2_detect.py (detection + ground-truth transform
chain, ADR-0006) and scripts/m0_takeoff.py (MAVSDK connect/telemetry
patterns) -- this script reuses both by import rather than copying them.

WHAT: takes off, enters MAVSDK OFFBOARD, and runs a simple two-axis
proportional (P) controller entirely off the AprilTag's measured pose from
pupil-apriltags (ADR-0003):
  - forward/back speed is P-control on RANGE error: close when the tag is
    farther than the STANDOFF_M target distance, back off when closer.
  - yaw rate is P-control on BEARING (the tag's left/right offset from the
    camera's boresight): turn toward the tag so it stays centered in view.
This is intentionally the simplest guidance law that closes a range and
centers a bearing -- a "pursuit" controller, not proportional navigation
(pro-nav, which reacts to the *rate of rotation* of the line of sight
rather than the offset itself, comes in M4 against a moving target; see
GOALS.md's guidance arc). Altitude is held near ALT_REF_M with its own
small P-loop so the tag stays inside the camera's vertical field of view
through the whole approach.

WHY BODY-FRAME VELOCITY SETPOINTS: the tag measurement (range, bearing)
from pupil-apriltags comes out in the camera's own OPTICAL frame -- it is a
statement about where the tag is *relative to the vehicle*, not about any
world/NED heading. Commanding MAVSDK's VelocityBodyYawspeed (forward,
right, down, yawspeed -- FRD body convention, GOALS.md coordinate frames)
lets the controller act directly on that body-relative measurement with no
compass/heading math in between. (Contrast with VelocityNedYaw, which
would require converting the body-relative bearing into a world heading
first -- an extra, unnecessary place for a sign error.)

THE FEEDBACK SPLIT (the core idea this milestone proves, GOALS.md's
"parent -> simulation translation" table): the vehicle's OWN state (its
altitude, its armed/flight-mode status) still comes from PX4's own
state estimate (EKF, fed by simulated IMU/GPS -- MAVSDK telemetry). Only
the TARGET's position comes from the camera. In the real hardware this is
the whole point of an onboard terminal seeker: even with the datalink cut,
the vehicle still knows where *it* is (inertial nav) and, independently,
where the *target* is (its own camera) -- it does not need ground truth or
a live link to either. Ground truth from the world pose topic
(/world/apriltag/pose/info, same ADR-0006 transform chain as M2) is
recorded on every tick purely for the PASS/FAIL measurement and the CSV
log; it is never read by the control law above -- grep for "gt_" and you
will only find it downstream of the command that was already sent.

OFFBOARD "STREAM BEFORE START": PX4 refuses to enter OFFBOARD mode (and
will kick you back out of it) unless setpoints are already being streamed
at the moment you request the mode switch, and keeps requiring a fresh
setpoint at >= 2 Hz after that or it fails safe. So this script sends one
zero-velocity VelocityBodyYawspeed BEFORE calling offboard.start(), then
keeps sending a fresh setpoint every tick at CONTROL_RATE_HZ (20 Hz) for
as long as offboard mode is active.

HOW THE GATE IS MEASURED: the controller "settles" once its own measured
range stays within SETTLE_TOL_M of STANDOFF_M continuously for
SETTLE_DURATION_S. It then holds station (same control law, same camera
feedback) for HOLD_MEASURE_S more, during which ground-truth camera-to-tag
range is sampled every tick. final_gt_range is the mean of those samples;
final_err = |final_gt_range - STANDOFF_M|. The gate PASSES iff the
controller settled and final_err < FINAL_ERR_THRESHOLD_M (0.5 m, GOALS.md).
Using ground truth only for this final number -- never for control -- is
what makes "final standoff error" an honest, camera-closed-loop result
rather than a claim about a controller that secretly had perfect
knowledge.

Run manually (with PX4 SITL + gz_x500_mono_cam + the apriltag world already
running, see scripts/check_m2.sh's launch block for the exact env):

    .venv/bin/python scripts/m3_static_intercept.py

Normally this is invoked by scripts/check_m3.sh, which also launches and
tears down the simulator.
"""

import asyncio
import csv
import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

import numpy as np
from apriltag_detector import Detector  # portable: pupil-apriltags (x86) / pyapriltags (aarch64), ADR-0012

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import LandedState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from m2_detect import (  # noqa: E402
    quat_to_matrix,
    compose,
    pose_to_pt,
    image_msg_to_gray,
    get_camera_intrinsics,
    PoseTracker,
    IMAGE_TOPIC,
    CAMERA_INFO_TOPIC,
    POSE_TOPIC,
    TAG_SIZE_M,
    TAG_FAMILY,
    DRONE_MODEL,
    CAMERA_LINK_NAME,
    TAG_MODEL_NAME,
    EXPECTED_WIDTH,
    EXPECTED_HEIGHT,
)
from m0_takeoff import (  # noqa: E402
    TelemetryState,
    track_position,
    track_flight_mode,
    track_armed,
    track_landed_state,
    wait_for_connection,
    wait_for_health,
    SYSTEM_ADDRESS,
)

# --- Guidance targets / gains (GOALS.md M3). See the module docstring for
# the control law itself; these are the only numbers it needs. ---
STANDOFF_M = 2.0  # desired 3D camera-to-tag distance
FINAL_ERR_THRESHOLD_M = 0.5  # GOALS.md M3 gate
# Takeoff/hold relative altitude. Puts the camera ~1.25 m up (base_link
# ~1.0 m + the ~0.24 m camera mount offset, see ADR-0006), tag center at
# 0.5 m -- so at the STANDOFF_M range the tag sits ~20-25 deg below
# boresight, comfortably inside the mono_cam's ~41.6 deg vertical
# half-FOV (see camera_intrinsics.json / NEXT.md).
ALT_REF_M = 1.0
KP_RANGE = 0.5  # 1/s
V_FWD_MAX = 1.0  # m/s
V_BACK_MAX = 0.5  # m/s
KP_ALT = 1.0  # 1/s
V_VERT_MAX = 0.5  # m/s
KYAW_DEG_PER_DEG = 1.5
YAWSPEED_MAX_DEG_S = 30.0
CONTROL_RATE_HZ = 20.0
MEAS_STALE_S = 0.4  # ignore (treat as no detection) measurements older than this
SETTLE_TOL_M = 0.15
SETTLE_DURATION_S = 3.0
HOLD_MEASURE_S = 2.0
ACQUIRE_TIMEOUT_S = 15.0  # must see first usable detection within this after entering offboard
LOST_TAG_ABORT_S = 10.0  # continuous no-detection -> abort
OFFBOARD_TIMEOUT_S = 90.0  # whole guidance (pre-settle) phase
RANGE_ABORT_M = 0.7  # measured range below this -> abort, too close

# --- MAVSDK / takeoff constants. Mirrors scripts/m0_takeoff.py's own
# constants (not reused via import -- M0 exports its functions/state class,
# not these numeric knobs). ---
CONNECT_TIMEOUT_S = 60
HEALTH_TIMEOUT_S = 90
ALTITUDE_SUCCESS_FRACTION = 0.8
ALTITUDE_TIMEOUT_S = 60
LAND_TIMEOUT_S = 90
LOG_POLL_HZ = 5.0
POSE_WAIT_TIMEOUT_S = 30.0
CAMERA_INFO_TIMEOUT_S = 30.0

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")

CSV_HEADER = [
    "t", "phase", "detected",
    "meas_x", "meas_y", "meas_z", "meas_range", "bearing_rad",
    "cmd_vf", "cmd_vr", "cmd_vd", "cmd_yawrate_deg_s",
    "alt_m",
    "gt_cam_x", "gt_cam_y", "gt_cam_z",
    "gt_tag_x", "gt_tag_y", "gt_tag_z", "gt_range",
    "settled",
]


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class Measurement:
    """One detection-thread result. meas_xyz is the tag position in the
    camera OPTICAL frame (x right, y down, z forward -- pupil-apriltags'
    pose_t convention). range_m is None when the tag was not detected in
    that frame."""

    t_mono: float
    range_m: Optional[float]
    bearing_rad: Optional[float]
    meas_xyz: Optional[np.ndarray]
    decision_margin: Optional[float]
    n_detections: int
    # ADR-0058 provenance (DEFAULTED -- every existing 6-positional-arg
    # construction, including this file's detection_loop, is byte-identical):
    # which stage produced this measurement. "" = AprilTag/plain path; "track" =
    # detect-then-track CSRT tracker box (decision_margin is then the STALE last
    # NN confidence); an NN class name (e.g. "drone") = fresh markerless NN
    # detection. Logged to m4's meas_source CSV column; never read by control.
    source: str = ""


class LatestFrame:
    """Latest camera frame, gz-transport callback -> asyncio-thread reader.

    Same lock-free pattern as m2_detect.PoseTracker: the callback (fired on
    a background C++ thread) replaces `self.latest` with a brand new tuple
    every time, and that single attribute assignment is atomic under the
    GIL. We deliberately do NOT queue frames -- if the detection thread is
    ever slower than the camera, we want it to skip straight to the newest
    frame, not fall behind processing stale ones.
    """

    def __init__(self):
        self.latest = (0, None)  # (seq, Image | None)

    def on_image(self, msg: Image) -> None:
        seq, _ = self.latest
        self.latest = (seq + 1, msg)


class MeasurementHolder:
    """Latest Measurement, detection-thread -> asyncio-thread reader (same
    atomic-attribute pattern as LatestFrame/PoseTracker)."""

    def __init__(self):
        self.latest = Measurement(
            t_mono=time.monotonic(), range_m=None, bearing_rad=None,
            meas_xyz=None, decision_margin=None, n_detections=0,
        )


def detection_loop(frame_holder, meas_holder, fx, fy, cx, cy, stop_event,
                   detector_kwargs=None):
    """Runs on a background thread for the whole flight: whenever a new
    camera frame seq shows up, decode it and run pupil-apriltags on it,
    publishing a fresh Measurement. The asyncio control loop never blocks
    on this -- it just reads whatever MeasurementHolder.latest is."""
    # detector_kwargs lets callers trade accuracy for CPU (e.g. M4 passes
    # quad_decimate=2: PX4 SITL runs WITHOUT lockstep against gz, so a
    # CPU-saturating detector measurably degrades PX4's own control-loop
    # tracking -- observed 2026-07-05: 4 m/s commanded, 3.68 achieved
    # unloaded vs 1.83 under detector load. M3 keeps the default (its gate
    # numbers were validated without decimation and its speeds are low).
    detector = Detector(families=TAG_FAMILY, **(detector_kwargs or {}))
    last_seq = -1
    warned_resolution = False
    while not stop_event.is_set():
        seq, msg = frame_holder.latest
        if msg is None or seq == last_seq:
            time.sleep(0.005)
            continue
        last_seq = seq
        t_mono = time.monotonic()

        if msg.width != EXPECTED_WIDTH or msg.height != EXPECTED_HEIGHT:
            if not warned_resolution:
                print(
                    f"[m3] WARNING: frame resolution {msg.width}x{msg.height} != "
                    f"expected {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}; treating as no "
                    "detection"
                )
                warned_resolution = True
            meas_holder.latest = Measurement(t_mono, None, None, None, None, 0)
            continue

        try:
            gray = image_msg_to_gray(msg)
        except ValueError as exc:
            print(f"[m3] WARNING: frame decode failed: {exc}")
            meas_holder.latest = Measurement(t_mono, None, None, None, None, 0)
            continue

        detections = detector.detect(
            gray, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy),
            tag_size=TAG_SIZE_M,
        )

        if detections:
            det = detections[0]
            meas = det.pose_t.flatten()
            range_m = float(np.linalg.norm(meas))
            # + = tag is RIGHT of boresight (x right, z forward -- optical frame).
            bearing_rad = math.atan2(meas[0], meas[2])
            meas_holder.latest = Measurement(
                t_mono, range_m, bearing_rad, meas,
                float(det.decision_margin), len(detections),
            )
        else:
            meas_holder.latest = Measurement(t_mono, None, None, None, None, 0)


def sample_measurement(meas_holder: MeasurementHolder):
    """Return (detected: bool, Measurement). detected is False both when
    the detector found nothing AND when the latest result is older than
    MEAS_STALE_S -- either way, not usable for control this tick."""
    meas = meas_holder.latest
    age = time.monotonic() - meas.t_mono
    stale = age > MEAS_STALE_S
    detected = meas.range_m is not None and not stale
    return detected, meas


def ground_truth_world_points(tracker: PoseTracker):
    """Return (p_cam_world, p_tag_world, gt_range) in world/ENU axes, or
    (None, None, None) if the pose topic hasn't populated all three
    entities yet this tick (it can briefly lag -- not an abort condition).

    Reuses the exact ADR-0006 compose chain as
    PoseTracker.ground_truth_rel_optical (camera_link composes directly
    against the drone model, NOT via base_link -- see m2_detect.py's
    module docstring) but returns world-frame points for CSV logging
    instead of the camera-optical relative vector.
    """
    latest = tracker.latest
    names = (DRONE_MODEL, CAMERA_LINK_NAME, TAG_MODEL_NAME)
    if any(n not in latest for n in names):
        return None, None, None
    p_model, R_model = latest[DRONE_MODEL]
    p_cam_rel, R_cam_rel = latest[CAMERA_LINK_NAME]
    p_tag_world, _ = latest[TAG_MODEL_NAME]
    p_cam_world, _ = compose(p_model, R_model, p_cam_rel, R_cam_rel)
    gt_range = float(np.linalg.norm(p_tag_world - p_cam_world))
    return p_cam_world, p_tag_world, gt_range


def compute_command(detected: bool, meas: Measurement, alt_m):
    """The M3 control law -- see the module docstring. Returns
    (v_forward, v_right, v_down, yawspeed_deg_s). All-zero (hover) when the
    latest measurement isn't usable (no detection, or stale)."""
    if not detected:
        return 0.0, 0.0, 0.0, 0.0

    range_m = meas.range_m
    bearing_rad = meas.bearing_rad

    v_forward = _clamp(
        KP_RANGE * (range_m - STANDOFF_M), -V_BACK_MAX, V_FWD_MAX
    )
    v_right = 0.0  # yaw does the lateral centering (pursuit-style)
    if alt_m is not None:
        v_down = _clamp(KP_ALT * (alt_m - ALT_REF_M), -V_VERT_MAX, V_VERT_MAX)
    else:
        v_down = 0.0
    yawspeed_deg_s = _clamp(
        KYAW_DEG_PER_DEG * math.degrees(bearing_rad),
        -YAWSPEED_MAX_DEG_S, YAWSPEED_MAX_DEG_S,
    )
    return v_forward, v_right, v_down, yawspeed_deg_s


def write_row(writer, log_file, t, phase, detected=None, meas: "Optional[Measurement]" = None,
              cmd=None, alt_m=None, gt_cam=None, gt_tag=None, gt_range=None, settled=None):
    def fmt(value, spec="{:.4f}"):
        return "" if value is None else spec.format(value)

    meas_xyz = meas.meas_xyz if (meas is not None and detected) else None
    meas_range = meas.range_m if (meas is not None and detected) else None
    bearing_rad = meas.bearing_rad if (meas is not None and detected) else None

    row = [
        f"{t:.3f}",
        phase,
        "" if detected is None else int(detected),
        fmt(meas_xyz[0]) if meas_xyz is not None else "",
        fmt(meas_xyz[1]) if meas_xyz is not None else "",
        fmt(meas_xyz[2]) if meas_xyz is not None else "",
        fmt(meas_range),
        fmt(bearing_rad, "{:.5f}"),
        fmt(cmd[0]) if cmd is not None else "",
        fmt(cmd[1]) if cmd is not None else "",
        fmt(cmd[2]) if cmd is not None else "",
        fmt(cmd[3]) if cmd is not None else "",
        fmt(alt_m),
        fmt(gt_cam[0]) if gt_cam is not None else "",
        fmt(gt_cam[1]) if gt_cam is not None else "",
        fmt(gt_cam[2]) if gt_cam is not None else "",
        fmt(gt_tag[0]) if gt_tag is not None else "",
        fmt(gt_tag[1]) if gt_tag is not None else "",
        fmt(gt_tag[2]) if gt_tag is not None else "",
        fmt(gt_range),
        "" if settled is None else int(settled),
    ]
    writer.writerow(row)
    log_file.flush()


async def main():
    started = time.monotonic()

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = os.path.join(LOGS_DIR, f"m3_intercept_{timestamp}.csv")

    node = Node()

    print(f"[m3] Reading camera intrinsics from {CAMERA_INFO_TOPIC} ...")
    try:
        fx, fy, cx, cy, width, height = get_camera_intrinsics(node, CAMERA_INFO_TIMEOUT_S)
    except (RuntimeError, queue.Empty) as exc:
        print(f"[m3] FAILED: could not get camera intrinsics: {exc}")
        return 1
    print(
        f"[m3] Intrinsics: fx={fx:.3f} fy={fy:.3f} cx={cx:.1f} cy={cy:.1f} "
        f"({width}x{height})"
    )

    tracker = PoseTracker()
    pose_sub = node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v)
    if not pose_sub:
        print(f"[m3] FAILED: could not subscribe to {POSE_TOPIC}")
        return 1

    frame_holder = LatestFrame()
    image_sub = node.subscribe(Image, IMAGE_TOPIC, frame_holder.on_image)
    if not image_sub:
        print(f"[m3] FAILED: could not subscribe to {IMAGE_TOPIC}")
        node.unsubscribe(POSE_TOPIC)
        return 1

    print(f"[m3] Subscribed to {IMAGE_TOPIC} and {POSE_TOPIC}")
    print(f"[m3] Logging to {log_path}")
    print("[m3] Waiting for the pose topic to populate...")
    deadline = time.monotonic() + POSE_WAIT_TIMEOUT_S
    while not tracker.latest and time.monotonic() < deadline:
        time.sleep(0.1)
    if not tracker.latest:
        print(f"[m3] FAILED: no pose data received within {POSE_WAIT_TIMEOUT_S}s")
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        return 1

    meas_holder = MeasurementHolder()
    stop_event = threading.Event()
    detector_thread = threading.Thread(
        target=detection_loop,
        args=(frame_holder, meas_holder, fx, fy, cx, cy, stop_event),
        daemon=True,
    )
    detector_thread.start()
    print("[m3] Detection thread started.")

    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(CSV_HEADER)
    log_file.flush()

    drone = System()
    state = TelemetryState()
    tracker_tasks = []

    settled = False
    settled_at_s = None
    aborted = False
    abort_reason = ""
    final_gt_range = float("nan")
    final_err = float("nan")
    n_ticks = 0
    n_detected_ticks = 0

    try:
        print(f"[m3] Connecting to {SYSTEM_ADDRESS} (timeout {CONNECT_TIMEOUT_S}s)...")
        await drone.connect(system_address=SYSTEM_ADDRESS)
        await wait_for_connection(drone, CONNECT_TIMEOUT_S)
        print("[m3] Connected.")

        tracker_tasks = [
            asyncio.create_task(track_position(drone, state)),
            asyncio.create_task(track_flight_mode(drone, state)),
            asyncio.create_task(track_armed(drone, state)),
            asyncio.create_task(track_landed_state(drone, state)),
        ]

        print(
            "[m3] Waiting for health (is_global_position_ok, is_home_position_ok), "
            f"timeout {HEALTH_TIMEOUT_S}s..."
        )
        await wait_for_health(drone, HEALTH_TIMEOUT_S)
        print("[m3] Health OK.")

        print(f"[m3] Setting takeoff altitude to {ALT_REF_M} m...")
        await drone.action.set_takeoff_altitude(ALT_REF_M)

        print("[m3] Arming...")
        await drone.action.arm()

        print("[m3] Commanding takeoff...")
        await drone.action.takeoff()

        success_altitude = ALT_REF_M * ALTITUDE_SUCCESS_FRACTION
        print(
            f"[m3] Waiting for relative_altitude_m >= {success_altitude:.2f} m, "
            f"timeout {ALTITUDE_TIMEOUT_S}s..."
        )
        altitude_reached = False
        deadline = time.monotonic() + ALTITUDE_TIMEOUT_S
        while time.monotonic() < deadline:
            detected, meas = sample_measurement(meas_holder)
            gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
            write_row(
                writer, log_file, time.monotonic() - started, "TAKEOFF",
                detected=detected, meas=meas, cmd=None,
                alt_m=state.relative_altitude_m,
                gt_cam=gt_cam, gt_tag=gt_tag, gt_range=gt_range, settled=0,
            )
            if (
                state.relative_altitude_m is not None
                and state.relative_altitude_m >= success_altitude
            ):
                altitude_reached = True
                break
            await asyncio.sleep(1.0 / LOG_POLL_HZ)

        if not altitude_reached:
            raise RuntimeError(
                f"altitude target not reached within {ALTITUDE_TIMEOUT_S}s "
                f"(last relative_altitude_m={state.relative_altitude_m})"
            )
        print(f"[m3] Altitude reached: {state.relative_altitude_m:.2f} m")

        print("[m3] Entering OFFBOARD (streaming a zero setpoint first)...")
        offboard_ok = True
        try:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await drone.offboard.start()
        except OffboardError as error:
            print(f"[m3] FAILED: offboard start failed: {error}")
            offboard_ok = False

        if offboard_ok:
            print("[m3] OFFBOARD active. Running guidance loop...")
            dt = 1.0 / CONTROL_RATE_HZ
            offboard_start_mono = time.monotonic()
            acquired = False
            last_success_mono = offboard_start_mono
            settle_start_mono = None
            phase = "GUIDE"
            hold_start_mono = None
            gt_range_samples = []

            while True:
                tick_start = time.monotonic()
                detected, meas = sample_measurement(meas_holder)

                if detected:
                    last_success_mono = tick_start
                    if not acquired:
                        acquired = True
                        acquire_s = tick_start - offboard_start_mono
                        print(
                            f"[m3] Tag acquired at t={acquire_s:.2f}s "
                            f"(range={meas.range_m:.2f} m)"
                        )

                if not acquired and (tick_start - offboard_start_mono) > ACQUIRE_TIMEOUT_S:
                    aborted = True
                    abort_reason = f"failed to acquire tag within {ACQUIRE_TIMEOUT_S}s"
                elif acquired and (tick_start - last_success_mono) > LOST_TAG_ABORT_S:
                    aborted = True
                    abort_reason = f"lost tag for more than {LOST_TAG_ABORT_S}s"
                elif detected and meas.range_m < RANGE_ABORT_M:
                    aborted = True
                    abort_reason = (
                        f"measured range {meas.range_m:.2f} m < abort threshold "
                        f"{RANGE_ABORT_M} m"
                    )
                elif phase == "GUIDE" and (tick_start - offboard_start_mono) > OFFBOARD_TIMEOUT_S:
                    aborted = True
                    abort_reason = f"did not settle within {OFFBOARD_TIMEOUT_S}s"

                if aborted:
                    print(f"[m3] ABORT: {abort_reason}")
                    break

                cmd = compute_command(detected, meas, state.relative_altitude_m)
                try:
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(*cmd))
                except OffboardError as error:
                    aborted = True
                    abort_reason = f"set_velocity_body failed: {error}"
                    print(f"[m3] ABORT: {abort_reason}")
                    break

                gt_cam, gt_tag, gt_range = ground_truth_world_points(tracker)
                if phase == "HOLD" and gt_range is not None:
                    gt_range_samples.append(gt_range)

                if phase == "GUIDE":
                    if detected and abs(meas.range_m - STANDOFF_M) < SETTLE_TOL_M:
                        if settle_start_mono is None:
                            settle_start_mono = tick_start
                        elif tick_start - settle_start_mono >= SETTLE_DURATION_S:
                            settled = True
                    else:
                        settle_start_mono = None

                n_ticks += 1
                if detected:
                    n_detected_ticks += 1

                write_row(
                    writer, log_file, time.monotonic() - started, phase,
                    detected=detected, meas=meas, cmd=cmd,
                    alt_m=state.relative_altitude_m,
                    gt_cam=gt_cam, gt_tag=gt_tag, gt_range=gt_range,
                    settled=settled,
                )

                if phase == "GUIDE" and settled:
                    settled_at_s = tick_start - offboard_start_mono
                    print(
                        f"[m3] Settled at t={settled_at_s:.2f}s (within "
                        f"{SETTLE_TOL_M} m of {STANDOFF_M} m standoff for "
                        f"{SETTLE_DURATION_S}s continuous). Holding to measure..."
                    )
                    phase = "HOLD"
                    hold_start_mono = tick_start

                if phase == "HOLD" and (time.monotonic() - hold_start_mono) >= HOLD_MEASURE_S:
                    break

                elapsed = time.monotonic() - tick_start
                await asyncio.sleep(max(0.0, dt - elapsed))

            if gt_range_samples:
                final_gt_range = float(np.mean(gt_range_samples))
                final_err = abs(final_gt_range - STANDOFF_M)
        else:
            aborted = True
            abort_reason = "offboard start failed"

        print("[m3] Guidance phase complete. Stopping offboard and landing...")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"[m3] WARNING: offboard.stop() failed (may not have been active): {error}")

        await drone.action.land()

        print(f"[m3] Waiting for landed/disarmed, timeout {LAND_TIMEOUT_S}s...")
        landed = False
        deadline = time.monotonic() + LAND_TIMEOUT_S
        while time.monotonic() < deadline:
            if state.landed_state == LandedState.ON_GROUND or state.armed is False:
                landed = True
                break
            await asyncio.sleep(1.0 / LOG_POLL_HZ)
        if not landed:
            print(
                f"[m3] WARNING: vehicle did not report landed/disarmed within "
                f"{LAND_TIMEOUT_S}s (last landed_state={state.landed_state}, "
                f"armed={state.armed})"
            )
        else:
            print("[m3] Landed / disarmed.")

        coverage = n_detected_ticks / n_ticks if n_ticks else 0.0
        print(
            f"[m3] n_ticks={n_ticks} detection_coverage={coverage:.3f} "
            f"settled={settled} settled_at_s="
            f"{'' if settled_at_s is None else f'{settled_at_s:.2f}'} "
            f"final_gt_range={final_gt_range:.3f} m final_err={final_err:.3f} m "
            f"aborted={aborted} log={log_path}"
        )

        # Gate: must have settled, must not have aborted (including an abort
        # during the HOLD window itself), and the ground-truth standoff
        # error over the hold window must clear the M3 threshold. NaN
        # (no ground-truth samples collected) correctly fails via isnan.
        passed = bool(
            settled and not aborted and not math.isnan(final_err)
            and final_err < FINAL_ERR_THRESHOLD_M
        )
        if passed:
            print("[m3] M3 static intercept check PASSED.")
            return 0
        else:
            print(
                f"[m3] FAILED: need settled=True (got {settled}), aborted=False "
                f"(got {aborted}), final_err < {FINAL_ERR_THRESHOLD_M} m "
                f"(got {final_err:.3f} m)"
            )
            return 1

    except asyncio.TimeoutError as exc:
        print(f"[m3] FAILED: timed out waiting for a stage: {exc}")
        return 1
    except ActionError as exc:
        print(f"[m3] FAILED: MAVSDK action error: {exc}")
        return 1
    except RuntimeError as exc:
        print(f"[m3] FAILED: {exc}")
        return 1
    except Exception as exc:  # noqa: BLE001 - top-level gate script, log and exit
        print(f"[m3] FAILED: unexpected error: {exc!r}")
        return 1
    finally:
        for task in tracker_tasks:
            task.cancel()
        for task in tracker_tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception:
                pass
        stop_event.set()
        detector_thread.join(timeout=2.0)
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        log_file.close()
        print(f"[m3] Log written to {log_path}")


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
