#!/usr/bin/env python3
"""M2 gate script: detect the AprilTag live and compare against sim ground
truth. See docs/goals.md milestone M2, models/apriltag_target/model.sdf (tag
geometry) and worlds/apriltag.sdf (world/topic names).

What: subscribes to three gz-transport topics from a running PX4 SITL +
gz_x500_mono_cam + apriltag-world instance --
  - the mono camera's Image stream (same decode as scripts/m1_capture.py)
  - its sibling CameraInfo (for the pinhole intrinsics fx, fy, cx, cy)
  - the world's Pose_V stream (every model/link's pose -- this is our
    ground truth, "cheating" only in the sense that M2 is explicitly a
    perception-accuracy check against known truth, not a guidance run)
Per frame: runs pupil-apriltags (ADR-0003) on the grayscale image, and
independently computes the true camera->tag relative position from the
pose topic, then logs both plus the error between them.

GROUND TRUTH TRANSFORM CHAIN (Fable reviews this -- it is the #1 sign-error
risk zone per docs/next.md). /world/apriltag/pose/info reports each entity's
pose RELATIVE TO ITS IMMEDIATE PARENT, except for top-level models (no
parent, so their reported pose already IS the world pose).

IMPORTANT CORRECTION found while validating M2 against the detector itself
(2026-07-04, logged in docs/decisions.md): a first draft assumed
"camera_link" was reported relative to "base_link" (chaining base_link's
own (0, 0, 0.24) model-relative offset underneath it), because both that
guess AND the correct answer happen to print the identical number
(0.12, 0.03, 0.242) for camera_link -- a coincidence of this specific SDF,
not evidence either way. That draft placed the camera ~0.24 m too high,
which showed up as a near-constant ~0.25 m bias concentrated in the
optical-Y (vertical) residual between measured and ground truth across
every frame (a constant, not noisy, error -- the tell that this was a
transform bug, not sensor/detector error).

Root cause, traced back to the SDF (not just live numbers, since a
"same number, different parent" coincidence like this can bite twice):
~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam/model.sdf does

    <include merge='true'><uri>x500</uri></include>
    <include merge='true'><uri>model://mono_cam</uri>
      <pose>.12 .03 .242 0 0 0</pose></include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent><child>camera_link</child>
      <pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
    </joint>

The `<pose>` on the merged `<include>` sets camera_link's placement
relative to the ENCLOSING model (x500_mono_cam) directly -- merge='true'
flattens every sub-model's links into one flat sibling list under the top
model, each with its own pose relative to that top model, not nested
link-to-link. The CameraJoint's `relative_to="base_link"` is a physics/
constraint-frame declaration for the (fixed, immovable) joint; it does not
re-parent or re-stack camera_link's actual spatial pose in Pose_V. Likewise
base_link's own (0, 0, 0.24) comes from x500_base/model.sdf's top-level
`<model><pose>0 0 .24 0 0 0</pose>`, applied by the SAME merge mechanism --
base_link and camera_link are siblings under one model, not parent/child.
So the real chain is:

    1. T_world_model = pose of model "x500_mono_cam_0"   (top-level -> world frame already)
    2. T_model_cam   = pose of link  "camera_link"        (relative to the model directly --
                                                             base_link is NOT in this chain)
    3. T_world_cam = compose(T_world_model, T_model_cam)
    4. T_world_tag = pose of model "apriltag_target"      (top-level -> world frame already;
                                                             its own link "base" has zero
                                                             offset, so the model pose IS the
                                                             tag-center world position)

(base_link is still read from the pose topic and logged/available -- e.g.
for future milestones -- but is deliberately NOT part of the camera chain;
see PoseTracker.ground_truth_rel_optical.)

Then, still in world/ENU axes:
    gt_rel_world = p_tag_world - p_cam_world

Rotate that into the camera_link's own body axes (gz sensor convention:
x-forward, y-left, z-up) using camera_link's full WORLD rotation
(R_world_cam, composed the same way as the position above):
    gt_rel_camlink = R_world_cam^T @ gt_rel_world

Finally apply the fixed camlink->optical axis permutation (docs/goals.md:
camera/optical frame is OpenCV convention, z-forward/x-right/y-down; gz's
sensor frame is x-forward/y-left/z-up):
    x_opt = -y_link
    y_opt = -z_link
    z_opt = +x_link
This is what pupil-apriltags' `pose_t` is expressed in (a standard pinhole/
OpenCV camera model), so gt_rel_optical is directly comparable to it.

Quaternion -> rotation matrix: implemented by hand below (`quat_to_matrix`)
rather than via scipy. Decision (see docs/decisions.md / task report): the
project already depends on gz's own protobuf/transport bindings and
pupil-apriltags; adding scipy for one 9-term formula would grow the
dependency surface for no real benefit (docs/goals.md: "keep the dependency
surface minimal"), and the formula is short and easy to unit-test/verify.

Ground truth deliberately reuses the SAME pose topic every frame rather
than trusting the mount-offset constants as fixed literals -- if the model
geometry ever changes, this script keeps working without edits.

Run manually (with PX4 SITL + gz_x500_mono_cam + the apriltag world already
running -- see worlds/apriltag.sdf for the exact launch env):

    .venv/bin/python scripts/m2_detect.py --duration 15

Normally this is invoked by scripts/check_m2.sh, which also launches and
tears down the simulator.
"""

import argparse
import csv
import os
import queue
import sys
import time
from datetime import datetime, timezone

import cv2
import numpy as np
from apriltag_detector import Detector  # portable: pupil-apriltags (x86) / pyapriltags (aarch64), ADR-0012

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image, RGB_INT8
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.pose_v_pb2 import Pose_V

# INTERCEPTOR_WORLD_NAME env var override (default "apriltag" -- byte-
# identical to every gated caller, none of which sets this var): lets the
# demo-video tooling point this SAME detector at worlds/apriltag_demo.sdf
# (world name "apriltag_demo") without forking the script. m4_intercept.py
# imports IMAGE_TOPIC/CAMERA_INFO_TOPIC/POSE_TOPIC from this module, and
# spawns scripts/m4_target_mover.py / scripts/s2_cue_mock.py as child
# processes that inherit this same env var (each has its own matching
# override -- see their WORLD_NAME comments), so setting this once before
# launch retargets the whole pipeline at once.
WORLD_NAME = os.environ.get("INTERCEPTOR_WORLD_NAME", "apriltag")
DRONE_MODEL = "x500_mono_cam_0"
# base_link is NOT used in the ground-truth chain -- see the module
# docstring's "IMPORTANT CORRECTION". Kept as a named constant only because
# it is a real, useful entity in the pose topic (e.g. for future milestones
# that need the vehicle body frame, not just the camera).
BASE_LINK_NAME = "base_link"
CAMERA_LINK_NAME = "camera_link"
# INTERCEPTOR_TARGET_MODEL retargets the ground-truth target entity looked up in
# the pose topic (default "apriltag_target", byte-identical for every gated
# caller). ADR-0033 item 2 sets it to "fpv_target_markerless" for the markerless
# A/B so gt_* scoring resolves the tag-less body -- gt stays SCORING-only either
# way. m3/m4 import this name, so the one env read propagates to the whole gt path.
TAG_MODEL_NAME = os.environ.get("INTERCEPTOR_TARGET_MODEL", "apriltag_target")

# The imager <sensor> can be TILTED inside camera_link (an up-tilt mount, e.g.
# scripts/experiments/uptilt_mounts/up35/mono_cam/model.sdf, applied via the
# models/mono_cam shadow in uptilt_run.sh). That tilt is NOT observable from the
# pose topic -- /world/<w>/pose/info publishes the model and camera_link poses
# but NOT the sensor's own pose inside camera_link -- so ground_truth_rel_optical
# would otherwise project every gt box as if the sensor sat LEVEL in camera_link.
# That silent bug made the up35 capture's gt labels level-projected and unusable
# (Fable round-4 check, ADR-0076 add #18k). INTERCEPTOR_SENSOR_PITCH_RAD carries
# the SDF sensor <pose> PITCH (5th number, rad; NEGATIVE = boresight UP, matching
# the SDF sign) so the transform can rotate gt into the ACTUAL sensor frame.
SENSOR_PITCH_RAD = float(os.environ.get("INTERCEPTOR_SENSOR_PITCH_RAD", "0") or "0")


def _rot_y(theta: float) -> np.ndarray:
    """Rotation about the +y axis (gz sensor/camera_link y is LEFT)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# Map a camera_link-frame vector into the (possibly tilted) sensor frame. The
# sensor->camera_link rotation is Rot_y(pitch); its transpose Rot_y(-pitch) takes
# a camera_link vector to the sensor frame. None (identity) for the level default,
# so gated callers are byte-identical unless they opt in via the env var.
_R_CAMLINK_TO_SENSOR = _rot_y(-SENSOR_PITCH_RAD) if SENSOR_PITCH_RAD else None

IMAGE_TOPIC = (
    f"/world/{WORLD_NAME}/model/{DRONE_MODEL}/link/{CAMERA_LINK_NAME}"
    "/sensor/imager/image"
)
CAMERA_INFO_TOPIC = (
    f"/world/{WORLD_NAME}/model/{DRONE_MODEL}/link/{CAMERA_LINK_NAME}"
    "/sensor/imager/camera_info"
)
POSE_TOPIC = f"/world/{WORLD_NAME}/pose/info"

# From ~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf.
EXPECTED_WIDTH = 1280
EXPECTED_HEIGHT = 960

# See models/apriltag_target/model.sdf for the 0.625 m plane / 0.5 m
# black-square derivation.
TAG_SIZE_M = 0.5
TAG_FAMILY = "tag36h11"

DEFAULT_DURATION_S = 15.0
CAMERA_INFO_TIMEOUT_S = 30.0
FRAME_WAIT_TIMEOUT_S = 30.0

# Gate thresholds (docs/goals.md milestone M2).
DETECTION_RATE_THRESHOLD = 0.90
MEAN_ERR_NORM_THRESHOLD_M = 0.25

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS_DIR = os.path.join(REPO_ROOT, "logs")


def image_msg_to_gray(msg: Image) -> np.ndarray:
    """Decode a gz.msgs10 Image into an 8-bit grayscale numpy array.

    Same RGB_INT8, no-row-padding assumption as scripts/m1_capture.py's
    image_msg_to_bgr -- see that file for why we fail loudly instead of
    guessing if the wire format ever changes.
    """
    if msg.pixel_format_type != RGB_INT8:
        raise ValueError(
            f"unexpected pixel_format_type {msg.pixel_format_type} "
            f"(expected RGB_INT8={RGB_INT8})"
        )
    expected_step = msg.width * 3
    if msg.step != expected_step:
        raise ValueError(
            f"unexpected step {msg.step} for width {msg.width} "
            f"(expected {expected_step}, i.e. no row padding)"
        )
    rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Hand-rolled quaternion -> 3x3 rotation matrix (no scipy -- see the
    module docstring for why). Standard Hamilton convention (x, y, z, w),
    matching gz.msgs.Quaternion field order and ROS/robotics convention.

    Returns R such that, for a vector v expressed in the CHILD frame this
    quaternion rotates FROM, `R @ v` gives that same vector expressed in
    the PARENT frame: v_parent = R @ v_child. This is the standard
    "rotation matrix from quaternion" formula (e.g. Diebel 2006, eq. 125;
    also matches Eigen::Quaterniond::toRotationMatrix()).
    """
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_to_pt(pose) -> tuple:
    """Extract (position np.array(3,), rotation 3x3 matrix) from a
    gz.msgs10 Pose entry."""
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    R = quat_to_matrix(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    )
    return p, R


def compose(p_parent, R_parent, p_rel, R_rel):
    """Compose a child's pose (given relative to its parent) with the
    parent's own pose (given relative to world) to get the child's pose
    relative to world. See the module docstring's transform-chain comment."""
    p_world = p_parent + R_parent @ p_rel
    R_world = R_parent @ R_rel
    return p_world, R_world


class PoseTracker:
    """Latest-known named poses from /world/<world>/pose/info.

    gz-transport subscribe() callbacks fire on a background C++ thread
    (same pattern noted in scripts/m1_capture.py). We just replace one
    plain dict reference per message -- a single Python attribute
    assignment is atomic under the GIL, so the main thread can read
    `tracker.latest` at any time without an explicit lock, matching the
    TelemetryState pattern in scripts/m0_takeoff.py.
    """

    def __init__(self):
        self.latest = {}
        self.count = 0

    def on_pose_v(self, msg: Pose_V) -> None:
        self.latest = {p.name: pose_to_pt(p) for p in msg.pose}
        self.count += 1

    def ground_truth_rel_optical(self):
        """Return (gt_rel_optical np.array(3,), ok bool, reason str).

        NOTE: base_link is intentionally NOT part of this chain -- see the
        module docstring's "IMPORTANT CORRECTION" for why camera_link is
        composed directly against the model, not via base_link, even
        though camera_link's own numbers happen to look like they could be
        relative to either.
        """
        names = (DRONE_MODEL, CAMERA_LINK_NAME, TAG_MODEL_NAME)
        latest = self.latest
        missing = [n for n in names if n not in latest]
        if missing:
            return None, False, f"pose topic missing entities: {missing}"

        p_model, R_model = latest[DRONE_MODEL]
        p_cam_rel, R_cam_rel = latest[CAMERA_LINK_NAME]
        p_tag_world, _ = latest[TAG_MODEL_NAME]

        p_cam_world, R_cam_world = compose(p_model, R_model, p_cam_rel, R_cam_rel)

        gt_rel_world = p_tag_world - p_cam_world
        gt_rel_camlink = R_cam_world.T @ gt_rel_world

        # Apply the imager's own mount tilt inside camera_link (invisible in the
        # pose topic; injected via INTERCEPTOR_SENSOR_PITCH_RAD) so a tilted
        # camera's gt boxes are projected in the ACTUAL sensor frame, not as if
        # it were level (Fable round-4 check, ADR-0076 add #18k). Identity no-op
        # for the default level mount.
        gt_rel_sensor = gt_rel_camlink if _R_CAMLINK_TO_SENSOR is None \
            else _R_CAMLINK_TO_SENSOR @ gt_rel_camlink

        # gz sensor frame (x-fwd, y-left, z-up) -> OpenCV optical frame
        # (z-fwd, x-right, y-down). See module docstring.
        gt_rel_optical = np.array(
            [-gt_rel_sensor[1], -gt_rel_sensor[2], gt_rel_sensor[0]]
        )
        return gt_rel_optical, True, ""


def get_camera_intrinsics(node: Node, timeout_s: float):
    """Subscribe once to CameraInfo and return (fx, fy, cx, cy, width, height)."""
    q: "queue.Queue[CameraInfo]" = queue.Queue()

    def on_info(msg: CameraInfo) -> None:
        q.put(msg)

    subscribed = node.subscribe(CameraInfo, CAMERA_INFO_TOPIC, on_info)
    if not subscribed:
        raise RuntimeError(f"could not subscribe to {CAMERA_INFO_TOPIC}")
    try:
        msg = q.get(timeout=timeout_s)
    finally:
        node.unsubscribe(CAMERA_INFO_TOPIC)
    k = msg.intrinsics.k
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    return fx, fy, cx, cy, msg.width, msg.height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration", type=float, default=DEFAULT_DURATION_S,
        help="how long to run the detection loop, in seconds",
    )
    args = parser.parse_args()

    os.makedirs(LOGS_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = os.path.join(LOGS_DIR, f"m2_detect_{timestamp}.csv")

    node = Node()

    print(f"[m2] Reading camera intrinsics from {CAMERA_INFO_TOPIC} ...")
    try:
        fx, fy, cx, cy, width, height = get_camera_intrinsics(node, CAMERA_INFO_TIMEOUT_S)
    except (RuntimeError, queue.Empty) as exc:
        print(f"[m2] FAILED: could not get camera intrinsics: {exc}")
        return 1
    print(f"[m2] Intrinsics: fx={fx:.3f} fy={fy:.3f} cx={cx:.1f} cy={cy:.1f} ({width}x{height})")

    tracker = PoseTracker()
    pose_sub = node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v)
    if not pose_sub:
        print(f"[m2] FAILED: could not subscribe to {POSE_TOPIC}")
        return 1

    frame_queue: "queue.Queue[Image]" = queue.Queue()

    def on_image(msg: Image) -> None:
        frame_queue.put(msg)

    image_sub = node.subscribe(Image, IMAGE_TOPIC, on_image)
    if not image_sub:
        print(f"[m2] FAILED: could not subscribe to {IMAGE_TOPIC}")
        node.unsubscribe(POSE_TOPIC)
        return 1

    print(f"[m2] Subscribed to {IMAGE_TOPIC} and {POSE_TOPIC}")
    print(f"[m2] Logging to {log_path}")
    print(f"[m2] Waiting for the pose topic to populate...")
    deadline = time.monotonic() + FRAME_WAIT_TIMEOUT_S
    while not tracker.latest and time.monotonic() < deadline:
        time.sleep(0.1)
    if not tracker.latest:
        print(f"[m2] FAILED: no pose data received within {FRAME_WAIT_TIMEOUT_S}s")
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        return 1

    detector = Detector(families=TAG_FAMILY)

    n_frames = 0
    n_detected = 0
    err_norms = []
    ranges = []

    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(
        [
            "t", "detected", "meas_x", "meas_y", "meas_z",
            "gt_x", "gt_y", "gt_z", "err_norm", "pose_err", "decision_margin",
        ]
    )

    started = time.monotonic()
    run_deadline = started + args.duration
    try:
        while time.monotonic() < run_deadline:
            remaining = run_deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                msg = frame_queue.get(timeout=min(remaining, 1.0))
            except queue.Empty:
                continue

            t = time.monotonic() - started

            gt_rel, gt_ok, gt_reason = tracker.ground_truth_rel_optical()
            if not gt_ok:
                print(f"[m2] FAILED: {gt_reason}")
                return 1

            if msg.width != EXPECTED_WIDTH or msg.height != EXPECTED_HEIGHT:
                print(
                    f"[m2] FAILED: frame resolution {msg.width}x{msg.height} != "
                    f"expected {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}"
                )
                return 1

            try:
                gray = image_msg_to_gray(msg)
            except ValueError as exc:
                print(f"[m2] FAILED: {exc}")
                return 1

            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(fx, fy, cx, cy),
                tag_size=TAG_SIZE_M,
            )

            n_frames += 1
            if detections:
                det = detections[0]
                meas = det.pose_t.flatten()
                err_norm = float(np.linalg.norm(meas - gt_rel))
                n_detected += 1
                err_norms.append(err_norm)
                ranges.append(float(np.linalg.norm(gt_rel)))
                writer.writerow(
                    [
                        f"{t:.3f}", 1,
                        f"{meas[0]:.4f}", f"{meas[1]:.4f}", f"{meas[2]:.4f}",
                        f"{gt_rel[0]:.4f}", f"{gt_rel[1]:.4f}", f"{gt_rel[2]:.4f}",
                        f"{err_norm:.4f}", f"{det.pose_err:.6e}", f"{det.decision_margin:.3f}",
                    ]
                )
            else:
                ranges.append(float(np.linalg.norm(gt_rel)))
                writer.writerow(
                    [
                        f"{t:.3f}", 0,
                        "", "", "",
                        f"{gt_rel[0]:.4f}", f"{gt_rel[1]:.4f}", f"{gt_rel[2]:.4f}",
                        "", "", "",
                    ]
                )
            log_file.flush()

        detection_rate = n_detected / n_frames if n_frames else 0.0
        mean_err = float(np.mean(err_norms)) if err_norms else float("nan")
        max_err = float(np.max(err_norms)) if err_norms else float("nan")
        mean_range = float(np.mean(ranges)) if ranges else float("nan")

        print(
            f"[m2] n_frames={n_frames} detection_rate={detection_rate:.3f} "
            f"mean_err_norm={mean_err:.4f} m max_err_norm={max_err:.4f} m "
            f"mean_range={mean_range:.3f} m"
        )

        passed = (
            n_frames > 0
            and detection_rate >= DETECTION_RATE_THRESHOLD
            and err_norms
            and mean_err <= MEAN_ERR_NORM_THRESHOLD_M
        )
        if passed:
            print("[m2] M2 AprilTag detection check PASSED.")
            return 0
        else:
            print(
                f"[m2] FAILED: need detection_rate >= {DETECTION_RATE_THRESHOLD} "
                f"(got {detection_rate:.3f}) AND mean_err_norm <= "
                f"{MEAN_ERR_NORM_THRESHOLD_M} m (got {mean_err:.4f} m)"
            )
            return 1
    finally:
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        log_file.close()
        print(f"[m2] Log written to {log_path}")


if __name__ == "__main__":
    sys.exit(main())
