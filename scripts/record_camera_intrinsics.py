#!/usr/bin/env python3
"""One-off utility: subscribe once to the mono camera's CameraInfo topic and
save fx, fy, cx, cy to camera_intrinsics.json. See GOALS.md milestone M2.

What: gz-transport publishes a gz.msgs10.CameraInfo message alongside the
Image stream, on the sibling topic .../sensor/imager/camera_info. Its `K`
matrix (named `intrinsics.k`, row-major 3x3) already has the pinhole
intrinsics Gazebo's renderer actually used -- fx, fy (focal lengths in
pixels) and cx, cy (principal point) -- so we read them directly instead of
re-deriving them from the declared horizontal_fov (that derivation is only
a cross-check, done below and printed, not the source of truth).

Why this matters: scripts/m2_detect.py needs (fx, fy, cx, cy) to hand to
pupil-apriltags' `estimate_tag_pose` (a standard pinhole camera model) --
get that wrong and every pose comes out mis-scaled.

Run manually (with PX4 SITL + gz_x500_mono_cam + the apriltag world already
running -- see worlds/apriltag.sdf for the launch recipe):

    .venv/bin/python scripts/record_camera_intrinsics.py
"""

import json
import math
import os
import queue
import sys
from datetime import datetime, timezone

from gz.transport13 import Node
from gz.msgs10.camera_info_pb2 import CameraInfo

WORLD_NAME = "apriltag"
DRONE_MODEL = "x500_mono_cam_0"
CAMERA_LINK = "camera_link"
CAMERA_INFO_TOPIC = (
    f"/world/{WORLD_NAME}/model/{DRONE_MODEL}/link/{CAMERA_LINK}"
    "/sensor/imager/camera_info"
)

# Declared in ~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf
# -- used only as a cross-check against the K matrix actually measured, not
# as the source of truth (see docstring above).
DECLARED_HFOV_RAD = 1.74
DECLARED_WIDTH = 1280

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(REPO_ROOT, "camera_intrinsics.json")


def main():
    print(f"[record_camera_intrinsics] Subscribing to {CAMERA_INFO_TOPIC}")
    q: "queue.Queue[CameraInfo]" = queue.Queue()

    def on_info(msg: CameraInfo) -> None:
        q.put(msg)

    node = Node()
    subscribed = node.subscribe(CameraInfo, CAMERA_INFO_TOPIC, on_info)
    if not subscribed:
        print(f"[record_camera_intrinsics] FAILED: could not subscribe to {CAMERA_INFO_TOPIC}")
        return 1

    try:
        msg = q.get(timeout=30)
    except queue.Empty:
        print("[record_camera_intrinsics] FAILED: no CameraInfo message within 30s")
        return 1
    finally:
        node.unsubscribe(CAMERA_INFO_TOPIC)

    k = msg.intrinsics.k
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]

    fx_cross_check = (DECLARED_WIDTH / 2) / math.tan(DECLARED_HFOV_RAD / 2)
    print(f"[record_camera_intrinsics] Measured: fx={fx:.4f} fy={fy:.4f} cx={cx:.1f} cy={cy:.1f}")
    print(
        f"[record_camera_intrinsics] Cross-check from declared hfov="
        f"{DECLARED_HFOV_RAD} rad, width={DECLARED_WIDTH}: "
        f"fx_est=(width/2)/tan(hfov/2)={fx_cross_check:.4f}"
    )

    record = {
        "resolution": {"width": msg.width, "height": msg.height},
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "fx_cross_check_from_declared_hfov": fx_cross_check,
        "declared_hfov_rad": DECLARED_HFOV_RAD,
        "source_topic": CAMERA_INFO_TOPIC,
        "recorded_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }

    with open(OUT_PATH, "w") as f:
        json.dump(record, f, indent=2)
        f.write("\n")
    print(f"[record_camera_intrinsics] Wrote {OUT_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
