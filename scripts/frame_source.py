#!/usr/bin/env python3
"""Frame-source abstraction: the seam that lets the SAME detection + guidance
run against the Gazebo sim OR a real onboard camera.

Everything above the camera (AprilTag detection, range/bearing extraction,
pro-nav guidance, MAVSDK offboard) only ever needs two things from "the
camera": a stream of grayscale frames, and the pinhole intrinsics
(fx, fy, cx, cy) to turn tag pixels into a metric pose. This module defines
that minimal interface (`FrameSource`) and two implementations:

  - `GzFrameSource`     -- sim: subscribes to the gz-transport Image +
                           CameraInfo topics (wraps what m2/m3/m4 do inline).
  - `OpenCVFrameSource` -- hardware: an OpenCV VideoCapture (USB/UVC, or a
                           CSI camera exposed through v4l2/libcamera) plus a
                           calibrated intrinsics JSON on disk.

The deployable on-drone entry point (fly_intercept.py) constructs an
`OpenCVFrameSource`; the sim gates construct a `GzFrameSource`. The guidance
code is identical either way -- that identity is the whole point of the
sim-to-hardware port (GOALS.md's parent-project translation table).

Intrinsics JSON format matches camera_intrinsics.json (the sim's recorded
values): a dict with "fx", "fy", "cx", "cy", and a "resolution":{"width",
"height"}. calibrate_camera.py writes this format for a REAL camera; the sim
value 539.9 is only valid for the sim camera and MUST be replaced by a real
calibration before flying hardware.
"""

import json
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class Intrinsics:
    """Pinhole camera intrinsics (OpenCV convention) + the resolution they
    were calibrated at. fx/fy in pixels; cx/cy the principal point;
    dist_coeffs the radial/tangential lens distortion (k1,k2,p1,p2[,k3]).

    The sim renders an IDEAL pinhole (dist_coeffs all ~0), so pupil-apriltags
    can pose-estimate straight off (fx,fy,cx,cy). A REAL lens distorts, and
    pupil-apriltags has no distortion model -- so on hardware the frame must
    be undistorted first (OpenCVFrameSource does this when dist_coeffs are
    non-negligible), which makes the pinhole (fx,fy,cx,cy) valid again."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist_coeffs: Optional[Tuple[float, ...]] = None

    @property
    def params(self) -> Tuple[float, float, float, float]:
        """(fx, fy, cx, cy) -- the tuple pupil-apriltags' detect() wants.
        Valid for pose estimation only on an undistorted (or ideal-pinhole)
        image; see the class docstring."""
        return (self.fx, self.fy, self.cx, self.cy)

    @property
    def has_distortion(self) -> bool:
        """True if the calibrated distortion is large enough to matter (a
        real lens); False for the sim's ideal pinhole (all ~0)."""
        return self.dist_coeffs is not None and any(
            abs(c) > 1e-4 for c in self.dist_coeffs
        )

    @classmethod
    def from_json(cls, path: str) -> "Intrinsics":
        with open(path) as f:
            d = json.load(f)
        res = d.get("resolution", {})
        dc = d.get("dist_coeffs")
        return cls(
            fx=float(d["fx"]), fy=float(d["fy"]),
            cx=float(d["cx"]), cy=float(d["cy"]),
            width=int(res.get("width", 0)), height=int(res.get("height", 0)),
            dist_coeffs=tuple(float(x) for x in dc) if dc else None,
        )


class FrameSource:
    """Interface: yield the newest grayscale frame + its capture time.

    read() returns (gray uint8 HxW ndarray, capture_monotonic_s) or
    (None, None) if no new frame is available yet. Callers keep the newest
    frame only (latest-wins) -- the detection loop must never fall behind
    processing stale frames (same rationale as m3/m4's LatestFrame).
    """

    @property
    def intrinsics(self) -> Intrinsics:
        raise NotImplementedError

    def read(self) -> Tuple[Optional[np.ndarray], Optional[float]]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class OpenCVFrameSource(FrameSource):
    """Real-camera source: an OpenCV VideoCapture + a calibrated intrinsics
    JSON. Covers USB/UVC webcams directly and most CSI cameras through
    v4l2/libcamera (the device string / GStreamer pipeline is passed in, so
    the same class serves a Pi Camera, a USB cam, or an analog cam behind a
    USB capture stick -- whatever the deployed hardware ends up being).

    Runs its own grabber thread so the guidance loop always gets the freshest
    frame with no queue backlog (V4L2 buffers otherwise hand you stale frames
    at high loop rates). read() is non-blocking and returns the latest grab.
    """

    def __init__(self, device, intrinsics_path: str, *,
                 width: Optional[int] = None, height: Optional[int] = None,
                 fourcc: Optional[str] = None, undistort: bool = True):
        import cv2  # local import: hardware-only dep, keeps sim import light

        self._cv2 = cv2
        self._intrinsics = Intrinsics.from_json(intrinsics_path)
        # Precompute an undistortion remap once (cheap per-frame cv2.remap
        # thereafter) when the lens has real distortion. Keeps the same
        # (fx,fy,cx,cy) valid for pupil-apriltags' distortion-free pose solve.
        self._remap = None
        if undistort and self._intrinsics.has_distortion:
            K = np.array([[self._intrinsics.fx, 0, self._intrinsics.cx],
                          [0, self._intrinsics.fy, self._intrinsics.cy],
                          [0, 0, 1]], dtype=np.float64)
            D = np.array(self._intrinsics.dist_coeffs, dtype=np.float64)
            size = (self._intrinsics.width, self._intrinsics.height)
            m1, m2 = cv2.initUndistortRectifyMap(K, D, None, K, size, cv2.CV_16SC2)
            self._remap = (m1, m2)
            print("[frame_source] real lens distortion -> undistorting frames")
        # `device` may be an int index (0), a /dev/videoN path, or a full
        # GStreamer pipeline string (CSI/libcamera) -- VideoCapture accepts
        # all three; we don't guess, we pass it through.
        self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            raise RuntimeError(f"could not open camera device {device!r}")
        if fourcc:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        # Request the calibrated resolution unless overridden; a mismatch
        # between capture and calibration resolution silently corrupts the
        # pose scale, so we assert it after opening.
        req_w = width or self._intrinsics.width
        req_h = height or self._intrinsics.height
        if req_w:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, req_w)
        if req_h:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, req_h)
        # Minimise driver-side buffering so read() returns fresh frames.
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        import threading
        self._lock = threading.Lock()
        self._latest = (None, None)  # (gray, t)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

    def _grab_loop(self) -> None:
        cv2 = self._cv2
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            t = time.monotonic()
            if frame.ndim == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame
            if self._remap is not None:
                gray = cv2.remap(gray, self._remap[0], self._remap[1],
                                 cv2.INTER_LINEAR)
            with self._lock:
                self._latest = (gray, t)

    def check_resolution(self) -> None:
        """Assert the actual capture resolution matches the calibration.
        Call once after construction (fly_intercept does): a capture/calib
        resolution mismatch scales the tag pose wrong and would silently
        ruin every range measurement."""
        aw = int(self._cap.get(self._cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(self._cap.get(self._cv2.CAP_PROP_FRAME_HEIGHT))
        iw, ih = self._intrinsics.width, self._intrinsics.height
        if iw and ih and (aw, ah) != (iw, ih):
            raise RuntimeError(
                f"camera capture resolution {aw}x{ah} != calibration "
                f"resolution {iw}x{ih}; recalibrate at the capture resolution "
                "or set width/height to match (pose scale depends on it)"
            )

    @property
    def intrinsics(self) -> Intrinsics:
        return self._intrinsics

    def read(self):
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._cap.release()


class GzFrameSource(FrameSource):
    """Sim source: wraps the gz-transport Image + CameraInfo subscriptions
    that m2/m3/m4 currently do inline. Kept thin -- it exists so the sim and
    hardware present the identical FrameSource interface to the guidance
    core. Imports gz only when constructed (hardware never imports it)."""

    def __init__(self, node, image_topic: str, camera_info_topic: str,
                 info_timeout_s: float = 30.0):
        from gz.transport13 import Node  # noqa: F401  (type hint / availability)
        from gz.msgs10.image_pb2 import Image, RGB_INT8
        from gz.msgs10.camera_info_pb2 import CameraInfo
        import cv2
        import queue

        self._cv2 = cv2
        self._RGB_INT8 = RGB_INT8
        self._Image = Image

        # One-shot CameraInfo read for intrinsics.
        q: "queue.Queue[CameraInfo]" = queue.Queue()
        node.subscribe(CameraInfo, camera_info_topic, lambda m: q.put(m))
        try:
            info = q.get(timeout=info_timeout_s)
        finally:
            node.unsubscribe(camera_info_topic)
        k = info.intrinsics.k
        self._intrinsics = Intrinsics(
            fx=k[0], fy=k[4], cx=k[2], cy=k[5],
            width=info.width, height=info.height,
        )

        import threading
        self._lock = threading.Lock()
        self._latest = (None, None)
        self._image_topic = image_topic
        self._node = node
        node.subscribe(Image, image_topic, self._on_image)

    def _on_image(self, msg) -> None:
        if msg.pixel_format_type != self._RGB_INT8:
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)
        )
        gray = self._cv2.cvtColor(rgb, self._cv2.COLOR_RGB2GRAY)
        with self._lock:
            self._latest = (gray, time.monotonic())

    @property
    def intrinsics(self) -> Intrinsics:
        return self._intrinsics

    def read(self):
        with self._lock:
            return self._latest

    def close(self) -> None:
        try:
            self._node.unsubscribe(self._image_topic)
        except Exception:
            pass
