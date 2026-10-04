"""flight/deploy/frame_feed.py -- real camera frames + a real detector for the
mission driver (flight/deploy/real_flight.py `--source`).

The perception half is NOT re-implemented here: frame sources are
seeker_loop.PicameraSource / seeker_loop.ImageDirSource, the ONNX detector is
seeker_loop.build_detector, the frame/intrinsics contract is
seeker_loop.frame_shape_fault. This module only adapts them to the duck-typed
`detect(frame, t)` interface run_mavsdk_mission consumes, in the same shape as
scripts/gazebo_pursuit_crosscheck.GazeboTagDetector:

  * the driver's placeholder frame is ignored; the NEWEST source frame is used,
  * a frame already consumed is never re-reported (it would feed the terminal's
    filter one measurement twice) -- a poll with no new frame returns
    `is_new=False`, which the state machine's streak treats as HOLD,
  * every result carries the real `frame` and its capture time `t_capture`
    (mission clock) so the frame recorder records what the detector saw.

Honesty: inputs are camera pixels and the camera calibration only.
"""
from __future__ import annotations

import threading
import time
import types
from typing import Callable, Optional, Tuple

import numpy as np

from flight.deploy.seeker_loop import (
    ImageDirSource,
    PicameraSource,
    build_detector,
    frame_shape_fault,
)

TAG_FAMILY = "tag36h11"          # the family every tag instrument here decodes
FIRST_FRAME_TIMEOUT_S = 10.0


def parse_source_spec(spec: str) -> Tuple[str, Optional[str]]:
    """'picamera' -> ('picamera', None); 'dir:PATH' -> ('dir', PATH)."""
    if spec == "picamera":
        return "picamera", None
    if spec.startswith("dir:") and len(spec) > 4:
        return "dir", spec[4:]
    raise ValueError(f"--source must be 'picamera' or 'dir:PATH', got {spec!r}")


def open_source(spec: str, cam, camera_fps: float,
                exposure_us: Optional[float] = None,
                gain: Optional[float] = None):
    """Construct the seeker_loop source. Returns (kind, source).

    picamera: the calibrated resolution AND a calibration provenance stamp are
    required -- the sim intrinsics file carries neither a `source` stamp nor the
    OV9281 grid, so a live camera may not run on it.

    exposure_us/gain default to PicameraSource's own flight spec (the <=1 ms
    pinned exposure, auto gain) when None. They exist for BENCH targets whose
    illumination is nothing like daylight -- a monitor at the 1 ms spec meters
    ~1/3 the brightness the decoder needs (measured 2026-10-04: screen tag
    decodes at frame mean ~90, the 1 ms desk run delivered ~36 and 0 hits).
    The flight condition is unchanged unless a flag is passed."""
    kind, path = parse_source_spec(spec)
    if kind == "picamera":
        if cam.width is None or cam.height is None:
            raise RefusedSource(
                "--source picamera needs intrinsics with a `resolution` block "
                "(scripts/calibrate_camera.py output); this file declares none")
        if not cam.source:
            raise RefusedSource(
                "--source picamera refused: the intrinsics carry no `source` "
                "stamp, i.e. they are not a measured calibration of this lens "
                "(the default configs/camera_intrinsics.json is the Gazebo "
                "camera_info dump). Pass --intrinsics <checkerboard calibration>")
        kwargs = {}
        if exposure_us is not None:
            kwargs["exposure_us"] = exposure_us
        if gain is not None:
            kwargs["gain"] = gain
        return kind, PicameraSource(size=(cam.width, cam.height),
                                    target_fps=camera_fps, **kwargs)
    return kind, ImageDirSource(path)


class RefusedSource(RuntimeError):
    """Startup refusal: the selected source cannot be flown on these intrinsics."""


class FrameFeed:
    """Newest-frame slot over a seeker_loop source.

    threaded=True (live camera): a daemon thread drains source.frames() so the
    control loop never blocks on capture; poll() returns the newest frame not
    yet returned. threaded=False (replay): poll() pulls the next file -- one
    frame per poll, deterministic."""

    def __init__(self, source, threaded: bool,
                 clock: Callable[[], float] = time.monotonic):
        self.source = source
        self.threaded = threaded
        self.clock = clock
        self._lock = threading.Lock()
        self._slot = None            # (frame, t_capture_clock, seq)
        self._seq = 0
        self._last_seq = 0
        self._gen = None
        self._pending = None
        self._thread = None
        self._stop = threading.Event()
        self.error: Optional[BaseException] = None
        self.exhausted = False
        self.n_captured = self.n_new = self.n_stale = 0

    def start(self) -> "FrameFeed":
        if self.threaded:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        else:
            self._gen = self.source.frames()
        return self

    def _run(self):
        try:
            for frame, _name in self.source.frames():
                if self._stop.is_set():
                    return
                with self._lock:
                    self._seq += 1
                    self._slot = (frame, self.clock(), self._seq)
                    self.n_captured += 1
        except BaseException as e:  # noqa: BLE001 -- surfaced via .error
            self.error = e
        finally:
            self.exhausted = True

    def _pull(self):
        if self._pending is not None:
            item, self._pending = self._pending, None
            return item
        if self.exhausted:
            return None
        try:
            frame, _name = next(self._gen)
        except StopIteration:
            self.exhausted = True
            return None
        self._seq += 1
        self.n_captured += 1
        return (frame, self.clock(), self._seq)

    def first_frame(self, timeout_s: float = FIRST_FRAME_TIMEOUT_S):
        """The first frame, WITHOUT consuming it (poll() still returns it)."""
        if not self.threaded:
            if self._pending is None:
                self._pending = self._pull()
            return None if self._pending is None else self._pending[0]
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._slot is not None:
                    return self._slot[0]
            if self.error is not None or self.exhausted:
                return None
            time.sleep(0.01)
        return None

    def poll(self):
        """(frame, t_capture_clock) newer than the last poll, else None."""
        if self.threaded:
            with self._lock:
                item = self._slot
            if item is None or item[2] == self._last_seq:
                self.n_stale += 1
                return None
            self._last_seq = item[2]
        else:
            item = self._pull()
            if item is None:
                self.n_stale += 1
                return None
        self.n_new += 1
        return item[0], item[1]

    def stop(self):
        self._stop.set()


def preflight(feed: FrameFeed, cam) -> Optional[str]:
    """None if the source delivers frames on the calibrated pixel grid, else the
    refusal message. Run BEFORE anything connects or arms."""
    frame = feed.first_frame()
    if frame is None:
        why = f" ({type(feed.error).__name__}: {feed.error})" if feed.error else ""
        return f"the source delivered no frame{why}"
    return frame_shape_fault(frame, cam)


def _miss(**kw):
    return types.SimpleNamespace(range_m=None, box_xywh=None, tag_range_m=None,
                                 **kw)


class TagFrameDetector:
    """AprilTag tag36h11 on a real frame -> the driver's detection object.
    Box/range convention mirrors scripts/gazebo_pursuit_crosscheck.tag_to_detection
    (range_m = fx*span/box_w; tag_range_m = slant range of the tag pose). With a
    distorted lens the pose comes from cv2.solvePnP on the corners with the
    calibrated distortion (the detector's own pose assumes a pinhole)."""

    def __init__(self, cam, span_m: float, tag_id: int = 0,
                 quad_decimate: float = 2.0, nthreads: int = 4):
        try:
            from pupil_apriltags import Detector
        except ImportError:  # the Pi (aarch64) ships pyapriltags, ADR-0012
            from pyapriltags import Detector
        self.det = Detector(families=TAG_FAMILY, nthreads=nthreads,
                            quad_decimate=quad_decimate)
        self.cam = cam
        self.span = float(span_m)
        self.tag_id = int(tag_id)

    def detect(self, frame, t=None):
        import cv2
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = self.cam
        dets = self.det.detect(gray, estimate_tag_pose=not c.has_distortion,
                               camera_params=(c.fx, c.fy, c.cx, c.cy),
                               tag_size=self.span)
        hits = [d for d in dets if d.tag_id == self.tag_id]
        if not hits:
            return _miss()
        d = hits[0]
        xs, ys = d.corners[:, 0], d.corners[:, 1]
        x0, y0 = float(xs.min()), float(ys.min())
        w, h = float(np.ptp(xs)), float(np.ptp(ys))
        if c.has_distortion:
            pose_r = self._pnp_range(d.corners)
        else:
            pose_r = (float(np.linalg.norm(d.pose_t))
                      if getattr(d, "pose_t", None) is not None else None)
        return types.SimpleNamespace(range_m=c.fx * self.span / max(w, 1e-6),
                                     box_xywh=(x0, y0, w, h), tag_range_m=pose_r)

    def _pnp_range(self, corners) -> Optional[float]:
        import cv2
        s = self.span / 2.0
        # apriltag corner order: (-1,+1), (+1,+1), (+1,-1), (-1,-1) in tag frame
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                       dtype=np.float64)
        c = self.cam
        K = np.array([[c.fx, 0, c.cx], [0, c.fy, c.cy], [0, 0, 1]], dtype=np.float64)
        dist = np.array([c.k1, c.k2, c.p1, c.p2, c.k3], dtype=np.float64)
        ok, _rvec, tvec = cv2.solvePnP(obj, np.asarray(corners, dtype=np.float64),
                                       K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        return float(np.linalg.norm(tvec)) if ok else None


def build_inner_detector(args, cam, span_m: float):
    """--detector tag | onnx. onnx is seeker_loop.build_detector unchanged."""
    if args.detector == "tag":
        return TagFrameDetector(cam, span_m, tag_id=args.tag_id,
                                quad_decimate=args.quad_decimate)
    return build_detector(args, cam, span_m)


class FeedDetector:
    """`detect(_frame, t)` for run_mavsdk_mission / run_desk. Ignores the
    driver's frame; detects on the newest feed frame; a stale poll is
    `is_new=False` (no new measurement), never a repeat."""

    def __init__(self, feed: FrameFeed, inner):
        self.feed = feed
        self.inner = inner
        self.n_detect = self.n_hit = 0
        self.last = None

    def detect(self, _frame, t=None):
        item = self.feed.poll()
        if item is None:
            self.last = _miss(is_new=False, frame=None, t_capture=None)
            return self.last
        frame, t_cap_clock = item
        d = self.inner.detect(frame, t)
        self.n_detect += 1
        t_mission = 0.0 if t is None else t
        # capture time on the caller's mission clock: replay frames are read at
        # the poll, live frames aged since the capture thread stamped them
        t_capture = (t_mission - (self.feed.clock() - t_cap_clock)
                     if self.feed.threaded else t_mission)
        rng = getattr(d, "range_m", None)
        self.n_hit += rng is not None
        self.last = types.SimpleNamespace(
            range_m=rng,
            box_xywh=getattr(d, "box_xywh", None) if rng is not None else None,
            tag_range_m=getattr(d, "tag_range_m", None) if rng is not None else None,
            is_new=True, frame=frame, t_capture=t_capture)
        return self.last
