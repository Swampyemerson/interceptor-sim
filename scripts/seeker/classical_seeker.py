"""Classical (no-NN) markerless seeker -- Stage A of ADR-0033 ("kill the AprilTag").

This is layer 1 of the parent design's detect-then-track pipeline (ADR-0015 s2):
a cheap classical detector that finds the TARGET DRONE'S OWN BODY (silhouette)
against sky / ground clutter -- NO fiducial -- and emits the SAME measurement the
guidance already consumes (see the interface contract in
docs/seeker_design_brief.md s1 and the `Measurement` dataclass at
scripts/m3_static_intercept.py:188).

What it detects and WHY it is not the tag
-----------------------------------------
The interceptor's onboard camera sees the target as a compact DARK, near-neutral
(low-saturation) blob against a much brighter sky (V~200) and a brighter green
ground (V~150). The airframe interior in the demo footage is a uniform ~gray-49
body with ZERO AprilTag black/white checker content (measured: tag-white fraction
0.000 inside the detected blob). This detector thresholds for the DARK SILHOUETTE,
which is the opposite of what a tag decoder keys on (a tag is ~50% WHITE and would
FRAGMENT a dark blob). Locking a solid dark wedge is therefore body detection, not
tag detection -- see eval_classical.py for the explicit masked-tag / real-tag-
detector cross-checks.

Emitted measurement (mirrors scripts/m3_static_intercept.py Measurement):
  t_mono, range_m (COARSE, known-size scaling; None if no detection),
  bearing_rad (PRIMARY: +=target right of boresight, atan2(x, z) optical frame),
  bearing_vert_rad (down positive), meas_xyz (optical frame x-right/y-down/z-fwd),
  decision_margin (repurposed as a 0..1 detector confidence), n_detections.

Range is deliberately coarse: the terminal miss is ~96% bearing/ZEM-determined
(ADR-0023), so bearing quality dominates and a +-15-30% range only throttles Vc /
trips gates (design brief s3). Bearing is what pro-nav actually steers on.

Honesty boundary (CLAUDE.md / ADR-0010): this seeker reads ONLY camera pixels +
the fixed intrinsics. It never touches any gt_* ground-truth field.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


# ---- Target geometry (from models/fpv_target/model.sdf, read-only) ----------
# Body 0.20x0.16x0.10 m; four arms length 0.42 m -> prop tips reach radial 0.50 m
# (a ~1.0 m tip-to-tip span). We use a single characteristic frontal width for the
# known-size range scaling and DOCUMENT that box-width->range is coarse and highly
# orientation-dependent (the target is seen at varying aspect, not head-on).
TARGET_WIDTH_M = 0.50  # characteristic span; carry the +-15-30% sigma (brief s3)


@dataclass
class SeekerDetection:
    """Standalone result object, field-compatible with the guidance-side
    `Measurement` (scripts/m3_static_intercept.py:188). Adapt with
    `to_measurement_fields()` at the detection-loop seam; nothing downstream
    changes (design brief s1.2)."""

    t_mono: float
    range_m: Optional[float]
    bearing_rad: Optional[float]          # horizontal, + = right of boresight
    bearing_vert_rad: Optional[float]     # vertical, + = below boresight
    meas_xyz: Optional[np.ndarray]        # optical frame (x right, y down, z fwd)
    decision_margin: Optional[float]      # 0..1 confidence (repurposed slot)
    n_detections: int
    # --- extra diagnostics (not part of the guidance contract) ---
    box: Optional[Tuple[int, int, int, int]] = None  # (x, y, w, h) pixels
    centroid: Optional[Tuple[float, float]] = None    # (u, v) pixels
    area_px: int = 0
    source: str = "detect"               # "detect" or "track"
    body_bright_frac: float = 0.0        # fraction of tag-white (>150) pixels ON
    #                                      the silhouette itself; ~0 => not a tag

    @staticmethod
    def empty(t_mono: float) -> "SeekerDetection":
        return SeekerDetection(t_mono, None, None, None, None, None, 0)

    def detected(self) -> bool:
        return self.bearing_rad is not None

    def to_measurement_fields(self):
        """Tuple in the exact order Measurement(...) expects, so the swap is:
        `meas_holder.latest = Measurement(*det.to_measurement_fields())`."""
        return (self.t_mono, self.range_m, self.bearing_rad, self.meas_xyz,
                self.decision_margin, self.n_detections)


def load_intrinsics(path: str = "camera_intrinsics.json"):
    """Return (fx, fy, cx, cy) from the calibrated intrinsics file."""
    d = json.loads(Path(path).read_text())
    return d["fx"], d["fy"], d["cx"], d["cy"]


class ClassicalSeeker:
    """Dark-salient-blob detector + optional correlation tracker (detect-then-track).

    detect(frame_bgr, t_mono) -> SeekerDetection   (stateless per-frame detector)
    update(frame_bgr, t_mono) -> SeekerDetection   (detect-then-track wrapper:
        holds a KCF/CSRT lock between detections, re-detects periodically and on
        loss -- this is the "track" layer that coasts through brief detector gaps,
        matching the latest-wins + alpha-beta coast the guidance already relies on).
    """

    def __init__(self, fx: float, fy: float, cx: float, cy: float,
                 dark_thresh: int = 110, sat_max: int = 70,
                 min_area: int = 120, max_area_frac: float = 0.35,
                 tracker: str = "KCF", redetect_every: int = 12):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.dark_thresh = dark_thresh
        self.sat_max = sat_max
        self.min_area = min_area
        self.max_area_frac = max_area_frac
        self.tracker_kind = tracker
        self.redetect_every = redetect_every
        # detect-then-track state
        self._tracker = None
        self._since_detect = 0
        self._last_box = None

    # ------------------------------------------------------------------ detect
    def _target_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Binary mask of DARK, LOW-SATURATION pixels = the airframe silhouette.

        Sky (V~200) and green ground (V~150) are both far brighter than the body
        (V~50-80), and the body is near-neutral (S~15) while the ground is green
        (S~54); the dark+desaturated AND gate rejects both backgrounds. This keys
        on the BODY, never on a (bright, high-contrast) tag."""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]
        mask = ((gray < self.dark_thresh) & (sat < self.sat_max)).astype(np.uint8) * 255
        # close small gaps, then open to drop specks
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        return mask, gray

    def detect(self, frame_bgr: np.ndarray, t_mono: Optional[float] = None,
               ignore_mask: Optional[np.ndarray] = None) -> SeekerDetection:
        """Per-frame stateless detection. `ignore_mask` (uint8, nonzero = ignore)
        lets the eval blank out a region -- used for the masked-tag body-not-tag
        test."""
        if t_mono is None:
            t_mono = time.monotonic()
        mask, gray = self._target_mask(frame_bgr)
        if ignore_mask is not None:
            mask[ignore_mask > 0] = 0
        H, W = gray.shape
        max_area = self.max_area_frac * H * W

        n, lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
        best = None
        for c in range(1, n):
            a = int(stats[c, cv2.CC_STAT_AREA])
            if a < self.min_area or a > max_area:
                continue
            x, y, w, h = (int(stats[c, cv2.CC_STAT_LEFT]),
                          int(stats[c, cv2.CC_STAT_TOP]),
                          int(stats[c, cv2.CC_STAT_WIDTH]),
                          int(stats[c, cv2.CC_STAT_HEIGHT]))
            # darkness contrast vs a dilated ring of local background
            blob = (lab == c)
            ring = cv2.dilate(blob.astype(np.uint8),
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
            ring = (ring > 0) & (~blob)
            bg = gray[ring]
            fg = gray[blob]
            if bg.size < 10:
                continue
            contrast = float(np.clip((bg.mean() - fg.mean()) / 128.0, 0.0, 1.0))
            score = contrast * math.log10(a + 10)
            # tag-white ON the silhouette itself (NOT the axis-aligned box, whose
            # corners are background sky): a fiducial lock would be ~0.5 here.
            body_bright = float((fg > 150).mean())
            cand = dict(area=a, box=(x, y, w, h), cent=(float(cent[c, 0]),
                        float(cent[c, 1])), contrast=contrast, score=score,
                        body_bright=body_bright)
            if best is None or score > best["score"]:
                best = cand

        if best is None:
            return SeekerDetection.empty(t_mono)
        return self._make_detection(best, t_mono, source="detect")

    def _make_detection(self, cand: dict, t_mono: float, source: str) -> SeekerDetection:
        x, y, w, h = cand["box"]
        u, v = cand["cent"]
        # bearing: optical frame, + = right / down; atan2(x_norm, z=1)
        bearing = math.atan2((u - self.cx) / self.fx, 1.0)
        bearing_v = math.atan2((v - self.cy) / self.fy, 1.0)
        # COARSE range via known-size scaling (design brief s3, option 1).
        # Orientation-dependent: uses apparent width as a frontal-width proxy.
        rng = self.fx * TARGET_WIDTH_M / max(w, 1)
        z = rng / math.sqrt(1.0 + math.tan(bearing) ** 2 + math.tan(bearing_v) ** 2)
        meas_xyz = np.array([z * math.tan(bearing), z * math.tan(bearing_v), z],
                            dtype=float)
        conf = float(np.clip(cand.get("contrast", 0.5) *
                             (0.5 + 0.5 * min(1.0, cand["area"] / 4000.0)), 0.0, 1.0))
        return SeekerDetection(
            t_mono=t_mono, range_m=float(rng), bearing_rad=float(bearing),
            bearing_vert_rad=float(bearing_v), meas_xyz=meas_xyz,
            decision_margin=conf, n_detections=1, box=(x, y, w, h),
            centroid=(u, v), area_px=cand["area"], source=source,
            body_bright_frac=cand.get("body_bright", 0.0))

    # -------------------------------------------------------- detect-then-track
    def _new_tracker(self):
        if self.tracker_kind == "CSRT":
            return cv2.TrackerCSRT_create()
        if self.tracker_kind == "MIL":
            return cv2.TrackerMIL_create()
        return cv2.TrackerKCF_create()

    def update(self, frame_bgr: np.ndarray, t_mono: Optional[float] = None) -> SeekerDetection:
        """Detect-then-track: run the tracker if we hold a lock, else (re)detect.
        Re-detects every `redetect_every` frames to correct drift and re-acquire."""
        if t_mono is None:
            t_mono = time.monotonic()
        force_detect = (self._tracker is None or
                        self._since_detect >= self.redetect_every)
        if not force_detect:
            ok, box = self._tracker.update(frame_bgr)
            self._since_detect += 1
            if ok:
                x, y, w, h = [int(round(b)) for b in box]
                cand = dict(area=max(1, w * h), box=(x, y, w, h),
                            cent=(x + w / 2.0, y + h / 2.0), contrast=0.5)
                self._last_box = (x, y, w, h)
                return self._make_detection(cand, t_mono, source="track")
            # tracker lost -> fall through to detect
            self._tracker = None
        # (re)detect and (re)init the tracker
        det = self.detect(frame_bgr, t_mono)
        self._since_detect = 0
        if det.detected():
            self._tracker = self._new_tracker()
            try:
                self._tracker.init(frame_bgr, tuple(det.box))
            except cv2.error:
                self._tracker = None
            self._last_box = det.box
        else:
            self._tracker = None
        return det
