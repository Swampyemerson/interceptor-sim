"""Markerless NN seeker (Stage-B nano-NN lane, ADR-0033 / seeker_design_brief.md).

WHAT THIS IS
------------
A drop-in replacement for the AprilTag detector inside the guidance loop's
`detection_loop` (m3_static_intercept.py:233-290). It looks at a raw onboard
camera frame, finds the *target drone's body* with a pre-trained lightweight
neural detector (YOLOv8n, COCO), and emits the SAME measurement the guidance
already consumes: a horizontal **bearing** (primary -- drives pro-nav LOS rate),
a **coarse range** (known-size scaling), and a **confidence**. No fiducial.

WHY A NANO NN (and its honest limits)
-------------------------------------
The brief (seeker_design_brief.md sec 2.2 / 6) calls for a pre-built lightweight
detector needing minimal adaptation. YOLOv8n is 3.15 M params / ~12.8 MB ONNX,
runs on a Pi-5-class CPU (~13 fps) and Pi5+Hailo-8 (~35 fps end-to-end in the
detect-then-track stack, ADR-0015). We run it here with onnxruntime on CPU.

HONESTY / no-cheat boundary (CLAUDE.md, ADR-0010): this class reads ONLY the
camera frame + fixed intrinsics. It never touches any `gt_*` pose. Range is a
coarse known-size estimate (carry 15-30% sigma, brief sec 3); steering is
bearing-only, which is what pro-nav actually needs (a_cmd = N*Vc*lambda_dot).

The COCO model has no "drone" class; a small quad most often reads as
**airplane** (or occasionally bird/kite). That is expected -- the point of this
lane is to prove the inference + bearing-extraction path with real, honest
latency/params numbers, and to give a clean seam where a *fine-tuned* nano drone
detector drops in unchanged. See eval_nn.py for the measured detection rate,
latency, and the body-not-tag evidence.

INTERFACE CONTRACT (brief sec 1)
--------------------------------
`SeekerDetection` mirrors `m3_static_intercept.Measurement` field-for-field, so
`SeekerDetection.as_measurement_tuple()` constructs the exact object the guidance
publishes. Optical frame is OpenCV: x right, y down, z forward.
`bearing_rad > 0` == target is RIGHT of boresight (load-bearing sign, brief 1.1).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

import numpy as np

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "nn_seeker needs opencv + onnxruntime from the ISOLATED seeker venv. "
        "Run with /home/emerson/interceptor-sim/.venv-seeker/bin/python "
        "(never the project .venv)."
    ) from exc

# ---- Camera intrinsics (configs/camera_intrinsics.json; gz_x500_mono_cam) -----------
FX = 539.9363327026367
FY = 539.9363708496094
CX = 640.0
CY = 480.0
IMG_W = 1280
IMG_H = 960

# ---- Target known size for coarse range (models/fpv_target/model.sdf) -------
# Arms/props reach ~0.50 m radial -> full prop-tip span ~1.0 m; a detector box
# encloses that span. Cross-check: frame 540 box=178 px at tag-range 3.2 m
# implies fx*W/w = 3.2 -> W = 1.05 m. We use 1.0 m and DISCLOSE ~15-30% range
# sigma (brief sec 3 / ADR-0015 row 8). Range only scales Vc + trips gates.
TARGET_SPAN_M = 1.0

# ---- COCO classes a small dark quad can plausibly trigger -------------------
# (index -> name). Full list lives in _COCO80; these are the ones we accept as a
# candidate target. A fine-tuned single-class drone model would drop this gate.
_COCO80 = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]
# Airborne / small-object classes a drone-vs-sky most often reads as.
_TARGET_CLASSES = {
    _COCO80.index(n): n for n in ("airplane", "bird", "kite", "frisbee",
                                  "sports ball", "boat", "surfboard")
}


@dataclass
class SeekerDetection:
    """Standalone result, field-compatible with m3.Measurement (brief 1.1).

    meas_xyz is target position in the camera OPTICAL frame (x right, y down,
    z forward). All target fields are None when nothing was detected."""
    t_mono: float
    range_m: Optional[float]
    bearing_rad: Optional[float]          # horizontal, + = target RIGHT
    bearing_vert_rad: Optional[float]     # vertical, + = target DOWN (bonus)
    meas_xyz: Optional[np.ndarray]
    decision_margin: Optional[float]      # detector confidence (0..1) in the
    #                                       Measurement.decision_margin slot
    n_detections: int
    # Extras beyond the guidance Measurement contract. NOT "debug-only" anymore
    # (stale comment fixed, ADR-0058 review): box_xywh is READ by the
    # detect-then-track wrapper (detect_track.py) to seed/validate its visual
    # tracker, and class_name doubles as the measurement-provenance tag m4 logs
    # as meas_source ("track" = tracker box via box_to_detection, an NN class
    # name = fresh NN detection). The guidance MATH still never reads either.
    box_xywh: Optional[Tuple[float, float, float, float]] = None
    class_name: Optional[str] = None

    def as_measurement_tuple(self):
        """The exact positional args for m3_static_intercept.Measurement(...)."""
        return (self.t_mono, self.range_m, self.bearing_rad, self.meas_xyz,
                self.decision_margin, self.n_detections)


class NNSeeker:
    """YOLOv8n(COCO)-ONNX markerless seeker. Load once, call detect() per frame.

    Parameters
    ----------
    weights : path to the .onnx (default scripts/seeker/weights/yolov8n.onnx)
    conf_thres : min detector score to accept a box
    max_box_frac : reject any box whose area exceeds this fraction of the frame.
        A small drone subtends a bounded angular size; a near-full-frame
        "airplane" box is the horizon line, not the target -- this gate removes
        that false positive (measured: horizon box ~0.55 area vs drone ~0.02).
    self_mask_margin : STATIC SELF-OCCLUSION MASK. The interceptor's own rotor
        arms/props are rigidly body-fixed and always intrude from the FOV
        periphery (they hug the frame edges/corners); a COCO detector reads
        those dark high-contrast blades as "airplane" (measured: conf 0.79 on
        frame 0's own left arm). A real seeker masks the known self-occlusion
        region for exactly this reason. We reject any candidate box that
        touches within this many px of the L/R/top border. The target itself is
        interior at acquisition (frame 540 box x=626), so this keeps every real
        target detection and drops the self-detections.
    """

    def __init__(self, weights: str = None, conf_thres: float = 0.15,
                 max_box_frac: float = 0.25, input_size: int = 640,
                 intra_threads: int = 2, self_mask_margin: int = 6,
                 max_aspect: float = 3.0,
                 fx: float = FX, fy: float = FY, cx: float = CX, cy: float = CY,
                 target_span_m: float = TARGET_SPAN_M,
                 restrict_classes: bool = True):
        import os
        import onnxruntime as ort
        if weights is None:
            weights = os.path.join(os.path.dirname(__file__), "weights",
                                   "yolov8n.onnx")
        so = ort.SessionOptions()
        so.intra_op_num_threads = intra_threads   # keep machine load modest
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(
            weights, so, providers=["CPUExecutionProvider"])
        self.inp_name = self.sess.get_inputs()[0].name
        self.conf_thres = conf_thres
        self.max_box_frac = max_box_frac
        self.self_mask_margin = self_mask_margin
        self.max_aspect = max_aspect
        self.sz = input_size
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.target_span_m = target_span_m
        self.restrict_classes = restrict_classes

    # -- preprocessing: letterbox to a square, keep aspect ratio --------------
    def _letterbox(self, img):
        h, w = img.shape[:2]
        r = min(self.sz / h, self.sz / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        resized = cv2.resize(img, (nw, nh))
        canvas = np.full((self.sz, self.sz, 3), 114, np.uint8)
        dw, dh = (self.sz - nw) // 2, (self.sz - nh) // 2
        canvas[dh:dh + nh, dw:dw + nw] = resized
        return canvas, r, dw, dh

    def _candidates(self, img_bgr) -> List[Tuple[float, str, Tuple]]:
        """Return [(score, class_name, (x,y,w,h))] passing conf + size + class
        + ROI gates, sorted highest score first (image-pixel coords)."""
        H, W = img_bgr.shape[:2]
        lb, r, dw, dh = self._letterbox(img_bgr)
        blob = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[None]
        out = self.sess.run(None, {self.inp_name: blob})[0][0].T  # (8400, 84)
        cls = out[:, 4:].argmax(1)
        score = out[:, 4:].max(1)
        keep = np.where(score > self.conf_thres)[0]
        res = []
        for i in keep:
            ci = int(cls[i])
            if self.restrict_classes and ci not in _TARGET_CLASSES:
                continue
            cx, cy, bw, bh = out[i, :4]
            x = (cx - bw / 2 - dw) / r
            y = (cy - bh / 2 - dh) / r
            ww, hh = bw / r, bh / r
            if ww * hh > self.max_box_frac * W * H:   # horizon-reject
                continue
            # static self-occlusion mask: drop own rotor arms at FOV periphery
            m = self.self_mask_margin
            if x <= m or (x + ww) >= (W - m) or y <= m:
                continue
            # aspect-ratio gate: the interceptor's own rotor blades read as a
            # long thin wedge (aspect ~4-8:1); the target quad body is compact
            # (measured ~1.4-1.7:1). Reject elongated blade false positives.
            aspect = max(ww, hh) / max(min(ww, hh), 1.0)
            if aspect > self.max_aspect:
                continue
            name = _COCO80[ci] if ci < len(_COCO80) else str(ci)
            res.append((float(score[i]), name, (float(x), float(y),
                                                float(ww), float(hh))))
        res.sort(key=lambda z: z[0], reverse=True)
        return res

    def _box_to_geometry(self, box):
        """box (x,y,w,h) in image px -> (bearing_h, bearing_v, range, xyz)."""
        x, y, w, h = box
        u = x + w / 2.0
        v = y + h / 2.0
        # bearing in the optical frame: ray = ((u-cx)/fx, (v-cy)/fy, 1)
        bearing_h = math.atan2((u - self.cx) / self.fx, 1.0)   # + = right
        bearing_v = math.atan2((v - self.cy) / self.fy, 1.0)   # + = down
        # coarse known-size range: range ~= fx * W_real / w_pixels (pinhole).
        # Use the larger box side vs span (props tip-to-tip), guard tiny boxes.
        w_px = max(w, h, 1.0)
        range_m = self.fx * self.target_span_m / w_px
        # target position in optical frame from range * unit ray
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        ray = ray / np.linalg.norm(ray)
        xyz = range_m * ray
        return bearing_h, bearing_v, range_m, xyz

    def detect(self, img_bgr, t_mono: Optional[float] = None) -> SeekerDetection:
        """Run the seeker on ONE BGR frame. Returns a SeekerDetection with the
        highest-confidence plausible target box converted to bearing/range, or
        a None-filled detection when nothing passed the gates."""
        if t_mono is None:
            t_mono = time.monotonic()
        cands = self._candidates(img_bgr)
        if not cands:
            return SeekerDetection(t_mono, None, None, None, None, None, 0)
        score, name, box = cands[0]
        bh, bv, rng, xyz = self._box_to_geometry(box)
        return SeekerDetection(
            t_mono=t_mono, range_m=rng, bearing_rad=bh, bearing_vert_rad=bv,
            meas_xyz=xyz, decision_margin=score, n_detections=len(cands),
            box_xywh=box, class_name=name,
        )
