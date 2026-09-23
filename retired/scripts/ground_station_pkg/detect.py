#!/usr/bin/env python3
r"""T19 -- the ground station's own detection wrapper: ground_v1.onnx (ADR-0047
single-class "drone" detector, ADR-0049 gate PASS) run via onnxruntime,
returning a pixel box + confidence per image. This is a SMALL, PURPOSE-BUILT
port of the same box-postprocessing convention `scripts/seeker/
finetuned_seeker.py`'s `FinetunedNNSeeker` uses (letterbox -> single-class
YOLO cx/cy/w/h/score columns -> best-confidence box) -- NOT an import from
`scripts/seeker/` (this task's scope explicitly keeps `scripts/seeker/`
untouched, and that module's self-mask/bearing/known-size-range logic is
monocular-airborne-seeker-specific, not needed by a static ground rig).

Verified against the T18 real-detector reference (`.venv-seeker`, comparing
this module's box centers on `logs/rig_captures/full_sweep_20260709T015530Z/`
frames to `logs/t18_nn_sigma_validation/nn_detections_per_frame.csv`'s
ultralytics-derived `det_u`/`det_v`): sub-0.02 px agreement, e.g. seq=15
left det_u 456.7556 here vs 456.7567 there. Small residual differences are
resize/interpolation implementation details (cv2 letterbox here vs
ultralytics' own preprocessing there), not a detection discrepancy.

DEPS: onnxruntime + opencv + numpy (`.venv-seeker` has all three, PLUS
gz-transport13 -- the one venv on this checkout with both halves T19 needs;
see `station.py`'s module docstring for the full interpreter finding). The
import is DEFERRED to `GroundDetector.__init__` (not module level) so this
file -- and anything that imports it -- stays importable under a venv
without onnxruntime/opencv (e.g. the main `.venv` pytest runs under for
`tests/test_ground_station.py`'s synthetic-centroid tests, which never
construct a real `GroundDetector`).

HONESTY: reads only image pixels + the fixed camera intrinsics baked into
the ONNX/letterbox math -- no `gt_*`, no gz subscription of any kind. Same
boundary `finetuned_seeker.py`/`nn_seeker.py` document for the airborne
seekers: the fine-tune's labels came from gt OFFLINE at training time
(ADR-0047/ADR-0049), the live detector at inference is gt-free.
"""
from __future__ import annotations

import os
from typing import NamedTuple, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.dirname(HERE)
REPO_ROOT = os.path.dirname(SCRIPTS_DIR)

DEFAULT_WEIGHTS = os.path.join(SCRIPTS_DIR, "seeker", "weights", "ground_v1.onnx")
DEFAULT_CONF = 0.25   # matches eval_ground_detector.py / t18_nn_sigma_validation.py defaults
DEFAULT_IMGSZ = 960   # matches ground_v1.onnx's own input shape (1,3,960,960)


class DetectionResult(NamedTuple):
    """Mirrors `t18_nn_sigma_validation.run_detector()`'s per-frame output
    shape `(hit, conf, box_xyxy)` -- same downstream `box_center()` call."""
    hit: bool
    conf: Optional[float]
    box_xyxy: Optional[Tuple[float, float, float, float]]


def box_center(box_xyxy) -> Tuple[float, float]:
    """Identical convention to `t18_nn_sigma_validation.box_center()`:
    (x0, y0, x1, y1) -> ((x0+x1)/2, (y0+y1)/2)."""
    x0, y0, x1, y1 = box_xyxy
    return (x0 + x1) / 2.0, (y0 + y1) / 2.0


def _letterbox(img, new_size):
    """Resize to a square `new_size` keeping aspect ratio (grey pad). Same
    algorithm as `scripts/seeker/finetuned_seeker.py::_letterbox` (a common,
    standard YOLO-preprocessing recipe -- reproduced, not imported, per this
    task's "do not touch scripts/seeker/" scope). Returns
    `(padded_img, scale, (pad_left, pad_top))`."""
    import cv2

    h, w = img.shape[:2]
    r = min(new_size / h, new_size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = (new_size - nw) / 2, (new_size - nh) / 2
    top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
    left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
    out = cv2.copyMakeBorder(resized, top, bottom, left, right,
                              cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out, r, (left, top)


class GroundDetector:
    """onnxruntime-backed single-class ("drone") detector for `ground_v1.onnx`.
    `.detect(frame_bgr) -> DetectionResult` / `.detect_path(path) ->
    DetectionResult` -- the ONLY two entry points `station.py` calls, so a
    test can substitute a stub object with the same two methods without
    ever importing onnxruntime/cv2."""

    def __init__(self, weights: str = DEFAULT_WEIGHTS, conf: float = DEFAULT_CONF,
                 imgsz: int = DEFAULT_IMGSZ, intra_op_num_threads: int = 2):
        import onnxruntime as ort  # deferred -- see module docstring

        self.conf = conf
        self.imgsz = imgsz
        so = ort.SessionOptions()
        so.intra_op_num_threads = intra_op_num_threads
        self.sess = ort.InferenceSession(weights, so, providers=["CPUExecutionProvider"])
        self.iname = self.sess.get_inputs()[0].name
        shp = self.sess.get_inputs()[0].shape
        try:
            self.imgsz = int(shp[-1]) if isinstance(shp[-1], int) else imgsz
        except Exception:
            pass
        self.weights_path = weights
        print(f"[ground-detect] loaded {os.path.basename(weights)} "
              f"imgsz={self.imgsz} conf={self.conf}")

    def detect_path(self, path: str) -> DetectionResult:
        import cv2

        img = cv2.imread(path)
        if img is None:
            return DetectionResult(False, None, None)
        return self.detect(img)

    def detect(self, frame_bgr) -> DetectionResult:
        import numpy as np

        if frame_bgr is None:
            return DetectionResult(False, None, None)
        img, r, (pad_l, pad_t) = _letterbox(frame_bgr, self.imgsz)
        blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run(None, {self.iname: np.ascontiguousarray(blob)})[0]
        preds = out[0]                       # (5, N) or (N, 5)
        if preds.shape[0] < preds.shape[1]:  # (5, N) -> (N, 5)
            preds = preds.T
        # columns: cx, cy, w, h, class0_score (single class, ADR-0047)
        best_score = 0.0
        best_box = None
        for row in preds:
            cx_, cy_, w_, h_, score = (
                float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4])
            )
            if score < self.conf or score <= best_score:
                continue
            bw = w_ / r
            bh = h_ / r
            u = (cx_ - pad_l) / r
            v = (cy_ - pad_t) / r
            best_score = score
            best_box = (u - bw / 2.0, v - bh / 2.0, u + bw / 2.0, v + bh / 2.0)
        if best_box is None:
            return DetectionResult(False, None, None)
        return DetectionResult(True, best_score, best_box)
