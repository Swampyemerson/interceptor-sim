#!/usr/bin/env python3
"""Shared offline ONNX inference + label/geometry helpers for the seeker-v3
forensics (Phase 0), phantom mining (build_onboard_dataset_v3.py) and the
v2-vs-v3 eval scorer (eval_seeker_v3.py). ONE place so the "deployed operating
point" is defined once and identically everywhere (plan Sec 8).

WHY a shared module: the plan's headline metric (phantom rate) and its
mining rule both key off "what fires at the DEPLOYED conf 0.25 + self-mask"
(finetuned_seeker.py). If the eval scorer and the miner decoded the ONNX or
applied the mask even slightly differently, the gates would not measure the
data the trainer actually saw. The decode + self-mask below are copied
byte-for-byte in behavior from scripts/seeker/finetuned_seeker.py
(FinetunedNNSeeker.detect / _letterbox) -- the deployed path -- and return
ALL boxes (not just the single best), which the phantom-rate count and the
cluster analysis need.

HONESTY (CLAUDE.md / ADR-0010): gt_* labels are read HERE only to SCORE and
to MANUFACTURE training data offline -- never fed to a live detector. Every
function here is offline (onnxruntime + numpy + cv2), never touches gz/mavsdk.

Boxes are CENTER-format pixel tuples (cx, cy, w, h) in ORIGINAL-frame pixels
throughout (matches the YOLO label convention once denormalized).
"""
from __future__ import annotations

import json
import math
import os
from typing import List, Optional, Tuple

import numpy as np
import cv2

# Fixed intrinsics -- configs/camera_intrinsics.json / mono_cam SDF (1280x960).
FX = FY = 539.936
CX, CY = 640.0, 480.0
IMG_W, IMG_H = 1280, 960

# Deployed operating point (finetuned_seeker.py defaults).
DEPLOY_CONF = 0.25
DEPLOY_MAX_BEARING_DEG = 30.0
DEPLOY_EDGE_MARGIN_PX = 40.0
DEFAULT_IMGSZ = 640
DEFAULT_SPAN_M = 1.0  # nn_seeker known-size default (overridden by calib sidecar)


def _letterbox(img, new_size: int):
    """Square-resize keeping aspect with gray pad -- byte-identical to
    finetuned_seeker._letterbox. Returns (padded, scale r, (pad_l, pad_t))."""
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


def load_session(weights: str, intra_threads: int = 2):
    """Return (session, input_name, imgsz). imgsz inferred from the ONNX
    input shape (fallback DEFAULT_IMGSZ), same as FinetunedNNSeeker.__init__."""
    import onnxruntime as ort
    so = ort.SessionOptions()
    so.intra_op_num_threads = intra_threads
    sess = ort.InferenceSession(weights, so, providers=["CPUExecutionProvider"])
    iname = sess.get_inputs()[0].name
    shp = sess.get_inputs()[0].shape
    imgsz = DEFAULT_IMGSZ
    try:
        if isinstance(shp[-1], int):
            imgsz = int(shp[-1])
    except Exception:
        pass
    return sess, iname, imgsz


def load_span(weights: str) -> Tuple[float, str]:
    """Resolve the effective range-calibration span for `weights`, in the
    SAME order as FinetunedNNSeeker: MARKERLESS_SPAN_M env, then a
    `<weights>.calib.json` sidecar, then the 1.0 m default."""
    env_span = os.environ.get("MARKERLESS_SPAN_M")
    if env_span:
        try:
            return float(env_span), f"env {env_span}"
        except ValueError:
            pass
    calib = str(weights) + ".calib.json"
    if os.path.exists(calib):
        try:
            with open(calib) as fh:
                return float(json.load(fh)["span_m_eff"]), f"calib {os.path.basename(calib)}"
        except Exception:
            pass
    return DEFAULT_SPAN_M, "default"


def infer_raw(sess, iname, imgsz: int, frame_bgr, conf: float = DEPLOY_CONF
              ) -> List[dict]:
    """Run the ONNX on one BGR frame and return ALL detections above `conf`,
    mask-OFF, as dicts {cx, cy, w, h, score} in ORIGINAL-frame pixels. Decode
    is byte-identical to FinetunedNNSeeker.detect (letterbox undo included).
    Handles both (5,N) and (N,5) single-class YOLO outputs."""
    H, W = frame_bgr.shape[:2]
    img, r, (pad_l, pad_t) = _letterbox(frame_bgr, imgsz)
    blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    out = sess.run(None, {iname: np.ascontiguousarray(blob)})[0]
    preds = out[0]
    if preds.shape[0] < preds.shape[1]:   # (5,N) -> (N,5)
        preds = preds.T
    dets = []
    for row in preds:
        cx_, cy_, w_, h_, score = (float(row[0]), float(row[1]), float(row[2]),
                                   float(row[3]), float(row[4]))
        if score < conf:
            continue
        bw = w_ / r
        bh = h_ / r
        u = (cx_ - pad_l) / r
        v = (cy_ - pad_t) / r
        dets.append({"cx": u, "cy": v, "w": bw, "h": bh, "score": score})
    return dets


def apply_self_mask(dets: List[dict], W: int = IMG_W, H: int = IMG_H,
                    max_bearing_deg: float = DEPLOY_MAX_BEARING_DEG,
                    edge_margin_px: float = DEPLOY_EDGE_MARGIN_PX,
                    fx: float = FX, cx: float = CX) -> List[dict]:
    """Deployed self-mask (finetuned_seeker.detect): drop any detection whose
    box-center bearing exceeds max_bearing_deg OR whose box touches within
    edge_margin_px of the L/R frame edge. This is the post-mask stream that
    feeds handoff -- the plan scores M1/M2 on THIS stream."""
    max_bear = max_bearing_deg * math.pi / 180.0
    kept = []
    for d in dets:
        u, bw = d["cx"], d["w"]
        bearing = math.atan2((u - cx) / fx, 1.0)
        if abs(bearing) > max_bear:
            continue
        if (u - bw / 2.0) < edge_margin_px or (u + bw / 2.0) > (W - edge_margin_px):
            continue
        kept.append(d)
    return kept


def best_by_score(dets: List[dict]) -> Optional[dict]:
    return max(dets, key=lambda d: d["score"]) if dets else None


def range_from_box(w_px: float, span_m: float, fx: float = FX) -> float:
    """Known-size range = fx*span/box_width_px (finetuned_seeker geometry)."""
    return fx * span_m / max(w_px, 1.0)


def bearing_deg_of(u: float, fx: float = FX, cx: float = CX) -> float:
    return math.degrees(math.atan2((u - cx) / fx, 1.0))


def load_gt_box(label_path: str, W: int = IMG_W, H: int = IMG_H
                ) -> Optional[Tuple[float, float, float, float]]:
    """Read a YOLO label file -> gt box (cx, cy, w, h) in PIXELS, or None if
    the file is empty/missing (a negative/empty frame). Uses the first
    non-blank line (single-class 'drone')."""
    if not os.path.isfile(label_path):
        return None
    with open(label_path) as fh:
        lines = [ln for ln in fh.read().splitlines() if ln.strip()]
    if not lines:
        return None
    parts = lines[0].split()
    if len(parts) < 5:
        return None
    _, ncx, ncy, nw, nh = parts[:5]
    return (float(ncx) * W, float(ncy) * H, float(nw) * W, float(nh) * H)


def _to_xyxy(box: Tuple[float, float, float, float]):
    cx, cy, w, h = box
    return cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0


def iou(box_a, box_b) -> float:
    """IoU of two center-format pixel boxes (cx, cy, w, h)."""
    ax0, ay0, ax1, ay1 = _to_xyxy(box_a)
    bx0, by0, bx1, by1 = _to_xyxy(box_b)
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    ua = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return inter / ua if ua > 0 else 0.0


def center_err(box_a, box_b) -> float:
    """Center-to-center pixel distance of two center-format boxes."""
    return math.hypot(box_a[0] - box_b[0], box_a[1] - box_b[1])


def det_box(d: dict) -> Tuple[float, float, float, float]:
    return (d["cx"], d["cy"], d["w"], d["h"])


if __name__ == "__main__":  # tiny self-check (no sim, no weights needed)
    b1 = (100.0, 100.0, 40.0, 40.0)
    b2 = (100.0, 100.0, 40.0, 40.0)
    assert abs(iou(b1, b2) - 1.0) < 1e-9, iou(b1, b2)
    b3 = (100.0, 100.0, 20.0, 20.0)  # fully inside b1 (area 400 vs 1600)
    assert abs(iou(b1, b3) - 0.25) < 1e-9, iou(b1, b3)
    b4 = (1000.0, 100.0, 40.0, 40.0)
    assert iou(b1, b4) == 0.0
    assert abs(center_err(b1, b4) - 900.0) < 1e-9
    # self-mask: a box far to the right (bearing > 30 deg) is dropped
    far = [{"cx": 1200.0, "cy": 480.0, "w": 60.0, "h": 60.0, "score": 0.9}]
    assert apply_self_mask(far) == []
    near = [{"cx": 640.0, "cy": 480.0, "w": 60.0, "h": 60.0, "score": 0.9}]
    assert len(apply_self_mask(near)) == 1
    print("[v3_onnx_infer] self-check PASS")
