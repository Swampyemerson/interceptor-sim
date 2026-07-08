#!/usr/bin/env python3
"""Full-frame fine-tuned single-class drone seeker (ADR-0033 item 2, Option B).

The two-stage seeker (classical proposal -> crop -> COCO NN verify) is limited by
COCO having no drone class -> sparse terminal-only detection (ADR-0038). Option B
fine-tunes a single-class YOLO nano on IN-DOMAIN Gazebo renders of the tag-less
body (render_sim_dataset.py, gt-projected labels, training-time only). Because the
fine-tune is reliable on the sim body, we run it DIRECTLY on the full frame (no
classical proposal) -- a simpler, denser detector than the two-stage.

Interface: `.detect(frame_bgr, t_mono) -> SeekerDetection` -- drop-in for
TwoStageSeeker inside markerless_loop.py (selected by the MARKERLESS_NN_WEIGHTS
env var). Same bearing/range geometry as nn_seeker.SeekerDetection.

HONESTY: reads only camera pixels + fixed intrinsics; never gt_*. The fine-tune's
labels came from gt OFFLINE at training time (analogous to a human labeler) -- the
live detector at inference is gt-free, so the no-cheat audit is unaffected.

DEPS: onnxruntime + opencv (in .venv-seeker / the combined flight venv). No torch,
no ultralytics at inference -- the ONNX runs standalone.
"""
from __future__ import annotations

import math
import os
from typing import Optional

import numpy as np
import cv2

from nn_seeker import SeekerDetection  # reuse the exact dataclass / interface

# Target body span for coarse known-size range (models/fpv_target_markerless is
# ~1.0 m tip-to-tip; nn_seeker uses the same 1.0 m, ~15-30% range sigma disclosed).
TARGET_SPAN_M = 1.0


def _letterbox(img, new_size):
    """Resize to a square new_size keeping aspect (gray pad). Returns
    (padded_img, scale, (pad_w, pad_h))."""
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


class FinetunedNNSeeker:
    def __init__(self, fx, fy, cx, cy, weights: str,
                 conf_thres: float = 0.25, imgsz: int = 640,
                 target_span_m: float = TARGET_SPAN_M,
                 max_bearing_deg: float = 30.0, edge_margin_px: float = 40.0):
        import onnxruntime as ort
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.conf = conf_thres
        self.imgsz = imgsz
        self.span = target_span_m
        # SELF-MASK (ADR-0038 forensic fix): a positives-only fine-tune false-locks
        # on the interceptor's OWN body-fixed prop arms at the FOV periphery
        # (observed at bearing -44/+33 deg, constant ~1.5 m range while gt=29 m --
        # the confident-wrong-bearing hazard, seeker_prototype_results.md s3.4). The
        # real target is compact and near boresight through a cue-guided approach, so
        # reject any detection beyond max_bearing_deg OR touching the L/R frame edge
        # (where the props intrude) -- the two-stage's interior/self-mask gate, which
        # this full-frame path had omitted.
        self.max_bearing_rad = max_bearing_deg * math.pi / 180.0
        self.edge_margin_px = edge_margin_px
        so = ort.SessionOptions()
        so.intra_op_num_threads = 2
        self.sess = ort.InferenceSession(weights, so,
                                         providers=["CPUExecutionProvider"])
        self.iname = self.sess.get_inputs()[0].name
        # infer the model's expected input size from the ONNX (fall back to imgsz)
        shp = self.sess.get_inputs()[0].shape
        try:
            self.imgsz = int(shp[-1]) if isinstance(shp[-1], int) else imgsz
        except Exception:
            pass
        print(f"[finetuned] loaded {os.path.basename(weights)} imgsz={self.imgsz} "
              f"conf={self.conf}")

    def _none(self, t_mono):
        return SeekerDetection(t_mono, None, None, None, None, None, 0)

    def detect(self, frame_bgr, t_mono: Optional[float] = None) -> SeekerDetection:
        H, W = frame_bgr.shape[:2]
        img, r, (pad_l, pad_t) = _letterbox(frame_bgr, self.imgsz)
        blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run(None, {self.iname: np.ascontiguousarray(blob)})[0]
        preds = out[0]                      # (5, N) or (N, 5)
        if preds.shape[0] < preds.shape[1]:  # (5, N) -> (N, 5)
            preds = preds.T
        # columns: cx, cy, w, h, class0_score  (single class)
        best_score, best_box = 0.0, None
        n_hits = 0
        for row in preds:
            cx_, cy_, w_, h_, score = float(row[0]), float(row[1]), \
                float(row[2]), float(row[3]), float(row[4])
            if score < self.conf:
                continue
            # undo letterbox back to original-frame pixels
            bw = w_ / r
            bh = h_ / r
            u = (cx_ - pad_l) / r
            v = (cy_ - pad_t) / r
            # SELF-MASK: reject own-airframe periphery locks (bearing gate + edge touch)
            bearing = math.atan2((u - self.cx) / self.fx, 1.0)
            if abs(bearing) > self.max_bearing_rad:
                continue
            if (u - bw / 2) < self.edge_margin_px or (u + bw / 2) > (W - self.edge_margin_px):
                continue
            n_hits += 1
            if score > best_score:
                best_score, best_box = score, (u, v, bw, bh)
        if best_box is None:
            return self._none(t_mono)
        u, v, bw, bh = best_box
        bearing_h = math.atan2((u - self.cx) / self.fx, 1.0)   # + = right
        bearing_v = math.atan2((v - self.cy) / self.fy, 1.0)   # + = down
        range_m = self.fx * self.span / max(bw, 1.0)
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        xyz = range_m * ray
        return SeekerDetection(
            t_mono=t_mono, range_m=range_m, bearing_rad=bearing_h,
            bearing_vert_rad=bearing_v, meas_xyz=xyz,
            decision_margin=best_score, n_detections=n_hits,
            box_xywh=(u - bw / 2, v - bh / 2, bw, bh), class_name="drone")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Full-frame fine-tuned seeker (offline).")
    ap.add_argument("--weights", required=True)
    ap.add_argument("--frame", required=True, help="a PNG to run once")
    ap.add_argument("--conf", type=float, default=0.25)
    a = ap.parse_args()
    s = FinetunedNNSeeker(539.936, 539.936, 640.0, 480.0, a.weights, conf_thres=a.conf)
    d = s.detect(cv2.imread(a.frame))
    if d.range_m:
        print(f"DETECT bearing={d.bearing_rad*180/math.pi:+.1f} deg "
              f"range~{d.range_m:.2f} m conf={d.decision_margin:.2f} box={d.box_xywh}")
    else:
        print("no detect")
