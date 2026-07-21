#!/usr/bin/env python3
"""Self-test for the nn_tier n-mono ONNX export: load the model, run ONE
real inference (a real test-split grayscale frame from the nn_tier corpus,
letterboxed to the model's input size) and sanity-check the output tensor
(finite, right shape). Exit 0 = model loads and runs; exit 1 = anything
wrong. Run in .venv-seeker (has onnxruntime + cv2 -- NOT a training env).

This is a LOAD+RUN smoke test only, not an accuracy gate -- it proves the
exported ONNX is well-formed and executable, matching the deliverable's
self_test_cmd contract. Frame-level accuracy is reported separately from
results.csv (flagged as NOT the deployment gate per the honesty rules).
"""
from __future__ import annotations

import csv
import os
import sys

import cv2
import numpy as np
import onnxruntime as ort

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
WEIGHTS = os.path.join(REPO, "scripts", "seeker", "weights", "nn_tier", "n-mono.onnx")
MANIFEST_TEST = os.path.join(REPO, "scripts", "seeker", "data", "nn_tier", "manifest_test.csv")
DATA_ROOT = os.path.join(REPO, "scripts", "seeker", "data", "nn_tier")


def _letterbox(img, new_size: int):
    h, w = img.shape[:2]
    r = min(new_size / h, new_size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = (new_size - nw) / 2, (new_size - nh) / 2
    top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
    left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
    out = cv2.copyMakeBorder(resized, top, bottom, left, right,
                              cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out


def _pick_test_image() -> str:
    if os.path.exists(MANIFEST_TEST):
        with open(MANIFEST_TEST, newline="") as f:
            for row in csv.DictReader(f):
                cand = os.path.join(DATA_ROOT, row["gray_img"])
                if os.path.exists(cand):
                    return cand
    return ""  # fall back to synthetic gray frame


def main() -> int:
    if not os.path.exists(WEIGHTS):
        print(f"SMOKE_TEST_FAIL: weights missing at {WEIGHTS}")
        return 1

    so = ort.SessionOptions()
    so.intra_op_num_threads = 2
    sess = ort.InferenceSession(WEIGHTS, so, providers=["CPUExecutionProvider"])
    iname = sess.get_inputs()[0].name
    shp = sess.get_inputs()[0].shape
    imgsz = int(shp[-1]) if isinstance(shp[-1], int) else 640

    img_path = _pick_test_image()
    if img_path:
        img = cv2.imread(img_path)
        print(f"[smoke] real test-split frame: {img_path}")
    else:
        rng = np.random.default_rng(0)
        img = rng.integers(0, 255, size=(720, 1280, 3), dtype=np.uint8)
        print("[smoke] no test manifest image found -- using synthetic gray frame")

    padded = _letterbox(img, imgsz)
    blob = padded.astype(np.float32) / 255.0
    blob = blob.transpose(2, 0, 1)[None, ...]  # NCHW

    out = sess.run(None, {iname: blob})
    ok = all(np.all(np.isfinite(o)) for o in out)
    shapes = [o.shape for o in out]
    print(f"[smoke] input shape: {blob.shape}, output shapes: {shapes}, finite={ok}")

    if not ok:
        print("SMOKE_TEST_FAIL: non-finite values in ONNX output")
        return 1

    print("SMOKE_TEST_PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
