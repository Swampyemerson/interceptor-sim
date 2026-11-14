#!/usr/bin/env python3
"""Float reference: run an ultralytics-export ONNX on the letterboxed eval set. Usage: eval_onnx.py <onnx> <eval_dir> <out_json>"""
import json, os, sys
import cv2, numpy as np, onnxruntime as ort
onnx, d, out = sys.argv[1:4]
meta = json.load(open(os.path.join(d, "gt.json"))); size = meta["size"]
sess = ort.InferenceSession(onnx, providers=["CPUExecutionProvider"]); iname = sess.get_inputs()[0].name
def nms(b, s, thr=0.7):
    order = s.argsort()[::-1]; keep = []
    while order.size:
        i = order[0]; keep.append(i)
        if order.size == 1: break
        r = order[1:]
        xx0 = np.maximum(b[i, 0], b[r, 0]); yy0 = np.maximum(b[i, 1], b[r, 1]); xx1 = np.minimum(b[i, 2], b[r, 2]); yy1 = np.minimum(b[i, 3], b[r, 3])
        inter = np.clip(xx1 - xx0, 0, None) * np.clip(yy1 - yy0, 0, None)
        iou = inter / ((b[i, 2] - b[i, 0]) * (b[i, 3] - b[i, 1]) + (b[r, 2] - b[r, 0]) * (b[r, 3] - b[r, 1]) - inter + 1e-9)
        order = r[iou <= thr]
    return keep
dets = {}
for k, name in enumerate(sorted(meta["gt"])):
    im = cv2.cvtColor(cv2.imread(os.path.join(d, name)), cv2.COLOR_BGR2RGB)
    x = (im.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
    y = sess.run(None, {iname: x})[0][0]                  # (4+nc, N): cx, cy, w, h, score
    sc = y[4:].max(0); m = sc >= 0.2; y = y[:, m]; sc = sc[m]
    b = np.stack([y[0] - y[2] / 2, y[1] - y[3] / 2, y[0] + y[2] / 2, y[1] + y[3] / 2], 1) if sc.size else np.zeros((0, 4))
    keep = nms(b, sc) if sc.size else []
    dets[name] = [[*map(float, b[i]), float(sc[i])] for i in keep]
    if k % 500 == 0: print(k, flush=True)
json.dump(dets, open(out, "w")); print("done", len(dets))
