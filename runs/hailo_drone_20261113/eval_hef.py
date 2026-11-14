#!/usr/bin/env python3
"""Chip run (on the Pi): one-class NMS HEF over the letterboxed eval set. Usage: eval_hef.py <hef> <eval_dir> <out_json>"""
import json, os, sys, time
import cv2
from picamera2.devices import Hailo
hef, d, out = sys.argv[1:4]
meta = json.load(open(os.path.join(d, "gt.json"))); size = meta["size"]; dets = {}; t0 = time.time()
with Hailo(hef) as hailo:
    h, w, _ = hailo.get_input_shape(); assert (h, w) == (size, size), (h, w, size)
    for k, name in enumerate(sorted(meta["gt"])):
        im = cv2.cvtColor(cv2.imread(os.path.join(d, name)), cv2.COLOR_BGR2RGB)
        res = hailo.run(im)
        if isinstance(res, list) and len(res) and hasattr(res[0], "shape") and getattr(res[0], "ndim", 2) == 1: res = [res]
        dets[name] = [[float(b[1]) * size, float(b[0]) * size, float(b[3]) * size, float(b[2]) * size, float(b[4])] for cls in res for b in cls]
        if k % 500 == 0: print(k, round(time.time() - t0, 1), flush=True)
json.dump(dets, open(out, "w")); print("done", len(dets), "in", round(time.time() - t0, 1), "s")
