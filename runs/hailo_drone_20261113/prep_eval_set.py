#!/usr/bin/env python3
"""Letterbox the colour TEST split to a square input and write ground truth in letterbox pixels.
Usage: prep_eval_set.py <size> <out_dir>"""
import glob, json, os, sys
import cv2, numpy as np
size, out = int(sys.argv[1]), sys.argv[2]
root = "scripts/seeker/data/nn_tier/color"; os.makedirs(out, exist_ok=True); gt = {}
for i, lp in enumerate(sorted(glob.glob(f"{root}/labels/test/*.txt"))):
    stem = lp.replace("/labels/", "/images/").rsplit(".", 1)[0]
    ip = next((stem + e for e in (".jpg", ".png", ".jpeg") if os.path.exists(stem + e)), None)
    if not ip: continue
    im = cv2.imread(ip); h, w = im.shape[:2]; s = size / max(h, w)
    r = cv2.resize(im, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, np.uint8)
    y0, x0 = (size - r.shape[0]) // 2, (size - r.shape[1]) // 2
    canvas[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
    name = f"t{i:05d}.jpg"; cv2.imwrite(os.path.join(out, name), canvas, [cv2.IMWRITE_JPEG_QUALITY, 95])
    boxes = []
    for l in open(lp):
        p = l.split()
        if len(p) < 5: continue
        cx, cy, bw, bh = (float(v) for v in p[1:5])
        boxes.append([x0 + (cx - bw / 2) * w * s, y0 + (cy - bh / 2) * h * s, x0 + (cx + bw / 2) * w * s, y0 + (cy + bh / 2) * h * s])
    gt[name] = boxes
json.dump({"size": size, "gt": gt}, open(os.path.join(out, "gt.json"), "w"))
print(f"wrote {len(gt)} images, {sum(len(b) for b in gt.values())} boxes, {sum(1 for b in gt.values() if not b)} target-free")
