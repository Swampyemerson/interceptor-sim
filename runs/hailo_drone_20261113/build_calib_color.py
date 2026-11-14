#!/usr/bin/env python3
"""Colour calibration set for the Hailo compile: letterboxed TRAIN images, half of them
frames whose target is under 16 px at the given input size. Usage: build_calib_color.py <size> <n> <out>"""
import glob, os, random, sys
import cv2, numpy as np
size, n, out = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
root = "scripts/seeker/data/nn_tier/color"
random.seed(0); os.makedirs(out, exist_ok=True)
small, other = [], []
for lp in sorted(glob.glob(f"{root}/labels/train/*.txt")):
    ip = next((lp.replace("/labels/", "/images/").rsplit(".", 1)[0] + e for e in (".jpg", ".png", ".jpeg")
               if os.path.exists(lp.replace("/labels/", "/images/").rsplit(".", 1)[0] + e)), None)
    if not ip: continue
    boxes = [l.split() for l in open(lp) if len(l.split()) >= 5]
    # assume the common 16:9 source for the size test; exact size is applied at letterbox time
    is_small = any(max(float(b[3]) * size, float(b[4]) * size * 9 / 16) < 16 * size / 640 for b in boxes)
    (small if is_small else other).append(ip)
random.shuffle(small); random.shuffle(other)
pick = small[: n // 2] + other[: n - min(len(small), n // 2)]
for i, ip in enumerate(pick):
    im = cv2.imread(ip); h, w = im.shape[:2]; s = size / max(h, w)
    r = cv2.resize(im, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, np.uint8)
    y0, x0 = (size - r.shape[0]) // 2, (size - r.shape[1]) // 2
    canvas[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
    cv2.imwrite(os.path.join(out, f"calib_{i:04d}.png"), canvas)
print(f"wrote {len(pick)} images ({min(len(small), n // 2)} small-target) to {out}")
