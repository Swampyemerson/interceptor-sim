#!/usr/bin/env python3
"""Turn the fetched AOT slice (scripts/nn/fetch_aot_frames.py) into YOLO training crops.

AOT frames are 2448 x 2048 grayscale with targets a few pixels across; shrinking a whole
frame to the network input would erase them. So each frame becomes ONE native-resolution
1280 x 1280 crop: around the drone (random offset, drone fully inside) for positives, around
the bird / airplane / helicopter for negatives (written with an EMPTY label file, so the net
learns those are not drones). Grayscale is replicated to 3 channels. Train/val split is by
FLIGHT so no flight is in both. Writes data/aot/yolo_crops/{train,val}/{images,labels} and
data/aot/yolo_crops/stats.json. Reads nothing but the plan and the fetched frames.
"""
import collections
import json
import os
import random
import sys
import zlib

import cv2

OUT = "data/aot/yolo_crops"; SIDE = 1280
plan = json.load(open("data/aot/pull_list_v2.json")); rng = random.Random(0)
stats = collections.Counter(); sizes = []
for sub in ("train", "val"):
    for d in ("images", "labels"):
        os.makedirs(os.path.join(OUT, sub, d), exist_ok=True)
for n, p in enumerate(plan):
    src = os.path.join("data/aot/images", f"part{p['part']}", p["flight"], p["img"])
    im = cv2.imread(src, cv2.IMREAD_GRAYSCALE)
    if im is None:
        stats["unreadable"] += 1; continue
    h, w = im.shape
    focus = [o for o in p["objects"] if o["cls"] == "Drone"] or p["objects"]
    l, t, bw, bh = focus[0]["bb"]; cx, cy = l + bw / 2, t + bh / 2
    x0 = int(min(max(cx - SIDE / 2 + rng.uniform(-0.35, 0.35) * SIDE, 0), w - SIDE))
    y0 = int(min(max(cy - SIDE / 2 + rng.uniform(-0.35, 0.35) * SIDE, 0), h - SIDE))
    crop = cv2.cvtColor(im[y0:y0 + SIDE, x0:x0 + SIDE], cv2.COLOR_GRAY2BGR)
    sub = "val" if zlib.crc32(p["flight"].encode()) % 10 == 0 else "train"
    stem = f"aot_{p['role']}_{os.path.splitext(p['img'])[0]}"
    lines = []
    if p["role"] == "drone":
        for o in p["objects"]:
            if o["cls"] != "Drone": continue
            l, t, bw, bh = o["bb"]; a, b, c, d = max(l - x0, 0), max(t - y0, 0), min(l + bw - x0, SIDE), min(t + bh - y0, SIDE)
            if c - a < 2 or d - b < 2: continue
            lines.append(f"0 {(a + c) / 2 / SIDE:.6f} {(b + d) / 2 / SIDE:.6f} {(c - a) / SIDE:.6f} {(d - b) / SIDE:.6f}")
            sizes.append(max(c - a, d - b))
        if not lines:
            stats["drone_frame_without_box_in_crop"] += 1; continue
    cv2.imwrite(os.path.join(OUT, sub, "images", stem + ".jpg"), crop, [cv2.IMWRITE_JPEG_QUALITY, 92])
    open(os.path.join(OUT, sub, "labels", stem + ".txt"), "w").write("\n".join(lines) + ("\n" if lines else ""))
    stats[f"{sub}_{p['role']}"] += 1
    if n % 1000 == 0: print(n, dict(stats), flush=True)
sizes.sort()
q = lambda f: round(sizes[int(f * (len(sizes) - 1))], 1) if sizes else None
out = {"counts": dict(stats), "drone_box_long_side_px": {"n": len(sizes), "p10": q(0.1), "median": q(0.5), "p90": q(0.9), "max": q(1.0)}}
json.dump(out, open(os.path.join(OUT, "stats.json"), "w"), indent=1)
print("AOT_CONVERT_DONE", json.dumps(out), flush=True)
