#!/usr/bin/env python3
"""Build the Hailo INT8 calibration set (docs/hailo_toolchain_runbook.md:
>=1024 images, input-size, FROM THE DEPLOYMENT CAMERA; small-target frames
weighted in, because INT8 recall loss concentrates exactly there).

Sources: one or more recorder dirs (frames/*.jpg, deploy layout) and/or
loose image dirs (sim renders). Frames are letterboxed to --size square,
grayscale -> replicated 3-channel (the compile path expects 3ch unless the
1-channel export is proven), written as PNG into --out.

Usage:
  .venv/bin/python scripts/nn/build_calib_set.py --out data/calib640 \
      --size 640 --n 1200 --src runs/desk_20261004T205433Z/frames \
      --src <more dirs...>
Fails loud if fewer than --min (default 1024) images land.
"""
import argparse
import glob
import hashlib
import os
import random
import sys

import cv2
import numpy as np


def letterbox(img, size):
    h, w = img.shape[:2]
    s = size / max(h, w)
    nh, nw = int(round(h * s)), int(round(w * s))
    r = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    out = np.full((size, size), 114, np.uint8)
    y0, x0 = (size - nh) // 2, (size - nw) // 2
    out[y0:y0 + nh, x0:x0 + nw] = r
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--src", action="append", required=True,
                    help="frame dir (repeatable)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--size", type=int, default=640)
    ap.add_argument("--n", type=int, default=1200)
    ap.add_argument("--min", type=int, default=1024)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    paths = []
    for src in args.src:
        for ext in ("jpg", "jpeg", "png"):
            paths += glob.glob(os.path.join(src, f"*.{ext}"))
    if not paths:
        print(f"no images under {args.src}; failing loud")
        return 1
    rng = random.Random(args.seed)
    rng.shuffle(paths)

    n_ok = 0
    for p in paths:
        if n_ok >= args.n:
            break
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        sq = letterbox(img, args.size)
        rgb = cv2.merge([sq, sq, sq])
        name = hashlib.sha1(p.encode()).hexdigest()[:16] + ".png"
        cv2.imwrite(os.path.join(args.out, name), rgb)
        n_ok += 1

    print(f"wrote {n_ok} calibration images to {args.out} "
          f"(size {args.size}, gray->3ch, letterbox 114)")
    if n_ok < args.min:
        print(f"FAIL: {n_ok} < required {args.min} -- add more --src dirs "
              f"(the Pi recorder runs hold thousands)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
