#!/usr/bin/env python3
"""Cut a drone out of photos into RGBA cut-outs for scripts/nn/cutpaste_synth.py.

Two sources:
  --photos DIR      photos or video frames of OUR drone held or hung against open sky or a plain
                    wall, drone roughly centred and filling 20-80% of the frame. No labels needed.
  --yolo ROOT SPLIT labelled images (ROOT/images/SPLIT, ROOT/labels/SPLIT); uses boxes of
                    --min-px or more as stand-in cut-outs until our own photos exist.
GrabCut (OpenCV) seeded from the box; a cut-out is KEPT only if the mask is one blob covering
8-75% of the box and not touching the crop edge. Rejections are counted and printed.
"""
import argparse, glob, os, sys
import cv2, numpy as np


def cut(img, box):
    h, w = img.shape[:2]; x0, y0, x1, y1 = box; bw, bh = x1 - x0, y1 - y0
    m = 0.25; cx0, cy0 = int(max(x0 - m * bw, 0)), int(max(y0 - m * bh, 0)); cx1, cy1 = int(min(x1 + m * bw, w)), int(min(y1 + m * bh, h))
    crop = img[cy0:cy1, cx0:cx1]
    if min(crop.shape[:2]) < 40: return None, "too small"
    # plain-background gate: the ring of the crop outside the box must be near-uniform (sky, wall).
    # Cluttered backgrounds gave cut-outs with chunks of scenery attached (seen 2026-11-13).
    ring = np.ones(crop.shape[:2], bool); ring[int(y0 - cy0):int(y1 - cy0), int(x0 - cx0):int(x1 - cx0)] = False
    if ring.sum() > 50 and float(crop[ring].std(axis=0).mean()) > 22: return None, "busy background"
    s = 320 / max(crop.shape[:2]); small = cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s < 1 else crop; s = min(s, 1)
    rect = (int((x0 - cx0) * s), int((y0 - cy0) * s), max(int(bw * s), 2), max(int(bh * s), 2))
    mask = np.zeros(small.shape[:2], np.uint8); bgd, fgd = np.zeros((1, 65)), np.zeros((1, 65))
    try: cv2.grabCut(small, mask, rect, bgd, fgd, 4, cv2.GC_INIT_WITH_RECT)
    except cv2.error: return None, "grabcut error"
    fg = ((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD)).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(fg)
    if n < 2: return None, "empty mask"
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA])); fg = (lab == k).astype(np.uint8)
    frac = fg.sum() / max(rect[2] * rect[3], 1)
    if not 0.08 <= frac <= 0.75: return None, "mask fraction"
    if fg[0].any() or fg[-1].any() or fg[:, 0].any() or fg[:, -1].any(): return None, "touches edge"
    alpha = cv2.GaussianBlur(cv2.resize(fg * 255, (crop.shape[1], crop.shape[0]), interpolation=cv2.INTER_LINEAR), (0, 0), max(crop.shape[0] / 200, 0.6))
    ys, xs = np.nonzero(alpha > 20); rgba = np.dstack([crop, alpha])[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    return rgba, "ok"


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--photos"); ap.add_argument("--yolo", nargs=2); ap.add_argument("--out", required=True)
    ap.add_argument("--min-px", type=int, default=120); ap.add_argument("--max-n", type=int, default=400); a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True); why = {}; n = 0; jobs = []
    if a.photos:
        for p in sorted(glob.glob(os.path.join(a.photos, "*"))):
            im = cv2.imread(p)
            if im is None: continue
            h, w = im.shape[:2]; jobs.append((p, im, (int(w * 0.08), int(h * 0.08), int(w * 0.92), int(h * 0.92))))
    else:
        root, split = a.yolo
        for lp in sorted(glob.glob(f"{root}/labels/{split}/*.txt")):
            rows = [l.split() for l in open(lp) if len(l.split()) >= 5]
            if len(rows) != 1: continue
            stem = lp.replace("/labels/", "/images/").rsplit(".", 1)[0]; ip = next((stem + e for e in (".jpg", ".png", ".jpeg") if os.path.exists(stem + e)), None)
            if not ip: continue
            im = cv2.imread(ip); h, w = im.shape[:2]; cx, cy, bw, bh = (float(v) for v in rows[0][1:5])
            if max(bw * w, bh * h) < a.min_px: continue
            jobs.append((ip, im, (int((cx - bw / 2) * w), int((cy - bh / 2) * h), int((cx + bw / 2) * w), int((cy + bh / 2) * h))))
            if len(jobs) >= 6 * a.max_n: break
    for p, im, box in jobs:
        rgba, r = cut(im, box); why[r] = why.get(r, 0) + 1
        if rgba is None: continue
        cv2.imwrite(os.path.join(a.out, f"cut_{n:04d}_{os.path.splitext(os.path.basename(p))[0][:40]}.png"), rgba); n += 1
        if n >= a.max_n: break
    print(f"cut-outs kept {n} of {len(jobs)} candidates; outcomes {why}")
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
