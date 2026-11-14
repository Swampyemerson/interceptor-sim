#!/usr/bin/env python3
"""Cut-and-paste training images: drone cut-outs (scripts/nn/make_cutouts.py) pasted onto real
target-free backgrounds at chosen pixel sizes, with the label box known exactly.

Why: there is no labelled dataset of 5-inch FPV quads seen from outside (survey 2026-11-13),
and the target cannot be flown yet. Photos of the real drone on the ground can still supply its
appearance; real air-to-air frames (the AOT negatives) supply the backgrounds and the clutter.

  cutpaste_synth.py --cutouts DIR --bg DIR [--bg DIR ...] --out DIR --n 8000 --size 1280

Each image: 85% get 1 drone (15% of those 2), 15% get none. Size is log-uniform over
--min-px..--max-px (long side, in output pixels). The paste is brightness-matched to its spot,
blurred to suit its size, optionally motion-blurred, and half the images go grey (the AOT
backgrounds are grey already). Cut-outs and backgrounds are split 90/10 into train/val by file,
so no cut-out or background appears in both. SYNTHETIC: never quote recall on this val set as
a real-world number.
"""
import argparse, glob, json, os, random, sys, zlib
import cv2, numpy as np


def paste(bg, rgba, long_side, rng):
    h, w = rgba.shape[:2]; s = long_side / max(h, w)
    if rng.random() < 0.5: rgba = rgba[:, ::-1]
    ang = rng.uniform(-25, 25); big = cv2.copyMakeBorder(rgba, h // 2, h // 2, w // 2, w // 2, cv2.BORDER_CONSTANT, value=(0, 0, 0, 0))
    M = cv2.getRotationMatrix2D((big.shape[1] / 2, big.shape[0] / 2), ang, s)
    out = cv2.warpAffine(big, M, (big.shape[1], big.shape[0]), flags=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    ys, xs = np.nonzero(out[:, :, 3] > 24)
    if len(xs) < 4: return None
    out = out[ys.min():ys.max() + 1, xs.min():xs.max() + 1].astype(np.float32); ph, pw = out.shape[:2]
    H, W = bg.shape[:2]
    if ph >= H - 4 or pw >= W - 4: return None
    x0, y0 = rng.randint(2, W - pw - 2), rng.randint(2, H - ph - 2); spot = bg[y0:y0 + ph, x0:x0 + pw].astype(np.float32)
    rgb, a = out[:, :, :3], out[:, :, 3:4] / 255.0
    if float(np.abs(spot[:, :, 0] - spot[:, :, 2]).mean()) < 2.0:      # grey background: the paste must be grey too
        rgb = np.repeat(rgb @ np.array([[0.114], [0.587], [0.299]], np.float32), 3, axis=2)
    # pull the paste part of the way toward the local brightness, keep it darker or lighter than the spot at random
    m_p = float((rgb * a).sum() / max(a.sum() * 3, 1)); m_b = float(spot.mean()); gain = np.clip((0.5 * m_b + 0.5 * m_p) / max(m_p, 1), 0.5, 1.8) * rng.uniform(0.8, 1.15)
    rgb = np.clip(rgb * gain, 0, 255)
    if rng.random() < 0.4:                                   # motion blur, up to about a fifth of the target
        L = max(3, int(long_side * rng.uniform(0.05, 0.2))) | 1; k = np.zeros((L, L), np.float32); k[L // 2] = 1.0 / L
        k = cv2.warpAffine(k, cv2.getRotationMatrix2D((L / 2 - 0.5, L / 2 - 0.5), rng.uniform(0, 180), 1), (L, L)); k /= max(k.sum(), 1e-6)
        rgb, a = cv2.filter2D(rgb, -1, k), cv2.filter2D(a, -1, k)[..., None]
    sig = rng.uniform(0.3, 0.9)
    rgb, a = cv2.GaussianBlur(rgb, (0, 0), sig), cv2.GaussianBlur(a, (0, 0), sig)[..., None]
    bg[y0:y0 + ph, x0:x0 + pw] = np.clip(rgb * a + spot * (1 - a), 0, 255).astype(np.uint8)
    ys, xs = np.nonzero(a[:, :, 0] > 0.15)
    if len(xs) < 2: return None
    return x0 + xs.min(), y0 + ys.min(), x0 + xs.max() + 1, y0 + ys.max() + 1


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--cutouts", required=True); ap.add_argument("--bg", action="append", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--n", type=int, default=8000); ap.add_argument("--size", type=int, default=1280)
    ap.add_argument("--min-px", type=float, default=8); ap.add_argument("--max-px", type=float, default=260); ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(); rng = random.Random(a.seed); np.random.seed(a.seed)
    cuts = sorted(glob.glob(os.path.join(a.cutouts, "*.png"))); bgs = sorted(p for d in a.bg for p in glob.glob(os.path.join(d, "*")) if p.lower().endswith((".jpg", ".png", ".jpeg")))
    if len(cuts) < 10 or len(bgs) < 50: sys.exit(f"need at least 10 cut-outs and 50 backgrounds, have {len(cuts)} and {len(bgs)}")
    is_val = lambda p: zlib.crc32(os.path.basename(p).encode()) % 10 == 0
    pools = {"train": ([c for c in cuts if not is_val(c)], [b for b in bgs if not is_val(b)]), "val": ([c for c in cuts if is_val(c)], [b for b in bgs if is_val(b)])}
    stats = {"sizes": [], "train": 0, "val": 0, "empty": 0, "cutouts": {k: len(v[0]) for k, v in pools.items()}, "backgrounds": {k: len(v[1]) for k, v in pools.items()}}
    for sub in pools:
        for d in ("images", "labels"): os.makedirs(os.path.join(a.out, sub, d), exist_ok=True)
    for i in range(a.n):
        sub = "val" if i % 10 == 0 else "train"; cpool, bpool = pools[sub]
        if not cpool or not bpool: sys.exit(f"empty {sub} pool")
        bg = cv2.imread(rng.choice(bpool)); H, W = bg.shape[:2]; S = a.size
        if min(H, W) < S: bg = cv2.resize(bg, None, fx=S / min(H, W) + 1e-3, fy=S / min(H, W) + 1e-3, interpolation=cv2.INTER_LINEAR); H, W = bg.shape[:2]
        x, y = rng.randint(0, W - S), rng.randint(0, H - S); bg = bg[y:y + S, x:x + S].copy(); lines = []
        k = 0 if rng.random() < 0.15 else (2 if rng.random() < 0.15 else 1)
        for _ in range(k):
            L = float(np.exp(rng.uniform(np.log(a.min_px), np.log(a.max_px)))); box = paste(bg, cv2.imread(rng.choice(cpool), cv2.IMREAD_UNCHANGED), L, rng)
            if box is None: continue
            x0, y0, x1, y1 = box; lines.append(f"0 {(x0 + x1) / 2 / S:.6f} {(y0 + y1) / 2 / S:.6f} {(x1 - x0) / S:.6f} {(y1 - y0) / S:.6f}"); stats["sizes"].append(max(x1 - x0, y1 - y0))
        if rng.random() < 0.5: bg = cv2.cvtColor(cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
        cv2.imwrite(os.path.join(a.out, sub, "images", f"cp_{i:06d}.jpg"), bg, [cv2.IMWRITE_JPEG_QUALITY, rng.randint(80, 95)])
        open(os.path.join(a.out, sub, "labels", f"cp_{i:06d}.txt"), "w").write("\n".join(lines) + ("\n" if lines else "")); stats[sub] += 1; stats["empty"] += not lines
    sz = sorted(stats.pop("sizes")); stats["box_long_side_px"] = {"n": len(sz), "p10": round(float(sz[len(sz) // 10]), 1), "median": round(float(sz[len(sz) // 2]), 1), "p90": round(float(sz[9 * len(sz) // 10]), 1)}
    json.dump(stats, open(os.path.join(a.out, "stats.json"), "w"), indent=1); print("CUTPASTE_DONE", json.dumps(stats))


if __name__ == "__main__":
    main()
