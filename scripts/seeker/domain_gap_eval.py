#!/usr/bin/env python3
"""Sim-to-real DOMAIN-GAP eval — estimates the recall penalty the clean sim
can't show because it never renders motion blur, outdoor backgrounds, real
sensor noise, or auto-exposure artifacts.

WHAT THIS IS
------------
Every other seeker eval in this directory (resolution_probe.py,
approach_recall.py, ...) scores the deployed detector on CLEAN Gazebo renders.
That is a CEILING: it tells us the best case, on a domain the network was
trained on. It says nothing about how much recall a real camera would lose to
motion blur, a cluttered outdoor background, bad lighting, or JPEG/sensor
noise — because the sim doesn't render any of those.

This script takes EXISTING captured sim frames (scripts/seeker/data/
quad_approach and/or setpose_sweep/{approach,receding}, whichever are
present) with their gt YOLO boxes, and applies PARAMETRIC AUGMENTATIONS at
several strengths to each positive frame:
  - motion blur       : linear kernel, several pixel lengths
  - outdoor background: composite the gt-boxed target cutout onto a
                         procedurally generated outdoor-like background
                         (sky gradient + horizon + ground clutter texture) —
                         no real background photos exist in this repo, so we
                         synthesize plausible clutter/texture instead
  - brightness/contrast/gamma: lighting-stress presets (dim/harsh/washed out)
  - JPEG + sensor noise: JPEG re-encode at low quality + Gaussian pixel noise

It then scores the DEPLOYED detector (drone_finetuned_quad_v2.onnx, via
FinetunedNNSeeker — the exact class the seeker uses in flight) on clean vs.
augmented frames, using the same box_hits_gt scoring resolution_probe.py
uses, and prints a recall table: augmentation x strength -> recall (+ delta
from the clean baseline).

HONESTY (CLAUDE.md no-cheat boundary): gt boxes are used ONLY to (a) locate
where to composite the cutout during augmentation and (b) score whether a
detection landed on the real target — never fed to the detector. The
detector sees only pixels + fixed intrinsics, exactly as in flight.

This is a BRACKETING ESTIMATE, not a measurement. Synthetic augmentations are
a proxy for real-world degradation; they are useful for ranking "how fragile
is this detector to X" and for putting an order-of-magnitude floor under the
clean-sim recall ceiling — they are NOT a substitute for real flight data
(that's ADR-0072's shapes R5/R6). Every printed number should be read as
"clean-sim recall is the ceiling; here is how much of that ceiling a
plausible real-world stressor could take away."

Usage:
    scripts/seeker/domain_gap_eval.py
    scripts/seeker/domain_gap_eval.py --conf 0.05 --max-frames 200
    scripts/seeker/domain_gap_eval.py --data quad_approach setpose_sweep/approach

Run under .venv-seeker (onnxruntime + cv2). Self-tests / dry-runs against
whatever frame sets are present on disk; skips gracefully (prints a note,
does not error) if none of the expected sets exist.
"""
import argparse
import glob
import os
import sys
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from finetuned_seeker import FinetunedNNSeeker  # noqa: E402
# Single source of truth for the hit gate (was a local copy). The off-axis
# sec^2 fix is inherited automatically; extent unification (gt_scale) is left at
# 1.0 here because domain_gap_eval mixes frames from several capture dirs (each
# with its own --extent-m) into one list, so a single global scale would be
# wrong -- this tool is a bracketing ESTIMATE, and the primary scoring-debt fix
# lands in resolution_probe.py where each dataset's capture extent is known.
from box_scoring import box_hits_gt  # noqa: E402,F401

FX = FY = 539.9363327026367   # sim gz_x500_mono_cam intrinsics @1280x960
CX, CY = 640.0, 480.0
W_FULL, H_FULL = 1280, 960
V2 = os.path.join(HERE, "weights", "drone_finetuned_quad_v2.onnx")

# Candidate frame sets, tried in this order; any missing dir is skipped.
DEFAULT_DATA_DIRS = [
    "quad_approach",
    "setpose_sweep/approach",
    "setpose_sweep/receding",
]

RNG_SEED = 20260717


# --------------------------------------------------------------------------
# gt / scoring helpers (same convention as resolution_probe.py)
# --------------------------------------------------------------------------

def load_gt_box(label_path):
    """YOLO 'class cx cy w h' (normalized) -> (cx_px, cy_px, w_px, h_px) or None."""
    try:
        parts = open(label_path).readline().split()
    except OSError:
        return None
    if len(parts) < 5:
        return None
    _, cx, cy, w, h = (float(v) for v in parts[:5])
    return cx * W_FULL, cy * H_FULL, w * W_FULL, h * H_FULL


def collect_frames(data_dir, max_frames=None):
    """Return [(image_path, gt_box), ...] for positive frames (label exists
    and non-empty) under data_dir/images + data_dir/labels."""
    img_dir = os.path.join(data_dir, "images")
    lbl_dir = os.path.join(data_dir, "labels")
    if not (os.path.isdir(img_dir) and os.path.isdir(lbl_dir)):
        return []
    out = []
    for ip in sorted(glob.glob(os.path.join(img_dir, "*.png"))):
        base = os.path.splitext(os.path.basename(ip))[0]
        gt = load_gt_box(os.path.join(lbl_dir, base + ".txt"))
        if gt is None:
            continue
        out.append((ip, gt))
        if max_frames and len(out) >= max_frames:
            break
    return out


# --------------------------------------------------------------------------
# augmentations — each takes (frame_bgr, gt_box, strength_level, rng) and
# returns an augmented frame_bgr (same shape). gt_box is used only to locate
# the target for the background-composite augmentation.
# --------------------------------------------------------------------------

def aug_clean(frame, gt, level, rng):
    return frame


def aug_motion_blur(frame, gt, level, rng):
    """Linear motion blur. `level` = kernel length in px (0 = no-op)."""
    k = int(level)
    if k <= 1:
        return frame
    kernel = np.zeros((k, k), dtype=np.float32)
    kernel[k // 2, :] = 1.0
    kernel /= k
    return cv2.filter2D(frame, -1, kernel, borderType=cv2.BORDER_REPLICATE)


def _procedural_background(h, w, rng, clutter=0.5):
    """Synthesize a plausible outdoor-ish background: a sky gradient over a
    textured ground, with `clutter` (0-1) controlling how much high-frequency
    noise/texture (trees/bushes/gravel stand-in) is layered in. No real
    background photography exists in this repo, so this is an explicit
    procedural stand-in — see module docstring's honesty note."""
    horizon = int(h * rng.uniform(0.35, 0.55))
    sky_top = np.array([185, 140, 90], dtype=np.float32)     # BGR, hazy blue
    sky_bot = np.array([225, 205, 180], dtype=np.float32)    # near-horizon haze
    ground_top = np.array([60, 95, 90], dtype=np.float32)    # BGR, dull green/brown
    ground_bot = np.array([40, 65, 65], dtype=np.float32)
    bg = np.zeros((h, w, 3), dtype=np.float32)
    if horizon > 0:
        t = np.linspace(0, 1, horizon, dtype=np.float32)[:, None, None]
        bg[:horizon] = sky_top[None, None, :] * (1 - t) + sky_bot[None, None, :] * t
    gh = h - horizon
    if gh > 0:
        t = np.linspace(0, 1, gh, dtype=np.float32)[:, None, None]
        bg[horizon:] = ground_top[None, None, :] * (1 - t) + ground_bot[None, None, :] * t
    if clutter > 0:
        # low-frequency blotches (tree lines / bushes) + high-freq speckle
        blotch = rng.normal(0, 1, (max(h // 16, 1), max(w // 16, 1))).astype(np.float32)
        blotch = cv2.resize(blotch, (w, h), interpolation=cv2.INTER_CUBIC)[..., None]
        speck = rng.normal(0, 1, (h, w, 1)).astype(np.float32)
        texture = (0.7 * blotch + 0.3 * speck)
        bg += texture * (25.0 * clutter)
    return np.clip(bg, 0, 255).astype(np.uint8)


def aug_background(frame, gt, level, rng):
    """Composite the gt-boxed target cutout onto a procedurally generated
    outdoor background. `level` = clutter strength 0-1 (0 = clean sim bg
    kept, i.e. no-op is not exactly this — level 0 still swaps in a plain
    low-clutter background so the arm is meaningful at its lowest strength)."""
    h, w = frame.shape[:2]
    gcx, gcy, gw, gh = gt
    x0 = int(max(0, gcx - gw / 2 - 4))
    y0 = int(max(0, gcy - gh / 2 - 4))
    x1 = int(min(w, gcx + gw / 2 + 4))
    y1 = int(min(h, gcy + gh / 2 + 4))
    if x1 <= x0 or y1 <= y0:
        return frame
    cutout = frame[y0:y1, x0:x1].copy()
    bg = _procedural_background(h, w, rng, clutter=level)
    # feather the cutout edges into the new background with an elliptical mask
    mask = np.zeros((y1 - y0, x1 - x0), dtype=np.float32)
    cv2.ellipse(mask, ((x1 - x0) // 2, (y1 - y0) // 2),
                (max((x1 - x0) // 2, 1), max((y1 - y0) // 2, 1)),
                0, 0, 360, 1.0, -1)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)[..., None]
    out = bg.astype(np.float32)
    out[y0:y1, x0:x1] = mask * cutout.astype(np.float32) + (1 - mask) * out[y0:y1, x0:x1]
    return np.clip(out, 0, 255).astype(np.uint8)


def aug_lighting(frame, gt, level, rng):
    """Brightness/contrast/gamma stress. `level` in {0..3}:
      0 = clean, 1 = mild (slightly dim/flat), 2 = moderate (harsh/washed),
      3 = severe (near-silhouette or blown-out)."""
    presets = {
        0: dict(gain=1.00, bias=0, gamma=1.00),
        1: dict(gain=0.85, bias=-10, gamma=1.15),
        2: dict(gain=1.35, bias=25, gamma=0.75),
        3: dict(gain=0.55, bias=-45, gamma=1.6),
    }
    p = presets.get(int(level), presets[0])
    f = frame.astype(np.float32) * p["gain"] + p["bias"]
    f = np.clip(f, 0, 255) / 255.0
    f = np.power(f, p["gamma"]) * 255.0
    return np.clip(f, 0, 255).astype(np.uint8)


def aug_noise(frame, gt, level, rng):
    """JPEG re-encode + Gaussian sensor noise. `level` in {0..3}:
      0 = clean (quality 95, no noise)
      1 = quality 60, sigma 6
      2 = quality 35, sigma 14
      3 = quality 15, sigma 25"""
    presets = {
        0: (95, 0.0),
        1: (60, 6.0),
        2: (35, 14.0),
        3: (15, 25.0),
    }
    q, sigma = presets.get(int(level), presets[0])
    f = frame.astype(np.float32)
    if sigma > 0:
        f = f + rng.normal(0, sigma, f.shape)
    f = np.clip(f, 0, 255).astype(np.uint8)
    ok, enc = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, int(q)])
    if not ok:
        return f
    return cv2.imdecode(enc, cv2.IMREAD_COLOR)


AUGMENTATIONS = {
    "clean":       (aug_clean,       [0]),
    "motion_blur": (aug_motion_blur, [8, 16, 24, 32]),
    "background":  (aug_background,  [0.15, 0.5, 1.0]),
    "lighting":    (aug_lighting,    [1, 2, 3]),
    "noise_jpeg":  (aug_noise,       [1, 2, 3]),
}


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", nargs="*", default=None,
                     help="data_dir(s) under scripts/seeker/data/ "
                          "(default: quad_approach, setpose_sweep/approach, "
                          "setpose_sweep/receding — whichever exist)")
    ap.add_argument("--conf", type=float, default=0.10)
    ap.add_argument("--max-frames", type=int, default=300,
                     help="cap frames scored per augmentation strength "
                          "(keeps the CPU-only ONNX eval fast)")
    args = ap.parse_args()

    data_root = os.path.join(HERE, "data")
    dirs = args.data if args.data else DEFAULT_DATA_DIRS
    frames = []
    used_dirs = []
    for d in dirs:
        full = d if os.path.isabs(d) else os.path.join(data_root, d)
        got = collect_frames(full)
        if got:
            used_dirs.append((d, len(got)))
            frames.extend(got)
        else:
            print(f"  [skip] {d}: not found or no positive frames")

    if not frames:
        print("=== domain_gap_eval: NO frame sets found (quad_approach / "
              "setpose_sweep absent) — nothing to score. Skipping cleanly. ===")
        return

    if args.max_frames and len(frames) > args.max_frames:
        step = len(frames) / args.max_frames
        frames = [frames[int(i * step)] for i in range(args.max_frames)]

    if not os.path.exists(V2):
        print(f"=== domain_gap_eval: detector weights not found at {V2} — "
              "skipping cleanly. ===")
        return

    v2 = FinetunedNNSeeker(FX, FY, CX, CY, V2, conf_thres=args.conf)

    print(f"=== domain-gap eval: {len(frames)} positive frames from "
          f"{', '.join(f'{d} ({n})' for d, n in used_dirs)}, conf={args.conf} ===")
    print("  ESTIMATE, not a measurement — synthetic augmentations bracket the "
          "recall penalty a clean sim can't show. clean-sim recall is a CEILING.\n")

    rng = np.random.default_rng(RNG_SEED)
    results = defaultdict(dict)  # {aug_name: {level: (hits, total)}}
    clean_recall = None

    for aug_name, (fn, levels) in AUGMENTATIONS.items():
        for level in levels:
            hits = 0
            for ip, gt in frames:
                frame = cv2.imread(ip)
                if frame is None:
                    continue
                aug_frame = fn(frame, gt, level, rng)
                gcx, gcy, gw, gh = gt
                boxes = v2._infer_boxes(aug_frame)
                if box_hits_gt(boxes, gcx, gcy, gw, gh):
                    hits += 1
            total = len(frames)
            results[aug_name][level] = (hits, total)
        if aug_name == "clean":
            h, t = results["clean"][0]
            clean_recall = h / t if t else 0.0

    # ---- print table ----
    col_w = 14
    print(f"  {'augmentation':<16}{'strength':<12}{'recall':>10}{'delta':>10}   n")
    print("  " + "-" * 62)
    for aug_name, (fn, levels) in AUGMENTATIONS.items():
        for level in levels:
            hits, total = results[aug_name][level]
            recall = hits / total if total else 0.0
            delta = recall - clean_recall if clean_recall is not None else 0.0
            label = "-" if aug_name == "clean" else str(level)
            print(f"  {aug_name:<16}{label:<12}{100*recall:>8.1f}%{100*delta:>+9.1f}pp   {hits}/{total}")
    print("  " + "-" * 62)
    print(f"\n  clean-sim recall (ceiling): {100*clean_recall:.1f}%" if clean_recall is not None else "")
    worst = {}
    for aug_name, (fn, levels) in AUGMENTATIONS.items():
        if aug_name == "clean":
            continue
        worst_level = max(levels)
        h, t = results[aug_name][worst_level]
        worst[aug_name] = h / t if t else 0.0
    if worst:
        floor_aug = min(worst, key=worst.get)
        print(f"  worst single-stressor recall (at max strength): "
              f"{floor_aug} -> {100*worst[floor_aug]:.1f}% "
              f"(delta {100*(worst[floor_aug]-clean_recall):+.1f}pp)")
    print("\n  READ: this is a BRACKETING ESTIMATE (synthetic augmentation is a "
          "proxy for real-world degradation), not a real-flight measurement. "
          "Use it to rank which stressor the detector is most fragile to and "
          "to put an order-of-magnitude floor under the clean-sim ceiling. "
          "Real data (ADR-0072 shapes R5/R6) is the only thing that replaces it.")


if __name__ == "__main__":
    main()
