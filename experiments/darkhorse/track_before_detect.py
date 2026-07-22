#!/usr/bin/env python3
"""Dark-horse #1 — TRACK-BEFORE-DETECT (temporal SNR integration), OFFLINE.

WHAT IT PROBES
--------------
The resolution-rejection experiment (docs/intercept_accuracy_levers.md "excluded")
varied SPATIAL pixels, one frame at a time, and found the same recall at the
8-12 m wall band. It never tested the TEMPORAL axis. Track-before-detect (TBD)
integrates the detector's SUB-THRESHOLD response across N consecutive frames
along a motion hypothesis: a target that is invisible in any single frame can
clear threshold in the integrated stack because the target response adds
coherently (~N) while the noise floor averages down (~sqrt(N)) -> effective SNR
rises ~sqrt(N). This is a TEMPORAL acquisition-range lever.

    seeker_acquisition_range_note.md: acquisition range ~ fx*W/w_floor. TBD
    lowers the effective per-frame w_floor by integrating, so it buys range the
    single-frame detector can't -- WITHOUT new hardware.

WHAT DATA IT USES (all real, honesty-clean)
-------------------------------------------
The markerless NN heatmap over LOGGED FRAMES. Default substrate is the
set-pose recall sweep (scripts/seeker/data/setpose_sweep/approach): 551 frames
of the tag-less target teleported to a level-FOV grid at 4.9/8/12/16/20/25 m
(~100 per range) -- the target is GUARANTEED in frame, isolating the DETECTOR
from the pointing confound (the whole point). We re-run the deployed seeker
weights (drone_finetuned_quad_v2.onnx) to get the raw per-frame response map;
no pre-computed heatmaps exist in the repo, so we generate them here.

HONESTY BOUNDARY
----------------
- The NN sees ONLY the camera frame. The filename gt range and the gt box
  center are used ONLY to (a) bin frames by range for scoring and (b) mark WHICH
  cell holds the target when measuring target-vs-background SNR. gt is never an
  input to the detector.
- MARKERLESS-ONLY: TBD needs a graded objectness heatmap. The AprilTag decoder
  emits a binary pose or nothing -- there is no sub-threshold response to
  integrate -- so TBD does NOT help the AprilTag first-kill baseline. It is a
  markerless-seeker lever only.
- The set-pose frames hold the target STATIONARY at each range (motion
  hypothesis = zero shift). That cleanly demonstrates the sqrt(N) mechanism but
  is easier than a real streaking approach. The decisive real test needs a
  continuous FRAMED far->near approach sequence (a pointing-fixed flight, or a
  scripted teleport ramp) so the motion-hypothesis registration is exercised.
  See --frames to point at such a sequence when it exists.

RUN
---
  # synthetic self-test (no data, proves the algorithm + sqrt(N) law):
  ../../.venv/bin/python track_before_detect.py --self-test

  # real heatmap integration on the set-pose sweep (needs .venv-seeker + weights):
  /home/emerson/interceptor-sim/.venv-seeker/bin/python track_before_detect.py \
      --sweep --n-integrate 1,4,8,16

Needs the .venv-seeker interpreter (onnxruntime + the weights) for --sweep /
--frames; --self-test runs on the plain project .venv (numpy only).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import darkhorse_common as dc  # noqa: E402


# ---------------------------------------------------------------------------
# Raw NN response map
# ---------------------------------------------------------------------------

# Response grid resolution (cells across the 640-letterbox). 40x40 = the YOLO
# stride-16 head footprint, coarse enough to be robust, fine enough to localize.
GRID = 40


class HeatmapSeeker:
    """Runs the single-class YOLO weights and returns a RAW response heatmap
    (max objectness per grid cell), NOT a thresholded box. This is the
    sub-threshold signal TBD integrates."""

    def __init__(self, weights: str, imgsz: int = 640):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(weights, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.sz = imgsz

    def _letterbox(self, img):
        import cv2
        h, w = img.shape[:2]
        r = min(self.sz / h, self.sz / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        resized = cv2.resize(img, (nw, nh))
        canvas = np.full((self.sz, self.sz, 3), 114, np.uint8)
        dw, dh = (self.sz - nw) // 2, (self.sz - nh) // 2
        canvas[dh:dh + nh, dw:dw + nw] = resized
        return canvas, r, dw, dh

    def response_map(self, img_bgr) -> Tuple[np.ndarray, float, int, int]:
        """Return (GRID x GRID response map in [0,1], letterbox r, dw, dh).

        The map cell (gy,gx) holds the MAX predicted objectness of any anchor
        whose box-center falls in that cell, in letterbox pixel space."""
        import cv2
        lb, r, dw, dh = self._letterbox(img_bgr)
        blob = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[None]
        out = self.sess.run(None, {self.inp: blob})[0][0]  # (5, 8400)
        out = out.T                                        # (8400, 5)
        cxs, cys = out[:, 0], out[:, 1]                    # letterbox px
        conf = out[:, 4]                                   # single-class score
        gm = np.zeros((GRID, GRID), np.float32)
        gx = np.clip((cxs / self.sz * GRID).astype(int), 0, GRID - 1)
        gy = np.clip((cys / self.sz * GRID).astype(int), 0, GRID - 1)
        # scatter-max
        np.maximum.at(gm, (gy, gx), conf)
        return gm, r, dw, dh

    def img_to_grid(self, u_px: float, v_px: float, r: float, dw: int, dh: int
                    ) -> Tuple[int, int]:
        """Image-pixel point -> response-grid cell (via the same letterbox)."""
        lx = u_px * r + dw
        ly = v_px * r + dh
        gx = int(np.clip(lx / self.sz * GRID, 0, GRID - 1))
        gy = int(np.clip(ly / self.sz * GRID, 0, GRID - 1))
        return gy, gx


# ---------------------------------------------------------------------------
# Real-data path: integrate NN heatmaps over the set-pose sweep
# ---------------------------------------------------------------------------

def _tgt_and_bg(gm: np.ndarray, gy: int, gx: int, guard: int = 2
                ) -> Tuple[float, float]:
    """Return (peak response in the target guard window, peak response in the
    background = everything outside it). A per-frame detection needs the target
    peak to (a) clear a conf threshold and (b) beat the background peak."""
    y0, y1 = max(0, gy - guard), min(GRID, gy + guard + 1)
    x0, x1 = max(0, gx - guard), min(GRID, gx + guard + 1)
    tgt = float(gm[y0:y1, x0:x1].max())
    bg = gm.copy()
    bg[y0:y1, x0:x1] = 0.0
    return tgt, float(bg.max())


def run_sweep(repo: str, n_list: List[int], weights: str, limit_per_range: int,
              conf_thr: float, blur_px: float, noise_dn: float, seed: int) -> int:
    try:
        import cv2  # noqa: F401
    except ImportError:
        print("[tbd] cv2/onnxruntime needed; run with the .venv-seeker python.")
        return 3
    sweep_dir = os.path.join(repo, "scripts/seeker/data/setpose_sweep/approach")
    frames = dc.parse_flight_dir(sweep_dir)
    if not frames:
        print(f"[tbd] no set-pose frames at {sweep_dir} -- generate them with "
              "scripts/seeker/setpose_recall_sweep.sh (needs the sim). Using "
              "--self-test instead is the runnable-now path.")
        return 2
    import cv2
    seeker = HeatmapSeeker(weights)
    rng = np.random.RandomState(seed)
    psf = dc.motion_psf(blur_px) if blur_px > 0 else None

    def degrade(img):
        if psf is not None:
            img = dc.apply_blur(img, psf)          # common-mode across frames
        if noise_dn > 0:
            img = dc.add_noise(img, noise_dn, rng)  # INDEPENDENT per frame
        return np.clip(img, 0, 255)

    def detected(img_u8, gy, gx):
        gm, r, dw, dh = seeker.response_map(img_u8)
        t, b = _tgt_and_bg(gm, gy, gx)
        return (t >= conf_thr and t > b)

    ranges = sorted({f.gt_range_m for f in frames})
    print(f"[tbd] weights={os.path.basename(weights)}  conf_thr={conf_thr}")
    print(f"[tbd] DEGRADATION (bench model, sim has none): "
          f"blur={blur_px:.1f}px (common-mode)  noise={noise_dn:.0f}DN "
          "(independent/frame)")
    print("[tbd] TBD = register + AVERAGE the raw frames (temporal denoise, "
          "noise ~1/sqrt(N)), THEN detect on the clean frame. Integrating the "
          "detector's OUTPUT confidence does NOT work -- a trained net collapses "
          "sub-threshold signal internally, so TBD must act pre-detection.")
    print(f"\n{'range_m':>8} {'w_px':>6} {'1f_rate':>8} | " +
          " ".join(f"avgN={n:<3}" for n in n_list))

    for R in ranges:
        fr = [f for f in frames if f.gt_range_m == R][:limit_per_range]
        if len(fr) < max(n_list):
            continue
        raw, cells, per_frame_det = [], [], 0
        for f in fr:
            img = cv2.imread(f.path)
            if img is None:
                continue
            deg = degrade(img.astype(np.float32))
            raw.append(deg)
            gm, r, dw, dh = seeker.response_map(deg.astype(np.uint8))
            box = dc.load_yolo_box(f.label_path) if f.label_path else None
            u, v = (box[0], box[1]) if box else (dc.CX, dc.CY)
            gy, gx = seeker.img_to_grid(u, v, r, dw, dh)
            cells.append((gy, gx))
            t, b = _tgt_and_bg(gm, gy, gx)
            if t >= conf_thr and t > b:
                per_frame_det += 1
        if len(raw) < max(n_list):
            continue
        gy = int(np.median([c[0] for c in cells]))
        gx = int(np.median([c[1] for c in cells]))
        one_f_rate = per_frame_det / len(raw)
        w_px = dc.FX * 0.3 / R           # pinhole size (scoring-only context)
        row = [f"{R:8.1f} {w_px:6.1f} {one_f_rate:8.2f} |"]
        for n in n_list:
            # register (set-pose -> zero shift) + average the RAW frames, detect once
            stacked = np.mean(raw[:n], axis=0).astype(np.uint8)
            row.append(f"  {'Y' if detected(stacked, gy, gx) else '.':>5}")
        print(" ".join(row))

    print(f"\n[tbd] READ: a range where 1f_rate is low but avgN flips to 'Y' is "
          "range TBD bought. Frame-stacking cuts INDEPENDENT noise ~1/sqrt(N); "
          "it does NOT cut common-mode motion BLUR (that is the deblur / event-"
          "camera dark horses' job). So TBD is a NOISE-limited-acquisition lever.")
    print("[tbd] On CLEAN sim frames v2 saturates (detects single-frame to 25m) "
          "-> the sim omits the very noise/blur TBD and its cousins address; the "
          "bench provides the real degraded frames.")
    print("[tbd] NOTE: set-pose frames are stationary per range -> zero-shift "
          "registration. A real streaking approach needs sub-pixel motion-"
          "hypothesis registration (--frames on a continuous framed sequence).")
    return 0


# ---------------------------------------------------------------------------
# Synthetic self-test: proves the algorithm + the sqrt(N) SNR law, runs NOW
# ---------------------------------------------------------------------------

def _patch_snr(img: np.ndarray, cx: float, cy: float, sig: int = 1,
               guard: int = 4) -> float:
    """SNR at a KNOWN center: mean of a tight (2*sig+1)^2 signal patch minus the
    background mean, over the background std. With the motion hypothesis giving
    the exact center, this scores the COHERENT sum -- so for a summed stack it
    grows ~sqrt(N) (signal ~N, background std ~sqrt(N)). No max-over-window, so
    it does not chase noise spikes (the bug the peak metric had)."""
    icy, icx = int(round(cy)), int(round(cx))
    sig_patch = img[max(0, icy - sig):icy + sig + 1,
                    max(0, icx - sig):icx + sig + 1]
    m = np.ones_like(img, bool)
    m[max(0, icy - guard):icy + guard + 1, max(0, icx - guard):icx + guard + 1] = False
    bg = img[m]
    return (sig_patch.mean() - bg.mean()) / (bg.std() + 1e-9)


def run_self_test(seed: int = 0) -> int:
    """Faint moving Gaussian blob in heavy noise: sub-threshold per frame,
    detectable after integration along the true motion hypothesis. Verifies
    both correctness (integration finds it) and the sqrt(N) SNR scaling."""
    rng = np.random.RandomState(seed)
    H, W = 96, 128
    n_frames = 48
    amp = 2.0            # target peak amplitude
    noise = 1.5          # per-frame noise sigma -> per-frame center SNR ~1.3 (marginal)
    # Integer motion hypothesis -> exact (coherent) registration for the demo.
    # Sub-pixel jitter partially decoheres the sum (real registration must be
    # sub-pixel-accurate); that realism cost is exercised on real data, not here.
    vx, vy = 1, 0        # true target velocity, px/frame
    x0, y0 = 20.0, 48.0
    yy, xx = np.mgrid[0:H, 0:W]

    frames, truth = [], []
    for k in range(n_frames):
        cx, cy = x0 + vx * k, y0 + vy * k
        blob = amp * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * 1.2 ** 2))
        frames.append(blob + rng.normal(0, noise, (H, W)))
        truth.append((cx, cy))
    frames = np.stack(frames)

    def aligned_stack(n):
        acc = np.zeros_like(frames[0])
        for k in range(n):
            acc += np.roll(np.roll(frames[k], -int(round(vy * k)), 0),
                           -int(round(vx * k)), 1)
        return acc

    print("[tbd self-test] faint moving blob, per-frame SNR intentionally sub-threshold")
    print(f"{'N':>4} {'no-align SNR':>13} {'motion-aligned SNR':>19}")
    ns, aligns = [], []
    for n in (1, 2, 4, 8, 16, 32, 48):
        naive = frames[:n].sum(0)                    # ignore motion -> smears
        snr_naive = _patch_snr(naive, *truth[n - 1])
        snr_align = _patch_snr(aligned_stack(n), x0, y0)
        ns.append(n)
        aligns.append(snr_align)
        print(f"{n:>4} {snr_naive:13.2f} {snr_align:19.2f}")

    naive48 = _patch_snr(frames[:48].sum(0), *truth[47])
    align48 = aligns[-1]
    # Fit the log-log slope over the STABLE regime (N>=8); the N=1 anchor is
    # itself a noisy single draw. Ideal temporal-integration slope = 0.5.
    reg = [(n, s) for n, s in zip(ns, aligns) if n >= 8 and s > 0]
    lx = np.log([n for n, _ in reg])
    ly = np.log([s for _, s in reg])
    slope = float(np.polyfit(lx, ly, 1)[0])
    # PASS: (i) SNR ~ N^0.5 (slope in [0.35,0.65]); (ii) clears a 5-sigma detect
    # bar at N=48; (iii) crushes the motion-blind naive stack.
    passed = (0.35 <= slope <= 0.65 and align48 >= 5.0 and align48 > 3.0 * naive48)
    print("\n[tbd self-test] PASS: aligned SNR ~ N^0.5, clears 5-sigma, beats naive.")
    print(f"  log-log slope (N>=8) = {slope:.2f}  (ideal 0.50)")
    print(f"  aligned N=48 = {align48:.2f}  naive N=48 = {naive48:.2f}  "
          f"(ratio {align48 / max(naive48, 1e-6):.1f}x)")
    print("  RESULT:", "PASS" if passed else "FAIL")
    return 0 if passed else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default=None, help="main checkout (data/weights)")
    ap.add_argument("--self-test", action="store_true",
                    help="synthetic sqrt(N) proof, no data needed")
    ap.add_argument("--sweep", action="store_true",
                    help="integrate NN heatmaps over the set-pose recall sweep")
    ap.add_argument("--frames", default=None,
                    help="alternate flight-capture dir (images/+labels/)")
    ap.add_argument("--weights", default=None,
                    help="ONNX weights (default: deployed v2)")
    ap.add_argument("--n-integrate", default="1,4,8,16",
                    help="comma list of integration depths N")
    ap.add_argument("--limit-per-range", type=int, default=60)
    ap.add_argument("--conf-thr", type=float, default=0.25,
                    help="per-frame objectness detect threshold (deployed=0.25)")
    ap.add_argument("--blur-px", type=float, default=0.0,
                    help="motion-blur smear length (bench model; sim has none). "
                         "e.g. 23 = 5ms exposure at 485 deg/s terminal LOS rate")
    ap.add_argument("--noise-dn", type=float, default=0.0,
                    help="additive sensor-noise sigma in DN (bench model)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if args.self_test:
        return run_self_test()

    repo = dc.main_repo(args.repo)
    weights = args.weights or os.path.join(
        repo, "scripts/seeker/weights/drone_finetuned_quad_v2.onnx")
    if not os.path.isfile(weights):
        print(f"[tbd] weights not found: {weights} (gitignored; run in main repo)")
        return 2
    n_list = [int(v) for v in args.n_integrate.split(",")]

    if args.frames:
        frames = dc.parse_flight_dir(args.frames)
        print(f"[tbd] {len(frames)} frames in {args.frames}; use --sweep for the "
              "binned set-pose analysis. (Per-sequence integration TODO once a "
              "continuous framed approach exists.)")
        return 0
    if args.sweep:
        return run_sweep(repo, n_list, weights, args.limit_per_range,
                         args.conf_thr, args.blur_px, args.noise_dn, args.seed)

    print("[tbd] nothing to do. Use --self-test (runs now) or --sweep (needs "
          ".venv-seeker + weights). See --help.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
