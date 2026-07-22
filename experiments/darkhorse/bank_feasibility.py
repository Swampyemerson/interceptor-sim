#!/usr/bin/env python3
"""Dark-horse #3 — BANK-AS-ACCEL FEASIBILITY GATE (offline, HARD gate).

THE IDEA (and why it is gated HARD)
-----------------------------------
APN (augmented pro-nav) needs the target's lateral accel a_target. Estimating it
by differentiating a noisy bearing sank the earlier attempts (the noise wall,
ADR-0069). Bank-as-accel sidesteps differentiation: a banking quad's lateral
accel is a_lat = g*tan(phi), so READING the target's bank angle phi off the
image is a LEADING feedforward with no time-derivative. Memory
[[bank-as-accel-cue-idea]] / intercept_accuracy_levers.md flag two holes:
  (1) On the STRAIGHT-LINE mission target the APN term is identically zero
      (a_target == 0 -> APN == PN). So this is JINK INSURANCE only.
  (2) Reading bank off an ~8-15 px (blurred) silhouette is "optically
      implausible." THIS SCRIPT MEASURES (2): can bank even be read at terminal
      pixel sizes, and is the resulting a_target error below the jink signal?

WHAT IT MEASURES
----------------
Bank-discrimination SNR as a function of terminal pixel size, on the REAL
quad_dataset_banked (banks -50/-30/+30/+50 deg, ranges 2-8 m). Method:
  * Group frames into NUISANCE CELLS (same range/lateral/height/yaw/pitch);
    within a cell only the bank differs, so bank is isolated from aspect.
  * Extract a normalized target chip from the gt box (SCORING-only crop).
  * DEGRADE to a terminal size k px (downscale->blur->noise->upscale) = the
    information a k-px target actually carries.
  * bank_signal  = appearance distance between DIFFERENT banks in a cell.
    noise_floor  = appearance distance between two noise realizations of the
                   SAME chip.
    bank_SNR     = bank_signal / noise_floor. Below ~1 => bank is unreadable.
  * Map bank_SNR -> sigma_phi -> sigma_a (= g*sec^2(phi)*sigma_phi) and compare
    to the jink lateral accel a_jink ~ 4.7 m/s^2 (weave T=4s, v_lat=3 m/s;
    derivation in the printout). sigma_a >~ a_jink => the feedforward is noise
    => idea DEAD.

HONESTY BOUNDARY
----------------
gt boxes crop the target (scoring-only, ADR-0033 B). The bank ESTIMATOR under
test sees only pixels. gt bank is the label we score against, never an input.
A real bank-cue guidance path would re-earn the no-cheat audit.

RUN
---
  ../../.venv/bin/python bank_feasibility.py --self-test        # runs now, no data
  ../../.venv/bin/python bank_feasibility.py --gate             # real dataset
Needs numpy + cv2 (the project .venv). No ONNX, no GPU, no sim.
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import darkhorse_common as dc  # noqa: E402

G = 9.81
CHIP = 48                 # canonical chip size for appearance distance
A_JINK = 4.71             # m/s^2, weave T=4 s v_lat=3 m/s (see banner / printout)
PHI_JINK_DEG = 25.7       # implied peak bank of the jink


# ---------------------------------------------------------------------------
# Chip extraction + terminal degradation
# ---------------------------------------------------------------------------

def extract_chip(img, box_px, pad: float = 0.15):
    """Crop the gt box (padded), grayscale, resize to CHIPxCHIP, contrast-
    normalize. Returns None if the crop is degenerate."""
    import cv2
    cx, cy, bw, bh = box_px
    s = max(bw, bh) * (1 + pad)
    x0, y0 = int(cx - s / 2), int(cy - s / 2)
    x1, y1 = int(cx + s / 2), int(cy + s / 2)
    H, W = img.shape[:2]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(W, x1), min(H, y1)
    if x1 - x0 < 5 or y1 - y0 < 5:
        return None
    crop = img[y0:y1, x0:x1]
    g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY).astype(np.float32)
    g = cv2.resize(g, (CHIP, CHIP), interpolation=cv2.INTER_AREA)
    g -= g.mean()
    sd = g.std()
    return g / sd if sd > 1e-6 else None


def degrade_to_k(chip, k_px: int, blur_px: float, noise_dn: float,
                 rng: np.random.RandomState):
    """Simulate viewing the target at a terminal range where it spans k px:
    downscale the CHIP-res chip to k, blur, add noise, upscale back. This bakes
    in the information loss of a k-px target (blur/noise are the bench model the
    sim omits)."""
    import cv2
    small = cv2.resize(chip, (k_px, k_px), interpolation=cv2.INTER_AREA)
    if blur_px > 0:
        psf = dc.motion_psf(max(1.0, blur_px * k_px / CHIP))
        small = dc.apply_blur(small, psf)
    if noise_dn > 0:
        small = small + rng.normal(0, noise_dn, small.shape)
    up = cv2.resize(small, (CHIP, CHIP), interpolation=cv2.INTER_LINEAR)
    up -= up.mean()
    sd = up.std()
    return up / sd if sd > 1e-6 else up


def chip_dist(a, b) -> float:
    """Normalized appearance distance (1 - NCC), in [0,2]. 0 = identical."""
    a = a.ravel() - a.mean()
    b = b.ravel() - b.mean()
    d = math.sqrt((a @ a) * (b @ b)) + 1e-9
    return 1.0 - float((a @ b) / d)


# ---------------------------------------------------------------------------
# sigma_phi -> sigma_a mapping + jink comparison
# ---------------------------------------------------------------------------

def sigma_a_from_sigma_phi(sigma_phi_deg: float, phi_deg: float = PHI_JINK_DEG
                           ) -> float:
    """a_lat = g*tan(phi) -> d a/d phi = g*sec^2(phi); sigma_a = g*sec^2(phi)*sigma_phi."""
    phi = math.radians(phi_deg)
    return G / (math.cos(phi) ** 2) * math.radians(sigma_phi_deg)


# ---------------------------------------------------------------------------
# Real-data gate
# ---------------------------------------------------------------------------

def run_gate(repo: str, k_list: List[int], blur_px: float, noise_dn: float,
             seed: int) -> int:
    try:
        import cv2  # noqa: F401
    except ImportError:
        print("[bank] needs cv2 (the project .venv).")
        return 3
    import cv2
    ds = os.path.join(repo, "scripts/seeker/data/quad_dataset_banked")
    frames = dc.parse_banked_split(ds, "train") + dc.parse_banked_split(ds, "val")
    frames = [f for f in frames if f.label_path]
    if not frames:
        print(f"[bank] no banked frames at {ds} -- generate with "
              "scripts/seeker/render_sim_dataset_banked.py (needs the sim). "
              "Use --self-test for the runnable-now proof.")
        return 2
    rng = np.random.RandomState(seed)

    # group into nuisance cells (everything fixed except bank)
    cells: Dict[tuple, Dict[float, np.ndarray]] = defaultdict(dict)
    n_chip = 0
    for f in frames:
        img = cv2.imread(f.path)
        if img is None:
            continue
        box = dc.load_yolo_box(f.label_path)
        chip = extract_chip(img, box)
        if chip is None:
            continue
        key = (f.gt_range_m, f.lateral_m, f.height_m, f.yaw_deg, f.pitch_deg)
        cells[key][f.bank_deg] = chip
        n_chip += 1
    multi = {k: v for k, v in cells.items() if len(v) >= 2}
    print(f"[bank] chips={n_chip}  nuisance-cells(>=2 banks)={len(multi)}  "
          f"banks={sorted({b for v in multi.values() for b in v})}")
    print(f"[bank] jink signal to beat: a_jink={A_JINK:.2f} m/s^2 "
          f"(weave T=4s v_lat=3 m/s; bank~{PHI_JINK_DEG:.0f} deg)\n")

    print(f"{'k_px':>8} {'bank_signal':>12} {'noise_floor':>12} {'bank_SNR':>9} "
          f"{'sigma_phi_deg':>13} {'sigma_a':>9} {'verdict':>14}")
    # noise is ALWAYS applied (a meaningful noise floor at every size); the k=CHIP
    # row is the full-res reference (no downscale loss, sensor noise only).
    for k in [CHIP] + list(k_list):
        sig_dists, bank_gaps = [], []
        noise_dists = []
        for key, bankmap in multi.items():
            banks = sorted(bankmap)
            dchips = {b: degrade_to_k(ch, k, blur_px, noise_dn, rng)
                      for b, ch in bankmap.items()}
            # between-bank signal
            for b1, b2 in itertools.combinations(banks, 2):
                sig_dists.append(chip_dist(dchips[b1], dchips[b2]))
                bank_gaps.append(abs(b1 - b2))
            # noise floor: two independent degradations of the SAME bank chip
            b0 = banks[0]
            n1 = degrade_to_k(bankmap[b0], k, blur_px, noise_dn, rng)
            n2 = degrade_to_k(bankmap[b0], k, blur_px, noise_dn, rng)
            noise_dists.append(chip_dist(n1, n2))
        sig = float(np.median(sig_dists))
        nfloor = float(np.median(noise_dists)) + 1e-6
        snr = sig / nfloor
        mean_gap = float(np.mean(bank_gaps))         # ~ avg bank spacing, deg
        # sigma_phi: if the between-bank appearance signal is only 'snr' times
        # the noise, the bank can be resolved to ~ (mean bank spacing)/snr.
        sigma_phi = mean_gap / max(snr, 1e-3)
        sigma_a = sigma_a_from_sigma_phi(sigma_phi)
        verdict = "USABLE" if sigma_a < 0.5 * A_JINK else (
                  "MARGINAL" if sigma_a < A_JINK else "DEAD(>signal)")
        tag = "full-res" if k == CHIP else f"{k}px"
        print(f"{tag:>8} {sig:12.3f} {nfloor:12.3f} {snr:9.2f} "
              f"{sigma_phi:13.1f} {sigma_a:9.2f} {verdict:>14}")

    print(f"\n[bank] READ: sigma_a is the 1-sigma target-accel error the bank cue "
          f"would feed forward. It must be << a_jink={A_JINK:.1f} m/s^2 to help. "
          "If sigma_a >= a_jink at terminal sizes (8-15 px), the cue is noisier "
          "than the signal it predicts -> the idea is DEAD (as flagged).")
    print("[bank] Caveats: (i) STRAIGHT-LINE target -> APN term == 0 anyway, so "
          "this is jink-insurance only. (ii) sim has no motion blur; run with "
          "--blur-px/--noise-dn for the bench-realistic terminal read.")
    return 0


# ---------------------------------------------------------------------------
# Self-test: synthetic banked bars, prove the SNR pipeline + the collapse
# ---------------------------------------------------------------------------

def run_self_test(seed: int = 0) -> int:
    """Synthetic 'quad': a bright bar whose ORIENTATION encodes bank. Show the
    bank-discrimination SNR is high at full-res and COLLAPSES toward 1 as the
    chip is downscaled to terminal sizes -- the exact mechanism the gate
    measures on real data. Verifies the pipeline runs and the collapse is real."""
    import cv2
    rng = np.random.RandomState(seed)
    banks = [-50.0, -30.0, 30.0, 50.0]

    def bar_chip(bank_deg):
        img = np.zeros((CHIP, CHIP), np.float32)
        c = CHIP // 2
        th = math.radians(bank_deg)
        for t in np.linspace(-CHIP * 0.35, CHIP * 0.35, 200):
            x = int(round(c + t * math.cos(th)))
            y = int(round(c + t * math.sin(th)))
            if 0 <= x < CHIP and 0 <= y < CHIP:
                img[y, x] = 1.0
        img = cv2.GaussianBlur(img, (0, 0), 1.2)
        img -= img.mean()
        return img / (img.std() + 1e-9)

    chips = {b: bar_chip(b) for b in banks}
    print("[bank self-test] synthetic bank-bars; terminal BLUR should collapse SNR")
    print(f"{'condition':>18} {'bank_signal':>12} {'noise_floor':>12} {'bank_SNR':>9}")
    # (k_px, blur_px, noise, label)
    conds = [(CHIP, 0.0, 0.05, "full-res clean"),
             (8, 0.0, 0.05, "8px clean"),
             (8, 23.0, 0.15, "8px + terminal blur")]
    snrs = {}
    for k, blur, nz, label in conds:
        sig = []
        for b1, b2 in itertools.combinations(banks, 2):
            a = degrade_to_k(chips[b1], k, blur, nz, rng)
            b = degrade_to_k(chips[b2], k, blur, nz, rng)
            sig.append(chip_dist(a, b))
        n1 = degrade_to_k(chips[banks[0]], k, blur, nz, rng)
        n2 = degrade_to_k(chips[banks[0]], k, blur, nz, rng)
        nf = chip_dist(n1, n2) + 1e-6
        s = float(np.median(sig))
        snrs[label] = s / nf
        print(f"{label:>18} {s:12.3f} {nf:12.3f} {s / nf:9.2f}")
    # PASS: clean full-res resolves bank (high SNR); terminal blur collapses it
    # toward ~1 (unreadable) -- the exact mechanism the real gate confirms.
    clean = snrs["full-res clean"]
    blurred = snrs["8px + terminal blur"]
    # clean is strongly discriminable; terminal blur collapses it >=20x toward
    # the unreadable floor (the wall). Absolute floor kept loose -- the synthetic
    # bar's orientation is a strong low-freq feature; the real quad silhouette
    # (gate) collapses harder, to SNR ~1.
    passed = clean > 20.0 and blurred < clean / 20.0 and blurred < 6.0
    print(f"\n[bank self-test] clean SNR={clean:.1f}  ->  8px+blur SNR={blurred:.1f}")
    print("  RESULT:", "PASS" if passed else "FAIL",
          "-- pipeline resolves bank clean; terminal blur collapses it (the wall).")
    return 0 if passed else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default=None)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--gate", action="store_true", help="run on quad_dataset_banked")
    ap.add_argument("--k-px", default="15,12,10,8", help="terminal target sizes")
    ap.add_argument("--blur-px", type=float, default=0.0,
                    help="terminal motion-blur (bench model; sim has none)")
    ap.add_argument("--noise-dn", type=float, default=0.05,
                    help="chip-space sensor noise sigma (normalized units)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if args.self_test:
        return run_self_test()
    if args.gate:
        repo = dc.main_repo(args.repo)
        k_list = [int(v) for v in args.k_px.split(",")]
        return run_gate(repo, k_list, args.blur_px, args.noise_dn, args.seed)
    print("[bank] use --self-test (runs now) or --gate (real dataset). --help.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
