#!/usr/bin/env python3
"""Dark-horse #2 — IMU-GUIDED MOTION-DEBLUR, Stage-0 BENCH script.

WHAT IT PROBES
--------------
Near closest approach the target streaks: LOS rate 485-1870 deg/s, and a 5 ms
exposure smears the body across ~23 px (docs/seeker_acquisition_range_note.md
§3.5). The target is only a few pixels wide to begin with, so the smear can
destroy it. IDEA: the interceptor's IMU + commanded body rates already measure
the ego-motion that causes most of the smear, so build the motion-blur PSF from
those rates + the known exposure and DECONVOLVE the track ROI to sharpen it --
no new hardware, unlike the event camera (dark-horse #4).

THE HONEST WALL (state it up front)
-----------------------------------
Deconvolution INVERTS a known blur; it CANNOT recover signal below the noise
floor. Blur spreads a few-pixel target's photons over ~23 px, so per-pixel SNR
falls ~20x; inverse/Wiener filtering then AMPLIFIES noise at the high spatial
frequencies it tries to restore. Deblur helps when the target is blur-limited
but still well above noise (bright target, short-ish exposure); it does little
when the target is already photon-starved (the regime a global shutter + short
exposure, or the event camera, actually fixes). This script MEASURES where that
line is, so the lever is chosen with eyes open -- before any event-camera spend.

WHY A BENCH SCRIPT (not a Gazebo run)
-------------------------------------
The sim has NO motion-blur model (ADR-0023), so there is nothing in-sim to
deblur. This is a Stage-0 bench tool: (a) a runnable-NOW synthetic self-test
(blur a known chip with a known PSF + noise, deconvolve, measure recovery vs
noise), and (b) the same pipeline pointed at real bench captures once they
exist (a target chip imaged on the OV9281 global-shutter cam while panned at a
known rate). It also consumes IMU/commanded-rate telemetry to BUILD the PSF the
way the real system would (see build_psf_from_rates).

HONESTY BOUNDARY
----------------
The PSF is built from own-state IMU/commanded rates + the known exposure --
own-sensor data, honesty-clean. No gt target pose is used to deblur. A new
image-restoration stage in the seeker path would re-earn the no-cheat audit
(it only reshapes the camera measurement; it never reads gt_*).

RUN
---
  ../../.venv/bin/python imu_deblur_bench.py --self-test         # runs now
  ../../.venv/bin/python imu_deblur_bench.py --noise-sweep       # SNR breakpoint
  ../../.venv/bin/python imu_deblur_bench.py --psf-from-rates 900 --exposure-ms 5
Needs only numpy + cv2 (the project .venv). No GPU, no sim.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import darkhorse_common as dc  # noqa: E402


# ---------------------------------------------------------------------------
# PSF construction from IMU / commanded rates (how the real system would do it)
# ---------------------------------------------------------------------------

def build_psf_from_rates(los_rate_dps: float, exposure_ms: float,
                         fx: float = dc.FX) -> Tuple[np.ndarray, float]:
    """Linear motion-blur PSF from a transverse angular rate + exposure.

    smear_px = los_rate[rad/s] * t_exp[s] * fx  (acquisition-range note §3.5).
    In the real seeker the transverse image-plane rate is (gyro body rate) +
    (target LOS rate); near CPA the target LOS rate dominates and is the part
    the IMU does NOT see -- an honest limit noted in the module docstring. Here
    we take the net transverse rate as given (bench sweeps it)."""
    smear = dc.blur_px_from_rate(los_rate_dps, exposure_ms, fx)
    return dc.motion_psf(smear), smear


# ---------------------------------------------------------------------------
# Wiener deconvolution (classical, known PSF)
# ---------------------------------------------------------------------------

def wiener_deconv(blurred: np.ndarray, psf: np.ndarray, nsr: float) -> np.ndarray:
    """Frequency-domain Wiener filter. nsr = noise-to-signal power ratio (the
    regularizer): larger nsr = trust the data less = less ringing but less
    sharpening. This single knob encodes 'can't recover below noise'."""
    img = blurred.astype(np.float64)
    H, W = img.shape
    psf_pad = np.zeros((H, W))
    kh, kw = psf.shape
    psf_pad[:kh, :kw] = psf
    psf_pad = np.roll(psf_pad, (-(kh // 2), -(kw // 2)), axis=(0, 1))
    Hf = np.fft.fft2(psf_pad)
    G = np.fft.fft2(img)
    F = np.conj(Hf) / (np.abs(Hf) ** 2 + nsr) * G
    return np.real(np.fft.ifft2(F))


# ---------------------------------------------------------------------------
# Metric: how well is the sharp chip recovered?
# ---------------------------------------------------------------------------

def _window(im: np.ndarray, center: Tuple[int, int], win_r: int) -> np.ndarray:
    cy, cx = center
    y0, y1 = max(0, cy - win_r), min(im.shape[0], cy + win_r + 1)
    x0, x1 = max(0, cx - win_r), min(im.shape[1], cx + win_r + 1)
    return im[y0:y1, x0:x1]


def ncc_to_ref(im: np.ndarray, ref: np.ndarray, center: Tuple[int, int],
               win_r: int) -> float:
    """Normalized cross-correlation of an image's target window against the
    KNOWN sharp truth's window. Scale/offset-invariant, so a Wiener level shift
    or overall attenuation does NOT fool it -- it measures whether the target's
    SHAPE was restored toward sharp. 1.0 = identical shape, ~0 = noise."""
    a = _window(im, center, win_r).astype(np.float64).ravel()
    b = _window(ref, center, win_r).astype(np.float64).ravel()
    a = a - a.mean()
    b = b - b.mean()
    denom = np.sqrt((a @ a) * (b @ b)) + 1e-9
    return float((a @ b) / denom)


def localized(im: np.ndarray, center: Tuple[int, int], win_r: int) -> bool:
    """True iff the GLOBAL brightest pixel lands inside the target window.
    False = an amplified-noise spike beat the target: the deconvolution failure
    mode this bench exists to expose."""
    cy, cx = center
    gy, gx = np.unravel_index(int(np.argmax(im)), im.shape)
    return abs(gy - cy) <= win_r and abs(gx - cx) <= win_r


# ---------------------------------------------------------------------------
# Synthetic target chip
# ---------------------------------------------------------------------------

def make_chip(size: int = 48, span_px: float = 12.0, contrast: float = 120.0,
              bg: float = 60.0) -> np.ndarray:
    """A small quad-like chip: a bright compact body with two darker 'arm'
    bars, on a mid-gray background. Not photoreal; a controllable stand-in for
    the bench measurement (a real target chip replaces it)."""
    img = np.full((size, size), bg, np.float64)
    c = size // 2
    r = int(span_px / 2)
    yy, xx = np.mgrid[0:size, 0:size]
    body = np.exp(-((xx - c) ** 2 + (yy - c) ** 2) / (2 * (r * 0.55) ** 2))
    img += contrast * body
    img[c - 1:c + 1, c - r:c + r] -= 0.4 * contrast   # arm bar (horizontal)
    img[c - r:c + r, c - 1:c + 1] -= 0.4 * contrast   # arm bar (vertical)
    return np.clip(img, 0, 255)


# ---------------------------------------------------------------------------
# Self-test: blur a known chip, recover it, ASSERT the pipeline works
# ---------------------------------------------------------------------------

def run_self_test(seed: int = 0) -> int:
    rng = np.random.RandomState(seed)
    sharp = make_chip()
    psf, smear = build_psf_from_rates(los_rate_dps=485.0, exposure_ms=3.0)  # ~13.7px
    blurred_clean = dc.apply_blur(sharp, psf)
    noise_dn = 2.0                        # low noise -> blur-limited, deblur wins
    blurred = np.clip(blurred_clean + rng.normal(0, noise_dn, sharp.shape), 0, 255)
    # Wiener with a physically-motivated nsr ~ (noise_var / signal_var)
    nsr = (noise_dn ** 2) / (sharp.var() + 1e-9)
    restored = wiener_deconv(blurred, psf, nsr)

    c = (sharp.shape[0] // 2, sharp.shape[1] // 2)
    win = 16
    ncc_blur = ncc_to_ref(blurred, sharp, c, win)
    ncc_rest = ncc_to_ref(restored, sharp, c, win)
    print(f"[deblur self-test] smear={smear:.1f}px  noise={noise_dn}DN  nsr={nsr:.3g}")
    print(f"  shape-match to sharp truth (NCC)   blurred={ncc_blur:.3f}  "
          f"restored={ncc_rest:.3f}")
    print(f"  restored target localized          {localized(restored, c, win)}")
    # PASS: in the low-noise (blur-limited) regime, deconvolution moves the
    # target SHAPE closer to the sharp truth and keeps it correctly localized.
    passed = localized(restored, c, win) and ncc_rest > ncc_blur + 0.03
    print("  RESULT:", "PASS" if passed else "FAIL",
          "-- deblur restores a blur-limited (low-noise) chip's shape.")
    return 0 if passed else 1


# ---------------------------------------------------------------------------
# Noise sweep: find the SNR breakpoint where deblur stops helping
# ---------------------------------------------------------------------------

def run_noise_sweep(seed: int = 0, detect_floor: float = 0.5) -> int:
    rng = np.random.RandomState(seed)
    # small, faint target = realistic terminal 8-12 px body, so the noise wall
    # lands in an illustrative range (not just at extreme DN).
    sharp = make_chip(size=48, span_px=10.0, contrast=70.0, bg=60.0)
    psf, smear = build_psf_from_rates(los_rate_dps=485.0, exposure_ms=5.0)  # worst-case
    blurred_clean = dc.apply_blur(sharp, psf)
    c = (sharp.shape[0] // 2, sharp.shape[1] // 2)
    win = 16
    print(f"[deblur noise-sweep] worst-case smear={smear:.1f}px "
          "(485 deg/s, 5 ms), faint 10px target. Deconvolution CANNOT beat noise.")
    print("[deblur] two filters: WIENER (nsr matched to noise = graceful) vs "
          "NAIVE inverse (fixed tiny nsr = the 'deconv amplifies noise' failure).")
    print(f"[deblur] detectability floor: target shape-match (NCC to sharp) >= "
          f"{detect_floor} AND localized.")
    print(f"\n{'noise_DN':>9} {'blur_NCC':>9} {'wiener_NCC':>11} {'wnr_loc':>8} "
          f"{'naive_NCC':>10} {'naive_loc':>10} {'verdict(wiener)':>16}")
    wiener_breakpoint = naive_breakpoint = None
    for noise_dn in (1, 2, 4, 8, 16, 32, 48):
        blurred = np.clip(blurred_clean + rng.normal(0, noise_dn, sharp.shape),
                          0, 255)
        nsr = (noise_dn ** 2) / (sharp.var() + 1e-9)   # matched
        wiener = wiener_deconv(blurred, psf, nsr)
        naive = wiener_deconv(blurred, psf, 1e-4)      # under-regularized
        ncc_b = ncc_to_ref(blurred, sharp, c, win)
        ncc_w = ncc_to_ref(wiener, sharp, c, win)
        ncc_n = ncc_to_ref(naive, sharp, c, win)
        loc_w = localized(wiener, c, win)
        loc_n = localized(naive, c, win)
        det_w = loc_w and ncc_w >= detect_floor
        if not det_w and wiener_breakpoint is None:
            wiener_breakpoint = noise_dn
        if not (loc_n and ncc_n >= detect_floor) and naive_breakpoint is None:
            naive_breakpoint = noise_dn
        print(f"{noise_dn:9.0f} {ncc_b:9.3f} {ncc_w:11.3f} {str(loc_w):>8} "
              f"{ncc_n:10.3f} {str(loc_n):>10} "
              f"{'DETECTABLE' if det_w else 'lost':>16}")
    print(f"\n[deblur] WIENER (matched) holds the target until ~"
          f"{wiener_breakpoint if wiener_breakpoint else '>48'} DN; NAIVE inverse "
          f"loses it by ~{naive_breakpoint if naive_breakpoint else '>48'} DN "
          "(noise amplification). Neither invents SNR -- above the wall the event "
          "camera / short-exposure global shutter (dark-horse #4) is the only fix.")
    print("[deblur] Bench must measure the REAL sensor's noise DN at the short "
          "exposure a fast crosser demands, then read the verdict off this line.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--noise-sweep", action="store_true")
    ap.add_argument("--psf-from-rates", type=float, default=None,
                    help="LOS/transverse rate deg/s -> print the PSF smear length")
    ap.add_argument("--exposure-ms", type=float, default=5.0)
    args = ap.parse_args()

    if args.psf_from_rates is not None:
        _, smear = build_psf_from_rates(args.psf_from_rates, args.exposure_ms)
        print(f"[deblur] {args.psf_from_rates:.0f} deg/s x {args.exposure_ms} ms "
              f"x fx={dc.FX:.0f}  ->  smear = {smear:.1f} px")
        return 0
    if args.noise_sweep:
        return run_noise_sweep()
    # default: self-test
    return run_self_test()


if __name__ == "__main__":
    raise SystemExit(main())
