"""Shared helpers for the dark-horse experiments (experiments/darkhorse/).

DATA LOCATION NOTE (important when running from a worktree): the datasets and
weights these experiments consume are gitignored, so they exist ONLY in the main
checkout at /home/emerson/interceptor-sim (scripts/seeker/data/*, scripts/seeker/
weights/*). Every script here resolves data paths against MAIN_REPO (override
with env DARKHORSE_REPO or the --repo flag), never against its own file location.

HONESTY BOUNDARY (CLAUDE.md): ground truth (filename-encoded gt range/bank, gt
label boxes) is used for SCORING and degradation-modelling ONLY — never as an
input to the estimator under test. Each script states per-metric which side of
the boundary its inputs sit on.

Camera constants trace to configs/camera_intrinsics.json (recorded 2026-07-04 from the
sim camera_info topic): fx=539.936, 1280x960. Motion-blur magnitudes trace to
docs/seeker_acquisition_range_note.md §3.5 (blur_px = lambda_dot * t_exp * fx;
terminal LOS rate 485-1870 deg/s per ADR-0023 row 7).
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Repo / data resolution
# ---------------------------------------------------------------------------

DEFAULT_MAIN_REPO = "/home/emerson/interceptor-sim"


def main_repo(cli_override: Optional[str] = None) -> str:
    """Resolve the main checkout that holds the gitignored data/weights."""
    repo = cli_override or os.environ.get("DARKHORSE_REPO") or DEFAULT_MAIN_REPO
    if not os.path.isdir(os.path.join(repo, "scripts", "seeker")):
        raise SystemExit(
            f"[darkhorse] main repo not found at {repo!r} "
            "(need scripts/seeker/). Pass --repo or set DARKHORSE_REPO."
        )
    return repo


# Camera intrinsics (configs/camera_intrinsics.json, sim ground truth of the sensor).
FX = 539.9363327026367
FY = 539.9363708496094
CX = 640.0
CY = 480.0
IMG_W = 1280
IMG_H = 960


# ---------------------------------------------------------------------------
# Filename ground-truth parsers (SCORING-side only)
# ---------------------------------------------------------------------------

# capture_flight_frames.py stems: <tag>_<seq:06d>_r<range:05.1f>_<pos|neg>
_FLIGHT_RE = re.compile(r"^(?P<tag>.+)_(?P<seq>\d{6})_r(?P<rng>\d+\.\d)_(?P<pn>pos|neg)$")

# render_sim_dataset_banked.py stems:
#   m<idx>_r<range>_y<lat>_h<hgt>_a<yaw>_b<bank>_p<pitch>
_BANKED_RE = re.compile(
    r"^m(?P<idx>\d+)_r(?P<rng>\d+\.\d)_y(?P<lat>[+-]?\d+\.\d+)_h(?P<hgt>\d+\.\d+)"
    r"_a(?P<yaw>[+-]\d+)_b(?P<bank>[+-]\d+)_p(?P<pitch>[+-]\d+)$"
)


@dataclass
class FlightFrame:
    path: str
    stem: str
    seq: int
    gt_range_m: float
    positive: bool
    label_path: Optional[str]  # YOLO gt box file (pos frames only)


@dataclass
class BankedFrame:
    path: str
    stem: str
    gt_range_m: float
    lateral_m: float
    height_m: float
    yaw_deg: float
    bank_deg: float
    pitch_deg: float
    label_path: Optional[str]


def parse_flight_dir(dir_path: str) -> List[FlightFrame]:
    """Parse a capture_flight_frames.py output dir (images/ + labels/)."""
    img_dir = os.path.join(dir_path, "images")
    lbl_dir = os.path.join(dir_path, "labels")
    out: List[FlightFrame] = []
    if not os.path.isdir(img_dir):
        return out
    for name in sorted(os.listdir(img_dir)):
        stem, ext = os.path.splitext(name)
        if ext.lower() != ".png":
            continue
        m = _FLIGHT_RE.match(stem)
        if not m:
            continue
        lbl = os.path.join(lbl_dir, stem + ".txt")
        out.append(
            FlightFrame(
                path=os.path.join(img_dir, name),
                stem=stem,
                seq=int(m.group("seq")),
                gt_range_m=float(m.group("rng")),
                positive=(m.group("pn") == "pos"),
                label_path=lbl if os.path.isfile(lbl) else None,
            )
        )
    out.sort(key=lambda f: f.seq)
    return out


def parse_banked_split(dataset_dir: str, split: str) -> List[BankedFrame]:
    """Parse quad_dataset_banked images/<split> with filename gt."""
    img_dir = os.path.join(dataset_dir, "images", split)
    lbl_dir = os.path.join(dataset_dir, "labels", split)
    out: List[BankedFrame] = []
    if not os.path.isdir(img_dir):
        return out
    for name in sorted(os.listdir(img_dir)):
        stem, ext = os.path.splitext(name)
        if ext.lower() != ".png":
            continue
        m = _BANKED_RE.match(stem)
        if not m:
            continue
        lbl = os.path.join(lbl_dir, stem + ".txt")
        out.append(
            BankedFrame(
                path=os.path.join(img_dir, name),
                stem=stem,
                gt_range_m=float(m.group("rng")),
                lateral_m=float(m.group("lat")),
                height_m=float(m.group("hgt")),
                yaw_deg=float(m.group("yaw")),
                bank_deg=float(m.group("bank")),
                pitch_deg=float(m.group("pitch")),
                label_path=lbl if os.path.isfile(lbl) else None,
            )
        )
    return out


def load_yolo_box(label_path: str, img_w: int = IMG_W, img_h: int = IMG_H
                  ) -> Optional[Tuple[float, float, float, float]]:
    """First YOLO-format box -> (cx, cy, w, h) in image pixels."""
    try:
        with open(label_path) as fh:
            line = fh.readline().split()
    except OSError:
        return None
    if len(line) < 5:
        return None
    _, xc, yc, w, h = (float(v) for v in line[:5])
    return xc * img_w, yc * img_h, w * img_w, h * img_h


# ---------------------------------------------------------------------------
# Degradation models (blur + noise) — bench-measurable knobs
# ---------------------------------------------------------------------------

def motion_psf(length_px: float, angle_deg: float = 0.0) -> np.ndarray:
    """Linear motion-blur PSF: a normalized line of `length_px` at `angle_deg`.

    blur_px = lambda_dot[rad/s] * t_exp[s] * fx  (acquisition-range note §3.5).
    A straight line is the constant-rate model over one exposure; real terminal
    sweeps curve slightly — noted, not modelled (Stage-0).
    """
    L = max(1, int(round(length_px)))
    if L == 1:
        return np.ones((1, 1), np.float32)
    k = max(L if L % 2 == 1 else L + 1, 3)
    psf = np.zeros((k, k), np.float32)
    c = k // 2
    th = np.deg2rad(angle_deg)
    dx, dy = np.cos(th), np.sin(th)
    n_steps = 4 * k
    for i in range(n_steps + 1):
        t = (i / n_steps - 0.5) * (L - 1)
        x = int(round(c + t * dx))
        y = int(round(c + t * dy))
        if 0 <= x < k and 0 <= y < k:
            psf[y, x] = 1.0
    s = psf.sum()
    return psf / s if s > 0 else psf


def apply_blur(img: np.ndarray, psf: np.ndarray) -> np.ndarray:
    import cv2
    return cv2.filter2D(img.astype(np.float32), -1, psf,
                        borderType=cv2.BORDER_REPLICATE)


def add_noise(img: np.ndarray, sigma_dn: float, rng: np.random.RandomState
              ) -> np.ndarray:
    """Additive Gaussian read/shot-noise proxy in digital numbers (0-255)."""
    if sigma_dn <= 0:
        return img.astype(np.float32)
    return img.astype(np.float32) + rng.normal(0.0, sigma_dn, img.shape)


def blur_px_from_rate(rate_dps: float, exposure_ms: float, fx: float = FX) -> float:
    """Smear length in pixels for a LOS-transverse angular rate over exposure."""
    return np.deg2rad(rate_dps) * (exposure_ms * 1e-3) * fx
