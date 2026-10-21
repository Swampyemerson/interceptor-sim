#!/usr/bin/env python3
"""Offline, reproducible domain randomization of simulator-rendered images.

Each input image (+ its YOLO label file, same stem) yields --copies augmented
grayscale JPEGs (quality 92) with labels carried through an albumentations
Compose (bbox_params format="yolo"). Every (stem, copy, seed) triple maps to a
fixed RNG seed, so a rerun reproduces byte-identical outputs.

No silent failures: unreadable images, missing/malformed labels, dropped or
out-of-range boxes and write errors are counted per reason; by default ANY
reject makes the exit code nonzero (3), and zero outputs is exit 2.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import warnings
from pathlib import Path
from typing import Any, Sequence

import albumentations as A
import cv2
import numpy as np

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_NO_OUTPUT = 2
EXIT_REJECTS = 3

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
GRAY_WEIGHT_MEAN_RGB = np.array([0.3, 0.5, 0.2])  # NIR-leaning (R, G, B)
GRAY_WEIGHT_CONCENTRATION = 40.0  # Dirichlet alpha = mean * this; higher = tighter
JPEG_QUALITY = 92

warnings.filterwarnings("ignore", message="Got processor for bboxes")


class LinearSkyGradient(A.ImageOnlyTransform):
    """Additive linear ramp of +/-amplitude DN across the image at a random angle."""

    def __init__(self, amplitude_dn: float = 25.0, p: float = 0.5) -> None:
        super().__init__(p=p)
        self.amplitude_dn = float(amplitude_dn)

    def get_params(self) -> dict[str, Any]:
        return {
            "angle": self.py_random.uniform(0.0, 2.0 * np.pi),
            "amp": self.py_random.uniform(-self.amplitude_dn, self.amplitude_dn),
        }

    def apply(self, img: np.ndarray, angle: float = 0.0, amp: float = 0.0, **_: Any) -> np.ndarray:
        h, w = img.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        proj = (xx - (w - 1) / 2) * np.cos(angle) + (yy - (h - 1) / 2) * np.sin(angle)
        span = float(np.abs(proj).max()) or 1.0
        ramp = (amp * proj / span).astype(np.float32)  # in [-|amp|, +|amp|]
        if img.ndim == 3:
            ramp = ramp[..., None]
        out = img.astype(np.float32) + ramp
        if img.dtype == np.uint8:
            return np.clip(out, 0, 255).astype(np.uint8)
        return np.clip(out, 0.0, 1.0).astype(img.dtype)

    def get_transform_init_args_names(self) -> tuple[str, ...]:
        return ("amplitude_dn",)


def stable_seed(stem: str, copy: int, seed: int) -> int:
    h = hashlib.sha256(f"{stem}|{copy}|{seed}".encode()).digest()
    return int.from_bytes(h[:4], "little")


def to_gray(img: np.ndarray, rng: np.random.Generator) -> tuple[np.ndarray, list[float] | None]:
    """3-channel (BGR as loaded by cv2) -> gray with jittered Dirichlet weights; 1-ch passes through."""
    if img.ndim == 2:
        return img, None
    if img.ndim == 3 and img.shape[2] == 1:
        return img[..., 0], None
    if img.ndim == 3 and img.shape[2] in (3, 4):
        w_rgb = rng.dirichlet(GRAY_WEIGHT_MEAN_RGB * GRAY_WEIGHT_CONCENTRATION)
        b, g, r = (img[..., i].astype(np.float32) for i in range(3))
        gray = w_rgb[0] * r + w_rgb[1] * g + w_rgb[2] * b
        return np.clip(np.rint(gray), 0, 255).astype(np.uint8), [float(x) for x in w_rgb]
    raise ValueError(f"unsupported image shape {img.shape}")


def noise_std_range(gray: np.ndarray) -> tuple[float, float]:
    """Signal-dependent noise: sigma ~ sqrt(k * mean DN) (shot-noise-like), k in [0.25, 1.0].

    albumentations 2.x GaussNoise takes std as a FRACTION of 255 (var_limit is gone).
    """
    mean = max(float(gray.mean()), 1.0)
    lo, hi = np.sqrt(0.25 * mean) / 255.0, np.sqrt(1.0 * mean) / 255.0
    return (float(lo), float(max(hi, lo + 1e-6)))


def build_compose(gray: np.ndarray, seed: int) -> A.Compose:
    return A.Compose(
        [
            A.MotionBlur(blur_limit=(3, 9), p=0.35),
            A.RandomGamma(gamma_limit=(80, 140), p=0.5),
            A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.5),
            LinearSkyGradient(amplitude_dn=25.0, p=0.5),
            A.GaussNoise(std_range=noise_std_range(gray), p=0.5),
            A.Downscale(scale_range=(0.5, 0.9), p=0.3),
        ],
        bbox_params=A.BboxParams(format="yolo", label_fields=["class_labels"], clip=True),
        seed=seed,
    )


def read_yolo(path: Path) -> tuple[list[list[float]], list[int]]:
    boxes: list[list[float]] = []
    classes: list[int] = []
    for ln, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        parts = line.split()
        if len(parts) != 5:
            raise ValueError(f"{path.name}:{ln}: expected 5 fields, got {len(parts)}")
        c = int(parts[0])
        cx, cy, w, h = (float(v) for v in parts[1:])
        if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < w <= 1 and 0 < h <= 1):
            raise ValueError(f"{path.name}:{ln}: box out of range {parts[1:]}")
        classes.append(c)
        boxes.append([cx, cy, w, h])
    return boxes, classes


def format_yolo(boxes: Sequence[Sequence[float]], classes: Sequence[int]) -> str:
    out = []
    for c, (cx, cy, w, h) in zip(classes, boxes):
        vals = [cx, cy, w, h]
        if not all(0.0 <= v <= 1.0 + 1e-6 for v in vals):
            raise ValueError(f"augmented box out of [0,1]: {vals}")
        cx, cy, w, h = (min(max(v, 0.0), 1.0) for v in vals)
        out.append(f"{int(c)} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return "\n".join(out) + ("\n" if out else "")


def augment_one(img: np.ndarray, boxes: list[list[float]], classes: list[int], stem: str, copy: int, seed: int
                ) -> tuple[np.ndarray, list[list[float]], list[int], dict[str, Any]]:
    s = stable_seed(stem, copy, seed)
    rng = np.random.default_rng(s)
    gray, weights = to_gray(img, rng)
    res = build_compose(gray, s)(image=gray, bboxes=boxes, class_labels=classes)
    out = res["image"]
    if out.ndim == 3:
        out = out[..., 0]
    out_boxes = [list(map(float, b)) for b in res["bboxes"]]
    out_cls = [int(c) for c in res["class_labels"]]
    if len(out_boxes) != len(boxes):
        raise ValueError(f"augmentation dropped boxes ({len(boxes)} -> {len(out_boxes)})")
    return out, out_boxes, out_cls, {"seed": s, "gray_weights_rgb": weights}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--images", type=Path, required=True)
    p.add_argument("--labels", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--copies", type=int, default=3)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--allow-rejects", action="store_true",
                   help="exit 0 even if some inputs were rejected (still counted in the manifest)")
    return p


def run(args: argparse.Namespace) -> int:
    if args.copies < 1:
        print("ERROR: --copies must be >= 1", file=sys.stderr)
        return EXIT_USAGE
    if not args.images.is_dir() or not args.labels.is_dir():
        print("ERROR: --images and --labels must be directories", file=sys.stderr)
        return EXIT_USAGE
    imgs = sorted(p for p in args.images.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    (args.out / "images").mkdir(parents=True, exist_ok=True)
    (args.out / "labels").mkdir(parents=True, exist_ok=True)

    rejects: dict[str, int] = {}
    per_file: list[dict[str, Any]] = []
    n_out = 0

    def reject(reason: str, n: int = 1) -> None:
        rejects[reason] = rejects.get(reason, 0) + n

    for ip in imgs:
        stem = ip.stem
        lp = args.labels / f"{stem}.txt"
        if not lp.exists():
            reject("missing_label", args.copies)
            continue
        try:
            boxes, classes = read_yolo(lp)
        except ValueError as exc:
            reject("bad_label", args.copies)
            per_file.append({"stem": stem, "error": str(exc)})
            continue
        img = cv2.imread(str(ip), cv2.IMREAD_UNCHANGED)
        if img is None:
            reject("read_failed", args.copies)
            continue
        if img.dtype != np.uint8:
            reject("unsupported_dtype", args.copies)
            continue
        for k in range(args.copies):
            try:
                out, ob, oc, info = augment_one(img, boxes, classes, stem, k, args.seed)
                text = format_yolo(ob, oc)
            except Exception as exc:  # counted, never swallowed
                reject(f"augment_failed:{type(exc).__name__}")
                per_file.append({"stem": stem, "copy": k, "error": str(exc)})
                continue
            name = f"{stem}_aug{k}"
            if not cv2.imwrite(str(args.out / "images" / f"{name}.jpg"), out,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]):
                reject("write_failed")
                continue
            (args.out / "labels" / f"{name}.txt").write_text(text)
            n_out += 1
            per_file.append({"stem": stem, "copy": k, "out": name, **info})

    manifest = {
        "n_in": len(imgs),
        "n_out": n_out,
        "n_expected": len(imgs) * args.copies,
        "reject_counts_by_reason": dict(sorted(rejects.items())),
        "seed": args.seed,
        "params_echo": {
            "copies": args.copies,
            "images": str(args.images),
            "labels": str(args.labels),
            "jpeg_quality": JPEG_QUALITY,
            "albumentations_version": A.__version__,
            "pipeline": {
                "MotionBlur": {"blur_limit": [3, 9], "p": 0.35},
                "RandomGamma": {"gamma_limit": [80, 140], "p": 0.5},
                "RandomBrightnessContrast": {"brightness_limit": 0.3, "contrast_limit": 0.3, "p": 0.5},
                "LinearSkyGradient": {"amplitude_dn": 25.0, "p": 0.5},
                "GaussNoise": {"std": "sqrt(k*mean_DN), k in [0.25,1]", "p": 0.5},
                "Downscale": {"scale_range": [0.5, 0.9], "p": 0.3},
                "gray_weights": {"mean_rgb": GRAY_WEIGHT_MEAN_RGB.tolist(),
                                 "dirichlet_concentration": GRAY_WEIGHT_CONCENTRATION},
            },
        },
        "files": per_file,
    }
    (args.out / "aug_manifest.json").write_text(json.dumps(manifest, indent=1))
    print(json.dumps({k: manifest[k] for k in ("n_in", "n_out", "n_expected", "reject_counts_by_reason")}))
    if n_out == 0:
        print(f"ERROR: zero augmented images written from {len(imgs)} inputs -- output is EMPTY",
              file=sys.stderr)
        return EXIT_NO_OUTPUT
    if rejects and not args.allow_rejects:
        print(f"ERROR: {sum(rejects.values())} outputs rejected: {manifest['reject_counts_by_reason']}",
              file=sys.stderr)
        return EXIT_REJECTS
    return EXIT_OK


def main(argv: Sequence[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
