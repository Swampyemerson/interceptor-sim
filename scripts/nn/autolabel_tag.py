#!/usr/bin/env python3
"""Auto-label quadcopter frames from a bolted-on AprilTag (tag36h11).

Pipeline per sampled frame:
  detect tag -> IPPE_SQUARE PnP (full distortion) -> quality gates
  (side px, hamming, reprojection error, incidence) -> project the caliper-
  measured target cuboid -> YOLO box (class 0) -> MASK the tag out of the
  saved image (so the detector cannot learn "find the fiducial") -> manifest.

Frame conventions (read this before measuring the cuboid):
  * PnP / AprilTag-native tag frame ("apriltag_native"): origin at tag centre,
    +x toward the tag's right edge, +y toward the tag's BOTTOM edge, +z INTO
    the tag face (away from a camera that sees it). This is the frame implied
    by object points (-1,+1),(+1,+1),(+1,-1),(-1,-1)*s/2 in the AprilTag
    corner order (bottom-left, bottom-right, top-right, top-left in image).
  * Cuboid JSON frame ("out_of_face", the default): origin at tag centre,
    +x toward the tag's right edge, +y toward the tag's TOP edge, +z OUT of
    the tag face (toward the camera). Right-handed. Converted to the native
    frame by diag(1, -1, -1) (a 180 deg rotation about x).
  Pass --cuboid-frame apriltag_native if your corners are already native.

No silent failures: every sampled frame ends either labeled or with exactly
one counted reject reason; zero labeled frames => exit code 2.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol, Sequence

import cv2
import numpy as np

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_NO_LABELS = 2

TAG_UNIT_CORNERS = np.array(
    [[-1.0, 1.0, 0.0], [1.0, 1.0, 0.0], [1.0, -1.0, 0.0], [-1.0, -1.0, 0.0]]
)
OUT_OF_FACE_TO_NATIVE = np.diag([1.0, -1.0, -1.0])
MAX_REPROJ_ERR_PX = 2.0
BOX_SHRINK_FRAC = 0.05
MASK_DILATE_PX = 2
RING_PX = 6
MIN_BOX_PX = 2.0


# --------------------------------------------------------------------------
# Data containers
# --------------------------------------------------------------------------
class Detection(Protocol):
    corners: Any  # (4, 2) image px, AprilTag order (bl, br, tr, tl)


class Detector(Protocol):
    def detect(self, img: np.ndarray) -> Sequence[Any]: ...


@dataclass
class Intrinsics:
    K: np.ndarray
    dist: np.ndarray
    width: int
    height: int

    @classmethod
    def from_json(cls, path: Path) -> "Intrinsics":
        d = json.loads(Path(path).read_text())
        K = np.array(
            [[d["fx"], 0.0, d["cx"]], [0.0, d["fy"], d["cy"]], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        dist = np.asarray(d.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=np.float64)
        res = d["resolution"]
        return cls(K=K, dist=dist, width=int(res["width"]), height=int(res["height"]))


@dataclass
class Pose:
    rvec: np.ndarray
    tvec: np.ndarray
    reproj_err_px: float
    incidence_deg: float


@dataclass
class FrameResult:
    ok: bool
    reason: str | None = None
    record: dict[str, Any] = field(default_factory=dict)
    image_out: np.ndarray | None = None
    bbox_px: tuple[float, float, float, float] | None = None
    tag_quad_px: np.ndarray | None = None


# --------------------------------------------------------------------------
# Detector factory
# --------------------------------------------------------------------------
def make_detector(family: str = "tag36h11") -> Detector:
    """Return a pupil_apriltags (or pyapriltags) Detector; raise ImportError if neither."""
    try:
        from pupil_apriltags import Detector as _D  # type: ignore
    except ImportError:
        try:
            from pyapriltags import Detector as _D  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "neither pupil_apriltags nor pyapriltags is installed "
                "(pip install pupil-apriltags)"
            ) from exc
    return _D(families=family)


# --------------------------------------------------------------------------
# Geometry (pure, unit-tested)
# --------------------------------------------------------------------------
def tag_object_points(tag_size_m: float) -> np.ndarray:
    return (TAG_UNIT_CORNERS * (tag_size_m / 2.0)).astype(np.float64)


def cuboid_to_native(corners: np.ndarray, frame: str = "out_of_face") -> np.ndarray:
    corners = np.asarray(corners, dtype=np.float64).reshape(-1, 3)
    if corners.shape != (8, 3):
        raise ValueError(f"cuboid must have 8 corners, got {corners.shape[0]}")
    if frame == "out_of_face":
        return corners @ OUT_OF_FACE_TO_NATIVE.T
    if frame == "apriltag_native":
        return corners.copy()
    raise ValueError(f"unknown cuboid frame {frame!r}")


def apparent_side_px(corners: np.ndarray) -> float:
    c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    return float(np.mean(np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)))


def incidence_deg(rvec: np.ndarray, tvec: np.ndarray) -> float:
    """Angle between the tag normal and the camera->tag-centre ray (0 = head-on)."""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    n = R[:, 2]
    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    c = abs(float(np.dot(n, t) / (np.linalg.norm(t) + 1e-12)))
    return math.degrees(math.acos(min(1.0, c)))


def project(points: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, intr: Intrinsics) -> np.ndarray:
    pts, _ = cv2.projectPoints(
        np.asarray(points, dtype=np.float64).reshape(-1, 1, 3),
        np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        intr.K,
        intr.dist,
    )
    return pts.reshape(-1, 2)


def solve_tag_pose(corners_px: np.ndarray, tag_size_m: float, intr: Intrinsics) -> Pose:
    """IPPE_SQUARE; both planar solutions are evaluated and the lower-error one kept."""
    obj = tag_object_points(tag_size_m)
    img = np.asarray(corners_px, dtype=np.float64).reshape(4, 1, 2)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj, img, intr.K, intr.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if n < 1:
        raise RuntimeError("solvePnP returned no solution")
    best: Pose | None = None
    for rv, tv in zip(rvecs, tvecs):
        proj = project(obj, rv, tv, intr)
        err = float(np.mean(np.linalg.norm(proj - img.reshape(4, 2), axis=1)))
        if float(np.asarray(tv).reshape(3)[2]) <= 0:
            continue
        if best is None or err < best.reproj_err_px:
            best = Pose(rv.reshape(3), tv.reshape(3), err, incidence_deg(rv, tv))
    if best is None:
        raise RuntimeError("no PnP solution in front of the camera")
    return best


def cuboid_bbox_px(
    cuboid_native: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    intr: Intrinsics,
    shrink_frac: float = BOX_SHRINK_FRAC,
) -> tuple[tuple[float, float, float, float], bool]:
    """Project cuboid corners -> (x0, y0, x1, y1) px and a 'truncated' flag.

    The 5 % shrink is applied to the UNCLIPPED projected box, then the box is
    clipped to the image (shrinking after clipping would pull a truncated edge
    off the image border). Raises ValueError if any corner is behind the camera.
    """
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    cam = cuboid_native @ R.T + np.asarray(tvec, dtype=np.float64).reshape(1, 3)
    if np.any(cam[:, 2] <= 1e-6):
        raise ValueError("cuboid_behind_camera")
    uv = project(cuboid_native, rvec, tvec, intr)
    x0, y0 = uv.min(axis=0)
    x1, y1 = uv.max(axis=0)
    dx, dy = (x1 - x0) * shrink_frac, (y1 - y0) * shrink_frac
    x0, x1, y0, y1 = x0 + dx, x1 - dx, y0 + dy, y1 - dy
    W, H = float(intr.width), float(intr.height)
    truncated = x0 < 0 or y0 < 0 or x1 > W or y1 > H
    box = (
        float(np.clip(x0, 0, W)),
        float(np.clip(y0, 0, H)),
        float(np.clip(x1, 0, W)),
        float(np.clip(y1, 0, H)),
    )
    return box, bool(truncated)


def bbox_to_yolo(box: tuple[float, float, float, float], width: int, height: int) -> tuple[float, float, float, float]:
    x0, y0, x1, y1 = box
    return ((x0 + x1) / 2 / width, (y0 + y1) / 2 / height, (x1 - x0) / width, (y1 - y0) / height)


def tag_mask(quad_px: np.ndarray, shape: tuple[int, int], dilate_px: int = MASK_DILATE_PX) -> np.ndarray:
    """uint8 {0,255} mask of the projected tag quad dilated by dilate_px (disc kernel)."""
    m = np.zeros(shape, dtype=np.uint8)
    pts = np.round(np.asarray(quad_px, dtype=np.float64).reshape(-1, 2)).astype(np.int32)
    cv2.fillConvexPoly(m, pts, 255)
    if dilate_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilate_px + 1, 2 * dilate_px + 1))
        m = cv2.dilate(m, k)
    return m


def apply_mask(
    gray: np.ndarray, mask: np.ndarray, mode: str, rng: np.random.Generator, ring_px: int = RING_PX
) -> np.ndarray:
    if gray.ndim != 2:
        raise ValueError("apply_mask expects a single-channel image")
    out = gray.copy()
    if mode == "inpaint":
        return cv2.inpaint(out, mask, 3, cv2.INPAINT_TELEA)
    if mode != "noise":
        raise ValueError(f"unknown mask mode {mode!r}")
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * ring_px + 1, 2 * ring_px + 1))
    ring = (cv2.dilate(mask, k) > 0) & (mask == 0)
    vals = gray[ring].astype(np.float64)
    if vals.size == 0:  # quad covers the whole visible neighbourhood
        vals = gray.astype(np.float64).ravel()
    mu, sd = float(vals.mean()), float(vals.std())
    sel = mask > 0
    out[sel] = np.clip(rng.normal(mu, sd, int(sel.sum())), 0, 255).astype(np.uint8)
    return out


# --------------------------------------------------------------------------
# Index / sampling
# --------------------------------------------------------------------------
def read_index(frames_dir: Path) -> list[dict[str, Any]]:
    path = frames_dir / "frames_index.csv"
    rows: list[dict[str, Any]] = []
    with path.open(newline="") as fh:
        rd = csv.DictReader(fh)
        missing = {"frame", "t_mono", "path"} - set(rd.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing columns {sorted(missing)}")
        for r in rd:
            rows.append({"frame": int(r["frame"]), "t": float(r["t_mono"]), "path": r["path"]})
    rows.sort(key=lambda r: (r["t"], r["frame"]))
    return rows


def subsample(rows: list[dict[str, Any]], hz: float) -> list[dict[str, Any]]:
    if hz <= 0:
        return list(rows)
    period = 1.0 / hz
    out: list[dict[str, Any]] = []
    last = -math.inf
    for r in rows:
        if r["t"] - last >= period - 1e-9:
            out.append(r)
            last = r["t"]
    return out


# --------------------------------------------------------------------------
# Per-frame processing
# --------------------------------------------------------------------------
@dataclass
class Config:
    intr: Intrinsics
    tag_size_m: float
    cuboid_native: np.ndarray
    min_tag_px: float = 18.0
    max_incidence_deg: float = 55.0
    max_reproj_px: float = MAX_REPROJ_ERR_PX
    max_hamming: int = 1
    tag_id: int | None = None
    mask_mode: str = "noise"


def process_frame(gray: np.ndarray, detector: Detector, cfg: Config, rng: np.random.Generator) -> FrameResult:
    H, W = gray.shape[:2]
    if (W, H) != (cfg.intr.width, cfg.intr.height):
        return FrameResult(False, "resolution_mismatch")
    dets = list(detector.detect(gray))
    if cfg.tag_id is not None:
        dets = [d for d in dets if int(getattr(d, "tag_id", -1)) == cfg.tag_id]
    if not dets:
        return FrameResult(False, "no_tag")
    if len(dets) > 1:
        return FrameResult(False, "multiple_tags")
    det = dets[0]
    if int(getattr(det, "hamming", 0)) > cfg.max_hamming:
        return FrameResult(False, "tag_hamming")
    corners = np.asarray(det.corners, dtype=np.float64).reshape(4, 2)
    side = apparent_side_px(corners)
    rec: dict[str, Any] = {"tag_side_px": round(side, 2)}
    if side < cfg.min_tag_px:
        return FrameResult(False, "tag_too_small", rec)
    try:
        pose = solve_tag_pose(corners, cfg.tag_size_m, cfg.intr)
    except (RuntimeError, cv2.error):
        return FrameResult(False, "pnp_failed", rec)
    rec["reproj_err_px"] = round(pose.reproj_err_px, 3)
    rec["incidence_deg"] = round(pose.incidence_deg, 2)
    if pose.reproj_err_px > cfg.max_reproj_px:
        return FrameResult(False, "reproj_err", rec)
    if pose.incidence_deg > cfg.max_incidence_deg:
        return FrameResult(False, "incidence", rec)
    try:
        box, truncated = cuboid_bbox_px(cfg.cuboid_native, pose.rvec, pose.tvec, cfg.intr)
    except ValueError:
        return FrameResult(False, "cuboid_behind_camera", rec)
    if box[2] - box[0] < MIN_BOX_PX or box[3] - box[1] < MIN_BOX_PX:
        return FrameResult(False, "box_degenerate", rec)
    quad = project(tag_object_points(cfg.tag_size_m), pose.rvec, pose.tvec, cfg.intr)
    mask = tag_mask(quad, (H, W))
    masked = apply_mask(gray, mask, cfg.mask_mode, rng)
    rec.update(
        bbox_px=[round(v, 2) for v in box],
        truncated=truncated,
        masked=True,
        mask_px=int((mask > 0).sum()),
    )
    return FrameResult(True, None, rec, masked, box, quad)


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--frames", type=Path, required=True)
    p.add_argument("--intrinsics", type=Path, required=True)
    p.add_argument("--tag-size-m", type=float, default=0.10)
    p.add_argument("--cuboid", type=Path, required=True)
    p.add_argument("--cuboid-frame", choices=["out_of_face", "apriltag_native"], default="out_of_face")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--min-tag-px", type=float, default=18.0)
    p.add_argument("--max-incidence-deg", type=float, default=55.0)
    p.add_argument("--max-reproj-px", type=float, default=MAX_REPROJ_ERR_PX)
    p.add_argument("--max-hamming", type=int, default=1)
    p.add_argument("--tag-id", type=int, default=None, help="accept only this tag id")
    p.add_argument("--sample-hz", type=float, default=2.0, help="<=0 keeps every frame")
    p.add_argument("--mask", choices=["noise", "inpaint"], default="noise")
    p.add_argument("--viz", type=int, default=0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--jpeg-quality", type=int, default=95)
    return p


def run(args: argparse.Namespace, detector: Detector | None = None) -> int:
    try:
        intr = Intrinsics.from_json(args.intrinsics)
        cub = json.loads(Path(args.cuboid).read_text())["corners_tag_frame"]
        cuboid_native = cuboid_to_native(np.asarray(cub), args.cuboid_frame)
        rows = read_index(args.frames)
    except (OSError, KeyError, ValueError) as exc:
        print(f"ERROR: bad input: {exc}", file=sys.stderr)
        return EXIT_USAGE
    if detector is None:
        try:
            detector = make_detector()
        except ImportError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return EXIT_USAGE

    cfg = Config(
        intr=intr,
        tag_size_m=args.tag_size_m,
        cuboid_native=cuboid_native,
        min_tag_px=args.min_tag_px,
        max_incidence_deg=args.max_incidence_deg,
        max_reproj_px=args.max_reproj_px,
        max_hamming=args.max_hamming,
        tag_id=args.tag_id,
        mask_mode=args.mask,
    )
    out: Path = args.out
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "labels").mkdir(parents=True, exist_ok=True)

    sampled = subsample(rows, args.sample_hz)
    rng = np.random.default_rng(args.seed)
    records: list[dict[str, Any]] = []
    rejects: dict[str, int] = {}
    labeled: list[tuple[str, FrameResult]] = []

    for r in sampled:
        rec: dict[str, Any] = {"frame": r["frame"], "t": r["t"], "path": r["path"]}
        src = args.frames / r["path"]
        res: FrameResult
        gray = cv2.imread(str(src), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            res = FrameResult(False, "read_failed")
        else:
            try:
                res = process_frame(gray, detector, cfg, rng)
            except Exception as exc:  # counted, never swallowed silently
                res = FrameResult(False, f"exception:{type(exc).__name__}")
                rec["error"] = str(exc)
        rec.update(res.record)
        rec.setdefault("masked", False)
        if res.ok:
            stem = Path(r["path"]).stem
            assert res.image_out is not None and res.bbox_px is not None
            ok_img = cv2.imwrite(
                str(out / "images" / f"{stem}.jpg"),
                res.image_out,
                [cv2.IMWRITE_JPEG_QUALITY, int(args.jpeg_quality)],
            )
            if not ok_img:
                res = FrameResult(False, "write_failed", res.record)
            else:
                cx, cy, w, h = bbox_to_yolo(res.bbox_px, intr.width, intr.height)
                (out / "labels" / f"{stem}.txt").write_text(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
                labeled.append((stem, res))
        rec["status"] = "labeled" if res.ok else "rejected"
        if not res.ok:
            rec["reject_reason"] = res.reason
            rejects[str(res.reason)] = rejects.get(str(res.reason), 0) + 1
        records.append(rec)

    if args.viz > 0 and labeled:
        vdir = out / "viz"
        vdir.mkdir(exist_ok=True)
        for stem, res in random.Random(args.seed).sample(labeled, min(args.viz, len(labeled))):
            img = cv2.cvtColor(res.image_out, cv2.COLOR_GRAY2BGR)
            x0, y0, x1, y1 = (int(round(v)) for v in res.bbox_px)  # type: ignore[union-attr]
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.polylines(img, [np.round(res.tag_quad_px).astype(np.int32)], True, (0, 0, 255), 1)
            cv2.imwrite(str(vdir / f"{stem}_viz.jpg"), img)

    n_labeled = len(labeled)
    summary = {
        "n_frames_seen": len(rows),
        "n_sampled": len(sampled),
        "n_labeled": n_labeled,
        "n_rejected": sum(rejects.values()),
        "reject_counts_by_reason": dict(sorted(rejects.items())),
        "params": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()},
    }
    assert summary["n_labeled"] + summary["n_rejected"] == summary["n_sampled"]
    (out / "manifest.json").write_text(json.dumps({"summary": summary, "frames": records}, indent=1))
    print(json.dumps({k: summary[k] for k in ("n_frames_seen", "n_sampled", "n_labeled", "reject_counts_by_reason")}))
    if n_labeled == 0:
        print(
            f"ERROR: zero frames labeled out of {len(sampled)} sampled "
            f"(rejects: {summary['reject_counts_by_reason']}) -- output is EMPTY, not a success",
            file=sys.stderr,
        )
        return EXIT_NO_LABELS
    return EXIT_OK


def main(argv: Sequence[str] | None = None, detector: Detector | None = None) -> int:
    return run(build_parser().parse_args(argv), detector)


if __name__ == "__main__":
    sys.exit(main())
