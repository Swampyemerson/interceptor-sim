#!/usr/bin/env python3
"""Seeker v3 vs v2 FRAME-LEVEL detector eval on the held-out eval flights, at
the DEPLOYED operating point (docs/seeker_v3_dataset_plan.md Sec 8, step 8 of
Sec 9). This is the scorer step -- it does NOT boot a simulator; it reads
already-captured PNG/label pairs (`scripts/seeker/capture_pass_v3.sh eval`,
`scripts/seeker/capture_flight_frames.py` output layout) and runs both ONNX
models over them offline (onnxruntime, `.venv-seeker`).

WHY frame-level, not closed-loop: closed-loop miss at 12 m/s + maneuver has a
~5 m sporadic run-to-run noise floor (ADR-0057) -- an n=8 closed-loop A/B is
structurally uninformative there. Scoring both models on the IDENTICAL
captured pixels removes that noise and gives a high-n, reproducible signal.
Closed-loop only runs LATER, after these gates pass (plan Sec 8).

HONESTY BOUNDARY (CLAUDE.md / ADR-0010): `gt_*` (the filename-encoded true
range and the label-file gt box) is read HERE for SCORING ONLY -- it was
manufactured offline by capture_flight_frames.py's gt-projection chain at
CAPTURE time, exactly like a human labeler would produce. Nothing in this
file, and nothing it calls (scripts/seeker/v3_onnx_infer.py), feeds gt_* to
either ONNX model or to any detection/guidance decision -- both models score
each frame from pixels + the fixed intrinsics only, through the exact decode
+ self-mask `finetuned_seeker.FinetunedNNSeeker` flies (byte-identical, see
v3_onnx_infer.py's docstring). If v3 is ever adopted as the deployed
`MARKERLESS_NN_WEIGHTS`, it re-earns the per-tick no-cheat audit like every
prior seeker change (CLAUDE.md standing rule) -- this script's numbers rank
v2 vs v3 offline; they do not themselves constitute that audit.

SCORING (plan Sec 8): for each model, run `v3_onnx_infer.infer_raw` (conf
0.25, mask OFF -- every box) then `apply_self_mask` (deployed |bearing|<=30
deg + 40 px L/R edge margin, `finetuned_seeker.py` defaults) to get the
POST-MASK stream -- this is what feeds handoff in flight, and what M1-M5
score. The raw (mask-off) count is kept as a diagnostic column (`n_raw`) but
never scored into a metric.

A post-mask detection MATCHES a gt box iff IoU >= 0.25 OR center error <
25 px (the OR is a protocol choice, fixed before running: IoU is brittle on
the ~8 px far-range boxes this regime cares about). A detection counts as a
PHANTOM (M1) iff IoU < 0.1 vs gt (or unconditionally, on a gt-absent frame --
every post-mask detection on an empty frame is a phantom).

METRICS (plan Sec 8 table):
  M1 phantom rate  -- mean post-mask phantom-detection COUNT per frame (not a
                       fraction: a frame can carry >1 phantom).
  M2 recall        -- over gt-present frames (box >=4 px both dims), fraction
                       with >=1 matching post-mask detection.
  M3 bearing qual. -- median box-center error (px) over matched detections
                       (the single best-matching -- lowest center-error --
                       detection per matched frame).
  M4 range qual.   -- median of (range_from_box(matched det width, span) /
                       filename gt_range_m) over matched detections.
  M5 confidence sep.-- median score of TRUE (matching) vs FALSE (non-matching
                       or on an empty frame) post-mask detections, pooled
                       over every detection (not just the per-frame best).

Breakdowns: per range-bucket (<=5 / 5-15 / 15-30 / >30 m, by the filename
gt_range_m -- the true 3D range, not a box-derived one), per flight, per
direction (l2r/r2l), per path (weave/jink/line).

AGGREGATION HONESTY (plan Sec 8 / CLAUDE.md statistics rule): frames within a
flight are temporally correlated, so the pooled per-frame numbers above are
reported as DESCRIPTIVE stats only. The VERDICT statistic is the PER-FLIGHT
paired comparison: an exact two-sided binomial sign test (no scipy; count
wins/losses/ties across the 10 eval flights) on v3-vs-v2 per-flight phantom
rate and per-flight recall. "Not significant at this n" language is printed
whenever p >= 0.05 (n=10 flights is a small paired sample).

GATES G1-G5 (plan Sec 8, pre-registered before training): see gate1..gate5
below; each prints PASS/FAIL with the numbers. The declared NULL BRANCH (G1
pass + G2 fail) is printed verbatim per the plan's Sec 8 text.

REFRAMING (coordinator directive 2026-07-09, pre-registered before any real
scoring): Phase-0 forensics showed v2's phantoms are RARE + SPORADIC
(post-mask phantom-frame rate ~0.007, seed-dependent), so a phantom-rate "5x
cut" ratio computed from single-digit counts can be pure counting noise. G1
now reports the phantom metric as a raw COUNT with a 95% Wilson score
interval (`wilson_ci`, no scipy) and an explicit `underpowered` flag (v2
phantom-frame count < 10, OR the v2/v3 CIs overlap) -- when underpowered,
the ratio is NOT headlined. The overall headline verdict (`verdict()`) is
REFRAMED to be primarily far-range recall (M2, or the `--grid-dir` held-out-
range probe when given) + no-regression (G3) + a numeric "no new phantom
class" proxy; the phantom-rate delta is reported as a CI'd secondary line.
`--grid-dir` (default scripts/seeker/data/v3_grid) scores BOTH models'
recall on the Phase-1 grid's HELD-OUT ranges (3.0 m / 18.0 m, neither model
trained on them) -- the primary defensible far-range generalization number.

RUN (after `capture_pass_v3.sh eval` has produced
scripts/seeker/data/v3_flight_eval/{images,labels}/ and v3 has been trained
+ exported to scripts/seeker/weights/drone_finetuned_v3.onnx):
    .venv-seeker/bin/python scripts/eval_seeker_v3.py

SELF-TEST (no sim, no real weights, no onnxruntime import):
    .venv-seeker/bin/python scripts/eval_seeker_v3.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import statistics
import sys
import tempfile
from dataclasses import dataclass, field, asdict
from typing import Callable, Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))            # .../scripts
REPO = os.path.dirname(HERE)                                  # repo root
SEEKER_DIR = os.path.join(HERE, "seeker")
sys.path.insert(0, SEEKER_DIR)

import v3_onnx_infer as v3i  # noqa: E402 -- numpy + cv2 only, no onnxruntime at import time

# ---------------------------------------------------------------------------
# Eval flight table (docs/seeker_v3_dataset_plan.md Sec 8; mirrors
# scripts/seeker/capture_pass_v3.sh EVAL_FLIGHTS verbatim -- the master-
# seed-42 held-out eval family, never in any v2 or v3 training set).
# ---------------------------------------------------------------------------
EVAL_FLIGHTS: Dict[str, Tuple[str, int, str]] = {
    # run_tag      : (path,   speed_mps, direction)
    "evalv3_01": ("weave", 12, "l2r"),
    "evalv3_02": ("weave", 12, "r2l"),
    "evalv3_03": ("weave", 12, "l2r"),
    "evalv3_04": ("weave", 12, "r2l"),
    "evalv3_05": ("jink", 12, "l2r"),
    "evalv3_06": ("jink", 12, "r2l"),
    "evalv3_07": ("weave", 9, "l2r"),
    "evalv3_08": ("weave", 9, "r2l"),
    "evalv3_09": ("line", 9, "l2r"),
    "evalv3_10": ("line", 6, "r2l"),
}

# "Maneuvering eval frames" (plan Sec 8 G1/G2): weave/jink @ 12 m/s -- the
# failing regime (ADR-0056).
MANEUVER_FLIGHTS = ["evalv3_01", "evalv3_02", "evalv3_03", "evalv3_04", "evalv3_05", "evalv3_06"]
# v2's domain / no-regression frames (G3): the straight-line anchors.
LINE_FLIGHTS = ["evalv3_09", "evalv3_10"]

RANGE_BUCKETS = [("<=5", 0.0, 5.0), ("5-15", 5.0, 15.0), ("15-30", 15.0, 30.0), (">30", 30.0, float("inf"))]

# Deployed match/phantom protocol (plan Sec 8, fixed before running).
MATCH_IOU_THRESH = 0.25
MATCH_CENTER_PX = 25.0
PHANTOM_IOU_THRESH = 0.1
GT_MIN_PX = 4.0  # capture_flight_frames.py MIN_BOX_PX -- "gt present" floor
# A mis-lock's box is OVERSIZED vs the real target it should have hit: the
# ~293 px interior blob vs the ~8-30 px real far target (coordinator directive
# 2026-07-09). area(det) >= this * area(gt) is the oversized test.
OVERSIZE_AREA_FACTOR = 2.0

# Filenames from capture_flight_frames.py: f"{run_tag}_{seq:06d}_r{gt_range:05.1f}_{pos|neg}"
FRAME_STEM_RE = re.compile(r"^(?P<run_tag>.+)_(?P<seq>\d{6})_r(?P<grange>\d+\.\d+)_(?P<posneg>pos|neg)$")

# Static-probe filenames from probe_markerless_range.py: f"r{r:04.1f}.png"
PROBE_STEM_RE = re.compile(r"^r(?P<range>\d+\.\d+)$")

CSV_FIELDS = ["image", "model", "run_tag", "path", "dir", "gt_range_m", "gt_present",
              "n_postmask", "n_phantom", "n_mislock", "n_empty_fire", "matched",
              "best_iou", "best_center_err_px", "best_range_ratio", "best_conf", "n_raw"]

# Held-out grid ranges (Phase-1 static teleport grid, --grid-dir): entire
# range buckets neither model was trained on -- v2 never saw the grid at
# all, v3's training held these two ranges out structurally (plan Sec 6 /
# scripts/seeker_v3_capture.py HELD_OUT_RANGES_M). The systematic far-range
# generalization headline (item 5, coordinator directive 2026-07-09).
GRID_HELD_OUT_RANGES_M = (3.0, 18.0)

# Wilson score interval default z (95% two-sided).
WILSON_Z_95 = 1.96


# ---------------------------------------------------------------------------
# Pure parsing / matching primitives (self-test exercises these directly)
# ---------------------------------------------------------------------------

def parse_frame_stem(stem: str) -> Tuple[str, int, float, bool]:
    """`{run_tag}_{seq:06d}_r{gt_range:05.1f}_{pos|neg}` -> (run_tag, seq,
    gt_range_m, is_pos). Raises ValueError on a stem that doesn't match the
    capture_flight_frames.py convention."""
    m = FRAME_STEM_RE.match(stem)
    if not m:
        raise ValueError(f"frame stem does not match capture convention: {stem!r}")
    return m.group("run_tag"), int(m.group("seq")), float(m.group("grange")), m.group("posneg") == "pos"


def is_match(det: dict, gt_box: Tuple[float, float, float, float]) -> bool:
    """M2 match rule: IoU >= 0.25 OR center error < 25 px."""
    db = v3i.det_box(det)
    return v3i.iou(db, gt_box) >= MATCH_IOU_THRESH or v3i.center_err(db, gt_box) < MATCH_CENTER_PX


def is_phantom(det: dict, gt_box: Optional[Tuple[float, float, float, float]]) -> bool:
    """M1 phantom rule (POOLED, kept for G1 continuity with plan Sec 8): IoU
    < 0.1 vs gt; unconditional (every det counts) on a gt-absent frame. NOTE:
    the coordinator-directed HEADLINE phantom signal is is_mislock() below --
    the pooled rule is retained only because G1 was pre-registered on it, and
    because v2's empty-frame fire rate is ~0 (mined_neg=0) the two nearly
    coincide anyway."""
    if gt_box is None:
        return True
    return v3i.iou(v3i.det_box(det), gt_box) < PHANTOM_IOU_THRESH


def is_mislock(det: dict, gt_box: Optional[Tuple[float, float, float, float]]) -> bool:
    """POSITIVE-FRAME MIS-LOCK -- the REAL v2 failure (coordinator directive
    2026-07-09, mined_neg=0/mined_pos=31 finding): with the true target
    PRESENT, v2 locks a big interior texture INSTEAD of the small target.
    Signature = IoU < 0.1 vs gt AND the det box is OVERSIZED (area >=
    OVERSIZE_AREA_FACTOR * gt box area) -- the oversized box is exactly what
    also yields the garbage/negative range (capv3_06 r_hat=-104 m; same
    'wrong box' cause as the range-channel fragility, ADR-0059/G2). Defined
    ONLY when a real target is present (gt_box not None); empty-frame
    false-fires are counted SEPARATELY (they are ~0 in v2, so pooling them
    would under-count this, the failure that actually matters)."""
    if gt_box is None:
        return False
    db = v3i.det_box(det)
    if v3i.iou(db, gt_box) >= PHANTOM_IOU_THRESH:
        return False
    det_area = db[2] * db[3]
    gt_area = gt_box[2] * gt_box[3]
    return det_area >= OVERSIZE_AREA_FACTOR * max(gt_area, 1.0)


def gt_present(gt_box: Optional[Tuple[float, float, float, float]]) -> bool:
    return gt_box is not None and gt_box[2] >= GT_MIN_PX and gt_box[3] >= GT_MIN_PX


def wilson_ci(k: int, n: int, z: float = WILSON_Z_95) -> Tuple[float, float]:
    """95% (default z=1.96) Wilson score interval for a binomial proportion
    k/n. Pure math, no scipy (coordinator directive 2026-07-09 item 2 --
    phantom counts across the 10-flight pool are likely single-digit, so a
    plain ratio is noise-dominated; report the frame-fraction with a CI
    instead). Returns (nan, nan) for n<=0."""
    if n <= 0:
        return float("nan"), float("nan")
    phat = k / n
    z2 = z * z
    denom = 1.0 + z2 / n
    center = (phat + z2 / (2 * n)) / denom
    half = (z * math.sqrt((phat * (1 - phat) / n) + (z2 / (4 * n * n)))) / denom
    lo = max(0.0, center - half)
    hi = min(1.0, center + half)
    return lo, hi


def ci_overlap(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    """True iff two [lo, hi] intervals intersect (touching counts as overlap)."""
    return not (a[1] < b[0] or b[1] < a[0])


def paired_frames_check(frames_v2: List["FrameScore"], frames_v3: List["FrameScore"]
                         ) -> Tuple[bool, str]:
    """Item 1 (coordinator directive): v2 and v3 MUST be scored on the exact
    same frames in the exact same order (both process_eval_dir calls list
    the same eval_dir with the same sorted(os.listdir(...)), so this should
    always hold -- this function makes that invariant explicit and checkable
    rather than assumed)."""
    imgs_v2 = [f.image for f in frames_v2]
    imgs_v3 = [f.image for f in frames_v3]
    if len(imgs_v2) != len(imgs_v3):
        return False, f"frame count mismatch: v2={len(imgs_v2)} v3={len(imgs_v3)}"
    if imgs_v2 != imgs_v3:
        first_mismatch = next((i for i, (a, b) in enumerate(zip(imgs_v2, imgs_v3)) if a != b), None)
        return False, (f"frame IDENTITY/ORDER mismatch at index {first_mismatch}: "
                        f"v2={imgs_v2[first_mismatch]!r} v3={imgs_v3[first_mismatch]!r}")
    return True, f"paired: {len(imgs_v2)} identical frames, identical order"


# ---------------------------------------------------------------------------
# Per-frame scoring
# ---------------------------------------------------------------------------

@dataclass
class FrameScore:
    image: str
    model: str
    run_tag: str
    path: str
    direction: str
    gt_range_m: float
    gt_present: bool
    n_postmask: int
    n_phantom: int
    matched: bool
    best_iou: Optional[float]
    best_center_err_px: Optional[float]
    best_range_ratio: Optional[float]
    best_conf: Optional[float]
    true_scores: List[float] = field(default_factory=list)
    false_scores: List[float] = field(default_factory=list)
    n_raw: int = 0
    # coordinator directive 2026-07-09: separate the REAL failure (mis-lock on
    # a positive frame) from empty-frame false-fires (~0 in v2).
    n_mislock: int = 0
    n_empty_fire: int = 0

    def to_csv_row(self) -> dict:
        d = asdict(self)
        d["dir"] = d.pop("direction")
        d.pop("true_scores")
        d.pop("false_scores")
        return {k: d[k] for k in CSV_FIELDS}


def score_frame(postmask_dets: List[dict], gt_box: Optional[Tuple[float, float, float, float]],
                 gt_range_m: float, span_m: float, image: str, model: str, run_tag: str,
                 path: str, direction: str) -> FrameScore:
    """Score ONE frame's post-mask detections against its gt box. Pure -- no
    ONNX, no file I/O -- so the self-test exercises it directly with
    synthetic detection dicts."""
    present = gt_present(gt_box)
    eff_gt = gt_box if present else None

    n_phantom = sum(1 for d in postmask_dets if is_phantom(d, eff_gt))
    # Separated failure modes (coordinator directive 2026-07-09):
    #   mis-lock = positive-frame oversized off-target lock (the REAL v2 phantom)
    #   empty-fire = any detection on a gt-absent frame (~0 in v2)
    n_mislock = sum(1 for d in postmask_dets if is_mislock(d, eff_gt))
    n_empty_fire = len(postmask_dets) if eff_gt is None else 0

    true_scores: List[float] = []
    false_scores: List[float] = []
    best_det, best_center = None, None
    for d in postmask_dets:
        if eff_gt is not None and is_match(d, eff_gt):
            true_scores.append(d["score"])
            ce = v3i.center_err(v3i.det_box(d), eff_gt)
            if best_center is None or ce < best_center:
                best_center, best_det = ce, d
        else:
            false_scores.append(d["score"])

    matched = best_det is not None
    best_iou = best_center_err_px = best_range_ratio = best_conf = None
    if matched:
        db = v3i.det_box(best_det)
        best_iou = v3i.iou(db, eff_gt)
        best_center_err_px = best_center
        best_range_ratio = (v3i.range_from_box(best_det["w"], span_m) / gt_range_m
                             if gt_range_m > 0 else None)
        best_conf = best_det["score"]
    elif postmask_dets:
        top = v3i.best_by_score(postmask_dets)
        best_conf = top["score"]
        if eff_gt is not None:
            best_iou = v3i.iou(v3i.det_box(top), eff_gt)
            best_center_err_px = v3i.center_err(v3i.det_box(top), eff_gt)

    return FrameScore(image=image, model=model, run_tag=run_tag, path=path, direction=direction,
                       gt_range_m=gt_range_m, gt_present=present, n_postmask=len(postmask_dets),
                       n_phantom=n_phantom, matched=matched, best_iou=best_iou,
                       best_center_err_px=best_center_err_px, best_range_ratio=best_range_ratio,
                       best_conf=best_conf, true_scores=true_scores, false_scores=false_scores,
                       n_mislock=n_mislock, n_empty_fire=n_empty_fire)


DetectFn = Callable[[str], List[dict]]


def process_eval_dir(eval_dir: str, detect_fn: DetectFn, span_m: float, model_name: str,
                      flights_table: Dict[str, Tuple[str, int, str]] = EVAL_FLIGHTS
                      ) -> List[FrameScore]:
    """Core per-frame loop: list images/, parse each stem, run `detect_fn`
    (RAW mask-off detections for that image path) -> apply the deployed
    self-mask -> load the gt label -> score_frame(). `detect_fn` is
    INJECTABLE (image_path -> raw dets) precisely so --self-test can drive
    this exact pipeline (filename parsing, self-mask, matching, aggregation)
    with a synthetic stand-in and zero onnxruntime dependency."""
    img_dir = os.path.join(eval_dir, "images")
    lbl_dir = os.path.join(eval_dir, "labels")
    frames: List[FrameScore] = []
    for fn in sorted(os.listdir(img_dir)):
        if not fn.lower().endswith(".png"):
            continue
        stem = os.path.splitext(fn)[0]
        try:
            run_tag, _seq, gt_range_m, _is_pos = parse_frame_stem(stem)
        except ValueError as exc:
            print(f"[eval_v3] WARNING skipping unparseable filename {fn!r}: {exc}")
            continue
        if run_tag not in flights_table:
            print(f"[eval_v3] WARNING skipping frame with unknown run_tag {run_tag!r}: {fn}")
            continue
        path, _speed, direction = flights_table[run_tag]
        img_path = os.path.join(img_dir, fn)
        lbl_path = os.path.join(lbl_dir, stem + ".txt")

        raw_dets = detect_fn(img_path)
        postmask = v3i.apply_self_mask(raw_dets)
        gt_box = v3i.load_gt_box(lbl_path)

        fs = score_frame(postmask, gt_box, gt_range_m, span_m, fn, model_name, run_tag, path, direction)
        fs.n_raw = len(raw_dets)
        frames.append(fs)
    return frames


# ---------------------------------------------------------------------------
# Aggregate metrics (M1-M5) + breakdowns
# ---------------------------------------------------------------------------

def median_or_nan(values: List[Optional[float]]) -> float:
    vals = [v for v in values if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return statistics.median(vals) if vals else float("nan")


def phantom_rate(frames: List[FrameScore]) -> float:
    """M1: mean per-frame post-mask phantom-detection COUNT (not a fraction)."""
    if not frames:
        return float("nan")
    return sum(f.n_phantom for f in frames) / len(frames)


def phantom_frame_stats(frames: List[FrameScore]) -> dict:
    """Item 2 (coordinator directive): the phantom metric as a raw COUNT with
    a 95% Wilson CI, alongside the M1 rate. Two views:
      (a) phantom-FRAMES / total-frames (a frame counts once no matter how
          many phantoms it carries) -- the CI'd headline count.
      (b) total phantom-detections / total post-mask-detections -- the
          detection-level ratio, informational.
    Phase-0 forensics found v2's phantoms RARE + SPORADIC (post-mask
    phantom-frame rate ~0.007, seed-dependent) -- these counts are expected
    to be single-digit on some pools, which is exactly why the CI matters
    more than a point ratio."""
    n = len(frames)
    n_phantom_frames = sum(1 for f in frames if f.n_phantom >= 1)
    frac = (n_phantom_frames / n) if n > 0 else float("nan")
    lo, hi = wilson_ci(n_phantom_frames, n) if n > 0 else (float("nan"), float("nan"))
    total_phantom_dets = sum(f.n_phantom for f in frames)
    total_postmask_dets = sum(f.n_postmask for f in frames)
    det_ratio = (total_phantom_dets / total_postmask_dets) if total_postmask_dets > 0 else float("nan")
    return {
        "n_frames": n,
        "n_phantom_frames": n_phantom_frames,
        "phantom_frame_frac": frac,
        "wilson_ci_lo": lo, "wilson_ci_hi": hi,
        "total_phantom_detections": total_phantom_dets,
        "total_postmask_detections": total_postmask_dets,
        "phantom_detection_ratio": det_ratio,
    }


def mislock_frame_stats(frames: List[FrameScore]) -> dict:
    """The REAL phantom signal (coordinator directive 2026-07-09): mis-lock
    frames / POSITIVE (gt-present) frames, with a 95% Wilson CI. A mis-lock =
    an oversized off-target lock while the true target is present (is_mislock)
    -- this is what triggers the false handoff and the garbage range, and it
    is measured ONLY over frames that HAVE a real target to mis-localize
    (denominator = positive frames), so it isn't diluted by the empty frames
    that dominate a flight."""
    pos = [f for f in frames if f.gt_present]
    n = len(pos)
    n_mislock_frames = sum(1 for f in pos if f.n_mislock >= 1)
    frac = (n_mislock_frames / n) if n > 0 else float("nan")
    lo, hi = wilson_ci(n_mislock_frames, n) if n > 0 else (float("nan"), float("nan"))
    return {
        "n_positive_frames": n,
        "n_mislock_frames": n_mislock_frames,
        "mislock_frame_frac": frac,
        "wilson_ci_lo": lo, "wilson_ci_hi": hi,
        "total_mislock_detections": sum(f.n_mislock for f in pos),
    }


def empty_fire_stats(frames: List[FrameScore]) -> dict:
    """The OTHER failure mode, reported SEPARATELY: any post-mask detection on
    an EMPTY (gt-absent) frame, over empty frames, with a 95% Wilson CI. v2's
    is ~0 (mined_neg=0), so pooling it into the phantom rate would under-count
    the mis-lock that actually matters -- hence the split."""
    empt = [f for f in frames if not f.gt_present]
    n = len(empt)
    n_fire_frames = sum(1 for f in empt if f.n_empty_fire >= 1)
    frac = (n_fire_frames / n) if n > 0 else float("nan")
    lo, hi = wilson_ci(n_fire_frames, n) if n > 0 else (float("nan"), float("nan"))
    return {
        "n_empty_frames": n,
        "n_empty_fire_frames": n_fire_frames,
        "empty_fire_frame_frac": frac,
        "wilson_ci_lo": lo, "wilson_ci_hi": hi,
        "total_empty_fire_detections": sum(f.n_empty_fire for f in empt),
    }


def recall(frames: List[FrameScore]) -> Tuple[float, int, int]:
    """M2: (recall, n_matched, n_gt_present) over gt-present frames."""
    gt_frames = [f for f in frames if f.gt_present]
    if not gt_frames:
        return float("nan"), 0, 0
    n_matched = sum(1 for f in gt_frames if f.matched)
    return n_matched / len(gt_frames), n_matched, len(gt_frames)


def bearing_quality(frames: List[FrameScore]) -> float:  # M3
    return median_or_nan([f.best_center_err_px for f in frames if f.matched])


def range_quality(frames: List[FrameScore]) -> float:  # M4
    return median_or_nan([f.best_range_ratio for f in frames if f.matched])


def confidence_separation(frames: List[FrameScore]) -> Tuple[float, float]:  # M5
    true_scores: List[float] = []
    false_scores: List[float] = []
    for f in frames:
        true_scores.extend(f.true_scores)
        false_scores.extend(f.false_scores)
    return median_or_nan(true_scores), median_or_nan(false_scores)


def metrics_for(frames: List[FrameScore]) -> dict:
    r, n_matched, n_gt = recall(frames)
    tmed, fmed = confidence_separation(frames)
    return {
        "n_frames": len(frames),
        "m1_phantom_rate": phantom_rate(frames),
        "m2_recall": r, "m2_n_matched": n_matched, "m2_n_gt_present": n_gt,
        "m3_bearing_err_px_median": bearing_quality(frames),
        "m4_range_ratio_median": range_quality(frames),
        "m5_true_conf_median": tmed, "m5_false_conf_median": fmed,
        "n_raw_mean": (sum(f.n_raw for f in frames) / len(frames)) if frames else float("nan"),
    }


def group_by(frames: List[FrameScore], keyfunc) -> Dict:
    groups: Dict = {}
    for f in frames:
        groups.setdefault(keyfunc(f), []).append(f)
    return groups


def bucket_of(gt_range_m: float) -> str:
    for name, lo, hi in RANGE_BUCKETS:
        if lo <= gt_range_m < hi:
            return name
    return "?"


def breakdown(frames: List[FrameScore], keyfunc) -> Dict[str, dict]:
    groups = group_by(frames, keyfunc)
    return {str(k): metrics_for(v) for k, v in sorted(groups.items(), key=lambda kv: str(kv[0]))}


# ---------------------------------------------------------------------------
# Sign test (exact two-sided binomial, no scipy) + per-flight pairing
# ---------------------------------------------------------------------------

def sign_test(outcomes: List[int]) -> dict:
    """Exact two-sided binomial sign test over +1 (v3 win) / -1 (v3 loss) /
    0 (tie) outcomes. wins+losses = n (ties excluded from n, per convention);
    p = 2 * P(X <= min(wins,losses)) under X~Binomial(n, 0.5), capped at 1.
    No scipy dependency (CLAUDE.md/statistics-honest instruction)."""
    wins = sum(1 for o in outcomes if o > 0)
    losses = sum(1 for o in outcomes if o < 0)
    ties = sum(1 for o in outcomes if o == 0)
    n = wins + losses
    if n == 0:
        p = 1.0
    else:
        k = min(wins, losses)
        p = min(1.0, 2.0 * sum(math.comb(n, i) for i in range(k + 1)) * (0.5 ** n))
    return {"wins": wins, "losses": losses, "ties": ties, "n": n, "p_value": p,
            "significant_at_0.05": bool(n > 0 and p < 0.05)}


def per_flight_metric(frames: List[FrameScore], run_tag: str, metric_fn) -> Optional[float]:
    sub = [f for f in frames if f.run_tag == run_tag]
    if not sub:
        return None
    return metric_fn(sub)


def paired_sign_test(frames_v2: List[FrameScore], frames_v3: List[FrameScore],
                      metric_fn, higher_is_better: bool, flight_tags: List[str] = None,
                      tol: float = 1e-9) -> dict:
    """Build per-flight (v2_val, v3_val) pairs over `flight_tags` (default:
    all 10 eval flights), classify each into win/loss/tie for v3, run
    sign_test. Flights where either side is undefined (NaN, e.g. zero
    gt-present frames) are EXCLUDED from n and reported separately --
    reporting a tie there would be misleading."""
    tags = flight_tags if flight_tags is not None else list(EVAL_FLIGHTS.keys())
    outcomes, excluded, per_flight = [], [], {}
    for tag in tags:
        v2v = per_flight_metric(frames_v2, tag, metric_fn)
        v3v = per_flight_metric(frames_v3, tag, metric_fn)
        if v2v is None or v3v is None or (isinstance(v2v, float) and math.isnan(v2v)) \
                or (isinstance(v3v, float) and math.isnan(v3v)):
            excluded.append(tag)
            continue
        per_flight[tag] = {"v2": v2v, "v3": v3v}
        diff = (v3v - v2v) if higher_is_better else (v2v - v3v)
        if diff > tol:
            outcomes.append(1)
        elif diff < -tol:
            outcomes.append(-1)
        else:
            outcomes.append(0)
    result = sign_test(outcomes)
    result["excluded_flights"] = excluded
    result["per_flight"] = per_flight
    return result


# ---------------------------------------------------------------------------
# Gates G1-G5 (plan Sec 8, pre-registered)
# ---------------------------------------------------------------------------

def gate1(v2_maneuver: List[FrameScore], v3_maneuver: List[FrameScore]) -> dict:
    """G1: v3 M1 phantom rate on maneuvering frames <= 0.2x v2's on the SAME
    frames (>=5x cut) -- KEPT for bookkeeping, but per coordinator directive
    2026-07-09 item 3 this ratio is NOT headlined when the underlying counts
    are too small to trust: Phase-0 forensics showed v2's phantoms are RARE +
    SPORADIC (post-mask phantom-frame rate ~0.007, seed-dependent coin-flip),
    so a "5x cut" computed from single-digit counts can be pure counting
    noise. UNDERPOWERED iff v2's phantom-FRAME count on the maneuver pool is
    small (< 10) OR the v2/v3 95% Wilson CIs on phantom-frame fraction
    OVERLAP -- in either case `passed` is None (neither PASS nor FAIL is
    statistically defensible) and `status` is "UNDERPOWERED"."""
    v2_stats = phantom_frame_stats(v2_maneuver)
    v3_stats = phantom_frame_stats(v3_maneuver)
    v2_m1 = phantom_rate(v2_maneuver)
    v3_m1 = phantom_rate(v3_maneuver)

    v2_ci = (v2_stats["wilson_ci_lo"], v2_stats["wilson_ci_hi"])
    v3_ci = (v3_stats["wilson_ci_lo"], v3_stats["wilson_ci_hi"])
    count_underpowered = v2_stats["n_phantom_frames"] < 10
    ci_underpowered = ci_overlap(v2_ci, v3_ci)
    underpowered = bool(count_underpowered or ci_underpowered)

    ratio_note = ""
    if v2_m1 == 0:
        ratio_passed = v3_m1 == 0
        ratio_note = "v2 baseline M1 phantom rate is exactly 0 on these frames; ratio-rule requires v3==0 too"
        threshold = 0.0
    else:
        threshold = 0.2 * v2_m1
        ratio_passed = v3_m1 <= threshold

    headline = (f"v2 {v2_stats['n_phantom_frames']}/{v2_stats['n_frames']} vs "
                f"v3 {v3_stats['n_phantom_frames']}/{v3_stats['n_frames']} phantom-frames "
                f"(95% CI v2=[{v2_ci[0]:.3f},{v2_ci[1]:.3f}] v3=[{v3_ci[0]:.3f},{v3_ci[1]:.3f}])")
    if underpowered:
        why = []
        if count_underpowered:
            why.append(f"v2 phantom-frame count {v2_stats['n_phantom_frames']} < 10")
        if ci_underpowered:
            why.append("v2/v3 Wilson CIs overlap")
        headline += f" -> UNDERPOWERED ({' and '.join(why)}), not significant at this count"
        status = "UNDERPOWERED"
    else:
        headline += f"; M1 ratio v3/v2 = {(v3_m1 / v2_m1) if v2_m1 else float('nan'):.3f} (threshold <= 0.2x)"
        status = "PASS" if ratio_passed else "FAIL"

    return {"gate": "G1",
            "desc": "M1 phantom rate (maneuvering): CI'd count is the reported metric; the "
                    "<=0.2x ratio rule is kept for bookkeeping but NOT headlined when underpowered",
            "v2_m1_phantom_rate": v2_m1, "v3_m1_phantom_rate": v3_m1, "threshold": threshold,
            "v2_phantom_frame_stats": v2_stats, "v3_phantom_frame_stats": v3_stats,
            "underpowered": underpowered, "count_underpowered": count_underpowered,
            "ci_underpowered": ci_underpowered, "ratio_rule_passed": ratio_passed,
            "status": status, "headline": headline, "note": ratio_note,
            "passed": (ratio_passed if not underpowered else None)}


def gate2(v2_maneuver: List[FrameScore], v3_maneuver: List[FrameScore],
          v2_maneuver_1530: List[FrameScore], v3_maneuver_1530: List[FrameScore],
          v2_all_1530: List[FrameScore], v3_all_1530: List[FrameScore]) -> dict:
    """G2: v3 M2 recall on maneuvering frames >= v2 - 0.05 absolute, AND v3
    recall on the 15-30 m bucket >= v2 + 0.10. PROTOCOL CHOICE (documented):
    the 15-30 m clause is scored WITHIN the maneuvering-frame population
    (maneuver ∩ 15-30m bucket) since G2's heading is "recall on maneuvering
    frames" and the acquisition-band concern is specifically the 12 m/s
    dash/false-handoff regime (plan Sec 1/Sec 8). The whole-eval-pool 15-30m
    bucket is ALSO computed and reported for transparency but is not the
    gate's official clause."""
    v2_r, _, _ = recall(v2_maneuver)
    v3_r, _, _ = recall(v3_maneuver)
    clause_a = v3_r >= (v2_r - 0.05)

    v2_r_b, _, n_gt_v2b = recall(v2_maneuver_1530)
    v3_r_b, _, n_gt_v3b = recall(v3_maneuver_1530)
    clause_b = (not math.isnan(v3_r_b) and not math.isnan(v2_r_b)
                and v3_r_b >= (v2_r_b + 0.10))

    v2_r_all, _, _ = recall(v2_all_1530)
    v3_r_all, _, _ = recall(v3_all_1530)

    passed = bool(clause_a and clause_b)
    return {"gate": "G2", "desc": "M2 recall (maneuvering) v3 >= v2-0.05 AND "
                                   "(maneuvering ∩ 15-30m) v3 >= v2+0.10",
            "v2_recall_maneuver": v2_r, "v3_recall_maneuver": v3_r, "clause_a_passed": clause_a,
            "v2_recall_maneuver_15_30m": v2_r_b, "v3_recall_maneuver_15_30m": v3_r_b,
            "n_gt_present_maneuver_15_30m_v2": n_gt_v2b, "n_gt_present_maneuver_15_30m_v3": n_gt_v3b,
            "clause_b_passed": clause_b,
            "informational_wholepool_15_30m_v2_recall": v2_r_all,
            "informational_wholepool_15_30m_v3_recall": v3_r_all,
            "phase0_absolute_floor_A3_note": "v2 baseline recall on maneuvering frames "
                                              f"printed above ({v2_r:.4f}) for freezing the [A3] "
                                              "absolute floor per the plan; no separate absolute "
                                              "threshold is enforced by this gate.",
            "passed": passed}


def gate3(v2_line: List[FrameScore], v3_line: List[FrameScore]) -> dict:
    """G3 no-regression: on line-6/line-9 eval flights, v3 recall within 0.02
    of v2 AND v3 phantom rate <= 1.5x v2's. Small-count caveat: only 2 flights
    feed this gate."""
    v2_r, _, n_gt_v2 = recall(v2_line)
    v3_r, _, n_gt_v3 = recall(v3_line)
    recall_ok = (not math.isnan(v2_r) and not math.isnan(v3_r)
                 and abs(v3_r - v2_r) <= 0.02)

    v2_m1 = phantom_rate(v2_line)
    v3_m1 = phantom_rate(v3_line)
    note = "SMALL-COUNT CAVEAT: only 2 eval flights (evalv3_09, evalv3_10) feed this gate."
    if v2_m1 == 0:
        phantom_ok = v3_m1 == 0
        note += " v2 baseline phantom rate is exactly 0; using absolute-zero comparison for v3."
        threshold = 0.0
    else:
        threshold = 1.5 * v2_m1
        phantom_ok = v3_m1 <= threshold

    passed = bool(recall_ok and phantom_ok)
    return {"gate": "G3", "desc": "no-regression (line-6/line-9): |v3-v2| recall <= 0.02 "
                                   "AND v3 phantom <= 1.5x v2",
            "v2_recall": v2_r, "v3_recall": v3_r, "n_gt_present_v2": n_gt_v2,
            "n_gt_present_v3": n_gt_v3, "recall_ok": recall_ok,
            "v2_m1_phantom_rate": v2_m1, "v3_m1_phantom_rate": v3_m1, "threshold": threshold,
            "phantom_ok": phantom_ok, "note": note, "passed": passed}


def gate4(probe_results: Optional[List[dict]]) -> dict:
    """G4 static probe: OPTIONAL. If a --static-probe-dir was scored, require
    ALL probed ranges to fire (v3, RAW/mask-off) at conf >= 0.9 (the ADR-0042
    G1 probe convention, probe_markerless_range.py). Else this gate is
    reported SKIPPED, not FAIL -- it is checked separately by that script."""
    if probe_results is None:
        return {"gate": "G4", "desc": "static probe 2-12m, v3 raw conf>=0.9 at every range",
                "passed": None, "status": "SKIPPED",
                "note": "no --static-probe-dir given; G4 checked separately via "
                        "probe_markerless_range.py"}
    n = len(probe_results)
    n_fired = sum(1 for r in probe_results if r["best_conf"] is not None and r["best_conf"] >= 0.9)
    passed = n > 0 and n_fired == n
    return {"gate": "G4", "desc": "static probe 2-12m, v3 raw conf>=0.9 at every range",
            "passed": passed, "status": "PASS" if passed else "FAIL",
            "n_ranges": n, "n_fired_ge_0.9": n_fired, "results": probe_results}


HONESTY_STATEMENT = (
    "G5 honesty: gt_* labels (the filename-encoded true range, the YOLO label "
    "box) were manufactured OFFLINE ONLY by capture_flight_frames.py's "
    "gt-projection chain at capture time -- analogous to a human labeler, "
    "never a runtime guidance input. The live detection path (v3_onnx_infer / "
    "finetuned_seeker.FinetunedNNSeeker) reads camera pixels and the fixed "
    "intrinsics only; it never sees gt_*. This script uses gt_* to SCORE the "
    "two models offline -- it is not itself the no-cheat audit. If v3 replaces "
    "v2 as the deployed MARKERLESS_NN_WEIGHTS, it re-earns the per-tick "
    "no-cheat audit before any closed-loop claim (CLAUDE.md standing rule)."
)


def gate5() -> dict:
    return {"gate": "G5", "desc": "honesty (documentation gate)", "passed": True,
            "status": "PASS", "note": HONESTY_STATEMENT}


NULL_BRANCH_TEXT = (
    "NULL BRANCH (declared, plan Sec 8): G1 passes but G2 fails -- phantoms "
    "are gone, but the target is still undetectable in fast-crossing frames. "
    "Finding: the maneuver regime is beyond a 640-input nano's recall. The "
    "imgsz-960 secondary arm and/or the detect-then-track path carry the fix; "
    "v3 still ships as a phantom-rate improvement if G3/G4 hold."
)

UNDERPOWERED_TEXT = (
    "G1 UNDERPOWERED: the maneuver-pool phantom-frame counts are too small "
    "(v2 < 10) or the v2/v3 95% Wilson CIs overlap for a phantom-rate verdict "
    "to be statistically defensible at this n. The headline verdict below "
    "leans on recall (M2 / the far-range grid probe) and no-regression (G3) "
    "instead of a phantom-cut claim; the phantom-rate delta is still reported "
    "with its CI as a secondary line, not a win condition."
)


def no_new_phantom_class_proxy(v2_stats: dict, v3_stats: dict) -> dict:
    """A NUMERIC proxy only -- true 'new phantom class' identification needs
    the qualitative Phase-0 clustering method (plan Sec 3 Phase 0: cluster
    >=0.25-conf detections by (u,v,w,h,IoU-vs-gt) and eyeball the texture),
    which is out of scope for this frame-level scorer. Proxy used here: v3
    does not show a STATISTICALLY DETECTABLE INCREASE in phantom-frame count
    vs v2 (more phantom frames AND non-overlapping CIs, i.e. more phantoms
    that the CI can actually distinguish from noise). This proxy can say
    "no detectable new/worse phantom behavior"; it cannot positively confirm
    the texture identity is unchanged -- that is Phase-0's job."""
    v2_ci = (v2_stats["wilson_ci_lo"], v2_stats["wilson_ci_hi"])
    v3_ci = (v3_stats["wilson_ci_lo"], v3_stats["wilson_ci_hi"])
    detectably_worse = bool(v3_stats["n_phantom_frames"] > v2_stats["n_phantom_frames"]
                             and not ci_overlap(v2_ci, v3_ci))
    return {"no_new_phantom_class": not detectably_worse, "proxy_only": True,
            "note": "numeric proxy (v3 phantom-frame count not detectably higher than v2's at "
                    "95% CI); does NOT replace the Phase-0 qualitative texture-clustering check "
                    "for a genuinely NEW phantom class"}


def verdict(g1: dict, g2: dict, g3: dict, grid_result: Optional[Dict[float, Dict[str, dict]]],
            mislock_v2: Optional[dict] = None, mislock_v3: Optional[dict] = None
            ) -> dict:
    """REFRAMED headline (coordinator ruling 2026-07-09, data-backed: v2 static
    grid recall is ~0.96 at 18 m -- near ceiling). The PRIMARY far-range signal
    is recall on the HELD-OUT MANEUVERING flights' 15-30 m in-flight bucket
    (G2 clause_b), v2-vs-v3 on identical frames -- the regime v2 actually fails.
    The static grid probe is NOT the headline; it is a SUPPORTING no-regression/
    generalization check, 3 m and 18 m reported SEPARATELY (never averaged).
    Phantom-rate (M1/G1) is a CI'd SECONDARY, never a win claim when
    underpowered. If the primary flight far-range signal is itself within noise
    at this n, that is the declared NULL branch, NOT a fallback to the
    near-ceiling static grid number."""
    def _nan(x):
        return isinstance(x, float) and math.isnan(x)

    statement = (
        "v3 is a win iff HELD-OUT maneuvering-flight far-range recall improves "
        "(G2 15-30 m in-flight bucket, v2-vs-v3 on identical frames -- the regime "
        "v2 fails) AND straight-line recall does not regress (G3) AND no NEW phantom "
        "class appears. Static-grid far-range is SUPPORTING generalization only "
        "(v2 ~0.96 at 18 m static = near ceiling, not a discriminator). Phantom-rate "
        "is a CI'd secondary, not claimed when underpowered."
    )

    # PRIMARY far-range signal = maneuvering-FLIGHT 15-30 m recall (ALWAYS; never
    # the static grid, which v2 already aces).
    far_source = ("G2 clause_b: HELD-OUT maneuvering-flight 15-30 m in-flight recall "
                  "(PRIMARY -- v2's actual failure regime)")
    v2_far_recall = g2.get("v2_recall_maneuver_15_30m")
    v3_far_recall = g2.get("v3_recall_maneuver_15_30m")
    n_far_v2 = g2.get("n_gt_present_maneuver_15_30m_v2") or 0
    n_far_v3 = g2.get("n_gt_present_maneuver_15_30m_v3") or 0
    far_range_improved = bool(g2.get("clause_b_passed"))
    far_low_n = bool(n_far_v2 < 20)
    far_note = ("far-range in-flight bucket is LOW-N (gt-present frames v2={} v3={}); "
                "read via the paired per-flight sign test, not the pooled point estimate "
                "-- if within noise, this is the declared NULL branch, NOT a fallback to "
                "the near-ceiling static grid number".format(n_far_v2, n_far_v3)
                ) if far_low_n else ""
    far_range_inconclusive = bool(far_low_n and not far_range_improved)

    # SUPPORTING: static-grid generalization / no-regression (3 m and 18 m SEPARATE).
    grid_generalization = None
    if grid_result is not None:
        grid_generalization = {"note": (
            "SUPPORTING ONLY -- static-grid far-range (18 m) is near ceiling for v2 (~0.96) so it "
            "cannot headline; used to confirm v3 did NOT regress static acquisition when maneuvering "
            "data was added. 3 m (near) and 18 m (far) reported SEPARATELY, never averaged.")}
        for r in GRID_HELD_OUT_RANGES_M:
            gr = grid_result.get(r, grid_result.get(float(r)))
            if gr:
                v2r = gr.get("v2", {}).get("postmask_recall")
                v3r = gr.get("v3", {}).get("postmask_recall")
                held = bool(v2r is not None and v3r is not None and not _nan(v2r)
                            and not _nan(v3r) and v3r >= v2r - 0.05)
                grid_generalization[f"r{r:.1f}m"] = {
                    "v2_recall": v2r, "v3_recall": v3r, "v3_held_static_within_0.05": held}

    straight_line_ok = bool(g3["passed"])
    no_new_class = no_new_phantom_class_proxy(g1["v2_phantom_frame_stats"], g1["v3_phantom_frame_stats"])

    # WIN is driven by the FLIGHT far-range signal (not the grid). An inconclusive
    # (low-n, no improvement) flight signal cannot be a win -> honest NULL.
    win = bool(far_range_improved and straight_line_ok and no_new_class["no_new_phantom_class"])

    if g1["underpowered"]:
        phantom_claim = "NOT claimed (G1 UNDERPOWERED): " + g1["headline"]
    else:
        phantom_claim = ("claimed: " if g1["status"] == "PASS" else "NOT claimed (G1 FAIL): ") + g1["headline"]

    # MIS-LOCK as a SEPARATE signal from far-range recall (coordinator directive
    # 2026-07-09): reported apart so that "recall improved but mis-lock persists"
    # (the under-powered-counter-supervision case -> remedy is targeted capture,
    # NOT hyperparameters) is legible rather than hidden inside one number. CIs
    # overlapping => mis-lock change not significant at this n (watch-item).
    mislock = None
    if mislock_v2 is not None and mislock_v3 is not None:
        v2_ci = (mislock_v2["wilson_ci_lo"], mislock_v2["wilson_ci_hi"])
        v3_ci = (mislock_v3["wilson_ci_lo"], mislock_v3["wilson_ci_hi"])
        reduced = bool(mislock_v3["mislock_frame_frac"] < mislock_v2["mislock_frame_frac"]
                       and not ci_overlap(v2_ci, v3_ci))
        mislock = {
            "v2_mislock_frac": mislock_v2["mislock_frame_frac"],
            "v3_mislock_frac": mislock_v3["mislock_frame_frac"],
            "v2_count": f"{mislock_v2['n_mislock_frames']}/{mislock_v2['n_positive_frames']}",
            "v3_count": f"{mislock_v3['n_mislock_frames']}/{mislock_v3['n_positive_frames']}",
            "v2_ci": v2_ci, "v3_ci": v3_ci,
            "reduced_significantly": reduced,
            "note": ("mis-lock (positive-frame oversized off-target) reported SEPARATELY from "
                     "far-range recall: if recall improved but mis-lock did NOT drop, the cause is "
                     "under-powered counter-supervision (31 mined_pos) -> remedy is TARGETED "
                     "capture of the mis-lock geometry, not a hyperparameter change"),
        }

    return {
        "statement": statement,
        "far_range_source": far_source,
        "v2_far_range_recall": v2_far_recall, "v3_far_range_recall": v3_far_recall,
        "far_range_n_gt_v2": n_far_v2, "far_range_n_gt_v3": n_far_v3,
        "far_range_improved": far_range_improved,
        "far_range_low_n": far_low_n, "far_range_note": far_note,
        "far_range_inconclusive": far_range_inconclusive,
        "straight_line_no_regression": straight_line_ok,
        "grid_generalization": grid_generalization,
        "mislock": mislock,
        "no_new_phantom_class": no_new_class,
        "phantom_rate_claim": phantom_claim,
        "win": win,
    }


# ---------------------------------------------------------------------------
# ONNX-backed detect_fn factory (used only by the real (non-self-test) path)
# ---------------------------------------------------------------------------

def make_onnx_detect_fn(weights: str) -> Tuple[DetectFn, float, str]:
    import cv2  # local import: keep this out of the self-test's import graph intent
    sess, iname, imgsz = v3i.load_session(weights)
    span_m, span_src = v3i.load_span(weights)

    def _fn(img_path: str) -> List[dict]:
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"[eval_v3] WARNING could not read image {img_path}")
            return []
        return v3i.infer_raw(sess, iname, imgsz, frame, conf=v3i.DEPLOY_CONF)

    return _fn, span_m, span_src


def run_static_probe(weights: str, probe_dir: str) -> List[dict]:
    """G4: run v3 RAW (mask-off) over every r####.#.png in probe_dir, return
    per-range best-conf results."""
    import cv2
    detect_fn, _span, _src = make_onnx_detect_fn(weights)
    results = []
    for fn in sorted(os.listdir(probe_dir)):
        if not fn.lower().endswith(".png"):
            continue
        stem = os.path.splitext(fn)[0]
        m = PROBE_STEM_RE.match(stem)
        if not m:
            continue
        r = float(m.group("range"))
        dets = detect_fn(os.path.join(probe_dir, fn))
        best = v3i.best_by_score(dets)
        results.append({"range_m": r, "n_dets": len(dets),
                         "best_conf": best["score"] if best else None})
    results.sort(key=lambda d: d["range_m"])
    return results


def load_grid_index(grid_dir: str) -> List[dict]:
    """Read the Phase-1 grid's index.csv (scripts/seeker_v3_capture.py
    schema: seq, stem, kind, gt_target_x/y/z, range_m, bearing_deg, height_m,
    yaw_deg, held_out_range)."""
    idx_path = os.path.join(grid_dir, "index.csv")
    with open(idx_path, newline="") as fh:
        return list(csv.DictReader(fh))


def grid_recall_probe(grid_dir: str, detect_fn: DetectFn,
                       held_out_ranges: Tuple[float, ...] = GRID_HELD_OUT_RANGES_M
                       ) -> Dict[float, dict]:
    """Item 5 (coordinator directive 2026-07-09): score recall on the
    HELD-OUT grid ranges ONLY (default 3.0 m / 18.0 m -- neither model
    trained on these: v2 never saw the grid at all, v3's training held them
    out structurally, plan Sec 6). This is the systematic far-range
    generalization headline -- the grid is the primary v3 fix (plan Sec 2).

    Reports BOTH post-mask and RAW (mask-off) recall per range: grid targets
    sit near boresight by construction (bearings -25/0/25 deg, well inside
    the self-mask), so the two numbers should rarely differ -- reporting
    both isolates detector recall from the self-mask's own edge/bearing
    behavior rather than assuming they never interact."""
    rows = load_grid_index(grid_dir)
    by_range: Dict[float, List[dict]] = {}
    for row in rows:
        if row.get("kind") != "positive":
            continue
        if int(row.get("held_out_range", 0) or 0) != 1:
            continue
        try:
            r = float(row["range_m"])
        except (TypeError, ValueError):
            continue
        if not any(abs(r - hr) < 1e-6 for hr in held_out_ranges):
            continue
        by_range.setdefault(r, []).append(row)

    img_dir = os.path.join(grid_dir, "images")
    lbl_dir = os.path.join(grid_dir, "labels")
    out: Dict[float, dict] = {}
    for r, rows_r in sorted(by_range.items()):
        n = 0
        n_match_post = n_match_raw = 0
        for row in rows_r:
            stem = row["stem"]
            gt_box = v3i.load_gt_box(os.path.join(lbl_dir, stem + ".txt"))
            if not gt_present(gt_box):
                continue
            n += 1
            raw = detect_fn(os.path.join(img_dir, stem + ".png"))
            postmask = v3i.apply_self_mask(raw)
            if any(is_match(d, gt_box) for d in postmask):
                n_match_post += 1
            if any(is_match(d, gt_box) for d in raw):
                n_match_raw += 1
        out[r] = {
            "n": n,
            "n_matched_postmask": n_match_post,
            "postmask_recall": (n_match_post / n) if n else float("nan"),
            "n_matched_raw": n_match_raw,
            "raw_mask_off_recall": (n_match_raw / n) if n else float("nan"),
        }
    return out


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def write_csv(frames_v2: List[FrameScore], frames_v3: List[FrameScore], out_path: str) -> None:
    with open(out_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        w.writeheader()
        for f in frames_v2 + frames_v3:
            w.writerow(f.to_csv_row())


def print_metrics_line(tag: str, m: dict) -> None:
    print(f"    {tag:28s} n={m['n_frames']:4d}  M1(phantom/frame)={m['m1_phantom_rate']:.3f}  "
          f"M2(recall)={m['m2_recall']:.3f} ({m['m2_n_matched']}/{m['m2_n_gt_present']})  "
          f"M3(bearing px)={m['m3_bearing_err_px_median']:.1f}  "
          f"M4(range ratio)={m['m4_range_ratio_median']:.3f}  "
          f"M5(true/false conf)={m['m5_true_conf_median']:.3f}/{m['m5_false_conf_median']:.3f}  "
          f"n_raw_mean={m['n_raw_mean']:.2f}")


def build_report(frames_v2: List[FrameScore], frames_v3: List[FrameScore], out_dir: str,
                  probe_results_v3: Optional[List[dict]],
                  grid_result: Optional[Dict[float, Dict[str, dict]]] = None) -> dict:
    print("\n" + "=" * 78)
    print("SEEKER v3 vs v2 -- FRAME-LEVEL EVAL (docs/seeker_v3_dataset_plan.md Sec 8)")
    print("=" * 78)

    # Item 1 (coordinator directive): PAIRED IDENTICAL FRAMES -- assert it.
    paired_ok, paired_msg = paired_frames_check(frames_v2, frames_v3)
    print(f"\n[paired-frames check] {paired_msg}")
    assert paired_ok, f"v2/v3 frame pairing invariant violated: {paired_msg}"

    v2_man = [f for f in frames_v2 if f.run_tag in MANEUVER_FLIGHTS]
    v3_man = [f for f in frames_v3 if f.run_tag in MANEUVER_FLIGHTS]
    v2_line = [f for f in frames_v2 if f.run_tag in LINE_FLIGHTS]
    v3_line = [f for f in frames_v3 if f.run_tag in LINE_FLIGHTS]

    v2_man_1530 = [f for f in v2_man if bucket_of(f.gt_range_m) == "15-30"]
    v3_man_1530 = [f for f in v3_man if bucket_of(f.gt_range_m) == "15-30"]
    v2_all_1530 = [f for f in frames_v2 if bucket_of(f.gt_range_m) == "15-30"]
    v3_all_1530 = [f for f in frames_v3 if bucket_of(f.gt_range_m) == "15-30"]

    print("\n[pooled per-frame -- descriptive only; see per-flight sign test for the verdict]")
    m_v2_all, m_v3_all = metrics_for(frames_v2), metrics_for(frames_v3)
    print_metrics_line("v2 ALL eval frames", m_v2_all)
    print_metrics_line("v3 ALL eval frames", m_v3_all)
    m_v2_man, m_v3_man = metrics_for(v2_man), metrics_for(v3_man)
    print_metrics_line("v2 MANEUVER (weave/jink@12)", m_v2_man)
    print_metrics_line("v3 MANEUVER (weave/jink@12)", m_v3_man)
    m_v2_line, m_v3_line = metrics_for(v2_line), metrics_for(v3_line)
    print_metrics_line("v2 LINE (evalv3_09/10)", m_v2_line)
    print_metrics_line("v3 LINE (evalv3_09/10)", m_v3_line)

    # SEPARATED FAILURE MODES (coordinator directive 2026-07-09): the phantom is
    # a POSITIVE-frame MIS-LOCK (oversized off-target while the real target is
    # present), NOT an empty-frame false-fire (~0 in v2, mined_neg=0). Report
    # them separately AND separately from recall so we can tell which changed.
    v2_man_mislock = mislock_frame_stats(v2_man)
    v3_man_mislock = mislock_frame_stats(v3_man)
    v2_man_emptyfire = empty_fire_stats(v2_man)
    v3_man_emptyfire = empty_fire_stats(v3_man)
    print("\n[SEPARATED FAILURE MODES on the MANEUVER pool (weave/jink@12) -- "
          "phantom = positive-frame MIS-LOCK, reported apart from empty-fire AND recall]")
    for tag, ml, ef in (("v2", v2_man_mislock, v2_man_emptyfire),
                        ("v3", v3_man_mislock, v3_man_emptyfire)):
        print(f"    {tag} MIS-LOCK  : {ml['n_mislock_frames']}/{ml['n_positive_frames']} pos-frames "
              f"(frac {ml['mislock_frame_frac']:.3f}, 95% CI [{ml['wilson_ci_lo']:.3f},{ml['wilson_ci_hi']:.3f}], "
              f"{ml['total_mislock_detections']} dets)")
        print(f"    {tag} empty-fire: {ef['n_empty_fire_frames']}/{ef['n_empty_frames']} empty-frames "
              f"(frac {ef['empty_fire_frame_frac']:.3f}) -- expected ~0 for v2")

    print("\n[breakdown by range bucket -- ALL eval frames]")
    bd_range_v2 = breakdown(frames_v2, lambda f: bucket_of(f.gt_range_m))
    bd_range_v3 = breakdown(frames_v3, lambda f: bucket_of(f.gt_range_m))
    for name, _lo, _hi in RANGE_BUCKETS:
        if name in bd_range_v2:
            print_metrics_line(f"v2 range {name} m", bd_range_v2[name])
        if name in bd_range_v3:
            print_metrics_line(f"v3 range {name} m", bd_range_v3[name])

    print("\n[breakdown by flight]")
    bd_flight_v2 = breakdown(frames_v2, lambda f: f.run_tag)
    bd_flight_v3 = breakdown(frames_v3, lambda f: f.run_tag)
    for tag in EVAL_FLIGHTS:
        if tag in bd_flight_v2:
            print_metrics_line(f"v2 {tag}", bd_flight_v2[tag])
        if tag in bd_flight_v3:
            print_metrics_line(f"v3 {tag}", bd_flight_v3[tag])

    print("\n[breakdown by direction (r2l asymmetry check)]")
    bd_dir_v2 = breakdown(frames_v2, lambda f: f.direction)
    bd_dir_v3 = breakdown(frames_v3, lambda f: f.direction)
    for d in ("l2r", "r2l"):
        if d in bd_dir_v2:
            print_metrics_line(f"v2 dir={d}", bd_dir_v2[d])
        if d in bd_dir_v3:
            print_metrics_line(f"v3 dir={d}", bd_dir_v3[d])

    print("\n[breakdown by path]")
    bd_path_v2 = breakdown(frames_v2, lambda f: f.path)
    bd_path_v3 = breakdown(frames_v3, lambda f: f.path)
    for p in ("weave", "jink", "line"):
        if p in bd_path_v2:
            print_metrics_line(f"v2 path={p}", bd_path_v2[p])
        if p in bd_path_v3:
            print_metrics_line(f"v3 path={p}", bd_path_v3[p])

    # --- per-flight paired sign tests (the VERDICT statistic) ---
    print("\n[per-flight paired sign test -- the VERDICT statistic, n=10 flights]")
    st_phantom = paired_sign_test(frames_v2, frames_v3, phantom_rate, higher_is_better=False)
    st_recall = paired_sign_test(frames_v2, frames_v3, lambda fr: recall(fr)[0], higher_is_better=True)
    for name, st in (("phantom rate (v3 lower is a win)", st_phantom),
                      ("recall (v3 higher is a win)", st_recall)):
        sig_txt = ("SIGNIFICANT (p<0.05)" if st["significant_at_0.05"]
                    else "NOT SIGNIFICANT AT THIS N (p>=0.05)")
        print(f"    {name}: wins={st['wins']} losses={st['losses']} ties={st['ties']} "
              f"n={st['n']} p={st['p_value']:.4f} -> {sig_txt}"
              + (f"  [excluded (no gt-present frames): {st['excluded_flights']}]"
                 if st["excluded_flights"] else ""))

    # --- far-range grid recall probe (item 5: the primary defensible win) ---
    grid_v2 = grid_v3 = None
    if grid_result is not None:
        print("\n[far-range grid recall probe -- HELD-OUT ranges, systematic generalization]")
        for r in sorted(grid_result.keys()):
            v2g, v3g = grid_result[r].get("v2", {}), grid_result[r].get("v3", {})
            print(f"    r={r:5.1f}m  v2: postmask_recall={v2g.get('postmask_recall', float('nan')):.3f} "
                  f"({v2g.get('n_matched_postmask', 0)}/{v2g.get('n', 0)})  "
                  f"raw={v2g.get('raw_mask_off_recall', float('nan')):.3f}   |   "
                  f"v3: postmask_recall={v3g.get('postmask_recall', float('nan')):.3f} "
                  f"({v3g.get('n_matched_postmask', 0)}/{v3g.get('n', 0)})  "
                  f"raw={v3g.get('raw_mask_off_recall', float('nan')):.3f}")

    # --- gates ---
    print("\n[gates G1-G5]")
    g1 = gate1(v2_man, v3_man)
    g2 = gate2(v2_man, v3_man, v2_man_1530, v3_man_1530, v2_all_1530, v3_all_1530)
    g3 = gate3(v2_line, v3_line)
    g4 = gate4(probe_results_v3)
    g5 = gate5()
    for g in (g1, g2, g3, g4, g5):
        status = g.get("status", "PASS" if g["passed"] else "FAIL")
        print(f"    {g['gate']}: {status:8s} -- {g['desc']}")
        for k, v in g.items():
            if k in ("gate", "desc", "passed", "status"):
                continue
            print(f"        {k}: {v}")

    # --- REFRAMED headline verdict (item 4, coordinator directive) ---
    v = verdict(g1, g2, g3, grid_result, mislock_v2=v2_man_mislock, mislock_v3=v3_man_mislock)
    print("\n[VERDICT -- reframed 2026-07-09: recall/generalization primary, phantom-rate CI'd secondary]")
    print(f"    {v['statement']}")
    print(f"    far-range recall source : {v['far_range_source']}")
    print(f"    v2 far-range recall     : {v['v2_far_range_recall']} "
          f"(n_gt={v.get('far_range_n_gt_v2')})")
    print(f"    v3 far-range recall     : {v['v3_far_range_recall']} "
          f"(n_gt={v.get('far_range_n_gt_v3')})")
    print(f"    far_range_improved      : {v['far_range_improved']}")
    if v.get('far_range_low_n'):
        print(f"    far-range CAVEAT        : {v['far_range_note']}")
    if v.get('far_range_inconclusive'):
        print("    far-range INCONCLUSIVE   : primary flight signal within noise at this n "
              "-> declared NULL branch (NOT a fallback to the near-ceiling static grid number)")
    if v.get('grid_generalization'):
        print("    grid generalization (SUPPORTING, not the headline):")
        for r in GRID_HELD_OUT_RANGES_M:
            key = f"r{r:.1f}m"
            g = v['grid_generalization'].get(key)
            if g:
                print(f"        {key:>7}: v2={g['v2_recall']} v3={g['v3_recall']} "
                      f"held_static(within 0.05)={g['v3_held_static_within_0.05']}")
    if v.get('mislock'):
        ml = v['mislock']
        print("    MIS-LOCK (separate signal, the real phantom -- positive-frame oversized off-target):")
        print(f"        v2 {ml['v2_count']} (frac {ml['v2_mislock_frac']:.3f}) vs "
              f"v3 {ml['v3_count']} (frac {ml['v3_mislock_frac']:.3f}); "
              f"reduced_significantly(non-overlap CI)={ml['reduced_significantly']}")
        print(f"        {ml['note']}")
    print(f"    straight_line_no_regression (G3): {v['straight_line_no_regression']}")
    print(f"    no_new_phantom_class     : {v['no_new_phantom_class']['no_new_phantom_class']} "
          f"({v['no_new_phantom_class']['note']})")
    print(f"    phantom_rate_claim       : {v['phantom_rate_claim']}")
    print(f"    -> v3 is a WIN: {v['win']}")

    if g1["underpowered"]:
        print("\n" + UNDERPOWERED_TEXT)
    if g1["status"] == "PASS" and not g2["passed"]:
        print("\n" + NULL_BRANCH_TEXT)

    summary = {
        "paired_frames_check": {"ok": paired_ok, "message": paired_msg},
        "pooled": {"v2_all": m_v2_all, "v3_all": m_v3_all,
                   "v2_maneuver": m_v2_man, "v3_maneuver": m_v3_man,
                   "v2_line": m_v2_line, "v3_line": m_v3_line},
        "breakdown_range_bucket": {"v2": bd_range_v2, "v3": bd_range_v3},
        "breakdown_flight": {"v2": bd_flight_v2, "v3": bd_flight_v3},
        "breakdown_direction": {"v2": bd_dir_v2, "v3": bd_dir_v3},
        "breakdown_path": {"v2": bd_path_v2, "v3": bd_path_v3},
        "sign_test_phantom_rate": st_phantom,
        "sign_test_recall": st_recall,
        "grid_recall_probe": grid_result,
        "gates": {"G1": g1, "G2": g2, "G3": g3, "G4": g4, "G5": g5},
        "null_branch_triggered": bool(g1["status"] == "PASS" and not g2["passed"]),
        "verdict": v,
    }
    return summary


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-dir", default=os.path.join(SEEKER_DIR, "data", "v3_flight_eval"),
                    help="flat images/+labels/ eval pool (capture_pass_v3.sh eval output)")
    ap.add_argument("--v2", default=os.path.join(SEEKER_DIR, "weights", "drone_finetuned_v2.onnx"))
    ap.add_argument("--v3", default=os.path.join(SEEKER_DIR, "weights", "drone_finetuned_v3.onnx"))
    ap.add_argument("--out", default=os.path.join(REPO, "logs", "seeker_v3_eval"))
    ap.add_argument("--static-probe-dir", default=None,
                    help="optional dir of r####.#.png boresight probes (G4, "
                         "probe_markerless_range.py naming convention)")
    ap.add_argument("--grid-dir", default=os.path.join(SEEKER_DIR, "data", "v3_grid"),
                    help="Phase-1 static grid dir (images/+labels/+index.csv, "
                         "scripts/seeker_v3_capture.py output) -- scores the far-range "
                         "held-out-range recall probe (item 5). Skipped if index.csv is absent.")
    ap.add_argument("--self-test", action="store_true",
                    help="run the offline self-test (no sim, no real weights) and exit")
    args = ap.parse_args()

    if args.self_test:
        return _run_self_test()

    if not os.path.isdir(os.path.join(args.eval_dir, "images")):
        print(f"[eval_v3] FAILED: no images/ under {args.eval_dir} -- has "
              f"'scripts/seeker/capture_pass_v3.sh eval' been run yet?")
        return 2
    for tag, w in (("v2", args.v2), ("v3", args.v3)):
        if not os.path.isfile(w):
            print(f"[eval_v3] FAILED: {tag} weights not found: {w}")
            return 2

    os.makedirs(args.out, exist_ok=True)

    detect_v2, span_v2, src_v2 = make_onnx_detect_fn(args.v2)
    detect_v3, span_v3, src_v3 = make_onnx_detect_fn(args.v3)
    print(f"[eval_v3] v2 weights={args.v2} span={span_v2:.4f} m ({src_v2})")
    print(f"[eval_v3] v3 weights={args.v3} span={span_v3:.4f} m ({src_v3})")
    print(f"[eval_v3] eval_dir={args.eval_dir}")

    frames_v2 = process_eval_dir(args.eval_dir, detect_v2, span_v2, "v2")
    frames_v3 = process_eval_dir(args.eval_dir, detect_v3, span_v3, "v3")
    print(f"[eval_v3] scored {len(frames_v2)} frames per model")
    if not frames_v2:
        print("[eval_v3] FAILED: zero scoreable frames in eval_dir")
        return 2

    probe_results = None
    if args.static_probe_dir:
        if not os.path.isdir(args.static_probe_dir):
            print(f"[eval_v3] WARNING --static-probe-dir not found: {args.static_probe_dir}")
        else:
            probe_results = run_static_probe(args.v3, args.static_probe_dir)

    grid_result = None
    if args.grid_dir and os.path.isfile(os.path.join(args.grid_dir, "index.csv")):
        print(f"[eval_v3] grid_dir={args.grid_dir} -- scoring held-out-range recall probe "
              f"({GRID_HELD_OUT_RANGES_M} m)")
        grid_v2 = grid_recall_probe(args.grid_dir, detect_v2)
        grid_v3 = grid_recall_probe(args.grid_dir, detect_v3)
        grid_result = {r: {"v2": grid_v2.get(r, {}), "v3": grid_v3.get(r, {})}
                        for r in sorted(set(grid_v2) | set(grid_v3))}
    else:
        print(f"[eval_v3] no index.csv under {args.grid_dir} -- skipping grid recall probe "
              "(Phase-1 grid capture not run yet, or --grid-dir not given)")

    csv_path = os.path.join(args.out, "frames.csv")
    write_csv(frames_v2, frames_v3, csv_path)
    print(f"[eval_v3] wrote {csv_path}")

    summary = build_report(frames_v2, frames_v3, args.out, probe_results, grid_result)

    json_path = os.path.join(args.out, "summary.json")
    with open(json_path, "w") as fh:
        json.dump(summary, fh, indent=2, default=lambda o: None if isinstance(o, float)
                   and math.isnan(o) else o)
    print(f"\n[eval_v3] wrote {json_path}")
    return 0


# ---------------------------------------------------------------------------
# Self-test (no sim, no real weights, no onnxruntime import)
# ---------------------------------------------------------------------------

def _mk_det(cx, cy, w, h, score) -> dict:
    return {"cx": cx, "cy": cy, "w": w, "h": h, "score": score}


def _run_self_test() -> int:
    ok = True

    def check(name: str, cond: bool, detail: str = "") -> None:
        nonlocal ok
        ok = ok and cond
        print(f"[self-test] {name}: {'PASS' if cond else 'FAIL'}"
              + (f"  ({detail})" if detail and not cond else ""))

    # --- 1. filename parsing ---
    rt, seq, rng, is_pos = parse_frame_stem("evalv3_05_000123_r018.7_pos")
    check("parse_frame_stem basic", (rt, seq, rng, is_pos) == ("evalv3_05", 123, 18.7, True),
          f"got {(rt, seq, rng, is_pos)}")
    rt2, seq2, rng2, is_pos2 = parse_frame_stem("evalv3_10_000009_r100.0_neg")
    check("parse_frame_stem 3-digit range + neg + run_tag with digits",
          (rt2, seq2, rng2, is_pos2) == ("evalv3_10", 9, 100.0, False),
          f"got {(rt2, seq2, rng2, is_pos2)}")
    try:
        parse_frame_stem("garbage_name")
        check("parse_frame_stem raises on garbage", False)
    except ValueError:
        check("parse_frame_stem raises on garbage", True)

    # --- 2. match / phantom rules ---
    gt = (640.0, 480.0, 30.0, 30.0)  # small gt box near boresight
    det_overlap = _mk_det(642.0, 481.0, 32.0, 28.0, 0.55)   # near-identical box
    det_far = _mk_det(200.0, 480.0, 40.0, 40.0, 0.40)        # a phantom, far from gt
    det_close_but_low_iou = _mk_det(660.0, 480.0, 6.0, 6.0, 0.20)  # <25px center err, tiny box
    check("is_match: overlapping box matches", is_match(det_overlap, gt))
    check("is_match: far box does not match", not is_match(det_far, gt))
    check("is_match: OR clause (center<25px rescues brittle IoU)",
          is_match(det_close_but_low_iou, gt),
          f"iou={v3i.iou(v3i.det_box(det_close_but_low_iou), gt):.3f} "
          f"center_err={v3i.center_err(v3i.det_box(det_close_but_low_iou), gt):.1f}")
    check("is_phantom: far det vs real gt is a phantom", is_phantom(det_far, gt))
    check("is_phantom: overlapping det vs real gt is NOT a phantom", not is_phantom(det_overlap, gt))
    check("is_phantom: ANY det on a gt-absent frame is a phantom", is_phantom(det_far, None))

    # is_mislock: the REAL v2 phantom = far/off-target AND OVERSIZED, gt present
    det_mislock = _mk_det(200.0, 480.0, 60.0, 60.0, 0.40)   # off-target + oversized (3600 >= 2*900)
    check("is_mislock: far + OVERSIZED det on a positive frame IS a mis-lock", is_mislock(det_mislock, gt))
    check("is_mislock: far but SMALL off-target det is NOT a mis-lock (1600 < 2*900)",
          not is_mislock(det_far, gt))
    check("is_mislock: on-target det is NOT a mis-lock", not is_mislock(det_overlap, gt))
    check("is_mislock: undefined on an empty frame (gt=None) -> False", not is_mislock(det_mislock, None))

    # --- 3. score_frame: a phantom-only frame ---
    fs_phantom = score_frame([det_far], gt, gt_range_m=10.0, span_m=1.0,
                              image="x.png", model="v2", run_tag="evalv3_01",
                              path="weave", direction="l2r")
    check("score_frame: phantom frame -> not matched", not fs_phantom.matched)
    check("score_frame: phantom frame -> n_phantom==1", fs_phantom.n_phantom == 1,
          f"got {fs_phantom.n_phantom}")
    check("score_frame: phantom frame -> false_scores has the phantom's conf",
          fs_phantom.false_scores == [0.40], f"got {fs_phantom.false_scores}")

    # --- 4. score_frame: a matching frame, plus a second phantom co-visible ---
    fs_match = score_frame([det_overlap, det_far], gt, gt_range_m=10.0, span_m=0.9216,
                            image="y.png", model="v3", run_tag="evalv3_01",
                            path="weave", direction="l2r")
    check("score_frame: matched frame -> matched True", fs_match.matched)
    check("score_frame: matched frame -> best_conf is the matching det's score",
          fs_match.best_conf == 0.55, f"got {fs_match.best_conf}")
    check("score_frame: matched frame -> the co-visible far det still counts as phantom",
          fs_match.n_phantom == 1, f"got {fs_match.n_phantom}")
    exp_ratio = v3i.range_from_box(det_overlap["w"], 0.9216) / 10.0
    check("score_frame: best_range_ratio uses filename gt_range_m + calib span",
          fs_match.best_range_ratio is not None and abs(fs_match.best_range_ratio - exp_ratio) < 1e-9)

    # --- 5. score_frame: gt box below the 4px floor -> treated as gt-absent ---
    tiny_gt = (640.0, 480.0, 2.0, 2.0)
    fs_tinygt = score_frame([det_far], tiny_gt, gt_range_m=90.0, span_m=1.0,
                             image="z.png", model="v2", run_tag="evalv3_05",
                             path="jink", direction="r2l")
    check("score_frame: sub-4px gt box treated as gt_present=False", not fs_tinygt.gt_present)
    check("score_frame: gt-absent frame -> any det is a phantom (count==1)",
          fs_tinygt.n_phantom == 1, f"got {fs_tinygt.n_phantom}")

    # --- 6. aggregate metrics on a tiny synthetic frame set ---
    # NOTE: fs_phantom was scored against the REAL gt box too (gt_present=True,
    # not matched -- its lone detection is a phantom); fs_match is gt_present
    # and matched; fs_tinygt's gt is below the 4px floor (gt_present=False).
    # So of the 3 frames, 2 carry a real gt box and exactly 1 of those matches.
    frames = [fs_phantom, fs_match, fs_tinygt]
    check("phantom_rate: mean over 3 frames == 1.0 (each has exactly 1 phantom)",
          abs(phantom_rate(frames) - 1.0) < 1e-9, f"got {phantom_rate(frames)}")
    r, n_matched, n_gt = recall(frames)
    check("recall: 1 matched / 2 gt-present (fs_phantom + fs_match; fs_tinygt excluded)",
          n_gt == 2 and n_matched == 1 and abs(r - 0.5) < 1e-9,
          f"got recall={r} matched={n_matched} gt={n_gt}")
    check("bearing_quality: median over the single matched frame equals its center_err",
          abs(bearing_quality(frames) - fs_match.best_center_err_px) < 1e-9)
    tmed, fmed = confidence_separation(frames)
    check("confidence_separation: true median == 0.55, false median == 0.40",
          abs(tmed - 0.55) < 1e-9 and abs(fmed - 0.40) < 1e-9, f"got {tmed}, {fmed}")

    # --- 6b. wilson_ci: hand-derived known values (Wikipedia's Wilson-score
    #         worked example n=100,k=50 -> ~95% CI (0.404, 0.596); a k=0,n=30
    #         "rule of three"-adjacent case), no scipy ---
    lo, hi = wilson_ci(50, 100)
    check("wilson_ci: k=50,n=100 center is exactly 0.5 (phat=0.5 always centers exactly)",
          abs((lo + hi) / 2.0 - 0.5) < 1e-9, f"got center={(lo+hi)/2.0}")
    check("wilson_ci: k=50,n=100 -> 95% CI ~ (0.404, 0.596) [Wikipedia worked example]",
          abs(lo - 0.403830) < 1e-3 and abs(hi - 0.596170) < 1e-3, f"got ({lo}, {hi})")
    lo0, hi0 = wilson_ci(0, 30)
    check("wilson_ci: k=0,n=30 -> lo==0.0, hi ~ 0.1135 (hand-derived)",
          abs(lo0 - 0.0) < 1e-9 and abs(hi0 - 0.113517) < 1e-3, f"got ({lo0}, {hi0})")
    check("wilson_ci: n<=0 -> (nan, nan)",
          all(math.isnan(v) for v in wilson_ci(0, 0)))

    check("ci_overlap: identical intervals overlap", ci_overlap((0.1, 0.3), (0.1, 0.3)))
    check("ci_overlap: touching intervals overlap (closed)", ci_overlap((0.1, 0.2), (0.2, 0.3)))
    check("ci_overlap: disjoint intervals do not overlap", not ci_overlap((0.1, 0.2), (0.3, 0.4)))
    check("ci_overlap: order-independent", ci_overlap((0.3, 0.4), (0.1, 0.2)) == ci_overlap((0.1, 0.2), (0.3, 0.4)))

    # --- 6c. phantom_frame_stats: known counts/ratios ---
    pf_frames = [fs_phantom, fs_match, fs_tinygt]  # n_phantom = [1, 1, 1] over 3 frames
    pf = phantom_frame_stats(pf_frames)
    check("phantom_frame_stats: n_phantom_frames==3 (every frame has >=1 phantom)",
          pf["n_phantom_frames"] == 3, f"got {pf}")
    check("phantom_frame_stats: phantom_frame_frac == 1.0", abs(pf["phantom_frame_frac"] - 1.0) < 1e-9)
    check("phantom_frame_stats: total_phantom_detections==3 (1 phantom per frame), "
          "total_postmask_detections==4 (fs_match carries 2: det_overlap+det_far; "
          "fs_phantom/fs_tinygt carry 1 each) -> ratio==0.75",
          pf["total_phantom_detections"] == 3 and pf["total_postmask_detections"] == 4
          and abs(pf["phantom_detection_ratio"] - 0.75) < 1e-9, f"got {pf}")
    pf_empty = phantom_frame_stats([])
    check("phantom_frame_stats: empty frame list -> nan frac, nan CI",
          math.isnan(pf_empty["phantom_frame_frac"]) and math.isnan(pf_empty["wilson_ci_lo"]))

    # --- 6c-bis. mis-lock vs empty-fire separation (coordinator 2026-07-09) ---
    fs_mislock = score_frame([det_mislock], gt, gt_range_m=20.0, span_m=1.0,
                              image="ml.png", model="v2", run_tag="evalv3_01", path="weave", direction="l2r")
    fs_ontarget = score_frame([det_overlap], gt, gt_range_m=5.0, span_m=1.0,
                               image="ok.png", model="v2", run_tag="evalv3_01", path="weave", direction="l2r")
    fs_emptyfire = score_frame([det_far], None, gt_range_m=30.0, span_m=1.0,
                                image="ef.png", model="v2", run_tag="evalv3_01", path="weave", direction="l2r")
    check("score_frame: mis-lock frame -> n_mislock==1, n_empty_fire==0",
          fs_mislock.n_mislock == 1 and fs_mislock.n_empty_fire == 0, f"got {fs_mislock}")
    check("score_frame: on-target frame -> n_mislock==0", fs_ontarget.n_mislock == 0)
    check("score_frame: empty frame with a det -> n_empty_fire==1, n_mislock==0",
          fs_emptyfire.n_empty_fire == 1 and fs_emptyfire.n_mislock == 0, f"got {fs_emptyfire}")
    mls = mislock_frame_stats([fs_mislock, fs_ontarget, fs_emptyfire])
    check("mislock_frame_stats: denominator is POSITIVE frames only (2), 1 mis-lock -> frac 0.5",
          mls["n_positive_frames"] == 2 and mls["n_mislock_frames"] == 1
          and abs(mls["mislock_frame_frac"] - 0.5) < 1e-9, f"got {mls}")
    efs = empty_fire_stats([fs_mislock, fs_ontarget, fs_emptyfire])
    check("empty_fire_stats: denominator is EMPTY frames only (1), 1 fired -> frac 1.0",
          efs["n_empty_frames"] == 1 and efs["n_empty_fire_frames"] == 1
          and abs(efs["empty_fire_frame_frac"] - 1.0) < 1e-9, f"got {efs}")

    # --- 6d. paired_frames_check (item 1): identical vs mismatched frame lists ---
    fa = FrameScore(image="a.png", model="v2", run_tag="evalv3_01", path="weave", direction="l2r",
                     gt_range_m=10.0, gt_present=False, n_postmask=0, n_phantom=0, matched=False,
                     best_iou=None, best_center_err_px=None, best_range_ratio=None, best_conf=None)
    fb = FrameScore(image="b.png", model="v2", run_tag="evalv3_01", path="weave", direction="l2r",
                     gt_range_m=10.0, gt_present=False, n_postmask=0, n_phantom=0, matched=False,
                     best_iou=None, best_center_err_px=None, best_range_ratio=None, best_conf=None)
    fa3, fb3 = FrameScore(**{**vars(fa), "model": "v3"}), FrameScore(**{**vars(fb), "model": "v3"})
    ok_pair, msg_pair = paired_frames_check([fa, fb], [fa3, fb3])
    check("paired_frames_check: identical image lists/order -> True", ok_pair, msg_pair)
    ok_bad, msg_bad = paired_frames_check([fa, fb], [fb3, fa3])
    check("paired_frames_check: same set, DIFFERENT order -> False with a diagnostic message",
          not ok_bad and "mismatch" in msg_bad, msg_bad)
    ok_bad2, msg_bad2 = paired_frames_check([fa], [fa3, fb3])
    check("paired_frames_check: different frame COUNT -> False",
          not ok_bad2 and "count mismatch" in msg_bad2, msg_bad2)

    # --- 7. sign test: known win/loss vectors ---
    st_allwin = sign_test([1, 1, 1, 1, 1, 1, 1, 1, 1, 1])  # n=10, all wins
    check("sign_test: 10/10 wins is significant (p<0.05)",
          st_allwin["n"] == 10 and st_allwin["wins"] == 10 and st_allwin["significant_at_0.05"],
          f"got {st_allwin}")
    # exact value: p = 2 * C(10,0) * 0.5^10 = 2/1024 = 0.001953125
    check("sign_test: 10/10 wins exact p-value",
          abs(st_allwin["p_value"] - (2.0 / 1024.0)) < 1e-9, f"got {st_allwin['p_value']}")

    st_5_5 = sign_test([1, -1, 1, -1, 1, -1, 1, -1, 1, -1])  # perfectly split
    check("sign_test: 5-5 split -> p==1.0, not significant",
          abs(st_5_5["p_value"] - 1.0) < 1e-9 and not st_5_5["significant_at_0.05"],
          f"got {st_5_5}")

    st_7_3 = sign_test([1, 1, 1, 1, 1, 1, 1, -1, -1, -1])  # 7 wins / 3 losses
    # exact: n=10 k=3, p=2*(C(10,0)+C(10,1)+C(10,2)+C(10,3))*0.5^10
    exp_p = 2.0 * sum(math.comb(10, i) for i in range(4)) * (0.5 ** 10)
    check("sign_test: 7-3 split exact p-value + not-significant language threshold",
          abs(st_7_3["p_value"] - exp_p) < 1e-9, f"got {st_7_3['p_value']} vs {exp_p}")
    check("sign_test: 7-3 split, n=10, is NOT significant at 0.05 (honest small-n language)",
          not st_7_3["significant_at_0.05"], f"p={st_7_3['p_value']:.4f}")

    st_ties = sign_test([1, 1, 0, 0, -1])
    check("sign_test: ties counted separately from n",
          st_ties["wins"] == 2 and st_ties["losses"] == 1 and st_ties["ties"] == 2
          and st_ties["n"] == 3, f"got {st_ties}")

    # --- 8. gate math on contrived numbers ---
    def _frames_with_phantom_frame_count(n_phantom_frames: int, n: int, run_tag: str,
                                          gt_present_frac: float = 1.0,
                                          matched_frac: float = 1.0) -> List[FrameScore]:
        """n frames where exactly n_phantom_frames of them carry ONE phantom
        each (so phantom_rate() == phantom_frame_stats()['phantom_frame_frac']
        exactly -- easy to reason about for both the M1 rate and the new
        CI'd frame-fraction in the same synthetic set)."""
        n_gt = int(round(n * gt_present_frac))
        n_matched = int(round(n_gt * matched_frac))
        out = []
        for i in range(n):
            has_gt = i < n_gt
            matched = has_gt and i < n_matched
            fs = FrameScore(image=f"{i}.png", model="x", run_tag=run_tag, path="weave",
                             direction="l2r", gt_range_m=20.0, gt_present=has_gt,
                             n_postmask=0, n_phantom=(1 if i < n_phantom_frames else 0),
                             matched=matched,
                             best_iou=(0.5 if matched else None),
                             best_center_err_px=(2.0 if matched else None),
                             best_range_ratio=(1.0 if matched else None),
                             best_conf=(0.5 if matched else None))
            out.append(fs)
        return out

    # Scenario A: NOT underpowered (v2 count=40>=10, CIs disjoint), ratio PASSES.
    v2_A = _frames_with_phantom_frame_count(40, 200, "evalv3_01", matched_frac=0.5)   # mean 0.2
    v3_A = _frames_with_phantom_frame_count(2, 200, "evalv3_01", matched_frac=0.5)    # mean 0.01
    g1_A = gate1(v2_A, v3_A)
    check("gate1 scenario A: not underpowered (v2=40/200, v3=2/200, CIs disjoint)",
          g1_A["underpowered"] is False, f"got {g1_A}")
    check("gate1 scenario A: ratio 0.01<=0.2*0.2(0.04) -> status PASS, passed True",
          g1_A["status"] == "PASS" and g1_A["passed"] is True, f"got {g1_A}")

    # Scenario B: NOT underpowered (v2=40/200, v3=80/200, CIs disjoint), ratio FAILS.
    v2_B = _frames_with_phantom_frame_count(40, 200, "evalv3_01", matched_frac=0.5)   # mean 0.2
    v3_B = _frames_with_phantom_frame_count(80, 200, "evalv3_01", matched_frac=0.5)   # mean 0.4
    g1_B = gate1(v2_B, v3_B)
    check("gate1 scenario B: not underpowered (v2=40/200, v3=80/200, CIs disjoint)",
          g1_B["underpowered"] is False, f"got {g1_B}")
    check("gate1 scenario B: ratio 0.4 > 0.2*0.2(0.04) -> status FAIL, passed False",
          g1_B["status"] == "FAIL" and g1_B["passed"] is False, f"got {g1_B}")

    # Scenario C: UNDERPOWERED via v2's phantom-frame COUNT < 10 (v2=7/300).
    v2_C = _frames_with_phantom_frame_count(7, 300, "evalv3_01", matched_frac=0.5)
    v3_C = _frames_with_phantom_frame_count(0, 300, "evalv3_01", matched_frac=0.5)
    g1_C = gate1(v2_C, v3_C)
    check("gate1 scenario C: underpowered via v2 count<10 (v2 n_phantom_frames=7)",
          g1_C["underpowered"] is True and g1_C["count_underpowered"] is True, f"got {g1_C}")
    check("gate1 scenario C: status UNDERPOWERED, passed is None (ratio NOT headlined)",
          g1_C["status"] == "UNDERPOWERED" and g1_C["passed"] is None, f"got {g1_C}")

    # Scenario D: UNDERPOWERED via CI OVERLAP despite v2 count>=10 (v2=12/300, v3=8/300).
    v2_D = _frames_with_phantom_frame_count(12, 300, "evalv3_01", matched_frac=0.5)
    v3_D = _frames_with_phantom_frame_count(8, 300, "evalv3_01", matched_frac=0.5)
    g1_D = gate1(v2_D, v3_D)
    check("gate1 scenario D: v2 count=12>=10 (count check alone would NOT trigger)",
          g1_D["count_underpowered"] is False, f"got {g1_D}")
    check("gate1 scenario D: underpowered via CI overlap (v2=12/300 vs v3=8/300)",
          g1_D["underpowered"] is True and g1_D["ci_underpowered"] is True, f"got {g1_D}")
    check("gate1 scenario D: status UNDERPOWERED, passed is None",
          g1_D["status"] == "UNDERPOWERED" and g1_D["passed"] is None, f"got {g1_D}")

    # Zero-baseline (v2 has ZERO phantoms at all): always underpowered by the
    # count rule (0 phantom-frames < 10), regardless of v3 -- but the
    # ratio_rule_passed bookkeeping field still reflects the old rule.
    g1_zero = gate1(_frames_with_phantom_frame_count(0, 50, "evalv3_01"),
                     _frames_with_phantom_frame_count(0, 50, "evalv3_01"))
    check("gate1 zero-baseline: underpowered True (0 phantom-frames < 10)",
          g1_zero["underpowered"] is True and g1_zero["passed"] is None, f"got {g1_zero}")
    check("gate1 zero-baseline: ratio_rule_passed bookkeeping still True (v3==0==v2)",
          g1_zero["ratio_rule_passed"] is True, f"got {g1_zero}")
    g1_zero_v3nonzero = gate1(_frames_with_phantom_frame_count(0, 50, "evalv3_01"),
                               _frames_with_phantom_frame_count(5, 50, "evalv3_01"))
    check("gate1 zero-baseline, v3 nonzero: ratio_rule_passed bookkeeping False, still underpowered",
          g1_zero_v3nonzero["ratio_rule_passed"] is False
          and g1_zero_v3nonzero["underpowered"] is True
          and g1_zero_v3nonzero["passed"] is None, f"got {g1_zero_v3nonzero}")

    # --- 8b. no_new_phantom_class_proxy ---
    no_new_ab = no_new_phantom_class_proxy(g1_A["v2_phantom_frame_stats"], g1_A["v3_phantom_frame_stats"])
    check("no_new_phantom_class_proxy: v3 has FEWER phantom-frames than v2 -> no_new_phantom_class True",
          no_new_ab["no_new_phantom_class"] is True, f"got {no_new_ab}")
    no_new_ba = no_new_phantom_class_proxy(g1_B["v2_phantom_frame_stats"], g1_B["v3_phantom_frame_stats"])
    check("no_new_phantom_class_proxy: v3 has detectably MORE phantom-frames than v2 (disjoint CIs) -> False",
          no_new_ba["no_new_phantom_class"] is False, f"got {no_new_ba}")
    no_new_cd = no_new_phantom_class_proxy(g1_D["v2_phantom_frame_stats"], g1_D["v3_phantom_frame_stats"])
    check("no_new_phantom_class_proxy: v3 has FEWER phantom-frames (12->8), CIs overlap -> True",
          no_new_cd["no_new_phantom_class"] is True, f"got {no_new_cd}")

    # --- 8c. verdict(): reframed headline, both the grid-provided and the
    #         G2-fallback far-range source, and the underpowered phantom-claim wording ---
    g2_pass_farbucket = {"clause_b_passed": True, "v2_recall_maneuver_15_30m": 0.2,
                          "v3_recall_maneuver_15_30m": 0.4}
    g3_pass = {"passed": True}
    g3_fail = {"passed": False}

    v_fallback_win = verdict(g1_A, g2_pass_farbucket, g3_pass, None)
    check("verdict (no grid, G2 far-bucket clause_b True, G3 pass, G1 not underpowered -> WIN)",
          v_fallback_win["win"] is True and v_fallback_win["far_range_improved"] is True,
          f"got {v_fallback_win}")
    check("verdict: phantom_rate_claim is CLAIMED when G1 status PASS (scenario A)",
          v_fallback_win["phantom_rate_claim"].startswith("claimed:"), v_fallback_win["phantom_rate_claim"])

    v_fallback_lose_g3 = verdict(g1_A, g2_pass_farbucket, g3_fail, None)
    check("verdict: G3 fail alone blocks the win even if far-range recall improved",
          v_fallback_lose_g3["win"] is False, f"got {v_fallback_lose_g3}")

    grid_win = {18.0: {"v2": {"postmask_recall": 0.10, "n": 30, "n_matched_postmask": 3},
                        "v3": {"postmask_recall": 0.60, "n": 30, "n_matched_postmask": 18}},
                3.0: {"v2": {"postmask_recall": 0.95, "n": 20, "n_matched_postmask": 19},
                      "v3": {"postmask_recall": 0.95, "n": 20, "n_matched_postmask": 19}}}
    v_grid_win = verdict(g1_A, g2_pass_farbucket, g3_pass, grid_win)
    check("verdict: PRIMARY far-range source is the maneuvering FLIGHT 15-30m (G2), NOT the grid",
          "flight" in v_grid_win["far_range_source"].lower()
          and "grid" not in v_grid_win["far_range_source"].lower(),
          v_grid_win["far_range_source"])
    check("verdict: win driven by FLIGHT far-range (G2 clause_b True) -> WIN True",
          v_grid_win["far_range_improved"] is True and v_grid_win["win"] is True, f"got {v_grid_win}")
    check("verdict: grid generalization reported SEPARATELY for 3m and 18m (supporting), 18m held_static True",
          v_grid_win["grid_generalization"] is not None
          and "r3.0m" in v_grid_win["grid_generalization"]
          and "r18.0m" in v_grid_win["grid_generalization"]
          and v_grid_win["grid_generalization"]["r18.0m"]["v3_held_static_within_0.05"] is True,
          f"got {v_grid_win.get('grid_generalization')}")

    # grid-18m REGRESSES but the flight far-range still improved: win STILL True
    # (grid is SUPPORTING-only, never a win-blocker), and the regression is FLAGGED
    # separately (3m and 18m never averaged).
    grid_regress = {18.0: {"v2": {"postmask_recall": 0.50}, "v3": {"postmask_recall": 0.40}},
                    3.0: {"v2": {"postmask_recall": 0.80}, "v3": {"postmask_recall": 0.82}}}
    v_grid_regress = verdict(g1_A, g2_pass_farbucket, g3_pass, grid_regress)
    check("verdict: grid-18m regression does NOT block the win (grid is supporting-only)",
          v_grid_regress["win"] is True, f"got {v_grid_regress}")
    check("verdict: grid-18m regression IS flagged held_static False while 3m held True",
          v_grid_regress["grid_generalization"]["r18.0m"]["v3_held_static_within_0.05"] is False
          and v_grid_regress["grid_generalization"]["r3.0m"]["v3_held_static_within_0.05"] is True,
          f"got {v_grid_regress['grid_generalization']}")

    # verdict: MIS-LOCK reported as a SEPARATE signal (coordinator 2026-07-09)
    ml_v2 = mislock_frame_stats([fs_mislock] * 20)   # 20 pos frames, all mis-lock -> frac 1.0
    ml_v3 = mislock_frame_stats([fs_ontarget] * 20)  # 20 pos frames, none -> frac 0.0
    v_ml = verdict(g1_A, g2_pass_farbucket, g3_pass, None, mislock_v2=ml_v2, mislock_v3=ml_v3)
    check("verdict: mis-lock is a separate signal; v3 0.0 vs v2 1.0 (non-overlap CI) -> reduced_significantly True",
          v_ml["mislock"] is not None and v_ml["mislock"]["reduced_significantly"] is True,
          f"got {v_ml.get('mislock')}")
    check("verdict: no mis-lock stats given -> mislock None (back-compat with earlier calls)",
          verdict(g1_A, g2_pass_farbucket, g3_pass, None)["mislock"] is None)

    v_underpowered = verdict(g1_C, g2_pass_farbucket, g3_pass, grid_win)
    check("verdict: G1 UNDERPOWERED (scenario C) -> phantom_rate_claim explicitly says NOT claimed",
          v_underpowered["phantom_rate_claim"].startswith("NOT claimed (G1 UNDERPOWERED)"),
          v_underpowered["phantom_rate_claim"])
    check("verdict: underpowered G1 does not itself block the win (win is recall/G3/no-new-class driven)",
          v_underpowered["win"] is True, f"got {v_underpowered}")

    st_g5 = gate5()
    check("gate5: honesty gate always PASS", st_g5["passed"] is True)

    g4_skip = gate4(None)
    check("gate4: skipped when no probe dir -> status SKIPPED, passed None",
          g4_skip["status"] == "SKIPPED" and g4_skip["passed"] is None)
    g4_pass = gate4([{"range_m": r, "n_dets": 1, "best_conf": 0.95} for r in range(2, 13)])
    check("gate4: all ranges >=0.9 conf -> PASS", g4_pass["passed"] is True)
    g4_fail = gate4([{"range_m": 2, "n_dets": 1, "best_conf": 0.95},
                      {"range_m": 12, "n_dets": 0, "best_conf": None}])
    check("gate4: one range never fires -> FAIL", g4_fail["passed"] is False)

    # --- 9. bucket_of edges ---
    check("bucket_of: 5.0 falls in 5-15 (half-open upper)", bucket_of(5.0) == "5-15")
    check("bucket_of: 4.999 falls in <=5", bucket_of(4.999) == "<=5")
    check("bucket_of: 30.0 falls in >30", bucket_of(30.0) == ">30")
    check("bucket_of: 29.999 falls in 15-30", bucket_of(29.999) == "15-30")

    # --- 10. end-to-end pipeline with an INJECTABLE (non-ONNX) detect_fn on a
    #         synthetic on-disk eval_dir (real files, no sim, no real weights) ---
    with tempfile.TemporaryDirectory() as td:
        img_dir = os.path.join(td, "images")
        lbl_dir = os.path.join(td, "labels")
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)
        import numpy as np
        blank = np.zeros((960, 1280, 3), dtype="uint8")
        import cv2
        # frame 1: evalv3_01 (maneuver, weave l2r), gt present near boresight
        stem1 = "evalv3_01_000001_r012.0_pos"
        cv2.imwrite(os.path.join(img_dir, stem1 + ".png"), blank)
        with open(os.path.join(lbl_dir, stem1 + ".txt"), "w") as fh:
            fh.write("0 0.500000 0.500000 0.030000 0.030000\n")  # 38.4x28.8 px @1280x960
        # frame 2: evalv3_09 (line, gt absent / negative frame)
        stem2 = "evalv3_09_000002_r050.0_neg"
        cv2.imwrite(os.path.join(img_dir, stem2 + ".png"), blank)
        open(os.path.join(lbl_dir, stem2 + ".txt"), "w").close()

        def fake_detect(img_path: str) -> List[dict]:
            # a deterministic stand-in for onnxruntime: fires exactly at
            # boresight for frame 1 (a match), fires nowhere for frame 2.
            if stem1 in img_path:
                return [_mk_det(640.0, 480.0, 40.0, 40.0, 0.6)]
            return []

        frames = process_eval_dir(td, fake_detect, span_m=1.0, model_name="v3fake")
        check("process_eval_dir: scores both synthetic frames", len(frames) == 2,
              f"got {len(frames)}")
        by_tag = {f.run_tag: f for f in frames}
        check("process_eval_dir: evalv3_01 frame matched via the injected detector",
              by_tag["evalv3_01"].matched, f"got {by_tag.get('evalv3_01')}")
        check("process_eval_dir: evalv3_09 negative frame has zero phantoms (no dets fired)",
              by_tag["evalv3_09"].n_phantom == 0, f"got {by_tag['evalv3_09'].n_phantom}")
        check("process_eval_dir: n_raw plumbed through from detect_fn",
              by_tag["evalv3_01"].n_raw == 1 and by_tag["evalv3_09"].n_raw == 0)

        # CSV round-trip
        csv_path = os.path.join(td, "frames.csv")
        write_csv(frames, [], csv_path)
        with open(csv_path) as fh:
            rows = list(csv.DictReader(fh))
        check("write_csv: header matches CSV_FIELDS and rows round-trip",
              list(rows[0].keys()) == CSV_FIELDS and len(rows) == 2, f"got {rows}")

    # --- 11. grid_recall_probe (item 5): synthetic on-disk grid matching the
    #          REAL scripts/seeker_v3_capture.py index.csv schema exactly
    #          (seq, stem, kind, gt_target_x/y/z, range_m, bearing_deg,
    #          height_m, yaw_deg, held_out_range) -- real files, no sim. ---
    with tempfile.TemporaryDirectory() as gd:
        gimg = os.path.join(gd, "images")
        glbl = os.path.join(gd, "labels")
        os.makedirs(gimg, exist_ok=True)
        os.makedirs(glbl, exist_ok=True)
        import numpy as np
        import cv2
        blank = np.zeros((960, 1280, 3), dtype="uint8")

        # 2 held-out r=18.0m positives, 1 held-out r=3.0m positive, 1
        # NON-held-out r=6.0m positive (must be excluded), 1 negative (must
        # be excluded). One of the r=18.0m rows places its gt box near the
        # RIGHT edge (within the 40px self-mask margin) so post-mask and raw
        # recall can be told apart by a detector that fires there.
        grid_rows = [
            {"seq": 0, "stem": "g0_r18_center", "kind": "positive", "gt_target_x": "18.0",
             "gt_target_y": "0.0", "gt_target_z": "0.5", "range_m": "18.0", "bearing_deg": "0.0",
             "height_m": "0.5", "yaw_deg": "0.0", "held_out_range": 1},
            {"seq": 1, "stem": "g1_r18_edge", "kind": "positive", "gt_target_x": "17.9",
             "gt_target_y": "8.0", "gt_target_z": "0.5", "range_m": "18.0", "bearing_deg": "25.0",
             "height_m": "0.5", "yaw_deg": "90.0", "held_out_range": 1},
            {"seq": 2, "stem": "g2_r3_center", "kind": "positive", "gt_target_x": "3.0",
             "gt_target_y": "0.0", "gt_target_z": "0.5", "range_m": "3.0", "bearing_deg": "0.0",
             "height_m": "0.5", "yaw_deg": "0.0", "held_out_range": 1},
            {"seq": 3, "stem": "g3_r6_not_heldout", "kind": "positive", "gt_target_x": "6.0",
             "gt_target_y": "0.0", "gt_target_z": "0.5", "range_m": "6.0", "bearing_deg": "0.0",
             "height_m": "0.5", "yaw_deg": "0.0", "held_out_range": 0},
            {"seq": 4, "stem": "neg_00000_empty", "kind": "negative", "gt_target_x": "-60.0",
             "gt_target_y": "0.0", "gt_target_z": "0.5", "range_m": "", "bearing_deg": "",
             "height_m": "", "yaw_deg": "", "held_out_range": 0},
        ]
        # boresight gt box for g0/g2 (near frame center), a right-of-frame
        # gt box for g1 (still >=4px, tests the self-mask edge-margin path).
        label_boxes = {
            "g0_r18_center": (0.500000, 0.500000, 0.020000, 0.015000),   # ~25.6x14.4 px, center
            "g1_r18_edge": (0.960000, 0.500000, 0.020000, 0.015000),     # cx~1228 px, near R edge
            "g2_r3_center": (0.500000, 0.500000, 0.150000, 0.120000),    # big close-range box
            "g3_r6_not_heldout": (0.500000, 0.500000, 0.070000, 0.055000),
        }
        for row in grid_rows:
            cv2.imwrite(os.path.join(gimg, row["stem"] + ".png"), blank)
            lbl_path = os.path.join(glbl, row["stem"] + ".txt")
            if row["stem"] in label_boxes:
                cx, cy, w, h = label_boxes[row["stem"]]
                with open(lbl_path, "w") as fh:
                    fh.write(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
            else:
                open(lbl_path, "w").close()
        with open(os.path.join(gd, "index.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(grid_rows[0].keys()))
            w.writeheader()
            w.writerows(grid_rows)

        idx = load_grid_index(gd)
        check("load_grid_index: reads all 5 rows with the real index.csv schema",
              len(idx) == 5 and set(idx[0].keys()) == set(grid_rows[0].keys()), f"got {len(idx)} rows")

        def grid_detect_good(img_path: str) -> List[dict]:
            # fires accurately at both r=18 boxes (one center, one near the
            # right edge) and at r=3 -- but NOT at the non-held-out r=6 frame
            # (that frame must never be scored/visited in a way that matters).
            if "g0_r18_center" in img_path:
                return [_mk_det(640.0, 480.0, 25.6, 14.4, 0.5)]
            if "g1_r18_edge" in img_path:
                # this box sits at u=1228 (within 40px of the 1280 right
                # edge) -- apply_self_mask DROPS it, so post-mask recall
                # misses it while raw (mask-off) recall still catches it.
                return [_mk_det(1228.0, 480.0, 25.6, 14.4, 0.5)]
            if "g2_r3_center" in img_path:
                return [_mk_det(640.0, 480.0, 192.0, 115.2, 0.9)]
            return []  # misses everything else, including the non-held-out r=6 frame

        grid_res = grid_recall_probe(gd, grid_detect_good)
        check("grid_recall_probe: only the 2 held-out ranges (3.0, 18.0) appear",
              set(grid_res.keys()) == {3.0, 18.0}, f"got {sorted(grid_res.keys())}")
        check("grid_recall_probe: r=18.0 has n=2 (g0 + g1; the non-held-out r=6 and the "
              "negative are excluded)", grid_res[18.0]["n"] == 2, f"got {grid_res[18.0]}")
        check("grid_recall_probe: r=18.0 RAW recall is 1.0 (both g0 and g1 detected, mask off)",
              abs(grid_res[18.0]["raw_mask_off_recall"] - 1.0) < 1e-9, f"got {grid_res[18.0]}")
        check("grid_recall_probe: r=18.0 POST-MASK recall is 0.5 (g1's edge box is self-masked "
              "away -- post-mask and raw genuinely differ, as the coordinator flagged)",
              abs(grid_res[18.0]["postmask_recall"] - 0.5) < 1e-9, f"got {grid_res[18.0]}")
        check("grid_recall_probe: r=3.0 has n=1, recall 1.0 (g2 detected)",
              grid_res[3.0]["n"] == 1 and abs(grid_res[3.0]["postmask_recall"] - 1.0) < 1e-9,
              f"got {grid_res[3.0]}")

        def grid_detect_blind(img_path: str) -> List[dict]:
            return []  # a "v2-like" model that never fires on the grid

        grid_res_blind = grid_recall_probe(gd, grid_detect_blind)
        check("grid_recall_probe: a detector that never fires -> recall 0.0 at both held-out ranges",
              abs(grid_res_blind[18.0]["postmask_recall"] - 0.0) < 1e-9
              and abs(grid_res_blind[3.0]["postmask_recall"] - 0.0) < 1e-9, f"got {grid_res_blind}")

    print(f"\n[self-test] {'ALL PASS' if ok else 'SOME FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
