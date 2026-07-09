#!/usr/bin/env python3
r"""T19b part-1 -- offline detection-cache builder (ADR-0051, docs/decisions.md).

READ FIRST: docs/decisions.md ADR-0051 (the decision this file implements) +
ADR-0046 (why the rig frames are pre-rendered/deterministic in the first
place) + ADR-0048 SS3 (the dual-gate `station.py`'s `--centroid-cache` mode
feeds into).

=======================================================================
WHAT THIS IS AND WHY (ADR-0051's "the key realization")
=======================================================================
`scripts/ground_station/station.py`'s LIVE path runs the real
`GroundDetector` (ADR-0047/ADR-0049's `ground_v1.onnx`) on both camera
frames of every captured pose, DURING the flight. That is genuine wall-clock
CPU compute (~0.85-1.25 s/frame-pair on this checkout's CPU-only
onnxruntime, ADR-0051's T19a evidence) -- and because the T16/T18 capture
frames are PRE-RENDERED and DETERMINISTIC (ADR-0046), running the SAME
detector on the SAME frame ahead of time produces the IDENTICAL detection
every time. There is nothing to be gained by paying that compute cost again
at flight time, and doing so only pollutes the flight's sim-time discipline
with uncontrolled live jitter (exactly what ADR-0048 SS3's latency FLOOR was
designed to model instead of inherit).

So this script is the PRECOMPUTE half of ADR-0051 option (b): run
`GroundDetector` ONCE, offline, over an entire capture directory's left/right
frames, and cache the resulting box-center centroids to `centroids.csv`.
`station.py --centroid-cache PATH` (this task's other half) then replays
those cached centroids through the still-LIVE triangulate -> track -> emit
pipeline, so the dual gate's release timing is governed by the MODELED
latency floor instead of by unmodeled detector compute -- see that module's
own "CENTROID-CACHE REPLAY MODE" docstring section.

DETERMINISTIC: same frames + same weights/conf/imgsz -> same `centroids.csv`,
every time (no randomness anywhere in this path -- CPU onnxruntime inference
on a fixed input is deterministic). `tests/test_ground_station.py`'s
determinism test empirically confirms this equals a LIVE
`GroundDetector.detect_path()` call on the same frames (ADR-0051's central
claim -- if that test does not hold, that is a critical finding, not a
detail to gloss over).

REGENERATE this cache whenever EITHER changes: (1) the capture directory
(different frames), or (2) the detector weights/conf/imgsz (different
detections). This script is idempotent and cheap to re-run; it does not
try to detect staleness itself -- `centroid_cache_meta.json` (written
alongside `centroids.csv`) records what produced the cache so a human/CI
check can compare it against the current capture/weights if that is ever
wanted.

HONESTY: reads only image pixels (via `GroundDetector`, deferred-imported
cv2/onnxruntime) + `index.csv`'s `seq`/`t_sim_nominal` columns (frame
identity/timing, not target ground truth) -- `gt_target_*` columns are never
read here. Same boundary as `station.py`'s own LIVE path.

DEPS: onnxruntime + opencv + numpy, i.e. `.venv-seeker` (matches
`scripts/ground_station/detect.py`'s own interpreter note) -- NOT the main
`.venv` (no onnxruntime there by design, so pytest can import this module's
non-detector helpers without pulling either in -- see the deferred-import
pattern below, same discipline as `station.py`).

Run (build the cache for the T16/T18 full-sweep capture, ground_v1 default
weights):
    .venv-seeker/bin/python scripts/ground_station/build_centroid_cache.py \\
        --capture-dir logs/rig_captures/full_sweep_20260709T015530Z

Then replay it (offline smoke, no PX4/Gazebo):
    .venv-seeker/bin/python scripts/ground_station/station.py \\
        --capture-dir logs/rig_captures/full_sweep_20260709T015530Z \\
        --centroid-cache logs/rig_captures/full_sweep_20260709T015530Z/centroids.csv \\
        --dash-direction r2l --replay-clock --seq-limit 8
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/ground_station
SCRIPTS_DIR = os.path.dirname(HERE)                         # scripts
REPO_ROOT = os.path.dirname(SCRIPTS_DIR)                    # repo root
sys.path.insert(0, SCRIPTS_DIR)

# Deferred-import discipline preserved: importing station/detect here does
# NOT force onnxruntime/cv2 (both modules defer those imports themselves, see
# their own docstrings) -- only actually CONSTRUCTING a GroundDetector does.
from ground_station.station import (  # noqa: E402
    CENTROID_CACHE_FIELDNAMES,
    DEFAULT_CAPTURE_DIR,
    frame_paths,
    load_capture,
)
from ground_station.detect import (  # noqa: E402
    DEFAULT_CONF,
    DEFAULT_IMGSZ,
    DEFAULT_WEIGHTS,
    GroundDetector,
    box_center,
)


# ============================================================================
# Per-row detection (the one thing this script and the determinism test both
# call, so "cache build" and "a live detect_path() call" are provably the
# exact same code path with a different detector instance/timing -- not two
# independently-written implementations that merely SHOULD agree).
# ============================================================================
def detect_row(row: dict, capture_dir: str, detector) -> dict:
    """Runs `detector.detect_path()` on ONE `index.csv` row's left+right
    frames, returning a dict shaped exactly like `CENTROID_CACHE_FIELDNAMES`
    (real numeric types, not yet CSV-stringified -- `csv.DictWriter` handles
    that). A miss on either camera writes `det_{l,r}=0` and blank
    `u/v/conf` for that side -- the same shape `station.CachedDetector`
    reads back (a miss on EITHER side is a whole-frame cache miss, mirroring
    `process_frame()`'s existing `not dl.hit or not dr.hit -> "miss"` rule).
    """
    seq = int(row["seq"])
    t_sim = float(row["t_sim_nominal"])
    left_path, right_path = frame_paths(capture_dir, seq)
    dl = detector.detect_path(left_path)
    dr = detector.detect_path(right_path)

    out = {"seq": seq, "t_sim": t_sim}
    if dl.hit:
        u_l, v_l = box_center(dl.box_xyxy)
        out.update({"det_l": 1, "u_l": u_l, "v_l": v_l, "conf_l": dl.conf})
    else:
        out.update({"det_l": 0, "u_l": "", "v_l": "", "conf_l": ""})
    if dr.hit:
        u_r, v_r = box_center(dr.box_xyxy)
        out.update({"det_r": 1, "u_r": u_r, "v_r": v_r, "conf_r": dr.conf})
    else:
        out.update({"det_r": 0, "u_r": "", "v_r": "", "conf_r": ""})
    return out


def build_cache(capture_dir: str, weights: str, conf: float, imgsz: int):
    """Runs `detect_row()` over every row of `capture_dir`'s `index.csv`
    (BOTH dash directions -- `seq` is globally unique across the whole
    capture, per `capture_meta.json`'s `signs_captured`, so caching by seq
    covers whichever direction a later replay selects). Returns
    `(out_rows, stats)`; raises `ImportError` if detector deps are missing
    (caller decides how to report that)."""
    _meta, rows = load_capture(capture_dir)
    detector = GroundDetector(weights=weights, conf=conf, imgsz=imgsz)

    out_rows = []
    n_hit_l = n_hit_r = n_hit_both = 0
    for row in rows:
        r = detect_row(row, capture_dir, detector)
        out_rows.append(r)
        n_hit_l += r["det_l"]
        n_hit_r += r["det_r"]
        n_hit_both += int(bool(r["det_l"]) and bool(r["det_r"]))
    stats = {
        "n_rows": len(rows),
        "n_hit_l": n_hit_l,
        "n_hit_r": n_hit_r,
        "n_hit_both": n_hit_both,
        "n_miss_either": len(rows) - n_hit_both,
    }
    return out_rows, stats


def write_cache_csv(out_rows: list, out_path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CENTROID_CACHE_FIELDNAMES)
        w.writeheader()
        for r in out_rows:
            w.writerow({k: r.get(k, "") for k in CENTROID_CACHE_FIELDNAMES})


# The header/README note (ADR-0051's own "disclose this precompute step"
# instruction) -- written as a JSON sidecar (mirrors the existing
# `capture_meta.json` convention next to `index.csv`) rather than as comment
# lines inside `centroids.csv`, so `centroids.csv` itself stays a plain,
# comment-free CSV that `csv.DictReader` (station.py's `load_centroid_cache`)
# parses with zero special-casing.
def write_meta_json(meta_path: str, capture_dir: str, weights: str, conf: float,
                     imgsz: int, stats: dict) -> None:
    payload = {
        "note": (
            "ADR-0051 (docs/decisions.md): this centroids.csv is the OFFLINE "
            "PRECOMPUTE step ADR-0051 chose over running detection live "
            "during a flight. The capture frames are pre-rendered and "
            "deterministic (ADR-0046), and detection on a fixed frame is "
            "deterministic, so this equals what running the SAME detector "
            "LIVE on the SAME frames would produce (empirically confirmed by "
            "tests/test_ground_station.py's determinism test). REGENERATE "
            "this cache (re-run build_centroid_cache.py) if the capture "
            "directory OR the detector weights/conf/imgsz change -- this "
            "file does not self-detect staleness."
        ),
        "capture_dir": capture_dir,
        "weights": weights,
        "conf": conf,
        "imgsz": imgsz,
        "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        **stats,
    }
    with open(meta_path, "w") as f:
        json.dump(payload, f, indent=2)


# ============================================================================
# CLI
# ============================================================================
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture-dir", default=DEFAULT_CAPTURE_DIR,
                     help="T16/T18 capture directory (capture_meta.json + index.csv + left/right PNGs)")
    ap.add_argument("--out", default=None,
                     help="output centroids.csv path (default: <capture-dir>/centroids.csv)")
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS, help="ground_v1 .onnx weights")
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF, help="detector confidence threshold")
    ap.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ, help="detector inference size")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    out_path = args.out or os.path.join(args.capture_dir, "centroids.csv")
    meta_path = os.path.join(os.path.dirname(os.path.abspath(out_path)), "centroid_cache_meta.json")

    print(f"[build-centroid-cache] capture_dir={args.capture_dir}")
    print(f"[build-centroid-cache] weights={args.weights} conf={args.conf} imgsz={args.imgsz}")
    try:
        out_rows, stats = build_cache(args.capture_dir, args.weights, args.conf, args.imgsz)
    except ImportError as exc:
        print(f"[build-centroid-cache] FAILED: detector deps missing ({exc}). Run under a venv with "
              "onnxruntime + opencv (this checkout: .venv-seeker).")
        return 2

    write_cache_csv(out_rows, out_path)
    write_meta_json(meta_path, args.capture_dir, args.weights, args.conf, args.imgsz, stats)

    print(f"[build-centroid-cache] wrote {out_path} ({len(out_rows)} rows)")
    print(f"[build-centroid-cache] wrote {meta_path}")
    print(f"[build-centroid-cache] stats: {stats}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
