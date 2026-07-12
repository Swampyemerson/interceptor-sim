#!/usr/bin/env python3
"""Phantom negative-mining for the quad seeker's r2l false-positive
(ADR-0076 addenda #5/#6, ADR-0077 addendum).

WHY: flying the quad r2l, `drone_finetuned_quad_v2.onnx` locks a big FALSE
box off-target during acquisition (reads `meas_range~1.5 m` while the true
range is 15-20 m) -- guidance chases it, 7 m miss, r2l Pk 0/16 (l2r is fine
at 81%, ADR-0076 add #5). The range-plausibility gate (ADR-0077) failed
because the box-width range is too noisy to gate on and a phantom seeds the
alpha-beta filter wrong. The robust fix is the classic hard-negative-mining
lever (ADR-0038): show the trained NN real onboard frames where it
false-fires, WITHOUT a box drawn on the false region, so it learns "nothing
there is a drone." This script is that miner, reusing the SAME "deployed
operating point" decode (conf 0.25 + self-mask) and the SAME phantom-vs-real
geometry (IoU + center-distance vs the ground-truth box) that
build_onboard_dataset_v3.py already established for the v3/billboard arc --
it is the identical medicine, retargeted at the quad's r2l aspect.

INPUT FORMAT (matches capture_flight_frames.py's OUTPUT LAYOUT exactly --
do not invent a new manifest): one or more --flight-dir, each a flat
`images/*.png` + `labels/*.txt` pair in the YOLO convention (an empty/missing-
content label file = negative/background, a one-line label = the
ground-truth target box). This is collected with
build_onboard_dataset_v3.collect_flat_dir (imported, not reimplemented) so
format compliance with the existing v3 pipeline is guaranteed by
construction. capture_flight_frames.py's filenames encode
`{run_tag}_{seq:06d}_r{gt_range:05.1f}_{pos|neg}`, but this script does not
require that convention -- collect_flat_dir only needs images/<stem>.<ext>
+ labels/<stem>.txt to line up.

THE PHANTOM DECISION (per-detection, then per-frame): run the quad ONNX at
the DEPLOYED op point (v3_onnx_infer.infer_raw at conf=DEPLOY_CONF=0.25 +
apply_self_mask -- byte-identical to finetuned_seeker.FinetunedNNSeeker's
live decode, so mining scores the exact stream that feeds guidance) on every
frame. A single detection is a PHANTOM if:
  * the frame's ground-truth box is None (target not in frame at all --
    ANY detection here is spurious by definition), OR
  * the frame HAS a ground-truth box, but this detection's IoU with it is
    <= --iou-thresh (default 0.1, matching build_onboard_dataset_v3's
    existing mined_pos convention) AND its center is >= --center-dist-px
    (default 100 px, per the ADR-0076 phantom being tens/hundreds of px off
    a compact real target) away from the gt center -- far enough that it
    cannot be a slightly-off box on the real target, only a different,
    spurious region.
A FRAME is mined as a hard negative if it has >=1 phantom detection by that
rule. The frame's EXISTING label file is copied VERBATIM (never edited,
never a box added for the phantom) -- an empty file stays the negative
label it already was; a frame that ALSO contains the real target keeps its
true gt-only box as the positive label. Either way, the phantom's pixel
region is UNLABELED in the copy, which is exactly the "teach it down" signal
(YOLO: unlabeled regions are implicit background). A box that overlaps the
gt (a real, if slightly noisy, detection) is left alone -- it is not mined,
the frame is simply skipped by this script (it is not new hard-negative
signal; merge_quad_hardened.py picks up the ordinary quad_dataset_v2
positives separately).

HONESTY (CLAUDE.md / ADR-0010): gt_* is read here ONLY to score/curate
already-captured frames offline (decide phantom vs real for LABELING
purposes) -- this script never talks to gz/mavsdk, never runs a sim, and
never feeds gt_* to a live detector. The trained model itself sees pixels
only, exactly the boundary v3_onnx_infer.py and build_onboard_dataset_v3.py
already document.

OUTPUT: --out/images/<stem>.<ext> + --out/labels/<stem>.txt (flat, unique-
stem-on-collision across --flight-dir sources, same pattern as
build_onboard_dataset_v3.write_dataset / merge_quad_v2._copy_split), plus
--out/mined_manifest.csv (stem, flight_dir, positive, n_dets, n_phantom) so
provenance is auditable. No train/val split here -- that is
merge_quad_hardened.py's job (hard negatives go MOSTLY to train there).

RUN (offline, needs onnxruntime + cv2 -- .venv-seeker; NOT sim, no gz/mavsdk):
    .venv-seeker/bin/python scripts/seeker/phantom_mine_quad.py \\
        --flight-dir scripts/seeker/data/quad_r2l_flight_capture \\
        --weights scripts/seeker/weights/drone_finetuned_quad_v2.onnx \\
        --out scripts/seeker/data/quad_hard_negatives

OFFLINE SELF-CHECK (no sim, no real weights -- synthesizes tiny fixture
frames + labels and injects a fake detector function in place of a real
ONNX; also exercises the phantom-decision helper directly on plain dicts):
    .venv-seeker/bin/python scripts/seeker/phantom_mine_quad.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import os
import shutil
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from build_onboard_dataset_v3 import collect_flat_dir, make_detector  # noqa: E402
from v3_onnx_infer import center_err, det_box, iou, load_gt_box  # noqa: E402

DEFAULT_WEIGHTS = os.path.join(HERE, "weights", "drone_finetuned_quad_v2.onnx")
DEFAULT_OUT = os.path.join(HERE, "data", "quad_hard_negatives")

# Env-tunable defaults (CLAUDE.md convention: knobs are bench/env tunable,
# not hardcoded); --iou-thresh / --center-dist-px override these directly.
ENV_IOU_THRESH = float(os.environ.get("PHANTOM_MINE_QUAD_IOU_THRESH", "0.1"))
ENV_CENTER_PX = float(os.environ.get("PHANTOM_MINE_QUAD_CENTER_PX", "100.0"))

IMG_EXTS = (".png", ".jpg", ".jpeg")


# ===========================================================================
# The phantom-vs-real decision (pure, injectable, unit-testable without a
# real ONNX or a real frame -- operates on plain (cx,cy,w,h) tuples/dicts).
# ===========================================================================
def is_phantom_det(det: dict, gt_box, iou_thresh: float, center_dist_px: float) -> bool:
    """One detection is a phantom if either:
      * gt_box is None (no real target anywhere in this frame) -- any
        detection at all is spurious, OR
      * gt_box exists but this det's IoU with it is <= iou_thresh AND its
        center is >= center_dist_px from the gt center (far enough that it
        cannot be a slightly-off box on the real target).
    A box that overlaps the gt, or is close to it, is NOT a phantom (kept,
    not mined) -- this is the "a box on gt -> kept as positive" half of the
    decision the task requires."""
    if gt_box is None:
        return True
    db = det_box(det)
    return iou(db, gt_box) <= iou_thresh and center_err(db, gt_box) >= center_dist_px


def frame_phantom_dets(dets, gt_box, iou_thresh: float, center_dist_px: float):
    """All phantom detections in one frame's detection list (order preserved)."""
    return [d for d in dets if is_phantom_det(d, gt_box, iou_thresh, center_dist_px)]


# ===========================================================================
# Mining: score every collected sample with an injectable detect_fn
# ===========================================================================
def mine_samples(samples, detect_fn, iou_thresh: float, center_dist_px: float):
    """Score every sample (as returned by collect_flat_dir: {image, label,
    positive}) with detect_fn(frame_bgr) -> list[{cx,cy,w,h,score}]. Returns
    (mined, report) where `mined` is the subset of samples with >=1 phantom
    detection (each augmented with n_dets/n_phantom for the manifest), and
    `report` is summary counts. Frames whose image fails to read are skipped
    (reported, not silently dropped) -- same policy as
    build_onboard_dataset_v3.mine_dataset."""
    mined = []
    n_scored = 0
    n_unreadable = 0
    n_mined_from_negative = 0
    n_mined_from_positive = 0
    for s in samples:
        frame = cv2.imread(s["image"])
        if frame is None:
            n_unreadable += 1
            print(f"[phantom-mine] WARNING: could not read image (skipped): {s['image']}")
            continue
        dets = detect_fn(frame)
        n_scored += 1
        gt = load_gt_box(s["label"])
        phantom_dets = frame_phantom_dets(dets, gt, iou_thresh, center_dist_px)
        if not phantom_dets:
            continue
        rec = dict(s)
        rec["n_dets"] = len(dets)
        rec["n_phantom"] = len(phantom_dets)
        mined.append(rec)
        if gt is None:
            n_mined_from_negative += 1
        else:
            n_mined_from_positive += 1
    report = {
        "n_scored": n_scored, "n_unreadable": n_unreadable,
        "n_mined": len(mined),
        "n_mined_from_negative": n_mined_from_negative,
        "n_mined_from_positive": n_mined_from_positive,
    }
    return mined, report


# ===========================================================================
# Write: copy mined frames' image + EXISTING label file verbatim
# ===========================================================================
def write_hard_negatives(mined, out_dir: str, flight_dir_tags=None):
    """Copy each mined sample's image + its EXISTING label file (never
    rewritten -- empty stays empty, a real gt box stays exactly that box)
    into out_dir/images, out_dir/labels (flat). Unique-stem-on-collision
    across sources, same pattern as build_onboard_dataset_v3.write_dataset.
    Writes out_dir/mined_manifest.csv for provenance. Returns
    (n_written, manifest_path)."""
    img_dir = os.path.join(out_dir, "images")
    lbl_dir = os.path.join(out_dir, "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)

    used_stems = set()
    rows = []
    for rec in mined:
        base = os.path.basename(rec["image"])
        stem, ext = os.path.splitext(base)
        dest_stem = stem
        n = 0
        while dest_stem in used_stems:
            n += 1
            dest_stem = f"{stem}_dup{n}"
        used_stems.add(dest_stem)

        shutil.copy2(rec["image"], os.path.join(img_dir, dest_stem + ext))
        shutil.copy2(rec["label"], os.path.join(lbl_dir, dest_stem + ".txt"))

        rows.append({
            "stem": dest_stem,
            "flight_dir": rec.get("flight_dir", ""),
            "positive": int(rec["positive"]),
            "n_dets": rec["n_dets"],
            "n_phantom": rec["n_phantom"],
        })

    manifest_path = os.path.join(out_dir, "mined_manifest.csv")
    with open(manifest_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["stem", "flight_dir", "positive",
                                           "n_dets", "n_phantom"])
        w.writeheader()
        w.writerows(rows)

    return len(rows), manifest_path


# ===========================================================================
# main
# ===========================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flight-dir", action="append", default=None,
                    help="repeatable; a capture_flight_frames.py-layout dir "
                         "(flat images/+labels/). Required unless --self-test.")
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS,
                    help="the quad seeker ONNX to mine against "
                         "(default %(default)s)")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--iou-thresh", type=float, default=ENV_IOU_THRESH,
                    help="a det with IoU <= this vs gt is IoU-wise a "
                         "candidate phantom (env PHANTOM_MINE_QUAD_IOU_THRESH, "
                         "default %(default)s)")
    ap.add_argument("--center-dist-px", type=float, default=ENV_CENTER_PX,
                    help="a det whose center is >= this many px from the gt "
                         "center is a candidate phantom (env "
                         "PHANTOM_MINE_QUAD_CENTER_PX, default %(default)s)")
    ap.add_argument("--self-test", action="store_true",
                    help="offline self-check: fake detector + synthesized "
                         "fixtures, no sim, no real weights")
    args = ap.parse_args()

    if args.self_test:
        return run_self_test()

    if not args.flight_dir:
        print("[phantom-mine] FAILED: --flight-dir is required (repeatable); "
              "see --help for the expected capture_flight_frames.py layout")
        return 2

    all_samples = []
    for d in args.flight_dir:
        img_dir = os.path.join(d, "images")
        lbl_dir = os.path.join(d, "labels")
        samples, n_missing = collect_flat_dir(img_dir, lbl_dir)
        for s in samples:
            s["flight_dir"] = os.path.basename(os.path.normpath(d))
        all_samples += samples
        pos = sum(1 for s in samples if s["positive"])
        print(f"[phantom-mine] {d}: {len(samples)} frames "
              f"(pos={pos} neg={len(samples) - pos}); missing_label={n_missing}")

    if not all_samples:
        print("[phantom-mine] FAILED: no frames collected from --flight-dir "
              "(check the images/+labels/ layout)")
        return 2

    if not os.path.isfile(args.weights):
        print(f"[phantom-mine] FAILED: --weights not found: {args.weights}")
        return 2

    detect_fn = make_detector(args.weights)
    mined, report = mine_samples(all_samples, detect_fn, args.iou_thresh,
                                 args.center_dist_px)
    print(f"[phantom-mine] scored {report['n_scored']} frames "
          f"(unreadable={report['n_unreadable']}) at the deployed op point "
          f"(conf=0.25 + self-mask), iou_thresh={args.iou_thresh} "
          f"center_dist_px={args.center_dist_px}")
    print(f"[phantom-mine] MINED {report['n_mined']} phantom frames "
          f"({report['n_mined_from_negative']} from already-negative frames, "
          f"{report['n_mined_from_positive']} from positive frames where the "
          "NN ALSO fired a phantom alongside/instead of the real target)")

    if not mined:
        print("[phantom-mine] nothing to write (0 phantom frames mined)")
        return 0

    n_written, manifest_path = write_hard_negatives(mined, args.out)
    print(f"[phantom-mine] wrote {n_written} hard-negative frames -> "
          f"{args.out}/images, {args.out}/labels")
    print(f"[phantom-mine] manifest: {manifest_path}")
    return 0


# ===========================================================================
# Offline self-test: unit checks on the decision helper + an end-to-end
# fake-detector pipeline run (no sim, no real ONNX)
# ===========================================================================
def _check(name, cond, detail=""):
    cond = bool(cond)
    suffix = f" ({detail})" if detail else ""
    print(f"[self-test] {name}: {'PASS' if cond else 'FAIL'}{suffix}")
    return cond


# Same denormalize-target trick as build_onboard_dataset_v3's self-test: the
# gt label is normalized against the fixed 1280x960 frame v3_onnx_infer's
# load_gt_box() denormalizes against, so it lands at an exact known pixel
# location independent of the tiny synthetic PNG's real size.
_IMG_W, _IMG_H = 1280, 960
_GT_BOX_NORM = (50.0 / _IMG_W, 50.0 / _IMG_H, 20.0 / _IMG_W, 20.0 / _IMG_H)
_ON_GT_DET = {"cx": 50.0, "cy": 50.0, "w": 20.0, "h": 20.0, "score": 0.9}
_PHANTOM_DET = {"cx": 1200.0, "cy": 900.0, "w": 10.0, "h": 10.0, "score": 0.5}


def _fake_detect_fn(frame_bgr):
    """Injected in place of a real quad ONNX: which boxes fire is keyed off
    the fixture's mean pixel value (same trick as
    build_onboard_dataset_v3._fake_detect_fn) so 4 distinct fixture
    brightness levels drive 4 distinct detector behaviors without needing
    onnxruntime or a real frame."""
    m = float(frame_bgr.mean())
    if m > 220:
        return [dict(_ON_GT_DET)]                       # real match only
    if m > 150:
        return [dict(_ON_GT_DET), dict(_PHANTOM_DET)]    # real + phantom
    if m > 80:
        return [dict(_PHANTOM_DET)]                      # phantom only
    return []                                            # nothing fires


def _write_fixture(img_path, lbl_path, fill_value, box_norm):
    cv2.imwrite(img_path, np.full((32, 32, 3), fill_value, dtype=np.uint8))
    with open(lbl_path, "w") as fh:
        if box_norm is not None:
            cx, cy, w, h = box_norm
            fh.write(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
        # else: empty file IS the negative label.


def run_self_test() -> int:
    all_ok = True

    # --- 1. pure decision-helper unit checks (plain dicts, no filesystem) ---
    gt = (50.0, 50.0, 20.0, 20.0)
    all_ok &= _check("is_phantom_det: box ON gt is NOT a phantom (kept)",
                     not is_phantom_det(_ON_GT_DET, gt, 0.1, 100.0))
    all_ok &= _check("is_phantom_det: box FAR from gt IS a phantom",
                     is_phantom_det(_PHANTOM_DET, gt, 0.1, 100.0))
    all_ok &= _check("is_phantom_det: ANY box on a gt-less (negative) frame "
                     "IS a phantom",
                     is_phantom_det(_ON_GT_DET, None, 0.1, 100.0)
                     and is_phantom_det(_PHANTOM_DET, None, 0.1, 100.0))
    near_miss = {"cx": 55.0, "cy": 52.0, "w": 20.0, "h": 20.0, "score": 0.8}
    all_ok &= _check("is_phantom_det: a slightly-off box on the real target "
                     "(high IoU) is NOT a phantom",
                     not is_phantom_det(near_miss, gt, 0.1, 100.0),
                     f"iou={iou(det_box(near_miss), gt):.3f}")
    far_but_small_iou_only = {"cx": 90.0, "cy": 50.0, "w": 6.0, "h": 6.0, "score": 0.6}
    # center dist ~40px < 100px threshold -> NOT far enough to be a phantom
    # even though IoU is already 0 -- both conditions must hold.
    all_ok &= _check("is_phantom_det: IoU~0 but center NOT far enough -> "
                     "NOT a phantom (both conditions required)",
                     not is_phantom_det(far_but_small_iou_only, gt, 0.1, 100.0),
                     f"center_err={center_err(det_box(far_but_small_iou_only), gt):.1f}px")
    frame_dets = [_ON_GT_DET, _PHANTOM_DET]
    phantoms = frame_phantom_dets(frame_dets, gt, 0.1, 100.0)
    all_ok &= _check("frame_phantom_dets: exactly the phantom det survives "
                     "the filter, the real match does not",
                     len(phantoms) == 1 and phantoms[0] is _PHANTOM_DET)

    # --- 2. end-to-end pipeline: fake detector over synthesized fixtures ---
    with tempfile.TemporaryDirectory(prefix="phantom_mine_quad_selftest_") as tmp:
        flight_dir = os.path.join(tmp, "flight")
        img_dir = os.path.join(flight_dir, "images")
        lbl_dir = os.path.join(flight_dir, "labels")
        os.makedirs(img_dir)
        os.makedirs(lbl_dir)

        # (stem, fill, box_norm_or_None, expect_mined)
        fixtures = [
            ("flt01_000001_r010.0_pos", 250, _GT_BOX_NORM, False),  # A: clean match
            ("flt01_000002_r010.0_pos", 180, _GT_BOX_NORM, True),   # B: match+phantom
            ("flt01_000003_r010.0_pos", 110, _GT_BOX_NORM, True),   # C: phantom only, real target present but missed
            ("flt01_000004_r010.0_neg", 110, None, True),           # C: true negative, phantom fires
            ("flt01_000005_r010.0_neg", 10, None, False),           # D: true negative, nothing fires
        ]
        for stem, fill, box, _ in fixtures:
            _write_fixture(os.path.join(img_dir, stem + ".png"),
                           os.path.join(lbl_dir, stem + ".txt"), fill, box)

        samples, n_missing = collect_flat_dir(img_dir, lbl_dir)
        for s in samples:
            s["flight_dir"] = "flight"
        all_ok &= _check("collect_flat_dir: no missing-label errors", n_missing == 0)
        all_ok &= _check("collect_flat_dir: 5 samples collected", len(samples) == 5,
                         f"{len(samples)}")

        mined, report = mine_samples(samples, _fake_detect_fn, iou_thresh=0.1,
                                     center_dist_px=100.0)
        mined_stems = {os.path.splitext(os.path.basename(r["image"]))[0] for r in mined}
        expect_mined = {stem for stem, _, _, exp in fixtures if exp}
        all_ok &= _check("mine_samples: mined EXACTLY the 3 phantom-triggering "
                         "frames (B match+phantom, C phantom-only pos, C neg)",
                         mined_stems == expect_mined,
                         f"{sorted(mined_stems)}")
        all_ok &= _check("mine_samples: report counts match "
                         "(1 mined-from-negative, 2 mined-from-positive)",
                         report["n_mined"] == 3
                         and report["n_mined_from_negative"] == 1
                         and report["n_mined_from_positive"] == 2,
                         f"{report}")

        out_dir = os.path.join(tmp, "out")
        n_written, manifest_path = write_hard_negatives(mined, out_dir)
        all_ok &= _check("write_hard_negatives: wrote exactly 3 frames",
                         n_written == 3, f"{n_written}")
        all_ok &= _check("write_hard_negatives: manifest.csv exists",
                         os.path.isfile(manifest_path))

        # The clean-match frame (A) and the quiet-negative frame (D) must NOT
        # be copied at all -- they were never mined.
        out_imgs = set(os.listdir(os.path.join(out_dir, "images")))
        all_ok &= _check("write_hard_negatives: clean-match frame (A) "
                         "excluded from the output",
                         "flt01_000001_r010.0_pos.png" not in out_imgs)
        all_ok &= _check("write_hard_negatives: quiet-negative frame (D) "
                         "excluded from the output",
                         "flt01_000005_r010.0_neg.png" not in out_imgs)

        # The mined true-negative frame's label must stay EMPTY.
        neg_lbl = os.path.join(out_dir, "labels", "flt01_000004_r010.0_neg.txt")
        with open(neg_lbl) as fh:
            neg_content = fh.read().strip()
        all_ok &= _check("write_hard_negatives: mined true-negative frame's "
                         "label stays EMPTY (background supervision)",
                         neg_content == "", f"{neg_content!r}")

        # The mined positive-with-phantom frames must keep the REAL gt box
        # as their label -- NOT empty, and NOT a box drawn on the phantom.
        for stem in ("flt01_000002_r010.0_pos", "flt01_000003_r010.0_pos"):
            lbl_path = os.path.join(out_dir, "labels", stem + ".txt")
            gt_box = load_gt_box(lbl_path)
            all_ok &= _check(
                f"write_hard_negatives: {stem} keeps the TRUE target's gt "
                "box as its label (phantom suppressed by omission, not by "
                "overwriting the real label)",
                gt_box is not None and abs(gt_box[0] - 50.0) < 1e-3
                and abs(gt_box[1] - 50.0) < 1e-3,
                f"{gt_box}")

        with open(manifest_path, newline="") as fh:
            manifest_rows = list(csv.DictReader(fh))
        all_ok &= _check("write_hard_negatives: manifest row count matches "
                         "written count", len(manifest_rows) == 3,
                         f"{len(manifest_rows)}")

    print(f"[self-test] {'ALL PASS' if all_ok else 'SOME FAILED'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
