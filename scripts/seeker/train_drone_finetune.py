"""Drone-detector fine-tune PIPELINE skeleton (DEFERRED training, ADR-0033 #9).

WHY THIS FILE EXISTS
--------------------
drone_detector_eval.py measured the honest result: an off-the-shelf, real-photo-
trained MIT drone detector (YOLOv11x, mAP50 0.905 on ITS domain) does NOT beat the
untuned-COCO baseline's acquisition range on OUR target — it fires only in a ~2-frame
terminal window (~1.4-1.8 m), no earlier than COCO, because the Gazebo target (a
synthetic primitive body carrying an AprilTag) is far out-of-distribution vs the
real drones the model learned. The model card itself warns "domain adaptation is
strongly recommended." So the fix is not a bigger off-the-shelf model — it is a
fine-tune on IN-DOMAIN (sim) imagery. This script is that pipeline, ready to run
when the machine is idle. **It does NOT train by default** (a Monte-Carlo batch is
running; heavy training is deferred per the task constraint) — call it with
`--go` to actually train; the default `--dry-run` only validates the plan.

THE DATA STRATEGY (two stages, both license-clean)
--------------------------------------------------
Stage 1 (optional general prior): pre-train a NANO base on a permissive real-drone
  dataset so it has a "small aerial object" prior:
    * Halmstad Drone dataset (Svanstrom et al.) — research-use, IR+visible.
    * ZhaoJ9014/Anti-UAV — MIT, 10k detection images, 35+ UAV models.
    * Drone-vs-Bird / WOSDETC — challenge data (check per-year terms).
  AVOID MMAUD for anything but a portfolio: it is CC-BY-NC-SA (non-commercial).
Stage 2 (the one that actually closes the domain gap): fine-tune on SIM imagery of
  the tag-less target. Generate it by flying models/fpv_target_markerless (brief
  sec 4; not built yet) and AUTO-LABELLING each frame's target box from the sim
  `gt_` pose projected through the camera intrinsics. NOTE the honesty boundary
  (CLAUDE.md / ADR-0010): using `gt_` to MANUFACTURE TRAINING LABELS offline is
  legal (it is dataset generation, not a runtime guidance input); the trained
  detector still reads ONLY pixels at inference. Keep the label-gen script in the
  scoring/offline path and never import gt_ into the seeker.

MODEL / LICENSE CHOICE (R6)
---------------------------
Base defaults to a nano YOLO for the recipe's sake, but YOLOv8n/YOLO11n are
AGPL-3.0 — publishing weights derived from them can obligate open-sourcing the
whole stack. For a clean public portfolio prefer **NanoDet-Plus (Apache-2.0)** or
**MobileNet-SSD (Apache)**; those need their own trainers (not ultralytics). This
skeleton is written against ultralytics because it is the fastest path and matches
the doguilmak recipe we already used; if the fine-tuned weights are to be
published, either accept AGPL knowingly or port the recipe to NanoDet-Plus. See
weights/LICENSES.md.

USAGE (deferred; run only when the batch is done and the machine is idle)
-------------------------------------------------------------------------
  # validate the plan without training (safe now):
  .venv-seeker-train/bin/python scripts/seeker/train_drone_finetune.py --dry-run
  # actually train (idle machine only):
  OMP_NUM_THREADS=8 .venv-seeker-train/bin/python \
    scripts/seeker/train_drone_finetune.py --go \
    --data scripts/seeker/data/drone_sim.yaml --base yolo11n.pt \
    --imgsz 1280 --epochs 80
  # then export to ONNX for onnxruntime (same path drone_detector_eval.py loads):
  #   handled automatically after --go, or re-run with --export-only WEIGHTS.pt
"""

from __future__ import annotations

import argparse
import os
import sys
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DATA = os.path.join(HERE, "data", "drone_sim.yaml")
WEIGHTS_DIR = os.path.join(HERE, "weights")

DATA_YAML_TEMPLATE = """\
# Single-class drone detector — fine-tune data config (ultralytics format).
# Fill in the paths once the sim/real frames + YOLO-format labels exist.
# Labels: one .txt per image, lines "0 xc yc w h" (class 0 = drone, normalized).
path: {root}          # dataset root
train: images/train   # relative to path
val:   images/val
names:
  0: drone
"""


def write_data_template(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not os.path.exists(path):
        with open(path, "w") as f:
            f.write(DATA_YAML_TEMPLATE.format(root=os.path.dirname(path)))
        print(f"[skeleton] wrote data-config template -> {path}")
    else:
        print(f"[skeleton] data config already present -> {path}")


def dry_run(args):
    print("=" * 72)
    print("FINE-TUNE PIPELINE — DRY RUN (no training performed)")
    print("=" * 72)
    have_data = os.path.exists(args.data)
    print(textwrap.dedent(f"""
      base model      : {args.base}   (nano; swap to NanoDet-Plus for Apache)
      input size      : {args.imgsz}px (LARGE on purpose — target is few-px)
      epochs          : {args.epochs}
      data config     : {args.data}  [{'FOUND' if have_data else 'MISSING -> template written'}]
      output weights  : {WEIGHTS_DIR}/drone_finetuned.pt (+ .onnx after export)

      PLAN
        1. (optional) Stage-1 prior: train on a permissive real-drone set
           (Anti-UAV MIT / Halmstad). Skip to go straight to sim domain.
        2. Stage-2 in-domain: fine-tune on auto-labelled sim frames of
           models/fpv_target_markerless (gt_-projected boxes; label-gen only).
        3. Export best.pt -> ONNX @ {args.imgsz} for onnxruntime.
        4. Re-run drone_detector_eval.py: does the IN-DOMAIN fine-tune finally
           acquire the body EARLIER than COCO (beat the ~1.5 m terminal ceiling)?

      DEFERRED: a Monte-Carlo batch is running; do not train now. Re-invoke with
      --go on an idle machine (CLAUDE.md batch-hygiene / worse-than-ideal rules).
    """))
    if not have_data:
        write_data_template(args.data)
    print("[dry-run] OK — config valid, nothing trained.")


def go(args):
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ultralytics not found — use the .venv-seeker-train venv.")
    if not os.path.exists(args.data):
        write_data_template(args.data)
        sys.exit(f"Fill in {args.data} (image/label paths) before --go.")
    print(f"[train] {args.base} @ {args.imgsz}px x {args.epochs} epochs on {args.data}")
    model = YOLO(args.base)
    model.train(data=args.data, imgsz=args.imgsz, epochs=args.epochs,
                batch=args.batch, project=os.path.join(HERE, "runs"),
                name="drone_finetune", exist_ok=True)
    best = os.path.join(HERE, "runs", "drone_finetune", "weights", "best.pt")
    dst = os.path.join(WEIGHTS_DIR, "drone_finetuned.pt")
    if os.path.exists(best):
        import shutil; shutil.copy(best, dst)
        YOLO(dst).export(format="onnx", imgsz=args.imgsz, opset=12, simplify=True)
        print(f"[train] done -> {dst} (+ .onnx). Now re-run drone_detector_eval.py.")


def export_only(pt, imgsz):
    from ultralytics import YOLO
    p = YOLO(pt).export(format="onnx", imgsz=imgsz, opset=12, simplify=True)
    print(f"[export] {p}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=DEFAULT_DATA)
    ap.add_argument("--base", default="yolo11n.pt",
                    help="nano base (AGPL: yolo11n/yolov8n; prefer NanoDet-Plus for Apache)")
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--batch", type=int, default=16)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--dry-run", action="store_true", default=True,
                   help="validate the plan, train nothing (DEFAULT — safe while batch runs)")
    g.add_argument("--go", action="store_true", help="actually train (idle machine only)")
    ap.add_argument("--export-only", metavar="WEIGHTS.pt",
                    help="just export an existing .pt to ONNX and exit")
    args = ap.parse_args()

    if args.export_only:
        export_only(args.export_only, args.imgsz)
    elif args.go:
        go(args)
    else:
        dry_run(args)


if __name__ == "__main__":
    main()
