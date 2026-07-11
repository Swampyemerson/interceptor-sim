#!/usr/bin/env python3
"""Detached quad-enemy v2 retrain daemon -- LEVEL+BANKED augmentation.

WHY: copied verbatim from train_daemon_quad.py's pattern -- the ONLY changes
across this retrain are the dataset (quad_dataset_v2, LEVEL union BANKED) and
the output artifact names, so the training recipe/daemon shape is reused
unchanged. `drone_finetuned_quad` (level+yaw only) detected `fpv_quad_enemy`
at 23 m but lost lock during the terminal dash (handoff slipped to 1.91 m, 26
tracker re-acquisitions) once the target banks (up to 55 deg) and pitches
nose-down (up to 25 deg) via `--orient-to-velocity` -- aspects the level-only
training set never showed. quad_dataset_v2 (merge_quad_v2.py) adds those
aspects via GT-auto-labeled banked renders (orientation-invariant bounding-
sphere label, so honesty boundary is unaffected).

Launched setsid-detached (immune to the ~51 min background-task reaper) and
RESUMES from a checkpoint if one exists, falling back to a FRESH run from
yolo11n.pt otherwise. On finish it exports best.pt ->
weights/drone_finetuned_quad_v2.{pt,onnx} and prints the completion sentinel
QUAD_V2_TRAIN_EXPORT_DONE.

Recipe is byte-identical to quad v1 (and v2/v3 billboard retrains): yolo11n.pt
base, imgsz 640, 60 epochs, batch 16, seed 0, ultralytics-default augmentation,
ONNX opset 12 simplify. Single variable vs v1 is the dataset (level+banked
union) + output artifact names.

Does NOT touch drone_finetuned_v2.{pt,onnx} (the deployed billboard seeker),
drone_finetuned_quad.{pt,onnx} (the level-only quad baseline, kept as-is),
models/fpv_target_markerless, or scripts/seeker/data/sim_dataset_v2 -- this is
a new artifact alongside both, not a replacement of either.
"""
import os
import shutil
import sys

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker
REPO = os.path.dirname(os.path.dirname(HERE))              # repo root
os.chdir(REPO)

RUN_DIR = os.path.join("scripts", "seeker", "runs", "drone_finetune_quad_v2")
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
DATA = os.path.join("scripts", "seeker", "data", "quad_dataset_v2", "data.yaml")
BASE = os.path.join(REPO, "yolo11n.pt")
DST_PT = os.path.join("scripts", "seeker", "weights", "drone_finetuned_quad_v2.pt")

TRAIN_KW = dict(data=DATA, imgsz=640, epochs=60, batch=16, seed=0,
                project=os.path.join(HERE, "runs"), name="drone_finetune_quad_v2",
                exist_ok=True)


def _fresh():
    print("[daemon] FRESH train from yolo11n.pt", flush=True)
    shutil.rmtree(RUN_DIR, ignore_errors=True)
    YOLO(BASE).train(**TRAIN_KW)


def main() -> int:
    if os.path.exists(LAST):
        try:
            print(f"[daemon] RESUME from {LAST}", flush=True)
            YOLO(LAST).train(resume=True)
        except Exception as exc:  # resume state unusable -> clean fresh run
            print(f"[daemon] resume failed ({exc!r}); falling back to FRESH", flush=True)
            _fresh()
    else:
        _fresh()

    if not os.path.exists(BEST):
        print(f"[daemon] FAILED: {BEST} missing after training", flush=True)
        return 2
    shutil.copy(BEST, DST_PT)
    YOLO(DST_PT).export(format="onnx", imgsz=640, opset=12, simplify=True)
    print(f"QUAD_V2_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
