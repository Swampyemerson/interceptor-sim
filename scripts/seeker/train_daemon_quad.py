#!/usr/bin/env python3
"""Detached quad-enemy retrain daemon (markerless seeker on the NEW
`fpv_quad_enemy` target model -- a proper 3D quad body that replaced the
flat-billboard `fpv_target_markerless`).

WHY: copied verbatim from train_daemon_v3.py's pattern -- the ONLY change
across this whole retrain is the target model (world worlds/quad_enemy.sdf,
dataset scripts/seeker/data/quad_dataset), so the training recipe/daemon
shape is reused unchanged. train_daemon_v3.py itself exists because the
first fine-tune launch ran under the run_in_background harness and was
REAPED at epoch 9 (background-task lifetime); this daemon is launched
setsid-detached (immune to that reaping) and RESUMES from a checkpoint if one
exists, falling back to a FRESH run from yolo11n.pt otherwise. On finish it
exports best.pt -> weights/drone_finetuned_quad.{pt,onnx} and prints the
completion sentinel QUAD_TRAIN_EXPORT_DONE.

Recipe is byte-identical to v2/v3 (plan Sec 7): yolo11n.pt base, imgsz 640,
60 epochs, batch 16, seed 0, ultralytics-default augmentation, ONNX opset 12
simplify. Single variable vs v2/v3 is the target model + dataset.

Does NOT touch drone_finetuned_v2.{pt,onnx}, models/fpv_target_markerless,
or scripts/seeker/data/sim_dataset_v2 -- this is a new artifact alongside the
deployed seeker, not a replacement.
"""
import os
import shutil
import sys

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker
REPO = os.path.dirname(os.path.dirname(HERE))              # repo root
os.chdir(REPO)

RUN_DIR = os.path.join("scripts", "seeker", "runs", "drone_finetune_quad")
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
DATA = os.path.join("scripts", "seeker", "data", "quad_dataset", "data.yaml")
BASE = os.path.join(REPO, "yolo11n.pt")
DST_PT = os.path.join("scripts", "seeker", "weights", "drone_finetuned_quad.pt")

TRAIN_KW = dict(data=DATA, imgsz=640, epochs=60, batch=16, seed=0,
                project=os.path.join(HERE, "runs"), name="drone_finetune_quad",
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
    print(f"QUAD_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
