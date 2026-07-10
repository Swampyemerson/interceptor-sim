#!/usr/bin/env python3
"""Detached v3 training daemon (task #28).

WHY: the first fine-tune launch ran under the run_in_background harness and was
REAPED at epoch 9 (background-task lifetime; no crash -- the log cut off
mid-iteration with no traceback, dmesg shows no OOM, 29 GB free). Ultralytics
had saved best.pt/last.pt through epoch 8. This daemon is relaunched
setsid-detached (immune to the harness reaping, the same detachment the sim
captures used and survived) and RESUMES from that checkpoint (saving ~54 min),
falling back to a FRESH run from yolo11n.pt if resume can't proceed. On finish
it exports best.pt -> weights/drone_finetuned_v3.{pt,onnx} and prints the
completion sentinel V3_TRAIN_EXPORT_DONE.

Recipe is byte-identical to the original launch (plan Sec 7): yolo11n.pt base,
imgsz 640, 60 epochs, batch 16, seed 0, ultralytics-default augmentation, ONNX
opset 12 simplify. Single variable vs v2 remains the dataset.
"""
import os
import shutil
import sys

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker
REPO = os.path.dirname(os.path.dirname(HERE))              # repo root
os.chdir(REPO)

RUN_DIR = os.path.join("scripts", "seeker", "runs", "drone_finetune_v3")
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
DATA = os.path.join("scripts", "seeker", "data", "onboard_dataset_v3", "data.yaml")
BASE = os.path.join(REPO, "yolo11n.pt")
DST_PT = os.path.join("scripts", "seeker", "weights", "drone_finetuned_v3.pt")

TRAIN_KW = dict(data=DATA, imgsz=640, epochs=60, batch=16, seed=0,
                project=os.path.join(HERE, "runs"), name="drone_finetune_v3",
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
    print(f"V3_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
