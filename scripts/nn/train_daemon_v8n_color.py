#!/usr/bin/env python3
"""Detached training daemon: yolov8n, 1 class, COLOUR real-media corpus, at 640 then 1280.

Why yolov8n: it is the Hailo-8 target architecture (ADR-0125; yolo11n is not
single-context). Why two sizes: the held-out test split has 62% of its boxes under
16 px at 640 input, so the question is whether 1280 input buys small-target recall
on the SAME architecture and data (pre-registered in runs/hailo_drone_20261113/README.md).

Launch setsid-detached (harness background tasks are reaped at ~51 min):
  setsid nohup .venv-seeker-train-gpu/bin/python scripts/nn/train_daemon_v8n_color.py \
      > logs/nn_tier/train_v8n_color.log 2>&1 &
Each arm resumes from its own last.pt if present. Prints V8N_COLOR_<size>_DONE per arm
and V8N_COLOR_ALL_DONE at the end. One GPU job at a time; batch sized for the 8 GB card.
"""
import os
import shutil
import sys

from ultralytics import YOLO

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.chdir(REPO)
DATA = os.path.join("scripts", "seeker", "data", "nn_tier", "dataset_color.yaml")
BASE = os.path.join("runs", "dfc_dryrun", "yolov8n.pt")       # stock COCO-pretrained yolov8n
RUNS = os.path.join(REPO, "scripts", "seeker", "runs")
OUT = os.path.join("scripts", "seeker", "weights", "nn_tier")
ARMS = [(640, 16), (1280, 6)]                                  # (imgsz, batch)


def run_arm(imgsz: int, batch: int) -> None:
    name = f"nn_tier_v8n_color_{imgsz}"
    wdir = os.path.join(RUNS, name, "weights")
    last, best = os.path.join(wdir, "last.pt"), os.path.join(wdir, "best.pt")
    dst = os.path.join(OUT, f"v8n-color-{imgsz}.pt")
    if os.path.exists(dst):
        print(f"arm {imgsz}: {dst} already exists, skipping", flush=True)
        return
    done = False
    if os.path.exists(last):
        try:
            YOLO(last).train(resume=True)
            done = True
        except Exception as e:                                # finished run or unusable checkpoint
            print(f"arm {imgsz}: resume not possible ({e!r})", flush=True)
            done = os.path.exists(best) and "nothing to resume" in str(e).lower()
    if not done:
        YOLO(BASE).train(data=DATA, imgsz=imgsz, epochs=60, batch=batch, seed=0, workers=4,
                         single_cls=True, project=RUNS, name=name, exist_ok=True)
    if not os.path.exists(best):
        sys.exit(f"arm {imgsz}: no best.pt produced")
    shutil.copy2(best, dst)
    print(f"V8N_COLOR_{imgsz}_DONE -> {dst}", flush=True)


if __name__ == "__main__":
    for imgsz, batch in ARMS:
        run_arm(imgsz, batch)
    print("V8N_COLOR_ALL_DONE", flush=True)
