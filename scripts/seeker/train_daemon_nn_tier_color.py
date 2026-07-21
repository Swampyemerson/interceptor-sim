#!/usr/bin/env python3
"""Detached nn_tier fine-tune daemon -- the COLOR A/B arm for the mono-vs-color
camera decision (docs/nn_tier/PLAN.md, baseline_scoreboard.md §5).

WHY: sibling of train_daemon_nn_tier.py (n-mono, the deployment-track
grayscale baseline). The camera decision itself is already KEEP MONO
(cheaper OV9281 mono global-shutter sensor). What was never isolated is
whether a model trained NATIVELY on color (not the OOD "color-net-fed-gray"
artifact behind the 0.667->0.967 bird false-fire regression in
baseline_scoreboard.md §5) shows meaningfully better drone-vs-bird
discrimination than a model trained NATIVELY on gray. This daemon trains
that color-native arm so the two can be compared apples-to-apples: n-mono
scored gray, n-color scored color, each in ITS OWN training modality.

Recipe is IDENTICAL to train_daemon_nn_tier.py except the data source:
yolo11n.pt (COCO pretrained) base, imgsz 640, batch 32 (recon's safe_batch
for the 8 GB 4070 laptop GPU at imgsz 640), up to 60 epochs with
patience=20 early-stop, seed 0, device 0 (GPU), ultralytics-default
augmentation (HSV jitter meaningfully affects hue/saturation here since the
corpus is TRUE color, unlike the gray-replicated n-mono corpus -- this is
expected and correct: color-native training should see real chroma jitter).

Data: scripts/seeker/data/nn_tier/dataset_color.yaml -- SAME images, SAME
source/video/scene-disjoint split as the gray dataset.yaml (built by the same
prepare_nn_tier_dataset.py run, just the color/ variant instead of gray/),
so the train/val/test membership is byte-identical to n-mono's -- the only
thing that differs between the two arms is color channel content.

Launched setsid-detached (immune to the ~51 min background-task reaper) and
RESUMES from a checkpoint if one exists, falling back to a FRESH run from
yolo11n.pt otherwise. Self-serializes on the single 8 GB GPU (same
nvidia-smi --query-compute-apps check as train_daemon_nn_tier_aug.py /
train_daemon_nn_tier_smono.py -- reads the process table only, no
pkill/pgrep, no inline literal pattern). On finish it exports best.pt ->
scripts/seeker/weights/nn_tier/n-color.{pt,onnx} and copies results.csv to
logs/nn_tier/ncolor_results.csv, then prints the completion sentinel
NN_TIER_COLOR_TRAIN_EXPORT_DONE.

Does NOT touch any deployed sim-lineage weights (drone_finetuned_quad_v2.*),
models/, the n-mono/n-mono-aug/s-mono lineages' files, or any file the
in-view-probe agent owns -- distinct artifact names throughout.
"""
import os
import shutil
import subprocess
import sys
import time

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker
REPO = os.path.dirname(os.path.dirname(HERE))              # repo root
os.chdir(REPO)

RUN_NAME = "nn_tier_n_color"
RUN_DIR = os.path.join("scripts", "seeker", "runs", RUN_NAME)
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
RESULTS_CSV = os.path.join(RUN_DIR, "results.csv")
DATA = os.path.join("scripts", "seeker", "data", "nn_tier", "dataset_color.yaml")  # color A/B
BASE = os.path.join(REPO, "yolo11n.pt")                    # COCO-pretrained, NEVER sim weights
DST_PT = os.path.join("scripts", "seeker", "weights", "nn_tier", "n-color.pt")
DST_ONNX = os.path.join("scripts", "seeker", "weights", "nn_tier", "n-color.onnx")
LOG_RESULTS_CSV = os.path.join("logs", "nn_tier", "ncolor_results.csv")

TRAIN_KW = dict(
    data=DATA,
    imgsz=640,
    epochs=60,
    batch=32,
    patience=20,
    seed=0,
    device=0,
    workers=8,
    project=os.path.join(HERE, "runs"),
    name=RUN_NAME,
    exist_ok=True,
)


def _other_gpu_training_active() -> bool:
    """True if some OTHER python training process is currently using the GPU.
    Checked via `nvidia-smi --query-compute-apps` (PID) cross-referenced
    against our own PID -- reads the process table only, no pkill/pgrep, no
    inline literal pattern (see train_daemon_nn_tier_aug.py, same check)."""
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=15,
        ).stdout
    except Exception:
        return False
    my_pid = os.getpid()
    for line in out.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            pid = int(line)
        except ValueError:
            continue
        if pid != my_pid:
            return True
    return False


def _wait_for_gpu_free(poll_s: int = 30) -> None:
    waited = 0
    while _other_gpu_training_active():
        if waited % 300 == 0:
            print(f"[daemon] GPU busy with another training run; waiting ({waited}s so far)...", flush=True)
        time.sleep(poll_s)
        waited += poll_s


def _fresh():
    print("[daemon] FRESH train from yolo11n.pt (COCO-init), COLOR A/B arm", flush=True)
    shutil.rmtree(RUN_DIR, ignore_errors=True)
    YOLO(BASE).train(**TRAIN_KW)


def main() -> int:
    os.makedirs(os.path.dirname(DST_PT), exist_ok=True)
    os.makedirs(os.path.dirname(LOG_RESULTS_CSV), exist_ok=True)

    print("[daemon] checking whether another training holds the GPU (one-at-a-time rule)...", flush=True)
    _wait_for_gpu_free()
    print("[daemon] GPU free -- proceeding with n-color training", flush=True)

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
    if os.path.exists(RESULTS_CSV):
        shutil.copy(RESULTS_CSV, LOG_RESULTS_CSV)
    YOLO(DST_PT).export(format="onnx", imgsz=640, opset=12, simplify=True)  # writes DST_ONNX next to DST_PT
    print(f"NN_TIER_COLOR_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
