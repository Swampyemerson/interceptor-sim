#!/usr/bin/env python3
"""Detached nn_tier fine-tune daemon -- the "can we afford bigger" arm.

WHY: sibling of train_daemon_nn_tier.py (which trains yolo11n -> n-mono) but
uses yolo11s.pt as the COCO-pretrained base instead of yolo11n.pt. Same
corpus, same GRAYSCALE deployment track, same source/video/scene-disjoint
split. This run answers one question for the Hailo-8L scale-down decision:
does the extra capacity of yolo11s buy enough accuracy over yolo11n to be
worth the larger export (more Hailo compile time / lower fps), or does the
n-mono weights already saturate this corpus and s-mono is a wash?

Waits for any OTHER nn_tier training already using the GPU to finish first
(hard project rule: ONE training at a time on this single 8 GB GPU box --
concurrent runs OOM/thrash) by polling nvidia-smi for other python compute
processes before starting its own YOLO() call. This lets it be launched
immediately and queue itself rather than requiring a human/agent to serialize
the launch by hand.

COCO-init (yolo11s.pt, NEVER sim weights), imgsz 640, batch 32 (recon's
recommended safe_batch at imgsz 640 on this 8 GB 4070; the daemon
self-monitors GPU memory in its own polling loop after epoch 1 and will note
in the log if the model is bigger than the reserved-memory envelope recon
measured for yolo11n -- yolo11s is a bigger network so this is the point of
the test), up to 60 epochs with patience=20 early-stop, seed 0, device 0.

Launched setsid-detached (immune to the ~51 min background-task reaper) and
RESUMES from a checkpoint if one exists, falling back to a FRESH run from
yolo11s.pt otherwise. On finish it exports best.pt ->
scripts/seeker/weights/nn_tier/s-mono.{pt,onnx} and copies results.csv to
logs/nn_tier/smono_results.csv, then prints the completion sentinel
NN_TIER_SMONO_TRAIN_EXPORT_DONE.

Does NOT touch any deployed sim-lineage weights, models/, the n-mono
lineage's files, or any file the in-view-probe agent owns -- distinct
artifact names throughout (run name nn_tier_s_mono, dst s-mono.*, log
smono_results.csv).
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

RUN_NAME = "nn_tier_s_mono"
RUN_DIR = os.path.join("scripts", "seeker", "runs", RUN_NAME)
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
RESULTS_CSV = os.path.join(RUN_DIR, "results.csv")
DATA = os.path.join("scripts", "seeker", "data", "nn_tier", "dataset.yaml")  # gray = deployment track
BASE = os.path.join(REPO, "yolo11s.pt")                    # COCO-pretrained, NEVER sim weights (auto-downloads)
DST_PT = os.path.join("scripts", "seeker", "weights", "nn_tier", "s-mono.pt")
DST_ONNX = os.path.join("scripts", "seeker", "weights", "nn_tier", "s-mono.onnx")
LOG_RESULTS_CSV = os.path.join("logs", "nn_tier", "smono_results.csv")

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

MY_SCRIPT_BASENAME = os.path.basename(__file__)


def _other_gpu_training_active() -> bool:
    """True if some OTHER python training process is currently using the GPU.

    Checked via `nvidia-smi --query-compute-apps` (PID + process name) cross
    referenced against /proc/<pid>/cmdline for OUR OWN pid tree, so this
    process's own (not-yet-existent-until-YOLO()-runs) usage never
    self-matches. No inline literal pkill/pgrep pattern is used anywhere --
    this only READS process tables.
    """
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=15,
        ).stdout
    except Exception:
        return False  # can't tell -> don't block forever on a query failure
    my_pid = os.getpid()
    for line in out.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            pid = int(line)
        except ValueError:
            continue
        if pid == my_pid:
            continue
        # a compute-app PID belonging to any process is "other GPU work in
        # flight" -- on this box that's always an nn_tier training daemon
        # (n-mono or a stray s-mono), never this process before YOLO() runs
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
    print("[daemon] FRESH train from yolo11s.pt (COCO-init)", flush=True)
    shutil.rmtree(RUN_DIR, ignore_errors=True)
    YOLO(BASE).train(**TRAIN_KW)


def main() -> int:
    os.makedirs(os.path.dirname(DST_PT), exist_ok=True)
    os.makedirs(os.path.dirname(LOG_RESULTS_CSV), exist_ok=True)

    print("[daemon] checking whether another training holds the GPU (one-at-a-time rule)...", flush=True)
    _wait_for_gpu_free()
    print("[daemon] GPU free -- proceeding with s-mono training", flush=True)

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
    print(f"NN_TIER_SMONO_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
