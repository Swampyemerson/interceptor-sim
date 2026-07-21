#!/usr/bin/env python3
"""Detached nn_tier fine-tune daemon -- markerless drone-detection NN, real-media
domain-matched to the ordered hardware (OV9281 mono, high-desert operating area).

WHY: this is a NEW model lineage, not a retrain of the deployed sim-trained
`drone_finetuned_quad_v2` -- it is COCO-init (never sim weights, per the
project's honesty/leakage rules) and fine-tuned on the nn_tier corpus built by
`fetch_nn_tier_sources.py` + `prepare_nn_tier_dataset.py`: real drone footage
(DVB / NPS / Roboflow-sourced) + high-desert composite positives + desert-plate
negatives, GRAYSCALE (matches the OV9281 mono sensor the real seeker will run
on -- the deployed v2 seeker was trained on COLOR sim renders, a flagged
mono/color confound this lineage corrects for), split SOURCE/VIDEO/SCENE-
disjoint (train/val/test never share a source video, per the v3/rebal NULL
lesson -- random-frame splits over-fit and looked great in-sample while
failing in flight).

Recipe: yolo11n.pt (COCO pretrained) base, imgsz 640, batch 32 (recon's
recommended safe_batch for the 8 GB 4070 laptop GPU at imgsz 640 -- ~4.2 GB
reserved, ~2.6 GB headroom), up to 60 epochs with patience=20 early-stop
(prior sim-lineage retrains saturated ~epoch 17-20; this corpus is far larger
and more diverse so patience is left a bit looser), seed 0, device 0 (GPU),
ultralytics-default augmentation (grayscale images are stored 3-channel
replicated so default HSV jitter degrades to a harmless V-only brightness
jitter -- S is already ~0 on true-gray pixels, verified against source frames).

Launched setsid-detached (immune to the ~51 min background-task reaper) and
RESUMES from a checkpoint if one exists, falling back to a FRESH run from
yolo11n.pt otherwise. On finish it exports best.pt ->
scripts/seeker/weights/nn_tier/n-mono.{pt,onnx} and copies results.csv to
logs/nn_tier/nmono_results.csv, then prints the completion sentinel
NN_TIER_TRAIN_EXPORT_DONE.

Does NOT touch any deployed sim-lineage weights (drone_finetuned_quad_v2.*),
models/, or any file the in-view-probe agent owns (logs/nn_tier/baseline_*,
docs/inview_probe_results.md, ground_background_sweep.py,
phantom_competition_replay.py, box_scoring.py) -- distinct artifact names
throughout.
"""
import os
import shutil
import sys

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker
REPO = os.path.dirname(os.path.dirname(HERE))              # repo root
os.chdir(REPO)

RUN_NAME = "nn_tier_n_mono"
RUN_DIR = os.path.join("scripts", "seeker", "runs", RUN_NAME)
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
RESULTS_CSV = os.path.join(RUN_DIR, "results.csv")
DATA = os.path.join("scripts", "seeker", "data", "nn_tier", "dataset.yaml")  # gray = deployment track
BASE = os.path.join(REPO, "yolo11n.pt")                    # COCO-pretrained, NEVER sim weights
DST_PT = os.path.join("scripts", "seeker", "weights", "nn_tier", "n-mono.pt")
DST_ONNX = os.path.join("scripts", "seeker", "weights", "nn_tier", "n-mono.onnx")
LOG_RESULTS_CSV = os.path.join("logs", "nn_tier", "nmono_results.csv")

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


def _fresh():
    print("[daemon] FRESH train from yolo11n.pt (COCO-init)", flush=True)
    shutil.rmtree(RUN_DIR, ignore_errors=True)
    YOLO(BASE).train(**TRAIN_KW)


def main() -> int:
    os.makedirs(os.path.dirname(DST_PT), exist_ok=True)
    os.makedirs(os.path.dirname(LOG_RESULTS_CSV), exist_ok=True)

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
    print(f"NN_TIER_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
