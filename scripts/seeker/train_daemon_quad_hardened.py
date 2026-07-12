#!/usr/bin/env python3
"""Detached quad-enemy HARDENED retrain daemon -- phantom negative-mining
(ADR-0076 add #5/#6, ADR-0077 addendum; ADR-0038 lever).

WHY: copied verbatim from train_daemon_quad_v2.py's pattern -- the ONLY
changes across this retrain are the dataset (quad_dataset_hardened,
quad_dataset_v2 UNION phantom_mine_quad.py's real-flight hard negatives) and
the output artifact names, so the training recipe/daemon shape is reused
unchanged. `drone_finetuned_quad_v2` (level+banked, ALL-POSITIVE training
data) intercepts well flying the quad l2r (81% Pk@2.5) but flying r2l it
locks a big FALSE box off-target during acquisition (reads
meas_range~1.5 m while the true range is 15-20 m) -- guidance chases it, 7 m
miss, r2l Pk 0/16 (ADR-0076 add #5/#6). A range-plausibility gate
(ADR-0077) failed to fix this (NULL, regressed l2r). quad_dataset_hardened
(merge_quad_hardened.py) adds the missing signal: real onboard frames where
the seeker false-fired, carrying their EXISTING label (empty for a true
negative, gt-only for a positive-with-phantom frame) so the phantom's pixel
region is taught down as background -- the classic hard-negative-mining
retrain.

Launched setsid-detached (immune to the ~51 min background-task reaper) and
RESUMES from a checkpoint if one exists, falling back to a FRESH run from
yolo11n.pt otherwise. On finish it exports best.pt ->
weights/drone_finetuned_quad_hardened.{pt,onnx} and prints the completion
sentinel HARDENED_TRAIN_EXPORT_DONE.

Recipe is byte-identical to quad v1/v2 (and the v2/v3 billboard retrains):
yolo11n.pt base, imgsz 640, 60 epochs, batch 16, seed 0, ultralytics-default
augmentation, ONNX opset 12 simplify. Single variable vs v2 is the dataset
(quad_dataset_v2 union real hard negatives) + output artifact names.

Does NOT touch drone_finetuned_v2.{pt,onnx} (the deployed billboard seeker),
drone_finetuned_quad.{pt,onnx} (the level-only quad baseline),
drone_finetuned_quad_v2.{pt,onnx} (the current deployed quad seeker, kept
as-is and as the A/B baseline this hardened retrain is measured against),
models/fpv_target_markerless, or scripts/seeker/data/sim_dataset_v2 -- this
is a new artifact alongside all of them, not a replacement.
"""
import os
import shutil
import sys

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker
REPO = os.path.dirname(os.path.dirname(HERE))              # repo root
os.chdir(REPO)

RUN_DIR = os.path.join("scripts", "seeker", "runs", "drone_finetune_quad_hardened")
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
DATA = os.path.join("scripts", "seeker", "data", "quad_dataset_hardened", "data.yaml")
BASE = os.path.join(REPO, "yolo11n.pt")
DST_PT = os.path.join("scripts", "seeker", "weights", "drone_finetuned_quad_hardened.pt")

TRAIN_KW = dict(data=DATA, imgsz=640, epochs=60, batch=16, seed=0,
                project=os.path.join(HERE, "runs"), name="drone_finetune_quad_hardened",
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
    print(f"HARDENED_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
