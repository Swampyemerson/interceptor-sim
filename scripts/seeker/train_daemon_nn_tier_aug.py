#!/usr/bin/env python3
"""Detached nn_tier fine-tune daemon -- the "robustness arm" (HEAVY augmentation).

WHY: sibling of train_daemon_nn_tier.py (n-mono, default-augmentation baseline)
and train_daemon_nn_tier_smono.py (s-mono, bigger COCO base, default
augmentation). This third arm answers a different question: does pushing the
augmentation MUCH harder buy generalization headroom for the deployment
environment -- outdoor, high-desert (sagebrush/juniper/basalt/dust, bright sky
+ hard shadows), a monochrome global-shutter sensor with no OIS, and a fast
terminal dash/jink where the target can motion-blur -- over and above what the
plain-augmentation n-mono arm gets from the corpus alone?

Same corpus as n-mono (`scripts/seeker/data/nn_tier/dataset.yaml`, the
GRAYSCALE/deployment-track split, source/video/scene-disjoint, never
random-frame -- the v3/rebal NULL lesson), same yolo11n.pt COCO-init (NEVER
sim weights), same imgsz/batch/device. What changes is the augmentation
pipeline:

  - HSV jitter forced OFF (hsv_h=hsv_s=hsv_v=0.0). The corpus is grayscale
    replicated to 3ch; the n-mono arm leaves ultralytics' default HSV jitter
    on and it degrades to a harmless V-only brightness jitter on true-gray
    pixels (S~0). This arm makes that explicit/deliberate instead of
    incidental, and moves the "brightness variation" job onto the
    Albumentations RandomBrightnessContrast/RandomGamma transforms below,
    which are tuned for the high-desert bright-sky/hard-shadow lighting this
    model will actually see instead of ultralytics' generic HSV-V jitter.
  - Mosaic left at ultralytics' max (mosaic=1.0, already the default) --
    "heavy" here means we don't dial it back, not that we invent a heavier
    mosaic mechanism.
  - Native affine hyperparameters raised moderately (degrees, translate,
    shear, perspective, mixup) for extra viewpoint/scale robustness. Kept
    moderate rather than maxed: recon's dataset report flags 70% of positives
    are already <=40px at 1280-equivalent (small-target-dominated corpus);
    an aggressive `scale` jitter would shrink already-tiny boxes below the
    label-usability floor and destroy signal rather than add robustness, so
    `scale` is left at the ultralytics default (0.5) deliberately.
    `copy_paste` is left at 0 -- ultralytics' CopyPaste is a no-op for
    bbox-only datasets with no segmentation polygons (see
    ultralytics.data.augment.CopyPaste.__call__: returns unchanged when
    `len(labels["instances"].segments) == 0`), and this corpus has box labels
    only, so enabling it would be a silent no-op, not real augmentation.
  - A custom heavy Albumentations transform list is injected via
    ultralytics' officially-supported `augmentations=` train() override (see
    ultralytics.data.augment.v8_transforms:
    `Albumentations(p=1.0, transforms=getattr(hyp, "augmentations", None))`,
    and ultralytics.cfg.check_dict_alignment's `allowed_custom_keys =
    {"augmentations", "save_dir"}` -- this is a supported extension point,
    not a monkeypatch) supplying BLUR (Blur/MotionBlur/MedianBlur -- defocus,
    atmospheric haze, and target motion-blur during a fast terminal
    dash/jink on a sensor with no OIS) and DUST (GaussNoise for
    airborne-dust/sensor grain, CoarseDropout with random-uniform fill for
    coarse occlusion patches standing in for dust puffs / sagebrush-juniper
    clutter partially occluding the target) plus RandomBrightnessContrast /
    RandomGamma / CLAHE (bright-sky + hard-shadow contrast swings) and
    ImageCompression (onboard-capture artifacts). CoarseDropout is one of
    ultralytics' recognized "spatial" Albumentations transforms (see the
    `spatial_transforms` set in ultralytics.data.augment.Albumentations), so
    the wrapper composes it with `bbox_params` and threads bboxes/labels
    through automatically -- no custom bbox handling needed here.

Requires `albumentations` (installed into this run's venv,
.venv-seeker-train-gpu, ~120 MB total download incl. scipy/opencv-headless
transitive deps -- logged here per the download-size rule; this is a small
package install, not dataset/model corpus, so it is reported in this
docstring/notes rather than the corpus gb_downloaded figure).

Follows the same "wait for the single 8 GB GPU to be free" self-serialization
as train_daemon_nn_tier_smono.py -- this box trains ONE config at a time
(hard project rule: concurrent trainings OOM/thrash), and at write time both
n-mono and s-mono daemons are already running/queued, so this arm must queue
behind them rather than assume it owns the GPU.

Launched setsid-detached (immune to the ~51 min background-task reaper) and
RESUMES from a checkpoint if one exists, falling back to a FRESH run from
yolo11n.pt otherwise. On finish it exports best.pt ->
scripts/seeker/weights/nn_tier/n-mono-aug.{pt,onnx} and copies results.csv to
logs/nn_tier/nmono_aug_results.csv, then prints the completion sentinel
NN_TIER_AUG_TRAIN_EXPORT_DONE.

Does NOT touch any deployed sim-lineage weights, models/, the n-mono/s-mono
lineages' files, or any file the in-view-probe agent owns -- distinct
artifact names throughout (run name nn_tier_n_mono_aug, dst n-mono-aug.*, log
nmono_aug_results.csv).
"""
import os
import shutil
import subprocess
import sys
import time

import albumentations as A
from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))          # scripts/seeker
REPO = os.path.dirname(os.path.dirname(HERE))              # repo root
os.chdir(REPO)

RUN_NAME = "nn_tier_n_mono_aug"
RUN_DIR = os.path.join("scripts", "seeker", "runs", RUN_NAME)
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
RESULTS_CSV = os.path.join(RUN_DIR, "results.csv")
DATA = os.path.join("scripts", "seeker", "data", "nn_tier", "dataset.yaml")  # gray = deployment track
BASE = os.path.join(REPO, "yolo11n.pt")                    # COCO-pretrained, NEVER sim weights
DST_PT = os.path.join("scripts", "seeker", "weights", "nn_tier", "n-mono-aug.pt")
DST_ONNX = os.path.join("scripts", "seeker", "weights", "nn_tier", "n-mono-aug.onnx")
LOG_RESULTS_CSV = os.path.join("logs", "nn_tier", "nmono_aug_results.csv")

# Heavy BLUR + DUST + high-desert-lighting Albumentations pipeline, injected
# via ultralytics' native `augmentations=` train() override (NOT a
# monkeypatch -- see module docstring). Pixel-only transforms except
# CoarseDropout, which ultralytics recognizes as "spatial" and wires bboxes
# through automatically.
HEAVY_ALBUMENTATIONS = [
    # --- blur family: defocus / atmospheric haze / terminal-dash motion blur ---
    A.Blur(blur_limit=(3, 7), p=0.20),
    A.MedianBlur(blur_limit=(3, 5), p=0.10),
    A.MotionBlur(blur_limit=(3, 9), p=0.20),
    # --- "dust": airborne grain + coarse occlusion patches ---
    A.GaussNoise(std_range=(0.02, 0.10), p=0.30),
    A.CoarseDropout(
        num_holes_range=(1, 4),
        hole_height_range=(0.03, 0.12),
        hole_width_range=(0.03, 0.12),
        fill="random_uniform",
        p=0.30,
    ),
    # --- high-desert lighting: bright sky + hard shadow contrast swings ---
    A.RandomBrightnessContrast(brightness_limit=0.25, contrast_limit=0.30, p=0.35),
    A.RandomGamma(gamma_limit=(70, 130), p=0.20),
    A.CLAHE(clip_limit=3.0, p=0.15),
    # --- onboard-capture artifacts ---
    A.ImageCompression(quality_range=(50, 100), p=0.15),
]

TRAIN_KW = dict(
    data=DATA,
    imgsz=640,
    epochs=60,
    batch=32,
    patience=25,           # a bit looser than n-mono/s-mono (20): heavier aug converges slower
    seed=0,
    device=0,
    workers=8,
    project=os.path.join(HERE, "runs"),
    name=RUN_NAME,
    exist_ok=True,
    # HSV jitter OFF -- grayscale corpus, replaced by the Albumentations
    # brightness/contrast/gamma transforms above, tuned for the deployment
    # lighting instead of ultralytics' generic HSV-V jitter.
    hsv_h=0.0,
    hsv_s=0.0,
    hsv_v=0.0,
    # mosaic left at ultralytics' max (1.0, the default) -- "heavy" means we
    # don't dial it back.
    mosaic=1.0,
    # moderate extra affine robustness; `scale` deliberately left at the
    # ultralytics default (0.5) -- see module docstring (70% of positives are
    # already <=40px, an aggressive scale-down would erase small-object signal).
    degrees=10.0,
    translate=0.2,
    shear=2.0,
    perspective=0.0003,
    mixup=0.15,
    # copy_paste deliberately left at 0 -- no-op on this bbox-only (no
    # segmentation polygon) corpus, see module docstring.
    augmentations=HEAVY_ALBUMENTATIONS,
)

MY_SCRIPT_BASENAME = os.path.basename(__file__)


def _other_gpu_training_active() -> bool:
    """True if some OTHER python training process is currently using the GPU.

    Checked via `nvidia-smi --query-compute-apps` (PID) cross-referenced
    against our own PID, so this process's own (not-yet-existent-until-
    YOLO()-runs) usage never self-matches. No inline literal pkill/pgrep
    pattern is used anywhere -- this only READS process tables.
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
        # flight" -- on this box that's always another nn_tier training
        # daemon (n-mono or s-mono), never this process before YOLO() runs
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
    print("[daemon] FRESH train from yolo11n.pt (COCO-init), HEAVY augmentation robustness arm", flush=True)
    shutil.rmtree(RUN_DIR, ignore_errors=True)
    YOLO(BASE).train(**TRAIN_KW)


def main() -> int:
    os.makedirs(os.path.dirname(DST_PT), exist_ok=True)
    os.makedirs(os.path.dirname(LOG_RESULTS_CSV), exist_ok=True)

    print("[daemon] checking whether another training holds the GPU (one-at-a-time rule)...", flush=True)
    _wait_for_gpu_free()
    print("[daemon] GPU free -- proceeding with n-mono-aug (heavy augmentation) training", flush=True)

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
    print(f"NN_TIER_AUG_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
