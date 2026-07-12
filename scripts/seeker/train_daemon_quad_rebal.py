#!/usr/bin/env python3
"""Balance-corrected quad seeker retrain (ADR-0076 add #15, Fable review lever).

WHY: the from-scratch hard-negative retrain (train_daemon_quad_hardened.py:
yolo11n base, 36% negatives) SUCCEEDED at suppressing the own-prop phantom
(it acquired the real r2l target early, ~15-18 m) but REGRESSED box precision
(l2r terminal 0.9 -> 2.7 m, overall 13% vs 44%) -- an unbalanced-injection
problem, not a sensor limit (Fable review, decisions.md ADR-0076 add #15).
The r2l failure is the interceptor's OWN propeller blades read as a phantom
target; every SPATIAL/downstream separator failed (range unreliable, prop
pixels OVERLAP the real target in image space -- the image-mask regressed l2r),
so APPEARANCE discrimination (this retrain) is the only remaining separator.

THE FIX (vs the hardened daemon): (1) INIT FROM drone_finetuned_quad_v2.pt
(the deployed, good-PRECISION quad seeker) instead of yolo11n -- so the phantom
negatives NUDGE an already-precise detector rather than dominating a
from-scratch one; (2) LOWER negative ratio (~15% via quad_dataset_rebal:
quad_dataset_v2 UNION 469 subsampled true-neg + 152 pos-with-phantom, vs the
hardened 36%); (3) GENTLE lr0 (0.002 vs the 0.01 default) + fewer epochs (30) --
a fine-tune, not a re-train, to KEEP v2's precision while teaching the prop down.

Success = r2l Pk@2.5 recovers toward l2r (the real target wins box-selection
once the phantom is suppressed) AND l2r HOLDS (precision preserved -- the thing
the hardened retrain lost). Validate leakage-free on unseen master-seed 123.

setsid-detached (immune to the ~51 min reaper); RESUMES from last.pt if present,
else fine-tunes fresh from v2. Exports best.pt -> drone_finetuned_quad_rebal.
{pt,onnx} and prints REBAL_TRAIN_EXPORT_DONE. Does NOT touch any deployed weight.
"""
import os
import shutil
import sys

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
os.chdir(REPO)

RUN_DIR = os.path.join("scripts", "seeker", "runs", "drone_finetune_quad_rebal")
LAST = os.path.join(RUN_DIR, "weights", "last.pt")
BEST = os.path.join(RUN_DIR, "weights", "best.pt")
DATA = os.path.join("scripts", "seeker", "data", "quad_dataset_rebal", "data.yaml")
BASE = os.path.join("scripts", "seeker", "weights", "drone_finetuned_quad_v2.pt")
DST_PT = os.path.join("scripts", "seeker", "weights", "drone_finetuned_quad_rebal.pt")

TRAIN_KW = dict(data=DATA, imgsz=640, epochs=30, batch=16, seed=0, lr0=0.002,
                project=os.path.join(HERE, "runs"), name="drone_finetune_quad_rebal",
                exist_ok=True)


def _fresh():
    print(f"[daemon] FINE-TUNE from {BASE} (lr0=0.002, 30 ep, ~15% neg)", flush=True)
    shutil.rmtree(RUN_DIR, ignore_errors=True)
    YOLO(BASE).train(**TRAIN_KW)


def main() -> int:
    if os.path.exists(LAST):
        try:
            print(f"[daemon] RESUME from {LAST}", flush=True)
            YOLO(LAST).train(resume=True)
        except Exception as exc:
            print(f"[daemon] resume failed ({exc!r}); falling back to FRESH", flush=True)
            _fresh()
    else:
        _fresh()

    if not os.path.exists(BEST):
        print(f"[daemon] FAILED: {BEST} missing after training", flush=True)
        return 2
    shutil.copy(BEST, DST_PT)
    YOLO(DST_PT).export(format="onnx", imgsz=640, opset=12, simplify=True)
    print(f"REBAL_TRAIN_EXPORT_DONE -> {DST_PT} + .onnx", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
