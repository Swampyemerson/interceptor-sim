#!/usr/bin/env python3
"""REAL-DATA seeker fine-tune (the R6 retrain) — trains a markerless drone
detector on AprilTag-auto-labeled real flight footage (docs/real_data_pipeline.md).

RECIPE (each choice traces to a logged sim finding — do not change without one):
- **INIT FROM COCO-pretrained yolo11n**, NOT the sim weights. The visual domain is
  the one thing that does NOT transfer (ADR-0061 v3 null, add #18d rebal null both
  aced their own val and regressed in flight because they leaned on sim appearance).
- **~15-20% negatives.** The rebal lesson (add #15): 36% hard-neg injection wrecked
  precision; ~15% preserved it. autolabel_from_apriltag.py already emits free
  negatives (tag-miss frames -> empty labels) — subsample to the ratio.
- **Held-out WHOLE FLIGHTS for val**, never random frames (the v3/rebal anti-mirage
  gate). Use build_onboard_dataset_v3.py's flight-level split, or split dirs by flight.
- **Gentle fine-tune** (lr0 ~0.002, ~30-50 ep) — teach the real domain without
  destroying COCO's general features.
- **Gate on APPROACH-phase recall vs range**, NOT val mAP (ADR-0076 add #18h: the
  make-or-break number is recall on the small, fast, APPROACHING target). See
  eval below / eval_nn.py adapted for approach frames.

Runs on a dev box / cheap cloud GPU; the Pi never trains and .venv-seeker-train is
CPU-only (~hours at this scale). setsid-detached + resume-from-last recommended for
long runs (see train_daemon_quad_rebal.py). Exports best.pt -> <name>.{pt,onnx}.

    scripts/seeker/train_real_data.py --data datasets/real_flight/data.yaml --name drone_real_v1
    scripts/seeker/train_real_data.py --dry-run   # verify wiring + the base model, no dataset needed
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(HERE, "weights")

RECIPE = dict(base="yolo11n.pt", epochs=40, lr0=0.002, imgsz=640,
              patience=15, single_cls=True)


def dry_run():
    """Verify the training wiring without a dataset: ultralytics importable, the
    COCO base fetchable, the recipe coherent. Exits 0/1 so CI can gate it before
    any real footage exists."""
    ok = True
    try:
        from ultralytics import YOLO
        print("[dry-run] ultralytics import: OK")
    except Exception as e:  # pragma: no cover
        print(f"[dry-run] ultralytics import FAILED: {e}")
        return False
    try:
        YOLO(RECIPE["base"])  # downloads/loads the COCO-pretrained nano
        print(f"[dry-run] base model {RECIPE['base']} (COCO-pretrained): OK")
    except Exception as e:
        print(f"[dry-run] base model load FAILED (needs network first time): {e}")
        ok = False
    print(f"[dry-run] recipe: init={RECIPE['base']} epochs={RECIPE['epochs']} "
          f"lr0={RECIPE['lr0']} imgsz={RECIPE['imgsz']} single_cls={RECIPE['single_cls']}")
    print("[dry-run] REMINDERS: COCO-init not sim-weights; ~15-20% negatives; "
          "held-out WHOLE FLIGHTS for val; gate on APPROACH-recall-vs-range not mAP.")
    print(f"[dry-run] {'PASS' if ok else 'FAIL'}")
    return ok


def train(args):
    from ultralytics import YOLO
    if not os.path.exists(args.data):
        raise SystemExit(f"dataset yaml not found: {args.data}\n"
                         "Build it with autolabel_from_apriltag.py + a flight-level "
                         "split (build_onboard_dataset_v3.py), then point --data at it.")
    model = YOLO(args.base or RECIPE["base"])  # COCO-pretrained nano
    print(f"[train-real] fine-tuning {args.base or RECIPE['base']} on {args.data} "
          f"({args.epochs} ep, lr0={args.lr0}) -> {args.name}")
    model.train(data=args.data, epochs=args.epochs, lr0=args.lr0,
                imgsz=RECIPE["imgsz"], patience=RECIPE["patience"],
                single_cls=RECIPE["single_cls"], name=args.name, exist_ok=True)
    best = os.path.join("runs", "detect", args.name, "weights", "best.pt")
    out_pt = os.path.join(WEIGHTS, args.name + ".pt")
    out_onnx = os.path.join(WEIGHTS, args.name + ".onnx")
    if os.path.exists(best):
        import shutil
        shutil.copy(best, out_pt)
        YOLO(best).export(format="onnx", imgsz=RECIPE["imgsz"])
        exported = os.path.join("runs", "detect", args.name, "weights", "best.onnx")
        if os.path.exists(exported):
            shutil.copy(exported, out_onnx)
        print(f"[train-real] exported -> {out_pt}, {out_onnx}")
        print("REAL_TRAIN_EXPORT_DONE")
    print("[train-real] NEXT: eval on HELD-OUT flights, gate on approach-recall-vs-range "
          "(NOT mAP) + FP/hour; then shadow-mode on real flights before it steers.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", help="YOLO data.yaml (flight-level split)")
    ap.add_argument("--name", default="drone_real_v1", help="output weights name")
    ap.add_argument("--base", default=None, help=f"base model (default {RECIPE['base']} COCO)")
    ap.add_argument("--epochs", type=int, default=RECIPE["epochs"])
    ap.add_argument("--lr0", type=float, default=RECIPE["lr0"])
    ap.add_argument("--dry-run", action="store_true", help="verify wiring + base model, no dataset")
    args = ap.parse_args()
    if args.dry_run:
        return 0 if dry_run() else 1
    if not args.data:
        ap.error("--data DATASET.yaml required (or --dry-run)")
    train(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
