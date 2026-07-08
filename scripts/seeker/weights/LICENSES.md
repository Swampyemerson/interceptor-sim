# Seeker model weights — provenance & licenses

The `weights/` dir is git-ignored (regenerable downloads). This file records
where each weight came from and under what license, so the portfolio's model
provenance is explicit (brief R6).

| File | Source | License | Notes |
|---|---|---|---|
| `yolov8n.onnx` | Ultralytics YOLOv8n, COCO-pretrained, exported to ONNX | **AGPL-3.0** | Untuned-COCO baseline (no drone class). NN-lane seeker. AGPL = network-copyleft; do NOT ship in a closed stack. |
| `drone_yolo11x.pt` | [doguilmak/Drone-Detection-YOLOv11x](https://huggingface.co/doguilmak/Drone-Detection-YOLOv11x) `weight/best.pt` | **MIT** (weights) | Single class `drone`; YOLOv11x (56.8 M params); mAP50 0.905 on its own real-photo domain. Trained on ~1k real drone images. Card: "domain adaptation is strongly recommended." |
| `drone_yolo11x_1280.onnx` | Above `.pt`, exported by us at imgsz=1280 | MIT weights; exported with **ultralytics (AGPL) tooling** | Export TOOL is AGPL but only used offline at build time; the resulting ONNX runs under onnxruntime with no ultralytics dependency. If publishing this ONNX, the MIT weight license governs the *weights*; note the tool provenance. |

## License takeaway for the public portfolio (R6)
- The COCO YOLOv8n baseline and any YOLOv8/YOLO11 fine-tune are **AGPL-3.0**.
- The doguilmak drone weights are **MIT** (clean), but the model is an
  extra-large (x) research model — **not** an embedded/edge detector, and it
  did not transfer to the sim target (see `../drone_detector_eval.py` result).
- For a clean, deployable public artifact prefer an **Apache-2.0** nano
  (NanoDet-Plus / MobileNet-SSD) fine-tuned on in-domain sim imagery
  (`../train_drone_finetune.py`), or accept AGPL knowingly.
- **unidrone** (StephanST) was rejected: its modified-MIT license **excludes
  defense/military use**, which conflicts with this counter-UAS framing, and its
  classes are ground objects, not drones.

## To regenerate
```
# drone weights (114 MB):
curl -L -o weights/drone_yolo11x.pt \
  https://huggingface.co/doguilmak/Drone-Detection-YOLOv11x/resolve/main/weight/best.pt
# export to ONNX @1280 (needs the .venv-seeker-train venv: ultralytics+torch):
.venv-seeker-train/bin/python -c "from ultralytics import YOLO; \
  YOLO('scripts/seeker/weights/drone_yolo11x.pt').export(format='onnx', imgsz=1280, opset=12, simplify=True)"
mv scripts/seeker/weights/drone_yolo11x.onnx scripts/seeker/weights/drone_yolo11x_1280.onnx
```
