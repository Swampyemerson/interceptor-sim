# Seeker model weights — provenance & licenses

The `weights/` dir is git-ignored (regenerable downloads). This file records
where each weight came from and under what license, so the portfolio's model
provenance is explicit (brief R6).

| File | Source | License | Notes |
|---|---|---|---|
| `yolov8n.onnx` | Ultralytics YOLOv8n, COCO-pretrained, exported to ONNX | **AGPL-3.0** | Untuned-COCO baseline (no drone class). NN-lane seeker. YOLOv8n is also the target model of the Hailo-8 NN seeker program (ADR-0125); any fine-tune of it inherits AGPL-3.0. AGPL = network-copyleft; do NOT ship in a closed stack. |
| `drone_yolo11x.pt` | [doguilmak/Drone-Detection-YOLOv11x](https://huggingface.co/doguilmak/Drone-Detection-YOLOv11x) `weight/best.pt` | **MIT** (weights) | Single class `drone`; YOLOv11x (56.8 M params); mAP50 0.905 on its own real-photo domain. Trained on ~1k real drone images. Card: "domain adaptation is strongly recommended." |
| `drone_yolo11x_1280.onnx` | Above `.pt`, exported by us at imgsz=1280 | MIT weights; exported with **ultralytics (AGPL) tooling** | Export TOOL is AGPL but only used offline at build time; the resulting ONNX runs under onnxruntime with no ultralytics dependency. If publishing this ONNX, the MIT weight license governs the *weights*; note the tool provenance. |
| `nn_tier/n-mono.onnx` (+ `.pt`) | Ours: Ultralytics YOLO11n, COCO-initialised, trained on the real-photo corpus (2026-07-21; `docs/detection_tracking_methods.md` §B.2) | **AGPL-3.0** (same as its YOLO11n base and Ultralytics tooling; see takeaway below) | **Flight default** (`flight/deploy/seeker_loop.py`; pinned by `tests/test_deployed_weights.py`). Not distributed: this dir is git-ignored. |
| `drone_finetuned_quad_v2.onnx` (+ `.pt`) | Ours: Ultralytics YOLO11n fine-tuned from `yolo11n.pt` on Gazebo-rendered quad frames (`scripts/seeker/train_daemon_quad_v2.py`, `docs/quad_target_retrain.md`) | **AGPL-3.0** (same as its YOLO11n base and Ultralytics tooling) | Sim seeker / historical bar; blind on real imagery. Not distributed. |
| `drone_finetuned_v2.onnx` (+ `.pt`) | Ours: YOLO11n fine-tune on Gazebo frames (ADR-0040/0042) | **AGPL-3.0** (same lineage) | Retired sim-era default (ADR-0061); kept locally only. |
| `object_tracking_vittrack_2023sep.onnx` | [opencv/opencv_zoo](https://github.com/opencv/opencv_zoo) `models/object_tracking_vittrack/` | **BSD-3-Clause** | ViT-based (transformer) tracker read by `cv2.TrackerVit`; `../detect_track.py`'s opt-in `MARKERLESS_TRACK_KIND=VIT` A/B arm against the default CSRT (ADR-0076 fix #2). ~700 KB (int8-quantized backbone), pre-trained, not sim-domain-tuned -- it tracks a generic visual blob's appearance/motion, not drone-specific features. |

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

# VIT tracker weight (~700 KB; opencv_zoo stores it via git-lfs, so pull the
# LFS blob directly rather than the raw.githubusercontent.com pointer file):
curl -fL -o weights/object_tracking_vittrack_2023sep.onnx \
  https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/object_tracking_vittrack/object_tracking_vittrack_2023sep.onnx
```
