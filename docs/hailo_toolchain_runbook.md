# Hailo toolchain runbook (pre-purchase de-risk + deployment)

Distilled 2026-10-21 from a sourced research pass (links in the ADR-0125
research outputs); verify versions against hailo.ai before executing.
Decision context: ADR-0125 ruling 1 (Hailo-8 AI HAT+, model yolov8n).

## The pre-purchase play (no hardware needed)

The Dataflow Compiler (DFC) runs WITHOUT a Hailo device and reports context
count + hardware-only fps. So before money moves:

1. **Builder action:** register a (free) Hailo Developer Zone account and
   download **DFC 3.33.x** (the 3.x line serves Hailo-8/8L; 5.x is 10/15
   only). Account creation is a builder-only step (credentials rule).
2. Install on WSL2 (officially listed; CPU-only quantization works, GPU
   under WSL2 is unofficial): Ubuntu 24.04 OK, Python 3.10–3.12, ≥16 GB RAM.
   CPU compile answers fit/context questions; final accuracy-grade
   optimization (needs GPU, ≥1024 calibration images) can run on a native
   boot or rented GPU later.
3. Dry-run compiles, in order: (a) stock yolov8n @640 → expect 1 context on
   hailo8 (reference build confirms); (b) yolov8n @1024 → UNKNOWN, this is
   the experiment that answers the higher-res question for the 118° lens;
   (c) only then any P2-head attempt (known-fragile: 8 end nodes, a
   yolov5-p2 precedent quantized to 0 mAP).

## Known-good pins

- DFC 3.33.x + HailoRT 4.23.0 (`sudo apt install dkms hailo-all` — Trixie
  only; our bench Pi IS on Trixie, verified 2026-10-21). Never compile with
  a DFC newer than the Pi's HailoRT (3.33-on-4.20 fails with a firmware
  error).
- Export: `yolo export model=best.pt format=onnx imgsz=640 opset=11`.
- Compile route A (model zoo): `hailomz compile yolov8n --ckpt best.onnx
  --hw-arch hailo8 --calib-path calib/ --classes 1`.
- Compile route B (ultralytics ≥8.4.97): `yolo export format=hailo
  name=hailo8 ...` — **default target is hailo8l; set hailo8 explicitly.**
- End nodes: the six head convs `/model.22/cv{2,3}.{0,1,2}/cv*.2/Conv`
  (v8) — read them from OUR onnx, numbering shifts with surgery.
- NMS runs on the Pi CPU via HailoRT (`engine=cpu` in the .alls), not
  on-chip, whatever the marketing says. Set `classes`/`image_dims` in the
  NMS json (single class for us).
- Calibration: ≥1024 images, RGB at input size, FROM THE DEPLOYMENT CAMERA
  (grayscale replicated 3-ch if we train 1-channel — verify export path).

## Pi-side pattern

- picamera2's Hailo wrapper (`create_infer_model` + `run_async` futures);
  request a 640×640 lores stream so the ISP does the resize. Submit frame
  N+1 while N is on-chip — naive loops measured as low as 15 fps while the
  same chip benches 62–144.
- Day-1 check (one test covers versions + context + PCIe x1 cost):
  `hailortcli fw-control identify` then `hailortcli benchmark model.hef`,
  compare hw-only vs streaming fps against the compile report.

## Watch-items

- INT8 small-target recall is the biggest accuracy risk: before flying,
  run the held-out real set through ONNX-CPU vs HEF and compare recall
  binned by target pixel size; re-calibrate with our own small-target
  frames if it drops. Pass/fail threshold written down BEFORE the run.
- Microchip bought Hailo (2026-09); DFC license is free but revocable;
  8L HATs are being delisted — another reason the 8 (26 TOPS) is the buy.
