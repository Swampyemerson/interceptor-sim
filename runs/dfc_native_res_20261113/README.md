# Compile test: stock yolov8n at the AR0234's input sizes (2026-11-13)

Question: does a nano detector still fit the Hailo-8 in one pass at the input sizes the
1920 x 1200 camera would feed it, and how fast is it on the real chip?

| Input | Compile | Contexts | Chip only (Pi 5, PCIe Gen 3) | Live camera to result |
|---|---|---|---|---|
| 1280 x 800 (1.5x shrink, same shape as the sensor) | OK, 94 s | 1 | 128 fps, 13.5 ms | 41.7 fps, 52.8 ms median |
| 1920 x 1216 (full frame, padded 16 rows) | OK, 120 s | 1 | 56 fps, 26.0 ms | 19.1 fps, 79.7 ms median |

Recipe: `run_native_res.sh` inside an `ubuntu:22.04` container with `.venv-dfc-2204`
(DFC 3.34.0, Model Zoo v2.17); logs `compile_*.log`, `run.log`. Pi-side numbers and the
live test: `runs/hailo_day1_20261113/`.

Limits: stock 80-class pretrained weights, SYNTHETIC calibration images, NMS config left at
640. Fit and speed evidence only. No accuracy number may be quoted from these files. The
project's 1-class variant compiled in 2 contexts at 640 (`runs/dfc_dryrun_single`), so a
1-class build at these sizes may also split and run slower; not tested here.
