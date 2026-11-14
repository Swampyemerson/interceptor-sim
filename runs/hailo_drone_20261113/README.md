# Drone detector on the Hailo-8: first real-weights build (opened 2026-11-13)

Builder directive 2026-11-13: "may as well have you try and get the drone detection NN
model going now". This folder holds the pre-registrations and results.

## What exists going in
- Real-media colour corpus (`scripts/seeker/data/nn_tier/color`): train 10,357 / val 859 /
  test 4,175 images, split by source so no video is in two splits.
- `n-color` (YOLO11n, 1 class, 640 input, trained 2026-07-21): held-out AP50 0.437,
  precision 0.69, recall 0.43 (`logs/nn_tier/eval_n-color_ncolor_heldout.csv`).
- Test-split target sizes at 640 input (box longer side), measured 2026-11-13:
  under 8 px 459, 8 to 16 px 2,212, 16 to 32 px 913, 32 to 64 px 288, 64 px and up 443.
  So 62% of the test targets are under 16 px at 640 input.

## PRE-REGISTRATION 1: does 8-bit conversion cost recall? (written before any chip run)
- **Config:** `n-color` exported to ONNX, compiled for hailo8 with DFC 3.34 on CPU
  (default optimisation level; calibration = 1,200 letterboxed colour TRAIN images,
  half of them chosen from frames whose target is under 16 px). Scored on the test split
  on the real chip, against the float ONNX run on the dev machine with the SAME letterbox,
  the SAME score threshold (0.25) and the same matching rule (a ground-truth box is found
  if a detection overlaps it with IoU of 0.3 or more).
- **Metric:** recall per target-size bin (the five bins above), and false detections per
  1,000 images on the 469 target-free test images.
- **Prediction:** the chip version loses under 5 recall points in every bin.
- **Pass:** recall drop of 5 points or less in every bin with 200 or more boxes, and no
  more than double the float model's false detections.
- **Fail means:** apply the rescue ladder in `docs/hailo_toolchain_runbook.md` in order;
  do not fly the chip version.
- **A null on the question itself:** if the FLOAT model's recall under 16 px is itself
  below 0.5, quantisation is not the limiter; input size or data is, and that sends the
  effort to pre-registration 2.

## PRE-REGISTRATION 2: does 1280 input buy small-target recall? (written before training)
- **Config:** yolov8n (the Hailo target architecture), 1 class, COCO-pretrained start,
  same colour corpus, 60 epochs, seed 0, default augmentation; arm A at 640 input
  (batch 16), arm B at 1280 input (batch 6). `scripts/nn/train_daemon_v8n_color.py`.
  Arm A exists so the comparison is one variable (input size), not architecture too.
- **Metric:** test-split recall per target-size bin at score 0.25 (bins defined at 640
  scale so both arms count the same boxes), AP50, and false detections per 1,000
  target-free images.
- **Prediction:** arm B raises recall in the 8 to 16 px bin by 10 points or more and
  AP50 by 0.05 or more over arm A.
- **Adopt 1280 if:** both of those hold and false detections do not double.
- **A null means:** input size is not the limiter on this corpus; the next spend is data
  from our own target and camera, not resolution or a bigger model.
- **Known limits:** the corpus is general drone media, not our target seen from our
  camera; a screen or desk test is not a flight test; statistics are per image, and
  images from one video are correlated.

## RESULT 1 (2026-11-13): the existing detector runs on the chip, and FAILS the 8-bit check

Build: `n-color` compiled with `compile_ncolor.sh` (DFC 3.34, CPU, so optimisation level 0:
the log says "Adaround skipped", "Finetune encoding skipped"). The first attempt stopped
with a quantisation error in the one-class output layer (`compile_ncolor_attempt1.log`);
forcing the three class outputs to the range 0 to 1 fixed it. The file is 3 contexts and
runs at 98.8 fps, 8.1 ms on the Pi (PCIe Gen 3).

All 4,175 test images, score 0.25, IoU 0.3 (`score.py`, `dets_*.json`):

| Target size at 640 input | Boxes | Float recall | Chip recall | Change |
|---|---|---|---|---|
| under 8 px | 493 | 0.243 | 0.095 | -14.8 points |
| 8 to 16 px | 2,191 | 0.588 | 0.422 | -16.6 |
| 16 to 32 px | 900 | 0.440 | 0.334 | -10.6 |
| 32 to 64 px | 289 | 0.439 | 0.349 | -9.0 |
| 64 px and up | 442 | 0.525 | 0.471 | -5.4 |
| all | 4,315 | 0.502 | 0.367 | -13.5 |

False detections per 1,000 target-free images: float 108.7, chip 29.9.

**Verdict against pre-registration 1: FAIL** (every bin drops more than 5 points). Do not
fly this chip build. The registered next step is rescue-ladder step 1: the optimisers never
ran, because the compile had no GPU.

Not registered, exploratory only: on the 1,775 boxes both versions find, the median score is
0.521 float and 0.396 on the chip, and false detections fell 3.6 times, so much of the loss
is the chip's scores sitting lower rather than targets vanishing. At the chip's lowest
available threshold (0.20) recall is 0.414 with 57.6 false per 1,000.

The float model itself is weak (recall about 0.5 at every size above 8 px), so data and
training matter more than the conversion.

## RESULT 2 (2026-11-13): live through the camera, one photo on a laptop screen

`live_drone.py`, 120 frames per size, a held-out drone photo shown full-screen and rotated so
it is upright for the camera (`drone_test_page.html`, `live_sizes.txt`):

| Drone size on the sensor (native px) | At 640 input | Frames with a detection |
|---|---|---|
| about 149 | about 50 px | 120 of 120 (score 0.38) |
| about 110 (estimated from the page scale) | about 37 px | 0 of 120 |
| about 73 | about 24 px | 2 of 120 |
| smaller (four sizes) | under 18 px | 0 of 120 |

Loop rate 59.7 fps, 29 ms from frame start to result (`live_drone_largest.jpg`).
This is ONE photo, seen at a steep angle with a cable across part of the frame, on a screen
closer than the lens's focus distance. It shows the whole chain works and that this build
only finds a large target; it is not a range measurement.

## RESCUE STEP 1, registered before running (2026-11-13)
Same model, same test, same metric and pass mark as pre-registration 1. One change: the
converter runs with the laptop's GPU (Docker + NVIDIA Container Toolkit, CUDA 12.5 image,
cuDNN 9.3 supplied separately because the image's 9.2 is too old for TensorFlow 2.18),
optimisation level 2, compression level 0, all 1,200 calibration images
(`compile_ncolor_gpu.sh`). Prediction: the recall loss shrinks to under 5 points in the bins
of 16 px and up, and to under 10 points below 16 px. If the log again says the optimisers
were skipped, the run does not count.

## SCREEN BENCH (2026-11-13, camera square-on to the laptop screen)

What it is: `screen_bench/bench.html` plays itself (white, black, a calibration disc, then
10 held-out drone photos at 8 sizes, 2.5 s each); `screen_bench/bench_pi.py` runs the detector
on every camera frame and cuts the frames into steps from the white-to-black edge. The disc
gives the scale (15.3 sensor px per 1% of screen height) and the target point. About 89
frames per step, 60 fps. A hit is a detection at the target point whose box is 0.5 to 1.6
times the drone's size.

Two instrument faults found and fixed before any number was kept:
1. First run (`bench_ncolor_640_run1_square_patch_artefact.json`): photos were shown as hard
   squares on grey and the detector fired on the SQUARE (box 3.3 times the drone) at small
   sizes. Fix: photos fade radially into their own edge colour; hits must be drone-sized.
2. Control pass (`bench_ncolor_640_control.json`, drone-free patches of the same photos):
   0 false hits at every size down to 32 px on the sensor, but 10% and 30% at the two
   smallest sizes. So the bench is NOT valid below about 11 px at 640 input.

Result for the current chip build of `n-color` (`bench_ncolor_640.json`):

| Drone on the sensor | At 640 input | Like a 0.3 m target at | Frames hit | Photos hit in half their frames | Control |
|---|---|---|---|---|---|
| 147 px | 49 px | 2.3 m | 0.70 | 7 of 10 | 0.00 |
| 108 px | 36 px | 3.2 m | 0.82 | 8 of 10 | 0.00 |
| 78 px | 26 px | 4.4 m | 0.67 | 7 of 10 | 0.00 |
| 59 px | 20 px | 5.8 m | 0.40 | 4 of 10 | 0.00 |
| 44 px | 15 px | 7.8 m | 0.38 | 4 of 10 | 0.00 |
| 32 px | 11 px | 10.6 m | 0.35 | 4 of 10 | 0.00 |
| 24 px | 8 px | 14 m | not valid | | 0.10 |
| 18 px | 6 px | 19 m | not valid | | 0.30 |

The distance column assumes the lens's listed 80 degree view (focal length 1,144 px); this
camera is not calibrated yet. Limits: 10 photos, all from one source dataset (DUT Anti-UAV),
chosen because the float model already finds them, so the numbers flatter the detector; a
screen is not the sky; one screen pixel is about one sensor pixel, which a full-resolution
model may see as a grid. Use it to COMPARE builds on the same optics, not as a range claim.
