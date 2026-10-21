# NN seeker program (markerless detector) — plan of record

Opened 2026-10-21 (ADR-0125; builder-ratified consult). Runs PARALLEL to the
tag first-kill thread; tag keeps all bench/flight priority. This file is the
program's home; numbers live in the runs/ dirs cited.

## The detector spec (what the NN must deliver)

From runs/nn_requirements_20261021 (pre-registered isim MC) with the
latency line GAZEBO-RE-EARNED (runs/gz_latency_20261022 — paired, 18
flights, camera-in-the-loop):

| Quantity | Requirement | Note |
|---|---|---|
| Glass-to-guidance latency | **≲30 ms full performance; 60 ms degraded (2.3×, outside ram radius); 120 ms broken** | Gazebo-re-earned 2026-10-22 (runs/gz_latency_20261022): 120 ms cliff CONFIRMED 6/6 @5.5×; the isim '60 ms passes' line REFUTED |
| Update rate | ≥ 10–15 Hz | 24+ Hz free margin |
| Detection range (terminal) | ≥ 8–12 m | acquisition range is set by the CUE geometry, not the terminal |
| Per-frame detect prob | ≥ 0.5 | KF rides through dropouts |

Resolution corollary (runs/res_guidance_20261021): the real variable is
**pixels-per-degree**. 640-wide input works on the delivered 70° lens,
is DEAD on the desired 118° lens (needs native-1280-equivalent); a zoom
lens is NOT a substitute (FOV loss collapses pointing). Predictive/PIP
guidance is NOT rescued by resolution (pre-registered null) — the
chase/pro-nav hybrid idea needs a different mechanism (candidates: predictive
mid-course → chase handoff; bank-angle accel cue ADR-0073).

## Hardware (ruling 1)

Hailo-8 AI HAT+ (26 TOPS, ~$110, ~35 g, ≈+2.7 W on the 5 V rail), model
yolov8n (single-context @640 on hailo8; yolo11n is not). Bench Pi measured
(runs/pi_nn_bench_20261021.json): CPU ORT yolov8n@640 154 ms / v2 net
131 ms — past the latency spec; NCNN ≈2× still leaves no margin.
Pre-purchase: DFC dry-run compiles on WSL2 (docs/hailo_toolchain_runbook.md)
— stock@640, then @1024 (the open higher-res experiment). PURCHASE +
DevZone registration = builder actions (builder_queue `hailo8-purchase`).
Rail impact: Pi 5 wants 5 V/5 A peak; +2.7 W ≈ +0.55 A — size the BEC tap
rework for it (the two-rail rule holds).

## Architecture (ruling 2)

Stock-head yolov8n @640 + detect-then-track (adopted) + prediction-centered
full-resolution ROI re-detect for confirm/reacquire (NOT for first
detection — a second pass cannot extend first-detection range). P2 head and
1280 input are AFTER-stock-compiles experiments, not the baseline.

## Training data (ruling 3) — full staged program, ~2.5–3 weeks wall

1. Permissive sets: Det-Fly (13k air-to-air, MIT; ignore Roboflow license
   relabels) + AOT slice (CDLA-permissive; pull groundtruth.json first,
   filter small-box frames, partial-fetch 50–100 GB — **>2 GB: builder
   go-ahead before the bulk pull**). Drone-vs-Bird/Anti-UAV: private eval
   only (licenses). Commit fetch scripts + SHA manifest, never the data.
2. Grayscale-native training (ultralytics ≥8.3.146 `channels: 1`,
   yolo11n-grayscale checkpoints exist; fall back to 3× replication if the
   Hailo export path balks at 1-ch).
3. Sim imagery with offline Albumentations randomization (rotation-blur
   dominates at 1 ms exposure; NIR-ish gray mixing — no published mono-NIR
   shift study, treat as hypothesis and histogram-match to real frames).
4. Fine-tune on tag-auto-labeled real footage of OUR target (tag pose →
   3-D cuboid → projected box; MASK/inpaint the tag; the tag only decodes
   on near frames — far frames via GNSS projection or tracker propagation).
   1,500–3,000 frames across ≥2 days at ~2 Hz sampling.
   TOOLING IN-TREE (2026-10-22, 13 tests green): scripts/nn/autolabel_tag.py
   (+ deploy-recorder index support) and scripts/nn/sim_randomize.py;
   SMOKED on real Pi desk footage (runs/desk_20261004T205433Z: 13/13 frames
   labeled, mask lands on the tag, calibrated-intrinsics pose clean).
   Remaining sign-off: cuboid-frame direction on one real tagged-QUAD frame
   with --viz before the first training labels are trusted.
5. Validation that matters: different-day different-site TAGLESS set;
   recall binned by target pixel size + first-detection range per approach
   run + false positives/1k frames on target-free clips. mAP50 secondary.

## Tracker/capture (research, to bench-verify)

OV9281 is natively 1280×800 (NOT ×960 — check any 960 assumption in our
camera configs). Plan: full-frame RAW8/Y-plane @~60 fps, 4–6 buffers,
newest-frame-only; KF on detections + pyramidal LK patch assist with
FC-gyro ego-motion compensation; MOSSE as the cheap in-between tracker,
re-seeded every detection; coast-then-expand-search reacquisition with
full-frame re-detect fallback. Two bench unknowns to measure: real
picamera2 fps/latency per mode (LED-flash test), and track-loss/recovery
on replayed flight clips at our detector rate.

## Sequencing (ruling 4)

Parallel background thread. Order of work: DFC dry-runs + dataset scripts
(now, no hardware) → builder: DevZone + HAT order → training stages on the
4070 → Hailo compile + Pi bench (AFTER brn/afr bench gates undisturbed) →
tagless validation day → isim/Gazebo seeker-model re-fit from measured
curves. The tag seeker remains the first-kill baseline throughout.
