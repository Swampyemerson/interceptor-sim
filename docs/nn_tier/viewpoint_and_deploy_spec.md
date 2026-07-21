# NN tier — viewpoint-match + deployment-envelope spec (RECON/target_deploy)

> **Purpose.** The two specs that make the markerless drone-detection NN MATCH THE ORDERED
> HARDWARE: (A) what our target (a 5-inch Kakute-H7 FPV quad) actually looks like to the
> ordered innomaker OV9281 seeker camera across the terminal band — the SCALES / ASPECTS /
> FRAME POSITIONS the Dataset phase must weight; and (B) the Hailo-8L deployment envelope +
> the scale-down decision framework — which yolo11 variant runs fast enough, and what "fast
> enough" means at this project's terminal LOS rates.
>
> Desk derivation + web-cited benchmarks only — no sim booted, no hardware in hand (the HAT
> is DEFERRED; the Pi 5 + camera are ordered but not arrived). Every number below is either
> derived here (formula shown), traced to a repo doc/run, or cited to a vendor/benchmark
> source. Numbers that can only be settled on real hardware are flagged BENCH-GATED.
> Date: 2026-07-21.

---

## A. Viewpoint-match spec — what the OV9281 sees of the target

### A1. Camera model — two honest bounds (per `docs/placard_sizing.md` §4 / `docs/camera_paper_check.md` §2)

The OV9281 module: **monochrome global shutter, 1280×800, HFoV 118°** (horizontal, stated by
innomaker; `camera_paper_check.md` §1). The delivered wide M12 lens's true projection is
unknown until checkerboard calibration, so this spec brackets it:

| bound | model | px/rad | px/deg (center) | meaning |
|---|---|---:|---:|---|
| **conservative** | ideal pinhole, fx = 640/tan(59°) | **384.6** | 6.71 | hard lower bound on center resolution (placard doc's "conservative") |
| **realistic** | equidistant (f-theta) wide lens, 1280 px / 118° | **621.5** | 10.85 | the `camera_paper_check.md` §2 avg-px/deg figure; typical for a 118° M12 |

A real barrel-distorted wide lens sits between these; **calibration on arrival replaces both**
(`scripts/calibrate_camera.py`). Vertical: 800 rows ⇒ VFoV ≈ 74° (equidistant), cy = 400.

### A2. Target geometry (the ordered Kakute H7 5-inch build)

| extent | value | source |
|---|---|---|
| tip-to-tip incl. props (the box "width" from most aspects) | **0.35 m** | the project's sanctioned 5" drone extent — `autolabel_from_apriltag.py` default; `placard_sizing.md` §1 |
| frame motor-to-motor (no props) | ~0.22–0.25 m | Source One V6 5" class geometry |
| vertical silhouette, level edge-on (stack + battery) | ~0.08 m | 5" build height, estimate |
| vertical silhouette incl. pitched rotor disc (target pitches ~10–20° at ≥9 m/s) | ~0.15 m | 0.35·sin(pitch) disc projection + body, estimate |

The sim's `models/fpv_quad_enemy` is an x500-scale stand-in (~0.5 m diagonal) — for REAL-media
dataset work use the **0.35 m** 5-inch extent, not the sim model's size.

### A3. Target pixel size across the 6–20 m terminal band  ← `target_px_by_range`

Box width W = f·(0.35 m)/R; height H = f·(0.08…0.15 m)/R. The 6–20 m band is the terminal
acquisition envelope: the money gate needs R_acq ≥ 6.0 m (`tripod_score.py` arithmetic,
`placard_sizing.md` §4), and ~20 m is the NEXT.md R_acq rule-of-thumb anchor.

| range | box width (px @1280) cons / real | box height (px @1280) cons / real | width at the **640 model input** cons / real |
|---:|---:|---:|---:|
| 6 m | 22.4 / 36.3 | 5.1–9.6 / 8.3–15.5 | 11.2 / 18.1 |
| 8 m | 16.8 / 27.2 | 3.8–7.2 / 6.2–11.7 | 8.4 / 13.6 |
| 10 m | 13.5 / 21.8 | 3.1–5.8 / 5.0–9.3 | 6.7 / 10.9 |
| 12 m | 11.2 / 18.1 | 2.6–4.8 / 4.1–7.8 | 5.6 / 9.1 |
| 15 m | 9.0 / 14.5 | 2.1–3.8 / 3.3–6.2 | 4.5 / 7.3 |
| 20 m | 6.7 / 10.9 | 1.5–2.9 / 2.5–4.7 | 3.4 / 5.4 |

**What this dictates to the Dataset phase:**
1. **This is a tiny-object task.** At the yolo11n@640 deployment anchor the target is
   **~3–18 px wide** across the band, with a **wide, flat aspect ratio (~3–4.5 : 1)**.
   The training corpus must be DOMINATED by boxes in this scale band (weight ≥70% of
   positives at ≤40 px @1280-equivalent), not the large near-field drones most public
   datasets serve up.
2. **Do NOT reach for crops/higher input res as the fix** — the auto-crop/foveated lever was
   TESTED AND REJECTED in sim (identical recall to full-frame at the wall band; resolution
   only mattered past ~24 m static — graveyard, ADR-0076 add #18j-fix / ADR-0074 addendum
   #3). The dataset matches the scales; it doesn't fight them with resolution.
3. **Grayscale, non-negotiable.** The OV9281 is monochrome; train and evaluate in GRAYSCALE
   (channel-replicated), per the flagged mono confound (`camera_paper_check.md` §4 — the
   color-trained v2's chroma cues are dead weight on this sensor).

### A4. Aspect distribution  ← `aspect_weights`

Geometry: coded open-loop dash aimed at a lead point on the crossing target's leg (target
straight-line ≥2 m/s, gate scenario 9 m/s closing), then pro-nav terminal. On a collision
course the seeker approaches from the target's forward hemisphere; as pro-nav leads onto the
crossing track the view rotates toward beam. Rear aspects occur only on re-attack after a
missed pass. **These weights are a design ESTIMATE derived from that geometry — not a
measured distribution; the first real dash ULogs + tripod session refine them.**

| aspect (angle off target's nose) | weight | why |
|---|---:|---|
| nose-on / front (0–30°) | **0.30** | early acquisition on the collision triangle |
| front-quarter (30–60°) | **0.35** | the bulk of a lead-pursuit closing geometry on a crosser |
| beam / side (60–90°) | **0.25** | late terminal + crossing legs |
| rear-quarter (>90°) | **0.10** | re-attack / tail-chase only |

**Elevation aspect (as important as azimuth):** both aircraft fly near co-altitude, so the
dominant view is **edge-on / near-level (±15°): weight 0.70** — the THIN silhouette (a 5"
quad seen level is a ~4:1 flat sliver, often mostly props; this is the sim's known thin-prop
regime). Moderate look-up/down (15–30°): 0.25. Steep (>30°, rotor disc opening up): 0.05.
Public "ground observer looking up at a drone belly" imagery is exactly the WRONG elevation
aspect — weight it low (it's the documented failure of `drone_yolo11x`,
`candidate_nn_shortlist.md`).

### A5. Frame-position distribution  ← `position_weights`

Drivers: (a) the ±30° open-loop aim tolerance (constraint `wide-fov`) sets the horizontal
spread; (b) the ~25–40° nose-down dash pitch (build-specific; `hardware_order_list.md` §0d/
Tier-2) would put a co-altitude target FAR HIGH in frame, but the ordered **fixed up-tilt
bracket is locked to the measured steady dash pitch** → re-centres the target at steady dash;
residual vertical spread comes from pitch transients (accel phase pitches past trim → target
appears high; braking → low) and altitude offsets. Again a design ESTIMATE from geometry.

| axis | band | weight |
|---|---|---:|
| horizontal | within ±10° of boresight (±~108 px of cx @10.85 px/deg) | 0.55 |
| horizontal | 10–30° off boresight | 0.35 |
| horizontal | >30° (frame edges; aim-tolerance tail + late-terminal crossing drag) | 0.10 |
| vertical | central band ±10° of boresight | 0.45 |
| vertical | upper 10–37° (accel-phase under-tilt residual — the high-in-frame bias) | 0.35 |
| vertical | lower 10–37° | 0.20 |

Practical consequence: **augment with placement over the full frame but bias center-high**,
and include edge-of-frame partial-visibility positives (the crossing target exits laterally).
Include ~15–20% NEGATIVE frames (empty desert sky, ground clutter, birds) per the standing
rebal lesson (`docs/real_data_pipeline.md`).

---

## B. Deployment-envelope spec — Hailo-8L + the scale-down framework

### B1. The hardware anchor

**Hailo-8L AI HAT+ (13 TOPS, $70, DEFERRED buy — `hardware_order_list.md` §0c)** on the Pi 5
8GB, single camera stream, batch = 1 (a live seeker has no batching). Deployment anchor:
**yolo11n @640, INT8, compiled to `.hef`**. The HAT purchase is gated on the markerless
phase; nothing here changes that gating.

### B2. Model sizes + Hailo-8L compiled performance (vendor model-zoo numbers)

From the official Hailo Model Zoo **HAILO8L** object-detection table (Dataflow Compiler
v2.19.0; COCO 640×640; FPS as published for the compiled model — chip-level, batch as noted):

| model | params | GFLOPs | ONNX fp32 (approx) | COCO mAP float→INT8 | **FPS batch=1** | FPS batch=8 |
|---|---:|---:|---:|---:|---:|---:|
| yolov8n | 3.2 M | 8.7 | ~13 MB | 37.0 → 36.4 | **202** | 438 |
| yolov8s | 11.2 M | 28.6 | ~43 MB | 44.6 → 43.9 | **110** | 208 |
| yolov8m | 25.9 M | 78.9 | ~99 MB (measured: our `flying_objects_yolov8m.onnx` is 99.3 MB) | 49.9 → 49.2 | **51** | 87 |
| **yolo11n** | **2.6 M** | **6.6** | **~10 MB** | 39.0 → 37.5 | **157** | 371 |
| yolo11s | 9.4 M | 21.6 | ~36 MB | 46.3 → 45.1 | **92** | 192 |
| yolo11m | 20.1 M | 68.1 | ~77 MB | 51.1 → 49.9 | **35** | 58 |

**Compile path (confirmed, standard):** Ultralytics `.pt` → ONNX → `hailomz compile
<model> --ckpt <our_finetune>.onnx --calib-path <calib_imgs>/ --hw-arch hailo8l` → `.hef`.
yolo11n/s and yolov8n/s/m are first-class Model Zoo entries for HAILO8L, so a custom
single-class fine-tune rides the stock recipe. Two real pipeline obligations: (a) INT8
quantization needs **200–1000 calibration images from OUR domain** — use grayscale
high-desert frames, not COCO; (b) quantized-vs-float mAP drop must be re-measured on OUR
val split (the table's ~0.5–1.5 pt COCO drop is not automatically ours). BENCH-GATED:
actual fps on our Pi, INT8 recall on our data.

### B3. End-to-end expectations on the Pi 5 (chip fps ≠ pipeline fps)

The model-zoo numbers are NPU-kernel figures. End-to-end (capture → grayscale →
letterbox → NPU → NMS/decode on CPU → track) costs roughly a 2× haircut per this repo's own
tiering (`docs/nn_transfer_plan.md`: BEST ~100+ chip-alone, EXPECTED ~30–35 end-to-end,
WORST ~15–20 under two-stream/thermal derate). Independent data point: Seeed's Pi 5 +
Hailo-8L bench ran yolov8s INT8 @640 at **80–120 fps** (batch 2–8, PCIe gen3 pipeline) —
consistent with a batch-1 end-to-end estimate in the tens of fps. Realistic single-stream
end-to-end estimates:

| model | chip batch=1 | **est. end-to-end on Pi 5** | verdict vs the 30-fps bar (B4) |
|---|---:|---:|---|
| **yolo11n** | 157 | **~40–80 fps** | **CLEARS with margin — the anchor** |
| yolo11s | 92 | ~25–45 fps | MARGINAL — affordable only if the bench shows ≥30 sustained under thermal load |
| yolo11m | 35 | ~15–25 fps | **FAILS** — offline teacher / auto-label assistant only, never deployed |

(The OV9281 itself does 800p @ up to ~120 fps, so the sensor is not the cap.) All
BENCH-GATED — constraint `pi5-emulation-gap`: fps cannot be pre-measured in emulation; the
first HAT bench replaces this table.

### B4. What fps is "usable at terminal LOS rates"?  ← `usable_fps_threshold`

Terminal LOS rates in this project: **mean ~485°/s, peaking 600–1870°/s inside the blind
window near CPA** (`docs/terminal_diagnosis.md`; `docs/seeker_acquisition_range_note.md`;
ADR-0023). Three traced arguments:

1. **Don't size fps to the CPA peak — it's unwinnable and already adjudicated.** Holding
   ≤2°/frame at 485°/s needs ~242 fps; at 1870°/s, ~935 fps. No Hailo-8L model + pipeline
   reaches that, and ADR-0023 already ruled the miss is KINEMATIC at handoff — the blind
   window is not the lever. So the threshold is set by the approach phase, not the endgame.
2. **The money-gate arithmetic sets the floor at 30 fps.** `tripod_score.py` constants
   (TGO_MIN 0.5 s, V_closing 9 m/s, HANDOFF_STREAK 5): streak burn = (5/fps)·9 m/s;
   t_go = (R_acq − burn)/9 ≥ 0.5 s. At the gate-threshold R_acq = 6.0 m: 30 fps → burn
   1.5 m → t_go = 0.50 s **PASS exactly**; 25 fps → 0.47 s FAIL; 15 fps → 0.33 s FAIL. At
   the placard-realistic R_acq = 7.1 m, 25 fps squeaks by (0.59 s) but 15 fps still fails.
3. **Match the validated loop + mid-course tracking.** The sim-validated seeker loop
   (ADR-0058 detect-then-track) ran at 30 Hz; at the 60°/s mid-course LOS spec, 30 fps =
   2°/frame ≈ 22 px/frame — inside a sane tracker gate. Slower than validated = an
   unmodeled gap.

**THRESHOLD: ≥30 fps end-to-end sustained (thermal-loaded), latency ≤ ~1 frame (~33 ms).**
60 fps is worth having if free (halves streak burn to 0.75 m → +0.08 s t_go, halves
per-frame LOS step) but is NOT required. Below 30 fps the money gate fails at the
gate-threshold acquisition range → not deployable.

### B5. Scale-down decision framework  ← `scale_down_verdict_framework`

**Standing verdict from the table: yolo11n suffices — no further scale-down is needed for
Hailo-8L.** The full decision procedure, for when the bench numbers land or the model choice
is revisited:

1. **Compile gate:** the candidate must compile to `.hef` at `--hw-arch hailo8l` with the
   stock Model Zoo recipe (yolo11n/s, yolov8n/s/m all do). A model without a zoo recipe
   (exotic head, transformer) = reject unless someone pays the porting cost.
2. **Chip-rate gate:** `hailortcli` batch=1 fps must be **≥2× the 30-fps bar (≥60)** to
   survive the pipeline haircut (B3's tiering). yolo11n 157 ✓, yolo11s 92 ✓(thin),
   yolo11m 35 ✗.
3. **Bench gate (the real one):** end-to-end ≥30 fps sustained on the Pi with the OV9281
   stream, active cooler on, thermal-soaked. This is the number that decides — vendor
   numbers only rank.
4. **If a candidate fails:** drop model size first (m→s→n), THEN input size (640→512→416),
   THEN prune/quantize harder. **Prefer model-size cuts over input-size cuts** — A3 shows
   the target is 3–18 px at 640 already; shrinking the input destroys signal linearly,
   while n-vs-s costs COCO mAP that may not even matter for our single-class tiny-target
   task.
5. **When to spend UP (n→s):** only if the fine-tuned n shows a recall gap on the held-out
   FLIGHT/SOURCE split that s demonstrably closes, AND s passes gate 3 on the bench. COCO
   mAP deltas (+7.6 for s) are NOT evidence for our task — frame-eval gains have twice
   failed to transfer to flight in this repo (ADR-0061).

### B6. CPU-baseline fallback (the HAT-less phase)  ← `cpu_fallback_note`

The Pi 5 CPU alone: **AprilTag baseline runs real-time (30+ fps)** — that is the entire
tag-phase plan and why the HAT is deferred (`hardware_order_list.md` §0c). **CPU YOLO is
~5–10 fps** (constraint `pi5-compute`; anchor: ~13 fps YOLOv8n measured on a desktop-class
CPU, ADR-0015) — it fails the B4 bar by 3–6× and is NOT viable for markerless terminal
guidance. Sanctioned CPU uses for the NN: **shadow-mode logging** during tag-guided flights
(log-only detections for offline scoring — the Phase-4b plan) and offline eval. The Hailo
HAT is the graduation gate to a markerless kill; nothing in this spec moves that gating.

### B7. MIT-licensed starting NN — confirmation  ← `mit_nn_confirm`

Confirmed per `docs/candidate_nn_shortlist.md` (+ `weights/LICENSES.md`): MIT-marked
drone-detection checkpoints EXIST and two are already in-repo/fetched —
`doguilmak/Drone-Detection-YOLOv11x` (MIT weights; already evaluated: did NOT transfer,
ground-observer-up aspect) and `Javvanny/yolov8m_flying_objects_detection` (MIT weights,
fetched 49.6 MB / ONNX 99.3 MB; aspect-mismatched but carries a labeled Bird class). Three
binding caveats:
1. **"MIT weights, AGPL tooling":** both MIT grants are uploader metadata on artifacts
   produced with Ultralytics' AGPL-3.0 code — fine for this portfolio, re-verify before any
   product ship (already logged in `weights/LICENSES.md`).
2. **None has the right aspect.** The closest aspect match (Det-Fly, air-to-air) is a
   DATASET with a genuinely ambiguous license (MIT repo file vs CC-BY mirror) — get written
   confirmation before training on it.
3. **The sanctioned fine-tune init is COCO, not any drone checkpoint** (`real_data_pipeline.md`
   recipe; and never sim weights). Starting from an MIT drone checkpoint is at most an A/B
   arm against COCO-init, not the default. Note also both MIT checkpoints are x/m-sized —
   far outside the Hailo-8L envelope (B2) — so whatever wins the A/B, the DEPLOYED artifact
   is an n(/s)-sized model trained on the domain-matched corpus this tier is building.

---

## Sources

- Repo: `docs/camera_paper_check.md`, `docs/placard_sizing.md`, `docs/hardware_order_list.md`
  (§0b/§0c/§0d), `docs/nn_transfer_plan.md`, `docs/candidate_nn_shortlist.md`,
  `docs/terminal_diagnosis.md`, `docs/seeker_acquisition_range_note.md`,
  `scripts/seeker/tripod_score.py` (gate constants), `docs/project_state.json` (constraints
  `pi5-compute`, `pi5-emulation-gap`, `wide-fov`; graveyard: foveated-crop),
  `scripts/seeker/autolabel_from_apriltag.py` (0.35 m extent).
- [Hailo Model Zoo — HAILO8L object detection table](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8L/HAILO8L_object_detection.rst)
  (fetched 2026-07-21; Dataflow Compiler v2.19.0 — all chip-fps/mAP numbers in B2).
- [Seeed: yolov8s benchmark on RPi5 + Hailo-8L](https://wiki.seeedstudio.com/benchmark_on_rpi5_and_cm4_running_yolov8s_with_rpi_ai_kit/) (80–120 fps, batch 2–8, INT8 @640).
- [Ultralytics Hailo export docs](https://docs.ultralytics.com/integrations/hailo) ·
  [Cytron: ONNX→HEF conversion for the AI HAT+](https://www.cytron.io/tutorial/raspberry-pi-ai-kit-onnx-to-hef-conversion) ·
  [YOLOv11n→HEF compile guide](https://common.rosecityrobotics.com/YOLO_ObjectDetection/YOLOv11n_to_Hailo8_Guide.html)
  (the `hailomz compile --hw-arch hailo8l` custom-weights path in B2).
- [innomaker CAM-MIPIOV9281 V2 product page](https://www.inno-maker.com/product/cam-mipi9281raw-v2/) (118° HFoV, via `camera_paper_check.md`).
