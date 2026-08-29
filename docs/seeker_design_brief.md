# Markerless seeker — design brief ("kill the AprilTag")

*Post-M5 build-queue item 2 (ADR-0033). Design-as-ADR-first research brief for
replacing the AprilTag target-lock with a **markerless** seeker that detects the
target drone's own body and feeds the SAME bearing/range interface the guidance
already consumes. Widened scope (builder, 2026-07-07): classical CV **and** a
pre-built lightweight neural detector needing minimal adaptation.*

> **Why this milestone exists.** docs/goals.md makes one honest simplification: the real
> onboard seeker is a Pi-class camera running classical CV; in the sim we swap that
> for an AprilTag — "a clean, robust fiducial that lets us focus on the *guidance*
> problem instead of the *perception* problem." This item deliberately **un-does**
> that simplification and attacks the project's #1 disclosed risk (ADR-0015 seat-5:
> "reliably ACQUIRING and HOLDING an onboard lock on a small, fast, comms-denied
> drone" is the existential problem no dollar buys out). Even a partial result
> converts the portfolio from "guidance demo with a solved perception problem" into
> "seeker development" (ADR-0033).

New terms, one line each (defined once, used freely after):
- **Fiducial** — a printed marker (our AprilTag) designed to be trivially detectable
  and pose-solvable. A real drone has none; that's the whole problem.
- **Bearing** — the *angle* from the camera boresight (its straight-ahead axis) to the
  target. Cheap and accurate from a camera.
- **Range** — the *distance* to the target. Expensive and inaccurate from one camera.
- **LOS (line of sight)** and **LOS rate (λ̇)** — the direction to the target, and how
  fast that direction is rotating. Pro-nav steers on λ̇, not on range (pronav skill).
- **Detect-then-track** — the layered pipeline the parent design already assumes:
  cheap motion proposal → small classifier → correlation tracker → filter, instead of
  running a heavy detector every frame (ADR-0015 §2).

---

## 1. The interface contract a replacement seeker MUST satisfy

The single seam that makes this a *seeker-swap* and not a guidance rewrite is one
dataclass and one thread. **Get these exactly right and nothing downstream changes.**

### 1.1 The data object — `Measurement`
`scripts/m3_static_intercept.py:188-200` (imported by `m4_intercept.py:220-228`):

| field | type | meaning | markerless equivalent |
|---|---|---|---|
| `t_mono` | float | capture time (monotonic seconds) | same — stamp when the frame was grabbed |
| `range_m` | float \| None | distance to target (m); `None` = not detected this frame | **the hard part** — see §3 |
| `bearing_rad` | float \| None | horizontal angle off boresight, `+`=target is RIGHT; `atan2(x, z)` in the camera optical frame | box-centroid pixel → angle via `atan2((u−cx)/fx, 1)` |
| `meas_xyz` | np.ndarray \| None | target position in the camera OPTICAL frame (x right, y down, z forward) | reconstruct from `range·[sinβ, sinβ_vert, cosβ]` once you have a range |
| `decision_margin` | float \| None | AprilTag decode confidence; used for quality gating | **repurpose as a detector confidence scalar** (NN score, or motion-blob strength) |
| `n_detections` | int | how many tags this frame | number of candidate boxes (1 for the single-target sim) |

The optical-frame convention (x right, y down, z forward) is OpenCV's and is fixed in
docs/goals.md's coordinate-frames section — get the bearing sign wrong and pro-nav steers
the wrong way. `+bearing = target right of boresight` is load-bearing.

### 1.2 The producer — `detection_loop`
`scripts/m3_static_intercept.py:233-290`. Runs on its own background thread for the
whole flight. Today it:
1. grabs the newest frame (latest-wins, never queues — `LatestFrame`, m3:203-219),
2. calls `detector.detect(gray, estimate_tag_pose=True, camera_params=(fx,fy,cx,cy),
   tag_size=TAG_SIZE_M)`,
3. takes `detections[0]`, sets `range_m = ‖pose_t‖`, `bearing_rad = atan2(x, z)`,
4. publishes a fresh `Measurement` to `MeasurementHolder.latest` (atomic assignment).

**A markerless seeker replaces steps 2–3 only.** Everything above (the
`FrameSource` seam, `scripts/frame_source.py`) and everything below (the filter and
guidance) is untouched. `frame_source.py` was built for exactly this: it yields
grayscale frames + intrinsics, and "the guidance code is identical either way"
(its docstring). The clean move is a new `markerless_detection_loop(frame_holder,
meas_holder, intrinsics, …)` with the same signature contract.

### 1.3 What the consumer actually needs (and how forgiving it is)
The guidance reads `MeasurementHolder.latest` once per control tick and does two things:
- **Steering (the important one):** `lambda_meas = psi_rad + meas.bearing_rad`
  (`m4_intercept.py:1590, 2320`) — own-yaw ψ plus camera bearing β gives the *inertial*
  LOS azimuth λ; an **α-β filter** (a constant-velocity smoother: position gain
  `ALPHA=0.5`, rate gain `BETA_GAIN_LAMBDA=0.30`, `m4:239-246`) turns the stream of λ
  samples into a smoothed λ and its rate λ̇, and pro-nav commands `a_cmd = N·Vc·λ̇`
  (N=5 FPV / N=4 M4, pronav skill).
- **Closing/gating (the forgiving one):** `range_m` feeds the closing-speed throttle
  (`compute_v_close`, m4:483) and the handoff/abort gates. It sets the *magnitude* of
  Vc and *when* phases switch — not the steering direction.

Two properties make the swap low-risk:
1. **Latest-wins + α-β coast.** The control loop never blocks on detection; a slower or
   blinkier markerless detector just publishes fewer `Measurement`s and the filter
   *coasts* (predicts) through the gaps (m4 filter docstrings; same pattern the tag
   already relies on through terminal dropout, ADR-0023). The interface was built to
   tolerate a worse sensor.
2. **`sample_measurement` staleness gate** (m3:293-300, `MEAS_STALE_S`) already treats
   an old result as "not detected," so a markerless seeker inherits graceful
   degradation for free.

**Contract summary for the builder:** produce a `Measurement` with a clean
`bearing_rad` (radians, +=right), a *coarse-is-OK* `range_m` (§3), a confidence in the
`decision_margin` slot, and `None`-fill every field when there's no detection; publish
latest-wins at whatever rate you can sustain (the tag runs ~14 Hz effective with
`quad_decimate=2`, ADR-0015 row 7 / ADR-0023). Target ≥ that, but the loop survives less.

---

## 2. Candidates

Rate columns are the honest embedded numbers (Pi-5-class / Hailo-8, the ADR-0012/0015
deployment target), not desktop-GPU marketing. Sim-side, all of these run
comfortably — the constraint is the *real* hardware the sim is standing in for.

### 2.1 Classical CV

| Approach | What it is | Strength for a small fast drone vs sky/clutter | Failure mode | Embedded cost |
|---|---|---|---|---|
| **Frame differencing / background subtraction + blob** | Subtract aligned consecutive frames; threshold; connected-component blobs are moving objects | Surfaces *tiny few-pixel* movers a detector misses; near-free CPU; no training data | Camera is **moving** → must ego-motion-compensate with a homography, which is only valid for pan/tilt/zoom or a planar background and **breaks under translation+parallax**; sky is textureless so the homography itself is fragile ([Sheikh & Javed, ICCV 2009](http://www.cs.cmu.edu/~yaser/SheikhJavedKanade_ICCV_2009.pdf); [SCIEPublish 2025 motion-region-proposal study](https://www.sciepublish.com/article/pii/491)) | Very low (a few ms/frame) |
| **KLT / optical-flow track-after-detect** | Seed feature points on a detected blob, track them frame-to-frame | Cheap continuation of a lock; gives per-point motion for a velocity estimate | Far/small targets are low-texture → too few trackable corners ([imrid tracker survey](http://imrid.net/?p=4441)); drifts without periodic re-detect | Low |
| **Correlation trackers — KCF / CSRT / MOSSE** | Given an init box, learn a correlation filter and follow the target's *appearance* | KCF/MOSSE handle low-texture small targets better than feature methods; hold a lock through brief detector blackouts | **Trackers, not detectors** — need a detect step to init and re-init (this is the "track" in detect-then-track); CSRT accurate but slow, MOSSE fast but weak | KCF ~30 fps, CSRT ~4 fps, MOSSE fastest ([imrid](http://imrid.net/?p=4441), [OpenCV Q&A](https://answers.opencv.org/question/201685/); desktop — derate on Pi) |

Reference point for "how hard is markerless drone-vs-sky": **YOLOMG** (drone-to-drone,
moving camera) reports that on the ARD100 dataset **42% of targets are <12×12 px** and
a drone at 100 m is **~10×10 px in 1080p**; their answer is exactly a
**motion-difference channel (3-frame difference + homography alignment) fused with
appearance**, reaching AP 0.85 @1280² — but only 35 fps on an **RTX 2080Ti**, no
embedded number, code released ([YOLOMG, arXiv 2503.07115](https://arxiv.org/html/2503.07115v1),
[github.com/Irisky123/YOLOMG](https://github.com/Irisky123/YOLOMG)). The takeaway: a
motion channel is the right *proposal* layer, but on its own it does not survive a
translating camera — which is why the parent design layers a classifier on top
(ADR-0015 §2, detect-then-track lifted tiny-target recall 0.405→0.861 @0.981 precision).

### 2.2 Lightweight pre-built neural detectors (the builder's ask)

| Model | Params / size | Input | Accuracy (COCO unless noted) | Embedded rate | License | "Minimal adaptation" reality |
|---|---|---|---|---|---|---|
| **YOLOv8n** | 3.15 M / ~6 MB, 8.7 GFLOPs | 640² | 37.3 mAP ([Ultralytics](https://docs.ultralytics.com/models/yolov8/)) | Pi-5 CPU ~13 fps (ADR-0015); Pi5+Hailo-8 ~35 fps end-to-end in the detect-then-track stack (ADR-0015); raw Hailo-8 HW ceiling ~431 fps ([Seeed/Hailo bench](https://wiki.seeedstudio.com/benchmark_of_multistream_inference_on_raspberrypi5_with_hailo8/)) | **AGPL-3.0** ([Ultralytics](https://docs.ultralytics.com/models/yolov8/)) — see risk R6 | Export ONNX→HEF for Hailo; **fine-tune needed** — a COCO "drone" class does not fire on a small quad vs sky |
| **YOLO11n** | ~2.6 M, ~37% less compute than v8 | 640² | ~0.61 mAP50-95 | Pi-5 CPU: OpenVINO 12.4 fps, MNN 8.6, NCNN 3.4; ~8–10 fps real pipeline, 25+ at low-res ([learnopencv](https://learnopencv.com/yolo11-on-raspberry-pi/)) | **AGPL-3.0** | Same as v8; newer, slightly leaner |
| **NanoDet-Plus-m** | **1.17 M / 1.2 MB int8**, 0.9 GFLOPs | 320² (or 416²) | 27.0 mAP@320 / 30.4@416 | ARM 11.97 ms@320 (~80 fps ceiling, Kirin-980 ncnn) ([RangiLyu/nanodet](https://github.com/RangiLyu/nanodet)) | **Apache-2.0** ✅ | Anchor-free; ncnn/OpenVINO/MNN export; COCO-pretrained; still needs a drone fine-tune |
| **MobileNet-SSD v2 / SSDLite** | ~4–5 M | 300² | COCO-tier (older) | ~4–25 fps Pi-4 int8 TFLite depending on variant ([EJ Tech](https://www.ejtech.io/learn/tflite-object-detection-model-comparison/), [Adafruit](https://blog.adafruit.com/2019/09/09/)) | Apache-2.0 ✅ | Mature TFLite path; weakest accuracy on tiny targets; fine-tune needed |

**Drone-specific fine-tunes with published weights** (the "needs minimal adaptation"
shortcut — but read the fine print):

- **doguilmak/Drone-Detection-YOLOv11x** — downloadable `best.pt`, **MIT**, fine-tuned
  on ~1,000 images, 1 class ("drone"), mAP50 0.905 / mAP50-95 0.546, 640², 8.9 ms
  **on a Colab GPU (not a Pi)** ([HF card](https://huggingface.co/doguilmak/Drone-Detection-YOLOv11x)).
  Caveat: it's the **x** (extra-large, ~57 M params) variant — great as a *recipe*
  ("fine-tune 1 drone class on ~1k images"), **not** an edge model. The card itself
  says "domain adaptation is strongly recommended for deployment in different
  environments" — i.e. it will not transfer to our Gazebo target unchanged.
- **ZhaoJ9014/Anti-UAV** — official Anti-UAV tracking benchmark + baselines, **MIT**
  ([GitHub](https://github.com/ZhaoJ9014/Anti-UAV)). Good source of drone imagery/labels.
- **Drone-vs-Bird / WOSDETC** challenge datasets + top entries (e.g. WRN-YOLO,
  [IJCNN2025-DvB](https://github.com/yjwong1999/IJCNN2025-DvB)) — useful training data.
- **MMAUD** ([ICRA-2024](https://github.com/ntu-aris/MMAUD)) — rich multi-modal, but
  **CC-BY-NC-SA (non-commercial)** — fine for a portfolio, *not* for any commercial claim.

**Honest read of "minimal adaptation."** If Emerson already has a trained lightweight
drone detector, "minimal" means: (a) re-export to the sim's resolution and to the
runtime format (ONNX for CPU, HEF for Hailo); (b) set a confidence threshold against
the sim's clean imagery; (c) wrap its `(box, score)` output in a `Measurement`
(centroid→bearing, box-width→range §3). What "minimal" does **not** remove: a nano
COCO model has no usable drone response, so *some* fine-tune on drone imagery is
unavoidable — but in the **sim** the target is one known Gazebo model against a known
sky/ground, so even a tiny fine-tune (or a strong color/shape prior) suffices. The real
adaptation cost lands on hardware (real clutter, blur, vibration), which is precisely
what the Stage-0 bench (ADR-0033 item 1) is for.

---

## 3. The range problem (a box is not a pose)

The AprilTag gave range almost for free: a known-size square + intrinsics →
`pose_t` → `‖pose_t‖`, good to ~5–8% (ADR-0015 row 8). A markerless detector gives a
**bounding box** — a bearing and an angular *size*, not a metric pose. Options, ranked:

1. **Known-size scaling (recommended for Vc/gating).** Pinhole geometry:
   `range ≈ fx · W_real / w_pixels`, where `W_real` is the target's true frontal width.
   Error is **~15–30% of range** for a real drone (ADR-0015 row 8), driven by
   uncertainty in `W_real` and noise in the box edges — and it *worsens as the box
   shrinks*: at ~10 px (YOLOMG's 100 m case), a 1-px edge error is a ~10% range error.
   **In the sim `W_real` is known exactly** (the target model's dimensions), so sim
   size-scaling is *optimistic* — we must inject the 15–30% σ as a realism knob (§4, §5)
   or the WORST tier is a lie.
2. **Range-rate from looming (a cheap bonus).** The box's *rate of expansion* estimates
   closing speed Vc more directly than differencing a noisy absolute range — useful
   because Vc, not absolute range, is what pro-nav's `a_cmd = N·Vc·λ̇` actually needs.
3. **Monocular depth NN — rejected.** Metric scale is *ill-posed* from a single image:
   "metric scale is inherently unobservable from a single view" and networks predict
   *relative* depth needing external calibration
   ([Survey on Monocular Metric Depth, arXiv 2501.11841](https://arxiv.org/html/2501.11841v3)).
   For a novel small object at unknown true size it gives no credible absolute range.
   Do not use it.
4. **Bearing-only guidance — the honest headline.** Pro-nav consumes **LOS rate λ̇**,
   an *angle* rate, for steering; range only scales Vc and sets gates (§1.3). So
   **bearing quality matters far more than range quality.** ADR-0023 makes this concrete
   and independent of range precision: the terminal miss is ~96% set by the
   zero-effort-miss (ZEM) at handoff (r²=0.99), and the levers are **acquisition range
   and mid-course track quality**, *not* terminal range accuracy. A coarse range
   (±15–30%) that only throttles Vc and trips gates is tolerable **provided β is clean**.

**Range strategy = known-size scaling for Vc/gating (carry 15–30% σ) + bearing-only
steering + looming for range-rate; never monocular depth.** This is also the more
*sophisticated* portfolio story: "I don't need good range, I need good bearing, and
here's the ZEM analysis (ADR-0023) that proves it."

---

## 4. Sim integration path

**Target model — build a tag-LESS variant.** `models/fpv_target/` exists but its
`model.sdf` header is explicit: it is the M2 AprilTag **wrapped** in a primitive quad
body, and its `tag_visual` block is copied *verbatim* from `models/apriltag_target/`
with a hard "DETECTION CONTRACT (do not break)" — i.e. it is still a fiducial target.
For markerless work, add a new model (e.g. `models/fpv_target_markerless/`) that keeps
the primitive-built body + props but **removes the tag plane**, so the seeker must find
the *body pixels* against the sky / `models/demo_ground_grid`. Keeping it a separate
model means every gated world (`worlds/apriltag.sdf`, M0–M4/S2) is untouched — the
ADR-0005 resource-path shadow precedent, same as the demo world already does.

**Honesty boundary (non-negotiable, CLAUDE.md + ADR-0010).** `gt_*` from
`/world/<name>/pose/info` (ground truth) is **scoring/logging only**; it is
structurally unreadable by guidance, which sees camera + own-state EKF. A markerless
seeker is a *new guidance path*, so it **re-earns the numeric no-cheat audit** — the
same pattern as `docs/audit_targets.md`: assert the seeker's `Measurement` never
derives from any `gt_` field, and that the CSV's `gt_range`/`gt_bearing` columns are
computed only in the scoring block. Frame math stays ENU/optical/FRD/NED per docs/goals.md.

**The swap is one function.** `GzFrameSource` (frame_source.py) already delivers frames
+ intrinsics from the tag-less world exactly as it does for the tag world; only the
detector body changes (§1.2). Wire a `--seeker {apriltag,classical,nn}` flag so the
*same* `m4_intercept.py` flight runs either seeker against the *same* mover paths — that
is what makes the A/B in §5 a paired, apples-to-apples comparison rather than two
different programs.

---

## 5. Evaluation plan

**A/B design.** Identical **paired-seed** engagements, AprilTag baseline vs each
markerless seeker, same mover paths (L→R, R→L, weave, jink, `oblique_close` — ADR-0033),
speeds 6/9/12 m/s (ADR-0029 comparability). Paired seeds are mandatory: single-flight
terminal-dropout noise is ~1 m, so a lone delta below that is noise — n≥8 paired plus a
mechanism story is the project's standing bar (CLAUDE.md; ADR-0023 discipline). Sims run
**one at a time at idle load** (batch-hygiene mandate).

**Metrics (each maps to an ADR-0015 data-constraint row so the numbers are comparable):**

| Metric | Why it matters | Baseline to beat |
|---|---|---|
| **Detection rate vs range** `P_detect(R)` | Sets **acquisition range** — the #1 miss lever (ADR-0023/0024); a markerless seeker likely acquires *later* than the fiducial | Tag detectable ~9–12 m (ADR-0015 row 10 / ADR-0024/0028) |
| **Bearing σ (deg)** | Pro-nav steers on β → λ̇; this is the quality that dominates (§3) | Fiducial sub-deg; ML ~1–2° (ADR-0015 rows 9) |
| **Range σ (% of R)** | Only scales Vc/gates; carry the realism | Tag 5–8% → ML 15–30% (ADR-0015 row 8) |
| **Track continuity at high LOS rate** | Detection drops as \|λ̇\| and blur rise; λ̇ hits 485–1870°/s near CPA (ADR-0023, row 7) | Tag holds to ~1.7 m / 0.07 s pre-CPA (ADR-0023) |
| **End-to-end miss / Pk-vs-radius** | The bottom line | ratified metric ram 0.35 m (ADR-0084, 5-inch pair) / net ~1.5 m |

**WORST-tier mandate (CLAUDE.md "simulate worse than ideal").** Three tiers
BEST/EXPECTED/WORST; the decision must survive **WORST**, and every seeker knob must map
to a **bench-measurable** quantity (the Stage-0 bench, ADR-0033 item 1, measures exactly
these five: detection Hz, max trackable \|λ̇\|, bearing σ, dropout burst length,
latency). Concretely: run the markerless A/B not just on the clean sim seeker but with
its measured/derived σ_β, σ_R, `P_detect(R)`, and LOS-rate-driven dropout injected —
reusing the `TERM_*` knobs ADR-0015 already added to the guidance stack. **Disclose the
sim's optimism:** the sim has *no motion-blur and no vibration model* (ADR-0023) and the
target is a clean primitive body, so **sim seeker Pk is an upper bound** on a real
seeker's (ADR-0024) — say so in the writeup, and let the bench be the reality check.

---

## 6. Staged recommendation (max portfolio value per unit effort)

**Build order: classical detect-then-track FIRST, NN upgrade SECOND. Bearing-only
steering + known-size range throughout. Never monocular depth.**

**Stage A — classical markerless baseline (build first).**
Motion proposal (ego-motion-compensated 3-frame difference, using PX4 own-state
yaw-rate/velocity — *legal* own-state, not `gt_` — to pre-align frames) → blob →
KCF/MOSSE correlation track → the existing α-β filter → `Measurement`. Rationale:
1. **No training data, works day one** in the benign sim (clean body vs sky, known
   ego-motion) — a *working* end-to-end markerless intercept is the deliverable that
   flips the portfolio narrative, and even a partial result is acceptable (ADR-0033).
2. **It is not throwaway** — it *is* layer 1 of the parent architecture (ADR-0015 §2:
   motion first, then classify). The NN slots on top, it doesn't replace this.
3. **De-risks the plumbing** — the `Measurement` contract, the tag-less model, the
   no-cheat audit, and the `--seeker` A/B harness all get proven *before* the NN adds a
   second variable. Cheapest path to a clean A/B vs the AprilTag baseline.

**Stage B — NN upgrade (build second).**
Drop the builder's already-built lightweight nano detector into the *same*
`markerless_detection_loop` as the classifier stage of detect-then-track (motion
proposal → **NN classify/detect** → correlation track → α-β). A/B it against **both**
the classical seeker and the AprilTag baseline on the same seeds. Then bench it on
Pi-5 + Hailo-8 (ADR-0033 item 1) — that "runs on the real embedded target with a
measured sim-vs-bench gap" is the credibility multiplier interviewers reward (ADR-0033).
Prefer an **Apache/MIT** model (NanoDet-Plus, or a drone MIT fine-tune) over AGPL YOLO
for a public repo unless the builder accepts AGPL terms (R6).

**Why not NN-first:** it front-loads the training-data dependency and the
license/export questions before the interface and honesty-audit scaffolding is proven,
and it skips the motion layer the NN needs anyway. Classical-first is strictly the
lower-risk ordering with an earlier working demo.

---

## 7. Risks

- **R1 — moving-camera ego-motion (the core classical risk).** Homography alignment is
  only valid for pan/tilt/zoom or planar scenes and breaks under
  translation+parallax; sky is textureless so the homography is itself fragile
  (Sheikh & Javed; §2.1). *Mitigation:* pre-compensate with PX4 own-state
  (yaw-rate/velocity — legal own-state), and lean on the NN appearance channel where
  motion alone fails (this is exactly why YOLOMG fuses the two).
- **R2 — acquisition range regresses (the one that can hurt the miss).** A markerless
  detector on a few-pixel body (YOLOMG: 10 px @ 100 m) likely acquires *later* than the
  fiducial. Since acquisition range is the dominant miss lever (correction capacity
  ∝ t_go²; acquiring at 12 m vs 6.5 m moves capacity 0.72→~4.3 m, ADR-0023/0024), a
  seeker that sees the target *later* directly worsens ZEM and miss. **Measure
  `P_detect(R)` against the tag baseline first** — expect the seeker, not the guidance,
  to be the honest bottleneck (ADR-0015 existential risk #5).
- **R3 — sim-to-real optimism.** No blur/vibration model in the sim (ADR-0023) and a
  clean primitive target → **sim seeker Pk is an upper bound** (ADR-0024). Disclose;
  the Stage-0 bench (ADR-0033) is the reality check.
- **R4 — range σ 15–30% vs tag 5–8%** (ADR-0015 row 8). Largely mitigated by
  bearing-only steering (§3), but it *does* loosen Vc/gating; verify the gates still
  latch under WORST-tier σ_R.
- **R5 — track continuity through CPA.** Detection drops with \|λ̇\| and blur near CPA
  (ADR-0015 row 7); the tag already holds only to ~1.7 m / 0.07 s pre-CPA (ADR-0023).
  A weaker markerless appearance lock may drop earlier — but per ADR-0023 the terminal
  hold is a ~2% lever, so this is a robustness concern, not a miss driver.
- **R6 — license (portfolio-relevant).** YOLOv8/YOLO11 are **AGPL-3.0** (network-copyleft;
  publishing a repo that uses them can obligate open-sourcing the whole stack). Prefer
  **Apache-2.0 (NanoDet-Plus, MobileNet-SSD)** or an **MIT** drone fine-tune for a public
  portfolio, or accept AGPL knowingly. Flag: **MMAUD is CC-BY-NC-SA (non-commercial)** —
  usable for a portfolio, not a commercial claim.
- **R7 — single-target assumption.** Guidance consumes `detections[0]` only; fine for
  the one-target sim, but a real sky has birds/clutter → the "drone-vs-bird" classifier
  (ADR-0015 §2) is what disambiguates, and is out of scope for the sim-only milestone.

---

## 8. Proposed ADR skeleton (for the main session to ratify)

> **ADR-00XX — Markerless seeker: classical detect-then-track first, NN upgrade second,
> bearing-only guidance.**
>
> **Context.** Post-M5 queue item 2 (ADR-0033): replace the AprilTag target-lock with a
> markerless seeker feeding the SAME `Measurement` interface (§1), attacking the #1
> disclosed risk (ADR-0015 #5). Scope widened to include a pre-built lightweight NN
> alongside classical CV. docs/goals.md's "no ML" rule was about *isolating* guidance; this
> milestone deliberately un-isolates it, and the parent architecture (ADR-0015
> Pi5+Hailo detect-then-track) already assumes an ML detector.
>
> **Options considered.**
> - **A — Classical-first**, NN as a later upgrade layer. + works day-one, no training
>   data, de-risks the interface/audit, *is* ADR-0015 layer 1. − weaker on a translating
>   camera (R1).
> - **B — NN-first** (nano detector as the primary seeker). + best raw accuracy. −
>   front-loads training-data + AGPL/export questions, skips the motion layer the NN
>   needs anyway.
> - **C — Full detect-then-track from the start** (motion + NN + tracker together). +
>   the eventual target architecture. − most moving parts before any A/B exists.
> - Range sub-options: known-size scaling / monocular-depth NN / bearing-only.
>
> **Decision.** **Stage A** classical motion→blob→correlation-track→α-β markerless
> seeker on a new tag-less target model, A/B'd vs the AprilTag baseline (paired seeds,
> n≥8, §5); **Stage B** drop the pre-built nano NN in as the detect-then-track classifier
> and A/B vs both, then bench on Pi5+Hailo (ADR-0033 item 1). **Range = known-size
> scaling (carry 15–30% σ) + bearing-only steering + looming for range-rate; monocular
> depth rejected.** Prefer Apache/MIT weights over AGPL for the public repo.
>
> **Why.** The miss is bearing/ZEM-limited, not range-limited (ADR-0023), so bearing-only
> is the correct and more sophisticated framing; classical-first delivers a working
> markerless intercept earliest and its components are reused (not thrown away) by the NN
> stage; the WORST-tier + bench discipline keeps the Pk claim honest (sim Pk is an upper
> bound, ADR-0024). Partial result acceptable (ADR-0033).
>
> **Honesty boundary.** New guidance path → re-earn the numeric no-cheat audit
> (`gt_*` scoring-only, `docs/audit_targets.md` pattern); disclose the no-blur/no-vibration
> sim optimism.

---

### Sources

Repo/ADR: `docs/goals.md`; `docs/decisions.md` ADR-0005/0010/0012/0015/0023/0024/0025/0028/0029/0033;
`scripts/m3_static_intercept.py` (`Measurement`, `detection_loop`);
`scripts/m4_intercept.py` (filter gains, LOS/pro-nav consumption); `scripts/frame_source.py`;
`docs/audit_targets.md`; `docs/perception_design.md`; `.claude/skills/pronav`.

Web (accessed 2026-07): [Ultralytics YOLOv8](https://docs.ultralytics.com/models/yolov8/) ·
[YOLO11 on Raspberry Pi (learnopencv)](https://learnopencv.com/yolo11-on-raspberry-pi/) ·
[Seeed/Hailo-8 multistream benchmark](https://wiki.seeedstudio.com/benchmark_of_multistream_inference_on_raspberrypi5_with_hailo8/) ·
[RangiLyu/NanoDet](https://github.com/RangiLyu/nanodet) ·
[TFLite object-detection comparison (EJ Tech)](https://www.ejtech.io/learn/tflite-object-detection-model-comparison/) ·
[doguilmak Drone-Detection-YOLOv11x (HF)](https://huggingface.co/doguilmak/Drone-Detection-YOLOv11x) ·
[ZhaoJ9014/Anti-UAV](https://github.com/ZhaoJ9014/Anti-UAV) ·
[MMAUD (ICRA-2024)](https://github.com/ntu-aris/MMAUD) ·
[Drone-vs-Bird WRN-YOLO](https://github.com/yjwong1999/IJCNN2025-DvB) ·
[YOLOMG (arXiv 2503.07115)](https://arxiv.org/html/2503.07115v1) · [YOLOMG code](https://github.com/Irisky123/YOLOMG) ·
[Sheikh & Javed, background subtraction for freely moving cameras (ICCV 2009)](http://www.cs.cmu.edu/~yaser/SheikhJavedKanade_ICCV_2009.pdf) ·
[Motion-region-proposal for small-drone detection (SCIEPublish 2025)](https://www.sciepublish.com/article/pii/491) ·
[OpenCV tracker survey (imrid)](http://imrid.net/?p=4441) ·
[OpenCV tracker FPS (OpenCV Q&A)](https://answers.opencv.org/question/201685/) ·
[Survey on Monocular Metric Depth Estimation (arXiv 2501.11841)](https://arxiv.org/html/2501.11841v3).
