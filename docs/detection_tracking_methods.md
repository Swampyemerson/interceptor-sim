# Detection & tracking methods — the seeker perception stack

> **What this doc is.** The design + research record for how the interceptor **sees** the
> target: what detects it, what tracks it between detections, how the detector is trained
> from real data, and which perception levers actually move **acquisition range and
> detection density** (the quantities that buy terminal time-to-go). Written against the
> **ordered** hardware (innomaker OV9281 mono global-shutter + Pi 5 8GB, Hailo HAT
> deferred — `docs/hardware_order_list.md` §0c).
>
> **What this doc is NOT.** No flight plans, no guidance laws, no launch mechanism, no
> field bring-up procedure — those are siblings (`docs/flight_plan_candidates.md`,
> `docs/launch_mechanism_plan.md`, `docs/field_bringup.md`, `docs/tripod_test_protocol.md`).
> Canonical project state stays `docs/project_state.json`; this doc is subordinate to it.
>
> **Reading order for the numbers:** every quantitative claim below cites a run, a file, or
> a written derivation. Claims are tagged **[MEASURED]**, **[DERIVED]**, **[VENDOR]**, or
> **[HYPOTHESIS]**. Nothing here is a verdict that hasn't been flown or computed.
>
> *Author: Opus 5 seeker-perception pass, 2026-07-24. Not committed to the contract — the
> head owns `project_state.json` updates.*

---

## 0. Terms, once each (the builder is learning — skip if these are familiar)

| term | one-line meaning | why it matters here |
|---|---|---|
| **detection recall** | of all frames where the target really is there, the fraction the detector finds | the seeker's core score; "0.8% in flight" is a recall number |
| **precision** | of all boxes the detector emits, the fraction that are really the target | low precision = phantoms competing for the handoff |
| **false-fire rate** | fraction of *drone-free* frames where the detector fires anything | the outdoor-clutter risk (sagebrush, juniper, birds) |
| **IoU** (intersection-over-union) | box overlap score, 0–1; 1.0 = identical boxes | how we proved the auto-labeler is trustworthy (0.965) |
| **AP50** | average precision at IoU ≥ 0.5 — a single summary number for a detector | used to rank models offline; NEVER an acceptance gate here |
| **hard negatives** | training images with **no** target that look tempting (props, bushes, birds) | teaches the net to shut up; 15–20% of the set (the rebal lesson) |
| **held-out split** | data the model never trained on, used to score it | ours must be held out by **whole flight/session**, never random frames |
| **shadow mode** | the detector runs and *logs*, but does not steer anything | measures real recall/phantoms at zero flight risk |
| **bearing-σ** | the standard deviation of the angle-to-target the seeker reports | the noise the guidance filter has to live with |
| **R_acq** | the range at which the seeker first reliably acquires | sets time-to-go, which sets terminal correction capacity |
| **t_go** | time-to-go: seconds left until closest approach | correction capacity grows as t_go² (ADR-0027) |
| **streak burn** | the range consumed while confirming a detection streak before handoff | acquisition you don't get to keep |
| **INT8 quantization** | shrinking a network's math from 32-bit floats to 8-bit ints so an NPU can run it | required to run on the Hailo HAT; can silently kill small-target recall |
| **letterbox** | resizing a frame into the net's square input, padding the rest | why a 1280×800 frame becomes a 640×640 tensor (`finetuned_seeker._letterbox`) |

---

## A. The perception problem, restated honestly

### A.1 In simulation, the detector is NOT the binding wall — pointing is

Four measurements settled this (all re-scored with the *fixed* `box_scoring.box_hits_gt`,
after the scoring artifact that produced the 5th mirage was found and quantified):

| measurement | value | source |
|---|---|---|
| in-flight approach recall, ≥8 m | **0.8%** (2 real vs 1187 phantom detections, 63 flights) | `scripts/seeker/approach_recall.py`; ADR-0076 add #18i/#18k |
| same detector, camera static, set-pose 8–22 m, both aspects | **~100%** | set-pose sweep 2026-07-16, ADR-0076 add #18k |
| terminal ticks with the target **in frame at all**, 8–12 m band | **25%** (43/171) — 75% out of frame | `docs/inview_probe_results.md` |
| deployed `detect()` recall **when the target is in view**, 8–12 m | **70%** (30/43); any-box ceiling **81%** (35/43) | `docs/inview_probe_results.md` Probe 2 |

Both candidate in-view failure mechanisms were then eliminated **in sim**:
ground-clutter background was **refuted** (recall GROUND 100%/99% ≈ HORIZON 100% ≈ SKY 92%
at every 2 m bin 6–20 m, level and banked — Probe 1), and phantom competition was
**confirmed but modest** (a phantom masks the real target in only 12% of in-view frames;
top-3 recovers 89% of localizable frames; a score-margin gate flips 80% of the masked ones).

And the pointing fix behaves exactly as that decomposition predicts: the Phase-A Gazebo A/B
lifted **8–12 m real-detection recall 3.0% (2/67) → 35.4% (28/79)**, ~12×, on paired seeds
(`scripts/experiments/loft_dive/gazebo_results.md`) — a *geometry* change, no detector change.

**So: no amount of sim-detector tuning is the lever.** That is why every sensor-side sim
lever returned NULL (v3 retrain, rebal retrain, auto-crop, resolution, subpixel bearing —
see §E.3 graveyard).

### A.2 What the sim structurally *cannot* test — and where the detector's real job is

Three gaps are unmeasurable in Gazebo, by construction:

1. **Terrain clutter.** The sim ground is a flat gray plane. Probe 1's "ground background
   is fine" verdict is explicitly **sim-scoped** (`docs/inview_probe_results.md`, and the
   `detector` stage note in the contract). Sagebrush/juniper/basalt is a different
   distribution.
2. **Motion blur.** `gz_x500_mono_cam` has **no blur model** (ADR-0076 add #18 changelog,
   2026-07-18: "Motion blur is a real-world risk, NOT a sim mechanism"). Terminal LOS rates
   are 485°/s mean, 600–1870°/s peak (`docs/terminal_diagnosis.md`).
3. **Real appearance + real sensor.** A real 5" quad through a real mono global-shutter lens,
   with real dynamic range (bright desert ground out-brightening sky).

And the sim→real transfer gap is no longer a hypothesis — it is **[MEASURED]**:

> On a source-disjoint held-out set of **real** imagery (n=4,175 frames / 4,315 GT boxes,
> grayscale, conf 0.25), the **deployed** sim-trained `drone_finetuned_quad_v2` scores
> **AP50 0.0003, recall 1.1%, precision 0.5%, and false-fires on 88.5% of drone-free
> frames** (2.21 fires per negative frame). Recall below 24 px — i.e. the entire terminal
> band — is **0.000**.
> — `logs/nn_tier/eval_s-mono_summary_cmp1.csv`, `docs/nn_tier/PLAN.md`

**The deployed detector is blind on real pixels.** That is the honest statement of the
perception problem: the sim wall is pointing, the *real* wall is a domain gap that only real
data closes.

### A.3 Why this is the "smallest intercept distance" lever, with the ZEM caveat attached

The caveat governs (contract `narrative.caveat`, ADR-0027): at a 9 m/s crossing the delivered
zero-effort-miss at handoff is ~4 m against ~0.27 m of terminal correction capacity from
today's geometry. Capacity = ½·a·t_go², so it grows with the **square** of time-to-go:
pushing handoff from ~6.5 m out to ~12 m lifts capacity **0.72 m → ~4.3 m**. Perception's
contribution to the intercept is therefore **R_acq and detection density**, i.e. *how early
and how continuously* the seeker can hand off — **not** bearing precision (the subpixel /
centroid bearing lever is a measured NULL at n=31, ADR-0071) and not holding the target
through CPA (that channel is worth −0.03 m, `docs/terminal_diagnosis.md`).

Perception can only *deliver* time-to-go. Whether the flight profile spends it is the
sibling docs' problem.

### A.4 The one arithmetic that connects recall to acquisition range (read this)

The handoff needs a **streak** of consecutive fresh detections (5 pre-registered in R5 /
`tripod_score.py:DEFAULT_HANDOFF_STREAK`; the sim's `m4_intercept.py` minimum is 3). Confirming
that streak burns range:

```
t_go = (R_acq − R_streak_burn) / V_closing         [tripod_score.gate_verdict]
R_streak_burn = (frames_to_confirm / fps) × V_closing
```

**Correction (2026-07-24, from reading the code — this doc's first draft overstated it).**
`tripod_score.gate_verdict` does **not** assume p = 1.0: it uses a **mean-rate** model,
charging the 5 detections at the measured decode rate (`decode_Hz = p × stream_fps`, so
`frames_to_confirm ≈ k/p`). The spec derivation in `viewpoint_and_deploy_spec.md` B4 *is* the
p = 1.0 form (5 frames at 30 fps → 1.5 m). **Both are still optimistic**, because the handoff
needs k **consecutive** detections, not k detections on average. For a per-frame recall `p`,
the expected number of frames to first complete a run of k successes is the standard result

```
E[T] = (1 − p^k) / (p^k · (1 − p))                  [DERIVED — classic Bernoulli run length]
```

**[DERIVED]** k = 5, V = 9 m/s, 30 fps (so 0.30 m of closure per frame):

| per-frame recall p | E[frames], run-length | streak burn @30 fps, 9 m/s | R_acq needed for t_go ≥ 0.5 s | mean-rate model (k/p) | optimism factor |
|---:|---:|---:|---:|---:|---:|
| 1.00 (B4's assumption) | 5.0 | 1.5 m | **6.0 m** | 5.0 | 1.00× |
| 0.90 | 6.9 | 2.1 m | 6.6 m | 5.6 | 1.25× |
| 0.80 | 10.3 | 3.1 m | 7.6 m | 6.2 | 1.64× |
| **0.70** (measured in-view sim recall) | **16.5** | **4.95 m** | **9.5 m** | 7.1 | **2.31×** |
| 0.60 | 29.7 | 8.9 m | 13.4 m | 8.3 | 3.56× |
| 0.50 | 62.0 | 18.6 m | 23.1 m | 10.0 | 6.20× |
| **0.4417** (n-mono held-out recall on real public imagery) | **104.7** | **31.4 m** | **35.9 m** — unformable | 11.3 | **9.25×** |

The last two columns are the honest size of the correction: the gate's mean-rate model is
**optimistic by 2.3× at p = 0.7 and 9× at p = 0.44**, always in the dangerous direction for a
purchase gate. A TODO carrying this formula is now attached to `gate_verdict` itself; it is
**not** applied, because changing it can flip a published PASS/FAIL on the ~$740 order and
that must be a logged decision, not a quiet patch.

*(Reproduce: `python3 scripts/seeker/streak_burn_derivation.py`; output archived at
`logs/streak_burn_derivation_20260724.txt`. Pure arithmetic — no sim, no data.)*

Two consequences, and they are the sharpest perception findings in this doc:

- **Per-frame recall enters the acquisition budget non-linearly.** Dropping from p = 1.0 to
  p = 0.7 costs 3.5 m of effective acquisition range at 30 fps; at p = 0.44 a strict
  5-consecutive rule essentially never forms inside the engagement.
- **Therefore "detection density" is not a soft metric — it is the acquisition-range term.**
  A model that gains 10 points of per-frame recall buys more effective R_acq than any lens
  or sensor change on the table (and resolution-for-range is graveyarded anyway, §E.3).

Honest bound on the model: real detections are **temporally correlated**, not independent
Bernoulli — bursty good stretches make long runs more likely than the table says, and
out-of-frame stretches make them less likely. The table **sizes expectations**; the real
number is the observed streak-formation range from shadow logs (§D.6). Do not quote the
table as a measurement.

**Design consequence (proposal, pre-registration in §E.2 lever 3):** an **M-of-N** confirm
rule (e.g. 5 of the last 8 fresh detections) instead of 5-consecutive. **[DERIVED]** at
p = 0.4417, P(≥5 hits in an 8-frame window) = **0.245**, so ~4.1 independent windows ≈ **33
frames ≈ 1.1 s ≈ 9.6 m** of closure at 30 fps / 9 m/s — versus **31.4 m** for the strict rule.
(A *sliding* window is more favourable still, ~11 frames; 33 is the conservative bound.)
The cost is symmetric and must be measured, not assumed: an M-of-N rule also makes it easier
for a **clutter false-fire cluster** to manufacture a handoff. That trade is measurable
offline on drone-free tripod sweeps (§D.6) and belongs to the `handoff` stage, so it is
raised here as a perception-driven proposal for the head, not adopted.

---

## B. Detector architecture — the decision for the ordered hardware

### B.1 Where the code actually is today

| piece | file | state |
|---|---|---|
| deployed sim seeker weights | `scripts/seeker/weights/drone_finetuned_quad_v2.onnx` | YOLO11n-class, @640, conf 0.25, color-sim-trained; **blind on real imagery** (§A.2) |
| real-hardware seeker loop | `flight/deploy/seeker_loop.py` | ONNX detect → bearing → LOS derotation → alpha-beta → pro-nav → MAVSDK; `--weights` defaults to **quad_v2** (line ~881) |
| ONNX inference wrapper | `scripts/seeker/finetuned_seeker.py` | letterbox → ONNX → boxes; infers input size from the model; **no grayscale conversion** |
| real-data model family | `scripts/seeker/weights/nn_tier/{n-mono,n-mono-aug,s-mono,n-color}.onnx` | trained 2026-07-21 on the 15,391-image real-media corpus |
| tripod two-curve scorer | `scripts/seeker/tripod_score.py` | `--weights` defaults to **quad_v2** (line 115) |

### B.2 The model call: YOLO11n @640, grayscale, COCO-init — `n-mono` is the anchor

The size question was answered from **both** directions on the same source-disjoint held-out
split (n = 4,175; grayscale; conf 0.25; `logs/nn_tier/heldout_scores.txt`,
`docs/nn_tier/PLAN.md`) **[MEASURED]**:

| model | AP50 | recall@.25 | precision@.25 | false-fire (drone-free) | ONNX size |
|---|---:|---:|---:|---:|---:|
| `v2_deployed` (sim-trained) | 0.0003 | 1.1% | 0.5% | 88.5% | 10 MB |
| **`n-mono` (YOLO11n, gray)** | **0.442** | **44.2%** | **71.4%** | **4.9%** | **10 MB** |
| `n-mono-aug` (heavy augmentation) | 0.388 | 41.4% | 66.9% | 5.1% | 10 MB |
| `s-mono` (YOLO11s, 4× params) | 0.419 | 44.2% | 65.6% | 9.8% | 38 MB |

Scaling **up** did not help (s-mono ties recall, worse precision, 2× the false-fire, 4× the
size); augmentation was a clean NULL. Mono-vs-color was closed separately: native-color buys
**nothing** for bird discrimination (3.07% vs 3.41% false-fire, z = 0.23, p = 0.82) and is
**significantly worse** on the deployment-relevant desert plates (14.6% vs 4.6%, p < 1e-6) —
`docs/nn_tier/mono_vs_color_bird_ab.md`, ADR-0078.

So: **keep YOLO11n @640, grayscale, COCO-init.** No architecture search is warranted; the
open variance is *data*, not *architecture*. Honest bound: 44% recall is **moderate** on hard
public small-drone imagery, it is a frame metric on public media, and it is **not** our
target / our camera / our site.

### B.3 The compute constraint that shapes everything: Pi 5 CPU first, Hailo deferred

| path | rate | status |
|---|---|---|
| AprilTag decode, Pi 5 CPU | **~30 fps real-time** | constraint `pi5-compute` (anchor; bench-gated) |
| YOLO11n @640, Pi 5 **CPU** | **~5–10 fps** | constraint `pi5-emulation-gap` (anchor ~13 fps YOLOv8n on desktop-class CPU, ADR-0015) |
| YOLO11n @640 INT8, **Hailo-8L** | 157 chip fps **[VENDOR]** → **~40–80 fps end-to-end** est. | `docs/nn_tier/viewpoint_and_deploy_spec.md` B2–B3; BENCH-GATED |

Feed those into §A.4's arithmetic **[DERIVED]** (k = 5 consecutive, p = 1.0 assumed, so these
are *floors*):

| fps | streak burn @9 m/s | R_acq needed (t_go ≥ 0.5 s) | @20 m/s head-on |
|---:|---:|---:|---:|
| 5 (CPU worst) | 9.0 m | 13.5 m | 30.0 m |
| 8 (CPU typical) | 5.6 m | 10.1 m | 22.5 m |
| 30 (the derived bar) | 1.5 m | **6.0 m** | 13.3 m |
| 60 (HAT, mid estimate) | 0.75 m | 5.3 m | 11.7 m |

**The HAT is worth ~4 m of effective acquisition range at 9 m/s (~9 m at 20 m/s) without
touching the model** (8 fps CPU ⇒ needs 10.1 m; 30–60 fps HAT ⇒ needs 6.0–5.3 m) — that is the
honest framing of the $70 purchase, and it is a stronger argument than any accuracy claim.
*(Same derivation script; `logs/streak_burn_derivation_20260724.txt` §B.3 block. The fps values
are VENDOR/anchor estimates and stay BENCH-GATED — constraint `pi5-emulation-gap`.)*

### B.4 ADR-lite D-1 — the staged detector path (the call)

**Context.** First real kills fly the AprilTag baseline (constraint `pi5-compute`; BOM §0c).
The markerless net is measured-blind on real pixels and needs data we don't have yet.

**Options.** (a) markerless-first (buy the HAT now, retrain, fly it); (b) tag-only forever;
(c) **staged: tag-guided kills, YOLO in shadow, YOLO-guided after shadow validation.**

**Decision: (c).** Three rungs, each with its own gate:

- **Stage 0 — AprilTag baseline seeker guides.** Runs real-time on Pi 5 CPU, needs no HAT,
  and the tag decode is 8-bit grayscale natively (mono is its home modality, ADR-0078).
  **Scope limit, from the graveyard and non-negotiable:** the AprilTag is *dead as a terminal
  seeker in a crossing dash* — the directional ~6 m tag went **0 detections** through a
  16 m/s crossing dash (ADR-0076 add #18e). So Stage 0 is scoped to the **straight-leg /
  low-speed rungs of the speed ladder**, and a Stage-0 kill must **never** be reported as
  validating a crossing-dash intercept. (This is the one place where the contract's "first
  kills fly the tag" and the graveyard's "tag is invisible in a crossing dash" have to be
  read together; they are compatible only under this scoping.)
- **Stage 1 — YOLO in SHADOW MODE, on CPU, during Stage-0 flights.** The detector runs and
  writes a log; **nothing downstream reads it**. Output: real bearing-σ, real false-fire
  density per minute, real R_acq, real streak-formation range, real per-frame recall vs
  range × position-in-frame. **Key enabling insight: shadow mode does NOT need the Hailo
  HAT.** 5–10 fps is unusable for guidance but perfectly adequate for *measurement*, because
  there is no control-latency requirement on a log — and every per-frame statistic
  (recall, precision, false-fire) is fps-independent. Only the fps-dependent quantities
  (streak burn, R_acq-at-handoff) must be re-derived for the deployed rate, which §A.4's
  formula does analytically. **So Stage 1 costs $0 of new hardware and de-risks the $70.**
- **Stage 2 — YOLO guides**, only after Stage 1's logs clear the §D.6 acceptance gate *and*
  the Tier-2 Hailo bench (`docs/nn_tier/PLAN.md` §3 Tier-2: compiles to `.hef`, chip ≥60 fps,
  ≥30 fps sustained thermal-loaded end-to-end, INT8-vs-fp32 delta re-measured on the ≤40 px
  bins where quantization bites first).

**Why.** It converts every open perception question into a measurement taken at zero flight
risk, in the exact order the money gates need them, and it makes the HAT purchase evidence-
driven rather than hopeful.

**Risk / what would break it.** If the tag baseline cannot itself acquire (curve (a) fails
the money gate), Stage 0 has no flights to shadow — then shadow mode moves onto the tripod
sessions and the bench instead of onto flights, and the interceptor order is the thing that
pauses (that is exactly the `⛔ MONEY GATE` logic in `docs/hardware_order_list.md` §0d).

### B.5 ADR-lite D-2 — stale deployment weights (a live defect, cheap to fix)

**Finding [MEASURED + code-read].** Two real-hardware entry points default to the
**known-blind** sim model:

- `flight/deploy/seeker_loop.py` (~line 881): `--weights` default
  `scripts/seeker/weights/drone_finetuned_quad_v2.onnx`
- `scripts/seeker/tripod_score.py:115`: `DEFAULT_WEIGHTS = .../drone_finetuned_quad_v2.onnx`
- `docs/tripod_test_protocol.md` §7.2 step 1 likewise instructs "run the deployed detector
  (`drone_finetuned_quad_v2.onnx` @640, conf 0.25)".

All three were written 2026-07-20 (`git log`: `69ab3f2`, `87c62da`); the n-mono held-out
result landed 2026-07-21 (`8fa98c8`). They are simply *older than the evidence*.

**Why it matters.** Scoring tripod curve (b) with quad_v2 produces a near-guaranteed-zero
recall curve (AP50 0.0003 on real imagery) which, read naively, would **wrongly damn the
markerless phase and the Hailo purchase**. And on hardware, `seeker_loop.py`'s default would
run a blind detector on real pixels.

**Decision.** (1) Re-point both defaults to `scripts/seeker/weights/nn_tier/n-mono.onnx`;
(2) keep quad_v2 available as the explicit *historical bar* and score curve (b) with **both**
(the n-mono-vs-v2 delta on our own frames is itself a result worth having); (3) add a
grayscale-preprocessing flag to the inference wrapper. **Note on (3):**
`finetuned_seeker.py` does **not** convert to gray, and `n-mono` was trained natively in
grayscale — feeding it a *color* frame (sim renders, a UVC webcam on the bench) is
out-of-distribution and will read falsely bad. Real OV9281 frames are mono already (a
single-channel PNG loads as replicated gray through OpenCV, which is exactly
`nn_tier/baseline_eval.to_gray3`'s convention), so the flag matters for **bench/sim replay**,
not for the flight camera. *(Implementation is a small, spec'd edit — deliberately not made
in this doc's pass; flagged for the head.)*

### B.6 Alternatives considered, briefly

- **Two-stage classical proposal → NN verify** (`scripts/seeker/two_stage_seeker.py`,
  `markerless_loop.py`): built and superseded by NN-only-every-frame; the standalone
  classical dark-blob stage self-locked on own-prop pixels (`seeker_prototype_results.md`
  §3.4). Keep as a code path, not a candidate.
- **Bigger nets (YOLO11s/m, the MIT YOLOv11x)**: s-mono measured worse (B.2); yolo11m is
  15–25 fps est. even on the HAT (FAILS the bar); the 56.9 M-param MIT net stays an **offline
  teacher / auto-label assistant** only (`docs/nn_tier/PLAN.md` §2).
- **Open-vocabulary / foundation detectors (Grounding-DINO class)**: sanctioned as a
  *negative-set enricher and label cross-check* only — not deployable at 13 TOPS
  (`docs/real_data_pipeline.md` alternatives table).
- **Event/DVS camera, second camera, gimbal, terminal range sensor**: all rejected with ZEM
  math in `docs/seeker_upgrades.md` (they attack the CPA-hold channel worth −0.03 m).

---

## C. Tracker — NN every frame stays; the estimator is the gap-bridger

### C.1 What the sim measured

- **CSRT detect-then-track** was *adopted* for the flat-billboard target (ADR-0058:
  camera-terminal jink 14/15 vs plain 1/8) and then **dropped for the 3D quad** — NN-only
  won (`project_state.json` detector stage, "detect-then-track/CSRT dropped for the quad,
  add #2"). Mechanism: with a phantom storm (median 17 boxes/frame in the 8–12 m band,
  `inview_probe_results.md`) the tracker's *seed* is the weak link — a tracker locked onto a
  phantom is worse than no tracker, because it is temporally *stable* about being wrong.
- **VIT tracker**: 0/8. `scripts/seeker/weights/object_tracking_vittrack_2023sep.onnx` is
  on disk; the arm is closed.
- The design intent behind detect-then-track is documented at the top of
  `scripts/seeker/detect_track.py` (worth reading for the honesty-boundary reasoning: the
  tracker sees pixels only, and post-handoff it may not consult any cue).

### C.2 The real-hardware reconsideration — honest, and still "no"

Two conditions exist on hardware that the sim did not have:

1. **A slow detector in the HAT-less phase** (5–10 fps). A tracker interpolating between
   detections would raise the *published* measurement rate toward 30 Hz.
2. **Motion-blur dropouts** the sim cannot generate.

Both arguments are weaker than they look:

- On (1), a correlation tracker competes for the **same Pi 5 CPU** the detector is starving
  on; CSRT on a 1280×800 frame is itself a ~10–30 fps CPU job, so the net rate gain is small
  and the thermal/latency cost is real. And the seeker already has a *free* gap-bridger: the
  **alpha-beta LOS filter** (`flight/estimator.py`) coasts through missing detections in
  angle space, at zero pixel cost — that is the architecture `markerless_loop.py` documents
  ("a missing detection is coasted, never a confident-wrong bearing").
- On (2), blur degrades a **template/correlation tracker at least as fast as a detector** —
  appearance correlation is exactly what blur destroys. A tracker is not a blur remedy; a
  ≤1 ms exposure is (§E.2 lever 4).

**ADR-lite D-3 — Decision: NN-every-frame stays; no visual tracker is built now; the
alpha-beta estimator remains the gap-bridger.** **[Confidence: high for sim, medium for
hardware — the hardware evidence does not exist yet.]**

**What would reopen it (pre-registered, so this isn't a permanent veto).** All three must
show up in Stage-1 shadow logs:
1. Detection **gaps ≥3 consecutive frames** at ranges >8 m **while the target is in frame**
   (position-in-frame logged) and **not** blur-explained (exposure logged per frame by
   `pi_capture.py`) — i.e. genuine appearance dropouts a tracker could bridge;
2. those gaps materially delaying streak formation (measured streak-formation range worse
   than §A.4's prediction for the measured p);
3. measured Pi 5 CPU headroom for a tracker at the deployed detector rate (bench, thermal-
   soaked).
If (1) and (2) hold but (3) fails, the correct answer is the **HAT**, not a tracker.

---

## D. Real-data capture → retrain protocol (the concrete program)

This is the operational core: what to capture, how it gets labeled for free, how it is split,
how it is trained, and what number lets it graduate. It reuses the built rails —
`docs/real_data_pipeline.md` (design), `docs/nn_tier/PLAN.md` §4 (wiring), and the tripod
protocol (field logistics, not repeated here).

### D.1 Capture — sessions, from the final mount geometry

- Tool: **`scripts/seeker/pi_capture.py`** (BUILT, self-test 17/17). A session is
  `SESSION_DIR/frames/*.png` + `index.csv` (path, monotonic timestamp, **actually applied**
  exposure/gain) + `meta.json` (`exposure_meets_spec`) + optional `tags.csv` (live decode).
  `autolabel_from_apriltag.py` and `tripod_score.py` consume this layout directly.
- **One directory per flight/session — the session id IS the split group** (`nn_tier/PLAN.md`
  §4.1). This is not bookkeeping; it is what makes §D.3's anti-mirage gate possible.
- **Shoot from the FINAL mount geometry** (camera-forward position **and** the chosen up-tilt),
  never a convenient bench pose — measured v2 box-precision degrades at a moved viewpoint
  (ADR-0076 add #16; `real_data_pipeline.md` premise note).
- **Content weights** (design estimates from engagement geometry, not measurements —
  `docs/nn_tier/viewpoint_and_deploy_spec.md` A4/A5): ≥70% near-level **edge-on** elevation
  aspects (the thin 4:1 sliver a co-altitude 5" quad presents); azimuth 0–30° nose-on 0.30 /
  30–60° 0.35 / 60–90° 0.25 / rear 0.10; frame position biased **center-high** with edge
  partials included. Ground-observer-looking-up imagery is the **wrong** elevation aspect —
  weight it low.
- **Approaching passes at speed are the one thing that must not be skipped.** A hover/static
  set aces frame-eval and fails in flight — that is precisely the v3/rebal trap
  (ADR-0061; `real_data_pipeline.md` stage 1).
- **Negatives, 15–20% of the set** (the rebal lesson): drone-free background sweeps from the
  same spots and same tilt, **plus prop-in-frame hard negatives grabbed during the props-on
  bench spin** (ADR-0076 add #16 follow-up), plus birds of opportunity.
- **Log camera frame-arrival rate separately from detection rate** — a vibrating boom's
  CSI-2 flex can drop frames in a way that *mimics* detection dropout (constraint
  `forward-camera-boom` item 5).

### D.2 Auto-label — the tag writes the labels, and its limits are known

- Tool: **`scripts/seeker/autolabel_from_apriltag.py`**. Tag pose → 3-D position → projected
  YOLO box, reusing the sim's projector. **[MEASURED, validated 2026-07-21 in the apriltag
  sim world]**: auto-label box vs gt box **IoU mean 0.965 / min 0.951**, tag-pose range error
  **+0.13 m mean / 0.25 m max**, CLI label count = decode count exactly
  (`docs/placard_sizing.md`, `scripts/seeker/validate_autolabel_sim.{py,sh}`).
- **Placard: 0.35 m edge** — the carry limit, and the only size clearing the t_go gate under
  realistic OV9281 scaling (`docs/placard_sizing.md`). Sim decode envelope
  **R_decode90 = 12 m** at the deployed `quad_decimate=2.0` (18 m at full-res).
- **Two hard rules, both already enforced in the tool:** (a) on target-**present** footage,
  tag-miss frames are **DROPPED**, never labeled background — labeling them background trains
  the net to *not* see the approaching target beyond the tag's decode range, manufacturing
  the exact wall we are trying to remove; (b) `--negatives-from-untagged` runs **only** on
  explicitly target-free segments.
- **The label ceiling is real and must be planned around.** The tag decodes to ~12 m; the
  drone is *visible* far beyond that. So the 12–30 m band — the band that most needs recall —
  has **no tag labels**. Three sanctioned sources for it: (i) `yolo11x_mit` teacher proposals
  + human spot-check (`nn_tier/PLAN.md` Tier-1 labeling); (ii) the target's **ULog track** for
  *scoring* range beyond the decode ceiling (scoring truth, not labels — tripod protocol §6);
  (iii) short manual passes. Report **label source per split** so a recall number is never
  read as better-founded than its labels.
- **Methodology gotcha:** GPU render corrupts fiducial decode at range → any *sim-side* tag
  sweep runs `SIM_GPU_RENDER=0` (`docs/placard_sizing.md`). Irrelevant on hardware; relevant
  if anyone regenerates the validation.

### D.3 Split — held-out FLIGHTS, never random frames (the guardrail that exists because we were burned)

Two retrains (v3, rebal) **aced frame-eval and failed in flight** (ADR-0061; ADR-0076
add #18d). The rule that came out of it:

- Split by **session/flight/scene group**, never by frame. `prepare_nn_tier_dataset.py`
  already implements deterministic **group-level** hashing plus a `--check` that re-asserts
  disjointness; the real-data ingest just declares `group = session_id`
  (`nn_tier/PLAN.md` §4.3).
- Hold out **whole days/lighting conditions** where the schedule allows — lighting is a
  domain axis, and a same-day split flatters.
- **Never validate on sessions that were mined for hard negatives** (mine-sessions and
  verdict-sessions disjoint — the Tier-3 rule, `nn_tier/PLAN.md` §3).
- **Watch for corpus leakage**, which has bitten this project once already: 68 of 361 DVB
  "test" bird images were already train negatives, and the fix was to re-derive and exclude
  the leaked set from the manifest (`docs/nn_tier/mono_vs_color_bird_ab.md` §2.b). Any
  imported corpus gets the same treatment.

### D.4 Train — COCO-init, corpus as regularizer

- **Init from COCO, never from sim weights** (ADR-0061: the visual domain is exactly what
  doesn't transfer). One sanctioned A/B arm: **n-mono weights as init** — legitimate because
  n-mono is *real-media*-trained, so the never-init-from-**sim** rule is not violated; COCO
  stays default unless the A/B wins on held-out **flights** (`nn_tier/PLAN.md` Tier-1).
- Mix tripod captures **with** the nn_tier corpus (15,391 images) as background-diversity
  regularizer; the ratio is a **logged experiment**, not a guess.
- Hold negatives at **15–20%** (rebal over-suppressed and flew blind: 0/16 acquisitions).
- Compute: `.venv-seeker-train-gpu` on the RTX 4070, ~72 s/epoch per ~10 k images
  (`nn_tier/compute_notes.md`) → a tripod-day fine-tune is an under-an-hour job. One training
  job at a time (WSL2 silent-spill: batch >8 GB spills to RAM at 4× slower, no hard OOM);
  setsid-detached with checkpoints (background tasks are reaped at ~51 min).
- Export ONNX to `scripts/seeker/weights/nn_tier/` — same rails as the existing daemons
  (`train_daemon_nn_tier.py` pattern).

### D.5 Score — the two curves, and what each one is allowed to gate

`scripts/seeker/tripod_score.py` is the built scorer (P0.3, adversarially verified):

- **Curve (a) — AprilTag decode envelope** → `R_decode_any` and `R_decode90` (the farthest
  range where decode *sustains* ≥90% inward), then the money-gate arithmetic with the
  explicit 5-streak burn. Gates the **~$740 interceptor order**.
- **Curve (b) — NN recall vs range × position-in-frame** → gates **only** the $70 Hailo HAT /
  markerless phase, never the interceptor order.
- **Position-in-frame binning is mandatory, not optional.** Range-only reporting is exactly
  what hid a 100%-static-vs-0.8%-in-flight gap for weeks (ADR-0076 add #18k). Score
  `--pos-bands` and report the matrix.
- Advisory already logged in the tool: `R_decode90` reports the **bin upper edge** (±one 2 m
  bin) — do not over-read a marginal result.
- Per §B.5, run curve (b) **twice**: `n-mono` (candidate) and `quad_v2` (historical bar).

### D.6 The acceptance gate — what a real-flight result has to look like

Pre-registered so it can't be re-negotiated after seeing the data. A retrain graduates to
Stage-2 (guiding) only when, **on held-out FLIGHTS** (never frames):

1. **Recall vs range × position-in-frame** beats the n-mono bar in the ≤40 px @1280-eq bins,
   *and* is reported per position band (top/middle/bottom third at minimum).
2. **Effective acquisition clears the money gate at the deployed frame rate**, computed with
   §A.4's *measured* p under the **run-length** model (not the gate's mean-rate model):
   `R_acq,effective = R_acq,first-detect − (E[frames to confirm]/fps)·V_closing`, scored at
   both V = 9 m/s (conservative) and ~20 m/s (head-on aggressive), needing **t_go ≥ 0.5 s**.
3. **False-fire cannot steal the handoff.** The bar is **not** the per-frame rate — clutter
   false fires are *temporally correlated* (the same bush stays in frame), so the honest
   statistic is the **observed maximum consecutive false-fire run length** on drone-free
   sweeps, which must be **< the confirm-rule length** with margin, reported with the sweep
   length in frames. (For calibration: n-mono's per-frame desert-plate false-fire is 4.9%;
   under a naive independence model a 5-run would be ~2.8e-4 per 1000 frames — that model is
   *wrong* for clutter and is quoted here only to show why the empirical run-length is the
   number that counts.)
4. **INT8 delta re-measured** on the same held-out flights if the artifact is quantized —
   specifically the small-bin recall (`nn_tier/PLAN.md` Tier-2 gate 4).
5. **Small-n honesty:** a field afternoon cannot buy the project's n ≥ 8 paired-seed sim
   standard; the caveat is stated in the result, every time (tripod protocol §4.6).

---

## E. The ranked perception levers

Ranked by expected effect on **acquisition range × detection density**, with the honest tag.
**SIM** = testable in Gazebo/offline today · **HW** = only real hardware can answer ·
**GRAVE** = considered and closed, listed so nobody re-proposes it.

### E.1 The short version

| # | lever | tag | expected effect | cost | evidence |
|---|---|---|---|---|---|
| 1 | Real-data mono retrain on **our** target/camera/mount (Tier-1) | **HW** | the only thing that closes a measured **0.0003 → 0.44 AP50** class of gap | a tripod afternoon + <1 GPU-hr | `nn_tier/PLAN.md`; §A.2 |
| 2 | **Shadow-mode measurement on Pi 5 CPU** during tag-guided flights | **HW**, $0 | converts every remaining hypothesis into a measurement; de-risks the $70 HAT | ~0 (code + a log) | §B.4 Stage 1 |
| 3 | **Confirm-rule redesign**: top-K / score-margin selection + M-of-N instead of 5-consecutive | **SIM** (offline-scoreable today) | top-3 recovers **89%** of localizable frames; margin gate flips **80%** of masked; M-of-N cuts confirm burn 31.4 m → ~9.6 m at p = 0.44 | hours | `inview_probe_results.md`; §A.4 |
| 4 | **Exposure discipline ≤1 ms** (+ gain, `FrameDurationLimits`) | **HW** | the only defense against terminal blur; **[DERIVED]** at 485°/s mean LOS on the OV9281: 5 ms ⇒ **26 px** smear vs a 3–18 px target; 1 ms ⇒ **5.3 px** | $0 (a control) | §E.2; `camera_paper_check.md` §3 |
| 5 | **Hailo-8L HAT** (fps ⇒ smaller streak burn) | **HW** | ~4 m effective R_acq at 9 m/s, ~10 m at 20 m/s, model unchanged | $70 | §B.3 |
| 6 | **Operating-point re-pick** (conf threshold from the *real* PR curve, at the streak level) | **SIM**/offline on real frames | 0.25 is inherited from the sim era; the deployment-relevant optimum is a streak-level, not per-frame, choice | hours | `tripod_score.py --nn-conf`; §D.6 |
| 7 | **Site hard-negative mining** (Tier-3, holding 15–20% negatives) | **HW** | attacks outdoor false-fire, the one thing that can steal a handoff | field time | `nn_tier/PLAN.md` Tier-3 |
| 8 | **Position-weighted training augmentation** (center-high bias + edge partials, per A4/A5) | **SIM** + real data | matches training placement to the measured engagement distribution | free at train time | `viewpoint_and_deploy_spec.md` A5 |
| 9 | **Air-to-air corpus expansion** (Det-Fly license email; retry AOT) | **SIM**/offline | the corpus's scarcest resource is the *right elevation aspect*; `dut` (3,000 imgs) is ground-looking-up = wrong aspect | an email | `nn_tier/PLAN.md` §5.2 |

### E.2 Notes on the non-obvious ones

**Lever 3 is the cheapest real win on this list and it is testable *today*, offline.** Both
halves are already measured on stored frames (`phantom_competition_replay.py`): top-1 → top-3
recovers 74% → 89% of localizable frames, and a score-margin gate flips 80% of phantom-masked
frames. The confirm-rule half (M-of-N) needs the §A.4 derivation validated against real
sequences, and it touches the `handoff` stage — so the correct move is a **pre-registered
offline A/B on tripod frames** (arms: 5-consecutive top-1 / 5-consecutive top-3 / 5-of-8
top-3 + margin gate; metrics: streak-formation range on target passes **and** false-streak
count on drone-free sweeps), result handed to the head for the contract. Caveat: the
committed camera-forward mount removes the *own-prop* phantom at source, so lever 3's value
outdoors is against **terrain/bird clutter**, not props — which is precisely what the sim
cannot rank.

**Lever 4's arithmetic, shown [DERIVED]** (`scripts/seeker/streak_burn_derivation.py`): at the
OV9281's **10.85 px/deg** (1280 px / 118° H, `camera_paper_check.md` §2), a 485°/s *mean*
terminal LOS rate smears 2.42° ≈ **26.3 px** at 5 ms and 0.48° ≈ **5.3 px** at 1 ms. The target
subtends 3.4–18 px at the 640 NN input over 6–20 m (`viewpoint_and_deploy_spec.md` A3), so a
5 ms exposure smears it across **more than its own width** — the same mechanism that killed the
bank-as-accel cue (σ_a 6–12 m/s² under ~23 px smear, graveyard; the BOM's ~23 px and this 26 px
are the same calculation at slightly different px/deg assumptions). **Honest bound:** at the
*peak* 1870°/s even 1 ms still smears **20 px** — exposure does not rescue the CPA blind window,
and it is not supposed to (the miss is kinematic at handoff, ADR-0023). Exposure protects the
**approach-phase** recall that sets R_acq, which is the whole point.

**Lever 6 has a trap.** Confidence was measured **inverted** in the billboard era (phantoms
scored *higher* than the real target, ADR-0057), which is why a confidence gate was banned in
`detect_track.py`. That inversion was a *phantom* artifact and the phantom is mount-removed —
so re-picking the operating point on **real** data is legitimate, but the result must be
re-checked for inversion before any threshold is trusted (plot confidence vs. true/false on
the real frames; if it inverts again, rank-based selection wins over thresholding).

### E.3 Graveyard — considered, closed, do not re-propose

Listed with *why*, because "we already tried it" is only useful if the mechanism is attached
(all in `docs/project_state.json` `graveyard` unless noted):

| closed lever | why it's closed |
|---|---|
| **v3 retrain / rebal retrain** | both aced frame-eval, failed in flight (v3 regressed all 10 held-out flights; rebal flew blind, 0/16 acquisitions) — ADR-0061, add #18d |
| **Auto-crop / foveated crop as a range lever** | crop = **identical** recall to full-frame at the 8–12 m wall band (33%/33%, 0%/0%); the "47→71% win" was edge-clipped ≤6 m targets (3rd mirage) — add #18j-fix |
| **Higher-res sensor / narrow-long lens for range** | resolution matters only past ~24 m static; a narrow lens re-creates the fast-crosser-walks-out-of-frame failure; wide FoV is a **hard constraint** — ADR-0024, `wide-fov` |
| **Subpixel / centroid bearing** | NULL at n = 31 (the n = 8 "win" was noise) — ADR-0071 |
| **CSRT detect-then-track for the quad; VIT tracker** | NN-only won for the 3D quad (add #2); VIT 0/8 — see §C |
| **Range-plausibility handoff gates** (both variants) | ADR-0077 regressed l2r 81% → 25%; the pre-flight window aborted both directions — the cause was ~0 real recall, nothing to acquire |
| **Bank-as-accel cue** | σ_a 6–12 m/s² vs a 4.7 m/s² signal under terminal smear — injects more error than signal (ADR-0073, measured dead) |
| **AprilTag as the terminal seeker in a crossing dash** | 0 detections through a 16 m/s crossing dash — add #18e. (Tag survives as the straight-leg first-kill baseline **only**; see §B.4.) |
| **Onboard acoustics** | own-prop noise ~40 dB the wrong way |
| **Fixed up-tilt in the cue-era terminal-parity context** | ADR-0068 parity FAIL — note this is *scoped*: the 2026-07-17 acquisition-first fixed tilt sized to the measured dash pitch is a different, live decision (the `pointing` stage), not a resurrection |

---

## F. Honesty — what every lever in this doc re-earns

1. **The boundary.** `gt_*` is **scoring/logging only**. The deployed seeker reads camera
   pixels + own-state EKF, nothing else (`flight/deploy/seeker_loop.py` header documents the
   no-cheat audit and states there *is* no ground truth on real hardware to read).
2. **The AprilTag's three sanctioned roles, and only three:** camera/range **calibration**,
   **training-time auto-labels**, and the **staged baseline seeker** for first kills. It is
   never an input to the deployed markerless seeker, and the operational target does not
   carry it (`real_data_pipeline.md`; constraint `honesty-boundary`).
3. **Every new guidance-relevant perception path re-earns the numeric no-cheat audit**, at
   the live gate — offline wiring does not discharge it (`markerless_loop.py` §honesty;
   `docs/audit_targets.md` pattern). That includes Stage-2 YOLO-guided and any confirm-rule
   change that touches handoff.
4. **Sensor/resolution changes are disclosure-gated.** Any change to camera, resolution, or
   input size must be disclosed and must **re-earn M1/M2 and re-scale σ_R** (ADR-0025;
   ADR-0074's gate). The measured OV9281-vs-sim penalty to carry forward is **~15% fewer
   px/deg, not 30%** — the 30% was the 148° diagonal mistaken for the 118° horizontal FoV
   (`camera_paper_check.md` §2, ledger `ov9281-pxdeg-30-vs-15`).
5. **Frame-eval is never a verdict.** AP50/recall on frames *ranks*; only **held-out FLIGHT**
   validation decides (ADR-0061). Every number in §B.2 is a public-media frame metric and is
   labeled as such in its own source doc — reproduce that labeling anywhere it is quoted.
6. **Shadow mode has its own honesty trap.** If the same sessions are used to pick the
   operating point *and* to report the acceptance number, that is train-on-test. Pre-register
   **tune sessions** and **verdict sessions** as disjoint before scoring, the same way
   mine-vs-validate sessions are disjoint in Tier-3.
7. **Label provenance bounds every recall claim.** Auto-labels come from the tag (IoU 0.965)
   inside ~12 m and from teacher proposals / manual passes beyond it. Report the label source
   per split; a recall number is never better-founded than its labels.
8. **Anti-mirage discipline** (constraint `anti-mirage`; five mirages caught in the coded-dash
   arc alone): a new perception "win" needs a control arm, paired seeds n ≥ 8 in sim, and
   held-out **flights** on hardware. A field afternoon's n is small and says so out loud.
9. **Simulate worse than ideal.** Where a real number does not exist yet, the WORST-credible
   value governs the decision: CPU YOLO at **5 fps** (not 10), Hailo end-to-end at **40 fps**
   (not 80), and mono real frames treated as **strictly harder** than the sim curve
   (`camera_paper_check.md` §4).

---

## Appendix — file map for this stack

| function | path |
|---|---|
| deployed real-hardware seeker loop | `flight/deploy/seeker_loop.py` |
| ONNX detector wrapper (letterbox/infer/box→detection) | `scripts/seeker/finetuned_seeker.py` |
| scoring gate (`box_hits_gt`, sec² off-axis widening, unified 0.52 m extent) | `scripts/seeker/box_scoring.py` |
| tripod two-curve scorer (money gate) | `scripts/seeker/tripod_score.py` |
| Pi capture sessions | `scripts/seeker/pi_capture.py` |
| tag → YOLO auto-labeler | `scripts/seeker/autolabel_from_apriltag.py` |
| auto-labeler validation (sim) | `scripts/seeker/validate_autolabel_sim.{py,sh}` |
| in-flight recall vs range | `scripts/seeker/approach_recall.py` |
| phantom-competition replay (top-K / margin analysis) | `scripts/seeker/phantom_competition_replay.py` |
| real-media corpus prep (group-level split, `--check`) | `scripts/seeker/nn_tier/prepare_nn_tier_dataset.py` |
| held-out evaluators | `scripts/seeker/nn_tier/eval_n_mono.py`, `eval_heldout_smono.py`, `eval_bird_negatives.py` |
| training daemons (setsid, checkpointed, auto-export) | `scripts/seeker/train_daemon_nn_tier*.py`, `train_real_data.py` |
| candidate weights | `scripts/seeker/weights/nn_tier/{n-mono,n-mono-aug,s-mono,n-color}.onnx` |
| deployed sim weights (historical bar) | `scripts/seeker/weights/drone_finetuned_quad_v2.onnx` |
| detect-then-track (closed arm, kept for its reasoning) | `scripts/seeker/detect_track.py` |

**Companion docs:** `docs/nn_tier/PLAN.md` (model sweep + tiered roadmap) ·
`docs/real_data_pipeline.md` (auto-label design) · `docs/inview_probe_results.md` (the two
in-view probes) · `docs/camera_paper_check.md` (OV9281 paper gate) ·
`docs/placard_sizing.md` (0.35 m placard + decode envelope) ·
`docs/tripod_test_protocol.md` (field logistics) ·
`docs/nn_tier/viewpoint_and_deploy_spec.md` (aspect/position weights, Hailo envelope) ·
`docs/nn_tier/mono_vs_color_bird_ab.md` (ADR-0078 supporting A/B).
