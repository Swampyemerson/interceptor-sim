# Seeker v3 dataset + fine-tune plan (onboard drone detector)

> **STATUS: DESIGN ONLY — ratifiable package.** Nothing here has been run.
> No sim was launched to produce this document (another job owns the
> simulator; one-sim-at-a-time rule). Companion skeleton:
> `scripts/seeker_v3_capture.py` (runnable offline via `--dry-run` /
> `--self-test` / `--print-flight-plan`; live capture is TODO-gated).
>
> Every quantitative choice below cites its source (ADR / script constant /
> log) or is flagged **[A#]** into the Assumption Register (§10) for
> capture-time verification.

## 1. Problem statement and evidence

`drone_finetuned_v2.onnx` (the recommended `MARKERLESS_NN_WEIGHTS`,
ADR-0042) collapses at 12 m/s weave/jink:

- **Pk@2.5 m: weave 1/8, jink 3/8 vs the AprilTag control's 8/8 at the SAME
  kinematics** — a perception limit, not kinematic (ADR-0056).
- **The phantom mechanism (ADR-0057, n=16 instrumented weave flights):**
  195/217 ENGAGE detections are FALSE (range error > 8 m) vs only 5 GOOD
  (≤ 3 m). The phantom is a **large interior box** that reads ~1.6–1.8 m
  range while the true target is 15–100 m away (18–22 m typical error). At
  the calibrated span 0.9216 m (`drone_finetuned_v2.onnx.calib.json`) and
  fx = 539.936 (`configs/camera_intrinsics.json`), a 1.7 m range readout implies a
  **~293 px-wide box** — this is not a speck, it is a huge, confident blob.
- **Confidence is INVERTED:** phantom median conf 0.34 vs real-target 0.17
  (the real target at 15–100 m is ~8 px and low-conf). A confidence gate
  keeps the phantoms and drops the target — dead as a fix (ADR-0057).
- **The phantom triggers a FALSE HANDOFF during DASH:** 12/16 flights
  latched camera-only onto empty space (miss median 4.53 m) vs 3/16 real-ish
  handoffs (1.75 m). The handoff detection IS the outcome (ADR-0057
  root-cause addendum).
- **Direction asymmetry:** markerless r2l much worse than l2r on both
  weave and jink; the AprilTag control shows none → a perception × view-
  geometry artifact (ADR-0056) → the capture matrix must cover BOTH
  directions.
- **Closed-loop noise floor at this regime is ~5 m run-to-run at a fixed
  seed** (ADR-0057 self-correction) → the v3 verdict must be FRAME-LEVEL,
  not closed-loop miss (§8).

This is the same disease the GROUND detector had and cured: ground_v1
trained at one range showed a −6.3 m out-of-domain bias at 50 m (ADR-0053);
multi-range training (T17-v2) fixed it with held-out-range proof
(ADR-0055). v3 applies the identical medicine — **train on the deployment
domain, split structurally, evaluate held-out** — to the onboard detector's
maneuvering domain.

Relationship to the detect-then-track pivot (ADR-0057, in progress):
complementary, not competing. The tracker still needs a clean NN seed
detection; v3 attacks the seed's phantom probability at the source. v3
touches **only data and weights** — no seeker-code edits (the seeker files
are mid-edit by another agent).

## 2. The domain gap: what v2 was trained on vs what 12 m/s maneuver presents

v2's training set (ADR-0042: 531 pos / 354 neg = v1 renders ∪ flight
capture, merged by `merge_dataset_v2.py`):

| Axis | v2 training data | 12 m/s weave/jink deployment | Gap |
|---|---|---|---|
| Range | Static grid 2–12 m (`render_sim_dataset.py` defaults) + flight positives mostly terminal | Target 15–100 m at the false-handoff moment (ADR-0057); terminal 1–5 m | **No far-range positives** (small, low-conf real target) |
| Aspect | Target yaw 0/±25° only (`render_sim_dataset.py --yaws-deg`) | Fast crossers present quartering/broadside aspects through the whole weave cycle | **±25° never covers a crossing geometry** |
| Crossing speed | cap1/cap2 line 9 m/s, cap3 weave 9 m/s (`capture_pass.sh` FLIGHTS table); grid static | 12 m/s base + weave (period 4 s, lat 3 m/s, amp 1.91 m) / jink (2 turns ≥ 40°) (`m4_target_mover.py` defaults, T21 arms ADR-0056) | **Never saw 12 m/s; one weave flight total** |
| Own-airframe attitude | Negatives from 3 near-straight-line chases: prop blades at frame TOP, drone-shaped ground shadow bottom-center, level-ish attitude (ADR-0042) | 12 m/s chase pulls sustained hard bank/pitch; shadow and prop arms sweep through the FRAME INTERIOR (past the L/R-edge self-mask) | **Phantom-prone textures at banked attitudes have zero negative supervision** [A5] |
| Altitude | Grid heights 0–0.5 m; flight z ≈ 0.5 m | Same z ≈ 0.5 m corridor (mover keeps z0; `m4_target_mover.py`) | Small — keep a modest height axis |

**Motion blur — the honest note.** The premise "motion-blurred poses" does
not exist in this sim: the camera SDF
(`~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf`) is a
plain 1280×960 @ 30 Hz camera, HFOV 1.74 rad, with **no motion-blur,
exposure, or lens model** — Gazebo Harmonic/ogre2 renders instantaneous
frames. What high speed produces in-domain is (a) large **inter-frame
displacement** (12 m/s at 10 m range ≈ 1.2 rad/s LOS rate ≈ 22 px between
30 Hz frames at fx 539.9; ~43 px at 5 m) and (b) rapid **apparent-size /
aspect dynamics**, plus render/RTF nondeterminism in WHICH instantaneous
poses get rendered (the ADR-0057 coin-flip). Consequences: (1) the capture
matrix targets pose/aspect/size/attitude diversity, not blur; (2) do NOT
add blur augmentation to the primary training arm — it would train for an
artifact the eval domain cannot show (it stays a flagged sim-to-real knob
for the future hardware seeker, per the parent-project translation table in
docs/goals.md).

## 3. Capture matrix

All captures on the `markerless` world / `fpv_target_markerless` target
(the deployment visuals — `worlds/markerless.sdf`), onboard camera topic
per `m2_detect.IMAGE_TOPIC`. Venv: `.venv-seeker` (cv2 + gz-transport13).
One sim at a time; idle machine (CLAUDE.md batch hygiene).

### Phase 0 — phantom forensics (BEFORE finalizing the negative mix)

The T21 flight CSVs are gone (`logs/mc_t21_*.csv` gitignored, not on disk),
and the flight CSV schema logs `meas_x/y/z, meas_conf` but not the box
(u, v, w, h) (`m4_intercept.py` CSV_HEADER) — so there is **no on-disk
pixel-level evidence of what the phantom locks onto**. Best current
evidence: v1-era phantoms were own prop arms at the FOV periphery
(bearing −44/+33°, ADR-0040) and v2's cured negatives were prop blades at
frame top + own ground shadow bottom-center (ADR-0042); the T21 phantom is
interior (passes the ±30°/edge self-mask) and ~300 px wide (§1). Leading
hypothesis: **shadow/prop-arm/horizon textures swept into the frame
interior at banked attitudes the straight-line negative pass never
produced** [A5].

Do: fly 2 ride-along weave-12 flights (eval-seed family, §8), run
`drone_finetuned_v2.onnx` OFFLINE over every captured frame, and cluster
all ≥ 0.25-conf detections by (u, v, w, h, IoU-vs-gt). Output: a short
forensics note pinning the phantom texture(s). These frames belong to the
EVAL pool, never to training.

### Phase 1 — static teleport grid (systematic pose coverage; one boot)

Implemented by `scripts/seeker_v3_capture.py` (modeled on
`multirange_capture.py`: gz-transport camera subscription + gt pose
logging + `gz service` subprocess teleports, "the mover pattern").
Interceptor parked at origin as in `render_sim_dataset.py`'s v1 grid [A8].

| Axis | Values | Source / rationale |
|---|---|---|
| Range (m) | 1.5, 2, 3, 4, 6, 8, 12, 18, 25, 35, 50 (11) | v1 grid was 2–12 (`render_sim_dataset.py`); extension to 50 m because the real target sits 15–100 m away at the false-handoff moment and is currently an unsupervised ~8 px low-conf blob (ADR-0057). 50 m projects to ~9.7 px native / ~4.9 px at the 640 input — detectability floor [A4] |
| Target yaw (deg) | 0, ±45, ±90, ±135, 180 (8) | Full aspect ring vs v1's 0/±25. In flight the mover is POSITION-only (orientation never set, ADR-0010 #6 / `m4_target_mover.py`), so deployed aspect variation comes from viewing geometry — the yaw ring is the static proxy for it |
| Bearing (deg) | −25, 0, +25 (3) | Inside the deployed ±30° self-mask (`finetuned_seeker.py` max_bearing_deg=30); lateral = range·tan(bearing) |
| Height (m) | 0.25, 0.5, 1.5 at ranges ≤ 18; 0.5 only at 25/35/50 (3→1) | z≈0.5 is the deployment corridor (`m4_target_mover.py` DEFAULT_START, mc_batch z=0.5); ±1 m at 50 m is only ~11 px of elevation — volume trim |
| Frames/cell | 1 | Grid cells are deterministic renders; diversity comes from the axes, not repeats (same choice as v1's 495-frame grid) |

Count: 8 ranges × 8 yaw × 3 bearing × 3 height = 576, plus 3 far ranges ×
8 × 3 × 1 = 72 → **648 grid positives**. Projected box widths (fx·0.9/R,
extent 0.9 m per `render_sim_dataset.py --extent-m`): 324 px @1.5 m …
9.7 px @50 m — all ≥ the 4 px label floor (`capture_flight_frames.py`
MIN_BOX_PX) at native resolution [A4].

### Phase 2 — ride-along maneuvering flights (the domain-matched data)

`capture_pass.sh` pattern (fresh boot per flight) + the PASSIVE recorder
`capture_flight_frames.py` (subscribe-only, zero service calls, gt
snapshot in the frame callback). Flight = a deployment-profile
`m4_intercept.py` markerless engagement with the target mover on the arm's
path. **Capture-aid flag:** fly capture flights with `--handoff-cue-gate 8`
ON (ADR-0057 attempt-3) — it stops the interceptor being fooled onto empty
space, so the camera keeps pointing near the true target → denser in-frame
positives at genuine chase attitudes. Capture aid ONLY, never an eval
config [A2].

Training-capture arms (path/cue seeds 1101–1114 — disjoint from the
master-seed-42 eval family, the ADR-0042 seed-disjointness discipline
[A6]):

| # | Path | Speed (m/s) | Direction | Purpose |
|---|---|---|---|---|
| 1–4 | weave | 12 | l2r, r2l × 2 seeds | The failing regime (ADR-0056); both directions (r2l asymmetry) |
| 5–8 | jink | 12 | l2r, r2l × 2 seeds | Second maneuver family (ADR-0056: fails the same way) |
| 9–10 | weave | 9 | l2r, r2l | Bridge to the validated straight-line-era speed (ADR-0038..0044) |
| 11 | line | 9 | l2r | Anti-forgetting anchor (v2's domain) |
| 12 | line | 6 | r2l | Anchor at the M5 default speed (`mc_batch.sh` SPEEDS=6.0) |
| 13–14 | weave | 15 | l2r, r2l | WORST-credible speed margin (simulate-worse-than-ideal mandate, CLAUDE.md) [A1] |

Weave/jink knobs at the T21 defaults: period 4 s, lat-speed 3 m/s
(amplitude 1.91 m), jink-count 2, jink-min-deg 40 (`m4_target_mover.py`
defaults; ADR-0056 used these). Geometry: same as the T21 arms — mc_batch
standard geometry, x0 6.5 [A1].

Recorder settings: `--every 3 --max-frames 300` per flight (14 × 300 =
≤ 4200 raw frames; v2's 3 flights at defaults yielded 36 pos / 694 neg —
positives are scarce in failing chases, hence the cue-gate aid and 14
flights) [A6]. Per-flight `--run-tag capv3_XX` (the flight-level split key,
§6).

### Phase 3 — hard negatives (see §4)

No extra sim time: negatives fall out of Phases 1–2 plus offline mining.

**Total sim budget:** 1 grid boot (~650 teleports ≈ 20–30 min) + 14
capture flights (~65–75 s flight each per ADR-0041 cost note, plus ~2–3 min
boot each ≈ 1–1.5 h) + Phase-0/eval flights (§8, 10 flights ≈ 45–60 min).
≈ **2.5–3 h simulator wall time**, all sequential, idle machine.

## 4. Hard negatives — explicit supervision for phantom-prone textures

Three sources, in priority order:

1. **Banked-attitude empty frames (automatic).** Every Phase-2 frame where
   the gt target projects behind-camera or fully off-frame is saved with
   an EMPTY label (`capture_flight_frames.py` classify() "negative"
   branch — already built). In a 12 m/s failing/cue-gated chase these
   frames show exactly the hard-bank prop-arm/shadow/horizon interior
   views the phantom feeds on — the attitude regime v2's straight-line
   negatives never covered (§2 table, row 4). Expect these to dominate raw
   negatives (v2 got 694 from 3 gentler flights).
2. **Phantom MINING (the targeted lever).** Run `drone_finetuned_v2.onnx`
   offline over every captured NEGATIVE frame; any frame where v2 fires at
   ≥ 0.25 conf (the deployed threshold, `markerless_loop.py`
   MARKERLESS_NN_CONF default / `finetuned_seeker.py` conf_thres=0.25) is
   a **guaranteed-include** hard negative, EXEMPT from the negative-cap
   subsample. Also mine POSITIVE frames where v2's best box has IoU < 0.1
   vs the gt box (phantom and target co-visible → the gt label itself is
   the counter-supervision). This finds the phantom textures empirically
   instead of guessing them — Phase-0's cluster analysis cross-checks it
   [A5].
3. **Legacy negatives retained.** v2's 354 negatives (prop blades top,
   shadow bottom-center, level attitude) stay in the union — they cured
   the v1 pollution class (0.751 → 0.000, ADR-0042) and dropping them
   risks re-opening it.

Curation: cap negatives at **45%** of the final set (v2 used 40%,
`merge_dataset_v2.py --max-neg-frac`; +5 pt because the failure mode is
false-positive-driven), with mined frames exempt from subsampling.
Deterministic subsample, seed 0 (same mechanism as `merge_dataset_v2.py`).

## 5. Auto-labeling (no hand labels)

Reuse the proven gt-projection chain unchanged:

- **Ride-along frames:** `capture_flight_frames.py` already labels at
  capture time — `PoseTracker.ground_truth_rel_optical()` (the M2-validated
  transform chain) snapshotted in the same frame callback, projected
  through the fixed intrinsics (FX=FY=539.936, CX=640, CY=480 — matches
  `configs/camera_intrinsics.json`), extent 0.9 m, three-way pos/neg/drop split at
  the 4 px floor. Label-skew at 12 m/s: the skew source is pose-topic vs
  render latency (~one physics tick, ~4 ms → 12 m/s × 4 ms ≈ 5 cm → ~5 px
  at 5 m) — the same argument the script's docstring makes at 9 m/s,
  still small vs box extent at 12 m/s [A7].
- **Grid frames:** same projection math implemented locally in
  `seeker_v3_capture.py` (documented copy of `capture_flight_frames.classify()`
  so the script's `--dry-run`/`--self-test` need no gz/cv2 import), gt from
  the same PoseTracker.
- **Honesty boundary (CLAUDE.md / ADR-0010, restated):** `gt_*` is used
  offline ONLY to manufacture training labels — dataset manufacture, not a
  runtime guidance input (the exact boundary `gen_sim_dataset.py` /
  `render_sim_dataset.py` / `capture_flight_frames.py` document). The
  trained detector reads pixels only; if v3 is adopted into the guidance
  path it re-earns the numeric no-cheat audit like every seeker change.
- **Verification step (ADR-0042 precedent "labels visually verified"):**
  before training, render 50 random labeled frames with their boxes drawn
  and eyeball them; reject the batch if close-range fast-crossing boxes
  are visibly off by > ~0.3 box widths [A7].

## 6. Train/val split — hold out entire FLIGHTS

Frames within a flight are temporally correlated (30 Hz, `--every 3`);
per-frame Bernoulli splitting (what `merge_dataset_v2.py` does) leaks
near-duplicates across the split — the exact ADR-0049 weakness the ground
detector's v2 fixed with a STRUCTURAL split (`build_ground_dataset_v2.py`,
held-out-by-standoff; validated by ADR-0055's held-out 70/130 m result).
v3 applies the same rule to its two data kinds via a new
`build_onboard_dataset_v3.py` (to be written at execution time; the split
key is already in the filenames):

- **Flight frames: split by run_tag.** Entire flights → val:
  capv3_03 (weave-12 l2r, 2nd seed), capv3_04 (weave-12 r2l, 2nd seed),
  capv3_08 (jink-12 r2l, 2nd seed), capv3_10 (weave-9 r2l). All frames of
  the other 10 flights → train. Both directions and both maneuver families
  are represented in val.
- **Grid frames: split by held-out RANGE bucket** (the
  build_ground_dataset_v2 pattern): ranges **3 m and 18 m** entirely →
  val (each sits between trained neighbors — 2/4 and 12/25 — so passing
  requires range generalization, not interpolation).
- **Legacy v1/v2 data: all → train.** Its old val role is retired; the v3
  val set must contain zero frames correlated with anything in train.
- Emit a `split_meta.csv` (per-image source/flight/range/split), the
  ADR-0055 verifier artifact that lets an independent check prove zero
  leakage structurally.

Note: this val set tunes training (early stop, checkpoint pick). The
v2-vs-v3 VERDICT uses the separate eval pool (§8) that neither model's
training ever touched.

## 7. Fine-tune configuration

**Base weights: `yolo11n.pt` (original COCO pretrain) — NOT
`drone_finetuned_v2.pt`.** The argument:

1. **Recipe discipline (the decisive one).** v2 was itself trained from
   yolo11n on v1-data ∪ capture with "dataset the ONLY variable"
   (ADR-0042). Starting v3 from the same base with the same recipe keeps
   the v2→v3 comparison a single-variable experiment: the dataset. Start
   from v2's weights and any improvement is confounded (data vs longer
   effective training vs init).
2. **Forgetting is handled by DATA, not weight inertia.** The
   straight-line domain is preserved by including v1's renders and v2's
   flight capture wholesale in the union (§4.3, §6) — the mechanism that
   demonstrably worked when v2 absorbed v1's domain. Warm-starting from v2
   would instead preserve exactly the high-confidence phantom features
   (conf 0.34 on empty ground) that v3 exists to remove.
3. **COCO-init convergence is cheap here.** This easy domain hits mAP50
   0.992 by epoch 6 from yolo11n (ADR-0040); v2-init saves nothing
   meaningful on a CPU budget.
4. Fallback ablation (only if the primary arm misses a §8 gate): one run
   initialized from `drone_finetuned_v2.pt`, identical data — cheap to
   add, not run by default.

**Recipe (identical to v2 except where flagged):**

| Knob | Value | Source |
|---|---|---|
| Trainer | `scripts/seeker/train_drone_finetune.py --go --run-name drone_finetune_v3 --out-name drone_finetuned_v3` | v2 pin discipline: never clobber v1/v2 artifacts (that script's --run-name comment) |
| imgsz | **640** | Deployed ONNX runs at 640 (`finetuned_seeker.py` imgsz=640; ADR-0042 "yolo11n @640"). imgsz 960 would cost ~2.25× inference → threatens the ~14 Hz NN cadence (ADR-0057 pivot note) — allowed only as a secondary arm if the recall-at-range gate fails AND a live rate check passes |
| epochs | **60, patience 15** (changed from v2's 80/none) | Metrics saturate by ~epoch 6–20 on this domain (ADR-0040; auto-memory note "metrics saturate ~epoch 20"); the dataset is ~4–5× bigger so wall-clock forces the trim — see budget below |
| batch | 16 | v2 recipe |
| seed | 0 | v2 recipe; deterministic split/subsample seeds all 0 |
| Augmentation | ultralytics defaults, unchanged | v1/v2 used defaults (`train_drone_finetune.py` passes none); recipe discipline. **No blur aug** (§2 — the eval domain cannot show blur). No aspect/rotation additions for the primary arm — aspect diversity comes from real captured data, which is the honest lever |
| Venv / hardware | `.venv-seeker-train`, CPU-only | cuda.is_available()=False in that venv (auto-memory); acceptable, don't fight it |
| Export | ONNX @640, opset 12, simplify | `train_drone_finetune.py` export path (matches v2) |
| Range calib | re-fit span sidecar on v3 val via `scripts/seeker/calibrate_range.py` → `drone_finetuned_v3.onnx.calib.json` | v2 sidecar mechanism (`finetuned_seeker.py` calib-sidecar loader; span_m_eff 0.9216 for v2) |

**CPU budget arithmetic** (from `logs/train_finetune.log`: ~2.4 s/it,
25 it/epoch at batch 16 ≈ 400 train imgs → ~60 s/epoch → **~0.15
s/img/epoch**): target final train split ≈ 2,300–2,600 images (648 grid +
~500–900 kept flight positives + capped negatives + ~880 legacy) → ~6
min/epoch → 60 epochs ≈ **6 h**, overnight on an idle machine — within
the "44 s/epoch on small sets, real cost on big ones" envelope. If the
merge lands > 3,000 train images, subsample correlated flight NEGATIVES
first (never positives, never mined frames).

License note (standing, `weights/LICENSES.md` / `train_drone_finetune.py`):
yolo11n is AGPL — fine for this sim artifact; the deployable real-hardware
answer stays the MIT model / an Apache-2.0 nano.

## 8. Eval protocol — frame-level, v2 vs v3 on the SAME held-out frames

**Why frame-level:** closed-loop miss at 12 m/s+maneuver has a ~5 m
sporadic run-to-run noise floor (ADR-0057) — an n=8 closed-loop A/B is
structurally uninformative; the frame-level detector comparison on
identical pixels is the clean, high-n signal. Closed-loop (n ≥ 16–24 per
ADR-0057 decision 2a) runs LATER, only after the frame gates pass, likely
combined with the detect-then-track A/B.

**Eval pool (never in any training set):** 10 ride-along flights on the
master-seed-42 family (the standing A/B convention: paired seeds, ADR-0042
seed disjointness — training capture used the 1101-series):
weave-12 × {l2r, r2l} × 2 seeds, jink-12 × {l2r, r2l}, weave-9 × {l2r,
r2l}, line-9 l2r, line-6 r2l. Includes Phase-0's 2 forensics flights.
Frames gt-labeled by the same chain (§5); flights flown ONCE, both models
scored offline on the identical PNGs.

**Scoring:** run v2 and v3 ONNX (onnxruntime, `.venv-seeker`) on every
eval frame at the DEPLOYED operating point — conf 0.25 + the deployed
self-mask (|bearing| ≤ 30°, 40 px L/R edge margin, `finetuned_seeker.py`)
— because the post-mask stream is what feeds handoff. Report raw
(mask-off) rates as a diagnostic column.

| Metric | Definition | Why |
|---|---|---|
| **M1 phantom rate** (headline) | post-mask detections/frame with IoU < 0.1 vs gt box (any detection counts on no-target frames) | The false-handoff driver (ADR-0057: 12/16) |
| **M2 recall** | fraction of frames with a gt box ≥ 4 px where a post-mask detection matches (IoU ≥ 0.25 OR center error < 25 px — the OR because IoU is brittle on ~8 px far-range boxes) [protocol choice, fixed before running] | The other half: v2 also MISSES the real target in this regime |
| M3 bearing quality | median box-center error (px) over matched detections | ADR-0056's binding limit is bearing quality |
| M4 range quality | median calibrated meas/gt over matched detections | v2's flight value was 0.935 (ADR-0042 G4) — don't regress |
| M5 confidence separation | median conf of true vs false detections | v2 is INVERTED (0.17 vs 0.34, ADR-0057); v3 must at least un-invert to give any downstream gate a chance |

Breakdowns: per range-bucket (≤5 / 5–15 / 15–30 / >30 m), per flight, per
direction (the r2l asymmetry), per path. Aggregation honesty: frames
within a flight are correlated → the pooled per-frame numbers are
reported, but the VERDICT statistic is the per-flight paired comparison
(sign test across the 10 eval flights), with "not significant at this n"
language where it applies (CLAUDE.md statistics rule).

**Pre-registered gates (v3 replaces v2 as the recommended
`MARKERLESS_NN_WEIGHTS` iff ALL pass; declared before training, ADR-0042
discipline):**

- **G1:** M1 phantom rate on 12 m/s maneuvering eval frames ≤ 0.2× v2's on
  the same frames (≥ 5× cut).
- **G2:** M2 recall on maneuvering frames ≥ v2 − 0.05 absolute, AND
  ≥ v2 + 0.10 on the 15–30 m bucket (the acquisition band where the real
  target is currently an unsupervised 8 px blob). Absolute floors set
  after Phase-0 measures v2's baseline on these frames [A3].
- **G3 no-regression (v2's domain):** on line-6/line-9 eval flights,
  recall within 0.02 and phantom rate not worse than 1.5× v2's (both
  models are near-zero-FP there; ratio on small counts read with care).
- **G4 static probe:** the ADR-0042 G1 probe (8/8 ranges 2–12 m fire at
  conf ≥ 0.9, `probe_markerless_range.py` list) still passes.
- **G5 honesty:** labels manufactured offline only; live path reads pixels
  only; if adopted, the guidance path re-earns the per-tick no-cheat audit
  (CLAUDE.md standing rule) before any closed-loop claim.

NULL branch (declared): if G1 passes but G2 fails (phantoms gone, target
still undetectable in fast-crossing frames), the finding is "the maneuver
regime is beyond a 640-input nano's recall" → the imgsz-960 secondary arm
(§7) and/or the detect-then-track path carry the fix; v3 still ships as a
phantom-rate improvement if G3/G4 hold.

## 9. Execution order and hard constraints

1. Write `build_onboard_dataset_v3.py` (merge + flight-level split +
   mining, §4/§6) and the eval scorer (`eval_seeker_v3.py`, §8) — offline,
   sim-free, can be built any time.
2. **[SIM]** Phase-1 grid capture (`seeker_v3_capture.py`, one boot).
3. **[SIM]** Phase-2 training-capture flights (14, sequential).
4. **[SIM]** Eval-pool flights (10, sequential; includes Phase-0).
5. Offline: Phase-0 forensics clustering → adjust mining emphasis [A5].
6. Offline: label spot-check (50 frames, §5) → merge → split_meta audit.
7. Train (CPU, overnight, idle) → export → recalibrate span.
8. Offline: score v2 + v3 on eval pool → gates → ADR with the verdict
   (pass or honest null), verifier-checked.

Constraints inherited: ONE sim at a time (steps 2–4 strictly sequential,
never alongside another batch); gates/batches at idle load; kill/poll via
script files not inline pkill patterns; git-stage specific paths only.
Steps 2–4 must wait for the simulator to be free.

## 10. Assumption register (verify at capture time — flagged, not sourced)

- **[A1]** Exact T21 12 m/s arm geometry (y0_mag, mover duration for a
  12–15 m/s crosser, west-angle) — reproduce from the T21 arm invocation;
  ADR-0056 pins path knobs but not the full mc_batch line. Verify against
  `mc_batch.sh` plan output (`--dry-run`-style stdout) before flying.
- **[A2]** `--handoff-cue-gate` flag name/semantics on the CURRENT
  `m4_intercept.py` (file is mid-edit by another agent) — verify
  `--help` output at capture time; if absent/renamed, fall back to plain
  chase flights and accept sparser positives.
- **[A3]** G2's absolute recall floors — set from Phase-0's v2 baseline
  measurement, then frozen before v3 training starts.
- **[A4]** Far-range visibility: verify at capture that the 35/50 m grid
  cells render a genuinely detectable target (≥ 4 px, non-degenerate
  contrast) and that settle-frames (start 6, `render_sim_dataset.py`
  default) suffice after long teleports; drop any cell whose median box
  < 4 px rather than train on invisible labels.
- **[A5]** Phantom texture identity — Phase-0 clustering output; adjust
  the mining emphasis and (if the cluster is NOT attitude-driven) revisit
  §4's premise in the ADR.
- **[A6]** Positives yield per failing chase flight (v2 precedent: only
  36 pos across 3 flights) — if < ~30/flight even cue-gated, add 2 more
  cue-gated weave-12 flights before merging.
- **[A7]** Label skew at 12 m/s close range — the 50-frame visual
  spot-check (§5); tighten if boxes trail the target by > ~0.3 widths.
- **[A8]** Grid boresight: confirm the parked interceptor's camera faces
  +X with the expected optical center before the batch (a --posecheck
  first frame, the `multirange_capture.py` posecheck pattern).

## 11. File map

| File | Status | Role |
|---|---|---|
| `docs/seeker_v3_dataset_plan.md` | this doc | The plan |
| `scripts/seeker_v3_capture.py` | skeleton, written | Phase-1 grid capture + `--print-flight-plan` (Phase-2/eval arm table) + offline `--dry-run`/`--self-test` |
| `scripts/seeker/capture_flight_frames.py` | exists, reuse unchanged | Phase-2 ride-along recorder |
| `scripts/seeker/capture_pass.sh` | exists, template | Per-flight boot/teardown orchestration (extend the FLIGHTS table) |
| `scripts/seeker/build_onboard_dataset_v3.py` | to write (step 1) | Merge + mining + FLIGHT-level split + split_meta.csv |
| `scripts/seeker/train_drone_finetune.py` | exists, reuse | `--go --run-name drone_finetune_v3 --out-name drone_finetuned_v3` |
| `scripts/seeker/calibrate_range.py` | exists, reuse | v3 span sidecar |
| `scripts/eval_seeker_v3.py` | to write (step 1) | §8 scorer (v2 vs v3, same frames, deployed operating point) |
