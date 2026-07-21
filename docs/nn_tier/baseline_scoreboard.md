# NN-tier RECON — baseline scoreboard (models on hand vs REAL labeled imagery)

**Date:** 2026-07-21 · **Phase:** markerless-NN tier recon (builder directive 2026-07-21:
build the markerless drone NN for the coming tiers — MIT-licensed start, heavy refinement on
real media, Central-Oregon high-desert domain, sized for the ordered hardware)
**Runner:** `scripts/seeker/nn_tier/baseline_eval.py` (self-test green; reproduce cmds §7)
**Raw numbers:** `logs/nn_tier/baseline_scoreboard_recon1.csv`, `logs/nn_tier/baseline_perimage_recon1.csv`
**Companions:** `docs/candidate_nn_shortlist.md` (fetch/licensing), `docs/transfer_bet_kill_test.md`
(the unlabeled smoke read this replaces with labeled metrics), `docs/camera_paper_check.md` item 4
(the mono confound this measures), `docs/nn_transfer_plan.md` (Hailo deployment anchor).

---

## 0. One-paragraph verdict (measured rows so far; heavy-model rows land from the daemon)

On REAL labeled imagery, every sim-trained net in this repo is **blind** (AP50 ≤ 0.002 on
large, obvious real drones — measured) and the deployed `drone_finetuned_quad_v2` is also
**trigger-happy on birds** (false-fires on 67% of drone-free bird images in color — and
**97% in grayscale**, the deployment camera's modality; mechanism probe: on 30 bird
frames its gray false-fires are universal, 30/30, with HIGHER confidence, median top-score
0.32→0.43 — mono makes real clutter look MORE like its matte sim training domain). The two
MIT real-photo checkpoints (`yolo11x_mit`, `flyingobj_mit`) are mid-eval in the detached
daemon at writing; their labeled rows land in `baseline_scoreboard_recon1.csv` (§4).
Whatever those rows say, neither is deployable as-is: `yolo11x_mit` is 22× the Hailo-8L
size class and `flyingobj_mit` called a real DJI "Drone" only 1/3 times in the transfer-bet
read. **Conclusion for the refine phase: there is no drop-in model. The path is a COCO-init
YOLO11n fine-tune on real, GRAYSCALE-evaluated, high-desert-matched media, with the MIT
checkpoints used as offline label-assist/teacher, not as init or deploy weights.**
(Numbers: §4; caveats that bound these claims: §6.)

## 1. Model inventory (everything in `scripts/seeker/weights/`)

| model | file | size | arch / input | license | status |
|---|---|---|---|---|---|
| **v2_deployed** | `drone_finetuned_quad_v2.onnx` | 10.6 MB | YOLO11n @640, 1-class | AGPL-3.0 (Ultralytics fine-tune) | **DEPLOYED** sim seeker (ADR-0058) — trained on color Gazebo renders only |
| quad_hardened | `drone_finetuned_quad_hardened.onnx` | 10.6 MB | YOLO11n @640 | AGPL-3.0 | sim variant (phantom-hardening round) |
| quad_rebal | `drone_finetuned_quad_rebal.onnx` | 10.6 MB | YOLO11n @640 | AGPL-3.0 | sim variant — honest NULL in flight (ADR-0061 lineage) |
| quad / quad_crop / v1 / v2 / v3 / v2_1280 | `drone_finetuned*.onnx` | 10.6–11.1 MB | YOLO11n @640 (v2_1280 @1280) | AGPL-3.0 | earlier sim fine-tunes, superseded |
| ground_v1 / ground_v2 | `ground_v*.onnx` | 10.8 MB | YOLO11n @960 | AGPL-3.0 | ground-rig lane, not the onboard seeker |
| **yolo11x_mit** | `drone_yolo11x_1280.onnx` (+ `.pt` 114.4 MB) | 228.2 MB | YOLO11x @1280, 1-class `drone` | **MIT** (HF `license:mit`, re-confirmed via HF API 2026-07-21) | candidate — real-photo trained ([doguilmak/Drone-Detection-YOLOv11x](https://huggingface.co/doguilmak/Drone-Detection-YOLOv11x)) |
| **flyingobj_mit** | `flying_objects_yolov8m.onnx` (+ `.pt` 52.0 MB) | 104.1 MB | YOLOv8m @1280, 5-class (Drone/Airplane/Helicopter/Bird/Background) | **MIT** (HF `license:mit`) | candidate — fetched 2026-07-17 ([Javvanny/yolov8m_flying_objects_detection](https://huggingface.co/Javvanny/yolov8m_flying_objects_detection)) |
| yolov8n COCO | `yolov8n.onnx` | 12.8 MB | YOLOv8n @640, 80-class | AGPL-3.0 | untuned baseline, no drone class — not re-evaled here |
| vittrack | `object_tracking_vittrack_2023sep.onnx` | 0.7 MB | ViT tracker @128 | BSD-3 | tracker, not a detector |

Provenance/license detail: `scripts/seeker/weights/LICENSES.md` (including the
"MIT weights, AGPL export tooling" disclosure that applies to both MIT candidates).

## 2. THE MIT-licensed drone-detection NN (the builder's named starting point)

**Identified: `doguilmak/Drone-Detection-YOLOv11x` — MIT (uploader-declared, HF API
`license:mit`), single-class `drone`, YOLO11x (56.9 M params), trained on ~1k real drone
photos (Kaggle amateur-drone corpus). Status: ALREADY FETCHED** (`weights/drone_yolo11x.pt`,
114.4 MB; our 1280² ONNX export 228.2 MB). It is the only on-hand model that detects real
drones well (§4). The second MIT option, `Javvanny/yolov8m_flying_objects_detection`
(25.9 M params), barely calls real DJIs "Drone" (§4 + transfer-bet 1/3) — kept only for its
labeled Bird channel.

**"Must it be scaled down?" — YES, and not by shrinking THIS checkpoint.** The deployment
anchor is **YOLO11n @640 INT8 on the Hailo-8L AI HAT+ (13 TOPS)** — Pi 5 CPU YOLO is
5–10 fps, non-viable at terminal LOS rates (`docs/nn_transfer_plan.md`; ADR-0016 Hailo tiers:
~105 fps NPU-kernel / ~35 fps end-to-end EXPECTED / 15–20 WORST for the n-scale family).
A 56.9 M-param x-scale model at 1280² has no usable operating point on a 13-TOPS 8L, and
weights do not transfer across YOLO scales (x→n is not a resize). Also note the standing
recipe rule: fine-tunes start from **COCO-init** (`docs/real_data_pipeline.md`), never from
another model's weights. So "start from the MIT NN" is operationally:

1. **Teacher / auto-label assist:** run `yolo11x_mit` offline over collected real media to
   propose boxes (human-spot-checked), cutting hand-labeling for the domain corpus;
2. **Upper-bound reference:** its color AP on each eval set bounds what a real-photo-trained
   detector can see there;
3. **Deploy weights:** a fresh **YOLO11n @640 fine-tune (COCO-init, grayscale-eval'd)** on
   the real corpus — that is the artifact that compiles to Hailo-8L INT8 at usable fps.
   License note: YOLO11n fine-tunes are AGPL-3.0 (the MIT label does NOT carry over);
   the Apache-2.0 NanoDet-Plus lane (`nn_transfer_plan.md` §2.2) remains the
   license-clean fallback if AGPL is unacceptable for the deployed artifact.

## 3. Eval corpus — real, labeled, license-clean (and its two audit findings)

**Mendeley "YOLOv7 Segmented Drone vs Bird"** v5 (DOI
[10.17632/6ghdz52pd7.5](https://data.mendeley.com/datasets/6ghdz52pd7/5)), **CC BY 4.0**,
fetched 2026-07-21: `Dataset.zip` **1,128,956,760 B (1.13 GB)**, sha256
`5bf5b257…ac51c` **verified** against the Mendeley manifest. 20,952 real images
(train 18,323 / valid 1,740 / test 889), YOLO bbox labels, 2 classes `['Drone','Bird']`.
Roboflow-derived (stretch-resized 640², train split carries rotation/shear/brightness
augmented copies). Local: `scripts/seeker/nn_tier/data/dvb_extract/` (+ the uploader's
raw-source-links xlsx: `dvb_raw_links.xlsx`).

**Audit finding 1 — the Bird partition's class bytes are wrong.** Filename prefixes encode
the true partition (`DT` = Drone Test, `BT` = Bird Test, …). In the test split, **271/361
`BT` (bird) label files carry class-0 "Drone" boxes drawn on birds** (eyeball audit, 15
rendered samples: all birds, several boxes sloppy). `DT` class-0 boxes audited correct
(tight, on the drone). **Eval policy therefore: positives = `DT` images with their class-0
GT; negatives = `BT` images with labels DISCARDED (treated as drone-free bird images).**
This is segment-truth by filename, disclosed here; implemented as
`--pos-prefix DT --neg-prefix BT`.

**Audit finding 2 — this corpus is NOT small-object.** DT test drone boxes: median width
**0.43 of frame** (~277 px @640), p10 0.23, **0.0% under 32 px**. So this scoreboard
measures **real-appearance generalization on large, obvious drones** — NOT the deployment
regime (a few-px-to-tens-of-px quad at range). A model failing HERE fails easy real
imagery; a model passing here has proven nothing yet about acquisition range.

**Split policy (anti-mirage):** we train NOTHING in this task; the corpus is
**train-disjoint from every evaluated model by construction** (v2 lineage = Gazebo renders
only; `yolo11x_mit` = Kaggle amateur-drone corpus; `flyingobj_mit` = its own 5-class corpus
— incidental web-image overlap with the two real-photo candidates cannot be fully excluded
and is noted). Sampling is deterministic: **n = 350 test-split images (200 DT positives,
200 GT boxes + 150 BT bird negatives), seed 0**. ⚠️ For the LATER fine-tune phase this
corpus's own train/test splits must NOT be trusted as source-disjoint (Roboflow
augmentation predates the split; per-image source identity is unrecoverable) — use it as a
TRAIN-side corpus only, and validate on held-out SOURCES from elsewhere (the standing
ADR-0061 rule).

## 4. Scoreboard — REAL test images, color AND grayscale (OV9281 modality)

Deployed operating point conf = 0.25 (`v3_onnx_infer.DEPLOY_CONF`); match IoU 0.5; NMS 0.45;
no self-mask (sim-geometry gate, meaningless on this footage). "False-fire" = fraction of
the 150 drone-free bird images with ≥1 drone-class fire @0.25 — birds are the classic
confuser and the deployment sky's main inhabitant. Latency = this box's CPU (24-thread
Ryzen-class, ORT 8 threads), NOT a Pi/Hailo number.

| model | mode | AP50 | recall@.25 | precision@.25 | false-fire (birds) | ms/frame |
|---|---|---|---|---|---|---|
| v2_deployed (sim, DEPLOYED) | color | 0.0017 | 0.005 | 0.002 | **0.667** | 33 |
| v2_deployed (sim, DEPLOYED) | gray | 0.0008 | 0.030 | 0.006 | **0.967** | 30 |
| quad_hardened (sim) | color | 0.000 | 0.000 | 0.000 | 0.120 | 28 |
| quad_hardened (sim) | gray | 0.000 | 0.000 | 0.000 | 0.007 | 28 |
| quad_rebal (sim) | color | 0.000 | 0.000 | 0.000 | 0.327 | 28 |
| quad_rebal (sim) | gray | 0.000 | 0.000 | 0.000 | 0.040 | 28 |
| yolo11x_mit (MIT, real-photo) | color+gray | *daemon in flight* — rows land in `logs/nn_tier/baseline_scoreboard_recon1.csv` | | | | ~1800–2100 |
| flyingobj_mit (MIT, real-photo) | color+gray | *daemon in flight* — same CSV | | | | ~800 |

**Reading the measured rows:** the sim-trained family divides into two failure modes on
real imagery — the deployed v2 is a *hallucinating* failure (fires everywhere, sees
nothing: recall 0.5–3%, bird false-fire 67–97%), while hardened/rebal are *silent*
failures (their sim phantom-hardening rounds taught them to reject everything real:
recall exactly 0, false-fire ≤ 0.33). Either way: **zero transferable recall from any
sim-trained checkpoint** — the labeled confirmation of the transfer-bet smoke read, and
the measured floor "refine HEAVILY" starts from. The daemon (setsid-detached,
reaper-safe) finishes the two MIT-model evals and writes the full CSVs; progress:
`logs/nn_tier/baseline_progress_recon1.log`.

## 5. Mono-vs-color delta (camera_paper_check item 4, now measured on real frames)

Measured on the SAME 350 real frames, only the modality changed (BGR→GRAY→3ch, exactly
what feeding the mono OV9281 into a color-trained net does):

- **v2_deployed:** bird false-fire **0.667 → 0.967** (+45% relative), and its (noise-level)
  detections rearrange (recall 0.005→0.030, AP50 0.0017→0.0008 — both floors). The mono
  probe on 30 bird frames shows gray fires on **30/30** with higher confidence (median
  top-score 0.32→0.43). Direction: **grayscale makes the deployed sim net WORSE in the
  dangerous direction** (more false tracks for the streak gate to survive), not merely
  "a recall penalty" — camera_paper_check item 4's UNVERIFIED verdict is now part-measured.
- **hardened/rebal:** gray suppresses even their false fires (0.12→0.007, 0.33→0.04) —
  consistent with the silent-failure mode (real texture in any modality reads as
  "not my sim target").
- **MIT real-photo models:** gray delta lands with the daemon rows (same CSV).

## 6. What bounds these claims (read before quoting)

1. **Frame metrics ≠ flight performance** (ADR-0061; `candidate_nn_shortlist.md` §5). This
   scoreboard ranks starting material for the fine-tune; it certifies no seeker.
2. **Large-object corpus** (§3 finding 2): real small-target acquisition is unmeasured here;
   the Det-Fly aspect problem (air-to-air, small) remains open — Det-Fly itself is
   OneDrive/Baidu-gated, not scriptably fetchable (checked: no HF mirror), license ambiguous
   (`candidate_nn_shortlist.md` §4).
3. **Domain mismatch remains:** these are worldwide web photos, not Central-Oregon
   sagebrush/juniper/basalt with dust and hard shadows; the domain corpus for training is the
   next phase's job (this task is the measured starting line).
4. **Grayscale here = channel-replicated color-net input**, which is exactly what feeding the
   OV9281 into these color-trained nets would do. A grayscale-TRAINED net is the fix, not a
   channel hack (`docs/camera_paper_check.md` item 4).
5. **The deployed system's streak/consistency gate is not in this test** — raw false-fire is
   a worst-case upper bound on the fielded system (`transfer_bet_kill_test.md` §6) — but a
   97% per-frame bird false-fire rate (v2, gray) is far beyond what any streak gate was ever
   validated against.

## 7. Reproduce

```bash
# self-test (no dataset/network; scorer math + ONNX smoke; exits 0/1):
.venv-seeker/bin/python scripts/seeker/nn_tier/baseline_eval.py --self-test

# fetch + verify corpus (1.13 GB; sha256 in the Mendeley manifest):
curl -L -o scripts/seeker/nn_tier/data/dvb_mendeley.zip \
  "https://data.mendeley.com/public-files/datasets/6ghdz52pd7/files/20ed32e1-7e9e-4177-a15d-87fae70770cf/file_downloaded"
sha256sum scripts/seeker/nn_tier/data/dvb_mendeley.zip   # 5bf5b257...ac51c
unzip -q scripts/seeker/nn_tier/data/dvb_mendeley.zip -d scripts/seeker/nn_tier/data/dvb_extract

# the scoreboard run (≈40 min CPU; deterministic):
.venv-seeker/bin/python scripts/seeker/nn_tier/baseline_eval.py \
  --data scripts/seeker/nn_tier/data/dvb_extract/Dataset --split test \
  --drone-class 0 --pos-prefix DT --neg-prefix BT \
  --max-pos 200 --max-neg 150 --seed 0 --tag recon1
```
