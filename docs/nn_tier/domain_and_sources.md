# NN tier — target visual domain + real-media corpus sources (Recon phase)

*Recon deliverable for the markerless-NN tier (builder directive 2026-07-21: MIT-licensed
drone NN as the starting point, refined HEAVILY on REAL media, matched to a Central
Oregon high-desert deployment, sized for the ordered hardware). Desk research only —
existence/license/size of every source was web-verified 2026-07-20/21; NOTHING large was
fetched in this phase (that is the Dataset phase's job). Companion docs:
`docs/candidate_nn_shortlist.md` (checkpoint shortlist), `docs/nn_transfer_plan.md`
(architecture/license plan), `docs/real_data_pipeline.md` (the eventual REAL-capture
pipeline this corpus is a head start for), `docs/camera_paper_check.md` item 4 (the
mono-vs-color confound that drives the grayscale policy below).*

**Standing honesty note:** everything trained on this corpus is a DOMAIN-MATCHED HEAD
START / upper bound, not deployment weights — no imagery of OUR target (the unbuilt
Kakute-FC 5" quad) exists yet. Final weights come from the AprilTag-auto-labeled real
captures (`real_data_pipeline.md`). Anti-mirage rules apply throughout: split by
SOURCE/video, never random frames; COCO-init; ~15–20% negatives; numbers trace to runs.

---

## 1. The target visual domain — Central Oregon high desert / badlands

Deployment environment: the sagebrush steppe east of the Cascades (Bend/Redmond area,
~3,000–4,000 ft). Reference landscape: the Oregon Badlands Wilderness — "craggy
hillsides dotted with sagebrush and juniper, dry river canyons, and fortress-like
basalt features" on an ~80,000-year-old Newberry lava flow; big sagebrush, rabbitbrush,
bunchgrasses (Idaho fescue, bluebunch wheatgrass), western juniper; <12 in annual rain
([BLM](https://www.blm.gov/visit/oregon-badlands-wilderness),
[Wikipedia](https://en.wikipedia.org/wiki/Oregon_Badlands_Wilderness)).

### 1.1 Background classes (what the camera will actually see behind the target)

| class | visual character (and in OV9281 GRAYSCALE terms) |
|---|---|
| **bright sky** | deep-blue high-altitude clear sky or thin cirrus → in mono: a NEAR-UNIFORM BRIGHT field. The dominant background at up-tilted camera angles; the easiest detection background and the one the sim already covers best. |
| **sagebrush steppe** | gray-green shrub mottle at 0.5–1.5 m scale over tan soil → mid-gray HIGH-FREQUENCY mottled texture. The dominant below-horizon background; a small gray quad against it is the HARD case. |
| **western juniper** | scattered dark rounded trees, 2–10 m, long hard shadows → isolated dark blobs a detector can confuse with a near drone; shadow edges as strong as object edges. |
| **basalt / volcanic rock** | dark gray-black blocky rimrock, pressure ridges, lava tumuli → large dark low-texture regions with sharp bright-dark boundaries at the skyline. |
| **tan/ochre/rust soil + dry grass** | pale tan pumice-derived soil, cured bunchgrass, dirt roads → BRIGHT ground, sometimes brighter than the sky near the horizon (inverts the usual dark-ground/bright-sky prior). |
| **dust / haze** | vehicle- and prop-raised dust, summer wildfire haze → local contrast loss, soft edges, and a real seeker stressor (own-prop dust on launch). |
| **hard shadows** | high sun + clear dry air → the harshest shadow contrast of any common test venue; shadow-side of the target can read near-black while the sky is near-saturation. |
| **horizon skyline** | juniper/rimrock silhouettes; distant Cascade peaks (sometimes snow) → strong horizontal high-contrast edge, the classic false-positive generator (already a known failure mode in this repo — horizon-reject gate, `seeker_nn_findings.md`). |
| **wildlife distractors** | raptors are RESIDENT here (prairie falcons, golden eagles per BLM) plus ravens, swallows → the bird-vs-drone discrimination problem is guaranteed at this site, not hypothetical. |

### 1.2 Lighting regime

- **Intense direct sun, low humidity, thin air** → very high scene dynamic range: a
  global-shutter mono sensor at the ≤1 ms exposures we plan (`camera_paper_check.md`
  item 3) will see near-saturated sky + near-black shadow in one frame.
- **Hard shadow edges everywhere** (no cloud diffusion most summer days) — a detector
  trained on overcast/urban footage has never seen shadow contrast this strong.
- **Glint/flare**: specular glints off the target's props/canopy and lens flare at
  low sun angles.
- **Dust haze near the ground**; occasional smoke haze (summer).

### 1.3 How this differs from generic drone datasets — and what it drives

Generic public drone sets are mostly: urban campus (USC, DUT), green fields /
overcast European sky (Drone-vs-Bird corpora), Swedish airport aprons (Halmstad),
or pure sky. The high-desert deltas that matter:

1. **Below-horizon clutter statistics** — sagebrush mottle is high-frequency,
   mid-gray, isotropic; urban clutter is rectilinear. A detector's false-positive
   population will be DIFFERENT here. → weight FIELD/MOUNTAIN background sub-splits
   of any dataset (Det-Fly explicitly tags these) and add REAL desert plates.
2. **Brightness inversion** — tan ground can out-bright the sky near the horizon;
   "target is dark blob on bright sky" priors partially break below the skyline.
3. **Extreme contrast + hard shadows** — drive lighting augmentation toward the
   harsh/washed end, not the dim end.
4. **Dust/haze** — local contrast-loss augmentation has a physical anchor here.
5. **MONO camera** (the big one, `camera_paper_check.md` item 4): the OV9281 is
   monochrome. ALL training and ALL eval for the deployment-track model run in
   GRAYSCALE (convert every RGB source at ingest; keep 3-channel replication only as
   the tensor format). This actually SHRINKS the color-domain gap between datasets
   (a green field and a tan steppe are closer in luminance than in color) but leaves
   the texture/contrast/clutter gaps above fully intact — and it makes the
   grayscale-native AOT corpus (§2) unusually valuable.

---

## 2. Real-media corpus sources (verified leads, ranked)

Ranking favors (a) permissive license, (b) AIR-TO-AIR / nose-on aspect (the deployed
viewpoint — the interceptor camera sees the target from another aircraft's vantage,
not from the ground looking up), (c) background match to §1, (d) fetchability.
**Every source below = one SPLIT UNIT** (the anti-mirage rule: hold out whole
sources/videos, never random frames). Sizes marked *est.* are estimates to verify at
fetch time; log actual bytes in the Dataset phase.

| # | name | license | aspect | ~count | bg match | fetch | size (GB) |
|---|---|---|---|---|---|---|---|
| 1 | **Det-Fly** ([GitHub](https://github.com/Jake-WU/Det-Fly)) | MIT LICENSE file in repo; data hosted off-repo (OneDrive/Baidu) — **ambiguity already logged** in `candidate_nn_shortlist.md` §4 (email authors before any product use); Roboflow mirror tagged CC BY 4.0 | **AIR-TO-AIR** — front 36.4% / top 32.5% / bottom 31.1% (the best aspect match found anywhere) | 13,271 imgs @ 3840×2160 | **HIGH-med** — sky/urban/**field**/**mountain** each ~20–30%; field+mountain sub-splits are the closest real analog to high desert in any public set | OneDrive links in README (manual, not scriptable) or Roboflow export (API key) | ~9 *est.* full-res (0.5–1 for a Roboflow 640 export — prefer FULL-RES: the target is small-object) |
| 2 | **Amazon Airborne Object Tracking (AOT)** ([registry.opendata.aws](https://registry.opendata.aws/airborne-object-tracking/)) | **CDLA-Permissive-1.0** (clean, commercial OK) | **AIR-TO-AIR** (aircraft-mounted camera, planned airborne encounters incl. small UAS, helicopters, aircraft, birds) | 4,943 sequences, 5.9M frames, 3.3M labels — **8-bit GRAYSCALE** 2448×2048 (native mono-sensor match!) | MED — sky/cloud/high-altitude terrain; little ground clutter, but the only large GRAYSCALE-native corpus | `aws s3 cp --no-sign-request` per-sequence prefixes | full corpus is **multi-TB — DO NOT bulk-fetch**; targeted labeled-sequence subset ~2–5 |
| 3 | **Purdue/NPS "UAV_Dataset" (NPS-Drones)** ([Purdue page](https://engineering.purdue.edu/~bouman/UAV_Dataset/)) | **BSD-3-Clause** (stated on page) | **AIR-TO-AIR** — GoPro on a delta-wing filming up to 8 UAVs | 50 videos, 70,250 HD frames (targets 10×8 → 65×21 px — right size regime) | MED — sky + coastal-scrub terrain; some dry-brush frames | direct HTTP from the Purdue page | ~3 *est.* |
| 4 | **DUT Anti-UAV** ([GitHub](https://github.com/wangdongdut/DUT-Anti-UAV)) | **MIT** | ground-observer-up (mixed elevation) | 10,000 det images + 20 tracking seqs | MED-low — varied sky/cloud/buildings/some terrain | GitHub-linked download (scriptable) | ~2 *est.* |
| 5 | **YOLOv7-Segmented Drone-vs-Bird** ([Mendeley DOI 10.17632/6ghdz52pd7.5](https://data.mendeley.com/datasets/6ghdz52pd7/5)) — already vetted in `bird_discrimination_design.md` | **CC BY 4.0**, no DUA | mixed, mostly ground-up | 20,925 imgs (12,474 drone / 8,451 bird) **with segmentation masks** | LOW-med — but the masks make it the **cutout donor** for §3 compositing, and it is the adopted bird-class source | direct Mendeley HTTP | ~2.5 *est.* |
| 6 | **Halmstad multi-sensor drone dataset** ([GitHub](https://github.com/DroneDetectionThesis/Drone-detection-dataset)) | **CC0-1.0** (public domain — fetch-verified this task) | ground-up | 285 visible videos (+365 IR, skip), 203k annotated frames total; classes Drone/Bird/Airplane/Helicopter | LOW — Swedish airport | GitHub-linked download; visible-only subset | ~4 *est.* (visible subset) |
| 7 | **USC MCL drone dataset** ([HF](https://huggingface.co/datasets/uscmcl/MCL_drone_dataset), [MCL page](https://mcl.usc.edu/mcl-drone-dataset/)) | MIT-style grant (stated on page) | ground-up | 30 videos @1080p, labels every 15th frame | LOW — urban campus | HF hub (trivially scriptable) | ~3 *est.* |
| 8 | **Roboflow Universe permissive sets** — e.g. [drone-detection vg2iu](https://universe.roboflow.com/drones-lfobz/drone-detection-vg2iu) (5.1k, drones/birds/heli/plane), [drone_mil](https://universe.roboflow.com/military-drone/drone_mil-u8fqk) (7.3k), [colleage drone-dataset](https://universe.roboflow.com/colleage-7thf7/drone-dataset-pw8lv) (1.9k incl. DJI-FPV class) | CC BY 4.0 per-set — **re-verify EACH set's license tag at fetch** (Roboflow defaults can be uploader-set) | mixed | ~14k combined | LOW-med, varied | Roboflow export API (key needed) | ~1.5 *est.* |
| 9 | **Pexels / Pixabay desert + FPV clips** ([Pixabay "desert drone" 6.7k+ clips](https://pixabay.com/videos/search/desert%20drone/), [Pexels FPV](https://www.pexels.com/search/videos/fpv%20drone/)) | platform free-commercial licenses (no attribution) | mostly aerial-POV (shot BY drones) → **primary value = REAL DESERT BACKGROUND PLATES** for §3; a minority of clips have a drone IN frame (filter for those) | thousands of clips; curate ~50–150 | **HIGH for backgrounds** (real arid terrain, some actual OR/high-desert) | per-clip HTTP download (scriptable) | ~2 curated |
| 10 | **YouTube CC-BY FPV freestyle/chase in desert** (two-ship chase footage = genuine nose-on air-to-air of a real 5" FPV quad — OUR exact target class) | per-video CC BY (self-reported — spot-check each video's license tag) | **AIR-TO-AIR chase** when found | opportunistic, ~10–30 usable videos | **HIGH** when desert freestyle (Utah/Nevada/Arizona common) | `yt-dlp` with CC filter + manual license check | ~2 curated |

**Rejected / held (logged so nobody re-derives them):**

- **Drone-vs-Bird challenge corpus (wosdetc)** — requires a signed DUA, non-commercial,
  email-gated ([challenge site](https://wosdetc.wordpress.com/challenge/)); the Mendeley
  set (#5) is the already-adopted substitute. Held, not fetched.
- **Anti-UAV410 / Anti-UAV benchmark data** — THERMAL-IR-centric, license terms
  unverifiable from public pages, ground-up; IR is also an already-REJECTED sensor
  ruling in this project. Rejected for this corpus.
- **FL-Drones** (TransVisDrone's third source) — permission-gated, not freely
  redistributable (already flagged in `candidate_nn_shortlist.md`). Rejected.
- **chuanenlin/drone-net** (2,664 DJI images) — **NO license** in repo → unusable
  despite convenient labels. Rejected.
- **Kaggle MAV-VID** (64 videos / 40k imgs) — real and relevant
  ([benchmark paper](https://arxiv.org/pdf/2103.13933)), but the Kaggle page's license
  could not be verified this task (page fetch 404 without login). HOLD until the
  Dataset phase confirms the license tag; do not plan around it.

**Fetch totals (report actual bytes at fetch):** recommended core = #1–#5 + curated
#9/#10 ≈ **~21 GB *est.***; minimum-viable core (Det-Fly-via-Roboflow + DUT + Mendeley
+ desert plates) ≈ **~8 GB *est.***. The standing authorization is "a few GB" — the
Dataset phase should fetch in priority order, log each size, and stop/ask at the
budget line rather than bulk-pull everything.

---

## 3. Negatives + background plan (avoiding the domain_gap_eval failure)

### 3.1 What failed before, precisely

`scripts/seeker/domain_gap_eval.py`'s background arm composites the gt-boxed target
cutout onto a **procedural** background (sky gradient + Gaussian blotch/speckle
"clutter") with an **elliptical feathered mask**, and the deployed detector read ~0%
recall at ALL strengths. Two mechanisms, both instructive:

1. **The background is out-of-distribution for ALL natural imagery** — a Gaussian
   blotch field has no natural-image texture spectrum; the detector isn't being
   stressed by "outdoor clutter," it's being handed an alien texture class.
2. **The compositing itself is detectable** — a feathered ellipse (from a BOX, not a
   mask) leaves a halo boundary and destroys local context; the "augmentation"
   perturbs exactly the local evidence the detector uses.

Lesson: **too-harsh synthetic compositing measures the compositor, not the detector.**

### 3.2 The plan: real backgrounds first, honest compositing second

1. **PRIMARY — real desert background plates, used raw as negatives.** Curate
   50–150 real high-desert stills/clips (source #9; plus Wikimedia Commons / BLM
   public-domain photos of the Oregon Badlands; plus builder phone/camera photos of
   the ACTUAL flying site once visited — free and a perfect match). Grayscale them,
   crop to training resolution, and include as **empty-label negative frames at
   ~15–20% of the corpus** (the rebal lesson) alongside bird negatives (#5, #6) and
   later real own-prop/dust frames from the rig.
2. **SECONDARY — mask-based cutout compositing onto REAL plates** (positives
   augmentation, applied to a bounded fraction — suggest ≤25–30% of positives):
   - **Cutout source:** the Mendeley segmented set (#5) provides real drone pixels
     WITH segmentation masks — composite by MASK, never by box-ellipse.
   - **Blend:** `cv2.seamlessClone` (Poisson) or mask-edge alpha with ≤2 px feather —
     no halo ring.
   - **Coherence constraints:** scale the cutout to a physically plausible px size
     for the OV9281 (~3–60 px span for 0.35 m at the ranges we care about); match
     the plate's grayscale luminance/contrast (histogram match the cutout to the
     insertion neighborhood); add matched blur/noise so the cutout isn't the sharpest
     object in frame; place BOTH above and below the skyline (below-horizon
     sagebrush clutter is the hard case §1 identifies).
   - **Everything grayscale** before compositing (the OV9281 policy, §1.3).
3. **VALIDATION FIREWALL (anti-mirage, non-negotiable):** composites are
   TRAINING-set material only. The held-out sets are **whole real sources/videos**,
   never composited, never random frames. If a composited-training model wins on the
   real held-out sources, the composite earned its keep; if training-set composite
   scores diverge wildly from real held-out scores, distrust the compositor first —
   that is the exact signature §3.1 documents.
4. **Sim frames:** per the phase directive, sim renders enter (if at all) only as
   extra hard-negatives (e.g., horizon/prop-arm false-positive frames), never as the
   positive corpus.

---

## 4. The "MIT drone-detection NN" — identification

**Best identification: `doguilmak/Drone-Detection-YOLOv11x` (Hugging Face, MIT-licensed
weights)** — already fetched in-repo as `scripts/seeker/weights/drone_yolo11x_1280.onnx`
and recorded in `weights/LICENSES.md`. This is the model `docs/project_state.json`'s
contradiction ledger calls "the MIT-licensed outdoor-drone-NN transfer bet," the same
one the P0.6 smoke read scored NOT-DEAD (`docs/transfer_bet_kill_test.md`) — by far
the most likely referent of the builder's "MIT drone detection NN."
Cross-ref `candidate_nn_shortlist.md`: the MIT family also includes
`Javvanny/yolov8m_flying_objects_detection` (MIT weights, adds a labeled Bird class,
also in-repo), Det-Fly (MIT repo — a DATASET, no checkpoint), DUT Anti-UAV (MIT
dataset), and TransVisDrone (MIT code, multi-frame architecture — not a drop-in).

**How to "start from" it honestly, given the ground truth:**

- It **cannot BE the deployed model**: YOLOv11x is 56.8M params; the deployment anchor
  is **yolo11n @640 INT8 on Hailo-8L**. "Scale down?" = YES by construction — the
  x-scale MIT model does not compile into the fps envelope; the deployed network is a
  nano-class model trained fresh.
- The fine-tune recipe stays **COCO-init** (`real_data_pipeline.md`; the hard rule
  bans SIM-weights init). Since the MIT weights are real-photo-trained (not sim), a
  single logged **A/B arm — COCO-init vs MIT-weights-derived init — is legitimate**,
  but at different scales (x vs n) a direct weight-init isn't even possible; the
  practical roles for the MIT model are:
  1. **Teacher / auto-label assistant** — run it (and `flying_objects_yolov8m` for
     its Bird class) over the §2 corpus and unlabeled CC footage to propose boxes for
     human-confirm labeling (it may NOT feed the deployed seeker at runtime — the
     gt/label-source-only boundary).
  2. **Benchmark arm** — its per-source scores on the §2 held-out splits are the
     "ready-made MIT model" baseline the fine-tuned nano must beat.
- License caveat carried forward from `weights/LICENSES.md`: MIT *weights*, produced
  with AGPL Ultralytics *tooling* — fine for this phase, re-verify before any product
  ship; the Apache-2.0 architecture question (`nn_transfer_plan.md` §5, YOLOX/NanoDet)
  re-opens at deployment time.

---

## 5. Hand-off to the Dataset phase

1. Fetch in §2 priority order into `scripts/seeker/nn_tier/data/<source_name>/`,
   logging bytes per source; stop at the budget line (~"a few GB" authorized; core
   list ≈ 21 GB *est.* — stage it).
2. Convert everything to grayscale at ingest; record each image's source ID for
   split-by-source bookkeeping.
3. Build the negatives pool per §3 (desert plates + birds; 15–20%).
4. Reserve WHOLE sources/videos as held-out (suggest: one Det-Fly background
   sub-split + ≥1 full video per video source + a desert-plate subset never trained on).
5. Verify at fetch: Det-Fly data license (email authors if this goes beyond
   portfolio), each Roboflow set's license tag, MAV-VID's Kaggle license (held).
