# Mono-vs-color camera decision — the residual bird-discrimination cell (CLOSED)

*2026-07-21. Closes the one open cell left on the mono-vs-color camera decision
(`docs/nn_tier/PLAN.md`, `baseline_scoreboard.md` §5). The camera decision itself is
already **KEEP MONO** (the OV9281 global-shutter mono sensor is cheaper/simpler, and
PLAN.md already shows a mono-NATIVE model, `n-mono`, beating the blind sim baseline by
a wide margin). What was never isolated: does training NATIVELY on color meaningfully
improve drone-vs-bird discrimination over training NATIVELY on gray, for the SAME
architecture / recipe / corpus? A large color advantage would be a flag worth
re-litigating the camera over; a small/zero one CONFIRMS keep-mono. Every number below
traces to a run (`logs/nn_tier/*.csv`) or a disclosed derivation.*

## TL;DR verdict — KEEP MONO, confirmed. Color buys nothing for bird discrimination.

Native-vs-native (n-mono scored gray, n-color scored color — each in its own training
modality):

| axis (native vs native) | n-mono (gray) | n-color (rgb) | delta | significant? |
|---|---|---|---|---|
| **bird false-fire** (DVB birds, clean n=293) | **3.41%** (10/293) | **3.07%** (9/293) | **−0.34 pts** (color slightly lower) | **NO** (z=0.23, p=0.82) |
| held-out AP50 (n=4175) | **0.4421** | 0.4373 | −0.005 (mono higher) | — |
| held-out recall@25 | **44.17%** | 42.69% | −1.5 pts (mono higher) | no (p=0.17) |
| held-out precision@25 | **71.41%** | 69.25% | −2.2 pts (mono higher) | — |
| **desert-plate false-fire** (n=417, the deployment clutter) | **4.56%** (19/417) | **14.63%** (61/417) | **+10 pts WORSE for color** | **YES** (z=−4.9, p<1e-6) |

**The answer to the open question is "no."** Color's bird-discrimination advantage is
**zero within noise** — 3.07% vs 3.41% is a 0.34-point difference a two-proportion
z-test cannot distinguish from zero (p=0.82), and it points the "wrong" way (color
slightly *fewer* false fires, but not meaningfully). On every other axis mono ties or
beats color, and on the deployment-relevant desert clutter (sagebrush/juniper/basalt
plates — the real false-positive generator at the Oregon site) the color-native model
is **significantly WORSE**, false-firing 3× as often. So chroma is **not load-bearing**
for this discrimination problem: the earlier 0.667→0.967 regression was an
out-of-domain artifact of feeding a color-trained net gray (§1), not evidence that a
mono model gives up bird/clutter rejection. **Nothing here flags the camera; the free
A/B confirms keep-mono.**

## 1. Why this cell was still open

`baseline_scoreboard.md` §5 measured a 0.667→0.967 bird false-fire jump when the
DEPLOYED sim-trained net (`v2_deployed`, trained on COLOR Gazebo renders) was fed
grayscale frames. That number is real but it is an **out-of-domain (OOD) artifact, not
proof chroma is load-bearing**: it measures a color-trained net seeing a modality it
never trained on — a different question from "does a model trained FROM SCRATCH on gray
lose discrimination it would have kept if trained on color." The PLAN.md sweep answered
the camera question (mono is the deployment sensor either way) but never ran the
apples-to-apples native-vs-native A/B. This task ran it.

## 2. Method

### 2.a Bird false-fire scorer

New `scripts/seeker/nn_tier/eval_bird_negatives.py` (self-test PASS), built rather than
reusing `baseline_eval.py`'s DVB harness because that harness used a bounded max-neg
SAMPLE (n=150) and did not surface the fire-confidence distribution — both needed here.
`v3_onnx_infer.load_session` / `baseline_eval.{run_model_on_image,to_gray3}` are imported
read-only (no existing eval script edited, per the task rule), so the decode/letterbox
inference path is byte-identical to every other nn_tier eval.

Bird negatives: the Mendeley "YOLOv7 Segmented Drone-vs-Bird" corpus's TEST-split Bird
partition (`.../dvb_extract/Dataset/test/images/BT*`, n=361 real bird photos). Every
`BT*` image is scored as drone-FREE regardless of its on-disk label (the corpus's own
Bird-class boxes are largely mislabeled onto birds — `baseline_scoreboard.md` §3 finding
1 — so labels are not read; any fire at conf ≥ 0.25 is definitionally a false fire).

### 2.b Leakage trap found and closed (headline uses the disjoint n=293)

**68 of the 361 DVB test bird images were already used as TRAIN-split negatives in the
nn_tier corpus.** `prepare_nn_tier_dataset.py`'s DVB ingest POOLS all three of DVB's own
splits ("their random split is ignored") and re-samples `dvb_bird` train negatives from
the pool, so DVB-test birds leaked into nn_tier TRAIN. Scoring "held-out" bird false-fire
on the naive 361-image folder would grade n-mono/n-color on 68 images they trained on,
biasing the rate down. `eval_bird_negatives.py:leaked_dvb_bird_test_basenames()`
re-derives the exact leaked set from `manifest_all.csv` (uid `dvb_bird_all_BT__<N>_`;
`BT` distinguishes DVB-test from `BTR` DVB-train and `BV` DVB-valid) and excludes it.
**Headline = disjoint n=293; the leak-included n=361 number is reported alongside,
clearly labeled, never as the verdict.** (Verified: all three nets shift only slightly
between the two sets, so the leak did not dominate — but the clean number is the honest
one.)

### 2.c n-color training arm

New `scripts/seeker/train_daemon_nn_tier_color.py` — a copy of
`train_daemon_nn_tier.py`'s recipe (yolo11n.pt COCO-init, NEVER sim weights; imgsz 640;
batch 32; epochs 60; patience 20; seed 0; device 0; workers 8), pointed at
`dataset_color.yaml` instead of `dataset.yaml`. The color and gray trees are the SAME
images / SAME source-video-scene-disjoint split — label files are byte-identical between
`color/labels/` and `gray/labels/` (verified), so n-mono and n-color trained on identical
train/val/test membership; the ONLY difference between the arms is color channel content.
Added a `nvidia-smi --query-compute-apps` GPU self-serialization wait (process-table read
only — no pkill/pgrep, no inline literal pattern) per the one-training-at-a-time rule.
setsid-detached, reaper-safe. Ran 46 epochs (early-stopped by patience at epoch 45 best),
~75 s/epoch on the 4070, ~57 min wall. Exported to
`scripts/seeker/weights/nn_tier/n-color.{pt,onnx}`.

### 2.d n-color held-out scoring

New `scripts/seeker/nn_tier/eval_n_color.py` — a copy of `eval_n_mono.py`'s eval
machinery (same greedy IoU matcher, AP50, scale/position bins, `box_scoring.box_hits_gt`
gate, read-only) with one change: images read from the manifest `color_img` column
instead of `gray_img`, so n-color is scored in ITS OWN native modality on the SAME
source-disjoint held-out TEST split (n=4175 images / 4315 GT / 469 drone-free negatives)
that `eval_n_mono.py` scored n-mono on in PLAN.md.

## 3. Results

### 3.a Bird false-fire (DVB test Bird partition, clean held-out n=293)

| model | fed modality | n | false-fire@.25 | 95% CI (Wilson) | top-score median | top-score mean |
|---|---|---|---|---|---|---|
| **n-mono** | gray (**native**) | 293 | **3.41%** (10/293) | [1.86, 6.17] | 0.331 | 0.351 |
| **n-color** | rgb (**native**) | 293 | **3.07%** (9/293) | [1.62, 5.73] | 0.306 | 0.342 |
| n-mono | color (OOD cross-ref) | 293 | 5.46% (16/293) | [3.39, 8.69] | 0.335 | 0.346 |
| v2_deployed (sim, for scale) | gray | 293 | 95.22% (279/293) | [92.14, 97.13] | 0.423 | 0.472 |
| v2_deployed (sim, for scale) | color | 293 | 65.19% (191/293) | [59.57, 70.41] | 0.365 | 0.425 |

Provenance: `logs/nn_tier/eval_bird_negatives_{nmono_bird_ab,ncolor_bird}.csv`. Two-prop
z-test n-mono-gray vs n-color-rgb: **z=0.23, p=0.82 → NOT significant.** Leak-included
(n=361, DO-NOT-USE-AS-VERDICT) rows sit in the same CSVs for auditing the leak effect.

**Reading it:** a mono-native and a color-native model, trained identically on the same
images, false-fire on real birds at statistically indistinguishable rates (3.4% vs 3.1%),
with near-identical confidence distributions (median top-score 0.33 vs 0.31 — both well
below any streak-gate-survivable level). Color chroma does not help the model tell a bird
from a drone. (For contrast, the OOD cross-ref rows show the sim-trained v2's brittleness:
95% gray vs 65% color — a 30-point modality swing on a NON-domain-matched net. A
real-media-native model in either modality is an order of magnitude cleaner AND far less
modality-sensitive: n-mono fed the "wrong" (color) modality only rises 3.4%→5.5%,
CIs overlapping.)

### 3.b Held-out overall (source-disjoint TEST, n=4175 / 4315 GT / 469 neg)

| model | mode | AP50 | recall@25 | precision@25 | false-fire@25 (all neg) | provenance |
|---|---|---|---|---|---|---|
| **n-mono** | gray (**native**) | **0.4421** | **44.17%** | **71.41%** | **4.90%** (23/469) | `eval_n-mono_heldout.csv` |
| **n-color** | rgb (**native**) | 0.4373 | 42.69% | 69.25% | 13.43% (63/469) | `eval_n-color_ncolor_heldout.csv` |
| n-mono | color (OOD cross-ref) | 0.4446 | 44.45% | 72.76% | 5.76% | `eval_n-color_nmono_color_crossref.csv` |
| v2_deployed (sim, for scale) | gray | 0.0003 | 1.11% | 0.51% | 88.49% | `eval_s-mono_summary_cmp1.csv` |

n-mono ties/beats n-color on all four detection axes; the recall gap (44.17 vs 42.69) is
not significant on n=4315 boxes (z=1.39, p=0.17). The overall false-fire gap (4.9% vs
13.4%) is real and is driven entirely by desert plates (§3.c).

### 3.c False-fire by negative source (the deployment-relevant cut)

| neg source | n | n-mono (gray) | n-color (rgb) | note |
|---|---|---|---|---|
| **plates** (high-desert terrain — the real clutter) | 417 | **4.56%** (19) | **14.63%** (61) | color 3× worse, **z=−4.9, p<1e-6 SIGNIFICANT** |
| nps (air-to-air empty frames) | 51 | 7.84% (4) | 3.92% (2) | both tiny n |
| dut | 1 | 0% | 0% | — |

Provenance: `logs/nn_tier/eval_n-{mono,color}_breakdown_*heldout.json` → `neg_by_source`.
**This is the load-bearing extra finding:** on the sagebrush/juniper/basalt desert
negatives that stand in for the actual deployment site, the color-native model
hallucinates drones **3× more often** than the mono-native one. Color doesn't just fail
to help — for the clutter we actually care about, it hurts, significantly. (Modality is
confounded with this-one-model's fit here — see caveat 1 — but the direction is the
opposite of "color helps," so it cannot rescue a keep-color case.)

Small-target recall (the ≤40 px terminal band) is a wash: n-mono 0.08/0.425/0.477 vs
n-color 0.027/0.376/0.484 across the 0-12/12-24/24-40 px bins — no color advantage in the
bins that matter for acquisition range either.

## 4. THE ANSWER

**Is color's bird-discrimination advantage large enough to reconsider the camera? NO —
there is no advantage.** The native-vs-native bird false-fire delta is 0.34 points and
statistically zero (p=0.82); color is, if anything, marginally behind on overall
detection and **significantly behind on desert-clutter rejection** (the deployment-
relevant false-positive source). The residual bird risk the held-out set didn't isolate
is now measured and it is **the same for both cameras** (~3% at conf 0.25, low-confidence
fires a streak gate is designed to reject). The 0.667→0.967 grayscale regression stays
correctly diagnosed as an OOD color-net-fed-gray artifact, **not** a chroma-is-essential
signal. **Keep-mono is confirmed on the merits, for free (no camera purchased).** The bird
problem is real but it is a *training-data / hard-negative-mining* problem (Tier-3 Oregon
desert + raptor captures, PLAN.md §3), not a *sensor-modality* problem — and it will be
attacked identically whether the camera is mono or color, so the cheaper mono sensor wins.

### Caveats (honest bounds — read before quoting)
1. **One training run per arm (seed 0).** Run-to-run YOLO variance is a real confound for
   the ~1.5 pt AP/recall deltas and even the desert-plate gap. But the BIRD number — the
   actual question — is a null in both directions; no plausible training-noise story turns
   "color meaningfully helps bird discrimination" true. A paired multi-seed run would
   tighten §3.b/§3.c but cannot flip the §3.a verdict.
2. **Desert-plate false-fire is color-plates-for-n-color vs gray-plates-for-n-mono** —
   the correct native-vs-native framing, but it confounds "modality" with "this model's
   fit to this clutter." The direction (color worse) still only strengthens keep-mono.
3. **Public media, not our target / camera / site.** Same standing caveat as all of
   PLAN.md: these are frame metrics on held-out SOURCES, NOT a flight gate. n=293 birds is
   modest power — it can rule out a LARGE color advantage (the decision-relevant question),
   not resolve sub-2-point differences.
4. The AprilTag/`gt_*` honesty boundary is untouched — labels score only; nothing here
   feeds a live seeker; no sim was booted; no deployed weights modified.

## 5. Reproduce

```bash
# self-tests (no training / no dataset-network; exit 0/1)
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_bird_negatives.py --self-test
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_n_color.py --self-test

# bird false-fire: n-mono/v2 (both modes) + n-color (native color), clean+leak-included
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_bird_negatives.py \
    --models n_mono v2_deployed --modes gray color --include-leaked --tag nmono_bird_ab
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_bird_negatives.py \
    --models n_color --modes color --include-leaked --tag ncolor_bird

# n-color training (setsid-detached, GPU-serialized, ~57 min on the 4070)
setsid .venv-seeker-train-gpu/bin/python scripts/seeker/train_daemon_nn_tier_color.py \
    > logs/nn_tier/train_ncolor_daemon.log 2>&1 < /dev/null &

# held-out scoring, native modality per arm (source-disjoint test, n=4175)
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_n_color.py --models n_color --per-source --tag ncolor_heldout
# (n-mono gray held-out is PLAN.md's logs/nn_tier/eval_n-mono_heldout.csv)
```

*Numbers trace to: `logs/nn_tier/eval_bird_negatives_{nmono_bird_ab,ncolor_bird}.csv`
(bird false-fire), `eval_n-mono_heldout.csv` + `eval_n-color_ncolor_heldout.csv`
(held-out), `eval_n-{mono,color}_breakdown_*heldout.json` (per-source / per-scale),
`scripts/seeker/runs/nn_tier_n_color/results.csv` (training trajectory). Models:
`scripts/seeker/weights/nn_tier/{n-mono,n-color}.onnx`.*
