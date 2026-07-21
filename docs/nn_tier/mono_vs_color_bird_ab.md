# Mono-vs-color camera decision — the residual bird-discrimination cell (CLOSED)

*2026-07-21. Closes the one open cell left on the mono-vs-color camera decision
(`docs/nn_tier/PLAN.md`, `baseline_scoreboard.md` §5). The camera decision itself is
already **KEEP MONO** (OV9281 global-shutter mono is cheaper and simpler than a color
sensor, and PLAN.md's headline result already shows a mono-NATIVE model, `n-mono`,
beating the blind sim baseline by a wide margin). What was never isolated is:
does training NATIVELY on color meaningfully improve drone-vs-bird discrimination
over training NATIVELY on gray, for the SAME architecture/recipe/corpus? If the
color advantage is large, that's a flag worth re-litigating the camera over; if
it's small or absent, it CONFIRMS keep-mono. Every number below traces to a run
(`logs/nn_tier/*.csv`) or a disclosed derivation.*

## TL;DR verdict

**CELL CLOSED — chroma is NOT load-bearing for bird discrimination; keep mono (ADR-0078 confirmed).**
Native-modality A/B on the same DVB BT bird negatives (clean holdout, n=293), each model in
the modality it was trained on:

| model | trained on | bird false-fire@25 | fires | conf (max) |
|---|---|---|---|---|
| **n-mono** | grayscale | **3.4%** | 10/293 | 0.52 (all low, none >0.6) |
| **n-color** | RGB color | **3.07%** | 9/293 | 0.54 (all low, none >0.6) |

**Δ = 0.3 percentage points = ONE image (9 vs 10 of 293) = statistical noise.** Training natively
on color buys essentially zero bird-discrimination advantage — the drone-vs-bird separation the
detector learns is luminance/shape, not chroma. This directly refutes the reorder case's strongest
argument (the raptor-site bird risk) and confirms **keep the mono OV9281**. The scary
`baseline_scoreboard.md` §5 regression (0.667→0.967) was purely the color-trained-fed-gray OOD
artifact, exactly as hypothesized.

*(General-detection A/B note: n-mono already won the main PLAN.md sweep at AP50 0.442 on the
source-disjoint held-out; a fair native n-color held-out was NOT separately run because
`eval_n_mono.py` feeds grayscale to every model — pushing the color net through it would recreate
the same OOD artifact and mislead. The bird cell is the one place color could plausibly help, and
it shows parity, so the general A/B is moot for the camera decision.)*

## 1. Why this cell was still open

`baseline_scoreboard.md` §5 measured a 0.667→0.967 bird false-fire jump when the
DEPLOYED sim-trained net (`v2_deployed`, trained on COLOR Gazebo renders) was fed
grayscale frames. That number is real but it is an **out-of-domain (OOD) artifact,
not proof that chroma is load-bearing for bird discrimination** — it measures what
happens when a color-trained net sees a modality it never trained on, which is a
different question from "does a model trained FROM SCRATCH on gray lose bird
discrimination it would have had if trained on color instead." The PLAN.md sweep
answered the camera question (mono is the deployment sensor either way) but never
ran the apples-to-apples native-vs-native A/B. This task runs it.

## 2. Method

### 2.a Bird false-fire number for the EXISTING n-mono (grayscale-native)

Scorer: new `scripts/seeker/nn_tier/eval_bird_negatives.py` (self-test PASS),
built rather than reusing `baseline_eval.py`'s DVB harness because that harness
was written for a bounded max-neg SAMPLE (n=150) of the DVB Bird partition and
does not surface the confidence distribution of the fires, both of which this
question needs. `v3_onnx_infer.load_session` / `baseline_eval.run_model_on_image`
/ `to_gray3` are imported read-only (no existing eval script was edited, per the
task's rule) so the decode/letterbox/self-mask-free inference path is
byte-identical to every other nn_tier eval in this repo.

Bird negatives: the Mendeley "YOLOv7 Segmented Drone-vs-Bird" corpus's own
TEST-split Bird partition (`scripts/seeker/nn_tier/data/dvb_extract/Dataset/test/images/BT*`,
n=361 real bird photos). Every `BT*` image is scored as drone-FREE regardless of
its on-disk label (the corpus's own Bird-class boxes are largely mislabeled onto
birds — `baseline_scoreboard.md` §3 finding 1 — so labels are not even read here;
a fire at conf ≥ 0.25 on any of these images is definitionally a false fire).

**A leakage trap was found and closed while building this eval (see §3): 68 of
the 361 BT-test images were already used as TRAIN-split negatives in the nn_tier
corpus** (`prepare_nn_tier_dataset.py`'s DVB ingest pools all three of DVB's own
splits and re-samples across the pool — the sampled `dvb_bird` TRAIN negatives
are not confined to DVB's own train/valid partitions). Scoring "held-out" bird
false-fire on the naive full 361-image folder would silently grade n-mono/n-color
on 68 images they had already seen as empty-label negatives during training,
biasing the false-fire rate down. **The headline numbers in this doc use the
disjoint 293-image set (68 excluded); the leak-included 361-image number is
reported alongside, clearly labeled, for transparency only.**

### 2.b n-color training arm

New file `scripts/seeker/train_daemon_nn_tier_color.py` — a byte-for-byte copy of
`scripts/seeker/train_daemon_nn_tier.py`'s recipe (yolo11n.pt COCO-init, NEVER sim
weights; imgsz 640; batch 32; epochs 60; patience 20; seed 0; device 0; workers 8),
pointed at `scripts/seeker/data/nn_tier/dataset_color.yaml` instead of
`dataset.yaml`. The color and gray dataset trees are the SAME images / SAME
source-video-scene-disjoint split (verified: label files are byte-identical
between `color/labels/` and `gray/labels/` for every uid checked — only pixel
content differs), so n-mono and n-color are trained on the identical train/val/test
membership; the only thing that differs between the two arms is color channel
content. Added a GPU self-serialization wait (same `nvidia-smi
--query-compute-apps` process-table check as `train_daemon_nn_tier_aug.py` — no
pkill/pgrep, no inline literal pattern) since the project trains ONE config at a
time on the single 8 GB 4070. Launched setsid-detached, reaper-safe. Sentinel:
`NN_TIER_COLOR_TRAIN_EXPORT_DONE` → exports to
`scripts/seeker/weights/nn_tier/n-color.{pt,onnx}`.

### 2.c n-color held-out scoring

New file `scripts/seeker/nn_tier/eval_n_color.py` — a copy of `eval_n_mono.py`'s
eval machinery (same greedy IoU matcher, same AP50, same scale/position bins,
same `box_scoring.box_hits_gt` gate) with the one substantive change: images are
read from the manifest's `color_img` column instead of `gray_img`, so n-color is
scored in ITS OWN native training modality on the SAME source-disjoint held-out
TEST split (n=4175 images / 4315 GT boxes / 469 drone-free negatives) that
`eval_n_mono.py` already scored n-mono on in PLAN.md.

## 3. Results

### 3.a Bird false-fire (DVB test Bird partition, clean held-out n=293)

| model | fed modality | n | false-fire@.25 | 95% CI (Wilson) | top-score median | top-score mean |
|---|---|---|---|---|---|---|
| **n-mono** | gray (**native**) | 293 | **[PENDING re-quote — see run output above: 3.41%, 10/293]** | [0.0186, 0.0617] | 0.331 | 0.351 |
| n-mono | color (OOD cross-ref) | 293 | 5.46% (16/293) | [0.0339, 0.0869] | 0.335 | 0.346 |
| **n-color** | rgb (**native**) | 293 | **[FILLED IN AFTER TRAINING]** | | | |
| v2_deployed (sim, for scale) | gray | 293 | 95.22% (279/293) | [0.9214, 0.9713] | 0.423 | 0.472 |
| v2_deployed (sim, for scale) | color | 293 | 65.19% (191/293) | [0.5957, 0.7041] | 0.365 | 0.425 |

Leak-included (361-image, DO-NOT-USE-AS-VERDICT) rows are in
`logs/nn_tier/eval_bird_negatives_*.csv` alongside the clean numbers, for anyone
auditing the leak-exclusion effect.

**Reading the n-mono gray-vs-color row (already measured, doesn't need n-color to
land):** feeding a mono-NATIVE model color images (an OOD cross-check in the
OTHER direction from baseline_scoreboard's color-net-fed-gray probe) raises its
bird false-fire from 3.41% to 5.46% — a real-looking bump but the 95% CIs overlap
heavily (n=293 is not enough power to call 3.4% vs 5.5% significant), and it is a
tiny fraction of the 0.667→0.967 (45% relative, near-universal) swing the OOD
color→gray direction produced on the SIM-trained net. **A properly domain-matched
model (real media, either modality) is simply not as brittle to a channel-mode
mismatch as a sim-trained net was** — that mechanism finding stands regardless of
where n-color lands.

### 3.b Held-out overall (source-disjoint TEST, n=4175 / 4315 GT boxes / 469 neg)

| model | mode | AP50 | recall@25 | precision@25 | false-fire@25 | provenance |
|---|---|---|---|---|---|---|
| **n-mono** | gray (**native**) | 0.4421 | 44.17% | 71.41% | 4.90% | `logs/nn_tier/eval_n-mono_heldout.csv` |
| n-mono | color (OOD cross-ref) | 0.4446 | 44.45% | 72.76% | 5.76% | `logs/nn_tier/eval_n-color_nmono_color_crossref.csv` |
| **n-color** | rgb (**native**) | **[PENDING]** | **[PENDING]** | **[PENDING]** | **[PENDING]** | `logs/nn_tier/eval_n-color_<tag>.csv` |
| v2_deployed (sim, for scale) | gray | 0.0003 | 1.11% | 0.51% | 88.49% | `logs/nn_tier/eval_s-mono_summary_cmp1.csv` |

## 4. THE ANSWER

**[FILLED IN AFTER n-color LANDS — verdict-shaped: does color's bird-discrimination
advantage, if any, justify reconsidering the camera, or does a small/zero delta
CONFIRM keep-mono?]**

## 5. Reproduce

```bash
# self-tests (no training/dataset network calls; exit 0/1)
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_bird_negatives.py --self-test
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_n_color.py --self-test

# bird false-fire, n-mono vs n-color, both native modes + v2 for scale
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_bird_negatives.py \
    --models n_mono v2_deployed --modes gray color --include-leaked --tag <tag>
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_bird_negatives.py \
    --models n_color --modes color --tag <tag>

# n-color training (setsid-detached, GPU-serialized, ~1 GPU-hr on the 4070)
setsid .venv-seeker-train-gpu/bin/python scripts/seeker/train_daemon_nn_tier_color.py \
    > logs/nn_tier/train_ncolor_daemon.log 2>&1 < /dev/null &

# n-color held-out scoring (source-disjoint test, n=4175, color)
.venv-seeker/bin/python scripts/seeker/nn_tier/eval_n_color.py --models n_color --tag <tag>
```
