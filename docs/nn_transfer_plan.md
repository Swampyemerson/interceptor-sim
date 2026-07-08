# Real-world NN transfer plan (Track D, T24)

*Design doc only — no sim runs, no training runs, no commits. Companion to
`docs/phase2_sim_to_real_plan.md` (Track D, §2) and the ground-rig companion
`docs/stereo_design.md`. Written for a builder new to edge ML deployment —
terms are defined on first use. All fps/accuracy numbers below are either a
literature citation for a comparable model on the SAME target hardware, or an
explicit analogy from the AGPL model this project already benched in sim
(flagged where that's the case). Nothing here is bench-measured — that is
exactly what the Stage-0 extension in §3 exists to produce.*

## Why this doc exists, in one paragraph

The project fine-tuned `yolo11n` (Ultralytics YOLO11-nano) on Gazebo-rendered
frames of the tag-less target body — `drone_finetuned_v2.onnx`
(`scripts/seeker/weights/`) — and closed a real detection-coverage gap doing
it (ADR-0040/0042). Phase 2 will do the same thing again for the ground
stereo rig's detector (T17). Neither of those trained *weights* will work on
real hardware: they were trained on a synthetic, matte, zero-blur, zero-noise
render of a primitive body, not a photograph of anything. And the base
architecture behind both — YOLO11n/YOLOv8n, from Ultralytics — is
**AGPL-3.0**, a license that matters once this stops being a sim-only
portfolio repo and starts being "could this be a real product." This doc is
the plan for what actually happens when Stage-0 hardware exists: which model
family to build on, how to get real training data, and what from the sim
work carries forward vs. gets rebuilt from scratch.

---

## 1. What transfers from the sim work vs. what gets rebuilt

The honest split, after reading ADR-0038 through ADR-0042 closely: **the
engineering lessons transfer, the trained weights do not.** That's the
expected, healthy outcome of doing perception work in sim first — you don't
throw away the sim effort, you throw away exactly one artifact (the weights)
and keep everything that taught you how to get there.

| Lesson (source ADR) | What it actually is | Transfers to real hardware? |
|---|---|---|
| **Hard negatives from the deployed camera's own view** (ADR-0040 addendum, ADR-0042) | v1's fine-tune, trained on clean static renders only, false-locked on the interceptor's own prop arms and ground shadow *in flight* — frames the static dataset never contained. Fix: capture real in-flight frames (own props visible, out-of-FOV, tiny/dropped) as **empty-label negatives**, not just "background." | **YES, as a principle.** Any camera bolted to a real airframe/rig will show its own mount, arms, or shadow in some frames — that is a physical fact of the camera's mounting, not a sim artifact. The *specific* negatives (Gazebo prop-arm pixels) don't transfer; the *practice* of capturing hard negatives from the real deployed camera before trusting a detector does. |
| **Self-mask / interior gate** (`FinetunedNNSeeker`, `two_stage_seeker.py`) | A cheap geometric rule applied *after* detection: reject a box whose bearing exceeds ~30° or that touches the L/R frame edge — because a body-fixed part (prop arm) always sits at a fixed edge-hugging location, while the real target moves freely through the frame interior. | **YES, as an architecture pattern**, ported directly (same code shape: geometric post-filter, no retraining needed to change it). The exact threshold (30°, edge-touch) is a function of the real camera's mount geometry and FOV and will need re-tuning per physical rig, but the *pattern* — "don't trust a detection where a known-bad structural false-positive lives" — is dataset-independent and cheap to keep. |
| **Range calibration method** (`calibrate_range.py`) | Positives-only, IoU-gated: fit an *effective* target span from validation-set boxes so `range = fx · span / box_width_px` reads true range in the median, rather than trusting an assumed nominal size. | **YES, the METHOD transfers exactly** — it only needs a labeled validation set and the camera's `fx`, both camera-agnostic. The *fitted constant* (span_eff ≈ 0.92 m for the sim body) does not transfer — a real target has a different apparent size, and the whole reason this method exists is that "the target is exactly 1.0 m" is never quite true. Re-fit per real camera + real target class. |
| **Detect-then-track / two-stage architecture** (`markerless_loop.py`, `two_stage_seeker.py`) | Proposal → self-mask/horizon reject → crop+upsample → classify, wrapped in a streak-gated track (N consecutive hits before trusting a lock). | **YES, as an architecture choice**, independent of which detector network sits inside it. The streak-length / gate constants were tuned to the sim's ~14 Hz cadence (ADR-0009) and **must be re-tuned to the real measured detection Hz** — `stage0_bench_plan.md` §3b flags this as the #1 still-unmeasured number for this exact reason. |
| **Polar fusion discipline** (ADR-0044: cue never touches the bearing channel) | A guidance-architecture decision, not a perception one: the ground cue may inform range/velocity, never the seeker's own bearing measurement. | **YES, unchanged, and orthogonal to every detector choice in this document.** Swapping the detector network does not touch this boundary — it is enforced above the detector, in the fusion/guidance layer. |
| **The trained weights themselves** (`drone_finetuned_v2.onnx` / `.pt`) | A YOLO11n backbone fine-tuned on ~530 positives / ~700 negatives, all rendered or captured *inside Gazebo* — a synthetic primitive body, flat matte shading, zero motion blur, zero lens distortion, zero real sensor noise. | **NO.** This is the one asset that does not survive the sim→real boundary, for the ordinary reason any sim-trained perception model doesn't: the visual domain gap. It's also worth noting the ADR-0042 *mechanism* finding doesn't automatically transfer either — "detection persistence wasn't the binding constraint, bearing-noise-throughput was" was measured against Gazebo's box-center bearing noise, which has no reason to match a real camera's bearing noise sources (rolling readout artifacts if not global-shutter, real optical distortion, real quantization noise). Re-measure this on real footage, don't assume it. |

**Read across:** four of five lessons are architecture or method — essentially
free to port, because they were never really about the *sim* in the first
place, they were about *not trusting a raw detector output blindly*. The
fifth (guidance-side polar discipline) is already fully decoupled from
perception. The one thing guaranteed to need full retraining is the trained
weights — which is also the *cheapest* single thing to redo, because the
pipeline that produces them (render/capture → hard negatives → fine-tune →
calibrate) is the part that transferred.

---

## 2. Model options for the real seekers

### 2.1 Terms, once

- **Edge NPU** — a chip built specifically to *run* (not train) a trained
  neural net fast and power-efficiently. The Hailo-8 (planned for the Pi 5
  onboard seeker, ADR-0016) is one; it cannot train a model, only execute one
  that's been compiled for it.
- **ONNX** — an open file format a trained model can be exported to so a
  *different* program can run it without needing the original training
  framework (PyTorch, etc.) installed. This project's seekers already run
  from ONNX via `onnxruntime` (`finetuned_seeker.py`).
- **Quantization** — shrinking a model's numbers, typically from 32-bit
  floats down to 8-bit integers, so it runs faster and fits an NPU's
  fixed-point hardware. Usually costs a little accuracy; how much depends on
  the architecture (CNNs quantize cleanly; transformer attention layers are a
  known harder case — see §2.3).
- **HEF** — Hailo's own compiled binary format. The Hailo toolchain path is
  **ONNX → (Hailo Dataflow Compiler: parse → quantize → compile) → HEF**; a
  model must reach `.hef` to actually run on Hailo-8 hardware.
- **TensorRT** — NVIDIA's compiler that turns an ONNX model into an optimized
  engine for a specific Jetson GPU. The ground rig's detector (ADR-0016) is
  planned to run through this path.
- **Copyleft vs. permissive license** — covered in full in §4; in short,
  *copyleft* (AGPL, GPL) can obligate you to open-source software that uses
  it, *permissive* (MIT, Apache-2.0) does not.

### 2.2 Licensing landscape (2026 snapshot, sourced)

| Model family | License | Architecture | Edge-NPU fit | Note |
|---|---|---|---|---|
| **YOLO11n / YOLOv8n** (Ultralytics) — *current sim weights* | **AGPL-3.0** | CNN, single-stage | Proven — this project already benched the family's Hailo/TensorRT numbers (ADR-0016) | Actively maintained, best tooling, but network-copyleft (§4). |
| **YOLOX** (Megvii, Nano/Tiny/S/M/L/X sizes) | **Apache-2.0** | CNN, anchor-free single-stage | Hailo Model Zoo ships **native NMS-config support** for YOLOX; TensorRT export is a standard, documented path | Development has slowed since ~2023 vs. the actively-developed Ultralytics line — re-check maintenance status before committing engineering time. [License](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/LICENSE) |
| **NanoDet-Plus** | **Apache-2.0** | CNN, ultra-lightweight (<1M params) | Hailo Model Zoo ships a **retraining recipe** (`training/nanodet/`) — the lowest-integration-risk option found | Already named as the project's intended Apache fallback in `train_drone_finetune.py` and `weights/LICENSES.md`, before this doc existed. |
| **RT-DETR** (Baidu, original) | **Apache-2.0** | Transformer (DETR-family) | Hailo Model Zoo lists plain DETR-ResNet support; transformer-attention quantization is a documented harder case on NPUs generally (§2.3) | **Caution:** the convenient Ultralytics *implementation* of RT-DETR ships under Ultralytics' AGPL-3.0 terms — only the original Baidu repo is Apache-2.0. Don't accidentally re-import the AGPL wrapper. |
| **D-FINE** | **Apache-2.0** | Transformer/DETR-family, ICLR 2025 | Not found in the Hailo Model Zoo; NPU deployment path unproven | Newer SOTA accuracy (up to 59.3% AP), but the least edge-NPU-proven option here. |
| **RF-DETR** (Roboflow) | **Apache-2.0** (core); Plus tier under a separate non-open **PML 1.0** | Transformer, DINOv2 backbone | Heaviest of the group even at "Nano" (30.5M params) — likely oversized for the Hailo-8/Jetson Orin Nano power budget | SOTA accuracy story, but sized for GPU-class edge, not a $110 NPU hat. |
| **YOLO-NAS** (Deci/SuperGradients) | Non-commercial for pretrained weights; commercial use needs a separate paid license | CNN, NAS-searched | N/A | **REJECTED** — the license itself blocks commercial/product use of the pretrained weights without a paid agreement; wrong fit for a "could be a real product" story. |

### 2.3 Edge deployment paths — what's sourced vs. analogized

**Onboard (Pi 5 + Hailo-8, terminal seeker).** This project already benched
the AGPL YOLO11n/YOLOv8n family's Hailo numbers (ADR-0016, ADR-0035):
**~105 fps / ~8 ms BEST** (single model, NPU kernel alone, [Hailo community
official benchmark](https://community.hailo.ai/t/official-fps-benchmark-on-hailo-8-using-raspberry-pi-5/18873)),
**~35 fps / ~29 ms EXPECTED** end-to-end (capture + pre + NMS + tracker on
the Pi's CPU, the load-bearing design number), **~15-20 fps WORST-credible**
(two streams + thermal derate). YOLOX-Nano (0.91M params) and NanoDet-Plus
(<1M params) are both *smaller* than YOLO11n (2.6M params), so the same tier
structure is a reasonable **starting anchor** — but it has not been measured
for either model on Hailo-8 specifically, and community benchmark numbers for
the exact same YOLOv8n model vary by 4× depending on PCIe lane config and
batch size (one report: 431 fps at batch 1; another: 104.9 fps single-stream
official). Compile the fine-tuned YOLOX/NanoDet model and re-bench once
Stage-0 hardware exists — do not ship the anchor numbers as measured.

**Ground (Jetson Orin Nano Super, stereo rig).** One sourced data point:
**YOLOX-S measured ~25 fps at 15 W with `jetson_clocks` enabled, JetPack 6.2**
([NVIDIA developer forum](https://forums.developer.nvidia.com/t/expected-fps-for-yolox-s-tensorrt-engine-on-jetson-orin-nano-jetpack-6-2/329875)).
The "S" size is larger than the Nano/Tiny sizes this project would actually
fine-tune, so a Nano/Tiny variant should run meaningfully faster — but that
is an *extrapolation*, not a citation, and the ground rig runs **two streams**
(stereo) which this project's own YOLOv8-family Jetson bench already showed
costs a large derate (ADR-0016: a clean 52 fps single-stream drops toward
~12-15 fps at two-stream + small-object resolution).

| Node | Tier | fps (estimate basis) |
|---|---|---|
| Onboard (Hailo-8) | BEST | ~100+ fps NPU-kernel-alone (analogized from YOLO11n's Hailo community number; **not bench-confirmed for YOLOX/NanoDet**) |
| Onboard (Hailo-8) | EXPECTED | ~30-35 fps end-to-end (same analogy — port this project's own ADR-0016 tier, re-measure) |
| Onboard (Hailo-8) | WORST | ~15-20 fps (two-stream + thermal derate, same analogy) |
| Ground (Orin Nano Super) | BEST | ~40-60 fps single-stream, Nano/Tiny size (extrapolated from the sourced 25 fps YOLOX-S figure, unconfirmed) |
| Ground (Orin Nano Super) | EXPECTED | ~15-20 fps, two-stream stereo (ADR-0016's own YOLOv8-class two-stream derate, ported by analogy) |
| Ground (Orin Nano Super) | WORST | ~8-10 fps (thermal throttle + contention, same derate as ADR-0016) |

**Transformer-family caution (RT-DETR/D-FINE/RF-DETR).** Multiple sources
agree transformer attention layers are a harder quantization case than CNNs
on fixed-point NPU hardware — accuracy degrades more under 8-bit
post-training quantization, and quantization-aware training (which recovers
it) is a heavier training-side lift than a CNN fine-tune
([lightweight-transformer edge survey](https://arxiv.org/html/2601.03290v1)).
One 2026 study did get RT-DETR onto a Raspberry Pi 5 + Hailo-8 at reasonable
accuracy retention (0.541 mAP50-95,
[Sci. Reports 2026](https://www.nature.com/articles/s41598-026-46453-6)), so
it isn't a hard no — it's a *less proven* path than the CNN options, and the
right amount of caution is "watch it," not "adopt it now" (see §5).

### 2.4 Fine-tune per camera, or one model for both?

**Recommend two separate fine-tunes, one architecture family.** The Phase-2
plan (A2, T17) already calls for this: the ground rig sees the target from a
different range band and a sky/horizon background, the onboard camera sees
it close-up against varied backgrounds during the terminal approach — the
learned appearance and box-size statistics genuinely differ per viewpoint (as
they did between the sim's own onboard-domain render and flight-frame
captures, ADR-0042). Reuse **one** architecture family and **one** recipe
shape (render/capture → hard negatives → fine-tune → calibrate) across both,
to keep exactly one training pipeline to maintain instead of two.

---

## 3. The Stage-0 data loop

`docs/stage0_bench_plan.md` already specifies a Pi 5 + global-shutter-camera
bench (~$230-310, parts ordered per NEXT.md) — but that plan is scoped
**narrowly to the AprilTag go/no-go gate** (detection rate, range/bearing
error, detection Hz, motion-blur threshold, lighting robustness, all against
a *printed fiducial*). It says nothing about capturing real drone-vs-bird
footage or feeding an NN fine-tune. This section is that missing piece — a
capture protocol that reuses the *same physical rig*, once it exists, for a
different purpose.

### 3.1 What to capture

| Category | What | Why |
|---|---|---|
| **Real target-drone footage** | A real small quadcopter (an RC/FPV proxy is fine — it does not need to be *this* project's eventual interceptor target, just a small multirotor) at varied ranges, angles, and lighting | The actual positive class for the detector. Captures pose/motion-blur variation no sim render has. |
| **Real bird footage** | Opportunistic — birds cannot be scheduled | The hardest negative class, and the one the sim could never provide at all (ADR-0035: "most drone-detection datasets have no labeled bird class"). |
| **Backgrounds** | Sky-only, horizon/tree-line, ground clutter, and — critically — **the camera's own mount/airframe edges once it's actually bolted to the real seeker platform** | Direct real-world analog of the own-prop hard-negative lesson (§1): captured from the REAL mount, not assumed. |
| **Ranges** | Near band (terminal, matches the onboard camera's eventual close-range role) and far band (matches the ground rig's ~60-160 m detection envelope, `stereo_design.md`) | The two fine-tunes (§2.4) need range-appropriate footage; a single near-range capture pass under-serves the ground rig. |
| **Motion states** | Hover, level transit, and — if a controllable proxy is available — a turn/evasive pass | Detection robustness against apparent motion blur and aspect change, the same axis `stage0_bench_plan.md` §3c measures for the fiducial. |

**Fill the gap with an already-vetted licensed dataset while opportunistic
capture accumulates.** `bird_discrimination_design.md` already identified the
**YOLOv7 Segmented Drone-vs-Bird** set (Mendeley, 20,925 images, 8,451
bird / 12,474 drone, **CC BY 4.0** — attribution only, no restriction) as
"best license fit" for exactly this classifier. Real birds can't be
scheduled on a build timeline; this corpus is a legitimate crutch for
bootstrapping the bird class, not a substitute for eventually capturing real
footage from the actual deployed camera and site — its species/behavior
distribution won't match wherever this eventually flies (flagged again in
"Honest limits" below).

### 3.2 How much

No invented target number — the project's own recent precedent argues for an
**empirical, gate-driven** volume rather than a fixed frame count. The v2
seeker capture pass was **3 flights → 36 gt-projected positives + 694 hard
negatives** and that was enough to collapse the false-positive pollution
signature from 0.751 to 0.000 (ADR-0042). Recommend the same shape: run
several capture sessions across lighting/background conditions, aim for
**order-of-hundreds to low-thousands of frames per class** as a starting
scale, and gate the *decision to stop capturing* on the same kind of
signature ADR-0042 used — does the acquisition-range regression close, does
the false-positive rate on held-out real frames drop to near zero — not on
hitting a pre-picked frame count.

### 3.3 Labeling approach

**Label with class from the start: `drone` / `bird` / `background`, not just
`target` / `background`.** The base detector fine-tune (§2) only strictly
needs "is a target" positives, the same as the sim recipe. But labeling
`bird` as its own class costs nothing extra at capture time and is exactly
the training data ADR-0035's `P(hostile)` classifier needs — recapturing
later to add the distinction would be pure waste. Bounding-box labeling,
auto-assist where possible (a first-pass detector proposes boxes, a human
confirms/corrects — faster than manual boxing from scratch), same tooling
shape as the sim's offline gt-projected labeling (training-time only, the
established honesty boundary — see §1, "detect-then-track").

### 3.4 How this feeds both fine-tunes and the bird gate

One capture protocol, three consumers:

1. **Onboard fine-tune (T17-onboard-analog)** — near-range footage from the
   Pi 5 rig, mounted in its eventual forward-facing seeker position (own-prop
   negatives included once mounted), feeds the same
   render/negatives/train/calibrate recipe `train_drone_finetune.py` already
   implements, with the base swapped to the §5 recommendation.
2. **Ground fine-tune (T17-ground, A2 in the Phase-2 plan)** — far-range
   footage from a ground-mounted vantage (sky/horizon background, matching
   the eventual stereo rig's viewpoint) feeds the identical recipe, separate
   weights (§2.4).
3. **Bird gate (ADR-0035 gate #8)** — the SAME footage, because it was
   labeled with class from the start, is the direct input to measuring
   `P_correct(R)` — real drone-vs-bird classification confidence as a
   function of range — which `bird_discrimination_design.md` §6.5 names as
   the one thing the Stage-0 bench must produce before any bird-rejection
   threshold can be set. **Until that curve exists, the interlock defaults
   to VETO-ALL (ADR-0035) — this capture loop is the critical path to lifting
   that default, not an optional add-on.**

---

## 4. Licensing — plain language

**Copyleft (AGPL-3.0, what Ultralytics ships).** The GNU Affero General
Public License is the strictest common open-source license: if you use
AGPL-licensed code or a model trained with AGPL-licensed tooling in a
product, you can be obligated to release your *entire* surrounding
software stack under the same AGPL terms — including, per Ultralytics' own
published commercial terms, an obligation to buy an **Enterprise license**
if you don't want to open-source everything. This applies whether the use is
internal or external, and the "AGPL" name specifically closes the loophole
plain GPL has (AGPL triggers even for software only ever used over a
network, never distributed as a binary — relevant for anything that's a
service or a fielded device, not just shrink-wrapped software).

**Permissive (MIT, Apache-2.0).** No share-alike obligation. You can use,
modify, and embed the code or model in a closed product and never release
your own source. Apache-2.0 additionally includes an explicit patent grant
(a small extra protection MIT lacks) — one more reason Apache-2.0 is the
generally-preferred permissive license for a model you might productize.

**What this means for THIS project, plain language:**

- **The sim's current choice (AGPL YOLO11n) was the right call for the sim
  phase, and it's already disclosed correctly** (`weights/LICENSES.md`,
  written before this doc existed). For a portfolio repo that's never sold
  or fielded as a product, AGPL is not a legal problem — the honesty is in
  disclosing it, which this project already does.
- **For the "could this be a real product" framing this project explicitly
  carries** (Anduril/Lattice-style counter-UAS pitch, GOALS.md), shipping an
  AGPL-trained detector inside a closed commercial device is a real
  exposure: either the whole stack goes open-source, or an Ultralytics
  Enterprise license gets purchased. Neither is free.
- **The real-hardware transfer plan (this doc) should therefore default to
  an Apache-2.0 model family** — not because the sim choice was wrong, but
  because the sim's job was "get a working detector fast to prove the
  pipeline," and the real-hardware job is "build something that could
  actually be fielded or sold." Different jobs, different license bar.

---

## 5. Decision recommendation

**Recommendation: adopt YOLOX (Apache-2.0) as the model family for BOTH the
real onboard seeker (Pi 5 + Hailo-8, YOLOX-Nano) and the real ground rig
detector (Jetson Orin Nano Super, YOLOX-S or -Tiny), replacing the sim's
AGPL YOLO11n at the sim→real boundary.**

Why:
- **License-clean across the whole stack** — one Apache-2.0 family, onboard
  and ground, no AGPL exposure anywhere in the fielded system.
- **Both edge paths are proven for this exact architecture**, not just the
  license: the Hailo Dataflow Compiler ships native NMS-config support for
  YOLOX (confirmed on the Hailo community forum), and YOLOX has a standard,
  documented TensorRT export path with a real sourced fps number on the
  Jetson Orin Nano Super (§2.3).
- **CNN, not transformer** — the safer quantization case on a fixed-point
  NPU (§2.3), where this project's most power-constrained node (Hailo-8,
  onboard) has the least room to absorb a quantization-accuracy hit.
- **Mirrors the compute split this project already chose** (ADR-0016): a
  smaller size onboard where power/latency is tight, a larger size on the
  ground where compute is comparatively free — the same shape, just an
  Apache-licensed family instead of AGPL.

**Fallback: NanoDet-Plus (Apache-2.0) for the onboard seeker specifically**,
if YOLOX doesn't compile cleanly on the Hailo Dataflow Compiler once actually
tried (the community has reported friction compiling *custom-trained* YOLOX
models — Focus-layer / NMS-config issues, §2.3 sources) — Hailo's own model
zoo ships a NanoDet **retraining recipe**, the single lowest-integration-risk
path found in this research.

**Training path:** reuse the recipe *shape* already built
(`train_drone_finetune.py`'s render → hard-negative-capture → fine-tune →
calibrate pipeline, §1) but **re-plumb the trainer** — YOLOX and NanoDet-Plus
each ship their own official trainer, not the `ultralytics.YOLO()` call this
project currently uses. That's a real, disclosed engineering task for
whoever picks up T17/T24 execution, not something this design doc can wave
away.

**What would change this decision:**
- If the Stage-0/T17 bench (once built) shows YOLOX genuinely missing the
  fps floor on Hailo-8, or a materially worse accuracy floor after
  fine-tuning than the AGPL YOLO11n baseline already proved out in sim —
  fall back to NanoDet-Plus, or make an explicit, ADR-logged builder call to
  accept AGPL YOLO11n if the portfolio story is judged to outweigh the
  product-license cost for now.
- If a transformer-family Apache model (RT-DETR/D-FINE/RF-DETR) matures a
  *proven* low-latency Hailo or TensorRT deployment path by the time
  hardware exists — this is a fast-moving space and everything in §2.3 is a
  mid-2026 snapshot — their higher small-object accuracy could win a
  re-bench, especially on the **ground/Jetson side**, where the onboard
  Hailo-8's tight power ceiling doesn't apply and there's more headroom to
  absorb a heavier model.

---

## Honest limits

- **Nothing in this document is bench-measured.** Every fps/mAP number is
  either a literature citation for a *different* nano-class model on the
  *same* target hardware, or an analogy from the AGPL family this project
  already benched in sim (ADR-0016/ADR-0035's Hailo/Jetson tiers). The real
  numbers require compiling the actual fine-tuned model on the actual
  hardware — that's the whole point of extending the Stage-0 bench (§3).
- **This document's own recommendation swaps one domain gap for another.**
  The sim→real gap it addresses gets replaced, at Stage-0, by a
  Stage-0-capture-conditions → eventual-fielded-conditions gap (season,
  weather, whatever proxy target vs. the real threat class). Expect at least
  one more re-training cycle after the "real" Stage-0 fine-tune — this is
  not a one-shot fix, and no claim in this doc should be read as "then it's
  done."
- **Real birds cannot be scheduled.** Opportunistic capture will be sparse
  and biased toward whatever happens to fly by the bench site. The licensed
  Drone-vs-Bird corpus (§3.1) is a real crutch, not a nicety — and its
  species/behavior distribution is guaranteed not to match the eventual
  deployment site exactly.
- **License risk is a snapshot (July 2026).** YOLOX's upstream repository
  has seen slower maintenance activity than the actively-developed
  Ultralytics line — re-verify before committing real engineering time to
  its (separate) trainer.
- **Transformer-family Apache models are licensed cleanly but their edge-NPU
  maturity is the least proven claim in this document** — a fixed low-power
  ASIC (Hailo-8) is a much harder target than a GPU, and this doc treats
  that family as a watch item, not a current recommendation, on purpose.
- **The Hailo/Jetson fps tiers reused here (§2.3) were built for a
  different-but-comparable-size model (YOLO11n/YOLOv8n family) in a prior
  ADR, not measured for YOLOX or NanoDet-Plus.** Flagged inline, repeating
  here because it's the single easiest thing for a future reader to
  misquote as a measured number.

## Sources

- Ultralytics AGPL-3.0 commercial terms / Enterprise license requirement — [YOLO Model Licenses: A Developer's Guide (Medium)](https://medium.com/@bingbai.jp/yolo-model-licenses-a-developers-guide-da722767b6f8)
- YOLOX license (Apache-2.0) — [Megvii-BaseDetection/YOLOX LICENSE](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/LICENSE)
- RT-DETR original (Baidu, Apache-2.0) vs. Ultralytics wrapper (AGPL-3.0) — [Ultralytics RT-DETR docs](https://docs.ultralytics.com/models/rtdetr); [lyuwenyu/RT-DETR (official)](https://github.com/lyuwenyu/RT-DETR)
- D-FINE (Apache-2.0, ICLR 2025) — [ustc-community/D-FINE (Hugging Face)](https://huggingface.co/collections/ustc-community/d-fine); [merve summary](https://huggingface.co/posts/merve/183549115190705)
- RF-DETR (Apache-2.0 core / PML 1.0 Plus tier) — [roboflow/rf-detr (GitHub)](https://github.com/roboflow/rf-detr); [RF-DETR blog](https://blog.roboflow.com/rf-detr/)
- YOLO-NAS license restrictions (non-commercial pretrained weights) — [Deci-AI/super-gradients LICENSE.YOLONAS.md](https://github.com/Deci-AI/super-gradients/blob/master/LICENSE.YOLONAS.md); [NVIDIA dev forum thread](https://forums.developer.nvidia.com/t/licensing-and-usage-terms-for-yolo-nas-model/326342)
- Hailo Dataflow Compiler ONNX→HEF path + native YOLOX NMS-config support — [Hailo community: YOLOX-s to ONNX to HEF](https://community.hailo.ai/t/yolox-s-to-onnx-to-compiled-hef-process-flow/18131); [Hailo Model Zoo NanoDet training recipe](https://github.com/hailo-ai/hailo_model_zoo/blob/master/training/nanodet/README.rst); [Hailo Model Zoo object detection docs](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst)
- Hailo-8 official FPS benchmark (Pi 5) — [Hailo community official benchmark](https://community.hailo.ai/t/official-fps-benchmark-on-hailo-8-using-raspberry-pi-5/18873)
- Jetson Orin Nano Super + YOLOX-S TensorRT fps (sourced, 25 fps @ 15W) — [NVIDIA developer forum](https://forums.developer.nvidia.com/t/expected-fps-for-yolox-s-tensorrt-engine-on-jetson-orin-nano-jetpack-6-2/329875)
- Transformer/DETR quantization difficulty on edge NPUs — [Lightweight Transformer Architectures for Edge Devices (arXiv 2601.03290)](https://arxiv.org/html/2601.03290v1); [RT-DETR on Hailo-8/Raspberry Pi 5, Sci. Reports 2026](https://www.nature.com/articles/s41598-026-46453-6)
- YOLOv7 Segmented Drone-vs-Bird dataset (CC BY 4.0) — already vetted in-repo, `docs/bird_discrimination_design.md`; [Mendeley DOI 10.17632/6ghdz52pd7.5](https://data.mendeley.com/datasets/6ghdz52pd7/5)
- This project's own Hailo/Jetson tier analysis (ADR-0016, `docs/compute_setup.md`) and the sim fine-tune's own findings (ADR-0038..0042, `docs/seeker_v2_plan.md`) — in-repo, cited throughout §1-2.

## Open questions

1. Does the Hailo Dataflow Compiler compile a **custom fine-tuned**
   YOLOX-Nano cleanly end-to-end (Focus-layer / NMS-config issues have been
   reported for custom-trained YOLOX in the Hailo community) — first thing
   to actually try once Stage-0 hardware exists, before trusting §5's
   recommendation further.
2. What's the real achievable capture cadence for drone-vs-bird footage
   (weather/season/wildlife-availability dependent)? §3.2's "order of
   hundreds to low-thousands of frames" is a starting scale, not a promised
   timeline.
3. Should the onboard and ground detectors ultimately be the *same* weights
   deployed twice, or genuinely separate fine-tunes (this doc's §2.4
   recommendation, matching the Phase-2 plan's A2)? Worth an explicit
   revisit if the two-pipeline maintenance cost becomes a real driver.
4. Does the `P(hostile)` bird classifier (ADR-0035) want its own dedicated
   architecture (a small classifier head on cropped detections) rather than
   reusing the base detector's own class output? ADR-0035 already specifies
   `P(hostile)` as a distinct posterior, never `decision_margin` — this doc
   assumes a shared backbone feeds both, but a dedicated head is the more
   likely real design and should be resolved at the T17/bird-gate build
   session, not assumed here.
5. Real lens distortion and real motion blur interact with a fine-tuned
   detector's learned box geometry the same way they interact with the
   AprilTag pose solve — `stage0_bench_plan.md` §3c already measures a
   yaw-rate/motion-blur dropout threshold for the fiducial; an equivalent
   test for the NN detector doesn't exist yet and should be added to the
   Stage-0 capture protocol extension (§3) once it's built.
6. The MIT-licensed `drone_yolo11x.pt` (real-photo-trained, did not transfer
   to the *sim* target per `weights/LICENSES.md`) might transfer *better* to
   the *real* target than a COCO-pretrained backbone, as a Stage-1 prior
   (`train_drone_finetune.py`'s "Stage 1: general prior" idea). Worth a bench
   comparison once real footage exists — though its license story is
   muddied (MIT weights, trained with AGPL Ultralytics tooling, per
   `weights/LICENSES.md`) and would need resolving for a clean product
   story, e.g. by re-deriving equivalent weights through an Apache-licensed
   trainer instead.
