# AprilTag placard sizing — sim-validated auto-labeler + decode-envelope transfer

> **build_plan P0** (`docs/project_state.json`) / `docs/real_data_pipeline.md`
> "Validate the auto-labeler itself (before the hardware, in sim)" — previously
> flagged **NOT yet run**. This is that run. It (1) validates
> `scripts/seeker/autolabel_from_apriltag.py` against ground truth in the `apriltag`
> sim world, and (2) scales the measured sim tag-decode envelope to the real OV9281
> to **size the AprilTag placard BEFORE it is printed/ordered**. The physical placard
> print is blocked on this number.
>
> Engineer's-notebook tone; every quantity cites a run or a derivation. The headline
> is a **sim-scaled ESTIMATE** — the tripod field day (`docs/tripod_test_protocol.md`
> §7.1/§8.1) measures the real curve and is the authority. Date: 2026-07-20.

---

## TL;DR

- **The auto-labeler is VALIDATED (not broken).** Wherever the tag decodes, the box it
  writes lands on the ground-truth box at **mean IoU 0.965 / median 0.964 / min 0.951**,
  with tag-pose range error **mean +0.13 m, max 0.25 m**. The shipped
  `autolabel_from_apriltag.py` CLI over the captures labeled **132 / 252** frames,
  exactly matching the per-frame decode count — the tool as-shipped agrees.
- **Sim decode envelope (0.5 m tag, deployed detector config):** clean **R_decode90 =
  R_decode_any = 12 m** with a hard cliff, governed by the detector's `quad_decimate=2.0`
  (halved resolution before decode). At `quad_decimate=1.0` (full-res) it extends to **18 m**.
- **Transfer to the OV9281 + candidate placard edges → money-gate arithmetic:** under the
  deployed `quad_decimate=2.0` and conservative camera scaling, **no candidate clears the
  gate with margin**; under the realistic (px/deg) scaling, 0.30–0.35 m clear; at
  full-res decode, ≥0.25 m clear.
- **➡️ RECOMMENDED PLACARD EDGE: `0.35 m` (the carry limit).** See the bolded line in
  §4 for the full reasoning and honesty caveat.

---

## 0. What was run

Boot `apriltag` world headless → set-pose the interceptor along the tag's boresight over a
2–30 m range ladder (+ six lateral off-boresight placements at 8/12/16 m) → sample ~12
camera frames per placement → per frame: read ground truth via the M2 transform chain
(`scripts/m2_detect.py` `PoseTracker`, gt is scoring/label-source ONLY — honesty boundary),
run the **shipped** `autolabel_from_apriltag.autolabel_frame`, and compute the auto-label
box, the gt box, their IoU, and the decoded-vs-gt range. Tooling:

- `scripts/seeker/validate_autolabel_sim.py` (`--sweep` capture / `--analyze` curves+math),
  `scripts/seeker/validate_autolabel_sim.sh` (boot → sweep → CLI cross-check → analyze).
- Reuses `scripts/check_m2_envelope.py`'s set-pose pattern (subscription-free mover child,
  gz-transport13 quirk) and `gen_sim_dataset.project_to_bbox` (the same projector both the
  auto-labeler and `gen_sim_dataset.py` use — so the IoU isolates *pose* error, not size).

**Run artifacts**

| file | what |
|---|---|
| `logs/autolabel_sim_20260721T032626Z.csv` | **software-render** sweep, per-frame (the trustworthy run — see §3) |
| `logs/autolabel_sim_20260721T032324Z.csv` | GPU-render sweep (kept for the render-anomaly comparison) |
| `scripts/seeker/data/autolabel_sim_20260721T032626Z/` | saved frames + `calib.json` + shipped-CLI `autolabel_out/` |
| `logs/autolabel_sim/summary_placard.json` | machine-readable summary (all numbers below) |
| `docs/images/autolabel_{label_rate,iou,range_error}_vs_range.png`, `docs/images/placard_sizing.png` | curves |

**Machine-load note (ADR-0015).** This is a **static set-pose capture**, not a
timing-sensitive flight — RTF sag does not distort a still-scene decode, so the ADR-0015
load confound does **not** bind here (other CPU-only agents were active during the run). The
one anomaly observed was a *render-backend* effect, not a load effect (§3), and it was
resolved by re-capturing under software render, not by waiting for idle.

---

## 1. The tag + camera geometry (traced numbers)

| quantity | value | source |
|---|---|---|
| Sim tag black-square edge (`tag_size`) | **0.500 m** | `models/apriltag_target/model.sdf`: 0.625 m plane × 0.8; `scripts/make_tag_texture.py` |
| Sim camera `fx = fy` | **539.936 px** | live `CameraInfo` this run; `gz_x500_mono_cam` mono_cam SDF |
| Sim resolution / cx,cy | 1280×960 / 640,480 | live `CameraInfo` |
| Sim HFoV / px-per-deg (avg) | **99.69° / 12.84 px/deg** | derived from `fx` (2·atan(640/fx)) |
| Auto-label drone extent | 0.35 m (5" quad) | `autolabel_from_apriltag.py` default; used for BOTH boxes |

`project_to_bbox` sizes a box from a physical extent at a range as `half = fx·(extent/2)/range`.
Both the gt box and the auto-label box use the **same 0.35 m extent and same projector**, so
IoU < 1 is caused **only** by the difference between the *decoded* tag pose (real pixels →
pupil-apriltags) and the *ground-truth* pose (world pose topic). This is the fix for the
circularity `real_data_pipeline.md` flagged in the earlier one-frame check (which reused the
same `fx·s/rng` twice).

---

## 2. Auto-labeler validation — the honest IoU + label-rate curves

**Box agreement (`docs/images/autolabel_iou_vs_range.png`).** Across every decoded frame,
mean IoU(auto-label box, gt box) = **0.965**, median 0.964, **min 0.951** — flat at 0.95–0.99
over the whole 2–12 m decode band. Verdict: **wherever the tag decodes, the auto-labeler
produces ground-truth-quality boxes.** The projection math the doc previously called
"partially validated / circular" now has a proper, non-circular measurement behind it.

**Pose range error (`docs/images/autolabel_range_error_vs_range.png`).** Decoded tag range
is biased **+0.13 m mean (max +0.25 m at 10 m)** vs gt — a small, consistent slight-far bias,
negligible against the 0.35 m box extent it feeds.

**Label rate vs range (`docs/images/autolabel_label_rate_vs_range.png`).** 100 % out to 12 m,
then a hard cliff to 0 %. **The cliff is the decoder's resolution wall, not the labeler
failing** — decision margin stays healthy (~100) right up to 12 m, then decode goes binary-off
(the classic decimation cliff, §3). The auto-labeler's own logic is correct: on the
target-present sweep it **drops** tag-miss frames rather than mislabeling them background
(the ADR-0076 add #18h poison-avoidance rule), exactly as designed.

*(Method limitation: a static set-pose render is deterministic, so the per-range rate is
effectively binary and the cliff is sharp — `R_decode90 = R_decode_any = 12 m`. A real moving
approach will show a softer rolloff around that range; the transfer below uses `R_decode90`,
which sits at the same place either way.)*

**Off-boresight.** At the 8 m and 12 m lateral placements (bearings 19°, 21°, 33° off axis,
within the range envelope) decode rate is **100 % with IoU 0.94–0.95** — off-axis viewing
does not hurt the labeler. (The one "18°" band that shows 0 % is the 16 m lateral placement
at gt-range 16.7 m — a *range* miss beyond the 12 m envelope, not an off-axis failure.)

**Shipped-tool cross-check.** `autolabel_from_apriltag.py --tag-size 0.5 --drone-size 0.35`
over the 252 saved frames: **132 labeled, 120 tag-miss dropped, rate 52 %** — 132 = 72
boresight (2–12 m) + 60 off-axis (within-envelope) decodes, i.e. the shipped CLI's count
equals the per-frame decode count exactly. No discrepancy between the tool and this analysis.

---

## 3. Two findings that shaped the number (both worth carrying forward)

**(a) GPU render shortens tag decode AND mis-decodes the dead-on view — use software render
for fiducial work.** The first sweep ran under the default GPU render (`GALLIUM_DRIVER=d3d12`,
`scripts/sim_gpu_render.sh`, ADR-0075). It gave a *shorter* envelope (R_decode90 = 10 m) **and**
an anomaly: the on-axis tag at 11.9 m failed to decode while an *off-axis* tag at 12.5 m
decoded fine — physically impossible for a genuine resolution limit. This is the aggressive
GPU texture mip-minification the `model.sdf` comment already warned about, washing out the
6×6 data cells at range (worst dead-on, where all cells minify uniformly). Re-capturing under
**software render (`SIM_GPU_RENDER=0`, llvmpipe)** removed the anomaly and gave a clean,
monotone cliff at 12 m — matching the historical M2 render era. **All numbers in this doc are
the software-render run.** Carry-forward: for AprilTag-decode measurements in this sim, prefer
software render (ADR-0075's own honesty note: treat a render-backend switch like a sensor change).

**(b) The envelope is set by `quad_decimate`, and the whole project uses the library default
2.0.** `m2_detect.py`, `autolabel_from_apriltag.py`, `check_m2_envelope.py` **and the real-frame
scorer `tripod_score.py`** all build `Detector(families="tag36h11")` = `quad_decimate=2.0`,
which halves resolution before quad detection (`tripod_score.py` line 723 even comments
"10–14 px → ~0 after quad_decimate 2.0"). Re-decoding the *same saved frames* offline:

| `quad_decimate` | R_decode90 (sim, 0.5 m tag) | cliff black-square px |
|---|---|---|
| **2.0 (deployed default)** | **12 m** | ~22 px |
| 1.0 (full-res) | 18 m (~1.5×) | ~15 px |

Full-res decode buys ~1.5× range at a **Pi-CPU/fps cost** (bench it in `tripod_test_protocol.md`
§7.3). This is the cheapest range-reclaim lever if the tripod day shows the tag margin is thin.

Both confirm the 12 m cliff is **real, expected behavior of the deployed detector config** —
not a bug and not the auto-labeler.

---

## 4. Placard sizing — scaling the sim envelope to the OV9281

**Physics.** A small centered tag decodes when its black-square pixel span `P = f_px·edge/R`
exceeds a decoder threshold `P_min`. So `R_decode = f_px · edge / P_min` — **linear in focal
length (px) and in tag edge.** Scaling sim → real:

```
R_decode90_real = R_decode90_sim × (camera scale) × (edge_real / 0.5 m)
```

**Camera scale — two honest bounds** (`docs/camera_paper_check.md` §2):

| scale | value | meaning |
|---|---|---|
| **conservative (fx-ratio)** | **0.712** | both as ideal pinholes: `fx_real/fx_sim`, `fx_real = (640)/tan(118°/2) = 384.6 px`. A hard lower bound (a real barrel-distorted wide lens resolves the *center* better than a 118° pinhole implies). |
| **middle (avg px/deg)** | **0.845** | `camera_paper_check.md`'s ~15 % penalty: `(1280/118)/(1280/99.69) = 10.85/12.84`. The more realistic estimate for a centered tag. |

**Money gate (verbatim from `scripts/seeker/tripod_score.py`).** `TGO_MIN=0.5 s`,
`V_closing=9.0 m/s` (conservative — the gate scenario), `HANDOFF_STREAK=5`, `STREAM_FPS=30`.
At the R90 decode rate (1.0 → 30 Hz): `R_streak_burn = (5/30)·9 = 1.5 m`, and
`t_go = (R90 − 1.5)/9 ≥ 0.5` ⇒ **gate threshold `R_decode90_real ≥ 6.0 m`.** (The aggressive
20 m/s scenario needs ≥13.3 m — no candidate reaches it in sim; per `tripod_score.py` it is
context-only, not the gate. The NEXT.md "~20 m R_acq" is a looser rule-of-thumb anchor; the
formula above is what the scorer actually computes and what this doc gates on.)

**Result — predicted real `R_decode90` (m) and gate verdict per candidate edge:**

| edge (m) | qd=2.0 conservative | qd=2.0 middle | qd=1.0 conservative | gate (cons / mid) |
|---:|---:|---:|---:|:--|
| 0.20 | 3.42 | 4.06 | 5.13 | FAIL / FAIL |
| 0.25 | 4.27 | 5.07 | 6.41 | FAIL / FAIL |
| 0.30 | 5.13 | 6.08 | 7.69 | FAIL / **PASS** (t_go 0.51 s) |
| **0.35** | 5.98 | **7.10** | 8.97 | FAIL(t_go 0.498) / **PASS** (t_go 0.62 s) |

(`0.35 m` = the carry limit, `docs/hardware_order_list.md` §E. Full matrix incl. full-res
verdicts in `logs/autolabel_sim/summary_placard.json`; plot `docs/images/placard_sizing.png`.)

### ➡️ **RECOMMENDED PLACARD EDGE (AprilTag black-square): `0.35 m`** ⬅️

**Why 0.35 m (the carry limit), not smaller:**
1. **Decode range scales linearly with edge and the sim envelope is SHORT** — nothing clears
   even the aggressive scenario, and 0.35 m is *marginal* (t_go ≈ 0.50 s) at the single most
   pessimistic corner (deployed `qd=2.0` + conservative pinhole camera scaling). There is **no
   room to size down**.
2. It is the **only** candidate that clears the money gate under the *realistic* (px/deg)
   camera scaling at the deployed detector config (t_go 0.62 s ≥ 0.5), and clears **comfortably**
   at full-res decode (8.97 m). 0.30 m clears middle-scaling too but with half the margin.
3. **Every un-modeled real penalty pushes range SHORTER** — motion blur (Gazebo has none),
   the mono sensor (color cue lost, `camera_paper_check.md` §4), outdoor light, and real lens
   MTF all raise `P_min`. So the biggest carriable placard is the prudent hedge.

**Honest caveat (do not over-read this):** this is a **sim-scaled estimate**, not a
measurement. The sim `P_min` is a best case (clean pinhole, zero blur); the real `P_min` is
larger. The tripod field day (`docs/tripod_test_protocol.md` §7.1 curve (a), §8.1 GO/NO-GO)
measures the *real* `R_decode90` on the deployed OV9281 + lens and is the authority — this
number just makes sure the placard we print/carry is the largest that de-risks that day. If
the field `R_decode90` still comes in thin at 0.35 m, the reclaim levers (in priority) are:
**full-res decode (`quad_decimate=1.0`, ~1.5×, costs Pi fps)** → a placard bigger than the
carry limit allows (needs a target airframe change) → a camera upgrade (AR0234,
`hardware_order_list.md` §2).

---

## 5. Reproduce

```bash
# full: boot apriltag world (software render for fiducial work), sweep, CLI cross-check, curves+math
SIM_GPU_RENDER=0 bash scripts/seeker/validate_autolabel_sim.sh

# offline re-analyze an existing sweep CSV (curves + placard math + qd=1.0 overlay):
.venv/bin/python scripts/seeker/validate_autolabel_sim.py \
    --analyze logs/autolabel_sim_20260721T032626Z.csv \
    --images-dir docs/images --sim-fx 539.9363327026367 --r90-hires 18.0 \
    --summary-out logs/autolabel_sim/summary_placard.json
```

---

## ⛔ CORRECTION (2026-07-24, ADR-0082) — the published threshold was optimistic; the SIZE stands, the DECODE SETTING changes

The gate threshold quoted above (`R_decode90 ≥ 6.0 m`) came from
`validate_autolabel_sim.tripod_gate()`, which used the **mean-rate** streak burn
(`k / decode_Hz`) at a **hardcoded 30 fps**. Both inputs were wrong:

1. **Wrong burn model.** ADR-0079 replaced mean-rate with the **run-length**
   expectation `E[T] = (1−p^k)/(p^k(1−p))` in `tripod_score.py` because the handoff
   needs `k` CONSECUTIVE decodes and mean-rate always understates the burn (the
   optimistic = dangerous direction for a purchase gate). `validate_autolabel_sim`
   was never updated — so the codebase carried **two different burn models**, and the
   *optimistic* one produced the published sizing numbers. Both now use the identical
   run-length model (parity asserted: p=0.9 → 6.9351 frames in both files).
2. **Unsourced frame rate.** 30 fps had no provenance. The deployed flight loop runs
   **20 Hz**; the only measured AprilTag cadence in the repo is **~14 Hz**.

**Corrected threshold band** (R_decode90 needed for t_go ≥ 0.5 s at 9 m/s, k=5):

| | 30 fps | 20 Hz (deployed loop) | 14 Hz (measured tag rate) |
|---|---:|---:|---:|
| p = 1.0 | 6.00 m | 6.75 m | 7.71 m |
| **p = 0.9** (worst the ≥90% band allows) | **6.58 m** | **7.62 m** | **8.96 m** |

**Verdict for the ADOPTED 0.35 m placard** (predicted 7.10 m realistic / 8.97 m at
full-res `quad_decimate=1.0`):

| decode setting | 30 fps | 20 Hz | 14 Hz |
|---|:--:|:--:|:--:|
| `qd=2.0` (deployed default), 7.10 m | PASS (t_go 0.558) | **FAIL (0.442)** | **FAIL (0.294)** |
| `qd=1.0` (full-res), 8.97 m | PASS (0.765) | **PASS (0.650)** | **PASS (0.501)** |

### The two decisions this forces

- **PLACARD SIZE IS UNCHANGED — 0.35 m tag / 0.45 m sheet.** It was already the
  **carry limit** (`docs/placard_mount.md`), so there is nothing larger to choose
  without a different target airframe. The correction does not make the placard
  wrong; it makes the *margin* honest — at the deployed decode setting there is none.
- **`quad_decimate=1.0` (full-res decode) is PROMOTED from "reclaim lever" to the
  PLANNED tripod-day setting.** It is the only configuration that clears the gate at
  every credible frame rate, and it costs Pi fps — which is precisely why the day must
  also **MEASURE** the achieved rate (`pi_capture` now records `stream_fps` + a p10
  slow-tail; `tripod_score` refuses to invent one). Capture at `qd=1.0`, and re-score
  the same frames offline at `qd=2.0` for the comparison — one session, both answers.

**Honest read:** this does not say the placard fails. It says the published PASS was
resting on an optimistic burn model at an unmeasured frame rate, and that the real
decision hinges on a number nobody has measured yet (the achieved Pi capture rate).
The tripod day was always the authority — this correction just makes sure the day is
run in the configuration that can actually clear the bar, and that its verdict is
computed with the conservative model.
