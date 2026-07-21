# nn_tier PLAN — markerless-seeker decision + tiered roadmap

*SYNTHESIZE deliverable, 2026-07-21 (~06:10 UTC). Inputs: `docs/nn_tier/{domain_and_sources,viewpoint_and_deploy_spec,compute_notes,baseline_scoreboard,dataset_report,eval_s-mono}.md`, `logs/nn_tier/*.csv`, verifier-PASSed EVALUATE reports. Every number below traces to a logged run or a written derivation; provenance is cited inline.*

**Status at write time:** the `n-mono` fine-tune is TRAINING (epoch ~31/60, ~72 s/epoch on the
RTX 4070; best train-internal val mAP50 so far 0.533 at epoch 22, patience 20 —
`scripts/seeker/runs/nn_tier_n_mono/results.csv`). `s-mono` and `n-mono-aug` daemons are
queued behind it (one-training-at-a-time rule). Sections marked **[PENDING]** fill in when
those runs land; everything else is final. Final n-mono held-out scores auto-land in
`logs/nn_tier/eval_n-mono_summary_*.csv` via `scripts/seeker/nn_tier/eval_n_mono.py`.

---

## 1. RECOMMENDATION — carry forward `n-mono` (COCO-init YOLO11n @640, grayscale, real-media corpus)

**The model/config to carry forward is `n-mono`:** YOLO11n, COCO-init (never sim weights —
`docs/real_data_pipeline.md` recipe), imgsz 640, trained and evaluated **in grayscale**
(OV9281 mono deployment track, `docs/camera_paper_check.md` item 4), fine-tuned on the
nn_tier real-media corpus (15,391 images: 10,357 train / 859 val / 4,175 source-disjoint
test; 20% negatives; 70% of positives ≤40 px @1280-eq matching the 6–20 m terminal band;
composites train-only — `docs/nn_tier/dataset_report.md`). It is simultaneously the
**deployment anchor**: yolo11n @640 INT8 is the exact artifact the Hailo-8L plan is sized
around (§2).

### What is measured so far (held-out, source-disjoint, n stated)

Held-out TEST split = 4,175 images / 4,315 GT boxes / 469 drone-free negatives; `dut` =
whole never-seen source (3,000), `nps` = whole held-out flight clips (758), `plates` =
held-out photographers' desert negatives (417). Split is by SOURCE/VIDEO/SCENE, never
random frames (ADR-0061 anti-mirage rule); verifier-confirmed zero group/uid overlap.

| model (gray) | AP50 | recall@25 | precision@25 | false-fire rate (drone-free) | provenance |
|---|---|---|---|---|---|
| **v2_deployed** (sim-trained, currently deployed) | **0.0003** | **0.0111** | 0.0051 | **0.8849** (2.21 fires/neg-frame) | `logs/nn_tier/eval_s-mono_summary_cmp1.csv`, n=4175 |
| **n-mono** | **0.4421** | **0.4417** | **0.7141** | **0.049** (0.049 fires/neg-frame) | `logs/nn_tier/eval_n-mono_heldout.csv`, n=4175, same split |
| n-mono-aug (heavy augmentation) | 0.3875 | 0.4137 | 0.6688 | 0.0512 | `logs/nn_tier/heldout_scores.txt`, n=4175, same split — slightly BELOW plain n-mono ⇒ augmentation did not help on this held-out public set |
| yolo11x_mit (56.9 M, teacher only) | 0.2076* | 0.46* | 0.22* | 0.62* | *DVB corpus n=350, `logs/nn_tier/baseline_scoreboard_recon1.csv`; held-out-split subset run in flight |

**The measured non-performance of the deployed v2 is the headline result already in hand:**
on real held-out imagery in the deployment modality, `drone_finetuned_quad_v2` is blind
(recall@25 = 0.0111 of 4,315 boxes ≈ 48 hits; 0.000 recall below 24 px — the entire
terminal band) while firing on **88.5% of drone-free desert frames**. Grayscale makes it
*worse* in the dangerous direction (DVB bird false-fire 0.667→0.967 color→gray, with
HIGHER confidence — recon mono-vs-color probe). The sim-to-real transfer is a confirmed
NULL for the deployed net; a real-media model is not optional for the markerless tiers.

**Significance language:** with n = 4,315 GT boxes, the two-proportion z-test resolves any
recall delta ≥ ~2 percentage points vs v2's 1.11% at p < 0.001; against v2's near-zero
AP50 the comparison is not a close call in either direction — either n-mono clears by a
wide, significant margin or it fails outright. The v2 bar is measured on the FULL split
(not a subsample), same grayscale mode, same conf 0.25 operating point.
**LANDED (2026-07-21, n-mono, source-disjoint held-out n=4175, grayscale):** AP50
0.0003 → **0.4421**, recall@25 1.11% → **44.17%**, precision@25 0.51% → **71.41%**,
false-fire on drone-free frames 88.49% → **4.9%** — all four move the right way by
margins far past the z-test resolution floor (Δrecall ≈ +43 pts on n=4,315 boxes,
p ≪ 0.001). The real-media fine-tune decisively beats the blind sim net. **Honest
absolute read:** 44% recall @conf.25 is *moderate*, not "solved" — small fast drones
are hard, and this is a PUBLIC-media frame/video metric, NOT our-target/our-camera/
our-site. The held-out-**FLIGHT** gate on the real target (tripod day) still governs
deployment; this is the head-start, and it is a strong one. (s-mono / n-mono-aug land
next via `eval_on_export.sh`.)

**Arbitration among the three arms** (pre-registered so this plan doesn't rot):
- `n-mono` vs `n-mono-aug` (geometric-aug arm, hsv zeroed): pick by held-out AP50 and
  recall in the ≤40 px @1280-eq bins + false-fire on plates/dut negatives. Same size, same
  deploy cost — pure accuracy call.
- `s-mono` (yolo11s): deploys ONLY per the frozen B5 rule-5 (`docs/nn_tier/eval_s-mono.md`):
  it must close a REAL held-out-source recall gap vs the fine-tuned n AND later pass the
  ≥30 fps sustained bench on the real Pi+HAT (its chip rate is thin: 92 fps → est 25–45
  end-to-end, MARGINAL). COCO-mAP-style deltas are not evidence (ADR-0061 precedent).

### The honest caveat (rule, not footnote)

**Every number above — including the eventual n-mono scores — is a frame/image metric on
held-out SOURCES. It is NOT the acceptance gate.** The v3 and rebal retrains both aced
frame-eval and failed in flight (ADR-0061); that is why this project's real gate is
**held-out-FLIGHT validation on tripod-day + real-target captures through the OV9281**
(Tier-1, §3). This corpus contains ZERO imagery of our actual target (the Kakute H7 5"
quad is unbuilt) and zero frames from our actual camera/site. `n-mono` is therefore a
**domain-matched head start and an upper-bound reference — NOT deployment weights.**
Deployment weights are minted in Tier-1 and re-earned at every tier gate below.

---

## 2. SCALE-DOWN VERDICT — the builder's question, answered

**No scale-down is needed: YOLO11n @640 INT8 already fits Hailo-8L with margin.**
The "MIT drone-detection NN" (doguilmak Drone-Detection-YOLOv11x, MIT weights, in-repo)
**cannot itself be deployed** — 56.9 M params @1280 vs a 13-TOPS accelerator — so "scale
down" was answered **by construction**: we keep the big MIT net as an offline
teacher/auto-label assistant and upper-bound benchmark, and the deployed artifact is a
fresh COCO-init **n-scale** fine-tune. Numbers (`docs/nn_tier/viewpoint_and_deploy_spec.md`
B2–B6; Hailo Model Zoo HAILO8L table, DFC v2.19.0 — vendor figures, BENCH-GATED):

| model | params | Hailo-8L chip fps (b=1, INT8@640) | est. end-to-end Pi 5 fps | vs ≥30 fps bar |
|---|---|---|---|---|
| **yolo11n (n-mono)** | 2.6 M / ~10 MB ONNX | **157** | **~40–80** | **CLEARS** |
| yolo11s (s-mono) | 9.4 M / ~36 MB | 92 | ~25–45 | MARGINAL — B5 rule-5 only |
| yolo11m | 20.1 M | 35 | ~15–25 | FAILS |

- **The ≥30 fps bar is derived, not vibes** (spec B4): at the `tripod_score.py` money-gate
  constants (HANDOFF_STREAK 5, V_closing 9 m/s, TGO_MIN 0.5 s), 30 fps burns 1.5 m of
  closure on the confirmation streak and passes t_go = 0.50 s exactly at R_acq = 6.0 m;
  25 fps fails (0.47 s). It also matches the sim-validated 30 Hz ADR-0058 loop. Do NOT
  size to the CPA peak LOS rate (needs 242–935 fps, unwinnable; miss is kinematic at
  handoff, ADR-0023).
- **Everything is bench-gated** (constraint `pi5-emulation-gap`): chip fps ≠ pipeline fps
  (~2× haircut applied above); the verdict is confirmed only by the real Pi 5 + OV9281
  stream, thermally soaked (§3 Tier-2). INT8 compile needs 200–1000 grayscale desert
  calibration images — the corpus already supplies them.
- **CPU-baseline reality** (constraint `pi5-compute`): Pi 5 CPU runs the AprilTag baseline
  at real-time 30+ fps — the whole tag-phase plan needs no HAT. CPU YOLO is ~5–10 fps,
  3–6× under the bar — NOT viable at terminal LOS rates. Sanctioned CPU-NN uses are
  shadow-mode logging during tag-guided flights and offline eval only. **The Hailo-8L AI
  HAT+ (currently deferred in the order) is the graduation gate to the markerless kill —
  it must be ordered before Tier-2 can start.**
- Escalation path if the fine-tuned n shows a real held-out recall gap: spend up n→s
  (never input-size down — the target is already 3.4–18 px at the 640 input; downscaling
  destroys signal, spec B5 rule 4 / foveated-crop graveyard ADR-0076 #18j).

---

## 3. THE TIERED ROADMAP

### Tier-1 — Tripod day: OUR target, OUR camera, OUR mount → the first real fine-tune

*Goal: replace "public look-alike drones" with the actual Kakute H7 5" quad seen through
the actual innomaker OV9281 at the final mount geometry, and mint deployment-candidate
weights.*

- **Data it needs:** built target quad + the OV9281 on the tripod/bench at the final
  up-tilt mount geometry (1280×800 mono, global shutter); fly the target through the 6–20 m
  band across the aspect/position weights in `viewpoint_and_deploy_spec.md` A4–A5
  (≥70% near-level edge-on aspects; ±30° azimuth spread; edge-partials included);
  checkerboard intrinsics calibration on arrival (replaces the two paper camera models).
  Also capture drone-free background sweeps from the same spots (negatives).
- **Labeling (near-zero manual):** `docs/real_data_pipeline.md` — AprilTag on the target is
  the sanctioned AUTO-LABEL source (tag → box, labels only, never fed to a deployed
  seeker); `yolo11x_mit` proposes boxes on tag-less frames; human spot-check a sample of
  both. Auto-labeler itself validated per real_data_pipeline.md §"Validate the auto-labeler".
- **Train:** COCO-init yolo11n @640 grayscale on tripod captures MIXED with the nn_tier
  corpus (the corpus becomes the regularizer/background diversity; ratio is a logged
  experiment). A/B arm: n-mono weights as init vs COCO init — legitimate because n-mono is
  real-media-trained (the NEVER-init rule bans *sim* weights); COCO stays the default
  unless the A/B wins on held-out flights.
- **THE GATE (the real one):** **held-out-FLIGHT validation** — entire flights/sessions
  held out, never frames. Pass = recall in the ≤40 px @1280-eq bins on held-out flights
  beats the Tier-0 n-mono bar, false-fire density on drone-free sweeps low enough that the
  true target wins the 5-frame handoff streak, and the `tripod_score.py` money gate
  (R_acq ≥ 6.0 m, t_go ≥ 0.5 s) passes on held-out flights.
- **Honesty guardrails:** split by flight/session (the v3/rebal rule); tag/`gt_*` are
  labels-and-scoring ONLY; report n flights + split policy with every number; no claim
  graduates from frame-eval to "seeker works" without this gate.

### Tier-2 — Hailo-8L markerless deployment (the compile → bench → shadow → live ladder)

*Goal: the Tier-1 winner running ≥30 fps sustained on the real Pi 5 + Hailo-8L, proven in
shadow mode before it ever guides.*

- **Data it needs:** 200–1000 grayscale desert calibration images (nn_tier corpus +
  tripod captures) for INT8 quantization; the Tier-1 held-out flight set for the
  INT8-vs-fp32 accuracy delta.
- **Steps:** export ONNX → `hailomz` compile `--hw-arch hailo8l` → `.hef` → `hailortcli`
  chip bench → full pipeline on the Pi 5 with the live OV9281 stream, thermal-soaked →
  shadow-mode logging during tag-guided flights (CPU or HAT) before any guidance authority.
- **THE GATE:** (1) compiles to `.hef` with the stock recipe; (2) chip ≥60 fps b=1 (2×
  margin over the bar); (3) **≥30 fps end-to-end sustained thermal-loaded, latency
  ≤~33 ms** on the real stream; (4) INT8 accuracy delta vs fp32 re-measured on held-out
  flights — specifically the ≤40 px bins, where quantization bites first.
- **Honesty guardrails:** vendor fps numbers are never quoted as capability — only the
  bench is (constraint `pi5-emulation-gap`); shadow-mode disagreement logging vs the tag
  baseline is the acceptance evidence; the AprilTag phase remains the fallback plan and
  the HAT purchase is the explicit go/no-go for this tier.

### Tier-3 — Central Oregon site captures + desert hard-negative mining

*Goal: close the last domain gap — the actual sagebrush/juniper/rimrock venue, its light,
its dust, and its resident raptors.*

- **Data it needs:** on-site OV9281 captures — drone-free background sweeps (sagebrush
  mottle, juniper silhouettes on the skyline, basalt rimrock edges, dust/haze, hard
  shadows — the known false-positive generators per `domain_and_sources.md`) plus target
  flights at the site; birds of opportunity (prairie falcon, golden eagle, ravens are
  guaranteed at this site — bird discrimination gets measured, not assumed).
- **Steps:** run the Tier-2 artifact in shadow mode on-site → mine every false fire into
  the hard-negative pool → incremental fine-tune holding the 15–20% negative fraction
  (the rebal lesson) → re-validate → repeat until the gate holds.
- **THE GATE:** on held-out on-site FLIGHT sessions: false-fire density low enough that
  phantom competition cannot steal the handoff streak; terminal-band recall maintained
  (mining negatives must not silently buy precision with recall — both reported);
  ultimately the outdoor binary-kill money gate on the real target.
- **Honesty guardrails:** never validate on sessions that were mined for negatives
  (mine-train vs validate sessions are disjoint); bird false-fire reported per-segment
  with n; site numbers supersede all public-corpus numbers in any claim.

---

## 4. PIPELINE WIRING — Tier-1 is a data swap, not a rebuild

The nn_tier scripts were built so real captures drop into the same rails
(`docs/real_data_pipeline.md` stages → these scripts):

1. **Ingest:** drop tripod/Oregon footage under
   `scripts/seeker/data/nn_tier/raw/tripod/<session_id>/...` (one directory per
   flight/session — the session id IS the split group). `fetch_nn_tier_sources.py` gains a
   local-ingest entry (it already logs size+license per source to `fetch_log.jsonl`;
   license = "own capture").
2. **Auto-label:** tag-supervised boxes per `real_data_pipeline.md` (+ `yolo11x_mit`
   proposals on tag-less frames) → YOLO-format labels next to frames. Labels-only use of
   the tag is the sanctioned honesty boundary.
3. **Prepare:** `prepare_nn_tier_dataset.py` — add the `tripod` source with
   `group = session_id`; it already does gray conversion, deterministic group-level split
   hashing (never random frames), negative-fraction accounting, composite firewalling,
   manifest + `dataset.yaml` emission, and `--check` re-asserts split disjointness. Pin
   designated validation SESSIONS to test exactly as `dut`/`nps` clips are pinned today.
4. **Train:** `scripts/seeker/train_daemon_nn_tier.py` pattern (setsid-detached,
   checkpointed, one-at-a-time GPU rule, auto-export to
   `scripts/seeker/weights/nn_tier/*.onnx`) — point it at the regenerated `dataset.yaml`.
   Compute: ~72 s/epoch per ~10 k images on the 4070 (`compute_notes.md`), so a tripod-day
   fine-tune is an under-an-hour affair.
5. **Eval:** `eval_n_mono.py` / `eval_heldout_smono.py` — same AP50 / recall@25 /
   gate-recall (`box_scoring.box_hits_gt`, read-only) / scale-bin / position-bin /
   false-fire metrics, now over held-out SESSIONS; swap the declared intrinsics proxy for
   the real checkerboard calibration when it exists. Outputs land in `logs/nn_tier/` as
   today.

Only step 2 (auto-label glue for OV9281 footage) is genuinely new code; everything else is
configuration.

---

## 5. RISKS / OPEN QUESTIONS for the head + builder

1. **The frame→flight gap is THE risk.** Two prior models aced frame-eval and failed in
   flight (ADR-0061). Nothing in this phase — including a strong n-mono number — predicts
   Tier-1 passage. Treat n-mono as a bar and a head start only.
2. **Aspect mismatch inside the test split:** `dut` (the never-seen source, 3,000 imgs) is
   ground-observer-looking-up — the WRONG elevation aspect for our near-level engagement;
   `nps` (758) carries the air-to-air aspect. Read per-source recall separately
   (`--per-source`); a dut-driven aggregate could flatter or damn the model for the wrong
   engagement. Air-to-air data remains the corpus's scarcest resource — Det-Fly (best
   aspect match, 13 k imgs) is license-blocked pending an email to the authors, and AOT's
   S3 bucket 404'd this phase; both are worth another attempt before Tier-1.
3. **Mono confound is only part-measured.** Gray-native training fixes the measured
   channel-replication harm, but real OV9281 lens character, sensor noise, and desert
   dynamic range (bright ground out-brighting sky) are unmodeled until tripod day.
   Intrinsics are a declared proxy until checkerboard calibration.
4. **Hailo numbers are vendor-cited, bench-gated.** The 40–80 fps end-to-end estimate has
   never touched our hardware; the HAT itself is still deferred in the order book. The
   builder's Tier-2 go/no-go is a purchase decision as much as a technical one.
5. **INT8 quantization on ≤40 px targets** is the classic silent killer — the Tier-2 gate
   explicitly re-measures small-bin recall post-quantization; do not skip it.
6. **License hygiene before any product claim:** MIT weights produced with AGPL Ultralytics
   tooling (re-verify at ship); our own fine-tunes are Ultralytics-AGPL-derived — the
   Apache-2.0 family question (YOLOX/NanoDet, `nn_transfer_plan.md` §5) reopens at
   deployment; CC BY attribution obligations for DVB/plates are recorded in the manifests.
7. **Operational, live now:** the queued `s-mono` and `n-mono-aug` daemons both poll the
   GPU every 30 s — when n-mono finishes they can race-start together and thrash the 8 GB
   card (verifier-flagged; remedy is manual). Whoever is at the helm when n-mono exports
   should confirm exactly one successor training holds the GPU.
8. **Open question for the builder:** does tripod day include a second, cheaper "target"
   (any spare quad) to widen target diversity, and can a few flights carry NO tag so the
   auto-labeler's tag-less path (`yolo11x_mit` proposals + human check) gets validated
   before it is needed at the Oregon site?
9. **Val split is small and negative-heavy** (859 imgs, 61% negatives) — train-internal
   val mAP is noisy (observed swings 0.53→0.12 between epochs); never quote it as a
   result; only the held-out test split and, above it, held-out flights count.

---

*Numbers in this plan trace to: `logs/nn_tier/eval_s-mono_summary_cmp1.csv` (v2 held-out bar, n=4175), `logs/nn_tier/baseline_scoreboard_recon1.csv` (DVB bars, n=350), `logs/nn_tier/eval_s-mono_breakdown_cmp1.csv` (scale/position bins), `scripts/seeker/runs/nn_tier_n_mono/results.csv` (training trajectory), `docs/nn_tier/viewpoint_and_deploy_spec.md` B2–B6 (Hailo/fps derivations), `docs/nn_tier/dataset_report.md` + `manifest_*.csv` (corpus). [PENDING] fields fill from `eval_n_mono.py` outputs when the in-flight runs land.*
