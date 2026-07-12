# Quad-target retrain — RESUME GUIDE (survives /clear)

> **Why this file exists.** The reproduction commands for this arc were driven by
> scratchpad scripts that are ephemeral. This doc is the durable, self-contained
> record so a fresh session (post-`/clear`, no chat memory) can resume the
> banking-pose retrain and finish the swap **without the conversation**. Status
> language is deliberate — read it before assuming anything is done.
> Context ADR: `docs/decisions.md` **ADR-0072** (all addenda). Live status:
> `NEXT.md` CURRENT block. Last updated 2026-07-11.

## The arc in one paragraph

The builder diagnosed that the old markerless target `models/fpv_target_markerless`
is a **flat billboard** (rotor plane in the local Y-Z *vertical* plane), so
`--orient-to-velocity` only tilts a vertical wall — it never looked like a real
banking quad (ADR-0072 addendum #2). Fix: a **proper 3D quad** `models/fpv_quad_enemy/`
(real x500 mesh, horizontal rotor plane, red enemy props, nose +X) that leaves the
validated billboard model untouched. Because the target's *appearance* changed, the
markerless YOLO seeker (`drone_finetuned_v2`, fine-tuned on billboard pixels) must be
**retrained** on the new quad and the intercept re-validated. **Appearance changes are
retrain-gated — this is not a free swap.**

## Current state (2026-07-11) — what is DONE vs IN FLIGHT

| Piece | State | Artifact |
|---|---|---|
| `fpv_quad_enemy` model | ✅ **DONE + render-verified** (banks like a real drone in chase view) | `models/fpv_quad_enemy/` (meshes vendored ~25 MB, git-LFS candidate once a remote exists), committed `53b0400` |
| Verify + capture/intercept worlds | ✅ DONE, symlinked into PX4 | `worlds/quad_enemy_verify.sdf` (chase-verify), `worlds/quad_enemy.sdf` (onboard-only capture + intercept), committed `0d9d20e` |
| Level+yaw dataset | ✅ DONE (1320 imgs: 1052 train / 268 val, gitignored, regenerable) | `scripts/seeker/data/quad_dataset/` via `render_sim_dataset.py` |
| **Level seeker** `drone_finetuned_quad` | ✅ **trained + exported**, ⚠️ **re-validated on ONE flight only (no Pk batch)** | `scripts/seeker/weights/drone_finetuned_quad.{pt,onnx}` (gitignored), daemon `train_daemon_quad.py`, committed `0d9d20e` |
| **Banking-pose retrain** `drone_finetuned_quad_v2` | ✅ **trained (stopped ~ep 17, saturated) + exported**; re-validated 1 flight = **handoff FIXED (6.75 m, 4 losses)** but that flight ABORTED in the terminal (see "v2 seeker RESULT" below) | `scripts/seeker/weights/drone_finetuned_quad_v2.{pt,onnx}` (gitignored); scripts committed `9a8fd56` |
| Characterization batch | ✅ **6/6 CLEAN** (median miss ~1.2 m, handoff ~6.8 m; the earlier abort was run-to-run noise) | see "v2 characterization batch" below |
| Pk batch on the new pair | ❌ not started (after the terminal issue is understood) | — |
| Swap to deployed | ❌ not done (fallback stays live) | — |

### The level-seeker re-validation result (single flight — NOT a Pk claim)
On the `quad_enemy` world with the level seeker, one intercept flight:
**first detection 23.3 m** (beats v2's 21.6 m), **intercepted 1.18 m clean** — **BUT
handoff was LATE at 1.91 m** (v2 hands off ~9 m) with **26 tracker re-acquisitions**,
because detection is **intermittent on the BANKED aspects** the level+yaw dataset never
showed. Log: `logs/m4_intercept_pronav_20260711T195539Z.csv`. Core works; robustness
(early, stable handoff) is the open gap → the banking-pose retrain below.

### Exactly where the in-flight retrain is
The three retrain scripts exist on disk (**uncommitted** — the builder will commit):
- `scripts/seeker/render_sim_dataset_banked.py` — level capture extended to command
  **roll (bank) + pitch + yaw** via a full ZYX-Euler quaternion (copied verbatim from
  the mover so poses match the live target). GT box is a bounding-sphere → **orientation-
  invariant**, so banked frames auto-label as correctly as level ones (honesty boundary
  intact: gt labels TRAINING data offline only, never inference).
- `scripts/seeker/merge_quad_v2.py` — unions level `quad_dataset` + `quad_dataset_banked`
  into `quad_dataset_v2` (source-tagged filenames, preserves each split).
- `scripts/seeker/train_daemon_quad_v2.py` — setsid-detached yolo11n fine-tune on
  `quad_dataset_v2`, sentinel `QUAD_V2_TRAIN_EXPORT_DONE`, exports
  `scripts/seeker/weights/drone_finetuned_quad_v2.{pt,onnx}`.

**UPDATE (2026-07-11, later): capture + merge DONE, v2-training LAUNCHED.** Banked capture
finished at **1344 frames** (1072 train / 272 val); the euler→quat in the extended capture
was verified **byte-identical (1e-12)** to the mover's `_euler_zyx_to_quat`, so captured
poses match exactly what the live `--orient-to-velocity` target flies; and same-position/
different-orientation gt boxes were confirmed **byte-identical** (orientation-invariance /
honesty boundary verified empirically). Merge produced `scripts/seeker/data/quad_dataset_v2/`
= **2664 frames** (train 2124 = 1052 level + 1072 banked; val 540 = 268 level + 272 banked).
The v2 daemon is **running setsid-detached** (was PID 1588430; verify by log freshness, not
PID) → `logs/train_quad_v2_20260711T204234Z.log`. At ~2.4 s/it × 133 it/epoch the full 60
epochs is **~5–5.5 h**, but metrics saturate ~epoch 20 → **export best.pt EARLY (step 4)**
once converged rather than waiting it out. Then re-validate (step 5). NOT yet done:
`drone_finetuned_quad_v2` weights export, re-validation, Pk batch, swap.

---

## RESUME — step by step

All commands run from the repo root `/home/emerson/interceptor-sim`. Sim work is
serialized (one sim, idle load). Flight venv `.venv-seeker`; training venv
`.venv-seeker-train` (CPU-only torch — expected, don't "fix" it).

### 0. Check the in-flight training / capture first
```bash
# Is a banked capture or v2 training still alive? (grep by SCRIPT NAME only — never
# pass a live sim-process pattern inline; that self-kills the tool call.)
ps -eo pid,etime,cmd | grep -E 'render_sim_dataset_banked|train_daemon_quad_v2' | grep -v grep

# Newest v2 training log + the completion sentinel:
ls -t logs/train_quad_v2*.log 2>/dev/null | head -1 | xargs -r grep -c QUAD_V2_TRAIN_EXPORT_DONE
# Banked capture progress (frame counts) + its exit line:
ls scripts/seeker/data/quad_dataset_banked/images/train | wc -l
tail -3 "$(ls -t logs/quad_dataset_banked_capture_*.log | head -1)"
```
Detect death by **log staleness**, not pgrep (a setsid daemon may outlive its watcher).

### 1. Finish the banked dataset capture (if it died / is incomplete)
Boot the `quad_enemy` world headless (see the px4-gazebo skill / `NEXT.md` "Key facts"
boot line; env: `PX4_GZ_WORLD=quad_enemy INTERCEPTOR_WORLD_NAME=quad_enemy
INTERCEPTOR_TARGET_MODEL=fpv_quad_enemy`, world already symlinked into PX4). Then:
```bash
INTERCEPTOR_WORLD_NAME=quad_enemy INTERCEPTOR_TARGET_MODEL=fpv_quad_enemy \
  .venv-seeker/bin/python scripts/seeker/render_sim_dataset_banked.py \
  --out=scripts/seeker/data/quad_dataset_banked \
  --ranges=2,2.5,3,4,5,6,8 --laterals=-0.75,0,0.75 --heights=0.25,0.5 \
  --yaws-deg=0,90,180,-90 --banks-deg=-50,-30,30,50 --pitches-deg=0,20 \
  --extent-m=0.9 --val-frac=0.2
```
> **Gotcha:** use the `--flag=value` form for `--laterals` (and any negative-leading
> list) — the bare `--laterals -0.75,...` form makes argparse read `-0.75` as a flag and
> exits 2 (bit us twice; see the 0-byte capture logs).

### 2. Merge level + banked → `quad_dataset_v2`
```bash
.venv-seeker/bin/python scripts/seeker/merge_quad_v2.py   # refuses if quad_dataset_v2 exists; rm -rf it to re-merge
```

### 3. Launch the v2 training daemon (setsid-detached — immune to the ~51-min reaper)
```bash
setsid .venv-seeker-train/bin/python scripts/seeker/train_daemon_quad_v2.py \
  > logs/train_quad_v2_$(date -u +%Y%m%dT%H%M%SZ).log 2>&1 < /dev/null &
```
The daemon **RESUMES** from `scripts/seeker/runs/drone_finetune_quad_v2/weights/last.pt`
if present, else runs FRESH from `yolo11n.pt`. Recipe: imgsz 640, 60 epochs, batch 16,
seed 0. It exports on finish and prints `QUAD_V2_TRAIN_EXPORT_DONE`. If it dies, just
re-run the exact same line (it auto-resumes from `last.pt`).

### 4. (Optional) export best.pt EARLY if saturated
Metrics saturate ~epoch 20; the daemon only exports at epoch 60. To shortcut once
`best.pt` looks converged:
```bash
cp scripts/seeker/runs/drone_finetune_quad_v2/weights/best.pt \
   scripts/seeker/weights/drone_finetuned_quad_v2.pt
.venv-seeker-train/bin/python -c "from ultralytics import YOLO; \
  YOLO('scripts/seeker/weights/drone_finetuned_quad_v2.pt').export(format='onnx',imgsz=640,opset=12,simplify=True)"
```
(Do NOT trust val mAP alone — the random split is leaky; the intercept in step 5 is the
real gate.)

### 5. Re-validate the intercept on the new quad
Boot `quad_enemy` headless, then fly the adopted deployment argv with the new weights +
the banking target:
```bash
MARKERLESS_NN_WEIGHTS=scripts/seeker/weights/drone_finetuned_quad_v2.onnx \
INTERCEPTOR_TARGET_MODEL=fpv_quad_enemy INTERCEPTOR_WORLD_NAME=quad_enemy \
INTERCEPTOR_ORIENT_TO_VEL=1 INTERCEPTOR_ORIENT_YAW_OFFSET_DEG=0 \
  .venv-seeker/bin/python scripts/m4_intercept.py \
  --fpv --handoff --law pronav --seeker markerless \
  --target-start=6.500,30.025,0.5 --target-vel=0.000,-12.000 --cue-seed 256788 \
  --dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --fuse-midcourse \
  --track --handoff-cue-gate 8
```
Note `INTERCEPTOR_ORIENT_YAW_OFFSET_DEG=0` — `fpv_quad_enemy`'s nose is baked **+X**
(the old billboard needed `-90`). Check **first-det range, handoff range, CPA**;
success = an **EARLIER, stable handoff** than the level seeker's 1.91 m (fewer tracker
losses), CPA still clean.

### 6. Pk batch, THEN swap (in that order — do not swap before Pk)
Run a paired weave Pk batch (mc-batch skill) on the new pair to confirm no regression
vs the fallback. **Swap criteria:** only after the batch validates the new pair
(no Pk/miss regression on the weave arm) does the deployed config point at the quad.

**Swap steps (after Pk passes):**
1. Default `MARKERLESS_NN_WEIGHTS` → `scripts/seeker/weights/drone_finetuned_quad_v2.onnx`.
2. Deployed `INTERCEPTOR_TARGET_MODEL` → `fpv_quad_enemy`, world → `quad_enemy`.
3. Point the markerless world's target `<include>` at `fpv_quad_enemy`.
4. Keep `--orient-to-velocity` ON **for demo/render only**; keep it OFF for measured MC
   batches (ADR-0010 #6 / ADR-0072 discipline — the flag is default-OFF and refused for
   apriltag_target, so mc_batch stays byte-identical).

**Fallback (stays live until the swap):** `fpv_target_markerless` + `drone_finetuned_v2`
(the deployed billboard seeker). Do NOT delete it — it is the rollback and every prior
Pk/detection number was measured against it.

## Honesty note
No `gt_*` reaches inference anywhere in this arc. The banked capture's gt labels are
TRAINING-data auto-labels only (offline), orientation-invariant bounding-sphere boxes.
The mover's `--orient-to-velocity` drives the *target's own* body orientation from its
*own* velocity/accel — not a sensor the interceptor reads. Every re-validation flight
re-earns the per-tick no-cheat audit.

## v2 (banking) seeker — RESULT (2026-07-11, single flight)
Trained to saturation (~epoch 17, stopped early), exported `drone_finetuned_quad_v2.onnx`.
One re-validation flight on `quad_enemy` (orient ON, offset 0):
- **Handoff FIXED:** first-det 21.9 m, **handoff 6.75 m** (level seeker was 1.91 m; v2 near the
  old ~9 m), **tracker losses 26 → 4.** The banked data did its job — consistent dash detection.
- **BUT this flight ABORTED in the terminal:** "lost tag >5 s far from target", CPA 1.027 m but
  `clean=0`, and **reacq_rejected=43** — the seeker saw detections post-loss but the re-acquisition
  gate (`seed_gate_m=8.0`, `reseed_iou=0.2`) rejected them. NEW failure mode vs the level seeker
  (which had a late handoff but then intercepted). Single flight — could be seed-specific OR a
  terminal re-acq-gate mismatch (the CSRT/gate were tuned for the billboard, not the maneuvering
  quad). Log `logs/m4_intercept_pronav_20260711T221501Z.csv`.
- **Next: a small characterization batch** (several path-seeds, v2 seeker) to see if the terminal
  abort is systematic; if so, investigate the re-acq gate for the maneuvering quad. Swap still
  Pk-gated + NOT done.

### v2 characterization batch — 6/6 CLEAN (the abort was noise)
6 flights, different path-seeds, v2 seeker on `quad_enemy` (orient ON):
| flight (path/cue seed) | first-det | handoff | miss | result |
|---|---|---|---|---|
| 1 (778181/256788) | 23.8 | 6.75 | 1.12 | clean |
| 2 (111222/314159) | 14.0 | 2.23 | 0.88 | clean |
| 3 (333444/271828) | 24.5 | 9.47 | 1.32 | clean |
| 4 (555666/141421) | 24.5 | 6.92 | 1.21 | clean |
| 5 (777888/173205) | 22.8 | 2.02 | 1.45 | clean |
| 6 (246810/223606) | 9.95 | 9.15 | 3.09 | clean |

**6/6 clean, median miss ~1.2 m, median handoff ~6.8 m** (level seeker was stuck at 1.9 m).
The single-flight terminal abort earlier was **run-to-run noise** — flight 1 here used the SAME
seeds (778181/256788) and came back CLEAN (1.12 m), confirming the abort was the ~1 m terminal-
dropout noise (methodology rule), not systematic. The one high miss (3.09 m, flight 6) is a LATE
ACQUISITION (first-det 9.95 m), the ADR-0038 acquire-late→miss-more tail, not a banking failure.
**Conclusion: the proper-quad model + banking retrain VALIDATES — clean maneuvering intercepts,
handoff restored.** NOT yet the n≥8 paired Pk bar (ADR-0064 discipline) → that batch is the swap
gate; the swap remains Pk-gated + not done, fallback (`fpv_target_markerless`+`drone_finetuned_v2`) live.

## AUTO-CROP hi-res seeker (ADR-0074, fix #3) — the coverage lever (2026-07-12)
Raises the ~50% NN-coverage ceiling by running the NN on a NATIVE-640 crop around the target
(2x pixels-on-target) instead of the downscaled full frame. Committed `6db3ed1` (default-OFF,
byte-identical). Files: `scripts/seeker/{crop_geom,auto_crop_seeker,render_sim_dataset_crop,
verify_autocrop}.py` + `finetuned_seeker.py`/`markerless_loop.py` hooks; `train_daemon_quad_crop.py`.
RESUME the arc (durable commands; scratchpad chain is ephemeral):
```bash
# 1) capture native-crop dataset (sim up on quad_enemy, GPU render):
INTERCEPTOR_WORLD_NAME=quad_enemy INTERCEPTOR_TARGET_MODEL=fpv_quad_enemy \
  .venv-seeker/bin/python scripts/seeker/render_sim_dataset_crop.py \
  --out=scripts/seeker/data/quad_dataset_crop --ranges=2,2.5,3,4,5,6,8 \
  --laterals=-0.75,0,0.75 --heights=0.25,0.5 --yaws-deg=0,90,180,-90 \
  --banks-deg=0,-50,-30,30,50 --pitches-deg=0,20 --extent-m=0.9 --val-frac=0.2
# 2) crop-retrain (setsid, CPU):
setsid .venv-seeker-train/bin/python scripts/seeker/train_daemon_quad_crop.py \
  > logs/train_quad_crop_$(date -u +%Y%m%dT%H%M%SZ).log 2>&1 < /dev/null &   # sentinel CROP_TRAIN_EXPORT_DONE
# 3) validate (NN-only + auto-crop):  MARKERLESS_AUTOCROP=1 MARKERLESS_NN_WEIGHTS=.../drone_finetuned_quad_crop.onnx
#    mc_batch quad config (MC_WORLD=quad_enemy MC_TARGET_MODEL=fpv_quad_enemy MC_SEEKER=markerless,
#    orient ON offset 0, NO --track) --n 32 --speeds 12.0 --path weave --master-seed 42
```
Success = NN-only+auto-crop Pk@2.5 BEATS the NN-only 640 baseline (the coverage lever worked).
