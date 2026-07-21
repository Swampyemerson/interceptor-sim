# nn_tier EVALUATE — s-mono (yolo11s @640, grayscale deployment track)

**Status: IN PROGRESS — s-mono training has NOT completed (honest report, 2026-07-21 06:00 UTC).**
The s-mono daemon (`scripts/seeker/train_daemon_nn_tier_smono.py`) is queued behind the
sibling n-mono run on the one-training-at-a-time GPU rule (epoch ~21+/60 at report time;
an n-mono-aug daemon is also queued). No `s-mono.{pt,onnx}` exists yet in
`scripts/seeker/weights/nn_tier/`. **Nothing below is an s-mono score.** This file carries
the frozen eval method + the comparator bars; the s-mono rows append when the daemon prints
`NN_TIER_SMONO_TRAIN_EXPORT_DONE` (log: `logs/nn_tier/train_smono_20260721T053258Z.log`).

## 1. The eval (built, self-test green)

Runner: `scripts/seeker/nn_tier/eval_heldout_smono.py` (`.venv-seeker`).
Self-test: `.venv-seeker/bin/python scripts/seeker/nn_tier/eval_heldout_smono.py --self-test`
→ **PASS, exit 0** (scorer math, box_scoring gate integration, scale/position binning,
plus a 5-image real end-to-end tiny run).

- **Split (anti-mirage):** the nn_tier TEST split is SOURCE/VIDEO/SCENE-disjoint
  (`docs/nn_tier/dataset_report.md`): `dut` = whole never-seen source (3000 imgs), `nps` =
  whole held-out clips (758), `plates` = held-out photographers (417). n = 4175 images
  (3706 pos / 469 neg; 4315 GT boxes). NEVER random frames (ADR-0061).
- **Modality:** GRAYSCALE (OV9281 deployment track; camera_paper_check item 4).
- **Metrics:** AP50; box-level recall/precision @ conf 0.25 (`v3_onnx_infer.DEPLOY_CONF`);
  per-GT-box **gate-recall** via the FIXED `scripts/seeker/box_scoring.box_hits_gt`
  (tol=15, size_ratio=3; `gt_scale=1.0`, `offaxis_aware=False` because real labels are
  drawn on actual pixels — the sec² widening corrects paraxial sim gt only); recall by
  SCALE (px @1280-eq bins 0-12/12-24/24-40/40-80/80+; the 6–20 m band of the 0.35 m
  target ≈ 7–36 px @1280-eq per `viewpoint_and_deploy_spec.md` A3) and by frame POSITION
  (3×3 grid); false-fire density on the 469 drone-free negatives by source; CPU ms/frame
  (this box, not Pi).
- **Comparison bar** (same split, same gray mode): deployed `drone_finetuned_quad_v2`,
  `yolo11x_mit`, `flyingobj_mit`. @1280 heavies run a deterministic seed-0 subset (all 469
  negatives + 700 positives); every model is ALSO summarized on that exact subset
  (`subset=heavy`) for apples-to-apples. **Bank running detached at report time**
  (v2 ~40% done; progress `logs/nn_tier/eval_smono_progress_cmp1.log`, outputs
  `logs/nn_tier/eval_s-mono_{summary,perimage,perbox,breakdown,clutter}_cmp1.csv).
- **Clutter false-fire:** transfer-bet drone-free segments (17 frames: bird/seagull/sky/
  ground) + the 4 `pos_photo_dji` frames as an UNLABELED anecdote
  (`docs/transfer_bet_kill_test.md`; density, never recall).

Prior-context bars already measured (DVB corpus, gray — `docs/nn_tier/baseline_scoreboard.md`):
v2_deployed AP50 0.0008 / recall 0.03 / bird false-fire 0.97; yolo11x_mit AP50 0.208 /
recall 0.46 / false-fire 0.62; flyingobj_mit ≈ 0 everywhere.

## 2. Deployability — the s-scale desk numbers (pre-registered; bench-gated)

From `docs/nn_tier/viewpoint_and_deploy_spec.md` B2–B6 (Hailo Model Zoo HAILO8L, DFC
v2.19.0; thresholds B4):

| item | yolo11s (s-mono) | yolo11n (n-mono anchor) |
|---|---|---|
| params / GFLOPs | 9.4 M / 21.6 | 2.6 M / 6.6 |
| ONNX fp32 (expected; measured when export lands) | ~36 MB | ~10 MB |
| Hailo-8L chip fps, batch=1, INT8 @640 | **92** | 157 |
| est. end-to-end Pi 5 fps | **~25–45 → MARGINAL vs the ≥30 bar** | ~40–80 → clears |
| B5 chip-rate gate (≥60 chip fps) | ✓ thin | ✓ |
| Pi 5 CPU-only | ~2–4 fps est (21.6/6.6 × the 5–10 fps n-scale anchor, constraint `pi5-compute`) — NOT viable | 5–10 fps — NOT viable |

**Scale-down framework verdict (B5 rule 5, pre-registered):** s-mono deploys ONLY if it
closes a REAL held-out-source recall gap vs n-mono AND later passes the ≥30 fps sustained
bench on the Pi (bench-gated, `pi5-emulation-gap`). Otherwise yolo11n stays the anchor.
The held-out comparison lands here when both exports exist.

## 3. To finish (same-turn checklist when the sentinel fires)

1. `onnx_smoke_test_smono.py` (load+run gate) → then
   `eval_heldout_smono.py --models s_mono n_mono --mode gray --clutter --tag smono1`
   (full 4175-image split; n_mono row if its export exists).
2. Append: scoreboard rows, scale/position breakdowns, false-fire by source + clutter,
   measured ONNX size + this-box CPU ms/frame, and the B5 rule-5 n-vs-s verdict.
3. Numbers trace to `logs/nn_tier/eval_s-mono_*_smono1.csv`.
