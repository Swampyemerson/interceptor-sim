# NEXT — top of the stack

*(One CURRENT section, one ordered build queue, one compressed Done list. Detail
lives in `docs/decisions.md` (ADRs) and `PROGRESS.md` (roll-up). Restructured
2026-07-06 during the Fable audit; rebuilt 2026-07-08 after the seeker-v2 +
fusion-P0/P1/P2 session.)*

## Current: capstone CLOSED (ADR-0044) — remaining items are builder-gated

The fusion capstone is DONE and the post-M5 software arc with it. Headline
(ADR-0044): **mid-course cue fusion WORKS under the markerless seeker** — the
hand-set polar FusedTrack beat fusion-off on 8/8 paired seeds (median
−0.356 m), rescued both chronic failures, flew 16/16 clean, and SURVIVED the
WORST-tier cue (8/8, max 2.88 m). `--fuse-midcourse` is now RECOMMENDED for
markerless configs. Covariance gating (EKF `correct_cue`) did NOT earn its
keep: variance blowup at EXPECTED (corrected 11/16 vs 16/16, though it also
produced the project's tightest-ever 0.151/0.229 m hits), REGRESSION at WORST
(5/8, 7.66 m tail — council finding F1 confirmed in flight). Honesty boundary
held everywhere: 0 post-latch cue updates across all 32 EKF flights.

### Open (all gated on the BUILDER or hardware)
1. **Hardware Stage 0 bench — ORDER PARTS (~$230)**: Pi 5 8GB + cooler/PSU/SD
   + global-shutter cam. Everything software-side is pre-staged and waiting.
2. **Bird MC gate #8**: needs the P(hostile) classifier (ADR-0035 red-team
   fixes + Stage-0 bench data). `bird_mc_harness.py` exits 1 by design.
3. **Parked**: deployment phases M-1..M-4 (design-as-ADR when picked up);
   README/portfolio polish pass over the new arcs (ADR-0038..0044) when the
   builder wants the resume artifact updated.

### Small logged follow-ups (non-blocking, ADR-0044)
- Near-CPA chi-square gating quirk (only if EKF path is ever promoted);
  FusedTrack state logging + L2 track-RMSE re-fly; p_diag_at_latch -> CSV;
  gain-washout metric; audit_per_tick cue-activity marker.

### Standing tooling from this session (use these)
- `scripts/audit_per_tick.py` — per-tick honesty audit for ANY batch arm
  (deployment-profile calibrated; (c) gates only when powered). Standing bar.
- `scripts/abort_lens.py` — pre/post-CPA abort reclassification
  (clean_corrected; 2.5 m Pk radius). Report alongside raw clean-rate.
- `scripts/check_seeker_v2.sh` — seeker v2 arc gate (6 checks, exit 0).
- `scripts/ekf_q_replay.py`, `scripts/ekf_lockout_forensics.py` — offline EKF
  replay harnesses (fixed-dt cadence; screens, not proof).
- `scripts/seeker/capture_pass.sh` + `capture_flight_frames.py` +
  `merge_dataset_v2.py` + `calibrate_range.py` — the v2 dataset/calib pipeline.
- Fine-tuned weights: `MARKERLESS_NN_WEIGHTS=scripts/seeker/weights/`
  **`drone_finetuned_v2.onnx`** is the RECOMMENDED markerless NN (sidecar
  auto-loads); v1 kept for reproducibility; `--seeker markerless` default
  (off-the-shelf two-stage) unchanged.

## Build queue (post-M5 — builder ratified 2026-07-07, ADR-0033; status 2026-07-08)

1. **Hardware Stage 0 bench (ADR-0012), ~$230 — BUILDER ACTION: order parts**
   (Pi 5 8GB ~$120, cooler/PSU/SD ~$35, global-shutter cam + wide lens ~$75).
   Software prep pre-staged; everything else can proceed without it.
2. **Kill the AprilTag (markerless seeker)** — DONE through v2 (ADR-0038/39/40/42):
   flies camera-only on the tag-less body, 6/8 clean, zero pollution, honest
   range, audit-clean. Remaining gap is guidance-side (Current item 2). The
   drone-vs-bird discrimination interlock still gates any "lethal" framing
   (ADR-0035 red-team fixes + Stage-0 bench → bird MC gate #8).
3. **EKF target-track A/B** — DONE + CORRECTED (ADR-0037 + addendum): miss/RMSE
   null; the clean-rate regression was post-CPA scoring bookkeeping; corrected
   = full parity at Q=64. `--tracker ekf` stays default-OFF pending L0.
4. **CAPSTONE — covariance-gated mid-course fusion** — design RATIFIED
   (ADR-0041, council-reviewed, two offline-demonstrated failure modes fixed
   pre-build); P0 done; P1+P2 built+reviewed on the worktree branch; P3/P4 =
   Current items 1/3/4.

## Follow-up flagged (not yet built)

- **P1/P2 loose ends (ADR-0041 M3):** `p_diag_at_latch` not yet logged to
  CSV/S2_RESULT; gain-washout offline metric not built; `audit_per_tick.py`
  fusion extension (per-tick cue-activity marker) pending; new tuning
  constants (latch floor 3.0 m, recovery floors, N=3) need a sanity pass
  before the L arms.
- **Abort-branch semantics (ADR-0037 addendum):** the analyzer-level lens is
  the adopted fix; flight-code branch redesign only if L0 shows it still
  binds. The M5/ADR-0036 clean-rates were apriltag-arm (unaffected — verified
  8/8 with the lens).
- **`compose_demo.sh`/`demo_capture_frames.py` time-alignment** (ADR-0032
  "Sync gotcha") — unchanged from before.

## Parked (designed, not scheduled)

- Deployment-profile phases M-1..M-4 (ground standby → launch-on-detect →
  climb-out dash → end-to-end timeline). Design-as-ADR first when picked up.
- P-9 real seeker / real ground rig; jam-envelope Gazebo confirmation (lab
  study done, ADR-0020).
- Bird MC gate #8 (`scripts/bird_mc_harness.py` exits 1 by design until the
  P(hostile) classifier exists; gated behind ADR-0035 fixes + Stage-0).

## Done (newest first — one line each; the ADR holds the story)

- **2026-07-08 seeker v2 (ADR-0042):** hard negatives + calib — pollution
  0.751→0.000, range 0.056→0.935, probe 8/8, per-tick audit green; clean-rate
  6/8 honest FAIL; mechanism = terminal bearing-noise throughput; v1's tight
  misses partly a broken-range artifact. `check_seeker_v2.sh` 6/6.
- **2026-07-08 fusion P0–P2 (ADR-0041 + addenda):** design council-reviewed
  (2 executed-experiment findings: tuned-Q bias-lock 81–100% camera rejection;
  Cartesian cue → ~9° bearing poison); Q-tune RETIRED — ADR-0037's regression
  was post-CPA bookkeeping (EKF exonerated, `abort_lens.py`); P1+P2 built,
  adversarially reviewed, fixed (31/31 tests) on the worktree branch.
- **2026-07-08 per-tick audit debt PAID (ADR-0041):** `audit_per_tick.py`;
  24 retro-audited flights (a)/(b) clean; (c) recalibrated for the deployment
  profile; 3 powered-(c) fails traced to seeker false-locks (the canary v2
  cleared).
- **2026-07-08 markerless A→C→B arc (ADR-0038/0039/0040):** off-the-shelf
  markerless flies (6/8, +1.03 m median); no handoff-timing lever recovers it;
  in-domain fine-tune v1 closes acquisition (2/8→8/8 static) but positives-only
  = necessary-not-sufficient.
- **2026-07-07 portfolio demo video (ADR-0032);** M5 final batch n=96
  Pk@2.5 27%→100% (ADR-0036); proximity metric ratified (ADR-0025); terminal
  diagnosis: miss is KINEMATIC (ADR-0023); EKF A/B null (ADR-0037, corrected
  2026-07-08); fusion Gazebo null under clean tag + αβ (ADR-0018) — the
  capstone re-opens it under markerless + EKF (ADR-0034/0041).
- **2026-07-05 M4.5-S1/S2, M4, M3, hardware stack (ADR-0009..0013);
  2026-07-04 M0–M2 (ADR-0001..0007).** All gated; PROGRESS.md has the table.

## Key facts for a fresh session

- PX4 at `~/PX4-Autopilot` (v1.17.0). Camera drone on the M2 world:
  `PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH=~/interceptor-sim/models HEADLESS=1 make px4_sitl gz_x500_mono_cam`.
  Markerless world: same line with `PX4_GZ_WORLD=markerless` (+ env
  `INTERCEPTOR_WORLD_NAME=markerless INTERCEPTOR_TARGET_MODEL=fpv_target_markerless`).
  World .sdf files must be symlinked into PX4's worlds dir (ADR-0005;
  check_m2.sh repairs the apriltag one).
- Camera: 1280×960 @ 30 Hz, fx=fy≈539.936, cx=640, cy=480
  (`camera_intrinsics.json`). Topics under `/world/<world>/...`.
- Boot-complete grep: "Startup script returned successfully" (ADR-0004).
  MAVSDK: `udpin://0.0.0.0:14540`. gt via `/world/<world>/pose/info` —
  SCORING ONLY (honesty boundary; per-tick audit enforces).
- Batch arm = mc-batch skill: `S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m
  0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma
  0.5"` + `MC_WORLD/MC_TARGET_MODEL/MC_SEEKER/MC_VENV_PYTHON` env +
  `--extra-args "--dash-speed 16 --early-handoff --cue-velocity
  --dash-unclamp"`, master-seed 42, `--x0 6.5 --y0-mag 29.3`. ONE sim at a
  time; idle load; never pkill from a shell whose argv matches sim names
  (scripts are immune, inline commands are NOT — relearned 2026-07-08).
- Markerless flight venv: `MC_VENV_PYTHON=$PWD/.venv-seeker/bin/python`;
  training venv `.venv-seeker-train`; main `.venv` untouched/gated.
- A/B hygiene: paired seeds n≥8 + mechanism evidence; ~1 m terminal noise
  floor; "not significant at this n" language; binomial metrics need Wilson
  CI / n=16 for verdicts (ADR-0041 F5). World→NED: north=world_y,
  east=world_x (ADR-0013).
- Minor: m0_takeoff.py duplicates its final CSV row — tidy if reused.
