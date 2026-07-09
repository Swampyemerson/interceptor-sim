# NEXT — top of the stack

*(One CURRENT section, one ordered build queue, one compressed Done list. Detail
lives in `docs/decisions.md` (ADRs) and `PROGRESS.md` (roll-up). Restructured
2026-07-06 during the Fable audit; rebuilt 2026-07-08 after the seeker-v2 +
fusion-P0/P1/P2 session.)*

## Current: PHASE 2 — Sim-to-Real (real pipelines = hardware blueprint)

**Builder ratified 2026-07-08 (evening).** Phase 1 (guidance + honesty +
markerless seeker + mid-course fusion) is DONE and gated (ADR-0001..0044).
Phase 2 replaces every MOCK with a REAL, implementable pipeline so the sim
becomes the hardware build blueprint. **Full plan + recommendations + ADR
skeletons: `docs/phase2_sim_to_real_plan.md` — READ IT FIRST.**

**Drive this autonomously.** Complete a step → gate → commit → START the next
unblocked step the same session ([[session-persistence-mandate]]). Between sim
runs, pull design/analysis forward. **Proactively run the analytical deep-dives**
(anti-jam mechanics, coverage, sim-to-real gaps) and answer from the logs —
don't wait to be asked. Council the one-way-door forks (F1/F2/F3 in the plan)
before building; verifier-gate every close-out; honesty boundary re-earned on
every new cue path.

### Phase-2 build queue (dependency-ordered; the durable todo)
1. **Stereo rig world + two real cameras** (T16, foundation) — **F1 DECIDED
   2026-07-08 (ADR-0046, council 3/3): trajectory-matched offline-render
   REPLAY** — rig renders offline at full 1920×1200 along the deterministic
   mover path; the REAL detect+triangulate+track code runs LIVE at flight
   time on those frames (sim-time paced, real :47800 link); live rendering
   deferred behind a pre-registered probe gate; separate-sim rejected.
   Build: `models/ground_stereo_rig/` + `worlds/stereo_intercept.sdf`
   (rig at broadside_160m) + snapshot-capture harness + gate script.
   Blocks 2,3,4 (T17 needs the rig viewpoint/intrinsics).
2. **Ground NN detector** (T17) — reuse the onboard v2 render→fine-tune→calibrate
   recipe from the ground viewpoint. ∥ with 7,9 (design docs); its render step
   holds the simulator, so sim-serialized with 1,3,6.
3. **Triangulation + ground velocity/track** (T18, needs 1+2) — 2-cam → 3D
   position + velocity; **VALIDATE measured σ_R vs ADR-0017 c=4.45e-05** (pivotal
   — may revise every fusion conclusion).
4. **Compute split + real link** (T19, needs 3) — `ground_station.py` process
   emits the real track; `m4_intercept.py` consumes unchanged; `--cue-source
   {mock,stereo}` flag, default `mock` (gated reproducibility preserved). Council F2.
5. **Fusion refinement** (T20) — bias-state EKF (estimate the datum offset, kill
   the WORST-tier bias-lock) + camera-favoring confidence. Prototype on mock;
   ADOPT only on real stereo data (builder's sequencing). Attacks the ADR-0044 null.
6. **Higher-speed + maneuvering arms** (T21) — 12 m/s + weave/jink with markerless
   + fusion (arc was 9 m/s straight-line only). Sim, serialized. Can run on mock now,
   re-run on real cue after 4.
7. **FPV fidelity** (T22, design first) — real FPV speed/accel/payload; `--fpv-fast`
   profile. Note ADR-0028: guidance ceiling binds, not airframe agility.
8. **Sim-to-real shortcomings audit** (T23) — gap table (severity + bench-measurable):
   frames, latency, comms, GPS/datum, intrinsics, IMU/EKF2, thermal/night, safety.
9. **Real-world NN transfer plan** (T24) — MIT model vs camera fine-tune, onboard +
   ground; Stage-0 data loop; what transfers vs rebuilt.

### Still builder-gated / parked (carried from Phase 1 close)
- **Hardware Stage 0 bench — ORDER PARTS (~$230)**: Pi 5 8GB + global-shutter cam.
  Blocks the bird gate + the real-seeker bearing-quality lever; software pre-staged.
- **Bird MC gate #8**: needs the P(hostile) classifier (ADR-0035 + Stage-0 data).
- Deployment phases M-1..M-4 (brief drafted `docs/deployment_phases_design_brief.md`);
  README/portfolio polish over ADR-0038..0044.

### Small logged follow-ups (non-blocking, ADR-0044)
- Near-CPA chi-square gating quirk (only if EKF path is promoted); FusedTrack
  state logging + L2 track-RMSE re-fly; p_diag_at_latch -> CSV; gain-washout
  metric; audit_per_tick cue-activity marker.

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
