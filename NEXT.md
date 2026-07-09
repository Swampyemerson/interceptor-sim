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
1. **Stereo rig world + two real cameras** (T16) — **DONE 2026-07-08 late
   evening, verifier-gated (check_t16.sh 5/5, independent re-run exit 0).**
   F1 decided (ADR-0046, council 3/3): trajectory-matched offline-render
   REPLAY — rig renders offline at full 1920×1200 along the deterministic
   mover path; the REAL detect+triangulate+track code runs LIVE at flight
   time on those frames (sim-time paced, real :47800 link); live rendering
   deferred behind a pre-registered probe gate; separate-sim rejected.
   Shipped: `models/ground_stereo_rig/`, `worlds/stereo_intercept.sdf`
   (rig at broadside_160m), `scripts/rig_snapshot_capture.py`,
   `scripts/check_t16.sh`. Next sim job: full capture sweep (both
   directions) = T17's dataset + T18's replay sequence.
2. **Ground NN detector** (T17) — **DONE + verifier-gated (ADR-0049,
   check_t17.sh exit 0) but NECESSARY-NOT-SUFFICIENT.** Single-class ground
   detector (ADR-0047 acquire-then-track), gate PASS is TRUE (no cheating,
   held-out-by-sequence, pixel-only). BUT proves only in-domain single-
   operating-point lock: all captures ~160 m / fixed 35 px box (detection
   floor untested), 1 unique negative scene, train/val near-pixel-identical.
   ground_v1 ADOPTED as T18/T19 PIPELINE DRIVER only; NOT the deployment
   detector. **KNOWN GAP → needs T17-v2 (multi-range captures + diverse
   negatives/backgrounds; Stage-0 real data) before any envelope/lethal
   framing.** Scripts committed; weights/dataset gitignored (regenerable).
3. **Triangulation + ground velocity/track** (T18, needs 1+2) — **scaffold
   DONE + validated (perfect-centroid σ_R matches model 1-2%); PIVOTAL
   CHECKPOINT run (ADR-0050): real-detector σ_R at ~161 m handoff = 0.48 m
   CLEAN (UNDER the mock's 1.15 m) → NO RED FLAG, fusion conclusions not
   threatened.** BUT best-case/thin (single range, n=10, train-adjacent; the
   naive 8 m ALL-number is a leakage+outlier artifact, quarantined). σ_R∝R²
   UNTESTED. Harness `scripts/t18_nn_sigma_validation.py`, notes
   `docs/t18_nn_validation_notes.md`. **σ_R not CLOSED → gated on the
   independent multi-range capture (below).**
4. **Compute split + real link** (T19, needs 3) — `ground_station.py` process
   emits the real track; `m4_intercept.py` consumes unchanged; `--cue-source
   {mock,stereo}` flag, default `mock` (gated reproducibility preserved).
   **F2 DECIDED (ADR-0048, council 3/3): reuse :47800 JSON; m4 spawns
   ground_station.py like the mock; sim-time via DUAL-GATE buffer-release
   (`sim_time>=deliver_t AND result_ready` — floor violations logged, floor
   profiled under concurrent Gazebo render); reproducibility via AST-token
   pin of the mock argv + s2_cue_mock hash pin (no shared helper between
   branches); ground_station.py runs in .venv-seeker (explicit venv const,
   not sys.executable).** Build gated on T18 real data.
   - **T19a DONE (offline, uncommitted→committed): `scripts/ground_station/
     station.py` + `detect.py` + `tests/test_ground_station.py` (17 tests).**
     Dual-gate CueScheduler built + offline-validated; .venv-seeker resolved
     (gz-transport + onnxruntime, native inference not ultralytics); a latent
     consumer-loop deadlock (stranded pending item on mid-frame exception)
     found + fixed. Full suite 56 pass, repro pins still 5/5.
   - **★ MEASURED FINDING → T19b DESIGN FORK (F4):** real detection is
     ~0.85–1.25 s/frame-pair ON CPU (.venv-seeker onnxruntime) — the dual-gate
     correctly HELD (proved it's load-bearing, not cosmetic) but ~1 s/frame is
     far too slow for a live 10 Hz cue. **T19b must resolve: (a) onnxruntime-
     GPU on the 4070 (likely ~10-30 ms → vindicates ADR-0046 live-detection);
     (b) precompute detections, replay cached centroids live (real triangulate
     live, detection cached — hybrid); (c) large floor + reduced cue rate.**
     Check (a) FIRST (cheapest, keeps the ratified design). This is the
     ground-station analog of ADR-0046's live-render feasibility gate.
   - **T19b wiring DONE (ace11d0): m4 `--cue-source {mock,stereo}` + explicit
     .venv-seeker interpreter; ALL 5 repro pins green (every gated path
     byte-identical); FLIP-LATER pins flipped.** Live-emit path smoke-VALIDATED
     (station.py subscribes to real /clock, emits correct positions, 0 floor
     violations).
   - **★ T19 VALIDATED + verifier-gated (ADR-0052 RESOLVED, e22d6a8,
     check_t19.sh exit 0): the drone flies on a genuinely computed stereo
     cue.** The first flight found a real clock-epoch bug (cache clock vs
     flight clock → burst → 24.36 m); fixed by epoch-rebase + mover co-start
     + a --min-conf gate (drops the ADR-0050 seq=2 outlier via the ordinary
     miss path). Cue tracks the moving target MEDIAN 0.99 m / MAX 2.06 m;
     handoff at ~8 m; miss ~3 m; audit clean (0/73 post-handoff cue leakage);
     5 pins green. **The whole T16→T17→T18→T19 real-pipeline spine works
     end-to-end.** The epoch bug IS the ground↔drone time-sync (PPS/RTK)
     problem ADR-0015/0017 flagged — now demonstrated + a headline T23 gap.
   - **T19 still OWES (non-blocking):** real latency-floor profile under
     concurrent Gazebo render; real `--spoof` corruption modes; the
     delivered-latency gate assertion. And T19's real-cue A/B vs mock (does
     the drone intercept as well on the real cue?) is a follow-up arm.
5. **Fusion refinement** (T20) — bias-state EKF (estimate the datum offset, kill
   the WORST-tier bias-lock) + camera-favoring confidence. Prototype on mock;
   ADOPT only on real stereo data (builder's sequencing). Attacks the ADR-0044 null.
6. **Higher-speed + maneuvering arms** (T21) — 12 m/s + weave/jink with markerless
   + fusion (arc was 9 m/s straight-line only). Sim, serialized. Can run on mock now,
   re-run on real cue after 4.
7. **FPV fidelity** (T22, design first) — real FPV speed/accel/payload; `--fpv-fast`
   profile. Note ADR-0028: guidance ceiling binds, not airframe agility.
8. **Sim-to-real shortcomings audit** (T23) — **DRAFTED `docs/sim_to_real_gaps.md`:**
   26-gap table across 9 dimensions, 23 FLATTER the design (dangerous kind); headline
   = the ADR-0052 clock-epoch/time-sync finding (critical). Stage-0 priority list
   (motion-blur L2 #1 — testable today, most-repeated "existential" gap). Living map;
   fold in future bench/flight findings. Honesty boundary framed as a STRENGTH.
9. **Real-world NN transfer plan** (T24) — MIT model vs camera fine-tune, onboard +
   ground; Stage-0 data loop; what transfers vs rebuilt.
10. **FINAL — Phase-2 demo video** (T25, builder-requested; do LAST, after the real
    pipeline T16–T19 + the maneuvering/fastest-speed arm T21 produce loggable
    flights — else it demos the mock/straight-line, not the real story). Successor
    to the ADR-0032 hero video (tooling to reuse/extend: `scripts/build_demo.py`,
    `compose_demo.sh`, `demo_capture_frames.py`; output pattern `demo_out/*.mp4`).
    **Narrative arc:** (1) OPEN on BOTH ground stereo rig views (split/side-by-side)
    detecting the threat near MAX reliable range (~150–160 m EXPECTED envelope,
    docs/stereo_design.md — NOT the 60 m WORST floor) with a HUD flash at the
    INSTANT of NN detection (bbox + "TARGET ACQUIRED" + range/bearing, driven from
    the REAL detector's actual detection frame — do not fake it). (2) A few seconds
    later CUT to the onboard drone camera as the interceptor takes off + begins the
    dash. (3) SLOW-MO the final seconds through interception. **Requirements:**
    threat on a NON-straight track (weave/jink mover — needs the T21 maneuvering
    flight logged first; the markerless+fusion arc is straight-line-only so far, so
    either fly T21 maneuvering first OR fall back to the tag arm if markerless
    maneuvering isn't ready — decide honestly). Fastest RELIABLE-speed intercept
    from the logged regime (M5/ADR-0025: 12 m/s dash clean under the decoupled
    terminal-speed rule, ADR-0033 #2 — confirm the equivalent for the
    markerless/fusion+real-cue config before committing the speed; do NOT inflate).
    **HUD sensor-attribution (the anti-jam money shot):** label which sensor owns
    the track each phase — ground stereo cue MID-COURSE → HANDOFF-LATCH marker →
    onboard-camera-ONLY terminal — with the fusion confidence / track-owner
    indicator flipping at the latch (visualize the honesty boundary, ADR-0044).
    **Production:** use any skills/plugins/connectors (install as needed) to
    maximize quality — two-cam + onboard compositing, HUD overlays, slow-mo ramp,
    titles; render GPU-accelerated (4070, GUI/demo path, not the headless batch
    path) at the highest stable resolution. **Audit step (required):** after
    render, extract SEVERAL intermediate frames and review for improvements — HUD
    legibility, timing/sync (WATCH the ADR-0032 compose_demo.sh/demo_capture_frames.py
    time-alignment gotcha), framing, slow-mo smoothness, whether the detection-instant
    and handoff-latch moments read clearly; iterate before calling it final.
    Feasibility note: F1 (ADR-0046) renders the two rig views OFFLINE as
    trajectory-matched frames, so both stereo views exist to composite — the video
    is achievable under the chosen architecture.

### Phase-2 follow-ups surfaced this session (dependency notes)
- **Independent multi-range capture campaign (serves 3 needs — ADR-0050):**
  a second capture at different rig pose(s)/standoff so the target spans a
  RANGE of distances + different lighting, never touching ground_v1's
  training → (a) validates σ_R∝R² (closes T18's pivotal question), (b) tests
  detector generalization = the **T17-v2** the ADR-0049 gap needs, (c) gives
  the truly held-out σ_R for **T20 fusion adoption**. Do before leaning on the
  0.48 m number or adopting any fusion default on "real" data.
- **d3d12 batch adoption:** validated on check_m3 (ADR-0046 addendum); first
  d3d12 BATCH arm still owes a paired-seed sanity vs stock before A/B use.

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
