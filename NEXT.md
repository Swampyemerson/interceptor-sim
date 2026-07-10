# NEXT — top of the stack

*(One PLAN, one CURRENT block, one BUILD QUEUE, one compressed DONE list.
Detail lives in `docs/decisions.md` (ADRs), `PROGRESS.md` (roll-up), and
`docs/audit_findings_tracker.md` (the 81-finding audit ledger). Rewritten
2026-07-10 by the Fable PM pass — the layered overnight/resume history blocks
are collapsed into DONE; nothing was lost, the ADRs hold every story.)*

## 🎯 THE PLAN — path to "done + portfolio-ready" (PM decision, 2026-07-10)

The mission (GOALS.md) is a portfolio piece proving the guidance+vision core
with reproducible, logged numbers. Six steps, in value order:

1. ✅ **DONE — Statistics (#33).** Fresh post-FIX-A weave n=72 landed:
   **Pk@2.8 m 72/72 = 95.0% CI-LB** (the ≥95%-CI bar cleared), **Pk@2.5 m
   71/72 = 98.6%** point / 92.5% CI-LB (ADR-0064, `d81e046`). The resume
   line's statistical backbone is set.
2. **The demo (T25).** Assemble + render the Phase-2 demo video — the single
   most visible portfolio artifact. Tooling is assemble-ready (9302483,
   715ce85); the builder's stated precondition (v3 eval complete) is MET
   (ADR-0061). Needs builder go-confirm; GPU render slot is now the first
   free sim-queue item (#33 is DONE — see below).
3. **The perception lever — #35 FLOWN (ADR-0067) → #40 mount-compose is now
   the active step.** The A/B answered the availability question: a fixed
   up-tilt CLOSES the ADR-0060 FoV gap (dash-above-FoV 32% → 0%, in-FoV 61%
   → ~90%), but the paired acquisition gain is small (+2–3 m; the interim
   "+7.6 m" was baseline-confounded — ADR-0067 documents the catch) and the
   UNCOMPENSATED terminal cost is dose-dependent (Pk@2.5 8/8 → 2/8 by 35°).
   NO mount adopted. Path to a verdict on un-HOLDing ADR-0059 recovery:
   #40 mount-compose (compensate FIX-A derotation for the tilt) → re-fly
   terminal at the chosen angle (up15 = leading candidate, not a choice) →
   tilted+compensated recovery re-test. Adaptive tilt (ADR-0065, #46)
   gained standing from this result.
4. **Correctness closure (#44, #40 remainder, #43 remainder).** Drive the
   audited known-bugs list to fixed-or-ADR'd — no silent known defects.
5. **Honesty hardening (DEEP-H2 handoff-streak tests).** ✅ #42/DEEP-N1 FIXED
   (`84eb756`, tracker `c497468`) — the seeker-side AST no-gt scan is
   committed. Remaining: DEEP-H2 handoff-streak state-machine unit tests →
   interview-proof boundary.
6. **Publish (BUILDER).** GitHub remote (private first → public after a
   claims scrub), LICENSE, CI on `scripts/run_tests.sh`; order the Stage-0
   cart (~$257) for the hardware arc. Until a remote exists the entire
   evidence base has ZERO off-machine backup. **Binding constraint on the
   project-end portfolio regeneration (ADR-0066):** the retired portfolio
   docs carried hard-won claim-scoping fixes (tracker rows FWD-A1/A2/A5/A6,
   DEEP-P4/H3/H4 — comms-denied HELD, r² wording, superseded numbers);
   regenerated material MUST re-apply those FIXED rows, not resurrect the
   overclaims.

**Definition of done:** headline stats flown+gated; demo rendered; every claim
correctly scoped/HELD on every surface; audit backlog groomed to
tracked-or-accepted; repo publishable on a remote.

## 📍 CURRENT (2026-07-10, live — builder away, autonomous)

- **⚠️ SESSION OVERRIDE (2026-07-10, builder): Fable weekly limit ~90% — use
  OPUS (`claude-opus-4-8`) everywhere the routing rules say Fable (review /
  gap-spotting / planning / hard tasks) for the REST OF THIS SESSION. No
  `model: fable` subagents. Session-scoped; the weekly limit resets — the
  permanent CLAUDE.md Fable-first policy is unchanged. See memory
  `fable-limit-opus-override`.**
- **#40 mount-compose COMPLETE — code shipped + BOTH re-fly stages flown
  (ADR-0068 + 2 addenda, committed `1f5693b`/`bfdb86e`/`3bc890b`):**
  `--cam-mount-up-deg` composes the fixed mount into FIX-A's LOS derotation
  (byte-identical at 0.0; passed a 5-lens adversarial review, 13/15 findings
  fixed pre-commit incl. an INVERTED validity gate). **Stage-1 (nominal terminal
  parity) = FAIL** (up15-compensated worse 7/8, median +0.16 m) → NO fixed +15°
  mount adopted; BUT compensation WORKS (Pk@2.5 4/8→8/8, penalty +0.82→+0.16 m).
  **Stage-2 (comms-jam RECOVERY re-test) = NULL** and it RE-SCOPES the recovery
  limit: the tilt fixes FoV availability (dash-above-FoV 32%→0%) but recovery
  stays NULL (RH 3/16, camera-never-detected 12/16 vs #39 baseline 10/16 — no
  improvement). **The binding recovery limit is FAR-RANGE NN DETECTION, not the
  dash-pitch FoV** (consistent with ADR-0061 v3 NULL). Perception AVAILABILITY
  is solved by the tilt; perception SENSITIVITY (far-range recall under jam) is
  the remaining hard blocker.
- **Publish-prep COMMITTED (`2de924e`):** `LICENSE` (MIT),
  `docs/license_notice_weights.md` (AGPL-on-weights nuance + builder decision),
  `.github/workflows/ci.yml`. Inert until a remote exists; first CI run will need
  one dep-pin pass.

- **#33 Pk campaign DONE** (ADR-0064, `d81e046`): fresh post-FIX-A weave n=72 →
  **Pk@2.8 m 72/72 = CP-LB 95.0%** (the ≥95%-CI bar cleared); **Pk@2.5 m 71/72 =
  98.6%** point / 92.5% LB (the one 2.757 m miss handed off cleanly = terminal
  noise, not a failure). Evidence CSVs committed. Radius-explicit framing (never
  a bare "95% Pk"); weave/12 m/s only, never pooled across paths.
- **#35 up-tilt A/B FLOWN + analyzed (ADR-0067):** mechanism CONFIRMED —
  dash-above-FoV 32% → **0%** at every tilt, in-FoV 61% → ~90%. But: paired
  acquisition gain only **+2–3 m** vs the up00 control (the earlier
  "14.3 → 21.9 m" read compared against the OLD r2 baseline, which detects
  5.3 m later than up00 on identical seeds — confound caught, corrected);
  uncompensated terminal penalty is dose-dependent (miss med 1.81 → 2.79 m,
  Pk@2.5 8/8 → 2/8; guidance still assumes zero mount). NO mount adopted —
  #40 mount-compose now gates everything terminal. Evidence CSVs + analysis
  committed. `models/mono_cam` shadow symlink **verified removed** post-chain.
- **adaptive camera tilt (ADR-0065, task #46) — case REVISED by the #40 Stage-2
  NULL.** Still valuable for NOMINAL acquisition (a pitch-following schedule buys
  the availability without the fixed mount's dose-dependent terminal cost —
  Stage-1 showed that cost is real). BUT it will NOT un-HOLD comms-denied
  recovery: Stage-2 proved the recovery limit is FAR-RANGE NN DETECTION, not FoV
  pointing (the fixed tilt fixed the FoV and recovery stayed NULL). So #46 is a
  nominal-acquisition improvement, NOT the comms-denied fix. The comms-denied
  recovery lever is a better far-range detector (v3 was a NULL, ADR-0061 — this
  is genuinely hard).
- While ANY batch flies: do not touch `scripts/`, `m4_intercept.py`, or anything
  a flight imports; one sim at a time; idle load only. NOTE (measured): the
  headless sim renders on llvmpipe (software) → it is CPU-bound with the RTX 4070
  IDLE; light/subagent work runs fine alongside a batch and the final demo can
  GPU-render, but a second CPU-heavy sim/pytest still distorts RTF.
- **Claim state:** "works comms-denied" **HELD** — the ADR-0059 fix is
  validated FAIL-SAFE, recovery is a NULL (coast-search engaged but did not
  recover; perception-bound). v2 stays the deployed detector (v3 = honest
  NULL, ADR-0061). Adopted config: `--track --handoff-cue-gate 8` + FIX-A
  derotation (ADR-0058/0062). Jink n=16 post-fix: 16/16 Pk@2.5 (d393a6b).
- **Uncommitted on disk (intentional pre-drafts — commit with their tasks,
  never mid-batch):** the #30 blur evidence
  (`scripts/experiments/logs/blur_replay_sample.csv`) — everything else
  (the #35 up-tilt harness + evidence, the #33 sizing script) was committed
  with ADR-0067.
  **Never stage:** `yolo11n.pt` (now enforced by `.gitignore`).
  `models/mono_cam` shadow symlink: verified removed after the #35 chain
  (2026-07-10) — if a future sweep strands it, run
  `scripts/uptilt_ab_arm.sh --cleanup` before trusting any other batch's
  world. `.claude/settings.local.json` **IS git-tracked** (`4f098b7`) — the
  real rule is commit permission drift separately and deliberately, never
  bundled into a task commit.
- **2026-07-10 doc declutter (ADR-0066):** retired 9 non-load-bearing
  portfolio/one-off docs (`docs/{interviewer_prep,portfolio_bullets,
  portfolio_visuals,WRITEUP,audit_deep_2026-07-10,audit_forward_2026-07-10,
  audit_pipeline_frames_2026-07-10,overnight_report_2026-07-10,
  review_track_fix_delta_20260709}.md`) to git history — findings live in
  `docs/audit_findings_tracker.md` + the ADRs; portfolio material
  regenerates at project end by a dedicated agent. Recoverable at `e6f06d3`.
- **No git remote exists** — "push" is impossible; remote setup is a builder
  decision (account + public/private). Recommendation logged in THE PLAN §6.

### Key facts for a fresh session
- PX4 at `~/PX4-Autopilot` (v1.17.0). Boot: `PX4_GZ_WORLD=apriltag
  GZ_SIM_RESOURCE_PATH=~/interceptor-sim/models HEADLESS=1 make px4_sitl
  gz_x500_mono_cam` (markerless: `PX4_GZ_WORLD=markerless` +
  `INTERCEPTOR_WORLD_NAME=markerless INTERCEPTOR_TARGET_MODEL=fpv_target_markerless`).
  World .sdf files symlinked into PX4's worlds dir (ADR-0005).
- Camera 1280×960 @30 Hz, fx=fy≈539.936, cx=640, cy=480. Boot grep:
  "Startup script returned successfully". MAVSDK `udpin://0.0.0.0:14540`.
  gt via `/world/<world>/pose/info` — SCORING ONLY (per-tick audit enforces).
- Batch arm = mc-batch skill: `S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m
  0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"`
  + `MC_WORLD/MC_TARGET_MODEL/MC_SEEKER/MC_VENV_PYTHON` + `--extra-args
  "--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp"`, master-seed
  42, `--x0 6.5 --y0-mag 29.3`. Adopted deployment config adds `--track
  --handoff-cue-gate 8`. Never pkill/pgrep with inline sim-name patterns
  (self-kill) — kill/poll logic lives in script FILES.
- Venvs: flight `MC_VENV_PYTHON=$PWD/.venv-seeker/bin/python`; training
  `.venv-seeker-train` (CPU-only torch); main `.venv` gated. Weights:
  `drone_finetuned_v2.onnx` deployed (v3 NULL).
- A/B hygiene: paired seeds n≥8 + mechanism evidence; ~1 m terminal noise
  floor (~5 m at 12 m/s+maneuver); Wilson CI / n=16 for binomial verdicts.
  World→NED: north=world_y, east=world_x (ADR-0013). Tests:
  `scripts/run_tests.sh` (130 incl. onnx parity).

## 🔨 BUILD QUEUE

### Sim queue (serialized, idle load, one at a time)
1. **T25 demo video render** — GO CONFIRMED (builder 2026-07-10), IN PROGRESS.
   Must feature (1) THE TILTING, (2) best tracking, (3) stereo handoff, (4) a
   great interception clip.
   - **DONE: the interception CORE (2/3/4 elements).** §A onboard hero flight
     captured clean (r2 run 0, CPA 2.076 m, real handoff) → shots 3–6 built + a
     17.7 s partial cut `demo_out/t25/t25_demo.mp4` (shots 0/3/4/5/6/8, shot 6
     8× slow-mo). CONFOUND found+fixed: passive capture perturbs the flight; the
     CHASE camera (2nd render) was the culprit → capture ONBOARD-ONLY on the
     plain `markerless` world (render plan §A, RESOLVED). demo_out is gitignored
     (regenerable) — footage is local.
   - **DONE: the TILT shot (element 1).** up15 dash footage captured (onboard-only,
     clean, miss 1.979 m; first-det 16.9 m vs up00 12.4 m — the availability win
     visible in the demo itself). Built `scripts/video/make_tilt_ab_card.py` — a
     matched-15 m-range LEVEL vs UP-TILT A/B card (`demo_out/t25/shot3b_tilt_ab.png`),
     framed as NOMINAL availability (32%→0% above-FoV, ADR-0067 arm stat), NOT
     recovery (Stage-2 NULL). Now shot 3b in the cut → **22.7 s partial**
     (shots 0/3/3b/4/5/6/8).
   - **DONE: the STEREO opener (element 3).** §B rig capture (40 pose pairs,
     staged 134–138 m) + ground-station replay (`ground_v2.onnx`, first firm
     track 135 m, mean range err 0.46 m). Built `scripts/video/make_stereo_shots.py`
     — a HUD-forward L|R stereo clip (SEARCHING→TRACKING lamp, triangulated range
     vs gt trace) → shots 1–2. **ALL FOUR required elements now in the cut →
     30.7 s** (shots 0/1/2/3/3b/4/5/6/8).
   - **STILL OPEN (nice-to-have, not one of the 4 required):** shot 7 (chase
     angle — separate lighter pass; the 2nd rendered camera perturbs the flight,
     so capture it ALONE, not concurrent with an onboard hero). After that, a
     final frame-honesty audit + `assemble_t25.sh --require-all`. Storyboard
     `docs/t25_storyboard.md`, plan `scripts/video/t25_render_plan.md`.
2. **#40 mount-compose RE-FLY — FLOWN; Stage-1 verdict FAIL on strict parity
   (ADR-0068 addendum, `bfdb86e`).** up15-compensated worse on 7/8 seeds
   (median +0.16 m) → NO fixed +15° mount adoption. BUT compensation WORKS:
   it cut the terminal penalty ~5× (uncompensated median +0.82 m → +0.16 m)
   and RESTORED Pk@2.5 4/8 → 8/8; the availability win holds (dash-above-FoV
   35% → 0%, first-det +4 m). The residual + fixed-angle compromise STRENGTHEN
   #46 adaptive tilt. Compensation code is validated+shipped.
   - **[NEXT SIM ITEM] Stage-2 tilted+compensated RECOVERY re-test** — the
     headline comms-denied question. Fly the ADR-0059 coast-search jam arm at
     r18, n=16, WITH the up15 shadow + `--cam-mount-up-deg 15` (harness:
     `scripts/stage2_tilt_recovery_arm.sh`); score by `scripts/check_jam_mc.py`
     against the pre-registered Stage-2 thresholds (RECOVERY iff RH ≥ 11/16 AND
     J ≥ 8/16; MECHANISM: camera-never-detected ≤ 4/16 vs the #39 baseline
     10/16). A positive result un-HOLDs "works comms-denied" (scoped). The
     availability mechanism the recovery needs is now confirmed strong.
   - Remaining #40 follow-ups (separate, lower): gate derotation, M7
     slant-range, attitude CSV cols, FIX-B tuning gate.
   - **#46 adaptive tilt (ADR-0065)** is now the favored perception-availability
     path (fixed mount has a residual terminal cost adaptive scheduling avoids).
3. **[conditional, after #40] #46 adaptive camera tilt sim A/B (ADR-0065)**
   — pitch-following schedule vs the best fixed mount; strengthened by
   ADR-0067 (fixed angle buys availability at a terminal price adaptive
   scheduling would avoid); candidate to un-HOLD ADR-0059 recovery.
4. **[m4 window] #44 ψ/β time-alignment (P-M6)** — detection-latency × yaw
   LOS bias; distinct from FIX-A (time-alignment, not rotation); bites under
   yaw acceleration = exactly the maneuver regime. Then its A/B.
5. **#32 r_hat honesty campaign** — aspect/bias/freeze probes + the G5
   vertical probe (FWD-C4) pinned into its arm list.
6. **#43 remainder** — sim-gated `--epoch-t0` shared co-start + P-NEW1 RTF
   sensitivity pass (the "biggest unexamined item"; sim-free half DONE c724347).
7. **NEW (needs task #): G8 clutter/decoy-world batch** (DEEP-P7/FWD-C5) —
   the adopted gate radii have only ever seen one empty world; scopes the
   "0/155 false detections" headline.

### Sim-free lane (parallel, anytime — safe during batches)
- **NEW (needs task #): handoff-streak state-machine unit tests** (DEEP-H2) —
  AST-pin tightening now (sim-free); `update_handoff_streak()` extraction is
  an m4-window item (mirror the FIX-B pattern).
- **Publish prep:** LICENSE recommendation (MIT) + CI workflow on
  `run_tests.sh` — write ready-to-commit, activate when the remote exists.
- **Audit-backlog grooming** (tracker §4/§5 — UNTRACKED down to **27**, from
  the original 51 at first writing; DEEP-P1/P-commitsD/P-commitsE are
  already FIXED, not residue): FWD-B5 HMAC cue-auth ADR draft (design only);
  FWD-B1 salvo-correlation $0 desk probe (both already TRACKED, sitting
  here). Top still-UNTRACKED per §4: DEEP-N2 (m4-side honesty-check scope —
  widen the AST gt-checks to attribute-form, mirroring #42), FWD-B2
  (own-ship GNSS denial — same fail-closed class as ADR-0059, undocumented),
  FWD-B4 (no stacked-WORST regression arm). **DEEP-R2 remainder** (PARTIAL:
  ADR-0060 pitch numbers need a `dash_pitch_probe.py --dump-summary-csv`
  re-run + commit) is real work but m4-window, not this-lane-safe (script
  execution).

### Builder decisions pending (surface when he's back — do NOT block on these)
1. **Order the Stage-0 cart** (~$257, #34; `docs/stage0_bench_plan.md`).
2. **GitHub remote** — recommend YES: private repo now (instant off-machine
   backup of the whole evidence base), flip public after a claims scrub +
   LICENSE. Needs his account + public/private choice.
3. **#30 real-footage NN eval** — needs his approval (multi-GB public dataset
   download or his phone video). The only probe that puts a real photon
   through the pipeline (FWD-C1).
4. **T25 go-confirm** (precondition met — see PLAN §2).

## ✅ DONE (newest first, one line each — the ADR holds the story)

- **2026-07-10 PM-pass day:** ADR-0062 FIX-A LOS full-attitude derotation
  (A/B 7/8 tighter, 0 worse) + FIX-B breakoff deadband (ships inert); jink
  n=16 post-fix 16/16; ADR-0059 recovery = NULL (coast-search engaged, did
  not recover; perception-bound → #35); audit tracker (81 findings, 51
  untracked → §4 backlog); doc-currency + portfolio-polish fixes; T19
  cue-error decomposition (#43 sim-free half); `scripts/run_tests.sh` (130
  tests); T25 tooling assemble-ready; CLAUDE.md Fable-first rewrite.
- **2026-07-10 overnight:** 3 deep audits (honesty spine HELD); v3 retrain =
  honest NULL (ADR-0061, forward audit caught a false win pre-score); jam MC
  = fail-closed demonstrated + fix validated FAIL-SAFE not recovery
  (ADR-0059); "works comms-denied" HELD; Option B ratified (Pi 5 + Hailo);
  design review delivered (ADR-0060 camera-pitch measured; blur bounded).
- **2026-07-09:** detect-then-track maneuvering fix (ADR-0058, d10c9ca) —
  camera-terminal weave 14/14, phantoms 12→0; builder maneuver directive
  SATISFIED. T21 limit diagnosed as perception not kinematics (ADR-0056/0057).
- **2026-07-08/09 Phase-2 spine (ADR-0045..0055):** stereo rig T16 → ground
  NN T17/T17-v2 → triangulation T18 (σ_R∝R² validated) → real-link T19 (drone
  flies on a genuinely computed stereo cue, 0.99 m median; clock-epoch bug =
  the real time-sync gap, ADR-0052); T20 fusion deferred as probable null.
- **2026-07-08:** seeker v2 (ADR-0042); fusion capstone CLOSED (ADR-0044 —
  hand-set FusedTrack 8/8, cov-gating doesn't earn its keep); markerless arc
  ADR-0038..0040; EKF A/B corrected null (ADR-0037).
- **2026-07-07:** M5 final batch n=96 Pk@2.5 100% (ADR-0036); demo video
  (ADR-0032); proximity-Pk metric ratified (ADR-0025).
- **2026-07-04/05:** M0–M4 + M4.5 realism + hardware stack (ADR-0001..0014);
  perception-half design (ADR-0015..0024).
