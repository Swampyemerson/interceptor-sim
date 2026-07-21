# NEXT — top of the stack

> **Canonical project state → [`docs/project_state.json`](docs/project_state.json)** (view: `docs/dashboard.html` · hosted: https://claude.ai/code/artifact/eb5e40d1-c12a-4b87-bca0-589ad5af96fc). That contract is the source of truth; this file is the subordinate work queue — keep them consistent (the drift check + the contradiction ledger enforce it).

## ⬜ REMAINING SIM PREP — todo (2026-07-20; canonical = the dashboard `build_plan` P0 + constraints)

**Hardware is ORDERED** (2026-07-20, Fable GO-WITH-FIXES — target-drone stack + seeker kit + interceptor brain; see `bom_tiers`). While parts ship, the remaining **sim/desk prep**, all decision-relevant to the tripod test / the build:

**Prereq (do first):**
- [x] **Fix `box_hits_gt`** — DONE 2026-07-21 (`b0a947e`): `scripts/seeker/box_scoring.py` is the one gate (sec² off-axis widening, unified TARGET_EXTENT_M=0.52 mesh-measured, centre-lag helper). Re-score: frametop 19→95%, banked 0→100%, negative control unchanged 0/692 — the near-6th-mirage quantified. 17 unit tests; adversarially verified.

**The two un-eliminated in-view mechanisms (the corrected `headline.next_probe`):**
- [ ] **Pitched-down + ground-background sweep** — reproduce the REAL in-flight scene (nose-on quad vs GROUND with its shadow, not the frame-top sweep's belly-vs-sky). Tests the ground-clutter mechanism (detector was 0% on the horizon vs 76–100% on sky). Score with the fixed scorer.
- [ ] **Phantom-competition test** (8–11 m band) — top-1 masking amid the 5–25 box/frame in-flight phantom storm (0 static).

**P0 software (for the tripod day + the parallel bench) — [2026-07-20 push: all but P0.2 DONE, each adversarially verified]:**
- [x] **P0.2 placard-sizing auto-labeler validation** — DONE 2026-07-21: labeler VALIDATED (IoU 0.965 vs gt, min 0.951); **PLACARD = 0.35 m edge** (carry limit; only size clearing the t_go gate under realistic OV9281 scaling — `docs/placard_sizing.md`); PRINT UNBLOCKED. Find: GPU render corrupts fiducial decode at range → tag sweeps use `SIM_GPU_RENDER=0`.
- [x] **P0.3 two-curve tripod SCORER** — DONE: `scripts/seeker/tripod_score.py` (curve (a) tag decode envelope → Tier-2 money gate with explicit t_go≥0.5 s arithmetic incl. the 5-streak burn; curve (b) NN recall vs range × position-in-frame → Hailo gate only; NN half no-ops cleanly without onnxruntime). Advisory: R_decode90 reports the bin UPPER edge (documented, ±one 2 m bin).
- [x] **P0.4 link `--cam-fwd-offset` to the model swap** — DONE (`4272fdf`): m4 startup parses the ACTIVE model SDF and exits 2 on a compensation mismatch (`--allow-cam-offset-mismatch` escape hatch); pure guard logic in `flight.geometry` + guard-matrix unit tests; byte-identical stock default.
- [x] **P0.5 camera paper-check** — DONE: `docs/camera_paper_check.md`. 118° is HORIZONTAL (clears ≥100° with margin); **px/deg penalty ~15%, NOT ~30%** (the 30% was the 148° diagonal misread — ledger `ov9281-pxdeg-30-vs-15`); ≤1 ms exposure clears (~40 µs floor; set `FrameDurationLimits` alongside); mono-vs-color NN confound UNVERIFIED-ON-PAPER (real-data mono retrain is the sanctioned fix); 22→15 CSI cable correct; `dtoverlay=ov9281` is mainline.
- [x] **Pi-side capture script** — DONE: `scripts/seeker/pi_capture.py` (picamera2/v4l2/dir-replay backends; session = frames/ + index.csv + meta.json + tags.csv; logs the ACTUAL applied exposure vs the ≤1 ms spec; autolabel consumes the layout directly, 100% label-rate verified). Bench-test on the real Pi when it arrives (build_tab `skr-05`).
- [x] **`field_score` ArduPilot-log support** — DONE: DataFlash .BIN via pymavlink DFReader (POS preferred over GPS, GWk/GMS UTC sync, per-side `--ulog/--bin/--csv`); self-test 5/5 incl. an end-to-end synthetic .BIN byte stream.
- [x] **Validate `deploy_seeker` MAVSDK OFFBOARD path** — DONE against local PX4 SITL (`1bdd310`): 3 real defects fixed (assumed-airborne offboard start, no connect timeout, no clean shutdown); `scripts/check_deploy_sitl.sh` boots+runs+kills, exit 0; evidence `logs/deploy_seeker_sitl_20260720T*.log`. Re-run on the 6C Mini props-off bench when it arrives (build_tab `brn-05`).

Full detail/status: the dashboard `build_plan` (P0–P5) + the constraints in `docs/project_state.json`. **The "THE PLAN (2026-07-10)" below is the pre-pivot Phase-2 plan — superseded by the real-build pivot + the dashboard; kept for history.**

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
   (ADR-0061). Needs builder go-confirm. **✅ GPU RENDER NOW ON (ADR-0075,
   `8568f09`): `scripts/sim_gpu_render.sh` (sourced by all boot scripts) →
   RTX 4070 via d3d12, measured RTF 0.336→0.951 (2.8×), images identical;
   toggle `SIM_GPU_RENDER=0`. Unblocks faster renders + 4K (ADR-0074).**
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
   gained standing from this result. **[SUPERSEDED — #40 FLEW: Stage-1
   parity FAIL → NO fixed-mount adoption; Stage-2 recovery re-test NULL
   (ADR-0068 addenda, see the prior-autonomous block below); fixed-tilt
   adoption is graveyarded (docs/project_state.json) — ADAPTIVE tilt #46
   is the open pointing lever (ADR-0076 add #18k).]**
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

## 📍 CURRENT (2026-07-16, live)

> **🔵 ROUND-5 CORRECTION — the "approach-aspect detector wall" was the 5th mirage (ADR-0076 add #18k,
> Fable strategic-review pass, independently re-scored). The detector is NOT blind.** The controlled
> set-pose sweep (level camera, target teleported to 8/12/16/20/25 m on boresight, n=527 approach +
> 558 receding, fixed `box_hits_gt`) shows the DEPLOYED detector scores **100% recall at every range
> 8–22 m on BOTH the nose-on approach aspect and receding**, static/centred — same detector, same
> threshold that reads 0.8% IN FLIGHT (#18i). So the in-flight wall is **FLIGHT-DYNAMIC, not a
> static range/aspect/appearance/resolution deficit**: under the ~40° nose-down dash pitch the target
> sits at the FRAME-TOP edge (~41° off-axis, #18j-fix) where lens-edge skew + off-boresight + training
> never-showed-it kill recall. **Corrected levers:** (1) **POINTING/attitude is dominant** — re-centre
> the target (adaptive tilt #46/ADR-0065; tilt-aware gt now exists, add #18k(2)); (2) real-data
> detector is **DEMOTED to a hypothesis** for the 8–12 m band (it would ACE a static/hover capture and
> still fail in flight — the v3/rebal trap) and is really about the OUTDOOR appearance gap, not the
> sim wall. **#1 thing we're NOT measuring: recall vs POSITION-IN-FRAME & bank-attitude (not range).**
> NEXT decisive sim probe (cheap, offline-scoreable): re-run the sweep at the frame-top off-axis
> position × {level, banked} to decompose pointing/attitude/edge-clip and SPEC the real capture.
> Fable strategic review (acoustics + transfer-bet + Pi5) landed same pass — key calls: acoustics DEAD
> onboard (own-noise ~40 dB the wrong way) / OK as a ground launch-cue; MIT-NN transfer is hope-past-
> evidence → validate with a **~$310 tripod + tagged-target afternoon** (measure real approach recall
> vs range/frame-position BEFORE building); Pi5 CPU runs the **AprilTag baseline real-time**, markerless
> **REQUIRES the deferred HAT** (BOM §0c already ruled this — first kills fly the tag). Honest resume
> line = "PN terminal guidance handed off from a no-datalink coded dash, validated by Monte-Carlo miss
> stats; quantified the perception acquisition envelope and identified the flight-dynamic recall limit."

### 🎯 THE ROADMAP — path to the goal (Fable judge-panel workflow + red-team, 2026-07-15)
> **⛔ CORRECTION (ADR-0076 add #18g, Fable adversarial verification 2026-07-15):** the R1–R3
> "camera-guided r2l kill (0.72 m)" claimed below is RETRACTED — a control arm that never engaged
> the camera scored the same ~0.54 m, so the sub-meter r2l is OPEN-LOOP DASH BALLISTICS, not camera
> guidance; no camera-tracked <1 m 3D-quad intercept exists. The real wall = ~0 detector recall on
> the APPROACHING target (not the phantom). NEXT sim test = **camera-forward × coded-dash** (remove
> the phantom at source, `scripts/experiments/cam_forward/` + `--cam-fwd-offset-m 0.40`) before any
> hardware inherits "prop-clearance fixes acquisition" (untested). **[SUPERSEDED 2026-07-16: that
> test WAS FLOWN — ADR-0076 add #18h: phantom removed at source (approach-phase phantom detections
> 1187→1) yet approach recall STILL ~0 both directions → "prop-clearance fixes acquisition" is
> REFUTED; the phantom is the false-handoff hazard only. Current next probe = the ROUND-5 frame-top
> off-axis sweep (🔵 banner above / docs/project_state.json headline.next_probe).]** Read R2/R3
> below THROUGH this AND the 🔵 ROUND-5 (#18k) banner above.

**GOAL (builder):** an imprecise coded-dash interceptor hits a moving target flying ≥20 mph
(~9 m/s), outdoors, camera-only (no AprilTag on target; tag OK for calibration), to **<1 m**.
**HONEST VERDICT (red-team ENDORSED-with-fixes):** <1 m markerless is regime-split:
- **9 m/s MANEUVERING: out of reach (MEASURED)** — even the AprilTag (best possible bearing)
  floors at median 1.64 m @12 m/s weave (ADR-0056); 4+ terminal NULLs close the tighter lane.
- **9 m/s STRAIGHT-crossing, in sim: genuinely OPEN (coin-flip), decidable THIS WEEK** — but
  the goal condition (coded-dash + quad_v2 + **line** @9 m/s) has **NEVER been flown** (all our
  numbers are extrapolated from 12 m/s *weave*). No config is sub-meter in BOTH dirs at once yet.
  *[STALE — ADR-0076 add #18e flew line@9 (fired the reframe: 3.38 m median, 0/12 @1 m) and add #18h closed the camera-guided-sim question; the binding wall is now the FLIGHT-DYNAMIC recall deficit (add #18k). See the ROUND-5 banner + `docs/project_state.json`.]*
- **Hardware outdoors: unlikely (~10–30%) without an acquisition-range win**, AND — the metrology
  gap — consumer GPS can't even MEASURE <1 m (→ RTK now in the BOM). **[SUPERSEDED 2026-07-15: RTK
  later CUT by the §0c binary-kill re-scope — success = BINARY KILL on seeker video + both ULogs,
  not a measured <1 m CPA (docs/hardware_order_list.md §0c, docs/project_state.json).]**
**REFRAME to adopt (keep literal <1 m as a gated stretch):** a **speed ladder** (markerless <1 m
first at ~5 m/s), **tag-guided <1 m @9 m/s** as the guidance proof (tag = sanctioned calib), and
at 9 m/s markerless a **Pk@1.5 m (net-class kill)** — a *tightening* of the 2.5 m metric, not a retreat.

**ROADMAP R1→R7** (each with a measurable gate; [S]=sim [H]=hardware):
- **R1 [S] ✅ FLOWN (seed 123) → FIRES THE REFRAME.** Goal condition coded-dash + quad_v2, **line @9 m/s**,
  camera-guided only (ENGAGE-tick audit, red-team fix #1 done): **combined median 3.38 m** (l2r 2.16 m /
  4-of-8 cam-guided; r2l 3.46 m / 8-of-8), **Pk@1.0 = 0/12, Pk@1.5 = 0/12, Pk@2.5 = 3/12.** WORSE than
  the pre-registered 1.5–2.5 m and worse than weave@12 — and NOT an artifact (recorded miss = true CPA
  every flight). WHY: the straight line holds the target at a constant east offset (x=6.5 m), which
  **UNMASKS the markerless east-under-commit** that the weave's near-crossing hid (ADR-0056/add #18c).
  Per the pre-registered rule (>1.5 m), **the reframe is now the official headline: literal <1 m is not
  achievable even in the straight-crossing sim regime.** (`logs/mc_coded_dash_qv2_line9_s123.csv`.)
  REMAINING to firm it: AprilTag control @line9 (does the tag close <1 m? → isolates markerless-perception
  vs kinematic) + a 2nd disjoint seed. Tool + speed knob committed (`coded_dash_summary.py` ENGAGE audit,
  `coded_dash_arm.sh` `QUAD_ARM_SPEED`).
- **R2 [S] ✅ FIRST READ (line@9, seed123, +30° global east-bias):** the sideways-aim correction WORKS
  for r2l — cam-guided median **3.46 → 1.02 m**, Pk@1.0 0/8→**3/8**, Pk@1.5 0/8→**7/8**, best **0.43 m
  (contact range)** — so the r2l straight-crossing miss is a CORRECTABLE aim deficit (same as weave +30°
  → 0.78 m). But +30° is GLOBAL → it BREAKS l2r (2.16→5.09 m, 6/8 stop acquiring), so the fix must be
  PER-DIRECTION (r2l +east, l2r opposite/none). l2r line@9 ALSO has an acquisition problem at baseline
  (4/8 never lock → R3 crop / real-data). Mirror confirmed: l2r optimum −30° → 1.58 m (2/5 @1.5, still
  acq-limited). **✅ BUILT `--dash-crossing-bias-deg` (add #18e): auto-keys the bias sign on the crossing
  direction (dash × --target-vel, a pre-flight constant) → r2l +bias / l2r −bias in one config.
  VALIDATED on 2 seeds (123+777, n=16/dir pooled — the single-seed "6/8 @1m" was optimistic, 2nd seed
  corrected it):** r2l **median 0.72 m, Pk@1.0 9/16 (56%), Pk@1.5 11/16, best 0.37 m** — a big gain off
  the 3.46 m baseline, best flights make CONTACT, but at the EDGE of kill range, not yet reliable; l2r
  acq-limited (8/16 lock), median 1.50 m, Pk@1.0 1/8. Combined median 1.27 m, Pk@2.5 23/24. So the
  per-direction aim fix substantially closes the gap (esp. r2l), but a RELIABLE markerless kill isn't
  demonstrated yet — r2l borderline, l2r bottlenecked by ACQUISITION (→ R3).
  (`logs/mc_coded_dash_qv2_line9_xbias30_s{123,777}.csv`.)
  **AprilTag control ABANDONED:** the
  directional ~6 m-range tag is invisible during a 16 m/s crossing dash (0 detections) — a worse seeker
  than the NN here, can't be the "best-bearing" yardstick (reinforces the markerless-NN rationale).
- **R3 [S] IN PROGRESS — DIAGNOSIS redirected the lever.** l2r acquisition fails NOT from too few
  pixels (auto-crop) but from **implausible-range FALSE detections**: the l2r dash-only flights DO detect
  (45–57/flight) but their first detections are at 95–113 m gt-range = the own-prop PHANTOM (implies
  ~1.5 m) polluting the acquire stream so no clean handoff forms (r2l locks 16/16, l2r only 8/16). So the
  targeted fix is a **handoff RANGE-PLAUSIBILITY gate** (`--coded-dash-acquire-range-min/max`, add #18f,
  = the red-team's #1 no-cue transfer-risk hardening): a detection advances the 5-streak only if its
  implied range is inside a pre-flight-plausible window → rejects the phantom.
  **RESULT = NULL (add #18f), and it re-confirms the core ADR-0076 finding.** The deeper diagnosis:
  the l2r detections are median **implied-range 1.6 m while true range is 16.6 m** — i.e. ~48/49 are
  the PHANTOM and the real target at 16 m is barely detected; the detector locks the prop, not the
  target. And the range gate (min 4 m) broke BOTH directions (r2l 8/8→0, l2r 3/8→0) — it rejected the
  very detections the coded-dash relies on, because the phantom (~1.5 m) and the usable detections
  **OVERLAP in implied range** (exactly ADR-0076 add #13: phantom overlaps real in range AND image
  space). No range threshold separates them. **So the phantom / l2r-acquisition is NOT software-fixable**
  (range-gate null, rebal appearance-retrain null add #18d, ~12 prior ADR-0076 levers null): the fix is
  **HARDWARE prop-clearance (remove the phantom at the source, BOM §0②) + a real-data-trained detector
  (R6)** — the same conclusion the whole ADR-0076 saga reached. The `--coded-dash-acquire-range-*` flag
  stays default-off (documented null). **NET SIM LANDING: guidance/aim is SOLVED (crossing-bias → r2l
  0.72 m kill-edge); the l2r-acquisition + r2l-tightening residuals are the known PERCEPTION walls that
  need hardware + real data, not more sim levers. The next real progress is BUILDING (R4+).**
  **[SUPERSEDED 2026-07-16 by ADR-0076 add #18g/#18h/#18k — read via the 🔵 ROUND-5 banner /
  docs/project_state.json: "r2l 0.72 m kill-edge" was open-loop dash ballistics (retracted, #18g);
  prop-clearance survives only as the false-handoff-hazard mitigation, NOT the acquisition fix
  (#18h flew it — phantom removed, recall still ~0); the real-data detector is DEMOTED to a
  hypothesis (outdoor appearance gap); the dominant lever is POINTING/adaptive tilt (#46) with a
  cheap next sim probe — "no more sim levers / next progress is BUILDING" no longer holds.]**
- **R4 [H bench]** Build + calibrate + prop-clearance gate + Hailo ≥14 Hz + **blur gate at TERMINAL LOS
  rates** + real max-lateral-accel + **RTK metrology σ≤0.3 m (non-negotiable).** **[SUPERSEDED
  2026-07-15 by §0c: RTK CUT (binary-kill re-scope — kill evidence = seeker video + phone slow-mo +
  both ULogs; buy RTK later only for a citable number); the Hailo ≥14 Hz gate is DEFERRED to the
  markerless phase — first kills fly the AprilTag baseline on Pi 5 CPU (§0c).]**
- **R5 [H field, zero intercept risk]** The **range walk** (target hovering 5–25 m): measure R_acq/σ.
  KILL NUMBER: R_acq must leave t_go ≥0.5 s post-handoff (≈≥20 m for 9 m/s) — else markerless <1 m is dead.
- **R6 [H]** Tag-guided **speed ladder 2→5→9 m/s** (the feasibility DECIDER; offset-aim surrogate first),
  YOLO in shadow every flight (real-data fine-tune validated on held-out FLIGHTS, not frame-eval). Fly BOTH
  laws (pursuit beat pro-nav at line@9 in sim). Gate: tag median <1.0 m absolute @9 m/s.
- **R7 [H]** Markerless-guided, only rungs cleared in shadow. Gate: markerless <1.0 m absolute per rung.
> ⚠️ **R4–R7 METROLOGY SUPERSEDED (2026-07-15 binary-kill re-scope):** RTK was CUT from the BOM
> (hardware_order_list.md §0c; project_state.json graveyard), so the R6/R7 "<1.0 m absolute" field
> gates are unmeasurable as written (line above: consumer GPS can't measure <1 m). Current field
> gate/evidence = BINARY KILL (seeker video + phone slow-mo + both ULogs); "<1 m" stands only as a
> sim-side number or an RTK buy-back stretch.
**BIGGEST RISK:** acquisition-range / t_go starvation on the real 0.35 m target (physics: 640-pipeline
detects at ~9.5 m, streak burns ~7 m → ~0 correction authority). Retire early via R3 (sim) + R5 (1 afternoon).
**Red-team fixes 2–4 folded in above** (~~markerless IS sub-meter one-directionally~~ *[retracted — ADR-0076 add #18g: the sub-meter r2l was open-loop DASH BALLISTICS, not camera-guided]*; R1 middle-band defined; R6 adverse prior stated).

### 🔧🚁 THE PIVOT — BUILD THE REAL INTERCEPTOR (builder, 2026-07-15)
> ⛔ **BOM SNAPSHOT BELOW SUPERSEDED by the §0c binary-kill cost-cut (same day, 2026-07-15):**
> current lean BOM = interceptor **~$740** / ~$1,390 all-new; camera = **OV9281 mono GS wide**
> (AR0234 = upgrade path); **Hailo HAT DEFERRED** — first kills fly the AprilTag baseline on Pi 5
> CPU (ROUND-5 banner); **fixed ~15° up-tilt adoption is DEAD** (ADR-0068 Stage-1 parity FAIL —
> the bracket survives for prop-clearance only). See `docs/hardware_order_list.md` §0c +
> `docs/project_state.json`. The coded-dash ARCHITECTURE in this block is still current.
Moving from sim-portfolio to a REAL hardware build off `docs/hardware_order_list.md`
(~$1,089 interceptor: GEPRC Mark5 5" 6S quad, **Pixhawk 6C Mini / PX4 = the exact sim
stack**, Pi5+Hailo YOLO11n @640, Arducam AR0234 global-shutter color cam + ~100° M12
lens + fixed ~15° up-tilt). **Real operating model: a CODED open-loop DASH in the right
direction → CAMERA-ONLY pro-nav terminal** (NO ground-sensor cue, NO datalink mid-course,
NO cue-fusion/handoff-gate/coast-search). "Gear the code for that." This reframes the r2l
saga: it was entangled with the CUE-GUIDED DASH + FusedTrack machinery the real build
won't have; the genuine carry-over concern is CAMERA-ONLY terminal BEARING quality (the
higher-res global-shutter cam may change it). See memory [[real-build-pivot]] and the
consolidated **`docs/real_build_coded_dash.md`** (architecture + validated numbers + the
portable `flight/` core + camera pipeline + the hardware build path).
- **✅ Fable plan delivered (P0–P3):** rule "the flight architecture decides / ablate to
  flight config before spending levers." P0.1 build `--coded-dash`, P0.2 re-earn numbers,
  P0.3 extract a `flight/` core, P1–P3 = real HW (frame source, calibration, MAVSDK serial).
- **✅ P0.1 BUILT + COMMITTED (ADR-0076 add #18/#18b):** new `--coded-dash` mode = the REAL
  architecture — OPEN-LOOP dash → CAMERA-ONLY ENGAGE terminal, the ENTIRE cue/handoff/fusion
  stack DROPPED. Auto-heading solves the COLLISION-LEAD azimuth from the pre-flight target
  kinematics (honest constant, not a live gt read); explicit `--dash-heading-deg` overrides.
  Byte-identical without the flag. Harness: `scripts/coded_dash_arm.sh` (mode m4, no cue).
> **⛔ ATTRIBUTION SUPERSEDED (ADR-0076 add #18g/#18h — the ⛔ banner above also governs the two
> ⭐ bullets below):** the camera-terminal credit is RETRACTED — ENGAGE streaks were phantom and a
> dash-only control matched the with-terminal CPA (0.30 vs 0.29 m; the camera added ~nothing).
> Read "ALL 48 flights acquire+engage" as DASH-robustness only; "8/8 Pk @0.78 m" / "13/16" /
> "recovers r2l to sub-meter" as open-loop dash ballistics + aim calibration; and "not a
> perception floor" as WRONG — the flight-dynamic perception wall (add #18k) IS the binding wall.
> Current truth: `docs/project_state.json` (launch_aim / coded_dash / terminal notes).
- **⭐ RETROACTIVE r2l TEST — PAIRED A/B, n=16, quad_v2/seed123/weave (geometry verified
  byte-identical). Coded-dash HELPS r2l, mildly HURTS l2r:** cue-guided r2l engaged 8/8 but
  at 3.25–4.27 m = **0/8 within the 2.5 m Pk gate** (that IS the "r2l 0%" of the saga — NOT a
  no-engagement null; correcting an interim over-claim). Coded-dash lifts **r2l 0/8→5/8 @Pk,
  mean 3.97→2.68 m**; l2r stays 8/8 @Pk but mean loosens 0.61→1.37 m (the cue helped l2r).
  Net: the 6.5× l2r/r2l gap collapses to 2×, combined **Pk@2.5m 8/16→13/16**. PARTLY REFUTES
  ADR-0076 add #17 ("no fix reaches r2l"): removing the fused dash — not a lead patch —
  recovers ~1.3 m; the residual r2l gap is the ADR-0056 bearing aspect-bias. **This is the
  number that matters for the hardware (no cue on it).** (`logs/mc_coded_dash_qv2_weave.csv`)
- **⭐ ROBUSTNESS envelope (ADR-0076 add #18b/c, 48 flights, heading err 0/+15/+30°):**
  **acquisition survives ≥30° aim error — ALL 48 flights acquire+engage** (the hand-programmed
  dash only has to point roughly right; the camera terminal does the rest). l2r Pk tolerance
  ~15–20° (8/8 @+15°, 0/8 @+30°); **r2l improves monotonically with clockwise/east bias →
  8/8 Pk @0.78 m by +30°**, proving the r2l residual is a *systematic, fully-correctable
  east-aim deficit*, not a perception floor. **r2l LEVER (real, unbuilt):** a deliberate
  east-bias on the r2l aspect recovers r2l to sub-meter without touching l2r.
- **✅ P0.3 flight/ core COMPLETE + WIRED:** portable `flight/` (geometry LOS-derotation +
  alpha-beta estimator + coded-dash aim + closing-speed + pro-nav law + Brown-Conrady lens
  undistortion), 25 tests, NO gz/gt/cue deps → runs on the real Pixhawk/Pi. m4's coded-dash
  aim now CALLS `flight.guidance` (byte-identical). `flight/camera.py` = the honesty-critical
  wide-M12 undistortion (raw-pixel bearing is wrong toward frame edges).

### 🔬 SEEKER real-build roadmap (Fable review, 2026-07-15 — NN detection + real drones + range)
Full review in the session transcript; actionable distillation:
- **#1 [SIM, TESTED → NULL, do NOT ship rebal].** Re-A/B'd phantom-free `rebal` vs `quad_v2`
  under coded-dash. **Rebal FAILS in flight:** coverage 0.000–0.019 across ALL 16 flights, every
  one ABORTED "no camera acquire". The apparent "16/16 Pk@2.5" was a MIRAGE — the coded-dash
  never acquired, so those were the OPEN-LOOP DASH's CPAs, not camera intercepts (the `clean=0`
  / `python_exit_1` flag was the tell; the Pk headline alone would have shipped a blind seeker).
  The rebal retrain killed the phantom by OVER-SUPPRESSING real-target recall in flight — the
  same "aced-frame-eval, failed-in-flight" trap as the v3 NULL (ADR-0061; its "precision = v2"
  was a frame metric that didn't transfer). **`quad_v2` stays the working seeker** (coverage
  ~0.23–0.33, real intercepts, 13/16 @Pk). REDIRECT: the phantom fix for hardware is **geometry
  (prop-clearance mount, now in the BOM §0②) + the #2 no-cue handoff hardening**, NOT a
  phantom-free retrain. (Open follow-up: does a lower confidence threshold rescue rebal's flight
  recall? — a tuning question, not a blocker.) (`logs/mc_coded_dash_rebal_weave.csv`)
- **#2 [SIM, PARTLY TESTED → NULL — superseded, see R3 / ADR-0076 add #18f + ADR-0077]** Harden
  the coded-dash handoff for the no-cue world: require the 5-streak
  detections to be RANGE-CONSISTENT with each other + a pre-flight range-plausibility window
  (honest launch-geometry constants, same class as the collision-lead heading). De-risks the
  #1 TRANSFER RISK: a confident phantom STEERS the airframe (vs acquisition-fail which safely
  times out). Gate: phantom-seeded false-handoff → 0 without dropping real acquisitions.
  **UPDATE: the pre-flight range-plausibility window WAS built (`--coded-dash-acquire-range-min/max`,
  add #18f) and NULLed — it aborted BOTH directions; graveyarded with the ADR-0077 gate
  (docs/project_state.json). Do NOT re-build a range window. Only the streak range-CONSISTENCY
  variant is untested — and a phantom-only streak is self-consistent (~1.5 m), so it likely
  cannot meet this gate alone.**
- **#3 [SIM] ⛔ SUPERSEDED (add #18j-fix — crop is NOT a lever at the wall band; do NOT build):**
  Auto-crop under coded-dash (crop weights exist, `drone_finetuned_quad_crop.onnx`)
  — ~~ADR-0074 range lever~~; watch the phantom-seeds-the-crop failure (add #14, which #1 removes).
- **REAL-DRONE PLAN (staged):** A bench (calibrate RMS≤1px → AprilTag detect+motion-blur ramp
  → Hailo yolo11n compile+fps → hard-negatives from the real mount AFTER the prop-clearance
  geometry check — design props OUT of FOV = the $0 phantom root-fix). B target-drone data +
  retrain (init from COCO-pretrained NOT sim weights; ~15-20% negs per the rebal lesson;
  **validate on held-out FLIGHTS not random frames — the v3-NULL guardrail, ADR-0061**).
  C live intercepts: AprilTag seeker FIRST, YOLO in SHADOW MODE (log-only) to measure real
  bearing-σ/phantom-rate/R_acq at zero risk, THEN YOLO-guided.
- **RANGE ("interceptor distance") levers, ranked — ⛔ SUPERSEDED (2026-07-16, ADR-0076 add
  #18j-fix/#18k; see the ROUND-5 banner + docs/project_state.json graveyard): ① foveated-crop/
  resolution is NOT a lever (crop = full-frame recall at the 8-12 m wall band — 3rd mirage);
  ② real-data fine-tune is DEMOTED to an outdoor-appearance hypothesis; the corrected dominant
  lever is POINTING/adaptive tilt (#46/ADR-0065). Kept for history:** ① foveated auto-crop on native res
  (keeps wide FoV, ~1.5-2× R_acq, near-free — DO FIRST); ② real-data fine-tune (the real
  acquisition lever, unknowable until measured); ③ up-tilt (+2-3m, already bought). **GUARD
  RAIL: do NOT buy a narrow lens** — the ±30° coded-dash aim tolerance NEEDS the wide FoV
  (seeker_upgrades.md rec marked SUPERSEDED). Real first target flies STRAIGHT legs ≥2 m/s
  (the easy ADR-0038/0042 regime), not the 12 m/s weave the sim stress-tested.
- **KEEP (transfers):** pro-nav terminal `derotate_bearing_lambda`, PX4/MAVSDK offboard,
  the fine-tuned YOLO seeker, the up-tilt geometry. **CUT under coded-dash (sim-only):**
  CUE_WAIT/S2-cue-mock, `--fuse-midcourse`, handoff-cue-gate, coast-search.
- r2l ARC (below) REFRAMED: the "markerless-bearing limit" was measured THROUGH the cue/
  fused-dash machinery; under coded-dash r2l engages and the residual is perception aspect-
  bias (~2–4 m, the ADR-0056 weave-mirror residual), a refinement target not a null.

## 🗄 SUPERSEDED (2026-07-12 session — history; the live block is 📍 CURRENT (2026-07-16) above + docs/project_state.json)

### 🎯 SESSION 2026-07-12 — realistic-quad r2l: DIAGNOSED, REFRAMED as fixable, and a validated fix-direction (ADR-0076 add #13–#16)
Builder: "get straight-line then maneuvering intercept working." The quad's r2l 0% was a muddled,
half-abandoned overnight thread; this session turned it into a clear, positive engineering story.

**✅ POSITIVE / VALIDATED wins:**
- **ROOT CAUSE NAILED (was guesswork).** r2l fails because the forward camera sees the interceptor's
  OWN propeller blades as a "phantom" target (big false box reading ~1.5 m). The GUIDANCE is provably
  symmetric; the real r2l target IS detectable early (15–18 m) once the phantom is gone. So it was never
  a fundamental sensor/aspect limit.
- **⭐ FABLE REVIEW paid off (head-builds/Fable-reviews).** Builder asked Fable "how impossible is it?"
  — Fable correctly overturned an earlier "needs a better sensor" overclaim, caught a real gap (the
  tracker POSITION was unprotected, only velocity), and pointed at the two real fixes (appearance
  retrain / remove the phantom source). r2l is REFINABLE.
- **⭐ CAMERA-MOUNT FIX VALIDATES THE PHANTOM REMOVAL (builder's instinct).** Moving the seeker camera
  forward past the props (PX4 model swap, restore-after) puts the props OUT of the FOV: rendered frames
  confirm empty top corners + target still visible; v2 then detects ONLY the real target (25 px @15 m,
  correct range vs the old 118–386 px phantom); in flight `first_dash_detection` jumps **3.64 → 12–15 m
  in BOTH directions**. This is the first lever to get the real r2l target acquired early.
- **NEW TOOLING (honest, default-off, byte-identical):** `--cam-fwd-offset-m` camera lever-arm
  correction (transforms the camera-anchored LOS/range into the vehicle frame — own yaw + a static mount
  constant, no gt); `scripts/quad_seeker_arm.sh` reusable quad-seeker A/B arm; `scripts/experiments/
  cam_forward*` camera variants; balance-corrected retrain scaffold (`train_daemon_quad_rebal.py`).
- **DELIVERED CLARITY (negative results, cleanly closed → no more dead-ends to re-try):** negative-mining
  CLOSED on a clean fully-trained test (add #13); auto-crop preserves l2r but doesn't fix r2l (add #14);
  every spatial/downstream separator (range-gate, --track, --fuse, size/position filters, cue-vel-hold,
  tracker-cue-gate, image-mask) is null/regression because the phantom overlaps the real target in BOTH
  range and image space (add #15). This is why the phantom-SOURCE fix (camera-move) is the right path.

**🔧 CAMERA-MOVE + LEVER-ARM (v1, yaw-only) TESTED → INSUFFICIENT (0/16), fix-direction still open.**
Built `--cam-fwd-offset-m` (default-off lever-arm, applied to the pro-nav/tracker/fused/handoff-gate).
Full n=16 (x=0.40, `--cam-fwd-offset-m 0.40`, v2, seed 42, weave-mirror): **l2r 0/8 @3.23 m (still
regressed), r2l 0/8 @3.96 m, overall 0/16** (handoff rejects dropped 23→0, one 0.66 m flight, but the
fleet did NOT recover). The phantom IS removed (early detection both dirs) but the yaw-only lever-arm
leaves a residual parallax through the ~30° dash pitch + the 0.24 m up-offset, AND the NN was trained at
the STOCK viewpoint (box precision degrades at the moved view). PX4 stock camera RESTORED.

**▶ r2l ARC — CONCLUDED after ~12 levers (ADR-0076 add #17): a rigorously-characterized markerless-BEARING limit.**
- **THE PIVOT (data-verified):** phantom-removal is a DEAD-END — clean seed-123 control: v2 (phantom) l2r 8/8
  @0.72m / r2l 0/8 @4.17m; rebal (phantom GONE, precision kept — the balance-corrected retrain WORKED:
  phantom 4.41→0.00/frame, IoU 0.97 = v2) l2r 1/8 @2.93m / r2l 1/8 @3.83m → paired l2r **+2.14m WORSE**.
  The phantom was HELPING l2r (late handoff → tight cue-dash kill); r2l floors ~3.8-4.2m WITH or WITHOUT it.
- **ROOT CAUSE:** the r2l DASH under-closes — it commands −east (drives WEST, ~4m gap) while l2r commands +east
  and hits 0.72m. `--dash-lead-cue-vel` (fed the clean cue velocity to both lead paths, default-off, reverted)
  improved cmd_ve (−8→+4.4) but r2l STILL ~4m: the interceptor commands east yet own_E doesn't move because the
  **fused target-POSITION east is wrong from the aspect-biased camera BEARING (ADR-0056)** → the dash aim
  under-commits east. So r2l = a perception BEARING-quality floor (subpixel was NULL, ADR-0071; phantom-removal
  regresses l2r; no lead fix reaches it). This IS the perception gap the parent-project seeker exists to close.
- **HONEST PORTFOLIO POSITION:** ✅ billboard 72/72 BOTH dirs (ADR-0064) = the headline "intercept works".
  ✅ quad l2r ~88%. 📌 quad r2l = characterized markerless-bearing limit (strong negative-results arc). Deployed
  v2 + billboard fallback UNTOUCHED throughout.
- **Tools kept (all default-off, byte-identical):** `--cam-{fwd,left,up}-offset-m` 3-D camera lever-arm
  (math-verified, sim-untested); `drone_finetuned_quad_rebal.onnx` (phantom-free seeker); `quad_seeker_arm.sh`.
- **If pushed further (bigger builds, not done):** a genuinely better terminal BEARING on the r2l aspect
  (higher-res/native-1280 retrain, or a learned keypoint/subpixel-corner head) — the one untried class; OR
  accept + headline the negative-results arc + polish the billboard demo (T25).

**HONEST STATE for "intercept working":** ✅ billboard 72/72 BOTH dirs (ADR-0064) = headline DONE.
✅ quad l2r ~88%. 🔧 quad r2l = refinable, phantom-removal validated, terminal fix in progress. Deployed
v2 + billboard fallback UNTOUCHED. **[SUPERSEDED same-session + 07-15/16: phantom-removal concluded a
DEAD-END (add #17 above; the rebal retrain is an in-flight NULL — do NOT ship, see the 07-15 seeker
roadmap #1) and the "terminal fix" is CLOSED by add #18h (no camera-guided 3D-quad intercept in this
sim; the wall is flight-dynamic pointing, add #18k) — see 📍 CURRENT (2026-07-16) + docs/project_state.json.]**
- Harness: `scripts/quad_seeker_arm.sh WEIGHTS SEED PATH OUT --go` (`MARKERLESS_AUTOCROP=1` for crop;
  `QUAD_ARM_EXTRA="--cam-fwd-offset-m 0.40 ..."` appends m4 flags; camera variant swapped separately).

### 🌙 OVERNIGHT AUTONOMOUS RUN (2026-07-12 ~02:30 PDT → ≥08:00, builder asleep; priority: ACCURACY + CLOSE INTERCEPTION)
Executing the ADR-0076 fix-#3 build (raise the ~50% NN-coverage ceiling on the quad):
1. **AUTO-CROP build (ADR-0074)** — worker coding `render_sim_dataset_crop.py` (native-640-crop dataset)
   + a `MARKERLESS_AUTOCROP` inference mode (crop native-res around the last detection, full-frame
   fallback when stale; gt only for offline capture labels). Then: capture crop dataset → retrain on
   crops (imgsz 640, normal cost) → re-validate NN-only Pk. This is the real close-interception lever.
2. **Honest NN-only 640 baseline Pk** — n=32 batch RUNNING (`mc_quad_nnonly_*`) to nail where the quad
   really sits (~50%?) with a Wilson CI.
3. Bank cue (ADR-0073) feasibility gate + higher-n Pk on the winner, if time.
**KEY FINDINGS (overnight → r2l arc CLOSED with a quantified conclusion):** The realistic quad's "41% Pk (l2r 81% / r2l 0%)" (audit-confirmed baseline) is now fully diagnosed. A 5-agent workflow + confirmation A/Bs split the r2l 0% into TWO causes: **(a) an unfair NON-MIRROR WEAVE (test-harness bug)** — `m4_target_mover.py`'s fixed left-hand perp made the r2l target cross FAR (8.4 m) vs l2r NEAR (4.6 m); FIXED via the committed default-OFF `INTERCEPTOR_WEAVE_MIRROR` flag — worth ~3.4 m of the miss (r2l 7.43 → 3.99 m); **(b) a residual markerless-perception r2l ASPECT BIAS (ADR-0056)** — the genuinely hard ~4.0 m residual that keeps r2l at 0% even on the FAIR weave (and on `--path line`: r2l 0/8). The **GUIDANCE is provably mirror-symmetric** (pro-nav/world-frame/yaw/terminal all clean; ADR-0026 gain 5.000 both ways). So the weave fix is a CORRECTNESS/fairness win, NOT a Pk recovery. **Levers fully mapped (all ADR-0076 add #5-#12):** range-gate NULL, `--track` retracted (geometry artifact), `--fuse` NULL, negative-mining regressed-but-proved-phantom≠cause, weave-mirror fixes the harness half. **REMAINING r2l lever = a PERCEPTION fix** (subpixel/centroid bearing — `docs/subpixel_bearing_prereg.md`, or the ADR-0056 aspect-bias root cause). **DEPLOYED CONFIG UNTOUCHED** (v2 seeker, l2r 88%). Builder calls: adopt `INTERCEPTOR_WEAVE_MIRROR=1` as the fair default + re-run key numbers? pursue the r2l perception fix? Also: fix `mc_batch.sh:532-535` coverage regex (add #9 Risk 4). Separately: `docs/hardware_order_list.md` = the parent-project BOM (delivered).
Config facts: quad = `fpv_quad_enemy` + `drone_finetuned_quad_v2` seeker, NN-only (drop `--track`),
orient ON offset 0, GPU render (ADR-0075). Standard mc_batch geometry. Resume: `docs/quad_target_retrain.md`.
NO swap (billboard baseline stands until the quad earns a Pk).

### 🎯 BUILDER CHALLENGES (2026-07-11) — 3 questions ADJUDICATED (ADR-0072, GO-WITH-CHANGES) → EXECUTING the target-realism fix
Builder pushback on the demo/video: "we got many sub-1 m intercepts on AprilTag, where's
the perfect-setup floor claim from? there has to be a better tracking system. The enemy
drone is a static, not-even-correctly-oriented model that doesn't resemble a maneuvering
drone. Can we pivot to a different sim?" A 5-agent research workflow (`wf_48003243-ca1`)
settled all three; adversary verdict **GO-WITH-CHANGES**:
1. **SIM PIVOT → NO. Stay on Gazebo.** Both complaints are algorithm/modeling issues
   IDENTICAL in any simulator, not Gazebo ceilings. A pivot invalidates ~70 Gazebo-measured
   ADRs + Pk 72/72 + the M3/M4/detection curves (non-transferable), forces a full YOLO
   retrain on new pixels, and needs a ≥16 GB GPU + native Linux this 4070/12 GB WSL2 box
   lacks (Isaac/Pegasus). Photoreal + banking are reachable far cheaper IN Gazebo. Revisit
   only if photoreal domain-randomization becomes central AND the builder moves to native
   Linux + a bigger GPU. **(My 1.64 m "floor" claim was CORRECTED: that is the 12 m/s WEAVE
   only; sub-meter is real at slower/straighter regimes — M3 0.02 m, M4 pursuit 0.945 m.)**
2. **TARGET REALISM → FIX IN PLACE. (A) orient-to-velocity = BUILT + COMMITTED (e4bd6ba,
   3eebab8).** The mover streamed POSITION ONLY (ADR-0010 #6) → the target crabbed sideways,
   frozen nose, dead-level. New `--orient-to-velocity` (env `INTERCEPTOR_ORIENT_TO_VEL`):
   yaw-to-travel + coordinated-turn BANK into the turn + nose-down PITCH, composed with the
   model's baked +Y-forward offset (`--orient-yaw-offset-deg -90`); signs verified vs the
   rotated body axes; 8 unit tests. **Default-OFF byte-identical (send_pose untouched when
   quat=None) + REFUSED for apriltag_target** → every measured MC batch unconfounded (mc_batch
   never passes the flag). **(B) reskin = PENDING** (vendored mesh demo model, gated on a
   seeker-detection check — the NN was trained on the primitive silhouette).
   **⚠️ CORRECTED (ADR-0072 add. #2, builder 2026-07-11): the orient MATH is fine but the
   MODEL is a flat BILLBOARD** (arms/props in the local Y-Z VERTICAL plane, a tag-panel
   heritage), so orienting it just tilts a vertical wall — NOT a real banking quad (my "chase
   frames validated banking" read was a misread; corrected). The `velocity_orientation()` unit
   tests stand; the defect is model geometry. **NEW ARC (builder-directed, retrain accepted):**
   1. ✅ **Proper 3D quad target BUILT + VERIFIED** — `models/fpv_quad_enemy/` (real x500 mesh,
      red enemy props, HORIZONTAL rotor plane, nose +X; meshes vendored ~25 MB = git-LFS
      candidate; `53b0400`). Chase render CONFIRMS it banks like a real drone (mover
      `--orient-to-velocity --orient-yaw-offset-deg 0`, 3184 poses, 0 fail). `worlds/quad_enemy_verify.sdf`.
   2. ⏳ **RETRAIN the seeker on the new appearance** (worker in flight) — in-domain gt-labeled
      dataset of `fpv_quad_enemy` (`render_sim_dataset.py`, level+yaw grid, wide yaws) →
      `train_daemon_quad.py` (yolo11n, imgsz 640, 60 ep, setsid-detached) →
      `weights/drone_finetuned_quad.{pt,onnx}`. Deployed `v2` + `fpv_target_markerless` untouched.
      First retrain is level+yaw; if banked detection is weak, add banking poses.
   3. ⏳ **RE-VALIDATED (first pass) — retrained seeker WORKS on the new quad, needs banking data.**
      `drone_finetuned_quad` (best.pt @ ~epoch 16, val mAP50 0.995 — but leaky random split so the
      real gate is the intercept): live intercept on `quad_enemy` world **first-det 23.3 m** (better
      than v2's 21.6 m), **intercepted 1.18 m clean** — BUT **handoff late at 1.91 m** (v2 was 9 m)
      with 26 tracker losses: detection is INTERMITTENT on the BANKED aspects (level+yaw dataset never
      showed them). Core works; robustness needs banking poses.
   4. ✅ **BANKING-POSE RETRAIN — TRAINED + VALIDATED (6/6 clean).** `drone_finetuned_quad_v2`
      (level ∪ 1344 banked = 2664 imgs; stopped ~ep 17 saturated, exported). **Characterization batch
      6/6 CLEAN, median miss ~1.2 m, median handoff ~6.8 m** — the banked data FIXED the level seeker's
      stuck 1.91 m handoff + intermittent banked detection (26→4 losses). The one earlier terminal abort
      was run-to-run noise (same seed re-flew clean). One high miss (3.09 m) was a late-acquisition tail
      (ADR-0038), not a banking failure. Scripts committed `9a8fd56`; weights gitignored.
      **⚠️ CORRECTED — the "6/6 clean" was EASY-GEOMETRY (target 30 m out). The real Pk batch on the
      STANDARD geometry (14 m, both dirs — the one the billboard aced 72/72) = POOR (~2/45 Pk@2.5)
      (ADR-0076).** GPU render is NOT the cause (paired CPU/GPU equivalent, ADR-0075 vindicated). The
      billboard FLATTERED detection; the realistic quad is detected intermittently → late handoff. Root
      cause: **the CSRT tracker loses the banking quad ~25×/flight** (NN coverage ~50%, reacq_rejected~0)
      — partly fixable. **NO SWAP** (quad Pk not validated); fallback (`fpv_target_markerless`+`v2`) live.
      **⭐ RESUME GUIDE: `docs/quad_target_retrain.md`.**
   4b. 🔧 **DETECTION-FIX PROGRAM (builder "implement all 3", ADR-0076 add #2/#3) — OUTCOME:**
      **#1 tracker levers ✅ DONE — NN-only WINS** (Pk@2.5 4/8 vs CSRT 1/8; drop `--track` for the
      maneuvering quad; more re-validation didn't help). **#2 appearance-robust tracker ✅ DONE — VIT
      integrated (`MARKERLESS_TRACK_KIND=VIT`, committed `b89fe1e`) but does NOT help (0/8)** — every
      tracker slips the banking lock; shelved as an option. **#3 coverage lift ⏳ CHARACTERIZED, build
      pending** — the ceiling is NN detection COVERAGE ~50%; a cheap 1280 re-export helped handoff
      (1.6→3.9 m) but not Pk (scale mismatch) → **resolution needs a RETRAIN, not a re-export.** The real
      fix-3 build = **AUTO-CROP** (native-crop dataset + normal-cost 640 retrain on crops, ADR-0074) +
      the **bank cue** (ADR-0073) on top. **Honest net: quad Pk on the standard geometry ~50% vs the
      billboard's flattering 72/72; NO swap; billboard baseline stands until a coverage retrain earns it.**
      `drone_finetuned_quad_v2_1280.onnx` (1280 re-export) kept for reference.
   The orient plumbing + the billboard finding (add. #2) are why. Deployed v2 + fpv_target_markerless
   still untouched until the quad seeker is robust + Pk-validated (swap is Pk-gated, not done).
   5. 💡 **NEW IDEA — BANK-AS-ACCEL CUE for maneuver-predictive guidance (ADR-0073, builder 2026-07-11).**
      The banking quad now EXPOSES its lateral accel: `tan(bank)=a_lat/g`, a LEADING indicator read
      WITHOUT differentiation → **bypasses the noise wall that killed ADR-0069's accel-prediction**.
      Small bank-regression head, runs on the Pi+Hailo, honesty-legal. Payoff small on the mild weave
      (~0.27 m maneuver term) but SCALES with maneuver aggressiveness. **Feasibility gate (design-time,
      no new sim): train a bank head on `quad_dataset_banked` (known banks), measure bank-error vs range,
      check the accel-SNR clears the ADR-0069 wall — DO THIS after the v2 re-validation.** Gate passes →
      council the guidance mechanization → lab APN A/B → Gazebo.
   6. 💡 **NEW IDEA — FOVEATED / AUTO-CROP detection + higher-res (ADR-0074, builder 2026-07-11).**
      Detect on a downscaled whole frame → crop the FULL-RES ROI → re-run the SAME detector on just
      the drone (compute ~2× a cheap pass, not a full-sensor high-res pass). **FREE WIN: we already
      downscale 1280→640, so an auto-crop on the NATIVE crop doubles pixels-on-target with NO new
      camera.** Attacks the v2 late-handoff/intermittent-banked-detection (ADR-0038 density) + is a
      near-prerequisite for the bank cue (ADR-0073, needs pixels). NOT the mild-weave miss (kinematic,
      like ADR-0071 subpixel null). **Cheapest first step: native-res auto-crop stage** → test steadier
      handoff. Honesty (ADR-0025): a real higher-res sensor must be DISCLOSED + M1/M2 re-earned + σ_R
      rescaled (not tag-inflation). Sim: hi-res camera costs render (GPU-render switch). Pairs with #5.
3. **BETTER TRACKING → the learned single-keypoint head is the CORRECT design but the WRONG
   lever here → OPTIONAL, pre-registered.** It refines bearing PRECISION, which is NEITHER
   the weave kinematic floor (ADR-0056: even a clean AprilTag = 1.64 m) NOR the slower-regime
   ACQUISITION-DENSITY gap (ADR-0038: missing/late detections, not imprecise ones). Likely a
   5th null. If built: pre-register PAIRED n≥8 vs a RECALL / tail-Pk metric with a kill-criterion.
   Otherwise **headline the existing 4-lever negative-results arc** (the more mature story).
- **NEXT (this thread) — follow `docs/quad_target_retrain.md`:** finish banked capture → merge
  (`merge_quad_v2.py`) → launch `train_daemon_quad_v2.py` (setsid) → export → re-validate the
  intercept (expect an earlier handoff than the level seeker's 1.91 m) → Pk batch → Pk-gated SWAP.
  Then GPU render → re-capture the T25 intercept clip with the banking quad target → assemble.
  Keypoint head deferred behind the pre-registration gate. **Commit the 3 uncommitted retrain
  scripts** (`render_sim_dataset_banked.py`, `merge_quad_v2.py`, `train_daemon_quad_v2.py`).

### 🎯 PRIOR DIRECTIVES (2026-07-10 evening) — tighter-miss lane CLOSED (context for the above)
1. **TIGHTER INTERCEPT — "2 m is too much." → ANSWERED (ADR-0069): accel-prediction
   is the WRONG lever; ACQUISITION RANGE (= adaptive tilt #46) is the right one.**
   A 5-agent research/design workflow + a kill-gate probe settled it: (a) the code
   uses ZERO target acceleration (plain pro-nav, LOS-rate nulling); (b) the ~2 m
   weave miss is ~84% a fast-crossing KINEMATIC FLOOR — a zero-accel straight 12 m/s
   crosser already misses ~1.6 m; only ~0.27 m avg is the maneuver term; the 0.35 s
   terminal is capacity-bound (needs ~33 m/s² vs ~8-13). (c) The kill-gate probe
   (`scripts/experiments/dash_accel_lead_probe.py`) proves NO estimator tuning gets a
   clean+in-phase accel from the σ=0.5 m/s cue (SNR<1, noise/lag wall) — so even the
   one legal experiment (accel-augmented DASH lead on the clean cue) is a pre-known
   NEGATIVE, NO sim spend. The terminal APN rejection (ADR-0010/0011) still holds
   (AprilTag clean-bearing control already sits 8/8 @ 1.64 m with plain pro-nav →
   lead/accel is not the missing lever).
   **UPDATE (ADR-0070, FLOWN): the acquisition-range lever ALSO does NOT tighten the
   miss.** A 3-arm A/B doubled t_go (handoff 6.5→13.5 m) with CPA UNCHANGED (paired
   median +0.02 m, 4/8 each way), and the fixed tilt made it WORSE (8/8 seeds,
   +0.48 m). The ~1.5 m miss is TERMINAL-BEARING-NOISE-floored (ADR-0056: a clean
   AprilTag seeker already sits 1.64 m), NOT time/capacity-limited. **So NO
   guidance/acquisition lever tightens it: not accel-prediction, not bigger t_go,
   not tilt (fixed OR adaptive — same bearing floor).** The only real levers for
   < 1.5 m: **(a) TERMINAL BEARING QUALITY** — subpixel/centroid vs the NN
   box-center (the ADR-0056 residual, a hard perception refinement); **(b) the
   honest METRIC REFRAME** — 1.5–2 m is already Pk@2.5 8/8 = a proximity-fuse KILL
   (ADR-0025).
   **BUILDER CHOSE (a) — SUBPIXEL BEARING (ADR-0071) → TESTED to n=31 → NULL (the
   n=8 "win" was small-sample NOISE).** The darkness-weighted centroid is real +
   honesty-clean, but the paired mean CPA REGRESSED as n grew (n=15 −0.16 → n=23
   −0.075 → **n=31 +0.06 m**, sign 16 worse/11 better). Marginal tail survivor (90th
   pctl 2.15 vs 2.40, Pk 30/31 vs 28/31) is within noise. Likely the α-β lambda
   filter already smooths the box-center jitter. **Kept DEFAULT-OFF (validated, no
   robust benefit).**
   **★ TIGHTER-MISS LANE CLOSED (ADR-0071 add. #2).** EVERY lever eliminated: accel
   (ADR-0069), t_go/handoff (ADR-0070), tilt (ADR-0070 worse), subpixel (null). The
   ~2 m weave CPA is the practical SEEKER FLOOR (kinematic ADR-0023 + bearing-quality
   ADR-0056) AND already a **Pk@2.5 KILL** (72/72, ADR-0064) — a proximity-fuse
   interceptor detonates on target at 2 m (ADR-0025), so "tighter CPA" is the wrong
   objective; the intercept already SUCCEEDS. A genuinely tighter miss needs a
   fundamentally different terminal seeker (subpixel-corner fiducial / higher-Hz
   clean bearing), not a guidance tweak. **Net: a rigorous 4-lever honest
   negative-results arc → the correct root cause. The video's current ~2 m intercept
   IS the honest best.**
2. **TILT for a tighter intercept — PIVOT to FIXED tilt first (builder 2026-07-10 eve).**
   Builder's call: try a FIXED up-tilt before the complex adaptive gimbal — the dash
   is "always at speed forward" (consistent nose-down pitch, ADR-0060) and the stereo
   cue guides the dash (no search needed), so a fixed tilt may be "about as good" for
   acquisition at far lower cost (no gimbal/controller/PX4-swap). **KEY INSIGHT: neither
   fixed nor adaptive tilt has been tested for TIGHTENING the miss yet.** #40/ADR-0067
   tested terminal PARITY (does the tilt make it WORSE — no, Pk stays 8/8 compensated);
   it did NOT test whether the earlier detection the tilt buys converts to an EARLIER
   HANDOFF at longer range → bigger t_go → tighter miss (the ADR-0069 lever). That
   handoff-range extension is the piece that actually tightens the miss, and it works
   with the SIMPLE fixed tilt (`--cam-mount-up-deg`, in-project #40 shadow, NO PX4 swap).
   - **NEXT EXPERIMENT (cheap): fixed up15 (compensated) + EXTENDED --handoff-range vs
     level control**, weave 12, paired n≥8. Metric: does it hand off at longer range →
     tighter CPA? Pre-registered `docs/adaptive_tilt_prereg.md`.
   - **Adaptive gimbal #46 = the FALLBACK (BUILT, committed 8657320 + adaptive_tilt_arm.sh):**
     model sim-validated, m4 own-state controller + live derotation, default-off
     byte-identical. Use ONLY if the fixed tilt leaves too much on the table (pitch
     variation / terminal residual). Builder approved the reversible PX4 model-swap.
   - (superseded adaptive-first build plan below, kept for the fallback):

   **ADAPTIVE TILT #46 (ADR-0065) — FALLBACK build plan.** CONCRETE BUILD PLAN
   (scoped 2026-07-10; PX4/Gazebo has NATIVE gimbal support — no gimbal from scratch):
   - **Model:** a `mono_cam_gimbal` variant of the seeker camera — keep `camera_link`
     as the airframe mount (x500_mono_cam's fixed `CameraJoint` attaches it, UNCHANGED),
     add a REVOLUTE PITCH joint `camera_link → camera_pitch_link` that holds the
     `imager` sensor with the SAME 1280×960 / hfov 1.74 / topic / intrinsics (NN
     pipeline untouched), + a `gz-sim-joint-position-controller-system` on the pitch
     joint (sub-topic `command/gimbal_pitch`, PID like the stock gimbal p=0.8). Shadow
     it via `models/mono_cam` (the #40 symlink mechanism) — REUSE the uptilt shadow
     guards (mc_deployment_arm refuses a stray shadow unless UPTILT_EXPECTED=1).
   - **Controller (honesty-clean, own-state ONLY):** a small gz-transport publisher
     (like s2_cue_mock) reads the vehicle pitch (own-state EKF attitude) and commands
     `gimbal_pitch = clamp(-vehicle_pitch + lead, limits)` to hold the boresight at the
     horizon through the nose-down dash. NO gt_*; NO camera-dependence for v1 (pure
     pitch-stabilization); optional v2 adds a small camera/cue-bearing elevation lead.
   - **Guidance:** extend the #40 `--cam-mount-up-deg` (static) to a TIME-VARYING live
     gimbal angle in `derotate_bearing_lambda` — the commanded pitch each tick (m4
     owns/knows it). Reuse the round-trip oracle test pattern (byte-identical at 0).
   - **Validate:** paired A/B vs level + vs the fixed +15° — does earlier acquisition
     (first-det range, coverage, FoV margin) → a TIGHTER terminal miss (the ADR-0069
     mechanism: bigger t_go / smaller delivered ZEM)? Pre-register the bar. Honest
     prior: availability WILL improve (the #40 mechanism); whether it converts to a
     sub-2 m miss is the open question (handoff-range/t_go must actually extend).
   - Then FEATURE it in the video (replaces the fixed-tilt shot 3b per directive #3).
3. **VIDEO fixes:** (a) MORE footage of the intercept itself; (b) the slow-mo
   comes TOO LATE — it starts after the interceptor has already PASSED the drone;
   retime so slow-mo covers the APPROACH-TO-CPA, not the flyby; (c) REMOVE the
   fixed-15° two-camera A/B (shot 3b) — show the ADAPTIVE TILT system instead.
4. **GitHub remote — SKIP for now** (builder, 2026-07-10 — was pending, now
   deferred; the zero-off-machine-backup risk still stands, re-surface later).
5. **Keep persistent docs updated with directives + work as it lands** (builder:
   survive a session clear). Capture SAME-TURN. See [[context-loss-is-the-1-frustration]].

---

### (prior autonomous block — 2026-07-10, builder now present)

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
   - **[SUPERSEDED — no longer the next sim item: the 2026-07-15 real-build pivot CUT the
     coast-search/cue stack this arm would fly (see the PIVOT "CUT under coded-dash" list above +
     docs/project_state.json); the current next probe is the ROUND-5 frame-top off-axis sweep]
     Stage-2 tilted+compensated RECOVERY re-test** — the
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
   scheduling would avoid); ~~candidate to un-HOLD ADR-0059 recovery~~
   **[SUPERSEDED: will NOT un-HOLD ADR-0059 — the #40 Stage-2 NULL proved the recovery limit is
   far-range NN detection, not FoV pointing (see the #46 revision in the prior-autonomous block);
   #46's live rationale is now the ADR-0076 add #18k flight-dynamic pointing wall (nominal
   acquisition), and the no-datalink real-build pivot moots the ADR-0059 recovery question.]**
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
