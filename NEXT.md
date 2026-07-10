# NEXT — top of the stack

*(One CURRENT section, one ordered build queue, one compressed Done list. Detail
lives in `docs/decisions.md` (ADRs) and `PROGRESS.md` (roll-up). Restructured
2026-07-06 during the Fable audit; rebuilt 2026-07-08 after the seeker-v2 +
fusion-P0/P1/P2 session.)*

## ⏩ LIVE (2026-07-10 continued — autonomous window, builder away) — READ FIRST

**Shipped this session (visible commits):**
- `706bb46` — **ADR-0062 guidance-correctness fixes #37/#38.** FIX-A: full-attitude
  LOS-bearing derotation (own-state quaternion, optical→body→NED) — A/B-VALIDATED
  help-or-neutral at dash tilt (paired weave n=8: 7/8 seeds tighter, 0 worse,
  −0.161 m median, both 8/8 Pk; identity-preserving at level so M3/M4/M5 bit-safe).
  FIX-B: past-CPA range-increase breakoff noise deadband — ships INERT (default
  byte-identical), tuning gated separately. Suite 102 pass / 2 skip.
- `149c9a8` — MC harnesses (deployment A/B arm + adopted-config jink n=16).
- `46026d4` — **jam_fixon_coast recovery arm** added to `mc_jam_arm.sh` (#39).

**SIM QUEUE (one at a time, idle load — event-driven as each frees the sim):**
1. **RUNNING:** jink n=16 adopted config (`mc_t21_trackgate_jink16.sh --go`,
   watcher `bx04qugf5`) — the #33 top stats item, closes the jink-on-post-fix gap
   (r2 headline was weave-only) and captures FIX-A on jink.
2. **NEXT:** `scripts/mc_jam_arm.sh jam_fixon_coast` at `CUTOFF_RANGE_M=18` then
   `=22` (#39 comms-denied RECOVERY) — compare REAL-ish recovery vs the ADR-0059
   fixon (no-coast) arms; if recovery jumps, un-HOLD "works comms-denied" as a
   scoped recovery claim + write the ADR. Risk: yaw-only sweep vs ADR-0060 top-of-
   frame pitch.
3. up-tilt mount A/B (#35) — NOTE: confounded by FIX-A's zero-mount-tilt assumption;
   needs the #40 mount-compose first. r_hat honesty (#32). stats n=48/72 (#33 opt A).

**Parallel (sim-free):** audit-findings tracker (#36, Sonnet worker) in flight.
**#40 FIX-A follow-ups** (uptilt mount-compose, gate derotation, M7 slant-range,
attitude CSV cols, FIX-B tuning gate) — all touch m4, so BETWEEN sim batches only.

## 🌙 OVERNIGHT PLAN (2026-07-10 night — builder asleep, "run multiple Fable audits, use the pure window")

**RUNNING (all Fable, sim-free, budget-spend):** deep codebase audit workflow (correctness/honesty/repro/tests/portfolio-redteam → verify → morning report); forward audit workflow (v3-eval + jam-MC methodology soundness / design-review missed gaps / claim completeness / next-value); flight-compute council (3 members → #34 ADR); pre-draft worker (jink n=16, stats n=72-vs-reframe, up-tilt mount SDF variants — so the sim queue fires instantly).

**SIM CRITICAL PATH (event-driven):** v3 fine-tune (~30 min left, setsid daemon) → V3_TRAIN_EXPORT_DONE → poke v3 agent → Phase-4 eval → v3 verdict (expect NULL per audit, accept honestly) → **sim frees** → jam MC (jam_fixon + control_active FIRST → un-HOLD comms-denied → commit jam fix) → jink n=16 post-fix → then T25 render / stats overnight.

**CADENCE:** the ~42-min self-terminating watcher is the heartbeat (re-arm each cycle — reap-proof). Each audit/agent completion re-invokes main → act on findings (sim-free fixes first), launch the next audit, drive the sim queue. Morning = consolidated report: audit findings + fixes, v3 verdict, jam-validation status, council recommendation, decisions needed.

**Committed this session:** 309c24e (design review + v3 tooling + probes), 1a14869 (portfolio surface). HELD for validated-commit: jam code (m4/detect_track/mc_jam_arm.sh/test_cue_staleness) + decisions.md (ADR-0059/0060).

## 🌅 OVERNIGHT OUTCOME (2026-07-10) — READ `docs/overnight_report_2026-07-10.md` FIRST

Big autonomous night. **Honesty spine held across 3 deep audits.** Flagship results: v3 retrain = honest NULL (forward audit CAUGHT a false-win before it scored; markerless still works via detect-then-track/v2); comms-denied jam MC = fail-closed DEMONSTRATED (monotonic witness collapse REAL-ish 12→2→0 across 15/18/22 m cutoffs) + the ADR-0059 fix VALIDATED-AS-FAIL-SAFE (anti-fooled, phantom eliminated) but NOT recovery → "works comms-denied" STAYS HELD; full recovery = fix+coast-search (#39). **6 commits** (309c24e/1a14869/7dad15e/2665bbb/f6427af/81562b8). **Option B ratified** (markerless flies). **DECISIONS NEEDED FROM EMERSON:** (a) commit the held jam-fix workstream as a fail-safe default-off option OR hold for fix+coast-search? (b) order the ~$257 Stage-0 cart + Hailo (#34). Open findings → tasks #37 (breakoff), #38 (roll/pitch derotation), #39 (coast-search recovery). Full detail in the morning report.

## ✅ CLOSED — jam fail-closed (ADR-0059): finding + fix built + validated as fail-safe (jam MC done 2026-07-10; recovery = #39)

The design review (task #29, `docs/design_review_sim_to_real_2026-07-10.md`) + an independent code-trace found that the **adopted deployment config `--track --handoff-cue-gate 8` FAILS CLOSED under a mid-dash cue jam**: `last_cue_pos` never ages out, so a frozen stale cue makes the handoff-gate AND the `_seed_ok` seed-gate reject the real target once it drifts >8 m from the frozen point (~0.67 s after the jam) → never hands off. Default config (no cue-gate) hands off fine under jam but eats phantoms — a genuine tension. Mid-dash jam is an IN-SCOPE WORST tier (ADR-0015 `--link-cutoff`), so this is a real regression, but LATENT (no ADR-0058 arm ever flew a jam). **Portfolio honesty: HOLD "works comms-denied" until fixed.** Fix design + plan in ADR-0059 → task #31. **UPDATE 2026-07-10: fix now BUILT + double-reviewed (v3 is TRAINING, not flying m4, so m4 was safe to edit); MC validation pending at idle post-train — see current-state block below.**

## 📍 CURRENT STATE + UNCOMMITTED WORK (2026-07-10, live session — READ THIS FIRST; the older resume block below is detect-then-track lineage, now committed as d10c9ca)

**Two big threads converging:**
- **v3 seeker retrain (task #28):** fine-tune LIVE (~epoch 2/60, ETA ~4–5 h), dataset triple-checked clean (build check + verify_split_v3 + Ultralytics 0-corrupt), Phase-4 eval fully pre-registered (mis-lock metric = positive-frame wrong-lock, recall-led verdict, NULL branch). Sim busy (CPU). On completion → v2-vs-v3 honest verdict + ADR-0061 closing #28. Watcher `b4s8idaru`.
- **Design review (task #29, DONE):** `docs/design_review_sim_to_real_2026-07-10.md` delivered. Actions tracked #30–35.

**Design-review findings tackled this session:**
- **Blur (G1, #30a):** sample DONE — detection survives the terminal-blur design band, bearing <0.25°; SCOPED (sim renders, not real optics — bench closes it). `scripts/experiments/blur_replay.py`.
- **Camera-pitch (var #19, ADR-0060):** MEASURED from ulog — dash pitches nose-down 27–36° (max ~52°), throws the horizon target to the TOP of frame; ~35% of dash ticks target above FoV; handoff threads a 0–6° sliver, ~zero vertical margin (real HW worse). Remedy = up-tilted camera mount (task #35). `scripts/experiments/dash_pitch_probe.py`.
- **Jam fail-closed (ADR-0059, #31):** FIX BUILT + double-reviewed (Opus build → Fable review caught a post-handoff false-STALE defect + overruled my horizon lean → fixes) → 86 tests green, honesty latch intact, horizon 1.0 s. PENDING the 4-arm MC validation at idle post-train.

**UNCOMMITTED WORKING TREE (commit at validated milestones; stage SPECIFIC paths, never -A):**
- Jam fix: `m4_intercept.py`, `scripts/seeker/detect_track.py` (comment-only), `tests/test_cue_staleness.py` (new), `scripts/mc_jam_arm.sh` (new), ADR-0059 in `docs/decisions.md` → commit AFTER `jam_fixon` MC arm validates.
- Design review + characterization: `docs/design_review_sim_to_real_2026-07-10.md`, ADR-0060 in `docs/decisions.md`, `scripts/experiments/{blur_replay,dash_pitch_probe}.py`, `NEXT.md`.
- v3 will add ADR-0061 + artifacts after eval. **NEVER stage: `yolo11n.pt`, `.claude/settings.local.json`.**

**SIM-GATED QUEUE for post-train idle (ONE sim at a time, idle load):** (1) jam MC 4 arms [#31, highest — validates the ADR-0059 fix], (2) up-tilt mount A/B [#35], (3) r_hat honesty [#32], (4) stats hardening n=48 [#33], (5) full blur sweep [#30a]. + BUILDER: Stage-0 cart [#34]. T25 video after v3 eval.

## ⚡ SESSION RESUME BLOCK (written 2026-07-09 ~22:45Z — SUPERSEDED by the current-state block above; kept for detect-then-track lineage only)

**Where we are: detect-then-track (task #27) DONE and COMMITTED (post-fix, micro-review green-lit, headline arm flown). ADR-0058 final. Next up: v3 dataset (#28), then T25.**

**State on disk (all uncommitted, working tree is the source of truth):**
- Feature + review fixes: `scripts/seeker/detect_track.py` (NEW: DetectThenTracker + SeekerSeedContext + legal_cue_pos() runtime honesty belt), `scripts/seeker/{markerless_loop,two_stage_seeker,finetuned_seeker,nn_seeker}.py`, `scripts/m4_intercept.py` (`--track`, requires `--handoff`; seed_ctx per tick; meas_source CSV column), `scripts/m3_static_intercept.py` (check what changed — likely Measurement-field seam; verify byte-safe), `tests/test_honesty_static.py` (70 pass 2 skip = 66 baseline + 4 new honesty tripwires), `scripts/analyze_track_ab.py` (three-way attribution classifier). `yolo11n.pt` + `.claude/settings.local.json` are UNRELATED — never stage them.
- ADR-0058 **WRITTEN to `docs/decisions.md`** (line ~1345; the prior "draft staged" claim was stale — it was never in the file, confirmed grep 0, now written fresh with the two-review verdict, the two defect fixes, and full three-way attribution for all four arms). Update it with the headline-arm (trackgate_weave12_r2 — r1 invalidated by a launch-args error, see the ADR provenance note) numbers once that batch lands.
- v3 dataset plan (task #28, execution-ready): `docs/seeker_v3_dataset_plan.md` + `scripts/seeker_v3_capture.py` (self-test 8/8, dry-run 648/648 verified).

**Results so far (corrected classifier, three-way attribution; details in ADR-0058 draft):**
- CAMERA-TERMINAL Pk@2.5 (post-handoff min gt — the honest metric): weave **10/15 med 2.28 m** (plain 3/16 med 13.3), jink **14/15 med 1.97, REAL-conditioned 13/13** (plain n=8 1/8 med 7.79). AprilTag ceiling 8/8 med 1.64. Phantom handoffs 12/16→2/16 (weave). Pooled Pk 15/16 both arms (flattering — includes legal cue-banked hits; always report all three levels).
- **★ HEADLINE ARM (r2, post-fix, `--track`+`--handoff-cue-gate 8`, weave n=16): CLEAN SWEEP — pooled 16/16 (med 2.11), camera-terminal 14/14 (med 2.03, max 2.48), ZERO phantom handoffs, 0/155 FALSE terminal dets, reacq_rejected=0. Full numbers + the two never-handoff characterizations in ADR-0058. Deployment config ADOPTED; builder's maneuver-fix directive SATISFIED.**

**In flight at session cutoff (CHECK FIRST — batch processes may have died with the session):**
1. ~~plain_jink~~ **DONE before cutoff: n=16, pooled Pk@2.5 3/16, med 3.58, max 5.67** (`logs/mc_t21_plain_jink12_n16.csv`) vs track_jink 15/16 / 1.98. Paired baseline COMPLETE — analyzed three-way in ADR-0058.
2. ~~Fable re-review~~ **DELIVERED + BOTH DEFECTS FIXED (RTF-invariant frame-count coast clock; REACQ cap 12 m) + minors folded (main-thread kind validation, thread-liveness log, tightened latch pin). Verdict `docs/review_track_fix_delta_20260709.md`; micro-review of the fix delta green-lit the commit.**

**Remaining queue (in order):**
1. ~~plain_jink verdict → GPU health → cooldown~~ **DONE** (GPU clean, dxgk flat/benign through both r-arms).
2. ~~HEADLINE ARM~~ **DONE (r2 — r1 was invalidated by a launch-args error, 16 argparse-rejected boots, no data; see ADR-0058 provenance note). Results above.**
3. ~~Full analysis → finalize ADR-0058 → fold review verdict → full test suite → commit SPECIFIC paths~~ **DONE — suite 71 passed / 2 skipped; committed (feature + tests + docs; yolo11n.pt/settings/v3-files excluded).** Optional residue: verifier gate re-run if main wants an independent pass.
4. **RUNNING NOW: task #28** — v3 capture (~2.5–3 h sim, plan §Phase-0 forensics first) → CPU fine-tune (~6 h, .venv-seeker-train) → frame-level v2-vs-v3 gates G1–G5 → re-A/B detect-then-track on v3. Owns the sim.
5. **RUNNING NOW (parallel, sim-free): task #29** — design review + sim-to-real interfering-variable inventory + cost-effective real-world implementation path (builder 2026-07-09). Ultracode workflow wf_a01f1020-19b (scout→analyze→adversarial-verify→synthesize). On completion: write the doc, present, log ADR, fold top actions into this queue. Feeds parked T23 gap audit + deployment M-1..M-4 + Stage-0.
6. **T25 demo video — SEQUENCED AFTER v3 EVALUATION (builder 2026-07-09: "after eval completes please").** The maneuver-fix deferral is now MET (ADR-0058, 14/14 camera-only maneuvering terminal); builder wants the demo to fly the BEST detector, so hold T25 until #28's G1–G5 eval finishes, then confirm-and-render.

**Operating pattern (builder-mandated):** Fable head; Opus subagents build; FABLE subagents review every correctness-critical diff before commit ("Opus misses obvious changes"). Sonnet only for rote volume. Never end session until builder says. Batches: sequential, sim-time, watcher via log-sentinel grep only (never pkill/pgrep with sim-name substrings inline — self-kill). Subagent auto-resume on batch exit is UNRELIABLE — always arm an independent sentinel watcher.

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
5. **Fusion refinement** (T20) — **DESIGN RATIFIED (ADR-0054, judge-panel of 3):
   refine the WINNER (hand-set FusedTrack, 8/8 WORST), NOT the regressed EKF —
   a range-agreement monitor (scalar along-LOS de-bias vs a shadow camera-only
   track, camera-lock-streak-gated) + camera-favoring confidence rolloff.**
   `--fuse-agreement-monitor` default OFF, byte-identical when off. Execution
   pending: STEP 1 = offline replay screens (scripts/agreement_monitor_replay.py,
   4 pre-registered screens) → guidance_lab rank → mock Gazebo ladder → adopt
   ONLY on real T17-v2 data. **⚑ Pre-registered NULL: if T17-v2 brings the real
   cue bias within the 0.5 m budget the hand-set already tolerates, this earns
   NOTHING (default stays plain --fuse-midcourse) — don't adopt on a wash.**
   **UPDATE (ADR-0055 verifier): T20 DEFERRED as a probable FULL null —**
   T17-v2 fixed the nominal bias AND ADR-0044's hand-set already survives the
   WORST 2.5 m datum bias 8/8, so the agreement-monitor has no regime left
   where it earns its complexity. Hand-set `--fuse-midcourse` stays the
   recommended fusion; ADR-0054 design stands as ratified-but-unbuilt, revisit
   only for a bias >2.5 m or active time-varying spoof. NOT the next work.
6. **Higher-speed + maneuvering arms** (T21) — **DONE (ADR-0056, plot
   docs/images/t21_maneuver_miss.png):** at 12 m/s + weave/jink, markerless+fusion
   degrades (Pk@2.5: weave 1/8, jink 3/8; miss median 3.7-5.4 m) but a clean
   AprilTag CONTROL holds 8/8 at 1.6 m → the limit is markerless BEARING QUALITY
   (perception), NOT kinematic (guidance/airframe catch the maneuverer) and NOT
   detection rate (both seekers sparse) — the maneuver removes the averaging that
   hid the markerless bearing noise (extends ADR-0042/0043). Fix = terminal
   tracker/subpixel or the real seeker, not guidance. Direction asymmetry =
   markerless perception artifact (tag shows none). Real-cue re-run needs
   maneuvering caches (not built). **Informs T25: feature the AprilTag maneuvering
   / markerless-straight-line regime, be honest about the markerless maneuver limit.**
   - **MANEUVER FIX (builder-directed, video deferred until done):** attempt 1
     `--terminal-reject-gate` NOT VALIDATED (ADR-0057, self-correction): my n=2
     pilot "win" was misread NOISE; the run-to-run false-lock noise floor at
     12 m/s+maneuver is ~5 m (a sporadic coin-flip, not ~1 m); n=8 A/B mixed
     (weave 1/8->5/8 but mostly noise; jink regressed; 50% harm on the 4/16
     gate-firing flights). Gate stays default-OFF/not-adopted. **REOPENED: fix
     at the DETECTION level (temporal/size/confidence consistency or detect-
     then-track), measure at n>=16-24 against the ~5 m noise floor.**
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
- **Multi-range capture — DONE (ADR-0053, rig at 4 standoffs 50-160 m via
  live set_pose):** σ_R∝R² **VALIDATED** at the physics/pipeline level
  (σ_R=3.62e-05·R^2.003, c matches model 0.98x) — closes T18's scaling gap.
  BUT the real detector does NOT generalize: ground_v1 (trained ~160 m) has a
  −6.3 m systematic bias at 50 m + a non-monotonic box-size head out-of-domain.
  → **T17-v2 (multi-range training) now justified by EVIDENCE + is T20's
  PREREQUISITE.** Harnesses `scripts/multirange_*.py` reusable for the v2
  dataset. Do NOT adopt a fusion default on ground_v1 multi-range data (biased).
- **T17-v2 (multi-range detector) — DONE + gate PASS (ADR-0055, verifier
  pending):** ground_v2 fixes the ADR-0053 out-of-domain bias, HELD-OUT-
  validated (70 m −6.17→−0.010 m, 130 m +0.26→−0.14 m; box-size monotonic).
  `ground_v2.{pt,onnx}` = recommended detector. **→ TRIPS T20's pre-registered
  NULL: nominal cue bias now <0.5 m, so the fusion de-bias earns nothing on
  nominal data (don't adopt as default).** T20's anti-jam value (injected
  WORST datum/spoof bias-lock, ADR-0044) stays the only open T20 question.
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
