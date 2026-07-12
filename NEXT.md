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

## 📍 CURRENT (2026-07-11, live)

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
