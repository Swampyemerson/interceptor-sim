# Phase 2 — Sim-to-Real: make the sim a hardware blueprint

*Ratified by the builder 2026-07-08 (evening), immediately after the post-M5
software arc closed (ADR-0038..0044). This is the master plan for Phase 2. A
fresh session should read this + NEXT.md + the anti-jam Q&A context below, then
DRIVE it autonomously — execute the phases in dependency order, convene a
council on the one-way-door forks, and proactively run the analytical
deep-dives (do not wait to be asked the questions; ask them yourself and
answer from the logs).*

## 0. Mission shift

Phase 1 proved the **guidance + honesty + perception core** in sim, but the
ground sensor was a MOCK: `s2_cue_mock.py` reads the target's true sim pose and
corrupts it with a measured noise model. Phase 2's mandate (builder's words):
**"Make the stereo rig in the sim, we will need code for those so I can
implement all this in real life."** Replace every mock with a REAL,
implementable pipeline so the sim doubles as the hardware build blueprint:
render two ground cameras → detect → triangulate distance+velocity → send over
a real link to the drone → fuse with jam-resistant confidence. Then push
fidelity (real FPV speeds), coverage (higher-speed/maneuvering intercepts), and
a systematic sim-to-real gap audit.

## 1. Where the mock ends and reality must begin (current state)

| Piece | Phase-1 state (mock) | Phase-2 target (real) | Task |
|---|---|---|---|
| Ground target position | true sim pose + synthetic σ_R/datum/dropout noise (`s2_cue_mock.py`) | two rendered cameras → NN detect → stereo triangulation | T16, T17, T18 |
| Ground target velocity | synthetic (EMA of true positions) | ground-side track filter on triangulated positions | T18 |
| Compute location | one process, everything on the PC | ground-station process + drone process, real UDP track link | T19 |
| Ground detector | none (reads gt) | sim-domain fine-tuned NN (like onboard v2) | T17 |
| Fusion confidence | hand-set polar FusedTrack (won) / covariance EKF (regressed under bias) | bias-state / camera-favoring confidence that rejects persistent jam | T20 |
| Airframe speed | x500 @ 16 m/s dash | real FPV envelope (25–40 m/s, payload mass) | T22 |
| Speed/maneuver coverage | markerless+fusion only at 9 m/s straight-line | 12 m/s + weave/jink with markerless+fusion | T21 |

**Honesty boundary is UNCHANGED and non-negotiable (ADR-0010/0034/0044):** the
handoff is a one-way latch; the terminal phase is camera-only; no cue
influence (state OR covariance) survives the latch. Making the cue REAL does
not relax this — it makes the jam-resistance story *testable with a real
spoof injected into the real pipeline* instead of asserted.

## 2. Technical grounding (from the 2026-07-08 anti-jam Q&A — read before building)

- **Effective range / coverage:** ground rig = 2× AR0234 + 16 mm, 2.0 m
  baseline (ADR-0017). Daytime EO-only detection envelope ~60–160 m; blind at
  night w/o thermal (deferred). σ_R ≈ 4.45e-05·R² (cm at 30 m, ~1 m at 150 m).
  Coverage is a WEDGE along the shared FoV, not a dome — wide area = multiple
  rigs (out of sim scope).
- **Frames / placement:** rig and drone each know their own position (GPS-like);
  the modeled error is the DATUM BIAS between their frames (0.3/0.5/2.5 m
  best/expected/worst). Moving the launch station re-learns nothing — the cue
  is world-frame, the drone converts via its own nav; only the residual datum
  error matters. This is the exact quantity a real GPS/survey offset produces.
- **Compute split (ADR-0016):** ground = triangulation + global track; drone =
  seeker inference (Pi 5 + Hailo); joined by a ~90-byte track message. The
  skinny jammable link is WHY the terminal must be self-contained.
- **What won / lost in fusion (ADR-0044):** hand-set polar FusedTrack (cue
  never touches the bearing channel) beat fusion-off 8/8 and survived WORST
  bias. Covariance-gated EKF regressed under persistent bias (the gate tightens
  around a spoofed offset then rejects the true camera). The refinement (T20)
  must attack THAT: a persistent bias is not zero-mean noise — estimate it
  (bias-state) or hard-favor the camera once locked.

## 3. Work breakdown — dependency-ordered, recommendation-first

Each item: what / recommended approach / the fork to decide (council the
one-way doors) / ADR skeleton owed. Build order follows dependencies.

### Track A — the real ground pipeline (the spine; T16→T17→T18→T19)

**A1. Two-camera stereo rig world + model (T16).**
- Recommended: a NEW `models/ground_stereo_rig/` with TWO camera sensors
  (left/right, 2.0 m baseline, matched intrinsics to ADR-0017's 16 mm-equiv),
  placed as a static model in a new `worlds/stereo_intercept.sdf` that also
  carries the tag-less target. Both cameras publish distinct gz-transport
  topics; a ground-station process subscribes to both. Keep the onboard mono
  camera unchanged (3 camera streams total headless — confirm GPU render load
  is OK on the 4070; the infra scout checks this).
- Fork F1 (council if uncertain): real two-camera render (recommended — the
  builder explicitly wants real cameras doing the estimate) vs. one ground
  camera + analytic range. Recommendation: real stereo — it's the blueprint.
- ADR: rig geometry, sensor blocks, topic names, why static placement.

**A2. Ground NN detector (T17).**
- Recommended: reuse the onboard v2 recipe end-to-end — render an in-domain
  dataset from the GROUND viewpoint/range band (`render_sim_dataset.py`
  pattern), hard negatives (the rig sees sky/horizon/ground clutter, not own
  props — different negatives), fine-tune yolo11n, calibrate. One detector runs
  on each camera (or the primary + a disparity search on the other).
- ADR: dataset grid for the ground viewpoint, negatives, gt-label honesty
  (training-time only, same boundary as onboard).

**A3. Triangulation + ground velocity/track (T18).**
- Recommended: detect target in both frames → disparity → triangulate 3D
  world position (single known target = trivial correspondence) → feed a track
  filter (alpha-beta or EKF) → emit position AND velocity. VALIDATE the
  empirical σ_R vs ADR-0017's c=4.45e-05 model — if the real triangulation is
  noisier/cleaner than the mock assumed, that's a finding that updates every
  prior fusion conclusion.
- Fork F3: NN-per-camera vs. detect-once-and-search; track filter choice.
- ADR: triangulation math, measured σ_R vs model, velocity method.

**A4. Compute split + real link (T19).**
- Recommended: promote `s2_cue_mock.py`'s UDP JSON transport (already the
  cue interface on :47800) into a real two-process split: a `ground_station.py`
  runs A2+A3 and emits the real track; `m4_intercept.py` consumes it UNCHANGED
  (it already reads that transport). Feature-flag: a `--cue-source
  {mock,stereo}` selector, default `mock` so EVERY gated M0-M5/S2/markerless
  reproducibility path stays byte-identical. The real pipeline is the new,
  opt-in path.
- Fork F2 (council): reuse UDP JSON vs. a tighter real message; how to keep
  gated reproducibility intact. Recommendation: reuse the transport, add the
  flag, preserve the mock as default.
- ADR: the split, the flag, the byte-identity guarantee, the message format
  vs the real 90-byte target.

### Track B — fusion refinement (T20; prototype on mock, ADOPT on real A3 data)

- The builder's push: "refine the confidence system to weight the onboard
  camera more and disregard the bad jam data." ADR-0044 showed WHY the naive
  covariance EKF fails: it models a persistent datum bias as zero-mean noise.
- Recommended two-pronged: (i) **bias-state augmentation** — add the datum
  offset to the EKF state so it's ESTIMATED, not averaged (directly kills the
  WORST-tier bias-lock); (ii) **camera-favoring confidence** — once the onboard
  camera holds a lock streak, down-weight the cue's influence on its own (the
  camera IS better terminally, measured). Keep the polar discipline (cue never
  touches bearing) regardless.
- SEQUENCING (builder's call, adopted): prototype/validate on the mock now, but
  do NOT adopt a fusion default until it's tested on REAL stereo data (A3) —
  the mock's noise model may not match the real triangulation's, and adopting
  on mock data would repeat the ADR-0018 "measured under the wrong conditions"
  mistake.
- ADR: the bias-state math, the confidence schedule, mock-vs-real validation.

### Track C — fidelity + coverage (T21, T22; independent, run anytime idle)

- **T22 FPV fidelity (design first, no sim):** identify PX4 params
  (MPC_XY_VEL_MAX, MPC_ACC_HOR_MAX, MPC_TILTMAX_AIR, thrust/mass) that set the
  speed/accel envelope; model payload mass; propose a `--fpv-fast` profile
  matching a real 2.5–7" build carrying the seeker payload. NOTE the ADR-0028
  finding: the binding constraint was the GUIDANCE command ceiling
  (V_PERP_MAX/V_TOTAL_MAX), not airframe agility — so raising airframe caps
  only helps if the guidance ceiling rises too. Design doc `fpv_fidelity_design.md`.
- **T21 higher-speed/maneuver arms (sim, serialized):** the markerless+fusion
  arc ran 9 m/s straight-line ONLY. Fly 12 m/s + weave/jink with `--seeker
  markerless --fuse-midcourse` (v2 weights), paired seeds, per-tick audit +
  abort lens. This closes the honesty gap flagged in the Q&A ("the newest
  results are still straight-line").

### Track D — the hardware-readiness reviews (T23, T24; design docs)

- **T23 sim-to-real shortcomings audit:** gap table (severity + bench-measurable)
  across frames, timing/latency realism, comms model, GPS/datum, intrinsics vs
  real lens, IMU/EKF2 realism, thermal/night, safety interlocks (the bird
  discrimination + fail-safe from ADR-0035).
- **T24 real-world NN transfer plan:** our sim YOLO11n won't transfer (Gazebo
  domain + AGPL). MIT model vs camera-specific fine-tune, for BOTH onboard and
  ground; the Stage-0 data loop; what transfers (hard-negatives lesson,
  self-mask, calibration, polar discipline) vs what's rebuilt.

## 4. Recommended build sequence

1. **T16 stereo rig world** (foundation; unblocks everything real).
2. **T17 ground NN** ∥ **T22 FPV design** ∥ **T24 NN transfer plan** (parallel).
3. **T18 triangulation+velocity** (needs T16+T17) — the σ_R validation is a
   pivotal checkpoint: it may revise every fusion conclusion.
4. **T19 compute split** (needs T18) — now the drone flies on REAL cue data.
5. **T21 higher-speed arms** (can run earlier on the mock; re-run on real cue
   after T19). **T20 fusion refinement** validated on the T18/T19 real data.
6. **T23 shortcomings audit** folds in findings from each step.

Sim-cost basis: ~65–75 s/flight (ADR-0041 measurement — 5-batch average; the
number survives F1 candidate (a) unchanged, but under (b)/(c) re-measure the
per-flight wall cost at the chosen rig config, and require rig-world RTF inside
the ADR-0009-safe band — not merely "boots" — before pre-registering any
real-cue gate). Design/NN/triangulation work is CPU/offline. Stereo rendering
adds GPU load — watch dmesg dxgk.

## 5. How to drive this phase (for the autonomous session)

- **Don't stop after one step.** Phase 2 is a chain; complete a step, gate it,
  commit it, and START the next unblocked one the same session
  ([[session-persistence-mandate]]). Between sim runs, pull design/analysis
  forward.
- **Ask the analytical questions yourself.** The builder values the deep-dives
  (anti-jam mechanics, coverage, sim-to-real gaps). When a step lands, proactively
  ask "what did this actually prove, where does it break, what's the honest
  limit" and answer FROM THE LOGS — don't wait to be asked.
- **Council the one-way doors** (F1 stereo realization, F2 compute-split
  reproducibility, F3 triangulation pipeline) before building them; make
  educated decisions + ADR-log the reversible ones.
- **Standing bars:** `audit_per_tick.py` + `abort_lens.py` on every arm;
  pre-register gates BEFORE flying; verifier-gate every close-out; honesty
  boundary re-earned on every new cue path.
- **The pivotal risk:** the single biggest way this blueprint could MISLEAD the
  hardware build is a sim-to-real fidelity gap that flatters the design — the
  T23 audit exists to surface those; treat a too-good sim number as a bug to
  investigate, not a win.

## 5.5 Infra map + feasibility (2026-07-08 scout — read before T16)

**Nothing real exists yet.** `stereo_model.py` is a pure analytic error budget
(no Gazebo); `s2_cue_mock.py` reads gt pose + adds noise. No disparity /
triangulation / rectification code anywhere. So T16–T18 is greenfield.

**Camera-sensor precedent to copy:** `worlds/apriltag_demo.sdf:336-369`
(`demo_chase_cam`) is a static `<model>` in the world carrying its own
`<sensor type="camera">` — exactly the rig shape (not bolted to the PX4
airframe). The onboard sensor block lives in PX4's tree
(`~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf:50-67`):
hfov 1.74, 1280×960, RGB_INT8 default (every decoder asserts this), no
`<topic>` (gz auto-derives `/world/<W>/model/<model>/link/<link>/sensor/<sensor>/image`).

**★ THE PIVOTAL FEASIBILITY RISK — RTF collapse (recorded at
`worlds/apriltag_demo.sdf:300-308`).** Adding ONE extra camera at 1280×720
collapsed real-time factor to **~0.05** (10× below the ADR-0009 under-load
floor — broke MAVSDK OFFBOARD entry); dropping to **960×540 restored ~1.0**.
A real stereo rig is TWO cameras + the onboard = **three camera renders**, and
`stereo_design.md` wants **1920×1200** per rig camera for 100 m ranging — far
past the safe config. Compounded by likely **llvmpipe software rendering**
(GPU-headless is unverified here; confirm `nvidia-smi` shows Gazebo load before
committing). **This reshapes fork F1:** live three-camera render during the
flight loop is probably infeasible. Strong candidate approaches for the fresh
session to weigh (council F1): (a) **decouple the rig render from the flight** —
teleport-snapshot the two rig views offline (the `render_sim_dataset.py`
pattern) to build/validate the triangulation pipeline, then feed its *output
statistics* into the cue at flight time (a measured, not synthetic, noise
model — a real improvement over the mock without paying live render) — BUT
note the tension the 2026-07-08 review confirmed: (a) satisfies the T16–T18
validation goals while the flight-time cue quietly stays a (recalibrated)
mock, which does NOT satisfy §1's "real spoof injected into the real
pipeline" promise as written. If (a) wins, T19 is REDEFINED as:
ground_station.py runs the real detect+triangulate code path on
replayed/snapshot frames and emits over the real :47800 link (real code, real
link, offline frames); the spoof test injects into THAT pipeline; and the
council must state explicitly whether offline-replay triangulation output
counts as "real stereo data" for T20's adoption gate; (b) low res + low rate
rig cameras live — the RATE lever is the credible one (the cue is consumed at
10 Hz sim-time, s2_cue_mock.py; 2×1920x1200@5–10 Hz and 2×960x540@10 Hz
straddle the known-good/known-bad throughput points), while any RESOLUTION
cut scales f_px down and the σ_R constant c = σ_d/(b·f_px) UP — so a low-res
rig invalidates the fixed c=4.45e-05 target and shrinks the 60–160 m
envelope; the T18 validation must compare against the resolution-scaled c
(see §5.6 #7); (c) a separate sim instance for the rig — near-disqualified:
it conflicts with the incident-backed ONE-sim-at-a-time batch-hygiene rule
and needs a cross-instance gt-pose bridge (a brand-new honesty-audit surface)
plus a clock-mapping layer between two independent sim clocks; only viable if
the council ratifies an explicit exception covering all three. Do
NOT assume live full-res stereo works — measure RTF with the rig first
(scripts/probe_stereo_rtf.sh is that measurement).

**The clean bridge (keeps everything downstream unchanged):** `s2_cue_mock.py`
already has a `--stereo-config B,F_PX,SIGMA_D` hook (baseline 2.0, focal 540,
σ_d 0.5 — `s2_cue_mock.py:173-175`, explicitly "swap in real calibration").
Real triangulation output can emit the SAME `:47800` JSON (`{seq,t_sim,x,y,z,
vx,vy,vz}`), so `m4_intercept.py`'s `CueReader` + the world→NED mapping
(north=world_y, east=world_x) stay byte-unchanged. New plumbing needed: a new
world SDF + its PX4 symlink (the `check_*.sh` pattern), rig topics into a
`GzFrameSource`/`demo_capture_frames.py`-style ingest, and a new
triangulation/track process replacing the mock. Multiple processes CAN each
hold camera subscriptions concurrently (proven in-repo); only a process that
calls gz *services* (set_pose) must stay subscription-free (the mover pattern).

## 5.6 Ratified-plan amendments (2026-07-08 adversarial review — 18 confirmed findings)

*A 5-dimension / 2-skeptic-per-finding review ran before any Phase-2 build
started. 3 raw findings were refuted (notably: the plan does NOT contradict
itself over §5.5 vs A1 — the council delegation is coherent; and no repo
evidence contradicts the llvmpipe suspicion). The 18 confirmed ones are folded
into the sections above where they were local edits; the requirement-level
additions live here. Each is BINDING on the task it names.*

1. **Sim-time discipline for the real cue (CRITICAL — binds T18/T19).** A real
   two-process pipeline's NN + triangulation + UDP latency happens in WALL
   time; under RTF < 1 that maps to a *smaller* sim-time delay, so the real
   pipeline would deliver cues systematically FRESHER than reality — and
   fresher than the mock it is A/B'd against (a flattering-gap violation of
   the sim-time rule, ADR-0009 class). Required: (i) stamp `t_sim` from the
   camera frame's sim-time header, never from a clock at emit time; (ii)
   impose a MODELED sim-time latency floor (mock-style `--latency-s`) so
   delivered latency is a controlled knob ≥ the measured pipeline latency;
   (iii) log delivered sim-time latency per datagram; (iv) T19's
   pre-registered gate includes measured sim-time latency vs the ADR-0016
   tier being modeled.
2. **Bias/spoof injection in the REAL pipeline (binds T18/T19; T20 depends).**
   In sim the rig and drone share one world frame — a real triangulation
   pipeline has ZERO natural datum bias, so the WORST-tier bias-lock tests
   (and §1's "real spoof" promise) are unrunnable without an explicit
   mechanism. Required: ground_station.py takes `--datum-bias-m` /
   `--assumed-rig-pose` perturbing the rig extrinsics used for triangulation
   (physically faithful to a GPS/survey offset), plus a track-message-level
   spoof injector for the jam story. Without this, T20's "adopt only on real
   data" gate is empty.
3. **m4_intercept.py's real change surface (binds T19/F2).** "Consumes it
   UNCHANGED" is true only of CueReader parsing — m4_intercept.py OWNS the
   cue process lifecycle (spawn, seed, kill). The F2 council decides who owns
   ground_station.py's lifecycle (recommended: spawned by m4_intercept.py
   like the mock); the mock branch stays byte-identical INCLUDING the spawned
   command line.
4. **Ground-side honesty mechanism (binds T18/T19 gates).** "Honesty boundary
   re-earned" gets a concrete form: a subscription audit asserting
   ground_station.py holds NO gt/pose subscriptions at runtime — camera
   topics + clock only (the mover-pattern equivalent for the ground side) —
   added to the pre-registered T18/T19 gates alongside audit_per_tick.py.
5. **Rig placement / FoV coverage analysis (binds T16).** The rig sees a
   wedge; nothing guarantees the ratified dash profile crosses it. Required
   T16 sub-step: compute the shared-FoV wedge (HFOV + 60–160 m envelope)
   against the flown intercept profile in BOTH directions, choose and
   document the rig pose so the target is inside the wedge from cue-wait
   through the handoff, and log a per-flight in-wedge fraction.
6. **Two drones in frame (binds T17/T18).** The ground NN will see the
   INTERCEPTOR too; "single known target = trivial correspondence" is false
   the moment both are airborne. Required: interceptor airframe in the ground
   dataset (distinguishable class or hard negative) + a ground-side identity
   gate excluding detections consistent with the interceptor's own downlinked
   nav position (legitimate ground-side info per ADR-0016).
7. **σ_R model must scale with the flown resolution (binds T16/T18).** If RTF
   forces a resolution below the 1920×1200 design point, f_px drops and
   c = σ_d/(b·f_px) rises — validating against the fixed c=4.45e-05 would be
   comparing to the wrong model. Required: T18 compares measured σ_R against
   the resolution-scaled c, and records which (resolution, f_px, c) triple
   feeds the flight-time cue.
8. **Cue-velocity quality analysis (binds T18, informs T16's sensor rate).**
   The fusion arc assumed σ_v ≈ 0.5 m/s from the mock. Required analytic
   pre-step: propagate σ_R through the candidate track filter at candidate
   frame rates → predicted (σ_v, lag) curve; choose the rig camera rate from
   it; measured σ_v vs the mock's 0.5 m/s becomes T18's second checkpoint
   next to σ_R.

## 6. Open ADRs owed (skeletons to fill as each ships)

ADR-0045 stereo rig + world · ADR-0046 ground NN · ADR-0047 triangulation +
measured σ_R · ADR-0048 compute split + real link · ADR-0049 fusion refinement
(bias-state) · ADR-0050 FPV fidelity · ADR-0051 higher-speed/maneuver coverage
· ADR-0052 sim-to-real gap audit · ADR-0053 real-world NN transfer.
(Numbers indicative — assign at write time.)
