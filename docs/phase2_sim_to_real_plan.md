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

Sim-cost basis: ~65–75 s/flight (ADR-0044 measurement). Design/NN/triangulation
work is CPU/offline. Stereo rendering adds GPU load — watch dmesg dxgk.

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

## 6. Open ADRs owed (skeletons to fill as each ships)

ADR-0045 stereo rig + world · ADR-0046 ground NN · ADR-0047 triangulation +
measured σ_R · ADR-0048 compute split + real link · ADR-0049 fusion refinement
(bias-state) · ADR-0050 FPV fidelity · ADR-0051 higher-speed/maneuver coverage
· ADR-0052 sim-to-real gap audit · ADR-0053 real-world NN transfer.
(Numbers indicative — assign at write time.)
