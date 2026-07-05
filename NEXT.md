# NEXT — top of the stack

## Current: M4.5 realism upgrade (ADR-0010 sequencing) — S1 ✅, S2 ✅ (built + gated), S3 next

### Where S2 (two-stage handoff) landed (2026-07-05, ADR-0013)
- **The comms-denied headline works in Gazebo:** cue mock (`scripts/s2_cue_mock.py`,
  sigma 0.5 m / 120 ms latency / 10 Hz, all sim-time) → `m4_intercept.py --handoff`
  DASHES at 10 m/s on the cue's PIP lead → camera first sees the tag ~8.8-9.3 m
  mid-dash → HANDOFF latches ~7.2-8.8 m (one-way: cue socket CLOSED, holder
  nulled — structurally unreadable, ADR-0010 #5) → untouched camera-only terminal.
- **6 m/s crosser (uncatchable from hover, ~4.7 m floor) now misses ~1.1-2.3 m.**
  Gate `scripts/check_s2.sh`: both laws, fresh boot each, tiered 2.5 m bar
  (ADR-0010 #7), audits: zero cue reads post-handoff / dash-before-engage /
  law-aware cmd-vs-camera-LOS no-cheat. Run-to-run variance ~1 m (terminal
  dropout timing) — single-flight deltas below ~1 m are noise in this regime.
- **Tracking study (builder-requested): lab winners must re-earn it in Gazebo,
  and Kalata DIDN'T.** Kalata-index filter gains (−24-28% in the lab!) made real
  flights WORSE (bimodal correction cadence → degenerate gains; range channel
  goes deaf → Vc starved). Kept as `--kalata`, default OFF, documented negative
  result. Cue-latency compensation: lab-validated, Gazebo-inconclusive at n=1 —
  kept as `--cue-latency-comp`, default OFF; M5's Monte-Carlo settles it. Lab
  Kalman tracker + range gate: lab-negative, unported. THREE lab-vs-Gazebo data
  points now (PIP, calibration, Kalata): the lab RANKS, Gazebo DECIDES.
- **World→NED mapping (empirical, ADR-0013): north=world_y, east=world_x** — the
  opposite of the naive guess; evidence in m4_intercept.py's docstring.
- S1 recap: `--fpv` profile; pure PN (N=5) 0.94 m vs 3 m/s from hover; PIP
  camera-only doesn't transfer (`--law pip` kept as negative result, ADR-0011).

### Next: S3 — path suite (does it adapt to maneuvers?)
Multi-segment / variable-speed / jink mover (sim-time scheduled — verify sim-time
scheduling composes with DISCONTINUOUS velocity changes). Gate on ~4 paths
{crossing L→R, R→L, S-weave, jink} through the S2 dash+handoff pipeline. Then S4
(all-up adaptation proof: miss-vs-intensity curve, pursuit-vs-pronav per path),
then M5. Full plan: ADR-0010. Note for S3: the board face is rigidly world −X
(ADR-0010 #6) — maneuvers are velocity-schedule changes only; keep target X >
interceptor X during the camera phase.

### M5 (still the finish line — protect it, ADR-0010)
Gate (GOALS.md): Monte-Carlo batch over target speeds/paths, matplotlib
trajectory + miss-distance plots, README with architecture diagram + results +
one GUI demo GIF.

Likely steps (refine before building):
- [ ] Batch runner: N runs × {law × target speed (2.0/2.5/3.0) × path variant
      (crossing y-offsets, maybe a receding case)} reusing check_m4.sh's
      boot-per-flight pattern (or one boot + re-place tag + re-takeoff per run if
      state contamination is manageable — drone lands displaced; probably NOT, see
      the fresh-sim-per-flight lesson). Runs are ~3 min each wall — budget hours,
      run in background, aggregate CSVs.
- [ ] Stats + plots: trajectory overlays (pursuit vs pro-nav, same path), miss
      histogram/CDF per law, maybe miss vs target speed. matplotlib, saved to
      docs/ or plots/. Numbers traced to run stamps (GOALS.md).
- [ ] README: mission, architecture diagram (camera → detector → filters →
      guidance → PX4; mover + ground-truth split), results tables, reproduce
      instructions (check_m0..m4 + batch), demo GIF (GUI + gz camera view?).
- [ ] Demo GIF: screen-record a GUI pursuit-vs-pronav pair (Emerson's standing
      request to watch is also the demo content). Tools: peek/ffmpeg x11grab?
- [ ] Consider small robustness items first: pro-nav miss variance across runs
      was 0.28-0.44 m (fine vs 1.0 gate); RTF-load sensitivity documented.

## Real-world deployment roadmap (movement + perception realism) — added 2026-07-05 per builder
These items track the "accuracy AND movement" goals toward the real deployable
concept (ground-standby → launch-on-detect → max-speed intercept). NONE are
built yet; they extend the MOVEMENT model. Accuracy is the ADR-0014 Pk work
already in flight. Design each as its own ADR before building. Sequence AFTER
the current Pk push + S3/S4, so we don't destabilize a passing S2.

**Movement / engagement-profile realism (new phases before CUE_WAIT):**
- [ ] **M-1 Ground standby (cold start).** Model the interceptor armed on a pad
      (charger), motors off, not the current hover-start. Adds an arm→spin-up→
      liftoff sequence and a reaction-latency budget (detect→launch decision→
      motors) — a real, measurable number for the resume ("X s from cue to
      airborne").
- [ ] **M-2 Launch-on-detect trigger.** The ground sensor (the S2 cue) firing a
      threat detection is what LAUNCHES the interceptor — today the cue only
      steers an already-flying drone. Wire the cue's first valid track to the
      arm/takeoff trigger; log cue-detect→liftoff latency.
- [ ] **M-3 Max-speed climb-out + dash.** Vertical/oblique boost to altitude then
      full-speed run-in until terminal (the dash already commands max toward a
      predicted intercept point — extend it to start from the pad and include the
      climb). Tie closing-speed envelope to ADR-0010 #2 (dash vs terminal decouple)
      and the FPV param bundle. This is the "launches up and is max speed until
      interception" behavior.
- [ ] **M-4 Standby→intercept end-to-end timeline.** One logged run: cue-detect →
      launch → climb → dash → handoff → terminal → CPA, with sim-time stamps per
      phase. The headline "reaction + intercept time" figure.

**Perception realism (the honest gap — see the answer logged below):**
- [ ] **P-1 Document the tag↔real-seeker gap explicitly in the README.** The
      AprilTag is a STAND-IN for "a reliable target lock exists" (GOALS.md's one
      honest simplification); a real hostile FPV drone carries no fiducial. What
      transfers is the GUIDANCE loop (bearing/LOS-rate → pro-nav → PX4), which is
      agnostic to how the bearing is produced. Pro-nav is bearing-only-friendly
      (it needs LOS *rate*, an angle) — a genuine plus for a monocular terminal
      seeker that gets angle reliably but range poorly.
- [ ] **P-2 (stretch, likely parent-project) Real terminal seeker:** classical CV
      (motion/blob detection + a correlation tracker like KCF/CSRT + an
      alpha-beta/Kalman track) or a tiny CNN detector — the actual hard, unsolved
      part. ADR-0012 called detection cadence + motion blur at FPV speed the
      EXISTENTIAL risk. GOALS.md forbids ML perception IN THIS SIM, so this is a
      hardware/parent-project line, not a sim milestone; note it, don't build here.
- [ ] **P-3 (stretch) Real ground-camera cue:** the S2 cue is a MOCK (degraded
      sim ground-truth). A real cue = stereo triangulation across 2+ ground
      cameras → EKF track → the same handoff interface. The handoff ARCHITECTURE
      is already built + validated; only the detector/triangulation is mocked
      (GOALS.md IS-NOT: "ground stereo rig … mocked away here").

## Done
- **M4.5-S2 (2026-07-05):** two-stage external-cue handoff — built, dev-validated,
  gated (`scripts/check_s2.sh`, tiered 2.5 m @ 6 m/s, both laws + honesty audits).
  Kalata gains + cue-latency-comp ported behind flags, A/B'd in Gazebo: Kalata
  rejected (negative result), latency-comp inconclusive; both default OFF.
  ADR-0013 has the full story.
- **M4.5-S1 (2026-07-05):** FPV profile behind `--fpv`; 0.94 m vs 3 m/s crosser
  from hover; hover-start capped at ~3 m/s (the S2 coupling finding). ADR-0010/0011.
- **M4 (2026-07-05):** moving-target intercept, pursuit vs pro-nav — GATE
  PASSING, verifier-confirmed (3 independent gate runs): pro-nav miss
  0.402/0.277/0.443 m (bar <1 m) vs pursuit 2.544/2.109/2.048 m, identical
  2.0 m/s crossing paths, camera-only + no-cheat numerically verified.
  Mechanization: strapdown λ=ψ+β, alpha-beta filters (λ rate gain 0.30),
  a=N·Vc·λ̇ (N=4) integrated into a world-frame lateral velocity, NED velocity
  + absolute-yaw setpoints, constant 3.0 m/s closing, terminal coast at 2.0 m,
  camera-only breakoff. THE debugging saga (5 dev runs + 2 failed gates → root
  cause: RTF ~0.5 under load + wall-clock mover = target effectively 2× speed;
  fixed by sim-time scheduling) is in ADR-0009 + two addenda — read before
  touching anything timing-related. quad_decimate=2 for M4 detector only.
- **M3 (2026-07-05):** static intercept — camera-only P-control (body-frame
  velocity + yawspeed, ADR-0008) closed 4.9 m → 2 m standoff in ~10 s, held.
  Final standoff error 0.018/0.035 m across two verifier runs (bar <0.5 m),
  detection coverage 1.000, zero overshoot. Gate `scripts/check_m3.sh`;
  guidance script `scripts/m3_static_intercept.py` (reuses m2_detect + m0_takeoff
  by import). Verifier numerically proved commands trace to measured range, not
  ground truth. Key integration facts: offboard needs a setpoint streamed BEFORE
  `offboard.start()` and ≥2 Hz after; detection runs on its own thread (latest-
  frame-wins, no queue) at full 20 Hz control coverage; `sleep 5` after boot
  before MAVSDK connect (check_m3.sh) since M3 arms right after health-OK.
- **M0 (2026-07-04):** toolchain — PX4 v1.17.0 built, Gazebo Harmonic 8.14, venv up.
- **M1 (2026-07-04):** camera pipeline — 10/10 frames @ 1280×960 via gz-transport13.
- **M2 (2026-07-04):** AprilTag detection — custom `worlds/apriltag.sdf` +
  `models/apriltag_target/` (tag36h11 id0, 0.625 m plane / 0.5 m black square, tag
  center at (5, 0, 0.5) facing -X); `scripts/m2_detect.py` gate: detection_rate
  1.000, mean err_norm 0.0861 m (bar ≤0.25 m), mean range 4.888 m
  (`logs/m2_detect_20260704T233941Z.csv`). Two real bugs found and fixed along the
  way — see ADR-0006 (ground-truth chain: camera_link composes directly against
  the model, NOT via base_link — a numeric coincidence made the wrong chain look
  right) and ADR-0007 (tag material needed `emissive_map`, not just a lit
  `albedo_map`, to stay legible at range under the world's fixed sun angle).

Key facts for a fresh session:
- PX4 at `~/PX4-Autopilot` (v1.17.0). Launch camera drone on the M2 world:
  `PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH=~/interceptor-sim/models HEADLESS=1 make px4_sitl gz_x500_mono_cam`
  (plain `gz_x500_mono_cam` with no world env var still boots the "default" world
  from M0/M1). `worlds/apriltag.sdf` must be symlinked into
  `~/PX4-Autopilot/Tools/simulation/gz/worlds/` — `scripts/check_m2.sh` creates/
  repairs that symlink automatically (see ADR-0005 for why it can't just be an
  env var).
- Camera: 1280×960 @ 30 Hz, hfov 1.74 rad, RGB_INT8. On the M2 world, topics are
  under `/world/apriltag/...` (world name matters for gz-transport topic paths —
  rediscover via `gz topic -l` if world/model names change). Intrinsics measured
  and recorded in `camera_intrinsics.json`: fx=fy≈539.936 px, cx=640, cy=480,
  matching the (fx≈(1280/2)/tan(hfov/2)) cross-check almost exactly.
- Ground truth for M2 (and a template for M3+) comes from `/world/apriltag/pose/info`
  (gz.msgs.Pose_V) — see scripts/m2_detect.py's docstring and ADR-0006 for the
  transform-chain gotcha (camera_link's pose is relative to the model directly,
  NOT to base_link, despite a numeric coincidence suggesting otherwise).
- venv sees system gz bindings via `.venv/.../site-packages/gz_system.pth`
  (python3-gz-transport13 + python3-gz-msgs10 from apt). No scipy — quaternion ->
  rotation matrix is hand-rolled in scripts/m2_detect.py (`quat_to_matrix`) to
  keep the dependency surface minimal (GOALS.md), reused rather than
  re-implemented for M3+ frame math.
- Boot-complete grep: "Startup script returned successfully" (ADR-0004 — never wait
  on "Ready for takeoff!" pre-MAVSDK).
- MAVSDK: `udpin://0.0.0.0:14540`. AprilTag lib: pupil-apriltags (ADR-0003).
- S2 runtime facts: cue mock streams UDP JSON on 127.0.0.1:47800 (sim-time
  scheduled); `--handoff` requires `--fpv`; S2 geometry default = target start
  (6.5,-14,0.5), vel (0,6); `check_s2.sh [pip|pronav]` runs a single law for dev.
  m4_intercept.py now holds a /clock subscription (SimClockHolder) — safe because
  it makes no gz service calls (only the mover does, in its own process).
- S2 A/B hygiene: miss variance across identical flights is ~1 m (terminal
  dropout timing). Never conclude from a single flight; use the mechanism
  evidence (logged filter gains, coverage) plus >=2-3 flights per config.
- Minor: m0_takeoff.py duplicates its final CSV row — tidy if reused as template.
