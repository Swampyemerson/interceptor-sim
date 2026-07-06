# NEXT — top of the stack

<!-- ============================================================= -->
<!-- CURRENT (2026-07-05 late, autonomous perception build)         -->
<!-- ============================================================= -->
## ✅ DONE — Gazebo emit-vs-differentiate A/B (ADR-0015 2nd addendum)
Paired batches (N=8/arm, master-seed 42, realistic cue): EMIT mean 1.394 m /
Pk@2 75% vs DIFF 1.808 m / Pk@2 50%; DIFF worse on 6/8 paired seeds (Δ+0.41 m,
not yet significant at n=8). Direction confirms the lab's #1 lever; magnitude
~5× smaller than lab. **Load confound found:** the old ~3.1-3.3 m DIFF points
flew with 2 other Claude sessions loading the box — batches only compare at
matched (idle) machine load; kill stale sessions before any batch. All 16/16
flights still end in terminal dropout at CPA → terminal perception stays THE
limiter. Verdict logged: ground link carries filtered VELOCITY (Gazebo-backed).

## ✅ DONE — stereo/onboard integration + compute setup (builder directive 2026-07-05)
Builder: usage unconstrained; Opus/Fable/Sonnet subagents. **Methodology mandate
(now a standing rule): simulate WORSE-than-ideal heavily; three tiers
BEST/EXPECTED/WORST-CREDIBLE; decisions must survive WORST; every sim knob maps
to a bench-measurable quantity; batches only at idle machine load.**
- **ADR-0016 compute setup (P-3/5/7):** `docs/compute_setup.md` — hybrid split,
  ~90-byte track message (7.2 kbps vs 32 kbps SiK; video impossible), 12-row
  3-tier latency budget. Sim's 0.12±0.05 s cue latency SURVIVES (EXPECTED
  ~90 ms, WORST ~210 ms) — add a 0.20 s stress tier. BOM: Orin NX 16GB ~$899,
  MANET $1.5-3k+ ≠ $50 SiK. Ground node ~$1.6k EO-only.
- **ADR-0017 stereo rig (P-2):** `scripts/stereo_model.py` + `docs/stereo_design.md`
  + plots. AR0234 ×2 + 16 mm, 2.0 m baseline (knee: calibration term R²·δθ/b,
  focal cancels; WORST optimum b≈2.24 m), hardware trigger MANDATORY. EXPECTED
  σ_R≤1 m to 150 m. **Sim correction: mock's c=0.008 is ~180× too steep as
  stereo noise — real split c=4.45e-05 + datum in --datum-bias-m (adopt at next
  cue change).**
- **ADR-0018 fusion (P-6):** FusedTrack bearing-weighted polar fusion + warm
  handoff, lab + Gazebo port, all default-OFF (byte-identical verified; S2 gate
  re-run PASS 1.963 m post-merge). Lab: mean-miss neutral BUT terminal coverage
  through CPA 0.65/0.35/0.08 → 0.94/0.96/0.96 (6/8/10 m/s, EXPECTED) — the
  exact Gazebo failure mode. No datum poisoning. WORST: link cut at 11.5 m
  precedes camera acquisition → fusion window never opens (coast-search is the
  mitigation there).

## ✅ DONE — Gazebo fusion A/B (ADR-0018 addendum)
3 paired arms N=8, seed 42, realistic cue. FUSE −0.088 m / FUSEWARM −0.074 m
mean vs BASE (within ~1 m noise, n=8). **Lab's coverage win did NOT transfer:**
Gazebo coverage flat 0.131/0.131/0.135, 8/8 dropout-at-CPA in every arm —
fusion is pre-latch, dropout is post-latch (camera-only by design), so it
can't touch the real failure mode. Verdict: keep --fuse-midcourse (small,
non-harmful handoff-geometry gain), warm-handoff adds nothing in Gazebo, both
default OFF. 5th lab-vs-Gazebo divergence. Reinforces: intercept is gated by
TERMINAL perception, which no mid-course aid reaches.

## ✅ DONE — P-1 ground sensor modality (ADR-0019, docs/ground_modality.md)
Staged-thermal HOLDS with 2 corrections: bird-rejection is radar/motion/ML not
thermal (birds are warm); "thermal=all-conditions" oversold (crossover-blind,
fog-degraded); Boson 640 = $3,558 (doubles the EO rig). RF-defeat-by-fiber-FPV
confirmed (NATO 2025). Honest EO-only envelope: ~60-160 m daytime, blind at
night. Cheapest night path = 1 mono LWIR core, not thermal stereo.

## ✅ DONE — README.md draft (M5 skeleton, P-0 honesty section)
Mission, headline results table (all traced), Mermaid architecture + ASCII
fallback, the P-0 honesty section (AprilTag stand-in; guidance transfers,
perception doesn't; lethal-radius = narrative assumption; lab-vs-Gazebo 5×;
worse-than-ideal 3-tier), guidance arc, perception-half pointers, reproduce
instructions, repo map. TODO slots: M5 Monte-Carlo final numbers + demo GIF.

## ✅ DONE — terminal-solutions research (5 lanes, builder "get solutions that pan out")
Root cause FOUND + verified (ADR-0023): the miss is KINEMATIC, 96% locked at
handoff (ZEM r²=0.99), NOT terminal perception — corrects ADR-0014. Consolidated
plan: `docs/terminal_solutions_plan.md`. Lanes: ADR-0020 jam envelope (maneuver
sets margin), ADR-0021 kill mechanism (ram/net, can't buy out perception),
ADR-0022 real-world guidance (PROPOSED, proximity metric council-worthy),
ADR-0023 diagnosis (linchpin), ADR-0024 seeker (narrower acquisition lens,
reverses wide-FOV). **The working plan, cheapest-first:**
- [ ] **T1 (free software, do first):** earlier handoff (start terminal at first
  detection ~7.6 m not latch ~6.2 m) + split-freeze/later-freeze + warm-settled
  filters (reclaims ~0.3-0.45 m) + keep cue-velocity. Lab A/B → Gazebo mc_batch,
  near-R=0 regression so M4 gate holds.
- [ ] **T2 (~$15-35, reopens M1/M2):** narrower/longer acquisition lens → earlier
  lock → bigger t_go (capacity ∝ t_go²). Narrow SDF hfov, raise HANDOFF_RANGE,
  re-baseline M1/M2 + disclose, re-run mc_batch.
- [ ] **T3 (BUILDER/COUNCIL call):** adopt proximity Pk-vs-radius metric (ram
  ~0.5 m wins slow regime; net ~1.5 m fast regime) — one-way door on the resume
  line, do NOT flip unilaterally.

## ✅ DONE — Tier-1 free-software reclaim (ADR-0023 addendum): terminal levers don't move the miss
Built A/B/C behind flags (default OFF, byte-identical; gates re-run PASS: M4
pronav 0.447 m, S2 1.734 m + audits). Lab ranked C>B>A; Gazebo paired N=12:
BASE 1.247 → C 1.218 (−0.03, noise) → BC 1.332 (+0.08 WORSE — split-freeze
whipsaw, 6th lab-vs-Gazebo divergence). None move the fast-regime miss →
STRENGTHENS the kinematic diagnosis. Flags retained for reproducibility, all
default OFF. Next lever is Tier-2 (acquisition range), not terminal tweaks.

## Audit prep (builder: Fable 5 will audit)
`docs/audit_targets.md` written — prioritized, refute-not-confirm brief. Top-3:
ADR-0023 diagnosis validity (the linchpin; ZEM causal-interp + ½at² bound
inputs), handoff honesty boundary, frame/sign conventions. Tier-2: lab
byte-identity, small-n stats, the unadopted ADR-0017 σ_R constants in
s2_cue_mock. Tier-3: sourced design numbers + metric-honesty.

## Next build (per ADR-0023/0024/0025, when audit clears)
- [ ] **Tier-2 acquisition range:** narrow SDF `<horizontal_fov>` + raise
  HANDOFF_RANGE → earlier lock → bigger t_go (capacity ∝ t_go²). Re-baseline
  M1/M2 + disclose (reopens ADR-0010 anti-tag-inflation door). The real lever.
- [ ] **Adopt corrected cue constants** (ADR-0017 σ_R c=4.45e-05 + datum-bias
  split) + 0.20 s WORST latency tier (ADR-0016) into s2_cue_mock defaults.
- [ ] **M5 Monte-Carlo** under proximity metric (ram 0.5 m / net 1.5 m curve).

## Remaining before M5 finish line
- [ ] **P-8 adopt corrected sim knobs then re-tune:** stereo σ_R split
  (c=4.45e-05 + --datum-bias-m, ADR-0017) into s2_cue_mock defaults; 0.20 s
  WORST latency stress tier (ADR-0016). These change the cue model → re-run S2
  gate + a Monte-Carlo to confirm before trusting new numbers.
- [ ] **M5 Monte-Carlo:** the real batch (laws × speeds 6/8/10 × paths), Pk-vs-
  radius curves under the corrected+realistic cue, fill README TODO slots.
- [ ] **Demo GIF:** GUI pursuit-vs-pronav side-by-side (Emerson's standing ask).
- [ ] Optional: Gazebo confirm of the jam link-cutoff envelope (lab study in
  flight) once its cliff-edge numbers land.
<!-- ============================================================= -->

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

**Perception design (ACTIVE — builder redirect 2026-07-05: design the perception
half BEFORE finalizing the interception math, since the real track's
rate/accuracy/latency/dropout become new data constraints on guidance).**
DESIGN DONE: 3 Opus councils -> **ADR-0015** + `docs/perception_design.md` (readable).
Headlines: onboard Pi5 + Hailo NPU (real ML, no ROS2 — overturns ADR-0012's
Pi-CPU call; Jetson to the ground); detect-then-track; ground global-frame
GPS-timestamped track fused with onboard bearing (RTK + PPS time-sync the
enablers); EO day-proof, thermal staged.
LAB REALISM STUDY DONE (ADR-0015 addendum, 2026-07-05): all 10 data-constraints
implemented in `guidance_lab.py` behind flags (`--adr0015`), baseline byte-identical
(verifier-confirmed). **The #1 lever is the ground station EMITTING a filtered
velocity, not just position** — differentiate→emit swings mean miss 3.16→1.01 m at
6 m/s, dwarfing every other constraint. Then cue range σ_R∝R² + jammer link-cutoff
timing (~−22 to −26% Pk@2 each). Naive realism is catastrophic (~3.2-3.8 m mean),
emit-velocity recovers most, +mitigations (RTK 0.5 m, emit, no-jam) BEATS the
idealized baseline at 8-10 m/s. PIP only beats pure PN if the cue emits clean
velocity (else they tie). PORTING NOW: the same knobs into s2_cue_mock.py +
m4_intercept.py (Opus worker), then re-run mc_batch.sh under realistic params
("lab ranks, Gazebo decides").
Empirical hook: baseline S2 Pk is gated by TERMINAL PERCEPTION (all 20/20 flights
fail by camera dropout at CPA — ADR-0014 addendum), so better perception directly
raises Pk. Builder's core hypothesis to design around: **ground stereo gives RANGE,
onboard cam gives BEARING, fuse mid-course for a better intercept track; terminal
stays comms-denied (onboard-only).** Bearing-only is range-ambiguous; ground range
resolves the intercept triangle — this is the right architecture. Now empirically
sharpened: the ground link must carry VELOCITY too, or the fused track is no better
than differentiating noise.
- [ ] **P-1 Ground sensor modality:** visual vs IR/thermal vs both/fused — detect a
      small fast FPV drone vs birds/clutter, day+night; detection range, false-alarm
      rate, cost. When is thermal mandatory vs luxury? Radar/acoustic as a cue layer?
- [ ] **P-2 Ground stereo geometry:** camera baseline (separation) vs usable range
      vs 3D triangulation accuracy vs cost/affordability — the "optimal distance to
      cover good range while remaining accurate and affordable" question. Concrete
      numbers: baseline/focal/resolution/#cameras for the accuracy the intercept
      needs. Where's the cost/accuracy knee?
- [ ] **P-3 Compute split (ground vs drone vs hybrid):** where detection, tracking,
      and triangulation run — per-camera edge, central ground box, onboard, or
      hybrid. Latency + cost of each split.
- [ ] **P-4 Ground↔drone comms link:** method/band/protocol to send a compact TARGET
      TRACK (position+velocity, not video) mid-course; bandwidth, INPUT LAG/latency,
      range, jam-resistance. Coexists with the comms-denied terminal (mid-course aid,
      never the terminal seeker).
- [ ] **P-5 Onboard ML chip:** what actually runs real-time small-drone ML detection
      at useful frame rate+latency on an FPV airframe — Jetson Orin Nano/NX vs
      Pi5+Hailo-8/8L vs Pi5+Coral vs other 2026 NPUs. Power/weight/thermal/$.
      REVISIT ADR-0012 (chose Pi5 CPU, rejected Jetson on no-ROS-2 — but that was for
      a FIDUCIAL; real ML may force an NPU/GPU and reopen that call).
- [ ] **P-6 Sensor-fusion mechanization:** onboard bearing + ground range — frames
      (ENU/NED/FRD/camera), time-sync, EKF, latency compensation (reuse the built
      cue-latency-comp idea). Resulting track quality; graceful degradation if the
      link drops or ground loses the target before onboard acquires.
- [ ] **P-7 End-to-end latency budget:** detection→triangulation→comms→onboard
      fusion→guidance; tie to the Pk/detection-quality coupling (ADR-0014 addendum).
- [ ] **P-8 Data constraints BACK to the guidance math (builder's central point):**
      the real fused track's update rate, position/velocity sigma, latency mean+jitter,
      and dropout model — the params to upgrade s2_cue_mock.py / the detector model
      with, and re-validate guidance against, BEFORE finalizing the intercept tuning.
- [ ] **P-9 (stretch, parent-project) real terminal seeker + real ground rig:** the
      actual detectors behind P-1..P-6 (classical CV / tiny CNN onboard; stereo+EKF
      on the ground). GOALS.md forbids ML perception IN THE SIM — these are the
      hardware/parent-project build lines; the sim models their OUTPUT (track quality)
      via P-8's upgraded mock, it does not run the detectors.
- [ ] **P-0 README honesty note:** the AprilTag is a STAND-IN for "a reliable target
      lock exists"; a real hostile FPV carries no fiducial. What transfers is the
      GUIDANCE loop (bearing/LOS-rate→pro-nav→PX4), agnostic to how bearing is
      produced. Pro-nav is bearing-only-friendly (needs LOS *rate* = an angle) — the
      structural reason the two-sensor split works.

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
