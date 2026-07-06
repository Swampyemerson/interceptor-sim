# NEXT — top of the stack

*(One CURRENT section, one ordered build queue, one compressed Done list. Detail
lives in `docs/decisions.md` (ADRs) and `PROGRESS.md` (roll-up). Restructured
2026-07-06 during the Fable audit — the old file had three generations of roadmap
interleaved.)*

## Current: Fable 5 audit (2026-07-06) — then the build queue below

`docs/audit_targets.md` is the brief (refute-not-confirm). Audit covers: handoff
honesty boundary, frame/sign conventions, ADR-0023 diagnosis validity (the
linchpin), lab byte-identity, small-n stats, cue-mock σ_R gap, sourced numbers,
metric honesty, repo hygiene, plus one end-to-end gate reproduction. Results get
appended to decisions.md as an audit ADR; anything REFUTED reorders the queue.

## Build queue (ordered, per ADR-0023/0024/0025)

1. **✅ Tier-2 acquisition range — CONCLUDED (ADR-0024 3rd addendum).** Full FOV/streak
   sweep done in Gazebo: **60° FOV narrowing REJECTED** (0/12 latch at 9 m/s — can't
   hold a fast crosser; 7th lab-vs-Gazebo divergence, reverted from the tree).
   **streak-min=2 (`--early-handoff`) ADOPTED for the fast/FPV regime** — ~doubled the
   9 m/s latch (42%→88%), but the miss stays kinematic (~3.3 m; 3rd reconfirmation of
   ADR-0023). Net: use `--early-handoff` for fast-regime M5 batches; the 9 m/s *miss*
   is a proximity-metric problem (ADR-0025), not a sensing one. Still open: an S2-gate
   re-check at streak=2 before making it the global default.
2. **Adopt corrected cue constants (P-8).** ADR-0017's stereo split — σ_R
   c=4.45e-05 (not 0.008·R²) + separate `--datum-bias-m` — into s2_cue_mock
   defaults, plus the 0.20 s WORST latency stress tier (ADR-0016). Changes the cue
   model → re-run S2 gate + a confirmation batch before trusting new numbers.
2b. **Audit follow-ups (ADR-0026/0027).**
   - **✅ Second-speed forensic batch DONE (ADR-0027):** kinematic diagnosis
     GENERALIZES (r² 0.818/0.957/0.994 at 3/6/9 m/s, tracks capacity/ZEM ratio) →
     ADR-0023 upgraded to HOLDS. New finding: 9 m/s handoff-latch reliability (5/12) —
     folded into Tier-2 above.
   - **L2R vs R2L asymmetry:** a real ~0.6 m mirror asymmetry (p≈0.01, single batch)
     is most likely the fixed-tag-aspect perception effect, not a sign bug — confirm
     with a second paired batch + write it up, or find the guidance/acquisition term
     if it isn't the tag aspect. The east/world_x axis has NEVER been mirror-tested
     (target start_x never flipped) — add a west-approach geometry to the M5 suite.
3. **S3/S4 folded into M5's Monte-Carlo (proposal, logged here 2026-07-06).**
   ADR-0010's S3 (path suite) and S4 (adaptation proof) merge into the M5 batch:
   mc_batch over {pursuit, pronav} × speeds {6, 8, 10} × paths {crossing L→R, R→L,
   S-weave, jink} through the dash+handoff pipeline, sim-time-scheduled mover
   (verify sim-time scheduling composes with DISCONTINUOUS velocity changes; board
   face stays world −X, so maneuvers are velocity-schedule changes only —
   ADR-0010 #6). Outputs: miss-vs-intensity curve, per-path pursuit-vs-pronav,
   Pk-vs-radius curves under the ADR-0025 proximity metric (ram 0.5 m / net 1.5 m).
4. **M5 finish + the DEMO (the protected finish line).** Trajectory overlays + miss
   histograms/CDF (matplotlib → plots/), README final numbers, reproduce
   instructions. **The demo video + portfolio packaging is now fully planned in
   `docs/demo_plan.md`** (data-driven glass-cockpit HUD from the CSV, 9-beat
   storyboard, honest kill depiction, 5-part portfolio package, ordered TODO).
   Ungated demo tooling (ffmpeg install, `t_sim` CSV column, `render_hud.py`, the
   kill graphic, WRITEUP skeleton) can be built NOW; the hero-take flight + S3
   maneuvering mover are gated on guidance being final (which it now largely is —
   Tier-2 concluded). Batches at idle load, paired seeds, `--early-handoff` for the
   fast regime.

## Parked (designed, not scheduled — do not build before M5 ships)

- Deployment-profile phases M-1..M-4 (ground standby → launch-on-detect →
  climb-out dash → end-to-end timeline). Design-as-ADR first when picked up.
- P-9 real seeker / real ground rig (parent-project hardware lines; sim models
  their OUTPUT only). Jam-envelope Gazebo confirmation (lab study done, ADR-0020).
- Hardware bring-up per ADR-0012 Stage 0 (bench perception ~$200) — blocked on
  builder buying parts; software port-gaps are pre-staged (FrameSource,
  pyapriltags fallback, calibrate_camera.py).

## Done (newest first — one line each; the ADR holds the story)

- **2026-07-06 Tier-1 terminal levers:** A/B/C flags built, gates PASS; none move
  the fast-regime miss in Gazebo (paired N=12) → strengthens the kinematic
  diagnosis; 6th lab-vs-Gazebo divergence. (ADR-0023 addendum)
- **2026-07-06 audit brief:** `docs/audit_targets.md` written for this audit.
- **2026-07-06 proximity metric RATIFIED** by builder: Pk-vs-radius headline
  (ram ~0.5 m slow regime, net ~1.5 m fast); M0–M4 gates unchanged. (ADR-0025)
- **2026-07-06 seeker upgrades reversed:** narrower/longer acquisition lens, not
  wide-FOV terminal hold. (ADR-0024)
- **2026-07-06 terminal diagnosis (THE LINCHPIN):** miss is kinematic — ZEM-vs-miss
  r²=0.96 at handoff / 0.99 at freeze (n=41); detection persists to ~1.7 m /
  0.07 s pre-CPA; blind window adds −0.03 m; corrects ADR-0014's
  perception-limited reading. (ADR-0023, `docs/terminal_diagnosis.md`)
- **2026-07-06 kill mechanism:** ram/net; a bigger warhead can't buy out the
  perception gap (prox fuze needs the same detection). (ADR-0021)
- **2026-07-06 jam envelope (lab):** target maneuver sets the coast margin —
  straight targets coastable tens of m, jinking collapses to ~2 m. (ADR-0020)
- **2026-07-06 ground modality (P-1):** staged thermal holds w/ corrections;
  EO-only honest envelope ~60–160 m day, blind at night. (ADR-0019)
- **2026-07-06 fusion Gazebo A/B:** lab's coverage win did NOT transfer (5th
  divergence); `--fuse-midcourse` kept default-OFF. (ADR-0018 addendum)
- **2026-07-05 fusion design (P-6):** bearing-weighted polar fusion + warm handoff,
  default-OFF, byte-identical baseline. (ADR-0018)
- **2026-07-05 stereo rig design (P-2):** AR0234×2 + 16 mm, 2.0 m baseline knee;
  σ_R mock correction found (c=4.45e-05, ~180× shallower than mock). (ADR-0017)
- **2026-07-05 compute setup (P-3/5/7):** hybrid split, 90-byte track link, 3-tier
  latency budget — sim's cue latency survives. (ADR-0016)
- **2026-07-05 perception architecture:** Pi5+Hailo onboard, detect-then-track,
  ground global-frame track + onboard bearing fusion. (ADR-0015 + addenda;
  emit-velocity = #1 lever, direction confirmed in Gazebo)
- **2026-07-05 Pk study + methodology mandate:** 3-tier BEST/EXPECTED/WORST,
  survive-WORST, bench-measurable knobs, idle-load batches. (builder directive)
- **2026-07-05 M4.5-S2 (gated):** two-stage cue→dash→camera-only handoff; 6 m/s
  crosser ~1.1–2.3 m (uncatchable from hover); honesty audits pass; Kalata
  rejected in Gazebo. (ADR-0013, `scripts/check_s2.sh`)
- **2026-07-05 M4.5-S1:** `--fpv` profile; PN N=5 0.94 m vs 3 m/s from hover;
  hover-start capped ~3 m/s → S2 dash is the enabler. (ADR-0011 + addenda; PIP
  camera-only = documented negative result)
- **2026-07-05 hardware stack decided:** Pixhawk 6C + Pi5 + global-shutter cam,
  X500 bring-up → 7" deployable; staged Stage 0–3 plan. (ADR-0012)
- **2026-07-05 M4 (gated):** pro-nav 0.28–0.44 m vs pursuit 2.0–2.5 m, camera-only,
  no-cheat verified; the RTF/sim-time saga. (ADR-0009 + addenda)
- **2026-07-05 M3 (gated):** camera-only 2 m standoff, err 0.018/0.035 m. (ADR-0008)
- **2026-07-04 M0/M1/M2 (gated):** toolchain; 1280×960 frames; tag detection
  err 0.086 m @ ~4.9 m. (ADR-0001..0007)

## Key facts for a fresh session

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
- Ground truth comes from `/world/apriltag/pose/info` (gz.msgs.Pose_V) — see
  scripts/m2_detect.py's docstring and ADR-0006 for the transform-chain gotcha
  (camera_link's pose is relative to the model directly, NOT to base_link,
  despite a numeric coincidence suggesting otherwise). gt_* is SCORING ONLY.
- venv sees system gz bindings via `.venv/.../site-packages/gz_system.pth`
  (python3-gz-transport13 + python3-gz-msgs10 from apt). No scipy — quaternion ->
  rotation matrix is hand-rolled in scripts/m2_detect.py (`quat_to_matrix`),
  reused rather than re-implemented for frame math.
- Boot-complete grep: "Startup script returned successfully" (ADR-0004 — never wait
  on "Ready for takeoff!" pre-MAVSDK).
- MAVSDK: `udpin://0.0.0.0:14540`. AprilTag lib: pupil-apriltags (ADR-0003;
  pyapriltags is the aarch64 drop-in, import is swappable — ADR-0012).
- S2 runtime facts: cue mock streams UDP JSON on 127.0.0.1:47800 (sim-time
  scheduled); `--handoff` requires `--fpv`; S2 geometry default = target start
  (6.5,-14,0.5), vel (0,6); `check_s2.sh [pip|pronav]` runs a single law for dev.
  m4_intercept.py holds a /clock subscription (SimClockHolder) — safe because
  it makes no gz service calls (only the mover does, in its own process).
- A/B hygiene: miss variance across identical flights is ~1 m (terminal dropout
  timing). Never conclude from a single flight; paired seeds n≥8 + mechanism
  evidence. Batches at idle machine load only. World→NED: north=world_y,
  east=world_x (ADR-0013).
- Minor: m0_takeoff.py duplicates its final CSV row — tidy if reused as template.
