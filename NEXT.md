# NEXT — top of the stack

*(One CURRENT section, one ordered build queue, one compressed Done list. Detail
lives in `docs/decisions.md` (ADRs) and `PROGRESS.md` (roll-up). Restructured
2026-07-06 during the Fable audit — the old file had three generations of roadmap
interleaved.)*

## Current: M5 FINISH — the protected finish line (builder ratified 2026-07-07, ADR-0033)

Scope is pinned in ADR-0033. Order below; sims ONE at a time at idle load; paired
seeds; `--early-handoff`; speeds **6/9/12** (ADR-0029 comparability — supersedes
the older 6/8/10 note). Demo video is DONE (ADR-0032); running-start Gazebo
confirm is DONE (ADR-0028 addendum); streak=2 held the 6 m/s S2 gate (ADR-0029c).

1. **Build (parallel, code-only, no sims):**
   a. **P-8 corrected cue constants → s2_cue_mock defaults** (σ_R c=4.45e-05 not
      0.008·R², separate `--datum-bias-m` — ADR-0017; + 0.20 s WORST latency
      stress tier — ADR-0016). Changes the cue model → S2 gate must re-earn.
   b. **Path suite:** S-weave + jink as sim-time-scheduled VELOCITY schedules in
      m4_target_mover (board face stays world −X; maneuvers are velocity-schedule
      changes only — ADR-0010 #6; jink schedule must derive from the run seed for
      paired A/B) + a **west-approach geometry** (east/world_x axis has never been
      mirror-tested; also the L2R/R2L asymmetry second batch, by construction).
      Wire through mc_batch.sh.
   c. **Fix the ADR-0032 pre-placement race in m4_intercept.py itself** — internal
      tag pre-place via a SUBPROCESS `gz service` call (NOT in-process: the script
      holds a /clock subscription and is safe only because it makes no gz service
      calls itself — see Key facts). README reproduce instructions expose this
      footgun to outsiders, so it's in-scope for M5.
   d. **Plot/analysis upgrades:** trajectory overlays, miss histogram/CDF,
      per-path pursuit-vs-pronav; keep Pk-vs-radius (ADR-0025 metric, ram 0.5 m /
      net 1.5 m). Dev against the existing ADR-0029 logs — no new sims needed.
2. **Gate + dev-verify (sequential sims, boot-budget discipline):** weave/jink/west
   mover dev flights (share boots where possible), then re-run `check_s2.sh` under
   the new cue defaults.
3. **Final batch (design as a short ADR first, then overnight at idle):**
   {pursuit, pronav} × {6, 9, 12 m/s} × paths {L→R, R→L, west, weave, jink} on the
   ADOPTED deployment profile (running start + velocity-emission cue +
   `--dash-unclamp` + `--early-handoff`, realistic degraded cue, corrected
   constants), n≥8 paired. Outputs: miss-vs-intensity, per-path law comparison,
   Pk-vs-radius.
4. **Analyze + ship:** plots → README final numbers + GIF (from demo_out) +
   reproduce instructions → verifier gate → commit.

## Build queue (post-M5 — builder ratified 2026-07-07, ADR-0033)

1. **Hardware Stage 0 bench (ADR-0012), ~$230.** Pi 5 + camera detecting a printed
   tag with the REAL detection code (`frame_source.py` port path), measured frame
   rate + pose error → a sim-vs-bench gap table. **BUILDER ACTION: order parts now**
   (Pi 5 8GB ~$120, cooler/PSU/SD ~$35, global-shutter cam + wide lens ~$75) —
   lead time overlaps the M5 finish; software prep is pre-staged.
2. **Kill the AprilTag (markerless seeker).** Markerless seeker on a tag-less
   target feeding the SAME bearing interface. Candidates (builder widened scope
   2026-07-07): classical CV (motion/blob/KLT) AND a pre-built lightweight neural
   detector needing minimal adaptation (nano-class, drone-fine-tuned) — the
   parent architecture (ADR-0015, Pi5+Hailo detect-then-track) already assumes an
   ML detector, so this is architecture-consistent, and GOALS.md's "no ML" rule
   was about isolating guidance, which this milestone deliberately un-isolates.
   Partial result acceptable. Design-as-ADR first; research brief in progress
   (`docs/seeker_design_brief.md`).
3. **EKF target-track A/B.** Proper EKF vs the alpha-beta baseline, paired seeds
   n≥8, honest null-result framing (Kalata's Gazebo rejection, ADR-0013, is the
   precedent). Pure software — may interleave with 1–2.
4. **CAPSTONE — covariance-gated mid-course FUSION (ADR-0034, the path forward as
   we drop the AprilTag).** Re-open the ADR-0018 `--fuse-midcourse` null (it was
   measured under a clean tag + fixed-gain αβ — the two conditions that suppress
   fusion) NOW that the seeker is markerless (noisier camera → cue carries info
   again) and the tracker is an EKF (weights sources by live covariance; the
   innovation gate IS the "fall back to camera when the ground track is worse"
   logic, for free). This is NOT a 4th thread — it's where items 2+3 MEET.
   Expected payoff is mid-course robustness (handoff-reach / fewer dash-aborts,
   the ADR-0030 binding failure), NOT terminal miss (kinematic ceiling ADR-0023).
   HARD honesty constraint: fusion stays MID-COURSE, terminal stays camera-only,
   and the `cue_reads_post_handoff=0` audit must extend to "no cue-tainted EKF
   state/covariance survives handoff" (else the jam-resistance thesis breaks
   through the filter's memory). Ladder up fusion{off,on}×tracker{αβ,EKF}×
   seeker{tag,markerless}, don't run all 8. Design-as-ADR before building.

## Follow-up flagged (not yet built)

- **`m4_intercept.py` target pre-placement race (ADR-0032):** promoted into the
  M5 finish scope above (step 1c) — no longer just flagged.
- **`compose_demo.sh`/`demo_capture_frames.py` time-alignment.** No automatic
  correction if a chase/onboard capture starts before the flight CSV's own
  `t_sim[0]` — desyncs the composite by the head-start (see ADR-0032, "Sync
  gotcha"). A `--sim-t-start` trim-to-match step would remove the manual fix
  needed this session.

## Parked (designed, not scheduled — do not build before M5 ships)

- Deployment-profile phases M-1..M-4 (ground standby → launch-on-detect →
  climb-out dash → end-to-end timeline). Design-as-ADR first when picked up.
- P-9 real seeker / real ground rig (parent-project hardware lines; sim models
  their OUTPUT only). Jam-envelope Gazebo confirmation (lab study done, ADR-0020).
- *(Hardware Stage 0 bench PROMOTED to the post-M5 build queue above, ADR-0033.)*

## Done (newest first — one line each; the ADR holds the story)

- **2026-07-07 portfolio demo video DONE:** hero flight captured (miss=1.061 m,
  clean, `--early-handoff` + ADR-0030 FIX config under the realistic degraded
  cue), FPV interceptor reskin verified number-safe (check_m2 unchanged),
  chase-cam pose data-grounded + iterated live, HUD+chase composite produced.
  Found + worked around a real pre-existing `m4_intercept.py` target
  pre-placement race (see "Follow-up flagged" above). (ADR-0032,
  `demo_out/README.md`)
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
