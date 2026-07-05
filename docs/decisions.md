# Decisions log (ADR-lite)

Format: context / options / decision / why / date. Councils noted where convened.

---

## ADR-0001 — PX4-Autopilot lives outside the project repo
- **Context:** PX4 is a ~2 GB clone with its own git history and build tree.
- **Options:** (a) clone inside `interceptor-sim/` and gitignore it; (b) clone to `~/PX4-Autopilot`.
- **Decision:** (b) `~/PX4-Autopilot`.
- **Why:** keeps this portfolio repo small and clean; no risk of accidentally committing build artifacts; standard PX4 dev layout.
- **Date:** 2026-07-04. (No council — reversible.)

## ADR-0004 — M0 gate waits on PX4's boot-complete line, not "Ready for takeoff!"
- **Context:** M0 gate deadlocked twice: gz_x500 airframes set `NAV_DLL_ACT=2` (datalink-loss failsafe), making "No connection to the GCS" a blocking preflight failure — PX4 only prints "Ready for takeoff!" after a GCS/MAVSDK link exists, but our gate started MAVSDK only after seeing that line. Root-caused by the verifier in PX4 source (Commander.cpp / rcAndDataLinkCheck.cpp / airframe 4001).
- **Options:** (a) set `NAV_DLL_ACT 0` in SITL; (b) gate on "Startup script returned successfully" and let the MAVSDK connection clear the preflight check.
- **Decision:** (b). Flight-safety parameters stay at airframe defaults; tests adapt to the system, not the reverse.
- **Why:** keeping the failsafe honest preserves the "no cheating" credibility of the sim (GOALS.md); the boot-complete line is unconditional and appeared in every log. Verified: after the fix, "Ready for takeoff!" appears immediately after MAVSDK's connection lands — the causal story confirmed end-to-end.
- **Date:** 2026-07-04.

## ADR-0003 — AprilTag library: pupil-apriltags (COUNCIL, unanimous 3-0)
- **Context:** The tag detector feeds every milestone from M2 on (detection → pose → guidance). Swapping later means re-validating the whole perception chain — a one-way door per CLAUDE.md, so a 3-member Sonnet council was convened with an identical brief.
- **Options:** (A) pupil-apriltags, (B) dt-apriltags, (C) cv2.aruco + solvePnP with the 36h11 dictionary.
- **Decision:** (A) **pupil-apriltags**, version pinned in requirements.
- **Why (council synthesis, all three voted A at high confidence):**
  - Actively maintained; 1.0.4.post11 (2025-04) ships cp312 manylinux wheels — one member verified the wheel downloads cleanly on this exact machine.
  - Wraps the genuine AprilTag 3 reference algorithm (not ArUco's reimplementation) and exposes `estimate_tag_pose` returning `pose_R`/`pose_t`/`pose_err` directly — no hand-rolled solvePnP, one less coordinate-convention surface to get wrong, and `pose_err` gives us a loggable per-detection accuracy signal.
  - (B) is dead: last release 2021, no Python 3.12 wheels. (C) works but adds pose-ambiguity pitfalls (`SOLVEPNP_IPPE_SQUARE` needed) and more of our own geometry code.
- **Risks & fallbacks (logged from council dissent/caveats):** small maintainer team — pin the version; if wheels ever break, the `pyapriltags` fork is the drop-in fallback. If M2 shows unacceptable pose jitter at oblique angles (inherent monocular planar-tag ambiguity), revisit (C) cv2.aruco + `SOLVEPNP_IPPE_SQUARE` as the swap — the detector interface will be isolated behind one module to keep that cheap.
- **Date:** 2026-07-04.

## ADR-0002 — Environment baseline: WSL2 + RTX 4070, headless by default
- **Context:** KICKOFF-PROMPT.md described a "fresh VM, no GPU"; CLAUDE.md (newer) and `nvidia-smi` confirm WSL2 with an RTX 4070.
- **Decision:** Trust the live environment: GPU available for camera rendering (M1+) and the demo; all automated/batch runs stay `HEADLESS=1` for speed and reproducibility.
- **Why:** headless runs are reproducible and faster; the GPU only matters where pixels matter.
- **Date:** 2026-07-04. (No council — observation, not a fork.)

## ADR-0005 — M2 custom world resolves by symlink into PX4's worlds dir
- **Context:** `worlds/apriltag.sdf` must live in this repo (task requirement), but PX4's `px4-rc.gzsim` resolves world files as `${PX4_GZ_WORLDS}/<name>.sdf`, and the runtime `gz_env.sh` (sourced from the SITL working dir on every launch) unconditionally overwrites `PX4_GZ_WORLDS` to PX4's own `Tools/simulation/gz/worlds` directory — exporting our own value first does not survive, since that assignment (unlike `GZ_SIM_RESOURCE_PATH`, which is appended to) is a plain overwrite.
- **Options:** (a) symlink `~/PX4-Autopilot/Tools/simulation/gz/worlds/apriltag.sdf` -> the repo copy; (b) duplicate the file inside PX4's tree; (c) patch PX4's `gz_env.sh`.
- **Decision:** (a) symlink, created/repaired by `scripts/check_m2.sh` on every run.
- **Why:** keeps the repo copy as the single source of truth (satisfies the task's "kept in the project repo" requirement) with zero duplication risk; (b) would drift; (c) edits PX4 itself, against ADR-0001's spirit of keeping PX4 untouched. Models resolve differently and need no such workaround: `GZ_SIM_RESOURCE_PATH=/home/emerson/interceptor-sim/models` set before launch IS preserved, because `gz_env.sh` appends to whatever is already set (`export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS`).
- **Date:** 2026-07-04. (No council — reversible, mechanical.)

## ADR-0006 — M2 ground-truth chain: camera_link composes directly against the model, not via base_link
- **Context:** First draft of `scripts/m2_detect.py` assumed `/world/apriltag/pose/info`'s "camera_link" entry was relative to "base_link" (stacking base_link's own (0,0,0.24) model-relative offset underneath camera_link's (0.12,0.03,0.242)), because both that guess and the correct answer print the identical camera_link number — a coincidence of this specific SDF (the CameraJoint's `<pose relative_to="base_link">` in `x500_mono_cam/model.sdf` happens to numerically equal the `<include><pose>` that actually places camera_link relative to the enclosing model). The wrong chain placed the simulated camera ~0.24 m too high, producing a constant (not noisy) ~0.25 m bias, concentrated in optical-Y, against the detector's own measurement across every one of ~340 static frames — the repeatability was the tell that this was a transform bug, not sensor noise.
- **Root cause (traced to the SDF, not just live numbers):** `merge='true'` includes flatten every sub-model's links into one flat sibling list directly under the top-level model; each link's declared `<pose>` (from its own model's top-level `<pose>` element, e.g. `x500_base`'s `<pose>0 0 .24 0 0 0</pose>`, or an include's own `<pose>` override, e.g. mono_cam's `.12 .03 .242`) is relative to that enclosing model — not nested link-to-link. The CameraJoint's `relative_to="base_link"` is a physics/constraint-frame declaration for an already-fixed, immovable joint; it does not re-parent camera_link's reported spatial pose.
- **Decision:** ground truth composes `T_world_cam = T_world_model ∘ T_model_camera_link` directly (base_link excluded from this chain, though still read from the pose topic and available for future milestones needing the vehicle body frame).
- **Why it matters beyond this bug:** a "same number, different assumed parent" coincidence like this can bite twice — logging it so a future edit to camera mount geometry (a real possibility before M3/M4) doesn't reintroduce the same wrong assumption.
- **Verification:** after the fix, mean position error dropped from 0.2596 m (FAIL) to 0.0861 m (PASS, threshold 0.25 m) at 5 m range, with the residual concentrated in range/Z (consistent with a small, expected sub-pixel corner-detection inset from GPU rendering/antialiasing at long range, not a further transform error — X and Y matched ground truth to <5 mm).
- **Date:** 2026-07-04.

## ADR-0007 — M2 AprilTag material: emissive_map, not lit albedo_map alone
- **Context:** The target board faces world -X (so the drone's forward camera sees it), but `worlds/apriltag.sdf`'s sun direction is kept exactly as PX4's `default.sdf` (task requirement — do not edit) at `~(0, 0.625, -0.78)`, nearly straight down. A -X-facing vertical plane gets a near-grazing/near-zero dot product with that light, so ordinary lit PBR shading (even with `<lighting>false</lighting>`, which this gz-sim/ogre2 build's `pbr/metal` path does not honor) rendered the tag's "white" cells at only ~100/255. That was enough contrast to *look* fine by eye, but GPU mip-minification of the small on-screen data-bit cells at range degraded contrast further, and pupil-apriltags detected at 1.2/1.5/2.0 m but failed at 2.5/3.0/5.0 m.
- **Options considered:** (a) move the sun (rejected — task explicitly requires keeping the world's sun/physics/ground exactly as PX4's default.sdf); (b) flat `<emissive>1 1 1 1</emissive>` (tried first — washes the whole plane to solid white, since flat emissive color adds uniformly on top of the albedo map rather than reproducing its pattern); (c) `<emissive_map>` using the same tag texture.
- **Decision:** (c). `<emissive_map>model://apriltag_target/tag36h11_00000.png</emissive_map>` alongside the existing `<albedo_map>`, with `<ambient>`/`<diffuse>` dropped to 0 (emissive alone now carries the crisp black/white pattern, independent of scene lighting/viewing angle).
- **Why:** emissive is additive self-illumination; as a *map* rather than a flat color it reproduces the actual black/white bit pattern at full contrast (0–255) regardless of sun angle or range, fixing detection out to at least 6 m (tested) while leaving the required world lighting untouched.
- **Date:** 2026-07-04.

## ADR-0008 — M3 control interface: body-frame velocity setpoints, camera-only target feedback, 3D standoff
- **Context:** M3 needs the first closed guidance loop: AprilTag detections (camera optical frame) must become PX4 OFFBOARD commands. Choices: command frame (body FRD vs NED), what feedback the controller may use, and what "2 m standoff" means precisely.
- **Options:** command frame — (a) `VelocityBodyYawspeed` (body FRD + yawspeed) vs (b) `VelocityNedYaw` (world frame, needs heading); standoff metric — 3D camera→tag distance vs horizontal-only.
- **Decision:** (a) body-frame velocity + yawspeed; P-control: forward = 0.5·(range−2.0) clipped [−0.5, +1.0] m/s, yawspeed = 1.5°/s per degree of bearing (clip ±30°/s), altitude its own P-loop to 1.0 m off EKF `relative_altitude_m`; standoff = **3D** camera→tag range (rotation-invariant, exactly what `pose_t` measures). Feedback split: target position from the camera ONLY; own-state (altitude, mode) from PX4's EKF; ground truth from the pose topic logged every tick but never read by the control law.
- **Why:** the measurement is inherently body-relative — body-frame commands need no compass/heading conversion (one fewer sign-error surface, the ADR-0006 lesson); the feedback split mirrors the parent project's comms-denied terminal seeker story (GOALS.md) and keeps the result honest. Gains chosen gentle-first and validated by run rather than tuned to the edge.
- **Verification:** `scripts/check_m3.sh` — three runs (dev + 2 verifier): final standoff error 0.040 / 0.018 / 0.035 m vs the 0.5 m gate, detection coverage 1.000, no overshoot (min gt_range ≥ 1.958 m), settle in ~10 s (`logs/m3_intercept_20260705T00{0424,0619,0818}Z.csv`). Verifier numerically confirmed commands derive from measured range, not ground truth (the two diverge up to 0.063 m/s at specific ticks; commands matched the camera side exactly).
- **Date:** 2026-07-05. (No council — GOALS.md already fixed "velocity setpoints first"; the rest is reversible and run-validated. The M4 pro-nav mechanization + gain WILL get a council per CLAUDE.md.)

## ADR-0009 — M4 pro-nav mechanization, gain, geometry (COUNCIL, 3 members)
- **Context:** M4 implements the resume-headline comparison: pursuit vs proportional navigation against a crossing target, camera-only. Pro-nav (a = N·Vc·λ̇) needs the LOS rate λ̇ and closing speed Vc mechanized from a body-fixed camera whose yaw actively tracks the tag. High-stakes/costly-to-redo → council convened per CLAUDE.md.
- **Council: unanimous on the core.** (1) Raw d(bearing)/dt is silently near-zero because the yaw loop nulls bearing by construction — λ̇ MUST be reconstructed in a yaw-compensated frame, λ = ψ(own EKF yaw) + β(camera bearing), the standard strapdown-seeker correction. Own-state is legal under the no-cheating rule; target ground truth stays logging-only. (2) 2D horizontal pro-nav; altitude keeps M3's independent P-loop. (3) Vc = −(filtered range-rate) from the camera range, floored positive. (4) Both laws share the SAME actuation (VelocityBodyYawspeed), same yaw/altitude/closing loops — the lateral term is the only independent variable (fair comparison). (5) Crossing geometry (head-on is undiagnostic: λ̇≈0 makes the laws indistinguishable). (6) Miss distance = min ground-truth 3D camera→tag over the whole run, computed from the full log, independent of the camera-only breakoff logic. (7) Verifier must repeat M3's numeric no-cheat check.
- **Split decisions (Fable's calls):** filter — alpha-beta (g-h) on λ and on range (predict at 20 Hz, correct on fresh detections, wrap residuals; α=0.5, β=0.15) over raw finite-diff + IIR (2 of 3 members; handles dropouts/irregular dt natively). N — members said 3 / 3.5 / 4; starting **N=4** (margin for filter lag against a constant-velocity target where any N≥3 is theoretically adequate), sweep {3,4,5} only if the first runs chatter or lag. Geometry — target from (6.5, −4, 0.5) crossing at +2.5 m/s along Y, tag facing −X throughout, interceptor from the usual origin; initial range ~7.6 m slightly exceeds M2's validated 6 m detection envelope — ACQUIRE phase gates on real detections before engaging, so an envelope problem fails loudly, early. Yaw cap raised 30→60 °/s for BOTH laws (at 2.5 m/s crossing the endgame LOS rate blows past 30 °/s; per council member A, a test where both laws saturate yaw is undiagnostic, not evidence of pursuit being worse).
- **New for M4 (design implications logged):** ALT_REF lowered 1.0→0.5 m so the camera-to-tag vertical offset (~0.25 m) doesn't consume the <1 m 3D miss budget. Terminal rules: v_perp integration freezes inside 1.5 m (λ̇ singular as R→0); inside 3 m a detection dropout holds the last command ≤1 s (endgame FOV loss is pursuit's EXPECTED failure mode — logged honestly, not papered over), outside 3 m dropouts hover-hold as in M3. Breakoff camera-only: measured range increasing on 3 consecutive fresh detections after dipping <2.5 m, or <0.5 m hard floor; then climb. Tag board has no collision element (checked) — a flythrough is physically harmless, breakoff kept for realism.
- **Infrastructure fact (bit us in prototyping):** gz-transport13 Python — a process with ANY topic subscription never receives service responses (requests still apply server-side). Target mover is therefore its own subscription-free process streaming /world/apriltag/set_pose at 50 Hz (measured: 194 requests, 0 failures, median 0.9 ms once discovery is warm).
- **Council dissent/warnings adopted:** bench-test the sign convention before trusting runs (spin in place vs static tag → λ̇≈0; `--bench` mode built into m4_intercept.py); don't soften the geometry to hide pursuit's endgame FOV loss; verifier reruns the numeric divergence check on λ̇/Vc inputs.
- **Date:** 2026-07-05. (Council: 3× council-member subagents, briefs identical, independent; synthesis by main session.)

## ADR-0009 addendum (2026-07-05, same session) — run-validated tuning deltas
Five dev iterations changed these from the council baseline (rationale in
scripts/m4_intercept.py comments + NEXT.md debugging trail; all changes shared
by BOTH laws, preserving the fairness design): yaw loop gained LOS-rate
feedforward (yawspeed = λ̇_hat + 3.0·β, was pure P at 1.5) after the P-loop
lagged a rotating LOS into FOV loss; BETA_GAIN split per channel (λ 0.30,
range 0.15); Vc floor 0.3→1.5 m/s (hover-start range-rate starved the lead);
V_PERP_MAX 2→3 m/s (saturated); ACQUIRE releases only after 6 consecutive
centered detections (mid-slew engage started the chase in a hole); target
speed 2.5→2.0 m/s per council member A's pre-registered criterion (yaw pinned
at cap = undiagnostic geometry — NOT to soften pursuit's expected failure).
--bench PASSED (mean |λ̇| 1.24°/s under ±20°/s spin). Best pro-nav dev run:
miss 0.945 m, clean, coverage 0.764 (logs/m4_intercept_pronav_20260705T013540Z.csv).
Official check_m4.sh gate + verifier still pending — see NEXT.md.

## ADR-0009 second addendum (2026-07-05) — the RTF discovery and the final M4 mechanization
- **Root cause of the stalemate chases (3 dev runs + 2 failed gates):** under full
  mission load this sim's real-time factor sags to ~0.5, and the target mover
  scheduled its path in WALL time — making a "2.0 m/s" target move ~4 m/s in SIM
  terms, faster than the interceptor's 4.0 m/s ceiling. Every engagement
  degenerated into a matched-speed tail chase by construction. The tell: PX4's own
  ulog showed near-perfect setpoint tracking (cmd (3.90,-0.89) → achieved
  (4.02,-0.88)) over a "fast phase" of 8.8 SIM seconds that our wall-clock CSV
  recorded as 18.6 s — exactly the RTF ratio. **Fix:** the mover schedules on sim
  time from a /clock-subscribing CHILD process (the mover itself must stay
  subscription-free per the service-response quirk); --duration is now sim-seconds.
- **Command path switched to `set_velocity_ned` + absolute yaw setpoint** (was
  body-frame + yawspeed; supersedes ADR-0008's frame choice for M4 only): guidance
  already computes a world-frame vector, body-frame velocity tracking measurably
  degrades at speed, and yaw-angle setpoints eliminate the entire LOS-rate
  feedforward/rate-lag problem (PX4's attitude loop does the pointing). Camera
  remains the only target sensor; ψ was already required for the strapdown λ.
- **Terminal coast:** at r_hat < 2.0 m the full commanded velocity vector freezes
  and the vehicle flies the established collision course through CPA (λ̇ is
  singular as R→0; at 1.5 m the estimate had already blown to -53°/s vs -13°/s at
  2.0 m, whipping the commanded direction and un-flying the intercept).
- **Constant closing speed 3.0 m/s during ENGAGE** (was 0.8·R, an M3 standoff
  habit): a proportional-to-range speed law let the target outrun the interceptor
  inside 2.5 m — CPA offset was 1.01 m *along the target's velocity* with only
  0.10 m cross-track. Intercept ≠ rendezvous.
- **Detector quad_decimate=2 for M4 only** (M3 keeps full-res; its gate numbers
  were validated without decimation). ~4× cheaper quad search, corner refinement
  still full-res; ENGAGE coverage rose ~0.5 → 0.75-0.85.
- **Gate criteria made asymmetric by design:** pro-nav must be clean AND < 1.0 m;
  pursuit must only have genuinely flown the same engagement (engaged=1, valid
  miss) — its failure to close IS the demonstrated result, and requiring it to be
  "clean" contradicted the experiment.
- **Verification (official gate, 3 independent runs — mine + 2 by verifier):**
  pro-nav miss 0.402 / 0.277 / 0.443 m (gate < 1.0); pursuit 2.544 / 2.109 /
  2.048 m on identical paths (4.6-7.6× worse). Verifier confirmed target motion
  at 2.000 m/s sim through CPA (mover sim_t CSV + gt_tag_y cross-check),
  recomputed all misses from raw gt_range, and numerically confirmed r_hat tracks
  meas_range (camera) not gt_range where they diverge (no-cheat, per council
  requirement). Logs: m4_intercept_{pursuit,pronav}_20260705T{0322xx,0324xx,0331xx}Z.csv.
- **Date:** 2026-07-05.

## ADR-0010 — M4.5 realism upgrade: FPV speed, two-stage sensor handoff, maneuvering targets (COUNCIL, 3 members, unanimous core)
- **Context:** After M4 (camera-only pro-nav beats pursuit on a 2 m/s straight crossing), the builder asked to make it realistic: both aircraft at FPV speeds; replicate the parent project's TWO-sensor→ONE architecture (external ground cue mid-course, hands off to onboard-camera-only terminal — the comms-denied headline); many variable target paths; prove the algorithm adapts. Then M5. High-stakes, multi-variable, one-way-door → council per CLAUDE.md.
- **Measured envelope (probe, this session):** gz_x500_mono_cam cleanly tracks 12 m/s (pitch −9°) and 16 m/s (pitch −12°) with `MPC_XY_VEL_MAX=20, MPC_ACC_HOR_MAX=12, MPC_TILTMAX_AIR=60, MPC_JERK_MAX=30`; altitude held, near-perfect NED-setpoint tracking. FPV speeds are achievable; the fixed forward camera stays roughly level.

### Unanimous decisions (all 3 seats)
1. **Guidance: keep the ADR-0009 plain-PN mechanization unchanged in structure.** N=4 fixed per run (swept in M5). NO augmented pro-nav / target-accel estimation, NO true-3D vector PN, NO in-flight adaptive N. All three seats: APN needs a 2nd derivative of an already-noisy filtered signal (or a new α-β-γ state) — a new failure-mode class, on a project that spent 5 iterations stabilizing the *current* filter; and plain PN's bounded miss growth under maneuver IS the resume finding (the miss-vs-intensity curve), not something to patch away. Plain PN is feedback on λ̇, so it already reacts to any maneuver shape.
2. **Decouple dash speed from terminal speed (THE pivotal call).** Interceptor dashes 12 m/s mid-course (under the external cue), then throttles commanded closing to ~5–6 m/s the instant it goes camera-only. Keeps terminal dynamics at M4's validated ~3–4 m/s difficulty even though headline speed is FPV. Resolves the detection-window tension WITHOUT enlarging the tag (2 of 3 seats explicitly reject tag inflation — it would dishonestly move M2's hard-won 6–8 m envelope numbers). Re-derive/re-bench the α-β rate gains and rescale the terminal ranges (`TERMINAL_RANGE_M`, `TERMINAL_FREEZE_RANGE_M`, `BREAKOFF_ARM_RANGE_M`, `VC_FLOOR_M_S`) so their *time* semantics survive at the new terminal speed — do NOT assume M4 constants port unchanged.
3. **PX4 params set via MAVSDK runtime API before arming** (not an airframe override file) — keeps PX4 untouched (ADR-0001/0005) and the exact params traceable in each run's own log. Log the 4 touched params into each run header; verify at each run start so a stale param can't contaminate a pursuit-vs-pronav pair.
4. **External cue = degraded ground truth, mocked (GOALS.md IS-NOT list literally says "ground stereo rig … mocked away here" — so this is mandated, not a shortcut).** Own subscription-free process (m4_target_mover pattern) reading `/world/apriltag/pose/info`, degraded to: position σ ≈ 0.5 m/axis, fixed latency ~100–150 ms, 10 Hz update (deliberately coarser than the ~14 Hz camera + 20 Hz control — an honestly *inferior* cue, which is what motivates the terminal seeker). Separate CSV columns `ext_*`, distinct from `gt_*` (score-only forever) and `meas_*` (camera). Hard switch, no fusion, no blending. Anchored to the parent project's own `docs/tradeoffs.md` ("sub-meter terminal accuracy, 50 m baseline") — the mock is deliberately kept coarser than the real rig. Honesty boundary: own-state (EKF) + external cue = legitimate; `/world/.../pose/info` stays scoring-only at every phase.
5. **Handoff:** camera achieves the (shortened) ACQUIRE lock streak OR range < ~8–10 m, whichever first; from that tick the external channel is STRUCTURALLY out of scope for the control function (illegal-state-unrepresentable, not unused-by-convention — the ADR-0006 lesson). Terminal-camera-only proven by extending the M3/M4 numeric no-cheat check: recompute what an external-cue-derived command would have been, confirm the sent command tracks camera `meas_*`/λ/R̂ at ticks where they diverge; assert zero external-labeled ticks after handoff.
6. **Board face is rigidly world −X (static SDF; mover sets position only).** So "maneuvers" are velocity-schedule changes, never presented-aspect changes — a real, disclosed GOALS.md simplification. Every path must keep target X > interceptor X for the camera phase. Any true heading/weave path needs the mover extended to stream a PRECOMPUTED orientation schedule (still subscription-free) if visual aspect ever matters — but for velocity-only maneuvers, position streaming suffices.
7. **Proof from existing log columns, no new instrumentation.** Unified x-axis = peak TRUE |λ̇| (or peak target lateral accel), computed post-hoc from the mover's own sim-time CSV + geometry. Miss-vs-intensity curve (pursuit vs pro-nav). Adaptation latency = sim-time from a logged jink/step onset to λ̇_hat re-settling (< ~5°/s). Per-path pursuit-vs-pronav miss ratio (require pro-nav to beat pursuit each path; don't demand M4's 4.6–7.6× margin holds at speed — a compressed-but-favorable margin under maneuver is itself an honest finding). **Tiered gate** thresholds by maneuver class, not one flat number.

### THE structural decision (scope-skeptic seat, adopted): sequence, don't bundle
M4 needed 5 dev iterations + 2 failed gates to root-cause ONE new variable (the RTF/sim-time bug). M4.5 as asked stacks THREE at once (speed regime + sensor subsystem + rewritten maneuvering mover) — a bad result would be unattributable (algorithm? mis-scheduled jink? bad noise draw? leaked GT read?), which destroys the "reproducible, logged, defensible" ethos that IS this portfolio. **Build as independently-gated sub-steps, each reusing check_m4.sh's boot/gate pattern so a failure isolates to one variable:**
- **S1 — Speed-only:** M4's straight-line crossing at FPV speed (target ~6 m/s, interceptor params bumped, terminal speed throttled). Re-tune gains + terminal ranges. Re-gate. Isolates "does the guidance survive the speed regime."
- **S2 — Handoff, still straight-line:** add external-cue mock + CUE→HANDOFF→TERMINAL state machine. Dash on the cue, hand off to camera-only. Re-gate on straight-line. Isolates "does the two-stage architecture work + stay honest."
- **S3 — Path suite:** add the multi-segment/variable-speed/jink mover (sim-time scheduled — verify sim-time scheduling composes with a DISCONTINUOUS velocity change). Gate on ~4 paths {crossing L→R, R→L, S-weave, jink}. Isolates "does it adapt to maneuvers."
- **S4 — All-up adaptation proof + verifier:** full path suite, miss-vs-intensity, pursuit-vs-pronav, no-cheat + terminal-camera-only audit.
- **Then M5.** PROTECTED: if S1–S4 start eating M5's budget, M5 wins — a complete honest static→pursuit→pro-nav→Monte-Carlo arc beats a half-built realism upgrade with no writeup.

### Dissent / warnings carried forward
- **Detection-vs-speed geometry (seat A, sharpest):** back-derived coast time constant ≈0.5 s → at 15 m/s terminal, coast would START at ~7.5 m (edge of detection), leaving ~zero actively-guided camera-only PN. This is WHY decision #2 (throttle terminal speed) is load-bearing, not optional; S1's gate must confirm real actively-guided window exists before S2.
- **Sim omits motion blur / rolling shutter (seat C):** detection may hold at ~14 Hz in sim where a real seeker degrades at speed — flag explicitly in the README as a known sim-vs-real gap; don't let it make terminal-window numbers look better than reality.
- **Oblique aspect (seat C):** AprilTag pose degrades at oblique incidence (ADR-0003 risk) even inside the FOV — check per-path empirically, don't assume from FOV numbers.
- **Cut list (seat C, adopted):** APN, true-3D w/ pitch pointing, in-flight adaptive N — cut entirely. Vertical-maneuvering targets, Kalman fusion, tag rotation — defer to README "future work". External-cue mock — keep to exactly 3 params, resist gold-plating.
- **Deferred sub-decision:** exact terminal speed (~5–6 m/s), gain schedule, and tiered thresholds are proposals — validated by S1/S3 dev runs, not derivations.
- **Date:** 2026-07-05. (Council: 3× council-member @ max effort, one seat briefed as scope-skeptic; independent briefs; synthesis by main session.)

## ADR-0011 — Guidance-design harness + Predicted Intercept Point (PIP) as the lead law
- **Context:** S1 dev runs showed pure pro-nav lagging a fast crosser (1.6 m miss at 4 m/s) — it nulls LOS rotation but never predicts where the target is going. Builder steered: improve prediction / build a fast auto-iterator to find the best method / research what's best.
- **What was built:** `scripts/guidance_lab.py` — a pure-Python point-mass Monte-Carlo harness (no Gazebo/MAVSDK) modeling the MEASURED interceptor envelope (v_max 16, a_max 12, first-order PX4 lag tau~0.3s) and a camera sensor model (8 m range, ~14 Hz, bearing/range noise, dropouts). ~3 s for ~3000 runs. Ranks guidance methods across 7 paths x 3 speeds x 20 seeds. Explicitly a DESIGN-TIME SURROGATE — its winner is a hypothesis that still needs a real Gazebo gate.
- **Result (full sweep, mean miss):** pip 0.181 m < pure_pn 0.352 < apn 0.371 < pn_plus_lead 0.478 < pursuit 0.691. PIP wins on ALL 7 paths, 2-4x. Best PIP gain: V_CLOSE=6.0 (0.125 m) vs 8.0 (0.194 m) — a speed/accuracy tradeoff. Best pure_pn: N=5, V_CLOSE=8 (matches ADR-0009's council range).
- **Key empirical findings:** (1) PIP dominates because it does a genuine intercept-triangle quadratic solve, not just LOS-rate feedback — the direct cure for "lagging behind." (2) APN does NOT reliably beat plain PN even on maneuvering paths (noisy 2nd-derivative term) — empirically vindicates ADR-0010's council rejection of APN. (3) pn_plus_lead (naive velocity feedforward added to PN) is often WORSE than plain PN — simpler isn't safer. (4) Research backs PIP/lead + APN as the maneuvering-target upgrades (PN leaves nonzero miss vs a mover by design).
- **Harness caveats (honest):** the lab's sensor has NO field-of-view cutoff, so it can't reproduce pursuit's real M4 failure mode (spinning off-boresight and losing the tag) — its pursuit numbers are for a purer "no lead" reason. And PIP needs a clean target position+velocity TRACK, which in Gazebo means trusting an alpha-beta track built off noisy monocular measurements — a cost the lab under-prices. So PIP may transfer less cleanly to Gazebo than the lab suggests.
- **Decision:** port PIP into the real guidance as a new `--law pip` option (pursuit + pronav stay intact for the comparison), then VALIDATE in Gazebo against the fast crosser and let the real gate decide. Frame math reuses the bench-validated LOS azimuth: rel_ned = range * (cos(psi+beta), sin(psi+beta)) [north=cos, east=sin], abs_target = own_NED_pos (EKF, own-state) + rel_ned. Honesty boundary unchanged: target info from the camera only; own position from the EKF. If PIP's noisy-track cost degrades it in Gazebo, fall back to tuned pure PN (N=5). NOT a full council: reversible (added option), evidence-backed (lab + research), and directly builder-requested — decide-and-log per CLAUDE.md's protocol; a from-scratch guidance rewrite would warrant one.
- **Date:** 2026-07-05.
