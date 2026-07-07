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

## ADR-0011 addendum (2026-07-05) — PIP did NOT transfer to Gazebo; pure PN is the robust choice
- **Ported PIP to the real sim** (`--law pip`, own-NED-position + camera rel vector, bench-validated psi+beta frame) and validated in Gazebo. Result: PIP is WORSE than pure PN in the real sim — 3.03 m vs pure-PN's 1.6 m at a 4 m/s crosser; 4.75 m at 6 m/s.
- **Why (the honest finding the lab under-priced):** PIP's advantage needs a clean target position+VELOCITY track. Gazebo's real monocular stream — ~14 Hz, bearing/range noise, and 30-40% detection dropouts as the tag nears the FOV edge — gives the alpha-beta track a poor velocity estimate, so PIP's lead point is wrong and it does worse than a law that needs no target-velocity track at all. Pure PN reacts only to the LOS *rate*, which survives the noisy/intermittent stream far better. This is a genuine, resume-worthy result: **the theoretically-superior lead-guidance law degraded under realistic monocular sensor noise; the simpler, sensor-light LOS-rate law proved more robust** — and it's exactly why ADR-0011 pre-committed to validate in Gazebo rather than trust the surrogate.
- **Decision (pre-registered fallback taken):** pure PN is the guidance for the FPV work, at the lab's best gain N=5. `--law pip` stays in the tree as a documented, reproducible negative result for the writeup (and would benefit from the two-stage handoff's cleaner mid-course track — a possible S2 revisit, not a blocker).
- **FPV-profile pure-PN validation:** N=5 + two-speed closing + rescaled terminal ranges intercepts a 3 m/s crosser at 0.939 m (clean, < 1 m gate) from a hover start. At 4 m/s: ~1.6 m; at 6 m/s: uncatchable from hover (min range ~4.7 m).
- **The coupling finding (S1 <-> S2):** a HOVER-START interceptor is kinematically speed-limited — it can't build enough closing speed to catch a fast crosser before the target crosses. So the full FPV target band (6-10 m/s) genuinely REQUIRES S2's external-cue DASH (a running start), exactly as council seats B/C warned that speed and handoff are coupled. S1's honest ceiling from hover is ~3 m/s clean. Recommendation: treat S2's dash as the enabler of fast intercept rather than pushing S1's hover-start against ever-faster targets.
- **Date:** 2026-07-05.

## ADR-0012 — Hardware stack for the FPV-drone deployment (COUNCIL, 3 members)
- **Context:** New goal — get the software ready to import onto a real, affordable FPV drone (full deployable build). User handed the hardware choice to the swarm. 3-member council (one scope-skeptic seat) researched 2026 parts + pricing. Strong convergence; one split (companion computer) resolved below.

### Decisions (unanimous unless noted)
1. **Flight controller: PX4 on a Holybro Pixhawk 6C (or 6C Mini).** NOT ArduPilot: the code pushes PX4-only `MPC_*` params at runtime (ADR-0010) and MAVSDK's Offboard plugin is PX4-native — ArduPilot would mean re-deriving the whole validated envelope for zero benefit. Betaflight confirmed a trap (transmit-only MAVLink telemetry as of 2026.6, no offboard setpoints). The trap is FIRMWARE, not the board — a Holybro AIO can run PX4 if flashed. 6C Mini ~$131 / 6C ~$166.
2. **Companion computer: Raspberry Pi 5 (8GB), ~$120** (price volatile in 2026, $95-205 on a DRAM shortage). **Split resolved:** two seats leaned Jetson Orin Nano ($249) for CPU/thermal headroom, but seat B's argument is decisive and the others conceded it — the Jetson's ONLY real advantage is CUDA-accelerated AprilTag via **Isaac ROS**, which is **ROS 2**, which GOALS.md explicitly forbids ("No ROS 2"). `pupil-apriltags`/`pyapriltags` is CPU-only with no CUDA path outside Isaac ROS, so the Jetson's TOPS are structurally unusable here — 2x price + 2-3x power for an idle GPU. Pi 5 is the best-documented PX4 companion, 3-9 W. **HARD CAVEAT (unanimous):** Pi-5 detection rate for this exact detector is UNMEASURED; ~15 Hz is plausible (AprilMAV ~20 fps @ 2 MP) but must be BENCHMARKED before trusting the ADR-0009/0010 timing constants. If it can't clear ~15 Hz, drop resolution / raise quad_decimate, or only then reconsider a Jetson (accepting CPU-only).
3. **Camera: global shutter, mono, ~90-100° HFOV** to match the sim's 1.74 rad (99.7°). Global shutter is MANDATORY (all 3 seats): the strapdown seeker yaws in lockstep with the tag at up to 60 deg/s (`YAWSPEED_MAX_DEG_S`), and rolling-shutter skew would corrupt the exact tag corners the pose solve needs — during the terminal phase where the sim is already marginal. Options: Arducam OV9281 mono USB (~$60-70, USB/UVC = smaller code change) or RPi Global Shutter Camera IMX296 CSI + wide M12 lens (~$75, needs a lens swap off the default 65° to hit ~100°). Grayscale-only detection already (`image_msg_to_gray`), so mono is a strict match.
4. **Airframe: STAGE IT. 2.5" is a fantasy** (avionics alone ~200-320 g > a whole micro's AUW). Bring-up rig = **Holybro X500 V2** (~$261 ARF / ~$533 full kit) — the real-world twin of `gz_x500_mono_cam`, PX4's documented Pi-companion frame, lowest integration risk. Deployable FPV interceptor = a **7" cinelifter/X8-class** frame later (real payload margin, matches the FPV-speed story). 5" is tight-to-unrealistic with this companion stack.
5. **Link + integration:** companion→FC on **TELEM2 UART, `SER_TEL2_BAUD=921600`**, 3-wire 3.3V (verify logic level with a meter — PX4 warns some companions are 1.8 V and can be damaged). Code change is ONE line: `SYSTEM_ADDRESS` `udpin://0.0.0.0:14540` → `serial:///dev/ttyAMA0:921600`. Own-state telemetry (`attitude_euler`, `position_velocity_ned`, `relative_altitude_m`) is unchanged API against real EKF2 — the payoff of building on MAVSDK. Fixed-forward camera mount matches the sim's fixed boresight (no gimbal, no change to `lambda=psi+beta`). **ELRS RX (~$25) mandatory** as a safety-pilot manual-override/kill link for first autonomous flights — never read by the guidance, so it doesn't touch the comms-denied terminal story. Terminal phase needs NO ground link by design.

### Confirmed port-gap changes (code)
- **AprilTag library swap: `pupil-apriltags` -> `pyapriltags`.** Confirmed: pupil-apriltags ships NO Linux aarch64 wheel (PyPI x86_64 + macOS arm64 only; piwheels 32-bit only); Pi 5 is aarch64. `pyapriltags` ships aarch64 wheels and ADR-0003 already pre-registered it as the drop-in fallback. `opencv` and `mavsdk` both ship aarch64 wheels — the detector is the one casualty. Make the import swappable (try pupil, fall back to pyapriltags) so dev-x86 and drone-aarch64 both work.
- **Real camera calibration + undistort** (built this session: `calibrate_camera.py` + `frame_source.py` undistort). `record_camera_intrinsics.py` only read the sim's ideal K — a real checkerboard calibration replaces the sim's fx=539.9, and undistort makes the pinhole valid for a real lens.
- **Camera source swap** via the new `FrameSource` abstraction (gz -> OpenCV). Detection/guidance downstream unchanged.
- **`TAG_SIZE_M`** must equal the real printed tag (pose_t scales linearly — a measurement error biases every range).
- **No ground truth on hardware** — the `gt_*` scoring/no-cheat columns come from `/world/apriltag/pose/info`, absent in reality. Keeping the same evidentiary claims needs an external reference (mocap / dual-GPS / reviewed video). A METHODOLOGY gap, not just code.
- **Re-tune timing constants** — every filter gain / ACQUIRE streak / terminal range was conditioned on the sim's ~14 Hz desktop cadence; assume none port unchanged (ADR-0009's RTF saga is the preview).

### THE unanimous biggest risk + the staged plan (adopt)
All three seats, emphatically: the #1 risk is **detection cadence + motion blur at FPV terminal speed shrinking the already-marginal camera-only window below viability** — the sim runs at the ragged edge of its detection envelope with ZERO real-world degradation (ADR-0010 seat-A's ~7.5 m coast-start math; ADR-0011's finding that even clean sim noise broke PIP). Real lens + real blur + real vibration + slower embedded CPU + uncontrolled lighting (ADR-0007 needed an emissive hack for even SIM contrast) only shrink it. **This is existential to the intercept, not a tuning nuisance — and no BOM dollar fixes it.**
Staged deployment (skeptic seat, adopted):
- **Stage 0 — bench perception, BEFORE buying an airframe.** Buy only Pi 5 + camera (~$200). Build pyapriltags on it, mount the camera, measure real detection Hz + range against a printed tag at the real intended size, in real outdoor light, spinning to reproduce yaw-rate blur. Go/no-go gate on the whole hardware plan; could save the airframe spend.
- **Stage 1 — full avionics on a bench/thrust-stand, tethered.** Real MAVLink link, real guidance loop, vehicle restrained. Catches wiring/EMI/UART cheaply.
- **Stage 2 — manual/position-hold flights, companion logging only.** Confirm real GPS/EKF quality + detection in flight, autonomy not touching sticks.
- **Stage 3 — re-prove M3 (slow static intercept) on hardware first**, safety pilot + OFFBOARD kill + exclusion zone, BEFORE any moving-target/FPV-speed attempt. The full FPV profile is a stretch goal, not the first hardware milestone. Defer `--law pip` (already shown not to transfer). Keep M5 Monte-Carlo sim-only. Target starts as a fixed pole -> slow cart -> only later a second autonomous drone.

### Bill of materials (Pi 5 / X500 bring-up build, cheapest defensible)
Pixhawk 6C Mini $131 · X500 V2 ARF (frame+motors+ESC+props) $261 · M9N GPS $55 · Pi 5 8GB ~$120 · Pi accessories (cooler/PSU/SD) $35 · global-shutter cam + wide lens $75 · ELRS RX $25 · 4S 5200 mAh $55 · misc $30 = **~$750-800** for the bring-up rig. A 7" deployable interceptor frame + 6S propulsion is a later ~$450-700 second iteration. (Jetson variant ~$1,100-1,350; rejected on the no-ROS-2 tiebreaker.)

- **Date:** 2026-07-05. (Council: 3× council-member @ max effort, one scope-skeptic seat; independent briefs + web research; synthesis by main session. Split on companion computer resolved by the No-ROS-2 constraint from GOALS.md.)

## ADR-0011 third addendum (2026-07-05) — guidance lab recalibrated to Gazebo; S2 dash validated offline
- **The lab now predicts reality.** Upgraded guidance_lab.py's sensor model to match Gazebo: a field-of-view cutoff with a yaw-tracking boresight (slews 45 deg/s toward the last detection; a fast crosser outruns it and walks off-frame), edge-weighted dropout, and realistic bearing/range noise (0.5->6 deg, 2->10%). Also fixed a real harness bug — every law's terminal freeze held its last command FOREVER, letting a frozen ballistic coast fly a lucky post-CPA pass and make 6 m/s look catchable; added a ground-truth breakoff (mirrors m4_intercept.py, never fed to guidance). Calibration (lab mean vs Gazebo): pure_pn 3 m/s 1.66 vs 0.94; 4 m/s 1.75 vs ~1.6; 6 m/s 2.36 (p90 4.92) vs uncatchable ~4.7; pip@4 2.01, WORSE than pure_pn — directionally correct on all four asks (miss rises with speed, PIP worse than PN, coverage 0.55-0.77 falls with speed). Honest gap: the lab UNDERSHOOTS the 6 m/s "uncatchable" severity (its mean is only "bad", not "hopeless") — a 2D point-mass surrogate converges too well once it has any LOS-rate signal; the p90 does match. Trust it for RANKING, not absolute miss.
- **Camera-only winner under realistic noise: pure_pn** (not PIP) — confirms Gazebo/ADR-0011. Pooled 7 paths x 3 speeds x 20 seeds: pure_pn 1.786 < apn 1.798 < pip 1.804 < pursuit 1.934. On the crossing paths (the real engagement), pure_pn wins at every speed. Under real noise PIP's clean-sensor 2-4x advantage evaporates.
- **S2 dash validated offline (the actionable finding).** Long-standoff geometry (target ~15 m out, real dash room). Camera-only hover baseline ~6 m (hopeless). Best dash config swept over dash_speed{10,12,14} x handoff{6,8,10} x terminal{pure_pn,pip}: **dash 10 m/s, handoff 10 m, terminal PIP** wins at all speeds — 6 m/s -> 0.71 m, 8 m/s -> 1.27 m, 10 m/s -> 2.13 m. So the two-stage dash makes the full FPV target band catchable (as ADR-0010/0012 predicted the coupling), AND **PIP WINS the terminal phase in two-stage mode** — reversing its camera-only loss — because the external cue's clean mid-course track (0.5 m sigma, 10 Hz) fixes exactly PIP's starved-velocity-estimate failure before handoff. **-> S2 should use the dash + a PIP terminal, and PIP is worth a real Gazebo re-validation WITH the handoff (its camera-only-from-hover failure was not the whole story).**
- **Process note (a real hazard hit):** main ran `git add -A` in hardware commits 15f03c4/e2cbe43 while this background worker was mid-edit on guidance_lab.py, sweeping partial work into unrelated commits. Recoverable (this addendum's commit reconciles to the validated version), but the lesson stands: do NOT `git add -A` while background agents are editing files — stage specific paths, or wait for workers to finish.
- **Date:** 2026-07-05.

## ADR-0013 — S2 built in Gazebo (two-stage handoff) + tracking-refinement study: what transferred, what didn't
- **Context:** Builder directive: "get rolling on refining the tracking algorithm and continue researching then implement methods that improve speed and reliability." Two parallel Sonnet threads: (1) build S2 (ADR-0010 sequencing, ADR-0011 3rd-addendum config) in Gazebo; (2) research tracking best practice + prototype candidates in the recalibrated lab, port winners only if they validate in Gazebo (the ADR-0011 workflow).

### S2 implementation (the two-stage handoff is real now)
- **`scripts/s2_cue_mock.py`** — the mocked external ground sensor (ADR-0010 #4, exactly 3 degradation knobs): samples tag truth on a 10 Hz SIM-time schedule, adds sigma=0.5 m/axis Gaussian noise (seeded), delivers each sample 0.12 s of sim time late, emits UDP JSON datagrams (noisy position only; truth goes only to its own audit log). Subscriptions are safe in this process because it makes zero gz *service* calls — the mover's ADR-0009 quirk kills service responses, not subscriptions.
- **`m4_intercept.py --handoff`** (requires `--fpv`): phases CUE_WAIT → DASH → HANDOFF → the *untouched* M4/S1 terminal. One shared TargetTracker across phases (lab TwoStageDash semantics): cue-corrected pre-handoff (camera preferred when both), camera-only after. DASH = PIP lead solve on the cue track at 10 m/s (lead cap 4 s). HANDOFF = >=3 consecutive fresh camera detections + camera range <= 10 m, then a ONE-WAY LATCH: the UDP socket is closed and the cue holder nulled — post-handoff cue reads are structurally impossible (ADR-0010 #5, illegal-state-unrepresentable), while the mock keeps logging as available-but-unread evidence. lambda/range filters stay camera-only throughout and warm up as detections begin late in the dash.
- **World→NED mapping determined empirically, not assumed: north=world_y, east=world_x** (no sign flip) — the opposite of the naive guess. Evidence: commanded-NED-velocity vs world-displacement residuals of −14.0°/−9.6°/−10.1° across 3 prior FPV logs under this mapping vs −33° to −75° under the alternative; independent yaw-at-engage cross-check agrees (~121.6° computed vs ~127-128° measured). Documented in m4_intercept.py's docstring.
- **Result (6 m/s crosser, 15.4 m standoff — uncatchable from hover per ADR-0011 addendum):** first camera detection at 8.8-9.3 m mid-dash, handoff latches at 7.2-8.8 m (~2.1-2.3 s of dash), camera-only terminal, all audits pass. Dev-flight misses: 1.087-2.062 m (pip), 1.867-2.247 m (pronav). The dash halves the hover-start's ~4.7 m floor on day one.

### Tracking-refinement study (research → lab → Gazebo A/B)
- **Lab phase (guidance_lab.py, opt-in candidates, baseline verified byte-identical):** Kalata tracking-index-derived alpha-beta gains (Kalata 1984 — gains recomputed from the ACTUAL dt each correction; principled dropout response for free) won camera-only pure_pn by **−24-28% mean / −18-21% p90** across 19/21 path×speed cells. Cue-latency compensation (advance a stale cue sample along track velocity by its known age — textbook OOSM retrodiction-lite) improved the two-stage config **11-14% pooled mean, no regressions at any speed**. Negative in the lab: range-outlier gating (nothing to catch in Gaussian noise), naive Kalata on the Cartesian tracker (needs absurd process sigma), 4-state CV Kalman (mixed vs the simpler fixes; kept lab-only for future Gazebo fat-tail work).
- **Gazebo A/B (the decisive phase): Kalata does NOT transfer.** Every Kalata-enabled flight was worse (3.24/3.24/2.25 m vs the 1.09-2.25 m no-flag cluster). Mechanism caught directly in the logged effective gains: Gazebo's real correction cadence is BIMODAL — ~14 Hz bursts and multi-second dash/handoff gaps — and the lab-tuned sigmas go degenerate at both extremes: range channel alpha=0.031/beta≈0.000 at burst cadence (rate filter goes deaf → Vc pinned at floor → a_cmd=N·Vc·λ̇ starved of lead intensity exactly like the ADR-0009 pathology), lambda channel alpha=0.999/beta=1.876 after multi-second gaps. Confirmed by an isolated `--kalata`-only flight replicating the combined-config miss almost exactly (3.240 vs 3.236). A same-code no-flag control flight ruled out environment drift (and set the session-best 1.087 m).
- **Cue-latency compensation in Gazebo: inconclusive** (n=1, 2.339 m, inside the no-flag spread). Measured ext_age_s 0.116-0.152 s (mean ~0.132) matches mock latency + jitter. Note a constant cue lag mostly biases *position* (which post-handoff camera corrections wash out), not the velocity slope PIP inherits — consistent with a small real effect at Gazebo's noise floor.
- **Decisions:** (1) Kalata REJECTED for flight defaults; stays as `--kalata` (default OFF) — a documented, reproducible negative result, same treatment as camera-only `--law pip` (ADR-0011 addendum). Future-work note: clamping the Kalata dt into a sane band, or per-regime sigmas, might rescue it — not worth flights now. (2) Cue-latency comp KEPT opt-in (`--cue-latency-comp`, default OFF) — physically sound bias fix, lab-validated never-regress; M5's Monte-Carlo has the statistical power to settle it. (3) The S2 gate runs plain `--fpv --handoff`, no tracking flags. **The meta-lesson now has three data points (PIP, lab-vs-Gazebo calibration, Kalata): the lab RANKS credibly but its winners must re-earn their result in Gazebo before touching flight defaults.**

### Gate design (`scripts/check_s2.sh`) + measured variance
- Fresh boot per law (M4 pattern); BOTH laws (pip, pronav) must individually pass: clean, handoff latched, audits, miss < **2.5 m tiered gate** at 6 m/s (ADR-0010 #7 tiered thresholds; rationale: S2's claim is the ARCHITECTURE — running-start + honest handoff makes an uncatchable target catchable — not sub-meter precision at 3× M4's speed; precision is S3/M5 scope). Audits: (a) zero non-empty ext_* cells from first ENGAGE row (the latch, forever); (b) >=1 DASH row precedes ENGAGE (no degenerate instant handoff); (c) law-aware no-cheat: commanded-velocity azimuth vs camera-derived lambda correlation >=0.7 pronav / >=0.55 pip (PIP's lead point legitimately decorrelates from raw LOS — the session's cleanest flight, 1.087 m, measured 0.689).
- **Run-to-run variance is real and documented:** identical no-flag S2 flights spanned ~1 m (1.087-2.062 pip). Driver: terminal camera-dropout timing (lock holds to ~2.0-2.4 m, coast flies the rest) — the known ADR-0009 failure class at FPV terminal speed, not an S2 bug. Single-flight A/B deltas below ~1 m are NOISE in this regime; the Kalata rejection rests on 3 consistent flights + the gain-mechanism evidence, not one number.
- **Official gate run (2026-07-05): PASS, exit 0.** pip miss 2.291 m (`logs/m4_intercept_pip_20260705T211520Z.csv`), pronav 1.992 m (`logs/m4_intercept_pronav_20260705T211632Z.csv`); both clean=1 handoff=1, cue_reads_post_handoff=0. Pronav detail: cue lock 1.59 s (5 datagrams), first camera detection 8.97 m mid-dash, handoff latch at 8.27 m (streak=3), terminal coast frozen at r_hat 3.42 m, audits (a) 0 ext cells over 94 post-ENGAGE rows / (b) 65 DASH rows precede ENGAGE / (c) corr 0.881.
- **VERIFIER-CONFIRMED (independent adversarial rerun, 2026-07-05): PASS.** Verifier re-flew the full gate (pip 2.270 m, pronav 2.342 m — a different draw inside the documented ~1 m variance; both < 2.5 m, clean, handed off, audits pass) and re-derived the evidence rather than trusting the script: min gt_range recomputed from CSVs matched S2_RESULT to the mm; audit-(c) correlations recomputed exactly (0.809 pip / 0.735 pronav); detector-vs-truth ranges never identical (0/67 rows; mean offset ~0.7 m, std ~0.3 m — a genuinely independent measurement chain); tracker cue-warmed ~80 rows before first camera detection; cue-mock degradation measured on-spec (sigma 0.533/0.534 m, latency 0.124 s); target speed fit 6.0000 m/s from the mover log, corroborated 5.97-6.01 from the cue's independent GT samples; DASH wall-duration 3.2-3.3 s (an apparent timeout anomaly resolved as a clock-epoch difference in the printouts). Two honest nuances recorded: audit (a) is structurally guaranteed by the latch code (`cue_reader=None` at handoff; sole read site guarded) — a regression detector rather than a test that can fail in normal operation, which is the intended illegal-state-unrepresentable design; and under RTF sag the cue mock's sampling thins near flight end, so its post-handoff log is best framed as "channel stays alive/available" rather than "dense fresh datapoints" (the structural latch proof is independent of this). Verifier CSVs: `logs/m4_intercept_{pip_...T211926Z,pronav_...T212051Z}.csv`.
- **Date:** 2026-07-05. (Process: 2 Sonnet workers — S2 build ~6 dev + 6 A/B flights, tracking study ~3000-run lab sweeps — Fable synthesis/decisions. One worker was killed by the account session-token limit before the final gate run; main session finished the gate edits and ran the gate itself. A machine suspend also froze one A/B flight mid-run [the 16:55Z pip flight]; its conclusion was re-validated by a clean post-resume replication before being trusted.)

## ADR-0014 — Path to a defensible intercept success rate (Pk): terminal-guidance fixes over sensor upgrades (COUNCIL, 3 members)
- **Context:** Builder asked to push the S2 camera-only interceptor to **>=95% success** and authorized a **proximity-fuse hit definition** (success = closest approach < R_lethal), offering "IR / higher-res if needed." High-stakes + touches the ADR-0010 anti-tag-inflation honesty ruling + genuinely uncertain -> 3-seat council (guidance/honesty seat A, scope-skeptic seat B, guidance-controls-depth seat C), each reasoning from the ADRs and two traced S2 flight CSVs (no sims, to avoid contention with a live Monte-Carlo worker). Builder is away long-term; this executes autonomously.

### THE re-diagnosis (seat C, adopted — corrects the earlier "FOV overflow" story)
Tick-by-tick trace of `logs/m4_intercept_pronav_2026070{5}T22{0838,1935}Z.csv`: the tag is lost at a camera **bearing of -18.7 deg to -42.2 deg — well inside the +-49.85 deg FOV half-cone, still ~100+ px wide**. It is NOT FOV overflow, oblique tag-face incidence (which is actually *improving* as lambda->90 deg), or quad_decimate pixel-starvation. The mechanism is a **yaw-tracking-rate deficit**: achieved yaw rate ~21-24 deg/s vs. required LOS rotation ~32-58 deg/s over the loss window, so the boresight walks off-target. This is the SAME R->0 angular-rate divergence that makes lambda_dot singular, hitting the yaw-pointing loop one step upstream of the guidance integrator. Corollary finding: the FPV PX4 bundle bumps translational agility (`MPC_XY_VEL_MAX/ACC_HOR_MAX/TILTMAX_AIR/JERK_MAX`) but sets **no yaw-rate param at all** — PX4's `MPC_YAWRAUTO_MAX` (community default ~45 deg/s) plausibly caps the slew below our own 60 deg/s command clamp (`YAWSPEED_MAX_DEG_S`), an untried, near-free lever.

### Unanimous decisions (all 3 seats)
1. **Metric = the full Pk-vs-R_lethal CURVE, not a single number.** Fix R from an independent physical rationale BEFORE looking at results; report per-speed (6/8/10 m/s, never pooled) with Wilson 95% CIs. Headline **R_lethal = 1.0 m** (GOALS.md's own established M4 "<1 m closest approach" bar — the project's existing resume-line precision standard, not a number invented to clear a threshold), with **0.5 m** shown as the aggressive kinetic point (two-small-quad body/prop half-span; the sim target has no airframe body, only a flat board — so any R is a disclosed narrative assumption, not a simulated collision volume) and the whole curve visible so a reader applies their own bar. **Refuse to headline R >= 2.0 m** (seat B: that region mostly scores the ballistic coast, and 2.5 m is the exact figure ADR-0013 already disclaimed as "architecture, not precision").
2. **Do NOT pre-commit to hitting 95%.** Implement the honest levers, MEASURE where the curve lands. If an honest 95% falls at R~2.5-3.5 m (seat C's miss-floor estimate: even with all terminal fixes, mean miss ~0.8-1.5 m, p95 ~2.5-3.5 m, so sub-meter 95% likely needs backing target speed toward the 3-4 m/s band where S1 already showed sub-1m), REPORT THAT — a disclosed degrading Pk-vs-speed curve beats an inflated flat number (same ethos as ADR-0011's "PIP didn't transfer" negative result).
3. **CUT as dishonest/ineffective:** IR (non-sequitur — the lock is a fiducial found by contrast, not a thermal signature; the bottleneck is geometric/kinematic, not sensing-modality); camera FOV/resolution widening (functionally tag-inflation on the sensor — reopens the ADR-0010 door, forces re-earning M1/M2, and diverges from ADR-0012's committed ~90-100 deg hardware camera); quad_decimate 2->1 for S2 (moves AWAY from hardware realism — ADR-0012 flags Pi-5 rate as unmeasured, may need MORE decimation). **Defer nested tags:** seat C's geometry shows a 1/5-scale tag only becomes detectable at ~1.2-1.6 m, a band the current mechanization never reaches (loses lock at 2.1-3.3 m) — it can't help until the terminal fixes push real detection down near ~1-1.5 m. Future work, additive-only, disclosure-gated if ever adopted.
4. **Monte-Carlo rigor before any "%":** ~100-150 flights per speed cell (n=20 gives a Wilson CI of ~[75%,99%] — cannot distinguish 90% from 95%); vary cue seed, crossing offset, and L2R/R2L direction; report "Pk = X% (95% CI Y-Z%) at R=1.0 m, <speed>, N=<n>", never a bare percentage.

### The build plan (ranked by honest Pk per unit effort; each lever behind a flag, lab-first then Gazebo A/B, per the "lab ranks, Gazebo decides" pattern)
1. **Yaw-rate authority (seat C, highest leverage, near-free).** First VERIFY on this PX4 build (dump params) whether yaw is param-capped (`MPC_YAWRAUTO_MAX`) vs. attitude-loop-bandwidth-limited — 10 min, avoids a dead end. If capped: add a yaw-rate param to the FPV bundle (same runtime-set + read-back mechanism as ADR-0010 #3, logged per run). Attacks the ROOT cause of detection loss (boresight can't track LOS).
2. **Split the terminal freeze + rate-cap lambda_dot (seats A/B/C converge).** Replace the whole-vector `frozen_vworld` latch with: freeze only the scalar `v_perp` magnitude; keep `v_close` (via `compute_v_close(r_hat)` — r_hat is monotone, never singular) and yaw and `lambda_hat` LIVE off fresh detections; reconstruct `vh0,vh1 = v_close*(cos lam_hat, sin lam_hat) + v_perp_frozen*(-sin, cos)` each tick. Cap `|lambda_dot_hat|` (~60-75 deg/s, tune) in **BOTH** `a_cmd` AND `lambda_filter.predict()` — applying it only to a_cmd and not to the filter's forward integration is the single easiest way to silently recreate the exact ADR-0009 whipsaw the freeze was built to prevent (seat C's biggest-risk flag). Reclaims the ~0.25 s of valid post-freeze detections one CSV shows being discarded today. **Requires a `--bench`-style near-R=0 regression check before any real flight is trusted.** Optional secondary: a SHORT (<=0.3-0.5 s), capped lead-extrapolation through the truly-blind tail using the already-running `TargetTracker` (seat A's F3 / the PIP machinery, which is fed every tick regardless of `--law`) — gated behind a minimum-corrections settle streak, opt-in, A/B'd like Kalata/cue-latency-comp.
3. **Slow terminal closing (seat C, cheap complement).** Sweep `V_CLOSE_TERMINAL` 5.5 -> 4.0-4.5 m/s: more 14 Hz detections per meter closed + smaller m/s multiplier on residual blind time, at ~zero cost to dash-catch (the catch job is done by `HANDOFF_RANGE`=10 m before throttling bites, ADR-0010 #2). Confirm it doesn't recreate the M4 too-slow-closing tail-chase; don't push below ~4 m/s without a clean-geometry run.
4. **Then the properly-powered Monte-Carlo (this IS M5's miss-distance batch).** Run the winning config at N>=100/speed across 6/8/10 m/s; produce Pk-vs-R per speed with Wilson CIs + the miss CDF. Infrastructure built this session: `scripts/mc_batch.sh` (fresh-boot-per-flight runner, varies seed+geometry) + `scripts/mc_analyze.py` (Pk-vs-R curve, Wilson CIs, plots).

### Honesty boundary (the one rule)
The >=95% claim is dishonest if reaching it required: (a) altering previously-validated sensor/target geometry (tag size/position, camera FOV/resolution) without re-earning and disclosing the milestones that depended on it — laundering "made the problem easier" as "improved the algorithm"; (b) choosing R_lethal AFTER seeing misses to clear a threshold, rather than fixing it a priori and publishing the full distribution; (c) a single pooled Pk hiding a worse number at the hard (high-speed) end; or (d) a percentage from too few trials to separate it from a materially lower true rate. Every retained simplification — fiducial-only lock, static board orientation, mocked cue, and the assumed (unmodeled) R_lethal — is stated plainly next to the headline number in the README, not buried in an ADR. The sim's ZERO sensor degradation (no motion blur / rolling shutter / vibration / lighting variation — flagged in ADR-0007/0010/0012) means any Pk here is an UPPER bound on a real seeker's; disclose that.

### Dissent / warnings carried forward
- **Seat B (scope-skeptic):** leans to fold Pk-vs-R into M5 rather than chase 95% as a standalone target, and refuses R >= 2.0 m headline. Adopted in spirit — decisions #1/#2 make the curve (not a target number) the deliverable, and the Monte-Carlo IS the M5 batch, so this is not a separate detour from M5.
- **Seat A/C:** the loss mechanism is geometric/kinematic (yaw-rate + LOS singularity), so it's plausible the terminal fixes help at 6 m/s but the honest curve still degrades below 95% before 10 m/s — the right response is to report the degrading curve, not keep pulling levers (esp. nested-tag/FOV) until a flat number appears.
- **Seat C verify-first flag:** if a param dump shows yaw is already uncapped (attitude-bandwidth-limited), the yaw-param lever is a dead end — fall back to slower terminal closing / more standoff.
- **Deferred sub-decisions (validated by dev runs, not derivation):** the lambda_dot cap value, the yaw-rate param value, exact V_CLOSE_TERMINAL, and the lead-extrapolation settle-streak are proposals.
- **Date:** 2026-07-05. (Council: 3x council-member @ max effort, seats briefed as guidance/honesty, scope-skeptic, guidance-depth; independent briefs + two-CSV traces; synthesis by main session. Execution autonomous while builder away.)

## ADR-0014 addendum (2026-07-05) — lab lever study + baseline Pk batch: 95% at a defensible radius is NOT reachable at FPV crossing speed
- **Lab lever ranking (guidance_lab.py `--adr0014`, n=150-300/config; baseline byte-identical, re-verified [audit 2026-07-06: byte-identical in stdout and in every prior CSV VALUE; the CSV header gained an additive `terminal_coverage` column at this commit — no existing number or RNG stream changed. "Byte-identical baseline" claims are hereby scoped to stdout + prior-column values]):** of the three ADR-0014 terminal levers, (1) **yaw-rate authority** raises terminal detection COVERAGE +10-15% but barely moves miss (-0.6% pooled S2; -3.4% on camera-only pure_pn) — evidence the FPV-speed miss floor is KINEMATIC (time-to-go / vehicle authority), not detection-limited; (2) **split-freeze + lambda_dot cap** is tiny in the lab (~0.1%), BUT the lab cannot model the ADR-0009 whipsaw (point-mass has no commanded-direction->achieved-velocity-collapse coupling), so it UNDER-values the fix — worth one Gazebo A/B, not more; (3) **slower terminal closing is monotonically WORSE** (recreates the too-slow tail-chase ADR-0009 already fixed) — rejected. **Decisive lab finding:** none of the three close the gap to a **PIP terminal law** (already built, `--law pip`), which beats pure_pn+all-levers by ~40% mean miss at 6 m/s in the lab — so the terminal law choice, not these micro-levers, is the lever that matters. (Lab caveat: Gazebo's own S2 gate showed pip 2.27-2.29 vs pronav 1.99-2.34 — roughly TIED, not pip-40%-better — the recurring lab-over-optimism; only a Gazebo Monte-Carlo of both laws can rank them for real.)
- **Baseline Monte-Carlo (scripts/mc_batch.sh + mc_analyze.py; pronav, 6 m/s, N=20, 10 L2R/10 R2L, distinct cue seeds + geometry jitter; `logs/mc_batch_20260705T225008Z.csv`):** 20/20 clean + handoff. Miss mean **2.189 m** (median 2.296, std 0.614, min 0.949, max 3.425, p90 2.749, p95 2.906). **Pk vs R_lethal (Wilson 95% CI):** R=1.0 m -> **5%** [0.9,24]; 1.5 m -> 15% [5,36]; 2.0 m -> 35% [18,57]; 2.5 m -> 70% [48,86]; 3.0 m -> 95% [76,99]. Plots: `plots/pk_vs_radius_20260705T231001Z.png`, `plots/miss_cdf_20260705T231001Z.png`.
- **THE universal failure mode (all 20/20 flights):** `lost detection for >1.0s inside terminal range` — terminal camera dropout right at CPA. n=20 confirms this is the SYSTEMIC limiter, not variance. **-> intercept accuracy at FPV speed is gated by PERCEPTION (terminal detection cadence/hold), not by the guidance math.** This is the empirical hook for the builder's redirect to design the perception half first (a better real seeker / fused track directly raises Pk); it also says the honest guidance deliverable is a **Pk-vs-(radius, speed) curve**, headline R=1.0 m, showing 95% holds only at lower speeds (S1 already got sub-meter at 3 m/s from hover) and degrades through the FPV band — a disclosed, defensible finding, NOT a number to reverse-engineer the radius toward (ADR-0014 honesty boundary).
- **Status:** Pk-dialing PARKED per builder (2026-07-05) pending the perception design — the real track's rate/accuracy/latency/dropout will change the guidance-tuning constraints, so finalize perception first (see NEXT.md perception roadmap + the perception-design council). Infrastructure (mc_batch/mc_analyze, the lab lever variants) is committed and ready to resume.
- **Date:** 2026-07-05.

## ADR-0015 — Perception-half architecture for the real interceptor (COUNCIL, 3 Opus members)
- **Context:** Builder redirect (2026-07-05): design the PERCEPTION half of the real deployable system (the sim proved guidance/control with an AprilTag stand-in; the real target is a small, fast, non-cooperative FPV drone with no fiducial) BEFORE finalizing the intercept math, because the real track's rate/accuracy/latency/dropout become new data constraints on guidance tuning. Builder hypothesis: ground stereo gives RANGE, onboard cam gives BEARING, fuse mid-course; terminal comms-denied. 3 Opus council-members (cost-first, capability-first, integration/data-constraint) researched 2026 parts + methods. Strong convergence on the core; forks resolved below.

### Unanimous decisions (all 3 seats)
1. **Onboard compute = Raspberry Pi 5 + Hailo-8/8L NPU (~$70-110), NOT Jetson-on-air, NOT Pi-CPU-only.** This OVERTURNS ADR-0012's Pi-5-CPU choice and re-grounds its Jetson rejection. Measured: Pi-5-CPU YOLOv8n ~13 fps / 76 ms (too slow for a ≥30 Hz FPV terminal); Pi 5 + Hailo-8 ~35 fps / 29 ms (real-time). Crucially, **real ML does NOT force ROS 2** — HailoRT ships a standalone Python API (`pyhailort`/`hailo_platform`), no ROS 2, no GStreamer required — so ADR-0012's "reject Jetson because its ML path is Isaac ROS = ROS 2" logic was correct for a FIDUCIAL but does not generalize; a $70 Hailo hat closes the ML gap while preserving EVERY ADR-0012 decision (MAVSDK/TELEM2 UART, fixed-forward cam, X500/7" airframe, no ROS 2). Jetson Orin NX goes on the GROUND (free power/weight, TOPS for EO+IR fusion). Confidence: HIGH.
2. **Detect-then-track, not single-frame YOLO.** Holding a lock on a small (few-px), fast, blurred, maneuvering drone needs: motion proposals (cheap, surfaces tiny movers) + a small ML classifier (drone-vs-bird) + a correlation tracker (KCF/CSRT) + a Kalman/alpha-beta track. Evidence: detect-then-track raised tiny-target recall 0.405->0.861 at 0.981 precision. A pure detector does not survive the terminal.
3. **Fusion mechanization:** the ground node emits a GLOBAL-frame (geodetic/ECEF), source-GPS-timestamped, covariance-carrying track at >=10 Hz; the drone converts to its OWN NED (own-state, legal per the honesty boundary) and fuses with its own bearing in an EKF. Bearing-only onboard is sufficient for pro-nav (it consumes LOS *rate* = an angle). Reuse the built cue-latency compensation (OOSM: advance a stale sample along its velocity by its known age). Confidence: HIGH.
4. **RF detection is defeated by fiber-optic / autonomous FPV drones (RF-silent, a real 2026 threat)** — which is the strongest justification for the whole EO/IR + continuous-track + onboard-terminal thesis: against an emit-nothing target, early EO/IR (+radio/acoustic cue) detection and an onboard seeker are THE answer. RF/SDR is a cheap cue layer only.
5. **Existential risk (all 3, sharpening ADR-0012):** reliably ACQUIRING and HOLDING an onboard bearing lock on a small, maneuvering, comms-denied FPV drone through motion blur + airframe vibration in the final ~1-2 s. Every other subsystem exists to deliver a clean lock into that window and shorten it. No BOM dollar closes it; the design de-risks it (throttled terminal speed, global shutter + short exposure, detect-then-track, warm handoff, and the ADR-0014 yaw-rate lever) or the intercept fails. The sim's 20/20 terminal-dropout-at-CPA finding (ADR-0014) is this risk showing up already.

### Forks resolved by main-session synthesis
- **Thermal: STAGE it (cost seat vs capability seat).** Cost-first: EO-only, day-only (thermal ~$2,100 roughly doubles the ground budget). Capability-first: EO+LWIR fusion is MANDATORY for a defensible claim (night, dawn/dusk, low-contrast sky, and bird-rejection — the #1 fielded-C-UAS failure). Resolution: EO-only for the PROOF/bring-up rig with the day-only limitation DISCLOSED; EO+LWIR fusion (FLIR Boson 640-class) as the capability upgrade for any all-conditions claim. Matches the project's staged ethos (ADR-0012 Stage 0-3).
- **The "range from ground / bearing from drone" framing is imprecise (integration seat, adopted).** BOTH sensors are angle-strong, range-weak: ground stereo range error grows as R^2 (`sigma_R ~ R^2*sigma_d/(b*f)`), monocular range is size-scaling (~15-30% for a real drone). The fusion's real value is (a) the ground rig covers the MID-COURSE where onboard can't yet resolve a small target, and (b) the wide cross-PLATFORM baseline triangulates range far better than either monocular. AND: once the ground track is in the drone's frame, error is dominated by cross-platform DATUM offset (RTK-vs-standard-GPS => 2-3 m common-mode) + CLOCK SKEW, not stereo precision. Enablers: shared RTK base (=> ~0.3-0.5 m relative) + GPS-PPS-discipline all three clocks (PX4-native, sub-us; the real analog of the sim's sim-time fix) + timestamp-at-source + OOSM latency comp.
- **The systemic "this will bite us" coupling (integration seat):** acquisition range is set by seeker FOCAL LENGTH, which trades DIRECTLY against terminal FOV / yaw-rate tracking — the SAME yaw-rate deficit ADR-0014 found causing 20/20 terminal dropouts. Comms-denied budget wants a LONGER lens (acquire earlier before jamming); terminal wants a WIDER FOV (hold the target as LOS rate spikes near CPA). One fixed-forward camera cannot satisfy both. Resolution order: (a) spend ADR-0014's near-free yaw-rate lever first; (b) if the acquisition budget still won't close, add a SECOND wide-FOV terminal camera (Hailo runs two streams) — NOT a longer single lens; (c) accept shorter acquisition + longer blind coast and report the degraded Pk-vs-maneuver curve.
- **Event camera (DVS):** capability seat's high-payoff blur/latency hedge (us-resolution, ~no motion blur, 120 dB, <5 ms). Note as R&D/stretch, co-boresighted with the frame camera; not the primary.
- **Radar (mmWave micro-Doppler):** worth adding as an all-weather cue + range-rate + bird-discriminant layer in the fuller build; optional for the proof rig.

### Handoff continuity — a real gap the sim never tested (integration seat)
The built S2 latch assumes the cue is ALIVE until the camera locks; in reality a JAMMER decides when the link dies. The design must satisfy `R_acquire >= R_cutoff + coast_margin`. If the link is cut before lock, the interceptor must dead-reckon a mid-course coast to a predicted basket, open a bounded seeker search, and acquire in the last ~10-15 m (standard inertial-midcourse -> terminal-acquisition missile pattern); if no acquisition inside t_go, break off rather than fly blind. Handoff should be a WARM lock-TRANSFER (seeker already tracking through the dash, cross-validated against the cue), never a cold acquisition. The sim has ZERO model of this and must add a configurable link-cutoff + dead-reckon-coast branch and gate on it.

### DATA CONSTRAINTS BACK TO THE GUIDANCE MATH (the builder's central ask) — update the sim models to these BEFORE re-tuning guidance
| # | Parameter | Sim today | Real-world value | Sim knob |
|---|---|---|---|---|
| 1 | Cue position sigma | 0.5 m flat | range-dependent sigma_R ∝ R^2; 0.5 m ONLY with shared-RTK+PPS, else 2-4 m; + per-run constant datum bias 0.3-3 m | s2_cue_mock: sigma(R) + fixed offset vector |
| 2 | Cue VELOCITY | not sent (drone differentiates) | send filtered vel + sigma_v 0.3-1 m/s, else differentiating gives ~7 m/s noise (PIP-killer) | add --emit-velocity/--vel-sigma + ingest |
| 3 | Cue latency | 0.12 s fixed | 0.05-0.15 s mean + 0.02-0.08 s JITTER | --latency-s + new --latency-jitter |
| 4 | Cue dropout | none | bursty Markov, 0.5-3 s outages, p_out 5-20% | new --dropout-markov |
| 5 | Cue frame/datum | perfect world->NED (north=world_y is SIM-ONLY) | geodetic->own-NED + relative-GPS bias | inject per-run constant NED offset |
| 6 | Link cutoff before lock | never (alive to handoff) | jammer-controlled cutoff time/range | new --link-cutoff + dead-reckon coast branch |
| 7 | Terminal detect rate | clean ~14 Hz | 20-50 fps but DROPS as f(\|lambda_dot\|, blur) | terminal detector drops on high LOS-rate |
| 8 | Terminal range sigma | AprilTag ~5-8% | ML box 15-30% of range | widen meas_range noise |
| 9 | Terminal bearing sigma | fiducial sub-deg | ~1-2 deg | widen beta noise |
| 10 | Acquisition range | fiducial ~9 m | drone+lens `P_detect(R)`; longer lens acquires earlier but shrinks FOV | parameterize jointly with FOV |

### The build plan (integration seat's prototype-first, adopted — pure SIM, zero hardware, matches "lab ranks, Gazebo decides")
1. **Upgrade `s2_cue_mock.py`** from its 3 knobs to the table above (velocity emission, range-dependent sigma + datum bias, latency jitter, bursty Markov dropout, link-cutoff) — all behind flags, S2 gate default byte-identical.
2. **`m4_intercept.py`:** ingest cue velocity; add the link-loss-before-acquisition dead-reckon-coast + bounded seeker-search + break-off branch; make terminal detection dropout a function of |lambda_dot| (models the yaw-rate-driven CPA loss).
3. **Prototype the realism in `guidance_lab.py` first**, then re-run `mc_batch.sh`/`mc_analyze.py` under REAL track quality to measure how far Pk falls — before spending a dollar or over-tuning a gain.
4. **Then** the cheapest HARDWARE go/no-go (folds into ADR-0012 Stage 0): a Pi 5 + Hailo-8 + global-shutter-cam bench test against a real/proxy fast small target at representative angular rates, outdoors, day+dusk, camera vibrating — measure the 5 numbers (detection Hz, max trackable |lambda_dot| before lock loss, bearing sigma, dropout burst length, latency) and feed them straight into the sim model. Plus a two-clock PPS time-sync bench + a static stereo-vs-mono range-accuracy measurement at 50-150 m.

### BOM deltas (2026)
- **Onboard, over ADR-0012's ~$800 rig: ~$70-110** — Hailo-8L/8 M.2 hat (+ SiK/MANET air radio ~$20-100). Pi 5, Pixhawk 6C, global-shutter cam, airframe unchanged.
- **Ground node: ~$3.2k-7.7k** if thermal-equipped (FLIR Boson 640 ~$1.5-3k dominates), or ~$0.8-1.3k EO-only day-proof; Jetson Orin NX ($400-600) + 2x global-shutter cams on a ~2 m baseline (sub-meter range to ~100 m) + GPS/PPS + MANET link. Radar cue optional (+$0.3-2k).

### Dissents / warnings carried forward
- Cost seat: EO-only is day-only — do NOT claim night/all-weather without thermal; RF cue is defeated by fiber-optic FPV.
- Capability seat: handoff must be a WARM lock-transfer; event camera is the high-payoff terminal-blur hedge; single-frame YOLO is insufficient.
- Integration seat: cross-platform datum+clock skew dominate cue error (not stereo precision) — set the sim cue sigma from THAT budget, never a flat 0.5 m; the acquisition-vs-terminal-FOV trade is the systemic bite; refuse LOCAL-frame cues (the ADR-0006 "same number, wrong parent" bug across two vehicles).
- **Date:** 2026-07-05. (Council: 3x council-member @ Opus/middleground per the model-orchestration policy; independent briefs + web research; synthesis by Fable main session. Pk-dialing stays parked pending step 1-3.)

## ADR-0015 addendum (2026-07-05) — realism cost study in the lab: velocity emission is the #1 lever; naive realism is catastrophic
- **What was built:** all 10 data-constraint rows of ADR-0015's table implemented in `guidance_lab.py` as opt-in perception variants behind a `PERCEPTION_DEFAULTS` dict threaded `simulate(perception=...)` → `Sensor`/trackers. Every new RNG draw is guarded by its feature, so `perception=None` reproduces the pre-ADR-0015 sensor exactly. **Baseline byte-identical (verifier-confirmed — full-stdout 0-line diff vs the pre-worker committed script, not just matching headline numbers):** default sweep pooled camera-only pure_pn 1.786 / apn 1.798 / pip 1.804 / pursuit 1.934 / pn_plus_lead 2.283; S2 dash10/handoff10/pip 0.709 / 1.268 / 2.132; and `--adr0014` produces identical stdout to the old code at every seed count tested. New driver `--adr0015` (default 60 seeds/cell; the study tables below were run at `--adr0015-seeds 80`, writes `logs/guidance_lab_adr0015_*.csv`).
- **Constraint→knob map:** (1) `CUE_SIGMA_BASE_M`+`CUE_SIGMA_QUAD` (σ_R∝R²) + per-run `CUE_DATUM_BIAS_MAG_M`; (2) `CUE_EMIT_VELOCITY`+`CUE_VEL_SIGMA_M_S` (Measurement gains `vel_xy`; trackers ingest it vs the baseline differentiate path); (3) `CUE_LATENCY_JITTER_S`; (4) `CUE_DROPOUT_MARKOV` (2-state chain from p_out, burst_s); (5) `CUE_LINK_CUTOFF_RANGE_M/TIME_S` + dead-reckon diagnostics; (6) `TERM_LOS_RATE_DROPOUT` (dropout ∝ true |λ̇|); (7) `TERM_BEARING_SIGMA_DEG`/`TERM_RANGE_NOISE_FRAC`.

### Sensitivity ranking (each constraint ALONE, PIP terminal, pooled 6/8/10 m/s vs idealized baseline: mean 1.42 m, Pk@1 36%, Pk@2 72%)
| constraint | mean (Δm) | Pk@1 (Δ) | Pk@2 (Δ) |
|---|---|---|---|
| c5 link cutoff 11.5 m | 2.19 (+0.78) | 20% (−16) | 46% (**−26**) |
| c1a cue σ_R(R) | 2.11 (+0.69) | 23% (−13) | 50% (**−22**) |
| c1 σ_R + datum | 2.13 (+0.72) | 25% (−12) | 51% (−21) |
| c4 bursty dropout | 1.58 (+0.17) | 27% (−10) | 70% (−2) |
| c1b datum bias 2.5 m | 1.63 (+0.21) | 38% (+1) | 66% (−6) |
| c3 latency jitter | 1.48 (+0.06) | 38% (+1) | 68% (−4) |
| c7 ML terminal noise | 1.34 (−0.07) | 40% (+3) | 73% (+1) |
| c6 LOS-rate term dropout | 1.39 (−0.03) | 36% (0) | 75% (+3) |

- **The single biggest lever is NOT in that table — it's constraint #2 (velocity HANDLING).** Whether the cue EMITS a filtered velocity or the drone DIFFERENTIATES a noisy position stream swings mean miss **3.16 → 1.01 m at 6 m/s** — a 2.1 m swing that dwarfs the worst single degradation (link cutoff, +0.78 m). This is the headline design requirement: **the ground station must transmit a filtered target velocity, not just position** (a few extra bytes in the same track message).
- **Decisive top-3 for the perception design:** (1) cue EMITS filtered velocity (dominates everything); (2) cue range accuracy σ_R∝R² + jammer link-cutoff timing (each ~−22 to −26% Pk@2; set by ground-rig stereo baseline/RTK and the jam model); (3) terminal seeker detection through CPA (lab-scored ~neutral but that is the lab's known blind spot — Gazebo says it is *the* real failure mode; treat high-priority regardless).

### Honest Pk-vs-R under COMBINED realism (PIP terminal, per speed, mean m / Pk@1 / Pk@2)
| config | 6 m/s | 8 m/s | 10 m/s |
|---|---|---|---|
| Idealized baseline | 0.70 / 74 / 96 | 1.35 / 31 / 86 | 2.19 / 4 / 34 |
| Realistic, DIFFERENTIATE (naive) | 3.16 / 21 / 48 | 3.56 / 19 / 36 | 3.78 / 4 / 28 |
| Realistic, EMIT velocity (honest headline) | 1.01 / 70 / 78 | 1.22 / 57 / 79 | 1.49 / 41 / 72 |
| Realistic + mitigations (RTK 0.5 m + emit + no-jam) | 0.54 / 80 / 98 | 0.77 / 69 / 95 | 1.10 / 51 / 88 |

- **Key result:** naive realism is catastrophic (mean ~3.2–3.8 m), but emit-velocity recovers most of it, and with the full designed mitigations the realistic curve actually **beats** the idealized baseline at 8–10 m/s — because an emitted σ_v=0.5 velocity is cleaner than differentiating even a perfect 0.5 m position cue at the handoff instant. Emit-velocity also makes the dead-reckon coast through a jammer link-cut viable.
- **PN-vs-PIP under realistic velocity (ADR-0011/0015 confirmation):** with EMITTED velocity, PIP beats pure_pn at every speed (Pk@1 70/57/41 vs 57/42/30). Forced to DIFFERENTIATE, both collapse to ~3.1–3.8 m and PIP's lead advantage EVAPORATES (essentially tied) — its lead point is built on garbage velocity. **PIP only wins if the cue emits a clean velocity.**

### Recommended "realistic" defaults ported to Gazebo (s2_cue_mock.py / m4_intercept.py)
cue σ_R(R)=0.4+0.008·R² m; datum bias = per-run constant, 0.5 m (shared RTK+PPS design target) / 2.5 m (standard GPS); **EMIT velocity, σ_v=0.5 m/s (highest-leverage recommendation)**; latency 0.12 s mean + 0.05 s jitter with OOSM comp; bursty dropout p_out 0.12 / burst 1.5 s; link cutoff satisfying R_acquire ≥ R_cutoff + coast_margin; terminal LOS-rate dropout ref 90°/s; ML terminal bearing σ 1.5°, range σ 22%.

### Caveats (what the lab still cannot see — trust RELATIVE ranking, not absolute Pk)
- **The lab UNDER-prices terminal constraints (6,7).** They score ~neutral because the point-mass frozen-coast flies a good established collision course even after losing the terminal camera — no ADR-0009 whipsaw, motion blur, or attitude coupling. In Gazebo these caused 20/20 terminal-dropout-at-CPA misses (ADR-0014). Real ranking puts terminal detection well above where the lab places it.
- **Lab-over-optimism persists (4th data point):** idealized lab Pk@1 at 6 m/s = 74% vs Gazebo mc_batch 5% (ADR-0014). The Gazebo port + mc_batch re-run is what turns these rankings into results.
- ML bearing 1.5° is *cleaner* than the AprilTag-calibrated 6°, so c7 partly improves angle in-lab; Gaussian-per-frame noise misses temporal blur streaks. Link-cutoff cost is highly sensitive to the chosen 11.5 m cutoff (earlier cuts are worse). Datum-bias wash-out assumes the seeker itself has no mount/boresight bias.
- **Date:** 2026-07-05. (Study: Opus sim-realism worker per the model policy's middleground tier; baseline reproduction verifier-confirmed; synthesis by Fable main session. Next: Gazebo port of the same knobs, then mc_batch re-run under realistic params — "lab ranks, Gazebo decides.")

## ADR-0015 2nd addendum (2026-07-05) — Gazebo paired A/B: carry VELOCITY on the ground link (direction confirmed); the lab's 2.1 m swing shrinks to ~0.4 m in Gazebo; a machine-load confound found and controlled
- **Setup (the "lab ranks, Gazebo decides" test of the #1 lab lever):** two paired
  `mc_batch.sh` arms, N=8 flights each, pronav, 6 m/s crosser, S2 dash+handoff,
  master-seed 42 — identical cue seeds and crossing geometry per run index. Both
  arms use the full realistic cue (`S2_CUE_MOCK_EXTRA`: σ_R(R) range-dependent +
  0.5 m datum bias + emit-velocity σ_v=0.5 + 0.05 s latency jitter + Markov
  dropout). Only difference: EMIT arm passes `--cue-velocity` (drone ingests the
  cue's filtered velocity, `logs/mc_realistic_EMIT_20260706T015033Z.csv`); DIFF
  arm differentiates the noisy cue positions itself
  (`logs/mc_realistic_DIFF_20260706T022749Z.csv`).
- **Result:** EMIT mean **1.394 m** / median 1.125 / Pk@1 37.5% / Pk@2 75% vs
  DIFF mean **1.808 m** / median 1.793 / Pk@1 25% / Pk@2 50%. Paired per-seed:
  DIFF worse on **6/8 seeds**, paired Δ = +0.414 m mean (+0.232 median), σ_Δ =
  1.114 m → paired t ≈ 1.05, NOT statistically significant at n=8. Direction
  matches the lab's #1 lever; the effect size is ~5× smaller than the lab's
  3.16→1.01 m swing.
- **The lab's "catastrophic differentiate" (~3.2 m) did NOT reproduce on a clean
  machine — a load confound was found.** The two prior ~3.1–3.3 m DIFF points
  (single n=1 flight; the dead first DIFF batch's run 0 = 3.257 m) were flown
  while two other Claude sessions loaded the box; the clean-machine rerun of the
  SAME seed (26226) gave 2.423 m. ADR-0009's RTF-load sensitivity applies to
  whole BATCHES: cross-batch comparisons are only valid at matched machine load
  (stale sessions killed before this rerun; make that a pre-batch checklist item).
- **Why the effect shrinks in Gazebo:** both arms end camera-only, and ALL 16/16
  flights (8 EMIT + 8 DIFF) still broke off by terminal dropout at CPA — terminal
  perception remains THE limiter (ADR-0014's 20/20, now 36/36 across batches).
  The velocity channel mainly improves dash/handoff geometry; the camera-only
  terminal washes part of that advantage out.
- **Cross-batch observation (needs n≥20 to firm up):** realistic-cue EMIT (mean
  1.394, Pk@1 37.5%, n=8) OUTPERFORMS the older idealized-flat-σ baseline batch
  (mean 2.19, Pk@1 5%, n=20, ADR-0014 addendum) — the Gazebo echo of the lab's
  "an emitted σ_v=0.5 velocity beats differentiating even a clean position cue."
  The velocity channel appears to matter more than all the added realism costs
  combined, though the two batches differ in more than one knob.
- **Decision (unchanged, now Gazebo-backed):** the ground track message carries a
  filtered VELOCITY (a few extra bytes) — direction-confirmed in lab AND Gazebo,
  and it is what makes a dead-reckon coast through a jammer link-cutoff possible.
  Honest claim wording: "consistent direction, modest effect at n=8" — never the
  lab's 2.1 m number.
- **Hygiene note:** commit 47f18d7 accidentally committed a condensed DUPLICATE of
  the first lab addendum, truncated mid-sentence; removed here — the complete
  block above (with the caveats and Date line) is the record.
- **Date:** 2026-07-05 (UTC stamps 2026-07-06T01:50/02:27). Batches run on an
  otherwise-idle machine; analysis `scripts/mc_analyze.py`, paired comparison in
  this addendum traces to the two CSVs above.

## ADR-0016 — Compute setup: hybrid split, track-message link, sourced latency budget (P-3/P-5/P-7)
- **Context:** make ADR-0015's compute split concrete — pipeline map, rates, track-message
  spec, and a sourced three-tier (BEST/EXPECTED/WORST-CREDIBLE) end-to-end latency budget
  the sim can adopt and a bench can later replace 1:1. Full doc: `docs/compute_setup.md`.
- **Options:** all-onboard / central-ground / per-camera-edge / hybrid.
- **Decision: HYBRID.** Ground Jetson Orin NX runs a two-stream small-object detector
  (YOLOv8n/8s-class, TensorRT — ~50-65 fps datasheet, **~15-20 fps/stream honest** after
  small-object input-resolution and two-stream deratings) → stereo triangulation → Kalman
  track emitting **filtered position AND velocity** at ≥10 Hz (the Gazebo-confirmed #1
  lever, ADR-0015 2nd addendum). Air Pi 5 + **Hailo-8 (26 TOPS, $110 — the 8L has no
  headroom for a possible second wide-FOV terminal cam)** runs terminal detect (~105 fps
  NPU-kernel BEST, **~35 fps / 29 ms end-to-end EXPECTED**) + correlation tracker + EKF
  fusion + 20 Hz guidance → MAVSDK/TELEM2 → PX4. Link carries a **~90-byte track message**
  (timestamp, id, global pos, filtered vel, covariance diag, quality): 7.2 kbps @10 Hz =
  ~22% of a $50 SiK radio's 32 kbps — while video (2-4 Mbps) is 60-125× over that link's
  entire capacity. Track-not-video is the architecture, now with the arithmetic.
- **Why hybrid:** heavy compute where power/weight are free; the jam-critical terminal is
  onboard = jam-immune; only hybrid degrades gracefully under jamming (dead-reckon +
  bounded seeker search; the lab prices a link-cut at −26% Pk@2 — worst single constraint
  but recoverable). Central-ground = total loss under jam; all-onboard = no mid-course cue
  (back to the ~3 m/s hover-start cap, ADR-0011).
- **Latency verdict (P-7):** cue-delivery chain (ground exposure → drone fusion) BEST ~20 /
  EXPECTED **~90** / WORST ~210 ms (SiK; MANET ~70 ms EXPECTED). **The sim's 0.12 s ± 0.05 s
  cue model SURVIVES — realistic-to-mildly-pessimistic**, so sim Pk is not flattered.
  Adopt: keep 0.12 s EXPECTED; **add a 0.20 s WORST stress tier** the sim never runs today;
  keep bursty Markov dropout (tail shape > wider Gaussian); OOSM comp worth re-testing at
  M5 Monte-Carlo power. Onboard control chain adds ~48 ms EXPECTED (modeled separately by
  the 20 Hz loop + PX4).
- **BOM staleness flags vs. ADR-0015:** Orin NX 16GB is **~$899 integrated / ~$989 bare**
  post-"Super" refresh (the $400-600 band now only buys the 8GB J4011 ~$599); "SiK/MANET
  $20-100" conflates radios — **SiK ~$50, jam-resistant MANET $1,500-3,000+** — budget as
  separate lines. Hailo $70-110 and RTK/PPS ~$200/end confirmed current. **Ground node
  ~$1,600 EO-only day-proof** (or ~$1,300 with 8GB + budget cams).
- **Date:** 2026-07-05. (No council — synthesis over ADR-0012/0015 + 2026 web sourcing by
  an Opus worker, reviewed by the main session; reversible; every number maps to the
  bench measurement in compute_setup.md §6.)

## ADR-0017 — Ground stereo rig design point (P-2) + correction to the cue mock's noise split
- **Context:** ADR-0015 said "2× global-shutter cams on a ~2 m baseline, sub-meter to
  ~100 m" on faith. This ADR grounds it in physics: `scripts/stereo_model.py` (full
  derivation in docstring; importable by s2_cue_mock) + `docs/stereo_design.md` +
  three plots in `plots/stereo_*.png`. Three tiers per the worse-than-ideal mandate.
- **Decision — design point:** onsemi AR0234 global-shutter (1920×1200, 3.0 µm) ×2 +
  16 mm C-mount (f_px≈5333, HFOV≈20°), **2.0 m baseline**, **hardware GPIO trigger**
  (mandatory — software sync ~1 ms would add ~0.75 m at 100 m vs a fast crosser; with
  the trigger, sync is 0.1% of variance), rigid shaded bar, daily recalibration,
  shared RTK+PPS datum. USB3-with-trigger cameras, NOT MIPI-CSI (a ribbon can't span
  2 m); GMSL2 is the upgrade path. EO proof-rig ≈ **$1,440** (above ADR-0015's
  $0.8-1.3k guess — the rigid mount + RTK + cabling are real money).
- **Why 2 m (the knee):** σ_R from noise scales 1/b, but the calibration-drift term is
  R²·δθ/b with FOCAL CANCELLING — no lens fixes it, only baseline or rigidity. 1→2 m
  nearly halves σ_R (0.85→0.45 m at 100 m EXPECTED); 2→4 m only reaches 0.26 m for a
  much harder mount, and in the WORST tier a floppier long bar makes accuracy WORSE
  (closed-form optimum b≈2.24 m [audit 2026-07-06: that closed form balances only the
  match+cal terms; a dense grid over ALL FOUR terms of the script's own σ_R() puts the
  true WORST-tier optimum at b≈2.95 m — the b=2 m choice is within ~15% of either on the
  accuracy axis and unpenalized in BEST/EXPECTED where σ_R is monotone in b, so 2 m
  stands as a mount-rigidity-vs-accuracy compromise, but "optimum ≈2 m" is imprecise]).
  Matching noise dominates the EXPECTED budget (71%).
- **Envelope (EXPECTED):** σ_R ≤ 1 m to ~150 m, ≈0.45 m at 100 m; detection floor
  ~160 m (accuracy binds, not detection — a balanced rig). Cross-range ≈ 1 cm at
  100 m. WORST: σ_R ≤ 1 m only to ~66 m and detection binds at ~59 m — the honest
  small-target/bad-light number to plan engagements around.
- **CORRECTION to the sim (adopt at the next cue-model change):** the mock's hand-set
  σ_R = 0.4 + 0.008·R² is ~180× too steep as STEREO NOISE (extrapolates to an absurd
  80 m at R=100 m; physics says ~0.45 m). The 2-4 m handoff-range cue error it models
  is REAL but is the GPS DATUM/CLOCK offset — a per-run constant that belongs in
  `--datum-bias-m`, not in c. Adopt: σ_R(R) = a + c·R² with (a,c) = (0.000, 1.08e-05)
  BEST / **(0.000, 4.45e-05) EXPECTED** / (0.214, 1.94e-04) WORST, plus datum-bias
  0.3 / 0.5 / 2.5 m respectively. Consistency check: c·(b·f_px) ⇒ σ_d ≈ 0.48 px
  EXPECTED, matching the assumed sub-pixel matching tier. Past realistic-cue batches
  (ADR-0015 addenda) used the steeper hand-set curve — HARSHER than physics, so their
  Pk is conservative, but the error was mis-attributed (noise vs datum); re-runs under
  the corrected split supersede them.
- **Date:** 2026-07-05. (Opus worker study, main-session review; every number traces to
  `stereo_model.py` output or a cited source — Teledyne stereo-error note, Johnson's
  criteria detection floor, stereo-DIC drift papers, Arducam/ArduSimple/NVIDIA prices,
  pessimistic value taken where sources conflict.)

## ADR-0018 — Mid-course fusion + warm handoff (P-6): built lab+Gazebo, lab says the win is terminal COVERAGE
- **Context:** ADR-0015 #3 decided the mechanization (ground global-frame track fused
  with own camera bearing on the drone); until now the sim's handoff was binary —
  cue steers the dash, then the camera-only terminal starts nearly cold. Built by a
  Fable worker, lab-first per "lab ranks, Gazebo decides."
- **Mechanization (both lab and Gazebo port):** `FusedTrack` — bearing-weighted polar
  fusion. The CAMERA owns the LOS angle (angle-strong sensor, and pro-nav consumes
  λ̇); the CUE's range folds into the range channel inverse-variance-weighted (weights
  from known specs, not fitted); cue's emitted velocity used when fresh; handoff rates
  are geometric (ṙ=(v_t−v_o)·û, λ̇=cross/R), not the still-converging alpha-beta
  rates. Warm handoff seeds the terminal filters (λ, λ̇, R, Ṙ, v_perp) + PIP track
  from the fused state at the latch. Honesty boundary intact: fusion is PRE-latch
  only; the one-way latch (ADR-0010 #5) and camera-only terminal are unchanged.
- **Flags (all default OFF, defaults verified byte-identical):** lab
  `FUSE_MIDCOURSE`/`WARM_HANDOFF` + `--p6-fusion` driver; Gazebo
  `m4_intercept.py --fuse-midcourse --warm-handoff` (require `--handoff`);
  `s2_cue_mock.py --stereo-config B,F_PX,SIGMA_D` (σ_R from stereo geometry —
  placeholder constants until ADR-0017's calibrated values are adopted). Also fixed a
  lab modeling gap: the legacy lab killed the cue at handoff_range; it now stays
  deliverable until a real streak≥3 latch, matching m4_intercept.py.
- **Lab verdict (80 seeds/cell, 2×2 × 3 tiers × 6/8/10 m/s, both terminals,
  `logs/guidance_lab_p6fusion_20260706T030057Z.csv`):** on MEAN MISS fusion is inside
  the ~15% noise floor at every tier (strictly: neutral). But under EXPECTED realism
  it adds a consistent +5 pts Pk@2 (92→97%, both terminals) and — the real signal —
  **terminal detection coverage through CPA recovers from 0.65/0.35/0.08 (6/8/10 m/s)
  to 0.94/0.96/0.96.** Coverage-at-CPA is exactly the lab's documented blind spot
  (frozen-coast still scores well) AND the actual Gazebo failure mode (ADR-0014's
  20/20, now 36/36) — so the lab likely UNDER-states fusion's Gazebo value.
  Non-fragile per the worse-than-ideal mandate: wins at EXPECTED, inert at WORST,
  sub-noise cost at BEST. Datum-poisoning probe: a 2.5 m-datum cue did NOT poison the
  camera's clean bearing (bearing-weighted design working as intended; 1.31→1.25 m,
  coverage 0.44→0.94). Warm handoff alone: sub-noise (slightly positive for pure_pn),
  rank below fusion, keep opt-in.
- **Honest limit:** at WORST the jammer cuts the link at 11.5 m — BEFORE camera
  acquisition (~8 m) — so the fusion window never opens and all variants tie. That is
  the ADR-0015 `R_acquire ≥ R_cutoff + coast_margin` constraint biting; the mitigation
  there is the dead-reckon `--coast-search` branch, not fusion.
- **Next:** Gazebo A/B on an idle machine (matched-load rule), paired seed 42, arms
  {baseline+cue-velocity, +fuse, +fuse+warm}, primary metric terminal coverage +
  dropout-at-CPA rate. Addendum will carry the verdict.
- **Date:** 2026-07-05. (Fable worker; default-path byte-identity re-verified by the
  main session — `--quick` numeric output matches HEAD exactly; S2 gate re-run before
  commit.)

### ADR-0018 addendum (2026-07-05) — Gazebo A/B: the lab's coverage win did NOT transfer; fusion is a small, non-harmful handoff-geometry gain (5th lab-vs-Gazebo divergence)
- **Setup:** three paired `mc_batch` arms, N=8 each, pronav 6 m/s, master-seed 42,
  identical realistic cue (`--sigma-range --datum-bias-m 0.5 --emit-velocity
  --vel-sigma 0.5 --latency-jitter-s 0.05 --dropout-markov`), idle machine.
  BASE = `--cue-velocity`; FUSE = `+--fuse-midcourse`; FUSEWARM = `+--warm-handoff`.
  CSVs `logs/mc_p6_{BASE,FUSE,FUSEWARM}_2026070*.csv`.
- **Result — miss (paired vs BASE):** BASE mean 1.249 / median 1.066; FUSE 1.161 /
  1.026 (**Δ −0.088 m mean, −0.081 median**, better on 5/8 seeds); FUSEWARM 1.175 /
  0.991 (Δ −0.074 m, better on 4/8). Direction is right and consistent but the delta
  (~0.08 m) is **well inside the ~1 m run-to-run terminal-dropout noise** (σ_Δ 0.18-0.21) —
  not significant at n=8. Pk@1 4/8 and Pk@2 7/8 in ALL three arms.
- **The lab's predicted mechanism did NOT reproduce.** Lab said fusion's win is
  terminal coverage-at-CPA (0.65/0.35/0.08 → ~0.96). In Gazebo mean terminal
  coverage is **flat: 0.131 / 0.131 / 0.135** across BASE/FUSE/FUSEWARM, and **all
  8/8 flights in every arm still break off by terminal camera dropout at CPA**
  (36/36 → now 60/60 across all realistic-cue batches). Fusion did not move the
  dropout at all.
- **Why (the honest read):** fusion is PRE-latch only (the ADR-0010 #5 one-way
  latch is intact and correct). The Gazebo terminal-dropout is a REAL perception
  failure — the camera physically loses the AprilTag at close range / high LOS rate,
  AFTER the latch — exactly where fusion is structurally absent by design. So fusion
  can only improve the dash/handoff geometry (the small, real −0.08 m it delivers),
  and cannot touch the terminal dropout that dominates the miss. The lab's "coverage"
  was a frozen-coast abstraction (ADR-0014/0015 documented blind spot); it measured
  track liveness, not the physical camera hold — the two diverge precisely here.
- **Verdict:** KEEP `--fuse-midcourse` (default off) — a small, consistent,
  never-harmful handoff-geometry improvement, honestly reported as sub-noise at
  n=8; a larger Monte-Carlo (M5) could firm the sign. `--warm-handoff` adds nothing
  measurable on top in Gazebo — keep opt-in, do not enable by default. **The decisive
  takeaway reinforces the whole perception pivot:** the intercept is gated by TERMINAL
  perception (camera hold through CPA), which is POST-latch and comms-denied by
  design — no mid-course aid, fusion included, can fix it. Only a better real terminal
  seeker (detect-then-track, ADR-0015) or a terminal-guidance change that reduces the
  LOS-rate demand (ADR-0014 yaw-rate lever) moves this number. This is the 5th
  documented case of the lab ranking optimistically vs Gazebo (after PIP,
  calibration, Kalata, absolute-Pk); the rule holds — lab ranks, Gazebo decides.
- **Audit note (2026-07-06):** these arms ran under the uncorrected σ_R=0.4+0.008·R²
  cue model — ADR-0017's corrected split (c≈4.45e-05 + separate datum bias) was
  already known but unadopted, and the steep curve overstates exactly the long-range
  cue noise a mid-course fusion would improve. The "coverage win did not transfer"
  verdict stands (the post-latch dropout is cue-independent), but the SIZE of the
  mid-course/fusion lever is potentially understated — re-measure after P-8 adopts
  the corrected constants before treating the mid-course conclusions as final.
- **Date:** 2026-07-05. (Batches on an idle machine per the matched-load rule; analysis
  `scripts/mc_analyze.py` + paired per-seed comparison traced to the three CSVs above.)

## ADR-0019 — Ground sensor modality (P-1): staged-thermal HOLDS, but bird-rejection re-attributes to radar/motion, not thermal
- **Context:** P-1 pressure-tested ADR-0015's asserted-not-sourced claims ("thermal
  mandatory for night AND bird-rejection", "RF defeated by fiber-optic FPV") against
  2025-2026 counter-UAS sources. Full study + verdict table + citations:
  `docs/ground_modality.md`. Pessimistic tier taken where sources conflict.
- **Decision — the staged plan holds:** EO stereo (ADR-0017) is the daytime proof-rig
  primary tracker; add ONE **mono uncooled LWIR core** (not a thermal STEREO pair —
  that doubles the priciest part for night range you can't reliably get) co-boresighted
  with the EO stereo for any night claim. Radar (micro-Doppler) is the all-weather +
  true-bird-rejection upgrade, budget-gated ($50k+ fielded; a ~$300 TI mmWave module
  only *demonstrates* the principle to ~150 m). Acoustic = ≤200 m close-in night cue
  only. RF/SDR stays a cheap cue, never primary.
- **Corrections to ADR-0015 (sourced):** (A) **bird-rejection is owned by micro-Doppler
  radar + motion-signature/ML, NOT thermal** — birds are warm-blooded, so LWIR gives a
  hot blob, not an ID; the council mis-credited thermal for the #1 fielded C-UAS
  failure. Keep thermal for NIGHT; credit radar/motion/ML for BIRDS. (B) "thermal =
  all-conditions" is OVERSOLD — thermal is a night enabler but is blind at dawn/dusk
  thermal crossover (ΔT≈0, twice daily), ΔT-starved on hot days (5-15 °C), and loses
  20-70% range in fog/humidity. (C) **Boson 640 module = $3,558** (GroupGets), at/above
  ADR-0015's thermal top end — a night channel roughly DOUBLES the ~$1.44k EO ground-rig
  budget; budget the real number.
- **Confirmed from ADR-0015:** RF-silent fiber-optic FPV defeats RF/EW detection (NATO
  2025: "EW counter-UAS systems ineffective against [fiber-optic FPV]"; the threat is
  supply-limited but ramping) — which is the strongest argument FOR the whole EO/IR +
  onboard-seeker thesis. EO-only is genuinely day-only.
- **Honest EO-only engagement envelope (reconciled with ADR-0017's 59-160 m + fielded
  data):** ~120-160 m for a CLASSIFIED, bird-rejected track on a 0.3 m FPV in good
  daylight (EXPECTED); ~250-440 m bare detection at BEST (a speck, narrower lens); ~59 m
  WORST (small/side-on, overcast/dusk), against the operational reality of ~40% detection
  at 95% confidence + hundreds of false alarms/day. The proof rig is an honest daytime
  ~60-160 m demonstration, blind at night/crossover — disclosed in the README.
- **Date:** 2026-07-05. (Opus research worker, WebSearch/WebFetch; main-session review
  adopted the proposed block; sources in ground_modality.md.)

## ADR-0020 — Jammer link-cutoff envelope (lab): target MANEUVER sets the coast margin, not the cutoff radius; and ADR-0015's "11.5 m = −26% Pk@2" was a compressed-geometry artifact
- **Context:** ADR-0015 said the comms-denied design must satisfy `R_acquire ≥ R_cutoff +
  coast_margin` and priced a jammer link-cut at 11.5 m as the WORST single degradation
  (−26% Pk@2). Those numbers came from the 15.4 m-standoff S2 geometry. This study maps
  the whole cliff on a realistic ~45 m runway, in `scripts/guidance_lab.py` behind
  `--jam-envelope` (Fable worker; all default-OFF, byte-identical re-verified vs HEAD;
  the driver also adopts ADR-0017's CORRECTED σ_R split — first use of it). Also ports
  m4_intercept's `--coast-search` (dead-reckon to the cue-predicted basket + bounded
  ±20° boresight sweep + break-off) into the lab, semantics matched 1:1.
- **The 11.5 m −26% point does NOT reproduce.** On a real runway an 11.5 m cutoff costs
  ~0 (EXPECTED Pk@2 100 vs 100; WORST 68-69 vs 67-68). It was an artifact of cutting an
  infant track mid-dash in compressed geometry. Real drivers: track maturity at the cut,
  coast distance, and target maneuver.
- **Target MANEUVER is the binding constraint, not the cutoff radius.** Against a
  straight-line (CV) crosser, emit-velocity dead-reckoning coasts cheaply: cliff edge
  (>10-pt pooled Pk@2 drop) is ~40 m by raw Pk@2, 20 m by Pk@1, 15 m by seeker-locked
  Pk@2. Against an s-weave target the edge collapses to ~9 m (total loss by 20 m) —
  the predicted basket is simply wrong once the target turns. Effective coast margin:
  +8 to +33 m (CV, by accounting) but **≈ +2 m (maneuvering).**
- **Design ruling (supersedes ADR-0015's coast_margin optimism):** size the jam claim by
  the MANEUVERING case — `R_acquire ≥ R_cutoff` with essentially NO slack unless
  straight-line flight can be assumed. Emit-velocity coasting buys tens of meters ONLY
  against non-maneuvering targets; it is near-worthless when the target jinks.
- **Coast-search: keep it (default OFF).** Raw-Pk-neutral-to-slightly-negative on CV
  targets (the hold sacrifices the running start), but at deep cutoffs it buys +5-6 pts
  of camera-LOCKED intercepts (30 m: 72→78%; 40 m: 63→69%) and converts blind dead-reckon
  flail into honest break-offs — its value is locked intercepts + honest aborts, invisible
  to raw min-range.
- **Fusion is orthogonal to the cliff** (|ΔPk@2| ≤ 1 pt everywhere): at cutoffs ≥
  acquisition range the both-sources window never opens (ADR-0018's limit, now confirmed
  across the whole cutoff axis); warm dead-reckon state does not move the edge.
- **Honesty audit:** at 20-40 m cutoffs, 21-27% of runs score inside 2 m with the camera
  NEVER locking (blind dead-reckon flybys — physically real min-ranges, legitimate under
  a proximity-fuse metric but with no terminal correction, and the point-mass likely
  flatters them vs Gazebo attitude dynamics). The "locked-Pk@2" column (miss ≤ 2 m AND
  camera locked) is the defensible number for the "own camera finishes the intercept"
  headline. Lab ranks/sizes; Gazebo decides. CSV: `logs/guidance_lab_jamenv_*.csv`.
- **Date:** 2026-07-05. (Fable worker; `--jam-envelope`, 80 seeds/cell, 17.6k runs;
  byte-identity vs HEAD confirmed. Gazebo confirmation queued: paired {no-cut, cut≈15 m,
  cut≈25 m} × coast off/on, primary metric camera-locked-at-CPA rate + link-lost-no-acq.)

## ADR-0021 — Kill mechanism + honest Pk-radius reframe (part of the "solutions that pan out" push): you cannot buy out the perception gap with a bigger warhead
- **Context:** the terminal miss floors at ~1-2 m at FPV speed and won't null. The
  real-missile answer is a lethal radius, not a zero-miss hit. Which cheap kill mechanism,
  what HONEST radius, and does it make the current system "enough"? Research-only (no sim,
  no code). Full study + citations: `docs/kill_mechanism.md`. Radii from physics (drone
  span / net span / grenade data), NOT reverse-engineered to a Pk — ADR-0014 boundary held.
- **Decision:** headline the **kinetic ram (hit-to-kill, $0 payload, 0 g)** — what ~70% of
  cheap real interceptor kills (Ukraine FPV interceptors) already do, matches our 2.5-7 in
  airframe, no fuze. Honest lethal radius **0.5 m BEST / 0.30 m EXPECTED / 0.15 m WORST**
  (prop-disc overlap of two small quads = ADR-0014's "0.5 m aggressive kinetic point").
  Recommend a **small net (R≈1.5 m, ~370 g, ~$100-200, 7 in-class airframe)** as the
  cost-optimal non-explosive FORGIVENESS upgrade for the fast regime, salvo-stacked 3-5×
  for a defensible Pk. **Reject fragmentation** — inappropriate to build for a portfolio,
  and its forgiving radius is gated by a proximity fuze anyway.
- **Reframe result (overlaid on our real miss numbers):** split verdict. **At 2-3 m/s the
  intercept is ALREADY a kill** — M4 pro-nav miss ~0.37 m is inside even the WORST ram
  radius → Pk ~95-100% at R=0.5 m, no reframe needed; the "sub-meter miss problem" there
  was partly judging against a hit-to-kill bar only the ram requires. **At 6 m/s it is
  NOT** — mean 2.19 m gives Pk ~5% at R=0.5 m; a ram can't rescue it, a net lifts
  realistic-cue (~1.4-1.8 m) to ~40-75% [audit 2026-07-06: that ~40-75% is a LAB
  (--adr0015) estimate, never Gazebo-confirmed — and the one component of that lab
  stack later Gazebo-tested (velocity emission) landed at ~1/5 the lab effect size;
  treat as a ranking, and re-derive from the M5 Gazebo Monte-Carlo before headlining]
  and needs the salvo to be defensible.
- **THE load-bearing insight (reinforces the whole perception pivot):** you cannot buy out
  the terminal perception problem with a bigger lethal radius. Sensor-free mechanisms
  (ram, net) forgive only ~0.5-2 m; the ONE mechanism that would forgive the 6 m/s miss
  (a proximity-fuzed charge, R~2-2.5 m) must DETECT the target at that same 2-3 m radius —
  which is exactly the terminal detection ADR-0014 found us losing inside 1 m of CPA
  (20/20). The reframe moves the perception gap from the guidance loop to the fuze; it
  does not close it. Fixing terminal detection (ADR-0015 redirect) stays the real lever.
- **Date:** 2026-07-05. (Opus research worker, WebSearch/WebFetch, every external number
  URL-cited; main-session review adopted the block; part of the 4-lane terminal-solutions
  research — diagnosis / real-world guidance / seeker upgrades still in flight.)

## ADR-0022 (PROPOSED, not ratified) — Real-world terminal guidance: what missiles do, and the cheap levers we've never tried
- **Status: PROPOSAL / research brief.** Records the real-world guidance research
  (`docs/terminal_guidance_realworld.md`). The metric change in it (hit-to-kill →
  proximity Pk) touches GOALS.md's resume-line success definition = a one-way door;
  ratify only with the builder and/or a guidance council. Logged here so the options
  and sourcing are on record.
- **Context:** every FPV-speed crosser loses the terminal tag at CPA, misses ~1-2 m
  (60/60, ADR-0014/0018). Builder asked what real missiles do and what is cheap to add.
- **What real homing does (Q1/Q6, sourced):** hit-to-kill (sub-cm precision, hypersonic)
  is a $M-class technique (PAC-3/THAAD, and even PAC-3 carries a proximity backstop).
  EVERY cheap-to-midrange fielded kinetic C-UAS kills within a few-meter lethal/catch
  radius: Coyote (blast-frag), Ukrainian ZIRKA/Sting FPV interceptors (~2 kg frag),
  APKWS, Fortem DroneHunter (net). Converges with ADR-0021: proximity/net, not hit-to-kill.
- **The technique we have NEVER tried (Q4) — the headline:** a **look-angle /
  FOV-constrained (strapdown) guidance law** (OLAGL-style). The missile literature's
  named problem "narrow-FOV strapdown seeker cannot maintain lock against a high-speed
  target under conventional PN" matches our failure signature; the field's answer is a
  law that BOUNDS the terminal look-angle demand to keep the target in the boresight,
  rather than a wider lens. FREE (software A/B). **CONTINGENCY:** this is the right fix
  ONLY IF the failure is FOV/look-angle escape (target leaves the boresight cone). The
  diagnosis lane (ADR-0023, in flight) is settling from logs whether the tag is lost
  OUTSIDE the FOV (→ look-angle law) or INSIDE it while undetected (→ a detection-cadence/
  blur fix instead). Do not build the look-angle law before that verdict lands.
- **The free software levers, in order (Q2/Q3/Q5):** (1) memory-tracking inertial
  terminal COAST — the missile-standard seeker-blink answer, = ADR-0014 lever #2 done in
  full (split-freeze v_close/yaw/λ̂ live + cap |λ̇| in both command AND filter-predict +
  ≤0.5 s capped lead-extrapolation off the existing tracker); PX4 IMU/EKF already onboard,
  cost ≈ $0. (2) the look-angle-constrained law A/B + free yaw-rate-authority param
  (`MPC_YAWRAUTO_MAX`) + a lead-collision dash geometry. Rejected as costly/ineffective:
  mechanical gimbal, IR seeker, APN (already tested-and-lost, ADR-0011), further terminal
  slowdown. DEFER the 2nd wide-FOV camera (~$60-75) until the free levers are spent.
- **Caveat (ADR-0014):** the residual miss floor is partly KINEMATIC at FPV crossing
  speed, so coverage/lock-retention fixes may hold the lock without moving miss much —
  every lever stays "lab ranks, Gazebo decides," gated by a near-R=0 regression before any
  flight.
- **Date:** 2026-07-05. (Opus research worker, WebSearch/WebFetch, URL-cited, vendor claims
  flagged; PROPOSAL pending diagnosis + builder/council ratification of the metric change.)

## ADR-0023 — Terminal-dropout root cause (the linchpin): the miss is KINEMATIC, locked at handoff — NOT terminal perception. Corrects ADR-0014's "perception-gated" reading
- **Context:** 60/60 realistic-cue fast-crosser flights end "lost detection >1 s at CPA",
  miss ~1.4 m. Before investing in a solution lane we needed the mechanism, not the
  symptom. Per-tick forensics over 41 flights (ZEM, true-bearing→pixel FOV projection,
  detection cadence, board incidence, achieved accel/yaw) + a point-mass perfect-camera
  control. Full analysis: `docs/terminal_diagnosis.md`; scripts in job 28aff4e9 tmp.
- **Finding (verified independently by the main session from the per-flight table):** the
  final miss is **96%+ determined at the handoff/freeze latch, BEFORE the camera-only
  terminal begins** — r²(ZEM@freeze, miss) = **0.990** (main-session recompute 0.990;
  ZEM@last-detection 0.992). The camera tracks the tag to mean **1.69 m / 0.074 s before
  CPA** (219 px tag, 19.5° incidence, no blur model exists in the sim); the ">1 s dropout"
  is ≥85% the outbound flythrough (tag geometrically behind the vehicle), not an early
  loss. FOV-escape is real (LOS rate 485°/s avg, peaks 600-1870°/s, vs achieved yaw ≤
  **124°/s** — a ~10:1 deficit, and note the airframe is NOT PX4-capped at 45°) but happens
  0.037 s before CPA as a CONSEQUENCE of the already-committed miss; blind-window
  contribution to the miss = **−0.03 m (nothing).**
- **The decisive disproof of "perception-limited":** the worst flights held the camera
  locked to short range and STILL missed — DIFF2#5 missed **3.37 m with detection to
  3.47 m**; EMIT#3 missed 2.68 m with detection to 2.72 m. A perfect terminal camera
  (point-mass control, 150 seeds) removes 100% of the dropout but cuts miss only **25%
  (1.003 → 0.755 m)**. The kinematic bound is model-independent: correction capacity
  ½·a·t_go² = ½·8.7 m/s²·(0.41 s)² = **0.72 m**, vs **1.69 m** ZEM delivered at handoff —
  the 0.41 s terminal window is physically too short to fix the delivered geometry.
- **Root-cause split of the ~1.4 m miss:** ~70% kinematic (delivered ZEM > terminal
  capacity); ~20-25% recoverable guidance-mechanization loss (freeze latch discards the
  last 0.20 s; α-β filter settle — realized only 0.28 m of an available 0.72 m); ~2%
  FOV-escape; ~0% blur/scale/cadence.
- **Decision — where the effort goes (reorders every prior solution lane):** treat this as
  a TIME-TO-GO / delivered-geometry problem. Invest, in order: (1) **longer-range
  ACQUISITION** — capacity scales with t_go², so acquiring at ~12 m instead of ~6.5 m
  raises correction capacity 0.72 → ~4.3 m, above even the worst delivered ZEM (4.0 m); for
  the real seeker this is DETECT RANGE, not FOV/resolution/deblur at CPA. (2) **Mid-course
  track quality** (sets ZEM@handoff): the velocity-emitting cue (ADR-0015) is exactly this
  lever; dash lead-law quality belongs here. (3) **Reclaim the ~0.3-0.45 m mechanization
  loss** (ADR-0014 split-freeze / later freeze + warm-settled filters). **Explicitly
  DE-PRIORITIZED by the data:** yaw-rate authority, camera FOV/resolution, motion-deblur,
  higher frame rate, nested tags, and terminal-perception hold generally — including the
  ADR-0022 look-angle law premise (FOV-escape is 2%, post-miss) and the ADR-0018 fusion
  coverage story.
- **Reconciliation:** STRENGTHENS ADR-0021 (miss is kinematically floored ~0.9-1.0 m even
  with perfect sensing at 6 m/s → a lethal-radius kill mechanism is not optional, it's
  required — you cannot null it). CORRECTS ADR-0014 addendum + memory "bottleneck =
  terminal blind window" — the blind window is the symptom, the handoff ZEM is the disease.
  REFRAMES ADR-0015's perception pivot: the perception lever that matters is ACQUISITION
  RANGE (detect earlier) + track quality, NOT terminal hold-through-CPA.
- **Caveat:** the perfect-camera-only-25% figure is from the point-mass lab, which is
  documented to UNDER-price terminal effects — but the load-bearing arguments are the
  Gazebo-log ZEM r²=0.99 and the model-independent ½·a·t_go² bound, not the lab. The ~0.4 m
  reclaimable mechanization loss is the one place better terminal handling still pays.
- **Date:** 2026-07-06. (Fable diagnostic worker, log forensics only; ZEM correlation +
  smoking-gun flights + yaw-cap re-verified independently by the main session; no sim boot.)

## ADR-0024 — Onboard seeker upgrades (P-5 cost/benefit, post-diagnosis): the only seeker lever is a cheaper NARROWER acquisition lens; reverses ADR-0015's wide-FOV plan
- **Context:** P-5 seeker-hardware cost/benefit, re-scoped after ADR-0023 showed the miss
  is kinematic (locked at handoff), not terminal perception. Full study + 29 cited
  part/price URLs: `docs/seeker_upgrades.md`. Every recommendation ranked by $/Pk-point.
- **Decision — cheapest-effective seeker stack:** keep ADR-0012's fixed-forward
  global-shutter mono camera; **swap to a NARROWER/longer acquisition lens (~40-60° HFOV,
  ~$15-35)** so the tag is detectable at longer range (more pixels-on-target at distance →
  earlier lock → bigger t_go, and correction capacity scales t_go²: acquire at 12 m vs
  6.5 m raises capacity 0.72 → ~4.3 m). Detect-then-track + IMU-aid ride free for
  robustness. **Total seeker delta over ADR-0012: ~$15-35**, minus a possible **−$40** by
  dropping Hailo-8→8L (the 2nd stream it was bought for is now rejected).
- **Rejected with cited cost (all target the ~0-2% CPA-hold channel → ≈0 Pk at positive
  cost):** wider-FOV 2nd camera, mechanical/digital gimbal, higher frame rate,
  motion-deblur, event/DVS camera, terminal range sensor. IMU-aid buys ≈0 terminal Pk
  under the corrected diagnosis (blind window = −0.03 m) so it no longer sets a $/Pk bar —
  but it's ~free, keep it.
- **Reversal of ADR-0015 (#287 acquisition-vs-terminal-FOV coupling):** resolved the
  OPPOSITE way — terminal FOV-hold is worthless, so pick the long/narrow lens and WITHDRAW
  the proposed 2nd wide-FOV terminal camera. Lower bound on "how narrow" is the ADR-0017
  handoff basket (RTK → ±2.4° @12 m; standard GPS → ±11.8°), not the terminal endgame.
- **Honesty boundary (ADR-0010):** narrowing the sim camera FOV REOPENS a validated sensor
  parameter (the anti-tag-inflation door) → must re-earn M1/M2 and disclose; every Pk
  delta here is a pre-Gazebo estimate (the sim has no blur, so sim Pk is an upper bound).
  Gazebo-testable directly: narrow the SDF `<horizontal_fov>`, raise `HANDOFF_RANGE`,
  re-run mc_batch. NOTE: most recoverable Pk (~70% kinematic ZEM + ~20-25% mechanization)
  lives in the GUIDANCE/mid-course lane, not seeker hardware — the lens is the seeker's
  complementary contribution to the t_go fix.
- **Date:** 2026-07-06. (Opus research worker, re-run against the diagnosis via steering;
  main-session review; part of the 5-lane terminal-solutions push, now complete.)

## ADR-0025 — RATIFIED: proximity/lethal-radius Pk is the headline success metric (builder decision, ADR-0022 proposal accepted)
- **Context:** ADR-0022 flagged the hit-to-kill → proximity metric change as council-worthy
  (it redefines GOALS.md's resume line). Presented to the builder with the full evidence
  (ADR-0021 kill mechanisms, ADR-0023 kinematic floor ~0.9-1.0 m even with perfect sensing
  at 6 m/s). **Builder ratified the proximity-radius metric on 2026-07-06.**
- **Decision:** the headline success metric is the **Pk-vs-lethal-radius CURVE** (already
  the ADR-0014 plan), with the radius set by a chosen cheap kill mechanism's PHYSICS, never
  reverse-engineered to a threshold: **kinetic ram ≈ 0.5 m** (headline; already scores
  Pk ~95-100% at 2-3 m/s where miss ~0.37 m) and **net ≈ 1.5 m** (the forgiveness upgrade
  for the fast/FPV regime, salvo-stackable). The no-collision-volume disclosure (ADR-0014)
  stays: the sim target is a flat board, so every radius is a disclosed narrative
  assumption. This is consistent with GOALS.md's resume line ("validated via Monte-Carlo
  miss-distance analysis" — proximity Pk-vs-radius IS that); no GOALS.md rewrite needed.
- **What this does NOT change:** the M0-M4 gates stand as-is (M4's <1 m bar already passed
  at 2 m/s). It reframes M5's HEADLINE from "sub-meter miss at all speeds" to "Pk-vs-radius
  across the speed band under the chosen kill mechanism." The guidance-improvement work
  (ADR-0023 Tier-1/2) still matters — it moves the whole curve left — but the bar is now
  honest and mechanism-anchored, not an unforgiving hit-to-kill standard only a warhead-less
  ram requires.
- **Date:** 2026-07-06. (Builder decision via the main session; supersedes ADR-0022's
  "PROPOSED, not ratified" status for the metric specifically. Next build per builder:
  ADR-0023 Tier-1 free guidance reclaim.)

## ADR-0023 addendum (2026-07-06) — Tier-1 free-software reclaim: built, lab-ranked, Gazebo-tested; NONE of the terminal levers move the miss — the kinematic diagnosis holds harder
- **What was built (all default OFF, feature-guarded):** three levers from ADR-0023's
  Tier-1 list — A `--early-handoff` (engage terminal at first solid detection streak
  ~7.6 m instead of the latch ~6.2 m), B `--split-freeze` (keep v_close/yaw/λ̂ live, cap
  |λ̇| at 60°/s in BOTH command and filter-predict, freeze later at 1.5 m), C reuse of the
  existing `--warm-handoff` (seed the terminal λ/λ̇/R/Ṙ filters from the pre-latch track).
  Lab driver `guidance_lab.py --tier1`; Gazebo flags on `m4_intercept.py`.
- **Defaults intact (byte-identity + RUNTIME gates):** lab `--quick` numeric output
  byte-identical vs HEAD; and with the flags OFF, `check_m4` PASS (pronav **0.447 m** < 1 m,
  pursuit 2.532 m) and `check_s2` PASS (pronav **1.734 m** < 2.5 m, no-cheat audits a/b/c
  all green). No default path changed.
- **Lab ranking (7,200 runs, 80 seeds/cell, 3 tiers):** C (warm) won at every tier
  (+7.8% mean-miss at EXPECTED, realized-correction 0.03→0.23 m); B (split-freeze)
  ~neutral (+1.4%) BUT the point-mass lab CANNOT model the ADR-0009 whipsaw, so it
  under-prices B's risk by construction; A (early-handoff) HURT (−14.6% EXPECTED) —
  committing to an immature camera track early costs more than the t_go it buys. Dropped A.
- **Gazebo decides (paired N=12, master-seed 42, realistic cue, per-seed vs BASE):**
  BASE mean **1.247 m** (Pk@1 4/12) → C **1.218 m** (−0.030 m paired, 6/12 better, σ_Δ
  0.085 — within the ~1 m run-to-run noise, NOT significant) → BC (B+C) **1.332 m**
  (**+0.084 m WORSE**, only 2/12 better, σ_Δ 0.112). **Split-freeze shows no benefit and
  is plausibly harmful in Gazebo** [audit correction 2026-07-06: paired t≈2.60, p≈0.025
  UNCORRECTED across this session's ~5 paired comparisons — borderline, and the whipsaw
  mechanism is cited by analogy to ADR-0009, not re-traced in these flights; the original
  "HURTS" phrasing over-claimed. Decision unchanged: default OFF] — consistent with the
  whipsaw the lab couldn't see (6th documented lab-vs-Gazebo divergence, after PIP,
  calibration, Kalata, absolute-Pk, fusion-coverage). Warm-handoff is marginal and
  non-significant, consistent with ADR-0018's "warm-handoff ~null in Gazebo."
- **Verdict:** none of the Tier-1 terminal-mechanization levers move the fast-regime miss
  in Gazebo. All kept **default OFF** as documented null/negative results; flags retained
  for reproducibility. **This STRENGTHENS ADR-0023:** the ~0.3-0.45 m "recoverable
  mechanization loss" the lab estimated did NOT materialize — the miss is even more
  stubbornly kinematic than the diagnosis's optimistic split. The real levers remain
  **Tier-2 (longer-range acquisition → bigger t_go, capacity ∝ t_go²)** and the ratified
  **proximity metric (ADR-0025)** — not terminal tweaks. Recommend Tier-2 (narrow the SDF
  FOV / raise HANDOFF_RANGE per ADR-0024, with the M1/M2 re-baseline) as the next build.
- **Date:** 2026-07-06. (Fable worker built lab+Gazebo; main session ran the paired A/B,
  the regression gates, and the analysis. Batches on an idle machine, matched-load rule;
  note: two batch-orchestration incidents (concurrent-chain sim collision + a self-killing
  pkill) were caught and recovered — the reported arms are the clean re-runs.)

## ADR-0026 — Fable 5 adversarial audit (2026-07-06): core claims HOLD; corrections applied, no reversals
- **Context:** Builder asked for a full audit of the project as it stands. Six parallel
  adversarial sub-audits (verifier/sonnet subagents, refute-not-confirm) ran the brief in
  `docs/audit_targets.md` across the highest-stakes claims. This entry records the verdicts
  so the corrections above are traceable and future sessions don't relitigate settled points.
- **Tier-1 (correctness the project rests on) — all HOLD:**
  - **A. Handoff honesty boundary — CONFIRMED by source trace.** Every cue/`ext_*` read is
    inside the `args.handoff` guarded pre-latch block; the single latch (`m4_intercept.py`
    ~1795) closes the socket and nulls the holder; no `gt_*` feeds any command; SimClockHolder
    is side-effect-free. Gap found: the SCRIPTED audits (check_s2.sh) don't cover
    non-detected-tick commands or a hypothetical `gt_*` leak, and the correlation audit (c)
    can't distinguish a partial blended leak. → hardened this session (static AST test +
    audit (a) extension + residual audit (d); see the audit-hardening commit).
  - **B. Frame/sign conventions — CONFIRMED.** `north=world_y, east=world_x` is the textbook
    ENU→NED swap (world declares ENU in apriltag.sdf:115), "counterintuitive" only vs the
    wrong world_x=north assumption — a naming matter, not a bug. Pro-nav sign nulls LOS rate;
    `--bench` passes bidirectionally. CAVEAT: a real, significant ~0.6 m L2R-vs-R2L miss
    asymmetry exists in the worktree mc_batch (p≈0.01, n=10/dir) — most consistent with the
    fixed-tag-aspect perception effect (first-detection range differs 12.1 vs 8.0 m), NOT a
    sign flip, but it is undocumented and single-batch. The east/world_x axis is never mirror-
    tested (target start_x never flipped). → tracked as an M5 check item.
  - **C. ADR-0023 kinematic diagnosis (the linchpin) — HOLDS-WITH-CAVEATS.** Every headline
    number reproduced independently from raw logs (ZEM@handoff 1.688 vs 1.69; r² 0.957/0.990;
    capacity 0.720 m; a 8.68 vs 8.7). ZEM is provably RTF-invariant; r² robust to differencing
    window. The crux (t_go endogeneity) resolves in the diagnosis's favor: the freeze latch is
    RANGE-triggered (3.5 m, fires while detected in 34/41 flights), so terminal-hold perception
    can't grow t_go — the bound is not circular w.r.t. the claim. Caveats: r²=0.990 at freeze is
    near-tautological (don't cite it as the proof — the capacity bound is); "70% kinematic" is
    contingent on the current ~7.6 m acquisition range; ALL 41 flights are one speed (6 m/s).
- **Tier-2/3 — solid, discipline strong, small corrections applied above:**
  - **D. Lab byte-identity — REFUTED only additively:** the `--adr0014` commit added a
    `terminal_coverage` CSV column; all prior values/RNG bit-identical. Other 4 study flags
    fully byte-identical. AlphaBetaFilter `gain_scale=1.0` is IEEE-exact. One lab-number-as-
    conclusion violation (net Pk) tagged.
  - **E. Stats — Wilson CI implementation correct (hand-reproduced); verdicts HONEST except
    the ADR-0023-addendum BC "HURTS"** (borderline, uncorrected) — softened above. One unnoted
    cross-batch load comparison flagged (low stakes).
  - **F. Cue-mock σ_R gap CONFIRMED:** s2_cue_mock still ships the steep 0.4+0.008·R²; the
    ADR-0017 correction is unadopted (P-8). Disclosure was thin → added to ADR-0018 + the
    build queue; the fusion-lever size may be understated (terminal-kinematics argument
    unaffected — it's post-latch/camera-only).
  - **G. Sourced numbers:** stereo σ_R formula + calibration term correct; two compute_setup
    citations misattributed (INT8 fps, Seeed URL) — struck/corrected. b≈2.24 m is a 2-term
    shortcut (true 4-term optimum ~2.95 m).
  - **H/meta:** M0–M4 gate thresholds verified UNCHANGED since creation (metric change didn't
    soften any gate); mc_analyze reports raw miss AND Pk-curve; radii physics-derived; no
    secrets; ADR numbering clean; one stale citation (−26% jam point) back-propagated; ADR-0023
    forensics preserved into `scripts/forensics/`.
- **Decision:** core thesis (comms-denied camera-only intercept; pro-nav beats pursuit; miss is
  kinematic at FPV speed; proximity metric) stands — nothing reversed. All corrections are
  scope/qualifier edits applied in-place. Open items rolled into the build queue (NEXT.md):
  harden honesty audits (done), the L2R/R2L asymmetry check + a second-speed forensic batch as
  the cheapest thing that would upgrade C to HOLDS, adopt P-8 corrected cue constants, then the
  M5 Monte-Carlo re-derives every lab-tagged Pk in Gazebo.
- **Date:** 2026-07-06. (6 parallel adversarial sub-audits + this synthesis; main session Fable.)

## ADR-0027 — Second-speed forensic batch (ADR-0026 follow-up): kinematic diagnosis GENERALIZES across speed; a NEW acquisition-latch failure appears at 9 m/s
- **Context:** ADR-0026 (audit item C) named a single-speed limitation — all 41 ADR-0023 forensic flights were 6 m/s — and flagged a second-speed batch as the cheapest thing that would upgrade the kinematic diagnosis from HOLDS-WITH-CAVEATS to HOLDS. Ran it autonomously (builder away, authorized): `mc_batch.sh --n 12 --laws pronav --speeds 3.0,9.0 --directions both --master-seed 42`, 24 flights, idle machine. Analysis reused the preserved ZEM computation from `scripts/forensics/terminal_forensics.py` (read-only; adapter in job tmp).
- **Result 1 — the ZEM→miss kinematic law holds and SHARPENS with speed.** r²(ZEM@handoff, final miss): **0.818 at 3 m/s** (n=12), 0.957 at 6 m/s (n=41, ADR-0023), **0.994 at 9 m/s** (n=5 handoff-reaching flights). The r² tracks the correction-capacity/delivered-ZEM ratio monotonically: 0.68 (3 m/s) → 0.43 (6) → 0.07 (9). Interpretation: the more terminal correction capacity approaches the delivered ZEM, the more mechanization scatter loosens the pure-ZEM fit — so r² SHOULD fall as speed drops, and it does. This is corroboration, not contradiction. Capacity ½·a·t_go²: 0.651 m vs 0.960 m delivered ZEM at 3 m/s; 0.273 m vs 4.089 m at 9 m/s (a rounding error against the miss). **The diagnosis generalizes: for every flight that reaches handoff, the miss is kinematically locked, more so as speed rises.**
- **Result 2 — Pk by speed (n=12 each, Wilson 95% CI).** 3 m/s: mean miss **0.821 m**, Pk@1.0 m **92%** [65,99], Pk@1.5 m 100%. 9 m/s: mean miss **3.845 m**, Pk@3.0 m only 8% — the fast end is genuinely not a kill without a large lethal radius, consistent with the whole FPV-difficulty story.
- **Result 3 — THE NEW FINDING (not covered by ADR-0023): at 9 m/s the interceptor often never reaches handoff at all.** Only **5/12 flights latched handoff** (42% [19,68]); the other 7/12 never left DASH. Cause is upstream of the entire ZEM framework: the tag transits the camera's usable detection envelope too fast for the `HANDOFF_STREAK_MIN=3` consecutive-fresh-detection latch to accumulate *within* `HANDOFF_RANGE_M=10 m`. Across those 7 flights only 11 raw detections over ~3,400 ticks — either too far (~14-15 m, outside the 10 m gate) or a 2-frame streak that breaks one short. At 6 m/s handoff was 100% (41/41); this is a detection-cadence-vs-crossing-speed interaction that only bites at higher speed.
- **Decision / implications:**
  1. **ADR-0023 upgraded to HOLDS** (from HOLDS-WITH-CAVEATS) for the flights it covers — the kinematic law replicated at two new speeds. The "70% kinematic at 6 m/s" figure remains contingent on acquisition geometry, as always stated.
  2. **A second, distinct lever is now on the board for the fast regime: acquisition-latch reliability**, separate from terminal correction capacity. Tier-2 (longer-range acquisition lens + raise `HANDOFF_RANGE_M`) helps BOTH slices (earlier lock → bigger t_go AND more flights clear the range gate). But the streak-vs-speed interaction specifically also wants a **lower/adaptive `HANDOFF_STREAK_MIN`** or **higher detection cadence relative to closing speed** — improving terminal kinematics alone cannot touch the 58% that never latch. Added to the build queue.
  3. Honesty note: n=12/speed, ~1 m run-to-run noise; the 9 m/s handoff-regression is n=5 — treat the r² as directional. The stark majority-never-latch pattern (11 detections / 3,400 ticks) is not noise.
- **Date:** 2026-07-06. (Autonomous: sonnet-worker ran the batch + analysis, main session Fable synthesized. Batch on an idle machine, matched-load rule; one self-matching-pgrep watcher bug hit and cleaned up — logged as a gotcha.)

## ADR-0024 addendum (2026-07-06) — Tier-2 pinned: camera FOV 99.7°→60°, HANDOFF_RANGE 10→12 m, streak-min A/B; L2R/R2L asymmetry confirmed benign
- **Context:** ADR-0024 ratified the DIRECTION (narrower/longer acquisition lens); ADR-0027 sharpened WHY (kinematic capacity ∝ t_go² AND a new 9 m/s handoff-latch failure). This addendum pins the VALUE + change list from three parallel offline studies (design scripts in job tmp). Decide-and-log, not a council: the fork was already council-ratified (ADR-0024), the value is evidence-consistent across every angle checked, and it's cheaply reversible (an SDF value + re-run).
- **Decision 1 — FOV = 60° (1.0472 rad), built as a repo-local SHADOW.** fx 539.9 → ~1108.5 px, detection range ~6 m → ~12 m (2.05×). Grounded terminal capacity (scaling ADR-0023/0027's measured windows) 0.73 → 3.08 m at 6 m/s — clears the mean delivered ZEM (1.69 m), not the worst tail (4.0 m); 50° is the next rung if the tail matters. FOV-escape cost of narrowing: ~1.4 cm of capacity (two orders below the gain) — no knee in 40–99.7°, consistent with ADR-0023's "terminal FOV-hold is ~2% of miss." Implemented as `models/mono_cam/model.sdf` + `.config` (full copy of PX4's, only `<horizontal_fov>` changed), resolved via `model://mono_cam` because launch scripts put `$REPO_ROOT/models` first in GZ_SIM_RESOURCE_PATH — same precedent as `models/apriltag_target/` (ADR-0005), PX4's tree untouched (ADR-0001). Maintenance: full-copy shadow goes stale on a PX4 update — diff on upgrade. `camera_intrinsics.json` is NOT load-bearing (guidance reads live `camera_info`); re-record for documentation only.
- **Decision 2 — HANDOFF_RANGE_M 10 → 12 m (a MODEST bump, not 1:1 to R_max).** The lab warned that scaling HANDOFF_RANGE all the way to the new ~12 m R_max reproduces the already-rejected early-handoff pathology (Tier-1 lever A, −14.6%): committing camera-only the instant of first far lock, on an immature track, is worse than staying on the cue-fed dash longer. 12 m buys streak-completion margin without that. Independently, ADR-0027's own logs show real detections at 14–15 m already, so HANDOFF_RANGE=10 was a pure software threshold, not the lens limit — this bump is testable even without the FOV change.
- **Decision 3 — HANDOFF_STREAK_MIN: A/B, do NOT default-change yet.** Lab (own streak-latch harness, m4-faithful reset-on-every-miss; reproduced ADR-0027's 9 m/s latch 38.6% vs the real 42%, inside its CI) ranks two Gazebo A/B arms: (a) LOW-RISK `gate=12, streak=3` → ~51% latch, no latch-quality erosion (fragile 42% vs 48%); (b) HIGHER-UPSIDE `--early-handoff` (`streak=2`) → ~64% latch but 40% fragile (premature-latch exposure — the reason streak=3 exists). **Score the A/B on Pk / raw handoff-rate, NOT conditional-miss** — the old `--early-handoff`-is-negative verdict was measured where handoff was guaranteed, a population ADR-0027 shows 58% of 9 m/s flights never enter. Watch for elevated λ̇/range-filter transients in the first ~0.5 s of ENGAGE (the fragile-latch signature).
- **Decision 4 — L2R/R2L asymmetry (ADR-0026 item B) upgraded to CONFIRMED BENIGN (multi-speed).** Pooled 12 batches / 3 speeds: the pro-nav response is byte-identical mirror-symmetric (fitted gain exactly 5.000, r²=1.000 both directions; residual same magnitude, opposite sign — a correct lead angle, not a bug). The ~0.2–0.44 m miss asymmetry (the audit's single-batch 0.6 m/p=0.01 was the most-powered draw; true effect smaller) tracks first-detection-range asymmetry (L2R 12.1 m vs R2L 8.0 m at 6 m/s), caused by the rigidly world-−X tag board showing mirror-opposite aspect angles (mover sets position only). An axis-mapping bug (ADR-0013) is ruled out: it would bias BOTH directions equally (shared fixed X), not differentially. **No code change.** → disclose as a sim artifact (a real hostile drone is not a flat fixed fiducial).
- **Re-baseline required (honesty boundary, ADR-0010):** M1 unaffected (same resolution/rate); M2 numeric gate re-run (expect PASS, likely better accuracy at 5 m); **M2 detection-ENVELOPE re-measured** with a new committed distance-sweep script (no such script existed — ADR-0007's 6 m was ad hoc; closing that gap); M3/M4/S1/S2 re-run for the record (ranges well inside R_max); all mc_batch/Pk baselines re-run at 6 & 9 m/s (old numbers become the wide-FOV "before"). Disclosure wording drafted into README.
- **Execution order:** (1) smoke-launch to confirm the shadow resolves over PX4's mono_cam (camera_info hfov≈1.047, fx≈1108); (2) M2 distance-sweep re-baseline + check_m2 gate; (3) HANDOFF A/B at 9 m/s (arm a first, arm b if a underdelivers); (4) full mc_batch re-baseline; (5) README disclosure. Each sim phase verified before the next.
- **Date:** 2026-07-06. (Three parallel offline studies — FOV design, asymmetry forensics, streak-min lab — synthesized by main session Fable; the speedup Emerson asked for, applied to front-load the sim campaign.)

## ADR-0024 2nd addendum (2026-07-06) — GAZEBO VERDICT: 60° FOV is TOO NARROW for fast crossers — it HURT the 9 m/s latch (0/12). NOT merged; FOV is a tradeoff, sweep pending. (7th lab-vs-Gazebo divergence)
- **The experiment:** paired mc_batch (master-seed 42, 60° lens + HANDOFF_RANGE=12, worktree `logs/mc_batch_20260706T202500Z.csv`) vs ADR-0027's wide-lens baseline, 3 & 9 m/s, n=12 each.
- **Result — the FOV narrowing BACKFIRED at speed:**
  - **9 m/s: handoff latch 5/12 (wide) → 0/12 (60°)** — every flight failed to hand off and dashed on the cue the whole way (miss ~3.36 m, marginally under 3.85 but MEANINGLESS: it never engaged the camera-only terminal the project exists to prove).
  - 3 m/s: still 12/12 latch, but miss 0.82 → ~1.0 m (slightly WORSE, within noise).
- **Root cause (per-tick forensics, one 9 m/s flight): the narrow field cannot HOLD a fast crosser.** The tag was detected only 2 ticks in 492, BOTH at ~15 m, ZERO within the 12 m gate; max in-gate detection streak = 0 (needs ≥3). first_det_range rose to ~15 m as predicted — but that "extra range" is 2 flicker-detections at the detectability edge, not a track: by the time the fast crosser is inside the handoff gate it has swung off the 60° boresight (the yaw loop can't keep a close, fast-crossing tag centered in a narrow field). **FOV narrowing trades away exactly the acquisition ability the fast regime is bottlenecked on (ADR-0027).** The analytic/lab models priced the range gain but not the angular-tracking cost — 7th documented lab-vs-Gazebo divergence.
- **Decision: DO NOT adopt/merge 60° as-is.** The 60° shadow + HANDOFF_RANGE=12 stay on the worktree branch (NOT merged to main — main keeps the wide lens) as a documented negative result until a sweep finds the sweet spot. Two things narrowing DID buy (M2 pose accuracy 0.086→0.026 m; slow-regime range) do not offset breaking fast-crosser acquisition, which is the harder problem.
- **Reframe of the lever:** ADR-0027's fast-latch fix was "longer-range acquisition OR lower streak-min OR higher cadence." Narrowing the FOV was the wrong way to buy longer-range acquisition — it shortened the *usable* (in-field) track. The remaining live levers for the fast regime are (a) **streak-min=2 on the WIDE lens** (independent of FOV; lab predicted 64% latch — untested in Gazebo), and (b) a **MILD** narrowing (75–85°) that might gain some range without breaking tracking — to be swept empirically at 9 m/s.
- **Next (the sweep, running):** 9 m/s, paired, arms: wide-99.7°/streak-3 (control, re-confirm ~5/12), wide-99.7°/streak-2 (the real acquisition lever), ~80°/streak-3 (mild narrow). Pick the config that maximizes 9 m/s latch + Pk, not detection range.
- **Date:** 2026-07-06. (Autonomous Gazebo A/B + per-tick forensics; main session Fable. The honest negative result IS the finding — narrowing looked right on paper and in the lab, and Gazebo overruled it.)

## ADR-0019 addendum (2026-07-06) — Affordable radar: a COMPLEMENT to the cameras, not a replacement (builder budget question, <$5k)
- **Question (builder):** would a single affordable radar (<$5k, ideally a few hundred $) be "much better" than the cameras? Researched via a 4-agent workflow (radar hardware/physics, modality tradeoff, fielded cheap systems, repo fit) with 2026 pricing + measured RCS sources. Decide-and-log per CLAUDE.md — reversible, an optional add-on to a cue that is already mocked in sim, ~80% pre-covered by ADR-0019. NOT a council (no one-way door).
- **Answer: NO — for both sensing roles, an affordable radar is not "much better," and for the role that matters it is worse or unusable.**
  - **The physics wall:** radar detection range scales as RCS^(1/4). Small drones are 0.01–0.1 m² (MEASURED: DJI Phantom 4 = 0.0314 m² @15 GHz; carbon FPV can approach 0.006 m²). A ~$283 TI IWR6843 single-chip module that reaches ~150–200 m on a car (~10 m²) drops to only ~25–80 m on a small drone, with a coarse ~15° native azimuth resolution (~±13 m cross-range at 100 m). The only peer-reviewed single-chip drone experiment tracked a 60 cm drone at 1–4 m indoors.
  - **The budget dead zone:** nothing useful for small-drone GROUND cueing exists in the $500–5,000 band. Radar that actually detects a small drone at a useful cue range (~1 km) is purpose-built C-UAS gear starting ~$50k (Echodyne EchoGuard) to ~$100k+ (Robin). There is a hard gap between ~$300 hobby modules (tens of m) and ~$50k real radar.
  - **Onboard SEEKER role:** the camera decisively beats cheap radar — pro-nav is driven by line-of-sight ANGLE RATE, and a camera gives sub-degree bearing at a few grams vs radar's ~15° (15–30× coarser). A cheap radar aboard is at best a proximity-fuze / closing-rate / radar-altimeter COMPLEMENT (~$66–283), never the seeker.
  - **Ground CUE role:** radar is conceptually correct for the project's decisive threat (autonomous / fiber-optic FPV that emit NO RF — so cheap RF/SDR detection, ~$30–300, is essentially useless against them), but affordable radar's tens-of-meters range is too short for useful launch-and-intercept lead time. Real cueing radar is $50k+.
  - **What cheap fielded C-UAS / FPV interceptors actually use (2025–26):** EO/thermal cameras + AI, backed by passive acoustic and/or passive RF. Ukrainian thermal-FPV interceptors validate the camera(+thermal-for-night) path. The fiber-optic/autonomous trend is pushing the whole field toward EO/thermal + acoustic (+ real radar for those who can afford it).
- **Decision (refines ADR-0019, does not overturn it):** keep the CAMERA as the onboard seeker (AprilTag stand-in in sim; real EO/thermal + classical-CV/AI later) and the EO (+staged thermal) staring camera as the affordable GROUND cue. A cheap radar does NOT replace the cameras and does NOT change ADR-0019's modality conclusion or ADR-0017's ~$1,440 EO stereo rig — it is an optional ~$66–300 all-weather/night/closing-rate COMPLEMENT, or a documented "what the real system uses and why it costs $50k+" trade study for the portfolio. **Sim impact: none architecturally** — a radar cue is just a re-parameterization of the existing s2_cue_mock (good range/velocity via Doppler, poor/absent bearing precision, all-weather), if we ever model it.
- **Date:** 2026-07-06. (4-agent research workflow + main-session synthesis; sources incl. arXiv 1911.05926 measured drone RCS, arXiv 2011.06730 single-chip IWR6843 drone tracking, DigiKey/vendor 2026 pricing, Echodyne/Robin/DroneShield C-UAS price floor.)

## ADR-0024 3rd addendum (2026-07-06) — TIER-2 VERDICT: 60° FOV rejected, streak=2 adopted for the fast regime, 9 m/s miss reconfirmed kinematic
- **The full Tier-2 acquisition-range experiment concluded across three paired Gazebo A/Bs (all 9 m/s, master-seed 42, vs the ADR-0027 wide-lens baseline of 5/12 latch / 3.85 m):**
  | Config | 9 m/s handoff latch | miss |
  |---|---|---|
  | wide 99.7° + streak 3 + gate 10 (ADR-0027 baseline) | 5/12 (42%) | 3.85 m |
  | **60° + streak 3 + gate 12** | **0/12 (0%)** — never engaged camera | 3.36 m* |
  | **wide 99.7° + streak 2 + gate 10** (`--early-handoff`) | **7/8 (88%)** | 3.34 m |
  (*60° "miss" is meaningless — it dashed on the cue the whole way, never proving the camera-only terminal.)
- **Finding 1 — FOV narrowing to 60° is REJECTED.** It detects farther (~15 m) but cannot HOLD a fast crosser (per-tick: 2 detections/492, both at ~15 m, 0 in the gate). Narrowing trades away the acquisition ability the fast regime is bottlenecked on. 7th lab-vs-Gazebo divergence.
- **Finding 2 — streak-min=2 on the WIDE lens is the real acquisition fix: it ~doubles the 9 m/s latch (42%→88%, n=8), matching the lab's ~64% prediction.** The 7/8 latched flights show kinematic-consistent misses (2.35–4.07 m), no obvious fragile-latch chaos — but n=8 and the lab flagged ~40% fragile, so this is ADOPTED FOR THE FAST/FPV REGIME via `--early-handoff`, NOT yet made the global default (an S2-gate re-check at streak=2 must confirm it doesn't destabilize the passing 6 m/s S2 gate first). Use `--early-handoff` in the M5 fast-regime batches.
- **Finding 3 — the 9 m/s MISS is kinematic, full stop.** Latching 88% instead of 42% barely moved the miss (3.34 vs 3.85 m) because the capacity bound (½·a·t_go² ≈ 0.27 m at 9 m/s, ADR-0027) is tiny against the ~4 m delivered ZEM. No sensing/acquisition change makes a 9 m/s crosser a sub-meter hit from this geometry. The levers remain the **proximity/lethal-radius metric (ADR-0025, net ~1.5 m gets the fast regime)** and/or a longer running-start dash — NOT the seeker. This is the third independent reconfirmation of the ADR-0023 kinematic diagnosis (now across FOV, gate, and streak changes).
- **Decision / reconciliation:** REVERT the 60° experiment from the branch config (shadow SDF, HANDOFF_RANGE 12→10, intrinsics, the FOV README disclosure) so the guidance config returns to the validated wide lens — the 60° change must not sit in the tree as if adopted. KEEP: this ADR chain (the findings), `scripts/check_m2_envelope.py` (envelope-measurement tooling — a keeper regardless of lens), the L2R/R2L asymmetry disclosure (confirmed benign), the radar ADR. The branch then converges to "wide lens + documented Tier-2 findings," cleanly mergeable to main.
- **Portfolio value:** this is a strong negative-result arc — a plausible, lab-endorsed idea (narrow the lens for range) empirically overturned by Gazebo, with the real fix (streak-min) found and the fundamental limit (kinematic) reconfirmed. Exactly the "reproducible, honest, the data decides" story the project sells.
- **Date:** 2026-07-06. (Three autonomous Gazebo A/Bs + per-tick forensics under ultracode; main session Fable. The speedup Emerson enabled ran the whole sweep to a conclusion in one session.)

## ADR-0028 — "Can we do better IRL?" quantified (lab): the running-start beats the airframe (builder question)
- **Question (builder):** the sim's fast-target misses (9 m/s ~3.3 m) look disappointing — "clearly we can do better if used IRL." Offline guidance_lab study (paired, n=80/cell, monkeypatched a_max/tau in a scratch script — repo lab untouched) to bound the upside honestly. Decide-and-log; lab RANKS, a Gazebo confirm decides.
- **Answer: yes, meaningfully — but mostly from the ENGAGEMENT GEOMETRY (running start), not the airframe.**
  - **Running start (the bigger lever, quadratic in t_go):** genuine standoff + a faster ground-launched dash cuts the 9 m/s miss ~47% alone (2.93→1.56 m; paired +1.37 m [1.18,1.56]). Needs standoff room AND a faster dash *together* — acquisition range in isolation is weak/non-monotonic (matches the real Gazebo FOV-narrowing backfire, ADR-0024 2nd addendum).
  - **Airframe agility (real but plateaus):** x500 (8.7 m/s²) → 2× cuts the 9 m/s miss ~27% (2.77→2.01 m), but 3×/30/50 m/s² buy almost nothing MORE — the *guidance law's own* commanded-velocity ceiling (V_PERP_MAX=8) saturates first. CAVEAT: a real FPV rebuild would re-tune those gains higher, so this plateau is partly a lab artifact of isolating one lever — treat as a soft ceiling, not physics. Lag (tau) is a smaller lever (~15% over a 3× cut).
  - **Combined (agile airframe + real running start):** the 12 m/s crosser goes from ~hopeless (4.15 m) to **~1.25 m** (−70%, Pk@1.5 m net-radius 0%→~65%); 9 m/s 2.93→1.32 m (−55%). The running start does most of the lifting; agility's marginal value is bigger at 12 than 9 m/s.
- **The honest boundary (critical):** this all uses the lab's clean AprilTag-calibrated sensor — **perception gets WORSE IRL, not better** (real seeker blur/vibration/embedded-CPU + no fiducial; ADR-0012's #1 risk). So these are an **UPPER BOUND on the upside**, not a promise, and a lab ranking (7 lab-vs-Gazebo divergences — one is exactly the acquisition-range lever this touches).
- **Strategic implication:** the biggest "do better" lever is the **engagement profile** (ground standby → launch-on-detect → long dash), which is LESS hardware-exotic than "a faster drone" — it's the deployment roadmap (NEXT.md M-1..M-4) and it works with the current x500 in sim (just geometry). This reframes the deployment story around the running start.
- **Next (queued after M5):** Gazebo confirm — re-run the 9-12 m/s S2 gate with a LONGER standoff geometry + faster dash (current x500, geometry-only — no new airframe needed) and a ~2× MPC_ACC bump, and see if the lab's ranking (room >> agility, agility plateaus) survives contact with the real sim.
- **Date:** 2026-07-06. (Offline guidance_lab paired study; main session Fable. Scratch driver in job tmp; repo lab read-only.)

## ADR-0029 — M5 Monte-Carlo (FPV band): pro-nav ≈ pursuit at crossing speed — the guidance-law lever gives way to the kinematic limit (a regime-mapping result)
- **The batch:** `mc_batch --n 8 --laws pursuit,pronav --speeds 6,9,12 --directions both --master-seed 42 --extra-args --early-handoff` (wide lens, streak=2 fast fix). 48/48 flights, `logs/mc_batch_20260706T213437Z.csv`, plots `plots/pk_vs_radius_20260706T222603Z.png` + `miss_cdf_*`.
- **Result — pursuit vs pro-nav by speed (n=8/cell, ~1 m run-to-run noise → these deltas are NOT significant; the finding is that they're TIED):**
  | speed | pronav latch / miss | pursuit latch / miss |
  |---|---|---|
  | 6 m/s | 8/8 · 2.17 m | 8/8 · 2.32 m |
  | 9 m/s | 6/8 · 3.61 m | 7/8 · 3.60 m |
  | 12 m/s | 0/8 · 4.44 m | 0/8 · 4.82 m |
  Pooled Pk-vs-radius (n=48): Pk@1.5 m = 0%, Pk@2.5 m = 27% [17–41], Pk@3 m = 35%; mean miss 3.49 m.
- **THE finding (honest + sophisticated, NOT a failure of pro-nav):** at FPV crossing speeds the pursuit-vs-pro-nav distinction **washes out** — both laws are kinematically limited (ADR-0023/0027), so the terminal *law* no longer sets the miss. Pro-nav's decisive advantage is a SLOWER-target result: M4 at 2 m/s gave pro-nav 0.28–0.44 m vs pursuit 2.0–2.5 m (4.6–7.6×, gated, stands). **The two results together MAP THE REGIME:** guidance-law choice dominates where there's enough time-to-go for a law to null the miss (slow/close); at FPV speed the levers become engagement GEOMETRY (running start, ADR-0028) and kill RADIUS (proximity metric, ADR-0025). This is a more mature systems finding than "pro-nav always wins."
- **Corollaries:** (a) **12 m/s is uncatchable from a hover-start** (0/8 both laws) — empirically nails why ADR-0028's running start is REQUIRED, not optional, for the top of the FPV band. (b) FPV-speed Pk from the current near-hover geometry is poor (Pk@1.5 m = 0%) — the hover-start, not the guidance, is the binding constraint. (c) **streak=2 (`--early-handoff`) held the 6 m/s gate** (8/8 latch, 2.17 m < the 2.5 m tiered bar) — no fragile-latch degradation seen; it can become the S2 default (the open ADR-0024-3rd-addendum item is resolved for 6 m/s).
- **Honesty:** n=8/cell is thin — the Pk CIs are wide and the pursuit/pro-nav deltas are within noise (reported as tied, not ranked). A larger-n batch would tighten the curve; the QUALITATIVE regime finding (law-washout at FPV, 12 m/s uncatchable-from-hover) is robust at n=8. Still the wide-lens, hover-start geometry — the ADR-0028 running-start Gazebo test is the natural follow-up that should move these numbers.
- **Portfolio framing:** headline the M4 pro-nav-vs-pursuit win (the classic result, gated) AND the M5 regime map (where it holds, where kinematics take over) — the pair reads as characterizing a system, not cherry-picking a win.
- **Date:** 2026-07-06. (Autonomous M5 batch under ultracode; main session Fable. Idle-load, master-seed 42.)

## ADR-0028 addendum (2026-07-06) — Gazebo CONFIRMS the running start (rare clean lab-agreement); "more agile airframe" is a NULL — the guidance command ceiling, not the airframe, is the limit
- **Running start CONFIRMED in real Gazebo** (paired n=6/cell, master-seed 42, current x500 + wide lens; `logs/mc_batch_{baseline,runningstart}_adr0028_confirm.csv`). Knobs were EXISTING flags — longer standoff `mc_batch --y0-mag 29.3` (≈30 m initial range vs the S2 default ≈15 m) + faster dash `--dash-speed 16` (vs 10); no FOV/airframe change:
  | speed | baseline (hover-start) | running-start | Δ |
  |---|---|---|---|
  | 9 m/s | 3.61 m, latch 5/6 | **1.90 m, latch 6/6** | **−47.3%** (matches ADR-0028 lab ~47%) |
  | 12 m/s | 4.46 m, latch 1/6 | **2.30 m, latch 6/6** | **−48.5%, uncatchable→catchable** |
  Pk@2.5 m 0%→75%, Pk@3 m →100%. Effect (Δ~1.7–2.2 m) dwarfs the per-arm SD (~0.3–0.6 m) — real at n=6. Baseline reproduced ADR-0029's M5 numbers (methodology sanity check). **This is one of the FEW clean lab→Gazebo AGREEMENTS in the project** (contrast the FOV-narrowing backfire) — the running start is a robust, real lever, and it validates the ground-standby→launch-on-detect→dash deployment concept as the thing that makes the FPV band catchable on an ordinary quad.
- **"Adjust the drone to be more agile" — TESTED and NULL** (the `--accel-boost` knob, new, committed 3cd83bf, default-OFF; MPC_ACC_HOR_MAX 12→20, MPC_TILTMAX_AIR 60→70, param read-back confirmed it took). Same paired seed on top of the running-start config: miss 1.39→1.52 m (within noise, if anything worse). **Root cause (the important finding): the interceptor achieves only ~6.7 m/s² lateral — well UNDER even the default 12 m/s² cap — so it is NOT airframe-limited.** Doubling the ceiling can't help something that isn't hitting the current ceiling. **The binding constraint is the GUIDANCE COMMAND CEILING (V_PERP_MAX=8 / V_TOTAL_MAX=13 m/s lateral-velocity clamps), not the airframe.** So a hotter/more-agile drone buys nothing here; the lever to squeeze more is RE-TUNING THE GUIDANCE to command harder turns — exactly ADR-0028's lab-predicted "agility plateaus because the guidance ceiling saturates." Confirms the lab.
- **Decisions:** (1) ADOPT the running-start geometry (longer standoff + faster dash) as the FPV/deployment engagement profile — it's the headline "do better" lever, real and Gazebo-confirmed. (2) `--accel-boost` stays as a documented NULL knob (default OFF) — don't chase a more agile airframe. (3) Next lever to test: raise the guidance V_PERP_MAX/V_TOTAL_MAX command ceilings on the running-start config — does letting the guidance turn harder squeeze the fast miss further, now that we know the airframe has headroom the guidance isn't using?
- **Honesty:** n=6 thin (robust given effect size); still the clean AprilTag sensor (perception is WORSE IRL — this is the sim's honest upper bound). Geometry-only; the x500 flew it unmodified.
- **Date:** 2026-07-06. (Autonomous Gazebo A/B under ultracode; worker ran the sims, main session Fable synthesized + wrote the ADR.)

## ADR-0030 — The real "get further" lever was the DASH TRACK (not the terminal); and an honesty correction: the running start was leaning on an idealized cue
- **Context:** builder pushed "there has to be a way to get further" past the FPV kinematic wall. A 5-agent analysis workflow + a paired Gazebo A/B (n=6/cell, 9+12 m/s, master-seed 42, running-start geometry) settled it. Two of my three original compound guesses were KILLED by the analysis before flying: raising the terminal command ceiling V_PERP_MAX is a lab-confirmed NULL (the terminal only ever demands ~3-4 m/s lateral, far under the 8 cap), and a PIP terminal law is not the main lever.
- **The real lever — the DASH TRACK (forensics + confirmed):** a perfect collision course EXISTS at handoff (counterfactual ZEM=0.00 in every flight) — the interceptor just doesn't fly it, because its mid-course VELOCITY track never converges (reaches ~5 of 9 m/s at latch; ~70-75% of the delivered ZEM). Root causes: the position-only cue's α-β velocity estimate can't track the mover's velocity step; ~0.12 s cue latency = ~1.1-1.6 m position lag; and a real BUG — the "16 m/s dash" silently flew 13 (V_TOTAL_MAX=13 clamped it while the lead-solve assumed 16).
- **HONESTY CORRECTION to ADR-0028 (this is the important one):** the running-start −47% was flown on an IDEALIZED cue (flat 0.5 m σ, fixed latency, no dropout/datum-bias/noise). Under a REALISTIC degraded cue (--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov), the running start alone degrades badly: 9 m/s 1.90→**2.93 m** (+54%), 12 m/s 2.30→**3.08 m** (+34%), and **33% of 9 m/s flights never reach handoff at all** (mid-course track collapse the idealized cue never showed). The running-start ARCHITECTURE survives; its published magnitude was optimistic.
- **THE FIX (velocity emission + dash-unclamp) — decisively gets further AND survives realism:** cue emits a filtered VELOCITY (--emit-velocity + --cue-velocity) + `--dash-unclamp` (new knob, V_TOTAL_MAX 13→18, dev-verified the dash now flies 16.0 m/s). Under the SAME realistic cue: 9 m/s **1.19 m**, 12 m/s **1.48 m**, handoff 6/6 both speeds — it ELIMINATES the dash-abort failure mode and roughly halves the miss (−59%/−52% vs ARM B). Crucially it makes the REALISTIC-cue arm BEAT the old IDEALIZED-cue running-start (1.19 vs 1.90 m @9; 1.48 vs 2.30 m @12). Delivered ZEM@handoff 3.20→1.38 m (~57% drop — mechanism confirmed). Echoes ADR-0015's "emitted σ_v=0.5 velocity beats differentiating a clean position stream."
- **Stats honesty:** pooled n=12 paired delta −1.29 m (t≈−2.67, significant); per-speed n=6 NOT individually significant (t=−1.42 @9, −2.24 @12). The pooled result + the binary handoff-collapse/recovery are the defensible parts; the per-speed absolute numbers are directional. A larger batch would tighten CIs before any headline.
- **The honesty boundary (disclose):** the fix leans on the GROUND SENSOR emitting a good velocity track — legitimate (the parent project's stereo+EKF / RTK ground rig, ADR-0016/0017 track message, measures velocity) but a real-system REQUIREMENT, not free. And the terminal is still a clean AprilTag. `--cue-latency-comp` is an untested further lever (ARM D, deferred).
- **Decision + strategic call:** ADOPT velocity-emission + dash-unclamp for the FPV/deployment profile. This is very likely the LAST big guidance lever — the analysis puts the theory floor near here, and the remaining gap is now genuinely the FAKED PERCEPTION half. Next honest work is NOT more guidance tuning: it is (a) stress perception AVAILABILITY (the ARM B dash-aborts show degraded cue causes catastrophic failures — quantify the jam/dropout envelope) and/or (b) SHIP THE PORTFOLIO (demo + writeup), disclosing the perception gap as future work.
- **Date:** 2026-07-06. (5-agent analysis workflow + paired Gazebo A/B; worker ran sims, main session Fable synthesized + wrote ADR. --dash-unclamp knob committed 832966c.)

## ADR-0031 — Perception-availability stress envelope: where the intercept breaks under degraded/jammed cue (the honest limitations number)
- **Context:** the portfolio needs an honest "where does it break," not a hand-wave. Gazebo sweep on the ADR-0030 FIX config (running-start + velocity-emission cue + --dash-unclamp, 9 m/s pronav, n=6/arm, master-seed 42, idle machine) varying the cue degradation. All knobs already exist in s2_cue_mock.py (--dropout-p, --link-cutoff-range-m).
- **The envelope:** the intercept HOLDS (≥83% handoff, mean miss ~1.2–1.5 m) as long as EITHER (a) Markov link dropout stays under ~2× nominal (p≤0.24: 6/6 handoff, 1.18 m), OR (b) a jammer cutoff leaves the cue alive to within ~12 m of the target (cutoff@12 m: 6/6, 1.50 m). It BREAKS beyond ~4× dropout (p=0.48: 5/6, 1.95 m) or a cutoff at ~16 m+ (cutoff@16 m: 50%; @20 m: 17%). Cliff between ~12–16 m of true range remaining when the link dies; camera first acquires ~3–10 m, so the cue needs only a few meters' margin past acquisition.
- **THE finding (and a methodology honesty point):** every catastrophic failure is the SAME mode — `DASH: failed to reach handoff range within 20 s`: the mid-course dead-reckon (no --coast-search here) drifts off, the camera never builds its detection streak, and the flight NEVER REACHES ENGAGE. So **perception AVAILABILITY, not terminal guidance accuracy, is the binding constraint once the cue is degraded.** Critically, `miss_m` is logged even for these blind flybys and is deceptively SMALL (a ballistic closest-approach) — averaging raw miss would FLATTER the broken arms (16 m-cutoff's raw mean 1.12 m looks better than baseline despite only 50% ever reaching handoff). **Handoff-rate, not mean miss, is the honest headline metric under degraded perception** — confirms ADR-0030's own warning.
- **New failure mode logged (4× dropout only):** a late, fragile 2-detection handoff that re-loses the tag far out and hard-aborts (`lost tag >5 s far from target`) — post-latch the one-way cue latch (ADR-0010 #5) leaves no fallback. Not previously in the ADR-0018/0030 taxonomy.
- **Honesty caveats:** n=6/arm is thin (brackets the 12–16 m cliff, can't pin it); the `--link-cutoff-range-m` arms are NOT cleanly paired (a cutoff permanently stops all RNG draws, diverging the stream per cutoff distance) — trust each arm's own n=6 aggregate, not cross-arm per-seed deltas. Dropout-rate arms ARE cleanly paired (identical draw sequence).
- **Decision:** report this envelope as the writeup's limitations number. Do NOT tune further guidance. Documented future lever (NOT built): `--coast-search` (bounded yaw-sweep reacquisition, ADR-0020) is the obvious mitigation against this exact cliff for non-maneuvering targets — a clean next experiment if the project resumes.
- **Date:** 2026-07-06. (Autonomous Gazebo stress sweep; worker ran sims, main session Fable wrote the ADR. Files: logs/mc_{dropout2x,dropout4x,jamcut8,jamcut12,jamcut16,jamcut20}_9ms.csv.)

## ADR-0032 — Portfolio demo video: hero flight captured, plus a real pre-existing race-condition bug found and worked around
- **Context:** finishing the portfolio demo video. Infra (world-name env-var fix, chase-camera sensor, `scripts/demo_capture_frames.py`, `scripts/render_hud.py`, `scripts/compose_demo.sh`) was already built and committed (ecc6568); remaining work was verifying the interceptor's FPV reskin, tightening the chase-camera framing, and capturing one clean intercept flight end-to-end.
- **`INTERCEPTOR_WORLD_NAME` env-var fix (from the prior session, gate-safety re-confirmed here):** `m2_detect.py`/`m4_target_mover.py`/`s2_cue_mock.py`'s hardcoded `WORLD_NAME = "apriltag"` gained an env-var override, default unchanged. Re-verified this session: `check_m2.sh` (which does not set the var) still PASSED against the plain `apriltag` world with numbers matching the pre-existing baseline (below) — the override is a true no-op for every existing gated caller.
- **Interceptor FPV reskin (`models/x500_base`, a resource-path shadow of PX4's own `x500_base` — ADR-0005 mechanism, material-only changes to 9 visuals: dark carbon-black body + electric-blue arm-tip accents): found and fixed a real bug, then verified number-safe.** The model FAILED TO SPAWN AT ALL on first boot — `models/x500_base/model.config`'s `<description>` prose contained unescaped literal `<material>`/`<visual>` substrings, which broke gz's XML parser (`XML_ERROR_MISMATCHED_ELEMENT`) and silently dropped the entire model (cascading into "at least one link" / frame-graph errors and every PX4 sensor showing "missing"). Root cause was NOT the GPU (no `dxgk` errors during that boot) — fixed by de-XML-ifying the prose (removed the angle brackets from the description text; `gz sdf --check` and `check_m2.sh` both re-verified clean after). **After the fix: `check_m2.sh` PASSED, `detection_rate=1.000`, `mean_err_norm=0.0858 m`** (baseline 0.0861 m — unchanged within noise; `logs/m2_detect_20260707T192233Z.csv`). Model geometry/collision/inertial/sensors are untouched (material-only, per the shadow's own header comment) — this result confirms that claim rather than just trusting it.
- **Chase-camera pose, re-derived from real trajectory data, not a guess.** The world file's two prior analytical poses both aimed at an assumed engagement point (~5,0,0.5) that turned out wrong — a real hero-flight CSV shows CPA/ENGAGE actually happens ~12 m further down the running-start corridor, around (6.3,-12.2,1.1)/(6.5,-12.2,0.5). Re-derived the pose from that real CPA region and iterated LIVE (via `gz service .../set_pose` on the static camera model — confirmed working, no reboot needed per pose iteration) against real captured frames: final pose `14.900 -7.612 3.388 0 0.262 -2.647` (~10 m out, broadside to the dash bearing, ~15° pitch) gives a level horizon with sky occupying the upper ~30% of frame and both drones clearly distinguishable at the CPA range. Confirmed working in the real hero flight's own chase footage.
- **A real, pre-existing race-condition bug found (not a new one introduced this session):** `worlds/apriltag_demo.sdf`'s target spawns at its world-file default pose `(5, 0, 0.5)` (same as the plain gated `apriltag.sdf` — not reskin-specific). `m4_target_mover.py`'s own "pre-warm" relocation to the real `--target-start` only fires once the mover subprocess spawns, well into `CUE_WAIT`/`DASH` — by which time the interceptor's always-on camera detection thread had already locked onto the nearby default-position board. When the mover then teleports the tag 30 m away, the tracker's alpha-beta filter doesn't know and coasts on its stale ~5 m estimate while `gt_range` diverges to 25-30 m, guaranteeing a BREAKOFF/abort. **Every gated/ADR-0028/0030/0031 number avoided this because `mc_batch.sh` always pre-places the tag via its own external `gz service` call before launching `m4_intercept.py`** — calling the script directly (as this task, and the module's own CLI docstring example, both do) has no such guard. Confirmed via CSV forensics on 2 failed attempts (seeds 17, 23), both showing the identical signature (`r_hat_m` frozen near 5 m while `gt_range` explodes). **Workaround applied (not a code fix):** an external tag pre-placement `gz service` call before launching `m4_intercept.py`, mirroring `mc_batch.sh`'s own pattern — 3rd attempt (seed 31) with this fix reached a clean intercept immediately. **Flagged, not fixed in code:** `m4_intercept.py` already knows `--target-start` and the world name; it should probably do this pre-placement internally rather than relying on every caller to replicate `mc_batch.sh`'s external step. Full forensic detail in `demo_out/README.md`.
- **Hero result:** `miss_distance_m=1.061, clean=1, engaged=1` (`logs/m4_intercept_pronav_20260707T194623Z.csv`, cue-seed 31, the ADR-0030 FIX config — running-start + velocity-emission cue + `--dash-unclamp`, realistic degraded cue, 9 m/s pronav — with `--early-handoff` retained). Consistent with ADR-0030/0031's published ~1.19 m mean for this exact config.
- **Chase/HUD sync gotcha (new, documented for reuse):** `scripts/demo_capture_frames.py` was started a few seconds before `m4_intercept.py` began logging (deliberate, to not miss takeoff); `compose_demo.sh`'s ffmpeg hstack pairs HUD and chase frames BY INDEX ONLY, with no time correction. Uncorrected, this desyncs the composite by the capture head-start (~4.7 s / ~141 frames here) — worst at the CPA moment, where the HUD would show "closest approach" while the chase pane was still mid-DASH. Fixed by trimming/renumbering the raw chase capture to start at the CSV's own `t_sim[0]` before compositing; verified by pulling a frame at the composite's CPA timestamp and confirming the HUD readout (`t=163.188s`, `RANGE 2.31m`) matches what the chase pane shows. Not yet automated — `compose_demo.sh`/`demo_capture_frames.py` should probably grow a `--sim-t-start` alignment step so this manual trim doesn't need repeating.
- **WSL2 GPU-wedge gotcha (re-confirmed, not re-triggered) + the fix:** a prior session's ~10 rapid boot/kill cycles wedged the WSL2 `/dev/dxg` GPU passthrough (`dxgkio_escape: Ioctl failed: -75` in `dmesg`, camera topics going permanently silent) — the documented fix is a host-side `wsl --shutdown` from Windows PowerShell (drops all WSL distros, coordinate before running), not fixable from inside the Linux guest. This session ran 6 total sim boots (over the ≤4 target, but each was deliberate: 2 for the reskin bug/verify, 1 for chase-cam pose tuning, 3 for flight attempts against the race-condition bug above) with a ≥30s cooldown and a post-boot `dmesg`+camera-topic check every time; every cluster of `dxgk` errors observed correlated with the PRIOR boot's teardown window (confirmed by timestamp), never a live boot's active window — the GPU never re-wedged this session. Discipline (cooldown + post-boot camera-publish check + immediate stop-and-report on any silent topic or new post-boot `dxgk` error) held up as the right operating procedure.
- **Date:** 2026-07-07.
