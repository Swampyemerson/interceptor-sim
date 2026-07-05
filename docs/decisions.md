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
