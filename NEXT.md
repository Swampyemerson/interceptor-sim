# NEXT — top of the stack

## Current: M4 — moving-target intercept, pursuit vs pro-nav
Goal: intercept a moving tag (straight-line, ≥ 2 m/s). Gate: < 1 m closest
approach, AND pursuit vs pro-nav miss distances compared on the same target
paths (this comparison is the resume line — see GOALS.md guidance arc).

Likely steps (refine before building):
- [ ] Decide how to move the target: `gz service -s /world/apriltag/set_pose`
      streamed from Python (already known to work for repositioning) vs a scripted
      actor/plugin in the world. Reproducible straight-line paths at set speeds.
- [ ] **Council (per CLAUDE.md): pro-nav mechanization + gain N.** How to get LOS
      rate from camera bearings (finite-difference + filter?), how to command
      lateral accel over MAVSDK (integrate accel into velocity setpoints vs
      attitude setpoints), N in the 3–5 range, and how closing speed Vc is
      estimated camera-only. ADR it.
- [ ] Pursuit runner first (M3's controller chases the moving tag — expect it to
      lag and trail), then pro-nav, on identical paths; log closest approach for
      both. `scripts/m4_*.py` + `scripts/check_m4.sh` + verifier + commit.
- [ ] Watch for: tag leaving the FoV during high LOS-rate moments (that IS the
      pursuit failure mode — log it, don't paper over it); detection latency vs
      20 Hz control; PX4 velocity-setpoint lag when the command changes fast.
- [ ] Miss distance definition: min 3D camera→tag (or body→tag?) distance over the
      run from ground truth — decide and keep it identical across both laws.

## Done
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
- Minor: m0_takeoff.py duplicates its final CSV row — tidy if reused as template.
