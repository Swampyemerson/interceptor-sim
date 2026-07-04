# NEXT — top of the stack

## Current: M3 — static intercept
Goal: fly the interceptor toward the static AprilTag using MAVSDK offboard control,
using the tag's detected relative position (camera + pupil-apriltags, from M2) as
the only position feedback, and hold a 2 m standoff. Gate: final standoff error
< 0.5 m, logged.

Likely steps (refine before building):
- [ ] Offboard velocity-setpoint control loop over MAVSDK (arm, switch to OFFBOARD,
      stream setpoints) — see MAVSDK docs for the offboard "must stream before
      switching modes" requirement.
- [ ] Wire M2's detection + ground-truth-frame math (scripts/m2_detect.py) into a
      live relative-position feedback signal (tag position in camera optical frame
      -> body/NED offset -> velocity command), holding a standoff distance instead
      of closing all the way to the tag.
- [ ] Convert camera-optical-frame tag position -> body FRD -> NED/ENU setpoint
      frame (GOALS.md coordinate-frame conventions) — reuse the same rotation
      utilities as m2_detect.py (quat_to_matrix, compose) rather than re-deriving.
  - Note: M2's world is static (drone parked on the ground) and never armed/flew,
    so this is the first milestone that actually needs MAVSDK OFFBOARD + the
    detection loop running concurrently — a genuinely new integration surface.
- [ ] `scripts/m3_static_intercept.py` + `scripts/check_m3.sh` gate + verifier + commit.
- [ ] Consider: does the M2 tag placement (5 m, facing -X) still make sense as the
      M3 approach target, or does the drone need a longer straight-line approach
      corridor? The apriltag world/model already supports moving the tag's pose
      live via `gz service -s /world/apriltag/set_pose` for quick iteration.

## Done
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
