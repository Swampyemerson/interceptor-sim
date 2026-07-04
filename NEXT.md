# NEXT — top of the stack

## Current: M2 — AprilTag detection
Goal: custom Gazebo world with a static AprilTag (36h11) on a stand; detect it in the
live camera feed with pupil-apriltags; compute relative position from tag pose +
camera intrinsics; log detection rate + pose error vs sim ground truth. Gate:
detection rate and pose error thresholds met, logged to CSV.

Steps:
- [ ] pip install pupil-apriltags (pinned; ADR-0003)
- [ ] Tag asset: 36h11 id 0 PNG (AprilRobotics/apriltag-imgs) on a textured plane
      model, documented physical tag size (black square edge)
- [ ] Custom world SDF in `worlds/` + model in `models/`; launch via PX4_GZ_WORLD /
      GZ_SIM_RESOURCE_PATH
- [ ] Intrinsics from the camera_info topic (not hand-derived) — record K
- [ ] `scripts/m2_detect.py`: subscribe camera, detect per frame, log
      (t, detected, tag_pose_cam, ground_truth_rel_pos, error) to CSV
- [ ] Ground truth from gz pose topic (world pose of tag + drone → true relative pos)
- [ ] `scripts/check_m2.sh` gate + verifier + commit

Frame-math caution (GOALS.md conventions): camera optical frame = OpenCV (z fwd,
x right, y down); Gazebo sensor frame = x fwd. The camera mount offset on
x500_mono_cam is (.12 .03 .242) w/ no rotation. Fable reviews the transform chain
before the gate — this is the #1 sign-error risk zone.

## Done
- **M0 (2026-07-04):** toolchain — PX4 v1.17.0 built, Gazebo Harmonic 8.14, venv up.
- **M1 (2026-07-04):** camera pipeline — 10/10 frames @ 1280×960 via gz-transport13.

Key facts for a fresh session:
- PX4 at `~/PX4-Autopilot` (v1.17.0). Launch camera drone: `HEADLESS=1 make px4_sitl gz_x500_mono_cam`.
- Camera: 1280×960 @ 30 Hz, hfov 1.74 rad, RGB_INT8, topic
  `/world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image`
  (instance-suffixed — rediscover via `gz topic -l` if world/model changes).
- venv sees system gz bindings via `.venv/.../site-packages/gz_system.pth`
  (python3-gz-transport13 + python3-gz-msgs10 from apt).
- Boot-complete grep: "Startup script returned successfully" (ADR-0004 — never wait
  on "Ready for takeoff!" pre-MAVSDK).
- MAVSDK: `udpin://0.0.0.0:14540`. AprilTag lib: pupil-apriltags (ADR-0003).
- Minor: m0_takeoff.py duplicates its final CSV row — tidy if reused as template.
