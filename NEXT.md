# NEXT — top of the stack

## Current: M1 — Camera pipeline
Goal: launch `gz_x500_mono_cam` (airframe 4010), pull camera frames into Python via
gz-transport, save them to disk. Gate: script captures N frames at the expected
resolution, exits 0.

Steps:
- [ ] `sudo apt install python3-gz-transport13` (Harmonic pairs with transport13; confirmed available)
- [ ] Launch sim with camera airframe: `HEADLESS=1 make px4_sitl gz_x500_mono_cam` (or `PX4_SIM_MODEL=gz_x500_mono_cam`) — confirm camera topic with `gz topic -l`
- [ ] Determine camera resolution/intrinsics from the model SDF (`~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/`)
- [ ] `scripts/m1_capture.py`: subscribe to the image topic, convert to numpy, save N frames as PNG to logs/
- [ ] `scripts/check_m1.sh` gate + verifier + commit

## Done
- **M0 (2026-07-04):** toolchain up — PX4 v1.17.0 built, Gazebo Harmonic 8.14, venv
  (mavsdk 3.15.3, pytest, numpy, opencv-headless). Gate passed & verifier-confirmed.

Key facts for a fresh session:
- PX4 at `~/PX4-Autopilot` (v1.17.0, built). MAVSDK on `udpin://0.0.0.0:14540`.
- Boot-complete grep line: "Startup script returned successfully" — NEVER wait on
  "Ready for takeoff!" before MAVSDK connects (NAV_DLL_ACT=2 deadlock; see ADR-0004
  and .claude/skills/px4-gazebo/SKILL.md).
- AprilTag library = pupil-apriltags (ADR-0003, unanimous council).
- Minor: m0_takeoff.py duplicates its final CSV row — cosmetic; tidy if reused as
  the logging template for M2+.
