---
name: px4-gazebo
description: How to launch, verify, and shut down PX4 SITL with Gazebo Harmonic headless for this project (gz_x500_mono_cam). Use whenever starting, checking, or stopping the simulator, or when a run needs the right environment variables.
---
# Running PX4 SITL + Gazebo Harmonic (headless)

You have an NVIDIA RTX 4070 (WSL2), so Gazebo can render GPU-accelerated. Still prefer headless (`HEADLESS=1`) for automated/batch runs; use the GPU for camera rendering and the demo. PX4 and MAVSDK both run here, so connect over local UDP.

## Launch (headless)
From the PX4-Autopilot directory:
- `HEADLESS=1 make px4_sitl gz_x500` - default x500 quad.
- Interceptor with a forward camera: use the `gz_x500_mono_cam` model (via `PX4_SIM_MODEL=gz_x500_mono_cam`, or the matching make target if it exists). Confirm the exact target name in the PX4 source before relying on it.
- Speed up non-visual runs with `PX4_SIM_SPEED_FACTOR=<N>` (start 2-4; drop to 1 if guidance/physics get unstable).
- Pick a world with `PX4_GZ_WORLD=<world>` (empty world is fine early on).

## Verify it is up
- PX4 prints "Ready for takeoff!" once it can arm.
- MAVSDK connects on `udpin://0.0.0.0:14540`.
- `gz topic -l` lists topics - confirm the camera topic exists before bridging frames.

## Shut down cleanly
- Type `shutdown` in the PX4 console, or kill the `px4` and `gz sim` processes. Stop old instances before a new run so ports do not clash.

## Notes
- Rendering warnings = the software (llvmpipe) renderer. Expected here; stay headless.
- Log each run (telemetry / detections / miss-distance) to `logs/` - see CLAUDE.md.