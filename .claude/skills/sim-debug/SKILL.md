---
name: sim-debug
description: Root-cause debugging for the PX4 / Gazebo / MAVSDK / gz-transport stack in this VM. Use when the simulator will not boot, MAVSDK cannot connect, camera frames do not arrive, or a run behaves wrongly.
---
# Debugging the simulation stack

Read the ACTUAL error first. Check PX4/Gazebo docs and GitHub issues before changing code. Fix the root cause, not the symptom.

## Common failure modes here
- Sim will not boot / hangs: an old `px4` or `gz sim` process is still running - kill it and retry. Check disk space and that the build finished.
- MAVSDK cannot connect: use `udpin://0.0.0.0:14540` (everything runs here in WSL - no host networking). Confirm PX4 is actually up.
- No camera frames in Python: `gz topic -l` to find the exact camera topic and encoding; use the `gz-transport` Python bindings (not a ROS bridge); confirm the model is `gz_x500_mono_cam`, not plain `gz_x500`.
- Rendering / GL errors: you have the RTX 4070 via WSL2 - confirm `nvidia-smi` works and `/usr/lib/wsl/lib` is on the loader path; WSL OpenGL can be finicky, so fall back to headless if a GUI run will not render.
- Guidance unstable: set `PX4_SIM_SPEED_FACTOR=1`, recheck coordinate-frame conversions (ENU world, OpenCV camera z-forward, FRD body - see GOALS.md) and units.

## Escalate
If a fix is not converging after two real attempts, convene the council or ask Emerson - do not stack hacks.