---
name: sim-debug
description: Root-cause debugging for the PX4 / Gazebo / MAVSDK / gz-transport stack in this VM. Use when the simulator will not boot, MAVSDK cannot connect, camera frames do not arrive, or a run behaves wrongly.
---
# Debugging the simulation stack

Read the ACTUAL error first. Check PX4/Gazebo docs and GitHub issues before changing code. Fix the root cause, not the symptom.

## First questions for ANY "run behaves wrongly" (these found the real bugs here)
- **Wall clock or sim clock?** RTF sags to ~0.3-0.5 under load; anything scheduled/measured in wall time distorts by that ratio (a "2 m/s" wall-scheduled mover moved ~4 m/s in sim terms — ADR-0009's whole saga). The intercept CSVs' `t` column is WALL time; measure per-flight RTF from ground-truth kinematics before trusting durations.
- **Was the machine loaded?** Batches/gates compared across different machine loads are invalid (documented load confound, ADR-0015 2nd addendum). Kill stale sessions; re-run at idle.
- **Is a second sim/batch alive?** Two at once kill each other. Check `ps aux | grep -E 'px4|gz sim'` before booting.
- **Is a subscription blocking gz services?** gz-transport13 Python: a process holding ANY subscription never receives gz service RESPONSES. Service callers must be their own subscription-free process (mover pattern).

## Common failure modes
- Sim will not boot / hangs: an old `px4` or `gz sim` process is still running - `scripts/sim_kill.sh` and retry. Check disk space and that the build finished.
- MAVSDK cannot connect: use `udpin://0.0.0.0:14540` (everything runs here in WSL - no host networking). Confirm PX4 is actually up; some flows need `sleep 5` post-boot before connecting.
- No camera frames in Python: `gz topic -l` for the exact topic (world name is part of the path: `/world/apriltag/...`); use the `gz-transport` Python bindings (not a ROS bridge); confirm the model is `gz_x500_mono_cam`, not plain `gz_x500`.
- Detection works close but fails at range: check the tag material (needs `emissive_map`, ADR-0007) and `quad_decimate` setting.
- Ground-truth numbers look subtly wrong: re-read ADR-0006 (camera_link composes against the MODEL, not base_link — a numeric coincidence makes the wrong chain look right).
- Rendering / GL errors: you have the RTX 4070 via WSL2 - confirm `nvidia-smi` works and `/usr/lib/wsl/lib` is on the loader path; WSL OpenGL can be finicky, so fall back to headless if a GUI run will not render.
- Guidance unstable: recheck coordinate-frame conversions and signs first (ENU world, OpenCV camera z-forward, FRD body, NED setpoints; world→NED is north=world_y, east=world_x — ADR-0013) and units. `m4_intercept.py --bench` spins in place vs the static tag to verify λ̇≈0 (sign/frame sanity) before trusting a flight.

## Escalate
If a fix is not converging after two real attempts, convene the council or ask Emerson - do not stack hacks.
