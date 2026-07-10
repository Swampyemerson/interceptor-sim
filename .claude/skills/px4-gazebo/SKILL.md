---
name: px4-gazebo
description: How to launch, verify, and shut down PX4 SITL with Gazebo Harmonic headless for this project (gz_x500_mono_cam, apriltag world). Use whenever starting, checking, or stopping the simulator, or when a run needs the right environment variables.
---
# Running PX4 SITL + Gazebo Harmonic (headless)

You have an NVIDIA RTX 4070 (WSL2), so Gazebo can render GPU-accelerated. Still prefer headless (`HEADLESS=1`) for automated/batch runs; use the GPU for camera rendering and the demo. PX4 and MAVSDK both run here, so connect over local UDP.

## Launch (headless) — the project-standard line
From `~/PX4-Autopilot` (v1.17.0):

    PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH=~/interceptor-sim/models HEADLESS=1 make px4_sitl gz_x500_mono_cam

- `worlds/apriltag.sdf` must be symlinked into `~/PX4-Autopilot/Tools/simulation/gz/worlds/` — `scripts/check_m2.sh` creates/repairs it (ADR-0005: `PX4_GZ_WORLDS` is overwritten by gz_env.sh, so an env var alone cannot work; `GZ_SIM_RESOURCE_PATH` IS preserved).
- Plain `HEADLESS=1 make px4_sitl gz_x500` boots the default world (M0/M1 era).
- On the apriltag world, gz topics live under `/world/apriltag/...` — rediscover with `gz topic -l` if world/model names change.
- `PX4_SIM_SPEED_FACTOR` exists but CAUTION: under load RTF sags to ~0.3-0.5 anyway; anything timing-critical must schedule on SIM time, not wall time (ADR-0009 — this cost 5 dev runs + 2 failed gates). FPV-speed runs push PX4 params at runtime via MAVSDK pre-arm (ADR-0010 #3), not airframe files.
- **`GALLIUM_DRIVER=d3d12` is RECOMMENDED for NEW runs** (ADR-0046 addendum): GPU rendering via WSL's d3d12 makes the baseline RTF rock-solid (probe: loaded 0.9989 mean/0.9597 min vs 0.82/0.065 default llvmpipe) and check_m3 passed under it (0.027 m standoff). Export it before the make line. Existing gated paths keep their stock env; first d3d12 BATCH arm needs a paired-seed sanity vs stock before A/B use.
- Background launches MUST hold stdin open: `setsid bash -c "cd ~/PX4-Autopilot && tail -f /dev/null | env <vars> make px4_sitl gz_x500_mono_cam" > log 2>&1 &` (check_m3.sh:117 pattern). A /dev/null stdin makes the pxh console re-print its prompt in a tight loop — 21 GB of "pxh> " in an hour (2026-07-08).
- `gz topic -e` in scripts: ALWAYS bound with `-n <count>` — `timeout` alone wedges (gz is a Ruby wrapper; TERM hits the wrapper, the real echoer keeps the pipe open and a command substitution never sees EOF).

## Launch (headless) — markerless world (the deployed config since ADR-0058)
The apriltag world above is the AprilTag-fiducial baseline. The **adopted deployment
config flies the markerless (NN) seeker instead** — `worlds/markerless.sdf`, a tag-less
target body, world/target env set to match. The exports below mirror
`scripts/mc_deployment_arm.sh` (the canonical wrapper for an mc_batch deployment arm —
prefer it for an actual batch; use the standalone launch line below only for a manual/debug
boot outside mc_batch):

    export MC_WORLD="markerless"
    export MC_TARGET_MODEL="fpv_target_markerless"
    export MC_SEEKER="markerless"
    export MC_VENV_PYTHON=~/interceptor-sim/.venv-seeker/bin/python
    export MARKERLESS_NN_WEIGHTS=~/interceptor-sim/scripts/seeker/weights/drone_finetuned_v2.onnx
    PX4_GZ_WORLD=markerless GZ_SIM_RESOURCE_PATH=~/interceptor-sim/models HEADLESS=1 make px4_sitl gz_x500_mono_cam

- `worlds/markerless.sdf` needs the same symlink treatment as `apriltag.sdf` (into
  `~/PX4-Autopilot/Tools/simulation/gz/worlds/`) before `PX4_GZ_WORLD=markerless` resolves.
- `mc_batch.sh` does this translation itself: it reads `MC_WORLD` (default `apriltag`) and
  passes it straight through as `PX4_GZ_WORLD` on its internal launch line — so setting
  `MC_WORLD=markerless` (as `mc_deployment_arm.sh` does) is sufficient for a batch; the
  `PX4_GZ_WORLD=markerless` line above is only for launching PX4/Gazebo by hand.
  `MC_TARGET_MODEL`/`MC_SEEKER` are consumed by `mc_batch.sh` and `m4_intercept.py`, not by
  the `make` line itself.
- `MARKERLESS_NN_WEIGHTS` must point at the deployed weights (`drone_finetuned_v2.onnx`,
  v2 — v3 is a closed honest NULL, ADR-0061; do not point this at a v3 checkpoint).
  `MC_VENV_PYTHON` must be the seeker venv (`.venv-seeker`), which is CPU-only torch — see
  memory note on that venv.
- Topics live under `/world/markerless/...` on this world (mirror the apriltag-world
  rediscovery note below with `gz topic -l` if names change).

## Verify it is up
- Boot-complete line to grep for: `Startup script returned successfully`.
- CAUTION: "Ready for takeoff!" only prints AFTER a GCS/MAVSDK link connects —
  the gz_x500 airframes set `NAV_DLL_ACT=2`, so "No connection to the GCS" is a
  blocking preflight failure until then. Never gate a launch-wait on that string
  (chicken-and-egg deadlock; hit this in M0, 2026-07-04).
- MAVSDK connects on `udpin://0.0.0.0:14540`; once connected, arming becomes possible. Some gates need `sleep 5` after boot before connecting (see check_m3.sh).
- `gz topic -l` lists topics - confirm the camera topic exists before bridging frames.

## Concurrency rules (hard)
- ONE sim at a time. Two concurrent sims/batches kill each other (port clashes, RTF collapse). Batch arms run sequentially; batches only at idle machine load.
- Never `pkill` a batch from a shell whose args contain the batch script's name — the pattern matches the caller and self-kills (mc_batch lesson).
- gz-transport13 Python quirk: a process with ANY subscription never receives gz service responses. Anything calling gz services (e.g. set_pose) must be its own subscription-free process (the m4_target_mover pattern).

## Shut down cleanly
- `scripts/sim_kill.sh` (project helper), or type `shutdown` in the PX4 console, or kill the `px4` and `gz sim` processes. Stop old instances before a new run so ports do not clash. `scripts/sim_gui.sh` launches the GUI variant for demos.

## Notes
- Rendering warnings = the software (llvmpipe) renderer. Expected here; stay headless.
- Log each run (telemetry / detections / miss-distance) to `logs/` - see CLAUDE.md.
- S2 runs also need the cue mock: `scripts/s2_cue_mock.py` streams UDP JSON on 127.0.0.1:47800 (sim-time scheduled); `m4_intercept.py --handoff` requires `--fpv`.
