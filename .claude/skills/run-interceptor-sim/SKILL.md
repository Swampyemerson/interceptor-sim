---
name: run-interceptor-sim
description: Build, run, and drive the interceptor sim (PX4 SITL + Gazebo Harmonic, camera-only pro-nav). Use when asked to run/launch/boot/smoke-test the simulator, verify the whole stack works on this machine, capture a camera frame / "screenshot" the sim, run the tests, or fly a milestone gate (M0–M5).
---
# Run the interceptor sim

A **simulation-only** counter-UAS interceptor: PX4 SITL + **Gazebo Harmonic** (the `gz_x500_mono_cam` quad + forward mono camera), flown headless over MAVSDK-Python, guided by camera-only proportional navigation. It has **no GUI you drive** — you drive it by **booting the sim and reading its topics** (camera frames over gz-transport, telemetry/miss-distance to CSV). The one handle a fresh agent should reach for first is the smoke driver:

> **`bash .claude/skills/run-interceptor-sim/smoke.sh`** — chains the portable tests + a real sim boot + a live camera capture into one PASS/FAIL, and prints a directory of `.png` frames you can open (the closest thing to a screenshot of the running app).

All paths below are relative to the repo root (`~/interceptor-sim`). Python runs through the project venvs directly (`.venv/bin/python`), no `activate` needed.

## Prerequisites (verified present this session — not installed from scratch here)
This machine (WSL2 Ubuntu, RTX 4070) was already provisioned. The stack the driver needs, and how each is confirmed:
- **PX4-Autopilot v1.17.0** at `~/PX4-Autopilot` (`make px4_sitl gz_x500_mono_cam` builds/boots it). — `cat ~/PX4-Autopilot/version.txt` → `v1.17.0`
- **Gazebo Harmonic** — `gz sim --version` → `8.14.0`.
- **`.venv`** (main: cv2, MAVSDK, gz bindings) and **`.venv-seeker`** (adds onnxruntime for the NN detector). Both must exist and be executable.
- Python deps are pinned in `requirements.txt`; project config files (`CLAUDE.md`, `docs/goals.md`, `docs/`) can be regenerated with `./bootstrap.sh` on a fresh VM (a normal clone doesn't need it).

If a gate later can't find the world file, run `scripts/check_m2.sh` once — it creates/repairs the `worlds/apriltag.sdf` symlink PX4 needs (PX4's `gz_env.sh` overwrites `PX4_GZ_WORLDS` every launch, ADR-0005, so an env var alone can't work).

## Run (agent path) — the smoke driver
```bash
# full: portable tests (~20 s, no sim)  +  boot PX4+Gazebo, capture 10 camera frames, teardown (~2–3 min)
bash .claude/skills/run-interceptor-sim/smoke.sh

# fast: portable core only, no sim boot (~20 s)
bash .claude/skills/run-interceptor-sim/smoke.sh --tests-only
```
A green run ends with `smoke: PASS` and a `logs/m1_frames_*/` path. **Open `frame_000.png` and look at it** — a real run shows a rendered horizon (sky over ground), 1280×960, pixel stddev > 0. A blank/uniform frame (stddev 0) means the camera bridge is up but the render is dead → sim-debug skill.

What the driver chains (both are committed, both verified this session):
- `scripts/run_tests.sh` → `176 passed, 2 skipped` (main, `.venv`) + `28 passed` (ONNX parity, `.venv-seeker`) = `ALL GREEN`.
- `scripts/check_m1.sh` → boots headless, `m1_capture.py` saves 10 frames, `[check_m1] PASS`, tears the sim down via an EXIT trap.

## Run (deeper drivers) — drive an actual flight
Each boots → flies → scores → tears down, exit 0 = pass (run one at a time, at idle load):
```bash
scripts/check_m0.sh    # arm → takeoff to 2 m → land (the "does it fly at all" gate)
scripts/check_m3.sh    # static intercept: hold a 2 m standoff, < 0.5 m final error
scripts/check_m4.sh    # moving-target intercept: pursuit vs pro-nav, pro-nav < 1.0 m
scripts/check_t21.sh   # re-asserts the detect-then-track headline from committed CSVs — NO sim
```
Monte-Carlo batches (many boots): use the **mc-batch** skill, never hand-rolled — it bakes in the self-kill/GPU-wedge guards. Env vars, world variants (apriltag vs markerless), and launch footguns: the **px4-gazebo** skill. A wedged stack (won't boot / no frames / MAVSDK won't connect): the **sim-debug** skill.

## Direct invocation (no full sim)
The portable guidance/geometry core is a dependency-light package you can exercise without booting anything:
```bash
.venv/bin/python -m pytest flight/tests/ tests/ -q      # what run_tests.sh stage 1 runs
```

## Human path
There's no window to open. `scripts/sim_gui.sh` launches the Gazebo GUI for a demo (needs WSLg); useless for headless/automated work — stay headless.

## Gotchas (each cost real debugging time — traced to ADRs/incidents)
- **Never `pkill -f`/`pgrep -f` a sim pattern (`px4`, `gz sim`, `sitl`, `mc_batch.sh`) typed inline in a tool call.** The harness evals the command text, so the pattern sits in an ancestor's argv, self-matches, and kills your own invocation (exit 144) even with zero real matches. Kill logic lives in script **files** invoked by path (`scripts/sim_kill.sh`); these driver scripts already do this.
- **ONE sim at a time, at idle machine load.** Two concurrent sims/batches kill each other (port clashes, RTF collapse). Batches confound under load (ADR-0015).
- **Sim time, never wall time** for anything measured/scheduled. RTF sags to ~0.3–0.5 under load; wall-clock timing silently distorts (cost 5 dev runs + 2 gates, ADR-0009).
- **Never wait on `"Ready for takeoff!"`** to detect boot — the gz_x500 airframes set `NAV_DLL_ACT=2`, so that line only prints *after* MAVSDK connects (chicken-and-egg deadlock). Grep the sim log for `Startup script returned successfully` instead (the drivers do).
- **Background sim launches must hold stdin open** (`tail -f /dev/null | env … make …`): a `/dev/null` stdin makes the pxh console reprint its prompt in a tight loop (21 GB of `pxh> ` in an hour, 2026-07-08).
- **GPU render** is on by default via `scripts/sim_gpu_render.sh` (sourced by the boot scripts → `GALLIUM_DRIVER=d3d12`, RTF ~0.95 vs ~0.34 software). Toggle off with `SIM_GPU_RENDER=0` if d3d12 misbehaves.
- **`.venv-seeker` torch is CPU-only** (the 4070 is used for *render*, not training) — fine for the small ONNX seeker, slow for real training.

## Troubleshooting
- **`FAIL: venv python not found`** → `.venv`/`.venv-seeker` missing; re-provision (`requirements.txt`).
- **`FAIL: PX4-Autopilot not found`** → set `PX4_DIR=/path/to/PX4-Autopilot` or install v1.17.0 at `~/PX4-Autopilot`.
- **`did not see "Startup script returned successfully"`** → sim didn't boot in time; read `logs/check_m1_sim_*.log`, then the **sim-debug** skill.
- **Frames captured but blank (stddev ~0)** → camera topic alive, render dead; check GPU render (`SIM_GPU_RENDER=0` to fall back to software), then sim-debug.
- **`exit 144` from a shell command** → you self-killed via an inline `pkill`/`pgrep` sim pattern (see Gotchas); move it into a script file.
