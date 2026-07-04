# NEXT — top of the stack

## Current: M0 — Toolchain
Environment confirmed: WSL2 Ubuntu 24.04.4, RTX 4070 available, 24 cores / 30 GB RAM.
Python 3.12.3, git 2.43. Nothing else installed yet.

Steps:
- [x] git init, scaffolding (.gitignore, PROGRESS.md, NEXT.md, docs/decisions.md)
- [ ] Clone PX4-Autopilot (latest stable tag) to `~/PX4-Autopilot`, run `Tools/setup/ubuntu.sh`
- [ ] Build px4_sitl; verify headless `make px4_sitl gz_x500` boots ("Ready for takeoff!")
- [ ] Python venv `.venv` with mavsdk, pytest, opencv-python, numpy
- [ ] `scripts/m0_takeoff.py` (MAVSDK: arm → takeoff 2 m → land) + `scripts/check_m0.sh` gate
- [ ] verifier runs gate → commit "M0: toolchain"

Key facts for a fresh session:
- PX4 lives OUTSIDE this repo at `~/PX4-Autopilot` (see ADR-0001)
- MAVSDK connects on `udpin://0.0.0.0:14540`; headless launch via `HEADLESS=1 make px4_sitl gz_x500`
- Launch/shutdown details: `.claude/skills/px4-gazebo/SKILL.md`
