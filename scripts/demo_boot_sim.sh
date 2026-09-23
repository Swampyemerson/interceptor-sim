#!/usr/bin/env bash
# Boot the headless demo sim (apriltag world) detached, for demo captures.
# Invoke by path so the launch text never sits in a tool-call argv
# (the inline-pattern self-kill rule, CLAUDE.md batch hygiene).
# Usage: bash scripts/demo_boot_sim.sh <simlog-path>
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIMLOG="${1:?usage: demo_boot_sim.sh <simlog>}"
source "$REPO_ROOT/scripts/sim_gpu_render.sh" 2>/dev/null || true
ln -sf "$REPO_ROOT/worlds/apriltag.sdf" "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds/apriltag.sdf"
setsid bash -c "cd \"$HOME/PX4-Autopilot\" && tail -f /dev/null | env PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH=\"$REPO_ROOT/models\" HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
    > "$SIMLOG" 2>&1 &
echo "$!" > "$SIMLOG.pgid"
echo "[demo_boot_sim] launched pgid $(cat "$SIMLOG.pgid") -> $SIMLOG"
