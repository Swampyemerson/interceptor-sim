#!/usr/bin/env bash
# Boot the headless demo sim on an arbitrary world, detached (world parameter added 2026-09-23; default apriltag keeps old callers byte-compatible).
# Usage: bash scripts/demo_boot_sim.sh <simlog-path> [world]
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIMLOG="${1:?usage: demo_boot_sim.sh <simlog> [world]}"
WORLD="${2:-apriltag}"
source "$REPO_ROOT/scripts/sim_gpu_render.sh" 2>/dev/null || true
ln -sf "$REPO_ROOT/worlds/${WORLD}.sdf" "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds/${WORLD}.sdf"
setsid bash -c "cd \"$HOME/PX4-Autopilot\" && tail -f /dev/null | env PX4_GZ_WORLD=${WORLD} GZ_SIM_RESOURCE_PATH=\"$REPO_ROOT/models\" HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
    > "$SIMLOG" 2>&1 &
echo "$!" > "$SIMLOG.pgid"
echo "[demo_boot_sim] world=$WORLD pgid $(cat "$SIMLOG.pgid") -> $SIMLOG"
