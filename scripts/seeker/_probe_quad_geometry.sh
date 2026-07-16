#!/usr/bin/env bash
# One-shot geometry probe for the set-pose recall sweep: boot quad_enemy, dump the
# interceptor + target poses and camera info, then tear down. Tells us the level
# camera's spawn pose/heading + target model default pose so setpose_recall_sweep.sh
# places the target at the correct boresight+range. Kill logic in-file (self-kill rule).
set -uo pipefail
cd "$(dirname "$0")/../.."
REPO_ROOT="$PWD"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WORLD=quad_enemy
TARGET_MODEL=fpv_quad_enemy
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="/tmp/claude-1000/probe_sim_${STAMP}.log"
READY_STR="Startup script returned successfully"; READY_TIMEOUT_S=150
source "$REPO_ROOT/scripts/sim_gpu_render.sh" 2>/dev/null || true
bash "$REPO_ROOT/scripts/sim_kill.sh" 2>/dev/null; sleep 2

setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" > "$SIM_LOG" 2>&1 &
ready=0; for ((i=0;i<READY_TIMEOUT_S;i++)); do grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null && { ready=1; break; }; sleep 1; done
[[ "$ready" -ne 1 ]] && { echo "FAIL sim-ready (see $SIM_LOG)"; bash "$REPO_ROOT/scripts/sim_kill.sh"; exit 1; }
sleep 6
echo "=== pose topic (one message) ==="
timeout 5 gz topic -e -n 1 -t /world/$WORLD/pose/info 2>/dev/null | \
  grep -A6 -E "name: \"(x500_mono_cam_0|camera_link|$TARGET_MODEL)\"" | \
  grep -E "name:|x:|y:|z:|w:" | head -60
echo "=== camera info intrinsics ==="
timeout 5 gz topic -e -n 1 -t /world/$WORLD/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info 2>/dev/null | grep -E "width:|height:" | head -4
bash "$REPO_ROOT/scripts/sim_kill.sh" 2>/dev/null
echo "=== DONE probe ==="
