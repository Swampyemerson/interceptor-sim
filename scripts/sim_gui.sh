#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo WITH THE GUI on the apriltag world, for demos and
# watch-it-fly sessions (WSLg display + RTX 4070 rendering). Automated/batch
# runs stay headless via the check_m*.sh scripts — this is the interactive
# counterpart.
#
# Why this is a file and not a one-liner: `pkill -f` matches full command
# lines, so any interactive one-liner that both kills stale sims AND
# contains the launch text (`make px4_sitl ...`) kills its own shell —
# see scripts/sim_kill.sh's header. Inside a script, the process cmdline is
# just the script path, so both halves are safe together.
#
# Usage: scripts/sim_gui.sh [tag_x tag_y tag_z]
#   With args: after boot, pre-places the tag at (tag_x, tag_y, tag_z).
#   Without:   tag stays at the world-file default (5, 0, 0.5).
# Prints READY and exits 0 once PX4's boot-complete line appears (sim keeps
# running in the background, detached); exits 1 on boot timeout.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/gui_sim_${TIMESTAMP}.log"
READY_TIMEOUT_S=120
READY_STRING="Startup script returned successfully"  # ADR-0004

mkdir -p "$LOGS_DIR"

"$REPO_ROOT/scripts/sim_kill.sh"

# Same symlink repair as the check scripts (ADR-0005).
ln -sf "$REPO_ROOT/worlds/apriltag.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/apriltag.sdf"

echo "[sim_gui] Launching PX4 SITL + Gazebo WITH GUI (world=apriltag) -> $SIM_LOG"
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[sim_gui] Launched (pgid $SIM_PID)."

elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        echo "[sim_gui] READY (after ~${elapsed}s)."
        sleep 5  # EKF settle before any MAVSDK connect (same as check_m3/m4)
        # Camera-liveness check: a wedged renderer (seen 2026-07-05 after
        # several rapid GUI reboots -- PX4 boots fine but the camera sensor
        # never publishes, GPU idle) would otherwise eat a whole flight.
        CAM_TOPIC="/world/apriltag/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info"
        if timeout 10 gz topic -e -t "$CAM_TOPIC" -n 1 > /dev/null 2>&1; then
            echo "[sim_gui] Camera topic alive."
        else
            echo "[sim_gui] FAIL: camera topic $CAM_TOPIC silent for 10s (renderer wedged?). Kill and relaunch."
            exit 1
        fi
        if [[ $# -ge 3 ]]; then
            echo "[sim_gui] Pre-placing tag at ($1, $2, $3)..."
            gz service -s /world/apriltag/set_pose --reqtype gz.msgs.Pose \
                --reptype gz.msgs.Boolean --timeout 2000 \
                --req "name: \"apriltag_target\" position { x: $1 y: $2 z: $3 }"
        fi
        exit 0
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done

echo "[sim_gui] FAIL: no \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
exit 1
