#!/usr/bin/env bash
# M1 scripted milestone gate: headless PX4 SITL + Gazebo boots with the
# gz_x500_mono_cam airframe, and real camera frames are captured over
# gz-transport (no ROS — see docs/goals.md). See docs/goals.md milestone M1 and
# .claude/skills/px4-gazebo/SKILL.md for launch/shutdown conventions.
#
# Usage: scripts/check_m1.sh
# Exit code: 0 = PASS, non-zero = FAIL (propagates m1_capture.py's exit code,
# or a gate-specific failure code if the sim never comes up).
set -uo pipefail
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/check_m1_sim_${TIMESTAMP}.log"
READY_TIMEOUT_S=120
# NOTE: do NOT wait for "Ready for takeoff!" here. The gz_x500 airframes set
# NAV_DLL_ACT=2, which makes "No connection to the GCS" a blocking preflight
# failure — PX4 only prints "Ready for takeoff!" AFTER a GCS/MAVSDK link
# connects. We don't even need a GCS link for M1 (camera-only check), so
# this caution matters even more here: never gate on that string.
# "Startup script returned successfully" is PX4's unconditional boot-complete
# line (see .claude/skills/px4-gazebo/SKILL.md).
READY_STRING="Startup script returned successfully"

SIM_PID=""

mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_m1] Killing any stale px4 / gz sim processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 2
}

cleanup() {
    echo "[check_m1] Cleaning up sim processes..."
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        # SIM_PID was launched via `setsid`, so it is its own process group
        # leader: -$SIM_PID targets that whole group (make + px4 + gz sim)
        # without touching this script's own process group.
        kill -TERM -- "-$SIM_PID" 2>/dev/null
        sleep 2
        kill -KILL -- "-$SIM_PID" 2>/dev/null
    fi
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    # gz sim's renderer can take a few seconds to unwind after SIGTERM
    # (observed during M1 development); give it a moment before returning.
    sleep 3
}
trap cleanup EXIT

kill_stale_sim

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[check_m1] FAIL: PX4-Autopilot not found at $PX4_DIR"
    echo "FAIL"
    exit 1
fi

if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[check_m1] FAIL: venv python not found at $VENV_PYTHON (run the env setup first)"
    echo "FAIL"
    exit 1
fi

echo "[check_m1] Launching HEADLESS PX4 SITL + Gazebo (gz_x500_mono_cam) from $PX4_DIR..."
echo "[check_m1] Sim output -> $SIM_LOG"

# stdin must stay open: PX4's interactive pxh console spins on EOF and floods
# the log with prompt reprints. `tail -f /dev/null` never EOFs, so pxh blocks
# quietly; it dies with the group on cleanup. (Same pattern as check_m0.sh.)
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[check_m1] Sim launched (pid/pgid $SIM_PID)."

echo "[check_m1] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\" in sim log..."
ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[check_m1] FAIL: sim process exited early (pid $SIM_PID). See $SIM_LOG"
        echo "FAIL"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done

if [[ "$ready" -ne 1 ]]; then
    echo "[check_m1] FAIL: did not see \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    echo "FAIL"
    exit 1
fi

echo "[check_m1] Sim ready. Running m1_capture.py..."
"$VENV_PYTHON" "$REPO_ROOT/scripts/m1_capture.py" --frames 10
CAPTURE_EXIT=$?

if [[ "$CAPTURE_EXIT" -eq 0 ]]; then
    echo "[check_m1] PASS"
else
    echo "[check_m1] FAIL (m1_capture.py exit code $CAPTURE_EXIT). See $SIM_LOG"
fi

exit "$CAPTURE_EXIT"
