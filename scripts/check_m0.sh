#!/usr/bin/env bash
# M0 scripted milestone gate: headless PX4 SITL + Gazebo boots, MAVSDK arms,
# takes off to 2 m, lands. See GOALS.md milestone M0 and
# .claude/skills/px4-gazebo/SKILL.md for launch/shutdown conventions.
#
# Usage: scripts/check_m0.sh
# Exit code: 0 = PASS, non-zero = FAIL (propagates m0_takeoff.py's exit code,
# or a gate-specific failure code if the sim never comes up).
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/check_m0_sim_${TIMESTAMP}.log"
READY_TIMEOUT_S=120
# NOTE: do NOT wait for "Ready for takeoff!" here. The gz_x500 airframe sets
# NAV_DLL_ACT=2, which makes "No connection to the GCS" a blocking preflight
# failure — PX4 only prints "Ready for takeoff!" AFTER a GCS/MAVSDK link
# connects, and our MAVSDK script is what creates that link. Waiting for it
# before launching the script deadlocks (verified 2026-07-04).
# "Startup script returned successfully" is PX4's unconditional boot-complete line.
READY_STRING="Startup script returned successfully"

SIM_PID=""

mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_m0] Killing any stale px4 / gz sim processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 2
}

cleanup() {
    echo "[check_m0] Cleaning up sim processes..."
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
}
trap cleanup EXIT

kill_stale_sim

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[check_m0] FAIL: PX4-Autopilot not found at $PX4_DIR"
    echo "FAIL"
    exit 1
fi

if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[check_m0] FAIL: venv python not found at $VENV_PYTHON (run the env setup first)"
    echo "FAIL"
    exit 1
fi

echo "[check_m0] Launching HEADLESS PX4 SITL + Gazebo (gz_x500) from $PX4_DIR..."
echo "[check_m0] Sim output -> $SIM_LOG"

# stdin must stay open: PX4's interactive pxh console spins on EOF and floods
# the log with prompt reprints (observed: 7 GB in 20 min). `tail -f /dev/null`
# never EOFs, so pxh blocks quietly; it dies with the group on cleanup.
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env HEADLESS=1 make px4_sitl gz_x500" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[check_m0] Sim launched (pid/pgid $SIM_PID)."

echo "[check_m0] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\" in sim log..."
ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[check_m0] FAIL: sim process exited early (pid $SIM_PID). See $SIM_LOG"
        echo "FAIL"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done

if [[ "$ready" -ne 1 ]]; then
    echo "[check_m0] FAIL: did not see \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    echo "FAIL"
    exit 1
fi

echo "[check_m0] Sim ready. Running m0_takeoff.py..."
"$VENV_PYTHON" "$REPO_ROOT/scripts/m0_takeoff.py"
TAKEOFF_EXIT=$?

if [[ "$TAKEOFF_EXIT" -eq 0 ]]; then
    echo "[check_m0] PASS"
else
    echo "[check_m0] FAIL (m0_takeoff.py exit code $TAKEOFF_EXIT). See $SIM_LOG and the CSV in $LOGS_DIR"
fi

exit "$TAKEOFF_EXIT"
