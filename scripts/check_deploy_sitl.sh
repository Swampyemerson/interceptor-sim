#!/usr/bin/env bash
# check_deploy_sitl.sh -- FUNCTIONAL check of the deployment seeker loop's LIVE
# MAVSDK OFFBOARD path (flight/deploy/seeker_loop.py, README item 4) against a
# local PX4 SITL. This is the same code the real interceptor's Pi 5 runs; the
# front-loaded Pixhawk 6C Mini will re-run the equivalent on the props-off bench,
# so this burns down the software risk first (NEXT.md "REMAINING SIM PREP",
# terminal-stage changelog 2026-07-18).
#
# It boots headless PX4 SITL + Gazebo (gz_x500, default world -- the smoke uses a
# SYNTHETIC detector, so no camera/apriltag world is needed), then drives
# `seeker_loop.py --sitl-smoke`: connect -> health -> arm -> takeoff -> enter
# OFFBOARD -> stream pro-nav NED velocity setpoints for ~45 s -> land + disarm.
# The point is the MAVLink PLUMBING (connection, mode switch, setpoint cadence,
# clean shutdown, no exceptions), NOT perception.
#
# This is a FUNCTIONAL check, not a measured gate, so the "idle-load only" rule
# for Monte-Carlo batches does not block it (no timing is being scored here).
#
# Boot / shutdown conventions mirror scripts/check_m0.sh and .claude/skills/
# px4-gazebo/SKILL.md (setsid process group, `tail -f /dev/null` stdin so the pxh
# console does not flood the log, "Startup script returned successfully" as the
# boot-complete line). pkill patterns live in THIS file (its cmdline is just the
# script path, so `pkill -f` cannot self-match -- see scripts/sim_kill.sh).
#
# Usage:  scripts/check_deploy_sitl.sh
# Exit:   0 = PASS (offboard entered + setpoints streamed + clean shutdown),
#         non-zero = FAIL (propagates seeker_loop.py's exit code, or a boot fail).
set -uo pipefail
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"   # mavsdk lives here; onnx NOT needed
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/check_deploy_sitl_sim_${TIMESTAMP}.log"
RUN_LOG="$LOGS_DIR/deploy_seeker_sitl_${TIMESTAMP}.log"   # the evidence log
READY_STRING="Startup script returned successfully"
READY_TIMEOUT_S=120
POST_READY_SETTLE_S=5      # let the EKF settle before MAVSDK connects (check_m3)
SMOKE_DURATION_S=45        # >=30 s sim time even if RTF sags (seeker_loop default)
# Outer wall bound: connect + health + takeoff + ~45 s stream + land, w/ margin.
PY_TIMEOUT_S=260

SIM_PID=""
mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_deploy_sitl] Killing any stale px4 / gz sim processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 2
}

cleanup() {
    echo "[check_deploy_sitl] Cleaning up sim processes..."
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        # setsid => SIM_PID is its own process-group leader; -$SIM_PID targets
        # the whole group (make + px4 + gz sim) without touching this script.
        kill -TERM -- "-$SIM_PID" 2>/dev/null
        sleep 2
        kill -KILL -- "-$SIM_PID" 2>/dev/null
    fi
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 3
}
trap cleanup EXIT

kill_stale_sim

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[check_deploy_sitl] FAIL: PX4-Autopilot not found at $PX4_DIR"
    exit 1
fi
if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[check_deploy_sitl] FAIL: venv python not found at $VENV_PYTHON"
    exit 1
fi

echo "[check_deploy_sitl] Launching HEADLESS PX4 SITL + Gazebo (gz_x500) from $PX4_DIR..."
echo "[check_deploy_sitl] Sim output -> $SIM_LOG"
echo "[check_deploy_sitl] Run  output -> $RUN_LOG"

# stdin must stay open (tail -f /dev/null) or the pxh console spins on EOF and
# floods the log with prompt reprints (px4-gazebo SKILL.md).
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env HEADLESS=1 make px4_sitl gz_x500" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[check_deploy_sitl] Sim launched (pid/pgid $SIM_PID)."

echo "[check_deploy_sitl] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\"..."
ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[check_deploy_sitl] FAIL: sim exited early (pid $SIM_PID). See $SIM_LOG"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
if [[ "$ready" -ne 1 ]]; then
    echo "[check_deploy_sitl] FAIL: no \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    exit 1
fi

echo "[check_deploy_sitl] Sim ready. Settling ${POST_READY_SETTLE_S}s before MAVSDK connects..."
sleep "$POST_READY_SETTLE_S"

echo "[check_deploy_sitl] Driving seeker_loop.py --sitl-smoke (outer timeout ${PY_TIMEOUT_S}s)..."
{
    echo "=== deploy_seeker SITL smoke @ ${TIMESTAMP} (UTC) ==="
    echo "host: $(uname -sr)  |  python: $VENV_PYTHON"
    echo "note: FUNCTIONAL check (not a measured gate); cadence is WALL time."
    echo "      GPU render (ADR-0075) holds RTF ~0.95, so ${SMOKE_DURATION_S}s wall >= 30s sim time."
    echo "sim boot log: $SIM_LOG"
    echo "----------------------------------------------------------------"
} | tee "$RUN_LOG"

timeout "$PY_TIMEOUT_S" "$VENV_PYTHON" -m flight.deploy.seeker_loop \
    --sitl-smoke --mavsdk-url udpin://0.0.0.0:14540 \
    --smoke-duration "$SMOKE_DURATION_S" 2>&1 | tee -a "$RUN_LOG"
PY_EXIT="${PIPESTATUS[0]}"

echo "----------------------------------------------------------------" | tee -a "$RUN_LOG"
if [[ "$PY_EXIT" -eq 124 ]]; then
    echo "[check_deploy_sitl] FAIL: seeker_loop hit the ${PY_TIMEOUT_S}s outer timeout." | tee -a "$RUN_LOG"
    exit 1
fi
if [[ "$PY_EXIT" -eq 0 ]]; then
    echo "[check_deploy_sitl] PASS (see $RUN_LOG)" | tee -a "$RUN_LOG"
else
    echo "[check_deploy_sitl] FAIL (seeker_loop exit $PY_EXIT; see $RUN_LOG and $SIM_LOG)" | tee -a "$RUN_LOG"
fi
exit "$PY_EXIT"
