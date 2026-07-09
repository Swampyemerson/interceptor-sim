#!/usr/bin/env bash
# ADR-0050 consolidated follow-up: boot the stereo_intercept world ONCE
# (GALLIUM_DRIVER=d3d12, ADR-0046 addendum -- stable RTF), run
# scripts/multirange_capture.py (mechanism (b): live set_pose rig teleports
# across standoffs, single boot), then tear down via scripts/sim_kill.sh
# (a FILE, never an inline pkill pattern -- sim_kill.sh's own header
# explains why: an inline `pkill -f "gz sim"` in this script's own argv
# would match itself).
#
# Usage: scripts/run_multirange_capture.sh [output_dir]
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_SEEKER_PYTHON="$REPO_ROOT/.venv-seeker/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/multirange_sim_${TIMESTAMP}.log"
OUT_DIR="${1:-$LOGS_DIR/rig_captures/multirange_${TIMESTAMP}}"
READY_TIMEOUT_S=180
READY_STRING="Startup script returned successfully"
POST_READY_SETTLE_S=5

WORLD_NAME="stereo_intercept"
TARGET_MODEL="fpv_target_markerless"
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/${WORLD_NAME}.sdf"
REPO_WORLD="$REPO_ROOT/worlds/${WORLD_NAME}.sdf"

mkdir -p "$LOGS_DIR" "$OUT_DIR"

echo "=== [multirange] STEP (i): stale-process cleanup (via sim_kill.sh) ==="
"$REPO_ROOT/scripts/sim_kill.sh"

echo "=== [multirange] STEP (ii): PX4 world symlink ==="
if [[ ! -d "$PX4_DIR" ]]; then
    echo "[multirange] FAIL: PX4-Autopilot not found at $PX4_DIR"; exit 1
fi
if [[ ! -x "$VENV_SEEKER_PYTHON" ]]; then
    echo "[multirange] FAIL: seeker venv python not found at $VENV_SEEKER_PYTHON"; exit 1
fi
if [[ ! -f "$REPO_WORLD" ]]; then
    echo "[multirange] FAIL: $REPO_WORLD not found"; exit 1
fi
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    echo "[multirange] Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi

echo "=== [multirange] STEP (iii): boot PX4 SITL + Gazebo headless (world=$WORLD_NAME, GALLIUM_DRIVER=d3d12) ==="
echo "[multirange] Sim output -> $SIM_LOG"
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD_NAME GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 GALLIUM_DRIVER=d3d12 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[multirange] Sim launched (pid/pgid $SIM_PID)."

ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[multirange] FAIL: sim process exited early. See $SIM_LOG"
        "$REPO_ROOT/scripts/sim_kill.sh"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
if [[ "$ready" -ne 1 ]]; then
    echo "[multirange] FAIL: did not see ready string within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    "$REPO_ROOT/scripts/sim_kill.sh"
    exit 1
fi
echo "[multirange] sim booted. Settling ${POST_READY_SETTLE_S}s..."
sleep "$POST_READY_SETTLE_S"

echo "=== [multirange] STEP (iv): run multirange_capture.py ==="
INTERCEPTOR_WORLD_NAME="$WORLD_NAME" INTERCEPTOR_TARGET_MODEL="$TARGET_MODEL" \
    "$VENV_SEEKER_PYTHON" "$REPO_ROOT/scripts/multirange_capture.py" --out "$OUT_DIR"
CAPTURE_EXIT=$?
echo "[multirange] capture exit code: $CAPTURE_EXIT"

echo "=== [multirange] STEP (v): teardown (via sim_kill.sh) ==="
"$REPO_ROOT/scripts/sim_kill.sh"

echo "[multirange] SIM_LOG=$SIM_LOG"
echo "[multirange] OUT_DIR=$OUT_DIR"
exit $CAPTURE_EXIT
