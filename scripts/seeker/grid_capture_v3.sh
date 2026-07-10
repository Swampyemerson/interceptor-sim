#!/usr/bin/env bash
# Seeker v3 Phase-1 static grid capture (docs/seeker_v3_dataset_plan.md §3
# Phase 1). ONE markerless sim boot; the drone sits DISARMED on the ground
# (camera still publishes at 30 Hz, /clock advances) while seeker_v3_capture.py
# teleports the tag-less target through the 648-pose grid and auto-labels via
# gt-projection. NEW file (task #28); boot pattern from capture_pass.sh /
# probe_markerless_range.py. No flight, no mover.
#
# Usage:  scripts/seeker/grid_capture_v3.sh [OUT_DIR]
set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PY="$REPO_ROOT/.venv-seeker/bin/python"
WORLD=markerless
TARGET_MODEL=fpv_target_markerless
OUT_DIR="${1:-$REPO_ROOT/scripts/seeker/data/v3_grid}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="$REPO_ROOT/logs/seeker_capture_v3"
mkdir -p "$LOG_DIR"
SIM_LOG="$LOG_DIR/grid_sim_${STAMP}.log"
RUN_LOG="$LOG_DIR/grid_run_${STAMP}.log"

READY_STR="Startup script returned successfully"
READY_TIMEOUT_S=150
SETTLE_S=6

teardown_sim() {
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 3
}

echo "[grid_v3] out=$OUT_DIR sim_log=$SIM_LOG run_log=$RUN_LOG"
teardown_sim

setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &

ready=0
for ((i=0; i<READY_TIMEOUT_S; i++)); do
    if grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null; then ready=1; break; fi
    sleep 1
done
if [[ "$ready" -ne 1 ]]; then
    echo "[grid_v3] FAIL: sim not ready in ${READY_TIMEOUT_S}s (see $SIM_LOG)"
    teardown_sim; echo "GRID_V3_COMPLETE rc=1"; exit 1
fi
sleep "$SETTLE_S"

# seeker_v3_capture.py runs its own posecheck [A8]/[A4], teleports the grid,
# auto-labels via gt-projection, writes index.csv + capture_meta.json.
INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
    "$VENV_PY" "$REPO_ROOT/scripts/seeker_v3_capture.py" --out "$OUT_DIR" \
    > "$RUN_LOG" 2>&1
RC=$?
echo "[grid_v3] capture exit=$RC"
tail -n 6 "$RUN_LOG" || true

teardown_sim
echo "GRID_V3_COMPLETE rc=$RC"
exit "$RC"
