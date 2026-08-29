#!/usr/bin/env bash
# WIND GATE-0 (W3) PHYSICS PROBE -- the scripted gate.
#
# Boots headless PX4 SITL + Gazebo (gz_x500_mono_cam, world=apriltag), runs
# scripts/wind_gate0_probe.py, tears everything down. Exit 0 = PASS.
#
# WHAT IT PROVES: that Gazebo actually applies the persistent EntityWrench that
# scripts/wind_driver.py publishes. Nothing else in the repo tests that link --
# the driver's offline self-test proves the force vector and the publish path,
# and its CSV logs what it PUT ON THE WIRE, so an inert wrench produces a
# perfect-looking log over a windless flight (ADR-0096).
#
# Pre-registration (config, prediction, criterion, meaning of a null, all fixed
# BEFORE this first ran): docs/wind_gate0_prereg.md
#
# Usage: scripts/check_wind_gate0.sh [extra args passed to wind_gate0_probe.py]
# Exit:  0 = PASS; 1 = NULL / PARTIAL / FAIL / VOID; 2 = the sim never came up.
set -uo pipefail
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="$LOGS_DIR/wind_gate0_${TIMESTAMP}"
SIM_LOG="$OUT_DIR/sim.log"
READY_TIMEOUT_S=180
# PX4's unconditional boot-complete line. Do NOT wait for "Ready for takeoff!"
# -- that only prints after a GCS link exists, and our script is what creates
# it (ADR-0004).
READY_STRING="Startup script returned successfully"

# The world file must resolve inside PX4's own worlds dir (ADR-0005).
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/apriltag.sdf"
REPO_WORLD="$REPO_ROOT/worlds/apriltag.sdf"

SIM_PID=""
mkdir -p "$OUT_DIR"

# NOTE: these pkill patterns live in a FILE on purpose. `pkill -f` matches full
# command lines, so a pattern typed inline in a tool-call command sits in an
# ancestor's argv and self-matches, killing the invocation (exit 144). A check
# script's own cmdline is just its path, so it is immune. See scripts/sim_kill.sh.
kill_stale() {
    echo "[gate0] Killing stale px4 / gz / wind-driver processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    # A stranded wind_driver.py holds a PERSISTENT wrench on the airframe that
    # survives into the NEXT boot's entity of the same name. Same reason
    # mc_batch.sh kills it before every arm.
    pkill -f "wind_driver.py" 2>/dev/null
    sleep 2
}

cleanup() {
    echo "[gate0] Tearing down..."
    pkill -f "wind_driver.py" 2>/dev/null
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
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

kill_stale

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[gate0] FAIL: PX4-Autopilot not found at $PX4_DIR"; exit 2
fi
if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[gate0] FAIL: venv python not found at $VENV_PYTHON"; exit 2
fi
if [[ ! -f "$REPO_WORLD" ]]; then
    echo "[gate0] FAIL: $REPO_WORLD not found"; exit 2
fi
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    echo "[gate0] Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi

echo "[gate0] Output dir -> $OUT_DIR"
echo "[gate0] Launching HEADLESS PX4 SITL + Gazebo (gz_x500_mono_cam, world=apriltag)..."

# stdin must stay open: PX4's interactive pxh console spins on EOF and floods
# the log (observed: 7 GB in 20 min). `tail -f /dev/null` never EOFs.
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[gate0] Sim launched (pid/pgid $SIM_PID). Log -> $SIM_LOG"

ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then ready=1; break; fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[gate0] FAIL: sim exited early. See $SIM_LOG"; exit 2
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
if [[ "$ready" -ne 1 ]]; then
    echo "[gate0] FAIL: no \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    exit 2
fi

# GATE0_PROBE lets the same boot/teardown drive a different measurement script
# against an identical sim -- used by the wrench diagnostic, which has to run in
# exactly this environment for its answer to mean anything. One harness, not two
# that can drift apart.
PROBE_SCRIPT="${GATE0_PROBE:-$REPO_ROOT/scripts/wind_gate0_probe.py}"
echo "[gate0] Sim ready. Running $(basename "$PROBE_SCRIPT")..."
"$VENV_PYTHON" "$PROBE_SCRIPT" --out-dir "$OUT_DIR" "$@" \
    2>&1 | tee "$OUT_DIR/probe.log"
PROBE_EXIT=${PIPESTATUS[0]}

echo "[gate0] probe exit=$PROBE_EXIT; artefacts in $OUT_DIR"
exit "$PROBE_EXIT"
