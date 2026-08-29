#!/usr/bin/env bash
# M2 scripted milestone gate: headless PX4 SITL + Gazebo boots with the
# gz_x500_mono_cam airframe on the custom "apriltag" world (a static
# AprilTag 36h11 target board), and the tag is detected live with pose
# accuracy checked against sim ground truth. See docs/goals.md milestone M2,
# worlds/apriltag.sdf, models/apriltag_target/, and
# .claude/skills/px4-gazebo/SKILL.md for launch/shutdown conventions.
#
# Usage: scripts/check_m2.sh
# Exit code: 0 = PASS, non-zero = FAIL (propagates m2_detect.py's exit code,
# or a gate-specific failure code if the sim never comes up).
set -uo pipefail
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/check_m2_sim_${TIMESTAMP}.log"
READY_TIMEOUT_S=120
# NOTE: do NOT wait for "Ready for takeoff!" here -- see ADR-0004 /
# .claude/skills/px4-gazebo/SKILL.md. M2 doesn't even need a GCS/MAVSDK
# link (pure perception check), so this caution applies just as much as
# in M0/M1. "Startup script returned successfully" is PX4's unconditional
# boot-complete line.
READY_STRING="Startup script returned successfully"

# worlds/apriltag.sdf is the checked-in source of truth (kept in this repo
# per the M2 task spec), but PX4's own px4-rc.gzsim resolves world files as
# ${PX4_GZ_WORLDS}/<name>.sdf, and gz_env.sh (sourced at runtime from the
# SITL working dir) unconditionally overwrites PX4_GZ_WORLDS to PX4's own
# Tools/simulation/gz/worlds directory -- that env var is NOT additive, so
# exporting it ourselves would just get clobbered. A symlink is the fix:
# PX4_GZ_WORLDS still points at PX4's own dir, but this specific world name
# resolves there too. GZ_SIM_RESOURCE_PATH (for our model) IS additive
# (gz_env.sh appends to whatever's already set), so that one just works via
# the env var below. See docs/decisions.md and worlds/apriltag.sdf.
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/apriltag.sdf"
REPO_WORLD="$REPO_ROOT/worlds/apriltag.sdf"

SIM_PID=""

mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_m2] Killing any stale px4 / gz sim processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 2
}

cleanup() {
    echo "[check_m2] Cleaning up sim processes..."
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
    echo "[check_m2] FAIL: PX4-Autopilot not found at $PX4_DIR"
    echo "FAIL"
    exit 1
fi

if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[check_m2] FAIL: venv python not found at $VENV_PYTHON (run the env setup first)"
    echo "FAIL"
    exit 1
fi

if [[ ! -f "$REPO_WORLD" ]]; then
    echo "[check_m2] FAIL: $REPO_WORLD not found"
    echo "FAIL"
    exit 1
fi

# Make sure the symlink into PX4's worlds dir exists and points at our repo
# copy (create/repair it rather than assuming a prior manual step held).
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    echo "[check_m2] Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi

echo "[check_m2] Launching HEADLESS PX4 SITL + Gazebo (gz_x500_mono_cam, world=apriltag) from $PX4_DIR..."
echo "[check_m2] Sim output -> $SIM_LOG"

# stdin must stay open: PX4's interactive pxh console spins on EOF and floods
# the log with prompt reprints. `tail -f /dev/null` never EOFs, so pxh blocks
# quietly; it dies with the group on cleanup. (Same pattern as check_m0/m1.sh.)
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[check_m2] Sim launched (pid/pgid $SIM_PID)."

echo "[check_m2] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\" in sim log..."
ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[check_m2] FAIL: sim process exited early (pid $SIM_PID). See $SIM_LOG"
        echo "FAIL"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done

if [[ "$ready" -ne 1 ]]; then
    echo "[check_m2] FAIL: did not see \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    echo "FAIL"
    exit 1
fi

echo "[check_m2] Sim ready. Running m2_detect.py..."
"$VENV_PYTHON" "$REPO_ROOT/scripts/m2_detect.py" --duration 15
DETECT_EXIT=$?

if [[ "$DETECT_EXIT" -eq 0 ]]; then
    echo "[check_m2] PASS"
else
    echo "[check_m2] FAIL (m2_detect.py exit code $DETECT_EXIT). See $SIM_LOG and the CSV in $LOGS_DIR"
fi

exit "$DETECT_EXIT"
