#!/usr/bin/env bash
# M4 scripted milestone gate: for EACH guidance law (pursuit, then pronav),
# boots a completely fresh headless PX4 SITL + Gazebo instance (gz_x500_mono_cam
# on the custom "apriltag" world), pre-places the AprilTag at the engagement
# start position, then runs scripts/m4_intercept.py to intercept a MOVING
# tag (scripts/m4_target_mover.py streams it along a straight line once
# scripts/m4_intercept.py's ENGAGE phase spawns it). See GOALS.md milestone
# M4, scripts/m4_intercept.py, scripts/m4_target_mover.py,
# worlds/apriltag.sdf, models/apriltag_target/, and
# .claude/skills/px4-gazebo/SKILL.md for launch/shutdown conventions.
#
# Two full boots (one per law) rather than one boot reused for both runs:
# each law gets a clean vehicle/EKF/tag state, so neither run can be
# influenced by whatever the other run left behind.
#
# Usage: scripts/check_m4.sh
# Exit code: 0 = PASS (both flights clean AND pro-nav miss < 1.0 m),
# non-zero = FAIL.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
READY_TIMEOUT_S=120
# NOTE: do NOT wait for "Ready for takeoff!" here -- see ADR-0004 /
# .claude/skills/px4-gazebo/SKILL.md. "Startup script returned successfully"
# is PX4's unconditional boot-complete line.
READY_STRING="Startup script returned successfully"
# Same reasoning as check_m3.sh: m4_intercept.py arms and enters OFFBOARD
# shortly after health-OK, so give the EKF a few seconds after boot-complete
# to settle before MAVSDK connects and starts issuing commands.
POST_READY_SETTLE_S=5
# Guards against a hung MAVSDK connect/guidance loop wedging the gate
# forever; m4_intercept.py has its own internal timeouts (connect, health,
# altitude, acquire, engage, land) that should all fire well before this.
PY_TIMEOUT_S=300
# GOALS.md M4 gate: pro-nav closest approach must be < this.
PRONAV_MISS_GATE_M=1.0
# The engagement start used both to pre-place the tag here AND as
# m4_intercept.py's (and m4_target_mover.py's) own --target-start default --
# see scripts/m4_intercept.py's module docstring / CLI --help.
TARGET_START_X=6.5
TARGET_START_Y=-4
TARGET_START_Z=0.5

# worlds/apriltag.sdf is the checked-in source of truth (kept in this repo
# per the M2 task spec), but PX4's own px4-rc.gzsim resolves world files as
# ${PX4_GZ_WORLDS}/<name>.sdf, and gz_env.sh (sourced at runtime from the
# SITL working dir) unconditionally overwrites PX4_GZ_WORLDS to PX4's own
# Tools/simulation/gz/worlds directory -- that env var is NOT additive, so
# exporting it ourselves would just get clobbered. A symlink is the fix:
# PX4_GZ_WORLDS still points at PX4's own dir, but this specific world name
# resolves there too. GZ_SIM_RESOURCE_PATH (for our model) IS additive
# (gz_env.sh appends to whatever's already set), so that one just works via
# the env var below. See docs/decisions.md (ADR-0005) and worlds/apriltag.sdf.
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/apriltag.sdf"
REPO_WORLD="$REPO_ROOT/worlds/apriltag.sdf"

SIM_PID=""

mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_m4] Killing any stale px4 / gz sim processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 2
}

teardown_sim() {
    echo "[check_m4] Tearing down sim processes..."
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
    SIM_PID=""
}
trap teardown_sim EXIT

is_number() {
    [[ "$1" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]]
}

kill_stale_sim

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[check_m4] FAIL: PX4-Autopilot not found at $PX4_DIR"
    echo "FAIL"
    exit 1
fi

if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[check_m4] FAIL: venv python not found at $VENV_PYTHON (run the env setup first)"
    echo "FAIL"
    exit 1
fi

if [[ ! -f "$REPO_WORLD" ]]; then
    echo "[check_m4] FAIL: $REPO_WORLD not found"
    echo "FAIL"
    exit 1
fi

# Make sure the symlink into PX4's worlds dir exists and points at our repo
# copy (create/repair it rather than assuming a prior manual step held).
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    echo "[check_m4] Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi

declare -A MISS
declare -A CLEAN
declare -A EXITCODE

for LAW in pursuit pronav; do
    echo ""
    echo "[check_m4] ================ Law: $LAW ================"

    kill_stale_sim

    TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
    SIM_LOG="$LOGS_DIR/check_m4_sim_${LAW}_${TIMESTAMP}.log"
    RUN_LOG="$LOGS_DIR/check_m4_run_${LAW}_${TIMESTAMP}.log"

    echo "[check_m4] Launching HEADLESS PX4 SITL + Gazebo (gz_x500_mono_cam, world=apriltag) for law=$LAW..."
    echo "[check_m4] Sim output -> $SIM_LOG"

    # stdin must stay open: PX4's interactive pxh console spins on EOF and
    # floods the log with prompt reprints. `tail -f /dev/null` never EOFs,
    # so pxh blocks quietly; it dies with the group on teardown. (Same
    # pattern as check_m0/m1/m2/m3.sh.)
    setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    echo "[check_m4] Sim launched (pid/pgid $SIM_PID) for law=$LAW."

    echo "[check_m4] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\" in sim log..."
    ready=0
    elapsed=0
    while (( elapsed < READY_TIMEOUT_S )); do
        if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
            ready=1
            break
        fi
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "[check_m4] FAIL: sim process exited early (pid $SIM_PID) for law=$LAW. See $SIM_LOG"
            echo "FAIL"
            exit 1
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    if [[ "$ready" -ne 1 ]]; then
        echo "[check_m4] FAIL: did not see \"$READY_STRING\" within ${READY_TIMEOUT_S}s for law=$LAW. See $SIM_LOG"
        echo "FAIL"
        exit 1
    fi

    echo "[check_m4] Sim ready. Sleeping ${POST_READY_SETTLE_S}s to let the EKF settle before MAVSDK connects..."
    sleep "$POST_READY_SETTLE_S"

    echo "[check_m4] Pre-placing the tag at the engagement start ($TARGET_START_X, $TARGET_START_Y, $TARGET_START_Z)..."
    placed=0
    for attempt in 1 2 3; do
        if gz service -s /world/apriltag/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
            --req "name: \"apriltag_target\" position { x: $TARGET_START_X y: $TARGET_START_Y z: $TARGET_START_Z }"; then
            placed=1
            break
        fi
        echo "[check_m4] WARNING: tag pre-placement attempt $attempt failed, retrying..."
        sleep 1
    done
    if [[ "$placed" -ne 1 ]]; then
        echo "[check_m4] FAIL: could not pre-place the tag after 3 attempts for law=$LAW"
        echo "FAIL"
        exit 1
    fi
    echo "[check_m4] Tag pre-placed."

    EXTRA_ARGS=()
    if [[ "$LAW" == "pronav" ]]; then
        # GOALS.md's M4 gate ("< 1 m closest approach") is on pro-nav;
        # pursuit is the baseline being compared against, so its own miss
        # is reported but not gated.
        EXTRA_ARGS+=(--require-miss "$PRONAV_MISS_GATE_M")
    fi

    echo "[check_m4] Running m4_intercept.py --law $LAW ${EXTRA_ARGS[*]:-} (outer timeout ${PY_TIMEOUT_S}s)..."
    echo "[check_m4] Run output -> $RUN_LOG (also streamed below)"
    timeout "$PY_TIMEOUT_S" "$VENV_PYTHON" "$REPO_ROOT/scripts/m4_intercept.py" --law "$LAW" "${EXTRA_ARGS[@]}" 2>&1 | tee "$RUN_LOG"
    PY_EXIT="${PIPESTATUS[0]}"
    EXITCODE[$LAW]="$PY_EXIT"

    if [[ "$PY_EXIT" -eq 124 ]]; then
        echo "[check_m4] FAIL: m4_intercept.py (law=$LAW) hit the outer ${PY_TIMEOUT_S}s timeout. See $SIM_LOG and $RUN_LOG"
    fi

    RESULT_LINE="$(grep '^M4_RESULT ' "$RUN_LOG" | tail -n1)"
    if [[ -z "$RESULT_LINE" ]]; then
        echo "[check_m4] FAIL: no M4_RESULT line found in output for law=$LAW"
        MISS[$LAW]=""
        CLEAN[$LAW]=""
    else
        MISS[$LAW]="$(echo "$RESULT_LINE" | grep -oP '(?<=miss=)[^ ]+')"
        CLEAN[$LAW]="$(echo "$RESULT_LINE" | grep -oP '(?<=clean=)[^ ]+')"
        echo "[check_m4] Parsed: law=$LAW miss=${MISS[$LAW]} clean=${CLEAN[$LAW]} (exit $PY_EXIT)"
    fi

    teardown_sim
done

echo ""
echo "[check_m4] ================ Comparison ================"
echo "[check_m4] pursuit: miss=${MISS[pursuit]:-<missing>} m  clean=${CLEAN[pursuit]:-<missing>}  exit=${EXITCODE[pursuit]:-<missing>}"
echo "[check_m4] pronav : miss=${MISS[pronav]:-<missing>} m  clean=${CLEAN[pronav]:-<missing>}  exit=${EXITCODE[pronav]:-<missing>}"

PASS=1

for LAW in pursuit pronav; do
    if [[ "${EXITCODE[$LAW]:-1}" -eq 124 ]]; then
        PASS=0
    fi
    if [[ "${CLEAN[$LAW]:-}" != "1" ]]; then
        echo "[check_m4] FAIL reason: $LAW flight not clean (clean=${CLEAN[$LAW]:-<missing>})."
        PASS=0
    fi
    if ! is_number "${MISS[$LAW]:-}"; then
        echo "[check_m4] FAIL reason: $LAW miss distance not a valid number (${MISS[$LAW]:-<missing>})."
        PASS=0
    fi
done

if [[ "$PASS" -eq 1 ]] && ! awk -v m="${MISS[pronav]}" -v g="$PRONAV_MISS_GATE_M" 'BEGIN{exit !(m < g)}'; then
    echo "[check_m4] FAIL reason: pro-nav miss ${MISS[pronav]} m >= ${PRONAV_MISS_GATE_M} m gate."
    PASS=0
fi

if is_number "${MISS[pursuit]:-}" && is_number "${MISS[pronav]:-}"; then
    DELTA="$(awk -v a="${MISS[pursuit]}" -v b="${MISS[pronav]}" 'BEGIN{printf "%.3f", a-b}')"
    echo "[check_m4] delta (pursuit miss - pronav miss) = ${DELTA} m (positive = pro-nav closed tighter, as expected)"
    if awk -v a="${MISS[pursuit]}" -v b="${MISS[pronav]}" 'BEGIN{exit !(a < b)}'; then
        echo "[check_m4] *** WARNING ***: pursuit miss (${MISS[pursuit]} m) is SMALLER than pro-nav's"
        echo "[check_m4] *** WARNING ***: (${MISS[pronav]} m). That is the OPPOSITE of the expected"
        echo "[check_m4] *** WARNING ***: result (pro-nav should beat pursuit against a mover) and"
        echo "[check_m4] *** WARNING ***: needs investigation before this milestone can be honestly"
        echo "[check_m4] *** WARNING ***: closed out, even though the gate below may still pass on"
        echo "[check_m4] *** WARNING ***: pro-nav's own number."
    fi
else
    echo "[check_m4] Could not compare pursuit vs pronav miss numerically (one or both missing/nan)."
fi

if [[ "$PASS" -eq 1 ]]; then
    echo "[check_m4] PASS"
    exit 0
else
    echo "[check_m4] FAIL"
    exit 1
fi
