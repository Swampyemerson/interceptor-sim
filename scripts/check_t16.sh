#!/usr/bin/env bash
# T16 scripted milestone gate (Phase 2, ADR-0046 F1: "trajectory-matched
# offline-render replay"): headless PX4 SITL + Gazebo boots on the new
# `stereo_intercept` world (worlds/stereo_intercept.sdf), the ground stereo
# rig's two camera sensors (models/ground_stereo_rig/) publish live frames
# at the design resolution, and scripts/rig_snapshot_capture.py's --smoke
# mode successfully teleports the target + captures synchronized L/R pairs
# through the real gz-transport pipeline. See docs/decisions.md ADR-0046,
# docs/phase2_sim_to_real_plan.md Sec 5.5-5.6, and
# .claude/skills/px4-gazebo/SKILL.md for launch/shutdown conventions.
#
# Usage: scripts/check_t16.sh
# Exit code: 0 = PASS, non-zero = FAIL (first failing step wins).
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_SEEKER_PYTHON="$REPO_ROOT/.venv-seeker/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/check_t16_sim_${TIMESTAMP}.log"
SMOKE_OUT="$LOGS_DIR/rig_captures/check_t16_smoke_${TIMESTAMP}"
READY_TIMEOUT_S=180
# NOTE: do NOT wait for "Ready for takeoff!" here -- see ADR-0004 /
# .claude/skills/px4-gazebo/SKILL.md. "Startup script returned successfully"
# is PX4's unconditional boot-complete line.
READY_STRING="Startup script returned successfully"
POST_READY_SETTLE_S=5
TOPIC_ECHO_TIMEOUT_S=15

WORLD_NAME="stereo_intercept"
RIG_MODEL="ground_stereo_rig"
TARGET_MODEL="fpv_target_markerless"
LEFT_TOPIC="/world/${WORLD_NAME}/model/${RIG_MODEL}/link/link/sensor/rig_left/image"
RIGHT_TOPIC="/world/${WORLD_NAME}/model/${RIG_MODEL}/link/link/sensor/rig_right/image"
EXPECTED_W=1920
EXPECTED_H=1200

# worlds/stereo_intercept.sdf is the checked-in source of truth, but PX4's
# own px4-rc.gzsim resolves world files as ${PX4_GZ_WORLDS}/<name>.sdf, and
# gz_env.sh (sourced at runtime from the SITL working dir) unconditionally
# overwrites PX4_GZ_WORLDS to PX4's own Tools/simulation/gz/worlds
# directory -- not additive, so a symlink is the fix (ADR-0005, same
# pattern as check_m2.sh/check_m3.sh). GZ_SIM_RESOURCE_PATH (for our
# models/ dir, incl. ground_stereo_rig and fpv_target_markerless) IS
# additive via the env var below.
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/${WORLD_NAME}.sdf"
REPO_WORLD="$REPO_ROOT/worlds/${WORLD_NAME}.sdf"

SIM_PID=""
STEP_FAIL=0

mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_t16] Killing any stale px4 / gz sim processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 2
}

cleanup() {
    echo "[check_t16] Cleaning up sim processes..."
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

echo "=== [check_t16] STEP (i): PX4 world symlink ==="
if [[ ! -d "$PX4_DIR" ]]; then
    echo "[check_t16] FAIL(i): PX4-Autopilot not found at $PX4_DIR"
    echo "FAIL"; exit 1
fi
if [[ ! -x "$VENV_SEEKER_PYTHON" ]]; then
    echo "[check_t16] FAIL(i): seeker venv python not found at $VENV_SEEKER_PYTHON"
    echo "FAIL"; exit 1
fi
if [[ ! -f "$REPO_WORLD" ]]; then
    echo "[check_t16] FAIL(i): $REPO_WORLD not found"
    echo "FAIL"; exit 1
fi
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    echo "[check_t16] Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi
echo "[check_t16] STEP (i) PASS: world symlink in place."

echo "=== [check_t16] STEP (ii): boot PX4 SITL + Gazebo headless (world=$WORLD_NAME) ==="
echo "[check_t16] Sim output -> $SIM_LOG"

# stdin must stay open: PX4's interactive pxh console spins on EOF and floods
# the log with prompt reprints. `tail -f /dev/null` never EOFs, so pxh blocks
# quietly; it dies with the group on cleanup. (Same pattern as
# check_m0/m1/m2/m3.sh -- COPIED EXACTLY, do not simplify away.)
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD_NAME GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[check_t16] Sim launched (pid/pgid $SIM_PID)."

echo "[check_t16] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\" in sim log..."
ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[check_t16] FAIL(ii): sim process exited early (pid $SIM_PID). See $SIM_LOG"
        echo "FAIL"; exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done

if [[ "$ready" -ne 1 ]]; then
    echo "[check_t16] FAIL(ii): did not see \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    echo "FAIL"; exit 1
fi
echo "[check_t16] STEP (ii) PASS: sim booted. Settling ${POST_READY_SETTLE_S}s..."
sleep "$POST_READY_SETTLE_S"

echo "=== [check_t16] STEP (iii): rig camera topics live at ${EXPECTED_W}x${EXPECTED_H} ==="
STEP3_OK=1
TOPIC_LIST="$(gz topic -l 2>/dev/null)"
for T in "$LEFT_TOPIC" "$RIGHT_TOPIC"; do
    if ! grep -qF "$T" <<< "$TOPIC_LIST"; then
        echo "[check_t16] FAIL(iii): topic not found in 'gz topic -l': $T"
        STEP3_OK=0
        continue
    fi
    echo "[check_t16] topic present: $T"
    # -n 1 bounds the echo so it EXITS BY ITSELF after one message; plain
    # `timeout gz topic ...` wedges because gz is a Ruby launcher wrapping
    # the real echoer process -- TERM hits the wrapper, not the child, and
    # the pipe never closes (scripts/probe_stereo_rtf.sh's rtf_stats()
    # comment documents this the hard way). timeout stays as a
    # belt-and-suspenders backstop only.
    ECHO_OUT="$(timeout "$TOPIC_ECHO_TIMEOUT_S" gz topic -e -t "$T" -n 1 2>/dev/null)"
    if [[ -z "$ECHO_OUT" ]]; then
        echo "[check_t16] FAIL(iii): no message received on $T within ${TOPIC_ECHO_TIMEOUT_S}s"
        STEP3_OK=0
        continue
    fi
    W="$(grep -m1 '^width:' <<< "$ECHO_OUT" | awk '{print $2}')"
    H="$(grep -m1 '^height:' <<< "$ECHO_OUT" | awk '{print $2}')"
    if [[ "$W" != "$EXPECTED_W" || "$H" != "$EXPECTED_H" ]]; then
        echo "[check_t16] FAIL(iii): $T reported ${W}x${H}, expected ${EXPECTED_W}x${EXPECTED_H}"
        STEP3_OK=0
        continue
    fi
    echo "[check_t16] $T delivered a frame at ${W}x${H}."
done

if [[ "$STEP3_OK" -ne 1 ]]; then
    echo "[check_t16] FAIL(iii): rig camera topic check failed. See above."
    echo "FAIL"; exit 1
fi
echo "[check_t16] STEP (iii) PASS."

echo "=== [check_t16] STEP (iv): rig_snapshot_capture.py --smoke ==="
mkdir -p "$SMOKE_OUT"
INTERCEPTOR_WORLD_NAME="$WORLD_NAME" INTERCEPTOR_TARGET_MODEL="$TARGET_MODEL" \
    "$VENV_SEEKER_PYTHON" "$REPO_ROOT/scripts/rig_snapshot_capture.py" \
    --smoke --out "$SMOKE_OUT"
CAPTURE_EXIT=$?

if [[ "$CAPTURE_EXIT" -ne 0 ]]; then
    echo "[check_t16] FAIL(iv): rig_snapshot_capture.py --smoke exited $CAPTURE_EXIT"
    echo "FAIL"; exit 1
fi

# Verify: index.csv has exactly 3 data rows, and each L/R pair decodes as
# EXPECTED_Wx EXPECTED_H, non-black (stddev > 1.0, matching m1_capture.py's
# MIN_STDDEV convention).
VERIFY_OUT="$("$VENV_SEEKER_PYTHON" - "$SMOKE_OUT" "$EXPECTED_W" "$EXPECTED_H" <<'PYEOF'
import csv, os, sys
import cv2
import numpy as np

out_dir, exp_w, exp_h = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
index_path = os.path.join(out_dir, "index.csv")
if not os.path.isfile(index_path):
    print(f"FAIL: index.csv missing at {index_path}")
    sys.exit(1)

with open(index_path, newline="") as fh:
    rows = list(csv.DictReader(fh))

if len(rows) != 3:
    print(f"FAIL: expected 3 index.csv rows, got {len(rows)}")
    sys.exit(1)

for row in rows:
    seq = int(row["seq"])
    for side in ("left", "right"):
        p = os.path.join(out_dir, side, f"{seq:05d}.png")
        if not os.path.isfile(p):
            print(f"FAIL: missing {p}")
            sys.exit(1)
        img = cv2.imread(p)
        if img is None:
            print(f"FAIL: {p} did not decode")
            sys.exit(1)
        h, w = img.shape[:2]
        if (w, h) != (exp_w, exp_h):
            print(f"FAIL: {p} is {w}x{h}, expected {exp_w}x{exp_h}")
            sys.exit(1)
        stddev = float(np.std(img))
        if stddev <= 1.0:
            print(f"FAIL: {p} looks black/flat (stddev={stddev:.3f})")
            sys.exit(1)
        print(f"OK: {p} {w}x{h} stddev={stddev:.2f}")

print("VERIFY_PASS")
sys.exit(0)
PYEOF
)"
VERIFY_EXIT=$?
echo "$VERIFY_OUT"

if [[ "$VERIFY_EXIT" -ne 0 ]] || ! grep -q "VERIFY_PASS" <<< "$VERIFY_OUT"; then
    echo "[check_t16] FAIL(iv): captured L/R pairs failed verification. See $SMOKE_OUT"
    echo "FAIL"; exit 1
fi
echo "[check_t16] STEP (iv) PASS: 3/3 L/R pairs verified at ${EXPECTED_W}x${EXPECTED_H}, non-black, index.csv OK."

echo "=== [check_t16] STEP (v): clean teardown ==="
# cleanup() runs via the EXIT trap; this step just confirms exit 0 reaches
# it. No inline pkill patterns here that could match this script's own
# argv -- cleanup()'s patterns match px4/gz sim processes only, and this
# script's own cmdline is just its path (same reasoning as sim_kill.sh's
# header comment).
echo "[check_t16] STEP (v) will run via EXIT trap."

echo "[check_t16] PASS"
exit 0
