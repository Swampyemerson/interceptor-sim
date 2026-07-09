#!/usr/bin/env bash
# T19 scripted milestone gate: the LIVE stereo-cue intercept flight.
# Boots headless PX4 SITL + Gazebo on the markerless world, flies
# scripts/m4_intercept.py under `--cue-source stereo` (ADR-0046/0048/0051):
# m4 spawns scripts/ground_station/station.py, which replays the T16/T18
# trajectory-matched capture (logs/rig_captures/full_sweep_20260709T015530Z)
# through the REAL detect(cached)->triangulate->track pipeline, LIVE during
# the flight, over the SAME :47800 UDP JSON cue wire the mock uses. This is
# T19b's own "OWES #2/#3": the flight gate, AND the first time this module's
# dual-gate/clock path is driven by a REAL PX4/Gazebo /clock under real RTF
# (ADR-0051's "T19b OWES" #3 -- never exercised end-to-end before this).
#
# See docs/decisions.md ADR-0046 (rig-as-offline-render-replay), ADR-0048
# (the :47800 wire + spawn lifecycle + dual-gate sim-time discipline),
# ADR-0050 (T18 real-detector sigma_R checkpoint), ADR-0051 (centroid-cache
# replay -- what this flight actually exercises), scripts/ground_station/
# station.py (THE spec, read its module docstring first), and
# scripts/check_s2.sh / scripts/check_t16.sh (the boot/teardown/audit
# patterns this script is adapted from).
#
# MOVER-CACHE ALIGNMENT (read before touching the trajectory args below):
# logs/rig_captures/full_sweep_20260709T015530Z/capture_meta.json records
# dash_x0_m=6.5, dash_y0_mag_m=29.3, dash_z0_m=0.5, dash_speed_m_s=9.0.
# station.py defaults `--dash-direction` to whichever direction the FIRST
# index.csv row belongs to -- that is "r2l" (index.csv seq 0-27), which
# starts at world y=+29.3 and DECREASES (9.0 m/s) to y=+5.0 over t_sim
# 0.0->2.7s (index.csv's own gt_target_y column, confirmed by direct read,
# NOT assumed). So the mover this gate flies MUST also start at
# (x=6.5, y=+29.3, z=0.5) and fly toward -y at 9.0 m/s to match: this is
# mc_batch.sh's own "r2l" sign convention (sign=-1.0 => start_y=+y0_mag,
# vy=-speed), independently re-derived here, not copied blind.
#
# KNOWN OPEN QUESTION THIS GATE EXISTS TO ANSWER (do not paper over it):
# station.py's dual-gate release condition is `sim_t >= deliver_t_sim`
# where `deliver_t_sim = frame_t_sim + latency_s` and `frame_t_sim` is the
# capture's BAKED t_sim_nominal (0.0-2.7s, from an EARLIER, separate capture
# run -- ADR-0048 SS3's own module docstring). station.py is spawned by
# m4_intercept.py at CUE_WAIT entry, which is AFTER this flight's own
# PX4 boot + settle + arm + takeoff -- i.e. THIS flight's live /clock is
# already tens of seconds in by the time station.py subscribes, vastly
# past the cache's ~2.82s max deadline. A prior smoke run
# (logs/ground_station_20260709T044854Z.csv) already shows exactly this:
# every one of 26 emitted cues shares the SAME t_sim_emit (12.124s) -- a
# single-instant burst, not a paced 2.7s stream. This gate's assertion 4
# is the honest instrument for whether that burst still lands close enough
# to the actual (matched) mover trajectory to count as "flying on the real
# cue" -- reported numerically, not asserted away.
#
# Usage: scripts/check_t19.sh
# Exit code: 0 = every pre-registered assertion PASSES, non-zero = FAIL
# (assertion 4's alignment number is always REPORTED, never silently
# forced to pass -- see the assertion-4 block below for its own threshold).
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"
# --seeker markerless needs onnxruntime (absent from the main .venv, ADR-0048
# SS5's own interpreter table) -- .venv-seeker is the ONE venv with gz +
# mavsdk + onnxruntime + cv2 together (mc_batch.sh's MC_VENV_PYTHON
# precedent, NEXT.md "Markerless flight venv"). The flight process itself
# (m4_intercept.py) runs under THIS interpreter, not .venv.
VENV_SEEKER_PYTHON="$REPO_ROOT/.venv-seeker/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
CAPTURE_DIR="$REPO_ROOT/logs/rig_captures/full_sweep_20260709T015530Z"
STEREO_CACHE="$CAPTURE_DIR/centroids.csv"
MARKERLESS_WEIGHTS="$REPO_ROOT/scripts/seeker/weights/drone_finetuned_v2.onnx"

READY_TIMEOUT_S=180
# NOTE: do NOT wait for "Ready for takeoff!" -- ADR-0004 / check_s2.sh.
READY_STRING="Startup script returned successfully"
POST_READY_SETTLE_S=5
# CUE_WAIT 15s + DASH 20s + ENGAGE 30s budget (check_s2.sh's own headroom),
# plus connect/health/climb/land overhead.
PY_TIMEOUT_S=360

WORLD_NAME="markerless"
TARGET_MODEL="fpv_target_markerless"
export INTERCEPTOR_WORLD_NAME="$WORLD_NAME"
export INTERCEPTOR_TARGET_MODEL="$TARGET_MODEL"

# Mover-cache alignment (see header comment): r2l, x0=6.5, y0=+29.3, z0=0.5,
# speed 9.0 m/s toward -y.
TARGET_START="6.5,29.3,0.5"
TARGET_VEL="0,-9.0"

PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/${WORLD_NAME}.sdf"
REPO_WORLD="$REPO_ROOT/worlds/${WORLD_NAME}.sdf"

SIM_PID=""

mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_t19] Killing any stale px4 / gz / cue / mover processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    pkill -f "ground_station/station.py" 2>/dev/null
    pkill -f "s2_cue_mock.py" 2>/dev/null
    pkill -f "m4_target_mover.py" 2>/dev/null
    sleep 2
}

teardown_sim() {
    echo "[check_t19] Tearing down sim processes..."
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
    pkill -f "ground_station/station.py" 2>/dev/null
    pkill -f "s2_cue_mock.py" 2>/dev/null
    pkill -f "m4_target_mover.py" 2>/dev/null
    # gz sim's renderer can take a few seconds to unwind after SIGTERM.
    sleep 3
    SIM_PID=""
}
trap teardown_sim EXIT

echo "=== [check_t19] PREFLIGHT ==="
FAIL=0

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[check_t19] FAIL: PX4-Autopilot not found at $PX4_DIR"
    echo "FAIL"; exit 1
fi
if [[ ! -x "$VENV_SEEKER_PYTHON" ]]; then
    echo "[check_t19] FAIL: .venv-seeker python not found/executable at $VENV_SEEKER_PYTHON"
    echo "FAIL"; exit 1
fi
if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[check_t19] FAIL: main .venv python not found/executable at $VENV_PYTHON"
    echo "FAIL"; exit 1
fi
if [[ ! -f "$REPO_WORLD" ]]; then
    echo "[check_t19] FAIL: $REPO_WORLD not found"
    echo "FAIL"; exit 1
fi
if [[ ! -f "$STEREO_CACHE" ]]; then
    echo "[check_t19] FAIL: centroid cache not found at $STEREO_CACHE"
    echo "FAIL"; exit 1
fi
if [[ ! -f "$MARKERLESS_WEIGHTS" ]]; then
    echo "[check_t19] FAIL: markerless terminal-seeker weights not found at $MARKERLESS_WEIGHTS"
    echo "FAIL"; exit 1
fi
echo "[check_t19] PASS: PX4 dir, venvs, world file, centroid cache, seeker weights all present."

# --- idle-machine check (ONE sim at a time; batches/gates only on an idle
# machine per project convention) -- 1-minute load average must be < 1.5.
LOAD1="$(uptime | grep -oP 'load average:\s*\K[0-9.]+' || echo "999")"
echo "[check_t19] uptime: $(uptime)"
if awk "BEGIN{exit !($LOAD1 < 1.5)}"; then
    echo "[check_t19] PASS: 1-min load average $LOAD1 < 1.5 (machine idle)"
else
    echo "[check_t19] FAIL: 1-min load average $LOAD1 >= 1.5 -- machine not idle, refusing to boot a sim"
    echo "FAIL"; exit 1
fi

kill_stale_sim

echo "=== [check_t19] World symlink ==="
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    echo "[check_t19] Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi
echo "[check_t19] World symlink in place."

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/check_t19_sim_${TIMESTAMP}.log"
RUN_LOG="$LOGS_DIR/check_t19_run_${TIMESTAMP}.log"

echo "=== [check_t19] Boot HEADLESS PX4 SITL + Gazebo (gz_x500_mono_cam, world=$WORLD_NAME, GALLIUM_DRIVER=d3d12) ==="
echo "[check_t19] Sim output -> $SIM_LOG"

# stdin must stay open: PX4's interactive pxh console spins on EOF and floods
# the log with prompt reprints. `tail -f /dev/null` never EOFs, so pxh blocks
# quietly; it dies with the group on teardown. (Same pattern as
# check_m0/m1/m2/m3/s1/s2/t16.sh -- COPIED EXACTLY, do not simplify away.)
# GALLIUM_DRIVER=d3d12: ADR-0046 addendum -- stable RTF, the smoke boot used
# it (12s), and check_m3.sh's own d3d12 validation run passed clean.
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD_NAME GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 GALLIUM_DRIVER=d3d12 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[check_t19] Sim launched (pid/pgid $SIM_PID)."

echo "[check_t19] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\" in sim log..."
ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[check_t19] FAIL: sim process exited early (pid $SIM_PID). See $SIM_LOG"
        echo "FAIL"; exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done

if [[ "$ready" -ne 1 ]]; then
    echo "[check_t19] FAIL: did not see \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    echo "FAIL"; exit 1
fi
echo "[check_t19] Sim ready. Sleeping ${POST_READY_SETTLE_S}s to let the EKF settle before MAVSDK connects..."
sleep "$POST_READY_SETTLE_S"

echo "=== [check_t19] Pre-place $TARGET_MODEL at ($TARGET_START) ==="
placed=0
IFS=',' read -r PX PY PZ <<< "$TARGET_START"
for attempt in 1 2 3; do
    if gz service -s "/world/${WORLD_NAME}/set_pose" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
        --req "name: \"${TARGET_MODEL}\" position { x: $PX y: $PY z: $PZ }"; then
        placed=1
        break
    fi
    echo "[check_t19] WARNING: target pre-placement attempt $attempt failed, retrying..."
    sleep 1
done
if [[ "$placed" -ne 1 ]]; then
    echo "[check_t19] FAIL: could not pre-place $TARGET_MODEL after 3 attempts"
    echo "FAIL"; exit 1
fi
echo "[check_t19] Target pre-placed."

echo "=== [check_t19] Flight: m4_intercept.py --law pronav --fpv --handoff --cue-source stereo --seeker markerless ==="
export STEREO_CENTROID_CACHE="$STEREO_CACHE"
export STEREO_CAPTURE_DIR="$CAPTURE_DIR"
export GROUND_STATION_VENV_PYTHON="$VENV_SEEKER_PYTHON"
export MARKERLESS_NN_WEIGHTS="$MARKERLESS_WEIGHTS"
echo "[check_t19] STEREO_CENTROID_CACHE=$STEREO_CENTROID_CACHE"
echo "[check_t19] STEREO_CAPTURE_DIR=$STEREO_CAPTURE_DIR"
echo "[check_t19] GROUND_STATION_VENV_PYTHON=$GROUND_STATION_VENV_PYTHON"
echo "[check_t19] MARKERLESS_NN_WEIGHTS=$MARKERLESS_NN_WEIGHTS"
echo "[check_t19] target_start=$TARGET_START target_vel=$TARGET_VEL (r2l, matches the cache -- see header comment)"
echo "[check_t19] Run output -> $RUN_LOG (also streamed below)"

timeout "$PY_TIMEOUT_S" "$VENV_SEEKER_PYTHON" "$REPO_ROOT/scripts/m4_intercept.py" \
    --law pronav --fpv --handoff --cue-source stereo --seeker markerless \
    --target-start="$TARGET_START" --target-vel="$TARGET_VEL" \
    2>&1 | tee "$RUN_LOG"
PY_EXIT="${PIPESTATUS[0]}"
echo "[check_t19] Flight process exit code: $PY_EXIT"

CSV_PATH="$(grep -oP '(?<=\[m4\] Logging to )\S+\.csv' "$RUN_LOG" | tail -n1)"
echo "[check_t19] Flight CSV: ${CSV_PATH:-<none found>}"

teardown_sim

echo ""
echo "=== [check_t19] GATE ASSERTIONS ==="
G_FAIL=0

# --- Assertion 1: the stereo branch spawned, not the mock. ---
if grep -q "cue stereo (ground_station)" "$RUN_LOG"; then
    echo "[check_t19] PASS (1): 'cue stereo (ground_station)' found -- stereo branch spawned, not mock."
else
    echo "[check_t19] FAIL (1): 'cue stereo (ground_station)' NOT found in run output."
    G_FAIL=1
fi

# --- Assertion 2: n_cue_received > 0 AND CUE_WAIT completed (cue lock). ---
CUE_LOCK_LINE="$(grep '^\[s2\] Cue lock at t=' "$RUN_LOG" | tail -n1)"
if [[ -n "$CUE_LOCK_LINE" ]]; then
    N_CUE_RECEIVED="$(echo "$CUE_LOCK_LINE" | grep -oP '\(\K[0-9]+(?= datagrams)')"
    N_TRACKER_CORR="$(echo "$CUE_LOCK_LINE" | grep -oP '[0-9]+(?= tracker)')"
    if [[ -n "$N_CUE_RECEIVED" ]] && [[ "$N_CUE_RECEIVED" -gt 0 ]]; then
        echo "[check_t19] PASS (2): CUE_WAIT completed -- $CUE_LOCK_LINE (n_cue_received=$N_CUE_RECEIVED, tracker_corrections=$N_TRACKER_CORR)"
    else
        echo "[check_t19] FAIL (2): cue-lock line found but n_cue_received not > 0: $CUE_LOCK_LINE"
        G_FAIL=1
    fi
else
    echo "[check_t19] FAIL (2): no '[s2] Cue lock at t=' line -- CUE_WAIT never completed (timed out or aborted)."
    ABORT_LINE="$(grep -i 'CUE_WAIT:\|breakoff_reason=' "$RUN_LOG" | tail -n3)"
    [[ -n "$ABORT_LINE" ]] && echo "[check_t19]   context: $ABORT_LINE"
    G_FAIL=1
fi

# --- Assertion 3: handoff occurred (outcome != no_handoff / handoff_t exists). ---
S2_RESULT_LINE="$(grep '^S2_RESULT ' "$RUN_LOG" | tail -n1)"
OUTCOME=""
HANDOFF_FLAG=""
MISS_VAL=""
if [[ -n "$S2_RESULT_LINE" ]]; then
    OUTCOME="$(echo "$S2_RESULT_LINE" | grep -oP '(?<=outcome=)\S+')"
    HANDOFF_FLAG="$(echo "$S2_RESULT_LINE" | grep -oP '(?<= handoff=)[^ ]+')"
    MISS_VAL="$(echo "$S2_RESULT_LINE" | grep -oP '(?<=miss=)[^ ]+')"
    echo "[check_t19] S2_RESULT: $S2_RESULT_LINE"
fi
if [[ "$HANDOFF_FLAG" == "1" && -n "$OUTCOME" && "$OUTCOME" != "no_handoff" ]]; then
    echo "[check_t19] PASS (3): handoff occurred (handoff=$HANDOFF_FLAG outcome=$OUTCOME)."
else
    echo "[check_t19] FAIL (3): handoff did not occur (handoff=${HANDOFF_FLAG:-<missing>} outcome=${OUTCOME:-<missing>})."
    G_FAIL=1
fi

# --- Assertion 4: ALIGNMENT PROOF -- cue-vs-actual-target error during
# mid-course, computed from the flight CSV's own ext_x/ext_y/ext_z (cue
# position that actually corrected the tracker, NED: ext_x=north=world_y,
# ext_y=east=world_x -- ADR-0013's world-frame mapping) vs gt_tag_x/y/z
# (world frame, SCORING-ONLY per the honesty boundary, legitimate for this
# audit exactly like check_s2.sh's own audit (d)). NOT gated to a pass/fail
# by a tight number -- reported honestly; only fails if the error is so
# large (>5 m) it signals outright misalignment, per the task's own bar.
ALIGN_OUT=""
if [[ -n "$CSV_PATH" && -f "$CSV_PATH" ]]; then
    ALIGN_OUT="$("$VENV_PYTHON" - "$CSV_PATH" <<'PYEOF'
import csv
import math
import sys

path = sys.argv[1]
with open(path, newline="") as f:
    rows = list(csv.DictReader(f))

errs = []
for r in rows:
    if r.get("ext_fresh") != "1":
        continue
    try:
        # ext_x(csv) = corr_n = cue world_y; ext_y(csv) = corr_e = cue world_x
        # (S2 WORLD-FRAME MAPPING, m4_intercept.py module docstring).
        cue_world_x = float(r["ext_y"])
        cue_world_y = float(r["ext_x"])
        cue_world_z = float(r["ext_z"])
        gt_x = float(r["gt_tag_x"])
        gt_y = float(r["gt_tag_y"])
        gt_z = float(r["gt_tag_z"])
    except (KeyError, ValueError, TypeError):
        continue
    err = math.sqrt((cue_world_x - gt_x) ** 2 + (cue_world_y - gt_y) ** 2 + (cue_world_z - gt_z) ** 2)
    errs.append(err)

if not errs:
    print("N=0")
    sys.exit(0)

errs.sort()
n = len(errs)
median = errs[n // 2] if n % 2 == 1 else 0.5 * (errs[n // 2 - 1] + errs[n // 2])
print(f"N={n} MEDIAN={median:.4f} MAX={max(errs):.4f} MIN={min(errs):.4f} MEAN={sum(errs)/n:.4f}")
PYEOF
)"
    echo "[check_t19] Cue-vs-target alignment (mid-course, ext_fresh=1 rows): $ALIGN_OUT"
    # (?<!\w) not (?<=...): plain "(?<=N=)" also matches the "N=" tail of
    # "MEDIAN=..." (a real bug hit on the first live run -- 'MEDIA[N=]24.36'
    # is a false match) -- exclude any lookbehind preceded by a word char.
    N_ALIGN="$(echo "$ALIGN_OUT" | grep -oP '(?<!\w)N=\K[0-9]+')"
    MAX_ALIGN="$(echo "$ALIGN_OUT" | grep -oP '(?<=MAX=)[0-9.]+')"
    if [[ -z "$N_ALIGN" || "$N_ALIGN" -eq 0 ]]; then
        echo "[check_t19] FAIL (4): zero ext_fresh=1 rows in the CSV -- no cue-corrected ticks to check alignment on."
        G_FAIL=1
    elif [[ -n "$MAX_ALIGN" ]] && awk "BEGIN{exit !($MAX_ALIGN > 5.0)}"; then
        echo "[check_t19] FAIL (4): max cue-vs-target error $MAX_ALIGN m > 5.0 m -- MISALIGNED trajectory/timing, reported honestly."
        G_FAIL=1
    else
        echo "[check_t19] PASS (4): cue-vs-target error stays <= 5.0 m (max=$MAX_ALIGN m)."
    fi
else
    echo "[check_t19] FAIL (4): no flight CSV to analyze."
    G_FAIL=1
fi

# --- Assertion 5: honesty -- scripts/audit_per_tick.py on this flight. ---
if [[ -n "$CSV_PATH" && -f "$CSV_PATH" ]]; then
    AUDIT_OUT="$("$VENV_PYTHON" "$REPO_ROOT/scripts/audit_per_tick.py" --flight-csv "$CSV_PATH" --law pronav 2>&1)"
    AUDIT_EXIT=$?
    echo "[check_t19] audit_per_tick.py output:"
    echo "$AUDIT_OUT" | sed 's/^/[check_t19]   /'
    if [[ "$AUDIT_EXIT" -eq 0 ]]; then
        echo "[check_t19] PASS (5): audit_per_tick.py exit 0 (honest -- no gt leak into guidance)."
    else
        echo "[check_t19] FAIL (5): audit_per_tick.py exit $AUDIT_EXIT."
        G_FAIL=1
    fi
else
    echo "[check_t19] FAIL (5): no flight CSV to audit."
    G_FAIL=1
fi

# --- Assertion 6: flight completes without crash; report final miss. ---
# Exit 0 (clean intercept) or 1 (scored-but-not-clean, e.g. no handoff/miss
# too large) are both a real, graceful completion of m4_intercept.py's own
# state machine -- NOT a crash. 124 (outer wall timeout) or anything else
# (signal-killed, uncaught exception) counts as a crash for this assertion.
# This gate does NOT threshold the miss number itself (task brief: "the
# gate is flies-on-the-real-cue + handoff + honesty + alignment, not a
# precision number").
if [[ -n "$CSV_PATH" && -f "$CSV_PATH" && ( "$PY_EXIT" -eq 0 || "$PY_EXIT" -eq 1 ) ]]; then
    echo "[check_t19] PASS (6): flight completed without crash (exit=$PY_EXIT). Final miss=${MISS_VAL:-<missing>} m outcome=${OUTCOME:-<missing>}."
else
    echo "[check_t19] FAIL (6): flight did not complete cleanly (exit=$PY_EXIT, csv=${CSV_PATH:-<none>})."
    G_FAIL=1
fi

echo ""
echo "=== [check_t19] SUMMARY ==="
echo "[check_t19] sim log: $SIM_LOG"
echo "[check_t19] run log: $RUN_LOG"
echo "[check_t19] flight csv: ${CSV_PATH:-<none>}"
echo "[check_t19] S2_RESULT: ${S2_RESULT_LINE:-<none>}"
echo "[check_t19] alignment: ${ALIGN_OUT:-<none>}"

if [[ "$G_FAIL" -eq 0 ]]; then
    echo "[check_t19] PASS"
    exit 0
else
    echo "[check_t19] FAIL"
    exit 1
fi
