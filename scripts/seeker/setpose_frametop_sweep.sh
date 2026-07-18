#!/usr/bin/env bash
# FRAME-TOP × ATTITUDE recall sweep (ADR-0076 add #18k next_probe / build_plan P0.1).
#
# The set-pose sweep (setpose_recall_sweep.sh) proved recall is ~100% when the
# target is CENTRED. But IN FLIGHT recall is ~0.8% (#18i), and #18k reframed the
# wall as FLIGHT-DYNAMIC: the ~40° nose-down dash pitch parks the target at the
# FRAME-TOP EDGE. This decomposes that: a LEVEL disarmed camera (rock-stable, so
# the tilt-blind-gt bug can't bite), target teleported to the SAME range grid but
# at three conditions:
#   centered        el=0°   roll=0°   (control -> expect ~100%, reproduces the baseline)
#   frametop        el=+38° roll=0°   (target physically high -> sits at the frame TOP,
#                                       the in-flight off-boresight position)
#   frametop_banked el=+38° roll=30°  (adds a bank attitude on top of the edge position)
# Placing the target at elevation el is optically the SAME as a level target under
# an el-pitched camera (the frame-top pixel + look-up-from-below aspect) — so it
# isolates POSITION-IN-FRAME (centered vs frametop) and ATTITUDE (± bank) as the
# two flight-dynamic candidates. Score offline: resolution_probe.py <dir> (box_hits_gt),
# compare each condition's recall-vs-range to the centered control.
#   frametop << centered  -> the fixed-tilt (re-centre) mechanism is REAL (it fixes it).
#   frametop ~ centered   -> position isn't the wall; look at bank/blur/ego-motion.
# gt is SCORING-only (honesty). One sim, idle load, kill logic in-file (self-kill rule).
set -uo pipefail
cd "$(dirname "$0")/../.."
REPO_ROOT="$PWD"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PY="$REPO_ROOT/.venv-seeker/bin/python"
WORLD=quad_enemy
TARGET_MODEL=fpv_quad_enemy
OUT_ROOT="${FRAMETOP_OUT:-$REPO_ROOT/scripts/seeker/data/frametop_sweep}"
LOG_DIR="$REPO_ROOT/logs/frametop_sweep"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOG_DIR/sim_${STAMP}.log"
mkdir -p "$LOG_DIR" "$OUT_ROOT"
READY_STR="Startup script returned successfully"; READY_TIMEOUT_S=150; SETTLE_S=6
CAM_X=0.12; CAM_Y=0.03; CAM_Z=0.229   # level camera world pos (probe-measured)
RANGES=(8 12 16 20 25)
HOLD_S=7
source "$REPO_ROOT/scripts/sim_gpu_render.sh" 2>/dev/null || true

teardown_sim() { bash "$REPO_ROOT/scripts/sim_kill.sh" 2>/dev/null; pkill -f "capture_flight_frames.py" 2>/dev/null; sleep 3; }

# R el_deg roll_deg -> "tx ty tz qx qy qz qw" (target at elevation el, range R, yaw 180 + roll)
place() {
python3 - "$1" "$2" "$3" "$CAM_X" "$CAM_Y" "$CAM_Z" <<'PY'
import math, sys
R, el, roll = float(sys.argv[1]), math.radians(float(sys.argv[2])), math.radians(float(sys.argv[3]))
cx, cy, cz = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
yaw = math.pi  # face -x, toward the camera (approach aspect)
tx = cx + R*math.cos(el); ty = cy; tz = cz + R*math.sin(el)
cyaw, syaw = math.cos(yaw/2), math.sin(yaw/2)
cp, sp = 1.0, 0.0                       # pitch 0
cr, sr = math.cos(roll/2), math.sin(roll/2)
w  = cr*cp*cyaw + sr*sp*syaw
qx = sr*cp*cyaw - cr*sp*syaw
qy = cr*sp*cyaw + sr*cp*syaw
qz = cr*cp*syaw - sr*sp*cyaw
print(f"{tx:.4f} {ty:.4f} {tz:.4f} {qx:.6f} {qy:.6f} {qz:.6f} {w:.6f}")
PY
}

set_target() { # x y z qx qy qz qw
    for a in 1 2 3; do
        gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
            --timeout 2000 --req "name: \"$TARGET_MODEL\" position { x: $1 y: $2 z: $3 } orientation { x: $4 y: $5 z: $6 w: $7 }" \
            >/dev/null 2>&1 && return 0
        sleep 1
    done
    return 1
}

echo "[frametop] boot $WORLD ..."
teardown_sim
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" > "$SIM_LOG" 2>&1 &
ready=0; for ((i=0;i<READY_TIMEOUT_S;i++)); do grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null && { ready=1; break; }; sleep 1; done
[[ "$ready" -ne 1 ]] && { echo "[frametop] FAIL sim-ready (see $SIM_LOG)"; teardown_sim; exit 1; }
sleep "$SETTLE_S"

# condition | el_deg | roll_deg
CONDS=("centered|0|0" "frametop|38|0" "frametop_banked|38|30")
for spec in "${CONDS[@]}"; do
    IFS='|' read -r cond el roll <<< "$spec"
    OUT="$OUT_ROOT/$cond"; mkdir -p "$OUT"
    REC_LOG="$LOG_DIR/rec_${cond}_${STAMP}.log"
    echo "[frametop] ===== $cond (el=$el° roll=$roll°) -> $OUT ====="
    INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
        "$VENV_PY" "$REPO_ROOT/scripts/seeker/capture_flight_frames.py" \
        --out "$OUT" --run-tag "$cond" --every 2 --extent-m 0.35 --max-frames 4000 > "$REC_LOG" 2>&1 &
    REC_PID=$!; sleep 2
    kill -0 "$REC_PID" 2>/dev/null || { echo "[frametop] FAIL recorder $cond"; teardown_sim; exit 1; }
    for R in "${RANGES[@]}"; do
        read -r tx ty tz qx qy qz qw <<< "$(place "$R" "$el" "$roll")"
        if set_target "$tx" "$ty" "$tz" "$qx" "$qy" "$qz" "$qw"; then
            echo "[frametop]   $cond r=$R -> ($tx,$ty,$tz)"
        else echo "[frametop]   WARN set_pose failed $cond r=$R"; fi
        sleep "$HOLD_S"
    done
    kill -TERM "$REC_PID" 2>/dev/null; for ((i=0;i<15;i++)); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done; kill -KILL "$REC_PID" 2>/dev/null
    echo "[frametop]   $cond frames: $(ls "$OUT/images" 2>/dev/null | wc -l)"
done
teardown_sim
echo "[frametop] DONE. Score: for c in centered frametop frametop_banked; do echo == \$c ==; $VENV_PY scripts/seeker/resolution_probe.py $OUT_ROOT/\$c --rmin 6 --rmax 27; done"
