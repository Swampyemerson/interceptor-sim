#!/usr/bin/env bash
# CONTROLLED SET-POSE RECALL SWEEP (ADR-0076 add #18k part 3, Fable round-4 rec).
#
# The decisive LEVEL-camera recall-vs-range measurement, isolating the DETECTOR
# from the pointing confound. Prior approach captures were pointing-limited (the
# nose-down dash pitch ejects the target out the frame top >11 m), so "recall ~0
# >10 m" conflated not-in-view with not-detected, on n=5 frames. Here the
# interceptor sits DISARMED on the ground (a rock-stable LEVEL forward camera at
# world ~(0.12,0.03,0.23), boresight +x -- verified by _probe_quad_geometry.sh),
# and we TELEPORT the target to a grid of ranges so it is GUARANTEED in the level
# FOV at every range. capture_flight_frames.py rides along (gt boxes + gt_range in
# the filename). Two aspects into SEPARATE dirs so we can compare:
#   approach (yaw 180, nose-toward camera)  vs  receding (yaw 0, tail-on).
# The ADR claims the detector "sees receding fine, approach ~0" -- this tests it
# head-on at matched range. Score offline: resolution_probe.py <dir> (fixed
# box_hits_gt). Hundreds of frames/bin -> replaces the n=5 wall claim + doubles as
# the R5/R6 hardware-capture spec. gt is SCORING-only (honesty boundary).
#
# One sim, idle load, sim-paced boot grep (ADR-0004). Kill logic in-file (self-kill
# rule). Usage: bash scripts/seeker/setpose_recall_sweep.sh
set -uo pipefail
cd "$(dirname "$0")/../.."
REPO_ROOT="$PWD"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PY="$REPO_ROOT/.venv-seeker/bin/python"
WORLD=quad_enemy
TARGET_MODEL=fpv_quad_enemy
OUT_ROOT="${SETPOSE_OUT:-$REPO_ROOT/scripts/seeker/data/setpose_sweep}"
LOG_DIR="$REPO_ROOT/logs/setpose_sweep"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOG_DIR/sim_${STAMP}.log"
mkdir -p "$LOG_DIR" "$OUT_ROOT"
READY_STR="Startup script returned successfully"; READY_TIMEOUT_S=150; SETTLE_S=6
# camera world position (probe): model(0,0,-0.013)+camera_link(0.12,0.03,0.242)
CAM_X=0.12; CAM_Y=0.03; CAM_Z=0.229
RANGES=(8 12 16 20 25)
HOLD_S=7   # per placement: ~7 s sim -> ~100 frames @30fps, --every 2 -> ~50 saved
source "$REPO_ROOT/scripts/sim_gpu_render.sh" 2>/dev/null || true

teardown_sim() { bash "$REPO_ROOT/scripts/sim_kill.sh" 2>/dev/null; pkill -f "capture_flight_frames.py" 2>/dev/null; sleep 3; }

set_target() { # x y z qz qw
    local x=$1 y=$2 z=$3 qz=$4 qw=$5
    for a in 1 2 3; do
        gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
            --timeout 2000 --req "name: \"$TARGET_MODEL\" position { x: $x y: $y z: $z } orientation { x: 0 y: 0 z: $qz w: $qw }" \
            >/dev/null 2>&1 && return 0
        sleep 1
    done
    return 1
}

echo "[setpose] boot $WORLD ..."
teardown_sim
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" > "$SIM_LOG" 2>&1 &
ready=0; for ((i=0;i<READY_TIMEOUT_S;i++)); do grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null && { ready=1; break; }; sleep 1; done
[[ "$ready" -ne 1 ]] && { echo "[setpose] FAIL sim-ready (see $SIM_LOG)"; teardown_sim; exit 1; }
sleep "$SETTLE_S"

# aspect | qz | qw  (yaw about world +z; approach=180deg faces -x toward camera)
ASPECTS=("approach|1|0" "receding|0|1")
for aspec in "${ASPECTS[@]}"; do
    IFS='|' read -r aspect qz qw <<< "$aspec"
    OUT="$OUT_ROOT/$aspect"; mkdir -p "$OUT"
    REC_LOG="$LOG_DIR/rec_${aspect}_${STAMP}.log"
    echo "[setpose] ===== aspect=$aspect -> $OUT ====="
    INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
        "$VENV_PY" "$REPO_ROOT/scripts/seeker/capture_flight_frames.py" \
        --out "$OUT" --run-tag "$aspect" --every 2 --extent-m 0.35 --max-frames 4000 > "$REC_LOG" 2>&1 &
    REC_PID=$!; sleep 2
    kill -0 "$REC_PID" 2>/dev/null || { echo "[setpose] FAIL recorder $aspect (see $REC_LOG)"; teardown_sim; exit 1; }
    for R in "${RANGES[@]}"; do
        tx=$(python3 -c "print($CAM_X + $R)")
        if set_target "$tx" "$CAM_Y" "$CAM_Z" "$qz" "$qw"; then
            echo "[setpose]   $aspect r=$R m -> target ($tx,$CAM_Y,$CAM_Z)"
        else
            echo "[setpose]   WARN set_pose failed $aspect r=$R"
        fi
        sleep "$HOLD_S"
    done
    kill -TERM "$REC_PID" 2>/dev/null; for ((i=0;i<15;i++)); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done; kill -KILL "$REC_PID" 2>/dev/null
    echo "[setpose]   $aspect frames: $(ls "$OUT/images" 2>/dev/null | wc -l)"
done
teardown_sim
echo "[setpose] DONE. Score: for a in approach receding; do echo == \$a ==; $VENV_PY scripts/seeker/resolution_probe.py $OUT_ROOT/\$a; done"
