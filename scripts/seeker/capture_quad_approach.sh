#!/usr/bin/env bash
# Capture APPROACH frames of the 3D quad target from a coded-dash flight, with
# ground-truth boxes (capture_flight_frames.py rides along). For the offline
# resolution-vs-aspect probe (ADR-0076 add #18i / Fable round-2): frames are
# saved as <tag>_<seq>_r<gt_range>_pos.png + a gt YOLO label, so the probe can
# score detectors on the 8-25 m APPROACHING frames. quad_enemy world + quad_v2.
# Kill logic lives in this FILE (invoked by path) so no self-kill (CLAUDE.md).
set -uo pipefail
cd "$(dirname "$0")/../.."
REPO_ROOT="$PWD"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PY="$REPO_ROOT/.venv-seeker/bin/python"
WORLD=quad_enemy
TARGET_MODEL=fpv_quad_enemy
WEIGHTS="$REPO_ROOT/scripts/seeker/weights/drone_finetuned_quad_v2.onnx"
OUT_DIR="${QUAD_APPROACH_OUT:-$REPO_ROOT/scripts/seeker/data/quad_approach}"
LOG_DIR="$REPO_ROOT/logs/quad_approach"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$LOG_DIR" "$OUT_DIR"
READY_STR="Startup script returned successfully"; READY_TIMEOUT_S=150; SETTLE_S=6; FLIGHT_TIMEOUT_S=200
source "$REPO_ROOT/scripts/sim_gpu_render.sh" 2>/dev/null || true

teardown_sim() { bash "$REPO_ROOT/scripts/sim_kill.sh" 2>/dev/null; pkill -f "capture_flight_frames.py" 2>/dev/null; sleep 3; }

# tag | target-start (x,y,z) | target-vel | crossing-bias-sign
FLIGHTS=(
    "l2r|6.500,-15.343,0.5|0.000,9.000"
    "r2l|6.500,14.807,0.5|0.000,-9.000"
)
overall_rc=0
for spec in "${FLIGHTS[@]}"; do
    IFS='|' read -r tag tstart tvel <<< "$spec"
    SIM_LOG="$LOG_DIR/sim_${tag}_${STAMP}.log"; RUN_LOG="$LOG_DIR/run_${tag}_${STAMP}.log"; REC_LOG="$LOG_DIR/rec_${tag}_${STAMP}.log"
    echo "[cap-quad] ===== $tag ====="
    teardown_sim
    setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" > "$SIM_LOG" 2>&1 &
    ready=0; for ((i=0;i<READY_TIMEOUT_S;i++)); do grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null && { ready=1; break; }; sleep 1; done
    [[ "$ready" -ne 1 ]] && { echo "[cap-quad] FAIL sim-ready $tag"; overall_rc=1; teardown_sim; continue; }
    sleep "$SETTLE_S"
    IFS=',' read -r tsx tsy tsz <<< "$tstart"
    placed=0; for a in 1 2 3; do gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req "name: \"$TARGET_MODEL\" position { x: $tsx y: $tsy z: $tsz }" && { placed=1; break; }; sleep 1; done
    [[ "$placed" -ne 1 ]] && { echo "[cap-quad] FAIL pre-place $tag"; overall_rc=1; teardown_sim; continue; }
    INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
        "$VENV_PY" "$REPO_ROOT/scripts/seeker/capture_flight_frames.py" --out "$OUT_DIR" --run-tag "${tag}_${STAMP}" --every 2 > "$REC_LOG" 2>&1 &
    REC_PID=$!; sleep 2
    kill -0 "$REC_PID" 2>/dev/null || { echo "[cap-quad] FAIL recorder $tag"; overall_rc=1; teardown_sim; continue; }
    INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL INTERCEPTOR_TARGET_PATH=line \
        INTERCEPTOR_ORIENT_TO_VEL=1 INTERCEPTOR_ORIENT_YAW_OFFSET_DEG=0 MARKERLESS_NN_WEIGHTS="$WEIGHTS" \
        timeout "$FLIGHT_TIMEOUT_S" "$VENV_PY" "$REPO_ROOT/scripts/m4_intercept.py" \
        --fpv --coded-dash --law pronav --seeker markerless \
        --target-start="$tstart" --target-vel="$tvel" --cue-seed 123 \
        --dash-speed 16 --dash-unclamp --dash-crossing-bias-deg 30 > "$RUN_LOG" 2>&1
    echo "[cap-quad] $tag flight exit=$? "
    kill -TERM "$REC_PID" 2>/dev/null; for ((i=0;i<15;i++)); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done; kill -KILL "$REC_PID" 2>/dev/null
    tail -n 2 "$REC_LOG" || true
    teardown_sim
done
echo "[cap-quad] DONE rc=$overall_rc  frames -> $OUT_DIR"
exit "$overall_rc"
