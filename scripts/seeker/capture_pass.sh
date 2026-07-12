#!/usr/bin/env bash
# Seeker v2 hard-negative capture pass (ADR-0040 addendum: primary v2 lever).
#
# Boots a COMPLETELY FRESH markerless sim per flight (the mc_batch.sh pattern),
# runs the PASSIVE frame recorder (capture_flight_frames.py) alongside a
# deployment-profile intercept flight, kills the recorder (SIGTERM -> manifest),
# tears down. The flight command reproduces the ADR-0038/0040 arm invocation
# EXCEPT the cue seeds: capture seeds are DELIBERATELY DISJOINT from the
# master-seed-42 A/B plan (26226, 256788, ...) so the v2 fine-tune never trains
# on frames from the exact trajectories it is later evaluated on.
#
# Not a gate. Dev data-collection: one sim at a time, idle-ish load, sim-paced
# (boot-complete grep ADR-0004; no wall-clock scheduling of sim behavior).
#
# Usage:  bash scripts/seeker/capture_pass.sh
set -u
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PY="$REPO_ROOT/.venv-seeker/bin/python"
WORLD=markerless
TARGET_MODEL=fpv_target_markerless
OUT_DIR="$REPO_ROOT/scripts/seeker/data/flight_capture"
TUNED_WEIGHTS="$REPO_ROOT/scripts/seeker/weights/drone_finetuned.onnx"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="$REPO_ROOT/logs/seeker_capture"
mkdir -p "$LOG_DIR" "$OUT_DIR"

READY_STR="Startup script returned successfully"
READY_TIMEOUT_S=150
SETTLE_S=6
FLIGHT_TIMEOUT_S=360

teardown_sim() {
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    pkill -f "s2_cue_mock.py" 2>/dev/null
    pkill -f "m4_target_mover.py" 2>/dev/null
    pkill -f "capture_flight_frames.py" 2>/dev/null
    sleep 3
}

# flight spec: run_tag | cue_seed | target-start | target-vel | path | weights(env or "-")
FLIGHTS=(
    "cap1|111|6.500,-28.900,0.5|0.000,9.000|line|$TUNED_WEIGHTS"
    "cap2|222|6.500,28.900,0.5|0.000,-9.000|line|$TUNED_WEIGHTS"
    "cap3|333|6.500,-28.900,0.5|0.000,9.000|weave|-"
)

overall_rc=0
for spec in "${FLIGHTS[@]}"; do
    IFS='|' read -r tag cue_seed tstart tvel path weights <<< "$spec"
    SIM_LOG="$LOG_DIR/sim_${tag}_${STAMP}.log"
    RUN_LOG="$LOG_DIR/run_${tag}_${STAMP}.log"
    REC_LOG="$LOG_DIR/rec_${tag}_${STAMP}.log"

    echo "[capture_pass] ===== flight $tag (seed=$cue_seed path=$path weights=$(basename "$weights")) ====="
    teardown_sim

    setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
        > "$SIM_LOG" 2>&1 &

    ready=0
    for ((i=0; i<READY_TIMEOUT_S; i++)); do
        if grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null; then ready=1; break; fi
        sleep 1
    done
    if [[ "$ready" -ne 1 ]]; then
        echo "[capture_pass] FAIL: sim not ready in ${READY_TIMEOUT_S}s for $tag (see $SIM_LOG)"
        overall_rc=1; teardown_sim; continue
    fi
    sleep "$SETTLE_S"

    IFS=',' read -r tsx tsy tsz <<< "$tstart"
    placed=0
    for attempt in 1 2 3; do
        if gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
            --req "name: \"$TARGET_MODEL\" position { x: $tsx y: $tsy z: $tsz }"; then
            placed=1; break
        fi
        sleep 1
    done
    if [[ "$placed" -ne 1 ]]; then
        echo "[capture_pass] FAIL: pre-place failed for $tag"
        overall_rc=1; teardown_sim; continue
    fi

    INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
        "$VENV_PY" "$REPO_ROOT/scripts/seeker/capture_flight_frames.py" \
        --out "$OUT_DIR" --run-tag "$tag" --every 3 \
        > "$REC_LOG" 2>&1 &
    REC_PID=$!
    sleep 2
    if ! kill -0 "$REC_PID" 2>/dev/null; then
        echo "[capture_pass] FAIL: recorder died at start for $tag (see $REC_LOG)"
        overall_rc=1; teardown_sim; continue
    fi

    WENV=()
    if [[ "$weights" != "-" ]]; then WENV=(MARKERLESS_NN_WEIGHTS="$weights"); fi
    env INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
        INTERCEPTOR_TARGET_PATH="$path" "${WENV[@]}" \
        timeout "$FLIGHT_TIMEOUT_S" "$VENV_PY" "$REPO_ROOT/scripts/m4_intercept.py" \
        --fpv --handoff --law pronav --seeker markerless \
        --target-start="$tstart" --target-vel="$tvel" --cue-seed "$cue_seed" \
        --dash-speed 16 --early-handoff --cue-velocity --dash-unclamp \
        > "$RUN_LOG" 2>&1
    PY_EXIT=$?
    echo "[capture_pass] flight $tag exit=$PY_EXIT (log $RUN_LOG)"

    kill -TERM "$REC_PID" 2>/dev/null
    for ((i=0; i<15; i++)); do
        kill -0 "$REC_PID" 2>/dev/null || break
        sleep 1
    done
    kill -KILL "$REC_PID" 2>/dev/null
    tail -n 3 "$REC_LOG" || true

    teardown_sim
done

echo "[capture_pass] done. Captured data in $OUT_DIR"
ls "$OUT_DIR/images" 2>/dev/null | wc -l
exit "$overall_rc"
