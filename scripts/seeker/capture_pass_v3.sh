#!/usr/bin/env bash
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)
# Seeker v3 ride-along capture pass (docs/seeker_v3_dataset_plan.md Phase 2 +
# eval pool). NEW file (task #28); models on capture_pass.sh (fresh boot per
# flight + passive recorder) and mc_batch.sh (the markerless deployment-profile
# m4_intercept invocation + T21 12/15 m/s geometry). Does NOT edit the committed
# capture_pass.sh or any guidance/seeker .py.
#
# WHAT each flight does: boot a COMPLETELY FRESH markerless sim, pre-place the
# tag-less target at the arm's start, ride along with the PASSIVE recorder
# (capture_flight_frames.py, subscribe-only, gt-projected auto-labels) while a
# deployment-profile markerless engagement flies the T21 arm, then SIGTERM the
# recorder (writes its manifest) and tear down. One sim at a time, sequential.
#
# CAPTURE CONFIG (plan Phase 2, [A1]/[A2] resolved from the surviving T21
# stdout logs): the exact T21 12 m/s markerless deployment profile
#   --fpv --handoff --law pronav --seeker markerless --dash-speed 16
#   --early-handoff --cue-velocity --dash-unclamp --fuse-midcourse
# geometry x0=6.5, y0-mag=29.3 (l2r start_y=-29.3 vy=+v ; r2l start_y=+29.3
# vy=-v), weave/jink knobs at the T21 defaults (period 4 s, lat 3 m/s,
# jink-count 2, jink-min 40 deg), master-seed-42-family seeds for the eval
# pool, disjoint 1101-1114 for training. Seeker weights = the deployment v2
# ONNX (so flight behavior matches deployment).
# CUE-GATE, per plan Sec 3 ("Capture aid ONLY, never an eval config"):
#   TRAIN flights add --handoff-cue-gate 8 (legal, PRE-handoff cue only; keeps
#     the interceptor pointed near the true target -> denser in-frame positives
#     at genuine chase attitudes; ADR-0057 attempt-3 / [A2]).
#   EVAL/PHASE0 flights DO NOT gate the handoff -- they must fly the RAW
#     failing regime so v2's phantom-false-handoff behavior (the thing the
#     eval measures, ADR-0056/0057) is present in the frames. Corrected after
#     a Phase-0 live check found the gate suppresses the eval's own signal.
# Deliberately NO --track for either set -- the v2 phantom textures are RICHER
# without the detect-then-track suppressor, which is what the hard-negative
# mining (plan Sec 4) wants to harvest and what the eval must expose.
#
# DEVIATION FROM PLAN (logged): recorder --max-frames is 2000, not the plan's
# 300. The 300 cap risks filling with pre-takeoff/CUE_WAIT off-frame negatives
# (target starts at bearing ~77deg, outside the 50deg half-FOV) BEFORE the
# DASH/ENGAGE phase where the real positives appear -- starving positives. A
# full-flight capture (flight-end SIGTERM) + the builder's 45% negative cap is
# what v2 did (36 pos / 694 neg across 3 flights) and is required to approach
# the plan's own 500-900 flight-positive train target. Negatives are curated
# downstream by build_onboard_dataset_v3.py.
#
# Usage:  scripts/seeker/capture_pass_v3.sh <phase0|train|eval> [tag1,tag2,...]
#   phase0 -> evalv3_01, evalv3_02 (the 2 forensics flights) into the eval dir
#   eval   -> all 10 eval-pool flights   into scripts/seeker/data/v3_flight_eval
#   train  -> all 14 training flights    into scripts/seeker/data/v3_flight_train
#   optional 2nd arg: comma list of run_tags to run just that subset.
set -u

SET="${1:-}"
SUBSET="${2:-}"
if [[ "$SET" != "phase0" && "$SET" != "train" && "$SET" != "eval" ]]; then
    echo "usage: $0 <phase0|train|eval> [tag1,tag2,...]" >&2
    exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PY="$REPO_ROOT/.venv-seeker/bin/python"     # combined venv (mavsdk+onnx+gz+cv2)
WORLD=markerless
TARGET_MODEL=fpv_target_markerless
V2_WEIGHTS="$REPO_ROOT/scripts/seeker/weights/drone_finetuned_v2.onnx"
# Standard M5/A-B degraded-cue realism (mc-batch skill; the T21 cue).
CUE_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"

if [[ "$SET" == "train" ]]; then
    OUT_DIR="$REPO_ROOT/scripts/seeker/data/v3_flight_train"
    CUE_GATE=(--handoff-cue-gate 8)   # capture aid: TRAIN only (plan Sec 3)
else
    OUT_DIR="$REPO_ROOT/scripts/seeker/data/v3_flight_eval"
    CUE_GATE=()                        # eval/phase0: raw failing regime (no gate)
fi
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="$REPO_ROOT/logs/seeker_capture_v3"
mkdir -p "$LOG_DIR" "$OUT_DIR"

READY_STR="Startup script returned successfully"
READY_TIMEOUT_S=150
SETTLE_S=5
FLIGHT_TIMEOUT_S=360

# flight spec: run_tag | path | speed | direction | cue_seed | path_seed
# Training flights (plan Phase 2 table): seeds 1101-1114, clean +/-29.3 start,
# disjoint from the master-seed-42 eval family.
TRAIN_FLIGHTS=(
    "capv3_01|weave|12|l2r|1101|1101"
    "capv3_02|weave|12|r2l|1102|1102"
    "capv3_03|weave|12|l2r|1103|1103"
    "capv3_04|weave|12|r2l|1104|1104"
    "capv3_05|jink|12|l2r|1105|1105"
    "capv3_06|jink|12|r2l|1106|1106"
    "capv3_07|jink|12|l2r|1107|1107"
    "capv3_08|jink|12|r2l|1108|1108"
    "capv3_09|weave|9|l2r|1109|1109"
    "capv3_10|weave|9|r2l|1110|1110"
    "capv3_11|line|9|l2r|1111|1111"
    "capv3_12|line|6|r2l|1112|1112"
    "capv3_13|weave|15|l2r|1113|1113"
    "capv3_14|weave|15|r2l|1114|1114"
    # [A6] top-up (plan Sec 10): per-flight positive yield came in ~12/flight
    # (matching the v2 precedent, below the 30/flight bar), so add 2 more
    # cue-gated weave-12 flights (the failing regime = the most phantom-draw-
    # prone, highest-density geometry). NEW train seeds 1115/1116, disjoint
    # from the 1101-1114 train series AND the master-seed-42 eval family.
    "capv3_15|weave|12|l2r|1115|1115"
    "capv3_16|weave|12|r2l|1116|1116"
)
# Eval-pool flights (plan Sec 8): EXACT master-seed-42 T21 seeds (from
# `mc_batch.sh --dry-run`), flown once, scored offline by BOTH v2 and v3.
# Flight index i has the same (cue_seed,path_seed) across arms: 0->26226/857665
# (l2r, jitter start_y -28.882), 1->256788/778181 (r2l, +30.025),
# 2->772247/832313 (l2r, -30.130), 3->776647/447246 (r2l, +28.107).
EVAL_FLIGHTS=(
    "evalv3_01|weave|12|l2r|26226|857665"
    "evalv3_02|weave|12|r2l|256788|778181"
    "evalv3_03|weave|12|l2r|772247|832313"
    "evalv3_04|weave|12|r2l|776647|447246"
    "evalv3_05|jink|12|l2r|26226|857665"
    "evalv3_06|jink|12|r2l|256788|778181"
    "evalv3_07|weave|9|l2r|26226|857665"
    "evalv3_08|weave|9|r2l|256788|778181"
    "evalv3_09|line|9|l2r|26226|857665"
    "evalv3_10|line|6|r2l|256788|778181"
)

if [[ "$SET" == "train" ]]; then
    FLIGHTS=("${TRAIN_FLIGHTS[@]}")
elif [[ "$SET" == "phase0" ]]; then
    FLIGHTS=("${EVAL_FLIGHTS[0]}" "${EVAL_FLIGHTS[1]}")
else
    FLIGHTS=("${EVAL_FLIGHTS[@]}")
fi

# optional subset filter
if [[ -n "$SUBSET" ]]; then
    filtered=()
    IFS=',' read -r -a want <<< "$SUBSET"
    for spec in "${FLIGHTS[@]}"; do
        tag="${spec%%|*}"
        for w in "${want[@]}"; do [[ "$tag" == "$w" ]] && filtered+=("$spec"); done
    done
    FLIGHTS=("${filtered[@]}")
fi

# eval flights carry their exact jittered start_y (T21 trajectories); training
# flights use clean +/-29.3. Direction sets the sign of start_y and vy.
start_y_for() {   # $1=set $2=direction $3=run_tag  -> echoes start_y
    local s="$1" d="$2" tag="$3"
    if [[ "$s" == "train" ]]; then
        [[ "$d" == "l2r" ]] && echo "-29.3" || echo "29.3"; return
    fi
    case "$tag" in
        evalv3_01|evalv3_05|evalv3_07|evalv3_09) echo "-28.882";;
        evalv3_02|evalv3_06|evalv3_08|evalv3_10) echo "30.025";;
        evalv3_03) echo "-30.130";;
        evalv3_04) echo "28.107";;
        *) [[ "$d" == "l2r" ]] && echo "-29.3" || echo "29.3";;
    esac
}

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

echo "[capv3] set=$SET out=$OUT_DIR flights=${#FLIGHTS[@]}"
overall_rc=0
for spec in "${FLIGHTS[@]}"; do
    IFS='|' read -r tag path speed direction cue_seed path_seed <<< "$spec"
    sy="$(start_y_for "$SET" "$direction" "$tag")"
    if [[ "$direction" == "l2r" ]]; then vy="$speed"; else vy="-$speed"; fi
    START="6.5,${sy},0.5"
    VEL="0,${vy}"

    SIM_LOG="$LOG_DIR/sim_${tag}_${STAMP}.log"
    RUN_LOG="$LOG_DIR/run_${tag}_${STAMP}.log"
    REC_LOG="$LOG_DIR/rec_${tag}_${STAMP}.log"

    echo "[capv3] ===== $tag (path=$path speed=$speed dir=$direction cue=$cue_seed pathseed=$path_seed start=$START vel=$VEL) ====="
    teardown_sim

    setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
        > "$SIM_LOG" 2>&1 &

    ready=0
    for ((i=0; i<READY_TIMEOUT_S; i++)); do
        if grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null; then ready=1; break; fi
        sleep 1
    done
    if [[ "$ready" -ne 1 ]]; then
        echo "[capv3] FAIL: sim not ready in ${READY_TIMEOUT_S}s for $tag (see $SIM_LOG)"
        overall_rc=1; teardown_sim; continue
    fi
    sleep "$SETTLE_S"

    placed=0
    for attempt in 1 2 3; do
        if gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
            --req "name: \"$TARGET_MODEL\" position { x: 6.5 y: $sy z: 0.5 }"; then
            placed=1; break
        fi
        sleep 1
    done
    if [[ "$placed" -ne 1 ]]; then
        echo "[capv3] FAIL: pre-place failed for $tag"
        overall_rc=1; teardown_sim; continue
    fi

    # passive recorder (subscribe-only; full-flight capture, flight-end SIGTERM)
    INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
        "$VENV_PY" "$REPO_ROOT/scripts/seeker/capture_flight_frames.py" \
        --out "$OUT_DIR" --run-tag "$tag" --every 3 --max-frames 2000 \
        > "$REC_LOG" 2>&1 &
    REC_PID=$!
    sleep 2
    if ! kill -0 "$REC_PID" 2>/dev/null; then
        echo "[capv3] FAIL: recorder died at start for $tag (see $REC_LOG)"
        overall_rc=1; teardown_sim; continue
    fi

    # deployment-profile markerless engagement on this arm (T21 config + cue-gate aid)
    env INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
        INTERCEPTOR_TARGET_PATH="$path" INTERCEPTOR_PATH_SEED="$path_seed" \
        INTERCEPTOR_WEAVE_PERIOD_S=4.0 INTERCEPTOR_WEAVE_LAT_SPEED=3.0 \
        INTERCEPTOR_JINK_COUNT=2 INTERCEPTOR_JINK_MIN_DEG=40.0 \
        MARKERLESS_NN_WEIGHTS="$V2_WEIGHTS" \
        S2_CUE_MOCK_EXTRA="$CUE_EXTRA" \
        timeout "$FLIGHT_TIMEOUT_S" "$VENV_PY" "$REPO_ROOT/scripts/m4_intercept.py" \
        --fpv --handoff --law pronav --seeker markerless \
        --target-start="$START" --target-vel="$VEL" --cue-seed "$cue_seed" \
        --dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --fuse-midcourse \
        "${CUE_GATE[@]}" \
        > "$RUN_LOG" 2>&1
    PY_EXIT=$?
    echo "[capv3] flight $tag exit=$PY_EXIT (log $RUN_LOG)"

    kill -TERM "$REC_PID" 2>/dev/null
    for ((i=0; i<15; i++)); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done
    kill -KILL "$REC_PID" 2>/dev/null
    grep -E "saved_pos|saved_neg|dropped" "$REC_LOG" | tail -2 || true

    teardown_sim
done

echo "[capv3] DONE set=$SET. Data in $OUT_DIR"
echo -n "[capv3] images so far: "; ls "$OUT_DIR/images" 2>/dev/null | wc -l
echo "CAPV3_PASS_COMPLETE set=$SET rc=$overall_rc"
exit "$overall_rc"
