#!/usr/bin/env bash
# T17-v2 orchestration: the WHOLE arc (capture -> label/build -> train ->
# export -> eval -> gate) in ONE script that BLOCKS internally, so a caller
# invokes it exactly once and waits for it to exit -- no repeated
# resume-wait polling of a live sim/training process from the calling
# agent's own loop (that trap cost a prior T17 worker real time).
#
# STEPS (each one is a hard gate -- if it fails, later steps are skipped
# and the script exits non-zero with a clear reason):
#   1. stale-process cleanup (scripts/sim_kill.sh -- a FILE, never inline
#      pkill; inline pkill from a shell whose own argv contains this
#      script's name would self-kill, CLAUDE.md batch-hygiene rule).
#   2. PX4 world symlink (worlds/stereo_intercept.sdf, same as
#      run_multirange_capture.sh).
#   3. boot headless PX4 SITL + Gazebo (GALLIUM_DRIVER=d3d12 -- ADR-0046
#      addendum, stable RTF), ONE sim at a time, idle-load only.
#   4. scripts/multirange_capture.py (.venv-seeker) -- the T17-v2 richer
#      multi-range + jitter + negatives capture (7 standoffs, wedge-sized
#      Y-grid, X/Z jitter, integrated negative capture -- see that script's
#      module docstring "T17-v2 EXTENSION" section).
#   5. teardown (scripts/sim_kill.sh) -- ALWAYS runs, even if step 4 failed.
#   6. scripts/seeker/build_ground_dataset_v2.py (.venv-seeker-train, has
#      both cv2 and torch/ultralytics) -- held-out-BY-STANDOFF split.
#   7. scripts/seeker/train_drone_finetune.py --go (.venv-seeker-train,
#      CPU-only torch, ~44 s/epoch per ADR-0049 -- runs as a FOREGROUND
#      child of THIS script, so this step blocks here, in-script, not in
#      the calling agent's own loop) -- exports ground_v2.pt + .onnx
#      automatically on completion.
#   8. scripts/check_t17v2.sh (offline) -- held-out-range recall/precision,
#      bias@50m fixed check, box-size monotonicity check, PRE-REGISTERED
#      thresholds (see that script's header) -- the pass/fail gate.
#
# Usage: scripts/run_t17v2.sh [epochs]
#   epochs defaults to 40 (ADR-0049: ground-detector metrics saturate
#   ~20 epochs on small sets; 2x margin, CPU-only budget).
# Exit code: check_t17v2.sh's exit code (0 = PASS), or an earlier non-zero
# code if capture/build/train itself failed before the gate could run.
set -uo pipefail
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_SEEKER_PYTHON="$REPO_ROOT/.venv-seeker/bin/python"
VENV_TRAIN_PYTHON="$REPO_ROOT/.venv-seeker-train/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/run_t17v2_sim_${TIMESTAMP}.log"
CAPTURE_DIR="$LOGS_DIR/rig_captures/multirange_v2_${TIMESTAMP}"
DATASET_DIR="$REPO_ROOT/scripts/seeker/data/ground_dataset_v2"
EPOCHS="${1:-40}"
IMGSZ=960
BATCH=16
RUN_NAME="ground_v2"
OUT_NAME="ground_v2"

WORLD_NAME="stereo_intercept"
TARGET_MODEL="fpv_target_markerless"
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/${WORLD_NAME}.sdf"
REPO_WORLD="$REPO_ROOT/worlds/${WORLD_NAME}.sdf"
READY_TIMEOUT_S=180
READY_STRING="Startup script returned successfully"
POST_READY_SETTLE_S=5

mkdir -p "$LOGS_DIR" "$CAPTURE_DIR"

section() { echo; echo "############################################################"; echo "# $*"; echo "############################################################"; }
note() { echo "[run_t17v2] $*"; }

section "T17-v2 ORCHESTRATION START ${TIMESTAMP}"
note "repo_root=$REPO_ROOT"
note "capture_dir=$CAPTURE_DIR"
note "dataset_dir=$DATASET_DIR"
note "epochs=$EPOCHS imgsz=$IMGSZ batch=$BATCH run_name=$RUN_NAME out_name=$OUT_NAME"

# ---------------------------------------------------------------------------
section "STEP 1: stale-process cleanup"
# ---------------------------------------------------------------------------
"$REPO_ROOT/scripts/sim_kill.sh"

# ---------------------------------------------------------------------------
section "STEP 2: PX4 world symlink"
# ---------------------------------------------------------------------------
if [[ ! -d "$PX4_DIR" ]]; then
    note "FAIL: PX4-Autopilot not found at $PX4_DIR"; exit 1
fi
if [[ ! -x "$VENV_SEEKER_PYTHON" ]]; then
    note "FAIL: seeker venv python not found at $VENV_SEEKER_PYTHON"; exit 1
fi
if [[ ! -x "$VENV_TRAIN_PYTHON" ]]; then
    note "FAIL: seeker-train venv python not found at $VENV_TRAIN_PYTHON"; exit 1
fi
if [[ ! -f "$REPO_WORLD" ]]; then
    note "FAIL: $REPO_WORLD not found"; exit 1
fi
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    note "Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi

# ---------------------------------------------------------------------------
section "STEP 3: boot headless PX4 SITL + Gazebo (world=$WORLD_NAME, GALLIUM_DRIVER=d3d12)"
# ---------------------------------------------------------------------------
note "sim output -> $SIM_LOG"
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD_NAME GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 GALLIUM_DRIVER=d3d12 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
note "sim launched (pid/pgid $SIM_PID)"

ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        note "FAIL: sim process exited early. See $SIM_LOG"
        "$REPO_ROOT/scripts/sim_kill.sh"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
if [[ "$ready" -ne 1 ]]; then
    note "FAIL: did not see ready string within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    "$REPO_ROOT/scripts/sim_kill.sh"
    exit 1
fi
note "sim booted. Settling ${POST_READY_SETTLE_S}s..."
sleep "$POST_READY_SETTLE_S"

# ---------------------------------------------------------------------------
section "STEP 4: scripts/multirange_capture.py (T17-v2 richer capture)"
# ---------------------------------------------------------------------------
INTERCEPTOR_WORLD_NAME="$WORLD_NAME" INTERCEPTOR_TARGET_MODEL="$TARGET_MODEL" \
    "$VENV_SEEKER_PYTHON" "$REPO_ROOT/scripts/multirange_capture.py" --out "$CAPTURE_DIR"
CAPTURE_EXIT=$?
note "capture exit code: $CAPTURE_EXIT"

# ---------------------------------------------------------------------------
section "STEP 5: teardown (scripts/sim_kill.sh)"
# ---------------------------------------------------------------------------
"$REPO_ROOT/scripts/sim_kill.sh"

if [[ "$CAPTURE_EXIT" -ne 0 ]]; then
    note "FAIL: multirange_capture.py exited $CAPTURE_EXIT -- aborting before build/train/gate."
    section "T17-v2 ORCHESTRATION RESULT: FAIL (capture step)"
    exit 1
fi

# ---------------------------------------------------------------------------
section "STEP 6: scripts/seeker/build_ground_dataset_v2.py (held-out BY STANDOFF)"
# ---------------------------------------------------------------------------
"$VENV_TRAIN_PYTHON" "$REPO_ROOT/scripts/seeker/build_ground_dataset_v2.py" \
    --capture-root "$CAPTURE_DIR" --held-out-standoffs 70 130 --out "$DATASET_DIR"
BUILD_EXIT=$?
note "build exit code: $BUILD_EXIT"
if [[ "$BUILD_EXIT" -ne 0 ]]; then
    note "FAIL: build_ground_dataset_v2.py exited $BUILD_EXIT -- aborting before train/gate."
    section "T17-v2 ORCHESTRATION RESULT: FAIL (dataset build step)"
    exit 1
fi

# ---------------------------------------------------------------------------
section "STEP 7: scripts/seeker/train_drone_finetune.py --go (FOREGROUND, blocks here)"
# ---------------------------------------------------------------------------
note "training ground_v2: epochs=$EPOCHS imgsz=$IMGSZ batch=$BATCH (CPU-only .venv-seeker-train, "
note "~44 s/epoch per ADR-0049 -- this WILL take a while; that is expected, not stuck)"
TRAIN_START_EPOCH_S=$(date +%s)
OMP_NUM_THREADS=8 "$VENV_TRAIN_PYTHON" "$REPO_ROOT/scripts/seeker/train_drone_finetune.py" \
    --go --data "$DATASET_DIR/data.yaml" --base yolo11n.pt \
    --imgsz "$IMGSZ" --epochs "$EPOCHS" --batch "$BATCH" \
    --run-name "$RUN_NAME" --out-name "$OUT_NAME"
TRAIN_EXIT=$?
TRAIN_END_EPOCH_S=$(date +%s)
note "train exit code: $TRAIN_EXIT (wall time: $((TRAIN_END_EPOCH_S - TRAIN_START_EPOCH_S)) s)"

WEIGHTS_PT="$REPO_ROOT/scripts/seeker/weights/${OUT_NAME}.pt"
WEIGHTS_ONNX="$REPO_ROOT/scripts/seeker/weights/${OUT_NAME}.onnx"
if [[ "$TRAIN_EXIT" -ne 0 || ! -f "$WEIGHTS_PT" ]]; then
    note "FAIL: train_drone_finetune.py exited $TRAIN_EXIT or $WEIGHTS_PT missing -- aborting before gate."
    section "T17-v2 ORCHESTRATION RESULT: FAIL (training step)"
    exit 1
fi
if [[ ! -f "$WEIGHTS_ONNX" ]]; then
    note "WARNING: $WEIGHTS_ONNX not found after training (export may have failed) -- continuing to gate, "
    note "which only needs the .pt; report this in the final summary."
fi
note "weights: $WEIGHTS_PT"
note "onnx:    $WEIGHTS_ONNX"

# ---------------------------------------------------------------------------
section "STEP 8: scripts/check_t17v2.sh (offline gate)"
# ---------------------------------------------------------------------------
"$REPO_ROOT/scripts/check_t17v2.sh" "$CAPTURE_DIR"
GATE_EXIT=$?

section "T17-v2 ORCHESTRATION COMPLETE"
note "capture_dir=$CAPTURE_DIR"
note "dataset_dir=$DATASET_DIR"
note "weights_pt=$WEIGHTS_PT"
note "weights_onnx=$WEIGHTS_ONNX"
note "sim_log=$SIM_LOG"
note "gate_exit=$GATE_EXIT"
if [[ "$GATE_EXIT" -eq 0 ]]; then
    note "FINAL VERDICT: PASS -- T17-v2 gate passed, see scripts/check_t17v2.sh output above for details."
else
    note "FINAL VERDICT: FAIL -- T17-v2 gate did not pass (see scripts/check_t17v2.sh output above for which check(s) missed). Reported honestly, not retried."
fi
exit "$GATE_EXIT"
