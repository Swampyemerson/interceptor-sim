#!/usr/bin/env bash
# --coded-dash validation arm (real-build P0.2, ADR-0076 add #18): the FLIGHT
# architecture -- OPEN-LOOP coded dash -> CAMERA-ONLY terminal, NO ground-sensor
# cue / handoff / fusion. Uses mc_batch --mode m4 (drops --fpv --handoff, spawns
# NO cue mock) and forwards --coded-dash. Same quad geometry as quad_seeker_arm.sh
# (orient ON, weave-mirror) so the RETROACTIVE r2l test is directly comparable to
# the cue-guided numbers -- if r2l recovers here, add #17's root cause (cue/fused-
# dash) is confirmed and the flight architecture is validated.
#
# Usage: scripts/coded_dash_arm.sh WEIGHTS_ONNX MASTER_SEED PATH OUT_CSV [--dry-run|--go]
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$PWD"
WEIGHTS="${1:?weights onnx path}"
SEED="${2:?master-seed}"
PATHMODE="${3:?path: line|weave|jink}"
OUT="${4:?out csv}"
MODE="${5:---dry-run}"
[ -f "$WEIGHTS" ] || { echo "MISSING weights: $WEIGHTS"; exit 2; }

# World/target overridable for the billboard clean-perception baseline (P0.2):
#   QUAD_ARM_WORLD=apriltag QUAD_ARM_TARGET=<model> QUAD_ARM_SEEKER=apriltag ...
export MC_WORLD="${QUAD_ARM_WORLD:-quad_enemy}"
export MC_TARGET_MODEL="${QUAD_ARM_TARGET:-fpv_quad_enemy}"
export MC_SEEKER="${QUAD_ARM_SEEKER:-markerless}"
export MC_VENV_PYTHON="$REPO/.venv-seeker/bin/python"
export MARKERLESS_NN_WEIGHTS="$WEIGHTS"
# quad banks (orient to velocity, nose +X -> offset 0); fair weave (both cross NEAR)
export INTERCEPTOR_ORIENT_TO_VEL="${QUAD_ARM_ORIENT:-1}"  # 0 for the AprilTag control (a directional tag goes edge-on/invisible under orient-to-vel)
export INTERCEPTOR_ORIENT_YAW_OFFSET_DEG=0
export INTERCEPTOR_WEAVE_MIRROR=1
# NO cue: mode m4 spawns no cue mock; --coded-dash reads no cue. --dash-unclamp
# lets the fast dash reach speed; QUAD_ARM_EXTRA appends (e.g. --track for phantom).
EXTRA_ARGS_STR="--fpv --coded-dash --dash-speed 16 --dash-unclamp ${QUAD_ARM_EXTRA:-}"

DRYFLAG=""
[ "$MODE" = "--dry-run" ] && DRYFLAG="--dry-run"
echo "[coded-dash-arm] weights=$WEIGHTS seed=$SEED path=$PATHMODE out=$OUT mode=$MODE"
exec scripts/mc_batch.sh \
    --mode m4 --laws pronav --path "$PATHMODE" --speeds "${QUAD_ARM_SPEED:-12.0}" --n "${QUAD_ARM_N:-16}" --directions both \
    --y0-mag 14.0 --x0 6.5 --master-seed "$SEED" \
    --extra-args "$EXTRA_ARGS_STR" \
    --out "$OUT" $DRYFLAG
