#!/usr/bin/env bash
# Quad-target seeker validation arm (ADR-0076 r2l) — NN-only, canonical quad
# geometry, orient-to-velocity + weave-mirror ON. NOT the billboard deployment
# arm (that is scripts/mc_deployment_arm.sh); this validates a quad seeker
# (v2 / hardened / crop) on fpv_quad_enemy. Mirrors the weave-mirror batch env
# (mc_batch_run_20260712T191213Z) exactly, swapping only WEIGHTS + master-seed.
#
# Usage:
#   scripts/quad_seeker_arm.sh WEIGHTS_ONNX MASTER_SEED PATH OUT_CSV [--dry-run|--go]
# e.g.
#   scripts/quad_seeker_arm.sh scripts/seeker/weights/drone_finetuned_quad_hardened.onnx \
#       123 weave logs/mc_quad_hardened_s123.csv --go
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$PWD"

WEIGHTS="${1:?weights onnx path}"
SEED="${2:?master-seed}"
PATHMODE="${3:?path: line|weave|jink}"
OUT="${4:?out csv}"
MODE="${5:---dry-run}"

[ -f "$WEIGHTS" ] || { echo "MISSING weights: $WEIGHTS"; exit 2; }

# --- markerless quad flight env (inherited by the m4 subprocess) ---
export MC_WORLD="quad_enemy"
export MC_TARGET_MODEL="fpv_quad_enemy"
export MC_SEEKER="markerless"
export MC_VENV_PYTHON="$REPO/.venv-seeker/bin/python"
export MARKERLESS_NN_WEIGHTS="$WEIGHTS"
# realistic degraded cue (same as the adopted arm / weave-mirror batch)
export S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"
# quad must bank (orient to velocity, nose +X baked -> offset 0); fair weave (both dirs cross NEAR)
export INTERCEPTOR_ORIENT_TO_VEL=1
export INTERCEPTOR_ORIENT_YAW_OFFSET_DEG=0
export INTERCEPTOR_WEAVE_MIRROR=1

# NN-only (drop --track for the maneuvering quad, ADR-0076 add #2)
# QUAD_ARM_EXTRA appends extra m4 flags (e.g. --cue-vel-hold) for an A/B arm.
EXTRA_ARGS_STR="--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp --fuse-midcourse --handoff-cue-gate 8 ${QUAD_ARM_EXTRA:-}"

DRYFLAG=""
[ "$MODE" = "--dry-run" ] && DRYFLAG="--dry-run"

echo "[quad-arm] weights=$WEIGHTS seed=$SEED path=$PATHMODE out=$OUT mode=$MODE"
exec scripts/mc_batch.sh \
    --laws pronav --path "$PATHMODE" --speeds 12.0 --n 16 --directions both \
    --y0-mag 14.0 --x0 6.5 --master-seed "$SEED" \
    --extra-args "$EXTRA_ARGS_STR" \
    --out "$OUT" $DRYFLAG
