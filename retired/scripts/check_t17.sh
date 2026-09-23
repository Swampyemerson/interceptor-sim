#!/usr/bin/env bash
# T17 scripted gate (ADR-0047: single-class ground drone detector,
# acquisition-only, no interceptor class, no type discrimination).
# OFFLINE -- no sim boot. Runs the trained detector (scripts/seeker/weights/
# ground_v1.pt) against its held-out val split
# (scripts/seeker/data/ground_dataset_v1) and asserts PRE-REGISTERED
# thresholds:
#   - NEAR-band recall    >= 0.8
#   - NEAR-band precision >= 0.9
#   - negative clean rate >= 0.90 (fraction of val negative frames with
#     ZERO predicted boxes)
# FAR-band recall is REPORTED, not gated (see the eval report / T17 dataset
# README for why this sweep's achieved range span is narrow -- ~160-163 m --
# so "far" here is a proxy for edge-of-wedge/off-boresight difficulty more
# than a true long-range acquisition test).
#
# Usage: scripts/check_t17.sh
# Exit code: 0 = PASS, non-zero = FAIL.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_TRAIN_PYTHON="$REPO_ROOT/.venv-seeker-train/bin/python"
WEIGHTS="$REPO_ROOT/scripts/seeker/weights/ground_v1.pt"
DATASET="$REPO_ROOT/scripts/seeker/data/ground_dataset_v1"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_JSON="$LOGS_DIR/check_t17_eval_${TIMESTAMP}.json"

NEAR_RECALL_MIN=0.8
NEAR_PRECISION_MIN=0.9
NEG_CLEAN_RATE_MIN=0.90

mkdir -p "$LOGS_DIR"
FAIL=0
note() { echo "[check_t17] $*"; }

echo "=== [check_t17] STEP (i): artifacts exist ==="
for f in "$WEIGHTS" "$DATASET/data.yaml" "$DATASET/split_meta.csv" "$VENV_TRAIN_PYTHON"; do
    if [[ ! -e "$f" ]]; then
        note "FAIL(i): missing $f"
        echo "FAIL"; exit 1
    fi
done
note "STEP (i) PASS: weights + dataset + venv present."

echo "=== [check_t17] STEP (ii): offline eval on held-out (val) split ==="
EVAL_OUT="$("$VENV_TRAIN_PYTHON" "$REPO_ROOT/scripts/seeker/eval_ground_detector.py" \
    --weights "$WEIGHTS" --dataset "$DATASET" --out-json "$OUT_JSON" 2>&1)"
EVAL_EXIT=$?
echo "$EVAL_OUT"

if [[ "$EVAL_EXIT" -ne 0 ]]; then
    note "FAIL(ii): eval_ground_detector.py exited $EVAL_EXIT"
    echo "FAIL"; exit 1
fi

SUMMARY_LINE="$(grep '^SUMMARY' <<< "$EVAL_OUT")"
if [[ -z "$SUMMARY_LINE" ]]; then
    note "FAIL(ii): no SUMMARY line in eval output"
    echo "FAIL"; exit 1
fi
note "STEP (ii) PASS: eval ran. $SUMMARY_LINE"

echo "=== [check_t17] STEP (iii): pre-registered thresholds ==="
NEAR_RECALL="$(grep -oP 'near_recall=\K[0-9.]+' <<< "$SUMMARY_LINE")"
NEAR_PRECISION="$(grep -oP 'near_precision=\K[0-9.]+' <<< "$SUMMARY_LINE")"
FAR_RECALL="$(grep -oP 'far_recall=\K[0-9.]+' <<< "$SUMMARY_LINE")"
NEG_CLEAN_RATE="$(grep -oP 'neg_clean_rate=\K[0-9.]+' <<< "$SUMMARY_LINE")"

if [[ -z "$NEAR_RECALL" || -z "$NEAR_PRECISION" || -z "$NEG_CLEAN_RATE" ]]; then
    note "FAIL(iii): could not parse thresholds out of SUMMARY line"
    echo "FAIL"; exit 1
fi

if awk "BEGIN{exit !($NEAR_RECALL >= $NEAR_RECALL_MIN)}"; then
    note "PASS: near-band recall $NEAR_RECALL >= $NEAR_RECALL_MIN"
else
    note "FAIL: near-band recall $NEAR_RECALL < $NEAR_RECALL_MIN"; FAIL=1
fi

if awk "BEGIN{exit !($NEAR_PRECISION >= $NEAR_PRECISION_MIN)}"; then
    note "PASS: near-band precision $NEAR_PRECISION >= $NEAR_PRECISION_MIN"
else
    note "FAIL: near-band precision $NEAR_PRECISION < $NEAR_PRECISION_MIN"; FAIL=1
fi

if awk "BEGIN{exit !($NEG_CLEAN_RATE >= $NEG_CLEAN_RATE_MIN)}"; then
    note "PASS: negative clean rate $NEG_CLEAN_RATE >= $NEG_CLEAN_RATE_MIN"
else
    note "FAIL: negative clean rate $NEG_CLEAN_RATE < $NEG_CLEAN_RATE_MIN"; FAIL=1
fi

note "REPORT (not gated): far-band recall = $FAR_RECALL"
note "eval json: $OUT_JSON"

if [[ "$FAIL" -eq 0 ]]; then
    note "PASS"
    echo "PASS"; exit 0
else
    note "FAIL: one or more thresholds not met (see above)"
    echo "FAIL"; exit 1
fi
