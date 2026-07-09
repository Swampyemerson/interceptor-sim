#!/usr/bin/env bash
# T17-v2 scripted gate (ADR-0053 fix: multi-range ground detector). OFFLINE
# -- no sim boot (the multirange capture + training that produce this gate's
# inputs are run separately by scripts/run_t17v2.sh; this script only reads
# their outputs). Asserts PRE-REGISTERED thresholds decided BEFORE training
# (see the task brief this script implements):
#
#   (i)   artifacts exist: ground_v2.pt/.onnx, ground_dataset_v2/{data.yaml,
#         split_meta.csv}, the two multirange_nn_detection.py run dirs
#         (ground_v1 vs ground_v2 head-to-head, same held-out capture).
#   (ii)  held-out-RANGE recall/precision -- eval_ground_detector.py against
#         ground_dataset_v2 (whose val split IS the held-out standoffs, see
#         build_ground_dataset_v2.py): near-band (lower held-out standoff)
#         and far-band (higher held-out standoff) recall >= 0.8, precision
#         >= 0.8 (looser than v1's 0.9 -- v2's val set is a genuinely harder,
#         never-trained-on range, not v1's near-duplicate interpolation).
#   (iii) bias @ 50 m FIXED: |ground_v2 bias| < 2.0 m (vs ADR-0053's
#         measured ground_v1 bias of -6.3 m at the same standoff) --
#         scripts/eval_t17v2_comparison.py's headline number.
#   (iv)  box-size head MONOTONIC now (ground_v2), vs ADR-0053's ground_v1
#         finding of a non-monotonic head (50 m boxes smaller than 90 m).
#
# Negative clean-rate is REPORTED, not gated (ADR-0049's disclosed flat-
# world negative-diversity limitation applies here too, and v2's negatives
# are a smaller, thinner set -- see build_ground_dataset_v2.py's README).
#
# Usage: scripts/check_t17v2.sh [capture_root]
#   capture_root defaults to the newest logs/rig_captures/multirange_v2_*
# Exit code: 0 = PASS, non-zero = FAIL (first failing step wins; ALL
# pre-registered checks still run and print before a FAIL exit, so a
# miss is reported honestly, not silently retried).
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_TRAIN_PYTHON="$REPO_ROOT/.venv-seeker-train/bin/python"
WEIGHTS_V2="$REPO_ROOT/scripts/seeker/weights/ground_v2.pt"
WEIGHTS_V1="$REPO_ROOT/scripts/seeker/weights/ground_v1.pt"
DATASET_V2="$REPO_ROOT/scripts/seeker/data/ground_dataset_v2"
LOGS_DIR="$REPO_ROOT/logs"
T17V2_DIR="$LOGS_DIR/t17v2"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"

CAPTURE_ROOT="${1:-}"
if [[ -z "$CAPTURE_ROOT" ]]; then
    CAPTURE_ROOT="$(ls -dt "$LOGS_DIR"/rig_captures/multirange_v2_* 2>/dev/null | head -1)"
fi

NEAR_RECALL_MIN=0.8
NEAR_PRECISION_MIN=0.8
FAR_RECALL_MIN=0.8
FAR_PRECISION_MIN=0.8
BIAS_50M_ABS_MAX=2.0   # pre-registered, vs ground_v1's measured -6.3 m (ADR-0053)

mkdir -p "$T17V2_DIR"
FAIL=0
note() { echo "[check_t17v2] $*"; }

echo "=== [check_t17v2] STEP (i): artifacts exist ==="
for f in "$WEIGHTS_V2" "$WEIGHTS_V1" "$DATASET_V2/data.yaml" "$DATASET_V2/split_meta.csv" "$VENV_TRAIN_PYTHON"; do
    if [[ ! -e "$f" ]]; then
        note "FAIL(i): missing $f"
        echo "FAIL"; exit 1
    fi
done
if [[ -z "$CAPTURE_ROOT" || ! -d "$CAPTURE_ROOT" ]]; then
    note "FAIL(i): no multirange_v2_* capture root found (pass one explicitly as \$1)"
    echo "FAIL"; exit 1
fi
note "STEP (i) PASS: weights (v1+v2) + dataset v2 + capture root ($CAPTURE_ROOT) present."

echo "=== [check_t17v2] STEP (ii): held-out-range recall/precision (eval_ground_detector.py on ground_dataset_v2) ==="
EVAL_JSON="$LOGS_DIR/check_t17v2_eval_${TIMESTAMP}.json"
EVAL_OUT="$("$VENV_TRAIN_PYTHON" "$REPO_ROOT/scripts/seeker/eval_ground_detector.py" \
    --weights "$WEIGHTS_V2" --dataset "$DATASET_V2" --out-json "$EVAL_JSON" 2>&1)"
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
note "STEP (ii) eval ran. $SUMMARY_LINE"

NEAR_RECALL="$(grep -oP 'near_recall=\K[0-9.]+' <<< "$SUMMARY_LINE")"
NEAR_PRECISION="$(grep -oP 'near_precision=\K[0-9.]+' <<< "$SUMMARY_LINE")"
FAR_RECALL="$(grep -oP 'far_recall=\K[0-9.]+' <<< "$SUMMARY_LINE")"
FAR_PRECISION="$(grep -oP 'far_precision=\K[0-9.]+' <<< "$SUMMARY_LINE")"
NEG_CLEAN_RATE="$(grep -oP 'neg_clean_rate=\K[0-9.]+' <<< "$SUMMARY_LINE")"

if [[ -z "$NEAR_RECALL" || -z "$NEAR_PRECISION" || -z "$FAR_RECALL" || -z "$FAR_PRECISION" ]]; then
    note "FAIL(ii): could not parse near/far recall+precision out of SUMMARY line"
    echo "FAIL"; exit 1
fi

if awk "BEGIN{exit !($NEAR_RECALL >= $NEAR_RECALL_MIN)}"; then
    note "PASS: held-out near-band recall $NEAR_RECALL >= $NEAR_RECALL_MIN"
else
    note "FAIL: held-out near-band recall $NEAR_RECALL < $NEAR_RECALL_MIN"; FAIL=1
fi
if awk "BEGIN{exit !($NEAR_PRECISION >= $NEAR_PRECISION_MIN)}"; then
    note "PASS: held-out near-band precision $NEAR_PRECISION >= $NEAR_PRECISION_MIN"
else
    note "FAIL: held-out near-band precision $NEAR_PRECISION < $NEAR_PRECISION_MIN"; FAIL=1
fi
if awk "BEGIN{exit !($FAR_RECALL >= $FAR_RECALL_MIN)}"; then
    note "PASS: held-out far-band recall $FAR_RECALL >= $FAR_RECALL_MIN"
else
    note "FAIL: held-out far-band recall $FAR_RECALL < $FAR_RECALL_MIN"; FAIL=1
fi
if awk "BEGIN{exit !($FAR_PRECISION >= $FAR_PRECISION_MIN)}"; then
    note "PASS: held-out far-band precision $FAR_PRECISION >= $FAR_PRECISION_MIN"
else
    note "FAIL: held-out far-band precision $FAR_PRECISION < $FAR_PRECISION_MIN"; FAIL=1
fi
note "REPORT (not gated): negative clean rate = ${NEG_CLEAN_RATE:-n/a} (ADR-0049 flat-world caveat applies)"

echo "=== [check_t17v2] STEP (iii)+(iv): head-to-head bias@50m + box-size monotonicity ==="
NN_V1_DIR="$T17V2_DIR/nn_detection_v1"
NN_V2_DIR="$T17V2_DIR/nn_detection_v2"
if [[ ! -f "$NN_V1_DIR/nn_detection_summary.json" || ! -f "$NN_V2_DIR/nn_detection_summary.json" ]]; then
    note "running multirange_nn_detection.py for v1 and v2 (not yet cached)..."
    "$VENV_TRAIN_PYTHON" "$REPO_ROOT/scripts/multirange_nn_detection.py" \
        --root "$CAPTURE_ROOT" --weights "$WEIGHTS_V1" --out-dir "$NN_V1_DIR" || {
        note "FAIL(iii): multirange_nn_detection.py (v1) failed"; echo "FAIL"; exit 1; }
    "$VENV_TRAIN_PYTHON" "$REPO_ROOT/scripts/multirange_nn_detection.py" \
        --root "$CAPTURE_ROOT" --weights "$WEIGHTS_V2" --out-dir "$NN_V2_DIR" || {
        note "FAIL(iii): multirange_nn_detection.py (v2) failed"; echo "FAIL"; exit 1; }
fi

CMP_OUT_JSON="$T17V2_DIR/comparison_summary_${TIMESTAMP}.json"
CMP_OUT="$("$VENV_TRAIN_PYTHON" "$REPO_ROOT/scripts/eval_t17v2_comparison.py" \
    --v1-dir "$NN_V1_DIR" --v2-dir "$NN_V2_DIR" --out "$CMP_OUT_JSON" 2>&1)"
CMP_EXIT=$?
echo "$CMP_OUT"
if [[ "$CMP_EXIT" -ne 0 ]]; then
    note "FAIL(iii): eval_t17v2_comparison.py exited $CMP_EXIT"
    echo "FAIL"; exit 1
fi
CMP_SUMMARY="$(grep '^SUMMARY' <<< "$CMP_OUT")"
if [[ -z "$CMP_SUMMARY" ]]; then
    note "FAIL(iii): no SUMMARY line in comparison output"
    echo "FAIL"; exit 1
fi
note "comparison ran. $CMP_SUMMARY"

BIAS50_V1="$(grep -oP 'bias50_v1=\K[-0-9.]+' <<< "$CMP_SUMMARY")"
BIAS50_V2="$(grep -oP 'bias50_v2=\K[-0-9.]+' <<< "$CMP_SUMMARY")"
MONOTONIC_V1="$(grep -oP 'monotonic_v1=\K(True|False)' <<< "$CMP_SUMMARY")"
MONOTONIC_V2="$(grep -oP 'monotonic_v2=\K(True|False)' <<< "$CMP_SUMMARY")"

if [[ -z "$BIAS50_V2" ]]; then
    note "FAIL(iii): could not parse bias50_v2 out of comparison SUMMARY"
    echo "FAIL"; exit 1
fi

BIAS50_V2_ABS="$(awk "BEGIN{v=$BIAS50_V2; if (v<0) v=-v; print v}")"
if awk "BEGIN{exit !($BIAS50_V2_ABS < $BIAS_50M_ABS_MAX)}"; then
    note "PASS: |ground_v2 bias @ 50 m| = $BIAS50_V2_ABS m < $BIAS_50M_ABS_MAX m (v1 was $BIAS50_V1 m)"
else
    note "FAIL: |ground_v2 bias @ 50 m| = $BIAS50_V2_ABS m >= $BIAS_50M_ABS_MAX m (v1 was $BIAS50_V1 m)"
    FAIL=1
fi

if [[ "$MONOTONIC_V2" == "True" ]]; then
    note "PASS: ground_v2 box-size head is monotonic vs range (v1 was monotonic_v1=$MONOTONIC_V1)"
else
    note "FAIL: ground_v2 box-size head is STILL non-monotonic vs range (v1 was monotonic_v1=$MONOTONIC_V1)"
    FAIL=1
fi

note "eval json: $EVAL_JSON"
note "comparison json: $CMP_OUT_JSON"

if [[ "$FAIL" -eq 0 ]]; then
    note "PASS"
    echo "PASS"; exit 0
else
    note "FAIL: one or more pre-registered thresholds not met (see above) -- reported honestly, not retried"
    echo "FAIL"; exit 1
fi
