#!/usr/bin/env bash
# Phase 4 (docs/seeker_v3_dataset_plan.md §8/§9): recalibrate the v3 range-span
# sidecar on the v3 VAL set, then score v2-vs-v3 on the held-out eval pool +
# the held-out grid far-range probe, producing gates G1-G5 + the reframed
# verdict (recall-primary, mis-lock separate, phantom CI'd secondary). Offline,
# no sim. NEW file (task #28).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY="$REPO/.venv-seeker/bin/python"
V2="$REPO/scripts/seeker/weights/drone_finetuned_v2.onnx"
V3="$REPO/scripts/seeker/weights/drone_finetuned_v3.onnx"
VAL="$REPO/scripts/seeker/data/onboard_dataset_v3/images/val"
EVAL="$REPO/scripts/seeker/data/v3_flight_eval"
GRID="$REPO/scripts/seeker/data/v3_grid"
OUT="$REPO/logs/seeker_v3_eval"

if [[ ! -f "$V3" ]]; then echo "PHASE4_FAIL: $V3 not found (training incomplete?)"; exit 2; fi
mkdir -p "$OUT"

echo "=== (1) recalibrate the v3 range-span sidecar on the v3 VAL set ==="
"$PY" "$REPO/scripts/seeker/calibrate_range.py" --weights "$V3" --images "$VAL"
CAL_RC=$?
echo "calibrate rc=$CAL_RC (span sidecar -> ${V3}.calib.json)"

echo "=== (2) score v2 vs v3 (identical held-out eval frames + grid 3/18m probe) ==="
"$PY" "$REPO/scripts/eval_seeker_v3.py" --v2 "$V2" --v3 "$V3" \
  --eval-dir "$EVAL" --grid-dir "$GRID" --out "$OUT" 2>&1 | tee "$OUT/eval_report.txt"
EVAL_RC="${PIPESTATUS[0]}"
echo "PHASE4_EVAL_COMPLETE eval_rc=$EVAL_RC calibrate_rc=$CAL_RC (report -> $OUT/eval_report.txt)"
exit "$EVAL_RC"
