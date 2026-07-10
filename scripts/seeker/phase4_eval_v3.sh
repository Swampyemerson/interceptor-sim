#!/usr/bin/env bash
# Phase 4 (docs/seeker_v3_dataset_plan.md §8/§9): recalibrate the v3 range-span
# sidecar on the v3 VAL set, then score v2-vs-v3 on the held-out eval pool +
# the held-out grid far-range probe, producing gates G1-G5 + the reframed
# verdict. Offline, no sim. NEW file (task #28).
#
# SPAN FOOTGUN GUARDS (audit fix 4): v3 has no calib sidecar until step (1), so
# a premature score would silently use span=1.0 (v3 range ~8.5% off vs v2's
# 0.9216). This script (a) FAILS if MARKERLESS_SPAN_M is set (it would override
# BOTH models' span), (b) recalibrates v3 FIRST and asserts the sidecar was
# written, (c) confirms the scorer resolved the v3 span from the SIDECAR (not
# 'default'), and (d) writes to a FRESH timestamped --out.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY="$REPO/.venv-seeker/bin/python"
V2="$REPO/scripts/seeker/weights/drone_finetuned_v2.onnx"
V3="$REPO/scripts/seeker/weights/drone_finetuned_v3.onnx"
VAL="$REPO/scripts/seeker/data/onboard_dataset_v3/images/val"
EVAL="$REPO/scripts/seeker/data/v3_flight_eval"
GRID="$REPO/scripts/seeker/data/v3_grid"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$REPO/logs/seeker_v3_eval/run_${STAMP}"

if [[ ! -f "$V3" ]]; then echo "PHASE4_FAIL: $V3 not found (training incomplete?)"; exit 2; fi
if [[ -n "${MARKERLESS_SPAN_M:-}" ]]; then
  echo "PHASE4_FAIL: MARKERLESS_SPAN_M is set ('${MARKERLESS_SPAN_M}') -- it would OVERRIDE both models' span. Unset it and rerun."
  exit 2
fi
mkdir -p "$OUT"
echo "[phase4] out=$OUT  MARKERLESS_SPAN_M=<unset OK>"

echo "=== (1) recalibrate the v3 range-span sidecar on the v3 VAL set ==="
"$PY" "$REPO/scripts/seeker/calibrate_range.py" --weights "$V3" --images "$VAL" | tee "$OUT/calibrate.log"
CAL_RC="${PIPESTATUS[0]}"
SIDECAR="${V3}.calib.json"
if [[ "$CAL_RC" -ne 0 || ! -f "$SIDECAR" ]]; then
  echo "PHASE4_FAIL: v3 calibration failed (rc=$CAL_RC) or sidecar missing ($SIDECAR)"; exit 2
fi
echo "[phase4] v3 span sidecar written: $SIDECAR -> $(cat "$SIDECAR")"

echo "=== (2) score v2 vs v3 (identical held-out eval frames + grid 3/18m probe) ==="
"$PY" "$REPO/scripts/eval_seeker_v3.py" --v2 "$V2" --v3 "$V3" \
  --eval-dir "$EVAL" --grid-dir "$GRID" --out "$OUT" 2>&1 | tee "$OUT/eval_report.txt"
EVAL_RC="${PIPESTATUS[0]}"

# (c) confirm the scorer resolved v3's span from the SIDECAR, not the 1.0 default
if grep -qE "^\[eval_v3\] v3 .*\(calib drone_finetuned_v3\.onnx\.calib\.json\)" "$OUT/eval_report.txt"; then
  echo "[phase4] OK: v3 span resolved from the calib sidecar (not the 1.0 default)."
else
  echo "[phase4] WARN: v3 span source is NOT the sidecar -- check the span line above!"
  grep -E "^\[eval_v3\] v[23] weights" "$OUT/eval_report.txt" || true
fi
echo "PHASE4_EVAL_COMPLETE eval_rc=$EVAL_RC calibrate_rc=$CAL_RC (report -> $OUT/eval_report.txt)"
exit "$EVAL_RC"
