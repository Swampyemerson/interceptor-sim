#!/usr/bin/env bash
# Fit/speed compile of STOCK yolov8n (80-class, pretrained) at the input sizes the
# AR0234 (1920x1200) would actually feed: 1280x800 (1.5x downscale, same aspect) and
# 1920x1216 (native, padded to a multiple of 32). Answers: how many contexts, and
# (on the Pi) how fast. Calibration is SYNTHETIC -- no accuracy number may be quoted
# from these builds. Runs INSIDE an ubuntu:22.04 container (runbook: 24.04 is a dead end).
set -uo pipefail
export USER="${USER:-root}" LOGNAME="${LOGNAME:-root}" HOME="${HOME:-/root}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/runs/dfc_native_res_20261113"; VENV="$REPO/.venv-dfc-2204"; CALIB="$REPO/runs/dfc_dryrun/calib_synth"
[ -f "$VENV/.install-ok" ] || { echo "venv sentinel missing"; exit 2; }
cd "$OUT"; cp -n "$REPO/runs/dfc_dryrun/yolov8n.pt" . 
for SZ in "800,1280 1280x800" "1216,1920 1920x1216"; do
    set -- $SZ; IMGSZ=$1; TAG=$2
    echo "=== export yolov8n @$TAG ==="
    "$VENV/bin/yolo" export model=yolov8n.pt format=onnx imgsz=$IMGSZ opset=11 > "export_$TAG.log" 2>&1 \
        || { echo "export @$TAG FAILED"; tail -3 "export_$TAG.log"; continue; }
    mv -f yolov8n.onnx "yolov8n_$TAG.onnx"
    ENDS=$("$VENV/bin/python" - "yolov8n_$TAG.onnx" <<'PYEOF'
import sys, re, onnx
m = onnx.load(sys.argv[1])
pat = re.compile(r'/model\.\d+/cv[23]\.\d+/cv[23]\.\d+\.2/Conv$')
names = [n.name for n in m.graph.node if pat.search(n.name)]
assert len(names) == 6, f"expected 6 head convs, found {len(names)}"
print(" ".join(names))
PYEOF
)
    echo "=== compile yolov8n @$TAG for hailo8 (CPU) ==="
    T0=$(date +%s)
    if CUDA_VISIBLE_DEVICES="" "$VENV/bin/hailomz" compile yolov8n --ckpt "yolov8n_$TAG.onnx" \
            --calib-path "$CALIB" --end-node-names $ENDS --hw-arch hailo8 > "compile_$TAG.log" 2>&1; then
        mv -f yolov8n.hef "yolov8n_$TAG.hef"; echo "compile @$TAG: OK in $(( $(date +%s) - T0 )) s"
    else
        echo "compile @$TAG: FAILED in $(( $(date +%s) - T0 )) s"; tail -4 "compile_$TAG.log"
    fi
    grep -iE "single context|multi.context|contexts|Bandwidth of inter" "compile_$TAG.log" | tail -4 || true
done
echo NATIVE_RES_DONE
