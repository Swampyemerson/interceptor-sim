#!/usr/bin/env bash
# Hailo DFC dry-run (docs/hailo_toolchain_runbook.md): answers "does our
# model compile, and in how many contexts, for hailo8" BEFORE purchase.
# No Hailo hardware needed; CPU-only quantization is fine for THIS question.
#
# Prereq (builder): download the DFC 3.33.x wheel from the Hailo Developer
# Zone (free account) and pass its path:
#   bash scripts/nn/dfc_dryrun.sh ~/Downloads/hailo_dataflow_compiler-*.whl
#
# What it does: fresh venv -> install DFC + ultralytics -> export stock
# yolov8n to ONNX at 640 and 1024 -> hailomz compile for hailo8 -> grep the
# context verdict out of each compile log. Results + logs land in
# runs/dfc_dryrun/. The 1024 compile is the open experiment (ADR-0125:
# higher-res input is REQUIRED if the 118-deg lens returns).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WHEEL="${1:?path to hailo_dataflow_compiler wheel}"
OUT="$REPO/runs/dfc_dryrun"
VENV="$REPO/.venv-dfc"
mkdir -p "$OUT"

if [ ! -d "$VENV" ]; then
    python3 -m venv "$VENV"
    "$VENV/bin/pip" install -q --upgrade pip
    "$VENV/bin/pip" install -q "$WHEEL" "ultralytics>=8.4.97" hailo-model-zoo \
        || { echo "install failed -- check the DFC wheel's python version"; exit 1; }
fi

cd "$OUT"
for SZ in 640 1024; do
    echo "=== exporting yolov8n @$SZ ==="
    "$VENV/bin/yolo" export model=yolov8n.pt format=onnx imgsz=$SZ opset=11 \
        > "export_$SZ.log" 2>&1
    mv -f "$REPO"/yolov8n.onnx "yolov8n_$SZ.onnx" 2>/dev/null || \
        mv -f yolov8n.onnx "yolov8n_$SZ.onnx" 2>/dev/null || true
    echo "=== compiling yolov8n @$SZ for hailo8 (CPU; may take a while) ==="
    if "$VENV/bin/hailomz" compile yolov8n --ckpt "yolov8n_$SZ.onnx" \
            --hw-arch hailo8 --classes 80 > "compile_$SZ.log" 2>&1; then
        echo "compile @$SZ: OK"
    else
        echo "compile @$SZ: FAILED (see $OUT/compile_$SZ.log)"
    fi
    grep -i "context" "compile_$SZ.log" | tail -4 || true
done
echo "=== verdicts ==="
for SZ in 640 1024; do
    echo "--- $SZ ---"
    grep -iE "single context|multi-context|Using Multi" "compile_$SZ.log" | tail -2 \
        || echo "(no context line -- read $OUT/compile_$SZ.log)"
done
