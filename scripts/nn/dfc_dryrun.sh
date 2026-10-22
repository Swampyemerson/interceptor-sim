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

# Hailo-8 is served by the DFC 3.x line ONLY. 5.x rejects --hw-arch hailo8
# outright (valid choices there are hailo10/12/15 archs; staff: "5.xx is for
# Hailo-10 and Hailo-15 only" -- community.hailo.ai/t/18193). Fail loud
# before burning an install on the wrong compiler.
case "$(basename "$WHEEL")" in
    hailo_dataflow_compiler-3.*) ;;
    *) echo "WRONG WHEEL: $(basename "$WHEEL") -- Hailo-8 needs the DFC 3.x line"
       echo "(3.33.x pinned in docs/hailo_toolchain_runbook.md; 5.x = Hailo-10/15 only)"
       exit 2 ;;
esac

# A venv DIR can exist half-built (an interrupted install leaves one), so
# completeness is marked by a sentinel written only after pip succeeds --
# never trust the directory alone.
# Model Zoo v2.17's pins (pillow<=9.3.0, scikit-image==0.20.0) have no
# Python 3.12 wheels and do not build there -- the venv must be 3.10/3.11
# (hit live 2026-10-21: 3.12 fell back to sdist builds, BackendUnavailable).
PY=$(command -v python3.11 || command -v python3.10) \
    || { echo "need python3.11 or 3.10 (MZ 2.17 pins predate 3.12)"; exit 2; }

if [ ! -f "$VENV/.install-ok" ]; then
    rm -rf "$VENV"
    "$PY" -m venv "$VENV"
    "$VENV/bin/pip" install -q --upgrade pip
    # hailo-model-zoo is NOT on PyPI -- it installs from Hailo's GitHub.
    # MZ 2.17 is the staff-named pairing for Hailo-8 on current DFC 3.x.
    # Its setup.py reads the DFC's package metadata, so it must install
    # AFTER the DFC and WITHOUT build isolation (isolated builds can't see
    # the venv's DFC -- PackageNotFoundError, hit live 2026-10-21).
    # A failed install removes the venv so a rerun retries instead of
    # assuming the half-built venv is good.
    "$VENV/bin/pip" install "$WHEEL" "ultralytics>=8.4.97" setuptools wheel \
        || { echo "DFC wheel install failed -- check its python version"
             rm -rf "$VENV"; exit 1; }
    "$VENV/bin/pip" install --no-build-isolation \
        "git+https://github.com/hailo-ai/hailo_model_zoo.git@v2.17" \
        || { echo "model-zoo install failed (check DFC/MZ version pairing)"
             rm -rf "$VENV"; exit 1; }
    touch "$VENV/.install-ok"
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
