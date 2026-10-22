#!/usr/bin/env bash
# Single-class (--classes 1) DFC compile dry-run for hailo8 -- one step past
# the stock dry-run (runs/dfc_dryrun): proves the REAL deployment compile
# path (our net is a 1-class yolov8n). Topology/fit evidence ONLY: the model
# is RANDOM-WEIGHT nc=1 surgery on the stock yaml, and calibration is the
# synthetic set -- no accuracy/recall claim can ever be quoted from this.
#
# Runs INSIDE the ubuntu:22.04 container (see runs/dfc_dryrun/README.md for
# why 24.04 is a dead end). Requires the stock dry-run's venv sentinel.
set -euo pipefail
export USER="${USER:-$(id -un)}"
export LOGNAME="${LOGNAME:-$USER}"
export HOME="${HOME:-/root}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/runs/dfc_dryrun_single"
VENV="${DFC_VENV:-$REPO/.venv-dfc-2204}"

[ -f "$VENV/.install-ok" ] || { echo "venv sentinel missing -- run the stock \
dry-run first (scripts/nn/dfc_dryrun.sh builds $VENV)"; exit 2; }

# NMS postprocess template (the --classes rewrite path needs it; pip MZ
# ships no postprocess_config/ -- same fetch as the stock script).
PPC="$("$VENV/bin/python" -c 'import hailo_model_zoo, os; print(os.path.join(os.path.dirname(hailo_model_zoo.__file__), "cfg", "postprocess_config"))')"
if [ ! -f "$PPC/yolov8n_nms_config.json" ]; then
    mkdir -p "$PPC"
    curl -fsSL -o "$PPC/yolov8n_nms_config.json" \
        "https://raw.githubusercontent.com/hailo-ai/hailo_model_zoo/v2.17/hailo_model_zoo/cfg/postprocess_config/yolov8n_nms_config.json" \
        || { echo "failed to fetch yolov8n_nms_config.json"; exit 1; }
fi

CALIB="$REPO/runs/dfc_dryrun/calib_synth"
[ -d "$CALIB" ] || { echo "synthetic calib set missing ($CALIB)"; exit 2; }

cd "$OUT"

echo "=== building + exporting RANDOM-WEIGHT single-class yolov8n @640 ==="
"$VENV/bin/python" - <<'PYEOF'
import pathlib, yaml
import ultralytics
from ultralytics import YOLO

# nc=1 surgery on the stock yaml; filename keeps the 'yolov8n' stem so
# ultralytics resolves the 'n' scale.
src = pathlib.Path(ultralytics.__file__).parent / "cfg" / "models" / "v8" / "yolov8.yaml"
cfg = yaml.safe_load(src.read_text())
cfg["nc"] = 1
out = pathlib.Path("yolov8n-1c.yaml")
out.write_text(yaml.safe_dump(cfg))
m = YOLO(str(out))           # random weights, 1-class head
m.export(format="onnx", imgsz=640, opset=11)
print("exported", [p.name for p in pathlib.Path(".").glob("*.onnx")])
PYEOF
ONNX=$(ls -t "$OUT"/*.onnx | head -1)
echo "onnx: $ONNX"

# End nodes from the ACTUAL exported onnx (numbering shifts with surgery).
ENDS=$("$VENV/bin/python" - "$ONNX" <<'PYEOF'
import sys, re, onnx
m = onnx.load(sys.argv[1])
pat = re.compile(r'/model\.\d+/cv[23]\.\d+/cv[23]\.\d+\.2/Conv$')
names = [n.name for n in m.graph.node if pat.search(n.name)]
assert len(names) == 6, f"expected 6 head convs, found {len(names)}: {names}"
print(" ".join(names))
PYEOF
)
echo "end nodes: $ENDS"

echo "=== compiling single-class yolov8n @640 for hailo8 (--classes 1) ==="
if CUDA_VISIBLE_DEVICES="" "$VENV/bin/hailomz" compile yolov8n \
        --ckpt "$ONNX" --calib-path "$CALIB" \
        --end-node-names $ENDS --classes 1 \
        --hw-arch hailo8 > "$OUT/compile_1c_640.log" 2>&1; then
    echo "compile single-class @640: OK"
else
    echo "compile single-class @640: FAILED (see $OUT/compile_1c_640.log)"
fi
grep -iE "single context|multi.context|Successful Compilation|error" \
    "$OUT/compile_1c_640.log" | tail -5 || true
