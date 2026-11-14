#!/usr/bin/env bash
# Compile a 1-class yolov8n ONNX for hailo8 in two variants (CPU, optimisation level 0):
#   plain : class outputs forced to the range 0..1 (the one-class fix)
#   h16   : the same, plus the six head output convs at 16-bit (rescue-ladder step 4)
# Usage (inside ubuntu:22.04): compile_v8n.sh <tag> <onnx> <calib_dir>
set -uo pipefail
export USER="${USER:-root}" LOGNAME="${LOGNAME:-root}" HOME="${HOME:-/root}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/runs/hailo_drone_20261113"; VENV="$REPO/.venv-dfc-2204"; TAG="$1"; ONNX="$2"; CALIB="$3"
PPC="$("$VENV/bin/python" -c 'import hailo_model_zoo, os; print(os.path.join(os.path.dirname(hailo_model_zoo.__file__), "cfg", "postprocess_config"))')"
JSON="$PPC/yolov8n_nms_config.json"; [ -f "$JSON.stock80" ] || cp "$JSON" "$JSON.stock80"
SIZE=$("$VENV/bin/python" -c "import onnx,sys; d=onnx.load(sys.argv[1]).graph.input[0].type.tensor_type.shape.dim; print(d[2].dim_value, d[3].dim_value)" "$ONNX")
"$VENV/bin/python" - "$JSON" $SIZE <<'PYEOF'
import json, sys
p, h, w = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]); d = json.load(open(p + ".stock80")); d["classes"] = 1; d["image_dims"] = [h, w]
json.dump(d, open(p, "w"), indent=1); print("NMS JSON: classes 1, image_dims", d["image_dims"])
PYEOF
restore() { cp -f "$JSON.stock80" "$JSON"; }; trap restore EXIT
ENDS=$("$VENV/bin/python" - "$ONNX" <<'PYEOF'
import sys, re, onnx
m = onnx.load(sys.argv[1]); pat = re.compile(r'/model\.\d+/cv[23]\.\d+/cv[23]\.\d+\.2/Conv$')
names = [n.name for n in m.graph.node if pat.search(n.name)]; assert len(names) == 6, names; print(" ".join(names))
PYEOF
)
for V in plain h16; do
    mkdir -p "$OUT/build_${TAG}_$V"; cd "$OUT/build_${TAG}_$V"
    { echo 'normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])'
      echo 'change_output_activation(conv42, sigmoid)'; echo 'change_output_activation(conv53, sigmoid)'; echo 'change_output_activation(conv63, sigmoid)'
      echo 'quantization_param([conv42, conv53, conv63], force_range_out=[0.0, 1.0])'
      [ "$V" = h16 ] && echo 'quantization_param([conv41, conv42, conv52, conv53, conv62, conv63], precision_mode=a16_w16)'
      echo "nms_postprocess(\"$JSON\", meta_arch=yolov8, engine=cpu)"; echo 'allocator_param(width_splitter_defuse=disabled)'; } > model.alls
    T0=$(date +%s)
    if CUDA_VISIBLE_DEVICES="" "$VENV/bin/hailomz" compile yolov8n --ckpt "$ONNX" --calib-path "$CALIB" --end-node-names $ENDS --model-script model.alls --hw-arch hailo8 > "$OUT/compile_${TAG}_$V.log" 2>&1; then
        mv -f yolov8n.hef "$OUT/${TAG}_$V.hef"; echo "$TAG $V: OK in $(( $(date +%s) - T0 )) s"
    else
        echo "$TAG $V: FAILED in $(( $(date +%s) - T0 )) s"; grep -E "Error|Exception|rror:" "$OUT/compile_${TAG}_$V.log" | tail -2 | cut -c1-300
    fi
    grep -iE "single context|Multi-context" "$OUT/compile_${TAG}_$V.log" | tail -1
done
echo V8N_COMPILE_DONE
