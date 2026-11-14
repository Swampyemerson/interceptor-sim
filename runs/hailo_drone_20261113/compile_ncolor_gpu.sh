#!/usr/bin/env bash
# Rescue-ladder step 1: n-color compiled WITH the GPU so the optimisers actually run.
# Runs inside nvidia/cuda:12.5.1-cudnn-runtime-ubuntu22.04 with --gpus all; cuDNN 9.3 at /cudnn.
set -uo pipefail
export USER="${USER:-root}" LOGNAME="${LOGNAME:-root}" HOME="${HOME:-/root}" TF_FORCE_GPU_ALLOW_GROWTH=true CUDA_VISIBLE_DEVICES=0   # explicit: the DFC only auto-picks an IDLE GPU
export LD_LIBRARY_PATH="/cudnn/nvidia/cudnn/lib:${LD_LIBRARY_PATH:-}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/runs/hailo_drone_20261113"; VENV="$REPO/.venv-dfc-2204"; CALIB="$OUT/calib640"
mkdir -p "$OUT/gpu_build"; cd "$OUT/gpu_build"; cp -f "$OUT/n-color.onnx" n-color.onnx
PPC="$("$VENV/bin/python" -c 'import hailo_model_zoo, os; print(os.path.join(os.path.dirname(hailo_model_zoo.__file__), "cfg", "postprocess_config"))')"
JSON="$PPC/yolov11n_nms_config.json"
"$VENV/bin/python" - "$JSON" <<'PYEOF'
import json, sys
p = sys.argv[1]; d = json.load(open(p + ".stock")); d["classes"] = 1; json.dump(d, open(p, "w"), indent=1)
PYEOF
restore() { cp -f "$JSON.stock" "$JSON"; }; trap restore EXIT
cat > n-color-gpu.alls <<ALLS
normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])
change_output_activation(conv54, sigmoid)
change_output_activation(conv65, sigmoid)
change_output_activation(conv80, sigmoid)
quantization_param([conv54, conv65, conv80], force_range_out=[0.0, 1.0])
model_optimization_flavor(optimization_level=2, compression_level=0)
model_optimization_config(calibration, calibset_size=1200)
nms_postprocess("$JSON", meta_arch=yolov8, engine=cpu)
allocator_param(width_splitter_defuse=disabled)
ALLS
T0=$(date +%s)
if "$VENV/bin/hailomz" compile yolov11n --ckpt n-color.onnx --calib-path "$CALIB" --model-script n-color-gpu.alls --hw-arch hailo8 > "$OUT/compile_ncolor_gpu.log" 2>&1; then
    mv -f yolov11n.hef "$OUT/n-color_640_opt2.hef"; echo "gpu compile: OK in $(( $(date +%s) - T0 )) s"
else
    echo "gpu compile: FAILED in $(( $(date +%s) - T0 )) s"; tail -5 "$OUT/compile_ncolor_gpu.log" | cut -c1-300
fi
grep -iE "optimization level|Adaround|Finetune|Selected GPU|no available GPU|single context|Multi-context|calibration" "$OUT/compile_ncolor_gpu.log" | grep -v "%" | tail -10 | cut -c1-200
echo NCOLOR_GPU_COMPILE_DONE
