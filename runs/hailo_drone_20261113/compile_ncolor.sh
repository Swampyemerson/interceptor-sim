#!/usr/bin/env bash
# Compile the trained n-color (YOLO11n, 1 class, 640) for hailo8. Runs INSIDE ubuntu:22.04.
set -uo pipefail
export USER="${USER:-root}" LOGNAME="${LOGNAME:-root}" HOME="${HOME:-/root}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/runs/hailo_drone_20261113"; VENV="$REPO/.venv-dfc-2204"; CALIB="$OUT/calib640"
cd "$OUT"; cp -f "$REPO/scripts/seeker/weights/nn_tier/n-color.pt" n-color.pt
"$VENV/bin/yolo" export model=n-color.pt format=onnx imgsz=640 opset=11 > export_ncolor.log 2>&1 || { echo "export FAILED"; tail -3 export_ncolor.log; exit 1; }
PPC="$("$VENV/bin/python" -c 'import hailo_model_zoo, os; print(os.path.join(os.path.dirname(hailo_model_zoo.__file__), "cfg", "postprocess_config"))')"
JSON="$PPC/yolov11n_nms_config.json"
"$VENV/bin/python" - "$JSON" <<'PYEOF'
import json, os, sys, urllib.request
p = sys.argv[1]
if not os.path.exists(p + ".stock"):
    if not os.path.exists(p):
        urllib.request.urlretrieve("https://raw.githubusercontent.com/hailo-ai/hailo_model_zoo/v2.17/hailo_model_zoo/cfg/postprocess_config/yolov11n_nms_config.json", p)
    open(p + ".stock", "w").write(open(p).read())
d = json.load(open(p + ".stock")); d["classes"] = 1
json.dump(d, open(p, "w"), indent=1); print("NMS JSON classes ->", d["classes"], "score th", d.get("nms_scores_th"))
PYEOF
restore() { cp -f "$JSON.stock" "$JSON"; }; trap restore EXIT
# 1-class fix (first attempt failed 2026-11-13: NegativeSlopeExponentNonFixable in conv80, the
# stride-32 class output, whose range is tiny when there is one class). Hailo-staff suggestion
# (community.hailo.ai/t/3556): force the class outputs' range to [0, 1].
cat > n-color.alls <<ALLS
normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])
change_output_activation(conv54, sigmoid)
change_output_activation(conv65, sigmoid)
change_output_activation(conv80, sigmoid)
quantization_param([conv54, conv65, conv80], force_range_out=[0.0, 1.0])
nms_postprocess("$JSON", meta_arch=yolov8, engine=cpu)
allocator_param(width_splitter_defuse=disabled)
ALLS
T0=$(date +%s)
if CUDA_VISIBLE_DEVICES="" "$VENV/bin/hailomz" compile yolov11n --ckpt n-color.onnx --calib-path "$CALIB" --model-script n-color.alls --hw-arch hailo8 > compile_ncolor.log 2>&1; then
    mv -f yolov11n.hef n-color_640.hef; echo "compile n-color: OK in $(( $(date +%s) - T0 )) s"
else
    echo "compile n-color: FAILED in $(( $(date +%s) - T0 )) s"; tail -6 compile_ncolor.log
fi
grep -iE "single context|multi.context|Bandwidth of inter|Adaround|Finetune|optimization level|compression" compile_ncolor.log | tail -8 || true
echo NCOLOR_COMPILE_DONE
