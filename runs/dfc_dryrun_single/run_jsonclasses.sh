#!/usr/bin/env bash
# Recovery-lead cell (registered in the fork's README open lead): same
# RANDOM-WEIGHT nc=1 onnx, but classes declared by EDITING the NMS JSON
# (classes: 80 -> 1) with NO --classes flag -- tests whether the 2-context
# split came from the --classes rewrite path or from the 1-class topology
# itself. Topology/fit evidence ONLY (random weights, synthetic calib).
# Runs INSIDE the ubuntu:22.04 container; JSON edited on a venv COPY and
# restored after, so the stock file never drifts.
set -euo pipefail
export USER="${USER:-$(id -un)}"
export LOGNAME="${LOGNAME:-$USER}"
export HOME="${HOME:-/root}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/runs/dfc_dryrun_single"
VENV="${DFC_VENV:-$REPO/.venv-dfc-2204}"
CALIB="$REPO/runs/dfc_dryrun/calib_synth"

[ -f "$VENV/.install-ok" ] || { echo "venv sentinel missing"; exit 2; }
ONNX=$(ls -t "$OUT"/*.onnx | head -1)
[ -n "$ONNX" ] || { echo "no nc=1 onnx -- run run_single.sh first"; exit 2; }
echo "onnx: $ONNX"

PPC="$("$VENV/bin/python" -c 'import hailo_model_zoo, os; print(os.path.join(os.path.dirname(hailo_model_zoo.__file__), "cfg", "postprocess_config"))')"
JSON="$PPC/yolov8n_nms_config.json"
[ -f "$JSON" ] || { echo "NMS JSON missing ($JSON)"; exit 2; }
cp "$JSON" "$JSON.stock"
restore() { mv -f "$JSON.stock" "$JSON"; }
trap restore EXIT
"$VENV/bin/python" - "$JSON" <<'PYEOF'
import json, sys
p = sys.argv[1]
d = json.load(open(p))
assert "classes" in d, f"no classes key; keys={list(d)}"
d["classes"] = 1
json.dump(d, open(p, "w"), indent=1)
print("NMS JSON classes ->", d["classes"])
PYEOF

ENDS=$("$VENV/bin/python" - "$ONNX" <<'PYEOF'
import sys, re, onnx
m = onnx.load(sys.argv[1])
pat = re.compile(r'/model\.\d+/cv[23]\.\d+/cv[23]\.\d+\.2/Conv$')
names = [n.name for n in m.graph.node if pat.search(n.name)]
assert len(names) == 6, f"expected 6 head convs, found {len(names)}"
print(" ".join(names))
PYEOF
)

echo "=== compiling nc=1 onnx, classes via NMS JSON, NO --classes ==="
if CUDA_VISIBLE_DEVICES="" "$VENV/bin/hailomz" compile yolov8n \
        --ckpt "$ONNX" --calib-path "$CALIB" \
        --end-node-names $ENDS \
        --hw-arch hailo8 > "$OUT/compile_1c_json_640.log" 2>&1; then
    echo "compile (json-classes) @640: OK"
else
    echo "compile (json-classes) @640: FAILED (see $OUT/compile_1c_json_640.log)"
fi
grep -iE "single context|multi.context|contexts|Bandwidth of inter" \
    "$OUT/compile_1c_json_640.log" | tail -5 || true
