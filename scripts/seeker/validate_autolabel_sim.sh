#!/usr/bin/env bash
# Boot the apriltag world headless, run the auto-labeler validation sweep, tear
# down, cross-check with the SHIPPED autolabel_from_apriltag.py CLI, and render
# the curves + placard-sizing math (build_plan P0; docs/real_data_pipeline.md
# "Validate the auto-labeler itself" -- flagged NOT yet run).
#
# ONE sim, project law. Boot grep is sim-paced (ADR-0004). Kill logic in-file via
# scripts/sim_kill.sh (self-kill rule). This is a STATIC set-pose capture, not a
# timing-sensitive flight, so the ADR-0015 load confound does not bind -- but if
# render anomalies appear, re-run at idle. Usage: bash scripts/seeker/validate_autolabel_sim.sh
set -uo pipefail
cd "$(dirname "$0")/../.."
REPO_ROOT="$PWD"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PY="$REPO_ROOT/.venv/bin/python"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$REPO_ROOT/scripts/seeker/data/autolabel_sim_${STAMP}"
CSV="$REPO_ROOT/logs/autolabel_sim_${STAMP}.csv"
LOG_DIR="$REPO_ROOT/logs/autolabel_sim"
SIM_LOG="$LOG_DIR/sim_${STAMP}.log"
IMAGES_DIR="$REPO_ROOT/docs/images"
SUMMARY="$LOG_DIR/summary_${STAMP}.json"
mkdir -p "$LOG_DIR" "$OUT" "$IMAGES_DIR"
READY_STR="Startup script returned successfully"; READY_TIMEOUT_S=160; SETTLE_S=6
source "$REPO_ROOT/scripts/sim_gpu_render.sh" 2>/dev/null || true

teardown() { bash "$REPO_ROOT/scripts/sim_kill.sh" 2>/dev/null; sleep 3; }

echo "[validate] boot apriltag world ..."
teardown
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" > "$SIM_LOG" 2>&1 &
ready=0
for ((i=0;i<READY_TIMEOUT_S;i++)); do
    grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null && { ready=1; break; }
    sleep 1
done
[[ "$ready" -ne 1 ]] && { echo "[validate] FAIL sim-ready (see $SIM_LOG)"; teardown; exit 1; }
sleep "$SETTLE_S"

echo "[validate] === sweep ==="
"$PY" "$REPO_ROOT/scripts/seeker/validate_autolabel_sim.py" --sweep --out "$OUT" --csv "$CSV"
SWEEP_RC=$?

echo "[validate] teardown sim"
teardown

if [[ "$SWEEP_RC" -ne 0 ]]; then
    echo "[validate] sweep FAILED rc=$SWEEP_RC"; exit "$SWEEP_RC"
fi

echo "[validate] === autolabel_from_apriltag.py CLI cross-check (shipped tool over the captures) ==="
"$PY" "$REPO_ROOT/scripts/seeker/autolabel_from_apriltag.py" \
    --frames "$OUT/images" --calib "$OUT/calib.json" \
    --tag-size 0.5 --drone-size 0.35 --out "$OUT/autolabel_out" 2>&1 | tee "$LOG_DIR/autolabel_cli_${STAMP}.log"

# measured sim fx (for the placard transfer) straight from the sim CameraInfo
SIM_FX="$("$PY" -c "import json;print(json.load(open('$OUT/calib.json'))['fx'])")"
echo "[validate] measured sim fx = $SIM_FX px"

echo "[validate] === analyze -> curves + placard sizing ==="
"$PY" "$REPO_ROOT/scripts/seeker/validate_autolabel_sim.py" --analyze "$CSV" \
    --images-dir "$IMAGES_DIR" --sim-fx "$SIM_FX" --summary-out "$SUMMARY"

echo "[validate] DONE. csv=$CSV summary=$SUMMARY images=$IMAGES_DIR"
