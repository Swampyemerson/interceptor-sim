#!/usr/bin/env bash
# Seeker v2 static-probe gate (pre-registered gate 1, docs/seeker_v2_plan.md).
# Boots the markerless sim, probes the v1 range list with the v2 weights,
# tears down. Runs as a FILE so the pkill patterns can't match the caller
# (the mc_batch self-kill lesson).
set -u
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM_LOG="$REPO/logs/probe_v2_sim_$(date -u +%Y%m%dT%H%M%SZ).log"
WEIGHTS="${1:-$REPO/scripts/seeker/weights/drone_finetuned_v2.onnx}"
OUT="${2:-$REPO/logs/markerless_probe_v2}"

kill_sim() {
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 3
}

kill_sim
setsid bash -c "cd '$HOME/PX4-Autopilot' && tail -f /dev/null | env PX4_GZ_WORLD=markerless GZ_SIM_RESOURCE_PATH='$REPO/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
    > "$SIM_LOG" 2>&1 &
ready=0
for _ in $(seq 1 150); do
    grep -q "Startup script returned successfully" "$SIM_LOG" 2>/dev/null && { ready=1; break; }
    sleep 1
done
if [[ "$ready" -ne 1 ]]; then
    echo "PROBE_V2: SIM_BOOT_FAIL (see $SIM_LOG)"; tail -5 "$SIM_LOG"; kill_sim; exit 1
fi
echo "PROBE_V2: sim ready"
sleep 6

MARKERLESS_NN_WEIGHTS="$WEIGHTS" \
INTERCEPTOR_WORLD_NAME=markerless INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \
"$REPO/.venv-seeker/bin/python" "$REPO/scripts/seeker/probe_markerless_range.py" \
    --ranges 2,3,4,5,6,7,9,12 --out "$OUT"
PROBE_EXIT=$?
kill_sim
echo "PROBE_V2: exit=$PROBE_EXIT results:"
cat "$OUT/probe_results.csv" 2>/dev/null
exit "$PROBE_EXIT"
