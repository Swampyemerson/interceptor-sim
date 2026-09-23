#!/usr/bin/env bash
# ONE hero flight of the ADOPTED sprint config (AE5dashZ: coded dash 16 m/s,
# accel-aware lead +5 deg trim, alt-ref -0.207, camera acquisition gated
# shut = open-loop sprint) against the 9 m/s line-9 crosser on the
# quad_enemy world, with the onboard camera recording. VIDEO CAPTURE ONLY:
# single flight, no claim beyond its own logged CPA.
# Usage: bash scripts/video/sprint_capture.sh <outdir> [cue_seed]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${1:?outdir}"; SEED="${2:-456123}"
READY="Startup script returned successfully"
TOPIC=/world/quad_enemy/model/x500_mono_cam_0/link/camera_link/sensor/imager/image
mkdir -p "$OUT"

bash "$REPO/scripts/sim_kill.sh" >/dev/null 2>&1; sleep 2
bash "$REPO/scripts/demo_boot_sim.sh" "$OUT/sim.log" quad_enemy >/dev/null
ok=0
for _ in $(seq 1 60); do
    grep -q "$READY" "$OUT/sim.log" 2>/dev/null && { ok=1; break; }; sleep 2
done
[ "$ok" -ne 1 ] && { echo "[sprint_capture] BOOT FAILED"; exit 1; }
sleep 5

setsid "$REPO/.venv/bin/python" "$REPO/scripts/demo_capture_frames.py" \
    --topic "$TOPIC" --out "$OUT/onboard_frames" --timeout 300 \
    > "$OUT/cap.log" 2>&1 &
CAP=$!

env MARKERLESS_NN_WEIGHTS="$REPO/scripts/seeker/weights/drone_finetuned_quad_v2.onnx" \
    INTERCEPTOR_WORLD_NAME=quad_enemy INTERCEPTOR_TARGET_MODEL=fpv_quad_enemy \
timeout 300 "$REPO/.venv-seeker/bin/python" "$REPO/scripts/m4_intercept.py" \
    --law pronav --seeker markerless \
    --target-start=6.5,-14,0.5 --target-vel=0,9 --cue-seed "$SEED" \
    --coded-dash --fpv --dash-unclamp --dash-speed 16 \
    --dash-accel-aware-lead --dash-heading-err-deg 5 \
    --coded-dash-acquire-range-min 999 --alt-ref-offset-m -0.207 \
    2>&1 | tee "$OUT/run.log" | grep -E "M4_RESULT|BREAKOFF|FAIL|dash" | tail -6

kill -TERM "$CAP" 2>/dev/null; sleep 2
bash "$REPO/scripts/sim_kill.sh" >/dev/null 2>&1
grep -h "M4_RESULT" "$OUT/run.log"
