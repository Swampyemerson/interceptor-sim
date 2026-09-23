#!/usr/bin/env bash
# SPEED SCOUTING for the fast intercept video (builder ask 2026-09-23).
# Flies the classic tag M4 scenario at increasing target speeds with the
# onboard camera recording, one fresh boot per flight. n=1 per speed --
# these are VIDEO-SCOUTING LEADS, not verdicts (no claim may quote them
# without a proper paired batch; statistics-before-verdicts rule).
# Usage: bash scripts/video/speed_scout.sh <outroot> <speed1> [speed2 ...]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTROOT="${1:?outroot}"; shift
READY="Startup script returned successfully"
TOPIC=/world/apriltag/model/x500_mono_cam_0/link/camera_link/sensor/imager/image
mkdir -p "$OUTROOT"

for SPD in "$@"; do
    TAG="v${SPD/./p}"
    echo "=== [speed_scout] target ${SPD} m/s ==="
    bash "$REPO/scripts/sim_kill.sh" >/dev/null 2>&1; sleep 2
    SIMLOG="$OUTROOT/sim_$TAG.log"
    bash "$REPO/scripts/demo_boot_sim.sh" "$SIMLOG" >/dev/null
    ok=0
    for _ in $(seq 1 60); do
        grep -q "$READY" "$SIMLOG" 2>/dev/null && { ok=1; break; }; sleep 2
    done
    [ "$ok" -ne 1 ] && { echo "[speed_scout] $TAG BOOT FAILED"; continue; }
    sleep 5
    gz service -s /world/apriltag/set_pose --reqtype gz.msgs.Pose \
        --reptype gz.msgs.Boolean --timeout 2000 \
        --req 'name: "apriltag_target" position { x: 6.5 y: -4 z: 0.5 }' >/dev/null
    FR="$OUTROOT/frames_$TAG"
    setsid "$REPO/.venv/bin/python" "$REPO/scripts/demo_capture_frames.py" \
        --topic "$TOPIC" --out "$FR" --timeout 300 \
        > "$OUTROOT/cap_$TAG.log" 2>&1 &
    CAP=$!
    # SCOUT_EXTRA_ARGS: optional extra m4 flags (e.g. the 2026-09-23
    # speed-envelope overrides); empty = the historical classic config.
    timeout 300 "$REPO/.venv/bin/python" "$REPO/scripts/m4_intercept.py" \
        --law pronav --target-vel "0,$SPD" ${SCOUT_EXTRA_ARGS:-} 2>&1 \
        | tee "$OUTROOT/run_$TAG.log" | grep -E "M4_RESULT|BREAKOFF|FAIL" | tail -3
    kill -TERM "$CAP" 2>/dev/null; sleep 2
    bash "$REPO/scripts/sim_kill.sh" >/dev/null 2>&1; sleep 3
done
echo "=== [speed_scout] results (n=1 leads, not verdicts) ==="
grep -h "M4_RESULT" "$OUTROOT"/run_*.log 2>/dev/null
