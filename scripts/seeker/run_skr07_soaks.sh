#!/usr/bin/env bash
# skr-07 — the three compute-bench arms, run SEQUENTIALLY on the real Pi 5.
#
# Sequential on purpose: two arms sharing the SoC would each measure the other's
# thermal load, and the whole point of the bench is a sustained single-workload
# rate. Same "one at a time" discipline the sim batches use.
#
# Launch detached so an SSH drop cannot kill a 45-minute measurement:
#   setsid nohup scripts/seeker/run_skr07_soaks.sh > runs/skr07/runner.log 2>&1 &
set -uo pipefail
cd "$(dirname "$0")/../.." || exit 1

PY=.venv-pi/bin/python
OUT="${SOAK_OUT:-runs/skr07}"
DUR="${SOAK_DURATION_S:-900}"
WARM="${SOAK_WARMUP_S:-300}"
mkdir -p "$OUT"

arm() {
    local name="$1"; shift
    echo "=== [$(date -Is)] ARM $name ==="
    $PY scripts/seeker/pi_fps_soak.py --duration-s "$DUR" --warmup-s "$WARM" \
        --out "$OUT/$name" "$@" 2>&1 | grep -v "^\["
    echo "=== [$(date -Is)] ARM $name done (exit ${PIPESTATUS[0]}) ==="
    sleep 30      # let the SoC settle so the next arm's warm-up is comparable
}

# 1. qd=2.0 uncapped — the headline candidate. quad_decimate 2.0 is the flying
#    default (pi_capture.py DEFAULT_QUAD_DECIMATE); --target-fps lifts the
#    picamera2 frame-duration cap that pinned delivery to 30 fps.
arm qd2_uncapped --mode apriltag --quad-decimate 2.0 --target-fps 143

# 2. qd=1.0 uncapped — ADR-0082's reclaim lever: buys incidence cone / range,
#    costs fps. The gate must be quoted at the decimate actually flown, so both
#    numbers have to exist (protocol §4.2b).
arm qd1_uncapped --mode apriltag --quad-decimate 1.0 --target-fps 143

# 3. CPU-YOLO on the DEPLOYED weights (n-mono.onnx, not the blind-sim quad_v2).
#    This is the evidence for or against the deferred Hailo HAT.
arm yolo_cpu --mode yolo --weights scripts/seeker/weights/nn_tier/n-mono.onnx --target-fps 143

echo "=== [$(date -Is)] ALL ARMS COMPLETE ==="
