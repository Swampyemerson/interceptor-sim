#!/usr/bin/env bash
# PROBE 1 orchestrator: boot quad_enemy (CANONICAL level camera), fly the
# interceptor to a stable PX4 HOLD hover, then teleport the STATIC target through
# a range x background x azimuth x attitude grid while capture_flight_frames.py
# rides along, and score offline. See ground_background_sweep.py's docstring for
# the geometry + honesty caveats (headline next_probe / detector stage note).
#
# ONE sim, project law. Boot grep sim-paced (ADR-0004). Kill logic in-file via
# scripts/sim_kill.sh (self-kill rule). STATIC set-pose captures, not
# timing-sensitive, so the ADR-0015 load confound does NOT bind (other CPU-only
# agents may be active) -- rerun at idle if render anomalies appear. GPU render ON:
# it corrupts FIDUCIAL decode only; this is markerless NN (no tags), so it's fine.
#
# Env knobs (for a quick validation): BACKGROUNDS="ground" ATTITUDES="level"
# RANGES="8 12" GBG_HOLD=4.
set -uo pipefail
cd "$(dirname "$0")/../.."
REPO_ROOT="$PWD"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="$REPO_ROOT/.venv/bin/python"
VENV_SEEKER="$REPO_ROOT/.venv-seeker/bin/python"
WORLD=quad_enemy
TARGET_MODEL=fpv_quad_enemy
ALT="${GBG_ALT:-9.0}"
BACKGROUNDS="${BACKGROUNDS:-sky horizon ground}"
ATTITUDES="${ATTITUDES:-level banked}"
RANGES="${RANGES:-6 8 10 12 16 20}"
EVERY="${GBG_EVERY:-5}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="$REPO_ROOT/logs/ground_background_sweep"
SIM_LOG="$LOG_DIR/sim_${STAMP}.log"
TO_LOG="$LOG_DIR/takeoff_${STAMP}.log"
DATA_ROOT="$REPO_ROOT/scripts/seeker/data"
mkdir -p "$LOG_DIR"
READY_STR="Startup script returned successfully"; READY_TIMEOUT_S=160; SETTLE_S=6
source "$REPO_ROOT/scripts/sim_gpu_render.sh" 2>/dev/null || true

# CANONICAL camera: make sure no uptilt/downtilt shadow is present.
rm -f "$REPO_ROOT/models/mono_cam"

TAKEOFF_PID=""
teardown() {
    [ -n "$TAKEOFF_PID" ] && kill -TERM "$TAKEOFF_PID" 2>/dev/null
    pkill -f "capture_flight_frames.py" 2>/dev/null
    bash "$REPO_ROOT/scripts/sim_kill.sh" 2>/dev/null
    sleep 3
}
trap teardown EXIT

echo "[gbg] boot $WORLD (canonical camera) ..."
bash "$REPO_ROOT/scripts/sim_kill.sh" 2>/dev/null; sleep 2
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=$WORLD GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" > "$SIM_LOG" 2>&1 &
ready=0
for ((i=0;i<READY_TIMEOUT_S;i++)); do grep -q "$READY_STR" "$SIM_LOG" 2>/dev/null && { ready=1; break; }; sleep 1; done
[[ "$ready" -ne 1 ]] && { echo "[gbg] FAIL sim-ready (see $SIM_LOG)"; exit 1; }
sleep "$SETTLE_S"

echo "[gbg] takeoff+hold to ${ALT} m ..."
setsid "$VENV" "$REPO_ROOT/scripts/seeker/ground_background_sweep.py" --takeoff "$ALT" > "$TO_LOG" 2>&1 &
TAKEOFF_PID=$!
held=0
for ((i=0;i<120;i++)); do
    grep -q "HOLDING" "$TO_LOG" 2>/dev/null && { held=1; break; }
    grep -qE "FAIL|Traceback|Error" "$TO_LOG" 2>/dev/null && { echo "[gbg] takeoff error:"; tail -5 "$TO_LOG"; exit 1; }
    kill -0 "$TAKEOFF_PID" 2>/dev/null || { echo "[gbg] takeoff died:"; tail -8 "$TO_LOG"; exit 1; }
    sleep 1
done
[[ "$held" -ne 1 ]] && { echo "[gbg] FAIL: no HOLD within timeout"; tail -8 "$TO_LOG"; exit 1; }
grep "HOLDING" "$TO_LOG" | tail -1
sleep 4   # let the hover settle

for att in $ATTITUDES; do
    ROOT="$DATA_ROOT/ground_bg_sweep"; [ "$att" = "banked" ] && ROOT="$DATA_ROOT/ground_bg_sweep_banked"
    for bg in $BACKGROUNDS; do
        OUT="$ROOT/$bg"; mkdir -p "$OUT"
        REC_LOG="$LOG_DIR/rec_${att}_${bg}_${STAMP}.log"
        echo "[gbg] ===== attitude=$att background=$bg -> $OUT ====="
        INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
            "$VENV_SEEKER" "$REPO_ROOT/scripts/seeker/capture_flight_frames.py" \
            --out "$OUT" --run-tag "${bg}_${att}" --every "$EVERY" --extent-m 0.9 \
            --max-frames 6000 > "$REC_LOG" 2>&1 &
        REC_PID=$!
        sleep 3
        kill -0 "$REC_PID" 2>/dev/null || { echo "[gbg] recorder died $att/$bg"; tail -5 "$REC_LOG"; exit 1; }
        INTERCEPTOR_WORLD_NAME=$WORLD INTERCEPTOR_TARGET_MODEL=$TARGET_MODEL \
            "$VENV" "$REPO_ROOT/scripts/seeker/ground_background_sweep.py" --sweep \
            --background "$bg" --attitude "$att" --world "$WORLD" --target "$TARGET_MODEL" \
            --ranges $RANGES
        kill -TERM "$REC_PID" 2>/dev/null
        for ((i=0;i<15;i++)); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done
        kill -KILL "$REC_PID" 2>/dev/null
        echo "[gbg]   $att/$bg frames: $(ls "$OUT/images" 2>/dev/null | wc -l)"
    done
done

echo "[gbg] teardown sim + hover"
teardown
trap - EXIT

LEVEL_CSV="$REPO_ROOT/logs/ground_background_sweep_level.csv"
BANKED_CSV="$REPO_ROOT/logs/ground_background_sweep_banked.csv"
echo "[gbg] === analyze (level) ==="
"$VENV_SEEKER" "$REPO_ROOT/scripts/seeker/ground_background_sweep.py" --analyze \
    "$DATA_ROOT/ground_bg_sweep" --conf 0.25 \
    --csv "$LEVEL_CSV" \
    --summary-out "$LOG_DIR/summary_level_${STAMP}.json" 2>&1 | grep -v "finetuned\]"
if [ -d "$DATA_ROOT/ground_bg_sweep_banked/ground/images" ] || \
   [ -d "$DATA_ROOT/ground_bg_sweep_banked/sky/images" ]; then
    echo "[gbg] === analyze (banked) ==="
    "$VENV_SEEKER" "$REPO_ROOT/scripts/seeker/ground_background_sweep.py" --analyze \
        "$DATA_ROOT/ground_bg_sweep_banked" --conf 0.25 \
        --csv "$BANKED_CSV" \
        --summary-out "$LOG_DIR/summary_banked_${STAMP}.json" 2>&1 | grep -v "finetuned\]"
fi
echo "[gbg] === plot (under .venv) ==="
"$VENV" "$REPO_ROOT/scripts/seeker/ground_background_sweep.py" --plot \
    --level-csv "$LEVEL_CSV" \
    $( [ -f "$BANKED_CSV" ] && echo --banked-csv "$BANKED_CSV" ) \
    --images-dir "$REPO_ROOT/docs/images"
echo "[gbg] DONE."
