#!/usr/bin/env bash
# check_deploy_bench.sh -- PROPS-OFF bench sibling of scripts/check_deploy_sitl.sh.
# Where check_deploy_sitl.sh drives flight/deploy/seeker_loop.py against a LOCAL
# PX4 SITL over UDP, this drives the SAME loop against the REAL Pixhawk 6C Mini
# over a REAL serial link -- the interceptor "brain bench" (docs/project_state.json
# build_tab subsystem `brain`, gate step brn-05). It retires the deploy path's one
# link the sim never exercised: MAVSDK OFFBOARD over a physical UART.
#
# WHAT THIS RUNS (and why it does NOT arm)
# ----------------------------------------
# It runs seeker_loop's VEHICLE OFFBOARD path -- the exact code the flying Pi runs
# (run_mavsdk(smoke=False)): connect -> stream own-state EKF -> prime a setpoint ->
# OFFBOARD mode switch (drone.offboard.start()) -> stream pro-nav velocity setpoints
# from real detections -> clean offboard.stop(). That path contains NO arm / takeoff
# / land (those are the coded dash's job on the real vehicle, and are SYNTHESISED
# only by the SITL smoke's `smoke=True` bookends). So on the bench the FC stays
# DISARMED the whole time -- nothing can spin, and there is no airframe anyway.
# We deliberately do NOT use --sitl-smoke here precisely because that path arms.
#
# WHAT "OFFBOARD ACCEPTANCE" NEEDS ON THE BENCH (verified here)
# ------------------------------------------------------------
#   1. A live MAVLink heartbeat over the real serial port (connect succeeds).
#   2. Own-state EKF streaming (attitude quat + yaw + rel-altitude) -- the loop's
#      only non-camera input; proves the companion link is bidirectional.
#   3. A setpoint stream established >=2 Hz for >=1 s BEFORE the mode switch.
#      NO GPS FIX IS REQUIRED FOR THIS BENCH [CORRECTED 2026-07-26 -- this comment
#      previously claimed a fix was mandatory, and the same false claim was in
#      bench.params + configs/px4_6cmini/README.md]. Two independent reasons, both
#      verified against the PX4 v1.16.0 source: (a) UserModeIntention::change()
#      -- "Always allow mode change while disarmed", so the health check that
#      would demand a position estimate is not applied disarmed; (b) the vehicle
#      path run_mavsdk(smoke=False) hard-requires only the EKF ATTITUDE stream --
#      the global-position wait lives in the SITL-only smoke branch. COM_ARM_WO_GPS=0
#      is about ARMING (it is the GPS preflight-check action in v1.16), and nothing
#      on this bench ever arms. Run it on the desk, no GPS module attached.
#   4. OFFBOARD mode ENTERED while DISARMED (PX4 allows the mode switch disarmed;
#      it simply does not actuate) -> drone.offboard.start() returns success.
#   5. Setpoints stream continuously over the wire until the frame source ends.
#      PASS = seeker_loop exits 0 (all of the above + a clean stop).
#
# WHAT IS DEFERRED TO THE AIRFRAME (NOT tested here)
# --------------------------------------------------
#   Arming, actual OFFBOARD control authority (motors responding to setpoints),
#   takeoff / land, ESC/DShot, battery monitoring, control-allocation geometry,
#   and MC PID tuning -- all meaningless without the physical frame. bench.params
#   lists these in its "WAITS FOR THE AIRFRAME" block.
#
# KILL SWITCH (brn-05 "kill switch honored")
# ------------------------------------------
#   With bench.params loaded and Radio calibrated, flipping the mapped RC kill
#   switch (RC_MAP_KILL_SW) is honored at any time and auto-disarms after
#   COM_KILL_DISARM seconds. Exercise it by hand during a real bench run and watch
#   QGC / the ULog -- it is an operator action, not something this script drives.
#
# HONESTY (CLAUDE.md #5): the loop's only inputs stay camera pixels + own-state EKF.
# No gt_* exists on real hardware; this script adds none. It just points the same
# gt-free loop at a real serial endpoint.
#
# Boot/shutdown conventions differ from check_deploy_sitl.sh on purpose: there is
# NO sim to launch or kill here -- the "device" is external hardware -- so there is
# no setsid process group and no pkill of px4/gz. (Kill/poll logic would live in a
# script FILE if it were ever needed; it is not, per house rule.)
#
# Usage:
#   scripts/check_deploy_bench.sh --check-only   # NO hardware: validate plumbing + files, exit 0/1
#   scripts/check_deploy_bench.sh                # REAL run against the bench FC (needs the 6C Mini wired)
#   scripts/check_deploy_bench.sh -h             # help
#
# Env (all optional; defaults shown):
#   BENCH_DEVICE       /dev/ttyAMA0                        # Pi UART to TELEM2 (USB fallback: /dev/ttyACM0)
#   BENCH_BAUD         921600                              # must match SER_TEL2_BAUD (57600 over USB-C)
#   BENCH_MAVSDK_URL   serial://$BENCH_DEVICE:$BENCH_BAUD  # full override wins (e.g. udpin://0.0.0.0:14540)
#   BENCH_PYTHON       <repo>/.venv-seeker/bin/python      # needs BOTH mavsdk AND onnxruntime (the vehicle path uses the real detector)
#   BENCH_SOURCE       <repo>/scripts/seeker/data/quad_approach/images  # frames w/ the target in view (or 'picamera' on the Pi)
#   BENCH_WEIGHTS      <repo>/scripts/seeker/weights/nn_tier/n-mono.onnx
#                      = flight/deploy/seeker_loop.py's OWN default (the REAL-DATA
#                      model). CORRECTED 2026-07-25: this defaulted to the
#                      sim-trained drone_finetuned_quad_v2.onnx long after the
#                      flight default moved, so the gate meant to CERTIFY the
#                      deployed loop was scoring a model that does not fly --
#                      and quad_v2 reads AP50 0.0003 / recall 1.1% / false-fire
#                      88.5% on real held-out imagery. Set BENCH_WEIGHTS
#                      explicitly to re-run the historical bar.
#   BENCH_INTRINSICS   <repo>/camera_intrinsics.json       # ON HARDWARE use the calibrate_camera.py output for the real lens
#   BENCH_MOUNT_TILT_DEG 0                                 # fixed camera up-tilt of the real mount (deg)
#   BENCH_FPS          20
#   BENCH_MAX_FRAMES   300                                 # bound the stream so the run terminates
#
# Exit: 0 = PASS, non-zero = FAIL (propagates seeker_loop.py's exit code, or a
#       bench-specific failure code for a missing device / failed --check-only).
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ------------------------------------------------------------------ config (env)
BENCH_DEVICE="${BENCH_DEVICE:-/dev/ttyAMA0}"
BENCH_BAUD="${BENCH_BAUD:-921600}"
BENCH_MAVSDK_URL="${BENCH_MAVSDK_URL:-serial://${BENCH_DEVICE}:${BENCH_BAUD}}"
BENCH_PYTHON="${BENCH_PYTHON:-$REPO_ROOT/.venv-seeker/bin/python}"
BENCH_SOURCE="${BENCH_SOURCE:-$REPO_ROOT/scripts/seeker/data/quad_approach/images}"
BENCH_WEIGHTS="${BENCH_WEIGHTS:-$REPO_ROOT/scripts/seeker/weights/nn_tier/n-mono.onnx}"
BENCH_INTRINSICS="${BENCH_INTRINSICS:-$REPO_ROOT/camera_intrinsics.json}"
BENCH_MOUNT_TILT_DEG="${BENCH_MOUNT_TILT_DEG:-0}"
BENCH_FPS="${BENCH_FPS:-20}"
BENCH_MAX_FRAMES="${BENCH_MAX_FRAMES:-300}"

SEEKER_MODULE="$REPO_ROOT/flight/deploy/seeker_loop.py"
PARAMS_FILE="$REPO_ROOT/configs/px4_6cmini/bench.params"
README_FILE="$REPO_ROOT/configs/px4_6cmini/README.md"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_LOG="$LOGS_DIR/deploy_seeker_bench_${TIMESTAMP}.log"
# Outer wall bound: connect(30) + ~15 s stream + teardown, w/ margin.
PY_TIMEOUT_S="${PY_TIMEOUT_S:-180}"

usage() {
    sed -n '2,60p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

# ------------------------------------------------------------------ helpers
# The command the real run WOULD execute (also printed by --check-only).
planned_cmd() {
    printf '%s\n' \
      "$BENCH_PYTHON -m flight.deploy.seeker_loop" \
      "    --source $BENCH_SOURCE" \
      "    --mavsdk-url $BENCH_MAVSDK_URL" \
      "    --weights $BENCH_WEIGHTS" \
      "    --intrinsics $BENCH_INTRINSICS" \
      "    --mount-tilt-deg $BENCH_MOUNT_TILT_DEG --fps $BENCH_FPS --max-frames $BENCH_MAX_FRAMES"
}

# Validate the MAVSDK URL plumbing WITHOUT touching hardware.
validate_url() {
    case "$BENCH_MAVSDK_URL" in
        serial://*:[0-9]*) return 0 ;;      # serial:///dev/ttyAMA0:921600
        udpin://*:[0-9]* | udpout://*:[0-9]* | tcpin://*:[0-9]* | tcpout://*:[0-9]*) return 0 ;;
        *) echo "[check_deploy_bench]   BAD url: '$BENCH_MAVSDK_URL' (want serial://<dev>:<baud> or udpin://<host>:<port>)"; return 1 ;;
    esac
}

# ------------------------------------------------------------------ arg parse
MODE="run"
case "${1:-}" in
    --check-only) MODE="check" ;;
    -h|--help)    usage; exit 0 ;;
    "")           MODE="run" ;;
    *)            echo "[check_deploy_bench] unknown arg: $1"; echo "try --check-only or -h"; exit 2 ;;
esac

mkdir -p "$LOGS_DIR"

# ================================================================== --check-only
# Self-test: validate this script's OWN argument plumbing + that every referenced
# file exists + that bench.params parses. NO hardware, NO sim, NO MAVSDK connect.
if [[ "$MODE" == "check" ]]; then
    echo "[check_deploy_bench] --check-only: validating argument plumbing + referenced files (no hardware)"
    fail=0

    # 1) MAVSDK URL plumbing.
    echo "[check_deploy_bench] resolved MAVSDK url: $BENCH_MAVSDK_URL"
    validate_url || fail=1

    # 2) Referenced FILES that must exist.
    for f in "$SEEKER_MODULE" "$PARAMS_FILE" "$README_FILE" "$BENCH_WEIGHTS" "$BENCH_INTRINSICS"; do
        if [[ -f "$f" ]]; then
            echo "[check_deploy_bench]   OK  file: $f"
        else
            echo "[check_deploy_bench]   MISSING file: $f"; fail=1
        fi
    done

    # 3) BENCH_SOURCE may be a dir, a file, or the literal 'picamera' (Pi camera).
    if [[ "$BENCH_SOURCE" == "picamera" || -e "$BENCH_SOURCE" ]]; then
        echo "[check_deploy_bench]   OK  source: $BENCH_SOURCE"
    else
        echo "[check_deploy_bench]   MISSING source: $BENCH_SOURCE"; fail=1
    fi

    # 4) BENCH_PYTHON interpreter present (needs mavsdk + onnxruntime at RUN time;
    #    we only check existence here so --check-only stays hardware/dep-free).
    if [[ -x "$BENCH_PYTHON" ]]; then
        echo "[check_deploy_bench]   OK  interpreter: $BENCH_PYTHON"
    else
        echo "[check_deploy_bench]   MISSING/!exec interpreter: $BENCH_PYTHON"; fail=1
    fi

    # 5) bench.params well-formed: every non-comment line is 5 TAB fields with
    #    numeric sysid/compid/value/type and type in {6 INT32, 9 REAL32}.
    if [[ -f "$PARAMS_FILE" ]]; then
        if python3 - "$PARAMS_FILE" <<'PY'
import sys
path = sys.argv[1]
ok = True
n = 0
with open(path) as fh:
    for i, raw in enumerate(fh, 1):
        s = raw.rstrip("\n")
        if not s or s.startswith("#"):
            continue
        parts = s.split("\t")
        if len(parts) != 5:
            print(f"    bench.params line {i}: expected 5 tab fields, got {len(parts)}"); ok = False; continue
        sysid, compid, name, val, typ = parts
        try:
            int(sysid); int(compid); int(typ); float(val)
        except ValueError:
            print(f"    bench.params line {i}: non-numeric field(s): {parts!r}"); ok = False; continue
        if typ not in ("6", "9"):
            print(f"    bench.params line {i}: unexpected MAV_PARAM_TYPE {typ}"); ok = False
        n += 1
if n == 0:
    print("    bench.params: no parameter lines found"); ok = False
print(f"    bench.params: {n} parameter line(s) parsed" + ("" if ok else " -- WITH ERRORS"))
sys.exit(0 if ok else 1)
PY
        then
            echo "[check_deploy_bench]   OK  bench.params parses"
        else
            echo "[check_deploy_bench]   BAD bench.params format"; fail=1
        fi
    fi

    echo "[check_deploy_bench] planned real-run command:"
    planned_cmd | sed 's/^/    /'

    if [[ "$fail" -eq 0 ]]; then
        echo "[check_deploy_bench] --check-only PASS"
        exit 0
    fi
    echo "[check_deploy_bench] --check-only FAIL"
    exit 1
fi

# ================================================================== real bench run
echo "[check_deploy_bench] REAL bench run (props-off, NO arm) against $BENCH_MAVSDK_URL"
echo "[check_deploy_bench] Run output -> $RUN_LOG"

if ! validate_url; then
    echo "[check_deploy_bench] FAIL: bad MAVSDK url"; exit 2
fi

# Pre-flight: if it is a serial:// URL, the device node must exist, else MAVSDK
# just hangs. (USB fallback: BENCH_DEVICE=/dev/ttyACM0 BENCH_BAUD=57600.)
if [[ "$BENCH_MAVSDK_URL" == serial://* ]]; then
    dev="${BENCH_MAVSDK_URL#serial://}"; dev="${dev%%:*}"
    if [[ ! -e "$dev" ]]; then
        echo "[check_deploy_bench] FAIL: serial device '$dev' not found."
        echo "[check_deploy_bench]   Wire TELEM2<->Pi (see configs/px4_6cmini/README.md), free the Pi console,"
        echo "[check_deploy_bench]   or set BENCH_DEVICE / BENCH_MAVSDK_URL. USB fallback: /dev/ttyACM0 @ 57600."
        exit 2
    fi
fi
if [[ ! -x "$BENCH_PYTHON" ]]; then
    echo "[check_deploy_bench] FAIL: interpreter not found/exec: $BENCH_PYTHON"
    echo "[check_deploy_bench]   (needs BOTH mavsdk AND onnxruntime -- the vehicle path uses the real detector)"
    exit 2
fi

{
    echo "=== deploy_seeker BENCH (props-off, no-arm OFFBOARD) @ ${TIMESTAMP} (UTC) ==="
    echo "host: $(uname -sr)  |  python: $BENCH_PYTHON"
    echo "url:  $BENCH_MAVSDK_URL   source: $BENCH_SOURCE"
    echo "note: NO arm/takeoff/land -- verifies connect + own-state + OFFBOARD switch + setpoint stream, DISARMED."
    echo "------------------------------------------------------------------"
} | tee "$RUN_LOG"

# Run from the repo root so `-m flight.deploy.seeker_loop` resolves the package.
cd "$REPO_ROOT" || { echo "[check_deploy_bench] FAIL: cannot cd $REPO_ROOT"; exit 2; }

timeout "$PY_TIMEOUT_S" "$BENCH_PYTHON" -m flight.deploy.seeker_loop \
    --source "$BENCH_SOURCE" \
    --mavsdk-url "$BENCH_MAVSDK_URL" \
    --weights "$BENCH_WEIGHTS" \
    --intrinsics "$BENCH_INTRINSICS" \
    --mount-tilt-deg "$BENCH_MOUNT_TILT_DEG" \
    --fps "$BENCH_FPS" \
    --max-frames "$BENCH_MAX_FRAMES" 2>&1 | tee -a "$RUN_LOG"
PY_EXIT="${PIPESTATUS[0]}"

echo "------------------------------------------------------------------" | tee -a "$RUN_LOG"
if [[ "$PY_EXIT" -eq 124 ]]; then
    echo "[check_deploy_bench] FAIL: seeker_loop hit the ${PY_TIMEOUT_S}s outer timeout." | tee -a "$RUN_LOG"
    exit 1
fi
if [[ "$PY_EXIT" -eq 0 ]]; then
    echo "[check_deploy_bench] PASS (connect + own-state + OFFBOARD + setpoints, disarmed; see $RUN_LOG)" | tee -a "$RUN_LOG"
else
    echo "[check_deploy_bench] FAIL (seeker_loop exit $PY_EXIT; see $RUN_LOG)" | tee -a "$RUN_LOG"
fi
exit "$PY_EXIT"
