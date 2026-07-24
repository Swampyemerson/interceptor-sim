#!/usr/bin/env bash
# check_real_flight_sitl.sh -- FUNCTIONAL check of the REAL interceptor's onboard
# FLIGHT STATE MACHINE (flight/deploy/real_flight.py) against a local PX4 SITL.
# This is docs/launch_mechanism_plan.md build item L1's "done-when":
#
#   headless SITL run: STANDBY hold -> GO -> dash on a commanded heading ->
#   synthetic handoff -> ENGAGE -> breakoff -> SAFE -> land, exit 0.
#
# It is the sibling of scripts/check_deploy_sitl.sh (which drives the TERMINAL
# seeker only, seeker_loop.py --sitl-smoke). Where that one synthesises the
# "already airborne" precondition, THIS one exercises the piece that creates it:
# arm -> takeoff to the standby/loft altitude -> hold OFFBOARD setpoints on the
# commanded aim yaw -> a synthetic GO edge -> the open-loop coded dash -> the
# 5-consecutive-fresh-detection handoff -> the camera terminal -> breakoff ->
# SAFE -> land + disarm.
#
# WHAT IS AND IS NOT PROVEN HERE
#   PROVEN: the state machine's transitions/guards run against a real PX4 (mode
#     acceptance, setpoint cadence, altitude/yaw hold good enough to satisfy the
#     arm gate, clean land+disarm), with a SYNTHETIC trigger and a SYNTHETIC
#     detector -- i.e. the PLUMBING and the SEQUENCE.
#   NOT PROVEN: the real ELRS/RC_CHANNELS trigger path (bench L3), the PWM
#     endpoints and channel map, the geofence/kill-switch interlocks, perception,
#     and every failsafe CONSTANT (all TODO-BUILDER in MissionConfig). This
#     script does not make real_flight.py flight-ready; it makes it reviewable.
#
# FUNCTIONAL check, not a measured gate -- no timing is scored, so the
# "idle-load only" Monte-Carlo rule does not block it.
#
# Boot/shutdown conventions mirror scripts/check_deploy_sitl.sh and
# .claude/skills/px4-gazebo/SKILL.md (setsid process group, `tail -f /dev/null`
# stdin so the pxh console does not flood the log, "Startup script returned
# successfully" as the boot-complete line). pkill patterns live in THIS FILE --
# its cmdline is just the script path, so `pkill -f` cannot self-match (house
# rule; see scripts/sim_kill.sh).
#
# Usage:  scripts/check_real_flight_sitl.sh
# Exit:   0 = PASS (STANDBY->CODED_DASH->ENGAGE->...->SAFE, heading latched once,
#             setpoints streamed, clean land+disarm)
#         non-zero = FAIL (propagates real_flight.py's exit code, or a boot fail).
set -uo pipefail
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"   # mavsdk lives here; onnx NOT needed
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SIM_LOG="$LOGS_DIR/check_real_flight_sim_${TIMESTAMP}.log"
RUN_LOG="$LOGS_DIR/real_flight_sitl_${TIMESTAMP}.log"      # the evidence log
TICK_CSV="$LOGS_DIR/real_flight_sitl_${TIMESTAMP}.csv"     # per-tick telemetry
READY_STRING="Startup script returned successfully"
READY_TIMEOUT_S=120
POST_READY_SETTLE_S=5      # let the EKF settle before MAVSDK connects (check_m3)

# Mission shape for the smoke. The aim heading is deliberately MODEST (25 deg)
# so PX4's yaw controller can reach it and satisfy the +-2 deg arm gate quickly;
# the GO edge is synthesised by --trigger gate-ready the moment that gate holds.
STANDBY_ALT_M="${STANDBY_ALT_M:-7.0}"
DASH_HEADING_DEG="${DASH_HEADING_DEG:-25}"
DASH_SPEED="${DASH_SPEED:-6.0}"        # gentle: this is a plumbing check, not a chase
DASH_MAX_S="${DASH_MAX_S:-10.0}"
ENGAGE_MAX_S="${ENGAGE_MAX_S:-8.0}"
SMOKE_DURATION_S="${SMOKE_DURATION_S:-90}"   # outer bound on the setpoint stream
YAW_TOL_DEG="${YAW_TOL_DEG:-3.0}"      # a hair looser than the 2 deg flight value
# Outer wall bound: connect(30) + health + takeoff + ~60 s mission + land, w/ margin.
PY_TIMEOUT_S="${PY_TIMEOUT_S:-300}"

SIM_PID=""
mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_real_flight] Killing any stale px4 / gz sim processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 2
}

cleanup() {
    echo "[check_real_flight] Cleaning up sim processes..."
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        # setsid => SIM_PID is its own process-group leader; -$SIM_PID targets
        # the whole group (make + px4 + gz sim) without touching this script.
        kill -TERM -- "-$SIM_PID" 2>/dev/null
        sleep 2
        kill -KILL -- "-$SIM_PID" 2>/dev/null
    fi
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    sleep 3
}
trap cleanup EXIT

kill_stale_sim

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[check_real_flight] FAIL: PX4-Autopilot not found at $PX4_DIR"
    exit 1
fi
if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[check_real_flight] FAIL: venv python not found at $VENV_PYTHON"
    exit 1
fi

# Offline gates FIRST -- they cost ~1 s and catch a broken state machine before
# a 2-minute sim boot. Both are hardware/sim-free.
echo "[check_real_flight] Offline pre-gates: honesty audit + pure-logic self-test"
"$VENV_PYTHON" -m flight.deploy.real_flight --audit || {
    echo "[check_real_flight] FAIL: honesty audit"; exit 1; }
"$VENV_PYTHON" -m flight.deploy.real_flight --self-test || {
    echo "[check_real_flight] FAIL: self-test"; exit 1; }

echo "[check_real_flight] Launching HEADLESS PX4 SITL + Gazebo (gz_x500) from $PX4_DIR..."
echo "[check_real_flight] Sim output -> $SIM_LOG"
echo "[check_real_flight] Run  output -> $RUN_LOG"
echo "[check_real_flight] Ticks       -> $TICK_CSV"

# stdin must stay open (tail -f /dev/null) or the pxh console spins on EOF and
# floods the log with prompt reprints (px4-gazebo SKILL.md).
setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env HEADLESS=1 make px4_sitl gz_x500" \
    > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "[check_real_flight] Sim launched (pid/pgid $SIM_PID)."

echo "[check_real_flight] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\"..."
ready=0
elapsed=0
while (( elapsed < READY_TIMEOUT_S )); do
    if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[check_real_flight] FAIL: sim exited early (pid $SIM_PID). See $SIM_LOG"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
if [[ "$ready" -ne 1 ]]; then
    echo "[check_real_flight] FAIL: no \"$READY_STRING\" within ${READY_TIMEOUT_S}s. See $SIM_LOG"
    exit 1
fi

echo "[check_real_flight] Sim ready. Settling ${POST_READY_SETTLE_S}s before MAVSDK connects..."
sleep "$POST_READY_SETTLE_S"

{
    echo "=== real_flight SITL smoke @ ${TIMESTAMP} (UTC) ==="
    echo "host: $(uname -sr)  |  python: $VENV_PYTHON"
    echo "note: FUNCTIONAL check (not a measured gate). Trigger + detector are"
    echo "      SYNTHETIC; the REAL ELRS RC_CHANNELS path is bench item L3."
    echo "sim boot log: $SIM_LOG"
    echo "----------------------------------------------------------------"
} | tee "$RUN_LOG"

echo "[check_real_flight] Driving real_flight.py --sitl-smoke (outer timeout ${PY_TIMEOUT_S}s)..."
timeout "$PY_TIMEOUT_S" "$VENV_PYTHON" -m flight.deploy.real_flight \
    --sitl-smoke --mavsdk-url udpin://0.0.0.0:14540 \
    --trigger gate-ready \
    --dash-heading-deg "$DASH_HEADING_DEG" \
    --standby-alt-m "$STANDBY_ALT_M" \
    --dash-speed "$DASH_SPEED" \
    --dash-max-s "$DASH_MAX_S" \
    --engage-max-s "$ENGAGE_MAX_S" \
    --yaw-tol-deg "$YAW_TOL_DEG" \
    --smoke-duration "$SMOKE_DURATION_S" \
    --log-csv "$TICK_CSV" 2>&1 | tee -a "$RUN_LOG"
PY_EXIT="${PIPESTATUS[0]}"

echo "----------------------------------------------------------------" | tee -a "$RUN_LOG"
if [[ "$PY_EXIT" -eq 124 ]]; then
    echo "[check_real_flight] FAIL: real_flight hit the ${PY_TIMEOUT_S}s outer timeout." | tee -a "$RUN_LOG"
    exit 1
fi
if [[ "$PY_EXIT" -eq 0 ]]; then
    echo "[check_real_flight] PASS (see $RUN_LOG and $TICK_CSV)" | tee -a "$RUN_LOG"
else
    echo "[check_real_flight] FAIL (real_flight exit $PY_EXIT; see $RUN_LOG and $SIM_LOG)" | tee -a "$RUN_LOG"
fi
exit "$PY_EXIT"
