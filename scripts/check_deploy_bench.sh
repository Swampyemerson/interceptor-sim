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
# -> clean offboard.stop(). That path contains NO arm / takeoff / land (those are
# the coded dash's job on the real vehicle, and are SYNTHESISED only by the SITL
# smoke's `smoke=True` bookends). We deliberately do NOT use --sitl-smoke here
# precisely because that path arms. The DISARMED precondition is no longer merely
# assumed from the absence of an arm call: the run passes --require-disarmed, so
# seeker_loop refuses to enter OFFBOARD unless the FC reports armed=False and
# aborts if it arms mid-run.
#
# THE DETECTOR IS DELIBERATELY SYNTHETIC HERE [CHANGED 2026-07-26]
# ----------------------------------------------------------------
# This gate certifies a LINK, so its verdict must not depend on perception. It
# used to run the real ONNX detector over replayed frames, and -- measured -- the
# first setpoint of a default run came from a HALLUCINATED detection on a frame
# with no target in it. Symmetrically, on a run where that false fire does not
# happen the identical, correctly-wired hardware produces zero setpoints and the
# gate prints FAIL. Detector noise was choosing the verdict. So the run now passes
# --synthetic-detector: SmokeSeeker + blank frames, no camera / weights /
# onnxruntime, setpoint count DETERMINISTIC, `smoke` still False so nothing arms.
# Set BENCH_SYNTHETIC=0 to run the real detector instead -- that is a PERCEPTION
# run, and its result does NOT belong to brn-05.
#
# WHAT "OFFBOARD ACCEPTANCE" NEEDS ON THE BENCH (verified here)
# ------------------------------------------------------------
#   1. A live MAVLink heartbeat over the real serial port (connect succeeds).
#   2. Own-state EKF streaming (attitude quat + yaw + rel-altitude) -- the loop's
#      only non-camera input; proves the companion link is bidirectional. The run
#      FAILS unless the own-state stamp ADVANCED across the whole stream, which is
#      the only proof bytes came BACK over the wire (setpoint counts cannot show
#      it: they count messages the Pi QUEUED to mavsdk_server, and the server
#      re-sends the last setpoint on its own timer into a dead port just as
#      happily). It also fails on a mid-run connection_state drop and on own-state
#      going stale past a 2 s bench liveness bound.
#   3. OFFBOARD mode ENTERED while DISARMED -> drone.offboard.start() succeeds.
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
#      NOT A PRECONDITION: the ">=2 Hz for >=1 s" figure in the PX4 offboard docs.
#      seeker_loop primes exactly ONE zero setpoint and calls start() on the very
#      next await; the mode switch is accepted because the vehicle is DISARMED,
#      not because a stream was established first. That figure governs ARMING in
#      offboard and HOLDING it (COM_OF_LOSS_T), neither of which happens here.
#   4. A setpoint stream that is actually MEASURED, post-switch, and gated
#      [ADDED 2026-07-26 -- previously it was reported and not gated, and the only
#      criterion was "more than zero", so ONE setpoint passed while the line above
#      the PASS read "mean cadence 0.0 Hz"]. seeker_loop now fails on: <2 setpoints
#      (cadence UNDEFINED, never 0.0), span < _STREAM_MIN_SPAN_S (1.0 s), mean
#      below _STREAM_MIN_HZ (2.0 Hz), or ANY inter-setpoint gap >=
#      _STREAM_MAX_GAP_S (1.0 s = PX4's COM_OF_LOSS_T default -- offboard
#      availability is a RECENCY test, so the GAP is the binding number, not the
#      mean). Frames / detections / setpoints / max gap / max own_age / first-lock
#      frame are all printed, so a shrinking denominator is counted, not absorbed.
#      NOTE the printed cadence is the rate the Pi COMPUTED setpoints at; the
#      MAVLink wire rate is mavsdk_server's own resend rate and is not measured
#      here.
#      PASS = seeker_loop exits 0 (all of the above + a clean, ACKed stop).
#
# WHAT IS DEFERRED TO THE AIRFRAME (NOT tested here)
# --------------------------------------------------
#   Arming, actual OFFBOARD control authority (motors responding to setpoints),
#   takeoff / land, ESC/DShot, battery monitoring, control-allocation geometry,
#   and MC PID tuning -- all meaningless without the physical frame. bench.params
#   lists these in its "WAITS FOR THE AIRFRAME" block.
#
# KILL SWITCH (brn-05 "kill switch honored") -- HALF OF IT IS DEFERRED
# --------------------------------------------------------------------
#   Flipping the mapped RC kill switch (RC_MAP_KILL_SW, which Radio calibration
#   does NOT set -- you set it by hand, README step 4) is honored at any time.
#   OBSERVABLE ON THIS DISARMED BENCH: QGC logs "Kill engaged" / "Kill disengaged"
#   and actuator_armed.manual_lockdown toggles in the ULog (that field is named
#   actuator_armed.kill on PX4 v1.17+). That is the whole switch->FC path, and it
#   is what this step can certify.
#   NOT OBSERVABLE HERE: the COM_KILL_DISARM auto-disarm. v1.16.0
#   Commander::handleAutoDisarm() runs that block only inside `if (isArmed())`, so
#   on a never-armed bench it is structurally unreachable -- it is DEFERRED to the
#   first props-off ARM test on the real frame. [CORRECTED 2026-07-26: this block
#   used to assert "auto-disarms after COM_KILL_DISARM seconds" as an expected
#   bench observation.] manual_lockdown LATCHES: flip the switch BACK and confirm
#   "Kill disengaged" before moving on, or the next arm test will refuse.
#   It is an operator action, not something this script drives.
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
# WHICH LINK DID YOU ACTUALLY TEST? (brn-05 is about ONE of them)
# ---------------------------------------------------------------
# The documented USB-C fallback (BENCH_DEVICE=/dev/ttyACM0) talks to PX4's USB
# CDC-ACM MAVLink instance, started by rcS/cdcacm_autostart with its OWN
# arguments. MAV_1_CONFIG / MAV_1_FLOW_CTRL / SER_TEL2_BAUD are not on that code
# path at all, BENCH_BAUD is ignored on CDC-ACM (the line rate is meaningless on
# USB), and the 3-wire floating-CTS failure mode MAV_1_FLOW_CTRL=0 exists to
# prevent is unreachable. A clean run there proves seeker_loop's MAVSDK plumbing
# and NOTHING about the link brn-05 exists to retire. So the script classifies
# the resolved endpoint and refuses to print the bare PASS string for anything
# but the TELEM2 UART:
#   serial://<non-ttyACM>  -> TELEM2 UART   -> PASS, brn-05 satisfied
#   serial://ttyACM*       -> PX4 USB CDC   -> PARTIAL, exit 3, brn-05 STAYS OPEN
#   udp*:// / tcp*://      -> not the bench link -> PARTIAL, exit 3
# Override with BENCH_LINK=telem2 only if you know the endpoint really is TELEM2
# (e.g. a USB-serial adapter into the TELEM2 JST-GH, which presents as ttyUSB*
# and is classified as TELEM2 already).
#
# Usage:
#   scripts/check_deploy_bench.sh --check-only   # NO hardware: validate plumbing + files, exit 0/1
#   scripts/check_deploy_bench.sh                # REAL run against the bench FC (needs the 6C Mini wired)
#   scripts/check_deploy_bench.sh -h             # help
#
# Env (all optional; defaults shown):
#   BENCH_DEVICE       /dev/ttyAMA0                        # Pi UART to TELEM2 (USB fallback: /dev/ttyACM0 -- see the link block above)
#   BENCH_BAUD         921600                              # must match SER_TEL2_BAUD. IGNORED on a ttyACM* (USB CDC) endpoint.
#   BENCH_MAVSDK_URL   serial://$BENCH_DEVICE:$BENCH_BAUD  # full override wins (e.g. udpin://0.0.0.0:14540)
#   BENCH_LINK         <auto from the URL>                 # telem2 | usb | other -- forces the classification above
#   BENCH_PYTHON       <repo>/.venv-seeker/bin/python      # needs mavsdk (and onnxruntime only when BENCH_SYNTHETIC=0)
#   BENCH_SYNTHETIC    1                                   # 1 = deterministic SmokeSeeker (the LINK test). 0 = the real ONNX detector (a PERCEPTION run; not brn-05).
#   BENCH_SOURCE       <repo>/scripts/seeker/data/quad_approach/images  # only used when BENCH_SYNTHETIC=0 (or 'picamera' on the Pi)
#   BENCH_WEIGHTS      <repo>/scripts/seeker/weights/nn_tier/n-mono.onnx
#                      = flight/deploy/seeker_loop.py's OWN default (the REAL-DATA
#                      model). CORRECTED 2026-07-25: this defaulted to the
#                      sim-trained drone_finetuned_quad_v2.onnx long after the
#                      flight default moved, so the gate meant to CERTIFY the
#                      deployed loop was scoring a model that does not fly --
#                      and quad_v2 reads AP50 0.0003 / recall 1.1% / false-fire
#                      88.5% on real held-out imagery. Set BENCH_WEIGHTS
#                      explicitly to re-run the historical bar.
#   BENCH_INTRINSICS   <repo>/configs/camera_intrinsics.json       # ON HARDWARE use the calibrate_camera.py output for the real lens
#   BENCH_MOUNT_TILT_DEG 0                                 # fixed camera up-tilt of the real mount (deg)
#   BENCH_FPS          20
#   BENCH_MAX_FRAMES   300                                 # bound the stream (~15 s at 20 fps). Now actually honoured by run_mavsdk.
#
# Exit: 0 = PASS, 3 = ran clean but NOT on the brn-05 link (PARTIAL), other
#       non-zero = FAIL (propagates seeker_loop.py's exit code, or a
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
BENCH_INTRINSICS="${BENCH_INTRINSICS:-$REPO_ROOT/configs/camera_intrinsics.json}"
BENCH_MOUNT_TILT_DEG="${BENCH_MOUNT_TILT_DEG:-0}"
BENCH_FPS="${BENCH_FPS:-20}"
BENCH_MAX_FRAMES="${BENCH_MAX_FRAMES:-300}"
BENCH_SYNTHETIC="${BENCH_SYNTHETIC:-1}"

SEEKER_MODULE="$REPO_ROOT/flight/deploy/seeker_loop.py"
PARAMS_FILE="$REPO_ROOT/configs/px4_6cmini/bench.params"
README_FILE="$REPO_ROOT/configs/px4_6cmini/README.md"
LOGS_DIR="$REPO_ROOT/logs"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_LOG="$LOGS_DIR/deploy_seeker_bench_${TIMESTAMP}.log"
# Outer wall bound. The stream itself is BENCH_MAX_FRAMES / BENCH_FPS = ~15 s at
# the defaults (that bound is now honoured -- it used to be inert on the MAVSDK
# path, so the run consumed the whole source), plus connect (<=30 s) + own-state
# wait (<=60 s) + teardown. 180 s leaves margin for a slower detector on the Pi
# when BENCH_SYNTHETIC=0; the synthetic run does no inference at all.
PY_TIMEOUT_S="${PY_TIMEOUT_S:-180}"

# Print the whole comment header (stop at the first non-comment line) instead of
# a hard-coded line range: the range was '2,60p' while the header ran to line 90,
# so -h silently truncated before the Usage/Env/Exit block -- the only part an
# operator at the bench needs. This form cannot drift again.
usage() {
    awk 'NR>1 { if ($0 ~ /^#/) { sub(/^# ?/, ""); print; next } exit }' \
        "${BASH_SOURCE[0]}"
}

# ------------------------------------------------------------------ helpers
# The command the real run WOULD execute (also printed by --check-only). Keep it
# in ONE place: --check-only greps these flags against seeker_loop's real parser.
planned_cmd() {
    printf '%s\n' \
      "$BENCH_PYTHON -m flight.deploy.seeker_loop" \
      "    ${SOURCE_ARGS}" \
      "    --mavsdk-url $BENCH_MAVSDK_URL" \
      "    --intrinsics $BENCH_INTRINSICS" \
      "    --mount-tilt-deg $BENCH_MOUNT_TILT_DEG --fps $BENCH_FPS --max-frames $BENCH_MAX_FRAMES" \
      "    --require-disarmed"
}

# The perception half of the command line: the LINK test uses the deterministic
# synthetic detector (no --source/--weights at all); BENCH_SYNTHETIC=0 swaps in
# the real ONNX detector over replayed frames.
if [[ "$BENCH_SYNTHETIC" == "1" ]]; then
    SOURCE_ARGS="--synthetic-detector"
else
    SOURCE_ARGS="--source $BENCH_SOURCE --weights $BENCH_WEIGHTS"
fi

# ---- which link is this? (see the header block) -----------------------------
# Only /dev/ttyACM* is reliably identifiable as the FC's own USB CDC endpoint.
# Everything else on a serial:// URL is treated as TELEM2 (a USB-serial adapter
# into the TELEM2 JST-GH legitimately presents as ttyUSB*), and a udp/tcp URL is
# not the bench link at all.
classify_link() {
    if [[ -n "${BENCH_LINK:-}" ]]; then
        echo "$BENCH_LINK"; return
    fi
    case "$BENCH_MAVSDK_URL" in
        serial://*)
            local dev="${BENCH_MAVSDK_URL#serial://}"; dev="${dev%%:*}"
            dev="$(readlink -f "$dev" 2>/dev/null || echo "$dev")"
            case "$dev" in
                /dev/ttyACM*) echo "usb" ;;
                *)            echo "telem2" ;;
            esac ;;
        *) echo "other" ;;
    esac
}

# Validate the MAVSDK URL plumbing WITHOUT touching hardware.
validate_url() {
    case "$BENCH_MAVSDK_URL" in
        serial://*:[0-9]*) return 0 ;;      # serial:///dev/ttyAMA0:921600
        udpin://*:[0-9]* | udpout://*:[0-9]* | tcpin://*:[0-9]* | tcpout://*:[0-9]*) return 0 ;;
        *) echo "[check_deploy_bench]   BAD url: '$BENCH_MAVSDK_URL' (want serial://<dev>:<baud> or udpin://<host>:<port>)"; return 1 ;;
    esac
}

# Every FILE / PATH the real run dereferences. Factored out of --check-only and
# run on the REAL path too (2026-07-26): a typo'd BENCH_SOURCE used to be caught
# only by --check-only, so on the real run it surfaced as "no setpoints were
# produced" AFTER a successful connect -- which reads as a perception or FC
# problem and sends the operator back to the TX/RX wiring. Returns 0/1.
check_referenced_files() {
    local fail=0 f
    local files=("$SEEKER_MODULE" "$PARAMS_FILE" "$README_FILE" "$BENCH_INTRINSICS")
    [[ "$BENCH_SYNTHETIC" == "1" ]] || files+=("$BENCH_WEIGHTS")
    for f in "${files[@]}"; do
        if [[ -f "$f" ]]; then
            echo "[check_deploy_bench]   OK  file: $f"
        else
            echo "[check_deploy_bench]   MISSING file: $f"; fail=1
        fi
    done
    # BENCH_SOURCE is only dereferenced when the real detector is in play. It may
    # be a dir, an IMAGE file, or the literal 'picamera'. `-e` alone accepted an
    # existing non-image (a .txt), which yields the same silent zero-frame run.
    if [[ "$BENCH_SYNTHETIC" != "1" ]]; then
        if [[ "$BENCH_SOURCE" == "picamera" || -d "$BENCH_SOURCE" ]]; then
            echo "[check_deploy_bench]   OK  source: $BENCH_SOURCE"
        elif [[ -f "$BENCH_SOURCE" && "$BENCH_SOURCE" =~ \.(png|jpg|jpeg|PNG|JPG|JPEG)$ ]]; then
            echo "[check_deploy_bench]   OK  source: $BENCH_SOURCE"
        else
            echo "[check_deploy_bench]   BAD source: $BENCH_SOURCE"
            echo "[check_deploy_bench]     (want an image dir, a .png/.jpg file, or the literal 'picamera')"
            fail=1
        fi
    fi
    return "$fail"
}

# PRODUCER->CONSUMER CONTRACT: every long flag this script passes must exist in
# seeker_loop's REAL parser. A rename would otherwise surface only at the bench,
# with the FC wired, as an argparse exit 2. Grepped out of planned_cmd so the
# check cannot fall out of sync with the command it is validating.
check_cli_flags() {
    local help fail=0 flag
    help="$(cd "$REPO_ROOT" && COLUMNS=200 "$BENCH_PYTHON" \
            -m flight.deploy.seeker_loop --help 2>&1)" || {
        echo "[check_deploy_bench]   SKIP flag contract: seeker_loop --help did not run"
        return 1; }
    for flag in $(planned_cmd | grep -oE '(^|[[:space:]])--[a-z0-9-]+' | tr -d ' '); do
        if ! printf '%s' "$help" | grep -q -- "$flag"; then
            echo "[check_deploy_bench]   MISSING flag in seeker_loop's parser: $flag"
            fail=1
        fi
    done
    [[ "$fail" -eq 0 ]] && echo "[check_deploy_bench]   OK  all passed flags exist in seeker_loop --help"
    return "$fail"
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

    # 1) MAVSDK URL plumbing + which link it resolves to.
    echo "[check_deploy_bench] resolved MAVSDK url: $BENCH_MAVSDK_URL"
    validate_url || fail=1
    echo "[check_deploy_bench] link classification: $(classify_link) (telem2 = brn-05)"

    # 2/3) Referenced FILES + the frame source.
    check_referenced_files || fail=1

    # 4) BENCH_PYTHON interpreter present (needs mavsdk at RUN time, plus
    #    onnxruntime only when BENCH_SYNTHETIC=0).
    if [[ -x "$BENCH_PYTHON" ]]; then
        echo "[check_deploy_bench]   OK  interpreter: $BENCH_PYTHON"
        check_cli_flags || fail=1
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

LINK="$(classify_link)"
echo "[check_deploy_bench] link: $LINK  (only 'telem2' can close brn-05 -- see -h)"

# Pre-flight: if it is a serial:// URL, the device node must exist, else MAVSDK
# just hangs. (USB fallback: BENCH_DEVICE=/dev/ttyACM0 -- baud is ignored there.)
if [[ "$BENCH_MAVSDK_URL" == serial://* ]]; then
    dev="${BENCH_MAVSDK_URL#serial://}"; dev="${dev%%:*}"
    if [[ ! -e "$dev" ]]; then
        echo "[check_deploy_bench] FAIL: serial device '$dev' not found."
        echo "[check_deploy_bench]   Wire TELEM2<->Pi (see configs/px4_6cmini/README.md), free the Pi console,"
        echo "[check_deploy_bench]   or set BENCH_DEVICE / BENCH_MAVSDK_URL. USB fallback: /dev/ttyACM0."
        exit 2
    fi
fi
if [[ ! -x "$BENCH_PYTHON" ]]; then
    echo "[check_deploy_bench] FAIL: interpreter not found/exec: $BENCH_PYTHON"
    echo "[check_deploy_bench]   (needs mavsdk; plus onnxruntime only when BENCH_SYNTHETIC=0)"
    exit 2
fi
# The SAME file/source preflight --check-only runs, BEFORE the serial port is
# opened -- so a typo'd path is unambiguously a source problem, not an FC one.
if ! check_referenced_files; then
    echo "[check_deploy_bench] FAIL: a referenced file/path is missing (above)."
    echo "[check_deploy_bench]   This is a CONFIG problem, not a link problem -- the FC was never contacted."
    exit 2
fi

{
    echo "=== deploy_seeker BENCH (props-off, no-arm OFFBOARD) @ ${TIMESTAMP} (UTC) ==="
    echo "host: $(uname -sr)  |  python: $BENCH_PYTHON"
    echo "url:  $BENCH_MAVSDK_URL   link: $LINK"
    echo "detector: $([[ "$BENCH_SYNTHETIC" == "1" ]] && echo "SYNTHETIC (deterministic; this is a LINK test)" || echo "REAL ONNX over $BENCH_SOURCE (a PERCEPTION run; does not decide brn-05)")"
    echo "note: NO arm/takeoff/land, --require-disarmed enforced -- verifies connect + own-state round trip + OFFBOARD switch + a MEASURED setpoint stream, DISARMED."
    echo "------------------------------------------------------------------"
} | tee "$RUN_LOG"

# Run from the repo root so `-m flight.deploy.seeker_loop` resolves the package.
cd "$REPO_ROOT" || { echo "[check_deploy_bench] FAIL: cannot cd $REPO_ROOT"; exit 2; }

# shellcheck disable=SC2086  # SOURCE_ARGS is a deliberate multi-word flag set
timeout "$PY_TIMEOUT_S" "$BENCH_PYTHON" -m flight.deploy.seeker_loop \
    $SOURCE_ARGS \
    --mavsdk-url "$BENCH_MAVSDK_URL" \
    --intrinsics "$BENCH_INTRINSICS" \
    --mount-tilt-deg "$BENCH_MOUNT_TILT_DEG" \
    --fps "$BENCH_FPS" \
    --max-frames "$BENCH_MAX_FRAMES" \
    --require-disarmed 2>&1 | tee -a "$RUN_LOG"
PY_EXIT="${PIPESTATUS[0]}"

echo "------------------------------------------------------------------" | tee -a "$RUN_LOG"
if [[ "$PY_EXIT" -eq 124 ]]; then
    echo "[check_deploy_bench] FAIL: seeker_loop hit the ${PY_TIMEOUT_S}s outer timeout." | tee -a "$RUN_LOG"
    exit 1
fi
if [[ "$PY_EXIT" -ne 0 ]]; then
    echo "[check_deploy_bench] FAIL (seeker_loop exit $PY_EXIT; see $RUN_LOG)" | tee -a "$RUN_LOG"
    exit "$PY_EXIT"
fi
# The run was clean. Was it clean on the link brn-05 is about? A verdict that
# does not say so is how a green gate certifies the one link still unverified.
if [[ "$LINK" != "telem2" ]]; then
    echo "[check_deploy_bench] PARTIAL (link=$LINK, not the TELEM2 UART): seeker_loop's MAVSDK" | tee -a "$RUN_LOG"
    echo "[check_deploy_bench]   plumbing is proven, but MAV_1_CONFIG / MAV_1_FLOW_CTRL / SER_TEL2_BAUD" | tee -a "$RUN_LOG"
    echo "[check_deploy_bench]   and the 3-wire harness were NOT on this code path. brn-05 REMAINS OPEN." | tee -a "$RUN_LOG"
    echo "[check_deploy_bench]   (exit 3; see $RUN_LOG)" | tee -a "$RUN_LOG"
    exit 3
fi
if [[ "$BENCH_SYNTHETIC" == "1" ]]; then
    echo "[check_deploy_bench] PASS (TELEM2 UART -- connect + own-state round trip + OFFBOARD" | tee -a "$RUN_LOG"
    echo "[check_deploy_bench]   accepted + a measured setpoint stream, disarmed. brn-05 satisfied;" | tee -a "$RUN_LOG"
    echo "[check_deploy_bench]   perception NOT exercised by design; see $RUN_LOG)" | tee -a "$RUN_LOG"
else
    echo "[check_deploy_bench] PASS (TELEM2 UART, REAL detector -- link + a perception run; the" | tee -a "$RUN_LOG"
    echo "[check_deploy_bench]   setpoint count here DEPENDS on what the detector fired on; see $RUN_LOG)" | tee -a "$RUN_LOG"
fi
exit 0
