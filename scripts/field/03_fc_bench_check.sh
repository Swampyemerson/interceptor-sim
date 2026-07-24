#!/usr/bin/env bash
# ===========================================================================
# 03_fc_bench_check.sh -- "can my laptop talk to the flight controller?"
# Step 3 of the field bring-up flow (runbook: docs/field_bringup.md).
#
# PROPS OFF, NOTHING SPINS, NOTHING ARMS. This connects over the REAL USB
# serial link, reads identity + telemetry + the health/arming readout, and
# disconnects. The work is done by scripts/field/fc_link_check.py, which parses
# its own source with `ast` at startup and refuses to run if it contains any
# call into MAVSDK's action/offboard/manual_control/param plugins -- so
# "it cannot arm" is mechanically enforced, not a promise in a comment.
#
# WHERE THIS SITS IN THE BUILD (docs/project_state.json build_tab -> `brain`):
#   brn-01/02  flash PX4 + GPS in QGroundControl        (QGC on Windows)
#   brn-04     MAVSDK heartbeat over the link           <-- THIS SCRIPT
#   brn-05     props-off OFFBOARD gate                  scripts/check_deploy_bench.sh
# The SITL rehearsal of the same deploy loop is scripts/check_deploy_sitl.sh.
#
# THE HARDWARE: Pixhawk 6C Mini running PX4 (interceptor brain) or the Kakute
# H7 running ArduPilot (target) -- both appear as a USB CDC-ACM serial port.
# On WSL2 the port only exists after usbipd-win shares it in (docs/field_bringup.md).
#
# DEVICE + BAUD: auto-detects /dev/serial/by-id/* then /dev/ttyACM*. Default
# baud 57600 is the USB fallback documented in scripts/check_deploy_bench.sh
# ("USB fallback: /dev/ttyACM0 @ 57600"); the flight link Pi<->TELEM2 is 921600.
# A USB CDC port ignores baud, but MAVSDK's serial:// URL requires a number.
#
# Usage:
#   scripts/field/03_fc_bench_check.sh                      # auto-detect
#   scripts/field/03_fc_bench_check.sh --device /dev/ttyACM0 --baud 57600
#   scripts/field/03_fc_bench_check.sh --url udpin://0.0.0.0:14540   # e.g. SITL
#   scripts/field/03_fc_bench_check.sh --require            # no FC = FAIL
#
# Exit: 0 PASS | 1 FAIL (port there, no MAVLink) | 2 usage | 3 NOT CONNECTED
# ===========================================================================
set -euo pipefail

FLD_TAG="03-fc"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/common.sh"

DEVICE=""
BAUD="57600"
URL=""
CONNECT_TIMEOUT="30"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --device)  DEVICE="${2:?}"; shift 2 ;;
        --baud)    BAUD="${2:?}"; shift 2 ;;
        --url)     URL="${2:?}"; shift 2 ;;
        --timeout) CONNECT_TIMEOUT="${2:?}"; shift 2 ;;
        --require) FLD_REQUIRE=1; shift ;;
        -h|--help) fld_usage_from_header "$(readlink -f "${BASH_SOURCE[0]}")"; exit 0 ;;
        *) fld_bad "unknown arg: $1"; fld_finish "$FLD_USAGE" "run with -h for usage" ;;
    esac
done

PY="$(fld_python)" || fld_finish "$FLD_USAGE" "no usable python venv"
CHECK="$FLD_DIR/fc_link_check.py"
[[ -f "$CHECK" ]] || fld_finish "$FLD_USAGE" "missing $CHECK"

LOG_DIR="$(fld_log_dir 03_fc)"
LOG="$LOG_DIR/report.txt"
exec > >(tee "$LOG") 2>&1
fld_say "log -> $LOG"

# ---------------------------------------------------- prove it cannot arm
fld_head "1. safety audit (before touching anything)"
if ! "$PY" "$CHECK" --self-test 2>&1 | sed "s/^/[$FLD_TAG]   | /"; then
    fld_finish "$FLD_FAIL" "the read-only link checker failed its own safety audit -- do NOT continue"
fi

# ------------------------------------------------------------ find the port
fld_head "2. finding the flight controller"
if [[ -n "$URL" ]]; then
    fld_ok "using the url you gave: $URL (skipping device auto-detect)"
fi
if [[ -z "$URL" ]]; then
    if [[ -z "$DEVICE" ]]; then
        mapfile -t SERIALS < <(fld_serial_candidates)
        if ((${#SERIALS[@]} == 0)); then
            fld_miss "no serial port found (/dev/serial/by-id, /dev/ttyACM*, /dev/ttyUSB*)"
            HINTS=("plug the Pixhawk 6C Mini in with a USB-C DATA cable (charge-only cables enumerate nothing)")
            if fld_is_wsl; then
                HINTS+=("WSL2: Windows owns the USB port until you hand it over --")
                HINTS+=("  1) admin PowerShell:  usbipd list          (find the BUSID)")
                HINTS+=("  2) admin PowerShell:  usbipd bind --busid <BUSID>")
                HINTS+=("  3) any PowerShell:    usbipd attach --wsl --busid <BUSID>")
                HINTS+=("  docs: https://learn.microsoft.com/en-us/windows/wsl/connect-usb")
            fi
            HINTS+=("then re-run: scripts/field/00_detect_devices.sh")
            fld_finish "$FLD_ABSENT" "${HINTS[@]}"
        fi
        DEVICE="${SERIALS[0]}"
        if ((${#SERIALS[@]} > 1)); then
            fld_warn "${#SERIALS[@]} serial ports present; using the first one"
            for s in "${SERIALS[@]}"; do fld_say "    - $s  [$(fld_guess_board "$(fld_describe_serial "$s")")]"; done
            fld_hint "pick explicitly with --device <path> if that is the wrong board"
        fi
    fi
    if [[ ! -e "$DEVICE" ]]; then
        fld_bad "device does not exist: $DEVICE"
        fld_finish "$FLD_ABSENT" "check the cable / usbipd attach, then re-run 00_detect_devices.sh"
    fi
    NODE="$(readlink -f "$DEVICE")"
    fld_ok "using $(fld_describe_serial "$DEVICE")"
    fld_say "board guess: $(fld_guess_board "$(fld_describe_serial "$DEVICE")")  (name-string match only)"
    if [[ ! -r "$NODE" || ! -w "$NODE" ]]; then
        fld_bad "no read/write permission on $NODE"
        fld_finish "$FLD_FAIL" \
            "sudo usermod -aG dialout \$USER   then close and reopen the WSL terminal" \
            "or, one-off: sudo chmod 666 $NODE"
    fi
    URL="serial://${NODE}:${BAUD}"
fi

# ---------------------------------------------------------------- talk to it
fld_head "3. MAVLink read-only check"
fld_say "url: $URL"
fld_say "(props off; this tool reads telemetry only and cannot arm)"
set +e
"$PY" "$CHECK" --url "$URL" --connect-timeout "$CONNECT_TIMEOUT" \
    --json-out "$LOG_DIR/verdict.json" 2>&1 | sed "s/^/[$FLD_TAG]   | /"
RC="${PIPESTATUS[0]}"
set -e

if [[ "$RC" -eq 2 ]]; then
    fld_finish "$FLD_USAGE" "bad arguments to fc_link_check.py"
fi
if [[ "$RC" -ne 0 ]]; then
    fld_finish "$FLD_FAIL" \
        "the port exists but no MAVLink came back" \
        "is QGroundControl (or another MAVSDK script) holding the port? close it and retry" \
        "is PX4 actually flashed? QGC -> Vehicle Setup -> Firmware (configs/px4_6cmini/README.md step 1)" \
        "wrong port? list them with scripts/field/00_detect_devices.sh and pass --device" \
        "on WSL, re-attach with: usbipd attach --wsl --busid <BUSID>"
fi

fld_finish "$FLD_OK" \
    "results: $LOG_DIR/verdict.json" \
    "next (props-off OFFBOARD gate, brn-05): scripts/check_deploy_bench.sh --check-only, then the real run" \
    "params to load in QGC: configs/px4_6cmini/bench.params"
