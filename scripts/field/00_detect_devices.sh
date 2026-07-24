#!/usr/bin/env bash
# ===========================================================================
# 00_detect_devices.sh -- "does my laptop see the gear?" -- step 0 of the field
# bring-up flow (scripts/field/, runbook docs/field_bringup.md).
#
# WHAT IT DOES: enumerates every place the ordered hardware could show up on
# this machine and prints FOUND / NOT CONNECTED against the expected BOM, with
# the exact next action for anything missing. It reads only -- it never binds,
# attaches, flashes, arms, or writes to a device.
#
# THE ORDERED HARDWARE AND WHERE IT SHOWS UP (docs/project_state.json
# build_tab; docs/hardware_order_list.md 0c/0d):
#   Pixhawk 6C Mini (PX4, interceptor brain)  USB-C -> /dev/ttyACM* (CDC-ACM)
#   Kakute H7 V1.5  (ArduPilot, target)       USB-C -> /dev/ttyACM* (CDC-ACM)
#   Raspberry Pi 5 (seeker rig)               NETWORK -> ssh interceptor-seeker.local
#   innomaker OV9281 camera                   MIPI CSI ribbon INTO THE PI 5 --
#                                             never on this laptop (it is not USB;
#                                             docs/camera_paper_check.md item 5)
#   microSD log cards (ULog / DataFlash)      card reader -> a Windows drive under /mnt
#
# THE WSL2 CATCH (the #1 "why can't my laptop see the Pixhawk"): a USB device
# plugged into Windows is NOT visible inside WSL2 until it is shared and
# attached with usbipd-win. This script detects that situation and prints the
# exact commands. Source: https://learn.microsoft.com/en-us/windows/wsl/connect-usb
#
# Usage:
#   scripts/field/00_detect_devices.sh                 # report (default expect: fc,pi)
#   scripts/field/00_detect_devices.sh --expect fc     # only the flight controller matters
#   scripts/field/00_detect_devices.sh --pi pi@interceptor-seeker.local
#   scripts/field/00_detect_devices.sh --require       # missing gear = FAIL (field-day gate)
#
# Exit: 0 PASS | 1 FAIL (gear present but broken) | 2 usage | 3 NOT CONNECTED
# ===========================================================================
set -euo pipefail

FLD_TAG="00-devices"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/common.sh"

EXPECT="fc,pi"
PI_HOST="$FIELD_PI_HOST"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --expect)  EXPECT="${2:-}"; shift 2 ;;
        --pi)      PI_HOST="${2:-$FIELD_PI_DEFAULT_HOST}"; shift 2 ;;
        --require) FLD_REQUIRE=1; shift ;;
        -h|--help) fld_usage_from_header "$(readlink -f "${BASH_SOURCE[0]}")"; exit 0 ;;
        *) fld_bad "unknown arg: $1"; fld_finish "$FLD_USAGE" "run with -h for usage" ;;
    esac
done
expects() { [[ ",$EXPECT," == *",$1,"* ]]; }

LOG_DIR="$(fld_log_dir 00_detect_devices)"
LOG="$LOG_DIR/report.txt"
exec > >(tee "$LOG") 2>&1     # everything from here on lands in logs/field/ too

fld_say "device scan -> $LOG"

# ---------------------------------------------------------------- host env
fld_head "1. This machine"
fld_say "kernel : $(uname -sr)"
if fld_is_wsl; then
    fld_ok "WSL2 detected -- USB gear needs usbipd-win (checked in section 2)"
else
    fld_say "not WSL -- plain Linux USB rules apply (no usbipd step needed)"
fi
# Do the drivers that CREATE the device nodes even exist in this kernel? On the
# stock WSL2 kernel they ship as modules; this catches a rebuilt/stripped kernel
# before you waste an hour blaming the cable.
for m in cdc-acm vhci-hcd uvcvideo; do
    if fld_kmod_present "$m"; then
        fld_ok "kernel driver present: $m"
    else
        fld_warn "kernel driver MISSING: $m  (nodes it creates will never appear)"
    fi
done

# --------------------------------------------------------------- usbipd-win
fld_head "2. USB pass-through from Windows (usbipd-win)"
USBIPD=""
USBIPD_ATTACHED=0
if ! fld_is_wsl; then
    fld_say "skipped (not WSL)"
elif USBIPD="$(fld_usbipd_exe)"; then
    fld_ok "usbipd found: $USBIPD"
    # `usbipd list` is read-only and needs no admin rights (admin is only
    # required for `bind`). Strip CRs: it is a Windows binary.
    if UL="$("$USBIPD" list 2>&1 | tr -d '\r')"; then
        printf '%s\n' "$UL" | sed "s/^/[$FLD_TAG]   | /"
        printf '%s\n' "$UL" > "$LOG_DIR/usbipd_list.txt"
        if printf '%s\n' "$UL" | grep -qi "Attached"; then
            USBIPD_ATTACHED=1
            fld_ok "at least one device is ATTACHED to WSL"
        else
            fld_miss "no device is attached to WSL yet"
            fld_hint "in an ADMIN PowerShell:  usbipd bind --busid <BUSID>"
            fld_hint "then in any PowerShell:  usbipd attach --wsl --busid <BUSID>"
        fi
    else
        fld_warn "could not run '$USBIPD list'"
    fi
else
    fld_miss "usbipd.exe not found on the Windows side"
    fld_hint "install it once, from an admin PowerShell:"
    fld_hint "  winget install --interactive --exact dorssel.usbipd-win"
    fld_hint "docs: https://learn.microsoft.com/en-us/windows/wsl/connect-usb"
fi

# ------------------------------------------------------------ serial devices
fld_head "3. Flight-controller serial ports"
mapfile -t SERIALS < <(fld_serial_candidates)
FC_PX4=""; FC_ARDU=""; FC_UNKNOWN=""; PERM_PROBLEM=0
if ((${#SERIALS[@]} == 0)); then
    fld_miss "no /dev/serial/by-id, /dev/ttyACM*, or /dev/ttyUSB* found"
else
    for dev in "${SERIALS[@]}"; do
        desc="$(fld_describe_serial "$dev")"
        board="$(fld_guess_board "$desc")"
        node="$(readlink -f "$dev")"
        if [[ -r "$node" && -w "$node" ]]; then
            fld_ok "$desc   [guess: $board]"
        else
            fld_bad "$desc   [guess: $board]  -- NO read/write permission"
            PERM_PROBLEM=1
        fi
        case "$board" in
            px4)       [[ -z "$FC_PX4"     ]] && FC_PX4="$dev" ;;
            ardupilot) [[ -z "$FC_ARDU"    ]] && FC_ARDU="$dev" ;;
            *)         [[ -z "$FC_UNKNOWN" ]] && FC_UNKNOWN="$dev" ;;
        esac
    done
fi

# ------------------------------------------------------------ video devices
fld_head "4. Camera devices on THIS machine"
mapfile -t VIDEOS < <(fld_video_devices)
if ((${#VIDEOS[@]} == 0)); then
    fld_miss "no /dev/video* here"
else
    for v in "${VIDEOS[@]}"; do fld_ok "$v (a USB/UVC camera)"; done
fi
fld_say "note: the seeker camera (innomaker OV9281) is a MIPI CSI part that"
fld_say "      plugs into the PI 5, not into this laptop -- it will NEVER"
fld_say "      appear as /dev/video* here (docs/camera_paper_check.md item 5)."

# ------------------------------------------------------------------- the Pi
fld_head "5. Raspberry Pi 5 seeker rig (over the network)"
PI_OK=0
PI_TRY="${PI_HOST:-$FIELD_PI_DEFAULT_HOST}"
if fld_pi_reachable "$PI_TRY"; then
    PI_OK=1
    fld_ok "ssh $PI_TRY works"
    ssh -o BatchMode=yes -o ConnectTimeout="${FIELD_SSH_TIMEOUT:-6}" "$PI_TRY" \
        'echo "    host: $(hostname)  model: $(tr -d "\0" </proc/device-tree/model 2>/dev/null || echo unknown)"' \
        2>/dev/null | sed "s/^/[$FLD_TAG]   | /" || true
else
    fld_miss "cannot ssh to '$PI_TRY'"
    fld_hint "Pi powered on, on the same Wi-Fi, SSH enabled (Pi Imager advanced options)?"
    fld_hint "provision it with: scripts/pi_setup/provision.sh --hostname interceptor-seeker"
    fld_hint "set a different address with: --pi pi@<ip-or-name>  (or export FIELD_PI_HOST)"
fi

# ------------------------------------------------------------- log-card scan
fld_head "6. microSD log cards (Windows drives under /mnt)"
CARDS=0
for m in /mnt/[d-z]; do
    [[ -d "$m" ]] || continue
    if compgen -G "$m/log/*" >/dev/null 2>&1; then
        fld_ok "$m looks like a PX4 card (log/ present) -- ULogs for field_score"
        CARDS=$((CARDS + 1))
    elif compgen -G "$m/APM/LOGS/*" >/dev/null 2>&1; then
        fld_ok "$m looks like an ArduPilot card (APM/LOGS/ present) -- .BIN logs"
        CARDS=$((CARDS + 1))
    fi
done
((CARDS == 0)) && fld_miss "no flight-log card mounted (normal until after a flight)"

# ------------------------------------------------------------------ verdict
fld_head "SUMMARY (expected set: $EXPECT)"
STATUS="$FLD_OK"
HINTS=()

if expects fc; then
    if [[ -n "$FC_PX4" ]]; then
        fld_ok "flight controller (PX4-looking): $FC_PX4"
    elif [[ -n "$FC_ARDU" ]]; then
        fld_ok "flight controller (ArduPilot-looking): $FC_ARDU"
    elif [[ -n "$FC_UNKNOWN" ]]; then
        fld_warn "a serial port exists but the name did not identify a board: $FC_UNKNOWN"
        fld_hint "run: scripts/field/03_fc_bench_check.sh --device $FC_UNKNOWN"
    else
        fld_miss "flight controller: NOT CONNECTED"
        [[ "$STATUS" == "$FLD_OK" ]] && STATUS="$FLD_ABSENT"
        HINTS+=("plug the Pixhawk 6C Mini into the laptop with a USB-C DATA cable (not charge-only)")
        if fld_is_wsl; then
            if [[ -n "$USBIPD" ]]; then
                HINTS+=("then share it into WSL: admin PowerShell 'usbipd bind --busid <ID>', then 'usbipd attach --wsl --busid <ID>'")
            else
                HINTS+=("install usbipd-win first: winget install --interactive --exact dorssel.usbipd-win")
            fi
        fi
    fi
    # Present-but-broken cases -> FAIL, not ABSENT.
    if ((PERM_PROBLEM)); then
        STATUS="$FLD_FAIL"
        HINTS+=("permission fix: sudo usermod -aG dialout \$USER  (then reopen WSL), or sudo chmod 666 /dev/ttyACM0")
    fi
    if ((USBIPD_ATTACHED)) && ((${#SERIALS[@]} == 0)); then
        STATUS="$FLD_FAIL"
        HINTS+=("usbipd says a device is ATTACHED but no serial node appeared -- detach/attach again, or check that cdc-acm is present (section 1)")
    fi
fi

if expects pi; then
    if ((PI_OK)); then
        fld_ok "Pi 5 seeker rig reachable at $PI_TRY"
    else
        fld_miss "Pi 5 seeker rig: NOT REACHABLE"
        [[ "$STATUS" == "$FLD_OK" ]] && STATUS="$FLD_ABSENT"
        HINTS+=("power the Pi 5, join it to Wi-Fi, then re-run; camera checks run ON the Pi (step 01)")
    fi
fi

HINTS+=("next step: scripts/field/01_camera_live_check.sh   (runbook: docs/field_bringup.md)")
fld_finish "$STATUS" "${HINTS[@]}"
