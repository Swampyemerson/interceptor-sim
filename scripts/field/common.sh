#!/usr/bin/env bash
# ===========================================================================
# scripts/field/common.sh -- shared helpers for the FIELD BRING-UP checks.
#
# SOURCE this file (`source .../common.sh`); it is not meant to be executed.
# It exists so the five bring-up scripts (00..04 + bringup.sh) share ONE
# definition of: the exit-code contract, the PASS/FAIL/ABSENT printing, the
# log-dir convention, venv resolution, and device discovery. Nothing here
# touches hardware or the sim.
#
# THE EXIT-CODE CONTRACT (every scripts/field/ script obeys it)
# ------------------------------------------------------------
#   0  PASS    the check ran against real gear (or a sanctioned dry run) and passed
#   1  FAIL    the gear IS there but something is wrong -> read the hint, fix it
#   2  USAGE   bad arguments / missing repo file (operator error, not hardware)
#   3  ABSENT  the gear is NOT plugged in yet. Advisory, NOT an error: this is
#              what you get today, before the parts arrive. `--require` promotes
#              ABSENT to FAIL so the same script can be a hard gate on field day.
#
# Why a separate ABSENT code: `bringup.sh` must be runnable on the laptop with
# nothing attached and still tell you the software half is healthy. Collapsing
# "not plugged in" into FAIL would make the pre-hardware run look broken.
#
# House rules honored here (CLAUDE.md): every run writes to logs/ ; no `pkill`/
# `pgrep` with an inline literal pattern anywhere in scripts/field/ (these
# scripts never kill anything); numbers/paths trace to a doc or to existing code.
# ===========================================================================

# --- exit codes ------------------------------------------------------------
FLD_OK=0
FLD_FAIL=1
FLD_USAGE=2
FLD_ABSENT=3

# --- repo paths ------------------------------------------------------------
# readlink -f so the scripts work through a symlink or from any cwd.
FLD_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_ROOT="$(cd "$FLD_DIR/../.." && pwd)"
FLD_LOGS="$REPO_ROOT/logs/field"

# --- pretty printing -------------------------------------------------------
# Colors only when stdout is a TTY, so tee'd logs stay clean text.
if [[ -t 1 ]]; then
    _C_G=$'\033[32m'; _C_R=$'\033[31m'; _C_Y=$'\033[33m'; _C_B=$'\033[1m'; _C_0=$'\033[0m'
else
    _C_G=""; _C_R=""; _C_Y=""; _C_B=""; _C_0=""
fi

FLD_TAG="${FLD_TAG:-field}"          # each script sets this to its own name

fld_say()   { printf '[%s] %s\n' "$FLD_TAG" "$*"; }
fld_ok()    { printf '[%s]   %sOK%s   %s\n'   "$FLD_TAG" "$_C_G" "$_C_0" "$*"; }
fld_warn()  { printf '[%s]   %sWARN%s %s\n'   "$FLD_TAG" "$_C_Y" "$_C_0" "$*"; }
fld_bad()   { printf '[%s]   %sBAD%s  %s\n'   "$FLD_TAG" "$_C_R" "$_C_0" "$*"; }
fld_miss()  { printf '[%s]   %s--%s   %s\n'   "$FLD_TAG" "$_C_Y" "$_C_0" "$*"; }
fld_head()  { printf '\n[%s] %s%s%s\n' "$FLD_TAG" "$_C_B" "$*" "$_C_0"; }
fld_hint()  { printf '[%s]      -> %s\n' "$FLD_TAG" "$*"; }

# fld_finish <code> [hint ...] -- the single exit point every script uses, so
# the PASS/FAIL/ABSENT line always looks the same and always carries a
# next-step hint. Respects FLD_REQUIRE=1 (the --require flag) by promoting
# ABSENT -> FAIL.
fld_finish() {
    local code="$1"; shift
    if [[ "$code" == "$FLD_ABSENT" && "${FLD_REQUIRE:-0}" == "1" ]]; then
        code="$FLD_FAIL"
        fld_say "(--require given: 'hardware not connected' counts as FAIL)"
    fi
    case "$code" in
        "$FLD_OK")     printf '\n[%s] %sPASS%s\n' "$FLD_TAG" "$_C_G" "$_C_0" ;;
        "$FLD_ABSENT") printf '\n[%s] %sNOT CONNECTED%s (advisory, exit %s)\n' \
                              "$FLD_TAG" "$_C_Y" "$_C_0" "$FLD_ABSENT" ;;
        "$FLD_USAGE")  printf '\n[%s] %sUSAGE ERROR%s (exit %s)\n' \
                              "$FLD_TAG" "$_C_R" "$_C_0" "$FLD_USAGE" ;;
        *)             printf '\n[%s] %sFAIL%s (exit %s)\n' "$FLD_TAG" "$_C_R" "$_C_0" "$code" ;;
    esac
    local h
    for h in "$@"; do [[ -n "$h" ]] && fld_hint "$h"; done
    exit "$code"
}

# --- logging ---------------------------------------------------------------
# fld_log_dir <name> -> creates logs/field/<name>_<UTC stamp>/ and echoes it.
# (CLAUDE.md: log everything to logs/; logs/ is gitignored.)
fld_log_dir() {
    local name="$1"
    local d="$FLD_LOGS/${name}_$(date -u +%Y%m%dT%H%M%SZ)"
    mkdir -p "$d"
    printf '%s\n' "$d"
}

# --- python / venv ---------------------------------------------------------
# The repo has several venvs (CLAUDE.md conventions):
#   .venv          cv2, numpy, mavsdk, pupil-apriltags, pyulog, pymavlink  <- what field/ needs
#   .venv-seeker   adds onnxruntime (the NN seeker); used by check_deploy_bench.sh
# Everything in scripts/field/ runs under .venv unless FIELD_PYTHON overrides it.
fld_python() {
    local p="${FIELD_PYTHON:-$REPO_ROOT/.venv/bin/python}"
    if [[ ! -x "$p" ]]; then
        fld_bad "python not found: $p"
        fld_hint "create it with: bash $REPO_ROOT/bootstrap.sh   (or set FIELD_PYTHON=...)"
        return 1
    fi
    printf '%s\n' "$p"
}

# --- host environment ------------------------------------------------------
fld_is_wsl() { grep -qi microsoft /proc/version 2>/dev/null; }

# Locate the Windows-side usbipd CLI from inside WSL (interop). Echoes the path
# on success, nothing on failure. usbipd-win is what passes a USB device from
# Windows into WSL2 -- without it the Pixhawk/Kakute never appear in Linux.
# Source: https://learn.microsoft.com/en-us/windows/wsl/connect-usb
fld_usbipd_exe() {
    local c
    for c in "$(command -v usbipd.exe 2>/dev/null || true)" \
             "/mnt/c/Program Files/usbipd-win/usbipd.exe" \
             "/mnt/c/Program Files (x86)/usbipd-win/usbipd.exe"; do
        [[ -n "$c" && -x "$c" ]] && { printf '%s\n' "$c"; return 0; }
    done
    return 1
}

# fld_kmod_present <name> -- is a kernel module shipped for THIS kernel (built
# in or as a .ko)? Used to tell "the device is missing" apart from "the driver
# that would create /dev/ttyACM0 isn't in this kernel at all".
fld_kmod_present() {
    local m="${1//_/-}" kd="/lib/modules/$(uname -r)" hit=""
    [[ -d "$kd" ]] || return 1
    # Capture into a variable rather than piping: under `set -o pipefail` a
    # find that hits an unreadable dir (lost+found) exits 1 and would poison
    # the pipeline's status even though the module WAS found.
    hit="$(find "$kd" -name "${m}.ko*" -print -quit 2>/dev/null || true)"
    [[ -n "$hit" ]] && return 0
    grep -q "/${m}\.ko" "$kd/modules.builtin" 2>/dev/null && return 0
    return 1
}

# --- device discovery ------------------------------------------------------
# fld_serial_candidates -- echo every plausible flight-controller serial port,
# one per line, stable-name (/dev/serial/by-id/...) first because that name
# survives replug and identifies the board. Both flight controllers in the BOM
# present as USB CDC-ACM (/dev/ttyACM*) over their USB-C port; a USB-TTL/FTDI
# adapter would show as /dev/ttyUSB*.
#   BOM: Pixhawk 6C Mini (PX4, interceptor brain) + Kakute H7 V1.5 (ArduPilot,
#   target) -- docs/project_state.json build_tab subsystems `brain` / `target`.
fld_serial_candidates() {
    local d
    if [[ -d /dev/serial/by-id ]]; then
        for d in /dev/serial/by-id/*; do [[ -e "$d" ]] && printf '%s\n' "$d"; done
    fi
    for d in /dev/ttyACM* /dev/ttyUSB*; do [[ -e "$d" ]] && printf '%s\n' "$d"; done
}

# fld_describe_serial <dev> -- one-line human description of a serial port:
# the by-id name (which carries the manufacturer/product strings) when we can
# resolve one, else just the node. NOTE: we match on NAME STRINGS only. The
# exact USB VID:PID of each board is NOT recorded anywhere in this repo yet --
# TODO: capture `lsusb` output on first plug-in and paste it into
# docs/field_bringup.md so future runs can match on VID:PID instead of a guess.
fld_describe_serial() {
    local dev="$1" byid real
    real="$(readlink -f "$dev" 2>/dev/null || printf '%s' "$dev")"
    if [[ -d /dev/serial/by-id ]]; then
        for byid in /dev/serial/by-id/*; do
            [[ -e "$byid" ]] || continue
            if [[ "$(readlink -f "$byid")" == "$real" ]]; then
                printf '%s (-> %s)\n' "$(basename "$byid")" "$(basename "$real")"
                return 0
            fi
        done
    fi
    printf '%s\n' "$(basename "$real")"
}

# fld_guess_board <string> -- best-effort board id from a by-id/lsusb string.
# Keyword match only (see the TODO above); prints one of: px4 | ardupilot |
# unknown. Used to *suggest* which port is the interceptor brain, never to
# decide anything safety-relevant.
fld_guess_board() {
    local s
    s="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
    case "$s" in
        *px4*|*pixhawk*|*fmu*|*holybro*|*auterion*) printf 'px4\n' ;;
        *betaflight*|*ardupilot*|*kakute*|*chibios*|*inav*) printf 'ardupilot\n' ;;
        *) printf 'unknown\n' ;;
    esac
}

# fld_video_devices -- echo /dev/video* nodes (USB/UVC cameras only).
# The BOM camera is an innomaker OV9281 on the Pi 5's MIPI CSI ribbon
# (docs/camera_paper_check.md item 5) -- CSI is NOT USB, so it can never show
# up here. On this laptop /dev/video* means "some UVC webcam", not the seeker.
fld_video_devices() {
    local d
    for d in /dev/video*; do [[ -e "$d" ]] && printf '%s\n' "$d"; done
}

# --- the Pi 5 (seeker rig) over the network --------------------------------
# The Pi is a separate computer; the laptop reaches it over SSH.
# The provisioning pack sets the hostname (scripts/pi_setup/README.md §1:
# `provision.sh --hostname interceptor-seeker`), so the default below matches.
#
# USE THE IP AT THE FIELD, NOT `.local`. The default is an mDNS name, and mDNS
# does NOT resolve from NAT-mode WSL2 (nsswitch is `files dns`, the VM sits
# behind a 172.x NAT), which is exactly where these scripts run. On a phone
# hotspot it is less reliable still. So:
#     export FIELD_PI_HOST=pi@192.168.43.17     # the IP the Pi shows on the hotspot
# or pin it once with a line in WSL's /etc/hosts. `scripts/field/05_pi_link_check.sh`
# tests this whole path (key, address, reachability, clock) before you pack.
FIELD_PI_HOST="${FIELD_PI_HOST:-}"                       # e.g. pi@192.168.43.17
FIELD_PI_DEFAULT_HOST="${FIELD_PI_DEFAULT_HOST:-interceptor-seeker.local}"
FIELD_PI_REPO="${FIELD_PI_REPO:-~/interceptor-sim}"      # scripts/pi_setup/README.md §1 clone path
FIELD_PI_PYTHON="${FIELD_PI_PYTHON:-~/interceptor-sim/.venv-pi/bin/python}"  # provision.sh default venv

# fld_pi_reachable <user@host> -- true if a non-interactive SSH works quickly.
# BatchMode=yes so a missing key fails fast instead of prompting for a password
# (a hanging password prompt inside a gate script is worse than a clean FAIL).
fld_pi_reachable() {
    local host="$1"
    [[ -n "$host" ]] || return 1
    ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
        -o ConnectTimeout="${FIELD_SSH_TIMEOUT:-6}" "$host" true 2>/dev/null
}

# --- shared arg parsing ----------------------------------------------------
# Every script accepts --require and -h/--help; this keeps that uniform.
FLD_REQUIRE="${FLD_REQUIRE:-0}"

# fld_usage_from_header <file> -- print the script's own header comment block
# as its --help text (same trick as scripts/check_deploy_bench.sh).
fld_usage_from_header() {
    sed -n '3,/^# ===/p' "$1" | sed 's/^# \{0,1\}//' | sed '$d'
}
