#!/usr/bin/env bash
# ===========================================================================
# bringup.sh -- THE ONE COMMAND: "did my laptop see everything?"
#
# Runs the field bring-up checks 00 -> 03 in order and prints one summary table.
# Run it the day the parts arrive, and again before every field day. Full
# plain-English runbook: docs/field_bringup.md
#
#   00_detect_devices.sh     what is plugged in / reachable (+ the WSL usbipd fix)
#   01_camera_live_check.sh  frames from the OV9281 on the Pi 5, exposure vs spec
#   02_apriltag_desk_check.sh the AprilTag baseline seeker decodes the placard
#   03_fc_bench_check.sh     MAVLink telemetry from the flight controller (no arming)
#
# (04_pull_logs.sh is NOT part of this gate -- it runs AFTER a flight.)
#
# EXIT-CODE POLICY -- read this once and the output makes sense:
#   Each check reports PASS, FAIL, or NOT CONNECTED.
#     PASS           checked real gear (or a sanctioned dry run), all good.
#     FAIL           the gear IS there and something is wrong -> fix it.
#     NOT CONNECTED  that piece is not plugged in yet. Not an error today.
#   bringup.sh exits 0 when NOTHING FAILED (missing gear is reported, not fatal),
#   1 if any check failed. With --require, "not connected" also counts as a
#   failure -- use that on field day when everything IS supposed to be attached.
#
# Usage:
#   scripts/field/bringup.sh                       # the friendly pre-hardware run
#   scripts/field/bringup.sh --pi pi@interceptor-seeker.local
#   scripts/field/bringup.sh --require             # field-day gate: all gear must be present
#   scripts/field/bringup.sh --only 00,03          # just those steps
#   scripts/field/bringup.sh --skip 01,02          # everything but those
#
# Exit: 0 nothing failed | 1 something failed | 2 usage
# ===========================================================================
set -uo pipefail          # NOT -e: a failing sub-check must not abort the sweep

FLD_TAG="bringup"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/common.sh"

ONLY=""
SKIP=""
PI_ARG=()
REQUIRE_ARG=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pi)               PI_ARG=(--pi "${2:?}"); shift 2 ;;
        --require|--strict) REQUIRE_ARG=(--require); FLD_REQUIRE=1; shift ;;
        --only)             ONLY="${2:?}"; shift 2 ;;
        --skip)             SKIP="${2:?}"; shift 2 ;;
        -h|--help)          fld_usage_from_header "$(readlink -f "${BASH_SOURCE[0]}")"; exit 0 ;;
        *) fld_bad "unknown arg: $1"; fld_finish "$FLD_USAGE" "run with -h for usage" ;;
    esac
done

wanted() {
    local id="$1"
    [[ -n "$ONLY" && ",$ONLY," != *",$id,"* ]] && return 1
    [[ -n "$SKIP" && ",$SKIP," == *",$id,"* ]] && return 1
    return 0
}

LOG_DIR="$(fld_log_dir bringup)"
LOG="$LOG_DIR/bringup.txt"
exec > >(tee "$LOG") 2>&1

cat <<BANNER
===========================================================================
 INTERCEPTOR FIELD BRING-UP
 host   : $(uname -sr)
 repo   : $REPO_ROOT
 log    : $LOG
 runbook: docs/field_bringup.md
===========================================================================
BANNER

STEP_IDS=()
STEP_NAMES=()
STEP_CODES=()

run_step() {
    local id="$1" name="$2"; shift 2
    if ! wanted "$id"; then
        fld_say "--- skipping $id $name"
        return
    fi
    printf '\n===========================================================================\n'
    printf ' STEP %s  %s\n' "$id" "$name"
    printf '===========================================================================\n'
    "$@"
    local rc=$?
    STEP_IDS+=("$id"); STEP_NAMES+=("$name"); STEP_CODES+=("$rc")
}

run_step 00 "device detection" \
    "$FLD_DIR/00_detect_devices.sh" "${PI_ARG[@]}" "${REQUIRE_ARG[@]}"
run_step 01 "camera frames" \
    "$FLD_DIR/01_camera_live_check.sh" "${PI_ARG[@]}" "${REQUIRE_ARG[@]}"
run_step 02 "apriltag decode" \
    "$FLD_DIR/02_apriltag_desk_check.sh" "${REQUIRE_ARG[@]}"
run_step 03 "flight-controller link" \
    "$FLD_DIR/03_fc_bench_check.sh" "${REQUIRE_ARG[@]}"

# ------------------------------------------------------------------ summary
printf '\n===========================================================================\n'
printf ' SUMMARY\n'
printf '===========================================================================\n'
n_fail=0; n_absent=0; n_pass=0
for i in "${!STEP_IDS[@]}"; do
    code="${STEP_CODES[$i]}"
    case "$code" in
        0) label="${_C_G}PASS${_C_0}         "; n_pass=$((n_pass + 1)) ;;
        3) label="${_C_Y}NOT CONNECTED${_C_0}"; n_absent=$((n_absent + 1)) ;;
        2) label="${_C_R}USAGE ERROR${_C_0}  "; n_fail=$((n_fail + 1)) ;;
        *) label="${_C_R}FAIL${_C_0}         "; n_fail=$((n_fail + 1)) ;;
    esac
    printf '  %-3s %-26s %b   (exit %s)\n' "${STEP_IDS[$i]}" "${STEP_NAMES[$i]}" "$label" "$code"
done
printf '\n  %d passed, %d not connected, %d failed\n' "$n_pass" "$n_absent" "$n_fail"

if ((n_fail > 0)); then
    printf '\n[%s] %sBRING-UP FAILED%s -- scroll up to the failing step and follow its "->" hints.\n' \
        "$FLD_TAG" "$_C_R" "$_C_0"
    printf '[%s] full log: %s\n' "$FLD_TAG" "$LOG"
    exit 1
fi
if ((n_absent > 0)); then
    printf '\n[%s] %sNOTHING IS BROKEN%s -- %d piece(s) of hardware are simply not connected yet.\n' \
        "$FLD_TAG" "$_C_G" "$_C_0" "$n_absent"
    printf '[%s] Plug them in and re-run. Use --require once everything IS attached.\n' "$FLD_TAG"
    printf '[%s] What to plug where: docs/field_bringup.md\n' "$FLD_TAG"
    printf '[%s] full log: %s\n' "$FLD_TAG" "$LOG"
    exit 0
fi
printf '\n[%s] %sALL CHECKS PASSED%s -- you are off to the races.\n' "$FLD_TAG" "$_C_G" "$_C_0"
printf '[%s] Next: props-off OFFBOARD gate  scripts/check_deploy_bench.sh\n' "$FLD_TAG"
printf '[%s] After a flight:                scripts/field/04_pull_logs.sh\n' "$FLD_TAG"
printf '[%s] full log: %s\n' "$FLD_TAG" "$LOG"
exit 0
