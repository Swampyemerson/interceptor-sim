#!/usr/bin/env bash
# ===========================================================================
# 01_camera_live_check.sh -- "is the seeker camera producing good frames?"
# Step 1 of the field bring-up flow (runbook: docs/field_bringup.md).
#
# ONE CODE PATH: every mode below drives scripts/seeker/pi_capture.py -- the
# same recorder the tripod field day uses -- so a frame you check here went
# through exactly the pipeline the money-gate data will. This script only picks
# a backend, moves the session to logs/field/, and grades it with
# scripts/field/camera_session_summary.py.
#
# WHERE THE CAMERA ACTUALLY LIVES: the BOM camera is an innomaker OV9281
# (1 MP mono GLOBAL SHUTTER, 1280x800, ~118 deg HFoV) on a MIPI CSI ribbon into
# the RASPBERRY PI 5 (docs/camera_paper_check.md; docs/project_state.json
# build_tab -> seeker). CSI is not USB, so the camera CANNOT be plugged into
# this laptop. Hence the three modes:
#
#   --pi [user@host]   REAL CHECK. ssh to the Pi, capture there with the
#                      picamera2 backend, copy the session back here. This is
#                      the mode that verifies the <=1 ms exposure spec.
#   --local [/dev/videoN]  a USB/UVC camera attached to this machine (needs
#                      usbipd on WSL). Useful for a desk rehearsal; NOTE V4L2
#                      exposure units are device-specific so the 1 ms spec is
#                      NOT graded in this mode.
#   --replay [DIR]     NO HARDWARE dry run: replays existing frames through the
#                      identical recorder so you can prove the pipeline today.
#                      Default DIR = the repo's sim AprilTag frames.
#   (no flag)          auto: Pi if reachable -> local camera if present ->
#                      replay dry run (and exit 3 NOT CONNECTED).
#
# WHY <=1 ms EXPOSURE: at the >=9 m/s engagement the terminal line-of-sight
# rate smears a long exposure across the target; the global shutter + short
# exposure is the whole reason this sensor was chosen. pi_capture.py REQUESTS
# it and logs what the sensor ACTUALLY applied (spec: pi_capture.py
# EXPOSURE_SPEC_US = 1000; docs/tripod_test_protocol.md 3.3/4.4).
#
# Usage:
#   scripts/field/01_camera_live_check.sh                    # auto
#   scripts/field/01_camera_live_check.sh --pi pi@interceptor-seeker.local
#   scripts/field/01_camera_live_check.sh --local /dev/video0
#   scripts/field/01_camera_live_check.sh --replay
#   scripts/field/01_camera_live_check.sh --frames 60 --exposure-us 1000
#   scripts/field/01_camera_live_check.sh --require          # no camera = FAIL
#
# Exit: 0 PASS | 1 FAIL (camera there but wrong) | 2 usage | 3 NOT CONNECTED
# ===========================================================================
set -euo pipefail

FLD_TAG="01-camera"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/common.sh"

# OV9281 native geometry -- same constants as pi_capture.py DEF_WIDTH/DEF_HEIGHT
# (docs/camera_paper_check.md item 2: 1280x800, NOT the sim's 1280x960).
WIDTH=1280
HEIGHT=800
EXPOSURE_US=1000          # the spec target, not a tuning knob (see header)
N_FRAMES=30               # enough to see a rate + a couple of good frames
MODE="auto"
PI_HOST="$FIELD_PI_HOST"
LOCAL_DEV=""
# No-hardware fixture: 252 sim frames that really contain a tag36h11, shipped
# with the repo (scripts/seeker/data/autolabel_sim_*/images). 1280x960 sim
# camera, so resolution is NOT graded in replay mode.
REPLAY_DIR="$REPO_ROOT/scripts/seeker/data/autolabel_sim_20260721T032324Z/images"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pi)
            MODE="pi"
            if [[ -n "${2:-}" && "${2:0:1}" != "-" ]]; then PI_HOST="$2"; shift 2
            else PI_HOST="${PI_HOST:-$FIELD_PI_DEFAULT_HOST}"; shift; fi ;;
        --local)
            MODE="local"
            if [[ -n "${2:-}" && "${2:0:1}" != "-" ]]; then LOCAL_DEV="$2"; shift 2
            else shift; fi ;;
        --replay)
            MODE="replay"
            if [[ -n "${2:-}" && "${2:0:1}" != "-" ]]; then REPLAY_DIR="$2"; shift 2
            else shift; fi ;;
        --frames)      N_FRAMES="${2:?}"; shift 2 ;;
        --exposure-us) EXPOSURE_US="${2:?}"; shift 2 ;;
        --width)       WIDTH="${2:?}"; shift 2 ;;
        --height)      HEIGHT="${2:?}"; shift 2 ;;
        --require)     FLD_REQUIRE=1; shift ;;
        -h|--help)     fld_usage_from_header "$(readlink -f "${BASH_SOURCE[0]}")"; exit 0 ;;
        *) fld_bad "unknown arg: $1"; fld_finish "$FLD_USAGE" "run with -h for usage" ;;
    esac
done

PY="$(fld_python)" || fld_finish "$FLD_USAGE" "no usable python venv"
CAPTURE="$REPO_ROOT/scripts/seeker/pi_capture.py"
SUMMARY="$FLD_DIR/camera_session_summary.py"
[[ -f "$CAPTURE" ]] || fld_finish "$FLD_USAGE" "missing $CAPTURE"
[[ -f "$SUMMARY" ]] || fld_finish "$FLD_USAGE" "missing $SUMMARY"

LOG_DIR="$(fld_log_dir 01_camera)"
SESSION="$LOG_DIR/session"
LOG="$LOG_DIR/report.txt"
exec > >(tee "$LOG") 2>&1

fld_say "log -> $LOG"

# ------------------------------------------------------- pick a mode (auto)
PI_TRY="${PI_HOST:-$FIELD_PI_DEFAULT_HOST}"
if [[ "$MODE" == "auto" ]]; then
    fld_head "choosing a source"
    if fld_pi_reachable "$PI_TRY"; then
        fld_ok "Pi 5 reachable at $PI_TRY -> real camera check"
        MODE="pi"; PI_HOST="$PI_TRY"
    else
        mapfile -t VIDEOS < <(fld_video_devices)
        if ((${#VIDEOS[@]} > 0)); then
            fld_ok "local camera ${VIDEOS[0]} -> desk rehearsal"
            MODE="local"; LOCAL_DEV="${VIDEOS[0]}"
        else
            fld_miss "no Pi at '$PI_TRY' and no /dev/video* here"
            fld_hint "falling back to the NO-HARDWARE replay dry run"
            MODE="replay"
            ABSENT_AFTER_REPLAY=1
        fi
    fi
fi

RC=0
EXPECT_RES="${WIDTH}x${HEIGHT}"

case "$MODE" in
# ---------------------------------------------------------------- Pi (real)
pi)
    fld_head "capturing on the Pi 5 ($PI_TRY)"
    if ! fld_pi_reachable "$PI_TRY"; then
        fld_bad "cannot ssh to $PI_TRY"
        fld_finish "$FLD_ABSENT" \
            "power the Pi, join it to Wi-Fi, enable SSH (Pi Imager advanced options)" \
            "provision it: scripts/pi_setup/provision.sh --hostname interceptor-seeker" \
            "or point at it explicitly: --pi pi@<ip>"
    fi
    REMOTE_OUT="$FIELD_PI_REPO/sessions/fieldcheck_$(date -u +%Y%m%dT%H%M%SZ)"
    fld_say "remote session: $REMOTE_OUT"
    # Runs the SAME pi_capture.py, on the Pi, with the picamera2 backend. The
    # Pi-side venv/repo paths come from scripts/pi_setup/README.md (clone at
    # ~/interceptor-sim, venv .venv-pi); override with FIELD_PI_REPO/FIELD_PI_PYTHON.
    REMOTE_CMD="cd $FIELD_PI_REPO && $FIELD_PI_PYTHON scripts/seeker/pi_capture.py \
--source picamera2 --out $REMOTE_OUT --n-frames $N_FRAMES --exposure-us $EXPOSURE_US \
--width $WIDTH --height $HEIGHT --run-tag fieldcheck"
    fld_say "\$ ssh $PI_TRY '$REMOTE_CMD'"
    if ! ssh -o BatchMode=yes -o ConnectTimeout="${FIELD_SSH_TIMEOUT:-6}" \
             "$PI_TRY" "$REMOTE_CMD" 2>&1 | sed "s/^/[$FLD_TAG]   | /"; then
        fld_bad "the capture command failed on the Pi"
        fld_finish "$FLD_FAIL" \
            "check the ribbon seating (22-pin end at the Pi) and 'rpicam-hello --list-cameras' on the Pi" \
            "re-run provisioning if the overlay is missing: scripts/pi_setup/provision.sh (dtoverlay=ov9281)" \
            "full walkthrough: scripts/pi_setup/README.md step 2"
    fi
    mkdir -p "$SESSION"
    fld_say "copying the session back to $SESSION"
    rsync -a "$PI_TRY:$REMOTE_OUT/" "$SESSION/" \
        || fld_finish "$FLD_FAIL" "rsync from the Pi failed -- is rsync installed on the Pi?"
    ;;

# ------------------------------------------------------- local USB/UVC camera
local)
    fld_head "capturing from a local camera"
    if [[ -z "$LOCAL_DEV" ]]; then
        mapfile -t VIDEOS < <(fld_video_devices)
        ((${#VIDEOS[@]} > 0)) && LOCAL_DEV="${VIDEOS[0]}"
    fi
    if [[ -z "$LOCAL_DEV" || ! -e "$LOCAL_DEV" ]]; then
        fld_miss "no /dev/video* on this machine"
        fld_finish "$FLD_ABSENT" \
            "the seeker OV9281 is CSI -> it lives on the Pi; use --pi instead" \
            "on WSL a USB webcam also needs: usbipd bind/attach (see docs/field_bringup.md)"
    fi
    fld_ok "device $LOCAL_DEV"
    fld_warn "V4L2 exposure units are device-specific -> the <=1 ms spec is NOT graded here"
    "$PY" "$CAPTURE" --source v4l2 --device "$LOCAL_DEV" --out "$SESSION" \
        --n-frames "$N_FRAMES" --exposure-us "$EXPOSURE_US" \
        --width "$WIDTH" --height "$HEIGHT" --run-tag fieldcheck \
        2>&1 | sed "s/^/[$FLD_TAG]   | /" || RC=1
    if ((RC)); then
        fld_finish "$FLD_FAIL" "camera opened but no frames -- try a different --local /dev/videoN"
    fi
    # A random webcam is not the OV9281, so its resolution is not graded.
    EXPECT_RES="any"
    ;;

# ------------------------------------------------------------ replay dry run
replay)
    fld_head "NO-HARDWARE dry run (dir replay)"
    if [[ ! -d "$REPLAY_DIR" ]]; then
        fld_bad "replay dir not found: $REPLAY_DIR"
        fld_finish "$FLD_USAGE" "pass one with --replay <DIR-of-images>"
    fi
    fld_say "replaying $REPLAY_DIR"
    fld_say "this proves the RECORDER + SESSION LAYOUT only -- no real optics, no exposure"
    "$PY" "$CAPTURE" --source "dir=$REPLAY_DIR" --out "$SESSION" \
        --n-frames "$N_FRAMES" --run-tag fieldcheck-replay \
        2>&1 | sed "s/^/[$FLD_TAG]   | /" || RC=1
    ((RC)) && fld_finish "$FLD_FAIL" "the replay backend itself failed -- that is a software bug, not hardware"
    EXPECT_RES="any"     # the fixture is the 1280x960 sim camera, not the OV9281
    ;;
esac

# --------------------------------------------------------------- grade it
fld_head "grading the session"
set +e
"$PY" "$SUMMARY" "$SESSION" \
    --expect-res "$EXPECT_RES" --min-frames 1 \
    --sample-out "$LOG_DIR/sample_frame.png" \
    --json-out "$LOG_DIR/verdict.json" 2>&1 | sed "s/^/[$FLD_TAG]   | /"
SUM_RC="${PIPESTATUS[0]}"
set -e

if [[ "$SUM_RC" -ne 0 ]]; then
    fld_finish "$FLD_FAIL" \
        "read the BAD line(s) above" \
        "wrong resolution -> check the sensor overlay on the Pi (dtoverlay=ov9281) and --width/--height" \
        "exposure over spec -> more light, or set FrameDurationLimits (docs/camera_paper_check.md item 3)" \
        "session kept at $SESSION"
fi

if [[ "${ABSENT_AFTER_REPLAY:-0}" == "1" ]]; then
    fld_finish "$FLD_ABSENT" \
        "the recorder pipeline is healthy, but NO REAL CAMERA was checked" \
        "plug the OV9281 ribbon into the Pi 5 (22-pin end at the Pi) and re-run with --pi" \
        "sample frame: $LOG_DIR/sample_frame.png"
fi

fld_finish "$FLD_OK" \
    "sample frame: $LOG_DIR/sample_frame.png (open it -- is it sharp, level, well lit?)" \
    "next step: scripts/field/02_apriltag_desk_check.sh --session $SESSION"
