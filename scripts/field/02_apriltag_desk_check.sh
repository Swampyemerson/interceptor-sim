#!/usr/bin/env bash
# ===========================================================================
# 02_apriltag_desk_check.sh -- "does the baseline seeker actually see the tag?"
# Step 2 of the field bring-up flow (runbook: docs/field_bringup.md).
#
# WHAT IT IS: hold the printed AprilTag placard a measured distance from the
# camera, capture, and confirm the detector decodes it -- plus how many pixels
# the tag covered, where it sat in frame, and what range its pose implies.
# This is the Pi-5-CPU BASELINE SEEKER sanity check: first real kills fly the
# TAG, not the neural net (docs/hardware_order_list.md 0c; project_state
# constraint `pi5-compute`), so a tag that will not decode is a stop-work.
#
# ONE CODE PATH: decoding is done by scripts/seeker/pi_capture.py --decode-tags
# (the same recorder + the same tag36h11 detector settings the field day uses);
# this script only feeds it frames and hands the resulting tags.csv to
# scripts/field/apriltag_decode_summary.py.
#
# INPUTS (first match wins):
#   --session DIR   a session from 01_camera_live_check.sh (uses its frames/)
#   --frames DIR    any directory of images
#   (nothing)       the newest logs/field/01_camera_*/session, else the repo's
#                   sim tag frames -> a NO-HARDWARE dry run (exit 3)
#
# CALIBRATION (this decides whether RANGE means anything):
#   --calib FILE, else camera_intrinsics_real.json (calibrate_camera.py output
#   for the REAL lens) if it exists -> range is GRADED; else the session's own
#   calib.json or the SIM configs/camera_intrinsics.json -> range printed INDICATIVE
#   ONLY. Sim intrinsics on real pixels give a silently wrong range: see the
#   header of scripts/calibrate_camera.py.
#
# TAG SIZE: --tag-size, default 0.35 m -- the black-square edge of the flight
# placard, sized by P0.2 (docs/placard_sizing.md: 0.35 m is the carry limit and
# the only size clearing the t_go money gate). The repo's SIM tag frames are a
# 0.5 m tag (models/apriltag_target/model.sdf), so the dry run overrides it.
#
# Usage:
#   scripts/field/02_apriltag_desk_check.sh                       # auto / dry run
#   scripts/field/02_apriltag_desk_check.sh --session logs/field/01_camera_.../session
#   scripts/field/02_apriltag_desk_check.sh --frames /path/to/images --expect-range 2.0
#   scripts/field/02_apriltag_desk_check.sh --calib camera_intrinsics_real.json \
#        --tag-size 0.35 --expect-range 3.0 --require
#
# Exit: 0 PASS | 1 FAIL (frames there, tag did not decode) | 2 usage | 3 NOT CONNECTED
# ===========================================================================
set -euo pipefail

FLD_TAG="02-apriltag"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/common.sh"

SESSION_IN=""
FRAMES_IN=""
CALIB=""
TAG_SIZE="0.35"          # flight placard edge, docs/placard_sizing.md
EXPECT_RANGE=""
MIN_DECODED=1
N_FRAMES=0               # 0 = all frames in the dir
DRY=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --session)      SESSION_IN="${2:?}"; shift 2 ;;
        --frames)       FRAMES_IN="${2:?}"; shift 2 ;;
        --calib)        CALIB="${2:?}"; shift 2 ;;
        --tag-size)     TAG_SIZE="${2:?}"; shift 2 ;;
        --expect-range) EXPECT_RANGE="${2:?}"; shift 2 ;;
        --min-decoded)  MIN_DECODED="${2:?}"; shift 2 ;;
        --n-frames)     N_FRAMES="${2:?}"; shift 2 ;;
        --require)      FLD_REQUIRE=1; shift ;;
        -h|--help)      fld_usage_from_header "$(readlink -f "${BASH_SOURCE[0]}")"; exit 0 ;;
        *) fld_bad "unknown arg: $1"; fld_finish "$FLD_USAGE" "run with -h for usage" ;;
    esac
done

PY="$(fld_python)" || fld_finish "$FLD_USAGE" "no usable python venv"
CAPTURE="$REPO_ROOT/scripts/seeker/pi_capture.py"
SUMMARY="$FLD_DIR/apriltag_decode_summary.py"
[[ -f "$CAPTURE" ]] || fld_finish "$FLD_USAGE" "missing $CAPTURE"
[[ -f "$SUMMARY" ]] || fld_finish "$FLD_USAGE" "missing $SUMMARY"

LOG_DIR="$(fld_log_dir 02_apriltag)"
LOG="$LOG_DIR/report.txt"
exec > >(tee "$LOG") 2>&1
fld_say "log -> $LOG"

# A session recorded by the dir-replay backend carries NO camera information
# (pi_capture.py meta.json: source == "dir"). Decoding it again tells you
# nothing about real optics, and its physical tag size is whatever produced the
# original frames -- so we must not silently grade ranges off it.
session_is_replay() {
    local mj="$1/meta.json"
    [[ -f "$mj" ]] || return 1
    grep -q '"source"[[:space:]]*:[[:space:]]*"dir"' "$mj"
}

# ------------------------------------------------------------- pick frames
fld_head "1. frames to decode"
FRAMES=""
if [[ -n "$FRAMES_IN" ]]; then
    FRAMES="$FRAMES_IN"
    fld_ok "using --frames $FRAMES"
elif [[ -n "$SESSION_IN" ]]; then
    FRAMES="$SESSION_IN/frames"
    fld_ok "using --session $SESSION_IN"
    if session_is_replay "$SESSION_IN"; then
        DRY=1
        fld_warn "that session is a REPLAY (no real camera) -- treating this as a dry run"
        fld_hint "its range numbers depend on the tag size that made the ORIGINAL frames"
    fi
else
    # Newest REAL camera session from step 01. We scan newest-first and take the
    # first non-replay, rather than testing only the single newest.
    #
    # BUG FIXED 2026-07-25 (audit hygiene): this used to `head -1` and, if THAT
    # one session was a replay, blank it and fall all the way through to the sim
    # fixture dry run -- silently discarding a perfectly good REAL capture that
    # was sitting one directory older. That is a wrong-and-confident answer (it
    # reports "no captured session found" while one exists), and it was easy to
    # trigger because scripts/field/selftest.sh's replay landed in this very
    # directory on every run_tests.sh invocation. A real capture now always wins
    # over a newer replay, however many replays are stacked on top of it.
    LATEST=""
    _skipped_replays=0
    while IFS= read -r _cand; do
        [[ -n "$_cand" ]] || continue
        if session_is_replay "$_cand"; then
            _skipped_replays=$((_skipped_replays + 1))
            continue
        fi
        [[ -d "$_cand/frames" ]] || continue
        LATEST="$_cand"
        break
    done < <(ls -1dt "$FLD_LOGS"/01_camera_*/session 2>/dev/null || true)
    if [[ "$_skipped_replays" -gt 0 ]]; then
        fld_say "skipped $_skipped_replays newer REPLAY session(s) looking for a real capture"
    fi
    if [[ -z "$LATEST" && "$_skipped_replays" -gt 0 ]]; then
        fld_miss "every camera session under $FLD_LOGS is a replay, not a real capture"
    fi
    if [[ -n "$LATEST" && -d "$LATEST/frames" ]]; then
        FRAMES="$LATEST/frames"
        fld_ok "using the newest camera session: $LATEST"
    else
        FRAMES="$REPO_ROOT/scripts/seeker/data/autolabel_sim_20260721T032324Z/images"
        # scripts/seeker/data/ is GITIGNORED: fall back to the TRACKED 32-frame
        # subset so the dry run works on a clean clone (see 01_camera_live_check.sh's
        # note -- this was the third cause of the 73/73 red CI streak).
        [[ -d "$FRAMES" ]] || FRAMES="$REPO_ROOT/tests/fixtures/apriltag_sim_frames"
        TAG_SIZE="0.5"          # the sim tag is 0.5 m (models/apriltag_target/model.sdf)
        DRY=1
        # The fixture holds 252 frames; a dry run only needs enough to prove the
        # decode path, so cap it unless the operator asked for a specific count.
        [[ "$N_FRAMES" == "0" ]] && N_FRAMES=24
        fld_miss "no captured session found"
        fld_hint "falling back to the repo's SIM tag frames -- a NO-HARDWARE dry run"
    fi
fi
if [[ ! -d "$FRAMES" ]]; then
    fld_bad "frames dir not found: $FRAMES"
    fld_finish "$FLD_USAGE" "run scripts/field/01_camera_live_check.sh first, or pass --frames DIR"
fi

# -------------------------------------------------------------- pick calib
fld_head "2. camera intrinsics (decides whether RANGE is meaningful)"
CALIB_TRUSTED=0
if [[ -n "$CALIB" ]]; then
    fld_ok "using --calib $CALIB"
    CALIB_TRUSTED=1
elif [[ -f "$REPO_ROOT/camera_intrinsics_real.json" ]]; then
    CALIB="$REPO_ROOT/camera_intrinsics_real.json"
    CALIB_TRUSTED=1
    fld_ok "using the REAL-lens calibration: $CALIB"
elif [[ -n "$SESSION_IN" && -f "$SESSION_IN/calib.json" ]]; then
    CALIB="$SESSION_IN/calib.json"
    fld_warn "using the session's calib.json -- range is INDICATIVE only"
elif [[ -f "$(dirname "$FRAMES")/calib.json" ]]; then
    CALIB="$(dirname "$FRAMES")/calib.json"
    fld_warn "using $CALIB -- range is INDICATIVE only"
else
    CALIB="$REPO_ROOT/camera_intrinsics.json"
    fld_warn "falling back to the SIM camera intrinsics ($CALIB) -- range is INDICATIVE only"
fi
if ((CALIB_TRUSTED == 0)); then
    fld_hint "to grade range: run scripts/calibrate_camera.py on the real camera"
    fld_hint "  (>=15 checkerboard views, RMS <= 1.0 px -> camera_intrinsics_real.json)"
fi
[[ -f "$CALIB" ]] || fld_finish "$FLD_USAGE" "calibration file not found: $CALIB"

# ---------------------------------------------------------------- decode
fld_head "3. decoding (scripts/seeker/pi_capture.py --decode-tags)"
SESSION="$LOG_DIR/session"
fld_say "tag family tag36h11, assumed tag edge ${TAG_SIZE} m"
CAP_ARGS=(--source "dir=$FRAMES" --out "$SESSION" --decode-tags
          --calib "$CALIB" --tag-size "$TAG_SIZE" --run-tag deskcheck)
[[ "$N_FRAMES" != "0" ]] && CAP_ARGS+=(--n-frames "$N_FRAMES")
set +e
"$PY" "$CAPTURE" "${CAP_ARGS[@]}" 2>&1 | sed "s/^/[$FLD_TAG]   | /"
CAP_RC="${PIPESTATUS[0]}"
set -e
if [[ "$CAP_RC" -ne 0 ]]; then
    fld_finish "$FLD_FAIL" \
        "the decoder run itself failed (software, not the tag)" \
        "check that the venv has pupil-apriltags: $PY -c 'import pupil_apriltags'"
fi

# ---------------------------------------------------------------- grade it
fld_head "4. verdict"
SUM_ARGS=("$SESSION" --min-decoded "$MIN_DECODED" --json-out "$LOG_DIR/verdict.json")
[[ -n "$EXPECT_RANGE" ]] && SUM_ARGS+=(--expect-range "$EXPECT_RANGE")
((CALIB_TRUSTED)) && SUM_ARGS+=(--calib-trusted)
set +e
"$PY" "$SUMMARY" "${SUM_ARGS[@]}" 2>&1 | sed "s/^/[$FLD_TAG]   | /"
SUM_RC="${PIPESTATUS[0]}"
set -e

if [[ "$SUM_RC" -ne 0 ]]; then
    fld_finish "$FLD_FAIL" \
        "no decode (or the range was out of tolerance) -- read the BAD line above" \
        "closer / bigger tag: decode range scales with the printed tag edge" \
        "light it: a short (<=1 ms) exposure needs daylight or more gain" \
        "focus the M12 lens ring; check the print is not glossy or warped" \
        "confirm the family is tag36h11 and the print is not scaled by the printer" \
        "session kept at $SESSION"
fi

if ((DRY)); then
    fld_finish "$FLD_ABSENT" \
        "decode pipeline is healthy, but this was the SIM-FRAME dry run" \
        "print the 0.35 m tag36h11 placard (docs/placard_sizing.md), run step 01, then re-run this" \
        "results: $LOG_DIR/verdict.json"
fi

fld_finish "$FLD_OK" \
    "results: $LOG_DIR/verdict.json" \
    "the real decode-vs-range CURVE (the money gate) is scripts/seeker/tripod_score.py" \
    "next step: scripts/field/03_fc_bench_check.sh"
