#!/usr/bin/env bash
# ===========================================================================
# 04_pull_logs.sh -- "get everything off the aircraft after a run" -- step 4 of
# the field bring-up flow (runbook: docs/field_bringup.md).
#
# WHY IT MATTERS: under the binary-kill re-scope the EVIDENCE is the onboard
# video plus BOTH flight-controller logs (docs/hardware_order_list.md 0c) --
# there is no RTK, so a run that loses its logs is a run that did not happen.
# This script gathers the three sources into one timestamped folder under
# logs/field/ and, when both aircraft's logs are present, runs the scorer.
#
# THE THREE SOURCES (docs/project_state.json build_tab):
#   interceptor  Pixhawk 6C Mini (PX4)     -> ULog  .ulg  on its microSD  (log/<date>/*.ulg)
#   target       Kakute H7 (ArduPilot)     -> DataFlash .BIN on its microSD (APM/LOGS/*.BIN)
#   seeker       Raspberry Pi 5 + OV9281   -> a pi_capture.py session dir over the network
# The microSD cards come off the aircraft into the card reader; in WSL2 the
# reader shows up as a Windows drive under /mnt (e.g. /mnt/d), no usbipd needed
# for a plain card reader.
#
# SCORING: with both a .ulg and a .BIN in hand, scripts/field_score.py computes
# the range history and the closest point of approach and calls KILL/MISS
# against a lethal radius (default 1.5 m, its DEFAULT_LETHAL_RADIUS_M). That is
# the supporting kinematic half of the evidence -- the video is still the call.
#
# Usage:
#   scripts/field/04_pull_logs.sh                          # auto-scan /mnt drives
#   scripts/field/04_pull_logs.sh --px4 /mnt/d --ardupilot /mnt/e
#   scripts/field/04_pull_logs.sh --pi pi@interceptor-seeker.local
#   scripts/field/04_pull_logs.sh --session tripod_pass01 --newest 3
#   scripts/field/04_pull_logs.sh --no-score               # copy only
#
# Exit: 0 PASS (something was pulled) | 1 FAIL (copy/scoring error)
#       | 2 usage | 3 NOT CONNECTED (nothing to pull)
# ===========================================================================
set -euo pipefail

FLD_TAG="04-logs"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/common.sh"

PX4_SRC=""
ARDU_SRC=""
PI_HOST="$FIELD_PI_HOST"
SESSION_NAME=""
NEWEST=1
DO_SCORE=1
LETHAL_RADIUS=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --px4)        PX4_SRC="${2:?}"; shift 2 ;;
        --ardupilot)  ARDU_SRC="${2:?}"; shift 2 ;;
        --pi)         PI_HOST="${2:-$FIELD_PI_DEFAULT_HOST}"; shift 2 ;;
        --session)    SESSION_NAME="${2:?}"; shift 2 ;;
        --newest)     NEWEST="${2:?}"; shift 2 ;;
        --lethal-radius) LETHAL_RADIUS="${2:?}"; shift 2 ;;
        --no-score)   DO_SCORE=0; shift ;;
        --require)    FLD_REQUIRE=1; shift ;;
        -h|--help)    fld_usage_from_header "$(readlink -f "${BASH_SOURCE[0]}")"; exit 0 ;;
        *) fld_bad "unknown arg: $1"; fld_finish "$FLD_USAGE" "run with -h for usage" ;;
    esac
done

PY="$(fld_python)" || fld_finish "$FLD_USAGE" "no usable python venv"
SCORER="$REPO_ROOT/scripts/field_score.py"

OUT="$FLD_LOGS/${SESSION_NAME:-run_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$OUT"/{interceptor,target,seeker}
LOG="$OUT/pull.txt"
exec > >(tee "$LOG") 2>&1
fld_say "collecting into $OUT"

PULLED=0

# copy_newest <glob-root> <find-pattern> <dest> <label>
# Copies the N most recently MODIFIED matching files. mtime is the only ordering
# a FAT card reliably gives us, and the newest files are the flight you just did.
copy_newest() {
    local root="$1" pattern="$2" dest="$3" label="$4"
    local -a files=()
    mapfile -t files < <(find "$root" -type f -iname "$pattern" -printf '%T@ %p\n' 2>/dev/null \
                         | sort -rn | head -n "$NEWEST" | cut -d' ' -f2-)
    if ((${#files[@]} == 0)); then
        fld_miss "no $label ($pattern) under $root"
        return 1
    fi
    local f
    for f in "${files[@]}"; do
        cp -v "$f" "$dest/" | sed "s/^/[$FLD_TAG]   | /"
        PULLED=$((PULLED + 1))
    done
    fld_ok "$label: ${#files[@]} file(s) from $root"
    return 0
}

# ------------------------------------------------------ auto-find the cards
fld_head "1. flight-controller log cards"
if [[ -z "$PX4_SRC" && -z "$ARDU_SRC" ]]; then
    for m in /mnt/[d-z]; do
        [[ -d "$m" ]] || continue
        if [[ -z "$PX4_SRC" ]] && compgen -G "$m/log/*" >/dev/null 2>&1; then
            PX4_SRC="$m"; fld_ok "PX4 card auto-detected at $m (log/ present)"
        elif [[ -z "$ARDU_SRC" ]] && compgen -G "$m/APM/LOGS/*" >/dev/null 2>&1; then
            ARDU_SRC="$m"; fld_ok "ArduPilot card auto-detected at $m (APM/LOGS/ present)"
        fi
    done
fi

ULOG=""; BINLOG=""
if [[ -n "$PX4_SRC" ]]; then
    if [[ -d "$PX4_SRC" ]]; then
        copy_newest "$PX4_SRC" '*.ulg' "$OUT/interceptor" "interceptor ULog" || true
        ULOG="$(ls -1t "$OUT/interceptor"/*.ulg 2>/dev/null | head -1 || true)"
    else
        fld_bad "--px4 path not found: $PX4_SRC"
    fi
else
    fld_miss "no PX4 card (interceptor ULog) -- insert the 6C Mini's microSD in the reader"
fi
if [[ -n "$ARDU_SRC" ]]; then
    if [[ -d "$ARDU_SRC" ]]; then
        copy_newest "$ARDU_SRC" '*.bin' "$OUT/target" "target DataFlash log" || true
        BINLOG="$(ls -1t "$OUT/target"/*.[bB][iI][nN] 2>/dev/null | head -1 || true)"
    else
        fld_bad "--ardupilot path not found: $ARDU_SRC"
    fi
else
    fld_miss "no ArduPilot card (target .BIN) -- insert the Kakute's microSD in the reader"
fi

# ------------------------------------------------------- the camera session
fld_head "2. seeker camera session (from the Pi)"
PI_TRY="${PI_HOST:-$FIELD_PI_DEFAULT_HOST}"
if fld_pi_reachable "$PI_TRY"; then
    # Newest session directory under the Pi's sessions/ folder (pi_capture.py's
    # --out convention; scripts/pi_setup/README.md step 3).
    REMOTE_SESS="$(ssh -o BatchMode=yes -o ConnectTimeout="${FIELD_SSH_TIMEOUT:-6}" "$PI_TRY" \
        "ls -1dt $FIELD_PI_REPO/sessions/*/ 2>/dev/null | head -1" || true)"
    REMOTE_SESS="${REMOTE_SESS%$'\r'}"
    if [[ -n "$REMOTE_SESS" ]]; then
        fld_ok "newest Pi session: $REMOTE_SESS"
        if rsync -a --info=stats1 "$PI_TRY:$REMOTE_SESS" "$OUT/seeker/" 2>&1 | sed "s/^/[$FLD_TAG]   | /"; then
            PULLED=$((PULLED + 1))
        else
            fld_bad "rsync from the Pi failed"
        fi
    else
        fld_miss "no session directories under $FIELD_PI_REPO/sessions/ on the Pi"
    fi
else
    fld_miss "Pi not reachable at $PI_TRY -- skipping the camera session"
fi

# --------------------------------------------------------------- score it
fld_head "3. scoring"
SCORED=0
if ((DO_SCORE)) && [[ -n "$ULOG" && -n "$BINLOG" && -f "$SCORER" ]]; then
    fld_say "both aircraft logs present -> scripts/field_score.py"
    ARGS=(--ulog-a "$ULOG" --bin-b "$BINLOG" --label-a interceptor --label-b target
          --out-dir "$OUT/score")
    [[ -n "$LETHAL_RADIUS" ]] && ARGS+=(--lethal-radius "$LETHAL_RADIUS")
    set +e
    "$PY" "$SCORER" "${ARGS[@]}" 2>&1 | sed "s/^/[$FLD_TAG]   | /"
    SC_RC="${PIPESTATUS[0]}"
    set -e
    if [[ "$SC_RC" -eq 0 ]]; then
        SCORED=1
        fld_ok "score written to $OUT/score"
    else
        SCORE_FAILED=1
        fld_bad "field_score.py exited $SC_RC (logs are still safely copied)"
    fi
elif ((DO_SCORE)); then
    fld_miss "need BOTH an interceptor .ulg and a target .BIN to score -- skipped"
    fld_hint "you can score later: $PY scripts/field_score.py --ulog-a <A.ulg> --bin-b <B.BIN>"
fi

# ---------------------------------------------------------------- verdict
fld_head "SUMMARY"
# Drop the per-source folders that stayed empty, so the run folder shows only
# what was really collected.
for d in "$OUT/interceptor" "$OUT/target" "$OUT/seeker"; do
    rmdir "$d" 2>/dev/null || true
done
fld_say "collected: $PULLED item(s) -> $OUT"
if ((PULLED == 0)); then
    fld_finish "$FLD_ABSENT" \
        "nothing found to pull" \
        "put the FC microSD cards in the reader (they mount as a Windows drive under /mnt)" \
        "power the Pi and re-run to fetch the camera session" \
        "or point at the sources: --px4 /mnt/d --ardupilot /mnt/e --pi pi@interceptor-seeker.local"
fi
((SCORED)) && fld_finish "$FLD_OK" "kinematic verdict + plots: $OUT/score" \
    "the video is still the call -- the score is the number behind it"
if [[ "${SCORE_FAILED:-0}" == "1" ]]; then
    fld_finish "$FLD_OK" \
        "THE COPY SUCCEEDED but SCORING FAILED -- see the traceback above" \
        "your logs are safe under $OUT; only the automatic score is missing" \
        "usual cause: a truncated/corrupt log, or the wrong file picked -- retry with an explicit path:" \
        "  scripts/field_score.py --ulog-a <A.ulg> --bin-b <B.BIN>"
fi
fld_finish "$FLD_OK" "everything is under $OUT" \
    "score later with scripts/field_score.py once both aircraft logs are in hand"
