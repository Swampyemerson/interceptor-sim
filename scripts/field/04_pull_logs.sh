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
# against field_score.py's DEFAULT_LETHAL_RADIUS_M -- see that constant for the
# mechanism note (ADR-0025 kinetic-ram radius; override with --lethal-radius,
# which this script forwards). The number is deliberately NOT restated here so
# it can never drift again. That is the supporting kinematic half of the
# evidence -- the video is still the call.
#
# Usage:
#   scripts/field/04_pull_logs.sh                          # auto-scan /mnt drives
#   scripts/field/04_pull_logs.sh --px4 /mnt/d --ardupilot /mnt/e
#   scripts/field/04_pull_logs.sh --pi pi@192.168.43.17    # IP, not .local (WSL2)
#   scripts/field/04_pull_logs.sh --session tripod_pass01 --newest 3
#   scripts/field/04_pull_logs.sh --no-score               # copy only
#
# Exit: 0 PASS (every requested source came off the aircraft AND parsed)
#       1 FAIL (a requested source FAILED to copy, landed EMPTY/TRUNCATED/
#               unreadable, or scoring crashed -- everything already copied is
#               still SAFE under the run folder; do not wipe the cards, re-run)
#       2 usage
#       3 NOT CONNECTED / UNCERTAIN (nothing to pull -- a source that was simply
#               not inserted is advisory, never FAIL; also returned when the
#               scorer wrote a report but REFUSED to call KILL/MISS. See
#               common.sh's contract; --require promotes it to FAIL.)
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
# PULL_FAILED: "a source we were ASKED for did not come off the aircraft (or the
# scorer crashed)". This is the ONLY thing that turns this script's verdict into
# FAIL -- a card that was simply not inserted stays advisory (fld_miss/ABSENT),
# per the exit contract in common.sh:11-22.
PULL_FAILED=0
SCORE_FAILED=0
SCORE_UNCERTAIN=0

# verify_log <landed-file> <source-file>
# "THE LOG CAME OFF THE AIRCRAFT" IS NOT "cp EXITED 0" (2026-07-26). Three real
# post-impact shapes all survive a successful copy and used to be reported PASS:
#   * a 0-byte .ulg -- the FC lost power before its first flush;
#   * a header-only / truncated log -- PX4 fsyncs the ULog at 1 Hz, so the last
#     second of a ram is routinely missing and a very short log is plausible;
#   * a STALE log -- this flight never recorded, so `find | sort -rn` picks the
#     PREVIOUS sortie, which is non-empty, parses fine, and scores fine.
# So: assert the byte count matches the card, then actually PARSE the file and
# require real position samples in it. The first two are then caught outright.
# The third cannot be caught by any check the script can make -- so we PRINT the
# numbers (bytes, sample count, flight duration, first fix time, card mtime); the
# operator is the only one who can spot "that's yesterday's log", and he can only
# spot it if it is on the screen.
verify_log() {
    local dst="$1" src="$2" base; base="$(basename "$dst")"
    local sz_d sz_s
    sz_d="$(stat -c %s "$dst" 2>/dev/null || echo 0)"
    sz_s="$(stat -c %s "$src" 2>/dev/null || echo 0)"
    if ((sz_d == 0)); then
        fld_bad "EMPTY LOG (0 bytes): $base -- this flight recorded NOTHING"
        fld_hint "do NOT wipe the card; check the FC's SD card and LOG_BITMASK/SDLOG_MODE"
        return 1
    fi
    if ((sz_d != sz_s)); then
        fld_bad "TRUNCATED COPY: $base is $sz_d B but the card holds $sz_s B"
        fld_hint "do NOT wipe the card; free disk space and re-pull"
        return 1
    fi
    local out rc=0
    if ! out="$("$PY" - "$dst" <<'PYEOF' 2>&1
# Probe: does this file PARSE, and does it contain real position samples? Reads
# the same messages field_score.py scores from, so "verified" here means "the
# scorer will find a track in it". The .BIN scan is CAPPED (DFReader is pure
# python and a 10-minute DataFlash log is tens of MB): the cap proves the log
# holds flight data, it is not a full read.
import os
import sys
import datetime

BIN_SCAN_CAP = 2000          # position messages; ~3 min at ArduPilot's 10 Hz POS

path = sys.argv[1]
ext = os.path.splitext(path)[1].lower()


def die(msg):
    print(msg)
    raise SystemExit(1)


def stamp(utc_s):
    if not utc_s or utc_s < 1.0e9:      # boot-relative, not a UTC epoch
        return "no GPS UTC fix"
    return datetime.datetime.fromtimestamp(
        utc_s, datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


if ext == ".ulg":
    try:
        from pyulog import ULog
    except ImportError as e:
        die(f"cannot verify: pyulog is not installed ({e})")
    try:
        ulog = ULog(path)
    except Exception as e:
        die(f"UNREADABLE ULog ({type(e).__name__}: {e})")
    n, dur, first = 0, 0.0, None
    for topic in ("vehicle_global_position", "vehicle_gps_position"):
        try:
            d = ulog.get_dataset(topic).data
        except (KeyError, IndexError):
            continue
        t = d["timestamp"]
        n = len(t)
        if n > 1:
            dur = float(t[-1] - t[0]) / 1e6
        utc = d.get("time_utc_usec")
        if utc is not None and len(utc) and float(max(utc)) > 1e15:
            first = float(min(v for v in utc if v > 1e15)) / 1e6
        break
    if n == 0:
        die("NO POSITION SAMPLES: the ULog parses but carries neither "
            "vehicle_global_position nor vehicle_gps_position")
    print(f"{n} position samples, {dur:.1f} s of flight, first fix {stamp(first)}")

elif ext == ".bin":
    try:
        from pymavlink import DFReader
    except ImportError as e:
        die(f"cannot verify: pymavlink is not installed ({e})")
    try:
        reader = DFReader.DFReader_binary(path)
    except Exception as e:
        die(f"UNREADABLE DataFlash log ({type(e).__name__}: {e})")
    n, t0, t1 = 0, None, None
    while n < BIN_SCAN_CAP:
        try:
            m = reader.recv_match(type=["POS", "GPS"])
        except Exception as e:
            die(f"UNREADABLE DataFlash log ({type(e).__name__}: {e}) "
                f"after {n} position message(s)")
        if m is None:
            break
        if getattr(m, "Lat", None) is None:
            continue
        n += 1
        t1 = float(m._timestamp)
        if t0 is None:
            t0 = t1
    if n == 0:
        die("NO POSITION SAMPLES: the .BIN parses but carries no POS/GPS "
            "message with a position (check the ArduPilot LOG_BITMASK)")
    capped = " (scan capped)" if n >= BIN_SCAN_CAP else ""
    print(f"{n} position samples{capped}, {t1 - t0:.1f} s of flight, "
          f"first fix {stamp(t0)}")
else:
    die(f"unknown log extension {ext!r} -- cannot verify")
PYEOF
    )"; then
        rc=1
    fi
    if ((rc != 0)); then
        fld_bad "UNUSABLE LOG: $base -- $out"
        fld_hint "do NOT wipe the card; the flight may still be recoverable from it"
        return 1
    fi
    fld_ok "$base: $sz_d B, $out  [card mtime $(date -u -r "$src" '+%Y-%m-%dT%H:%M:%SZ')]"
    fld_hint "CHECK THAT TIME IS THIS FLIGHT -- a card that did not record leaves the PREVIOUS sortie as the newest log"
    return 0
}

# copy_newest <glob-root> <find-pattern> <dest> <label>
# Copies the N most recently MODIFIED matching files. mtime is the only ordering
# a FAT card reliably gives us, and the newest files are the flight you just did.
# Sets FLD_NEWEST_COPIED to the destination path of the NEWEST source file that
# actually landed AND VERIFIED (see the call sites: re-deriving the newest from
# destination mtimes is wrong, because plain `cp` stamps the COPY time and
# inverts the order).
copy_newest() {
    local root="$1" pattern="$2" dest="$3" label="$4"
    local -a files=()
    FLD_NEWEST_COPIED=""
    mapfile -t files < <(find "$root" -type f -iname "$pattern" -printf '%T@ %p\n' 2>/dev/null \
                         | sort -rn | head -n "$NEWEST" | cut -d' ' -f2-)
    if ((${#files[@]} == 0)); then
        fld_miss "no $label ($pattern) under $root"
        return 1
    fi
    local f copied=0
    for f in "${files[@]}"; do
        # CHECK THE COPY. `set -e` cannot help here: this function is called from
        # an `if`/`||` context, which disables errexit for its whole body -- so an
        # unchecked cp used to print OK + count the file with zero bytes landed.
        # `--preserve=timestamps` keeps the FLIGHT time on the destination, and
        # 2>&1 keeps cp's error inside the tagged stream instead of scrolling past.
        if cp --preserve=timestamps -v "$f" "$dest/" 2>&1 | sed "s/^/[$FLD_TAG]   | /"; then
            # A copy that landed a file the scorer cannot read is NOT a pull.
            # Nothing below counts it, so FLD_NEWEST_COPIED never points the
            # scorer at an empty/truncated log either.
            if ! verify_log "$dest/$(basename "$f")" "$f"; then
                PULL_FAILED=1
                continue
            fi
            copied=$((copied + 1))
            PULLED=$((PULLED + 1))
            if [[ -z "$FLD_NEWEST_COPIED" ]]; then
                FLD_NEWEST_COPIED="$dest/$(basename "$f")"
            fi
        else
            fld_bad "COPY FAILED (nothing landed): $f"
            PULL_FAILED=1
        fi
    done
    # Report what is ON DISK, not what find matched.
    local landed
    landed="$(find "$dest" -maxdepth 1 -type f 2>/dev/null | wc -l)"
    if ((copied < ${#files[@]})); then
        fld_bad "$label: only $copied of ${#files[@]} file(s) copied ($landed now in $dest)"
        PULL_FAILED=1
        return 1
    fi
    fld_ok "$label: $copied file(s) landed (of ${#files[@]} found) from $root"
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
        if copy_newest "$PX4_SRC" '*.ulg' "$OUT/interceptor" "interceptor ULog"; then
            ULOG="$FLD_NEWEST_COPIED"
        fi
    else
        fld_bad "--px4 path not found: $PX4_SRC"
        PULL_FAILED=1
    fi
else
    fld_miss "no PX4 card (interceptor ULog) -- insert the 6C Mini's microSD in the reader"
fi
if [[ -n "$ARDU_SRC" ]]; then
    if [[ -d "$ARDU_SRC" ]]; then
        if copy_newest "$ARDU_SRC" '*.bin' "$OUT/target" "target DataFlash log"; then
            BINLOG="$FLD_NEWEST_COPIED"
        fi
    else
        fld_bad "--ardupilot path not found: $ARDU_SRC"
        PULL_FAILED=1
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
            # SAME FAIL-OPEN AS THE CARDS (2026-07-26): rsync of an EMPTY session
            # dir succeeds. pi_capture.py creates the session directory before the
            # first frame arrives, so a camera that never delivered a frame used
            # to count as a green "seeker session pulled".
            SEEKER_FILES="$(find "$OUT/seeker" -type f 2>/dev/null | wc -l)"
            SEEKER_FRAMES="$(find "$OUT/seeker" -type f \( -iname '*.png' -o -iname '*.jpg' \) 2>/dev/null | wc -l)"
            if ((SEEKER_FILES == 0)); then
                fld_bad "the Pi session copied ZERO files -- the camera captured nothing"
                fld_hint "do NOT wipe the Pi; check pi_capture.py's output and re-pull"
                PULL_FAILED=1
            else
                PULLED=$((PULLED + 1))
                fld_ok "seeker session: $SEEKER_FILES file(s), $SEEKER_FRAMES frame(s) landed"
                ((SEEKER_FRAMES == 0)) && fld_warn "NO image frames in the session -- \
index/metadata only, so there is no seeker evidence for this attempt"
            fi
        else
            fld_bad "rsync from the Pi failed"
            PULL_FAILED=1
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
    # Name the exact PAIR being scored: with --newest > 1 the folder holds more
    # than one sortie, and a silently mis-paired sortie still produces a
    # confident KILL/MISS (the two .BIN/.ulg windows can overlap).
    fld_say "scoring: $(basename "$ULOG")  x  $(basename "$BINLOG")"
    if ((NEWEST > 1)); then
        fld_warn "--newest $NEWEST: only the NEWEST log on each card was scored"
        fld_hint "score another sortie explicitly: $PY scripts/field_score.py --ulog-a <A.ulg> --bin-b <B.BIN>"
    fi
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
    elif [[ "$SC_RC" -eq 3 ]]; then
        # field_score exits 3 when it produced a report but REFUSED to call
        # KILL/MISS (unsynced clocks, a dropout across the CPA, non-finite
        # positions, a truncated overlap). That is a measurement it could not
        # make -- advisory per common.sh's contract, NOT a crash and NOT a FAIL:
        # the logs are safely copied and the report says which condition fired.
        SCORED=1
        SCORE_UNCERTAIN=1
        fld_warn "field_score.py returned INCONCLUSIVE (exit 3) -- a report was "
        fld_warn "written to $OUT/score but it does NOT say KILL or MISS"
        fld_hint "read the verdict + honesty_note in $OUT/score/*.json; the VIDEO is still the call"
    else
        SCORE_FAILED=1
        PULL_FAILED=1
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
# ORDER MATTERS. A FAILED source outranks "nothing was pulled": the ABSENT path
# below means "you did not put a card in", which is advisory; a card that WAS
# requested and did not copy is a FAIL you must act on before wiping anything.
if ((PULL_FAILED)); then
    if ((SCORE_FAILED)) && ((PULLED > 0)); then
        fld_finish "$FLD_FAIL" \
            "THE COPY SUCCEEDED but SCORING CRASHED -- see the traceback above" \
            "ALL COPIED FILES ARE SAFE under $OUT -- do NOT wipe the cards" \
            "usual cause: a truncated/corrupt log, or the wrong file picked -- retry with explicit paths:" \
            "  $PY scripts/field_score.py --ulog-a <A.ulg> --bin-b <B.BIN> --out-dir $OUT/score" \
            "re-pull instead with: scripts/field/04_pull_logs.sh --session $(basename "$OUT")"
    fi
    fld_finish "$FLD_FAIL" \
        "one or more REQUESTED sources failed to come off the aircraft -- do NOT wipe the cards" \
        "ALL COPIED FILES ARE SAFE under $OUT (nothing here deletes a source)" \
        "re-seat the card / check the path / free disk space, then re-run:" \
        "  scripts/field/04_pull_logs.sh --session $(basename "$OUT")"
fi
if ((PULLED == 0)); then
    fld_finish "$FLD_ABSENT" \
        "nothing found to pull" \
        "put the FC microSD cards in the reader (they mount as a Windows drive under /mnt)" \
        "power the Pi and re-run to fetch the camera session" \
        "or point at the sources: --px4 /mnt/d --ardupilot /mnt/e --pi pi@<Pi IP>"
fi
if ((SCORE_UNCERTAIN)); then
    fld_finish "$FLD_ABSENT" \
        "the logs are all safely under $OUT, but the KINEMATIC VERDICT IS INCONCLUSIVE" \
        "field_score refused to call KILL/MISS -- open $OUT/score/*.json and read 'verdict' + 'honesty_note'" \
        "do NOT wipe the cards until you have decided the attempt from the VIDEO"
fi
((SCORED)) && fld_finish "$FLD_OK" "kinematic verdict + plots: $OUT/score" \
    "the video is still the call -- the score is the number behind it"
# NOT VERIFIED on this path: no KILL/MISS number was computed (either --no-score,
# or only one aircraft's log was in hand). PASS here means "the files came off the
# aircraft and parse", nothing about the engagement.
fld_finish "$FLD_OK" "everything is under $OUT -- copied and PARSE-VERIFIED; NO kinematic verdict was computed" \
    "score later with scripts/field_score.py once both aircraft logs are in hand"
