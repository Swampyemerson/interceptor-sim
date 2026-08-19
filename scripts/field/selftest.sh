#!/usr/bin/env bash
# ===========================================================================
# selftest.sh -- offline gate for the scripts/field/ bring-up pack. Exits 0/1,
# NO hardware, safe to run on any machine (and in CI). Same role as
# scripts/pi_setup/selftest.sh does for the Pi provisioning pack.
#
# What it proves:
#   1. every shell script parses (`bash -n`) and every python tool compiles;
#   2. `fc_link_check.py --self-test` passes its AST safety audit, AND the audit
#      really bites -- an injected `drone.action.arm()` is detected (adversarial
#      check, run against a temp copy, never against the real file);
#   3. the no-hardware paths of 00 / 01 / 02 / 05 run end-to-end and return the
#      documented "NOT CONNECTED" exit code 3 (or 0 if you happen to have the
#      gear attached -- both are accepted here);
#   4. `--help` works on every script;
#   5. THE FAILING BRANCHES ACTUALLY BITE (added 2026-07-25, review 2): a copy
#      that fails, a session shorter than the frames requested, and "--require
#      with no real camera" must all exit NONZERO. Each of those three was a
#      silent PASS before; a fix nothing exercises is a fix that rots;
#   6. every exit code a 0*.sh header ADVERTISES is actually reachable in that
#      script (the guard against a dead exit-contract line).
#
# Run:  bash scripts/field/selftest.sh
# ===========================================================================
set -uo pipefail

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
PY="${FIELD_PYTHON:-$REPO/.venv/bin/python}"
TMP="$(mktemp -d)"
trap 'chmod -R u+rwX "$TMP" 2>/dev/null; rm -rf "$TMP"' EXIT

fails=0
ok()  { printf '[field-selftest] OK   %s\n' "$*"; }
bad() { printf '[field-selftest] FAIL %s\n' "$*"; fails=$((fails + 1)); }

# --- 1. syntax ------------------------------------------------------------
for f in "$HERE"/*.sh; do
    if bash -n "$f"; then ok "bash -n $(basename "$f")"; else bad "bash -n $(basename "$f")"; fi
done
if [[ -x "$PY" ]]; then
    for f in "$HERE"/*.py; do
        if "$PY" -m py_compile "$f"; then ok "compile $(basename "$f")"
        else bad "compile $(basename "$f")"; fi
    done
else
    bad "python venv not found at $PY (run scripts/env/bootstrap.sh)"
fi

# --- 2. the no-arm audit, and proof that it bites -------------------------
if [[ -x "$PY" ]]; then
    if "$PY" "$HERE/fc_link_check.py" --self-test >/dev/null 2>&1; then
        ok "fc_link_check.py passes its own safety audit"
    else
        bad "fc_link_check.py FAILED its safety audit"
    fi
    # THREE injection forms, not one. The audit used to match CALL nodes only,
    # so forms (b) and (c) walked past a check whose printed guarantee covered
    # them. Each goes into its own TEMP COPY -- never the real file.
    inject_bites() {   # inject_bites <label> <python source to append>
        local label="$1" src="$2" f="$TMP/injected_$RANDOM.py"
        cp "$HERE/fc_link_check.py" "$f"
        printf '%s\n' "$src" >> "$f"
        "$PY" "$f" --self-test >/dev/null 2>&1
        local rc=$?
        if [[ "$rc" -eq 2 ]]; then ok "safety audit DETECTS $label (exit 2)"
        else bad "safety audit MISSED $label (exit $rc, expected 2)"; fi
    }
    inject_bites "a direct drone.action.arm() call" \
'async def _injected(drone):
    await drone.action.arm()'
    inject_bites "an ALIASED plugin handle (a = drone.action; a.arm())" \
'async def _injected(drone):
    a = drone.action
    await a.arm()'
    inject_bites "an offboard IMPORT (from mavsdk.offboard import ...)" \
'from mavsdk.offboard import VelocityNedYaw'
fi

# --- 3. the no-hardware paths run and report the documented codes ---------
# 0 = PASS (an explicit dry run counts) and 3 = NOT CONNECTED are BOTH healthy
# outcomes here; anything else means the script itself broke.
run_step() {
    local name="$1"; shift
    "$@" >/dev/null 2>&1
    local rc=$?
    case "$rc" in
        0) ok "$name ran clean (exit 0 PASS)" ;;
        3) ok "$name ran clean (exit 3 NOT CONNECTED -- expected with no hardware)" ;;
        *) bad "$name exited $rc (expected 0 or 3)" ;;
    esac
}
run_step "00_detect_devices"      "$HERE/00_detect_devices.sh"
run_step "01_camera_live_check"   "$HERE/01_camera_live_check.sh" --replay --frames 6
run_step "02_apriltag_desk_check" "$HERE/02_apriltag_desk_check.sh" --n-frames 6
run_step "03_fc_bench_check"      "$HERE/03_fc_bench_check.sh"
FIELD_SSH_TIMEOUT="${FIELD_SSH_TIMEOUT:-3}" \
run_step "05_pi_link_check"       "$HERE/05_pi_link_check.sh"

# --- 4. help text ---------------------------------------------------------
for f in "$HERE"/0*.sh "$HERE/bringup.sh"; do
    if "$f" --help >/dev/null 2>&1; then ok "--help $(basename "$f")"
    else bad "--help $(basename "$f")"; fi
done

# --- 5. the FAILING branches must actually bite ---------------------------
# Every case below returned 0/PASS before 2026-07-25 while the thing it claims
# to check had not happened. No hardware, no network, seconds to run.
want_fail() {   # want_fail <name> <cmd...>  -- passes when the command exits != 0
    local name="$1"; shift
    "$@" >/dev/null 2>&1
    local rc=$?
    if [[ "$rc" -ne 0 ]]; then ok "$name exits $rc (nonzero, as it must)"
    else bad "$name exited 0 -- the failing branch is dead"; fi
}

# 5a. 04: a source that was REQUESTED but could not be copied is a FAIL, not a
#     green PASS with zero bytes on disk.
CARD="$TMP/card_unreadable"
mkdir -p "$CARD"
printf 'x' > "$CARD/log_001.ulg"
chmod 000 "$CARD/log_001.ulg"
FIELD_SSH_TIMEOUT=2 want_fail "04 copy-failure" \
    "$HERE/04_pull_logs.sh" --px4 "$CARD" --no-score --session selftest_copyfail
chmod 644 "$CARD/log_001.ulg" 2>/dev/null

# 5b. 04: an EMPTY card (nothing inserted / nothing on it) must stay ADVISORY --
#     exit 3, never FAIL. This is the other half of the contract.
mkdir -p "$TMP/card_empty"
FIELD_SSH_TIMEOUT=2 "$HERE/04_pull_logs.sh" --px4 "$TMP/card_empty" --no-score \
    --session selftest_empty >/dev/null 2>&1
rc=$?
if [[ "$rc" -eq 3 ]]; then ok "04 empty card is ADVISORY (exit 3, not FAIL)"
else bad "04 empty card exited $rc (expected 3 NOT CONNECTED)"; fi

# 5c. 01: a session that delivers FEWER frames than requested must FAIL
#     (it was graded against --min-frames 1 and passed).
ONE="$TMP/one_frame"
mkdir -p "$ONE"
# scripts/seeker/data/ is GITIGNORED (the 15,391-image real-media corpus), so on a
# clean clone this check found nothing and the pack exited 1 -- the second cause
# that kept CI red on 73/73 runs (audit 2026-07-25, completeness critic #1).
# tests/fixtures/field_selftest_frame.png is a TRACKED 28 KB frame, so the branch
# now runs everywhere; the dataset dir stays first so a local run still uses it.
FIXTURE="$(find "$REPO/scripts/seeker/data" "$REPO/tests/fixtures" \
    -name '*.png' -print -quit 2>/dev/null)"
if [[ -n "$FIXTURE" ]]; then
    cp "$FIXTURE" "$ONE/"
    want_fail "01 short session (1 frame of 30)" \
        "$HERE/01_camera_live_check.sh" --replay "$ONE" --frames 30
else
    bad "no fixture PNG under scripts/seeker/data or tests/fixtures -- cannot test the short-session branch"
fi

# 5d. 01: --require must not be satisfiable by a replay fixture (no real camera).
want_fail "01 --replay --require (no seeker camera)" \
    "$HERE/01_camera_live_check.sh" --replay --frames 6 --require

# --- 6. every advertised exit code is reachable ---------------------------
# Each 0*.sh header documents its exit codes; a code that appears in the header
# but never in an fld_finish argument is a promise the script cannot keep (how
# 04's documented "1 FAIL" sat dead while scoring crashes exited 0).
for f in "$HERE"/0*.sh; do
    base="$(basename "$f")"
    missing=""
    grep -q "fld_finish" "$f" || continue
    for code in FLD_OK:0 FLD_FAIL:1 FLD_USAGE:2 FLD_ABSENT:3; do
        name="${code%%:*}"; want="${code##*:}"
        # Advertised in the header's "Exit:" block?
        if sed -n '/^# Exit:/,/^# ===/p' "$f" | grep -qE "(^|[^0-9])$want[ )]"; then
            # Reachable? Either passed straight to fld_finish, or assigned to the
            # status variable some scripts accumulate into (00_detect_devices).
            grep -qE "\\\$$name" "$f" || missing="$missing $want"
        fi
    done
    if [[ -z "$missing" ]]; then ok "exit contract reachable: $base"
    else bad "$base advertises exit code(s)$missing that it can never return"; fi
done

printf '\n[field-selftest] %d failure(s)\n' "$fails"
[[ "$fails" -eq 0 ]] || exit 1
printf '[field-selftest] PASS\n'
exit 0
