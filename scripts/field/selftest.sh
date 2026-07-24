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
#   3. the no-hardware paths of 00 / 01 / 02 run end-to-end and return the
#      documented "NOT CONNECTED" exit code 3 (or 0 if you happen to have the
#      gear attached -- both are accepted here);
#   4. `--help` works on every script.
#
# Run:  bash scripts/field/selftest.sh
# ===========================================================================
set -uo pipefail

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
PY="${FIELD_PYTHON:-$REPO/.venv/bin/python}"

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
    bad "python venv not found at $PY (run bootstrap.sh)"
fi

# --- 2. the no-arm audit, and proof that it bites -------------------------
if [[ -x "$PY" ]]; then
    if "$PY" "$HERE/fc_link_check.py" --self-test >/dev/null 2>&1; then
        ok "fc_link_check.py passes its own safety audit"
    else
        bad "fc_link_check.py FAILED its safety audit"
    fi
    TMP="$(mktemp -d)"
    trap 'rm -rf "$TMP"' EXIT
    cp "$HERE/fc_link_check.py" "$TMP/injected.py"
    # Inject exactly what must never exist in that file.
    printf '\nasync def _injected(drone):\n    await drone.action.arm()\n' >> "$TMP/injected.py"
    "$PY" "$TMP/injected.py" --self-test >/dev/null 2>&1
    rc=$?
    if [[ "$rc" -eq 2 ]]; then
        ok "safety audit DETECTS an injected drone.action.arm() (exit 2)"
    else
        bad "safety audit MISSED an injected arm call (exit $rc, expected 2)"
    fi
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

# --- 4. help text ---------------------------------------------------------
for f in "$HERE"/0*.sh "$HERE/bringup.sh"; do
    if "$f" --help >/dev/null 2>&1; then ok "--help $(basename "$f")"
    else bad "--help $(basename "$f")"; fi
done

printf '\n[field-selftest] %d failure(s)\n' "$fails"
[[ "$fails" -eq 0 ]] || exit 1
printf '[field-selftest] PASS\n'
exit 0
