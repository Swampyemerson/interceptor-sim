#!/usr/bin/env bash
# =============================================================================
# run_tests.sh -- the project's ONE automated test runner (audit finding DEEP-T1).
#
# Before this existed, 100+ offline tests passed but nothing ran them together,
# and the real-detector parity tests silently SKIPPED on every run: they
# `pytest.importorskip("onnxruntime")`, and the MAIN venv (.venv) has no
# onnxruntime -- so under .venv they never actually exercise the ONNX decode.
# This runner routes them to .venv-seeker (which HAS onnxruntime + cv2), so the
# parity tests run for real, and aggregates a single pass/fail exit code.
#
# GREEN MEANS RAN (review-2 hardening, 2026-07-24): stage 2 preflights its one
# real skip trigger (a missing onnxruntime/cv2 MODULE) and fails if pytest
# reports ANY skip, and stage 4 invokes the previously-uncovered dependency-
# light --self-test entry points (before it, only 1 of 29 self-tests was
# reachable from this runner or CI). A stage that silently skips its substance
# is a FAILURE here, never a pass -- docs/error_handling_policy.md.
#
# Layout (verified 2026-07-10):
#   .venv         : cv2 yes, onnxruntime NO  -> main suite; onnx tests skip here
#   .venv-seeker  : cv2 yes, onnxruntime yes -> the real-detector parity tests run
#
# CI-friendly: exits 0 iff every stage passed. IDLE-LOAD ONLY if a sim is flying
# (pytest is a CPU spike; the idle-load rule, ADR-0015 2nd addendum) -- this
# runner is meant to gate a sim run, not overlap one.
#
# Usage:  scripts/run_tests.sh            # all stages, quiet
#         scripts/run_tests.sh -v         # pass-through pytest verbosity
# =============================================================================
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROOT="$(pwd)"
PYTEST_ARGS=("${@:--q}")
rc=0

echo "== [1/4] main offline suite  (.venv)  =="
# flight/tests/ = the portable real-build core (geometry + coded-dash aim); pure
# math, no onnx/cv2, so it runs in the main suite (ADR-0076 add #18, P0.3).
#
# SKIP ENFORCEMENT ADDED 2026-08-10, and it is the point of this stage, not
# decoration. Stage 1 used to run without -rs and without checking skips, so a
# test that had become STRUCTURALLY INCAPABLE OF RUNNING reported as green. Two
# were found that night: test_camera_fps_pinned's CLI-drift guard called a
# build_argparser() that did not exist, and test_decode_fps_bound_contract's
# regression pointed at a sessions/ path that has never existed in any commit.
# Both had skipped silently since the day they were written -- guarding numbers
# that feed the ~$740 gate. Only stage 2 had this check; that asymmetry is
# exactly what hid them.
#
# The rule: EVERY skip must be named in ALLOWED_SKIPS below, with a reason. A
# new skip fails the stage until a human decides it is legitimate.
ALLOWED_SKIPS=(
    # onnxruntime is deliberately absent from .venv; stage 2 re-runs these same
    # tests under .venv-seeker where they EXECUTE, and fails there on any skip.
    "could not import 'onnxruntime'"
    # The rescore acceptance gate follows flight_csv_path to the gitignored
    # 168-flight per-tick archive, which exists only on the dev machine. On the
    # dev machine the data is present and the gate RUNS (no skip fires); on CI
    # and fresh clones it skips with this reason instead of failing the suite
    # (it kept CI red 2026-08-11..13). Accepted 2026-08-19.
    "per-tick flight CSV archive not on this machine"
)
if [ -x .venv/bin/python ]; then
    stage1_out="$(mktemp)"
    .venv/bin/python -m pytest tests/ flight/tests/ -rs "${PYTEST_ARGS[@]}" \
        2>&1 | tee "$stage1_out" || rc=1
    # Every "SKIPPED [n] path::test: reason" line must match an allowlist entry.
    while IFS= read -r line; do
        ok=0
        for alw in "${ALLOWED_SKIPS[@]}"; do
            case "$line" in *"$alw"*) ok=1; break;; esac
        done
        if [ "$ok" -eq 0 ]; then
            echo "  FAIL: UNDECLARED SKIP -- $line"
            echo "        A skipped test protects nothing. Either make it run, or"
            echo "        add its reason to ALLOWED_SKIPS in scripts/run_tests.sh"
            echo "        with a comment saying why a human accepted it."
            rc=1
        fi
    done < <(grep '^SKIPPED' "$stage1_out" || true)
    rm -f "$stage1_out"
else
    echo "  SKIP: .venv not found"; rc=1
fi

echo ""
echo "== [2/4] real-detector ONNX parity  (.venv-seeker: has onnxruntime)  =="
if [ -x .venv-seeker/bin/python ]; then
    # test_ground_station.py holds the importorskip('onnxruntime') parity cases;
    # under .venv-seeker they EXECUTE instead of skipping. The ONLY skip path is
    # a missing onnxruntime/cv2 MODULE (absent weights raise NoSuchFile = test
    # FAILS; an absent capture dir raises FileNotFoundError = test ERRORs --
    # both already loud). So: (a) preflight the modules and fail loudly if the
    # venv lost them; (b) run with -rs and fail if ANYTHING skipped anyway.
    # Green in this stage must mean the parity tests actually RAN.
    if ! .venv-seeker/bin/python -c 'import onnxruntime, cv2' 2>/dev/null; then
        echo "  FAIL: .venv-seeker lost onnxruntime/cv2 -- the ONNX parity tests"
        echo "        would silently SKIP, not run. Reinstall into .venv-seeker."
        rc=1
    else
        stage2_out="$(mktemp)"
        .venv-seeker/bin/python -m pytest tests/test_ground_station.py -rs \
            "${PYTEST_ARGS[@]}" 2>&1 | tee "$stage2_out" || rc=1
        if grep -qE "[0-9]+ skipped" "$stage2_out"; then
            echo "  FAIL: ONNX parity stage reported SKIPPED tests (see -rs lines"
            echo "        above) -- a skip here is a missed parity check, not a pass."
            rc=1
        fi
        rm -f "$stage2_out"
    fi
else
    echo "  SKIP: .venv-seeker not found (ONNX parity NOT exercised)"; rc=1
fi

echo ""
echo "== [3/4] project-state dashboard sync (docs/project_state.json <-> docs/dashboard.html) =="
# The state file is the CONTRACT; the dashboard embeds a copy. Drift fails the
# suite -- including the hand-authored layer's number-traceability guard
# (every VISIBLE decimal number must appear verbatim in the contract).
python3 scripts/render_dashboard.py --check || rc=1
# Second generated view of the SAME contract: the MBSE view set. Same rule --
# the model may not drift from the build. The check compares a digest of the
# contract against the one baked into docs/mbse.html at render time.
python3 scripts/render_mbse.py --check || rc=1

echo ""
echo "== [4/4] uncovered --self-test entry points (each must exit 0) =="
# These five were unreachable from any automated runner (review 2: only 1 of
# 29 self-tests ran in CI). Each is offline, dependency-light, ~1 s.
# DELIBERATE SKIP SET (do not cite an unrun self-test as evidence): the other
# ~25 self-tests either duplicate stronger stage-1 pytest coverage (pi_capture,
# tripod_score, range_truth_join, real_flight) or need weights/GPU/hardware
# (nn_tier/*, seeker capture tools, darkhorse/*).
run_selftest() {
    local label="$1"; shift
    local out
    out="$(mktemp)"
    if "$@" >"$out" 2>&1; then
        echo "  OK   $label"
    else
        echo "  FAIL $label (exit $?) -- output:"
        sed 's/^/       /' "$out" | tail -20
        rc=1
    fi
    rm -f "$out"
}
run_selftest "scripts/field_score.py --self-test (kill/CPA scorer)" \
    "$ROOT/.venv/bin/python" "$ROOT/scripts/field_score.py" --self-test
run_selftest "flight/deploy/seeker_loop.py --self-test (deployed terminal loop)" \
    "$ROOT/.venv/bin/python" "$ROOT/flight/deploy/seeker_loop.py" --self-test
run_selftest "scripts/field/parse_flight_log.py --self-test (onboard decision-log parser)" \
    "$ROOT/.venv/bin/python" "$ROOT/scripts/field/parse_flight_log.py" --self-test
# Gated because this one decides a HARDWARE verdict (brn-03/brn-04): its FAIL /
# UNCERTAIN / PASS split is what stops "bytes arrived" being read as "the link
# works". 26 checks, stdlib-only, ~40 ms, no hardware.
run_selftest "scripts/seeker/fc_link_check.py --self-test (FC->Pi MAVLink link verdict)" \
    "$ROOT/.venv/bin/python" "$ROOT/scripts/seeker/fc_link_check.py" --self-test

# NO LOG POLLUTION (audit hygiene fix, 2026-07-25): the field pack's scripts
# each mkdir a timestamped run dir under logs/field/, so every invocation of
# this runner used to leave ~8 REAL-LOOKING capture dirs behind (53 had piled
# up by 2026-07-25). Test droppings that look exactly like field evidence are
# an evidence-integrity hazard, not just clutter. FIELD_LOG_ROOT (honoured by
# scripts/field/common.sh) redirects them into a scratch dir we delete after.
FIELD_TMP_LOGS="$(mktemp -d)"
run_selftest "scripts/field/selftest.sh (field bring-up pack incl. AST arming audit)" \
    env FIELD_LOG_ROOT="$FIELD_TMP_LOGS" bash "$ROOT/scripts/field/selftest.sh"
rm -rf "$FIELD_TMP_LOGS"

# ALL THREE selftest.sh PACKS (audit fix, 2026-07-25). `git ls-files | grep
# selftest.sh` returns three; until now only scripts/field was gated here or in
# CI, which is precisely how the stale tripod mission geometry survived inside
# the UNGATED configs/target_kakute pack. Both are stdlib-only and offline.
run_selftest "configs/target_kakute/selftest.sh (mission generator + target.param lint)" \
    bash "$ROOT/configs/target_kakute/selftest.sh"
run_selftest "scripts/pi_setup/selftest.sh (Pi provisioning pack, dry-run no-change proof)" \
    bash "$ROOT/scripts/pi_setup/selftest.sh"

# COVERAGE-DRIFT GUARD (audit, reader 5: "the skip set is prose, not data").
# The DELIBERATE SKIP SET above is a comment, and a comment cannot notice a pack
# added tomorrow -- which is the same shape as the hole stage 4 was built to
# close. The packs are uniformly cheap and offline, so make their coverage DATA:
# every tracked *selftest.sh must appear in this file. (The ~37 python
# --self-test entry points stay prose-governed; they are NOT uniformly cheap --
# several need weights/GPU/hardware.)
missing_packs=""
while read -r pack; do
    [ -n "$pack" ] || continue
    grep -qF "$pack" "$ROOT/scripts/run_tests.sh" || missing_packs="$missing_packs $pack"
done < <(cd "$ROOT" && git ls-files -- '*selftest.sh' 2>/dev/null)
if [ -n "$missing_packs" ]; then
    echo "  FAIL tracked selftest.sh pack(s) NOT wired into stage 4:$missing_packs"
    echo "       (add a run_selftest line above, or the pack is uncovered)"
    rc=1
else
    echo "  OK   every tracked selftest.sh pack is wired into stage 4"
fi

echo ""
if [ "$rc" -eq 0 ]; then
    echo "run_tests: ALL GREEN"
else
    echo "run_tests: FAILURES (rc=$rc)"
fi
exit "$rc"
