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
if [ -x .venv/bin/python ]; then
    .venv/bin/python -m pytest tests/ flight/tests/ "${PYTEST_ARGS[@]}" || rc=1
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

echo ""
echo "== [4/4] uncovered --self-test entry points (each must exit 0) =="
# These three were unreachable from any automated runner (review 2: only 1 of
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
run_selftest "scripts/field/selftest.sh (field bring-up pack incl. AST arming audit)" \
    bash "$ROOT/scripts/field/selftest.sh"

echo ""
if [ "$rc" -eq 0 ]; then
    echo "run_tests: ALL GREEN"
else
    echo "run_tests: FAILURES (rc=$rc)"
fi
exit "$rc"
