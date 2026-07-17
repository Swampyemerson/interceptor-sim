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
# Layout (verified 2026-07-10):
#   .venv         : cv2 yes, onnxruntime NO  -> main suite; onnx tests skip here
#   .venv-seeker  : cv2 yes, onnxruntime yes -> the real-detector parity tests run
#
# CI-friendly: exits 0 iff every stage passed. IDLE-LOAD ONLY if a sim is flying
# (pytest is a CPU spike; the idle-load rule, ADR-0015 2nd addendum) -- this
# runner is meant to gate a sim run, not overlap one.
#
# Usage:  scripts/run_tests.sh            # both stages, quiet
#         scripts/run_tests.sh -v         # pass-through pytest verbosity
# =============================================================================
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
PYTEST_ARGS=("${@:--q}")
rc=0

echo "== [1/2] main offline suite  (.venv)  =="
# flight/tests/ = the portable real-build core (geometry + coded-dash aim); pure
# math, no onnx/cv2, so it runs in the main suite (ADR-0076 add #18, P0.3).
if [ -x .venv/bin/python ]; then
    .venv/bin/python -m pytest tests/ flight/tests/ "${PYTEST_ARGS[@]}" || rc=1
else
    echo "  SKIP: .venv not found"; rc=1
fi

echo ""
echo "== [2/2] real-detector ONNX parity  (.venv-seeker: has onnxruntime)  =="
if [ -x .venv-seeker/bin/python ]; then
    # test_ground_station.py holds the importorskip('onnxruntime') parity cases;
    # under .venv-seeker they EXECUTE instead of skipping. (If the cached frames/
    # weights are absent, importorskip/skips remain -- still not a failure.)
    .venv-seeker/bin/python -m pytest tests/test_ground_station.py "${PYTEST_ARGS[@]}" || rc=1
else
    echo "  SKIP: .venv-seeker not found (ONNX parity NOT exercised)"; rc=1
fi

echo ""
echo "== project-state dashboard sync (docs/project_state.json <-> docs/dashboard.html) =="
# The state file is the CONTRACT; the dashboard embeds a copy. Drift fails the suite.
python3 scripts/render_dashboard.py --check || rc=1

echo ""
if [ "$rc" -eq 0 ]; then
    echo "run_tests: ALL GREEN"
else
    echo "run_tests: FAILURES (rc=$rc)"
fi
exit "$rc"
