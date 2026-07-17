#!/usr/bin/env bash
# run-interceptor-sim SMOKE DRIVER — the one-command "does the whole thing build,
# launch, and fly on THIS machine" check. It chains the two already-committed,
# already-verified gates so a fresh agent gets a single PASS/FAIL + visual evidence:
#
#   [1/2] scripts/run_tests.sh  — portable core (guidance/geometry + coded-dash aim
#                                 + ONNX real-detector parity). NO sim. ~20 s.
#   [2/2] scripts/check_m1.sh   — boots PX4 SITL + Gazebo Harmonic HEADLESS with the
#                                 gz_x500_mono_cam airframe, captures 10 real 1280x960
#                                 camera frames over gz-transport (no ROS), tears the
#                                 sim down. ~2–3 min (boot-dominated). Exit 0 = PASS.
#
# On success it prints the captured-frame directory — open frame_000.png to SEE the
# running camera (a real rendered horizon, stddev>0 = not a blank buffer).
#
# Deeper "drive an actual flight" gates (each boots+flies+scores+tears-down, exit 0/1):
#   scripts/check_m0.sh  arm → takeoff to 2 m → land
#   scripts/check_m3.sh  static intercept, holds a 2 m standoff (<0.5 m error)
#   scripts/check_m4.sh  moving-target intercept (pursuit vs pro-nav)
# Env vars / launch footguns: the px4-gazebo skill. Debugging a wedged stack: sim-debug.
# Monte-Carlo batches: the mc-batch skill.  ONE sim at a time, at idle machine load.
#
# Usage:
#   bash .claude/skills/run-interceptor-sim/smoke.sh              # tests + sim  (full)
#   bash .claude/skills/run-interceptor-sim/smoke.sh --tests-only # fast, no sim boot
set -uo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"
TESTS_ONLY=0
[[ "${1:-}" == "--tests-only" ]] && TESTS_ONLY=1

echo "=========================================================="
echo " run-interceptor-sim smoke   $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo " repo: $REPO_ROOT"
[[ "$TESTS_ONLY" -eq 1 ]] && echo " mode: --tests-only (no sim boot)"
echo "=========================================================="

echo; echo ">>> [1/2] portable core tests (scripts/run_tests.sh, no sim) ..."
if scripts/run_tests.sh; then
    echo ">>> [1/2] PASS"
else
    echo ">>> [1/2] FAIL — portable tests are red; fix before trusting any sim run"
    exit 1
fi

if [[ "$TESTS_ONLY" -eq 1 ]]; then
    echo; echo ">>> smoke (--tests-only): PASS"; exit 0
fi

echo; echo ">>> [2/2] sim smoke (scripts/check_m1.sh: boot + capture 10 frames + teardown) ..."
if scripts/check_m1.sh; then
    FRAMES_DIR="$(ls -dt "$REPO_ROOT"/logs/m1_frames_* 2>/dev/null | head -1)"
    echo ">>> [2/2] PASS — camera frames in: ${FRAMES_DIR:-<none found>}"
else
    echo ">>> [2/2] FAIL — sim boot or camera capture failed; see logs/check_m1_sim_*.log"
    echo "    (debug with the sim-debug skill: .claude/skills/sim-debug)"
    exit 1
fi

echo; echo "=========================================================="
echo " smoke: PASS — tests green + sim boots + camera pipeline live"
echo " SEE it: open the newest  logs/m1_frames_*/frame_000.png"
echo "=========================================================="
