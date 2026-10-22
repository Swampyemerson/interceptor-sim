#!/usr/bin/env bash
# Gazebo confirming rung for docs/pursuit_vs_dash_prereg_2026-10-22.md (amendment).
# 4 sequential blocks of 4 fresh-boot flights: (5deg,10deg) x (+,-) on the
# ADR-0118/0120 recommended config. Kill/boot lives in xcheck_fly.sh/sim_kill.sh
# (batch-hygiene rule: no inline pkill patterns anywhere in this file).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
G="$REPO/runs/pursuit_vs_dash_20261022/gazebo"
run_block() {  # name  err_deg
    local name="$1" err="$2"
    echo "=== [rung] block $name (cue err $err deg) start $(date -u +%H:%M:%SZ) ==="
    bash "$REPO/scripts/xcheck_fly.sh" 1 4 "$G/$name" --pursuit-brake --cue-aim-error-deg "$err"
    echo "=== [rung] block $name done; cooldown 120 s ==="
    sleep 120
}
run_block g5p   5
run_block g5n  -5
run_block g10p 10
run_block g10n -10
echo "GAZEBO_RUNG_ALL_DONE"
