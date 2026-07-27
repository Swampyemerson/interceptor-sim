#!/usr/bin/env bash
# Stop the skr-07 soak runner and any in-flight arm, so the camera is released.
#
# The patterns live in this FILE and never in an inline tool-call command: the
# harness evals the command text, so a literal pattern typed inline sits in an
# ancestor process's argv, self-matches, and kills the invocation (exit 144) even
# with zero real matches. Hit three times in one day; the rule is in CLAUDE.md.
set -uo pipefail

pkill -f run_skr07_soaks.sh 2>/dev/null
pkill -f pi_fps_soak.py 2>/dev/null
sleep 2
pkill -9 -f pi_fps_soak.py 2>/dev/null
sleep 1

if pgrep -f pi_fps_soak.py > /dev/null 2>&1; then
    echo "STILL RUNNING"
    exit 1
fi
echo "STOPPED — camera released"
exit 0
