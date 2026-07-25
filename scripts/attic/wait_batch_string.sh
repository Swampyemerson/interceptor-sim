#!/bin/bash
# Bounded waiter for an mc_batch arm. Polls the arm's stdout LOG for the
# "Batch complete" sentinel (or detects a stall / overall timeout). It greps a
# LOG STRING only and contains NO sim-process pattern (no px4 / gz sim / sitl /
# gzserver / ruby / bin/px4) in its argv, so mc_batch's between-flight
# `pkill -f` cannot self-kill it. Never calls pkill/sim_kill.
#   usage: wait_batch_string.sh <stdout_log> [max_wait_s] [stall_s]
LOG="$1"
MAXWAIT="${2:-3600}"
STALL="${3:-480}"
start=$(date +%s)
while true; do
  if grep -qa "Batch complete" "$LOG" 2>/dev/null; then echo "BATCH_COMPLETE"; break; fi
  now=$(date +%s)
  if [ -f "$LOG" ]; then age=$(( now - $(stat -c %Y "$LOG") )); else age=999; fi
  if [ "$age" -gt "$STALL" ]; then echo "STALLED_${age}s_no_log_update"; break; fi
  if [ $(( now - start )) -gt "$MAXWAIT" ]; then echo "MAXWAIT_EXCEEDED"; break; fi
  sleep 20
done
echo "--- tail ---"
tail -4 "$LOG" 2>/dev/null
