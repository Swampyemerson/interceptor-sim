#!/bin/bash
# Eval-on-export watcher for the nn_tier fine-tunes (ORDER-INDEPENDENT). Each
# cycle it scans all models and scores ANY whose ONNX has appeared and is stable
# and hasn't been scored yet — so whichever daemon exports first gets scored
# first, regardless of the training queue order. Each eval also scores
# v2_deployed as the bar. Appends a one-line summary + per-model EVAL_DONE_<name>
# marker; EVAL_ALL_DONE when all three are scored. Reaper-safe: launch
# setsid-detached; it survives the session and the ~51-min background reap.
set -u
cd /home/emerson/interceptor-sim || exit 1
PY=.venv-seeker/bin/python
OUT=logs/nn_tier
RESULTS=$OUT/heldout_scores.txt
mkdir -p "$OUT"
echo "[$(date -u +%H:%M:%S)] eval_on_export (order-independent) up (pid $$)" >>"$RESULTS"

# name | onnx | eval-script | eval-args
JOBS=(
  "n-mono|scripts/seeker/weights/nn_tier/n-mono.onnx|scripts/seeker/nn_tier/eval_n_mono.py|--models n_mono v2_deployed --tag heldout"
  "s-mono|scripts/seeker/weights/nn_tier/s-mono.onnx|scripts/seeker/nn_tier/eval_heldout_smono.py|--models s_mono v2_deployed --tag heldout"
  "n-mono-aug|scripts/seeker/weights/nn_tier/n-mono-aug.onnx|scripts/seeker/nn_tier/eval_n_mono_aug.py|--models n-mono-aug v2_deployed --tag heldout"
)

stable() {  # $1 = path; true if exists and size unchanged across a 30s window
  local p=$1 s1 s2
  [ -f "$p" ] || return 1
  s1=$(stat -c %s "$p" 2>/dev/null); sleep 30; s2=$(stat -c %s "$p" 2>/dev/null)
  [ -n "$s1" ] && [ "$s1" = "$s2" ]
}

score() {  # name onnx script args
  local name=$1 onnx=$2 script=$3 args=$4 log=$OUT/eval_${name}_heldout_run.log rc
  echo "[$(date -u +%H:%M:%S)] $name exported; scoring held-out set..." >>"$RESULTS"
  $PY "$script" $args >"$log" 2>&1; rc=$?
  echo "----- $name held-out (rc=$rc) $(date -u +%H:%M:%S) -----" >>"$RESULTS"
  grep -E '"model"|mAP50|recall|precision|false.?fire|AP50|SUMMARY|WIN|BEAT' "$log" | tail -40 >>"$RESULTS"
  touch "$OUT/EVAL_DONE_${name}"
}

while :; do
  alldone=1
  for job in "${JOBS[@]}"; do
    IFS='|' read -r name onnx script args <<<"$job"
    [ -f "$OUT/EVAL_DONE_${name}" ] && continue     # already scored
    alldone=0
    if stable "$onnx"; then score "$name" "$onnx" "$script" "$args"; fi
  done
  [ "$alldone" = 1 ] && break
  sleep 30
done
touch "$OUT/EVAL_ALL_DONE"
echo "[$(date -u +%H:%M:%S)] all held-out evals complete" >>"$RESULTS"
