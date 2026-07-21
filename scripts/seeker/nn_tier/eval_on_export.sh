#!/bin/bash
# Eval-on-export watcher for the nn_tier fine-tunes. Waits (patiently, by file
# stability) for each daemon to export its ONNX, then runs that model's
# source-disjoint held-out eval in .venv-seeker (each also scores v2_deployed as
# the bar). Appends a one-line summary per model + a per-model DONE marker; an
# ALL_DONE marker at the end. Reaper-safe: launch setsid-detached; it survives
# the session and the ~51-min background reap. Poll RESULTS/markers from the head.
set -u
cd /home/emerson/interceptor-sim || exit 1
PY=.venv-seeker/bin/python
OUT=logs/nn_tier
RESULTS=$OUT/heldout_scores.txt
mkdir -p "$OUT"
echo "[$(date -u +%H:%M:%S)] eval_on_export watcher up (pid $$)" >>"$RESULTS"

# name | onnx | eval-script | eval-args
JOBS=(
  "n-mono|scripts/seeker/weights/nn_tier/n-mono.onnx|scripts/seeker/nn_tier/eval_n_mono.py|--models n_mono v2_deployed --tag heldout"
  "s-mono|scripts/seeker/weights/nn_tier/s-mono.onnx|scripts/seeker/nn_tier/eval_heldout_smono.py|--models s_mono v2_deployed --tag heldout"
  "n-mono-aug|scripts/seeker/weights/nn_tier/n-mono-aug.onnx|scripts/seeker/nn_tier/eval_n_mono_aug.py|--models n-mono-aug v2_deployed --tag heldout"
)

wait_stable() {  # $1 = path; returns when it exists and size is unchanged for 30s
  local p=$1 s1 s2
  while [ ! -f "$p" ]; do sleep 30; done
  while :; do
    s1=$(stat -c %s "$p" 2>/dev/null); sleep 30; s2=$(stat -c %s "$p" 2>/dev/null)
    [ "$s1" = "$s2" ] && [ -n "$s1" ] && break
  done
}

for job in "${JOBS[@]}"; do
  IFS='|' read -r name onnx script args <<<"$job"
  echo "[$(date -u +%H:%M:%S)] waiting for $name export ($onnx)" >>"$RESULTS"
  wait_stable "$onnx"
  echo "[$(date -u +%H:%M:%S)] $name exported; scoring held-out set..." >>"$RESULTS"
  log=$OUT/eval_${name}_heldout_run.log
  $PY "$script" $args >"$log" 2>&1
  rc=$?
  # pull the headline lines the eval prints (mAP/recall/false-fire) into RESULTS
  echo "----- $name held-out (rc=$rc) $(date -u +%H:%M:%S) -----" >>"$RESULTS"
  grep -iE "mAP50|recall|precision|false.?fire|AP50|n_mono|s_mono|n-mono|v2_deployed|SUMMARY|WIN|BEAT" "$log" | tail -40 >>"$RESULTS"
  touch "$OUT/EVAL_DONE_${name}"
done
touch "$OUT/EVAL_ALL_DONE"
echo "[$(date -u +%H:%M:%S)] all held-out evals complete" >>"$RESULTS"
