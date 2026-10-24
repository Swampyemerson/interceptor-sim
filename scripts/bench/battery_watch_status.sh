#!/usr/bin/env bash
# One-line bench battery status, read from the Pi's battery_watch daemon.
#   scripts/bench/battery_watch_status.sh [pi-host]     (default admin@192.168.0.42)
# Exit: 0 printed a status | 1 no status.json on the Pi (daemon not running?)
set -euo pipefail
PI="${1:-admin@192.168.0.42}"
JSON="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI" \
    'cat ~/interceptor-sim/logs/battery_watch/status.json' 2>/dev/null || true)"
if [[ -z "$JSON" ]]; then
    echo "[battery] NO STATUS -- is battery_watch running on the Pi?"
    exit 1
fi
printf '%s' "$JSON" | python3 -c "
import json, sys
s = json.load(sys.stdin)
v = s.get('verdict', '?')
if v == 'UNKNOWN':
    print('[battery] UNKNOWN: %s (misses=%s)'
          % (s.get('reason', ''), s.get('n_misses')))
else:
    proj = s.get('projected_s_to_warn')
    proj_s = ('n/a (not discharging or <2 samples)' if proj is None
              else '%.0f min' % (proj / 60))
    print('[battery] %s  %s V (%s V/cell, %s%%)  slope %s V/min  '
          'time-to-WARN %s  [sample %ss old, n=%s, miss=%s]'
          % (v, s.get('pack_v'), s.get('cell_v'), s.get('remaining_pct'),
             s.get('slope_v_per_min'), proj_s, s.get('age_s'),
             s.get('n_samples'), s.get('n_misses')))
"
