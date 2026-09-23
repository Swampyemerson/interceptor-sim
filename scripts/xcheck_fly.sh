#!/usr/bin/env bash
# Fly N Gazebo cross-check flights of the pursuit terminal, one fresh sim
# boot per flight (docs/xcheck_gazebo_pursuit_prereg.md). Kill/boot text
# lives in script FILES invoked by path (batch-hygiene rule).
# Usage: bash scripts/xcheck_fly.sh <first_idx> <last_idx> <outdir>
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIRST="${1:?first idx}"; LAST="${2:?last idx}"; OUTDIR="${3:?outdir}"
mkdir -p "$OUTDIR"
READY="Startup script returned successfully"

for i in $(seq "$FIRST" "$LAST"); do
    echo "=== [xcheck_fly] flight $i ==="
    bash "$REPO/scripts/sim_kill.sh" >/dev/null 2>&1
    sleep 2
    SIMLOG="$OUTDIR/sim_f$i.log"
    bash "$REPO/scripts/demo_boot_sim.sh" "$SIMLOG" >/dev/null
    ok=0
    for _ in $(seq 1 60); do
        grep -q "$READY" "$SIMLOG" 2>/dev/null && { ok=1; break; }
        sleep 2
    done
    if [ "$ok" -ne 1 ]; then
        echo "[xcheck_fly] flight $i: SIM BOOT FAILED (see $SIMLOG)"
        continue
    fi
    sleep 5
    timeout 420 "$REPO/.venv/bin/python" "$REPO/scripts/gazebo_pursuit_crosscheck.py" \
        --out "$OUTDIR/f$i.csv" --engage-max-s 25 \
        2>&1 | tee "$OUTDIR/run_f$i.log" | grep -E "XCHECK_RESULT|FAIL|Traceback|\[mavsdk\]|\[xcheck\]" | tail -8
    bash "$REPO/scripts/sim_kill.sh" >/dev/null 2>&1
    sleep 3
done
echo "=== [xcheck_fly] done; results: ==="
grep -h "XCHECK_RESULT" "$OUTDIR"/run_f*.log 2>/dev/null
