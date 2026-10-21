#!/usr/bin/env bash
# Fly a serial batch of cinema takes (one fresh sim boot per take; one sim at
# a time -- batch hygiene). Each line of the TAKES array = "name|driver args".
# Usage: bash scripts/cinema/fly_takes.sh <outdir> [takes-file]
#   takes-file: optional file of name|args lines (else the built-in list).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTDIR="${1:?outdir}"
TAKESFILE="${2:-}"
mkdir -p "$OUTDIR"
READY="Startup script returned successfully"

TAKES=()
if [ -n "$TAKESFILE" ]; then
    while IFS= read -r line; do
        [ -n "$line" ] && [ "${line:0:1}" != "#" ] && TAKES+=("$line")
    done < "$TAKESFILE"
else
    TAKES=(
        "t3|--lead 22 --cross-range 4 --speed 12 --cue-offset-n 2.5 --cue-vel-rot-deg 5 --window-s 30"
        "t4|--lead 22 --cross-range 4 --speed 12 --cue-offset-n -3.0 --cue-vel-rot-deg -6 --window-s 30"
        "t5|--lead 25 --cross-range 6.5 --speed 12 --cue-offset-n 2.0 --cue-vel-rot-deg 4 --window-s 30"
        "t6|--lead 22 --cross-range 4 --speed 12 --cue-offset-n 2.5 --cue-vel-rot-deg 5 --window-s 30"
    )
fi

for spec in "${TAKES[@]}"; do
    name="${spec%%|*}"; targs="${spec#*|}"
    echo "=== [fly_takes] take $name: $targs ==="
    bash "$REPO/scripts/sim_kill.sh" >/dev/null 2>&1
    sleep 2
    SIMLOG="$OUTDIR/sim_$name.log"
    bash "$REPO/scripts/demo_boot_sim.sh" "$SIMLOG" cinema >/dev/null
    ok=0
    for _ in $(seq 1 60); do
        grep -q "$READY" "$SIMLOG" 2>/dev/null && { ok=1; break; }
        sleep 2
    done
    if [ "$ok" -ne 1 ]; then
        echo "[fly_takes] take $name: SIM BOOT FAILED (see $SIMLOG)"
        continue
    fi
    sleep 5
    # shellcheck disable=SC2086
    timeout 420 "$REPO/.venv/bin/python" "$REPO/scripts/cinema/cinema_run.py" \
        --out "$OUTDIR/$name.csv" $targs \
        2>&1 | tee "$OUTDIR/run_$name.log" | grep -E "CINEMA_RESULT|FAIL|Traceback" | tail -4
    bash "$REPO/scripts/sim_kill.sh" >/dev/null 2>&1
    sleep 3
done
echo "=== [fly_takes] done ==="
grep -h "CINEMA_RESULT" "$OUTDIR"/run_*.log 2>/dev/null
