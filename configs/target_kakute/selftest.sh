#!/usr/bin/env bash
# selftest.sh -- the target-config-pack gate. Exits 0 only if both mission
# generators' self-tests AND the target.param lint pass. No hardware, no sim.
#   configs/target_kakute/selftest.sh
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# repo .venv if present, else system python3 (scripts are stdlib-only)
PY="$DIR/../../.venv/bin/python"
[ -x "$PY" ] || PY="$(command -v python3)"

echo "== gen_tripod_mission.py --self-test =="
"$PY" "$DIR/gen_tripod_mission.py" --self-test

echo
echo "== gen_loop_mission.py --self-test =="
"$PY" "$DIR/gen_loop_mission.py" --self-test

echo
echo "== lint_param.py target.param =="
"$PY" "$DIR/lint_param.py" "$DIR/target.param"

echo
echo "== lint_param.py must REJECT the 2026-11-14 crash settings (floor 0.15, hover 0.2) =="
MUT="$(mktemp)"
sed -e 's/^MOT_SPIN_MIN,.*/MOT_SPIN_MIN,0.15/' -e 's/^MOT_THST_HOVER,.*/MOT_THST_HOVER,0.2/' "$DIR/target.param" > "$MUT"
grep -q '^MOT_SPIN_MIN,0.15$' "$MUT" && grep -q '^MOT_THST_HOVER,0.2$' "$MUT" || { echo "FAIL: mutant not built"; exit 1; }
if "$PY" "$DIR/lint_param.py" "$MUT" > "$MUT.out"; then
    rm -f "$MUT"
    echo "FAIL: the lint passed a file whose thrust ceiling is above hover"
    exit 1
fi
grep -q "thrust ceiling" "$MUT.out" || { cat "$MUT.out"; rm -f "$MUT" "$MUT.out"; echo "FAIL: rejected, but not for the thrust ceiling"; exit 1; }
rm -f "$MUT" "$MUT.out"
echo "rejected for the thrust ceiling, as it must be"

echo
echo "== ap_thrust_ceiling_check.py --self-test =="
"$PY" "$DIR/../../scripts/bench/ap_thrust_ceiling_check.py" --self-test

echo
echo "ALL TARGET-CONFIG-PACK CHECKS PASSED"
