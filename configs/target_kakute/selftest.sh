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
echo "== lint_param.py must REJECT the 2026-11-14 crash settings (full motor range) =="
MUT="$(mktemp)"
sed 's/^MOT_SPIN_MAX,.*/MOT_SPIN_MAX,0.95/' "$DIR/target.param" > "$MUT"
if "$PY" "$DIR/lint_param.py" "$MUT" > /dev/null; then
    rm -f "$MUT"
    echo "FAIL: the lint passed a file whose thrust ceiling is above hover"
    exit 1
fi
rm -f "$MUT"
echo "rejected, as it must be"

echo
echo "== ap_thrust_ceiling_check.py --self-test =="
"$PY" "$DIR/../../scripts/bench/ap_thrust_ceiling_check.py" --self-test

echo
echo "ALL TARGET-CONFIG-PACK CHECKS PASSED"
