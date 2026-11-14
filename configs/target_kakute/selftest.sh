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
echo "ALL TARGET-CONFIG-PACK CHECKS PASSED"
