#!/usr/bin/env bash
# check_m5.sh -- scripted M5 milestone gate (ADR-0036).
#
# M5 is a DELIVERABLES milestone (Monte-Carlo + plots + README + GIF, GOALS.md),
# but the headline numbers must still be reproducible from the logs, not eyeballed.
# This gate recomputes the ADR-0036 headline from the merged final-batch CSV and
# asserts it, and confirms the committed plots + the analysis tool are present.
# Exits 0 (pass) / 1 (fail). Rerun after any re-analysis. See ADR-0036, ADR-0025.
set -uo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"
VENV_PYTHON="${REPO}/.venv/bin/python"
ALL_CSV="${REPO}/logs/mc_final_all.csv"

echo "[check_m5] repo=${REPO}"

# The merged CSV is the concatenation of the 6 arm CSVs (regenerate if missing).
if [[ ! -f "$ALL_CSV" ]]; then
    echo "[check_m5] merged CSV missing; regenerating from the 6 arm CSVs..."
    "$VENV_PYTHON" - <<'PY' || { echo "[check_m5] FAIL: could not regenerate mc_final_all.csv (arm CSVs missing?)"; exit 1; }
import csv, os
arms = ['line6','line9','line12','weave9','jink9','oblique6']
files = [f'logs/mc_final_{a}.csv' for a in arms]
rows=[]; header=None
for f in files:
    with open(f) as fh:
        rd=csv.reader(fh); h=next(rd)
        header = header or h
        assert h==header, f"header mismatch in {f}"
        rows += list(rd)
with open('logs/mc_final_all.csv','w',newline='') as out:
    w=csv.writer(out); w.writerow(header); w.writerows(rows)
print(f"regenerated mc_final_all.csv: {len(rows)} flights")
PY
fi

"$VENV_PYTHON" - "$ALL_CSV" <<'PY'
import csv, math, sys, statistics
path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
def miss(r):
    v = r.get("miss_m","")
    try: return float(v)
    except (TypeError,ValueError): return float("nan")
misses = [miss(r) for r in rows]
n = len(rows)
valid = [m for m in misses if not math.isnan(m)]
clean = sum(1 for r in rows if r.get("clean")=="1")

def pk(radius):
    # ADR-0025: fraction of ALL flights with miss <= radius; nan counts as a miss.
    return sum(1 for m in misses if (not math.isnan(m)) and m <= radius) / n

fails = []
def check(name, got, lo, hi):
    ok = lo <= got <= hi
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {got:.3f} (expect [{lo},{hi}])")
    if not ok: fails.append(name)

print(f"[check_m5] n={n} flights, {len(valid)} valid, clean {clean}/{n}")
# ADR-0036 headline, with tolerance bands (recompute must land here).
check("n_flights", n, 96, 96)
check("clean_frac", clean/n, 0.95, 1.00)
check("mean_miss_m", statistics.mean(valid), 1.00, 1.17)
check("median_miss_m", statistics.median(valid), 0.85, 1.00)
check("Pk@1.0m", pk(1.0), 0.48, 0.58)
check("Pk@1.5m_net", pk(1.5), 0.74, 0.84)
check("Pk@2.0m", pk(2.0), 0.87, 0.96)
check("Pk@2.5m", pk(2.5), 0.98, 1.00)

if fails:
    print(f"[check_m5] FAIL: {len(fails)} assertion(s) off: {', '.join(fails)}")
    sys.exit(1)
print("[check_m5] headline numbers OK (match ADR-0036)")
PY
NUM_OK=$?
[[ $NUM_OK -ne 0 ]] && exit 1

# Deliverable artifacts must exist.
MISSING=0
for p in docs/images/m5_pk_vs_radius_by_arm.png docs/images/m5_miss_hist_cdf.png \
         docs/images/m5_traj_overlay.png scripts/mc_analyze.py docs/decisions.md; do
    if [[ -e "$p" ]]; then echo "  [PASS] artifact present: $p"
    else echo "  [FAIL] artifact MISSING: $p"; MISSING=1; fi
done
grep -q "ADR-0036" docs/decisions.md && echo "  [PASS] ADR-0036 logged" || { echo "  [FAIL] ADR-0036 not in decisions.md"; MISSING=1; }
[[ $MISSING -ne 0 ]] && { echo "[check_m5] FAIL: missing deliverable artifact(s)"; exit 1; }

echo "[check_m5] PASS -- M5 headline reproduces from logs and all deliverables present."
exit 0
