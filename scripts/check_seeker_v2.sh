#!/usr/bin/env bash
# Seeker v2 (hard negatives + range calibration) — scripted close-out check.
# Asserts the MECHANISM gates that PASSED and the audit bar, against the logs
# (docs/seeker_v2_plan.md pre-registration; ADR-0042 holds the full verdict
# including the honestly-FAILED outcome gates 2/3 — this script asserts what
# the arc CLAIMS, not more).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
FAIL=0
note() { echo "[check_seeker_v2] $*"; }

# 1. Artifacts exist (v2 weights + calib sidecar; v1 untouched)
for f in scripts/seeker/weights/drone_finetuned_v2.onnx \
         scripts/seeker/weights/drone_finetuned_v2.onnx.calib.json \
         scripts/seeker/weights/drone_finetuned.onnx \
         logs/markerless_probe_v2/probe_results.csv \
         logs/mc_ab_markerless_tuned_v2.csv; do
    [[ -e "$f" ]] || { note "FAIL: missing $f"; FAIL=1; }
done
if [[ -e scripts/seeker/weights/drone_finetuned.onnx.calib.json ]]; then
    note "FAIL: v1 weights gained a calib sidecar (A/B comparability broken)"; FAIL=1
fi

# 2. Static probe gate: v2 fires on ALL 8 probe frames at conf >= 0.9
PROBE_OK=$(.venv-seeker/bin/python - <<'EOF'
import glob, subprocess, re
n_ok = 0
frames = sorted(glob.glob("logs/markerless_probe_v2/frames/r*.png"))
for f in frames:
    r = subprocess.run([".venv-seeker/bin/python", "scripts/seeker/finetuned_seeker.py",
                        "--weights", "scripts/seeker/weights/drone_finetuned_v2.onnx",
                        "--frame", f, "--conf", "0.5"],
                       capture_output=True, text=True)
    m = re.search(r"DETECT .*conf=(\d+\.\d+)", r.stdout)
    if "DETECT" in r.stdout and m and float(m.group(1)) >= 0.9:
        n_ok += 1
print(f"{n_ok}/{len(frames)}")
EOF
)
if [[ "$PROBE_OK" == "8/8" ]]; then note "PASS: static probe $PROBE_OK ranges at conf>=0.9"
else note "FAIL: static probe $PROBE_OK (need 8/8)"; FAIL=1; fi

# 3. Mechanism gates from the flight A/B (pollution collapse + honest range)
MECH=$(.venv/bin/python - <<'EOF'
import csv, statistics
rows_by = {r["cue_seed"]: r["flight_csv_path"] for r in csv.DictReader(open("logs/mc_ab_markerless_tuned_v2.csv"))}
ratios = []
n_poll = n_det = 0
for path in rows_by.values():
    for r in csv.DictReader(open(path)):
        if r.get("detected") != "1" or not r.get("gt_range") or not r.get("meas_range"):
            continue
        g = float(r["gt_range"])
        if g <= 0.5:
            continue
        n_det += 1
        ratio = float(r["meas_range"]) / g
        ratios.append(ratio)
        if ratio < 0.2:
            n_poll += 1
poll = n_poll / max(n_det, 1)
med = statistics.median(ratios) if ratios else 0.0
print(f"{poll:.4f} {med:.4f} {n_det}")
EOF
)
read -r POLL MED NDET <<< "$MECH"
if awk "BEGIN{exit !($POLL <= 0.15)}"; then note "PASS: pollution fraction $POLL <= 0.15 (n=$NDET; v1 was 0.751)"
else note "FAIL: pollution fraction $POLL > 0.15"; FAIL=1; fi
if awk "BEGIN{exit !($MED >= 0.7 && $MED <= 1.3)}"; then note "PASS: median meas/gt $MED in [0.7,1.3] (v1 was 0.056)"
else note "FAIL: median meas/gt $MED outside [0.7,1.3]"; FAIL=1; fi

# 4. Per-tick honesty audit: v2 arm must exit 0 (all flights pass a/b, powered-c)
if .venv/bin/python scripts/audit_per_tick.py logs/mc_ab_markerless_tuned_v2.csv > /dev/null 2>&1; then
    note "PASS: per-tick honesty audit exit 0 on the v2 arm"
else
    note "FAIL: per-tick honesty audit nonzero on the v2 arm"; FAIL=1
fi

# 5. cue_reads_post_handoff=0 on all 8 flights (structural no-cheat line)
NCUE=$(grep -l "" /dev/null 2>/dev/null; grep -o "cue_reads_post_handoff=0" logs/mc_batch_run_*.log 2>/dev/null | wc -l)
NRES=$(grep -c "cue_reads_post_handoff=0" logs/mc_ab_markerless_tuned_v2_stdout.log 2>/dev/null || echo 0)
if [[ "$NRES" -ge 8 ]]; then note "PASS: cue_reads_post_handoff=0 on $NRES/8 flights"
else note "FAIL: cue_reads_post_handoff=0 seen on only $NRES/8 flights"; FAIL=1; fi

# 6. Static honesty suite
if .venv/bin/python -m pytest tests/test_honesty_static.py -q > /dev/null 2>&1; then
    note "PASS: test_honesty_static.py suite"
else
    note "FAIL: test_honesty_static.py suite"; FAIL=1
fi

if [[ "$FAIL" -eq 0 ]]; then note "ALL CHECKS PASS"; exit 0
else note "CHECKS FAILED"; exit 1; fi
