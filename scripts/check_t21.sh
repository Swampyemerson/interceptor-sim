#!/usr/bin/env bash
# check_t21.sh -- scripted T21 gate for the ADR-0058 culmination headline
# (deep-audit 2026-07-10 finding H1: the 14/14 camera-terminal number had no
# scripted gate and lived only in gitignored logs).
#
# Re-runs scripts/analyze_track_ab.py against the COMMITTED aggregate CSV of
# the pre-registered deployment-config validation arm (`--track
# --handoff-cue-gate 8`, 12 m/s weave, n=16 paired seeds):
#   logs/mc_t21_trackgate_weave12_r2.csv
# and asserts the ADR-0058 three-way attribution (README "Detect-then-track
# terminal" row):
#   (i)   pooled miss_m Pk@2.5 m           == 16/16 (median ~2.11, max <= 2.50)
#   (ii)  post-handoff camera-terminal Pk  == 14/14 (median ~2.03 m)
#   (iii) real-handoff-conditioned Pk      == 14/14
#   PHANTOM handoffs                       == 0
#   terminal detections                    == 155 total, 0 FALSE (>8 m err)
# Exits 0 (pass) / 1 (fail). Sim-free: pure CSV analysis, no Gazebo.
#
# NOTE on scope: the analyzer derives (ii)/(iii) and the PHANTOM classification
# from the 16 per-tick flight CSVs referenced by flight_csv_path (big,
# deliberately NOT committed). This gate resolves those paths against this
# repo's logs/ dir by basename (so it survives a repo move) and fails LOUDLY if
# any are missing, rather than silently passing on the pooled number alone.
set -uo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"
VENV_PYTHON="${REPO}/.venv/bin/python"
ARM_CSV="${REPO}/logs/mc_t21_trackgate_weave12_r2.csv"

echo "[check_t21] repo=${REPO}"
[[ -f "$ARM_CSV" ]] || { echo "[check_t21] FAIL: committed aggregate missing: $ARM_CSV"; exit 1; }
[[ -f "scripts/analyze_track_ab.py" ]] || { echo "[check_t21] FAIL: scripts/analyze_track_ab.py missing"; exit 1; }

"$VENV_PYTHON" - "$ARM_CSV" <<'PY'
import csv, os, re, subprocess, sys, tempfile

arm_csv = sys.argv[1]
repo = os.getcwd()

# --- 1. Resolve per-tick flight CSV paths (absolute-on-origin-machine ->
#        fall back to this repo's logs/ by basename), fail loudly if gone.
rows = list(csv.DictReader(open(arm_csv)))
if len(rows) != 16:
    print(f"[check_t21] FAIL: expected n=16 flights in aggregate, got {len(rows)}")
    sys.exit(1)
missing = []
for r in rows:
    p = r.get("flight_csv_path", "")
    if not os.path.exists(p):
        alt = os.path.join(repo, "logs", os.path.basename(p))
        if os.path.exists(alt):
            r["flight_csv_path"] = alt
        else:
            missing.append(p)
if missing:
    print(f"[check_t21] FAIL: {len(missing)}/16 per-tick flight CSVs missing "
          f"(needed for the post-handoff / phantom attribution):")
    for p in missing:
        print(f"  MISSING {p}")
    print("[check_t21] these are local-only evidence (gitignored); restore from "
          "backup or re-fly the arm before re-asserting ADR-0058.")
    sys.exit(1)

tmp = tempfile.NamedTemporaryFile("w", suffix="_t21_resolved.csv",
                                  newline="", delete=False)
try:
    w = csv.DictWriter(tmp, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
    tmp.close()

    # --- 2. Re-run the ADR-0058 analyzer (the same tool the ADR cites).
    proc = subprocess.run(
        [sys.executable, os.path.join(repo, "scripts", "analyze_track_ab.py"),
         f"trackgate_weave12_r2={tmp.name}"],
        capture_output=True, text=True)
    out = proc.stdout
    print(out)
    if proc.returncode != 0:
        print(f"[check_t21] FAIL: analyzer exited {proc.returncode}\n{proc.stderr}")
        sys.exit(1)
finally:
    os.unlink(tmp.name)

# --- 3. Parse + assert the published three-way attribution.
def grab(pattern, what):
    m = re.search(pattern, out)
    if not m:
        print(f"[check_t21] FAIL: could not parse analyzer output for {what}")
        sys.exit(1)
    return m

pooled = grab(r"\(i\)\s+POOLED miss_m Pk@2\.5: (\d+)/(\d+)\s+median ([\d.]+) "
              r"mean [\d.]+ max ([\d.]+) m", "(i) pooled Pk")
post = grab(r"\(ii\) POST-HANDOFF min gt Pk@2\.5: (\d+)/(\d+)\s+median ([\d.]+)",
            "(ii) post-handoff Pk")
cond = grab(r"\(iii\) REAL-handoff-conditioned post-handoff Pk@2\.5: (\d+)/(\d+)",
            "(iii) conditioned Pk")
outcome = grab(r"HANDOFF OUTCOME: (.+)", "handoff outcome").group(1)
oc = dict(kv.split("=") for kv in outcome.split())
term = grab(r"TERMINAL DETS \(pooled ENGAGE\): (\d+) total\s+.*FALSE\(> 8\.0 m\)=(\d+)",
            "terminal detections")

fails = []
def check(name, got, lo, hi):
    ok = lo <= got <= hi
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {got:g} (expect [{lo:g},{hi:g}])")
    if not ok:
        fails.append(name)

print("[check_t21] asserting the ADR-0058 headline:")
check("(i) pooled hits",        int(pooled.group(1)), 16, 16)
check("(i) pooled n",           int(pooled.group(2)), 16, 16)
check("(i) pooled median_m",    float(pooled.group(3)), 2.05, 2.15)   # cite: 2.11
check("(i) pooled max_m",       float(pooled.group(4)), 0.0, 2.50)    # cite: 2.48
check("(ii) post-handoff hits", int(post.group(1)), 14, 14)
check("(ii) post-handoff n",    int(post.group(2)), 14, 14)
check("(ii) post median_m",     float(post.group(3)), 1.98, 2.08)     # cite: 2.03
check("(iii) conditioned hits", int(cond.group(1)), 14, 14)
check("(iii) conditioned n",    int(cond.group(2)), 14, 14)
check("PHANTOM handoffs",       int(oc.get("PHANTOM", 0)), 0, 0)
check("terminal dets total",    int(term.group(1)), 155, 155)
check("terminal dets FALSE",    int(term.group(2)), 0, 0)

if fails:
    print(f"[check_t21] FAIL: {len(fails)} assertion(s) off: {', '.join(fails)}")
    sys.exit(1)
print("[check_t21] PASS -- ADR-0058 three-way attribution reproduces from the "
      "committed aggregate (16/16 pooled, 14/14 camera-terminal, 0 phantoms, "
      "0/155 false terminal dets).")
PY
exit $?
