#!/usr/bin/env bash
# =============================================================================
# stats_hardening_options.sh -- TASK #33: the two statistically honest paths
# to Pk language for the adopted deployment config (ADR-0058, weave 12 m/s).
#
# THE MATH (task #33 correction; Clopper-Pearson, two-sided 95% convention,
# i.e. alpha/2 = 0.025 in the lower tail -- the project's Wilson/CI convention,
# ADR-0041 F5; exact for x=n: LB = 0.025^(1/n)):
#   n=16 all-clean (r2 today)  -> LB 79.4%   ("Pk >= 95%" NOT supportable)
#   n=48 all-clean             -> LB 92.6%   (0.025^(1/48))
#   n=72 all-clean             -> LB 95.0%   (0.95^72 = 0.0249 <= 0.025; n=71 fails)
# ANY unclean flight kills the closed form; general LB (x of n) =
#   scipy.stats.beta.ppf(0.025, x, n - x + 1)   (.venv has scipy)
#
# THE TWO OPTIONS (main session picks ONE):
#   OPTION A -- fly to n=72 for a real "Pk >= 95% (95% CI)" claim.
#     A1 POOL: keep r2's 16/16 + fly 56 new (2 sub-arms of 28, master seeds
#        1042/2042 so no seed-point duplicates r2's stream). Pooling is
#        legitimate ONLY because config+code are byte-identical post-fix and
#        the pooling is pre-registered here, before the new flights fly.
#     A2 FRESH: one homogeneous 72 = 3 sub-arms of 24 (master seeds
#        42/1042/2042); no pooling argument needed; costs 16 more flights.
#   OPTION B -- honest reframe at n=48: fly ONE arm of 32 (master seed 1042),
#     pool with r2's 16 -> "48/48 clean, 95% CI lower bound 92.6%".
#
# DENOMINATOR HONESTY (pre-registered): the LB applies to whichever Pk level
# keeps a perfect numerator. r2's CAMERA-TERMINAL denominator was 14 of 16
# (two legal never-handoff characterizations excluded), so pooled n=72 yields
# ~63 camera-terminal flights -> LB ~94.3% (NOT >=95%). A camera-terminal
# ">=95%" claim needs ~72 CT flights ~= 83 pooled. The printout below computes
# all of this exactly. Report all three levels (pooled / camera-terminal /
# REAL-conditioned) per ADR-0058, whichever option is flown.
#
# BATCH DISCIPLINE (mc-batch skill): sub-arms run SEQUENTIALLY, >=120 s
# cooldown + dxgk check between; ~48 boots is the proven-safe cluster (no
# sub-arm here exceeds 32); each sub-arm has its own --out CSV so a wedge
# loses only that sub-arm (resume it, never restart). ~73 s/flight at idle
# (r2 measured): A1 ~68 min + cooldowns, A2 ~88 min, B ~39 min.
#
# USAGE (default: print the analysis + exact ready-to-run commands, run NOTHING):
#   scripts/stats_hardening_options.sh            # the note + commands
#   scripts/stats_hardening_options.sh --dry-run A1|A2|B   # plan-print sub-arms (NO sim)
#   scripts/stats_hardening_options.sh --go A1|A2|B        # fly the chosen plan
# Merge/analyze afterwards (identical headers; NEVER pool across paths):
#   head -1 first.csv > logs/mc_t21_trackgate_weave12_n72.csv && \
#     tail -q -n +2 <sub-arm CSVs...> >> logs/mc_t21_trackgate_weave12_n72.csv
#   .venv/bin/python scripts/mc_analyze.py logs/mc_t21_trackgate_weave12_n72.csv
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

LAUNCH="$SCRIPT_DIR/mc_deployment_arm.sh"

# sub-arm spec: "<n> <master_seed> <out_csv>"
A1_ARMS=(
    "28 1042 logs/mc_t21_trackgate_weave12_n72a.csv"
    "28 2042 logs/mc_t21_trackgate_weave12_n72b.csv"
)
A2_ARMS=(
    "24 42   logs/mc_t21_trackgate_weave12_f72a.csv"
    "24 1042 logs/mc_t21_trackgate_weave12_f72b.csv"
    "24 2042 logs/mc_t21_trackgate_weave12_f72c.csv"
)
B_ARMS=(
    "32 1042 logs/mc_t21_trackgate_weave12_n48b.csv"
)

print_math() {
    python3 - <<'PY'
import math
def lb(n, alpha=0.025):           # exact Clopper-Pearson LB for x = n
    return alpha ** (1.0 / n)
print("Exact Clopper-Pearson lower bounds (all-clean, two-sided 95% -> alpha/2=0.025):")
for n in (14, 16, 48, 63, 72):
    print(f"  {n:>3}/{n:<3} clean -> LB = {lb(n)*100:6.2f}%")
n95 = next(n for n in range(1, 200) if 0.95 ** n <= 0.025)
print(f"  minimal n for LB >= 95%: n = {n95}  (0.95^{n95} = {0.95**n95:.4f}; "
      f"0.95^{n95-1} = {0.95**(n95-1):.4f} fails)")
ct_ratio = 14.0 / 16.0            # r2 camera-terminal denominator fraction
pooled_needed = math.ceil(n95 / ct_ratio)
print(f"  camera-terminal check: pooled n=72 -> ~{int(72*ct_ratio)} CT flights "
      f"-> LB ~{lb(int(72*ct_ratio))*100:.1f}% (NOT >=95%)")
print(f"  a camera-terminal >=95% claim needs ~{n95} CT ~= {pooled_needed} pooled flights")
print( "  one-sided-95% convention (for reference only): n=59 suffices "
      f"(0.95^59 = {0.95**59:.4f} <= 0.05) -- project convention is two-sided")
print( "  general LB (x of n): scipy.stats.beta.ppf(0.025, x, n - x + 1)")
PY
}

print_plan() {
    local label="$1"; shift
    local arms=("$@")
    local total=0
    echo ""
    echo "--- $label ---"
    for spec in "${arms[@]}"; do
        read -r n seed out <<< "$spec"
        total=$((total + n))
        echo "  $LAUNCH --path weave --n $n --master-seed $seed --out $out --go"
    done
    echo "  (sequential; sleep 120 + dxgk check between sub-arms; ~$((total * 73 / 60)) min flying)"
}

run_arms() {
    local first=1
    for spec in "$@"; do
        read -r n seed out <<< "$spec"
        if [[ "$first" -ne 1 ]]; then
            echo "[stats-hardening] cooldown 120 s before next sub-arm (mc-batch skill)..."
            sleep 120
        fi
        first=0
        "$LAUNCH" --path weave --n "$n" --master-seed "$seed" --out "$out" "$MODE_FLAG"
    done
}

MODE="print"
CHOICE=""
if [[ $# -gt 0 ]]; then
    case "$1" in
        --go) MODE="go"; MODE_FLAG="--go"; CHOICE="${2:-}";;
        --dry-run) MODE="dry"; MODE_FLAG="--dry-run"; CHOICE="${2:-}";;
        *) echo "usage: $0 [--dry-run A1|A2|B] [--go A1|A2|B]" >&2; exit 2;;
    esac
    if [[ "$MODE" != "print" && -z "$CHOICE" ]]; then
        echo "usage: $0 $1 A1|A2|B" >&2; exit 2
    fi
fi

if [[ "$MODE" == "print" ]]; then
    print_math
    echo ""
    echo "Baseline already on disk: logs/mc_t21_trackgate_weave12_r2.csv (16/16 pooled,"
    echo "camera-terminal 14/14 -- ADR-0058 headline arm, post-fix adopted config)."
    print_plan "OPTION A1 (pool r2 16 + 56 new -> 72; pre-registered pooling)" "${A1_ARMS[@]}"
    print_plan "OPTION A2 (fresh homogeneous 72; no pooling argument)" "${A2_ARMS[@]}"
    print_plan "OPTION B  (reframe: +32 -> 48/48, claim 'LB 92.6%')" "${B_ARMS[@]}"
    echo ""
    echo "Run '$0 --dry-run <choice>' to plan-print (no sim), '--go <choice>' to fly."
    exit 0
fi

case "$CHOICE" in
    A1) run_arms "${A1_ARMS[@]}";;
    A2) run_arms "${A2_ARMS[@]}";;
    B)  run_arms "${B_ARMS[@]}";;
    *) echo "[stats-hardening] FAIL: choice must be A1, A2 or B (got '$CHOICE')" >&2; exit 2;;
esac
echo "[stats-hardening] $CHOICE complete. Merge sub-arm CSVs (see header) and run"
echo "  .venv/bin/python scripts/mc_analyze.py <merged.csv>"
echo "Report pooled + camera-terminal + REAL-conditioned levels (ADR-0058)."
