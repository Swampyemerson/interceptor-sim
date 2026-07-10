#!/usr/bin/env bash
# =============================================================================
# mc_t21_trackgate_jink16.sh -- TASK #33 TOP STATS ITEM: the ADR-0058 ADOPTED
# deployment config, JINK path, n=16 paired, POST-FIX code.
#
# WHAT THIS VALIDATES:
#   The adopted deployment config (`--track --handoff-cue-gate 8` + dash-speed
#   16 / early-handoff / cue-velocity / dash-unclamp / fuse-midcourse) has only
#   ever been BATCH-validated on WEAVE (r2 headline arm, 16/16 pooled,
#   camera-terminal 14/14 -- ADR-0058). The jink numbers in ADR-0058
#   (track_jink 15/16, camera-terminal 14/15) flew PRE-FIX code (wall-time
#   aging + cap-20) and WITHOUT --handoff-cue-gate. This arm closes that gap:
#   the exact adopted config, weave->jink, everything else identical to r2.
#
# EXACT MIRROR OF THE r2 HEADLINE ARM (logs/mc_t21_trackgate_weave12_r2_stdout.log):
#   same env, same extra-args (argv order preserved), same n=16 / master-seed
#   42 / speeds 12 / geometry standard / x0 6.5 / y0-mag 29.3 -- only
#   --path weave -> jink and the --out name change. Jink shape params stay the
#   project defaults (jink_count=2, jink_min_deg=40) -- identical to every
#   prior jink arm (see logs/mc_t21_plain_jink12_n16_stdout.log header).
#
# PAIRED COMPARATORS (all master-seed 42 -> per-seed pairing is exact):
#   logs/mc_t21_plain_jink12_n16.csv   plain markerless jink baseline (3/16, med 3.58)
#   logs/mc_t21_track_jink12.csv       pre-fix --track jink arm (15/16 pooled)
#   logs/mc_t21_trackgate_weave12_r2.csv  the adopted-config weave headline (16/16)
#
# VERDICT RULES (pre-registered):
#   - Report all three Pk levels (pooled / camera-terminal / REAL-conditioned)
#     per ADR-0058 -- pooled alone is flattering.
#   - Binomial metric: n=16 needs Wilson-CI language (ADR-0041 F5); single-
#     flight deltas < ~1 m are noise; the 12 m/s maneuver false-lock noise
#     floor is ~5 m (ADR-0057).
#   - Post-run honesty bar: audit_per_tick.py; three-way attribution:
#     analyze_track_ab.py against the comparators above.
#
# EXPECTED DURATION: ~20 min (r2: 16 flights in ~19 min at idle).
#
# USAGE (default ECHO-ONLY; nothing runs):
#   scripts/mc_t21_trackgate_jink16.sh            # print env + exact command
#   scripts/mc_t21_trackgate_jink16.sh --dry-run  # mc_batch plan print, NO sim
#   scripts/mc_t21_trackgate_jink16.sh --go       # fly (idle load only; run backgrounded)
# Wait on the sentinel ONLY (never inline pgrep/pkill with sim substrings):
#   for i in $(seq 1 120); do grep -q "Batch complete" \
#       logs/mc_t21_trackgate_jink12_stdout.log && break; sleep 15; done
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "$SCRIPT_DIR/mc_deployment_arm.sh" \
    --path jink --n 16 --master-seed 42 --speeds 12 \
    --out logs/mc_t21_trackgate_jink12.csv \
    "$@"
