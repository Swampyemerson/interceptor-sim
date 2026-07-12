#!/usr/bin/env bash
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/sim_gpu_render.sh" 2>/dev/null || source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../sim_gpu_render.sh" 2>/dev/null || true  # GPU render (ADR-0075)
# Monte-Carlo BATCH runner (GOALS.md M5: "Monte-Carlo batch over target
# speeds/paths"). Flies N engagements per (law, speed) config through the
# SAME validated pipeline scripts/check_s2.sh gates (m4_intercept.py --fpv
# --handoff on the apriltag world), each with a DIFFERENT random draw, and
# appends ONE ROW PER FLIGHT to a single aggregate CSV so scripts/mc_analyze.py
# can compute Pk-vs-lethal-radius with honest confidence intervals.
#
# WHAT VARIES PER FLIGHT (two independent axes, both RNG-seeded from
# --master-seed so a whole batch is reproducible):
#   (1) --cue-seed forwarded to scripts/s2_cue_mock.py -- so the DEGRADED
#       cue's Gaussian sensor noise differs flight to flight (m4_intercept.py
#       already exposes --cue-seed, default 0, passed straight through to the
#       cue mock's own --seed; nothing was hardcoded here, see NEXT.md/this
#       task's brief -- verified by reading m4_intercept.py before writing
#       this script).
#   (2) A small crossing-geometry jitter on the target's start Y (+/-
#       --jitter meters, default 1.5) PLUS, under --directions both (the
#       default), an alternating left-to-right / right-to-left crossing --
#       mirrors scripts/guidance_lab.py's build_crossing() L2R/R2L idea
#       (direction flips BOTH start-y's sign and vy's sign together, so a
#       flipped target still actually crosses through the intercept
#       geometry instead of just flying away -- see that function's
#       docstring for the exact bug this avoids). Target start X is NEVER
#       jittered and stays > 0 (the interceptor's spawn X) so the rigid tag
#       face stays camera-facing throughout the engagement (ADR-0010 #6).
#
# WHAT DOES NOT VARY (out of scope for this baseline batch -- gated on a
# council decision the main session runs in parallel, per this task's
# brief): guidance gains, filter gains, detector constants, the tag model,
# DASH_SPEED / HANDOFF_RANGE_M. This script and m4_intercept.py are
# UNCHANGED from the S2-gated pipeline; it just flies it many times with
# different noise/geometry draws and records what happens.
#
# REUSES scripts/check_s2.sh's fresh-boot-per-flight machinery almost
# verbatim (kill_stale_sim / teardown_sim / the "wait for Startup script
# returned successfully, then settle, then gz-service pre-place the tag"
# sequence) -- see that script's own header comments for the "why" behind
# each step (ADR-0004 boot-complete string, ADR-0005 world symlink,
# ADR-0009 ready-settle). The one hard rule this project has learned about
# batch flights (NEXT.md): boot a COMPLETELY FRESH sim per flight -- the
# drone lands displaced, so state does NOT reset between flights inside one
# sim instance.
#
# ROBUSTNESS (this task's brief): a single crashed / timed-out / stuck-boot
# flight must NOT abort the whole batch -- it logs one row with miss=nan
# clean=0 and a breakoff_reason explaining what happened, then the loop
# CONTINUES to the next flight (which gets its own completely fresh sim
# boot). Only a handful of one-time PRE-FLIGHT problems (missing PX4 dir,
# missing venv, missing world file) abort the whole script, exactly like
# check_s2.sh.
#
# Usage:
#   scripts/mc_batch.sh --dry-run
#   scripts/mc_batch.sh --n 20 --laws pronav --speeds 6.0
#   scripts/mc_batch.sh --n 10 --laws pronav,pip --speeds 4.0,6.0,8.0 \
#       --directions both --master-seed 42
#
# Flags (all optional, defaults in parens):
#   --n N                 flights per (law, speed) config (20)
#   --laws L1,L2,...      comma-separated guidance laws: pursuit|pronav|pip (pronav)
#   --speeds S1,S2,...    comma-separated |target vy| values, m/s (6.0)
#   --directions MODE     both|l2r|r2l -- both alternates L2R/R2L across the
#                          n flights of each config (both)
#   --path MODE           line|weave|jink -- target maneuver path forwarded to
#                          m4_target_mover.py (via env, see below). line is the
#                          straight crosser (default, unchanged); weave is an
#                          S-curve; jink is sharp seed-scheduled turns (line)
#   --geometry MODE       standard|oblique_close -- crossing geometry.
#                          standard is the current fixed-downrange Y-crosser
#                          (unchanged). oblique_close is a constant-speed
#                          oblique CLOSING approach from the east (a westward
#                          vx added by rotating the crossing velocity by
#                          --west-angle-deg toward world -X) -- it exercises
#                          world_x-axis MOTION, but it is NOT the ADR-0026
#                          item B mirror-symmetry test (that gap is "target
#                          start_x never flipped negative"; closing it needs
#                          interceptor look-direction support in
#                          m4_intercept.py, out of this lane's file scope --
#                          reviewer correction, ADR-0033 path-suite lane).
#                          Treat oblique_close as its OWN distinct geometry,
#                          not a substitute for the axis-mirror test. Keeps
#                          target X>0 so the rigid -X-facing tag stays
#                          visible (ADR-0010 #6); the plan builder FAILS FAST
#                          if --x0/--west-angle-deg/--speeds don't leave at
#                          least MIN_CLOSING_RUNWAY_S=20s of closing time
#                          before the oblique leg would cross world X=0 --
#                          this is a LARGE --x0 at the 45deg/6.0 defaults
#                          (~85m: x0 >= X_FLOOR_M + 20*speed*cos(135deg)),
#                          since the runway must outlast the mover's whole
#                          scripted --duration, not just time-to-intercept.
#                          The error message states the exact minimum for
#                          your config. (standard)
#   --west-angle-deg D     oblique_close-geometry closing angle off the
#                          crossing axis, toward world -X, degrees (45.0)
#   --weave-period-s S     --path weave S-curve period, sim s (4.0)
#   --weave-lat-speed V    --path weave peak lateral speed, m/s (3.0)
#   --jink-count N         --path jink number of sharp turns (2)
#   --jink-min-deg D       --path jink minimum turn per jink, degrees (40.0)
#   --master-seed N       seeds the WHOLE plan's RNG draws (cue_seed + y
#                          jitter), reproducible given the same value (42)
#   --x0 X                target start X, meters, held fixed (never
#                          jittered) -- MUST stay > interceptor spawn X=0
#                          per ADR-0010 #6 (6.5, matches S2 default)
#   --y0-mag Y             |target start Y| before jitter/direction sign
#                          (14.0, matches S2's "real dash runway", ADR-0011
#                          third addendum)
#   --jitter J             +/- meters of uniform jitter applied to start Y (1.5)
#   --mode MODE            s2 (--fpv --handoff, the current validated
#                          pipeline) | m4 (plain camera-only ACQUIRE/ENGAGE,
#                          no --fpv/--handoff) (s2)
#   --extra-args "..."     forwarded verbatim to m4_intercept.py, word-split
#                          (mirrors check_s2.sh's EXTRA_ARGS convention --
#                          e.g. "--kalata --cue-latency-comp") ("")
#   --out PATH             override the aggregate CSV path (default
#                          logs/mc_batch_<UTCstamp>.csv)
#   --dry-run              print the planned flight matrix and exit 0 --
#                          NO sim boot, no CSV/log files written
#
# Output: logs/mc_batch_<UTCstamp>.csv (one row per flight -- see the
# header written near the top of the non-dry-run path below for the exact
# column list) and logs/mc_batch_run_<UTCstamp>.log (this script's own
# stdout/stderr, via `tee`; each flight ALSO gets its own
# logs/mc_batch_sim_<run_idx>_<flight-UTCstamp>.log /
# mc_batch_run_<run_idx>_<flight-UTCstamp>.log, exactly like
# check_s2_sim_*.log / check_s2_run_*.log).
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
# --seeker markerless needs onnxruntime (absent from the gated .venv); set
# MC_VENV_PYTHON to the combined venv (.venv-seeker + the gz/mavsdk bridge,
# ADR-0033 item 2) for markerless arms. Default keeps every gated apriltag arm
# on .venv, byte-identical.
VENV_PYTHON="${MC_VENV_PYTHON:-$REPO_ROOT/.venv/bin/python}"
# Markerless A/B knobs (ADR-0033 item 2). All default to the gated apriltag
# path, so an untouched batch is byte-identical. MC_WORLD selects BOTH the world
# name and the worlds/<name>.sdf file; MC_TARGET_MODEL is the set_pose handle;
# MC_SEEKER is forwarded verbatim to m4_intercept.py --seeker.
MC_WORLD="${MC_WORLD:-apriltag}"
MC_TARGET_MODEL="${MC_TARGET_MODEL:-apriltag_target}"
MC_SEEKER="${MC_SEEKER:-apriltag}"
LOGS_DIR="$REPO_ROOT/logs"
READY_TIMEOUT_S=120
# NOTE: do NOT wait for "Ready for takeoff!" -- ADR-0004 / check_s2.sh.
READY_STRING="Startup script returned successfully"
POST_READY_SETTLE_S=5
# Same headroom as check_s2.sh: CUE_WAIT 15s + DASH 20s + ENGAGE 30s budget
# plus connect/health/climb/land overhead.
PY_TIMEOUT_S=360

# --- defaults ---
N=20
LAWS="pronav"
SPEEDS="6.0"
DIRECTIONS="both"
MASTER_SEED=42
X0=6.5
Y0_MAG=14.0
JITTER=1.5
MODE="s2"
EXTRA_ARGS=""
OUT_CSV=""
DRY_RUN=0
PATH_MODE="line"
GEOMETRY="standard"
WEST_ANGLE_DEG=45.0
WEAVE_PERIOD_S=4.0
WEAVE_LAT_SPEED=3.0
JINK_COUNT=2
JINK_MIN_DEG=40.0

usage() {
    sed -n '2,111p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --n) N="$2"; shift 2;;
        --laws) LAWS="$2"; shift 2;;
        --speeds) SPEEDS="$2"; shift 2;;
        --directions) DIRECTIONS="$2"; shift 2;;
        --master-seed) MASTER_SEED="$2"; shift 2;;
        --x0) X0="$2"; shift 2;;
        --y0-mag) Y0_MAG="$2"; shift 2;;
        --jitter) JITTER="$2"; shift 2;;
        --mode) MODE="$2"; shift 2;;
        --path) PATH_MODE="$2"; shift 2;;
        --geometry) GEOMETRY="$2"; shift 2;;
        --west-angle-deg) WEST_ANGLE_DEG="$2"; shift 2;;
        --weave-period-s) WEAVE_PERIOD_S="$2"; shift 2;;
        --weave-lat-speed) WEAVE_LAT_SPEED="$2"; shift 2;;
        --jink-count) JINK_COUNT="$2"; shift 2;;
        --jink-min-deg) JINK_MIN_DEG="$2"; shift 2;;
        --extra-args) EXTRA_ARGS="$2"; shift 2;;
        --out) OUT_CSV="$2"; shift 2;;
        --dry-run) DRY_RUN=1; shift;;
        -h|--help) usage; exit 0;;
        *) echo "[mc_batch] Unknown argument: $1" >&2; usage; exit 1;;
    esac
done

if [[ "$MODE" != "s2" && "$MODE" != "m4" ]]; then
    echo "[mc_batch] FAIL: --mode must be 's2' or 'm4' (got '$MODE')" >&2
    exit 1
fi
if [[ "$DIRECTIONS" != "both" && "$DIRECTIONS" != "l2r" && "$DIRECTIONS" != "r2l" ]]; then
    echo "[mc_batch] FAIL: --directions must be 'both', 'l2r', or 'r2l' (got '$DIRECTIONS')" >&2
    exit 1
fi
if [[ "$PATH_MODE" != "line" && "$PATH_MODE" != "weave" && "$PATH_MODE" != "jink" ]]; then
    echo "[mc_batch] FAIL: --path must be 'line', 'weave', or 'jink' (got '$PATH_MODE')" >&2
    exit 1
fi
if [[ "$GEOMETRY" != "standard" && "$GEOMETRY" != "oblique_close" ]]; then
    echo "[mc_batch] FAIL: --geometry must be 'standard' or 'oblique_close' (got '$GEOMETRY')" >&2
    exit 1
fi
IFS=',' read -r -a LAW_ARR <<< "$LAWS"
for LAW in "${LAW_ARR[@]}"; do
    if [[ "$LAW" != "pursuit" && "$LAW" != "pronav" && "$LAW" != "pip" ]]; then
        echo "[mc_batch] FAIL: unknown law '$LAW' (expected pursuit, pronav, or pip)" >&2
        exit 1
    fi
done

# --- build the flight plan (deterministic given --master-seed): one line
# per flight, tab-separated: run_idx law speed direction start_x start_y
# vx vy cue_seed. Direction alternates l2r/r2l within each (law,speed)
# config's n flights when --directions both (default); jitter + cue_seed are
# both drawn from ONE random.Random(master_seed) instance in a fixed
# iteration order (law outer, speed middle, run index inner) so the WHOLE
# plan is reproducible from --master-seed alone. ---
PLAN_FILE="$(mktemp)"
"$VENV_PYTHON" - "$N" "$LAWS" "$SPEEDS" "$DIRECTIONS" "$MASTER_SEED" "$X0" "$Y0_MAG" "$JITTER" \
    "$GEOMETRY" "$WEST_ANGLE_DEG" \
    > "$PLAN_FILE" <<'PYEOF'
import math
import sys
import random

n = int(sys.argv[1])
laws = sys.argv[2].split(",")
speeds = [float(s) for s in sys.argv[3].split(",")]
directions_mode = sys.argv[4]
master_seed = int(sys.argv[5])
x0 = float(sys.argv[6])
y0_mag = float(sys.argv[7])
jitter = float(sys.argv[8])
geometry = sys.argv[9]
west_angle_deg = float(sys.argv[10])
west_angle = math.radians(west_angle_deg)

# Reviewer correction (ADR-0033 path-suite lane): oblique_close's vx<0 leg
# WILL cross world X=0 in finite time (x0/|vx|), after which the interceptor
# is west of the target and the rigidly -X-facing tag board (ADR-0010 #6)
# points away from it -- the same invariant scripts/m4_target_mover.py's
# X_FLOOR_M enforces for --path jink. Fail the WHOLE plan fast (before any
# sim boots) rather than silently booting a flight that is kinematically
# doomed by construction. X_FLOOR_M mirrors m4_target_mover.py's own
# constant; MIN_CLOSING_RUNWAY_S mirrors m4_intercept.py's
# S2["MOVER_DURATION_DEFAULT_S"] (20.0s, --mode s2's default and this
# script's default --mode) -- the conservative/binding case since --mode m4
# uses a shorter 12s mover duration.
X_FLOOR_M = 0.5
MIN_CLOSING_RUNWAY_S = 20.0

rng = random.Random(master_seed)
# A SEPARATE, independent RNG stream feeds the per-flight --path-seed so
# adding it does NOT perturb the existing y_jitter/cue_seed draw order --
# standard/line arms stay byte-identical to pre-M5 plans (no change to any
# existing arm's behavior). Deterministic given --master-seed.
path_rng = random.Random((master_seed * 2654435761 + 12345) & 0xFFFFFFFF)
run_idx = 0
for law in laws:
    for speed in speeds:
        for i in range(n):
            if directions_mode == "both":
                direction = "l2r" if i % 2 == 0 else "r2l"
            else:
                direction = directions_mode
            sign = 1.0 if direction == "l2r" else -1.0
            # These two draws are UNCHANGED in order/semantics from before.
            y_jitter = rng.uniform(-jitter, jitter)
            cue_seed = rng.randint(1, 999999)
            path_seed = path_rng.randint(1, 999999)
            start_y = -sign * y0_mag + y_jitter
            if geometry == "standard":
                # Fixed-downrange Y-crosser (current behavior, verbatim).
                start_x = x0
                vx = 0.0
                vy = sign * speed
            elif geometry == "oblique_close":
                # constant-speed oblique CLOSING approach from the east.
                # Rotate the crossing heading (sign*90 deg) toward world -X
                # by west_angle, preserving |v|=speed; vx<0 (closing),
                # vy=sign*|lateral| still crosses -> the L2R/R2L pair survives
                # "by construction" (ADR-0033). start_x stays at x0 (>0), so
                # the -X-facing tag stays visible through CPA (ADR-0010 #6).
                # NOTE: this is NOT the ADR-0026 item B axis-mirror test (see
                # the --geometry usage comment above) -- it is its own
                # distinct world_x-motion geometry.
                heading = sign * (math.pi / 2.0) + sign * west_angle
                start_x = x0
                vx = speed * math.cos(heading)
                vy = speed * math.sin(heading)
                if vx < 0:
                    t_cross = (start_x - X_FLOOR_M) / abs(vx)
                    if t_cross < MIN_CLOSING_RUNWAY_S:
                        print(
                            f"geometry=oblique_close: x0={x0} west_angle_deg="
                            f"{west_angle_deg} speed={speed} direction={direction} "
                            f"gives only {t_cross:.1f}s of closing runway before "
                            f"the target crosses world X={X_FLOOR_M} (need >= "
                            f"{MIN_CLOSING_RUNWAY_S}s). Raise --x0 or lower "
                            "--west-angle-deg / --speeds.",
                            file=sys.stderr,
                        )
                        sys.exit(1)
            else:
                print(f"unknown --geometry {geometry!r}", file=sys.stderr)
                sys.exit(1)
            print(
                "\t".join(
                    [
                        str(run_idx), law, f"{speed:.3f}", direction,
                        f"{start_x:.3f}", f"{start_y:.3f}", f"{vx:.3f}", f"{vy:.3f}",
                        str(cue_seed), str(path_seed),
                    ]
                )
            )
            run_idx += 1
PYEOF
PLAN_EXIT=$?
if [[ "$PLAN_EXIT" -ne 0 ]]; then
    echo "[mc_batch] FAIL: flight-plan generation failed (exit $PLAN_EXIT) -- see error above (e.g. an oblique_close runway check). No sim will be booted." >&2
    rm -f "$PLAN_FILE"
    exit 1
fi

TOTAL_FLIGHTS="$(wc -l < "$PLAN_FILE" | tr -d ' ')"

if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "[mc_batch] DRY RUN -- planned $TOTAL_FLIGHTS flights (mode=$MODE, master_seed=$MASTER_SEED)."
    echo "[mc_batch] path=$PATH_MODE geometry=$GEOMETRY west_angle_deg=$WEST_ANGLE_DEG x0=$X0 y0_mag=$Y0_MAG"
    echo "[mc_batch] No sim will be booted, no files written."
    printf "%-8s %-8s %-6s %-5s %-8s %-8s %-8s %-8s %-8s %-8s\n" \
        run_idx law speed dir start_x start_y vx vy cue_seed path_seed
    while IFS=$'\t' read -r run_idx law speed direction sx sy vx vy cue_seed path_seed; do
        printf "%-8s %-8s %-6s %-5s %-8s %-8s %-8s %-8s %-8s %-8s\n" \
            "$run_idx" "$law" "$speed" "$direction" "$sx" "$sy" "$vx" "$vy" "$cue_seed" "$path_seed"
    done < "$PLAN_FILE"
    rm -f "$PLAN_FILE"
    exit 0
fi

# --- pre-flight checks (fatal -- same set as check_s2.sh) ---
if [[ ! -d "$PX4_DIR" ]]; then
    echo "[mc_batch] FAIL: PX4-Autopilot not found at $PX4_DIR" >&2
    rm -f "$PLAN_FILE"
    exit 1
fi
if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[mc_batch] FAIL: venv python not found at $VENV_PYTHON" >&2
    rm -f "$PLAN_FILE"
    exit 1
fi
REPO_WORLD="$REPO_ROOT/worlds/${MC_WORLD}.sdf"
if [[ ! -f "$REPO_WORLD" ]]; then
    echo "[mc_batch] FAIL: $REPO_WORLD not found" >&2
    rm -f "$PLAN_FILE"
    exit 1
fi
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/${MC_WORLD}.sdf"
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    echo "[mc_batch] Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi

mkdir -p "$LOGS_DIR"
mkdir -p "$REPO_ROOT/plots"

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "$OUT_CSV" ]]; then
    OUT_CSV="$LOGS_DIR/mc_batch_${TIMESTAMP}.csv"
fi
BATCH_LOG="$LOGS_DIR/mc_batch_run_${TIMESTAMP}.log"

# Log everything (this script's own stdout/stderr) to both the terminal AND
# a batch log file (task requirement: "Log progress to stdout + a batch
# log"). Per-flight sim/run logs are separate files (see the header).
exec > >(tee -a "$BATCH_LOG") 2>&1

echo "[mc_batch] ================================================"
echo "[mc_batch] Monte-Carlo batch starting at $TIMESTAMP"
echo "[mc_batch] Plan: $TOTAL_FLIGHTS flights, laws=$LAWS speeds=$SPEEDS directions=$DIRECTIONS"
echo "[mc_batch] mode=$MODE x0=$X0 y0_mag=$Y0_MAG jitter=$JITTER master_seed=$MASTER_SEED"
echo "[mc_batch] path=$PATH_MODE geometry=$GEOMETRY west_angle_deg=$WEST_ANGLE_DEG weave_period_s=$WEAVE_PERIOD_S weave_lat_speed=$WEAVE_LAT_SPEED jink_count=$JINK_COUNT jink_min_deg=$JINK_MIN_DEG"
echo "[mc_batch] Aggregate CSV: $OUT_CSV"
echo "[mc_batch] Batch log: $BATCH_LOG"
echo "[mc_batch] ================================================"

"$VENV_PYTHON" - "$OUT_CSV" <<'PYEOF'
import csv
import sys

path = sys.argv[1]
header = [
    "run_idx", "law", "target_vx", "target_vy", "target_start_y", "cue_seed",
    "miss_m", "clean", "handoff", "handoff_range_m", "handoff_t_s",
    "first_det_range_m", "coverage", "breakoff_reason", "flight_csv_path",
    # extra informational columns (harmless -- mc_analyze.py reads by name)
    "config", "direction", "target_start_x", "py_exit_code",
    "sim_log_path", "run_log_path",
    # M5 path/geometry provenance (trailing, read-by-name safe)
    "path", "geometry", "path_seed",
]
with open(path, "w", newline="") as f:
    csv.writer(f).writerow(header)
PYEOF

SIM_PID=""

kill_stale_sim() {
    echo "[mc_batch] Killing any stale px4 / gz / cue-mock / mover processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    pkill -f "s2_cue_mock.py" 2>/dev/null
    pkill -f "m4_target_mover.py" 2>/dev/null
    sleep 2
}

teardown_sim() {
    echo "[mc_batch] Tearing down sim processes..."
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        kill -TERM -- "-$SIM_PID" 2>/dev/null
        sleep 2
        kill -KILL -- "-$SIM_PID" 2>/dev/null
    fi
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    pkill -f "s2_cue_mock.py" 2>/dev/null
    pkill -f "m4_target_mover.py" 2>/dev/null
    sleep 3
    SIM_PID=""
}
trap teardown_sim EXIT

# record_row: ALWAYS appends exactly one row to $OUT_CSV, whether the flight
# succeeded, crashed, or never got a sim to boot -- this is the "one bad
# flight cannot abort the batch" contract. Parses the flight's own RUN_LOG
# text for the S2_RESULT / M4_RESULT / handoff / coverage / breakoff_reason
# lines (see scripts/m4_intercept.py's docstring + check_s2.sh's own
# grep -oP parsing for the exact line formats this mirrors), using Python's
# csv module for correct quoting (breakoff_reason strings contain spaces
# and parentheses).
record_row() {
    local run_idx="$1" law="$2" speed="$3" direction="$4"
    local tsx="$5" tsy="$6" tvx="$7" tvy="$8" cue_seed="$9"
    local run_log="${10}" py_exit="${11}" forced_reason="${12:-}" sim_log="${13:-}"
    local path_seed="${14:-}"

    "$VENV_PYTHON" - "$OUT_CSV" "$run_idx" "$law" "$tvx" "$tvy" "$tsy" "$cue_seed" \
        "$run_log" "$py_exit" "$forced_reason" "${law}_${speed}" "$direction" "$tsx" "$sim_log" \
        "$PATH_MODE" "$GEOMETRY" "$path_seed" \
        <<'PYEOF'
import csv
import math
import re
import sys

(
    out_csv, run_idx, law, tvx, tvy, tsy, cue_seed,
    run_log, py_exit, forced_reason, config, direction, tsx, sim_log,
    path_mode, geometry, path_seed,
) = sys.argv[1:18]

miss = float("nan")
clean = 0
handoff = ""
handoff_range = ""
handoff_t = ""
first_det_range = ""
coverage = ""
breakoff_reason = forced_reason
flight_csv_path = ""

text = ""
if run_log and run_log != "-":
    try:
        with open(run_log, "r", errors="replace") as f:
            text = f.read()
    except OSError:
        text = ""

if text:
    s2_match = re.search(r"^S2_RESULT (.+)$", text, re.MULTILINE)
    m4_match = re.search(r"^M4_RESULT (.+)$", text, re.MULTILINE)
    result_line = s2_match.group(1) if s2_match else (m4_match.group(1) if m4_match else None)

    if result_line is not None:
        m = re.search(r"miss=(\S+)", result_line)
        try:
            miss = float(m.group(1)) if m else float("nan")
        except ValueError:
            miss = float("nan")
        m = re.search(r"clean=(\S+)", result_line)
        clean = int(m.group(1)) if m and m.group(1) in ("0", "1") else 0
        # Leading space required (not just "handoff=") -- distinguishes from
        # the substring "..._post_handoff=0" later in the same S2_RESULT line
        # (mirrors check_s2.sh's own grep -oP '(?<= handoff=)[^ ]+').
        m = re.search(r" handoff=(\S+)", result_line)
        if m:
            handoff = m.group(1)
        m = re.search(r"handoff_range=(\S+)", result_line)
        if m and m.group(1) != "nan":
            handoff_range = m.group(1)
        m = re.search(r"handoff_t=(\S+)", result_line)
        if m and m.group(1) != "nan":
            handoff_t = m.group(1)

    m = re.search(r"First camera detection during DASH at range=([0-9.]+)", text)
    if m:
        first_det_range = m.group(1)

    m = re.search(
        r"^\[m4\] law=\S+ miss_distance_m=\S+ n_ticks=\d+ coverage=([0-9.]+) "
        r"breakoff_reason=(.*?) aborted=\S+ log=(\S+)$",
        text, re.MULTILINE,
    )
    if m:
        coverage = m.group(1)
        breakoff_reason = m.group(2).strip()
        flight_csv_path = m.group(3)
    elif not breakoff_reason:
        breakoff_reason = "no_coverage_line_in_run_log"
else:
    if not breakoff_reason:
        breakoff_reason = "no_run_log"

if not flight_csv_path and text:
    m = re.search(r"\[m4\] Logging to (\S+\.csv)", text)
    if m:
        flight_csv_path = m.group(1)

row = [
    run_idx, law, tvx, tvy, tsy, cue_seed,
    "nan" if math.isnan(miss) else f"{miss:.4f}",
    clean, handoff, handoff_range, handoff_t,
    first_det_range, coverage, breakoff_reason, flight_csv_path,
    config, direction, tsx, py_exit, sim_log, run_log,
    path_mode, geometry, path_seed,
]
with open(out_csv, "a", newline="") as f:
    csv.writer(f).writerow(row)
print(
    f"[mc_batch] Recorded run_idx={run_idx} law={law} miss={row[6]} "
    f"clean={clean} handoff={handoff or 'n/a'} breakoff_reason={breakoff_reason!r}"
)
PYEOF
}

run_counter=0

while IFS=$'\t' read -r run_idx law speed direction tsx tsy tvx tvy cue_seed path_seed; do
    run_counter=$((run_counter + 1))
    echo ""
    echo "[mc_batch] ============ Flight $run_counter/$TOTAL_FLIGHTS (run_idx=$run_idx) ============"
    echo "[mc_batch] law=$law speed=$speed direction=$direction target_start=(${tsx},${tsy},0.5) target_vel=(${tvx},${tvy}) cue_seed=$cue_seed path=$PATH_MODE path_seed=$path_seed geometry=$GEOMETRY"

    kill_stale_sim

    FLIGHT_TS="$(date -u +%Y%m%dT%H%M%SZ)"
    SIM_LOG="$LOGS_DIR/mc_batch_sim_${run_idx}_${FLIGHT_TS}.log"
    RUN_LOG="$LOGS_DIR/mc_batch_run_${run_idx}_${FLIGHT_TS}.log"
    forced_reason=""

    echo "[mc_batch] Launching HEADLESS PX4 SITL + Gazebo (gz_x500_mono_cam, world=${MC_WORLD})..."
    echo "[mc_batch] Sim output -> $SIM_LOG"
    setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=${MC_WORLD} GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    echo "[mc_batch] Sim launched (pid/pgid $SIM_PID)."

    ready=0
    elapsed=0
    while (( elapsed < READY_TIMEOUT_S )); do
        if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
            ready=1
            break
        fi
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "[mc_batch] WARNING: sim process exited early (pid $SIM_PID)."
            break
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    if [[ "$ready" -ne 1 ]]; then
        echo "[mc_batch] WARNING: sim did not become ready within ${READY_TIMEOUT_S}s for run_idx=$run_idx. See $SIM_LOG"
        echo "[mc_batch] Recording failure row and continuing to the next flight."
        record_row "$run_idx" "$law" "$speed" "$direction" "$tsx" "$tsy" "$tvx" "$tvy" "$cue_seed" \
            "-" "" "sim_boot_timeout" "$SIM_LOG" "$path_seed"
        teardown_sim
        continue
    fi

    echo "[mc_batch] Sim ready. Sleeping ${POST_READY_SETTLE_S}s to let the EKF settle..."
    sleep "$POST_READY_SETTLE_S"

    echo "[mc_batch] Pre-placing the tag at (${tsx}, ${tsy}, 0.5)..."
    placed=0
    for attempt in 1 2 3; do
        if gz service -s /world/${MC_WORLD}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
            --req "name: \"${MC_TARGET_MODEL}\" position { x: $tsx y: $tsy z: 0.5 }"; then
            placed=1
            break
        fi
        echo "[mc_batch] WARNING: tag pre-placement attempt $attempt failed, retrying..."
        sleep 1
    done
    if [[ "$placed" -ne 1 ]]; then
        echo "[mc_batch] WARNING: could not pre-place the tag after 3 attempts for run_idx=$run_idx."
        echo "[mc_batch] Recording failure row and continuing to the next flight."
        record_row "$run_idx" "$law" "$speed" "$direction" "$tsx" "$tsy" "$tvx" "$tvy" "$cue_seed" \
            "-" "" "tag_preplacement_failed" "$SIM_LOG" "$path_seed"
        teardown_sim
        continue
    fi
    echo "[mc_batch] Tag pre-placed."

    MODE_ARGS=()
    if [[ "$MODE" == "s2" ]]; then
        MODE_ARGS=(--fpv --handoff)
    fi

    # NB: --target-vel uses the =-attached form because oblique_close's vx is
    # NEGATIVE (e.g. -4.243); a space-separated "-4.243,4.243" is mis-read by
    # argparse as an option flag ("expected one argument"). The =-form is
    # byte-identical for the positive/zero vx of every other geometry.
    echo "[mc_batch] Running m4_intercept.py ${MODE_ARGS[*]:-} --law $law --seeker $MC_SEEKER --target-start=${tsx},${tsy},0.5 --target-vel=${tvx},${tvy} --cue-seed $cue_seed $EXTRA_ARGS (outer timeout ${PY_TIMEOUT_S}s)..."
    echo "[mc_batch] Run output -> $RUN_LOG (also streamed below)"
    # The path is plumbed to the mover through the ENVIRONMENT (m4_intercept.py
    # is UNCHANGED -- it spawns m4_target_mover.py as a subprocess that
    # inherits this env; same INTERCEPTOR_WORLD_NAME pattern). The env-var
    # prefix applies to `timeout` and is inherited by the python child + the
    # mover it spawns. line/standard leave these at the mover's own defaults,
    # so an untouched line/standard batch is byte-identical to before.
    # shellcheck disable=SC2086
    INTERCEPTOR_TARGET_PATH="$PATH_MODE" \
    INTERCEPTOR_PATH_SEED="$path_seed" \
    INTERCEPTOR_WEAVE_PERIOD_S="$WEAVE_PERIOD_S" \
    INTERCEPTOR_WEAVE_LAT_SPEED="$WEAVE_LAT_SPEED" \
    INTERCEPTOR_JINK_COUNT="$JINK_COUNT" \
    INTERCEPTOR_JINK_MIN_DEG="$JINK_MIN_DEG" \
    INTERCEPTOR_WORLD_NAME="$MC_WORLD" \
    INTERCEPTOR_TARGET_MODEL="$MC_TARGET_MODEL" \
    timeout "$PY_TIMEOUT_S" "$VENV_PYTHON" "$REPO_ROOT/scripts/m4_intercept.py" \
        "${MODE_ARGS[@]}" --law "$law" --seeker "$MC_SEEKER" \
        --target-start="${tsx},${tsy},0.5" --target-vel="${tvx},${tvy}" \
        --cue-seed "$cue_seed" $EXTRA_ARGS 2>&1 | tee "$RUN_LOG"
    PY_EXIT="${PIPESTATUS[0]}"

    if [[ "$PY_EXIT" -eq 124 ]]; then
        forced_reason="outer_python_timeout_${PY_TIMEOUT_S}s"
        echo "[mc_batch] WARNING: m4_intercept.py hit the outer ${PY_TIMEOUT_S}s timeout."
    elif [[ "$PY_EXIT" -ne 0 ]]; then
        forced_reason="python_exit_${PY_EXIT}"
    fi

    record_row "$run_idx" "$law" "$speed" "$direction" "$tsx" "$tsy" "$tvx" "$tvy" "$cue_seed" \
        "$RUN_LOG" "$PY_EXIT" "$forced_reason" "$SIM_LOG" "$path_seed"

    teardown_sim
done < "$PLAN_FILE"

rm -f "$PLAN_FILE"

echo ""
echo "[mc_batch] ================================================"
echo "[mc_batch] Batch complete: $run_counter/$TOTAL_FLIGHTS flights attempted."
echo "[mc_batch] Aggregate CSV: $OUT_CSV"
echo "[mc_batch] Batch log: $BATCH_LOG"
echo "[mc_batch] Next: .venv/bin/python scripts/mc_analyze.py $OUT_CSV"
echo "[mc_batch] ================================================"
