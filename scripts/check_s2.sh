#!/usr/bin/env bash
# S2 scripted milestone gate (M4.5 sub-step S2, ADR-0010 decisions #4/#5):
# for EACH guidance law (pip, then pronav), boots a completely fresh
# headless PX4 SITL + Gazebo instance (gz_x500_mono_cam on the custom
# "apriltag" world), pre-places the AprilTag at the S2 long-standoff dash
# start position, then runs scripts/m4_intercept.py --fpv --handoff to
# exercise the full CUE_WAIT -> DASH -> HANDOFF -> ENGAGE/BREAKOFF two-stage
# architecture (scripts/s2_cue_mock.py is spawned automatically by that
# script's CUE_WAIT phase, scripts/m4_target_mover.py by its DASH phase).
# See GOALS.md, docs/decisions.md ADR-0010 (#4 cue mock, #5 handoff) +
# ADR-0011 third addendum (the lab-validated S2 config this gate exercises:
# dash 10 m/s, handoff 10 m), scripts/check_s1.sh (the pattern this is
# copied from), and .claude/skills/px4-gazebo/SKILL.md /
# .claude/skills/sim-debug/SKILL.md for launch/shutdown/debug conventions.
#
# Two full boots (one per law) rather than one boot reused for both runs:
# each law gets a clean vehicle/EKF/tag state, so neither run can be
# influenced by whatever the other run left behind (same reasoning as
# check_m4.sh / check_s1.sh).
#
# Usage: scripts/check_s2.sh [law]
#   scripts/check_s2.sh            # both laws: pip, then pronav
#   scripts/check_s2.sh pip        # just pip (dev iteration)
#   scripts/check_s2.sh pronav     # just pronav
#   S2_LAW=pip scripts/check_s2.sh # same, via env var (for tooling that
#                                  # can't easily pass a positional arg)
#
# Exit code: 0 = PASS (every requested flight clean + handed off, audit
# (a)/(b)/(c) all pass, AND EVERY requested law's miss < MISS_GATE_M),
# non-zero = FAIL. Tiered-gate rationale (ADR-0010 decision #7, ADR-0013):
# a 6 m/s crosser is UNCATCHABLE from hover (ADR-0011 addendum, min range
# ~4.7 m) -- S2's gate proves the two-stage architecture + honesty at a
# threshold honest to its measured run variance (no-flag flights spanned
# 1.09-2.25 m); precision tightening is S3/M5 scope.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"
LOGS_DIR="$REPO_ROOT/logs"
READY_TIMEOUT_S=120
# NOTE: do NOT wait for "Ready for takeoff!" here -- see ADR-0004 /
# .claude/skills/px4-gazebo/SKILL.md. "Startup script returned successfully"
# is PX4's unconditional boot-complete line.
READY_STRING="Startup script returned successfully"
# Same reasoning as check_m3/m4/s1.sh: m4_intercept.py arms and enters
# OFFBOARD shortly after health-OK, so give the EKF a few seconds after
# boot-complete to settle before MAVSDK connects and starts issuing commands.
POST_READY_SETTLE_S=5
# S2 adds a CUE_WAIT + DASH phase before ENGAGE even starts, on top of an
# already-generous per-phase timeout budget (CUE_WAIT 15s, DASH 20s, ENGAGE
# 30s) -- give the outer wall-clock timeout real headroom over check_s1.sh's.
PY_TIMEOUT_S=360
# S2 tiered gate (ADR-0013): EVERY requested law's miss must be < this.
# 2.5 m at 6 m/s is deliberately looser than M4's 1.0 m at 2 m/s -- tiered
# thresholds by engagement class per ADR-0010 decision #7. Evidence base:
# five no-flag S2 flights on 2026-07-05 spanned 1.09-2.25 m (terminal
# camera-dropout timing is the variance driver); the architecture/honesty
# requirements (clean, handoff, audits) carry the real proof burden here.
MISS_GATE_M=2.5

# S2 long-standoff dash-runway geometry -- MUST match scripts/m4_intercept.py's
# S2["TARGET_START_DEFAULT"] / S2["TARGET_VEL_DEFAULT"] (mirrors
# guidance_lab.py's S2_PATH_PARAMS: x0=6.5, y0_mag=14.0 -> ~15.4 m initial
# range, "real dash runway", ADR-0011 third addendum). check_s1.sh has the
# same implicit-sync convention with m4_intercept.py's own M4/S1 default --
# these are NOT passed explicitly on the CLI below (m4_intercept.py already
# defaults to them under --handoff); they exist here ONLY so this script's
# own tag pre-placement command matches where the flight will actually start.
TARGET_START_X=6.5
TARGET_START_Y=-14
TARGET_START_Z=0.5

# worlds/apriltag.sdf is the checked-in source of truth (kept in this repo
# per the M2 task spec), but PX4's own px4-rc.gzsim resolves world files as
# ${PX4_GZ_WORLDS}/<name>.sdf, and gz_env.sh (sourced at runtime from the
# SITL working dir) unconditionally overwrites PX4_GZ_WORLDS to PX4's own
# Tools/simulation/gz/worlds directory -- that env var is NOT additive, so
# exporting it ourselves would just get clobbered. A symlink is the fix; see
# docs/decisions.md (ADR-0005) and worlds/apriltag.sdf.
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
WORLD_SYMLINK="$PX4_WORLDS_DIR/apriltag.sdf"
REPO_WORLD="$REPO_ROOT/worlds/apriltag.sdf"

SIM_PID=""

mkdir -p "$LOGS_DIR"

kill_stale_sim() {
    echo "[check_s2] Killing any stale px4 / gz / cue-mock / mover processes..."
    pkill -f "bin/px4" 2>/dev/null
    pkill -f "px4_sitl" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "gz-sim" 2>/dev/null
    pkill -f "s2_cue_mock.py" 2>/dev/null
    pkill -f "m4_target_mover.py" 2>/dev/null
    sleep 2
}

teardown_sim() {
    echo "[check_s2] Tearing down sim processes..."
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        # SIM_PID was launched via `setsid`, so it is its own process group
        # leader: -$SIM_PID targets that whole group (make + px4 + gz sim)
        # without touching this script's own process group.
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
    # gz sim's renderer can take a few seconds to unwind after SIGTERM
    # (observed during M1 development); give it a moment before returning.
    sleep 3
    SIM_PID=""
}
trap teardown_sim EXIT

is_number() {
    [[ "$1" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]]
}

# --- law selection: positional arg, or S2_LAW env var, or both laws ---
if [[ -n "${1:-}" ]]; then
    LAWS=("$1")
elif [[ -n "${S2_LAW:-}" ]]; then
    LAWS=("$S2_LAW")
else
    LAWS=(pip pronav)
fi
for LAW in "${LAWS[@]}"; do
    if [[ "$LAW" != "pip" && "$LAW" != "pronav" ]]; then
        echo "[check_s2] FAIL: unknown law '$LAW' (expected pip or pronav)"
        echo "FAIL"
        exit 1
    fi
done

kill_stale_sim

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[check_s2] FAIL: PX4-Autopilot not found at $PX4_DIR"
    echo "FAIL"
    exit 1
fi

if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "[check_s2] FAIL: venv python not found at $VENV_PYTHON (run the env setup first)"
    echo "FAIL"
    exit 1
fi

if [[ ! -f "$REPO_WORLD" ]]; then
    echo "[check_s2] FAIL: $REPO_WORLD not found"
    echo "FAIL"
    exit 1
fi

# Make sure the symlink into PX4's worlds dir exists and points at our repo
# copy (create/repair it rather than assuming a prior manual step held).
if [[ ! -e "$WORLD_SYMLINK" || "$(readlink -f "$WORLD_SYMLINK" 2>/dev/null)" != "$(readlink -f "$REPO_WORLD")" ]]; then
    echo "[check_s2] Linking $WORLD_SYMLINK -> $REPO_WORLD"
    ln -sf "$REPO_WORLD" "$WORLD_SYMLINK"
fi

# --- audit function: reads a flight CSV and checks (a) no ext_* activity
# at/after the first ENGAGE row, (b) at least one DASH row before it, and
# (c) commanded velocity azimuth tracks the camera-derived lambda_deg on
# ENGAGE rows with a detection (numeric no-cheat check, adapted from the
# project's existing camera-not-ground-truth audits -- ADR-0008/M3,
# ADR-0009/M4 -- to this two-stage handoff's specific claim: post-handoff,
# nothing but the camera drives the command). LAW-AWARE correlation bound
# (ADR-0013): pronav >= 0.7 (its lateral term is built directly off lambda);
# pip >= 0.55, because PIP's lead-point solve legitimately decorrelates the
# commanded azimuth from the raw LOS -- the cleanest flight of the dev
# session (m4_intercept_pip_20260705T174315Z.csv, miss 1.087 m, structurally
# honest on audits (a)/(b)) measured corr 0.689 and would fail a flat 0.7.
audit_csv() {
    local csv_path="$1"
    local law="$2"
    "$VENV_PYTHON" - "$csv_path" "$law" <<'PYEOF'
import csv
import math
import sys

path = sys.argv[1]
with open(path, newline="") as f:
    rows = list(csv.DictReader(f))

ok = True

engage_idx = None
for i, r in enumerate(rows):
    if r["phase"] == "ENGAGE":
        engage_idx = i
        break

if engage_idx is None:
    print("[audit] FAIL: no ENGAGE phase rows found in", path)
    sys.exit(1)

# (a) zero non-empty ext_* cells from the first ENGAGE row onward (ENGAGE +
# BREAKOFF both -- the cue channel must stay latched closed forever once
# HANDOFF triggers, ADR-0010 #5).
bad_rows = [
    i for i, r in enumerate(rows[engage_idx:], start=engage_idx)
    if r["ext_x"] or r["ext_y"] or r["ext_z"] or r["ext_fresh"] not in ("", "0")
]
if bad_rows:
    print(
        f"[audit] FAIL (a): {len(bad_rows)} rows at/after first ENGAGE "
        f"(row {engage_idx}) have non-empty ext_* -- cue used post-handoff "
        f"(first offender: row {bad_rows[0]})"
    )
    ok = False
else:
    print(
        f"[audit] PASS (a): zero non-empty ext_* cells across "
        f"{len(rows) - engage_idx} rows at/after first ENGAGE (row {engage_idx})"
    )

# (b) at least one DASH row precedes the first ENGAGE row -- proves the
# running-start phase actually ran, not just a degenerate instant handoff.
n_dash_before = sum(1 for r in rows[:engage_idx] if r["phase"] == "DASH")
if n_dash_before < 1:
    print(
        f"[audit] FAIL (b): {n_dash_before} DASH rows precede the first "
        f"ENGAGE row (row {engage_idx}); expected >= 1"
    )
    ok = False
else:
    print(
        f"[audit] PASS (b): {n_dash_before} DASH rows precede the first "
        f"ENGAGE row (row {engage_idx})"
    )

# (c) post-handoff no-cheat: on ENGAGE rows with a camera detection, does
# the COMMANDED horizontal velocity azimuth track the CAMERA-derived
# lambda_deg (own yaw + measured bearing -- never ground truth, never the
# external cue)? Correlation, not exact match -- PIP's lead-point solve
# legitimately points ahead of the raw LOS.
xs, ys = [], []
for r in rows[engage_idx:]:
    if r["phase"] != "ENGAGE":
        continue
    if r["detected"] != "1" or not r["lambda_deg"] or not r["cmd_vn"] or not r["cmd_ve"]:
        continue
    lam = float(r["lambda_deg"])
    vn = float(r["cmd_vn"])
    ve = float(r["cmd_ve"])
    if abs(vn) < 1e-6 and abs(ve) < 1e-6:
        continue
    az = math.degrees(math.atan2(ve, vn))
    # Normalize away atan2's +-180 branch cut relative to lambda_deg (a pure
    # +-360 artifact, not a real angular difference) -- see check_s2.sh
    # comment.
    diff = az - lam
    if diff > 180.0:
        az -= 360.0
    elif diff < -180.0:
        az += 360.0
    xs.append(lam)
    ys.append(az)

n = len(xs)
if n < 5:
    print(f"[audit] WARN (c): only {n} ENGAGE rows with a camera detection -- skipping correlation check")
else:
    mx = sum(xs) / n
    my = sum(ys) / n
    cov = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    vx = sum((v - mx) ** 2 for v in xs)
    vy = sum((v - my) ** 2 for v in ys)
    # Law-aware bound -- see the comment above audit_csv().
    law = sys.argv[2] if len(sys.argv) > 2 else ""
    bound = 0.55 if law == "pip" else 0.7
    if vx < 1e-6 or vy < 1e-6:
        print(f"[audit] WARN (c): degenerate variance (vx={vx:.4f}, vy={vy:.4f}) over {n} rows -- skipping correlation check")
    else:
        corr = cov / math.sqrt(vx * vy)
        if corr < bound:
            print(f"[audit] FAIL (c): cmd-velocity-azimuth vs camera lambda_deg correlation {corr:.3f} < {bound} over {n} rows")
            ok = False
        else:
            print(f"[audit] PASS (c): cmd-velocity-azimuth vs camera lambda_deg correlation {corr:.3f} >= {bound} over {n} rows")

sys.exit(0 if ok else 1)
PYEOF
}

declare -A MISS
declare -A CLEAN
declare -A HANDOFF
declare -A EXITCODE
declare -A AUDITCODE
declare -A LOGPATH

for LAW in "${LAWS[@]}"; do
    echo ""
    echo "[check_s2] ================ Law: $LAW ================"

    kill_stale_sim

    TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
    SIM_LOG="$LOGS_DIR/check_s2_sim_${LAW}_${TIMESTAMP}.log"
    RUN_LOG="$LOGS_DIR/check_s2_run_${LAW}_${TIMESTAMP}.log"

    echo "[check_s2] Launching HEADLESS PX4 SITL + Gazebo (gz_x500_mono_cam, world=apriltag) for law=$LAW..."
    echo "[check_s2] Sim output -> $SIM_LOG"

    # stdin must stay open: PX4's interactive pxh console spins on EOF and
    # floods the log with prompt reprints. `tail -f /dev/null` never EOFs,
    # so pxh blocks quietly; it dies with the group on teardown. (Same
    # pattern as check_m0/m1/m2/m3/s1.sh.)
    setsid bash -c "cd '$PX4_DIR' && tail -f /dev/null | env PX4_GZ_WORLD=apriltag GZ_SIM_RESOURCE_PATH='$REPO_ROOT/models' HEADLESS=1 make px4_sitl gz_x500_mono_cam" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    echo "[check_s2] Sim launched (pid/pgid $SIM_PID) for law=$LAW."

    echo "[check_s2] Waiting up to ${READY_TIMEOUT_S}s for \"$READY_STRING\" in sim log..."
    ready=0
    elapsed=0
    while (( elapsed < READY_TIMEOUT_S )); do
        if grep -q "$READY_STRING" "$SIM_LOG" 2>/dev/null; then
            ready=1
            break
        fi
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "[check_s2] FAIL: sim process exited early (pid $SIM_PID) for law=$LAW. See $SIM_LOG"
            echo "FAIL"
            exit 1
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    if [[ "$ready" -ne 1 ]]; then
        echo "[check_s2] FAIL: did not see \"$READY_STRING\" within ${READY_TIMEOUT_S}s for law=$LAW. See $SIM_LOG"
        echo "FAIL"
        exit 1
    fi

    echo "[check_s2] Sim ready. Sleeping ${POST_READY_SETTLE_S}s to let the EKF settle before MAVSDK connects..."
    sleep "$POST_READY_SETTLE_S"

    echo "[check_s2] Pre-placing the tag at the S2 dash-start position ($TARGET_START_X, $TARGET_START_Y, $TARGET_START_Z)..."
    placed=0
    for attempt in 1 2 3; do
        if gz service -s /world/apriltag/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
            --req "name: \"apriltag_target\" position { x: $TARGET_START_X y: $TARGET_START_Y z: $TARGET_START_Z }"; then
            placed=1
            break
        fi
        echo "[check_s2] WARNING: tag pre-placement attempt $attempt failed, retrying..."
        sleep 1
    done
    if [[ "$placed" -ne 1 ]]; then
        echo "[check_s2] FAIL: could not pre-place the tag after 3 attempts for law=$LAW"
        echo "FAIL"
        exit 1
    fi
    echo "[check_s2] Tag pre-placed."

    # EXTRA_ARGS (env var, opt-in, empty by default -- zero effect on the
    # documented gate invocation): lets dev iteration A/B extra flags (e.g.
    # tracking-refinement's --kalata/--cue-latency-comp) through the same
    # validated boot/preplace/teardown machinery without hardcoding them
    # into the gate itself. Word-split intentionally (not quoted) so a
    # space-separated flag list works as expected.
    echo "[check_s2] Running m4_intercept.py --fpv --handoff --law $LAW ${EXTRA_ARGS:-} (outer timeout ${PY_TIMEOUT_S}s)..."
    echo "[check_s2] Run output -> $RUN_LOG (also streamed below)"
    # shellcheck disable=SC2086
    timeout "$PY_TIMEOUT_S" "$VENV_PYTHON" "$REPO_ROOT/scripts/m4_intercept.py" --fpv --handoff --law "$LAW" ${EXTRA_ARGS:-} 2>&1 | tee "$RUN_LOG"
    PY_EXIT="${PIPESTATUS[0]}"
    EXITCODE[$LAW]="$PY_EXIT"

    if [[ "$PY_EXIT" -eq 124 ]]; then
        echo "[check_s2] FAIL: m4_intercept.py (law=$LAW) hit the outer ${PY_TIMEOUT_S}s timeout. See $SIM_LOG and $RUN_LOG"
    fi

    RESULT_LINE="$(grep '^S2_RESULT ' "$RUN_LOG" | tail -n1)"
    if [[ -z "$RESULT_LINE" ]]; then
        echo "[check_s2] FAIL: no S2_RESULT line found in output for law=$LAW"
        MISS[$LAW]=""
        CLEAN[$LAW]=""
        HANDOFF[$LAW]=""
    else
        MISS[$LAW]="$(echo "$RESULT_LINE" | grep -oP '(?<=miss=)[^ ]+')"
        CLEAN[$LAW]="$(echo "$RESULT_LINE" | grep -oP '(?<=clean=)[^ ]+')"
        HANDOFF[$LAW]="$(echo "$RESULT_LINE" | grep -oP '(?<= handoff=)[^ ]+')"
        echo "[check_s2] Parsed: law=$LAW miss=${MISS[$LAW]} clean=${CLEAN[$LAW]} handoff=${HANDOFF[$LAW]} (exit $PY_EXIT)"
    fi

    CSV_PATH="$(grep -oP '(?<=\[m4\] Logging to )\S+\.csv' "$RUN_LOG" | tail -n1)"
    LOGPATH[$LAW]="$CSV_PATH"
    if [[ -z "$CSV_PATH" || ! -f "$CSV_PATH" ]]; then
        echo "[check_s2] FAIL: could not find this flight's CSV log path in $RUN_LOG"
        AUDITCODE[$LAW]=1
    else
        echo "[check_s2] Auditing $CSV_PATH ..."
        if audit_csv "$CSV_PATH" "$LAW"; then
            AUDITCODE[$LAW]=0
        else
            AUDITCODE[$LAW]=1
        fi
    fi

    teardown_sim
done

echo ""
echo "[check_s2] ================ Comparison ================"
for LAW in "${LAWS[@]}"; do
    echo "[check_s2] $LAW: miss=${MISS[$LAW]:-<missing>} m  clean=${CLEAN[$LAW]:-<missing>}  handoff=${HANDOFF[$LAW]:-<missing>}  audit_exit=${AUDITCODE[$LAW]:-<missing>}  exit=${EXITCODE[$LAW]:-<missing>}  log=${LOGPATH[$LAW]:-<missing>}"
done

PASS=1
BEST_MISS=""

for LAW in "${LAWS[@]}"; do
    if [[ "${EXITCODE[$LAW]:-1}" -eq 124 ]]; then
        PASS=0
    fi
    if [[ "${HANDOFF[$LAW]:-}" != "1" ]]; then
        echo "[check_s2] FAIL reason: $LAW never reached HANDOFF (handoff=${HANDOFF[$LAW]:-<missing>})."
        PASS=0
    fi
    if [[ "${CLEAN[$LAW]:-}" != "1" ]]; then
        echo "[check_s2] FAIL reason: $LAW flight not clean (clean=${CLEAN[$LAW]:-<missing>})."
        PASS=0
    fi
    if [[ "${AUDITCODE[$LAW]:-1}" -ne 0 ]]; then
        echo "[check_s2] FAIL reason: $LAW failed the CSV audit (a)/(b)/(c) -- see audit output above."
        PASS=0
    fi
    if ! is_number "${MISS[$LAW]:-}"; then
        echo "[check_s2] FAIL reason: $LAW miss distance not a valid number (${MISS[$LAW]:-<missing>})."
        PASS=0
    else
        # Per-law tiered gate (ADR-0013): EVERY requested law must beat it,
        # not just the best one -- a best-of gate would let one lucky law
        # mask the other's regression.
        if ! awk -v m="${MISS[$LAW]}" -v g="$MISS_GATE_M" 'BEGIN{exit !(m < g)}'; then
            echo "[check_s2] FAIL reason: $LAW miss ${MISS[$LAW]} m >= ${MISS_GATE_M} m gate."
            PASS=0
        else
            echo "[check_s2] $LAW miss ${MISS[$LAW]} m < ${MISS_GATE_M} m gate: OK"
        fi
        if [[ -z "$BEST_MISS" ]] || awk -v a="${MISS[$LAW]}" -v b="$BEST_MISS" 'BEGIN{exit !(a < b)}'; then
            BEST_MISS="${MISS[$LAW]}"
        fi
    fi
done

echo "[check_s2] Best miss across requested flights: ${BEST_MISS:-<none>} m (per-law gate < ${MISS_GATE_M} m)"

if [[ "$PASS" -eq 1 ]]; then
    echo "[check_s2] PASS"
    exit 0
else
    echo "[check_s2] FAIL"
    exit 1
fi
