#!/usr/bin/env bash
# scripts/seeker/real_data_pipeline_run.sh -- END-TO-END real-hardware seeker
# pipeline orchestrator (docs/real_data_pipeline.md). Chains EXISTING pieces:
#
#   flight frames --[autolabel_from_apriltag.py]--> YOLO labels
#                 --[split_real_dataset.py]--> flight-level train/val dataset
#                 --[train_real_data.py]--> COCO-init fine-tuned detector
#                 --[resolution_probe.py]--> recall-vs-range plumbing check
#
# so the tripod-capture afternoon is turnkey: one command per flight, then one
# command to train and check.
#
# REAL USE (after a tripod/flight capture session -- each --flight-frames is
# one recorded session's raw PNG/JPG frames; --val-flight NAME must match the
# basename of one of the --flight-frames dirs and is held out WHOLE for val):
#
#   scripts/seeker/real_data_pipeline_run.sh \
#     --flight-frames captures/session1 --flight-frames captures/session2 \
#     --val-flight session2 \
#     --neg-frames captures/prelaunch_empty \
#     --calib captures/calib.json --tag-size 0.10 --drone-size 0.35 \
#     --name drone_real_v1 --out-dir datasets/real_flight_01 --epochs 40
#
# DRY RUN (sim-frame plumbing rehearsal -- no tripod footage needed, proves
# paths/formats/label round-trip on synthetic composited-tag frames pulled
# from scripts/seeker/data/* under the sim's own camera intrinsics):
#
#   scripts/seeker/real_data_pipeline_run.sh --dry-run
#
# Requires .venv-seeker (cv2, pupil-apriltags) and .venv-seeker-train
# (ultralytics/torch, CPU-only in this repo).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
cd "$REPO"

SEEKER_PY="$REPO/.venv-seeker/bin/python"
TRAIN_PY="$REPO/.venv-seeker-train/bin/python"
[ -x "$SEEKER_PY" ] || { echo "[real-data-pipeline] FAIL: missing $SEEKER_PY (.venv-seeker)"; exit 1; }
[ -x "$TRAIN_PY" ]  || { echo "[real-data-pipeline] FAIL: missing $TRAIN_PY (.venv-seeker-train)"; exit 1; }

DRY_RUN=0
FLIGHT_FRAMES=()
VAL_FLIGHTS=()
NEG_FRAMES=()
CALIB=""
TAG_SIZE=0.10
DRONE_SIZE=0.35
NAME="drone_real_v1"
OUT_DIR=""
EPOCHS=""
MAX_NEG_FRAC=0.18
KEEP=0

usage() {
  sed -n '2,33p' "$0" | sed 's/^# \{0,1\}//'
  exit 1
}

while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY_RUN=1; shift ;;
    --flight-frames) FLIGHT_FRAMES+=("$2"); shift 2 ;;
    --val-flight) VAL_FLIGHTS+=("$2"); shift 2 ;;
    --neg-frames) NEG_FRAMES+=("$2"); shift 2 ;;
    --calib) CALIB="$2"; shift 2 ;;
    --tag-size) TAG_SIZE="$2"; shift 2 ;;
    --drone-size) DRONE_SIZE="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --epochs) EPOCHS="$2"; shift 2 ;;
    --max-neg-frac) MAX_NEG_FRAC="$2"; shift 2 ;;
    --keep) KEEP=1; shift ;;
    -h|--help) usage ;;
    *) echo "[real-data-pipeline] unknown arg: $1"; usage ;;
  esac
done

fail() { echo "[real-data-pipeline] FAIL: $*" >&2; exit 1; }

if [ "$DRY_RUN" = "1" ]; then
  echo "=== real_data_pipeline_run.sh --dry-run (sim-frame plumbing rehearsal) ==="
  SCRATCH="$(mktemp -d /tmp/rdp_dryrun.XXXXXX)"
  trap 'rm -rf "$SCRATCH"' EXIT
  NAME="drone_real_dryrun"
  OUT_DIR="$SCRATCH/dataset"
  EPOCHS="${EPOCHS:-1}"
  DRYRUN_NEG_FRAC="${MAX_NEG_FRAC}"
  [ "$DRYRUN_NEG_FRAC" = "0.18" ] && DRYRUN_NEG_FRAC=0.2

  echo "--- [0/7] self-tests (no data needed): synth, autolabel, split ---"
  "$SEEKER_PY" "$HERE/synth_tag_frames.py" --self-test || fail "synth_tag_frames self-test"
  "$SEEKER_PY" "$HERE/autolabel_from_apriltag.py" --self-test || fail "autolabel self-test"
  "$SEEKER_PY" "$HERE/split_real_dataset.py" --self-test || fail "split self-test"
  "$TRAIN_PY" "$HERE/train_real_data.py" --dry-run || fail "train_real_data dry-run wiring"

  echo "--- [1/7] generating synthetic tag flights from existing sim frames ---"
  "$SEEKER_PY" "$HERE/synth_tag_frames.py" --out "$SCRATCH/raw_flightA" --run-tag flightA --seed 1 \
      || fail "synth flightA"
  "$SEEKER_PY" "$HERE/synth_tag_frames.py" --out "$SCRATCH/raw_flightB" --run-tag flightB --seed 2 \
      || fail "synth flightB"

  echo "--- [2/7] autolabeling synthetic flights (tag pose -> YOLO labels) ---"
  "$SEEKER_PY" "$HERE/autolabel_from_apriltag.py" \
      --frames "$SCRATCH/raw_flightA/frames" --calib "$SCRATCH/raw_flightA/calib.json" \
      --tag-size 0.5 --drone-size "$DRONE_SIZE" --out "$SCRATCH/auto_flightA" || fail "autolabel flightA"
  "$SEEKER_PY" "$HERE/autolabel_from_apriltag.py" \
      --frames "$SCRATCH/raw_flightB/frames" --calib "$SCRATCH/raw_flightB/calib.json" \
      --tag-size 0.5 --drone-size "$DRONE_SIZE" --out "$SCRATCH/auto_flightB" || fail "autolabel flightB"
  "$SEEKER_PY" "$HERE/autolabel_from_apriltag.py" \
      --frames "$SCRATCH/raw_flightA/frames_negfree" --calib "$SCRATCH/raw_flightA/calib.json" \
      --negatives-from-untagged --tag-size 0.5 --drone-size "$DRONE_SIZE" \
      --out "$SCRATCH/auto_negatives" || fail "autolabel negatives"

  N_A=$(find "$SCRATCH/auto_flightA/images" -name '*.png' | wc -l)
  N_B=$(find "$SCRATCH/auto_flightB/images" -name '*.png' | wc -l)
  [ "$N_A" -gt 0 ] || fail "flightA produced zero labeled frames -- tag decode/label round-trip broke"
  [ "$N_B" -gt 0 ] || fail "flightB produced zero labeled frames -- tag decode/label round-trip broke"
  echo "[dry-run] flightA=$N_A labeled frames, flightB=$N_B labeled frames"

  echo "--- [3/7] flight-level split (flightB held out whole for val) ---"
  "$SEEKER_PY" "$HERE/split_real_dataset.py" \
      --flight "$SCRATCH/auto_flightA" --flight "$SCRATCH/auto_flightB" --val-flight auto_flightB \
      --neg "$SCRATCH/auto_negatives" --max-neg-frac "$DRYRUN_NEG_FRAC" --out "$OUT_DIR" || fail "split"
  [ -f "$OUT_DIR/data.yaml" ] || fail "data.yaml not written"

  echo "--- [4/7] tiny fine-tune ($EPOCHS epoch, COCO-init, .venv-seeker-train) ---"
  "$TRAIN_PY" "$HERE/train_real_data.py" --data "$OUT_DIR/data.yaml" --name "$NAME" --epochs "$EPOCHS" \
      || fail "tiny fine-tune"
  [ -f "$HERE/weights/$NAME.pt" ]   || fail "no $HERE/weights/$NAME.pt exported"
  [ -f "$HERE/weights/$NAME.onnx" ] || fail "no $HERE/weights/$NAME.onnx exported"

  echo "--- [5/7] recall-vs-range plumbing check (resolution_probe.py) ---"
  "$SEEKER_PY" "$HERE/resolution_probe.py" "$SCRATCH/auto_flightA" --rmin 3 --rmax 10 || fail "resolution_probe"

  echo "--- [6/7] label round-trip spot-check (valid YOLO line, class 0, values in [0,1]) ---"
  LBL="$(find "$OUT_DIR/labels/train" -name '*.txt' -size +0c | head -1)"
  [ -n "$LBL" ] || fail "no non-empty label file found to spot-check"
  "$SEEKER_PY" - "$LBL" <<'PYEOF' || fail "label round-trip spot-check"
import sys
parts = open(sys.argv[1]).readline().split()
assert len(parts) == 5, f"expected 5 fields, got {len(parts)}"
cls, cx, cy, w, h = int(parts[0]), *map(float, parts[1:])
assert cls == 0
for v in (cx, cy, w, h):
    assert 0.0 <= v <= 1.0, f"value {v} out of [0,1]"
print(f"[dry-run] label OK: class={cls} cx={cx:.3f} cy={cy:.3f} w={w:.3f} h={h:.3f}")
PYEOF

  echo "--- [7/7] cleanup: removing dry-run artifacts from tracked dirs ---"
  rm -f "$HERE/weights/$NAME.pt" "$HERE/weights/$NAME.onnx"
  rm -rf "$REPO/runs/detect/$NAME"

  echo ""
  echo "=== DRY-RUN PASS: frames -> autolabel -> split -> train -> recall-vs-range wired correctly ==="
  echo "NOTE: this rehearsal proves PLUMBING only (paths/formats/label round-trip) on"
  echo "synthetic composited-tag frames + the SIM camera intrinsics. It is NOT an"
  echo "accuracy claim -- see this script's docstring / open_issues for real gaps"
  echo "(resolution_probe.py's hardcoded sim intrinsics + fixed v2/crop weight paths;"
  echo "approach_recall.py's mc_batch-CSV-only input) before trusting numbers on real"
  echo "tripod footage."
  exit 0
fi

# ---------------------------- REAL RUN ----------------------------
[ ${#FLIGHT_FRAMES[@]} -gt 0 ] || fail "need >=1 --flight-frames DIR (or --dry-run); see --help"
[ -n "$CALIB" ] || fail "need --calib calib.json (or --dry-run)"
[ -n "$OUT_DIR" ] || fail "need --out-dir (or --dry-run)"
[ ${#VAL_FLIGHTS[@]} -gt 0 ] || fail "need >=1 --val-flight NAME matching a --flight-frames " \
    "dir's basename -- an empty val split breaks training and hides the real val floor"
EPOCHS="${EPOCHS:-40}"

WORK="$(mktemp -d /tmp/rdp_work.XXXXXX)"
[ "$KEEP" = "1" ] || trap 'rm -rf "$WORK"' EXIT

echo "=== real_data_pipeline_run.sh: $NAME ==="
FLIGHT_ARGS=()
for f in "${FLIGHT_FRAMES[@]}"; do
  base="$(basename "$f")"
  auto_out="$WORK/auto_${base}"
  echo "--- autolabeling $f -> $auto_out ---"
  "$SEEKER_PY" "$HERE/autolabel_from_apriltag.py" --frames "$f" --calib "$CALIB" \
      --tag-size "$TAG_SIZE" --drone-size "$DRONE_SIZE" --out "$auto_out" || fail "autolabel $f"
  FLIGHT_ARGS+=(--flight "$auto_out")
done

NEG_ARGS=()
for n in "${NEG_FRAMES[@]}"; do
  base="$(basename "$n")"
  auto_out="$WORK/auto_neg_${base}"
  echo "--- autolabeling negatives $n -> $auto_out ---"
  "$SEEKER_PY" "$HERE/autolabel_from_apriltag.py" --frames "$n" --calib "$CALIB" \
      --negatives-from-untagged --tag-size "$TAG_SIZE" --drone-size "$DRONE_SIZE" \
      --out "$auto_out" || fail "autolabel neg $n"
  NEG_ARGS+=(--neg "$auto_out")
done

VAL_ARGS=()
for v in "${VAL_FLIGHTS[@]}"; do
  VAL_ARGS+=(--val-flight "auto_${v}")
done

echo "--- splitting (flight-level, held-out val) -> $OUT_DIR ---"
"$SEEKER_PY" "$HERE/split_real_dataset.py" "${FLIGHT_ARGS[@]}" "${VAL_ARGS[@]}" "${NEG_ARGS[@]}" \
    --max-neg-frac "$MAX_NEG_FRAC" --out "$OUT_DIR" || fail "split"

echo "--- fine-tuning ($EPOCHS epochs, COCO-init) ---"
"$TRAIN_PY" "$HERE/train_real_data.py" --data "$OUT_DIR/data.yaml" --name "$NAME" --epochs "$EPOCHS" \
    || fail "train"

echo "--- recall-vs-range (resolution_probe.py -- see open_issues: hardcoded sim intrinsics" \
     "+ fixed weight paths, needs a --weights/--calib flag before this is a real-hardware-" \
     "accurate number) ---"
FIRST_AUTO="$WORK/auto_$(basename "${FLIGHT_FRAMES[0]}")"
"$SEEKER_PY" "$HERE/resolution_probe.py" "$FIRST_AUTO" --rmin 8 --rmax 30 \
    || echo "[real-data-pipeline] resolution_probe.py exited non-zero (non-fatal, see note above)"

echo ""
echo "=== DONE: $OUT_DIR/data.yaml, $HERE/weights/$NAME.{pt,onnx} ==="
echo "NEXT: eval on held-out real flights, gate on approach-recall-vs-range (ADR-0076 add #18h)"
echo "+ FP/hour, shadow-mode before it steers (train_real_data.py's own reminder)."
