#!/usr/bin/env bash
# Render the placard-mount parts to STL.
#
#   bash scripts/print_artifacts/render_stls.sh            # -> hardware/prints/
#   bash scripts/print_artifacts/render_stls.sh --web-t 3.5 --suffix _web35
#
# STATUS ON THIS MACHINE (2026-07-25): openscad is NOT INSTALLED, so NO STL HAS
# BEEN PRODUCED and none is committed. The .scad sources are the deliverable.
# Installing openscad is an apt change outside the project directory, which
# CLAUDE.md says to ASK the builder for first -- so it was not done.
#
#   sudo apt install openscad          # ~120 MB with deps, WSL2 headless is fine
#   # or, no install at all:
#   #   Windows: https://openscad.org/downloads.html, open the .scad, F6, export STL
#   #   web:     https://openscad.cloud/openscad/  (drag the whole directory in --
#   #            it needs placard_mount_params.scad alongside the part files)
#
# After rendering, LOOK AT EACH PART before printing. The static checker
# (scripts/print_artifacts/check_scad.py) proves identifiers resolve and the
# derived dimensions close; it cannot prove the CSG is manifold or that the
# part looks like docs/placard_mount.md 6. Only your eyes on the render can.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="$REPO/hardware/prints"
SUFFIX=""
DEFINES=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --suffix) SUFFIX="$2"; shift 2 ;;
    --out)    OUT="$2"; shift 2 ;;
    --*)      DEFINES+=("-D" "${1#--}=$2"); shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if ! command -v openscad >/dev/null 2>&1; then
  cat >&2 <<'EOF'
[render_stls] openscad is NOT installed -- nothing rendered, nothing written.
[render_stls] This is expected on this machine (see the header of this script).
[render_stls] The .scad sources in scripts/print_artifacts/ are the deliverable;
[render_stls] render them on a machine that has OpenSCAD, or install it with
[render_stls]     sudo apt install openscad
[render_stls] then re-run this script. NO STL IS FABRICATED BY ANY OTHER MEANS.
EOF
  exit 3
fi

mkdir -p "$OUT"
# Run the static check first -- a typo'd parameter renders a silently wrong part.
python3 "$HERE/check_scad.py"

for part in placard_index_disc placard_shoe placard_spar_cap; do
  stl="$OUT/${part}${SUFFIX}.stl"
  echo "[render_stls] $part -> $stl"
  openscad -o "$stl" "${DEFINES[@]+"${DEFINES[@]}"}" "$HERE/${part}.scad"
done

echo "[render_stls] done. Print counts (docs/placard_mount.md 7 L741-744):"
echo "[render_stls]   placard_index_disc  x1"
echo "[render_stls]   placard_shoe        x4   (1 flight + 3 spares -- it is the crash part)"
echo "[render_stls]   placard_spar_cap    x4"
echo "[render_stls] PETG. Read the FRANGIBILITY WARNING in placard_mount_params.scad"
echo "[render_stls] before you print all four shoes at the same web thickness."
