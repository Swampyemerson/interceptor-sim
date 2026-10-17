#!/usr/bin/env bash
# =============================================================================
# check_sysml_official.sh -- validate docs/sysml/*.sysml with the OMG SysML v2
# REFERENCE implementation (the "Pilot Implementation", the same parser that
# backs the Jupyter SysML kernel). Exit 0 = every file parsed and resolved with
# zero errors; exit 1 = errors (printed with file:line); exit 2 = could not run.
#
# Why this exists next to scripts/render_sysml.py: the renderer reads only a
# documented SUBSET and adds checks the reference tool does not make (port types
# and wire direction, build-sheet coverage). This script is the other half --
# proof that the text is real SysML v2, not a look-alike dialect.
#
# NOT in run_tests.sh on purpose: it needs Java 17+ and a one-time ~124 MB
# download (conda-forge jupyter-sysml-kernel, cached under ~/.cache). A check
# that cannot run is a FAILURE here (exit 2), never a silent pass.
#
# Usage:  scripts/sysml/check_sysml_official.sh
#         SYSML_KERNEL_DIR=/path/to/share/jupyter/kernels/sysml scripts/sysml/check_sysml_official.sh
# =============================================================================
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="$(pwd)"
VERSION="0.62.0"
PKG="jupyter-sysml-kernel-${VERSION}-pyhd8ed1ab_0.conda"
CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/interceptor-sim/sysml-${VERSION}"
KDIR="${SYSML_KERNEL_DIR:-$CACHE/pkg/share/jupyter/kernels/sysml}"

command -v java >/dev/null || { echo "CANNOT RUN: java not found (need JDK 17+)"; exit 2; }

if [ ! -f "$KDIR/jupyter-sysml-kernel-${VERSION}-all.jar" ]; then
    echo "Fetching the SysML v2 reference implementation ${VERSION} (one time, ~124 MB) ..."
    mkdir -p "$CACHE" && cd "$CACHE" || exit 2
    curl -fsSL -o "$PKG" "https://conda.anaconda.org/conda-forge/noarch/$PKG" \
        || { echo "CANNOT RUN: download failed"; exit 2; }
    python3 - "$PKG" <<'PY' || { echo "CANNOT RUN: unpack failed (pip install zstandard)"; exit 2; }
import io, sys, tarfile, zipfile
import zstandard
z = zipfile.ZipFile(sys.argv[1])
inner = [n for n in z.namelist() if n.startswith("pkg-") and n.endswith(".tar.zst")][0]
with z.open(inner) as f, zstandard.ZstdDecompressor().stream_reader(f) as r:
    tarfile.open(fileobj=r, mode="r|").extractall("pkg")
PY
    cd "$ROOT" || exit 2
fi

# Same order as MODEL_FILES in scripts/render_sysml.py (libraries before users).
FILES=$(python3 -c "import sys; sys.path.insert(0,'scripts'); import render_sysml as r; \
print(' '.join(str(r.MODEL_DIR / f) for f in r.MODEL_FILES))")

out="$(java -cp "$KDIR/*" "$ROOT/scripts/sysml/ValidateSysML.java" "$KDIR/sysml.library/" $FILES 2>&1)"
rc=$?
echo "$out" | grep -v '^Reading \|^Picked up JAVA_TOOL_OPTIONS\|^log4j:'
if [ $rc -eq 0 ]; then
    echo "OK: all $(echo $FILES | wc -w) SysML v2 files valid under the OMG reference implementation ${VERSION}"
else
    echo "FAIL: the reference implementation reported errors (above)"
fi
exit $rc
