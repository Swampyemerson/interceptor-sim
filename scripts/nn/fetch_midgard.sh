#!/usr/bin/env bash
# Fetch MIDGARD (CTU MRS group; real air-to-air footage of open-frame research quads, 752x480,
# auto-labelled boxes, about 3.5 GB) one sequence folder at a time into data/midgard/.
# The dataset page links nasmrs.felk.cvut.cz, whose certificate is only valid for
# nasmrs.fel.cvut.cz -- use the latter (checked 2026-11-13); never disable verification.
# Cite: Walter, Vrba, Saska, ICRA 2020. No licence is stated on the page: research use, do not
# redistribute the frames.  Usage: setsid nohup bash scripts/nn/fetch_midgard.sh > logs/nn_tier/fetch_midgard.log 2>&1 &
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; OUT="$REPO/data/midgard"; mkdir -p "$OUT"; cd "$OUT"
H=https://nasmrs.fel.cvut.cz; T=YKF6vzsDxiDLc1m; fail=0
for item in "countryside-natural north-narrow" "countryside-natural south-narrow" "semi-urban" "urban" "outdoor-historical" \
            "countryside-natural north-fisheye" "countryside-natural south-fisheye" "indoor-modern" "indoor-historical"; do
    set -- $item; name="${1}${2:+_$2}"
    [ -d "$name" ] && { echo "have $name"; continue; }
    if [ -n "${2:-}" ]; then url="$H/index.php/s/$T/download?path=%2F$1&files=$2"; else url="$H/index.php/s/$T/download?path=%2F&files=$1"; fi
    if curl -sS -m 7200 -L -o "$name.zip" "$url" && python3 -I -c "
import sys, zipfile
z = zipfile.ZipFile(sys.argv[1]); bad = [n for n in z.namelist() if n.startswith('/') or '..' in n.split('/')]
sys.exit('unsafe paths in archive') if bad else z.extractall(sys.argv[2]); print(sys.argv[2], len(z.namelist()), 'entries')" "$name.zip" "$name"; then rm -f "$name.zip"
    else echo "FAILED $name"; fail=1; fi
done
du -sh "$OUT" | cut -f1; [ $fail = 0 ] && echo MIDGARD_FETCH_DONE || echo MIDGARD_FETCH_INCOMPLETE
