"""Extra numbers quoted in the spec's result section (reads rows, prints)."""
import collections
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "scripts"))
from flight_hardware_rerun import go_time  # noqa: E402


def rows(stage):
    p = os.path.join(HERE, f"{stage}.json")
    return json.load(open(p)) if os.path.exists(p) else []


def late(rs, s):
    return sum(1 for r in rs if r["miss"] <= 0.35 and r["t_cpa"] - go_time(r) > s)


for st in ("a", "c", "brackets", "b", "btilt", "grid", "gridb", "lost", "brake20", "search"):
    rs = rows(st)
    if not rs:
        continue
    k = [r for r in rs if r["miss"] <= 0.35]
    tk = [r["t_cpa"] - go_time(r) for r in k]
    print(f"{st:9s}: {len(rs)} runs, {len(k)} kills; time from GO to a kill median {np.median(tk):.1f} s, "
          f"p90 {np.percentile(tk, 90):.1f}, max {max(tk):.1f}; later than 30/45/60 s: "
          f"{late(rs, 30)}/{late(rs, 45)}/{late(rs, 60)}")

b = rows("b")
if b:
    det = [r for r in b if r["first_det"] is not None and r["tag"].endswith("T2")]
    print("stage b, clock 2 s: first-decode range median %.1f m (p10 %.1f, p90 %.1f); speed %.1f m/s; lean %.1f deg" % (
        np.median([r["range_first"] for r in det]), np.percentile([r["range_first"] for r in det], 10),
        np.percentile([r["range_first"] for r in det], 90), np.median([r["v_first"] for r in det]),
        np.median([r["tilt_first"] for r in det])))
    # no-decode side: along-track sign at closest approach (same caveat as the geometry note)
    for err in ("e10", "e4", "e2", "e1", "e0"):
        nod = [r for r in b if r["first_det"] is None and r["tag"].startswith("st|") and f"|{err}|" in r["tag"] and r["tag"].endswith("T2")]
        if nod:
            al = np.array([r["miss_along"] for r in nod])
            print(f"  straight {err}: never decoded {len(nod)} of 100; target behind at closest approach in {int((al < 0).sum())}; "
                  f"closest approach median {np.median([r['miss'] for r in nod]):.1f} m")
    # worst per-cell loss of the 6 s clock
    cells = collections.defaultdict(dict)
    for r in b:
        cells[r["tag"]][r["seed"]] = r
    worst = 0
    for t, c in cells.items():
        if t.endswith("T2"):
            c6 = cells[t[:-2] + "T6"]
            worst = max(worst, sum(1 for s in c if c[s]["miss"] <= 0.35 and c6[s]["miss"] > 0.35))
    print("  6 s clock: most kills lost in any one cell:", worst)
a = rows("a")
if a:
    sat = collections.defaultdict(list)
    for r in a:
        sat[r["tag"].split("|")[1] + "|" + r["tag"].split("|")[2]].append(r["peak_v"])
    print("stage a peak speed by envelope and drag:", {k: round(float(np.median(v)), 1) for k, v in sorted(sat.items())})
