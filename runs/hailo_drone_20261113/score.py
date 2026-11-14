#!/usr/bin/env python3
"""Recall per target-size bin and false detections on target-free images.
Usage: score.py <eval_dir> <dets_json> [<dets_json> ...]   (bins are in 640-scale pixels)"""
import json, os, sys
d = sys.argv[1]; meta = json.load(open(os.path.join(d, "gt.json"))); size = meta["size"]; k = 640 / size
BINS = [("<8", 0, 8), ("8-16", 8, 16), ("16-32", 16, 32), ("32-64", 32, 64), (">=64", 64, 1e9)]
def iou(a, b):
    iw = min(a[2], b[2]) - max(a[0], b[0]); ih = min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0 or ih <= 0: return 0.0
    return iw * ih / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - iw * ih)
res = {}
for path in sys.argv[2:]:
    dets = json.load(open(path)); hit = {b[0]: 0 for b in BINS}; tot = {b[0]: 0 for b in BINS}; fp_empty = 0; n_empty = 0; ndet = 0
    for name, gts in meta["gt"].items():
        ds = [x for x in dets.get(name, []) if x[4] >= 0.25]; ndet += len(ds)
        if not gts: n_empty += 1; fp_empty += len(ds); continue
        for g in gts:
            m = max(g[2] - g[0], g[3] - g[1]) * k; lab = next(b[0] for b in BINS if b[1] <= m < b[2]); tot[lab] += 1
            hit[lab] += any(iou(g, x) >= 0.3 for x in ds)
    r = {lab: {"n": tot[lab], "recall": round(hit[lab] / tot[lab], 3) if tot[lab] else None} for lab in tot}
    r["all"] = {"n": sum(tot.values()), "recall": round(sum(hit.values()) / max(1, sum(tot.values())), 3)}
    r["false_per_1000_target_free"] = round(1000 * fp_empty / max(1, n_empty), 1); r["target_free_images"] = n_empty; r["detections"] = ndet
    res[os.path.basename(path)] = r
print(json.dumps(res, indent=1))
