#!/usr/bin/env python3
"""Side-by-side recall table from score.py output on stdin: compare.py <float_json_name> <chip_json_name>"""
import json, sys
r = json.load(sys.stdin); a, b = r[sys.argv[1]], r[sys.argv[2]]
for k in ("<8", "8-16", "16-32", "32-64", ">=64", "all"):
    print(f"{k:6s} n={a[k]['n']:5d}  A {a[k]['recall']:.3f}  B {b[k]['recall']:.3f}  B-A {100 * (b[k]['recall'] - a[k]['recall']):+.1f} pts")
print("false per 1000 target-free images: A", a["false_per_1000_target_free"], " B", b["false_per_1000_target_free"])
