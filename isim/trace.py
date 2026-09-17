"""CSV read/write for the engagement trace dict produced by
`isim.engine.run_engagement(..., record_trace=True)`. Plain CSV, one header
row, NaN for missing detection fields.
"""
from __future__ import annotations

import csv
from typing import Dict

import numpy as np

# Vector fields -> their flat CSV column names, in engine.py's own order.
_VECTOR_COLS = {
    "own_pos": ("own_n", "own_e", "own_d"),
    "own_vel": ("own_vn", "own_ve", "own_vd"),
    "quat": ("quat_w", "quat_x", "quat_y", "quat_z"),
    "cmd": ("cmd_vn", "cmd_ve", "cmd_vd", "cmd_yaw_deg"),
    "tgt_pos": ("tgt_n", "tgt_e", "tgt_d"),
    "tgt_vel": ("tgt_vn", "tgt_ve", "tgt_vd"),
}
_ORDER = [
    "t", "own_pos", "own_vel", "quat", "yaw", "cmd", "tgt_pos", "tgt_vel",
    "range", "saturated", "det_new", "det_u", "det_v", "det_range",
]


def _flat_header() -> list:
    header = []
    for key in _ORDER:
        if key in _VECTOR_COLS:
            header.extend(_VECTOR_COLS[key])
        else:
            header.append(key)
    return header


def write_trace_csv(path: str, trace: Dict[str, np.ndarray]) -> None:
    """Write a trace dict (as produced by run_engagement) to plain CSV."""
    header = _flat_header()
    n = len(trace["t"])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for i in range(n):
            row = []
            for key in _ORDER:
                if key in _VECTOR_COLS:
                    row.extend(float(x) for x in trace[key][i])
                else:
                    row.append(float(trace[key][i]))
            w.writerow(row)


def read_trace_csv(path: str) -> Dict[str, np.ndarray]:
    """Inverse of write_trace_csv: returns the same dict-of-arrays shape."""
    with open(path, newline="") as f:
        r = csv.reader(f)
        header = next(r)
        rows = [[float(x) for x in row] for row in r]
    ncols = len(header)
    data = np.array(rows, dtype=np.float64) if rows else np.zeros((0, ncols))
    col_idx = {name: i for i, name in enumerate(header)}
    trace: Dict[str, np.ndarray] = {}
    for key in _ORDER:
        if key in _VECTOR_COLS:
            idxs = [col_idx[c] for c in _VECTOR_COLS[key]]
            trace[key] = data[:, idxs]
        else:
            trace[key] = data[:, col_idx[key]]
    return trace
