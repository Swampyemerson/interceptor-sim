import math
import os

import numpy as np

from isim.trace import read_trace_csv, write_trace_csv


def _make_trace(n: int) -> dict:
    rng = np.random.default_rng(1)
    trace = {
        "t": np.arange(n, dtype=np.float64) * 0.005,
        "own_pos": rng.normal(size=(n, 3)),
        "own_vel": rng.normal(size=(n, 3)),
        "quat": rng.normal(size=(n, 4)),
        "yaw": rng.normal(size=n),
        "cmd": rng.normal(size=(n, 4)),
        "tgt_pos": rng.normal(size=(n, 3)),
        "tgt_vel": rng.normal(size=(n, 3)),
        "range": np.abs(rng.normal(size=n)),
        "saturated": (rng.random(n) > 0.5).astype(np.float64),
        "det_new": np.zeros(n, dtype=np.float64),
        "det_u": np.full(n, math.nan),
        "det_v": np.full(n, math.nan),
        "det_range": np.full(n, math.nan),
    }
    # a few rows "have" a detection
    for i in (0, 3, n - 1):
        if 0 <= i < n:
            trace["det_new"][i] = 1.0
            trace["det_u"][i] = 100.0 + i
            trace["det_v"][i] = 200.0 + i
            trace["det_range"][i] = 50.0 + i
    return trace


def test_trace_round_trip(tmp_path):
    trace = _make_trace(12)
    path = str(tmp_path / "trace.csv")
    write_trace_csv(path, trace)
    assert os.path.exists(path)
    back = read_trace_csv(path)

    assert set(back.keys()) == set(trace.keys())
    for key in trace:
        np.testing.assert_allclose(back[key], trace[key], equal_nan=True, atol=1e-9)


def test_trace_round_trip_empty(tmp_path):
    trace = _make_trace(0)
    path = str(tmp_path / "empty.csv")
    write_trace_csv(path, trace)
    back = read_trace_csv(path)
    for key in trace:
        assert back[key].shape[0] == 0
