"""The static-camera auto-labeller must find a moving target, box all of it, and take negatives only
from before the target appears. Runs the tool's own synthetic self-test (no video file needed)."""
import importlib.util
import pathlib

import pytest

pytest.importorskip("cv2")


def _load():
    p = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "nn" / "ground_video_labels.py"
    spec = importlib.util.spec_from_file_location("ground_video_labels", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_self_test_passes():
    assert _load().self_test() == 0


def test_no_target_is_a_failure_not_a_pass(tmp_path):
    """A clip with nothing moving must produce zero positives (the caller then exits 1), never invented boxes."""
    import argparse

    import numpy as np

    g = _load()
    rng = np.random.default_rng(1)
    base = rng.integers(120, 200, (120, 160, 3), dtype=np.uint8)

    def gen():
        for i in range(240):
            yield i, np.clip(base.astype(np.int16) + rng.integers(-3, 4, base.shape), 0, 255).astype(np.uint8)

    a = argparse.Namespace(thr=30.0, flight_start=2.0, flight_end=1e9, guard_s=0.2, neg_every_s=0.1, stride=1, min_area=12, max_side=200,
                           coast=15, min_obs=20, min_travel=40.0, label_fps=10.0, neg_frac=0.2, merge_px=4, max_clutter=0.08,
                           clutter_stride=2, clutter_segments=3, clutter_grow=3)
    summary, rows = g.run(gen, lambda i: i / 60.0, a, str(tmp_path))
    assert summary["positives_written"] == 0
    assert summary["negatives_written"] == 0      # negatives are capped as a share of positives: none without positives
