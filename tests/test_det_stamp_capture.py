"""--det-stamp-capture (cinema driver, LJ90-S arm): a RELEASED detection
republishes its capture stamp as `t_capture` (the attribute the flight
code's --pursuit-stamped-age path reads) -- flag on only; flag off must not
grow the attribute, keeping the flight code's gz-stamp exclusion meaningful.
"""
from __future__ import annotations

import os
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "cinema"))

cr = pytest.importorskip("cinema_run")


class _Inner:
    latest = None

    def detect(self, frame, t=None):
        return types.SimpleNamespace(range_m=5.0, box_xywh=(1, 2, 3, 4),
                                     t_capture_sim=10.0, tag_range_m=None,
                                     is_new=True)


def _released(stamp_capture: bool):
    inner = _Inner()
    dd = cr.DelayedDetector(inner, 0.09, jitter_s=0.0, seed=1,
                            stamp_capture=stamp_capture)
    inner.latest = object()
    old = cr.xck.stamp_s
    cr.xck.stamp_s = lambda m: 10.2          # past capture+delay -> release
    try:
        return dd.detect(None)
    finally:
        cr.xck.stamp_s = old


def test_flag_on_republishes_capture_stamp_on_the_monotonic_axis():
    """Review F1 (2026-10-22): the stamp must be `t_capture_monotonic`
    (absolute axis, == time.monotonic() under the SimClockShim), NEVER
    `t_capture` -- real_flight ages t_capture against MISSION-RELATIVE t,
    so an absolute stamp there clamps every age to 0.0 (a stamp that
    silently un-ages the measurement). The first version of this test
    pinned that wrong convention and green-lit the bug."""
    d = _released(True)
    assert d.range_m == 5.0
    assert d.t_capture_monotonic == 10.0     # == t_capture_sim, same axis
    assert not hasattr(d, "t_capture")       # the mission-relative name


def test_flag_off_does_not_grow_the_attribute():
    d = _released(False)
    assert d.range_m == 5.0
    assert not hasattr(d, "t_capture")
    assert not hasattr(d, "t_capture_monotonic")


def test_sm_consumption_pins_both_clock_conventions():
    """R2-F2: the SM-side helper must age mission-relative `t_capture`
    against mission time and absolute `t_capture_monotonic` against the
    monotonic clock -- the F1 bug was exactly these crossed."""
    from flight.deploy.real_flight import _stamped_det_age

    # FeedDetector convention: mission-relative stamp, mission t.
    d = types.SimpleNamespace(t_capture=2.5)
    assert _stamped_det_age(d, t_mission=2.8, now_monotonic=1000.0) == pytest.approx(0.3)

    # Cinema convention: absolute monotonic stamp, aged against monotonic.
    d = types.SimpleNamespace(t_capture_monotonic=997.0)
    assert _stamped_det_age(d, t_mission=2.8, now_monotonic=1000.0) == pytest.approx(3.0)

    # The F1 failure shape: an ABSOLUTE stamp wrongly published as
    # t_capture would clamp to 0 against mission time -- assert the helper
    # still exposes that hazard honestly (0.0, not a negative), so the
    # driver-side test (above) remains the guard that the wrong attribute
    # is never published.
    d = types.SimpleNamespace(t_capture=997.0)
    assert _stamped_det_age(d, t_mission=2.8, now_monotonic=1000.0) == 0.0

    # No stamp at all -> None (constant-age fallback path).
    assert _stamped_det_age(types.SimpleNamespace(), 2.8, 1000.0) is None
