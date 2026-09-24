"""flight/tests/test_frame_recorder.py -- the onboard footage recorder
(flight/deploy/frame_recorder.FrameRecorder, docs/next.md builder directive
#3): a decoupled writer thread saving reduced-rate/quality JPEGs behind a
BOUNDED queue that DROPS frames rather than EVER blocking the seeker loop.

WHAT MUST BE PINNED, AND WHY
-----------------------------
* `offer()` must never block, even with a wedged writer -- that is the ONE
  behaviour the whole module exists to guarantee (a stalled SD card must not
  cost the seeker's control-loop latency budget).
* the bounded queue must never exceed `max_queue` (the stdlib `queue.Queue`
  already enforces this; still pinned as a black-box property of
  FrameRecorder, not an implementation detail we trust silently).
* the target_fps rate limit must actually throttle -- a caller offering every
  tick (~20-60 Hz) must not write every tick.
* `close()` must drain within its deadline and its meta.json totals must be
  internally consistent (n_offered == n_written + n_dropped when nothing was
  wedged, and the reverse -- an un-drained close still reports honestly).
* `index.csv` rows must parse and be monotonic in `seq` -- the field record
  the crash-safety claim rests on.
* the seeker_loop hook (`run_over_source`) must call `offer()` exactly once
  per processed frame, and only AFTER the detector has consumed it -- using
  the same drive pattern flight/tests/test_seeker_loop_coast.py establishes
  for exercising the deployed loop without a camera/MAVSDK/sim.

No camera, no MAVSDK, no sim: everything here runs on synthetic in-memory
frames.
"""
import csv
import json
import os
import sys
import threading
import time

import numpy as np
import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from flight.deploy.frame_recorder import FrameRecorder, INDEX_HEADER  # noqa: E402


def _frame(w=16, h=12, val=0):
    """A tiny synthetic grayscale frame -- cheap to encode, real enough for
    cv2.imencode to exercise the actual JPEG path (not a mock)."""
    return np.full((h, w), val, dtype=np.uint8)


# ============================================================ offer() never blocks


def test_offer_never_blocks_with_a_wedged_writer(tmp_path):
    """THE CORE GUARANTEE. A writer stuck on a slow encode must not cost the
    caller's thread anything beyond the bounded queue check -- offer() has to
    return fast and the drop counter has to climb, never a hang."""
    unblock = threading.Event()
    entered = threading.Event()

    def wedged_write_one(self, seq, frame, t_capture, t_wall):
        entered.set()
        unblock.wait(timeout=5.0)   # simulates a stalled SD card

    orig = FrameRecorder._write_one
    FrameRecorder._write_one = wedged_write_one
    try:
        rec = FrameRecorder(str(tmp_path), max_queue=4)
        # Offer ONE frame to give the writer thread something to pick up and
        # wedge on, then confirm it actually did before flooding it.
        accepted = 1 if rec.offer(_frame(), 0.0) else 0
        assert entered.wait(timeout=2.0), "writer thread never started"

        t0 = time.monotonic()
        for i in range(1, 60):
            # Timestamps spaced 1.0 s apart in the CALLER's own clock so every
            # call clears the target_fps rate gate deterministically, with no
            # dependency on real wall-clock timing.
            if rec.offer(_frame(), i * 1.0):
                accepted += 1
        elapsed = time.monotonic() - t0

        assert elapsed < 1.0, (
            f"offer() blocked on a wedged writer: {elapsed:.2f}s for 60 calls")
        assert rec.n_dropped > 0, "a wedged writer must produce queue-full drops"
        assert accepted < 60, "some offers must have been dropped, not queued"
        assert rec.queue_depth() <= 4
    finally:
        unblock.set()
        FrameRecorder._write_one = orig
        meta = rec.close(deadline_s=2.0)
    assert meta["n_dropped"] > 0
    assert meta["closed_cleanly"] is True   # the writer catches up once unblocked


def test_offer_returns_false_and_costs_nothing_once_closed(tmp_path):
    rec = FrameRecorder(str(tmp_path))
    rec.close()
    assert rec.offer(_frame(), 0.0) is False


# ============================================================ bounded queue


def test_bounded_queue_never_exceeds_max_queue(tmp_path):
    unblock = threading.Event()

    def wedged(self, seq, frame, t_capture, t_wall):
        unblock.wait(timeout=5.0)

    orig = FrameRecorder._write_one
    FrameRecorder._write_one = wedged
    try:
        rec = FrameRecorder(str(tmp_path), max_queue=5)
        depths = []
        for i in range(40):
            rec.offer(_frame(), i * 1.0)
            depths.append(rec.queue_depth())
        assert max(depths) <= 5, f"queue depth exceeded max_queue: {depths}"
    finally:
        unblock.set()
        FrameRecorder._write_one = orig
        rec.close(deadline_s=2.0)


# ============================================================ rate limiting


def test_rate_limiting_honors_target_fps(tmp_path):
    """A caller offering every tick of a fast loop must only get every
    Nth frame ACCEPTED, at the configured target_fps -- not throttled by
    wall time (the caller's own clock, `t_capture`, drives the gate)."""
    rec = FrameRecorder(str(tmp_path), target_fps=10.0, max_queue=1000)
    loop_hz = 100.0
    n_ticks = 500     # 5.0 s of virtual capture time at 100 Hz
    accepted = 0
    for i in range(n_ticks):
        if rec.offer(_frame(), i / loop_hz):
            accepted += 1
    meta = rec.close(deadline_s=2.0)
    # 5.0 s at 10 fps = ~50 accepted frames. Tolerance 2 (not 1): float
    # accumulation in `i / loop_hz` can land a handful of ticks on either
    # side of the exact 0.1 s boundary.
    expected = 5.0 * 10.0
    assert abs(accepted - expected) <= 2, (
        f"accepted {accepted} frames over 5.0s at target_fps=10 "
        f"(expected ~{expected:.0f})")
    assert rec.n_rate_skipped == n_ticks - accepted
    assert meta["n_written"] == accepted, "an unwedged writer must write everything offered"


def test_first_frame_is_always_accepted_regardless_of_rate(tmp_path):
    rec = FrameRecorder(str(tmp_path), target_fps=1.0)
    assert rec.offer(_frame(), 0.0) is True
    rec.close(deadline_s=2.0)


# ============================================================ close() + meta.json


def test_close_drains_and_meta_totals_match(tmp_path):
    rec = FrameRecorder(str(tmp_path), target_fps=1000.0, max_queue=100)
    n = 20
    for i in range(n):
        assert rec.offer(_frame(val=i % 256), i * 1.0) is True
    meta = rec.close(deadline_s=5.0)

    assert meta["closed_cleanly"] is True
    assert meta["pending_at_close"] == 0
    assert meta["n_offered"] == n
    assert meta["n_written"] == n
    assert meta["n_dropped"] == 0
    assert meta["n_encode_failed"] == 0
    assert meta["drop_rate"] == pytest.approx(0.0)
    assert meta["config"] == {"target_fps": 1000.0, "jpeg_quality": 80,
                              "max_queue": 100}

    with open(os.path.join(str(tmp_path), "meta.json")) as fh:
        on_disk = json.load(fh)
    assert on_disk == meta

    # n JPEGs actually landed on disk.
    jpgs = [f for f in os.listdir(str(tmp_path)) if f.endswith(".jpg")]
    assert len(jpgs) == n


def test_close_is_idempotent(tmp_path):
    rec = FrameRecorder(str(tmp_path))
    rec.offer(_frame(), 0.0)
    m1 = rec.close(deadline_s=2.0)
    m2 = rec.close(deadline_s=2.0)
    assert m1 == m2


def test_a_wedged_close_still_writes_meta_json_honestly(tmp_path):
    """close() must never claim a clean drain it did not achieve."""
    unblock = threading.Event()

    def wedged(self, seq, frame, t_capture, t_wall):
        unblock.wait(timeout=5.0)

    orig = FrameRecorder._write_one
    FrameRecorder._write_one = wedged
    try:
        rec = FrameRecorder(str(tmp_path), max_queue=10)
        for i in range(5):
            rec.offer(_frame(), i * 1.0)
        meta = rec.close(deadline_s=0.2)   # deliberately too short to drain
    finally:
        unblock.set()
        FrameRecorder._write_one = orig
    assert meta["closed_cleanly"] is False
    assert meta["pending_at_close"] >= 1
    with open(os.path.join(str(tmp_path), "meta.json")) as fh:
        assert json.load(fh)["closed_cleanly"] is False


# ============================================================ index.csv


def test_index_csv_rows_parse_and_are_monotonic_in_seq(tmp_path):
    rec = FrameRecorder(str(tmp_path), target_fps=1000.0, max_queue=100)
    n = 15
    for i in range(n):
        rec.offer(_frame(), i * 1.0)
    rec.close(deadline_s=5.0)

    with open(os.path.join(str(tmp_path), "index.csv"), newline="") as fh:
        rows = list(csv.reader(fh))
    assert rows[0] == INDEX_HEADER
    body = rows[1:]
    assert len(body) == n
    seqs = [int(r[0]) for r in body]
    assert seqs == sorted(seqs), "seq must be monotonic"
    assert seqs == list(range(n)), "no queue-full drops expected here"
    for r in body:
        seq, t_capture, t_wall, dropped_so_far, queue_depth = r
        float(t_capture)       # parses as a float
        float(t_wall)
        assert int(dropped_so_far) >= 0
        assert int(queue_depth) >= 0


def test_index_csv_readable_after_construction_with_zero_frames(tmp_path):
    """CRASH-SAFETY: even a session where nothing was ever offered leaves a
    valid, readable index.csv (header only)."""
    rec = FrameRecorder(str(tmp_path))
    with open(os.path.join(str(tmp_path), "index.csv"), newline="") as fh:
        rows = list(csv.reader(fh))
    assert rows == [INDEX_HEADER]
    rec.close(deadline_s=2.0)


def test_index_csv_gaps_at_queue_full_drops(tmp_path):
    """A queue-full drop consumes a seq that never appears in index.csv --
    the gap IS the evidence of the drop, not a lie about frame count."""
    unblock = threading.Event()

    def wedged(self, seq, frame, t_capture, t_wall):
        unblock.wait(timeout=5.0)

    orig = FrameRecorder._write_one
    FrameRecorder._write_one = wedged
    try:
        rec = FrameRecorder(str(tmp_path), max_queue=2)
        for i in range(10):
            rec.offer(_frame(), i * 1.0)
        assert rec.n_dropped > 0
    finally:
        unblock.set()
        FrameRecorder._write_one = orig
        rec.close(deadline_s=2.0)

    with open(os.path.join(str(tmp_path), "index.csv"), newline="") as fh:
        body = list(csv.reader(fh))[1:]
    seqs = [int(r[0]) for r in body]
    assert seqs == sorted(seqs)
    assert len(seqs) == len(set(seqs)), "seq must not repeat"
    assert len(seqs) < 10, "the dropped seqs must be absent, not backfilled"


# ============================================================ the copy/no-copy contract


def test_offer_copies_the_accepted_frame_not_a_reference(tmp_path):
    """COPY POLICY (module docstring): offer() must not hold a bare reference
    to a mutable buffer the caller goes on to mutate/reuse -- e.g.
    seeker_loop.SyntheticSource and real_flight.py's own placeholder frame
    reuse ONE array across every tick. Mutating the source array after
    offer() accepted it must not change what gets written."""
    rec = FrameRecorder(str(tmp_path), target_fps=1000.0, max_queue=10)
    buf = _frame(val=5)
    assert rec.offer(buf, 0.0) is True
    buf[:, :] = 200   # mutate AFTER offer() returned
    # Drain the queue item directly (before the writer thread races us).
    seq, queued_frame, t_capture, t_wall = rec._queue.get(timeout=2.0)
    assert (queued_frame == 5).all(), (
        "offer() held a bare reference -- the post-offer mutation leaked "
        "into the queued frame")
    rec.close(deadline_s=2.0)


# ============================================================ the seeker_loop hook


class _FakeSource:
    """Minimal `.frames()` source, mirroring seeker_loop's ImageDirSource
    contract (yields (frame, name) pairs) without touching disk."""

    def __init__(self, n):
        self.n = n

    def frames(self):
        for i in range(self.n):
            yield _frame(val=i % 256), f"synth{i:05d}"


class _CountingDetector:
    """Appends 'detect' to a SHARED events list on every call, so the test can
    reconstruct the true single-threaded emission order against the
    offer()-spy's 'offer' entries in the SAME list (list.append order IS
    call order here -- everything runs on one thread, synchronously)."""

    def __init__(self, events):
        self.events = events

    def detect(self, frame, t):
        from types import SimpleNamespace
        self.events.append(("detect", t))
        return SimpleNamespace(box_xywh=None, range_m=None)


def test_seeker_loop_offers_exactly_once_per_frame_after_detection(tmp_path):
    """flight/deploy/seeker_loop.run_over_source: with a recorder attached,
    offer() must be called exactly once per processed frame, and strictly
    AFTER detector.detect() for that same frame -- driven the same way
    flight/tests/test_seeker_loop_coast.py drives the deployed loop (no
    camera, no MAVSDK, no sim)."""
    from flight.camera import CameraModel
    from flight.deploy.seeker_loop import GuidanceConfig, SeekerGuidance, run_over_source

    events = []
    detector = _CountingDetector(events)
    cam = CameraModel(539.936, 539.936, 640.0, 480.0)
    guidance = SeekerGuidance(GuidanceConfig(), cam, span_m=1.0)
    n_frames = 12
    source = _FakeSource(n_frames)
    rec = FrameRecorder(str(tmp_path), target_fps=1000.0, max_queue=100)

    orig_offer = FrameRecorder.offer

    def spy_offer(self, frame, t_capture):
        events.append(("offer", t_capture))
        return orig_offer(self, frame, t_capture)

    FrameRecorder.offer = spy_offer
    try:
        run_over_source(source, detector, guidance, dry_run=True,
                        verbose=False, cam=cam, frame_recorder=rec)
    finally:
        FrameRecorder.offer = orig_offer
        meta = rec.close(deadline_s=2.0)

    assert len(events) == 2 * n_frames, (
        f"expected exactly one detect + one offer per frame, got {events}")
    # Strict per-tick ordering: events[0], events[2], ... are the detects and
    # events[1], events[3], ... are the offers -- i.e. for every frame the
    # detect() call for it precedes the offer() call for it, with no
    # detect-before-any-offer batching and no offer preceding its own detect.
    kinds = [e[0] for e in events]
    assert kinds == ["detect", "offer"] * n_frames, (
        "offer() must fire immediately after detect() for the SAME frame, "
        f"never before it and never batched: {kinds}")
    # And the timestamps line up tick-for-tick (same t passed to both calls).
    for i in range(n_frames):
        assert events[2 * i][1] == events[2 * i + 1][1]
    assert meta["n_written"] == n_frames


def test_run_over_source_with_no_recorder_is_unaffected(tmp_path):
    """Default-off path: frame_recorder=None must not import/touch the
    recorder machinery, and behaves exactly as before."""
    from flight.camera import CameraModel
    from flight.deploy.seeker_loop import GuidanceConfig, SeekerGuidance, run_over_source

    detector = _CountingDetector([])
    cam = CameraModel(539.936, 539.936, 640.0, 480.0)
    guidance = SeekerGuidance(GuidanceConfig(), cam, span_m=1.0)
    source = _FakeSource(5)
    log = run_over_source(source, detector, guidance, dry_run=True,
                          verbose=False, cam=cam)
    assert len(log) == 5
