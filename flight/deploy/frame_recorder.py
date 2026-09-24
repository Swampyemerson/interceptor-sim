#!/usr/bin/env python3
"""flight/deploy/frame_recorder.py -- the onboard footage recorder for the
Raspberry Pi 5 flight computer (docs/next.md builder directive #3).

WHY THIS EXISTS (triple purpose, one file): every real flight is currently
UNRECORDED -- the seeker sees frames, decides, and throws them away. That
costs three things at once: (1) ENGAGEMENT EVIDENCE (a video of the actual
intercept attempt, the thing a field day is FOR), (2) REAL-DATA RETRAIN
FRAMES (the seeker's weights are fine-tuned offline; every flight not
recorded is training data that never existed), and (3) FIELD DEBUGGING (a
seeker fault report with no imagery is a fault report you cannot diagnose).

THE PI 5 HAS NO HARDWARE H.264 ENCODER. Software video encoding in the flight
loop would burn CPU the seeker's 20 Hz control loop cannot spare (ADR-0090:
the detector alone already consumes ~60% of the CPU budget at the flown
rate). So there is NO video encoding here, in the loop or off it. The
pattern instead: save JPEG frames (cheap, hardware-agnostic `cv2.imencode`)
at a REDUCED rate (~10-15 fps, well under the ~20-60 Hz the seeker itself
runs at) and quality (~80, plenty for both a human-watchable video and a
retrain label), on a DECOUPLED WRITER THREAD, through a BOUNDED QUEUE that
DROPS frames rather than EVER blocking the caller. `offer()` is the only
call the flight loop makes, and it must cost the seeker's latency budget
nothing -- a proximity-fused intercept has no slack for a stalled disk write.
A video can be assembled offline from the JPEG sequence + index.csv any time
after the flight; that is a deliberate, cheap, off-Pi step, not this file's
job.

DESIGN, ONE DECISION AT A TIME
-------------------------------
* RATE-LIMIT BEFORE COPY. `offer()` is called every seeker tick (~20-60 Hz),
  but only accepts a frame once per `1/target_fps` of CAPTURE TIME
  (`t_capture`, not wall time -- the caller's own clock, so this works
  identically against a sim clock or the Pi's monotonic clock). A REJECTED
  frame costs one float comparison: no array is touched, no lock beyond a
  counter increment. This is what keeps `offer()` cheap at the loop's full
  rate rather than the recorder's reduced one.
* COPY POLICY -- documented, not incidental. `offer()` takes `frame.copy()`
  ONLY on frames that pass the rate gate (the throttled ~10-15 fps), never on
  the rejected majority. A bare reference would be cheaper still, but is NOT
  safe in general: some frame sources in this repo hand back the SAME mutable
  buffer on every call (`seeker_loop.SyntheticSource`, and `real_flight.py`'s
  own placeholder smoke frame is a single `np.zeros(...)` reused every tick)
  -- a bare reference into a buffer like that risks the writer thread encoding
  a frame the caller has since overwritten (or, for the reused-buffer cases in
  this repo specifically, encoding stale-but-identical content, which is
  merely wasteful rather than wrong). A camera-backed source
  (`PicameraSource`, `cv2.imread`, `cv2.VideoCapture.read`) already hands back
  a freshly-owned array per call, so the copy costs it nothing for
  correctness -- but `offer()` cannot tell the two kinds of caller apart, so
  it makes the safe assumption for both. Net effect: the array-copy cost is
  paid ONLY at the reduced ~10-15 fps rate the whole design exists to bound
  the caller to, never at the seeker's full loop rate.
* BOUNDED QUEUE, DROP NOT BLOCK. `queue.Queue(maxsize=max_queue)` +
  `put_nowait()`: a full queue (writer stalled on a slow SD card, or just
  behind) raises `Full` immediately, `offer()` counts the drop and returns
  -- it NEVER waits on the writer. This is the one behaviour the whole
  module exists to guarantee, so it is exercised directly by
  `flight/tests/test_frame_recorder.py`'s wedged-writer test.
* CRASH-SAFE INDEX. `index.csv` is opened and header-written at construction
  (so even a zero-frame session leaves a valid, readable file) and every row
  is `flush()`ed the moment it is written, in the writer thread, one row per
  saved JPEG. A SIGKILL'd process therefore leaves a readable index for every
  frame that made it to disk before the kill -- nothing buffered, nothing
  lost silently (the same crash-safety discipline as
  `scripts/seeker/pi_capture.py`'s meta.json-from-`finally`, applied here to
  the ongoing per-row write instead of a single end-of-run summary).
* `close()` signals the writer to stop, DRAINS whatever is already queued
  within a wall-clock deadline, and always writes `meta.json` (counts +
  config + drop rate) even if the writer did not finish in time -- the same
  "never silently claim success" discipline as `pi_capture.record_session`'s
  `finally`-written meta.

NOT DONE HERE (by design): video encoding/muxing (assembled offline from the
JPEG sequence), camera capture itself (the caller's job -- `offer()` takes
whatever frame the seeker already has in hand), disk-space accounting (a
field-ops concern, not a correctness one for this module).
"""
from __future__ import annotations

import csv
import json
import os
import queue
import threading
import time
from datetime import datetime, timezone
from typing import Optional

# Defaults per docs/next.md builder directive #3: a reduced rate/quality that
# is plenty for engagement evidence + retrain frames + field debugging, well
# under the seeker's own 20-60 Hz loop rate (ADR-0090).
DEFAULT_TARGET_FPS = 12.0
DEFAULT_JPEG_QUALITY = 80
DEFAULT_MAX_QUEUE = 8
DEFAULT_CLOSE_DEADLINE_S = 5.0

# seq: the writer's own frame counter, assigned in the CALLER thread at
# offer()-time so a queue-full drop still consumes a seq (index.csv shows a
# gap at exactly the dropped frame, not a lie about how many frames existed).
# t_capture: the caller's own clock (sim time or the Pi's monotonic clock --
# never assumed to be wall time, per this project's sim/wall-time rule).
# t_wall: time.time() at the moment the WRITER thread processed the item --
# the axis a human lines up against a wall-clock event (a field-day log, a
# radio call), distinct from t_capture's rate-limiting role.
# dropped_so_far / queue_depth: the running counters AS OF this row, so a
# post-flight read of index.csv alone (no meta.json needed) shows exactly
# when a stall or a drop happened, not just that one occurred somewhere.
INDEX_HEADER = ["seq", "t_capture", "t_wall", "dropped_so_far", "queue_depth"]


def _now_utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class FrameRecorder:
    """Decoupled JPEG frame recorder. `offer()` is the only call the flight
    loop makes; a daemon writer thread does everything that touches disk.

    Public counters (read any time, thread-safe): `n_offered` (passed the
    rate gate, an enqueue was attempted), `n_rate_skipped` (rejected by the
    fps gate -- zero array cost), `n_written` (JPEG + index row landed on
    disk), `n_dropped` (the queue was full -- the ONLY thing `offer()` ever
    drops a frame FOR), `n_encode_failed` (cv2.imencode() itself failed on an
    accepted frame -- rare, never silently absorbed into `n_written`).
    """

    def __init__(self, out_dir: str, target_fps: float = DEFAULT_TARGET_FPS,
                 jpeg_quality: int = DEFAULT_JPEG_QUALITY,
                 max_queue: int = DEFAULT_MAX_QUEUE):
        self.out_dir = out_dir
        self.target_fps = float(target_fps)
        self.jpeg_quality = int(jpeg_quality)
        self.max_queue = int(max_queue)
        self._min_dt = (1.0 / self.target_fps) if self.target_fps > 0 else 0.0

        os.makedirs(out_dir, exist_ok=True)
        self._index_path = os.path.join(out_dir, "index.csv")
        # Opened + header-written NOW, not on the first accepted frame: a
        # zero-frame session (recorder attached, nothing ever offered/kept)
        # must still leave a readable, valid index.csv behind.
        self._index_f = open(self._index_path, "w", newline="")
        self._index_w = csv.writer(self._index_f)
        self._index_w.writerow(INDEX_HEADER)
        self._index_f.flush()

        self._queue: "queue.Queue" = queue.Queue(maxsize=self.max_queue)
        self._lock = threading.Lock()      # guards the counters + seq below
        self._last_accepted_t: Optional[float] = None
        self._next_seq = 0

        self.n_offered = 0
        self.n_rate_skipped = 0
        self.n_written = 0
        self.n_dropped = 0
        self.n_encode_failed = 0

        self._stop_evt = threading.Event()
        self._closed = False
        self._close_lock = threading.Lock()
        self._meta: Optional[dict] = None
        self._created_utc = _now_utc()

        self._thread = threading.Thread(target=self._writer_loop,
                                        name="frame-recorder-writer",
                                        daemon=True)
        self._thread.start()

    # ------------------------------------------------------------ hot path

    def offer(self, frame, t_capture: float) -> bool:
        """The ONLY call the flight loop makes. O(1): a rejected frame (rate
        gate, or the recorder is closed) costs a comparison and nothing else;
        an accepted frame costs one bounded array copy (see module docstring
        COPY POLICY) plus a non-blocking queue put. NEVER blocks on the
        writer -- a full queue is a drop, not a wait. Returns True if the
        frame was queued for writing, False otherwise (rate-skipped,
        queue-full dropped, or closed)."""
        if self._closed or frame is None:
            return False
        if self._min_dt > 0.0 and self._last_accepted_t is not None \
                and (t_capture - self._last_accepted_t) < self._min_dt:
            with self._lock:
                self.n_rate_skipped += 1
            return False
        self._last_accepted_t = t_capture
        with self._lock:
            seq = self._next_seq
            self._next_seq += 1
            self.n_offered += 1
        try:
            self._queue.put_nowait((seq, frame.copy(), float(t_capture),
                                    time.time()))
        except queue.Full:
            with self._lock:
                self.n_dropped += 1
            return False
        return True

    def queue_depth(self) -> int:
        """Current backlog (0..max_queue). For tests/diagnostics; never
        consulted by `offer()` itself (that would reintroduce a blocking
        check on the hot path -- `put_nowait` already does the bounding)."""
        return self._queue.qsize()

    # ------------------------------------------------------------ writer thread

    def _writer_loop(self) -> None:
        while True:
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                if self._stop_evt.is_set():
                    return
                continue
            self._write_one(*item)

    def _write_one(self, seq: int, frame, t_capture: float,
                   t_wall: float) -> None:
        import cv2  # writer-thread-only: offer() never pays this import
        ok, buf = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            # FAIL CLOSED on a measured quantity: an encode failure is
            # COUNTED, never silently folded into n_written.
            with self._lock:
                self.n_encode_failed += 1
            return
        t_ms = int(round(t_capture * 1000.0))
        fname = f"frame_{seq:08d}_{t_ms}.jpg"
        path = os.path.join(self.out_dir, fname)
        with open(path, "wb") as fh:
            fh.write(buf.tobytes())
        with self._lock:
            self.n_written += 1
            dropped_so_far = self.n_dropped
        try:
            self._index_w.writerow([seq, f"{t_capture:.6f}", f"{t_wall:.6f}",
                                    dropped_so_far, self._queue.qsize()])
            # CRASH-SAFE: flushed per row, so a SIGKILL'd process leaves every
            # row up to this one readable (module docstring).
            self._index_f.flush()
        except (ValueError, OSError):
            # close()'s deadline expired before this item was processed and
            # the index file is already closed (best-effort past the
            # deadline -- close() already recorded closed_cleanly=False /
            # pending_at_close for this case). The JPEG above is still on
            # disk; only its index row is missing.
            pass

    # ------------------------------------------------------------ shutdown

    def close(self, deadline_s: float = DEFAULT_CLOSE_DEADLINE_S) -> dict:
        """Stop accepting new frames, drain whatever is already queued within
        `deadline_s` wall seconds, and ALWAYS write meta.json -- even if the
        writer did not finish draining (a stalled writer must not silently
        claim a clean close). Idempotent: a second call returns the same
        meta dict without re-draining or re-writing meta.json."""
        with self._close_lock:
            if self._closed:
                return self._meta
            self._closed = True

        self._stop_evt.set()
        self._thread.join(timeout=max(0.0, deadline_s))
        drained = not self._thread.is_alive()
        pending = self._queue.qsize()

        try:
            self._index_f.close()
        except OSError:
            pass

        with self._lock:
            n_offered, n_written = self.n_offered, self.n_written
            n_dropped, n_rate_skipped = self.n_dropped, self.n_rate_skipped
            n_encode_failed = self.n_encode_failed

        meta = {
            "out_dir": self.out_dir,
            "config": {
                "target_fps": self.target_fps,
                "jpeg_quality": self.jpeg_quality,
                "max_queue": self.max_queue,
            },
            "n_offered": n_offered,
            "n_rate_skipped": n_rate_skipped,
            "n_written": n_written,
            "n_dropped": n_dropped,
            "n_encode_failed": n_encode_failed,
            "drop_rate": (n_dropped / n_offered) if n_offered else 0.0,
            "closed_cleanly": drained,
            "pending_at_close": 0 if drained else pending,
            "created_utc": self._created_utc,
            "closed_utc": _now_utc(),
            "index_csv": "index.csv",
        }
        with open(os.path.join(self.out_dir, "meta.json"), "w") as fh:
            json.dump(meta, fh, indent=2)
        self._meta = meta
        return meta
