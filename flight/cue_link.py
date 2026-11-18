"""Live cue link: the target's GPS records arrive on the Pi over UDP all through
STANDBY, and the belief is latched ONCE, at the GO edge, already advanced to
that instant.

WHY THIS EXISTS (docs/interface_audit_2026-11-18.md, G3 and G9). The flight
program read the cue file once at program start and then armed, took off and
waited for GO. Nothing advanced the belief to the GO instant, so for a moving
target it was wrong by the target's speed times however long that took: 225 to
480 m at 9 to 12 m/s. The cue plan (docs/cue_relay_plan.md section 8) already
said what to do: keep the stream running through standby, take the FRESHEST
fit at the GO edge, extrapolate it to the trigger instant, then stop reading.
This module is that piece. It does not fit anything itself: the fit, the
quality gate and the extrapolation are `flight.cue_relay`'s.

CLOCKS. Each record is stamped on THIS machine's clock, so `latch()` can
compare it with "now" without any shared time base (the audit's point: the
laptop, the Pi and PX4 share none). A sender may add `t_src`, its own clock at
the moment the sample was taken. Then the spacing between records comes from
`t_src` (no network jitter in it) and only the anchor comes from the receive
clock, through the smallest (receive time minus `t_src`) seen in the window:
the usual minimum-delay estimate. What is left is the link's MINIMUM latency,
which makes the belief lag the truth by speed times that latency (0.6 m at
12 m/s and 50 ms). `scripts/bench/cue_link_bench.py` measures it.

HONESTY BOUNDARY. This is a pre-flight given, graded given-noisy: real GPS
error, read before GO. `latch()` works exactly once; it closes the socket, and
every record that arrives afterwards is counted and dropped. Nothing after GO
can reach the belief through this module. The cue is the external sensor's
track; after the latch the link is dead by design.

FAIL CLOSED. `latch()` raises (`CueQualityError` from the fit, or
`CueLinkError`) on too few records, a stale stream, a bad fit or a second
call. The caller must treat that as "do not GO", never as "fly on the old
belief".
"""
from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, List, Optional, Tuple

from flight.cue_relay import (DEFAULT_CAPTURE_DURATION_S, CueQualityError,
                              CueRecord, CueSolution, fit_cue)

__all__ = ["CueLinkError", "CueReceiver", "LinkStats", "DEFAULT_PORT",
           "DEFAULT_MAX_STALENESS_AT_GO_S", "encode_record"]

DEFAULT_PORT = 47810
# The newest record must be this fresh at the GO edge. The cue plan's rule is
# "keep the latch-to-trigger gap under about 2 s"; with a live stream at 5 Hz
# the gap is normally 0.2 s, so 1.0 s means "the stream is alive", and a
# stream that died more than a second ago refuses GO.
DEFAULT_MAX_STALENESS_AT_GO_S = 1.0
# How much history to keep. Only the last capture window is fitted; the rest
# is there for the link statistics and the minimum-delay anchor.
DEFAULT_KEEP_S = 12.0


class CueLinkError(RuntimeError):
    """The link, not the data, is the problem: latched twice, never started,
    or nothing received."""


@dataclass
class LinkStats:
    n_rx: int                 # records accepted
    n_bad: int                # datagrams or lines that did not parse
    n_after_latch: int        # records that arrived after the latch (dropped)
    newest_age_s: Optional[float]   # None until the first record
    rate_hz: Optional[float]        # over the kept history
    max_gap_s: Optional[float]      # largest spacing in the kept history
    span_s: float                   # kept history, first to last
    latched: bool


def encode_record(rec: CueRecord, *, t_src: Optional[float] = None,
                  seq: Optional[int] = None) -> bytes:
    """One record as one datagram (a JSON object, newline-terminated). `t_src`
    is the sender's clock when the sample was TAKEN; `rec.t` is not sent,
    because the receiver stamps its own."""
    row = {"lat": rec.lat, "lon": rec.lon, "alt_m_msl": rec.alt_m_msl}
    if rec.vn is not None and rec.ve is not None and rec.vd is not None:
        row.update(vn=rec.vn, ve=rec.ve, vd=rec.vd)
    if t_src is not None:
        row["t_src"] = float(t_src)
    if seq is not None:
        row["seq"] = int(seq)
    return (json.dumps(row) + "\n").encode()


class CueReceiver:
    """Receive cue records, keep the freshest, latch once.

    `feed()` is the whole data path and needs no socket, so the logic is
    testable offline; `start()` only adds the thread that calls it."""

    def __init__(self, bind: str = "0.0.0.0", port: int = DEFAULT_PORT, *,
                 keep_s: float = DEFAULT_KEEP_S,
                 clock: Callable[[], float] = time.monotonic,
                 on_record: Optional[Callable[[dict, Optional[tuple]], None]] = None
                 ) -> None:
        self._bind, self._port = bind, int(port)
        self._keep_s = float(keep_s)
        self._clock = clock
        self._on_record = on_record
        self._lock = threading.Lock()
        # (t_rx, t_src or None, CueRecord with t = t_rx for now)
        self._buf: Deque[Tuple[float, Optional[float], CueRecord]] = deque()
        self._n_rx = self._n_bad = self._n_after_latch = 0
        self._latched: Optional[CueSolution] = None
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # ----------------------------------------------------------------- input

    def feed(self, payload: bytes, t_rx: Optional[float] = None,
             addr: Optional[tuple] = None) -> int:
        """Parse one datagram (one or more JSON lines) received at `t_rx` on
        this machine's clock. Returns how many records were accepted."""
        t_rx = self._clock() if t_rx is None else float(t_rx)
        accepted = 0
        for line in payload.decode("utf-8", errors="replace").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
                rec = CueRecord(
                    t=t_rx, lat=float(d["lat"]), lon=float(d["lon"]),
                    alt_m_msl=float(d["alt_m_msl"]),
                    vn=(float(d["vn"]) if "vn" in d else None),
                    ve=(float(d["ve"]) if "ve" in d else None),
                    vd=(float(d["vd"]) if "vd" in d else None))
                t_src = float(d["t_src"]) if "t_src" in d else None
                if not (-90.0 <= rec.lat <= 90.0 and -180.0 <= rec.lon <= 180.0):
                    raise ValueError("latitude/longitude out of range")
            except (ValueError, KeyError, TypeError):
                with self._lock:
                    self._n_bad += 1
                continue
            with self._lock:
                if self._latched is not None:
                    self._n_after_latch += 1
                    continue
                self._buf.append((t_rx, t_src, rec))
                self._n_rx += 1
                while self._buf and t_rx - self._buf[0][0] > self._keep_s:
                    self._buf.popleft()
            accepted += 1
            if self._on_record is not None:
                self._on_record(d, addr)
        return accepted

    def start(self) -> None:
        """Bind the socket and start the receive thread."""
        if self._thread is not None:
            raise CueLinkError("receiver already started")
        if self._latched is not None:
            raise CueLinkError("receiver already latched; make a new one")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._bind, self._port))
        self._sock.settimeout(0.2)
        self._port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._run, name="cue-link",
                                        daemon=True)
        self._thread.start()

    @property
    def port(self) -> int:
        return self._port

    def _run(self) -> None:
        sock = self._sock
        while not self._stop.is_set() and sock is not None:
            try:
                payload, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.feed(payload, self._clock(), addr)

    def close(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    # ---------------------------------------------------------------- output

    def _records_on_local_clock(self) -> List[CueRecord]:
        """The kept records with `t` on this machine's clock. With `t_src` on
        every record: spacing from the sender, anchor from the minimum delay.
        Otherwise: plain receive time."""
        items = list(self._buf)
        if items and all(ts is not None for _, ts, _ in items):
            offset = min(t_rx - ts for t_rx, ts, _ in items)
            return [CueRecord(t=ts + offset, lat=r.lat, lon=r.lon,
                              alt_m_msl=r.alt_m_msl, vn=r.vn, ve=r.ve, vd=r.vd)
                    for _, ts, r in items]
        return [r for _, _, r in items]

    def stats(self, now_t: Optional[float] = None) -> LinkStats:
        now_t = self._clock() if now_t is None else float(now_t)
        with self._lock:
            ts = [t for t, _, _ in self._buf]
            n_rx, n_bad, n_after = self._n_rx, self._n_bad, self._n_after_latch
            latched = self._latched is not None
        if not ts:
            return LinkStats(n_rx, n_bad, n_after, None, None, None, 0.0, latched)
        span = ts[-1] - ts[0]
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        return LinkStats(
            n_rx, n_bad, n_after, now_t - ts[-1],
            ((len(ts) - 1) / span if span > 0 else None),
            (max(gaps) if gaps else None), span, latched)

    def latch(self, own_lat: float, own_lon: float, own_alt_m_msl: float, *,
              now_t: Optional[float] = None,
              window_s: float = DEFAULT_CAPTURE_DURATION_S,
              max_staleness_s: float = DEFAULT_MAX_STALENESS_AT_GO_S,
              **gate_kwargs) -> CueSolution:
        """THE GO-EDGE CALL. Fit the freshest `window_s` of records, refuse if
        the newest is older than `max_staleness_s`, advance the fit to `now_t`
        and freeze. Works once; closes the link whether or not it succeeds in
        fitting (a failed latch means "do not GO", and a new receiver)."""
        now_t = self._clock() if now_t is None else float(now_t)
        with self._lock:
            if self._latched is not None:
                raise CueLinkError("cue already latched -- the belief is "
                                   "written once, at the GO edge")
            records = self._records_on_local_clock()
        if not records:
            self.close()
            raise CueLinkError("no cue records received -- do not GO")
        newest = records[-1].t
        window = [r for r in records if r.t >= newest - float(window_s)]
        try:
            sol = fit_cue(window, own_lat, own_lon, own_alt_m_msl, now_t=now_t,
                          max_staleness_s=max_staleness_s, **gate_kwargs)
            sol = sol.extrapolate_to(now_t)
        except CueQualityError:
            self.close()
            raise
        with self._lock:
            self._latched = sol
        self.close()
        return sol

    @property
    def latched(self) -> Optional[CueSolution]:
        return self._latched
