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

CLOCKS, AND WHERE THE LINK DELAY GOES (builder ruling 2026-11-18: "it should
be timestamped so delay is factored into guidance"). `latch()` needs every
record's time on THIS machine's clock. Three ways to get it, best first; the
one that produced the belief is reported in `latch_info["time_base"]`:

  "utc"        Each record carries `t_utc`, the UTC time its sample was TAKEN
               (the target autopilot's SYSTEM_TIME; see
               `flight.cue_relay.BootToUtc`), and this side has a UTC source of
               its own (`MonoToUtc`, fed from the interceptor Pixhawk's
               SYSTEM_TIME, or the system clock as a declared fallback). Each
               record's true age is then `utc_at_receipt - t_utc`: the delay of
               radio, laptop and Wi-Fi, MEASURED per record. The sample is
               placed that far back on the local clock, so advancing the fit to
               GO removes the delay instead of assuming it away.
  "min_delay"  No shared clock, but the sender stamped `t_src` on its own
               clock. Spacing comes from `t_src`; the anchor is the smallest
               (receive time minus `t_src`) in the kept history. What is left
               is the link's MINIMUM latency: the belief lags the truth by
               speed times that (0.6 m at 12 m/s and 50 ms).
  "receive"    Neither. Plain receive time; jitter and delay both remain.

FAIL CLOSED ON THE CLOCK. The "utc" path is refused when a record claims to be
from the future, when a delay is implausibly long, when the sender's UTC and
its own `t_src` disagree about the spacing (its clock mapping jumped), or when
this side's UTC source reports a fault. With `allow_unsynced=False` (what the
flight program passes unless told otherwise) that refusal is a refusal to GO;
with it True the receiver drops to "min_delay"/"receive" and says so.

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
import math
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, List, Optional, Tuple

from flight.cue_relay import (DEFAULT_CAPTURE_DURATION_S, MIN_VALID_UNIX_S,
                              CueQualityError, CueRecord, CueSolution, fit_cue)

__all__ = ["CueLinkError", "CueReceiver", "LinkStats", "DEFAULT_PORT",
           "DEFAULT_MAX_STALENESS_AT_GO_S", "encode_record",
           "MonoToUtc", "SystemClockUtc"]

DEFAULT_PORT = 47810
# The newest record must be this fresh at the GO edge. The cue plan's rule is
# "keep the latch-to-trigger gap under about 2 s"; with a live stream at 5 Hz
# the gap is normally 0.2 s, so 1.0 s means "the stream is alive", and a
# stream that died more than a second ago refuses GO.
DEFAULT_MAX_STALENESS_AT_GO_S = 1.0
# How much history to keep. Only the last capture window is fitted; the rest
# is there for the link statistics and the minimum-delay anchor.
DEFAULT_KEEP_S = 12.0
# "utc" time base plausibility. A record may not be from the future by more
# than this (two GPS-disciplined clocks agree far better; this is slack for the
# 1 Hz SYSTEM_TIME mapping), and may not be older than MAX at receipt.
UTC_FUTURE_TOL_S = 0.10
UTC_MAX_DELAY_S = 3.0
# The sender's UTC spacing and its own t_src spacing must agree to this, or
# its boot-to-UTC mapping moved mid-window.
UTC_SPACING_TOL_S = 0.05


class MonoToUtc:
    """THIS machine's monotonic clock -> UTC, from the interceptor autopilot's
    SYSTEM_TIME messages as they arrive over MAVLink.

    Each message gives UTC at the instant the autopilot SENT it; it reaches us
    a few milliseconds later, so `unix - receive_time` reads low by the serial
    delay. The largest value over the last `window` messages is the one with
    the least delay (the same minimum-delay idea as the cue anchor). FAIL
    CLOSED: `utc_now` returns None before `need` agreeing messages, when the
    newest message is older than `stale_s`, when the autopilot has no GPS time,
    and for `need` messages after the offset jumps."""

    source = "vehicle-system-time"

    def __init__(self, window: int = 10, need: int = 3, jump_tol_s: float = 0.25,
                 stale_s: float = 5.0,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._offs: Deque[float] = deque(maxlen=int(window))
        self.need, self.jump_tol_s, self.stale_s = int(need), float(jump_tol_s), float(stale_s)
        self._clock = clock
        self._last_rx: Optional[float] = None
        self.n_msgs = self.n_jumps = self.n_invalid = 0
        self._lock = threading.Lock()

    def ingest(self, time_unix_usec: float, t_rx: Optional[float] = None) -> None:
        t_rx = self._clock() if t_rx is None else float(t_rx)
        unix_s = float(time_unix_usec) / 1e6
        with self._lock:
            self.n_msgs += 1
            if unix_s < MIN_VALID_UNIX_S:
                self.n_invalid += 1
                self._offs.clear()
                return
            off = unix_s - t_rx
            if self._offs and abs(off - max(self._offs)) > self.jump_tol_s:
                self.n_jumps += 1
                self._offs.clear()
            self._offs.append(off)
            self._last_rx = t_rx

    def utc_now(self, mono: Optional[float] = None) -> Optional[float]:
        mono = self._clock() if mono is None else float(mono)
        with self._lock:
            if len(self._offs) < self.need or self._last_rx is None:
                return None
            if mono - self._last_rx > self.stale_s:
                return None
            return mono + max(self._offs)

    @property
    def ok(self) -> bool:
        return self.utc_now() is not None


class SystemClockUtc:
    """DECLARED FALLBACK: this machine's own wall clock as UTC. Right only when
    the clock is disciplined (NTP at the desk, or a GPS-fed chrony). In the
    field the Pi has no network time; use `MonoToUtc` there."""

    source = "system-clock"
    ok = True

    def __init__(self, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time) -> None:
        self._clock, self._wall = clock, wall

    def utc_now(self, mono: Optional[float] = None) -> Optional[float]:
        now_m = self._clock()
        return self._wall() + ((float(mono) - now_m) if mono is not None else 0.0)


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
                  seq: Optional[int] = None,
                  t_utc: Optional[float] = None) -> bytes:
    """One record as one datagram (a JSON object, newline-terminated). `t_src`
    is the sender's clock when the sample was TAKEN and `t_utc` the UTC time it
    was taken (module docstring); `rec.t` is not sent, because the receiver
    stamps its own."""
    row = {"lat": rec.lat, "lon": rec.lon, "alt_m_msl": rec.alt_m_msl}
    if rec.vn is not None and rec.ve is not None and rec.vd is not None:
        row.update(vn=rec.vn, ve=rec.ve, vd=rec.vd)
    if t_src is not None:
        row["t_src"] = float(t_src)
    if t_utc is not None:
        row["t_utc"] = float(t_utc)
    if seq is not None:
        row["seq"] = int(seq)
    return (json.dumps(row) + "\n").encode()


class CueReceiver:
    """Receive cue records, keep the freshest, latch once.

    `feed()` is the whole data path and needs no socket, so the logic is
    testable offline; `start()` only adds the thread that calls it.

    `utc`: this side's UTC source (`MonoToUtc` or `SystemClockUtc`), or None.
    `allow_unsynced`: may the fit fall back to the "min_delay"/"receive" time
    bases when the "utc" one is unavailable or implausible. True by default for
    bench tools; the flight program passes False unless the operator says
    otherwise."""

    def __init__(self, bind: str = "0.0.0.0", port: int = DEFAULT_PORT, *,
                 keep_s: float = DEFAULT_KEEP_S,
                 clock: Callable[[], float] = time.monotonic,
                 on_record: Optional[Callable[[dict, Optional[tuple]], None]] = None,
                 utc=None, allow_unsynced: bool = True) -> None:
        self._bind, self._port = bind, int(port)
        self._keep_s = float(keep_s)
        self._clock = clock
        self._on_record = on_record
        self._utc = utc
        self.allow_unsynced = bool(allow_unsynced)
        self._lock = threading.Lock()
        # (t_rx, t_src or None, CueRecord with t = t_rx, t_utc or None,
        #  utc_at_receipt or None)
        self._buf: Deque[tuple] = deque()
        self._n_rx = self._n_bad = self._n_after_latch = 0
        self._latched: Optional[CueSolution] = None
        self.latch_info: Optional[dict] = None
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # ----------------------------------------------------------------- input

    def feed(self, payload: bytes, t_rx: Optional[float] = None,
             addr: Optional[tuple] = None) -> int:
        """Parse one datagram (one or more JSON lines) received at `t_rx` on
        this machine's clock. Returns how many records were accepted."""
        t_rx = self._clock() if t_rx is None else float(t_rx)
        utc_rx = self._utc.utc_now(t_rx) if self._utc is not None else None
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
                t_utc = float(d["t_utc"]) if "t_utc" in d else None
                if not (-90.0 <= rec.lat <= 90.0 and -180.0 <= rec.lon <= 180.0):
                    raise ValueError("latitude/longitude out of range")
                # Every number must be finite (review 2026-11-26). json.loads
                # accepts NaN and Infinity: a NaN height made every fit fail
                # until the record aged out, and an infinite velocity reached
                # the belief latch and raised at the GO edge.
                nums = [rec.lat, rec.lon, rec.alt_m_msl, rec.vn, rec.ve, rec.vd,
                        t_src, t_utc]
                if not all(math.isfinite(x) for x in nums if x is not None):
                    raise ValueError("non-finite number in a cue record")
            # OverflowError: float() of a huge integer. It was not caught and
            # killed the receive thread, after which GO could never fire.
            except (ValueError, KeyError, TypeError, OverflowError):
                with self._lock:
                    self._n_bad += 1
                continue
            with self._lock:
                if self._latched is not None:
                    self._n_after_latch += 1
                    continue
                self._buf.append((t_rx, t_src, rec, t_utc, utc_rx))
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
            try:
                self.feed(payload, self._clock(), addr)
            except Exception:  # noqa: BLE001 -- one bad datagram must not end the link
                with self._lock:
                    self._n_bad += 1

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

    def _utc_refusal(self, window: list) -> Optional[str]:
        """Why the "utc" time base cannot be used for `window`, or None if it
        can. Every reason is a specific clock fault, never a default."""
        if self._utc is None:
            return "this side has no UTC source"
        if any(tu is None for _, _, _, tu, _ in window):
            return "a record in the window carries no t_utc"
        if any(ur is None for _, _, _, _, ur in window):
            return ("this side's UTC source had no valid time when a record "
                    "arrived")
        delays = [ur - tu for _, _, _, tu, ur in window]
        if min(delays) < -UTC_FUTURE_TOL_S:
            return (f"a record is from the future by {-min(delays):.3f} s "
                    f"(tolerance {UTC_FUTURE_TOL_S:.2f} s): the two clocks disagree")
        if max(delays) > UTC_MAX_DELAY_S:
            return (f"a record was {max(delays):.2f} s old on arrival (limit "
                    f"{UTC_MAX_DELAY_S:.1f} s): a clock is wrong or the link is "
                    f"not usable")
        if all(ts is not None for _, ts, _, _, _ in window):
            ts0, tu0 = window[0][1], window[0][3]
            worst = max(abs((tu - tu0) - (ts - ts0)) for _, ts, _, tu, _ in window)
            if worst > UTC_SPACING_TOL_S:
                return (f"the sender's UTC stamps and its own clock disagree by "
                        f"{worst:.3f} s inside the window: its time mapping jumped")
        return None

    def _window_on_local_clock(self, window_s: float
                               ) -> Tuple[List[CueRecord], dict]:
        """The freshest `window_s` of records with `t` on this machine's clock
        (module docstring), plus how that was done. Raises CueLinkError when
        the only usable time base is one the caller did not allow."""
        items = list(self._buf)
        if not items:
            raise CueLinkError("no cue records received -- do not GO")
        newest_rx = items[-1][0]
        window = [it for it in items if it[0] >= newest_rx - float(window_s)]
        why_not_utc = self._utc_refusal(window)
        info: dict = {"n_window": len(window), "utc_refusal": why_not_utc}
        if why_not_utc is None:
            delays = sorted(ur - tu for _, _, _, tu, ur in window)
            info.update(time_base="utc",
                        utc_source=getattr(self._utc, "source", "?"),
                        delay_min_s=delays[0], delay_max_s=delays[-1],
                        delay_median_s=delays[len(delays) // 2])
            recs = [CueRecord(t=t_rx - (ur - tu), lat=r.lat, lon=r.lon,
                              alt_m_msl=r.alt_m_msl, vn=r.vn, ve=r.ve, vd=r.vd)
                    for t_rx, _, r, tu, ur in window]
            return recs, info
        if not self.allow_unsynced:
            raise CueLinkError(
                f"no shared time base, so link delay cannot be measured: "
                f"{why_not_utc} -- do not GO (allow_unsynced is off)")
        if all(ts is not None for _, ts, _, _, _ in items):
            offset = min(t_rx - ts for t_rx, ts, _, _, _ in items)
            info["time_base"] = "min_delay"
            return [CueRecord(t=ts + offset, lat=r.lat, lon=r.lon,
                              alt_m_msl=r.alt_m_msl, vn=r.vn, ve=r.ve, vd=r.vd)
                    for _, ts, r, _, _ in window], info
        info["time_base"] = "receive"
        return [r for _, _, r, _, _ in window], info

    def stats(self, now_t: Optional[float] = None) -> LinkStats:
        now_t = self._clock() if now_t is None else float(now_t)
        with self._lock:
            ts = [it[0] for it in self._buf]
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

    def _fit(self, own_lat: float, own_lon: float, own_alt_m_msl: float,
             now_t: float, window_s: float, max_staleness_s: float,
             max_gap_s: Optional[float], gate_kwargs: dict
             ) -> Tuple[CueSolution, dict]:
        with self._lock:
            records, info = self._window_on_local_clock(window_s)
        if max_gap_s is not None and len(records) >= 2:
            gap = max(b.t - a.t for a, b in zip(records, records[1:]))
            if gap > float(max_gap_s):
                raise CueQualityError(
                    f"the window has a {gap:.2f} s hole in it (limit "
                    f"{float(max_gap_s):.2f} s): wait for a clean window")
        sol = fit_cue(records, own_lat, own_lon, own_alt_m_msl, now_t=now_t,
                      max_staleness_s=max_staleness_s, **gate_kwargs)
        info["fit_latch_t"] = sol.latch_t
        info["advanced_by_s"] = now_t - sol.latch_t
        return sol.extrapolate_to(now_t), info

    def peek(self, own_lat: float, own_lon: float, own_alt_m_msl: float, *,
             now_t: Optional[float] = None,
             window_s: float = DEFAULT_CAPTURE_DURATION_S,
             max_staleness_s: float = DEFAULT_MAX_STALENESS_AT_GO_S,
             max_gap_s: Optional[float] = None,
             **gate_kwargs) -> CueSolution:
        """What `latch()` WOULD return right now, changing nothing. Raises the
        same refusals. For the pre-GO question "is there a usable cue yet?";
        it is a pre-flight read and must never be called after the latch."""
        now_t = self._clock() if now_t is None else float(now_t)
        if self._latched is not None:
            raise CueLinkError("cue already latched -- nothing reads the link "
                               "after the GO edge")
        sol, _info = self._fit(own_lat, own_lon, own_alt_m_msl, now_t, window_s,
                               max_staleness_s, max_gap_s, gate_kwargs)
        return sol

    def latch(self, own_lat: float, own_lon: float, own_alt_m_msl: float, *,
              now_t: Optional[float] = None,
              window_s: float = DEFAULT_CAPTURE_DURATION_S,
              max_staleness_s: float = DEFAULT_MAX_STALENESS_AT_GO_S,
              max_gap_s: Optional[float] = None,
              **gate_kwargs) -> CueSolution:
        """THE GO-EDGE CALL. Fit the freshest `window_s` of records, refuse if
        the newest is older than `max_staleness_s`, advance the fit to `now_t`
        and freeze. Works once; closes the link whether or not it succeeds in
        fitting (a failed latch means "do not GO", and a new receiver).
        `latch_info` then says which time base produced the belief and, on the
        "utc" base, the measured link delay."""
        now_t = self._clock() if now_t is None else float(now_t)
        if self._latched is not None:
            raise CueLinkError("cue already latched -- the belief is "
                               "written once, at the GO edge")
        try:
            sol, info = self._fit(own_lat, own_lon, own_alt_m_msl, now_t,
                                  window_s, max_staleness_s, max_gap_s,
                                  gate_kwargs)
        except (CueQualityError, CueLinkError):
            self.close()
            raise
        with self._lock:
            self._latched = sol
            self.latch_info = info
        self.close()
        return sol

    @property
    def latched(self) -> Optional[CueSolution]:
        return self._latched
