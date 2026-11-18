#!/usr/bin/env python3
"""Ground side of the cue link, fed from the RADIO CONTROL LINK instead of a
telemetry radio: read raw CRSF telemetry bytes, decode the GPS frame, and send
one UDP datagram per GPS frame to the Pi (same datagram as
`flight.cue_link.encode_record`, default port 47810).

    target autopilot --CRSF telemetry--> receiver ==radio==> transmitter
        --USB serial ("telemetry mirror")--> THIS PROGRAM --UDP/Wi-Fi--> Pi

Sources (pick one):
  --port COM7        a serial port (Windows Python + pyserial; WSL2 cannot see
                     COM ports). pyserial is imported only when this is used.
  --file PATH        raw bytes from a file, or "-" for stdin (bench and tests).
  --udp-listen PORT  raw bytes from UDP datagrams (the transmitter's Wi-Fi
                     "backpack"). EXPERIMENTAL, see UNVERIFIED below.

This program only LISTENS to the radio and SENDS position reports to the Pi.
It never writes a byte to the serial port and it commands nothing.

Stand-alone on purpose: standard library plus (lazily) pyserial, no import
from this repository, so the single file can be copied to a Windows folder.
`tests/test_crsf_cue_source.py` holds it to the receiver's own writer and
reader (`flight.cue_link.encode_record`, `CueReceiver.feed`).


WIRE FORMAT -- CONFIRMED AGAINST PRIMARY SOURCES (read 2026-11-18)

[1] TBS CRSF specification, https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
    * Frame:  [sync] [length] [type] [payload ...] [crc]
    * "Sync byte might be one of (so, receiving device should expect any of):
      Serial sync byte: 0xC8; Broadcast device address: 0x00; Device address".
    * Length = "number of bytes in the frame excluding Sync byte and Frame
      Length (basically, entire frame size -2)", i.e. type + payload + crc.
      "Valid range is between 2 and 62"; a frame is at most 64 bytes.
    * "CRC includes Type and Payload of each frame (doesn't include sync byte
      and frame length)." "CRC8 implementation with polynom = x7+ x6+ x4+ x2+
      x0 (0xD5)". Initial value 0, no reflection (the spec's lookup table
      starts 0x00, 0xD5, 0x7F, 0xAA ..., which is that CRC).
    * "Endianness: Big endian (MSB)".
    * 0x02 GPS, 15 payload bytes:
          int32_t  latitude;     // degree / 10`000`000
          int32_t  longitude;    // degree / 10`000`000
          uint16_t groundspeed;  // km/h / 10
          uint16_t heading;      // degree / 100
          uint16_t altitude;     // meter - 1000m offset
          uint8_t  satellites;   // # of sats in view
      (So the raw integers are degrees x 1e7, km/h x 10, degrees x 100 and
      metres + 1000.)
    * Types below 0x27 have this short header; 0x28 and above carry a
      destination and origin address first. Both are only COUNTED here.

[2] EdgeTX, radio/src/telemetry/crossfire.cpp (main branch): the handset reads
    the same frame with the same offsets and scalings (lat/lon signed 4 bytes,
    ground speed km/h with 1 decimal, heading degrees with 2 decimals,
    altitude minus 1000, satellites), and accepts a frame whose first byte is
    0xEA (the handset's address) or 0xC8. ExpressLRS addresses frames it hands
    to the handset to 0xEA. Hence SYNC_BYTES below.

[3] ArduPilot, libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp, tag Copter-4.7.0,
    `calc_gps()` -- what the target actually puts in the frame:
          latitude    = loc.lat            (first GPS, degrees x 1e7)
          longitude   = loc.lng
          groundspeed = ground_speed() * 36.0     (m/s -> km/h x 10)
          altitude    = constrain(loc.alt / 100, 0, 5000) + 1000
          gps_heading = ground_course() * 100     (course over ground, deg)
          satellites  = num_sats()
    The same file caps the GPS frame at one every 500 ms (2 Hz) when the link
    has more than 30 telemetry slots a second, and one every 1000 ms
    otherwise. OLDER ArduPilot (4.6.0 and before) multiplied ground speed by
    100000/3600 instead of 36, so speed read 23% low; fixed 2025-04-30
    ("CRSF ground speed incorrect by some 20%"), present in 4.6.3 and 4.7.0.


WHAT THE FRAME CANNOT TELL US (limits, not bugs)

  * NO FIX FLAG. The frame has no fix-type field. "No fix" is inferred from:
    fewer than --min-sats satellites (default 4, the least a 3-D fix can
    have), latitude and longitude both exactly zero, or a field out of range.
    Any of these REFUSES the frame: nothing is sent, the refusal is printed
    loudly and counted, and the exit code is non-zero if nothing was sent.
  * SPEED AND COURSE, NOT A VELOCITY VECTOR. vn and ve are computed here from
    ground speed and course over ground; vd is sent as 0.0 because the frame
    has no vertical speed. Course is meaningless when the target is nearly
    stationary. Resolution: 0.1 km/h (0.028 m/s) and 0.01 degree.
  * ALTITUDE IN WHOLE METRES, GPS height above mean sea level, clipped by
    ArduPilot to 0..5000 m.
  * NO SAMPLE TIME. `t_src` is this machine's time.monotonic() when the read
    that delivered the frame's last byte returned. It includes the radio's
    delay and its jitter (tens to hundreds of milliseconds), unlike the
    MAVLink source in cue_send.py, which carries the autopilot's own clock.
  * AT MOST 2 Hz from ArduPilot 4.7 (see [3]).


UNVERIFIED UNTIL TRIED ON THE REAL TRANSMITTER (2026-11-18: desk work only)

  * That the RadioMaster Pocket's USB port, in "USB Serial (VCP)" mode with
    USB-VCP set to "Telem Mirror", emits these raw frames byte for byte. The
    EdgeTX source says it mirrors every byte read from the internal module;
    nobody has looked at the bytes from this radio. Use --record to keep them.
  * Whether the port needs DTR asserted to start sending (--dtr on/off).
  * The --udp-listen path. The ExpressLRS Backpack source (Tx_main.cpp,
    devWIFI.cpp, master) forwards each CRSF telemetry frame, wrapped in an MSP
    packet, as a UDP broadcast on port 14550 when Backpack telemetry is set to
    WiFi. The scanner here finds the CRSF frame inside the wrapper. Which
    released Backpack version does this is not confirmed, and it is untested.
  * "satellites": the spec says "in view", ArduPilot sends its num_sats().
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass
from typing import BinaryIO, Callable, Deque, Dict, Iterator, List, Optional, Tuple

__all__ = [
    "DEFAULT_PORT", "SYNC_BYTES", "FRAME_TYPE_GPS", "GPS_PAYLOAD_LEN",
    "crc8_d5", "build_frame", "build_gps_payload", "CrsfFrame", "CrsfParser",
    "GpsFix", "NoGpsFix", "decode_gps", "check_fix", "encode_datagram",
    "CueSource", "main",
]

DEFAULT_PORT = 47810            # flight.cue_link.DEFAULT_PORT (checked by the tests)

# First byte of a frame. 0xC8: the serial sync byte / flight controller.
# 0xEA: the handset, which is how the transmitter module addresses what it
# hands to the radio. 0xEE / 0xEC: transmitter module / receiver. The
# broadcast address 0x00 is left out on purpose: zeros are common in garbage
# and every false start costs a CRC check and a line in the failure count.
SYNC_BYTES = frozenset((0xC8, 0xEA, 0xEE, 0xEC))
LEN_MIN, LEN_MAX = 2, 62        # spec: valid range of the length byte

FRAME_TYPE_GPS = 0x02
GPS_PAYLOAD_LEN = 15
_GPS_STRUCT = struct.Struct(">iiHHHB")      # big endian, 15 bytes

FRAME_NAMES: Dict[int, str] = {
    0x02: "GPS", 0x03: "GPSTIME", 0x06: "GPSEXT", 0x07: "VARIO", 0x08: "BATT",
    0x09: "BARO", 0x0A: "AIRSPD", 0x0B: "HBEAT", 0x14: "LINK", 0x16: "RC",
    0x1C: "LINKRX", 0x1D: "LINKTX", 0x1E: "ATT", 0x21: "MODE", 0x28: "PING",
    0x29: "DEVINFO", 0x3A: "RADIO", 0x80: "APCUST",
}

DEFAULT_MIN_SATS = 4
ALT_OFFSET_M = 1000


# ------------------------------------------------------------------ framing

def _make_crc_table() -> Tuple[int, ...]:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return tuple(table)


_CRC_TABLE = _make_crc_table()


def crc8_d5(data: bytes) -> int:
    """CRC-8, polynomial 0xD5, initial value 0, no reflection (DVB-S2)."""
    c = 0
    for b in data:
        c = _CRC_TABLE[c ^ b]
    return c


def build_frame(frame_type: int, payload: bytes, sync: int = 0xC8) -> bytes:
    """One CRSF frame. Used by the tests and for making bench files."""
    body = bytes([frame_type & 0xFF]) + bytes(payload)
    length = len(body) + 1
    if not LEN_MIN <= length <= LEN_MAX:
        raise ValueError(f"CRSF length byte {length} outside {LEN_MIN}..{LEN_MAX}")
    return bytes([sync & 0xFF, length]) + body + bytes([crc8_d5(body)])


def build_gps_payload(lat_deg: float, lon_deg: float, speed_ms: float,
                      course_deg: float, alt_m_msl: float, sats: int) -> bytes:
    """The 15 GPS payload bytes from engineering values, rounded the way
    ArduPilot rounds them (see [3] in the module docstring)."""
    return _GPS_STRUCT.pack(
        int(round(lat_deg * 1e7)), int(round(lon_deg * 1e7)),
        int(round(speed_ms * 36.0)), int(round(course_deg * 100.0)),
        int(round(alt_m_msl)) + ALT_OFFSET_M, int(sats))


@dataclass(frozen=True)
class CrsfFrame:
    sync: int
    type: int
    payload: bytes
    t_read: float       # clock value of the read that delivered the last byte


class CrsfParser:
    """Find CRSF frames in a byte stream that may start mid-frame, contain
    garbage, and arrive in arbitrary pieces.

    A candidate starts at a sync byte followed by a length in 2..62. It is a
    frame only if its CRC matches. On a CRC mismatch ONE byte is dropped and
    the search resumes, so a false start inside garbage cannot swallow the
    real frame behind it. An incomplete candidate waits for more bytes;
    whatever is still waiting at the end of the input is `pending_bytes`."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._base = 0                      # stream offset of _buf[0]
        self._chunks: Deque[Tuple[int, float]] = deque()   # (end offset, t)
        self.n_bytes = 0
        self.n_frames = 0
        self.n_crc_fail = 0
        self.n_skipped_bytes = 0            # bytes thrown away while resyncing

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)

    def _time_of(self, end_offset: int) -> float:
        """Clock value of the chunk that contained stream byte end_offset-1."""
        while len(self._chunks) > 1 and self._chunks[0][0] < end_offset:
            self._chunks.popleft()
        return self._chunks[0][1]

    def feed(self, data: bytes, t: float) -> List[CrsfFrame]:
        """Add bytes that one read returned at clock value `t`; return every
        frame completed so far, in order."""
        if not data:
            return []
        self._buf += data
        self.n_bytes += len(data)
        self._chunks.append((self._base + len(self._buf), float(t)))
        out: List[CrsfFrame] = []
        buf = self._buf
        i = 0
        n = len(buf)
        while i < n:
            if buf[i] not in SYNC_BYTES:
                i += 1
                self.n_skipped_bytes += 1
                continue
            if i + 1 >= n:
                break                       # need the length byte
            length = buf[i + 1]
            if not LEN_MIN <= length <= LEN_MAX:
                i += 1
                self.n_skipped_bytes += 1
                continue
            end = i + 2 + length
            if end > n:
                break                       # need the rest of the frame
            if crc8_d5(bytes(buf[i + 2:end - 1])) != buf[end - 1]:
                self.n_crc_fail += 1
                i += 1
                self.n_skipped_bytes += 1
                continue
            out.append(CrsfFrame(sync=buf[i], type=buf[i + 2],
                                 payload=bytes(buf[i + 3:end - 1]),
                                 t_read=self._time_of(self._base + end)))
            self.n_frames += 1
            i = end
        del buf[:i]
        self._base += i
        # chunks that ended at or before the consumed point can never be the
        # home of a later frame's last byte
        while self._chunks and self._chunks[0][0] <= self._base:
            self._chunks.popleft()
        return out


# ---------------------------------------------------------------- GPS frame

class NoGpsFix(ValueError):
    """The GPS frame does not describe a usable position. Never sent."""


@dataclass(frozen=True)
class GpsFix:
    lat: float              # degrees
    lon: float              # degrees
    speed_ms: float         # ground speed, m/s
    course_deg: float       # course over ground, degrees from north, clockwise
    alt_m_msl: float        # metres above mean sea level (whole metres)
    sats: int
    lat_e7: int
    lon_e7: int

    @property
    def vn(self) -> float:
        return self.speed_ms * math.cos(math.radians(self.course_deg))

    @property
    def ve(self) -> float:
        return self.speed_ms * math.sin(math.radians(self.course_deg))


def decode_gps(payload: bytes) -> GpsFix:
    """Decode the 15-byte payload of a type 0x02 frame. Raises ValueError on
    any other length: a GPS frame of the wrong size is not guessed at."""
    if len(payload) != GPS_PAYLOAD_LEN:
        raise ValueError(f"GPS payload is {len(payload)} bytes, expected "
                         f"{GPS_PAYLOAD_LEN}")
    lat_e7, lon_e7, gs, hdg, alt, sats = _GPS_STRUCT.unpack(payload)
    return GpsFix(lat=lat_e7 / 1e7, lon=lon_e7 / 1e7,
                  speed_ms=(gs / 10.0) / 3.6, course_deg=hdg / 100.0,
                  alt_m_msl=float(alt - ALT_OFFSET_M), sats=sats,
                  lat_e7=lat_e7, lon_e7=lon_e7)


def check_fix(fix: GpsFix, min_sats: int = DEFAULT_MIN_SATS) -> None:
    """Raise NoGpsFix unless the frame can be a real position. The frame has
    no fix flag, so this is inference (module docstring, "no fix flag")."""
    if fix.sats <= 0:
        raise NoGpsFix("zero satellites")
    if fix.sats < min_sats:
        raise NoGpsFix(f"only {fix.sats} satellites, need {min_sats}")
    if fix.lat_e7 == 0 and fix.lon_e7 == 0:
        raise NoGpsFix("latitude and longitude are both exactly zero")
    if not (-90.0 <= fix.lat <= 90.0 and -180.0 <= fix.lon <= 180.0):
        raise NoGpsFix(f"position out of range ({fix.lat}, {fix.lon})")
    if fix.course_deg > 360.0:
        raise NoGpsFix(f"course {fix.course_deg} degrees is out of range")


def encode_datagram(fix: GpsFix, t_src: float, seq: int) -> bytes:
    """One JSON object and a newline: byte for byte what
    `flight.cue_link.encode_record` writes for the same values."""
    row = {"lat": fix.lat, "lon": fix.lon, "alt_m_msl": fix.alt_m_msl,
           "vn": fix.vn, "ve": fix.ve, "vd": 0.0,
           "t_src": float(t_src), "seq": int(seq)}
    return (json.dumps(row) + "\n").encode()


# ------------------------------------------------------------------ the pump

class CueSource:
    """Bytes in, datagrams out, counters kept. `send` gets each datagram; it
    may raise OSError (Wi-Fi dropped), which is counted and reported, and the
    sequence number still advances so the receiver can see the gap."""

    def __init__(self, send: Callable[[bytes], None], *,
                 min_sats: int = DEFAULT_MIN_SATS,
                 clock: Callable[[], float] = time.monotonic,
                 err=None) -> None:
        self._send = send
        self._min_sats = int(min_sats)
        self._clock = clock
        self._err = err if err is not None else sys.stderr
        self.parser = CrsfParser()
        self.by_type: Counter = Counter()       # every frame, by type
        self.n_gps = 0                          # GPS frames seen
        self.n_sent = 0
        self.n_refused = 0                      # no fix: not sent
        self.n_gps_malformed = 0                # GPS frame of the wrong size
        self.n_send_errors = 0
        self.seq = 0
        self.last_fix: Optional[GpsFix] = None
        self.last_refusal: Optional[str] = None
        self.t_last_gps: Optional[float] = None
        self._t_last_loud = -1e9
        # snapshot for the per-second status line
        self._snap_t = clock()
        self._snap_types: Counter = Counter()
        self._snap = (0, 0, 0, 0, 0)

    def _loud(self, text: str, t: float) -> None:
        """Print a refusal, at most once a second so a long no-fix wait is
        readable. Every refusal is still counted."""
        if t - self._t_last_loud >= 1.0:
            self._t_last_loud = t
            print(f"!! NOT SENT: {text}", file=self._err, flush=True)

    def feed(self, data: bytes, t: Optional[float] = None) -> int:
        """Process the bytes of one read. Returns the datagrams sent."""
        t = self._clock() if t is None else float(t)
        sent = 0
        for frame in self.parser.feed(data, t):
            self.by_type[frame.type] += 1
            if frame.type != FRAME_TYPE_GPS:
                continue
            self.n_gps += 1
            self.t_last_gps = frame.t_read
            try:
                fix = decode_gps(frame.payload)
            except ValueError as exc:
                self.n_gps_malformed += 1
                self.last_refusal = str(exc)
                self._loud(f"malformed GPS frame ({exc})", t)
                continue
            try:
                check_fix(fix, self._min_sats)
            except NoGpsFix as exc:
                self.n_refused += 1
                self.last_refusal = str(exc)
                self._loud(f"no GPS fix ({exc})", t)
                continue
            self.last_fix = fix
            datagram = encode_datagram(fix, frame.t_read, self.seq)
            self.seq += 1
            try:
                self._send(datagram)
            except OSError as exc:
                self.n_send_errors += 1
                self.last_refusal = f"UDP send failed ({exc})"
                self._loud(self.last_refusal, t)
                continue
            self.n_sent += 1
            sent += 1
        return sent

    def status_line(self, now: Optional[float] = None) -> str:
        """One line covering the time since the previous call."""
        now = self._clock() if now is None else float(now)
        dt = max(now - self._snap_t, 1e-6)
        p = self.parser
        d_types = self.by_type - self._snap_types
        cur = (self.n_gps, self.n_sent, p.n_crc_fail, self.n_refused, p.n_bytes)
        d_gps, d_sent, d_crc, d_ref, d_bytes = (a - b for a, b in zip(cur, self._snap))
        self._snap_t, self._snap = now, cur
        self._snap_types = Counter(self.by_type)
        types = " ".join(
            f"{FRAME_NAMES.get(k, 'type')}(0x{k:02X})={v / dt:.1f}"
            for k, v in sorted(d_types.items())) or "none"
        parts = [f"frames/s: {types}",
                 f"GPS {d_gps / dt:.1f} Hz, sent {d_sent / dt:.1f} Hz "
                 f"(total {self.n_sent})",
                 f"CRC fail {d_crc} (total {p.n_crc_fail})"]
        if self.n_refused or self.n_gps_malformed or self.n_send_errors:
            parts.append(f"NOT SENT: no-fix {self.n_refused} (+{d_ref}), "
                         f"malformed {self.n_gps_malformed}, "
                         f"send errors {self.n_send_errors}")
        if d_bytes == 0:
            parts.append("NO BYTES ARRIVING")
        elif d_gps == 0:
            age = "never" if self.t_last_gps is None else f"{now - self.t_last_gps:.1f} s ago"
            parts.append(f"NO GPS FRAME (last: {age})")
        if self.last_fix is not None:
            f = self.last_fix
            parts.append(f"last {f.lat:.7f},{f.lon:.7f} {f.alt_m_msl:.0f} m "
                         f"{f.speed_ms:.1f} m/s {f.course_deg:05.1f} deg "
                         f"{f.sats} sats")
        return "[crsf] " + " | ".join(parts)

    def summary(self) -> dict:
        p = self.parser
        return {"bytes": p.n_bytes, "frames": p.n_frames,
                "frames_by_type": {f"0x{k:02X}": v for k, v in sorted(self.by_type.items())},
                "gps_frames": self.n_gps, "sent": self.n_sent,
                "refused_no_fix": self.n_refused,
                "gps_malformed": self.n_gps_malformed,
                "send_errors": self.n_send_errors,
                "crc_failures": p.n_crc_fail,
                "skipped_bytes": p.n_skipped_bytes,
                "pending_bytes_at_end": p.pending_bytes}


# ------------------------------------------------------------------- inputs
# Each yields (bytes, clock value when the read returned). An empty bytes
# object means "nothing this time"; the generator ending means end of input.

def read_stream(stream: BinaryIO, chunk: int = 256,
                clock: Callable[[], float] = time.monotonic
                ) -> Iterator[Tuple[bytes, float]]:
    while True:
        data = stream.read(chunk)
        if not data:
            return
        yield data, clock()


def read_serial(port: str, baud: int, dtr: Optional[bool],
                clock: Callable[[], float] = time.monotonic
                ) -> Iterator[Tuple[bytes, float]]:
    try:
        import serial   # pyserial, needed only for --port
    except ImportError:
        raise SystemExit("pyserial is not installed. In a Windows terminal: "
                         "py -m pip install pyserial")
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.05
    if dtr is not None:
        ser.dtr = dtr
    ser.open()
    try:
        while True:
            data = ser.read(1)                  # waits up to the timeout
            if data and ser.in_waiting:
                data += ser.read(ser.in_waiting)
            yield data, clock()
    finally:
        ser.close()


def read_udp(port: int, clock: Callable[[], float] = time.monotonic
             ) -> Iterator[Tuple[bytes, float]]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.05)
    try:
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                data = b""
            yield data, clock()
    finally:
        sock.close()


# --------------------------------------------------------------------- main

def _parse_dest(text: str) -> Tuple[str, int]:
    host, _, port = text.partition(":")
    if not host:
        raise argparse.ArgumentTypeError("--to needs HOST or HOST:PORT")
    return host, int(port) if port else DEFAULT_PORT


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Read CRSF telemetry, send the target's GPS to the Pi "
                    "over UDP (one datagram per GPS frame).")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port, e.g. COM7 (needs pyserial)")
    src.add_argument("--file", help="raw CRSF bytes from a file; '-' is stdin")
    src.add_argument("--udp-listen", type=int, metavar="PORT",
                     help="EXPERIMENTAL: raw bytes from UDP datagrams")
    ap.add_argument("--to", type=_parse_dest, metavar="HOST[:PORT]",
                    help=f"where to send (default port {DEFAULT_PORT})")
    ap.add_argument("--dry-run", action="store_true",
                    help="decode and report, send nothing")
    ap.add_argument("--baud", type=int, default=115200,
                    help="serial speed; ignored by a USB virtual port")
    ap.add_argument("--dtr", choices=("on", "off"), default=None,
                    help="force the serial DTR line (default: leave it alone)")
    ap.add_argument("--min-sats", type=int, default=DEFAULT_MIN_SATS,
                    help="refuse GPS frames with fewer satellites (default "
                         f"{DEFAULT_MIN_SATS}; never below 1)")
    ap.add_argument("--duration", type=float, default=None,
                    help="stop after this many seconds (default: until Ctrl-C "
                         "or end of file)")
    ap.add_argument("--record", metavar="PATH",
                    help="also append every byte read to this file")
    args = ap.parse_args(argv)
    if args.to is None and not args.dry_run:
        ap.error("give --to HOST[:PORT], or --dry-run to send nothing")
    if args.min_sats < 1:
        ap.error("--min-sats must be at least 1: a frame with zero "
                 "satellites is never sent")

    sock = None
    if args.dry_run:
        def send(_datagram: bytes) -> None:
            return None
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def send(datagram: bytes) -> None:
            sock.sendto(datagram, args.to)

    file_stream = None
    if args.port:
        reads = read_serial(args.port, args.baud,
                            None if args.dtr is None else args.dtr == "on")
        what = f"serial {args.port}"
    elif args.udp_listen is not None:
        reads = read_udp(args.udp_listen)
        what = f"UDP port {args.udp_listen} (experimental)"
    elif args.file == "-":
        reads = read_stream(sys.stdin.buffer)
        what = "stdin"
    else:
        file_stream = open(args.file, "rb")
        reads = read_stream(file_stream)
        what = f"file {args.file}"
    dest = "nowhere (dry run)" if args.dry_run else f"{args.to[0]}:{args.to[1]}"
    print(f"[crsf] reading {what}, sending to {dest}, "
          f"min satellites {args.min_sats}", flush=True)

    source = CueSource(send, min_sats=args.min_sats)
    rec = open(args.record, "ab") if args.record else None
    t_start = time.monotonic()
    t_status = t_start + 1.0
    try:
        for data, t in reads:
            if data:
                if rec is not None:
                    rec.write(data)
                source.feed(data, t)
            now = time.monotonic()
            if now >= t_status:
                print(source.status_line(now), flush=True)
                t_status = now + 1.0
            if args.duration is not None and now - t_start >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        reads.close()
        for f in (rec, file_stream, sock):
            if f is not None:
                f.close()

    print(source.status_line(), flush=True)
    result = source.summary()
    print(json.dumps(result), flush=True)
    if source.n_sent == 0:
        why = ("no GPS frame arrived" if source.n_gps == 0 else
               f"every GPS frame was refused or failed to send "
               f"(last reason: {source.last_refusal})")
        print(f"!! FAILED: nothing was sent -- {why}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
