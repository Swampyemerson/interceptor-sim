#!/usr/bin/env python3
"""FC->Pi serial link check — is the Pixhawk 6C Mini actually talking to the Pi?

WHY THIS EXISTS: `configs/px4_6cmini/README.md` §5/§5a has the builder HAND-MAKING
the 3-wire TELEM2 lead (JST-GH pin 2 = FC TX -> Pi header pin 10 / RXD, pin 3 = FC
RX -> Pi pin 8 / TXD, pin 6 = GND -> Pi pin 6). That cable is the one link in the
whole chain with no silkscreen, no self-test and no error message, and §5a step 11
says the only test that actually proves a serial lead is BYTES ARRIVING. Today that
test is `stty ... && timeout 3 cat /dev/ttyAMA0 | xxd | head` (README §6) — which
means the operator eyeballs a hexdump for `fd`/`fe` and decides. This script makes
that a MEASURED, fail-closed verdict instead: it counts real MAVLink frames and
HEARTBEATs and refuses to say PASS off zero of either. Steps `brn-03` / `brn-04`
(`docs/project_state.json` -> `build_tab` -> subsystem `brain`).

STANDARD LIBRARY ONLY, ON PURPOSE. The Pi is headless, reached over SSH, and we
cannot assume ANY package is installed on it — pyserial and mavsdk may be absent
and `.venv-seeker` may not exist yet at this point in the bring-up. So the port is
configured with `termios` (or `stty` as a documented fallback, `--use-stty`) and
the device file is read directly. `python3 scripts/seeker/fc_link_check.py` works
on a stock Raspberry Pi OS image with nothing installed.

WHAT IT PROVES / DOES NOT PROVE
  Proves: the FC->Pi DIRECTION (README §5a step 11 stage one: GND + FC TX only).
    Bytes here mean GND and FC TX are both correct and TELEM2 is open at this baud.
  Does NOT prove: the Pi->FC direction (the wire the Pi drives). That needs the FC
    to answer, i.e. README §6's MAVSDK `connected True`, or `brn-05`
    (`scripts/check_deploy_bench.sh`). A PASS here with a broken Pi TX wire is
    expected and normal — it is exactly what stage one of step 11 is for.

MAVLINK FRAMING (implemented here; no external mavlink library)
  v2: 0xFD | len | incompat_flags | compat_flags | seq | sysid | compid |
      msgid[3, little-endian] | payload[len] | checksum[2]   => 12 + len bytes,
      PLUS 13 more (link_id 1 + timestamp 6 + signature 6) when
      incompat_flags & 0x01 (the SIGNED bit) — PX4 can sign, so it is handled.
  v1: 0xFE | len | seq | sysid | compid | msgid[1] | payload[len] |
      checksum[2]                                            => 8 + len bytes.
  HEARTBEAT is msgid 0.

  *** WE DO NOT VALIDATE THE CRC, AND THAT IS A REAL LIMITATION. *** The MAVLink
  checksum needs a per-message CRC_EXTRA seed table that we deliberately do not
  carry (it is generated from the message XML; hard-coding a subset would be a
  fixture that silently rots). So framing is validated STRUCTURALLY: a candidate
  frame is only counted if the byte sitting where the NEXT frame's magic must
  start is itself a valid magic byte (0xFD/0xFE), or the stream ends exactly
  there. MAVLink frames are back-to-back on the wire with no inter-frame bytes, so
  that is a strong test — but resync-on-magic without a CRC can THEORETICALLY
  false-positive on random data. The defence is the verdict bar, not the parser:
  PASS demands >= MIN_CONSECUTIVE_FRAMES (3) frames that chain end-to-start AND
  >= 1 HEARTBEAT. Random bytes essentially never chain three deep (measured: 200
  bytes of `random.Random(1234)` contain a single 0xFD), and an unknown
  incompat_flags bit is a structural reject, which throws out most garbage
  candidates immediately.

VERDICTS (fail-closed; a count of zero is a verdict of its own, never a PASS)
  PASS      exit 0  >= 3 structurally-consecutive frames AND >= 1 HEARTBEAT.
  FAIL      exit 1  ZERO bytes arrived -> the diagnosis ladder from README §7.
  UNCERTAIN exit 2  bytes arrived but the verdict is not earned. Three flavours,
                    and the DISTINCTION is the diagnostic value:
                      - bytes but NO valid framing  -> characteristic of a BAUD
                        MISMATCH (the wire works, the bit rate does not). This is
                        NOT a wiring fault and must not be reported as FAIL.
                      - framing but < 3 consecutive -> marginal/corrupt stream.
                      - framing but 0 HEARTBEAT     -> link up, nothing useful.
  ERROR     exit 3  the device could not be opened (errno explained).

USAGE (on the Pi, over SSH)
    python3 scripts/seeker/fc_link_check.py                      # 3 s on /dev/ttyAMA0 @ 921600
    python3 scripts/seeker/fc_link_check.py --seconds 5 --device /dev/serial0
    python3 scripts/seeker/fc_link_check.py --check-port          # preconditions only, reads no bytes
    python3 scripts/seeker/fc_link_check.py --self-test           # offline, exits 0/1, no hardware

HONESTY BOUNDARY (CLAUDE.md): nothing here reads `gt_*` and nothing here feeds
guidance. This is a link instrument, and per the "instruments are evidence" rule a
bug in it would invalidate every bring-up decision made on it — hence the parser
is a PURE function (bytes -> stats) driven by `--self-test`, separate from the I/O.
"""
from __future__ import annotations

import argparse
import errno as errno_mod
import os
import select
import subprocess
import sys
import time

LOG = "[fc-link]"

# --- the wire, per configs/px4_6cmini/README.md §5 + bench.params -------------
# /dev/ttyAMA0 is the Pi 5 GPIO14/15 UART (README §5). `SER_TEL2_BAUD` = 921600.
DEF_DEVICE = "/dev/ttyAMA0"
DEF_BAUD = 921600
DEF_SECONDS = 3.0

# --- MAVLink framing constants ------------------------------------------------
MAGIC_V2 = 0xFD
MAGIC_V1 = 0xFE
MAGICS = (MAGIC_V2, MAGIC_V1)
HEARTBEAT_MSGID = 0
V2_HEADER_LEN = 10          # magic,len,incompat,compat,seq,sysid,compid,msgid[3]
V1_HEADER_LEN = 6           # magic,len,seq,sysid,compid,msgid[1]
CHECKSUM_LEN = 2
SIGNATURE_LEN = 13          # link_id(1) + timestamp(6) + signature(6)
INCOMPAT_SIGNED = 0x01
# Any OTHER incompat bit is unknown to us; MAVLink says such a frame MUST be
# dropped, and treating it as a structural reject is also our best cheap garbage
# filter in the absence of a CRC.
INCOMPAT_KNOWN_MASK = INCOMPAT_SIGNED

# THE PASS BAR. Three frames that chain end-to-start is what makes a CRC-less
# structural parse trustworthy (see the docstring). Lowering this weakens the only
# defence against a false positive on random data.
MIN_CONSECUTIVE_FRAMES = 3

DEF_HEXDUMP_BYTES = 64
# A read cap so a wedged loop cannot eat memory: 921600 baud is ~115 kB/s, so
# 8 MB is ~70 s of a saturated link — far past any sane --seconds.
MAX_CAPTURE_BYTES = 8 * 1024 * 1024

# A small, honest subset of msgid names — just enough that the breakdown reads as
# English to the builder. An unlisted id prints as `msgid N` (never a guess).
MSGID_NAMES = {
    0: "HEARTBEAT", 1: "SYS_STATUS", 2: "SYSTEM_TIME", 4: "PING",
    22: "PARAM_VALUE", 24: "GPS_RAW_INT", 27: "RAW_IMU", 29: "SCALED_PRESSURE",
    30: "ATTITUDE", 31: "ATTITUDE_QUATERNION", 32: "LOCAL_POSITION_NED",
    33: "GLOBAL_POSITION_INT", 36: "SERVO_OUTPUT_RAW", 65: "RC_CHANNELS",
    74: "VFR_HUD", 77: "COMMAND_ACK", 83: "ATTITUDE_TARGET",
    85: "POSITION_TARGET_LOCAL_NED", 105: "HIGHRES_IMU", 111: "TIMESYNC",
    140: "ACTUATOR_CONTROL_TARGET", 141: "ALTITUDE", 147: "BATTERY_STATUS",
    148: "AUTOPILOT_VERSION", 230: "ESTIMATOR_STATUS", 231: "WIND_COV",
    241: "VIBRATION", 245: "EXTENDED_SYS_STATE", 253: "STATUSTEXT",
    331: "ODOMETRY", 340: "UTM_GLOBAL_POSITION", 410: "OPEN_DRONE_ID_LOCATION",
}


def msgid_name(msgid):
    n = MSGID_NAMES.get(int(msgid))
    return f"{n} ({msgid})" if n else f"msgid {msgid}"


# ==========================================================================
# PURE PARSER — bytes in, stats out. No I/O, no clock, no globals. The
# self-test drives THIS, which is the whole reason it is separated out.
# ==========================================================================

class FrameStats:
    """Structural MAVLink framing stats for one captured byte stream."""

    __slots__ = ("n_bytes", "n_frames", "max_consecutive", "n_heartbeats",
                 "n_v1", "n_v2", "n_signed", "n_skipped_bytes", "msgid_counts",
                 "id_counts", "truncated_tail", "tail_bytes", "n_rejected")

    def __init__(self):
        self.n_bytes = 0
        self.n_frames = 0          # frames ACCEPTED (structurally well-formed)
        self.max_consecutive = 0   # longest run of frames that chain end->start
        self.n_heartbeats = 0
        self.n_v1 = 0
        self.n_v2 = 0
        self.n_signed = 0
        self.n_skipped_bytes = 0   # bytes discarded while resyncing
        self.msgid_counts = {}     # msgid -> count
        self.id_counts = {}        # (sysid, compid) -> count
        self.truncated_tail = False   # stream ended mid-frame (normal, expected)
        self.tail_bytes = 0
        self.n_rejected = 0        # magic-looking candidates that failed structure

    # --- derived, and deliberately None-when-unmeasurable ------------------
    def frame_hz(self, elapsed_s):
        if not elapsed_s or elapsed_s <= 0:
            return None
        return self.n_frames / float(elapsed_s)

    def heartbeat_hz(self, elapsed_s):
        if not elapsed_s or elapsed_s <= 0:
            return None
        return self.n_heartbeats / float(elapsed_s)

    def top_msgids(self, k=8):
        return sorted(self.msgid_counts.items(), key=lambda kv: (-kv[1], kv[0]))[:k]

    def sysid_pairs(self):
        return sorted(self.id_counts.items(), key=lambda kv: (-kv[1], kv[0]))


def _candidate_frame(data, i):
    """Try to read ONE frame starting at data[i] (which must be a magic byte).

    Returns (total_len, msgid, sysid, compid, version, signed) or None when the
    bytes at `i` cannot be a frame HEADER we accept (unknown incompat bit) —
    (None, "short") when there simply are not enough bytes left to hold the frame
    (a truncated tail, which is the normal end of any timed capture, not garbage).
    """
    n = len(data)
    magic = data[i]
    if magic == MAGIC_V2:
        if n - i < V2_HEADER_LEN:
            return "short"
        length = data[i + 1]
        incompat = data[i + 2]
        if incompat & ~INCOMPAT_KNOWN_MASK:
            return None          # unknown incompat bit -> MUST be dropped
        signed = bool(incompat & INCOMPAT_SIGNED)
        sysid, compid = data[i + 5], data[i + 6]
        msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
        total = V2_HEADER_LEN + length + CHECKSUM_LEN
        if signed:
            total += SIGNATURE_LEN
        if n - i < total:
            return "short"
        return (total, msgid, sysid, compid, 2, signed)
    if magic == MAGIC_V1:
        if n - i < V1_HEADER_LEN:
            return "short"
        length = data[i + 1]
        sysid, compid = data[i + 3], data[i + 4]
        msgid = data[i + 5]
        total = V1_HEADER_LEN + length + CHECKSUM_LEN
        if n - i < total:
            return "short"
        return (total, msgid, sysid, compid, 1, False)
    return None


def parse_stream(data):
    """Count structurally-valid MAVLink frames in `data`. PURE: same bytes in,
    same stats out, no clock and no I/O.

    A candidate frame is ACCEPTED only when the byte where the next frame's magic
    must sit is itself a magic byte, or the frame ends exactly at the end of the
    stream. See the module docstring for why this replaces CRC validation and how
    the >=3-consecutive verdict bar covers the gap.
    """
    if data is None:
        data = b""
    st = FrameStats()
    st.n_bytes = len(data)
    n = len(data)
    i = 0
    run = 0            # current chain length (frames whose ends touch)
    expect_at = None   # offset the current chain expects the next frame at
    while i < n:
        if data[i] not in MAGICS:
            i += 1
            st.n_skipped_bytes += 1
            run, expect_at = 0, None
            continue
        cand = _candidate_frame(data, i)
        if cand == "short":
            # Not enough bytes left for the frame this header claims. That is the
            # normal end of a TIMED capture (we cut the stream mid-frame), so it
            # is recorded, never counted as a frame and never as garbage.
            st.truncated_tail = True
            st.tail_bytes = n - i
            break
        if cand is None:
            i += 1
            st.n_skipped_bytes += 1
            st.n_rejected += 1
            run, expect_at = 0, None
            continue
        total, msgid, sysid, compid, version, signed = cand
        end = i + total
        ok = (end == n) or (data[end] in MAGICS)
        if not ok:
            i += 1
            st.n_skipped_bytes += 1
            st.n_rejected += 1
            run, expect_at = 0, None
            continue
        # accepted
        st.n_frames += 1
        if version == 2:
            st.n_v2 += 1
        else:
            st.n_v1 += 1
        if signed:
            st.n_signed += 1
        if msgid == HEARTBEAT_MSGID:
            st.n_heartbeats += 1
        st.msgid_counts[msgid] = st.msgid_counts.get(msgid, 0) + 1
        key = (sysid, compid)
        st.id_counts[key] = st.id_counts.get(key, 0) + 1
        run = run + 1 if expect_at == i else 1
        st.max_consecutive = max(st.max_consecutive, run)
        expect_at = end
        i = end
    return st


# ==========================================================================
# VERDICT — fail-closed. Pure function of (stats, elapsed). No I/O.
# ==========================================================================

class Verdict:
    __slots__ = ("name", "exit_code", "reason", "ladder")

    def __init__(self, name, exit_code, reason, ladder=()):
        self.name = name
        self.exit_code = int(exit_code)
        self.reason = reason
        self.ladder = list(ladder)

    def __repr__(self):   # pragma: no cover - debugging aid
        return f"Verdict({self.name}, exit={self.exit_code}, {self.reason!r})"


# The zero-bytes ladder, in LIKELIHOOD ORDER, straight from
# configs/px4_6cmini/README.md §7 item 1 (+ §5a step 11's precondition note).
ZERO_BYTE_LADDER = (
    "1. WRONG DEVICE NODE. On a Pi 5 the GPIO14/15 UART is /dev/ttyAMA0; check "
    "`ls -l /dev/serial0` and use whatever the symlink resolves to. "
    "(`--check-port` reports this for you.)",
    "2. TX/RX NOT CROSSED. FC TELEM2 pin 2 (FC TX) must land on Pi header pin 10 "
    "(GPIO15/RXD), and FC pin 3 (FC RX) on Pi pin 8 (GPIO14/TXD). README §5. "
    "Moving the FC TX wire to a neighbouring Pi pin is harmless (§5a step 11).",
    "3. THE PI'S LOGIN CONSOLE STILL HOLDS THE PORT. Check `console=serial0` (or "
    "console=ttyAMA0) in /boot/firmware/cmdline.txt and "
    "`systemctl is-active serial-getty@ttyAMA0.service`. Fix with "
    "`sudo raspi-config` -> Interface Options -> Serial Port: login shell NO, "
    "serial hardware YES, then REBOOT. README §5.",
    "4. BAUD MISMATCH. TELEM2 is 921600 (SER_TEL2_BAUD). A mismatch usually gives "
    "GARBAGE BYTES rather than silence, so this is below the wiring causes — but "
    "a wildly wrong rate can also yield nothing readable.",
    "5. bench.params WAS NEVER LOADED, so MAV_1_CONFIG=102 never opened TELEM2. "
    "It needs a REBOOT to take effect (README §3 / §5a step 11 precondition). "
    "Also re-check MAV_1_FLOW_CTRL=0: the default (2=auto) opens TELEM2 with "
    "CTS/RTS and our 3-wire lead leaves CTS floating. NOTE: re-applying an "
    "airframe in QGC WIPES these — reload bench.params afterwards (README §2).",
    "6. THE FC IS NOT POWERED / NOT BOOTED, or GND is not actually joined. GND "
    "(TELEM2 pin 6 -> Pi header pin 6) is the wire that makes the other two mean "
    "anything; without it a 'signal' is measured against a floating reference.",
)

BAUD_GARBAGE_LADDER = (
    "1. BAUD MISMATCH — this is the classic fingerprint. Bytes are arriving, so "
    "GND and the FC TX wire are FINE; the two ends just disagree on the bit rate. "
    "TELEM2 should be 921600 (SER_TEL2_BAUD). Re-run with --baud on the other "
    "candidates (57600 is PX4's other common telemetry rate) and see which one "
    "produces frames.",
    "2. The port was configured by something else after we set it (a getty, a "
    "leftover mavproxy/mavsdk_server holding /dev/ttyAMA0). Check "
    "`systemctl is-active serial-getty@ttyAMA0.service` and `fuser -v DEVICE`.",
    "3. Electrical noise on an unshielded Dupont lead at 921600 — README §5a "
    "step 5 says keep the finished lead under ~25 cm. A long/flapping lead "
    "corrupts bytes without stopping them.",
    "4. NOT MAVLink at all: something else is driving that TX line (a serial "
    "console spewing boot text is text, not 0xFD/0xFE frames).",
)


def classify(stats, elapsed_s=None, device=None, baud=None):
    """(Verdict) from framing stats. FAIL-CLOSED: PASS requires >= 3
    structurally-consecutive frames AND >= 1 HEARTBEAT. Every other outcome that
    saw bytes is UNCERTAIN, because 'bytes but no frames' and 'no bytes at all'
    point at DIFFERENT faults and collapsing them destroys the diagnosis."""
    where = f"{device or 'device'} @ {baud or '?'} baud"
    if stats.n_bytes == 0:
        return Verdict(
            "FAIL", 1,
            f"ZERO bytes arrived from {where}. The FC->Pi direction is NOT "
            f"working (or nothing is transmitting).",
            ZERO_BYTE_LADDER)
    if stats.n_frames == 0:
        return Verdict(
            "UNCERTAIN", 2,
            f"{stats.n_bytes} bytes arrived from {where} but NOT ONE valid "
            f"MAVLink frame could be structurally parsed "
            f"({stats.n_rejected} magic-looking candidates rejected). Bytes "
            f"without framing is the signature of a BAUD MISMATCH, not of a "
            f"wiring fault -- so this is deliberately NOT called FAIL.",
            BAUD_GARBAGE_LADDER)
    if stats.max_consecutive < MIN_CONSECUTIVE_FRAMES:
        return Verdict(
            "UNCERTAIN", 2,
            f"only {stats.max_consecutive} structurally-CONSECUTIVE frame(s) "
            f"(bar is {MIN_CONSECUTIVE_FRAMES}) in {stats.n_bytes} bytes from "
            f"{where}. Without CRC validation a short chain is not trustworthy "
            f"evidence of a MAVLink stream (see the module docstring), so this "
            f"is NOT a PASS. Most likely a corrupt/marginal stream or a "
            f"too-short capture -- re-run with a longer --seconds.",
            BAUD_GARBAGE_LADDER)
    if stats.n_heartbeats == 0:
        return Verdict(
            "UNCERTAIN", 2,
            f"{stats.n_frames} valid frames from {where} but ZERO HEARTBEAT "
            f"(msgid 0). The link is UP and MAVLink is flowing, yet the one "
            f"message that proves a live autopilot on the other end never "
            f"arrived -- 'link up, nothing useful'. NOT a PASS.",
            ("1. Capture longer (--seconds 5): HEARTBEAT is ~1 Hz, so a very "
             "short window can genuinely miss it. This is the first thing to "
             "rule out.",
             "2. MAV_1_MODE / MAV_1_RATE on the FC: the Onboard stream set is "
             "expected (bench.params sets MAV_1_MODE=2). A custom/odd mode could "
             "in principle suppress it.",
             "3. Frames are being MIS-FRAMED (we count structure, not CRC), so a "
             "corrupt stream can yield 'frames' with nonsense msgids. Check the "
             "msgid breakdown above: if it is a scatter of unrelated ids with no "
             "repetition, treat this as the baud-mismatch case."))
    return Verdict(
        "PASS", 0,
        f"{stats.n_frames} MAVLink frames ({stats.max_consecutive} "
        f"structurally-consecutive at best) including {stats.n_heartbeats} "
        f"HEARTBEAT from {where}. The FC->Pi direction is WORKING: GND and the "
        f"FC TX wire (TELEM2 pin 2 -> Pi header pin 10) are both correct and "
        f"TELEM2 is open at this baud. This does NOT prove the Pi->FC direction "
        f"-- for that, run README §6's MAVSDK check or scripts/check_deploy_bench.sh.")


# ==========================================================================
# Reporting helpers
# ==========================================================================

def hexdump(data, limit=DEF_HEXDUMP_BYTES, prefix=LOG + "   "):
    """Classic offset/hex/ascii dump of the first `limit` bytes, as a list of
    lines. Empty input yields a single explicit 'no bytes' line -- never a blank
    that could read as 'looks fine'."""
    if not data:
        return [f"{prefix}(no bytes at all)"]
    chunk = data[:limit]
    lines = []
    for off in range(0, len(chunk), 16):
        row = chunk[off:off + 16]
        hexes = " ".join(f"{b:02x}" for b in row)
        ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        lines.append(f"{prefix}{off:04x}  {hexes:<47}  |{ascii_}|")
    if len(data) > len(chunk):
        lines.append(f"{prefix}... ({len(data) - len(chunk)} more bytes)")
    return lines


def _hz(v):
    return "UNMEASURED" if v is None else f"{v:.2f} Hz"


def report(stats, elapsed_s, data=b"", hexdump_bytes=DEF_HEXDUMP_BYTES):
    """Print the measurement block. Rates print as UNMEASURED rather than 0.00
    when the window is unknown -- a fabricated denominator is exactly the
    'substitute a default for a measured number' failure CLAUDE.md forbids."""
    win = "UNMEASURED" if not elapsed_s or elapsed_s <= 0 else f"{elapsed_s:.3f} s"
    print(f"{LOG} capture window: {win} (first byte -> end of listening)")
    print(f"{LOG} bytes: {stats.n_bytes}   frames: {stats.n_frames} "
          f"(v2 {stats.n_v2} / v1 {stats.n_v1}, signed {stats.n_signed})")
    print(f"{LOG} longest structurally-CONSECUTIVE run: {stats.max_consecutive} "
          f"(PASS bar {MIN_CONSECUTIVE_FRAMES})")
    print(f"{LOG} HEARTBEAT (msgid 0): {stats.n_heartbeats}")
    print(f"{LOG} rates: frames {_hz(stats.frame_hz(elapsed_s))} | "
          f"HEARTBEAT {_hz(stats.heartbeat_hz(elapsed_s))}")
    print(f"{LOG} resync: {stats.n_skipped_bytes} bytes skipped, "
          f"{stats.n_rejected} candidates rejected"
          + (f", tail truncated mid-frame ({stats.tail_bytes} bytes, normal for "
             f"a timed capture)" if stats.truncated_tail else ""))
    pairs = stats.sysid_pairs()
    if pairs:
        print(f"{LOG} distinct (sysid, compid): {len(pairs)} -> "
              + ", ".join(f"({s},{c}) x{n}" for (s, c), n in pairs))
    else:
        print(f"{LOG} distinct (sysid, compid): 0")
    top = stats.top_msgids()
    if top:
        print(f"{LOG} top msgids: "
              + ", ".join(f"{msgid_name(m)} x{n}" for m, n in top))
    else:
        print(f"{LOG} top msgids: none (no frames parsed)")
    if stats.n_frames == 0 and stats.n_bytes:
        print(f"{LOG} first {min(hexdump_bytes, stats.n_bytes)} bytes "
              f"(no framing found -- read this for boot text vs. random garbage):")
        for line in hexdump(data, hexdump_bytes):
            print(line)


def announce(verdict):
    print(f"{LOG} {verdict.name}: {verdict.reason}")
    if verdict.ladder:
        print(f"{LOG} diagnosis, in likelihood order "
              f"(configs/px4_6cmini/README.md §7):")
        for item in verdict.ladder:
            print(f"{LOG}   {item}")


# ==========================================================================
# I/O — termios port config + a bounded raw read. Kept OUT of the parser.
# ==========================================================================

class PortError(Exception):
    """Device could not be opened/configured -> exit 3, with the errno explained."""


def _baud_const(baud):
    import termios
    return getattr(termios, f"B{int(baud)}", None)


def _baud_from_const(const):
    """Reverse the termios Bxxxx constant -> integer baud, or None."""
    import termios
    for name in dir(termios):
        if name.startswith("B") and name[1:].isdigit():
            if getattr(termios, name) == const:
                return int(name[1:])
    return None


def explain_open_errno(exc, device):
    """Plain-English cause for an open() failure, per README §7's ladder."""
    e = getattr(exc, "errno", None)
    if e == errno_mod.ENOENT:
        return (f"{device} DOES NOT EXIST (ENOENT). Wrong device node: on a Pi 5 "
                f"the GPIO14/15 UART is /dev/ttyAMA0. Run `ls -l /dev/serial0` "
                f"and use what it resolves to; if there is no /dev/serial0 either, "
                f"`enable_uart=1` is missing from /boot/firmware/config.txt (needs "
                f"a reboot). Try --check-port.")
    if e in (errno_mod.EACCES, errno_mod.EPERM):
        return (f"PERMISSION DENIED on {device} (EACCES). Your user is almost "
                f"certainly not in the `dialout` group. Fix: "
                f"`sudo usermod -aG dialout $USER` then LOG OUT and back in "
                f"(group membership is applied at login, so `newgrp dialout` or a "
                f"fresh ssh session is required). `sudo` also works as a one-off.")
    if e == errno_mod.EBUSY:
        return (f"{device} is BUSY (EBUSY) -- something else holds it. Check "
                f"`systemctl is-active serial-getty@ttyAMA0.service` and "
                f"`fuser -v {device}`.")
    if e == errno_mod.ENXIO:
        return (f"{device} exists but there is NO DEVICE behind it (ENXIO) -- the "
                f"UART is not enabled in the boot config. Check `enable_uart=1` in "
                f"/boot/firmware/config.txt and reboot.")
    return f"could not open {device}: {type(exc).__name__}: {exc}"


def fd_name(fd):
    """The device path behind an fd, for an error message. Falls back to the fd
    number -- an error message must never be the thing that raises."""
    try:
        return os.readlink(f"/proc/self/fd/{fd}")
    except OSError:
        return f"fd {fd}"


def configure_port_termios(fd, baud):
    """Put `fd` in RAW 8N1 at `baud` with hardware flow control OFF.

    Flow control OFF is not a detail: the hand-made lead is 3-wire, so CTS floats
    (README §7 item 2 / bench.params MAV_1_FLOW_CTRL=0). Leaving CRTSCTS on can
    stall the port for half a second at a time and look like a dead link.
    """
    import termios
    const = _baud_const(baud)
    if const is None:
        raise PortError(
            f"baud {baud} has no termios constant on this platform (available: "
            f"{sorted(int(n[1:]) for n in dir(termios) if n.startswith('B') and n[1:].isdigit())}). "
            f"Use --use-stty to configure the port with `stty` instead.")
    # termios.error is NOT an OSError subclass, so it must be named explicitly --
    # measured 2026-07-29: pointing --device at a regular file crashed with a raw
    # `termios.error: (25, 'Inappropriate ioctl for device')` traceback and exit 1,
    # i.e. a NON-tty looked like a FAIL verdict. Exit 1 means "zero bytes on a good
    # port"; a port we never configured must be exit 3 (nothing was measured).
    try:
        attrs = termios.tcgetattr(fd)
    except (termios.error, OSError) as exc:
        raise PortError(
            f"{fd_name(fd)} is NOT A SERIAL PORT (tcgetattr: "
            f"{type(exc).__name__}: {exc}). A regular file, pipe or socket cannot "
            f"be configured as a UART -- check the --device path.") from exc
    iflag, oflag, cflag, lflag, _ispeed, _ospeed, cc = attrs
    # raw input: no translation, no software flow control, no parity fixups
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
               | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON
               | termios.IXOFF | termios.INPCK)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG
               | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    crtscts = getattr(termios, "CRTSCTS", 0)
    if crtscts:
        cflag &= ~crtscts        # 3-wire lead: CTS floats, flow control MUST be off
    cc = list(cc)
    cc[termios.VMIN] = 0         # non-blocking reads; select() does the waiting
    cc[termios.VTIME] = 0
    try:
        termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag,
                                                const, const, cc])
        # drop stale bytes queued before we set the baud
        termios.tcflush(fd, termios.TCIFLUSH)
    except (termios.error, OSError) as exc:
        raise PortError(
            f"could not apply raw 8N1 @ {baud} to {fd_name(fd)} "
            f"({type(exc).__name__}: {exc}). Try --use-stty.") from exc


def configure_port_stty(device, baud):
    """DOCUMENTED FALLBACK for a platform whose termios lacks the baud constant
    (or when --use-stty is passed). `stty` ships on every Pi OS image. Same
    settings as configure_port_termios: raw 8N1, no flow control."""
    cmd = ["stty", "-F", device, str(int(baud)), "raw", "-echo", "-crtscts",
           "cs8", "-parenb", "-cstopb", "min", "0", "time", "0"]
    try:
        p = subprocess.run(cmd, capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise PortError(f"`stty` not found, cannot configure {device}: {exc}")
    if p.returncode != 0:
        raise PortError(f"`{' '.join(cmd)}` failed (rc={p.returncode}): "
                        f"{(p.stderr or p.stdout).strip()}")


def read_raw(device, baud, seconds, use_stty=False, max_bytes=MAX_CAPTURE_BYTES):
    """Open + configure `device` and read raw bytes for `seconds` (bounded).

    Returns (data, elapsed_s) where elapsed_s is the window from the FIRST BYTE
    received to the end of listening -- the honest denominator for a rate. When no
    byte ever arrives, elapsed_s is None (there is no window, and inventing one
    would manufacture a 0.00 Hz that looks measured). Raises PortError.
    """
    try:
        fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as exc:
        raise PortError(explain_open_errno(exc, device)) from exc
    try:
        if use_stty:
            configure_port_stty(device, baud)
        else:
            try:
                configure_port_termios(fd, baud)
            except PortError:
                raise
            except Exception as exc:
                # Backstop: ANY configure failure must surface as a PortError
                # (exit 3 = nothing was measured), never as a traceback and never
                # as an exit code that could be read as a link verdict.
                raise PortError(
                    f"{device} could not be configured as a serial port "
                    f"({type(exc).__name__}: {exc}). Is it really a tty? "
                    f"`ls -l {device}`; try --use-stty.") from exc
        chunks = []
        total = 0
        t_first = None
        t_start = time.monotonic()
        deadline = t_start + float(seconds)
        while True:
            now = time.monotonic()
            if now >= deadline or total >= max_bytes:
                break
            r, _w, _x = select.select([fd], [], [], max(0.0, deadline - now))
            if not r:
                continue
            try:
                buf = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError as exc:
                print(f"{LOG} WARNING: read error after {total} bytes "
                      f"({type(exc).__name__}: {exc}) -- stopping the capture",
                      file=sys.stderr)
                break
            if not buf:
                continue
            if t_first is None:
                t_first = time.monotonic()
            chunks.append(buf)
            total += len(buf)
        t_end = time.monotonic()
        data = b"".join(chunks)
        elapsed = (t_end - t_first) if t_first is not None else None
        if elapsed is not None and elapsed <= 0:
            elapsed = None
        return data, elapsed
    finally:
        os.close(fd)


# ==========================================================================
# --check-port : the PRECONDITIONS, without reading a byte
# ==========================================================================

def _read_text(path):
    """File contents or None. Guarded so every one of these degrades off-Pi."""
    try:
        with open(path, "r", errors="replace") as f:
            return f.read()
    except OSError:
        return None


def _first_existing(paths):
    for p in paths:
        if os.path.exists(p):
            return p
    return None


def _systemctl_is_active(unit):
    """('active'|'inactive'|'unknown-...'|None) -- None when systemctl is absent
    (i.e. not a systemd box, so the question is unanswerable here)."""
    try:
        p = subprocess.run(["systemctl", "is-active", unit],
                           capture_output=True, text=True, timeout=10)
    except (FileNotFoundError, subprocess.SubprocessError):
        return None
    return (p.stdout or p.stderr or "").strip() or None


def _current_baud(device):
    """(baud_int_or_None, note). Reads the device's CURRENT termios speed without
    changing it. Never raises."""
    try:
        import termios
    except ImportError:            # pragma: no cover - non-POSIX
        return None, "termios unavailable on this platform"
    try:
        fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as exc:
        return None, explain_open_errno(exc, device)
    try:
        attrs = termios.tcgetattr(fd)
    except OSError as exc:
        return None, f"not a tty / tcgetattr failed ({type(exc).__name__}: {exc})"
    finally:
        os.close(fd)
    ospeed = attrs[5]
    baud = _baud_from_const(ospeed)
    return baud, ("from tcgetattr(ospeed)" if baud
                  else f"unrecognised termios speed constant {ospeed}")


def _in_dialout():
    """(bool_or_None, detail) -- whether this user is in the device group that
    matters. None when the group does not exist on this system."""
    try:
        import grp
    except ImportError:            # pragma: no cover
        return None, "grp unavailable"
    try:
        g = grp.getgrnam("dialout")
    except KeyError:
        return None, "no `dialout` group on this system"
    try:
        gids = set(os.getgroups()) | {os.getgid()}
    except OSError:                # pragma: no cover
        gids = set()
    return (g.gr_gid in gids), f"dialout gid={g.gr_gid}"


def check_port(device, baud):
    """Report the things that make the byte test INVALID, reading no bytes.

    Every check returns one of OK / BAD / UNKNOWN, and UNKNOWN is never counted as
    a pass. Exit 0 only when nothing is BAD and the device actually exists; exit 2
    otherwise (including off-Pi, where the honest answer is 'this check cannot
    tell you anything here')."""
    bad, unknown = [], []

    def emit(state, label, detail):
        print(f"{LOG}   [{state:<7}] {label}: {detail}")
        if state == "BAD":
            bad.append(label)
        elif state == "UNKNOWN":
            unknown.append(label)

    print(f"{LOG} --check-port: preconditions for the FC->Pi byte test "
          f"(no bytes are read)")

    # 0. is this even a Pi? (decides how to read every UNKNOWN below)
    is_pi = False
    model = _read_text("/proc/device-tree/model")
    if model:
        model = model.replace("\x00", "").strip()
        is_pi = "raspberry pi" in model.lower()
    emit("OK" if is_pi else "UNKNOWN", "board model",
         model if model else "no /proc/device-tree/model (not a Raspberry Pi, or "
                             "no device tree) -- this whole check is only "
                             "meaningful ON the Pi")

    # 1. the device node
    if os.path.exists(device):
        try:
            st = os.stat(device)
            import stat as stat_mod
            kind = "character device" if stat_mod.S_ISCHR(st.st_mode) else "NOT a character device"
            emit("OK" if stat_mod.S_ISCHR(st.st_mode) else "BAD", f"{device} exists",
                 f"{kind}, mode {stat_mod.filemode(st.st_mode)}")
        except OSError as exc:
            emit("BAD", f"{device} exists", f"stat failed: {exc}")
    else:
        emit("BAD", f"{device} exists",
             "MISSING. Wrong node, or the UART is not enabled (see enable_uart "
             "below). On a Pi 5 the GPIO14/15 UART is /dev/ttyAMA0.")

    # 2. /dev/serial0 symlink target -- the README's own first diagnostic
    if os.path.islink("/dev/serial0"):
        try:
            tgt = os.readlink("/dev/serial0")
            real = os.path.realpath("/dev/serial0")
            same = os.path.realpath(device) == real if os.path.exists(device) else False
            emit("OK" if same else "UNKNOWN", "/dev/serial0 symlink",
                 f"-> {tgt} (resolves to {real})"
                 + ("" if same else f" -- this is NOT the --device you gave "
                                    f"({device}). Use the resolved node."))
        except OSError as exc:
            emit("UNKNOWN", "/dev/serial0 symlink", f"readlink failed: {exc}")
    elif os.path.exists("/dev/serial0"):
        emit("OK", "/dev/serial0", "exists but is not a symlink")
    else:
        emit("UNKNOWN" if not is_pi else "BAD", "/dev/serial0 symlink",
             "absent. On Pi OS this symlink is created when the UART is enabled; "
             "its absence usually means enable_uart=1 is missing (needs a reboot).")

    # 3. is the login console holding the port?
    cmdline_path = _first_existing(["/boot/firmware/cmdline.txt", "/boot/cmdline.txt"])
    cmdline = _read_text(cmdline_path) if cmdline_path else None
    if cmdline is None:
        emit("UNKNOWN", "cmdline.txt console=",
             "no /boot/firmware/cmdline.txt (or /boot/cmdline.txt) -- cannot tell "
             "whether the kernel console is on the serial port from here")
    else:
        toks = [t for t in cmdline.split() if t.startswith("console=")]
        serial_consoles = [t for t in toks
                           if "serial0" in t or "ttyAMA" in t or "ttyS" in t]
        emit("BAD" if serial_consoles else "OK", f"{cmdline_path} console=",
             (f"KERNEL CONSOLE IS ON THE SERIAL PORT: {' '.join(serial_consoles)} "
              f"-- remove it (`sudo raspi-config` -> Interface Options -> Serial "
              f"Port -> login shell NO) and REBOOT, or the console will fight the "
              f"FC for this UART."
              if serial_consoles else
              f"no serial console entry ({' '.join(toks) if toks else 'no console= at all'})"))

    # 4. the getty
    unit = "serial-getty@" + os.path.basename(device) + ".service"
    state = _systemctl_is_active(unit)
    if state is None:
        emit("UNKNOWN", unit, "systemctl unavailable here -- cannot tell")
    else:
        emit("BAD" if state == "active" else "OK", unit,
             (f"{state} -- A LOGIN SHELL IS HOLDING {device}. "
              f"`sudo systemctl disable --now {unit}` (raspi-config does this)."
              if state == "active" else state))

    # 5. enable_uart
    config_path = _first_existing(["/boot/firmware/config.txt", "/boot/config.txt"])
    config = _read_text(config_path) if config_path else None
    if config is None:
        emit("UNKNOWN", "config.txt enable_uart",
             "no /boot/firmware/config.txt (or /boot/config.txt) -- cannot tell")
    else:
        lines = [ln.strip() for ln in config.splitlines()]
        hits = [ln for ln in lines
                if ln.replace(" ", "").startswith("enable_uart=")
                and not ln.startswith("#")]
        on = any(ln.replace(" ", "").endswith("=1") for ln in hits)
        emit("OK" if on else "BAD", f"{config_path} enable_uart",
             ("enable_uart=1" if on else
              (f"present but not 1: {hits}" if hits else
               "MISSING -- add `enable_uart=1` and REBOOT (README §5).")))

    # 6. permissions
    if os.path.exists(device):
        readable = os.access(device, os.R_OK)
        in_grp, grp_detail = _in_dialout()
        emit("OK" if readable else "BAD", f"read permission on {device}",
             (f"readable by uid {os.getuid()}" if readable else
              f"NOT readable by uid {os.getuid()} -- "
              f"`sudo usermod -aG dialout $USER` then log out and back in")
             + f" ({grp_detail}, member={in_grp})")
    else:
        emit("UNKNOWN", f"read permission on {device}", "device absent")

    # 7. the port's CURRENT baud
    if os.path.exists(device):
        cur, note = _current_baud(device)
        if cur is None:
            emit("UNKNOWN", "current termios baud", note)
        else:
            emit("OK" if cur == int(baud) else "UNKNOWN", "current termios baud",
                 f"{cur} ({note})"
                 + ("" if cur == int(baud) else
                    f" -- differs from --baud {baud}; harmless, the check sets "
                    f"the baud itself before reading, but a surprising value here "
                    f"means something else configured this port"))
    else:
        emit("UNKNOWN", "current termios baud", "device absent")

    print(f"{LOG} --check-port summary: {len(bad)} BAD, {len(unknown)} UNKNOWN")
    if bad:
        print(f"{LOG} PRECONDITIONS NOT SATISFIED: {', '.join(bad)}. Fix these "
              f"before trusting (or debugging) a byte capture.")
        return 2
    if unknown:
        print(f"{LOG} PRECONDITIONS UNDETERMINED ({', '.join(unknown)}) -- an "
              f"UNKNOWN is NOT an OK. Off a Raspberry Pi this is the expected "
              f"result and the check simply cannot answer the question here.")
        return 2
    print(f"{LOG} preconditions OK -- nothing found that would invalidate the "
          f"byte test.")
    return 0


# ==========================================================================
# Live check
# ==========================================================================

def run_check(args):
    # flush=True: stdout is block-buffered when piped/tee'd while stderr is not,
    # so without it an ERROR line printed to stderr lands ABOVE the "listening"
    # line in a captured bench log (measured 2026-07-29) -- which reads as if the
    # error preceded the attempt.
    print(f"{LOG} listening on {args.device} @ {args.baud} baud for "
          f"{args.seconds:.1f} s (FC->Pi direction only; "
          f"configs/px4_6cmini/README.md §5a step 11)", flush=True)
    try:
        data, elapsed = read_raw(args.device, args.baud, args.seconds,
                                 use_stty=args.use_stty)
    except PortError as exc:
        print(f"{LOG} ERROR: {exc}", file=sys.stderr)
        print(f"{LOG} ERROR (exit 3): the port could not be opened/configured, so "
              f"NOTHING about the link was measured. This is not a FAIL and not a "
              f"PASS. Run `--check-port` for the full precondition list.",
              file=sys.stderr)
        return 3
    stats = parse_stream(data)
    report(stats, elapsed, data, args.hexdump_bytes)
    v = classify(stats, elapsed, args.device, args.baud)
    announce(v)
    if v.name == "PASS":
        print(f"{LOG} next: README §5a step 11 stage two -- add the third wire "
              f"(FC RX <- Pi header pin 8) and run README §6's MAVSDK check, then "
              f"scripts/check_deploy_bench.sh for brn-05.")
    return v.exit_code


# ==========================================================================
# SELF-TEST — synthesizes byte streams, drives the PURE parser + verdict.
# Runs on the WSL dev box; opens no device, touches no hardware.
# ==========================================================================

def _mk_v2(msgid=HEARTBEAT_MSGID, seq=0, sysid=1, compid=1, payload=None,
           signed=False, incompat=None):
    """One synthetic MAVLink v2 frame. The checksum bytes are ARBITRARY on
    purpose -- this parser validates structure, not CRC (see the docstring), so a
    fixture with a real CRC would test something the code does not claim."""
    if payload is None:
        payload = bytes(9)          # HEARTBEAT payload is 9 bytes
    inc = (INCOMPAT_SIGNED if signed else 0x00) if incompat is None else incompat
    frame = bytes([MAGIC_V2, len(payload), inc, 0x00, seq & 0xFF, sysid, compid,
                   msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    frame += bytes(payload) + b"\x11\x22"
    if inc & INCOMPAT_SIGNED:
        frame += bytes(SIGNATURE_LEN)   # link_id + 6-byte timestamp + 6-byte sig
    return frame


def _mk_v1(msgid=HEARTBEAT_MSGID, seq=0, sysid=1, compid=1, payload=None):
    if payload is None:
        payload = bytes(9)
    return (bytes([MAGIC_V1, len(payload), seq & 0xFF, sysid, compid,
                   msgid & 0xFF]) + bytes(payload) + b"\x11\x22")


def self_test():
    import random

    n_ok = 0
    n_tot = 0

    def check(cond, msg):
        nonlocal n_ok, n_tot
        n_tot += 1
        if cond:
            n_ok += 1
        print(f"{LOG} {'OK  ' if cond else 'FAIL'} {msg}")

    # --- (a) five well-formed v2 HEARTBEATs -> PASS --------------------------
    a = b"".join(_mk_v2(seq=i, sysid=42, compid=190) for i in range(5))
    sa = parse_stream(a)
    va = classify(sa, elapsed_s=2.5, device="/dev/null", baud=DEF_BAUD)
    check(sa.n_frames == 5 and sa.n_v2 == 5 and sa.n_heartbeats == 5
          and sa.max_consecutive == 5 and sa.n_skipped_bytes == 0
          and sa.n_bytes == len(a) and not sa.truncated_tail,
          f"(a) 5 v2 HEARTBEATs: frames={sa.n_frames} hb={sa.n_heartbeats} "
          f"consecutive={sa.max_consecutive} skipped={sa.n_skipped_bytes}")
    check(va.name == "PASS" and va.exit_code == 0,
          f"(a) verdict {va.name} exit {va.exit_code} (expected PASS/0)")
    check(sa.sysid_pairs() == [((42, 190), 5)],
          f"(a) sysid/compid decoded: {sa.sysid_pairs()} (expected [(42,190) x5])")
    check(abs(sa.frame_hz(2.5) - 2.0) < 1e-9
          and abs(sa.heartbeat_hz(2.5) - 2.0) < 1e-9
          and sa.frame_hz(None) is None and sa.frame_hz(0) is None,
          f"(a) rates over the window: frames {sa.frame_hz(2.5)} Hz, "
          f"hb {sa.heartbeat_hz(2.5)} Hz; UNMEASURABLE window -> None "
          f"(never a fabricated 0.0)")

    # --- (b) the same, v1 framing -> PASS ------------------------------------
    b = b"".join(_mk_v1(seq=i, sysid=1, compid=1) for i in range(5))
    sb = parse_stream(b)
    vb = classify(sb, 2.0, "/dev/null", DEF_BAUD)
    check(sb.n_frames == 5 and sb.n_v1 == 5 and sb.n_v2 == 0
          and sb.n_heartbeats == 5 and sb.max_consecutive == 5
          and sb.n_skipped_bytes == 0 and vb.name == "PASS" and vb.exit_code == 0,
          f"(b) 5 v1 HEARTBEATs -> {vb.name}/{vb.exit_code}, v1={sb.n_v1} "
          f"v2={sb.n_v2} hb={sb.n_heartbeats} (v1 stride is 8+len)")

    # --- (c) SIGNED v2 frames: right stride (+13) ----------------------------
    # Mixed payload lengths so a wrong stride cannot accidentally line up.
    sig_frames = [_mk_v2(seq=i, payload=bytes(n), signed=True)
                  for i, n in enumerate((9, 4, 17))]
    c = b"".join(sig_frames)
    sc = parse_stream(c)
    vc = classify(sc, 1.5, "/dev/null", DEF_BAUD)
    check(sc.n_frames == 3 and sc.n_signed == 3 and sc.n_skipped_bytes == 0
          and sc.max_consecutive == 3 and sc.n_bytes == len(c)
          and len(c) == sum(12 + n + SIGNATURE_LEN for n in (9, 4, 17)),
          f"(c) 3 SIGNED v2 frames parsed at the right stride "
          f"(frames={sc.n_frames} signed={sc.n_signed} skipped="
          f"{sc.n_skipped_bytes}, {len(c)} bytes = sum(12+len+13))")
    check(vc.name == "PASS" and sc.n_heartbeats == 3,
          f"(c) signed HEARTBEATs still counted -> {vc.name}")
    # ...and an UNKNOWN incompat bit must be REJECTED (MAVLink says drop it).
    bad_inc = _mk_v2(incompat=0x04) + _mk_v2()
    s_bad = parse_stream(bad_inc)
    check(s_bad.n_rejected >= 1 and s_bad.n_frames <= 1,
          f"(c2) unknown incompat bit 0x04 rejected "
          f"(rejected={s_bad.n_rejected}, frames={s_bad.n_frames})")

    # --- (d) 200 bytes of FIXED-SEED garbage -> UNCERTAIN, never PASS --------
    # random.Random(1234), NOT os.urandom: CLAUDE.md forbids nondeterminism in a
    # test. This stream contains exactly one 0xFD and no 0xFE (measured).
    d = random.Random(1234).randbytes(200)
    sd = parse_stream(d)
    vd = classify(sd, 3.0, "/dev/ttyAMA0", DEF_BAUD)
    check(vd.name == "UNCERTAIN" and vd.exit_code == 2 and vd.name != "PASS",
          f"(d) 200 bytes of seeded garbage -> {vd.name}/{vd.exit_code} "
          f"(frames={sd.n_frames} consecutive={sd.max_consecutive}) -- "
          f"bytes-but-no-framing is NOT a FAIL and NOT a PASS")
    check("BAUD MISMATCH" in " ".join(vd.ladder).upper()
          or "BAUD MISMATCH" in vd.reason.upper(),
          "(d) garbage verdict names BAUD MISMATCH as the likely cause")
    check(sd.n_bytes == 200 and sd.n_skipped_bytes + sd.tail_bytes >= 199,
          f"(d) every garbage byte accounted for (skipped={sd.n_skipped_bytes}, "
          f"tail={sd.tail_bytes}, total={sd.n_bytes})")

    # --- (e) empty stream -> FAIL exit 1 ------------------------------------
    se = parse_stream(b"")
    ve = classify(se, None, "/dev/ttyAMA0", DEF_BAUD)
    check(ve.name == "FAIL" and ve.exit_code == 1 and ve.name != "PASS"
          and se.n_bytes == 0 and len(ve.ladder) >= 5,
          f"(e) zero bytes -> {ve.name}/{ve.exit_code} with a "
          f"{len(ve.ladder)}-step diagnosis ladder")
    check("DEVICE NODE" in ve.ladder[0].upper(),
          f"(e) ladder is in README §7 likelihood order (first: "
          f"{ve.ladder[0][:34]}...)")

    # --- (f) frames but NO heartbeat -> not PASS ----------------------------
    f_stream = b"".join(_mk_v2(msgid=30, seq=i) for i in range(5))   # ATTITUDE
    sf = parse_stream(f_stream)
    vf = classify(sf, 3.0, "/dev/ttyAMA0", DEF_BAUD)
    check(sf.n_frames == 5 and sf.n_heartbeats == 0 and vf.name == "UNCERTAIN"
          and vf.exit_code == 2 and vf.name != "PASS",
          f"(f) 5 non-HEARTBEAT frames -> {vf.name}/{vf.exit_code} "
          f"('link up, nothing useful'; hb={sf.n_heartbeats})")
    check("HEARTBEAT" in vf.reason.upper(),
          "(f) the reason says the HEARTBEAT is what is missing")

    # --- (g) leading garbage then good frames -> resync and PASS -------------
    # Magic bytes are EXCLUDED from this garbage on purpose: the assertion here is
    # about RESYNC, and an accidental in-garbage frame would make the expected
    # counts a coin flip. The magic-in-random case is check (d).
    rg = random.Random(99)
    g_junk = bytes(rg.choice([x for x in range(256) if x not in MAGICS])
                   for _ in range(40))
    g = g_junk + b"".join(_mk_v2(seq=i) for i in range(5))
    sg = parse_stream(g)
    vg = classify(sg, 2.0, "/dev/ttyAMA0", DEF_BAUD)
    check(sg.n_frames == 5 and sg.n_heartbeats == 5 and sg.max_consecutive == 5
          and sg.n_skipped_bytes == 40 and vg.name == "PASS"
          and vg.exit_code == 0,
          f"(g) 40 junk bytes then 5 frames -> {vg.name}: resynced after "
          f"{sg.n_skipped_bytes} skipped bytes, frames={sg.n_frames}")

    # --- (h) exactly 2 good frames -> NOT PASS (the bar is real) -------------
    h = b"".join(_mk_v2(seq=i) for i in range(2))
    sh = parse_stream(h)
    vh = classify(sh, 1.0, "/dev/ttyAMA0", DEF_BAUD)
    check(sh.n_frames == 2 and sh.max_consecutive == 2
          and vh.name == "UNCERTAIN" and vh.exit_code == 2 and vh.name != "PASS",
          f"(h) exactly 2 good frames (both HEARTBEATs) -> {vh.name}/"
          f"{vh.exit_code}: below the >={MIN_CONSECUTIVE_FRAMES} bar, so NOT a "
          f"PASS even with a heartbeat present")
    check(str(MIN_CONSECUTIVE_FRAMES) in vh.reason,
          f"(h) the reason states the bar ({MIN_CONSECUTIVE_FRAMES})")
    # ...and 3 is the first PASS -- the threshold is exactly where it claims.
    s3 = parse_stream(b"".join(_mk_v2(seq=i) for i in range(3)))
    v3 = classify(s3, 1.0, "/dev/ttyAMA0", DEF_BAUD)
    check(v3.name == "PASS" and v3.exit_code == 0 and s3.max_consecutive == 3,
          f"(h2) exactly 3 good frames -> {v3.name} (the bar is at 3, not 4)")

    # --- extra: 3-byte little-endian msgid decode ---------------------------
    # 0x030201 exercises all three bytes; a 1-byte or 2-byte read gives 1 or 513.
    s_mid = parse_stream(b"".join(_mk_v2(msgid=0x030201, seq=i) for i in range(3)))
    check(list(s_mid.msgid_counts.keys()) == [0x030201],
          f"(i) v2 msgid is 3-byte little-endian: decoded "
          f"{list(s_mid.msgid_counts.keys())} (expected [{0x030201}])")

    # --- extra: truncated tail is recorded, not counted or called garbage ----
    full = b"".join(_mk_v2(seq=i) for i in range(4))
    s_tr = parse_stream(full[:-5])
    check(s_tr.n_frames == 3 and s_tr.truncated_tail and s_tr.tail_bytes == 16
          and classify(s_tr, 1.0).name == "PASS",
          f"(j) a mid-frame cut (normal end of a timed capture) -> "
          f"frames={s_tr.n_frames}, truncated_tail={s_tr.truncated_tail}, "
          f"tail={s_tr.tail_bytes} bytes, verdict still PASS")

    # --- extra: multiple (sysid, compid) pairs are distinguished -------------
    s_ids = parse_stream(_mk_v2(sysid=1, compid=1) + _mk_v2(sysid=1, compid=1)
                         + _mk_v2(sysid=1, compid=191) + _mk_v2(sysid=2, compid=1))
    check(len(s_ids.sysid_pairs()) == 3 and s_ids.sysid_pairs()[0] == ((1, 1), 2),
          f"(k) distinct (sysid,compid) pairs counted: {s_ids.sysid_pairs()}")

    # --- extra: a structurally BROKEN chain does not pass -------------------
    # A frame whose successor byte is neither a magic nor the end of stream is
    # rejected: this is the one thing standing in for the absent CRC.
    broken = _mk_v2() + b"\x00" + _mk_v2()
    s_br = parse_stream(broken)
    check(s_br.n_rejected >= 1 and s_br.max_consecutive < MIN_CONSECUTIVE_FRAMES
          and classify(s_br, 1.0).name != "PASS",
          f"(l) a frame followed by a non-magic byte is REJECTED "
          f"(rejected={s_br.n_rejected}, frames={s_br.n_frames}, "
          f"consecutive={s_br.max_consecutive}) -> "
          f"{classify(s_br, 1.0).name}")

    # --- extra: the hexdump the UNCERTAIN path prints -----------------------
    hd = hexdump(bytes(range(20)), 16)
    check(len(hd) == 2 and "0000" in hd[0] and "00 01 02" in hd[0]
          and "4 more bytes" in hd[1] and hexdump(b"")[0].endswith("(no bytes at all)"),
          f"(m) hexdump renders 16-byte rows + a remainder note, and empty input "
          f"says so explicitly")

    # --- extra: an open failure is exit 3 with a real explanation ------------
    exc = OSError(errno_mod.EACCES, "Permission denied")
    exc.errno = errno_mod.EACCES
    msg = explain_open_errno(exc, "/dev/ttyAMA0")
    exc2 = OSError(errno_mod.ENOENT, "No such file")
    exc2.errno = errno_mod.ENOENT
    msg2 = explain_open_errno(exc2, "/dev/ttyAMA0")
    check("dialout" in msg and "ENOENT" in msg2 and "device node" in msg2.lower(),
          "(n) open() errno explained: EACCES -> dialout group, ENOENT -> wrong "
          "device node")

    # --- extra: a NON-TTY --device is a PortError (exit 3), not a traceback ---
    # Found by verification, 2026-07-29: termios.error is NOT an OSError subclass,
    # so pointing --device at a regular file crashed with a raw traceback and exit
    # 1 -- and exit 1 is the "zero bytes on a good port" FAIL verdict. A port we
    # never configured must be exit 3 (nothing measured). Uses a temp FILE: still
    # no hardware, no device node, fully deterministic.
    import tempfile
    with tempfile.NamedTemporaryFile(prefix="fc_link_notatty_") as tf:
        tf.write(b"not a uart")
        tf.flush()
        try:
            read_raw(tf.name, DEF_BAUD, 0.1)
            raised = None
        except PortError as pe:
            raised = pe
        except Exception as other:                    # pragma: no cover
            raised = other
    check(isinstance(raised, PortError) and "NOT A SERIAL PORT" in str(raised),
          f"(o) a non-tty --device raises PortError (-> exit 3, 'nothing was "
          f"measured'), not a termios traceback: {type(raised).__name__}")

    ok = (n_ok == n_tot)
    print(f"{LOG} SELF-TEST {'PASS' if ok else 'FAIL'} ({n_ok}/{n_tot})")
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default=DEF_DEVICE,
                    help=f"serial device (default {DEF_DEVICE} = the Pi 5 "
                         f"GPIO14/15 UART; /dev/serial0 is a symlink to it)")
    ap.add_argument("--baud", type=int, default=DEF_BAUD,
                    help=f"baud (default {DEF_BAUD} = SER_TEL2_BAUD from "
                         f"configs/px4_6cmini/bench.params)")
    ap.add_argument("--seconds", type=float, default=DEF_SECONDS,
                    help=f"how long to listen (default {DEF_SECONDS}); "
                         f"HEARTBEAT is ~1 Hz so keep this >= 3")
    ap.add_argument("--hexdump-bytes", type=int, default=DEF_HEXDUMP_BYTES,
                    help=f"bytes to dump when no framing is found (default "
                         f"{DEF_HEXDUMP_BYTES})")
    ap.add_argument("--use-stty", action="store_true",
                    help="configure the port by shelling out to `stty` instead of "
                         "termios (documented fallback for a platform whose "
                         "termios lacks the baud constant)")
    ap.add_argument("--check-port", action="store_true",
                    help="report the PRECONDITIONS only (device node, "
                         "/dev/serial0, console=serial0, serial-getty, "
                         "enable_uart, permissions, current baud) and read no "
                         "bytes")
    ap.add_argument("--self-test", action="store_true",
                    help="offline parser + verdict check on synthetic streams; "
                         "exits 0/1, opens no device")
    args = ap.parse_args(argv)

    if args.self_test:
        return 0 if self_test() else 1
    if args.check_port:
        return check_port(args.device, args.baud)
    if args.seconds <= 0:
        # A zero-length capture would produce a zero-byte FAIL that says nothing
        # about the link -- refuse rather than manufacture a verdict.
        print(f"{LOG} --seconds must be > 0 (got {args.seconds}): a zero-length "
              f"capture cannot produce a verdict about the link", file=sys.stderr)
        return 2
    return run_check(args)


if __name__ == "__main__":
    sys.exit(main())
