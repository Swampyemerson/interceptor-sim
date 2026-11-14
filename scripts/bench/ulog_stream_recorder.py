#!/usr/bin/env python3
"""Flight-log recorder for a Pixhawk with no SD card.

PX4 can send its flight log (ULog) over MAVLink instead of writing it to a card.
This asks for that stream on whatever link it is given (the Pi's serial cable, a
USB cable, or a UDP port from a MAVLink router) and writes a normal .ulg file
that pyulog / Flight Review / PlotJuggler open.

Toward flight behaviour it is READ-ONLY: it sends heartbeats, the two logging
commands (start / stop) and packet acknowledgements. No params, no arming, no
setpoints.

    probe (bench):  ulog_stream_recorder.py --port /dev/ttyACM0 --seconds 60
    flight:         ulog_stream_recorder.py --port auto --per-flight \
                        --fallback-port /dev/ttyAMA10 --on-arm "<video command>"
    stop cleanly:   touch /tmp/ulog_stream_recorder.stop   (or SIGTERM / Ctrl-C)

--on-arm runs a command (the camera recorder) when the Pixhawk reports ARMED and
interrupts it when the flight's file closes, a few seconds after disarm.

--port auto uses the Pixhawk's USB port when it is plugged into this computer
and otherwise the fallback serial port. The serial cable is too slow for the
whole log (about a third is lost, measured), so files recorded there say GAPS.
While it holds a serial port it holds it EXCLUSIVELY: a second program opening
the same port fails loudly instead of both silently reading half the bytes.

Each .ulg gets a .stats.json beside it: bytes, packet gaps, the stream rates seen
on the link while recording, and a verdict. A gap is written INTO the log as a
ULog dropout record, so a lossy capture cannot pass as a clean one.

Built for a power cut: the file is synced to the card every second, and a file
left half-written (battery pulled, crash) is trimmed to its last whole record
and kept as fc_<time>_recovered.ulg the next time the recorder starts.

Why every session begins with STOP: PX4's logger refuses START while it thinks a
stream is running, and a client that died without STOP leaves it in that state
until reboot (docs/fc_logging_without_sd.md).
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import time

STOP_FLAG = "/tmp/ulog_stream_recorder.stop"
ULOG_MAGIC = b"ULog\x01\x12\x35"
ULOG_FILE_HEADER_LEN = 16
# Every ULog record type PX4 writes. Anything else at a record boundary means the
# framing is lost and the assembler must resync.
ULOG_TYPES = frozenset(b"BFIMPQARDLCSO")
NO_MESSAGE_START = 255

MAV_CMD_LOGGING_START = 2510
MAV_CMD_LOGGING_STOP = 2511
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_IN_PROGRESS = 5
MAV_TYPE_ONBOARD_CONTROLLER = 18
MAV_COMP_ID_LOG = 155
ARMED_FLAG = 128
MIN_FREE_BYTES = 2 * 1024**3
GROUND_FILES_KEPT = 3
# PX4 buffers log data while the (acknowledged) header is being sent and can
# overflow once at the hand-over. ONE gap this soon after the data begins is
# reported as a start-up gap, separately from gaps during the recording.
STARTUP_GRACE_S = 1.0
SYNC_EVERY_S = 1.0
HOOK_MIN_FREE_BYTES = 5 * 1024**3   # do not start a video with less card space than this
HOOK_STOP_WAIT_S = 20.0
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class UlogAssembler:
    """Turn the numbered MAVLink log packets into a well-formed ULog byte stream.

    Pure (no I/O beyond `sink.write`), so the tests can drive it directly. Only
    whole ULog records reach the sink. When a packet is missing, the half-record
    in hand is thrown away, a dropout record is written in its place, and writing
    resumes at the next record boundary PX4 points to.
    """

    def __init__(self, sink):
        self.sink = sink
        self.buf = bytearray()
        self.expected = None          # next packet number
        self.header_done = False
        self.need_resync = False
        self.bytes_written = 0
        self.packets = 0
        self.duplicates = 0
        self.gaps = 0
        self.lost_packets = 0
        self.framing_errors = 0
        self.dropout_ms_total = 0
        self.gap_times = []           # monotonic time of each gap
        self._last_good_t = None

    def feed(self, seq: int, length: int, first_off: int, data: bytes,
             now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        if self.expected is None:
            self.expected = seq
        if seq == (self.expected - 1) & 0xFFFF:
            # A resend of the packet we already have (our ack was lost).
            self.duplicates += 1
            return
        if seq != self.expected:
            self.gaps += 1
            self.gap_times.append(now)
            self.lost_packets += (seq - self.expected) & 0xFFFF
            self._lose_sync()
        self.expected = (seq + 1) & 0xFFFF
        self.packets += 1
        payload = bytes(data[:length])
        if self.need_resync:
            if first_off == NO_MESSAGE_START or first_off >= length:
                return                # no record starts in this packet; keep waiting
            payload = payload[first_off:]
            self.need_resync = False
            self._write_dropout(now)
        self.buf += payload
        self._flush_whole_records()
        if not self.need_resync:
            self._last_good_t = now

    def _lose_sync(self) -> None:
        self.buf.clear()
        self.need_resync = True

    def _write_dropout(self, now: float) -> None:
        if not self.header_done:
            return
        ms = 0 if self._last_good_t is None else int((now - self._last_good_t) * 1000)
        ms = max(1, min(ms, 0xFFFF))
        self.dropout_ms_total += ms
        self._emit(struct.pack("<HBH", 2, ord("O"), ms))

    def _emit(self, b: bytes) -> None:
        self.sink.write(b)
        self.bytes_written += len(b)

    def _flush_whole_records(self) -> None:
        if not self.header_done:
            if len(self.buf) < ULOG_FILE_HEADER_LEN:
                return
            if not self.buf.startswith(ULOG_MAGIC):
                raise ValueError("stream does not begin with the ULog file header "
                                 f"(got {bytes(self.buf[:8])!r})")
            self._emit(bytes(self.buf[:ULOG_FILE_HEADER_LEN]))
            del self.buf[:ULOG_FILE_HEADER_LEN]
            self.header_done = True
        while len(self.buf) >= 3:
            size, mtype = struct.unpack_from("<HB", self.buf, 0)
            if mtype not in ULOG_TYPES:
                self.framing_errors += 1
                self._lose_sync()
                return
            total = 3 + size
            if len(self.buf) < total:
                return
            self._emit(bytes(self.buf[:total]))
            del self.buf[:total]


def count_startup_gaps(gap_times: list, first_data_t: float | None) -> int:
    """How many gaps are the one-off hand-over gap: 1 or 0, never more.

    Two or more gaps in the first second is a link that is losing data, not a
    start-up artefact, so none of them is excused (a lossy 6 s capture once
    scored CLEAN with a looser rule).
    """
    if first_data_t is None:
        return 0
    early = [g for g in gap_times if g - first_data_t <= STARTUP_GRACE_S]
    return 1 if len(early) == 1 else 0


def verdict(stats: dict) -> str:
    """One word for the capture. Never PASS on an empty or gappy file."""
    if stats["bytes_written"] <= ULOG_FILE_HEADER_LEN or stats["packets"] == 0:
        return "EMPTY"
    if stats["gaps_after_startup"] or stats["framing_errors"]:
        return "GAPS"
    if stats.get("stop_result") != "accepted":
        return "UNCLEAN_STOP"
    return "CLEAN"


def _prune_ground_files(out_dir: str) -> None:
    ground = sorted(f for f in os.listdir(out_dir) if f.endswith("_ground.ulg"))
    for f in ground[:-GROUND_FILES_KEPT]:
        for path in (os.path.join(out_dir, f),
                     os.path.join(out_dir, f[:-4] + ".stats.json")):
            if os.path.exists(path):
                os.remove(path)


def whole_record_length(data: bytes) -> int:
    """Length of the longest prefix of `data` that is a file header + whole records."""
    if len(data) < ULOG_FILE_HEADER_LEN or not data.startswith(ULOG_MAGIC):
        return 0
    pos = ULOG_FILE_HEADER_LEN
    while pos + 3 <= len(data):
        size, mtype = struct.unpack_from("<HB", data, pos)
        if mtype not in ULOG_TYPES or pos + 3 + size > len(data):
            break
        pos += 3 + size
    return pos


def recover_part_files(out_dir: str) -> list[str]:
    """Keep what a power cut left behind: trim each .part to whole records."""
    kept = []
    for name in sorted(os.listdir(out_dir)):
        if not name.endswith(".ulg.part"):
            continue
        path = os.path.join(out_dir, name)
        with open(path, "rb") as f:
            data = f.read()
        n = whole_record_length(data)
        if n <= ULOG_FILE_HEADER_LEN:
            os.remove(path)
            continue
        final = os.path.join(out_dir, name[:-len(".ulg.part")] + "_recovered.ulg")
        with open(path, "r+b") as f:
            f.truncate(n)
        os.replace(path, final)
        with open(final[:-4] + ".stats.json", "w") as sf:
            json.dump({"verdict": "RECOVERED", "bytes_written": n,
                       "bytes_trimmed": len(data) - n,
                       "note": "recorder did not close this file (power cut or crash); "
                               "trimmed to the last whole record"}, sf, indent=1)
        print(f"recovered {final} ({n / 1e6:.2f} MB, trimmed {len(data) - n} bytes)", flush=True)
        kept.append(final)
    return kept


def pick_port(usb_port: str, fallback_port: str | None, exists=os.path.exists) -> str | None:
    """USB when the Pixhawk is plugged into this computer, else the fallback, else None."""
    if exists(usb_port):
        return usb_port
    if fallback_port and exists(fallback_port):
        return fallback_port
    return None


class ArmHook:
    """Run a command while the vehicle is flying (the camera recorder).

    start() at arm, stop() when the flight's file closes. A hook that fails to
    start, or dies, is reported in the stats and never stops the log recording.
    """

    def __init__(self, cmd: str | None, cwd: str, popen=subprocess.Popen,
                 free_bytes=lambda path: shutil.disk_usage(path).free):
        self.argv = shlex.split(cmd) if cmd else None
        self.cwd = cwd
        self._popen = popen
        self._free_bytes = free_bytes
        self.proc = None
        self.report: dict = {}

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, stamp: str, t_since_arm: float = 0.0) -> None:
        if self.argv is None or self.proc is not None or self.report:
            return                    # no command, already running, or already tried this flight
        self.report = {"cmd": " ".join(self.argv)}
        if self._free_bytes(self.cwd) < HOOK_MIN_FREE_BYTES:
            self.report["skipped"] = "less than 5 GB free on the card"
            print("on-arm command SKIPPED: card nearly full", flush=True)
            return
        try:
            self.proc = self._popen(self.argv, cwd=self.cwd,
                                    env={**os.environ, "FC_LOG_STAMP": stamp})
        except OSError as e:
            self.report["skipped"] = f"could not start: {e}"
            print(f"on-arm command FAILED to start: {e}", flush=True)
            return
        self.report.update(pid=self.proc.pid, started_unix=round(time.time(), 3),
                           started_s_after_arm_seen=round(t_since_arm, 2))
        print(f"on-arm command started (pid {self.proc.pid})", flush=True)

    def stop(self) -> dict:
        """Interrupt the command, wait for it to finish its files, return the report."""
        report, proc = self.report, self.proc
        self.proc, self.report = None, {}
        if proc is None:
            return report
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=HOOK_STOP_WAIT_S)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
                report["killed"] = "did not stop within 20 s of the interrupt"
        else:
            report["died_early"] = True
        report["exit_code"] = proc.returncode
        print(f"on-arm command ended (exit {proc.returncode})", flush=True)
        return report


class Recorder:
    def __init__(self, args):
        # The log packets only exist in MAVLink 2; PX4 answers in the version it hears.
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.args = args
        self.stop_requested = False
        self.port = args.active_port
        self.m = mavutil.mavlink_connection(self.port, baud=args.baud,
                                            source_system=args.source_system,
                                            source_component=MAV_COMP_ID_LOG)
        self._claim_port()
        self.armed = self.real_armed = False
        self.last_beat = 0.0
        self.hook = ArmHook(args.on_arm, args.hook_cwd)
        self.switch_to_usb = False

    def _claim_port(self) -> None:
        """Make a second open() of this serial port fail (EBUSY) instead of sharing it."""
        fd = getattr(getattr(self.m, "port", None), "fd", None)
        if fd is None or not sys.platform.startswith("linux"):
            return
        import fcntl
        import termios
        fcntl.ioctl(fd, termios.TIOCEXCL)

    def usb_appeared(self) -> bool:
        a = self.args
        return (a.port == "auto" and self.port != a.usb_port and os.path.exists(a.usb_port))

    # -- link helpers ------------------------------------------------------
    def beat(self) -> None:
        now = time.monotonic()
        if now - self.last_beat >= 1.0:
            self.m.mav.heartbeat_send(MAV_TYPE_ONBOARD_CONTROLLER,
                                      self.mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_beat = now

    def wait_vehicle(self, timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and not self.should_stop():
            self.beat()
            msg = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg is not None and msg.get_srcComponent() == 1:
                self.sysid = msg.get_srcSystem()
                self.real_armed = self.armed = bool(msg.base_mode & ARMED_FLAG)
                return True
        return False

    def should_stop(self) -> bool:
        return self.stop_requested or os.path.exists(STOP_FLAG)

    def command(self, cmd: int, on_msg=None, timeout: float = 3.0):
        """Send a COMMAND_LONG and return the final ack result (None = no ack)."""
        self.m.mav.command_long_send(self.sysid, 1, cmd, 0, 0, 0, 0, 0, 0, 0, 0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.beat()
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is None:
                continue
            if msg.get_type() == "COMMAND_ACK" and msg.command == cmd:
                if msg.result == MAV_RESULT_IN_PROGRESS:
                    continue
                return msg.result
            if on_msg is not None:
                on_msg(msg)
        return None

    def start_stream(self) -> tuple[bool, list]:
        """STOP, then START; retry while PX4 says 'temporarily rejected'."""
        history = []
        for _ in range(5):
            if not self.args.no_stop_first:
                history.append(("stop", self.command(MAV_CMD_LOGGING_STOP)))
            r = self.command(MAV_CMD_LOGGING_START, timeout=3.0)
            history.append(("start", r))
            if r == MAV_RESULT_ACCEPTED:
                return True, history
            if self.args.no_stop_first or self.should_stop():
                break
            time.sleep(0.5)
        return False, history

    # -- one file ----------------------------------------------------------
    def record_one(self) -> dict:
        a = self.args
        os.makedirs(a.out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        tmp_path = os.path.join(a.out_dir, f"fc_{stamp}.ulg.part")
        stats = {"started_utc": stamp, "port": self.port, "baud": a.baud,
                 "link": "usb" if self.port == a.usb_port or "ACM" in self.port
                 or self.port.upper().startswith("COM") else "serial or other"}
        ok, history = self.start_stream()
        stats["start_history"] = [[k, v] for k, v in history]
        if not ok:
            stats.update(bytes_written=0, packets=0, gaps=0, gaps_after_startup=0,
                         framing_errors=0, verdict="START_REFUSED")
            print(f"FAIL: log stream did not start: {history}", flush=True)
            return stats

        rates: dict[str, int] = {}
        was_armed = self.armed
        ever_armed = self.armed
        disarm_t = None
        t0 = time.monotonic()
        first_data_t = None
        last_data_t = t0
        last_print = t0
        last_sync = t0
        first_unacked_t = None
        acked = 0
        arm_seen_t = None
        clock_pairs = []              # (this computer's unix time, Pixhawk ms since boot)

        with open(tmp_path, "wb") as f:
            asm = UlogAssembler(f)

            def handle(msg) -> None:
                nonlocal acked, first_data_t, last_data_t, first_unacked_t
                t = msg.get_type()
                if t == "LOGGING_DATA" and first_unacked_t is None:
                    first_unacked_t = time.monotonic()
                if t == "LOGGING_DATA_ACKED":
                    self.m.mav.logging_ack_send(msg.get_srcSystem(),
                                                msg.get_srcComponent(), msg.sequence)
                    acked += 1
                if t in ("LOGGING_DATA", "LOGGING_DATA_ACKED"):
                    now = time.monotonic()
                    first_data_t = first_data_t or now
                    last_data_t = now
                    asm.feed(msg.sequence, msg.length, msg.first_message_offset,
                             bytes(msg.data), now)
                elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    self.real_armed = bool(msg.base_mode & ARMED_FLAG)
                else:
                    rates[t] = rates.get(t, 0) + 1
                    if t == "SYSTEM_TIME":
                        pair = [round(time.time(), 3), msg.time_boot_ms]
                        clock_pairs[1:] = [pair]      # keep the first and the latest

            reason = "stop requested"
            while True:
                now = time.monotonic()
                self.beat()
                if self.should_stop():
                    break
                if a.seconds is not None and now - t0 >= a.seconds:
                    reason = "--seconds reached"
                    break
                if now - last_data_t > a.stall_s:
                    reason = f"no log data for {a.stall_s:.0f} s"
                    break
                if shutil.disk_usage(a.out_dir).free < MIN_FREE_BYTES:
                    reason = "disk nearly full"
                    self.stop_requested = True
                    break
                # --pretend-armed: bench test of everything that hangs off "armed" (file
                # naming, the on-arm command) without arming anything.
                pretend = (a.pretend_armed is not None
                           and a.pretend_armed[0] <= now - t0 < sum(a.pretend_armed))
                self.armed = self.real_armed or pretend
                if self.armed:
                    ever_armed = True
                    if arm_seen_t is None:
                        arm_seen_t = now
                    self.hook.start(stamp, now - arm_seen_t)
                if was_armed and not self.armed:
                    disarm_t = now
                if self.armed:
                    disarm_t = None
                was_armed = self.armed
                if a.per_flight and disarm_t is not None and now - disarm_t > a.linger_s:
                    reason = "flight ended"
                    break
                if (a.per_flight and not ever_armed
                        and now - t0 > a.ground_rotate_min * 60.0):
                    reason = "ground file rotated"
                    break
                if not self.armed and disarm_t is None and self.usb_appeared():
                    reason = "USB link appeared; switching to it"
                    self.switch_to_usb = True
                    break
                if now - last_sync >= SYNC_EVERY_S:
                    f.flush()
                    os.fsync(f.fileno())
                    last_sync = now
                # Short timeout on purpose: on Windows pymavlink SLEEPS half of it when
                # idle, and every header packet waits for our ack (0.2 s here made the
                # header take 22 s and cost a gap when the data began).
                msg = self.m.recv_match(blocking=True, timeout=0.02)
                if msg is not None:
                    handle(msg)
                if now - last_print > (5.0 if a.seconds is not None else 60.0):
                    dt = now - t0
                    print(f"  {dt:5.0f}s armed={self.armed} {asm.bytes_written / 1e6:.2f} MB "
                          f"({asm.bytes_written / dt / 1000:.1f} KB/s) gaps={asm.gaps} "
                          f"lost={asm.lost_packets}", flush=True)
                    last_print = now

            t_stream_end = time.monotonic()
            # STOP makes PX4 send a short acked tail (post-flight CPU load); keep
            # acking it until the final ack arrives.
            try:
                if a.leave_running:
                    stop_r = "not sent (--leave-running)"
                else:
                    stop_r = self.command(MAV_CMD_LOGGING_STOP, on_msg=handle, timeout=4.0)
                f.flush()
                os.fsync(f.fileno())
            finally:
                hook_report = self.hook.stop()

        dt = max(1e-6, t_stream_end - t0)
        stats.update(
            seconds=round(dt, 2), end_reason=reason, ever_armed=ever_armed,
            bytes_written=asm.bytes_written, kb_per_s=round(asm.bytes_written / dt / 1000, 2),
            packets=asm.packets, acked_packets=acked, duplicates=asm.duplicates,
            gaps=asm.gaps, lost_packets=asm.lost_packets,
            gaps_at_startup=count_startup_gaps(asm.gap_times, first_unacked_t),
            gap_times_s=[round(g - t0, 2) for g in asm.gap_times[:100]],
            header_seconds=None if first_unacked_t is None else round(first_unacked_t - t0, 2),
            framing_errors=asm.framing_errors, dropout_ms_total=asm.dropout_ms_total,
            first_data_delay_s=None if first_data_t is None else round(first_data_t - t0, 2),
            stop_result={MAV_RESULT_ACCEPTED: "accepted", None: "no ack"}.get(stop_r, str(stop_r)),
            link_rates_hz={k: round(v / dt, 1) for k, v in sorted(rates.items())},
            armed_seen_s=None if arm_seen_t is None else round(arm_seen_t - t0, 2),
            armed_was_pretended=bool(a.pretend_armed) and not self.real_armed,
            clock_pairs_unix_vs_fc_boot_ms=clock_pairs,
        )
        if hook_report:
            stats["on_arm_command"] = hook_report
        stats["gaps_after_startup"] = stats["gaps"] - stats["gaps_at_startup"]
        stats["verdict"] = verdict(stats)
        kind = "flight" if ever_armed else "ground"
        final = os.path.join(a.out_dir, f"fc_{stamp}_{kind}.ulg")
        if stats["verdict"] == "EMPTY":
            os.remove(tmp_path)
            final = None
        else:
            os.replace(tmp_path, final)
            with open(final[:-4] + ".stats.json", "w") as sf:
                json.dump(stats, sf, indent=1)
            _prune_ground_files(a.out_dir)
        stats["file"] = final
        print(f"{stats['verdict']}: {final} {asm.bytes_written / 1e6:.2f} MB in {dt:.0f}s "
              f"({stats['kb_per_s']} KB/s), gaps={asm.gaps} "
              f"({stats['gaps_at_startup']} at start-up, {stats['gaps_after_startup']} after), "
              f"lost packets={asm.lost_packets}, ended: {reason}", flush=True)
        return stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", required=True,
                    help="/dev/ttyACM0 (USB), /dev/ttyAMA10 (serial), udpin:0.0.0.0:14551, or "
                         "'auto' = --usb-port when it exists, else --fallback-port")
    ap.add_argument("--usb-port", default="/dev/pixhawk-usb")
    ap.add_argument("--fallback-port", default=None,
                    help="serial port to use with --port auto while no USB link exists")
    ap.add_argument("--on-arm", default=None,
                    help="command to run while armed (interrupted when the flight's file closes)")
    ap.add_argument("--hook-cwd", default=REPO_ROOT, help="working directory for --on-arm")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=None, help="stop after this long")
    ap.add_argument("--per-flight", action="store_true",
                    help="close the file after each disarm and start a new one")
    ap.add_argument("--linger-s", type=float, default=5.0,
                    help="keep recording this long after disarm (per-flight mode)")
    ap.add_argument("--ground-rotate-min", type=float, default=10.0,
                    help="per-flight mode: start a fresh file this often while never armed "
                         f"(only the newest {GROUND_FILES_KEPT} ground files are kept)")
    ap.add_argument("--stall-s", type=float, default=10.0,
                    help="restart the stream if no log data arrives for this long")
    ap.add_argument("--wait-s", type=float, default=None,
                    help="how long to wait for the Pixhawk (default: 15 s, forever with --per-flight)")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/fc_logs"))
    ap.add_argument("--source-system", type=int, default=1,
                    help="MAVLink system id to speak as (default: the vehicle's own, 1)")
    ap.add_argument("--pretend-armed", type=float, nargs=2, metavar=("AFTER_S", "FOR_S"),
                    default=None,
                    help="diagnostic only: treat the vehicle as armed for FOR_S seconds, "
                         "AFTER_S into each file (tests the on-arm command without arming)")
    ap.add_argument("--no-stop-first", action="store_true",
                    help="diagnostic only: send START without the STOP in front")
    ap.add_argument("--leave-running", action="store_true",
                    help="diagnostic only: end the session without sending STOP "
                         "(reproduces a crashed client)")
    args = ap.parse_args()

    if os.path.exists(STOP_FLAG):
        os.remove(STOP_FLAG)
    os.makedirs(args.out_dir, exist_ok=True)
    recover_part_files(args.out_dir)

    stop = {"now": False}

    def on_signal(*_):
        stop["now"] = True
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    worst = 0
    while not stop["now"]:
        args.active_port = args.port
        if args.port == "auto":
            args.active_port = pick_port(args.usb_port, args.fallback_port)
            if args.active_port is None:
                if not args.per_flight:
                    print(f"FAIL: neither {args.usb_port} nor the fallback port exists", flush=True)
                    return 2
                time.sleep(1.0)
                continue
        rec = Recorder(args)
        rec.should_stop = lambda: (stop["now"] or rec.stop_requested
                                   or os.path.exists(STOP_FLAG))
        print(f"link: {args.active_port}", flush=True)
        try:
            wait = args.wait_s if args.wait_s is not None else (1e12 if args.per_flight else 15.0)
            if not rec.wait_vehicle(wait):
                if rec.should_stop():
                    break
                print("FAIL: no heartbeat from the Pixhawk", flush=True)
                return 2
            while True:
                stats = rec.record_one()
                v = stats["verdict"]
                worst = max(worst, {"CLEAN": 0, "GAPS": 3, "UNCLEAN_STOP": 3}.get(v, 2))
                if rec.should_stop() or args.seconds is not None or not args.per_flight:
                    return worst
                if rec.switch_to_usb:
                    break
                if v in ("START_REFUSED", "EMPTY"):
                    time.sleep(2.0)       # Pixhawk rebooting or link down: keep trying
                    rec.wait_vehicle(1e12)
        finally:
            rec.hook.stop()
            rec.m.close()
    return worst


if __name__ == "__main__":
    raise SystemExit(main())
