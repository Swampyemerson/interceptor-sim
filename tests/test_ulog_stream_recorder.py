"""The flight-log recorder's packet assembler, tested against a REAL PX4 stream.

The fixture is a trimmed capture from the interceptor's own Pixhawk (PX4 v1.16.0,
2026-11-13, runs/fc_log_stream_20261113), i.e. it comes from the producer, not
from a hand-typed imitation. The tests re-cut it into MAVLink-sized packets the
way PX4 does, damage the packet stream, and require that what the recorder
writes is still a file the standard reader (pyulog) opens.
"""
import io
import json
import os
import sys

import pytest
from pyulog import ULog

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts", "bench"))
import ulog_stream_recorder as rec  # noqa: E402

FIXTURE = os.path.join(REPO, "tests", "fixtures", "px4_v1160_log_stream_capture.ulg")
PAYLOAD = 249  # bytes of log per MAVLink LOGGING_DATA packet


@pytest.fixture(scope="module")
def capture() -> bytes:
    with open(FIXTURE, "rb") as f:
        data = f.read()
    assert len(data) > 100_000, "fixture missing or truncated"
    assert rec.whole_record_length(data) == len(data), "fixture must end on a record boundary"
    return data


def record_starts(data: bytes) -> list[int]:
    starts, pos = [0], rec.ULOG_FILE_HEADER_LEN
    while pos + 3 <= len(data):
        starts.append(pos)
        pos += 3 + int.from_bytes(data[pos:pos + 2], "little")
    return starts


def packetize(data: bytes, first_seq: int = 0):
    """Cut a log into (seq, length, first_message_offset, payload) like PX4 does."""
    starts = record_starts(data)
    packets, si = [], 0
    for i, base in enumerate(range(0, len(data), PAYLOAD)):
        chunk = data[base:base + PAYLOAD]
        while si < len(starts) and starts[si] < base:
            si += 1
        first = starts[si] - base if si < len(starts) and starts[si] < base + PAYLOAD else 255
        packets.append(((first_seq + i) & 0xFFFF, len(chunk), first,
                        chunk + bytes(PAYLOAD - len(chunk))))
    return packets


def assemble(packets):
    sink = io.BytesIO()
    asm = rec.UlogAssembler(sink)
    for i, (seq, length, first, payload) in enumerate(packets):
        asm.feed(seq, length, first, payload, now=i * 0.006)
    return asm, sink.getvalue()


def n_dropouts(data: bytes, tmp_path) -> int:
    p = tmp_path / "out.ulg"
    p.write_bytes(data)
    return len(ULog(str(p)).dropouts)


def test_lossless_stream_reproduces_the_log_exactly(capture):
    asm, out = assemble(packetize(capture))
    assert out == capture
    assert (asm.gaps, asm.lost_packets, asm.framing_errors, asm.duplicates) == (0, 0, 0, 0)


def test_packet_numbers_wrap_at_65535(capture):
    asm, out = assemble(packetize(capture, first_seq=65_300))
    assert out == capture and asm.gaps == 0


def test_resent_packet_is_not_written_twice(capture):
    packets = packetize(capture)
    packets.insert(41, packets[40])
    asm, out = assemble(packets)
    assert out == capture and asm.duplicates == 1 and asm.gaps == 0


def test_lost_packets_leave_a_readable_log_with_the_gap_recorded(capture, tmp_path):
    packets = packetize(capture)
    before = n_dropouts(capture, tmp_path)
    damaged = packets[:600] + packets[604:]
    asm, out = assemble(damaged)
    assert (asm.gaps, asm.lost_packets) == (1, 4)
    assert len(out) < len(capture)
    assert rec.whole_record_length(out) == len(out), "only whole records may be written"
    assert n_dropouts(out, tmp_path) == before + 1, "the gap must be visible in the log"


def test_every_single_packet_loss_position_still_yields_whole_records(capture):
    packets = packetize(capture)
    n_resynced = 0
    for k in range(540, len(packets) - 2, 7):      # the data part, past the header
        asm, out = assemble(packets[:k] + packets[k + 1:])
        assert asm.gaps == 1 and asm.framing_errors == 0, k
        assert rec.whole_record_length(out) == len(out), k
        n_resynced += 1
    assert n_resynced > 15, "the loop must actually have exercised losses"


def test_a_stream_that_is_not_a_px4_log_is_refused():
    asm = rec.UlogAssembler(io.BytesIO())
    with pytest.raises(ValueError):
        asm.feed(0, PAYLOAD, 0, bytes(range(1, PAYLOAD + 1)))


def test_half_written_file_is_recovered_to_whole_records(capture, tmp_path):
    cut = len(capture) - 37                         # power cut mid-record
    (tmp_path / "fc_20261113T000000Z.ulg.part").write_bytes(capture[:cut])
    (tmp_path / "fc_20261113T000001Z.ulg.part").write_bytes(capture[:16])   # header only
    kept = rec.recover_part_files(str(tmp_path))
    assert [os.path.basename(k) for k in kept] == ["fc_20261113T000000Z_recovered.ulg"]
    data = open(kept[0], "rb").read()
    assert 0 < len(data) <= cut and rec.whole_record_length(data) == len(data)
    assert len(ULog(kept[0]).data_list) > 50
    stats = json.load(open(kept[0][:-4] + ".stats.json"))
    assert stats["verdict"] == "RECOVERED" and stats["bytes_trimmed"] == cut - len(data)
    assert not list(tmp_path.glob("*.part"))


def test_verdict_never_passes_an_empty_or_gappy_capture():
    base = dict(bytes_written=500_000, packets=2000, gaps=0, gaps_after_startup=0,
                framing_errors=0, stop_result="accepted")
    assert rec.verdict(base) == "CLEAN"
    assert rec.verdict({**base, "gaps": 1}) == "CLEAN"          # start-up gap only
    assert rec.verdict({**base, "gaps": 3, "gaps_after_startup": 2}) == "GAPS"
    assert rec.verdict({**base, "framing_errors": 1}) == "GAPS"
    assert rec.verdict({**base, "stop_result": "no ack"}) == "UNCLEAN_STOP"
    assert rec.verdict({**base, "packets": 0}) == "EMPTY"
    assert rec.verdict({**base, "bytes_written": 16}) == "EMPTY"


def test_only_one_early_gap_is_excused_as_start_up():
    assert rec.count_startup_gaps([10.3], first_data_t=10.0) == 1
    assert rec.count_startup_gaps([10.3, 10.6], first_data_t=10.0) == 0   # lossy link
    assert rec.count_startup_gaps([14.0], first_data_t=10.0) == 0         # mid-recording
    assert rec.count_startup_gaps([], first_data_t=10.0) == 0
    assert rec.count_startup_gaps([10.3], first_data_t=None) == 0


def test_auto_port_prefers_usb_then_fallback_then_nothing():
    have = lambda *names: (lambda p: p in names)
    assert rec.pick_port("/dev/pixhawk-usb", "/dev/ttyAMA10", have("/dev/pixhawk-usb", "/dev/ttyAMA10")) == "/dev/pixhawk-usb"
    assert rec.pick_port("/dev/pixhawk-usb", "/dev/ttyAMA10", have("/dev/ttyAMA10")) == "/dev/ttyAMA10"
    assert rec.pick_port("/dev/pixhawk-usb", None, have("/dev/ttyAMA10")) is None
    assert rec.pick_port("/dev/pixhawk-usb", "/dev/ttyAMA10", have()) is None


class FakeProc:
    def __init__(self, argv, cwd=None, env=None, dies=False):
        self.argv, self.env, self.pid, self.returncode = argv, env, 4242, None
        self.signals = []
        if dies:
            self.returncode = 1

    def poll(self):
        return self.returncode

    def send_signal(self, sig):
        self.signals.append(sig)
        self.returncode = 0

    def wait(self, timeout=None):
        return self.returncode


def test_on_arm_command_runs_once_per_flight_and_is_interrupted_at_the_end():
    started = []

    def popen(argv, cwd=None, env=None):
        started.append(FakeProc(argv, cwd, env))
        return started[-1]

    hook = rec.ArmHook("record_video.py --note 'tuning flight'", "/repo", popen=popen,
                       free_bytes=lambda _: 40 * 1024**3)
    for _ in range(50):                      # the record loop calls start() every pass while armed
        hook.start("20261113T000000Z")
    assert len(started) == 1 and hook.running()
    assert started[0].argv == ["record_video.py", "--note", "tuning flight"]
    assert started[0].env["FC_LOG_STAMP"] == "20261113T000000Z"
    report = hook.stop()
    assert started[0].signals == [rec.signal.SIGINT] and report["exit_code"] == 0
    hook.start("20261113T000100Z")           # next flight starts a new one
    assert len(started) == 2


def test_on_arm_command_is_skipped_on_a_full_card_and_reports_an_early_death():
    calls = []
    hook = rec.ArmHook("video", "/repo", popen=lambda *a, **k: calls.append(1),
                       free_bytes=lambda _: 1 * 1024**3)
    for _ in range(5):
        hook.start("x")
    assert calls == [] and "skipped" in hook.stop()

    hook = rec.ArmHook("video", "/repo", popen=lambda argv, cwd=None, env=None: FakeProc(argv, dies=True),
                       free_bytes=lambda _: 40 * 1024**3)
    hook.start("x")
    report = hook.stop()
    assert report["died_early"] is True and report["exit_code"] == 1


def test_no_on_arm_command_means_nothing_runs():
    hook = rec.ArmHook(None, "/repo", popen=lambda *a, **k: 1 / 0)
    hook.start("x")
    assert hook.stop() == {}


# --- ground-station bridge ---------------------------------------------------

class _FakeSock:
    def __init__(self):
        self.inbox, self.sent, self.fail_sends = [], [], False

    def recvfrom(self, _n):
        if not self.inbox:
            raise BlockingIOError
        return self.inbox.pop(0)

    def sendto(self, data, addr):
        if self.fail_sends:
            raise OSError("Network is unreachable")
        self.sent.append((data, addr))


def _packets():
    """Real MAVLink 2 packets, built by the same library the recorder reads with."""
    os.environ["MAVLINK20"] = "1"
    from pymavlink.dialects.v20 import common as mav
    fc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    gcs = mav.MAVLink(None, srcSystem=255, srcComponent=190)
    parse = mav.MAVLink(None)
    parse.robust_parsing = True

    def decoded(m):
        return parse.parse_buffer(m.pack(fc))[0]
    return {
        "fc_heartbeat": decoded(mav.MAVLink_heartbeat_message(2, 12, 0, 0, 0, 3)),
        "fc_attitude": decoded(mav.MAVLink_attitude_message(1, 0, 0, 0, 0, 0, 0)),
        "fc_log": decoded(mav.MAVLink_logging_data_message(1, 155, 7, 3, 255, [0] * 249)),
        "fc_ack_to_recorder": decoded(mav.MAVLink_command_ack_message(
            2510, 0, target_system=1, target_component=155)),
        "gcs_heartbeat": mav.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3).pack(gcs),
        "gcs_param_list": mav.MAVLink_param_request_list_message(1, 1).pack(gcs),
    }


def _bridge():
    sock, now, to_fc = _FakeSock(), [100.0], []
    b = rec.GcsBridge(14550, sock=sock, clock=lambda: now[0])
    b.write_to_fc = to_fc.append
    return b, sock, now, to_fc


def test_bridge_announces_itself_then_talks_only_to_the_ground_station():
    b, sock, now, to_fc = _bridge()
    p = _packets()
    QGC = ("192.168.0.25", 14550)
    # Nobody connected: only the Pixhawk's heartbeat goes out, as a broadcast.
    b.to_gcs(p["fc_attitude"])
    b.to_gcs(p["fc_heartbeat"])
    assert sock.sent == [(bytes(p["fc_heartbeat"].get_msgbuf()), ("255.255.255.255", 14550))]
    # The ground station answers; what it sends reaches the Pixhawk byte for byte.
    sock.inbox += [(p["gcs_heartbeat"], QGC), (p["gcs_param_list"], QGC)]
    now[0] += 1.0
    b.pump()
    assert to_fc == [p["gcs_heartbeat"], p["gcs_param_list"]]
    # From now on telemetry goes to it directly, and the log packets do not.
    sock.sent.clear()
    for name in ("fc_attitude", "fc_log", "fc_ack_to_recorder", "fc_heartbeat"):
        b.to_gcs(p[name])
    assert sock.sent == [(bytes(p["fc_attitude"].get_msgbuf()), QGC),
                         (bytes(p["fc_heartbeat"].get_msgbuf()), QGC)]
    report = b.take_report()
    assert report["ground_stations_seen"] == ["192.168.0.25"]
    assert report["bytes_from_ground_station"] == len(p["gcs_heartbeat"]) + len(p["gcs_param_list"])


def test_bridge_ignores_its_own_broadcast_and_anything_that_is_not_mavlink():
    b, sock, now, to_fc = _bridge()
    p = _packets()
    echo = bytes(p["fc_heartbeat"].get_msgbuf())
    sock.inbox += [(echo, ("192.168.0.61", 14550)), (b"GET / HTTP/1.1\r\n", ("192.168.0.9", 5000)),
                   (b"", ("192.168.0.9", 5001))]
    now[0] += 1.0
    b.pump()
    assert to_fc == [] and b.clients == {}


def test_bridge_drops_a_silent_ground_station_and_survives_a_dead_network():
    b, sock, now, to_fc = _bridge()
    p = _packets()
    sock.inbox.append((p["gcs_heartbeat"], ("192.168.0.25", 14550)))
    now[0] += 1.0
    b.pump()
    assert len(b.clients) == 1
    sock.fail_sends = True
    b.to_gcs(p["fc_attitude"])              # must not raise: the recording comes first
    now[0] += rec.GCS_SILENT_AFTER_S + 1.0
    b.pump()
    assert b.clients == {}
    b.to_gcs(p["fc_heartbeat"])             # back to announcing, network still down
    assert b.take_report()["bytes_to_ground_station"] == 0
