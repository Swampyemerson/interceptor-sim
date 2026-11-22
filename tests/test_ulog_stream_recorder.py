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


# --- failure protection (2026-11-22) ------------------------------------------

def test_a_dead_on_arm_command_is_started_again_while_armed_but_not_for_ever():
    started, now = [], [0.0]

    def popen(argv, cwd=None, env=None):
        started.append(FakeProc(argv, cwd, env))
        return started[-1]

    hook = rec.ArmHook("video", "/repo", popen=popen, free_bytes=lambda _: 40 * 1024**3,
                       clock=lambda: now[0])
    hook.start("flight1")
    started[0].returncode = 2                # the camera recording dies in flight
    hook.start("flight1")                    # same instant: too soon to retry
    assert len(started) == 1 and hook.state() == "failed"
    now[0] += rec.HOOK_RETRY_EVERY_S + 0.1
    hook.start("flight1")
    assert len(started) == 2 and hook.running() and hook.state() == "recording"
    for _ in range(100):                     # a command that dies every time is not retried for ever
        started[-1].returncode = 2
        now[0] += rec.HOOK_RETRY_EVERY_S + 0.1
        hook.start("flight1")
    assert len(started) == rec.HOOK_MAX_STARTS
    report = hook.stop()
    assert report["starts"] == rec.HOOK_MAX_STARTS and len(report["died"]) == rec.HOOK_MAX_STARTS - 1
    assert report["died"][0]["exit_code"] == 2


def test_keep_alive_restarts_only_a_command_this_flight_already_started():
    started, now = [], [0.0]

    def popen(argv, cwd=None, env=None):
        started.append(FakeProc(argv, cwd, env))
        return started[-1]

    hook = rec.ArmHook("video", "/repo", popen=popen, free_bytes=lambda _: 40 * 1024**3,
                       clock=lambda: now[0])
    hook.keep_alive()                        # never armed: nothing to keep alive
    assert started == [] and hook.state() == "idle"
    hook.start("flight1")
    started[0].returncode = 2
    now[0] += rec.HOOK_RETRY_EVERY_S + 0.1
    hook.keep_alive()                        # link is away, armed state unknown: keep the video
    assert len(started) == 2 and started[1].env["FC_LOG_STAMP"] == "flight1"


def _fake_proc_tree(tmp_path, holders: dict):
    """A /proc look-alike: {pid: (name, [paths it has open])}."""
    for pid, (name, paths) in holders.items():
        fd = tmp_path / str(pid) / "fd"
        fd.mkdir(parents=True)
        (tmp_path / str(pid) / "comm").write_text(name + "\n")
        for i, path in enumerate(paths):
            os.symlink(path, fd / str(i))
    return str(tmp_path)


def test_the_serial_cable_is_left_alone_while_another_program_has_it_open(tmp_path):
    root = _fake_proc_tree(tmp_path, {"4100": ("mavsdk_server", ["/dev/null", "/dev/ttyAMA10"]),
                                      "4200": ("bash", ["/dev/null"]),
                                      str(os.getpid()): ("python", ["/dev/ttyAMA10"])})
    assert rec.others_holding("/dev/ttyAMA10", proc_root=root) == ["4100 mavsdk_server"]
    assert rec.others_holding("/dev/ttyAMA11", proc_root=root) == []
    have = lambda p: p == "/dev/ttyAMA10"
    busy = lambda path: rec.others_holding(path, proc_root=root)
    assert rec.pick_port("/dev/pixhawk-usb", "/dev/ttyAMA10", have, holders=busy) is None
    assert rec.pick_port("/dev/pixhawk-usb", "/dev/ttyAMA10", have, holders=lambda _: []) == "/dev/ttyAMA10"
    usb_too = lambda p: True
    assert rec.pick_port("/dev/pixhawk-usb", "/dev/ttyAMA10", usb_too, holders=busy) == "/dev/pixhawk-usb"


def test_status_file_is_whole_json_and_written_at_most_once_a_second(tmp_path):
    now = [10.0]
    path = tmp_path / "status.json"
    st = rec.StatusFile(str(path), clock=lambda: now[0])
    st.write(state="recording", gaps=0)
    st.write(state="recording", gaps=7)      # same second: skipped
    assert json.loads(path.read_text())["gaps"] == 0
    now[0] += 1.1
    st.write(state="recording", gaps=7)
    got = json.loads(path.read_text())
    assert got["gaps"] == 7 and got["updated_unix"] > 0
    rec.StatusFile(None).write(state="x")                       # no path: no file, no error
    rec.StatusFile("/nonexistent-dir/s.json").write(state="x")  # unwritable: never an error


def test_bridge_messages_reach_the_ground_station_and_never_the_pixhawk():
    b, sock, now, to_fc = _bridge()
    b.say("video recording")                 # nobody connected: nothing sent
    assert sock.sent == []
    sock.inbox.append((_packets()["gcs_heartbeat"], ("192.168.0.25", 14550)))
    b.pump()
    to_fc.clear()
    b.say("Pixhawk link LOST on /dev/pixhawk-usb WHILE ARMED; video keeps recording", 4)
    assert to_fc == [] and len(sock.sent) == 1
    os.environ["MAVLINK20"] = "1"
    from pymavlink.dialects.v20 import common as mav
    msg = mav.MAVLink(None).parse_buffer(sock.sent[0][0])[0]
    assert msg.get_type() == "STATUSTEXT" and msg.severity == 4
    assert msg.text.startswith("Pi: Pixhawk link LOST") and len(msg.text) <= 50
    assert (msg.get_srcSystem(), msg.get_srcComponent()) == (1, rec.MAV_COMP_ID_LOG)


class _FakeLink:
    """Stands in for the pymavlink connection: plays a script of messages, then the cable is pulled."""

    def __init__(self, script):
        self.script, self.sent, self.closed = list(script), [], False
        link = self

        class _Mav:
            def heartbeat_send(self, *a):
                pass

            def command_long_send(self, sysid, comp, cmd, *a):
                link.sent.append(cmd)
                link.script.insert(0, link.ack(cmd))

            def logging_ack_send(self, *a):
                pass
        self.mav = _Mav()

    @staticmethod
    def ack(cmd):
        os.environ["MAVLINK20"] = "1"
        from pymavlink.dialects.v20 import common as mav
        fc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        return mav.MAVLink(None).parse_buffer(mav.MAVLink_command_ack_message(
            cmd, 0, target_system=1, target_component=155).pack(fc))[0]

    def recv_match(self, blocking=True, timeout=None):
        if not self.script:
            raise OSError("device reports readiness to read but returned no data")
        return self.script.pop(0)

    def write(self, data):
        pass

    def close(self):
        self.closed = True


def _log_messages(capture: bytes, armed: bool, n: int = 100000):
    os.environ["MAVLINK20"] = "1"
    from pymavlink.dialects.v20 import common as mav
    fc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    parse = mav.MAVLink(None)
    hb = parse.parse_buffer(mav.MAVLink_heartbeat_message(2, 12, 128 if armed else 0, 0, 0, 3).pack(fc))[0]
    out = [hb]
    for seq, length, first, payload in packetize(capture)[:n]:
        out.append(parse.parse_buffer(mav.MAVLink_logging_data_message(
            1, 155, seq, length, first, list(payload)).pack(fc))[0])
    return out


def _recorder(tmp_path, link, hook):
    import argparse
    r = rec.Recorder.__new__(rec.Recorder)
    r.args = argparse.Namespace(
        out_dir=str(tmp_path), baud=921600, usb_port="/dev/pixhawk-usb", port="auto",
        no_stop_first=False, seconds=None, stall_s=10.0, pretend_armed=None, per_flight=True,
        linger_s=5.0, ground_rotate_min=10.0, leave_running=False)
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil
    r.mavutil = mavutil
    r.m, r.port, r.bridge, r.hook = link, "/dev/pixhawk-usb", None, hook
    r.status = rec.StatusFile(str(tmp_path / "status.json"))
    r.stop_requested = r.switch_to_usb = r.link_lost = False
    r.armed = r.real_armed = False
    r.last_beat, r.sysid = 0.0, 1
    return r


def test_a_pulled_cable_while_armed_closes_the_log_cleanly_and_leaves_the_video_running(capture, tmp_path):
    started = []

    def popen(argv, cwd=None, env=None):
        started.append(FakeProc(argv, cwd, env))
        return started[-1]

    hook = rec.ArmHook("video", "/repo", popen=popen, free_bytes=lambda _: 40 * 1024**3)
    r = _recorder(tmp_path, _FakeLink(_log_messages(capture, armed=True)), hook)
    stats = r.record_one()                   # runs until the fake cable is pulled: must not raise
    assert r.link_lost and stats["end_reason"].startswith("link lost")
    assert stats["verdict"] == "LINK_LOST" and stats["ever_armed"] is True
    assert stats["stop_result"] == "not sent (link lost)"
    assert stats["file"].endswith("_flight.ulg")
    assert len(ULog(stats["file"]).data_list) > 0, "what was received before the pull must open"
    assert hook.running() and started[0].signals == [], "the video must outlive the cable"
    assert stats["on_arm_command"]["still_running_into_next_file"] is True
    assert not [f for f in os.listdir(tmp_path) if f.endswith(".part")]

    # The cable comes back with the aircraft now disarmed: the video is closed after the linger.
    r2 = _recorder(tmp_path, _FakeLink(_log_messages(capture, armed=False, n=60)), hook)
    r2.args.linger_s = 0.0
    stats2 = r2.record_one()
    assert stats2["end_reason"] == "flight ended"
    assert not hook.running() and started[0].signals == [rec.signal.SIGINT]
    assert len(started) == 1


def test_a_pulled_cable_on_the_ground_is_not_a_crash_either(capture, tmp_path):
    hook = rec.ArmHook(None, "/repo")
    r = _recorder(tmp_path, _FakeLink(_log_messages(capture, armed=False)), hook)
    stats = r.record_one()
    assert r.link_lost and stats["verdict"] == "LINK_LOST" and stats["file"].endswith("_ground.ulg")


# --- go / no-go reader ----------------------------------------------------------

sys.path.insert(0, os.path.join(REPO, "scripts", "field"))
import recorder_check  # noqa: E402
import record_video  # noqa: E402

GOOD = {"state": "recording", "link": "usb", "port": "/dev/pixhawk-usb", "file": "fc_x.ulg.part",
        "recent_kb_per_s": 40.2, "last_gap_s_ago": None, "video": "idle", "free_gb": 37.0,
        "armed": False, "updated_unix": 1000.0}


def _go(st, now=1001.0):
    return all(ok for ok, _ in recorder_check.judge(st, now))


def test_recorder_check_passes_a_healthy_status_and_nothing_else():
    assert _go(GOOD)
    assert not _go(None), "no status file is a NO-GO, not a pass"
    assert not _go(GOOD, now=1000.0 + recorder_check.STALE_S + 1), "a stale status is a NO-GO"
    for bad in ({"state": "no link to the Pixhawk"}, {"link": "serial or other"},
                {"recent_kb_per_s": 12.0}, {"recent_kb_per_s": None}, {"last_gap_s_ago": 4.0},
                {"video": "failed"}, {"video": "off"}, {"free_gb": 6.0}, {"free_gb": None}):
        assert not _go({**GOOD, **bad}), bad
    assert _go({**GOOD, "video": "recording", "last_gap_s_ago": 300.0})


def test_recorder_check_reads_what_the_recorder_itself_writes(capture, tmp_path):
    """Producer to consumer: the status the record loop writes must be the one judge() accepts."""
    r = _recorder(tmp_path, _FakeLink(_log_messages(capture, armed=False, n=2000)), rec.ArmHook("video", "/repo"))
    clock = [0.0]
    real = rec.time.monotonic
    rec.time.monotonic = lambda: clock.__setitem__(0, clock[0] + 0.002) or clock[0]
    try:
        r.status = rec.StatusFile(str(tmp_path / "status.json"), clock=rec.time.monotonic)
        r.record_one()
    finally:
        rec.time.monotonic = real
    st = json.loads((tmp_path / "status.json").read_text())
    assert st["state"] == "recording" and st["link"] == "usb" and st["video"] == "idle"
    assert st["recent_kb_per_s"] >= recorder_check.MIN_KB_PER_S
    lines = recorder_check.judge(st, st["updated_unix"] + 1.0)
    assert [ok for ok, _ in lines] == [True] * len(lines), lines


def test_video_watchdog_fires_only_when_the_file_stops_growing():
    w = record_video.GrowthWatch(3.0, now=0.0)
    assert not w.stalled(0, 1.0) and not w.stalled(1000, 2.9)
    assert not w.stalled(2000, 5.0) and not w.stalled(2000, 7.9)
    assert w.stalled(2000, 8.1)
