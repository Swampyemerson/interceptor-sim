"""scripts/field/crsf_cue_source.py: CRSF telemetry bytes in, cue datagrams out.

Frames are BUILT here from the layout in the TBS CRSF specification (sync,
length, type, payload, CRC-8 polynomial 0xD5, big endian), with an independent
bit-by-bit CRC, so a mistake in the module's own table or `build_frame` cannot
hide behind itself.

The datagram is held to the RECEIVER's own code (producer -> consumer, the
fixture comes from the consumer's writer): it must equal
`flight.cue_link.encode_record` byte for byte and be accepted by
`flight.cue_link.CueReceiver.feed`.

Run: `.venv/bin/python -m pytest -q tests/test_crsf_cue_source.py`
"""
import importlib.util
import io
import json
import math
import os
import socket
import struct
import subprocess
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(REPO, "scripts", "field", "crsf_cue_source.py")
sys.path.insert(0, REPO)

_spec = importlib.util.spec_from_file_location("crsf_cue_source", SCRIPT)
ccs = importlib.util.module_from_spec(_spec)
sys.modules["crsf_cue_source"] = ccs
_spec.loader.exec_module(ccs)


# ------------------------------------------------- independent frame builder

def ref_crc8(data: bytes) -> int:
    """CRC-8 poly 0xD5, init 0, MSB first -- bit by bit, no table."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(ftype: int, payload: bytes, sync: int = 0xC8) -> bytes:
    body = bytes([ftype]) + payload
    return bytes([sync, len(body) + 1]) + body + bytes([ref_crc8(body)])


def gps_payload(lat_e7, lon_e7, gs_kmh_x10, hdg_deg_x100, alt_m_plus_1000, sats):
    return struct.pack(">iiHHHB", lat_e7, lon_e7, gs_kmh_x10, hdg_deg_x100,
                       alt_m_plus_1000, sats)


# A fix near Boulder: 9 m/s (32.4 km/h -> 324) on course 060.00, 1655 m, 14 sats.
LAT_E7, LON_E7 = 400150123, -1052705456
GPS = gps_payload(LAT_E7, LON_E7, 324, 6000, 2655, 14)
GPS_FRAME = frame(0x02, GPS)
ATT_FRAME = frame(0x1E, struct.pack(">hhh", 100, -200, 15000))
BATT_FRAME = frame(0x08, struct.pack(">hh", 158, 42) + b"\x00\x01\x90" + b"\x4b")
LINK_FRAME = frame(0x14, bytes([60, 60, 100, 8, 0, 4, 2, 55, 100, 9]), sync=0xEA)


def collect():
    sent = []
    return sent, ccs.CueSource(sent.append, err=io.StringIO())


# ------------------------------------------------------------------ framing

def test_crc_matches_the_standard_check_value_and_the_spec_table():
    # CRC-8/DVB-S2 (poly 0xD5, init 0, no reflection) check value
    assert ccs.crc8_d5(b"123456789") == 0xBC
    assert ref_crc8(b"123456789") == 0xBC
    # first entries of the lookup table printed in the TBS specification
    assert [ccs.crc8_d5(bytes([i])) for i in range(8)] == [
        0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54]


def test_spec_example_frames_parse():
    # two complete frames quoted in the TBS specification (type 0x32 commands)
    for text in ("C8 07 32 EE 13 13 01 66 06", "C8 07 32 EE 13 13 02 12 8E"):
        raw = bytes.fromhex(text)
        parser = ccs.CrsfParser()
        (f,) = parser.feed(raw, 1.0)
        assert (f.sync, f.type, f.payload) == (0xC8, 0x32, raw[3:-1])
        assert parser.n_crc_fail == 0


def test_module_builder_agrees_with_the_independent_one():
    assert ccs.build_frame(0x02, GPS) == GPS_FRAME
    assert len(GPS_FRAME) == 19 and GPS_FRAME[1] == 17
    assert ccs.build_gps_payload(40.0150123, -105.2705456, 9.0, 60.0, 1655, 14) == GPS


def test_gps_decode_values():
    fix = ccs.decode_gps(GPS)
    assert fix.lat == pytest.approx(40.0150123, abs=1e-9)
    assert fix.lon == pytest.approx(-105.2705456, abs=1e-9)
    assert fix.speed_ms == pytest.approx(9.0, abs=1e-9)        # 32.4 km/h
    assert fix.course_deg == pytest.approx(60.0)
    assert fix.alt_m_msl == 1655.0
    assert fix.sats == 14
    assert fix.vn == pytest.approx(9.0 * math.cos(math.radians(60.0)))
    assert fix.ve == pytest.approx(9.0 * math.sin(math.radians(60.0)))


@pytest.mark.parametrize("course,vn,ve", [
    (0, 1.0, 0.0), (9000, 0.0, 1.0), (18000, -1.0, 0.0), (27000, 0.0, -1.0)])
def test_course_is_degrees_clockwise_from_north(course, vn, ve):
    fix = ccs.decode_gps(gps_payload(LAT_E7, LON_E7, 36, course, 1100, 9))
    assert fix.speed_ms == pytest.approx(1.0)                 # 3.6 km/h
    assert fix.vn == pytest.approx(vn, abs=1e-12)
    assert fix.ve == pytest.approx(ve, abs=1e-12)


def test_negative_latitude_and_altitude_below_the_offset():
    fix = ccs.decode_gps(gps_payload(-338688000, 1512093000, 0, 0, 990, 7))
    assert fix.lat == pytest.approx(-33.8688) and fix.lon == pytest.approx(151.2093)
    assert fix.alt_m_msl == -10.0


def test_wrong_size_gps_payload_is_not_guessed_at():
    with pytest.raises(ValueError):
        ccs.decode_gps(GPS[:-1])
    sent, src = collect()
    src.feed(frame(0x02, GPS[:-1]), 1.0)
    assert sent == [] and src.n_gps_malformed == 1 and src.n_gps == 1


def test_frames_split_across_reads_byte_by_byte():
    stream = ATT_FRAME + GPS_FRAME + BATT_FRAME + GPS_FRAME
    parser = ccs.CrsfParser()
    got = []
    for i, b in enumerate(stream):
        got += parser.feed(bytes([b]), float(i))
    assert [f.type for f in got] == [0x1E, 0x02, 0x08, 0x02]
    assert got[1].payload == GPS
    # each frame carries the clock of the read that delivered its LAST byte
    ends = []
    pos = 0
    for raw in (ATT_FRAME, GPS_FRAME, BATT_FRAME, GPS_FRAME):
        pos += len(raw)
        ends.append(float(pos - 1))
    assert [f.t_read for f in got] == ends
    assert parser.n_crc_fail == 0 and parser.n_skipped_bytes == 0
    assert parser.pending_bytes == 0


def test_frame_split_in_two_reads_takes_the_second_reads_time():
    parser = ccs.CrsfParser()
    assert parser.feed(GPS_FRAME[:7], 10.0) == []
    (f,) = parser.feed(GPS_FRAME[7:] + ATT_FRAME[:3], 10.5)
    assert f.type == 0x02 and f.t_read == 10.5
    (g,) = parser.feed(ATT_FRAME[3:], 11.0)
    assert g.type == 0x1E and g.t_read == 11.0


def test_frame_delayed_behind_a_false_start_keeps_its_own_read_time():
    # 0xC8 then 60 looks like the start of a 62-byte frame, so the parser has
    # to wait for more bytes before it can rule that out. The real frame behind
    # it was fully read at t = 1.0 and must say so, not 2.0.
    parser = ccs.CrsfParser()
    assert parser.feed(bytes([0xC8, 60]) + GPS_FRAME, 1.0) == []
    (f,) = parser.feed(bytes(41), 2.0)
    assert f.type == 0x02 and f.payload == GPS
    assert f.t_read == 1.0
    assert parser.n_crc_fail == 1


def test_bad_crc_is_counted_and_the_next_frame_survives():
    bad = bytearray(GPS_FRAME)
    bad[5] ^= 0x01                          # one flipped payload bit
    sent, src = collect()
    src.feed(bytes(bad) + ATT_FRAME + GPS_FRAME, 1.0)
    assert src.parser.n_crc_fail >= 1
    assert src.by_type[0x1E] == 1 and src.by_type[0x02] == 1
    assert len(sent) == 1                   # only the intact GPS frame


def test_bad_crc_byte_itself():
    bad = GPS_FRAME[:-1] + bytes([GPS_FRAME[-1] ^ 0xFF])
    sent, src = collect()
    src.feed(bad, 1.0)
    assert sent == [] and src.n_gps == 0 and src.parser.n_crc_fail >= 1


def test_truncated_frame_is_never_emitted():
    sent, src = collect()
    src.feed(ATT_FRAME + GPS_FRAME[:12], 1.0)       # stream ends mid-frame
    assert src.by_type[0x1E] == 1 and src.n_gps == 0 and sent == []
    assert src.parser.pending_bytes == 12
    assert src.summary()["pending_bytes_at_end"] == 12


def test_truncated_frame_followed_by_a_good_one():
    # a frame cut short (bytes lost), then a whole frame: the stub fails its
    # CRC or length test and the whole frame behind it is still found
    sent, src = collect()
    src.feed(GPS_FRAME[:9] + GPS_FRAME + ATT_FRAME, 1.0)
    assert src.by_type[0x02] == 1 and src.by_type[0x1E] == 1
    assert len(sent) == 1


def test_garbage_between_frames_including_false_sync_bytes():
    garbage = bytes([0x00, 0xFF, 0xC8, 0x05, 0x11, 0x22, 0xEA, 0x90, 0x13, 0xC8])
    sent, src = collect()
    src.feed(garbage + GPS_FRAME + garbage[:6] + ATT_FRAME + b"\x55\xaa"
             + GPS_FRAME, 1.0)
    assert src.by_type[0x02] == 2 and src.by_type[0x1E] == 1
    assert len(sent) == 2
    assert src.parser.n_skipped_bytes > 0


def test_frame_inside_an_msp_style_wrapper_is_found():
    # the Wi-Fi backpack path wraps each CRSF frame in another packet
    wrapped = b"$X<\x00\x11\x00" + bytes([len(GPS_FRAME), 0]) + GPS_FRAME + b"\x7e"
    sent, src = collect()
    src.feed(wrapped, 1.0)
    assert len(sent) == 1


def test_handset_address_is_accepted_as_a_frame_start():
    sent, src = collect()
    src.feed(frame(0x02, GPS, sync=0xEA) + LINK_FRAME, 1.0)
    assert len(sent) == 1 and src.by_type[0x14] == 1


def test_other_frame_types_are_counted_not_sent():
    sent, src = collect()
    src.feed(ATT_FRAME + BATT_FRAME + LINK_FRAME + ATT_FRAME, 1.0)
    assert sent == []
    assert dict(src.by_type) == {0x1E: 2, 0x08: 1, 0x14: 1}
    assert src.n_gps == 0


def test_length_byte_outside_the_valid_range_is_not_a_frame():
    parser = ccs.CrsfParser()
    assert parser.feed(bytes([0xC8, 0x01, 0x02, 0x00]) + bytes([0xC8, 63]) + bytes(70), 1.0) == []
    with pytest.raises(ValueError):
        ccs.build_frame(0x02, bytes(61))


# ----------------------------------------------------------------- datagram

def test_datagram_contents():
    sent, src = collect()
    src.feed(GPS_FRAME, 123.25)
    src.feed(ATT_FRAME + GPS_FRAME, 123.75)
    assert len(sent) == 2
    assert all(d.endswith(b"\n") and d.count(b"\n") == 1 for d in sent)
    a, b = (json.loads(d) for d in sent)
    assert set(a) == {"lat", "lon", "alt_m_msl", "vn", "ve", "vd", "t_src", "seq"}
    assert a["lat"] == pytest.approx(40.0150123, abs=1e-9)
    assert a["lon"] == pytest.approx(-105.2705456, abs=1e-9)
    assert a["alt_m_msl"] == 1655.0
    assert a["vn"] == pytest.approx(4.5) and a["ve"] == pytest.approx(7.794228634)
    assert a["vd"] == 0.0
    assert (a["t_src"], a["seq"]) == (123.25, 0)
    assert (b["t_src"], b["seq"]) == (123.75, 1)


def test_datagram_equals_the_receivers_own_writer_and_is_accepted_by_it():
    from flight.cue_link import DEFAULT_PORT, CueReceiver, encode_record
    from flight.cue_relay import CueRecord

    assert ccs.DEFAULT_PORT == DEFAULT_PORT == 47810
    sent, src = collect()
    src.feed(GPS_FRAME, 50.0)
    fix = ccs.decode_gps(GPS)
    expected = encode_record(
        CueRecord(t=0.0, lat=fix.lat, lon=fix.lon, alt_m_msl=fix.alt_m_msl,
                  vn=fix.vn, ve=fix.ve, vd=0.0), t_src=50.0, seq=0)
    assert sent[0] == expected

    rx = CueReceiver()
    assert rx.feed(sent[0], t_rx=60.0) == 1
    stats = rx.stats(now_t=60.0)
    assert (stats.n_rx, stats.n_bad) == (1, 0)
    (rec,) = rx._records_on_local_clock()
    assert rec.lat == fix.lat and rec.alt_m_msl == 1655.0
    assert math.hypot(rec.vn, rec.ve) == pytest.approx(9.0)
    assert rec.vd == 0.0


def test_a_two_hertz_stream_passes_the_receivers_fit():
    """A straight 9 m/s track sampled at 2 Hz (ArduPilot 4.7's ceiling for
    this frame) for 5.5 s, through real frames, is latched by the receiver
    with its default gates."""
    from flight.cue_link import CueReceiver

    own = (40.0, -105.0, 1650.0)
    course, speed = 60.0, 9.0
    vn, ve = speed * math.cos(math.radians(course)), speed * math.sin(math.radians(course))
    sent, src = collect()
    rx = CueReceiver()
    t = 100.0
    for k in range(12):
        dt = 0.5 * k
        lat = own[0] + math.degrees((50.0 + vn * dt) / 6378137.0)
        lon = own[1] + math.degrees((20.0 + ve * dt) / (6378137.0 * math.cos(math.radians(own[0]))))
        raw = frame(0x02, ccs.build_gps_payload(lat, lon, speed, course, 1680, 12))
        src.feed(ATT_FRAME + raw, t + dt)
        rx.feed(sent[-1], t_rx=t + dt + 0.08)       # 80 ms of link delay
    assert len(sent) == 12
    sol = rx.latch(*own, now_t=t + 5.5 + 0.08 + 0.2)
    assert sol.n_points == 12
    assert sol.speed_ms == pytest.approx(speed, abs=0.2)
    assert sol.heading_deg == pytest.approx(course, abs=2.0)


# ------------------------------------------------------- no fix: fail closed

@pytest.mark.parametrize("payload,why", [
    (gps_payload(LAT_E7, LON_E7, 324, 6000, 2655, 0), "zero satellites"),
    (gps_payload(LAT_E7, LON_E7, 324, 6000, 2655, 3), "only 3 satellites"),
    (gps_payload(0, 0, 0, 0, 1000, 9), "both exactly zero"),
    (gps_payload(910000000, LON_E7, 0, 0, 1000, 9), "out of range"),
    (gps_payload(LAT_E7, 1810000000, 0, 0, 1000, 9), "out of range"),
    (gps_payload(LAT_E7, LON_E7, 324, 36001, 2655, 9), "course"),
])
def test_no_fix_is_refused_loudly_and_never_sent(payload, why):
    sent = []
    err = io.StringIO()
    src = ccs.CueSource(sent.append, err=err)
    src.feed(frame(0x02, payload) + GPS_FRAME, 5.0)
    assert len(sent) == 1                       # only the good frame went out
    assert json.loads(sent[0])["seq"] == 0      # a refusal uses no sequence number
    assert src.n_refused == 1 and src.n_gps == 2
    assert "NOT SENT" in err.getvalue() and why in err.getvalue()
    assert "no-fix 1" in src.status_line(6.0)


def test_check_fix_passes_a_real_fix_and_min_sats_is_adjustable():
    ccs.check_fix(ccs.decode_gps(GPS))
    three = ccs.decode_gps(gps_payload(LAT_E7, LON_E7, 0, 0, 1000, 3))
    ccs.check_fix(three, min_sats=3)
    with pytest.raises(ccs.NoGpsFix):
        ccs.check_fix(three)
    zero = ccs.decode_gps(gps_payload(LAT_E7, LON_E7, 0, 0, 1000, 0))
    with pytest.raises(ccs.NoGpsFix):
        ccs.check_fix(zero, min_sats=0)         # zero satellites: never


def test_send_failure_is_counted_and_the_gap_shows_in_seq():
    calls = []

    def flaky(datagram):
        calls.append(datagram)
        if len(calls) == 2:
            raise OSError("network is unreachable")

    src = ccs.CueSource(flaky, err=io.StringIO())
    src.feed(GPS_FRAME * 3, 1.0)
    assert src.n_sent == 2 and src.n_send_errors == 1
    assert [json.loads(d)["seq"] for d in calls] == [0, 1, 2]


# ------------------------------------------------------------- status + CLI

def test_status_line_reports_rates_per_type_and_crc_failures():
    sent, src = collect()
    src.status_line(0.0)                        # start the interval at t = 0
    bad = GPS_FRAME[:-1] + bytes([GPS_FRAME[-1] ^ 0x5A])
    src.feed(GPS_FRAME + ATT_FRAME + GPS_FRAME + bad + LINK_FRAME, 0.5)
    line = src.status_line(1.0)
    assert "\n" not in line
    assert "GPS(0x02)=2.0" in line and "ATT(0x1E)=1.0" in line
    assert "LINK(0x14)=1.0" in line
    assert "GPS 2.0 Hz" in line and "sent 2.0 Hz" in line
    assert "CRC fail" in line and "CRC fail 0" not in line
    quiet = src.status_line(2.0)                # nothing arrived this second
    assert "NO BYTES ARRIVING" in quiet and "GPS 0.0 Hz" in quiet


def test_module_does_not_need_pyserial_to_import_or_to_read_a_file(tmp_path):
    raw = tmp_path / "crsf.bin"
    raw.write_bytes(b"\x00\x13" + ATT_FRAME + GPS_FRAME + BATT_FRAME + GPS_FRAME[:10])
    # an empty directory first on the path with a `serial` that cannot import
    (tmp_path / "block").mkdir()
    (tmp_path / "block" / "serial.py").write_text("raise ImportError('blocked')\n")
    env = dict(os.environ, PYTHONPATH=str(tmp_path / "block"))
    out = subprocess.run([sys.executable, SCRIPT, "--file", str(raw), "--dry-run"],
                         capture_output=True, text=True, env=env, timeout=60)
    assert out.returncode == 0, out.stderr
    summary = json.loads(out.stdout.strip().splitlines()[-1])
    assert summary["sent"] == 1 and summary["gps_frames"] == 1
    assert summary["frames_by_type"] == {"0x02": 1, "0x08": 1, "0x1E": 1}
    assert summary["pending_bytes_at_end"] == 10
    assert "[crsf]" in out.stdout


def test_cli_stdin_to_a_real_udp_socket(tmp_path):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(5.0)
    port = rx.getsockname()[1]
    try:
        out = subprocess.run(
            [sys.executable, SCRIPT, "--file", "-", "--to", f"127.0.0.1:{port}",
             "--record", str(tmp_path / "raw.bin")],
            input=ATT_FRAME + GPS_FRAME + GPS_FRAME, capture_output=True, timeout=60)
        assert out.returncode == 0, out.stderr
        rows = [json.loads(rx.recvfrom(4096)[0]) for _ in range(2)]
    finally:
        rx.close()
    assert [r["seq"] for r in rows] == [0, 1]
    assert rows[0]["lat"] == pytest.approx(40.0150123, abs=1e-9)
    assert rows[0]["t_src"] <= rows[1]["t_src"]
    assert (tmp_path / "raw.bin").read_bytes() == ATT_FRAME + GPS_FRAME + GPS_FRAME


def test_cli_exits_non_zero_when_nothing_could_be_sent(tmp_path):
    raw = tmp_path / "nofix.bin"
    raw.write_bytes(ATT_FRAME + frame(0x02, gps_payload(0, 0, 0, 0, 1000, 0)) * 3)
    out = subprocess.run([sys.executable, SCRIPT, "--file", str(raw), "--dry-run"],
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 1
    assert "FAILED: nothing was sent" in out.stderr
    assert "zero satellites" in out.stderr
    assert json.loads(out.stdout.strip().splitlines()[-1])["refused_no_fix"] == 3


def test_cli_needs_a_destination_unless_dry_run(tmp_path):
    raw = tmp_path / "x.bin"
    raw.write_bytes(GPS_FRAME)
    out = subprocess.run([sys.executable, SCRIPT, "--file", str(raw)],
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 2 and "--to" in out.stderr
