"""Is the GPS really talking to an ArduPilot board, or is the board hearing itself?

Takes over the board's GPS1 port for a few seconds (MAVLink SERIAL_CONTROL,
exclusive) and, at each baud rate given:
  1. listens while sending nothing (a working u-blox is never silent);
  2. sends a nonsense test string and checks whether it comes straight back
     (a GPS never echoes; an echo means the board's GPS transmit and receive
     lines are connected to each other);
  3. asks the module who it is, what its fix is and which messages it is set to
     send (read-only polls).
Changes nothing on the module and hands the port back to the board's driver.

    python.exe ap_gps_wire_check.py COM4 result.json 230400 115200

Why it exists: ArduPilot prints "GPS 1: probing for u-blox at 230400 baud" as
soon as it sees ANY valid u-blox frame on the port, including its own
configuration messages echoed back. On 2026-11-14 the target's board had been
"detecting" its GPS that way since 2026-10-13 without ever receiving a byte
from it. The message that means a GPS is really there is "GPS 1: detected
u-blox". The board must be disarmed. ArduPilot only.
"""
import sys, time, struct, json
from pymavlink import mavutil
port, out = sys.argv[1], sys.argv[2]
bauds = [int(b) for b in sys.argv[3:]] or [230400]
DEV_GPS1, RESPOND, EXCL, MULTI = 2, 2, 4, 16
m = mavutil.mavlink_connection(port, baud=115200); assert m.wait_heartbeat(timeout=20)
hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3); assert not (hb.base_mode & 128), "armed"

def ubx(cls, mid, payload=b""):
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body: a = (a + x) & 255; b = (b + a) & 255
    return b"\xb5\x62" + body + bytes([a, b])

def xfer(baud, tx, secs):
    data = bytearray(); t0 = time.time(); first = True; last = 0
    while time.time() - t0 < secs:
        if time.time() - last > 0.05:
            buf = tx if first else b""
            m.mav.serial_control_send(DEV_GPS1, RESPOND | EXCL | MULTI, 50, baud, len(buf), list(buf) + [0] * (70 - len(buf)))
            first = False; last = time.time()
        msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.05)
        if msg is not None and msg.count: data += bytes(msg.data[:msg.count])
    return bytes(data)

def frames(d):
    out = []; i = 0
    while i < len(d) - 7:
        if d[i] == 0xB5 and d[i + 1] == 0x62:
            ln = d[i + 4] | d[i + 5] << 8
            if i + 8 + ln <= len(d):
                a = b = 0
                for x in d[i + 2:i + 6 + ln]: a = (a + x) & 255; b = (b + a) & 255
                if (a, b) == (d[i + 6 + ln], d[i + 7 + ln]):
                    out.append((d[i + 2], d[i + 3], d[i + 6:i + 6 + ln])); i += 8 + ln; continue
        i += 1
    return out

KEYS = {0x20910007: "NAV-PVT output rate on UART1", 0x40520001: "UART1 baud", 0x10740001: "UART1 sends UBX",
        0x10740002: "UART1 sends NMEA", 0x30210001: "measurement period ms", 0x209100ba: "NMEA GGA rate on UART1"}
res = {}
try:
    for b in bauds:
        r = res[str(b)] = {}
        xfer(b, b"", 0.5)
        idle = xfer(b, b"", 3.0); r["bytes_heard_while_silent_3s"] = len(idle)
        d = xfer(b, ubx(0x0A, 0x04), 1.5); f = [x for x in frames(d) if x[:2] == (0x0A, 0x04) and len(x[2]) >= 40]
        r["answers_who_are_you"] = bool(f); r["raw_bytes_after_poll"] = len(d)
        if f:
            p = f[0][2]; r["sw"] = p[0:30].split(b"\0")[0].decode("ascii", "replace"); r["hw"] = p[30:40].split(b"\0")[0].decode("ascii", "replace")
            r["ext"] = [p[40 + 30 * k:70 + 30 * k].split(b"\0")[0].decode("ascii", "replace") for k in range((len(p) - 40) // 30)]
        probe = bytes([0xB5, 0x62, 0x55, 0xAA, 0x04, 0x00, 0xDE, 0xAD, 0xBE, 0xEF]) + b"ECHO-TEST-1234567890"
        d = xfer(b, probe, 1.0)
        r["echo_test_sent_bytes"] = len(probe); r["echo_test_came_back"] = probe in d; r["echo_test_heard_hex"] = d[:48].hex()
        d = xfer(b, ubx(0x0A, 0x04), 1.5)
        r["who_are_you_heard_hex"] = d[:64].hex(); r["who_are_you_frames"] = [("%02X-%02X" % (a, c), len(pl)) for a, c, pl in frames(d)]
        d = xfer(b, ubx(0x01, 0x07), 1.5); f = [x for x in frames(d) if x[:2] == (0x01, 0x07) and len(x[2]) >= 92]
        if f:
            p = f[0][2]; r["fix_type"] = p[20]; r["fix_flags"] = p[21]; r["satellites_used"] = p[23]; r["pdop"] = struct.unpack_from("<H", p, 76)[0] / 100
        else: r["fix_type"] = "no answer"
        d = xfer(b, ubx(0x06, 0x8B, struct.pack("<BBH", 0, 0, 0) + b"".join(struct.pack("<I", k) for k in KEYS)), 1.5)
        f = [x for x in frames(d) if x[:2] == (0x06, 0x8B) and len(x[2]) > 4 + 4 * len(KEYS)]
        if f:
            p = f[0][2][4:]; i = 0; cfg = {}
            while i + 4 <= len(p):
                k = struct.unpack_from("<I", p, i)[0]; size = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}.get((k >> 28) & 7, 1)
                cfg[KEYS.get(k, hex(k))] = int.from_bytes(p[i + 4:i + 4 + size], "little"); i += 4 + size
            r["settings_in_use"] = cfg
        else: r["settings_in_use"] = "no answer (%d bytes, frames %s)" % (len(d), [(hex(a), hex(c), len(pl)) for a, c, pl in frames(d)])
        d = xfer(b, ubx(0x06, 0x01, bytes([0x01, 0x07])), 1.0); f = [x for x in frames(d) if x[:2] == (0x06, 0x01) and len(x[2]) >= 8]
        r["legacy_NAV_PVT_rates_per_port"] = list(f[0][2][2:]) if f else "no answer"
        print(b, json.dumps(r))
finally:
    for _ in range(3):
        m.mav.serial_control_send(DEV_GPS1, 0, 0, 0, 0, [0] * 70); time.sleep(0.2)
json.dump(res, open(out, "w"), indent=1)
