#!/usr/bin/env python3
"""Run READ-ONLY status commands in the PX4 shell over an existing MAVLink link.

The Pixhawk's console is reachable through any MAVLink link (the Pi's serial
cable, USB), so status questions do not need QGroundControl or a debug cable:

    px4_shell_readonly.py /dev/ttyAMA10 921600 "logger status" "mavlink status" dmesg
    python.exe px4_shell_readonly.py COM5 115200 "param show SDLOG*"

Only commands on the ALLOWED list run; anything else is refused before it is
sent. The list holds status and listing commands only: nothing that sets a
parameter, starts or stops a module, or touches an actuator.
"""
import os
os.environ.setdefault("MAVLINK20", "1")
import sys, time
from pymavlink import mavutil
ALLOWED = ("logger status", "mavlink status", "dmesg", "ls /fs", "ls /fs/microsd", "mount", "df",
           "ver all", "free", "top once", "uorb status ulog_stream", "listener ", "perf", "work_queue status",
           "param show ")
port, baud = sys.argv[1], int(sys.argv[2]); cmds = sys.argv[3:]
for c in cmds:  # refuse BEFORE opening the link
    if not any(c.startswith(a) for a in ALLOWED):
        raise SystemExit(f"REFUSED, not on the read-only list: {c!r}")
m = mavutil.mavlink_connection(port, baud=baud, source_system=255, source_component=190)
for _ in range(3):
    m.mav.heartbeat_send(18, 8, 0, 0, 0); time.sleep(0.3)  # onboard computer, not a ground station
assert m.wait_heartbeat(timeout=10) is not None, "no heartbeat"
DEV = 10  # SERIAL_CONTROL_DEV_SHELL
def send(s):
    b = s.encode()
    while b:
        chunk, b = b[:70], b[70:]
        buf = list(chunk) + [0] * (70 - len(chunk))
        m.mav.serial_control_send(DEV, 2 | 4, 0, 0, len(chunk), buf)
def drain(secs):
    out = b""; t = time.time(); last = time.time()
    while time.time() - t < secs:
        msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.3)
        if msg is None:
            if out and time.time() - last > 1.2: break
            m.mav.serial_control_send(DEV, 2 | 4, 0, 0, 0, [0] * 70)
            continue
        if msg.count:
            out += bytes(msg.data[:msg.count]); last = time.time()
    return out.decode(errors="replace")
send("\n"); drain(1.5)
for c in cmds:
    send(c + "\n")
    print(f"=== {c}\n{drain(8)}\n", flush=True)
