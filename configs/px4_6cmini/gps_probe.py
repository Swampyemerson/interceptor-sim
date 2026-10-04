#!/usr/bin/env python3
"""Quick GPS presence probe for the 6C Mini (READ-ONLY).

RUNS ON *WINDOWS* PYTHON:
    python.exe configs/px4_6cmini/gps_probe.py --port COM5

Watches GPS_RAW_INT + STATUSTEXT for ~10 s and reads GPS_1_CONFIG.
Indoors "no fix" is expected; "no messages at all" means the module is
not talking (unplugged / unseated connector).
"""
import argparse
import os
import struct
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.port, baud=115200)
    m.wait_heartbeat(timeout=10)
    if m.target_component == 0:
        m.target_component = 1

    m.mav.param_request_read_send(
        m.target_system, m.target_component, b"GPS_1_CONFIG", -1)
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
    if msg and msg.param_id.rstrip("\x00") == "GPS_1_CONFIG":
        v = struct.unpack("<i", struct.pack("<f", msg.param_value))[0] \
            if msg.param_type == 6 else msg.param_value
        print(f"GPS_1_CONFIG = {v}  (201 = GPS1 port)")

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        24, 500000, 0, 0, 0, 0, 0)  # GPS_RAW_INT at 2 Hz

    n = 0
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        msg = m.recv_match(type=["GPS_RAW_INT", "STATUSTEXT"],
                           blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"  STATUSTEXT: {msg.text}")
            continue
        n += 1
        if n <= 3:
            print(f"  GPS_RAW_INT: fix_type={msg.fix_type} sats={msg.satellites_visible}")
    print(f"GPS_RAW_INT messages in {args.seconds:.0f}s: {n}")
    print("VERDICT:", "module talking" if n else
          "NO GPS DATA -- module not detected (check GPS1 connector seating)")
    return 0 if n else 1


if __name__ == "__main__":
    raise SystemExit(main())
