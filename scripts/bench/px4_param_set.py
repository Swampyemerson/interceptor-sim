#!/usr/bin/env python3
"""Write a handful of PX4 params over a Windows COM port, with READ-BACK.

PX4 ONLY (refuses any other autopilot). The PX4 landmine this exists for: an
INT32 param travels as its raw bytes reinterpreted in MAVLink's float field, so
each write first READS the param to learn the type the FC reports, encodes by
that type, then re-reads and refuses to report success unless every value
landed. A write echo is not proof -- follow with a reboot and a fresh
px4_param_dump.py, and diff the two dumps.

    python.exe px4_param_set.py --port COM5 --set SYS_AUTOSTART=4001 CA_ROTOR0_PX=1
    python.exe px4_param_set.py --port COM5 --reboot
"""
from __future__ import annotations

import argparse
import struct
import time

INT_TYPES = {1, 2, 3, 4, 5, 6}
TOL = 1e-5


def decode(msg):
    if msg.param_type in INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
    return float(msg.param_value)


def read(m, name, tries=4):
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, 1, name.encode(), -1)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is not None and msg.param_id == name:
                return msg
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--set", nargs="*", default=[], metavar="NAME=VALUE")
    ap.add_argument("--reboot", action="store_true")
    args = ap.parse_args()

    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.port, baud=args.baud,
                                   source_system=255, source_component=190)
    for _ in range(4):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.3)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("FAIL: no heartbeat")
        return 2
    if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        print("FAIL: not PX4 -- refusing (encoding here is PX4-specific)")
        return 2
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("FAIL: vehicle is ARMED -- refusing to write")
        return 2

    bad = 0
    for item in args.set:
        name, _, raw = item.partition("=")
        before = read(m, name)
        if before is None:
            print(f"FAIL {name}: param does not answer (unknown name?)")
            bad += 1
            continue
        is_int = before.param_type in INT_TYPES
        if is_int:
            want = int(raw)
            wire = struct.unpack("<f", struct.pack("<i", want))[0]
        else:
            want = float(raw)
            wire = want
        m.mav.param_set_send(m.target_system, 1, name.encode(), wire,
                             before.param_type)
        time.sleep(0.3)
        after = read(m, name)
        got = None if after is None else decode(after)
        ok = got is not None and (got == want if is_int else abs(got - want) < TOL)
        print(f"{'ok  ' if ok else 'FAIL'} {name}: {decode(before)} -> {got} "
              f"(wanted {want})")
        bad += 0 if ok else 1
    if bad:
        print(f"FAIL: {bad} parameter(s) did not land")
        return 1
    if args.reboot:
        m.mav.command_long_send(m.target_system, 1,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1)
        print("reboot sent -- give it ~25 s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
