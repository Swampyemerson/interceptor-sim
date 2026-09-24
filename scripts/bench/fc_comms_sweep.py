#!/usr/bin/env python3
"""Comms sweep for a halted (Config Error) ArduPilot board: exercises every
channel that still works pre-sensor-init -- heartbeat, the PARAM protocol,
AUTOPILOT_VERSION, and an inventory of every message type the board emits in
the window. Says explicitly what CANNOT be judged in this state.

    python.exe scripts/bench/fc_comms_sweep.py --port COM7
"""
from __future__ import annotations
import argparse, time
from collections import Counter

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=10.0)
    args = ap.parse_args()
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.port, baud=args.baud)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("heartbeat: FAIL"); return 2
    print("heartbeat: OK (link + firmware main loop alive)")

    # PARAM protocol: read three params that matter.
    def fetch(name):
        for _ in range(4):
            m.mav.param_request_read_send(m.target_system, m.target_component,
                                          name.encode(), -1)
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
            while msg is not None and msg.param_id != name:
                msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
            if msg is not None:
                return msg.param_value
        return None
    ok = 0
    for name in ("SERIAL6_PROTOCOL", "SERVO_BLH_RVMASK", "WPNAV_SPEED"):
        v = fetch(name)
        print(f"param {name}: " + ("FAIL" if v is None else f"{v:g}"))
        if v is not None: ok += 1
    print(f"param protocol: {'OK' if ok == 3 else f'{ok}/3 -- degraded'} (flash/config readable)")

    # Firmware version block.
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                            0, 0, 0, 0, 0, 0)
    av = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3.0)
    if av:
        fw = av.flight_sw_version
        print(f"AUTOPILOT_VERSION: OK (fw {fw>>24 & 0xff}.{fw>>16 & 0xff}.{fw>>8 & 0xff})")
    else:
        print("AUTOPILOT_VERSION: no reply (not fatal)")

    # Inventory of everything the halted board volunteers.
    counts = Counter()
    t0 = time.time()
    while time.time() - t0 < args.secs:
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg is not None:
            counts[msg.get_type()] += 1
    print(f"message types seen in {args.secs:.0f}s: " +
          ", ".join(f"{k} x{v}" for k, v in counts.most_common()))
    print("NOT judgeable while the baro halts sensor init: GPS serial, compass, "
          "IMU health, RC stream, battery monitor -- all init after the halt point.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
