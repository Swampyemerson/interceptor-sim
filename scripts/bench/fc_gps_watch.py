#!/usr/bin/env python3
"""Live GPS watch over a Windows COM port (ArduPilot): prints one line per
second -- module detected? fix_type (0=NO MODULE, 1=no fix, 2=2D, 3=3D), sats,
plus any GPS-related STATUSTEXT. For wiggle-testing an intermittent harness.

    python.exe scripts/bench/fc_gps_watch.py --port COM7 --secs 25
"""
from __future__ import annotations
import argparse, time

FIX = {0: "NO-MODULE", 1: "no-fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTKf", 6: "RTKF"}

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=25.0)
    args = ap.parse_args()
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.port, baud=args.baud)
    if m.wait_heartbeat(timeout=10) is None:
        print("FAIL: no heartbeat"); return 2
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    t0 = time.time()
    last = None
    last_print = 0.0
    while time.time() - t0 < args.secs:
        msg = m.recv_match(type=["GPS_RAW_INT", "STATUSTEXT"], blocking=True, timeout=1.0)
        now = time.time() - t0
        if msg is None:
            print(f"[{now:5.1f}s] (no GPS message this second)")
            continue
        if msg.get_type() == "STATUSTEXT":
            if "gps" in msg.text.lower() or "GPS" in msg.text:
                print(f"[{now:5.1f}s] STATUSTEXT: {msg.text}")
            continue
        state = (msg.fix_type, msg.satellites_visible)
        if state != last or now - last_print >= 1.0:
            print(f"[{now:5.1f}s] fix={FIX.get(msg.fix_type, msg.fix_type):9s} "
                  f"sats={msg.satellites_visible}")
            last, last_print = state, now
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
