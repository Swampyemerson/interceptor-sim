#!/usr/bin/env python3
"""Catch an ArduPilot board's early boot STATUSTEXT chatter: waits for the COM
port to APPEAR (user replugs USB while this runs), opens it immediately, and
prints every STATUSTEXT for the capture window. For diagnosing boards that
halt in Config Error: subsystem probe results (GPS/compass/IMU) that print
BEFORE the halt are only visible this way.

    python.exe scripts/bench/fc_boot_log.py --port COM7 --wait 60 --capture 25
"""
from __future__ import annotations
import argparse, time

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wait", type=float, default=60.0)
    ap.add_argument("--capture", type=float, default=25.0)
    args = ap.parse_args()
    from serial.tools import list_ports
    def present():
        return any(p.device == args.port for p in list_ports.comports())
    t0 = time.time()
    if present():
        print(f"{args.port} is present -- waiting for it to VANISH (unplug USB now)")
        while present():
            if time.time() - t0 > args.wait:
                print("FAIL: port never vanished"); return 2
            time.sleep(0.25)
        print("port gone -- now plug USB back in")
    else:
        print(f"waiting up to {args.wait:.0f}s for {args.port} to appear -- plug USB in now")
    while time.time() - t0 < args.wait:
        if present():
            break
        time.sleep(0.25)
    else:
        print("FAIL: port never appeared"); return 2
    print(f"{args.port} appeared after {time.time()-t0:.1f}s -- settling 2s, then opening")
    time.sleep(2.0)
    from pymavlink import mavutil
    m = None
    for attempt in range(4):
        try:
            m = mavutil.mavlink_connection(args.port, baud=args.baud)
            break
        except Exception as e:
            print(f"(open attempt {attempt+1} failed: {e.__class__.__name__}; retrying)")
            time.sleep(1.0)
    if m is None:
        print("FAIL: could not open after retries"); return 2
    t1 = time.time()
    seen = set()
    while time.time() - t1 < args.capture:
        try:
            msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1.0)
        except Exception as e:
            print(f"(serial dropped: {e.__class__.__name__} -- waiting for the port to return)")
            td = time.time()
            while time.time() - td < 20.0 and not present():
                time.sleep(0.3)
            if not present():
                print("(port did not return)"); break
            time.sleep(2.5)
            try:
                m = mavutil.mavlink_connection(args.port, baud=args.baud)
                print("(reconnected)")
            except Exception as e2:
                print(f"(reopen failed: {e2.__class__.__name__})"); break
            continue
        if msg is None:
            continue
        line = msg.text
        key = line
        if key in seen and "Config Error" in line:
            continue  # print the halt loop once, not forever
        seen.add(key)
        print(f"[{time.time()-t1:5.1f}s] {line}")
    print("capture done")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
