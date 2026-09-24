#!/usr/bin/env python3
"""Write a handful of ARDUPILOT params over a Windows COM port, then READ EACH
BACK from a fresh fetch and refuse to report success unless every one landed
(the 2026-09-16 lesson: 'wrote 23/23' with 4 not landed). ArduPilot ONLY --
plain-float params; NEVER point at PX4 (int-encoding trap, see
configs/target_kakute/upload_params.py header).

    python.exe scripts/bench/fc_param_set.py --port COM7 --set SERVO1_FUNCTION=36 SERVO_BLH_RVMASK=6
    python.exe scripts/bench/fc_param_set.py --port COM7 --verify-only --set ...
    python.exe scripts/bench/fc_param_set.py --port COM7 --reboot
"""
from __future__ import annotations
import argparse, time

TOL = 1e-4

def fetch(m, name, tries=5):
    from pymavlink import mavutil
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        while msg is not None and msg.param_id != name:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if msg is not None:
            return float(msg.param_value)
    return None

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--set", nargs="*", default=[], metavar="NAME=VALUE")
    ap.add_argument("--verify-only", action="store_true")
    ap.add_argument("--reboot", action="store_true")
    args = ap.parse_args()
    kv = []
    for s in args.set:
        name, _, val = s.partition("=")
        kv.append((name.strip().upper(), float(val)))
    if not kv and not args.reboot:
        print("FAIL: nothing to do"); return 2

    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.port, baud=args.baud)
    if m.wait_heartbeat(timeout=10) is None:
        print("FAIL: no heartbeat"); return 2

    failed = 0
    for name, val in kv:
        before = fetch(m, name)
        if before is None:
            print(f"{name}: FAIL -- board does not report this param"); failed += 1; continue
        if not args.verify_only and abs(before - val) > TOL:
            m.mav.param_set_send(m.target_system, m.target_component,
                                 name.encode(), val,
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.3)
        after = fetch(m, name)
        ok = after is not None and abs(after - val) <= TOL
        print(f"{name}: was {before:g} -> read-back {after if after is None else round(after,6):g}"
              f"  target {val:g}  {'OK' if ok else 'FAIL'}")
        if not ok:
            failed += 1
    if args.reboot:
        print("rebooting the FC ...")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1)
    if failed:
        print(f"RESULT: {failed} of {len(kv)} params did NOT land"); return 1
    print(f"RESULT: all {len(kv)} landed" if kv else "RESULT: reboot sent")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
