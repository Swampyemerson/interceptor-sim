#!/usr/bin/env python3
"""Bench motor test for the ARDUPILOT Kakute: spin ONE motor letter, low
throttle, props off. Prints the COMMAND_ACK and any STATUSTEXT so a refusal
is loud, never silent.

NOTE ArduPilot's motor-test indexing is the A/B/C/D CLOCKWISE-FROM-FRONT-RIGHT
sequence (Mission Planner letters), NOT the motor output numbers:
  1=A front-right, 2=B back-right, 3=C back-left, 4=D front-left (quad X).
Expected spin directions (viewed from above), ArduPilot quad X:
  A front-right CCW · B back-right CW · C back-left CCW · D front-left CW.

    python.exe scripts/bench/fc_motor_test.py --port COM7 --motor 1
    python.exe scripts/bench/fc_motor_test.py --port COM7 --motor 1 --pct 8 --secs 3
"""
from __future__ import annotations
import argparse, time

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--motor", type=int, help="1..4 = A..D sequence letter")
    ap.add_argument("--all", action="store_true", help="spin all four together")
    ap.add_argument("--pct", type=float, default=8.0)
    ap.add_argument("--secs", type=float, default=3.0)
    args = ap.parse_args()
    if not args.all and (args.motor is None or not (1 <= args.motor <= 4)):
        print("FAIL: --motor must be 1..4 (or use --all)"); return 2
    if args.pct > 15:
        print("FAIL: bench cap is 15%% throttle"); return 2

    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.port, baud=args.baud)
    if m.wait_heartbeat(timeout=10) is None:
        print("FAIL: no heartbeat"); return 2
    motors = [1, 2, 3, 4] if args.all else [args.motor]
    print(f"connected sys {m.target_system}; spinning "
          f"{'ALL FOUR' if args.all else 'motor ' + 'ABCD'[args.motor-1]} "
          f"at {args.pct}% for {args.secs}s")
    for mi in motors:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
            mi,                                           # sequence letter index
            mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,  # throttle type
            args.pct, args.secs,
            0, 0, 0)
        time.sleep(0.05)
    t0 = time.time()
    acked = False
    while time.time() - t0 < args.secs + 4:
        msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            res = mavutil.mavlink.enums["MAV_RESULT"][msg.result].name
            print(f"ACK: {res}")
            acked = True
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                pass  # keep listening for the STATUSTEXT that explains it
        elif msg.get_type() == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")
    if not acked:
        print("WARN: no COMMAND_ACK seen")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
