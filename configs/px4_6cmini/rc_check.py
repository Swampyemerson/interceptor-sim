#!/usr/bin/env python3
"""RC-link bench check for the 6C Mini (afr-03): watch RC_CHANNELS over USB.

RUNS ON *WINDOWS* PYTHON (the FC enumerates as a COM port WSL2 cannot see):
    python.exe rc_check.py --port COM5 [--seconds 12]

Prints one line per second: channel count, ch1..ch8, plus any STATUSTEXT.
CORRECTED 2026-10-04 (review finding): in PX4 v1.16 "Manual control
lost/regained" is an EVENTS-protocol message, NOT a STATUSTEXT — so a TX-off
test prints no announcement here; the per-second RC_CHANNELS lines simply
STOP (and resume on relink). Watch for the gap, or use QGC, which decodes
events. "Kill engaged/disengaged" IS a mavlink_log STATUSTEXT and does show.
Exit 0 = RC frames seen; exit 1 = none.
No arming, no parameter writes, read-only.
"""
import argparse, sys, time

from pymavlink import mavutil


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--seconds", type=float, default=12.0)
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.port, baud=115200)
    m.wait_heartbeat(timeout=10)
    if m.target_component == 0:
        m.target_component = 1  # address the autopilot, not broadcast
    print(f"heartbeat from sys {m.target_system} comp {m.target_component}")

    # Ask for RC_CHANNELS (#65) at 5 Hz; harmless if already streaming.
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 200000, 0, 0, 0, 0, 0)

    t_end = time.time() + args.seconds
    last_print = 0.0
    n_frames = 0
    while time.time() < t_end:
        msg = m.recv_match(type=["RC_CHANNELS", "STATUSTEXT", "SYS_STATUS"],
                           blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"  STATUSTEXT: {msg.text}")
            continue
        if msg.get_type() == "SYS_STATUS":
            bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
            now = time.time()
            if now - last_print >= 1.0 and n_frames == 0:
                print(f"  rc-receiver sensor: present={bool(msg.onboard_control_sensors_present & bit)}"
                      f" enabled={bool(msg.onboard_control_sensors_enabled & bit)}"
                      f" healthy={bool(msg.onboard_control_sensors_health & bit)}")
                last_print = now
            continue
        n_frames += 1
        now = time.time()
        if now - last_print >= 1.0:
            chans = [getattr(msg, f"chan{i}_raw") for i in range(1, 9)]
            print(f"chancount={msg.chancount} ch1-8={chans} rssi={msg.rssi}")
            last_print = now

    if n_frames == 0:
        print("FAIL: no RC_CHANNELS received -- no RC link reaching PX4")
        return 1
    print(f"OK: {n_frames} RC_CHANNELS frames in {args.seconds:.0f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
