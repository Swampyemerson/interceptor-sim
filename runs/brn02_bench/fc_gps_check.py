"""Bench check: which FC is on USB, does it see its GPS and compass?

Run with WINDOWS python (WSL2 cannot see COM ports). Read-only: it requests
nothing but message streams, writes no parameter, never arms.
Exit 0 = GPS module talking AND compass present+healthy; 1 = not; 2 = no FC.
"""
import sys
import time

import serial.tools.list_ports as lp
from pymavlink import mavutil

LISTEN_S = 25.0
MAG = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
GPS = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS

ports = [p for p in lp.comports()]
if not ports:
    print("NO COM PORTS -- no flight controller on USB")
    sys.exit(2)
for p in ports:
    print("port:", p.device, "|", p.description, "|", p.hwid)

m = None
for p in ports:
    try:
        c = mavutil.mavlink_connection(p.device, baud=115200)
        hb = c.wait_heartbeat(timeout=6)
        if hb is not None:
            m = c
            ap = {3: "ArduPilot", 12: "PX4"}.get(hb.autopilot, str(hb.autopilot))
            print(f"HEARTBEAT on {p.device}: autopilot={ap} sysid={c.target_system}")
            break
        c.close()
    except Exception as e:  # noqa: BLE001
        print(f"  {p.device}: {e}")
if m is None:
    print("ports exist but none spoke MAVLink within 6 s")
    sys.exit(2)

m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
t0 = time.time()
gps_msgs = 0
best_fix, best_sats = -1, -1
mag_present = mag_health = gps_present = gps_health = None
second_gps = 0
while time.time() - t0 < LISTEN_S:
    msg = m.recv_match(type=["GPS_RAW_INT", "GPS2_RAW", "SYS_STATUS", "STATUSTEXT"],
                       blocking=True, timeout=1)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "GPS_RAW_INT":
        gps_msgs += 1
        best_fix = max(best_fix, msg.fix_type)
        if msg.satellites_visible != 255:
            best_sats = max(best_sats, msg.satellites_visible)
    elif t == "GPS2_RAW":
        second_gps += 1
    elif t == "SYS_STATUS":
        pr, he = msg.onboard_control_sensors_present, msg.onboard_control_sensors_health
        mag_present, mag_health = bool(pr & MAG), bool(he & MAG)
        gps_present, gps_health = bool(pr & GPS), bool(he & GPS)
    elif t == "STATUSTEXT":
        print("  FC says:", msg.text)

FIX = {-1: "no GPS_RAW_INT at all", 0: "NO GPS MODULE DETECTED", 1: "module talking, no fix",
       2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
print("---- result over %.0f s" % LISTEN_S)
print(f"GPS_RAW_INT messages : {gps_msgs}")
print(f"best fix_type        : {best_fix} ({FIX.get(best_fix, '?')})")
print(f"best satellites      : {best_sats}")
print(f"SYS_STATUS GPS       : present={gps_present} healthy={gps_health}")
print(f"SYS_STATUS compass   : present={mag_present} healthy={mag_health}")
if mag_present is None:
    print("UNCERTAIN: no SYS_STATUS received -- compass NOT verified")
    sys.exit(1)
ok = best_fix >= 1 and mag_present and mag_health
print("VERDICT:", "PASS (GPS module talking + compass healthy)" if ok else "FAIL")
sys.exit(0 if ok else 1)
