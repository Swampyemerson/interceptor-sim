#!/usr/bin/env python3
"""Pre-battery bench status snapshot for the 6C Mini (afr-02 prep). READ-ONLY.

RUNS ON *WINDOWS* PYTHON (the FC enumerates as a COM port WSL2 cannot see):
    python.exe configs/px4_6cmini/bench_status.py --port COM5

Prints: firmware version + git hash, airframe (SYS_AUTOSTART), battery-monitor
params (6S readiness), RC calibration state, SD card storage, and SYS_STATUS
sensor-health bits. No writes, no arming.
"""
import argparse
import os
import struct
import sys
import time

os.environ.setdefault("MAVLINK20", "1")  # STORAGE_INFORMATION is MAVLink2-only
from pymavlink import mavutil

MSG_ID_STORAGE_INFORMATION = 261  # absent from the v10 dialect module by name

MAV_PARAM_TYPE_INT32 = 6

PARAMS = [
    # battery monitor (PM02 on POWER1) — 6S readiness
    "BAT1_N_CELLS", "BAT1_SOURCE", "BAT1_V_DIV", "BAT1_A_PER_V",
    "BAT1_V_EMPTY", "BAT1_V_CHARGED", "BAT_LOW_THR", "BAT_CRIT_THR",
    "CBRK_SUPPLY_CHK",
    # airframe + allocation
    "SYS_AUTOSTART", "CA_AIRFRAME", "CA_ROTOR_COUNT",
    # RC calibration evidence (QGC cal writes these away from defaults)
    "RC1_MIN", "RC1_MAX", "RC1_TRIM", "RC2_MIN", "RC2_MAX",
    "RC3_MIN", "RC3_MAX", "RC4_MIN", "RC4_MAX", "RC_CHAN_CNT",
    "RC_MAP_THROTTLE", "RC_MAP_ROLL", "RC_MAP_PITCH", "RC_MAP_YAW",
    # calibration IDs (0 = never calibrated)
    "CAL_GYRO0_ID", "CAL_ACC0_ID", "CAL_MAG0_ID",
    # safety
    "RC_MAP_KILL_SW", "COM_KILL_DISARM", "COM_ARM_WO_GPS", "COM_RC_LOSS_T",
    "NAV_RCL_ACT", "COM_LOW_BAT_ACT", "GF_ACTION", "COM_OF_LOSS_T",
]


def read_param(m, name, timeout=3.0):
    for _ in range(3):
        m.mav.param_request_read_send(
            m.target_system, m.target_component, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            got = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            if got.rstrip("\x00") == name:
                if msg.param_type == MAV_PARAM_TYPE_INT32:
                    return struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
                return msg.param_value
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.port, baud=115200)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        # NO VACUOUS VERDICTS: a dead port must not print a clean-looking
        # snapshot and exit 0 (2026-10-04 review finding).
        print(f"FAIL: no heartbeat on {args.port} in 10 s")
        return 1
    if m.target_component == 0:
        m.target_component = 1
    print(f"heartbeat: sys {m.target_system} comp {m.target_component}")

    # firmware version
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    ver = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=5)
    if ver:
        v = ver.flight_sw_version
        print(f"PX4 firmware: {(v >> 24) & 0xFF}.{(v >> 16) & 0xFF}.{(v >> 8) & 0xFF} "
              f"(type {v & 0xFF})  git {bytes(ver.flight_custom_version)[::-1].hex()[:8]}")
    else:
        print("PX4 firmware: AUTOPILOT_VERSION not received")

    # SD card
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        MSG_ID_STORAGE_INFORMATION, 0, 0, 0, 0, 0, 0)
    st = m.recv_match(type="STORAGE_INFORMATION", blocking=True, timeout=5)
    if st:
        print(f"SD card: status={st.status} total={st.total_capacity:.0f}MB "
              f"free={st.available_capacity:.0f}MB")
    else:
        print("SD card: STORAGE_INFORMATION not received (not conclusive)")

    print()
    for name in PARAMS:
        val = read_param(m, name)
        print(f"  {name:<18} {val if val is not None else 'NOT FOUND'}")

    # live SYS_STATUS: health bits + measured voltage (USB power => ~0)
    msg = m.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
    if msg:
        print(f"\nSYS_STATUS: voltage={msg.voltage_battery} mV "
              f"current={msg.current_battery} load={msg.load/10:.0f}%")
        bits = [
            ("gyro", 0x1), ("accel", 0x2), ("mag", 0x4), ("baro", 0x8),
            ("GPS", 0x20), ("RC", 0x10000),
        ]
        for nm, b in bits:
            pres = bool(msg.onboard_control_sensors_present & b)
            en = bool(msg.onboard_control_sensors_enabled & b)
            ok = bool(msg.onboard_control_sensors_health & b)
            print(f"  {nm:<6} present={pres} enabled={en} healthy={ok}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
