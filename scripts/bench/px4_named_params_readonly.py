#!/usr/bin/env python3
"""READ-ONLY: heartbeat + a named list of PX4 params through the Pi's UDP bridge.
Sends only HEARTBEAT (GCS) and PARAM_REQUEST_READ. No PARAM_SET, no commands."""
import struct, sys, time
from pymavlink import mavutil
NAMES = """MPC_XY_VEL_MAX MPC_XY_CRUISE MPC_VEL_MANUAL MPC_ACC_HOR_MAX MPC_ACC_HOR MPC_JERK_MAX MPC_JERK_AUTO
MPC_TILTMAX_AIR MPC_MAN_TILT_MAX MPC_Z_VEL_MAX_UP MPC_Z_VEL_MAX_DN MPC_THR_HOVER MPC_THR_MAX MPC_THR_MIN MPC_USE_HTE
MPC_XY_VEL_P_ACC MPC_XY_VEL_I_ACC MPC_XY_VEL_D_ACC MPC_XY_P MPC_Z_VEL_P_ACC MPC_Z_VEL_I_ACC MPC_Z_P
COM_OF_LOSS_T COM_OBL_RC_ACT COM_RC_IN_MODE COM_RCL_EXCEPT COM_RC_LOSS_T NAV_RCL_ACT NAV_DLL_ACT COM_DL_LOSS_T
COM_FLTMODE1 COM_FLTMODE2 COM_FLTMODE3 COM_FLTMODE4 COM_FLTMODE5 COM_FLTMODE6 RC_MAP_FLTMODE RC_MAP_OFFB_SW
RC_MAP_KILL_SW RC_MAP_ARM_SW RC_MAP_AUX1 RC_MAP_AUX2 RC_MAP_AUX3 COM_ARM_SWISBTN COM_DISARM_PRFLT
MAV_0_CONFIG MAV_0_MODE MAV_0_RATE MAV_1_CONFIG MAV_1_MODE MAV_1_RATE MAV_1_FORWARD MAV_2_CONFIG SER_TEL1_BAUD SER_TEL2_BAUD
CBRK_IO_SAFETY COM_PREARM_MODE COM_ARM_WO_GPS SDLOG_MODE SDLOG_PROFILE EKF2_HGT_REF SYS_AUTOSTART SENS_BOARD_ROT
GF_ACTION GF_MAX_HOR_DIST GF_MAX_VER_DIST COM_LOW_BAT_ACT BAT1_N_CELLS BAT1_A_PER_V BAT1_V_DIV BAT_CRIT_THR BAT_LOW_THR
MC_ROLLRATE_P MC_PITCHRATE_P IMU_GYRO_CUTOFF IMU_DGYRO_CUTOFF""".split()
INT = {1, 2, 3, 4, 5, 6}
m = mavutil.mavlink_connection(sys.argv[1], source_system=254, source_component=190)
m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
hb = None; t0 = time.time()
while time.time() - t0 < 10:
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    if msg and msg.get_srcComponent() == 1:
        hb = msg; break
if hb is None:
    print("NO HEARTBEAT from autopilot -- UNCERTAIN"); sys.exit(2)
armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
print(f"heartbeat: sys {m.target_system} autopilot {hb.autopilot} type {hb.type} ARMED={armed} custom_mode={hb.custom_mode:#x}")
got = {}
for name in NAMES:
    for attempt in range(3):
        m.mav.param_request_read_send(m.target_system, 1, name.encode(), -1)
        t1 = time.time()
        while time.time() - t1 < 1.2:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
            if msg and msg.param_id.strip("\x00") == name:
                v = struct.unpack("<i", struct.pack("<f", msg.param_value))[0] if msg.param_type in INT else round(float(msg.param_value), 5)
                got[name] = v; break
        if name in got: break
missing = [n for n in NAMES if n not in got]
for n in NAMES:
    if n in got: print(f"{n:20s} {got[n]}")
print("MISSING (no reply; may not exist in this firmware):", " ".join(missing) or "none")
for typ in ("SYS_STATUS", "GPS_RAW_INT", "ATTITUDE", "RC_CHANNELS", "EXTENDED_SYS_STATE"):
    msg = m.recv_match(type=typ, blocking=True, timeout=3)
    if msg is None: print(typ, "not seen in 3 s"); continue
    d = msg.to_dict()
    keep = {k: d[k] for k in d if k in ("voltage_battery","current_battery","battery_remaining","fix_type","satellites_visible","eph","roll","pitch","yaw","chancount","chan5_raw","chan6_raw","chan7_raw","chan8_raw","rssi","landed_state","onboard_control_sensors_health")}
    print(typ, keep)
