import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1])
t0 = u.start_timestamp
print("duration %.0f s" % ((u.last_timestamp - t0) / 1e6))
seen = {}
for m in u.logged_messages:
    seen.setdefault(m.message, []).append(round((m.timestamp - t0) / 1e6, 1))
for msg, ts in seen.items():
    print("  MSG x%d (first %.1f s, last %.1f s): %s" % (len(ts), ts[0], ts[-1], msg))
def ds(name, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i:
            return d.data
    return None
ff = ds("failsafe_flags")
if ff is not None:
    on = [k for k, v in ff.items() if k != "timestamp" and v.dtype != np.uint64 and v[-1] != 0]
    print("failsafe_flags set at the end:", on)
vs = ds("vehicle_status")
if vs is not None:
    for k in ("arming_state", "nav_state", "nav_state_user_intention", "pre_flight_checks_pass", "failsafe",
              "rc_calibration_in_progress", "calibration_enabled", "safety_button_available", "safety_off",
              "usb_connected", "power_input_valid", "open_drone_id_system_present", "open_drone_id_system_healthy",
              "latest_disarming_reason", "latest_arming_reason", "can_set_nav_states"):
        if k in vs: print("  vehicle_status.%s: %s" % (k, sorted(set(vs[k].tolist()))[:6]))
g = ds("sensor_gps")
if g is not None:
    print("gps: fix %s sats %s..%s eph %.1f hdop %.1f jam %s noise %s" % (sorted(set(g["fix_type"].tolist())), g["satellites_used"].min(), g["satellites_used"].max(), g["eph"][-1], g["hdop"][-1], g["jamming_indicator"][-1], g["noise_per_ms"][-1]))
else: print("gps: NO sensor_gps topic")
mc = ds("manual_control_setpoint")
if mc is not None:
    print("sticks: throttle %.2f..%.2f yaw %.2f..%.2f valid %s source %s" % (mc["throttle"].min(), mc["throttle"].max(), mc["yaw"].min(), mc["yaw"].max(), sorted(set(mc["valid"].tolist())), sorted(set(mc["data_source"].tolist()))))
sw = ds("manual_control_switches")
if sw is not None:
    print("switches:", {k: sorted(set(v.tolist())) for k, v in sw.items() if k.endswith("_switch") or k.endswith("slot")})
rc = ds("input_rc")
if rc is not None:
    print("rc: lost %s failsafe %s rssi %s ch1-8 last %s" % (sorted(set(rc["rc_lost"].tolist())), sorted(set(rc["rc_failsafe"].tolist())), rc["rssi"][-1], [int(rc["values[%d]" % i][-1]) for i in range(8)]))
b = ds("battery_status")
if b is not None:
    print("battery: %.2f V remaining %.2f warning %s" % (b["voltage_v"][-1], b["remaining"][-1], sorted(set(b["warning"].tolist()))))
ev = ds("event")
print("events logged:", 0 if ev is None else len(ev["timestamp"]))
for k in ["COM_ARM_WO_GPS", "COM_ARM_SDCARD", "CBRK_IO_SAFETY", "CBRK_USB_CHK", "CBRK_SUPPLY_CHK", "COM_RC_IN_MODE", "RC_MAP_ARM_SW", "RC_MAP_KILL_SW",
          "COM_ARM_SWISBTN", "MAN_ARM_GESTURE", "COM_RC_ARM_HYST", "COM_PREARM_MODE", "SYS_HAS_GPS", "SYS_HAS_MAG", "COM_ARM_MAG_STR", "COM_ARM_MAG_ANG",
          "CAL_MAG0_ID", "CAL_MAG0_PRIO", "CAL_MAG1_ID", "CAL_MAG1_PRIO", "CAL_MAG0_ROT", "EKF2_MAG_TYPE", "GF_ACTION", "COM_ARM_ODID", "NAV_DLL_ACT", "COM_FLTMODE1", "COM_FLTMODE4", "COM_FLTMODE6", "COM_ARM_WO_OBLOG", "SDLOG_MODE", "COM_HOME_EN", "COM_ARM_AUTH_REQ", "COM_ARM_MIS_REQ", "COM_POWER_COUNT", "COM_ARM_BAT_MIN", "COM_ARM_HFLT_CHK"]:
    if k in u.initial_parameters: print("  param %s = %s" % (k, u.initial_parameters[k]))
print("changed params during log:", [(round((t - t0) / 1e6), n, v) for t, n, v in u.changed_parameters][:20])
