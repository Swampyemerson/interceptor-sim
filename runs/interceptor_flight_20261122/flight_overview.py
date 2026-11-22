"""Overview of one PX4 flight log: timeline, messages, modes, arming, failsafes, battery, GPS."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = u.start_timestamp
T = lambda ts: (ts - t0) / 1e6
def ds(name, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i: return d.data
print("duration %.0f s, dropouts %d (%d ms)" % (T(u.last_timestamp), len(u.dropouts), sum(d.duration for d in u.dropouts)))
print("--- messages"); last = None
for m in u.logged_messages:
    if m.message != last: print("  %6.1f  %s" % (T(m.timestamp), m.message))
    last = m.message
vs = ds("vehicle_status"); t = T(vs["timestamp"])
def changes(name, arr, tt):
    idx = [0] + [i for i in range(1, len(arr)) if arr[i] != arr[i-1]]
    print("  %s: " % name + ", ".join("%.1fs=%s" % (tt[i], arr[i]) for i in idx[:25]))
print("--- state changes")
for k in ("arming_state", "nav_state", "failsafe", "latest_arming_reason", "latest_disarming_reason", "pre_flight_checks_pass"):
    if k in vs: changes(k, vs[k], t)
ld = ds("vehicle_land_detected")
if ld is not None:
    for k in ("landed", "maybe_landed", "ground_contact", "freefall"): changes(k, ld[k], T(ld["timestamp"]))
arm = t[vs["arming_state"] == 2]
if len(arm) == 0: print("never armed"); sys.exit()
ta, td = arm[0], arm[-1]; print("ARMED %.1f s to %.1f s (%.1f s)" % (ta, td, td - ta))
sw = ds("manual_control_switches")
if sw is not None:
    for k in ("mode_slot", "kill_switch"): changes(k, sw[k], T(sw["timestamp"]))
ff = ds("failsafe_flags")
if ff is not None:
    tt = T(ff["timestamp"])
    for k, v in ff.items():
        if k.startswith("mode_req") or k == "timestamp" or v.dtype == np.uint64: continue
        w = (tt > ta - 1) & (tt < td + 1)
        if len(set(v[w].tolist())) > 1 or (v[w].any() and k not in ()): changes("failsafe_flags." + k, v[w], tt[w])
b = ds("battery_status"); tb = T(b["timestamp"]); w = (tb > ta) & (tb < td)
print("--- battery armed: V %.2f..%.2f  I mean %.1f max %.1f A  discharged %.0f mAh  remaining %.2f->%.2f  warning %s" % (
    b["voltage_v"][w].min(), b["voltage_v"][w].max(), b["current_a"][w].mean(), b["current_a"][w].max(),
    b["discharged_mah"][w][-1] - b["discharged_mah"][w][0], b["remaining"][w][0], b["remaining"][w][-1], sorted(set(b["warning"][w].tolist()))))
g = ds("sensor_gps"); tg = T(g["timestamp"]); w = (tg > ta - 5) & (tg < td + 2)
print("--- gps armed: sats %d..%d fix %s hdop %.1f..%.1f s_var %.2f..%.2f jam %d..%d" % (g["satellites_used"][w].min(), g["satellites_used"][w].max(),
      sorted(set(g["fix_type"][w].tolist())), g["hdop"][w].min(), g["hdop"][w].max(), g["s_variance_m_s"][w].min(), g["s_variance_m_s"][w].max(), g["jamming_indicator"][w].min(), g["jamming_indicator"][w].max()))
lp = ds("vehicle_local_position"); tl = T(lp["timestamp"]); w = (tl > ta) & (tl < td)
print("--- local pos armed: height above start %.2f max m, |vxy| max %.1f m/s, vz %.1f..%.1f, xy_valid %s z_valid %s" % (
    (-(lp["z"][w] - lp["z"][w][0])).max(), np.hypot(lp["vx"][w], lp["vy"][w]).max(), lp["vz"][w].min(), lp["vz"][w].max(),
    sorted(set(lp["xy_valid"][w].tolist())), sorted(set(lp["z_valid"][w].tolist()))))
print("--- params:", {k: u.initial_parameters.get(k) for k in ("COM_DISARM_LAND", "COM_DISARM_PRFLT", "COM_KILL_DISARM", "MPC_THR_HOVER", "MPC_THR_MIN", "MPC_MANTHR_MIN", "THR_MDL_FAC", "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLLRATE_K", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCHRATE_K", "MC_YAWRATE_P", "MC_ROLL_P", "MC_PITCH_P", "IMU_GYRO_CUTOFF", "IMU_DGYRO_CUTOFF", "IMU_GYRO_NF0_FRQ", "IMU_GYRO_DNF_EN", "CA_ROTOR_COUNT", "CA_ROTOR0_PX", "CA_ROTOR0_PY", "CA_ROTOR1_PX", "CA_ROTOR1_PY", "CA_ROTOR2_PX", "CA_ROTOR2_PY", "CA_ROTOR3_PX", "CA_ROTOR3_PY", "CA_ROTOR0_KM", "CA_ROTOR1_KM", "CA_ROTOR2_KM", "CA_ROTOR3_KM", "COM_LOW_BAT_ACT", "FD_FAIL_R", "FD_FAIL_P", "CBRK_FLIGHTTERM", "FD_ACT_EN")})
print("changed params:", [(round(T(t_)), n, v) for t_, n, v in u.changed_parameters][:20])
