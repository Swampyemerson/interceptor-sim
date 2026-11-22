"""Stability, power and estimator read of one PX4 flight log (airborne window only)."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = u.start_timestamp
T = lambda ts: (ts - t0) / 1e6
def ds(name, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i: return d.data
def eul(d, pre="q"):
    w, x, y, z = (d["%s[%d]" % (pre, i)] for i in range(4))
    return (np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))), np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1))),
            np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))))
ld = ds("vehicle_land_detected"); tl = T(ld["timestamp"])
air = tl[ld["landed"] == 0]; ta, td = air[0], air[-1]
vs = ds("vehicle_status"); tv = T(vs["timestamp"])
print("AIRBORNE %.1f to %.1f s (%.1f s)" % (ta, td, td - ta))
P = u.initial_parameters
for grp in (("MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLLRATE_K", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCHRATE_K", "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P"),
            ("IMU_GYRO_CUTOFF", "IMU_DGYRO_CUTOFF", "IMU_GYRO_NF0_FRQ", "IMU_GYRO_NF0_BW", "IMU_GYRO_DNF_EN", "IMU_GYRO_RATEMAX", "MPC_THR_HOVER", "MPC_USE_HTE", "THR_MDL_FAC", "MPC_MANTHR_MIN", "MPC_THR_MIN", "MPC_THR_MAX"),
            ("CA_ROTOR0_PX", "CA_ROTOR0_PY", "CA_ROTOR0_KM", "CA_ROTOR1_PX", "CA_ROTOR1_PY", "CA_ROTOR1_KM", "CA_ROTOR2_PX", "CA_ROTOR2_PY", "CA_ROTOR2_KM", "CA_ROTOR3_PX", "CA_ROTOR3_PY", "CA_ROTOR3_KM"),
            ("PWM_MAIN_MIN1", "PWM_MAIN_MAX1", "PWM_MAIN_DIS1", "PWM_MAIN_TIM0", "PWM_MAIN_FUNC1", "PWM_MAIN_FUNC2", "PWM_MAIN_FUNC3", "PWM_MAIN_FUNC4", "BAT1_R_INTERNAL", "BAT1_V_EMPTY", "BAT1_V_CHARGED", "BAT1_V_LOAD_DROP", "BAT1_CAPACITY", "BAT_LOW_THR", "BAT_CRIT_THR", "BAT_EMERGEN_THR"),
            ("GF_ACTION", "GF_MAX_HOR_DIST", "GF_MAX_VER_DIST", "COM_POS_FS_EPH", "EKF2_REQ_NSATS", "EKF2_REQ_PDOP", "EKF2_GPS_CTRL", "EKF2_MAG_TYPE", "EKF2_HGT_REF", "SENS_BOARD_ROT", "CAL_MAG0_ROT", "CAL_MAG1_ROT", "EKF2_MAG_CHECK", "COM_ARM_MAG_STR")):
    print("  " + "  ".join("%s=%s" % (k, round(P[k], 4) if isinstance(P.get(k), float) else P.get(k)) for k in grp))
# attitude tracking
a = ds("vehicle_attitude"); tt = T(a["timestamp"]); r, p, y = eul(a)
sp = ds("vehicle_attitude_setpoint"); ts = T(sp["timestamp"]); rs, ps, ys = eul(sp, "q_d")
w = (tt > ta + 1) & (tt < td - 1)
ri = np.interp(tt[w], ts, rs); pi = np.interp(tt[w], ts, ps)
print("attitude: roll %.1f..%.1f deg (sp %.1f..%.1f), pitch %.1f..%.1f (sp %.1f..%.1f); tracking error RMS roll %.1f pitch %.1f deg, max %.1f / %.1f" % (
    r[w].min(), r[w].max(), ri.min(), ri.max(), p[w].min(), p[w].max(), pi.min(), pi.max(),
    np.sqrt(np.mean((r[w]-ri)**2)), np.sqrt(np.mean((p[w]-pi)**2)), np.abs(r[w]-ri).max(), np.abs(p[w]-pi).max()))
# rates
g = ds("vehicle_angular_velocity"); tg = T(g["timestamp"]); rsps = ds("vehicle_rates_setpoint"); tr = T(rsps["timestamp"])
w = (tg > ta + 1) & (tg < td - 1); fs = 1 / np.median(np.diff(tg[w]))
for i, (ax, k) in enumerate((("roll", "roll"), ("pitch", "pitch"), ("yaw", "yaw"))):
    x = np.degrees(g["xyz[%d]" % i][w]); s = np.degrees(np.interp(tg[w], tr, rsps[k]))
    e = x - s; f = np.fft.rfftfreq(len(e), 1 / fs); A = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x))))
    band = f > 1.0; pk = f[band][np.argmax(A[band])]
    print("rate %-5s: actual RMS %.0f deg/s (max %.0f), setpoint RMS %.0f, error RMS %.0f; strongest above 1 Hz at %.1f Hz (log rate %.0f Hz)" % (
        ax, np.sqrt(np.mean(x**2)), np.abs(x).max(), np.sqrt(np.mean(s**2)), np.sqrt(np.mean(e**2)), pk, fs))
# motors
m = ds("actuator_motors"); tm = T(m["timestamp"]); w = (tm > ta + 1) & (tm < td - 1)
c = np.array([m["control[%d]" % i][w] for i in range(4)])
print("motors (0..1): mean %s  min %s  max %s  saturated-high %.1f%%  at-floor %.1f%%" % (np.round(c.mean(1), 2), np.round(c.min(1), 2), np.round(c.max(1), 2),
      100 * (c > 0.98).any(0).mean(), 100 * (c < 0.02).any(0).mean()))
h = ds("hover_thrust_estimate")
if h is not None:
    th = T(h["timestamp"]); w = (th > ta + 2) & (th < td - 1)
    if w.any(): print("hover thrust estimate: %.3f..%.3f (last %.3f) valid %s" % (h["hover_thrust"][w].min(), h["hover_thrust"][w].max(), h["hover_thrust"][w][-1], sorted(set(h["valid"][w].tolist()))))
tsp = ds("vehicle_thrust_setpoint"); tts = T(tsp["timestamp"]); w = (tts > ta + 2) & (tts < td - 2)
print("thrust setpoint z (0..1): mean %.3f, 10-90%% %.3f..%.3f" % (-tsp["xyz[2]"][w].mean(), np.percentile(-tsp["xyz[2]"][w], 10), np.percentile(-tsp["xyz[2]"][w], 90)))
# battery
b = ds("battery_status"); tb = T(b["timestamp"]); w = (tb > ta + 2) & (tb < td - 1)
V, I = b["voltage_v"][w], b["current_a"][w]; k = np.polyfit(I, V, 1)
print("battery airborne: I mean %.1f A (10-90%% %.1f..%.1f), V mean %.2f min %.2f; fit V = %.2f - %.3f*I  (apparent resistance %.0f milliohm)" % (I.mean(), np.percentile(I, 10), np.percentile(I, 90), V.mean(), V.min(), k[1], -k[0], -k[0] * 1000))
pre = (tb > ta - 30) & (tb < ta - 6); post = tb > td + 4
print("  resting V before %.2f, after %.2f; used %.0f mAh airborne; 'remaining' before %.2f, min in flight %.2f, after %.2f" % (b["voltage_v"][pre].mean(), b["voltage_v"][post].mean() if post.any() else float("nan"),
      b["discharged_mah"][w][-1] - b["discharged_mah"][w][0], b["remaining"][pre].mean(), b["remaining"][w].min(), b["remaining"][post].mean() if post.any() else float("nan")))
# vibration
s = ds("sensor_combined"); tsc = T(s["timestamp"]); w = (tsc > ta + 2) & (tsc < td - 2)
print("accel std airborne (m/s2): x %.1f y %.1f z %.1f  (log rate %.0f Hz)" % tuple([s["accelerometer_m_s2[%d]" % i][w].std() for i in range(3)] + [1 / np.median(np.diff(tsc[w]))]))
for i in range(2):
    vi = ds("vehicle_imu_status", i)
    if vi is not None:
        tvi = T(vi["timestamp"]); w = (tvi > ta + 2) & (tvi < td - 2)
        print("  imu%d vibration metric: accel %.1f (max %.1f) gyro %.2f (max %.2f) clipping %s" % (i, vi["accel_vibration_metric"][w].mean(), vi["accel_vibration_metric"][w].max(), vi["gyro_vibration_metric"][w].mean(), vi["gyro_vibration_metric"][w].max(),
              [int(vi["accel_clipping[%d]" % j][w][-1] - vi["accel_clipping[%d]" % j][w][0]) for j in range(3)]))
# sticks
mc = ds("manual_control_setpoint"); tmc = T(mc["timestamp"]); w = (tmc > ta) & (tmc < td)
print("sticks airborne: roll %.2f..%.2f pitch %.2f..%.2f yaw %.2f..%.2f throttle %.2f..%.2f (mean %.2f)" % (mc["roll"][w].min(), mc["roll"][w].max(), mc["pitch"][w].min(), mc["pitch"][w].max(), mc["yaw"][w].min(), mc["yaw"][w].max(), mc["throttle"][w].min(), mc["throttle"][w].max(), mc["throttle"][w].mean()))
# position / gps timeline
lp = ds("vehicle_local_position"); tlp = T(lp["timestamp"]); gps = ds("sensor_gps"); tgp = T(gps["timestamp"])
print("timeline (every ~6 s): t | mode | height m | est. dist from start m | est speed m/s | xy_valid | sats hdop s_var jam")
for tq in np.arange(ta - 6, td + 4, 6.0):
    i = np.searchsorted(tlp, tq); j = min(np.searchsorted(tgp, tq), len(tgp) - 1); kq = min(np.searchsorted(tv, tq), len(tv) - 1)
    if i >= len(tlp): break
    print("  %6.1f | %2d | %5.2f | %7.1f | %5.1f | %d | %2d %4.1f %5.2f %3d" % (tq, vs["nav_state"][kq], -(lp["z"][i] - lp["z"][np.searchsorted(tlp, ta)]), np.hypot(lp["x"][i] - lp["x"][np.searchsorted(tlp, ta)], lp["y"][i] - lp["y"][np.searchsorted(tlp, ta)]),
          np.hypot(lp["vx"][i], lp["vy"][i]), lp["xy_valid"][i], gps["satellites_used"][j], gps["hdop"][j], gps["s_variance_m_s"][j], gps["jamming_indicator"][j]))
