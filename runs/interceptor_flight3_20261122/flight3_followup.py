"""Follow-up questions on the three flights of 2026-11-22: height sources, the yaw runaway, compass calibration, stick delay, vibration."""
import sys, numpy as np
from pyulog import ULog
def load(p):
    u = ULog(p); t0 = u.start_timestamp
    def ds(name, i=0):
        for d in u.data_list:
            if d.name == name and d.multi_id == i: return d.data
    return u, (lambda ts: (ts - t0) / 1e6), ds
def eul(d, pre="q"):
    w, x, y, z = (d["%s[%d]" % (pre, i)] for i in range(4))
    return (np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)), np.arcsin(np.clip(2*(w*y-z*x), -1, 1)), np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
flights = [("flight 1", sys.argv[1]), ("flight 2", sys.argv[2]), ("flight 3", sys.argv[3])]
for name, path in flights:
    u, T, ds = load(path)
    ld = ds("vehicle_land_detected"); tl = T(ld["timestamp"]); air = tl[ld["landed"] == 0]; ta, td = air[0], air[-1]
    if name == "flight 2": td = 139.5
    if name == "flight 1": td = 251.5
    print("==== %s (airborne %.0f-%.0f s)" % (name, ta, td))
    P = u.initial_parameters
    print("  compass 0 calibration: offsets %.3f %.3f %.3f  scales %.3f %.3f %.3f  rot %s | EKF2_HGT_REF %s EKF2_BARO_CTRL %s EKF2_GPS_CTRL %s EKF2_MAG_CHECK %s SENS_BOARD_X_OFF %.2f" % (
        P.get("CAL_MAG0_XOFF"), P.get("CAL_MAG0_YOFF"), P.get("CAL_MAG0_ZOFF"), P.get("CAL_MAG0_XSCALE"), P.get("CAL_MAG0_YSCALE"), P.get("CAL_MAG0_ZSCALE"), P.get("CAL_MAG0_ROT"), P.get("EKF2_HGT_REF"), P.get("EKF2_BARO_CTRL"), P.get("EKF2_GPS_CTRL"), P.get("EKF2_MAG_CHECK"), P.get("SENS_BOARD_X_OFF")))
    # --- height sources
    lp = ds("vehicle_local_position"); tlp = T(lp["timestamp"]); ad = ds("vehicle_air_data"); tad = T(ad["timestamp"]); g = ds("sensor_gps"); tg = T(g["timestamp"])
    tq = np.arange(ta, td, 5.0)
    ez = -(np.interp(tq, tlp, lp["z"]) - np.interp(ta, tlp, lp["z"])); bz = np.interp(tq, tad, ad["baro_alt_meter"]) - np.interp(ta, tad, ad["baro_alt_meter"])
    gz = np.interp(tq, tg, g["altitude_msl_m"]) - np.interp(ta, tg, g["altitude_msl_m"])
    print("  height above takeoff every 5 s (m)")
    print("    estimator: " + " ".join("%5.1f" % v for v in ez))
    print("    barometer: " + " ".join("%5.1f" % v for v in bz))
    print("    GPS      : " + " ".join("%5.1f" % v for v in gz))
    print("    estimator minus barometer: start %.1f, end %.1f, largest %.1f m; GPS minus barometer largest %.1f m; GPS vertical accuracy (epv) median %.1f m" % (ez[0] - bz[0], ez[-1] - bz[-1], np.abs(ez - bz).max(), np.abs(gz - bz).max(), np.median(g["epv"][(tg > ta) & (tg < td)])))
    # --- stick to attitude delay by mode
    mc = ds("manual_control_setpoint"); tmc = T(mc["timestamp"]); a = ds("vehicle_attitude"); tt = T(a["timestamp"]); r, p, y = eul(a)
    vs = ds("vehicle_status"); tv = T(vs["timestamp"])
    dt = 0.02; tq2 = np.arange(ta + 1, td - 1, dt); nav = vs["nav_state"][np.clip(np.searchsorted(tv, tq2) - 1, 0, len(tv) - 1)]
    for mode, label in ((2, "Position"), (1, "Altitude"), (15, "Stabilized")):
        sel = nav == mode
        if sel.sum() * dt < 8: continue
        for nm, st, at in (("roll", mc["roll"], np.degrees(r)), ("pitch", mc["pitch"], -np.degrees(p))):
            s = np.interp(tq2, tmc, st); x = np.interp(tq2, tt, at)
            best = (0, 0)
            for L in range(0, 75):
                idx = np.where(sel)[0]; idx = idx[idx + L < len(tq2)]
                if s[idx].std() < 0.03: break
                c = np.corrcoef(s[idx], x[idx + L])[0, 1]
                if abs(c) > abs(best[0]): best = (c, L * dt)
            if best[0]: print("  stick-to-tilt delay in %s mode, %s: %.2f s (correlation %.2f, %d s of data, stick std %.2f)" % (label, nm, best[1], best[0], sel.sum() * dt, np.interp(tq2, tmc, st)[sel].std()))
    # --- vibration spectra (accel 222 Hz, gyro 48 Hz)
    s = ds("sensor_combined"); ts = T(s["timestamp"]); w = (ts > ta + 2) & (ts < td - 2); fs = 1 / np.median(np.diff(ts[w]))
    out = []
    for i, ax in enumerate("xyz"):
        x = s["accelerometer_m_s2[%d]" % i][w]; f = np.fft.rfftfreq(len(x), 1 / fs); S = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x))))**2
        band = f > 5; pk = f[band][np.argmax(S[band])]; tot = S[f > 1].sum()
        out.append("%s: std %.2f, peak %.0f Hz, share 5-20 Hz %.0f%%, 20-60 %.0f%%, 60-111 %.0f%%" % (ax, x.std(), pk, *[100 * S[(f >= lo) & (f < hi)].sum() / tot for lo, hi in ((5, 20), (20, 60), (60, 112))]))
    print("  accelerometer (%.0f Hz log): " % fs + " | ".join(out))
    gy = ds("vehicle_angular_velocity"); tgy = T(gy["timestamp"]); w = (tgy > ta + 2) & (tgy < td - 2); fs2 = 1 / np.median(np.diff(tgy[w])); out = []
    for i, ax in enumerate(("roll", "pitch", "yaw")):
        x = np.degrees(gy["xyz[%d]" % i][w]); f = np.fft.rfftfreq(len(x), 1 / fs2); S = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x))))**2
        band = f > 6; pk = f[band][np.argmax(S[band])]; out.append("%s: above 6 Hz RMS %.1f deg/s, peak %.1f Hz" % (ax, np.sqrt(2 * S[band].sum()) / len(x) / np.sqrt(np.mean(np.hanning(len(x))**2)), pk))
    print("  gyro (%.0f Hz log): " % fs2 + " | ".join(out))
    for i in range(2):
        vi = ds("vehicle_imu_status", i)
        if vi is not None:
            tvi = T(vi["timestamp"]); w = (tvi > ta + 2) & (tvi < td - 2)
            print("  imu%d: accel vibration metric mean %.1f max %.1f; gyro %.2f; accel clipping counts in flight %s" % (i, vi["accel_vibration_metric"][w].mean(), vi["accel_vibration_metric"][w].max(), vi["gyro_vibration_metric"][w].mean(), [int(vi["accel_clipping[%d]" % j][w][-1] - vi["accel_clipping[%d]" % j][w][0]) for j in range(3)]))
    if name == "flight 3":
        print("  --- last 12 s: t | yaw est | yaw setpoint | gyro z deg/s | yaw stick | mode | mag fault")
        sp = ds("vehicle_attitude_setpoint"); tsp = T(sp["timestamp"]); _, _, ys = eul(sp, "q_d"); gy = ds("vehicle_angular_velocity"); tgy = T(gy["timestamp"]); ef = ds("estimator_status_flags"); te = T(ef["timestamp"])
        for tq3 in np.arange(406, 420.5, 1.0):
            k = np.searchsorted(tt, tq3); k2 = min(np.searchsorted(tsp, tq3), len(tsp) - 1); k3 = np.searchsorted(tgy, tq3); m = np.searchsorted(tmc, tq3); n = np.searchsorted(tv, tq3) - 1; e = np.searchsorted(te, tq3) - 1
            print("    %5.0f | %5.0f | %5.0f | %6.1f | %5.2f | %2d | %d" % (tq3, np.degrees(y[k]), np.degrees(ys[k2]), np.degrees(gy["xyz[2]"][max(0, k3 - 24):k3 + 1].mean()), mc["yaw"][m], vs["nav_state"][n], ef["cs_mag_fault"][e]))
        mg = ds("vehicle_magnetometer"); 
        if mg is not None:
            tm_ = T(mg["timestamp"]); w = (tm_ > ta) & (tm_ < td)
            print("  compass in use: device ids %s; calibration count %s" % (sorted(set(mg["device_id"][w].tolist())), sorted(set(mg["calibration_count"][w].tolist()))))
