"""Heading error from tilt-against-GPS acceleration, compass health, estimator flags, hover trim. args: log t_start t_end"""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = u.start_timestamp; A, B = float(sys.argv[2]), float(sys.argv[3])
T = lambda ts: (ts - t0) / 1e6; G = 9.81
def ds(name, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i: return d.data
def smooth(x, n): return np.convolve(x, np.ones(n) / n, mode="same")
a = ds("vehicle_attitude"); tt = T(a["timestamp"]); w_, x_, y_, z_ = (a["q[%d]" % i] for i in range(4))
r = np.arctan2(2*(w_*x_+y_*z_), 1-2*(x_*x_+y_*y_)); p = np.arcsin(np.clip(2*(w_*y_-z_*x_), -1, 1)); y = np.arctan2(2*(w_*z_+x_*y_), 1-2*(y_*y_+z_*z_))
gp = ds("vehicle_gps_position"); tg = T(gp["timestamp"])
dt = 0.1; tq = np.arange(A, B, dt)
vN = smooth(np.interp(tq, tg, gp["vel_n_m_s"]), 7); vE = smooth(np.interp(tq, tg, gp["vel_e_m_s"]), 7)
aN, aE = np.gradient(vN, dt), np.gradient(vE, dt)
R = smooth(np.interp(tq, tt, r), 7); P = smooth(np.interp(tq, tt, p), 7); Y = np.interp(tq, tt, np.unwrap(y))
ab = (-G * np.tan(P)) + 1j * (G * np.tan(R))            # forward + i*right, heading frame
aw = ab * np.exp(1j * Y); ag = aN + 1j * aE
big = np.abs(ab) > 1.5                                   # deliberate tilts only
print("window %.0f-%.0f s; yaw %.0f..%.0f deg; mean roll %.2f deg, mean pitch %.2f deg; mean GPS velocity N %.2f E %.2f m/s; GPS net displacement %.1f m" % (
    A, B, np.degrees(Y.min()), np.degrees(Y.max()), np.degrees(R.mean()), np.degrees(P.mean()), vN.mean(), vE.mean(), np.hypot(vN.sum() * dt, vE.sum() * dt)))
if big.sum() > 20:
    best = None
    for lag in range(-8, 9):                             # GPS velocity lags attitude; search +-0.8 s
        s = slice(max(0, lag), len(tq) + min(0, lag)); s2 = slice(max(0, -lag), len(tq) + min(0, -lag))
        m_ = big[s2]; z = np.sum(ag[s][m_] * np.conj(aw[s2][m_])); c = np.abs(z) / np.sqrt(np.sum(np.abs(ag[s][m_])**2) * np.sum(np.abs(aw[s2][m_])**2))
        if best is None or c > best[0]: best = (c, lag, np.degrees(np.angle(z)), np.abs(z) / np.sum(np.abs(aw[s2][m_])**2))
    print("HEADING: tilt-implied acceleration against GPS acceleration (%d samples above 1.5 m/s2): best match at GPS lag %.1f s, correlation %.2f, gain %.2f, rotation %.0f deg" % (big.sum(), best[1] * dt, best[0], best[3], best[2]))
    print("  (rotation = how far the estimated heading must turn for the two to agree; near 0 means heading is right)")
else: print("HEADING: too few deliberate tilts to test")
for i in range(2):
    mg = ds("sensor_mag", i)
    if mg is None: continue
    tmg = T(mg["timestamp"]); nrm = np.sqrt(mg["x"]**2 + mg["y"]**2 + mg["z"]**2); w = (tmg > A) & (tmg < B)
    b = ds("battery_status"); cur = np.interp(tmg[w], T(b["timestamp"]), b["current_a"])
    cc = np.corrcoef(cur, nrm[w])[0, 1] if w.sum() > 5 else float("nan"); k = np.polyfit(cur, nrm[w], 1) if w.sum() > 5 else (0, 0)
    gnd = tmg < min(A, 8)
    print("  mag%d (id %s): field %.3f G on the ground, %.3f..%.3f in the window; against battery current: correlation %.2f, %.4f G per A (%.2f G at 77 A)" % (i, int(mg["device_id"][0]), nrm[gnd].mean() if gnd.any() else float("nan"), nrm[w].min(), nrm[w].max(), cc, k[0], k[0] * 77))
ef = ds("estimator_status_flags")
if ef is not None:
    te = T(ef["timestamp"])
    for k in ("cs_gnss_pos", "cs_gnss_vel", "cs_gps", "cs_mag_hdg", "cs_mag_3d", "cs_mag", "cs_mag_fault", "cs_yaw_align", "cs_mag_field_disturbed", "cs_inertial_dead_reckoning", "cs_gnss_yaw", "cs_gps_yaw_fault", "fs_bad_mag_x", "fs_bad_hdg", "reject_hor_vel", "reject_hor_pos", "reject_yaw", "reject_mag_x"):
        if k in ef:
            v = ef[k]; idx = [0] + [i for i in range(1, len(v)) if v[i] != v[i-1]]
            if len(idx) > 1 or v[0]: print("  %s: " % k + ", ".join("%.1fs=%d" % (te[i], v[i]) for i in idx[:16]))
es = ds("estimator_status")
if es is not None:
    te = T(es["timestamp"]); w = (te > A) & (te < B)
    for k in ("vel_test_ratio", "pos_test_ratio", "hgt_test_ratio", "mag_test_ratio", "hdg_test_ratio"):
        if k in es: print("  %s: median %.2f, max %.2f, fraction above 1: %.0f%%" % (k, np.median(es[k][w]), es[k][w].max(), 100 * (es[k][w] > 1).mean()))
ev = ds("estimator_event_flags")
if ev is not None:
    te = T(ev["timestamp"])
    for k in ev:
        if k in ("timestamp", "timestamp_sample"): continue
        v = ev[k]
        if v.dtype != np.uint64 and v[-1] != v[0] and not k.endswith("_changes"): print("  event %s first at %.1f s" % (k, te[np.argmax(v != v[0])]))
        if k in ("information_event_changes", "warning_event_changes"): print("  %s: %d" % (k, int(v[-1]) - int(v[0])))
b = ds("battery_status"); tb = T(b["timestamp"]); post = tb > B + 3
if post.any(): print("resting voltage after landing (last %d s of log): %.2f V = %.2f V/cell; current %.2f A" % (tb[-1] - tb[post][0], b["voltage_v"][post][-5:].mean(), b["voltage_v"][post][-5:].mean() / 6, b["current_a"][post][-5:].mean()))
print("SENS_BOARD_X_OFF %s  Y_OFF %s  CAL_MAG0_ROT %s  EKF2_MAG_TYPE %s  EKF2_GPS_CTRL %s" % tuple(u.initial_parameters.get(k) for k in ("SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF", "CAL_MAG0_ROT", "EKF2_MAG_TYPE", "EKF2_GPS_CTRL")))
