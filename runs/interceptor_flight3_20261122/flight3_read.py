"""Third flight, 2026-11-22: Position-mode hold, what the pilot felt, estimator, compass, GPS checks."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = u.start_timestamp
T = lambda ts: (ts - t0) / 1e6; G = 9.81
def ds(name, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i: return d.data
def eul(d, pre="q"):
    w, x, y, z = (d["%s[%d]" % (pre, i)] for i in range(4))
    return (np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)), np.arcsin(np.clip(2*(w*y-z*x), -1, 1)), np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
def smooth(x, n): return np.convolve(x, np.ones(n) / n, mode="same")
def changes(name, arr, tt, lim=24):
    idx = [0] + [i for i in range(1, len(arr)) if arr[i] != arr[i-1]]
    print("  %s: " % name + ", ".join("%.1fs=%s" % (tt[i], arr[i]) for i in idx[:lim]))
ld = ds("vehicle_land_detected"); tl = T(ld["timestamp"]); air = tl[ld["landed"] == 0]; ta, td = air[0], air[-1]
print("airborne %.1f to %.1f s (%.1f s)" % (ta, td, td - ta))
P = u.initial_parameters
print("position-mode feel params:", {k: (round(P[k], 3) if isinstance(P.get(k), float) else P.get(k)) for k in ("MPC_POS_MODE", "MPC_VEL_MANUAL", "MPC_ACC_HOR", "MPC_ACC_HOR_MAX", "MPC_JERK_MAX", "MPC_JERK_AUTO", "MPC_MAN_TILT_MAX", "MPC_TILTMAX_AIR", "MPC_MAN_Y_MAX", "MPC_MAN_Y_TAU", "MPC_XY_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC", "MPC_Z_P", "MPC_Z_VEL_P_ACC", "MPC_Z_VEL_I_ACC", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "MPC_ACC_UP_MAX", "MPC_ACC_DOWN_MAX", "MPC_HOLD_MAX_XY", "MPC_VELD_LP", "MPC_XY_VEL_ALL", "MPC_LAND_ALT1", "MPC_LAND_ALT2", "MPC_XY_ERR_MAX", "MC_ROLL_P", "MC_PITCH_P", "MC_ROLLRATE_P", "MC_ROLLRATE_MAX", "MC_PITCHRATE_MAX", "MC_YAWRATE_MAX", "EKF2_HGT_REF", "EKF2_GPS_DELAY", "EKF2_GPS_P_NOISE", "EKF2_GPS_V_NOISE", "EKF2_MAG_TYPE", "EKF2_MAG_CHECK", "EKF2_MAG_NOISE", "SENS_MAG_MODE", "CAL_MAG0_ROT", "CAL_MAG0_XOFF", "CAL_MAG0_YOFF", "CAL_MAG0_ZOFF", "CAL_MAG0_XSCALE", "CAL_MAG0_YSCALE", "CAL_MAG0_ZSCALE")})
vs = ds("vehicle_status"); tv = T(vs["timestamp"])
a = ds("vehicle_attitude"); tt = T(a["timestamp"]); r, p, y = eul(a)
sp = ds("vehicle_attitude_setpoint"); ts = T(sp["timestamp"]); rs, ps, ys = eul(sp, "q_d")
lp = ds("vehicle_local_position"); tlp = T(lp["timestamp"])
lsp = ds("vehicle_local_position_setpoint"); tls = T(lsp["timestamp"]) if lsp is not None else None
gp = ds("vehicle_gps_position"); tg = T(gp["timestamp"])
g = ds("sensor_gps"); tgs = T(g["timestamp"])
mc = ds("manual_control_setpoint"); tmc = T(mc["timestamp"])
# ---- per-second story of the flight
z0 = lp["z"][np.searchsorted(tlp, ta)]
lat0, lon0 = g["latitude_deg"][np.searchsorted(tgs, ta)], g["longitude_deg"][np.searchsorted(tgs, ta)]
x0, y0 = lp["x"][np.searchsorted(tlp, ta)], lp["y"][np.searchsorted(tlp, ta)]
print("every 2 s:  t | mode | height | est pos N,E (m) | GPS pos N,E (m) | est speed | GPS speed | roll,pitch | yaw | sats sacc | stick r,p,thr | xy_valid")
for tq in np.arange(ta - 2, td + 1, 2.0):
    i = min(np.searchsorted(tlp, tq), len(tlp) - 1); j = min(np.searchsorted(tgs, tq), len(tgs) - 1); k = min(np.searchsorted(tt, tq), len(tt) - 1)
    m = min(np.searchsorted(tmc, tq), len(tmc) - 1); n = min(np.searchsorted(tv, tq), len(tv) - 1); q = min(np.searchsorted(tg, tq), len(tg) - 1)
    gn = (g["latitude_deg"][j] - lat0) * 111320.0; ge = (g["longitude_deg"][j] - lon0) * 111320.0 * np.cos(np.radians(lat0))
    print("  %6.1f | %2d | %5.2f | %6.1f %6.1f | %6.1f %6.1f | %4.1f | %4.1f | %5.1f %5.1f | %4.0f | %2d %.2f | %5.2f %5.2f %5.2f | %d" % (
        tq, vs["nav_state"][n], -(lp["z"][i] - z0), lp["x"][i] - x0, lp["y"][i] - y0, gn, ge, np.hypot(lp["vx"][i], lp["vy"][i]), np.hypot(gp["vel_n_m_s"][q], gp["vel_e_m_s"][q]),
        np.degrees(r[k]), np.degrees(p[k]), np.degrees(y[k]), g["satellites_used"][j], g["s_variance_m_s"][j], mc["roll"][m], mc["pitch"][m], mc["throttle"][m], lp["xy_valid"][i]))
# ---- position-mode window
pos = (tv > ta) & (vs["nav_state"] == 2) & (tv < 397.0)
A, B = ta + 1.5, 396.5
w = (tt > A) & (tt < B); ri = np.interp(tt[w], ts, rs); pi = np.interp(tt[w], ts, ps)
print("POSITION MODE %.1f-%.1f s: roll %.1f..%.1f deg (mean %.2f), pitch %.1f..%.1f (mean %.2f); commanded tilt std roll %.1f pitch %.1f; attitude error RMS %.2f / %.2f deg" % (A, B,
      np.degrees(r[w]).min(), np.degrees(r[w]).max(), np.degrees(r[w]).mean(), np.degrees(p[w]).min(), np.degrees(p[w]).max(), np.degrees(p[w]).mean(), np.degrees(ri).std(), np.degrees(pi).std(),
      np.degrees(np.sqrt(np.mean((r[w] - ri)**2))), np.degrees(np.sqrt(np.mean((p[w] - pi)**2)))))
w = (tmc > A) & (tmc < B)
print("  sticks: roll mean %.2f std %.2f (max |%.2f|), pitch mean %.2f std %.2f (max |%.2f|), throttle mean %.2f std %.2f" % (mc["roll"][w].mean(), mc["roll"][w].std(), np.abs(mc["roll"][w]).max(), mc["pitch"][w].mean(), mc["pitch"][w].std(), np.abs(mc["pitch"][w]).max(), mc["throttle"][w].mean(), mc["throttle"][w].std()))
# stick -> attitude response lag, this flight (position mode) : cross-correlate stick with attitude
def lag_of(tsrc, stick, tatt, att, A, B, maxlag=2.0, dt=0.02):
    tq = np.arange(A, B, dt); s = np.interp(tq, tsrc, stick); x = np.interp(tq, tatt, att)
    s = s - s.mean(); x = x - x.mean()
    if s.std() < 1e-3: return None
    best = max(((np.corrcoef(s[:len(s) - L], x[L:])[0, 1], L * dt) for L in range(0, int(maxlag / dt))), key=lambda v: abs(v[0]))
    return best
for nm, st, at in (("roll", mc["roll"], np.degrees(r)), ("pitch", mc["pitch"], -np.degrees(p))):
    b = lag_of(tmc, st, tt, at, A, B)
    if b: print("  stick-to-attitude, %s: best correlation %.2f at %.2f s delay" % (nm, b[0], b[1]))
# velocity tracking in position mode
if lsp is not None:
    w = (tlp > A) & (tlp < B)
    vxs = np.interp(tlp[w], tls, lsp["vx"]); vys = np.interp(tlp[w], tls, lsp["vy"])
    ok = np.isfinite(vxs) & np.isfinite(vys)
    if ok.any():
        print("  velocity: setpoint speed max %.1f m/s; tracking error RMS %.2f m/s; estimated speed max %.1f" % (np.hypot(vxs[ok], vys[ok]).max(), np.sqrt(np.mean((lp["vx"][w][ok] - vxs[ok])**2 + (lp["vy"][w][ok] - vys[ok])**2)), np.hypot(lp["vx"][w], lp["vy"][w]).max()))
    zs = np.interp(tlp[w], tls, lsp["z"]); okz = np.isfinite(zs)
    if okz.any(): print("  height: setpoint follows? error RMS %.2f m; height range %.2f m; vz std %.2f m/s" % (np.sqrt(np.mean((lp["z"][w][okz] - zs[okz])**2)), np.ptp(lp["z"][w]), lp["vz"][w].std()))
# estimator position against GPS position in the window (does the estimate wander?)
w = (tlp > A) & (tlp < B)
gn = (np.interp(tlp[w], tgs, g["latitude_deg"]) - lat0) * 111320.0; ge = (np.interp(tlp[w], tgs, g["longitude_deg"]) - lon0) * 111320.0 * np.cos(np.radians(lat0))
en, ee = lp["x"][w] - x0, lp["y"][w] - y0
print("  estimator against raw GPS position: difference RMS %.2f m (max %.2f); raw GPS wander: N %.1f m, E %.1f m; estimated wander: N %.1f m, E %.1f m" % (np.sqrt(np.mean((en - gn)**2 + (ee - ge)**2)), np.hypot(en - gn, ee - ge).max(), np.ptp(gn), np.ptp(ge), np.ptp(en), np.ptp(ee)))
# rates
gy = ds("vehicle_angular_velocity"); tgy = T(gy["timestamp"]); rsp = ds("vehicle_rates_setpoint"); trs = T(rsp["timestamp"])
w = (tgy > A) & (tgy < B); fs = 1 / np.median(np.diff(tgy[w]))
for i, k in enumerate(("roll", "pitch", "yaw")):
    x = np.degrees(gy["xyz[%d]" % i][w]); s = np.degrees(np.interp(tgy[w], trs, rsp[k])); f = np.fft.rfftfreq(len(x), 1 / fs); S = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x))))**2
    tot = S[f > 0.3].sum(); pk = f[f > 0.3][np.argmax(S[f > 0.3])]
    print("  %-5s rate: RMS %.1f deg/s (setpoint %.1f), max %.0f; peak at %.1f Hz; energy 0.3-1 Hz %.0f%%, 1-3 %.0f%%, 3-8 %.0f%%, 8-24 %.0f%%" % (k, x.std(), s.std(), np.abs(x).max(), pk,
          *[100 * S[(f >= lo) & (f < hi)].sum() / tot for lo, hi in ((0.3, 1), (1, 3), (3, 8), (8, 24))]))
m = ds("actuator_motors"); tm = T(m["timestamp"]); w = (tm > A) & (tm < B); c = np.array([m["control[%d]" % i][w] for i in range(4)])
print("  motors mean %s, std %s; thrust setpoint std %.3f" % (np.round(c.mean(1), 3), np.round(c.std(1), 3), c.mean(0).std()))
# ---- estimator / compass / gps checks over the airborne time
ef = ds("estimator_status_flags"); te = T(ef["timestamp"])
print("ESTIMATOR flags (changes while armed):")
for k in ("cs_gnss_pos", "cs_gnss_vel", "cs_mag_hdg", "cs_mag_3d", "cs_mag", "cs_mag_fault", "cs_yaw_align", "cs_inertial_dead_reckoning", "cs_gnss_yaw", "reject_hor_vel", "reject_hor_pos", "reject_yaw", "reject_mag_x", "reject_mag_y", "reject_mag_z", "fs_bad_mag_x", "fs_bad_mag_y", "fs_bad_mag_z", "fs_bad_hdg"):
    if k in ef:
        w = te > ta - 8; v = ef[k][w]
        if len(set(v.tolist())) > 1 or v[0]: changes(k, v, te[w])
es = ds("estimator_status"); te2 = T(es["timestamp"]); w = (te2 > ta) & (te2 < td)
for k in ("vel_test_ratio", "pos_test_ratio", "hgt_test_ratio", "mag_test_ratio", "hdg_test_ratio"):
    if k in es and np.isfinite(es[k][w]).any(): print("  %s: median %.2f, max %.2f (at %.1f s), above 1: %.0f%%" % (k, np.nanmedian(es[k][w]), np.nanmax(es[k][w]), te2[w][np.nanargmax(es[k][w])], 100 * np.nanmean(es[k][w] > 1)))
eg = ds("estimator_gps_status")
if eg is not None:
    teg = T(eg["timestamp"]); w = (teg > ta) & (teg < td)
    print("  GPS checks failing while airborne: " + ", ".join("%s %.0f%%" % (k.replace("check_fail_", ""), 100 * eg[k][w].mean()) for k in eg if k.startswith("check_fail") and eg[k][w].any()))
    changes("checks_passed", eg["checks_passed"][teg > ta - 30], teg[teg > ta - 30], 30)
ev = ds("estimator_event_flags")
if ev is not None:
    tev = T(ev["timestamp"])
    for k in ev:
        if k in ("timestamp", "timestamp_sample") or k.endswith("_changes"): continue
        v = ev[k]; idx = [i for i in range(1, len(v)) if v[i] and not v[i-1] and tev[i] > ta - 10]
        if idx: print("  event %s at %s s" % (k, [round(float(tev[i]), 1) for i in idx[:8]]))
# yaw: estimated heading vs time, and reset counters
print("  yaw estimate: start %.0f deg, end %.0f deg, range %.0f..%.0f; quat_reset_counter %d -> %d" % (np.degrees(y[np.searchsorted(tt, ta)]), np.degrees(y[np.searchsorted(tt, td) - 1]), np.degrees(y[(tt > ta) & (tt < td)]).min(), np.degrees(y[(tt > ta) & (tt < td)]).max(), a["quat_reset_counter"][np.searchsorted(tt, ta)], a["quat_reset_counter"][np.searchsorted(tt, td) - 1]))
b = ds("battery_status"); tb = T(b["timestamp"])
for i in range(2):
    mg = ds("sensor_mag", i)
    if mg is None: continue
    tmg = T(mg["timestamp"]); nrm = np.sqrt(mg["x"]**2 + mg["y"]**2 + mg["z"]**2)
    gnd = (tmg > ta - 60) & (tmg < ta - 5); fl = (tmg > ta + 1) & (tmg < td - 1)
    cur = np.interp(tmg[fl], tb, b["current_a"]); k = np.polyfit(cur, nrm[fl], 1)
    print("  mag%d: field on the ground %.3f G (std %.3f); airborne %.3f G (std %.3f, range %.3f..%.3f); per amp %.4f G (corr %.2f); rate %.1f Hz; ground xyz mean %s, air xyz mean %s" % (i, nrm[gnd].mean(), nrm[gnd].std(), nrm[fl].mean(), nrm[fl].std(), nrm[fl].min(), nrm[fl].max(), k[0], np.corrcoef(cur, nrm[fl])[0, 1], 1 / np.median(np.diff(tmg)),
          np.round([mg[c_][gnd].mean() for c_ in "xyz"], 3), np.round([mg[c_][fl].mean() for c_ in "xyz"], 3)))
# heading check against GPS (as in flight 2)
dt = 0.1; tq = np.arange(ta + 1, td - 1, dt)
vN = smooth(np.interp(tq, tg, gp["vel_n_m_s"]), 7); vE = smooth(np.interp(tq, tg, gp["vel_e_m_s"]), 7); aN, aE = np.gradient(vN, dt), np.gradient(vE, dt)
R = smooth(np.interp(tq, tt, r), 7); Pp = smooth(np.interp(tq, tt, p), 7); Y = np.interp(tq, tt, np.unwrap(y))
ab = (-G * np.tan(Pp)) + 1j * (G * np.tan(R)); aw = ab * np.exp(1j * Y); ag = aN + 1j * aE
for nm, lo, hi in (("position mode", ta + 1, 397), ("after the dropout", 397, td - 1), ("whole flight", ta + 1, td - 1)):
    sel = (tq > lo) & (tq < hi) & (np.abs(ab) > 0.7)
    if sel.sum() < 15: print("  heading test, %s: too few tilts (%d)" % (nm, sel.sum())); continue
    best = None
    for lag in range(0, 9):
        s1 = np.where(sel)[0]; s1 = s1[s1 + lag < len(tq)]
        z = np.sum(ag[s1 + lag] * np.conj(aw[s1])); cc = np.abs(z) / np.sqrt(np.sum(np.abs(ag[s1 + lag])**2) * np.sum(np.abs(aw[s1])**2))
        if best is None or cc > best[0]: best = (cc, lag * dt, np.degrees(np.angle(z)))
    print("  heading test, %s (%d samples): correlation %.2f at GPS lag %.1f s, heading must turn %.0f deg to agree" % (nm, sel.sum(), best[0], best[1], best[2]))
w = (tb > ta + 1) & (tb < td - 1)
print("BATTERY: before %.2f V; airborne I mean %.1f A, V mean %.2f min %.2f; used %.0f mAh; resting after %.2f V" % (b["voltage_v"][(tb > ta - 30) & (tb < ta - 8)].mean(), b["current_a"][w].mean(), b["voltage_v"][w].mean(), b["voltage_v"][w].min(), b["discharged_mah"][w][-1] - b["discharged_mah"][w][0], b["voltage_v"][tb > td + 3][-5:].mean()))
ws = ds("sensor_combined"); tws = T(ws["timestamp"]); w = (tws > A) & (tws < B)
print("VIBRATION accel std x %.1f y %.1f z %.1f m/s2" % tuple(ws["accelerometer_m_s2[%d]" % i][w].std() for i in range(3)))
