"""Second flight, 2026-11-22: battery and sag, full-throttle thrust, GPS, lean angle against speed, level trim."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = u.start_timestamp
T = lambda ts: (ts - t0) / 1e6
G = 9.81; MASS = 0.930
def ds(name, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i: return d.data
def eul(d, pre="q"):
    w, x, y, z = (d["%s[%d]" % (pre, i)] for i in range(4))
    return (np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)), np.arcsin(np.clip(2*(w*y-z*x), -1, 1)), np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
ld = ds("vehicle_land_detected"); tl = T(ld["timestamp"]); air = tl[ld["landed"] == 0]; ta, td = air[0], 139.5
print("airborne %.1f to %.1f s" % (ta, td))
# ---- battery
b = ds("battery_status"); tb = T(b["timestamp"]); V, I = b["voltage_v"], b["current_a"]
pre = tb < 3.0; w = (tb > ta + 1) & (tb < td)
print("BATTERY: resting before arming %.2f V (%.2f V/cell), 'remaining' %.2f; capacity param seen %s" % (V[pre].mean(), V[pre].mean() / 6, b["remaining"][pre].mean(), sorted(set(b["capacity"].tolist()))))
print("  in flight: I mean %.1f A, max %.1f A; V min %.2f (%.2f V/cell) at %.1f s; used %.0f mAh; remaining end %.2f; internal R estimate %.4f" % (
    I[w].mean(), I[w].max(), V[w].min(), V[w].min() / 6, tb[w][np.argmin(V[w])], b["discharged_mah"][w][-1] - b["discharged_mah"][w][0], b["remaining"][w][-1], b["internal_resistance_estimate"][w][-1]))
k = np.polyfit(I[w], V[w], 1); print("  whole-flight fit V = %.2f - %.4f*I" % (k[1], -k[0]))
for lo, hi in ((10, 50), (50, 90), (90, 139)):
    ww = (tb > lo) & (tb < hi)
    if ww.sum() > 20:
        kk = np.polyfit(I[ww], V[ww], 1); print("  %3d-%3d s: V = %.2f - %.4f*I  (I %.1f..%.1f A), remaining %.2f -> %.2f, time_remaining %.0f -> %.0f s, warning %s" % (lo, hi, kk[1], -kk[0], I[ww].min(), I[ww].max(), b["remaining"][ww][0], b["remaining"][ww][-1], b["time_remaining_s"][ww][0], b["time_remaining_s"][ww][-1], sorted(set(b["warning"][ww].tolist()))))
# ---- full throttle events
m = ds("actuator_motors"); tm = T(m["timestamp"]); c = np.array([m["control[%d]" % i] for i in range(4)]); cm = c.mean(0)
s = ds("sensor_combined"); ts = T(s["timestamp"]); az = s["accelerometer_m_s2[2]"]; ax = s["accelerometer_m_s2[0]"]; ay = s["accelerometer_m_s2[1]"]
def smooth(x, n): return np.convolve(x, np.ones(n) / n, mode="same")
azs = smooth(az, 21)       # ~0.1 s
print("THROTTLE: motor command mean over flight %.2f; max of the 4-motor mean %.2f; samples with all four above 0.95: %d (log rate %.0f Hz)" % (cm[(tm > ta) & (tm < td)].mean(), cm[(tm > ta) & (tm < td)].max(), int(((c.min(0) > 0.95) & (tm > ta) & (tm < td)).sum()), 1 / np.median(np.diff(tm))))
ev = []; hi = (cm > 0.6) & (tm > ta) & (tm < td); i = 0
while i < len(tm):
    if hi[i]:
        j = i
        while j + 1 < len(tm) and hi[j + 1]: j += 1
        ev.append((tm[i], tm[j])); i = j + 1
    else: i += 1
merged = []
for a_, b_ in ev:
    if merged and a_ - merged[-1][1] < 1.0: merged[-1] = (merged[-1][0], b_)
    else: merged.append((a_, b_))
vs = ds("vehicle_status"); tv = T(vs["timestamp"])
for a_, b_ in merged:
    ws = (ts > a_ - 0.1) & (ts < b_ + 0.1); wb = (tb > a_ - 0.3) & (tb < b_ + 0.5); wm = (tm >= a_) & (tm <= b_)
    if not ws.any() or not wb.any(): continue
    f = -azs[ws].min()                       # peak specific force along body up, m/s2
    nav = vs["nav_state"][min(np.searchsorted(tv, a_), len(tv) - 1)]
    print("  punch %.1f-%.1f s (mode %d): motor mean peak %.2f, per-motor peaks %s; peak up-force %.1f m/s2 = %.2f g -> %.2f kg total, %.2f kg per motor; I peak %.1f A, V min %.2f" % (
        a_, b_, nav, cm[wm].max(), np.round(c[:, wm].max(1), 2), f, f / G, MASS * f / G, MASS * f / G / 4, I[wb].max(), V[wb].min()))
# thrust vs command curve (binned): specific force vs mean motor command
w = (tm > ta + 1) & (tm < td); fz = -np.interp(tm[w], ts, azs)
print("  up-force by motor command bin (median m/s2 | implied kg per motor | n):")
for lo, hi_ in ((0.15, 0.2), (0.2, 0.25), (0.25, 0.3), (0.3, 0.4), (0.4, 0.5), (0.5, 0.6), (0.6, 0.7), (0.7, 0.8), (0.8, 0.9), (0.9, 1.01)):
    ww = (cm[w] >= lo) & (cm[w] < hi_)
    if ww.sum() >= 3: print("    %.2f-%.2f: %5.1f | %.2f | %d" % (lo, hi_, np.median(fz[ww]), MASS * np.median(fz[ww]) / G / 4, ww.sum()))
# ---- gps
g = ds("sensor_gps"); tg = T(g["timestamp"]); w = (tg > ta) & (tg < td)
print("GPS in flight: sats median %d (%d..%d), fix %s, speed accuracy median %.2f (%.2f..%.2f), hdop median %.1f, jam median %d; rate %.1f Hz" % (np.median(g["satellites_used"][w]), g["satellites_used"][w].min(), g["satellites_used"][w].max(),
      sorted(set(g["fix_type"][w].tolist())), np.median(g["s_variance_m_s"][w]), g["s_variance_m_s"][w].min(), g["s_variance_m_s"][w].max(), np.median(g["hdop"][w]), np.median(g["jamming_indicator"][w]), 1 / np.median(np.diff(tg))))
print("  sats by 10 s: " + " ".join("%d:%d" % (a_, np.median(g["satellites_used"][(tg >= a_) & (tg < a_ + 10)])) for a_ in range(0, 150, 10) if ((tg >= a_) & (tg < a_ + 10)).any()))
# ---- attitude / speed / lean
a = ds("vehicle_attitude"); tt = T(a["timestamp"]); r, p, y = eul(a)
tilt = np.degrees(np.arccos(np.clip(np.cos(r) * np.cos(p), -1, 1)))
w = (tt > ta + 1) & (tt < td)
print("LEAN: tilt median %.1f deg, 95th pct %.1f, max %.1f deg (roll %.1f..%.1f, pitch %.1f..%.1f); yaw range %.0f..%.0f deg" % (np.median(tilt[w]), np.percentile(tilt[w], 95), tilt[w].max(), np.degrees(r[w]).min(), np.degrees(r[w]).max(), np.degrees(p[w]).min(), np.degrees(p[w]).max(), np.degrees(y[w]).min(), np.degrees(y[w]).max()))
gp = ds("vehicle_gps_position") if ds("vehicle_gps_position") is not None else g
tgp = T(gp["timestamp"]); vn, ve = gp["vel_n_m_s"], gp["vel_e_m_s"]; spd = np.hypot(vn, ve)
w = (tgp > ta) & (tgp < td)
print("  GPS ground speed: median %.1f, 95th pct %.1f, max %.1f m/s (GPS velocity rate %.1f Hz)" % (np.median(spd[w]), np.percentile(spd[w], 95), spd[w].max(), 1 / np.median(np.diff(tgp))))
lat, lon = g["latitude_deg"][(tg > ta) & (tg < td)], g["longitude_deg"][(tg > ta) & (tg < td)]
n_ = (lat - lat[0]) * 111320.0; e_ = (lon - lon[0]) * 111320.0 * np.cos(np.radians(lat[0]))
print("  GPS track extent: north %.0f m, east %.0f m; max distance from takeoff %.0f m" % (n_.max() - n_.min(), e_.max() - e_.min(), np.hypot(n_, e_).max()))
lp = ds("vehicle_local_position"); tlp = T(lp["timestamp"]); w = (tlp > ta) & (tlp < td)
print("  height above takeoff: max %.1f m; climb rate max %.1f m/s, sink max %.1f m/s (estimator, baro-led)" % ((-(lp["z"][w] - lp["z"][w][0])).max(), (-lp["vz"][w]).max(), lp["vz"][w].max()))
# ---- level trim + drag fit:  dv/dt = tilt accel(offsets) - k (v - wind)
dt = 0.1; tq = np.arange(ta + 2, td - 1, dt)
def at(tsrc, x, n=11): return smooth(np.interp(tq, tsrc, x), n)
vN, vE = at(tgp, vn), at(tgp, ve)
aN, aE = np.gradient(vN, dt), np.gradient(vE, dt)
R, P, Y = at(tt, r), at(tt, p), np.interp(tq, tt, np.unwrap(y))
thr = at(tm, cm); sacc = np.interp(tq, tg, g["s_variance_m_s"])
good = (np.abs(thr - np.median(thr)) < 0.06) & (sacc < 1.2)
print("TRIM/DRAG fit: %d of %d samples used (throttle near hover, GPS speed accuracy under 1.2 m/s); yaw spread in those %.0f deg" % (good.sum(), len(tq), np.degrees(Y[good].max() - Y[good].min())))
bN = -G * np.tan(P) * np.cos(Y) - G * np.tan(R) * np.sin(Y)
bE = -G * np.tan(P) * np.sin(Y) + G * np.tan(R) * np.cos(Y)
A = np.zeros((2 * good.sum(), 5)); rhs = np.zeros(2 * good.sum()); n = good.sum()
A[:n, 0] = G * np.sin(Y[good]); A[:n, 1] = G * np.cos(Y[good]); A[:n, 2] = -vN[good]; A[:n, 3] = 1
A[n:, 0] = -G * np.cos(Y[good]); A[n:, 1] = G * np.sin(Y[good]); A[n:, 2] = -vE[good]; A[n:, 4] = 1
rhs[:n] = aN[good] - bN[good]; rhs[n:] = aE[good] - bE[good]
x, res, rk, sv = np.linalg.lstsq(A, rhs, rcond=None)
resid = rhs - A @ x
# crude uncertainty by block bootstrap (10 s blocks)
rng = np.random.default_rng(0); blk = 100; idx = np.arange(n); nb = max(2, n // blk); boots = []
for _ in range(300):
    pick = np.concatenate([idx[s_:s_ + blk] for s_ in rng.integers(0, max(1, n - blk), nb)])
    xb = np.linalg.lstsq(np.vstack([A[pick], A[n + pick]]), np.concatenate([rhs[pick], rhs[n + pick]]), rcond=None)[0]; boots.append(xb)
sd = np.std(boots, 0)
print("  roll offset %.2f deg (+-%.2f), pitch offset %.2f deg (+-%.2f); linear drag k %.3f 1/s (+-%.3f); wind-like term N %.2f E %.2f m/s2; residual RMS %.2f m/s2; condition %.0f" % (
    np.degrees(x[0]), np.degrees(sd[0]), np.degrees(x[1]), np.degrees(sd[1]), x[2], sd[2], x[3], x[4], resid.std(), sv[0] / sv[-1]))
print("  mean attitude over those samples: roll %.2f deg, pitch %.2f deg; mean GPS velocity N %.2f E %.2f m/s" % (np.degrees(R[good].mean()), np.degrees(P[good].mean()), vN[good].mean(), vE[good].mean()))
mc = ds("manual_control_setpoint"); tmc = T(mc["timestamp"]); w = (tmc > ta + 2) & (tmc < td)
print("  sticks over flight: roll mean %.3f, pitch mean %.3f (1.0 = full stick)" % (mc["roll"][w].mean(), mc["pitch"][w].mean()))
# lean vs speed table from GPS speed bins (steady samples)
sp = np.hypot(vN, vE); acc = np.hypot(aN, aE); tl_ = np.degrees(np.arccos(np.clip(np.cos(R) * np.cos(P), -1, 1)))
print("  lean angle by GPS speed, samples with horizontal acceleration under 1 m/s2 (median deg | n):")
for lo, hi_ in ((0, 2), (2, 4), (4, 6), (6, 8), (8, 10), (10, 14), (14, 20)):
    ww = (sp >= lo) & (sp < hi_) & (acc < 1.0) & (sacc < 1.2)
    if ww.sum() >= 5: print("    %2d-%2d m/s: %.1f | %d" % (lo, hi_, np.median(tl_[ww]), ww.sum()))
print("PARAMS:", {k_: u.initial_parameters.get(k_) for k_ in ("SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF", "SENS_BOARD_Z_OFF", "TRIM_ROLL", "TRIM_PITCH", "MPC_MAN_TILT_MAX", "MPC_TILTMAX_AIR", "MPC_Z_VEL_MAX_UP", "MPC_MANTHR_MIN", "MPC_THR_MAX", "MPC_XY_VEL_MAX", "COM_FLTMODE1", "COM_FLTMODE4", "COM_FLTMODE6", "COM_FLT_TIME_MAX", "COM_LOW_BAT_ACT", "BAT_LOW_THR", "BAT1_CAPACITY", "COM_ARM_BAT_MIN")})
