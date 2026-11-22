"""Clean hover window only (after the position loss, before the geofence descent)."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = u.start_timestamp; A, B = float(sys.argv[2]), float(sys.argv[3])
T = lambda ts: (ts - t0) / 1e6
def ds(name, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i: return d.data
def eul(d, pre="q"):
    w, x, y, z = (d["%s[%d]" % (pre, i)] for i in range(4))
    return np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))), np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
a = ds("vehicle_attitude"); tt = T(a["timestamp"]); r, p = eul(a)
sp = ds("vehicle_attitude_setpoint"); ts = T(sp["timestamp"]); rs, ps = eul(sp, "q_d")
w = (tt > A) & (tt < B); ri = np.interp(tt[w], ts, rs); pi = np.interp(tt[w], ts, ps)
print("window %.0f-%.0f s. attitude error RMS: roll %.2f deg pitch %.2f deg; max %.1f / %.1f; commanded tilt RMS roll %.1f pitch %.1f" % (A, B,
      np.sqrt(np.mean((r[w]-ri)**2)), np.sqrt(np.mean((p[w]-pi)**2)), np.abs(r[w]-ri).max(), np.abs(p[w]-pi).max(), ri.std(), pi.std()))
g = ds("vehicle_angular_velocity"); tg = T(g["timestamp"]); rsp = ds("vehicle_rates_setpoint"); tr = T(rsp["timestamp"])
w = (tg > A) & (tg < B); fs = 1 / np.median(np.diff(tg[w]))
for i, k in enumerate(("roll", "pitch", "yaw")):
    x = np.degrees(g["xyz[%d]" % i][w]); s = np.degrees(np.interp(tg[w], tr, rsp[k])); e = x - s
    f = np.fft.rfftfreq(len(x), 1 / fs); S = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x))))**2
    bands = [(0.3, 1), (1, 3), (3, 8), (8, 24)]
    tot = S[f > 0.3].sum()
    # lag of actual behind setpoint
    lags = np.arange(0, 12); cc = [np.corrcoef(s[:len(s)-l] if l else s, x[l:])[0, 1] for l in lags]
    print("  %-5s rate: actual RMS %.1f, setpoint RMS %.1f, error RMS %.1f deg/s, max |rate| %.0f; best lag %.0f ms (corr %.2f); energy by band %s" % (
        k, x.std(), s.std(), e.std(), np.abs(x).max(), 1000 * lags[int(np.argmax(cc))] / fs, max(cc),
        " ".join("%g-%gHz %.0f%%" % (lo, hi, 100 * S[(f >= lo) & (f < hi)].sum() / tot) for lo, hi in bands)))
m = ds("actuator_motors"); tm = T(m["timestamp"]); w = (tm > A) & (tm < B)
c = np.array([m["control[%d]" % i][w] for i in range(4)])
print("  motors mean %s std %s (max spread between motors %.2f)" % (np.round(c.mean(1), 3), np.round(c.std(1), 3), c.mean(1).max() - c.mean(1).min()))
lp = ds("vehicle_local_position"); tl = T(lp["timestamp"]); w = (tl > A) & (tl < B)
z = -lp["z"][w]; print("  height: std %.2f m, range %.2f m; vz std %.2f m/s" % (z.std(), z.max() - z.min(), lp["vz"][w].std()))
gp = ds("sensor_gps"); tg2 = T(gp["timestamp"]); w = (tg2 > A) & (tg2 < B)
lat, lon = gp["latitude_deg"][w], gp["longitude_deg"][w]
n = (lat - lat[0]) * 111320.0; e = (lon - lon[0]) * 111320.0 * np.cos(np.radians(lat[0]))
print("  GPS-reported travel in window: max %.1f m from window start; GPS ground speed mean %.1f max %.1f m/s (the estimator claimed ~12 m/s)" % (np.hypot(n, e).max(), gp["vel_m_s"][w].mean(), gp["vel_m_s"][w].max()))
mc = ds("manual_control_setpoint"); tmc = T(mc["timestamp"]); w = (tmc > A) & (tmc < B)
print("  sticks: roll std %.2f pitch std %.2f throttle mean %.2f std %.2f" % (mc["roll"][w].std(), mc["pitch"][w].std(), mc["throttle"][w].mean(), mc["throttle"][w].std()))
b = ds("battery_status"); tb = T(b["timestamp"]); w = (tb > A) & (tb < B)
print("  battery: mean %.2f A at %.2f V = %.0f W; used %.0f mAh in %.0f s" % (b["current_a"][w].mean(), b["voltage_v"][w].mean(), (b["current_a"][w] * b["voltage_v"][w]).mean(), b["discharged_mah"][w][-1] - b["discharged_mah"][w][0], B - A))
# jamming / sats around arming and takeoff
w = (tg2 > 150) & (tg2 < 190)
print("  GPS around arming (172.2 armed, ~173.8 video starts, 175.8 takeoff): t:sats/jam/noise  " + "  ".join("%.0f:%d/%d/%d" % (t_, s_, j_, n_) for t_, s_, j_, n_ in list(zip(tg2[w], gp["satellites_used"][w], gp["jamming_indicator"][w], gp["noise_per_ms"][w]))[::2]))
mg0, mg1 = ds("sensor_mag", 0), ds("sensor_mag", 1)
for i, mg in enumerate((mg0, mg1)):
    if mg is None: continue
    tmg = T(mg["timestamp"]); gnd = (tmg > 150) & (tmg < 170); fl = (tmg > A) & (tmg < B)
    nrm = np.sqrt(mg["x"]**2 + mg["y"]**2 + mg["z"]**2)
    print("  mag%d field strength: ground %.3f G, airborne mean %.3f (std %.3f) G" % (i, nrm[gnd].mean(), nrm[fl].mean(), nrm[fl].std()))
