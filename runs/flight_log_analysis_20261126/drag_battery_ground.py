"""Three further readings of the interceptor's logs.

    .venv/bin/python runs/flight_log_analysis_20261126/drag_battery_ground.py drag LOG.ulg ...
    .venv/bin/python runs/flight_log_analysis_20261126/drag_battery_ground.py battery LOG.ulg ...
    .venv/bin/python runs/flight_log_analysis_20261126/drag_battery_ground.py ground LOG.ulg ...

drag     horizontal drag from tilt against measured acceleration and speed:
         a_measured = g * tilt_vector - k * v. Fits k (1/s), the number the fast
         sim calls drag_linear_horiz, and the pitch a steady speed would need.
battery  what the battery gauge said when the board's low and critical battery
         messages fired.
ground   how far the GPS receiver's own position and height wander with the
         aircraft sitting still, and how the Pixhawk's reported height compares.
"""
import math
import sys

import numpy as np
from pyulog import ULog

G = 9.80665


def topic(u, name):
    return next((d.data for d in u.data_list if d.name == name and d.multi_id == 0), None)


def hold(ts, xs, t):
    return xs[np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(ts) - 1)]


def drag(paths):
    V, A = [], []
    for p in paths:
        u = ULog(p, ["vehicle_local_position", "vehicle_attitude", "vehicle_land_detected"])
        lp, att, ld = topic(u, "vehicle_local_position"), topic(u, "vehicle_attitude"), topic(u, "vehicle_land_detected")
        t = lp["timestamp"] / 1e6
        air = ~hold(ld["timestamp"] / 1e6, ld["landed"], t).astype(bool) & lp["v_xy_valid"].astype(bool)
        ta = att["timestamp"] / 1e6
        q0, q1, q2, q3 = (np.interp(t, ta, att[f"q[{i}]"]) for i in range(4))
        # body z axis (down) in world; thrust is along -z_body
        zx, zy, zz = 2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)
        ok = air & (zz > 0.5)
        # horizontal acceleration the tilt asks for, if thrust holds height
        an, ae = -G * zx / zz, -G * zy / zz
        k = 13                                   # ~0.25 s smoothing at 50 Hz
        sm = lambda x: np.convolve(x, np.ones(k) / k, mode="same")   # noqa: E731
        dt = np.median(np.diff(t))
        vn, ve = sm(lp["vx"]), sm(lp["vy"])
        mn, me = np.gradient(vn, dt), np.gradient(ve, dt)
        resid_n, resid_e = sm(an) - mn, sm(ae) - me      # = k * v  (+ wind, + errors)
        sp = np.hypot(vn, ve)
        m = ok & (sp > 1.0)
        n_s = m.sum() * dt
        top = f"{sp[ok].max():.2f} m/s" if ok.any() else "none (never airborne)"
        print(f"{p.split('/')[-1]}: {n_s:.1f} s above 1 m/s, top speed {top}")
        if n_s < 3:
            continue
        V += [vn[m], ve[m]]
        A += [resid_n[m], resid_e[m]]
    if not V:
        print("no flight above 1 m/s for long enough: drag NOT MEASURED")
        return
    v, a = np.concatenate(V), np.concatenate(A)
    kfit = float(np.dot(v, a) / np.dot(v, v))
    r = float(np.corrcoef(v, a)[0, 1])
    # rough spread: fit on halves
    h = len(v) // 2
    k1, k2 = float(np.dot(v[:h], a[:h]) / np.dot(v[:h], v[:h])), float(np.dot(v[h:], a[h:]) / np.dot(v[h:], v[h:]))
    print(f"linear drag k = {kfit:.3f} 1/s (halves {k1:.3f}, {k2:.3f}; correlation {r:.2f}; "
          f"{len(v) / 2 * 0.02:.0f} s of samples, speeds to {np.abs(v).max():.1f} m/s)")
    print("steady pitch this k implies, nose down, if drag stays linear (EXTRAPOLATED above the speeds flown):")
    for s in (2, 5, 9, 12, 16, 26):
        print(f"   {s:2d} m/s: {math.degrees(math.atan(kfit * s / G)):4.1f} deg   (fast sim's 0.25: "
              f"{math.degrees(math.atan(0.25 * s / G)):4.1f} deg)")


def battery(paths):
    for p in paths:
        u = ULog(p, ["battery_status", "vehicle_land_detected"])
        b = topic(u, "battery_status")
        tb = b["timestamp"] / 1e6
        print("==", p.split("/")[-1], f"start {b['voltage_v'][0]:.2f} V, used {b['discharged_mah'][-1] - b['discharged_mah'][0]:.0f} mAh, "
              f"'remaining' {b['remaining'][0] * 100:.0f} % -> {b['remaining'][-1] * 100:.0f} %")
        for m in u.logged_messages:
            txt = m.message
            if "attery" in txt or "Failsafe" in txt:
                i = min(np.searchsorted(tb, m.timestamp / 1e6), len(tb) - 1)
                print(f"   {m.timestamp / 1e6 - tb[0]:6.1f} s  {txt.split('] ', 1)[-1].strip():<50} "
                      f"{b['voltage_v'][i]:.2f} V ({b['voltage_v'][i] / 6:.2f} per cell) {b['current_a'][i]:.1f} A, "
                      f"gauge {b['remaining'][i] * 100:.0f} %, used {b['discharged_mah'][i] - b['discharged_mah'][0]:.0f} mAh")


def ground(paths):
    R = 6378137.0
    for p in paths:
        u = ULog(p, ["sensor_gps", "vehicle_global_position", "vehicle_land_detected"])
        g, gp = topic(u, "sensor_gps"), topic(u, "vehicle_global_position")
        t = g["timestamp"] / 1e6
        lat0, lon0 = np.median(g["latitude_deg"]), np.median(g["longitude_deg"])
        n = np.radians(g["latitude_deg"] - lat0) * R
        e = np.radians(g["longitude_deg"] - lon0) * R * math.cos(math.radians(lat0))
        alt = g["altitude_msl_m"]
        print("==", p.split("/")[-1], f"{t[-1] - t[0]:.0f} s, sats {np.median(g['satellites_used']):.0f}, "
              f"speed acc {np.median(g['s_variance_m_s']):.2f} m/s")
        print(f"   GPS receiver position: RMS {math.sqrt(np.var(n) + np.var(e)):.2f} m, span N {np.ptp(n):.2f} m, E {np.ptp(e):.2f} m")
        print(f"   GPS receiver height: median {np.median(alt):.1f} m, span {np.ptp(alt):.1f} m, RMS {np.std(alt):.2f} m")
        w = 60.0                                  # worst movement inside any minute
        worst = 0.0
        for T in np.arange(t[0], t[-1] - w, 10.0):
            m = (t >= T) & (t < T + w)
            worst = max(worst, float(np.hypot(np.ptp(n[m]), np.ptp(e[m]))))
        print(f"   worst movement of the receiver's position inside any one minute: {worst:.2f} m")
        print(f"   reported GPS speed while still: median {np.median(g['vel_m_s']):.2f}, 95th pct {np.percentile(g['vel_m_s'], 95):.2f} m/s")
        if gp is not None:
            d = alt - hold(gp["timestamp"] / 1e6, gp["alt"], t)
            print(f"   GPS height minus the Pixhawk's reported height: median {np.median(d):.1f} m, "
                  f"range {d.min():.1f} to {d.max():.1f} m")


if __name__ == "__main__":
    {"drag": drag, "battery": battery, "ground": ground}[sys.argv[1]](sys.argv[2:])
