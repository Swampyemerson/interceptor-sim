"""Heading hold in a hands-off Position-mode hover, and the largest lean and speed per flight mode.

    .venv/bin/python hover_yaw_and_modes.py LOG.ulg ...
"""
import sys
import numpy as np
from pyulog import ULog
NAV = {0: "Manual", 1: "Altitude", 2: "Position", 4: "Hold", 5: "Return", 14: "Offboard", 15: "Stabilized"}
def hold(ts, xs, t): return xs[np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(ts) - 1)]
for p in sys.argv[1:]:
    u = ULog(p, ["vehicle_local_position", "vehicle_status", "manual_control_setpoint", "vehicle_land_detected", "vehicle_attitude"])
    d = {x.name: x.data for x in u.data_list}
    lp, st, mc, ld, att = (d[k] for k in ("vehicle_local_position", "vehicle_status", "manual_control_setpoint", "vehicle_land_detected", "vehicle_attitude"))
    t = lp["timestamp"] / 1e6
    air = ~hold(ld["timestamp"] / 1e6, ld["landed"], t).astype(bool)
    nav = hold(st["timestamp"] / 1e6, st["nav_state"], t)
    stick = hold(mc["timestamp"] / 1e6, np.maximum.reduce([np.abs(mc["roll"]), np.abs(mc["pitch"]), np.abs(mc["yaw"])]), t)
    q1, q2 = (np.interp(t, att["timestamp"] / 1e6, att[f"q[{i}]"]) for i in (1, 2))
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q1 * q1 + q2 * q2), -1, 1)))
    sp = np.hypot(lp["vx"], lp["vy"])
    print("==", p.split("/")[-1])
    for k in np.unique(nav[air]) if air.any() else []:
        m = air & (nav == k)
        print(f"   {NAV.get(int(k), int(k)):<10} {m.sum() * np.median(np.diff(t)):6.1f} s  lean max {tilt[m].max():5.1f} deg  speed max {sp[m].max():5.2f} m/s")
    m = air & (nav == 2) & (stick < 0.05)
    i, n = 0, len(t)
    while i < n:
        if not m[i]: i += 1; continue
        j = i
        while j < n and m[j]: j += 1
        if t[j - 1] - t[i] >= 8.0:
            k0 = i + int(np.searchsorted(t[i:j], t[i] + 2.0))
            h = np.degrees(np.unwrap(lp["heading"][k0:j]))
            print(f"   hands-off hover {t[j - 1] - t[k0]:5.1f} s: heading RMS {np.std(h):.2f} deg, span {np.ptp(h):.2f} deg, drift {h[-1] - h[0]:+.2f} deg")
        i = j
