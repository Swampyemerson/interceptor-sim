"""1 s GPS quality around arming and takeoff: is the drop tied to arming (video starts), takeoff, height or heading?"""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = u.start_timestamp
def ds(n): return next(d.data for d in u.data_list if d.name == n and d.multi_id == 0)
g = ds("sensor_gps"); tg = (g["timestamp"] - t0) / 1e6
ld = ds("vehicle_land_detected"); tl = (ld["timestamp"] - t0) / 1e6
vs = ds("vehicle_status"); tv = (vs["timestamp"] - t0) / 1e6
b = ds("battery_status"); tb = (b["timestamp"] - t0) / 1e6
arm_t = tv[np.argmax(vs["arming_state"] == 2)]; air_t = tl[np.argmax(ld["landed"] == 0)]
print("armed at %.1f s, airborne at %.1f s" % (arm_t, air_t))
for T in np.arange(arm_t - 6, min(air_t + 14, tg[-1]), 1.0):
    m = (tg >= T) & (tg < T + 1)
    if not m.any(): continue
    cur = b["current_a"][(tb >= T) & (tb < T + 1)]
    print("  %6.1f | %s%s | sats %4.1f | sacc %.2f | hacc %.2f | noise %3.0f | jam %3.0f | amps %4.1f" % (T, "ARM" if T >= arm_t else "   ", " AIR" if T >= air_t else "    ",
          g["satellites_used"][m].mean(), g["s_variance_m_s"][m].mean(), g["eph"][m].mean(), g["noise_per_ms"][m].mean(), g["jamming_indicator"][m].mean(), cur.mean() if len(cur) else float("nan")))
