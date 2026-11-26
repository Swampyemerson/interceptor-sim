"""GPS quality against UTC time for one or more logs: satellites, speed accuracy, noise and jamming indicator, every N s."""
import sys, time, numpy as np
from pyulog import ULog
step = float(__import__("os").environ.get("STEP", 6.0))
for f in sys.argv[1:]:
    u = ULog(f, ["sensor_gps", "vehicle_land_detected", "vehicle_status"])
    g = next(d.data for d in u.data_list if d.name == "sensor_gps")
    ld = next(d.data for d in u.data_list if d.name == "vehicle_land_detected")
    t = g["timestamp"] / 1e6; utc = g["time_utc_usec"] / 1e6
    print("==", f.split("/")[-1]); print("  utc      | air | sats | sacc  | hacc | noise | jam | fix")
    for T in np.arange(t[0], t[-1], step):
        m = (t >= T) & (t < T + step)
        if not m.any(): continue
        la = ld["landed"][np.searchsorted(ld["timestamp"] / 1e6, T + step / 2) - 1]
        print("  %s |  %s  | %4.1f | %5.2f | %4.2f | %5.0f | %3.0f | %d" % (time.strftime("%H:%M:%S", time.gmtime(utc[m][0])), " " if la else "A",
              g["satellites_used"][m].mean(), g["s_variance_m_s"][m].mean(), g["eph"][m].mean(), g["noise_per_ms"][m].mean(), g["jamming_indicator"][m].mean(), g["fix_type"][m].min()))
