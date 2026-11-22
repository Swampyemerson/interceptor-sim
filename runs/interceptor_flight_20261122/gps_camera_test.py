"""GPS figures before / during / after a camera recording on the ground (windows in log seconds)."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1], ["sensor_gps", "vehicle_status"]); t0 = u.start_timestamp
g = u.get_dataset("sensor_gps").data; t = (g["timestamp"] - t0) / 1e6
on, off = float(sys.argv[2]), float(sys.argv[3])
for name, a, b in (("camera off, before", on - 25, on - 1), ("camera RECORDING", on + 3, off - 1), ("camera off, after", off + 3, off + 25)):
    w = (t > a) & (t < b)
    if not w.any(): print(name, "no data"); continue
    print("%-20s n=%2d  sats %d..%d (median %d)  jam median %3d (%d..%d)  noise %3d  speed acc %.2f m/s  hdop %.1f  fix %s" % (name, w.sum(), g["satellites_used"][w].min(), g["satellites_used"][w].max(), np.median(g["satellites_used"][w]),
          np.median(g["jamming_indicator"][w]), g["jamming_indicator"][w].min(), g["jamming_indicator"][w].max(), np.median(g["noise_per_ms"][w]), np.median(g["s_variance_m_s"][w]), np.median(g["hdop"][w]), sorted(set(g["fix_type"][w].tolist()))))
w = (t > on - 26) & (t < off + 26)
print("series t:sats/jam  " + " ".join("%.0f:%d/%d" % (a, b, c) for a, b, c in zip(t[w], g["satellites_used"][w], g["jamming_indicator"][w])))
