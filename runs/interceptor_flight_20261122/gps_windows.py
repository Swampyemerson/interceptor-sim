"""GPS figures in named windows of a log: args = log, then name:start:end ..."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1], ["sensor_gps"]); t0 = u.start_timestamp
g = u.get_dataset("sensor_gps").data; t = (g["timestamp"] - t0) / 1e6
for spec in sys.argv[2:]:
    name, a, b = spec.split(":"); w = (t > float(a)) & (t < float(b))
    if not w.any(): print(name, "no data"); continue
    print("%-34s n=%2d  sats median %d (%d..%d)  jam median %3d  speed acc %.2f m/s  fix %s" % (name, w.sum(), np.median(g["satellites_used"][w]), g["satellites_used"][w].min(), g["satellites_used"][w].max(),
          np.median(g["jamming_indicator"][w]), np.median(g["s_variance_m_s"][w]), sorted(set(g["fix_type"][w].tolist()))))
