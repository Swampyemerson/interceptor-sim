import sys, numpy as np
from pyulog import ULog
for p in sys.argv[1:]:
    try:
        u = ULog(p, ["sensor_gps", "vehicle_status"])
        g = u.get_dataset("sensor_gps").data
    except Exception as e:
        print(p.split("/")[-1], "ERR", e); continue
    keys = [k for k in ("jamming_indicator", "noise_per_ms", "jamming_state", "spoofing_state", "satellites_used", "fix_type", "automatic_gain_control") if k in g]
    print(p.split("/")[-1], "| %.0f s |" % ((u.last_timestamp - u.start_timestamp) / 1e6),
          " ".join("%s %s..%s (median %s)" % (k, g[k].min(), g[k].max(), int(np.median(g[k]))) for k in keys))
