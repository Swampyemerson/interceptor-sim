"""Compass and GPS noise figures before / during / after a camera recording (windows in log seconds)."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = u.start_timestamp
on, off = float(sys.argv[2]), float(sys.argv[3])
wins = (("camera off, before", on - 24, on - 1), ("camera RECORDING", on + 3, off - 1), ("camera off, after", off + 3, off + 24))
names = sorted({(d.name, d.multi_id) for d in u.data_list if d.name in ("sensor_mag", "vehicle_magnetometer", "sensor_gps")})
print("topics:", names, " log span %.0f s" % ((u.last_timestamp - t0) / 1e6))
for d in u.data_list:
    if d.name == "sensor_mag":
        t = (d.data["timestamp"] - t0) / 1e6
        v = np.c_[d.data["x"], d.data["y"], d.data["z"]]
        base = None
        for nm, a, b in wins:
            w = (t > a) & (t < b)
            if not w.any(): print("mag", d.multi_id, nm, "no data"); continue
            m = v[w].mean(0); sd = v[w].std(0)
            if base is None: base = m
            print("mag %d dev %s  %-20s n=%4d  mean %+.4f %+.4f %+.4f G  |B| %.4f  sd %.4f  shift from before %.4f G" % (
                d.multi_id, d.data["device_id"][0], nm, w.sum(), m[0], m[1], m[2], np.linalg.norm(m), np.linalg.norm(sd), np.linalg.norm(m - base)))
    if d.name == "sensor_gps":
        g = d.data; t = (g["timestamp"] - t0) / 1e6
        for nm, a, b in wins:
            w = (t > a) & (t < b)
            if not w.any(): print("gps", nm, "no data"); continue
            print("gps  %-20s n=%3d  sats %d..%d  jam median %3d (%d..%d)  noise %3d  fix %s" % (nm, w.sum(), g["satellites_used"][w].min(), g["satellites_used"][w].max(),
                  np.median(g["jamming_indicator"][w]), g["jamming_indicator"][w].min(), g["jamming_indicator"][w].max(), np.median(g["noise_per_ms"][w]), sorted(set(g["fix_type"][w].tolist()))))
