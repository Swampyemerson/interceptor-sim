"""The target's control-link position reports while it sat still (2026-11-26,
runs 3 and 4 of the ground rehearsal), decoded from the raw bytes the laptop
recorded, set beside the interceptor's own GPS receiver over the same minutes.

    .venv/bin/python target_reports_still.py RAW.bin INTERCEPTOR_GROUND.ulg

The raw file has no timestamps: reports are taken as evenly spaced (the route
sends 2 a second), so only spreads and medians are reported, not a time match.
"""
import math, sys
import numpy as np
sys.path.insert(0, "scripts/field")
import crsf_cue_source as ccs
from pyulog import ULog

R = 6378137.0
raw = open(sys.argv[1], "rb").read()
p = ccs.CrsfParser()
fixes = []
for f in p.feed(raw, 0.0):
    if f.type == ccs.FRAME_TYPE_GPS:
        try:
            fixes.append(ccs.decode_gps(f.payload))
        except Exception:
            pass
N_USE = int(sys.argv[3]) if len(sys.argv) > 3 else len(fixes)   # first N reports only
fixes = fixes[:N_USE]
lat = np.array([g.lat for g in fixes]); lon = np.array([g.lon for g in fixes])
alt = np.array([g.alt_m_msl for g in fixes]); spd = np.array([g.speed_ms for g in fixes])
crs = np.array([g.course_deg for g in fixes]); sats = np.array([g.sats for g in fixes])
lat0, lon0 = np.median(lat), np.median(lon)
n = np.radians(lat - lat0) * R; e = np.radians(lon - lon0) * R * math.cos(math.radians(lat0))
print(f"target reports: {len(fixes)} (about {len(fixes) / 2 / 60:.1f} min at 2 a second), satellites {np.median(sats):.0f}")
print(f"  position: RMS {math.sqrt(np.var(n) + np.var(e)):.2f} m, span N {np.ptp(n):.2f} m, E {np.ptp(e):.2f} m")
w = 120      # reports in one minute
worst = max(float(np.hypot(np.ptp(n[i:i + w]), np.ptp(e[i:i + w]))) for i in range(0, max(1, len(n) - w), 20))
print(f"  worst movement inside any one minute: {worst:.2f} m")
print(f"  height (whole metres): median {np.median(alt):.0f} m, values {sorted(set(alt.astype(int)))}")
print(f"  reported speed while still: median {np.median(spd):.2f}, 95th pct {np.percentile(spd, 95):.2f}, max {spd.max():.2f} m/s")
print(f"  reported course while still: spread over {np.ptp(crs):.0f} deg (meaningless when not moving)")
# drift rate that a 5 s fit would see: slope over each 5.5 s window (11 reports)
sl = []
for i in range(0, len(n) - 11):
    tt = np.arange(11) * 0.5
    sl.append(math.hypot(np.polyfit(tt, n[i:i + 11], 1)[0], np.polyfit(tt, e[i:i + 11], 1)[0]))
sl = np.array(sl)
print(f"  speed a 5.5 s position fit would report for this still target: median {np.median(sl):.3f}, 95th pct {np.percentile(sl, 95):.3f}, max {sl.max():.3f} m/s")
u = ULog(sys.argv[2], ["sensor_gps"]); g = u.data_list[0].data
ilat, ilon, ialt = np.median(g["latitude_deg"]), np.median(g["longitude_deg"]), np.median(g["altitude_msl_m"])
dn = math.radians(lat0 - ilat) * R; de = math.radians(lon0 - ilon) * R * math.cos(math.radians(ilat))
print(f"interceptor GPS receiver (median over {sys.argv[2].split('/')[-1]}): target is {dn:+.1f} m north, {de:+.1f} m east, "
      f"{math.hypot(dn, de):.1f} m away, {np.median(alt) - ialt:+.1f} m higher by the two receivers' heights")
