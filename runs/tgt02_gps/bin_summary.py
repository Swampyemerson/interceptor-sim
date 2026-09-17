import sys, json, collections
from pymavlink import mavutil
p = sys.argv[1]
m = mavutil.mavlink_connection(p)
c = collections.Counter(); gps = []; pos = []; msgs = []
while True:
    x = m.recv_match(blocking=False)
    if x is None: break
    t = x.get_type(); c[t] += 1
    if t == "GPS" and getattr(x, "I", 0) in (0, None): gps.append((x.Status, x.NSats, x.HDop, x.Lat, x.Lng, x.Alt))
    elif t == "POS": pos.append((x.Lat, x.Lng, x.Alt))
    elif t == "MSG": msgs.append(x.Message)
out = {"file": p, "total": sum(c.values()), "types": len(c), "has_GPS": c["GPS"], "has_POS": c["POS"], "has_MAG": c["MAG"],
       "gps_status_max": max((g[0] for g in gps), default=None), "gps_sats_max": max((g[1] for g in gps), default=None),
       "gps_hdop_min": min((g[2] for g in gps), default=None),
       "gps_3dfix_rows": sum(1 for g in gps if g[0] >= 3),
       "pos_rows_nonzero": sum(1 for q in pos if abs(q[0]) > 1e-6),
       "pos_lat_span_m": (max(q[0] for q in pos) - min(q[0] for q in pos)) * 111320 if pos else None,
       "banner": [s for s in msgs if "PreArm" not in s][:6], "prearm": sorted({s for s in msgs if "PreArm" in s})}
print(json.dumps(out, indent=1))
