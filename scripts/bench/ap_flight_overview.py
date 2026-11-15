"""Flight overview from an ArduPilot .BIN: arm/disarm, modes, messages, errors, events, mission progress,
battery, height tracking, speed, attitude and rate tracking. Prints no positions."""
import sys, collections, math
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1])
want = ["MODE", "ARM", "EV", "ERR", "MSG", "BAT", "CTUN", "ATT", "RATE", "CMD", "NTUN", "PSCN", "PSCE", "PSCD", "GPS", "VIBE", "XKF4", "RCIN", "RCOU", "PARM", "MAG", "POS"]
rows = collections.defaultdict(list); parms = {}; n = 0; t_first = t_last = None; types = collections.Counter()
while True:
    x = m.recv_match()
    if x is None: break
    t = x.get_type(); types[t] += 1; n += 1
    ts = getattr(x, "TimeUS", None)
    if ts:
        if t_first is None: t_first = ts
        t_last = ts
    if t == "PARM": parms[x.Name] = x.Value
    elif t in want: rows[t].append(x)
print("messages parsed", n, "| span s", round((t_last - t_first) / 1e6, 1))
print("types:", dict(types.most_common(45)))
T = lambda x: round(x.TimeUS / 1e6, 1)
for k in ("WP_SPD", "WP_ACC", "WP_RADIUS_M", "LOG_BITMASK", "ATC_RAT_RLL_P", "ATC_RAT_PIT_P", "ATC_RAT_RLL_D", "ATC_RAT_PIT_D", "PSC_D_ACC_P", "PSC_D_ACC_I", "MOT_THST_HOVER", "MOT_SPIN_MIN", "AHRS_TRIM_X", "AHRS_TRIM_Y", "BATT_LOW_VOLT", "FENCE_RADIUS", "FENCE_ALT_MAX"):
    if k in parms: print("  ", k, "=", round(parms[k], 5))
for t in ("ARM", "MODE", "EV", "ERR"):
    for x in rows[t]:
        d = x.to_dict(); d.pop("mavpackettype", None); ts = d.pop("TimeUS") / 1e6
        print(t, round(ts, 1), {k: (round(v, 2) if isinstance(v, float) else v) for k, v in d.items()})
for x in rows["MSG"]: print("MSG", T(x), x.Message)
seen = None
for x in rows["CMD"]:
    key = (x.CNum, x.CId)
    if key != seen: seen = key; print("CMD", T(x), "item", x.CNum, "of", x.CTot, "cmd", x.CId, "p1", round(x.Prm1, 1), "p2", round(x.Prm2, 1))
b = rows["BAT"]
if b: print("battery V: start", round(b[0].Volt, 2), "min", round(min(x.Volt for x in b), 2), "end", round(b[-1].Volt, 2), "| mAh", round(b[-1].CurrTot, 0), "| max A", round(max(x.Curr for x in b), 1))
