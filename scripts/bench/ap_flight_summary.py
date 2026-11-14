"""Summarise an ArduPilot .BIN flight log around a climb-away: mode, throttle stick,
what the height controller asked for, what the motors were sent, vibration, messages.
Prints no positions."""
import sys, collections
from pymavlink import mavutil
path = sys.argv[1]; step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
m = mavutil.mavlink_connection(path)
want = ["CTUN", "RCIN", "RCOU", "VIBE", "MODE", "EV", "ERR", "MSG", "BAT", "ARM", "ATT", "PARM", "BARO", "RATE", "MOTB", "IMU"]
rows = collections.defaultdict(list); parms = {}
while True:
    x = m.recv_match(type=want)
    if x is None: break
    t = x.get_type()
    if t == "PARM": parms[x.Name] = x.Value; continue
    rows[t].append(x)
print("messages:", {k: len(v) for k, v in rows.items()})
for k in ("MOT_SPIN_MIN", "MOT_SPIN_ARM", "MOT_THST_HOVER", "MOT_THST_EXPO", "FLTMODE6", "FLTMODE4", "FLTMODE1", "RC5_OPTION", "AHRS_ORIENTATION", "INS_GYRO_FILTER", "PILOT_THR_BHV", "MOT_PWM_TYPE", "MOT_PWM_MIN", "MOT_PWM_MAX"):
    if k in parms: print(f"  {k} = {parms[k]}")
for t in ("MODE", "ARM", "EV", "ERR"):
    for x in rows[t]:
        d = x.to_dict(); d.pop("mavpackettype", None); print(t, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in d.items()})
for x in rows["MSG"]: print("MSG %.1f %s" % (x.TimeUS / 1e6, x.Message))
arm = [x for x in rows["ARM"] if getattr(x, "ArmState", 0)]
t0 = arm[-1].TimeUS / 1e6 - 1 if arm else (rows["CTUN"][0].TimeUS / 1e6 if rows["CTUN"] else 0)
def near(lst, t, i=[0]):
    best = None
    for x in lst:
        if x.TimeUS / 1e6 <= t: best = x
        else: break
    return best
print("\n   t   thrStick  ThI   ThO   ThH   DAlt    Alt   BAlt   CRt  DCRt | motors(us)            | vibZ clip | roll pitch | V     A")
if rows["CTUN"]:
    tend = rows["CTUN"][-1].TimeUS / 1e6; t = t0
    while t <= tend + step:
        c = near(rows["CTUN"], t); r = near(rows["RCIN"], t); o = near(rows["RCOU"], t); v = near(rows["VIBE"], t); a = near(rows["ATT"], t); b = near(rows["BAT"], t)
        if c is not None:
            g = lambda obj, n, f="%5.2f": (f % getattr(obj, n)) if obj is not None and hasattr(obj, n) else "  -  "
            clip = (getattr(v, "Clip", None) if v is not None else None)
            print("%6.1f  %5s   %s %s %s %s %s %s %s %s | %s %s %s %s | %s %s | %s %s | %s %s" % (
                t - t0, g(r, "C3", "%d"), g(c, "ThI"), g(c, "ThO"), g(c, "ThH"), g(c, "DAlt", "%6.1f"), g(c, "Alt", "%6.1f"), g(c, "BAlt", "%6.1f"),
                g(c, "CRt", "%5.0f" if not hasattr(c, "CRt") or abs(c.CRt) > 50 else "%5.1f"), g(c, "DCRt", "%5.1f"),
                g(o, "C1", "%d"), g(o, "C2", "%d"), g(o, "C3", "%d"), g(o, "C4", "%d"),
                g(v, "VibeZ", "%5.1f"), clip if clip is not None else "-", g(a, "Roll", "%5.1f"), g(a, "Pitch", "%5.1f"), g(b, "Volt", "%5.2f"), g(b, "Curr", "%5.1f")))
        t += step
