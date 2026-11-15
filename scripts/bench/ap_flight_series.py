"""Time series from an ArduPilot .BIN between two log times: sticks, switches, throttle out, height above
home, climb rate, lean, motor outputs, battery, vertical acceleration. Prints no positions.

    python.exe ap_flight_series.py log.bin 602 614 0.25
"""
import sys, collections
from pymavlink import mavutil
a, b, step = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
m = mavutil.mavlink_connection(sys.argv[1])
want = ["CTUN", "RCIN", "RCOU", "ATT", "BAT", "MODE", "EV", "ARM", "IMU", "ERR", "MSG"]
rows = collections.defaultdict(list)
while True:
    x = m.recv_match(type=want)
    if x is None: break
    t = x.TimeUS / 1e6
    if a - 1 <= t <= b + 1: rows[x.get_type()].append(x)
def near(k, t):
    best = None
    for x in rows[k]:
        if x.TimeUS / 1e6 <= t: best = x
        else: break
    return best
for k in ("MODE", "EV", "ARM", "ERR", "MSG"):
    for x in rows[k]:
        d = x.to_dict(); d.pop("mavpackettype"); ts = d.pop("TimeUS") / 1e6
        if a <= ts <= b: print(k, round(ts, 2), d)
print("   t   | thr-stick ch5  ch6 | thrOut | height  want  climb | roll pitch | motors max/min |  V     A  | accZ")
t = a
while t <= b:
    c, r, o, at, bt, im = (near(k, t) for k in ("CTUN", "RCIN", "RCOU", "ATT", "BAT", "IMU"))
    if c and r and o and at and bt:
        mo = (o.C1, o.C2, o.C3, o.C4)
        print(f"{t:7.1f} | {r.C3:5d} {r.C5:5d} {r.C6:5d} | {c.ThO:5.3f} | {c.Alt:6.2f} {c.DAlt:6.2f} {c.CRt:6.2f} | {at.Roll:5.1f} {at.Pitch:5.1f} | {max(mo):5d} {min(mo):5d} | {bt.Volt:5.2f} {bt.Curr:5.1f} | {im.AccZ if im else 0:6.1f}")
    t += step
