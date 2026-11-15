"""Clock times (UTC, from the log's GPS week and milliseconds) of arm, disarm and chosen log seconds in an
ArduPilot .BIN, for lining a flight log up with a video. The two event times are edited in the last lines.
Prints no positions.
"""
import sys, datetime
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1]); g = None; first = None; arm = []; ct = []
while True:
    x = m.recv_match(type=["GPS", "ARM", "CTUN", "MODE", "BAT"])
    if x is None: break
    t = x.get_type()
    if t == "GPS" and x.Status >= 3 and g is None and x.GWk > 0: g = x
    elif t == "ARM": arm.append(x)
    elif t == "CTUN": ct.append(x)
epoch = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc) + datetime.timedelta(weeks=g.GWk, milliseconds=g.GMS) - datetime.timedelta(seconds=18)
def utc(ts): return (epoch + datetime.timedelta(seconds=ts - g.TimeUS / 1e6))
for a in arm: print("ARM state", a.ArmState, "log t", round(a.TimeUS / 1e6, 1), "UTC", utc(a.TimeUS / 1e6).strftime("%H:%M:%S.%f")[:-4])
air = [x for x in ct if x.Alt > 0.5]
print("first airborne log t", round(air[0].TimeUS / 1e6, 1), "UTC", utc(air[0].TimeUS / 1e6).strftime("%H:%M:%S"))
print("failsafe log t 546.77 UTC", utc(546.77).strftime("%H:%M:%S.%f")[:-4], "| kill 549.87 UTC", utc(549.87).strftime("%H:%M:%S.%f")[:-4])
print("unix of kill", utc(549.87).timestamp())
