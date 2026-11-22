"""Stand-in ground station: connect to the Pi's bridge and print the recorder's own messages."""
import os, sys, time
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
m = mavutil.mavlink_connection("udpout:192.168.0.61:14550", source_system=254, source_component=190)
t0 = time.time(); last = 0
while time.time() - t0 < float(sys.argv[1]):
    if time.time() - last > 1:
        m.mav.heartbeat_send(6, 8, 0, 0, 0); last = time.time()
    msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
    if msg is not None and msg.get_srcComponent() == 155:
        print(time.strftime("%H:%M:%S"), "sev", msg.severity, repr(msg.text), flush=True)
