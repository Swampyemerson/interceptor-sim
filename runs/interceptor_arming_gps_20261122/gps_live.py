"""Read-only: listen to the Pixhawk through the Pi's bridge and print GPS state once a second."""
import os, sys, time
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
m = mavutil.mavlink_connection("udpout:192.168.0.61:14550", source_system=254, source_component=190)
t0 = time.time(); last_hb = 0; last_print = 0; hb = None; texts = []
while time.time() - t0 < float(sys.argv[1]):
    if time.time() - last_hb > 1:
        m.mav.heartbeat_send(6, 8, 0, 0, 0); last_hb = time.time()
    msg = m.recv_match(type=["GPS_RAW_INT", "HEARTBEAT", "STATUSTEXT", "SYS_STATUS"], blocking=True, timeout=0.5)
    if msg is None: continue
    t = msg.get_type()
    if t == "HEARTBEAT" and msg.get_srcComponent() == 1: hb = msg
    elif t == "STATUSTEXT" and msg.text not in texts: texts.append(msg.text); print("   TEXT:", msg.text, flush=True)
    elif t == "GPS_RAW_INT" and time.time() - last_print >= 2:
        last_print = time.time()
        print("%s  fix %d  sats %d  eph %s  armed %s  mode %s" % (time.strftime("%H:%M:%S"), msg.fix_type, msg.satellites_visible, msg.eph,
              None if hb is None else bool(hb.base_mode & 128), None if hb is None else hb.custom_mode >> 16), flush=True)
