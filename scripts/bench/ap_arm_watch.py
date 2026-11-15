"""Listen-only record of an ArduPilot arm attempt: board messages, armed state, mode, radio channels,
health bits and GPS, each printed when it changes. Never arms and never writes a setting.

    python.exe ap_arm_watch.py COM4 200 out.txt     (run with Windows Python from C:\\Temp\\bench)
"""
import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], baud=115200); assert m.wait_heartbeat(timeout=20)
dur = float(sys.argv[2]); out = open(sys.argv[3], "w", encoding="utf-8")
def say(*a):
    line = time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a); print(line, flush=True); out.write(line + "\n"); out.flush()
for mid, us in ((1, 500000), (65, 100000), (24, 1000000)):
    m.mav.command_long_send(m.target_system, m.target_component, 511, 0, mid, us, 0,0,0,0,0)
MODES = {0: "Stabilize", 2: "AltHold", 3: "Auto", 5: "Loiter", 6: "RTL", 9: "Land"}
t0 = time.time(); last = {}
say("listening only; this script never arms")
while time.time() - t0 < dur:
    x = m.recv_match(type=["STATUSTEXT", "SYS_STATUS", "HEARTBEAT", "RC_CHANNELS", "GPS_RAW_INT"], blocking=True, timeout=1)
    if x is None: continue
    t = x.get_type()
    if t == "STATUSTEXT": say("BOARD SAYS:", x.text)
    elif t == "HEARTBEAT" and x.get_srcComponent() == 1:
        v = ("ARMED" if x.base_mode & 128 else "disarmed", MODES.get(x.custom_mode, x.custom_mode))
        if last.get("hb") != v: last["hb"] = v; say("state:", *v)
    elif t == "RC_CHANNELS":
        v = (round(x.chan5_raw, -2), round(x.chan6_raw, -2), round(x.chan3_raw / 50) * 50, round(x.chan4_raw / 50) * 50, round(x.chan1_raw / 50) * 50, round(x.chan2_raw / 50) * 50)
        if last.get("rc") != v: last["rc"] = v; say("radio: arm-switch ch5", v[0], "| mode ch6", v[1], "| throttle", v[2], "| yaw", v[3], "| roll", v[4], "| pitch", v[5], "| rssi", x.rssi)
    elif t == "SYS_STATUS":
        bad = x.onboard_control_sensors_enabled & ~x.onboard_control_sensors_health; pre = bool(x.onboard_control_sensors_health & 0x10000000)
        v = (hex(bad), pre)
        if last.get("ss") != v: last["ss"] = v; say("health: unhealthy bits", v[0], "| pre-arm ok", pre, "| battery V", x.voltage_battery / 1000.0)
    elif t == "GPS_RAW_INT":
        v = (x.fix_type, x.satellites_visible // 4)
        if last.get("gps") != v: last["gps"] = v; say("gps: fix", x.fix_type, "sats", x.satellites_visible)
say("done")
