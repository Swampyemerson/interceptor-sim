"""Does the card accept a new log? Turns on logging-while-disarmed for ~25 s, watches the board,
lists the logs, then ALWAYS puts LOG_DISARMED back to 0 (read back). Never arms."""
import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], baud=115200); assert m.wait_heartbeat(timeout=20)
def setp(name, val):
    for _ in range(3):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val), 9)
        t = time.time() + 4
        while time.time() < t:
            x = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if x and x.param_id.strip("\x00").strip() == name: print(name, "read back =", x.param_value); return x.param_value
    print("!!", name, "NOT confirmed"); return None
def logs():
    m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xffff)
    d = {}; end = time.time() + 15
    while time.time() < end:
        e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=2)
        if e is None: continue
        d[e.id] = e.size
        if e.num_logs and len(d) >= e.num_logs: break
    return d
before = logs(); print("before: count", len(before), "newest", max(before), before[max(before)], "bytes | sizes", {k: before[k] for k in sorted(before)})
m.mav.command_long_send(m.target_system, m.target_component, 511, 0, 1, 500000, 0,0,0,0,0)
try:
    setp("LOG_DISARMED", 1)
    t0 = time.time(); bad = 0; n = 0; seen = []
    while time.time() - t0 < 25:
        x = m.recv_match(type=["STATUSTEXT", "SYS_STATUS"], blocking=True, timeout=1)
        if x is None: continue
        if x.get_type() == "STATUSTEXT":
            if x.text not in seen: seen.append(x.text); print(round(time.time() - t0, 1), "board says:", x.text)
        else:
            n += 1; bad += 0 if x.onboard_control_sensors_health & 0x1000000 else 1
    print("logging health: unhealthy in", bad, "of", n, "samples")
    after = logs(); new = {k: v for k, v in after.items() if before.get(k) != v}
    print("after: count", len(after), "| changed or new logs:", new)
finally:
    setp("LOG_DISARMED", 0)
