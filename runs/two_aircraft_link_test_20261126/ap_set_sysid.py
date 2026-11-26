"""Set the target's MAVLink system ID (MAV_SYSID, or SYSID_THISMAV on older firmware) and read it back. Never arms."""
import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], baud=115200); assert m.wait_heartbeat(timeout=20)
want = float(sys.argv[2])
def read(name):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 3:
        x = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if x and x.param_id == name: return x
    return None
print("heartbeat from system", m.target_system, "armed" if m.motors_armed() else "disarmed")
assert not m.motors_armed()
for name in ("MAV_SYSID", "SYSID_THISMAV"):
    x = read(name)
    if x is None: print(name, "not on this board"); continue
    print(name, "before:", x.param_value)
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), want, x.param_type)
    time.sleep(1.0)
    y = read(name) 
    print(name, "after (fresh read):", None if y is None else y.param_value)
    break
