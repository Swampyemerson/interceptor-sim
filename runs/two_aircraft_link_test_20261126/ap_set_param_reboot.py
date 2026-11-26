"""Set one ArduPilot parameter, read it back, then restart the board (refuses if armed)."""
import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], baud=115200); assert m.wait_heartbeat(timeout=20)
name, want = sys.argv[2], float(sys.argv[3])
def read(n):
    m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 3:
        x = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if x and x.param_id == n: return x
print("system", m.target_system); assert not m.motors_armed()
x = read(name); print(name, "before:", x.param_value)
m.mav.param_set_send(m.target_system, m.target_component, name.encode(), want, x.param_type)
time.sleep(1); y = read(name); print(name, "after (fresh read):", y.param_value)
assert y.param_value == want
print("MAV_SYSID:", read("MAV_SYSID").param_value)
if len(sys.argv) > 4 and sys.argv[4] == "reboot":
    m.mav.command_long_send(m.target_system, m.target_component, 246, 0, 1, 0, 0, 0, 0, 0, 0); print("restart sent")
