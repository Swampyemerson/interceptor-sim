"""Read-only: list the target autopilot's onboard logs. Sends only a heartbeat and LOG_REQUEST_LIST."""
import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], baud=115200)
hb = m.wait_heartbeat(timeout=12)
if hb is None: print("no heartbeat"); sys.exit(1)
print("system", m.target_system, "autopilot", hb.autopilot, "armed" if m.motors_armed() else "disarmed")
m.mav.log_request_list_send(m.target_system, m.target_component, 0, 0xFFFF)
t0 = time.time(); seen = {}
while time.time() - t0 < 8:
    x = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=1)
    if x:
        seen[x.id] = (x.size, x.time_utc); t0 = time.time() if len(seen) < x.num_logs else t0 - 6
for k in sorted(seen): print("log", k, "bytes", seen[k][0], "utc", time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(seen[k][1])) if seen[k][1] else "no time")
print("n", len(seen))
