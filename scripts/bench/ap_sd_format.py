"""Ask an ArduPilot board to format its microSD card (MAV_CMD_STORAGE_FORMAT 526, param1=1 param2=1;
ArduPilot GCS_Common.cpp handle_command_storage_format). ERASES EVERY LOG ON THE CARD. Refuses if armed."""
import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], baud=115200); assert m.wait_heartbeat(timeout=20)
hb = None
for _ in range(10):
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb is not None and hb.get_srcComponent() == 1: break
if hb is None or hb.base_mode & 128: raise SystemExit("FAIL: no heartbeat or board is ARMED; not formatting")
m.mav.command_long_send(m.target_system, m.target_component, 526, 0, 1, 1, 0, 0, 0, 0, 0)
RES = {0: "ACCEPTED (done)", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
t0 = time.time(); final = None
while time.time() - t0 < 120:
    x = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
    if x is None: continue
    if x.get_type() == "STATUSTEXT": print(round(time.time() - t0, 1), "board says:", x.text, flush=True); continue
    if x.command != 526: continue
    print(round(time.time() - t0, 1), "format ack:", RES.get(x.result, x.result), flush=True)
    if x.result != 5: final = x.result; break
print("RESULT:", "formatted" if final == 0 else f"NOT formatted ({RES.get(final, final)})")
sys.exit(0 if final == 0 else 1)
