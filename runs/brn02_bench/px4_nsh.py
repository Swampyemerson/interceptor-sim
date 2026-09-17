"""Run read-only PX4 shell commands over MAVLink SERIAL_CONTROL. Windows python."""
import sys, time
from pymavlink import mavutil
port, cmds = sys.argv[1], sys.argv[2:]
m = mavutil.mavlink_connection(port, baud=115200); m.wait_heartbeat(timeout=8)
SH = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
FL = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
def send(s):
    b = [ord(c) for c in s]
    while b:
        n = b[:70]; b = b[70:]
        m.mav.serial_control_send(SH, FL, 0, 0, len(n), n + [0]*(70-len(n)))
def read(t):
    out = ""; t0 = time.time()
    while time.time()-t0 < t:
        x = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.3)
        if x and x.count: out += "".join(chr(c) for c in x.data[:x.count])
    return out
send("\n"); read(1)
for c in cmds:
    send(c + "\n"); print("$", c); print(read(3.5))
