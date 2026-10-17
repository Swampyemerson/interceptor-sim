#!/usr/bin/env python3
"""Props-off motor mapping sequence: announce ourselves with GCS heartbeats
(PX4 only routes targeted replies to systems it has seen), then spin motors
1..4 one at a time, printing every ACK/STATUSTEXT."""
import time

from pymavlink import mavutil

CMD_ACTUATOR_TEST = 310
FUNC_MOTOR1 = 1  # MAVLink ACTUATOR_OUTPUT_FUNCTION_MOTOR1 -- NOT PX4's internal 101!
# (param5 takes the MAVLink enum: MOTOR1..4 = 1..4. Sending PX4's internal
#  output-function numbers (101..104) is silently dropped with NO ack -- that
#  mistake cost the 2026-10-16 bench two hours. QGC sends 1..4.)
VALUE = 0.15
SECS = 3.0
GAP = 6.0

m = mavutil.mavlink_connection("/dev/serial0", baud=921600,
                               source_system=255, source_component=190)
if m.wait_heartbeat(timeout=15) is None:
    print("FAIL: no heartbeat")
    raise SystemExit(2)
print(f"FC heartbeat seen (sys {m.target_system}); announcing as GCS...")


def beat():
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


for _ in range(3):
    beat()
    time.sleep(0.4)

for motor in (1, 2, 3, 4):
    print(f"=== MOTOR {motor}: spinning {SECS:.0f}s at {VALUE:.0%} ===", flush=True)
    beat()
    m.mav.command_long_send(m.target_system, 1, CMD_ACTUATOR_TEST, 0,
                            VALUE, SECS, 0, 0, FUNC_MOTOR1 + motor - 1, 0, 0)
    t0 = time.time()
    while time.time() - t0 < SECS + 2:
        msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"],
                           blocking=True, timeout=1.0)
        if msg is None:
            beat()
            continue
        if msg.get_type() == "COMMAND_ACK" and msg.command == CMD_ACTUATOR_TEST:
            res = mavutil.mavlink.enums["MAV_RESULT"][msg.result].name
            print(f"  ACK: {res}", flush=True)
        elif msg.get_type() == "STATUSTEXT":
            print(f"  FC says: {msg.text}", flush=True)
    time.sleep(GAP)
print("sequence done")
