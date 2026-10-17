#!/usr/bin/env python3
"""Set the interceptor FC's CBRK_IO_SAFETY posture over MAVLink, with read-back
(FC param writes need a read-back), then reboot the FC -- Safety.cpp reads the
param only at commander start, so a reboot is required each way.

Values: 0 = safety button REQUIRED (the bench's fail-closed default posture,
configs/px4_6cmini/bench.params section 2) / 22027 = safety BYPASSED (the
props-off motor-test exception the README sanctions "knowingly" -- restore 0
after the session). BUILDER-AUTHORIZED USE ONLY: flipping to 22027 is the
builder's call, never an agent's (2026-10-16 bench; the harness refused it,
correctly).

    .venv-pi/bin/python scripts/bench/fc_set_cbrk_safety.py 22027   # bypass
    .venv-pi/bin/python scripts/bench/fc_set_cbrk_safety.py 0       # restore
"""
import struct
import sys
import time

from pymavlink import mavutil

NAME = b"CBRK_IO_SAFETY"
ALLOWED = (0, 22027)


def main() -> int:
    if len(sys.argv) != 2 or int(sys.argv[1]) not in ALLOWED:
        print(f"usage: {sys.argv[0]} <0|22027>")
        return 2
    target = int(sys.argv[1])
    m = mavutil.mavlink_connection("/dev/serial0", baud=921600)
    if m.wait_heartbeat(timeout=10) is None:
        print("FAIL: no heartbeat")
        return 2

    # PX4 carries INT32 params as raw bytes in the float field
    val_as_float = struct.unpack("f", struct.pack("i", target))[0]
    m.mav.param_set_send(m.target_system, 1, NAME, val_as_float,
                         mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    ok = False
    t0 = time.time()
    while time.time() - t0 < 5:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg and msg.param_id == NAME.decode():
            got = struct.unpack("i", struct.pack("f", msg.param_value))[0]
            print(f"read-back: CBRK_IO_SAFETY = {got}")
            ok = got == target
            break
    if not ok:
        print("FAIL: read-back did not confirm the write")
        return 1

    print("rebooting FC (param is read only at commander start)...")
    m.mav.command_long_send(m.target_system, 1,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                            0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1)
    print("reboot sent -- give it ~25 s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
