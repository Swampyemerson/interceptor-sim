#!/usr/bin/env python3
"""Read-only bench status snapshot from an ARDUPILOT FC over a Windows COM port.

Prints: firmware heartbeat, armed state + mode, battery monitor voltage/current,
sensor-health bitmask summary, RC link (channel count + values), and prearm text
messages seen during the listen window. Writes NOTHING to the board.

ArduPilot only (plain-float params; see upload_params.py header for the PX4
int-encoding trap -- this tool never touches params anyway).

    python.exe scripts/bench/fc_status_read.py --port COM7
"""
from __future__ import annotations
import argparse, time

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--listen-s", type=float, default=6.0)
    args = ap.parse_args()

    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.port, baud=args.baud)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("FAIL: no heartbeat in 10 s")
        return 2
    print(f"heartbeat: sys {m.target_system} comp {m.target_component} "
          f"type {hb.type} autopilot {hb.autopilot}")
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"armed: {armed}  custom_mode: {hb.custom_mode}")

    # Ask for the standard streams for the listen window.
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    want = {"SYS_STATUS": None, "BATTERY_STATUS": None, "RC_CHANNELS": None,
            "GPS_RAW_INT": None}
    texts = []
    t0 = time.time()
    while time.time() - t0 < args.listen_s:
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()
        if t in want and want[t] is None:
            want[t] = msg
        elif t == "STATUSTEXT":
            texts.append(msg.text)
        if all(v is not None for v in want.values()):
            break

    s = want["SYS_STATUS"]
    if s:
        v = s.voltage_battery / 1000.0
        i = s.current_battery / 100.0 if s.current_battery != -1 else float("nan")
        print(f"SYS_STATUS: batt {v:.2f} V  current {i:.2f} A  load {s.load/10.0:.0f}%")
        bad = s.onboard_control_sensors_enabled & ~s.onboard_control_sensors_health
        print(f"sensors: enabled=0x{s.onboard_control_sensors_enabled:08x} "
              f"unhealthy-bits=0x{bad:08x}" + ("  (all enabled sensors healthy)" if bad == 0 else ""))
    else:
        print("SYS_STATUS: none received")
    b = want["BATTERY_STATUS"]
    if b:
        cells = [c for c in b.voltages if 0 < c < 65535]
        print(f"BATTERY_STATUS: {len(cells)} cell-field(s), pack "
              f"{sum(cells)/1000.0:.2f} V" if cells else "BATTERY_STATUS: no cell voltages")
    r = want["RC_CHANNELS"]
    if r:
        ch = [getattr(r, f"chan{k}_raw") for k in range(1, 9)]
        print(f"RC_CHANNELS: count {r.chancount}  ch1-8 {ch}  rssi {r.rssi}")
    else:
        print("RC_CHANNELS: none received (no RC link yet is normal pre-bind)")
    g = want["GPS_RAW_INT"]
    if g:
        print(f"GPS: fix_type {g.fix_type}  sats {g.satellites_visible}")
    for t in texts[:8]:
        print(f"STATUSTEXT: {t}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
