#!/usr/bin/env python3
"""Hover recorder: log attitude, body rates and motor outputs from a PX4 FC to CSV.

WHY: the FC's microSD slot is dead, so every wobble verdict so far was by eye.
This asks PX4 for three plain MAVLink streams at a fixed rate and writes them to
a file on whatever computer is on the link (the Pi over TELEM2 in flight, or a
PC over USB on the bench). It is NOT the parked ULog-streaming tool
(pi_ulog_stream_catcher.py) -- no log-streaming protocol, just telemetry.

READ-ONLY toward flight behaviour: it sends heartbeats and
MAV_CMD_SET_MESSAGE_INTERVAL (stream rates on THIS link only). No params, no
arming, no setpoints.

    Pi (flight):  .venv-pi/bin/python scripts/bench/hover_recorder.py --port /dev/ttyAMA10 --baud 921600
    PC (bench):   python.exe hover_recorder.py --port COM5 --seconds 10
    stop:         touch /tmp/hover_recorder.stop      (or Ctrl-C, or --seconds)

Analyse with scripts/bench/hover_wobble_analysis.py.
"""
from __future__ import annotations

import argparse
import csv
import os
import time

STOP_FLAG = "/tmp/hover_recorder.stop"
# message id -> name
STREAMS = {30: "ATTITUDE", 36: "SERVO_OUTPUT_RAW", 1: "SYS_STATUS"}
FIELDS = ["t_boot_ms", "msg", "armed", "roll", "pitch", "yaw",
          "p", "q", "r", "o1", "o2", "o3", "o4", "vbat"]


def write_header(w) -> None:
    w.writerow(FIELDS)


def att_row(t_ms, armed, roll, pitch, yaw, p, q, r):
    return [t_ms, "ATT", int(armed), f"{roll:.5f}", f"{pitch:.5f}", f"{yaw:.5f}",
            f"{p:.5f}", f"{q:.5f}", f"{r:.5f}", "", "", "", "", ""]


def out_row(t_ms, armed, o, vbat=""):
    return [t_ms, "OUT", int(armed), "", "", "", "", "", "",
            o[0], o[1], o[2], o[3], vbat]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate-hz", type=float, default=100.0)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--out-dir", default=os.path.expanduser("~/hover_logs"))
    args = ap.parse_args()

    from pymavlink import mavutil
    os.makedirs(args.out_dir, exist_ok=True)
    if os.path.exists(STOP_FLAG):
        os.remove(STOP_FLAG)
    path = os.path.join(args.out_dir,
                        time.strftime("hover_%Y%m%dT%H%M%SZ.csv", time.gmtime()))

    m = mavutil.mavlink_connection(args.port, baud=args.baud,
                                   source_system=255, source_component=190)

    def beat():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    for _ in range(4):
        beat()
        time.sleep(0.3)
    if m.wait_heartbeat(timeout=10) is None:
        print("FAIL: no heartbeat", flush=True)
        return 2
    for mid, rate in ((30, args.rate_hz), (36, args.rate_hz / 2), (1, 2.0)):
        m.mav.command_long_send(m.target_system, 1,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, mid, 1e6 / rate, 0, 0, 0, 0, 0)
    print(f"recording -> {path}", flush=True)

    armed = False
    n = {"ATT": 0, "OUT": 0}
    t0 = time.time()
    last_beat = last_print = t0
    vbat = ""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        write_header(w)
        try:
            while True:
                now = time.time()
                if args.seconds is not None and now - t0 >= args.seconds:
                    break
                if os.path.exists(STOP_FLAG):
                    break
                if now - last_beat > 1.0:
                    beat()
                    last_beat = now
                msg = m.recv_match(blocking=True, timeout=0.5)
                if msg is None:
                    continue
                t = msg.get_type()
                if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    armed = bool(msg.base_mode
                                 & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == "ATTITUDE":
                    w.writerow(att_row(msg.time_boot_ms, armed, msg.roll, msg.pitch,
                                       msg.yaw, msg.rollspeed, msg.pitchspeed,
                                       msg.yawspeed))
                    n["ATT"] += 1
                elif t == "SERVO_OUTPUT_RAW":
                    w.writerow(out_row(msg.time_usec // 1000, armed,
                                       (msg.servo1_raw, msg.servo2_raw,
                                        msg.servo3_raw, msg.servo4_raw), vbat))
                    n["OUT"] += 1
                elif t == "SYS_STATUS":
                    vbat = f"{msg.voltage_battery / 1000.0:.2f}"
                if now - last_print > 5.0:
                    f.flush()
                    dt = now - t0
                    print(f"  {dt:5.0f}s armed={armed} att={n['ATT'] / dt:.0f}Hz "
                          f"out={n['OUT'] / dt:.0f}Hz vbat={vbat}", flush=True)
                    last_print = now
        except KeyboardInterrupt:
            pass
    dt = time.time() - t0
    print(f"done: {n['ATT']} attitude rows ({n['ATT'] / dt:.0f} Hz), "
          f"{n['OUT']} output rows ({n['OUT'] / dt:.0f} Hz) -> {path}", flush=True)
    if n["ATT"] == 0:
        print("FAIL: zero attitude rows -- nothing was recorded", flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
