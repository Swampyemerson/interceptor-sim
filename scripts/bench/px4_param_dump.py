#!/usr/bin/env python3
"""READ-ONLY full parameter dump from a PX4 FC over a Windows COM port.

Writes NOTHING to the board (no PARAM_SET anywhere in this file). Fetches the
whole table with PARAM_REQUEST_LIST, re-requests any missing index, and FAILS
(exit 2) if the table is incomplete -- a partial dump must never pass as "the
board's parameters" (docs/error_handling_policy.md, no vacuous verdicts).

PX4 INT32 TRAP: PX4 sends an INT32 parameter as its raw bytes reinterpreted in
MAVLink's float field (202 arrives as 2.83e-43). This tool decodes by
param_type, so integers print as integers.

    python.exe px4_param_dump.py --port COM5 --out px4_params.tsv [--show PWM_ DSHOT_]
"""
from __future__ import annotations

import argparse
import struct
import time

INT_TYPES = {1, 2, 3, 4, 5, 6}  # MAV_PARAM_TYPE_UINT8..INT32


def decode(msg):
    if msg.param_type in INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
    return float(msg.param_value)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", required=True)
    ap.add_argument("--show", nargs="*", default=[],
                    help="name prefixes to echo to stdout")
    args = ap.parse_args()

    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.port, baud=args.baud,
                                   source_system=255, source_component=190)
    for _ in range(4):
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.3)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("FAIL: no heartbeat")
        return 2
    print(f"heartbeat: sys {m.target_system} comp {m.target_component} "
          f"autopilot {hb.autopilot} type {hb.type}")
    if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        print("FAIL: not a PX4 autopilot -- this tool's int decoding is PX4-only")
        return 2

    got: dict[int, tuple[str, object, int]] = {}
    total = None
    m.mav.param_request_list_send(m.target_system, 1)
    last = time.time()
    while time.time() - last < 4.0:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.param_index == 65535:
            continue
        last = time.time()
        total = msg.param_count
        got[msg.param_index] = (msg.param_id, decode(msg), msg.param_type)
        if total and len(got) >= total:
            break
    if not total:
        print("FAIL: no PARAM_VALUE received")
        return 2
    for _ in range(3):
        missing = [i for i in range(total) if i not in got]
        if not missing:
            break
        print(f"re-requesting {len(missing)} missing indices")
        for i in missing:
            m.mav.param_request_read_send(m.target_system, 1, b"", i)
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
            if msg is not None and msg.param_index != 65535:
                got[msg.param_index] = (msg.param_id, decode(msg), msg.param_type)
    missing = [i for i in range(total) if i not in got]

    rows = sorted(got.values())
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(f"# px4_param_dump.py  port={args.port}  "
                f"utc={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}  "
                f"count={len(rows)}/{total}\n")
        for name, val, typ in rows:
            f.write(f"{name}\t{val}\t{typ}\n")
    print(f"wrote {len(rows)}/{total} params -> {args.out}")
    for name, val, typ in rows:
        if any(name.startswith(p) for p in args.show):
            print(f"  {name} = {val}")
    if missing:
        print(f"FAIL: {len(missing)} parameters never arrived -- dump INCOMPLETE")
        return 2
    print("done (read-only; nothing written)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
