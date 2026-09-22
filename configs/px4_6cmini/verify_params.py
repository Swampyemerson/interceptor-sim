#!/usr/bin/env python3
"""Read back configs/px4_6cmini/bench.params from the Pixhawk 6C Mini and diff.

The PX4 counterpart of configs/target_kakute/upload_params.py's read-back half
-- READ-ONLY by default. `--write` writes only the params that differ, then
re-reads each from a fresh request and exits non-zero unless every one landed
(the 2026-09-16 bench found "wrote 23/23" with 4 silently not landed; a write
that didn't take is invisible from the writing side).

PX4 WIRE ENCODING, the trap this file exists to handle: PX4 transmits an INT32
parameter's RAW BYTES REINTERPRETED into the MAVLink float field (value-cast is
ArduPilot's convention, not PX4's). So an INT32 must be decoded
struct.unpack('<i', struct.pack('<f', param_value)) and written the mirror way
-- a naive float read of SER_TEL2_BAUD=921600 shows 1.29e-38 and a naive write
puts garbage on the vehicle with a plausible echo. Found live 2026-08-29
(GPS_1_CONFIG 202 arrived as 2.83e-43); see upload_params.py's ARDUPILOT-ONLY
banner for the same fact from the other side.

NO VACUOUS VERDICTS: zero parameters compared is a FAILURE, never a pass.

RUNS ON *WINDOWS* PYTHON (the FC enumerates as a COM port WSL2 cannot see):
    python.exe configs/px4_6cmini/verify_params.py --port COM5
    python.exe configs/px4_6cmini/verify_params.py --port COM5 --write
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time

from pymavlink import mavutil

PARAMS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bench.params")
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9


def load_pack(path: str) -> list[tuple[str, float, int]]:
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 5:
                raise SystemExit(f"unparseable line in {path!r}: {line!r}")
            _sys, _comp, name, value, ptype = parts
            out.append((name, float(value), int(ptype)))
    if not out:
        raise SystemExit(f"{path!r} yielded ZERO parameters -- refusing a vacuous run")
    return out


def wire_to_value(msg) -> float | int:
    """Decode a PARAM_VALUE respecting PX4's raw-bytes INT32 encoding."""
    if msg.param_type == MAV_PARAM_TYPE_REAL32:
        return float(msg.param_value)
    return struct.unpack("<i", struct.pack("<f", msg.param_value))[0]


def value_to_wire(value: float, ptype: int) -> float:
    """Encode a desired value into the float field PX4 expects."""
    if ptype == MAV_PARAM_TYPE_REAL32:
        return float(value)
    return struct.unpack("<f", struct.pack("<i", int(value)))[0]


def read_param(m, name: str, timeout_s: float = 4.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode("ascii"), -1)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True,
                           timeout=max(0.1, deadline - time.time()))
        if msg is not None and msg.param_id == name:
            return msg
    return None


def matches(got, want: float, ptype: int) -> bool:
    if ptype == MAV_PARAM_TYPE_REAL32:
        return abs(float(got) - want) <= 1e-4 * max(1.0, abs(want))
    return int(got) == int(want)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--write", action="store_true",
                    help="write the params that differ, then re-read each")
    args = ap.parse_args()

    pack = load_pack(PARAMS_FILE)
    m = mavutil.mavlink_connection(args.port, baud=args.baud)
    hb = m.wait_heartbeat(timeout=8)
    if hb is None:
        print("FAIL: no heartbeat")
        return 1
    if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        print(f"FAIL: autopilot={hb.autopilot} is not PX4 -- wrong board? "
              "(the Kakute uses configs/target_kakute/upload_params.py)")
        return 1

    diffs, unread = [], []
    print(f"reading {len(pack)} params back from {args.port} ...")
    for name, want, ptype in pack:
        msg = read_param(m, name)
        if msg is None:
            unread.append(name)
            print(f"  {name:<18} NO ANSWER")
            continue
        got = wire_to_value(msg)
        ok = matches(got, want, ptype)
        if msg.param_type != ptype:
            print(f"  {name:<18} TYPE MISMATCH: pack says {ptype}, board says "
                  f"{msg.param_type} (board value {got})")
            diffs.append((name, want, ptype, got))
            continue
        print(f"  {name:<18} board={got!r:<12} pack={want!r:<12} "
              f"{'OK' if ok else 'DIFF'}")
        if not ok:
            diffs.append((name, want, ptype, got))

    if unread:
        print(f"FAIL: {len(unread)} param(s) unreadable: {', '.join(unread)}")
        return 1
    if not diffs:
        print(f"VERIFY PASS: all {len(pack)} params match bench.params")
        return 0
    if not args.write:
        print(f"VERIFY DIFF: {len(diffs)} param(s) differ -- rerun with --write "
              "to apply (writes ONLY the differing ones, then re-reads each)")
        return 2

    print(f"writing {len(diffs)} differing param(s) ...")
    failed = []
    for name, want, ptype, _got in diffs:
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode("ascii"), value_to_wire(want, ptype), ptype)
        time.sleep(0.3)
        msg = read_param(m, name)   # fresh request, never the set's echo
        got2 = wire_to_value(msg) if msg is not None else None
        landed = msg is not None and matches(got2, want, ptype)
        print(f"  {name:<18} -> {want!r}: {'LANDED' if landed else f'DID NOT LAND (reads {got2!r})'}")
        if not landed:
            failed.append(name)
    if failed:
        print(f"WRITE FAIL: {len(failed)} did not land: {', '.join(failed)} "
              "-- a reboot + second pass may be needed (2026-09-16 lesson)")
        return 1
    print(f"WRITE PASS: {len(diffs)} written and read back. Reboot the board and "
          "run verify once more -- some params only latch across a reboot.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
