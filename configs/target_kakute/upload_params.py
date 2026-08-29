#!/usr/bin/env python3
"""Upload target.param to the Kakute H7 over MAVLink, then READ IT BACK and diff.

WHY THIS EXISTS
    configs/target_kakute/README.md §5 says, of the read-back step: "This is the
    step people skip. A write that didn't take is invisible from the writing
    side — you only see it by reading back." Doing that by eye in Mission
    Planner's parameter tree, across 44 parameters, is exactly the kind of
    manual check that gets skipped or done badly at a bench at midnight.

    So this does the write AND the verification, and exits non-zero if a single
    parameter did not land. It is the same discipline the flash itself gets
    (STM32CubeProgrammer's `-v` read-back verify) applied one layer up.

THE FAILURE THIS CATCHES
    ArduPilot 4.7 renamed five of the parameters this pack sets, changed the
    units of three, and inverted the sense of one. Mission Planner SKIPS a name
    it does not recognise with only a warning. On the wrong firmware the config
    half-loads and looks fine -- including WPNAV_SPEED, which is what holds the
    target at >=9 m/s, i.e. the entire point of the aircraft. A silent partial
    load is the worst outcome available, so this script refuses to report
    success unless every parameter reads back at its intended value.

NO VACUOUS VERDICTS
    Zero parameters compared is a FAILURE, never a pass. A run that connected to
    nothing, or parsed an empty file, exits non-zero and says so.

RUNS ON *WINDOWS* PYTHON, because the FC enumerates as a Windows COM port and
WSL2 cannot see it:
    python.exe configs/target_kakute/upload_params.py --port COM7
    python.exe configs/target_kakute/upload_params.py --port COM7 --dry-run
"""

from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PARAM_FILE = os.path.join(HERE, "target.param")

# Parameters that only take effect after a reboot; reported so the operator
# knows a power-cycle is required before the values mean anything.
REBOOT_REQUIRED = {"BRD_ALT_CONFIG", "BATT_MONITOR", "SERIAL6_PROTOCOL",
                   "MOT_PWM_TYPE", "SERVO_BLH_AUTO"}

# A tolerance, not equality: MAVLink carries parameters as float32, so a value
# like 176126 or 0.15 does not necessarily survive the round trip bit-exact.
# Relative tolerance with an absolute floor for values near zero.
RTOL = 1e-6
ATOL = 1e-6


def parse_param_file(path):
    """ArduPilot .param: `NAME,VALUE` or `NAME<whitespace>VALUE`, # comments."""
    out = {}
    with open(path) as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.split("#")[0].strip()
            if not line:
                continue
            if "," in line:
                name, _, val = line.partition(",")
            else:
                parts = line.split()
                if len(parts) < 2:
                    continue
                name, val = parts[0], parts[1]
            name = name.strip().upper()
            try:
                out[name] = float(val.strip())
            except ValueError:
                print(f"  ! line {lineno}: cannot parse value {val!r} for {name}")
    return out


def close_enough(a, b):
    return abs(a - b) <= max(ATOL, RTOL * max(abs(a), abs(b)))


def fetch_all_params(master, timeout_s=90, quiet_after_s=4.0):
    """Request the full parameter set and collect until the stream goes quiet."""
    master.mav.param_request_list_send(master.target_system,
                                       master.target_component)
    params = {}
    deadline = time.time() + timeout_s
    last_msg = time.time()
    expected = None
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - last_msg > quiet_after_s and params:
                break
            continue
        last_msg = time.time()
        params[msg.param_id.strip()] = msg.param_value
        expected = msg.param_count
        if expected and len(params) >= expected:
            break
    return params, expected


def set_param(master, name, value, timeout_s=3.0, attempts=4):
    """Write one parameter and confirm the FC echoes the new value back."""
    for _ in range(attempts):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode("utf-8"), float(value),
            9)  # MAV_PARAM_TYPE_REAL32 -- ArduPilot decodes by its own type
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True,
                                    timeout=0.5)
            if msg and msg.param_id.strip() == name:
                if close_enough(msg.param_value, float(value)):
                    return True, msg.param_value
                return False, msg.param_value
    return False, None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True,
                    help="Windows COM port of the FC, e.g. COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--file", default=DEFAULT_PARAM_FILE)
    ap.add_argument("--dry-run", action="store_true",
                    help="report what WOULD change; write nothing")
    args = ap.parse_args()

    from pymavlink import mavutil  # noqa: PLC0415

    want = parse_param_file(args.file)
    if not want:
        print(f"FAIL: parsed ZERO parameters from {args.file} -- refusing to "
              f"report success on an empty file.")
        return 2
    print(f"[params] {len(want)} parameters to enforce, from {args.file}")

    print(f"[params] connecting to {args.port} @ {args.baud} ...")
    master = mavutil.mavlink_connection(args.port, baud=args.baud)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print("FAIL: no MAVLink heartbeat. Is the board running ArduPilot and "
              "is the COM port right?")
        return 2
    print(f"[params] heartbeat: system {master.target_system} "
          f"component {master.target_component}")

    have, expected = fetch_all_params(master)
    print(f"[params] read {len(have)} parameters off the board"
          + (f" (it reports {expected})" if expected else ""))
    if not have:
        print("FAIL: read ZERO parameters from the board.")
        return 2

    missing = sorted(n for n in want if n not in have)
    todo = {n: v for n, v in want.items()
            if n in have and not close_enough(have[n], v)}

    if missing:
        print(f"\n!! {len(missing)} parameter(s) DO NOT EXIST on this firmware:")
        for n in missing:
            print(f"     {n}")
        print("   This is the 4.7 rename trap. Confirm the board is running "
              "ArduCopter 4.7.0 -- on 4.5/4.6 these names are silently ignored\n"
              "   and the config half-loads.")

    print(f"\n[params] {len(todo)} need changing, "
          f"{len(want) - len(todo) - len(missing)} already correct")
    for n, v in sorted(todo.items()):
        print(f"     {n}: {have[n]} -> {v}")

    if args.dry_run:
        print("\n--dry-run: nothing written.")
        return 1 if missing else 0

    failed_writes = []
    for n, v in sorted(todo.items()):
        ok, got = set_param(master, n, v)
        if not ok:
            failed_writes.append((n, v, got))
            print(f"  ! write FAILED {n} -> {v} (board said {got})")
    print(f"[params] wrote {len(todo) - len(failed_writes)}/{len(todo)}")

    # --- THE READ-BACK. A fresh full fetch, not the write echoes. -----------
    print("\n[params] re-reading the FULL parameter set to verify...")
    time.sleep(1.0)
    after, _ = fetch_all_params(master)
    if not after:
        print("FAIL: read ZERO parameters on the verification pass.")
        return 2

    mismatched, absent = [], []
    for n, v in sorted(want.items()):
        if n not in after:
            absent.append(n)
        elif not close_enough(after[n], v):
            mismatched.append((n, v, after[n]))

    checked = len(want) - len(absent)
    print(f"[params] verified {checked} parameter(s) by read-back")
    if checked == 0:
        print("FAIL: verified ZERO parameters -- that is a failure, not a pass.")
        return 2

    if mismatched:
        print(f"\n!! {len(mismatched)} parameter(s) did NOT take:")
        for n, wanted, got in mismatched:
            print(f"     {n}: wanted {wanted}, board has {got}")
    if absent:
        print(f"\n!! {len(absent)} parameter(s) absent from this firmware:")
        for n in absent:
            print(f"     {n}")

    touched_reboot = sorted(REBOOT_REQUIRED & set(todo))
    if touched_reboot:
        print(f"\n[params] REBOOT REQUIRED before these mean anything: "
              f"{', '.join(touched_reboot)}")

    if mismatched or absent or failed_writes:
        print("\nRESULT: FAIL -- the board does not match target.param.")
        return 1
    print(f"\nRESULT: PASS -- all {checked} parameters read back at their "
          f"intended values.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
