#!/usr/bin/env python3
"""Set the PX4 speed-envelope params on a running SITL, with READ-BACK
verification (standing rule: param writes need a read-back — the FC bench
'wrote 23/23 but 4 missing' lesson).

The envelope is the SAME one scripts/m4_intercept.py has always set via
MAVSDK before arming (MPC_XY_VEL_MAX 20 etc.) — the cinema driver path
(real_flight) deliberately does not touch params, so demo runs that need the
envelope set it explicitly here instead of relying on SITL param persistence
from historical m4 runs. Connects on the GCS link (udp:14550), so it can run
BEFORE the driver opens 14540.

Usage: .venv/bin/python scripts/cinema/set_px4_params.py [--url udpin:0.0.0.0:14550]
Exit 0 only when every param reads back at the requested value.
"""
import argparse
import sys
import time

# NOTE on MPC_XY_VEL_MAX: the firmware declares @max 20, but that is GCS
# metadata -- MulticopterPositionControl uses the raw param value with no
# code-side clamp (checked 2026-10-21 in MulticopterPositionControl.cpp), so a
# raw MAVLink set beyond 20 takes effect. Beyond-envelope values are untested
# controller territory; the demo flies them knowingly and the HUD shows truth.
PARAMS = {
    "MPC_XY_VEL_MAX": 20.0,   # overridable via --vmax
    "MPC_ACC_HOR_MAX": 15.0,
    "MPC_TILTMAX_AIR": 60.0,
    "MPC_JERK_MAX": 30.0,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--url", default="udpin:0.0.0.0:14550")
    ap.add_argument("--timeout-s", type=float, default=20.0)
    ap.add_argument("--vmax", type=float, default=None,
                    help="override MPC_XY_VEL_MAX (raw set; >20 exceeds the "
                         "declared metadata max knowingly)")
    args = ap.parse_args()
    if args.vmax is not None:
        PARAMS["MPC_XY_VEL_MAX"] = float(args.vmax)

    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.url)
    hb = m.wait_heartbeat(timeout=args.timeout_s)
    if hb is None:
        print("[px4-params] FAIL: no heartbeat")
        return 2
    ok = True
    for name, val in PARAMS.items():
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(val),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        got = None
        deadline = time.time() + 5.0
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg and msg.param_id == name:
                got = float(msg.param_value)
                break
        # read-back from a FRESH request, not the set echo
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg and msg.param_id == name:
                got = float(msg.param_value)
                break
        status = "OK" if (got is not None and abs(got - val) < 1e-3) else "FAIL"
        if status == "FAIL":
            ok = False
        print(f"[px4-params] {status} {name} = {got} (want {val})")
    print(f"[px4-params] {'ALL OK' if ok else 'FAILURES'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
