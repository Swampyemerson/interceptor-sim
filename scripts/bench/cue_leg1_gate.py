#!/usr/bin/env python3
"""Leg-1 bench gate: does target GPS telemetry actually reach the ground?

The cue relay's ONE unbuilt link (docs/cue_relay_plan.md §2/§11) is getting
MAVLink telemetry OFF the target aircraft wirelessly -- either the ELRS
WiFi-backpack path (TX rebroadcasts MAVLink on UDP) or a dedicated telemetry
radio on the Kakute's SERIAL2 (already MAVLink2, verified 2026-09-22 probe).
This script is the PASS/FAIL instrument for either path, run on the ground
machine that will play cue relay at the field.

PRE-REGISTERED CRITERIA (written before the first run, per the standing rule):

  TRANSPORT-PASS  >= 30 s of stream observed AND GLOBAL_POSITION_INT mean
                  rate >= 2.0 Hz over the window AND max inter-arrival gap
                  <= 2.0 s. This certifies the RF+plumbing leg only.
  CUE-PASS        TRANSPORT-PASS and additionally a 3D GPS fix was present
                  (GPS_RAW_INT fix_type >= 3) and `flight.cue_relay.fit_cue`
                  emits a CueSolution from the final 2.5 s window. On an
                  indoor bench with no fix this is EXPECTED TO FAIL -- the
                  verdict then reads TRANSPORT-PASS / CUE-UNTESTED, which is
                  a pass for this bench item (the fitter itself is already
                  contract-tested against a real .BIN).
  FAIL            anything else. ZERO messages is a loud FAIL, never a
                  silent empty log (no vacuous verdicts).

Exit code: 0 on TRANSPORT-PASS (with or without CUE), 1 otherwise.

USAGE
  # ELRS backpack default (UDP broadcast on the backpack's network):
  python3 scripts/bench/cue_leg1_gate.py --conn udpin:0.0.0.0:14550
  # Dedicated radio / SiK on a serial port (Windows: run with python.exe):
  python.exe scripts/bench/cue_leg1_gate.py --conn COM8 --baud 57600
  # Wired control run (proves the instrument itself against USB truth):
  python.exe scripts/bench/cue_leg1_gate.py --conn COM7 --baud 115200 --request

Every run appends a summary JSON to runs/cue_leg1/ and the raw records to a
cue JSONL written by flight.cue_relay's OWN writer (producer->consumer
discipline: the file this gate writes is the file the flight CLI reads).
"""
import argparse
import json
import os
import sys
import time

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from flight.cue_relay import (  # noqa: E402
    CueQualityError, cue_record_from_global_position_int, fit_cue,
    write_cue_jsonl,
)

TRANSPORT_MIN_STREAM_S = 30.0
TRANSPORT_MIN_RATE_HZ = 2.0
TRANSPORT_MAX_GAP_S = 2.0
FIT_WINDOW_S = 2.5


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--conn", required=True,
                    help="pymavlink connection string: udpin:0.0.0.0:14550 "
                         "(ELRS backpack), COMx / /dev/ttyUSBx (radio), ...")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--duration", type=float, default=45.0,
                    help="listen window, seconds (>= 30 for a valid verdict)")
    ap.add_argument("--request", action="store_true",
                    help="actively request GLOBAL_POSITION_INT @5Hz (wired "
                         "control runs; the RF paths should stream unasked)")
    ap.add_argument("--out-dir", default=os.path.join(_REPO, "runs", "cue_leg1"))
    args = ap.parse_args()

    from pymavlink import mavutil  # local: keep the module importable without it
    m = mavutil.mavlink_connection(args.conn, baud=args.baud)
    print(f"[leg1] listening on {args.conn} for {args.duration:.0f}s ...")
    hb = m.wait_heartbeat(timeout=15)
    if hb is None:
        print("[leg1] FAIL: no HEARTBEAT in 15 s -- nothing is arriving at "
              "all (TX off? wrong network/port? backpack not forwarding?)")
        return 1
    print(f"[leg1] heartbeat from sysid {m.target_system} "
          f"(autopilot {hb.autopilot})")
    if args.request:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, 33, 200000, 0, 0, 0, 0, 0)

    records, gp_times, fix_type = [], [], None
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "GPS_RAW_INT"],
                           blocking=True, timeout=1.0)
        if msg is None:
            continue
        now = time.monotonic()
        if msg.get_type() == "GLOBAL_POSITION_INT":
            gp_times.append(now)
            records.append(cue_record_from_global_position_int(msg, now))
        else:
            fix_type = max(fix_type or 0, msg.fix_type)

    span = (gp_times[-1] - gp_times[0]) if len(gp_times) >= 2 else 0.0
    rate = (len(gp_times) - 1) / span if span > 0 else 0.0
    max_gap = max((b - a for a, b in zip(gp_times, gp_times[1:])), default=None)
    transport_ok = (span >= TRANSPORT_MIN_STREAM_S
                    and rate >= TRANSPORT_MIN_RATE_HZ
                    and max_gap is not None and max_gap <= TRANSPORT_MAX_GAP_S)

    cue_verdict, cue_detail = "CUE-UNTESTED", "no 3D fix during the window"
    if transport_ok and (fix_type or 0) >= 3:
        tail = [r for r in records if r.t >= records[-1].t - FIT_WINDOW_S]
        try:
            sol = fit_cue(tail, records[-1].lat, records[-1].lon,
                          records[-1].alt_m_msl)
            cue_verdict = "CUE-PASS"
            cue_detail = (f"model={sol.model} n={sol.n_points} "
                          f"residual={sol.residual_rms_m:.2f} m")
        except CueQualityError as e:
            cue_verdict, cue_detail = "CUE-FAIL", str(e)

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    if records:
        write_cue_jsonl(records, os.path.join(args.out_dir, f"cue_{stamp}.jsonl"))
    summary = {
        "stamp": stamp, "conn": args.conn, "n_gpi": len(gp_times),
        "span_s": round(span, 2), "rate_hz": round(rate, 2),
        "max_gap_s": round(max_gap, 3) if max_gap is not None else None,
        "fix_type": fix_type,
        "transport": "TRANSPORT-PASS" if transport_ok else "TRANSPORT-FAIL",
        "cue": cue_verdict, "cue_detail": cue_detail,
        "criteria": {"min_stream_s": TRANSPORT_MIN_STREAM_S,
                     "min_rate_hz": TRANSPORT_MIN_RATE_HZ,
                     "max_gap_s": TRANSPORT_MAX_GAP_S},
    }
    with open(os.path.join(args.out_dir, f"summary_{stamp}.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    print(json.dumps(summary, indent=1))
    if not gp_times:
        print("[leg1] FAIL: heartbeats but ZERO GLOBAL_POSITION_INT -- the "
              "link is up and the position stream is not (rate config, or "
              "the backpack forwards only some messages).")
        return 1
    print(f"[leg1] {summary['transport']} / {cue_verdict}")
    return 0 if transport_ok else 1


if __name__ == "__main__":
    sys.exit(main())
