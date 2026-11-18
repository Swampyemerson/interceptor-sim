#!/usr/bin/env python3
"""Ground side of the live cue link: send the target's GPS to the Pi over UDP.

Sources (pick one):
  --mavlink URL      the target's own telemetry (GLOBAL_POSITION_INT), e.g.
                     udpin:0.0.0.0:14550 or a serial/COM port. `t_src` is the
                     autopilot's own sample clock (time_boot_ms), so radio and
                     laptop jitter do not reach the fit.
  --replay FILE      a cue JSON-lines file (flight.cue_relay schema), replayed
                     at its own spacing.
  --synthetic N,E,ALT,VN,VE
                     a made-up straight track for the bench: start N,E metres
                     from --own-lla, ALT metres above it, velocity VN,VE m/s.

TIMESTAMPS (--stamp-utc). Each record can carry `t_utc`, the UTC time its
sample was TAKEN, so the Pi can measure the link delay per record and take it
out when it advances the belief to GO:
  gps     (default for --mavlink) the target autopilot's own GPS-disciplined
          clock: its SYSTEM_TIME pairs UTC with time_boot_ms, and every
          position report carries time_boot_ms. Records are sent WITHOUT t_utc
          until that mapping is trusted (the Pi then refuses to GO unless it
          was told unsynced records are allowed).
  system  this computer's wall clock at the moment of sampling (--synthetic,
          --replay) or of receipt (--mavlink: the radio leg's delay is then NOT
          measured). Right only if this clock is disciplined. A declared
          fallback, default for --synthetic and --replay.
  none    no t_utc.

Every record is one datagram (flight.cue_link.encode_record). With --echo the
sender also listens for the receiver's echoes (scripts/bench/cue_link_bench.py)
and reports round-trip times and loss.

This program only SENDS position reports. It commands nothing.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from flight.cue_link import DEFAULT_PORT, encode_record            # noqa: E402
from flight.cue_relay import (BootToUtc, CueRecord, cue_record_from_global_position_int,  # noqa: E402
                              read_cue_jsonl)

_R = 6378137.0


def _ned_to_lla(n, e, up, own):
    lat = own[0] + math.degrees(n / _R)
    lon = own[1] + math.degrees(e / (_R * math.cos(math.radians(own[0]))))
    return lat, lon, own[2] + up


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--to", required=True, help="Pi address, HOST or HOST:PORT")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--mavlink")
    src.add_argument("--replay")
    src.add_argument("--synthetic")
    ap.add_argument("--own-lla", default="40.0,-105.0,1650.0",
                    help="origin for --synthetic: lat,lon,alt_m_msl")
    ap.add_argument("--rate", type=float, default=5.0, help="Hz, --synthetic only")
    ap.add_argument("--noise-m", type=float, default=0.0, help="--synthetic position noise, 1 sigma")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--stamp-utc", choices=("gps", "system", "none"), default=None,
                    help="where t_utc comes from (see the module text); default "
                         "gps for --mavlink, system otherwise")
    ap.add_argument("--echo", action="store_true")
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    host, _, port = args.to.partition(":")
    dest = (host, int(port) if port else DEFAULT_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.0)
    sent = {}          # seq -> send time
    rtts = []
    n_sent = 0

    stamp = args.stamp_utc or ("gps" if args.mavlink else "system")
    if stamp == "gps" and not args.mavlink:
        ap.error("--stamp-utc gps needs --mavlink (the autopilot's SYSTEM_TIME)")
    n_utc = 0

    def send(rec: CueRecord, t_src: float, t_utc=None) -> None:
        nonlocal n_sent, n_utc
        if t_utc is not None:
            n_utc += 1
        sock.sendto(encode_record(rec, t_src=t_src, seq=n_sent, t_utc=t_utc), dest)
        sent[n_sent] = time.monotonic()
        n_sent += 1

    def drain() -> None:
        if not args.echo:
            return
        while True:
            try:
                data, _ = sock.recvfrom(4096)
            except (BlockingIOError, socket.timeout):
                return
            except OSError:
                return
            try:
                seq = int(json.loads(data.decode())["seq"])
            except (ValueError, KeyError):
                continue
            if seq in sent:
                rtts.append(time.monotonic() - sent.pop(seq))

    t_start = time.monotonic()
    synth = None
    if args.synthetic:
        import random
        n0, e0, up, vn, ve = (float(x) for x in args.synthetic.split(","))
        own = tuple(float(x) for x in args.own_lla.split(","))
        rng = random.Random(1)
        synth = dict(n0=n0, e0=e0, up=up, vn=vn, ve=ve, own=own, t0=t_start)
        k = 0
        while time.monotonic() - t_start < args.duration:
            t_due = t_start + k / args.rate
            while time.monotonic() < t_due:
                drain()
                time.sleep(0.002)
            ts = time.monotonic()
            dt = ts - t_start
            lat, lon, alt = _ned_to_lla(n0 + vn * dt + rng.gauss(0, args.noise_m),
                                        e0 + ve * dt + rng.gauss(0, args.noise_m), up, own)
            send(CueRecord(0.0, lat, lon, alt, vn, ve, 0.0), ts,
                 time.time() if stamp == "system" else None)
            k += 1
    elif args.replay:
        recs = read_cue_jsonl(args.replay)
        if not recs:
            print("replay file has no records -- nothing sent"); return 1
        for r in recs:
            due = t_start + (r.t - recs[0].t)
            while time.monotonic() < due:
                drain()
                time.sleep(0.002)
            send(r, r.t, time.time() if stamp == "system" else None)
    else:
        from pymavlink import mavutil
        conn = mavutil.mavlink_connection(args.mavlink)
        clock = BootToUtc()
        while time.monotonic() - t_start < args.duration:
            msg = conn.recv_match(type=["GLOBAL_POSITION_INT", "SYSTEM_TIME"],
                                  blocking=True, timeout=0.5)
            drain()
            if msg is None:
                continue
            if msg.get_type() == "SYSTEM_TIME":
                clock.ingest(msg.time_unix_usec, msg.time_boot_ms)
                continue
            if stamp == "gps":
                t_utc = clock.utc_of_boot_ms(msg.time_boot_ms)
            elif stamp == "system":
                t_utc = time.time()
            else:
                t_utc = None
            send(cue_record_from_global_position_int(msg, 0.0),
                 msg.time_boot_ms / 1000.0, t_utc)
        out_clock = {"system_time_msgs": clock.n_msgs, "jumps": clock.n_jumps,
                     "no_gps_time": clock.n_invalid, "trusted_at_end": clock.ok}

    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:
        drain()
        time.sleep(0.01)

    out = {"sent": n_sent, "dest": f"{dest[0]}:{dest[1]}", "duration_s": time.monotonic() - t_start - 1.0,
           "stamp_utc": stamp, "sent_with_t_utc": n_utc}
    if args.mavlink:
        out["target_clock"] = out_clock
    if synth:
        out["synthetic"] = synth
    if args.echo:
        rt = sorted(rtts)
        out["echoes"] = len(rt)
        out["lost"] = n_sent - len(rt)
        if rt:
            out["rtt_ms"] = {"min": rt[0] * 1e3, "median": rt[len(rt) // 2] * 1e3,
                             "p95": rt[int(0.95 * (len(rt) - 1))] * 1e3, "max": rt[-1] * 1e3}
    print(json.dumps(out, indent=1))
    if args.json_out:
        json.dump(out, open(args.json_out, "w"), indent=1)
    if n_sent == 0:
        print("UNCERTAIN: nothing was sent"); return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
