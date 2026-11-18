#!/usr/bin/env python3
"""Pi side of the cue-link bench: receive for a while, echo every record back
to its sender (so the sender can time the round trip), then latch as the GO
edge would and print the belief and the link statistics as JSON.

    on the Pi:      python scripts/bench/cue_link_bench.py --listen-s 30
    on the laptop:  python scripts/field/cue_send.py --to PI --synthetic 200,-300,10,0,12 --echo

No vehicle link is opened. Nothing is commanded.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from flight.cue_link import DEFAULT_PORT, CueLinkError, CueReceiver   # noqa: E402
from flight.cue_relay import CueQualityError                           # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--listen-s", type=float, default=30.0)
    ap.add_argument("--own-lla", default="40.0,-105.0,1650.0")
    ap.add_argument("--max-staleness-s", type=float, default=1.0)
    ap.add_argument("--utc", choices=("none", "system"), default="none",
                    help="system = use this computer's clock as UTC and REQUIRE "
                         "the records' t_utc stamps (both clocks must be "
                         "disciplined): the link delay is then measured per "
                         "record and reported in latch_info")
    args = ap.parse_args()
    own = tuple(float(x) for x in args.own_lla.split(","))

    holder = {}

    def on_record(d, addr):
        # Echo from the RECEIVING socket: a reply from any other port does not
        # get back through the laptop's address translation.
        sock = getattr(holder.get("rx"), "_sock", None)
        if addr is not None and "seq" in d and sock is not None:
            try:
                sock.sendto(json.dumps({"seq": d["seq"]}).encode(), addr)
            except OSError:
                pass

    if args.utc == "system":
        from flight.cue_link import SystemClockUtc
        rx = CueReceiver(port=args.port, on_record=on_record,
                         utc=SystemClockUtc(), allow_unsynced=False)
    else:
        rx = CueReceiver(port=args.port, on_record=on_record)
    holder["rx"] = rx
    rx.start()
    t0 = time.monotonic()
    first = None
    while time.monotonic() - t0 < args.listen_s:
        st = rx.stats()
        if first is None and st.n_rx:
            first = time.monotonic() - t0
        time.sleep(0.05)
    st = rx.stats()
    out = {"listen_s": args.listen_s, "first_record_after_s": first, "stats": st.__dict__}
    # the sender-clock time that the latch instant corresponds to, by the same
    # minimum-delay anchor the receiver uses (lets the sender compare the belief
    # with its own truth without the two clocks being synchronised)
    items = list(rx._buf)
    go = time.monotonic()
    try:
        sol = rx.latch(*own, now_t=go, max_staleness_s=args.max_staleness_s)
        out["latched"] = True
        out["belief_r0_ned"] = sol.belief_r0_ned
        out["belief_vel0_ned"] = sol.belief_vel0_ned
        out["n_points"] = sol.n_points
        out["span_s"] = sol.span_s
        out["residual_rms_m"] = sol.residual_rms_m
        out["vel_source"] = sol.vel_source
        out["model"] = sol.model
        # each kept item is (t_rx, t_src, record, t_utc, utc_at_receipt)
        if items and all(it[1] is not None for it in items):
            out["latch_t_on_sender_clock"] = go - min(it[0] - it[1] for it in items)
        out["latch_info"] = rx.latch_info
    except (CueQualityError, CueLinkError) as e:
        out["latched"] = False
        out["refused"] = str(e)
    print(json.dumps(out, indent=1))
    return 0 if out["latched"] else 1


if __name__ == "__main__":
    sys.exit(main())
