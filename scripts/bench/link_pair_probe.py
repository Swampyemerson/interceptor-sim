#!/usr/bin/env python3
"""READ-ONLY probe of the two MAVLink links the flight program can use at once.

Connects one MAVSDK server to each link (the Pi's serial cable to TELEM2, and
the flight recorder's UDP bridge to the Pixhawk's USB port), listens for a few
seconds and prints, per link: how often each stream the flight program reads
arrives, the longest gap, the battery figure and the GPS quality figures.

It sends NOTHING of its own: no setpoint, no command, no parameter write. (The
MAVSDK servers send their heartbeats and their usual start-up requests, as the
flight program's would.) Safe with props on.

    .venv-pi/bin/python scripts/bench/link_pair_probe.py --seconds 15
"""
import argparse
import asyncio
import json
import time

STREAMS = ("attitude_quaternion", "attitude_euler", "position", "velocity_ned",
           "flight_mode", "armed", "landed_state", "battery", "gps_info", "raw_gps")
DIRECT = ("RC_CHANNELS", "SYSTEM_TIME")


async def listen(name, system, seconds, out):
    stats = {s: {"n": 0, "last": None, "gap": 0.0, "sample": None}
             for s in STREAMS + DIRECT}

    def hit(key, sample):
        st, now = stats[key], time.monotonic()
        if st["last"] is not None:
            st["gap"] = max(st["gap"], now - st["last"])
        st["last"], st["n"], st["sample"] = now, st["n"] + 1, sample

    async def tel(key):
        async for x in getattr(system.telemetry, key)():
            hit(key, x)

    async def direct(key):
        async for m in system.mavlink_direct.message(key):
            hit(key, None)

    tasks = [asyncio.ensure_future(tel(k)) for k in STREAMS]
    tasks += [asyncio.ensure_future(direct(k)) for k in DIRECT]
    await asyncio.sleep(seconds)
    for t in tasks:
        t.cancel()
    rows = {}
    for key, st in stats.items():
        rows[key] = {"hz": round(st["n"] / seconds, 1),
                     "longest_gap_s": round(st["gap"], 3)}
    b = stats["battery"]["sample"]
    g = stats["gps_info"]["sample"]
    r = stats["raw_gps"]["sample"]
    rows["_battery"] = None if b is None else {
        "voltage_v": round(b.voltage_v, 2),
        "remaining_percent_as_mavsdk_gives_it": b.remaining_percent}
    rows["_gps"] = None if g is None or r is None else {
        "satellites": g.num_satellites, "fix_type": str(g.fix_type),
        "horizontal_uncertainty_m": round(r.horizontal_uncertainty_m, 2),
        "velocity_uncertainty_m_s": round(r.velocity_uncertainty_m_s, 2)}
    out[name] = rows


async def main_async(args):
    from mavsdk import System
    links = [("primary", args.primary, 50061, 190),
             ("backup", args.backup, 50062, 191)]
    systems, out = [], {}
    try:
        for name, url, port, compid in links:
            if not url:
                continue
            s = System(port=port, compid=compid)
            print(f"connecting {name}: {url} ...", flush=True)
            try:
                await asyncio.wait_for(s.connect(system_address=url), timeout=25)
            except asyncio.TimeoutError:
                print(f"  {name}: NO CONNECTION within 25 s", flush=True)
                out[name] = "no connection"
                proc = getattr(s, "_server_process", None)
                if proc is not None:
                    proc.kill()
                continue
            systems.append((name, s))
        await asyncio.gather(*(listen(n, s, args.seconds, out) for n, s in systems))
    finally:
        for _, s in systems:
            proc = getattr(s, "_server_process", None)
            if proc is not None and proc.poll() is None:
                proc.kill()
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--primary", default="serial:///dev/ttyAMA10:921600")
    ap.add_argument("--backup", default="udpout://127.0.0.1:14550")
    ap.add_argument("--seconds", type=float, default=15.0)
    asyncio.run(main_async(ap.parse_args()))
