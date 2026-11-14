#!/usr/bin/env python3
"""Make (and optionally upload) the target's LOOPING route: fly a straight leg
out and back, over and over, until the pilot takes it out of Auto.

The route is two points, A and B. A is where the lap starts; B is --leg-m
metres from A along --bearing-deg (compass degrees, 0 = north, 90 = east).
The mission is: take off, set the speed, fly to A, fly to B, jump back to
"fly to A" (--repeats times, -1 = forever), then return home.

Where A comes from, one of:
    --start-lat / --start-lon      coordinates you supply
    --from-fc COM4                 the drone's own GPS position right now (set
                                   the drone down where the lap should start,
                                   battery in, and wait for a GPS fix)

    # at the field, drone sitting at the start of the leg, nose anywhere:
    python.exe gen_loop_mission.py --from-fc COM4 --bearing-deg 90 --upload COM4
    # from known coordinates, file only (open it in QGroundControl's Plan view):
    python3 gen_loop_mission.py --start-lat 37.1 --start-lon -122.1 \\
        --bearing-deg 90 --out loop.waypoints

HOW TO FLY IT: take off in Loiter or AltHold, climb to roughly the mission
height, then flip the mode switch DOWN to Auto. Flip it all the way UP
(Loiter) to stop and hold at any time. The mission cannot be started from the ground:
ArduPilot refuses to arm in Auto with this parameter set.

--upload writes the mission to the flight controller, reads it back and
compares every item; anything that differs is a FAILURE (exit 1). It refuses
to upload to an armed vehicle.

ArduPilot only (the target's Kakute). Runs on Windows Python for --from-fc and
--upload, because the flight controller is a Windows COM port.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WPL_HEADER = "QGC WPL 110"
EARTH_R_M = 6378137.0
FRAME_GLOBAL = 0
FRAME_REL_ALT = 3          # height above the arming point
CMD_WAYPOINT, CMD_RTL, CMD_TAKEOFF, CMD_DO_JUMP, CMD_DO_CHANGE_SPEED = 16, 20, 22, 177, 178
SEQ_FIRST_LEG_POINT = 3    # the item DO_JUMP returns to ("fly to A")
MIN_SATS, MIN_FIX = 8, 3


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Latitude/longitude of a point north_m / east_m from (lat, lon). Flat-earth,
    good to millimetres over a 100 m leg."""
    dlat = math.degrees(north_m / EARTH_R_M)
    dlon = math.degrees(east_m / (EARTH_R_M * math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


def distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    north = math.radians(b[0] - a[0]) * EARTH_R_M
    east = math.radians(b[1] - a[1]) * EARTH_R_M * math.cos(math.radians(a[0]))
    return math.hypot(north, east)


def build(lat: float, lon: float, bearing_deg: float, leg_m: float, alt_m: float,
          speed_ms: float, repeats: int) -> list[dict]:
    """The mission as a list of items (dicts with the 12 QGC WPL columns)."""
    b_lat, b_lon = offset(lat, lon, leg_m * math.cos(math.radians(bearing_deg)),
                          leg_m * math.sin(math.radians(bearing_deg)))

    def item(cmd, frame=FRAME_REL_ALT, p1=0.0, p2=0.0, p3=0.0, p4=0.0, x=0.0, y=0.0, z=0.0):
        return dict(frame=frame, command=cmd, p1=p1, p2=p2, p3=p3, p4=p4, x=x, y=y, z=z)

    items = [
        item(CMD_WAYPOINT, frame=FRAME_GLOBAL, x=lat, y=lon),   # 0: home slot (the FC overwrites it at arming)
        item(CMD_TAKEOFF, z=alt_m),                             # 1
        item(CMD_DO_CHANGE_SPEED, p1=1, p2=speed_ms, p3=-1),    # 2: ground speed, m/s
        item(CMD_WAYPOINT, x=lat, y=lon, z=alt_m),              # 3: A
        item(CMD_WAYPOINT, x=b_lat, y=b_lon, z=alt_m),          # 4: B
        item(CMD_DO_JUMP, p1=SEQ_FIRST_LEG_POINT, p2=repeats),  # 5: back to "fly to A"
        item(CMD_RTL),                                          # 6: only reached when repeats run out
    ]
    for seq, it in enumerate(items):
        it["seq"] = seq
    return items


def to_wpl(items: list[dict]) -> str:
    lines = [WPL_HEADER]
    for it in items:
        lines.append("\t".join([
            str(it["seq"]), "1" if it["seq"] == 0 else "0", str(it["frame"]), str(it["command"]),
            f"{it['p1']:.6f}", f"{it['p2']:.6f}", f"{it['p3']:.6f}", f"{it['p4']:.6f}",
            f"{it['x']:.7f}", f"{it['y']:.7f}", f"{it['z']:.3f}", "1"]))
    return "\n".join(lines) + "\n"


def fence_radius_m() -> float | None:
    """FENCE_RADIUS from target.param beside this file (None if unreadable)."""
    try:
        with open(os.path.join(HERE, "target.param")) as fh:
            for ln in fh:
                if ln.startswith("FENCE_RADIUS,"):
                    return float(ln.split(",")[1])
    except (OSError, ValueError):
        pass
    return None


def describe(items: list[dict], leg_m: float, speed_ms: float, repeats: int, wp_acc: float = 3.5) -> str:
    a, b = (items[3]["x"], items[3]["y"]), (items[4]["x"], items[4]["y"])
    ramp = speed_ms ** 2 / (2 * wp_acc)
    at_speed = max(0.0, leg_m - 2 * ramp)
    out = [f"A (lap start): {a[0]:.7f}, {a[1]:.7f}",
           f"B (far end):   {b[0]:.7f}, {b[1]:.7f}   {distance_m(a, b):.1f} m from A",
           f"height {items[3]['z']:.0f} m above the arming point, {speed_ms:g} m/s, "
           + ("laps until you leave Auto" if repeats < 0 else f"{repeats + 1} laps then return home"),
           f"at {wp_acc:g} m/s/s it needs about {ramp:.0f} m to reach speed and {ramp:.0f} m to stop: "
           f"about {at_speed:.0f} m of each {leg_m:g} m leg is at full speed"
           + ("" if at_speed > 0 else "  <-- NEVER reaches the set speed; lengthen the leg or slow down")]
    r = fence_radius_m()
    if r is not None:
        budget = r - leg_m - 5.0
        out.append(f"fence: {r:g} m around where you ARM. Arm within {budget:.0f} m of A "
                   f"(B is {leg_m:g} m further out; 5 m kept in hand)"
                   if budget > 0 else
                   f"fence: {r:g} m around where you ARM is SMALLER than this leg; the target will "
                   f"turn for home mid-leg")
    return "\n".join(out)


# --- talking to the flight controller ----------------------------------------

def connect(port: str):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(port, baud=115200)
    hb = m.wait_heartbeat(timeout=15)
    if hb is None:
        raise SystemExit(f"FAIL: no heartbeat on {port}")
    if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        raise SystemExit("FAIL: this is not an ArduPilot board (the interceptor's Pixhawk runs PX4)")
    return m, bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def position_from_fc(m, wait_s: float = 20.0) -> tuple[float, float]:
    """The drone's GPS position, refused unless it has a real fix."""
    t0, last = time.time(), None
    while time.time() - t0 < wait_s:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2.0)
        if msg is None:
            continue
        last = msg
        if msg.fix_type >= MIN_FIX and msg.satellites_visible >= MIN_SATS:
            return msg.lat / 1e7, msg.lon / 1e7
    if last is None:
        raise SystemExit("FAIL: the flight controller sent no GPS message")
    raise SystemExit(f"FAIL: no usable GPS fix (fix type {last.fix_type}, {last.satellites_visible} "
                     f"satellites; need a 3D fix and {MIN_SATS}+). Fix type 0 means the GPS is not "
                     f"powered or not connected: it needs the flight battery, USB alone does not run it.")


def upload(m, items: list[dict]) -> None:
    for attempt in range(3):
        m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
        t0 = time.time()
        while time.time() - t0 < 15:
            msg = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                               blocking=True, timeout=3.0)
            if msg is None:
                break
            if msg.get_type() == "MISSION_ACK":
                if msg.type == 0:
                    return
                raise SystemExit(f"FAIL: the flight controller rejected the mission (code {msg.type})")
            it = items[msg.seq]
            m.mav.mission_item_int_send(
                m.target_system, m.target_component, it["seq"], it["frame"], it["command"],
                0, 1, it["p1"], it["p2"], it["p3"], it["p4"],
                int(round(it["x"] * 1e7)), int(round(it["y"] * 1e7)), it["z"], 0)
    raise SystemExit("FAIL: mission upload did not complete in 3 tries")


def download(m) -> list[dict]:
    m.mav.mission_request_list_send(m.target_system, m.target_component, 0)
    cnt = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=5.0)
    if cnt is None:
        raise SystemExit("FAIL: the flight controller did not answer the read-back request")
    got = []
    for seq in range(cnt.count):
        for _ in range(4):
            m.mav.mission_request_int_send(m.target_system, m.target_component, seq, 0)
            msg = m.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=2.0)
            if msg is not None and msg.seq == seq:
                got.append(dict(seq=seq, frame=msg.frame, command=msg.command, p1=msg.param1,
                                p2=msg.param2, p3=msg.param3, p4=msg.param4,
                                x=msg.x / 1e7, y=msg.y / 1e7, z=msg.z))
                break
        else:
            raise SystemExit(f"FAIL: read-back of mission item {seq} timed out")
    m.mav.mission_ack_send(m.target_system, m.target_component, 0, 0)
    return got


def differences(want: list[dict], got: list[dict]) -> list[str]:
    """Every way the read-back differs from what was sent. Item 0 is the home
    slot, which the flight controller owns, so only its presence is checked."""
    if len(want) != len(got):
        return [f"sent {len(want)} items, the flight controller holds {len(got)}"]
    out = []
    for w, g in zip(want[1:], got[1:]):
        for k, tol in (("command", 0), ("frame", 0), ("p1", 1e-3), ("p2", 1e-3), ("p3", 1e-3),
                       ("x", 2e-7), ("y", 2e-7), ("z", 0.01)):
            # ArduPilot keeps no frame for a command that has no position (speed
            # change, jump, return home) and reads it back as 0. Seen on the
            # board 2026-11-14.
            if k == "frame" and w["command"] not in (CMD_WAYPOINT, CMD_TAKEOFF):
                continue
            if abs(w[k] - g[k]) > tol:
                out.append(f"item {w['seq']} {k}: sent {w[k]}, read back {g[k]}")
    return out


def self_test() -> int:
    items = build(37.0, -122.0, 90.0, 60.0, 10.0, 5.0, -1)
    a, b = (items[3]["x"], items[3]["y"]), (items[4]["x"], items[4]["y"])
    checks = [
        ("B is leg-m from A", abs(distance_m(a, b) - 60.0) < 0.01),
        ("bearing 90 goes east at the same latitude", abs(b[0] - a[0]) < 1e-9 and b[1] > a[1]),
        ("bearing 0 goes north", build(37.0, -122.0, 0.0, 60.0, 10, 5, -1)[4]["x"] > 37.0
         and abs(build(37.0, -122.0, 0.0, 60.0, 10, 5, -1)[4]["y"] + 122.0) < 1e-9),
        ("the jump returns to the item that is 'fly to A'",
         items[5]["command"] == CMD_DO_JUMP and items[int(items[5]["p1"])] is items[3]
         and items[3]["command"] == CMD_WAYPOINT),
        ("-1 repeats = forever", items[5]["p2"] == -1),
        ("ends in return-home for a finite count", items[-1]["command"] == CMD_RTL),
        ("heights are above the arming point", all(i["frame"] == FRAME_REL_ALT for i in items[1:])),
        ("12 tab-separated columns per line",
         all(len(ln.split("\t")) == 12 for ln in to_wpl(items).splitlines()[1:])),
        ("read-back compare passes on an identical copy",
         differences(items, [dict(i) for i in items]) == []),
        ("read-back compare catches a moved waypoint",
         differences(items, [dict(i, x=i["x"] + (1e-5 if i["seq"] == 4 else 0)) for i in items]) != []),
        ("read-back compare catches a dropped item", differences(items, items[:-1]) != []),
        ("a leg too short to reach speed is called out",
         "NEVER reaches" in describe(build(37.0, -122.0, 0, 20.0, 10, 10.0, -1), 20.0, 10.0, -1)),
    ]
    for name, ok in checks:
        print(("  ok   " if ok else "  FAIL ") + name)
    bad = [n for n, ok in checks if not ok]
    print("SELF-TEST " + ("PASSED" if not bad else f"FAILED: {bad}") + f" ({len(checks)} checks)")
    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start-lat", type=float)
    ap.add_argument("--start-lon", type=float)
    ap.add_argument("--from-fc", metavar="PORT", help="use the drone's current GPS position as A")
    ap.add_argument("--bearing-deg", type=float, help="compass direction from A to B (0 north, 90 east)")
    ap.add_argument("--leg-m", type=float, default=60.0, help="length of the leg, metres (default 60)")
    ap.add_argument("--alt-m", type=float, default=10.0, help="height above the arming point (default 10)")
    ap.add_argument("--speed-ms", type=float, default=5.0,
                    help="speed along the leg, m/s (default 5: first flights; intercept passes want 9+)")
    ap.add_argument("--repeats", type=int, default=-1, help="extra laps; -1 = until you leave Auto (default)")
    ap.add_argument("--out", help="write a .waypoints file (opens in QGroundControl / Mission Planner)")
    ap.add_argument("--upload", metavar="PORT", help="write the mission to the flight controller and verify")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if a.bearing_deg is None or not 0 <= a.bearing_deg < 360:
        ap.error("--bearing-deg is required, 0 to 359.9")
    if not 10 <= a.leg_m <= 500 or not 3 <= a.alt_m <= 100 or not 1 <= a.speed_ms <= 15:
        ap.error("out of range: --leg-m 10..500, --alt-m 3..100, --speed-ms 1..15")
    if (a.start_lat is None) != (a.start_lon is None) or (a.start_lat is None) == (a.from_fc is None):
        ap.error("give EITHER --start-lat and --start-lon OR --from-fc PORT")
    if not (a.out or a.upload):
        ap.error("nothing to do: give --out FILE and/or --upload PORT")

    link, armed = None, False
    if a.from_fc or a.upload:
        link, armed = connect(a.from_fc or a.upload)
        if a.from_fc and a.upload and a.from_fc != a.upload:
            ap.error("--from-fc and --upload must name the same port")
    if a.from_fc:
        lat, lon = position_from_fc(link)
    else:
        lat, lon = a.start_lat, a.start_lon
    if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (abs(lat) < 1e-6 and abs(lon) < 1e-6):
        raise SystemExit(f"FAIL: start position {lat}, {lon} is not a real place")

    items = build(lat, lon, a.bearing_deg, a.leg_m, a.alt_m, a.speed_ms, a.repeats)
    print(describe(items, a.leg_m, a.speed_ms, a.repeats))
    if a.out:
        with open(a.out, "w", newline="\n") as fh:
            fh.write(to_wpl(items))
        print(f"wrote {a.out}")
    if a.upload:
        if armed:
            raise SystemExit("FAIL: the vehicle is ARMED; not uploading a mission")
        upload(link, items)
        diffs = differences(items, download(link))
        if diffs:
            print("FAIL: the mission on the flight controller differs from what was sent:")
            print("\n".join("  " + d for d in diffs))
            return 1
        print(f"UPLOADED and read back: {len(items)} items match")
    return 0


if __name__ == "__main__":
    sys.exit(main())
