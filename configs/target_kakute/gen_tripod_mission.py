#!/usr/bin/env python3
"""gen_tripod_mission.py -- emit an ArduPilot/QGC .waypoints AUTO mission for the
tripod field-test pass matrix (docs/tripod_test_protocol.md 4).

WHAT / WHY / WHERE
------------------
WHAT: writes a Mission Planner / QGroundControl "QGC WPL 110" plain-text mission
      file that flies the TARGET quad (Kakute H7, ArduCopter -- see target.param)
      along the tripod test's APPROACH and CROSSING passes, repeatably, in AUTO.
WHY : the protocol wants the range stations GPS-repeatable pass-to-pass so every
      capture bins to the same range (docs/tripod_test_protocol.md 4.1). A scripted
      AUTO mission gives that; hand-flying does not.
WHERE: the emitted file loads via Mission Planner "Plan" > Load WP File, or QGC
       "Plan" > Open. Format ref: https://mavlink.io/en/file_formats/ (QGC WPL 110);
       columns are TAB-separated:
         INDEX  CURRENT  FRAME  COMMAND  P1  P2  P3  P4  X/LAT  Y/LON  Z/ALT  AUTOCONT

GEOMETRY (local ENU about the tripod = HOME)
--------------------------------------------
The camera on the tripod faces --bearing-deg (compass degrees from North). The
target flies IN FRONT of it:
  * APPROACH pass -- straight down the boresight, from --leg-length m out to
    --near-range m, mirroring the sim's head-on regime (protocol 4.2). One pass
    per entry in --lateral-offset (m, +right of the view) so you also sweep
    recall-vs-position-in-frame (protocol 7.2 / ADR-0076 add #18k).
  * CROSSING pass -- a lateral leg at a fixed --standoff m that transects the FOV,
    from -half-width to +half-width (protocol 4.2; the AprilTag-goes-invisible-in-
    sim aspect worth checking for real).
Waypoint speed is set per-pass with DO_CHANGE_SPEED (--speed, and --slow-speed for
the control passes). ENU->lat/lon uses a local equirectangular tangent plane about
HOME (fine at these <100 m ranges).

NOTE: the placeholder HOME is a clearly-fake default -- set --home-lat/--home-lon
to the SURVEYED tripod GPS before flying (protocol 6: the tripod position is the
range ground-truth). The script warns on the placeholder.

  python3 gen_tripod_mission.py --home-lat 37.8199 --home-lon -122.4786 \
      --bearing-deg 90 --out tripod_mission.waypoints
  python3 gen_tripod_mission.py --self-test     # offline, no hardware, exits 0/1
"""
from __future__ import annotations

import argparse
import io
import math
import sys
import tempfile
from dataclasses import dataclass

# ---- QGC WPL 110 constants -------------------------------------------------
WPL_HEADER = "QGC WPL 110"
FRAME_GLOBAL = 0            # MAV_FRAME_GLOBAL (abs alt) -- used for the HOME row
FRAME_REL_ALT = 3          # MAV_FRAME_GLOBAL_RELATIVE_ALT -- alt is AGL above home
CMD_WAYPOINT = 16          # MAV_CMD_NAV_WAYPOINT
CMD_TAKEOFF = 22           # MAV_CMD_NAV_TAKEOFF
CMD_RTL = 20               # MAV_CMD_NAV_RETURN_TO_LAUNCH
CMD_CHANGE_SPEED = 178     # MAV_CMD_DO_CHANGE_SPEED
SPEED_TYPE_GROUND = 1      # DO_CHANGE_SPEED param1: 1 = ground speed
N_COLS = 12                # QGC WPL 110 has exactly 12 tab-separated columns

# metres per degree latitude (WGS84 mean); longitude scaled by cos(lat)
M_PER_DEG_LAT = 111320.0

PLACEHOLDER_LATLON = (0.0, 0.0)


@dataclass
class MissionItem:
    seq: int
    current: int
    frame: int
    command: int
    p1: float
    p2: float
    p3: float
    p4: float
    x: float          # latitude (deg) for NAV cmds, else 0
    y: float          # longitude (deg) for NAV cmds, else 0
    z: float          # altitude (m)
    autocontinue: int

    def to_line(self) -> str:
        # Mission Planner writes ints for seq/current/frame/cmd/autocont and
        # high-precision floats for the rest. Match that so QGC/MP parse it.
        return "\t".join([
            str(self.seq), str(self.current), str(self.frame), str(self.command),
            _fmt(self.p1), _fmt(self.p2), _fmt(self.p3), _fmt(self.p4),
            _fmt(self.x, 8), _fmt(self.y, 8), _fmt(self.z, 6),
            str(self.autocontinue),
        ])


def _fmt(v: float, prec: int = 6) -> str:
    return f"{float(v):.{prec}f}"


# ---- local ENU (metres) -> lat/lon (deg) -----------------------------------
def enu_to_latlon(home_lat: float, home_lon: float, east_m: float, north_m: float):
    dlat = north_m / M_PER_DEG_LAT
    dlon = east_m / (M_PER_DEG_LAT * math.cos(math.radians(home_lat)))
    return home_lat + dlat, home_lon + dlon


def view_and_right(bearing_deg: float):
    """Unit ENU vectors for the camera VIEW direction (compass bearing) and the
    RIGHT direction (bearing + 90 deg). Returns ((vE,vN),(rE,rN))."""
    b = math.radians(bearing_deg)
    view = (math.sin(b), math.cos(b))               # bearing: E=sin, N=cos
    right = (math.sin(b + math.pi / 2), math.cos(b + math.pi / 2))
    return view, right


def _front_point(home_lat, home_lon, bearing_deg, range_m, lateral_m):
    """A point range_m in front of the camera, offset lateral_m to the right."""
    (vE, vN), (rE, rN) = view_and_right(bearing_deg)
    east = range_m * vE + lateral_m * rE
    north = range_m * vN + lateral_m * rN
    return enu_to_latlon(home_lat, home_lon, east, north)


# ---- mission construction --------------------------------------------------
def build_mission(*, home_lat, home_lon, home_alt, bearing_deg, alt,
                  leg_length, near_range, standoff, cross_halfwidth,
                  speed, slow_speed, lateral_offsets, passes):
    """Return a list[MissionItem]. seq numbers are assigned at the end."""
    items: list[MissionItem] = []

    def nav_wp(lat, lon, accept_radius=2.0):
        items.append(MissionItem(0, 0, FRAME_REL_ALT, CMD_WAYPOINT,
                                 0.0, accept_radius, 0.0, 0.0, lat, lon, alt, 1))

    def change_speed(mps):
        items.append(MissionItem(0, 0, FRAME_REL_ALT, CMD_CHANGE_SPEED,
                                 float(SPEED_TYPE_GROUND), float(mps), -1.0, 0.0,
                                 0.0, 0.0, 0.0, 1))

    # HOME (seq 0). Mission Planner requires row 0 = home, abs alt.
    items.append(MissionItem(0, 1, FRAME_GLOBAL, CMD_WAYPOINT,
                             0.0, 0.0, 0.0, 0.0, home_lat, home_lon, home_alt, 1))
    # TAKEOFF to the pass altitude (relative alt).
    items.append(MissionItem(0, 0, FRAME_REL_ALT, CMD_TAKEOFF,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alt, 1))

    if "approach" in passes:
        for lat_off in lateral_offsets:
            # full-speed approach: far -> near, straight down boresight
            change_speed(speed)
            far = _front_point(home_lat, home_lon, bearing_deg, leg_length, lat_off)
            near = _front_point(home_lat, home_lon, bearing_deg, near_range, lat_off)
            nav_wp(*far)
            nav_wp(*near)
            # one slow control pass on the same line (protocol 4.4)
            change_speed(slow_speed)
            nav_wp(*far)
            nav_wp(*near)

    if "crossing" in passes:
        # full-speed crossing at fixed standoff, left -> right across the FOV
        change_speed(speed)
        left = _front_point(home_lat, home_lon, bearing_deg, standoff, -cross_halfwidth)
        right = _front_point(home_lat, home_lon, bearing_deg, standoff, +cross_halfwidth)
        nav_wp(*left)
        nav_wp(*right)
        # slow control crossing
        change_speed(slow_speed)
        nav_wp(*left)
        nav_wp(*right)

    # RTL home.
    items.append(MissionItem(0, 0, FRAME_REL_ALT, CMD_RTL,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1))

    # assign sequential seq numbers; current flag only on seq 0
    for i, it in enumerate(items):
        it.seq = i
        it.current = 1 if i == 0 else 0
    return items


def serialize(items) -> str:
    out = io.StringIO()
    out.write(WPL_HEADER + "\n")
    for it in items:
        out.write(it.to_line() + "\n")
    return out.getvalue()


# ---- parser (used by --self-test round-trip AND as a re-usable reader) ------
def parse(text: str):
    """Parse QGC WPL 110 text -> list[MissionItem]. Raises ValueError on any
    structural problem (bad header, wrong column count, non-sequential seq)."""
    lines = [ln for ln in text.splitlines() if ln.strip() != ""]
    if not lines:
        raise ValueError("empty mission")
    if lines[0].strip() != WPL_HEADER:
        raise ValueError(f"bad header: {lines[0]!r} (expected {WPL_HEADER!r})")
    items = []
    for i, ln in enumerate(lines[1:]):
        cols = ln.split("\t")
        if len(cols) != N_COLS:
            raise ValueError(
                f"row {i}: expected {N_COLS} tab-separated columns, got {len(cols)}")
        seq = int(cols[0])
        if seq != i:
            raise ValueError(f"row {i}: non-sequential seq {seq}")
        items.append(MissionItem(
            seq=seq, current=int(cols[1]), frame=int(cols[2]), command=int(cols[3]),
            p1=float(cols[4]), p2=float(cols[5]), p3=float(cols[6]), p4=float(cols[7]),
            x=float(cols[8]), y=float(cols[9]), z=float(cols[10]),
            autocontinue=int(cols[11])))
    return items


# ---- self-test -------------------------------------------------------------
def self_test() -> int:
    failures = []

    def check(cond, msg):
        if not cond:
            failures.append(msg)

    # 1) default build, several parameter combinations all valid + round-trip
    combos = [
        dict(home_lat=37.8199, home_lon=-122.4786, home_alt=12.0, bearing_deg=90.0,
             alt=10.0, leg_length=30.0, near_range=8.0, standoff=17.0,
             cross_halfwidth=20.0, speed=9.0, slow_speed=3.0,
             lateral_offsets=[0.0], passes=["approach", "crossing"]),
        dict(home_lat=-33.87, home_lon=151.21, home_alt=0.0, bearing_deg=0.0,
             alt=15.0, leg_length=25.0, near_range=6.0, standoff=15.0,
             cross_halfwidth=18.0, speed=9.5, slow_speed=2.0,
             lateral_offsets=[-5.0, 0.0, 5.0], passes=["approach"]),
        dict(home_lat=51.5, home_lon=-0.12, home_alt=30.0, bearing_deg=215.0,
             alt=8.0, leg_length=30.0, near_range=8.0, standoff=20.0,
             cross_halfwidth=20.0, speed=10.0, slow_speed=4.0,
             lateral_offsets=[0.0], passes=["crossing"]),
    ]
    for ci, kw in enumerate(combos):
        items = build_mission(**kw)
        text = serialize(items)

        # header present
        check(text.startswith(WPL_HEADER + "\n"), f"combo{ci}: missing WPL header")

        # every line has exactly 12 tab columns
        body = [ln for ln in text.splitlines()[1:] if ln.strip()]
        check(all(len(ln.split("\t")) == N_COLS for ln in body),
              f"combo{ci}: a row does not have {N_COLS} columns")

        # seq 0 is HOME (global frame, current flag set)
        home = items[0]
        check(home.seq == 0 and home.current == 1 and home.frame == FRAME_GLOBAL
              and home.command == CMD_WAYPOINT, f"combo{ci}: bad HOME row")

        # exactly one TAKEOFF, one RTL, >=1 waypoint, >=1 change-speed
        cmds = [it.command for it in items]
        check(cmds.count(CMD_TAKEOFF) == 1, f"combo{ci}: expected 1 TAKEOFF")
        check(cmds.count(CMD_RTL) == 1, f"combo{ci}: expected 1 RTL")
        check(cmds[-1] == CMD_RTL, f"combo{ci}: mission must end with RTL")
        check(cmds.count(CMD_WAYPOINT) >= 3, f"combo{ci}: too few waypoints")
        check(cmds.count(CMD_CHANGE_SPEED) >= 1, f"combo{ci}: no DO_CHANGE_SPEED")

        # a full-speed DO_CHANGE_SPEED carries the requested speed in param2
        speeds = [it.p2 for it in items if it.command == CMD_CHANGE_SPEED]
        check(any(abs(s - kw["speed"]) < 1e-6 for s in speeds),
              f"combo{ci}: full speed {kw['speed']} not in any DO_CHANGE_SPEED")
        check(any(abs(s - kw["slow_speed"]) < 1e-6 for s in speeds),
              f"combo{ci}: slow speed {kw['slow_speed']} not in any DO_CHANGE_SPEED")

        # approach count: 2 passes (full+slow) x 2 wps x N offsets
        if "approach" in kw["passes"]:
            n_off = len(kw["lateral_offsets"])
            # crossing (if any) adds 4 waypoints; count only nav_wp with nonzero lat
            expect_appr_wp = 4 * n_off
            # sanity: waypoints excluding home should include the approach set
            nav_wps = cmds.count(CMD_WAYPOINT) - 1  # minus HOME
            check(nav_wps >= expect_appr_wp,
                  f"combo{ci}: expected >= {expect_appr_wp} approach waypoints")

        # lat/lon finite and within a sane box around home (<0.01 deg ~ 1.1 km)
        for it in items:
            if it.command in (CMD_WAYPOINT,) and it.seq != 0:
                check(math.isfinite(it.x) and math.isfinite(it.y),
                      f"combo{ci}: non-finite lat/lon")
                check(abs(it.x - kw["home_lat"]) < 0.01,
                      f"combo{ci}: waypoint lat too far from home")

        # round-trip: parse -> re-serialize -> parse again, structurally identical
        parsed = parse(text)
        check(len(parsed) == len(items), f"combo{ci}: round-trip length mismatch")
        text2 = serialize(parsed)
        check(text == text2, f"combo{ci}: round-trip serialize not stable")
        parsed2 = parse(text2)
        check(len(parsed2) == len(parsed), f"combo{ci}: second round-trip mismatch")

    # 2) parser rejects malformed input
    def expect_reject(bad, why):
        try:
            parse(bad)
        except ValueError:
            return
        failures.append(f"parser accepted bad input ({why})")

    expect_reject("NOT A HEADER\n", "bad header")
    expect_reject(WPL_HEADER + "\n0\t1\t0\t16\t0\t0\n", "too few columns")
    expect_reject(WPL_HEADER + "\n5\t1\t0\t16\t0\t0\t0\t0\t0\t0\t0\t1\n",
                  "non-sequential seq")

    # 3) geometry sanity: an approach waypoint at range r, offset 0, bearing 90
    #    (due east) must lie ~r metres EAST of home and ~same latitude.
    it = build_mission(home_lat=0.0, home_lon=0.0, home_alt=0.0, bearing_deg=90.0,
                       alt=10.0, leg_length=30.0, near_range=8.0, standoff=17.0,
                       cross_halfwidth=20.0, speed=9.0, slow_speed=3.0,
                       lateral_offsets=[0.0], passes=["approach"])
    far = next(m for m in it if m.command == CMD_WAYPOINT and m.seq != 0)
    east_m = far.y * (M_PER_DEG_LAT * math.cos(0.0))
    check(abs(east_m - 30.0) < 0.5, f"geometry: far east={east_m:.2f} m, expected 30")
    check(abs(far.x) < 1e-6, f"geometry: far lat drift {far.x}")

    # 4) file write/read on disk (the real emit path)
    with tempfile.NamedTemporaryFile("w+", suffix=".waypoints", delete=True) as fh:
        fh.write(serialize(build_mission(
            home_lat=37.0, home_lon=-122.0, home_alt=5.0, bearing_deg=45.0, alt=10.0,
            leg_length=30.0, near_range=8.0, standoff=17.0, cross_halfwidth=20.0,
            speed=9.0, slow_speed=3.0, lateral_offsets=[0.0],
            passes=["approach", "crossing"])))
        fh.flush()
        fh.seek(0)
        reread = parse(fh.read())
        check(reread[0].command == CMD_WAYPOINT and reread[0].seq == 0,
              "disk round-trip: bad home")

    if failures:
        print("SELF-TEST FAILED:")
        for f in failures:
            print("  - " + f)
        return 1
    print("SELF-TEST PASSED: mission structure + round-trip + geometry OK "
          f"({len(combos)} param combos)")
    return 0


# ---- CLI -------------------------------------------------------------------
def _parse_offsets(s: str):
    return [float(x) for x in s.split(",") if x.strip() != ""]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Emit an ArduPilot/QGC .waypoints AUTO mission for the tripod "
                    "test pass matrix (docs/tripod_test_protocol.md).")
    ap.add_argument("--self-test", action="store_true",
                    help="run offline structural + round-trip checks, exit 0/1")
    ap.add_argument("--out", default="tripod_mission.waypoints",
                    help="output .waypoints path (default: %(default)s)")
    ap.add_argument("--home-lat", type=float, default=PLACEHOLDER_LATLON[0],
                    help="SURVEYED tripod latitude (deg). REQUIRED for a real flight.")
    ap.add_argument("--home-lon", type=float, default=PLACEHOLDER_LATLON[1],
                    help="SURVEYED tripod longitude (deg). REQUIRED for a real flight.")
    ap.add_argument("--home-alt", type=float, default=0.0,
                    help="home absolute altitude AMSL (m). default 0.")
    ap.add_argument("--bearing-deg", type=float, default=0.0,
                    help="compass bearing the camera FACES (deg from North). "
                         "The target approaches from this direction. default 0 (N).")
    ap.add_argument("--alt", type=float, default=10.0,
                    help="target flight altitude AGL for the passes (m). "
                         "Co-altitude with the camera (protocol 3.1). default 10.")
    ap.add_argument("--leg-length", type=float, default=30.0,
                    help="approach start range = farthest station (m). "
                         "protocol 4.1 max station is 30 m. default 30.")
    ap.add_argument("--near-range", type=float, default=8.0,
                    help="approach end range = nearest station (m). default 8.")
    ap.add_argument("--standoff", type=float, default=17.0,
                    help="crossing-pass lateral standoff (m). protocol 4.2 ~15-20. default 17.")
    ap.add_argument("--cross-halfwidth", type=float, default=20.0,
                    help="crossing-pass half-length each side of boresight (m). default 20.")
    ap.add_argument("--speed", type=float, default=9.0,
                    help="full-speed pass ground speed (m/s). protocol 4.4 >=9. default 9.")
    ap.add_argument("--slow-speed", type=float, default=3.0,
                    help="control-pass ground speed (m/s). protocol 4.4 ~2-4. default 3.")
    ap.add_argument("--lateral-offset", default="0",
                    help="comma list of approach lateral offsets, +right of view (m). "
                         "One approach pass each -> sweeps position-in-frame. default '0'. "
                         "For NEGATIVE offsets use the = form so argparse doesn't read the "
                         "leading '-' as a flag, e.g. --lateral-offset=-5,0,5 .")
    ap.add_argument("--passes", default="approach,crossing",
                    help="comma list from {approach,crossing}. default both.")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    passes = [p.strip() for p in args.passes.split(",") if p.strip()]
    for p in passes:
        if p not in ("approach", "crossing"):
            ap.error(f"unknown pass {p!r} (choose approach and/or crossing)")

    if (args.home_lat, args.home_lon) == PLACEHOLDER_LATLON:
        print("WARNING: home is the placeholder (0,0). Set --home-lat/--home-lon to "
              "the SURVEYED tripod GPS before flying (protocol 6).", file=sys.stderr)

    items = build_mission(
        home_lat=args.home_lat, home_lon=args.home_lon, home_alt=args.home_alt,
        bearing_deg=args.bearing_deg, alt=args.alt, leg_length=args.leg_length,
        near_range=args.near_range, standoff=args.standoff,
        cross_halfwidth=args.cross_halfwidth, speed=args.speed,
        slow_speed=args.slow_speed, lateral_offsets=_parse_offsets(args.lateral_offset),
        passes=passes)
    text = serialize(items)
    # verify what we wrote parses back before touching disk
    parse(text)
    with open(args.out, "w") as fh:
        fh.write(text)
    n_wp = sum(1 for it in items if it.command == CMD_WAYPOINT) - 1
    print(f"Wrote {args.out}: {len(items)} mission items ({n_wp} waypoints, "
          f"passes={'+'.join(passes)}, speed={args.speed} m/s, "
          f"offsets={args.lateral_offset}).")
    print("Load it in Mission Planner (Plan > Load WP File) or QGC (Plan > Open).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
