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
    from -half-width to +half-width (protocol 4.2). With the ADOPTED BEAM-FACING
    placard (docs/placard_mount.md DECISION 1) this is the tag's PRIMARY aspect
    and the block curve (a) actually lives in -- see the geometry note below.
Waypoint speed is set per-pass with DO_CHANGE_SPEED (--speed, and --slow-speed for
the control passes). ENU->lat/lon uses a local equirectangular tangent plane about
HOME (fine at these <100 m ranges).

⛔ THE DEFAULTS WERE SWEPT 2026-07-25 -- THEY USED TO MANUFACTURE A FALSE NO-GO.
This generator shipped --leg-length 30 --near-range 8 --standoff 17, with help
text citing the SUPERSEDED "protocol 4.2 ~15-20". docs/tripod_test_protocol.md
4.1 was corrected on 2026-07-24 (commit dbca8cf) to the contiguous 4..20 m
station grid precisely because the old 8-30 m ladder sits ENTIRELY OUTSIDE the
predicted 7.10 m R_decode90 for the adopted 0.35 m placard: every station would
have returned ~0% decode, curve (a) would read "no range bin sustains >=90%",
and gate_verdict would have returned FAIL -> do not buy the ~$740 interceptor,
for a purely geometric reason. The tool that emits the FLOWN mission was never
swept, so a builder following the corrected protocol would still have flown the
pre-correction geometry. Defaults are now 20 / 4 / 5.5 m, and --self-test
ASSERTS the emitted geometry against the protocol (see check_conformance) so a
future drift FAILS a check instead of flying.

PROTOCOL CONFORMANCE IS ENFORCED, NOT ADVISORY. Emitting a mission whose
geometry cannot answer curve (a) exits 2 with the reason. The legitimate
off-protocol case -- the curve-(b)/NN crossing block at a 15-20 m standoff,
which is NOT envelope-limited the same way -- is available with --off-protocol.

  python3 gen_tripod_mission.py --home-lat 37.8199 --home-lon -122.4786 \
      --bearing-deg 90 --out tripod_mission.waypoints
  # the curve-(b) NN crossing block (deliberately outside the tag envelope):
  python3 gen_tripod_mission.py --home-lat .. --home-lon .. --passes crossing \\
      --standoff 17 --off-protocol --out nn_crossing.waypoints
  python3 gen_tripod_mission.py --self-test     # offline, no hardware, exits 0/1
"""
from __future__ import annotations

import argparse
import io
import math
import os
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

# ---- PROTOCOL GEOMETRY (docs/tripod_test_protocol.md, corrected 2026-07-24) --
# Every constant below is the PROTOCOL's number, not a tunable. The self-test
# asserts the EMITTED mission against them, so a default that drifts away from
# the protocol fails a check instead of flying.
#
# 4.1 range stations, corrected because the predicted R_decode90 for the adopted
# 0.35 m placard is 7.10 m realistic (5.98 m conservative, 8.97 m full-res --
# docs/placard_sizing.md 5). The 4/6/10 m near stations are "the money stations".
PROTOCOL_STATIONS_M = (4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0)
# 4.1 verbatim: "The 4/6/10 m near stations are the money stations -- they are
# where the tag is predicted to actually decode."
PROTOCOL_MONEY_STATIONS_M = (4.0, 6.0, 10.0)
# 4.1 "Bin curve (a) in 2 m bins ... --range-bin 2 --max-range 20". The grid must
# be CONTIGUOUS on this bin: scripts/seeker/tripod_score.py r90_walk STEPS one
# bin at a time and STOPS at the first absent/underpopulated bin, returning
# UNCERTAIN (never PASS). A hole in the flown grid = a wasted field day.
PROTOCOL_RANGE_BIN_M = 2.0
# 4.2 "Use a ~5-6 m standoff for the dedicated tag-envelope crossing block"
# (inside the predicted decode envelope). With the beam-facing placard
# (docs/placard_mount.md DECISION 1) this block IS curve (a).
PROTOCOL_TAG_CROSS_STANDOFF_M = (5.0, 6.0)
# 4.2 "the old ~15-20 m standoff is retained ONLY for the NN/curve-(b) crossing
# passes, where detection is not envelope-limited the same way." Reaching it
# requires --off-protocol, so it can never be flown for curve (a) by accident.
PROTOCOL_NN_CROSS_STANDOFF_M = (15.0, 20.0)
# 4.4 "pushing toward the eventual >=9 m/s engagement regime"; 4.4 control pass
# "~2-4 m/s".
PROTOCOL_FULL_SPEED_MIN_MPS = 9.0
PROTOCOL_SLOW_SPEED_BAND_MPS = (2.0, 4.0)
# docs/placard_mount.md 8, HARD limit: at the 90 deg (nose-on) placard index the
# panel is normal to the flow -- 11.8 N of drag, ANGLE_MAX reached at 5.83 m/s,
# and the trim pitch itself throws the tag outside the incidence cone. The
# nose-on block is the ONLY source of approach-aspect decodes (and therefore of
# autolabel_from_apriltag.py's approach-aspect training frames), so the tool
# refuses to emit one above the cap.
PROTOCOL_NOSE_ON_INDEX_DEG = 90.0
PROTOCOL_NOSE_ON_MAX_MPS = 4.0

# Emitted-geometry tolerance (m). The equirectangular round-trip is exact to
# <1 mm at these ranges; this only absorbs float formatting in the .waypoints
# file (6 decimal places of latitude ~ 0.1 m).
GEOM_TOL_M = 0.25


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


def latlon_to_enu(home_lat: float, home_lon: float, lat: float, lon: float):
    """Inverse of enu_to_latlon (same tangent plane). Used to read the EMITTED
    mission's geometry back out of the .waypoints file, so conformance is
    checked against what will actually be FLOWN, not against the arguments that
    were meant to produce it."""
    north = (lat - home_lat) * M_PER_DEG_LAT
    east = (lon - home_lon) * (M_PER_DEG_LAT * math.cos(math.radians(home_lat)))
    return east, north


def waypoint_geometry(items, home_lat, home_lon, bearing_deg):
    """Every NAV waypoint (excluding HOME/seq 0) as a dict with the three numbers
    the protocol is written in:

      along_m    distance along the camera boresight (= the RANGE STATION for an
                 approach waypoint, and the STANDOFF for a crossing waypoint)
      lateral_m  offset +right of the view
      range_m    slant range from the tripod = hypot(along, lateral)

    This is the read-back the self-test asserts on. `alt` is deliberately not
    folded into range_m: the protocol flies the target CO-ALTITUDE with the
    camera (3.1), so the mission's AGL alt is the camera's height, not a
    vertical separation."""
    (vE, vN), (rE, rN) = view_and_right(bearing_deg)
    out = []
    for it in items:
        if it.command != CMD_WAYPOINT or it.seq == 0:
            continue
        e, n = latlon_to_enu(home_lat, home_lon, it.x, it.y)
        along = e * vE + n * vN
        lateral = e * rE + n * rN
        out.append({"seq": it.seq, "along_m": along, "lateral_m": lateral,
                    "range_m": math.hypot(along, lateral)})
    return out


def check_conformance(geom, *, passes, speed, slow_speed, placard_index_deg,
                      stations=PROTOCOL_STATIONS_M, bin_m=PROTOCOL_RANGE_BIN_M,
                      tag_standoff=PROTOCOL_TAG_CROSS_STANDOFF_M):
    """Does this EMITTED geometry match docs/tripod_test_protocol.md 4?

    Returns (violations, warnings) -- both lists of strings. A violation means
    the mission cannot answer curve (a) (the ~$740 gate); main() exits 2 on one
    unless --off-protocol is given. This is the check that had TEETH missing:
    the old --self-test verified mission STRUCTURE and round-trip, so the
    pre-correction 8/17/30 m defaults passed it green.

    Checked against the EMITTED waypoints (waypoint_geometry), not the args."""
    v, w = [], []
    lo_station, hi_station = float(min(stations)), float(max(stations))

    appr = [g for g in geom if abs(g["lateral_m"]) <= 0.5 * bin_m]
    cross = [g for g in geom if abs(g["lateral_m"]) > 0.5 * bin_m]

    if "approach" in passes:
        if not appr:
            v.append("approach pass requested but NO on-boresight waypoint was "
                     "emitted")
        else:
            near = min(g["range_m"] for g in appr)
            far = max(g["range_m"] for g in appr)
            if near > lo_station + GEOM_TOL_M:
                v.append(
                    f"approach NEAR end is {near:.1f} m but protocol 4.1's "
                    f"nearest station is {lo_station:.0f} m -- the "
                    f"{'/'.join(f'{s:.0f}' for s in PROTOCOL_MONEY_STATIONS_M)} m "
                    f"MONEY STATIONS (where the tag is predicted to actually decode, "
                    f"R_decode90 ~7.10 m) would never be flown")
            if far < hi_station - GEOM_TOL_M:
                v.append(
                    f"approach FAR end is {far:.1f} m but protocol 4.1 wants the "
                    f"grid out to {hi_station:.0f} m -- without it the >=90% walk "
                    f"ends on `end_of_data`, which is bounded by what was never "
                    f"shot and returns UNCERTAIN, never PASS")
            # Contiguity on the scorer's own 2 m bin: a continuous inbound leg
            # fills every bin between its endpoints, so the test is that the leg
            # SPANS the whole station grid.
            n_bins = int(round((hi_station - lo_station) / bin_m))
            missing = [lo_station + k * bin_m for k in range(n_bins)
                       if not (near <= lo_station + k * bin_m + GEOM_TOL_M
                               and far >= lo_station + (k + 1) * bin_m - GEOM_TOL_M)]
            if missing:
                v.append(
                    f"the approach leg does not cover the contiguous {bin_m:.0f} m "
                    f"bins {[f'{m:.0f}' for m in missing]} -- r90_walk STOPS at the "
                    f"first absent bin (tripod_score.r90_walk) and the verdict is "
                    f"UNCERTAIN")

    if "crossing" in passes:
        if not cross:
            v.append("crossing pass requested but NO off-boresight waypoint was "
                     "emitted")
        else:
            stand = sorted({round(g["along_m"], 3) for g in cross})
            lo_s, hi_s = tag_standoff
            bad = [s for s in stand if not (lo_s - GEOM_TOL_M <= s <= hi_s + GEOM_TOL_M)]
            if bad:
                v.append(
                    f"crossing standoff {['%.1f' % s for s in bad]} m is outside "
                    f"protocol 4.2's {lo_s:.0f}-{hi_s:.0f} m TAG-ENVELOPE band. "
                    f"The adopted placard is BEAM-FACING (placard_mount DECISION 1), "
                    f"so crossing is the tag's PRIMARY aspect and this block IS "
                    f"curve (a) -- flown at the old 15-20 m NN standoff it sits "
                    f"outside the 7.10 m envelope and returns structural zeros. "
                    f"Use --off-protocol only for the curve-(b)/NN crossing block")

    if speed < PROTOCOL_FULL_SPEED_MIN_MPS:
        w.append(f"--speed {speed:.1f} m/s is below protocol 4.4's >=9 m/s "
                 f"full-speed regime -- the motion-blur read is the one thing "
                 f"this day can measure that sim cannot")
    lo_sl, hi_sl = PROTOCOL_SLOW_SPEED_BAND_MPS
    if not (lo_sl <= slow_speed <= hi_sl):
        w.append(f"--slow-speed {slow_speed:.1f} m/s is outside protocol 4.4's "
                 f"{lo_sl:.0f}-{hi_sl:.0f} m/s control band")

    if placard_index_deg is not None:
        idx = abs(float(placard_index_deg))
        if idx >= PROTOCOL_NOSE_ON_INDEX_DEG - 1e-9 and speed > PROTOCOL_NOSE_ON_MAX_MPS:
            v.append(
                f"--placard-index-deg {placard_index_deg:.0f} (nose-on) at "
                f"{speed:.1f} m/s breaks the HARD <= {PROTOCOL_NOSE_ON_MAX_MPS:.0f} m/s "
                f"cap in docs/placard_mount.md 8: nose-on the 450 mm panel makes "
                f"11.8 N of drag, ANGLE_MAX is reached at 5.83 m/s, and the trim "
                f"pitch throws the tag outside its own incidence cone")
        if 1e-9 < idx < PROTOCOL_NOSE_ON_INDEX_DEG - 1e-9 and "approach" in passes:
            w.append(f"--placard-index-deg {placard_index_deg:.0f} with approach "
                     f"passes: only the 90 deg index yields approach-aspect "
                     f"decodes at all (placard_mount 3.6)")
    return v, w


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

    # 1) default build, several parameter combinations all valid + round-trip.
    #    SWEPT 2026-07-25 to the corrected protocol 4.1 grid (4..20 m) and the
    #    protocol 4.2 tag-envelope crossing standoff (5-6 m). These presets are
    #    also the fixtures the conformance assertions below run on, so a preset
    #    that drifts back to the 8/17/30 m geometry fails the self-test.
    combos = [
        dict(home_lat=37.8199, home_lon=-122.4786, home_alt=12.0, bearing_deg=90.0,
             alt=10.0, leg_length=20.0, near_range=4.0, standoff=5.5,
             cross_halfwidth=20.0, speed=9.0, slow_speed=3.0,
             lateral_offsets=[0.0], passes=["approach", "crossing"]),
        dict(home_lat=-33.87, home_lon=151.21, home_alt=0.0, bearing_deg=0.0,
             alt=15.0, leg_length=20.0, near_range=4.0, standoff=6.0,
             cross_halfwidth=18.0, speed=9.5, slow_speed=2.0,
             lateral_offsets=[-5.0, 0.0, 5.0], passes=["approach"]),
        dict(home_lat=51.5, home_lon=-0.12, home_alt=30.0, bearing_deg=215.0,
             alt=8.0, leg_length=20.0, near_range=4.0, standoff=5.0,
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
                       alt=10.0, leg_length=20.0, near_range=4.0, standoff=5.5,
                       cross_halfwidth=20.0, speed=9.0, slow_speed=3.0,
                       lateral_offsets=[0.0], passes=["approach"])
    far = next(m for m in it if m.command == CMD_WAYPOINT and m.seq != 0)
    east_m = far.y * (M_PER_DEG_LAT * math.cos(0.0))
    check(abs(east_m - 20.0) < 0.5, f"geometry: far east={east_m:.2f} m, expected 20")
    check(abs(far.x) < 1e-6, f"geometry: far lat drift {far.x}")

    # 4) file write/read on disk (the real emit path)
    with tempfile.NamedTemporaryFile("w+", suffix=".waypoints", delete=True) as fh:
        fh.write(serialize(build_mission(
            home_lat=37.0, home_lon=-122.0, home_alt=5.0, bearing_deg=45.0, alt=10.0,
            leg_length=20.0, near_range=4.0, standoff=5.5, cross_halfwidth=20.0,
            speed=9.0, slow_speed=3.0, lateral_offsets=[0.0],
            passes=["approach", "crossing"])))
        fh.flush()
        fh.seek(0)
        reread = parse(fh.read())
        check(reread[0].command == CMD_WAYPOINT and reread[0].seq == 0,
              "disk round-trip: bad home")

    # ---------------------------------------------------------------------
    # 5) PROTOCOL CONFORMANCE -- the teeth (2026-07-25).
    #
    # WHAT WAS WRONG WITH THE OLD SELF-TEST: it checked mission STRUCTURE and
    # round-trip only, so it passed green while the tool shipped the
    # pre-correction 8/17/30 m geometry that manufactures a FALSE NO-GO on the
    # ~$740 order. A self-test that cannot fail on the defect it is guarding is
    # not a check. These assertions run on the EMITTED waypoints, read back
    # through waypoint_geometry, and they FAIL if a default drifts.
    # ---------------------------------------------------------------------
    def _emit_geom(**kw):
        home = (kw["home_lat"], kw["home_lon"])
        items = build_mission(**kw)
        # go through serialize->parse so the check sees the FILE, not the objects
        return waypoint_geometry(parse(serialize(items)), home[0], home[1],
                                 kw["bearing_deg"])

    # (5a) THE ARGPARSE DEFAULTS THEMSELVES conform. This is the assertion that
    # would have caught the stale 8/17/30 m defaults, and it reads them out of
    # the parser rather than restating them.
    dflt = main_parser().parse_args([])
    dgeom = _emit_geom(home_lat=37.8199, home_lon=-122.4786, home_alt=0.0,
                       bearing_deg=dflt.bearing_deg, alt=dflt.alt,
                       leg_length=dflt.leg_length, near_range=dflt.near_range,
                       standoff=dflt.standoff, cross_halfwidth=dflt.cross_halfwidth,
                       speed=dflt.speed, slow_speed=dflt.slow_speed,
                       lateral_offsets=_parse_offsets(dflt.lateral_offset),
                       passes=[p.strip() for p in dflt.passes.split(",")])
    dviol, _dw = check_conformance(
        dgeom, passes=["approach", "crossing"], speed=dflt.speed,
        slow_speed=dflt.slow_speed, placard_index_deg=None)
    check(not dviol,
          "CLI DEFAULTS conform to protocol 4.1/4.2: " + "; ".join(dviol))

    # (5b) the emitted numbers ARE the protocol's stations, not merely "valid".
    dappr = [g["range_m"] for g in dgeom if abs(g["lateral_m"]) <= 1.0]
    dcross = sorted({round(g["along_m"], 3) for g in dgeom
                     if abs(g["lateral_m"]) > 1.0})
    check(dappr and abs(min(dappr) - min(PROTOCOL_STATIONS_M)) < GEOM_TOL_M
          and abs(max(dappr) - max(PROTOCOL_STATIONS_M)) < GEOM_TOL_M,
          f"emitted approach leg spans {min(dappr):.2f}..{max(dappr):.2f} m, "
          f"protocol stations {min(PROTOCOL_STATIONS_M):.0f}.."
          f"{max(PROTOCOL_STATIONS_M):.0f} m")
    check(dcross and all(PROTOCOL_TAG_CROSS_STANDOFF_M[0] - GEOM_TOL_M <= s
                         <= PROTOCOL_TAG_CROSS_STANDOFF_M[1] + GEOM_TOL_M
                         for s in dcross),
          f"emitted crossing standoff {dcross} m is inside the protocol 4.2 "
          f"tag-envelope band {list(PROTOCOL_TAG_CROSS_STANDOFF_M)} m")

    # (5c) THE MUTANT MUST FAIL. Re-emit with the exact pre-2026-07-25 defaults
    # and assert the checker CATCHES all three (near end outside the envelope,
    # far end short of 20 m, crossing standoff in the NN band). If this block
    # ever reports "no violations", the conformance check has gone toothless.
    stale = _emit_geom(home_lat=37.8199, home_lon=-122.4786, home_alt=0.0,
                       bearing_deg=90.0, alt=10.0, leg_length=30.0,
                       near_range=8.0, standoff=17.0, cross_halfwidth=20.0,
                       speed=9.0, slow_speed=3.0, lateral_offsets=[0.0],
                       passes=["approach", "crossing"])
    sviol, _sw = check_conformance(stale, passes=["approach", "crossing"],
                                   speed=9.0, slow_speed=3.0,
                                   placard_index_deg=None)
    check(len(sviol) >= 2
          and any("MONEY STATIONS" in m for m in sviol)
          and any("TAG-ENVELOPE" in m for m in sviol),
          f"the PRE-CORRECTION 8/17/30 m geometry is REJECTED "
          f"({len(sviol)} violations: "
          f"{[m.split(' -- ')[0][:48] for m in sviol]})")

    # (5d) each stale axis alone is caught (no single check is carrying them all)
    only_near = _emit_geom(home_lat=0.0, home_lon=0.0, home_alt=0.0,
                           bearing_deg=0.0, alt=10.0, leg_length=20.0,
                           near_range=8.0, standoff=5.5, cross_halfwidth=20.0,
                           speed=9.0, slow_speed=3.0, lateral_offsets=[0.0],
                           passes=["approach"])
    v_near, _ = check_conformance(only_near, passes=["approach"], speed=9.0,
                                  slow_speed=3.0, placard_index_deg=None)
    check(any("MONEY STATIONS" in m for m in v_near),
          "an 8 m near end alone is REJECTED (the money stations)")
    short = _emit_geom(home_lat=0.0, home_lon=0.0, home_alt=0.0, bearing_deg=0.0,
                       alt=10.0, leg_length=12.0, near_range=4.0, standoff=5.5,
                       cross_halfwidth=20.0, speed=9.0, slow_speed=3.0,
                       lateral_offsets=[0.0], passes=["approach"])
    v_short, _ = check_conformance(short, passes=["approach"], speed=9.0,
                                   slow_speed=3.0, placard_index_deg=None)
    check(any("end_of_data" in m for m in v_short) or any("bins" in m for m in v_short),
          "a 12 m far end alone is REJECTED (r90 walk would end on missing data)")
    v_nn, _ = check_conformance(
        _emit_geom(home_lat=0.0, home_lon=0.0, home_alt=0.0, bearing_deg=0.0,
                   alt=10.0, leg_length=20.0, near_range=4.0, standoff=17.0,
                   cross_halfwidth=20.0, speed=9.0, slow_speed=3.0,
                   lateral_offsets=[0.0], passes=["crossing"]),
        passes=["crossing"], speed=9.0, slow_speed=3.0, placard_index_deg=None)
    check(any("TAG-ENVELOPE" in m for m in v_nn),
          "a 17 m crossing standoff alone is REJECTED (curve-(a) block)")

    # (5e) the placard_mount 8 HARD nose-on cap is enforced on the emitted block
    v_nose, _ = check_conformance(dgeom, passes=["approach"], speed=9.0,
                                  slow_speed=3.0, placard_index_deg=90.0)
    v_nose_ok, _ = check_conformance(dgeom, passes=["approach"], speed=4.0,
                                     slow_speed=3.0, placard_index_deg=90.0)
    check(any("nose-on" in m for m in v_nose) and not v_nose_ok,
          "nose-on (90 deg index) at 9 m/s REJECTED, at 4 m/s allowed "
          "(docs/placard_mount.md 8 HARD cap)")

    # (5f) end-to-end through main(): conforming defaults exit 0, the stale
    # geometry exits 2, and --off-protocol lets the NN block through.
    # (The next three invocations print their own REFUSED / OFF-PROTOCOL blocks
    # to stderr ON PURPOSE -- that output is the behaviour under test.)
    print("[self-test] --- exercising main(): expect one REFUSED block and one "
          "OFF-PROTOCOL block below ---")
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "m.waypoints")
        rc_ok = main(["--home-lat", "37.8199", "--home-lon", "-122.4786",
                      "--bearing-deg", "90", "--out", out])
        rc_bad = main(["--home-lat", "37.8199", "--home-lon", "-122.4786",
                       "--bearing-deg", "90", "--near-range", "8",
                       "--standoff", "17", "--leg-length", "30",
                       "--out", os.path.join(td, "bad.waypoints")])
        rc_nn = main(["--home-lat", "37.8199", "--home-lon", "-122.4786",
                      "--bearing-deg", "90", "--passes", "crossing",
                      "--standoff", "17", "--off-protocol",
                      "--out", os.path.join(td, "nn.waypoints")])
        check(rc_ok == 0 and os.path.exists(out),
              "main() with the shipped defaults exits 0 and writes the mission")
        check(rc_bad == 2 and not os.path.exists(os.path.join(td, "bad.waypoints")),
              f"main() with the PRE-CORRECTION geometry exits 2 and writes "
              f"NOTHING (got rc={rc_bad})")
        check(rc_nn == 0 and os.path.exists(os.path.join(td, "nn.waypoints")),
              "main() --off-protocol emits the curve-(b)/NN 17 m crossing block")

    if failures:
        print("SELF-TEST FAILED:")
        for f in failures:
            print("  - " + f)
        return 1
    print("SELF-TEST PASSED: mission structure + round-trip + geometry + "
          f"PROTOCOL CONFORMANCE of the emitted waypoints OK ({len(combos)} param "
          f"combos; stations {min(PROTOCOL_STATIONS_M):.0f}-"
          f"{max(PROTOCOL_STATIONS_M):.0f} m, crossing standoff "
          f"{PROTOCOL_TAG_CROSS_STANDOFF_M[0]:.0f}-"
          f"{PROTOCOL_TAG_CROSS_STANDOFF_M[1]:.0f} m)")
    return 0


# ---- CLI -------------------------------------------------------------------
def _parse_offsets(s: str):
    return [float(x) for x in s.split(",") if x.strip() != ""]


def main_parser() -> argparse.ArgumentParser:
    """The CLI parser, exposed so --self-test can read the SHIPPED DEFAULTS out
    of it and assert them against the protocol (see self_test 5a). Restating the
    defaults inside the test would test the restatement, not the tool."""
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
    ap.add_argument("--leg-length", type=float, default=20.0,
                    help="approach start range = farthest station (m). protocol "
                         "4.1's CORRECTED grid is 4..20 m contiguous on a 2 m "
                         "bin; 25/30 m were dropped as certain zeros that only "
                         "cost field time. default 20.")
    ap.add_argument("--near-range", type=float, default=4.0,
                    help="approach end range = nearest station (m). protocol "
                         "4.1: the 4/6/10 m near stations are THE MONEY STATIONS "
                         "-- the predicted R_decode90 is 7.10 m, so an 8 m near "
                         "end (the old default) sits outside the whole envelope. "
                         "default 4.")
    ap.add_argument("--standoff", type=float, default=5.5,
                    help="crossing-pass lateral standoff (m). protocol 4.2: use "
                         "5-6 m for the TAG-ENVELOPE crossing block (the beam-"
                         "facing placard makes crossing the tag's primary "
                         "aspect). The old 15-20 m standoff is for the curve-(b)/"
                         "NN block ONLY and needs --off-protocol. default 5.5.")
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
    ap.add_argument("--placard-index-deg", type=float, default=None,
                    help="the placard's 15-deg-indexable mount angle for this "
                         "block (0 = beam/default, 90 = nose-on). Given, it is "
                         "CHECKED against docs/placard_mount.md 8's HARD "
                         "<=4 m/s nose-on cap. Record the same number on the "
                         "pass card -- without it no frame's incidence can be "
                         "reconstructed offline (protocol 4.2b).")
    ap.add_argument("--off-protocol", action="store_true",
                    help="emit a mission that VIOLATES docs/tripod_test_protocol.md "
                         "4 anyway. The one legitimate use is the curve-(b)/NN "
                         "crossing block at a 15-20 m standoff, which is not "
                         "envelope-limited the same way. Without it a "
                         "non-conforming geometry exits 2.")
    return ap


def main(argv=None) -> int:
    ap = main_parser()
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

    # PROTOCOL CONFORMANCE ON THE EMITTED GEOMETRY -- read back out of the
    # waypoints that will actually be flown, not off the arguments.
    geom = waypoint_geometry(parse(text), args.home_lat, args.home_lon,
                             args.bearing_deg)
    viol, warns = check_conformance(
        geom, passes=passes, speed=args.speed, slow_speed=args.slow_speed,
        placard_index_deg=args.placard_index_deg)
    for wmsg in warns:
        print(f"WARNING: {wmsg}", file=sys.stderr)
    if viol:
        head = ("OFF-PROTOCOL (allowed by --off-protocol)" if args.off_protocol
                else "REFUSED -- this mission cannot answer curve (a)")
        print(f"{head}: {len(viol)} protocol violation(s) vs "
              f"docs/tripod_test_protocol.md 4:", file=sys.stderr)
        for m in viol:
            print(f"  - {m}", file=sys.stderr)
        if not args.off_protocol:
            print("  Fix the geometry, or pass --off-protocol if this is the "
                  "deliberate curve-(b)/NN block.", file=sys.stderr)
            return 2

    with open(args.out, "w") as fh:
        fh.write(text)
    n_wp = sum(1 for it in items if it.command == CMD_WAYPOINT) - 1
    appr = [g["range_m"] for g in geom if abs(g["lateral_m"]) <= 1.0]
    cross = sorted({round(g["along_m"], 1) for g in geom if abs(g["lateral_m"]) > 1.0})
    print(f"Wrote {args.out}: {len(items)} mission items ({n_wp} waypoints, "
          f"passes={'+'.join(passes)}, speed={args.speed} m/s, "
          f"offsets={args.lateral_offset}).")
    print(f"  emitted geometry: "
          + (f"approach {min(appr):.1f}..{max(appr):.1f} m" if appr else "no approach")
          + (f" | crossing standoff {cross} m" if cross else "")
          + (f" | placard index {args.placard_index_deg:.0f} deg"
             if args.placard_index_deg is not None else "")
          + (f"  [PROTOCOL OK]" if not viol else "  [OFF-PROTOCOL]"))
    print("Load it in Mission Planner (Plan > Load WP File) or QGC (Plan > Open).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
