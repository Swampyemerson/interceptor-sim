#!/usr/bin/env python3
"""Was the target INSIDE THE PICTURE on the ticks where the seeker did not detect it?

docs/next.md 3b(ii): in-flight detection is far below static recall, and until the
vehicle's own attitude was logged (2026-09-16) the cause could not be attributed.
This projects the TRUE target position into the camera image using the logged
attitude quaternion and cross-tabulates in-frame x detected.

SCORING/FORENSIC tool: reads gt_* (ground truth). Not on any guidance path.

Geometry: world is ENU (x east, y north, z up); att_q* is body(FRD)->NED; the camera
looks along body +x, pitched up by --mount-up-deg (default 0; a tilted-mount arm MUST
be analysed with its tilt or every number below is wrong), pinhole fx=fy=539.94, 1280x960.

INSTRUMENT CHECK: --validate-tag runs the projection against AprilTag flights, where
every detection IS the target (the markerless seeker's detections are mostly own-prop
phantoms reporting ~1.5 m, so they cannot validate anything). A markerless detection
counts as REALLY seeing the target only if it lands within TRUE_DET_PX of where the
target truly projects. "CENTRAL" = within 25 deg of boresight and 120 px clear of the
top/bottom edge -- the seeker deliberately masks detections near the frame edge.

usage: inframe_attribution.py ARM_INDEX.csv [...]
       inframe_attribution.py --validate-tag TAG_FLIGHT.csv [...]
       inframe_attribution.py --self-test
"""
import csv
import math
import sys

FX = FY = 539.9363
W, H = 1280, 960
CX, CY = W / 2.0, H / 2.0


def _f(r, k):
    v = r.get(k)
    return float(v) if v not in (None, "") else None


def quat_conj_rotate(q, v):
    """Rotate NED vector v into the BODY frame: v_b = q^-1 * v * q (q = body->NED)."""
    w, x, y, z = q
    x, y, z = -x, -y, -z
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


MOUNT_UP_DEG = 0.0      # physical camera up-tilt of the arm being analysed (--mount-up-deg)


def project(row):
    """-> (u, v, in_frame, elev_deg) or None when an input is missing."""
    need = ("gt_cam_x", "gt_cam_y", "gt_cam_z", "gt_tag_x", "gt_tag_y", "gt_tag_z",
            "att_qw", "att_qx", "att_qy", "att_qz")
    vals = [_f(row, k) for k in need]
    if any(v is None for v in vals):
        return None
    cx, cy, cz, tx, ty, tz, qw, qx, qy, qz = vals
    ned = (ty - cy, tx - cx, -(tz - cz))            # ENU diff -> NED
    bx, by, bz = quat_conj_rotate((qw, qx, qy, qz), ned)   # FRD: x fwd, y right, z down
    if MOUNT_UP_DEG:
        # camera pitched UP by th about body +y: boresight = (cos th, 0, -sin th) in FRD,
        # camera-down = (sin th, 0, cos th). Re-express the body vector in that frame.
        th = math.radians(MOUNT_UP_DEG)
        bx, bz = (bx * math.cos(th) - bz * math.sin(th),
                  bx * math.sin(th) + bz * math.cos(th))
    if bx <= 0.05:
        return (None, None, False, None)
    u = CX + FX * by / bx
    v = CY + FY * bz / bx
    return (u, v, 0.0 <= u < W and 0.0 <= v < H, math.degrees(math.atan2(-bz, bx)))


YAW_BIAS_PX = 65.0      # see validate_on_tag_flights(): a constant horizontal offset
TRUE_DET_PX = 250.0     # a detection this close to the projected target IS the target
CENTRAL_BEARING_DEG = 25.0
CENTRAL_V_MARGIN_PX = 120.0
BANDS = (("8-22 m", 8.0, 22.0), ("4-8 m", 4.0, 8.0), ("<4 m", 0.0, 4.0))


def measured_pixel(row):
    mx, my, mz = _f(row, "meas_x"), _f(row, "meas_y"), _f(row, "meas_z")
    if None in (mx, my, mz) or mz <= 0:
        return None
    return CX + FX * mx / mz, CY + FY * my / mz


def validate_on_tag_flights(flight_csvs):
    """INSTRUMENT CHECK against a seeker that does not hallucinate: on AprilTag
    flights every detection IS the target, so measured-minus-projected pixel error
    is the projection's own error. 2026-09-17, two check_m4 flights: vertical 4-6 px,
    horizontal a CONSTANT +59..+85 px (about 7 deg of yaw; cause not established --
    it is subtracted as YAW_BIAS_PX, and a reader should treat horizontal claims as
    good to ~10 deg, vertical claims as good to ~1 deg)."""
    import statistics
    du, dv = [], []
    for p in flight_csvs:
        for r in csv.DictReader(open(p)):
            if r.get("detected") != "1":
                continue
            pr, mp = project(r), measured_pixel(r)
            if pr is None or pr[0] is None or mp is None:
                continue
            du.append(mp[0] - pr[0])
            dv.append(mp[1] - pr[1])
    if not du:
        print("UNCERTAIN / VACUOUS: no detected ticks with attitude in those files")
        return
    print(f"tag-flight validation, n={len(du)} detected ticks: median du {statistics.median(du):+.0f} px, "
          f"median dv {statistics.median(dv):+.0f} px")


def tabulate(arm_paths, phases=("CODED_DASH", "ENGAGE")):
    """Pre-closest-approach ticks only. Every tick lands in exactly one cell."""
    tab = {b[0]: {"out": 0, "edge": 0, "central": 0, "edge_seen": 0, "central_seen": 0,
                  "out_top": 0, "out_side": 0, "out_behind": 0} for b in BANDS}
    skipped = beyond = 0
    for arm in arm_paths:
        for a in csv.DictReader(open(arm)):
            rows = list(csv.DictReader(open(a["flight_csv_path"])))
            ranged = [(i, _f(r, "gt_range")) for i, r in enumerate(rows) if _f(r, "gt_range") is not None]
            if not ranged:
                continue
            cpa = min(ranged, key=lambda x: x[1])[0]
            for r in rows[:cpa]:
                if r.get("phase") not in phases:
                    continue
                pr = project(r)
                if pr is None:
                    skipped += 1
                    continue
                g = _f(r, "gt_range")
                band = next((b[0] for b in BANDS if b[1] <= g < b[2]), None)
                if band is None:
                    beyond += 1
                    continue
                u, v, inside, _ = pr
                c = tab[band]
                if not inside:
                    c["out"] += 1
                    c["out_behind" if u is None else "out_top" if v < 0 else "out_side"] += 1
                    continue
                bearing = math.degrees(math.atan2(u - YAW_BIAS_PX - CX, FX))
                zone = ("central" if abs(bearing) <= CENTRAL_BEARING_DEG
                        and CENTRAL_V_MARGIN_PX <= v <= H - CENTRAL_V_MARGIN_PX else "edge")
                c[zone] += 1
                mp = measured_pixel(r) if r.get("detected") == "1" else None
                if mp and math.hypot(mp[0] - u - YAW_BIAS_PX, mp[1] - v) < TRUE_DET_PX:
                    c[zone + "_seen"] += 1
    return tab, skipped, beyond


def report(result):
    tab, skipped, beyond = result
    total = sum(c["out"] + c["edge"] + c["central"] for c in tab.values())
    print(f"{total} pre-closest-approach ticks banded ({skipped} skipped: no attitude/gt; "
          f"{beyond} beyond 22 m)")
    if total == 0:
        print("UNCERTAIN / VACUOUS: nothing to attribute.")
        return
    pct = lambda a, b: f"{100.0 * a / b:.0f}%" if b else "n/a"
    print("true range | ticks | target OUTSIDE picture (top/side/behind) | at the EDGE, really seen | CENTRAL, really seen")
    for name, c in tab.items():
        n = c["out"] + c["edge"] + c["central"]
        print(f"{name:10s} | {n:5d} | {c['out']:4d} = {pct(c['out'], n)} "
              f"({c['out_top']}/{c['out_side']}/{c['out_behind']}) | "
              f"{c['edge']:4d}, seen {c['edge_seen']} = {pct(c['edge_seen'], c['edge'])} | "
              f"{c['central']:4d}, seen {c['central_seen']} = {pct(c['central_seen'], c['central'])}")


def self_test():
    ok = True

    def case(name, cond):
        nonlocal ok
        print(("  ok   " if cond else "  FAIL ") + name)
        ok = ok and cond
    base = dict(gt_cam_x="0", gt_cam_y="0", gt_cam_z="1", gt_tag_z="1")
    # level, facing NORTH (identity quaternion = body x along N): target 10 m north -> centre
    r = dict(base, gt_tag_x="0", gt_tag_y="10", att_qw="1", att_qx="0", att_qy="0", att_qz="0")
    u, v, inside, el = project(r)
    case("target dead ahead projects to the image centre", abs(u - CX) < 1e-6 and abs(v - CY) < 1e-6 and inside)
    # target to the EAST of a north-facing camera -> right half
    u, v, inside, _ = project(dict(r, gt_tag_x="3"))
    case("target to the right appears right of centre", u > CX and inside)
    # target ABOVE -> upper half (v smaller)
    u, v, inside, _ = project(dict(r, gt_tag_z="3"))
    case("target above appears above centre", v < CY and inside)
    # 45 deg nose-DOWN: a level target sits 45 deg above boresight -> off the top (half-VFOV 41.6)
    h = math.radians(-45.0) / 2.0
    u, v, inside, el = project(dict(r, att_qw=str(math.cos(h)), att_qy=str(math.sin(h))))
    case("45 deg nose-down throws a level target off the TOP edge", (not inside) and v < 0 and abs(el - 45.0) < 1e-6)
    case("missing attitude -> None, never 'level'", project(dict(r, att_qw="")) is None)
    u, v, inside, _ = project(dict(r, gt_tag_y="-10"))
    case("target behind the camera is never in frame", inside is False)
    global MOUNT_UP_DEG
    MOUNT_UP_DEG = 35.0
    up = 10.0 * math.tan(math.radians(35.0))
    u, v, inside, _ = project(dict(r, gt_tag_z=str(1.0 + up)))
    case("35 deg up-tilt puts a target 35 deg above a level vehicle at the image centre",
         abs(u - CX) < 1e-6 and abs(v - CY) < 1e-6 and inside)
    u, v, inside, _ = project(r)
    case("... and a level target then sits BELOW centre", v > CY)
    MOUNT_UP_DEG = 0.0
    return 0 if ok else 1


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    if "--mount-up-deg" in sys.argv:
        k = sys.argv.index("--mount-up-deg")
        MOUNT_UP_DEG = float(sys.argv[k + 1])
        del sys.argv[k:k + 2]
        print(f"camera mount up-tilt: {MOUNT_UP_DEG:.1f} deg")
    if len(sys.argv) > 2 and sys.argv[1] == "--validate-tag":
        validate_on_tag_flights(sys.argv[2:])
        sys.exit(0)
    if len(sys.argv) < 2:
        sys.exit("usage: inframe_attribution.py ARM_INDEX.csv [...] | --validate-tag FLIGHT.csv [...] | --self-test")
    report(tabulate(sys.argv[1:]))
