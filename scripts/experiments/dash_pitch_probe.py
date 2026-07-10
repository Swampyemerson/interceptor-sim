#!/usr/bin/env python3
"""Design-review Action 1(c) / variable #19 scoping probe (READ-ONLY on data).

Question: does the body-fixed forward camera point below the horizon during
the 16 m/s DASH, exactly when handoff acquisition (2-3 consecutive
detections) must happen?

Method (no sim runs -- existing artifacts only):
 - Flight CSVs (m4_intercept_*.csv, per-tick) hold phase, t_sim, detections,
   and Gazebo-world gt_cam/gt_tag positions -- but NO vehicle attitude.
 - PX4 SITL ulogs (~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/) hold
   vehicle_attitude (quaternion, body FRD -> NED) at ~250 Hz on the SIM clock.
 - Match each flight CSV to its ulog by start timestamp (one sim per run,
   mc_batch runs sequentially), then align the two sim clocks by scanning a
   constant offset that best matches CSV alt_m against ulog -z.
 - Camera: x500_mono_cam mono_cam, ZERO mount tilt (boresight = body +x),
   HFOV 1.74 rad (99.7 deg), 1280x960 => VFOV 2*atan(0.75*tan(0.87))
   = 83.3 deg (half 41.6 deg). fx=539.9 (recorded intrinsics) agrees.
 - Per DASH tick: pitch (nose-up +), LOS-to-target elevation vs horizon
   (world ENU gt), and the LOS angles IN THE CAMERA/body frame via the full
   quaternion (so roll during weave is honest): vert = atan2(z_frd, x_frd)
   (positive = target BELOW boresight), horiz = atan2(y_frd, x_frd).

Usage:
  .venv/bin/python scripts/experiments/dash_pitch_probe.py \
      logs/mc_t21_trackgate_weave12_r2.csv [more summary CSVs...]
"""
import csv
import math
import os
import re
import sys
from datetime import datetime, timedelta, timezone

import numpy as np
from pyulog import ULog

ULOG_ROOT = os.path.expanduser(
    "~/PX4-Autopilot/build/px4_sitl_default/rootfs/log")
HALF_HFOV_DEG = math.degrees(1.74 / 2.0)                       # 49.85
HALF_VFOV_DEG = math.degrees(math.atan(0.75 * math.tan(0.87)))  # 41.63


def csv_start_utc(flight_csv_path):
    m = re.search(r"(\d{8}T\d{6})Z", os.path.basename(flight_csv_path))
    return datetime.strptime(m.group(1), "%Y%m%dT%H%M%S").replace(
        tzinfo=timezone.utc)


def find_ulog(csv_utc):
    """Nearest ulog that started at or just before the flight CSV (PX4 boots
    seconds before the m4 script starts writing). One sim at a time
    (batch-hygiene rule), so nearest-preceding within 180 s is unambiguous."""
    best = None
    for delta_day in (0, -1):
        day = (csv_utc + timedelta(days=delta_day)).strftime("%Y-%m-%d")
        d = os.path.join(ULOG_ROOT, day)
        if not os.path.isdir(d):
            continue
        for f in os.listdir(d):
            if not f.endswith(".ulg"):
                continue
            t = datetime.strptime(day + " " + f[:-4], "%Y-%m-%d %H_%M_%S"
                                  ).replace(tzinfo=timezone.utc)
            dt = (csv_utc - t).total_seconds()
            if 0 <= dt <= 180 and (best is None or dt < best[0]):
                best = (dt, os.path.join(d, f))
    return best[1] if best else None


def quat_to_pitch_roll(w, x, y, z):
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return pitch, roll


def ned_to_body(q, v):
    """Rotate NED vector v into body FRD using q = [w,x,y,z] (body->NED)."""
    w, x, y, z = q
    # R(q) rotates body->NED; body vector = R^T @ v
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return r.T @ v


def analyze_run(flight_csv, run_log_path=None):
    rows = list(csv.DictReader(open(flight_csv)))
    ticks = [r for r in rows if r.get("t_sim")]
    if not ticks:
        return None, "no t_sim (not a --handoff run)"
    ulog_path = find_ulog(csv_start_utc(flight_csv))
    if not ulog_path:
        return None, "no matching ulog"
    u = ULog(ulog_path, ["vehicle_attitude", "vehicle_local_position"])
    att = {d.name: d for d in u.data_list}
    va = att.get("vehicle_attitude")
    vlp = att.get("vehicle_local_position")
    if va is None or vlp is None:
        return None, "ulog missing attitude/local_position"
    ta = va.data["timestamp"] / 1e6
    q = np.stack([va.data["q[0]"], va.data["q[1]"],
                  va.data["q[2]"], va.data["q[3]"]], axis=1)
    tp = vlp.data["timestamp"] / 1e6
    alt_u = -vlp.data["z"]
    spd_u = np.hypot(vlp.data["vx"], vlp.data["vy"])

    # --- clock alignment: t_ulog = t_sim + off, fit on altitude ---
    t_csv = np.array([float(r["t_sim"]) for r in ticks if r.get("alt_m")])
    a_csv = np.array([float(r["alt_m"]) for r in ticks if r.get("alt_m")])
    offs = np.arange(-15.0, 15.0, 0.05)
    errs = [np.mean(np.abs(np.interp(t_csv + o, tp, alt_u) - a_csv))
            for o in offs]
    off = float(offs[int(np.argmin(errs))])
    resid = float(min(errs))

    pitch_deg = np.degrees([quat_to_pitch_roll(*qi)[0] for qi in q])
    roll_deg = np.degrees([quat_to_pitch_roll(*qi)[1] for qi in q])

    def at(t_sim, arr, tbase):
        return float(np.interp(t_sim + off, tbase, arr))

    dash, first_det, handoff_row = [], None, None
    seen_dash = False
    for r in ticks:
        ph = r["phase"]
        if ph == "DASH":
            seen_dash = True
            dash.append(r)
        elif seen_dash and handoff_row is None and ph != "DASH":
            handoff_row = r
        if first_det is None and r.get("detected") == "1" and seen_dash:
            first_det = r

    if not dash:
        return None, "no DASH phase"

    def geom(r):
        """(pitch, roll, los_el_world, vert_cam, horiz_cam) at row r; camera
        angles positive-down/right in the image. None if gt missing."""
        t = float(r["t_sim"])
        p = at(t, pitch_deg, ta)
        rl = at(t, roll_deg, ta)
        try:
            dx = float(r["gt_tag_x"]) - float(r["gt_cam_x"])   # ENU east
            dy = float(r["gt_tag_y"]) - float(r["gt_cam_y"])   # ENU north
            dz = float(r["gt_tag_z"]) - float(r["gt_cam_z"])   # ENU up
        except (ValueError, KeyError):
            return p, rl, None, None, None
        los_el = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        qi = q[np.searchsorted(ta, t + off).clip(0, len(ta) - 1)]
        v_b = ned_to_body(qi, np.array([dy, dx, -dz]))  # ENU->NED->FRD
        vert = math.degrees(math.atan2(v_b[2], v_b[0]))   # + = below boresight
        horiz = math.degrees(math.atan2(v_b[1], v_b[0]))
        return p, rl, los_el, vert, horiz

    g = [geom(r) for r in dash]
    pitches = np.array([x[0] for x in g])
    verts = np.array([x[3] for x in g if x[3] is not None])
    horizs = np.array([x[4] for x in g if x[4] is not None])
    in_vfov = np.abs(verts) <= HALF_VFOV_DEG
    in_fov = in_vfov & (np.abs(horizs) <= HALF_HFOV_DEG)
    below = verts > HALF_VFOV_DEG   # target BELOW the bottom FoV edge
    above = verts < -HALF_VFOV_DEG

    t0, t1 = float(dash[0]["t_sim"]), float(dash[-1]["t_sim"])
    m = (tp >= t0 + off) & (tp <= t1 + off)
    out = {
        "flight_csv": flight_csv,
        "ulog": ulog_path,
        "align_off_s": off, "align_resid_m": resid,
        "dash_t0": t0, "dash_dur_s": t1 - t0, "n_dash": len(dash),
        "dash_speed_max_m_s": float(spd_u[m].max()) if m.any() else float("nan"),
        "dash_speed_med_m_s": float(np.median(spd_u[m])) if m.any() else float("nan"),
        "pitch_med_deg": float(np.median(pitches)),
        "pitch_min_deg": float(pitches.min()),   # most nose-DOWN
        "vert_med_deg": float(np.median(verts)) if len(verts) else None,
        "vert_max_deg": float(verts.max()) if len(verts) else None,
        "frac_in_vfov": float(in_vfov.mean()) if len(verts) else None,
        "frac_in_fov": float(in_fov.mean()) if len(verts) else None,
        "frac_below": float(below.mean()) if len(verts) else None,
        "frac_above": float(above.mean()) if len(verts) else None,
    }
    for name, row in (("first_det", first_det), ("handoff", handoff_row)):
        if row is not None and row.get("t_sim"):
            p, rl, el, vert, horiz = geom(row)
            out[name] = {"t_sim": float(row["t_sim"]), "pitch_deg": p,
                         "roll_deg": rl, "los_el_world_deg": el,
                         "vert_cam_deg": vert, "horiz_cam_deg": horiz,
                         "range": row.get("gt_range") or row.get("meas_range")}
        else:
            out[name] = None
    return out, None


def main():
    summaries = sys.argv[1:]
    if not summaries:
        print(__doc__)
        sys.exit(2)
    all_runs = []
    for s in summaries:
        arm = os.path.basename(s)
        for row in csv.DictReader(open(s)):
            fp = row["flight_csv_path"]
            if not os.path.exists(fp):
                continue
            res, err = analyze_run(fp, row.get("run_log_path"))
            if err:
                print(f"[skip] {arm} run {row['run_idx']}: {err}")
                continue
            res["arm"] = arm
            res["run_idx"] = row["run_idx"]
            all_runs.append(res)
            fd = res["first_det"]
            print(f"{arm} run {row['run_idx']}: align_off={res['align_off_s']:+.2f}s "
                  f"(resid {res['align_resid_m']:.2f} m) "
                  f"dash {res['dash_dur_s']:.1f}s spd_max={res['dash_speed_max_m_s']:.1f} "
                  f"pitch med={res['pitch_med_deg']:+.1f} min={res['pitch_min_deg']:+.1f} deg | "
                  f"tgt vert_cam med={res['vert_med_deg']:+.1f} max={res['vert_max_deg']:+.1f} deg "
                  f"in_vfov={res['frac_in_vfov']:.0%} below={res['frac_below']:.0%} "
                  f"above={res['frac_above']:.0%}"
                  + (f" | 1st det t={fd['t_sim']:.1f}s pitch={fd['pitch_deg']:+.1f} "
                     f"vert={fd['vert_cam_deg']:+.1f} R={fd['range']}"
                     if fd else " | no det"))
    if not all_runs:
        sys.exit(1)
    print("\n=== AGGREGATE over", len(all_runs), "runs ===")
    for k in ("dash_speed_max_m_s", "pitch_med_deg", "pitch_min_deg",
              "vert_med_deg", "vert_max_deg", "frac_in_vfov", "frac_below",
              "frac_above"):
        v = np.array([r[k] for r in all_runs if r[k] is not None])
        print(f"  {k}: median={np.median(v):+.2f} min={v.min():+.2f} max={v.max():+.2f}")
    fdp = np.array([r["first_det"]["pitch_deg"] for r in all_runs if r["first_det"]])
    fdv = np.array([r["first_det"]["vert_cam_deg"] for r in all_runs
                    if r["first_det"] and r["first_det"]["vert_cam_deg"] is not None])
    if len(fdp):
        print(f"  pitch @ first detection: median={np.median(fdp):+.2f} "
              f"range [{fdp.min():+.2f}, {fdp.max():+.2f}] deg (n={len(fdp)})")
    if len(fdv):
        print(f"  target vert-in-camera @ first detection: median={np.median(fdv):+.2f} "
              f"range [{fdv.min():+.2f}, {fdv.max():+.2f}] deg "
              f"(half-VFOV {HALF_VFOV_DEG:.1f})")


if __name__ == "__main__":
    main()
