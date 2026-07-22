#!/usr/bin/env python3
"""Phase A loft-dive Gazebo A/B parser (gazebo_run_recipe.md "What to measure").

Reads one or more mc_batch arm CSVs (ARM A baseline / ARM B lever) and reports,
per arm and per flight, the recipe's five decisive numbers:

 1. IN-FLIGHT REAL-DETECTION RECALL in the 8-12 m gt band (CODED_DASH+ENGAGE
    ticks; REAL = the coded_dash_summary._is_real_target criterion,
    |meas_range - gt_range|/gt_range < 0.5 -- gt for SCORING only).
 2. ACHIEVED BODY PITCH through the 8-12 m band (ulog vehicle_attitude,
    matched by start-timestamp like dash_pitch_probe.find_ulog, aligned by an
    offset+scale fit of CSV wall-t/alt_m onto ulog alt; m4-mode CSVs have no
    t_sim so wall-t + scale absorbs RTF).  Also the in-camera vertical angle
    of the target (positive = below boresight), mount-adjusted (+10 deg for
    ARM B's up10 physical wedge => vert_cam = vert_body + mount).
 3. DID THE DIVE HOLD <5 m: REAL recall in the <5 m band (out-the-bottom trap).
 4. CPA/miss + Pk@2.5 m + whether ENGAGE saw REAL detections (camera-tracked)
    -- same split as scripts/coded_dash_summary.py engagement().
 5. CLOSING-SPEED COST: ulog ground speed at the CODED_DASH->ENGAGE handoff
    tick and the dash peak speed.

Usage:
  .venv/bin/python scripts/experiments/loft_dive/parse_ab.py \
      logs/mc_loftdive_armA_line9_s123.csv [logs/mc_loftdive_armB_line9_s123.csv] \
      [--mount-deg-for ARMCSV=10]
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
HALF_HFOV_DEG = math.degrees(1.74 / 2.0)
HALF_VFOV_DEG = math.degrees(math.atan(0.75 * math.tan(0.87)))
BAND = (8.0, 12.0)
NEAR = 5.0
PK_GATE = 2.5
INFLIGHT_PHASES = ("CODED_DASH", "ENGAGE")


def fnum(r, k):
    try:
        return float(r[k])
    except (KeyError, TypeError, ValueError):
        return None


def is_det(r):
    return r.get("detected") in ("1", "True", "true")


def is_real(r):
    """coded_dash_summary._is_real_target: implied range agrees with gt within
    50%. gt is SCORING-only here (honesty boundary)."""
    mr, gr = fnum(r, "meas_range"), fnum(r, "gt_range")
    return (mr is not None and gr is not None and gr > 0
            and abs(mr - gr) / gr < 0.5)


def csv_start_utc(p):
    m = re.search(r"(\d{8}T\d{6})Z", os.path.basename(p))
    return (datetime.strptime(m.group(1), "%Y%m%dT%H%M%S")
            .replace(tzinfo=timezone.utc) if m else None)


def find_ulog(csv_utc):
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


def quat_pitch(w, x, y, z):
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def ned_to_body(q, v):
    w, x, y, z = q
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return r.T @ v


def align_wall_to_ulog(t_wall, alt_csv, tp, alt_u):
    """Fit t_ulog ~= scale * t_wall + off on altitude (scale absorbs RTF;
    m4-mode CSVs carry only wall t). Coarse grid, then refine."""
    best = (None, None, float("inf"))
    for scale in np.arange(0.80, 1.10, 0.01):
        ts = scale * t_wall
        for off in np.arange(tp[0] - ts[-1] - 5.0, tp[-1] - ts[0] + 5.0, 0.25):
            e = float(np.mean(np.abs(np.interp(ts + off, tp, alt_u) - alt_csv)))
            if e < best[2]:
                best = (scale, off, e)
    scale, off, _ = best
    for off2 in np.arange(off - 0.5, off + 0.5, 0.02):
        e = float(np.mean(np.abs(np.interp(scale * t_wall + off2, tp, alt_u)
                                 - alt_csv)))
        if e < best[2]:
            best = (scale, off2, e)
    return best


def flight_metrics(flight_csv, mount_up_deg):
    rows = list(csv.DictReader(open(flight_csv)))
    infl = [r for r in rows if r.get("phase") in INFLIGHT_PHASES]
    eng = [r for r in rows if r.get("phase") == "ENGAGE"]
    dash = [r for r in rows if r.get("phase") == "CODED_DASH"]

    # INBOUND leg only (through CPA): band stats on the closing approach --
    # post-CPA outbound ticks re-enter the range bands with the target behind
    # the interceptor and would corrupt both recall and the camera angles.
    grs = [fnum(r, "gt_range") for r in infl]
    cpa_i = min((i for i in range(len(infl)) if grs[i] is not None),
                key=lambda i: grs[i], default=None)
    inbound = infl[:cpa_i + 1] if cpa_i is not None else infl

    def bandstats(lo, hi):
        band = [r for r in inbound
                if fnum(r, "gt_range") is not None
                and lo <= fnum(r, "gt_range") <= hi]
        det = [r for r in band if is_det(r)]
        rl = [r for r in det if is_real(r)]
        return len(band), len(det), len(rl)

    b_n, b_d, b_r = bandstats(*BAND)
    n_n, n_d, n_r = bandstats(0.0, NEAR)
    e_d = sum(1 for r in eng if is_det(r))
    e_r = sum(1 for r in eng if is_det(r) and is_real(r))

    out = {
        "flight": os.path.basename(flight_csv),
        "band_ticks": b_n, "band_det": b_d, "band_real": b_r,
        "near_ticks": n_n, "near_det": n_d, "near_real": n_r,
        "eng_ticks": len(eng), "eng_det": e_d, "eng_real": e_r,
        "dash_ticks": len(dash),
        "pitch_band_med": None, "pitch_band_min": None,
        "pitch_dash_med": None, "pitch_dash_min": None,
        "vert_cam_band_med": None, "vert_cam_band_frac_in": None,
        "spd_handoff": None, "spd_peak": None,
        "align_resid": None, "align_scale": None, "ulog": None,
    }

    # --- ulog attitude/speed ---
    utc = csv_start_utc(flight_csv)
    up = find_ulog(utc) if utc else None
    if up is None:
        return out
    try:
        u = ULog(up, ["vehicle_attitude", "vehicle_local_position"])
    except Exception:
        return out
    d = {x.name: x for x in u.data_list}
    va, vlp = d.get("vehicle_attitude"), d.get("vehicle_local_position")
    if va is None or vlp is None:
        return out
    ta = va.data["timestamp"] / 1e6
    q = np.stack([va.data["q[0]"], va.data["q[1]"],
                  va.data["q[2]"], va.data["q[3]"]], axis=1)
    pitch_deg = np.degrees([quat_pitch(*qi) for qi in q])
    tp = vlp.data["timestamp"] / 1e6
    alt_u = -vlp.data["z"]
    spd_u = np.hypot(vlp.data["vx"], vlp.data["vy"])

    ticks = [r for r in rows if r.get("t") and r.get("alt_m")]
    t_wall = np.array([float(r["t"]) for r in ticks])
    a_csv = np.array([float(r["alt_m"]) for r in ticks])
    scale, off, resid = align_wall_to_ulog(t_wall, a_csv, tp, alt_u)
    out["align_resid"] = resid
    out["align_scale"] = scale
    out["ulog"] = os.path.basename(up)
    if resid is None or resid > 0.35:   # alignment failed -> no attitude claims
        return out

    def tu(r):
        return scale * float(r["t"]) + off

    def pitch_at(r):
        return float(np.interp(tu(r), ta, pitch_deg))

    def vert_cam(r):
        try:
            dx = float(r["gt_tag_x"]) - float(r["gt_cam_x"])
            dy = float(r["gt_tag_y"]) - float(r["gt_cam_y"])
            dz = float(r["gt_tag_z"]) - float(r["gt_cam_z"])
        except (KeyError, ValueError):
            return None
        qi = q[min(np.searchsorted(ta, tu(r)), len(ta) - 1)]
        v_b = ned_to_body(qi, np.array([dy, dx, -dz]))  # ENU->NED->FRD
        vert_body = math.degrees(math.atan2(v_b[2], v_b[0]))
        return vert_body + mount_up_deg  # + = below CAMERA boresight

    band_rows = [r for r in inbound if r.get("t")
                 and fnum(r, "gt_range") is not None
                 and BAND[0] <= fnum(r, "gt_range") <= BAND[1]]
    dash_rows = [r for r in dash if r.get("t")]
    if band_rows:
        pb = [pitch_at(r) for r in band_rows]
        vb = [v for v in (vert_cam(r) for r in band_rows) if v is not None]
        out["pitch_band_med"] = float(np.median(pb))
        out["pitch_band_min"] = float(np.min(pb))
        if vb:
            out["vert_cam_band_med"] = float(np.median(vb))
            out["vert_cam_band_frac_in"] = float(
                np.mean([abs(v) <= HALF_VFOV_DEG for v in vb]))
    if dash_rows:
        pd_ = [pitch_at(r) for r in dash_rows]
        out["pitch_dash_med"] = float(np.median(pd_))
        out["pitch_dash_min"] = float(np.min(pd_))
        mwin = (tp >= tu(dash_rows[0])) & (tp <= tu(dash_rows[-1]))
        if mwin.any():
            out["spd_peak"] = float(spd_u[mwin].max())
        out["spd_handoff"] = float(np.interp(tu(dash_rows[-1]), tp, spd_u))
    return out


def arm_report(arm_csv, mount_up_deg):
    rows = list(csv.DictReader(open(arm_csv)))
    print(f"\n===== ARM: {arm_csv}  (mount_up={mount_up_deg} deg) =====")
    per = []
    for r in rows:
        fp = r.get("flight_csv_path", "")
        if not fp or not os.path.exists(fp):
            print(f"  run {r.get('run_idx')}: NO flight csv ({r.get('breakoff_reason')})")
            continue
        m = flight_metrics(fp, mount_up_deg)
        m["run_idx"] = r.get("run_idx")
        m["direction"] = r.get("direction")
        m["miss_m"] = fnum(r, "miss_m")
        m["clean"] = r.get("clean")
        per.append(m)
        rec = (100.0 * m["band_real"] / m["band_ticks"]) if m["band_ticks"] else float("nan")
        nr = (100.0 * m["near_real"] / m["near_ticks"]) if m["near_ticks"] else float("nan")
        print(f"  run {m['run_idx']:>2} {m['direction']} miss={m['miss_m']}"
              f" band[8-12] {m['band_real']}/{m['band_ticks']} ({rec:.1f}%)"
              f" near[<5] {m['near_real']}/{m['near_ticks']} ({nr:.0f}%)"
              f" ENGAGE real/det/ticks {m['eng_real']}/{m['eng_det']}/{m['eng_ticks']}"
              f" pitch_band med/min {m['pitch_band_med'] and round(m['pitch_band_med'],1)}/"
              f"{m['pitch_band_min'] and round(m['pitch_band_min'],1)}"
              f" vert_cam med {m['vert_cam_band_med'] and round(m['vert_cam_band_med'],1)}"
              f" spd handoff/peak {m['spd_handoff'] and round(m['spd_handoff'],1)}/"
              f"{m['spd_peak'] and round(m['spd_peak'],1)}"
              f" align(res={m['align_resid'] and round(m['align_resid'],2)},"
              f"x{m['align_scale'] and round(m['align_scale'],2)})")

    # ---- pooled arm stats ----
    bt = sum(m["band_ticks"] for m in per)
    br = sum(m["band_real"] for m in per)
    bd = sum(m["band_det"] for m in per)
    nt = sum(m["near_ticks"] for m in per)
    nr_ = sum(m["near_real"] for m in per)
    miss = [m["miss_m"] for m in per if m["miss_m"] is not None]
    engaged_real = sum(1 for m in per if m["eng_real"] > 0)
    pk = sum(1 for m in per if m["miss_m"] is not None and m["miss_m"] <= PK_GATE)
    pooled_pitch = [m["pitch_band_med"] for m in per if m["pitch_band_med"] is not None]
    pooled_vert = [m["vert_cam_band_med"] for m in per if m["vert_cam_band_med"] is not None]
    pooled_spd = [m["spd_handoff"] for m in per if m["spd_handoff"] is not None]
    print(f"  ---- ARM POOLED (n={len(per)} flights) ----")
    print(f"  band[8-12] REAL recall: {br}/{bt} = {100.0*br/bt if bt else float('nan'):.1f}%"
          f"   (any-det {bd}/{bt} = {100.0*bd/bt if bt else float('nan'):.1f}%)")
    print(f"  near[<5]  REAL recall: {nr_}/{nt} = {100.0*nr_/nt if nt else float('nan'):.1f}%")
    print(f"  flights with REAL-detection ENGAGE: {engaged_real}/{len(per)}")
    if miss:
        print(f"  miss: med {np.median(miss):.2f} m  mean {np.mean(miss):.2f}"
              f"  min-max {min(miss):.2f}-{max(miss):.2f}  Pk@{PK_GATE}: {pk}/{len(per)}")
    if pooled_pitch:
        print(f"  pitch through band (per-flight med): med {np.median(pooled_pitch):.1f} deg"
              f"  min {min(m['pitch_band_min'] for m in per if m['pitch_band_min'] is not None):.1f}")
    if pooled_vert:
        print(f"  vert_cam through band (per-flight med): med {np.median(pooled_vert):+.1f} deg"
              f"  (+=below boresight; half-VFOV {HALF_VFOV_DEG:.1f})")
    if pooled_spd:
        print(f"  speed at handoff: med {np.median(pooled_spd):.1f} m/s"
              f"  peak med {np.median([m['spd_peak'] for m in per if m['spd_peak'] is not None]):.1f}")
    return per


def main():
    args = [a for a in sys.argv[1:]]
    mounts = {}
    for a in list(args):
        if a.startswith("--mount-deg-for"):
            args.remove(a)
            k, v = a.split("=", 1)[0].replace("--mount-deg-for", "").strip(), a.split("=", 1)[1]
            # form: --mount-deg-forCSV=DEG unsupported; use CSV=DEG pairs below
    csvs = []
    for a in args:
        if "=" in a and not os.path.exists(a):
            p, deg = a.rsplit("=", 1)
            mounts[p] = float(deg)
            csvs.append(p)
        else:
            csvs.append(a)
    for p in csvs:
        arm_report(p, mounts.get(p, 0.0))


if __name__ == "__main__":
    main()
