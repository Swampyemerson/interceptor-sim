"""Turn logged PX4/Gazebo flights into fit segments for isim.fit_vehicle.

FITTING / SCORING SIDE ONLY: this reads gt_* (the logged true camera position)
to learn how the vehicle responded to its commands. Nothing here is reachable
from guidance. Log frame is ENU (gt_cam_x = east, _y = north, _z = up); isim is
NED. Time base is t_sim (never wall time).
"""
from __future__ import annotations

import csv
import glob
import os
from typing import Dict, Iterable, List, Optional

import numpy as np

Segment = Dict[str, np.ndarray]
PHASES = ("CODED_DASH", "ENGAGE")


def _f(row: dict, key: str) -> float:
    v = row.get(key, "")
    return float(v) if v not in ("", None) else float("nan")


def load_flight(path: str, phases: Iterable[str] = PHASES,
                min_rows: int = 40) -> Optional[Segment]:
    """One flight CSV -> one segment (commanded phases only), or None if the
    flight has no attitude columns, no sim clock, or too few commanded rows."""
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows or "att_qw" not in rows[0]:
        return None
    keep = [r for r in rows if r["phase"] in phases and r["t_sim"] != ""
            and r["cmd_vn"] != "" and r["gt_cam_x"] != "" and r["att_qw"] != ""]
    if len(keep) < min_rows:
        return None
    t = np.array([_f(r, "t_sim") for r in keep])
    order = np.argsort(t, kind="stable")
    uniq = np.concatenate(([True], np.diff(t[order]) > 1e-6))
    idx = order[uniq]
    g = lambda k: np.array([_f(keep[i], k) for i in idx])  # noqa: E731
    t = g("t_sim")
    pos_n, pos_e, pos_d = g("gt_cam_y"), g("gt_cam_x"), -g("gt_cam_z")
    seg: Segment = {
        "t": t - t[0],
        "cmd_vn": g("cmd_vn"), "cmd_ve": g("cmd_ve"), "cmd_vd": g("cmd_vd"),
        "cmd_yaw_deg": g("cmd_yaw_deg"),
        "pos_n": pos_n, "pos_e": pos_e, "pos_d": pos_d,
        "vel_n": np.gradient(pos_n, t), "vel_e": np.gradient(pos_e, t),
        "vel_d": np.gradient(pos_d, t),
        "quat_w": g("att_qw"), "quat_x": g("att_qx"),
        "quat_y": g("att_qy"), "quat_z": g("att_qz"),
    }
    if not all(np.isfinite(v).all() for v in seg.values()):
        return None
    return seg


def flights_from_batches(pattern: str = "logs/mc_fp_arm*_line9_s*.csv") -> List[str]:
    """Per-flight CSV paths named by the batch summary files."""
    out: List[str] = []
    for batch in sorted(glob.glob(pattern)):
        with open(batch, newline="") as fh:
            for row in csv.DictReader(fh):
                p = row.get("flight_csv_path", "")
                if p and os.path.exists(p) and p not in out:
                    out.append(p)
    return out


def load_all(pattern: str = "logs/mc_fp_arm*_line9_s*.csv") -> Dict[str, Segment]:
    """path -> segment for every usable flight. Reports the denominator: callers
    must print how many flights were SKIPPED, never absorb it."""
    segs: Dict[str, Segment] = {}
    for p in flights_from_batches(pattern):
        s = load_flight(p)
        if s is not None:
            segs[p] = s
    return segs


if __name__ == "__main__":
    paths = flights_from_batches()
    segs = load_all()
    n = sum(len(s["t"]) for s in segs.values())
    dur = sum(float(s["t"][-1]) for s in segs.values())
    print(f"flights named by batches: {len(paths)}  usable: {len(segs)}  "
          f"skipped: {len(paths) - len(segs)}  rows: {n}  seconds: {dur:.0f}")
    if segs:
        rate = n / dur
        vmax = max(float(np.hypot(s["vel_n"], s["vel_e"]).max()) for s in segs.values())
        print(f"mean log rate {rate:.1f} Hz, max ground speed {vmax:.1f} m/s")
