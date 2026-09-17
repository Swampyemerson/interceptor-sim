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
PRE_S = 0.5


def _local_slope(t: np.ndarray, x: np.ndarray, half: int = 4) -> np.ndarray:
    """Velocity as the least-squares slope over +-`half` samples on the REAL
    timestamps. Plain differencing turns the log's pose-sample jitter into
    5-75 m/s spikes, and a bad first sample poisons a replay's initial state."""
    v = np.empty_like(x)
    n = len(t)
    for i in range(n):
        a, b = max(0, i - half), min(n, i + half + 1)
        tt = t[a:b] - t[a:b].mean()
        v[i] = float(tt @ (x[a:b] - x[a:b].mean())) / max(float(tt @ tt), 1e-12)
    return v


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
    ok = [r for r in rows if r["t_sim"] != "" and r["gt_cam_x"] != ""
          and r["att_qw"] != ""]
    cmd = [r for r in ok if r["phase"] in phases and r["cmd_vn"] != ""]
    if len(cmd) < min_rows:
        return None
    # Lead-in: PRE_S of the hover BEFORE the first command, as zero-velocity
    # commands at the first commanded yaw. Without it the replay starts exactly
    # on the step and command latency is unobservable (nothing to delay).
    t_first = float(cmd[0]["t_sim"])
    lead = [dict(r, cmd_vn="0", cmd_ve="0", cmd_vd="0",
                 cmd_yaw_deg=cmd[0]["cmd_yaw_deg"])
            for r in ok if t_first - PRE_S <= float(r["t_sim"]) < t_first
            and r["phase"] not in phases]
    keep = lead + cmd
    t = np.array([_f(r, "t_sim") for r in keep])
    order = np.argsort(t, kind="stable")
    uniq = np.concatenate(([True], np.diff(t[order]) > 1e-6))
    idx = order[uniq]
    g = lambda k: np.array([_f(keep[i], k) for i in idx])  # noqa: E731
    t = g("t_sim")
    pos_n, pos_e, pos_d = g("gt_cam_y"), g("gt_cam_x"), -g("gt_cam_z")
    seg: Segment = {
        "t": t - t[0],
        "t_cmd0": np.full_like(t, t_first - t[0]),   # when the first command lands
        "cmd_vn": g("cmd_vn"), "cmd_ve": g("cmd_ve"), "cmd_vd": g("cmd_vd"),
        "cmd_yaw_deg": g("cmd_yaw_deg"),
        "pos_n": pos_n, "pos_e": pos_e, "pos_d": pos_d,
        "vel_n": _local_slope(t, pos_n), "vel_e": _local_slope(t, pos_e),
        "vel_d": _local_slope(t, pos_d),
        # logged TARGET track (scoring side only) -- lets a replay be scored
        # as an engagement, not just as a vehicle response.
        "tgt_n": g("gt_tag_y"), "tgt_e": g("gt_tag_x"), "tgt_d": -g("gt_tag_z"),
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
