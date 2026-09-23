#!/usr/bin/env python3
"""Speed-ladder forensics: which constraint binds as target speed rises?

Analyzes the 2026-09-23 scouting flights (classic tag M4 scenario, pro-nav,
n=1 per speed -- leads, not verdicts) per the ADR-0023/0027 budget: at
handoff, the zero-effort miss (ZEM) is what the terminal must remove, and
0.5*a*t_go^2 is what it CAN remove. Everything from the per-tick CSV; gt_*
is scoring-only.
"""
import csv
import math
import sys

import numpy as np

FLIGHTS = [
    (2.0, "logs/m4_intercept_pronav_20260923T025539Z.csv"),
    (3.0, "logs/m4_intercept_pronav_20260923T055719Z.csv"),
    (4.0, None),   # resolved from the scout run logs below
    (6.0, None),
]
SCOUT_LOGS = {4.0: "logs/speed_scout_20260923/run_v4.log",
              6.0: "logs/speed_scout_20260923/run_v6.log"}


def resolve(speed, path):
    if path:
        return path
    for line in open(SCOUT_LOGS[speed]):
        if "Log written to" in line:
            return line.strip().split()[-1].replace(
                "/home/emerson/interceptor-sim/", "")
    raise SystemExit(f"no CSV for {speed}")


def f(row, k):
    v = row.get(k)
    return float(v) if v not in (None, "", "nan") else math.nan


def analyze(speed, path):
    rows = list(csv.DictReader(open(path)))
    gt = [(f(r, "t"), np.array([f(r, "gt_tag_x") - f(r, "gt_cam_x"),
                                f(r, "gt_tag_y") - f(r, "gt_cam_y"),
                                f(r, "gt_tag_z") - f(r, "gt_cam_z")]),
           r) for r in rows if r.get("gt_range")]
    rng = [(t, float(np.linalg.norm(p)), r) for t, p, r in gt]
    t_cpa, cpa, _ = min(rng, key=lambda x: x[1])
    eng = [r for r in rows if r["phase"] == "ENGAGE"]
    t_ho = f(eng[0], "t") if eng else math.nan
    # relative velocity near handoff from gt positions (finite difference)
    i_ho = next(i for i, (t, _, _) in enumerate(gt) if t >= t_ho)
    j = min(i_ho + 10, len(gt) - 1)
    dt = gt[j][0] - gt[i_ho][0]
    relv = (gt[j][1] - gt[i_ho][1]) / dt
    relp = gt[i_ho][1]
    r_ho = float(np.linalg.norm(relp))
    vc = float(-np.dot(relp, relv) / r_ho)
    t_go = r_ho / vc if vc > 0 else math.nan
    # ZEM = |relp + relv*t_go| (miss if nobody steers further)
    zem = float(np.linalg.norm(relp + relv * t_go)) if t_go == t_go else math.nan
    # capacity: horizontal accel cap -- m4 pro-nav accel limit (A_CMD cap).
    # Read the actual peak commanded accel as the honest effective cap.
    a_pk = np.nanmax([abs(f(r, "a_cmd_m_s2")) for r in eng]) if eng else math.nan
    cap = 0.5 * a_pk * t_go ** 2 if t_go == t_go else math.nan
    # perception in terminal
    det = [r for r in eng if r.get("detected") == "1"]
    cov = len(det) / len(eng) if eng else math.nan
    pre_cpa_eng = [r for r in eng if f(r, "t") <= t_cpa]
    det_pre = [r for r in pre_cpa_eng if r.get("detected") == "1"]
    last_det_t = f(det_pre[-1], "t") if det_pre else math.nan
    last_det_rng = next((v for t, v, r in rng if t >= last_det_t), math.nan) \
        if det_pre else math.nan
    lam_pk = np.nanmax([abs(f(r, "lambda_dot_deg_s")) for r in eng]) if eng else math.nan
    # bearing at last detection (deg off boresight, from meas bearing)
    b_last = math.degrees(abs(f(det_pre[-1], "bearing_rad"))) if det_pre else math.nan
    # v_perp saturation (V_PERP cap read as the observed rail)
    vperp = [abs(f(r, "v_perp_m_s")) for r in eng]
    vp_pk = np.nanmax(vperp) if vperp else math.nan
    vp_sat = np.mean([v > 0.97 * vp_pk for v in vperp]) if vperp else math.nan
    print(f"\n== {speed:.0f} m/s  ({path})  CPA {cpa:.3f} m at t={t_cpa:.2f}")
    print(f"  handoff t={t_ho:.2f}  range {r_ho:.2f} m  Vc {vc:.1f} m/s  "
          f"t_go {t_go:.2f} s")
    print(f"  ZEM at handoff {zem:.2f} m  vs capacity 0.5*a*t_go^2 = "
          f"{cap:.2f} m  (a_pk {a_pk:.1f} m/s^2)  -> "
          f"{'CAPACITY-LIMITED' if zem == zem and cap == cap and zem > cap else 'capacity OK'}")
    print(f"  terminal detection coverage {cov:.0%}  last det before CPA at "
          f"range {last_det_rng:.2f} m (t={last_det_t:.2f}, bearing "
          f"{b_last:.1f} deg)  peak LOS rate {lam_pk:.0f} deg/s")
    print(f"  v_perp peak {vp_pk:.2f} m/s, near-rail fraction {vp_sat:.0%}")


def main():
    for speed, path in FLIGHTS:
        analyze(speed, resolve(speed, path))


if __name__ == "__main__":
    main()
