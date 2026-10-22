#!/usr/bin/env python3
"""ACTUATION term of the intercept error budget -- measured from flown logs only.

WHAT IT DOES (plain English): for every flight in an mc_* arm summary it compares
what the guidance ASKED the airframe to do (cmd_vn/cmd_ve/cmd_vd) against what the
airframe ACTUALLY did (differentiated ground-truth position), and converts the gap
into metres of miss with explicit counterfactuals.

TIME BASE (important): coded-dash logs written before 2026-07-25 have a BLANK
t_sim column, and the `t` column is WALL time while the physics runs on SIM time
(RTF ~0.75-0.80 here).  Differentiating position against wall time therefore
under-reads every speed by the RTF factor.  This script instead RECONSTRUCTS sim
time from the target mover, which runs on /clock at a known constant speed
(scripts/m4_target_mover.py): t_sim = (gt_tag displacement along its velocity) / |v_target|.
Validated: steady-state achieved |v| = 15.976 +/- 0.531 m/s against a commanded
16.000 (scale 0.9985) on the 8 headline flights.

Usage:  python3 scripts/experiments/actuation_error_budget.py [arm_name ...]
        (arm_name = basename of a logs/mc_*.csv arm summary, no extension)
"""
import csv, math, os, sys, statistics as S
import numpy as np

LOGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs")
LOGS = os.path.normpath(LOGS) + os.sep
ALT_REF_M = 0.5          # scripts/m4_intercept.py
PHASES = ("CODED_DASH", "ENGAGE", "TERMINAL", "ACQUIRE")


def _load(p):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def _f(row, key):
    v = row.get(key, "")
    if v in (None, ""):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def sim_time(rows, tvx, tvy):
    """Reconstruct SIM seconds from the /clock-driven constant-velocity mover."""
    spd = math.hypot(tvx, tvy)
    base, out = None, []
    for r in rows:
        tx, ty = _f(r, "gt_tag_x"), _f(r, "gt_tag_y")
        if tx is None:
            out.append(None); continue
        if base is None:
            base = (tx, ty)
        out.append(((tx - base[0]) * tvx + (ty - base[1]) * tvy) / spd / spd)
    return out


def cpa(pathfn, t, T, n=4001):
    tt = np.linspace(t.min(), t.max(), n)
    Tf = np.array([np.interp(tt, t, T[:, k]) for k in range(3)]).T
    d = np.linalg.norm(pathfn(tt) - Tf, axis=1)
    i = int(np.argmin(d))
    return float(d[i]), float(tt[i])


def flight(path, tvx, tvy, tmax=8.0, cpa_tmax=3.0):
    rows = _load(path)
    ts = sim_time(rows, tvx, tvy)
    idx = [i for i, r in enumerate(rows)
           if r["phase"] in PHASES and _f(r, "cmd_vn") is not None and ts[i] is not None]
    if len(idx) < 10:
        return None
    t = np.array([ts[i] - ts[idx[0]] for i in idx])
    P = np.array([[_f(rows[i], "gt_cam_x"), _f(rows[i], "gt_cam_y"), _f(rows[i], "gt_cam_z")] for i in idx], float)
    T = np.array([[_f(rows[i], "gt_tag_x"), _f(rows[i], "gt_tag_y"), _f(rows[i], "gt_tag_z")] for i in idx], float)
    alt = np.array([_f(rows[i], "alt_m") if _f(rows[i], "alt_m") is not None else np.nan for i in idx], float)
    cmdh = np.array([math.hypot(_f(rows[i], "cmd_vn"), _f(rows[i], "cmd_ve")) for i in idx], float)
    vperp = np.array([_f(rows[i], "v_perp_m_s") if _f(rows[i], "v_perp_m_s") is not None else np.nan for i in idx], float)
    m = (t <= tmax) & np.isfinite(alt)
    t, P, T, alt, cmdh, vperp = t[m], P[m], T[m], alt[m], cmdh[m], vperp[m]

    # the CPA search stays inside the engagement window; the speed profile uses the
    # whole record so the steady-state sanity check has data.
    c = t <= cpa_tmax
    tc, Pc, Tc, altc = t[c], P[c], T[c], alt[c]

    def fnc(Z):
        def f(tt):
            return np.array([np.interp(tt, tc, Z[:, k]) for k in range(3)]).T
        return f

    miss_true, t_cpa = cpa(fnc(Pc), tc, Tc)
    # (A) ACTUATION counterfactual: the airframe holds the altitude it was told to hold.
    Pa = Pc.copy(); Pa[:, 2] = Pc[:, 2] - (altc - ALT_REF_M)
    miss_altheld, _ = cpa(fnc(Pa), tc, Tc)
    # (B) floor: zero vertical error at all (camera exactly co-altitude with the aimpoint)
    Pz = Pc.copy(); Pz[:, 2] = Tc[:, 2]
    miss_novert, _ = cpa(fnc(Pz), tc, Tc)
    # Achieved speed profile.  NOTE: a single-tick difference is NOT usable here --
    # gt_tag (which sets the reconstructed sim clock) is quantized to the mover's own
    # ~20 Hz update while the guidance tick is ~25 Hz, so consecutive dt alternate
    # between ~0.024 s and ~0.048 s and a one-tick derivative is badly biased (it read
    # a_eff = 6.3 m/s^2 vs the correct 9.0).  Use a least-squares slope over a +/-4
    # tick window instead.
    v = np.full_like(t, np.nan)
    for i in range(len(t)):
        a, b = max(0, i - 4), min(len(t), i + 5)
        if b - a < 3 or (t[b - 1] - t[a]) < 0.02:
            continue
        A = np.vstack([t[a:b], np.ones(b - a)]).T
        sol = np.linalg.lstsq(A, P[a:b, :2], rcond=None)[0]
        v[i] = math.hypot(sol[0, 0], sol[0, 1])
    ramp = np.isfinite(v) & (v > 2.0) & (v < 12.0)   # clearly on the linear ramp, off both ends
    a_eff = float(np.polyfit(t[ramp], v[ramp], 1)[0]) if ramp.sum() >= 4 else float("nan")
    ss = np.isfinite(v) & (t > 3.0)  # steady-state check (validates the whole time base)
    v_ss = float(np.mean(v[ss])) if ss.sum() >= 5 else float("nan")
    # Direct, FIT-FREE along-path measurement at CPA: how far the vehicle actually
    # travelled vs the constant-accel ramp the aim solution assumes (a=10, V=16).
    # Preferred over a_eff, which is window-sensitive (8.5-10.0 depending on the fit
    # band) because the real ramp has a dead-band at the start and a knee at the end.
    jc = int(np.argmin(np.abs(tc - t_cpa)))
    s_act_cpa = float(np.sum(np.linalg.norm(np.diff(Pc[:jc + 1, :2], axis=0), axis=1)))
    a10, V = 10.0, 16.0
    s_a10 = float(0.5 * a10 * t_cpa ** 2 if t_cpa < V / a10 else V * t_cpa - V * V / (2 * a10))
    j = int(np.argmin(np.abs(t - t_cpa)))
    return dict(miss_true=miss_true, t_cpa=t_cpa, miss_altheld=miss_altheld, miss_novert=miss_novert,
                alt_err_cpa=float(alt[j] - ALT_REF_M), alt_err_peak=float(np.nanmax(alt) - ALT_REF_M),
                a_eff=a_eff, cmdh_max=float(np.nanmax(cmdh)),
                rail_h=float(np.mean((np.abs(cmdh - 13.0) < 1e-3) | (np.abs(cmdh - 18.0) < 1e-3))),
                rail_perp=float(np.nanmean(np.abs(np.abs(vperp) - 8.0) < 1e-4)), v_ss=v_ss,
                s_act_cpa=s_act_cpa, s_a10=s_a10)


def arm(name):
    rows = _load(LOGS + name + ".csv")
    out = []
    for r in rows:
        p = r["flight_csv_path"]
        if not os.path.exists(p):
            continue
        d = flight(p, float(r["target_vx"]), float(r["target_vy"]))
        if d:
            d["miss_logged"] = float(r["miss_m"]); d["dir"] = r["direction"]
            out.append(d)
    if not out:
        print(f"{name}: NO USABLE FLIGHTS -- refusing to report a verdict on zero units")
        return 1
    def col(k): return [d[k] for d in out]
    print(f"\n=== {name}  n={len(out)} ===")
    print(f"  logged miss (per-tick sampled min) : {S.mean(col('miss_logged')):.3f} +/- {S.pstdev(col('miss_logged')):.3f} m")
    print(f"  true CPA  (time-interpolated)      : {S.mean(col('miss_true')):.3f} +/- {S.pstdev(col('miss_true')):.3f} m")
    vss=[x for x in col('v_ss') if x == x]
    print(f"  SANITY: steady-state achieved |v|   : {(S.mean(vss) if vss else float('nan')):.3f} m/s vs a 16.000 m/s command (must be ~1.00x or the time base is wrong)")
    print(f"  effective forward accel a_eff      : {S.mean(col('a_eff')):.2f} +/- {S.pstdev(col('a_eff')):.2f} m/s^2  (WINDOW-SENSITIVE 8.5-10.0; aim assumes 10.0)")
    print(f"  along-path distance at CPA         : {S.mean(col('s_act_cpa')):.3f} m flown vs {S.mean(col('s_a10')):.3f} m for the assumed a=10 ramp"
          f"  -> {S.mean(col('s_a10'))-S.mean(col('s_act_cpa')):+.3f} m shortfall (fit-free)")
    print(f"  altitude error at CPA              : {S.mean(col('alt_err_cpa')):+.3f} +/- {S.pstdev(col('alt_err_cpa')):.3f} m   peak {S.mean(col('alt_err_peak')):+.3f} m")
    d = [a - b for a, b in zip(col('miss_true'), col('miss_altheld'))]
    print(f"  ACTUATION (vertical) contribution  : {S.mean(d):+.3f} +/- {S.pstdev(d):.3f} m   -> miss would be {S.mean(col('miss_altheld')):.3f} m")
    print(f"  floor with zero vertical error     : {S.mean(col('miss_novert')):.3f} m")
    print(f"  inside 0.35 m ram radius           : as flown {sum(1 for x in col('miss_true') if x < .35)}/{len(out)}"
          f" | alt held {sum(1 for x in col('miss_altheld') if x < .35)}/{len(out)}"
          f" | zero vertical {sum(1 for x in col('miss_novert') if x < .35)}/{len(out)}")
    print(f"  command railed: |cmd_h| {100*S.mean(col('rail_h')):.1f}% of ticks | v_perp at 8.0 m/s {100*S.mean(col('rail_perp')):.1f}% of ticks")
    return 0


if __name__ == "__main__":
    names = sys.argv[1:] or ["mc_fp_armAE5dash_line9_s123", "mc_fp_armAE5dash_line9_s777",
                             "mc_speed10_dash_s123", "mc_fp_armAE5_line9_s123"]
    rc = 0
    for n in names:
        rc |= arm(n)
    sys.exit(rc)
