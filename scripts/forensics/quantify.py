#!/usr/bin/env python3
"""Third pass: ZEM@handoff vs miss correlation, achieved accel (deduped),
relative speed, measurement latency estimate, correction-capacity census."""
import csv, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from terminal_forensics import AGGREGATES, REPO, analyze_flight

def collect():
    results = []
    for arm, agg in AGGREGATES:
        if not os.path.exists(agg):
            continue
        with open(agg) as f:
            for row in csv.DictReader(f):
                fpath = row["flight_csv_path"]
                if not os.path.exists(fpath):
                    fpath = os.path.join(REPO, "logs", os.path.basename(fpath))
                if not os.path.exists(fpath):
                    continue
                tvy = float(row["target_vy"]) if row.get("target_vy") else 6.0
                res = analyze_flight(fpath, tvy, f"{arm}#{row['run_idx']}")
                if res is None:
                    continue
                res["arm"] = arm
                results.append(res)
    return results

def lsq_slope(t, x):
    m = np.isfinite(t) & np.isfinite(x)
    if m.sum() < 3 or t[m].max() - t[m].min() < 1e-6:
        return np.nan
    return np.polyfit(t[m], x[m], 1)[0]

def main():
    results = collect()
    n = len(results)
    print(f"{n} flights")

    zem_eng, zem_frz, miss, tgo_eng, tgo_frz, vrel_frz = [], [], [], [], [], []
    acc_max_w, lat_est = [], []
    engage_dur = []
    for r in results:
        s = r["_series"]
        t, rtf, icpa = s["t"], s["rtf"], s["icpa"]
        phase = s["phase"]
        # dedupe timestamps for accel work
        keep = np.concatenate([[True], np.diff(t) > 1e-6])
        ieng = next((i for i in range(len(t)) if phase[i] == "ENGAGE"), None)
        if ieng is None:
            continue
        miss.append(r["miss"])
        zem_eng.append(s["zem"][ieng])
        zem_frz.append(r["zem_freeze"])
        tgo_eng.append((t[icpa] - t[ieng]) * rtf)
        tgo_frz.append((t[icpa] - r["t_freeze"]) * rtf if np.isfinite(r["t_freeze"]) else np.nan)
        engage_dur.append((t[icpa] - t[ieng]) * rtf)

        # |vrel| at freeze: from zem-geometry pieces we didn't store; use
        # rdot + lamdot at freeze via gtr slope and lam slope -> just use
        # 3D vrel norm from stored van/vae? those are own-vehicle only.
        # Approximate |vrel| from gtr closing rate + tangential R*lamdot:
        ifz = int(np.argmin(np.abs(t - r["t_freeze"]))) if np.isfinite(r["t_freeze"]) else None
        if ifz is not None:
            R = s["gtr"][ifz]
            rd = lsq_slope(t[max(0,ifz-3):ifz+4], s["gtr"][max(0,ifz-3):ifz+4]) / rtf
            ld = np.radians(s["lamdot_true"][ifz])
            if np.isfinite(rd) and np.isfinite(ld):
                vrel_frz.append(math.hypot(rd, R * ld))

        # achieved accel: max |delta v| over sliding ~0.25 s SIM windows in
        # the window [ENGAGE start .. CPA], velocities from deduped LSQ slopes
        td, xd = t[keep], None
        van, vae = s["van"][keep], s["vae"][keep]
        tsim = td * rtf
        amax = 0.0
        win = 0.25
        for i in range(len(td)):
            if not (ieng is not None and t[keep][i] >= t[ieng] and td[i] <= t[icpa]):
                continue
            j = np.searchsorted(tsim, tsim[i] + win)
            if j >= len(td):
                break
            dv = math.hypot(van[j] - van[i], vae[j] - vae[i])
            dt = tsim[j] - tsim[i]
            if dt > 0.05 and np.isfinite(dv):
                amax = max(amax, dv / dt)
        if amax > 0:
            acc_max_w.append(amax)

        # measurement latency: shift delta minimizing |beta_true(t_event - d) - beta_meas(event)|
        # over terminal detection events (gt<6m). beta_meas from bearing col at event ticks.
        ev = [i for i in np.where(s["new_det"])[0] if i <= icpa and s["gtr"][i] < 6.0]
        if len(ev) >= 5:
            bt = s["beta_true"]
            best_d, best_e = np.nan, 1e9
            for d in np.arange(0.0, 0.45, 0.025):  # wall seconds
                errs = []
                for i in ev:
                    tt = t[i] - d
                    j = np.searchsorted(t, tt)
                    if 0 < j < len(t):
                        # linear interp of beta_true
                        f = (tt - t[j-1]) / max(t[j] - t[j-1], 1e-9)
                        btt = bt[j-1] + f * (bt[j] - bt[j-1])
                        # beta_meas from r? need bearing column: reconstruct
                        errs.append(btt)
                if errs:
                    pass
                # need beta_meas; recompute below instead
            # do it properly with bearing values
        lat_est.append(np.nan)

    miss = np.array(miss); zem_eng = np.array(zem_eng); zem_frz = np.array(zem_frz)
    tgo_eng = np.array(tgo_eng); tgo_frz = np.array(tgo_frz)
    print(f"\nENGAGE duration (handoff->CPA, sim s): mean {np.nanmean(engage_dur):.2f} "
          f"med {np.nanmedian(engage_dur):.2f} min {np.nanmin(engage_dur):.2f} max {np.nanmax(engage_dur):.2f}")
    print(f"time freeze->CPA (sim s): mean {np.nanmean(tgo_frz):.2f} med {np.nanmedian(tgo_frz):.2f}")
    print(f"|vrel| at freeze: mean {np.nanmean(vrel_frz):.2f} med {np.nanmedian(vrel_frz):.2f} m/s (n={len(vrel_frz)})")
    print(f"achieved max |dV|/dt over 0.25s-sim windows in ENGAGE: "
          f"mean {np.nanmean(acc_max_w):.2f} med {np.nanmedian(acc_max_w):.2f} "
          f"p90 {np.nanpercentile(acc_max_w,90):.2f} m/s^2 (n={len(acc_max_w)})")

    m = np.isfinite(miss) & np.isfinite(zem_eng)
    r2 = np.corrcoef(zem_eng[m], miss[m])[0,1]**2
    print(f"\nZEM@ENGAGE-start vs final miss: r^2 = {r2:.3f} (n={m.sum()})")
    mz = np.isfinite(miss) & np.isfinite(zem_frz)
    r2f = np.corrcoef(zem_frz[mz], miss[mz])[0,1]**2
    print(f"ZEM@freeze vs final miss:        r^2 = {r2f:.3f} (n={mz.sum()})")
    print(f"mean correction achieved during ENGAGE (ZEM@engage - miss): "
          f"{np.nanmean(zem_eng[m] - miss[m]):.3f} m (med {np.nanmedian(zem_eng[m]-miss[m]):.3f})")
    # census vs correction capacity
    for cap in (1.0, 1.5, 2.0):
        k = (zem_eng > cap).sum()
        mm = miss[zem_eng > cap]
        print(f"flights with ZEM@handoff > {cap:.1f} m: {k}/{len(zem_eng)}"
              + (f"  (their mean miss {mm.mean():.2f} m)" if k else ""))
    # and the low-ZEM flights
    k = (zem_eng <= 1.0).sum()
    print(f"flights with ZEM@handoff <= 1.0 m: {k} -> mean miss {miss[zem_eng<=1.0].mean():.2f} m, "
          f"max {miss[zem_eng<=1.0].max():.2f}")

    # correction capacity math (shown derivation):
    tgo = np.nanmedian(tgo_eng); a_eff = np.nanmedian(acc_max_w)
    print(f"\ncorrection capacity from handoff: 0.5*a_eff*t_go^2 = 0.5*{a_eff:.1f}*{tgo:.2f}^2 "
          f"= {0.5*a_eff*tgo**2:.2f} m (a_eff = measured median achieved, t_go = median ENGAGE dur)")
    tgf = np.nanmedian(tgo_frz)
    print(f"correction capacity from freeze latch: 0.5*{a_eff:.1f}*{tgf:.2f}^2 = {0.5*a_eff*tgf**2:.2f} m")

if __name__ == "__main__":
    main()
