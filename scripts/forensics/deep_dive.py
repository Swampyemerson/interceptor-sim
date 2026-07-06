#!/usr/bin/env python3
"""Follow-up forensics: per-flight validation traces + corrected/extra
aggregates. Uses terminal_forensics.analyze_flight for the shared loaders."""
import csv, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from terminal_forensics import (AGGREGATES, REPO, analyze_flight, FX, CX, CY,
                                HFOV_HALF_DEG, TAG_M, wrap180)

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
                res["agg_miss"] = float(row["miss_m"])
                results.append(res)
    return results

def trace_flight(r, n_last=50):
    s = r["_series"]
    t, gtr, det = s["t"], s["gtr"], s["det"]
    icpa, ilast = s["icpa"], s["ilast"]
    rtf = s["rtf"]
    print(f"\n===== TRACE {r['label']} ({r['path']}) miss={r['miss']:.3f} rtf={rtf:.3f} =====")
    print("  t_wall  dt_cpa_sim  phase    r_gt   det new  beta_t  beta_m?   u_px    lamdot_t  psidot  cmdyaw-psi  zem    inc")
    i0 = max(0, icpa - n_last)
    for i in range(i0, min(icpa + 6, len(t))):
        dt_sim = (t[icpa] - t[i]) * rtf
        mark = "<-- last det" if i == ilast else ("<== CPA" if i == icpa else "")
        beta = s["beta_true"][i]
        u = s["u"][i]
        print(f" {t[i]:7.2f}  {dt_sim:8.3f}  {s['phase'][i][:7]:7s} {gtr[i]:6.2f}  {int(det[i]) if np.isfinite(det[i]) else '-'}  "
              f"{int(s['new_det'][i])}  {beta:7.1f} {'':8s} {u if np.isfinite(u) else float('nan'):7.0f}  "
              f"{s['lamdot_true'][i]:8.1f} {s['psidot'][i]:7.1f}  "
              f"{wrap180(s['cmd_yaw'][i]-s['psi_unwrap'][i]) if np.isfinite(s['cmd_yaw'][i]) else float('nan'):8.1f} "
              f"{s['zem'][i]:6.2f} {180-s['inc'][i]:5.1f} {mark}")

def main():
    results = collect()
    print(f"{len(results)} flights")

    # --- corrected incidence (180 - reported) ---
    inc_last = np.array([180 - r["inc_lastdet"] for r in results])
    inc_max_blind = np.array([180 - r["max_inc_blind"] for r in results])  # NOTE: max reported = min actual
    print(f"\nincidence (angle off tag-face normal) at last det: "
          f"mean {np.nanmean(inc_last):.1f} med {np.nanmedian(inc_last):.1f} "
          f"min {np.nanmin(inc_last):.1f} max {np.nanmax(inc_last):.1f} deg")

    # --- measured bearing at last det (where the tag was in the PROCESSED frame) ---
    bm = np.array([abs(r["beta_meas_lastdet"]) for r in results])
    bt = np.array([abs(r["beta_true_lastdet"]) for r in results])
    print(f"|beta| at last det:  MEASURED(frame) mean {np.nanmean(bm):.1f} med {np.nanmedian(bm):.1f} max {np.nanmax(bm):.1f} | "
          f"TRUE(at publish tick) mean {np.nanmean(bt):.1f} med {np.nanmedian(bt):.1f} max {np.nanmax(bt):.1f} deg "
          f"(gap = detector+frame latency x LOS rate)")

    # --- miss vector decomposition at CPA: along target velocity (world Y) vs cross ---
    along, cross, vert = [], [], []
    for r in results:
        s = r["_series"]; icpa = s["icpa"]
        # reconstruct components from the flight csv columns already in series?
        # dx,dy,dz not stored; recompute from zem series? store not available ->
        # use gtr + beta geometry is awkward; recompute directly:
        pass
    # simpler: re-read the flight csvs for the CPA row
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
                rows = list(csv.DictReader(open(fpath)))
                g = [(float(x["gt_range"]) if x["gt_range"] else np.nan,
                      float(x["gt_tag_x"]) - float(x["gt_cam_x"]) if x["gt_tag_x"] and x["gt_cam_x"] else np.nan,
                      float(x["gt_tag_y"]) - float(x["gt_cam_y"]) if x["gt_tag_y"] and x["gt_cam_y"] else np.nan,
                      float(x["gt_tag_z"]) - float(x["gt_cam_z"]) if x["gt_tag_z"] and x["gt_cam_z"] else np.nan)
                     for x in rows]
                gr = np.array([q[0] for q in g])
                i = int(np.nanargmin(gr))
                _, dx, dy, dz = g[i]
                tvy = float(row["target_vy"]) if row.get("target_vy") else 6.0
                along.append(abs(dy) if tvy != 0 else np.nan)  # target moves along world Y
                cross.append(abs(dx))
                vert.append(abs(dz))
    along, cross, vert = map(np.array, (along, cross, vert))
    print(f"\nmiss vector at CPA: |along-track(Y)| mean {np.nanmean(along):.2f} med {np.nanmedian(along):.2f} | "
          f"|cross(X)| mean {np.nanmean(cross):.2f} med {np.nanmedian(cross):.2f} | "
          f"|vertical| mean {np.nanmean(vert):.2f} med {np.nanmedian(vert):.2f} m")

    # --- achieved lateral acceleration during last 2 sim s (from gt cam vel) ---
    accs, acc_maxs = [], []
    zem_at_engage, zem_at_handoff = [], []
    for r in results:
        s = r["_series"]; t = s["t"]; rtf = s["rtf"]; icpa = s["icpa"]
        van, vae = s["van"], s["vae"]
        # accel = d(v)/dt_sim
        an = np.gradient(van, t) / rtf
        ae = np.gradient(vae, t) / rtf
        anorm = np.hypot(an, ae)
        w = [i for i in range(len(t)) if i <= icpa and (t[icpa]-t[i])*rtf <= 2.0 and s["phase"][i] == "ENGAGE"]
        if w:
            accs.append(np.nanmean(anorm[w])); acc_maxs.append(np.nanmax(anorm[w]))
        # ZEM at first ENGAGE tick
        ieng = next((i for i in range(len(t)) if s["phase"][i] == "ENGAGE"), None)
        if ieng is not None:
            zem_at_engage.append(s["zem"][ieng])
    print(f"\nachieved |lateral+longitudinal accel| last 2 sim s (ENGAGE): "
          f"mean-of-means {np.nanmean(accs):.2f} m/s^2, mean-of-max {np.nanmean(acc_maxs):.2f} m/s^2 "
          f"(MPC_ACC_HOR_MAX=12)")
    ze = np.array(zem_at_engage)
    print(f"ZEM at ENGAGE start (handoff latch): mean {np.nanmean(ze):.2f} med {np.nanmedian(ze):.2f} "
          f"min {np.nanmin(ze):.2f} max {np.nanmax(ze):.2f} m (n={np.isfinite(ze).sum()})")

    # --- ZEM plateau check: mean ZEM in sim-time bins before CPA ---
    bins = [(2.0,1.5),(1.5,1.0),(1.0,0.8),(0.8,0.6),(0.6,0.4),(0.4,0.2),(0.2,0.0)]
    print("\nZEM by sim-time-to-CPA bin (mean over flights of per-flight bin means):")
    for hi, lo in bins:
        vals = []
        for r in results:
            s = r["_series"]; t = s["t"]; rtf = s["rtf"]; icpa = s["icpa"]
            w = [i for i in range(len(t)) if i <= icpa and lo <= (t[icpa]-t[i])*rtf < hi]
            if w:
                vals.append(np.nanmean(s["zem"][w]))
        print(f"  {hi:.1f}-{lo:.1f} s: mean ZEM {np.nanmean(vals):6.2f} m  (n={len(vals)})")

    # --- per-arm summary ---
    print("\nper-arm: miss vs ZEM@lastdet vs blind gap:")
    for arm in ["BASE","FUSE","FUSEWARM","EMIT","DIFF1","DIFF2"]:
        rs = [r for r in results if r["arm"] == arm]
        if not rs: continue
        miss = np.array([r["miss"] for r in rs]); zl = np.array([r["zem_lastdet"] for r in rs])
        bg = np.array([r["blind_sim_s"] for r in rs])
        rl = np.array([r["range_lastdet"] for r in rs])
        print(f"  {arm:9s} n={len(rs)}  miss {np.mean(miss):.2f}  ZEM@last {np.mean(zl):.2f}  "
              f"blind {np.mean(bg):.3f}s  range@last {np.mean(rl):.2f} m")

    # --- freshness vs event: how long does detected=1 persist after last event (0.4s staleness) ---
    persist = []
    for r in results:
        s = r["_series"]
        persist.append((s["t"][min(len(s['t'])-1, 0)],))
    # (skip; not needed)

    # --- trace two representative flights: one median miss, one worst, one best ---
    order = sorted(results, key=lambda r: r["miss"])
    for rr in [order[len(order)//2], order[-1], order[0]]:
        trace_flight(rr, n_last=42)

if __name__ == "__main__":
    main()
