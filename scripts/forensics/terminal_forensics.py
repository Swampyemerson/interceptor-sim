#!/usr/bin/env python3
"""Terminal-dropout root-cause forensics over the realistic-cue Monte-Carlo
flight logs. Read-only over logs/; writes per-flight + aggregate results to
stdout and a machine-readable summary CSV in the job tmp dir.

Frames / conventions (from scripts/m4_intercept.py + docs/goals.md):
  gt_* columns are Gazebo WORLD ENU: x=East, y=North, z=Up.
  psi_deg is PX4 NED yaw: 0=North, positive clockwise (toward East).
  NED LOS azimuth lambda_true = atan2(dE, dN) = atan2(dx_world, dy_world).
  Camera bearing beta = wrap(lambda - psi), positive = tag right of nose.
  Camera: fx=fy=539.936, cx=640, cy=480, 1280x960 (run-log header).
  HFOV half = atan(640/539.936) = 49.85 deg; VFOV half = atan(480/539.936)=41.63 deg.
  Tag: 0.5 m (TAG_SIZE_M, m2_detect.py).
  t column is WALL seconds; sim time runs slower by RTF. Per-flight RTF is
  estimated from d(gt_tag_y)/dt vs the aggregate's commanded target_vy.
"""
import csv, math, os, sys, glob
import numpy as np

REPO = "/home/emerson/interceptor-sim/.claude/worktrees/s2-handoff"
FX = 539.936; CX = 640.0; CY = 480.0; W = 1280; H = 960
HFOV_HALF_DEG = math.degrees(math.atan(CX / FX))   # 49.85
VFOV_HALF_DEG = math.degrees(math.atan(CY / FX))   # 41.63
TAG_M = 0.5
TERMINAL_RANGE = 5.0     # FPV TERMINAL_RANGE_M (hold-last-cmd + breakoff clock)
FREEZE_RANGE = 3.5       # FPV TERMINAL_FREEZE_RANGE_M (whole-vector coast latch)

AGGREGATES = [
    ("BASE",     f"{REPO}/logs/mc_p6_BASE_20260706T031509Z.csv"),
    ("FUSE",     f"{REPO}/logs/mc_p6_FUSE_20260706T032302Z.csv"),
    ("FUSEWARM", f"{REPO}/logs/mc_p6_FUSEWARM_20260706T033055Z.csv"),
    ("EMIT",     f"{REPO}/logs/mc_realistic_EMIT_20260706T015033Z.csv"),
    ("DIFF1",    f"{REPO}/logs/mc_realistic_DIFF_20260706T021425Z.csv"),
    ("DIFF2",    f"{REPO}/logs/mc_realistic_DIFF_20260706T022749Z.csv"),
]

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

def load_flight(path):
    rows = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    return rows

def fnum(row, k):
    v = row.get(k, "")
    if v is None or v == "":
        return math.nan
    try:
        return float(v)
    except ValueError:
        return math.nan

def smooth_diff(t, x, half=2):
    """Least-squares slope of x vs t over a +/-half tick window (robust
    finite difference)."""
    n = len(t); out = np.full(n, np.nan)
    for i in range(n):
        a, b = max(0, i-half), min(n, i+half+1)
        tt, xx = t[a:b], x[a:b]
        m = np.isfinite(tt) & np.isfinite(xx)
        if m.sum() >= 3 and (tt[m].max() - tt[m].min()) > 1e-6:
            out[i] = np.polyfit(tt[m], xx[m], 1)[0]
    return out

def analyze_flight(path, target_vy, label):
    rows = load_flight(path)
    if not rows:
        return None
    t = np.array([fnum(r, "t") for r in rows])
    phase = [r["phase"] for r in rows]
    det = np.array([fnum(r, "detected") for r in rows])
    meas_range = np.array([fnum(r, "meas_range") for r in rows])
    bearing = np.array([fnum(r, "bearing_rad") for r in rows])
    psi = np.array([fnum(r, "psi_deg") for r in rows])
    lam_hat = np.array([fnum(r, "lambda_deg") for r in rows])
    lamdot_hat = np.array([fnum(r, "lambda_dot_deg_s") for r in rows])
    r_hat = np.array([fnum(r, "r_hat_m") for r in rows])
    cmd_vn = np.array([fnum(r, "cmd_vn") for r in rows])
    cmd_ve = np.array([fnum(r, "cmd_ve") for r in rows])
    cmd_yaw = np.array([fnum(r, "cmd_yaw_deg") for r in rows])
    cx_ = np.array([fnum(r, "gt_cam_x") for r in rows])
    cy_ = np.array([fnum(r, "gt_cam_y") for r in rows])
    cz_ = np.array([fnum(r, "gt_cam_z") for r in rows])
    tx = np.array([fnum(r, "gt_tag_x") for r in rows])
    ty = np.array([fnum(r, "gt_tag_y") for r in rows])
    tz = np.array([fnum(r, "gt_tag_z") for r in rows])
    gtr = np.array([fnum(r, "gt_range") for r in rows])

    n = len(rows)
    # --- RTF: target moves at |target_vy| m/s SIM along world Y. Fit the
    # wall-time slope over the window where the tag is actually moving.
    moving = np.abs(smooth_diff(t, ty, 3)) > 0.5
    rtf = np.nan
    if moving.sum() > 20:
        idx = np.where(moving)[0]
        a, b = idx[0], idx[-1]
        rtf = abs(np.polyfit(t[a:b], ty[a:b], 1)[0]) / abs(target_vy)
    if not np.isfinite(rtf) or rtf <= 0:
        rtf = 1.0

    # --- CPA: min gt_range, restricted to before the breakoff climb ends
    icpa = int(np.nanargmin(gtr))
    t_cpa = t[icpa]; miss = gtr[icpa]

    # --- true geometry per tick ---
    dx = tx - cx_; dy = ty - cy_; dz = tz - cz_
    horiz = np.hypot(dx, dy)
    lam_true = np.degrees(np.arctan2(dx, dy))          # NED azimuth
    beta_true = wrap180(lam_true - psi)                # deg, + = right of nose
    elev = np.degrees(np.arctan2(dz, horiz))           # + = above camera
    # pixel coords under zero-pitch/roll assumption (documented caveat)
    with np.errstate(invalid="ignore"):
        u = np.where(np.abs(beta_true) < 89.0,
                     CX + FX * np.tan(np.radians(beta_true)), np.nan)
        v = np.where(np.abs(elev) < 89.0,
                     CY - FX * np.tan(np.radians(elev)), np.nan)
    in_fov_h = np.abs(beta_true) < HFOV_HALF_DEG
    in_fov_v = np.abs(elev) < VFOV_HALF_DEG
    in_fov = in_fov_h & in_fov_v
    # incidence angle onto the board face (normal -X world)
    with np.errstate(invalid="ignore"):
        cosinc = np.clip((-dx) / np.maximum(gtr, 1e-6), -1, 1)
    inc = np.degrees(np.arccos(cosinc))  # 0 = head-on to face

    # --- true LOS rate + own yaw rate (SIM-time deg/s) ---
    lam_unwrap = np.degrees(np.unwrap(np.radians(lam_true)))
    lamdot_true = smooth_diff(t, lam_unwrap, 2) / rtf
    psi_unwrap = np.degrees(np.unwrap(np.radians(psi)))
    psidot = smooth_diff(t, psi_unwrap, 2) / rtf
    cmd_yaw_unwrap = np.degrees(np.unwrap(np.radians(cmd_yaw)))

    # --- detection EVENTS (new detector output): meas_range present and
    # different from the previous present value, or first after a blank ---
    new_det = np.zeros(n, dtype=bool)
    last_val = None; last_i = None
    for i in range(n):
        if np.isfinite(meas_range[i]):
            if last_val is None or meas_range[i] != last_val or (last_i is not None and i - last_i > 1 and not np.isfinite(meas_range[i-1])):
                new_det[i] = True
            last_val = meas_range[i]; last_i = i

    # last detection EVENT at/before CPA
    ev_pre = np.where(new_det & (np.arange(n) <= icpa))[0]
    if len(ev_pre) == 0:
        return None
    ilast = ev_pre[-1]
    # last fresh tick (detected==1) before CPA
    fresh_pre = np.where((det == 1) & (np.arange(n) <= icpa))[0]
    ilastfresh = fresh_pre[-1] if len(fresh_pre) else ilast

    # freeze instant: first ENGAGE tick with r_hat < FREEZE_RANGE
    ifreeze = None
    for i in range(n):
        if phase[i] == "ENGAGE" and np.isfinite(r_hat[i]) and r_hat[i] < FREEZE_RANGE:
            ifreeze = i; break

    # --- ZEM (zero-effort miss) from gt states, sim-time velocities ---
    vdx = smooth_diff(t, dx, 2) / rtf   # d(rel)/dt_sim
    vdy = smooth_diff(t, dy, 2) / rtf
    vdz = smooth_diff(t, dz, 2) / rtf
    def zem_at(i):
        rel = np.array([dx[i], dy[i], dz[i]])
        vr = np.array([vdx[i], vdy[i], vdz[i]])
        if not np.all(np.isfinite(vr)) or np.dot(vr, vr) < 1e-9:
            return np.nan
        tau = -np.dot(rel, vr) / np.dot(vr, vr)
        if tau < 0:
            return float(np.linalg.norm(rel))
        return float(np.linalg.norm(rel + vr * tau))
    zem = np.array([zem_at(i) for i in range(n)])

    # closing speed at last detection (sim)
    rdot_true = smooth_diff(t, gtr, 2) / rtf
    # achieved velocity (world ENU -> NED n/e) vs commanded, in last 1 s pre-CPA
    van = smooth_diff(t, cy_, 2) / rtf  # north = world y
    vae = smooth_diff(t, cx_, 2) / rtf  # east = world x

    # window bookkeeping (sim seconds relative to CPA)
    def sim_dt(i, j):
        return (t[j] - t[i]) * 1.0 / (1.0 / rtf) if False else (t[j] - t[i]) * rtf
    # NOTE: t is wall; sim elapsed = wall_elapsed * rtf
    blind_sim = (t_cpa - t[ilast]) * rtf

    # detection cadence per gt-range band, pre-CPA, sim-time
    bands = [(10.0, 99.0, "gt>10"), (5.0, 10.0, "5-10"), (3.5, 5.0, "3.5-5"), (0.0, 3.5, "<3.5")]
    cadence = {}
    for lo, hi, name in bands:
        m = (gtr >= lo) & (gtr < hi) & (np.arange(n) <= icpa) & np.isfinite(gtr)
        if m.sum() < 2:
            cadence[name] = (np.nan, np.nan, 0)
            continue
        wall_span = t[m].max() - t[m].min()
        sim_span = wall_span * rtf
        nev = int(new_det[m].sum())
        cadence[name] = (nev / sim_span if sim_span > 0 else np.nan, sim_span, nev)

    # gap-length distribution inside terminal (gt<5) pre-CPA: sim-time gaps
    # between consecutive detection events, plus the final blind gap to CPA
    ev_term = [i for i in np.where(new_det)[0] if i <= icpa and gtr[i] < TERMINAL_RANGE]
    gaps = []
    if ev_term:
        for a, b in zip(ev_term[:-1], ev_term[1:]):
            gaps.append((t[b] - t[a]) * rtf)
    # FOV occupancy during the blind window [ilast+1 .. icpa]
    blind_idx = np.arange(ilast + 1, icpa + 1)
    if len(blind_idx):
        infov_frac = float(np.mean(in_fov[blind_idx]))
        infov_h_frac = float(np.mean(in_fov_h[blind_idx]))
        max_beta_blind = float(np.nanmax(np.abs(beta_true[blind_idx])))
        # first tick the target exits the horizontal FOV, if ever
        exit_idx = None
        for i in blind_idx:
            if not in_fov_h[i]:
                exit_idx = i; break
        exit_dt_sim = (t_cpa - t[exit_idx]) * rtf if exit_idx is not None else np.nan
        exit_range = gtr[exit_idx] if exit_idx is not None else np.nan
    else:
        infov_frac = infov_h_frac = max_beta_blind = np.nan
        exit_idx = None; exit_dt_sim = exit_range = np.nan

    # per-tick classification over the last 2 SIM seconds pre-CPA
    last2 = [i for i in range(n) if i <= icpa and (t_cpa - t[i]) * rtf <= 2.0 and phase[i] in ("ENGAGE",)]
    cls = {"infov_det": 0, "infov_nodet": 0, "outfov": 0}
    for i in last2:
        if not in_fov_h[i]:
            cls["outfov"] += 1
        elif det[i] == 1:
            cls["infov_det"] += 1
        else:
            cls["infov_nodet"] += 1

    # pixel step between consecutive detection events in terminal band
    px_steps = []
    for a, b in zip(ev_term[:-1], ev_term[1:]) if ev_term else []:
        ua = CX + FX * math.tan(bearing[a]) if np.isfinite(bearing[a]) else np.nan
        ub = CX + FX * math.tan(bearing[b]) if np.isfinite(bearing[b]) else np.nan
        if np.isfinite(ua) and np.isfinite(ub):
            px_steps.append(abs(ub - ua))

    # ZEM at reference instants
    def zem_near_simtime_before_cpa(dt_sim):
        tgt_wall = t_cpa - dt_sim / rtf
        i = int(np.argmin(np.abs(t - tgt_wall)))
        return zem[i]

    out = dict(
        label=label, path=os.path.basename(path), rtf=rtf, miss=miss,
        t_cpa=t_cpa, phase_cpa=phase[icpa],
        # last detection event
        t_last=t[ilast], blind_sim_s=blind_sim,
        range_lastdet=gtr[ilast], beta_true_lastdet=beta_true[ilast],
        beta_meas_lastdet=math.degrees(bearing[ilast]) if np.isfinite(bearing[ilast]) else np.nan,
        u_px_lastdet=u[ilast], inc_lastdet=inc[ilast],
        tagpx_lastdet=FX * TAG_M / gtr[ilast] if np.isfinite(gtr[ilast]) else np.nan,
        lamdot_true_lastdet=lamdot_true[ilast],
        rdot_lastdet=rdot_true[ilast],
        # blind window
        infov_frac_blind=infov_frac, infov_h_frac_blind=infov_h_frac,
        max_beta_blind=max_beta_blind, fov_exit_dtsim=exit_dt_sim,
        fov_exit_range=exit_range,
        max_lamdot_true_blind=float(np.nanmax(np.abs(lamdot_true[blind_idx]))) if len(blind_idx) else np.nan,
        max_psidot_blind=float(np.nanmax(np.abs(psidot[blind_idx]))) if len(blind_idx) else np.nan,
        mean_psidot_blind=float(np.nanmean(np.abs(psidot[blind_idx]))) if len(blind_idx) else np.nan,
        max_inc_blind=float(np.nanmax(inc[blind_idx])) if len(blind_idx) else np.nan,
        inc_at_lastdet=inc[ilast],
        # ZEM story
        zem_lastdet=zem[ilast],
        zem_freeze=zem[ifreeze] if ifreeze is not None else np.nan,
        range_freeze=gtr[ifreeze] if ifreeze is not None else np.nan,
        t_freeze=t[ifreeze] if ifreeze is not None else np.nan,
        zem_1s=zem_near_simtime_before_cpa(1.0),
        zem_05s=zem_near_simtime_before_cpa(0.5),
        zem_15s=zem_near_simtime_before_cpa(1.5),
        zem_2s=zem_near_simtime_before_cpa(2.0),
        # cadence
        cadence=cadence, gaps_term=gaps, px_steps=px_steps, cls_last2=cls,
        # series for optional deep dive
        _series=dict(t=t, rtf=rtf, gtr=gtr, beta_true=beta_true, det=det,
                     new_det=new_det, lamdot_true=lamdot_true, psidot=psidot,
                     inc=inc, zem=zem, phase=phase, icpa=icpa, ilast=ilast,
                     in_fov_h=in_fov_h, u=u, elev=elev,
                     cmd_vn=cmd_vn, cmd_ve=cmd_ve, van=van, vae=vae,
                     cmd_yaw=cmd_yaw_unwrap, psi_unwrap=psi_unwrap,
                     lamdot_hat=lamdot_hat, r_hat=r_hat),
    )
    return out


def main():
    results = []
    for arm, agg in AGGREGATES:
        if not os.path.exists(agg):
            print(f"[skip] {agg}")
            continue
        with open(agg) as f:
            for row in csv.DictReader(f):
                fpath = row["flight_csv_path"]
                if not os.path.exists(fpath):
                    # worktree path may differ; try basename under repo logs
                    fpath = os.path.join(REPO, "logs", os.path.basename(fpath))
                if not os.path.exists(fpath):
                    print(f"[missing flight] {row['flight_csv_path']}")
                    continue
                tvy = float(row["target_vy"]) if row.get("target_vy") else 6.0
                res = analyze_flight(fpath, tvy, f"{arm}#{row['run_idx']}")
                if res is None:
                    print(f"[no-data] {fpath}")
                    continue
                res["agg_miss"] = float(row["miss_m"])
                res["breakoff_reason"] = row.get("breakoff_reason", "")
                results.append(res)
    print(f"\n=== {len(results)} flights analyzed ===\n")

    def col(key):
        return np.array([r[key] for r in results], dtype=float)

    def stats(name, arr, fmt="{:7.3f}"):
        arr = arr[np.isfinite(arr)]
        if len(arr) == 0:
            print(f"{name:34s}  (no data)"); return
        print(f"{name:34s} mean {fmt.format(np.mean(arr))}  med {fmt.format(np.median(arr))} "
              f" min {fmt.format(np.min(arr))}  max {fmt.format(np.max(arr))}  n={len(arr)}")

    print("--- headline per-flight numbers (aggregated) ---")
    stats("miss (min gt_range) [m]", col("miss"))
    stats("RTF (sim/wall)", col("rtf"))
    stats("blind gap lastdet->CPA [sim s]", col("blind_sim_s"))
    stats("gt_range at last detection [m]", col("range_lastdet"))
    stats("|beta_true| at last det [deg]", np.abs(col("beta_true_lastdet")))
    stats("pixel u at last det [px]", col("u_px_lastdet"))
    stats("incidence at last det [deg]", col("inc_lastdet"))
    stats("tag width at last det [px]", col("tagpx_lastdet"))
    stats("|lamdot_true| at last det [d/s]", np.abs(col("lamdot_true_lastdet")))
    stats("closing rdot at last det [m/s]", col("rdot_lastdet"))
    stats("max|beta_true| blind window", col("max_beta_blind"))
    stats("frac blind window in H-FOV", col("infov_h_frac_blind"))
    stats("FOV-exit time before CPA [sim s]", col("fov_exit_dtsim"))
    stats("FOV exit range [m]", col("fov_exit_range"))
    stats("max|lamdot_true| blind [d/s]", col("max_lamdot_true_blind"))
    stats("max|psidot| blind [d/s]", col("max_psidot_blind"))
    stats("max incidence blind [deg]", col("max_inc_blind"))
    stats("ZEM at last detection [m]", col("zem_lastdet"))
    stats("ZEM at freeze latch [m]", col("zem_freeze"))
    stats("range at freeze latch [m]", col("range_freeze"))
    stats("ZEM 2.0 s before CPA [m]", col("zem_2s"))
    stats("ZEM 1.5 s before CPA [m]", col("zem_15s"))
    stats("ZEM 1.0 s before CPA [m]", col("zem_1s"))
    stats("ZEM 0.5 s before CPA [m]", col("zem_05s"))

    # FOV exit census
    nexit = int(np.isfinite(col("fov_exit_dtsim")).sum())
    print(f"\nFOV-escape census: {nexit}/{len(results)} flights have the target "
          f"leave the horizontal FOV before CPA (rest stay in-frame).")

    # in-FOV-but-undetected census over last 2 sim seconds
    tot = {"infov_det": 0, "infov_nodet": 0, "outfov": 0}
    for r in results:
        for k in tot:
            tot[k] += r["cls_last2"][k]
    s = sum(tot.values())
    if s:
        print(f"last-2-sim-s tick census (ENGAGE only): in-FOV+detected {tot['infov_det']} "
              f"({100*tot['infov_det']/s:.1f}%), in-FOV+NOT-detected {tot['infov_nodet']} "
              f"({100*tot['infov_nodet']/s:.1f}%), out-of-FOV {tot['outfov']} ({100*tot['outfov']/s:.1f}%)")

    # cadence aggregate
    print("\n--- detection cadence by gt-range band (events/sim-s) ---")
    for band in ["gt>10", "5-10", "3.5-5", "<3.5"]:
        vals = [r["cadence"][band][0] for r in results if np.isfinite(r["cadence"][band][0])]
        nev = sum(r["cadence"][band][2] for r in results)
        if vals:
            print(f"  {band:6s}: mean {np.mean(vals):5.2f} Hz  med {np.median(vals):5.2f}  (total events {nev})")
        else:
            print(f"  {band:6s}: no data (total events {nev})")

    # gap distribution in terminal
    all_gaps = [g for r in results for g in r["gaps_term"]]
    if all_gaps:
        ag = np.array(all_gaps)
        print(f"\nterminal (gt<5m) inter-detection gaps (sim s): n={len(ag)} "
              f"mean {ag.mean():.3f} med {np.median(ag):.3f} p90 {np.percentile(ag,90):.3f} max {ag.max():.3f}")
    bg = col("blind_sim_s")
    print(f"final blind gap to CPA (sim s): mean {np.nanmean(bg):.3f} med {np.nanmedian(bg):.3f} "
          f"p90 {np.nanpercentile(bg,90):.3f} max {np.nanmax(bg):.3f}")

    all_px = [p for r in results for p in r["px_steps"]]
    if all_px:
        ap = np.array(all_px)
        print(f"pixel step between consecutive terminal detections: n={len(ap)} "
              f"mean {ap.mean():.0f} med {np.median(ap):.0f} p90 {np.percentile(ap,90):.0f} max {ap.max():.0f} px")

    # miss decomposition: how much of the final miss was already locked at
    # last detection / at freeze (ZEM), vs accrued blind
    miss_ = col("miss"); zl = col("zem_lastdet"); zf = col("zem_freeze")
    m = np.isfinite(miss_) & np.isfinite(zl)
    print(f"\n--- miss decomposition (n={m.sum()}) ---")
    print(f"final miss:              mean {np.mean(miss_[m]):.3f} m")
    print(f"ZEM at last detection:   mean {np.mean(zl[m]):.3f} m  "
          f"(fraction of miss already locked: {np.mean(zl[m])/np.mean(miss_[m])*100:.0f}%)")
    mf = np.isfinite(miss_) & np.isfinite(zf)
    if mf.sum():
        print(f"ZEM at freeze latch:     mean {np.mean(zf[mf]):.3f} m (n={mf.sum()})")
    print(f"blind-accrued (miss - ZEM_lastdet): mean {np.mean(miss_[m]-zl[m]):.3f} m")

    # save per-flight table
    outp = "/home/emerson/.claude/jobs/28aff4e9/tmp/per_flight_summary.csv"
    keys = [k for k in results[0].keys() if not k.startswith("_") and k not in ("cadence", "gaps_term", "px_steps", "cls_last2")]
    with open(outp, "w", newline="") as f:
        w = csv.writer(f); w.writerow(keys)
        for r in results:
            w.writerow([r[k] for k in keys])
    print(f"\nper-flight table -> {outp}")
    return results

if __name__ == "__main__":
    main()
