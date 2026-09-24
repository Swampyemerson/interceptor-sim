#!/usr/bin/env python3
"""Endgame-estimator diagnosis, GAZEBO half (isim/specs/endgame_estimator_diag_2026-09-24.md).
Scratch analysis only. Reuses scripts/forensics/xcheck_tick_trace.py's loaders
and join conventions (drv/rf by row index, truth from _gtpose via gt_interp at
sim_t, consumed det at tick k <=> n_det_consumed increment at drv row k+1).
"""
import csv, json, math, os, re, sys
import numpy as np

REPO = "/home/emerson/interceptor-sim"
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "scripts", "forensics"))
import xcheck_tick_trace as xt  # noqa: E402
from flight.pursuit_terminal import (_ConstVelKF, _pursuit_measurement_noise,  # noqa: E402
                                     PursuitTerminalConfig)
from flight.camera import CameraModel  # noqa: E402

ARMS = {"brake": os.path.join(REPO, "logs/xcheck_gz_20260924_brake"),
        "nobrake": os.path.join(REPO, "logs/xcheck_gz_20260923_refly")}
ULOG_ROOT = "/home/emerson/PX4-Autopilot/build/px4_sitl_default/rootfs"
CFG = PursuitTerminalConfig()
CAM = CameraModel(540.3, 540.3, 640.0, 480.0, width=1280, height=960)
TAG = 0.5
BELIEF_VT = np.array([0.0, 9.0, 0.0])   # run log: belief "+0.0/+9.0 m/s" (N/E); mover vel=9 E
CAM_OFS = np.array([0.10, 0.0, -0.05])  # mount fwd 0.10, up 0.05 (FRD body) -- defaults


def f_(x):
    return math.nan if x in ("", None) else float(x)


def quat_rot_body_to_ned(yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def load_ulog_vel(arm_dir, k):
    log = open(os.path.join(arm_dir, f"sim_f{k}.log")).read()
    m = re.search(r"Opened full log file: \./(log/\S+\.ulg)", log)
    if not m:
        return None
    path = os.path.join(ULOG_ROOT, m.group(1))
    if not os.path.exists(path):
        return None
    from pyulog import ULog
    u = ULog(path, ["vehicle_local_position"])
    d = u.data_list[0].data
    return (d["timestamp"] / 1e6, np.stack([d["vx"], d["vy"], d["vz"]]))


def analyze(arm, k):
    arm_dir = ARMS[arm]
    drv, rf, gt, log = xt.load_flight(arm_dir, k)
    assert len(drv) == len(rf)
    gt_t, rel = xt.gt_interp(gt)
    rng_gt = np.linalg.norm(rel, axis=0)
    own_gt = np.stack([np.array([float(r["gt_veh_y"]) for r in gt]),
                       np.array([float(r["gt_veh_x"]) for r in gt]),
                       -np.array([float(r["gt_veh_z"]) for r in gt])])
    tgt_gt = own_gt + rel
    vown_gt = np.stack([np.gradient(own_gt[i], gt_t) for i in range(3)])
    # tag pose is re-set by the mover each frame with timing jitter: per-frame
    # gradient spans 4-14 m/s for a 9 m/s mover -> smooth over 25 frames (0.5 s)
    vtgt_gt = np.stack([xt.smooth(np.gradient(tgt_gt[i], gt_t), 25) for i in range(3)])
    rr_gt = np.gradient(xt.smooth(rng_gt, 5), gt_t)

    sim_t = np.array([f_(r["sim_t"]) for r in drv])
    mt = np.array([f_(r["t_s"]) for r in rf])
    eng = np.array([r["sm_state_pre_step"] == "ENGAGE" for r in drv])
    ncons = np.array([int(r["n_det_consumed"]) for r in drv])
    rhat = np.array([f_(r["r_hat_m"]) for r in rf])
    cmd = np.stack([np.array([f_(r[c]) for r in rf]) for c in ("v_north", "v_east", "v_down")])
    coast = np.array([r["terminal_coast"] in ("1", "True") for r in rf])
    N = len(drv)
    det = np.zeros(N, bool)
    cap_t = np.full(N, np.nan)
    pose_r = np.full(N, np.nan)
    for i in range(N - 1):
        if ncons[i + 1] > ncons[i] and drv[i + 1]["det_t_capture_sim"]:
            det[i] = True
            cap_t[i] = f_(drv[i + 1]["det_t_capture_sim"])
            pose_r[i] = f_(drv[i + 1]["det_tagpose_range_m"])
    phB = ~np.isnan(rhat) & eng
    rtrue = np.interp(sim_t, gt_t, rng_gt)
    reltrue = xt.interp_vec(sim_t, gt_t, rel)
    ownp = xt.interp_vec(sim_t, gt_t, own_gt)
    vown_t = xt.interp_vec(sim_t, gt_t, vown_gt)
    vtgt_t = xt.interp_vec(sim_t, gt_t, vtgt_gt)
    rr_t = np.interp(sim_t, gt_t, rr_gt)
    e = rhat - rtrue

    # CPA (tool convention: min gt range in [go, go+25])
    go_t = sim_t[np.argmax(eng)]
    m = (gt_t >= go_t) & (gt_t <= go_t + 25.0)
    j = int(np.argmin(np.where(m, rng_gt, np.inf)))
    t_cpa, cpa = float(gt_t[j]), float(rng_gt[j])

    # last-update bookkeeping
    last_upd = np.full(N, np.nan)
    lu = math.nan
    for i in range(N):
        if not phB[i]:
            lu = math.nan
            continue
        if det[i]:
            lu = sim_t[i]
        last_upd[i] = lu
    tsince = sim_t - last_upd

    # own-velocity input actually used by the KF predict at tick i: the
    # previous tick's commanded velocity (own_vel_fallback; vel_ned None).
    used_v = np.full((3, N), np.nan)
    used_v[:, 1:] = cmd[:, :-1]
    # true own displacement over (i-1, i]
    dpos = np.full((3, N), np.nan)
    dpos[:, 1:] = ownp[:, 1:] - ownp[:, :-1]
    dt = np.full(N, np.nan)
    dt[1:] = np.diff(sim_t)
    u = reltrue / np.maximum(np.linalg.norm(reltrue, axis=0), 1e-9)
    # scalar own-input contribution to d(e) per tick, along the true LOS
    de_own = -np.einsum("ij,ij->j", u, used_v * dt - dpos)
    v_in_err = used_v - dpos / dt           # vector input error (m/s)

    # ULog EKF velocity (registered candidate (c))
    ekf = load_ulog_vel(arm_dir, k)
    ekf_err_h = None
    ekf_v = None
    if ekf is not None:
        et, ev = ekf
        # PX4 lockstep time should equal gz sim time; fit a small offset
        best = (1e9, 0.0)
        sel = eng
        for off in np.arange(-1.0, 1.001, 0.004):
            evi = np.stack([np.interp(sim_t[sel] + off, et, ev[i]) for i in range(2)])
            err = np.nanmedian(np.linalg.norm(evi - vown_t[:2, sel], axis=0))
            if err < best[0]:
                best = (err, off)
        off = best[1]
        ekf_v = np.stack([np.interp(sim_t + off, et, ev[i]) for i in range(3)])
        ekf_err_h = np.linalg.norm(ekf_v[:2] - vown_t[:2], axis=0)

    idx = np.where(phB)[0]
    tend = sim_t[idx[-1]]
    last2 = phB & (sim_t >= tend - 2.0)
    pre2 = phB & (sim_t >= t_cpa - 2.0) & (sim_t <= t_cpa)

    # KF replica on logged inputs (vector); variants of own_vel input / measurement
    def replica(own_mode="cmd", meas_mode="logged"):
        kf = _ConstVelKF(CFG.kf_q_accel_ms2)
        out = np.full(N, np.nan)
        vt_out = np.full((3, N), np.nan)
        rvec = np.full((3, N), np.nan)
        prev_i = None
        vt_prev = BELIEF_VT.copy()
        init = False
        for i in range(N):
            if not phB[i]:
                if init:
                    vt_prev = kf.v_t.copy()
                    n = np.linalg.norm(vt_prev)
                    cap = max(9.0 * CFG.fallback_speed_cap_mult, CFG.fallback_speed_floor_ms)
                    if n > cap:
                        vt_prev *= cap / n
                init = False
                prev_i = None
                continue
            if own_mode == "cmd":
                vo = used_v[:, i]
            elif own_mode == "ekf":
                vo = ekf_v[:, i]
            else:
                vo = vown_t[:, i]
            if det[i]:
                tcap = sim_t[i] - CFG.meas_latency_s if meas_mode in ("perfect", "pose_only") else cap_t[i]
                z0 = xt.interp_vec(np.array([tcap]), gt_t, rel)[:, 0]
                if meas_mode in ("perfect", "lat_only"):
                    z = z0
                else:
                    zc = xt.interp_vec(np.array([cap_t[i]]), gt_t, rel)[:, 0]
                    # logged pose range is camera->tag; truth is CG->tag. The KF adds
                    # the camera offset back, so the effective along-LOS error is
                    # (pose - |cam->tag true at capture|).
                    yaw = f_(rf[i]["yaw_deg"])
                    ofs = quat_rot_body_to_ned(yaw if not math.isnan(yaw) else 0.0) @ CAM_OFS
                    r_cam = np.linalg.norm(zc - ofs)
                    perr = pose_r[i] - r_cam if not math.isnan(pose_r[i]) else 0.0
                    z = z0 * (1.0 + perr / np.linalg.norm(z0))
                rng_m = np.linalg.norm(z)
            if not init:
                if not det[i]:
                    continue
                kf.init(z, vt_prev, CFG.kf_p0_pos_m, CFG.kf_p0_vel_ms)
                init = True
                prev_i = i
                out[i] = np.linalg.norm(kf.r)
                vt_out[:, i] = kf.v_t
                rvec[:, i] = kf.r
                continue
            d_t = max(1e-3, mt[i] - mt[prev_i])
            kf.predict(d_t, vo)
            if det[i]:
                R = _pursuit_measurement_noise(rng_m, z / max(rng_m, 1e-9), CFG, CAM, TAG)
                kf.update(z, R, age_s=CFG.meas_latency_s, v_own=vo)
            prev_i = i
            out[i] = np.linalg.norm(kf.r)
            vt_out[:, i] = kf.v_t
            rvec[:, i] = kf.r
        return out, vt_out, rvec

    reps = {}
    for name, om, mm in (("R0_cmd", "cmd", "logged"), ("R1_ekf", "ekf", "logged"),
                         ("R2_truevel", "true", "logged"), ("R3_truevel_perfmeas", "true", "perfect"),
                         ("R4_cmd_perfmeas", "cmd", "perfect"),
                         ("R5_truevel_poseonly", "true", "pose_only"),
                         ("R6_truevel_latonly", "true", "lat_only")):
        if om == "ekf" and ekf_v is None:
            continue
        reps[name] = replica(om, mm)

    # per-detection measurement stats
    dets = []
    for i in np.where(det & phB)[0]:
        rc = float(np.interp(cap_t[i], gt_t, rng_gt))
        age = sim_t[i] - cap_t[i]
        rr_c = float(np.interp(sim_t[i], gt_t, rr_gt))
        yaw = f_(rf[i]["yaw_deg"])
        z0 = xt.interp_vec(np.array([cap_t[i]]), gt_t, rel)[:, 0]
        ofs = quat_rot_body_to_ned(yaw if not math.isnan(yaw) else 0.0) @ CAM_OFS
        r_cam = float(np.linalg.norm(z0 - ofs))
        dets.append(dict(t=sim_t[i], r_true=float(rtrue[i]), r_cap=rc, age=age,
                         pose_err=pose_r[i] - rc, pose_err_cam=pose_r[i] - r_cam,
                         lat_err_m=-(age - CFG.meas_latency_s) * rr_c,
                         e_post=float(e[i]), e_pre=float(e[i - 1] + de_own[i]) if i > 0 else math.nan,
                         in_last2=bool(last2[i]), in_pre2=bool(pre2[i])))

    # coast intervals: consecutive phase-B ticks after an update, until next update
    intervals = []
    i = 0
    while i < N:
        if phB[i] and det[i]:
            j0 = i
            jj = i + 1
            while jj < N and phB[jj] and not det[jj]:
                jj += 1
            seg = list(range(j0, jj))       # includes the update tick at j0
            if len(seg) >= 3:
                ts = sim_t[seg] - sim_t[j0]
                es = e[seg] - e[j0]
                own_cum = np.concatenate([[0.0], np.cumsum(de_own[seg[1:]])])
                slope = np.polyfit(ts, es, 1)[0]
                slope_own = np.polyfit(ts, own_cum, 1)[0]
                intervals.append(dict(t0=sim_t[j0], dur=ts[-1], n=len(seg), slope=slope,
                                      slope_own=slope_own, slope_rest=slope - slope_own,
                                      r0=float(rtrue[j0]),
                                      in_last2=bool(last2[j0] or last2[seg[-1]]),
                                      in_pre2=bool(pre2[j0] or pre2[seg[-1]]),
                                      coast_latch=bool(coast[seg].any())))
            i = jj
        else:
            i += 1

    # implied closing-rate error on non-update phase-B ticks
    rdot_hat = np.full(N, np.nan)
    for i in range(1, N):
        if phB[i] and phB[i - 1] and not det[i]:
            rdot_hat[i] = (rhat[i] - rhat[i - 1]) / (sim_t[i] - sim_t[i - 1])
    rr_mid = np.full(N, np.nan)
    rr_mid[1:] = (rtrue[1:] - rtrue[:-1]) / np.diff(sim_t)
    rdot_err = rdot_hat - rr_mid
    own_rdot = de_own / dt   # own-input part of the implied closing-rate error

    res = dict(arm=arm, flight=k, cpa=cpa, t_cpa=t_cpa, t_end=tend,
               n_phB=int(phB.sum()), n_det=int((det & phB).sum()),
               e_all_med=float(np.nanmedian(e[phB])),
               e_last2_med=float(np.nanmedian(e[last2])), n_last2=int(last2.sum()),
               e_pre2_med=float(np.nanmedian(e[pre2])) if pre2.any() else math.nan,
               n_pre2=int(pre2.sum()),
               e_last2_mean=float(np.nanmean(e[last2])),
               absr_last2_min=float(np.nanmin(rhat[last2])),
               cpa_before_end_s=float(tend - t_cpa),
               n_det_last2=int((det & last2).sum()), n_det_pre2=int((det & pre2).sum()),
               last_det_before_cpa_s=float(t_cpa - max([d["t"] for d in dets if d["t"] <= t_cpa],
                                                      default=math.nan)),
               vin_err_h_med_eng=float(np.nanmedian(np.linalg.norm(v_in_err[:2, phB], axis=0))),
               vin_err_h_med_pre2=float(np.nanmedian(np.linalg.norm(v_in_err[:2, pre2], axis=0)))
               if pre2.any() else math.nan,
               vin_err_los_med_pre2=float(np.nanmedian(np.einsum("ij,ij->j", u[:, pre2],
                                                                  v_in_err[:, pre2])))
               if pre2.any() else math.nan,
               ekf_err_h_med_eng=float(np.nanmedian(ekf_err_h[eng])) if ekf_err_h is not None else math.nan,
               ekf_err_h_p90_eng=float(np.nanpercentile(ekf_err_h[eng], 90)) if ekf_err_h is not None else math.nan,
               ekf_err_h_med_pre2=float(np.nanmedian(ekf_err_h[pre2])) if (ekf_err_h is not None and pre2.any()) else math.nan,
               rdot_err_med_pre2=float(np.nanmedian(rdot_err[pre2])) if np.isfinite(rdot_err[pre2]).any() else math.nan,
               n_rdot_pre2=int(np.isfinite(rdot_err[pre2]).sum()),
               own_rdot_med_pre2=float(np.nanmedian(own_rdot[pre2 & np.isfinite(rdot_err)]))
               if np.isfinite(rdot_err[pre2]).any() else math.nan,
               rdot_err_med_last2=float(np.nanmedian(rdot_err[last2])) if np.isfinite(rdot_err[last2]).any() else math.nan,
               own_rdot_med_last2=float(np.nanmedian(own_rdot[last2 & np.isfinite(rdot_err)]))
               if np.isfinite(rdot_err[last2]).any() else math.nan,
               )
    for name, (rr, vt, rv) in reps.items():
        er = rr - rtrue
        res[f"{name}_fid_med_abs_all"] = float(np.nanmedian(np.abs(rr[phB] - rhat[phB])))
        res[f"{name}_fid_med_abs_last2"] = float(np.nanmedian(np.abs(rr[last2] - rhat[last2])))
        res[f"{name}_e_last2_med"] = float(np.nanmedian(er[last2]))
        res[f"{name}_e_pre2_med"] = float(np.nanmedian(er[pre2])) if pre2.any() else math.nan
        res[f"{name}_e_last2_mean"] = float(np.nanmean(er[last2]))
        res[f"{name}_vt_err_pre2_med"] = float(np.nanmedian(np.linalg.norm(
            vt[:, pre2] - vtgt_t[:, pre2], axis=0))) if pre2.any() else math.nan
        res[f"{name}_vec_err_pre2_med"] = float(np.nanmedian(np.linalg.norm(
            rv[:, pre2] - reltrue[:, pre2], axis=0))) if pre2.any() else math.nan
        # implied relative-velocity error (vector) near CPA, replica state
        if name == "R0_cmd" and pre2.any():
            vrel_hat = vt[:, pre2] - used_v[:, pre2]
            vrel_true = vtgt_t[:, pre2] - vown_t[:, pre2]
            res["R0_vrel_err_pre2_med"] = float(np.nanmedian(np.linalg.norm(vrel_hat - vrel_true, axis=0)))
            res["R0_vt_part_pre2_med"] = float(np.nanmedian(np.linalg.norm(vt[:, pre2] - vtgt_t[:, pre2], axis=0)))
            res["R0_own_part_pre2_med"] = float(np.nanmedian(np.linalg.norm(used_v[:, pre2] - vown_t[:, pre2], axis=0)))
    # vector error of the replicas in last2 / final coast growth rate
    for name, (rr, vt, rv) in reps.items():
        E = np.linalg.norm(rv - reltrue, axis=0)
        res[f"{name}_vec_err_last2_med"] = float(np.nanmedian(E[last2]))
        res[f"{name}_vt_err_last2_med"] = float(np.nanmedian(np.linalg.norm(
            vt[:, last2] - vtgt_t[:, last2], axis=0)))
        # final coast (from last consumed det in phase B to phase-B end): slope of |E|
        dl = np.where(det & phB)[0]
        if dl.size:
            i0 = dl[-1]
            seg = np.arange(i0, idx[-1] + 1)
            seg = seg[phB[seg]]
            if seg.size >= 3:
                res[f"{name}_final_coast_vecE_slope"] = float(np.polyfit(
                    sim_t[seg] - sim_t[i0], E[seg] - E[i0], 1)[0])
                res[f"{name}_final_coast_dur"] = float(sim_t[seg[-1]] - sim_t[i0])
    # coast latch (terminal_coast: r_hat < hold_range 1.0 m -> command frozen)
    ci = np.where(coast & phB)[0]
    if ci.size:
        c0 = ci[0]
        res["latch_t_minus_cpa"] = float(sim_t[c0] - t_cpa)
        res["latch_rhat"] = float(rhat[c0])
        res["latch_rtrue"] = float(rtrue[c0])
        for name, (rr, vt, rv) in reps.items():
            res[f"latch_{name}_r"] = float(rr[c0])
            # would this replica have latched by then? first tick with |r|<1 m
            lat = np.where(phB & (rr < CFG.hold_range_m))[0]
            res[f"latch_{name}_t_minus_cpa"] = float(sim_t[lat[0]] - t_cpa) if lat.size else math.nan
            res[f"latch_{name}_rtrue_at"] = float(rtrue[lat[0]]) if lat.size else math.nan
        vrel_c = vtgt_t[:, c0] - vown_t[:, c0]
        vh = vrel_c / max(np.linalg.norm(vrel_c), 1e-9)
        rp = reltrue[:, c0] - float(np.dot(reltrue[:, c0], vh)) * vh
        res["latch_rperp_true"] = float(np.linalg.norm(rp))
    else:
        res["latch_t_minus_cpa"] = math.nan
    return res, dets, intervals


def main():
    out_dir = os.path.join(REPO, "logs/endgame_diag_20260924")
    os.makedirs(out_dir, exist_ok=True)
    allres, alldets, allint = [], [], []
    for arm in ARMS:
        for k in range(1, 9):
            r, d, iv = analyze(arm, k)
            allres.append(r)
            for x in d:
                x.update(arm=arm, flight=k)
            for x in iv:
                x.update(arm=arm, flight=k)
            alldets += d
            allint += iv
    json.dump(dict(flights=allres, dets=alldets, intervals=allint),
              open(os.path.join(out_dir, "gz_raw.json"), "w"), indent=1, default=float)
    for name, rows in (("gz_flights.csv", allres), ("gz_dets.csv", alldets),
                       ("gz_coast_intervals.csv", allint)):
        keys = list(rows[0].keys())
        for r in rows:
            for kk in r:
                if kk not in keys:
                    keys.append(kk)
        with open(os.path.join(out_dir, name), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    print("wrote", out_dir)


if __name__ == "__main__":
    main()
