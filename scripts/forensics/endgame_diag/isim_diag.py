#!/usr/bin/env python3
"""Endgame-estimator diagnosis, ISIM half. Scratch harness: matched-optics
port-arm runs with a per-tick recorder wrapped around the built guidance.
No isim/ or flight/ edits. Truth is read by the RECORDER only (scoring), never
handed to guidance.

Arms: nobrake / brake = the registered matched arms (own vel_ned = isim truth,
as the adapter passes it). *_novel = EXPLORATORY counterfactual (not in the
registration): VehicleObs.vel_ned withheld, reproducing the Gazebo driver's
wiring (run_mavsdk_mission never sets vel_ned -> pursuit_terminal's
own_vel_fallback = last commanded velocity).
"""
import contextlib, dataclasses, io, json, math, os, sys
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import numpy as np

REPO = "/home/emerson/interceptor-sim"
sys.path.insert(0, REPO)

BRAKE = {"brake_shaping": True, "brake_accel_ms2": 3.0, "brake_vert_sync": True}
ARMS = {"nobrake": (None, False), "brake": (BRAKE, False),
        "nobrake_novel": (None, True), "brake_novel": (BRAKE, True)}
LAT = 0.045
HOLD = 1.0


def qrot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def run_one(job):
    arm, seed = job
    from isim.scenario import Scenario, build
    from isim.replay_a0 import load_params
    from isim.engine import run_engagement
    ov, novel = ARMS[arm]
    vp = load_params(os.path.join(REPO, "isim/fits/vehicle_gazebo_x500_engage.json"))
    scn = Scenario(concept="flyby", terminal="pursuit", tag_facing="rear", cam_fx_px=540.3,
                   tag_side_m=0.5, seed=seed, port_pursuit_overrides=ov)
    ecfg, vehicle, target, seeker, guidance, init = build(scn, vp)
    guidance.supply_pose_range = True
    seeker.dec = dataclasses.replace(seeker.dec, min_decode_interval_s=0.096)
    mount = np.asarray(seeker.cam.mount_xyz_body, float)

    rows = []
    orig_step = guidance.step

    def rec_step(t, own, det):
        sm = guidance._sm
        if novel and not getattr(sm, "_novel_patched", False):
            sm_step = sm.step

            def step_novel(obs):
                return sm_step(dataclasses.replace(obs, vel_ned=None))
            sm.step = step_novel
            sm._novel_patched = True
        st_pre = str(sm.state)
        cmd = orig_step(t, own, det)
        term = sm.guidance
        kf = term._kf
        tg = target.state(t)
        row = dict(t=t, st=st_pre, phase=term._phase,
                   x=kf.x.copy() if (term._phase == "B" and kf.initialized) else None,
                   det=det is not None,
                   tcap=det.t_capture if det is not None else math.nan,
                   drange=det.range_m if det is not None else math.nan,
                   own=np.asarray(own.pos_ned, float).copy(),
                   vown=np.asarray(own.vel_ned, float).copy(),
                   q=tuple(own.quat_wxyz),
                   tgt=np.asarray(tg.pos_ned, float).copy(),
                   vtgt=np.asarray(tg.vel_ned, float).copy(),
                   cmd=np.array([cmd.v_north, cmd.v_east, cmd.v_down]),
                   prev_cmd=term._prev_v_cmd.copy())
        rows.append(row)
        return cmd

    guidance.step = rec_step
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        res = run_engagement(ecfg, vehicle, target, seeker, guidance, init, record_trace=True)
    tr = res.trace
    return analyze(arm, seed, rows, res, tr, mount)


def analyze(arm, seed, rows, res, tr, mount):
    N = len(rows)
    t = np.array([r["t"] for r in rows])
    phB = np.array([r["x"] is not None and r["st"].endswith("ENGAGE") for r in rows])
    det = np.array([r["det"] for r in rows]) & phB
    rel = np.stack([r["tgt"] - r["own"] for r in rows], axis=1)
    rtrue = np.linalg.norm(rel, axis=0)
    rk = np.full((3, N), np.nan)
    vt = np.full((3, N), np.nan)
    for i, r in enumerate(rows):
        if r["x"] is not None:
            rk[:, i] = r["x"][0:3]
            vt[:, i] = r["x"][3:6]
    rhat = np.linalg.norm(rk, axis=0)
    e = rhat - rtrue
    E = np.linalg.norm(rk - rel, axis=0)
    vown = np.stack([r["vown"] for r in rows], axis=1)
    vtgt = np.stack([r["vtgt"] for r in rows], axis=1)
    t_cpa, cpa = res.t_cpa, res.miss_m
    out = dict(arm=arm, seed=seed, cpa=cpa, t_cpa=t_cpa, n_phB=int(phB.sum()),
               n_det=int(det.sum()))
    if not phB.any():
        out["no_phaseB"] = True
        return out, [], []
    idx = np.where(phB)[0]
    tend = t[idx[-1]]
    last2 = phB & (t >= tend - 2.0)
    pre2 = phB & (t >= t_cpa - 2.0) & (t <= t_cpa)
    # own-velocity input used by the KF at tick i
    novel = arm.endswith("_novel")
    used_v = np.full((3, N), np.nan)
    if novel:
        used_v[:, 1:] = np.stack([r["cmd"] for r in rows], axis=1)[:, :-1]
    else:
        used_v = vown.copy()
    tt = tr["t"]
    op = tr["own_pos"]

    def ownpos_at(tq):
        return np.array([np.interp(tq, tt, op[:, j]) for j in range(3)])
    dets = []
    rdrng = np.gradient(rtrue, t)
    for i in np.where(det)[0]:
        r = rows[i]
        tc = r["tcap"]
        tgt_c = rows[i]["tgt"] - rows[i]["vtgt"] * (r["t"] - tc)   # CV target
        own_c = ownpos_at(tc)
        qi = int(np.argmin(np.abs(tt - tc)))
        cam_c = own_c + qrot(tr["quat"][qi]) @ mount
        r_cam = float(np.linalg.norm(tgt_c - cam_c))
        age = r["t"] - tc
        dets.append(dict(t=r["t"], r_true=float(rtrue[i]), age=age,
                         pose_err_cam=r["drange"] - r_cam,
                         lat_err_m=-(age - LAT) * float(rdrng[i]),
                         e_post=float(e[i]), E_post=float(E[i]),
                         in_last2=bool(last2[i]), in_pre2=bool(pre2[i])))
    intervals = []
    i = 0
    while i < N:
        if det[i]:
            j0 = i
            jj = i + 1
            while jj < N and phB[jj] and not det[jj]:
                jj += 1
            seg = list(range(j0, jj))
            if len(seg) >= 3:
                ts = t[seg] - t[j0]
                intervals.append(dict(t0=t[j0], dur=ts[-1], n=len(seg),
                                      slope=np.polyfit(ts, e[seg] - e[j0], 1)[0],
                                      slopeE=np.polyfit(ts, E[seg] - E[j0], 1)[0],
                                      r0=float(rtrue[j0]),
                                      in_last2=bool(last2[j0] or last2[seg[-1]]),
                                      in_pre2=bool(pre2[j0] or pre2[seg[-1]])))
            i = jj
        else:
            i += 1
    rdot_hat = np.full(N, np.nan)
    rr_mid = np.full(N, np.nan)
    for k in range(1, N):
        if phB[k] and phB[k - 1] and not det[k]:
            rdot_hat[k] = (rhat[k] - rhat[k - 1]) / (t[k] - t[k - 1])
            rr_mid[k] = (rtrue[k] - rtrue[k - 1]) / (t[k] - t[k - 1])
    rdot_err = rdot_hat - rr_mid
    vrel_err = np.linalg.norm((vt - used_v) - (vtgt - vown), axis=0)
    vin_err = np.linalg.norm(used_v - vown, axis=0)
    ci = np.where(phB & (rhat < HOLD))[0]

    def md(a, m):
        a = a[m]
        a = a[np.isfinite(a)]
        return float(np.median(a)) if a.size else math.nan
    out.update(
        t_end=tend, cpa_before_end_s=float(tend - t_cpa),
        e_all_med=md(e, phB), e_last2_med=md(e, last2), e_last2_mean=float(np.nanmean(e[last2])),
        n_last2=int(last2.sum()), e_pre2_med=md(e, pre2), n_pre2=int(pre2.sum()),
        E_last2_med=md(E, last2), E_pre2_med=md(E, pre2),
        n_det_last2=int((det & last2).sum()), n_det_pre2=int((det & pre2).sum()),
        last_det_before_cpa_s=float(t_cpa - max([d["t"] for d in dets if d["t"] <= t_cpa],
                                                default=math.nan)),
        vt_err_pre2_med=md(np.linalg.norm(vt - vtgt, axis=0), pre2),
        vt_err_last2_med=md(np.linalg.norm(vt - vtgt, axis=0), last2),
        vrel_err_pre2_med=md(vrel_err, pre2), vin_err_pre2_med=md(vin_err, pre2),
        vin_err_eng_med=md(vin_err, phB),
        rdot_err_med_pre2=md(rdot_err, pre2), rdot_err_med_last2=md(rdot_err, last2),
        n_rdot_pre2=int(np.isfinite(rdot_err[pre2]).sum()),
        latch_t_minus_cpa=float(t[ci[0]] - t_cpa) if ci.size else math.nan,
        latch_rtrue=float(rtrue[ci[0]]) if ci.size else math.nan,
    )
    dl = np.where(det)[0]
    if dl.size:
        i0 = dl[-1]
        seg = np.arange(i0, idx[-1] + 1)
        seg = seg[phB[seg]]
        if seg.size >= 3:
            out["final_coast_vecE_slope"] = float(np.polyfit(t[seg] - t[i0], E[seg] - E[i0], 1)[0])
            out["final_coast_dur"] = float(t[seg[-1]] - t[i0])
    return out, dets, intervals


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 16
    jobs = [(a, s) for a in ARMS for s in range(n)]
    ctx = get_context("spawn")
    with ProcessPoolExecutor(max_workers=12, mp_context=ctx) as ex:
        outs = list(ex.map(run_one, jobs))
    flights, dets, ints = [], [], []
    for (a, s), (o, d, iv) in zip(jobs, outs):
        flights.append(o)
        for x in d:
            x.update(arm=a, seed=s)
        for x in iv:
            x.update(arm=a, seed=s)
        dets += d
        ints += iv
    od = os.path.join(REPO, "logs/endgame_diag_20260924")
    os.makedirs(od, exist_ok=True)
    json.dump(dict(flights=flights, dets=dets, intervals=ints),
              open(os.path.join(od, "isim_raw.json"), "w"), indent=1, default=float)
    print("done", len(flights))


if __name__ == "__main__":
    main()
