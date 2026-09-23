#!/usr/bin/env python3
"""Tick-trace diagnosis of the 2026-09-23 Gazebo pursuit cross-check
(logs/xcheck_gz_20260923_fixed, f1..f8) against the five REGISTERED suspects of
docs/xcheck_gazebo_pursuit_prereg.md -- the follow-up the prereg names after the
n=8 flights FAILED the bar (median CPA 1.510 m vs the 0.122 m matched-optics
isim prediction).

Method (the parity_trace_pursuit.py approach, adapted to offline Gazebo CSVs:
per-tick truth vs what the guidance was HANDED, then per-suspect effect sizes):

  S1 latency      consume_sim_t - det_t_capture_sim per consumed detection.
                  The driver loop polls/logs FIRST, then detects, then steps the
                  SM (real_flight.run_mavsdk_mission), so the detection whose
                  consumed-count increment is VISIBLE at row k was decoded and
                  consumed during tick k-1: consume_t = sim_t[k-1] (a lower
                  bound; sim advances during the in-loop decode). Effect size =
                  (age - 0.045 s assumed) * |v_rel| at consume time.
  S2 measurement  det ranges vs TRUE range at capture time, interpolated from
                  the gtpose ground-truth (scoring-only data). Bias + spread,
                  compared against _pursuit_measurement_noise's along-LOS sigma
                  at the same ranges (tag 0.5 m as flown, and 0.30 m for the
                  "shaped on 0.30" contrast).
  S3 cadence      sim_t tick deltas (ENGAGE vs STANDBY, detect vs non-detect
                  ticks) vs isim's 20 ms guidance tick; consumed-detection rate
                  vs the isim prediction's own decode rate.
  S4 response     replay the LOGGED setpoint stream through isim's fitted
                  QuadVelocityModel (isim/fits/vehicle_gazebo_x500.json) from
                  the ENGAGE-entry state and compare against the Gazebo-true
                  velocity/position; plus a per-axis first-order (delay, tau)
                  fit of achieved velocity vs command. Also EKF-vs-truth
                  altitude datum, and the KF's r_hat vs true range.
  S5 endgame      miss-vector decomposition at CPA (vertical/horizontal,
                  along/cross relative velocity), range-rate at SAFE entry,
                  re-approach structure after the first local minimum.

HONESTY: forensics/scoring only. gt/gtpose columns were never fed to guidance
in the flights; here they grade it after the fact.

Usage:
  python3 scripts/forensics/xcheck_tick_trace.py \
      [--log-dir /home/emerson/interceptor-sim/logs/xcheck_gz_20260923_fixed]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

LAT_ASSUMED_S = 0.045          # PursuitTerminalConfig.meas_latency_s
ISIM_GUIDANCE_DT = 0.02        # isim.engine.EngagementConfig.guidance_dt
ISIM_SEEKER_FPS = 38.2         # isim.seeker.CameraParams.fps
FX = 540.3                     # gz mono_cam (CameraInfo; prediction used 540.3)
TAG_FLOWN_M = 0.5
SIGMA_SIDE_PX = 0.3 * math.sqrt(2.0)   # PursuitTerminalConfig.sigma_side_px
WINDOW_S = 25.0                # driver --window-s (CPA scoring window)


def _f(x):
    return math.nan if x in ("", None) else float(x)


def load_flight(log_dir: str, k: int):
    base = os.path.join(log_dir, f"f{k}")
    with open(base + ".csv") as fh:
        drv = list(csv.DictReader(fh))
    with open(base + "_rf.csv") as fh:
        rf = list(csv.DictReader(fh))
    with open(base + "_gtpose.csv") as fh:
        gt = list(csv.DictReader(fh))
    log = open(os.path.join(log_dir, f"run_f{k}.log")).read()
    return drv, rf, gt, log


def gt_interp(gt):
    """-> (t, rel_ned[3xN] interpolators). ENU world -> NED: n=y, e=x, d=-z."""
    t = np.array([float(r["sim_t"]) for r in gt])
    rel = np.stack([
        np.array([float(r["gt_tag_y"]) - float(r["gt_veh_y"]) for r in gt]),
        np.array([float(r["gt_tag_x"]) - float(r["gt_veh_x"]) for r in gt]),
        np.array([-(float(r["gt_tag_z"]) - float(r["gt_veh_z"])) for r in gt]),
    ])
    return t, rel


def interp_vec(tq, t, vec):
    return np.stack([np.interp(tq, t, vec[i]) for i in range(vec.shape[0])])


def smooth(x, n=5):
    if len(x) < n:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode="same")


def med(a):
    a = np.asarray([v for v in a if not (isinstance(v, float) and math.isnan(v))],
                   dtype=np.float64)
    return float(np.median(a)) if a.size else math.nan


def pct(a, q):
    a = np.asarray([v for v in a if not (isinstance(v, float) and math.isnan(v))],
                   dtype=np.float64)
    return float(np.percentile(a, q)) if a.size else math.nan


def first_order_fit(t, cmd, ach, delays, taus):
    """Best (delay, tau, rms) for  v' = (cmd(t - delay) - v)/tau  on grid t.
    Rows with NaN in cmd or ach are dropped (kept on a common grid)."""
    ok = ~(np.isnan(cmd) | np.isnan(ach))
    t, cmd, ach = t[ok], cmd[ok], ach[ok]
    if len(t) < 10:
        return (math.nan, math.nan, math.nan)
    best = (math.nan, math.nan, math.inf)
    for d in delays:
        cmd_d = np.interp(t - d, t, cmd)
        for tau in taus:
            v = np.empty_like(ach)
            v[0] = ach[0]
            for i in range(1, len(t)):
                dt = max(1e-3, t[i] - t[i - 1])
                a = dt / max(tau, 1e-3)
                v[i] = v[i - 1] + (cmd_d[i - 1] - v[i - 1]) * min(1.0, a)
            rms = float(np.sqrt(np.mean((v - ach) ** 2)))
            if rms < best[2]:
                best = (d, tau, rms)
    return best


def analyze_flight(k, drv, rf, gt, log):
    out = {"flight": k}
    gt_t, rel = gt_interp(gt)
    rng_gt = np.linalg.norm(rel, axis=0)
    vrel = np.gradient(rel, gt_t, axis=1)          # relative velocity, NED
    rr_gt = np.gradient(smooth(rng_gt, 9), gt_t)   # range rate

    sim_t = np.array([_f(r["sim_t"]) for r in drv])
    state = [r["sm_state_pre_step"] for r in drv]
    ncons = np.array([int(r["n_det_consumed"]) for r in drv])
    eng = np.array([s == "ENGAGE" for s in state])
    stand = np.array([s == "STANDBY" for s in state])
    i_go = int(np.argmax(eng)) if eng.any() else None
    go_t = sim_t[i_go]
    out["go_sim_t"] = go_t

    # ---------------- S1: true render->consume latency -----------------------
    ages, age_errs, bias_m, det_rows = [], [], [], []
    for i in range(1, len(drv)):
        if ncons[i] > ncons[i - 1] and drv[i]["det_t_capture_sim"]:
            cap = _f(drv[i]["det_t_capture_sim"])
            consume = sim_t[i - 1]                 # decoded during tick i-1
            age = consume - cap
            ages.append(age)
            age_errs.append(age - LAT_ASSUMED_S)
            v = np.linalg.norm(interp_vec(np.array([consume]), gt_t, vrel)[:, 0])
            bias_m.append((age - LAT_ASSUMED_S) * v)
            det_rows.append(i)
    out["s1"] = {
        "n_det": len(ages), "age_med_s": med(ages), "age_p90_s": pct(ages, 90),
        "age_err_med_s": med(age_errs),
        "stale_bias_med_m": med(bias_m), "stale_bias_p90_m": pct(bias_m, 90),
    }

    # ---------------- S2: measurement quality vs truth at capture ------------
    res_pose, res_depth, model_sig, ranges = [], [], [], []
    for i in det_rows:
        cap = _f(drv[i]["det_t_capture_sim"])
        r_true = float(np.interp(cap, gt_t, rng_gt))
        rp = _f(drv[i]["det_tagpose_range_m"])
        rd = _f(drv[i]["det_range_m"])
        res_pose.append(rp - r_true)
        res_depth.append(rd - r_true)
        ranges.append(r_true)
        model_sig.append(r_true ** 2 * SIGMA_SIDE_PX / (FX * TAG_FLOWN_M))
    # approximate the range the GUIDANCE reconstructs: depth / cos(off-axis),
    # with off-axis estimated from own yaw vs the true LOS (no pitch/roll --
    # a rough bound, flagged as such in the spec).
    yaw_rf = np.array([_f(r["yaw_deg"]) for r in rf])
    res_corr = []
    for j, i in enumerate(det_rows):
        cap = _f(drv[i]["det_t_capture_sim"])
        los = interp_vec(np.array([cap]), gt_t, rel)[:, 0]
        yaw = math.radians(np.interp(cap, sim_t[~np.isnan(yaw_rf)],
                                     yaw_rf[~np.isnan(yaw_rf)]))
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        cosa = max(0.2, float(np.dot(los / max(np.linalg.norm(los), 1e-9), fwd)))
        rd = _f(drv[i]["det_range_m"])
        res_corr.append(rd / cosa - ranges[j])
    out["s2"] = {
        "pose_bias_med_m": med(res_pose),
        "pose_spread_p16_84_m": (pct(res_pose, 84) - pct(res_pose, 16)) / 2.0
        if res_pose else math.nan,
        "depth_bias_med_m": med(res_depth),
        "corr_bias_med_m": med(res_corr),
        "range_med_m": med(ranges),
        "model_along_sigma_med_m": med(model_sig),
    }
    out["_res_pose"] = list(zip(ranges, res_pose, res_depth, res_corr))
    # consumed-frame capture-stamp spacing (effective decoded-frame cadence)
    caps = [_f(drv[i]["det_t_capture_sim"]) for i in det_rows]
    out["s2"]["cap_spacing_med_s"] = med(np.diff(caps)) if len(caps) > 2 else math.nan

    # ---------------- S3: tick cadence ---------------------------------------
    dts = np.diff(sim_t)
    eng_dts = dts[eng[1:] & eng[:-1]]
    stand_dts = dts[stand[1:] & stand[:-1]]
    det_tick = np.zeros(len(drv), dtype=bool)
    for i in det_rows:
        det_tick[i - 1] = True                     # detect ran during tick i-1
    det_dts = dts[det_tick[:-1] & eng[:-1]]
    nodet_dts = dts[(~det_tick[:-1]) & eng[1:] & eng[:-1]]
    t_eng = sim_t[eng][-1] - go_t if eng.sum() > 1 else math.nan
    out["s3"] = {
        "dt_standby_med_s": med(stand_dts), "dt_engage_med_s": med(eng_dts),
        "dt_engage_p90_s": pct(eng_dts, 90), "dt_engage_max_s": float(np.max(eng_dts))
        if eng_dts.size else math.nan,
        "dt_det_tick_med_s": med(det_dts), "dt_nodet_tick_med_s": med(nodet_dts),
        "engage_dur_s": t_eng,
        "det_rate_engage_hz": len(det_rows) / t_eng if t_eng and t_eng > 0 else math.nan,
    }

    # ---------------- S4: command tracking (replay + first-order fit) --------
    # merge drv/rf by index (same ticks; verify)
    assert len(drv) == len(rf), f"f{k}: drv {len(drv)} vs rf {len(rf)} rows"
    for a, b in zip(drv[:5], rf[:5]):
        assert abs(_f(a["mission_t"]) - _f(b["t_s"])) < 2e-3
    own = np.stack([
        np.array([_f(r["own_n_gz"]) for r in drv]),
        np.array([_f(r["own_e_gz"]) for r in drv]),
        np.array([_f(r["own_d_gz"]) for r in drv]),
    ])
    v_ach = np.stack([smooth(np.gradient(own[i], sim_t), 5) for i in range(3)])
    cmd = np.stack([
        np.array([_f(r["v_north"]) for r in rf]),
        np.array([_f(r["v_east"]) for r in rf]),
        np.array([_f(r["v_down"]) for r in rf]),
    ])
    yaw_cmd = np.array([_f(r["yaw_cmd_deg"]) for r in rf])

    idx = np.where(eng)[0]
    i0, i1 = idx[0], idx[-1]
    tt = sim_t[i0:i1 + 1]
    fits = {}
    delays = np.arange(0.0, 0.45, 0.03)
    taus = np.arange(0.06, 1.1, 0.06)
    for ax, name in enumerate("ned"):
        d, tau, rms = first_order_fit(tt, cmd[ax, i0:i1 + 1], v_ach[ax, i0:i1 + 1],
                                      delays, taus)
        fits[name] = {"delay_s": d, "tau_s": tau, "rms_ms": rms}
    vel_track_rms = float(np.sqrt(np.mean(
        np.sum((cmd[:, i0:i1 + 1] - v_ach[:, i0:i1 + 1]) ** 2, axis=0))))

    # replay through isim's fitted vehicle model
    from isim.vehicle import QuadVelocityModel, VehicleParams
    from isim.types import VehicleState, VelCmd
    fit = json.load(open(os.path.join(_REPO, "isim", "fits",
                                      "vehicle_gazebo_x500.json")))
    vp = VehicleParams(**{f: v for f, v in fit["params"].items()
                          if f in VehicleParams.__dataclass_fields__})
    vp.wind_ned = tuple(vp.wind_ned)
    model = QuadVelocityModel(vp)
    yaw0 = math.radians(_f(rf[i0]["yaw_deg"]) if rf[i0]["yaw_deg"] else 0.0)
    st0 = VehicleState(t=tt[0], pos_ned=own[:, i0].copy(), vel_ned=v_ach[:, i0].copy(),
                       quat_wxyz=(1.0, 0.0, 0.0, 0.0), yaw_rad=yaw0, saturated=False)
    model.reset(st0, np.random.default_rng(0))
    mvel = np.zeros((3, i1 - i0 + 1))
    mpos = np.zeros((3, i1 - i0 + 1))
    mvel[:, 0], mpos[:, 0] = v_ach[:, i0], own[:, i0]
    stm = st0
    for j in range(1, i1 - i0 + 1):
        dt = max(1e-3, tt[j] - tt[j - 1])
        c = VelCmd(v_north=float(cmd[0, i0 + j - 1]), v_east=float(cmd[1, i0 + j - 1]),
                   v_down=float(cmd[2, i0 + j - 1]),
                   yaw_deg=float(yaw_cmd[i0 + j - 1]) if not math.isnan(
                       yaw_cmd[i0 + j - 1]) else math.degrees(stm.yaw_rad))
        stm = model.step(c, dt)
        mvel[:, j], mpos[:, j] = stm.vel_ned, stm.pos_ned
    replay_vel_rms = float(np.sqrt(np.nanmean(
        np.sum((mvel - v_ach[:, i0:i1 + 1]) ** 2, axis=0))))
    replay_pos_end = float(np.linalg.norm(mpos[:, -1] - own[:, i1]))
    # what CPA would the model trajectory have had against the true target?
    tgt = np.stack([interp_vec(tt, gt_t, rel)[i] + own[i, i0:i1 + 1]
                    for i in range(3)])  # true target NED = rel + true own
    rng_model = np.linalg.norm(tgt - mpos, axis=0)
    # the isim model's OWN command-following, same fit -> apples-to-apples lag
    mfits = {}
    for ax, name in enumerate("ned"):
        d, tau, rms = first_order_fit(tt, cmd[ax, i0:i1 + 1], mvel[ax], delays, taus)
        mfits[name] = {"delay_s": d, "tau_s": tau, "rms_ms": rms}
    model_track_rms = float(np.sqrt(np.nanmean(
        np.sum((cmd[:, i0:i1 + 1] - mvel) ** 2, axis=0))))
    # ANCHORED replay: re-seed the model from the TRUE state every 1.0 s and
    # measure the 1.0 s-horizon velocity/position prediction error -- model
    # mismatch without open-loop compounding.
    anch_vel, anch_pos = [], []
    j = 0
    while j < i1 - i0:
        j_end = j
        while j_end < i1 - i0 and tt[j_end] - tt[j] < 1.0:
            j_end += 1
        if np.any(np.isnan(v_ach[:, i0 + j])) or np.any(np.isnan(own[:, i0 + j])):
            j = j_end
            continue
        m2 = QuadVelocityModel(vp)
        m2.reset(VehicleState(t=tt[j], pos_ned=own[:, i0 + j].copy(),
                              vel_ned=v_ach[:, i0 + j].copy(),
                              quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                              yaw_rad=math.radians(_f(rf[i0 + j]["yaw_deg"]) or 0.0),
                              saturated=False), np.random.default_rng(0))
        s2m = None
        for q in range(j + 1, j_end + 1):
            dt = max(1e-3, tt[q] - tt[q - 1])
            c = VelCmd(v_north=float(cmd[0, i0 + q - 1]),
                       v_east=float(cmd[1, i0 + q - 1]),
                       v_down=float(cmd[2, i0 + q - 1]),
                       yaw_deg=float(yaw_cmd[i0 + q - 1]) if not math.isnan(
                           yaw_cmd[i0 + q - 1]) else 0.0)
            s2m = m2.step(c, dt)
        if s2m is not None and not np.any(np.isnan(v_ach[:, i0 + j_end])):
            anch_vel.append(float(np.linalg.norm(
                s2m.vel_ned - v_ach[:, i0 + j_end])))
            anch_pos.append(float(np.linalg.norm(
                s2m.pos_ned - own[:, i0 + j_end])))
        j = j_end
    out["s4"] = {
        "fits": fits, "model_fits": mfits, "vel_track_rms_ms": vel_track_rms,
        "model_track_rms_ms": model_track_rms,
        "replay_vel_rms_ms": replay_vel_rms, "replay_pos_err_end_m": replay_pos_end,
        "cpa_model_replay_m": float(np.min(rng_model)),
        "anch1s_vel_err_med_ms": med(anch_vel),
        "anch1s_pos_err_med_m": med(anch_pos),
        "isim_model_latency_s": vp.latency_s,
    }
    # EKF altitude datum vs gz truth (whole flight, airborne rows)
    alt_ekf = np.array([_f(r["alt_m"]) for r in rf])
    ok = ~np.isnan(alt_ekf)
    alt_err = alt_ekf[ok] - (-own[2, ok])
    out["s4"]["alt_ekf_minus_gz_med_m"] = med(alt_err)
    # KF r_hat vs true range (phase-B ticks)
    rhat = np.array([_f(r["r_hat_m"]) for r in rf])
    bidx = np.where(~np.isnan(rhat) & eng)[0]
    if bidx.size:
        r_true_b = np.interp(sim_t[bidx], gt_t, rng_gt)
        e = rhat[bidx] - r_true_b
        out["s4"]["rhat_err_med_m"] = med(e)
        out["s4"]["rhat_err_last2s_med_m"] = med(
            e[sim_t[bidx] >= sim_t[bidx][-1] - 2.0])
        out["phaseB_t0"] = float(sim_t[bidx[0]])
        # r_hat error at INBOUND true-range crossings (clean of post-CPA junk)
        out["s4"]["rhat_err_at_crossing_m"] = {}
        for rx in (6.0, 4.0, 3.0, 2.0):
            hit = [i for q, i in enumerate(bidx[:-1])
                   if r_true_b[q] >= rx > r_true_b[q + 1]]
            if hit:
                i = hit[0]
                out["s4"]["rhat_err_at_crossing_m"][str(rx)] = float(
                    rhat[i] - np.interp(sim_t[i], gt_t, rng_gt))
    # COAST latch: first phase-B tick with terminal_coast set -- from then on
    # the previous command is HELD (steering stops). Where was the truth?
    coast = np.array([r["terminal_coast"] in ("1", "True") for r in rf])
    ci = np.where(coast & eng)[0]
    if ci.size:
        t_c = float(sim_t[ci[0]])
        r_true_c = float(np.interp(t_c, gt_t, rng_gt))
        vrel_c = interp_vec(np.array([t_c]), gt_t, vrel)[:, 0]
        rel_c = interp_vec(np.array([t_c]), gt_t, rel)[:, 0]
        vh = vrel_c / max(np.linalg.norm(vrel_c), 1e-9)
        rperp_c = rel_c - float(np.dot(rel_c, vh)) * vh
        out["coast"] = {
            "t": t_c, "r_true_m": r_true_c,
            "r_hat_m": _f(rf[ci[0]]["r_hat_m"]),
            "n_ticks": int(coast[eng].sum()),
            "rr_ms": float(np.interp(t_c, gt_t, rr_gt)),
            "rperp_true_m": float(np.linalg.norm(rperp_c)),
        }
    else:
        out["coast"] = None

    # final-approach kinematics at the last inbound 6 m crossing: commanded vs
    # achieved closing rate (negative = closing) and true cross-track offset.
    tvel = np.gradient(np.stack([
        np.array([float(r["gt_tag_y"]) for r in gt]),
        np.array([float(r["gt_tag_x"]) for r in gt]),
        np.array([-float(r["gt_tag_z"]) for r in gt])]), gt_t, axis=1)
    m0 = (gt_t >= go_t) & (gt_t <= go_t + WINDOW_S)
    j_cpa0 = int(np.argmin(np.where(m0, rng_gt, np.inf)))
    x6 = [i for i in range(len(gt_t) - 1)
          if go_t < gt_t[i] < gt_t[j_cpa0] and rng_gt[i] >= 6.0 > rng_gt[i + 1]]
    out["x6"] = None
    if x6:
        i = x6[-1]
        los = rel[:, i] / rng_gt[i]
        it = int(np.argmin(np.abs(sim_t - gt_t[i])))
        vt = tvel[:, i]
        vh = vrel[:, i] / max(np.linalg.norm(vrel[:, i]), 1e-9)
        rp = rel[:, i] - float(np.dot(rel[:, i], vh)) * vh
        out["x6"] = {
            "cmd_close_ms": float(np.dot(cmd[:, it] - vt, los)),
            "ach_close_ms": float(np.dot(v_ach[:, it] - vt, los)),
            "rperp_true_m": float(np.linalg.norm(rp)),
            "r_hat_m": float(rhat[it]) if not math.isnan(rhat[it]) else math.nan,
        }

    # ---------------- S5: endgame geometry -----------------------------------
    m = (gt_t >= go_t) & (gt_t <= go_t + WINDOW_S)
    ti = gt_t[m]
    ri = rel[:, m]
    rni = np.linalg.norm(ri, axis=0)
    j_cpa = int(np.argmin(rni))
    t_cpa = float(ti[j_cpa])
    miss_vec = ri[:, j_cpa]
    cpa = float(rni[j_cpa])
    vrel_cpa = interp_vec(np.array([t_cpa]), gt_t, vrel)[:, 0]
    vhat = vrel_cpa / max(np.linalg.norm(vrel_cpa), 1e-9)
    along = float(np.dot(miss_vec, vhat))
    cross_vec = miss_vec - along * vhat
    # SAFE entry
    rf_state = [r["state"] for r in rf]
    i_safe = next((i for i, s in enumerate(rf_state) if s == "SAFE"), None)
    t_safe = float(sim_t[i_safe]) if i_safe is not None else math.nan
    rr_safe = float(np.interp(t_safe - 0.3, gt_t, rr_gt)) if i_safe is not None \
        else math.nan
    # re-approach: range structure after CPA
    after = rni[j_cpa:]
    reclose = float(np.min(after[int(2.0 / 0.02):])) if after.size > 100 else math.nan
    det_ts = [sim_t[i - 1] for i in det_rows]
    pre = [t for t in det_ts if t <= t_cpa]
    last_det_t = pre[-1] if pre else math.nan
    reason = re.search(r"TERMINATED in SAFE \(reason=(.*?)\)", log)
    xres = re.search(r"XCHECK_RESULT cpa_m=([\d.]+) t_cpa=([\d.]+)", log)
    out["s5"] = {
        "cpa_m": cpa, "cpa_reported_m": float(xres.group(1)) if xres else math.nan,
        "t_cpa": t_cpa, "t_safe": t_safe, "safe_after_cpa_s": t_safe - t_cpa,
        "miss_n": float(miss_vec[0]), "miss_e": float(miss_vec[1]),
        "miss_d": float(miss_vec[2]),
        "miss_vert_m": abs(float(miss_vec[2])),
        "miss_horiz_m": float(np.hypot(miss_vec[0], miss_vec[1])),
        "along_m": along, "cross_m": float(np.linalg.norm(cross_vec)),
        "rr_at_safe_ms": rr_safe,
        "range_at_safe_m": float(np.interp(t_safe, gt_t, rng_gt))
        if not math.isnan(t_safe) else math.nan,
        "min_range_after_cpa_plus2s_m": reclose,
        "last_det_before_cpa_s": t_cpa - last_det_t,
        "safe_reason": reason.group(1) if reason else "",
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir",
                    default="/home/emerson/interceptor-sim/logs/xcheck_gz_20260923_fixed")
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    flights = []
    for k in range(1, 9):
        drv, rf, gt, log = load_flight(args.log_dir, k)
        flights.append(analyze_flight(k, drv, rf, gt, log))

    def row(label, fn, fmt="{:8.3f}"):
        vals = [fn(f) for f in flights]
        pooled = med(vals)
        print(f"{label:<38}" + "".join(
            "     nan" if isinstance(v, float) and math.isnan(v) else fmt.format(v)
            for v in vals) + "  | med " + (fmt.format(pooled)))

    print("flight                                " + "".join(f"      f{f['flight']}"
                                                             for f in flights))
    print("\n== S1 latency (assumed 0.045 s) ==")
    row("n detections", lambda f: f["s1"]["n_det"], "{:8.0f}")
    row("age median [s]", lambda f: f["s1"]["age_med_s"])
    row("age p90 [s]", lambda f: f["s1"]["age_p90_s"])
    row("staleness bias med [m]", lambda f: f["s1"]["stale_bias_med_m"])
    row("staleness bias p90 [m]", lambda f: f["s1"]["stale_bias_p90_m"])

    print("\n== S2 measurement vs truth at capture ==")
    row("range at det median [m]", lambda f: f["s2"]["range_med_m"])
    row("pose-range bias med [m]", lambda f: f["s2"]["pose_bias_med_m"])
    row("pose-range spread(1s) [m]", lambda f: f["s2"]["pose_spread_p16_84_m"])
    row("depth-range bias med [m]", lambda f: f["s2"]["depth_bias_med_m"])
    row("yaw-corrected bias med [m]", lambda f: f["s2"]["corr_bias_med_m"])
    row("model along-sigma med [m]", lambda f: f["s2"]["model_along_sigma_med_m"])
    row("consumed-frame spacing med [s]", lambda f: f["s2"]["cap_spacing_med_s"])
    # pooled residual-by-range bins
    allres = [t for f in flights for t in f["_res_pose"]]
    print("pooled range residuals by true range (pose|depth|yaw-corrected):")
    for lo, hi in [(0, 3), (3, 6), (6, 10), (10, 30)]:
        sel = [r for r in allres if lo <= r[0] < hi]
        if sel:
            b = med([s[1] for s in sel])
            sp = (pct([s[1] for s in sel], 84) - pct([s[1] for s in sel], 16)) / 2
            bd = med([s[2] for s in sel])
            bc = med([s[3] for s in sel])
            print(f"  {lo:>2}-{hi:<2} m  n={len(sel):3d}  pose {b:+.3f} "
                  f"(spread {sp:.3f})   depth {bd:+.3f}   corrected {bc:+.3f}")

    print("\n== S3 tick cadence (isim: 0.020 s guidance tick, ~25 det/s) ==")
    row("dt STANDBY med [s]", lambda f: f["s3"]["dt_standby_med_s"])
    row("dt ENGAGE med [s]", lambda f: f["s3"]["dt_engage_med_s"])
    row("dt ENGAGE p90 [s]", lambda f: f["s3"]["dt_engage_p90_s"])
    row("dt ENGAGE max [s]", lambda f: f["s3"]["dt_engage_max_s"])
    row("dt detect-tick med [s]", lambda f: f["s3"]["dt_det_tick_med_s"])
    row("dt non-detect-tick med [s]", lambda f: f["s3"]["dt_nodet_tick_med_s"])
    row("ENGAGE duration [s]", lambda f: f["s3"]["engage_dur_s"])
    row("consumed det rate [Hz]", lambda f: f["s3"]["det_rate_engage_hz"])

    print("\n== S4 command tracking / vehicle+EKF ==")
    for ax in "ned":
        row(f"1st-order delay {ax} [s]", lambda f, a=ax: f["s4"]["fits"][a]["delay_s"])
        row(f"1st-order tau {ax} [s]", lambda f, a=ax: f["s4"]["fits"][a]["tau_s"])
    for ax in "ned":
        row(f"isim-model delay {ax} [s]",
            lambda f, a=ax: f["s4"]["model_fits"][a]["delay_s"])
        row(f"isim-model tau {ax} [s]",
            lambda f, a=ax: f["s4"]["model_fits"][a]["tau_s"])
    row("cmd-vs-achieved RMS [m/s]", lambda f: f["s4"]["vel_track_rms_ms"])
    row("cmd-vs-isim-model RMS [m/s]", lambda f: f["s4"]["model_track_rms_ms"])
    row("anchored-1s vel err med [m/s]", lambda f: f["s4"]["anch1s_vel_err_med_ms"])
    row("anchored-1s pos err med [m]", lambda f: f["s4"]["anch1s_pos_err_med_m"])
    row("isim-model replay vel RMS [m/s]", lambda f: f["s4"]["replay_vel_rms_ms"])
    row("EKF alt - gz alt med [m]", lambda f: f["s4"]["alt_ekf_minus_gz_med_m"])
    row("KF r_hat err med [m]", lambda f: f["s4"].get("rhat_err_med_m", math.nan))
    row("KF r_hat err last2s med [m]",
        lambda f: f["s4"].get("rhat_err_last2s_med_m", math.nan))
    for rx in ("6.0", "4.0", "3.0", "2.0"):
        row(f"r_hat err at true {rx} m inbound",
            lambda f, r=rx: f["s4"].get("rhat_err_at_crossing_m", {}).get(
                r, math.nan))

    print("\n== COAST latch (steering stops when r_hat < 1.0 m) ==")
    row("coast onset r_true [m]",
        lambda f: f["coast"]["r_true_m"] if f["coast"] else math.nan)
    row("coast onset r_hat [m]",
        lambda f: (f["coast"]["r_hat_m"] if f["coast"] else math.nan) or math.nan)
    row("coast onset range rate [m/s]",
        lambda f: f["coast"]["rr_ms"] if f["coast"] else math.nan)
    row("coast onset r_perp true [m]",
        lambda f: f["coast"]["rperp_true_m"] if f["coast"] else math.nan)
    row("coast ticks", lambda f: f["coast"]["n_ticks"] if f["coast"] else 0, "{:8.0f}")

    print("\n== final approach at the last inbound 6 m crossing ==")
    row("commanded d(range)/dt [m/s]",
        lambda f: f["x6"]["cmd_close_ms"] if f["x6"] else math.nan)
    row("achieved d(range)/dt [m/s]",
        lambda f: f["x6"]["ach_close_ms"] if f["x6"] else math.nan)
    row("true cross-track offset [m]",
        lambda f: f["x6"]["rperp_true_m"] if f["x6"] else math.nan)
    row("r_hat at 6 m [m]", lambda f: f["x6"]["r_hat_m"] if f["x6"] else math.nan)

    # cross-flight correlation: KF range bias vs CPA
    xs = np.array([f["s4"].get("rhat_err_med_m", math.nan) for f in flights])
    ys = np.array([f["s5"]["cpa_m"] for f in flights])
    ok = ~(np.isnan(xs) | np.isnan(ys))
    if ok.sum() > 2:
        r = float(np.corrcoef(xs[ok], ys[ok])[0, 1])
        print(f"\ncross-flight: corr(KF r_hat bias, CPA) = {r:+.2f} (n={ok.sum()})")

    print("\n== S5 endgame ==")
    row("CPA (this script) [m]", lambda f: f["s5"]["cpa_m"])
    row("CPA (driver-reported) [m]", lambda f: f["s5"]["cpa_reported_m"])
    row("miss vertical |d| [m]", lambda f: f["s5"]["miss_vert_m"])
    row("miss horizontal [m]", lambda f: f["s5"]["miss_horiz_m"])
    row("miss along v_rel [m]", lambda f: f["s5"]["along_m"])
    row("miss cross v_rel [m]", lambda f: f["s5"]["cross_m"])
    row("last det before CPA [s]", lambda f: f["s5"]["last_det_before_cpa_s"])
    row("SAFE after CPA [s]", lambda f: f["s5"]["safe_after_cpa_s"])
    row("range rate at SAFE [m/s]", lambda f: f["s5"]["rr_at_safe_ms"])
    row("range at SAFE [m]", lambda f: f["s5"]["range_at_safe_m"])
    row("min range CPA+2s..end [m]", lambda f: f["s5"]["min_range_after_cpa_plus2s_m"])
    for f in flights:
        print(f"  f{f['flight']}: safe_reason = {f['s5']['safe_reason']}")

    if args.json_out:
        for f in flights:
            f.pop("_res_pose", None)
        with open(args.json_out, "w") as fh:
            json.dump(flights, fh, indent=1)
        print(f"\nwrote {args.json_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
