"""Per-flight measured numbers from the interceptor's PX4 logs (pyulog).

    .venv/bin/python runs/flight_log_analysis_20261126/flight_numbers.py LOG.ulg [...] > out.json

For each log: the airborne span, flight modes, GPS quality, estimator drops and
resets, speed / lean / thrust / battery extremes, pitch against forward speed,
position wander with the sticks centred in Position mode, and how the velocity
follows its setpoint in Position mode. A quantity that cannot be computed is
null, never a default.
"""
import json
import math
import sys

import numpy as np
from pyulog import ULog

NAV = {0: "Manual", 1: "Altitude", 2: "Position", 3: "Mission", 4: "Hold", 5: "Return",
       10: "Acro", 14: "Offboard", 15: "Stabilized", 17: "Takeoff", 18: "Land"}
TOPICS = ["vehicle_local_position", "vehicle_status", "sensor_gps", "battery_status",
          "manual_control_setpoint", "vehicle_local_position_setpoint",
          "estimator_status_flags", "actuator_motors", "vehicle_land_detected",
          "hover_thrust_estimate", "vehicle_attitude", "vehicle_acceleration",
          "estimator_status", "sensor_combined", "vehicle_global_position"]


def topic(u, name, mid=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == mid:
            return d.data
    return None


def tsec(d):
    return d["timestamp"] / 1e6


def at(t_src, x_src, t):
    """Sample-and-hold lookup of x at times t."""
    i = np.clip(np.searchsorted(t_src, t, side="right") - 1, 0, len(t_src) - 1)
    return x_src[i]


def spans(mask, t, min_s=0.0):
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if (not m or i == len(mask) - 1) and start is not None:
            end = i if not m else i + 1
            if t[end - 1] - t[start] >= min_s:
                out.append((start, end))
            start = None
    return out


def pct(x, q):
    return None if len(x) == 0 else float(np.percentile(x, q))


def r(x, n=2):
    return None if x is None or (isinstance(x, float) and not math.isfinite(x)) else round(float(x), n)


def one(path):
    u = ULog(path, TOPICS)
    out = {"log": path.split("/")[-1]}
    lp, st, gps = topic(u, "vehicle_local_position"), topic(u, "vehicle_status"), topic(u, "sensor_gps")
    ld = topic(u, "vehicle_land_detected")
    t = tsec(lp)
    landed = at(tsec(ld), ld["landed"], t).astype(bool)
    air = ~landed
    out["log_s"] = r(t[-1] - t[0], 1)
    out["airborne_s"] = r(float(np.sum(np.diff(t)[air[:-1]])), 1)
    if out["airborne_s"] < 3:
        air = np.ones_like(air)         # a ground log: use everything
        out["note"] = "never airborne: numbers are for the whole log on the ground"
    nav = at(tsec(st), st["nav_state"], t)
    modes = {}
    for k in np.unique(nav[air]):
        modes[NAV.get(int(k), str(int(k)))] = r(float(np.sum(np.diff(t)[(air & (nav == k))[:-1]])), 1)
    out["mode_seconds_airborne"] = modes

    # ---- GPS receiver
    tg = tsec(gps)
    ga = at(t, air, tg).astype(bool)
    out["gps"] = {
        "sats_min_med_max": [int(gps["satellites_used"][ga].min()), r(np.median(gps["satellites_used"][ga]), 0),
                             int(gps["satellites_used"][ga].max())],
        "speed_acc_ms_med_p95_max": [r(np.median(gps["s_variance_m_s"][ga])), r(pct(gps["s_variance_m_s"][ga], 95)),
                                     r(gps["s_variance_m_s"][ga].max())],
        "horiz_acc_m_med_max": [r(np.median(gps["eph"][ga])), r(gps["eph"][ga].max())],
        "vert_acc_m_med_max": [r(np.median(gps["epv"][ga])), r(gps["epv"][ga].max())],
        "fix_min": int(gps["fix_type"][ga].min()),
        "jam_indicator_med_max": [r(np.median(gps["jamming_indicator"][ga]), 0), int(gps["jamming_indicator"][ga].max())],
        "noise_med": r(np.median(gps["noise_per_ms"][ga]), 0),
    }
    # ---- estimator
    xyv = lp["xy_valid"].astype(bool)
    bad = air & ~xyv
    out["estimator"] = {
        "position_invalid_s_airborne": r(float(np.sum(np.diff(t)[bad[:-1]])), 1),
        "position_invalid_episodes": len(spans(bad, t)),
        "dead_reckoning_s_airborne": r(float(np.sum(np.diff(t)[(air & lp["dead_reckoning"].astype(bool))[:-1]])), 1),
        "xy_resets": int(lp["xy_reset_counter"][-1] - lp["xy_reset_counter"][0]),
        "vxy_resets": int(lp["vxy_reset_counter"][-1] - lp["vxy_reset_counter"][0]),
        "z_resets": int(lp["z_reset_counter"][-1] - lp["z_reset_counter"][0]),
        "heading_resets": int(lp["heading_reset_counter"][-1] - lp["heading_reset_counter"][0]),
    }
    es = topic(u, "estimator_status")
    if es is not None:
        ea = at(t, air, tsec(es)).astype(bool)
        out["estimator"]["gps_vel_test_ratio_med_max"] = [r(np.median(es["vel_test_ratio"][ea])), r(es["vel_test_ratio"][ea].max())]
        out["estimator"]["gps_pos_test_ratio_med_max"] = [r(np.median(es["pos_test_ratio"][ea])), r(es["pos_test_ratio"][ea].max())]
        out["estimator"]["mag_strength_G_med"] = r(np.median(es["mag_strength_gs"][ea]))
        out["estimator"]["mag_strength_ref_G"] = r(np.median(es["mag_strength_ref_gs"][ea]))
    ef = topic(u, "estimator_status_flags")
    if ef is not None:
        fa = at(t, air, tsec(ef)).astype(bool)
        out["estimator"]["mag_fault_seen"] = bool(ef["cs_mag_fault"][fa].any())
        out["estimator"]["gnss_pos_fused_frac"] = r(float(ef["cs_gnss_pos"][fa].mean()))
        out["estimator"]["baro_hgt_frac"] = r(float(ef["cs_baro_hgt"][fa].mean()))
        out["estimator"]["gps_hgt_frac"] = r(float(ef["cs_gps_hgt"][fa].mean()))
    # ---- height: barometer-referenced estimate against the GPS receiver
    gp = topic(u, "vehicle_global_position")
    if gp is not None:
        alt_est = at(tsec(gp), gp["alt"], tg)
        d = gps["altitude_msl_m"] - alt_est
        out["height_gps_minus_pixhawk_m_med_min_max"] = [r(np.median(d), 1), r(d.min(), 1), r(d.max(), 1)]
    # ---- speed, lean, climb
    sp = np.hypot(lp["vx"], lp["vy"])
    att = topic(u, "vehicle_attitude")
    ta = tsec(att)
    q0, q1, q2, q3 = (att[f"q[{i}]"] for i in range(4))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)))
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q1 * q1 + q2 * q2), -1, 1)))
    aa = at(t, air, ta).astype(bool)
    okv = air & lp["v_xy_valid"].astype(bool)
    out["motion"] = {
        "ground_speed_ms_max_p95": [r(sp[okv].max()) if okv.any() else None, r(pct(sp[okv], 95))],
        "climb_ms_max": r(float((-lp["vz"][air]).max())), "descent_ms_max": r(float(lp["vz"][air].max())),
        "height_above_takeoff_m_max": r(float((-(lp["z"][air] - lp["z"][~air][0] if (~air).any() else lp["z"][air])).max()), 1),
        "tilt_deg_max_p95": [r(tilt[aa].max(), 1), r(pct(tilt[aa], 95), 1)],
        "horiz_accel_ms2_max_p95": [r(float(np.hypot(lp["ax"], lp["ay"])[air].max())), r(pct(np.hypot(lp["ax"], lp["ay"])[air], 95))],
    }
    # ---- pitch against forward (nose-direction) speed
    hd = at(t, lp["heading"], ta)
    vf = at(t, lp["vx"], ta) * np.cos(hd) + at(t, lp["vy"], ta) * np.sin(hd)
    vok = at(t, okv, ta).astype(bool)
    bins = [(-0.5, 0.5), (0.5, 2), (2, 4), (4, 6), (6, 9), (9, 13), (13, 40)]
    pv = []
    for lo, hi in bins:
        m = vok & (vf >= lo) & (vf < hi)
        if m.sum() > 100:
            pv.append({"forward_speed_ms": [lo, hi], "seconds": r(m.sum() * np.median(np.diff(ta)), 1),
                       "pitch_deg_med": r(np.median(pitch[m]), 1), "pitch_deg_p10": r(pct(pitch[m], 10), 1),
                       "pitch_deg_p90": r(pct(pitch[m], 90), 1)})
    out["pitch_vs_forward_speed"] = pv
    out["pitch_deg_most_nose_down"] = r(pitch[aa].min(), 1)
    # ---- thrust, battery
    am, bat = topic(u, "actuator_motors"), topic(u, "battery_status")
    mm = np.nanmean(np.vstack([am[f"control[{i}]"] for i in range(4)]), axis=0)
    ma = at(t, air, tsec(am)).astype(bool)
    ba = at(t, air, tsec(bat)).astype(bool)
    acc = topic(u, "vehicle_acceleration")
    sf = np.sqrt(acc["xyz[0]"] ** 2 + acc["xyz[1]"] ** 2 + acc["xyz[2]"] ** 2) / 9.80665
    k = 25
    sfs = np.convolve(sf, np.ones(k) / k, mode="same")
    ca = at(t, air, tsec(acc)).astype(bool)
    hte = topic(u, "hover_thrust_estimate")
    out["power"] = {
        "motor_cmd_mean4_max_p95_med": [r(mm[ma].max()), r(pct(mm[ma], 95)), r(np.median(mm[ma]))],
        "hover_thrust_estimate_med": None if hte is None else r(np.median(hte["hover_thrust"][at(t, air, tsec(hte)).astype(bool)]), 3),
        "specific_force_g_max_smoothed": r(sfs[ca].max()),
        "battery_v_rest_before": r(float(np.median(bat["voltage_v"][:10]))),
        "battery_v_min_airborne": r(float(bat["voltage_v"][ba].min())),
        "battery_a_max_med": [r(float(bat["current_a"][ba].max()), 1), r(float(np.median(bat["current_a"][ba])), 1)],
        "mah_used": r(float(bat["discharged_mah"][-1] - bat["discharged_mah"][0]), 0),
    }
    if ba.sum() > 50 and np.ptp(bat["current_a"][ba]) > 10:
        A = np.vstack([bat["current_a"][ba], np.ones(ba.sum())]).T
        slope, icpt = np.linalg.lstsq(A, bat["voltage_v"][ba], rcond=None)[0]
        out["power"]["pack_resistance_mohm_fit"] = r(-slope * 1000, 0)
    # ---- Position-mode hold with the sticks centred
    mc = topic(u, "manual_control_setpoint")
    stick = np.maximum.reduce([np.abs(mc["roll"]), np.abs(mc["pitch"]), np.abs(mc["yaw"])])
    centred = at(tsec(mc), stick, t) < 0.05
    thr_mid = np.abs(at(tsec(mc), mc["throttle"], t)) < 0.15
    hold = air & (nav == 2) & centred & xyv
    holds = []
    for a, b in spans(hold, t, min_s=6.0):
        i0 = a + int(np.searchsorted(t[a:b], t[a] + 2.0))      # let it stop first
        if b - i0 < 50:
            continue
        x, y, z = lp["x"][i0:b], lp["y"][i0:b], lp["z"][i0:b]
        holds.append({"t_s": [r(t[i0] - t[0], 1), r(t[b - 1] - t[0], 1)], "seconds": r(t[b - 1] - t[i0], 1),
                      "horiz_wander_rms_m": r(float(np.sqrt(np.var(x) + np.var(y)))),
                      "horiz_span_m": r(float(max(np.ptp(x), np.ptp(y)))),
                      "height_span_m": r(float(np.ptp(z))) if thr_mid[i0:b].all() else None,
                      "speed_rms_ms": r(float(np.sqrt(np.mean(sp[i0:b] ** 2))))})
    out["position_hold_sticks_centred"] = holds
    # GPS raw against the estimate during those holds: how far the receiver itself wanders
    # ---- velocity following its setpoint, Position mode
    ls = topic(u, "vehicle_local_position_setpoint")
    posm = air & (nav == 2) & lp["v_xy_valid"].astype(bool)
    foll = None
    if posm.sum() > 400:
        tu = np.arange(t[posm][0], t[posm][-1], 0.02)
        keep = at(t, posm, tu).astype(bool)
        res = {}
        for ax in ("vx", "vy"):
            c = np.interp(tu, tsec(ls), np.nan_to_num(ls[ax]))
            v = np.interp(tu, t, lp[ax])
            c, v = c[keep], v[keep]
            if np.std(c) < 0.3:
                continue
            best = (None, -2)
            for lag in range(0, 60):
                cc = np.corrcoef(c[:len(c) - lag], v[lag:])[0, 1]
                if cc > best[1]:
                    best = (lag * 0.02, cc)
            lag_n = int(round(best[0] / 0.02))
            gain = float(np.dot(c[:len(c) - lag_n], v[lag_n:]) / np.dot(c[:len(c) - lag_n], c[:len(c) - lag_n]))
            res[ax] = {"lag_s": r(best[0]), "corr": r(best[1]), "gain": r(gain), "setpoint_rms_ms": r(float(np.std(c)))}
        foll = res or None
    out["velocity_follows_setpoint_position_mode"] = foll
    # ---- messages worth a line
    seen = {}
    for m in u.logged_messages:
        txt = m.message.split("] ", 1)[-1]
        if any(k in txt for k in ("Failsafe", "invalid", "reset", "GPS", "mag", "Mag", "Position", "position", "Disarm", "Armed", "Takeoff", "Landing", "battery", "Battery")) \
                and "Kill switch" not in txt and "SD Card" not in txt and "termination" not in txt:
            seen[txt] = seen.get(txt, 0) + 1
    out["board_messages"] = seen
    return out


if __name__ == "__main__":
    json.dump([one(p) for p in sys.argv[1:]], sys.stdout, indent=1)
