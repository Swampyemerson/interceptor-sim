"""Measured numbers from the target's ArduPilot logs (.bin, pymavlink DFReader).

    .venv/bin/python runs/flight_log_analysis_20261126/target_numbers.py LOG.bin [...]

Per log: whether it flew, the flight modes, and for the automatic (Auto) part
its ground speed, turn rate, height hold and how steady its reported speed and
course are on straight stretches. Null where a quantity cannot be computed.
"""
import json
import math
import sys

import numpy as np
from pymavlink import DFReader


def load(path):
    r = DFReader.DFReader_binary(path)
    d = {k: [] for k in ("GPS", "MODE", "CTUN", "ATT", "BAT", "EV", "MSG", "ARM")}
    while True:
        m = r.recv_match(type=list(d))
        if m is None:
            break
        d[m.get_type()].append(m.to_dict())
    return d


def col(rows, k):
    return np.array([x[k] for x in rows], dtype=float)


def one(path):
    d = load(path)
    out = {"log": path.split("/")[-1]}
    g = [x for x in d["GPS"] if x.get("I", 0) == 0]
    if len(g) < 20:
        out["note"] = "no GPS records"
        return out
    t = col(g, "TimeUS") / 1e6
    spd, crs, alt = col(g, "Spd"), col(g, "GCrs"), col(g, "Alt")
    sats = col(g, "NSats")
    out["log_s"] = round(float(t[-1] - t[0]), 1)
    out["gps_sats_med"] = float(np.median(sats))
    out["gps_rate_hz"] = round(float(1 / np.median(np.diff(t))), 1)
    modes = [(m["TimeUS"] / 1e6, m.get("Mode"), m.get("ModeNum")) for m in d["MODE"]]
    names = {0: "Stabilize", 2: "AltHold", 3: "Auto", 5: "Loiter", 6: "RTL", 9: "Land"}
    out["modes"] = [[round(a - t[0], 1), names.get(int(c), str(c))] for a, b, c in modes]
    out["messages"] = sorted({m["Message"] for m in d["MSG"] if any(k in m["Message"] for k in
                              ("Arm", "arm", "Fail", "fail", "Mission", "Land", "Crash", "EKF", "Batt"))})[:25]
    ct = d["CTUN"]
    if ct:
        tc = col(ct, "TimeUS") / 1e6
        # this aircraft hovers near 8 % throttle, so "flying" is taken from height
        flying = np.interp(t, tc, (col(ct, "Alt") > 0.3).astype(float)) > 0.5
    else:
        flying = spd > 0.5
    out["flying_s"] = round(float(np.sum(np.diff(t)[flying[:-1]])), 1)
    if out["flying_s"] < 5:
        out["note"] = "did not fly in this log"
        return out
    # the Auto part
    mode_at = np.array([next((b for a, b, c in reversed(modes) if a <= x), None) for x in t])
    num_at = np.array([next((c for a, b, c in reversed(modes) if a <= x), -1) for x in t])
    auto = flying & (num_at == 3)
    out["auto_s"] = round(float(np.sum(np.diff(t)[auto[:-1]])), 1)
    out["speed_ms_flying_med_p95_max"] = [round(float(np.median(spd[flying])), 2), round(float(np.percentile(spd[flying], 95)), 2),
                                          round(float(spd[flying].max()), 2)]
    sel = auto if auto.sum() > 30 else flying
    out["numbers_below_are_for"] = "Auto" if auto.sum() > 30 else "all flying (no Auto part)"
    out["speed_ms_med_p10_p90_max"] = [round(float(np.median(spd[sel])), 2), round(float(np.percentile(spd[sel], 10)), 2),
                                       round(float(np.percentile(spd[sel], 90)), 2), round(float(spd[sel].max()), 2)]
    dcrs = (np.diff(crs) + 180) % 360 - 180
    rate = np.abs(dcrs / np.diff(t))
    moving = sel[:-1] & (spd[:-1] > 0.5)
    if moving.sum() > 20:
        out["turn_rate_deg_s_med_p90_max"] = [round(float(np.median(rate[moving])), 1), round(float(np.percentile(rate[moving], 90)), 1),
                                              round(float(rate[moving].max()), 1)]
        straight = moving & (rate < 5.0)
        out["straight_s"] = round(float(np.sum(np.diff(t)[straight])), 1)
        # steadiness of reported speed and course on straight stretches: scatter about a 3 s running mean
        k = max(3, int(round(3.0 / np.median(np.diff(t)))))
        sm = lambda x: np.convolve(x, np.ones(k) / k, mode="same")   # noqa: E731
        if straight.sum() > 3 * k:
            ds = (spd - sm(spd))[:-1][straight]
            cu = np.unwrap(np.radians(crs))
            dc = np.degrees(cu - sm(cu))[:-1][straight]
            out["on_straights_speed_scatter_ms_rms"] = round(float(np.std(ds)), 3)
            out["on_straights_course_scatter_deg_rms"] = round(float(np.std(dc)), 2)
    if ct:
        dalt, calt = col(ct, "DAlt"), col(ct, "Alt")
        cs = np.interp(tc, t, sel.astype(float)) > 0.5
        if cs.sum() > 50:
            err = (calt - dalt)[cs]
            out["height_m_med"] = round(float(np.median(calt[cs])), 2)
            out["height_error_m_rms_max"] = [round(float(np.sqrt(np.mean(err ** 2))), 2), round(float(np.abs(err).max()), 2)]
    out["gps_height_span_m_while_selected"] = round(float(np.ptp(alt[sel])), 1)
    b = d["BAT"]
    if b:
        out["battery_v_start_min"] = [round(float(b[0]["Volt"]), 2), round(float(min(x["Volt"] for x in b)), 2)]
        out["battery_a_max"] = round(float(max(x.get("Curr", float("nan")) for x in b)), 1)
    return out


if __name__ == "__main__":
    json.dump([one(p) for p in sys.argv[1:]], sys.stdout, indent=1)
