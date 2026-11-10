#!/usr/bin/env python3
"""Turn a hover_recorder.py CSV into a measured wobble: frequency, size, axis.

Reads only the ARMED part of the log (trimmed 2 s at each end to drop takeoff
and landing), removes the slow drift, and finds the strongest oscillation in
each body rate (roll / pitch / yaw) with an FFT. Also reports the motor-output
spread and whether any output sat at a limit.

HOW TO READ THE NUMBER (PX4 multicopter, rule of thumb -- a guide, not a law):
  below ~8 Hz   the ATTITUDE loop (MC_ROLL_P / MC_PITCH_P too high for the rate
                loop underneath it) or something mechanical swinging
  above ~12 Hz  the RATE loop (MC_*RATE_K/P/D too high, or too much delay:
                filters, PWM latency)
  in between    ambiguous -- change one thing and re-measure

FAIL-CLOSED: no armed data, too few samples, or a sample rate too low to see
the band => exit 2 with the reason. Never a default number.

    python3 scripts/bench/hover_wobble_analysis.py hover_....csv
    python3 scripts/bench/hover_wobble_analysis.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import tempfile

import numpy as np

MIN_SECONDS = 5.0
MIN_RATE_HZ = 40.0
TRIM_S = 2.0
BAND = (1.0, 45.0)


def load(path):
    att, out = [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if row["armed"] != "1":
                continue
            if row["msg"] == "ATT":
                att.append([float(row["t_boot_ms"]) / 1000.0, float(row["roll"]),
                            float(row["pitch"]), float(row["p"]), float(row["q"]),
                            float(row["r"])])
            elif row["msg"] == "OUT":
                out.append([float(row["t_boot_ms"]) / 1000.0]
                           + [float(row[k]) for k in ("o1", "o2", "o3", "o4")])
    return np.array(att), np.array(out)


def peak(t, x, fs):
    """Strongest sinusoid in BAND: (freq_hz, amplitude in x's units)."""
    n = len(x)
    ti = t[0] + np.arange(n) / fs
    xi = np.interp(ti, t, x)
    xi = xi - np.polyval(np.polyfit(ti, xi, 2), ti)
    win = np.hanning(n)
    spec = np.abs(np.fft.rfft(xi * win)) * 2.0 / win.sum()
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    sel = (freqs >= BAND[0]) & (freqs <= min(BAND[1], fs / 2 - 1))
    i = np.argmax(spec[sel])
    return float(freqs[sel][i]), float(spec[sel][i]), float(np.std(xi))


def analyse(path):
    att, out = load(path)
    if len(att) == 0:
        return 2, "FAIL: no ARMED attitude rows in the log (was it armed while recording?)"
    t = att[:, 0]
    keep = (t >= t[0] + TRIM_S) & (t <= t[-1] - TRIM_S)
    att = att[keep]
    if len(att) < 10:
        return 2, "FAIL: armed segment shorter than the 2 s trims"
    t = att[:, 0]
    dur = t[-1] - t[0]
    fs = (len(t) - 1) / dur if dur > 0 else 0.0
    if dur < MIN_SECONDS:
        return 2, f"FAIL: only {dur:.1f} s of armed data after trimming (need {MIN_SECONDS:.0f})"
    if fs < MIN_RATE_HZ:
        return 2, (f"FAIL: attitude arrived at {fs:.0f} Hz (need {MIN_RATE_HZ:.0f}); "
                   "the link did not deliver the requested rate")
    lines = [f"armed segment analysed: {dur:.1f} s, {len(t)} samples, {fs:.0f} Hz "
             f"(sees up to ~{min(BAND[1], fs / 2 - 1):.0f} Hz)"]
    res = {}
    for name, col in (("roll", 3), ("pitch", 4), ("yaw", 5)):
        f0, amp, rms = peak(t, att[:, col], fs)
        ang = math.degrees(amp / (2 * math.pi * f0))
        res[name] = (f0, amp, rms)
        lines.append(f"  {name:5s} rate: peak {f0:5.1f} Hz, +/-{math.degrees(amp):6.1f} deg/s "
                     f"(~+/-{ang:.2f} deg of tilt), rms {math.degrees(rms):.1f} deg/s")
    if len(out):
        o = out[(out[:, 0] >= t[0]) & (out[:, 0] <= t[-1])][:, 1:]
        if len(o):
            mean = o.mean(axis=0)
            lines.append("  motor outputs (us): mean " + " ".join(f"{v:.0f}" for v in mean)
                         + f" | spread between motors {mean.max() - mean.min():.0f}"
                         + f" | min {o.min():.0f} max {o.max():.0f}")
    else:
        lines.append("  motor outputs: NONE recorded")
    worst = max(("roll", "pitch"), key=lambda k: res[k][1])
    f0 = res[worst][0]
    band = ("ATTITUDE-loop / mechanical band" if f0 < 8 else
            "RATE-loop band" if f0 > 12 else "ambiguous band")
    lines.append(f"VERDICT: dominant wobble {f0:.1f} Hz on {worst} -> {band}")
    return 0, "\n".join(lines)


def self_test() -> int:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import hover_recorder as hr  # fixture comes from the PRODUCER's own writer

    def make(path, f_hz, amp, armed=True, rate=100.0, secs=20.0):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            hr.write_header(w)
            for i in range(int(secs * rate)):
                ts = i / rate
                p = amp * math.sin(2 * math.pi * f_hz * ts) + 0.01 * ts
                w.writerow(hr.att_row(int(ts * 1000), armed, 0.0, 0.0, 0.0, p, 0.0, 0.0))
                if i % 2 == 0:
                    w.writerow(hr.out_row(int(ts * 1000), armed, (1400, 1420, 1390, 1410)))
    ok = True
    with tempfile.TemporaryDirectory() as d:
        for f_hz in (3.0, 17.0):
            p = os.path.join(d, "a.csv")
            make(p, f_hz, 0.5)
            rc, txt = analyse(p)
            good = rc == 0 and f"{f_hz:.1f} Hz on roll" in txt
            print(f"{'ok  ' if good else 'FAIL'} recovers a {f_hz} Hz roll wobble")
            ok &= good
        p = os.path.join(d, "b.csv")
        make(p, 5.0, 0.5, armed=False)
        rc, _ = analyse(p)
        print(f"{'ok  ' if rc == 2 else 'FAIL'} disarmed-only log fails closed")
        ok &= rc == 2
        make(p, 5.0, 0.5, rate=20.0)
        rc, _ = analyse(p)
        print(f"{'ok  ' if rc == 2 else 'FAIL'} 20 Hz log fails closed")
        ok &= rc == 2
        make(p, 5.0, 0.5, secs=6.0)
        rc, _ = analyse(p)
        print(f"{'ok  ' if rc == 2 else 'FAIL'} too-short log fails closed")
        ok &= rc == 2
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if not a.csv:
        ap.error("give a CSV or --self-test")
    rc, txt = analyse(a.csv)
    print(txt)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
