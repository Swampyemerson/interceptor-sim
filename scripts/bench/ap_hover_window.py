"""Still-hover numbers from an ArduPilot .BIN for one or more windows (start:end in log seconds): stick
ranges, mean attitude, rate error, height error, biggest rate-spectrum peak. Prints no positions.

    python.exe ap_hover_window.py log.bin 566:596
"""
import sys, collections, math, statistics as st
import numpy as np
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1]); wins = [(float(a), float(b)) for a, b in (w.split(":") for w in sys.argv[2:])]
lo, hi = min(w[0] for w in wins), max(w[1] for w in wins)
rows = collections.defaultdict(list)
while True:
    x = m.recv_match(type=["ATT", "RATE", "CTUN", "PSCN", "PSCE", "RCIN", "IMU", "BAT", "MAG", "XKF4"])
    if x is None: break
    if lo <= x.TimeUS / 1e6 <= hi: rows[x.get_type()].append(x)
rms = lambda v: math.sqrt(sum(q * q for q in v) / len(v))
for a, b in wins:
    s = lambda k: [x for x in rows[k] if a <= x.TimeUS / 1e6 <= b]
    at, r, c, pn, pe, rc = s("ATT"), s("RATE"), s("CTUN"), s("PSCN"), s("PSCE"), s("RCIN")
    print(f"\n== {a:.0f}..{b:.0f} s")
    print("  sticks: roll", min(x.C1 for x in rc), max(x.C1 for x in rc), "pitch", min(x.C2 for x in rc), max(x.C2 for x in rc), "yaw", min(x.C4 for x in rc), max(x.C4 for x in rc), "thr", min(x.C3 for x in rc), max(x.C3 for x in rc))
    print("  attitude deg: roll mean", round(st.mean(x.Roll for x in at), 2), "sd", round(st.pstdev([x.Roll for x in at]), 2), "| pitch mean", round(st.mean(x.Pitch for x in at), 2), "sd", round(st.pstdev([x.Pitch for x in at]), 2))
    print("  rate error deg/s rms: roll", round(rms([x.RDes - x.R for x in r]), 2), "pitch", round(rms([x.PDes - x.P for x in r]), 2), "| rate rms roll", round(rms([x.R for x in r]), 2), "pitch", round(rms([x.P for x in r]), 2))
    e = [x.Alt - x.DAlt for x in c]; print("  height error m: rms", round(rms(e), 3), "worst", round(max(abs(q) for q in e), 3), "| speed m/s mean", round(st.mean(math.hypot(x.VN, y.VE) for x, y in zip(pn, pe)), 3))
    t = np.array([x.TimeUS / 1e6 for x in r]); fs = 1 / np.median(np.diff(t))
    for nm, f in (("roll", "R"), ("pitch", "P")):
        v = np.array([getattr(x, f) for x in r]); v = v - v.mean(); sp = np.abs(np.fft.rfft(v * np.hanning(len(v)))) * 2 / len(v) * 2; fr = np.fft.rfftfreq(len(v), 1 / fs)
        k = (fr > 5) & (fr < 150); i = np.argmax(sp[k]); print(f"  {nm} rate spectrum: sample rate {fs:.0f} Hz, biggest peak {fr[k][i]:.1f} Hz, amplitude {sp[k][i]:.2f} deg/s")
