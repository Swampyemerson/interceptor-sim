"""Per-phase numbers from an ArduPilot .BIN. Prints no positions (only errors, speeds, heights above home)."""
import sys, collections, math, statistics as st
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1])
want = ["MODE", "ARM", "MSG", "BAT", "CTUN", "ATT", "RATE", "PSCN", "PSCE", "PSCD", "GPS", "VIBE", "XKF4", "XKF3", "RCIN", "RCOU", "ESC", "EV", "ERR"]
rows = collections.defaultdict(list)
while True:
    x = m.recv_match(type=want)
    if x is None: break
    rows[x.get_type()].append(x)
T = lambda x: x.TimeUS / 1e6
tail = [x for x in rows["MSG"] if T(x) > 480]
for x in tail: 
    if "Reached" not in x.Message and "ChangeSpeed" not in x.Message and " WP" not in x.Message: print("MSG", round(T(x), 1), x.Message)
for x in rows["ERR"]: print("ERR", round(T(x), 1), x.Subsys, x.ECode)
print("PSCN fields:", rows["PSCN"][0].get_fieldnames() if rows["PSCN"] else None)
print("CTUN fields:", rows["CTUN"][0].get_fieldnames())
print("RATE fields:", rows["RATE"][0].get_fieldnames())
print("ESC fields:", rows["ESC"][0].get_fieldnames() if rows["ESC"] else None)
print("VIBE fields:", rows["VIBE"][0].get_fieldnames())
modes = [(T(x), x.Mode) for x in rows["MODE"]]
NAME = {0: "Stabilize", 3: "Auto", 5: "Loiter"}
ct = rows["CTUN"]; air0 = next((T(x) for x in ct if x.Alt > 0.5), None); air1 = next((T(x) for x in reversed(ct) if x.Alt > 0.5), None)
print("airborne (height > 0.5 m) from", round(air0, 1), "to", round(air1, 1), "=", round(air1 - air0, 1), "s")
segs = []
for i, (t, md) in enumerate(modes):
    t1 = modes[i + 1][0] if i + 1 < len(modes) else T(ct[-1])
    a, b = max(t, air0), min(t1, air1)
    if b - a > 1.5: segs.append((a, b, NAME.get(md, md)))
def sel(k, a, b): return [x for x in rows[k] if a <= T(x) <= b]
def rms(v): return math.sqrt(sum(q * q for q in v) / len(v)) if v else float("nan")
def pct(v, p): v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")
for a, b, nm in segs:
    print(f"\n== {nm} {a:.1f}..{b:.1f} ({b - a:.1f} s)")
    c = sel("CTUN", a, b)
    if c:
        e = [x.Alt - x.DAlt for x in c]
        print("  height m: mean", round(st.mean(x.Alt for x in c), 2), "min", round(min(x.Alt for x in c), 2), "max", round(max(x.Alt for x in c), 2), "| height error rms", round(rms(e), 3), "worst", round(max(abs(q) for q in e), 2))
        print("  throttle out: mean", round(st.mean(x.ThO for x in c), 3), "hover est", round(c[-1].ThH, 3), "| climb rate max", round(max(x.CRt for x in c), 2), "min", round(min(x.CRt for x in c), 2))
    pn, pe = sel("PSCN", a, b), sel("PSCE", a, b)
    if pn and pe and len(pn) == len(pe):
        f = pn[0].get_fieldnames()
        tp, p_, tv, v_ = ("TPN", "PN", "TVN", "VN") if "TPN" in f else ("DPN", "PN", "DVN", "VN")
        tpe, pe_, tve, ve_ = [s.replace("N", "E") for s in (tp, p_, tv, v_)]
        err = [math.hypot(getattr(x, tp) - getattr(x, p_), getattr(y, tpe) - getattr(y, pe_)) for x, y in zip(pn, pe)]
        spd = [math.hypot(getattr(x, v_), getattr(y, ve_)) for x, y in zip(pn, pe)]
        tsp = [math.hypot(getattr(x, tv), getattr(y, tve)) for x, y in zip(pn, pe)]
        print("  horizontal position error m: rms", round(rms(err), 2), "p95", round(pct(err, .95), 2), "max", round(max(err), 2))
        print("  ground speed m/s: mean", round(st.mean(spd), 2), "p95", round(pct(spd, .95), 2), "max", round(max(spd), 2), "| commanded speed mean", round(st.mean(tsp), 2), "max", round(max(tsp), 2))
    at = sel("ATT", a, b)
    if at:
        print("  lean deg: roll p95", round(pct([abs(x.Roll) for x in at], .95), 1), "max", round(max(abs(x.Roll) for x in at), 1), "| pitch p95", round(pct([abs(x.Pitch) for x in at], .95), 1), "max", round(max(abs(x.Pitch) for x in at), 1),
              "| attitude error rms roll", round(rms([x.DesRoll - x.Roll for x in at]), 2), "pitch", round(rms([x.DesPitch - x.Pitch for x in at]), 2))
    r = sel("RATE", a, b)
    if r:
        print("  rate error deg/s rms: roll", round(rms([x.RDes - x.R for x in r]), 1), "pitch", round(rms([x.PDes - x.P for x in r]), 1), "yaw", round(rms([x.YDes - x.Y for x in r]), 1),
              "| rate rms: roll", round(rms([x.R for x in r]), 1), "pitch", round(rms([x.P for x in r]), 1))
    v = sel("VIBE", a, b)
    if v: print("  vibration m/s2: x", round(st.mean(x.VibeX for x in v), 1), "y", round(st.mean(x.VibeY for x in v), 1), "z", round(st.mean(x.VibeZ for x in v), 1), "| clipping", v[-1].Clip - v[0].Clip)
    bt = sel("BAT", a, b)
    if bt: print("  battery: V", round(bt[0].Volt, 2), "->", round(bt[-1].Volt, 2), "min", round(min(x.Volt for x in bt), 2), "| current mean A", round(st.mean(x.Curr for x in bt), 1), "| mAh at end", round(bt[-1].CurrTot))
    g = sel("GPS", a, b)
    if g: print("  gps: sats min", min(x.NSats for x in g), "hdop max", round(max(x.HDop for x in g), 2))
    k = sel("XKF4", a, b)
    if k:
        k0 = [x for x in k if getattr(x, "C", 0) == 0]
        if k0: print("  estimator test ratios (1.0 = limit) max: velocity", round(max(x.SV for x in k0), 2), "position", round(max(x.SP for x in k0), 2), "height", round(max(x.SH for x in k0), 2), "compass", round(max(x.SM for x in k0), 2))
    o = sel("RCOU", a, b)
    if o: print("  motor outputs us: mean", [round(st.mean(getattr(x, f"C{i}") for x in o)) for i in (1, 2, 3, 4)], "max", max(max(x.C1, x.C2, x.C3, x.C4) for x in o))
b = rows["BAT"]; print("\nbattery whole log: start", round(b[0].Volt, 2), "min", round(min(x.Volt for x in b), 2), "end", round(b[-1].Volt, 2), "| mAh", round(b[-1].CurrTot))
