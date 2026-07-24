#!/usr/bin/env python3
"""Anti-mirage 2x2 verdict for the Phase A loft-dive A/B (ADR-0076 #18g/#18h).

The question the Phase A verdict left OPEN: is the with-camera intercept
CAMERA-DRIVEN, or is the recorded miss just the open-loop coded-dash's
ballistic CPA (a "mirage" — the camera terminal added ~nothing)?

The decisive test is a paired dash-only CONTROL arm: same geometry, same
seeds, but the acquire gate is held shut (--coded-dash-acquire-range-min 999)
so ENGAGE never fires -> the recorded miss is pure dash ballistics through CPA.
Those control flights exit clean=0 / breakoff=python_exit_1 (no camera intercept
ever completed); their miss_m is still the true CPA of the airframe.

Verdict logic, PER DIRECTION, PAIRED by run_idx (seed-matched geometry):
  delta = miss(camera ON) - miss(camera OFF/dash-only)
    delta << 0  -> camera TIGHTENS the miss  -> camera-DRIVEN (not a mirage)
    delta ~ 0   -> camera changed nothing     -> MIRAGE (dash ballistics)
    delta >> 0  -> camera LOOSENS the miss     -> camera steers it WRONG
                                                  (real effect, wrong sign;
                                                   e.g. ADR-0056 aspect bias)
A single median hides the pairing; we report the paired per-flight deltas,
their median + sign-consistency, and a Wilcoxon-style sign tally. No wall time,
no gt in the guidance path — this reads only logged miss_m (a scoring quantity).

Usage:
  antimirage_verdict.py A=logs/mc_loftdive_armA_line9_s123.csv \
                        Adash=logs/mc_loftdive_armAdash_line9_s123.csv \
                        B=logs/mc_loftdive_armB_line9_s123.csv \
                        Bdash=logs/mc_loftdive_armBdash_line9_s123.csv
"""
import csv
import statistics as st
import sys


def load(path):
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r.get("miss_m"):
                rows[int(r["run_idx"])] = {
                    "miss": float(r["miss_m"]),
                    "dir": r["direction"],
                    "clean": r["clean"],
                    "handoff": r.get("handoff") or "",
                }
    return rows


def direction_of(rows, idx):
    return rows[idx]["dir"]


def paired(cam, dash, label):
    """cam = camera-ON arm rows, dash = dash-only control rows."""
    print(f"\n{'='*66}\n{label}: camera-ON vs dash-only control (paired by run_idx)\n{'='*66}")
    for d in ("l2r", "r2l"):
        idxs = sorted(i for i in cam if i in dash and cam[i]["dir"] == d)
        if not idxs:
            print(f"  {d}: no paired flights")
            continue
        deltas = []
        print(f"  {d}:  run  cam_miss  dash_miss   delta(cam-dash)")
        for i in idxs:
            c, dh = cam[i]["miss"], dash[i]["miss"]
            deltas.append(c - dh)
            tag = ""
            if c - dh < -0.3:
                tag = "  <- camera tighter"
            elif c - dh > 0.3:
                tag = "  <- camera worse"
            print(f"       {i:>3}  {c:>7.2f}  {dh:>8.2f}   {c-dh:>+8.2f}{tag}")
        med = st.median(deltas)
        n_tight = sum(1 for x in deltas if x < -0.3)
        n_worse = sum(1 for x in deltas if x > 0.3)
        n_same = len(deltas) - n_tight - n_worse
        cam_med = st.median([cam[i]["miss"] for i in idxs])
        dash_med = st.median([dash[i]["miss"] for i in idxs])
        # plain verdict
        if n_tight >= max(1, len(deltas) - 1) and med < -0.3:
            verd = "CAMERA-DRIVEN (tightens, consistent) -> NOT a mirage"
        elif n_same >= len(deltas) - 1 and abs(med) <= 0.3:
            verd = "MIRAGE: camera ~= dash ballistics (adds nothing)"
        elif n_worse >= max(1, len(deltas) - 1) and med > 0.3:
            verd = "CAMERA STEERS WRONG (real effect, wrong sign; aspect bias?)"
        else:
            verd = "MIXED: no consistent camera effect at this n"
        print(f"    -> n={len(deltas)}  cam_med={cam_med:.2f}  dash_med={dash_med:.2f}  "
              f"paired_delta_med={med:+.2f}")
        print(f"       tighter/worse/same = {n_tight}/{n_worse}/{n_same}   VERDICT: {verd}")


def main(argv):
    arms = {}
    for a in argv[1:]:
        k, _, path = a.partition("=")
        arms[k] = load(path)
    missing = [k for k in ("A", "Adash", "B", "Bdash") if k not in arms]
    if missing:
        print(f"[antimirage] note: missing arms {missing} (pass them to compare)")
    if "A" in arms and "Adash" in arms:
        paired(arms["A"], arms["Adash"], "ARM A (flat dash baseline, no lever)")
    if "B" in arms and "Bdash" in arms:
        paired(arms["B"], arms["Bdash"], "ARM B (loft-dive + accel-cap LEVER)")
    print("\n(reads logged miss_m only; guidance never sees gt. "
          "'dash-only' = --coded-dash-acquire-range-min 999, ENGAGE never fires.)")


if __name__ == "__main__":
    main(sys.argv)
