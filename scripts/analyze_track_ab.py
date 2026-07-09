#!/usr/bin/env python3
"""ADR-0058 detect-then-track A/B analysis. Reads mc_batch aggregate CSVs and, for
each flight, opens its per-tick flight CSV to compute the STRUCTURAL metrics the
ADR-0057 lesson demands (isolate signal from the ~5 m miss-noise floor):

  (a) HANDOFF-OUTCOME breakdown -- never / REAL (camera tracked the true target
      through the terminal) / RANGE_BIASED_REAL (locked the real target but the
      known-size range runs 5-8 m biased, with ZERO gross false locks -- review
      finding #3: calling these PHANTOM undercounts REAL) / PHANTOM (terminal
      locks include gross false-locks: median err >8 m, or biased median (5,8]
      WITH >8 m outlier dets present).
  (b) TERMINAL detection quality -- ENGAGE meas_range vs gt_range error
      distribution (%GOOD <=3 m err vs %FALSE >8 m err, the ADR-0057 characterization).
  (c) Pk@2.5 m + miss, vs plain markerless AND the AprilTag control -- reported
      THREE ways (review finding #3: miss_m is a whole-flight running min, so
      pooled Pk credits hits banked during the legal cue-guided dash):
        (i)  pooled miss_m Pk (whole-flight, the M5 convention);
        (ii) POST-HANDOFF min gt_range Pk (the camera-terminal metric -- what
             the camera-only terminal itself delivered after the latch);
        (iii) terminal-attributable Pk conditioned on a REAL(-ish) handoff.

Usage: analyze_track_ab.py LABEL=agg.csv [LABEL2=agg2.csv ...]
gt_* is SCORING ONLY here (this is offline analysis of logged runs, never control).
"""
import csv
import math
import os
import sys
from statistics import median, mean

PK_M = 2.5
REAL_ERR_M = 5.0     # median terminal range-err below this -> REAL camera track
GOOD_ERR_M = 3.0     # a single ENGAGE detection this close in range -> GOOD
FALSE_ERR_M = 8.0    # ... this far off -> FALSE (phantom)
REALISH = ("REAL", "RANGE_BIASED_REAL")


def read_agg(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return rows


def flight_terminal(flight_csv):
    """Terminal (ENGAGE) detection quality + the post-handoff camera-terminal
    miss (min gt_range over every row from the first ENGAGE row onward --
    ENGAGE only starts at the handoff latch under --handoff, and the CPA can
    land in ENGAGE or early BREAKOFF)."""
    if not flight_csv or not os.path.exists(flight_csv):
        return None
    abs_errs = []
    n_engage_det = 0
    post_handoff_min_gt = None
    seen_engage = False
    with open(flight_csv) as f:
        for r in csv.DictReader(f):
            phase = r.get("phase")
            if phase == "ENGAGE":
                seen_engage = True
            if seen_engage and phase in ("ENGAGE", "BREAKOFF"):
                try:
                    g = float(r["gt_range"])
                    if post_handoff_min_gt is None or g < post_handoff_min_gt:
                        post_handoff_min_gt = g
                except (ValueError, KeyError, TypeError):
                    pass
            if phase != "ENGAGE" or r.get("detected") != "1":
                continue
            try:
                mr = float(r["meas_range"]); gr = float(r["gt_range"])
            except (ValueError, KeyError, TypeError):
                continue
            n_engage_det += 1
            abs_errs.append(abs(mr - gr))
    out = dict(n_engage_det=n_engage_det, post_min_gt=post_handoff_min_gt,
               med_abs_err=None, n_good=0, n_false=0)
    if abs_errs:
        out.update(
            med_abs_err=median(abs_errs),
            n_good=sum(1 for e in abs_errs if e <= GOOD_ERR_M),
            n_false=sum(1 for e in abs_errs if e > FALSE_ERR_M),
        )
    return out


def classify(handoff, term):
    """Review finding #3 correction: PHANTOM requires EVIDENCE of gross false
    locks (median err > 8 m, or a 5-8 m-biased median accompanied by >8 m
    outlier detections). A 5-8 m median with zero >8 m dets is the known-size
    range BIAS on a real lock, not a phantom -> RANGE_BIASED_REAL."""
    if not handoff:
        return "never"
    if term is None or term["n_engage_det"] == 0:
        return "never_terminal"   # handed off but zero terminal detections (pure coast)
    med = term["med_abs_err"]
    if med <= REAL_ERR_M:
        return "REAL"
    if med <= FALSE_ERR_M and term["n_false"] == 0:
        return "RANGE_BIASED_REAL"
    return "PHANTOM"


def analyze(label, agg_path):
    rows = read_agg(agg_path)
    flights = []
    for r in rows:
        try:
            miss = float(r["miss_m"])
        except (ValueError, KeyError, TypeError):
            miss = float("nan")
        handoff = r.get("handoff") == "1"
        term = flight_terminal(r.get("flight_csv_path"))
        flights.append(dict(
            run=r.get("run_idx"), dir=r.get("direction"), cue=r.get("cue_seed"),
            miss=miss, handoff=handoff, hr=r.get("handoff_range_m"),
            term=term, outcome=classify(handoff, term),
        ))
    return flights


def fmt_pk(vals):
    vals = [v for v in vals if v is not None and not math.isnan(v)]
    return len(vals), sum(1 for v in vals if v <= PK_M), vals


def print_arm(label, flights):
    n, hits, misses = fmt_pk([f["miss"] for f in flights])
    print(f"\n### {label}  (n={n})")
    if misses:
        print(f"  (i)  POOLED miss_m Pk@{PK_M}: {hits}/{n}   median {median(misses):.2f} "
              f"mean {mean(misses):.2f} max {max(misses):.2f} m   "
              f"[whole-flight running min -- credits cue-dash-banked CPAs]")
    # (ii) camera-terminal metric: post-handoff min gt_range
    post = [f["term"]["post_min_gt"] for f in flights
            if f["term"] and f["term"]["post_min_gt"] is not None]
    pn, phits, pv = fmt_pk(post)
    if pv:
        print(f"  (ii) POST-HANDOFF min gt Pk@{PK_M}: {phits}/{pn}   median {median(pv):.2f} "
              f"mean {mean(pv):.2f} max {max(pv):.2f} m   [camera-only terminal delivery]")
    # (iii) terminal-attributable, conditioned on REAL(-ish) handoff
    rpost = [f["term"]["post_min_gt"] for f in flights
             if f["outcome"] in REALISH and f["term"]
             and f["term"]["post_min_gt"] is not None]
    rn, rhits, rv = fmt_pk(rpost)
    if rv:
        print(f"  (iii) REAL-handoff-conditioned post-handoff Pk@{PK_M}: {rhits}/{rn}   "
              f"median {median(rv):.2f} m   [terminal-attributable]")
    # handoff outcome
    oc = {}
    for f in flights:
        oc[f["outcome"]] = oc.get(f["outcome"], 0) + 1
    order = ["REAL", "RANGE_BIASED_REAL", "PHANTOM", "never_terminal", "never"]
    oc_str = "  ".join(f"{k}={oc.get(k,0)}" for k in order if oc.get(k, 0))
    print(f"  HANDOFF OUTCOME: {oc_str}")
    # pooled terminal quality
    tot_det = sum(f["term"]["n_engage_det"] for f in flights if f["term"])
    tot_good = sum(f["term"]["n_good"] for f in flights if f["term"])
    tot_false = sum(f["term"]["n_false"] for f in flights if f["term"])
    print(f"  TERMINAL DETS (pooled ENGAGE): {tot_det} total  "
          f"GOOD(<= {GOOD_ERR_M} m err)={tot_good}  FALSE(> {FALSE_ERR_M} m)={tot_false}")
    # per-flight table
    print("   run dir     miss  postGT  hand  hr     n_eng  medAbsErr  good/false  outcome")
    for f in flights:
        t = f["term"] or {}
        mae = t.get("med_abs_err")
        mae_s = f"{mae:6.1f}" if mae is not None else "   -  "
        pg = t.get("post_min_gt")
        pg_s = f"{pg:6.2f}" if pg is not None else "   -  "
        print(f"   {str(f['run']):>3} {f['dir']:<4} {f['miss']:6.2f}  {pg_s}  "
              f"{'Y' if f['handoff'] else 'n'}   {str(f['hr'] or '-'):>5}  "
              f"{t.get('n_engage_det',0):>4}   {mae_s}    "
              f"{t.get('n_good',0):>2}/{t.get('n_false',0):<3}   {f['outcome']}")


def paired(label_a, fa, label_b, fb):
    """Paired-by-cue_seed miss deltas (A - B)."""
    bmap = {f["cue"]: f for f in fb}
    deltas = []
    print(f"\n### PAIRED {label_a} - {label_b} (by cue_seed)")
    print("   cue      dir   missA   missB   delta   outA / outB")
    for f in fa:
        g = bmap.get(f["cue"])
        if not g or math.isnan(f["miss"]) or math.isnan(g["miss"]):
            continue
        d = f["miss"] - g["miss"]
        deltas.append(d)
        print(f"   {str(f['cue']):>7} {f['dir']:<4} {f['miss']:6.2f}  {g['miss']:6.2f}  "
              f"{d:+6.2f}   {f['outcome']} / {g['outcome']}")
    if deltas:
        print(f"  median delta {median(deltas):+.2f} m  mean {mean(deltas):+.2f} m  "
              f"(negative = A better)  n={len(deltas)}")


if __name__ == "__main__":
    arms = {}
    for a in sys.argv[1:]:
        label, path = a.split("=", 1)
        arms[label] = analyze(label, path)
    for label, flights in arms.items():
        print_arm(label, flights)
    # auto-pair track vs plain if both present
    keys = list(arms)
    for tk in [k for k in keys if "track" in k.lower()]:
        base = tk.lower().replace("track", "plain")
        for bk in keys:
            if bk.lower() == base or (("plain" in bk.lower() or "markerless" in bk.lower())
                                      and bk.split("_")[-1] == tk.split("_")[-1]):
                paired(tk, arms[tk], bk, arms[bk])
                break
