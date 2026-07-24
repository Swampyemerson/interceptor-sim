#!/usr/bin/env python3
"""measure_aspect_bias.py -- OFFLINE calibration of the terminal LOS aspect bias.

WHAT IT MEASURES
----------------
The markerless seeker's estimated inertial LOS azimuth `lambda_deg` carries a
DIRECTION-DEPENDENT constant offset (the ADR-0056 "aspect bias"): the NN's box
centre does not sit on the target's centroid, and which way it slides depends on
which way the target is presenting (crossing l2r vs r2l).  Measured on ENGAGE
ticks of the flown arms it is roughly +9..+17 deg for l2r and -17..-21 deg for
r2l (docs/flight_plan_candidates.md Sec 1.4).  Because the terminal builds its
velocity command in the LOS frame (u = (cos lambda, sin lambda)), that offset
ROTATES the whole commanded velocity vector -- which is why the camera terminal
can make a well-aimed dash worse (the Sec-SYNTHESIS decomposition).

This script turns that observation into a NUMBER you can paste into
`scripts/m4_intercept.py --terminal-bearing-bias-{l2r,r2l}-deg`.

    bias(dir) = median over ENGAGE ticks of  wrap180( lambda_deg - gt_LOS_deg )

    gt_LOS_deg = degrees(atan2(gt_tag_x - gt_cam_x, gt_tag_y - gt_cam_y))
                 (world ENU: x=east, y=north -> compass azimuth from north,
                  the SAME convention m4 uses for lambda)

HONESTY (the project's #1 rule -- CLAUDE.md "Honesty boundary")
---------------------------------------------------------------
This script reads `gt_*`.  That is legal here and ONLY here, because it runs
**offline, after the flight, on logged CSVs** -- it is a CALIBRATION bench, the
exact analogue of a human labelling a bore-sight target or measuring camera
intrinsics on a checkerboard.  Its OUTPUT is a single pre-flight constant per
crossing direction.  Nothing in this file runs in flight and nothing it produces
gives the guidance loop a live ground-truth read: at inference the terminal sees
the camera bearing plus that one constant.

TWO RULES that keep it honest, and they are not optional:
  1. CALIBRATE AND TEST ON DISJOINT FLIGHTS.  Fitting the constant on seed 123
     and then quoting seed 123 as the win is the fit-and-test mirage this project
     has already been burned by (ADR-0061, memory [[reproduce-canonical-gate-
     geometry]]).  Calibrate on 123, fly the verdict on 777.
  2. The constant must be measured on RAW (uncompensated) flights.  m4 logs the
     UNCORRECTED `lambda_deg` even when the compensation flag is active, so a
     compensated flight re-measured here shows the RESIDUAL-plus-applied bias,
     not zero.  Pass --applied-l2r/--applied-r2l to subtract a known applied
     constant and read the true residual.

USAGE
-----
    .venv/bin/python scripts/experiments/flight_plans/measure_aspect_bias.py \
        logs/mc_fp_armG20_line9_s123.csv logs/mc_loftdive_armB_line9_s123.csv

    # per-flight detail
    ... --per-flight
    # residual check on an already-compensated arm
    ... --applied-l2r 13.0 --applied-r2l -19.0

Accepts either an mc_batch ARM csv (has `flight_csv_path` + `direction` columns)
or bare per-flight m4 CSVs (then pass --direction l2r|r2l).
"""

import argparse
import csv
import math
import os
import statistics
import sys

# Same REAL-detection criterion as scripts/experiments/loft_dive/parse_ab.py
# (coded_dash_summary._is_real_target): the implied range agrees with gt within
# 50%.  A phantom lock has an arbitrary LOS, so pooling phantoms into the median
# would measure the phantom distribution, not the aspect bias.
REAL_RANGE_TOL = 0.5


def fnum(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def wrap180(deg):
    """Wrap an angle in degrees into (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def is_det(row):
    return row.get("detected") in ("1", "True", "true")


def is_real(row):
    mr, gr = fnum(row, "meas_range"), fnum(row, "gt_range")
    return (mr is not None and gr is not None and gr > 0
            and abs(mr - gr) / gr < REAL_RANGE_TOL)


def gt_los_deg(row):
    """Ground-truth LOS azimuth, camera -> target, compass degrees (atan2(E, N)).

    World frame is ENU (x=east, y=north), so this is the same convention as m4's
    lambda = atan2(east, north).  SCORING/CALIBRATION ONLY.
    """
    cx, cy = fnum(row, "gt_cam_x"), fnum(row, "gt_cam_y")
    tx, ty = fnum(row, "gt_tag_x"), fnum(row, "gt_tag_y")
    if None in (cx, cy, tx, ty):
        return None
    de, dn = tx - cx, ty - cy
    if abs(de) < 1e-9 and abs(dn) < 1e-9:
        return None
    return math.degrees(math.atan2(de, dn))


def flight_errors(flight_csv, phase="ENGAGE", band=None):
    """Per-tick (lambda - gt_LOS) errors in degrees for one flight CSV."""
    errs, rng = [], []
    try:
        rows = list(csv.DictReader(open(flight_csv)))
    except OSError as exc:
        print(f"  !! cannot read {flight_csv}: {exc}", file=sys.stderr)
        return errs, rng
    for r in rows:
        if phase and r.get("phase") != phase:
            continue
        if not (is_det(r) and is_real(r)):
            continue
        lam = fnum(r, "lambda_deg")
        gt = gt_los_deg(r)
        gr = fnum(r, "gt_range")
        if lam is None or gt is None:
            continue
        if band is not None and (gr is None or not band[0] <= gr <= band[1]):
            continue
        errs.append(wrap180(lam - gt))
        rng.append(gr)
    return errs, rng


def load_arm(path, forced_direction=None):
    """Yield (label, direction, flight_csv_path) for an arm CSV or a bare flight CSV."""
    with open(path) as fh:
        head = fh.readline()
    if "flight_csv_path" in head:
        for row in csv.DictReader(open(path)):
            fp = row.get("flight_csv_path") or ""
            d = forced_direction or row.get("direction") or "?"
            if fp:
                yield (f"{os.path.basename(path)}#{row.get('run_idx', '?')}", d, fp)
    else:
        yield (os.path.basename(path), forced_direction or "?", path)


def summarize(vals):
    if not vals:
        return None
    v = sorted(vals)
    n = len(v)
    q1 = v[max(0, n // 4)]
    q3 = v[min(n - 1, (3 * n) // 4)]
    return {
        "n": n,
        "median": statistics.median(v),
        "mean": statistics.fmean(v),
        "q1": q1,
        "q3": q3,
        "min": v[0],
        "max": v[-1],
    }


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Measure the per-direction terminal LOS aspect bias (offline "
                    "calibration; gt is used for CALIBRATION ONLY -- see the module "
                    "docstring).")
    ap.add_argument("csvs", nargs="+",
                    help="mc_batch arm CSV(s) (with flight_csv_path+direction) "
                         "and/or bare per-flight m4 CSVs")
    ap.add_argument("--direction", choices=["l2r", "r2l"], default=None,
                    help="force the crossing direction (required for bare flight CSVs)")
    ap.add_argument("--phase", default="ENGAGE",
                    help="phase to measure on (default ENGAGE -- the terminal, which "
                         "is where the constant is applied; use '' for all phases)")
    ap.add_argument("--band", nargs=2, type=float, default=None, metavar=("LO", "HI"),
                    help="restrict to a gt_range band in metres (e.g. --band 2 12)")
    ap.add_argument("--per-flight", action="store_true",
                    help="print the per-flight medians (ticks WITHIN a flight are "
                         "correlated -- the median-of-flight-medians is the honest "
                         "n, the pooled median overstates significance)")
    ap.add_argument("--applied-l2r", type=float, default=0.0,
                    help="a compensation constant already applied in flight (deg); "
                         "subtracted from the l2r measurement to give the RESIDUAL")
    ap.add_argument("--applied-r2l", type=float, default=0.0,
                    help="as --applied-l2r, for r2l")
    ap.add_argument("--loo", action="store_true",
                    help="LEAVE-ONE-FLIGHT-OUT transfer test: fit the constant on "
                         "the OTHER flights of the same direction and score the "
                         "held-out one. This is the offline proxy for the seed-123 "
                         "-> seed-777 transfer, and it is the number that predicts "
                         "whether the compensation will help or be a NULL -- run it "
                         "BEFORE spending sim time.")
    args = ap.parse_args(argv)

    band = tuple(args.band) if args.band else None
    pooled = {"l2r": [], "r2l": []}
    per_flight_med = {"l2r": [], "r2l": []}
    per_flight_errs = {"l2r": [], "r2l": []}
    skipped = []

    print("# terminal LOS aspect bias  (lambda_deg - gt_LOS_deg), phase=%s%s"
          % (args.phase or "ALL", "" if band is None else "  band=%.1f-%.1f m" % band))
    print("# gt is CALIBRATION-ONLY (offline). Output = a PRE-FLIGHT constant.")
    for path in args.csvs:
        for label, direction, flight_csv in load_arm(path, args.direction):
            if direction not in pooled:
                skipped.append((label, f"unknown direction '{direction}'"))
                continue
            errs, _rng = flight_errors(flight_csv, args.phase or None, band)
            if not errs:
                skipped.append((label, "0 REAL %s ticks" % (args.phase or "ALL")))
                continue
            med = statistics.median(errs)
            pooled[direction].extend(errs)
            per_flight_med[direction].append(med)
            per_flight_errs[direction].append((label, errs))
            if args.per_flight:
                print("  %-34s %-4s n=%-4d median=%+7.2f deg   [%+.1f .. %+.1f]"
                      % (label, direction, len(errs), med, min(errs), max(errs)))

    print()
    result = {}
    for d in ("l2r", "r2l"):
        applied = args.applied_l2r if d == "l2r" else args.applied_r2l
        s_pool = summarize(pooled[d])
        s_fl = summarize(per_flight_med[d])
        if s_pool is None:
            print("  %s: NO DATA" % d)
            result[d] = None
            continue
        # The per-flight median-of-medians is the headline number: ticks within a
        # flight are correlated, so the pooled n is not an independent n.
        est = s_fl["median"] + applied
        result[d] = est
        print("  %s  flights=%-2d  median-of-flight-medians=%+7.2f deg "
              "(flight medians %+.1f .. %+.1f)"
              % (d, s_fl["n"], s_fl["median"], s_fl["min"], s_fl["max"]))
        print("       pooled ticks n=%-4d median=%+7.2f  IQR[%+.1f, %+.1f]  "
              "range[%+.1f, %+.1f]"
              % (s_pool["n"], s_pool["median"], s_pool["q1"], s_pool["q3"],
                 s_pool["min"], s_pool["max"]))
        if applied:
            print("       + applied %+.2f deg in flight -> TRUE bias estimate %+.2f deg"
                  % (applied, est))
        # STABILITY VERDICT.  A calibration constant is only a constant if it is
        # the SAME on every flight.  The flown arms split hard on this: the framed
        # ARM B holds +9..+17 / -17..-21 across flights, while the level-camera
        # arms (A, G20) get 1-6 REAL ENGAGE ticks per flight, all inside 5 m where
        # the LOS slews hundreds of deg/s, and their per-flight medians scatter
        # over 100+ deg.  Fitting a "constant" to the latter is fitting noise.
        spread = s_fl["max"] - s_fl["min"]
        same_sign = s_fl["min"] * s_fl["max"] > 0
        if s_fl["n"] >= 3 and spread <= 12.0 and same_sign:
            print("       STABILITY: OK  (flight-median spread %.1f deg over %d "
                  "flights, consistent sign) -> usable as a calibration constant"
                  % (spread, s_fl["n"]))
        else:
            why = []
            if s_fl["n"] < 3:
                why.append("only %d flight(s)" % s_fl["n"])
            if spread > 12.0:
                why.append("flight-median spread %.1f deg" % spread)
            if not same_sign:
                why.append("SIGN FLIPS between flights")
            print("       STABILITY: **NOT STABLE** (%s) -> this is NOT a "
                  "calibration constant; do not fly it." % ", ".join(why))

    if args.loo:
        print("\n# ---- LEAVE-ONE-FLIGHT-OUT transfer (the honest predictor) ----")
        print("#  fit the constant on the OTHER flights of the same direction,")
        print("#  score the held-out flight. If median|err| does not DROP here, the")
        print("#  compensation will not transfer to a disjoint seed either.")
        base_all, comp_all = [], []
        for d in ("l2r", "r2l"):
            fl = per_flight_errs[d]
            if len(fl) < 2:
                print("  %s: need >=2 flights for LOO (have %d)" % (d, len(fl)))
                continue
            for k, (label, errs) in enumerate(fl):
                others = [e for j, (_l, es) in enumerate(fl) if j != k for e in es]
                c = statistics.median(others)
                base = statistics.median([abs(e) for e in errs])
                comp = statistics.median([abs(e - c) for e in errs])
                base_all += [abs(e) for e in errs]
                comp_all += [abs(e - c) for e in errs]
                print("  %-34s %-4s n=%-3d  median|err|  %6.2f -> %6.2f deg  "
                      "(fitted c=%+.1f)" % (label, d, len(errs), base, comp, c))
        if base_all:
            b0 = statistics.median(base_all)
            c0 = statistics.median(comp_all)
            print("  POOLED HELD-OUT: median|err| %.2f -> %.2f deg  (%+.0f%%)"
                  % (b0, c0, 100.0 * (c0 - b0) / b0))
            if c0 >= b0:
                print("  VERDICT: the constant does NOT transfer flight-to-flight "
                      "here -- expect a NULL under sim. Do not spend the arm.")

    if skipped:
        print("\n  skipped: " + "; ".join("%s (%s)" % s for s in skipped))

    if result["l2r"] is not None and result["r2l"] is not None:
        b_l2r, b_r2l = result["l2r"], result["r2l"]
        print("\n# ---- calibration constants (paste into the m4 arm) ----")
        print("#   per-direction (expresses the measured asymmetry exactly):")
        print("    --terminal-bearing-bias-l2r-deg %.1f "
              "--terminal-bearing-bias-r2l-deg %.1f" % (b_l2r, b_r2l))
        sym = (abs(b_l2r) + abs(b_r2l)) / 2.0
        print("#   symmetric fallback (one knob, sign auto-keyed l2r=+ / r2l=-):")
        print("    --terminal-bearing-bias-deg %.1f" % sym)
        if b_l2r <= 0 or b_r2l >= 0:
            print("#   NOTE: the measured signs are NOT the expected (+l2r / -r2l)")
            print("#   pattern -- the symmetric flag would push the WRONG way on at")
            print("#   least one direction. Use the per-direction flags.")
        print("#   REMINDER: calibrate on one seed, fly the verdict on a DISJOINT seed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
