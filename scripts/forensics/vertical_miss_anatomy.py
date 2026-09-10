#!/usr/bin/env python3
"""Where does the VERTICAL miss come from -- the terminal, or the dash?

WHY THIS EXISTS (2026-09-09/10). ADR-0095 measured the adopted config's residual
closest approach as 0.374 m VERTICAL against 0.174 m horizontal: 82% of the
SQUARED miss is on an axis the terminal cannot steer, because the only vertical
command anywhere is an altitude-hold P-loop to a preset height and every seeker's
`bearing_vert_rad` is discarded (contradiction
`terminal-vertical-channel-decided-not-built`).

That raises the question this tool answers: is the vertical error GENERATED
during the dash, or is it a terminal failure? It matters because
scripts/forensics/handoff_closing_speed.py then measured only 0.02-0.17 m of
terminal correction capacity on the AprilTag path -- far too little to null a
0.374 m bias. If the error is generated during the dash it must be corrected
DURING THE DASH (a pre-flight trim, like the crossing-bias aim calibration that
already works, ADR-0080/0083), not in a terminal that has no authority left.

THE HYPOTHESIS, AS FIRST WRITTEN AND THEN CORRECTED BY THIS TOOL'S OWN OUTPUT.
The original guess was a SAG: a P-only altitude loop cannot null a persistent
disturbance, the airframe pitches nose-down 27-36 deg during the dash, vertical
thrust falls to about cos(35 deg) = 0.82 of hover, so the vehicle should settle
LOW -- matching the README's "the interceptor flies low".

Run against the committed fleet that is REFUTED IN ITS SPECIFIC FORM and
SUPPORTED IN ITS GENERAL ONE. The interceptor is ABOVE the target on 24 of 24
flights (median +0.485 m) and altitude RISES +0.320 m across the dash. Same sign,
same order -- so the mechanism is real but it is not "sag", it is ALTITUDE-HOLD
DRIFT ACROSS THE DASH, whose direction depends on the configuration (these
cue-era arms fly a running start and a loft, which climb).

So the test below is the GENERAL one: does dash-time altitude drift account for
the vertical miss, in sign and in magnitude? Reporting only "did it sag" would
have returned NOT SUPPORTED on data that supports the mechanism, which is the
wrong-question failure this repo keeps catching in its own scorers.

WHAT IT MEASURES, per flight, from committed per-tick CSVs:
  * vertical separation at CPA          gt_cam_z - gt_tag_z at min gt_range
  * altitude sag during the dash        alt_m at dash end minus pre-dash median
  * whether the two have the same sign and order of magnitude

`gt_*` is ground truth: this is a SCORING/FORENSIC tool and is not on any
guidance path (honesty boundary, CLAUDE.md). `gt_cam_z` is the CAMERA's world z;
per ADR-0095 the camera sits ~2 mm above the airframe datum, so it stands in for
the airframe centre to well inside the numbers here -- the 0.120 m camera offset
is HORIZONTAL and does not enter a vertical measurement.

SCOPE LIMIT. The committed logs are the cue-era two-stage set (phase `DASH`), not
the coded-dash arms ADR-0095 scored -- that archive is gitignored and lives only
on the dev machine. So this tests the MECHANISM on the data available; it does
not reproduce ADR-0095's -0.374 m, which was a different fleet. Run with
`--phase CODED_DASH` on the dev machine to close it.

USAGE
    python3 scripts/forensics/vertical_miss_anatomy.py
    python3 scripts/forensics/vertical_miss_anatomy.py --glob 'logs/mc_fp_*.csv' \
        --phase CODED_DASH
    python3 scripts/forensics/vertical_miss_anatomy.py --self-test

EXIT CODES (docs/error_handling_policy.md)
    0 PASS   measured, verdict printed
    1 FAIL   a file was present and unreadable
    2 USAGE  bad arguments
    3 VACUOUS/UNCERTAIN  nothing measurable -- never a PASS
"""

from __future__ import annotations

import argparse
import csv
import glob as globmod
import statistics
import sys

REQUIRED = ("phase", "gt_range", "gt_cam_z", "gt_tag_z", "alt_m")
# ADR-0095's number, for comparison only. NOT a threshold: it was measured on a
# different (gitignored) fleet, so this tool reports agreement, never asserts it.
ADR0095_VERTICAL_M = -0.374


def _f(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if v == v else None


def anatomy(rows, dash_phase):
    """Per-flight vertical anatomy.

    Returns a dict, or None when the flight carries no usable CPA. Every reason
    for returning None is COUNTED by the caller, never silently absorbed.
    """
    # CPA = minimum true range. Logged tick, not interpolated: this tool is about
    # the vertical BIAS, which is a metre-scale systematic, not the centimetre
    # interpolation term rescore_cpa.py handles.
    best_r, at_cpa = None, None
    for r in rows:
        rng = _f(r.get("gt_range"))
        if rng is None:
            continue
        if best_r is None or rng < best_r:
            best_r, at_cpa = rng, r
    if at_cpa is None:
        return None

    cam_z, tag_z = _f(at_cpa.get("gt_cam_z")), _f(at_cpa.get("gt_tag_z"))
    if cam_z is None or tag_z is None:
        return None
    vert_at_cpa = cam_z - tag_z          # negative = interceptor BELOW target

    # Pre-dash altitude reference: the median of every tick BEFORE the dash
    # starts. Using the median rather than the last sample so one settling
    # transient cannot define the baseline.
    pre, during = [], []
    seen_dash = False
    for r in rows:
        a = _f(r.get("alt_m"))
        if a is None:
            continue
        if r.get("phase") == dash_phase:
            seen_dash = True
            during.append(a)
        elif not seen_dash:
            pre.append(a)

    sag = None
    if pre and during:
        # Sag = how far the END of the dash sits below the pre-dash baseline.
        # The end matters, not the mean: that is the altitude the vehicle
        # actually delivers to the terminal.
        tail = during[-max(1, len(during) // 5):]
        sag = statistics.median(tail) - statistics.median(pre)

    return {"cpa_m": best_r, "vert_at_cpa": vert_at_cpa, "sag": sag,
            "n_pre": len(pre), "n_dash": len(during)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--glob", default="logs/*.csv")
    ap.add_argument("--phase", default=None,
                    help="dash phase name; default = try CODED_DASH then DASH")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)

    if args.self_test:
        return 0 if self_test() else 1

    paths = sorted(globmod.glob(args.glob))
    if not paths:
        print(f"[vertical_miss_anatomy] UNCERTAIN: no files matched {args.glob!r}")
        return 3

    rec, skipped_schema, skipped_nocpa = [], 0, 0
    for p in paths:
        try:
            with open(p, newline="") as fh:
                rd = csv.DictReader(fh)
                if not rd.fieldnames or any(c not in rd.fieldnames for c in REQUIRED):
                    skipped_schema += 1
                    continue
                rows = list(rd)
        except OSError as exc:
            print(f"[vertical_miss_anatomy] ERROR: {p}: {exc}", file=sys.stderr)
            return 1
        phases = {r["phase"] for r in rows}
        dash = args.phase or ("CODED_DASH" if "CODED_DASH" in phases else "DASH")
        a = anatomy(rows, dash)
        if a is None:
            skipped_nocpa += 1
            continue
        a["path"], a["dash_phase"] = p, dash
        rec.append(a)

    if not rec:
        print(f"[vertical_miss_anatomy] UNCERTAIN / VACUOUS: matched {len(paths)} "
              f"file(s), measured 0 ({skipped_schema} lacked {REQUIRED}, "
              f"{skipped_nocpa} had no usable CPA). Nothing was measured.")
        return 3

    verts = [r["vert_at_cpa"] for r in rec]
    sags = [r["sag"] for r in rec if r["sag"] is not None]
    cpas = [r["cpa_m"] for r in rec]

    print(f"[vertical_miss_anatomy] measured {len(rec)} flight(s) of {len(paths)} "
          f"matched ({skipped_schema} on schema, {skipped_nocpa} no CPA)")
    print(f"  dash phase(s): {', '.join(sorted({r['dash_phase'] for r in rec}))}")
    print(f"  CPA (logged tick)            median {statistics.median(cpas):+.3f} m")
    print(f"  vertical at CPA (cam - tag)  median {statistics.median(verts):+.3f} m "
          f"(min {min(verts):+.3f}, max {max(verts):+.3f})")
    n_below = sum(1 for v in verts if v < 0)
    print(f"  interceptor BELOW the target on {n_below}/{len(verts)} flights")
    if sags:
        print(f"  altitude sag over the dash   median {statistics.median(sags):+.3f} m "
              f"(n={len(sags)})")
    else:
        print("  altitude sag over the dash   n/a (no pre-dash baseline in these logs)")
    print(f"  ADR-0095 reference (other fleet, corrected ruler): "
          f"{ADR0095_VERTICAL_M:+.3f} m vertical")

    med_v = statistics.median(verts)
    med_s = statistics.median(sags) if sags else None

    print("  VERDICT:")
    if n_below >= 0.75 * len(verts):
        print(f"    A SYSTEMATIC low bias is present here too: the interceptor is "
              f"below the target on {n_below} of {len(verts)} flights, median "
              f"{med_v:+.3f} m. Same SIGN as ADR-0095.")
    elif n_below <= 0.25 * len(verts):
        print(f"    A systematic HIGH bias ({len(verts) - n_below}/{len(verts)} "
              f"above). Same class of defect, opposite sign -- do not reuse "
              f"ADR-0095's magnitude.")
    else:
        print(f"    NO systematic vertical bias in this fleet ({n_below}/"
              f"{len(verts)} below). The bias ADR-0095 measured may be specific "
              f"to the coded-dash arms; re-run with --phase CODED_DASH.")

    if med_s is None:
        print("    DRIFT MECHANISM: UNTESTED here -- these logs carry no pre-dash "
              "baseline, so the hypothesis is neither supported nor refuted.")
    elif abs(med_s) < 0.05:
        print(f"    DRIFT MECHANISM NOT SUPPORTED: altitude holds to "
              f"{med_s:+.3f} m across the dash, so it cannot account for a "
              f"{med_v:+.3f} m vertical miss. Next candidates: the altitude "
              f"DATUM, or the target's own altitude (ADR-0085).")
    elif (med_s > 0) == (med_v > 0) and abs(med_s) >= 0.4 * abs(med_v):
        share = 100.0 * abs(med_s) / abs(med_v)
        print(f"    DRIFT MECHANISM SUPPORTED: altitude moves {med_s:+.3f} m "
              f"across the dash and the miss is {med_v:+.3f} m -- same sign, and "
              f"the drift is {share:.0f}% of the miss. The vertical error is "
              f"DELIVERED BY THE DASH, so it must be corrected there (a "
              f"pre-flight altitude trim, the analogue of the crossing-bias aim "
              f"calibration) -- NOT in a terminal with 0.02-0.17 m of authority "
              f"on the tag path.")
    else:
        print(f"    DRIFT MECHANISM PARTIAL / OPPOSED: drift {med_s:+.3f} m vs "
              f"miss {med_v:+.3f} m. Signs or magnitudes disagree, so dash "
              f"altitude drift is not the whole story here.")
    print("    SCOPE: cue-era DASH unless the phase line says CODED_DASH. "
          "gt_* is scoring-only. Attitude is NOT logged in these CSVs, so the "
          "pitch-to-sag link is inferred, not measured (deep-audit DEEP-R2).")
    return 0


def self_test():
    ok = True

    def case(name, cond):
        nonlocal ok
        print(f"  [self-test] {name}: {'PASS' if cond else 'FAIL'}")
        ok = ok and cond

    # A flight that hovers at 5 m, sags to 4.6 m through the dash, and passes
    # 0.40 m BELOW a target held at 5 m.
    rows = []
    for i in range(20):                               # pre-dash hover
        rows.append({"phase": "HOLD", "gt_range": f"{30 - i * 0.01:.3f}",
                     "gt_cam_z": "5.000", "gt_tag_z": "5.000", "alt_m": "5.000"})
    expect_cpa = None
    for i in range(50):                               # dash, altitude sagging
        alt = 5.0 - 0.4 * (i / 49.0)
        rng = 25.0 - i * 0.45
        expect_cpa = rng if expect_cpa is None else min(expect_cpa, rng)
        rows.append({"phase": "DASH", "gt_range": f"{rng:.3f}",
                     "gt_cam_z": f"{alt:.3f}", "gt_tag_z": "5.000",
                     "alt_m": f"{alt:.3f}"})
    a = anatomy(rows, "DASH")
    # Expectation COMPUTED from the fixture, not hand-typed -- a hand-typed 2.75
    # here was simply wrong (the real minimum is 2.95) and would have shipped as
    # a failing self-test on a working tool.
    case("CPA found at the minimum range",
         a and abs(a["cpa_m"] - expect_cpa) < 1e-9)
    case("vertical at CPA recovers -0.40 m", a and abs(a["vert_at_cpa"] + 0.40) < 0.02)
    case("sag recovers about -0.40 m", a and a["sag"] is not None
         and abs(a["sag"] + 0.40) < 0.05)

    # No pre-dash ticks: sag must be None, NOT 0.0. Reporting 0.0 would read as
    # "measured, no sag" when the truth is "not measured" -- the fail-closed rule.
    only_dash = [r for r in rows if r["phase"] == "DASH"]
    a2 = anatomy(only_dash, "DASH")
    case("no baseline -> sag is None, not 0.0", a2 and a2["sag"] is None)

    # A flight with no usable ground truth must return None, not a zero verdict.
    a3 = anatomy([{"phase": "DASH", "gt_range": "", "gt_cam_z": "",
                   "gt_tag_z": "", "alt_m": ""}], "DASH")
    case("no CPA -> None (caller counts it)", a3 is None)

    # A level flyby must NOT report a vertical bias.
    level = [{"phase": "DASH", "gt_range": f"{10 - i:.1f}", "gt_cam_z": "5.0",
              "gt_tag_z": "5.0", "alt_m": "5.0"} for i in range(9)]
    a4 = anatomy(level, "DASH")
    case("level flyby reports zero vertical", a4 and abs(a4["vert_at_cpa"]) < 1e-9)

    # A CLIMB must be reported as the same mechanism as a sag. Coding the check
    # as "did it go down" returned NOT SUPPORTED on the real fleet, which climbs.
    climb = []
    for i in range(20):
        climb.append({"phase": "HOLD", "gt_range": "30.0", "gt_cam_z": "5.000",
                      "gt_tag_z": "5.000", "alt_m": "5.000"})
    for i in range(50):
        alt = 5.0 + 0.4 * (i / 49.0)
        climb.append({"phase": "DASH", "gt_range": f"{25.0 - i * 0.45:.3f}",
                      "gt_cam_z": f"{alt:.3f}", "gt_tag_z": "5.000",
                      "alt_m": f"{alt:.3f}"})
    a5 = anatomy(climb, "DASH")
    case("a CLIMB is measured with the same machinery",
         a5 and a5["sag"] is not None and abs(a5["sag"] - 0.40) < 0.05
         and a5["vert_at_cpa"] > 0)

    print(f"  [self-test] {'ALL PASS' if ok else 'FAILURES'}")
    return ok


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(2)
