#!/usr/bin/env python3
"""Measure the TRUE closing speed while the handoff streak is forming.

WHY THIS EXISTS (2026-09-09, builder question "is there really a physics limit
to where the tag can be read and adjustments made in time?").

The tripod money gate computes both the streak burn and the surviving
time-to-go at a single `V_closing = 9.0 m/s`:

    t_go = (R_decode90 - R_streak_burn) / V_closing        >= 0.5 s
    R_streak_burn = E[frames] / fps * V_closing

But the streak forms during CODED_DASH, at `dash_speed_ms = 16.0`
(flight/deploy/real_flight.py). ENGAGE -- which commands `v_close_runin = 9.0`
tapering to 5.5 -- only begins AFTER the streak completes. So the burn may be
paid at dash speed while the gate prices it at the terminal speed, which is the
optimistic direction on a ~$740 purchase decision. Same shape as the invented
30 fps that ADR-0082 caught.

WHAT THIS MEASURES, and its honest scope. It reads the per-tick `gt_range`
against `t_sim` in COMMITTED flight CSVs and reports the closing speed during
the dash, in the last second before handoff, and after handoff. `gt_*` is
ground truth and this is a SCORING/FORENSIC tool -- it is not on any guidance
path (honesty boundary: CLAUDE.md).

SCOPE LIMIT, STATED UP FRONT: the committed logs carry the cue-era two-stage
`DASH` phase, not `CODED_DASH`. The coded-dash per-tick archive is gitignored
and lives only on the dev machine. The MECHANISM is the same in both (the
streak forms while the vehicle is still running in at dash speed), so these
numbers BOUND the question; they do not close it for the coded-dash arms. Run
this on the dev machine over the coded-dash CSVs to close it -- it accepts any
CSV with `t_sim`, `phase` and `gt_range`, and `--phase` selects the dash phase
name.

USAGE
    python3 scripts/forensics/handoff_closing_speed.py                 # committed logs
    python3 scripts/forensics/handoff_closing_speed.py --glob 'logs/mc_fp_*.csv' \
        --phase CODED_DASH                                             # dev machine
    python3 scripts/forensics/handoff_closing_speed.py --self-test     # offline, no data

EXIT CODES (docs/error_handling_policy.md)
    0 PASS   measured, verdict printed
    1 FAIL   a file was present and unreadable/inconsistent
    2 USAGE  bad arguments
    3 VACUOUS/UNCERTAIN  nothing measurable was found -- never a PASS
"""

from __future__ import annotations

import argparse
import csv
import glob as globmod
import statistics
import sys

# The gate's own constants, so a reader can see what is being challenged.
GATE_V_CLOSING = 9.0        # scripts/seeker/tripod_score.py
GATE_TGO_MIN = 0.5          # s
DASH_SPEED_MS = 16.0        # flight/deploy/real_flight.py cfg.dash_speed_ms
REQUIRED = ("t_sim", "phase", "gt_range")


def _f(x):
    """float() that returns None for blanks and non-numbers, never raises."""
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if v == v else None          # drop NaN


def closing_rates(rows, dash_phase, engage_phase="ENGAGE"):
    """Per-segment closing speed (-d gt_range / d t_sim) split by phase.

    Returns (dash, pre_handoff, engage, handoff_range_m). `pre_handoff` is the
    last 1.0 s of sim time before the first engage tick -- the window the
    streak actually forms in. Segments with a non-positive or absurd dt are
    dropped and COUNTED, never silently absorbed (policy rule 5).
    """
    dash, pre, eng = [], [], []
    dropped = 0
    handoff_t = None
    handoff_r = None
    for r in rows:
        if r["phase"] == engage_phase and handoff_t is None:
            handoff_t = _f(r["t_sim"])
            handoff_r = _f(r["gt_range"])

    # ENGAGE must be measured PRE-CPA ONLY. After closest approach the target is
    # receding (the outbound flythrough is ~85% of the ">1 s dropout", ADR-0023),
    # so pooling the whole engage leg returns a NEGATIVE "closing speed" that is
    # arithmetically right and answers the wrong question. Find the minimum-range
    # tick and stop there.
    cpa_t = None
    best = None
    for r in rows:
        rr, tt = _f(r["gt_range"]), _f(r["t_sim"])
        if rr is None or tt is None:
            continue
        if best is None or rr < best:
            best, cpa_t = rr, tt

    for a, b in zip(rows, rows[1:]):
        ta, tb = _f(a["t_sim"]), _f(b["t_sim"])
        ra, rb = _f(a["gt_range"]), _f(b["gt_range"])
        if None in (ta, tb, ra, rb) or not (1e-4 < tb - ta < 1.0):
            dropped += 1
            continue
        rate = -(rb - ra) / (tb - ta)     # positive = closing
        if a["phase"] == dash_phase:
            dash.append(rate)
            if handoff_t is not None and 0.0 <= handoff_t - ta <= 1.0:
                pre.append(rate)
        elif a["phase"] == engage_phase:
            if cpa_t is None or ta <= cpa_t:
                eng.append(rate)
    return dash, pre, eng, handoff_r, dropped


def _med(xs):
    return statistics.median(xs) if xs else None


def _fmt(x, unit=" m/s"):
    return "n/a" if x is None else f"{x:.2f}{unit}"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--glob", default="logs/*.csv",
                    help="CSV glob (default: logs/*.csv)")
    ap.add_argument("--phase", default=None,
                    help="dash phase name; default = try CODED_DASH then DASH")
    ap.add_argument("--self-test", action="store_true",
                    help="offline arithmetic check, needs no data")
    args = ap.parse_args(argv)

    if args.self_test:
        return 0 if self_test() else 1

    paths = sorted(globmod.glob(args.glob))
    if not paths:
        print(f"[handoff_closing_speed] UNCERTAIN: no files matched {args.glob!r}")
        return 3

    per_file = []
    skipped_schema = 0
    dropped_total = 0
    for p in paths:
        try:
            with open(p, newline="") as fh:
                rd = csv.DictReader(fh)
                if not rd.fieldnames or any(c not in rd.fieldnames for c in REQUIRED):
                    skipped_schema += 1
                    continue
                rows = list(rd)
        except OSError as exc:
            print(f"[handoff_closing_speed] ERROR: {p}: {exc}", file=sys.stderr)
            return 1
        phases = {r["phase"] for r in rows}
        dash_phase = args.phase or ("CODED_DASH" if "CODED_DASH" in phases else "DASH")
        if dash_phase not in phases:
            skipped_schema += 1
            continue
        d, pre, eng, hr, dropped = closing_rates(rows, dash_phase)
        dropped_total += dropped
        if not d:
            continue
        per_file.append((p, dash_phase, _med(d), _med(pre), _med(eng), hr))

    # NO VACUOUS VERDICTS: zero measured flights is UNCERTAIN, never a PASS.
    if not per_file:
        print(f"[handoff_closing_speed] UNCERTAIN / VACUOUS: matched {len(paths)} file(s) "
              f"but 0 carried {REQUIRED} plus a dash phase "
              f"({skipped_schema} skipped on schema). Nothing was measured.")
        return 3

    dash_meds = [x[2] for x in per_file if x[2] is not None]
    pre_meds = [x[3] for x in per_file if x[3] is not None]
    eng_meds = [x[4] for x in per_file if x[4] is not None]
    phases_used = sorted({x[1] for x in per_file})

    print(f"[handoff_closing_speed] measured {len(per_file)} flight(s) "
          f"of {len(paths)} matched ({skipped_schema} skipped on schema, "
          f"{dropped_total} segment(s) dropped on dt)")
    print(f"  dash phase(s) used: {', '.join(phases_used)}")
    print(f"  closing speed during the dash        median {_fmt(_med(dash_meds))} "
          f"(n={len(dash_meds)} flights)")
    print(f"  closing speed, last 1.0 s pre-handoff median {_fmt(_med(pre_meds))} "
          f"(n={len(pre_meds)})")
    print(f"  closing speed after handoff, PRE-CPA median {_fmt(_med(eng_meds))} "
          f"(n={len(eng_meds)})")
    print(f"  the gate prices BOTH the burn and t_go at {GATE_V_CLOSING:.1f} m/s")

    v_pre = _med(pre_meds)
    if v_pre is None:
        print("  VERDICT: UNCERTAIN -- no pre-handoff window was measurable.")
        return 3
    ratio = v_pre / GATE_V_CLOSING
    print(f"  ratio (pre-handoff measured / gate assumption) = {ratio:.2f}x")
    if ratio > 1.15:
        print(f"  VERDICT: the gate UNDER-PRICES the streak burn by ~{ratio:.2f}x. "
              f"Re-derive the money gate with the burn at the measured pre-handoff "
              f"speed before spending on the airframe.")
    elif ratio < 0.85:
        print("  VERDICT: the gate OVER-prices the burn (conservative). No action.")
    else:
        print("  VERDICT: the gate assumption matches the measured pre-handoff "
              "closing speed within 15%. No action.")
    print("  SCOPE: cue-era DASH unless the dash phase above says CODED_DASH. "
          "gt_* is scoring-only.")
    return 0


def self_test():
    """Offline: synthetic tracks with known closing speeds must be recovered."""
    ok = True

    def case(name, cond):
        nonlocal ok
        print(f"  [self-test] {name}: {'PASS' if cond else 'FAIL'}")
        ok = ok and cond

    # A track closing at exactly 16 m/s in DASH then 9 m/s in ENGAGE.
    rows = []
    t, r = 0.0, 40.0
    while r > 12.0:                       # dash leg at 16 m/s
        rows.append({"t_sim": f"{t:.3f}", "phase": "DASH", "gt_range": f"{r:.4f}"})
        t += 0.05
        r -= 16.0 * 0.05
    while r > 2.0:                        # engage leg at 9 m/s
        rows.append({"t_sim": f"{t:.3f}", "phase": "ENGAGE", "gt_range": f"{r:.4f}"})
        t += 0.05
        r -= 9.0 * 0.05
    d, pre, eng, hr, dropped = closing_rates(rows, "DASH")
    case("dash leg recovers 16.0 m/s", abs(_med(d) - 16.0) < 1e-6)
    case("engage leg recovers 9.0 m/s", abs(_med(eng) - 9.0) < 1e-6)
    case("pre-handoff window is the dash speed", abs(_med(pre) - 16.0) < 1e-6)
    case("handoff range captured", hr is not None and abs(hr - 12.0) < 0.9)
    case("no segments dropped on a clean track", dropped == 0)

    # Blank and NaN cells must be dropped and COUNTED, never treated as 0.
    dirty = [{"t_sim": "0.00", "phase": "DASH", "gt_range": "20.0"},
             {"t_sim": "",     "phase": "DASH", "gt_range": "19.0"},
             {"t_sim": "0.10", "phase": "DASH", "gt_range": "nan"},
             {"t_sim": "0.15", "phase": "DASH", "gt_range": "17.6"},
             {"t_sim": "0.20", "phase": "DASH", "gt_range": "16.8"}]
    d2, _, _, _, dropped2 = closing_rates(dirty, "DASH")
    case("blank/NaN cells dropped and counted", dropped2 == 3 and len(d2) == 1)
    case("surviving segment is correct", abs(d2[0] - 16.0) < 1e-6)

    # THE PRE-CPA RULE: a receding tail must not drag the engage number negative.
    fly = []
    t, r = 0.0, 20.0
    while r > 3.0:
        fly.append({"t_sim": f"{t:.3f}", "phase": "DASH", "gt_range": f"{r:.4f}"})
        t += 0.05
        r -= 16.0 * 0.05
    while r > 1.0:                        # inbound engage at 9 m/s
        fly.append({"t_sim": f"{t:.3f}", "phase": "ENGAGE", "gt_range": f"{r:.4f}"})
        t += 0.05
        r -= 9.0 * 0.05
    for _ in range(40):                   # outbound flythrough, range grows
        fly.append({"t_sim": f"{t:.3f}", "phase": "ENGAGE", "gt_range": f"{r:.4f}"})
        t += 0.05
        r += 9.0 * 0.05
    _, _, eng4, _, _ = closing_rates(fly, "DASH")
    case("engage excludes the outbound flythrough",
         eng4 and abs(_med(eng4) - 9.0) < 1e-6)
    case("engage median is not negative", eng4 and _med(eng4) > 0)

    # A zero-length dt must not produce an infinite rate.
    zero = [{"t_sim": "1.0", "phase": "DASH", "gt_range": "10.0"},
            {"t_sim": "1.0", "phase": "DASH", "gt_range": "9.0"}]
    d3, _, _, _, dropped3 = closing_rates(zero, "DASH")
    case("zero dt dropped, no divide-by-zero", d3 == [] and dropped3 == 1)

    print(f"  [self-test] {'ALL PASS' if ok else 'FAILURES'}")
    return ok


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(2)
