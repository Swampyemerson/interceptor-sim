#!/usr/bin/env python3
"""How far can the monocular RANGE channel actually see before it goes blind?

WHY THIS EXISTS (2026-09-11). Prompted by arXiv:2409.17497, "Precise
Interception Flight Targets by Image-based Visual Servoing of Multicopter"
(Yan et al., IEEE T-IE 72(11), 2025), whose central move is to control in IMAGE
SPACE rather than reconstructing the target's 3-D position first. That is only
an interesting idea for this project if our own 3-D reconstruction is actually
the weak link, so this tool measures it instead of assuming it.

WHAT IT MEASURES, from committed per-tick CSVs, on ticks that carry a FRESH
detection: the ratio of the seeker's `meas_range` to the true `gt_range`, binned
by TRUE range. Plus two summary quantities:

  * the CEILING -- the high percentiles of `meas_range` itself. A range channel
    whose output tops out while the truth keeps growing is BLIND past that
    point, not merely noisy, and the two failure modes have different fixes.
  * the HORIZON -- the first true-range bin at which the median ratio leaves a
    tolerance band. Beyond it, any quantity computed FROM range is scaled wrong.

WHY THE ANSWER MATTERS MORE THAN IT LOOKS. Pro-nav commands
`a = N * Vc * lambda_dot`, and `Vc` is a range RATE. If range is scaled by k,
`Vc` is scaled by k and the whole lateral command is scaled by k with it -- so a
range channel that reads 0.3x truth under-commands pro-nav by roughly 3x at
exactly the moment the command matters. And the project's own headline lever
runs straight into this: ADR-0023/0027 measured the miss as largely committed at
handoff, with correction capacity going as t_go^2, so the prescribed fix is to
ACQUIRE FARTHER (12 m instead of 6.5 m was costed at 0.72 m -> ~4.3 m of
capacity). If the range channel degrades with range, then the lever the project
wants and the input pro-nav depends on pull in opposite directions. That is the
question this tool exists to answer, and it is answerable offline today.

`gt_*` is ground truth: this is a SCORING/FORENSIC tool and sits on no guidance
path (honesty boundary, CLAUDE.md).

SCOPE LIMIT, stated up front. The committed logs are the cue-era two-stage set
(phase `DASH`/`ENGAGE`, `meas_source` = track/drone, i.e. the markerless
detector), not the adopted coded-dash arms -- that archive is gitignored and
lives only on the dev machine. So this measures the MECHANISM on the data
available; the magnitudes do not transfer. Re-run with `--phase CODED_DASH` on
the dev machine to close it. Far-range bins are thin (tens of ticks), so the
per-bin n is PRINTED and bins under `--min-bin-n` are reported as UNDERPOWERED
rather than folded into the verdict.

USAGE
    python3 scripts/forensics/range_channel_horizon.py
    python3 scripts/forensics/range_channel_horizon.py --glob 'logs/mc_fp_*.csv' \
        --phase CODED_DASH
    python3 scripts/forensics/range_channel_horizon.py --self-test

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

REQUIRED = ("detected", "meas_range", "gt_range")

# A bin with fewer than this many ticks is reported but NOT allowed to set the
# horizon. The far bins on the committed fleet hold tens of ticks, and a median
# over a handful of samples is not a measurement.
MIN_BIN_N_DEFAULT = 20

# The median ratio may leave this band before the channel is called degraded.
# +-15% is wider than the 3% seen inside 8 m and far tighter than the 3-4x seen
# past 20 m, so the horizon does not depend on where in that gap it is set --
# which the sensitivity line in the output demonstrates rather than asserts.
TOL_DEFAULT = 0.15


def _f(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if v == v else None


def _med(xs):
    return statistics.median(xs) if xs else None


def collect(rows, bin_m, phase=None):
    """Per-tick (true_range, measured_range) on FRESH-detection ticks.

    Returns (pairs, dropped) where `dropped` counts every tick rejected and why,
    so a shrinking denominator is reported rather than absorbed.
    """
    pairs = []
    dropped = {"no_detection": 0, "bad_meas": 0, "bad_truth": 0, "wrong_phase": 0}
    for r in rows:
        if phase is not None and r.get("phase") != phase:
            dropped["wrong_phase"] += 1
            continue
        if _f(r.get("detected")) != 1.0:
            dropped["no_detection"] += 1
            continue
        m = _f(r.get("meas_range"))
        g = _f(r.get("gt_range"))
        if m is None or m <= 0.0:
            dropped["bad_meas"] += 1
            continue
        if g is None or g <= 0.0:
            dropped["bad_truth"] += 1
            continue
        pairs.append((g, m, r.get("phase")))
    return pairs, dropped


def horizon(pairs, bin_m=2.0, tol=TOL_DEFAULT, min_bin_n=MIN_BIN_N_DEFAULT):
    """Bin by TRUE range; find the first adequately-powered degraded bin.

    Returns a dict. `horizon_m` is None when no powered bin is degraded -- which
    is a real answer ("not degraded over the range this data covers"), NOT a
    pass by default, and the caller says which.
    """
    bins = {}
    for g, m, _ph in pairs:
        b = int(g // bin_m) * bin_m
        bins.setdefault(b, []).append(m / g)

    rows = []
    for b in sorted(bins):
        v = bins[b]
        rows.append({"lo": b, "hi": b + bin_m, "n": len(v), "median": _med(v),
                     "min": min(v), "max": max(v),
                     "powered": len(v) >= min_bin_n})

    h = None
    for row in rows:
        if row["powered"] and abs(row["median"] - 1.0) > tol:
            h = row["lo"]
            break
    return {"bins": rows, "horizon_m": h, "tol": tol, "min_bin_n": min_bin_n}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--glob", default="logs/*.csv")
    ap.add_argument("--phase", default=None,
                    help="restrict to one phase (e.g. CODED_DASH); default = all")
    ap.add_argument("--bin-m", type=float, default=2.0)
    ap.add_argument("--tol", type=float, default=TOL_DEFAULT,
                    help=f"ratio tolerance band (default {TOL_DEFAULT})")
    ap.add_argument("--min-bin-n", type=int, default=MIN_BIN_N_DEFAULT)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)

    if args.self_test:
        return 0 if self_test() else 1
    if args.bin_m <= 0:
        print("[range_channel_horizon] USAGE: --bin-m must be > 0", file=sys.stderr)
        return 2

    paths = sorted(globmod.glob(args.glob))
    if not paths:
        print(f"[range_channel_horizon] UNCERTAIN: no files matched {args.glob!r}")
        return 3

    pairs, skipped_schema, agg_dropped, files_used = [], 0, {}, 0
    for p in paths:
        try:
            with open(p, newline="") as fh:
                rd = csv.DictReader(fh)
                if not rd.fieldnames or any(c not in rd.fieldnames for c in REQUIRED):
                    skipped_schema += 1
                    continue
                rows = list(rd)
        except OSError as exc:
            print(f"[range_channel_horizon] ERROR: {p}: {exc}", file=sys.stderr)
            return 1
        got, dropped = collect(rows, args.bin_m, args.phase)
        for k, v in dropped.items():
            agg_dropped[k] = agg_dropped.get(k, 0) + v
        if got:
            files_used += 1
        pairs.extend(got)

    if not pairs:
        print(f"[range_channel_horizon] UNCERTAIN / VACUOUS: matched {len(paths)} "
              f"file(s), {skipped_schema} lacked {REQUIRED}, and 0 ticks survived "
              f"(dropped: {agg_dropped}). Nothing was measured.")
        return 3

    res = horizon(pairs, args.bin_m, args.tol, args.min_bin_n)
    meas = sorted(m for _g, m, _p in pairs)
    truth = sorted(g for g, _m, _p in pairs)
    phases = sorted({p for _g, _m, p in pairs if p})

    print(f"[range_channel_horizon] {len(pairs)} fresh-detection tick(s) from "
          f"{files_used} file(s) of {len(paths)} matched ({skipped_schema} on schema)")
    print(f"  dropped and counted: {agg_dropped}")
    print(f"  phases present: {', '.join(phases) or 'n/a'}")

    def pct(xs, q):
        return xs[min(len(xs) - 1, int(q * len(xs)))]

    print(f"  MEASURED range, the channel's own output: median "
          f"{_med(meas):.2f} m, 95th {pct(meas, .95):.2f}, 99th {pct(meas, .99):.2f}, "
          f"max {meas[-1]:.2f}")
    print(f"  TRUE range over the same ticks:            median "
          f"{_med(truth):.2f} m, 95th {pct(truth, .95):.2f}, 99th {pct(truth, .99):.2f}, "
          f"max {truth[-1]:.2f}")

    print(f"  ratio measured/true by TRUE-range bin (bin {args.bin_m:g} m, "
          f"powered = n >= {args.min_bin_n}):")
    for row in res["bins"]:
        flag = "" if row["powered"] else "   UNDERPOWERED"
        print(f"    {row['lo']:5.0f}-{row['hi']:<5.0f} n={row['n']:<5d} "
              f"median {row['median']:.3f}  (min {row['min']:.3f}, "
              f"max {row['max']:.3f}){flag}")

    # Sensitivity: does the horizon depend on where the band was drawn?
    alts = []
    for t in (0.10, 0.15, 0.20, 0.30):
        alts.append((t, horizon(pairs, args.bin_m, t, args.min_bin_n)["horizon_m"]))
    print("  horizon under other tolerance bands: "
          + ", ".join(f"+-{int(t*100)}% -> "
                      + ("none" if h is None else f"{h:.0f} m") for t, h in alts))

    # POOLED FAR TAIL. Each 2 m bin past ~18 m holds only a handful of ticks, so
    # no single one of them can carry a claim. Pooling them into ONE bin is the
    # honest way to power the far question -- and it is reported separately from
    # the binned table so nobody mistakes it for a bin.
    tail_lo = None
    for row in res["bins"]:
        if not row["powered"]:
            tail_lo = row["lo"]
            break
    if tail_lo is not None:
        tail = [m / g for g, m, _p in pairs if g >= tail_lo]
        powered = len(tail) >= args.min_bin_n
        print(f"  POOLED far tail, TRUE range >= {tail_lo:.0f} m: n={len(tail)}, "
              f"median ratio {_med(tail):.3f} (min {min(tail):.3f}, "
              f"max {max(tail):.3f}) -- "
              + ("adequately powered" if powered
                 else f"STILL UNDERPOWERED (< {args.min_bin_n})"))

    print("  VERDICT:")
    h = res["horizon_m"]
    if h is None:
        print(f"    NOT DEGRADED over the true-range span this data covers "
              f"(up to {truth[-1]:.1f} m), at +-{int(args.tol*100)}% on every "
              f"adequately-powered bin. That is a real answer, not a default "
              f"pass -- but note the powered bins only reach as far as the data "
              f"does, so it is silent about longer ranges.")
    else:
        far = [m / g for g, m, _p in pairs if g >= h]
        over = sum(1 for r in far if r > 1.0 + args.tol)
        under = sum(1 for r in far if r < 1.0 - args.tol)
        # PAIRED PER-TICK, not a ratio of two pooled medians. The pooled form
        # read 12.76 m against 13.12 m -- about 0.97, i.e. "fine" -- because it
        # averaged a 1.3x OVER-read near 8 m against a 0.25x UNDER-read past
        # 20 m. Two opposite errors cancelling into a reassuring number is the
        # exact shape this repo keeps retracting; the sign split is printed so
        # they cannot hide each other.
        print(f"    THE RANGE CHANNEL LEAVES THE +-{int(args.tol*100)}% BAND FROM "
              f"ABOUT {h:.0f} m. Over the {len(far)} tick(s) beyond it the PAIRED "
              f"per-tick ratio has median {_med(far):.3f} -- but it is NOT a "
              f"one-directional error: {over} tick(s) over-read, {under} "
              f"under-read. Read the binned table, not this median.")
        print(f"    CONSEQUENCE: pro-nav's a = N*Vc*lambda_dot scales with range, "
              f"so a command built on this channel is mis-scaled by roughly the "
              f"same factor past the horizon. Anything that reconstructs 3-D "
              f"position first inherits it; an IMAGE-SPACE law (IBVS, "
              f"arXiv:2409.17497) needs range only as a Jacobian scale factor, "
              f"which moves convergence RATE rather than the aim point. That is "
              f"the case for trying one -- it is NOT evidence that it works.")
        print(f"    AND THE TENSION WORTH NAMING: ADR-0023/0027 prescribe "
              f"acquiring FARTHER to buy t_go^2 correction capacity, which is "
              f"exactly where this channel is worst. The lever and the input "
              f"pull against each other.")
    print("    SCOPE: cue-era markerless fleet unless the phase line says "
          "CODED_DASH; gt_* is scoring-only; far bins are thin, so read the n "
          "column before quoting any single bin.")
    return 0


def _rows(pairs, phase="ENGAGE", detected=1):
    return [{"phase": phase, "detected": str(detected),
             "gt_range": f"{g:.4f}", "meas_range": f"{m:.4f}"} for g, m in pairs]


def self_test():
    ok = True

    def case(name, cond):
        nonlocal ok
        print(f"  [self-test] {name}: {'PASS' if cond else 'FAIL'}")
        ok = ok and cond

    # A PERFECT channel: ratio 1.0 everywhere -> no horizon.
    perfect = [(g, g) for g in [x * 0.5 for x in range(4, 80)]] * 3
    r = horizon(_perfect_pairs(perfect), 2.0, 0.15, 20)
    case("a perfect channel reports NO horizon", r["horizon_m"] is None)

    # A channel that SATURATES at 10 m: fine near, blind far.
    sat = []
    for g in [x * 0.5 for x in range(4, 80)]:
        sat += [(g, min(g, 10.0))] * 30
    r2 = horizon(_perfect_pairs(sat), 2.0, 0.15, 20)
    case("a channel saturating at 10 m is caught, and near 10 m",
         r2["horizon_m"] is not None and 10.0 <= r2["horizon_m"] <= 14.0)

    # UNDERPOWERED bins must not set the horizon. One wild bin of 3 ticks.
    thin = [(4.0, 4.0)] * 60 + [(30.0, 3.0)] * 3
    r3 = horizon(_perfect_pairs(thin), 2.0, 0.15, 20)
    case("a 3-tick bin cannot set the horizon", r3["horizon_m"] is None)
    case("and that bin is still REPORTED, not dropped",
         any(row["n"] == 3 and not row["powered"] for row in r3["bins"]))

    # Drop accounting: every rejected tick is counted under a named reason.
    rows = (_rows([(5.0, 5.0)]) + _rows([(5.0, 5.0)], detected=0)
            + [{"phase": "ENGAGE", "detected": "1", "gt_range": "5.0",
                "meas_range": ""}]
            + [{"phase": "ENGAGE", "detected": "1", "gt_range": "",
                "meas_range": "5.0"}])
    pairs, dropped = collect(rows, 2.0)
    case("one usable tick survives", len(pairs) == 1)
    case("the three rejects are counted by reason",
         dropped["no_detection"] == 1 and dropped["bad_meas"] == 1
         and dropped["bad_truth"] == 1)
    case("dropped ticks sum to what was offered",
         len(pairs) + sum(dropped.values()) == len(rows))

    # A phase filter must count what it excluded rather than hiding it.
    mixed = _rows([(5.0, 5.0)], phase="DASH") + _rows([(6.0, 6.0)], phase="ENGAGE")
    pairs2, dropped2 = collect(mixed, 2.0, phase="DASH")
    case("the phase filter keeps only its phase", len(pairs2) == 1)
    case("and counts the excluded tick", dropped2["wrong_phase"] == 1)

    # THE BINS ARE KEYED ON TRUE RANGE, NOT MEASURED. This looks pedantic and
    # is not: binning a SATURATING channel by its own output piles every far
    # tick into the saturation bin, which hides the horizon behind the very
    # defect being measured -- a scorer reading its own broken ruler. The
    # saturation fixture above cannot tell the two apart (both answers land near
    # 10 m), so this pins it directly with a tick whose true and measured bins
    # are far apart: true 30 m, measured 3 m.
    r4 = horizon(_perfect_pairs([(30.0, 3.0)] * 25), 2.0, 0.15, 20)
    keys = [row["lo"] for row in r4["bins"]]
    case("bins are keyed on TRUE range (30 m), not measured (3 m)",
         keys == [30.0])

    # A zero or negative measured range is a reject, never a ratio of 0.
    pairs3, dropped3 = collect(_rows([(5.0, 0.0), (5.0, -2.0)]), 2.0)
    case("non-positive measured range is rejected, not ratioed",
         pairs3 == [] and dropped3["bad_meas"] == 2)

    print(f"  [self-test] {'ALL PASS' if ok else 'FAILURES'}")
    return ok


def _perfect_pairs(pairs):
    return [(g, m, "ENGAGE") for g, m in pairs]


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(2)
    except BrokenPipeError:
        try:
            sys.stdout.close()
        finally:
            sys.exit(0)
