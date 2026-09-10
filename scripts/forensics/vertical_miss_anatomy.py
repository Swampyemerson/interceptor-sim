#!/usr/bin/env python3
"""Where does the VERTICAL miss come from -- the dash, the terminal, or the datum?

WHY THIS EXISTS (2026-09-09/10). ADR-0095 measured the adopted config's residual
closest approach as 0.374 m VERTICAL against 0.174 m horizontal: 82% of the
SQUARED miss is on an axis the terminal cannot steer, because the only vertical
command anywhere is an altitude-hold P-loop to a preset height and every seeker's
`bearing_vert_rad` is discarded (contradiction
`terminal-vertical-channel-decided-not-built`).

That raises the question this tool answers: WHERE along the flight is the vertical
error accumulated? It matters because scripts/forensics/handoff_closing_speed.py
measured only 0.02-0.17 m of terminal correction capacity on the AprilTag path --
far too little to null a 0.374 m bias. An error delivered by the DASH must be
corrected there (a pre-flight trim, the analogue of the crossing-bias aim
calibration, ADR-0080/0083). An error accumulated AFTER handoff cannot be.

WHAT IT REPORTS: an EXACT, ADDITIVE decomposition of the vertical separation at
closest approach, per flight, in one coordinate (`gt_cam_z - gt_tag_z`):

    vert_at_CPA  =  origin  +  dash_delta  +  post_dash_delta

  origin           vertical separation over the SETTLED pre-dash hover -- the
                   offset the vehicle already carries before the dash begins
                   (a DATUM/scenario term, not a guidance one)
  dash_delta       change in that separation from the settled hover to the end
                   of the dash -- what a `dash_alt_trim_m` can null
  post_dash_delta  change from the end of the dash to the CPA tick -- everything
                   the terminal (and the breakoff) does

Also reported: `alt_drift`, the same dash-window change measured on the vehicle's
OWN `alt_m` rather than on the ground-truth separation. That is the number a
human transcribes into `derive_dash_alt_trim_m`, so it is printed separately and
CROSS-CHECKED against `dash_delta`. An earlier docstring promised that
cross-check and no code implemented it, which is worse than not promising it --
and the two DO disagree (`alt_drift` -0.019 m against `dash_delta` +0.044 m).
The target holds a constant altitude on every tick of every flight
(`gt_tag_z == 0.500` throughout), so the target is not the cause; the camera
lever arm below is. The check is now real, and a disagreement is printed.

TWO CORRECTIONS THIS TOOL HAS ALREADY MADE TO ITSELF -- read them before quoting it.

1. THE HYPOTHESIS WAS A SAG; THE FLEET CLIMBS. The original guess: a P-only
   altitude loop cannot null a persistent disturbance, the airframe pitches
   nose-down 27-36 deg during the dash, vertical thrust falls to about
   cos(35 deg) = 0.82 of hover, so the vehicle should settle LOW. The committed
   fleet ends ABOVE the target on 16 of 16 flights. Coding the check as "did it
   go down" would have returned NOT SUPPORTED on data that supports the general
   mechanism, so the test below is the general one -- direction is an output, not
   an assumption.

2. THE BASELINE USED TO INCLUDE THE TAKEOFF, WHICH INVENTED THE DRIFT (found
   2026-09-10 by adversarial review, finding C1; this is the important one).
   The pre-dash baseline was "the median of every tick before the dash", which on
   this fleet pools 87-108 `TAKEOFF` ticks with 21-24 settled `CUE_WAIT` ticks.
   `alt_m` is AGL against the arm-point datum (ADR-0085), so it reads 0.000 ON
   THE GROUND: the `TAKEOFF` median is +0.005 m. The pooled baseline therefore
   measured *the takeoff climb*, and the reported "+0.320 m of dash altitude
   drift" was mostly the vehicle leaving the ground. The baseline is now the
   SETTLED pre-dash hover only (`CLIMB_PHASES` excluded), and a flight with no
   settled pre-dash hover FAILS CLOSED to `alt_drift = None` rather than
   returning a takeoff number. That case is not hypothetical: the coded-dash
   configuration goes TAKEOFF -> CODED_DASH with no hover at all, so on the
   dev-machine fleet this tool must report "no settled baseline" instead of
   handing `derive_dash_alt_trim_m` a ~+1 m fiction.

`gt_*` is ground truth: this is a SCORING/FORENSIC tool and is not on any
guidance path (honesty boundary, CLAUDE.md).

THE CAMERA LEVER ARM DOES ENTER THIS MEASUREMENT, and an earlier version of this
docstring said it did not ("the 0.120 m camera offset is HORIZONTAL and does not
enter a vertical measurement"). That is true only while the airframe is LEVEL.
`gt_cam_z` is the CAMERA's world z, and the camera sits on a forward boom, so the
moment the airframe pitches the boom lifts the camera relative to the airframe
datum. Measured as `gt_cam_z - alt_m`, which ought to be a fixed geometric
offset:

    settled hover -> dash tail   median +0.033 m
    settled hover -> CPA         median +0.087 m   (25% of `post_dash_delta`)

So roughly a quarter of the post-handoff term is the camera swinging on its boom,
not the airframe moving. This tool now measures and PRINTS that term
(`lever_arm_*`) instead of asserting it away. It cannot be REMOVED from these
logs, because they carry no attitude columns (deep-audit DEEP-R2) -- which is why
the fix is to log attitude, not to model it here.

SCOPE LIMIT. The committed logs are the cue-era two-stage set (phase `DASH`), not
the coded-dash arms ADR-0095 scored -- that archive is gitignored and lives only
on the dev machine. So this tests the MECHANISM on the data available; it does
not reproduce ADR-0095's -0.374 m, which was a different fleet. Run with
`--phase CODED_DASH` on the dev machine to close it -- and expect the
no-settled-baseline path to fire there.

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

# Phases that are NOT a settled hover and must never define the baseline. `alt_m`
# is AGL against the arm-point datum, so a TAKEOFF tick can be a GROUND tick;
# pooling them is what manufactured the original drift number (finding C1).
CLIMB_PHASES = frozenset({"TAKEOFF", "ARM", "ARMING", "LAND", "LANDING", "RTL",
                          "DISARM", "DISARMED", "IDLE", "PREFLIGHT"})

# Below this many settled pre-dash ticks the baseline is a transient, not a
# hover. MEASURED tick rate on this fleet is ~31 Hz (median dt 0.032 s in the
# pre-dash window; ENGAGE runs faster still, ~42 Hz), so 10 ticks is ~0.32 s --
# NOT the "0.5 s at 20 Hz" an earlier version of this comment asserted from the
# nominal decide rate instead of from the logs. A rate is a measured quantity
# here like any other.
MIN_BASELINE_TICKS = 10

# ADR-0095's number, for comparison only. NOT a threshold: it was measured on a
# different (gitignored) fleet, so this tool reports agreement, never asserts it.
ADR0095_VERTICAL_M = -0.374


def _f(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if v == v else None


def _med(xs):
    return statistics.median(xs) if xs else None


def anatomy(rows, dash_phase):
    """Per-flight vertical anatomy.

    Returns a dict, or None when the flight carries no usable CPA. Every reason a
    sub-measurement comes back None is NAMED in `drift_reason` and counted by the
    caller -- never silently absorbed, and never defaulted to 0.0 (a 0.0 there
    would read as "measured, no drift" when the truth is "not measured").
    """
    # CPA = minimum true range. Logged tick, not interpolated: this tool is about
    # the vertical BIAS, which is a metre-scale systematic, not the centimetre
    # interpolation term rescore_cpa.py handles.
    i_cpa, best_r = None, None
    for i, r in enumerate(rows):
        rng = _f(r.get("gt_range"))
        if rng is None:
            continue
        if best_r is None or rng < best_r:
            i_cpa, best_r = i, rng
    if i_cpa is None:
        return None

    at_cpa = rows[i_cpa]
    cam_z, tag_z = _f(at_cpa.get("gt_cam_z")), _f(at_cpa.get("gt_tag_z"))
    if cam_z is None or tag_z is None:
        return None
    vert_at_cpa = cam_z - tag_z          # negative = interceptor BELOW target

    out = {"cpa_m": best_r, "vert_at_cpa": vert_at_cpa, "i_cpa": i_cpa,
           "alt_drift": None, "dash_delta": None, "post_dash_delta": None,
           "origin": None, "n_base": 0, "n_dash": 0, "base_creep": None,
           "lever_arm_dash": None, "lever_arm_cpa": None,
           "drift_reason": None}

    dash_idx = [i for i, r in enumerate(rows) if r.get("phase") == dash_phase]
    if not dash_idx:
        out["drift_reason"] = f"no {dash_phase} ticks"
        return out
    i_dash0, i_dash1 = dash_idx[0], dash_idx[-1]
    out["n_dash"] = len(dash_idx)

    # THE SETTLED BASELINE: pre-dash ticks that are not a climb/ground phase.
    base_idx = [i for i in range(i_dash0)
                if rows[i].get("phase") not in CLIMB_PHASES]
    base_alt = [a for a in (_f(rows[i].get("alt_m")) for i in base_idx)
                if a is not None]
    out["n_base"] = len(base_alt)
    if len(base_alt) < MIN_BASELINE_TICKS:
        out["drift_reason"] = (
            f"no settled pre-dash hover: {len(base_alt)} non-climb pre-dash "
            f"tick(s) with an altitude, need {MIN_BASELINE_TICKS}"
            f" (pre-dash phases seen: "
            f"{','.join(sorted({rows[i].get('phase') or '' for i in range(i_dash0)})) or 'none'})")
        return out

    # Is the "settled" window actually settled? Report its own internal creep so a
    # reader can see whether the baseline is a hover or still a transient.
    half = max(1, len(base_alt) // 3)
    out["base_creep"] = _med(base_alt[-half:]) - _med(base_alt[:half])

    # DASH TAIL: the altitude the vehicle actually DELIVERS to the terminal. The
    # end matters, not the mean.
    tail_idx = dash_idx[-max(1, len(dash_idx) // 5):]
    tail_alt = [a for a in (_f(rows[i].get("alt_m")) for i in tail_idx)
                if a is not None]
    if not tail_alt:
        out["drift_reason"] = "no altitude on any dash-tail tick"
        return out

    # SIGNED: (end of dash) - (settled baseline). POSITIVE means the vehicle ends
    # the dash ABOVE where it started. This was once named `sag` and commented as
    # "how far the END sits BELOW the baseline" -- the opposite of the arithmetic.
    # A misleading label here is the highest-consequence defect available, because
    # ADR-0099 records that a wrong-signed trim DOUBLES the error.
    out["alt_drift"] = _med(tail_alt) - _med(base_alt)
    # Kept so main() can re-measure the drift under other baseline windows
    # (the sensitivity sweep). Underscored: internal, not part of the verdict.
    out["_base_alt"], out["_tail_alt"] = base_alt, tail_alt

    # The same three windows in the SEPARATION coordinate, which is what the miss
    # is measured in. This decomposition is exact by construction.
    def _vert(idxs):
        vs = []
        for i in idxs:
            cz, tz = _f(rows[i].get("gt_cam_z")), _f(rows[i].get("gt_tag_z"))
            if cz is not None and tz is not None:
                vs.append(cz - tz)
        return _med(vs)

    v_base, v_tail = _vert(base_idx), _vert(tail_idx)
    if v_base is not None and v_tail is not None:
        out["origin"] = v_base
        out["dash_delta"] = v_tail - v_base
        out["post_dash_delta"] = vert_at_cpa - v_tail
        # If the CPA is inside or before the dash there is no post-dash window;
        # say so rather than reporting a term that means something else.
        out["cpa_after_dash"] = i_cpa > i_dash1

    # THE CAMERA LEVER ARM. `gt_cam_z - alt_m` is a fixed geometric offset only
    # while the airframe is level; under pitch the forward boom lifts the camera.
    # Measuring its CHANGE separates "the vehicle moved" from "the camera swung".
    def _lever(idxs):
        vs = []
        for i in idxs:
            cz, a = _f(rows[i].get("gt_cam_z")), _f(rows[i].get("alt_m"))
            if cz is not None and a is not None:
                vs.append(cz - a)
        return _med(vs)

    l_base, l_tail = _lever(base_idx), _lever(tail_idx)
    l_cpa = (None if _f(at_cpa.get("alt_m")) is None
             else cam_z - _f(at_cpa.get("alt_m")))
    if l_base is not None:
        if l_tail is not None:
            out["lever_arm_dash"] = l_tail - l_base
        if l_cpa is not None:
            out["lever_arm_cpa"] = l_cpa - l_base
    return out


def _fmt(v, unit=" m"):
    return "n/a" if v is None else f"{v:+.3f}{unit}"


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

    rec, skipped_schema, skipped_nocpa, skipped_nodash = [], 0, 0, 0
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
        # THE MISSING GUARD (found 2026-09-10, review finding on this tool). Without
        # it, a file carrying NEITHER dash phase fell through to dash="DASH", found a
        # CPA from gt_range, produced drift=None, and was still COUNTED AS MEASURED.
        # Eight such files were pooled into the vertical statistic. The sibling tool
        # handoff_closing_speed.py always had this check, which is the whole reason
        # the two disagreed on the same directory -- and the disagreement was the tell.
        if dash not in phases:
            skipped_nodash += 1
            continue
        a = anatomy(rows, dash)
        if a is None:
            skipped_nocpa += 1
            continue
        a["path"], a["dash_phase"] = p, dash
        rec.append(a)

    if not rec:
        print(f"[vertical_miss_anatomy] UNCERTAIN / VACUOUS: matched {len(paths)} "
              f"file(s), measured 0 ({skipped_schema} lacked {REQUIRED}, "
              f"{skipped_nodash} had no dash phase, {skipped_nocpa} had no usable "
              f"CPA). Nothing was measured.")
        return 3

    verts = [r["vert_at_cpa"] for r in rec]
    drifts = [r["alt_drift"] for r in rec if r["alt_drift"] is not None]
    no_base = [r for r in rec if r["alt_drift"] is None]
    cpas = [r["cpa_m"] for r in rec]

    print(f"[vertical_miss_anatomy] measured {len(rec)} flight(s) of {len(paths)} "
          f"matched ({skipped_schema} on schema, {skipped_nodash} with no dash "
          f"phase, {skipped_nocpa} no CPA)")
    print(f"  dash phase(s): {', '.join(sorted({r['dash_phase'] for r in rec}))}")
    print(f"  CPA (logged tick)            median {statistics.median(cpas):+.3f} m")
    print(f"  vertical at CPA (cam - tag)  median {statistics.median(verts):+.3f} m "
          f"(min {min(verts):+.3f}, max {max(verts):+.3f})")
    n_below = sum(1 for v in verts if v < 0)
    print(f"  interceptor BELOW the target on {n_below}/{len(verts)} flights")

    # --- the decomposition (review finding C2): three terms, not one ---------
    dec = [r for r in rec if r["dash_delta"] is not None
           and r.get("cpa_after_dash")]
    print(f"  DECOMPOSITION of the vertical separation, n={len(dec)} of {len(rec)} "
          f"flight(s) with a settled baseline AND a post-dash CPA:")
    if dec:
        o = _med([r["origin"] for r in dec])
        dd = _med([r["dash_delta"] for r in dec])
        pd = _med([r["post_dash_delta"] for r in dec])
        print(f"    origin  (settled pre-dash separation)  {_fmt(o)}  [median]")
        print(f"      dash_delta      (hover -> dash end)  {_fmt(dd)}  [median]")
        print(f"      post_dash_delta (dash end -> CPA)    {_fmt(pd)}  [median]")
        # NO MISLEADING "=" HERE. The three medians above sum to something that is
        # NOT the median of the sums, and neither equals the headline (a third
        # population, n=16). Printing `a + b + c = d` across three different
        # populations invites a phone reader to read an identity that does not
        # hold. The PER-FLIGHT decomposition is exact -- that is what is asserted.
        print(f"    medians do NOT add: sum of the three above "
              f"{_fmt(o + dd + pd)}, median of the per-flight sums "
              f"{_fmt(_med([r['origin'] + r['dash_delta'] + r['post_dash_delta'] for r in dec]))}"
              f"; the per-flight identity holds exactly on each flight.")
        # PAIRED per-flight shares. A ratio of medians taken across different
        # populations is not the share of anything -- the same error the ADR-0099
        # addendum was written to fix, repeated here on the next number. It read
        # dash=2% where the paired figure is 5x that, understating the very term
        # the trim lever addresses.
        for label, key in (("origin", "origin"), ("dash", "dash_delta"),
                           ("post-dash", "post_dash_delta")):
            sh = []
            for r in dec:
                tot = (abs(r["origin"]) + abs(r["dash_delta"])
                       + abs(r["post_dash_delta"]))
                if tot > 1e-9:
                    sh.append(100.0 * abs(r[key]) / tot)
            if sh:
                print(f"    share of the excursion, {label:9} PAIRED per-flight "
                      f"median {_med(sh):.0f}%  (range {min(sh):.0f}-{max(sh):.0f}%, "
                      f"n={len(sh)})")
    else:
        print("    NOT MEASURABLE on this fleet -- no flight has both a settled "
              "pre-dash baseline and a CPA after the dash ends.")

    if drifts:
        print(f"  altitude DRIFT over the dash (own alt_m, settled baseline) "
              f"median {statistics.median(drifts):+.3f} m (n={len(drifts)})")
        creeps = [r["base_creep"] for r in rec if r["base_creep"] is not None]
        if creeps:
            print(f"    baseline window: median {statistics.median([r['n_base'] for r in rec if r['alt_drift'] is not None]):.0f} "
                  f"tick(s), internal creep median {statistics.median(creeps):+.3f} m "
                  f"(a creep comparable to the drift means the baseline is a "
                  f"transient, not a hover)")
    # --- THE CROSS-CHECK THE DOCSTRING PROMISES (finding S6) ------------------
    # `alt_drift` (own alt_m) and `dash_delta` (ground-truth separation) measure
    # the same window. They agree only if the target holds altitude AND the
    # camera-to-airframe offset is constant. Print the comparison; a disagreement
    # is a finding, not noise to hide.
    pairs = [(r["alt_drift"], r["dash_delta"]) for r in rec
             if r["alt_drift"] is not None and r["dash_delta"] is not None]
    if pairs:
        gap = _med([abs(a - d) for a, d in pairs])
        print(f"  CROSS-CHECK alt_drift vs dash_delta over the same window, "
              f"n={len(pairs)}:")
        print(f"    alt_drift  (own alt_m)            "
              f"{_fmt(_med([a for a, _ in pairs]))}")
        print(f"    dash_delta (gt separation)        "
              f"{_fmt(_med([d for _, d in pairs]))}")
        print(f"    median absolute per-flight gap    {_fmt(gap)}")
        lev_d = [r["lever_arm_dash"] for r in rec
                 if r["lever_arm_dash"] is not None]
        lev_c = [r["lever_arm_cpa"] for r in rec if r["lever_arm_cpa"] is not None]
        if lev_d or lev_c:
            print(f"    CAMERA LEVER ARM (gt_cam_z - alt_m), which should be a "
                  f"CONSTANT offset:")
            if lev_d:
                print(f"      change hover -> dash tail       "
                      f"{_fmt(_med(lev_d))} (n={len(lev_d)})")
            if lev_c:
                lc = _med(lev_c)
                extra = ""
                if dec:
                    pdm = _med([r["post_dash_delta"] for r in dec])
                    if pdm and abs(pdm) > 1e-9:
                        extra = f" = {100 * abs(lc) / abs(pdm):.0f}% of post_dash_delta"
                print(f"      change hover -> CPA             "
                      f"{_fmt(lc)} (n={len(lev_c)}){extra}")
        if gap is not None and gap > 0.02:
            print(f"    DISAGREEMENT: the two rulers differ by {gap:+.3f} m. The "
                  f"target holds a constant altitude in these logs, so this is "
                  f"the camera lever arm under pitch, not the target. Treat the "
                  f"ground-truth separation as the miss and `alt_drift` as what "
                  f"the vehicle's own loop did -- they are NOT interchangeable, "
                  f"and only `alt_drift` may be fed to derive_dash_alt_trim_m.")

    if no_base:
        print(f"  NOT MEASURED for drift on {len(no_base)}/{len(rec)} flight(s) "
              f"-- FAILED CLOSED, not defaulted to 0.0:")
        for r in no_base[:3]:
            print(f"    {r['path']}: {r['drift_reason']}")
        if len(no_base) > 3:
            print(f"    ... and {len(no_base) - 3} more")
    print(f"  ADR-0095 reference (other fleet, corrected ruler): "
          f"{ADR0095_VERTICAL_M:+.3f} m vertical")

    # --- BASELINE SENSITIVITY (answers the creep warning above, finding (a)) --
    # The tool warns that a baseline creep comparable to the drift means the
    # window is a transient. Leaving that warning unanswered is worse than not
    # printing it, so the answer is computed: re-measure the drift under several
    # defensible baseline choices. A number that moves across them is not a
    # measurement; a number that holds is.
    if drifts:
        print("  BASELINE SENSITIVITY -- the same drift under other defensible "
              "baseline windows:")
        for label, pick in (
            ("whole settled window", lambda xs: xs),
            ("last 10 ticks", lambda xs: xs[-10:]),
            ("last 20 ticks", lambda xs: xs[-20:]),
            ("first 10 ticks", lambda xs: xs[:10]),
            ("drop the first 3", lambda xs: xs[3:] or xs),
            ("last tick only", lambda xs: xs[-1:]),
        ):
            alt = []
            for r in rec:
                if r.get("_base_alt") and r.get("_tail_alt"):
                    b = pick(r["_base_alt"])
                    if b:
                        alt.append(statistics.median(r["_tail_alt"])
                                   - statistics.median(b))
            if alt:
                print(f"    {label:22} {_med(alt):+.3f} m (n={len(alt)})")
        print("    If these disagree in SIGN or straddle the 0.05 m decision "
              "threshold, the drift is not measured -- say so instead of "
              "quoting one of them.")

    med_v = statistics.median(verts)
    med_s = statistics.median(drifts) if drifts else None

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
        print("    DRIFT MECHANISM: UNTESTED here -- no flight carries a settled "
              "pre-dash hover, so the hypothesis is neither supported nor "
              "refuted. Do NOT derive a trim from this run.")
    elif abs(med_s) < 0.05:
        print(f"    DRIFT MECHANISM NOT SUPPORTED: measured against a SETTLED "
              f"baseline the altitude holds to {med_s:+.3f} m across the dash, so "
              f"it cannot account for a {med_v:+.3f} m vertical miss. Look at the "
              f"decomposition above for where the error actually comes from, and "
              f"at the altitude DATUM (ADR-0085) / the target's own commanded "
              f"altitude for the `origin` term.")
    elif (med_s > 0) == (med_v > 0) and abs(med_s) >= 0.4 * abs(med_v):
        # PAIRED, per flight, over the flights that have BOTH numbers. A ratio of
        # two medians taken over two different populations is not the share of
        # anything -- and it read 66% where the paired figure differed.
        paired = [(r["alt_drift"], r["vert_at_cpa"]) for r in rec
                  if r["alt_drift"] is not None and r["vert_at_cpa"] != 0.0]
        dropped = len([r for r in rec if r["alt_drift"] is not None]) - len(paired)
        share = (100.0 * statistics.median([abs(d) / abs(v) for d, v in paired])
                 if paired else None)
        share_txt = "n/a (0 pairable flights)" if share is None else f"{share:.0f}%"
        print(f"    DRIFT MECHANISM SUPPORTED IN PART: altitude moves {med_s:+.3f} m "
              f"across the dash and the miss is {med_v:+.3f} m -- same sign, and "
              f"the drift is {share_txt} of the miss (PAIRED per-flight median over "
              f"{len(paired)} flight(s); {dropped} dropped for a zero miss). "
              f"A dash-time trim can only claim the `dash_delta` row above -- read "
              f"the decomposition before asserting the dash DELIVERS the error.")
    else:
        print(f"    DRIFT MECHANISM PARTIAL / OPPOSED: drift {med_s:+.3f} m vs "
              f"miss {med_v:+.3f} m. Signs or magnitudes disagree, so dash "
              f"altitude drift is not the whole story here.")
    print("    SCOPE: cue-era DASH unless the phase line says CODED_DASH. "
          "gt_* is scoring-only. Attitude is NOT logged in these CSVs, so the "
          "pitch-to-drift link is inferred, not measured (deep-audit DEEP-R2).")
    return 0


def _rows(phase, n, alt, tag=5.0, rng0=30.0, drng=0.0, cam=None):
    out = []
    for i in range(n):
        a = alt(i) if callable(alt) else alt
        c = a if cam is None else (cam(i) if callable(cam) else cam)
        out.append({"phase": phase, "gt_range": f"{rng0 + i * drng:.3f}",
                    "gt_cam_z": f"{c:.3f}", "gt_tag_z": f"{tag:.3f}",
                    "alt_m": f"{a:.3f}"})
    return out


def self_test():
    ok = True

    def case(name, cond):
        nonlocal ok
        print(f"  [self-test] {name}: {'PASS' if cond else 'FAIL'}")
        ok = ok and cond

    # A flight that hovers at 5 m, sags to 4.6 m through the dash, and passes
    # 0.40 m BELOW a target held at 5 m.
    rows = _rows("HOLD", 20, 5.0, rng0=30.0, drng=-0.01)
    dash = _rows("DASH", 50, lambda i: 5.0 - 0.4 * (i / 49.0), rng0=25.0, drng=-0.45)
    rows += dash
    expect_cpa = min(float(r["gt_range"]) for r in rows)
    a = anatomy(rows, "DASH")
    # Expectation COMPUTED from the fixture, not hand-typed -- a hand-typed 2.75
    # here was simply wrong (the real minimum is 2.95) and would have shipped as
    # a failing self-test on a working tool.
    case("CPA found at the minimum range",
         a and abs(a["cpa_m"] - expect_cpa) < 1e-9)
    case("vertical at CPA recovers -0.40 m", a and abs(a["vert_at_cpa"] + 0.40) < 0.02)
    case("drift recovers about -0.40 m", a and a["alt_drift"] is not None
         and abs(a["alt_drift"] + 0.40) < 0.05)

    # No pre-dash ticks: drift must be None, NOT 0.0. Reporting 0.0 would read as
    # "measured, no drift" when the truth is "not measured" -- the fail-closed rule.
    a2 = anatomy([r for r in rows if r["phase"] == "DASH"], "DASH")
    case("no baseline -> drift is None, not 0.0", a2 and a2["alt_drift"] is None)

    # A flight with no usable ground truth must return None, not a zero verdict.
    a3 = anatomy([{"phase": "DASH", "gt_range": "", "gt_cam_z": "",
                   "gt_tag_z": "", "alt_m": ""}], "DASH")
    case("no CPA -> None (caller counts it)", a3 is None)

    # A level flyby must NOT report a vertical bias.
    a4 = anatomy(_rows("DASH", 9, 5.0, rng0=10.0, drng=-1.0), "DASH")
    case("level flyby reports zero vertical", a4 and abs(a4["vert_at_cpa"]) < 1e-9)

    # A CLIMB must be reported as the same mechanism as a sag. Coding the check
    # as "did it go down" returned NOT SUPPORTED on the real fleet, which climbs.
    a5 = anatomy(_rows("HOLD", 20, 5.0)
                 + _rows("DASH", 50, lambda i: 5.0 + 0.4 * (i / 49.0),
                         rng0=25.0, drng=-0.45), "DASH")
    case("a CLIMB is measured with the same machinery",
         a5 and a5["alt_drift"] is not None and abs(a5["alt_drift"] - 0.40) < 0.05
         and a5["vert_at_cpa"] > 0)

    # REGRESSION for the missing dash-phase guard: a file with NEITHER dash phase
    # used to be counted as a measured flight (8 of them were), because the caller
    # never checked that the chosen phase is present. anatomy() itself is happy to
    # produce a CPA with an empty dash window, so the guard has to live in main() --
    # this pins the property anatomy() must expose for that guard to work.
    a6 = anatomy(_rows("TAKEOFF", 9, 5.0, rng0=20.0, drng=-1.0), "DASH")
    case("a flight with no dash phase yields drift=None, never 0.0",
         a6 is not None and a6["alt_drift"] is None)

    # ---- REGRESSION for finding C1: the TAKEOFF climb must not be the baseline.
    # `alt_m` is AGL against the arm point, so takeoff ticks start at 0.0. A
    # vehicle that climbs to 5.0, hovers there, then holds 5.0 flat through the
    # dash has ZERO drift. Pooling the takeoff makes it read as about +4 m.
    climb_then_flat = (_rows("TAKEOFF", 90, lambda i: 5.0 * min(1.0, i / 80.0))
                       + _rows("HOLD", 24, 5.0)
                       + _rows("DASH", 50, 5.0, rng0=25.0, drng=-0.45))
    a7 = anatomy(climb_then_flat, "DASH")
    case("C1: takeoff climb is EXCLUDED from the baseline (drift ~0, not ~+4)",
         a7 and a7["alt_drift"] is not None and abs(a7["alt_drift"]) < 0.02)
    case("C1: baseline counts only the settled ticks",
         a7 and a7["n_base"] == 24)

    # ---- REGRESSION for finding C1's fail-closed half: TAKEOFF -> DASH with no
    # hover at all is the CODED-DASH case. It must be None, not a takeoff number.
    no_hover = (_rows("TAKEOFF", 90, lambda i: 5.0 * min(1.0, i / 80.0))
                + _rows("DASH", 50, 5.0, rng0=25.0, drng=-0.45))
    a8 = anatomy(no_hover, "DASH")
    case("C1: no settled hover -> drift is None (fail closed)",
         a8 and a8["alt_drift"] is None)
    case("C1: and the reason names the missing hover",
         a8 and a8["drift_reason"] and "settled" in a8["drift_reason"])

    # A baseline shorter than MIN_BASELINE_TICKS is a transient, not a hover.
    short = (_rows("TAKEOFF", 30, lambda i: 5.0 * min(1.0, i / 25.0))
             + _rows("HOLD", MIN_BASELINE_TICKS - 1, 5.0)
             + _rows("DASH", 50, 5.0, rng0=25.0, drng=-0.45))
    case("C1: a sub-threshold baseline fails closed too",
         anatomy(short, "DASH")["alt_drift"] is None)

    # ---- finding C2: the decomposition must be exact and must expose a
    # post-dash term. Separation: origin +0.0, dash +0.2, post-dash +0.5.
    dec = (_rows("HOLD", 20, 5.0)
           + _rows("DASH", 50, lambda i: 5.0 + 0.2 * (i / 49.0),
                   rng0=25.0, drng=-0.40)
           + _rows("ENGAGE", 20, lambda i: 5.2 + 0.5 * (i / 19.0),
                   rng0=5.0, drng=-0.25))
    a9 = anatomy(dec, "DASH")
    case("C2: origin is the settled separation", a9 and abs(a9["origin"]) < 1e-9)
    case("C2: dash_delta recovers +0.20 m",
         a9 and abs(a9["dash_delta"] - 0.20) < 0.03)
    case("C2: post_dash_delta recovers +0.50 m and is NOT folded into the dash",
         a9 and abs(a9["post_dash_delta"] - 0.50) < 0.05)
    case("C2: the decomposition sums to the measured vertical at CPA",
         a9 and abs((a9["origin"] + a9["dash_delta"] + a9["post_dash_delta"])
                    - a9["vert_at_cpa"]) < 1e-9)
    case("C2: the CPA is flagged as after the dash", a9 and a9["cpa_after_dash"])

    # A CPA that falls INSIDE the dash has no post-dash window; say so.
    inside = (_rows("HOLD", 20, 5.0)
              + _rows("DASH", 40, 5.0, rng0=20.0, drng=-0.5)
              + _rows("DASH", 10, 5.0, rng0=1.0, drng=+0.5))
    a10 = anatomy(inside, "DASH")
    case("C2: a CPA inside the dash is flagged, not silently decomposed",
         a10 and a10.get("cpa_after_dash") is False)

    print(f"  [self-test] {'ALL PASS' if ok else 'FAILURES'}")
    return ok


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(2)
    except BrokenPipeError:
        # `| head` closes the pipe; a traceback there is noise, not a finding.
        try:
            sys.stdout.close()
        finally:
            sys.exit(0)
