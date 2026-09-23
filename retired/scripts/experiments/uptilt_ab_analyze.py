#!/usr/bin/env python3
"""Task #35 / ADR-0060 up-tilt camera-mount A/B analyzer (OFFLINE ONLY --
reads arm summary CSVs + flight CSVs + PX4 ulogs; never touches the sim).

Extends scripts/experiments/dash_pitch_probe.py (imported, not modified) with
a MOUNT ANGLE: the up-tilt lives in the SENSOR pose only (see
scripts/uptilt_make_variants.py), so the vehicle attitude and the gt
camera-link pose are untouched and the mount is applied analytically:

    vert_in_camera = vert_in_body + mount_up_deg
    (dash_pitch_probe convention: positive = below boresight, so an up-tilt
    shifts an above-FoV target, vert < -41.6 deg, back toward frame center)

Per arm it reports, seed-paired across arms (same master-seed 42 plan):
  (a) dash-detection recovery -- frac of dash ticks target-in-FoV / above /
      below (WITH the mount), first_det_range_m + coverage from the arm CSV;
  (b) handoff vertical margin -- degrees between the target and the TOP FoV
      edge at first detection and at handoff (ADR-0060 baseline: 0-6 deg);
  (c) ENGAGE-phase pointing tradeoff -- frac of ENGAGE ticks the target sits
      BELOW the bottom FoV edge (the cost of tilting up), plus miss/clean.

CAVEAT on (c), PER ARM (not blanket): a tilted arm flown WITHOUT
--cam-mount-up-deg (the ADR-0067 sweep) has an uncompensated elevation bias =
mount angle, so its miss/clean measure the UNCOMPENSATED worst case, not the
design ceiling. A `_up15c`-style COMPENSATED re-fly arm (#40) flies WITH the
mount composed into guidance -- there the (c)-metrics ARE the design condition.
The footer prints the right caveat per arm from the filename `c` suffix.

MECHANISM CHECK (shadow-swap live validation), CORRECTED: the mount is added
ANALYTICALLY (vert_cam = vert_body + mount) and vert_body is independent of the
sensor pose, so a FAILED shadow swap flies a LEVEL camera and reads
d(top_margin) ~ +mount EXACTLY with d(first_det_range) ~ 0 -- that pair is the
INVALID (swap-did-not-take) signature. A WORKING swap detects the target
higher/earlier, so d(top_margin) sits BELOW +mount (flown ADR-0067:
+9.6/+18.6/+27.5 vs 15/25/35) with a POSITIVE d(first_det_range). The paired
section flags the invalid signature; treat such an arm as INVALID and stop.

Usage:
  .venv/bin/python scripts/experiments/uptilt_ab_analyze.py \
      logs/mc_uptilt_weave12_up00.csv logs/mc_uptilt_weave12_up15.csv \
      logs/mc_uptilt_weave12_up25.csv logs/mc_uptilt_weave12_up35.csv \
      [--baseline logs/mc_t21_trackgate_weave12_r2.csv]
Mount angle is parsed from `_upNN` in the filename; --baseline is treated as
mount 0 (the r2 zero-mount n=16 reference; its first 8 seeds pair with n=8
arms -- prefix-stable plan RNG, verified in mc_batch.sh plan-gen).
"""
import csv
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dash_pitch_probe as dpp  # noqa: E402  (helpers reused, file unmodified)

HALF_V = dpp.HALF_VFOV_DEG
HALF_H = dpp.HALF_HFOV_DEG


def analyze_run_mount(flight_csv, mount_up_deg):
    """dash_pitch_probe.analyze_run, extended with a mount angle + ENGAGE
    stats. Returns (dict, None) or (None, 'skip reason')."""
    rows = list(csv.DictReader(open(flight_csv)))
    ticks = [r for r in rows if r.get("t_sim")]
    if not ticks:
        return None, "no t_sim (not a --handoff run)"
    ulog_path = dpp.find_ulog(dpp.csv_start_utc(flight_csv))
    if not ulog_path:
        return None, "no matching ulog"
    from pyulog import ULog
    u = ULog(ulog_path, ["vehicle_attitude", "vehicle_local_position"])
    d = {x.name: x for x in u.data_list}
    va, vlp = d.get("vehicle_attitude"), d.get("vehicle_local_position")
    if va is None or vlp is None:
        return None, "ulog missing attitude/local_position"
    ta = va.data["timestamp"] / 1e6
    q = np.stack([va.data["q[0]"], va.data["q[1]"],
                  va.data["q[2]"], va.data["q[3]"]], axis=1)
    tp = vlp.data["timestamp"] / 1e6
    alt_u = -vlp.data["z"]

    t_csv = np.array([float(r["t_sim"]) for r in ticks if r.get("alt_m")])
    a_csv = np.array([float(r["alt_m"]) for r in ticks if r.get("alt_m")])
    offs = np.arange(-15.0, 15.0, 0.05)
    errs = [np.mean(np.abs(np.interp(t_csv + o, tp, alt_u) - a_csv))
            for o in offs]
    off = float(offs[int(np.argmin(errs))])

    def vert_cam(r):
        """target vertical angle IN THE (tilted) CAMERA frame at row r, deg;
        positive = below boresight. None if gt missing."""
        try:
            dx = float(r["gt_tag_x"]) - float(r["gt_cam_x"])   # ENU east
            dy = float(r["gt_tag_y"]) - float(r["gt_cam_y"])   # ENU north
            dz = float(r["gt_tag_z"]) - float(r["gt_cam_z"])   # ENU up
        except (ValueError, KeyError):
            return None, None
        t = float(r["t_sim"])
        qi = q[np.searchsorted(ta, t + off).clip(0, len(ta) - 1)]
        v_b = dpp.ned_to_body(qi, np.array([dy, dx, -dz]))  # ENU->NED->FRD
        vert_body = math.degrees(math.atan2(v_b[2], v_b[0]))
        horiz = math.degrees(math.atan2(v_b[1], v_b[0]))
        return vert_body + mount_up_deg, horiz

    dash = [r for r in ticks if r["phase"] == "DASH"]
    engage = [r for r in ticks if r["phase"] == "ENGAGE"]
    if not dash:
        return None, "no DASH phase"
    first_det = next((r for r in ticks if r.get("detected") == "1"
                      and r["phase"] in ("DASH", "HANDOFF", "ENGAGE")), None)
    seen = False
    handoff_row = None
    for r in ticks:
        if r["phase"] == "DASH":
            seen = True
        elif seen:
            handoff_row = r
            break

    def phase_stats(phase_rows):
        vs, hs = [], []
        for r in phase_rows:
            v, h = vert_cam(r)
            if v is not None:
                vs.append(v)
                hs.append(h)
        if not vs:
            return None
        vs, hs = np.array(vs), np.array(hs)
        return {
            "n": len(vs),
            "vert_med": float(np.median(vs)),
            "in_fov": float(np.mean((np.abs(vs) <= HALF_V)
                                    & (np.abs(hs) <= HALF_H))),
            "above": float(np.mean(vs < -HALF_V)),   # off the TOP edge
            "below": float(np.mean(vs > HALF_V)),    # off the BOTTOM edge
        }

    out = {"flight_csv": flight_csv, "ulog": ulog_path,
           "mount_up_deg": mount_up_deg,
           "dash": phase_stats(dash), "engage": phase_stats(engage)}
    for name, row in (("first_det", first_det), ("handoff", handoff_row)):
        if row is None:
            out[name] = None
            continue
        v, h = vert_cam(row)
        out[name] = None if v is None else {
            "vert_cam_deg": v,
            # degrees of margin from the TOP FoV edge (ADR-0060's 0-6 sliver)
            "top_margin_deg": v + HALF_V,
            "range": row.get("gt_range") or row.get("meas_range"),
        }
    return out, None


def mount_from_name(path):
    m = re.search(r"_up(\d+)", os.path.basename(path))
    return int(m.group(1)) if m else 0


def is_compensated(path):
    """True if this arm's flights ran WITH --cam-mount-up-deg (the #40 re-fly).
    Convention (scripts/uptilt_ab_arm.sh --compensate): a non-zero compensated
    arm's CSV carries a 'c' suffix on the mount token, e.g. `_up15c.csv`; the
    refly control is `_up00.csv` (flown with --cam-mount-up-deg 0, byte-
    identical, so 'uncompensated' is the honest label for it)."""
    return bool(re.search(r"_up\d+c\b", os.path.basename(path)))


def parse_float_or_none(s):
    """Parse a CSV cell to float, treating '', None, and 'nan' as MISSING (None)
    -- NOT as a silently-dropped-by-truthiness or NaN-that-counts-as-a-tie
    value. Used so a lost/crashed flight becomes an explicit dropped pair, not
    an attrition that asymmetrically favors PASS (review finding, #40)."""
    if s is None:
        return None
    s = str(s).strip()
    if not s or s.lower() == "nan":
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    return None if math.isnan(v) else v


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(2)
    arms = []           # (label, mount_deg, summary_csv)
    i = 0
    while i < len(args):
        if args[i] == "--baseline":
            arms.append((os.path.basename(args[i + 1]) + " [baseline, mount 0]",
                         0, args[i + 1]))
            i += 2
        else:
            arms.append((os.path.basename(args[i]), mount_from_name(args[i]),
                         args[i]))
            i += 1

    per_arm = {}        # label -> {run_idx: merged row dict}
    arm_path = {}       # label -> source summary CSV path (compensation detect)
    arm_skipped = {}    # label -> count of runs dropped (missing csv / unanalyzable)
    for label, mount, s in arms:
        runs = {}
        arm_path[label] = s
        skipped = 0
        for row in csv.DictReader(open(s)):
            fp = row["flight_csv_path"]
            if not fp or not os.path.exists(fp):
                print(f"[skip] {label} run {row['run_idx']}: flight csv missing")
                skipped += 1
                continue
            res, err = analyze_run_mount(fp, mount)
            if err:
                print(f"[skip] {label} run {row['run_idx']}: {err}")
                skipped += 1
                continue
            res["summary"] = row
            runs[row["run_idx"]] = res
        per_arm[label] = (mount, runs)
        arm_skipped[label] = skipped

    print("\n===== PER-ARM AGGREGATES =====")
    hdr = (f"{'arm':<42} {'mount':>5} {'n':>3} {'dashInFoV':>9} "
           f"{'dashAbove':>9} {'1stDetR_m':>9} {'topMargin':>9} "
           f"{'hoTopMarg':>9} {'engBelow':>8} {'missMed':>7} {'Pk@2.5':>6} {'clean':>5}")
    print(hdr)
    agg = {}
    for label, (mount, runs) in per_arm.items():
        if not runs:
            print(f"{label:<42} {mount:>5} {0:>3}  -- no analyzable runs --")
            continue
        rs = list(runs.values())

        def med(vals):
            vals = [v for v in vals if v is not None]
            return float(np.median(vals)) if vals else float("nan")

        d_in = med([r["dash"]["in_fov"] for r in rs if r["dash"]])
        d_ab = med([r["dash"]["above"] for r in rs if r["dash"]])
        fdr = med([float(r["summary"]["first_det_range_m"] or "nan")
                   for r in rs if r["summary"].get("first_det_range_m")])
        tm = med([r["first_det"]["top_margin_deg"] for r in rs
                  if r["first_det"]])
        hm = med([r["handoff"]["top_margin_deg"] for r in rs if r["handoff"]])
        eb = med([r["engage"]["below"] for r in rs if r["engage"]])
        misses = [float(r["summary"].get("miss_m") or "nan") for r in rs]
        miss = med(misses)
        pk25 = sum(1 for m_ in misses if m_ < 2.5)   # ratified proximity metric (ADR-0025)
        clean = sum(int(r["summary"].get("clean") or 0) for r in rs)
        agg[label] = dict(mount=mount, first_det_med=fdr, top_margin_med=tm)
        print(f"{label:<42} {mount:>5} {len(rs):>3} {d_in:>9.0%} {d_ab:>9.0%} "
              f"{fdr:>9.2f} {tm:>9.1f} {hm:>9.1f} {eb:>8.0%} "
              f"{miss:>7.2f} {pk25:>3}/{len(rs)} {clean:>3}/{len(rs)}")

    # ---- paired per-seed deltas vs the zero-mount CONTROL -----------------
    # Control selection (review finding, #40): the PRIMARY pre-registered
    # statistic is up15c MINUS the WITHIN-SWEEP refly up00; picking the FIRST
    # mount-0 arm in argv order silently re-based it to a legacy cross-era up00
    # when both were passed (the --compensate analyze hint feeds both). Cross-
    # era acquisition comparisons are formally INVALID (ADR-0067 addendum), so:
    # prefer a `refly`/compensated-sweep mount-0 arm as control, and if MORE
    # THAN ONE mount-0 arm is present, WARN LOUDLY naming the chosen control.
    zero_arms = [lbl for lbl, (m, r) in per_arm.items() if m == 0 and r]
    zero = None
    if zero_arms:
        refly_zeros = [lbl for lbl in zero_arms if "refly" in lbl]
        zero = (refly_zeros or zero_arms)[0]
        if len(zero_arms) > 1:
            print(f"\n[WARN] {len(zero_arms)} mount-0 arms passed "
                  f"({', '.join(zero_arms)}). CONTROL = {zero} "
                  f"({'refly/within-sweep preferred' if refly_zeros else 'first in argv -- NO refly arm found'}). "
                  "Cross-era pairing is INVALID (ADR-0067 addendum): confirm "
                  "the control is the within-sweep up00 before trusting the "
                  "PRIMARY parity read.")
    if zero:
        z_runs = per_arm[zero][1]
        print(f"\n===== PAIRED PER-SEED DELTAS vs {zero} (same run_idx = same "
              "cue/path seeds) =====")

        def med_or_nan(vals):
            return float(np.median(vals)) if vals else float("nan")

        def sign_line(name, unit, deltas, neg_word, pos_word, n_expect,
                      dropped, fmt="+.2f"):
            """Per-seed deltas + SIGN COUNTS + med + med|delta| (ADR-0067
            addendum / the #40 re-fly pre-registration: medians alone are not
            acceptance evidence; at n=8 paired, 7/8 one-direction ~ p 0.035).
            Prints the SIGNED median AND the median-ABSOLUTE delta -- the
            gate-0 machinery-stability tripwire binds median|delta| <= 1.0 m,
            which a signed median masks under symmetric drift. Denominators are
            NOT pinned inside the tool: the pre-registration requires re-flying
            any lost seed so n_paired == n_expect; a mismatch is surfaced
            LOUDLY here rather than silently shrinking the sign denominator."""
            if not deltas:
                return f"    d({name}): no paired values (dropped {dropped})"
            neg = sum(1 for d in deltas if d < 0)
            pos = sum(1 for d in deltas if d > 0)
            tie = len(deltas) - neg - pos
            vals = " ".join(format(d, fmt) for d in deltas)
            mad = float(np.median([abs(d) for d in deltas]))
            warn = ("" if len(deltas) == n_expect and not dropped
                    else f"  [!] denom {len(deltas)} != expected {n_expect} "
                         f"(dropped {dropped}) -- re-fly the lost seed(s) "
                         "before scoring (bars are counts of exactly N)")
            return (f"    d({name}) [{unit}] per-seed: {vals}\n"
                    f"      -> {neg_word} {neg}/{len(deltas)}, {pos_word} "
                    f"{pos}/{len(deltas)}"
                    + (f", tie {tie}" if tie else "")
                    + f"; med={med_or_nan(deltas):+.3f}, med|d|={mad:.3f}{warn}")

        for label, (mount, runs) in per_arm.items():
            if label == zero or not runs:
                continue
            common = sorted(set(runs) & set(z_runs), key=int)
            if not common:
                print(f"{label}: no common run_idx with control")
                continue
            dfd, dtm, dmiss, deb = [], [], [], []
            drop_fd = drop_miss = 0
            for k in common:
                a, b = runs[k], z_runs[k]
                fa = parse_float_or_none(a["summary"].get("first_det_range_m"))
                fb = parse_float_or_none(b["summary"].get("first_det_range_m"))
                if fa is not None and fb is not None:
                    dfd.append(fa - fb)
                else:
                    drop_fd += 1
                if a["first_det"] and b["first_det"]:
                    dtm.append(a["first_det"]["top_margin_deg"]
                               - b["first_det"]["top_margin_deg"])
                ma_ = parse_float_or_none(a["summary"].get("miss_m"))
                mb_ = parse_float_or_none(b["summary"].get("miss_m"))
                if ma_ is not None and mb_ is not None:
                    dmiss.append(ma_ - mb_)
                else:
                    drop_miss += 1
                if a["engage"] and b["engage"]:
                    deb.append(a["engage"]["below"] - b["engage"]["below"])
            # mechanism check (CORRECTED, review finding, #40): the analyzer
            # adds the mount ANALYTICALLY (vert_cam = vert_body + mount), and
            # vert_body is independent of the sensor pose, so d(top_margin) =
            # d(vert_body at first-det) + mount. A FAILED shadow swap flies a
            # LEVEL camera -> first-det geometry == control -> d(top_margin)
            # ~ +mount EXACTLY and d(first_det_range) ~ 0. A WORKING swap
            # detects the target higher/earlier -> d(top_margin) sits BELOW
            # +mount (flown ADR-0067: +9.6/+18.6/+27.5 for 15/25/35) and
            # d(first_det_range) is positive (+2-3 m). So "d ~ +mount AND
            # d(first_det) ~ 0" is the INVALID (swap-did-not-take) signature,
            # NOT the healthy one.
            dtm_med = med_or_nan(dtm)
            dfd_med = med_or_nan(dfd)
            swap_flag = ""
            if mount > 0 and dtm:
                near_mount = abs(dtm_med - mount) < 2.0
                fd_flat = (not dfd) or abs(dfd_med) < 1.0
                if near_mount and fd_flat:
                    swap_flag = ("  [!] d(top_margin) ~ +mount AND "
                                 "d(first_det) ~ 0 == SHADOW SWAP DID NOT TAKE "
                                 "(arm INVALID -- level camera, mount added "
                                 "only analytically)")
                elif dtm_med < mount - 1.0 and dfd_med > 0:
                    swap_flag = "  (swap took: d below +mount, d(first_det) > 0)"
            print(f"{label} (n_paired={len(common)}): "
                  f"d(first_det_range) med={dfd_med:+.2f} m, "
                  f"d(top_margin) med={dtm_med:+.1f} deg (mount +{mount})"
                  f"{swap_flag}")
            print(sign_line("miss", "m", dmiss, "tighter", "worse",
                            len(common), drop_miss))
            print(sign_line("first_det_range", "m", dfd, "closer", "further",
                            len(common), drop_fd))
            print(sign_line("engBelow", "frac", deb, "lower", "higher",
                            len(common), len(common) - len(deb), fmt="+.3f"))

    # ---- per-arm compensation-aware caveat (review finding, #40) ----------
    # The (c)-metrics fly UNCOMPENSATED ONLY on the ADR-0067 sweep; the #40
    # re-fly arms fly WITH --cam-mount-up-deg (compensation is the hypothesis
    # under test), so a blanket "UNCOMPENSATED" footer is FALSE on committed
    # re-fly evidence. State it per arm.
    comp = {lbl: is_compensated(p) for lbl, p in arm_path.items()}
    print("\nCAVEATS (per arm):")
    for lbl in per_arm:
        if per_arm[lbl][1]:
            print(f"  {lbl}: (c)-metrics "
                  + ("COMPENSATED (--cam-mount-up-deg active; guidance composes "
                     "the mount) -- this is the #40 design condition."
                     if comp[lbl] else
                     "UNCOMPENSATED (m4 assumes zero mount) -- worst case, not "
                     "the design ceiling (ADR-0067 sweep, or a mount-0 arm)."))
    print("  ALL arms: n small -> Wilson-CI language for any binomial claim "
          "(ADR-0041 F5); ~1 m single-flight miss noise; ~5 m false-lock "
          "noise at 12 m/s maneuver (ADR-0057).")


if __name__ == "__main__":
    main()
