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

PRE-REGISTERED CAVEAT on (c): m4 maps camera bearings to body assuming ZERO
mount, so tilted arms fly with an uncompensated elevation bias = mount angle;
miss/clean there measure the UNCOMPENSATED worst case, not the design ceiling.

MECHANISM CHECK (shadow-swap live validation): if a tilted arm's first-det
vert-in-camera does NOT shift by ~ the mount angle vs control (up00), the
models/mono_cam shadow did not take -- treat the arm as INVALID (resolution-
order assumption failed) and stop the sweep.

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
    for label, mount, s in arms:
        runs = {}
        for row in csv.DictReader(open(s)):
            fp = row["flight_csv_path"]
            if not os.path.exists(fp):
                print(f"[skip] {label} run {row['run_idx']}: flight csv missing")
                continue
            res, err = analyze_run_mount(fp, mount)
            if err:
                print(f"[skip] {label} run {row['run_idx']}: {err}")
                continue
            res["summary"] = row
            runs[row["run_idx"]] = res
        per_arm[label] = (mount, runs)

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

    # ---- paired per-seed deltas vs the zero-mount arm --------------------
    zero = next((lbl for lbl, (m, r) in per_arm.items() if m == 0 and r), None)
    if zero:
        z_runs = per_arm[zero][1]
        print(f"\n===== PAIRED PER-SEED DELTAS vs {zero} (same run_idx = same "
              "cue/path seeds) =====")
        for label, (mount, runs) in per_arm.items():
            if label == zero or not runs:
                continue
            common = sorted(set(runs) & set(z_runs), key=int)
            if not common:
                print(f"{label}: no common run_idx with control")
                continue
            dfd, dtm = [], []
            for k in common:
                a, b = runs[k], z_runs[k]
                fa = a["summary"].get("first_det_range_m")
                fb = b["summary"].get("first_det_range_m")
                if fa and fb:
                    dfd.append(float(fa) - float(fb))
                if a["first_det"] and b["first_det"]:
                    dtm.append(a["first_det"]["top_margin_deg"]
                               - b["first_det"]["top_margin_deg"])
            print(f"{label} (n_paired={len(common)}): "
                  f"d(first_det_range) med={np.median(dfd):+.2f} m, "
                  f"d(top_margin) med={np.median(dtm):+.1f} deg "
                  f"(mechanism check: expect ~ +{mount} deg; if ~0 the "
                  "shadow swap DID NOT take -- arm invalid)")
    print("\nCAVEATS: (c)-metrics (miss/clean/engBelow) fly UNCOMPENSATED "
          "(m4 assumes zero mount) -- worst case, not the design ceiling. "
          "n=8/arm: Wilson-CI language for any binomial claim (ADR-0041 F5); "
          "~1 m single-flight miss noise; ~5 m false-lock noise at "
          "12 m/s maneuver (ADR-0057).")


if __name__ == "__main__":
    main()
