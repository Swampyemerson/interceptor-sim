#!/usr/bin/env python3
r"""ADR-0050 consolidated follow-up -- OPTIONAL bonus pass: does ground_v1
(ADR-0047/0049's single-range 160 m detector) generalize across a real RANGE
sweep (50/90/130/160 m), or does it only fire near its training band?
Answers the "detection-floor / real-detector generalization" question the
perfect-centroid sigma_R fit (scripts/multirange_sigma_fit.py) cannot --
that harness ASSUMES a centroid was found; this one tests whether the real
detector actually FINDS one, at ranges/apparent-sizes it never trained on.

PURE OFFLINE: runs ground_v1 on every already-captured frame in
logs/rig_captures/multirange_*/standoff_*m/{left,right}/*.png (reuses
scripts/t18_nn_sigma_validation.py's run_detector()/box_center() UNCHANGED)
and reports hit-rate + confidence + box size PER STANDOFF -- the honest,
disclosed caveat being that ground_v1 was trained EXCLUSIVELY on ~160 m
frames (ADR-0049), so standoffs 50/90/130 m are, by construction, entirely
out-of-domain for this detector (a different physical object scale it has
never seen) -- this is a GENERALIZATION test, not a second in-domain
validation.

VENV: needs ultralytics/torch -- .venv-seeker-train (matches
t18_nn_sigma_validation.py):
    .venv-seeker-train/bin/python scripts/multirange_nn_detection.py --root logs/rig_captures/multirange_run1
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import statistics  # noqa: E402

from t18_nn_sigma_validation import run_detector, box_center  # noqa: E402
from ground_station.triangulate import RigConfig, project_world_to_stereo_pixels, triangulate  # noqa: E402
from t18_sigma_validation import load_capture, true_range_from_rig  # noqa: E402

OUT_DIR = os.path.join(REPO_ROOT, "logs", "multirange_nn_detection")
DEFAULT_WEIGHTS = os.path.join(REPO_ROOT, "scripts", "seeker", "weights", "ground_v1.pt")


def discover_standoff_dirs(root):
    return sorted(glob.glob(os.path.join(root, "standoff_*m")))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True)
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS)
    ap.add_argument("--conf", type=float, default=0.25, help="matches check_t17.sh's own default")
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--out-dir", default=OUT_DIR,
                    help="T17-v2 addition: override the output dir (default unchanged) so this "
                         "script can be run once per weights set (e.g. ground_v1 vs ground_v2) "
                         "against the SAME --root without one run clobbering the other's CSVs.")
    args = ap.parse_args()

    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)
    standoff_dirs = discover_standoff_dirs(args.root)
    print(f"[multirange-nn] root={args.root}  weights={args.weights}  conf={args.conf}  out_dir={out_dir}")

    all_rows = []
    rigs_by_standoff = {}
    for d in standoff_dirs:
        tag = os.path.basename(d)
        meta, index_rows = load_capture(d)
        rig = RigConfig.from_capture_meta(os.path.join(d, "capture_meta.json"))
        rigs_by_standoff[tag] = rig
        frames = []
        for row in index_rows:
            seq = int(row["seq"])
            gt_world = (float(row["gt_target_x"]), float(row["gt_target_y"]), float(row["gt_target_z"]))
            true_R = true_range_from_rig(rig, gt_world)
            for camera in ("left", "right"):
                path = os.path.join(d, camera, f"{seq:05d}.png")
                frames.append({"standoff": tag, "seq": seq, "camera": camera, "path": path,
                                "gt_world": gt_world, "true_R_m": true_R})
        image_paths = [f["path"] for f in frames]
        dets = run_detector(args.weights, image_paths, args.conf, args.imgsz)
        for f, (hit, conf, box) in zip(frames, dets):
            f["hit"] = hit
            f["det_conf"] = conf
            if hit:
                u, v = box_center(box)
                x0, y0, x1, y1 = box
                f["det_u"], f["det_v"] = u, v
                f["box_w_px"] = x1 - x0
                f["box_h_px"] = y1 - y0
                f["box_diag_px"] = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            else:
                f["det_u"] = f["det_v"] = None
                f["box_w_px"] = f["box_h_px"] = f["box_diag_px"] = None
            all_rows.append(f)

        n = len(frames)
        n_hit = sum(1 for f in frames if f["hit"])
        mean_R = sum(f["true_R_m"] for f in frames) / n
        hit_sizes = [f["box_diag_px"] for f in frames if f["hit"]]
        mean_size = sum(hit_sizes) / len(hit_sizes) if hit_sizes else None
        print(f"[multirange-nn] {tag:>16}: R~{mean_R:6.1f} m  hit={n_hit}/{n} "
              f"({n_hit/n:.0%})  " +
              (f"mean_box_diag={mean_size:.1f} px" if mean_size else "no hits"))

    csv_path = os.path.join(out_dir, "nn_detections_multirange.csv")
    fieldnames = ["standoff", "seq", "camera", "true_R_m", "hit", "det_conf",
                  "det_u", "det_v", "box_w_px", "box_h_px", "box_diag_px"]
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames)
        w.writeheader()
        for r in all_rows:
            w.writerow({k: r.get(k) for k in fieldnames})
    print(f"[multirange-nn] wrote {csv_path} ({len(all_rows)} rows)")

    # summary
    summary = {}
    for tag in sorted(set(r["standoff"] for r in all_rows)):
        sub = [r for r in all_rows if r["standoff"] == tag]
        n_hit = sum(1 for r in sub if r["hit"])
        summary[tag] = {
            "n": len(sub), "n_hit": n_hit, "hit_rate": n_hit / len(sub),
            "mean_true_R_m": sum(r["true_R_m"] for r in sub) / len(sub),
            "mean_box_diag_px": (sum(r["box_diag_px"] for r in sub if r["hit"]) / n_hit) if n_hit else None,
            "mean_conf": (sum(r["det_conf"] for r in sub if r["hit"]) / n_hit) if n_hit else None,
        }
    summary_path = os.path.join(out_dir, "nn_detection_summary.json")
    with open(summary_path, "w") as fh:
        json.dump(summary, fh, indent=2)
    print(f"[multirange-nn] wrote {summary_path}")
    print(json.dumps(summary, indent=2))

    # ------------------------------------------------------------------
    # REAL-CENTROID triangulation (not gt-projected/perfect) -- the true
    # "held-out real-detector sigma_R vs R" question, thin n=5 pairs/bin,
    # honestly disclosed as such (n=5 is NOT a robust std estimate; this is
    # a sanity/trend check, not a statistical claim).
    # ------------------------------------------------------------------
    print("\n" + "=" * 78)
    print("REAL-CENTROID triangulation (actual detected boxes, not gt-projected)")
    print("=" * 78)
    by_standoff_seq = {}
    for r in all_rows:
        by_standoff_seq.setdefault((r["standoff"], r["seq"]), {})[r["camera"]] = r

    tri_rows = []
    for (tag, seq), cams in sorted(by_standoff_seq.items()):
        fl, fr = cams.get("left"), cams.get("right")
        if fl is None or fr is None or not fl["hit"] or not fr["hit"]:
            continue
        rig = rigs_by_standoff[tag]
        res = triangulate(fl["det_u"], fl["det_v"], fr["det_u"], fr["det_v"],
                           rig.true_pose, rig.baseline_m, rig.f_px, rig.cx, rig.cy)
        if res is None:
            print(f"  {tag} seq={seq}: degenerate (disparity<=0) -- skipping")
            continue
        true_R = fl["true_R_m"]
        tri_rows.append({"standoff": tag, "seq": seq, "true_R_m": true_R,
                          "triangulated_R_m": res.range_m, "residual_m": res.range_m - true_R})

    print(f"{'standoff':>16} {'seq':>4} {'true_R':>8} {'tri_R':>8} {'resid':>8}")
    for r in tri_rows:
        print(f"{r['standoff']:>16} {r['seq']:>4} {r['true_R_m']:>8.2f} "
              f"{r['triangulated_R_m']:>8.2f} {r['residual_m']:>+8.3f}")

    real_sigma_by_standoff = {}
    for tag in sorted(set(r["standoff"] for r in tri_rows)):
        resids = [r["residual_m"] for r in tri_rows if r["standoff"] == tag]
        mean_R = statistics.mean(r["true_R_m"] for r in tri_rows if r["standoff"] == tag)
        if len(resids) >= 2:
            sigma = statistics.pstdev(resids)
        else:
            sigma = None
        real_sigma_by_standoff[tag] = {
            "n": len(resids), "mean_R_m": mean_R,
            "sigma_R_m": sigma, "mean_residual_bias_m": statistics.mean(resids) if resids else None,
        }
        print(f"[multirange-nn] {tag:>16}: n={len(resids)}  mean_R={mean_R:.1f} m  "
              f"REAL-CENTROID sigma_R={sigma if sigma is not None else 'n/a (need >=2)'}  "
              f"mean_bias={statistics.mean(resids):+.3f} m" if resids else "no usable pairs")

    tri_csv = os.path.join(out_dir, "real_centroid_triangulation.csv")
    if tri_rows:
        with open(tri_csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(tri_rows[0].keys()))
            w.writeheader()
            for r in tri_rows:
                w.writerow(r)
        print(f"[multirange-nn] wrote {tri_csv}")

    real_sigma_path = os.path.join(out_dir, "real_centroid_sigma_by_standoff.json")
    with open(real_sigma_path, "w") as fh:
        json.dump(real_sigma_by_standoff, fh, indent=2, default=str)
    print(f"[multirange-nn] wrote {real_sigma_path}")
    _ns = [v.get("n", 0) for v in real_sigma_by_standoff.values()]
    _nstr = f"{min(_ns)}-{max(_ns)}" if _ns else "?"
    print(f"\n[multirange-nn] HONEST CAVEAT: n={_nstr} pairs/standoff is thin for a robust "
          "sigma_R estimate (vs t18_sigma_validation's n_trials in the hundreds/thousands per "
          "point) -- read this as a coarse SANITY TREND, not a statistical confirmation. "
          "(A detector trained on a NARROW range band is out-of-domain at other ranges -- "
          "ground_v1 was ~160 m only, ADR-0049; ground_v2 spans 50-160 m, ADR-0055.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
