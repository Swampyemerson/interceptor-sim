#!/usr/bin/env python3
"""PROBE 2 -- phantom-competition replay (headline next_probe, the SECOND of the
two un-eliminated in-view detection mechanisms; detector stage note + handoff
stage note + docs/project_state.json).

THE QUESTION
------------
In flight the forward camera on the STOCK mount sees the interceptor's own prop
arms as a "phantom" target -- 5-25 boxes/frame (0 static). The deployed seeker
takes the TOP-1 box (after a self-mask: |bearing|<=30 deg and no L/R frame-edge
touch, finetuned_seeker.detect()). If a target-hitting box EXISTS in a frame but a
higher-scoring PHANTOM wins top-1 (or the self-mask rejects the real box), the
detector reports a miss even though the target was localizable -- a SCORING-style
mirage at the SELECTION stage, distinct from a true detector failure. The fixed
box_scoring gate now lets us test this OFFLINE over existing in-flight captures:
score ALL boxes vs gt, and ask where the target-hitting box ranks.

WHY OFFLINE / WHY IT MATTERS EVEN THOUGH THE MOUNT FIXES IT
----------------------------------------------------------
The committed 3D-printed forward camera mount (handoff stage note, builder
2026-07-17) designs the props OUT of the FoV -> it REMOVES the phantom at source in
hardware, so this is NOT a lever to build. The probe's value is (1) EXPLAINING the
sim's in-flight approach recall ~0 (approach_recall.py 0.8% >=8 m) -- closing the
mirage ledger by decomposing "detector miss" into "true miss" vs "real box lost to
a phantom / self-mask"; and (2) QUANTIFYING the RESIDUAL risk if the mount's
prop-clearance is imperfect (a stray phantom still in frame). It runs over frames
already on disk -- no sim.

HONESTY: gt (`gt_*`) is used ONLY to score which boxes hit the real target
(labeling), never fed to guidance. box_scoring is the single fixed gate at its
unified defaults (extent 0.52 m, sec^2 off-axis) -- the same one Probe 1 and the
frame-top re-score use.

METRICS (per gt-range band, over positive/in-view frames)
  any_hit        : some box hits gt          -> the recall CEILING if selection were perfect
  top1_raw_hit   : the highest-SCORE box hits gt (naive argmax, no self-mask)
  deployed_hit   : finetuned_seeker.detect() (self-mask + top-1) hits gt -> the SHIPPED behavior
  gap            : any_hit AND NOT deployed_hit -> real box MASKED (the mechanism, quantified)
  target_hit_rank: score-rank of the best target-hitting box among all boxes
  topK recovery  : fraction of any-hit frames whose target box is within top-K by score
  loss reason    : for masked frames, WHY -- outscored-by-phantom vs self-mask-rejected
  score margin   : phantom_top1_score - best_target_score (small -> a margin gate could help)

Run under .venv-seeker (onnxruntime + cv2). --self-test needs neither (pure logic).

Usage:
  scripts/seeker/phantom_competition_replay.py \
      scripts/seeker/data/quad_r2l_flight_capture scripts/seeker/data/v3_flight_eval \
      --rmin 8 --rmax 12
  scripts/seeker/phantom_competition_replay.py --self-test
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from box_scoring import box_hits_gt, gt_scale_for, TARGET_EXTENT_M  # noqa: E402

FX = FY = 539.9363327026367
CX, CY = 640.0, 480.0
W_FULL, H_FULL = 1280, 960
MAX_BEARING_RAD = math.radians(30.0)   # finetuned_seeker.detect() self-mask
EDGE_MARGIN_PX = 40.0
_RANGE_RE = re.compile(r"_r(\d+\.?\d*)_")


# ==========================================================================
# pure selection logic (exercised by --self-test; no cv2/onnx)
# ==========================================================================
def rank_of_target_box(boxes, is_hit):
    """Score-rank (1 = highest score) of the best target-hitting box among all
    `boxes` (each (score,u,v,bw,bh)), or None if none hit. `is_hit(box)`->bool."""
    for i, bx in enumerate(sorted(boxes, key=lambda r: -r[0])):
        if is_hit(bx):
            return i + 1
    return None


def self_mask_ok(box, W=W_FULL):
    """True iff `box` survives finetuned_seeker.detect()'s self-mask
    (|bearing|<=30 deg AND not touching the L/R frame edge)."""
    _score, u, _v, bw, _bh = box
    bearing = math.atan2((u - CX) / FX, 1.0)
    if abs(bearing) > MAX_BEARING_RAD:
        return False
    if (u - bw / 2) < EDGE_MARGIN_PX or (u + bw / 2) > (W - EDGE_MARGIN_PX):
        return False
    return True


def deployed_top1(boxes):
    """The box finetuned_seeker.detect() would return: highest score among
    self-mask survivors, or None."""
    best = None
    for bx in boxes:
        if not self_mask_ok(bx):
            continue
        if best is None or bx[0] > best[0]:
            best = bx
    return best


def topk_recovers(boxes, is_hit, k):
    """True iff a target-hitting box is within the top-k by score."""
    for bx in sorted(boxes, key=lambda r: -r[0])[:k]:
        if is_hit(bx):
            return True
    return False


def _self_test() -> int:
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and cond
        print(f"[self-test] {name}: {'PASS' if cond else 'FAIL'}")

    # boxes: (score,u,v,bw,bh). target near centre; a higher-score phantom that
    # ALSO survives the self-mask (near boresight, no edge touch) -> it wins top-1.
    tgt = (0.55, 640.0, 300.0, 40.0, 40.0)
    ph1 = (0.80, 640.0, 700.0, 60.0, 60.0)   # phantom, near boresight, outscores
    boxes = [ph1, tgt]

    def is_hit(bx):  # a hit iff it's (approximately) the target box
        return abs(bx[1] - 640) < 30 and abs(bx[2] - 300) < 30

    check("rank of target box = 2 (one phantom outscores)",
          rank_of_target_box(boxes, is_hit) == 2)
    check("top-1 raw misses (phantom wins)",
          not is_hit(sorted(boxes, key=lambda r: -r[0])[0]))
    check("top-2 recovers", topk_recovers(boxes, is_hit, 2))
    check("top-1 does not recover", not topk_recovers(boxes, is_hit, 1))
    # self-mask standalone rejections
    edge_ph = (0.9, 30.0, 500.0, 80.0, 80.0)   # u-bw/2 = -10 < 40 -> rejected
    check("self-mask rejects edge-touching box", not self_mask_ok(edge_ph))
    check("self-mask keeps centred target", self_mask_ok(tgt))
    far_ph = (0.9, 1270.0, 480.0, 20.0, 20.0)  # bearing ~44 deg -> rejected
    check("self-mask rejects high-bearing box", not self_mask_ok(far_ph))
    check("self-mask keeps the near-boresight phantom", self_mask_ok(ph1))
    # deployed top-1 among survivors: ph1 (0.80) survives and outscores tgt
    dt = deployed_top1(boxes)
    check("deployed top-1 = surviving phantom (masks the target)",
          dt is not None and not is_hit(dt))
    # if the phantom is removed (mount), deployed top-1 = the target
    check("phantom-free -> deployed top-1 hits target",
          is_hit(deployed_top1([tgt])))

    print(f"[self-test] {'ALL PASS' if ok else 'SOME FAILED'}")
    return 0 if ok else 1


# ==========================================================================
# replay (needs onnx + cv2)
# ==========================================================================
def gt_range_of(name):
    m = _RANGE_RE.search(name)
    return float(m.group(1)) if m else None


def load_gt_box(lp):
    try:
        parts = open(lp).readline().split()
    except OSError:
        return None
    if len(parts) < 5:
        return None
    _, cx, cy, w, h = (float(v) for v in parts[:5])
    return cx * W_FULL, cy * H_FULL, w * W_FULL, h * H_FULL


def capture_extent(ddir, override):
    if override is not None:
        return override
    mp = os.path.join(ddir, "manifest.json")
    if os.path.exists(mp):
        try:
            return float(json.load(open(mp)).get("extent_m", 0.9))
        except (OSError, ValueError, TypeError):
            pass
    return 0.9


def run(args) -> int:
    import cv2
    from finetuned_seeker import FinetunedNNSeeker
    V2 = os.path.join(HERE, "weights", "drone_finetuned_quad_v2.onnx")
    if not os.path.exists(V2):
        print(f"detector weights missing: {V2}"); return 1
    det = FinetunedNNSeeker(FX, FY, CX, CY, V2, conf_thres=args.conf)

    os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
    pf = open(args.csv, "w", newline="")
    wr = csv.writer(pf)
    wr.writerow(["dataset", "frame", "gt_range", "gbearing_deg", "n_boxes",
                 "n_target_boxes", "n_phantom_boxes", "any_hit", "top1_raw_hit",
                 "deployed_hit", "target_hit_rank", "loss_reason",
                 "score_margin_phantom_minus_target"])

    # aggregates
    N = 0
    tot = defaultdict(int)      # any_hit / top1_raw_hit / deployed_hit counts
    rank_hist = defaultdict(int)
    topk = defaultdict(int)     # k -> # any-hit frames recovered within top-k
    loss_reason = defaultdict(int)
    margins = []
    per_band = defaultdict(lambda: defaultdict(int))   # band -> counters
    box_counts = []

    for ddir in args.datasets:
        cap_ext = capture_extent(ddir, args.capture_extent_m)
        gscale = gt_scale_for(cap_ext)
        sk = dict(gt_scale=gscale, offaxis_aware=True)
        name = os.path.basename(os.path.normpath(ddir))
        imgs = sorted(glob.glob(os.path.join(ddir, "images", "*.png")))
        lbl_dir = os.path.join(ddir, "labels")
        used = 0
        for ip in imgs:
            base = os.path.splitext(os.path.basename(ip))[0]
            gr = gt_range_of(base)
            if gr is None or not (args.rmin <= gr <= args.rmax):
                continue
            gt = load_gt_box(os.path.join(lbl_dir, base + ".txt"))
            if gt is None:
                continue  # negative frame (target not in view)
            frame = cv2.imread(ip)
            if frame is None:
                continue
            gcx, gcy, gw, gh = gt
            gbearing = math.degrees(math.atan2((gcx - CX) / FX, 1.0))
            boxes = det._infer_boxes(frame)
            if not boxes:
                continue  # no boxes at all -> a TRUE miss, not a competition case
            N += 1
            used += 1
            box_counts.append(len(boxes))

            def is_hit(bx):
                return box_hits_gt([bx], gcx, gcy, gw, gh, **sk)

            hitting = [bx for bx in boxes if is_hit(bx)]
            phantoms = [bx for bx in boxes if not is_hit(bx)]
            any_hit = len(hitting) > 0
            ordered = sorted(boxes, key=lambda r: -r[0])
            top1_raw_hit = is_hit(ordered[0])
            dep = deployed_top1(boxes)
            deployed_hit = dep is not None and is_hit(dep)
            rank = rank_of_target_box(boxes, is_hit)

            band = int(gr // 2) * 2
            tot["any"] += int(any_hit)
            tot["top1_raw"] += int(top1_raw_hit)
            tot["deployed"] += int(deployed_hit)
            per_band[band]["n"] += 1
            per_band[band]["any"] += int(any_hit)
            per_band[band]["top1_raw"] += int(top1_raw_hit)
            per_band[band]["deployed"] += int(deployed_hit)

            reason = ""
            margin = ""
            if any_hit:
                rank_hist[rank] += 1
                for k in range(1, 6):
                    if topk_recovers(boxes, is_hit, k):
                        topk[k] += 1
                if not deployed_hit:
                    # a real box existed but the SHIPPED detector missed it. Why?
                    best_tgt = max(hitting, key=lambda r: r[0])
                    if not self_mask_ok(best_tgt):
                        # would the real box survive the mask at all?
                        if any(self_mask_ok(bx) for bx in hitting):
                            reason = "phantom_outscores_within_mask"
                        else:
                            reason = "self_mask_rejects_target"
                    else:
                        reason = "phantom_outscores"
                    loss_reason[reason] += 1
                    if dep is not None:
                        margin = dep[0] - best_tgt[0]
                        margins.append(margin)
            wr.writerow([name, base, f"{gr:.1f}", f"{gbearing:+.1f}", len(boxes),
                         len(hitting), len(phantoms), int(any_hit),
                         int(top1_raw_hit), int(deployed_hit),
                         rank if rank else "", reason,
                         (f"{margin:.3f}" if margin != "" else "")])
        print(f"[replay] {name}: {used} in-view frames in "
              f"{args.rmin}-{args.rmax} m")
    pf.close()

    if N == 0:
        print("[replay] no in-view frames with boxes in the band -- "
              "check datasets / band."); return 1

    # ------------------------------------------------------------------ report
    import statistics
    print(f"\n=== PROBE 2: phantom competition, {args.rmin}-{args.rmax} m band "
          f"(conf {args.conf}, unified extent {TARGET_EXTENT_M} m, N={N} in-view "
          f"frames with >=1 box) ===")
    print(f"  boxes/frame: median {statistics.median(box_counts):.0f} "
          f"min {min(box_counts)} max {max(box_counts)}  (0 static -> phantom storm)")
    print(f"\n  {'metric':<34}{'rate':>16}")
    print(f"  {'any box hits target (ceiling)':<34}"
          f"{f'{100*tot['any']/N:.0f}% ({tot['any']}/{N})':>16}")
    print(f"  {'naive top-1 (argmax) hits':<34}"
          f"{f'{100*tot['top1_raw']/N:.0f}% ({tot['top1_raw']}/{N})':>16}")
    print(f"  {'DEPLOYED detect() hits (shipped)':<34}"
          f"{f'{100*tot['deployed']/N:.0f}% ({tot['deployed']}/{N})':>16}")
    gap = tot["any"] - tot["deployed"]
    print(f"\n  MASKED (any_hit but deployed missed) = {gap}/{N} "
          f"({100*gap/N:.0f}%)  <- the phantom-competition / self-mask loss")
    if tot["any"]:
        print(f"  ... = {100*gap/tot['any']:.0f}% of the {tot['any']} frames "
              f"where the target WAS localizable")

    print(f"\n  target-hit RANK distribution (among all boxes, of the "
          f"{sum(rank_hist.values())} any-hit frames):")
    for r in sorted(k for k in rank_hist if k is not None):
        print(f"    rank {r:<3}: {rank_hist[r]:>4}  ({100*rank_hist[r]/max(1,tot['any']):.0f}%)")

    print(f"\n  top-K recovery (target box within top-K by score):")
    for k in range(1, 6):
        print(f"    top-{k}: {topk[k]:>4}/{tot['any']}  "
              f"({100*topk[k]/max(1,tot['any']):.0f}% of localizable)")

    if loss_reason:
        print(f"\n  loss reason for the {gap} masked frames:")
        for r, c in sorted(loss_reason.items(), key=lambda kv: -kv[1]):
            print(f"    {r:<34}{c:>5}  ({100*c/max(1,gap):.0f}%)")
    if margins:
        print(f"\n  phantom_top1 - best_target score margin on masked frames: "
              f"median {statistics.median(margins):+.3f} "
              f"min {min(margins):+.3f} max {max(margins):+.3f}")
        small = sum(1 for m in margins if m < 0.10)
        print(f"    margin < 0.10: {small}/{len(margins)} "
              f"({100*small/len(margins):.0f}%)  <- a conf-vs-phantom margin gate "
              f"could flip these")

    print(f"\n  per-range band (n | any% | top1raw% | deployed%):")
    for band in sorted(per_band):
        d = per_band[band]
        print(f"    {band}-{band+2} m: n={d['n']:<4} "
              f"any={100*d['any']/d['n']:.0f}%  "
              f"top1raw={100*d['top1_raw']/d['n']:.0f}%  "
              f"deployed={100*d['deployed']/d['n']:.0f}%")

    if args.summary_out:
        summary = dict(
            band=[args.rmin, args.rmax], conf=args.conf, n=N,
            boxes_per_frame_median=statistics.median(box_counts),
            any_hit_rate=round(100 * tot["any"] / N, 1),
            top1_raw_rate=round(100 * tot["top1_raw"] / N, 1),
            deployed_rate=round(100 * tot["deployed"] / N, 1),
            masked_frames=gap, masked_pct_of_all=round(100 * gap / N, 1),
            masked_pct_of_localizable=(round(100 * gap / tot["any"], 1)
                                       if tot["any"] else None),
            rank_hist={str(k): v for k, v in rank_hist.items() if k is not None},
            topk={str(k): topk[k] for k in range(1, 6)},
            loss_reason=dict(loss_reason),
            score_margin_median=(round(statistics.median(margins), 3)
                                 if margins else None))
        with open(args.summary_out, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"\n[replay] summary -> {args.summary_out}")
    print(f"[replay] per-frame -> {args.csv}")
    print("[replay] (plot: run under .venv with --plot --csv <that csv> "
          "-- matplotlib lives in .venv, onnx in .venv-seeker)")
    return 0


# ==========================================================================
# --plot  (no onnx; matplotlib -> run under .venv, reads the per-frame CSV)
# ==========================================================================
def run_plot(args) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if not os.path.exists(args.csv):
        print(f"[plot] no CSV at {args.csv}"); return 1
    per_band = defaultdict(lambda: defaultdict(int))
    rank_hist = defaultdict(int)
    rmin, rmax = 1e9, -1e9
    for r in csv.DictReader(open(args.csv)):
        gr = float(r["gt_range"]); rmin = min(rmin, gr); rmax = max(rmax, gr)
        b = int(gr // 2) * 2
        per_band[b]["n"] += 1
        per_band[b]["any"] += int(r["any_hit"])
        per_band[b]["top1_raw"] += int(r["top1_raw_hit"])
        per_band[b]["deployed"] += int(r["deployed_hit"])
        if r["target_hit_rank"]:
            rank_hist[int(r["target_hit_rank"])] += 1
    os.makedirs(args.images_dir, exist_ok=True)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))
    bands = sorted(per_band)
    centers = [b + 1 for b in bands]
    for key, lbl, style in (("any", "any box hits (ceiling)", "o-"),
                            ("top1_raw", "naive top-1", "^--"),
                            ("deployed", "DEPLOYED detect()", "s-")):
        ys = [100 * per_band[b][key] / per_band[b]["n"] if per_band[b]["n"] else 0
              for b in bands]
        ax1.plot(centers, ys, style, lw=2, label=lbl)
    ax1.set_xlabel("gt range (m)"); ax1.set_ylabel("recall (%)")
    ax1.set_ylim(-5, 105); ax1.grid(alpha=0.3); ax1.legend(fontsize=8)
    ax1.set_title("Recall: localizable vs selected")
    ranks = sorted(rank_hist)
    if ranks:
        xs = list(range(1, min(max(ranks), 10) + 1))
        ax2.bar(xs, [rank_hist.get(r, 0) for r in xs], color="#d62728", alpha=0.8)
        ax2.set_xlabel("score-rank of the target-hitting box")
        ax2.set_ylabel("# frames"); ax2.grid(alpha=0.3, axis="y")
        ax2.set_title("Where the real box ranks amid phantoms")
    fig.suptitle(f"Probe 2: phantom competition {rmin:.0f}-{rmax:.0f} m "
                 f"(v2, conf {args.conf})")
    fig.tight_layout()
    p = os.path.join(args.images_dir, "probe2_phantom_competition.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"[plot] -> {p}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("datasets", nargs="*",
                    help="in-flight capture dirs (images/ + labels/)")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--plot", action="store_true",
                    help="plot from the per-frame --csv (no onnx; run under .venv)")
    ap.add_argument("--rmin", type=float, default=8.0)
    ap.add_argument("--rmax", type=float, default=12.0)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--capture-extent-m", type=float, default=None,
                    help="override the --extent-m the dataset was labelled at "
                         "(default: read manifest, else 0.9)")
    ap.add_argument("--csv", default=os.path.join(
        HERE, "..", "..", "logs", "phantom_competition_replay.csv"))
    ap.add_argument("--summary-out", default=None)
    ap.add_argument("--images-dir", default=None)
    args = ap.parse_args()
    if args.self_test:
        return _self_test()
    if args.plot:
        return run_plot(args)
    if not args.datasets:
        ap.error("give one or more dataset dirs, --self-test, or --plot")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
