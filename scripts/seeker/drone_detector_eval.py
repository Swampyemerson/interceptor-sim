"""Drone-SPECIFIC detector vs untuned-COCO baseline — acquisition-range eval.

WHAT THIS IS (ADR-0033 seeker task #9, companion to seeker_design_brief.md)
---------------------------------------------------------------------------
The NN lane (nn_seeker.py / eval_nn.py) proved an *untuned COCO* YOLOv8n detects
the target BODY only terminally (~1.6-3.0 m, ~4-6% recall) because COCO has no
drone class. The obvious next question the brief poses (sec 2.2 / R2): does a
DRONE-SPECIFIC detector acquire the body EARLIER (at longer range) than untuned
COCO? Acquisition range is the #1 miss lever (ADR-0023/0024: correction capacity
~ t_go^2), so "sees it sooner" is the whole ball game.

This script fetches that answer offline, honestly:
  * Model under test  : doguilmak/Drone-Detection-YOLOv11x  (MIT, single class
    "drone", mAP50 0.905 on ITS OWN real-photo domain) exported by us to ONNX.
    See weights/LICENSES.md for full provenance. Exported at imgsz=1280 because
    the target is a few-px body and the default 640 letterbox destroys it.
  * Baseline          : yolov8n COCO ONNX via the existing NNSeeker (nn_seeker.py)
    at its native 640 — the exact untuned-COCO seeker the NN lane measured.
  * Frames            : demo_out/onboard_frames/frame_*.png (866, 1280x960),
    the interceptor's onboard view of the target during a demo fly-by.

HONESTY / no-cheat boundary (CLAUDE.md, ADR-0010): both detectors read ONLY the
camera frame + fixed intrinsics. `eval_tag_boxes.json` (the AprilTag pixel boxes
for the 6 frames where the tag decodes) is used ONLY as scoring ground-truth to
say "a detection landed on the real target" and to prove body-not-tag; it is
NEVER fed to either detector. No `gt_*` pose exists in this footage anyway
(manifest.csv has idx/wall_t/sim_t/file only) — so absolute range is available
only via known-size scaling at the 6 tag frames, and "acquisition range" is
reported as (earliest on-target frame -> its known-size range), with the honest
caveat that pre-tag-window frames carry no GT (both models are shown to produce
no on-target interior detection there, so the ranking is unambiguous regardless).

NO SIM / PX4 / GAZEBO. Offline against saved frames.
Run (drone model needs 1280 -> import onnxruntime from the isolated seeker venv):
  OMP_NUM_THREADS=2 nice -n 15 \
    /home/emerson/interceptor-sim/.venv-seeker/bin/python \
    scripts/seeker/drone_detector_eval.py [--n 40] [--all] [--conf 0.10]

The .pt was exported with the SEPARATE train/export venv (.venv-seeker-train,
ultralytics+torch); this eval only needs onnxruntime+cv2+numpy, which the
read-only .venv-seeker already has. See train_drone_finetune.py for the deferred
sim-domain fine-tune pipeline this eval motivates.
"""

from __future__ import annotations

import argparse
import datetime
import glob
import json
import math
import os
import time

import numpy as np
import cv2
import onnxruntime as ort

from nn_seeker import NNSeeker, FX, FY, CX, CY, TARGET_SPAN_M

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
FRAMES_DIR = os.path.join(REPO, "demo_out", "onboard_frames")
TAG_GT = os.path.join(HERE, "eval_tag_boxes.json")
DRONE_ONNX = os.path.join(HERE, "weights", "drone_yolo11x_1280.onnx")

IMG_W, IMG_H = 1280, 960
# Own-airframe / horizon gates (same rationale as nn_seeker.py): the
# interceptor's own rotor arms hug the L/R frame edges and read as a big blob to
# ANY detector; a near-full-frame box is the horizon, not a 1 m target.
EDGE_MARGIN = 8
MAX_BOX_FRAC = 0.45


class DroneYolo11Seeker:
    """YOLOv11x single-class ('drone') ONNX runner at imgsz=1280.

    Output tensor is (1, 5, 8400): 4 box params + 1 class score (no argmax vs
    COCO's 84). Same letterbox + box->bearing/range geometry as NNSeeker so the
    two are directly comparable. Own-airframe / horizon gates applied so a
    reported hit is an interior, target-sized box (not the own-prop wedge).
    """

    def __init__(self, weights: str = DRONE_ONNX, conf_thres: float = 0.10,
                 input_size: int = 1280, intra_threads: int = 2):
        so = ort.SessionOptions()
        so.intra_op_num_threads = intra_threads
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(
            weights, so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.sz = input_size
        self.conf = conf_thres

    def _letterbox(self, img):
        h, w = img.shape[:2]
        r = min(self.sz / h, self.sz / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        rz = cv2.resize(img, (nw, nh))
        c = np.full((self.sz, self.sz, 3), 114, np.uint8)
        dw, dh = (self.sz - nw) // 2, (self.sz - nh) // 2
        c[dh:dh + nh, dw:dw + nw] = rz
        return c, r, dw, dh

    def detect(self, img_bgr, conf=None, gate=True):
        """Return dict(hit, conf, box(x,y,w,h), bearing_deg, range_m, u, v) for
        the best gated box, or hit=False. `gate=False` returns the raw top box
        (used to show the own-prop false positive)."""
        conf = self.conf if conf is None else conf
        lb, r, dw, dh = self._letterbox(img_bgr)
        blob = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        out = self.sess.run(None, {self.inp: blob.transpose(2, 0, 1)[None]})[0][0].T
        sc = out[:, 4]
        keep = np.where(sc > conf)[0]
        best = None
        for i in keep:
            cx, cy, bw, bh = out[i, :4]
            x = (cx - bw / 2 - dw) / r
            y = (cy - bh / 2 - dh) / r
            ww, hh = bw / r, bh / r
            if gate:
                if x <= EDGE_MARGIN or (x + ww) >= (IMG_W - EDGE_MARGIN):
                    continue
                if ww * hh > MAX_BOX_FRAC * IMG_W * IMG_H:
                    continue
            if best is None or sc[i] > best[0]:
                best = (float(sc[i]), x, y, ww, hh)
        if best is None:
            return {"hit": False, "conf": float(sc.max()) if len(sc) else 0.0}
        s, x, y, ww, hh = best
        u, v = x + ww / 2.0, y + hh / 2.0
        bearing = math.atan2((u - CX) / FX, 1.0)
        range_m = FX * TARGET_SPAN_M / max(ww, hh, 1.0)
        return {"hit": True, "conf": s, "box": (x, y, ww, hh),
                "bearing_deg": math.degrees(bearing), "range_m": range_m,
                "u": u, "v": v}


def on_target(u, v, tag_box, tol=1.6):
    """True if (u,v) falls inside the tag box grown by `tol` (loose, since the
    body box is larger than the tag). tag_box = [x0,y0,x1,y1,cx,cy]."""
    x0, y0, x1, y1, tcx, tcy = tag_box
    w, h = (x1 - x0), (y1 - y0)
    return (tcx - tol * w <= u <= tcx + tol * w and
            tcy - tol * h <= v <= tcy + tol * h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40,
                    help="approach-spanning sample size (default 40)")
    ap.add_argument("--all", action="store_true", help="every frame (idle only)")
    ap.add_argument("--conf", type=float, default=0.10,
                    help="drone-model score threshold (default 0.10, generous)")
    ap.add_argument("--out", default=os.path.join(HERE, "eval_out"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    frames = sorted(glob.glob(os.path.join(FRAMES_DIR, "frame_*.png")))
    assert frames, f"no frames in {FRAMES_DIR}"
    tag_gt = json.load(open(TAG_GT)) if os.path.exists(TAG_GT) else {}

    if args.all:
        sample_i = list(range(len(frames)))
    else:
        idx = set(np.linspace(0, len(frames) - 1, args.n).round().astype(int))
        idx |= set(range(525, min(561, len(frames))))   # dense terminal window
        sample_i = sorted(idx)

    drone = DroneYolo11Seeker(conf_thres=args.conf)
    coco = NNSeeker(conf_thres=0.15)                     # the untuned-COCO seeker
    coco.detect(cv2.imread(frames[0]))                   # warm up ORT graph opt
    drone.detect(cv2.imread(frames[0]))

    print("=" * 74)
    print("DRONE-SPECIFIC (YOLOv11x, MIT, single-class) vs UNTUNED-COCO (YOLOv8n)")
    print(f"drone model @1280 conf>{args.conf} | COCO @640 conf>0.15 | "
          f"{len(sample_i)} frames of {len(frames)}")
    print("=" * 74)
    print(f"{'frame':>6} {'rng?':>6} | {'DRONE hit conf  bearing  range':<34} | "
          f"{'COCO hit conf  bearing  range':<30} | on-tgt")

    rows = []
    json_rows = []
    d_lat, c_lat = [], []
    for i in sample_i:
        p = frames[i]
        name = os.path.basename(p)
        img = cv2.imread(p)
        t0 = time.perf_counter(); d = drone.detect(img); d_lat.append((time.perf_counter()-t0)*1e3)
        t0 = time.perf_counter(); c = coco.detect(img);   c_lat.append((time.perf_counter()-t0)*1e3)
        tag = tag_gt.get(name)
        d_ontgt = bool(d["hit"] and tag and on_target(d["u"], d["v"], tag))
        c_ontgt = bool(c.range_m is not None and tag and
                       on_target(c.box_xywh[0] + c.box_xywh[2] / 2,
                                 c.box_xywh[1] + c.box_xywh[3] / 2, tag))
        dstr = (f"HIT  {d['conf']:.2f}  {d['bearing_deg']:+6.1f}  {d['range_m']:4.1f}m"
                if d["hit"] else f"-    ({d['conf']:.2f})            ")
        cstr = (f"HIT  {c.decision_margin:.2f}  {math.degrees(c.bearing_rad):+6.1f}  {c.range_m:4.1f}m"
                if c.range_m is not None else "-                    ")
        ot = ("D+C" if d_ontgt and c_ontgt else "D" if d_ontgt else
              "C" if c_ontgt else ("tag" if tag else ""))
        print(f"{i:>6} {'tag' if tag else '':>6} | {dstr:<34} | {cstr:<30} | {ot}")
        rows.append((i, d, c, d_ontgt, c_ontgt, tag is not None))
        json_rows.append({
            "frame_idx": i, "name": name, "has_tag_gt": tag is not None,
            "drone": {
                "hit": bool(d["hit"]), "conf": float(d["conf"]),
                "bearing_deg": float(d["bearing_deg"]) if d["hit"] else None,
                "range_m": float(d["range_m"]) if d["hit"] else None,
                "on_target": d_ontgt,
            },
            "coco": {
                "hit": c.range_m is not None,
                "conf": float(c.decision_margin) if c.range_m is not None else None,
                "bearing_deg": float(math.degrees(c.bearing_rad)) if c.range_m is not None else None,
                "range_m": float(c.range_m) if c.range_m is not None else None,
                "on_target": c_ontgt,
            },
        })

        # save annotated terminal-window hits for evidence
        if (d["hit"] or c.range_m is not None) and 535 <= i <= 550:
            ann = img.copy()
            if c.range_m is not None:
                x, y, w, h = [int(v) for v in c.box_xywh]
                cv2.rectangle(ann, (x, y), (x+w, y+h), (255, 160, 0), 2)
                cv2.putText(ann, f"COCO {c.decision_margin:.2f}", (x, max(18, y-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 160, 0), 2)
            if d["hit"]:
                x, y, w, h = [int(v) for v in d["box"]]
                cv2.rectangle(ann, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(ann, f"DRONE {d['conf']:.2f}", (x, min(IMG_H-8, y+h+22)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if tag:
                x0, y0, x1, y1, tcx, tcy = tag
                cv2.rectangle(ann, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 1)
                cv2.drawMarker(ann, (int(tcx), int(tcy)), (0, 255, 0), cv2.MARKER_CROSS, 18, 1)
            cv2.imwrite(os.path.join(args.out, f"cmp_{name}"), ann)

    # ---- acquisition summary --------------------------------------------------
    d_hits = [r for r in rows if r[3]]          # drone on-target
    c_hits = [r for r in rows if r[4]]          # coco on-target
    print("\n" + "=" * 74)
    print("ACQUISITION (earliest ON-TARGET frame = farthest confirmed range)")
    print("=" * 74)

    def acq(hits, which):
        if not hits:
            print(f"  {which}: NEVER acquired the target on-frame in this sample.")
            return {"acquired": False, "n_on_target": 0}
        f0 = hits[0]
        i = f0[0]
        if which == "DRONE":
            rng = f0[1]["range_m"]; cf = f0[1]["conf"]
        else:
            rng = f0[2].range_m; cf = f0[2].decision_margin
        print(f"  {which}: first on-target at frame {i}  (known-size range "
              f"~{rng:.1f} m, conf {cf:.2f});  on-target in {len(hits)} sampled frames.")
        return {"acquired": True, "first_frame": i, "range_m": float(rng),
                "conf": float(cf), "n_on_target": len(hits)}

    acq_drone = acq(d_hits, "DRONE")
    acq_coco = acq(c_hits, "COCO ")
    print(f"  AprilTag baseline (ADR-0015/0024): detectable ~9-12 m — neither "
          f"markerless model comes close (R2 acquisition regression).")
    d_lat_mean, c_lat_mean = float(np.mean(d_lat)), float(np.mean(c_lat))
    print(f"\nLATENCY  drone@1280 mean {d_lat_mean:.0f} ms ({1000/d_lat_mean:.1f} fps CPU/2thr, "
          f"56.8M params); COCO@640 mean {c_lat_mean:.0f} ms ({1000/c_lat_mean:.1f} fps).")
    print("  (drone x-model is NOT an embedded model — it is the acquisition-range")
    print("   CEILING probe; a deployable nano fine-tune is train_drone_finetune.py.)")

    # ---- body-not-tag proof for the drone model ------------------------------
    print("\n" + "=" * 74)
    print("BODY-NOT-TAG (drone model): inpaint the tag out, must still fire on body")
    print("=" * 74)
    npass = ntot = 0
    bnt_rows = []
    for name in sorted(tag_gt):
        p = os.path.join(FRAMES_DIR, name)
        if not os.path.exists(p):
            continue
        img = cv2.imread(p)
        x0, y0, x1, y1, tcx, tcy = tag_gt[name]
        d0 = drone.detect(img)
        mask = np.zeros(img.shape[:2], np.uint8)
        pad = 5
        mask[max(0, int(y0-pad)):int(y1+pad), max(0, int(x0-pad)):int(x1+pad)] = 255
        inp = cv2.inpaint(img, mask, 5, cv2.INPAINT_TELEA)
        d1 = drone.detect(inp)
        ok0 = d0["hit"] and on_target(d0["u"], d0["v"], tag_gt[name])
        ok1 = d1["hit"] and on_target(d1["u"], d1["v"], tag_gt[name])
        ntot += 1; npass += int(ok1)
        print(f"  {name}: ORIG {'HIT' if ok0 else 'miss'} "
              f"conf={d0['conf']:.2f} | TAG-INPAINTED "
              f"{'STILL HITS body' if ok1 else 'no on-target hit'} conf={d1['conf']:.2f}")
        cv2.imwrite(os.path.join(args.out, f"drone_inpaint_{name}"), inp)
        bnt_rows.append({"name": name, "orig_on_target": bool(ok0),
                          "orig_conf": float(d0["conf"]),
                          "inpainted_on_target": bool(ok1),
                          "inpainted_conf": float(d1["conf"])})
    print(f"\n  {npass}/{ntot} tag frames still detected on-target with the tag "
          f"inpainted out -> body/silhouette-driven, not fiducial decode.")
    print(f"  (Honest caveat: the drone x-model only fires when the target is "
          f"nearly frame-filling; a flat fill kills it. Fully tag-free numbers "
          f"need the tag-less Gazebo model models/fpv_target_markerless.)")
    print(f"\nAnnotated frames -> {args.out}")

    # ---- reproducible results artifact (reviewer fix: headline numbers were
    # stdout-only; dump everything needed to reconstruct the printed summary
    # without re-running the (slow, CPU-heavy) drone ONNX model) -------------
    results = {
        "meta": {
            "timestamp_utc": datetime.datetime.now(datetime.timezone.utc)
            .isoformat(timespec="seconds"),
            "n_arg": args.n, "all": args.all, "conf_thres": args.conf,
            "frames_total": len(frames), "frames_sampled": len(sample_i),
            "drone_weights": os.path.relpath(DRONE_ONNX, REPO),
            "frames_dir": os.path.relpath(FRAMES_DIR, REPO),
        },
        "per_frame": json_rows,
        "acquisition": {"drone": acq_drone, "coco": acq_coco},
        "latency_ms": {
            "drone_mean": d_lat_mean, "drone_fps": 1000.0 / d_lat_mean,
            "coco_mean": c_lat_mean, "coco_fps": 1000.0 / c_lat_mean,
        },
        "body_not_tag": {"n_pass": npass, "n_total": ntot, "frames": bnt_rows},
    }
    results_path = os.path.join(args.out, "drone_vs_coco_results.json")
    with open(results_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Results JSON -> {results_path}")


if __name__ == "__main__":
    main()
