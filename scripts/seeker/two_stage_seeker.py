"""Two-stage markerless seeker: cheap PROPOSAL -> NN CLASSIFIER on a crop.

ADR-0033 tasks #10 (two-stage seeker) + #12 (fix the classical self-lock) -- the
SAME fix. Companion to docs/seeker_design_brief.md (s6 detect-then-track) and
docs/seeker_prototype_results.md (s3.4: the standalone classical seeker emits a
CONFIDENT WRONG bearing on the interceptor's OWN prop-arm, which would latch the
alpha-beta LOS filter and steer pro-nav at itself).

HEADLINE, RE-STATED HONESTLY (post-review): what this file VALIDATES today is the
self-lock fix (task #12) -- architectural, reproduced NN-free on 2 frames (own-prop
frame 0, real-target frame 540), see _run_eval's "SELF-LOCK FIX" section. The
acquisition-range extension implied by the "ACQUISITION CONTINUITY" section below
(confirms as early as frame ~528) is NOT independently validated: the AprilTag GT
(eval_tag_boxes.json) covers only frames 540-545, so the 528-539 confirms have no
ground truth behind them, only LOS-continuity + visual inspection. It is also
CONTINGENT on the untuned COCO classifier's context-dependent drone->"airplane"
response (see the CROP+UP honest finding below), which the crop+upsample path can
break rather than fix. Treat "extends acquisition to ~frame 528" as a MECHANISM
claim (the pipeline can propagate a confirm earlier when the classifier happens to
fire), not a measured P_detect(R) result. A real acquisition-range number needs
models/fpv_target_markerless (tag-free target model) with gt logging in Gazebo, per
docs/seeker_acquisition_range_note.md -- not this frame-replay eval.

THE ARCHITECTURE (ADR-0015 s2 detect-then-track / the YOLOMG motion-channel idea)
---------------------------------------------------------------------------------
Running the big NN on all 1280x960 pixels is expensive AND, on a few-pixel target,
low-recall (letterboxing 1280->640 halves the already-tiny target -> R2 acquisition
regression, seeker_prototype_results.md s2/s4). Instead:

  (1) PROPOSAL  -- classical dark-blob saliency (+ optional ego-motion frame diff)
                   returns candidate ROIs. Cheap, wide-area, NO bearing emitted.
  (2) SELF-MASK -- drop proposals that touch the FOV periphery (the interceptor's
     + HORIZON    own rotor arms are body-fixed at the L/R/top edges -- verified:
                   frame 0/565 own-prop box touches x=0; the target is INTERIOR at
                   acquisition, frame 545 box x0=371) + drop wide sky/ground-edge
                   boxes (horizon reject).
  (3) CROP+UP   -- crop each surviving ROI (padded to include the airframe span,
                   not just the dark core) and upsample small/far crops so the
                   few-pixel target fills the NN input -> recovers effective
                   resolution the full-frame NN threw away. HONEST FINDING (this
                   footage, COCO nano): the untuned COCO net's drone->"airplane"
                   response is a FULL-SCENE-context artifact (it needs the arm
                   "wings" + sky), so a TIGHT upsampled crop makes it read
                   laptop/clock and a CONTEXT-PRESERVING crop is what recovers it.
                   The upsample path is exercised for far crops, but its detection
                   payoff is contingent on an appearance-based classifier -- a
                   fine-tuned single-class drone nano is the drop-in that realizes
                   the resolution-recovery (mechanism; see docs/seeker_prototype_results.md).
  (4) CLASSIFY  -- run the nano NN (nn_seeker, imported READ-ONLY) on the crop as a
                   verifier. Emit a Measurement-compatible SeekerDetection ONLY for
                   a CONFIRMED crop, with the bearing computed from the NN box
                   centroid mapped back to FULL-image pixels via camera intrinsics.

WHY THIS KILLS THE SELF-LOCK (task #12)
---------------------------------------
The proposal stage never emits a standalone bearing, so a dark own-prop blob is no
longer a "detection" -- it is a candidate the self-mask removes (it touches an edge)
and, even if one slipped through, the NN classifier gate would have to confirm it.
A missing detection the alpha-beta filter COASTS through (safe, by design); a
confident wrong bearing it does NOT (the hazard we remove).

HONESTY BOUNDARY (CLAUDE.md / ADR-0010): reads ONLY camera pixels + fixed
intrinsics; never any gt_* field. The AprilTag boxes used in --eval are
EVAL-ONLY ground truth, never consumed by the seeker.

Run (offline, isolated venv, modest load):
  OMP_NUM_THREADS=2 nice -n 15 \
    /home/emerson/interceptor-sim/.venv-seeker/bin/python \
    scripts/seeker/two_stage_seeker.py --eval [--n 48] [--motion]
"""

from __future__ import annotations

import math
import os
import sys
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classical_seeker import ClassicalSeeker, Proposal, load_intrinsics  # noqa: E402
from nn_seeker import NNSeeker, SeekerDetection, TARGET_SPAN_M  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
FRAMES_DIR = os.path.join(REPO, "demo_out", "onboard_frames")


class TwoStageSeeker:
    """Proposal -> self-mask/horizon -> crop+upsample -> NN classify pipeline.

    Emits a `nn_seeker.SeekerDetection` (Measurement-compatible) for the single
    highest-confidence CONFIRMED candidate, or a None-filled detection when
    nothing is confirmed (the guidance filter then coasts -- safe)."""

    def __init__(self, fx: float, fy: float, cx: float, cy: float,
                 nn: Optional[NNSeeker] = None,
                 proposer: Optional[ClassicalSeeker] = None,
                 self_mask_margin: int = 10,
                 horizon_max_width_frac: float = 0.55,
                 crop_pad_frac: float = 3.5, crop_up_long: int = 640,
                 nn_conf: float = 0.08, restrict_classes: bool = True,
                 agree_factor: float = 2.5, target_span_m: float = TARGET_SPAN_M):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.self_mask_margin = self_mask_margin
        self.horizon_max_width_frac = horizon_max_width_frac
        self.crop_pad_frac = crop_pad_frac
        self.crop_up_long = crop_up_long
        self.agree_factor = agree_factor
        self.target_span_m = target_span_m
        self.proposer = proposer or ClassicalSeeker(fx, fy, cx, cy)
        # NN as a CROP CLASSIFIER: the self-mask now lives in the proposal stage
        # (full-image), so disable the NN's own border reject (a tight crop's
        # target legitimately touches the crop edge) and raise max_box_frac (the
        # target should FILL the crop). Keep the class gate -- that is the verifier.
        self.nn = nn or NNSeeker(conf_thres=nn_conf, self_mask_margin=0,
                                 max_box_frac=0.99, restrict_classes=restrict_classes,
                                 fx=fx, fy=fy, cx=cx, cy=cy,
                                 target_span_m=target_span_m)

    # ------------------------------------------------------ stage-2 gates
    def _passes_self_mask(self, box, W: int, H: int) -> bool:
        """Reject own prop-arms: body-fixed, they hug the L/R/top FOV edges.
        A confirmed target is interior at acquisition (frame 545 x0=371)."""
        x, y, w, h = box
        m = self.self_mask_margin
        if x <= m or (x + w) >= (W - m) or y <= m:
            return False
        return True

    def _passes_horizon(self, box, W: int) -> bool:
        """Reject the sky/ground boundary and long thin blade wedges: any box
        wider than a fraction of the frame is scene structure, not a compact
        target (the target span is a bounded angular size)."""
        return box[2] <= self.horizon_max_width_frac * W

    # ------------------------------------------------------ stage-3 crop+up
    def _crop_upsample(self, frame_bgr, box):
        """Pad the ROI, crop, and upsample so its long side ~= crop_up_long.
        Returns (crop_up, ox, oy, up) where a crop-local pixel (lx,ly) maps to
        full-image (ox + lx/up, oy + ly/up)."""
        H, W = frame_bgr.shape[:2]
        x, y, w, h = box
        pad = self.crop_pad_frac
        x0 = max(0, int(x - pad * w)); y0 = max(0, int(y - pad * h))
        x1 = min(W, int(x + w + pad * w)); y1 = min(H, int(y + h + pad * h))
        crop = frame_bgr[y0:y1, x0:x1]
        ch, cw = crop.shape[:2]
        if ch == 0 or cw == 0:
            return None, 0, 0, 1.0
        up = self.crop_up_long / max(ch, cw)
        up = max(1.0, up)  # only upsample small crops; never shrink
        crop_up = cv2.resize(crop, (int(round(cw * up)), int(round(ch * up))),
                             interpolation=cv2.INTER_CUBIC)
        return crop_up, x0, y0, up

    # ------------------------------------------------------ geometry
    def _geometry(self, u: float, v: float, w_px: float, h_px: float):
        """Full-image box centroid (u,v) + apparent size -> bearing/range/xyz."""
        bearing_h = math.atan2((u - self.cx) / self.fx, 1.0)   # + = right
        bearing_v = math.atan2((v - self.cy) / self.fy, 1.0)   # + = down
        range_m = self.fx * self.target_span_m / max(w_px, h_px, 1.0)
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        ray = ray / np.linalg.norm(ray)
        return bearing_h, bearing_v, range_m, range_m * ray

    # ------------------------------------------------------ full pipeline
    def detect(self, frame_bgr, t_mono: Optional[float] = None,
               prev_gray: Optional[np.ndarray] = None, include_motion: bool = False,
               return_debug: bool = False):
        if t_mono is None:
            t_mono = time.monotonic()
        H, W = frame_bgr.shape[:2]
        proposals = self.proposer.propose(frame_bgr, prev_gray=prev_gray,
                                           include_motion=include_motion)
        survivors = [p for p in proposals
                     if self._passes_self_mask(p.box, W, H)
                     and self._passes_horizon(p.box, W)]
        rejected = [p for p in proposals if p not in survivors]

        best = None  # (score, full_box, name)
        for p in survivors:
            crop_up, ox, oy, up = self._crop_upsample(frame_bgr, p.box)
            if crop_up is None:
                continue
            cands = self.nn._candidates(crop_up)   # NN as classifier on the crop
            if not cands:
                continue
            # PROPOSAL-AGREEMENT gate: the NN VERIFIES this proposal, so keep only
            # an NN box whose (mapped) centroid lands on the proposal blob -- not a
            # free "airplane" the COCO net hallucinates on the sky context elsewhere
            # in the crop. Agreement radius scales with the proposal size (the NN
            # box encloses the whole airframe span, larger than the dark-core blob).
            pcx, pcy = p.centroid
            agree = self.agree_factor * max(p.box[2], p.box[3])
            for score, name, (bx, by, bw, bh) in cands:
                full_box = (ox + bx / up, oy + by / up, bw / up, bh / up)
                fu, fv = full_box[0] + full_box[2] / 2, full_box[1] + full_box[3] / 2
                if math.hypot(fu - pcx, fv - pcy) > agree:
                    continue
                if best is None or score > best[0]:
                    best = (score, full_box, name)
                break  # highest-scoring agreeing candidate for this proposal

        if best is None:
            det = SeekerDetection(t_mono, None, None, None, None, None, 0)
            return (det, proposals, survivors, rejected) if return_debug else det

        score, (fx0, fy0, fw, fh), name = best
        u, v = fx0 + fw / 2.0, fy0 + fh / 2.0
        bh_, bv_, rng, xyz = self._geometry(u, v, fw, fh)
        det = SeekerDetection(
            t_mono=t_mono, range_m=rng, bearing_rad=bh_, bearing_vert_rad=bv_,
            meas_xyz=xyz, decision_margin=score, n_detections=len(survivors),
            box_xywh=(fx0, fy0, fw, fh), class_name=name)
        return (det, proposals, survivors, rejected) if return_debug else det


# ===========================================================================
#  Offline evaluation (no sim / PX4 / Gazebo)
# ===========================================================================
def _tag_gt():
    import json
    p = os.path.join(HERE, "eval_tag_boxes.json")
    return json.load(open(p)) if os.path.exists(p) else {}


def _box_covers(box, cx, cy) -> bool:
    if box is None:
        return False
    x, y, w, h = box
    return x <= cx <= x + w and y <= cy <= y + h


def _touches_border(box, W, H, m=10) -> bool:
    x, y, w, h = box
    return x <= m or (x + w) >= (W - m) or y <= m


def _run_eval(n_sample: int, use_motion: bool, out_dir: str):
    import glob
    frames = sorted(glob.glob(os.path.join(FRAMES_DIR, "frame_*.png")))
    assert frames, f"no frames in {FRAMES_DIR}"
    os.makedirs(out_dir, exist_ok=True)
    fx, fy, cx, cy = load_intrinsics(os.path.join(REPO, "camera_intrinsics.json"))
    tag_gt = _tag_gt()

    # sample: uniform spread across approach UNION the terminal target window
    idx = set(np.linspace(0, len(frames) - 1, n_sample).round().astype(int))
    idx |= {i for i in range(525, min(561, len(frames)))}
    sample = [int(i) for i in sorted(idx)]

    seeker = TwoStageSeeker(fx, fy, cx, cy)
    classical = ClassicalSeeker(fx, fy, cx, cy)   # standalone, for the A/B on self-lock

    # warm up ORT graph optimisation
    seeker.detect(cv2.imread(frames[sample[0]]))

    prev_gray = None
    n_prop = n_e2e = 0
    latencies = []
    approach_track = []   # (frame_idx, u, bearing_deg) for interior confirms, continuity check
    # target-present GT frames (from tag boxes)
    gt_frames = {int(k.split("_")[1].split(".")[0]): v for k, v in tag_gt.items()}
    prop_recall_hits = prop_recall_tot = 0
    e2e_recall_hits = e2e_recall_tot = 0
    # self-lock accounting
    twostage_border_locks = 0        # two-stage confirmed dets at the border (should be 0)
    twostage_any = 0
    classical_border_locks = 0       # standalone classical dets at the border (the bug)
    classical_any = 0
    det_rows = []

    for i in sample:
        img = cv2.imread(frames[i])
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        pg = prev_gray if (use_motion and prev_gray is not None) else None
        t0 = time.perf_counter()
        det, proposals, survivors, rejected = seeker.detect(
            img, prev_gray=pg, include_motion=use_motion, return_debug=True)
        latencies.append((time.perf_counter() - t0) * 1e3)
        prev_gray = gray

        # ---- standalone classical (the self-locking baseline) ----
        cdet = classical.detect(img)
        if cdet.detected():
            classical_any += 1
            if _touches_border(cdet.box, img.shape[1], img.shape[0]):
                classical_border_locks += 1

        has_prop = len(proposals) > 0
        n_prop += int(has_prop)
        confirmed = det.range_m is not None
        n_e2e += int(confirmed)
        if confirmed:
            twostage_any += 1
            u_c = det.box_xywh[0] + det.box_xywh[2] / 2.0
            approach_track.append((i, u_c, math.degrees(det.bearing_rad)))
            if _touches_border(det.box_xywh, img.shape[1], img.shape[0],
                               seeker.self_mask_margin):
                twostage_border_locks += 1

        # ---- recall vs GT (only frames where the tag gives us a target box) ----
        if i in gt_frames:
            x0, y0, x1, y1, tcx, tcy = gt_frames[i]
            prop_recall_tot += 1
            e2e_recall_tot += 1
            prop_hit = any(_box_covers(p.box, tcx, tcy) for p in proposals)
            prop_recall_hits += int(prop_hit)
            e2e_hit = confirmed and _box_covers(det.box_xywh, tcx, tcy)
            e2e_recall_hits += int(e2e_hit)
            det_rows.append(
                f"  frame {i:3d} [GT target u={tcx:.0f}]: "
                f"proposals={len(proposals)} survivors={len(survivors)} "
                f"prop_hit={prop_hit} "
                f"e2e={'CONFIRM u=%.0f b=%+.1fdeg r~%.1fm cls=%s' % (det.box_xywh[0]+det.box_xywh[2]/2, math.degrees(det.bearing_rad), det.range_m, det.class_name) if e2e_hit else 'miss'}")
            # save annotated evidence
            ann = img.copy()
            for p in rejected:
                x, y, w, h = p.box
                cv2.rectangle(ann, (x, y), (x + w, y + h), (60, 60, 200), 1)
            for p in survivors:
                x, y, w, h = p.box
                cv2.rectangle(ann, (x, y), (x + w, y + h), (0, 180, 255), 2)
            if confirmed:
                x, y, w, h = [int(v) for v in det.box_xywh]
                cv2.rectangle(ann, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.drawMarker(ann, (int(x + w / 2), int(y + h / 2)), (0, 255, 0),
                               cv2.MARKER_CROSS, 26, 2)
            cv2.drawMarker(ann, (int(tcx), int(tcy)), (0, 0, 255),
                           cv2.MARKER_TILTED_CROSS, 22, 2)
            cv2.imwrite(os.path.join(out_dir, f"twostage_{i:04d}.png"), ann)

    lat = np.array(latencies)
    N = len(sample)
    print("=" * 72)
    print("TWO-STAGE MARKERLESS SEEKER EVAL  (classical proposal -> NN-on-crop)")
    print(f"sample: {N} frames (uniform approach + terminal 525-560), "
          f"motion_channel={'ON' if use_motion else 'off'}")
    print("=" * 72)
    print(f"proposal stage fired (>=1 ROI): {n_prop}/{N} frames")
    print(f"end-to-end CONFIRMED (prop->self-mask->NN): {n_e2e}/{N} frames")
    print(f"latency/frame (whole pipeline, CPU 2thr): mean {lat.mean():.0f} ms "
          f"median {np.median(lat):.0f} ms p95 {np.percentile(lat,95):.0f} ms "
          f"-> {1000/lat.mean():.1f} fps")
    print()
    print("--- RECALL vs AprilTag GT (frames 540-545, EVAL-ONLY GT) ---")
    if prop_recall_tot:
        print(f"proposal recall (ROI covers target): {prop_recall_hits}/{prop_recall_tot}")
        print(f"end-to-end recall (NN confirms target): {e2e_recall_hits}/{e2e_recall_tot}")
    for r in det_rows:
        print(r)
    print()
    print("--- SELF-LOCK FIX (task #12): confirmed detections at the FOV border ---")
    print(f"standalone classical detect(): {classical_border_locks}/{classical_any} "
          f"detections are own-prop EDGE locks (the confident-wrong-bearing bug)")
    print(f"two-stage confirmed detections: {twostage_border_locks}/{twostage_any} "
          f"at the border  -> own-prop false lock {'GONE' if twostage_border_locks == 0 else 'STILL PRESENT'}")

    # ---- ACQUISITION CONTINUITY: how EARLY does the target get acquired? ----
    # UNVALIDATED / MECHANISM-ONLY (post-review correction): the tag only DECODES
    # at frame 540+ (eval_tag_boxes.json has exactly 6 entries, 540-545), so this
    # section has NO ground truth to score against below frame 540. A smooth LOS
    # track (small frame-to-frame |dbearing|) is consistent with one real target,
    # but "consistent with" is not "confirmed" -- do not read the numbers below as
    # a measured acquisition-range extension. See docs/seeker_acquisition_range_note.md
    # for what a real P_detect(R) measurement requires (tag-free Gazebo model + gt).
    print()
    print("--- ACQUISITION CONTINUITY (UNVALIDATED below frame 540 -- no GT; "
          "mechanism-only, NOT a measured acquisition-range result) ---")
    appr = [t for t in approach_track if t[0] >= 500]
    if len(appr) >= 2:
        deltas = [abs(appr[k][2] - appr[k-1][2]) for k in range(1, len(appr))]
        first = appr[0][0]
        print(f"first confirmed frame >=500: {first} "
              f"(vs full-frame NN's first confirm ~540; UNGROUNDED below 540)")
        print(f"confirmed approach frames: {[t[0] for t in appr]}")
        print(f"bearing track (deg): {['%+.1f' % t[2] for t in appr]}")
        print(f"frame-to-frame |dbearing|: median {np.median(deltas):.1f}deg "
              f"max {np.max(deltas):.1f}deg (small+monotonic is CONSISTENT WITH one "
              f"real target, not proof -- no GT below frame 540)")
        print(f"  CAVEAT: this is NOT a validated acquisition-range extension. It "
              f"depends on the untuned COCO classifier's context-dependent "
              f"drone->\"airplane\" response (honest finding above: a tight upsampled "
              f"crop can instead read laptop/clock), so early confirms here are "
              f"contingent on a classifier behavior we have not characterized, not a "
              f"validated capability. A real P_detect(R) curve needs "
              f"models/fpv_target_markerless with gt logging in Gazebo -- this "
              f"frame-replay eval cannot produce that number.")

    # ---- BODY-NOT-TAG proof: inpaint the tag out, re-run the pipeline ----
    print()
    print("--- BODY-NOT-TAG proof (inpaint tag region -> reconstruct body) ---")
    n_orig = n_inp = 0
    for name in sorted(tag_gt):
        p = os.path.join(FRAMES_DIR, name)
        if not os.path.exists(p):
            continue
        img = cv2.imread(p)
        x0, y0, x1, y1, tcx, tcy = tag_gt[name]
        d0 = seeker.detect(img)
        pad = 5
        mask = np.zeros(img.shape[:2], np.uint8)
        mask[max(0, int(y0-pad)):int(y1+pad), max(0, int(x0-pad)):int(x1+pad)] = 255
        inp = cv2.inpaint(img, mask, 5, cv2.INPAINT_TELEA)
        d1 = seeker.detect(inp)
        orig_ok = d0.range_m is not None and _box_covers(d0.box_xywh, tcx, tcy)
        inp_ok = d1.range_m is not None and _box_covers(d1.box_xywh, tcx, tcy)
        n_orig += int(orig_ok); n_inp += int(inp_ok)
        cv2.imwrite(os.path.join(out_dir, f"twostage_inpaint_{name}"), inp)
        print(f"  {name}: ORIG {'CONFIRM' if orig_ok else 'miss'} "
              f"conf={d0.decision_margin if d0.range_m else 0:.2f} | "
              f"TAG-INPAINTED {'STILL confirms body' if inp_ok else 'no confirm'} "
              f"conf={d1.decision_margin if d1.range_m else 0:.2f}")
    print(f"  SUMMARY: original {n_orig} / inpainted {n_inp} confirmed on the body "
          f"(inpaint survival => body/silhouette-driven, not fiducial decode).")
    print(f"  (Honest caveat: same as the NN lane -- inpaint reconstructs body")
    print(f"   texture; a fully tag-free number needs models/fpv_target_markerless.)")
    print(f"\nAnnotated evidence -> {out_dir}")


def _main():
    import argparse
    ap = argparse.ArgumentParser(description="Two-stage markerless seeker (offline).")
    ap.add_argument("--eval", action="store_true", help="run the offline evaluation")
    ap.add_argument("--frame", help="run the pipeline on a single frame and print")
    ap.add_argument("--n", type=int, default=48, help="eval sample size")
    ap.add_argument("--motion", action="store_true",
                    help="enable the (fragile) ego-motion frame-diff proposal channel")
    ap.add_argument("--out", default=os.path.join(HERE, "twostage_out"))
    args = ap.parse_args()

    if args.eval:
        _run_eval(args.n, args.motion, args.out)
    elif args.frame:
        fx, fy, cx, cy = load_intrinsics(os.path.join(REPO, "camera_intrinsics.json"))
        seeker = TwoStageSeeker(fx, fy, cx, cy)
        img = cv2.imread(args.frame)
        det, props, surv, rej = seeker.detect(img, return_debug=True)
        print(f"proposals={len(props)} survivors={len(surv)}")
        if det.range_m is not None:
            print(f"CONFIRMED: bearing={math.degrees(det.bearing_rad):+.2f}deg "
                  f"range~{det.range_m:.2f}m conf={det.decision_margin:.2f} "
                  f"cls={det.class_name} box={tuple(round(v) for v in det.box_xywh)}")
        else:
            print("no confirmed target (filter would coast -- safe)")
    else:
        ap.print_help()


if __name__ == "__main__":
    _main()
