#!/usr/bin/env python3
"""Full-frame fine-tuned single-class drone seeker (ADR-0033 item 2, Option B).

The two-stage seeker (classical proposal -> crop -> COCO NN verify) is limited by
COCO having no drone class -> sparse terminal-only detection (ADR-0038). Option B
fine-tunes a single-class YOLO nano on IN-DOMAIN Gazebo renders of the tag-less
body (render_sim_dataset.py, gt-projected labels, training-time only). Because the
fine-tune is reliable on the sim body, we run it DIRECTLY on the full frame (no
classical proposal) -- a simpler, denser detector than the two-stage.

Interface: `.detect(frame_bgr, t_mono) -> SeekerDetection` -- drop-in for
TwoStageSeeker inside markerless_loop.py (selected by the MARKERLESS_NN_WEIGHTS
env var). Same bearing/range geometry as nn_seeker.SeekerDetection.

HONESTY: reads only camera pixels + fixed intrinsics; never gt_*. The fine-tune's
labels came from gt OFFLINE at training time (analogous to a human labeler) -- the
live detector at inference is gt-free, so the no-cheat audit is unaffected.

DEPS: onnxruntime + opencv (in .venv-seeker / the combined flight venv). No torch,
no ultralytics at inference -- the ONNX runs standalone.
"""
from __future__ import annotations

import json
import math
import os
from typing import Optional

import numpy as np
import cv2

from nn_seeker import SeekerDetection  # reuse the exact dataclass / interface

# Target body span for coarse known-size range (models/fpv_target_markerless is
# ~1.0 m tip-to-tip; nn_seeker uses the same 1.0 m, ~15-30% range sigma disclosed).
TARGET_SPAN_M = 1.0

# ---------------------------------------------------------------- input modality
# GRAY-NATIVE MODELS (docs/nn_tier/PLAN.md, ADR-0078). The real-data nn_tier track
# trains and evaluates in GRAYSCALE because the ordered seeker camera is the MONO
# global-shutter OV9281; the older sim-trained weights (drone_finetuned_quad_v2 and
# friends) were fine-tuned on COLOR Gazebo renders. Feeding a model the modality it
# did NOT train on is a measured, one-directional harm -- the color-trained v2 fed
# gray false-fires on 88.5% of drone-free frames (logs/nn_tier/eval_s-mono_summary_cmp1.csv)
# -- so the conversion must be MODEL-APPROPRIATE, never a global switch.
#
# On the real OV9281 the frames are already monochrome (a 1-channel PNG loads through
# cv2.imread as three IDENTICAL channels), and BGR2GRAY of B==G==R is exactly the
# identity (0.114+0.587+0.299 = 1.0), so the gray step is a NO-OP on real capture and
# only bites on sim/bench COLOR replay -- which is precisely where it is needed.
#
# Resolution order (first hit wins):
#   1. explicit `gray_input=True/False` argument,
#   2. env MARKERLESS_GRAY_INPUT (1/0/true/false),
#   3. a `<weights>.modality.json` sidecar: {"input": "gray"|"color"}  <- for future
#      tripod-day models, so a new fine-tune declares its own modality,
#   4. the built-in stem table below,
#   5. default COLOR (byte-identical to the pre-2026-07-24 behaviour).
GRAY_NATIVE_STEMS = {"n-mono", "n-mono-aug", "s-mono"}
_TRUEISH = ("1", "true", "yes", "on", "t")
_FALSEISH = ("0", "false", "no", "off", "f")


def resolve_input_modality(weights, gray_input="auto"):
    """-> (gray: bool, source: str). See the resolution order above."""
    if gray_input is not True and gray_input is not False and gray_input != "auto":
        raise ValueError(f"gray_input must be True/False/'auto', got {gray_input!r}")
    if gray_input is True or gray_input is False:
        return gray_input, "explicit argument"
    env = os.environ.get("MARKERLESS_GRAY_INPUT")
    if env is not None:
        e = env.strip().lower()
        if e in _TRUEISH:
            return True, f"env MARKERLESS_GRAY_INPUT={env}"
        if e in _FALSEISH:
            return False, f"env MARKERLESS_GRAY_INPUT={env}"
    sidecar = str(weights) + ".modality.json"
    if os.path.exists(sidecar):
        try:
            with open(sidecar) as fh:
                mode = str(json.load(fh)["input"]).strip().lower()
            if mode in ("gray", "grey", "grayscale", "mono"):
                return True, f"sidecar {os.path.basename(sidecar)}"
            if mode in ("color", "colour", "rgb", "bgr"):
                return False, f"sidecar {os.path.basename(sidecar)}"
        except Exception:
            pass
    stem = os.path.splitext(os.path.basename(str(weights)))[0]
    if stem in GRAY_NATIVE_STEMS:
        return True, f"gray-native model table ({stem})"
    return False, "default (color-trained weights)"


def to_gray3(frame_bgr):
    """BGR (or already-replicated-gray) -> 3-channel grayscale, the EXACT
    convention every nn_tier eval used (`nn_tier/baseline_eval.to_gray3`), so a
    live detection matches the offline held-out numbers. Identity on a frame whose
    channels are already equal (real mono capture)."""
    g = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    return cv2.merge([g, g, g])


def _letterbox(img, new_size):
    """Resize to a square new_size keeping aspect (gray pad). Returns
    (padded_img, scale, (pad_w, pad_h))."""
    h, w = img.shape[:2]
    r = min(new_size / h, new_size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = (new_size - nw) / 2, (new_size - nh) / 2
    top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
    left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
    out = cv2.copyMakeBorder(resized, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out, r, (left, top)


class FinetunedNNSeeker:
    def __init__(self, fx, fy, cx, cy, weights: str,
                 conf_thres: float = 0.25, imgsz: int = 640,
                 target_span_m: float = TARGET_SPAN_M,
                 max_bearing_deg: float = 30.0, edge_margin_px: float = 40.0,
                 gray_input="auto"):
        import onnxruntime as ort
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.conf = conf_thres
        self.imgsz = imgsz
        # SELF-MASK (ADR-0038 forensic fix): a positives-only fine-tune false-locks
        # on the interceptor's OWN body-fixed prop arms at the FOV periphery
        # (observed at bearing -44/+33 deg, constant ~1.5 m range while gt=29 m --
        # the confident-wrong-bearing hazard, seeker_prototype_results.md s3.4). The
        # real target is compact and near boresight through a cue-guided approach, so
        # reject any detection beyond max_bearing_deg OR touching the L/R frame edge
        # (where the props intrude) -- the two-stage's interior/self-mask gate, which
        # this full-frame path had omitted.
        self.max_bearing_rad = max_bearing_deg * math.pi / 180.0
        self.edge_margin_px = edge_margin_px
        so = ort.SessionOptions()
        so.intra_op_num_threads = 2
        self.sess = ort.InferenceSession(weights, so,
                                         providers=["CPUExecutionProvider"])
        self.iname = self.sess.get_inputs()[0].name
        # infer the model's expected input size from the ONNX (fall back to imgsz)
        shp = self.sess.get_inputs()[0].shape
        try:
            self.imgsz = int(shp[-1]) if isinstance(shp[-1], int) else imgsz
        except Exception:
            pass
        # Input modality (gray-native nn_tier models vs color-trained sim models).
        self.gray_input, gray_source = resolve_input_modality(weights, gray_input)
        print(f"[finetuned] loaded {os.path.basename(weights)} imgsz={self.imgsz} "
              f"conf={self.conf} input={'gray' if self.gray_input else 'color'} "
              f"({gray_source})")

        # RANGE CALIBRATION (ADR-0040 addendum): the known-size range formula
        # (range = FX * span / box_width_px) underestimates badly (meas/gt
        # median ~0.06 in flight) because this model's boxes run large vs the
        # assumed 1.0 m span -- calibrate_range.py fits an effective span from
        # an IoU-gated positives-only render set. This is the SECONDARY lever
        # (a range fix can't help when the detection itself is wrong; the
        # PRIMARY v2 lever is hard-negative training). Resolution order:
        # 1) MARKERLESS_SPAN_M env override, 2) a calib sidecar next to
        # `weights`, 3) the target_span_m default (byte-identical to before
        # this patch when neither exists -- e.g. the gated v1 arm's weights).
        self.span = target_span_m
        span_source = "default"
        env_span = os.environ.get("MARKERLESS_SPAN_M")
        if env_span:
            try:
                self.span = float(env_span)
                span_source = f"env MARKERLESS_SPAN_M={env_span}"
            except ValueError:
                pass
        else:
            calib_path = str(weights) + ".calib.json"
            if os.path.exists(calib_path):
                try:
                    with open(calib_path) as fh:
                        calib = json.load(fh)
                    self.span = float(calib["span_m_eff"])
                    span_source = f"calib sidecar {os.path.basename(calib_path)}"
                except Exception:
                    pass
        print(f"[finetuned] span={self.span:.3f} m ({span_source})")

    def _none(self, t_mono):
        return SeekerDetection(t_mono, None, None, None, None, None, 0)

    def box_to_detection(self, box_xywh, t_mono, score, n_hits, class_name="track"):
        """Convert a box (x0, y0, w, h top-left, image pixels) to a
        SeekerDetection using the EXACT SAME geometry detect() uses (width-based
        known-size range, box-center bearing). This is the seam the
        detect-then-track tracker (detect_track.py) publishes through, so a
        tracker-sourced Measurement is identical in construction to an NN one --
        only the box comes from CSRT instead of the net. detect() itself is left
        untouched (the plain `--seeker markerless` path stays byte-identical).
        `box_xywh` MUST already be in FULL-FRAME pixel coords -- the AUTO-CROP
        wrapper (auto_crop_seeker.AutoCropSeeker, ADR-0074) maps its crop-local
        box through crop_geom.box_crop_to_full BEFORE calling this, so the
        bearing/range math below (which uses self.cx/self.cy, the FULL-FRAME
        principal point) is always correct regardless of which caller reached
        it. `class_name` defaults to "track" (unchanged default, so the
        existing detect_track.py caller -- which never passes it -- is
        byte-identical); AutoCropSeeker passes "drone_crop" so the m4
        meas_source CSV column can tell an auto-crop-sourced NN hit apart from
        a CSRT tracker box."""
        x0, y0, bw, bh = box_xywh
        u = x0 + bw / 2.0
        v = y0 + bh / 2.0
        bearing_h = math.atan2((u - self.cx) / self.fx, 1.0)   # + = right
        bearing_v = math.atan2((v - self.cy) / self.fy, 1.0)   # + = down
        range_m = self.fx * self.span / max(bw, 1.0)
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        xyz = range_m * ray
        return SeekerDetection(
            t_mono=t_mono, range_m=range_m, bearing_rad=bearing_h,
            bearing_vert_rad=bearing_v, meas_xyz=xyz,
            decision_margin=score, n_detections=n_hits,
            box_xywh=(x0, y0, bw, bh), class_name=class_name)

    def _infer_boxes(self, frame_bgr):
        """ONE forward pass on `frame_bgr` (whatever its size -- the full
        1280x960 frame, or a 640x640 AUTO-CROP window, ADR-0074) -> a list of
        (score, u, v, bw, bh) rows above `self.conf`, in `frame_bgr`'s OWN
        pixel coordinate space (letterbox undone, but NO self-mask/bearing/
        edge filtering applied yet -- that filtering is meaningful only in the
        FULL frame's own coordinate space, see detect() vs. raw_boxes_crop()
        below). Split out of detect() so both callers share ONE inference
        path; detect()'s own control flow / outputs are unchanged by this
        split (pure extraction, verified equivalent).

        MODALITY: if these weights are gray-native (see resolve_input_modality)
        the frame is converted to 3-channel grayscale FIRST, so the live input
        matches the modality the model was trained and held-out-scored in. When
        gray_input is False this branch is skipped entirely -- byte-identical to
        the pre-2026-07-24 path for every color-trained sim model."""
        if self.gray_input:
            frame_bgr = to_gray3(frame_bgr)
        img, r, (pad_l, pad_t) = _letterbox(frame_bgr, self.imgsz)
        blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = self.sess.run(None, {self.iname: np.ascontiguousarray(blob)})[0]
        preds = out[0]                      # (5, N) or (N, 5)
        if preds.shape[0] < preds.shape[1]:  # (5, N) -> (N, 5)
            preds = preds.T
        # columns: cx, cy, w, h, class0_score  (single class)
        rows = []
        for row in preds:
            cx_, cy_, w_, h_, score = float(row[0]), float(row[1]), \
                float(row[2]), float(row[3]), float(row[4])
            if score < self.conf:
                continue
            # undo letterbox back to the input frame's own pixels
            bw = w_ / r
            bh = h_ / r
            u = (cx_ - pad_l) / r
            v = (cy_ - pad_t) / r
            rows.append((score, u, v, bw, bh))
        return rows

    def raw_boxes_crop(self, crop_bgr):
        """AUTO-CROP inference (ADR-0074): run `_infer_boxes` on a native-res
        CROP and return the single best (score, box_xywh) in the CROP's OWN
        (crop-local) pixel coords, top-left (x, y, w, h) -- or (0.0, None) if
        nothing cleared `self.conf`.

        Deliberately DOES NOT apply detect()'s own-airframe self-mask (the
        bearing gate + edge-touch reject): that mask exists to reject the
        interceptor's OWN rotor arms, which intrude from the FULL 1280x960
        frame's periphery (self.cx/self.cy/self.max_bearing_rad/
        edge_margin_px are all full-frame quantities) -- applying it against
        a small local crop's own coordinate frame would reject valid
        detections almost everywhere in the crop (e.g. self.cx=640 is off the
        RIGHT EDGE of a 640-wide crop) rather than mask anything real. The
        caller (auto_crop_seeker.AutoCropSeeker) maps the returned box back to
        FULL-FRAME pixels via crop_geom.box_crop_to_full and then calls
        box_to_detection() on that full-frame box for the actual bearing/
        range math -- the self-mask isn't needed here because a crop is only
        ever taken around an ALREADY-trusted recent detection, not blindly
        over the whole frame."""
        best_score, best_box = 0.0, None
        for score, u, v, bw, bh in self._infer_boxes(crop_bgr):
            if score > best_score:
                best_score, best_box = score, (u - bw / 2.0, v - bh / 2.0, bw, bh)
        return best_score, best_box

    def detect(self, frame_bgr, t_mono: Optional[float] = None) -> SeekerDetection:
        H, W = frame_bgr.shape[:2]
        # columns: cx, cy, w, h, class0_score  (single class)
        best_score, best_box = 0.0, None
        n_hits = 0
        for score, u, v, bw, bh in self._infer_boxes(frame_bgr):
            # SELF-MASK: reject own-airframe periphery locks (bearing gate + edge touch)
            bearing = math.atan2((u - self.cx) / self.fx, 1.0)
            if abs(bearing) > self.max_bearing_rad:
                continue
            if (u - bw / 2) < self.edge_margin_px or (u + bw / 2) > (W - self.edge_margin_px):
                continue
            n_hits += 1
            if score > best_score:
                best_score, best_box = score, (u, v, bw, bh)
        if best_box is None:
            return self._none(t_mono)
        u, v, bw, bh = best_box
        bearing_h = math.atan2((u - self.cx) / self.fx, 1.0)   # + = right
        bearing_v = math.atan2((v - self.cy) / self.fy, 1.0)   # + = down
        range_m = self.fx * self.span / max(bw, 1.0)
        ray = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        xyz = range_m * ray
        return SeekerDetection(
            t_mono=t_mono, range_m=range_m, bearing_rad=bearing_h,
            bearing_vert_rad=bearing_v, meas_xyz=xyz,
            decision_margin=best_score, n_detections=n_hits,
            box_xywh=(u - bw / 2, v - bh / 2, bw, bh), class_name="drone")


def _self_test_modality() -> int:
    """Offline check of the input-modality plumbing (NO onnxruntime, NO weights
    file needed): resolution order + the two properties the design leans on --
    gray3 is the IDENTITY on already-mono frames, and it really does strip chroma
    from a color frame. Exits 0/1."""
    import tempfile
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}{(' -- ' + detail) if detail else ''}")
        ok = ok and bool(cond)

    print("[self-test] --- part 1: modality resolution order ---")
    os.environ.pop("MARKERLESS_GRAY_INPUT", None)
    g, src = resolve_input_modality("/w/drone_finetuned_quad_v2.onnx")
    check("color-trained sim weights default to COLOR", g is False, src)
    g, src = resolve_input_modality("/w/nn_tier/n-mono.onnx")
    check("n-mono resolves to GRAY via the stem table", g is True, src)
    g, src = resolve_input_modality("/w/nn_tier/n-color.onnx")
    check("n-color resolves to COLOR", g is False, src)
    g, src = resolve_input_modality("/w/nn_tier/n-mono.onnx", gray_input=False)
    check("explicit argument overrides the table", g is False, src)
    os.environ["MARKERLESS_GRAY_INPUT"] = "0"
    g, src = resolve_input_modality("/w/nn_tier/n-mono.onnx")
    check("env overrides the table", g is False, src)
    g, src = resolve_input_modality("/w/nn_tier/n-mono.onnx", gray_input=True)
    check("explicit argument overrides the env", g is True, src)
    os.environ.pop("MARKERLESS_GRAY_INPUT", None)
    with tempfile.TemporaryDirectory() as td:
        wp = os.path.join(td, "tripod_day1.onnx")
        with open(wp + ".modality.json", "w") as fh:
            json.dump({"input": "gray"}, fh)
        g, src = resolve_input_modality(wp)
        check("a future model declares GRAY via its modality sidecar", g is True, src)
    try:
        resolve_input_modality("/w/x.onnx", gray_input="grey")
        check("bad gray_input value raises", False)
    except ValueError:
        check("bad gray_input value raises", True)

    print("[self-test] --- part 2: to_gray3 properties ---")
    rng = np.random.default_rng(0)
    color = rng.integers(0, 256, size=(48, 64, 3), dtype=np.uint8)
    g3 = to_gray3(color)
    check("gray3 keeps 3 channels + shape", g3.shape == color.shape, str(g3.shape))
    check("gray3 channels are identical",
          bool((g3[:, :, 0] == g3[:, :, 1]).all() and (g3[:, :, 1] == g3[:, :, 2]).all()))
    check("gray3 actually changes a COLOR frame", bool((g3 != color).any()))
    # The real-capture case: an OV9281 mono PNG loads as 3 identical channels.
    mono = np.repeat(rng.integers(0, 256, size=(48, 64, 1), dtype=np.uint8), 3, axis=2)
    check("gray3 is BIT-EXACT IDENTITY on an already-mono frame (real OV9281 no-op)",
          bool((to_gray3(mono) == mono).all()))
    print(f"[self-test] RESULT: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Full-frame fine-tuned seeker (offline).")
    ap.add_argument("--weights")
    ap.add_argument("--frame", help="a PNG to run once")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--gray-input", choices=("auto", "on", "off"), default="auto",
                    help="feed the net 3-channel GRAYSCALE. 'auto' (default) picks "
                         "per-model: gray for the gray-native nn_tier weights, color "
                         "for the color-trained sim weights.")
    ap.add_argument("--self-test", action="store_true",
                    help="offline modality-plumbing check (no weights/onnx); exits 0/1")
    a = ap.parse_args()
    if a.self_test:
        raise SystemExit(_self_test_modality())
    if not a.weights or not a.frame:
        ap.error("--weights and --frame are required (or use --self-test)")
    gray = {"auto": "auto", "on": True, "off": False}[a.gray_input]
    s = FinetunedNNSeeker(539.936, 539.936, 640.0, 480.0, a.weights,
                          conf_thres=a.conf, gray_input=gray)
    d = s.detect(cv2.imread(a.frame))
    if d.range_m:
        print(f"DETECT bearing={d.bearing_rad*180/math.pi:+.1f} deg "
              f"range~{d.range_m:.2f} m conf={d.decision_margin:.2f} box={d.box_xywh}")
    else:
        print("no detect")
