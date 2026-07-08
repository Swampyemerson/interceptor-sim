#!/usr/bin/env python3
"""Passive in-flight frame recorder for the markerless seeker v2 dataset
(ADR-0040 addendum).

WHY: ADR-0040's addendum diagnosed the v1 fine-tune's regressions as terminal
DROPOUTS plus an in-gate range bias, both traced to a POSITIVES-ONLY training
set. render_sim_dataset.py's grid renders are static GROUND shots of the
tag-less target -- they never show the INTERCEPTOR'S OWN body-fixed prop arms
creeping into the camera's field of view, because there is no interceptor in
that render loop. In real flight those prop arms sit at the FOV periphery on
every frame, and a positives-only model has never been taught they are NOT the
target -- so it false-locks on them (the exact confident-wrong-bearing hazard
documented in finetuned_seeker.py's self-mask comment and
docs/seeker_prototype_results.md s3.4). The fix is HARD NEGATIVE frames drawn
from real flight, not more positives. This script harvests them (and, as a
side effect, denser flight-realistic positives) by riding along on a live
intercept, completely passively.

HONESTY: ground truth (`gt_*`) is used ONLY to auto-label TRAINING data
offline, exactly the same scoring-only boundary as render_sim_dataset.py and
CLAUDE.md's gt_* rule -- never fed to any guidance or detection path at
inference. This script issues ZERO gz service calls and subscribes only to
the camera Image topic and the world Pose_V topic; it cannot steer, teleport,
or otherwise perturb the flight it is riding along on. (It is also, as it
happens, a subscribing process -- so per the documented gz-transport13 quirk
it could never receive a service RESPONSE even if it tried one; it simply
never tries.)

WHAT: run this ALONGSIDE an already-running intercept flight (e.g.
m4_intercept.py) on the markerless world -- PX4 SITL + gz_x500_mono_cam must
already be up and the flight already streaming frames. It samples every
--every-th received camera frame. At each SAMPLED frame it snapshots the
latest ground-truth camera-optical target vector *immediately*
(PoseTracker.ground_truth_rel_optical, the exact transform chain M2
validated) -- doing this right away, in the same callback that received the
frame, minimizes moving-target label skew: at the M4 arc's ~9 m/s target
speed a single 30 Hz frame of pose lag is a few centimeters of true motion,
which projects to on the order of tens of pixels at typical engagement
ranges -- small next to the projected box extent, and an acceptable amount of
label noise (the extent itself gives slack).

Each sampled frame is classified into exactly one of:
  POSITIVE  -- target in frame, both projected box dims >= 4 px:
               save PNG + a one-line YOLO label ("0 cx cy w h", normalized).
  NEGATIVE  -- target BEHIND the camera OR its projected box is FULLY outside
               the frame: save PNG + an EMPTY label file. These are exactly
               the frames the addendum says v1 was missing -- they show the
               interceptor's own prop-arm periphery with no real target in
               view, teaching the model what "not the target" looks like.
  DROP      -- target in frame but its projected box is smaller than 4 px
               (too small/ambiguous to trust as background OR as a usable
               positive box): never saved, counted separately.
  DROP (no gt) -- ground truth unavailable this tick (pose topic missing an
               entity this frame): never saved, counted separately.

The three-way split (behind/off-frame vs. too-small vs. positive) is
reimplemented LOCALLY in `classify()` below rather than reusing
render_sim_dataset.project_box, because that function deliberately collapses
"behind camera", "fully off-frame", and "too small" into a single `None` --
exactly the distinction this script exists to make. render_sim_dataset.py is
not modified.

OUTPUT LAYOUT: --out/images/*.png + --out/labels/*.txt, FLAT (no train/val
split here -- that happens later, once, at merge_dataset_v2.py, over the
union of every capture run plus the v1 renders). Filenames encode the run and
the ground-truth range (useful for later range-calibration work):

    f"{run_tag}_{seq:06d}_r{gt_range:05.1f}_{'pos' if positive else 'neg'}"

--run-tag (default "flt") disambiguates frames across separate flights so
repeated capture runs into the same --out never collide.

SHUTDOWN: stops on --max-frames SAVED frames (a plain frame-COUNT cap, per
project convention: schedule/measure in sim time or frame counts, never
wall-clock -- CLAUDE.md), or on SIGINT/SIGTERM from whatever orchestrates the
flight. Either way, a manifest.json (frame/label counts) is written from a
try/finally block so it survives being killed mid-run.

RUN (alongside an already-running markerless flight -- see
probe_markerless_range.py's docstring for how to boot that world):
    INTERCEPTOR_WORLD_NAME=markerless INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \\
      .venv-seeker/bin/python scripts/seeker/capture_flight_frames.py \\
      --out scripts/seeker/data/flight_capture --run-tag flt001 --every 3

SELF-CHECK (no sim needed): `--self-test` runs the local classify() against 4
synthetic rel_optical vectors (in-frame / behind-camera / off-frame / tiny)
and prints PASS/FAIL per case, then exits -- this is what CI / a human runs
to sanity-check the classification logic without a live Gazebo instance.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
REPO = os.path.dirname(SCRIPTS)
os.environ.setdefault("INTERCEPTOR_WORLD_NAME", "markerless")
os.environ.setdefault("INTERCEPTOR_TARGET_MODEL", "fpv_target_markerless")
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, HERE)

import cv2  # noqa: E402

from m1_capture import image_msg_to_bgr  # noqa: E402 (reuse the decode, don't duplicate)
from m2_detect import (  # noqa: E402
    IMAGE_TOPIC, POSE_TOPIC, WORLD_NAME, TAG_MODEL_NAME, PoseTracker,
)
from gz.transport13 import Node  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402

# Same fixed intrinsics as render_sim_dataset.py (mono_cam SDF, 1280x960).
FX = FY = 539.936
CX, CY = 640.0, 480.0

MIN_BOX_PX = 4.0
DEFAULT_EXTENT_M = 0.9  # same silhouette extent default as render_sim_dataset.py
STARTUP_TIMEOUT_S = 20.0


def classify(rel_optical, extent_m: float, W: int, H: int, min_px: float = MIN_BOX_PX):
    """Classify one sampled frame's ground-truth target vector.

    Returns (label, box_or_None) where label is one of "positive",
    "negative", "drop" and box (only for "positive") is the clipped YOLO box
    (cx, cy, w, h), all normalized to [0, 1].

    This is the same projection math as render_sim_dataset.project_box, but
    it does NOT collapse "behind camera" / "fully off-frame" / "too small"
    into one None -- this script needs to tell a hard NEGATIVE (behind /
    off-frame, still a real frame worth teaching as background) from a DROP
    (in frame but too small to trust either way) apart. render_sim_dataset.py
    is not modified; this is a deliberate, documented reimplementation.
    """
    x, y, z = float(rel_optical[0]), float(rel_optical[1]), float(rel_optical[2])
    if z <= 0.05:
        return "negative", None  # behind the camera

    u = CX + FX * (x / z)
    v = CY + FY * (y / z)
    half_w = FX * (extent_m / 2.0) / z
    half_h = FY * (extent_m / 2.0) / z
    x0, x1 = u - half_w, u + half_w
    y0, y1 = v - half_h, v + half_h

    if x1 < 0 or x0 > W or y1 < 0 or y0 > H:
        return "negative", None  # fully outside the frame

    # clip to frame for the size test and the final box, same as project_box
    cx0, cx1 = max(0.0, x0), min(float(W), x1)
    cy0, cy1 = max(0.0, y0), min(float(H), y1)
    bw, bh = cx1 - cx0, cy1 - cy0
    if bw < min_px or bh < min_px:
        return "drop", None  # in frame, but too small/ambiguous

    cx, cy = (cx0 + cx1) / 2.0, (cy0 + cy1) / 2.0
    return "positive", (cx / W, cy / H, bw / W, bh / H)


def _run_self_test() -> int:
    """4 synthetic rel_optical cases exercising each classify() branch."""
    W, H = 1280, 960
    cases = [
        ("in-frame near boresight", (0.0, 0.0, 5.0), "positive"),
        ("behind camera", (0.0, 0.0, -3.0), "negative"),
        ("far off to the side (off-frame)", (50.0, 0.0, 5.0), "negative"),
        ("far away (tiny box)", (0.0, 0.0, 500.0), "drop"),
    ]
    all_ok = True
    for name, rel, expect in cases:
        label, box = classify(np.array(rel), DEFAULT_EXTENT_M, W, H)
        ok = label == expect
        all_ok = all_ok and ok
        print(f"[self-test] {name}: rel_optical={rel} -> {label!r} "
              f"(expected {expect!r}) box={box} {'PASS' if ok else 'FAIL'}")
    print(f"[self-test] {'ALL PASS' if all_ok else 'SOME FAILED'}")
    return 0 if all_ok else 1


class Capture:
    """Owns the image callback: samples every Nth frame, classifies it
    against the latest ground truth, and saves + counts the result."""

    def __init__(self, tracker: PoseTracker, img_dir: str, lbl_dir: str,
                 run_tag: str, every: int, extent_m: float, max_frames: int):
        self.tracker = tracker
        self.img_dir = img_dir
        self.lbl_dir = lbl_dir
        self.run_tag = run_tag
        self.every = max(1, every)
        self.extent_m = extent_m
        self.max_frames = max_frames

        self.frames_seen = 0
        self.seq = 0
        self.saved_pos = 0
        self.saved_neg = 0
        self.dropped_small = 0
        self.dropped_nogt = 0
        self.done = threading.Event()

    @property
    def total_saved(self) -> int:
        return self.saved_pos + self.saved_neg

    def on_image(self, msg: Image) -> None:
        try:
            self.frames_seen += 1
            if self.done.is_set():
                return
            if self.frames_seen % self.every != 0:
                return

            # Snapshot gt IMMEDIATELY -- see module docstring re: label skew.
            # tracker.latest reads are safe without a lock (single attribute
            # assignment is atomic under the GIL -- same reasoning as
            # m2_detect.PoseTracker's own docstring).
            rel, ok, _reason = self.tracker.ground_truth_rel_optical()
            if not ok:
                self.dropped_nogt += 1
                return

            label, box = classify(rel, self.extent_m, msg.width, msg.height)
            if label == "drop":
                self.dropped_small += 1
                return

            try:
                bgr = image_msg_to_bgr(msg)
            except ValueError as exc:
                print(f"[capture] frame decode error: {exc}")
                return

            gt_range = float(np.linalg.norm(rel))
            self.seq += 1
            positive = label == "positive"
            stem = (f"{self.run_tag}_{self.seq:06d}_r{gt_range:05.1f}_"
                    f"{'pos' if positive else 'neg'}")
            cv2.imwrite(os.path.join(self.img_dir, stem + ".png"), bgr)
            with open(os.path.join(self.lbl_dir, stem + ".txt"), "w") as fh:
                if positive:
                    fh.write(f"0 {box[0]:.6f} {box[1]:.6f} {box[2]:.6f} {box[3]:.6f}\n")
                # else: leave the file EMPTY -- that IS the negative label.

            if positive:
                self.saved_pos += 1
            else:
                self.saved_neg += 1

            if self.total_saved >= self.max_frames:
                self.done.set()
        except Exception as exc:  # pragma: no cover -- never kill the callback thread
            print(f"[capture] on_image error (frame skipped): {exc}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(HERE, "data", "flight_capture"))
    ap.add_argument("--run-tag", default="flt",
                    help="prefix distinguishing this flight's frames from others "
                         "captured into the same --out (default 'flt')")
    ap.add_argument("--every", type=int, default=3,
                    help="sample every Nth received frame (default 3)")
    ap.add_argument("--extent-m", type=float, default=DEFAULT_EXTENT_M,
                    help="target silhouette extent (m) for the bbox, same default "
                         "as render_sim_dataset.py (default %(default)s)")
    ap.add_argument("--max-frames", type=int, default=2000,
                    help="stop after this many SAVED frames, pos+neg (default 2000)")
    ap.add_argument("--self-test", action="store_true",
                    help="run classify() against 4 synthetic cases and exit (no sim needed)")
    args = ap.parse_args()

    if args.self_test:
        return _run_self_test()

    img_dir = os.path.join(args.out, "images")
    lbl_dir = os.path.join(args.out, "labels")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)

    print(f"[capture] world={WORLD_NAME} target={TAG_MODEL_NAME}")
    print(f"[capture] camera={IMAGE_TOPIC}")
    print("[capture] PASSIVE listener: subscribe-only, zero gz service calls, "
          "never perturbs the flight.")

    node = Node()
    tracker = PoseTracker()
    if not node.subscribe(Pose_V, POSE_TOPIC, tracker.on_pose_v):
        print(f"[capture] FAILED: subscribe {POSE_TOPIC}")
        return 2

    cap = Capture(tracker, img_dir, lbl_dir, args.run_tag, args.every,
                  args.extent_m, args.max_frames)
    if not node.subscribe(Image, IMAGE_TOPIC, cap.on_image):
        print(f"[capture] FAILED: subscribe {IMAGE_TOPIC}")
        node.unsubscribe(POSE_TOPIC)
        return 2

    stop = {"flag": False}

    def _handle_signal(signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    timed_out = False
    try:
        print(f"[capture] waiting up to {STARTUP_TIMEOUT_S:.0f}s for the first frame...")
        t0 = time.time()
        while cap.frames_seen == 0 and not stop["flag"]:
            if time.time() - t0 > STARTUP_TIMEOUT_S:
                timed_out = True
                break
            time.sleep(0.1)

        if timed_out:
            print(f"[capture] FAILED: no frames within {STARTUP_TIMEOUT_S:.0f}s "
                  "(is the flight sim up and streaming?)")
        elif not stop["flag"]:
            print(f"[capture] streaming: every={args.every} "
                  f"max_frames={args.max_frames} run_tag={args.run_tag} out={args.out}")
            while not stop["flag"] and not cap.done.is_set():
                time.sleep(0.1)
    finally:
        node.unsubscribe(IMAGE_TOPIC)
        node.unsubscribe(POSE_TOPIC)
        manifest = {
            "run_tag": args.run_tag,
            "every": args.every,
            "extent_m": args.extent_m,
            "max_frames": args.max_frames,
            "frames_seen": cap.frames_seen,
            "saved_pos": cap.saved_pos,
            "saved_neg": cap.saved_neg,
            "dropped_small": cap.dropped_small,
            "dropped_nogt": cap.dropped_nogt,
            "timed_out": timed_out,
        }
        with open(os.path.join(args.out, "manifest.json"), "w") as fh:
            json.dump(manifest, fh, indent=2)
        print(f"[capture] done: frames_seen={cap.frames_seen} "
              f"saved_pos={cap.saved_pos} saved_neg={cap.saved_neg} "
              f"dropped_small={cap.dropped_small} dropped_nogt={cap.dropped_nogt}")
        print(f"[capture] manifest: {os.path.join(args.out, 'manifest.json')}")

    return 0 if cap.total_saved > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
