#!/usr/bin/env python3
"""Pi 5 AprilTag decode bench — the three bench-measurable halves of the
decode-range question (ADR-0124), run on the REAL flight computer:

  T1 CLIFF  decode rate vs tag pixel span, deployed detector config, by
            compositing the repo's own tag36h11 texture at known pixel spans.
            Measures P_min (the sim-claimed ~22 px cliff at quad_decimate=2.0)
            for the real library build on the real CPU. HONESTY: clean synthetic
            pixels = a software/decoder floor (given-perfect imagery). A blur
            arm (sigma 1 px) approximates optics; real light/motion blur still
            only comes from the tripod field day — this does NOT replace
            curve (a), it brackets its software half.
  T2 FPS    pure decode latency/fps on real captured frames (bench scene =
            edge-busy, pessimistic-ish) and on a sky-like synthetic frame
            (optimistic), at 1280x800 (OV9281) and 1920x1200 (AR0234 cost
            proxy via upscale — upscaling adds no real detail; it proxies the
            detector's pixel-count cost only), quad_decimate 2.0 and 1.0,
            deployed nthreads=4 (+ nthreads=1 reference). Pose estimation ON
            (flight pays it).
  T3 LOOP   sustained end-to-end capture->gray->decode fps from the live
            OV9281 via picamera2, qd 2.0 and 1.0 — the number the tripod money
            gate calls stream_fps/decode cadence (break-even ~24 fps at the
            predicted R_decode90, tripod_test_protocol.md §8.1).

Run ON THE PI:  .venv-pi/bin/python3 pi_decode_bench.py --out runs/bench_YYYYMMDD_decode
Skips T3 cleanly if picamera2/camera is absent (dev box).
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except ImportError:
    from pyapriltags import Detector

# Deployed flight config (flight/deploy/frame_feed.py TagFrameDetector):
# tag36h11, nthreads=4, quad_decimate=2.0, pose estimation on.
TAG_FAMILY = "tag36h11"
# fx target per hardware_order_list.md ("pick nearest fx~540@1280"); cost proxy only.
CAM_1280 = (540.0, 540.0, 640.0, 400.0)
CAM_1920 = (810.0, 810.0, 960.0, 600.0)
# models/apriltag_target/tag36h11_00000.png: black square = 0.8x of the plane
# (synth_tag_frames.py PNG_BORDER_FRAC — confirmed empirically there).
PNG_BORDER_FRAC = 0.8


def load_tag_png(repo):
    p = os.path.join(repo, "models", "apriltag_target", "tag36h11_00000.png")
    img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"tag texture not found: {p}")
    return img


def composite(tag_png, w, h, span_px, rng, blur_sigma=0.0, bg_val=170):
    """Gray frame with the tag's BLACK SQUARE spanning span_px, jittered."""
    frame = np.full((h, w), bg_val, np.uint8)
    plane = max(int(round(span_px / PNG_BORDER_FRAC)), 6)
    interp = cv2.INTER_AREA if plane < tag_png.shape[0] else cv2.INTER_LINEAR
    small = cv2.resize(tag_png, (plane, plane), interpolation=interp)
    jx = int(rng.integers(-40, 41))
    jy = int(rng.integers(-40, 41))
    x0 = (w - plane) // 2 + jx
    y0 = (h - plane) // 2 + jy
    frame[y0:y0 + plane, x0:x0 + plane] = small
    if blur_sigma > 0:
        frame = cv2.GaussianBlur(frame, (0, 0), blur_sigma)
    return frame


def t1_cliff(tag_png, out):
    res = []
    spans = list(range(8, 49, 2))
    for qd in (2.0, 1.0):
        det = Detector(families=TAG_FAMILY, nthreads=4, quad_decimate=qd)
        for blur in (0.0, 1.0):
            rng = np.random.default_rng(7)
            for span in spans:
                ok = 0
                n = 16
                for _ in range(n):
                    f = composite(tag_png, 1280, 800, span, rng, blur)
                    dets = det.detect(f)
                    ok += any(d.tag_id == 0 for d in dets)
                res.append({"qd": qd, "blur_sigma": blur, "span_px": span,
                            "rate": ok / n, "n": n})
        del det
    out["t1_cliff"] = res
    # cliff90 = smallest span from which the rate stays >= 0.9 upward
    for qd in (2.0, 1.0):
        for blur in (0.0, 1.0):
            arm = [r for r in res if r["qd"] == qd and r["blur_sigma"] == blur]
            cliff = None
            for i, r in enumerate(arm):
                if all(a["rate"] >= 0.9 for a in arm[i:]):
                    cliff = r["span_px"]
                    break
            key = f"cliff90_px_qd{qd:g}_blur{blur:g}"
            out[key] = cliff
            print(f"T1  qd={qd:g} blur={blur:g}: cliff90 = {cliff} px")


def _time_decode(det, frames, cam, reps=30):
    # warmup
    det.detect(frames[0], estimate_tag_pose=True, camera_params=cam, tag_size=0.35)
    ts = []
    for i in range(reps):
        f = frames[i % len(frames)]
        t0 = time.perf_counter()
        det.detect(f, estimate_tag_pose=True, camera_params=cam, tag_size=0.35)
        ts.append(time.perf_counter() - t0)
    ts = np.array(ts)
    return {"ms_p50": float(np.median(ts) * 1e3),
            "ms_p90": float(np.percentile(ts, 90) * 1e3),
            "fps_at_p50": float(1.0 / np.median(ts)), "reps": reps}


def t2_fps(tag_png, real_frames, out):
    rng = np.random.default_rng(3)
    sky_1280 = composite(tag_png, 1280, 800, 100, rng)
    scenes_1280 = {"sky_synth": [sky_1280]}
    if real_frames:
        scenes_1280["bench_real"] = real_frames
    rows = []
    for scene, frames in scenes_1280.items():
        frames_1920 = [cv2.resize(f, (1920, 1200)) for f in frames]
        for (label, fs, cam) in (("1280x800", frames, CAM_1280),
                                 ("1920x1200_proxy", frames_1920, CAM_1920)):
            for qd in (2.0, 1.0):
                det = Detector(families=TAG_FAMILY, nthreads=4, quad_decimate=qd)
                r = _time_decode(det, fs, cam)
                r.update(scene=scene, res=label, qd=qd, nthreads=4)
                rows.append(r)
                print(f"T2  {scene:10s} {label:15s} qd={qd:g} nt=4: "
                      f"{r['ms_p50']:6.1f} ms p50 -> {r['fps_at_p50']:5.1f} fps")
                del det
    # nthreads=1 reference at the deployed res/qd
    det = Detector(families=TAG_FAMILY, nthreads=1, quad_decimate=2.0)
    fs = scenes_1280.get("bench_real", scenes_1280["sky_synth"])
    r = _time_decode(det, fs, CAM_1280)
    r.update(scene="bench_real" if real_frames else "sky_synth",
             res="1280x800", qd=2.0, nthreads=1)
    rows.append(r)
    print(f"T2  nthreads=1 ref 1280x800 qd=2: {r['ms_p50']:.1f} ms p50")
    out["t2_fps"] = rows


def grab_camera(n=20, w=1280, h=800):
    try:
        from picamera2 import Picamera2
    except ImportError:
        return None, None, "picamera2 not installed (dev box) — T3 skipped"
    try:
        picam2 = Picamera2()
        cfg = picam2.create_video_configuration(main={"size": (w, h)})
        picam2.configure(cfg)
        picam2.start()
        time.sleep(0.8)  # AE settle
        frames = []
        for _ in range(n):
            a = picam2.capture_array("main")
            g = a if a.ndim == 2 else cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
            frames.append(np.ascontiguousarray(g[:h, :w]))
        return picam2, frames, None
    except Exception as e:
        return None, None, f"camera open failed: {e}"


def t3_loop(picam2, out, seconds=8.0):
    rows = []
    for qd in (2.0, 1.0):
        det = Detector(families=TAG_FAMILY, nthreads=4, quad_decimate=qd)
        lat = []
        t_end = time.monotonic() + seconds
        nframes = 0
        while time.monotonic() < t_end:
            t0 = time.perf_counter()
            a = picam2.capture_array("main")
            g = a if a.ndim == 2 else cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
            det.detect(g, estimate_tag_pose=True, camera_params=CAM_1280,
                       tag_size=0.35)
            lat.append(time.perf_counter() - t0)
            nframes += 1
        lat = np.array(lat)
        r = {"qd": qd, "sustained_fps": float(nframes / seconds),
             "ms_p50": float(np.median(lat) * 1e3),
             "ms_p90": float(np.percentile(lat, 90) * 1e3),
             "frames": nframes, "seconds": seconds}
        rows.append(r)
        print(f"T3  LOOP qd={qd:g}: {r['sustained_fps']:.1f} fps sustained "
              f"({r['ms_p50']:.1f} ms p50, {r['ms_p90']:.1f} ms p90) "
              f"vs 24 fps break-even")
        del det
    out["t3_loop"] = rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--repo", default=os.path.expanduser("~/interceptor-sim"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    out = {"date": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "host": os.uname().nodename,
           "detector_lib": Detector.__module__,
           "deployed_config": "tag36h11 nthreads=4 qd=2.0 pose=on (frame_feed.py)"}
    tag_png = load_tag_png(a.repo)

    picam2, real_frames, skip = grab_camera()
    out["camera"] = skip or "OV9281 via picamera2, 20 bench frames captured"
    if skip:
        print("T3  " + skip)

    t1_cliff(tag_png, out)
    t2_fps(tag_png, real_frames, out)
    if picam2 is not None:
        t3_loop(picam2, out)
        picam2.stop()
        if real_frames:
            cv2.imwrite(os.path.join(a.out, "bench_frame_example.png"),
                        real_frames[0])

    with open(os.path.join(a.out, "decode_bench.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(f"\nwrote {a.out}/decode_bench.json")


if __name__ == "__main__":
    main()
