#!/usr/bin/env python3
"""Pi side of the screen bench: run a one-class drone HEF on every frame while bench.html plays,
then cut the frames into the page's steps using the white-to-black sync edge.
Usage: bench_pi.py <hef> <out_json>     (start it, then load bench.html within a few seconds)"""
import json, sys, time
import numpy as np, cv2
from picamera2 import Picamera2
from picamera2.devices import Hailo
hef, out = sys.argv[1], sys.argv[2]
SIZES = [150, 110, 80, 60, 45, 33, 24, 18]; N = 10; STEP = 2.5; TOTAL = 7.5 + N * len(SIZES) * STEP + 14
rec = []; cal_frames = []
with Hailo(hef) as hailo:
    h, w, _ = hailo.get_input_shape()
    assert (h, w) in ((640, 640), (800, 1280)), (h, w)
    lo_size = (640, 400) if h == 640 else (1280, 800)
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (1920, 1200), "format": "RGB888"},
                                                 lores={"size": lo_size, "format": "RGB888"}, controls={"FrameRate": 60}))
    cam.start(); time.sleep(1.0); t_start = time.time()
    while time.time() - t_start < TOTAL:
        req = cam.capture_request(); lo = req.make_array("lores"); ts = req.get_metadata()["SensorTimestamp"] / 1e9; req.release()
        if h == 640:
            frame = np.full((640, 640, 3), 114, np.uint8); frame[120:520] = lo
        else:
            frame = lo
        res = hailo.run(frame)
        dets = res[0] if isinstance(res, list) and len(res) and getattr(res[0], "ndim", 1) == 2 else res
        k = 1920 / lo_size[0]; roi = lo[int(450 / k):, :, 1]
        nat = []
        for d in dets:
            if d[4] < 0.2: continue
            if h == 640: b = [d[1] * 1920, (d[0] * 640 - 120) * 3, d[3] * 1920, (d[2] * 640 - 120) * 3]
            else: b = [d[1] * 1920, d[0] * 1200, d[3] * 1920, d[2] * 1200]
            nat.append([round(float(v), 1) for v in b] + [round(float(d[4]), 3)])
        rec.append((ts, float(roi[::4, ::4].mean()), nat))
        if len(cal_frames) < 2000: cal_frames.append((ts, cv2.resize(lo[:, :, 1], (640, 400)) if lo_size[0] != 640 else lo[:, :, 1].copy()))
        else: cal_frames.pop(0) if False else None
    cam.stop()
ts = np.array([r[0] for r in rec]); br = np.array([r[1] for r in rec])
white = np.where(br > 140)[0]
if not len(white): sys.exit("no white sync phase seen -- was the page started?")
after = np.where((br < 45) & (np.arange(len(br)) > white[0]))[0]
if not len(after): sys.exit("no black sync phase seen")
t_black = ts[after[0]]; t_cal = t_black + 2.0; t_steps = t_cal + 2.5
# calibration disc: dark blob on white, below the bezel (lores y > 150)
cal = [f for t, f in cal_frames if t_cal + 1.0 < t < t_cal + 2.2]
if not cal: sys.exit("calibration frames missing")
g = np.median(np.stack(cal[:20]), 0)[150:, :]; mask = g < 70; ys, xs = np.nonzero(mask)
if len(xs) < 50: sys.exit("calibration disc not found")
cx, cy = xs.mean() * 3, (ys.mean() + 150) * 3; dia = 2 * np.sqrt(len(xs) / np.pi) * 3; px_per_vh = dia / 20.0
steps = []
for s in range(N * len(SIZES)):
    a, b = t_steps + s * STEP + 0.7, t_steps + (s + 1) * STEP - 0.3
    idx = np.where((ts >= a) & (ts <= b))[0]; exp = SIZES[s % len(SIZES)] * px_per_vh / 15.6; tol = max(30.0, 0.6 * exp)
    hit = anyd = 0; sc = []; sz = []
    for i in idx:
        ds = [d for d in rec[i][2] if d[4] >= 0.25]; anyd += bool(ds)
        # a hit must be at the target point AND drone-sized (0.5x to 1.6x): the first run counted
        # detections of the photo's square outline (box = 3.3x the drone) as hits at small sizes
        near = [d for d in ds if abs((d[0] + d[2]) / 2 - cx) < tol and abs((d[1] + d[3]) / 2 - cy) < tol
                and 0.5 * exp <= max(d[2] - d[0], d[3] - d[1]) <= 1.6 * exp]
        if near:
            d = max(near, key=lambda x: x[4]); hit += 1; sc.append(d[4]); sz.append(max(d[2] - d[0], d[3] - d[1]))
    steps.append({"image": s // len(SIZES), "target_native_px": round(exp, 1), "frames": int(len(idx)), "frames_hit": hit, "frames_any_detection": anyd,
                  "score_median": round(float(np.median(sc)), 3) if sc else None, "box_native_px_median": round(float(np.median(sz)), 1) if sz else None})
json.dump({"hef": hef, "input": [w, h], "frames_total": len(rec), "fps": round(len(rec) / (ts[-1] - ts[0]), 1),
           "calibration": {"centre_native": [round(cx, 1), round(cy, 1)], "disc_dia_native_px": round(dia, 1), "px_per_vh": round(px_per_vh, 2)}, "steps": steps}, open(out, "w"), indent=1)
by = {}
for st in steps: by.setdefault(st["target_native_px"], []).append(st)
print("px_per_vh", round(px_per_vh, 2), "centre", round(cx), round(cy), "fps", round(len(rec) / (ts[-1] - ts[0]), 1))
for z, lst in by.items():
    f = sum(x["frames"] for x in lst); hh = sum(x["frames_hit"] for x in lst)
    print(f"target {z:6.1f} px native ({z/3:5.1f} at 640): hit {hh}/{f} = {hh/max(1,f):.2f}; images with >=50% hits: {sum(1 for x in lst if x['frames'] and x['frames_hit']/x['frames']>=0.5)}/{len(lst)}")
