#!/usr/bin/env python3
"""Pi side of the screen bench: run a one-class drone HEF on every frame while bench.html plays,
then cut the frames into the page's steps using the white-to-black sync edge.
Usage: bench_pi.py <hef> <out_json> [page_ppv]   (start it, then load bench.html?ppv=<page_ppv> within a few seconds)"""
import json, os, sys, time
import numpy as np, cv2
from picamera2 import Picamera2
from picamera2.devices import Hailo
hef, out = sys.argv[1], sys.argv[2]
PPV_PAGE = float(sys.argv[3]) if len(sys.argv) > 3 else 15.6      # the ?ppv= value the page was loaded with
# sync patch: a spot ON the screen, beside the calibration disc (native px), from a prior disc calibration
SYNC_XY = (float(sys.argv[4]) - 260, float(sys.argv[5])) if len(sys.argv) > 5 else (600.0, 800.0)
SIZES = [150, 110, 80, 60, 45, 33, 24, 18]; N = 10; STEP = 2.5; TOTAL = float(os.environ.get("BENCH_TOTAL", 7.5 + N * len(SIZES) * STEP + 14))
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
        g_small = lo[::lo_size[1] // 200, ::lo_size[0] // 320, 1]            # about 320 x 200 green channel
        nat = []
        for d in dets:
            if d[4] < 0.2: continue
            if h == 640: b = [d[1] * 1920, (d[0] * 640 - 120) * 3, d[3] * 1920, (d[2] * 640 - 120) * 3]
            else: b = [d[1] * 1920, d[0] * 1200, d[3] * 1920, d[2] * 1200]
            nat.append([round(float(v), 1) for v in b] + [round(float(d[4]), 3)])
        py, px = int(SYNC_XY[1] / 1200 * g_small.shape[0]), int(SYNC_XY[0] / 1920 * g_small.shape[1])
        rec.append((ts, float(g_small[max(py - 8, 0):py + 8, max(px - 8, 0):px + 8].mean()), nat))
        if time.time() - t_start < 30 and len(rec) % 4 == 0: cal_frames.append((ts, g_small.copy()))
    cam.stop()
ts = np.array([r[0] for r in rec]); br = np.array([r[1] for r in rec])
# Sync: the page goes white (3 s) then black (2 s). Read a small patch ON the screen (whole-frame
# brightness fails: auto-exposure lifts the dark room). Edge = dark after 2 s of steady white.
early = br[ts < ts[0] + 30]; p5, p95 = np.percentile(early, 5), np.percentile(early, 95)
lo_th, hi_th = p5 + 0.3 * (p95 - p5), p5 + 0.7 * (p95 - p5); t_black = None
for i in range(130, len(br)):
    if ts[i] > ts[0] + 30: break
    prev = br[i - 125:i - 5].min()            # 2 s of steady white (auto-exposure settles it near 160, not 255)
    if prev > 100 and br[i] < 0.4 * prev:
        t_black = ts[i]; break
if t_black is None:
    json.dump({"t": [round(float(x - ts[0]), 3) for x in ts[::6]], "patch_brightness": [round(float(x), 1) for x in br[::6]], "lo": float(lo_th), "hi": float(hi_th)}, open(out + ".trace.json", "w"))
    sys.exit("no white-then-black sync edge seen -- trace written to " + out + ".trace.json")
t_cal = t_black + 2.0; t_steps = t_cal + 2.5
# Calibration: the screen is the big bright area in the white phase; the disc is the dark blob inside it.
pick = lambda t0: min(cal_frames, key=lambda f: abs(f[0] - t0))[1]
white, calf = pick(t_black - 1.0), pick(t_cal + 1.5); sy, sx = 1200 / white.shape[0], 1920 / white.shape[1]
scr = (white > 0.5 * white.max()).astype(np.uint8); n, lab, st, _ = cv2.connectedComponentsWithStats(scr)
if n < 2: sys.exit("screen not found in the white phase")
k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA])); inside = cv2.erode((lab == k).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
ys, xs = np.nonzero((calf < 0.35 * white.max()) & inside)
if len(xs) < 30: sys.exit("calibration disc not found")
cx, cy = float(xs.mean() * sx), float(ys.mean() * sy); dia = 2 * np.sqrt(len(xs) * sx * sy / np.pi); px_per_vh = dia / 20.0
steps = []
for s in range(N * len(SIZES)):
    a, b = t_steps + s * STEP + 0.7, t_steps + (s + 1) * STEP - 0.3
    idx = np.where((ts >= a) & (ts <= b))[0]; exp = SIZES[s % len(SIZES)] * px_per_vh / PPV_PAGE; tol = max(30.0, 0.6 * exp)
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
