#!/usr/bin/env python3
"""Live camera -> Hailo -> drone detection (on the Pi). The 1920x1200 frame is shrunk by the
camera pipeline to 640x400 and padded to 640x640, so shapes are not distorted.
Usage: live_drone.py <hef> <n_frames> <save_jpg>"""
import json, sys, time
import numpy as np, cv2
from picamera2 import Picamera2
from picamera2.devices import Hailo
hef, n_frames, save = sys.argv[1], int(sys.argv[2]), sys.argv[3]
with Hailo(hef) as hailo:
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (1920, 1200), "format": "RGB888"},
                                                 lores={"size": (640, 400), "format": "RGB888"}, controls={"FrameRate": 60}))
    cam.start(); time.sleep(1.0)
    lat = []; scores = []; sizes = []; hits = 0; best = None; t0 = time.time()
    for n in range(n_frames):
        req = cam.capture_request(); lo = req.make_array("lores"); md = req.get_metadata()
        big = req.make_array("main") if n == n_frames // 2 else None
        req.release()
        frame = np.full((640, 640, 3), 114, np.uint8); frame[120:520] = lo
        res = hailo.run(frame); now = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        lat.append((now - md["SensorTimestamp"]) / 1e6)
        dets = res[0] if isinstance(res, list) and len(res) and getattr(res[0], "ndim", 1) == 2 else res
        dets = [d for d in dets if d[4] >= 0.25]
        if dets:
            hits += 1; d = max(dets, key=lambda x: x[4]); scores.append(float(d[4]))
            box = [float(d[1]) * 1920, (float(d[0]) * 640 - 120) * 3, float(d[3]) * 1920, (float(d[2]) * 640 - 120) * 3]   # native px
            sizes.append(max(box[2] - box[0], box[3] - box[1])); best = box
        if big is not None:
            keep = big.copy(); keep_box = best
    wall = time.time() - t0; cam.stop()
    if keep_box: cv2.rectangle(keep, (int(keep_box[0]), int(keep_box[1])), (int(keep_box[2]), int(keep_box[3])), (0, 255, 0), 3)
    cv2.imwrite(save, keep)
print(json.dumps({"frames": n_frames, "frames_with_drone": hits, "loop_fps": round(n_frames / wall, 1),
                  "score_median": round(float(np.median(scores)), 3) if scores else None,
                  "box_long_side_native_px_median": round(float(np.median(sizes)), 1) if sizes else None,
                  "capture_to_result_ms_median": round(float(np.median(lat)), 1), "p95": round(float(np.percentile(lat, 95)), 1)}))
