import sys, time, json
import numpy as np, cv2
from picamera2 import Picamera2
from picamera2.devices import Hailo
hef, label, save = sys.argv[1], sys.argv[2], sys.argv[3]
BOOT = time.CLOCK_BOOTTIME
with Hailo(hef) as hailo:
    h, w, _ = hailo.get_input_shape()
    cam = Picamera2()
    native = h > 1200          # model taller than the sensor: feed the full frame, padded
    streams = {"main": {"size": (1920, 1200), "format": "RGB888"}}
    if not native: streams["lores"] = {"size": (w, h), "format": "RGB888"}
    cfg = cam.create_video_configuration(**streams, controls={"FrameRate": 60})
    cam.configure(cfg); cam.start(); time.sleep(1.0)
    lat = []; inf = []; dts = []; t_prev = None; best = {}; n = 0; t0 = time.time(); hits = 0
    while n < 300:
        req = cam.capture_request(); md = req.get_metadata()
        if native:
            big_now = req.make_array("main"); frame = np.zeros((h, w, 3), np.uint8); frame[:1200] = big_now
        else:
            frame = req.make_array("lores")
        if n == 150: big = big_now.copy() if native else req.make_array("main")
        req.release()
        t1 = time.perf_counter(); out = hailo.run(frame); t2 = time.perf_counter()
        now = time.clock_gettime_ns(BOOT); ts = md["SensorTimestamp"]
        lat.append((now - ts) / 1e6); inf.append((t2 - t1) * 1e3)
        if t_prev: dts.append((ts - t_prev) / 1e6)
        t_prev = ts; n += 1; got = False
        for cls, dets in enumerate(out):
            for d in dets:
                if d[4] >= 0.4:
                    got = True
                    if d[4] > best.get(cls, (0,))[0]: best[cls] = (float(d[4]), [round(float(x), 3) for x in d[:4]])
        hits += got
    wall = time.time() - t0; cam.stop()
    if 11 in best:
        y0, x0, y1, x1 = best[11][1]; H, W = big.shape[:2]
        cv2.rectangle(big, (int(x0 * W), int(y0 * H)), (int(x1 * W), int(y1 * H)), (0, 255, 0), 4)
    cv2.imwrite(save, big)
p = lambda a, q: round(float(np.percentile(a, q)), 1)
print(json.dumps({"model": label, "input": [w, h], "frames": n, "loop_fps": round(n / wall, 1),
  "frame_interval_ms_median": p(dts, 50), "infer_call_ms_median": p(inf, 50), "infer_call_ms_p95": p(inf, 95),
  "capture_to_result_ms_median": p(lat, 50), "p95": p(lat, 95), "max": p(lat, 100),
  "frames_with_detection_ge_0.4": hits, "best_by_coco_class": {str(k): v for k, v in best.items()}}))
