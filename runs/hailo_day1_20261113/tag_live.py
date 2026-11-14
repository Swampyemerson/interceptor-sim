import time, json, sys
import numpy as np, cv2
from picamera2 import Picamera2
try:
    from pyapriltags import Detector
except ImportError:
    from pupil_apriltags import Detector
det = Detector(families="tag36h11", quad_decimate=2.0, nthreads=4)
cam = Picamera2()
out = {}
for label, ctrl in (("auto", {}), ("1ms", {"AeEnable": False, "ExposureTime": 1000, "AnalogueGain": 4.0})):
    cfg = cam.create_video_configuration(main={"size": (1920, 1200), "format": "RGB888"}, controls={"FrameRate": 60, **ctrl})
    cam.configure(cfg); cam.start(); time.sleep(1.0)
    n = hits = 0; ids = set(); sizes = []; dts = []; exps = []; t_prev = None; t0 = time.time()
    while n < 90:
        req = cam.capture_request(); img = req.make_array("main"); md = req.get_metadata(); req.release()
        ts = md.get("SensorTimestamp"); 
        if t_prev is not None and ts: dts.append((ts - t_prev) / 1e9)
        t_prev = ts; exps.append(md.get("ExposureTime"))
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        t1 = time.time(); d = det.detect(g); dec = time.time() - t1
        n += 1
        if d:
            hits += 1; ids.update(x.tag_id for x in d)
            c = d[0].corners; sizes.append(float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)])))
        if n == 45: cv2.imwrite(f"/home/admin/tag_live_{label}.jpg", img)
    wall = time.time() - t0; cam.stop()
    out[label] = {"frames": n, "frames_with_tag": hits, "ids": sorted(ids), "tag_side_px_median": round(float(np.median(sizes)), 1) if sizes else None,
                  "sensor_fps_median": round(1 / float(np.median(dts)), 1) if dts else None, "loop_fps_incl_decode": round(n / wall, 1),
                  "last_decode_ms": round(dec * 1000, 1), "exposure_us_median": float(np.median([e for e in exps if e])), "mean_gray": round(float(g.mean()), 1)}
print(json.dumps(out, indent=1))
