import json, time
import numpy as np, cv2
from picamera2 import Picamera2
cam = Picamera2(); cam.configure(cam.create_video_configuration(main={"size": (1920, 1200), "format": "RGB888"}, controls={"FrameRate": 60})); cam.start(); time.sleep(2.0)
img = cam.capture_array("main"); cam.stop(); g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
scr = (g > 110).astype(np.uint8); n, lab, st, _ = cv2.connectedComponentsWithStats(scr)
k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA])); x, y, w, h = st[k, :4]
hull = cv2.morphologyEx((lab == k).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((151, 151), np.uint8))
dark = ((g < 70) & (cv2.erode(hull, np.ones((25, 25), np.uint8)) > 0)).astype(np.uint8)
n2, lab2, st2, cen2 = cv2.connectedComponentsWithStats(dark)
out = {"screen_bbox": [int(x), int(y), int(w), int(h)], "screen_area_frac": round(float(st[k, cv2.CC_STAT_AREA]) / g.size, 3)}
if n2 > 1:
    j = 1 + int(np.argmax(st2[1:, cv2.CC_STAT_AREA])); dia = 2 * np.sqrt(st2[j, cv2.CC_STAT_AREA] / np.pi)
    out.update({"disc_centre": [round(float(cen2[j][0]), 1), round(float(cen2[j][1]), 1)], "disc_dia_px": round(float(dia), 1), "px_per_vh": round(float(dia) / 20, 2),
                "disc_bbox_wh": [int(st2[j, 2]), int(st2[j, 3])]})
cv2.imwrite("/home/admin/drone_eval/cal2.jpg", img); print(json.dumps(out))
