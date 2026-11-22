#!/usr/bin/env bash
# Which part of the camera chain disturbs the GPS? Ground, disarmed, read-only on the Pixhawk side.
cd ~/interceptor-sim; PY=.venv-pi/bin/python
sec() { $PY -c "import json;print(json.load(open('/dev/shm/flight_recorder_status.json'))['seconds'])"; }
echo "FILE $(ls -t ~/fc_logs/*.part | head -1)"
sleep 12
echo "A_CPU_ONLY_ON $(sec)"
$PY - <<'PY'
import multiprocessing, time
def burn(): 
    t = time.time()
    while time.time() - t < 18: pass
ps = [multiprocessing.Process(target=burn) for _ in range(4)]
[p.start() for p in ps]; [p.join() for p in ps]
PY
echo "A_OFF $(sec)"; sleep 14
echo "B_CAMERA_1920x1200_NO_ENCODE_ON $(sec)"
$PY - <<'PY' 2>/dev/null
import time
from picamera2 import Picamera2
c = Picamera2(); c.configure(c.create_video_configuration(main={"size": (1920, 1200), "format": "YUV420"}, controls={"FrameRate": 60})); c.start(); time.sleep(18); c.stop(); c.close()
PY
echo "B_OFF $(sec)"; sleep 14
echo "C_CAMERA_1280x720_NO_ENCODE_ON $(sec)"
$PY - <<'PY' 2>/dev/null
import time
from picamera2 import Picamera2
c = Picamera2(); c.configure(c.create_video_configuration(main={"size": (1280, 720), "format": "YUV420"}, raw={"size": (1280, 720)}, controls={"FrameRate": 60})); c.start(); time.sleep(18); c.stop(); c.close()
PY
echo "C_OFF $(sec)"; sleep 14
echo "END $(sec)"
ls -t ~/fc_logs/*.part | head -1
