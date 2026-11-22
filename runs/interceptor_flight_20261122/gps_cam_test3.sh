#!/usr/bin/env bash
# Camera on/off against the GPS, after shielding. Ground, disarmed, read-only on the Pixhawk side.
cd ~/interceptor-sim; PY=.venv-pi/bin/python
sec() { $PY -c "import json;s=json.load(open('/dev/shm/flight_recorder_status.json'));print(s['seconds'], s['file'], s['port'], 'armed=%s' % s['armed'])"; }
echo "START $(sec)"; sleep 25
echo "CAM_ON $(sec)"
$PY scripts/field/record_video.py --flip --seconds 30 --note gps-interference-test-after-foil --out-dir /tmp/gps_cam_test > /tmp/gps_cam_test.log 2>&1
echo "CAM_OFF $(sec) exit $?"; sleep 25
echo "END $(sec)"
