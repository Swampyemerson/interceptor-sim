#!/usr/bin/env bash
# Does the camera recording disturb the GPS? Aircraft on the ground, disarmed. Read-only on
# the Pixhawk side: the GPS figures are read from the log the recorder is already saving.
cd ~/interceptor-sim
echo "status: $(tr -d '\n' < /dev/shm/flight_recorder_status.json | cut -c1-150)"
echo "T0 $(date +%s.%N)"; sleep 25
echo "CAM_ON $(date +%s.%N)"
.venv-pi/bin/python scripts/field/record_video.py --flip --seconds 30 --note gps-interference-test --out-dir /tmp/gps_cam_test > /tmp/gps_cam_test.log 2>&1
echo "CAM_OFF $(date +%s.%N) exit $?"; sleep 25
echo "T1 $(date +%s.%N)"
ls -t ~/fc_logs/*.part | head -1
