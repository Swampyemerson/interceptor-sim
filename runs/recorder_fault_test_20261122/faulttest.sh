#!/usr/bin/env bash
# Bench fault test of the flight recorder, run ON the Pi. Nothing here can arm:
# "armed" is the recorder's own --pretend-armed flag. Sends the Pixhawk only what
# the service sends (log start/stop, acks).
set -u
cd ~/interceptor-sim
PY=.venv-pi/bin/python
T=/tmp/rec_test; rm -rf $T; mkdir -p $T/video
DEV=$(for d in /sys/bus/usb/devices/*; do [ -f $d/idVendor ] && [ "$(cat $d/idVendor)" = 3185 ] && basename $d; done)
unplug() { echo $DEV | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null; echo "[$(date +%T)] USB UNPLUGGED (software)"; }
replug() { echo $DEV | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null; echo "[$(date +%T)] USB PLUGGED BACK"; }
snap() { echo "[$(date +%T)] $1: $($PY -c "import json;s=json.load(open('$T/status.json'));print({k:s.get(k) for k in ('state','port','armed','video','video_starts','recent_kb_per_s','detail')})")"; }
start_rec() {
  $PY scripts/bench/ulog_stream_recorder.py --port auto --fallback-port /dev/ttyAMA10 --per-flight \
     --gcs-udp-port 14550 --out-dir $T --status-file $T/status.json --pretend-armed 0 1000000 \
     --on-arm "$PY scripts/field/record_video.py --flip --note bench-fault-test --out-dir $T/video" > $T/rec_$1.log 2>&1 &
  REC=$!
}
sudo systemctl stop flight-recorder
echo "===== RUN 1: armed (pretend), USB pulled, serial cable free; then the video is killed ====="
start_rec 1; sleep 15; snap "before"
unplug; sleep 12; snap "USB out"
VPID=$(grep -o "on-arm command started (pid [0-9]*" $T/rec_1.log | tail -1 | grep -o "[0-9]*$")
kill -9 $VPID; echo "[$(date +%T)] video process $VPID KILLED"; sleep 9; snap "after video kill"
replug; sleep 6; snap "USB back (still armed)"
kill -TERM $REC; wait $REC; echo "recorder exit code $?"
cat $T/rec_1.log | grep -v "^\[\|^ *\"\|^{\|^}\|INFO\|WARN" | cut -c1-210
echo "===== RUN 2: armed (pretend), USB pulled WHILE another program holds the serial cable ====="
sleep 100 3<>/dev/ttyAMA10 & HOLD=$!
sleep 1; start_rec 2; sleep 15; snap "before"
unplug; sleep 10; snap "USB out, serial busy"
replug; sleep 12; snap "USB back"
kill -TERM $REC; wait $REC; echo "recorder exit code $?"; kill $HOLD
cat $T/rec_2.log | grep -v "^\[\|^ *\"\|^{\|^}\|INFO\|WARN" | cut -c1-210
echo "===== files ====="
ls -la $T | grep -v status; for m in $T/video/*/meta.json; do echo $m; $PY -c "import json;m=json.load(open('$m'));print({k:m.get(k) for k in ('frames','seconds_of_video','measured_fps','gaps_over_1.5_frames','stalled','flip_180')})"; done
for f in $T/*.stats.json; do $PY -c "import json,sys;s=json.load(open('$f'));print('$f'.split('/')[-1], s.get('verdict'), s.get('port'), s.get('seconds'), 's', s.get('end_reason'), '| video:', s.get('on_arm_command'))"; done
sudo systemctl start flight-recorder; sleep 4; systemctl is-active flight-recorder
echo FAULTTEST_DONE
