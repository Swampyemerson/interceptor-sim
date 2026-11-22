#!/usr/bin/env bash
# The compass-calibration case with the installed service, disarmed: Pixhawk USB
# out of the Pi for 30 s, then back. Software unplug; sends the Pixhawk nothing new.
set -u
cd ~/interceptor-sim && git pull -q --ff-only && git log --oneline -1
sudo systemctl restart flight-recorder; sleep 40
DEV=$(for d in /sys/bus/usb/devices/*; do [ -f $d/idVendor ] && [ "$(cat $d/idVendor)" = 3185 ] && basename $d; done)
echo "--- before:"; .venv-pi/bin/python scripts/field/recorder_check.py
echo $DEV | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null; echo "[$(date +%T)] USB OUT"; sleep 30
echo "--- while out:"; .venv-pi/bin/python scripts/field/recorder_check.py
echo $DEV | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null; echo "[$(date +%T)] USB BACK"; sleep 8
echo "--- 8 s after plugging back:"; .venv-pi/bin/python scripts/field/recorder_check.py; sleep 30
echo "--- 38 s after plugging back:"; .venv-pi/bin/python scripts/field/recorder_check.py
systemctl show flight-recorder -p NRestarts -p ActiveState
journalctl -u flight-recorder --no-pager --since "-115s" | cut -c41-240
echo CALTEST_DONE
