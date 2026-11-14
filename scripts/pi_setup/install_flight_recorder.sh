#!/usr/bin/env bash
# Install (or update) the flight recorder service on the Pi. Run ON the Pi:
#     scripts/pi_setup/install_flight_recorder.sh
# Undo:
#     sudo systemctl disable --now flight-recorder
#     sudo rm /etc/systemd/system/flight-recorder.service /etc/udev/rules.d/99-pixhawk-usb.rules
#     sudo systemctl daemon-reload && sudo udevadm control --reload
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
USER_NAME="$(id -un)"
PY="$REPO/.venv-pi/bin/python"

[[ -x "$PY" ]] || { echo "FAIL: $PY not found (run scripts/pi_setup/provision.sh first)"; exit 1; }
"$PY" -c "import pymavlink, picamera2" || { echo "FAIL: pymavlink or picamera2 missing from .venv-pi"; exit 1; }
for g in dialout video; do
    id -nG | tr ' ' '\n' | grep -qx "$g" || { echo "FAIL: $USER_NAME is not in the $g group"; exit 1; }
done

TMP="$(mktemp)"
sed -e "s|^User=.*|User=$USER_NAME|" \
    -e "s|/home/admin/interceptor-sim|$REPO|g" \
    -e "s|/home/admin/fc_logs|$HOME/fc_logs|g" \
    "$HERE/flight-recorder.service" > "$TMP"
sudo install -m 644 "$TMP" /etc/systemd/system/flight-recorder.service
rm -f "$TMP"
sudo install -m 644 "$HERE/99-pixhawk-usb.rules" /etc/udev/rules.d/99-pixhawk-usb.rules
if [[ ! -e /etc/default/flight-recorder ]]; then
    sudo tee /etc/default/flight-recorder > /dev/null <<CONF
# Settings for flight-recorder.service. Remove the leading '#' to change one,
# then: sudo systemctl restart flight-recorder
#
# Serial port used while the Pixhawk's USB is NOT plugged into the Pi.
#FALLBACK_PORT=/dev/ttyAMA10
#
# Command run while armed. Outdoors a fixed short exposure freezes motion:
#VIDEO_CMD=$PY scripts/field/record_video.py --note flight --exposure-us 1000
CONF
fi
if ! grep -q GCS_PORT /etc/default/flight-recorder; then
    sudo tee -a /etc/default/flight-recorder > /dev/null <<CONF
#
# Network port where QGroundControl reaches the Pixhawk through this Pi.
# 0 turns the bridge off (nothing on the network can then reach the Pixhawk).
#GCS_PORT=14550
CONF
fi
mkdir -p "$HOME/fc_logs"
sudo systemctl daemon-reload
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=tty
sudo systemctl enable flight-recorder
sudo systemctl restart flight-recorder
sleep 3
echo "Pixhawk USB plugged into this Pi: $([[ -e /dev/pixhawk-usb ]] && echo yes || echo 'no (recording over the serial cable, which loses part of the log)')"
systemctl --no-pager --lines=6 status flight-recorder || true
