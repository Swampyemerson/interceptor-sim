# New Pi 5 bench, 2026-11-13: Hailo day-1, camera probe, Pixhawk link probe

Board: Raspberry Pi 5 8 GB (revision d04171), kernel 6.18.50+rpt-rpi-2712, Debian 13,
booted from the first Pi's SD card, AI HAT+ fitted, at 192.168.0.61. Pixhawk 6C Mini on
USB to the PC (COM5). All run over SSH from the WSL dev machine.

## Hailo
- Installed `hailort`, `hailort-pcie-driver`, `python3-hailort`, all 4.23.0 (NOT `hailo-all`,
  which would replace the camera libraries and pull about 200 build packages).
  The driver installed WITHOUT DKMS, so a kernel update will drop it; reinstall then.
- `identify.log`: Hailo-8, firmware 4.23.0.
- `hailortcli benchmark -t 15`, chip only, synthetic input:

| Model file | Input | Contexts | Frames/s (hardware) | Frames/s (streaming) | Latency |
|---|---|---|---|---|---|
| `runs/dfc_dryrun_single/yolov8n.hef` (1 class) | 640 x 640 | 2 | 65.3 | 65.2 | 13.2 ms |
| `runs/dfc_dryrun/yolov8n.hef` (80 classes) | 1024 x 1024 | 1 | 91.7 | 91.6 | 13.5 ms |

Pi at 47 C afterwards, no throttle flag. These are NOT camera-to-detection rates and the
files are compile dry-runs, not the trained seeker weights.

## Camera
- `dtoverlay=ov9281` fails on both sockets (no answer at I2C address 0x60).
- `dtoverlay=arducam-pivariety,cam0` answers: firmware 0x10004, 1920 x 1200, 10-bit GRBG.
  That is the AR0234 (B0353) in socket CAM/DISP0.
- `rpicam-hello --list-cameras` still says "No cameras available": the stock camera
  library (libcamera 0.7.2) has no support for this sensor family. Arducam's own
  packages are needed; not installed.
- `/boot/firmware/config.txt` now carries `dtoverlay=arducam-pivariety,cam0`
  (backup: `config.txt.camprobe.bak`). `provision.sh` still writes the OV9281 line.

## Pixhawk link (builder reports a cable is plugged in)
- 0 bytes received on `/dev/ttyAMA0` and `/dev/ttyAMA10` at 921600, 115200 and 57600.
- Edge monitor on header pins GPIO2 to GPIO27 for 4 s: 0 edges on every pin
  (control: a pin toggled by hand registered 10 edges). No pin held high against a pull-down.
- 3-pin socket pins read 2,420 and 3,583 times: never low
  (control: the Pi's own slow transmit read low 1,847 times out of 2,067).
- So no transmit signal from the Pixhawk reaches any Pi pin. Swapped wires alone would
  put that signal on the Pi's transmit pin, where it would have been seen.
- Neither end can swap the two wires in software: the Pi's pins are fixed, and the
  Pixhawk's settings list has only speed settings for its serial ports.
- Pixhawk settings dump: `configs/px4_6cmini/dumps/20261113_bench_new_pi.tsv`.
