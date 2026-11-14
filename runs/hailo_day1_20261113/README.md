# New Pi 5 bench, 2026-11-13: Hailo day-1, camera probe, Pixhawk link probe

Board: Raspberry Pi 5 8 GB (revision d04171), kernel 6.18.50+rpt-rpi-2712, Debian 13,
booted from the first Pi's SD card, AI HAT+ fitted, at 192.168.0.61. Pixhawk 6C Mini on
USB to the PC (COM5). All run over SSH from the WSL dev machine.

## Hailo
- Installed `hailort`, `hailort-pcie-driver`, `python3-hailort`, all 4.23.0 (NOT `hailo-all`,
  which would replace the camera libraries and pull about 200 build packages), plus
  `hailo-models` 1.0.0-2 (Raspberry Pi's ready-made model files).
  The driver installed WITHOUT DKMS, so a kernel update will drop it; reinstall then.
- `identify.log`: Hailo-8, firmware 4.23.0. DFC 3.34 files run on HailoRT 4.23 (pairing confirmed).
- **PCIe speed was the first bottleneck.** The link came up at Gen 2 (5 GT/s, one lane).
  Adding `dtparam=pciex1_gen=3` to `/boot/firmware/config.txt` brought it to Gen 3 (8 GT/s):
  `pcie_link_gen3.log`.

`hailortcli benchmark`, chip only, synthetic input, one frame at a time unless noted:

| Model file | Input | Contexts | Gen 2 | Gen 3 | Gen 3 latency |
|---|---|---|---|---|---|
| project 1-class compile (`runs/dfc_dryrun_single`), random weights | 640 x 640 | 2 | 65 fps | 119 fps | 6.4 ms |
| same, batch 8 | 640 x 640 | 2 | 139 fps | 276 fps | 16.8 ms |
| stock yolov8n, COCO (Hailo Model Zoo v2.17) | 640 x 640 | 1 | - | 431 fps | 3.4 ms |
| yolov6n, COCO (`hailo-models`) | 640 x 640 | 1 | - | 586 fps | 3.2 ms |
| yolov8s, COCO (`hailo-models`) | 640 x 640 | 1 | - | 310 fps | 6.7 ms |
| yolov8m, COCO (Hailo Model Zoo v2.17) | 640 x 640 | 3 | - | 31 fps | 25.8 ms |
| stock yolov8n compile (`runs/dfc_dryrun`) | 1024 x 1024 | 1 | 92 fps | 125 fps | 12.6 ms |
| stock yolov8n compile (`runs/dfc_native_res_20261113`) | 1280 x 800 | 1 | - | 128 fps | 13.5 ms |
| stock yolov8n compile (same folder) | 1920 x 1216 | 1 | - | 56 fps | 26.0 ms |

Logs: `bench_*.log`, `bench_sizes_gen3.txt`. Reading: nano and small are far inside the
budget; medium falls off a cliff because it no longer fits the chip in one piece (3 contexts);
the 1-class compile pays about 3.6x for its second context but is still 119 fps.

## Camera (AR0234, B0353)
- `dtoverlay=ov9281` fails on both sockets (no answer at I2C address 0x60).
- `dtoverlay=arducam-pivariety,cam0` answers: firmware 0x10004, 1920 x 1200, 10-bit GRBG.
- The stock camera library could not use it. Fixed by installing Arducam's rebuilt
  `libcamera0.7` and `libcamera-ipa` 0.7.2+rpt20260819-2 (two .deb files from Arducam's
  GitHub release, read before install; same version line as stock) and holding both with
  `apt-mark hold`. Before/after package lists: `packages_before.txt`, `packages_after.txt`.
  Roll back with `apt install --allow-downgrades libcamera0.7=0.7.2+rpt20260817-1 libcamera-ipa=0.7.2+rpt20260817-1`.
- Modes now listed: 1920 x 1200 at 60 fps, 1920 x 1080 at 60, 1280 x 720 at 120.
- A warning remains: no colour tuning file for this sensor (`arducam-pivariety.json`).
- `/boot/firmware/config.txt` after the session: `config_txt_after.txt`
  (backup on the Pi: `config.txt.camprobe.bak`). `provision.sh` still writes the OV9281 line
  and knows nothing of the PCIe line; re-running it would undo the camera setting.
- The image is upside down as mounted on the bench (see the saved frames).

## Live tests, camera pointed at the laptop screen (`live_results.json`)
- **AprilTag (CPU):** tag36h11 id 0 decoded in 90/90 frames in auto exposure and 90/90 at a
  1 ms exposure (992 us applied), 59.5 fps sensor rate, about 9 to 10 ms per decode
  (`tag_live.py`, `tag_on_screen_1ms.jpg`). The tag was 358 px across: a works/doesn't check,
  not a range test.
- **Camera to Hailo to detection (`hailo_live.py`):** a stop sign on the screen
  (`stop_sign_test_page.html`), COCO models, 300 frames each, 640 x 640 input resized by the
  camera pipeline:

| Model | Loop rate | Inference call | Frame start to result (median / 95% / worst) | Frames with the sign |
|---|---|---|---|---|
| yolov8s | 59.2 fps | 11.1 ms | 31.8 / 33.3 / 91.0 ms | 300/300 (0.79) |
| yolov8n | 59.6 fps | 7.9 ms | 28.5 / 30.0 / 35.4 ms | 300/300 (0.81) |
| project 1-class compile (random weights, timing only) | 59.6 fps | 7.9 ms | 28.8 / 30.2 / 35.5 ms | n/a |
| yolov8n at 1280 x 800 | 41.7 fps | 23.0 ms | 52.8 / 61.1 / 65.6 ms | 300/300 (0.88) |
| yolov8n at 1920 x 1216 (full frame) | 19.1 fps | 48.1 ms | 79.7 / 89.2 / 90.9 ms | 300/300 (0.79) |

  At 640 the loop is limited by the camera's 60 fps, not the chip. At the two larger sizes
  the simple one-frame-at-a-time Python call is the limit (the chip alone does 128 and 56 fps);
  the larger-input files use pretrained weights with synthetic calibration, so they show
  timing and fit, not accuracy, and their box coordinates are not valid. Annotated frame:
  `live_detect_yolov8s.jpg`. Not measured: the trained seeker weights on the chip, a
  quantised-versus-float recall comparison, small or distant targets, motion blur, sunlight.

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
