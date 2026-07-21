# `scripts/pi_setup/` — Pi 5 seeker-rig provisioning pack

Everything needed to bring up the **seeker rig's Raspberry Pi 5** for the tripod
field test, prepared **before the Pi arrives** so the first live run is a smoke
test, not a debugging session. This is the software half of the Tier-1 `seeker`
subsystem (`docs/project_state.json` `build_tab` steps **skr-01..skr-08**;
`docs/tripod_test_protocol.md` §1).

The camera is the **innomaker CAM-MIPIOV9281 V2** — a 1 MP **monochrome
global-shutter** OV9281, 1280×800, ~118° HFoV. Every camera-specific line below
cites the paper-check that cleared it: **`docs/camera_paper_check.md`**.

## What's in the pack

| File | What it is |
|---|---|
| `provision.sh` | Idempotent bring-up: apt deps, a `--system-site-packages` venv, the marker-guarded `config.txt` block (`dtoverlay=ov9281`, `camera_auto_detect=0`, `enable_uart=1`), frees the UART for TELEM2, ssh/mDNS. `--dry-run` prints the full plan and changes nothing; `--revert` undoes it. |
| `requirements-pi.txt` | The pinned pip deps that go **into the venv** (mavsdk, onnxruntime, pyapriltags). numpy/opencv/picamera2 come from apt — see the file's header for why. |
| `capture.service` | Optional systemd unit to run one `pi_capture.py` session on demand. |
| `selftest.sh` | Offline gate (exits 0/1, no hardware): `bash -n` + a `--dry-run` that asserts zero file changes and a full printed plan. |

---

## First-boot walkthrough

### 0. Flash the OS  (skr-01)

Flash **Raspberry Pi OS Bookworm, 64-bit** to the microSD with Raspberry Pi
Imager. In the Imager's advanced options, pre-set **hostname, user/password,
Wi-Fi, and enable SSH** — that gets you in headless with no monitor. Boot the Pi
5, **fit the active cooler** (skr-01 runs the CPU hard during the skr-07 thermal
soak), and seat the camera:

- The OV9281 board is **15-pin (1 mm)**; the **Pi 5 CSI port is 22-pin (0.5 mm)**,
  so the **Wonrabai 22→15 adapter cable is required** — narrow 22-pin end at the
  Pi (`docs/camera_paper_check.md` item 5). Ribbon contacts face the right way,
  connector latches down.

### 1. Provision  (idempotent)

Clone the repo onto the Pi, then **preview** the plan (safe, changes nothing),
and **apply** it as root:

```bash
git clone <this-repo> ~/interceptor-sim && cd ~/interceptor-sim

# preview — prints every action, touches nothing
bash scripts/pi_setup/provision.sh --dry-run

# apply (needs root: writes /boot + /etc, installs apt)
sudo bash scripts/pi_setup/provision.sh --hostname interceptor-seeker

sudo reboot   # config.txt + cmdline changes only take effect after a reboot
```

What it changes, and why each camera line is safe:

- **`dtoverlay=ov9281` + `camera_auto_detect=0`** in `/boot/firmware/config.txt`
  — the OV9281 is a **mainline libcamera/kernel sensor**, no vendor driver to
  build; on Pi 5 / Bookworm you turn the auto-probe off so the overlay is
  honored (`docs/camera_paper_check.md` item 5). The base libcamera tree has no
  OV9281 tuning file — `uncalibrated.json` is fine here because **mono means no
  AWB/CCM to lose** (same item 5).
- **`enable_uart=1`** + the serial console freed off `ttyAMA0` — so the **TELEM2
  MAVLink link to the flight controller** (3.3 V, 921600 baud, TX/RX crossed)
  can own the GPIO UART later (`docs/hardware_order_list.md` FC↔Pi UART;
  `project_state.json` `build_tab` brain, `MAV_1_CONFIG=TELEM2`). No FC is wired
  for the tripod day — this just gets the OS out of the way in advance.
- The block is **marker-guarded** (`# >>> interceptor-sim seeker provision >>>`),
  so re-running is a no-op and `sudo bash provision.sh --revert` strips it
  cleanly (originals are also backed up to `*.interceptor-sim.bak`).

Re-running `provision.sh` any time is safe — every step is a no-op when already
applied.

### 2. Verify the camera  (skr-02)

After the reboot, prove live frames at native resolution with the libcamera
stills tool (Bookworm's is `rpicam-still`):

```bash
rpicam-hello --list-cameras                 # OV9281 should enumerate
rpicam-still -t 1000 --width 1280 --height 800 -o /tmp/ov9281.jpg
```

You want a **1280×800** frame (`docs/camera_paper_check.md` item 2: 1280×**800**
vs the sim's 1280×960 — the 800-row sensor sees a shorter vertical strip). If
`rpicam-hello` times out, the fix is almost always **an out-of-date libcamera**
on Pi 5 / Bookworm — update it before anything else (item 5).

### 3. Smoke-test the capture script  (skr-03, skr-05)

First the **offline** self-test (no camera — proves the pipeline/layout):

```bash
.venv-pi/bin/python scripts/seeker/pi_capture.py --self-test    # exits 0/1
```

Then a **live** picamera2 pass on a static indoor scene, requesting the **≤1 ms
exposure** — the short global-shutter exposure is the whole reason for this
sensor and is what the field day's motion-blur read depends on
(`docs/camera_paper_check.md` item 3: `ExposureTime` is exposed in µs, sensor
floor ~40 µs ≪ 1 ms; make sure there's enough **daylight/gain** or the frame
goes dark):

```bash
.venv-pi/bin/python scripts/seeker/pi_capture.py \
    --source picamera2 --out sessions/smoke --n-frames 30 --exposure-us 1000
```

**Check `sessions/smoke/meta.json`** — this is the skr-03/skr-05 pass criterion:

```bash
python3 -c "import json;m=json.load(open('sessions/smoke/meta.json'));\
print('applied_us =',m['applied_exposure_us'],'meets_spec =',m['exposure_meets_spec'])"
```

`exposure_meets_spec` must be **`true`** and `applied_exposure_us` **≤ 1000**.
The value is read from the sensor's own per-frame metadata, so you are
*verifying* the ≤1 ms floor on this Pi, not assuming it. If it reads over 1 ms,
the sensor didn't apply the short exposure (usually too little light, or
`FrameDurationLimits` not lowered) — fix it now, indoors, not at the field.

> **Mono ≠ color caveat** (`docs/camera_paper_check.md` item 4): the deployed NN
> `drone_finetuned_quad_v2` was trained on **color** sim renders; these OV9281
> frames are **monochrome**, a domain shift that can only *lower* recall. Log it,
> don't "fix" it with a channel hack — the sanctioned fix is the real-data mono
> retrain (`docs/real_data_pipeline.md`). Relevant to curve (b), not to the ≤1 ms
> exposure check above.

### 4. (Optional) run captures via systemd

For hands-off field captures you can drive `pi_capture.py` from `capture.service`
instead of a terminal:

```bash
sudo cp scripts/pi_setup/capture.service /etc/systemd/system/
sudo systemctl daemon-reload
# tune per-pass values without editing the unit:
printf 'SESSION=sessions/pass01\nNFRAMES=300\nEXPOSURE_US=1000\n' \
    | sudo tee /etc/default/interceptor-capture
sudo systemctl start capture.service     # begin;  stop to end early
journalctl -u capture -f                 # watch
```

Edit `User=` and the `/home/pi/...` paths in the unit first if your flashed
username differs. Start it **on demand** — do not `enable` it for boot.

---

## The rest of the Build-tab bench items (skr-01..skr-08)

This pack directly covers **skr-01..skr-05**; the remaining three are bench tasks
that need real light/pixels or the flight controller and are pointed to here so
the walkthrough hands off cleanly:

| Step | Item | Where |
|---|---|---|
| **skr-01** | Flash Pi OS, boot, fit the active cooler | §0 above |
| **skr-02** | Seat the CSI ribbon (22-pin end at the Pi), prove live 1280×800 frames | §2 above (`rpicam-still`) |
| **skr-03** | Verify exposure control reaches **≤1 ms** | §3 above (`meta.json` `exposure_meets_spec`); `camera_paper_check.md` item 3 |
| **skr-04** | Paper-check the FoV — **DONE 2026-07-20** | `docs/camera_paper_check.md` (118° is **horizontal**; px/deg penalty ~15%, not 30%) |
| **skr-05** | Bench `pi_capture.py` indoors on a static scene, confirm ≤1 ms applied | §3 above (the live picamera2 pass) |
| skr-06 *(gate)* | Calibrate: ≥15 checkerboard views, RMS ≤ 1.0 px; **re-measure the delivered adjustable lens HFoV** here | `scripts/calibrate_camera.py`; `docs/tripod_test_protocol.md` §5; `camera_paper_check.md` items 1–2 |
| skr-07 | Bench the real Pi 5: sustained AprilTag fps + CPU-YOLO fps through a thermal soak | `docs/tripod_test_protocol.md` §7.3 (`pi5-emulation-gap`) |
| skr-08 | Run the NN shadow-scoring path on bench frames (v2 @640, conf 0.25) | `docs/tripod_test_protocol.md` §7.2 |

Two things to re-check the moment hardware lands (`camera_paper_check.md` "Net"):
(a) re-measure the **adjustable lens HFoV** against the checkerboard at skr-06;
(b) confirm `dtoverlay=ov9281` + `uncalibrated.json` actually brings up frames on
*this* Pi 5 / Bookworm (§2) before trusting any range number.

---

## Reversing the changes

```bash
sudo bash scripts/pi_setup/provision.sh --revert && sudo reboot
```

Strips the managed `config.txt` block, restores `cmdline.txt` from its backup,
and unmasks the serial-getty. apt packages and the venv are left in place
(harmless to keep).

## Self-test (before you trust it)

```bash
bash scripts/pi_setup/selftest.sh     # exits 0/1, no hardware, safe on any box
```

Runs `bash -n provision.sh` and a sandboxed `--dry-run` that asserts the plan is
complete and that **no files were written and no venv created** — the proof that
`--dry-run` is genuinely side-effect-free, which is what makes it safe to preview
on this desk before running it on the Pi.
