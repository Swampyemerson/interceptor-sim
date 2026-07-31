# Field bring-up — "the parts arrived, now what?"

This is the plug-it-in-and-go runbook. One command per piece of gear, a clear
PASS/FAIL, and the two things most likely to go wrong with each.

You do not need to remember any of it. Start here:

```bash
cd ~/interceptor-sim
scripts/field/bringup.sh
```

That runs the laptop-side checks in order and prints one summary table.
Run it today with nothing plugged in — it will tell you what is missing instead
of crashing. Run it again each time a box arrives.

---

## 0. What the checks are

| Step | Command | Question it answers |
|---|---|---|
| 00 | `scripts/field/00_detect_devices.sh` | What does my laptop actually see? |
| 05 | `scripts/field/05_pi_link_check.sh` | Can I reach the Pi on the FIELD network — key, IP, clock? |
| 01 | `scripts/field/01_camera_live_check.sh` | Is the camera making good frames? |
| 02 | `scripts/field/02_apriltag_desk_check.sh` | Does the tag decoder see the placard? |
| 03 | `scripts/field/03_fc_bench_check.sh` | Can I talk to the flight controller? |
| 04 | `scripts/field/04_pull_logs.sh` | Get the logs + video off after a flight |
| all | `scripts/field/bringup.sh` | 00 → 05 → 01 → 02 → 03 in one go |
| — | `scripts/field/selftest.sh` | Are the check scripts themselves healthy? (no hardware, exits 0/1) |

Every script prints one of three verdicts:

- **PASS** — checked, all good.
- **FAIL** — the gear is there and something is wrong. Follow the `->` hints.
- **NOT CONNECTED** — that piece is not plugged in yet. Not an error today.

Exit codes, if you ever script around them: `0` pass, `1` fail, `2` bad
arguments, `3` not connected. Add `--require` to any script (or to
`bringup.sh`) to make "not connected" count as a failure — use that on field
day, when everything *is* supposed to be attached.

Every run writes a folder under `logs/field/` with the full text output, a
sample frame, and a small `verdict.json`. Nothing is lost.

---

## 1. The gear map — what plugs where

| Part | Plugs into | How the laptop reaches it |
|---|---|---|
| **Pixhawk 6C Mini** (interceptor brain, runs PX4) | laptop **USB‑C** | a serial port, `/dev/ttyACM0` — **needs the WSL step in §2** |
| **Kakute H7** (target drone, runs ArduPilot) | laptop **USB‑C** | same as above |
| **innomaker OV9281 camera** | the **Raspberry Pi 5**, by ribbon cable | **not the laptop.** The laptop talks to the Pi over Wi‑Fi |
| **Raspberry Pi 5** (seeker rig) | its own USB‑C power | over the network: `ssh pi@<the Pi's IP>` (see §3.0 — `.local` does **not** resolve from WSL2) |
| **microSD log cards** | a card reader in the laptop | a Windows drive under `/mnt/d`, `/mnt/e`, … |

Two things worth saying once, because they cause most of the confusion:

- **The camera cannot plug into your laptop.** It is a *CSI* camera — a ribbon
  cable that only fits the Pi. It will never show up as a camera on the laptop.
  (Source: `docs/camera_paper_check.md` item 5.)
- **A "charge-only" USB‑C cable will do nothing.** The flight controllers need a
  **data** cable. If nothing at all appears when you plug in, try another cable
  before debugging anything else.

---

## 2. The WSL2 USB step (the #1 gotcha)

You run Ubuntu inside Windows (that is what WSL is). **Windows keeps USB devices
to itself.** Until you hand a device over, Ubuntu — and therefore every script in
this repo — cannot see your Pixhawk at all. The tool that hands it over is
**usbipd-win**. Official instructions:
<https://learn.microsoft.com/en-us/windows/wsl/connect-usb>

### One-time install (Windows side)

Open **PowerShell as Administrator** and run:

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

### Every time you plug a flight controller in

1. Keep an Ubuntu/WSL terminal open (this keeps the Linux VM alive).
2. In **PowerShell (Administrator)**, list the devices and find the one you just
   plugged in. Copy its **BUSID** (looks like `4-4`):

   ```powershell
   usbipd list
   ```

3. Share it (first time per device, needs Administrator):

   ```powershell
   usbipd bind --busid 4-4
   ```

4. Attach it to Ubuntu (no Administrator needed):

   ```powershell
   usbipd attach --wsl --busid 4-4
   ```

5. Back in Ubuntu, confirm:

   ```bash
   scripts/field/00_detect_devices.sh
   ```

   You want a line under *Flight-controller serial ports* naming your board.

6. When you are done — or when you want to use **QGroundControl on Windows** —
   give the device back:

   ```powershell
   usbipd detach --busid 4-4
   ```

**Important:** while a device is attached to WSL, **Windows cannot use it**. So
QGroundControl (which you run on Windows to flash firmware and calibrate
sensors) will not find the board until you `detach`. Rule of thumb: *QGC first,
detached; then attach for the Linux scripts.*

Convenience: `usbipd attach --wsl --busid 4-4 --auto-attach` keeps re-attaching
the device on that bus ID if you unplug and replug it. Leave that command
running in its own PowerShell window.

### If the port appears but the scripts say "no permission"

Linux guards serial ports. One-time fix, then close and reopen your WSL
terminal:

```bash
sudo usermod -aG dialout $USER
```

Or, just for this session: `sudo chmod 666 /dev/ttyACM0`.

---

## 3. Raspberry Pi 5 + the OV9281 camera

The Pi is the seeker: it holds the camera and records the frames. It is a small
separate computer, so you set it up once and then reach it over Wi‑Fi.

### 3.0 The networking dry run — DO THIS BEFORE YOU PACK

**This is the most likely way to lose a whole field day with hardware that all
works.** Every capture is *started* by ssh'ing from the laptop to the Pi, and
`04_pull_logs.sh` fetches the session back the same way. At the field the only
network is your phone's hotspot, and three things have to be true that are not true
out of the box:

1. **The Pi has to join the hotspot.** The Pi Imager writes exactly ONE Wi‑Fi
   network — your home one. On site it will look for that, not find it, and never
   appear on the network.
2. **The laptop has to find it.** `interceptor-seeker.local` is *mDNS*, and **mDNS
   does not resolve from WSL2** (your Ubuntu sits behind a NAT with plain
   `files dns` name resolution). Use the **IP**.
3. **ssh has to work without a password.** Every script here uses
   `BatchMode=yes` — a password prompt inside a check script would just hang — so a
   key must exist on the laptop *and* be installed on the Pi.

Do all of it at the desk, in this order:

```bash
# 1. make an ssh key (once, no passphrase -- it lives on your own laptop)
ssh-keygen -t ed25519 -C interceptor-field -N "" -f ~/.ssh/id_ed25519

# 2. turn your PHONE HOTSPOT on, and add it to the Pi as a SECOND Wi-Fi network
#    (do this while the Pi is still on your home Wi-Fi, over ssh):
sudo nmcli connection add type wifi ifname wlan0 con-name field-hotspot \
    ssid '<HOTSPOT SSID>' wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<PASSWORD>'
sudo nmcli connection modify field-hotspot connection.autoconnect yes

# 3. reboot the Pi with the hotspot up, and read the IP it got
#    (on the Pi: `hostname -I`; or look at the phone's hotspot client list)

# 4. install the key on the Pi, then TELL THE SCRIPTS to use the IP
ssh-copy-id pi@192.168.43.17            # <- your Pi's actual hotspot IP
export FIELD_PI_HOST=pi@192.168.43.17   # put this in ~/.bashrc so it always applies

# 5. prove the whole chain, from the laptop, on the hotspot
scripts/field/05_pi_link_check.sh
scripts/field/01_camera_live_check.sh --pi "$FIELD_PI_HOST"
scripts/field/04_pull_logs.sh --pi "$FIELD_PI_HOST" --no-score
```

Step 5 is the point of the whole section: **`01` → `04` must have reached the Pi
from this laptop at least once, over the hotspot, before you drive anywhere.**

`05_pi_link_check.sh` checks the key, the address (and warns about `.local` under
WSL2), that the Pi has the repo + venv + `pi_capture.py` + `rsync`, that its
`sessions/` dir is writable, that there is disk space for a day of PNGs, and the
**laptop↔Pi clock offset** — because the hotspot is also the Pi's only NTP source,
and the frame↔log time join that produces every range number depends on it
(`docs/tripod_test_protocol.md` §6.1). It changes nothing on either machine.

**If it still cannot reach the Pi at the field:**

- Many hotspots have **AP/client isolation** on, which blocks laptop→Pi entirely.
  Turn it off in the phone's hotspot settings (test at the desk!).
- Fallback that always works: **an ssh app on the phone itself** — the hotspot host
  can always reach its own clients. Install one and try it once before the field.
- Pinning the name instead of exporting an IP: `echo '192.168.43.17 interceptor-seeker' | sudo tee -a /etc/hosts`
  inside WSL.

### 3.1 Prepare the microSD card (once)

1. Install **Raspberry Pi Imager** on Windows.
2. Choose **Raspberry Pi OS (64‑bit)**, Bookworm.
3. Open the Imager's **advanced/settings** gear **before** writing and set:
   **hostname `interceptor-seeker`**, a username + password, your **Wi‑Fi**, and
   **enable SSH**. This is what lets you log in with no monitor or keyboard.
4. Write the card, put it in the Pi, fit the **active cooler**, connect the
   camera ribbon (see below), then power the Pi from its own 27 W USB‑C supply.

**Camera ribbon:** the camera board has a **15‑pin** connector; the Pi 5 has a
**22‑pin** one. The included **22→15 adapter cable** is required — the *narrow
22‑pin end goes into the Pi*. Push the latch down until it clicks.

### 3.2 Set the Pi up (once)

From your laptop:

```bash
ssh pi@192.168.1.50        # the Pi's IP; use the username you set in the Imager
                           # (.local works from Windows/macOS, NOT from WSL2 -- see 3.0)
git clone <this repo> ~/interceptor-sim && cd ~/interceptor-sim
bash scripts/pi_setup/provision.sh --dry-run          # shows the plan, changes nothing
sudo bash scripts/pi_setup/provision.sh --hostname interceptor-seeker
sudo reboot
```

Full detail (what it changes and why) is in `scripts/pi_setup/README.md`.

### 3.3 Check the camera — one command, from the laptop

```bash
scripts/field/01_camera_live_check.sh --pi "$FIELD_PI_HOST"   # e.g. pi@192.168.43.17, see 3.0
```

**What a PASS looks like:** `frames: 30`, `resolution: 1280x800`, and
`exposure: applied ~<something ≤1000> us … -> OK`, then `PASS`. Open the saved
`sample_frame.png` it points at and check the picture is sharp and well lit.

Why 1280×800 and why ≤1 ms: that is the sensor's native size, and a very short
exposure is the whole reason we bought a *global shutter* camera — it freezes a
fast-moving target instead of smearing it. The script does not assume the
exposure worked; it reads back what the sensor actually applied.

**Top failure modes**

| Symptom | Fix |
|---|---|
| The command fails on the Pi, or `rpicam-hello --list-cameras` shows nothing | Ribbon is in backwards or not latched, or the overlay is missing. Re-seat the cable (22‑pin end at the Pi), then re-run `provision.sh` (it adds `dtoverlay=ov9281`) and reboot. |
| `exposure … OVER SPEC` or the picture is nearly black | Not enough light for a 1 ms exposure. Do this check in daylight or under a bright lamp, and raise `--gain` if needed. |

**No Pi yet?** Just run `scripts/field/01_camera_live_check.sh`. With no camera
anywhere it replays frames from the repo through the exact same recorder, tells
you the pipeline is healthy, and reports NOT CONNECTED.

Two things it deliberately will *not* do any more (both used to print PASS):

- a **laptop webcam** or the **replay fixture** now reports **NOT CONNECTED**, not
  PASS — neither is evidence the OV9281 works, and `--require` must not be
  satisfiable by them;
- a session that delivers **fewer frames than you asked for** now FAILs (it was
  graded against "at least 1 frame").

### 3.4 Recording an actual pass (not the health check)

`01` is a *health check*. A real tripod/field pass is recorded by running the
recorder on the Pi with the pass sized up front and the calibration + tag size
recorded into the session:

```bash
ssh "$FIELD_PI_HOST" '~/interceptor-sim/.venv-pi/bin/python \
  ~/interceptor-sim/scripts/seeker/pi_capture.py --source picamera2 \
  --out ~/interceptor-sim/sessions/pass01 --duration 20 --exposure-us 1000 \
  --width 1280 --height 800 --calib ~/interceptor-sim/calib.json \
  --tag-size 0.35 --run-tag pass01'
```

The `--calib` / `--tag-size 0.35` pair is what makes the session scorable later;
`docs/tripod_test_protocol.md` §6.0b has the per-pass sanity check to run on the Pi
before flying the next one.

---

## 4. The AprilTag placard check

An **AprilTag** is the black-and-white square marker on the target drone. It is
not a toy: the first real intercepts fly on the *tag*, because the tag decoder
runs fast enough on the Pi's own processor while the neural-net seeker needs the
add-on accelerator we deliberately deferred.

Print the tag first: **tag36h11, id 0, 0.35 m black-square edge** — which is a
**450 × 450 mm physical sheet** (the black square plus the mandatory 1-cell quiet
zone: 0.35 / 0.8 = 437.5 mm, plus margin) — glued to something rigid and flat
(`docs/placard_sizing.md`, `docs/placard_mount.md` §2). The two numbers are not
interchangeable: 0.35 m is what `--tag-size` means (it is what sets the pose), and
0.45 m is what you print and carry.

Then: capture with step 01, and score it:

```bash
scripts/field/01_camera_live_check.sh --pi "$FIELD_PI_HOST"
scripts/field/02_apriltag_desk_check.sh --expect-range 3.0     # 3.0 m tape-measured
```

**What a PASS looks like:** `decoded: 30 (100.0% of frames)`, a tag id, a
`tag in frame: ~NNN px per edge` line, and `PASS`.

About **range**: the tool prints the distance implied by the tag's pose, but it
marks it *INDICATIVE ONLY* until you have calibrated **this** camera. Camera
calibration means measuring the lens's real focal length from photos of a
checkerboard — without it, every distance is wrong by an unknown factor:

```bash
# on the Pi, capture the views (headless, ssh-friendly, reports coverage per shot):
.venv-pi/bin/python scripts/seeker/capture_calib_views.py --out ~/calib_views
# ENTER grabs · s skips · q finishes + verdict. Then solve:
scripts/calibrate_camera.py --images '/home/admin/calib_views/calib_*.png' \
    --cols 9 --rows 6 --square 0.025 --out camera_intrinsics_real.json
```

Once `camera_intrinsics_real.json` exists in the repo root, step 02 picks it up
automatically and starts *grading* range against your tape measure.

#### How far from the board to stand — GET CLOSE (derived 2026-07-31, bench session)

The 118° lens is wide enough that intuition is wrong here by a factor of several,
and the failure is silent: you shoot twenty views, every one of them lands in the
FAR bucket, and `capture_calib_views.py` refuses READY at the end of the session.
Frame width = `2·tan(59°)·d` = **3.33·d**, so a board's share of frame AREA falls
as 1/d². For the checkerboard displayed on a screen
(`hardware/prints/checkerboard_9x6_screen.html`, squares = `min(9.2vw, 12.4vh)`):

| display | board | NEAR (≥25 %) | MID (8–25 %) | FAR (<8 %) |
|---|---|---|---|---|
| 13" laptop 16:9 | 20×14 cm | 13 cm | 13–23 cm | >23 cm |
| 15.6" laptop 16:9 | 24×17 cm | 15 cm | 15–27 cm | >27 cm |
| 24" monitor 16:9 | 37×26 cm | 24 cm | 24–42 cm | >42 cm |
| 27" monitor 16:9 | 42×29 cm | 27 cm | 27–47 cm | >47 cm |

**Use the largest display available.** Not for convenience — for FOCUS. The lens
is ~1.15 mm focal length (118° HFoV across the OV9281's 3.84×2.40 mm sensor), so
with CoC = diag/1500 = 0.0030 mm the hyperfocal distance is **≈22 cm at f/2**
(≈16 cm at f/2.8). A lens focused for FIELD distances is therefore sharp only down
to ~16–22 cm: on a 15.6" laptop the NEAR views sit at 15 cm, *inside* the blur
limit, and come back NOT FOUND. On a 24"+ screen every bucket clears it.

The same number carries the good news: **do not refocus the lens to shoot close.**
Depth of field at this focal length runs from ~20 cm to infinity, and `skr-06`
requires the intrinsics measured with the lens exactly as tripod day will use it —
refocusing between calibration and the field invalidates the calibration.

A printed board is no larger than a laptop screen at these square sizes
(A3 @ 25 mm = 25×17.5 cm), so printing does not solve the distance problem;
only a physically bigger board or a bigger display does.

**Top failure modes**

| Symptom | Fix |
|---|---|
| `only 0 frame(s) decoded` | Get closer, or print a bigger tag; make sure the tag is sharp and in focus (turn the lens ring), evenly lit, not glossy, not curled. |
| Decodes fine but the range is way off | You are using someone else's camera numbers. Run the calibration above. Also check you passed the real printed size with `--tag-size`. |

This is a sanity check, not the real measurement. The proper decode-range curve
— the one that decides whether to order the interceptor airframe — is
`scripts/seeker/tripod_score.py`, run on a tripod session.

---

## 5. Flight controller on the bench (props OFF)

Two boards, same procedure: the **Pixhawk 6C Mini** (interceptor, PX4) and the
**Kakute H7** (target, ArduPilot).

### 5.1 First, on Windows: flash and calibrate

Do this in **QGroundControl** with the board **detached** from WSL (§2):

- **Pixhawk 6C Mini:** *Vehicle Setup → Firmware →* **PX4 Flight Stack (stable)**.
  QGC detects the board as **FMUv6C**. Then *Airframe →* **Generic Quadcopter**,
  Apply & reboot. Put a **microSD** in it — the flight logs are our evidence.
  Step-by-step: `configs/px4_6cmini/README.md`.
- **Kakute H7 (target):** ArduPilot, firmware target **`KakuteH7`** — *not* the
  v2 target. It also needs its own microSD or there is no target log.

### 5.2 Then, in Ubuntu: the link check

```bash
scripts/field/03_fc_bench_check.sh
```

**Props off. This script cannot arm the vehicle** — it only reads. That is not a
promise in a comment: at startup it parses its own source and refuses to run if it
so much as *mentions* a plugin that could command the aircraft — a call, an alias
(`a = drone.action`), an import, or a dynamic `getattr` on the vehicle handle. Six
injected attempts are tested against it in `scripts/field/selftest.sh`.

Scope, stated honestly: that audit covers **this script's own source**, not
everything it could import, and **props off is still the physical interlock.**

**What a PASS looks like:**

```
[fc-link] safety audit OK: ... -- this tool CANNOT arm the vehicle
[fc-link] connected -- heartbeat is alive
[fc-link] armed         : False   <- this tool never changes it
[fc-link] attitude      : roll +0.3 deg  pitch -1.1 deg  yaw +87.0 deg
[fc-link] health readout (read-only):
[fc-link]   OK   gyrometer calibration
...
[03-fc] PASS
```

Tilt the board while it runs — the attitude numbers must move sensibly. That is
your proof the sensors are alive, not just the cable.

`--  global position` will be missing indoors. That is normal: no GPS lock in
the house.

**Top failure modes**

| Symptom | Fix |
|---|---|
| `no serial port found` | The WSL hand-over (§2) has not been done, or the cable is charge-only. |
| `no MAVLink came back` | Something else already owns the port — usually **QGroundControl still open on Windows**. Close it (and `usbipd detach`/`attach` again). Or the board has no firmware yet: flash it in QGC first. |

### 5.3 The next gate after this one

This step proves the *link*. The gate that proves the **autonomy path** —
PX4 accepting streamed guidance commands, still props-off and disarmed — is:

```bash
scripts/check_deploy_bench.sh --check-only     # no hardware: validates the plumbing
scripts/check_deploy_bench.sh                  # the real props-off run
```

Load `configs/px4_6cmini/bench.params` in QGC first, and run it near a window —
that check needs a GPS fix. The pure-software rehearsal of the same code against
the simulator is `scripts/check_deploy_sitl.sh`.

---

## 6. After a flight — collect everything

```bash
scripts/field/04_pull_logs.sh
```

Put both microSD cards in the reader (they appear as Windows drives, so no
usbipd needed) and have the Pi powered on. The script copies:

- the interceptor's PX4 log (`.ulg`) from its card,
- the target's ArduPilot log (`.BIN`) from its card,
- the newest camera session from the Pi,

into one folder under `logs/field/`, and — when both flight logs are present —
runs `scripts/field_score.py`, which computes how close the two aircraft got and
calls KILL or MISS against a lethal radius.

Point it at things explicitly if the auto-scan misses:

```bash
scripts/field/04_pull_logs.sh --px4 /mnt/d --ardupilot /mnt/e \
    --pi "$FIELD_PI_HOST" --session tripod_pass01
```

The **video is still the verdict** (the project's success test is a binary kill
you can see); the score is the number behind it.

**What its verdict means (it changed 2026‑07‑25):**

- **PASS** — every source you asked for came off the aircraft.
- **NOT CONNECTED** — you did not put a card in / the Pi was off. Advisory.
- **FAIL** — a source you *asked for* did not copy, **or** the scorer crashed.
  Either way **everything already copied is safe** in the run folder and nothing
  here ever deletes a source: fix the cause and re-run with the same `--session`.
  **Do not wipe the cards on a FAIL.** (Before this change, a scoring crash printed
  `PASS` and exited 0 — the shape of failure this whole pack exists to prevent.)

The kill/miss threshold is `field_score.py`'s `DEFAULT_LETHAL_RADIUS_M`; read the
mechanism note at that constant rather than any number quoted in a runbook, and
override with `--lethal-radius` (04 forwards it).

**Scoring the tripod session is a different, three-step job** — decode →
`range_truth_join.py` → re-score. Do not point `tripod_score.py` at a session and
expect a verdict: `docs/tripod_test_protocol.md` §7.0 has the exact commands (and
which of the two venvs each one needs).

---

## 7. Order of operations when the boxes land

1. `scripts/field/bringup.sh` on the laptop — proves the software half. (Works
   today, with nothing attached.)
2. Flash the Pi card, provision it, **then do the networking dry run → §3.0 /
   step 05**. Do this the same day the Pi boots, not the night before the field.
3. Check the camera → **step 01**.
4. Print the tag, check the decode → **step 02**. Calibrate the camera.
5. Flash the flight controllers in QGC, then the link check → **step 03**.
6. Props-off OFFBOARD gate → `scripts/check_deploy_bench.sh`.
7. Build, bench, fly (`docs/project_state.json` → `build_tab` has the full
   step list per subsystem, including the safety gates).
8. After every flight → **step 04**.

Safety, non-negotiable: props off for every bench test; first power-up of a
soldered board through the smoke stopper; batteries charged inside the fireproof
bag, attended; glasses on. The build tab lists these as hard gates for a reason.

---

## 8. Where the numbers in this doc come from

- Camera part, 1280×800, ~118° horizontal field of view, ≤1 ms exposure:
  `docs/camera_paper_check.md`; recorder + spec constant:
  `scripts/seeker/pi_capture.py`.
- Placard 0.35 m, tag36h11: `docs/placard_sizing.md`.
- Boards, cards, wiring, and the per-subsystem step lists:
  `docs/project_state.json` (`build_tab`), `docs/hardware_order_list.md` §0c/§0d.
- PX4 flashing, bench parameters, and the OFFBOARD gate:
  `configs/px4_6cmini/README.md`, `scripts/check_deploy_bench.sh`.
- Pi provisioning: `scripts/pi_setup/README.md`.
- WSL USB pass-through commands:
  <https://learn.microsoft.com/en-us/windows/wsl/connect-usb>
  (verified 2026‑07‑24; `usbipd` 5.x command set: `list` / `bind` / `attach
  --wsl` / `detach`).
- Scoring a real flight: `scripts/field_score.py`.

One thing this doc deliberately does **not** state: the exact USB
vendor/product IDs of the two flight controllers. Nothing in the repo records
them yet, so the scripts identify boards by their USB *name* strings instead.
When you first plug each board in, run `lsusb` (`sudo apt install usbutils`) and
paste the two lines here — after that the detection can be exact.
