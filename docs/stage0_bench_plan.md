# Stage 0 Hardware Bench Plan — measure the real perception gap before buying an airframe

> **Status:** execution plan for post-M5 queue item 1 (ADR-0033). Not started.
> **Scope:** ADR-0012 "Stage 0" — a ~$257 Raspberry Pi 5 + camera rig (design-review
> §6 re-cost of ADR-0012's original ~$230 figure) that runs
> the project's *real* detection code against a *printed* AprilTag and produces a
> measured **sim-vs-bench gap table**. This is the cheapest possible go/no-go on the
> whole hardware plan: it can save the airframe spend if the numbers say the
> camera-only terminal window collapses on real hardware.
> **What this is NOT:** no flight, no PX4, no airframe, no MAVSDK link. Perception
> only. (ADR-0012 Stage 0; ADR-0015 build-plan step 4.)

---

## 0. Why this bench exists (the one-paragraph "why")

Every miss-distance and Pk (probability-of-kill) number this project has produced
was measured against a **perfect fiducial** in a sim that renders **zero** motion
blur, zero lens distortion, zero vibration, and ideal contrast (it even needed an
emissive-lighting hack just to get the tag bright enough — ADR-0007). The council's
unanimous #1 risk (ADR-0012, sharpened in ADR-0015 #5) is that a **real** lens +
real blur + a slower embedded CPU + uncontrolled outdoor light shrink the already
marginal camera-only detection window below viability. **No BOM dollar fixes that
risk — only a measurement retires it.** Stage 0 is that measurement. It replaces
three "assumed" sim knobs (ADR-0015 data-constraints table, rows 7–9) with numbers
taken on real hardware, and it settles the single biggest open unknown from
ADR-0012: **the Raspberry Pi 5 detection rate for this exact detector is UNMEASURED.**

*New term — fiducial:* a purpose-built visual marker (here the AprilTag) that is
easy and robust to detect. In the real system it stands in for a hard classical-CV
seeker lock; on the bench it lets us measure the *camera + detector + CPU* chain
without also fighting the perception problem.

---

## 1. Parts list (2026 pricing, US resellers)

All prices are USD and current as of **July 2026**. **Read the lead-time note first**
— the Pi 5 price is the volatile line item.

### 1.1 The compute + power core (identical for both carts)

| Part | Why | Price | Source |
|---|---|---|---|
| **Raspberry Pi 5 (8 GB)** | The ADR-0012 companion computer. 8 GB gives headroom toward the later ADR-0015 Hailo/ML stage; **4 GB is plenty for Stage 0** (pyapriltags is CPU-only) — see budget cart. | **~$130** (Apr 2026 official) — **volatile $120–205** | [rpi 5 product](https://www.raspberrypi.com/products/raspberry-pi-5/), [price-rise notice](https://www.raspberrypi.com/news/more-memory-driven-price-rises/), [Tom's Hardware (16 GB → $205)](https://www.tomshardware.com/raspberry-pi/raspberry-pi-5-price-increases-drastically-as-ai-shortage-bites-16gb-version-now-usd205-second-price-increase-in-three-months-over-70-percent-more-expensive-than-original-msrp) |
| **Official Active Cooler** | The Pi 5 throttles under sustained load (a 30–60 s detection-Hz run is exactly that). $5, clips straight on. | **$5** | [rpi Active Cooler](https://www.amazon.com/Raspberry-Pi-Active-Cooler/dp/B0CLXZBR5P) |
| **Official 27 W USB-C PSU** | The Pi 5 needs the 5.1 V/5 A profile to power the board + a USB camera without brownouts. Do not substitute a phone charger. | **~$12–14** | [rpi 27 W PSU](https://www.raspberrypi.com/products/27w-power-supply/), [Adafruit 5814](https://www.adafruit.com/product/5814) |
| **microSD, 32 GB, A2/V30** | OS + venv + logs. A2 rating matters for OS responsiveness. Official card is ~$15; a generic SanDisk Extreme 32 GB A2 is ~$8–10. | **~$9–15** | [rpi SD cards](https://www.raspberrypi.com/products/sd-cards/), [Adafruit 6010](https://www.adafruit.com/product/6010) |

Core subtotal: **~$156–164** (8 GB Pi at ~$130).

### 1.2 Camera options — the real decision

The sim camera is **1280×960, fx = fy = 539.9 px, HFOV ≈ 99.7°** (= 1.74 rad; ADR-0012).
That HFOV is the number to match. Two hard requirements carry over from ADR-0012:

- **Global shutter is MANDATORY.** *New term — global vs rolling shutter:* a global
  shutter exposes every pixel at the same instant; a rolling shutter scans row-by-row,
  so a fast pan **skews** the image. The strapdown seeker yaws at up to 60 °/s
  (`YAWSPEED_MAX_DEG_S`), and rolling-shutter skew corrupts the exact tag corners the
  pose solve needs — in the terminal phase where the sim is already marginal (ADR-0012 #3).
- **Grayscale is the native input.** Detection already runs on gray
  (`image_msg_to_gray` in the sim; `cvtColor(...BGR2GRAY)` in `OpenCVFrameSource`), so a
  **mono** sensor is a strict match — no debayering artifacts.

A key optics fact that makes matching easy: **HFOV = 2·atan(W_sensor / (2·f))**, and for
*any* 1280-px-wide sensor set to 99.7° HFOV, the horizontal focal length works out to
**fx = 640 / tan(49.85°) ≈ 540 px** — the same 539.9 the sim uses — **regardless of the
physical sensor size.** So if you match HFOV and horizontal resolution, the *pixel-domain*
detection geometry (how many pixels the tag spans at a given range) is identical to the
sim. That is the whole trick to making the bench numbers comparable.

| Option | Sensor / shutter | Native FOV | Lens to hit ~99.7° HFOV | Code seam | Price | Source |
|---|---|---|---|---|---|---|
| **A. Arducam OV9281 USB (B0332)** ← *primary* | 1/4″ OV9281 **mono, global**, 1280×800, 100 fps@720p, UVC | ships **70° (H)** M12 lens | needs a wider M12 lens (**~1.6 mm**, see math) | **USB/UVC → zero code change** (`cv2.VideoCapture(0)`) | **~$60–70** (€56.90) | [Arducam B0332](https://www.amazon.com/Arducam-Distortion-Microphones-Computer-Raspberry/dp/B096M5DKY6), [welectron €56.90](https://www.welectron.com/Arducam-B0332-120fps-Global-Shutter-USB-Camera-Board_1), [datasheet PDF](https://www.uctronics.com/download/Amazon/B0332_OV9281_Global_Shutter_UVC_Camera_Datasheet.pdf) |
| **B. RPi Global Shutter Camera** | 1/2.9″ IMX296 **color, global**, 1456×1088, 3.45 µm | **no lens** (C/CS mount) | needs a wide CS or M12-via-adapter (~2.1 mm) lens, bought separately | CSI → **needs a Picamera2 shim** (see 2.3) | **$50** body **+ lens** | [rpi GS Camera](https://www.raspberrypi.com/products/raspberry-pi-global-shutter-camera/), [Seeed](https://www.seeedstudio.com/Raspberry-Pi-Global-Shutter-Camera-p-5591.html) |
| **C. Arducam IMX296 M12 module (B0444 mono / B0445 color)** | 1/2.9″ IMX296 **global**, M12 mount | ships **45° (H)** M12 lens | needs a wider M12 lens (**~2.1 mm**) | CSI → **needs a Picamera2 shim** | **~$75** (€74.90) | [Arducam IMX296 M12](https://www.arducam.com/1-58mp-imx296-color-global-shutter-camera-module-with-m12-lens-for-raspberry-pi.html) |
| **D. RPi Camera Module 3 Wide** ← *budget fallback* | 1/2.43″ IMX708 **color, ROLLING** | **120° diag ≈ ~102° HFOV** (best native FOV match) | none needed | CSI → **needs a Picamera2 shim** | **$35** | [rpi CamMod3](https://www.raspberrypi.com/products/camera-module-3/), [product brief PDF](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf) |
| Wide M12 lens (for A / C) | low-distortion M12, ~1.6 mm (OV9281) or ~2.1 mm (IMX296) | — | — | — | **~$15–25** | [Arducam M12 lenses](https://www.arducam.com/camera-component/lenses/m12-lens-arducam.html), [M12 lens set 20–180°](https://www.amazon.com/Arducam-Telephoto-Fisheye-Cleaning-Optical/dp/B096V2NP2T) |

**Focal-length math (so you buy the right lens):**
- **OV9281**, 1/4″, W = 1280 × 3.0 µm = **3.84 mm**. Stock 70°(H) lens ⇒ f ≈ 1.92/tan(35°) ≈ **2.74 mm**. For 99.7° ⇒ f = 1.92/tan(49.85°) ≈ **1.62 mm**.
- **IMX296**, 1/2.9″, W = 1456 × 3.45 µm = **5.02 mm**. Stock 45°(H) lens ⇒ f ≈ 2.51/tan(22.5°) ≈ **6.06 mm**. For 99.7° ⇒ f = 2.51/tan(49.85°) ≈ **2.12 mm**.

> ⚠️ **Camera Module 3 is rolling-shutter (IMX708)** — it **fails** ADR-0012's global-shutter
> mandate. It is the cheapest and the closest native FOV match, so it is a genuine
> *budget* option for the **static** measurements (§3a/b/d), but its rolling-shutter skew
> **confounds the yaw-rate/motion-blur test (§3c, row 7)** — that number is not comparable
> and must come from a global-shutter camera. Say so in the gap table if you use it.

### 1.3 Recommended carts

**PRIMARY (recommended) — Arducam OV9281 USB, ~$240:**

| Line | Price |
|---|---|
| Pi 5 8 GB | ~$130 |
| Active Cooler | $5 |
| 27 W PSU | ~$14 |
| 32 GB microSD | ~$9 |
| Arducam OV9281 USB (B0332) | ~$65 |
| Wide ~1.6 mm M12 lens | ~$18 |
| **Total** | **~$241** (Pi-price-dependent: $230 → $310); + the $15 yaw-ramp jig the design review (§6, 2026-07-10) moved into this cart → **adopted cart ~$257** |

Why the OV9281 USB is the primary despite needing a lens swap:
1. **USB/UVC is the smallest possible code change — literally none.** `OpenCVFrameSource`
   (`scripts/frame_source.py`) passes its `device` argument straight into
   `cv2.VideoCapture` (line 141). For a UVC webcam that is just `device=0`, and the
   `opencv-python(-headless)` wheel opens UVC devices out of the box. This is exactly the
   seam the abstraction was built for. **ADR-0012 named "USB/UVC = smaller code change"
   as the reason to prefer the OV9281.** The CSI options (B/C/D) all need a small extra
   frame-grab shim (see §2.3) because the pip OpenCV wheel cannot open a libcamera CSI
   device — real integration cost the bench should avoid.
2. **Mono global shutter** = strict grayscale match, satisfies the global-shutter mandate,
   and is the only class of camera that can run the motion-blur test (§3c) honestly.
3. **M12 mount = tunable FOV.** Swapping to a ~1.6 mm lens hits ~100° to match the sim, and
   the stock **70° lens is a free bonus experiment**: it's a narrower/longer-range lens, so
   running the sweep on both directly measures the FOV-vs-range trade ADR-0024 theorized
   but never checked on hardware.
4. OV9281 is exactly **ADR-0012's option A**.

**BUDGET FALLBACK — Camera Module 3 Wide, ~$140:**

| Line | Price |
|---|---|
| Pi 5 4 GB (sufficient for CPU pyapriltags) | $75 |
| Active Cooler | $5 |
| 27 W PSU | ~$14 |
| 32 GB microSD | ~$9 |
| RPi Camera Module 3 Wide (120° diag) | $35 |
| RPi 22-pin ↔ 15-pin camera cable (Pi 5 uses the narrow connector; CamMod3 ships the old wide cable) | ~$1–6 |
| **Total** | **~$140–145** |

Use the budget cart to prove out the pipeline and get **static** detection-rate,
range/bearing accuracy, detection-Hz, and lighting numbers cheaply (rows 8/9 + Hz + the
lighting caveat) with the best native FOV match — then borrow/buy a global-shutter camera
for the one number it can't give you (§3c). Sources for the 4 GB Pi price:
[PiShop 4 GB $75](https://www.pishop.us/product/raspberry-pi-5-4gb/).

### 1.4 Lead-time reality (read this)

- **The Pi 5 is the schedule and budget risk, not the camera.** A DRAM/AI-fab shortage has
  driven the price up repeatedly through 2025–2026 (8 GB ~$80 MSRP → ~$130 Apr 2026 → up to
  ~$199 later; 16 GB hit $205) and the official notice warns of further rises
  ([price-rise notice](https://www.raspberrypi.com/news/more-memory-driven-price-rises/),
  [Tom's Hardware](https://www.tomshardware.com/raspberry-pi/raspberry-pi-5-price-increases-drastically-as-ai-shortage-bites-16gb-version-now-usd205-second-price-increase-in-three-months-over-70-percent-more-expensive-than-original-msrp)).
  **Order the Pi first, check live stock at [rpilocator.com](https://rpilocator.com), and
  don't wait for a price dip that may not come.** (The design review §6's adopted cart —
  official-reseller Pi + the yaw-ramp jig moved in — is ~$257, the figure now quoted
  project-wide.) The ADR-0012 "~$230 Stage 0" figure assumed
  the pre-shortage Pi; at today's price the primary cart is ~$240–310. That is still an order
  of magnitude below buying the X500 airframe (~$260–530, ADR-0012) that this bench exists to
  de-risk.
- **Cameras are routinely in stock** at Arducam / Amazon / PiShop / Adafruit; the discontinued
  100°-diagonal OV9281 (UB0232) is gone — use **B0332 + a wide lens**, not the old SKU.
- **Global-shutter camera bodies never ship a lens matched to 99.7°** off the shelf, so a
  wide M12 lens (or the Arducam 20°–180° M12 lens set) is a mandatory add for A/C.

---

## 2. Bring-up procedure

### 2.1 OS

Flash **Raspberry Pi OS Bookworm, 64-bit (aarch64)** with the Raspberry Pi Imager. The
64-bit build is **required** — `pyapriltags` only ships 64-bit `aarch64` wheels, and a
64-bit Pi OS reports `aarch64` ([pyapriltags on PyPI](https://pypi.org/project/pyapriltags/)).
In the Imager's advanced options, set the hostname, enable SSH, and set Wi-Fi so you can run
headless over SSH.

### 2.2 Python venv + the detector (the aarch64 install path)

The project's `requirements.txt` pins `pupil-apriltags`, which has **no Linux aarch64 wheel**
(PyPI is x86_64 + macOS-arm64 only; ADR-0003/0012) — so on the Pi you install the drop-in fork
`pyapriltags` instead. The portable import in `scripts/apriltag_detector.py` already tries
`pupil_apriltags` first and **falls back to `pyapriltags`**, so nothing downstream changes; it
logs `DETECTOR_BACKEND` so a run's log records which detector produced its numbers.

```bash
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
# Install deps individually — do NOT `pip install -r requirements.txt` on the Pi:
# it pins pupil-apriltags, which has no aarch64 wheel and will fail (ADR-0012).
.venv/bin/pip install numpy opencv-python pyapriltags
```

- Both `opencv-python` and `pyapriltags` ship `aarch64` wheels, so this is a wheel install,
  not a from-source build ([pyapriltags PyPI](https://pypi.org/project/pyapriltags/)).
- `requirements.txt` uses `opencv-python-**headless**`, which has **no `cv2.imshow`**. The live
  calibration GUI (`calibrate_camera.py --live`) needs a window, so either install full
  `opencv-python` **or** use the headless `--images` calibration mode (§2.4). Pick one now.
- `mavsdk` is not needed for Stage 0 (no flight).

### 2.3 Wire the camera to `frame_source.py` — the exact seam

The whole point of `scripts/frame_source.py` is that detection + guidance only ever ask "the
camera" for two things: a stream of gray frames and the pinhole intrinsics `(fx, fy, cx, cy)`.
The hardware implementation is `OpenCVFrameSource`. The seam is its constructor:

```python
# scripts/frame_source.py, line ~118
OpenCVFrameSource(device, intrinsics_path, *, width=None, height=None,
                  fourcc=None, undistort=True)
```

- **USB/UVC (primary, OV9281):** `device = 0` (or `/dev/video0`). It is passed unmodified into
  `cv2.VideoCapture` (line 141). After construction call `check_resolution()` (line 186) — it
  **asserts the capture resolution equals the calibration resolution**, because a mismatch
  silently rescales every tag range. **This path needs no code change.**
- **CSI/libcamera (options B/C/D):** the pip OpenCV wheel is **not** built with GStreamer, so
  `VideoCapture` cannot open a libcamera CSI device by index. Two routes, both extra work the
  bench plan flags but does **not** implement here: (a) build OpenCV with GStreamer and pass a
  `"libcamerasrc ! ... ! appsink"` **pipeline string** as `device` (the constructor docstring,
  lines 138–141, already anticipates a pipeline string); or (b) add a small `Picamera2`-backed
  `FrameSource` subclass that grabs a NumPy frame and reports the same `Intrinsics`. This is the
  concrete reason the **USB** camera is the primary: it exercises the existing seam untouched.
- **Distortion:** `OpenCVFrameSource` auto-undistorts when the calibrated `dist_coeffs` are
  non-negligible (`has_distortion`, lines 64–70; remap built at line 135), which keeps the
  distortion-free pinhole `(fx,fy,cx,cy)` valid for pyapriltags' pose solve. A wide M12 lens
  **will** have real distortion, so this path matters — it is why calibration (next) is not
  optional.

### 2.4 Camera calibration (replace the sim's fx = 539.9 with the real one)

*New term — camera intrinsics:* the pinhole numbers `fx, fy` (focal length in pixels) and
`cx, cy` (image center) that convert tag pixels into a metric pose. The sim baked in
`fx = 539.9` for its ideal camera; **that number is meaningless for any real lens**, and flying
it would scale every range by `f_true/539.9` — a silent, total failure (`calibrate_camera.py`
header). So you measure the real intrinsics with a chessboard.

**Chessboard print spec:** a standard OpenCV checkerboard. Give the script the **inner-corner**
count, not the square count — a 10×7-square board has **9×6 inner corners** (`--cols 9 --rows 6`).
Print on rigid backing (foamboard/clipboard) so it stays **dead flat** — a curled board poisons
the calibration. Suggested: ~25 mm squares on Letter/A4, or larger for outdoor captures.

**Print-scale verification (do not skip):** consumer printers scale 2–5%. After printing,
measure one square (better: measure across N squares and divide) with calipers or a steel ruler,
and pass the **measured** size in metres as `--square` — not the nominal.

```bash
# Live mode (needs a display + full opencv-python):
.venv/bin/python scripts/calibrate_camera.py --live 0 \
    --cols 9 --rows 6 --square 0.025 --out camera_intrinsics_real.json
# Headless: capture ~20 stills first (any tool), then:
.venv/bin/python scripts/calibrate_camera.py --images 'calib/*.png' \
    --cols 9 --rows 6 --square 0.025 --out camera_intrinsics_real.json
```

- Capture **15–20 views** at varied angles, tilts, and distances (fill the frame corners).
- Watch the printed **RMS reprojection error**: the script warns if it is **> 1.0 px** — recapture
  until it is ≤ 1.0 px before trusting any range.
- **Calibrate at the SAME resolution you will capture at.** `check_resolution()` enforces this;
  a calibrate/capture resolution mismatch corrupts the pose scale (lines 186–199).
- The output JSON matches `camera_intrinsics.json`'s format and feeds `OpenCVFrameSource` directly.
- **If you swap the M12 lens (70° → ~1.6 mm), re-calibrate** — intrinsics belong to the *lens*,
  not the body.

### 2.5 Tag print spec (and why the size is a deliberate choice)

- **Family / ID:** **tag36h11, id 0** — the exact tag the sim uses (`TAG_FAMILY = "tag36h11"`,
  `scripts/make_tag_texture.py` builds `tag36h11_00000.png`). Download the canonical PNG from
  AprilRobotics/apriltag-imgs and scale it up with **nearest-neighbor** (never smooth) so the bit
  cells stay crisp.
- **`tag_size` = the black-square edge, and it scales pose linearly.** In the sim
  `TAG_SIZE_M = 0.5` m — that is the **black square** (the outer white quiet-zone is extra). Set
  the detector's `tag_size` / `--tag-size` to the **caliper-measured** black-square edge of your
  print; a 2% measurement error biases *every* range by 2% (ADR-0012 port-gap note).
- **Which size to print — tie it to the range band, via the tag's pixel width.** Detection depends
  on how many pixels the tag spans: `w_px ≈ fx · tag_size / range ≈ 540 · tag_size / range` (using
  the matched fx ≈ 540). The sim's validated points on the wide lens:
  - M2 gate point: 0.5 m tag at ~4.9 m ⇒ ~**55 px** wide (detection_rate 1.000, mean err 0.086 m).
  - M2 envelope edge: ~**6 m** ⇒ ~**45 px** wide (`~6 m` figure, ADR-0007 / M2 envelope).

  A **smaller** printed tag reproduces the *same pixel geometry* at a **proportionally shorter,
  bench-friendly** range. Pick tag size to put the sim's band where you have room to test:

  | Print black-square | Reproduces sim gate (~55 px) at | Reproduces sim edge (~45 px) at | Good for |
  |---|---|---|---|
  | **0.10 m** | ~0.98 m | ~1.20 m | indoor bench (short room) |
  | **0.20 m** | ~1.96 m | ~2.40 m | indoor / garage |
  | **0.30 m** | ~2.95 m | ~3.60 m | hallway / outdoor short |
  | **0.50 m** (= sim) | ~4.9 m | ~6.0 m | outdoor, literal sim band |

  **Recommendation:** print **two** tags — a **0.10 m** for indoor pixel-matched runs and a
  **0.30–0.50 m** for outdoor runs that span the sim's literal 5–6 m band. Report which tag/range
  pairing produced each row.
- **Contrast & mounting:** matte black on white, laser print, "Actual size" (100%, no fit-to-page),
  a white quiet zone ≥ 1 cell all around, mounted flat and rigid. ADR-0007 is the warning here: the
  *sim* needed an emissive hack just to get enough contrast for the small on-screen cells at range —
  real ink under real light is precisely the untested thing this bench measures (§3d).

---

## 3. Measurement protocol (mirror the sim gates so numbers are comparable)

Run each test through the **same detector + pose chain** the sim gates use
(`apriltag_detector.Detector` → `detect(estimate_tag_pose=True, camera_params=(fx,fy,cx,cy),
tag_size=...)`), logging one CSV row per frame (`detected, meas_x/y/z, decision_margin, +
tape-measured GT`) — the same schema as `m2_detect.py` / `check_m2_envelope.py`, so analysis reuses
the existing pattern. **The sim's ground truth (`/world/apriltag/pose/info`) does not exist on
hardware (ADR-0012); the external reference is a tape/laser-measured range** — that substitution *is*
the methodology gap ADR-0012 flagged, so measure carefully and note it.

Run all tests at **quad_decimate = 2** (the sim's terminal setting — ADR-0014 warns the Pi may need
*more* decimation, not less) and also at **1**, so the Hz-vs-accuracy trade is on record.

### (a) Static detection rate + pose error vs tape-measured range

At **3–5 ranges** spanning the sim's validated band (scaled by your tag size, §2.5): mount the tag
flat, facing the camera, on the boresight; tape/laser-measure camera-to-tag distance (GT); sample
~100 frames; compute **detection rate**, **mean |meas_range − GT|**, and **lateral (bearing) error**.

- **Sim baseline to beat:** detection_rate **1.000**, mean pose err **0.0861 m** at ~4.9 m
  (X,Y matched GT to <5 mm — ADR-0006/0007; `logs/m2_detect_20260707T192233Z.csv`). M2 gate thresholds:
  detection_rate ≥ **0.90**, mean err ≤ **0.25 m**.
- **PASS** — detection_rate ≥ 0.90 and range err ≤ 0.25 m at the sim-equivalent range → matches M2,
  the fiducial assumption holds. **INTERESTING** — detection holds but err 0.25–0.5 m → real accuracy
  is worse than the sim's clean fiducial; **widen sim range noise** (row 8). **FAIL** — detection_rate
  < 0.90 at the sim-equivalent range or err > 0.5 m → the sim is optimistic; the acquisition-range /
  P_detect(R) model (row 10) and the M2 envelope (~6 m) need discounting.
- **Feeds:** max reliable range → **acquisition range (row 10)** + the M2 envelope; range err →
  **row 8** (terminal range sigma; sim assumes AprilTag ~5–8%); lateral err → **row 9** (bearing sigma;
  sim assumes sub-degree).

### (b) Sustained detection Hz on-target

Hold the camera on a static tag at a mid-band range; run the detect loop **30–60 s**; log per-frame
timestamps; report **median** and **p10** Hz at quad_decimate 1 and 2.

- **This is the headline unknown ADR-0012 could not answer** ("Pi-5 detection rate for this exact
  detector is UNMEASURED; ~15 Hz plausible but must be BENCHMARKED"). Every filter gain, ACQUIRE
  streak, and terminal range in the guidance was tuned to the sim's ~**14 Hz** desktop cadence
  (ADR-0009); if the Pi is slower they don't port unchanged.
- **PASS** — ≥ ~15 Hz → the ADR-0009/0010 timing constants' cadence assumption survives.
  **INTERESTING** — 8–15 Hz → re-tune terminal timing constants, or raise quad_decimate / drop
  resolution. **FAIL** — < 8 Hz → the ADR-0012 #1 risk realized on the CPU path; forces heavier
  decimation or validates the ADR-0015 Hailo/ML rethink.
- **Feeds:** the **row-7** baseline detect rate and the ADR-0009 cadence constants.

### (c) Yaw-rate / motion-blur test (the row-7 high-LOS-rate dropout knob)

*New term — LOS rate (λ̇):* the angular rate at which the line-of-sight to the target sweeps. It spikes
as range → 0 near closest approach, and the seeker must yaw to keep the tag in frame. Mount the camera
on something you can rotate at a **measured** angular rate (a turntable, a servo with a known rate, or a
hand-pan past angle markers with a phone/IMU logging °/s); keep the tag at a fixed mid-band range; **ramp
the yaw rate up and find the °/s at which detection rate falls below ~50%.** Log the camera exposure —
a shorter exposure (global shutter + short exposure, ADR-0015) raises the trackable rate.

- **Sim context:** the sim models **no blur at all**. Its yaw clamp is 60 °/s (`YAWSPEED_MAX_DEG_S`),
  and the required LOS rotation near CPA hits **32–58 °/s** (ADR-0014/0023). So the bench must show
  detection **survives to at least ~60 °/s** for the sim's zero-blur assumption to be safe.
- **Global shutter is why this test is valid.** On the rolling-shutter budget camera the skew *itself*
  corrupts corners — this number is **not comparable** there (ADR-0012).
- **PASS** — detection holds to ≥ 60 °/s → the sim's no-blur assumption is safe. **INTERESTING** —
  drops at 30–60 °/s → **give the sim a blur model**: set the row-7 `TERM_LOS_RATE_DROPOUT` threshold to
  the measured °/s (the sim currently has none). **FAIL** — drops < 30 °/s → terminal blur is worse than
  every sim number assumed; this is the ADR-0012/0014/0015 existential risk, quantified.
- **Feeds:** **row 7** directly — the sim's terminal detector should drop detection above the measured
  |λ̇| threshold.

### (d) Lighting variation (indoor / outdoor-sun / backlit)

Repeat (a) at **one** mid-band range under three lighting conditions. Log exposure/gain (watch
auto-exposure behavior). Note IR sensitivity: some OV9281 modules are NoIR / lack an IR-cut filter,
which can wash black/white contrast in sunlight.

- **Sim context:** the sim renders **ideal** contrast and even needed an emissive hack to get there
  (ADR-0007); real-print contrast under real light is untested.
- **PASS** — detection_rate ≥ 0.90 in all three. **INTERESTING** — degrades backlit / low-contrast →
  real contrast is a factor the sim doesn't model (informs how optimistic the fiducial-lock assumption
  is). **FAIL** — fails outdoors → contrast/exposure handling needs work before any outdoor claim.
- **Feeds:** no single sim knob — it **bounds the fiducial-lock optimism** and becomes a stated caveat
  next to the Pk headline (the ADR-0014/0015 perception-gap disclosure). Discount P_detect accordingly.

---

## 4. Sim-vs-bench gap table (template, pre-filled with the sim's current numbers)

Fill "Bench measured" and "Gap" from the runs above. All sim numbers are from the **wide-lens** config
that main currently ships (ADR-0024 3rd addendum: the 60° narrow lens was rejected; main keeps 99.7°).

| # | Measured quantity | Sim assumption (sourced) | Bench measured | Gap | Sim knob to update |
|---|---|---|---|---|---|
| 1 | Static detection rate | **1.000** at ~4.9 m (M2; thr ≥ 0.90) — `logs/m2_detect_20260707T192233Z.csv` | ___ | ___ | row 7 detect-rate baseline; `TERM_*` in `guidance_lab.py`/`s2_cue_mock.py` |
| 2 | Max reliable detection range (envelope) | **~6 m** for a 0.5 m tag on the 99.7° lens (ADR-0007; M2 envelope, `check_m2_envelope.py`) | ___ | ___ | acquisition range (row 10) / `P_detect(R)` |
| 3 | Range (pose-Z) error | **0.0861 m** at ~5 m (≈1.7%); sim assumes AprilTag ~**5–8%** (row 8) | ___ | ___ | **row 8** `TERM_RANGE_NOISE_FRAC` / widen `meas_range` noise |
| 4 | Bearing (lateral) error | **sub-degree** fiducial (X,Y matched GT < 5 mm — ADR-0006/0007) | ___ | ___ | **row 9** `TERM_BEARING_SIGMA_DEG` |
| 5 | Sustained detection Hz | **~14 Hz** desktop (ADR-0009); **Pi-5 rate UNMEASURED** (ADR-0012) | ___ | ___ | ADR-0009/0010 cadence constants; `quad_decimate` |
| 6 | Max trackable yaw/LOS rate before dropout | **none modeled** (yaw clamp 60 °/s; LOS 32–58 °/s at CPA — ADR-0014/0023) | ___ | ___ | **row 7** `TERM_LOS_RATE_DROPOUT` threshold (new) |
| 7 | Lighting robustness | **ideal** emissive contrast, zero degradation (ADR-0007) | ___ | ___ | new caveat / `P_detect` discount (no direct knob) |
| 8 | Calibrated fx (documentation) | **539.9 px** @ 1280×960, 99.7° HFOV | ___ (from `calibrate_camera.py`) | ___ | `camera_intrinsics_real.json`; set real `TAG_SIZE_M` |

*Rows 3/4/6 are ADR-0015 data-constraints rows 8/9/7 — the three "assumed" terminal-sensor numbers this
bench exists to replace with measured values (ADR-0015 build-plan step 4).*

---

## 5. Time budget — the honest "one afternoon"

**Do the prep on separate days (not the afternoon):** order parts (5 min) then wait days–weeks for
shipping; print + mount the chessboard and tags (~30 min); flash the SD, boot, make the venv, and pip-install
(~30–45 min, mostly download — do it the night before). **If the SD/venv/print prep is already done, the
measurement session is genuinely one afternoon (~4–5 h):**

| Step | Time |
|---|---|
| Mount camera, first frame, confirm resolution (`check_resolution`) | 20–30 min |
| Camera calibration (15–20 views + RMS sanity ≤ 1.0 px) | 30–45 min |
| Fit/adjust wide M12 lens to ~100° + re-calibrate (**primary only**) | +30 min |
| (a) Static detection + pose at 3–5 ranges (tape-measure each, ~100 frames) | 60–75 min |
| (b) Detection Hz at quad_decimate 1 & 2 | 15 min |
| (c) Yaw-rate / motion-blur ramp | 30–45 min |
| (d) Lighting variation (3 conditions at one range) | 30 min |
| Fill the gap table from the CSV logs | 20 min |
| **Total** | **~4–5 h** |

**Honesty flag:** (a), (b), and (d) plus calibration comfortably fit one afternoon. **The yaw-rate rig
(c) is the schedule risk** — building a *measured* constant-rate turntable/servo is the item most likely
to slip to a **second session**. Budget for that rather than promising the motion-blur number the same day.
The CSI camera options (B/C/D) add the Picamera2-shim work (§2.3) on top — another reason the USB primary
keeps the afternoon honest.

---

## 6. Go/no-go readout (what the bench decides)

Stage 0 is a **gate on the whole hardware plan** (ADR-0012). Interpret the filled table as:

- **GO** — detection rate ≥ 0.90 and range err ≤ 0.25 m at the sim-equivalent range (§3a PASS),
  **and** sustained ≥ ~15 Hz (§3b PASS), **and** detection survives to ≥ 60 °/s (§3c PASS). The sim's
  perception assumptions hold on real hardware; proceed to ADR-0012 Stage 1 (tethered avionics).
- **CONDITIONAL** — any test lands INTERESTING. Update the flagged sim knob(s), **re-run the affected
  Pk/miss batches under the corrected perception model** (ADR-0015 step 3), and re-decide with honest
  numbers before spending on the airframe.
- **NO-GO (as designed)** — §3b < 8 Hz or §3c < 30 °/s. The camera-only terminal window collapses on
  real hardware exactly as the council feared (ADR-0012 #1 / ADR-0015 #5). **This is a successful ~$240
  result:** it redirects effort to the ADR-0015 Hailo/ML seeker (or a slower-terminal / longer-standoff
  mechanization) *before* the airframe spend — which is the entire reason Stage 0 comes first.
