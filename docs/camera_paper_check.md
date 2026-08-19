# Camera paper-check — innomaker OV9281 mono GS wide (CAM-MIPIOV9281 V2)

> P0.5 (docs/next.md). The seeker camera is **ORDERED** (2026-07-20 Tier-1 BOM). This is the
> paper gate: verify the datasheet clears the constraints BEFORE frames arrive, so the
> tripod day measures the real curve instead of discovering a spec surprise. Engineer's-
> notebook tone; every number cites a source; each item ends in a single VERDICT line.
> Anything that can only be settled with real light/pixels is called UNVERIFIED-ON-PAPER.

**Part:** innomaker CAM-MIPIOV9281 V2 — OmniVision OV9281, 1 MP **monochrome global-shutter**,
1280x800, RAW8/RAW10, 2-lane MIPI, external-trigger capable. Sources:
[innomaker product page](https://www.inno-maker.com/product/cam-mipi9281raw-v2/) ·
[Amazon listing](https://www.amazon.com/Raspberry-External-Monochrome-Bullseye-libcamera/dp/B09WTP5GZH) ·
[OV9281 sensor page (OmniVision)](https://www.ovt.com/products/ov9281/) ·
[OV9281 product brief PDF](https://www.ovt.com/wp-content/uploads/2024/05/OV9281-PB-v1.4-WEB.pdf).
**Sim baseline** (for the deltas below): PX4 x500 mono cam **1280x960, ~100 deg HFoV**
(`docs/project_state.json` → `sim_harness.active`).

At a glance: (1) HFoV CLEARS · (2) resolution CLEARS (penalty ~15%, HALF the feared 30%) ·
(3) exposure CLEARS · (4) mono-vs-color UNVERIFIED-ON-PAPER · (5) Pi 5 CSI CLEARS.

---

## 1. Is the "118 deg" FoV horizontal or diagonal? (constraint `wide-fov`: >=100 deg HORIZONTAL)

innomaker publishes **both** axes explicitly: **FoV(H) = 118 deg, FoV(D) = 148 deg**
([product page](https://www.inno-maker.com/product/cam-mipi9281raw-v2/)). So the 118 deg is
the **horizontal** number, not the diagonal — 118 >= 100 with 18 deg of margin. (The 148 deg
is the diagonal; note the focus/lens is adjustable, so the delivered lens should be re-measured
against a checkerboard at calibration, but the shipped optic clears on paper.) The wide FoV is
a HARD requirement because the +/-30 deg open-loop coded-dash aim tolerance depends on it
(constraint `wide-fov`); a diagonal-only 118 would have implied HFoV ~103 deg (still >100), so
either reading clears — but the honest reading is that H is stated directly and clears outright.

**VERDICT: CLEARS** — 118 deg is horizontal (D=148 deg stated separately); >=100 deg HFoV met with margin.

## 2. Vertical FoV + px/deg vs the sim (project expected ~30% fewer px/deg — verify or correct)

Both cameras are **1280 px wide**, so the horizontal px/deg is a clean FoV ratio:
- Sim:       1280 px / 100 deg = **12.8 px/deg** (H)
- OV9281:    1280 px / 118 deg = **10.85 px/deg** (H)
- Delta = 1 - 10.85/12.8 = **~15% fewer px/deg horizontally** — NOT 30%.

**Correcting the 30%:** the ~30% figure is what you get if you (wrongly) treat the 148 deg
DIAGONAL as the HFoV: 1280/148 = 8.65 px/deg → 32% fewer. Using the true 118 deg horizontal
(item 1) it is **~15%**. So the project's ~30% is the diagonal-confusion number; the honest
horizontal penalty is **half that**.

Vertical: OV9281 has **800 rows vs the sim's 960** (16.7% fewer rows). Its VFoV under a
roughly-equidistant wide lens ~ 118*(800/1280) ~ **74 deg** → 800/74 = **10.8 px/deg** (V);
the sim's pinhole VFoV ~ 2*atan(0.75*tan50) ~ **83.6 deg** → 960/83.6 = **11.5 px/deg** (V).
So vertical px/deg is also **~5-15% lower**, and the 800-row sensor simply sees a shorter
vertical strip (less sky/ground) — the on-target resolution loss is the ~15% px/deg, not the
row-count difference.

**Range penalty:** pixels-on-target scale with px/deg, so ~15% fewer px/deg → a target must be
~15% CLOSER to subtend the same pixels → **~15% shorter decode/acquisition range** at a fixed
min-pixel threshold. This eats into the t_go/R_acq margin (constraint `pi5-compute`/R5: R_acq
must leave t_go >= ~0.5 s, ~>=20 m at 9 m/s) but is HALF the feared 30%. The ABSOLUTE tag-decode
and NN acquisition range on this sensor is a real-light measurement (the tripod range-walk R5 /
P0.3 scorer), not a paper number.

**VERDICT: CLEARS** — px/deg penalty ~15% (H and V), half the assumed 30%; the assumed 30% was the diagonal-FoV error. Absolute decode/NN range stays tripod-gated (not a paper claim).

## 3. Can the OV9281 + Pi libcamera be driven to <=1 ms exposure, and is the control exposed?

Yes. `ExposureTime` is a first-class Picamera2/libcamera control in **microseconds**, and the
OV9281 minimum is **register value 4 ~ 40 us** — far below the 1000 us (1 ms) target
([Picamera2 exposure how-to, issue #145](https://github.com/raspberrypi/picamera2/issues/145) ·
[RPi forum: OV9281 min exposure](https://forums.raspberrypi.com/viewtopic.php?t=336785)).
Two gotchas to bench: (a) you must also lower `FrameDurationLimits` when you shorten exposure or
the pipeline throws "Dequeue timer expired"; (b) at very short exposure the frame goes dark
unless there is enough light + gain — a scene/illumination issue, not a control limit, and
outdoor daylight is exactly where a fast global-shutter exposure is meant to run (freezing a
fast target is the whole reason for this sensor). Control is exposed; the floor clears 1 ms by 25x.

**VERDICT: CLEARS** — ExposureTime (us) is exposed in Picamera2; sensor floor ~40 us << 1 ms; set FrameDurationLimits alongside it and ensure daylight.

## 4. Mono-vs-color confound for drone_finetuned_quad_v2

`drone_finetuned_quad_v2` was fine-tuned on **color** Gazebo renders; the OV9281 is
**monochrome**. Feeding a gray frame (channel-replicated to 3ch) into a color-trained net
removes whatever chroma cue it learned — e.g. the red enemy props — so the expected recall
shift is **downward (a penalty), direction-known but magnitude-unknown on paper**. This stacks
on top of the already-diagnosed FLIGHT-DYNAMIC recall wall (ADR-0076 add #18k), so a mono frame
should be treated as strictly harder, never easier, than the sim curve. The sanctioned fix is
NOT a channel hack — it is the **real-data retrain**: capture AprilTag-auto-labeled MONO frames
of the real target from the final mount geometry and fine-tune (COCO-init, held-out FLIGHTS,
per `docs/real_data_pipeline.md`). Until that curve exists, any sim/color recall number is an
optimistic upper bound for the mono sensor.

**VERDICT: UNVERIFIED-ON-PAPER** — direction is a recall penalty (color cue lost); magnitude needs real mono frames; the real-data mono retrain (`docs/real_data_pipeline.md`) is the sanctioned fix.

## 5. Pi 5 CSI: 22-pin requirement, Wonrabai 22->15 bridge, native ov9281 driver

The **Raspberry Pi 5 uses 22-pin (0.5 mm) CSI connectors**, while camera boards (incl. the
innomaker OV9281) use the standard **15-pin (1 mm)** FPC — so a **22->15 adapter cable is
required**, and the ordered **Wonrabai 22->15** cable is exactly that bridge
([RPi camera cable product](https://www.raspberrypi.com/products/camera-cable/) ·
[Arducam Pi camera pinout](https://docs.arducam.com/Raspberry-Pi-Camera/raspberry-pi-camera-pinout/)).
Driver: **ov9281 is a mainline libcamera/kernel sensor** — enabled with `dtoverlay=ov9281` in
`/boot/firmware/config.txt`, **no vendor kernel driver to build**
([innomaker CAM-OV9281RAW-V2 repo](https://github.com/INNO-MAKER/CAM-OV9281RAW-V2)).
Paper residual to settle at the bench, not a blocker: on **Pi 5 / Bookworm** you set
`camera_auto_detect=0`, and the base libcamera tree has **no ov9281 tuning file** yet — use
`uncalibrated.json` (fine for us: mono means no AWB/CCM to lose) on a current libcamera; some
users report timeouts until libcamera is up to date
([RPi forum: ov9281 on Pi5 Bookworm](https://forums.raspberrypi.com/viewtopic.php?t=362009)).

**VERDICT: CLEARS** — Pi 5 needs the 22->15 cable (the ordered Wonrabai is correct); ov9281 is a native mainline overlay (no vendor driver); Pi5/Bookworm only needs `camera_auto_detect=0` + `uncalibrated.json`, verified at the bench.

---

## Net

All five items clear on paper except the mono NN curve, which is unmeasurable on paper by
construction (it needs real mono frames — that IS the tripod day's job, `docs/real_data_pipeline.md`
+ P0.3 scorer). The one number to carry forward: the resolution penalty vs the sim is **~15%,
not 30%** — the 30% expectation was the 148 deg diagonal mistaken for the horizontal FoV. The
two things to check the moment hardware lands: (a) re-measure the delivered (adjustable) lens
HFoV against a checkerboard at calibration; (b) confirm `dtoverlay=ov9281` + `uncalibrated.json`
brings up frames on this Pi 5 / Bookworm before trusting the range-walk.
