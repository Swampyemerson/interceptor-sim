# Avionics pod: mass, vibration and heat (2026-11-09)

**What this is:** calculations on the CAD and on estimated part weights. Nothing here was
measured on the aircraft, because the pod is not printed. Each section says what it rests
on and what would test it. Numbers come from `analysis.py` (run it to reprint them) and
`fea_modal.py` (results saved in `fea_results/`).

## 1. Mass and balance, whole aircraft

Three layouts, each with frame, motors, battery underneath and all electronics:

| Layout | Mass | Centre of mass, height | Roll inertia | Pitch inertia |
|---|---|---|---|---|
| A. Old plan: boards low on the top plate, camera on a nose boom | 824 g | 23 mm below the props | 1.77 g·m² | 2.11 g·m² |
| B. Pod, camera on a nose boom | 849 g | 20 mm below | 1.96 (+11 %) | 2.35 (+12 %) |
| C. Pod with the camera cradle on its lid | 853 g | 17.5 mm below | 2.17 (+23 %) | 2.36 (+12 %) |

Inertia is how hard the aircraft is to start and stop turning about an axis.

- **Weight:** the pod costs about 25 to 30 g over the old plan (3 %).
- **Balance:** the battery underneath outweighs the pod on top, so the centre of mass
  stays below the prop plane in every layout. The pod raises it by 3 to 5 mm. Fore-aft it
  stays within 2 mm of the frame centre, on the assumption that the pod sits centred.
- **Turning:** roll gets 11 to 23 % heavier to turn. The rate gains now on the board were
  tuned by feel on the aircraft without the pod, so expect to re-check them after the pod
  goes on: the same gain gives a slightly slower response.
- **Rests on:** estimated weights for nearly every part (`docs/airframe_numbers.md` §3) and
  an assumed frame height stack. Weighing the built aircraft and a string-balance test
  would replace all of it.

## 2. Vibration

**The forcing.** A motor shakes the frame once per turn. From `docs/airframe_numbers.md`
§7 (estimates): about 213 Hz in hover, rising to about 583 Hz at full throttle. A part
whose own natural frequency sits inside that range will be shaken hardest.

**The pod tower, floor held rigid** (finite-element model, lowest natural frequencies):

| Case | 1st | 2nd | 3rd | What moves first |
|---|---|---|---|---|
| No camera | 273 Hz | 337 | 444 | whole stack swaying side to side |
| No camera, softer plastic | 246 | 297 | 393 | same |
| With camera cradle | 138 | 221 | 266 | stack swaying, camera leading |
| With camera cradle, softer plastic | 122 | 189 | 232 | same |

- These are **upper bounds**. The model glues every joint; real screwed joints through two
  circuit boards are softer. Expect the real numbers 20 to 40 % lower. That figure is my
  judgment, not a calculation.
- **Without a camera:** the stack's first sway sits at or just above the hover motor rate.
  What rides on the tower is the Pi and the HAT, which tolerate shaking. The Pixhawk sits
  on the floor, not on the tower, so its sensors do not see this mode.
- **With the camera on the lid:** adding 35 g at the top halves the first frequency. Two
  sway modes land between 120 and 220 Hz, below and at the hover motor rate. A camera
  there will see sway. How much, in degrees, this model cannot say: that needs the
  size of the shaking force, which is unmeasured.
- **Where the flex is:** in the first mode the energy splits about evenly between the
  lid, the printed base and the two circuit boards. No single part is the weak link, so
  thickening one part will not fix it. I tried: a stiffer cradle moved the first mode from
  110 to 138 Hz; moving the camera back over the lid reached about 166 Hz; cross-beams
  under the Pi had no room to be deep enough to matter.

**The whole pod rocking on foam tape** (hand formula, pod treated as rigid):

| Tape | No camera | With camera |
|---|---|---|
| 1 mm, soft (shear stiffness 0.2 MPa) | 207 Hz | 162 Hz |
| 1 mm, medium (0.5 MPa) | 327 | 256 |
| 1 mm, firm (1.0 MPa) | 462 | 362 |
| 3 mm, medium | 189 | 148 |

- This mode carries the **Pixhawk** with it, so it matters with or without a camera.
- Every row lands inside or just under the motor band. Tape stiffness varies several-fold
  with brand and temperature, so tape cannot be relied on to stay out of it.
- **Conclusion: bolt the dock.** The frame drawing gives four standoff screws under the
  dock (see `GAP_REVIEW.md` item 1); the dock now uses them. Tape plus zip ties stays as
  the no-drill fallback for a first fit check, not for flight with the Pixhawk aboard.

**What this means for the camera.** The lid cradle is the least stiff place on the
aircraft to put it. Three ways forward, best first by my judgment:

1. **Camera on a short post off the dock's nose.** Low and stiff, bolted to the carbon
   plate through the dock, still next to the Pi (ribbon about 100 mm). Prop margin from
   `camera_on_pod_fov.py` at 60 to 80 mm forward, 30 mm up: +16 to +25° at 25° of tilt for
   the 70° and 80° lenses, but only +1 to +12° at 10° of tilt. So it works at dash tilt
   and not for a shallow tilt. Not drawn yet.
2. **Keep the lid cradle and measure it.** Print it, tap it, film it with the phone
   slow-motion tool (`scripts/bench/phone_wobble_freq.py`). If the sway is small it wins
   on prop margin and on everything listed in the README.
3. **Walls from the base straight up to the lid,** outside the boards, so the camera load
   bypasses the circuit boards. Stiffest, heaviest, and it boxes in the Pi's side ports.

**Would test it:** a tap test on the printed pod (phone slow-motion, or the Pi's own
camera pointed at a ruled target), then the same with motors running.

## 3. Heat

- **Heat to remove:** roughly 10 to 17 W with the seeker running (Pi 5 at 6 to 9 W,
  Hailo-8 at 2.5 to 5 W, camera about 1 W, Pixhawk 1 to 1.5 W). These are typical figures
  from memory, not measured here.
- **Air needed:** 0.9 to 1.4 litres per second holds the air inside to a 10 °C rise.
  Through the 9 cm² gap between the Pi and the HAT that is 1.6 m/s at the high end.
- **In flight:** the pod's sides are open and it sits in prop wash, so that airflow is
  there with a large margin.
- **On the ground, props stopped:** only the Pi's fan and natural draught move air, and
  the HAT sits 16 mm above the cooler. The repo's one measured soak (58 °C at 96.6
  frames per second, `skr-07`) was a bare Pi on the bench with no HAT, lid or pod. It does
  not cover this arrangement.
- **Pixhawk under the Pi:** the gap is 2.55 mm. If the Pi's underside runs 20 °C warmer
  than the Pixhawk's lid, about 0.4 W crosses the gap, on top of the Pixhawk's own 1 to
  1.5 W. Expect it to warm up further and for longer than it does now. Its height sensor
  and gyros drift as they warm, so calibrate and judge height hold only after a few
  minutes powered, in the pod.
- **The plastic:** PETG starts to soften near 80 °C. The posts touch the Pi board at its
  corners, the coolest part, so normal running is fine. The risk is a dark pod left in a
  hot car, where the clamped screw inserts can creep. Do not print it in PLA, which
  softens near 60 °C.
- **Would test it:** a 15-minute soak with everything stacked in the printed pod, seeker
  running, props off, reading the Pi's temperature and throttle flag
  (`vcgencmd measure_temp`, `vcgencmd get_throttled`). Pass: below 80 °C and no throttle
  flag at the warmest air temperature you expect to fly in.

## 4. What I could not analyse

- Shaking amplitude, and therefore image blur: needs a measured vibration level.
- Strength in a crash or a ram hit: needs loads nobody has measured.
- Electrical noise from the speed controller into the camera ribbon and the serial cable.
- Compass error from the Pi and the power wiring: the GPS and compass position is not
  designed yet (`GAP_REVIEW.md` item 12).
