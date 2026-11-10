# Avionics pod: mass, vibration and heat (2026-11-09, nose post added 2026-11-10)

**What this is:** calculations on the CAD and on estimated part weights. Nothing here was
measured on the aircraft, because the pod is not printed. Each section says what it rests
on and what would test it. Numbers come from `analysis.py` (run it to reprint them) and
`fea_modal.py` (results saved in `fea_results/`).

## 1. Mass and balance, whole aircraft

Four layouts, each with frame, motors, battery underneath and all electronics:

| Layout | Mass | Centre of mass, height | Roll inertia | Pitch inertia |
|---|---|---|---|---|
| A. Old plan: boards low on the top plate, camera on a nose boom | 824 g | 23 mm below the props | 1.77 g·m² | 2.11 g·m² |
| B. Pod, camera on a nose boom | 849 g | 20 mm below | 1.96 (+11 %) | 2.35 (+12 %) |
| C. Pod with the camera cradle on its lid | 852 g | 17.5 mm below | 2.17 (+23 %) | 2.36 (+12 %) |
| D. Pod, camera on the dock's nose post (chosen 2026-11-10) | 850 g | 19 mm below | 2.01 (+14 %) | 2.25 (+7 %) |

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
| With camera cradle | 137 | 220 | 265 | stack swaying, camera leading |
| With camera cradle, softer plastic | 121 | 189 | 229 | same |
| **Nose post** (dock held flat on the plate) | 254 | 276 | 545 | post and camera swaying side to side |
| Nose post, softer plastic | 215 | 234 | 496 | same |

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

**Camera dimensions (2026-11-10).** The camera plates and the camera stand-in now use
Arducam's own model of the B0353 (board 38 x 38, holes on a 34.04 mm square, lens barrel
14 mm across and 19.2 mm tall). Fitting it widened the plate's window and slimmed its four
posts, which is why the nose-post figures here are a little lower than the first run
(270 Hz then, 254 Hz now).

**SolidWorks cross-check (builder, 2026-11-10).** The builder ran two frequency studies
on the nose-post assembly in SolidWorks Simulation, with the same inputs as this model
(printed PETG at 1900 MPa, a 15 g camera stand-in; the pod and boards left out).

| Study | 1st | 2nd | 3rd | Picture |
|---|---|---|---|---|
| "Nose post modal" (like-for-like with the table above) | 275 Hz | 293 | 552 | `media/solidworks_nose_post_modal_modes_275hz.png` |
| "Nose post bolted" (held at the bolts only) | 173 Hz | not recorded | not recorded | `media/solidworks_nose_post_bolted_mode1_173hz.png` |

- **The two programs agree.** SolidWorks gives 275 / 293 / 552 Hz against 254 / 276 / 545
  here, 8 % apart at most, and the same shape (post and camera swaying).
- **How it is held matters more than which program.** Holding the dock at its bolts
  instead of across its whole underside drops the first sway to 173 Hz, 37 % lower.
  That is the more realistic case, and it is below the hover motor rate (about 213 Hz):
  the motors pass through it on spin-up but do not sit on it in flight.
- Not recorded here: the exact fixture faces and the contact settings of each study.

**What this means for the camera.** The lid cradle is the least stiff place on the
aircraft to put it. The builder chose the nose post on 2026-11-10; it is drawn
(`dock_cam`, `cam_plate_nose`) and modelled:

- **Stiffness.** First sway 254 Hz in the model against 137 Hz for the lid cradle, nearly
  twice as stiff, and the load path is short: camera, plate, two cheeks, dock, carbon.
  No circuit boards or standoffs in it. The same caveat applies (glued joints, so read
  it as an upper bound; the softer-plastic row is 215 Hz, right at the hover motor rate).
- **It is not out of the motor band.** Hover is about 213 Hz and full throttle about
  583 Hz, so 215 to 275 Hz is crossed on every throttle-up. No printed mount gets above
  583 Hz. What the stiffer mount buys is less movement for the same shake. Whether the
  picture is steady enough is still a question for the printed part and a tap test.
- **Props out of the picture** (`nose_post_check.py`, lens front as the viewpoint, props
  level with the plate / 10 mm higher): 70° lens +22 / +17° at 10° of tilt, +40 / +34° at
  25°; AR0234 80° lens +18 / +12° at 10°, +36 / +30° at 25°. The published 118° lens has
  the props in frame at 10° (-1 / -6°) and clear from about 25° (+18 / +12°).
- **The camera sits over the edge of the prop circles.** In top view the plate's corners
  come within 1.8 to 3.1 mm of the front prop circles; they are 16 to 21 mm above the
  blades. The post itself keeps 6.5 mm at blade height, the same as the frame's own top
  plate. All of that assumes the pod is centred on the frame and the props are level with
  the top plate, and neither is measured. Check by turning the props by hand.
- **The other two options** stay in the CAD: the lid cradle (best prop margin, softest)
  and walls from the base straight up to the lid (not drawn).

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
