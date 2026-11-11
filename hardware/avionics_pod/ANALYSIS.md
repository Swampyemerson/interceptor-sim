# Avionics pod: mass, vibration and heat (2026-11-09, nose post added 2026-11-10)

**What this is:** calculations on the CAD and on estimated part weights. Nothing here was
measured on the aircraft, because the pod is not printed. Each section says what it rests
on and what would test it. Numbers come from `analysis.py` (run it to reprint them) and
`fea_modal.py` (results saved in `fea_results/`).

## 1. Mass and balance, whole aircraft

Four layouts, each with frame, motors, battery underneath and all electronics:

| Layout | Mass | Centre of mass, height | Roll inertia | Pitch inertia |
|---|---|---|---|---|
| A. Old plan: boards low on the top plate, camera on a nose boom | 824 g | 23 mm below the props | 1.95 g·m² | 1.92 g·m² |
| B. Pod, camera on a nose boom | 850 g | 20 mm below | 2.14 (+10 %) | 2.15 (+12 %) |
| C. Pod with the camera cradle on its lid | 853 g | 17.5 mm below | 2.35 (+21 %) | 2.19 (+14 %) |
| D. Pod, camera on the dock's nose post (chosen 2026-11-10) | 853 g | 19 mm below | 2.19 (+12 %) | 2.12 (+10 %) |

Inertia is how hard the aircraft is to start and stop turning about an axis.

- **Weight:** the pod costs about 25 to 30 g over the old plan (3 %).
- **Balance:** the battery underneath outweighs the pod on top, so the centre of mass
  stays below the prop plane in every layout. The pod raises it by 3 to 5 mm. Fore-aft it
  stays within 2 mm of the frame centre. The pod's place on the frame is now fixed by the
  dock's four screws, which TBS's drawing puts either side of the frame centre.
- **Turning:** roll gets 10 to 21 % heavier to turn. (Motor positions corrected
  2026-11-11 from TBS's drawing: 77 mm fore and aft, 92 mm left and right, not 85 and 85.) The rate gains now on the board were
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
| No camera | 270 Hz | 343 | 437 | whole stack swaying side to side |
| No camera, softer plastic | 241 | 303 | 389 | same |
| With camera cradle | 136 | 222 | 266 | stack swaying, camera leading |
| With camera cradle, softer plastic | 121 | 190 | 232 | same |
| **Nose post** (dock held flat on the plate) | 258 | 287 | 550 | post and camera swaying side to side |
| Nose post, softer plastic | 218 | 244 | 500 | same |
| Nose post, held only at its six screws | 187 | 252 | 411 | the post nodding fore and aft on its tongue |
| Nose post, screws plus a 5 mm patch of underside round each | 216 | 256 | 512 | same |
| Nose post dock with the loaded pod on it, held at the six screws | 150 to 160 | 187 | 241 | the pod stack swaying on the dock; then the camera post |

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
- **Bolting is necessary, not sufficient.** See the cross-check below: the bolted dock
  still shows a pod mode near 110 to 160 Hz. The tap test with the dock bolted to the
  frame and the loaded pod latched on is now the first vibration check to do, followed
  by a Pixhawk vibration log at hover.

**Camera dimensions (2026-11-10).** The camera plates and the camera stand-in now use
Arducam's own model of the B0353 (board 38 x 38, holes on a 34.04 mm square, lens barrel
14 mm across and 19.2 mm tall). Fitting it widened the plate's window and slimmed its four
posts, which is why the nose-post figures here are a little lower than the first run
(270 Hz then, 254 Hz after).

**Revision 5 (2026-11-11).** The nose-post rows above are for the post as redrawn for the
real top plate: screws behind the post instead of ahead of it, post 5 mm further
forward, tongue 6 mm thick. Before the tongue was thickened the screws-only case was
92 Hz, because the post nodded on a 3 mm tongue; that is why it is thick. Revision 4's
figures were 254 Hz held flat and 169 Hz at the screws.

**SOLIDWORKS cross-check (2026-11-10), made on revision 4.** The "this model" column
below is revision 4 as well, so the two columns compare like with like; neither is the
current part. The builder ran the nose post and dock in
SOLIDWORKS Simulation with the same materials. Full report: `SOLIDWORKS_CROSSCHECK.md`;
numbers: `solidworks_results.json`; pictures: `media/solidworks_*.png` at the repo root.

| Case | SOLIDWORKS 1st / 2nd / 3rd | This model |
|---|---|---|
| Nose post, underside held flat | 275 / 293 / 552 Hz | 254 / 276 / 545 |
| Nose post, six screw holes held | 173 / 184 / 410 | 169 / 184 / 411 (`--nose --screws`) |
| Same, pod added | 127 / 173 / 185 (pod as a rigid 162 g lump) | 109 to 160 / 180 / 190 (`--full`, pod meshed) |

- **The two programs agree** to 8 % or better wherever they model the same thing.
- **How the dock is held matters more than the program.** Held at its screws instead of
  flat, the camera post's first sway drops from about 260 to about 170 Hz. That replaces
  the "20 to 40 % lower" judgment above with a calculated 32 to 37 % for this part.
- **Bolting the dock does not take the Pixhawk out of the motor band.** With the pod on
  the screw-held dock, both programs put a pod mode between about 110 and 160 Hz, below
  the hover motor rate of 213 Hz and crossed on every spool-up. The Pixhawk rides in the
  pod, so this one reaches its sensors.
- **That pod figure is the least certain number here.** In this model it moves with mesh
  size (160 Hz at 4 mm elements, 144 to 149 at 2.5 mm). The 3 mm run is discarded: on
  revision 5 it returned a zero-frequency mode, meaning a part lost its glued contact in
  meshing, and the 109 Hz it gave on revision 4 was probably the same fault. This model glues the pod floor to the dock and leaves out the Pixhawk's 39 g;
  SOLIDWORKS uses a lump with an estimated inertia. Neither has the carbon plate under
  the dock, which would raise it, or the real hook, pins and single tail screw, which
  would lower it.
- **Static strength is not the limit.** At 10 g on all three axes at once the camera post
  peaks at 1.0 MPa. The dock carrying the pod peaks at 27 MPa at the edge of one rigidly
  held screw hole (an artifact of holding the hole wall) and stays under about 6 MPa
  elsewhere. Printed PETG strength is assumed at 45 MPa, 25 MPa across layers.
- **`pod_base` would not mesh in SOLIDWORKS.** Each Pi boss was exactly tangent to the
  side panel's outer face and left a 0.2 mm sliver face. Fixed in `pod.py` the same day
  (bosses 6.6 mm, panels 1.2 mm longer at each end); the pod rows of the table above are
  rerun on the new base. Not yet retried in SOLIDWORKS.

**What this means for the camera.** The lid cradle is the least stiff place on the
aircraft to put it. The builder chose the nose post on 2026-11-10; it is drawn
(`dock_cam`, `cam_plate_nose`) and modelled:

- **Stiffness.** First sway 258 Hz in the model against 136 Hz for the lid cradle, nearly
  twice as stiff, and the load path is short: camera, plate, two cheeks, dock, carbon.
  No circuit boards or standoffs in it. The same caveat applies (glued joints, so read
  it as an upper bound; the softer-plastic row is 218 Hz, right at the hover motor rate).
- **It is not out of the motor band.** Hover is about 213 Hz and full throttle about
  583 Hz, so 190 to 290 Hz is crossed on every throttle-up. No printed mount gets above
  583 Hz. What the stiffer mount buys is less movement for the same shake. Whether the
  picture is steady enough is still a question for the printed part and a tap test.
- **Props out of the picture** (`nose_post_check.py`, lens front as the viewpoint, props
  level with the plate / 10 mm higher): 70° lens +35 / +34° at 10° of tilt, +52 / +46° at
  25°; AR0234 80° lens +31 / +29° at 10°, +48 / +42° at 25°. The published 118° lens is
  marginal at 10° (+12 / +9.5°) and clear at 25° (+30 / +23°).
- **Clear of the prop circles.** In top view the post and the camera plate stay 8.4 to
  10 mm outside the front prop circles, and the plate is 16 to 21 mm above the blades.
  That uses TBS's drawing for the motor positions and assumes the props are level with
  the top plate, which is not measured. Check by turning the props by hand.
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
