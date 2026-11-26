# GPS mast: field study brief for SOLIDWORKS (written 2026-11-25)

**Who this is for:** whoever sets the study up in SOLIDWORKS, a person or an AI session
driving it. It is self-contained: everything needed is in this file and the three STEP
files beside it. Nothing here was measured on the printed part; the part is not printed.

**The two questions:**

- **A. Compass.** How much magnetic field do the battery leads put on the compass inside
  the GPS unit, on the old lid and on the new mast?
- **B. GPS reception.** How much less radio noise from the Pi, the AI board and the camera
  ribbon reaches the GPS antenna on the mast, compared with the old lid?

## 1. What SOLIDWORKS can and cannot do

SOLIDWORKS Simulation (including the Student Edition used for the 2026-11-10 cross-check)
does structure, vibration, heat and flow. **It has no electromagnetic solver.** Fields need
a paid add-in from EMWorks, a separate company: **EMS** for steady and slow magnetic fields
(study A) and **HFWorks** for radio frequencies (study B). Both run inside SOLIDWORKS and
both have a free trial on emworks.com. One software listing gives EMS as starting at
CAD 7,500; no student price was found.

Without the add-ins:

- **Study A needs no add-in.** It is a short calculation (section 4 has the formulas and
  check values). Given the lead route, the repo's Python can run it in minutes.
- **Study B** can be done in a free solver (openEMS) from the same STEP files.
- The STEP files are still worth opening in SOLIDWORKS for a fit check: section 6.

## 2. Files and frame

| File | What it is |
|---|---|
| `avionics_pod_assembly_nose_camera.step` | Everything in flight position: printed parts, plus plain blocks named `REF_...` standing in for the boards and the GPS unit |
| `pod_lid_mast.step` | The lid with its four posts and arches, in print pose (upside down) |
| `gps_deck.step` | The round deck the GPS sits on, in print pose |

Frame of the assembly: **x forward, y left-right, z up from the airframe's top plate**, in
millimetres. Check the import with the Pi board: its underside should read z = 28.85.

| Thing | Where (mm) | Source |
|---|---|---|
| Pi 5 board (`REF_pi5_board`) | z 28.85 to 30.45, x -42.5 to 42.5, y -28 to 28 | `pod.py` |
| AI board (`REF_ai_hat_board`) | z 46.45 to 48.05, x -42.5 to 22.5 | `pod.py` |
| Top of the lid | z 57.65 | `pod.py` |
| Deck top face (the foil) | z 109.1, centre x -10, y 0; 88 across, 84.8 inside the rim | `pod.py` |
| GPS unit on the mast (`REF_gps_compass_unit`) | 50 across, z 109.1 to 123.5 | Vendor size, not yet measured |
| GPS unit as flown on 2026-11-22 | same unit, z 57.65 to 72.05; assume the same x and y | Its exact spot on the lid was not recorded |
| Compass sensor inside the unit | assume the unit's centre, 5 mm above its underside | **Assumed.** Repeat 5 mm higher and lower to see if it matters |
| GPS antenna inside the unit | assume the centre, 3 mm below the top face | **Assumed** |
| Camera tilt axis | x 65, z 36 | `pod.py` |

To get the as-flown case, copy the GPS body down by 51.45 mm and suppress the mast and deck.

## 3. The sources, and how well each is known

| Source | Value | Grade |
|---|---|---|
| Battery current | 8.8 A in hover, 77 A at full throttle | Measured, flights of 2026-11-22 |
| Compass pickup of battery current, GPS on the lid | 0.0036 G per amp (0.36 microtesla per amp) | Measured, flight 3 |
| Battery lead route and spacing | Inside the airframe body and below it, so 63 mm or more under the compass as flown. Red and black within 5 mm of each other everywhere except the last 20 mm, where they split round the capacitor onto the speed controller. Spare length is looped at the back of the frame with the pair kept together | Builder's second description 2026-11-25, after looking again (the first said up to 20 mm apart). Not measured |
| Capacitor split | Where the leads part round the capacitor: 20 mm long, about 70 mm below the surface the GPS sat on (so about z -12, inside the body). Its width is not known | Builder's description 2026-11-25, by eye |
| Foil as flown | One sheet the width of the lid (56 mm), under the GPS and running forward to the camera, over the ribbon. Not connected to ground as far as is known | Builder's description 2026-11-25 |
| Pi supply current | up to 5 A | Vendor rating of the Pi 5 supply. The real draw has not been measured |
| Pi supply route | Into the Pi, then straight down into the carbon body. The 5 V converter is inside the body | Builder's description 2026-11-25 |
| Earth's field at the site | 0.513 G total | Model figure used elsewhere in the repo |
| Radio noise from the camera ribbon | **strength not known.** Camera streaming on a bare ribbon took the GPS from 6 or 7 satellites to none | Measured effect, unmeasured level |
| Radio noise from the Pi and AI board | **strength not known.** Processor load alone cost one satellite | Measured effect, unmeasured level |
| Ribbon route and width | About 15 mm wide. From the Pi's camera connector it runs aft about 40 mm under the lid, folds back, and runs forward to the top of the camera | Builder's description 2026-11-25, by eye. This is as flown on 2026-11-22; the builder is moving the fold away from under the lid |

No datasheet gives the radio noise level of a Pi or a camera link. So study B cannot
predict satellites or a fix. It can only rank arrangements against each other.

## 4. Study A: battery leads and the compass (EMS, "Magnetostatic")

1. Open the assembly. Add two round copper bodies for the red and black battery leads
   inside the body (below z = 0), 5 mm apart, with a 20 mm long split at the speed
   controller end. If their layout is known, add the speed controller board and the
   battery pack as current paths: the leads alone do not explain the measurement.
2. Add an air box at least 400 mm on a side around everything.
3. Material: copper for the leads; air for every printed part and board (plastic and
   circuit board do not change a steady magnetic field).
4. Each lead is a "solid coil" carrying 1 A, out along red and back along black.
5. Mesh control: 1 mm on the leads and in a 10 mm ball at each compass point.
6. Solve. Read the flux density at the compass point for the unit on the lid
   (z = 62.65) and on the mast (z = 114.1). The answer per amp scales in a straight line.

**Checks before believing the result:**

- **Solver check.** One long straight wire carrying 1 A must give 0.020 G at 100 mm
  (field = 2e-7 x amps / metres, in tesla; 1 tesla = 10,000 G).
- **Calibration.** The on-lid case must come out near the measured 0.0036 G per amp. If it
  is more than twice off, the lead route in the model is wrong. Fix that first.
- **Hand formula for a close pair of leads:** field = 2e-7 x amps x gap / distance squared.
  A 5 mm gap gives the measured figure at 53 mm, 8 mm at 67 mm, 12 mm at 82 mm, so the
  measurement is the size a lead pair several centimetres away should give.

**Expected result,** from the measured figure and the 51.45 mm the mast adds:

| Leads below the compass today | If they act as a long pair (square law) | If they act as a small loop (cube law) |
|---|---|---|
| 40 mm | 0.0007 G per amp (5.2 times better) | 0.0003 (12 times) |
| 60 mm | 0.0010 (3.5 times) | 0.0006 (6.4 times) |
| 80 mm | 0.0013 (2.7 times) | 0.0008 (4.4 times) |
| 110 mm | 0.0017 (2.2 times) | 0.0011 (3.2 times) |

**With the builder's description** (leads 5 mm apart, the split 20 mm long, both about
75 mm under the compass sensor):

| Source | On the lid, G per amp | On the mast | Mast gain |
|---|---|---|---|
| Leads, 5 mm pair (square law) | 0.0018 | 0.0006 | 2.8 times |
| Split, 20 x 10 mm loop, compass straight above it (cube law) | 0.0009 | 0.0002 | 4.8 times |
| Split, 20 x 15 mm loop | 0.0014 | 0.0003 | 4.8 times |
| Both together | 0.0020 to 0.0032 | 0.0007 to 0.0009 | |
| Measured in flight 3 | 0.0036 | 0.0008 to 0.0013 if scaled by those gains | |

So the leads and the split together are the right size to explain most of the
measurement; any remainder would be the speed controller board, the battery pack's
internal path or the power module, none tested. The mast lands on either side of the
target depending on the mix. The split's field is in proportion to its area, so making
it narrower or shorter is a direct gain. Aluminium foil does nothing to a steady
magnetic field, so the foil sheet plays no part in study A.

**Target:** under 0.001 G per amp on the mast (0.08 G at 77 A, against Earth's 0.513 G).
**If the model misses it,** try the leads taped tightly together (the field falls in
proportion to the gap) and a route further from the mast, and report which gets there.

Smaller item, same study: a Pi supply pair 2 mm apart carrying the full 5 A straight
under the unit gives 0.027 G on the lid and 0.003 G on the mast. Both figures are an
upper bound on assumed geometry; model it only if the real supply route is known.

## 5. Study B: radio noise at the GPS antenna (HFWorks, "Antennas" or "S-parameters")

**Frequency:** 1575.42 MHz (GPS L1 and Galileo E1). Sweep 1.55 to 1.61 GHz to cover the
other constellations. The wavelength is 190.3 mm.

**Simplifications to enter:**

- Pi board, AI board, camera board and the foil: perfect conductor. For the foil, apply
  the perfect-conductor condition to the deck's top face inside the rim, not a thin body.
- Printed plastic: relative permittivity 3 (**assumed**, typical for PETG). Run one case
  with it set to 1 to show how little it matters.
- Outer boundary: radiation (absorbing), at least 48 mm (a quarter wavelength) clear of
  everything.
- Mesh: no element over 19 mm in air (a tenth of a wavelength), 3 mm at conductor edges.

**Noise sources.** Each is a 30 mm straight wire with a port at its middle, driven at the
same level every time. The direction of the real noise current is not known, so run each
source lying along x and again along y.

| Source | Where |
|---|---|
| S1, AI board | centre of the AI board, 2 mm above it |
| S2, Pi board | centre of the Pi board, 2 mm above it |
| S3, camera ribbon | on the folded part of the ribbon under the lid, lying along the ribbon. Draw the ribbon as a 15 mm wide conductor strip between the AI board (top at z 48.05) and the lid (underside at z 56.05): from the Pi's camera connector aft 40 mm, folded back, then forward to the camera at x 65, z 36. Take the connector's position from the Pi 5 drawing |

**What to read:** the electric field strength averaged over a 25 mm square at the antenna
point (z = 69.05 on the lid, z = 120.5 on the mast).

**Arrangements:**

| Case | GPS position | Plate |
|---|---|---|
| B0 | on the lid, as flown | the foil sheet as flown: 56 mm wide, from under the GPS forward to the camera, not grounded |
| B0n | on the lid | none |
| B1 | on the lid | 84.8 mm plate under it |
| B2 | on the mast as drawn (51.45 mm up) | none |
| B3 | on the mast as drawn | 84.8 mm plate (the design) |
| B4 | mast 25 mm shorter | 84.8 mm plate |
| B5 | mast 50 mm taller | 84.8 mm plate |
| B6 | as B3 | plate joined to the Pi board by one thin wire down a post |
| B7 | as B3 | 60 mm plate |

**Report** each case in decibels against B0 (as flown), per source and direction: 20 x log10 (field
in the case / field in B0). More negative is better.

**Expected, from distance alone** (no plate): the noise close to a small source falls
with the cube of distance inside about 30 mm (wavelength divided by 2 pi) and in
proportion to distance far outside it. The mast takes the underside of the GPS from
9.6 mm to 61.0 mm above the AI board and from 27.2 mm to 78.6 mm above the Pi board. That
brackets B2 at 16 to 48 dB better for S1 and 9 to 28 dB better for S2. The ribbon's
fold lies 2 to 9 mm under the GPS as flown and 54 to 60 mm under it on the mast: 17 dB
or more better for S3. A result outside
those brackets means the setup is wrong or the plate and boards are doing something
worth understanding.

**Also check** the antenna still sees the sky: with the plate fitted, look at the pattern
of a source placed at the antenna point and confirm it is strongest upward.

## 6. Fit check (plain SOLIDWORKS, no add-in)

- Interference detection on the assembly: no clashes between `pod_lid_mast`, `gps_deck`
  and the reference blocks.
- Measure: the deck 88.0 across and 4.4 tall; the lid with mast 65.0 x 56.0 x 51.6.
- A screwdriver line through each of the deck's four 6 mm access holes reaches a lid
  screw without touching a post or an arch.
- The four post tops take an M3 insert 8 mm deep; the deck's four 3.4 mm holes line up.
- A frequency study with the dock screw holes held and a 32 g GPS on the deck: the repo's
  own model gives 130 Hz for the first mode with every joint glued.

## 7. What the results can and cannot decide (written before any run)

- **Study A can decide** whether the mast alone meets the compass target, because it is
  calibrated to a measurement. If it does not, the leads get taped together or rerouted
  before printing.
- **Study B can rank** mast heights and plate sizes. A difference under about 6 dB between
  two cases is not worth acting on (a judgment, given the assumed sources). It **cannot**
  say whether the GPS will hold a fix, because no noise level is known. The outdoor bench
  session in `docs/gps_reception_plan.md` decides that, and its pass mark is unchanged.
- **If study B says the mast height hardly matters,** print the shortest mast that passes
  the bench session: shorter is lighter and stiffer.

## 8. What to send back

For each run: the study type and settings, the mesh size, the two check values in
section 4, the result table, and one picture. A JSON file in the shape of
`solidworks_results.json` lets the repo record it.
