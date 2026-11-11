# Avionics pod: gap review (2026-11-09 overnight)

A second look at the pod CAD for things that would only show up when the parts
arrive. Everything below was measured from `pod.py` as it stood during the review
(camera cheeks at y = 27 to 30.5), from Holybro's and Raspberry Pi's own CAD, and
from TBS's published frame drawing. Nothing was printed or test-fitted. Run on the
WSL dev machine.

Three sources did most of the work:

- **TBS publishes the Source One V6 drawing** (`So1-V6-2025-AUG-19.dxf`,
  <https://github.com/tbs-trappy/source_one>). The top plate is in it, so most of
  the "measure the frame" list no longer needs calipers.
- **Holybro's case model** (`6C-MINI-A_rev.stp`), probed point by point to find
  every connector opening.
- **Raspberry Pi's Pi 5 model** (`RaspberryPi5-step.zip`), every part's position.

Axes as in `pod.py`: x toward the nose, y to the left, z up from the top plate.

## Ranked findings (worst first)

| # | Item | What is wrong | Measured | Proposed fix |
|---|---|---|---|---|
| 1 | Dock to frame | The dock assumes a 38 mm wide flat plate with a free 30.5 x 30.5 hole pattern. The real top plate has neither. Its only holes are the eight standoff screws, and six of those screw heads sit under the dock, so the dock cannot lie flat. | Plate 160.3 x 42.5 mm, but 42.5 wide for only 40 mm of its length; 29 mm wide elsewhere. Standoff holes at y = +-14.6 mm, stations 25.0 and 55.77 mm apart. | Bolt the dock to four of the standoff screws with longer M3s: pattern 29.2 x 55.77 mm. No drilling, and it is the rigid mount a camera needs. Parameters below. |
| 2 | Pi power plug | The Pi's USB-C power socket faces sideways (right side, x = -31). A straight plug and its cable loop hang out over the rear-right prop disc, 31 mm above the blades. A loose cable there falls into a prop. | Plug end 1.7 mm inside the disc in plan; cable bend 7.7 mm inside. A right-angle plug stays 6.4 mm outside. | Use a right-angle USB-C plug pointing aft and tie the cable to the rear tab. Write it into the README as a build rule. |
| 3 | Pixhawk model in the assembly is backwards | `assembly()` assumes the vendor model's +x end is the arrow end. It is the other way round, so the STEP shows the connector end forward and the servo pins aft, the opposite of how it flies. | -x face: two rows of three openings (POWER/TELEM2/TELEM1 low, GPS1/CAN2/CAN1 high). +x face: one large opening (servo pins). | Rotate by 180 only when `FC_ARROW_AFT` is False; fix the comment. Printed parts are unaffected (the pocket is symmetric). |
| 4 | Pixhawk USB-C for the bench | The side window stops below the top of a USB-C plug. | Socket at x = -22.7, z = 19.6 to 22.2, on the right side. A 12 x 7 mm plug body reaches z = 24.4; the window top is 22.85. Clash 1.55 mm. | Local notch in the right-hand panel: x from -32 to -13, up to `Z_PI - 2.5`. |
| 5 | Standoffs | The HAT kit's spacers are plastic, with plastic screws. They are too soft to carry a camera, and a plain spacer cannot screw down into the insert in the pod's boss. | Kit: nylon standoffs and nylon screws (Elektor build guide). | Buy brass M2.5 male-female hex standoffs: 4 x 16 mm and 4 x 8 mm. The stud goes through the Pi into the boss insert. |
| 6 | TELEM2 on the Pi's serial socket | Workable, with three conditions. The socket is top-entry, not side-entry: the plug goes in from above. | Socket 5.0 x 2.9 mm, 2.5 mm in from the board edge, about 1 mm from each HDMI shell, at pod x = -10, right side. 16 mm of height under the HAT. | (a) Plug it in before the HAT goes on. (b) The cable needs a 3-pin JST-SH on one end and a 6-pin JST-GH on the other. (c) Software: this socket is a different serial device from the GPIO pins (`ttyAMA10`, not `ttyAMA0`) and is the boot console by default. Untested. |
| 7 | Camera ribbon | The Pi's two camera sockets are on the right side, 19.5 mm off the centreline, and the front-right lid standoff stands in the straight path forward. | Sockets at pod x = 4.6 to 13.9, y = -27.9 to -11.1. Ribbon would overlap the standoff by 3.5 mm. | Fold the ribbon diagonally under the lid (8 mm of room) so it leaves the front on the centreline. Path about 100 to 110 mm; the 150 mm cable reaches. |
| 8 | Camera plate | The plate has no opening where the camera board's ribbon connector normally sits (bottom edge, back face). | Window is only +-12 mm; plate covers down to -21 mm; gap behind the board 2.5 mm. | Extend the window to the bottom edge as a U-notch. Confirm against the real board first. |
| 9 | Dock rear corners | The full-width rear of the dock comes close to the rear prop tips, at prop height. | 3.8 mm in plan at (-71, +-19) if the pod is centred. | Narrow the dock to +-10 mm aft of x = -44; it only carries the release boss there. This also takes it off two standoff screw heads. |
| 10 | Cables from the stack below | The speed-controller signal, receiver and power cables start under the top plate. The dock covers the plate and has no way through. | The plate has a cut-out 14.4 x 9.9 mm that lands at pod x = 16 to 26 with the mapping below. | Add a matching slot through dock and floor at x = 17 to 25, y within +-7, just ahead of the servo pins. |
| 11 | Zip-tie stations | The no-drill option ties round the whole plate, through the electronics bay. The plate has two unused strap slots that do the job better now the battery rides underneath. | Slots 2 mm wide at y = +-16.25 to 18.25, from pod x = -26 to +10. | `TIE_XS = (-20, 6)`; start the dock's edge notches at y = 16.0. |
| 12 | GPS and compass | Nothing in the design says where the GPS puck goes. The pod now occupies the top plate, and a compass on the lid would sit 10 mm above the HAT and 30 mm from the Pi and its fan. | Not measurable here. | A short mast on the dock's rear tab, as far aft and as high as the cable allows. Decide before printing. |
| 13 | Printing | No wall is under 1.2 mm, but three features need a print service to use supports or care. | Side-window top is a flat 48 mm bridge. Camera cheeks overhang the lid by about 37 mm. Thinnest spots: 1.4 mm under the bolt countersinks, 1.6 mm under the tie grooves, 1.75 mm under the tilt slot. | Tell the print service "supports on" for `pod_lid_cam` and `pod_base`. Camera lid: thicken from 1.6 to 2.4 mm. |
| 14 | README prop-disc figure | The "-0.2 mm" Pi-corner figure does not reproduce. | I get +7.4 mm at the Pi corners and +5.6 mm at the USB overhang, pod centred, 240 mm true-X. | Correct the README sentence. The camera cheeks' front corners are 3.7 mm inside the disc in plan, but 58 mm above it. |

Checked and fine:

- **Pi underside to Pixhawk lid:** 2.55 mm clear (Pi pins hang 1.45 mm below the board).
- **Pi bosses:** no component within 3.5 mm of any mounting hole on the underside.
- **Arrow-end plugs (aft):** POWER/TELEM row at z = 13.6 to 17.2, GPS1/CAN row at
  19.6 to 23.2; they pass above the release-screw boss (top 10.9) with 17.9 mm of length.
- **Servo-pin end (forward):** opening from z = 9.2 up; the hook top is 8.5, so plugs
  pass over it. The lowest plugs (RC IN) clear it by under 1 mm: tight.
- **Hinging the pod onto the dock:** the rear socket moves 0.19 mm fore-aft on the
  way down against 0.30 mm of clearance.
- **Pi SD card:** reachable from aft, 4.2 mm above the GPS1 plug. Cramped, workable.
- **Pi power button and light:** on the aft face, clear.
- **HAT's own ribbon and the fan cable:** both stay inside the board outline.
- **Pixhawk status lights:** on its top face, 4 mm under the Pi. Barely visible; use
  the GPS puck's light.

## The frame numbers, from TBS's drawing

Top plate, 2 mm, measured from its own centre (drawing y = long axis):

- Outline 160.3 x 42.5 mm. Full 42.5 width only from y = -25 to +15; 29.0 mm elsewhere.
- Standoff screw holes, 3.0 mm: (+-14.6, -57.66), (+-14.6, -32.66), (+-14.6, +23.11), (+-11.0, +75.98).
- Two strap slots 2 x 36 mm at x = +-17.25, y = -22.8 to +13.2.
- Standoffs per the drawing's parts list: 4 x 30 mm, 4 x 22 mm.

**Checked with calipers, 2026-11-11.** The builder measured the top plate of the target
drone, which is the same frame (its plate was easier to reach). Each distance was taken
twice, to the inner and the outer edges of the screw sockets, and averaged to get the
centre distance.

| Distance | Inner | Outer | Centre | Drawing | Dock tolerance |
|---|---|---|---|---|---|
| Left to right, one screw row | 27.5 | 31.1 | 29.3 mm | 29.2 | holes 3.4 mm for 3.0 mm screws |
| Fore and aft, front row to rear row of the dock's four | 54.22 | 57.78 | 56.0 mm | 55.77 | slots give +-0.5 mm |

| Fore and aft, dock's front row to the nose row | 23.2 | 26.9 | 25.05 mm | 25.0 | slots give +-0.5 mm |

**Which end is the nose: the opposite of what this review assumed (settled 2026-11-11).**
In the builder's photo of the target, going from the GPS (rear) to the nose, the screw
rows are spaced about 53, 56 and 25 mm. The 25.05 mm caliper reading at the nose
confirms it. In this drawing's plate coordinates that is the order +75.98, +23.11,
-32.66, -57.66: the narrow 22 mm pair at +75.98 is the REAR, and the nose has a
full-width row 25.0 mm ahead of the dock's front row.

TBS's drawing also has an assembled view. In it the motors are 77.13 mm fore and aft and
91.93 mm left and right of the frame centre, and the dock's four screws sit 27.9 mm
either side of that centre. So the frame centre is at pod x = -7.975, the front motors
at pod x = +69.15, the nose row of screws at +44.9, and the plate's nose edge at +67.4.
The plate's cut-out (item 10) is at pod x = -42 to -32, under the Pixhawk's tail.

`pod.py` revision 5 uses these. What it replaced: `NOSE_SCREW_X` 72.78 and
`NOSE_SCREW_DY` 22.0, the mapping `plate y = pod x + 3.2`, the front cable slot, and
motors at 84.85 / 84.85. Items 1 and 10 in the table above were written before this.

All three readings match the drawing within the dock's own slack. Not yet measured: the
rear narrow pair, anything on the interceptor's own plate, and the motor positions. The
printed coupon is still the final check.

## Parameter changes proposed for `pod.py`

```
DOCK_HOLE_DX = 55.77     # fore-aft, between the middle standoff rows
DOCK_HOLE_DY = 29.2      # across
DOCK_HOLE_X0 = -8.0      # unchanged: puts the pod's centre of mass over the bay
DOCK_SLOT    = 1.0       # the pattern is fixed now
TIE_XS       = (-20.0, 6.0)
```

Plus, in code: narrow the dock aft of x = -44 (item 9); notch the right panel for
USB-C (item 4); cable slot at x = 17 to 25 (item 10); U-notch in the camera plate
(item 8); fix the vendor-model rotation (item 3); tie notches from y = 16.0.
With `DOCK_HOLE_X0 = -8` the rear holes land at x = -35.9, 6.1 mm from the
locating pins at x = -30: it fits, with about 1 mm to spare.

## Only the builder can check these

1. Which end of the top plate is the nose, and are its screws flat-head or raised?
   (On the target the GPS sits at the rear, builder 2026-11-11. Its top-plate screws
   look raised in the photos. The interceptor's own plate is not yet confirmed.)
2. Is the electronics bay really at the middle of the frame (motor to motor)?
3. Camera board: hole spacing, and where its ribbon connector sits.
4. Tallest part on top of the HAT (lid gap is 8 mm).
5. The HAT kit's spacers: plastic or metal, threaded stud or plain?
6. Where the GPS puck sits today, and how long its cable is.
7. Prop height against the top plate, to the millimetre.

## Sources

- TBS Source One V6 drawing: <https://github.com/tbs-trappy/source_one>
- Holybro 6C Mini CAD: <https://docs.holybro.com/autopilot/pixhawk-6c-mini/download>
- Raspberry Pi 5 STEP: <https://datasheets.raspberrypi.com/rpi5/RaspberryPi5-step.zip>
- AI HAT+ documentation: <https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html>
- AI kit assembly guide (plastic standoffs): <https://elektormagazine.com/amp/news/unboxing-and-assembling-the-raspberry-pi-ai-kit-step-by-step-guide>
- Pi 5 serial ports (UART10 on the 3-pin socket): <https://github.com/raspberrypi/documentation/pull/3294>
