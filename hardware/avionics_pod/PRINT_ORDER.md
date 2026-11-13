# Avionics pod: what to print where (decided 2026-11-11)

The builder can send parts to the ITLL at CU Boulder, which runs Markforged printers.
This is the print plan for pod revision 6 (the order section at the end is current; the
research notes above it are from 2026-11-11). Prices and the fibre option are **not
confirmed**: the ITLL does not publish them, so the first step is a question to its staff.

## What the ITLL offers (read from its wiki, 2026-11-11)

- **Markforged, by request.** The form is "Markforged Onyx Print" under Requests, Make a
  Request, 3D Printing. Staff email questions and a payment link; work starts after payment.
- **Onyx** is nylon filled with chopped carbon fibre. That is the "carbon fibre" on offer
  by default. It is stiffer and far more heat-tolerant than PETG, not a carbon laminate.
- **Continuous fibre:** an ITLL equipment listing names the machine as an Onyx Pro. That
  model lays continuous **fibreglass**, not continuous carbon. Not confirmed with staff.
- **Prusa MK4S, self-service** in PLA or TPU after a workshop; PETG by request.
- **Files:** STL in millimetres. The STLs in this folder already are.
- **Purpose field** on the form offers Class, Research or Student Club. A personal
  project is not listed; ask staff which applies before submitting.
- **Price:** not published. The Idea Forge, also at CU, lists Onyx at $0.25 per gram,
  fibreglass-reinforced at $0.30 and carbon-reinforced at $0.35. Used here as a guide only.

## Decisions

| Part | File | Material | Why |
|---|---|---|---|
| Fit-check set | all four flight parts below | PLA | A few cents. Checks the six screw holes, that both boards drop in, the latch, and the camera post's clearance, before paying for Onyx. Bench only: PLA softens near 55 to 60 C |
| Template (optional) | `dock_cam_coupon.stl` | PLA | Flat copy of the dock's six holes. Redundant once `dock_cam` is printed in PLA; useful only for a faster first answer |
| Dock with camera post | `dock_cam.stl` | **Onyx, solid fill** | The part every vibration number depends on. Ask for fibreglass in the tongue if offered (below) |
| Camera plate | `cam_plate_nose.stl` | **Onyx, solid fill** | Same load path as the post |
| Pod base | `pod_base.stl` | **Onyx, solid fill** | Carries the Pi and sits over the Pixhawk: most of the flex in the pod's sway mode is in this part, and it is the one next to the heat |
| Pod lid | `pod_lid.stl` | Onyx if it rides along in the same order, PETG otherwise | Lightly loaded; material matters least here |

The plain dock and the lid-cradle camera parts are no longer in this folder (removed
2026-11-11), so every STL here is a current part.

**Sizes on the bed** (measured from the STL files; the request form asks for inches)

| File | Millimetres | Inches |
|---|---|---|
| `dock_cam.stl` | 140.0 x 40.0 x 40.0 | 5.51 x 1.57 x 1.57 |
| `cam_plate_nose.stl` | 42.0 x 39.4 x 20.2 | 1.65 x 1.55 x 0.79 |
| `pod_base.stl` | 101.0 x 56.0 x 25.9 | 3.98 x 2.20 x 1.02 |
| `pod_lid.stl` | 65.0 x 56.0 x 4.6 | 2.56 x 2.20 x 0.18 |
| `dock_cam_coupon.stl` | 137.9 x 40.0 x 0.8 | 5.43 x 1.57 x 0.03 |

**Settings to ask for**

- **Orientation:** print each STL exactly as it sits in the file; they are saved in print
  pose. `dock_cam` and `pod_lid` have a flat face down and need no supports.
  `cam_plate_nose` rests on its four screw bosses and **needs supports** under the plate.
  `pod_base` needs supports in two places only: the two side cable windows (a flat
  48 mm span, per `GAP_REVIEW.md` item 13) and nothing else. Say this on the request.
  The PLA fit set (received 2026-11-13) was sent with "Supports ok" and came back with
  support material in many places, including the cable windows, where it blocks the
  wires until broken out. All of it is meant to be removed; none is part of the design.
  Corrected 2026-11-11: this note used to say the lid goes upside down and that the camera
  plate prints flat as modelled. Both were wrong; the lid's flat face is its underside,
  and the camera plate's file was tilted in flight position.
- **Fill:** solid (100 %). The default Markforged fill is a sparse triangle pattern, and
  these parts are thin enough (2.4 to 6 mm walls) that sparse fill would leave mostly skin.
- **Layer height:** the printer's default (0.1 mm) is fine.
- **Fibre, optional, `dock_cam` only:** continuous fibreglass, concentric rings, in the
  flat tongue (the bottom 6 mm). The post nods on that tongue, and fibre lies in exactly
  the plane that stiffens it. It will not help the upright cheeks, where the load crosses
  the layers. Take it if the staff say the part is wide enough to route fibre; skip it
  without regret if not.

**Cost guess:** the four flight parts weigh about 56 g. At the Idea Forge's Onyx rate that
is about $14, perhaps $20 to $25 with a reprint or fibre. The ITLL's own rate may differ.

## What Onyx changes, and what it does not

- **Stiffness:** Onyx is roughly a quarter stiffer than the printed-PETG figure used in
  `ANALYSIS.md` (2.4 GPa against 1.9, from the maker's data sheet as I remember it, not
  looked up today). Frequencies scale with the square root, so expect about 10 % higher:
  the post held at its screws about 205 Hz instead of 187, the pod's sway about 165 to
  175 instead of 150 to 160. Not rerun in the model.
- **That is not a clear win by itself.** 205 Hz is closer to the hover motor rate (about
  213 Hz) than 187 is. The model is not accurate to 10 %, so this cannot be tuned on
  paper; the tap test on the printed part decides. Fibre in the tongue would move the
  nodding mode well clear, which is the main argument for asking about it.
- **Heat:** PETG softens near 75 to 80 C; Onyx holds to about 145 C. With a Pi and its
  cooler inside, that margin is the most certain benefit.
- **Moisture:** nylon takes up water and softens a little as it does. Keep the parts
  indoors and dry; do the tap test on parts that have sat a few days, not fresh off the bed.
- **Electrical:** chopped carbon makes Onyx slightly conductive. No bare solder joint or
  board edge should rest on it. The Pi and HAT are on standoffs and the Pixhawk is cased,
  so the design already respects this; keep it in mind for loose wires.
- **Heat-set inserts** go into nylon the same way as into PETG. Hole sizes are unchanged.
- **Strength** was never the limit (1 MPa at 10 g in the SOLIDWORKS check).

## Order of work

1. Print the four flight parts in PLA. Bolt `dock_cam` to the interceptor's top plate:
   six holes must line up and the tongue must stop short of the plate's nose edge. Fit
   the Pixhawk and the Pi into `pod_base`, latch it into the dock, fit the camera plate.
2. Look at where the wires come up from the stack. Revision 5 has no cable slot; they are
   meant to pass beside the plate into the pod's side windows.
3. Ask ITLL staff: is a personal project eligible and under which purpose, the price per
   gram, whether continuous fibre is available and in which material, and turnaround.
4. Submit the four flight parts as one request.
5. On the printed parts: weigh them, fit them, then the tap test in `ANALYSIS.md`.

## The Onyx order (2026-11-13, revision 6 files)

**Two CU shops run Markforged printers, and only one can lay carbon.**

| Shop | Machines (from its own pages) | Continuous fibre |
|---|---|---|
| Idea Forge, Rapid Prototyping Lab | Markforged X7 and Onyx One | The X7 lays continuous carbon. Request through the Idea Portal; "parts are printed at cost"; contact named on the page is Patrick Maguire |
| ITLL | Markforged Onyx Pro (its equipment page) | Fibreglass only on that model (from memory of the maker's range, not confirmed with staff). Request form "Markforged Onyx Print" |

The $0.25 per gram (Onyx) and $0.35 per gram (carbon reinforced) figures match the Idea
Forge's cost sheet. How the $0.35 is applied (whole part or fibre only) is not stated.

**Decision: continuous carbon fibre in `dock_cam` only, and only in its flat base.**

- Fibre is laid flat inside a layer. It stiffens a flat plate against bending and does
  nothing for an upright wall, where the load crosses the layers.
- `dock_cam`'s base (3 mm dock, 6 mm tongue) is the spring the camera post nods on: the
  one place fibre pays. Ask for it as a sandwich: fibre layers near the bottom and near
  the top of the base, none in the upright cheeks.
- `pod_base` flexes in its upright side panels, `pod_lid` is 1.6 mm thick and lightly
  loaded, `cam_plate_nose` is small and stiff already: plain Onyx for all three.
- Not modelled: how far fibre moves the post's nodding mode (187 Hz in PETG at its
  screws). The tap test on the printed part gives the number.

**Estimated mass and cost** (STL volume at 1.2 g/cm3, solid; supports extra)

| File | Bed size, inches | Bed size, mm | Solid Onyx |
|---|---|---|---|
| `dock_cam.stl` | 5.51 x 1.57 x 1.57 | 140 x 40 x 40 | 27.7 g |
| `pod_base.stl` | 3.98 x 2.20 x 1.02 | 101 x 56 x 26 | 18.8 g |
| `pod_lid.stl` | 2.56 x 2.20 x 0.18 | 65 x 56 x 4.6 | 6.5 g |
| `cam_plate_nose.stl` | 1.65 x 1.55 x 0.79 | 42 x 39 x 20 | 5.1 g |

About 58 g. At the quoted rates: about $15 all plain; about $17 if the dock is charged at
$0.35; allow $20 to $25 with support material.

**Supports.** Markforged's slicer (Eiger) places supports itself and they peel away, so
the request asks for its standard supports with the files left in their saved pose.
Places that will get supports and must be cleared: `pod_base`'s two side cable windows
and the blind round pocket under its tail (9.6 mm across, 5.3 mm deep, the cup that drops
over the dock's boss); under `cam_plate_nose`'s plate. `pod_lid` needs none.

**Known risks accepted with this order.** The revision 6 front pins have not been
printed in PLA. The rear cup and cable windows were not reshaped to print without
supports. Both are recoverable on the printed part (clip a pin, dig out a pocket).

## Sources

- Idea Forge, Rapid Prototyping Lab: <https://www.colorado.edu/ideaforge/facilities/rapid-prototyping-lab>
- Idea Forge, Fabrication: <https://www.colorado.edu/ideaforge/fabrication>
- ITLL wiki, Submit a Request: <https://itll-wiki.colorado.edu/bookstack/books/itll-website-usage/page/submit-a-request>
- ITLL wiki, 3D Printing FAQ: <https://itll-wiki.colorado.edu/bookstack/books/3d-printing-equipment/page/3d-printing-faq>
- ITLL wiki, Submit a Request: <https://itll-wiki.colorado.edu/bookstack/books/itll-website-usage/page/submit-a-request>
- Idea Forge rapid prototyping costs: <https://www.colorado.edu/ideaforge/node/338>
