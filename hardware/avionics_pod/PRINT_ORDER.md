# Avionics pod: what to print where (decided 2026-11-11)

The builder can send parts to the ITLL at CU Boulder, which runs Markforged printers.
This is the print plan for pod revision 5. Prices and the fibre option are **not
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
| Template | `dock_cam_coupon.stl` | PLA, self-service Prusa | A 5 minute, few-cent fit check. Do this before paying for anything |
| Dock with camera post | `dock_cam.stl` | **Onyx, solid fill** | The part every vibration number depends on. Ask for fibreglass in the tongue if offered (below) |
| Camera plate | `cam_plate_nose.stl` | **Onyx, solid fill** | Same load path as the post |
| Pod base | `pod_base.stl` | **Onyx, solid fill** | Carries the Pi and sits over the Pixhawk: most of the flex in the pod's sway mode is in this part, and it is the one next to the heat |
| Pod lid | `pod_lid.stl` | Onyx if it rides along in the same order, PETG otherwise | Lightly loaded; material matters least here |

Not printing: `dock`, `dock_coupon` (the plain dock without a camera post), `pod_lid_cam`
and `cam_plate` (the lid-cradle option that was not chosen).

**Settings to ask for**

- **Orientation:** flat face on the bed, no supports; all four were drawn for that.
  `dock_cam`, `cam_plate_nose` and `pod_base` sit as modelled. `pod_lid` goes upside down
  from the model (flat top on the bed, rails up).
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

1. Print `dock_cam_coupon.stl` in PLA and lay it on the interceptor's top plate. Six
   holes must line up and the tongue must stop short of the plate's nose edge.
2. Look at where the wires come up from the stack. Revision 5 has no cable slot; they are
   meant to pass beside the plate into the pod's side windows.
3. Ask ITLL staff: is a personal project eligible and under which purpose, the price per
   gram, whether continuous fibre is available and in which material, and turnaround.
4. Submit the four flight parts as one request.
5. On the printed parts: weigh them, fit them, then the tap test in `ANALYSIS.md`.

## Sources

- ITLL wiki, 3D Printing FAQ: <https://itll-wiki.colorado.edu/bookstack/books/3d-printing-equipment/page/3d-printing-faq>
- ITLL wiki, Submit a Request: <https://itll-wiki.colorado.edu/bookstack/books/itll-website-usage/page/submit-a-request>
- Idea Forge rapid prototyping costs: <https://www.colorado.edu/ideaforge/node/338>
