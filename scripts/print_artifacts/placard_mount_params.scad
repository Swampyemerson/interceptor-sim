// =====================================================================
// placard_mount_params.scad -- SHARED PARAMETRIC DIMENSIONS
// AprilTag placard mount for the TBS Source One V6 target drone.
//
// Implements docs/placard_mount.md DECISION 3 (5.2 parts table, 5.3
// assembly, 5.4 frangibility, 6 dimensioned sketch).
//
// EVERY dimension below carries one of two tags:
//    [DOC  L<n>]  taken verbatim from docs/placard_mount.md, line <n>
//    [CHOSEN]     NOT specified anywhere in the doc; chosen conservatively
//                 here and repeated in the DIMENSIONS TO CONFIRM list at
//                 the bottom of this file. Confirm on the first print.
//
// Units: millimetres. Origin conventions are stated per part.
// Nothing here has ever been printed or fitted to hardware. Date 2026-07-25.
// =====================================================================

$fn = 96;                       // [CHOSEN] render smoothness

// ---------------------------------------------------------------------
// 0. Fasteners and stock
// ---------------------------------------------------------------------
m3_clear      = 3.3;   // [CHOSEN] M3 clearance hole. 3.2 is the textbook close
                       //   fit; 3.3 allows for FDM elephant-foot on a Z-axis
                       //   hole without reaming. Nylon M3x10 per [DOC L534/L535].
m3_head_d     = 6.4;   // [CHOSEN] counterbore for an M3 pan head (nylon pan
                       //   heads run ~5.5-6.0 mm) + print clearance
m3_head_h     = 2.4;   // [CHOSEN] counterbore depth
m3_nut_af     = 5.6;   // [CHOSEN] M3 nylon nut across-flats + clearance
m3_nut_h      = 2.6;   // [CHOSEN]

spar_od       = 4.0;   // [DOC L533] "2 x O4 x 0.5 mm wall carbon tube"
spar_id       = 3.0;   // [DOC L533] 4.0 - 2*0.5
spar_fit      = 0.35;  // [CHOSEN] radial clearance for the spar seat (printed
                       //   holes come out undersize; this is a slip fit, the
                       //   rubber bands do the clamping)

panel_t       = 5.0;   // [DOC L496] "5 mm paper-faced foamboard"
slot_w        = 6.0;   // [DOC L531] "6.0 mm x 110 mm slot" (0.5 mm per side on
                       //   a 5 mm panel)

// ---------------------------------------------------------------------
// 1. P1 -- INDEX DISC   [DOC L530, L679]
//    "O75 x 4 mm, 4 x M3 clearance on the frame's top-plate standoff
//     pattern, 24 x M3 holes on a O56 mm circle (15 deg steps), keyed
//     centre boss"
//    Origin: centre of the disc, bottom face on z = 0 (= the top plate's
//    upper surface, the datum used throughout placard_mount.md 6).
// ---------------------------------------------------------------------
disc_d        = 75.0;  // [DOC L530/L679]
disc_t        = 4.0;   // [DOC L530/L679]
index_pcd     = 56.0;  // [DOC L530] pitch circle of the 24 index holes
index_n       = 24;    // [DOC L530] 24 holes = 15 deg steps [DOC L721]
                       //   the shoe uses any DIAMETRICALLY OPPOSED pair [DOC L551]

// --- the frame interface. THIS IS THE BIGGEST UNKNOWN IN THE FILE. ---
// placard_mount.md L549 says only "the four top-plate standoff positions".
// The doc records the standoff LENGTHS (30/22 mm, L75) but NOWHERE records
// the standoff BOLT PATTERN of the TBS Source One V6 top plate, and no
// vendor drawing is cited anywhere in the repo.
standoff_dx     = 30.0; // [CHOSEN] MEASURE THIS ON THE FRAME BEFORE PRINTING
standoff_dy     = 30.0; // [CHOSEN] MEASURE THIS ON THE FRAME BEFORE PRINTING
standoff_rot    = 0.0;  // [CHOSEN] deg. Many 5" frames sit the top-plate
                        //   standoffs on the 45 deg diagonal -- if so, set 45.

// --- keyed centre boss [DOC L530 names it; gives no dimension] ---
boss_d        = 12.0;  // [CHOSEN] mates the shoe's pocket; carries the SHEAR so
                       //   the two index bolts see mostly clamp load
boss_h        = 3.0;   // [CHOSEN]
boss_key      = 2.5;   // [CHOSEN] depth of the flat that keys rotation
boss_fit      = 0.30;  // [CHOSEN] radial clearance in the shoe's pocket

disc_lighten  = false; // [CHOSEN] optional lightening pockets -- see the mass
                       //   note in DIMENSIONS TO CONFIRM #6
disc_light_d  = 9.0;   // [CHOSEN]
disc_light_pcd = 42.0; // [CHOSEN]
disc_light_n  = 6;     // [CHOSEN]

// ---------------------------------------------------------------------
// 2. P2 -- SHOE   [DOC L531, L555, L676-677]
//    "~110 x 40 x 24 mm: 6.0 mm x 110 mm slot 18 mm deep, two spar
//     saddles, two rubber-band hooks, 1.5 mm frangible web at the slot
//     base"
//    Origin: centre of the base, bottom face on z = 0 (it sits on the
//    disc's TOP face, i.e. global z = disc_t).
// ---------------------------------------------------------------------
shoe_h        = 24.0;  // [DOC L531]
shoe_w        = 40.0;  // [DOC L531]
slot_len      = 110.0; // [DOC L531]
slot_depth    = 18.0;  // [DOC L531] and [DOC L555] "the panel's lower 18 mm
                       //   sits in the slot"
base_t        = shoe_h - slot_depth;   // = 6.0  [DERIVED from DOC L531]
// GEOMETRIC CONSISTENCY CHECK (this is why the numbers above are trustworthy):
//   disc top      = 4 mm above the top plate            [DOC L530]
//   + base_t      = 6 mm                                [DERIVED]
//   = slot floor  = 10 mm above the top plate
//   which is exactly [DOC L499/L671] "panel bottom edge +10 mm above the top
//   plate". The doc's three independent numbers close. Good sign.

shoe_len      = 130.0; // [CHOSEN, DEVIATION from DOC L531's "~110"] the SLOT is
                       //   110 (as documented); the body is 10 mm longer at each
                       //   end to carry the band posts OUTBOARD of the slot, so a
                       //   band can cross the spar where the spar is exposed.
                       //   See DIMENSIONS TO CONFIRM #3.
wall_t        = 5.0;   // [CHOSEN] slot wall thickness. The doc's 40 mm width
                       //   would imply 17 mm solid walls (~125 g of PETG solid);
                       //   the 40 mm is treated as the BASE FLANGE envelope.

// --- spar saddle [DOC L531 "two spar saddles"; no dimensions given] ---
saddle_depth  = 1.5;   // [CHOSEN] how far the spar seat is sunk below the slot
                       //   floor, so the spar -- not the foam -- carries the load
                       //   ([DOC L486] "the foam never carries a mount load")
saddle_continuous = true; // [CHOSEN, DEVIATION] one continuous 110 mm seat
                          //   instead of two discrete saddles: more bearing
                          //   area, stiffer, one less print artefact. See
                          //   DIMENSIONS TO CONFIRM #4.

// --- the frangible web [DOC L531 "1.5 mm frangible web at the slot base"] ---
web_t         = 1.5;   // [DOC L531] -- KEPT AS DOCUMENTED, but see the
                       //   FRANGIBILITY WARNING below. This is the single most
                       //   important number to settle on the bench.
web_h         = 2.0;   // [CHOSEN] vertical extent of the necked band
web_relief    = 1.0;   // [CHOSEN] extra relief so the neck is a clean notch

// --- rubber-band posts [DOC L531 "two rubber-band hooks"; no dimensions] ---
post_x        = 60.0;  // [CHOSEN] outboard of the slot ends (slot ends at 55)
post_y        = 6.0;   // [CHOSEN] half-spacing; the spar passes between them
post_d        = 4.0;   // [CHOSEN]
post_h        = 12.0;  // [CHOSEN] tall enough that the band crosses ABOVE the
                       //   spar's crown and presses it into the seat
post_head_d   = 6.5;   // [CHOSEN] mushroom head so a #64 band cannot walk off
post_head_h   = 2.0;   // [CHOSEN]

// ---------------------------------------------------------------------
// 3. P8 -- SPAR END CAP   [DOC L537]
//    "2 x spar end cap ... placard_spar_cap.stl" -- NO dimensions given
//    anywhere. Everything here is [CHOSEN].
//    Origin: centre of the flange, flange bottom on z = 0.
// ---------------------------------------------------------------------
cap_plug_d    = spar_id - 0.15;  // [CHOSEN] 2.85 -- light interference in a
                                 //   3.0 mm ID tube; glue with a drop of CA
cap_plug_h    = 6.0;             // [CHOSEN]
cap_flange_d  = 5.5;             // [CHOSEN] slightly proud of the 4.0 mm tube
cap_flange_h  = 2.0;             // [CHOSEN]
cap_chamfer   = 0.6;             // [CHOSEN] chamfer on the exposed outer face
                                 //   (it is the part that hits the ground
                                 //   first). A chamfer, not a dome: a dome on a
                                 //   2 g part would need supports.

// =====================================================================
// FRANGIBILITY WARNING -- READ BEFORE PRINTING
// =====================================================================
// docs/placard_mount.md 5.4 (L561-565) sets "TARGET RELEASE 60-80 N", derived
// from a worst inertial load of "193 g at 10 g = 19 N". Those two numbers are
// quoted as bare FORCES, but they act at DIFFERENT LEVER ARMS above the web:
// the inertial load acts at the panel's area centre (+235 mm, 4.2 table) while
// the 9-item-7 pull test applies its force at the panel's TOP EDGE (+450 mm).
// Converted to moments about the web:
//
//     worst flight load  19 N @ 235 mm  =  4.46 N.m   ( = 9.9 N at the top edge)
//     stated release  60-80 N @ 450 mm  =  27-36 N.m  ( = 6-8x the flight load)
//
// And the documented web itself, in bending (sigma = M/Z, Z = b*t^2/6,
// b = 110 mm, PETG ~50 MPa [ESTIMATE]):
//
//     web t (mm)    M_break (N.m)    equivalent pull at the panel top edge (N)
//        1.5            2.06                        4.6      <-- AS DOCUMENTED
//        2.5            5.73                       12.7
//        3.5           11.23                       25.0
//        5.5           27.73                       61.6      <-- hits 60-80 N
//        6.0           33.00                       73.3
//
// So, as documented, the 1.5 mm web releases at ~4.6 N at the top edge --
// BELOW the doc's own 19 N worst flight load (9.9 N equivalent), i.e. it would
// shed the panel in hard flight, not just in a crash. Conversely, reaching the
// stated 60-80 N band needs t ~ 5.5-6.0 mm, which is a nearly solid 6 mm base
// with no frangible feature left at all.
//
// This file KEEPS web_t = 1.5 because that is what the doc specifies -- a
// generator does not get to silently rewrite a documented dimension. It is
// raised as DIMENSIONS TO CONFIRM #1 and it is exactly what 9 item 7 (luggage
// scale pull test) exists to settle. A self-consistent target, 2-3x the worst
// flight moment, would be 9-13 N.m => web_t ~ 3.2-3.7 mm => 20-30 N at the top
// edge. Print one shoe at 1.5, one at 3.5, and pull both.
//
// =====================================================================
// DIMENSIONS TO CONFIRM ON PRINT  (repeated verbatim in the worker report)
// =====================================================================
// 1. web_t = 1.5 mm [DOC L531] -- predicted release ~4.6 N at the panel top
//    edge, vs the doc's stated 60-80 N target and its own 19 N worst flight
//    load. Numbers do not close (see above). BENCH: 9 item 7 pull test; print
//    1.5 mm and 3.5 mm shoes and pull both.
// 2. standoff_dx / standoff_dy / standoff_rot = 30 / 30 / 0 [CHOSEN] -- the
//    TBS Source One V6 top-plate standoff BOLT PATTERN is recorded NOWHERE in
//    the repo (L75 records standoff LENGTHS only). MEASURE the real frame with
//    calipers before printing the disc. Everything else on the disc is doc-fixed.
// 3. shoe_len = 130 mm [CHOSEN, deviates from L531's "~110 x 40 x 24"] -- the
//    slot stays 110 mm as documented; the body grows 10 mm at each end so the
//    band posts sit OUTBOARD of the slot and a #64 band can cross the exposed
//    spar. If the doc means a strict 110 mm body, the bands need another
//    anchoring scheme (L531 gives none).
// 4. saddle_continuous = true [CHOSEN, deviates from L531's "two spar saddles"]
//    -- implemented as one continuous 110 mm seat. Set false only if a real
//    reason appears; two discrete saddles halve the bearing area.
// 5. wall_t = 5 mm [CHOSEN] -- L531's 40 mm width taken as the BASE FLANGE
//    envelope, not as 17 mm solid walls (which would be ~125 g of solid PETG
//    against the doc's 22 g estimate at L531/L303).
// 6. MASS. Solid PETG (1.27 g/cm3): disc ~17.7 cm3 = 22 g, shoe ~52 cm3 = 66 g,
//    against L300-L303's 8 g / 22 g estimates. At 3 perimeters + 20 % infill
//    they land nearer ~10 g / ~28 g -- still over. WEIGH the first prints and
//    correct placard_mount.md 4.1's 41 g mount total (it feeds the CG in 4.2).
//    Set disc_lighten = true if the disc needs to come down.
// 7. boss_d / boss_h / boss_key = 12 / 3 / 2.5 [CHOSEN] -- L530 says "keyed
//    centre boss" and gives no dimension. Check it clears the flight stack and
//    the top plate's centre cutout on the real frame.
// 8. m3_clear = 3.3 [CHOSEN] -- if your printer runs tight, ream to 3.2-3.4.
//    NYLON screws only, no steel: L642/L573 (the M10Q-5883 compass).
// 9. post_* (band posts) and cap_* (spar end cap) -- L531 and L537 name these
//    parts and give NO dimensions at all. Everything is chosen. Check a real
//    #64 band reaches over the spar with useful preload, and that the cap is a
//    firm push fit in a real 3.0 mm ID tube.
// 10. saddle_depth = 1.5 mm [CHOSEN] assumes the bottom spar protrudes ~1.5 mm
//    PROUD of the foam's bottom edge, so the load path is spar -> saddle ->
//    shoe and the foam carries nothing (L486). L553 says "epoxy P4 in flush",
//    which could equally mean fully recessed -- in which case the saddle never
//    touches the spar and the FOAM carries the mount load. Settle this when the
//    panel is built; it decides whether saddle_depth is 1.5 or 0.
