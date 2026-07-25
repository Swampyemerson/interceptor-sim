// =====================================================================
// placard_shoe.scad  --  P2, the SACRIFICIAL shoe.  PRINT FOUR.
//
// docs/placard_mount.md 5.2 L531:
//   "Shoe, ~110 x 40 x 24 mm: 6.0 mm x 110 mm slot 18 mm deep, two spar
//    saddles, two rubber-band hooks, 1.5 mm frangible web at the slot base
//    | PETG | 22 g | print -- placard_shoe.stl (x4: 1 + 3 spares)"
// docs/placard_mount.md 5.3 L551-552:
//   "P2 bolts to P1 through any diametrically-opposed pair of the 15 deg
//    holes. That bolt pair IS the index setting."
// docs/placard_mount.md 5.3 L555:
//   "Drop the bottom spar into the shoe's saddles; the panel's lower 18 mm
//    sits in the slot. Loop P7's two rubber bands over the spar into the
//    shoe hooks; add the hook-and-loop strap as a secondary."
// docs/placard_mount.md 5.4 L561-565: TARGET RELEASE 60-80 N.
// docs/placard_mount.md 6 L676-677: the shoe + "1.5 mm FRANGIBLE WEB".
//
// WHY THIS PART EXISTS (placard_mount.md 5.6 L646-648): it puts a 30-cent
// printed part in the crash load path of an aircraft the BOM itself describes
// as "the target takes the hard landings". It is MEANT to break. Print four.
//
// *** READ THE FRANGIBILITY WARNING IN placard_mount_params.scad. ***
// The documented 1.5 mm web is predicted to release at ~4.6 N applied at the
// panel's top edge, which is BELOW the doc's own 19 N worst flight load and
// 13x below its stated 60-80 N target. web_t is a parameter for exactly that
// reason: print one at 1.5 and one at 3.5 and run the 9-item-7 pull test.
//
// ORIGIN: centre of the base; bottom face on z = 0, which sits on the index
// disc's TOP face (global z = disc_t = 4 mm above the top plate).
//
// LOAD PATH (placard_mount.md 4.6 L486 "the foam never carries a mount load"):
//   panel -> bonded bottom spar -> saddle seat -> base -> 2 x M3 -> disc.
//   The slot's walls locate the panel; they do not carry it.
//
// PRINT ORIENTATION MATTERS HERE (all [CHOSEN]): PETG, 0.2 mm layers, 4
// perimeters, 30 % infill, printed FLAT (base on the bed, slot opening up),
// no supports needed. Printed this way the frangible web's layer lines run
// HORIZONTALLY, i.e. the web breaks along a layer boundary -- which makes the
// release load repeatable but LOWER than the bulk-material estimate. That is
// the direction you want for a sacrificial part, but it is another reason the
// pull test, not this file, sets the final web_t.
// NO STEEL, NO MAGNETS (placard_mount.md 5.4 L573 -- the M10Q-5883 compass).
// =====================================================================

include <placard_mount_params.scad>

wall_in  = slot_w / 2;              // inner face of each slot wall
wall_out = slot_w / 2 + wall_t;     // outer face of each slot wall
spar_seat_r = spar_od / 2 + spar_fit;
bolt_x   = index_pcd / 2;           // 28.0 -- the diametrically opposed pair
                                    // on the disc's O56 circle [DOC L530/L551]

module band_post() {
    // Mushroom post. A #64 band hooks over the pair of posts at one end of
    // the shoe and crosses ABOVE the exposed spar, pressing it into the seat.
    // [DOC L531] names "two rubber-band hooks" and gives no geometry -- all
    // of this is [CHOSEN], TO-CONFIRM #9.
    cylinder(h = post_h, d = post_d);
    translate([0, 0, post_h - post_head_h])
        cylinder(h = post_head_h, d1 = post_head_d, d2 = post_d);
}

module placard_shoe() {
    difference() {
        union() {
            // ---- base flange ----------------------------------------
            // 40 mm width is [DOC L531]; the length is [CHOSEN] 130 so the
            // band posts sit outboard of the 110 mm slot (TO-CONFIRM #3).
            translate([-shoe_len / 2, -shoe_w / 2, 0])
                cube([shoe_len, shoe_w, base_t]);

            // ---- the two slot walls, standing on the FRANGIBLE WEB ----
            // Above the web the wall is full thickness; through the web band
            // it necks to web_t. That neck is the designed break.  [DOC L531]
            for (s = [-1, 1]) {
                // necked web section, z = [base_t, base_t + web_h]
                translate([-slot_len / 2, s * (wall_in + wall_t / 2) - web_t / 2,
                           base_t])
                    cube([slot_len, web_t, web_h]);
                // full wall above the web
                translate([-slot_len / 2, s == -1 ? -wall_out : wall_in,
                           base_t + web_h])
                    cube([slot_len, wall_t, shoe_h - base_t - web_h]);
            }

            // ---- rubber-band posts, on the BASE (below the web) -------
            // Deliberately below the web so the band's load reacts straight
            // into the base and the disc: the band and the web are the TWO
            // INDEPENDENT frangible paths placard_mount.md 5.4 L567 asks for,
            // not one in series with the other.
            for (sx = [-1, 1], sy = [-1, 1])
                translate([sx * post_x, sy * post_y, base_t])
                    band_post();
        }

        // ---- spar seat along the slot floor -------------------------
        // [DOC L531] "two spar saddles"; implemented as one continuous seat
        // (TO-CONFIRM #4). Runs the full body length so the spar is supported
        // right out to where the bands cross it.
        translate([-shoe_len / 2 - 1, 0, base_t - saddle_depth + spar_seat_r])
            rotate([0, 90, 0])
                cylinder(h = shoe_len + 2, r = spar_seat_r);
        // ...and open the seat upward so the spar can be dropped in, not
        // threaded in [DOC L555 "Drop the bottom spar into the shoe's saddles"]
        translate([-shoe_len / 2 - 1, -spar_seat_r,
                   base_t - saddle_depth + spar_seat_r])
            cube([shoe_len + 2, 2 * spar_seat_r, spar_seat_r + 1]);

        // ---- 2 x M3 to the disc, on the O56 pair ---------------------
        // [DOC L530/L551]. Counterbored so the head sits BELOW the spar seat;
        // the spar bridges the 6.4 mm counterbore mouth.
        for (sx = [-1, 1]) {
            translate([sx * bolt_x, 0, -1])
                cylinder(h = base_t + 2, d = m3_clear);
            translate([sx * bolt_x, 0, base_t - saddle_depth - m3_head_h])
                cylinder(h = m3_head_h + 0.01, d = m3_head_d);
        }

        // ---- keyed pocket for the disc's centre boss -----------------
        // [CHOSEN] TO-CONFIRM #7. Carries shear so the index bolts do not.
        translate([0, 0, -0.01])
            difference() {
                cylinder(h = boss_h + boss_fit, d = boss_d + 2 * boss_fit);
                translate([boss_d / 2 + boss_fit - boss_key, -boss_d, -0.5])
                    cube([boss_d, 2 * boss_d, boss_h + boss_fit + 1]);
            }

        // ---- web relief notches --------------------------------------
        // Clean re-entrant corners at the neck so the break is a notch, not a
        // ragged tear, and so a slicer cannot bridge the neck solid. [CHOSEN]
        for (s = [-1, 1]) {
            translate([-slot_len / 2 - 1,
                       s * (wall_in + wall_t / 2) + web_t / 2, base_t])
                cube([slot_len + 2, wall_t / 2 + web_relief, web_h]);
            translate([-slot_len / 2 - 1,
                       s * (wall_in + wall_t / 2) - web_t / 2
                       - (wall_t / 2 + web_relief), base_t])
                cube([slot_len + 2, wall_t / 2 + web_relief, web_h]);
        }

        // ---- panel-entry chamfer on the slot mouth -------------------
        // [CHOSEN] makes fitting the panel in the field a one-hand job.
        // A square bar rotated 45 deg and sat on the wall's top inner edge
        // takes a ~1.2 mm chamfer off both faces of that corner.
        for (s = [-1, 1])
            translate([0, s * wall_in, shoe_h])
                rotate([45, 0, 0])
                    cube([slot_len + 2, 1.7, 1.7], center = true);
    }
}

placard_shoe();
