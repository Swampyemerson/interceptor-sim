// =====================================================================
// placard_index_disc.scad  --  P1, the 15-degree indexable disc
//
// docs/placard_mount.md 5.2 L530:
//   "Index disc, O75 x 4 mm, 4 x M3 clearance on the frame's top-plate
//    standoff pattern, 24 x M3 holes on a O56 mm circle (15 deg steps),
//    keyed centre boss | PETG | 8 g | print -- placard_index_disc.stl"
// docs/placard_mount.md 5.3 L549:
//   "P1 bolts to the four top-plate standoff positions with P5 (longer
//    nylon screws into the frame's existing 30 mm standoffs -- no new holes)"
// docs/placard_mount.md 6 L679:  "INDEX DISC (P1) O75 x 4, 24 holes @15deg"
// docs/placard_mount.md 8 L721:  index steps 0, +/-15, +/-30, +/-45, +/-60,
//                                +/-75, 90 deg
//
// WHY THIS PART EXISTS (placard_mount.md 5.6): it makes the placard's YAW
// ASPECT -- the thing the design is least certain about (3.3: the usable
// incidence cone is only about +/-32 deg at the deployed quad_decimate) -- a
// two-minute field change instead of a rebuild, without drilling the carbon.
//
// ORIGIN: centre of the disc, bottom face on z = 0, which is the top plate's
// upper surface (the datum for the whole 6 sketch).
//
// PRINT (all [CHOSEN]): PETG, 0.2 mm layers, 4 perimeters, 40 % infill, flat
// on the bed, NO supports. Printed flat, the 24 index holes and the 4 mounting
// holes are all Z-axis holes and come out slightly undersize -- m3_clear is
// 3.3 for that reason. NO STEEL AND NO MAGNETS anywhere in this assembly
// (placard_mount.md 5.4 L573 / 5.6 L642: the M10Q-5883 carries the compass).
//
// Nothing here has been printed or fitted. Read the DIMENSIONS TO CONFIRM
// list at the bottom of placard_mount_params.scad first -- item 2 (the frame's
// standoff bolt pattern) is a genuine unknown and it is in THIS part.
// =====================================================================

include <placard_mount_params.scad>

module key_boss(d, h, key, extra = 0) {
    // A cylinder with one flat -- the flat is what stops the shoe rotating on
    // the boss, so the two index bolts carry clamp load rather than shear.
    difference() {
        cylinder(h = h, d = d + 2 * extra);
        translate([d / 2 + extra - key, -d, -0.5])
            cube([d, 2 * d, h + 1]);
    }
}

module placard_index_disc() {
    difference() {
        union() {
            cylinder(h = disc_t, d = disc_d);                       // [DOC L530]
            translate([0, 0, disc_t])
                key_boss(boss_d, boss_h, boss_key);                 // [CHOSEN]
        }

        // 24 index holes on the O56 pitch circle, 15 deg apart.   [DOC L530]
        // The shoe uses any DIAMETRICALLY OPPOSED pair [DOC L551], which is
        // what makes the available settings 0, +/-15 ... 90 deg [DOC L721].
        for (i = [0 : index_n - 1])
            rotate([0, 0, i * (360 / index_n)])
                translate([index_pcd / 2, 0, -1])
                    cylinder(h = disc_t + 2, d = m3_clear);

        // 4 x M3 clearance on the frame's top-plate standoff pattern.
        // [DOC L530/L549] -- PATTERN ITSELF IS [CHOSEN], see TO-CONFIRM #2.
        rotate([0, 0, standoff_rot])
            for (sx = [-1, 1], sy = [-1, 1])
                translate([sx * standoff_dx / 2, sy * standoff_dy / 2, -1])
                    cylinder(h = disc_t + 2, d = m3_clear);

        // Optional lightening pockets (TO-CONFIRM #6, mass).
        if (disc_lighten)
            for (i = [0 : disc_light_n - 1])
                rotate([0, 0, i * (360 / disc_light_n) + 180 / disc_light_n])
                    translate([disc_light_pcd / 2, 0, -1])
                        cylinder(h = disc_t + 2, d = disc_light_d);

        // 0 deg index witness notch -- placard_mount.md 5.3 L552 tells you to
        // mark 0 deg with a paint pen so a field re-index cannot be got wrong.
        // A printed notch cannot rub off. [CHOSEN]
        translate([disc_d / 2 - 1.2, -1.0, disc_t - 0.8])
            cube([2.4, 2.0, 1.6]);
    }
}

placard_index_disc();
