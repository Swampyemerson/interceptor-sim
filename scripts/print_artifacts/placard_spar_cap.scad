// =====================================================================
// placard_spar_cap.scad  --  P8, carbon spar end cap.  PRINT FOUR.
//
// docs/placard_mount.md 5.2 L537:
//   "P8 | 2 x spar end cap | PETG (print) | 2 g | print -- placard_spar_cap.stl"
// docs/placard_mount.md 5.2 L533 (the tube it caps):
//   "P4 | 2 x O4 x 0.5 mm wall carbon tube, 480 mm (protrudes 15 mm each side
//    of the bottom panel edge)"
// docs/placard_mount.md 7 L744: "placard_spar_cap.stl x 4"
//   (four, not two -- both spars, both ends; L537's "2 g" is the mass of a
//    pair. The build checklist is the authority on the count.)
//
// WHY: a raw carbon tube end is a splintering hazard and a snag point, and the
// bottom spar's protruding ends are the first thing to hit the ground when the
// frangible shoe does its job and the panel departs (5.4). Capping them also
// keeps the tube's bore clean so the cap can be re-fitted after a crash.
//
// *** EVERY DIMENSION IN THIS PART IS [CHOSEN]. *** docs/placard_mount.md
// names the part and gives no geometry whatsoever. See DIMENSIONS TO CONFIRM
// #9 in placard_mount_params.scad.
//
// ORIGIN: centre of the flange, flange bottom face on z = 0, plug pointing +z.
//
// PRINT (all [CHOSEN]): PETG, 0.15 mm layers, 3 perimeters, 60 % infill,
// flange DOWN on the bed, no supports. Small part -- print all four at once
// with a brim. If the plug is tight, sand it; do not force it into the tube
// (0.5 mm carbon wall splits).
// =====================================================================

include <placard_mount_params.scad>

module placard_spar_cap() {
    union() {
        // flange -- slightly proud of the 4.0 mm tube OD so it also acts as a
        // bump stop when the spar end grounds. The OUTER face is z = 0 and it
        // is printed flat on the bed, so the "rounded end" is a chamfer, not a
        // dome: a dome would need supports on a 2 g part.
        cylinder(h = cap_chamfer, d1 = cap_flange_d - 2 * cap_chamfer,
                 d2 = cap_flange_d);
        translate([0, 0, cap_chamfer])
            cylinder(h = cap_flange_h - cap_chamfer, d = cap_flange_d);

        // plug -- light interference in the tube's 3.0 mm bore, with a lead-in
        translate([0, 0, cap_flange_h]) {
            cylinder(h = cap_plug_h - 1.0, d = cap_plug_d);
            translate([0, 0, cap_plug_h - 1.0])
                cylinder(h = 1.0, d1 = cap_plug_d, d2 = cap_plug_d - 0.6);
        }
    }
}

placard_spar_cap();
