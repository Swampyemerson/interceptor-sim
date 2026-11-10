# Avionics pod: SOLIDWORKS cross-check of the nose post and dock (2026-11-10)

**What this is:** finite-element runs made in SOLIDWORKS Simulation 2025 SP5 (Student
Edition, academic use only) on `avionics_pod_assembly_nose_camera.step`, as an independent
check on `fea_modal.py` and as a first static-load check. Like `ANALYSIS.md`, these are
calculations on the CAD and on estimated part weights. Nothing here was measured on the
aircraft. Each section says what it rests on. Section 7 lists suggested repo changes.

**Follow-up in the repo (same day):** changes 1 to 4 of section 7 are done; see
`ANALYSIS.md` section 2 for the Python reruns. The results in section 8 are also in
`solidworks_results.json`, and the dock stress picture is
`media/solidworks_dock_static_10g_pod_lump_stress.png` at the repo root.

## 1. Summary

- **The Python modal model holds up.** With the dock underside held flat, SOLIDWORKS gives
  275 Hz for the nose post's first mode against 254.5 Hz from `fea_modal.py --nose`. All
  six modes agree within 1 to 8 %, in the same order and directions.
- **Held only at its six screws, the nose post's first mode drops to 173 Hz.** That is 37 %
  below the SOLIDWORKS glued case and 32 % below the Python figure, inside the
  "20 to 40 % lower" judgment in `ANALYSIS.md` section 2.
- **With the pod's weight on the dock, a new first mode appears at 127 Hz.** It is the pod
  pitching on the dock. The Pixhawk rides in the pod, so this mode reaches its sensors.
  Hover motor rate is about 213 Hz, so 127, 173 and 185 Hz are all crossed on spool-up.
- **Static strength is not the limit.** At 10 g on all three axes at once (17.3 g
  combined) the camera post peaks at 1.0 MPa. The dock carrying the pod peaks at 27.1 MPa,
  but only at the edge of one rigidly held screw hole; away from hole edges it stays under
  about 6 MPa.
- **`pod_base` could not be meshed in SOLIDWORKS,** so there is no stress result inside the
  pod base. The pod is represented as a 162 g lump.

## 2. Setup common to all runs

- **Geometry:** the STEP import, unmodified. Frame matches `pod.py` (x forward, y
  left-right, z up from the top plate). Checked with the Pi board's centre of mass, which
  reads z = 29.7 mm in SOLIDWORKS against 29.65 mm from `pod.py`.
- **Bodies included:** `dock_cam`, `cam_plate_nose`, camera stand-in. Everything else is
  excluded from analysis.
- **Joints:** global bonded interaction with the bonding gap raised to 0.35 % of model size
  (0.64 mm), so the plate ears bond to the cheeks across their 0.5 mm clearance. No screws
  are modelled. This is stiffer than the four 6 mm steel discs in `fea_modal.py`.
- **Mesh:** blended curvature-based solid mesh, 3 mm maximum element for frequency
  studies, 2 mm for static studies.

| Material | E (MPa) | Poisson | Density (kg/m3) | Yield (MPa) | Source |
|---|---|---|---|---|---|
| Printed PETG | 1900 | 0.38 | 1143 | 45 (assumed) | `fea_modal.py` defaults; 1.27 g/cc times `PRINT_FILL` 0.9 |
| Camera stand-in, 15 g | 5000 | 0.30 | 2860 | placeholder | density set so the body weighs 15 g |

The 45 MPa yield (and 50 MPa tensile) is a typical figure for printed PETG that I entered
because a static study requires one. It is not from the repo and not measured. Across
layers, 25 MPa is a more cautious figure.

## 3. Vibration

Motor forcing from `docs/airframe_numbers.md` section 7: about 213 Hz in hover, about
583 Hz at full throttle.

| Mode | A. Underside held flat | B. Six screw holes held | C. Six screw holes held, pod lump added | Python `nose_post.json` |
|---|---|---|---|---|
| 1 | 275 | 173 | 127.44 | 254.5 |
| 2 | 293 | 184 | 173.09 | 275.7 |
| 3 | 552 | 410 | 184.57 | 544.5 |
| 4 | 662 | 414 | 414.50 | 639.4 |
| 5 | 1011 | 549 | 495.95 | 945.2 |
| 6 | 1016 | 731 | 548.76 | 957.3 |

All values in Hz. Cases A and B were recorded to the nearest 1 Hz.

- **Case A** (study "Nose post modal"): whole underside face of `dock_cam` fixed. Same
  intent as the Python clamp, which holds the central 29 mm strip. SOLIDWORKS reads 1 to
  8 % higher because the full underside and the full ear-to-cheek overlap are glued.
- **Case B** (study "Nose post bolted"): only the walls of the six M3 holes fixed (four
  dock holes plus the nose pair). The whole post now rocks from its base on the tongue.
- **Case C** (study "Dock modal 6 screws pod lump"): case B plus the removable pod as a
  rigid 162 g mass at its centre of mass (-7.1, 0, 29.4) mm from `mass_report.json`,
  rigidly tied to the dock's two rear top faces. Rotational inertia entered as
  7.6e-5, 1.31e-4 and 1.40e-4 kg.m2 about x, y and z, estimated from a uniform block
  85 x 56 x 50 mm.

Mass participation for case C (normalized):

| Mode | Hz | x (fore-aft) | y (side to side) | z (up-down) | What moves |
|---|---|---|---|---|---|
| 1 | 127.44 | 0.285 | 0.000 | 0.266 | pod pitching on the dock, tail tab bending |
| 2 | 173.09 | 0.000 | 0.103 | 0.000 | camera post swaying side to side (same as case B mode 1) |
| 3 | 184.57 | 0.058 | 0.000 | 0.008 | camera post fore-aft (same as case B mode 2) |
| 4 | 414.50 | 0.015 | 0.000 | 0.123 | |
| 5 | 495.95 | 0.000 | 0.579 | 0.000 | pod side to side on the dock |
| 6 | 548.76 | 0.000 | 0.000 | 0.000 | |

**Reading:**

- The pod adds two modes of its own (127 and 496 Hz) and leaves the camera post's modes
  where case B put them.
- 127 Hz is 40 % below the hover motor rate. Modes 1 to 3 are crossed on every spool-up
  and modes 4 to 6 sit inside the 213 to 583 Hz band.
- `ANALYSIS.md` concludes "bolt the dock" to get the Pixhawk away from the foam-tape
  rocking mode. This run says the bolted dock still has a pod pitch mode that needs
  checking on hardware.

**What it rests on, and which way each assumption pushes:**

- Fixing the hole walls ignores the carbon top plate under the dock and the screw heads
  clamping it. Real frequencies should be higher than cases B and C on that count.
- The rigid tie between the pod lump and the dock top faces, and the glued plate-to-cheek
  joint, are stiffer than the real hook, pins, tail screw and M3 pivots. Real frequencies
  should be lower on that count.
- Printed PETG as one uniform stiffness. The softer-plastic row in `ANALYSIS.md`
  (E = 1300 MPa) was not rerun; scaling by the Python ratio gives about 232 Hz for case A.
- Pod mass and centre of mass come from `pod.py` board weights marked MEASURE.

**Would test it:** the tap test already listed in `ANALYSIS.md`, done with the dock bolted
to the frame and the loaded pod latched on, then a Pixhawk vibration log at hover.

## 4. Static load

Load in every case: 98.1 m/s2 along each of the three axes at once (10 g per axis,
17.3 g combined). Only one sign combination was run.

| Study | Hold | What is loaded | Peak von Mises | Peak deflection |
|---|---|---|---|---|
| "Nose post static 10g" | underside held flat | dock, plate, 15 g camera | 1.0 MPa | 0.072 mm at the lens |
| "Dock static 10g pod lump" | six screw-hole walls | same, plus 162 g pod lump | 27.1 MPa at one screw-hole edge | 1.21 mm at the tail tab tip |
| "Pod static 10g" | underside held flat | same, plus `pod_base` and Pi stand-in | not solved | not solved |

- **Camera post:** 1.0 MPa is about 45 times under the assumed 45 MPa yield, about 25
  times under 25 MPa.
- **Dock with the pod's weight:** the 27.1 MPa peak is at the edge of the rear-left dock
  screw hole, where the model holds the hole wall rigid. That is a known artifact of a
  fixed hole face. With the plot capped at 10 MPa and read by colour, the rest of the dock
  is mostly under 3 MPa, with 4 to 6 MPa around the rear holes and the tail tab root, and
  small spots above that at hole edges and one notch corner.
- **Taken at face value** the hotspot gives a factor of 1.7 on 45 MPa and 0.9 on 25 MPa,
  so the screw holes are the place to watch, not the camera post.
- **Hand check on the screws (my estimate, not FEA):** total load is 35 N across six
  screws, about 6 N each, or 0.6 MPa average bearing on a 3 mm x 3.2 mm hole wall.
  Sideways 10 g on the pod gives a 0.47 N.m overturning moment, about 16 N per side
  across the 29.2 mm screw spacing.
- **Tail tab:** the 1.21 mm deflection assumes nothing under the tab. If the carbon plate
  extends under it, downward movement is blocked and the figure applies to lift-off only.
- **Coupling differs from the modal run:** the static study spreads the pod lump's load
  over the dock's two rear top faces (distributed connection). The modal study ties it
  rigidly, because the frequency study type offers no distributed option.

**Would test it:** a pull or pry test on a printed dock bolted to a scrap plate, loaded at
the pod's centre-of-mass height.

## 5. `pod_base` did not mesh

SOLIDWORKS failed to mesh `pod_base` with four settings: blended curvature 2 mm,
curvature 2 mm, standard 1.5 mm with 0.15 mm tolerance, and blended curvature with a
0.05 mm minimum element. Every other body meshed.

Likely cause, from reading `pod.py` (SOLIDWORKS only names the body, so this is unverified):

- Each Pi boss is exactly tangent to the side panel's outer face:
  `PI_HOLE_DY / 2 + BOSS_D / 2` = 24.5 + 3.5 = 28.0 = `PI_W / 2`.
- The panels end exactly one boss radius past the end holes
  (`px0, px1 = HOLE_XS[0] - BOSS_D / 2, HOLE_XS[1] + BOSS_D / 2`), which leaves a strip of
  the panel's inner face 0.18 mm wide and about 23 mm tall beside each end boss.

Rough stand-in (hand estimate, not FEA): each boss as a free-standing 7 mm post with a
3.2 mm bore carrying a quarter of a 105 g stack gives about 1.9 MPa at the root for 10 g
sideways and 3.2 MPa at 17.3 g. Margin above 10 on 45 MPa.

## 6. What was not done

- No harmonic or random vibration run, so no shake amplitude or image-blur figure.
- No softer-plastic (1300 MPa) rerun in SOLIDWORKS.
- No lower-bound modal case with the pod as dead weight and no added stiffness.
- No screws, heat-set inserts, contact or preload.
- No stress result for `pod_base`, `pod_lid` or the Pi bosses.
- Static load in one sign combination only; no crash or impact case.

## 7. Suggested repo changes

These are proposals. Design decisions stay with the builder.

1. **`ANALYSIS.md` section 2:** add the SOLIDWORKS columns from section 3 here. The
   "20 to 40 % lower" line can cite case B (32 to 37 % lower) as one bound instead of
   judgment alone.
2. **`ANALYSIS.md` "bolt the dock" conclusion:** add that a bolted dock modelled at its six
   screws still shows a pod pitch mode near 127 Hz, with the caveats in section 3, and
   move the bolted-dock tap test up the list.
3. **`fea_modal.py`:** add a clamp option that holds only nodes on the six screw-hole
   walls, and an optional rigid pod lump (162 g at the module centre of mass), so the
   Python model can reproduce cases B and C. Targets: 173 and 184 Hz for B; 127, 173 and
   185 Hz for C.
4. **`pod.py` `pod_base`:** remove the tangency and the sliver. Two candidate edits, not
   yet tried: run the side panels 1 mm past the end bosses (`px0 - 1.0`, `px1 + 1.0`), and
   stop the bosses short of the outer face (for example `BOSS_D` 7.0 to 6.6, or panels
   0.3 mm proud of `PI_W / 2`). Then regenerate the STEP and retry the mesh.
5. **Dock design, if the tap test confirms a low pod mode:** options are a seventh
   fastener at the tail tab, a thicker dock or ribs between the rear screws and the tab,
   or confirming the dock seats flat on the carbon plate along its full length.
6. **Weights:** the board masses in `pod.py` are still marked MEASURE. Cases C and the
   dock static run scale with them.

## 8. Results in machine-readable form

```json
{
 "date": "2026-11-10",
 "tool": "SOLIDWORKS Simulation 2025 SP5 Student Edition",
 "model": "avionics_pod_assembly_nose_camera.step",
 "e_petg_mpa": 1900.0,
 "poisson_petg": 0.38,
 "density_petg_kg_m3": 1143.0,
 "yield_petg_mpa_assumed": 45.0,
 "camera_mass_g": 15.0,
 "motor_band_hz": [213, 583],
 "modal": {
  "underside_fixed": {
   "study": "Nose post modal",
   "mesh_max_mm": 3.0,
   "freqs_hz": [275, 293, 552, 662, 1011, 1016],
   "python_nose_post_hz": [254.5, 275.7, 544.5, 639.4, 945.2, 957.3]
  },
  "six_screw_holes_fixed": {
   "study": "Nose post bolted",
   "mesh_max_mm": 3.0,
   "freqs_hz": [173, 184, 410, 414, 549, 731]
  },
  "six_screw_holes_fixed_pod_lump": {
   "study": "Dock modal 6 screws pod lump",
   "mesh_max_mm": 3.0,
   "pod_mass_g": 162.0,
   "pod_cg_mm": [-7.1, 0.0, 29.4],
   "pod_inertia_kg_m2": [7.6e-5, 1.31e-4, 1.4e-4],
   "pod_connection": "rigid, to the two rear top faces of dock_cam",
   "freqs_hz": [127.44, 173.09, 184.57, 414.5, 495.95, 548.76],
   "mass_participation_xyz": [
    [0.28545, 3.0343e-08, 0.26617],
    [5.4405e-07, 0.10293, 2.5096e-07],
    [0.058475, 7.4234e-07, 0.007912],
    [0.015163, 1.7571e-08, 0.12291],
    [2.4632e-08, 0.57856, 1.2043e-07],
    [1.355e-08, 2.5536e-05, 8.7359e-11]
   ],
   "mass_participation_sum_xyz": [0.35909, 0.68152, 0.39699]
  }
 },
 "static": {
  "accel_m_s2_per_axis": 98.1,
  "combined_g": 17.32,
  "nose_post_underside_fixed": {
   "study": "Nose post static 10g",
   "mesh_max_mm": 2.0,
   "max_von_mises_mpa": 1.005,
   "max_displacement_mm": 0.0724
  },
  "dock_six_screws_pod_lump": {
   "study": "Dock static 10g pod lump",
   "mesh_max_mm": 2.0,
   "pod_mass_g": 162.0,
   "pod_connection": "distributed, to the two rear top faces of dock_cam",
   "max_von_mises_mpa": 27.1,
   "max_von_mises_location": "edge of the rear-left dock screw hole (fixed hole wall)",
   "field_stress_mpa_away_from_holes": "under about 6, read from the colour scale",
   "max_displacement_mm": 1.21,
   "max_displacement_location": "tail tab tip"
  },
  "pod_base_included": {
   "study": "Pod static 10g",
   "status": "not solved, pod_base failed to mesh"
  }
 }
}
```

## 9. State of the SOLIDWORKS file

- Studies in `avionics_pod_assembly_nose_camera.SLDASM`: "Nose post modal", "Nose post
  bolted", "Nose post static 10g", "Pod static 10g", "Dock static 10g pod lump",
  "Dock modal 6 screws pod lump".
- "Nose post bolted" lost its six-hole fixture selection when the assembly was reloaded
  mid-session. Its numbers in section 3 were read before that. Case C reproduces its modes
  at 173, 185, 414 and 549 Hz.
- Custom materials added to the SOLIDWORKS library (Custom Materials, Plastic):
  "PETG printed E1900", "Camera stand-in 15g", "Pi5 board stand-in 71g".
- The stress plot in "Dock static 10g pod lump" has its legend capped at 10 MPa.
