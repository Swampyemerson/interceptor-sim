# M12 lens options for the OV9281 — DECIDED

> **RULED (builder, 2026-10-22, ADR-0126): the delivered ~70° lens FLIES**, for px/deg
> range; the ≥100° wide-FOV constraint is superseded (rewritten in the contract,
> graveyard #17 scope-amended). The 110° Arducam M40180H13L already ordered 2026-10-12
> (this survey missed it — it was in the contract's build tab) becomes the calibrated
> SPARE/fallback if the 70° acquisition funnel proves too tight in the field. Derived
> requirement from the same-day isim A/B: launch cue inside ~10° (ADR-0128 addendum).

2026-10-22, head-inline survey (the subagent lane bounced; evidence log).
Context: the delivered lens measured ~70° HFOV against a 118° listing — the
wide-fov HARD constraint (≥100°) is violated (2026-10-04 bench finding).
The resolution×guidance study (runs/res_guidance_20261021) reframed the
choice: the real variable is PIXELS-PER-DEGREE × FOV together, and the NN
detector's input size couples to it (640-wide input is dead at 118°, fine
at 70°).

## The physics table (OV9281: 1280×800 @3 µm → 3.84×2.40 mm active)

Pinhole math (real wide lenses gain a few deg from barrel distortion — the
delivered "~3.3 mm-equivalent" lens solves at 69.4° vs 60° naive):

| f (mm) | fx (px, native 1280) | HFOV | px on 0.3 m target @20 m (native / @640 input) |
|---|---|---|---|
| 1.4 | 467 | ~108° | 7.0 / 3.5 |
| 1.56 | 518 | ~100° | 7.8 / 3.9 |
| 1.8 | 600 | ~94° | 9.0 / 4.5 |
| 2.1 | 700 | ~85° (listed 80°H on 1/4") | 10.5 / 5.2 |
| 2.8 | 933 | ~69° | 14.0 / 7.0 |
| delivered (~3.3) | 1104 (measured) | 69.4° (measured) | 16.6 / 8.3 |

Floor for a usable detection: ~8 px (both the tag decoder and the nano-NN
small-object regime agree on the order).

## What the table says

1. **The ≥100° constraint forces f ≤ ~1.56 mm on this tiny sensor** → ~8 px
   at 20 m at NATIVE 1280 — no margin, and a 640 input is blind past ~10 m.
   A ≥100° lens is only compatible with the accelerator + native-res path
   (exactly the resolution study's conclusion), and low-distortion ≤1.6 mm
   M12 glass for 1/4" is fisheye-class or industrial (Commonlands class,
   quote-priced); expect undistort cost + edge-recall loss.
2. **A middle lens exists and is cheap:** Arducam M40210M09S, 2.1 mm, 1/4",
   <1% distortion, listed 80°H on 1/4", **$11.99**
   ([product](https://www.arducam.com/product/arducam-1-4-m12-mount-2-1mm-focal-length-distortion-camera-lens-m40210m09s/)).
   80° vs the delivered 70° buys +15% FOV (aim tolerance ±40°-ish at the
   frame edge vs ±30 asymmetric today) for a 37% px/deg cost (16.6→10.5 px
   @20 m native). With the Hailo at native input, 10.5 px @20 m still
   clears the floor; at 640 it does not.
3. **Keeping the delivered 70° lens is defensible** now that the pointing
   stage (not FOV) is the validated bottleneck-breaker: it has the best
   px/deg, it is calibrated, and the aim-tolerance shortfall is a CUE
   quality requirement, not necessarily a lens requirement.

## Recommendation (two scenarios, builder's call)

- **(a) Pointing/cue accuracy holds (±15-20° aim):** KEEP the delivered 70°
  lens. Zero cost, best detection range, already calibrated. Re-state the
  wide-fov constraint as superseded by the measured aim tolerance rather
  than leaving it silently violated.
- **(b) The ±30° aim tolerance must stand:** buy the $12 Arducam 2.1 mm
  (80°H) as the compromise AND hold the ≥100° ambition until the Hailo +
  native-1280 pipeline is proven (a ≥100° lens without it is blind at
  range). Recalibration is minutes via the screen-checkerboard flow.
- Either way: NOT a purchase blocker for the Hailo decision; the lens can
  change after the detector pipeline exists (M12 swap + recal is cheap).

Sources: [Arducam M40210M09S](https://www.arducam.com/product/arducam-1-4-m12-mount-2-1mm-focal-length-distortion-camera-lens-m40210m09s/),
[Arducam M12 catalog](https://www.arducam.com/blog/product-category/camera-component/lenses/m12-lens-arducam/),
[Arducam M12 kit (1/2.8" angles, not 1/4")](https://core-electronics.com.au/arducam-low-distortion-m12-mount-camera-lens-kit-for-arduino-and-raspberry-pi-camera.html.plain.md),
[Commonlands M12 low-distortion line](https://commonlands.com/products/low-distortion-2mm-m7-lens-cil820),
[Kurokesu M12 lenses](https://shop.kurokesu.com/Lenses).
