# Acquisition range vs. sensor resolution & focal length — design note

*Builder question: "How much does a better camera (more pixels / longer lens) buy us
in acquisition range for the markerless seeker?" This note answers it with the pinhole
model, ties the answer to why our two prototypes only acquire terminally, and lays out
the trades so the lever is chosen with eyes open. Companion to
`docs/seeker_design_brief.md` (interface + eval plan) and
`docs/seeker_prototype_results.md` (the measured prototype outcome). No sim was booted
for this note; every number below is a derivation or a cite.*

> **One-line answer.** Acquisition range is set by **pixels-on-target**, and
> pixels-on-target scale as `fx · W / R`. So at a fixed detector floor, **acquisition
> range grows roughly linearly with linear resolution (or focal length)** — double the
> pixels-per-degree and you roughly double the range at which the target clears the
> detector's minimum-blob size. A longer focal length does the same **but trades away
> field of view**, and in this project narrowing the FOV already *backfired* on fast
> crossers (ADR-0024). The honest lever is more pixels at fixed FOV, capped by optics,
> motion blur, and low-light SNR — measured for real at the Stage-0 bench (ADR-0033).

New terms, one line each:
- **Pixels-on-target** — how many image pixels the target's body spans. A detector
  needs a minimum blob (roughly ~10 px across) before it can separate target from noise;
  below that floor the target is invisible to it. This is the whole game.
- **fx (focal length in pixels)** — the camera intrinsic that maps angles to pixels. Our
  `gz_x500_mono_cam` has **fx = 539.936**, image width **1280 px**, horizontal FOV
  **1.74 rad = 99.7°** (`configs/camera_intrinsics.json`; self-consistent:
  `2·atan(1280 / (2·539.936)) = 1.740 rad`, verified). That is **12.84 px/deg**.
- **IFOV (instantaneous field of view)** — the angle one pixel subtends, `1/fx` rad =
  **1.85e-3 rad = 0.106°/px** here. The camera cannot resolve anything finer than this.
- **W** — the target's true frontal width. We size to **W = 0.3 m** (a 2.5–3" class quad).

---

## 1. Why the seeker acquires *terminally* — the pinhole derivation

**Pinhole primer.** A camera projects a real-world frontal segment of width `W` at
distance `R` onto the image plane at a pixel width of

```
w_px = fx · W / R              (small-target pinhole projection)
```

This is exact for a frontal segment (the ±W/2 edges each project to fx·(W/2)/R, summing
to fx·W/R); it is *not* the same as `angular_size × px_per_deg`, which over-counts on a
wide lens because pixels-per-degree is not uniform across a 99.7° field. Use `fx·W/R`.

**Pixels-on-target for our camera** (W = 0.3 m, fx = 539.936, verified with the pinhole
model):

| Range R | `w_px = fx·W/R` | vs a ~10 px detector floor |
|--------:|----------------:|---|
| 5 m   | **32.4 px** | comfortably detectable |
| 10 m  | **16.2 px** | just above floor |
| 20 m  | **8.1 px**  | at/under floor |
| 30 m  | **5.4 px**  | below floor — a smudge |
| 50 m  | **3.2 px**  | sub-blob |
| 100 m | **1.6 px**  | a single flickering pixel |

**This table *is* the explanation for the prototype results.** Both markerless
prototypes (`docs/seeker_prototype_results.md`) only produced a genuine target detection
in a **~4-frame terminal window at ~1.5–3 m**, where the body spans 30–65+ px. That is
not a bug in the detectors — it is the pinhole geometry. A YOLOv8n-COCO trained on
resolved objects, or a blob detector needing ~10 px of contiguous body, simply has *no
signal* to lock at 20–30 m on this lens, where the target is 5–8 px. The independent
literature agrees: YOLOMG reports a drone at 100 m is **~10×10 px in 1080p** and 42% of
targets in the ARD100 set are **<12×12 px** (brief §2.1) — the same few-pixel wall.

**Contrast with the AprilTag baseline.** The fiducial is detectable at **9–12 m**
(ADR-0015 row 10 / ADR-0024/0028) because a decoder integrates a *known high-contrast
pattern* — it extracts pose from far fewer effective pixels than a generic body detector
needs. Removing the tag therefore forfeits acquisition range: this is the **R2
acquisition-range regression** the brief predicted and the prototypes confirmed
(terminal-only ~1.5–3 m vs the tag's 9–12 m). Recovering it is what this note is about.

**Why acquisition range is the lever that matters.** Pro-nav's terminal miss is ~96%
set by the zero-effort-miss (ZEM) at handoff (r² = 0.99, ADR-0023), and the correction
capacity the interceptor has left scales as **t_go²** (½·a·t_go²). Acquiring earlier
means more t_go, quadratically more capacity: ADR-0024/0027 put acquiring at 12 m vs
6.5 m as capacity 0.72 → ~4.3 m. So a few extra meters of acquisition range is a
first-order miss lever — far more valuable than terminal range accuracy. That is
precisely why "how far can the seeker see the body" is the right question to optimize.

---

## 2. The lever: linear resolution and focal length (roughly linear in range)

Set a fixed detector floor `w_floor` (the minimum pixels-on-target the detector needs;
~10 px is a reasonable working value for a small-object detector, and is the value the
Stage-0 bench must actually measure). The **acquisition range** is where the target
first clears that floor:

```
R_acq = fx · W / w_floor
```

Everything the seeker's *reach* depends on lives in this one equation. Two ways to grow
`R_acq`, both increasing `fx` (pixels per radian):

**(a) More linear resolution at fixed FOV.** Doubling the horizontal pixel count from
1280 → 2560 while holding HFOV = 99.7° doubles fx (539.9 → ~1080 px/rad). `R_acq` scales
**linearly**:

| Detector floor | `R_acq` at 1280 px | `R_acq` at 2560 px (2× res, same FOV) |
|---|---:|---:|
| 10 px | **16.2 m** | **32.4 m** |
| 5 px  | **32.4 m** | **64.8 m** |

(Both `R_acq` columns are `fx·W/w_floor`, verified.) So *at a fixed FOV, acquisition
range is roughly linear in linear resolution* — 2× the pixels ≈ 2× the range, until one
of the physical ceilings in §3 bites. Note this is **linear** resolution (pixels across
a dimension), not megapixels: 4× the megapixels is only 2× the linear resolution.

**(b) Longer focal length (narrower FOV).** A longer lens raises fx the same way, but by
*shrinking the angular field*, not adding pixels. Same `R_acq` gain — **but you give up
field of view**, and that is not free here.

---

## 3. The trades — why "just narrow the FOV / add pixels" isn't a free lunch

### 3.1 FOV is a hard, already-tested trade (the backfire)
This project split the problem into an **acquisition** phase and a **terminal** phase
exactly because a wide search field and a long-reach lens pull in opposite directions
(the two-stage handoff architecture, ADR-0013). When we actually built the narrow lens
and flew it, it **backfired**: FOV 99.7° → 60° (fx 539.9 → ~1108.5 px) did double
detection range in the lab (~6 → ~12 m), but in Gazebo the narrower field let a **fast
crosser walk out of frame**, *hurting* the 9 m/s latch (0/12) — so 60° was **rejected**,
not merged (ADR-0024 2nd & 3rd addenda). The lesson: for a **crossing** target the FOV
must be wide enough to *keep* the target in view through the engagement; buying reach by
narrowing the field can lose the target you already had. **Longer focal length is only
safe once a wide acquisition stage hands off a track** — which is the whole point of the
two-stage design. For the onboard terminal seeker, the honest lever is therefore **more
pixels at a fixed (wide-enough) FOV**, i.e. lever (a), not (b).

### 3.2 Compute — solved, don't let it veto resolution
Running a detector over a 2560²-class frame every tick is expensive on a Pi-class board.
This is **already addressed** by the two-stage detector-crop approach (build-queue
task #10): detect/track on a small crop around the predicted target, not the full frame,
so inference cost tracks the crop size, not the sensor size. That decouples "more sensor
pixels for reach" from "more pixels to process per frame" — the reason resolution is a
usable lever at all on embedded hardware.

### 3.3 Sensor cost & bandwidth
More pixels means a costlier sensor, more data off the sensor per frame, and higher
readout bandwidth/power — and, at fixed sensor die size, **smaller pixels** (see 3.5).
On the deployment target (Pi-5 + Hailo-8, ADR-0012/0015) the interface and memory
bandwidth are real constraints; a global-shutter high-res sensor is heavier and pricier
than the current module. This is a budget line, not a physics wall.

### 3.4 Diffraction / optics MTF ceiling
Adding pixels only helps if the *lens* can deliver detail that fine. Diffraction blurs a
point to an Airy disk of angular size `θ ≈ 1.22·λ/D` (λ ≈ 550 nm, D = aperture). Setting
`θ = 1 IFOV = 1/fx` gives the aperture below which optics, not pixels, limit resolution:

```
D_limit = 1.22 · λ · fx = 1.22 · 550e-9 · 539.936 ≈ 0.36 mm  (current res)
```

At today's fx we have headroom: a modest **2 mm** aperture gives a diffraction spot of
**0.18 px** (verified), so the sensor — not the lens — is the limit, and adding pixels
helps. But at **2× resolution** the break-even aperture rises to **~0.72 mm**, and if the
optic is stopped down small (common on tiny fixed-focus Pi cameras) diffraction can erase
the extra pixels. **Rule: adding pixels only buys range while the aperture stays above
`1.22·λ·fx`; past that the MTF (modulation transfer function — how much contrast the lens
preserves at a given detail scale) is the ceiling, not pixel count.**

### 3.5 Motion blur (why a global shutter matters)
A few-pixel target smears if it moves across the sensor during the exposure. Blur in
pixels ≈ `λ̇ · t_exposure · fx`. Near closest approach the LOS rate λ̇ hits **485–1870°/s**
(ADR-0023 row 7). Worked cases (verified):

| exposure | λ̇ = 50°/s (mid-course) | λ̇ = 485°/s (near CPA) |
|---|---:|---:|
| 1 ms (fast global shutter) | 0.47 px | 4.6 px |
| 5 ms | 2.4 px | **22.9 px** |

A 5 ms exposure at terminal LOS rate smears the target across ~23 px — it **destroys**
a target that is only a few pixels wide to begin with, and a **rolling** shutter adds
geometric skew on top. So the resolution lever demands a **short exposure + global
shutter**, which in turn costs light (3.6). This is also why the brief flags the sim as
optimistic: it has **no motion-blur model** (ADR-0023), so sim acquisition range is an
**upper bound** on a real seeker's (ADR-0024) — the bench is the reality check.

### 3.6 Low-light SNR of smaller pixels
At a fixed sensor size, more pixels = **smaller pixels** = fewer photons each = lower
signal-to-noise, especially at the short exposures 3.5 demands and in low light. A
few-pixel target that is also photon-starved may not clear the detector's contrast
threshold even if geometrically it spans enough pixels. Smaller pixels can *give back* the
resolution they promised. This is a genuine physical tension — resolution vs. sensitivity
— with no free win; the right operating point is sensor-specific and, again, **measured**,
not assumed.

---

## 4. Tie-off: recovering the R2 regression, and where it gets measured for real

The markerless seeker's central cost is the **R2 acquisition-range regression**: it sees
the body terminally (~1.5–3 m measured) where the tag saw the marker at 9–12 m, and
because correction capacity ∝ t_go² that regression is the dominant threat to the miss
(ADR-0023/0024). This note shows the **primary knob to claw it back is pixels-on-target
via linear resolution at a fixed (wide-enough) FOV** — roughly linear range gain — while
respecting that (i) narrowing FOV to get reach *backfires* on crossers (§3.1, ADR-0024),
and (ii) optics, motion blur, and SNR cap how far pixels alone carry you (§3.4–3.6).

Crucially, **this note only ranks the lever; it does not conclude.** The detector floor
`w_floor`, the true `P_detect(R)` vs. range curve, and how it shifts with resolution/lens
are exactly what the **Stage-0 bench (ADR-0033 item 1)** exists to measure — it captures
detection Hz, max trackable λ̇, bearing σ, dropout burst length, and latency on the real
sensor/board, which is where "double the pixels ≈ double the range" gets confirmed or
corrected against real clutter, blur, and SNR. Per project doctrine ("lab ranks, Gazebo
decides"; and here, *the bench decides*), no acquisition-range claim ships as a number
until that measurement exists. The tag-less Gazebo model flown with ground-truth logging
(brief §4–5) gives the sim-side `P_detect(R)`; the Stage-0 bench gives the hardware truth
and the sim-to-real gap.

---

### Sources
Repo/ADR: `docs/seeker_design_brief.md` (§2.1, §3, §5); `docs/seeker_prototype_results.md`
(terminal-only acquisition, both lanes); `docs/decisions.md` ADR-0012/0013/0015/0023/0024
(+2nd & 3rd addenda)/0027/0028/0033; `configs/camera_intrinsics.json` / `models/mono_cam`
(fx = 539.936, HFOV 1.74 rad); `.claude/skills/pronav` (a_cmd = N·Vc·λ̇, ZEM). Two-stage
detector-crop: build-queue task #10.

Derived here (pinhole `w_px = fx·W/R`, `R_acq = fx·W/w_floor`, diffraction
`D_limit = 1.22·λ·fx`, motion blur `λ̇·t·fx`), all recomputed and checked:
pixels-on-target 32.4/16.2/8.1/5.4/3.2/1.6 px at 5/10/20/30/50/100 m;
`R_acq` 16.2 → 32.4 m (10 px floor) and 32.4 → 64.8 m (5 px floor) at 1× → 2× linear
resolution; IFOV 0.106°/px; diffraction break-even D ≈ 0.36 mm (0.72 mm at 2× res),
2 mm aperture → 0.18 px spot; motion blur 0.47–22.9 px across the exposure/λ̇ grid.

Web (via the brief, accessed 2026-07): YOLOMG (arXiv 2503.07115) — ~10×10 px @100 m in
1080p, 42% of ARD100 targets <12×12 px.
