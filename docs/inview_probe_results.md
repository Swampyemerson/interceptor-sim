# In-view detection mechanisms — Probe 1 (ground background) + Probe 2 (phantom competition)

**Date:** 2026-07-21 · **Detector:** `drone_finetuned_quad_v2.onnx` (the deployed
markerless seeker) · **Scorer:** `scripts/seeker/box_scoring.py` at its unified
defaults (extent 0.52 m, sec²(θ) off-axis widening) — the SAME fixed gate the
2026-07-21 frame-top re-score used.

This closes out the headline `next_probe`: the **two un-eliminated in-view detection
mechanisms**, now testable because the fixed scorer removed the near-6th-mirage
scoring artifact. The frame-top sweep (P0.1) had already shown the detector
localizes the target at ~100 % at the frame-top edge AND banked, 8–22 m, against
clean SKY. What remained: **(1) ground-clutter background** (deployed detector read
0 % on the horizon line vs 76–100 % on sky at the same range in flight) and **(2)
phantom competition** (5–25 phantom boxes/frame in flight vs 0 static; top-1
selection may mask a real detection).

---

## TL;DR — verdicts

- **Probe 1 — ground-clutter background: REFUTED in sim.** With a stable elevated
  hover and the nose-on target teleported BELOW / AT / ABOVE the boresight (ground /
  horizon / sky background) at 6–20 m, recall is **~100 % against GROUND and HORIZON,
  matching or BEATING sky** — for both level and banked targets, all-boxes and the
  deployed top-1. The detector is **not** background-sensitive in the sim.
  *Caveat:* the sim "ground" is a near-uniform gray plane (0.8) + the target's cast
  shadow + the horizon seam, **not** real outdoor terrain clutter — so this bounds,
  but does not equal, the real outdoor-appearance gap (a real-data/R5–R6 question).

- **Probe 2 — phantom competition: CONFIRMED but MODEST, and NOT the ~0-recall
  cause.** In the 8–12 m band over real coded-dash/approach captures (phantom storm:
  median **17** boxes/frame), a target-hitting box EXISTS in **81 %** of in-view
  frames, the deployed `detect()` picks it in **70 %**, and the phantom/self-mask
  masks a real detection in **12 %** of in-view frames (14 % of the localizable
  ones). The target box is rank-1 in 74 % of localizable frames; a top-3 or a
  conf-vs-phantom margin gate recovers most of the loss. **The committed
  camera-forward mount removes the phantom at source**, so the probe's value is
  explaining the sim and bounding the residual risk (≤~12 % of in-view frames if
  mount clearance is imperfect).

- **The binding wall is POINTING, not either in-view mechanism.** In the 8–12 m
  band only **25 %** of ticks even have the target in the frame — **75 % is
  out-of-frame / FoV loss** under the ~35–40° nose-down dash pitch. With ground
  refuted and phantom-competition modest, the two-in-view-mechanism hypothesis is
  **largely closed**: the dominant lever is pointing (adaptive tilt #46 / ADR-0065),
  exactly the #18k reframe.

---

## Probe 1 — ground-background sweep (needs the sim)

### Method — why an elevated hover, not an SDF camera tilt or a set-pose hold
To silhouette a 6–20 m target against GROUND you need a **downward line of sight**
(the camera above the target). Three approaches were tried:

1. **SDF camera-sensor down-tilt** (`INTERCEPTOR_SENSOR_PITCH_RAD`, the #18k(2)
   tilt-aware gt chain): on a bench check the gt box sat ~100–165 px off the
   rendered target — but that test was CONFOUNDED (the interceptor was set_pose-held
   at altitude and oscillating, and the diagnostic read gt and image asynchronously),
   so it does NOT cleanly indict the env correction (which is validated for the
   +up-tilt / negative-pitch case). Rather than debug a down-tilt gt path under a
   moving camera, Probe 1 sidesteps it entirely. Rejected for THIS probe.
2. **set_pose-holding the disarmed airframe at altitude**: measured to **oscillate
   ±1.6–3.0 m** vertically — unusable for clean captures.
3. **PX4 HOLD hover (adopted).** The interceptor is flown to a real ~11 m hover
   (stable to cm, low velocity), the **canonical LEVEL camera** unchanged, and the
   **static** target is teleported through a range × background × azimuth × attitude
   grid via `set_pose`. Background is set by the target's camera-frame ELEVATION:
   `ground` = −22° (below → gray ground plane + shadow), `horizon` = 0°, `sky` =
   +15° (above → sky). A level camera at depression |φ| is **optically equivalent**,
   for the detector's appearance/background/top-down aspect, to a camera pitched down
   |φ| viewing a co-elevation target — differing only in frame POSITION, which the
   frame-top sweep already proved irrelevant. The gt is the exact per-frame-synced
   **native** m2_detect chain the validated frame-top 'centered' control uses, so the
   gt boxes land on the rendered target (verified visually — see
   `docs/images/probe1_background_examples.png`).

`gt_*` is scoring/label-only (the mover only teleports the static target, never
reads gt into guidance). GPU render was ON — it corrupts **fiducial** decode only,
and this is markerless NN with no tags, so it is fine. Static set-pose captures are
not timing-sensitive, so the ADR-0015 machine-load confound does not bind (a
Probe-2 CPU replay ran concurrently); rerun at idle only if render anomalies appear.

Scripts: `scripts/seeker/ground_background_sweep.py` (`--takeoff`/`--sweep`/
`--analyze`/`--plot`/`--self-test`) + `scripts/seeker/ground_background_sweep.sh`.

### Results — recall vs range × background (v2, conf 0.25)

Pooled over 6–20 m, on-axis + off-axis, ~290–307 in-view frames per cell:

| attitude | background | ALL-boxes recall | deployed top-1 recall |
|---|---|---|---|
| **level** | sky | 92 % (278/303) | 89 % (269/303) |
| **level** | horizon | **100 %** (294/295) | 96 % (282/295) |
| **level** | ground | **100 %** (286/287) | 96 % (276/287) |
| **banked** | sky | 92 % (281/307) | 86 % (265/307) |
| **banked** | horizon | **100 %** (304/304) | 96 % (292/304) |
| **banked** | ground | **99 %** (299/301) | 98 % (295/301) |

Ground and horizon score **100 %** all-boxes at every 2 m bin 6–20 m (level and
banked); sky sits slightly LOWER at 90–96 %. The deployed top-1 dips only at the
far edge (20–22 m: sky 77–82 %, ground/horizon 84–94 %) — that far-range dip is the
Probe-2 phantom-competition effect, not a background effect, and it is present in
ALL three backgrounds. Example frames (sky/horizon/ground at r=10 m, green =
detector box, red = gt): `docs/images/probe1_background_examples.png`.

Plot: `docs/images/probe1_ground_background_recall.png` (level + banked panels).
CSVs: `logs/ground_background_sweep_level.csv`, `logs/ground_background_sweep_banked.csv`.

### Verdict
Ground/horizon background does **not** impair the deployed detector in the sim —
recall is at or above the sky baseline across 6–20 m, level and banked. The
in-flight "0 % on the horizon line" was therefore **not** a gray-plane / shadow /
horizon-seam background effect; it was the frame-top POSITION (already cleared) plus
the phantom/pointing effects. The **real** outdoor terrain-clutter gap is a distinct,
real-data-only question (the sim's flat gray plane cannot stand in for it).

---

## Probe 2 — phantom-competition replay (offline, no sim)

### Method
Re-ran v2 over EXISTING in-flight captures (`quad_r2l_flight_capture`,
`v3_flight_eval`, `quad_canonical_r2l_capture` — real coded-dash/approach flights on
the stock mount, where the props ARE in frame), scoring **every** box vs gt with the
fixed gate. For each in-view frame we record: does any box hit (the recall
CEILING), does the naive top-1 (argmax) hit, does the **deployed** `detect()`
(self-mask: |bearing|≤30° and no L/R edge, then top score) hit, and the score-RANK
of the best target-hitting box among all boxes. Script:
`scripts/seeker/phantom_competition_replay.py` (`--self-test`, `--plot`).

### Results — 8–12 m band (N = 43 in-view frames, phantom storm median 17 boxes/frame)

| metric | rate |
|---|---|
| any box hits target (recall ceiling) | **81 %** (35/43) |
| naive top-1 (argmax) hits | 60 % (26/43) |
| **DEPLOYED `detect()` hits (shipped)** | **70 %** (30/43) |
| MASKED (localizable but deployed missed) | **5/43 = 12 %** (14 % of localizable) |

- **Target-hit rank:** rank-1 = 74 % of localizable frames; the rest lose top-1 to
  a phantom (rank 2: 9 %, rank 3: 6 %, rank ≥5: 11 %).
- **top-K recovery:** top-1 74 % → top-3 89 % → top-5 91 % of localizable frames.
- **Loss reason** (5 masked): 4 phantom-outscores, 1 self-mask-rejects-target.
- **Score margin** (phantom_top1 − best_target): median **+0.068**, **80 % < 0.10**
  → a conf-vs-phantom margin gate would flip most masked frames.

### Robustness — 6–16 m band (N = 181)
Same picture, more N: ceiling 56 %, deployed 46 %, masked **9 %** (17 % of
localizable), rank-1 = 74 %, top-3 recovers 85 %. Here the loss splits **59 %
self-mask-rejects-target / 41 % phantom-outscores** — at the frame edge the
anti-phantom self-mask (built to reject the prop) also rejects the off-boresight
real target (a real cost of the mitigation). Recall degrades with range
(14–16 m ceiling 39 %) as the target shrinks. Plot:
`docs/images/probe2_phantom_competition.png`; example frame (green = target box,
orange = phantom boxes, red = gt): `docs/images/probe2_phantom_storm_example.png`.

### The pointing decomposition (why deployed 70 %, yet in-flight recall ~0)
In the 8–12 m band only **43/171 = 25 %** of ticks have the target in view; **75 %
is out-of-frame** under the nose-down dash pitch. So the in-flight approach-recall
~0 (approach_recall.py 0.8 % ≥8 m) is dominated by POINTING (target not in frame),
NOT phantom competition — when the target IS in view the deployed detector hits it
70 %. Phantom competition is a real but secondary in-view loss (~12 %).

### Verdict
Phantom competition is **confirmed and quantified**: real target boxes exist but
lose top-1 to prop phantoms (or are rejected by the anti-phantom self-mask) in
~12 % of in-view 8–12 m frames (~17 % at 6–16 m). It is **not** the cause of the
in-flight ~0 recall (pointing is). The committed forward camera mount removes the
phantom at source → recovers this loss (deployed → toward the 81 % ceiling); the
**residual risk** if prop-clearance is imperfect is bounded at ≤~12 % of in-view
frames and is cheaply recoverable by a top-K or a conf-vs-phantom margin gate
(80 % of masked frames have a <0.10 margin; top-3 recovers ~89 %).

---

## What this closes (two-mechanism hypothesis)
Both candidate in-view mechanisms are now settled: **ground-background REFUTED**
(in sim), **phantom-competition CONFIRMED-but-modest** (≤~12 % of in-view frames,
mount-removable). Neither explains the in-flight ~0 recall — **pointing** does
(75 % out-of-frame in the 8–12 m band). This corroborates the detector stage note /
#18k: the dominant lever is **camera pointing / adaptive tilt (#46)**, and the
real-data detector's remaining scope is the **outdoor-appearance / terrain-clutter
gap**, which the sim's flat gray ground cannot test.

## Files
- Scripts: `scripts/seeker/ground_background_sweep.py` + `.sh`,
  `scripts/seeker/phantom_competition_replay.py` (each with a `--self-test` needing
  no sim).
- CSVs: `logs/ground_background_sweep_level.csv`,
  `logs/ground_background_sweep_banked.csv`,
  `logs/phantom_competition_replay.csv`, `logs/phantom_competition_replay_wide.csv`
  (+ JSON summaries under `logs/`, `logs/ground_background_sweep/`).
- Plots: `docs/images/probe1_ground_background_recall.png`,
  `docs/images/probe1_background_examples.png`,
  `docs/images/probe2_phantom_competition.png`,
  `docs/images/probe2_phantom_storm_example.png`.

## Honesty / anomalies
- The sim "ground" is a flat near-uniform gray plane + shadow, not outdoor terrain —
  Probe 1's ground-refutation is sim-scoped; the real terrain-clutter gap is untested
  and stays a real-data question.
- The SDF camera-sensor-tilt gt correction (`INTERCEPTOR_SENSOR_PITCH_RAD`) LOOKED
  ~100–165 px off on a DOWN-tilt bench check, but that check was confounded (moving
  set_pose-held camera + async gt/image read), so it is NOT a clean indictment of the
  env path (validated for +up-tilt). Probe 1 avoided the whole question with the
  native level-camera gt chain; a clean down-tilt gt validation, if ever needed, is
  left as separate work.
- Probe 2 N is modest in the exact 8–12 m band (43 in-view frames) because 75 % of
  8–12 m ticks are out-of-frame; the 6–16 m band (N = 181) corroborates the rates.
