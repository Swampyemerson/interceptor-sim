# Tripod field-test protocol — build_plan P2 (decision-grade)

> **Read this before you leave the house.** This is the ONE in-person session that
> produces the data BOTH money gates hang on (`docs/project_state.json` `build_plan`
> P2 + the `⛔ MONEY GATE` in `docs/hardware_order_list.md` §0d). One capture set,
> scored two ways. If a step here is skipped, the afternoon's data can't decide
> anything and you burn a field day for nothing — follow it in order.
>
> Sources: `docs/project_state.json` (`build_plan.P2`, `bom_tiers[0]`, stages
> `detector`/`pointing`/`handoff`), `docs/real_data_pipeline.md`,
> `docs/hardware_order_list.md` §0c/§0d, `NEXT.md` ROADMAP R5, ADR-0076 add
> #18h/#18i/#18j-fix/#18k.

---

## 0. What this session decides — READ FIRST

**ONE flight set (target quad + AprilTag placard, camera on a fixed tripod)
scores TWO curves, which gate TWO different purchases:**

| Curve | What it measures | Gates | Does NOT gate |
|---|---|---|---|
| **(a) AprilTag decode envelope** | Range/rate the tag `tag36h11` decodes at, on the deployed camera+lens | The **~$740 Tier-2 interceptor order** (first real kills fly the tag on Pi 5 CPU) | — |
| **(b) NN approach recall vs range × position-in-frame** | The markerless detector's real-target recall on the SAME footage — scored with the **real-data `n-mono`** model (primary) AND the sim-trained `drone_finetuned_quad_v2` (historical bar); see §7.2 | ONLY the **deferred $70 Hailo AI HAT+ / markerless phase** | The interceptor order — never |

A weak curve (b) delays markerless, nothing else. A weak curve (a) is the one that
stops you spending $740 — go back to a bigger placard or a camera upgrade instead.

**Compute bench (Pi 5 fps) rides along the same afternoon** but is a separate
bench task (§7.3) — it does not need the target flying.

---

## 1. Pre-session desk checklist (P0 — must be GREEN before you drive out)

- [ ] **Frame-top off-axis sim sweep run** (`build_plan.P0` task 1) — confirms the
      fixed-tilt mechanism is real before you rely on it for the camera setup below.
      If not yet run, use the sim-anchored default tilt in §3.2 and treat the field
      tilt as provisional.
- [x] **Auto-labeler validated in the `apriltag` sim world** (tag-box vs gt-box IoU
      vs range) — **DONE 2026-07-21** (IoU 0.965 vs gt, min 0.951; `docs/placard_sizing.md`).
      *(The "NOT yet run / use the ~0.3 m BOM default" text here was STALE and is
      superseded — it would have sent you to the field with an undersized placard.)*
      **ADOPTED PLACARD: 0.35 m AprilTag black square = a 0.45 m PHYSICAL SHEET.**
      Note the two numbers: 0.35 m is the *tag* (what `tag_size` means for pose), the
      *object you carry and print* is **450 × 450 mm** (black square + the mandatory
      1-cell quiet zone: 0.35/0.8 = 437.5 mm + margin — `docs/placard_mount.md` §2).
      Every other doc that says "~0.3–0.35 m print" is naming the tag, not the sheet.
- [ ] **Camera paper-checks done** (before/at purchase, `build_plan.P0` task 5):
      HFOV ≥100°; **vertical FoV** — OV9281 is 1280×800 vs the sim's 1280×960, and
      the wall is a *vertical* pointing problem, so check this explicitly, not just
      HFOV; exposure control ≤1 ms; mono-vs-color-trained-NN confound noted (v2 was
      trained on sim color/greyscale renders — mono real frames are a domain shift,
      log it, don't fix it today).
- [ ] **$0 kill-test run**: candidate NN already tried on public outdoor drone
      footage — a free first read on the transfer bet, done before spending a field
      day (`build_plan.P0` task 6).
- [ ] **FAA registration + Remote ID / FRIA field settled** — see §9. Do this
      BEFORE the field day; it's a legal blocker, not a data blocker.
- [x] **Capture tooling exists**: `scripts/seeker/pi_capture.py` — BUILT. Records a
      session as `SESSION_DIR/frames/*.png` + `index.csv` (per-frame path, monotonic
      timestamp, exposure/gain actually applied) + `meta.json` + optional `tags.csv`
      (live tag36h11 decode). Backends: `--source picamera2` (real Pi OV9281, requests
      the ≤1 ms exposure of §3.3 and logs the ACTUAL applied value into `meta.json`),
      `--source v4l2` (UVC fallback), `--source dir=PATH` (replay, for dev). The
      layout is exactly what `autolabel_from_apriltag.py` (§7.1) and the two-curve
      tripod scorer (§7.2) consume — point them at `SESSION_DIR/frames`. Bench it
      before the field day with `pi_capture.py --self-test` (offline, no hardware,
      exits 0/1) and a real indoor `--source picamera2` static-scene pass to confirm
      the sensor actually hits ≤1 ms (check `meta.json` `exposure_meets_spec`).
- [ ] **Field power for the Pi 5 solved**: a USB-C PD source (≥27 W) that isn't the
      flight LiPo — a power bank or a car inverter. Not in the Tier-1 BOM; bring one.

---

## 2. What to bring

**Hardware (Tier 1, `bom_tiers[0]` in `project_state.json`):** target quad (E) +
AprilTag `tag36h11` placard (E) + Arducam OV9281 camera (C) + Raspberry Pi 5 8GB
(C) + checkerboard calibration board (C) + USB/power bits (D). Plus the ground/safety
set to actually fly the target (F): TX + charger + LiPo-safety/solder kit — this is
the ~$620–780 realistic first-outdoor-test outlay, not just the ~$310 tripod core.

**Also bring:**
- Tripod or mast for the camera (stable, adjustable height + a way to lock a tilt
  angle — an articulating ball head + a phone inclinometer app is enough).
- USB-C PD power bank for the Pi (§1 gap item).
- Laptop (offline scoring happens after the session, not required live, but useful
  for spot-checking captured frames between passes so a bad calibration/exposure
  isn't discovered after everyone's gone home).
- Tape measure or laser rangefinder, and cones/flags/GPS waypoint markers to mark
  the range stations in §4.
- A phone for slow-mo/overview video of each pass (context record, not the primary
  science data — see §6).
- Full safety kit — see §10; do not skip items to save a trip to the car.

---

## 3. Camera setup — the FINAL mount geometry, as best as it can be known pre-airframe

> **Honesty note:** the real interceptor airframe does not exist yet at P2 (Tier 2
> hasn't been ordered) — there is no measured "final" dash pitch yet (that comes
> from the first real dash ULog at `build_plan.P3`). So "final mount geometry" here
> means the **best pre-registered estimate**, captured explicitly as provisional so
> nobody later mistakes it for the measured number.

### 3.1 Position

- Camera **forward-facing**, mounted at approximately the target's flight altitude
  during passes (co-altitude with the target, matching the terminal engagement
  geometry the fixed tilt is sized for). Coordinate with the pilot to fly level
  passes at a fixed, briefed AGL so the tripod and the target line up.
- No prop-clearance geometry applies here (no airframe/props yet) — that HARD gate
  is a P4 bench item once Tier 2 is built. Don't spend field-day time on it.

### 3.2 Tilt

- **Provisional fixed up-tilt: ~30–35°** — the sim default (`--adaptive-tilt-max-deg`
  default 35.0 in `scripts/m4_intercept.py`; ADR-0076 add #18j-fix measured the sim
  dash pitch swinging ~40° down (accelerating) to ~34° up (braking); real-airframe
  range expected 25–40° per `hardware_order_list.md` §0d). **This is NOT the
  measured real dash pitch — it is a placeholder until `build_plan.P3` measures it
  from the first real dash ULog.** Lock the printed bracket only after that
  measurement (§0d: "print an ADJUSTABLE 10–30° bracket... lock it from the first
  dash ULog").
- Capture **one bracket pass** at **0° (level)** per aspect as a same-day, cheap
  empirical cross-check against the sim's "fixed tilt just relocates the in-view
  window" finding (ADR-0076 add #18j-fix) — do not over-invest here; the real
  tilt-value decision is a sim-side sweep (`build_plan.P0` task 1/8), not this field
  day's job.
- Record the exact tilt angle used (inclinometer reading) in the session log (§11)
  — every downstream range/position-in-frame number is meaningless without it.

### 3.3 Orientation / lens

- Fit **only** the wide element (~2.5–2.8 mm, ~100° HFOV) — never the narrow/long
  spares (`hardware_order_list.md` §0①, hard requirement, not a tunable).
- Confirm exposure control reaches ≤1 ms before the full-speed passes (§4.4) — this
  is what the motion-blur read depends on.

---

## 4. Field layout & capture matrix

### 4.1 Range stations

> ⛔ **CORRECTED 2026-07-24 — the original stations (8/12/16/20/25/30 m) COULD NOT
> MEASURE THE THING THIS DAY EXISTS TO MEASURE, and would have manufactured a FALSE
> NO-GO on the ~$740 interceptor order.** The predicted `R_decode90` for the adopted
> 0.35 m placard is **7.10 m** (realistic px/deg scaling; 5.98 m conservative, 8.97 m
> full-res — `docs/placard_sizing.md` §5). The old NEAREST station was **8 m**, i.e.
> *outside* the entire predicted envelope: every station would have returned ~0%
> decode, curve (a) would read "no range bin sustains ≥90%", and `gate_verdict` would
> have returned **FAIL → do not buy the interceptor** — for a purely geometric reason,
> not a real perception limit. Stations must BRACKET the predicted envelope, not sit
> beyond it. (Found by the placard-mount design pass, `docs/placard_mount.md` §11.)

Mark **4, 6, 8, 10, 12, 16, 20 m** from the tripod along the approach line (use a
rangefinder or pre-plan the target's AUTO waypoint mission in QGC so the ranges are
GPS-repeatable pass to pass — the ArduPilot target flies a scripted box/line, so
this is buildable once, then reused for every pass). The **4/6/10 m near stations are
the money stations** — they are where the tag is predicted to actually decode. Keep
16/20 m as the upper bracket (they establish where the envelope dies, which is a real
number worth having); 25/30 m are dropped as certain zeros that only cost field time.

**Bin curve (a) in 2 m bins from 4–14 m** (not the old wide bins) — the decision
hinges on where inside 4–10 m the 90% line falls, so the resolution has to be there.

### 4.2 Aspects

- **Approach** — target flies straight at the camera boresight (mirrors the sim's
  head-on regime).
- **Crossing** — target flies a lateral leg at a fixed standoff that transects the
  FOV (mirrors the sim's l2r/r2l crossing regime — the regime where the AprilTag goes
  *invisible* in sim, ADR-0076 add #18e; this is exactly the aspect worth checking for
  real). **Use a ~5–6 m standoff for the dedicated tag-envelope crossing block**
  (inside the predicted decode envelope); the old ~15–20 m standoff is retained ONLY
  for the NN/curve-(b) crossing passes, where detection is not envelope-limited the
  same way.

### 4.2b Incidence is a variable, not a nuisance — record and score it

The placard is a **flat fiducial**: decode range falls with the angle between the
camera boresight and the placard normal (the *incidence angle*). The measured law is
`R_decode(θ) ≈ R_decode(0°)·cos θ`, with a usable cone of **±32.3°** at the deployed
`quad_decimate=2.0` (`docs/placard_mount.md` §6). A pass flown at 30° incidence and a
pass flown at 0° are **different experiments** — pooling them smears the curve.

- **Record the mount index angle** (the placard mount is 15°-indexable) on every pass
  card, and **compute per-frame incidence offline** from the target ULog attitude +
  the surveyed tripod position + that index angle. This costs **zero field time**.
- **Score curve (a) as decode vs `(range × incidence)`**, not range alone.
- **Re-decode the same captured frames offline at `quad_decimate ∈ {2.0, 1.0}`.**
  `qd=1.0` is the unexercised range/incidence-widening lever (~1.5× range, ±48–56°
  cone) and costs only Pi fps — so if the gate is marginal at `qd=2.0`, this is the
  first reclaim lever and it needs no second field day.

### 4.3 Backgrounds

Reposition the tripod/flight line (or fly at different times of day) to get, as
field time allows:
- **Sky** — target against open sky, high contrast.
- **Horizon** — target crossing the horizon line.
- **Ground clutter** — target low against trees/buildings/terrain (the harder
  case; best-effort if the field doesn't offer it).

### 4.4 Speeds

- **Full-speed passes are the ONE thing this field day can measure that the sim
  cannot** — Gazebo has no motion-blur model. Fly the target at its real cruise/dash
  speed, pushing toward the eventual **≥9 m/s** engagement regime as pilot skill and
  field length allow. This is the primary read on the OV9281's ≤1 ms exposure claim
  (§3.3) and on whether real motion blur eats the tag's decode range or the NN's
  recall.
- **One slow-speed control pass per cell** (~2–4 m/s) — the regime the sim
  originally validated (M4 gate); a same-day sanity baseline to compare full-speed
  degradation against.

### 4.5 Attitude

Add a handful of **banked-turn passes** (diving/banking crossing legs, not level) —
ADR-0076 add #18k names banked attitude as an untested candidate factor in the
flight-dynamic recall wall alongside frame position. A few banked passes at the
tripod, even without a full sweep, is the cheapest real check of that hypothesis.

### 4.6 Capture matrix (target pass counts)

| Aspect | Background | Speed | Passes | Notes |
|---|---|---|---|---|
| Approach | Sky | Full-speed | 4 | primary decode-envelope + recall data |
| Approach | Sky | Slow (control) | 2 | sim-regime baseline |
| Approach | Horizon | Full-speed | 3 | best-effort ordering — do first if field allows |
| Approach | Ground clutter | Full-speed | 3 | best-effort |
| Approach | Sky | Full-speed, 0° tilt | 2 | tilt cross-check (§3.2) |
| Approach | Sky | Full-speed, banked | 2 | attitude factor (§4.5) |
| Crossing | Sky | Full-speed | 4 | AprilTag-invisible-in-sim check |
| Crossing | Horizon | Full-speed | 3 | best-effort |
| Crossing | Ground clutter | Full-speed | 3 | best-effort |
| Crossing | Sky | Full-speed, banked | 2 | attitude factor |

**~28 passes total** (sky-background cells are the must-get; horizon/clutter/tilt/
banked cells are ordered by priority — do sky first, drop the "best-effort" rows
if batteries/daylight run out). With 3× 6S 1500 mAh packs and the HOTA D6 Pro
charger in the BOM, plan on mid-session recharge cycles rather than trying to fly
all packs back-to-back.

This is intentionally **not** n≥8 paired-seed statistics (CLAUDE.md's sim standard)
— a field afternoon can't buy that. Treat curve (a)/(b) as a first honest read, not
a statistically tight one; note the small-n caveat explicitly when reporting results.

---

## 5. Calibration checklist — do this FIRST, before any flight

1. Print/mount the checkerboard (9×6 inner corners) rigidly and flat.
2. Capture **≥15 checkerboard images** at varied angles/distances/positions in
   frame with the OV9281 in its final tripod mount (position + tilt from §3).
3. Run:
   ```
   scripts/calibrate_camera.py --images "IMAGES_GLOB" --cols 9 --rows 6 \
       --square SQUARE_SIZE_M --out calib.json
   ```
   (or `--live DEVICE` to capture interactively from the Pi camera directly).
4. **Acceptance bar: RMS reprojection error ≤ 1.0 px.** The script prints a
   `WARNING` and recommends more views above that — do not proceed to flights on a
   marginal calibration; every downstream range/bearing number is wrong until this
   passes (`hardware_order_list.md` §0③, first bench gate).
5. `calib.json` is the `--calib` input to `autolabel_from_apriltag.py` (§7.1) —
   keep it with the session's data, not just on the laptop that ran it.
6. **Re-run a short checkerboard set at the END of the session** (5–8 images is
   enough) and diff the fx/fy/cx/cy against the morning calibration — thermal drift
   or a bumped mount between passes should show up here, not silently corrupt the
   range numbers.

---

## 6. What to record — every pass

- [ ] **Raw camera frames** — PNG/raw, not compressed video, for the frames that
      feed curve (a)/(b) scoring. Compression artifacts specifically hurt small,
      distant-tag decode — the exact regime curve (a) is trying to measure. Use
      video only for the secondary context record below.
- [ ] **Synced tag-pose per frame** — run `autolabel_from_apriltag.py` (or a bare
      `pupil_apriltags` decode loop) over the captured frames after the fact; do
      NOT rely on live onboard tag decode during capture — decouples the capture
      script from the detector.
- [ ] **The calibration file** (`calib.json` from §5) used for that session.
- [ ] **The target's ULog** — GPS/attitude/timestamp record. This is the
      independent **range ground truth beyond the tag's decode ceiling**: the tag
      only decodes to whatever range curve (a) turns out to be, but the drone is
      visible in frame from much farther out, and the NN-recall curve (b) needs a
      range value out there too. A surveyed/GPS-logged tripod position + the
      target's GPS track gives that. *(No interceptor ULog exists yet at P2 — only
      one aircraft flies this session; "both ULogs" becomes relevant starting at
      `build_plan.P5`.)*
- [ ] **Overview video** (phone slow-mo or similar) per pass — a context record for
      later sanity-checking aspect/background/lighting, not the primary science
      data.
- [ ] **Session log entries** (§11) — pass number, aspect, background, speed,
      tilt angle, battery, any anomaly (RC glitch, sun glare, wrong heading).

### 6.1 Time sync

Frame timestamps (Pi clock) and the target's ULog (autopilot GPS time) must agree
well within one range bin's time-of-flight (~2 m of travel at 9 m/s ≈ 0.22 s per
bin — keep sync error well under that):
- NTP-sync the Pi's clock (phone hotspot) within a few minutes of the first pass.
- Mark one clear **sync event per pass** visible in both streams — e.g., the pilot
  flies the target directly over the tripod at a known moment, or flashes a light
  toward the camera at the pass start — and note the frame index / ULog timestamp
  of that event in the session log.

---

## 7. Offline scoring plan (after the field day)

### 7.1 Curve (a) — AprilTag decode envelope

1. Run `autolabel_from_apriltag.py --frames DIR --calib calib.json --tag-size
   TAG_EDGE_M --drone-size 0.35 --out DATASET` per pass directory. It reports the
   tag's label/decode rate — that print IS the raw curve-(a) input.
2. Bin decode success (tag found vs not) by the frame's ground-truth range (from
   the ULog track, §6) into the same range bins as §4.1 (or finer, e.g. 2 m bins
   like `approach_recall.py`'s convention).
3. Report **two numbers per pass-set**: `R_decode_any` (farthest range with ANY
   successful decode) and `R_decode90` (farthest range where the decode rate
   *sustains* ≥90% inward) — the streak needs a sustained rate, not a lucky single
   frame, to actually form a handoff.

### 7.2 Curve (b) — NN approach recall vs range × position-in-frame

The dedicated harness (`build_plan.P0` task 3) is **BUILT**:
`scripts/seeker/tripod_score.py` (it reuses `approach_recall.py`'s range-binning and
`resolution_probe.py`'s box-hit test, tag-truthed instead of gt-truthed). Point it at
the session dir; it scores curve (a) and curve (b) in one pass.

1. **Score curve (b) with TWO models on the same frames** — this is the default and
   you should not override it:
   - **PRIMARY (the candidate): `scripts/seeker/weights/nn_tier/n-mono.onnx` @640,
     conf 0.25** — YOLO11n, COCO-init, **grayscale-native** (matches the mono OV9281).
     On source-disjoint REAL held-out imagery it scores **AP50 0.442 / recall 44.2% /
     precision 71.4% / false-fire 4.9%** (`logs/nn_tier/eval_n-mono_heldout.csv`,
     n=4175). **This is the number curve (b) is about.**
   - **BAR (historical): `drone_finetuned_quad_v2.onnx`** — the sim-trained model.
     ⚠️ It is **measured BLIND on real imagery**: AP50 **0.0003**, recall **1.1%**,
     false-firing on **88.5%** of drone-free frames. Scoring curve (b) with quad_v2
     *alone* (as this protocol originally said, before the 2026-07-21 nn_tier result)
     would produce a near-zero curve that reads like a markerless failure when it is
     really the already-known sim→real NULL. Keep it only as the contrast row.

   ```
   .venv-seeker/bin/python scripts/seeker/tripod_score.py SESSION_DIR \
       --calib calib.json --out-dir logs/tripod_score
   # -> curve_b_recall.csv (n-mono, PRIMARY) + curve_b_recall_drone_finetuned_quad_v2.csv (BAR)
   #    plus the primary-minus-bar delta in verdict.txt and gate.json:curve_b_models
   # (--no-weights-bar scores the primary only; --weights / --weights-bar override either)
   ```
   Input modality is resolved **per model** (gray for the gray-native nn_tier weights,
   color for the sim weights) — on the mono OV9281 the gray step is a bit-exact no-op,
   so no flag is needed in the field.
   **Read the PRIMARY row against §8.2's threshold; a low BAR row is expected and gates
   nothing.**
2. Ground-truth each frame's range/position with the tag pose where the tag
   decoded, and with the ULog-derived range beyond that (§6) — this is the one
   place curve (a)'s decode ceiling matters for scoring curve (b): don't drop
   frames beyond the tag's decode range, that's exactly the far-band data curve (b)
   needs (`real_data_pipeline.md`'s "tag-miss frames dropped" rule is about
   TRAINING labels, not this scoring pass).
3. Bin recall by **range** (as approach_recall.py does) **AND by position-in-frame**
   (e.g. top/middle/bottom third, or degrees off the vertical boresight) — ADR-0076
   add #18k's finding was that recall vs range alone hid a 100%-static-vs-0.8%-
   in-flight gap that only position-in-frame explained. Do not report range-only
   numbers as the final read.

### 7.3 Compute bench (Pi 5 fps) — separate, no flight needed

Bench the real Pi 5 the same day (`build_plan.P2` task 5): sustained AprilTag fps
and CPU-YOLO fps on the actual hardware (`pi5-emulation-gap` constraint — emulation
cannot measure this). Anchors to compare against: AprilTag ~30 fps CPU-real-time;
CPU YOLO ~5–10 fps (not viable at terminal LOS rates, hence the deferred Hailo HAT).

---

## 8. Pass/no-go thresholds — THE MONEY GATE

### 8.1 Curve (a) — gates the ~$740 Tier-2 interceptor order

**Rule (NEXT.md R5, the pre-registered kill number):** the decode range must leave
**t_go ≥ 0.5 s post-handoff** at the real closing speed. Documented anchor:
**R_acq ≈ ≥20 m for a 9 m/s closing speed.**

General relation to apply to this session's numbers:

```
t_go = (R_decode90 - R_streak_burn) / V_closing
```

- `R_decode90` — measured this session (§7.1).
- `R_streak_burn` — range consumed forming the 5-consecutive-detection handoff
  streak at the tag's measured decode *rate* (not the sim's ~7 m NN figure — that's
  a different pipeline; compute it from this session's own rate curve:
  `R_streak_burn ≈ (5 / decode_Hz) × V_closing`).
- `V_closing` — depends on final engagement geometry, which isn't fixed yet. Score
  against **two scenarios**: conservative (target-only speed, ~9 m/s — matches the
  NEXT.md anchor) and aggressive (interceptor's own dash speed ~16 m/s combined
  with the target, ~20–25 m/s head-on).

**GO:** `t_go ≥ 0.5 s` under the conservative (9 m/s) scenario at minimum → unlock
the Tier-2 order. **NO-GO:** loop back to a bigger placard or a camera upgrade
(AR0234, `hardware_order_list.md` §2) — never spend the interceptor money on a
failed curve (a) hoping it'll work out; that's the whole point of running this
session first.

### 8.2 Curve (b) — gates ONLY the $70 Hailo HAT + markerless phase

**No hard pre-registered numeric bar exists in the source docs** — `project_state.
json`/`hardware_order_list.md` deliberately leave this a judgment call ("gates ONLY
the Hailo HAT + the markerless phase"), unlike curve (a)'s NEXT.md R5 number.
Working threshold proposed here (protocol-author judgment, not a sourced number —
revisit once real data exists): **≥50% recall in the tilt-compensated boresight
band (roughly ±20° off-axis) across the 10–25 m operational band.**

- **PASS:** proceed to the real-data retrain / shadow-mode validation path
  (`real_data_pipeline.md`) before spending the $70.
- **FAIL:** do NOT chase it with resolution/crop tricks — that lever is already
  tested-and-rejected in sim (graveyard: "Auto-crop / foveated-crop... TESTED AND
  REJECTED"). A fail here means real-data retrain first, Hailo spend deferred
  further, interceptor unaffected.

---

## 9. FAA / Remote ID / FRIA checklist

- [ ] Target quad is **>250 g** (it is, ~700–900 g class) → FAA registration
      required (Part 107 or recreational, per how you're flying it) — $5/aircraft.
- [ ] **Broadcast Remote ID** solved one of two ways: (a) fly at a **FRIA**
      (FAA-Recognized Identification Area) field — $0, no module needed; or
      (b) fit a broadcast Remote ID module (~$35–60) if no FRIA is available near
      you. Decide by field, before the outdoor flight (`hardware_order_list.md`
      §0c).
- [ ] Registration number physically displayed on the aircraft per FAA rule.
- [ ] Confirm the field is legal for the flight profile (altitude, proximity to
      airports/people) independent of the Remote ID question.

---

## 10. Safety checklist

- [ ] Safety glasses (ANSI Z87) on **any time props are on and a pack is in.**
- [ ] Fireproof LiPo bag for all charging/storage; never charge unattended.
- [ ] LiPo cell checker / low-voltage alarm used before every flight.
- [ ] Fire blanket on-site.
- [ ] Smoke-stopper (VIFLY ShortSaver 2 or equivalent) inline for the first
      power-up after any wiring change.
- [ ] Props OFF during any bench work; verify motor directions + kill switch in
      QGC/the TX **before** the first armed flight of the day.
- [ ] Line-of-sight maintained on the target at all times; briefed abort/RTL plan
      if it flies out of sight or the link drops.
- [ ] Spotter assigned if the pilot's attention is on the camera/tripod rig instead
      of the aircraft.
- [ ] First-aid kit and phone signal confirmed at the field.

---

## 11. Session log template (fill in per pass, field-side)

```
Pass #: ___   Time: ___   Aspect: approach / crossing   Background: sky / horizon / clutter
Speed: full / slow   Attitude: level / banked   Tilt (deg, measured): ___
Battery: pack # ___   Range stations confirmed: Y/N
Sync event (frame idx / ULog t): ___
Anomalies: ______________________________________________
Frame dir: ______________   ULog file: ______________   Video file: ______________
```

Keep one filled sheet (or a spreadsheet row) per pass — it's the only thing that
lets §7's offline scoring reconstruct what each frame directory actually was.

---

## 12. After the session — what "done" looks like

- [ ] `calib.json` (start-of-day + end-of-day) archived with the session.
- [ ] All pass frame directories + ULogs + videos archived, named by the §11 log.
- [ ] Curve (a) `R_decode90` / `R_decode_any` computed, t_go scenarios run (§8.1),
      GO/NO-GO recorded with the numbers.
- [ ] Curve (b) recall-vs-range × position-in-frame computed (§7.2), PASS/FAIL
      against §8.2's working threshold recorded, with the honest small-n caveat
      (§4.6).
- [ ] Pi 5 compute bench numbers recorded (§7.3).
- [ ] Result written back into `docs/project_state.json` (`build_plan.P2`
      changelog + the `hardware` / `detector` stage notes) and `NEXT.md` — this
      session's whole point is to move the money-gate decision, so log it where the
      next session reads it.
