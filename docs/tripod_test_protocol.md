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
- [ ] **NETWORKING DRY RUN DONE AT THE DESK** — `scripts/field/05_pi_link_check.sh`
      PASSES against the Pi joined to the **actual field phone hotspot**, and
      `01_camera_live_check.sh --pi` → `04_pull_logs.sh --pi` have each been run
      once end-to-end from the laptop. Every pass is started by ssh; an ssh that
      does not work is a field day with zero frames. Full recipe (ssh key, second
      Wi-Fi profile, IP-not-`.local`): `docs/field_bringup.md` §3.0.

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
- **Laptop — REQUIRED LIVE, not optional.** Scoring is offline, but *starting a
  pass is not*: the recorder runs on the Pi and is launched over ssh (§6.0b). If
  the laptop cannot reach the Pi, nothing captures. Bring it charged, and prove the
  link at the desk first with `scripts/field/05_pi_link_check.sh` (ssh key, IP vs
  `.local`, clock). It is also how you spot-check frames between passes so a bad
  calibration/exposure isn't discovered after everyone's gone home.
  - **Backup way to start a pass, if the laptop↔Pi link dies at the field:** an ssh
    app on the PHONE. The hotspot HOST always reaches its own clients, even when
    the hotspot has AP/client isolation — strictly more robust than the laptop.
    Install and test it before you leave.
- **Phone hotspot** — it is the network AND the Pi's only NTP source; the frame↔log
  clock join (§6.1) depends on it. Keep mobile data on.
- Tape measure or laser rangefinder, and cones/flags/GPS waypoint markers to mark
  the range stations in §4.
- A phone for slow-mo/overview video of each pass (context record, not the primary
  science data — see §6).
- Full safety kit — see §10; do not skip items to save a trip to the car.

**Placard-specific kit (merged 2026-07-25 from `docs/placard_mount.md` §11 item 2
— this list was written and never folded in):**
- [ ] **Handheld anemometer (~$20).** It is in **no BOM tier**. Without it §4.7's
      wind validity limits are *unenforceable* and crosswind becomes an
      uncontrolled variable in curve (a) — an 18° crab at a 3 m/s crosswind on a
      9 m/s leg spends **half** the placard's whole incidence cone
      (`placard_mount.md` §4.5). Buy one with the rest of the parts order.
- [ ] **3 spare printed frangible shoes + 1 spare pre-printed panel.** The shoe is
      designed to release at 60–80 N on a hard landing; the day's landings are
      what consumes them. A broken shoe with no spare ends the placard passes.
- [ ] **M3 nylon hardware + spare rubber bands** (the frangible link) and the
      **paint-pen-marked index disc**.
- [ ] **A way to read the index angle**: the disc is 15°-indexable and the index
      angle must go on **every** pass card (§4.2b/§11) — without it no frame's
      incidence can be reconstructed offline and §7.1's incidence scoring is
      impossible after the fact.

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
- **For the tag-decode block (curve (a)) the tripod must be ABEAM the target's leg,
  not on its extension** (added 2026-07-25 from `docs/placard_mount.md` §11 item 3).
  The placard is **beam-facing** (§4.2), so the tripod has to sit off to the side of
  the flight line at the briefed **5–6 m standoff** — a tripod placed on the leg's
  extension sees the tag edge-on and records structural zeros. State the intended
  standoff explicitly in the brief, and mark it on the ground before pass 1.
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

Mark **4, 6, 8, 10, 12, 14, 16, 18, 20 m** from the tripod along the approach line
(use a rangefinder or pre-plan the target's AUTO waypoint mission in QGC so the
ranges are GPS-repeatable pass to pass — the ArduPilot target flies a scripted
box/line, so this is buildable once, then reused for every pass). The **4/6/10 m
near stations are the money stations** — they are where the tag is predicted to
actually decode. Keep 12–20 m as the upper bracket (it establishes where the
envelope dies, which is a real number worth having); 25/30 m are dropped as certain
zeros that only cost field time.

**Bin curve (a) in 2 m bins** — the decision hinges on where inside 4–10 m the 90%
line falls, so the resolution has to be there. Score with the shipped defaults plus
a max range that matches the grid you actually shot:

```
--range-bin 2 --max-range 20
```

> ⛔ **THE STATION GRID MUST BE CONTIGUOUS ON THAT 2 m BIN — 14 m and 18 m were ADDED
> 2026-07-25 for this reason.** The gate walks outward from the nearest bin one
> `--range-bin` step at a time and STOPS at the first bin that is missing or
> underpopulated (`scripts/seeker/tripod_score.py` `r90_walk` /
> `gate_verdict`). A stop for `bin_absent`, `bin_underpopulated` **or**
> `end_of_data` is bounded by data you never shot, so the verdict is **UNCERTAIN —
> never PASS**. The old 4/6/8/10/12/**16/20** list left 12–14 and 16–18 empty on the
> 2 m grid, so a placard that DECODES BETTER than predicted (≥90% out past 12 m)
> would have hit the hole and returned UNCERTAIN on a perfectly good field day.
> Two extra cones cost nothing; a re-shoot costs a field day.
>
> The other half of contiguity is HOW you fly it: true range is per-frame (from the
> GPS join, §7.0 step (b)), so a **continuous inbound approach — start at/beyond 20 m and
> fly all the way in to ~4 m without pausing — fills every bin by itself.** Fixed-
> standoff (crossing) passes only ever populate their own bin. Do not let the far
> band be crossing-only.
>
> ⚠️ A PASS also requires the walk to end on a MEASURED failure (`rate_below_90`) —
> i.e. the grid must extend far enough out that the tag actually stops decoding. At
> the predicted 7.10 m `R_decode90` the 8/10/12 m stations already deliver that; the
> 14–20 m marks are the insurance if the placard over-performs.

### 4.2 Aspects

> ⛔ **RE-BRIEFED 2026-07-25 — WHICH ASPECT THE TAG LIVES IN IS THE OPPOSITE OF
> WHAT THIS SECTION USED TO IMPLY** (`docs/placard_mount.md` §11 item 5, §3.6).
> DECISION 1 committed a **single vertical panel held EDGE-ON to flight** (tag
> normal points out the left and right beams) because nose-on it makes **11.8 N**
> of drag at 9 m/s against **0.28 N** edge-on — the target cannot fly its briefed
> ≥9 m/s leg at all with a nose-on panel. Consequences you must brief before pass 1:
>
> - **CROSSING is the tag's PRIMARY aspect.** The tag-decode block — i.e. **curve
>   (a), the ~$740 gate** — is a crossing block.
> - **APPROACH is the tag's ~90° incidence NULL.** A straight-in approach presents
>   the beam-facing tag edge-on to the camera and yields **zero decodes BY
>   GEOMETRY**. A zero on an approach pass is *not* a tag failure and must never be
>   read as one — it is the expected structural zero.
> - Approach passes still earn their place: they are the **NN / curve-(b)** aspect
>   (the NN sees the airframe, not the tag) and the **documented null-aspect
>   control** that proves the null is real. They are just not where curve (a) lives.

- **Crossing (PRIMARY for curve (a))** — target flies a lateral leg at a fixed
  standoff that transects the FOV (mirrors the sim's l2r/r2l crossing regime — the
  regime where the AprilTag goes *invisible* in sim, ADR-0076 add #18e; this is
  exactly the aspect worth checking for real, and `docs/placard_mount.md` §12
  finding 5 notes the sim's tag was WORLD-fixed, so that sim result is not a model
  of this mount). **Use a ~5–6 m standoff for the dedicated tag-envelope crossing
  block** (inside the predicted decode envelope); the old ~15–20 m standoff is
  retained ONLY for the NN/curve-(b) crossing passes, where detection is not
  envelope-limited the same way. A crossing leg at a 5–6 m standoff sweeps **range
  and incidence together**, which is exactly what §4.2b needs.
- **Approach (curve (b) + the NULL-ASPECT CONTROL)** — target flies straight at the
  camera boresight (mirrors the sim's head-on regime). Expect ~zero tag decodes at
  the 0° (beam) index; that is the control result.
- **Nose-on block (the ONLY approach-aspect tag decodes)** — index the placard to
  **90°** and fly approach passes at **≤4 m/s, HARD** (§4.7). This is the only way
  to get approach-aspect *decoded* frames, and therefore the only source of
  approach-aspect training labels for `autolabel_from_apriltag.py`
  (`docs/placard_mount.md` §11 item 6 / §12 finding 7).

**The mission generator emits this geometry for you:**
`configs/target_kakute/gen_tripod_mission.py` writes the AUTO `.waypoints` file
(approach 4→20 m, crossing at the 5–6 m tag standoff) and **REFUSES to emit a
mission that violates §4.1/§4.2** — flying the old 8/17/30 m defaults is now a
hard error, not a silent option (`--off-protocol` is the deliberate escape for the
curve-(b)/NN crossing block). Run `configs/target_kakute/selftest.sh` before you
leave.

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

> ✅ **NOW EXECUTABLE (shipped 2026-07-25).** The ⛔ banner that stood here — every
> decode site constructing a bare `Detector(families="tag36h11")` at the silent
> library default 2.0, with no flag and no record — is closed:
>
> - `pi_capture.py --quad-decimate`, `tripod_score.py --quad-decimate` and
>   `autolabel_from_apriltag.py --quad-decimate` all exist; the default is stated
>   explicitly as the library's **2.0**, and **ADR-0082 PLANS 1.0 for this day**.
> - The value is **STAMPED into `meta.json`** (`quad_decimate` +
>   `quad_decimate_source`) on every session, so after the field day you can still
>   tell what produced the frames — the thing that used to be lost with the session.
> - `tripod_score` **REFUSES** to score a session at a decimate other than the one
>   its `meta.json` records. The §4.2b comparison is therefore a *deliberate* act:
>   ```
>   # (1) the like-for-like re-score, at whatever the session recorded
>   tripod_score.py SESSION --calib calib.json --redecode ...
>   # (2) the RECLAIM read on the SAME frames
>   tripod_score.py SESSION --calib calib.json --redecode \
>       --quad-decimate 1.0 --allow-decimate-mismatch \
>       --stream-fps <the §7.3 bench AT qd=1.0>   # NOT the qd=2.0 bench
>   ```
>   The mismatch is stamped into `gate.json`/`verdict.txt`. **Use the fps bench for
>   the decimate you scored** — qd=1.0 buys range but costs fps, and the burn model
>   goes as 1/fps, so quoting the qd=2.0 rate against a qd=1.0 envelope is the
>   flattering-direction error the gate cannot afford.

**How incidence is actually scored (read this before quoting an incidence number):**

`tripod_score` writes `curve_a_incidence.csv` + a `curve_a_incidence` block in
`gate.json`, and there are **two different things it can produce**:

| Situation | What you get | Why | How you get there |
|---|---|---|---|
| `index.csv` carries a per-frame `incidence_deg` for EVERY binnable frame | a real **decode-rate vs (range × incidence)** curve | the misses have an incidence too, so each cell has an honest denominator | **RUN THE PRODUCER** — `range_truth_join.py … --incidence --mount-index-deg <the pass card's index>` (see the box below) |
| only the tag's own pose is available (the fallback, and what you get if you skip `--incidence`) | an **ANNOTATION**: the incidence *distribution of the decodes* (median/p90/max), and NO rate cells | incidence is recovered FROM the decode, so a rate computed on it would have a denominator of "frames that decoded" — identically 100%, and meaningless | nothing to do — this is what the join writes when no attitude is supplied, and it says so |

The tool says which one it produced and refuses to dress the second up as the
first. To get the rate curve, the per-frame incidence must come from the target
log's attitude + the surveyed tripod position + the mount index angle — which is
why **the index angle on the pass card is not optional** (§11): without it no
frame's incidence can be reconstructed after the day.

> ✅ **THE RATE ROW IS NOW REACHABLE (shipped 2026-07-25).** That recipe used to be
> prose with no code behind it — the protocol told you to score `(range × incidence)`
> while nothing in the repo could write a per-frame `incidence_deg`, so the aspect
> axis of the $740 curve could only ever be the annotation. The producer is
> `scripts/seeker/range_truth_join.py --incidence` (geometry in
> `scripts/seeker/placard_incidence.py`; contract test
> `tests/test_incidence_rate_curve_contract.py`). Run it in §7.0 step (b):
>
> ```
> scripts/seeker/range_truth_join.py --session-dir SESSION \
>     --bin target_flight.BIN \
>     --tripod-lat .. --tripod-lon .. --tripod-alt ..  --clock-offset-s -0.42 \
>     --incidence --mount-index-deg 0        # 0 = beam, 90 = nose-on, 15° steps
> ```
>
> - Attitude comes from the target `.BIN`'s **ATT** records (same log, same clock as
>   the POS track it is already joining); `--attitude-bin` / `--attitude-csv`
>   (`t_utc_s,roll_deg,pitch_deg,yaw_deg`) override the source.
> - **`--mount-index-deg` is mandatory with `--incidence`** — there is no default,
>   because a guessed index rotates *every* frame's aspect by the guess.
> - It is **fail-closed**: no assumed level attitude (a yaw-only attitude export is
>   refused unless you pass `--incidence-yaw-only` by name), no extrapolation past the
>   attitude span, **zero aspected frames REFUSES** (an empty incidence column is not a
>   curve), and a range-truthed frame that could not be given an aspect is **counted**
>   and refuses by default (`--max-incidence-gap-frac`) — because `tripod_score` needs
>   an incidence on **every** binnable frame to call the result a rate, so one
>   aspectless frame costs the whole session its rate curve.
> - It reports a **measured** bound on the yaw-only approximation and a per-frame
>   `incidence_sigma_deg`. Read that sigma before quoting a 15° bin: at 10 m range a
>   2 m GPS sigma alone is ~11° of line-of-sight bearing, comparable to the bin width.
>   Roll/pitch are used by default; for the **beam** mount pitch costs exactly 0°
>   (`placard_mount.md` §3.4) and roll costs 1:1, so the yaw-only difference is
>   normally the bank angle's worth — but it is measured per session, not assumed.
> - Other flags: `--placard-single-face` (default is the adopted **double-sided**
>   print, so incidence is the near face's and stays in 0–90°), `--attitude-sigma-deg`
>   (fold in a MEASURED attitude 1-σ; omitted and stamped as omitted by default,
>   because ArduPilot's ATT record carries no covariance).

Also record the index angle into the score with `--mount-index-deg` so it lands in
`gate.json`, and state the engagement aspect the GO is meant to cover with
`--engagement-incidence-deg` (§8.1).

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

> ⛔ **RE-BRIEFED 2026-07-25 — THE OLD MATRIX SPENT 16 OF ~28 PASSES ON THE TAG'S
> NULL ASPECT.** It scheduled approach-heavy flying and labelled it *"primary
> decode-envelope + recall data"*, which was written before the placard mount was
> designed. With the adopted **beam-facing** panel (§4.2), a straight-in approach
> at the 0° index yields **zero decodes by geometry** — so more than half the day's
> passes would have produced structural zeros that read as a tag failure, on the
> curve that gates $740. The matrix is now organised by **BLOCK**, because each
> block answers a different question and each has its own placard index and its own
> speed cap.

**BLOCK A — TAG-ENVELOPE CROSSING (placard index 0°, beam). This block IS curve
(a); it is the block that decides the ~$740 order. Fly it FIRST, on the best
battery, in the calmest air.** Standoff **5–6 m** (§4.2), which sweeps range
*and* incidence together along the leg (§4.2b).

| # | Background | Speed / attitude | Passes | What it buys |
|---|---|---|---|---|
| A1 | Sky | Full-speed ≥9 m/s | **5** | **the decode envelope — `R_decode90`, the money number** |
| A2 | Sky | Slow control 3–4 m/s | **3** | motion-blur delta vs A1 (the one thing sim cannot model, §4.4) |
| A3 | Sky | Full-speed, index **15° / 30° / 45°** (one pass each) | **3** | the `cos θ` law **past 32.6°**, where it is currently only a HYPOTHESIS (`placard_mount.md` §13) — the single largest information gain available |
| A4 | Sky | Full-speed, banked | **2** | bank = incidence φ (§4.5); a 20° bank costs 6% of range |
| A5 | Sky | Full-speed, camera at **0° tilt** | **2** | tilt cross-check (§3.2) |
| A6 | Horizon | Full-speed | **2** | best-effort |
| A7 | Ground clutter | Full-speed | **2** | best-effort |

**BLOCK B — NOSE-ON APPROACH (placard index 90°). ≤4 m/s, HARD CAP (§4.7).**
The only approach-aspect *decoded* frames in the whole day, and therefore the only
source of approach-aspect training labels for `autolabel_from_apriltag.py`
(`placard_mount.md` §11 item 6).

| # | Background | Speed / attitude | Passes | What it buys |
|---|---|---|---|---|
| B1 | Sky | **≤4 m/s** approach, 20→4 m continuous | **3** | approach-aspect decodes + NN training labels |
| B2 | Horizon or clutter | **≤4 m/s** approach | **2** | best-effort |

**BLOCK C — NN / CURVE (b) (placard index 0°; the NN sees the airframe, the tag is
irrelevant to it). Gates only the $70 Hailo phase — never the interceptor.**

| # | Aspect | Background | Speed | Passes | What it buys |
|---|---|---|---|---|---|
| C1 | Approach, 20→4 m continuous | Sky | Full-speed | **3** | curve (b) approach recall × position-in-frame |
| C2 | Crossing at the **15–20 m** NN standoff | Sky | Full-speed | **2** | curve (b) crossing (`--off-protocol` on the mission generator) |
| C3 | Approach | Horizon / clutter | Full-speed | **2** | best-effort background sweep |

**BLOCK D — the CLOCK-SYNC pass, §4.6b: one per battery segment, non-negotiable.**
It is a low, close crossing through a CPA — i.e. Block A geometry — so fly it as
the first pass of every segment and it costs almost nothing extra.

**~29 passes + one sync pass per battery segment.** Priority if daylight or packs
run out: **A1 → A2 → D → A3 → B1 → C1 → everything else.** A1+A2+D alone still
produce a defensible curve (a); dropping A1 produces nothing.

> **Also record, per pass:** the placard **index angle** and the **measured wind**
> (§4.7 / §11). A pass flown outside §4.7's wind limits is **not invalid data — it
> is data with an uncontrolled variable**; write the wind down and let the scoring
> decide, rather than discovering at 9 pm that half the curve was flown in a 5 m/s
> crosswind you never measured.

**PACK BUDGET — plan 4–5 packs, not 3** (merged from `placard_mount.md` §11 item 7
/ §4.6): the fitted placard costs a **derived 25–35% endurance loss** (`T^1.5`
induced-power scaling), so a 3-pack plan is ~one full block short. With the HOTA
D6 Pro charger in the BOM, plan mid-session recharge cycles rather than flying all
packs back-to-back.

#### 4.6b THE CLOCK-SYNC PASS — one per battery segment, non-negotiable

Add **one dedicated LOW, CLOSE CROSSING pass at the start of every battery
segment**: standoff **≤5 m**, target within ~2 m of the tripod's height, flown at
full speed, straight through the tripod's beam so the range **closes and then
opens** (a real CPA).

Why it is its own row: that is the ONLY geometry in this matrix where
`range_truth_join.py --auto-sync` can resolve the frame↔log clock offset — an
approach pass has a near-constant closing rate, which auto-sync refuses as
unidentifiable by design (§6.1). It must also be CLOSE, because auto-sync aligns
*tag-decoded* ranges against log ranges: at the predicted ~7.1 m decode envelope a
pass flown at 8 m standoff yields **zero** tag-ranged frames to align, and it
refuses again (it needs ≥8 such frames).

**Reuse that segment's offset for every other pass in the segment** via
`--clock-offset-s` — the Pi's clock does not jump mid-battery. And still mark the
§6.1 hand sync event on every pass card: it is the primary source, and it is the
only recovery if the sync pass itself is dropped.

This is intentionally **not** n≥8 paired-seed statistics (CLAUDE.md's sim standard)
— a field afternoon can't buy that. Treat curve (a)/(b) as a first honest read, not
a statistically tight one; note the small-n caveat explicitly when reporting results.

### 4.7 Placard configuration and wind limits — VALIDITY LIMITS, not suggestions

> Imported wholesale 2026-07-25 from `docs/placard_mount.md` §8 (recommended as
> "NEW §4.7" in its §11 item 8 and never merged). These are the conditions under
> which a pass **counts**. They are derived numbers, not vibes — the derivations
> are in `placard_mount.md` §4.3–§4.5/§5.5 and reproducible from its §14.

| Limit | Value | Why (basis) |
|---|---|---|
| Target airspeed, **0° index (beam)** | **no aerodynamic cap** — 9 m/s and beyond is fine | added drag 0.28 N, trim 7.0° |
| Target airspeed, **90° index (nose-on)** | **≤ 4 m/s. HARD.** | nose-on the panel makes 11.8 N of drag; `ANGLE_MAX` is hit at 5.83 m/s and the decode gate fails at ~6.0 m/s — both fail together, and the 48° trim pitch at 9 m/s throws the tag outside its own incidence cone |
| **Steady crosswind component** | **≤ 3 m/s** for a *valid* pass | 9.4° bank + 18° crab already spends **half** the ±32° incidence cone |
| Gust / peak crosswind | **≤ 5 m/s** | 24.7° bank; track hold is lost at 5.6 m/s |
| Total wind, hard no-fly | **> 6 m/s steady or > 8 m/s gusting** | compounding of the above (judgment call, flagged as an assumption) |
| Aircraft **on the ground**, placard fitted | **≤ 3 m/s** wind — or lay it down / hold it. **Fit the panel last.** | calculated tip-over at **4.3 m/s** — the tightest wind limit on this list |
| Briefed bank angle in a turn | **≤ 20°** | costs 6% of decode range; past 30° it eats the cone |
| Leg heading | brief **within ±30° of the wind line** where the field allows | minimises the crosswind component, which is the optically expensive one |
| Packs per session | **4–5**, not 3 | 25–35% endurance loss with the placard fitted |
| PID tune | **Autotune with the placard fitted**, calm air, before any AUTO mission | the CG crosses the rotor plane with the panel on |

**Index setting per block (§4.6):** Block A + C = **0° (beam)**; Block B = **90°
(nose-on), ≤4 m/s**. Re-index on the ground only, with the props stopped; it is a
2-minute change on the 15°-indexable disc. **Write the index angle on the pass
card every time** — §7.1's incidence scoring is impossible to reconstruct without
it, and `configs/target_kakute/gen_tripod_mission.py --placard-index-deg 90`
will refuse to emit a nose-on mission above the 4 m/s cap.

**You need a way to measure wind (§2): a ~$20 handheld anemometer, in no BOM
tier.** Without it every limit in this table is unenforceable and crosswind
becomes an uncontrolled variable in curve (a).

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

### 6.0 SURVEY THE TRIPOD — once per setup, BEFORE pass 1 (hard input, not prose)

> **Nothing downstream works without these three numbers.** Curve (a) and curve (b)
> are binned by TRUE RANGE, and true range is computed by
> `scripts/seeker/range_truth_join.py` from *(target GPS track) − (tripod position)*.
> With no surveyed tripod position there is no range column, `n_binnable == 0`, and
> the money gate returns `UNCERTAIN: no TRUE ranges in the session`. The field day
> produces no verdict. This is the single cheapest way to waste the whole afternoon.

- [ ] **Tripod latitude (deg): `________.______`**
- [ ] **Tripod longitude (deg): `________.______`**
- [ ] **Tripod altitude (m), IN THE LOG'S DATUM: `______`** — ArduPilot `POS`/`GPS`
      `Alt` is **AMSL**, so this must be AMSL too, *not* height above ground.
- [ ] Re-survey (and re-record) if the tripod is MOVED between blocks. One row per
      tripod position, noted on every pass card that used it.

**How to get them, best method first (all $0):**

1. **Use the target aircraft as the survey instrument** (best — same receiver, same
   datum, so the ~1–3 m inter-receiver bias largely cancels): with the Kakute
   powered and GPS-locked, set the target down at the tripod's base for **≥60 s**
   before the first pass, then read that plateau's lat/lon/alt out of the same
   `.BIN` you will score with. Add the **camera height above that spot** to the
   altitude (measure it with the tape — it is typically 1–1.5 m and it matters).
2. QGC's map/plan readout or a phone GPS averaged for a minute (~3–5 m, degrades
   every range number by that much).

**The datum blunder this catches:** giving AGL where the log is AMSL puts a constant
tens-of-metres error into every range. `range_truth_join.py` REFUSES a join whose
constant bias exceeds `--max-bias-m` (default **5.0 m**) and names exactly this
cause — a refusal is the tool working, not a tool bug. A metre-scale bias (tag
placard vs GPS antenna, camera vs survey point) is expected and is absorbed.

### 6.0b Start each pass with the real capture command

The recorder is `scripts/seeker/pi_capture.py`, run **on the Pi** (over ssh from the
laptop, or from an ssh app on the phone). Size the pass with `--duration` or
`--n-frames` rather than stopping it by hand:

```bash
# on the Pi (one pass, ~20 s at 30 fps), tag decode + tag size RECORDED into meta.json
~/interceptor-sim/.venv-pi/bin/python ~/interceptor-sim/scripts/seeker/pi_capture.py \
    --source picamera2 --out ~/interceptor-sim/sessions/pass01 \
    --duration 20 --exposure-us 1000 --width 1280 --height 800 \
    --calib ~/interceptor-sim/calib.json --tag-size 0.35 \
    --quad-decimate 1.0 --run-tag pass01
```

- `--calib` + `--tag-size 0.35` are **not optional**: they put `tag_size_m` and a
  ranged `tags.csv` into the session. Without them the capture-time `tags.csv` is
  presence-only, `--auto-sync` (§7.0 step (b)) has nothing to align, and any later scorer
  that has to guess a tag size shortens every tag-derived range.
- **`--quad-decimate 1.0` is the ADR-0082 planned setting for this day** (§4.2b).
  It is the only configuration that clears `t_go ≥ 0.5 s` at every credible frame
  rate; the library default 2.0 fails the gate at both 20 Hz and 14 Hz. The value
  is stamped into `meta.json`, so if you *do* capture at 2.0 the session still
  says so and the frames can be re-decoded at 1.0 afterwards — but the **Pi 5 fps
  bench (§7.3) must then be run at whichever decimate you intend to claim**,
  because the burn model goes as `1/fps`.
- Ctrl-C still works (`pi_capture` writes `meta.json` from a `finally` and marks
  `terminated_early`), but a sized pass is cleaner and self-documenting.
- **Per-pass check, on the Pi, before you fly the next one:**

  ```bash
  S=~/interceptor-sim/sessions/pass01
  test -f "$S/meta.json" && \
    [ "$(tail -n +2 "$S/index.csv" | wc -l)" = "$(ls "$S/frames" | wc -l)" ] && echo PASS-OK
  ```

  That second test is the one that catches a silently truncated session — frames on
  disk that the index never recorded (or vice versa).

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

### 6.1 Time sync — the marked sync event is the PRIMARY clock source

Frame timestamps (Pi wall clock) and the target's log (autopilot GPS-UTC) must agree
well within one range bin's time-of-flight (~2 m of travel at 9 m/s ≈ 0.22 s per
bin — keep sync error well under that). Two independent things to do, in this order:

1. **NTP-sync the Pi's clock** (phone hotspot) within a few minutes of the first
   pass. `scripts/field/05_pi_link_check.sh` measures the laptop↔Pi offset and
   grades it against the 0.22 s bar — run it once the hotspot is up.
2. **Mark one clear sync event PER PASS, visible in both streams**, and write it on
   the pass card. This is the **primary** clock source, not a backup — e.g. the
   pilot flies the target directly over the tripod at a known moment, or flashes a
   light toward the camera at pass start. Record BOTH sides of it:

   ```
   offset_s = t_ULog(sync event)  -  index.csv t_wall_unix(that frame index)
   ```

   That number is what you hand to `range_truth_join.py --clock-offset-s` (§7.0 step (b)).
   Write down the raw pair (log timestamp, frame index), not just the difference —
   the difference can be recomputed at 9pm; a missing frame index cannot.

> ⚠️ **`--auto-sync` is a CROSS-CHECK, not the plan.** It estimates the offset by
> aligning the tag-decoded range against the log-derived range — and by design it
> **REFUSES a constant-closing-rate geometry**, because with a constant range rate a
> clock error is mathematically indistinguishable from a constant range bias
> (`range_truth_join.py`, the `UNIDENTIFIABLE` refusal; its own self-test pins this).
> **CORRECTED 2026-07-25 (this used to say "the capture matrix is approach-heavy —
> i.e. it is mostly exactly that geometry").** The §4.6 re-brief made the matrix
> CROSSING-majority, and a 5–6 m crossing leg *does* fly through a CPA, so
> auto-sync is now identifiable on most Block A passes rather than refusing them.
> The plan does **not** change: the hand-marked §6.1 event stays PRIMARY (it is
> free, it survives a pass with no decodes, and it does not depend on the tag), and
> the §4.6b sync pass + `--clock-offset-s` reuse stays the routine. What changes is
> that auto-sync is now a *usable* cross-check on more passes — run it on one and
> compare it against the hand offset; a disagreement is a real finding.
> Auto-sync still only resolves a pass whose range rate REVERSES: one that
> flies through a CPA (see the dedicated sync pass in §4.6). Every other pass needs
> the hand-marked offset above. A pass with neither is unrecoverable: no clock, no
> range truth, no curve — permanently.

> ✅ **THE EXPLICIT-OFFSET PATH IS NOW CHECKED TOO (2026-07-25).** Until today every
> survey / altitude-datum / bias / residual test in `range_truth_join.py` lived
> inside `if do_auto_sync:` — so the path this protocol tells you to *actually use*
> (`--clock-offset-s`, on most passes) had **no integrity checks at all**. A 1.1 km
> survey error, an AMSL-vs-AGL datum blunder and a 3 s clock error each exited 0
> with empty warnings. Now, whichever way the offset was chosen, the tool checks the
> tag-derived range against the log-derived range **at the offset actually used**
> and REFUSES (exit 2) on an implausible residual or constant bias. Two things
> follow for the field:
>
> 1. **A wrong hand-marked offset is now caught**, because on a closing pass a
>    `dt`-second clock error becomes a `|dR/dt|·dt` constant range error — a 3 s slip
>    at 4.5 m/s is 13.5 m of bias against a 5 m ceiling. That is a refusal, not a
>    silently wrong curve.
> 2. **A pass with no tag decodes at all cannot be cross-checked**, and an un-run
>    check is not a passed check — so it REFUSES unless you pass
>    `--allow-unverified-survey` (legitimate on the far NN-only crossing block,
>    where the survey is inherited from that battery segment's §4.6b sync pass). The
>    override is stamped into `range_truth.json:integrity`.

---

## 7. Offline scoring plan (after the field day)

### 7.0 Scoring sequence — run these THREE, in this order

> **Read this before you run anything.** Pointing the scorer at a session and
> expecting a verdict does not work, and the way it fails is quiet: it prints
> `GATE UNCERTAIN — no TRUE ranges in the session`. That is not a bad field day,
> it is a **missing step**. The scorer bins by TRUE range; nothing in the capture
> writes one. `range_truth_join.py` is what turns the target's log + the surveyed
> tripod position (§6.0) into `index.csv:true_range_m`. Verified end-to-end on a
> synthetic session 2026-07-25: step (a) alone → `GATE UNCERTAIN`; after step (b)
> the identical command → a real `GATE PASS/FAIL`.
>
> **Note the two different interpreters.** They are not interchangeable:
> `.venv-seeker` has `onnxruntime` (curve b) but **no `pymavlink`/`pyulog`**, so it
> cannot read a `.BIN`/`.ulg`; `.venv` has the log readers and `matplotlib` (plots)
> but no `onnxruntime`. Running step (b) under `.venv-seeker` exits 1 with
> `ImportError: pymavlink is required to read ArduPilot .BIN logs` (measured) —
> loud, not silent, but it will stop you at 9pm if you do not expect it. (It is the
> LOG READERS that need `.venv`; a pre-exported `--csv` track works under either.)

**(a) DECODE PASS — produces the tag ranges. It WILL say UNCERTAIN; that is expected.**

```bash
.venv-seeker/bin/python scripts/seeker/tripod_score.py SESSION_DIR \
    --calib calib.json --tag-size 0.35 --redecode \
    --out-dir logs/tripod_score
```

`--redecode` is the offline re-decode §6 asks for (it ignores any capture-time
`tags.csv`, so the detector is decoupled from the recorder). It writes
`logs/tripod_score/<session>/tags.csv` carrying a per-frame `tag_range_m`. Expect
`>>> GATE UNCERTAIN <<<` with the reason naming `range_truth_join.py` — this pass
exists to make the tag ranges, not a verdict.

**(b) RANGE-TRUTH JOIN — the step the whole verdict hangs on. Different interpreter.**

```bash
.venv/bin/python scripts/seeker/range_truth_join.py \
    --session-dir SESSION_DIR --bin target_flight.BIN \
    --tripod-lat <§6.0> --tripod-lon <§6.0> --tripod-alt <§6.0, AMSL> \
    --clock-offset-s <from the §6.1 sync card> \
    --tags-csv logs/tripod_score/<session>/tags.csv \
    --incidence --mount-index-deg <this pass's placard index, §4.2b>
```

- `--tripod-lat/lon/alt` are the §6.0 survey. All three or none; the altitude must
  be in the **log's** datum (ArduPilot `POS`/`GPS` `Alt` is **AMSL**).
- `--clock-offset-s` = `t_ULog(sync event) − index.csv t_wall_unix(that frame)` (§6.1).
- `--tags-csv` is what `--auto-sync` aligns against; point it at step (a)'s OFFLINE
  re-decode rather than the capture-time file. (Harmless but unused when you supply
  `--clock-offset-s` without `--auto-sync`.)
- **`--auto-sync` is the CROSS-CHECK; the §6.1 hand offset is the plan.** Auto-sync
  refuses a constant-closing-rate APPROACH as *unidentifiable* **by design**, so it
  will (correctly) refuse the Block C/D approach passes. Since the §4.6 re-brief the
  crossing blocks DO fly through a CPA, so it can now resolve most Block A passes —
  use that: run it on one pass per segment and compare against the hand offset
  (§4.6b). Then reuse the segment's offset via `--clock-offset-s`.
- It writes `true_range_m` + `range_quality` / `range_sigma_m` back into
  `index.csv`, atomically, keeping a pristine `index.csv.bak`. Exit `2` = REFUSED
  (an unsafe join) — read the reason, it names which of {wrong log, bad survey,
  datum blunder, wrong clock offset, unidentifiable clock, implausible geometry,
  no attitude / no aspected frames} it caught. **A refusal is the tool working.**
- **`--incidence --mount-index-deg <index>` is what makes §4.2b's decode-RATE vs
  incidence curve possible** — it also writes `incidence_deg` (+ `incidence_quality`
  / `incidence_src_dt_s` / `incidence_sigma_deg`) from the log's **ATT** attitude,
  the surveyed tripod position and the pass card's index angle. Skip it and
  `tripod_score` honestly degrades that curve to the decodes-only ANNOTATION (the
  §4.2b table). The index angle has **no default**; see the ✅ box in §4.2b for the
  fail-closed rules and the `incidence_sigma_deg` caveat.
- **`--tags-csv` is now load-bearing on the explicit path too**: the tag ranges are
  what the survey/datum/bias cross-check compares against (see the ✅ box in §6.1).
  Point it at step (a)'s offline re-decode on *every* pass, not just sync passes.
- Two deliberate overrides, both stamped into `range_truth.json:integrity`:
  `--allow-unverified-survey` (a pass with <8 tag-ranged frames — the cross-check
  cannot run) and `--allow-implausible-geometry` (median target height >150 m above
  the tripod, which is otherwise refused as an altitude-datum blunder).
  > ⚠️ **You WILL need `--allow-unverified-survey`, and it is not free.** With the
  > beam-facing placard, an **approach pass decodes nothing** (§4.2) — so every
  > Block B/C approach pass legitimately has zero tag ranges and the cross-check
  > genuinely cannot run there. Two consequences:
  > 1. **Get the survey right on a pass that CAN check it.** The Block A/D
  >    crossing passes have tag ranges; if their join is clean, the same survey
  >    and the same battery-segment offset are what you carry to the approach
  >    passes. Never let the FIRST join of a session be an overridden one.
  > 2. **The override tightens the other guard.** With the cross-check off, the
  >    tool refuses a median target height above the **30 m advisory** (not the
  >    150 m ceiling), because an AMSL-vs-AGL blunder lands at ~100 m and would
  >    otherwise sail through unverified. Forcing past that needs **both**
  >    overrides, and both are recorded. If you hit it, the answer is almost
  >    always that `--tripod-alt` was given AGL.
- Repeat per pass directory.

**(c) RE-SCORE — the real verdict.**

```bash
.venv-seeker/bin/python scripts/seeker/tripod_score.py SESSION_DIR \
    --calib calib.json --tag-size 0.35 --redecode \
    --range-bin 2 --max-range 20 \
    --stream-fps <the §7.3 Pi 5 bench number> \
    --mount-index-deg <this pass's placard index, §4.2b> \
    --out-dir logs/tripod_score
```

Same command as (a) — the session now carries the ranges, so curve (a), curve (b)
and the money gate all resolve. Two additions:

- `--quad-decimate` is **not** passed here on purpose: with no flag the scorer uses
  the value the session's `meta.json` recorded, and REFUSES if you ask for a
  different one (`--allow-decimate-mismatch` is the §4.2b reclaim comparison).
- `--mount-index-deg` puts the pass card's index angle into `gate.json`, and
  `--engagement-incidence-deg` states the aspect the GO is meant to cover (§8.1).
  Left at 0° the verdict carries a printed warning that it is a **dead-on** GO.

- `--stream-fps` is the **§7.3 bench** number (sustained end-to-end decode Hz on the
  real Pi 5 at the flying `quad_decimate`), not the capture rate. The scorer refuses
  to invent one; if you omit it and the session's own recorder rate is used, the
  verdict line says so in words. The gate flips across ~24 fps, so this matters.
- Re-running with `--redecode` decodes the frames a second time (a few minutes,
  offline). To skip that, copy step (a)'s `logs/tripod_score/<session>/tags.csv`
  into `SESSION_DIR/` and drop `--redecode` — the scorer reads both tags.csv
  schemas.

**Want the PNG curves?** Re-run (c) under `.venv` (matplotlib) — curve (b)'s NN half
then cleanly no-ops (no `onnxruntime`), which is fine: the plots are for curve (a).

**Three UNCERTAIN messages and what each actually means:**

| It prints | It means | Do this |
|---|---|---|
| `no TRUE ranges in the session` | step (b) was never run for this pass | run (b) |
| `bounded by MISSING DATA (bin_absent / bin_underpopulated / end_of_data)` | the ≥90% band ran into a station you never shot — a lower bound, not a measurement | §4.1: contiguous 2 m grid, shoot farther, or re-shoot that station |
| `MISSING MEASUREMENT: capture/decode stream_fps` | no `--stream-fps` and no measured rate in `meta.json` | pass the §7.3 bench number |

### 7.1 Curve (a) — AprilTag decode envelope

> §7.0 is the path you actually run; this section explains what it computes and
> gives the manual equivalent if you want to check it by hand.

1. Run `autolabel_from_apriltag.py --frames DIR --calib calib.json --tag-size
   TAG_EDGE_M --drone-size 0.35 --out DATASET` per pass directory. It reports the
   tag's label/decode rate — that print IS the raw curve-(a) input. (Its "decode
   ceiling" line is only meaningful on target-PRESENT footage; pointed at
   target-free negatives it is not measuring the tag.)
2. Bin decode success (tag found vs not) by the frame's ground-truth range (from
   the ULog track, §6) into the same range bins as §4.1 (or finer, e.g. 2 m bins
   like `approach_recall.py`'s convention).
3. Report **two numbers per pass-set**: `R_decode_any` (farthest range with ANY
   successful decode) and `R_decode90` (farthest range where the decode rate
   *sustains* ≥90% inward) — the streak needs a sustained rate, not a lucky single
   frame, to actually form a handoff.
4. **Score decode against `(range × INCIDENCE)`, not range alone** (§4.2b;
   `placard_mount.md` §11 item 9). Two passes at 0° and 30° incidence are
   *different experiments* and pooling them smears the curve. The tool writes
   `curve_a_incidence.csv` and a `curve_a_incidence` block in `gate.json`; pass
   `--mount-index-deg <the pass card's index>` and `--incidence-bin-deg 15`
   (15° = the mount's own index step). Read §4.2b's table first.
   **To get a RATE curve rather than an annotation, §7.0 step (b) must have been
   run with `--incidence --mount-index-deg`** — that is what puts a per-frame
   `incidence_deg` on the MISSES as well as the decodes. Check
   `gate.json:curve_a_incidence.rate_computable`: `true` = a measured rate;
   `false` = the incidence *distribution of the decodes* only. If some frames
   carry an aspect and some do not, the scorer still shows cells but flags them
   `cells_are_partial` / `rate_is_measured=0` — a **shrunken denominator**, not
   the session's decode rate; re-join so every binnable frame has an aspect.
5. **Re-decode the same frames at `quad_decimate ∈ {2.0, 1.0}`** (§4.2b) and
   report both envelopes. Zero field time; it is the whole no-second-field-day
   reclaim lever, and it is now runnable (`--quad-decimate` +
   `--allow-decimate-mismatch`). Use the §7.3 fps bench **for the decimate you
   scored**.

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
       --calib calib.json --tag-size 0.35 --redecode \
       --range-bin 2 --max-range 20 --stream-fps <§7.3 Pi 5 bench> \
       --out-dir logs/tripod_score
   # -> curve_b_recall.csv (n-mono, PRIMARY) + curve_b_recall_drone_finetuned_quad_v2.csv (BAR)
   #    plus the primary-minus-bar delta in verdict.txt and gate.json:curve_b_models
   # (--no-weights-bar scores the primary only; --weights / --weights-bar override either)
   ```
   This is step (c) of §7.0 — it only produces curve (b) after the range-truth
   join (§7.0 step (b)) has run, and `--stream-fps` is the §7.3 bench number, not the
   capture rate.
   Input modality is resolved **per model** (gray for the gray-native nn_tier weights,
   color for the sim weights) — on the mono OV9281 the gray step is a bit-exact no-op,
   so no flag is needed in the field.
   **Read the PRIMARY row against §8.2's threshold; a low BAR row is expected and gates
   nothing.**
2. Ground-truth each frame's range/position with the tag pose where the tag
   decoded, and with the log-derived range beyond that (§6) — this is the one
   place curve (a)'s decode ceiling matters for scoring curve (b): don't drop
   frames beyond the tag's decode range, that's exactly the far-band data curve (b)
   needs (`real_data_pipeline.md`'s "tag-miss frames dropped" rule is about
   TRAINING labels, not this scoring pass).
   > ⚠️ **The shipped scorer does NOT do this yet.** `tripod_score.curve_b` builds
   > its truth box from the TAG's recovered pose, so it can only score frames where
   > the tag decoded — curve (b)'s far edge is bounded by curve (a)'s decode
   > ceiling (the tool says so in its own note and flags it rather than dropping it
   > silently). Truthing the far band needs log-derived truth boxes, which is not
   > built. Until it is: **beyond the decode ceiling, curve (b) is a fire-rate read
   > only** (how often the NN boxes *something*), not a recall number — and §8.2's
   > 10–25 m band is only answerable where the two overlap. `tripod_score` reports
   > that overlap explicitly (`curve_b_band_coverage`); if it says the band is not
   > answerable, do NOT quote the overall recall as the §8.2 answer.
3. Bin recall by **range** (as approach_recall.py does) **AND by position-in-frame**
   (e.g. top/middle/bottom third, or degrees off the vertical boresight) — ADR-0076
   add #18k's finding was that recall vs range alone hid a 100%-static-vs-0.8%-
   in-flight gap that only position-in-frame explained. Do not report range-only
   numbers as the final read.

### 7.3 Compute bench (Pi 5 fps) — separate, no flight needed

Bench the real Pi 5 the same day (`build_plan.P2` task 5): sustained AprilTag fps
and CPU-YOLO fps on the actual hardware (`pi5-emulation-gap` constraint — emulation
cannot measure this).

> ✅ **RUN 2026-07-26 — this section is now MEASURED, not anchored.** Harness:
> `scripts/seeker/pi_fps_soak.py` (750 s soaks, tag in frame, thermals sampled
> throughout, fails closed on a short/cold/tagless run). Results in
> `runs/skr07_tagged/`; reasoning in ADR-0090.
>
> | arm | sustained | decode | max temp | throttled |
> |---|---|---|---|---|
> | AprilTag, `quad_decimate=2.0` (the flying default) | **96.6 fps** | 8.9 ms | 57.9 °C | none |
> | AprilTag, `quad_decimate=1.0` (the reclaim lever) | **40.3 fps** | 23.3 ms | 59.0 °C | none |
> | CPU-YOLO, `n-mono.onnx` @640 | **6.09 fps** | 162 ms | 67.2 °C | none |
>
> **The old anchor here read "AprilTag ~30 fps CPU-real-time" and was never
> measured — it was low by ~3.2×.** The rig really was delivering 30.0 fps, but
> because neither `pi_capture.py` nor `flight/deploy/seeker_loop.py:PicameraSource`
> set picamera2's `FrameDurationLimits`: an inherited default, not the sensor
> (143 fps at 1280×800) and not the CPU (~112 fps of decode throughput). Steady
> state ≈ cold start on every arm, so there is **no thermal derating** with the
> active cooler fitted.
>
> **Which gate each number may feed.** Only the AprilTag arms may supply
> `tripod_score --stream-fps`, and **at the decimate you scored** (§4.2b). The
> CPU-YOLO figure gates the **deferred Hailo/markerless phase ONLY** — it is a
> different pipeline and must never be quoted as curve (a)'s cadence. At 6.09 fps
> the ~5–10 fps CPU-YOLO anchor is CONFIRMED, so the Hailo HAT requirement stands.
>
> **Caveat, in the conservative direction:** measured with tags filling the frame,
> which is more decode work than one distant placard — so the flight case should be
> no slower. The tagless upper bound was 108.6 fps, i.e. the missing tag inflated
> the rate ~12%, which is why the harness refuses a tagless run outright.

---

## 8. Pass/no-go thresholds — THE MONEY GATE

### 8.1 Curve (a) — gates the ~$740 Tier-2 interceptor order

**Rule (NEXT.md R5, the pre-registered kill number):** the decode range must leave
**t_go ≥ 0.5 s post-handoff** at the real closing speed. Documented anchor:
**R_acq ≈ ≥20 m for a 9 m/s closing speed.**

General relation to apply to this session's numbers — stated in the
**INCIDENCE-AWARE** form (merged 2026-07-25 from `placard_mount.md` §11 item 10):

```
t_go = (R_decode90(0°) · cos θ_eng  -  R_streak_burn) / V_closing
```

- `θ_eng` — the **worst-case tag incidence the REAL engagement will present**, not
  the incidence this session happened to fly. **A GO measured at θ = 0° and applied
  to a θ = 30° engagement is not a GO** (`R_decode(θ) = R_decode(0°)·cos θ`,
  `placard_mount.md` §3.2 — DERIVED, and validated in sim only to **32.6°**;
  beyond that it is a listed HYPOTHESIS, which is what §4.6 Block A3/B exists to
  test). Pass it with `--engagement-incidence-deg`; at the default 0° the scorer
  prints a warning on the verdict saying the GO is dead-on only.
- `R_decode90` — measured this session (§7.1), at the incidence the session flew.
- `R_streak_burn` — range consumed forming the 5-consecutive-detection handoff
  streak (not the sim's ~7 m NN figure — that's a different pipeline). **Use the
  RUN-LENGTH form (ADR-0079, what the scorer gates on):**

  ```
  E[T] = (1 - p^k) / (p^k · (1 - p))          # expected frames to get k in a row
  R_streak_burn = (E[T] / stream_fps) × V_closing
  ```

  with `p` = the decode rate at `R_decode90` and `k = 5`. The older mean-rate form
  `(5 / decode_Hz) × V_closing` is **REJECTED as the gate model**: it understates
  the burn, always in the flattering direction for a ~$740 purchase (k=5: ~1.2× at
  p=0.9, ~2.3× at p=0.7, ~6.2× at p=0.5). It is still *reported* for transparency
  (`R_streak_burn_meanrate_m`). Reproduce the comparison with
  `scripts/seeker/streak_burn_derivation.py`.
- `stream_fps` — the **§7.3 bench** decode cadence on the real Pi 5, passed with
  `--stream-fps`. Not the recorder's capture rate: the recorder pays a PNG write the
  flight loop never pays and (with decode off) no decode cost at all. The gate flips
  across ~24 fps at the predicted `R_decode90`, so this is a real input, not a
  formality — the scorer refuses to invent it.
- Honest caveat carried by the tool: real decodes are temporally CORRELATED, not
  independent Bernoulli, so BOTH analytic burns are surrogates. If a session gives
  an EMPIRICAL streak-formation range, prefer it.
- `V_closing` — depends on final engagement geometry, which isn't fixed yet. Score
  against **two scenarios**: conservative (target-only speed, ~9 m/s — matches the
  NEXT.md anchor) and aggressive (interceptor's own dash speed ~16 m/s combined
  with the target, ~20–25 m/s head-on).

**STATE THE GATE IN INCIDENCE-AWARE FORM (added 2026-07-25 —
`placard_mount.md` §11 item 10).** The relation above is the **θ = 0°** case. The
placard is a flat fiducial, so decode range falls as `cos θ`, and the real
engagement will not be dead-on:

```
t_go = (R_decode90(0°)·cos θ_eng − R_streak_burn) / V_closing  ≥  0.5 s
```

where `θ_eng` is the **worst-case incidence the real engagement will present**.
Score it with `--engagement-incidence-deg θ_eng`; `verdict.txt` prints both the
θ=0 number and the incidence-derated one.

> ⚠️ **A GO measured at θ = 0° and applied to a θ = 30° engagement is NOT a GO.**
> `cos 30° = 0.866`, i.e. a 13% haircut straight off `R_decode90` — and the
> derated budget is tight: at the pessimistic corner (`qd = 2.0` + conservative
> camera scaling) the usable cone is **θ_max = 0°**, i.e. **no incidence budget at
> all** (`placard_mount.md` §3.3). Under realistic scaling it is ±32°, which
> ordinary flight attitudes can consume by themselves (target bank in a gust ±10°,
> interceptor elevation from the 2–4 m loft +12–17°, crab in a crosswind 18°).
> If the day comes back marginal, `quad_decimate = 1.0` is the first reclaim
> lever (±48–56° cone) and it costs no field time — §4.2b.

**GO:** `t_go ≥ 0.5 s` under the conservative (9 m/s) scenario at minimum, **at
the engagement incidence you intend to claim**, and with `r90_stop_reason ==
rate_below_90` (a measured cutoff — a band bounded by missing data is UNCERTAIN,
never PASS) → unlock the Tier-2 order. **NO-GO:** loop back to a bigger placard or
a camera upgrade (AR0234, `hardware_order_list.md` §2) — never spend the
interceptor money on a failed curve (a) hoping it'll work out; that's the whole
point of running this session first.

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

- [ ] Target quad is **>250 g** → FAA registration required (Part 107 or
      recreational, per how you're flying it) — $5/aircraft.
      **Mass corrected 2026-07-25** (`docs/placard_mount.md` §4.2 component
      roll-up, via its §11 item 12): **~610 g bare / ~805 g with the placard
      fitted**, not the "~700–900 g class" this line used to carry. The
      >250 g ⇒ register conclusion is unchanged; the number is corrected so it
      is not cited elsewhere as if measured.
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

**Session header — fill ONCE per tripod position, before pass 1 (§6.0):**

```
Tripod lat: ________.______   lon: ________.______   alt (m, AMSL): ______
  survey method: target-on-the-spot / QGC / phone      camera height above that spot (m): ____
Target log file (.BIN): ______________     Calib file: ______________
Hotspot up + Pi NTP-synced (05_pi_link_check offset, s): ______
```

**Per pass:**

```
Pass #: ___   Time: ___   Block (§4.6): A_ / B_ / C_ / D(sync)
Aspect: approach / crossing   Background: sky / horizon / clutter   Standoff (m): ____
Speed: full / slow (___ m/s)   Attitude: level / banked   Tilt (deg, measured): ___
Placard mount index angle (deg): ___   <- MANDATORY (§4.2b): no index, no incidence, ever
WIND: steady ____ m/s from ____   gust ____ m/s   -> inside §4.7 limits? Y/N
quad_decimate used at capture: ____   (pi_capture --quad-decimate; also in meta.json)
Battery: pack # ___   Range stations confirmed: Y/N
SYNC EVENT -- log timestamp t_ULog: ____________   frame index: ______
  (offset_s = t_ULog - index.csv t_wall_unix of that frame; §6.1. WRITE BOTH RAW NUMBERS.)
Is this the battery's CLOCK-SYNC pass (close crossing through a CPA, §4.6b)? Y/N
Per-pass check ran on the Pi (meta.json present, index rows == frame files)? Y/N
Anomalies: ______________________________________________
Frame dir: ______________   Video file: ______________
```

Keep one filled sheet (or a spreadsheet row) per pass — it's the only thing that
lets §7's offline scoring reconstruct what each frame directory actually was. The
sync-event pair and the tripod survey are the two entries that cannot be
reconstructed afterwards: without them that pass has no range truth, permanently.

---

## 12. After the session — what "done" looks like

- [ ] `calib.json` (start-of-day + end-of-day) archived with the session.
- [ ] All pass frame directories + the target `.BIN` + videos archived, named by the
      §11 log — **including the tripod survey and the per-pass sync-event pairs**.
- [ ] **The three-step scoring sequence (§7.0) run per pass**, in order: decode →
      `range_truth_join` → re-score. A pass whose `verdict.txt` still says
      `no TRUE ranges` has NOT been scored.
- [ ] Curve (a) `R_decode90` / `R_decode_any` computed, t_go scenarios run (§8.1),
      GO/NO-GO recorded with the numbers — and the `r90_stop_reason` recorded with
      them: only `rate_below_90` is a measured cutoff that may PASS.
- [ ] **The GO/NO-GO is recorded WITH the incidence it is claimed at** (§8.1's
      `cos θ_eng` form) and with the `quad_decimate` it was measured at — a
      verdict quoted without both is not consumable by the money gate.
- [ ] **The `quad_decimate ∈ {2.0, 1.0}` re-score run on the same frames** (§4.2b)
      and both envelopes recorded. It is free and it is the only reclaim lever
      that does not need a second field day.
- [ ] Per-pass **index angle + measured wind** transcribed off the pass cards into
      the session record (§4.7/§11) — they are the two variables that decide
      whether a pass is a valid data point, and neither is reconstructable later.
- [ ] Curve (b) recall-vs-range × position-in-frame computed (§7.2), PASS/FAIL
      against §8.2's working threshold recorded, with the honest small-n caveat
      (§4.6).
- [ ] Pi 5 compute bench numbers recorded (§7.3).
- [ ] Result written back into `docs/project_state.json` (`build_plan.P2`
      changelog + the `hardware` / `detector` stage notes) and `NEXT.md` — this
      session's whole point is to move the money-gate decision, so log it where the
      next session reads it.
