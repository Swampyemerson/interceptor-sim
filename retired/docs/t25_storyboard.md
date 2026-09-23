# T25 Demo Video — Shot-by-Shot Storyboard

> **🔁 RE-RENDERED 2026-07-25 — ram-radius caption pass + ADR-0066 claims-scrub
> (captions only; no shot was added, removed, re-cut, or re-shot).** Output
> `demo_out/t25/t25_demo.mp4` — 30.4 s / 912 frames / 1920×1080 / 30 fps, the same
> 7 shots (0, 1, 2, 3, 3b, 6, 8; shot 7 chase still un-rendered). What changed, in
> the rendered pixels:
> - **Ram radius (ADR-0084).** The RANGE-bar tick is now named by mechanism —
>   `R_lethal 1.5` → **`R_net 1.5`** (`render_hud.radius_label`/`fmt_radius`), and the
>   shot-6 CPA close-out reads *"NET-CLASS criterion R=1.5 m (ADR-0025); ram/contact
>   radius 0.35 m (ADR-0084) — not a modeled collision; GT, scoring only"*. The
>   0.5 m ram figure was NEVER in this cut (it is the retired 7-inch number); the
>   defect was that a net-class criterion was labelled only "lethal radius", which
>   post-ADR-0084 reads as a ram kill it is not.
> - **Comms-denied HELD (tracker FWD-A1 / DEEP-H3).** End-card disclosure was stale
>   ("jam-denial validation in progress") — the ADR-0059 MC FLEW: the config fails
>   closed and the staleness fix is fail-SAFE, **not** recovery. Now stated as such,
>   claim still HELD.
> - **Superseded numbers (tracker FWD-A2).** The n=16 arm alone supports only a
>   79.4% CP lower bound; the end card now also carries the RATIFIED **ADR-0064**
>   claim (Pk ≥ 95% at R=2.8 m, 95% CI, n=72; 98.6%/92.5% at 2.5 m), recomputed and
>   asserted from `logs/mc_pk72_weave_s*.csv` + `mc_fixA_on_weave.csv` at card-build.
> - **Target-model scoping (ADR-0072 add #2 / ADR-0076).** Title + end card now state
>   the target is the legacy markerless stand-in (flat billboard) and that the
>   realistic 3-D quad built later is harder — these numbers do NOT transfer to it.
> - **Tilt card (ADR-0068).** "terminal Pk@2.5 restored to 8/8" now carries the
>   mandated honest note: strict parity NOT met, NO fixed mount adopted, tilt does
>   NOT enable comms-denied recovery.
> The frame audit below re-ran clean (`hud_overlay` honesty check: 0 frames with a
> live cue element after the latch; slow-mo watermark on every retimed frame).

*Builder's spec (2026-07-09): show BOTH ground stereo cameras detecting the
threat first, from near max range, with a HUD element marking the detection
instant; a few seconds later cut to the interceptor's own camera as it takes
off and begins the dash; slow-motion for the last few seconds to interception;
the threat flies a NON-straight-line track; HUD elements throughout say which
sensor is contributing what — including the honesty handoff moment where the
cue goes dark; and the intercept is flown at the fastest speed we can
RELIABLY stand behind.*

**Sequencing (docs/next.md item 6, builder directive):** render AFTER the v3
detector evaluation completes — the demo flies the best detector. The
maneuver-fix deferral is already met (ADR-0058, 14/14 camera-only maneuvering
terminal), so nothing else gates this.

**ADDED 2026-07-10 (builder, mid-session): the FINAL video must feature FOUR
elements — (1) THE TILTING (the up-tilt camera mount / #40 mount-compose),
(2) the final best tracking (detect-then-track, ADR-0058), (3) the stereo
camera handoff (ground stereo → cue → onboard handoff), (4) a great
interception clip.** (2)/(3)/(4) are the existing storyboard (shots 1–7).
(1) is NEW and NOT in the shot list below yet — it is GATED on the #40
compensated re-fly verdict (`docs/adr0067_refly_preregistration.md`), which is
flying at the time of writing. What the tilt shot can honestly show:
- The AVAILABILITY win is a GATED ADR-0067 result and showable regardless of
  the terminal verdict: side-by-side dash frames, target ABOVE-FoV (level cam)
  vs IN-FoV (up-tilt) — dash-above-FoV 32% → 0%. This is the visually
  compelling "target stays in the frame during the pitched dash" beat.
- Whether we show the tilt as an ADOPTED terminal config depends on the re-fly:
  PARITY PASS → show tilt+compensation carrying the intercept; FAIL → show the
  availability win only and flag adaptive tilt (#46) as the live lever (do NOT
  imply an adopted fixed mount). Pre-registration bars decide; no claim ahead
  of the verdict.
Add a tilt shot (proposed slot: between the dash and terminal beats, or as an
A/B inset) to the shot list once the re-fly verdict is logged.

**RE-FLY VERDICT IN (2026-07-10, ADR-0068 addenda) — what the tilt shot can and
cannot honestly claim:**
- **SHOW (gated, honest):** the NOMINAL availability win — side-by-side dash
  frames, target above-FoV (level) vs in-FoV (up-tilt), dash-above-FoV 32%→0%,
  first-det +4 m. This is the visually compelling "stays in frame during the
  pitched dash" beat.
- **Stage-1 (nominal terminal) = FAIL on strict parity** → do NOT show the fixed
  +15° mount as an ADOPTED terminal config. If a terminal number is shown, it is
  the compensated arm's Pk@2.5 8/8 (compensation recovers the cost) with the
  honest note that strict parity wasn't met and no fixed mount is adopted.
- **Stage-2 (comms-jam recovery) = NULL** → the tilt does NOT un-HOLD
  comms-denied recovery. **Do NOT imply the tilt enables comms-denied recovery.**
  The honest framing: the tilt solves perception AVAILABILITY (FoV); the
  remaining blocker is far-range detection. The existing honesty-handoff beat
  (cue goes dark) must keep "works comms-denied" HELD wording (storyboard
  constraint), now reinforced by Stage-2.
- Net: the tilt shot is a NOMINAL-ACQUISITION availability story, not a
  comms-denied-recovery story.

---

## The featured engagement (what "fastest reliable" means, exactly)

- **Config:** the adopted markerless deployment config — detect-then-track +
  cue-gated handoff (`--track --handoff-cue-gate 8`), running-start profile,
  16 m/s dash.
- **Target:** **12 m/s WEAVE** (non-straight-line, per the spec). This is the
  fastest maneuvering regime with a real validation behind it:
  `logs/mc_t21_trackgate_weave12_r2.csv` — n=16 paired seeds, pooled Pk@2.5
  16/16 (median 2.11 m, max 2.48 m), post-handoff camera-terminal 14/14
  (median 2.03 m), zero phantom handoffs, 0/155 false terminal detections
  (ADR-0058, commit `d10c9ca`, verifier-passed). Do **not** stage anything
  faster or claim a config that has only anecdotal flights.
- **Hero-flight candidates from the validated batch** (re-fly the same seed
  with camera/GUI recording, or pick the best of the fresh post-v3 renders):
  - run 1 (r2l): miss **1.09 m**, clean, handoff at 7.5 m, first detection
    15.5 m — the tightest clean flight in the arm
    (`logs/m4_intercept_pronav_20260709T232447Z.csv`).
  - run 13 (r2l): miss **1.17 m**, clean, handoff at 9.8 m, first detection
    15.7 m (`logs/m4_intercept_pronav_20260709T233852Z.csv`).
  - If the rendered flight lands near the arm median (~2.1 m) rather than
    ~1.1 m, ship it anyway and print the arm statistics on the end card — the
    video must not silently cherry-pick beyond choosing a *clean* flight.

## Honesty constraints (non-negotiable, checked at review)

1. **The terminal must be genuinely camera-only.** After the handoff latch,
   no cue-derived readout may appear as a live input on the HUD — the cue
   lamp goes DARK and stays dark. This is true in the code (one-way latch,
   socket closed, holder nulled, ADR-0010 #5 / ADR-0058 `legal_cue_pos()`),
   so the HUD must simply reflect the CSV, never re-animate it.
2. **Do not present jam-resistance as proven.** The handoff latch (link
   structurally closed *at* handoff) is real and may be shown as the
   "comms-denied by construction from here" moment. A *mid-dash* jam is a
   different claim: the adopted config currently fails closed under it, the
   fix is in validation (ADR-0059) — the claim is HELD. The end card must say
   "camera-only terminal (structural latch); mid-dash jam-denial validation
   in progress," not "jam-proof."
3. **Disclose the cue's provenance.** The validated 12 m/s weave arm flew the
   MOCK ground cue (real stereo maneuvering caches are not built — ADR-0056
   follow-ups, design review G6). The ground-stereo shots (1–2) are honest
   *renders of the real T16 rig + T17 detector + T18/T19 triangulation
   pipeline* watching the same weave trajectory, but the cue that steered the
   validated flight is the mock (noise/latency-matched). One HUD footnote or
   end-card line: "ground-track shots: real stereo pipeline (validated
   straight-line, ADR-0053); flown cue: calibrated mock (maneuvering stereo
   validation pending, T20/T21)."
4. **Disclose retiming** (slow-mo) on screen, as the existing demo already
   does (ADR-0032), and keep the "lethal-radius criterion, not a modeled
   collision" label on the close-out.
5. **Ground-truth markers are scoring-only** and must be labeled as such if
   drawn (miss-distance ring, CPA marker).

---

## Shot list

Total target length ~60–75 s. Every HUD readout traces to a CSV/log column
(the `build_demo.py` discipline); "NEW render" means new Gazebo footage is
required, "existing" means committed/regenerable assets already cover it.

| # | Beat (duration) | View | What happens / HUD elements | Source |
|---|---|---|---|---|
| 0 | Title + architecture strip (~4 s) | Static card | One line: "Counter-UAS interceptor — ground stereo cue → dash → camera-only terminal. PX4 SITL + Gazebo, all numbers from logs." Mini architecture diagram. | Existing: README architecture diagrams (`docs/images/architecture_block_diagram.{png,svg}`, `docs/images/process_flow_diagram.{png,svg}`) (shot-list doc retired, ADR-0066). |
| 1 | **Ground stereo detection** (~6 s) | BOTH rig cameras side-by-side (left \| right) | Threat drone enters as a few-pixel speck near max usable range (design detection floor ~160 m EXPECTED / ~60 m WORST, `docs/stereo_design.md`; stage it inside the validated 50–160 m band, e.g. first firm track ~120–150 m). **HUD: "DETECT" flash + sim-time timestamp at the first detection instant on each camera**; per-camera NN boxes; "GROUND STEREO — SEARCHING → TRACKING" state lamp. | **NEW render:** T16 rig world + weave target replay through the T17 ground detector (`ground_v2` or its v3-era successor). Rig capture harness exists (T16, ADR-0045..0049); the weave trajectory replayed from the hero flight's target schedule. |
| 2 | **Track established → cue up** (~5 s) | Rig view + inset map | Triangulated track appears: range readout with a σ_R uncertainty bar growing with R² (ADR-0053, measured exponent 2.003); "TRACK MSG → INTERCEPTOR, 10 Hz" indicator. **HUD detection-to-track latency printed from the log.** Footnote line per honesty constraint #3 (mock-cue disclosure). | **NEW render** (same pass as shot 1) + `station.py` triangulation output (T18/T19); track numbers from the ground-station CSV (`logs/ground_station_*.csv` format). |
| 3 | **Launch + dash begins** (~8 s) | CUT to interceptor onboard camera (the spec's "a few seconds later") | Takeoff, nose-over, dash to 16 m/s. **HUD: sensor lamp "EXTERNAL CUE (ground stereo)" lit; camera lamp "SEARCHING"; speed/range tapes; pitch readout** (the ADR-0060 nose-down pitch is *visible* — the horizon drops; an honest touch worth one caption: "dash pitch ≈ 30° nose-down"). | **NEW render:** re-fly hero seed with onboard frame capture (`build_demo.py` pipeline, demo_out workflow). |
| 4 | **Terminal acquisition — detect-then-track** (~6 s) | Onboard camera | First in-range NN detection (~13–16 m per the r2 arm); **HUD: "NN ACQUIRE" flash → persistent CSRT track box with track-age counter; "NN RE-VALIDATE" tick every 8 frames.** If the flight log shows a rejected phantom (`phantoms_ignored` > 0), caption it: "phantom detection rejected — track continuity gate." | **NEW render** (same flight); events from the flight CSV + tracker stats line. |
| 5 | **HANDOFF — the honesty moment** (~4 s, brief speed ramp-down) | Onboard camera | Handoff latch at ≤10 m: **cue lamp flips EXTERNAL CUE → dark; terminal lamp lights "CAMERA-ONLY"; one-line caption: "ground link closed — structurally unreadable from this tick" (ADR-0010 #5).** Range + closing-speed readouts now sourced from camera filters only. | **NEW render**; latch tick + handoff range from the flight CSV (`handoff_range_m`). |
| 6 | **Terminal slow-mo to intercept** (~10 s, retimed) | Onboard camera, SLOW-MO | The last ~2–3 s of flight retimed; weave visibly crossing the frame; CSRT box holding through the maneuver; **"SLOW MOTION (retimed)" watermark per honesty constraint #4**; proximity close-out overlay at CPA: "closest approach X.XX m — lethal-radius criterion, not a modeled collision." | **NEW render**; CPA and miss from the flight CSV; overlay per `build_demo.py` close-out. |
| 7 | **Chase-cam replay** (~8 s) | Wide world camera | Same intercept from outside, real-time then slow-mo at CPA; trajectory ribbon + scoring-only CPA marker (labeled). | **NEW render** (chase capture, existing `compose_demo.sh`/`build_demo.py` path). |
| 8 | **Results end card** (~6 s) | Static card | The validated numbers, three-level honest: "12 m/s weaving target, n=16 validation arm: pooled Pk@2.5 16/16 · camera-only terminal 14/14 (median 2.03 m) · zero phantom handoffs (ADR-0058)." Plus the two disclosure lines from honesty constraints #2 and #3. | Existing: `logs/mc_t21_trackgate_weave12_r2.csv` / ADR-0058. |

## Frame audit (do this before publishing)

Pull **several in-between frames** (not just the beat frames) and check each
against the flight CSV row at that sim-time:

- [ ] No cue-derived HUD element lit on ANY frame after the handoff tick
      (scan every frame from latch to CPA, not a sample).
- [ ] Camera lamp/state matches `coverage`/detection events in the CSV
      (no "TRACKING" shown during a detection gap — show "COAST" honestly).
- [ ] Range tape monotonicity vs. the CSV `r_hat`; speed tape vs. EKF
      velocity; handoff caption timestamp = the CSV latch tick.
- [ ] Shot-1/2 detection-instant timestamps match the ground-station log.
- [ ] Slow-mo watermark present on every retimed frame.
- [ ] End-card numbers byte-match ADR-0058 / the r2 CSV.

## Production notes

- Pipeline: `scripts/build_demo.py` (HUD overlay from CSV, retiming
  disclosure) + the demo_out workflow in `demo_out/README.md`; the shot-1/2
  ground-stereo pass is the only genuinely new machinery (rig-camera frame
  capture during a target-only replay — the T16 capture harness is the
  starting point).
- Render at RTF ≈ 1 on an idle machine (batch-hygiene rule); one sim at a
  time.
- The GUI/GPU may be used for these renders (the headless-by-default rule
  exempts the final demo).
- Keep every intermediate CSV/log for the audit trail; the video description
  links this storyboard and ADR-0058.
