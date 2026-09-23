# Pre-registration 2: Gazebo pursuit cross-check RE-FLY (post tick-trace fixes)

Registered 2026-09-23, BEFORE any flight of this arm. Follow-up to
`docs/xcheck_gazebo_pursuit_prereg.md` (registered FAIL: median CPA 1.510 m vs
matched-optics isim 0.122 m) and to the registered diagnosis
`isim/specs/xcheck_tick_trace_2026-09-23.md`, whose three evidence-based fixes
this re-fly carries. **Flying is NOT part of the work that produced this doc**
— this registers the expectation first.

## What changed since prereg 1 (the three fixes, each with its evidence)

1. **S2 — pose-range channel (flight code + driver).** The tick trace measured
   the AABB box-width range 15-25% short on a rotated/perspective tag (KF range
   ran 0.4-1.1 m short; per-flight bias vs CPA corr = -0.84) while the same
   detector's PnP pose range was clean (-0.03 m bias, 0.109 m spread, n=415).
   `flight/pursuit_terminal.py` now accepts an OPTIONAL `det_range_pose_m`
   (pursuit-scoped only, `SUPPORTS_POSE_RANGE`; the stock/tag terminals' AABB
   channel stays under the separate ADR-0105 ruling), and the driver
   (`scripts/gazebo_pursuit_crosscheck.py`) supplies |pose_t|. Honesty: pose_t
   is the same detector's camera-pixel output — no ground truth. The
   no-pose-range path is byte-identical to the pre-change module (regression
   test against the pre-change golden fixture,
   `flight/tests/test_pursuit_terminal.py::test_pose_range_absent_is_byte_identical_to_prefeature_code`).
2. **S3 — every-tick detect (driver).** The old `n_tick % 2 == 0` gate in
   `run_mavsdk_mission` capped consumption at ~9.6 Hz (measured 5.2 det/s
   consumed in ENGAGE vs isim's 25.5); the gate is removed. Tick trace showed
   detect ticks cost only +4 ms, but clause C2 below re-checks tick dt live.
3. **S4 — honest ENGAGE-regime plant fit (instrument honesty).** The deployed
   isim fit was dash-regime and ~2x too agile horizontally in the braking/
   lateral ENGAGE regime. `scripts/fit_vehicle_engage.py` re-fit the same
   model class on the 8 cross-check flights' ENGAGE segments →
   `isim/fits/vehicle_gazebo_x500_engage.json` (the dash fit is untouched —
   the regime rule cuts both ways). Verification against the tick-trace band:
   replaying the same command streams, the new fit's horizontal first-order
   lag is delay/tau 0.07/0.78 and 0.01/0.78 s vs the real vehicle's 0.22/0.54
   and 0.18/0.60 — the (delay, tau) SPLIT differs (the model has no free
   pure-transport-delay knob; the fit ridge is shallow) but the TOTAL
   effective lag matches within ~10% (0.79-0.85 s vs 0.76-0.78 s). Known
   remaining infidelity, declared: vertical stays pessimistic (model tau 1.08
   vs real 0.30 s), and the measurement-noise model still under-models the
   live PnP spread ~1.6x (0.067 vs 0.109 m at 6.5 m).

## The honest isim re-prediction (n=50 seeds/arm, run BEFORE the re-fly)

`scripts/xcheck_isim_reprediction.py` →
`logs/xcheck_isim_reprediction_20260923.csv`, same matched-optics scenario as
prereg 1 (canonical crossing, fx 540.3, tag 0.5 m, rear-facing, no scatter):

| arm | plant fit | cadence | pose supply | median | p90 | %<=0.35 m |
|---|---|---|---|---|---|---|
| (a) sanity pin | dash (old) | full | no | **0.122 m** | 0.195 | 100% |
| (b) honest plant | ENGAGE (new) | full (~20/s) | no | 0.313 m | 0.371 | 80% |
| (c) + honest cadence | ENGAGE | 5.2 det/s | no | 0.087 m | 0.177 | 100% |
| (d) + pose supply | ENGAGE | 5.2 det/s | yes | 0.087 m | 0.177 | 100% |
| (d10) band point | ENGAGE | 10.4 det/s | yes | 0.136 m | 0.230 | 100% |
| (dfull) band point | ENGAGE | full | yes | 0.313 m | 0.371 | 80% |

Arm (a) reproduces prereg 1's 0.122 m exactly — the harness is pinned to the
original prediction, and (since it exercises the default paths end-to-end) it
doubles as the bit-stability check that the new pose-range/cadence knobs are
inert when off. (d) equals (c) to ~13 significant digits per seed as expected
(same slant computed by a different float ordering) — isim's synthesized
box is an ideal pinhole side with no AABB defect, so the pose fix is a
Gazebo-side repair isim structurally cannot see; a liveness probe (pose range
corrupted 1.3x → miss 0.3 → 1.1-1.8 m) confirmed the channel is wired, not
inert.

**Two findings the re-prediction itself produced, registered here:**

- **The "sim was flattering the plant" hypothesis is REFUTED as the full
  story.** The tick trace allowed for the honest plant re-fit to reproduce
  ~1-1.5 m; it does not — arms (b)-(d) predict 0.09-0.31 m. In isim's world
  (unbiased range) the 2x-slower plant costs only ~0.2 m of the 1.39 m gap.
  The dominant attributed driver of the Gazebo miss remains the S2 range-bias
  channel (per-flight corr -0.84) and its couplings (coast latched on a
  biased ruler; miss-abort at true 4-8 m), which the pose-range fix removes.
- **Detection cadence has a real, counterintuitive effect on the new plant**
  (mechanism traced, deterministic, survives zero pixel noise): denser
  decodes fill the acquisition window ~0.2 s sooner, the terminal hands off
  to Phase B earlier, and on the laggier honest plant that earlier-handoff
  trajectory ends ~0.2 m wider (paired seeds: c<b in 45/50). Because the
  FIXED driver detects every tick, the re-fly's delivered cadence is
  uncertain (~10 det/s expected: 2x the old gated 5.2; decode-success
  limited), so the registered prediction is a BAND over cadence, not a point.

## Registered expectation for the re-fly

Best estimate at the expected ~10 det/s: **median CPA ~0.14 m**; honest band
**0.09-0.31 m** across the plausible delivered-cadence range (5 det/s - full
rate), with p90 up to ~0.4 m. This is still a best-case-leaning number: the
launch cue remains error-free (`given-perfect` — upper-bound framing per the
assumptions register), isim's noise model under-carries the live PnP spread
~1.6x, and the plant fit's delay/tau split is imperfect. The prereg criteria
below are therefore looser than the band, exactly as prereg 1's were.

## Re-fly configuration (registered)

- Identical to prereg 1 in every respect (Gazebo Harmonic + PX4 SITL,
  `gz_x500_mono_cam`, apriltag world, canonical crossing rotated to +X,
  target 9 m/s, `--engage-max-s 25`, fresh sim boot per flight, idle machine,
  n = 8 flights, centre-to-centre CPA scoring from scoring-only gt) EXCEPT:
- the FIXED driver: every-tick detect (S3) + `tag_range_m` = |pose_t|
  supplied to the pursuit terminal as `det_range_pose_m` (S2). No other
  flight-code or config change; no tuning.

## Adopt / reject criteria (registered before flying)

The re-fly PASSES iff, over the 8 flights:

1. >= 6/8 reach CPA <= 1.0 m, AND
2. median CPA <= 0.5 m, AND
3. >= 2/8 flights reach CPA <= 0.35 m, AND
4. 0 failsafe aborts before the pass (post-pass endings do not count), AND
5. every flight consumes camera detections through ENGAGE (anti-mirage
   clause, unchanged from prereg 1).

**Instrument clauses (fix-effect-observed; scored from the tick-trace
instrument on the new CSVs, pass/fail independent of CPA):**

- C1 (S2 landed): median KF range bias (r_hat - r_true) at the last inbound
  6 m crossing within +/-0.30 m (was -1.07 m).
- C2 (S3 landed, loop intact): in-ENGAGE consumed cadence >= 9 det/s AND
  median driver tick dt <= 0.06 s (the every-tick decode must not stretch
  the loop).

## What the outcomes would mean (written now)

- **PASS (criteria 1-5 + C1-C2):** the transfer gap is closed as attributed —
  measurement-channel bias plus a flattering plant fit, both repaired
  honestly. The `docs/next.md` 0(c) default-swap question reopens as a
  builder decision. Direction-only language still applies: no quantitative
  isim-vs-Gazebo margin quote (arm-asymmetric-instrument rule), and no
  real-world claim (perfect cue, no wind, billboard tag, not the OV9281).
- **NULL with C1-C2 green** (fixes demonstrably landed, CPA still >~1 m):
  the three registered suspects were not the binding constraint — the
  residual is either the pursuit LAW's fit to a slow plant (hot approach /
  late braking: a guidance-design question, now runnable as an ordinary
  pre-registered isim experiment against the honest ENGAGE fit) or something
  outside the tick trace's five suspects. NOT a license to tune in Gazebo.
- **NULL with C1 or C2 red:** the fixes did not land in the flying
  configuration; fix the instrumentation/wiring first — the CPA numbers carry
  no verdict either way (a defect in the fix path invalidates the arm, the
  instruments-are-evidence rule).
- **A result INSIDE the registered band (<=0.5 m median):** closes the
  contradiction-ledger question of the 12x gap; the honest-plant isim becomes
  the sanctioned surrogate for pursuit-terminal design iteration (still
  "lab ranks, Gazebo decides").

## Provenance

- Diagnosis: `isim/specs/xcheck_tick_trace_2026-09-23.md` (instrument:
  `scripts/forensics/xcheck_tick_trace.py`, data
  `logs/xcheck_gz_20260923_fixed/`).
- Re-fit: `scripts/fit_vehicle_engage.py` →
  `isim/fits/vehicle_gazebo_x500_engage.json` (flights/segments/weights
  documented inside the fit file; fit RMS horiz vel 1.31/1.60 → 0.57/0.64 m/s).
- Re-prediction: `scripts/xcheck_isim_reprediction.py` →
  `logs/xcheck_isim_reprediction_20260923.csv` (arms a-d + d10/dfull, n=50
  each).
- Code: `flight/pursuit_terminal.py`, `flight/deploy/real_flight.py`,
  `isim/flight_adapter.py`, `isim/seeker.py`,
  `scripts/gazebo_pursuit_crosscheck.py`; tests in
  `flight/tests/test_pursuit_terminal.py` (pose-range block).
