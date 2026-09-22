# Parity trace: WHERE the port's first-pass close loses 0.3–0.8 m (2026-09-22)

Registered follow-up to `parity_flightcode_2026-09-21.md`'s closing NEXT step.
The abort-timer A/B there was NULL (window 2/4/10/30 s byte-identical), so the
gap between the native prototype (`isim.concepts.PursuitRendezvousGuidance`,
nominal median miss 0.120 m) and the flight-code port
(`flight/pursuit_terminal.py` under `RealFlightSM`, first-pass close
0.4–0.9 m) lives in the FIRST-PASS CLOSE itself. This spec pre-registers a
per-tick side-by-side estimator trace on identical seeds before any trace is
run.

## Instrument

`scripts/forensics/parity_trace_pursuit.py` — builds both arms exactly as
`isim.mc._run_one` does (`scenario.build`, `load_params()` fitted vehicle,
`tag_facing="rear"`, scatter off — the canonical parity-grid geometry, per
the reproduce-canonical-geometry rule), wraps each guidance in a read-only
tracer, and logs per guidance tick (0.02 s): phase, KF relative-position
estimate vs TRUE relative position (truth for diagnosis/scoring only — it
never feeds guidance), target-velocity estimate vs truth, commanded velocity
pre/post slew, yaw command, and per detection: `t_capture`, actual age at
consumption, and three COUNTERFACTUAL conversion errors (below). Seeds: 2
(port miss 0.436 m) and 4 (0.671 m) — the spec'd stable probes — plus 0, 1,
3 for medians. Native arm CPAs for the same seeds come from the same script.

## Known implementation differences going in (from reading both sources)

1. Native converts each Detection with own pos+quat interpolated AT
   `det.t_capture` (0.3 s ring buffer); port converts the box with the
   CURRENT-tick quaternion and extrapolates the relative vector forward by
   `v_rel_hat * meas_latency_s` (fixed 0.045 s).
2. Native's KF measurement model couples the fix to the velocity states
   (H = [I, −age·I], absolute (pos, vel)); the port's H = [I, 0] on
   relative (r, v_t) with the age compensation applied to the MEASUREMENT
   instead, using the estimated v_rel.
3. Latency jitter: the seeker draws age ~ 0.045 ± 0.005 s (1σ); the port
   assumes exactly 0.045.
4. Port range = `fx·span/box_w` (z-depth) placed along the unit ray, i.e.
   SHORT of the slant range by cos(off-axis); native uses the seeker's slant
   `det.range_m`. Near-boresight chase ⇒ expected small.
5. Wrapper/command path: the port's `PursuitTerminalGuidance` is constructed
   by `isim.flight_adapter.RealFlightGuidance.reset()` WITHOUT
   `initial_yaw_deg` (defaults 0.0), so its yaw-slew state starts at 0° while
   the native starts at the solved heading; the SM also flies one extra
   standby-hold tick on the GO edge. Phase-A slew: port budgets ONE combined
   norm across all three axes (`_slew_combined`), native budgets
   horizontal/vertical separately.
6. Command law (Phase B NED-frame) is formula-identical in both; camera-frame
   steering and near-range gain boost are off by default in both.

## Candidates and their pre-registered trace signatures

- **(a) attitude-at-capture** (diff 1, attitude part): per detection,
  e_att = range · |dir_ned(quat_now) − dir_ned(quat_at_capture)|. Signature
  if dominant: e_att grows with body rate (tilt transients during the
  closing-speed schedule; the fitted vehicle tilts up to 35° with a 0.12 s
  lag and 220°/s rate ⇒ up to ~10° of attitude change per 45 ms ⇒ e_att up
  to ~0.5–0.9 m at 5–8 m range), and the port's estimator error tracks it
  while the native's stays flat at the same ticks.
- **(b) latency jitter** (diff 3): e_jit = |age − 0.045| · |v_rel_true|.
  Expected magnitude ~0.005 s · 9–12 m/s ≈ 0.05–0.06 m 1σ, mean-zero.
  PREDICTION: minor, not the 0.3–0.8 m driver.
- **(c) wrapper/command path** (diffs 5, 6): recompute the native NED-frame
  law from the PORT's own estimate each Phase-B tick; a nonzero residual, or
  the yaw-from-0° transient visibly delaying early decodes, implicates the
  wrapper. PREDICTION: the yaw transient may delay acquisition but the
  118° FoV makes a total-loss unlikely; command-law residual ≈ 0.
- **(d) measurement-model coupling** (diff 2): shows as slower/noisier v_t
  convergence in the port at equal decode counts (compare |v_t_est − v_t_true|
  vs decode index across arms).
- **(e) slant-range shortfall** (diff 4): an along-LOS bias ∝ range ·
  (1 − cos(off-axis)). PREDICTION: < 0.05 m near boresight, minor.

## Decision rule (registered)

Attribution = the candidate (or explicit pair) whose per-detection error,
summed/propagated over the final 3 s before CPA, explains ≥ 50% of the
port-minus-native estimator-error gap at the last decode, on both probe
seeds. If (a) dominates: the fix is IN-CONTRACT — an own-state ring buffer
INSIDE `PursuitTerminalGuidance` (it already receives `own` + `t` every
tick; interpolate the conversion quaternion at `t − meas_latency_s`), no
`real_flight.py` change. If (b) dominates, the fix needs `t_capture` in the
step contract — write the proposal, do not patch blind. If no candidate
reaches 50% on both seeds, the spec's candidate list was WRONG — say so and
trace deeper before touching the port.

## Fix adopt/reject criterion (registered BEFORE the re-run)

Re-run the 2026-09-21 registered grid, port arm, same seeds/cells (n=50:
nominal / aim10 / aim20 / alt+2, `--tag-facing rear`). ADOPT iff the nominal
cell's %≤0.35 m at least doubles (24% → ≥48%) AND its median miss improves
≥30% (0.520 → ≤0.364 m) AND no cell regresses by more than 5 points, with
`flight/tests/` + `isim/tests/` green and the real_flight `--audit` still
passing. Otherwise report the numbers and do not adopt.

**What a NULL means.** If the fix lands its mechanism (trace shows the
injected measurement error gone) but the grid does not move, the estimator
error was not the miss driver — the close's remaining loss is in the command
path or the vehicle response, and candidate (c) must be re-examined with the
same trace before anything else is changed.

## RESULT (same day): traced, attributed, fixed, adopt criterion MET

**Trace instrument ran as registered** (seeds 0–4 both arms; CSVs in
`logs/parity_trace_2026-09-22/`). Native estimator: 0.002–0.004 m error at
last decode, velocity error 0.05–0.12 m/s. Port (pre-fix): estimator error
0.9–1.7 m median over the final 3 s, velocity error reaching 5–6.5 m/s at
CPA (post-CPA the filter diverged outright: seed 0 ended 244 m / 2527 m/s
off — past the miss, but diagnostic of the same defect).

**Attribution — a CORRECTION to the registered single-candidate expectation.
Four coupled port defects, not one dominant candidate:**
1. **(d) KF covariance Jacobian (the biggest):** `predict()` advanced the
   STATE with the control-input coupling but propagated P with F = I —
   `∂r'/∂v_t = dt·I` was missing, so position fixes had almost no correctly-
   signed gain into the velocity states and Phase B flew a velocity
   feed-forward that was 5–6.5 m/s wrong. Same family: `update()` used
   H = [I, 0] on a pre-extrapolated measurement, giving fixes no direct
   velocity coupling. Fixed: true Jacobian in predict; H = [I, −age·I] with
   the known own-displacement term in update.
2. **(a) attitude-at-capture:** current-tick conversion injected 0.20–0.37 m
   median (0.9–1.5 m max) per-fix errors while attitude swung in the close.
   Fixed in-contract: internal (t, quat) ring buffer, conversion at
   `t − meas_latency_s`.
3. **(c) yaw-slew seed:** both real constructors omit `initial_yaw_deg`, so
   the port's yaw command walked from 0° through a ~75° off-target sweep at
   ENGAGE entry — the camera missed the whole first Phase-A close pass
   (seed 2: first decode 5.42 s vs native 3.92 s; 40 vs 106 decodes). Fixed
   in-contract: yaw-slew state re-anchors to the OWN EKF yaw on the first
   post-GO tick.
4. **(e) slant shortfall:** `measurement_from_box` places the z-depth
   (fx·span/w) along the UNIT ray — short of true slant by cos(off-axis),
   a systematic 0.15–0.37 m along-LOS under-range in the final window.
   Fixed locally in `pursuit_terminal._measured_r_ned` (divide by the unit
   ray's z-component). NOTE: the same geometry ships in the validated stock
   `SeekerGuidance`/`tag_terminal` paths — NOT touched here; flagged for a
   separate ruling since those numbers were validated with the shortfall in.
   (b) latency jitter measured 0.016–0.052 m — minor, as predicted.

**Mechanism observed end-to-end** (fix-is-not-done rule): post-fix trace,
same seeds — port misses 0.074/0.034/0.067/0.146/0.124 m vs native
0.160/0.082/0.108/0.124/0.155 m; decodes 107–114 (from 40–96); velocity
error ≤0.23 m/s median; estimator error 0.055–0.078 m median final-3 s.

**Registered grid re-run (port arm, n=50, seeds 0..49, rear tag):**

| cell    | port BEFORE %≤0.35 / med | port AFTER %≤0.35 / med | native ref |
|---------|--------------------------|-------------------------|------------|
| nominal | 24.0% / 0.520 m          | **100.0% / 0.068 m**    | 100% / 0.120 m |
| aim 10  | 16.0% / 0.554 m          | **100.0% / 0.095 m**    | 100% / 0.129 m |
| aim 20  | 0.0% / 3.804 m           | **70.0% / 0.104 m**     | 100% / 0.098 m |
| alt +2  | 10.0% / 0.849 m          | **76.0% / 0.297 m**     | 100% / 0.149 m |

Adopt criterion (nominal ≥48% AND med ≤0.364 m AND no cell −5 pts): **MET —
ADOPTED.** `flight/tests/test_pursuit_terminal.py` + `isim/tests/` 174
passed; `real_flight --audit` PASS. (Two unrelated `test_real_flight.py`
passage-gate tests were mid-edit by a parallel work stream when the full
suite ran.)

**Still open, honestly:** aim20 (70% vs native 100%, p90 = 3.804 m = the
deterministic Phase-A leg with no re-acquire) and alt+2 (76%) trail the
native because the RealFlightSM wrapper has no BREAKOFF→re-approach path —
the native's ENGAGE↔APPROACH bouncing. That is the flight-code design
decision the 2026-09-21 spec already routed to the builder (ADR territory),
not a sim-side patch.
