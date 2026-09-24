# Brake shaping on the honest ENGAGE plant — pre-registration (2026-09-24, overnight)

Registered BEFORE any run (standing rule). The next unit of work named by the contract
after the 2026-09-23 Gazebo re-fly ("Registered isim guidance experiments vs the honest
ENGAGE fit — braking/approach shaping").

## The attributed residual, made quantitative

The re-fly closed the range-bias and cadence suspects (instruments green) but failed the
CPA bar (median 0.967 m vs the 0.09–0.31 m honest-plant band); the registered residual
branch is "the pursuit law on a slow plant (hot approach / late braking)". The ENGAGE fit
(`isim/fits/vehicle_gazebo_x500_engage.json`) makes that concrete:

- fitted braking authority `max_setpoint_accel_horiz` = **5.94 m/s²** (dash fit: 10.4),
  velocity-loop kp = 3.34 (≈ 0.30 s time constant; dash: 5.67), latency 0.14 s.
- Stopping distance from a 16 m/s approach ≈ v²/2a + v·(latency + 1/kp) ≈ 21.5 + 7 ≈
  **28 m**. Phase B's closing schedule (`v_close = clamp(0.6·range, 1.5, 6)`) only starts
  tapering at 10 m and the total command is otherwise capped at `v_max` — the command asks
  the plant for a deceleration profile it cannot fly, so the vehicle arrives hot and the
  miss is un-nulled cross-track at CPA (tick-trace S-finding, med 1.41 m).

## The lever (config-gated, default OFF, byte-identity pinned)

`PursuitConfig.brake_shaping: bool = False` + `brake_accel_ms2: float = 4.0` +
`brake_lead_s: float = 0.45` (≈ latency + 1/kp of the honest plant; all three
TODO-BUILDER estimate-graded — the real airframe's first braking ULog replaces them).
When ON, in Phase A (inside `2×` the stopping envelope of the rendezvous point) and
Phase B, the RELATIVE commanded speed is capped by physical stopping distance with lead:

```
d_eff   = max(range_est − brake_lead_s · v_rel_closing, 0)
v_rel_cap = sqrt(v_close_min² + 2 · brake_accel_ms2 · d_eff)
|cmd − v_t| ≤ v_rel_cap        (direction preserved; v_t = believed/KF target velocity)
```

This is the classic proportional-navigation-era "brake to the basket" envelope: the
command can never demand a relative speed the configured deceleration cannot shed before
the remaining range runs out. It does NOT read lock quality (the ADR-0108 timidity
rejection is about staleness-modulated closure, not range physics — stated to avoid
resurrecting that graveyard entry by accident).

## Arms & cells

- Arms: baseline (OFF) · a=3 · a=4 · a=5 (all lead 0.45) · a=4/lead 0 (isolates the lead
  term). Sweep script: `scripts/brake_shaping_ab.py`.
- Plants: **ENGAGE fit (honest, primary)** and **dash fit (no-regression control)** —
  the sweep script loads each fit explicitly and runs `mc._run_one` under its own pool.
- Cells (port arm, `concept="flyby", terminal="pursuit", tag_facing="rear",
  scatter=Scatter()`): nominal · aim20 · alt+3 with the ADR-0114 pair
  (cam_tilt_up_deg=10, tag_mount_pitch_deg=12) · weave. Realism rungs R0 and R2
  (tag_realism_v1 §F definitions — experiments must not be tuned to the upright-tag
  optimism). n = 50 paired seeds (0..49) per cell-arm-plant-rung.

## Predictions (registered)

1. **Honest plant, nominal, R2:** at least one braking arm improves contact ≤0.35 m by
   **≥ 15 points** over baseline, via smaller CPA overshoot (median miss down, and the
   mechanism check: median |own speed at 3 m range| drops toward the schedule).
2. **Dash plant (control):** every arm within ±5 points of baseline on nominal — the cap
   should be inactive on a plant that can fly the schedule. If the cap CHANGES the fast
   plant materially, the lever is mis-sized, not a win.
3. a=5 risks under-braking (above the fitted 5.94 with no margin), a=3 risks timidity
   (longer exposure, more decode time — could go either way); a=4 is the predicted
   adopt point.
4. **Null branch:** if no arm clears #1, the hot approach is NOT the residual's main
   term and the registered "outside the five suspects" branch takes over — do NOT tune
   further constants; write the null.

## Adopt / reject (registered)

Adopt the best arm iff prediction #1 holds AND #2 holds AND no cell on either plant
degrades > 5 points. Adoption = default-OFF config recommended for the re-fly (the
default swap to hardware stays blocked on the Gazebo re-fly as always); then re-predict
the Gazebo band and pre-register re-fly #3 before flying it.

## RESULT (flown 2026-09-24, verbatim log: logs/brake_shaping_20260924/sweep.txt — full
tables also in the worker report; adjudicated against the registered rules)

- **Prediction #1 PASS:** honest plant, nominal, R2: base 64% → a3 94% (+30), a4 82%
  (+18); mechanism confirmed (median closing speed at CPA 2.91 → 1.67 m/s; median miss
  0.254 → 0.110 m). R0 agrees (62% → 98%).
- **Prediction #2 PASS for a3/a4** (dash nominal within ±5); a5 and a4_lead0 breach the
  band upward on R0.
- **Prediction #3 WRONG:** a3, not a4, is the best arm everywhere on the honest plant.
- **Side findings:** aim20 improves on BOTH plants for every arm; honest-plant weave
  ≤1.0 m goes 18–20% → up to 62%.
- **ADOPT RULE FAILS for every arm (as registered):** the dash-plant alt+3 pair cell
  degrades 12–24 points (70/64% → 46–54%), p90 0.8–1.0 → 2.2–3.5 m, and closing speed
  at CPA RISES there; honest-plant alt+3 also grows a p90 tail for a3/a4/a5.
  **NOTHING IS ADOPTED under the registered criterion.**

## AMENDMENT #1 (registered 2026-09-24, post-hoc-MOTIVATED by the alt+3 failure —
labelled as such; new prediction registered BEFORE the amendment arm flies)

**Mechanism read (from the tables + the code, not per-run traces):** `_brake_cap` clips
the full 3-D relative vector, so a climbing approach (alt+3) has its CLIMB scaled down
whenever horizontal closure demands braking — the vehicle arrives low, decodes drop
(dash alt+3 med_dec 60 → 52), and the passage window ends the chase with the gap
un-closed (closing speed at CPA rises because the vehicle is still chasing vertically).
The cap's physics (braking authority, tilt, drag) are HORIZONTAL; the vertical channel
has separate budgets and, at alt+3, needs sustained climb.

**Amendment:** `brake_horizontal_only: bool = True` (only meaningful with
`brake_shaping`): compute closing/range on the HORIZONTAL components and cap only the
horizontal part of `(cmd − v_t)`; the vertical command passes through untouched.

**Registered predictions:** (A1) dash-plant alt+3 returns within ±5 points of base at
a3/a4 horizontal; (A2) the honest-plant nominal/aim20/weave gains are retained within
5 points of the 3-D-cap arms; (A3) honest alt+3 p90 tail shrinks back toward base.
**Adopt (amended):** a3-horizontal iff A1 AND A2 AND the original no->5-point-regression
rule now holds on every measured cell, both plants, both rungs. Otherwise the lever is
recorded as honest-plant-only evidence and NOT recommended for the re-fly.
