# Pursuit ("chase only") port — flight code, 2026-09-17

## Decision

Builder ruling (ADR-0103, superseding the hybrid ruling): **pursuit alone**, not the
sprint-and-fly-by hybrid. `isim/concepts.py:PursuitRendezvousGuidance` (measured 82%
inside 0.35 m nominal, all realistic errors on, fx 385, rear tag — ADR-0103 v7) is
validated only inside isim's own prototype, against isim's synthetic `Detection` type.
It has never run through the actual flight code (`flight/deploy/real_flight.py` +
`flight/guidance.py`) that will fly on hardware. This doc tracks porting it there.

## Design

**Reuse, don't re-derive, the box→NED geometry.** `flight/tag_terminal.py` already
solves "turn a detector pixel box into a 3-D relative position" for the existing
`TagInterceptGuidance` (PIP) terminal: `measurement_from_box` (undistorted bearing +
known-tag-size range), `_optical_vec_to_ned`, `_cam_offset_ned`, the fixed
`meas_latency_s` extrapolation. The port reuses these UNCHANGED rather than adding a
second geometry implementation.

**Do not reuse `_RelStateKF`'s tuning.** That filter is tuned for the PIP law's own
measurement-noise model. Pursuit's validated 82%/68% numbers depend on its OWN tuned
constants (`cross_sigma_floor_m=0.20`, `kf_q_accel_ms2=1.0`, the closing-speed schedule,
etc., swept and measured v2–v6, `isim/specs/pursuit_concept*.md`). The port keeps
Pursuit's own 6-state constant-velocity KF and its own measurement-noise formula
(`_measurement_r` in `isim/concepts.py`), translated to plain NumPy — same tuning,
different plumbing.

**Skip the default-off ablations.** `isim/concepts.py`'s v4–v6 iterations tried camera-
frame close-in steering, a KF timestamp-bias state, and Phase-A search sweeps — all
measured NEGATIVE or NOT adopted, and all default OFF (`cam_frame_range_m=0.0`,
`estimate_ts_bias=False`, `vsearch_amplitude_m=0.0`). The port implements only the
CORE, adopted-default path: Phase A belief rendezvous → acquire gate → Phase B KF
terminal with the NED-frame closing law → fallback to a fresh Phase A on a long
dropout → freeze inside `hold_range_m` (fly through).

**Latency:** the isim original interpolates an own-state ring buffer at each
detection's `t_capture`. The flight-code contract (`step(det_box, own, t)`) has no
per-detection capture timestamp — mirror `TagInterceptGuidance`'s existing, already-
shipped simplification instead: a fixed `meas_latency_s` constant, extrapolating the
measured position forward by `v_rel_hat * meas_latency_s`. Disclosed simplification,
not a silent one.

## Contract (duck-typed, same as `TagInterceptGuidance`)

```
step(det_box_xywh, own: OwnState, t: float) -> (Optional[Setpoint], StepTelemetry)
```

Drops into `RealFlightSM` unmodified (no `isinstance` anywhere on `RealFlightSM.guidance`).

## Work items

1. **`flight/pursuit_terminal.py`** (opus5-worker) — `PursuitTerminalConfig` +
   `PursuitTerminalGuidance`, ported per the design above. Unit tests in
   `flight/tests/test_pursuit_terminal.py`.
2. **`isim/flight_adapter.py`** (opus5-worker, same task) — `RealFlightGuidance`
   gains `terminal="pursuit"` alongside the existing `"stock"`/`"tag"`, building
   `PursuitTerminalGuidance` the same way `"tag"` builds `TagInterceptGuidance`.
3. **`flight/deploy/real_flight.py`** (head — touches flight code) —
   - A `pursuit_mode` flag on `MissionConfig` (or equivalent) that suppresses the
     past-CPA RECESSION breakoff trigger (`_recede_streak`/`breakoff_range_increases`)
     while KEEPING `breakoff_hard_floor_m` (contact — the 0.35 m ram radius, ADR-0084),
     `engage_lost_target_s`, and `engage_max_s` active. Pursuit does not fly by; a
     receding-range breakoff firing during Phase A's speed-matching approach would be
     a false abort, not a real recession.
   - `--terminal {stock,tag,pursuit}` on the CLI (`main()`), for parity with the isim
     adapter, plus threading `target_start`/`target_vel`/the GO instant into the
     pursuit guidance's Phase-A belief seed (the SAME pre-flight constant
     `collision_lead_heading` already uses — honesty-clean, no live read).
   - `flight/pursuit_terminal.py` added to `_AUDITED_MODULES` (the no-cheat AST audit).
4. **Validation** — `python -m isim.mc requirement --terminal pursuit --concept flyby
   --n <N>` (or the sweep equivalent) run through the REAL flight code, compared
   against `isim/concepts.py`'s native numbers (ADR-0103's grid) as an A0-style parity
   check. A gap here is itself the finding — the flight-code contract's fixed-latency
   simplification and the honesty-clean Phase-A belief (vs. isim's own internal
   plumbing) are both real differences, not bugs, but their size needs a number.
5. **Gazebo transfer check** — deferred; isim/flight-code parity comes first
   (`docs/new_sim_plan.md`'s own acceptance order: A0/A1 → sweep → THEN Gazebo).

## Status

- 2026-09-17: plan written, work items 1–2 dispatched to `opus5-worker`.
- 2026-09-17: items 1–2 landed (commit 1309d1b) plus the `pursuit_mode` half of item 3.
- 2026-09-21: item 3 COMPLETE (ADR-0105): `--terminal {stock,tag,pursuit}` on the real
  CLI via `build_terminal()`; in pursuit mode the GO edge enters ENGAGE directly (the
  acquire-streak gate is a sprint-era mechanism a chase can never satisfy); belief seed
  = `--target-start`/`--target-vel` → relative NED `[N, E, +dash_loft_m]`, `go_at_s=0`
  (the SM gates entry, which is what makes rc/gate triggers seedable). 7 directed tests
  (`flight/tests/test_terminal_cli.py`); audit + self-test PASS; full suite 1064 passed.
  Disclosed open interaction: `engage_lost_target_s=2.0` fires before pursuit's
  `fallback_s=3.0` recovery can act — needs a ruling before either number moves.
- Items 4 (isim parity) and 5 (Gazebo) remain open.
