# Spec v5: try to BREAK the pursuit result before anyone believes it

State: after a head fix (delayed-measurement Kalman update in `_ConstVelKF.update`, `age_s`),
the grid is 92-98% inside 0.35 m nominal (see docs/decisions.md ADR-0103). That is a
simulator number with PERFECT own-state. Your job this round is adversarial: add the errors a
real vehicle has and report how fast the result degrades. Do not tune to rescue it.

## Add to `Scatter` (each with a unit comment and "estimate" source note; all default ON when
## scatter is on; each individually switchable to zero so its cost can be isolated)
1. Own-attitude error as seen by GUIDANCE (not by physics): slowly varying roll/pitch bias,
   sigma 1.0 deg, plus yaw bias sigma 3 deg, plus white noise 0.3 deg. Implement as a wrapper
   that perturbs the `VehicleState` handed to guidance only (the engine must keep the true
   state for physics and scoring). Put the wrapper in `isim/ownstate.py`.
2. Own-velocity error seen by guidance: bias sigma 0.15 m/s per axis + white 0.1 m/s.
3. Own-position error seen by guidance: slow drift (random walk, 0.05 m/sqrt(s)) -- should
   mostly cancel because estimate and control share the frame; verify that claim.
4. Frame timestamp error: guidance sees `t_capture` with a constant bias per run, uniform
   +/- 20 ms, plus jitter sigma 5 ms (the true capture time is unchanged).
5. Tag decode realism: `p_max` 0.98 -> draw per run uniform 0.6-0.98; pixel noise 0.3 ->
   uniform 0.3-1.0 px.
6. Target that does not fly straight: `WeaveTarget` (amp 1-3 m, period 4-8 s, drawn per run)
   and a target that changes speed by +/- 2 m/s at a random time. Scenario field
   `target_motion: "straight" | "weave" | "speed_change"`.

## Measure (n = 100 per cell, both tag facings, fx 933, tilt 12)
- Nominal, aim 30 deg, alt -2 / +2 m, target speed 4 and 9, for: baseline scatter; each new
  error source alone; all together; all together at 2x the sigmas.
- Weave and speed-change targets with all errors on.
- A table of % inside 0.35 m and median miss per cell, plus which error source costs most.

## Also check (short)
- The honesty boundary still holds: guidance receives only the (perturbed) own state and
  detections. Keep the spy tests; add one proving the perturbed state, not the true state,
  is what guidance sees, and that scoring uses the true state.
- `concept="flyby"` must stay bit-identical when the new scatter terms are zero.

Rules as before: loose test bounds, print measured values, no tuning to flatter, say what you
doubt. Parameters must travel through `Scenario`/`Scatter` fields (spawned workers do not
see monkeypatches). Verify with `.venv/bin/python -m pytest isim/tests -q`.
