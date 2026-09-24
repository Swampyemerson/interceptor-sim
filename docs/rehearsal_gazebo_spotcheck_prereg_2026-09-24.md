# Rehearsal-mode Gazebo spot-check — registration (2026-09-24, before flying)

The adopted rehearsal mode's margins (worst-case escape gap 1.59–1.73 m, zero
contacts) are isim numbers conditional on the practice profile. This spot-check asks
only whether the mode BEHAVES in the full-physics sim — it is a smoke-level rung
(n = 6), not a margin re-derivation.

## Configuration (registered)

`bash scripts/xcheck_fly.sh 1 6 <outdir> --pursuit-rehearsal` — the standard
cross-check scenario; the one flag applies the whole practice profile (rehearsal +
brake + v_max 10, ADR-0118-discipline decision in the rehearsal spec).

## Registered checks (all six flights)

- R1: the rehearsal trigger fires (the cross-check geometry is a would-pass: re-fly
  #5 put 6/8 inside 0.35 m), the mission ends `rehearsal_breakoff`, and NO flight's
  CPA is inside the 0.35 m contact envelope.
- R2: minimum true separation after the trigger ≥ **0.7 m** on every flight (the
  original safety bar; isim predicts 1.6+).
- R3: zero failsafe aborts; `FAULT own_vel_fallback` 0/6; the practice profile's
  startup line present 6/6.
- **Outcome meanings:** all green → the mode is Gazebo-consistent and field practice
  can be planned (still starting gentle; margins remain isim-derived numbers).
  R2 red → the isim margin does not transfer; the mode is NOT field-ready and the
  gap gets a trace. R1 red (no trigger / a contact) → trace before anything else.
