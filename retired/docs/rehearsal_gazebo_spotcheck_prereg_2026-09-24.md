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

## RESULT (2026-09-24, six scored flights — scored by the head session)

Flight 1 aborted PRE-GO (cold-boot health-wait `standby_timeout`, zero frames
consumed — an infra abort of the first boot in the batch, not mode behaviour);
replacement flight 7 flown under this registration, disclosed here. The six SCORED
flights (f2–f7):

- **R1 GREEN 6/6:** every flight triggered and ended `rehearsal_breakoff`; CPAs
  2.286 · 2.395 · 2.479 · 2.488 · 2.516 · 2.719 m — no flight anywhere near the
  0.35 m contact envelope.
- **R2 GREEN 6/6:** minimum true separation ≥ 2.286 m against the 0.7 m bar —
  wider than isim's 1.59–1.73 m prediction (the Gazebo evade turns away earlier).
- **R3 GREEN:** zero engagement aborts, `FAULT own_vel_fallback` 0/6, practice-
  profile startup line 6/6.

**Verdict: the rehearsal mode is Gazebo-consistent.** Field practice passes can be
planned — still starting gentle: the margins are sim-derived until real hardware
data exists, and the field would-have claim remains ULogs + video (P2 of the isim
round). Logs: `logs/rehearsal_gz_20260924/`.
