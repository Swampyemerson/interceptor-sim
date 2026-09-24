# Commit-sprint in the terminal blind coast — registration (2026-09-24, builder idea)

Builder (2026-09-24, verbatim intent): *"would it be in our interest to build a system
that actually goes full throttle last few seconds in that coast phase?"* Registered
BEFORE the build. The blanket version (sprint while the camera can still correct) is
contraindicated by ADR-0115/0116 measurements (correction capacity ∝ t_go²); the
version with a real mechanism is scoped to the window where corrections are already
impossible.

## The lever (config-gated, default OFF)

`PursuitTerminalConfig.commit_sprint: bool = False`. Scope: ONLY once the terminal
COAST latch has fired (Phase B, r_hat below the coast range — steering is already
frozen and the vehicle holds `_prev_v_cmd`). When ON, the held command keeps its
DIRECTION exactly (no steering change — the latch's contract is untouched) but its
horizontal magnitude is raised to `v_max_ms`. Vertical component unchanged. The
rehearsal trigger and all failsafes see the same state they would otherwise.

Mechanism claimed: inside the latch window (~0.2–0.4 s post-ADR-0117) no correction
can land regardless, so added speed there costs no correction authority; it shortens
the window in which target lateral motion (weave) accrues miss, and raises closing
speed at the pass. Mechanism risks: the plant's lag means little speed is actually
gained in ~0.3 s; a wrong frozen direction is flown FASTER.

## Sweep (isim, port arm, ENGAGE fit + pose, rung R2, n=50 paired seeds/cell)

Cells: nominal · aim20 · alt+3 pair · **weave** (the cell the mechanism targets).
Arms: OFF · ON, each at two base configs: plain default and the brake package
(`--pursuit-brake` constants). Script: extend the brake A/B pattern.

## Registered predictions & adopt bar

- **P1 (the mechanism cell):** weave contact ≤0.35 m improves by ≥5 points with the
  sprint ON (either base config).
- **P2:** nominal/aim20/alt+3 within ±5 points (the window is too short to matter
  there either way).
- **P3 (mechanism check):** median closing speed at CPA RISES on ON arms.
- **Adopt** (default stays OFF; adoption = recommended-config note) iff some cell
  gains ≥10 points with no cell losing >5, else record the honest null — the plant's
  lag likely eats the window, and that is a finding about the airframe, not a failure
  of the idea.
