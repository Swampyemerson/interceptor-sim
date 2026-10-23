---
name: pronav
description: Reference for the guidance laws in this project - chase pursuit (the FLYING DEFAULT, ADR-0103/0118), the retired sim-phase proportional navigation (pro-nav), and the tested-and-rejected alternatives (PIP, APN, Kalata). Use when building or tuning any guidance work (the pursuit terminal, or the frozen legacy sim paths).
---
# Guidance: pursuit and proportional navigation

**STATUS (2026-09-24, ADR-0118 — read this before anything below): the flying terminal law
is CHASE PURSUIT, not pro-nav.** ADR-0103 (builder decision, 2026-09-17) replaced the
sprint-and-fly-by pro-nav terminal with a slow-arrival chase; `flight/pursuit_terminal.py`
is the default (relative-state KF → Phase A belief rendezvous → Phase B camera close).
Pro-nav below is the SIM-PHASE law: it survives only in the frozen stock/tag paths
(selectable as-flown) and the M4 gate. Do not plan new work on pro-nav without a new
ruling — the 2026-10-22 hybrid/PIP-aim study (pre-registered) came back NULL: chase won
every informative cell (`docs/hybrid_guidance_notes_2026-10-22.md`).

## Pursuit (baseline)
Aim the interceptor's velocity straight at the target's current position. Simple, but lags a moving target because it always chases where the target *was*.

## Proportional navigation (pro-nav)
Command lateral acceleration proportional to the line-of-sight (LOS) rotation rate:
  a_cmd = N * Vc * lambda_dot
- `lambda_dot` = how fast the bearing to the target is rotating (LOS rate).
- `Vc` = closing speed.
- `N` = navigation constant. This project's validated value: **N=5** for the FPV profile (N=4 for the original M4 gate) — from a lab sweep confirmed in Gazebo (ADR-0011).
Intuition: if the bearing is not rotating you are already on a collision course; if it is, turn to null the rotation. This anticipates the target, so miss distance is far smaller than pursuit against a mover.

## The validated mechanization (do not re-derive from scratch — it's built)
`scripts/m4_intercept.py` is the source of truth; ADR-0009 (+2 addenda) and ADR-0013 hold the rationale:
- Strapdown LOS: λ = ψ (own EKF yaw) + β (camera bearing) — raw d(bearing)/dt is silently ~0 because the yaw loop nulls bearing.
- α-β filters on λ and range (λ rate gain 0.30, range 0.15); Vc = −filtered range-rate, floored.
- Command path: `set_velocity_ned` + absolute yaw setpoints; world→NED is north=world_y, east=world_x (ADR-0013).
- Terminal: freeze the commanded vector at close range (λ̇ is singular as R→0); camera-only breakoff.
- Sanity check any change with `--bench` (spin vs static tag → λ̇≈0) BEFORE flights.

## Negative results (tested, documented — do not re-propose without new evidence)
- **PIP (predicted intercept point)** wins in the clean lab but LOSES camera-only in Gazebo (noisy monocular track starves its velocity estimate; ADR-0011 addendum). It re-earns its place only with the S2 cue's clean mid-course track (dash phase uses PIP lead).
- **APN** (augmented PN): rejected — needs a 2nd derivative of an already-noisy signal; doesn't reliably beat plain PN even in the lab (ADR-0010, ADR-0011).
- **Kalata-index filter gains**: big lab win, WORSE in real flights (bimodal correction cadence → degenerate gains; ADR-0013). Kept as `--kalata`, default OFF.
- Standing rule: **the lab (guidance_lab.py) RANKS, Gazebo DECIDES** — six documented divergences.

## The kinematic reality (ADR-0023 — read before "fixing" the terminal phase)
At fast-crosser speeds the miss is ~96% determined by the zero-effort-miss (ZEM) at handoff; correction capacity ≈ ½·a·t_go². The levers that matter: acquisition range (t_go²), mid-course track quality (delivered ZEM), mechanization reclaim. NOT: camera hold through CPA, FOV, frame rate, yaw authority.

## In this sim
- Get bearing/LOS from the AprilTag detection (tag pose + camera intrinsics -> relative position -> LOS angle).
- Run pursuit and pro-nav on the SAME target paths (paired seeds — terminal-dropout noise is ~1 m/run), log both miss distances, and compare. That comparison is the resume evidence.
- Mind units and frames (ENU / OpenCV / FRD / NED - see docs/goals.md, ADR-0013).
