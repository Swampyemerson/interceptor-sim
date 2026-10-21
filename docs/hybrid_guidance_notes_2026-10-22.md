# Hybrid guidance (predictive midcourse / chase terminal) — mechanism notes

2026-10-22, head-inline research (subagent lane bounced on this topic; see
docs/subagent_safeguard_log.md). Context: builder wants to know if a
pro-nav/chase hybrid can beat the validated pure chase; the resolution
study (runs/res_guidance_20261021) showed the PIP law's failure is
POINTING (it keeps the target out of frame — fewer decodes at every
resolution), not bearing noise. This memo is the "what mechanism would
actually fix that" answer.

## What the literature says the failure is called

Our PIP law's problem is the classic **look-angle (seeker FOV) constraint**
problem: an aggressive geometry-shaping law commands headings whose look
angle (velocity-to-LOS angle) exceeds the camera FOV, and the seeker goes
blind mid-homing. A whole literature exists on guidance laws with the FOV
as an explicit constraint — the standard fixes:

1. **Switching-boundary composite:** fly PN while the look angle is inside
   a boundary; when it approaches the FOV edge, switch to a law that steers
   the look angle back (sliding-mode / Lyapunov steer-back), then resume.
   ([KAIST impact-angle-with-FOV line](https://dspace.kaist.ac.kr/handle/10203/179610))
2. **Biased PN:** add a bias term to the PN command, scheduled on look
   angle, so the trajectory bends without ever pushing the target to the
   frame edge; variants exist for passive/bearing-only seekers (no range).
   ([BIT time-varying biased PN with FOV](https://pure.bit.edu.cn/en/publications/time-varying-biased-proportional-guidance-with-seekers-field-of-v/))
3. **Three-phase optimal:** drive TO the max allowable look angle, HOLD it
   constant through midcourse, release into the terminal — i.e. the
   midcourse is defined by "as much geometry shaping as the FOV allows,
   never more." (Park/Kim/Tahk 2013.)

Quad-on-quad practice agrees: the working vision-intercept systems are
PN-CLASS TERMINALS with the camera constraint handled explicitly — a 2024
IBVS+PN composite reports 0.089 m CEP (sim) and >80% real-world intercepts
with a DELAYED Kalman filter in the loop; a 2026 planar-sector LOS law
relaxes the FOV cone and shows 138 m outdoor intercepts.
([IBVS+PN 2024 TIE via search], [Planar-Sector LOS](https://arxiv.org/pdf/2606.10639),
[ITU visual drone interception](https://polen.itu.edu.tr/items/e38d4b69-afaf-464e-a4c2-1422002c5021))

## Candidates for THIS project

A. **Keep pure chase (live option, the incumbent).** CLAIM: at our closing
   speeds and short engagements, shaping buys little. EVIDENCE: chase wins
   every isim comparison to date; hybrids pay mainly at high closing
   speed/long range. TRADEOFF: chase's known cost is terminal closing-speed
   geometry (the accel-cap story) — the thing a hybrid could improve.
B. **Look-angle-clamped biased PN terminal** (mechanism 2): PN on the KF
   LOS rate + a bias that zeroes as look angle nears a clamp (e.g. 0.6×
   HFOV/2). Bearing-only compatible. CLAIM: keeps chase's pointing virtue,
   adds geometry shaping where it is free. This is the literature's direct
   fix for exactly our PIP failure mode.
C. **Phase-A shaping only** (mechanism 3 lite): keep the chase terminal
   untouched; make the EXISTING Phase-A belief rendezvous fly a
   constant-look-angle approach instead of direct-to-point (our
   approach_dash already aims at a rendezvous — adding the hold-look-angle
   schedule shapes arrival geometry without touching the validated
   terminal). Lowest-risk hybrid.

## Pre-registered discriminating experiment (isim — lab ranks, Gazebo decides)

Implement B and C as isim concepts (NOT flight code), then: speeds {9, 12},
res scales {1, 2} (same-FOV), n=8, canonical + far geometries, vs the
pursuit terminal. Pre-register: adopt-for-Gazebo iff a candidate beats
pursuit's median miss at BOTH speeds at s=1 with ≥ equal decode counts
(the pointing metric — the thing PIP failed). A null = chase stays, and
the pro-nav answer to the builder is "the literature's fix exists but does
not pay at our scale" with the numbers to show it.

Status: experiment NOT yet run — queued as the next isim build.
