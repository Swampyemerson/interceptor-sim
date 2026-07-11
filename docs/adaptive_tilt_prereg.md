# Fixed-tilt + extended-handoff — pre-registration (does tilt TIGHTEN the miss?)

**Written 2026-07-10 eve, BEFORE the arms fly.** Builder pivot: try a FIXED
up-tilt (cheap, no gimbal/PX4-swap) before the adaptive gimbal (#46, built as the
fallback). This tests the ACTUAL builder goal — a TIGHTER intercept — which #40
never tested (#40 tested terminal PARITY, not whether the tilt makes the miss
BETTER).

## The mechanism under test (ADR-0069)

The ~2 m weave miss is ~84% a fast-crossing kinematic floor set by ACQUISITION
RANGE: the camera-only terminal starts at ~6–10 m handoff (t_go ~0.26–0.4 s) with
only ~0.3–0.7 m of correction capacity (½·a·t_go²), too little to null the ~1.6 m
zero-effort-miss it inherits. Capacity scales with **t_go²**, so handing off at
LONGER range collapses the miss. A fixed up-tilt keeps the horizon-level target in
frame during the nose-down dash (ADR-0067: dash-above-FoV 32%→0%, first-det +2–4 m)
→ reliable detection at LONGER range → an EARLIER HANDOFF at longer range → bigger
t_go → tighter CPA. The `--handoff-range` extension is the knob that converts the
tilt's earlier detection into the bigger t_go.

## Arms (weave 12 m/s, adopted deployment config, master-seed 42, PAIRED, n=8)

| Arm | Camera | m4 extra | Purpose |
|---|---|---|---|
| **A (control)** | level | (deployment default handoff ~10 m) | today's baseline |
| **A2** | level | `--handoff-range 15` | does extending handoff ALONE help (is level detection reliable at 15 m, or does it just phantom / never hand off)? |
| **B (treatment)** | up15 shadow | `--cam-mount-up-deg 15 --handoff-range 15` | the tilt makes detection reliable at 15 m → handoff at longer range → tighter miss? |

In-project (#40 camera shadow `models/mono_cam → up15`, UPTILT_EXPECTED=1; NO PX4
swap). Idle load, one sim at a time, `scripts/sim_kill.sh` by path.

## Pre-registered metrics + reads

- **PRIMARY — terminal CPA** (paired per-seed miss, B − A and A2 − A). Report the
  per-seed deltas + sign counts + median (the `uptilt_ab_analyze.py` convention).
- **MECHANISM — handoff range achieved** per arm (does B hand off materially
  LATER/longer than A? if not, the t_go lever didn't engage and a tighter CPA can't
  be attributed to it). Also first-det range + Pk@2.5.
- **DELIVERED ZEM at handoff** if computable (the ADR-0023 quantity).

## Pre-registered verdict

- **TILT TIGHTENS (adopt fixed tilt)** iff: B hands off at materially longer range
  than A (mechanism engaged) AND B's paired CPA is tighter than A (median delta
  negative, worse on ≤ 3/8) AND Pk@2.5 non-regressed. Then the SIMPLE fixed tilt is
  the answer — no adaptive gimbal needed (builder's hypothesis confirmed).
- **HANDOFF-RANGE ALONE** iff A2 already tightens ≈ as much as B → the tilt isn't
  even needed, just extend the handoff range (even simpler). (Unlikely — level
  detection at 15 m is expected to be unreliable/phantom-prone, ADR-0060.)
- **NULL / tilt doesn't convert** iff B hands off later but CPA doesn't tighten →
  the extra t_go was eaten by delivered-ZEM/terminal (re-scope to mid-course track
  quality) — OR B doesn't hand off later (detection at 15 m still unreliable even
  tilted) → the fixed angle/handoff combo needs tuning, and the adaptive gimbal
  (finer pointing) becomes the lever.
- n=8 is a SCREEN (the ~1 m noise floor); a clear win extends to n=16 before adopting.

Honest prior: the tilt's first-det gain is +2–4 m; whether that converts to a
meaningful t_go/CPA gain is genuinely open — this experiment answers it cheaply.
