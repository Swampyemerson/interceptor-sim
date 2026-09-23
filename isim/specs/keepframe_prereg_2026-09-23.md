# Keep-in-frame vertical assist — pre-registration (2026-09-23)

Registered BEFORE any A/B run was launched (standing rule: pre-register
before you fly). Feature: `flight.pursuit_terminal` keep-in-frame vertical
assist (`PursuitTerminalConfig.keepframe_assist`, DEFAULT OFF; margin
`keepframe_margin_frac=0.35` of the half-frame height, max added climb
`keepframe_vz_max_ms=2.5` m/s, dropout hold `keepframe_hold_s=0.5` s).

## What it attacks

The chase concept's altitude-error cliff on the HIGH side
(`isim/specs/alt_sensitivity_2026-09-21.md`, native prototype, scatter on,
n=50/point): 74–88% inside 0.35 m from −2..+2 m of target-altitude error,
**42% at +3 m above**. Hypothesized mechanism: a target well above sits near
the TOP edge of the frame; decodes go sparse/lost faster than the
position-driven vertical correction converges, and Phase A/acquisition has no
camera feedback until decodes arrive. The assist reads ONLY the box pixel
position (top-edge margin) plus the capture-instant own-attitude/mount-tilt
NED conversion (target-above gate) — camera + own-state only, no ground
truth — and adds a bounded climb term before the norm clip and slew.

NOTE ON THE BASELINE: the 42% cliff was measured on the NATIVE prototype
(`concept="pursuit"`). This A/B flies the PORT (`concept="flyby"`,
`terminal="pursuit"` — the real flight code), so the OFF arm re-measures the
cliff in the port before the ON arm is read against it. If the port's OFF arm
shows no +3 m cliff at all, the comparison is still valid but the finding is
"the port does not reproduce the native cliff" and is reported as such.

## Config (exact)

- Arm mechanics copied from `scripts/adaptive_speed_ab.py`; script:
  `scripts/keepframe_ab.py`.
- Base scenario: `Scenario(concept="flyby", terminal="pursuit",
  tag_facing="rear", scatter=Scatter(), target_alt_offset_m=<cell>,
  aim_error_deg=<cell>)` — all realistic errors ON via `Scatter()` defaults.
  (Scatter's v5 own-state-noise block wraps only the native concepts, so the
  port arm sees every scatter term EXCEPT own-state noise; identical for ON
  and OFF, so the pairing is fair.)
- Cells: `target_alt_offset_m` ∈ {−2, 0, +1, +2, +3, +4} m ×
  `aim_error_deg` ∈ {0, 20}. (+4 m extends past the measured cliff;
  descriptive only, not part of the criterion.)
- Arms: OFF = `port_pursuit_overrides=None` (defaults);
  ON = `port_pursuit_overrides={"keepframe_assist": True}`.
- n = 50 paired seeds per cell (seeds 0..49, identical across arms; scatter
  draws are seed-keyed, so pairs share their world).
- Metric: % of runs with `miss_m <= 0.35` (the binary-kill radius), plus
  median/p90 miss.
- Mechanism metric (per run, logged so any win is attributable to holding
  lock, not luck): `n_decoded` and the decode fraction
  `n_decoded / n_frames`; medians reported per cell.

## Predictions (registered)

1. **+3 m, aim 0°:** ON improves the %≤0.35 m rate by **≥ 15 points** over
   OFF.
2. **0 m cells (both aim values):** ON changes the %≤0.35 m rate by **≤ 2
   points** in either direction (no regression on the nominal geometry).
3. Mechanism: where ON wins, its median decode count / decode fraction at
   that cell exceeds OFF's (the win must come with more lock, or it is not
   this mechanism).

## Adopt / reject criterion (registered)

- **ADOPT-recommend** (recommend enabling for elevated-target engagements;
  the flag stays default-off in code either way) iff prediction 1 AND
  prediction 2 both hold, and the mechanism metric moves the right way.
- **MIXED** if the +3 m cell improves ≥ 15 points but a 0 m cell moves > 2
  points (report both; no recommendation without a follow-up).
- **NULL** if the +3 m cell does NOT improve ≥ 15 points.

## What a NULL means (registered)

If the +3 m cell does not improve, the cliff is NOT (primarily) a
frame-top-exit mechanism reachable by a camera-cued climb — candidate
alternatives: climb authority/slew during the close, or the Phase-A belief
altitude error itself (no decodes ever arrive to trigger the assist). The
assist then stays OFF and the mechanism hunt moves to the per-tick trace,
not to re-tuning this feature's gains.

---

## RESULTS (appended after the run, 2026-09-23; `scripts/keepframe_ab.py`, n=50 paired seeds/cell)

med = median miss (m), p90 = 90th-pct miss (m), %≤0.35 = binary-kill rate,
med_dec = median decode count, decfrac = median decode fraction (n_decoded/n_frames).

| cell         | arm | med   | p90   | ≤0.35m | ≤1.0m | med_dec | decfrac |
|--------------|-----|-------|-------|-------:|------:|--------:|--------:|
| alt−2 aim0   | OFF | 0.207 | 0.549 |  78%   |  92%  |   82    | 0.086 |
| alt−2 aim0   | ON  | 0.207 | 0.549 |  78%   |  92%  |   82    | 0.086 |
| alt+0 aim0   | OFF | 0.130 | 0.252 |  94%   |  96%  |   94    | 0.099 |
| alt+0 aim0   | ON  | 0.138 | 0.279 |  94%   |  96%  |   94    | 0.098 |
| alt+1 aim0   | OFF | 0.147 | 0.372 |  88%   |  96%  |   86    | 0.091 |
| alt+1 aim0   | ON  | 0.185 | 0.394 |  86%   |  96%  |   94    | 0.099 |
| alt+2 aim0   | OFF | 0.201 | 0.983 |  72%   |  90%  |   66    | 0.069 |
| alt+2 aim0   | ON  | 0.213 | 0.959 |  72%   |  90%  |   70    | 0.073 |
| alt+3 aim0   | OFF | 0.420 | 4.180 |  34%   |  74%  |   33    | 0.035 |
| alt+3 aim0   | ON  | 0.460 | 4.180 |  38%   |  78%  |   34    | 0.036 |
| alt+4 aim0   | OFF | 0.497 | 4.902 |  26%   |  64%  |   34    | 0.036 |
| alt+4 aim0   | ON  | 0.536 | 4.902 |  16%   |  64%  |   30    | 0.031 |
| alt−2 aim20  | OFF | 0.221 | 4.827 |  68%   |  78%  |   72    | 0.076 |
| alt−2 aim20  | ON  | 0.221 | 4.827 |  68%   |  78%  |   72    | 0.076 |
| alt+0 aim20  | OFF | 0.149 | 4.341 |  80%   |  82%  |   79    | 0.083 |
| alt+0 aim20  | ON  | 0.171 | 4.341 |  78%   |  82%  |   78    | 0.081 |
| alt+1 aim20  | OFF | 0.158 | 4.432 |  66%   |  72%  |   68    | 0.071 |
| alt+1 aim20  | ON  | 0.191 | 4.432 |  66%   |  74%  |   65    | 0.068 |
| alt+2 aim20  | OFF | 0.615 | 4.737 |  38%   |  52%  |   22    | 0.023 |
| alt+2 aim20  | ON  | 0.650 | 4.671 |  36%   |  54%  |   26    | 0.027 |
| alt+3 aim20  | OFF | 1.613 | 5.364 |  24%   |  48%  |   13    | 0.014 |
| alt+3 aim20  | ON  | 1.629 | 5.364 |  30%   |  48%  |   16    | 0.017 |
| alt+4 aim20  | OFF | 3.996 | 5.978 |  32%   |  44%  |    6    | 0.006 |
| alt+4 aim20  | ON  | 1.439 | 5.978 |  24%   |  46%  |    7    | 0.007 |

### Read against the registered criteria

- **Prediction 1 — FAILED.** +3 m, aim 0°: OFF 34% → ON 38% = **+4 points**,
  far short of the registered ≥15. (+3 m, aim 20°: +6 points — same story.)
- **Prediction 2 — held.** 0 m cells: aim0 94→94 (0), aim20 80→78 (−2 pt,
  exactly at the registered bound). No meaningful nominal regression.
- **Prediction 3 (mechanism) — the tell.** At +3 m aim0 the median decode
  count moved 33 → 34 (decfrac 0.035 → 0.036): the assist bought essentially
  **no additional lock**. The OFF arm's decode count already collapses with
  altitude error (94 at 0 m → 33 at +3 m → 34 at +4 m): by the time the
  target is 3 m high, most of the decode loss happens where the assist cannot
  act (few/no fresh top-edge decodes to trigger on — the assist is
  decode-TRIGGERED, and the starved cells starve it too).
- The port's OFF arm DOES reproduce the native cliff (72% at +2 m → 34% at
  +3 m; native was 76% → 42%), so the null is not "no cliff to fix".
- Descriptive only (registered as such): at +4 m the ON arm is WORSE on hit
  rate (26→16 aim0, 32→24 aim20; n=50 noise scale is ~±10 points, so
  direction uncertain, but there is certainly no win past the cliff either).

### VERDICT: **NULL**, per the registered criterion.

The registered meaning applies: the +3 m cliff is NOT (primarily) a
frame-top-exit mechanism reachable by a camera-cued climb. The mechanism
metric says why: decodes are already gone at those cells before the assist
can trigger — the loss looks acquisition-side (Phase A flies the WRONG
believed altitude and the tag rarely enters frame at all), not
hold-the-lock-side. **`keepframe_assist` stays DEFAULT OFF and is NOT
recommended for adoption.** The code + tests remain in the tree as a
config-gated, byte-identity-locked lever (cost: none while off), but the
next attack on the cliff should target the Phase-A belief-altitude error /
acquisition geometry via the per-tick trace, not this trigger's tuning.

---

## +3 m acquisition attribution (diagnosis follow-up, 2026-09-23)

DIAGNOSIS ONLY — ground truth (engine trace + FrameReport + each run's own
scattered camera/decode params) is used here to attribute frames after the
fact; none of it feeds guidance. Per-frame attribution over the APPROACH
WINDOW (ENGAGE entry → first decode, or run end if none), +3 m vs +1 m
aim-0 OFF arms, 20 seeds each, every no-decode camera frame assigned to
exactly one cause (frame-out buckets from the true target projection
through the run's true camera incl. tilt error and current attitude;
in-frame buckets from the seeker's own decode model, dominant suppressor):

| cause (frame share of window)     | +3 m (4384 fr) | +1 m (1498 fr) |
|-----------------------------------|---------------:|---------------:|
| out of frame — TOP (vertical)     |     **40.8%**  |     15.2%      |
| in frame — too small (<8 px eff)  |     **34.8%**  |      2.1%      |
| in frame — tag backside (i≥90°)   |     15.2%      |   **63.0%**    |
| in frame — small size (stochastic)|      5.3%      |      1.3%      |
| out of frame — both axes          |      2.2%      |      6.9%      |
| in frame — corner clipped         |      0.8%      |      2.3%      |
| in frame — oblique incidence      |      0.8%      |      4.5%      |
| out of frame — horizontal only    |      0.0%      |      2.3%      |
| behind camera / other             |      0.0%      |      2.2%      |

Context rows: runs with ≥1 decode after ENGAGE: 18/20 (+3 m) vs 20/20
(+1 m); median approach-window length 3.9 s vs 1.8 s. **Vertical gap
through Phase A (+ = target above): +3 m cell median +3.04 m at ENGAGE →
+3.00 m at window end (min |gap| 2.88 m)** — Phase A NEVER closes the
vertical error; it station-keeps at the believed (wrong) altitude
indefinitely, exactly as designed, because the belief carries zero
altitude error and nothing else commands vertical motion before decodes
arrive. (+1 m cell: +1.04 → +0.90 m — it doesn't close it either, but 1 m
sits inside the vertical FoV/decode envelope so it doesn't need to.)

READ: the +3 m acquisition failure is (1) frame-top exit, 41% — but during
NO-DECODE frames, which is why the decode-triggered keepframe assist
couldn't reach it — and (2) too-small/range, 35%, the longer slant range
the unclosed 3 m offset forces. Both trace to the same root: Phase A holds
the believed altitude with no vertical strategy. The candidate lever
(proposal only, nothing implemented): a bounded **Phase-A vertical
bracket/search sweep** about the believed target altitude, biased UP since
the measured cliff is one-sided — it would attack both dominant buckets at
once. **HISTORY GATE (found on the mandatory check before building): this
lever has a MEASURED NEGATIVE on record.** The native concept's v4 #2/#3
Phase-A vertical+yaw search (`isim/concepts.py`, `vsearch_*` config block;
n=100 × 2 facings × 5 cases) made the rear-tag **alt+3m cell WORSE, 43% →
32%**, plus nominal 71→67, alt−2m 74→68, aim20° 54→40 — "perturbing a
Phase-A trajectory that was about to succeed anyway costs more often than
a genuinely-stuck trajectory gets rescued" — and is default-off (both
amplitudes 0.0) as a measured verdict, not a mere non-adoption. Caveats
that could justify a REGISTERED re-test (a decision above this doc's pay
grade): that measurement predates the v7 lens correction (fx 933-era vs
today's fx 385 / 118° HFOV — a different vertical-FoV regime; "a threshold
validated at one operating point is not validated at another" cuts both
ways), the tested design differed (arrival-triggered, symmetric, coupled
to a yaw sweep — not a continuous biased-up bracket), and this attribution
is new mechanism evidence the v4 test did not have. Secondary options
without that history: a deliberate upward standby-altitude bias, and
`cam_tilt_up_deg` (raises the vertical FoV ceiling; interacts with the
existing tilt history). `d_behind_m` alone is weaker: it lowers the target
in frame but lengthens range, feeding the too-small bucket.
