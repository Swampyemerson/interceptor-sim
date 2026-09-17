# Pointing — pre-registration (2026-09-17, written before any of these arms flew)

**Plain English.** The camera is bolted facing forward and the vehicle yaws to face the way it
is *going*. Against a target crossing in front of it, the vehicle flies a lead course — it aims
ahead of the target — so the target is never in front of it; it is off to one side. And
because the vehicle tips ~43° nose-down to sprint, the target is also far above the camera's
centre line. Tonight's measurement (first time the vehicle's tilt was logged):

| true range | target outside the picture | at the picture's edge | central |
|---|---|---|---|
| 8–22 m | 86% of ticks | 14% | **0 of 858 ticks** |
| 4–8 m | 74% | 18% | 8% (really detected on 72% of those) |
| under 4 m | 56% | 24% | 20% (really detected on 86%) |

32 camera flights, `scripts/forensics/inframe_attribution.py`. Off the nose sideways by
**40–75°** for most of the sprint (the seeker accepts ±30°, the picture ends at ±50°); above the
centre line by a median **46°** (the picture ends at 41.6°). So the seeker is not failing to
recognise the target — it is almost never shown it.

## The two levers

1. **Yaw toward the predicted target** (`--dash-yaw-to-predicted-los`, built tonight, default
   OFF, tested). The velocity command is untouched; only the yaw setpoint changes, from "along
   the sprint" to "toward where the target is predicted to be" (pre-flight target kinematics —
   the same launch cue the aim already uses — plus the vehicle's own displacement). Honesty:
   own-state + the declared launch-cue given; no live target sensing. Cost: zero hardware.
   Expected side effect: the sprint tilt splits into ~28° pitch + ~33° roll, so the image is
   rolled ~30° and the target should sit ~28° above centre instead of 46°.
2. **Fixed up-tilt, 25°** (`up25` mount + `--cam-mount-up-deg 25`). Offline from tonight's logs a
   25° tilt centres the target vertically on 100% / 89% / 69% of ticks in the three range bands
   (35° gives 89/84/63; 15° gives 58/54/76) — the first time the contract's "size the wedge to the
   dash pitch" decision has a number from attitude data. This is the hardware wedge already
   decided (stage `pointing`); it is gated on prop clearance on the real airframe.

## Arms (seed 123, n = 8, both directions, sequential, idle load; all carry ADR-0100's height offset)

| arm | what |
|---|---|
| `PCAM` | control: accel-aware lead, camera live |
| `PYAW` | + yaw toward the predicted target |
| `PYAWT` | + yaw + 25° up-tilt |
| `PDASH` / `PYAWD` | dash-only twins without / with the yaw lever — does yawing change the sprint itself? |

## Predictions

- **M1, central at 8–22 m** (% of pre-closest-approach ticks): `PCAM` ≈ 0%; `PYAW` ≥ 30%; `PYAWT` ≥ 60%.
- **M2, REAL detections at 8–22 m** (within 250 px of the true target), summed over 8 flights:
  `PCAM` ≤ 3; `PYAW` ≥ 20.
- **M3, range of the first real detection**, per-flight median: `PCAM` ≈ 4–5 m; `PYAW` ≥ 8 m.
- **Ballistic harm check:** `PYAWD` median closest approach within 0.10 m of `PDASH`.

**Adopt the yaw lever iff** M1 and M2 are met and the harm check passes. M3 is reported.
**Whether the miss improves is NOT a criterion here** — it is reported, against its dash-only
twin, and is the next question, not this one.

## What a NULL would mean

- Target becomes central but real detections do not appear at 8–22 m → in-flight recognition at
  range IS a wall after all (a ~30° rolled image, a small target, latency), and tonight's "it is
  pointing, not recognition" is wrong beyond 8 m. That would be worth knowing before a wedge is printed.
- Real detections appear but `PYAW`'s miss is no better than `PYAWD`'s → seeing the target earlier
  does not help this terminal; the limit is then guidance/time-to-go, not perception.
- `PYAWD` worse than `PDASH` by > 0.10 m → yawing off the velocity vector changes the sprint
  (PX4 couples yaw and thrust direction) and the aim must be re-solved for it, as with the
  climb gain (vertical_channel_prereg.md §10.9).

## Arm asymmetries and regime limits

- Only the yaw arms fly with a large roll; roll-dependent failure modes (NN on a rolled image,
  range-from-box-size bias) are reachable only there.
- Only `PYAWT` can lose the target out of the BOTTOM of the picture when the vehicle brakes nose-up.
- The own-airframe phantom (~1.5 m, graveyard) is untouched by both levers; more real detections
  do not remove it. Count phantoms separately; do not read "more detections" as "more real ones".
- 40–75° is this geometry (6.5 m standoff, 9 m/s crossing). A head-on target has no lead angle
  and needs neither lever; the numbers do not transfer.

## Amendment, written while the five arms above are still flying and BEFORE any was read (2026-09-17)

Checking how fast the vehicle can actually turn its nose, I looked at one old camera flight's
first second: **it begins the sprint facing 97° (roughly east, the way the sim spawns it) while
the commanded heading is 65°, and it turns at only ~25°/s while pitching over** — 97° → 83° in
the first 0.57 s of a ~1.5 s sprint. So in every camera arm ever flown here, the nose spent the
whole sprint catching up with its command. The real launch procedure does not do this: the
vehicle sits in a standby hover with the aim heading already held (stage `launch_aim`,
assumption `standby-yaw-hold`). The sim skipped that step. It costs the sprint nothing (the
velocity command is in world axes) but it costs the CAMERA most of its field of view — a
scenario-realism defect that only the camera arms can suffer.

**Consequence for the arms in the air:** `PYAW` asks for a 60–75° yaw change it cannot complete
in 1.5 s. I now expect `PYAW` to **under-deliver on M1/M2 as registered**, for a reason that is
not the lever's fault. That prediction is written here before reading it, so that a weak `PYAW`
is neither spun as a success nor taken as the lever's verdict.

**Added arms (same seed 123, same metrics M1–M3 and harm check, flown after the first five,
on a HEAD that adds only the default-OFF `--dash-prealign-yaw`):** the vehicle first hovers and
yaws to the sprint's first yaw command (within 3° for 0.3 s; logged as phase `STANDBY`; the
target does not start moving until the sprint does).

| arm | what |
|---|---|
| `PCAMA` | control + pre-align (nose on the sprint heading — what the real procedure does) |
| `PYAWA` | yaw-to-predicted-target + pre-align |
| `PYAWTA` | … + 25° up-tilt |
| `PDASHA` / `PYAWDA` | dash-only twins |

**Predictions for the aligned arms:** `PCAMA` M1 stays low (< 10%: the lead angle alone keeps the
target 30–60° off the nose) — if `PCAMA` alone reaches M1 ≥ 30% then the missing standby step, not
the lead geometry, was the wall, and the yaw lever is unnecessary. `PYAWA` M1 ≥ 30%, M2 ≥ 20.
`PYAWTA` M1 ≥ 60%. The adopt rule is unchanged and is applied to the ALIGNED arms; the un-aligned
five are reported alongside as the "what the sim had been doing" baseline.

## RESULT — the first five arms (un-aligned), seed 123, HEAD `7392dfe` (2026-09-17)

`scripts/forensics/inframe_attribution.py` (with `--mount-up-deg 25` for `PYAWT`), pre-closest-approach ticks:

| arm | 8–22 m: outside (top / side) | 8–22 m: CENTRAL | … really seen when central | 4–8 m: central, really seen | real detections at 8–22 m (M2) | median miss |
|---|---|---|---|---|---|---|
| `PCAM` control | 92% (157 / 82) | 5 of 261 = 2% | 1 of 5 | 23 of 70, 87% | 3 | 0.94 m |
| `PYAW` yaw only | 98% (171 / 83) | 2 of 260 = 1% | 0 of 2 | 2 of 28, 50% | 1 | 1.15 m |
| `PYAWT` yaw + 25° tilt | **49% (0 / 121)** | **54 of 249 = 22%** | **35 of 54 = 65%** | **49 of 51, 94%** | **43** | 0.85 m |

Against the registered predictions:
- `PCAM` M1 ≈ 0% — **met** (2%).
- `PYAW` M1 ≥ 30%, M2 ≥ 20 — **NOT met (1%, 1).** As the amendment predicted before reading: the nose
  cannot slew 60–75° in a 1.5 s sprint, and with no tilt the target leaves by the TOP on even
  more ticks (171) than the control. Yaw alone, un-aligned, does nothing.
- `PYAWT` M1 ≥ 60% — **NOT met (22%)**; M2 ≥ 20 — **met (43)**. The tilt removed the top-edge loss
  completely (157 → 0 ticks); what is left is 121 ticks off the SIDE — the slow yaw again.
- Harm check: `PYAWD` 0.675 m vs `PDASH` 0.648 m median (lens ruler, n = 7 vs 8 — one `PYAWD` flight
  failed to enter OFFBOARD before the sprint, a boot failure, counted not dropped). Within 0.10 m:
  **passes.** NOTE the airframe-centre scorer correctly REFUSES yawed flights (it reconstructs
  the body from the velocity heading); yawed arms are scored on the lens ruler until it is
  taught the logged attitude.

**The registered NULL question is answered, in the good direction.** "Target central but not
detected at 8–22 m → recognition in flight at range is a wall": with the target central the
seeker really found it on **65% of ticks at 8–22 m, 94% at 4–8 m, 100% under 4 m**, in flight, at
~43° of pitch and with the image rolled. Recognition is not the wall. The seeker had never been
shown the target.

**Adopt rule:** applies to the ALIGNED arms (amendment). Nothing is adopted from this table.
**Miss:** no arm's median moved beyond the scatter (0.85–1.15 m, n = 8). Seeing the target earlier
has not yet made the terminal land closer — that is the next question, and it is the one the
whole camera story hangs on.

## RESULT — the first two ALIGNED arms (seed 123), and the finding they forced (2026-09-17)

| arm | 8–22 m: target CENTRAL | … really seen when central | median miss, left-to-right / right-to-left |
|---|---|---|---|
| `PCAM` (un-aligned control, for scale) | 2% | — | ≈ 0.9 m overall |
| `PYAWA` aligned + yaw | **151 of 257 = 59%** | 77% | **1.98 / 5.13 m** |
| `PYAWTA` aligned + yaw + 25° tilt | **245 of 248 = 99%** | 81% | **1.79 / 4.85 m** |

Pre-alignment works in flight (nose within ~3° of its command after ~2.5 s of hover).
**M1 and M2 are met overwhelmingly — the pointing problem is solved in this sim: the seeker is
shown the target on 99% of ticks from 22 m in, and recognises it on 81% of them.**

**And the miss got two to five times WORSE.** Mechanism, measured, not guessed: with the target
in view from the first tick, the "5 fresh detections" handoff fires at **~17.5 m, with the vehicle
still at 0.1 m/s** — before the sprint has happened. The camera terminal then commands its own
closing-speed law, **9 m/s tapering to 5.5 m/s** (sized in ADR-0010 for a ~6 m/s target and a
short-range handoff), against a target crossing at 9 m/s. The vehicle never sprints; it
tail-chases. In the un-aligned control the good flights are exactly the ones whose handoff came
LATE (2–4 m, at 11.5 m/s true speed) and the bad ones handed off early (11–14 m).

**So for the whole history of this project the camera terminal looked "about as good as the
sprint" only because it was blind until the last 0.2 s. Shown the target early — tonight, for the
first time — it is far worse than not using it.** That is the most important thing learned
tonight, and it re-ranks the project: pointing was the wall in front of the real wall.

**Decision on the levers (adopt rule as registered):** M1/M2/harm-check are met, so the pointing
levers are VALID as pointing levers. They are **NOT put into the adopted flight config**, because
with today's terminal they make the intercept worse — adopting them now would be obeying the
letter of my own criterion against its purpose. They become the test bench for the terminal.

## Pre-registration — does the terminal's SPEED LAW explain it? (written before flying)

**Lever:** `--terminal-vclose-min 16` (built, default OFF): the terminal may never command less
along-line-of-sight closing speed than the sprint. One change; the steering law is untouched.
**Arms (seed 123, n = 8):** `PYAWTAV` = `PYAWTA` + the floor, against tonight's `PYAWTA` and the
sprint-only twin `PYAWDA`.
**Prediction:** the early handoff stops costing speed, so `PYAWTAV`'s median falls from ~3.3 m to
**under 1.0 m**; if the steering law is sound it should then BEAT its sprint-only twin (≈ 0.65 m)
on ≥ 6/8 pairs — the first time in this project the camera would have earned its handoff.
**Adopt iff** ≥ 6/8 paired wins over `PYAWDA` AND median ≤ 0.65 m; then replicate on seed 777.
**NULL (stated now):** if `PYAWTAV` is still ≥ 1.5 m, the speed law is not the (only) problem —
the pro-nav steering itself cannot hold a 25 m/s closure against a crossing target with this
range/rate channel, and the honest next step is a terminal REDESIGN (sprint-speed,
look-angle-constrained), which is a council-grade decision, not a flag. If it lands between
0.65 and 1.5 m: better, not earning — say exactly that.
**Asymmetry check:** only the floor arm can overshoot at 25 m/s closure inside the last metres
(line-of-sight rate blow-up — the reason ADR-0010 slowed the terminal). Report the fraction of
flights whose miss is dominated by the last 3 ticks, and the right-to-left / left-to-right split.
