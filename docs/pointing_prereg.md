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
