# Arrival geometry of the 500 m approach dash under a wrong target-speed belief (geometry check, 2026-11-18)

Read-only check of the head's claim: "500 m: about 25 s of transit, so 10 % of 12 m/s is
roughly 30 m of error on arrival; 13 to 17 kills of 50; the tag never decoded in 61 % of
runs", and of its reading that the target is behind the vehicle in the no-decode runs.

Everything below is the fast sim (isim), real flight code port arm, profile 36 / 4, plant R,
no drag. 466 traced runs of my own, scripts and rows in
`/tmp/claude-1000/-home-emerson-interceptor-sim/9f356921-9aa9-4fa7-970a-02847616f721/scratchpad/geom/`
(`A.json` 100 full-error runs with 2 Hz series; `BC.json` 166 controlled runs, speed belief
only; `D.json` 200 runs for two what-ifs; `tgtlog.json` the real target flight log).
Sign convention: "along" is the target's position minus the vehicle's, along the target's
direction of travel. Positive = target ahead of the vehicle. "eps" is believed speed over true
speed minus 1; positive = belief too fast.

## 1. Verdict on the head's reading

| Claim | Verdict |
|---|---|
| The speed-belief error decides decode versus no decode | **Confirmed.** Sorted by each run's actual eps the outcome is a clean band (section 2); no other error moves a run across it |
| "About 25 s of transit, roughly 30 m of error on arrival" | **Corrected.** The vehicle reaches its park point 18.2 s after GO (17.9 to 18.8). 25 s is when the kills happen. Belief error at arrival is 2.19 m per 1 % of speed error: 21 to 23 m one sigma at 10 % (measured sd 21.2 m near, 22.9 m demo), not 30 |
| "In the no-decode runs the target is behind the vehicle (28 of 31, 25 of 30)" | **Mostly right, wrong count and wrong instrument.** Classified by where the target sits after arrival: near 25 behind and 6 ahead; demo 25 behind and 5 ahead. The sign at closest approach is not usable for the "ahead" runs: their closest approach is the head-on pass on the way out, where along is about zero |
| "The tag is inside the picture for hundreds of frames without decoding" | **True but two different things.** Behind runs: about 480 frames (12.6 s), all on the way out, all looking at the FRONT of the target (the back of the tag), 499 m down to 24 m. After the U-turn the target is never in the picture again. Ahead runs: about 4,180 frames (109 s) looking at the tag from astern, but from 31 to 63 m at arrival and receding, below the 8-pixel floor |
| "The vehicle parks 8 m behind the believed target facing forward" | **Confirmed**, and it stays there: within 0.2 to 0.4 m of that point for the rest of the window, flying the believed speed in a straight line. It does not hold a steady offset to the real target: the gap grows at eps times 12 m/s for 100 s |
| "The error is along-track" | **Confirmed for the speed error; the heading error is a second, cross-track term.** The 2 degree heading error rotates the whole believed track about the launch point: 4.4 to 4.9 m of cross-track error per degree at the rendezvous (sd 8 to 9 m), plus 2.6 m per degree along-track in the demo geometry. Kills survive up to 15 m of it, so it does not gate the outcome |

## 2. Outcome against the actual belief error

How isim draws it (`isim/scenario.py` `build`): one rng per seed; truth start jitter 0.5 m;
`speed_eps ~ N(0, 0.10)` scales the BELIEVED velocity only; heading noise `N(0, 2 deg)`
rotates the believed start AND velocity about the launch origin. Recovered per run from the
guidance object (believed over true speed): sd 0.097 over these 50 seeds.

Full error package, 50 seeds per geometry (rows `A.json`):

| eps range | near: runs, outcome | Target at arrival | What happens next |
|---|---|---|---|
| -25.7 to -11.3 % | 6, no decode | 31 to 63 m ahead | Tag in view from astern the whole time, too small. Vehicle flies 10.4 m/s, target 12: gap 87 to 187 m at 60 s, 161 to 357 m at 115 s |
| -10.6 to -8.5 % | 3, decoded, no kill | 25 to 29 m ahead | 2 to a few decodes at 30 to 37 m, target lost, state machine goes SAFE and hovers; target 450 m away at 60 s |
| -5.9 to +2.6 % | 16, all kills | 18 m ahead to 0.8 m behind | First decode about 19 s after GO, contact about 25 s |
| +2.9 to +21.3 % | 25, no decode | 1.5 to 41 m behind | Vehicle flies 12.9 to 13.3 m/s ahead of a 12 m/s target, camera forward: gap 12 to 140 m at 60 s, 31 to 279 m at 115 s |

Demo geometry is the same table with 5 / 3 / 17 / 25 runs.

With every other error off (`BC.json`, two seeds per point):

| Lead, time to park | Decode band (eps) | Camera-kill band (eps) | Same band as target position at arrival |
|---|---|---|---|
| 500 m, 18.3 s | -12.5 to +2.5 % | -10 (1 of 2) or -7.5 to +2.5 % | decode 0 to 33 m ahead; kill 0 to 22 to 27 m ahead |
| 250 m, 12.3 s | -17.5 to +2.5 % | -12.5 to +2.5 % | decode 1 to 31 m; kill 1 to 24 m |
| 125 m, 8.2 s | -25 to +5 % | -17.5 to +5 % | decode 0 to 31 m; kill 0 to 23 m |

- **The window is fixed in metres, not in percent.** The target must be between about 0 and
  22 m ahead of the parked vehicle to be killed, 0 to about 31 m to be decoded at all. The
  speed-error budget is that window divided by speed times time to park.
- **It is lopsided.** With the 8 m park the belief may be up to about 13 to 20 m behind the
  truth but only about 5.5 m ahead of it. Fast-side tolerance is +2.5 %, slow-side 6 to 10 %.
- At +3.0 % with no other errors both runs are "kills" with zero decodes: the target runs
  into the vehicle parked exactly on its track. A blind hit, not a camera kill; it disappears
  with any cross-track error.

## 3. What the vehicle and target are doing (near geometry; all times after GO)

The launch point is AHEAD of the target on its track (the target flies toward it). So every
run is: fly out head-on, U-turn at the believed target, follow.

| Time | Kill (seed 44, eps -0.1 %) | Belief fast (seed 38, +10.5 %) | Belief slow (seed 8, -11.3 %) |
|---|---|---|---|
| 5 s | 381 m out, 26 m/s, lean 44 deg | same | same |
| 10 s | 166 m, braking, 25 m/s | 172 m | 159 m |
| 15 s | 30 m in front of target, 5 m/s | 47 m in front, 3.5 m/s | 16 m in front, 7.5 m/s |
| 16 to 17.5 s | passes the target head-on at about 17.5 s, 7 m to the side | stops and U-turns 23 to 33 m IN FRONT of the target; never passes it | passes head-on at about 16 s, 8 m to the side |
| 18 s (parked) | target 4 m ahead | target 17.6 m behind; picture empty from here on | target 27.5 m ahead |
| 19 to 20 s | first decode 19.3 s, target 7 to 8 m ahead, aspect 40 deg | 16 m behind, vehicle now 12 to 13 m/s | 33 to 35 m ahead, tag 8.9 then 7.9 px, one frame with a 4 % chance |
| 22 to 25 s | contact (0.04 m) | 23 m behind at 25 s | 42 m ahead at 25 s |
| 60 s | (hovering, SAFE) | 67 m behind, vehicle 13.3 m/s | 90 m ahead, vehicle 10.6 m/s |
| 115 s | | 136 m behind, 30 m to the side | 164 m ahead |

- Belief fast: the U-turn point (the believed target) is closer than the real target, so the
  vehicle turns around in front of it and then outruns it. On the way out the camera saw
  the target's nose from 499 m to 24 m; a rear tag cannot be read from there.
- Belief slow: the vehicle passes the real target, turns late, comes out too far astern and
  1.3 m/s too slow to close.
- "Closest approach" is a fair summary only for the belief-fast runs (it is roughly the gap
  at arrival). For belief-slow runs it is the sideways gap of the head-on pass.
- With no decode the vehicle follows the phantom to the end of the window: about 1 km
  from the launch point at 115 s in isim's 120 s window. Out of scope here, but it matters
  for the geofence and for what the engage time limit should be.

## 4. The decode envelope in this sim (0.5 m rear tag, fx 540.3, 1280 x 800)

From `isim/seeker.py` (`DecodeParams`, `p_decode`) and checked against the traces:

| Limit | Number |
|---|---|
| Apparent tag side | 270 px divided by depth in metres, times cos(aspect) |
| Hard floor, 8 px | 33.8 m depth dead astern. Farthest decode in my traces 37.6 to 38.8 m slant range (smallest decoded side 8.3 px) |
| Size term 50 % | 22 px = 12.3 m. 95 % at 35 px = 7.7 m. About 17 % at 18 m, 7 % at 25 m, 5 % at 30 m per attempt |
| Aspect (angle off the tag's normal, i.e. off dead astern) | 50 % at 60 deg, about 5 % at 75 deg, zero at 90 deg and beyond. Largest aspect at a decode in the traces 66 deg |
| Picture | 99.7 deg wide, 73 deg tall; all four corners must be inside |
| Attempt rate | one per 0.096 s at most (thinning), each scaled by p_max 0.6 to 0.98 under scatter |
| Hand-over to the camera phase | 2 decodes inside 0.3 s. At 5 to 7 % per attempt that almost never happens, which is why the useful range is about 20 m, not 34 |

Observed: first decode at 14 to 17 m median; kills when the target is 0 to about 21 m ahead
at arrival; decodes without a kill at 25 to 29 m.

## 5. The sim's 10 % and 4 % against what the real cue can deliver

- **What the fitter uses.** With MAVLink `GLOBAL_POSITION_INT` every record carries the
  target autopilot's own ground velocity, and `fit_cue` then takes the MEAN of those over the
  window (`flight/cue_relay.py:375-378`, `vel_source="reported"`), not the position slope.
  The plan's "worst 1.60 m/s" is for the position-slope path with 2.29 m of white position
  noise on a 2 s window. That path is the fallback, and the 2.29 m came from one static bench
  capture.
- **Measured today, from the real target's flight six log** (`C:\Temp\bench\target_flight6_20261115\log2.bin`,
  outdoors, 2026-11-15; `geom/tgtlog.py`): the GPS's own accuracy estimates in flight are
  speed 0.09 m/s median (0.08 to 0.13), horizontal 0.21 m, vertical 0.30 m, at 5 Hz. The
  bench figures in the plan were 1.72 m/s, 2.29 m and 2.84 m. These are the receiver's
  self-reported figures, not errors against a reference.
- **A consistency check on the same log:** over 213 sliding 3 s windows the mean reported
  velocity and the 3 s position slope differ by 0.22 m/s rms (0.28 m/s in the 51 windows above
  1 m/s). That flight was 3 m legs at under 2.7 m/s with constant turning, so this is an upper
  bound inflated by manoeuvring.
- **So, as an estimate:** 0.1 to 0.3 m/s of speed error for a target on a steady straight leg.
  That is 0.8 to 2.5 % at 12 m/s and 1.1 to 3.3 % at 9 m/s. The sim's 4 % (0.48 m/s) is
  pessimistic by this estimate and its 10 % (1.2 m/s) is about the plan's worst case.
  **Not measured:** any straight leg at 9 to 12 m/s; no such flight exists.
- **The age of the cue is the bigger term, and today it is unbounded.** The speed error acts
  over cue age plus the 18 s to park. And per the interface audit (G3) the flight program does
  not advance the cue to GO at all: as coded the belief is wrong by target speed times the
  25 to 90 s between the last GPS sample and GO, hundreds of metres. Everything in this note
  assumes that is fixed.

## 6. Geometric levers, ranked

Simple model that reproduces the measurements: a kill needs the believed-minus-true
along-track error e inside (d - 21 m, d - 2.5 m), d = park distance behind the belief;
e is normal with sd = speed error (m/s) times time to park. It gives 0.32 at 10 % (measured
16 / 50 and 17 / 50) and 0.67 at 4 % (measured 27 / 50 and 31 / 50, so it is 5 to 13 points
generous there).

| Speed error | Time | Error sd at park | Park 8 m (today) | Park 12 m | Drop back only | Forward sweep from 2 sd astern |
|---|---|---|---|---|---|---|
| 1.2 m/s (10 %) | 18.2 s | 21.8 m | 0.32 | 0.33 | 0.72 | 0.97 |
| 0.48 m/s (4 %) | 18.2 s | 8.7 m | 0.67 | 0.71 | 0.93 | 0.96 |
| 0.3 m/s | 18.2 s | 5.5 m | 0.83 | 0.91 | 0.99 | (park is enough) |
| 0.2 m/s | 18.2 s | 3.6 m | 0.93 | 0.99 | 1.00 | (park is enough) |
| 1.2 m/s | 12.3 s (250 m) | 14.8 m | 0.46 | 0.47 | 0.81 | 0.97 |
| 1.2 m/s | 8.2 s (125 m) | 9.8 m | 0.62 | 0.65 | 0.91 | 0.96 |
| 0.2 m/s, cue 30 s old at GO | 48 s | 9.6 m | 0.63 | 0.66 | 0.91 | 0.96 |

1. **Forward sweep from deep astern (best single lever).** Park D0 = max(12 m, 2 sd) behind
   the belief, face along the track, then advance along the track at 4 to 6 m/s relative to
   the belief until the tag decodes. Every case inside 2 sd then has the target ahead and
   closing into the cone from dead astern, which is the tag's best aspect. Covers both sides
   in one motion: about 0.97 of runs get a decode opportunity at any speed error. Costs: at
   10 % D0 is 44 m and the sweep takes 6 s typically, about 40 s for a target 2 sd on the far
   side (88 m ahead, receding at 2.4 m/s); it needs ground speed of target speed plus the
   sweep rate, 16 to 18 m/s against a 12 m/s target, which the board's stock 12 m/s limit
   cannot give. Arriving from astern by a different route is the same lever.
2. **Drop back at the park point.** If nothing decodes within about 2 s of parking, fly 2 to
   4 m/s slower than the believed target. A target behind catches up (22 m in 4 to 7 s),
   passes 7 to 9 m to the side and comes out ahead inside the cone (needs 6 to 8 m ahead
   for the picture edge and the 60 degree aspect), pulling away at 3 to 5 m/s. Recovers the
   belief-fast runs, which are 25 of the 30 or 31 losses: about 0.72 at 10 %, 0.93 at 4 %.
   Needs no speed margin, so it works on the stock board. It makes the belief-slow runs
   worse unless a forward push follows, and it hands the terminal a receding target, which is
   the case the other fork's lost-target work is about.
3. **A better and fresher cue.** At 0.2 m/s and a cue advanced to GO the park alone gives
   0.93 to 0.99. This is the cheapest lever in flight terms and the only one that also fixes
   the hundreds of metres from an unadvanced cue.
4. **Park deeper, 12 to 14 m instead of 8.** Centres the lopsided window. Worth nothing at
   10 % (measured 19 / 50 against 16 at 14 m), worth having once the cue is good (measured
   38 / 50 against 27 at 4 % with 14 m; the smooth model says +3 to +8 of 50, the sample has
   a cluster of seeds just past the old threshold).
5. **Shorter transit.** Error scales with time to park. Halving the cue range (250 m) gives
   0.46, a quarter (125 m) 0.62 at 10 %. Raising the dash cap from 20 to 36 m/s moves the
   time to park by about 3 s, a couple of points. Range matters; top speed barely does.
6. **An any-aspect detector, or a second tag facing forward.** Measured with isim's
   best-case stand-in (tag always faces the camera, same size and range): decodes in 48 / 50
   and 47 / 50 runs instead of 19 and 20, first seen 14 to 17 s after GO on the way OUT at
   20 to 43 m. But kills only rise to 33 / 50 and 26 / 50, and 8 runs that were kills with
   the rear tag become misses of 0.7 to 18 m (two more land at 0.37 to 0.38 m): the terminal
   takes its first sight head-on at about 20 m/s closing and flies a pass. So this lever is the largest for seeing the
   target and needs a guidance change before it pays: use an out-bound sighting to correct
   the belief's along-track position and keep flying to the stern position, not to start the
   close. A detector with 50 to 60 m of range would also cover the belief-slow runs (target
   31 to 63 m ahead at arrival).
7. **Which way to face / a yaw scan.** Looking back from the park point shows the target's
   nose. With a rear tag that reads nothing. It only helps together with lever 6.
8. **A second tag facing sideways.** The target passes abeam at 7 to 9 m, 90 degrees off
   the camera axis and outside the 99.7 degree picture. No gain in this geometry.

## 7. Where I disagree with the head

- 30 m at arrival should be 22 m (18 s, not 25 s). The conclusion survives: 22 m of sd
  against a 19 to 22 m window.
- "Behind in 28 of 31" should be 25 of 31 (and 25 of 30); the other 6 (and 5) are the target
  too far AHEAD, and those need the opposite remedy.
- "The runs where the belief runs slow leave the target ahead and in view, and those are the
  runs that decode" holds only for a belief up to about 6 to 10 % slow. Beyond that the
  target is ahead, in view and out of range.
- The cue plan's bench accuracy figures (2.29 m, 1.72 m/s) look far too pessimistic against
  the target's own flight log (0.21 m, 0.09 m/s self-reported). Worth re-deriving the plan's
  budget from flight data once a straight fast leg exists.

## 8. Limits of this check

- isim only; the vehicle model is the x500 fit with a 60 degree lean limit and 4 g thrust.
- The 50 seeds are the same 50 in every package, and their eps draws are lumpy (seven seeds
  between +7.6 and +8.0 %), so differences of a few runs between packages are not resolvable.
- The decode model's size and aspect curves are literature guesses, not bench fits
  (`DecodeParams` docstring); the 20 m useful range moves with them.
- The lever probabilities other than park depth and any-aspect are arithmetic, not runs.
