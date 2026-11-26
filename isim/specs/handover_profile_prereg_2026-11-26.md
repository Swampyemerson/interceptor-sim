# Pilot hand-over profile in the fast sim: pre-registration (2026-11-26)

Written and committed BEFORE any registered run. Development smoke runs use
seeds 1000 and up only; the registered seeds are 0 to 23.

## Question

On Saturday 2026-11-28 the builder wants to fly, in order: (1) a 2 mph target
carrying a rear-facing AprilTag, the interceptor closing on the tag and pulling
away before contact; (2) the same at full speed; (3) contact. The flight program
gained a pilot hand-over mode on 2026-11-26 (`flight/deploy/real_flight.py`,
commit d8c34dd: `HandoverLimits`, `run_pilot_handover`). Its plumbing was tested
on a fake vehicle with a synthetic detector. Nothing has tested whether the
profile can acquire a tag, close and pull away. This asks the fast sim.

Lab ranks, Gazebo and the field decide. Nothing here is a flown result.

## What is run

Runner: `scripts/handover_profile_isim.py` (new; no change to `isim/` or
`flight/`). The state machine, the pursuit terminal and the limiter are the
flight code's own objects:

- arguments are parsed by `real_flight.build_arg_parser()` from the command line
  the aircraft would be given (`--pilot-handover --trigger cue --terminal pursuit
  --source picamera --detector tag --camera-mount upside-down --min-agl-m 2
  --pursuit-rehearsal --mount-tilt-deg 20 --handover-max-speed-ms CAP ...`), and
  must pass `validate_handover_args`;
- `build_config` then `handover_config` give the mission config; `build_terminal`
  gives the pursuit terminal (so the flying preset and the rehearsal practice
  profile are whatever the flight code applies);
- the belief reaches the terminal the way it does in flight: on the GO tick, as
  `VehicleObs.cue_belief`, with `cue_go` on;
- every setpoint passes `HandoverLimits.limit` with `camera_guided` computed by
  the same expression the control loop uses (terminal phase "B", or the
  rehearsal trigger set);
- guidance ticks at 20 Hz (`--fps 20`), decodes at most 20 a second.

Sim-side world (truth, never shown to guidance):

| Item | Value | Source |
|---|---|---|
| Camera | 1280 x 800, fx = fy = 800 px (77 deg wide, 53 deg tall), cx 640, cy 400 | AR0234 3.0 um pixels behind the 3.6 mm lens, read out at 2/3 scale. NO calibration exists; this is arithmetic |
| Camera tilt | 20 deg up, told to the flight code as 20 | as mounted 17 to 21 from video frames (`runs/flight_log_analysis_20261126`) |
| Tag | rear-facing, bolted to the target body, black square 0.10 / 0.16 / 0.20 / 0.35 m | builder has not said which; 0.35 m is the print-ready placard |
| Vehicle | `isim/fits/vehicle_gazebo_x500_engage.json` with the 2026-11-18 rerun's "now" envelope (45 deg lean, 5 m/s2, thrust 4 g, speed limit 33 m/s) | the logs of 2026-11-26 agree with its velocity lag (0.12 to 0.16 s) |
| Target | (a) hover; (b) straight at 0.9 m/s; (c) circle of radius 2.75 m at 1.0 m/s (21 deg/s; the measured course is about 5 by 6 m, 19 deg/s); fast rung: straight at 5 and 9 m/s. Height 6 m | `runs/flight_log_analysis_20261126` |
| Start | interceptor hovering 8 to 20 m behind the target (uniform), up to 20 deg off dead astern, height = target height + U(-2, +2) m, nose toward the target +/- 30 deg | directive |
| Cue error (belief at GO) | horizontal position 2 to 3 m in a random direction; height U(-5, +5) m; velocity 0.05 to 0.13 m/s in a random direction | 2026-11-26 ground runs and logs |
| GO | at 3.0 s; the target is already moving | |
| Window | 75 s (slow rungs), 60 s (fast rung) | |

Arms:

- **S (slow grid):** targets a, b, c x tag 0.10, 0.16, 0.20, 0.35 x cap 0.9, 1.5,
  2.5 m/s. 36 cells x 24 seeds.
- **H (height hold):** the as-built limiter against a counterfactual that lets the
  capped vertical command through from GO (vertical on the belief). Targets a and
  b, tags 0.20 and 0.35, cap 1.5. Run with the registered height error (+/-5 m)
  and with a height error of +/-1 m.
- **G (start geometry):** target b, tag 0.20 and 0.35, cap 1.5: start height pinned
  at -2, -1, 0, +1, +2 m relative to the target; start distance pinned at 6, 10,
  15, 20 m.
- **W (worst tier)** on the configuration recommended from S, G: OU gust 1.5 m/s
  horizontal and 0.5 m/s vertical, true camera tilt 17 deg while the flight code
  is told 20, decode ceiling 0.80, target wobble 3 deg RMS, cue horizontal error 3
  to 5 m.
- **F (fast rung):** target straight at 5 and 9 m/s, cap 16 m/s
  (`--handover-allow-fast`), tags 0.20 and 0.35, same break-off, vertical cap 1.0.

## Measures (truth-side, scoring only)

Per run: whether any frame decoded and the time and true range of the first; when
the terminal entered its camera phase; whether the rehearsal break-off triggered,
its time and true range; "too late" flag; minimum centre-to-centre separation over
the run and after the trigger; contact = separation under 0.35 m; final state and
reason; for frames where the tag was large enough to decode (apparent side at
least 22 px) but not in the picture, whether it was off the top, the bottom or a
side. Every cell reports its denominator; a cell with fewer than 24 rows is
UNCERTAIN, not a pass.

## Predictions (written from arithmetic, before running)

- **P1, tag size sets acquisition.** 22 px apparent side (50 % decode) is reached
  at 3.6 / 5.8 / 7.3 / 12.7 m for 0.10 / 0.16 / 0.20 / 0.35 m. The chase flies to a
  point 8 m behind the believed target, and the belief is 2 to 3 m wrong. Hover
  target, cap 1.5: acquire fraction under 30 % for 0.10 m, 30 to 70 % for 0.16 m,
  over 60 % for 0.20 m, over 90 % for 0.35 m.
- **P2, height.** Tilted 20 deg up with 26.6 deg of half-height, the level camera
  sees from 6.6 deg below the nose to 46.6 deg above it. A target 1 m lower than
  the interceptor at 8 m is at the bottom edge. With height held until the camera
  has the tag: starts 1 to 2 m ABOVE the target acquire in under 25 % of runs;
  starts at or below acquire at least twice as often.
- **P3, the cap.** At a 0.9 m/s cap against a 0.9 m/s receding target the gap never
  closes: the break-off triggers in at most 5 % of runs. At 1.5 and 2.5 it closes.
- **P4, break-off margin at slow closure.** The trigger is time-to-go under 1.5 s
  with the closing speed floored at 1.5 m/s, so it fires near 2.25 m and is "too
  late" inside 0.9 m. Where it triggers at caps up to 2.5 m/s: separation after the
  trigger stays at or above 1.0 m in at least 95 % of triggered runs, and no run
  makes contact.
- **P5, the loop.** The frozen belief is stale within seconds. Acquire fraction is
  lower than on the straight target for the same tag and cap, and the break-off
  triggers in under 50 %.
- **P6, the fast rung.** The rehearsal profile caps the chase at 10 m/s. From a
  hover 8 to 20 m behind a 9 m/s target the gap opens while the interceptor
  accelerates and then closes at 1 m/s or less: the break-off triggers in under
  50 % of runs inside 60 s and the aircraft travels over 300 m. Behind a 5 m/s
  target it triggers in over 70 % with the 0.35 m tag.

## Criteria

- **Recommended configuration for Saturday's first run (rung 1).** A cell (tag,
  cap, plus any start-geometry restriction taken from arm G) is recommended only
  if, over 24 seeds on target b: tag acquired in at least 22, break-off triggered
  in at least 20, zero contacts, zero "too late", and separation after the trigger
  at or above 1.0 m in every triggered run. If several pass, the one with the most
  triggers, then the largest worst-case separation. If none passes, none is
  recommended and the best cell is reported as such.
- **Worst tier.** The recommended cell must keep zero contacts under arm W. Its
  acquire and trigger counts are reported and may fall.
- **Height hold costs acquisitions** if the counterfactual acquires in at least 5
  more runs of 24 than the as-built limiter in any H cell. It is "not shown to
  cost" if the difference is 2 or fewer in every H cell.
- **Fast rung looks sane** only if, for some tag, the 9 m/s cell has break-off
  triggered in at least 20 of 24, zero contacts, zero "too late" and separation
  after the trigger at or above 1.5 m in every triggered run.

## What a null would mean

- No slow cell meets the rung-1 criterion: the first camera run on Saturday
  should not be expected to pull away by itself. The pilot's GO switch is then
  the only separation, and the run should be planned as "does it acquire and
  head for the tag", ended by hand.
- The break-off never triggers at slow closure although the tag is acquired: the
  trigger's 1.5 m/s closing floor and the limiter's caps do not fit a 2 mph
  course. That is a property of the configuration, to be reported with the
  evidence, not tuned here.
- The fast rung fails its criterion: full speed with this break-off, from a hover
  behind the target, is not ready on sim evidence, whatever rung 1 shows in the
  field.

## Known divergences from the real world that bear on this

- The tag is an ideal square scored by a size, incidence and blur model; nothing
  renders. At under 2 m a real tag overfills or leaves the picture in ways the
  model treats only as "out of view".
- The target's attitude is derived from its path; a real small quad at 1 m/s in
  wind wobbles and yaws on its own. Arm W adds 3 deg of wobble only.
- GPS error is a fixed offset drawn once per run, not a wandering one, and the
  loop target's belief uses its true instantaneous velocity plus the error; the
  real fit over 5 s of a turning target would be worse.
- Own-state errors (attitude, velocity, heading) are zero. The 2026-11-26 logs
  show heading swings of 11 to 29 degrees on the three poor-GPS flights.
- The vehicle is the Gazebo x500 fit, not the built aircraft; only its velocity
  lag has been compared with a real log.
- Separation is centre to centre. The aircraft are about 0.3 m across.

## Amendment 1 (2026-11-26, written AFTER arms S, H, G and F were read, before arms W and A ran)

Post hoc, and labelled as such: these arms were chosen because of what the
registered arms showed, so they explain, they do not confirm.

What prompted it:

1. The sim decoded 0.16 and 0.20 m tags at 13 m, where they are 10 to 12 px across.
   That is the tail of the sim's size curve (a few per cent per frame at 20 frames a
   second). The flight code runs the detector at `quad_decimate` 2, which halves the
   pixels it works on. Prediction P1 was wrong against the sim, but the sim's
   acquisition ranges are not believable for a tag-size choice.
2. Contacts happened with the break-off switched on, all on the hovering target, and
   all in runs that started more than 1.2 m above it. One traced run shows the tag
   sliding off the bottom of the picture at 2 to 2.75 m, just outside where the
   break-off triggers (about 2.25 m); the break-off needs a fed track, never fires,
   and the chase flies its last estimate into the target.
3. The flight program's `--mount-tilt-deg` defaults to 0. The camera is at about 20.

Arms (24 seeds each, seeds 0 to 23):

- **A strict:** hover and straight targets, cap 1.5, tags 0.10 to 0.35, with a decode
  floor of 20 px and half-probability at 30 px.
- **A hover height:** hover target, tags 0.16, 0.20, 0.35, caps 0.9 and 1.5, start
  height pinned at -2, -1, 0, +1, +2 m relative to the target.
- **A tilt told:** straight target, tag 0.20, cap 1.5, true tilt 20, flight code told
  0, 17 and 23.
- **W** as registered, on straight / tag 0.20 / cap 1.5 / start 0 to 2 m below the
  target (the cell the registered criterion selects: 24 of 24 triggers, largest
  worst-case separation among the cells that pass).

No criterion is attached to arm A. Expectations, stated before running: contacts on
the hover target concentrate at start heights of +1 and +2 m and are absent at 0 and
below; with the strict decode the 0.10 m tag is acquired late or not at all and the
0.35 m tag still acquires beyond 8 m; told 0 degrees, the chase climbs or descends
the wrong way and loses the tag.
