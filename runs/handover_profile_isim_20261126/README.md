# The pilot hand-over profile in the fast sim, 26 November 2026

Asked before Saturday's flying: can the new hand-over profile acquire a
rear-facing AprilTag, close on it, and pull away before contact, and with what
settings? Fast sim only. The flight program's own state machine, pursuit terminal
and command limiter fly (commit d8c34dd); this folder's runner only builds the
world and scores. Nothing here has flown, and the fast sim ranks options; it does
not decide them.

- Pre-registration: `isim/specs/handover_profile_prereg_2026-11-26.md`, committed
  before any registered run (bc13df2). Amendment 1 (post hoc arms) was committed
  before those arms ran (5f01392).
- Runner: `scripts/handover_profile_isim.py`. Rerun an arm with
  `.venv/bin/python scripts/handover_profile_isim.py --arm S` (S, H, G, F, A;
  W takes `--tag-m --cap --dh-lo --dh-hi [--strict] [--suffix]`); `--report`
  prints every table from the saved rows.
- One run step by step: `trace_one.py TARGET V_T TAG CAP SEED [DH]`.
- 24 seeds per cell (0 to 23), every cell complete. `*.json` are the rows,
  `*.log` the tables as printed.

No change was made to `isim/` or `flight/`.

## The short version

1. **On a straight 2 mph target it works in the sim, if the speed limit is above
   the target's speed and the interceptor does not start above the target.** At a
   1.5 m/s limit: tag acquired 24 of 24, break-off triggered 24 of 24, no contact,
   and never closer than 1.7 m after the trigger, for tags of 0.16 m and up.
2. **At the default 0.9 m/s limit it never closes on a 0.9 m/s target.** 0 of 96
   break-offs; the gap stays where it started.
3. **Contacts happened with the break-off switched on: 35 of 864 runs in the main
   grid.** The cause is one mechanism (next section), and it is a gap in the flight
   code's break-off, not a tuning matter.
4. **The backyard loop is the wrong course for this.** The break-off triggered in
   5 of 288 runs and 13 made contact.
5. **Tag size: only the 0.35 m placard is safe to plan on.** The sim "decodes" the
   0.16 and 0.20 m tags at 13 m, where they are 10 to 12 px across, which a real
   detector will not do. With a stricter decode rule the 0.20 m tag was acquired
   in 9 to 11 of 24 runs and the 0.35 m tag in 22 to 24.
6. **The full-speed rung is a long tail chase.** The break-off profile caps the
   chase at 10 m/s whatever limit is typed. Behind a 9 m/s target it took a median
   33 to 36 s and 330 to 360 m to close (worst 466 m, past the 457 m geofence).
   It then broke off in 24 of 24 with at least 1.5 m.

## The contact mechanism (the finding that matters most)

The break-off only fires on a track that is being fed by decodes (three inside
the last half second). It fires at about 2.1 to 2.3 m at these speeds. The camera
points 20 degrees up, so a target at the interceptor's own height sits near the
bottom of the picture, about 3.6 degrees from the edge. As the gap closes the tag
grows and slides off the bottom: for the 0.35 m tag at 2 to 2.75 m, the same
distance the break-off would have fired at. If the interceptor is even slightly
above the target, the tag goes first. Then:

- no decodes, so the track is no longer "fed" and the break-off cannot fire;
- the chase keeps flying its last estimate toward the target at the speed limit;
- the lost-target rule ends it 2.0 s later (`pursuit_miss ... target lost 2.0s`).
  At 1.5 m/s that is 3 m of travel from a 2 to 2.75 m gap.

`trace_one.py hover 0 0.35 1.5 6` shows it: decodes at 16 to 20 a second down to
2.75 m, none from 2.0 m, the command still 1.5 m/s, closest approach 0.05 m at
16.0 s, SAFE at 16.2 s.

Of the 35 contacts in the main grid, 32 ended this way with no break-off at all.
Where the interceptor started (its height minus the target's), main grid, all
tags and limits:

| Target | Start height | Runs | Contacts | Break-offs |
|---|---|---|---|---|
| Hover | 1 to 2 m below | 72 | 0 | 70 |
| Hover | 0 to 1 m below | 72 | 0 | 68 |
| Hover | 0 to 1 m above | 48 | 0 | 44 |
| Hover | 1 to 2 m above | 96 | 14 | 39 |
| Straight 0.9 m/s | 1 to 2 m below | 72 | 0 | 46 |
| Straight 0.9 m/s | 0 to 1 m below | 72 | 0 | 48 |
| Straight 0.9 m/s | 0 to 1 m above | 48 | 0 | 32 |
| Straight 0.9 m/s | 1 to 2 m above | 96 | 8 | 43 |

(The straight rows include the 0.9 m/s limit, which never closes, so their
break-off counts are low for that reason.)

Pinned start heights on the hover target (amendment arm, post hoc), 24 runs each,
contacts / break-offs:

| Tag, limit | 2 m below | 1 m below | level | 1 m above | 2 m above |
|---|---|---|---|---|---|
| 0.16 m, 0.9 m/s | 0 / 24 | 0 / 24 | 0 / 24 | 0 / 24 | 0 / 6 |
| 0.16 m, 1.5 m/s | 0 / 24 | 0 / 24 | 0 / 24 | 3 / 13 | 0 / 1 |
| 0.20 m, 0.9 m/s | 0 / 24 | 0 / 24 | 0 / 24 | 0 / 23 | 0 / 13 |
| 0.20 m, 1.5 m/s | 0 / 24 | 0 / 24 | 0 / 24 | 5 / 13 | 1 / 2 |
| 0.35 m, 0.9 m/s | 0 / 24 | 0 / 24 | 0 / 24 | 2 / 18 | 5 / 2 |
| 0.35 m, 1.5 m/s | 0 / 24 | 0 / 24 | 0 / 24 | 7 / 11 | 0 / 0 |

Level or below: 0 contacts in 432 runs and a break-off every time. One metre
above: contacts in up to 7 of 24. The worst tier (below) shows the same loss
happening from other causes even when starting below, without contact.

**Reported as a defect in the flight code, not fixed here** (`flight/` was not
edited): with the rehearsal break-off on, losing the tag at close range does not
break off or stop. The chase flies on for the 2.0 s lost-target clock at full
commanded speed. Evidence: the rows in `S.json` and `A.json` with `contact` true
and `trig_t` null, and the trace above.

## Pre-registered predictions and criteria

| | Prediction | Result |
|---|---|---|
| P1 | Tag size sets acquisition: under 30 % for 0.10 m, 30 to 70 % for 0.16 m, over 60 % for 0.20 m, over 90 % for 0.35 m (hover, limit 1.5) | **Wrong against the sim as built**: 18, 24, 24, 24 of 24. Right in kind under the stricter decode rule (post hoc): 3, 9, 11, 22 of 24. The sim's size curve has a tail that lets a 10 px tag decode |
| P2 | Starts 1 to 2 m above acquire in under 25 %; at or below at least twice as often | **Wrong on acquisition, right on the consequence.** Starting above still acquires at long range (the tag is in the picture at 14 m). What fails is close in: at 2 m above, 0 to 13 of 24 break-offs |
| P3 | Limit 0.9 against a 0.9 m/s target: break-off in at most 5 % | **Confirmed**: 0 of 96 |
| P4 | Where it triggers at limits up to 2.5: at least 1.0 m after the trigger in 95 % of triggered runs, and no run makes contact | **Half wrong.** After a trigger the separation held: 391 of 395 triggered runs in the main grid stayed at or above 1.0 m (99 %; the others 0.99, 0.85, 0.73, and 0.12 on the loop). But "no run makes contact" failed: 35 contacts, 33 of them in runs where it never triggered |
| P5 | Loop: fewer acquisitions than straight, break-off under 50 % | **Confirmed**, and worse: 5 break-offs in 288 runs, 13 contacts |
| P6 | 9 m/s target: break-off in under 50 % inside 60 s, over 300 m travelled; 5 m/s: over 70 % with 0.35 m | **Wrong on the first count** (24 of 24 at 9 m/s), right on distance (median 327 and 357 m, worst 466 m) and on 5 m/s (24 of 24) |

Criteria:

- **Rung 1 recommended cell** (target straight 0.9 m/s; acquire at least 22,
  break-off at least 20, no contact, no "too late", at least 1.0 m after every
  trigger): **met** by limit 1.5 m/s with tag 0.16 (24, 24, worst 1.75 m), 0.20
  (24, 24, 1.76 m) and 0.35 (24, 24, 1.72 m). The registered tie-break picks
  0.20 m. No cell at 0.9 m/s meets it (no break-offs), and no cell at 2.5 m/s
  meets it (1 to 3 contacts each).
- **Worst tier keeps zero contacts: met.** Tag 0.20, limit 1.5, start 0 to 2 m
  below: expected tier 24 acquire / 24 break-off / 0 contact; worst tier (gusts,
  camera 3 degrees off what the code is told, weaker decode, wobbling tag, 3 to
  5 m of cue error) 24 / 21 / 0. The three without a break-off lost the tag
  inside 3 m and stopped 0.87, 1.17 and 1.80 m away. Tag 0.35 (post hoc): 24 /
  19 / 0, closest 0.95 m.
- **Height hold costs acquisitions: not shown.** The counterfactual (vertical let
  through from GO) acquired the same 24 of 24 in every cell. It triggered 3 to 4
  more break-offs on the hover target and did not remove the contacts (3 of 24
  against 4 of 24 for the 0.35 m tag). In the sim the as-built limiter starts
  using vertical 0.1 to 2 s after GO anyway, because the tag is decoded almost at
  once; under the stricter decode rule the hold would last until the tag is close.
- **Fast rung looks sane (9 m/s cell: break-off at least 20, no contact, no "too
  late", at least 1.5 m after every trigger): met on the letter**, 24 of 24 with
  1.55 and 1.51 m worst. See the caveats below before reading that as a go.

## Tables

Columns: acquired (any decode after GO) / break-off triggered / contacts, then
the worst separation after a trigger, m.

Slow grid, hover target:

| Tag | Limit 0.9 | Limit 1.5 | Limit 2.5 |
|---|---|---|---|
| 0.10 m | 15 / 15 / 0, 1.54 | 18 / 14 / 0, 0.73 | 18 / 9 / 0, 1.49 |
| 0.16 m | 24 / 24 / 0, 1.50 | 24 / 18 / 2, 1.30 | 24 / 20 / 1, 1.58 |
| 0.20 m | 24 / 24 / 0, 1.51 | 24 / 20 / 2, 1.14 | 24 / 20 / 4, 1.72 |
| 0.35 m | 24 / 20 / 1, 1.39 | 24 / 16 / 4, 1.07 | 24 / 21 / 0, 0.85 |

Slow grid, straight at 0.9 m/s:

| Tag | Limit 0.9 | Limit 1.5 | Limit 2.5 |
|---|---|---|---|
| 0.10 m | 2 / 0 / 0 | 20 / 20 / 0, 1.70 | 20 / 17 / 1, 1.06 |
| 0.16 m | 17 / 0 / 0 | 24 / 24 / 0, 1.75 | 24 / 21 / 2, 1.17 |
| 0.20 m | 24 / 0 / 0 | 24 / 24 / 0, 1.76 | 24 / 20 / 3, 1.05 |
| 0.35 m | 24 / 0 / 0 | 24 / 24 / 0, 1.72 | 24 / 19 / 2, 1.21 |

Slow grid, loop (radius 2.75 m at 1 m/s):

| Tag | Limit 0.9 | Limit 1.5 | Limit 2.5 |
|---|---|---|---|
| 0.10 m | 4 / 2 / 0 | 2 / 0 / 0 | 3 / 1 / 1 |
| 0.16 m | 17 / 1 / 0 | 14 / 0 / 0 | 15 / 0 / 2 |
| 0.20 m | 23 / 0 / 1 | 22 / 0 / 0 | 22 / 0 / 2 |
| 0.35 m | 24 / 0 / 0 | 24 / 0 / 0 | 24 / 1 / 7 |

Start distance, straight target, limit 1.5 (6 / 10 / 15 / 20 m), break-offs of
24: tag 0.20: 22 / 23 / 24 / 24; tag 0.35: 24 / 22 / 24 / 24. No contacts.
At 6 m the tag began below the picture in most frames before the first decode.

Stricter decode rule (post hoc; nothing under 20 px, half at 30 px), limit 1.5:

| Tag | Hover: acquired / break-off | Straight: acquired / break-off | Range at first decode |
|---|---|---|---|
| 0.10 m | 3 / 3 | 1 / 1 | 4 m |
| 0.16 m | 9 / 9 | 6 / 5 | 6 to 7 m |
| 0.20 m | 11 / 11 | 9 / 9 | 8 m |
| 0.35 m | 22 / 16 | 24 / 24 | 13 to 14 m |

Fast rung (limit typed 16 m/s; the break-off profile holds the chase to 10 m/s):

| Target | Tag | Break-off | Time from GO, median (worst) | Distance flown in 60 s, median (worst) | Trigger range | Worst separation after |
|---|---|---|---|---|---|---|
| 5 m/s | 0.20 m | 24 of 24 | 5.1 s (12.3) | 50 m (91) | 5.8 to 7.7 m | 1.26 m |
| 5 m/s | 0.35 m | 24 of 24 | 4.8 s (6.5) | 48 m (62) | 6.1 to 8.0 m | 1.84 m |
| 9 m/s | 0.20 m | 24 of 24 | 32.7 s (44.4) | 327 m (438) | 2.1 to 2.2 m | 1.55 m |
| 9 m/s | 0.35 m | 24 of 24 | 36.2 s (47.6) | 357 m (466) | 2.1 to 2.1 m | 1.51 m |

Camera tilt told wrong (post hoc; true 20, told 0 / 17 / 23; straight, tag 0.20,
limit 1.5): 24 / 24 / 0 in all three, worst separation 1.61 / 1.71 / 1.77 m. My
expectation that "told 0" would lose the tag was wrong in the sim. Pass the real
angle anyway: the flight program's `--mount-tilt-deg` defaults to 0.

Where the tag was when it was big enough to decode but not in the picture: off
the BOTTOM in nearly every case (start 2 m above: 100 % of such frames; start
distance 6 m: 61 to 81 %), never off the top on straight or hover targets. On the
loop it was mostly off the side.

## Answers

**Settings for the first 2 mph run.**
- Speed limit **1.5 m/s** (`--handover-max-speed-ms 1.5`). 0.9 never closes on a
  0.9 m/s target; 2.5 produced contacts.
- Target on a **long straight leg**, flying away, not the backyard loop and not
  hovering close in front. A hovering target closes at the full limit and was the
  case with the most contacts.
- Start **level with the target or up to 2 m below it, never above**. This is the
  single most important setting. Height above ground is the pilot's to judge; the
  floor was 2 m and the lowest any run reached was 3.1 m from a 4 to 8 m start.
- Start **10 to 20 m behind**, nose within about 30 degrees of the target. At 6 m
  the tag starts under the picture.
- Tag: **the 0.35 m placard**. The registered tie-break picks 0.20 m, on a
  0.04 m difference in worst-case separation, but the sim's acquisition of the
  small tags rests on decodes at 10 to 12 px. Under the stricter rule only 0.35 m
  is acquired reliably. A larger tag does leave the picture earlier when the
  interceptor is above the target, which is one more reason to start below.
- Tell the program the camera angle (`--mount-tilt-deg 20`).

**Does holding height until the camera has the tag cost acquisitions?** Not in
this sim: 24 of 24 either way. It cost 3 to 4 break-offs of 24 on the hover
target. The honest limit: the sim decodes the tag almost at once, so the hold
lasted under 2 s. If the real detector first decodes at 8 m instead of 14, the
interceptor flies the first several metres at whatever height the pilot left it,
which again says start level or below.

**Does the break-off leave margin at slow closure under the limiter's caps?**
When it fires, yes: it fired at 2.1 to 2.4 m and the aircraft stayed 1.7 m or
more away on the straight target at 1.5 m/s (1.1 m or more on the hover target).
The evade is slow under the caps but the closing speed is slower. The weakness is
that it does not always fire: its trigger distance (about 2.25 m, set by the
1.5 m/s closing floor) is the same distance at which a 0.35 m tag leaves the
bottom of a 20-degree-up camera, and with no decodes there is no break-off.

**Does the full-speed rung look sane?** On the registered criterion it passes.
In practice it is not a rung to climb on Saturday on this evidence:
- "full speed" is 10 m/s under the break-off profile, so a 9 m/s target is caught
  at 1 m/s of closure after half a minute and a third of a kilometre, dead
  straight, ending at or past the geofence, on a pack that has been lasting 2.5
  to 4 minutes;
- at the trigger the closing speed is about 1 m/s, the same slow regime as rung
  1, so it tests nothing new about the break-off at speed. The 5 m/s target is
  the case that does (closing 4.8 m/s, trigger at 6 to 8 m, worst 1.26 m);
- no own-state error, no wind in those cells, and the target's tag is assumed
  readable while it flies nose-down at 9 m/s.

## Divergences from the real world that bear on these numbers

- **Tag rendering.** Nothing is rendered. The tag is a square scored by size,
  incidence and blur. Decodes at 10 to 12 px are an artefact of the size curve's
  tail. At under 2 m a real tag also defocuses and overfills; the sim only knows
  "in the picture or not".
- **Target attitude at low speed.** Derived from its path; a real small quad at
  1 m/s in wind yaws and wobbles on its own, and a hovering one has no reason to
  keep its tail toward the interceptor. The hover cells assume the tag faces the
  interceptor's side within 20 degrees.
- **GPS error.** A fixed offset drawn once per run. The real error wanders, and
  the two receivers' height disagreed by 2.4 to 4.8 m on the ground. The loop
  cells use the target's true velocity at GO plus error; a real 5 s fit of a
  turning target is worse.
- **Own state.** Perfect heading, attitude and velocity. The 26 November logs show
  heading swings of 11 to 29 degrees on the poor-GPS flights.
- **Camera.** fx 800 px at 1280 x 800 is arithmetic from the lens and pixel size.
  No calibration exists. A shorter or longer real focal length moves every range
  in this note in proportion.
- **Vehicle.** The Gazebo x500 fit with the board's envelope. Only its velocity
  lag has been compared with a log of the real aircraft.
- **Separation** is centre to centre; the aircraft are about 0.3 m across, and
  0.35 m is used as contact.
