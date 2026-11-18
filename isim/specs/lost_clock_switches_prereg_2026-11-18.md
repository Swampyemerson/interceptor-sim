# Lost-target clock switches: one fast-sim cell (registered before flying)

Builder ruling 2026-11-18 on the lost-target rule: "whatever gets highest
intercept rate, I'll be supervising so I can disengage if it gets ugly". Three
switches were built in the flight program (`flight/deploy/real_flight.py`,
all default off):

- **(a) needs track:** a decode does not start the clock until the terminal
  has had a track.
- **(c) off in search:** the clock does not run while the terminal is in its
  arrival search.
- **(b) far-loss clock:** 6.0 s instead of 2.0 s when the target was last
  estimated beyond the 3 m miss range.

`--fly` takes all three from one preset (`FLY_PRESET`). This cell asks whether
that preset does what the arrival-search study assumed when it ran its search
arms under a 120 s clock, that is, with the clock effectively removed.

Script: `scripts/lost_clock_switch_cell.py`. Rows:
`runs/lost_clock_switches_20261118/cell.json`.

## Config

One cell of the drag screen's Stage 2, unchanged: straight target, near
geometry (cross 6.5 m, lead 500 m), profile 36 / 4, plant R with the height
repair on, no drag, all errors on (speed-belief error 10 % one sigma), seeds
0 to 49, October's optics (fx 540, 0.5 m tag, no camera tilt). The real
flight code runs in the loop; the switches are set through `MissionConfig`.

| Arm | Arrival search | Lost-target rule |
|---|---|---|
| A0 | off | 2.0 s (as flown; must reproduce Stage 2's stored 16 of 50) |
| A1 | off | (a) + (b) |
| A2 | stern | 2.0 s |
| A3 | stern | 120 s clock (the arrival-search study's arm) |
| A4 | stern | (a) + (c) + (b): the `--fly` preset |
| A5 | stern | (a) + (c), no far-loss clock |

## Predictions (written before any run)

1. A0 reproduces the stored rows exactly (instrument check).
2. A1 is within 2 kills of A0 either way. Without the search a run that the
   clock stops losing still has to find the target on belief alone.
3. A2 is well below A3: the 2.0 s clock cuts the search short. I expect A2 at
   least 10 kills under A3.
4. **The one that matters:** A4 is within 2 kills of A3 and never more than 1
   below A2 on paired seeds. The preset gives the search the room the 120 s
   clock gave it.
5. A5 lands between A2 and A4: the far-loss clock still matters after the
   first track is lost while the target is ahead.

## What a null means

If A4 is no better than A2, the switches do not protect the search as built
and the preset should not be the flying default until that is understood. If
A4 is more than 2 kills under A3, something other than these three rules is
ending engagements under the preset, and the rows' own end reasons say what.

## Limits

One cell, 50 seeds, the fast sim on an extrapolated vehicle fit, October's
optics. It ranks; the flight-hardware sims and a Gazebo rung decide. Nothing
is adopted on this cell: the head sets `FLY_PRESET` from the fuller results.
