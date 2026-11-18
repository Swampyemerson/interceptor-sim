# Arrival search at a long cue (registered before flying)

Builder ask 2026-11-18: look for guidance improvements for the 500 m case. At a
500 m cue the approach dash takes about 25 s; a 10 % error in the believed
target speed becomes about 30 m of along-track error on arrival; with all errors
on the tag never decodes in 61 % of runs and the dash kills 13 to 17 of 50
(`isim/specs/approach_dash_drag_prereg_2026-11-18.md`, Stage 2 and 2b: the
speed-belief error alone does it).

**isim ranks, Gazebo and the airframe decide.** Nothing here is adopted.

Code: `flight/pursuit_terminal.py` (`arrival_search`, default OFF).
Runner: `scripts/arrival_search_sweep.py`. Rows: `runs/arrival_search_20261118/`.

## What was built

Phase A today parks 8 m behind the BELIEVED target and looks at it. With the
flag on, Phase A searches the along-track interval the target can be in instead.

- **Interval.** Half-width `h(t) = k * (sigma_pos0 + sigma_speed * believed
  speed * time since GO)`, with `k` 2, `sigma_pos0` 3 m and `sigma_speed` a
  DECLARED fraction (10 % unless the arm says otherwise). The two sigmas are
  givens describing the cue: configuration constants, never read from anything
  live. Guidance is not told which error level is really flying.
- **Legs.** `fwd`: run up the believed track 8 m/s faster than the believed
  target, nose along the track, until `h` ahead of the believed target. `back`:
  fall back 6 m/s slower than the believed target until `8 m + h` behind it.
  Repeat until the tag decodes; the ordinary acquisition gate hands over to
  Phase B, which is unchanged.
- **Two starts.** `stern`: fly straight to `8 m + h` behind the believed target
  and start with `fwd`, so every target inside the interval is met from astern
  in one pass. `believed`: fly to the legacy point (8 m behind), wait 1 s, then
  `back`, then `fwd`: identical to today when the cue is good, at the price of a
  wasted leg when the target is ahead.
- **Short-track fallback.** When Phase B loses the target after fewer than 10
  filter updates, the new belief keeps the last measured position but takes its
  velocity from the pre-flight belief, not from the filter.

## Instrument check (done before registering; not a result)

`--stage check`: the runner's baseline arm reproduces 48 of 48 stored rows of
Stage 2b (straight target, three error levels, two geometries) and Stage 2
(weaving target) exactly: miss, time of closest approach, decode count. It was
run with the modified `flight/pursuit_terminal.py`, so it also shows the change
is inert with the flag off.

## What development probes showed (seeds 1000 to 1011, never the test seeds)

Three cells of 12 runs each, used to make the mechanism work. Two things had to
change before the search could be measured at all, and both are declared here
because they shape the design of the test:

1. **The wrapper's lost-target clock ends the search.**
   `MissionConfig.engage_lost_target_s` (2.0 s as flown) starts on the first
   decode, whatever the terminal is doing. One or two stray decodes at 25 to
   35 m on the way in armed it and the engagement ended before the search
   began; at 30 s it still cut one search 2 s before it re-acquired. The search
   arms therefore run with the clock set to the whole 120 s window. That clock
   lives in `flight/deploy/real_flight.py`, which this work does not touch: the
   test changes a configuration value, and the wrapper rule the search really
   needs (do not run the clock while the terminal is searching) is a separate
   change for the head and the builder.
2. **A filter fed five long-range fixes hands back a bad velocity** (14.4 m/s
   and 10 degrees off on a 12 m/s target in one trace), and a search run along
   it walks away from the target. Hence the short-track fallback above.

On those 36 probe runs, with both in place: baseline 5, 4 and 12 kills of 12
(near 10 %, demo 10 %, near 0 %); `stern` 12, 11 and 11; `believed` 11, 11 and 12.
These guided the predictions below. They are not the result.

## AMENDMENT 1 (2026-11-18, before any registered row existed)

The registered run was launched once at commit `4654843` and stopped a few
minutes later, before it had written a row (`main.log` empty, no `main.json`),
because the feature's own unit tests turned up a third defect:

3. **A fallback pins the belief on a stale decode.** The acquisition list is
   pruned to its 0.3 s window only when a new decode arrives. After a Phase B
   dropout it can still hold two fixes from before the dropout, so the next tick
   restarts Phase B from the last of them, the dropout clock (still at the old
   decode time) falls straight back to Phase A, and this repeats every tick. The
   belief stays pinned at the stale relative offset and no Phase A law runs at
   all, search or legacy. One probe run sat in `approach` for 100 s this way.
   This is in the DEFAULT path too: it is likely part of why the September
   abort-timer A/B found the fallback "never produces a second converging pass".
   - **Search on:** the list is now pruned by time every tick.
   - **Default path:** left exactly as it is, and reported. A strict-xfail unit
     test (`test_legacy_fallback_is_pinned_by_stale_decodes`) asserts the
     correct behaviour and fails on the default path today; it will pass, and
     demand its marker be removed, when someone fixes it there.
   - **Consequence for the control.** `base_T` runs the default path under a
     long clock, so its fallback is the pinned one. That is the as-built
     behaviour and the honest control for "the clock alone", but part of any
     search gain could then be this repair and not the sweep. So the repair is
     also its own switch (`stale_decode_guard`, default OFF) and the matrix
     gains a seventh arm, **`base_T_guard`**: search off, guard on, long clock.
     Prediction 8: it gains 2 to 8 kills of 50 over `base_T` at 10 % (runs that
     saw the target briefly and lost it), loses none at 0 %, and stays far
     below the search arms because it cannot touch a run that never decodes.

Nothing else changed: same cells, seeds, predictions 1 to 7 and thresholds;
seven arms, 4200 runs.
Probe numbers with all three repairs in place (seeds 1000 to 1011): baseline 5,
4 and 12 kills of 12 (near 10 %, demo 10 %, near 0 %); `base_T` 5, 6 and 12;
`stern` 12, 12 and 11; `believed` 11, 11 and 12. The figures quoted above this
amendment were from the build before the third repair.

## Config (registered)

- **Setup:** Stage 2 exactly. Real flight code (port arm), profile 36 / 4, plant
  R with the tilt-saturation repair, no body drag, rear 0.5 m tag, fx 540.3,
  pose-range on, decode thinning 0.096 s, all errors on (`Scatter()`), 120 s
  window, target 12 m/s.
- **Cells (12):** target straight or weaving; geometry near (6.5 / 500 m) or
  demo (150 / 477 m); TRUE speed-belief error 10 %, 4 % or 0 % (one sigma).
- **Arms (7; the seventh added by amendment 1):**
  - `base`: as flown (search off, 2.0 s clock). Context.
  - `base_T`: search off, clock at the window. **The control.**
  - `base_T_guard`: `base_T` plus the stale-decode guard, nothing else.
  - `stern`, `believed`: search on, declared sigma 10 %, clock at the window.
  - `stern_decl04`: `stern` with the declared sigma at 4 %.
  - `believed_T2`: `believed` under the flying 2.0 s clock.
- **Seeds:** 0 to 49, paired across arms. 4200 runs.
- **Kill:** closest approach 0.35 m or less. **Time to kill:** first instant
  inside 0.35 m. **Blind pass:** the vehicle came within 3 m of the target
  before it had decoded anything. **Time near:** seconds within 5 m before
  first contact.

## Known asymmetry between arms, declared up front

The long clock gives an arm the whole 120 s window to score; the 2.0 s clock
ends `base` at its first loss. So `base` cannot suffer or enjoy a second
attempt and every long-clock arm can. Kill discordance is therefore paired
against **`base_T`**, which shares the clock, and `base` is shown beside it.
The sim also flies on after a first contact, which a real engagement does not;
times are measured to FIRST contact and kills are also reported with a 45 s
bound so a slow kill cannot hide in the long window.

Failure modes only the search arms can have: a sweep that carries the vehicle
past the target without a decode; a longer engagement at speed; a re-search
along a wrong track after a brief acquisition; a search that leaves the
searched interval behind when the declared sigma is too small.

## Predictions (written before the registered run)

1. **10 % error, straight target.** `stern` and `believed` each kill 40 or more
   of 50 in both geometries (`base` 16 and 17), with 3 or fewer no-decode runs.
2. **4 % error, straight target.** 43 or more of 50 (`base` 27 and 31).
3. **0 % error (the control that matters).** `believed` loses at most 1 kill of
   50 net against `base_T` in every cell and its median time to kill is within
   2 s of `base_T`. `stern` loses at most 4 of 50 net and kills 6 to 10 s later.
4. **Weaving target.** Both search arms gain at 10 % but land lower than the
   straight cells: 25 or more of 50 (`base` 6 to 8). Their remaining misses are
   narrow terminal misses, not no-decode runs.
5. **The clock alone is not the fix.** `base_T` is within 3 of 50 of `base` in
   every cell at 10 % error. `believed_T2` lands between `base` and `believed`
   and keeps more than half of `believed`'s gain; I am least sure of this one.
6. **Declaring too small a sigma.** `stern_decl04` at a true 10 % kills 3 to 10
   fewer of 50 than `stern` and still 30 or more. At a true 4 % and 0 % it
   kills at least 3 s sooner than `stern`.
7. **Blind passes** do not rise by more than 3 of 50 in any cell against
   `base_T`.

## What counts, and what a null means

- **Candidate for a Gazebo rung:** an arm that, against `base_T` with a straight
  target, gains 15 or more kills of 50 net at 10 % AND 8 or more at 4 % in both
  geometries, AND loses no more than 2 of 50 net in either 0 % cell. If both
  arms qualify, the one with the lower median time to kill at 0 % goes first,
  unless the other has 5 or more extra kills summed over the two 10 % cells.
- **Null:** neither arm gains 8 of 50 at 10 % in both geometries. Then an
  along-track search as built does not recover the lost acquisitions and the
  lever is elsewhere: cue velocity quality, a tag readable from more aspects
  (or the markerless seeker), or a search across the track.
- **Not decided here:** the lost-target clock rule, the declared sigmas' real
  values (they come from a measured cue on a moving target, which does not
  exist yet), and anything about drag or the flight envelope.

## Known limits

- The plant is the x500 fit with a raised tilt limit; above 16 to 18 m/s it is
  extrapolated, as the parent spec says.
- The search is along the track only. A cross-track belief error larger than
  the decode range (about 12 m for this tag and lens) is not searched.
- The engagement window (120 s) is longer than a real flight's time bound
  (`engage_max_s` is 30 s in the flight program's defaults).
- The sweep speeds are sized by hand from the brake schedule and the decode
  range; they were not swept.

## RESULT (2026-11-18; 4200 runs, 84 cells of 50, no cell short)

Rows: `runs/arrival_search_20261118/main.json`. Tables:
`python scripts/arrival_search_report.py` (output saved as `report.txt`).
Ran on the dev laptop. Fast sim only.

**Kills of 50** (in brackets: first contact inside 45 s)

| Target, geometry, true speed error | `base` | `base_T` | `base_T_guard` | `stern` | `believed` | `stern_decl04` | `believed_T2` |
|---|---|---|---|---|---|---|---|
| straight, near, 10 % | 16 | 17 | 17 | 46 [46] | 48 [38] | 43 [39] | 43 [38] |
| straight, near, 4 % | 27 | 29 | 28 | 50 [50] | 49 [45] | 49 [46] | 44 [43] |
| straight, near, 0 % | 50 | 50 | 50 | 49 [49] | 50 [48] | 50 [49] | 48 [48] |
| straight, demo, 10 % | 17 | 19 | 18 | 47 [46] | 49 [40] | 42 [38] | 41 [38] |
| straight, demo, 4 % | 31 | 31 | 31 | 50 [50] | 49 [44] | 50 [46] | 44 [44] |
| straight, demo, 0 % | 45 | 45 | 45 | 49 [49] | 50 [48] | 50 [49] | 48 [48] |
| weave, near, 10 % | 8 | 10 | 11 | 15 | 18 | 21 | 13 |
| weave, near, 4 % | 10 | 11 | 11 | 21 | 16 | 23 | 13 |
| weave, near, 0 % | 21 | 22 | 22 | 22 | 19 | 24 | 16 |
| weave, demo, 10 % | 6 | 9 | 8 | 20 | 18 | 13 | 12 |
| weave, demo, 4 % | 14 | 15 | 15 | 22 | 22 | 25 | 17 |
| weave, demo, 0 % | 16 | 18 | 18 | 27 | 17 | 14 | 14 |

**Paired against `base_T`, straight target (kills lost / gained):** `stern`
0 / 29, 0 / 28 at 10 %; 0 / 21, 0 / 19 at 4 %; 1 / 0, 1 / 5 at 0 %. `believed`
0 / 31, 0 / 30 at 10 %; 0 / 20, 0 / 18 at 4 %; 0 / 0, 0 / 5 at 0 %.

**Runs that never decode, straight target at 10 %:** 31 and 30 of 50 for the
three baselines; 2 and 2 for `stern`, 1 and 1 for `believed`, 7 and 8 for
`stern_decl04`.

**Median time to first contact, straight target:** `base_T` 25.4 to 28.2 s.
`stern` 33.0 to 33.5 s at every error level. `believed` 25.6 and 26.5 s at 0 %,
28.5 and 28.9 s at 4 %, 30.8 s at 10 %, with a long tail at 10 %: 10 and 9 of
its kills come after 45 s and 7 and 7 after 60 s (the latest at 81 s). `stern`
has one kill after 45 s in those two cells and none after 46 s.

### Predictions

1. **10 %, straight: held.** 46, 47 (`stern`) and 48, 49 (`believed`) against 40.
2. **4 %, straight: held.** 49 or 50 in all four cells against 43.
3. **0 % control: held.** `believed` loses nothing and is 0.2 and 0.9 s slower.
   `stern` nets -1 and +4 and is 7.9 and 7.4 s slower. Its two lost kills are
   misses of 0.37 and 0.40 m.
4. **Weaving target: failed.** I predicted 25 or more of 50 at 10 %; the search
   arms got 15 to 20. They do gain (+5 to +11 net), and almost every run now
   decodes, but against a weaving target the kills are decided in the terminal:
   the baselines only manage 16 to 22 of 50 with a perfect speed belief, and
   paired seeds flip both ways (8 to 12 lost against 5 to 17 gained at 0 %).
   The search does not help a target the terminal cannot hit.
5. **The clock alone is not the fix: held.** `base_T` is 1 to 3 above `base`.
   `believed_T2` keeps most of the gain (43 and 41 against 48 and 49) but loses
   5 to 8 kills per straight cell to the 2.0 s clock; pooled over all cells, 243
   of its 600 runs decode, do not kill and end on that clock, 37 of them with 5
   or fewer decodes in the whole run (`base`: 134 and 13).
6. **Too small a declared sigma: held on kills, missed on time.** `stern_decl04`
   at a true 10 % kills 3 and 5 fewer than `stern` and leaves 7 and 8 runs
   undecoded. At 4 % and 0 % it is 1.5 to 2.9 s quicker than `stern`, not the 3 s
   I predicted.
7. **Blind passes: held for `stern`, missed for `believed`.** Against `base_T`,
   `believed` has 4 and 3 more at 10 % straight (9 and 12 of 50) and 5 and 4
   more against a weaving target; `stern` has 2 more and 4 fewer. With a
   straight target nearly all of them still end as kills (9 of 9, 11 of 12).
8. **The stale-decode guard alone: failed.** I predicted +2 to +8; it changes
   nothing measurable (0, -1, +1, -1 at 10 %). So the search arms' gain is the
   sweep. The guard still matters inside the search (without it a fallback
   pins the search), but as a repair to the default path it buys nothing here.

### Against the registered criterion

Both `stern` and `believed` meet it (so do `stern_decl04` and `believed_T2`).
By the registered tie-break `believed` goes first: it costs nothing when the
cue is good (25.6 against 33.3 s at 0 %), and `stern` is not 5 kills ahead at
10 % (93 against 97 over the two cells). Recorded beside that, because the
tie-break did not ask: inside a 45 s bound `stern` has 92 kills at 10 % and
`believed` 78. Which matters more depends on the engagement time limit, which
is 30 s in the flight program today; under 30 s neither arm's median kill at
10 % fits.

### Failure modes only the search arms have

- **Longer engagements.** +5 s (`believed`) to +8 s (`stern`) to the median
  kill at 10 %; `believed`'s slow tail reaches 60 to 81 s when its first leg
  goes the wrong way. Up to 5 to 7 legs flown in the worst runs.
- **Blind passes** on the `believed` back leg: the target overtakes from behind
  before it can be seen. Counts above.
- **A wrong declared sigma** leaves targets outside the first pass (7 and 8
  undecoded of 50 with 4 % declared against 10 % true).
- **Time within 5 m before contact** is unchanged for a straight target (2 s
  median) and longer against a weaving one (3 to 5 s median, 10 to 17 s at the
  90th percentile, against 0 to 4 and 5 to 22 for the baselines).
- Not seen: any straight-target cell where a search arm loses more than 1 kill
  against `base_T`.

### What this result does not show

- **Optics.** fx 540.3 (a 100 degree lens), a 0.5 m tag and zero camera
  up-tilt: the October demo setup, not the flying camera or tag. The decode
  range (about 12 m at even odds, useful to about 20 m) sets the sweep speed
  and every time above.
- **The flight controller's speed limit.** The forward leg flies the target's
  speed plus 8 m/s, about 20 m/s here. The interceptor's board is limited to
  12 m/s today (read 2026-11-18), which is the target's speed in this test: the
  forward leg cannot close on it. Only the back leg works without speed margin.
- **The wrapper clock.** Five of the seven arms run with the lost-target clock
  set to the window. The search as built needs the wrapper not to run that
  clock while the terminal is searching; that rule is not written.
- **Across-track error.** Pooled, 2 runs of 600 (`stern`) end more than 12 m
  off to the side after decoding; the heading error here is 2 degrees. A worse
  heading would need a search across the track, which this is not.
- **Drag, a real plant, a real camera.** isim, extrapolated plant, no body drag.
- **The independent geometry check** (same day, another agent) puts the
  along-track error at arrival at about 22 m one sigma, not the 30 m quoted at
  the top of this file (the vehicle parks 18 s after GO, not 25), and counts 25
  of the 30 no-decode runs with the target behind and 5 to 6 with it far ahead.
  Neither changes the design: the search covers both sides.
