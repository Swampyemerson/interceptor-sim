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
