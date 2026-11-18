# Guidance reliability review: approach dash on (fork, 2026-11-18)

Read-only review of `flight/pursuit_terminal.py`, the ENGAGE / BREAKOFF / SAFE logic in
`flight/deploy/real_flight.py`, and what they call. No repository file was changed.
Evidence: the head's `runs/approach_dash_drag_20261118/stage1.json` (864 runs) and
`stage2.json` (1200 runs, finished while I worked), plus about 320 fast-sim runs of my own
from scratch scripts (listed at the end; more than the few dozen asked for, because the
fair tests needed whole paired cells). Everything here is fast-sim (isim) evidence on an
extrapolated x500 fit: it ranks and locates, it does not decide.

## 1. The mechanism behind the 13 "target lost after the merge" misses

**Named: lean outruns the camera's vertical field of view during the catch-up, and the
wrapper's 2.0 s lost-target clock ends the engagement while the track is healthy and the
vehicle is closing.**

Traced runs (`fork_trace.py`): `demo|v20a3|D2|R` seed 3 and `near|v36a3|D2|X` seed 3,
against the no-drag twin `demo|v20a3|D0|R` seed 3.

1. In the near and demo geometries the target approaches the vehicle. The tag faces
   rearward, so nothing is decodable (incidence 150 to 180 degrees) until the target has
   passed. The pass happens at 0.75 to 3.5 m with 6 to 8 m/s of relative speed, before any
   detection. That is where the "closest approach at or before first detection" comes from.
2. This is by construction, not a drag effect. Phase A aims 8 m behind the target and the
   brake schedule brings relative speed to its floor at that point
   (`pursuit_terminal.py:1004-1011`, `:1032-1046`), so when the target itself goes by, 8 m
   early, the schedule still allows `sqrt(1.5^2 + 2*a*(8 - lead*closing))`, about 5 to
   7 m/s. The no-drag twin passes at 3.49 m and 6 m/s too.
3. First decode comes 0 to 1.2 s after the pass, at 3.6 to 6.4 m and opening at 3 m/s.
   Phase B starts and commands the full 16 m/s cap (`:866-889`, `:963-969`) to chase a
   12 m/s target from a 6 to 9 m/s start.
4. The body-fixed camera has no up-tilt in this setup (`isim/scenario.py:343`,
   `cam_tilt_up_deg = 0.0`) and a vertical half field of view of 36.5 degrees at fx 540.
   Accelerating needs 36 to 38 degrees of lean without drag, and the tag leaves the top of
   the frame for about 1.0 s. With doubled drag, holding 14 m/s already takes about 30
   degrees, the lean sits at 38 to 44 degrees (plant X sits on its 37.7 degree limit), and
   the tag stays out of frame for more than 2 s.
5. At 2.0 s without a decode the wrapper leaves ENGAGE (`real_flight.py:1684-1713`,
   `engage_lost_target_s` at `:269`). At that instant, in both traced failures: true range
   5.1 to 5.6 m, closing at 2.2 to 2.5 m/s, Kalman range 5.7 to 6.2 m, Kalman target speed
   12.2 to 12.3 against 12.0 true. BREAKOFF commands zero horizontal speed
   (`:1725-1727`). As the vehicle levels out the tag comes back into view at 3.8 to 5.0 m
   within about half a second, and decodes resume, but BREAKOFF cannot return to ENGAGE
   (`:1729-1735`).

Whether a given seed dies is decode luck inside that window, which is why 12 or 13 of 16
seeds in the same cell pass.

**Does the wrapper clock pre-empt the terminal's own fallback? Yes.** The terminal's
Phase B to A fallback needs a dropout longer than `fallback_s` = 3.0 s and range of 3 m or
more (`pursuit_terminal.py:776-783`). The wrapper's clock is 2.0 s, is checked in the same
tick after `guidance.step`, and once it fires the terminal is never stepped again. With
the defaults, `fallback_s`, `no_fallback_range_m`, `fallback_speed_cap_mult`,
`fallback_speed_floor_ms` and `_sane_fallback_vel` (`:1149-1160`) cannot run in flight. The
earlier abort-timer null (2 / 4 / 10 / 30 s) was earned at the short-range 9 m/s point,
where losses are inside 3 m; it does not cover a loss at 5 to 7 m with a closing track.

**Checked on the failing seeds and on whole cells** (`fork_exp.py`, `fork_exp2.py`):

| Change (truth side or scratch monkeypatch) | The 13 failing seeds | Whole cells, paired with stage 1 |
|---|---|---|
| none (rerun) | 0 / 13, identical | 50 / 64 |
| camera up-tilt 10 degrees | 13 / 13, median 0.076 m | 64 / 64, fixed 14, broke 0 |
| crude lean governor in Phase B | 4 / 13, no target-lost exits, median 0.37 m | not run |
| plant integrator clamp 3 to 12 m/s2 | 10 / 13 | not run |

The 13-seed column is selection-biased: these seeds were picked because they failed, and
any change re-rolls the decode luck. About three in four would pass on a re-roll alone, so
10 / 13 for the integrator clamp is no evidence, and 4 / 13 for the governor is worse than
chance. The whole-cell column is the fair one. It was run for the tilt only.

**With the flying lens it bites at expected drag, not only doubled** (`fork_exp4.py`,
demo, 36 / 4, plant R, fx 933, 0.30 m tag, 16 seeds, no scatter):

| Drag | Tilt 0 | Tilt 10 degrees |
|---|---|---|
| D0 | 16 / 16 | 16 / 16 |
| D1 (expected) | 7 / 16 (7 target-lost, median 0.445 m) | 16 / 16 (median 0.064 m) |

fx 933 and the 0.30 m tag are my reading of the flying configuration (ADR-0126's lens, the
tag size quoted in ADR-0132); the real intrinsics come from the calibration that is still
owed. One cell, 16 seeds.

## 2. Failure modes found by reading the code

Ordered by how sure I am that they cost intercepts.

**F1. Any single detection during Phase A starts the 2.0 s clock, and Phase A has no
track to lose.** `real_flight.py:1608` sets `_last_det_t` on every detection in ENGAGE;
`:1684-1685` then ends the engagement 2.0 s after the last one. In pursuit mode the whole
transit runs inside ENGAGE and is meant to fly on belief with no decodes. Demonstrated
(`fork_exp2.py`): one injected detection at t = 10 s in an otherwise clean kill gives
`BREAKOFF@12.0: target_lost(2.0s)`. In the head's stage 2 this is a measured loss: 63 of
1200 runs had one to three decodes in total, all 63 exit `target_lost > breakoff_complete`,
none is a kill. Among straight-target runs that detected but missed, median first-detection
range is 30.6 m and median decode count is 3.

**F2. Sparse decodes are thrown away in Phase A.** Phase B needs two decodes inside 0.3 s
(`pursuit_terminal.py:130-131`, `:762-770`). A lone decode is stored and discarded; it does
not correct the belief. At 20 to 30 m the tag is 9 to 14 px and decodes arrive one at a
time, so the vehicle keeps flying to the wrong rendezvous while looking at the target.

**F3. The flight default engage window is 30 s, the sim uses 120 s, and the launch check
uses the commanded speed.** `real_flight.py:3258-3260` sets 30 s for pursuit;
`isim/scenario.py:800` overrides it to the 120 s window. The launch warning
(`real_flight.py:3366-3369`) estimates transit as range over `dash_v_max_ms`, so a vehicle
that cannot reach the cap (drag, or the stock 12 m/s limit on the real board) passes the
check and times out mid-approach (`:1715`). In stage 1, 218 of 864 runs have their closest
approach more than 30 s after GO, 216 of them scored as kills (every tail cell with doubled
drag, 82 of 96 at expected drag). In stage 2, 69 of 245 kills come later than GO + 30 s.

**F4. Lean against the camera's vertical field of view is unmanaged.** Nothing in the
terminal knows the camera is body-fixed. Phase B asks for the full cap regardless of the
lean that costs (`pursuit_terminal.py:963-969`), and the flight program's
`--mount-tilt-deg` defaults to 0.0 (`real_flight.py:3700`). The earlier registered
chase-tilt study already recommended a 10 degree bracket for a different reason (target
above); section 1 says the same bracket is what keeps the tag in frame under drag.

**F5. Phase A has no arrival behaviour when the target is not where the belief says.**
`_phase_a_cmd` (`pursuit_terminal.py:927-946`) flies formation on the believed point
forever with the camera on the believed bearing. In stage 2, 732 of 1200 runs never detect
the target; their closest approach is 12.5 m median (4.5 to 35 m, 10th to 90th
percentile). This is the largest loss in the data by a wide margin. Caution in section 4:
the scatter model that produced it was sized for short-range engagements.

**F6. The relative belief is dead-reckoned from own velocity for the whole transit.**
`pursuit_terminal.py:758` integrates `v_track - own_vel` every tick. That was chosen for
engagements of 2 to 13 s. Over a 25 to 45 s transit a 0.15 m/s velocity bias is 4 to 7 m
of belief error, on top of the cue's own velocity error times the same time. Drag
lengthens the transit, so it feeds F5.

**F7. The Phase B cap leaves little overtake margin, and drag removes what is left.**
`v_max_ms` = 16 with adaptive speed off (`:126`, `:866-889`) is 4 m/s over a 12 m/s
target on paper, 2 to 2.5 m/s achieved in the traces. The registered adaptive-speed study
already showed the legacy cap failing at 15 m/s targets. Raising the cap does not help when
the plant is the limit, and it raises the lean.

**F8. Yaw swings through up to 180 degrees at the merge.** Phase A yaws at the believed
target (`:945`). In the head-on trace the yaw error reaches 109 degrees just after the
pass and the tag is out of frame for about 1 s, during which the range opens from 0.9 to
6.4 m. Guidance slews at 180 degrees per second (`:163`); the fitted plant allows 120.

**F9. Per-axis slew, not a combined one.** `_slew_combined` (`:415-421`) says "ONE
combined-norm slew budget" and clips each axis separately. The budget is up to 1.7 times
the stated value on a diagonal and the command does not move straight toward its target
during a reversal. Small; a docstring and behaviour mismatch.

**F10. BREAKOFF is a fly-by behaviour in a chase.** Zero horizontal speed and a 1 m/s
climb for 1.5 s (`real_flight.py:335-336`, `:1725-1735`), no way back to ENGAGE. In both
traced failures the tag was back in view 0.5 s into BREAKOFF. Changing this touches the
builder's 2026-09-22 ruling on missed intercepts, so it is his call, not a default.

**Constants earned at 9 to 12 m/s and short range that this regime re-uses without
re-earning:** `engage_lost_target_s` 2.0, `pursuit_miss_range_m` 3.0,
`no_fallback_range_m` 3.0, `acquire_n` / `acquire_window_s`, `d_behind_m` 8,
`v_close_max_ms` 6, `v_max_ms` 16, `brake_accel_ms2` and `brake_lead_s` (sized to the
x500's 5.94 m/s2 braking and symmetric, while drag helps slowing and opposes speeding back
up), `kf_q_accel_ms2` 1.0, the choice to carry no own position, and the skipped Phase-A
search sweeps (`:84-87`).

## 3. Ranked candidates

Gain is my estimate from the stage 1 and stage 2 counts. All are config-gated and default
off in this repo's style. None is adopted by this review.

**1. Arm the lost-target clock only once the terminal has a track, and let one decode
correct the Phase A belief (F1 + F2).** Gain: up to about 5 points of total kill rate in
stage 2 conditions. Risk: low for an AprilTag seeker; with the neural seeker a false
positive would drag the belief, so that seeker needs a plausibility gate.

```python
# real_flight.py, MissionConfig
pursuit_lost_clock_needs_track: bool = False
# _step_engage, after the detection bookkeeping (~1609)
if cfg.pursuit_mode and cfg.pursuit_lost_clock_needs_track \
        and tel is not None and getattr(tel, "phase", None) == "A":
    self._last_det_t = None          # Phase A flies on belief; nothing to lose yet

# pursuit_terminal.py, PursuitTerminalConfig
phase_a_reseed: bool = False
# step(), inside `if det_box_xywh is not None:` (~767)
if cfg.phase_a_reseed and self._phase == "A":
    self._r_track = meas_r.copy()    # position only; v_track stays the pre-flight belief
```

Prototype result (`fork_exp3.py`, scratch monkeypatch, paired with the head's stage 2
rows, which my runner reproduces exactly): two cells of 50 seeds, 33 / 100 to 38 / 100
kills, fixed 5 of the 6 detected-but-missed runs (misses of 7 to 22 m became 0.01 to
0.19 m), broke 0. Prediction for a registered run: converts at least half of the
"one to three decodes then lost" runs in every stage 2 cell, no cell loses kills. Null
would mean those decodes come too late or too far for the chase to finish, and the clock
is not what kills them.

**2. Fly and simulate the camera at its real up-tilt, and re-run stages 1 and 2 with the
flying lens (F4).** Not a code change in the terminal: `cam_tilt_up_deg` in the sweep,
`--mount-tilt-deg` in flight, plus a launch print of the lean margin (vertical half field
of view plus tilt) against the expected chase lean. Gain: removes all 13 stage 1 losses in
the four cells tested and takes the flying-lens cell from 7 / 16 to 16 / 16 at expected
drag. Risk: a higher tilt costs the bottom of the frame when the vehicle is level or
braking; the chase-tilt study measured that at 10 degrees as 2 points or less on the
nominal band, at a different operating point. Prediction: with tilt 10 the drag tiers stop
differing in kill rate in stage 1. Null would mean the losses are not a field-of-view
effect and section 1 is wrong.

**3. Stop the wrapper clock pre-empting a healthy, closing track (the head's experiment).**
Either `engage_lost_target_s` above `fallback_s`, or a separate far-loss clock used only
when the last Kalman range is beyond `pursuit_miss_range_m`. Add a launch warning when the
wrapper clock is not longer than the terminal's fallback. Gain: most of the 13 in stage 1; in stage 2 there are 117
`target_lost` exits in all, 63 of them the sparse-decode runs of F1, leaving about 54
that this candidate addresses. Risk: a blind coast on a
constant-velocity model; a weaving target can walk 1 to 3 m in that time, so re-acquisition
is not certain. Ranked under candidate 2 because tilt keeps the measurement instead of
tolerating its loss.

**4. Size the engage window from speed the vehicle can deliver (F3).** Use the smaller of
`dash_v_max_ms` and a declared or read airframe limit in the transit estimate, make the
shortfall raise `engage_max_s` or refuse, and stop `isim/scenario.py:800` hiding the flight
default. Gain: none in the sim as scored (it already runs 120 s), but 216 stage 1 kills
and 69 stage 2 kills are only real if someone passes `--engage-max-s`. Risk: low.

**5. Yaw ahead of the merge (F8).** When the believed target is within about 1 s of
passing abeam from ahead, command yaw along the believed track instead of following the
line of sight through the swing. Gain: about 1 s earlier first decode at about 3 m less
range in head-on merges. Not tested. Risk: low; wrong only if the belief's track direction
is badly off.

**6. An arrival search for Phase A (F5).** A bounded yaw scan about the believed bearing
and an along-track speed sweep once the vehicle has held the believed rendezvous with no
decode for a few seconds. Gain: the 61 % that never detect is the largest loss, so even a
small conversion matters. Risk: highest of the list; search sweeps were measured negative
at short range, a rear-facing tag cannot be seen from ahead, and the error model needs
fixing first (section 4). Not tested.

**7. Shape the merge for approaching targets.** Finish velocity matching before the target
passes instead of 8 m after, so the pass is slow and the catch-up is short. Could be a
smaller `d_behind_m` for front-hemisphere arrivals or a cap measured to the target rather
than the aim point. Not tested; the stern-approach null was at the 9 m/s short-range point
on the hybrid concept and does not cover it.

**8. A lean-aware Phase B governor.** My crude version (hold the horizontal command near
own speed when lean nears 30 degrees) removed the target-lost exits but closed too slowly:
4 / 13, median 0.37 m. One parameter set, no tuning. Low rank unless tilt alone cannot give
the margin with the real lens.

**Probably the x500 fit, not the guidance:**

- The six near-misses just outside 0.35 m and the rise in median miss with drag. The fit's
  velocity integrator is clamped at 3 m/s2 (`ki_vel_horiz_limit`, a default that was not
  in the fit's free list), so it cannot cancel 5 to 7 m/s2 of drag and the vehicle flies a
  permanent 0.6 m/s short. PX4's integrator is not clamped that way. My check was
  inconclusive (3 / 6 and 10 / 13 on failing seeds, about what a re-roll gives). The
  camera tilt fixed all six, which suggests decode starvation at high lean is part of it.
- Height droop of 0.2 to 0.5 m during the saturated-lean chase (soft, unfitted vertical
  loop). It pushes the tag further toward the top edge and makes F4 look slightly worse
  than it is.
- Plant X sitting on a 37.7 degree lean limit: that is the Gazebo x500's limit, not the
  airframe's.

## 4. Things about the instrument the head should know

- **The sweep camera is not the flying camera.** Zero up-tilt, fx 540 and a 0.5 m tag are
  the Gazebo-matched optics from October. Section 1 shows the answer changes with both
  tilt and lens.
- **The stage 2 scatter was sized for 16 m engagements.** A 2 degree heading sigma is 17 m
  at 500 m and a 10 % speed-belief sigma is 1.2 m/s over a 20 to 25 s transit. The cue
  relay plan's own budget is a few metres and about 0.45 m/s. That still gives roughly
  10 m of along-track error after 25 s, so the never-detect problem is real in size, but
  its shape (lateral against along-track) is not what a GPS cue would produce.
- **The scorer counts blind passes as kills.** 11 of 245 stage 2 kills have no detection
  at all (2) or their closest approach before the first detection (9). None in stage 1.
- **I could not see scatter draws in the rows**, so I cannot say which error term drives
  the never-detect runs.

## 5. What I am unsure of

- Everything is the fast sim on an extrapolated fit; drag tiers are guesses.
- The paired tilt result is four cells of 16 at doubled drag, and one cell of 16 with the
  flying lens. The cost of tilt at zero drag was not re-measured here.
- Candidate 1's prototype is two cells of 50. Zero breakage is encouraging, not proof.
- The real camera intrinsics, the real bracket angle and the real tag size are my reading
  of the docs, not checked against the calibration.
- I assumed PX4 holds height when lean-limited (it sizes collective from the vertical
  demand). I did not test that on the bench.
- Candidates 5, 6 and 7 are from reading traces and code; none was run.

## Scratch files (all under the session scratchpad)

`fork_trace.py` (per-tick trace with terminal internals), `fork_gaps.py`,
`fork_exp.py` + `fork_exp1.json` (failing-seed tests), `fork_exp2.py` + `fork_exp2.json`
(stray-detection injection, paired tilt cells), `fork_exp3.py` + `fork_exp3.json`
(clock-arming and belief re-seed prototype against stage 2), `fork_exp4.py` +
`fork_exp4.json` (flying lens and tag at two tilts).
