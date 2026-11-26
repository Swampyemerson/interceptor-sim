# NEXT — the live queue

> **Canonical state → [`project_state.json`](project_state.json)** · view: [dashboard](dashboard.html)
> **Task board → https://github.com/users/Swampyemerson/projects/1** (issues in `Swampyemerson/interceptor-sim`)
> **History → [`next_archive.md`](next_archive.md)** — the full 883-line queue as it stood 2026-07-26, verbatim. Reasoning lives in [`decisions.md`](decisions.md) (the ADRs).
>
> *Rewritten 2026-07-26: this file had grown to 883 lines of stacked superseded banners — the "too much text" problem in miniature. It is now the SHORT live queue only. **Rule: items live here or on the board, not both; superseded blocks go to the archive the same turn they are superseded, never left in place with a strikethrough.***

## Where the project is (2026-11-22)

**The assembled interceptor flew by hand three times on 2026-11-22; the GPS is the wall.**
The aircraft follows its commands well and the Pi saved a clean log and video every time.
The camera link jams the GPS; foil on the ribbon made it marginal, not good, and every
flight failed from the estimator dropping the GPS. **The plan is
`docs/gps_reception_plan.md`: measure the GPS at several heights with a metal plate under
it, then print a mount to the size that works.** No automatic flight before that. What
follows is the state as of 2026-11-18, still true.

**No automatic flight has happened yet, and the next step is a props-off rehearsal on the
real Pi and Pixhawk.** The interceptor flies by hand (2026-11-09) and the target flies a
drawn course by itself (2026-11-15; it then fell 5.8 m and needs inspecting). On 2026-11-18
the flight program gained what it lacked to be flown deliberately, all tested on the laptop
only:

- **Flight modes.** `--fly` is the only mode that arms on the real link, and only after a
  radio arm permit. `--bench-props-off` runs the same loop and cannot arm. A watchdog ends
  the command stream if the Pi program hangs (`docs/interface_audit_2026-11-18.md`).
- **The cue is the GO command.** The target's GPS arrives live, the freshest 5 s are fitted,
  and the belief is locked at GO with the link delay removed (`flight/cue_link.py`,
  `docs/cue_relay_plan.md`). Nothing yet gets the real target's GPS off the target;
  `docs/cue_over_control_link.md` is the first route to try.
- **Long-range dash.** With a realistic error in the target's believed speed, the 500 m dash
  parked in the wrong place and never saw the tag in 61 % of fast-sim runs. A search along
  the believed track on arrival, now in the flight preset, takes a straight target from 117
  to 190 kills of 192 with the flying lens at 20 degrees of camera tilt
  (`isim/specs/flight_hardware_rerun_prereg_2026-11-18.md`). A weaving target is still
  rarely caught. None of it is Gazebo-checked or flown.
- **The board** now allows 33 m/s (was 12). With the assumed drag the dash peaks near
  26 m/s on the 45 degree lean limit; the "80 mph" Gazebo take had no drag and no cue speed
  error.

Decisions: `docs/decisions.md` ADR-0139 and the 2026-11-18 addenda to ADR-0130 and ADR-0132.

## Waiting on the builder

| | what | where |
|---|---|---|
| ⬜ | **Take the props off and say so.** Then I run the bench rehearsal (`--bench-props-off`), the watchdog check and the reboot check of the 33 m/s limit on the real board. **HELD until then: nothing live runs on the vehicle link.** | `docs/interface_audit_2026-11-18.md` (bench steps), `runs/real_flight_driver_sih_20261118/README.md` |
| ⬜ | **Tag facing.** The placard is designed side-facing; the chase from behind cannot read it (0 of 16 in the sim), and a rear-facing plate is the option the mount design rejected for drag. | contract question `target-tag-facing`; `docs/placard_mount.md` section 3 |
| ⬜ | **Camera plate angle.** Ruled +10 degrees with a canted placard (2026-09-24); the flying-lens rerun says 20, under different conditions. Do not set it yet. | contract question `camera-plate-angle` |
| ⬜ | **Try the target's GPS over the control link**: three menu settings on the transmitter, target disarmed, then check sticks and kill switch with the USB cable in. | `docs/cue_over_control_link.md` |
| ⬜ | **Fly the target on one long straight leg at 9 to 12 m/s, logged.** It measures the cue's real speed error, the number the long-range dash turns on. | contract assumptions `cue-speed-error-at-long-range`, `search-declared-cue-quality` |
| ⬜ | **Geofence against long range**: 457 m, Hold. Keep cues inside about 300 m, or widen it for those flights. | contract question `dash-envelope-on-real-fc` |
| ⬜ | **Target after its fall (2026-11-15)**: inspect; choose low battery = land for the yard; fly on a 6 minute timer. | ADR-0138 addenda 17 and 18 |
| ⬜ | **GPS bench session** (outdoors, props off, about 30 min): the GPS unit at several heights with a plate under it, camera running; I read the numbers and we pick the mount's size. | `docs/gps_reception_plan.md` step 1 |
| ⬜ | **GPS mast and deck are drawn, not printed, and printing now waits on a proving flight.** After the wiring tidy of 2026-11-25 the GPS reads 14 to 15 satellites with the camera recording and speed accuracy near 0.17 m/s on the ground, so the mast may not be needed for reception. Next: recalibrate the compass outdoors (the wiring moved), fly one short hover as it is, and decide from that log. The mast would still cut the compass's pickup of battery current 2.8 to 4.8 times. | `runs/interceptor_rewire_20261125/README.md`, `docs/gps_reception_plan.md` |
| ✅ | **The three approved settings are on the board** (barometer height reference, sharper Position mode): written 2026-11-25 and read back again after a restart. Still to do: fly them. | `configs/px4_6cmini/bench.params` |
| ⬜ | **Recalibrate the compass outdoors, away from metal.** Today's recalibration left it reading 0.39 to 0.42 G against 0.51; I check the reading afterwards. | `runs/interceptor_flight3_20261122/README.md` |
| ⬜ | **Check the pod's fasteners and do the tap test**: sideways vibration grew across the three flights. | same |
| ⬜ | **Note the mAh the charger puts back** into today's packs. | `runs/interceptor_flight2_20261122/README.md` |
| ⬜ | **Roll trim of 2 degrees applied (2026-11-22)**; flight 3 could not confirm it. Watch for sideways drift in Altitude mode next time. | same |
| ⬜ | **For lean angle against speed** (drag, and the camera angle that follows from it): straight runs of 5 s or more at two or three steady speeds. | same |
| ⬜ | **Charge habit:** the avionics drain the pack while it sits powered, unseen by the gauge (about a quarter of the pack an hour, estimated). Plug the flight pack in last. | `runs/interceptor_flight_20261122/README.md` |
| ✅ | **Interceptor recorder at a real arming** (2026-11-22): clean log and 88 s of video on the first flight. Before any flight `recorder_check.py` must say GO. | `docs/fc_logging_without_sd.md` |
| ⬜ | **Radio loss in automatic flight** returns home after 0.5 s on this board, so the transmitter-off test cannot pass as set. Change only after the watchdog is bench-proven. | interface audit G5 |
| ⬜ | **Second transmitter (about $65)**, anemometer, tripod: not confirmed bought. Each aircraft needs its own arm and kill for a deliberate mid-air (ADR-0089). | [#5](https://github.com/Swampyemerson/interceptor-sim/issues/5) |
| ⬜ | **Stamped-age latency mode**: adopt or keep off (prediction confirmed, strict bar missed 3 of 6; recommendation: keep off until the Pi's timestamps are bench-checked). | contract question `stamped-age-adoption` |
| ⬜ | **Publication leftovers** (the repo is already public): rotate the VM password, re-point the launcher. | `docs/publish_cleanup_work_order.md` |
| ⬜ | **Logs retention policy** (about 1 to 1.5 GB of gitignored run data). | `docs/deletion_candidates_2026-10-22.md` Tier 3 |

No longer waiting (ruled 2026-11-18): lean limit (leave 45 degrees), engage time (no limit),
lost-target rule (the flight preset), cue window (5 s), board speed limit (33 m/s).

## Next work, in order (mine)

0. ⬜ **The nose camera is upside down and the flight program does not know it**
   (measured 2026-11-22, aircraft level). The video recorder flips its picture; the
   flight program's camera reader, the calibration capture and the lens calibration do
   not. Until they do, the chase would steer away from the target. One switch for all
   camera readers, then recalibrate on the mounted camera. Contract ledger
   `nose-camera-upside-down-flight-reader-has-no-flip`.
1. ⬜ **Props-off bench run on the real Pi and Pixhawk. HELD: props are on.** In order:
   `--bench-props-off` with a timer GO (expect the interlocks to pass on USB power), then
   with `--trigger cue` and the laptop's made-up target, then the watchdog (`kill -STOP`
   and `kill -9` during ENGAGE; PX4 must leave OFFBOARD), then the transmitter's GO and
   permit channels read from the real board (rate on TELEM2 is unmeasured).
2. ⬜ **One fast-sim grid for the camera plate**: flying lens, target pitching, placard
   canted 12 degrees, tilt 10 / 15 / 20 / 25. Pre-register. Settles the tilt conflict.
3. ⬜ **Side-arrival geometry with the search**, side-facing tag, pre-registered: is the
   designed placard usable if the interceptor comes in from the beam?
4. ⬜ **One Gazebo rung** for the search and the flight preset at a 10 % speed error (demo
   geometry, flying lens model, machine idle). The fast sim ranks; Gazebo decides.
5. ⬜ **The weaving target.** 2 to 11 kills of 50 whatever the cue; the misses are close
   passes and lost targets in the final close, not failures to find it. Unexplained.
6. ⬜ **Default-path defect**: after a camera dropout a stale decode can restart the camera
   phase every tick. Fixed behind `stale_decode_guard` (on in the flight preset), pinned by
   a strict-xfail test on the default path. Decide whether to make the guard the default.
7. ⬜ **Cue loose ends**: standby height and yaw are still typed, not taken from the cue;
   the handset-USB reader has never met the real transmitter; with that source the link
   delay is assumed, not measured.
8. ⬜ **Interface audit leftovers**: `docs/kill_day_protocol.md` is ahead of any bench
   proof. (G8 closed 2026-11-22: the recorder no longer takes the serial cable while
   another program has it open; bench-tested with a stand-in holder, not with the flight
   program itself.)
8b. ⬜ **From the first flight's log (2026-11-22), not yet acted on**: the board's battery
   gauge works from voltage alone and read 21 % after 12 % of the pack was used, so a
   dash would trip the low-battery failsafe early (set the pack's capacity and
   resistance, 32 milliohms measured); with no GPS the estimator's position ran 470 m
   away and tripped the geofence; the height reference is set to GPS; `navigator low on
   stack` repeats every 32 s; compass 1 reads the aircraft's own field.
9. ⬜ **Real numbers the sims still assume**: drag (lean angle at a steady known speed),
   braking, full-throttle current through the power module (`docs/airframe_numbers.md`
   section 6), the decode curve outdoors.

Older threads still open, unchanged by this work (details in `docs/next_archive.md` and the
contract): the vertical channel arms (ADR-0099/0100), the rehearsal break-off margins, the
markerless seeker program (`docs/nn_seeker_program.md`), the target's compass recalibration.

## Standing cautions

- **Every first power-up after soldering goes through the smoke stopper, props off** (tgt-04 gate). The stopper is in hand as of 2026-07-26 — the gate is now a step to perform, not a part to wait for.
- **LiPos:** store ~3.8 V/cell, in the bag, never charge unattended. Charge the CNHL 6S 1500 packs at **3.0 A (2C)** for field turnaround, 1.5 A (1C) when not in a hurry; the vendor page lists 5C (7.5 A) as the maximum (builder read it 2026-11-15), which is a limit, not a habit. Cool to room temperature before charging; land at about 3.5 V/cell resting.
- **One sim at a time, at idle load.** Gates and batches only when the machine is quiet.
- **A green run is only as good as the machine that produced it** — `docs/verification_environment.md` says what the dev box, CI and a fresh cloud clone each can and cannot verify. Nothing from a cloud session has flown.
- **Pre-register** any arm that could change a belief — prediction, criterion, and what a null means — *before* it flies.
