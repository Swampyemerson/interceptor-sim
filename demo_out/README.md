# demo_out/ — portfolio demo video staging area

This directory is **gitignored** (see `.gitignore`) — it holds regenerable frame
sequences and large MP4/GIF media, not source. Two sample stills are committed
separately under `docs/images/` (`demo_chase_final.png`, `demo_onboard_final.png`)
for quick viewing without regenerating anything.

## Status (2026-07-07): DONE — hero flight captured, composited, packaged

```
demo_out/
  onboard_frames/        -- 867 PNGs, interceptor's own camera (gz Image topic),
                             sim_t 145.2-173.8s, covers TAKEOFF through landing
  chase_frames/           -- 715 PNGs, the world's static chase-camera sensor,
                             TIME-SYNCED to the hero CSV's own t_sim origin
                             (trimmed/renumbered from a longer raw capture --
                             see "Sync gotcha" below)
  hud_opaque/              -- render_hud.py frames, opaque background, 409 frames
  hud_transparent/         -- render_hud.py frames, alpha=0 (Premiere overlay), 409 frames
  interceptor_demo.mp4      -- compose_demo.sh's HUD+chase composite (2560x1080, 23.8s)
  interceptor_demo.gif      -- README-friendly GIF of the same
  onboard_raw.mp4            -- standalone onboard-camera video (ffmpeg, no HUD)
  chase_raw.mp4               -- standalone chase-camera video (ffmpeg, no HUD)
```

## The hero flight

```
INTERCEPTOR_WORLD_NAME=apriltag_demo \
S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5" \
.venv/bin/python scripts/m4_intercept.py \
    --fpv --handoff --law pronav \
    --target-start 6.5,-29.3,0.5 --target-vel 0,9.0 \
    --cue-seed 31 \
    --dash-speed 16 --early-handoff --cue-velocity --dash-unclamp
```

Result: **miss_distance_m=1.061, clean=1, engaged=1**
(`logs/m4_intercept_pronav_20260707T194623Z.csv`). Under the 1.5 m target from
the task brief and consistent with ADR-0030/0031's published realistic-cue
numbers for this config (~1.19 m mean @ 9 m/s, n=6). CPA at t_sim=163.252s,
`gt_range=1.0613` — camera=(5.72,-12.16,1.07), tag=(6.50,-11.73,0.50).

**Required a pre-placement fix, not just a lucky seed** — see "The real bug
found and fixed" below. Two earlier attempts (cue-seed 17, then 23 with
`--early-handoff` dropped) both hard-failed with near-identical symptoms
before the fix was identified and applied.

## The real bug found and fixed (this is the important part)

Two flight attempts (`logs/m4_intercept_pronav_20260707T193501Z.csv` seed=17,
and `logs/m4_intercept_pronav_20260707T194142Z.csv` seed=23) both failed with
the same signature: `[s2] First camera detection during DASH at range=~5.0 m`
happening almost immediately, followed by a fragile HANDOFF and then
`BREAKOFF: lost detection` or a hard abort, both landing near `miss=4.8-4.9m`.

CSV forensics (`gt_range` vs `r_hat_m` in the DASH->ENGAGE rows) found the
real cause: `worlds/apriltag_demo.sdf`'s `apriltag_target` include spawns at
its **world-file default pose (5, 0, 0.5)** — a few meters from the
interceptor's own origin start. `m4_target_mover.py` does its own
"pre-warming" `set_pose` call to relocate the tag to the CLI's
`--target-start` (6.5,-29.3,0.5), but that call only fires once the mover
subprocess is spawned, which is well into `CUE_WAIT`/`DASH` -- by which time
the interceptor's ALWAYS-RUNNING camera detection thread had already locked
onto the nearby DEFAULT-position board. When the mover's pre-warm then
teleports the tag 30 m away, the shared `TargetTracker`'s alpha-beta filter
doesn't know about the discontinuity and just coasts on its stale ~5 m
estimate while `gt_range` diverges to 25-30 m -- guaranteed BREAKOFF/abort.

**This is a pre-existing race in `m4_intercept.py`'s own flow, not a
demo-world or reskin bug** (the plain gated `apriltag.sdf` has the exact same
`(5, 0, 0.5)` default spawn pose). It didn't show up in any gated/ADR run
because **`mc_batch.sh` always pre-places the tag via its own external
`gz service .../set_pose` call BEFORE launching `m4_intercept.py`**
(see `mc_batch.sh`'s "Pre-placing the tag..." step) — every ADR-0028/0030/0031
number was produced through that wrapper, which silently avoided the race.
Calling `m4_intercept.py` directly (as this demo-capture task does, and as
the module docstring's own CLI example does) has no such guard.

**The fix applied here**: an external `gz service -s
/world/apriltag_demo/set_pose ... --req 'name: "apriltag_target" position {
x: 6.5 y: -29.3 z: 0.5 }'` call, run manually before launching
`m4_intercept.py`, exactly mirroring `mc_batch.sh`'s own pattern. This is a
**workaround, not a code fix** -- flagging for the main session: consider
having `m4_intercept.py` itself do this pre-placement internally (it already
knows `--target-start` and the world name) rather than relying on every
caller to replicate `mc_batch.sh`'s external step correctly.

## Sync gotcha: chase-camera capture start vs CSV `t_sim` origin

`scripts/demo_capture_frames.py` was started (deliberately, to not miss
takeoff) a few seconds before `m4_intercept.py` began logging. The hero CSV's
first row is `t_sim=150.292` (already `TAKEOFF`); the raw chase capture's
`manifest.csv` started at `sim_t=145.564`, live for the ENTIRE takeoff-to-
well-past-landing window. `compose_demo.sh`'s ffmpeg hstack pairs
`hud_frames/frame_NNNNNN.png` with `<chase_dir>/frame_NNNNNN.png` **by index
only** — with no correction, HUD frame 0 (`t_sim=150.292`) would be paired
against a chase frame from ~4.7 s (~141 frames) EARLIER, desyncing the
composite by that much for the whole clip (worst at the CPA moment: the HUD
would show "closest approach" while the chase pane was still mid-DASH).

**Fix**: found the chase-manifest row closest to the CSV's own `t_sim[0]`
(`150.292`, matched `sim_t=150.284` at raw index 143), then wrote a trimmed +
renumbered copy of just that tail (`frame_000000.png` = the matched frame)
before running `compose_demo.sh`. Verified by extracting a frame at the
composite's CPA timestamp (`t≈12.9s` into the 23.8s clip) and confirming the
HUD's `t = 163.188s (sim)` / `RANGE 2.31m` reading lines up with the chase
pane showing the two drones close together, not mid-DASH.

**Flag for the main session**: `compose_demo.sh`/`demo_capture_frames.py`
don't currently do this trim automatically -- worth adding a `--sim-t-start`
alignment step (read the CSV's first `t_sim`, trim the frame dir to the
closest matching manifest row) so future hero flights don't need this manual
step repeated.

## Chase-camera pose (final, validated against real frames)

`worlds/apriltag_demo.sdf`'s `demo_chase_cam` static model pose:
`14.900 -7.612 3.388 0 0.262 -2.647` (x y z roll pitch yaw, meters/radians).
Tuned LIVE against real rendered frames (via `gz service .../set_pose` on the
static model -- confirmed working, no reboot needed per iteration) using
real CPA-region coordinates pulled from an actual hero-flight CSV (not a
guess): camera sits ~10 m out along the perpendicular to the interceptor's
dash bearing (broadside view), pitched ~15 deg down, giving a level horizon
with sky occupying the upper ~30% of frame and both drones clearly
distinguishable as separate shapes at the CPA range. See the world file's own
header comment for the full derivation and the two earlier (rejected, too
far / too little sky) pose guesses.

## GPU / boot discipline note

6 total sim boots this session (2 for reskin/GPU verification + a real bug
fix, 1 for chase-cam pose tuning, 3 for flight attempts -- 2 hit the
pre-placement race above, the 3rd succeeded after the fix). `sudo dmesg |
grep dxgkio_escape` was checked after every boot; no NEW errors appeared
during any boot's active window the whole session (all clusters seen were
delayed teardown echoes from the PRIOR boot's cleanup, confirmed by
timestamp correlation) -- the GPU never re-wedged.

## compose_demo.sh usage (once a hero CSV + chase_frames/ exist)

```
scripts/compose_demo.sh <hero_flight_csv> demo_out/chase_frames
```

Produces `demo_out/interceptor_demo.mp4` (HUD beside the chase render,
hstacked) and `demo_out/interceptor_demo.gif`. Requires `ffmpeg` (confirmed
installed: `ffmpeg version 6.1.1-3ubuntu5`). **Read the "Sync gotcha" section
above before pointing this at a fresh raw chase capture** -- trim/renumber
it to the CSV's own `t_sim[0]` first, or the composite will be desynced.
Standalone onboard/chase MP4s for Premiere are just:

```
ffmpeg -y -framerate 30 -i demo_out/onboard_frames/frame_%06d.png -vf format=yuv420p -c:v libx264 -crf 20 demo_out/onboard_raw.mp4
ffmpeg -y -framerate 30 -i demo_out/chase_frames/frame_%06d.png  -vf format=yuv420p -c:v libx264 -crf 20 demo_out/chase_raw.mp4
```
