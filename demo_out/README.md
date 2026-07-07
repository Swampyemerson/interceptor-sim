# demo_out/ — portfolio demo video staging area

This directory is **gitignored** (see `.gitignore`) — it holds regenerable frame
sequences and large MP4/GIF media, not source. Two sample stills are committed
separately under `docs/images/` for quick viewing without regenerating anything.

## Status (2026-07-07): INFRASTRUCTURE BUILT + VALIDATED, HERO FLIGHT NOT YET CAPTURED

Read `demo_out/PARTIAL_NOT_HERO/` literally: everything in there is **proof the
pipeline works**, not the finished demo. A hard, environment-level blocker (see
"Blocker" below) stopped the task before a clean intercept flight could be
captured. This file explains what's done, what's blocked, and exactly how to
finish it once the blocker clears.

## What a completed run should produce here (the target layout)

```
demo_out/
  onboard_frames/     -- PNG sequence from the interceptor's own camera (gz Image topic)
  chase_frames/        -- PNG sequence from the world's static chase-camera sensor
  hud_opaque/           -- render_hud.py frames, opaque background
  hud_transparent/      -- render_hud.py frames, alpha=0 background (Premiere overlay)
  interceptor_demo.mp4  -- compose_demo.sh's HUD+chase composite
  interceptor_demo.gif  -- README-friendly GIF of the same
  onboard_raw.mp4        -- standalone onboard-camera video (ffmpeg, no HUD)
  chase_raw.mp4           -- standalone chase-camera video (ffmpeg, no HUD)
```

None of that exists yet at the top level. What DOES exist is
`demo_out/PARTIAL_NOT_HERO/`, holding the same kinds of artifacts but from
diagnostic/failed runs, kept only to prove the mechanism works:

```
demo_out/PARTIAL_NOT_HERO/
  onboard_frames/    630 real frames, mono_cam topic, mostly ground/takeoff
                      (flight errored at OFFBOARD entry moments after capture
                      started -- see "What happened" below)
  chase_frames/       632 real frames, the world's chase-camera sensor,
                      SAME failed session -- both drones visible but tiny
                      (original camera pose was too far back; see the world
                      file's "POSE FINDING" comment -- an untested, closer
                      replacement pose is already checked in)
  hud_opaque/, hud_transparent/, hud_pipeline_validation.mp4
                      render_hud.py + ffmpeg run against a DIFFERENT flight
                      (a real DASH-ABORT -- cue-seed 4242 never reached
                      HANDOFF, logs/m4_intercept_pronav_20260707T184549Z.csv)
                      to prove the HUD-rendering + encode pipeline works on
                      fresh apriltag_demo-world data. This shows PHASE: DASH,
                      APRILTAG: SEARCHING the whole time -- an honest, real
                      failure mode (ADR-0031's "mid-course track never
                      converges"), NOT an intercept. Do not use this MP4/PNGs
                      as "the demo."
  onboard_raw.mp4, chase_raw.mp4
                      ffmpeg encodes of the two frame sets above -- same
                      "not the hero" caveat.
```

`onboard_frames/` and `chase_frames/` here are from the SAME failed session
(a flight that errored during OFFBOARD entry, ~90s of a stationary/hovering
drone captured afterward while the sim was being investigated) -- they are
**not** synchronized with the `hud_*` artifacts (a different flight
entirely). Do not composite them together; `compose_demo.sh` was not run.

## What's built and CONFIRMED working

1. **World-name hardcoding fix** (`scripts/m2_detect.py`, `m4_target_mover.py`,
   `s2_cue_mock.py`): each hardcoded `WORLD_NAME = "apriltag"`. Added an
   `INTERCEPTOR_WORLD_NAME` env-var override, default unchanged (byte-identical
   for every existing gated caller -- verified `m2_detect.WORLD_NAME ==
   "apriltag"` with the var unset). `m4_intercept.py` needed no edit: it
   imports the topic constants from `m2_detect` and spawns the mover/cue as
   subprocesses, which inherit the parent's env, so setting the var once
   before launch retargets the whole pipeline. **This needs an ADR-lite entry
   from the main session** (per CLAUDE.md) -- flagging, not logging it myself.
2. **Chase-camera sensor** added to `worlds/apriltag_demo.sdf` (a static
   `<model name="demo_chase_cam">` with a `type="camera"` sensor, same
   RGB_INT8/no-padding wire format as PX4's own mono_cam, so
   `scripts/m1_capture.py`'s decode works unchanged). Confirmed alive via
   `gz topic -e ... camera_info` and confirmed rendering REAL content (not
   blank) -- see `docs/images/demo_chase_sample_PARTIAL.png`.
3. **Detection validation (the non-negotiable check) -- PASSED, and it's a
   strong result:**
   ```
   [m2] n_frames=114 detection_rate=1.000 mean_err_norm=0.0861 m
        max_err_norm=0.0861 m mean_range=4.888 m
   [m2] M2 AprilTag detection check PASSED.
   ```
   (`logs/m2_detect_20260707T182521Z.csv`, run against `worlds/apriltag_demo.sdf`
   with `INTERCEPTOR_WORLD_NAME=apriltag_demo`.) **0.0861 m is not just "close
   to" the plain-board baseline -- it matches ADR-0006's own plain-board number
   (0.0861 m at 5 m range) to 4 decimal places.** The `fpv_target` drone body
   dress-up causes ZERO measurable detection-rate or pose-accuracy regression:
   the model's occlusion-safety design (tag geometry always in front of the
   opaque body from the approach direction, ADR-0007's emissive material
   untouched) holds up exactly as designed.
4. **`scripts/demo_capture_frames.py`** (new, not gated): subscribes to any gz
   Image topic + `/clock`, saves a numbered PNG sequence + a `manifest.csv`
   (idx, wall_t, sim_t). Safe to run concurrently with a flight (subscribe-only,
   never calls a gz service, so it can't trigger the "a subscribing process
   never gets service responses" gz-transport13 quirk that forces the
   mover/cue into their own processes). Proven: captured 630 (onboard) + 632
   (chase) real frames in `demo_out/PARTIAL_NOT_HERO/`.
5. **A perf finding worth keeping**: a second full camera sensor at 1280x720
   collapsed idle real-time factor to ~0.05 (vs ~1.0 for the single-camera
   gated world) -- an order of magnitude past ADR-0009's documented ~0.3-0.5
   "under load" floor. Dropping to 960x540 restored ~1.0 RTF in isolated
   (no-Python-client) tests. Logged in the world file's own comment so it
   isn't re-discovered the hard way.

## The hero flight config (chosen, reasoned, NOT yet successfully captured)

```
INTERCEPTOR_WORLD_NAME=apriltag_demo \
S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5" \
.venv/bin/python scripts/m4_intercept.py \
    --fpv --handoff --law pronav \
    --target-start 6.5,-29.3,0.5 --target-vel 0,9.0 \
    --cue-seed <PICK A NEW ONE, e.g. not 4242> \
    --dash-speed 16 --early-handoff --cue-velocity --dash-unclamp
```

This is the **ADR-0030 "FIX" config** (running-start + velocity-emission cue +
`--dash-unclamp`) at 9 m/s pronav, run under the **realistic degraded cue**
(`--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov`) --
the same config already published in the README's headline number (0.58 m
miss at 9 m/s) and stress-tested in ADR-0031. Chose realistic over idealized
deliberately: it's the honestly-representative number, not the flattering one.
`--early-handoff` (streak=2) is the adopted fast-regime default per NEXT.md.
The exact flags were cross-checked against `logs/mc_batch_run_20260707T005613Z.log`
(the real ADR-0031 batch invocation), not reconstructed from memory.

**One real attempt completed** (`--cue-seed 4242`,
`logs/m4_intercept_pronav_20260707T184549Z.csv`): OFFBOARD entry succeeded,
CUE_WAIT/DASH ran, but the DASH aborted -- `failed to reach handoff range
within 20.0s`. This is exactly ADR-0031's own documented ~33%-of-flights
failure mode under the realistic cue (mid-course track never converges), not
a new problem. Per ADR-0031, roughly 2/3 to 5/6 of flights at this config DO
reach handoff -- **the fix is simply a different `--cue-seed` on the next
attempt**, not a config change.

## The blocker (environment-level, not a code/config problem)

Mid-task, new sim boots stopped rendering ANY camera sensor (topic goes
permanently silent) -- confirmed on BOTH the new `apriltag_demo` world AND the
long-proven single-camera gated `apriltag` world, so it is not caused by this
task's world/script edits. Diagnosis:

- `dmesg` shows repeated `misc dxg: dxgk: dxgkio_escape: Ioctl failed: -75`
  around the same time window -- the WSL2 GPU-passthrough kernel module
  (`/dev/dxg`) erroring, i.e. a **host-side (Windows) GPU/WSL driver state
  issue**, not an application bug.
- This matches a PRE-EXISTING documented gotcha (`scripts/sim_gui.sh`'s own
  header comment, dated 2026-07-05): "a wedged renderer... seen after several
  rapid GUI reboots." Today's session did ~10 rapid full boot/kill cycles
  while debugging RTF/resolution/OFFBOARD flakiness, which plausibly tipped
  it over the same edge.
- Ruled out as the cause (tested, all negative): stale gz topics/services
  (none registered while idle), leaked `/dev/shm`/semaphores (none), file
  descriptor exhaustion (nowhere close to the limit), forcing pure
  CPU/software rendering (`LIBGL_ALWAYS_SOFTWARE=1`, still wedged), waiting
  longer between boots (tried up to 180 s cooldown, still wedged).
- **Not fixable from inside this Linux/WSL guest as an agent.** The standard
  remedy for this class of WSL2 `/dev/dxg` issue is a host-side reset:
  `wsl --shutdown` from Windows PowerShell (drops ALL WSL distros/sessions --
  coordinate before running it), then relaunch. A GPU driver update or closing
  other GPU-heavy Windows apps may also help if the shutdown alone doesn't.

## Exact next steps (for whoever resumes this)

1. From Windows: `wsl --shutdown`, wait a few seconds, reopen the WSL
   terminal. Confirm recovery cheaply first: boot the plain gated `apriltag`
   world and check `gz topic -e -t
   /world/apriltag/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info
   -n 1` responds within ~10 s.
2. Symlink is already in place: `~/PX4-Autopilot/Tools/simulation/gz/worlds/apriltag_demo.sdf`
   -> this repo's `worlds/apriltag_demo.sdf` (worktree copy). Re-verify it
   points at the checkout you're actually editing.
3. Fly the hero config above with a fresh `--cue-seed` (2-3 tries should land
   a clean handoff per ADR-0031's odds). **Boot a completely fresh sim per
   attempt** (the project's own hard rule -- a landed-and-displaced drone
   does not reset state within one sim instance).
4. Start `scripts/demo_capture_frames.py` (onboard topic + chase topic, two
   separate background processes -- see the exact commands used earlier in
   this session, preserved in the task's own report / git history of this
   file if trimmed) BEFORE the flight launches, kill them after it lands.
5. Once a clean run lands (miss < 1.5 m per the task brief): render both HUD
   variants (`render_hud.py ... --out demo_out/hud_opaque`,
   `... --transparent --out demo_out/hud_transparent`), then
   `scripts/compose_demo.sh <hero_csv> demo_out/chase_frames`.
6. THEN re-verify (or re-tweak) the chase-camera pose against a real captured
   frame -- the world file's pose was analytically tightened this session
   (24 m -> ~14.7 m, pitch 38 deg -> ~30.7 deg) but never re-rendered; treat
   it as a first guess, not a validated final pose.
7. Delete/replace `demo_out/PARTIAL_NOT_HERO/` once the real hero artifacts
   land in the top-level layout described above.

## compose_demo.sh usage (once a hero CSV + chase_frames/ exist)

```
scripts/compose_demo.sh <hero_flight_csv> demo_out/chase_frames
```

Produces `demo_out/interceptor_demo.mp4` (HUD beside the chase render,
hstacked) and `demo_out/interceptor_demo.gif`. Requires `ffmpeg` (confirmed
installed this session: `ffmpeg version 6.1.1-3ubuntu5`). Standalone
onboard/chase MP4s for Premiere are just:

```
ffmpeg -y -framerate 30 -i demo_out/onboard_frames/frame_%06d.png -vf format=yuv420p -c:v libx264 -crf 20 demo_out/onboard_raw.mp4
ffmpeg -y -framerate 30 -i demo_out/chase_frames/frame_%06d.png  -vf format=yuv420p -c:v libx264 -crf 20 demo_out/chase_raw.mp4
```
