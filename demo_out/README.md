# demo_out/ — portfolio demo video staging area

This directory is **gitignored** (see `.gitignore`) — it holds regenerable frame
sequences and large MP4/GIF media, not source. Two sample stills are committed
separately under `docs/images/` (`demo_chase_final.png`, `demo_onboard_final.png`)
for quick viewing without regenerating anything.

## Finished demo video — `scripts/build_demo.py` (2026-07-07)

The produced cut is assembled offline (no Gazebo/GPU — the 700+ frames are already
captured) by **`scripts/build_demo.py`** from the hero flight
`logs/m4_intercept_pronav_20260707T211601Z.csv` (miss **0.632 m**; that CSV was
rotated off disk and never committed — the result is durably logged in the
ADR-0032 addendum in `docs/decisions.md`, and the captured frames here are the
surviving artifact). Run it with:

```
.venv/bin/python scripts/build_demo.py          # onboard MP4+GIF + chase MP4
.venv/bin/python scripts/build_demo.py --fast    # onboard MP4 only (quick iteration)
```

**PRIMARY = the ONBOARD (seeker-POV) cut** — the interceptor's own camera with the
FPV OSD HUD overlaid, ending on the proximity-fuse close-out. **Re-cut 2026-07-07 on
builder feedback**: come in from further away, keep the slow-mo interception, remove
the mini-map, de-cringe the OSD, drop the outro. Beats (~20 s total):
1. **Title card (short)** — one headline line ("Camera-only proportional-navigation
   counter-UAS intercept") + one understated honesty line. No lecture.
2. **ESTABLISH (REAL-TIME 1×, ~4.5 s)** — the interceptor holds on the external cue
   while the target sits as a distant shape near the horizon: it comes in from far
   away. HUD `PHASE TAKEOFF/CUE_WAIT`, `SENSOR EXT CUE (mock)`, `APRILTAG SEARCHING`,
   `RANGE ACQUIRING`.
3. **BUILDUP / DASH-IN (SLOW MOTION ~4×, ~7 s)** — gently paced so the target visibly
   GROWS from a distant shape to an AprilTag speck as range closes 30 m → ~5 m; the
   `GND SPD` gauge ramps; `APRILTAG SEARCHING`.
4. **HANDOFF beat (SLOW MOTION ~7×)** — `SENSOR` flips to `CAMERA-ONLY` with a
   "DATALINK DENIED → CAMERA-ONLY TERMINAL" callout (comms-denied headline; keyed off
   `ext_fresh` 1→0, not a phase value); `APRILTAG` flips `SEARCHING → LOCKED`.
5. **Terminal (SLOW MOTION ~7×)** — the AprilTag looms and fills the frame while the
   HUD firing solution collapses: `RANGE` across the red R_lethal tick, LOS rate
   spiking, `T-GO` counting down, `APRILTAG LOCKED`. Kept exactly (builder liked it).
6. **PROXIMITY FUSE — DETONATE hold (~1.9 s), then cut to black** — de-cringed: ONE
   thin criterion ring on the tag (no white flash, no expanding shockwave, no double
   rings), captioned **"CPA 0.63 m < 1.5 m lethal radius (ADR-0025 criterion — not a
   modeled blast)"**. No collision/blast volume in the sim (ADR-0014). The old
   outro/metrics card was REMOVED (builder feedback #4); the reel ends here.

**SECONDARY = the CHASE (wide) cut** — the same intercept from the world chase
camera + a compact HUD, ~3.8 s, a brief B-roll wide angle. Deliberately short; the
onboard seeker view is the centerpiece.

**Retiming is disclosed on-screen** (`REAL-TIME 1×` establish / `SLOW MOTION ~4×`
buildup / `SLOW MOTION ~7×` terminal pills): the dash closes 30 m in ~2 s of sim
time and the terminal is only ~0.36 s, so the approach is gently paced and the
terminal slowed for clarity. Slow-mo in-betweens are honest cross-dissolves of
adjacent captured frames; the HUD is overlaid **unblended** (crisp) on top so its
ticks/readouts never ghost.

**The HUD** is `scripts/render_hud.py --layout overlay` (FPV OSD): every widget
traces to a CSV column — phase/sensor/AprilTag lamps, a heading tape (`psi_deg`),
`T-GO` (`r_hat_m`/`vc_m_s`), a depleting `RANGE` bar with the R_lethal tick
(`r_hat_m`), `CLOSING` (`vc_m_s`), `LOS RATE` (`lambda_dot_deg_s`), a `GND SPD` gauge
(0.3 s-smoothed d/dt of `gt_cam_x/y`, GT-derived/display-only), an `ALT` gauge
(`alt_m`), and a fixed thin boresight reticle. **The two-series mini-map, the
"INTERCEPT SOLUTION" status line, and the "LAW PRONAV" label were REMOVED from the
overlay in the 2026-07-07 de-cringe pass** (builder: read like a clean instrument
panel, not a moving-map video game — err toward LESS). The honesty footnotes (mocked
cue; `GND SPD`/CPA are GT scoring-only, never fed to guidance) stay, just smaller. No
pitch/roll horizon — the CSV has no honest source for it, and this low-altitude
engagement is essentially level (see the module docstring). The `sidebar` layout is
retained (with its mini-map) for back-compat with `compose_demo.sh`.

**Outputs:** `interceptor_onboard.mp4` (~20 s, 1280×960, PRIMARY), a highlight-loop
`interceptor_onboard.gif` (handoff→fuse, ~3 MB), and `interceptor_chase.mp4` (~3.8 s,
960×540, SECONDARY). The committed stills `docs/images/demo_onboard_final.png` (the
proximity-fuse frame) and `demo_chase_final.png` are refreshed from this flight (both
de-cringed: no mini-map).

## Capture-session history (2026-07-07) — re-capture after builder feedback (3 fixes + 1 add)

*(Historical context for how the raw frames were captured. The finished video is
built from them by `build_demo.py`, documented above.)*

Builder (Emerson) reviewed the first hero capture and caught three real problems;
a fourth (ground texture) was added by the main session mid-task. All four are
fixed and validated against a **freshly re-flown Gazebo run**, not just a
render-side patch:

1. **Target drone body re-oriented** (`models/fpv_target/model.sdf`) so it reads
   as "flying forward along its travel path" (nose along +Y, the crossing
   direction) instead of "strafing sideways" (nose fixed toward -X, the tag's
   own facing). The AprilTag plane itself is **byte-identical** — same pose,
   size, material (verified via `git diff`, zero changes to the `tag_visual`
   block) — only the decorative fuselage box + camera/antenna nub were
   re-yawed **in place** (position unchanged, only the `yaw` pose field
   changed), which a hand-verified numeric check confirmed keeps everything
   fully behind/outside the tag's occlusion footprint. The 4-fold-symmetric
   arm/prop/hub "shield" ring was left untouched on purpose — a naive whole-
   assembly rotation about the tag's own origin was checked and DOES push
   those parts in front of the tag plane (a real occlusion breach); see the
   model file's own "BODY RE-ORIENT" comment for the full derivation.
   **Detection re-validated on an actual re-flown Gazebo run**: ENGAGE+BREAKOFF
   detection rate 0.408 (new, post-reorient) vs 0.208 (old, pre-reorient) —
   not degraded. (Correction 2026-07-07: an earlier version of this note
   claimed the body was "never visible" to the onboard camera — wrong. The
   onboard camera DOES see the target's body at long range, as a distant
   growing shape; the 20 s re-cut's establish beat is built from exactly those
   authentic frames. What the re-orient preserves, and what detection depends
   on, is that no body part enters the TAG's occlusion footprint — the tag
   face the detector sees is geometrically unchanged in either orientation.)
2. **HUD mini-map track viz fixed** (`scripts/render_hud.py`): (a) the
   degraded ground-cue estimate (`tgt_n_hat`/`tgt_e_hat`) is now plotted as
   the FULL-LENGTH array (NaNs left in place) instead of a NaN-filtered/
   compacted array — matplotlib naturally breaks the line at any dropout
   tick instead of drawing a fake straight line bridging the gap (the "fake
   zigzags" the builder saw). (b) the ground-truth path (`gt_tag_x/y`) is now
   a smooth solid line labeled "true path (reference, not seen by guidance)".
   (c) the noisy estimate is labeled "degraded ground-cue estimate". (d) a
   SECOND, less obvious bug found while fixing this: the mini-map's axis
   limits were being computed FROM the noisy estimate too, so its real
   (honest, not fake) 5-15 m sample-to-sample swings were dragging the whole
   map's zoom out and squashing the smooth ground-truth path into a sliver.
   Fixed by anchoring the map extent on ground truth only (`gt_cam_x/y`,
   `gt_tag_x/y`) — the noisy estimate still plots every tick, it just gets
   clipped by the normal axis range instead of rescaling the whole frame.
   Both `--transparent` and the default opaque mode re-validated.
3. **Target already inbound near the start of the captured shot**: the tag
   genuinely doesn't start moving until `m4_intercept.py`'s DASH phase
   (an `m4_target_mover.py` subprocess spawned right at that transition —
   changing that timing would touch guidance-critical, ADR-0030/0031-tuned
   behavior, out of scope for a video polish task). Fix applied at
   CAPTURE/COMPOSE time instead: trimmed the CSV + frame sequences to start
   a few seconds before the target begins crossing, and (a real finding
   made mid-task) discovered the fixed chase camera can't actually see
   EITHER drone until ~1s into DASH anyway (its FOV is aimed at the
   engagement corridor, not the interceptor's TAKEOFF hold point) — so the
   MAIN composite is trimmed tight to `t_sim>=18.0s` (target starts moving at
   18.548s, ~0.5s into the clip), giving a punchy ~3s clip that is
   essentially 100% "the enemy is already crossing, the interceptor meets
   it" with almost no dead establishing footage. The wider trim
   (`t_sim>=15.4s`, before CUE_WAIT) is kept for the two STANDALONE
   `onboard_raw.mp4`/`chase_raw.mp4` videos, which can better afford a brief
   establishing beat. **Ties to ADR-0028's velocity-step finding**: the real
   deployment concept (ground-standby -> launch-on-detect -> dash) already
   assumes the threat is inbound before the interceptor ever launches — this
   fix is also the more DEPLOYMENT-HONEST framing, not just a cosmetic one.
4. **Ground plane texture** (`worlds/apriltag_demo.sdf` + new
   `models/demo_ground_grid/grid_ground.png`, generated by
   `scripts/gen_demo_ground_texture.py`): the flat single-color ground gave
   no sense of speed/parallax. SDF's `<plane>` geometry has no UV-tiling
   field and gz-sim's PBR material has no repeat/scale field either, so the
   5 m grid spacing is baked directly into a 2048x2048 pre-tiled PNG (100x100
   cells over the 500 m plane) referenced as a `<pbr><albedo_map>`. Demo-world
   only; the gated `worlds/apriltag.sdf` ground plane is untouched.

```
demo_out/
  == raw captures (Premiere source layers) + their manifest.csv (idx,sim_t,file) ==
  onboard_frames/            -- 704 PNGs, interceptor's own camera (1280x960),
                                FULL raw capture (sim_t ~4.1-27.3s)
  chase_frames/              -- 706 PNGs, world chase camera (960x540), FULL raw
  == transparent HUD overlay LAYERS (render_hud.py --layout overlay, alpha=0) ==
  hud_onboard_transparent/   -- FPV OSD frames (1280x960), named frame_<capIdx>,
                                index-aligned 1:1 with onboard_frames (engagement
                                window) -- the Premiere HUD layer for the onboard cut
  hud_chase_transparent/     -- same for the chase cut (960x540)
  == assembled composites (build_demo.py output) ==
  onboard_final_frames/      -- the PRIMARY onboard cut, composited + cards + fuse
  chase_final_frames/        -- the SECONDARY chase cut
  interceptor_onboard.mp4    -- PRIMARY finished video (~15s, 1280x960)
  interceptor_onboard.gif    -- README highlight loop (handoff -> fuse)
  interceptor_chase.mp4      -- SECONDARY wide view (~3.8s, 960x540)
  == legacy (first-pass, kept; superseded by build_demo.py) ==
  onboard_frames_trimmed/, chase_frames_trimmed/, chase_frames_composite/,
  hud_opaque/, hud_transparent/, hud_frames/, interceptor_demo.{mp4,gif},
  onboard_raw.mp4, chase_raw.mp4   -- the earlier compose_demo.sh / sidebar-HUD pass
```

## The hero flight (v2, post-fix)

```
INTERCEPTOR_WORLD_NAME=apriltag_demo \
S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5" \
.venv/bin/python scripts/m4_intercept.py \
    --fpv --handoff --law pronav \
    --target-start 6.5,-29.3,0.5 --target-vel 0,9.0 \
    --cue-seed 31 \
    --dash-speed 16 --early-handoff --cue-velocity --dash-unclamp
```

Same ADR-0030 FIX config + cue-seed 31 as the first hero capture (ADR-0032).
Result: **miss_distance_m=0.632, clean=1, engaged=1, handoff=1**
(`logs/m4_intercept_pronav_20260707T211601Z.csv` — since rotated off disk and
never committed; the ADR-0032 addendum in `docs/decisions.md` is the durable
record of this number) — even better than the
first capture's 1.061 m, and consistent with ADR-0030/0031's published
~1.19 m mean for this config (run-to-run noise, both within range). CPA at
`t_sim=20.528s`, `gt_range=0.6324`.

**Tag was pre-placed externally BEFORE launch** (`gz service .../set_pose`,
mirroring `mc_batch.sh`'s own pattern and this project's documented
ADR-0032 race-condition fix) — no repeat of that race this time.

**First attempt this session hard-failed for an unrelated reason**: a
`mavsdk_server` subprocess crash (SIGABRT, confirmed via
`sudo dmesg | grep CaptureCrash`) right as `m4_intercept.py` tried to enter
OFFBOARD, under elevated system load (2 extra camera-capture processes +
the chase-cam rendering load on top of the already-heavy 2-camera world).
Not a GPU wedge (camera topics kept publishing fine both before and after;
`dxgkio_escape` entries were the same pre-existing boot-time cluster, not
new/live ones) and not an SDF/model bug (offline `gz sdf -k` parse-checked
clean beforehand). Treated as the transient it looked like — a full fresh
reboot (mirroring `mc_batch.sh`'s "boot fresh per flight" policy, ≥30s
cooldown observed) on the very next attempt flew clean. 2 sim boots total
this session, both with a post-boot camera-topic-publish check and a
`dmesg` correlation check; the GPU never re-wedged.

## compose_demo.sh usage (once a hero CSV + a chase frame dir exist)

```
scripts/compose_demo.sh <hero_flight_csv> <chase_frames_dir>
```

For a clean composite, `<chase_frames_dir>`'s frame COUNT should roughly
match what `render_hud.py --fps 30` will produce from the CSV (duration *
30) — `compose_demo.sh`'s ffmpeg hstack pairs frames BY INDEX ONLY with no
duration/shortest clamp, so a much-longer chase dir makes the HUD pane
freeze on its last frame for the overrun (this bit us on this task's first
composite attempt: 362 chase frames vs 167 HUD frames produced a ~7s frozen-
HUD tail; fixed by capping the chase frame dir to the HUD's own frame count
before composing — see `chase_frames_composite/` above). Also re-read the
"Sync gotcha" methodology below before pointing this at a fresh raw chase
capture.

## Sync gotcha: chase-camera capture start vs CSV `t_sim` origin

`scripts/demo_capture_frames.py` is started (deliberately, to not miss
takeoff) a few seconds before `m4_intercept.py` begins logging, so its
`manifest.csv`'s `sim_t` origin doesn't match the flight CSV's `t_sim[0]`.
Fix: find the chase-manifest row closest to the CSV's own (possibly
trimmed) `t_sim[0]`, then write a trimmed + renumbered copy of just that
tail (`frame_000000.png` = the matched frame) before running
`compose_demo.sh`. Verified this run by pulling a still at the CPA
timestamp and confirming the HUD's readout matches the chase pane.

## Ground-truth-anchored chase-camera pose (unchanged this session)

`worlds/apriltag_demo.sdf`'s `demo_chase_cam` static model pose is
untouched from the first capture: `14.900 -7.612 3.388 0 0.262 -2.647`.
**New finding this session**: this fixed FOV does not actually contain
EITHER drone until ~1s into DASH (both spend TAKEOFF/CUE_WAIT well outside
its frame) — worth knowing before assuming a wider capture window buys more
usable footage; see fix 3 above for how this changed the trim strategy.

## GPU / boot discipline note (this session)

2 sim boots. Both checked `sudo dmesg | grep dxgkio_escape` immediately
post-boot (camera-topic-publish check, `gz topic -e -n1`, before any other
work) and again at teardown; every cluster observed correlated with a PRIOR
boot's delayed teardown window by timestamp, never a live boot's active
window. Boot 1 flew but the `m4_intercept.py` process itself crashed
(mavsdk_server SIGABRT, unrelated to the GPU) before reaching DASH; boot 2
(after a ≥30s cooldown) flew clean end-to-end. Well within the ≤4-boot
budget.
