# T25 render plan — shot → source map + exact commands

Companion to `docs/t25_storyboard.md` (the 9-shot storyboard + honesty
constraints). This file answers ONE question per shot: **does it come from
existing committed data, or does it need a fresh sim render — and if fresh,
what is the exact command?** Offline tooling is already built
(`scripts/video/hud_overlay.py`, `make_t25_cards.py`, `assemble_t25.sh` +
`t25_shots.conf`); the sim renders are the only remaining gated work and are
sequenced by the builder (idle machine, ONE sim at a time, RTF ≈ 1 — no
`PX4_SIM_SPEED_FACTOR` on render passes; batch-hygiene rules apply).

**Sequencing gate (storyboard):** render AFTER the v3 detector evaluation
completes — the demo flies the best detector. A guidance-fix build is
touching `m4_intercept.py`/`detect_track.py` at the time of writing; renders
also wait for that to land and re-validate. Nothing below boots a sim.

## Shot → source table

| # | Beat | Source | Status |
|---|------|--------|--------|
| 0 | Title + architecture strip | **EXISTING / offline** — `make_t25_cards.py` → `demo_out/t25/cards/shot0_title.png` | ✅ buildable now (validated) |
| 1 | Ground stereo detection (both rig cams, DETECT flash) | **NEW render** — stereo rig capture pass (§B). Harness extension BUILT + dry-run-validated (§B gap 1 ✅); still needs the shot-1/2 compositor (§B gap 3, gated on the capture existing) | ⏳ sim-gated |
| 2 | Track established → cue up (σ_R bar, latency readout) | **NEW render** — same §B pass + `scripts/ground_station/station.py` replay audit CSV | ⏳ sim-gated |
| 3 | Launch + dash begins (onboard cam) | **NEW render** — hero-seed re-fly with onboard frame capture (§A) | ⏳ sim-gated |
| 4 | Terminal acquisition — detect-then-track (NN ACQUIRE → CSRT box) | **NEW render** — same §A flight | ⏳ sim-gated |
| 5 | HANDOFF — cue lamp goes dark (the honesty beat) | **NEW render** — same §A flight | ⏳ sim-gated |
| 6 | Terminal slow-mo to intercept + CPA close-out | **NEW render** — same §A flight, retimed at assembly (§A step 5) | ⏳ sim-gated |
| 7 | Chase-cam replay | **NEW render** — same §A pass; chase-cam world variant BUILT + gz-sdf-validated (§C ✅) | ⏳ sim-gated |
| 8 | Results end card (ADR-0058 numbers + disclosures) | **EXISTING / offline** — `make_t25_cards.py` recomputes from committed `logs/mc_t21_trackgate_weave12_r2.csv`, asserts byte-match vs ADR-0058 → `demo_out/t25/cards/shot8_endcard.png` | ✅ buildable now (validated) |

**Existing committed flight data backing the shots** (validation + numbers,
NOT the video frames): the r2 arm `logs/mc_t21_trackgate_weave12_r2.csv`
(n=16, pooled Pk@2.5 16/16, median 2.11 m, max 2.48 m; camera-terminal
14/14, median 2.03 m) and its hero flights —
run 1 (r2l, miss **1.09 m**, handoff 7.5 m, first det 15.5 m):
`logs/m4_intercept_pronav_20260709T232447Z.csv`;
run 13 (r2l, miss **1.17 m**, handoff 9.8 m, first det 15.7 m):
`logs/m4_intercept_pronav_20260709T233852Z.csv`.
The committed CSVs were used to validate `hud_overlay.py` offline; the video
frames for shots 3–7 must come from a re-fly, and **the HUD for those frames
must be rendered from the RE-FLY's own CSV** (frames + CSV = same flight,
the `build_demo.py` discipline). The committed CSVs remain the end card's
statistics source and the reference for what the re-fly should look like.

---

## §A — Onboard hero re-fly (shots 3–6, + 7 in the same pass)

One flight, captured passively while it flies. Everything below is verbatim
from the validated r2 arm's plumbing (`scripts/mc_deployment_arm.sh` env +
`logs/mc_t21_trackgate_weave12_r2_stdout.log` argv, run 1; mc_batch forwards
the weave path to the mover through `INTERCEPTOR_*` env).

> **⚠️ CONFOUND FOUND 2026-07-10 (first capture attempt) — passive capture
> PERTURBS the flight; the archived hero seeds do NOT re-fly clean under it.**
> Flying the hero re-fly with TWO concurrent passive captures (onboard + chase)
> produced "User callback queue slow" gz-transport warnings throughout and
> pushed BOTH sanctioned seeds out of the clean envelope: hero seed (256788)
> detected fine in DASH (174/394 rows) but never CLOSED to the 10 m handoff gate
> → abort; backup seed (167415) latched handoff at 9.05 m then broke off
> (miss 6.62 m). The archived r2 flights (1.09 / 1.17 m clean) were flown in a
> BATCH with NO concurrent frame capture — the second/third topic subscriber
> starves m4's own camera+guidance loop and shifts timing enough to change the
> outcome. The demo world is byte-identical scene (only adds the chase cam) — NOT
> the confound. Code-version drift is also in play: the archived reference beats
> are PRE-FIX-A; re-flies run current post-FIX-A code. **Remedies (pick one when
> resuming §A): (1) capture ONBOARD-ONLY (drop the chase pass; halves the
> contention) and legitimately SELECT a clean demo flight from several r2-arm
> seeds — allowed by the storyboard rule since the end-card stats come from the
> ARM, not the chosen flight; (2) if contention still binds, dump frames from
> m4's OWN subscriber (an m4 change) instead of a second subscriber; (3) or a
> deterministic trajectory-replay render like §B uses for the rig.** Chase
> (shot 7) becomes a SEPARATE lighter pass, not concurrent with the onboard hero.
>
> **RESOLVED 2026-07-10 (same session): remedy (1) WORKED.** Onboard-only capture
> on the plain `markerless` world (ONE rendered camera, matching the clean
> baseline) + selecting a clean demo flight from the arm seeds produced a clean
> captured hero flight: **r2 run 0 (cue 26226, path 857665), CPA 2.076 m, real
> handoff @ 9.95 m, normal breakoff** (`logs/m4_intercept_pronav_20260710T232121Z.csv`,
> 1097 onboard frames in `demo_out/t25/onboard_frames/`). Shots 3–6 built + a
> 17.7 s partial cut (`demo_out/t25/t25_demo.mp4`, shots 0/3/4/5/6/8; shot 6 now
> 8× slow-mo). NOTE: this flight's early-handoff closed acquisition+handoff on the
> same frame → shots 4/5 are honestly tight (9/7 frames). STILL OPEN for a full
> render: shots 1–2 (ground-stereo, §B compositor ~half-day, not built), shot 7
> (chase — separate lighter pass), and the NEW tilt shot (nominal availability
> A/B, needs its own capture; NOT a recovery claim — ADR-0068 Stage-2 NULL).

```bash
cd ~/interceptor-sim
# 1) deployment-config env (ADR-0058, mirrored from mc_deployment_arm.sh)
export MC_WORLD=markerless
export MARKERLESS_NN_WEIGHTS=$PWD/scripts/seeker/weights/drone_finetuned_v2.onnx
#   ^ swap for the v3 winner IF the v3 evaluation adopts it — and if so, the
#     end card must keep attributing the n=16 arm stats to the v2-config
#     ADR-0058 arm (or a fresh v3 validation arm) — never cross-label.
export S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"
# weave path -> the mover (run 1 hero seeds shown; run 13 backup, verbatim from
# the r2 CSV row 13: --target-start=6.500,30.076,0.5 --target-vel=0.000,-12.000
# --cue-seed 167415, INTERCEPTOR_PATH_SEED=289338, miss 1.172 m)
export INTERCEPTOR_TARGET_PATH=weave INTERCEPTOR_PATH_SEED=778181
export INTERCEPTOR_WEAVE_PERIOD_S=4.0 INTERCEPTOR_WEAVE_LAT_SPEED=3.0
export INTERCEPTOR_WORLD_NAME=markerless INTERCEPTOR_TARGET_MODEL=fpv_target_markerless

# 2) boot sim (mc_batch.sh line ~585 pattern; HEADLESS=1 is fine — all T25
#    footage comes off gz camera TOPICS, not the GUI; GPU still renders cams)
cd ~/PX4-Autopilot && env PX4_GZ_WORLD=markerless \
    GZ_SIM_RESOURCE_PATH=$HOME/interceptor-sim/models HEADLESS=1 \
    make px4_sitl gz_x500_mono_cam   # wait for "Ready for takeoff!"

# 3) passive captures (safe alongside the flight: subscribe-only, see
#    scripts/demo_capture_frames.py docstring) — start BEFORE the flight
.venv/bin/python scripts/demo_capture_frames.py \
    --topic /world/markerless/model/x500_mono_cam_0/link/camera_link/sensor/imager/image \
    --out demo_out/t25/onboard_frames &        # onboard seeker POV (shots 3-6)
# + chase capture in the same pass if §C's world variant is in place

# 4) the flight — EXACT r2 run-1 argv (hero: miss 1.091 m; backup run 13
#    argv in the r2 stdout log lines ~1560+; venv = .venv-seeker, as batched)
.venv-seeker/bin/python scripts/m4_intercept.py --fpv --handoff --law pronav \
    --seeker markerless --target-start=6.500,30.025,0.5 --target-vel=0.000,-12.000 \
    --cue-seed 256788 --dash-speed 16 --early-handoff --cue-velocity \
    --dash-unclamp --fuse-midcourse --track --handoff-cue-gate 8
# note the flight CSV path it prints: logs/m4_intercept_pronav_<STAMP>Z.csv

# 5) OFFLINE from here. HUD-composited frames from the RE-FLY's own CSV:
.venv/bin/python scripts/video/hud_overlay.py logs/m4_intercept_pronav_<STAMP>Z.csv \
    --out demo_out/t25/shot36_frames \
    --frames demo_out/t25/onboard_frames/manifest.csv --rt-label
#   it prints the beats (first NN acquisition / handoff latch / CPA sim-times
#   + rows) — cut shots 3/4/5/6 at those sim-times. Per-shot clips by frame
#   range (frame idx <-> sim_t via demo_out/t25/onboard_frames/manifest.csv +
#   demo_out/t25/shot36_frames/hud_audit.csv):
ffmpeg -framerate 30 -start_number <N0> -i demo_out/t25/shot36_frames/frame_%06d.png \
    -frames:v <COUNT> -vf format=yuv420p -c:v libx264 -crf 19 demo_out/t25/shot3_launch_dash.mp4
#   ... same for shot4/shot5. Shot 6 (terminal slow-mo): cut the clip at 1x
#   here, then set `speed=8` on shot 6's line in t25_shots.conf —
#   assemble_t25.sh retimes by frame duplication AND burns the
#   "SLOW MOTION (retimed)" watermark in the same code path (constraint #4 by
#   construction; avoids a double label). NEVER use ffmpeg minterpolate:
#   invented in-between motion violates the build_demo.py honesty rule
#   (cross-dissolves of real frames are the only allowed in-betweens).

# 6) cards (offline, already validated) + assembly
.venv/bin/python scripts/video/make_t25_cards.py
scripts/video/assemble_t25.sh                 # partial cut, skips missing shots
scripts/video/assemble_t25.sh --require-all   # final publish pass
```

Reference beats from committed run 1 (the re-fly will differ slightly — use
`hud_overlay.py`'s printed beats, not these): TAKEOFF 9.94 s, CUE_WAIT
20.67 s, DASH 21.26 s, first NN acquisition 22.31 s @ 15.5 m, handoff latch
22.84 s @ 7.5 m, ENGAGE 22.97 s, **CPA 23.20 s @ 1.091 m**, BREAKOFF 24.27 s.

**Acceptance:** re-fly is CLEAN (no phantom handoff, real handoff latched,
miss ≤ 2.5 m). If it lands near the arm median (~2.1 m) instead of ~1.1 m,
**ship it anyway** — the end card carries the arm statistics; the video must
not silently cherry-pick beyond choosing a clean flight (storyboard rule).
If it reproduces a no-handoff flight (runs 10/12 in the arm had none),
re-fly the backup seed (run 13's row in the r2 CSV) instead.

## §B — Ground-stereo pass (shots 1–2)

The only genuinely new machinery (storyboard production note). World:
`worlds/stereo_intercept.sdf` (T16 rig at `broadside_160m`). Target-only
replay — no interceptor flight.

```bash
cd ~/PX4-Autopilot && env PX4_GZ_WORLD=stereo_intercept \
    GZ_SIM_RESOURCE_PATH=$HOME/interceptor-sim/models HEADLESS=1 \
    make px4_sitl gz_x500_mono_cam
# capture: scripts/rig_snapshot_capture.py (T16 harness, .venv-seeker) —
# teleport-replay of the target with BOTH rig cameras captured per pose.
# Ground-station replay over the capture (detect -> triangulate -> track,
# emits the cue + an audit CSV in the logs/ground_station_*.csv schema):
#   .venv-seeker/bin/python scripts/ground_station/station.py ... (see its header)
```

Three flagged gaps before shots 1–2 can render:

1. **Weave replay — ✅ BUILT (2026-07-10, sim-free pass), option (a):**
   `rig_snapshot_capture.py` now takes `--poses-from <flight_csv>` (teleport
   schedule from the flight's `t_sim` + `gt_tag_x/y/z`, nearest-row resample
   at `--rate`, NO interpolation — no invented poses), plus
   `--t-start/--t-end/--pre-roll-s` windowing, `--x/y/z-shift` staging, and
   `--dry-run` (schedule + staging metrics offline, gz untouched). The line
   path is byte-identical when the flag is absent (`--smoke`/full dry-runs
   verified; index.csv gains an appended `t_sim_source` column ONLY in
   replay mode). Dry-run against the committed run-1 hero CSV: 47 poses @
   10 Hz over the mover window, `--x-shift -25` stages the corridor at
   **133.2–138.5 m** from the broadside_160m rig — inside the validated
   50–160 m band and the storyboard's ~120–150 m first-firm-track note
   (zero shift = 158–163 m, the T16-native band, at/over the validated
   edge — use the shift). Option (b) (line path) remains available as the
   fallback via the unchanged default mode. Constraint #3's mock-cue
   disclosure line stays on the end card and in the shot-1/2 footnote.
   One honest framing note for the edit: from the rig the weave's lateral
   component lies mostly ALONG its line of sight (range oscillation), so
   the rig view itself looks near-straight — the weave is visible in the
   shot-2 inset map / track trace, not the raw rig image. Do not caption
   the rig image as "visibly weaving."
2. **Detector choice — ✅ DECIDED: `ground_v2.onnx`** (pass explicitly:
   `--weights scripts/seeker/weights/ground_v2.onnx`; input 960×960, same
   as v1, so the default `--imgsz 960` holds — verified via onnxruntime).
   Why v2 is the honest, non-cherry-picked choice: ADR-0055's decision (1)
   makes ground_v2 the RECOMMENDED ground detector, verifier-confirmed,
   with held-out-range generalization proof (70/130 m never trained, bias
   −0.010/−0.14 m); ground_v1 is a single ~160 m operating point
   (ADR-0049 necessary-not-sufficient) with a −6.3 m systematic bias at
   50 m and a non-monotonic box head out-of-domain (ADR-0053) — picking v1
   and a range band where it happens to behave would be exactly the
   cherry-pick the storyboard forbids, while v2 is valid across the whole
   50–160 m staging envelope and its adoption (ADR-0055) predates this
   video. Two caption obligations: (i) name the weights on the shot
   ("ground detector: ground_v2, multi-range, ADR-0055"); (ii) the
   historical T18/T19 gates were driven by v1 (ADR-0049/0053 provenance) —
   the shot-2 pipeline caption cites the pipeline, not v1 specifically, so
   no conflict, but don't claim v2 drove those gates. Note there is NO
   "v3-era successor" for the GROUND detector: the pending v3 evaluation is
   the ONBOARD seeker (`drone_finetuned_v3.onnx`, terminal 1.5–50 m
   domain) — it gates the §A re-fly config, not this choice.
3. **Shot-1/2 compositor (offline, NOT yet built):** a side-by-side
   left|right rig view with the DETECT flash + sim-time stamp at each
   camera's first-detection instant, per-camera NN boxes, the
   SEARCHING→TRACKING state lamp, and shot 2's σ_R uncertainty bar (∝ R²,
   measured exponent 2.003, ADR-0053) + detection-to-track latency from the
   ground-station audit CSV. It is a ~half-day offline build in
   `scripts/video/` once ONE rig capture + ground-station CSV exist to
   composite against — the columns/frames it needs don't exist yet, which is
   why it isn't built alongside `hud_overlay.py`. **Mock-cue disclosure
   (constraint #3) applies to these shots**: the rendered rig pass is the
   real pipeline, but the cue that steered the §A flight is the calibrated
   mock — one footnote line, already worded in the storyboard.

## §C — Chase view (shot 7) — ✅ WORLD BUILT (2026-07-10, sim-free pass)

`worlds/markerless_demo.sdf` now exists: byte-identical to
`worlds/markerless.sdf` (physics/scene/target include unchanged — verified
by diff) except the world name and the added `demo_chase_cam` model
(`apriltag_demo.sdf`'s exact 960×540 @ 30 Hz sensor block — the RTF-safe
resolution per that file's RESOLUTION FINDING). `gz sdf -k` PASS (invoke
with `SDF_PATH=$PWD/models`, not GZ_SIM_RESOURCE_PATH — the bare CLI has no
gz-sim find callback). Symlinked into
`~/PX4-Autopilot/Tools/simulation/gz/worlds/` (the established pattern), so
`PX4_GZ_WORLD=markerless_demo` resolves.

**The pose was NOT copied** — apriltag_demo's chase pose aims at the OLD
engagement point (6.4, −12.2), which is ~92° off THIS flight's CPA (checked
against the committed hero CSV; it would film empty ground). The new pose
`14.445 2.440 3.431 0 0.262 2.533` is re-derived by the same documented
recipe from `logs/m4_intercept_pronav_20260709T232447Z.csv`: 10 m broadside
of the dash bearing, aimed at the CPA midpoint (6.66, 8.72, 0.75), pitch
15°, yaw biased +4° toward the launch origin — full derivation + framing
check (takeoff, dash, CPA, breakoff all inside the 1.7 rad hfov) is in the
world file's header. The re-fly uses the same seeds, so the geometry holds;
fine-tune live via `set_pose` if needed (no reboot — see the header note,
and mind the quaternion gotcha).

§A boots with `PX4_GZ_WORLD=markerless_demo` /
`INTERCEPTOR_WORLD_NAME=markerless_demo` (ALL topics gain the new world
prefix — the onboard capture topic in §A step 3 becomes
`/world/markerless_demo/model/x500_mono_cam_0/link/camera_link/sensor/imager/image`)
and a second passive capture runs (link name confirmed from the world file
— no placeholder):

```bash
.venv/bin/python scripts/demo_capture_frames.py \
    --topic /world/markerless_demo/model/demo_chase_cam/link/link/sensor/chase_cam/image \
    --out demo_out/t25/chase_frames &     # sanity: gz topic -l | grep chase_cam
```

Chase HUD: `hud_overlay.py --frames demo_out/t25/chase_frames/manifest.csv`
works as-is (the overlay is camera-agnostic; the seeker track box will be
drawn from onboard-camera measurements, so consider cropping it out of the
chase layout later or simply accept the lamps/readouts — decide at edit).
Trajectory ribbon + labeled CPA marker: `render_hud.py --layout sidebar`'s
mini-map already draws both if a side-panel treatment is preferred.

## Honesty checklist pointers (enforced by the tooling)

- Cue lamp is a pure function of the one-way latch index — it cannot
  re-animate after handoff (`hud_overlay.py`, constraint #1); verify with
  `demo_out/t25/*/hud_audit.csv`: `awk` for `LIVE` rows after the latch tick.
- No caption says "jam-proof"; the handoff wording is the storyboard's
  (constraint #2); the end card carries the HELD-claim line verbatim.
- Mock-cue provenance: per-frame footnote (shots 3–6) + end card
  (constraint #3); shots 1–2 add their own footnote at composite time.
- Retiming disclosure is structural: `assemble_t25.sh` cannot slow a clip
  without burning the watermark; `hud_overlay.py --slowmo` stamps per-frame
  (constraint #4).
- CPA overlay is labeled "criterion, not a modeled collision", GT
  scoring-only (constraint #5), and never appears before the CPA tick.
- Pre-publish frame audit (storyboard checklist): scan `hud_audit.csv` for
  every frame from latch to CPA, not a sample.

---

## Readiness (2026-07-10 sim-free pass — all buildable-now blockers cleared)

Per-shot state after this pass. "Remaining" = sim-gated steps only, all
sequenced by main behind the v3-eval + guidance-fix gate (top of file).

| # | Shot | Ready? | Remaining sim step(s) |
|---|------|--------|----------------------|
| 0 | Title card | ✅ render-ready | none (offline, already validated) |
| 1–2 | Ground stereo opener | 🟡 harness ready | ONE §B rig-replay boot + capture + station replay (sequence below); then the gap-3 compositor (offline, ~half-day, deliberately built only once the capture exists) |
| 3–6 | Onboard hero beats | ✅ render-ready | the ONE §A re-fly (unchanged §A sequence, but boot `markerless_demo` per §C so shot 7 comes free in the same pass) |
| 7 | Chase replay | ✅ render-ready | same §A re-fly (world variant built/validated/symlinked; second passive capture per §C) |
| 8 | Results end card | ✅ render-ready | none (offline, already validated) |

**What this pass built (no sim booted, nothing committed):**

1. `worlds/markerless_demo.sdf` — chase-cam world variant, `gz sdf -k`
   PASS, pose re-derived for the weave corridor (§C), symlinked into PX4's
   worlds dir. Guidance-fix files untouched.
2. `scripts/rig_snapshot_capture.py` `--poses-from` weave-replay mode +
   `--dry-run` offline validator (§B gap 1); default line path
   byte-identical, `--smoke` dry-run verified.
3. Ground-detector decision for shots 1–2: **ground_v2.onnx** (§B gap 2,
   ADR-0047/0049/0053/0055 chain).

**Exact §B render sequence for shots 1–2 (copy-paste, idle machine, ONE
sim):**

```bash
# 0) OFFLINE rehearsal first (no sim; uses the re-fly's CSV once §A has
#    flown, or the committed run-1 CSV — same seeds => same target track):
.venv-seeker/bin/python scripts/rig_snapshot_capture.py \
    --poses-from logs/m4_intercept_pronav_<REFLY_STAMP>Z.csv \
    --x-shift -25 --pre-roll-s 3 --dry-run

# 1) boot the rig world (per §B / check_t16.sh)
cd ~/PX4-Autopilot && env PX4_GZ_WORLD=stereo_intercept \
    GZ_SIM_RESOURCE_PATH=$HOME/interceptor-sim/models HEADLESS=1 \
    make px4_sitl gz_x500_mono_cam   # wait for "Ready for takeoff!"

# 2) weave replay capture (same flags as the rehearsal, minus --dry-run)
cd ~/interceptor-sim && INTERCEPTOR_WORLD_NAME=stereo_intercept \
    INTERCEPTOR_TARGET_MODEL=fpv_target_markerless \
    .venv-seeker/bin/python scripts/rig_snapshot_capture.py \
    --poses-from logs/m4_intercept_pronav_<REFLY_STAMP>Z.csv \
    --x-shift -25 --pre-roll-s 3 \
    --out logs/rig_captures/t25_weave_replay

# 3) ground-station replay over the capture -> cue + audit CSV
#    (offline mode: --replay-clock; weights per §B gap 2 DECISION)
.venv-seeker/bin/python scripts/ground_station/station.py \
    --capture-dir logs/rig_captures/t25_weave_replay \
    --weights scripts/seeker/weights/ground_v2.onnx \
    --replay-clock --emit-velocity
# keep the audit CSV path it prints — the gap-3 compositor reads it

# 4) sim down; the shot-1/2 compositor build (gap 3) is now unblocked
#    (offline; composites left|right frames + DETECT flash + sigma_R bar
#    from the capture dir + audit CSV)
```

Staging knobs, pre-decided (from the dry-run numbers above): `--x-shift
-25` → 133–139 m from the rig (validated band + storyboard staging);
`--pre-roll-s 3` gives ~3 s of honest empty-frame SEARCHING footage before
the threat's run (shot 1's opening beat; target enters the hfov wedge ~1.5 s
after mover start). Keep `--rate` at the default 10 Hz — that IS the rig
sensor's design cadence (`docs/stereo_design.md`, 10 Hz cue), so the
station replay's latency/track numbers stay authentic; the 10 fps rig
footage plays at assembly via plain frame duplication to 30 fps
(frame-RATE conversion, not retiming — no watermark owed, no invented
frames). Content-length note for the edit: the hero mover only runs ~3.6 s
start→CPA at 12 m/s, so shots 1–2 have ~3 s pre-roll + ~4.6 s of live
track to cut from — if the storyboard's ~11 s combined target feels long,
hold shot 2 on the track/σ_R/latency readouts (compositor cards), don't
slow the rig footage.

**Station-replay caveats for the operator (known, acceptable):** (i) the
track filter's validation is straight-line (ADR-0053) — the weave replay
exercises it out-of-validation, which is exactly what the storyboard's
constraint-#3 footnote already discloses; treat the emitted track as the
honest visualization it is, NOT as a new validation claim. (ii)
`station.py --dash-direction` filter stays unset (replay rows are labeled
`weave_replay`, not r2l/l2r). (iii) the replay index.csv carries
`t_sim_source` (appended column) tracing every pose to its flight-CSV row
— the compositor should stamp shot-1/2 sim-times from `t_sim_nominal`
(station.py's ADR-0052 rebase semantics, unchanged).
