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
| 1 | Ground stereo detection (both rig cams, DETECT flash) | **NEW render** — stereo rig capture pass (§B). Needs a small harness extension (§B gap 1) + the shot-1/2 compositor (§B gap 3) | ⏳ sim-gated |
| 2 | Track established → cue up (σ_R bar, latency readout) | **NEW render** — same §B pass + `scripts/ground_station/station.py` replay audit CSV | ⏳ sim-gated |
| 3 | Launch + dash begins (onboard cam) | **NEW render** — hero-seed re-fly with onboard frame capture (§A) | ⏳ sim-gated |
| 4 | Terminal acquisition — detect-then-track (NN ACQUIRE → CSRT box) | **NEW render** — same §A flight | ⏳ sim-gated |
| 5 | HANDOFF — cue lamp goes dark (the honesty beat) | **NEW render** — same §A flight | ⏳ sim-gated |
| 6 | Terminal slow-mo to intercept + CPA close-out | **NEW render** — same §A flight, retimed at assembly (§A step 5) | ⏳ sim-gated |
| 7 | Chase-cam replay | **NEW render** — same §A pass IF the chase camera is added to the markerless world first (§C gap) | ⏳ sim-gated + world edit |
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

1. **Weave replay:** `rig_snapshot_capture.py` currently replays the `line`
   path only. Either (a) extend it with a `--poses-from <csv>` teleport
   schedule built from the hero flight's `gt_tag_x/y/z,t_sim` columns (the
   storyboard's "weave trajectory replayed from the hero flight's target
   schedule"), or (b) stage the line path and caption shot 1–2 as the
   straight-line regime. (a) matches the storyboard; (b) is arguably MORE
   honest since the stereo pipeline's validation IS straight-line
   (ADR-0053). Decide at render time; either way constraint #3's mock-cue
   disclosure line stays on the end card and in the shot-1/2 footnote.
   Staging: place the replay inside the validated 50–160 m band (first firm
   track ~120–150 m per the storyboard, `docs/stereo_design.md` floor).
2. **Detector choice:** `ground_station/detect.py` defaults to
   `ground_v1.onnx`; `ground_v2.onnx` exists in `scripts/seeker/weights/`.
   The storyboard says "ground_v2 or its v3-era successor" — pass the
   chosen weights explicitly and note it in the shot's caption/log.
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

## §C — Chase view (shot 7)

`worlds/markerless.sdf` has **no chase camera** — only `apriltag_demo.sdf`
carries the `demo_chase_cam` model (a plain camera sensor publishing an
Image topic; that's how the M5-era chase footage was captured). Before the
§A pass, create `worlds/markerless_demo.sdf`: copy `markerless.sdf`, rename
the world to `markerless_demo`, and paste in `apriltag_demo.sdf`'s
`demo_chase_cam` `<model>` block (lines ~336–383). Then the §A pass boots
with `PX4_GZ_WORLD=markerless_demo` / `INTERCEPTOR_WORLD_NAME=markerless_demo`
(topics gain the new world prefix) and a second passive capture runs:

```bash
.venv/bin/python scripts/demo_capture_frames.py \
    --topic /world/markerless_demo/model/demo_chase_cam/link/<link>/sensor/chase_cam/image \
    --out demo_out/t25/chase_frames &     # exact topic: gz topic -l after boot
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
