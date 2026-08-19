# Demo video pipeline (current)

The portfolio demo is assembled **offline** — no Gazebo/GPU at render time — by
`scripts/build_demo.py` from a captured hero flight (frames + its per-tick CSV):

```
.venv/bin/python scripts/build_demo.py          # onboard MP4+GIF + chase MP4
.venv/bin/python scripts/build_demo.py --fast   # onboard MP4 only (quick iteration)
```

**PRIMARY cut = the onboard (seeker-POV) view** with the FPV-OSD HUD overlaid
(`scripts/render_hud.py --layout overlay`), ~20 s: title card → real-time
establish on the external cue → slow-motion dash-in → the HANDOFF beat
(`SENSOR` flips to `CAMERA-ONLY`, "DATALINK DENIED" callout keyed off
`ext_fresh` 1→0) → terminal → a single-ring proximity-fuse close-out, then cut
to black. **SECONDARY = a ~4 s chase-camera wide.** Retiming is disclosed
on-screen (REAL-TIME / SLOW MOTION pills); slow-mo in-betweens are honest
cross-dissolves; every HUD widget traces to a CSV column, and the GT-derived
readouts carry their scoring-only footnote.

`scripts/compose_demo.sh` is the older single-flight HUD-panel path and is kept
for the `sidebar` HUD layout it depends on.

## Status (why there is no shipped video right now)

- The 2026-07-07 assets (onboard MP4/GIF, chase MP4) were **retired by builder
  ruling 2026-08-10** (issue #2): their closing caption reads "CPA 0.63 m <
  1.5 m lethal radius", which after ADR-0084 ratified the **0.35 m ram radius**
  reads as a ram-kill claim it is not (0.63 m is a net-class number).
- `build_demo.py`'s `fuse_banner()` is **already fixed** — it now renders the
  net-class labelling ("NOT a ram kill (ram/contact radius 0.35 m, ADR-0084)").
- A caption-only re-render is impossible: the hero flight's CSV
  (`logs/m4_intercept_pronav_20260707T211601Z.csv`) was rotated off disk (the
  result survives in the ADR-0032 addendum). **Shipping a new demo = flying a
  new hero flight on the sim machine, then `build_demo.py`** — a re-shoot, not
  a caption pass. The T25 cut was separately re-rendered with the corrected
  labelling (`scripts/video/t25_render_plan.md`).

Full capture-session history, HUD widget provenance, and the de-cringe design
notes live in git history (`demo_out/README.md`, removed 2026-08-19) and
`docs/decisions.md` ADR-0032/0066/0084.
