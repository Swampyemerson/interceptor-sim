# Subpixel-centroid bearing A/B — pre-registration (does it break the ~1.5 m floor?)

Written 2026-07-11 BEFORE the arms fly. ADR-0070 proved the ~1.5 m weave miss is
terminal-BEARING-NOISE-floored (box-center jitter), NOT time/acquisition-limited.
ADR-0071 replaces the published terminal bearing with a darkness-weighted subpixel
centroid (default-off, byte-identical). Does it tighten the miss?

## Arms (weave 12 m/s, adopted deployment config, level camera, master-seed 42, PAIRED, n=8)
- **A (control):** box-center bearing (`MARKERLESS_TRACK_SUBPIXEL=0`, default).
- **B (treatment):** subpixel centroid (`MARKERLESS_TRACK_SUBPIXEL=1`).
Only the env var differs -> the paired A/B isolates the bearing refinement. No
tilt (ADR-0070: tilt hurts). Idle load, one sim, sim_kill by path.

## Pre-registered verdict (PRIMARY = paired per-seed terminal CPA, B - A)
- **SUBPIXEL HELPS (adopt)** iff B tighter than A: paired median delta <= -0.15 m
  AND better on >= 5/8 seeds AND Pk@2.5 non-regressed. -> the real lever for
  < 1.5 m is found; extend to n=16, then n=72 (ADR-0064) before a headline.
- **NULL** iff |median delta| < 0.15 m / no clear sign -> the box-center wasn't the
  binding noise (the range channel or the guidance filter is), re-scope.
- **HURTS** iff B worse -> the centroid is biased (loose box picks background/horizon);
  tune `MARKERLESS_TRACK_SUBPIXEL_PCTL` or the dark-blob method, re-run.
- n=8 is a SCREEN (~1 m single-flight noise); a clear win must survive n=16.
Honesty: camera pixels only (no cue, no gt); the tracker/gating stay box-center.
