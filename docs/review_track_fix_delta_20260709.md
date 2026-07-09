# Fable re-review verdict — detect-then-track fix delta (2026-07-09, pre-commit)

**Overall: CONDITIONAL COMMIT.** All four prior majors genuinely fixed and
mutation-verified; `--track` default-OFF; no honesty-boundary breach. But two
new defects sit exactly on the anti-phantom re-acquire gate and MUST be fixed
before any A/B conclusion or ADR-0058 finalization.

## Per-fix verdicts
1. **Honesty tripwire: VERIFIED.** 15/15 tests pass; mutation-tested (latch-deleted,
   direct .cue_pos read, injected gt_ read — all caught). Every cue read goes via
   legal_cue_pos() (detect_track.py:232,267). Fail-safe semantics real: thread death →
   meas stale 0.4 s → detected=False → hold/breakoff/abort ladder (m4:283 MEAS_STALE_S,
   TERMINAL_HOLD_MAX_S=1.0, LOST_TAG_ABORT_S=5, DASH 20 s).
2. **Re-acquire gate: PARTIAL** — logic right (cam_track updated every tick m4:2709-2712,
   camera-only post-latch verified, None→coast safe) but see defects #1/#2 below.
3. **Classifier: VERIFIED** (analyze_track_ab.py:86-100 boundaries exact; terminal
   restriction to ENGAGE rows correct; three-way Pk present).
4. **KCF refusal: VERIFIED** at __init__ (detect_track.py:171-175) — caveats #4 below.
5. **Minors: VERIFIED** (--track requires --handoff; _init_tracker keeps old lock on
   failure; Measurement.source defaulted trailing field, all 6-positional sites valid;
   class_name unified).

## New defects (severity-ordered) — fix #1/#2 BEFORE headline arm
1. **Coast aging on WALL time** (detect_track.py:250 via markerless_loop.py:132
   time.monotonic) — ADR-0009 violation; radius grows 2-3× too fast per sim-second
   under load. Plumb sim time or scale the rate.
2. **REACQ cap 20 m > ~18 m canonical phantom offset** — gate stops the documented
   phantom only for coasts < ~2 s wall. Lower cap below ~15 m or justify in ADR.
3. Static latch pin bypassable by an appended second assignment (mutation-proven);
   runtime belt covers it — tighten pin to "ALL assignments match" when convenient.
4. KCF refusal = silent detector-thread death mid-run (constructed inside the thread,
   markerless_loop.py:119); unknown MARKERLESS_TRACK_KIND values silently coerce to
   CSRT (_new_tracker:277-278). Validate the env var at parse/startup.
5. No detector-thread liveness check/log — honesty-raise deaths visible only as stderr
   traceback + timeout abort. Add is_alive() check or a death log line.
6. **ADR-0058 is NOT in docs/decisions.md** (grep count 0) — the "draft staged" claim
   was stale. Write it (including MARKERLESS_TRACK_REACQ_* knobs 8 m/5 m/s/20 m,
   currently only in code comments/help text) as part of the commit.

## Commit terms
Fix defects 1+2 (and re-run nothing yet — they change --track behavior, so the
HEADLINE ARM must fly post-fix code); write ADR-0058; stage SPECIFIC paths only —
exclude yolo11n.pt, .claude/settings.local.json, and the v3-dataset files
(docs/seeker_v3_dataset_plan.md, scripts/seeker_v3_capture.py — separate commit).
