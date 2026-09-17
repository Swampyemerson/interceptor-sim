# Spec v7: the HYBRID the builder ruled for -- fly-by first, turn-around to slow arrival if missed

Builder ruling 2026-09-17: contact radius stays 0.35 m; two tags on the target are fine;
direction = sprint-and-fly-by first and, if it misses, a rapid turn-around into the slow
arrival; keep improving the fly-by against the same requirement. Lens corrected from the
order log: innomaker OV9281, 118 deg HFOV => use `cam_fx_px=385` as the nominal from now on
(re-measured, all errors on, rear tag: 82% nominal / 59% aim 20 deg / 75% target +2 m).

## Build `concept="hybrid"` in isim (isim/concepts.py + scenario wiring)
Phase S (sprint): the same open-loop sprint the fly-by flies (heading from
`resolve_preflight_heading` + aim error, dash speed 16 m/s x sprint_scale, accel-limited like
the real flight code's dash), holding the believed altitude. While sprinting, use any tag
decodes to start the target filter (same filter as pursuit), and if the filter is healthy in
the last ~1 s before the pass, steer the pass (lateral + vertical correction limited to what
keeps the tag in view) -- an improved fly-by, not a blind one.
Phase T (turn-around): when range starts growing (passed) or the sprint timer expires without
contact, do NOT stop: brake and turn toward the target's predicted position using the filter
if it has one, else the pre-flight belief; then hand over to the existing pursuit Phase A/B.
Report the time lost in the turn-around and how far behind the vehicle ends up.
Scoring: closest approach over the whole window (as pursuit). Record per run whether contact
(<= 0.35 m) happened on the FIRST pass or in the chase, and the time of contact.

## Measure (all v5 errors on, fx 385, tilt 12, n = 100, tag facings rear and rear_dual35)
Requirement grid: aim error 0-30, altitude offset -2..+3, target speed 0/4/9, sprint_scale
1.0/0.5/0.0, weave. Columns: % contact first pass, % contact total, median time to contact.
Compare against concept="pursuit" and concept="flyby" on the same cells.

Rules as before (loose test bounds, print measured values, report every tuned value, say what
you doubt, truth never reaches guidance, parameters travel through Scenario fields).
Verify with `.venv/bin/python -m pytest isim/tests -q`.
