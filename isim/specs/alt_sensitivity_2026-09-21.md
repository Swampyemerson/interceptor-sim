# Chase-only altitude-difference sensitivity (builder question, 2026-09-21)

How much does a target-altitude error cost the chase-only concept? Native
pursuit prototype, rear tag, scatter ON (all errors), n=50/point, paired
seeds 0..49, `Scenario(concept="pursuit", target_alt_offset_m=X)`. A
descriptive sweep of an existing lever (reference points already on record:
ADR-0103 hardened gave nominal 82%, -2 m 78%, +3 m 64% under slightly
different defaults).

| target alt offset | inside 0.35 m | median miss |
|------------------:|--------------:|------------:|
|              -2 m |   43/50 (86%) |     0.199 m |
|              -1 m |   37/50 (74%) |     0.236 m |
|               0 m |   44/50 (88%) |     0.177 m |
|              +1 m |   42/50 (84%) |     0.211 m |
|              +2 m |   38/50 (76%) |     0.228 m |
|              +3 m |   21/50 (42%) |     0.388 m |

READ: flat within run noise from -2 to +2 m (74-88%; the -1 m point sitting
BELOW the -2 m point is the noise scale at n=50, ~±5/50), then a real cliff
at +3 m ABOVE. It is the high side that breaks, not the low side — the
mechanism is not traced here; plausible candidates are the up-tilted camera
FoV ceiling during the close and climb authority, and attributing it needs
the per-tick trace, same instrument as the parity follow-up. CONTRAST with
the retired sprint concept: ±0.25 m of height error took it 13/16 → 3/16
(budget ~±0.1 m). Chase-only widens the usable altitude-error band by
roughly an order of magnitude and a half — this is the measured basis for
the ADR-0102 requirement "intercept despite a different altitude".
