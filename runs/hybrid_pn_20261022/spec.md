# Pre-registered: look-angle-clamped PIP aim (candidate B) vs pure chase

2026-10-22. Design + criteria: docs/hybrid_guidance_notes_2026-10-22.md
(registered there BEFORE this run; implementation = PursuitConfig.pip_aim +
look_angle_max_deg, default-off, isim suite 197 green after the change).

Arms (concept="pursuit", scatter on, n=8): baseline {} vs
B30 {pip_aim, look 30°} vs B20 {pip_aim, look 20°} vs PIPonly {pip_aim, no
clamp — isolates whether the clamp is what saves PIP}. Speeds {9, 12};
res scales s {1, 2} (same-FOV, cam fields); geometries: canonical
(cross 6.5 / lead 16.2) and far (cross 150 / lead 477).

ADOPT-FOR-GAZEBO iff a B arm beats baseline median miss at BOTH speeds at
s=1 with >= equal med_decoded (pointing metric). NULL = chase stays; the
answer to the builder is "the literature's fix exists but does not pay at
our scale," with numbers.

## RESULT (2026-10-22, run as registered — NULL, chase stays)

Canonical geometry (the informative cells): baseline pure chase wins EVERY
cell — 0.170/0.228 m (s=1, 9/12 m/s) vs B30 0.622/0.347, B20 0.362/0.439,
PIPonly 0.496/0.366; same ordering at s=2. The adopt rule (beat baseline at
both speeds at s=1 with >= decodes) is met by NO arm. Key mechanism note:
PIPonly decoded 5-6x MORE frames than baseline (the lead course holds the
target in the decodable band longer) and still missed worse — in THIS
implementation (yaw always tracks the LOS) the failure is the PIP aim
itself under bearing-only tgo noise, not camera pointing; the look-angle
clamp partially repairs it (B20 < PIPonly at 9 m/s) but never past chase.
Far-geometry cells were UNINFORMATIVE (all arms identical, 10-14 m, engage
50-75%): the pursuit CONCEPT lacks the approach-dash Phase A that the
flight-code terminal has — do not read them as a law comparison.

Verdict per the registered null branch: the literature's fix exists but
does not pay at our scale; pure chase remains the law. The pro-nav answer
is now a two-study chain (runs/res_guidance_20261021 + this): neither
resolution nor FOV-constrained PIP aim beats chase for this vehicle.
