# Pre-registration: stern-approach geometry vs cue error (builder question, 2026-10-21)

**Question (builder, verbatim concern):** after the blind dash, the turn to
the target is aimed by the dead-reckoned pre-flight belief. Does approaching
from *behind the target's velocity vector* (stern conversion) make acquisition
robust to cue error — and is the current trail distance (`d_behind_m = 8`)
the right number?

**What exists already:** Phase A aims at `pos_belief − d_behind_m·track_dir`
with velocity matched to belief (`isim/concepts.py:742-754`); the 500 m video
take hr5 flew this with injected cue error (CPA 0.087 m, best of 5; lr3 shows
the failure mode — cue draw past the decode cliff → SAFE abort).

**Design (isim, fast sim; Gazebo only if a decision rides on it):**
- Sweep: `pursuit_d_behind_m` ∈ {4, 8, 16, 24} × `dash_aim_behind_m`
  ∈ {0, 8, 16, 32} m × cue error draws (Scatter on, plus `aim_error_deg`
  at {0, 4, 8}°), n ≥ 8 paired seeds per cell, concept="hybrid".
- *AMENDED before flying (2026-10-21, no data taken):* the registered
  "approach bearing offset" axis is replaced by `dash_aim_behind_m` — the
  implementable stern lever (the dash aims astern of the believed track;
  commit 18cc7b2). A true bearing-offset arc is a router (stage-3) feature;
  if this sweep shows the astern lever matters, bearing earns its plumbing.
- Target: the isim-earned hybrid operating point (9 m/s crossing, fx 933,
  6.5 m cross range — the concept-test canon; *amended before flying* from
  "the 12 m/s crossing", which is the GAZEBO latency study's point, not an
  isim-earned one). Vehicle: the FITTED isim envelope (default dash). *AMENDED before flying:* the hr-class 36 m/s
  envelope is outside the isim vehicle fit's earned regime (regime rule);
  if the geometry lever matters at the fitted envelope, the hr-envelope
  confirmation is a registered Gazebo rung, not an isim extrapolation.
- Metrics: acquisition probability, time-to-acquire after turn, CPA median
  over acquiring flights, abort rate.

**Predictions (written before flying):**
1. Stern (0°) beats beam (90°) on acquisition probability under cue error,
   because trailing geometry keeps bearing error from turning into LOS rate.
2. `d_behind_m` has an interior optimum: too short overshoots under cue
   error, too long starves the terminal of closing time (lr3's cliff).
3. If (1) comes back null — geometry doesn't move acquisition — the binding
   variable is cue error vs decode range alone, and the dash law should NOT
   grow a geometry knob (delete the question, keep the 8 m default).

**Adopt/reject rule:** change `d_behind_m` or add an approach-bearing
constraint ONLY if a cell beats the current default on acquisition
probability by more than the paired-seed noise band AND does not worsen
median CPA; otherwise the default stands and the null is logged.

**A NULL means:** the stern-approach intuition is already fully priced into
the existing 8 m trail; remaining misses are cue-error-vs-decode-range,
which no dash-geometry change can buy back — the lever would be acquisition
range (placard/camera) instead.

Status: REGISTERED, not yet flown. Queued behind the DFC dry-run verdicts.
