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

Status: FLOWN 2026-10-21 — **NULL by the registered rule** (no cell beats
the default beyond paired-seed noise; the astern dash aim is actively
harmful; cue error dominates). Defaults stand, the stage-3 router is not
built. Verdicts: `runs/stern_approach_20261021/findings.md`.

## Registered follow-up (2026-10-21 overnight, before flying): d_behind {4,8} at n=32

The NULL's footnote, promoted to its own registered question: at ZERO
deliberate cue error (Scatter noise only), does `d_behind=4` really halve
terminal median miss vs the default 8 (sweep saw 0.24 vs 0.51 at n=8),
with acquisition unchanged? Cells: d_behind ∈ {4, 8} × dash_aim_behind=0 ×
aim_error=0, n=32 paired seeds (0..31), same canon config.

PREDICTION: the n=8 gap shrinks (regression to noise) but d_behind=4 stays
directionally better on median miss; engagement equal within noise.
ADOPT BAR: d_behind=4 becomes the new PursuitConfig default ONLY if its
median miss beats the default by >0.1 m (the known run-to-run noise floor)
AND pct_engage is not lower AND p90 miss is not worse. A NULL means the
n=8 gap was seed luck; the 8 m default stands and the footnote closes.
