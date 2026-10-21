# Pre-registered: Gazebo re-earn of the isim latency cliff

2026-10-22 (registered before the batch flew). Purpose: the NN compute
ruling (ADR-0125) leans on the isim finding that detector latency ≥120 ms
breaks the pursuit terminal. That number currently gates a purchase, so per
the project's own rule it must be re-earned in the deciding simulator.

## Method

scripts/cinema/cinema_run.py grew `--inject-det-latency-s` (DelayedDetector
wrapper in the DRIVER; flight/ untouched; 0 = wrapper never installed, the
validated path byte-identical). Mechanism matches isim's `latency_s`: the
detection arrives late and the driver stamps it at arrival, so guidance
treats a stale bearing as current.

Arms: injected latency {0, 60, 120 ms} × 6 cue-error variants (PAIRED — the
same cue/geometry triplet differs only in latency). Geometry: the validated
demo crossing (lead 22, cross 4, 12 m/s, 30 s window), frames off. One sim
at a time, serial, idle machine.

## Pre-registered predictions + criteria

- From isim (runs/nn_requirements_20261021): the 120 ms arm's median CPA
  degrades ≥2× vs the 0 ms arm (isim said 0.16→4.0 m at 12 m/s); the 60 ms
  arm is within noise of 0 ms (run-to-run σ ≈ 0.2 m at this geometry).
- Paired comparison, n=6/arm: direction + consistency (how many of 6 pairs
  degrade) is the verdict; a 6/6 or 5/6 degradation at 120 ms CONFIRMS the
  cliff. Margins quoted as ranges, not points (n small).
- NULL: if 120 ms ≈ 0 ms (≤2/6 pairs degrade beyond noise), the isim
  latency surface does NOT transfer and the Hailo case must rest only on
  the measured Pi CPU times vs the 60 ms isim bar — say so on the dashboard
  and re-examine isim's measurement-delivery model before quoting it again.

## Honesty

Latency is INJECTED in the scoring-honest direction (pipeline made worse,
never better). The 0 ms arm is the unmodified validated path.

## RESULT (2026-10-22, run as registered; analyze.py over logs/gz_latency/)

| arm | med CPA (m) | pairs degraded >0.2 m | verdict |
|---|---|---|---|
| 0 ms (unmodified path) | 0.238 | — | baseline healthy (0.12–0.33) |
| 60 ms | 0.552 (2.3×) | 4/6 | **PREDICTION FAILED** — isim said within-noise; Gazebo says 60 ms already costs ~2.3× and pushes median outside the 0.35 m ram radius |
| 120 ms | 1.309 (5.5×) | 6/6 | **CLIFF CONFIRMED** per the registered rule (≥5/6 and ≥2×) |

Reading: the latency cliff TRANSFERS and is SHARPER in Gazebo than isim.
The detector spec's latency line tightens: ≤120 ms is broken, 60 ms is
degraded-but-flyable (sub-metre, outside ram radius), full performance
needs ≲30 ms glass-to-guidance. Consequences: CPU-only (131–154 ms
measured, ~70–90 ms NCNN-optimistic) is ruled out at ANY input size for
the terminal; the Hailo path must still budget its CPU-side pre/post +
tracker tightly. isim's seeker latency surface under-penalizes 60 ms —
re-fit before isim is quoted on latency margins again (contradiction-class:
model optimistic at mid-latency).

## FOLLOW-UP ARM (pre-registered 2026-10-22, before flying): COMPENSATED 60 ms

Root-cause reading of the RESULT: the flight KF compensates a FIXED assumed
measurement age (meas_latency_s = 45 ms), so the injected arms measured
UNCOMPENSATED EXCESS latency. A real NN pipeline's latency is measurable
and ~stable, so the fair question is: does TELLING the KF the true age
recover the loss? New flag --pursuit-meas-latency-s (deploy-relevant
regardless). Arm LC60: the same 6 cue variants, +60 ms injected, KF assumed
age 105 ms (45+60). PREDICTION: LC60 recovers most of the L060 loss (median
back within ~1.5x of the 0 ms baseline); the residual = geometry staleness
of the command itself, which no stamp can remove. If LC60 does NOT recover,
latency is intrinsically harmful and the ~30 ms budget stands. Either way
the 120 ms cliff conclusion is unaffected (jitter + command staleness).

## LC60 RESULT + LC120 pre-registration (2026-10-22)

LC60 (compensated 60 ms): 0.142/0.231/0.242/0.326/0.451/0.497, median
0.284 m vs baseline 0.238 — RECOVERED (1.19x, inside the predicted 1.5x;
uncompensated L060 was 2.3x). So the 60 ms loss was the KF's fixed 45 ms
age assumption, not the latency itself. Caveat: the injection wrapper
releases one detection per driver poll, so the effective excess under load
is not exactly 15 ms — the arm pair is read as compensated-vs-not, not as
a per-ms calibration.

LC120 (flying next, registered now): same 6 cues, +120 ms injected, KF age
= 165 ms. PREDICTION (uncertain, stated before flying): PARTIAL recovery —
stamping fixes the filter but the vehicle still acts on 120 ms-old
geometry at up to ~16 m/s closing (≈2 m of unseen relative motion), so
median lands between the compensated-60 (0.28) and uncompensated-120 (1.31)
arms; closer to which end decides whether the ~120 ms ceiling is intrinsic.

## LC120 RESULT — FULL recovery; the latency axis is really an AGE-KNOWLEDGE axis

LC120 (compensated 120 ms): 0.133/0.200/0.207/0.293/0.483/0.490, median
0.250 m vs baseline 0.238 (1.05x) — FULL recovery; the registered "partial"
prediction was WRONG (the command-staleness residual is not measurable at
these closing speeds and n). Three-arm truth:

| arm | 60 ms | 120 ms |
|---|---|---|
| KF age assumption WRONG (fixed 45 ms) | 2.3x | 5.5x |
| KF age assumption CORRECT | 1.19x | 1.05x |

FINAL spec reading (supersedes both earlier latency lines): detector
latency up to at least 120 ms costs ~nothing IF THE KF KNOWS IT. The
binding detector requirements are therefore: (1) latency KNOWN — stable
and calibrated into meas_latency_s (now CLI-settable), or per-detection
stamps (queued flight-code improvement); (2) JITTER small vs the assumed
constant (the injected arms were near-constant delay — jitter is untested
and is the thing a fixed constant cannot absorb); (3) detection RATE
>=10-15 Hz; (4) terminal range >=8-12 m. CPU-only STILL fails — on RATE
(154 ms => 6.5 Hz at 640) and on load-dependent jitter — so the Hailo
ruling stands, but on the rate/jitter/headroom argument, not the retired
latency-cliff argument. isim was never wrong: it models the age-known case.

## JITTER ARM (pre-registered 2026-10-22, before flying): LJ90

The one untested word in the final spec. Arm LJ90: injected latency 90 ms
+/-60 ms uniform per detection (range 30-150 ms, seeded per cue), KF age
assumption = the TRUE MEAN (45+90 = 135 ms). Same 6 cue variants.
PREDICTION: degradation BETWEEN the compensated and uncompensated arms --
each measurement is mis-aged by up to +/-60 ms even though the mean is
right; at 16 m/s closing that is +/-1 m of equivalent position error on a
subset of updates. If LJ90 lands near baseline (<=1.5x), a fixed calibrated
constant suffices for even CPU-class jitter and the stamped-age improvement
is a nicety; if it degrades >=2x, stamped-age is REQUIRED before any
high-jitter pipeline flies, and the Hailo's low jitter becomes a named
purchase argument.
