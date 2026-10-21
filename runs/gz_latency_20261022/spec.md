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
