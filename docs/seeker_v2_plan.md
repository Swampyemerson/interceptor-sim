# Seeker v2 — hard negatives + range calibration (pre-registered plan)

*Written 2026-07-08, BEFORE the v2 model exists. ADR-0040 addendum sets the
direction: the two tuned-v1 regressions (seeds 772247, 33327) were terminal
dropouts of the real target caused by residual own-airframe false-positives
INSIDE the self-mask gate (near-boresight meas/gt range median 0.06, n=506
ticks), so hard NEGATIVE flight frames are the PRIMARY lever; range
calibration is SECONDARY. New diagnostic this session: on clean static
renders, IoU-gated true positives show span_eff = 0.91 m vs the assumed
1.0 m — only ~10% range bias — confirming the flight-time ~16× collapse is
detection POLLUTION, not box geometry.*

## Method (controlled: dataset is the only variable)

- **Capture pass** (`capture_pass.sh` + `capture_flight_frames.py`): 3 fresh-boot
  markerless deployment-profile flights, cue seeds **111/222/333 — deliberately
  DISJOINT from the master-seed-42 A/B plan** so v2 never trains on frames from
  its own evaluation trajectories. Passive recorder: gt-projected positives
  (training-time-only gt use, the established labeler boundary), out-of-FOV /
  behind-camera frames saved as EMPTY-label hard negatives (own prop arms in
  view — the frames v1's static ground renders never contained). In-FOV-but-
  tiny (<4 px) frames are DROPPED, never taught as background.
- **Dataset v2** (`merge_dataset_v2.py`): v1's 495 renders + capture, seeded
  re-split, negatives capped at 40%.
- **Training**: identical v1 recipe (yolo11n.pt local base, imgsz 640, epochs
  80, batch 16, seed 0, CPU `.venv-seeker-train`), only the dataset changes.
  Output `weights/drone_finetuned_v2.onnx` (v1 artifacts untouched).
- **Range calibration**: `calibrate_range.py` on v2 val positives →
  `drone_finetuned_v2.onnx.calib.json` sidecar (~0.91 m expected); the seeker
  auto-loads it (env MARKERLESS_SPAN_M > sidecar > default 1.0).

## Pre-registered gates (paired n=8, master-seed 42, vs the ADR-0040 v1 arm)

1. **Static probe (regression guard):** v2 must hold v1's 8/8 ranges (2–12 m).
   FAIL → do not fly the A/B; diagnose first.
2. **PRIMARY — clean-rate:** v1 = 6/8. SUCCESS needs v2 ≥ 7/8 AND mechanism
   evidence (n=8 binomial alone cannot carry it):
   (a) the pollution signature collapses — per-tick near-boresight detections
   with meas/gt range < 0.2 go from v1's dominant mode toward zero;
   (b) the two v1 dropout-abort seeds (772247, 33327) complete clean.
3. **No regression:** coverage ≥ ~0.25 (v1), median gap-to-tag ≤ +0.66 m (v1),
   the two flybys v1 fixed (256788, 776647) STAY fixed.
4. **Range (secondary):** post-calibration true-positive meas/gt median in
   [0.7, 1.3] (v1 flight: 0.06).
5. **Honesty:** `cue_reads_post_handoff=0` on all 8; `test_honesty_static.py`
   7/7; PLUS the full per-tick no-cheat audit NEXT.md owes at A/B close.
6. **NULL outcome (pre-declared):** if clean-rate stays ≤ 6/8 with the
   pollution signature gone, the residual aborts are NOT false-positive-driven
   → log honestly, next lever becomes detection-persistence (imgsz / fusion
   capstone), not more negatives.

Run-to-run noise ~1 m on miss; "not significant at this n" language applies to
any miss delta inside that. Batches at idle load, arms sequential.

---

# ADR-0043 pre-registration — terminal bearing-noise lever (added 2026-07-08, BEFORE the arms fly)

Mechanism measured in ADR-0042: v2's endgame dispersion (+1.1 m median gap-to-tag)
comes from PN following box-center bearing noise (tick-to-tick p90 2.8 deg vs the
tag's subpixel ~0.00 deg, 3.5-12 m band, n=139 pairs) with honest vc — NOT from
detection coverage (v2 sees the real target 78-89% of final-second ticks, MORE
than the tag's 5-7/~40).

**Arms (n=8 paired, master-seed 42, v2 weights + sidecar, same profile):**
- **B (gain rolloff):** `--terminal-gain-scale 0.35` — lambda-filter gain scaled
  in ENGAGE only. Sizing: noise ratio ~1.5-2x => Kalman-ish gain ratio
  1/ratio^2 ~= 0.25-0.44; 0.35 is the midpoint. Keeps corrections, weights them.
- **C (early freeze):** `--terminal-freeze-range 6.0` — ballistic through the
  noise band (mimics deliberately what the tag's sparse detections did
  accidentally). Forfeits real corrections below 6 m.

**Pre-registered outcomes:**
- PRIMARY: median paired gap-to-tag. SUCCESS = gap shrinks from +1.142 m toward
  or past v1's +0.658 m WITH >=6/8 seeds moving the same direction (the
  noise-floor-aware sign test), clean-rate >= 6/8, no new flyby (max miss <= 4 m).
- Mechanism check: endgame |a_cmd| medians drop toward tag-like (B) / exact 0
  inside 6 m (C); pollution stays 0.000; per-tick audit + cue_reads green.
- NULL: neither arm achieves the sign-consistent shrink -> log honestly; the
  lever moves to measurement-side smoothing or the fusion capstone.
- PRE-NOTED RISK: a line-path win for C does NOT license adopting C as default —
  early freeze forfeits real corrections against maneuvering targets; a weave
  sensitivity arm is REQUIRED before any default change.
