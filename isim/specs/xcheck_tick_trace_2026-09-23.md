# Tick-trace diagnosis: Gazebo pursuit cross-check transfer gap (2026-09-23)

**Question.** The pre-registered Gazebo cross-check (n=8, camera-in-the-loop,
`docs/xcheck_gazebo_pursuit_prereg.md`) failed its bar: camera-driven chase
confirmed (46-67 detections consumed/flight, 0 aborts, clean SAFE endings) but
median CPA **1.510 m** (0.549-2.334) vs the matched-optics isim prediction
**0.122 m** (n=50, `logs/xcheck_isim_prediction_20260923.csv`). This is the
registered next step: attribute the 12x gap to the five registered suspects,
with numbers. **Not tuning** — no config was changed and no fix implemented.

**Instrument.** `scripts/forensics/xcheck_tick_trace.py` (this spec's registered
instrument, built on `parity_trace_pursuit.py`'s method: per-tick truth vs what
guidance was handed, then per-suspect effect sizes). Data:
`logs/xcheck_gz_20260923_fixed/f{1..8}{,_rf,_gtpose}.csv` + run logs. All truth
columns are the scoring-only gt stream; nothing here ever fed guidance.
Timing convention: the driver loop polls/logs, then detects, then steps the SM,
so a detection whose consumed-count increment is visible at logged row *k* was
decoded and consumed during tick *k-1* (consume time = `sim_t[k-1]`, a lower
bound tight to one 52 ms tick).

## Verdict in one paragraph

The gap is **estimator bias x vehicle-response mismatch**, locked in by a hot
final approach. The KF's range estimate runs **0.4-1.1 m short** through the
endgame because the **box-width->range channel under-ranges ~15-25%
systematically** (the AABB of the rotated/perspective tag is wider than the
pinhole side isim synthesizes); per-flight KF range bias predicts CPA at
**corr = -0.84**. Independently, the real Gazebo velocity loop is **~2x slower
plus ~0.2 s more delayed** than the fitted isim vehicle model in this braking/
lateral-nulling regime, so the vehicle arrives at 6 m out still closing
**6.6 m/s achieved vs 4.9 commanded** (isim closed at 1.28 m/s at CPA) with a
median **1.38 m cross-track offset** it can no longer null — the misses are
cross-track (med 1.41 m cross vs 0.66 along, 0.39 vertical). Latency (S1) and
tick stretching (S3-as-registered) are **exonerated**; the re-approach gap (S5)
is exonerated as a *cause* (isim's prediction runs the same no-re-approach SM)
but it locks every first-pass miss in.

## Per-suspect findings

### S1 — fixed 45 ms `meas_latency_s` vs true render-to-consume delay: EXONERATED

| pooled (8 flights, 415 detections) | value |
|---|---|
| true age (capture stamp -> consume tick), median / p90 | **0.052 s / 0.064 s** |
| assumed | 0.045 s |
| staleness range bias = (age-0.045) x \|v_rel\|, median / p90 | **0.029 m / 0.108 m** |

The assumption is 7 ms optimistic; the induced bias is ~2% of the 1.39 m gap.
(Mechanically: the 30 fps camera frame waits at most ~33 ms + in-tick decode;
the newest-frame poll keeps it fresh.)

### S2 — tag size / measurement noise: the registered wording is minor, but the CHANNEL is defective — CONFIRMED (reframed)

The registered concern (noise floor shaped on a 0.30 m tag vs the 0.5 m world
tag) is measurable and mild: pose-range spread **0.109 m** (1 sigma) vs the
noise model's along-LOS sigma **0.067 m** at the median 6.5 m detection range —
under-modeled ~1.6x, not 12x.

The real finding is a **systematic bias in the channel guidance actually eats**.
Range residual vs true range at capture (pooled, by true range):

| true range | n | PnP pose range | box-depth (`fx*span/w`) | after slant corr. (approx) |
|---|---|---|---|---|
| 0-3 m | 60 | -0.095 m (spread 0.087) | **-1.000 m** | -0.609 m |
| 3-6 m | 131 | -0.070 m (0.090) | **-0.825 m** | -0.660 m |
| 6-10 m | 136 | +0.009 m (0.113) | **-2.041 m** | -1.863 m |
| 10-30 m | 88 | +0.061 m (0.093) | **-2.637 m** | -2.570 m |

The AprilTag PnP pose range is essentially perfect (bias -0.03 m pooled; the
close-range -0.07..-0.10 m is consistent with the ~0.11 m camera lever arm the
center-to-center gt ruler does not carry). The **box-width depth is 15-25%
short**: the driver (`gazebo_pursuit_crosscheck.tag_to_detection`) and the
flight code (`measurement_from_box`) derive range from the **axis-aligned
bounding box** of the corners, and a tag rotated in the image / seen in
perspective has an AABB wider than its true side (up to sqrt(2)); the
terminal's cos(off-axis) slant fix (yaw-approximated here) recovers only part.
isim's seeker synthesizes an *ideal* pinhole `side_px`, so **isim structurally
cannot see this defect** — a genuine real-optics vs sim measurement-model
divergence.

Downstream (the KF eats it): `r_hat - r_true` at inbound true-range crossings,
median across flights: **-1.07 m at 6 m, -0.55 m at 4 m, -0.39 m at 3 m**;
last-2 s median -2.17 m. Per-flight KF range bias vs CPA: **corr = -0.84
(n=8)** — the two flights with the smallest bias (f2 -0.38, f5 -0.37) are the
two best CPAs (1.01, 0.55); the largest (f4 -1.03, f6 -0.86) are the two worst
(2.33, 2.33). Concrete mechanisms observed:

- **f1**: a 1.5 s decode gap at r_hat<1 m latched the coast hold 5.1 s before
  CPA at true range 2.32 m with a true cross-track offset of **0.84 m — CPA
  came out 0.839 m**, exactly the frozen offset. Steering stopped; the miss was
  the bias.
- 6/8 flights entered coast at r_hat ~0.95 while true range was 1.9-4.3 m
  (mostly at/after CPA, so it shaped the *exit*, not the miss itself).
- Every `pursuit_miss` abort fired on `r_hat <= 3 m` while true range was
  **4.0-8.3 m** — the abort logic is calibrated on a biased ruler.

### S3 — detect-in-loop tick stretching: STRETCHING EXONERATED, RATE CONFIRMED (reframed)

No stretching: driver tick `sim_t` deltas are **0.052 s median / 0.056 p90 /
0.100 max**, identical in STANDBY and ENGAGE; ticks that ran the decoder are
only ~4 ms longer (0.056 vs 0.052). The decode does not distort the loop.

The rate is another matter: the driver consumes at most one detection per two
ticks of its 20 Hz loop (`real_flight.run_mavsdk_mission`'s `n_tick % 2 == 0`
detect gate), i.e. a ~9.6 Hz detect ceiling against a 30 fps camera, and
in-ENGAGE consumption averaged **5.2 det/s vs the isim prediction's 25.5
det/s** (164 decodes / 6.4 s ENGAGE) — **5x fewer KF updates**, with occasional
0.4-3.2 s gaps. This amplifies S2's bias (less averaging, longer coasts on a
biased state) but cannot alone explain the gap (f2/f5 had the same 5 Hz and
still reached ~0.55-1.0 m).

### S4 — real EKF/controller dynamics isim does not carry: CONFIRMED (vehicle response; own-state EKF exonerated)

Own-state EKF is clean: EKF altitude minus gz truth median **+0.03 m** across
flights (worst +0.11 m); no own-state contribution worth ranking.

The **velocity-setpoint tracking** is not. First-order (delay, tau) fits of
achieved (gz-true) velocity vs the commanded setpoints over ENGAGE, against the
same fit run on isim's fitted vehicle model (`isim/fits/vehicle_gazebo_x500.
json`) replaying the *identical* command stream:

| axis | Gazebo real (delay / tau) | isim fitted model (delay / tau) |
|---|---|---|
| north | **0.24 s / 0.51 s** | 0.03 s / 0.30 s |
| east | **0.18 s / 0.60 s** | 0.02 s / 0.36 s |
| down | 0.08 s / 0.30 s | 0.29 s / 0.99 s |

Horizontally the real vehicle is **~2x slower with ~0.2 s more dead time** than
the model isim predicted with; vertically the model is the pessimist (fit was
shaped on dash segments — the regime rule: a fit validated at one operating
point is not validated at another). Command-tracking RMS over ENGAGE: real
**3.76 m/s** vs the model's own 2.80 m/s. Re-anchoring the model to the true
state every 1 s: it diverges **1.34 m in position and 2.06 m/s in velocity per
second** of terminal flight — the same order as the whole miss, every second.

Where it bites: at the last inbound 6 m crossing (~0.9 s to go) the commanded
closing rate was already hot (median **4.9 m/s**; isim's CPA closing: 1.28 m/s)
and the achieved closing was hotter still (**6.6 m/s** — the plant had not yet
realized the braking), while the true cross-track offset was median **1.38 m**.
With ~0.8 s of effective lag, that offset is physically un-nullable in the time
remaining; the CPA decomposition shows exactly that residual: median miss
**1.41 m cross-v_rel** vs 0.66 along and 0.39 vertical.

### S5 — no re-approach after a missed first pass: EXONERATED as a gap cause; confirmed as the lock-in

The isim prediction runs the **same** `RealFlightSM` with no re-approach and
still scores 0.122 m, so this cannot explain isim-vs-Gazebo. In Gazebo all 8
flights were single-pass: after CPA no flight ever re-closed below its
first-pass CPA (post-CPA minima 2.5-20.8 m). f2/f5 lost the tag after passing,
the fallback adopted a corrupted KF velocity (r_hat error **+7.1 / +11.0 m**
in the last 2 s) and flew away to 22-28 m before `target_lost` -> BREAKOFF;
the other six ended via `pursuit_miss` at true range 4.0-8.3 m (biased r_hat,
see S2). Re-approach remains the queued builder design question; it would
*recover* misses, not prevent them.

## Ranking (contribution to the 1.51 m vs 0.122 m median gap)

1. **S4 vehicle response (~0.4-0.9 m, the universal floor).** Present in every
   flight: the best-estimated, never-coasting flights (f2, f5) still missed by
   1.01/0.55 m vs 0.122 predicted, with the hot-arrival + un-nulled cross-track
   signature. isim's fitted plant is 2x too agile horizontally in this regime.
2. **S2 box->range bias (~0.5-1.3 m, the spread and the tail).** KF range runs
   0.4-1.1 m short; per-flight bias predicts CPA (corr -0.84); it froze f1's
   steering at the exact miss distance, mis-times coast, and fires the miss
   abort at true 4-8 m. The noise-floor part of the registered suspect is
   real but minor (1.6x); the bias is the finding.
3. **S3 measurement rate (amplifier, not a driver).** 5.2 det/s consumed vs
   isim's 25.5; no tick stretching. Multiplies 1 and 2; cannot produce the gap
   alone.
4. **S1 latency: exonerated** (0.029 m median effect).
5. **S5 re-approach: exonerated as cause** (same code both sides); locks in
   whatever 1-3 produce, and its abort thresholds inherit S2's biased ruler.

## Recommended next action (for the confirmed suspects only — no fixes implemented here)

- **S2 (flight code, surgical):** stop deriving range from the AABB width.
  The same detector already computes the PnP pose range, measured here at
  **-0.03 m bias, 0.109 m spread** over 415 live detections — use it (or a
  rotation-robust side estimate, e.g. min corner-pair edge length /
  homography scale) in `measurement_from_box`'s depth channel and in the
  driver's `tag_to_detection`. Note the same AABB channel ships in the
  validated `SeekerGuidance`/`tag_terminal` paths (the ADR-0105 slant-fix
  ruling already flagged that surface) — same decision applies.
- **S4 (instrument honesty):** re-fit `isim/fits/vehicle_gazebo_x500.json` on
  the terminal regime — these eight flights' ENGAGE segments are exactly the
  braking/lateral data the dash-segment fit lacked — and re-run the isim
  prediction with the re-fitted plant before any re-flight; if the re-fitted
  isim reproduces ~1-1.5 m, the transfer gap is closed as "sim was flattering
  the plant", and any *guidance* change (earlier braking, gentler schedule)
  becomes an ordinary pre-registered isim experiment instead of a Gazebo hunt.
- Only after both: re-run this cross-check (same prereg bar) to see what
  remains of the gap.

Every number above is reproducible from the logged CSVs via
`scripts/forensics/xcheck_tick_trace.py` (defaults point at
`logs/xcheck_gz_20260923_fixed/`).
