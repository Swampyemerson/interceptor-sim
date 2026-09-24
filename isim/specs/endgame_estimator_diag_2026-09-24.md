# Endgame-estimator diagnosis — registration (2026-09-24, after re-fly #3)

Registered BEFORE any analysis ran. Follow-up to ADR-0116: across three Gazebo
campaigns (1.510 → 0.967 → 0.928 m median) every registered fix landed as attributed,
and the last measured suspect is the ENDGAME ESTIMATOR — the KF's range error is
−0.065 m overall but **−2.58 m median over the last 2 s** (pose measurements clean at
capture), and the brake cap fed by that state could not brake (closing 2.84 = 2.84
m/s with the package provably on). This round is DIAGNOSIS ONLY: no guidance or
estimator tuning, no new Gazebo flights.

## Data

- Gazebo: `logs/xcheck_gz_20260924_brake/f{1..8}{,_rf}.csv` (brake arm) and
  `logs/xcheck_gz_20260923_refly/f{1..8}{,_rf}.csv` (no-brake) — 16 flights, per-tick
  `r_hat_m` (live KF), gz-truth own/target positions, det columns incl.
  `det_tagpose_range_m` and `det_t_capture_sim`.
- isim: matched-optics port-arm runs (ENGAGE fit, fx 540.3, tag 0.5 m, rear; brake
  and no-brake arms, ≥8 seeds each) with a per-tick r_hat recording shim (scratch
  wrapper around the built guidance; no isim/flight code edits).

## Questions (each answered with a number, not prose)

- **Q1 — where the last-2s error comes from (Gazebo).** Per tick, err = r_hat −
  r_true. Split: (a) residual AT each consumed update (post-update err) binned by
  true range; (b) err GROWTH during coast intervals vs time-since-last-consumed-det
  (slope, m/s); (c) the KF's implied relative-velocity error near CPA vs gz truth;
  (d) same stats, brake vs no-brake arm (does braking lengthen the critical coast?).
- **Q2 — the isim fidelity gap.** The same statistics from matched isim traces:
  last-2s err distribution, update residual by range, coast slope. Quantify
  Gazebo/isim ratios per statistic.
- **Q3 — mechanism ranking.** Score the candidates against Q1's numbers:
  (a) target-velocity estimate error × coast time (predicts linear err growth in Δt,
  slope ~1.5–4 m/s); (b) capture-latency/age mis-modelling as LOS rate rises near CPA
  (predicts update residuals jumping at short range while coast slope stays modest);
  (c) own-EKF horizontal velocity error (directly measurable from the rf CSV vs gz
  truth); (d) close-range pose bias (predicts update-instant bias growing below ~4 m).

## Registered predictions

- **P1:** coast growth (a) dominates — the last-2s collapse is mostly v_t-error ×
  coast, slope in the 1.5–4 m/s band; update residuals stay near the ±0.3 m class.
- **P2:** isim's matched last-2s statistic is ≤ 1/3 of Gazebo's, and the gap sits
  mainly in coast behaviour / measurement spread (the declared 1.6× PnP
  under-modelling), not in the update-instant bias.
- **Null branches:** if update residuals (not coast) carry the collapse, the story is
  measurement/latency at short range (mechanism (b)/(d)) and the fix class is
  different (noise/latency model, not velocity estimation). If own-EKF velocity error
  (c) is the driver, it is a plant/EKF question upstream of the terminal entirely.

## Deliverable

An attribution table (mechanism → share of the last-2s error, both sims), plus the
specific isim model deltas the gap demands (e.g., measured PnP sigma into the seeker
noise model; coast/latency handling) — EACH as its own registered follow-up, none
built in this round.
