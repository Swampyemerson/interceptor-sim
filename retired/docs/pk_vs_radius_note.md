# Pk vs. Lethal Radius — Sensitivity Note (design review G15)

*The design review (G15) flagged that every "Pk" in this project is a
closest-approach distance compared against a **chosen** lethal radius — there
is no collision volume or fuze/net model in the sim — and asked for a $0
sensitivity analysis so a reader can see exactly how much the headline metric
moves with the assumed radius. This note is that analysis. Computed
2026-07-09 directly from the M5 final-batch CSV; the recomputation command is
at the bottom.*

**Data source:** `logs/mc_final_all.csv` — the M5 final Monte-Carlo batch
(ADR-0036): n=96 flights, pursuit + pro-nav × 6/9/12 m/s × line/weave/jink/
oblique paths on the adopted running-start deployment profile, clean-AprilTag
seeker, realistic degraded cue. Per this repo's convention most `logs/` CSVs are gitignored (regenerable), but
the key evidence CSV for this note (`logs/mc_final_all.csv`, the ADR-0036 n=96
batch) is now **committed** (`2665bbb`) — so the numbers below trace to a
committed artifact, are regenerable via the ADR-0036 `mc_batch.sh` configuration,
and cross-check exactly against the committed plot
`docs/images/m5_pk_vs_radius_by_arm.png` and the pooled curve already
published in the README (53% / 79% / 92% / 100% at 1.0/1.5/2.0/2.5 m).

**Physical anchors (ADR-0025, `docs/kill_mechanism.md`):** the two radii with
a physical story are **kinetic ram ≈ 0.5 m** and **net ≈ 1.5 m**. Radii are
never reverse-engineered to clear a threshold; the whole curve is the metric.

---

## Pooled curve (context only — ADR-0025 says never pool alone)

| R_lethal | 0.5 m | 1.0 m | 1.5 m | 2.0 m | 2.5 m | 3.0 m |
|---|---|---|---|---|---|---|
| Pk (n=96) | 9/96 (**9.4%**) | 51/96 (**53.1%**) | 76/96 (**79.2%**) | 88/96 (**91.7%**) | 96/96 (**100%**) | 96/96 (100%) |

Batch miss statistics: mean 1.084 m, median 0.929 m, min 0.271 m, max 2.459 m
(93/96 clean).

## Per-arm curves (the honest view — each cell is n=8 or n=16)

| Arm (law · speed · path) | n | Pk@0.5 | Pk@1.0 | Pk@1.5 | Pk@2.0 | Pk@2.5 | median | max |
|---|---|---|---|---|---|---|---|---|
| pro-nav · 6 m/s · oblique | 16 | 8/16 | 16/16 | 16/16 | 16/16 | 16/16 | 0.49 m | 0.81 m |
| pursuit · 6 m/s · line | 8 | 0/8 | 8/8 | 8/8 | 8/8 | 8/8 | 0.77 m | 0.93 m |
| pro-nav · 6 m/s · line | 8 | 0/8 | 6/8 | 7/8 | 8/8 | 8/8 | 0.80 m | 1.56 m |
| pursuit · 9 m/s · line | 8 | 0/8 | 4/8 | 8/8 | 8/8 | 8/8 | 0.99 m | 1.49 m |
| pro-nav · 9 m/s · line | 8 | 0/8 | 3/8 | 7/8 | 8/8 | 8/8 | 1.13 m | 1.56 m |
| pursuit · 9 m/s · jink | 8 | 0/8 | 3/8 | 8/8 | 8/8 | 8/8 | 1.06 m | 1.45 m |
| pro-nav · 9 m/s · jink | 8 | 0/8 | 3/8 | 7/8 | 8/8 | 8/8 | 1.05 m | 1.55 m |
| pursuit · 9 m/s · weave | 8 | 0/8 | 4/8 | 4/8 | 4/8 | 8/8 | 1.48 m | 2.25 m |
| pro-nav · 9 m/s · weave | 8 | 1/8 | 4/8 | 4/8 | 5/8 | 8/8 | 1.41 m | 2.46 m |
| pursuit · 12 m/s · line | 8 | 0/8 | 0/8 | 4/8 | 7/8 | 8/8 | 1.62 m | 2.18 m |
| pro-nav · 12 m/s · line | 8 | 0/8 | 0/8 | 3/8 | 8/8 | 8/8 | 1.56 m | 1.97 m |

(The README's per-speed numbers pool the oblique arm into 6 m/s: e.g.
"6 m/s Pk@1.5 = 96%" is 23/24 across the three 6 m/s pro-nav rows above.)

## How to read the sensitivity

- **The claim is radius-dominated below ~1.5 m.** Pooled Pk swings 9% → 53% →
  79% between R=0.5 and R=1.5 m — roughly **+45 percentage points per
  half-meter of assumed radius** on the steep segment. Any headline quoted at
  a single radius in this band is a statement about the *assumption* at least
  as much as about the *system*.
- **At the ram radius (0.5 m) this system does not kill fast targets.**
  Only the 6 m/s oblique-approach arm scores (8/16); every 9–12 m/s arm is
  0/8 (one weave flight excepted). That is the kinematic floor (ADR-0023) —
  ~0.9–1.0 m at 6 m/s even with perfect sensing — showing up exactly where it
  should. The ram number is honestly conceded, not hidden.
- **At the net radius (1.5 m) the design mostly works, and the binding cells
  are visible:** straight-line and jink arms sit at 7–8/8, but weave@9 holds
  4/8 and 12 m/s line 3–4/8. The 12 m/s arms then saturate at 2.0 m (7–8/8)
  while weave stays the worst cell until 2.5 m — the weave's residual is the
  bearing-quality mechanism ADR-0056 later isolated.
- **Saturation by 2.5 m.** Every arm is 8/8 at 2.5 m (batch max miss
  2.459 m), so quoting Pk@2.5 alone carries almost no information about the
  system — which is exactly why the project's convention is the whole curve.

## The maneuvering deployment config (secondary, with the three-level caveat)

Same sweep over the ADR-0058 headline validation arm
(`logs/mc_t21_trackgate_weave12_r2.csv`, markerless detect-then-track +
cue-gated handoff, 12 m/s weave, n=16). **Caveat:** this CSV's `miss_m` is
the **pooled whole-flight** minimum (level i of the three-level Pk
convention) — it includes closest approaches banked during the legal
cue-guided dash. The camera-terminal numbers (level ii: 14/14 @2.5, median
2.03 m) come from the per-flight logs via `scripts/analyze_track_ab.py`, not
from this file.

| R_lethal | 0.5 m | 1.0 m | 1.5 m | 2.0 m | 2.5 m |
|---|---|---|---|---|---|
| Pk pooled (n=16) | 0/16 | 0/16 | 5/16 | 7/16 | 16/16 |

The markerless maneuvering regime sits at its ~2 m perception ceiling
(AprilTag control ceiling: 8/8 at median 1.64 m, ADR-0056), so its Pk is far
more radius-sensitive than the M5 tag numbers: the whole outcome flips
between R=1.5 and R=2.5 m. A net-class (1.5 m) kill claim is **not**
supported in this regime; a 2.5 m proximity criterion is.

## Statistics caveats

- Per-arm cells are n=8/n=16: a 95% Clopper-Pearson interval on 8/8 has a
  lower bound of ~63%, and on 16/16 of ~79.4% — "no failures observed," not
  "Pk ≥ 95%" (design review G14).
- The pooled 96/96 at 2.5 m has a 95% lower bound of ~96.2%, but pooling
  mixes regimes and is context-only per ADR-0025.

## Recompute it

```bash
~/interceptor-sim/.venv/bin/python - <<'EOF'
import csv, collections, statistics
rows = list(csv.DictReader(open('logs/mc_final_all.csv')))
radii = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
miss = [float(r['miss_m']) for r in rows]
print('POOLED n=%d:' % len(rows),
      {R: sum(m <= R for m in miss) for R in radii})
arms = collections.defaultdict(list)
for r in rows:
    key = (r['law'], r['config'].split('_')[-1], r['path'], r['geometry'])
    arms[key].append(float(r['miss_m']))
for k in sorted(arms):
    m = arms[k]
    print(k, len(m), [f"{sum(x <= R for x in m)}/{len(m)}" for R in radii],
          f"med {statistics.median(m):.2f} max {max(m):.2f}")
EOF
```

Swap the filename for `logs/mc_t21_trackgate_weave12_r2.csv` to reproduce the
secondary table. If `logs/mc_final_all.csv` is absent (fresh clone — logs are
gitignored), regenerate the batch per ADR-0036 / `docs/m5_final_batch_plan.md`
before running this, or read the committed plot
`docs/images/m5_pk_vs_radius_by_arm.png`, which encodes the same per-arm
curves.
