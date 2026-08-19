# M5 Final Monte-Carlo Batch — executable plan (pending scope ratification)

> Design-only plan for the M5 final batch (ADR-0033 step 0d/3). Every flight count
> is `--dry-run`-confirmed against the committed `scripts/mc_batch.sh`. The adopted-
> profile config is mined from the ADR-0031 run-logs, not guessed. Scope decision is
> the builder's (it's the overnight GPU-hammering run); the chosen option becomes an
> ADR once flown. Produced by an Opus design agent, 2026-07-07.

## 1. What this batch adds over ADR-0029 (n=48)

ADR-0029 already flew {pursuit, pronav} × {6, 9, 12 m/s} × both directions × n=8 and
gave the regime map (laws TIE at FPV speed — kinematic lock, ADR-0023). It is **not**
reusable as the M5 headline for two structural reasons:

1. **Wrong geometry** — it was the hover-ish / short-standoff geometry. The adopted
   deployment profile (ADR-0028 addendum, ADR-0030) is the **running start**:
   `--y0-mag 29.3` + `--dash-speed 16 --dash-unclamp` + velocity-emission cue +
   `--early-handoff`.
2. **Wrong cue constants** — ADR-0030/0031's realistic-cue numbers flew under the old
   ~180×-too-steep σ_R curve. The P-8 corrected constant (c=4.45e-05) is now the
   default, so `--sigma-range` means something different now. `check_s2.sh` cannot
   re-earn these (it flies flat `--sigma 0.5`; gate is regression-only, ADR-0033
   addendum) — **the corrected-constants re-earn happens only in this batch.**

So the batch is simultaneously (a) a **re-baseline** of the regime map on the adopted
profile + corrected constants, and (b) the vehicle for three new-science items:

| New science | Question |
|---|---|
| Maneuvering paths (weave + jink) | Does the law-washout + kinematic-lock story survive under maneuver? |
| `oblique_close` sign channel | Any systematic east/world_x asymmetry (frame/guidance sign bug)? First-ever nonzero target world-X velocity (vx≈−4.24 m/s). |
| Adopted profile + corrected cue | Re-earn miss/Pk under running-start + realistic corrected degraded cue |

**Representative-speed decision:** full 6/9/12 sweep ONLY for the straight-line
re-baseline; maneuver arms at **9 m/s only** (the canonical FPV point every prior
running-start study is anchored to). Weave/jink at 6/12 would re-confirm the same
kinematic mechanism at low information-per-boot (6 m/s bracketed by the straight arm;
12 m/s maneuvering is mostly dash-aborts = perception-availability, already the
ADR-0031 story).

**Pairing win (dry-run-verified):** running `line@9`, `weave@9`, `jink@9` as separate
single-speed arms with the same `--master-seed 42` draws **byte-identical cue_seeds per
run_idx** (only `path_seed` differs), so line-vs-weave-vs-jink at 9 m/s is **per-seed
paired** at zero extra boots. Those cue_seeds also match ADR-0031's baseline arm → an
old-σ_R-vs-new-σ_R A/B for free.

## 2. Shared adopted-profile config (every arm)

- **Env (realistic degraded cue, corrected constants):**
  `S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"`
  (EXPECTED tier; the WORST 0.20 s latency stays opt-in, not used here.)
- **`--extra-args "--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp"`**
- **Geometry:** `--y0-mag 29.3 --master-seed 42 --directions both`
- Each arm is its own `mc_batch.sh` call (`--path`/`--geometry` are single-valued) and
  boots a **fresh sim per flight** (boots == flights).

## 3. Arm table (RECOMMENDED = MIDDLE, 96 boots)

Prefix each with the `S2_CUE_MOCK_EXTRA=…` env; cwd `~/interceptor-sim`.

| # | Arm | Command (key flags) | Boots | Question |
|---|---|---|---|---|
| A1 | line@6 | `--laws pursuit,pronav --speeds 6 --directions both --path line --geometry standard --n 8 --y0-mag 29.3 --x0 6.5 --master-seed 42 --extra-args "…" --out logs/mc_final_line6.csv` | 16 | Re-baseline low end |
| A2 | line@9 | as A1, `--speeds 9 --out …line9.csv` | 16 | FPV re-baseline; paired control for weave/jink |
| A3 | line@12 | as A1, `--speeds 12 --out …line12.csv` | 16 | Top-of-band re-baseline |
| B | weave@9 | `--speeds 9 --path weave --geometry standard --out …weave9.csv` | 16 | Washout under gentle S-curve (paired to A2) |
| C | jink@9 | `--speeds 9 --path jink --geometry standard --out …jink9.csv` | 16 | …under sharp seed-scheduled jinks (paired to A2/B) |
| D | oblique@6 | `--laws pronav --speeds 6 --directions both --path line --geometry oblique_close --west-angle-deg 45 --x0 90 --y0-mag 29.3 --n 16 --out …oblique6.csv` | 16 | East/world_x sign channel: L2R vs R2L symmetry |

**Oblique runway check:** default `--x0 6.5` fails fast ("0.9 s of closing runway, need
≥20 s"). Minimum passing `--x0` = **86 at 6 m/s/45°** (128 at 9 m/s) → use `--x0 90` at
6 m/s (margin). 6 m/s chosen so `--x0` stays tractable and CPA stays clean (a sign error
surfaces as a miss, not a blind flyby).

## 4. Boot budget, wall-clock, wedge-safety

**Per-flight duration (measured):** ADR-0031 running-start analog logs = 67–76 s/flight
(mean ~71.5 s); ADR-0029 hover = 64 s. Planning figure **75 s** standard / **95 s**
oblique (longer standoff). No flight nears the 360 s timeout.

| Option | Boots | Wall clock |
|---|---|---|
| LEAN (A1,A2,A3 + jink@9 + oblique n=8) | 72 | ~1.5–2 h |
| **MIDDLE (recommended)** | **96** | **~2.5 h** |
| FULL (MIDDLE + weave/jink @6&12 + oblique pursuit) | 144 | ~3.5–4 h (2 nights) |

**Wedge-safe execution (GPU wedged once at ~10 RAPID boots, ADR-0032; fix = host-side
`wsl --shutdown`):**
1. **Sequential arms, never concurrent** (two batches kill each other's sims). Launch
   the next arm only after grepping the prior run-log for `Batch complete` — never poll
   via `pgrep -f mc_batch.sh` (self-match, never exits — mc-batch gotcha).
2. **No invocation exceeds the proven-safe 48** — every arm is 16 boots.
3. **Between-arm gate:** ≥120 s cooldown, then `dmesg | grep -iE 'dxgk'` for genuinely
   new clusters + confirm no orphaned sim; on the next boot confirm the camera topic
   publishes. Any new dxgk cluster or silent topic → STOP and report (recover via host
   `wsl --shutdown`).
4. **Checkpointing = free** (each arm → own `--out` CSV; cue_seeds deterministic from
   `--master-seed 42`). A wedge on arm N loses only arm N → **resume-at-failed-arm,
   never restart**.
5. **Idle load only**, overnight, no other sims.

**Session split:** MIDDLE's 96 boots is 2× the proven-safe 48 → one idle-overnight
session with the between-arm gate as tripwire, OR conservative 2-night split (night 1 =
A1/A2/A3 = 48; night 2 = B/C/D = 48).

**Oblique shakedown FIRST:** `oblique_close` has never been flown at scale (only its vx
value was dev-verified). At the ~90 m standoff under a degraded cue it risks the
ADR-0030/0031 "dash failed to reach handoff in 20 s" mode. **Fly 2–3 shakedown flights
before committing arm D**; if it mostly dash-aborts, the L2R/R2L symmetry of the aborts
is still a valid sign check, or drop to `--west-angle-deg 30`.

## 5. Recommendation

**MIDDLE (96 boots, ~2.5 h).** Delivers all three new-science items at defensible
strength — both maneuver classes (gentle weave + sharp jink) per-seed paired to line@9,
and oblique at 8 flights/direction for a real asymmetry read — structured as six 16-boot
arms, each ≤ the proven-safe ADR-0029 size and each checkpointed.

**Honest caveat for the ADR:** n=8/cell (16 with both directions) keeps Pk CIs wide and
pursuit-vs-pronav deltas within the ~1 m run-to-run noise — the batch tightens the
*curve* and settles the *qualitative* maneuver/sign questions, not tight per-cell
significance.

**Deliverables:** per-speed Pk-vs-radius (ram 0.5 / net 1.5, never pooled — ADR-0025),
pursuit-vs-pronav-by-speed regime map, per-path law comparison at 9 m/s (paired), the
oblique L2R/R2L symmetry table, and the corrected-constants restatement of ADR-0030/0031.
