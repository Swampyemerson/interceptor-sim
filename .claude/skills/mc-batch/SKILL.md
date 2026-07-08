---
name: mc-batch
description: Run a Monte-Carlo batch arm (scripts/mc_batch.sh) safely and gate it — the launch → wait → parse → GPU-health → cooldown loop, with the self-kill/GPU-wedge footguns baked in. Use whenever running an mc_batch arm or a multi-arm sweep (M5 final batch, EKF/seeker A/B, bird decoy MC). Complements px4-gazebo (env) and sim-milestone (close-out).
---
# Running a Monte-Carlo batch arm, safely

`scripts/mc_batch.sh` boots a **completely fresh sim per flight** (boots == flights), so
a batch is many boot/kill cycles — the exact thing that can wedge the WSL2 GPU. This skill
is the loop that ran the M5 final batch (ADR-0036, 96 flights, 6 arms, zero wedges).

## The adopted deployment-profile arm (the M5/A-B config)

Every arm shares the ADR-0028/0030 running-start profile + realistic corrected cue. Set the
cue env, then call one arm (one `--path`+`--geometry` per invocation):

```bash
cd /home/emerson/interceptor-sim
export S2_CUE_MOCK_EXTRA="--sigma-range --datum-bias-m 0.5 --latency-jitter-s 0.05 --dropout-markov --emit-velocity --vel-sigma 0.5"
scripts/mc_batch.sh --laws pursuit,pronav --speeds 9 --directions both \
  --path line --geometry standard --n 8 --y0-mag 29.3 --x0 6.5 --master-seed 42 \
  --extra-args "--dash-speed 16 --early-handoff --cue-velocity --dash-unclamp" \
  --out logs/mc_ARMNAME.csv > logs/mc_ARMNAME_stdout.log 2>&1
```
- `--n` × number of `--laws` = flight count (both directions alternate within `--n`).
- Maneuvers: `--path {line,weave,jink}` (jink is deterministic from the run seed).
- Sideways sign channel: `--geometry oblique_close --west-angle-deg 45 --x0 90` (needs the
  big `--x0`; **its target vx is NEGATIVE** — mc_batch passes `--target-vel=` attached, do
  not "fix" that to a space).
- ALWAYS `--dry-run` first to confirm the flight count and (for oblique) the runway check.

## The gate loop (per arm, sequential — NEVER two arms at once)

1. **Launch** the arm as a tracked background command (just the `mc_batch.sh` line — no
   trailing `&`, no `nohup`; a trailing `&` + run_in_background makes the *launcher* return
   exit-0 immediately while the batch runs on, misreading it as done). You get notified when
   the batch process itself exits.
2. **Wait** for completion by grepping the stdout log for the sentinel `Batch complete`.
   **CRITICAL — the self-kill footgun:** any waiter/health command whose argv contains a
   sim-process pattern (`px4`, `gz sim`, `sitl`, `bin/px4`) will be killed by mc_batch's own
   between-flight `pkill -f`. Poll the LOG STRING only:
   ```bash
   for i in $(seq 1 30); do grep -q "Batch complete" logs/mc_ARMNAME_stdout.log && break; sleep 15; done
   ```
3. **Parse** the arm CSV per law/direction (clean%, handoff, miss mean/range). Column is
   `miss_m` (not `miss_distance_m`). Compare to the sibling/prior arm for the delta.
4. **GPU health:** `dmesg | grep -icE dxgk` — a FLAT count vs the pre-arm baseline is fine
   (the `dxgkio_escape: -75` lines are benign WSL2 render noise, and correlate with the
   PRIOR teardown, not a live boot). A NEW cluster **plus a silent camera topic** is a real
   wedge → STOP and tell the builder (fix = host-side `wsl --shutdown` from Windows, only
   they can run it). Confirm teardown left no orphan sim.
5. **Cooldown** ≥120 s before the next arm (`sleep 120` inside the next launch command).
6. Each arm writes its own `--out` CSV → a mid-sweep wedge only loses that arm; cue_seeds are
   deterministic from `--master-seed`, so **resume the failed arm, never restart** the sweep.

## Analyze + close

Merge the per-arm CSVs (identical headers) into `logs/mc_final_all.csv`, then
`scripts/mc_analyze.py logs/mc_final_all.csv` (per-speed×law Pk — never pooled, ADR-0025 —
plus trajectory/hist/CDF/per-path plots). Gate the headline with `scripts/check_m5.sh`
(recomputes + asserts). Then follow the **sim-milestone** skill to commit/close.

## Hard rules (each traces to an incident)
- ONE sim at a time, at IDLE machine load (batches confound under load, ADR-0015 2nd add.).
- Two mc_batch runs at once kill each other's sims (they both `pkill` on startup).
- Never `pkill`/wait from a shell whose argv contains `mc_batch.sh` or a sim-process name.
- Sim time, never wall time, for anything measured (RTF sags under load).
- Keep the boot budget modest and watch dmesg; ~48 boots/arm-cluster is the proven-safe size.
