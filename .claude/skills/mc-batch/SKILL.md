---
name: mc-batch
description: Run a Monte-Carlo batch arm (scripts/mc_batch.sh) safely and gate it — the launch → wait → parse → GPU-health → cooldown loop, with the self-kill/GPU-wedge footguns baked in. Use whenever running an mc_batch arm or a multi-arm sweep (M5 final batch, EKF/seeker A/B, bird decoy MC). Complements px4-gazebo (env) and sim-milestone (close-out).
---
# Running a Monte-Carlo batch arm, safely

`scripts/mc_batch.sh` boots a **completely fresh sim per flight** (boots == flights), so
a batch is many boot/kill cycles — the exact thing that can wedge the WSL2 GPU. This skill
is the loop that ran the M5 final batch (ADR-0036, 96 flights, 6 arms, zero wedges).

## The adopted deployment-profile arm (the M5/A-B config)

**Canonical entry point: `scripts/mc_deployment_arm.sh`.** It bakes in the full ADOPTED
ADR-0058/0062 deployment argv and env so an arm can no longer be launched from a stale
template. Read its header before using it — it is the single source of truth for what
"the adopted config" means at any given time. Do not hand-roll the `mc_batch.sh` invocation
below from memory; call the wrapper instead:

```bash
cd ~/interceptor-sim
scripts/mc_deployment_arm.sh --path weave --n 16 --out logs/mc_ARMNAME.csv           # echo only (default): prints env + command
scripts/mc_deployment_arm.sh --path weave --n 16 --out logs/mc_ARMNAME.csv --dry-run # mc_batch plan print, boots nothing
scripts/mc_deployment_arm.sh --path weave --n 16 --out logs/mc_ARMNAME.csv --go      # actually fly (idle load only)
```
- `--path {line,weave,jink}`, `--n N`, `--master-seed S` (default 42), `--speeds V` (default
  12), `--out logs/NAME.csv`. `--force` overrides the idle-load gate.
- The wrapper sets the markerless flight env (`MC_WORLD`, `MC_TARGET_MODEL`, `MC_SEEKER`,
  `MC_VENV_PYTHON`, `MARKERLESS_NN_WEIGHTS`) and the realistic-cue env
  (`S2_CUE_MOCK_EXTRA`) itself — do not set these by hand for a deployment arm, and do not
  copy the extra-args string out of the wrapper into a separate `mc_batch.sh` call; call the
  wrapper.
- It already refuses to run with a leftover `models/mono_cam` up-tilt shadow, refuses a
  second sim, and gates on 1-min loadavg — see its header for the full safety-rail list.
- `--n` × number of laws in the wrapper's fixed `--laws pronav` = flight count (both
  directions alternate within `--n`). Maneuvers: `--path {line,weave,jink}` (jink is
  deterministic from the run seed).
- For a **non-deployment** arm (a different law/geometry/extra-args combo the wrapper
  doesn't cover — e.g. a pursuit comparator, or `--geometry oblique_close`), call
  `scripts/mc_batch.sh` directly and mirror the wrapper's env/argv by hand; sideways sign
  channel needs the big `--x0` and **its target vx is NEGATIVE** — mc_batch passes
  `--target-vel=` attached, do not "fix" that to a space.
- ALWAYS `--dry-run` first (wrapper or raw `mc_batch.sh`) to confirm the flight count and
  (for oblique) the runway check.

## The gate loop (per arm, sequential — NEVER two arms at once)

1. **Launch** the arm as a tracked background command (just the `mc_batch.sh` line, or the
   `scripts/mc_deployment_arm.sh ... --go` line for a deployment arm — no trailing `&`, no
   `nohup`; a trailing `&` + run_in_background makes the *launcher* return exit-0
   immediately while the batch runs on, misreading it as done). You get notified when the
   batch process itself exits.
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
