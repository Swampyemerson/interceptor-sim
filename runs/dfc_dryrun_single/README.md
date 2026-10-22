# Single-class DFC compile dry-run — VERDICT (2026-10-21 overnight)

**Question (overnight queue #1):** the stock dry-run (`runs/dfc_dryrun/`)
proved 80-class yolov8n compiles single-context on hailo8 — but OUR
deployment net is **1-class**. Does the real path (`--classes 1`, the NMS
config rewrite) also land single-context?

## Verdict: compiles OK, but **TWO CONTEXTS**, not one

```
[info] Trying to compile the network in a single context
[info] Single context flow failed: Recoverable single context error
[info] Using Multi-context flow
[info] Successful Compilation (compilation time: 4s)
```

- Contexts: **2** (`yolov8n_context_0/1`), inter-context tensor bandwidth
  12.5 Mbps/frame, DDR 0.0 Mbps. Mapping time 3 m 54 s. `compile_1c_640.log`.
- Same container recipe, venv, calib set, and end-nodes extraction as the
  stock run (end nodes unchanged by the nc=1 surgery: the six
  `/model.22/cv{2,3}.{0,1,2}/*.2/Conv`).

**What this changes:** the headline "single context at 640" from the stock
dry-run does NOT automatically transfer to the deployment (1-class) compile
on this DFC 3.34 + MZ v2.17 pairing — the `--classes 1` path adds the NMS
rewrite and the mapper takes the multi-context fallback. Multi-context ≠
broken: it still compiles and runs, at some fps cost (context switching);
whether that cost matters vs the ≥10–15 Hz floor is a **benchmark-on-HW
question** (`hailortcli benchmark` on the real HAT, day-1 check in the
runbook). It does soften the "single-context fit" purchase argument from
ADR-0125 — the 26-TOPS 8 remains the right buy (the 8L would only be worse),
but the quoted fit claim should say "stock 80-class: single; 1-class via
--classes rewrite: 2 contexts, pending on-HW fps check".

Open question for the head session (not pursued here, 3-attempt rule not
spent — this was a clean verdict, not a failure): whether a different NMS
config (`classes` edited in the JSON instead of `--classes` CLI) or MZ tag
recovers single-context; worth one cell before the HAT benchmark day.

## Honesty bounds

RANDOM-WEIGHT nc=1 model (topology surgery on the stock yaml; the compile
cares about topology, not weights) + the synthetic calib set — **fit/context
evidence only, no accuracy/recall claim can be quoted from this build.**

Repro: `runs/dfc_dryrun_single/run_single.sh` inside the ubuntu:22.04
container (same bootstrap as the stock run; venv sentinel required).
