# NEXT — the live queue

> **Canonical state → [`project_state.json`](project_state.json)** · view: [dashboard](dashboard.html)
> **Task board → https://github.com/users/Swampyemerson/projects/1** (issues in `Swampyemerson/interceptor-sim`)
> **History → [`next_archive.md`](next_archive.md)** — the full 883-line queue as it stood 2026-07-26, verbatim. Reasoning lives in [`decisions.md`](decisions.md) (the ADRs).
>
> *Rewritten 2026-07-26: this file had grown to 883 lines of stacked superseded banners — the "too much text" problem in miniature. It is now the SHORT live queue only. **Rule: items live here or on the board, not both; superseded blocks go to the archive the same turn they are superseded, never left in place with a strikethrough.***

## Where the project is

The **sim phase is closed** — it has told us what it can. The **physical build has started**
(target frame + Pixhawk 6C Mini + Pi 5 in hand). The next real milestone is **tripod day**, which
gates the ~$740 interceptor airframe order.

**The honest headline:** aiming the launch correctly matters more than the camera does. Sub-metre
interception is real at 10 mph (8/8 flights, median 0.73 m) but it is **ballistic** — camera off.
The camera has not beaten a well-aimed blind dash at any speed or aim error tested. **Nothing
reliably lands inside the 0.35 m ram radius that defines a kill** (best config 3/16 logged,
5/16 interpolated under the corrected centre-to-centre scorer — `docs/rescore_2026-08-10.md`,
which retracted an interim 12/16 claim). And every one of those numbers was
measured with the launch aim solved from the target's exactly-known path — now declared an explicit
project constraint, with the cue-error sweep as the deliverable that replaces it.

## Waiting on the builder

| | what | where |
|---|---|---|
| ✅ | **Launch-aim cue** — RULED 2026-07-26: declare it a project constraint; real interceptor launches off a GPS-derived cue **latched at trigger, link then dies**; inject error to find the tolerance | [#1](https://github.com/Swampyemerson/interceptor-sim/issues/1) |
| ⬜ | Retire or re-shoot the 3 stale demo assets (they read as a ram-kill claim they are not) | [#2](https://github.com/Swampyemerson/interceptor-sim/issues/2) |
| ⬜ | Fill `docs/regulatory_site_capture.md` (which regime, which site, mid-air permitted in writing?) | [#4](https://github.com/Swampyemerson/interceptor-sim/issues/4) |
| ⬜ | Confirm the 2nd RadioMaster Pocket TX isn't already ordered (ADR-0089) | [#5](https://github.com/Swampyemerson/interceptor-sim/issues/5) |
| ⬜ | **Publication checklist** (repo is ALREADY public): rotate the VM password, re-point the launcher, merge the cleanup branch to `main`, set About/topics. MIT license **CONFIRMED 2026-08-19** | [`publish_cleanup_work_order.md`](publish_cleanup_work_order.md) |

## Next work, in order

1. **Fix the past-CPA breakoff discriminator** — [#3](https://github.com/Swampyemerson/interceptor-sim/issues/3). **Blocks everything below it in sim.** The measured-range rise test carries no information (false rises median 0.175 m vs true 0.152 m); the dead-band is measurement-ruled-out. Needs a different signal + an A/B.
2. **The cue-error sensitivity sweep** — extend the aim-error arms to 20/25/30° and find the crossover where the seeker starts earning its place. That curve is the portfolio deliverable and the derived accuracy requirement for the cue. *(Blocked by 1 — wider-error arms trip the same false abort.)*
3. **The vertical channel — REPRIORITISED 2026-09-10 after my own measurement was corrected (ADR-0099 second addendum).** The miss is 82% vertical on the adopted config and the terminal has no vertical steering at all; ADR-0085 decided one in July and no code was ever written. **What changed:** the "+0.320 m of dash drift, 66% of the miss, the error is delivered by the dash" finding was measured against a baseline that included the TAKEOFF (altitude reads 0.000 on the ground), and is retracted. Against a settled hover the dash drifts **−0.019 m**. The corrected split is **23% a datum offset present before the dash, 10% dash, 66% after handoff**. So the queue order is now:
   - **3a. Reconcile the two altitude datums — DO THIS FIRST, it is the cheapest 23% in the project and needs no arm.** `ALT_REF_M = 0.5` is AGL above the arm point; the target's altitude is world z; nothing reconciles the ~0.22 m the camera sits above ground at rest. One line of scenario setup, no guidance change, and it removes 23% of the vertical error. Desk work.
   - **3b. Log attitude in the flight CSV** (deep-audit DEEP-R2). Without it, ~24% of the post-handoff term is un-attributable: `gt_cam_z − alt_m` should be a constant offset and moves +0.087 m hover→CPA, i.e. the camera swinging on its forward boom is being counted as vertical miss.
   - **3c. Measure the hover steady-state error.** The altitude loop sits ≈ −0.087 m off its reference while continuously commanding −0.087 m/s. If that loop does not track its own setpoint, **both** levers below are built on sand. Needs a sim run.
   - **3d. Then the arms.** `dash_alt_trim_m` is a **10%** lever, and on the adopted coded-dash config there is no measured drift to derive it from (no pre-dash hover, so the tool fails closed — that is correct, not a bug). `--dropout-hold-vertical` is registered with a **null** prediction and must fly as a **2×2 with `--terminal-coast-gate`**, never standalone: the far-dropout branch zeroes vertical and horizontal on the same 792 ticks (0 discordant), so a vertical-only arm is not identifiable. Pre-registration: `docs/vertical_channel_prereg.md` §8.
4. **Make the gz stub finder deterministic — small, and it recovered 106 CI tests already.** `tests/test_rescore_cpa.py` appends a `gz`/`mavsdk` stub finder to `sys.meta_path` at module scope, and three other files plus `test_ekf_tracker.py` silently depend on that process-wide side effect via collection ORDER. Ignoring that one file in CI broke the other four, which is why `.github/workflows/ci.yml` carried 5 unnecessary `--ignore` entries (772 → 878 passing once removed). The coupling is now pinned by `tests/test_ci_gz_deselect_list.py` so it fails with an explanation rather than a mystery — but the durable fix is to move the stub into a `conftest.py` so it is deterministic rather than alphabetical.
5. **Check the money gate's closing speed — a ~$740 decision.** The gate prices the streak burn at 9.0 m/s; the committed logs measure **17.60 m/s** in the last second before handoff (`scripts/forensics/handoff_closing_speed.py`), which under-prices the burn ~1.96× in the flattering direction. Same shape as the invented 30 fps (ADR-0082). Confirm with `--phase CODED_DASH` on the dev machine, then decide — deliberately NOT patched, because it can flip a published PASS/FAIL.
6. **Bench bring-up — UNBLOCKED, hardware in hand (2026-07-26).** Two independent chains, both fully in hand: the **seeker rig** (`skr-01`→`skr-06`, Pi 5 + OV9281, ends at the calibration gate) and the **brain bench** (`brn-01`→`brn-05`, Pi 5 + 6C Mini + M10, ends at the props-off OFFBOARD gate — the one link the sim never exercised). Packs are written: `scripts/pi_setup/`, `configs/px4_6cmini/`. Pull `skr-03` (exposure ≤1 ms) forward — a null there kills tripod day's motion-blur read.
   - ✅ **`skr-07` DONE remotely 2026-07-26 — and it moved a number the gate turns on.** Sustained on the real Pi 5 through 750 s soaks, tag in frame, **no throttling on any arm**: **96.6 fps** at `quad_decimate=2.0`, **38.2** at 1.0, **CPU-YOLO 6.09 fps**. The seeker had been running at 30 fps because neither `pi_capture.py` nor `seeker_loop.py:PicameraSource` sets picamera2's `FrameDurationLimits` — an inherited default, not the sensor (143 fps) and not the CPU. `R_streak_burn` goes as 1/fps, so this cuts the range burned forming the handoff ~3.2×, and `quad_decimate=1.0` (the range/incidence reclaim lever) is now affordable at 38.2 fps — above the 30 the project assumed at the *coarse* setting. ADR-0090; harness `scripts/seeker/pi_fps_soak.py`; data `runs/skr07_tagged/`.
   - ✅ **The frame-duration cap is WIRED — this line used to say "still open" and was stale (corrected 2026-08-10).** `seeker_loop.py:PicameraSource` pins `FrameDurationLimits` from `--camera-fps`, default `CAMERA_FPS_DEFAULT = 60.0` — exactly ADR-0090's recommendation, not uncapped. It refuses a frame duration shorter than the exposure, and it logs what was **applied** rather than what was intended (an earlier version printed success off `target_fps` alone and a mutation test that deleted the assignment still saw it claim victory). Regression-tested: `flight/tests/test_camera_fps_pinned.py`, 4 passed. Note the **recorder** (`pi_capture.py`) is deliberately separate and still inherits ~30 fps — a live 5-frame check on 2026-08-10 measured `stream_fps=30.799`. That is correct for a bench recorder that pays a per-frame PNG write, but do **not** quote a recorder rate as a seeker cadence.
   - **Still open:** CPU-YOLO at 6.09 fps **confirms** the ~5–10 fps anchor, so the Hailo HAT requirement stands and the frame-rate win does **not** transfer to the markerless path.
7. **Tripod day** — [#6](https://github.com/Swampyemerson/interceptor-sim/issues/6). Desk prep closed; needs the print, a tripod, a field afternoon — **and the target flying**, so it waits on 8.
8. **Build the target drone** — [#7](https://github.com/Swampyemerson/interceptor-sim/issues/7). **Blocked on the Kakute H7 + power parts, still in transit.** Everything else for it (SD, tools, solder kit, safety gate) is in hand. Flash + prove `.BIN` logging (`tgt-01`/`tgt-02`) with the board loose on the desk before it goes into the stack.

## Standing cautions

- **Every first power-up after soldering goes through the smoke stopper, props off** (tgt-04 gate). The stopper is in hand as of 2026-07-26 — the gate is now a step to perform, not a part to wait for.
- **LiPos:** store ~3.8 V/cell, in the bag, never charge unattended.
- **One sim at a time, at idle load.** Gates and batches only when the machine is quiet.
- **A green run is only as good as the machine that produced it** — `docs/verification_environment.md` says what the dev box, CI and a fresh cloud clone each can and cannot verify. Nothing from a cloud session has flown.
- **Pre-register** any arm that could change a belief — prediction, criterion, and what a null means — *before* it flies.
