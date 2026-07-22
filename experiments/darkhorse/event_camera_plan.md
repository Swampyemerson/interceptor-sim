# Dark-horse #4 — Event / neuromorphic camera: bench / HIL test plan

> **Status: BENCH-ONLY TEST PLAN (2026-07-21).** Gazebo has no event-sensor
> model, so this dark horse is **not sim-scorable** — it is a Stage-0 bench /
> hardware-in-the-loop (HIL) plan, to run only if the software levers
> (track-before-detect #1, IMU-deblur #2) leave a residual terminal-blur wall
> and only *after* the pointing fix frames the target at all. Companion to
> `docs/seeker_acquisition_range_note.md` §3.5 (the motion-blur physics this
> attacks) and `docs/intercept_accuracy_levers.md` (event camera listed as a
> gated hardware bet).

## 1. What it is, in one paragraph

An **event camera** (a.k.a. dynamic-vision sensor / neuromorphic sensor) has no
exposure window and no frames. Each pixel independently fires an asynchronous
"event" the instant its log-brightness changes by a threshold, timestamped to
**~microseconds**. Because there is no shutter integration time, there is **no
motion smear** — the exact wall that destroys a few-pixel target near closest
approach (a 5 ms exposure smears the body ~23 px at 485 °/s; §3.5). It also has
**>120 dB dynamic range** (sun-to-shadow) and **µs latency**. The candidate part
is the **Prophesee GenX320** (320×320 event sensor, grams-class, with a Pi-5
starter/eval kit) as a **terminal-only** blur-killer, running alongside — not
replacing — the OV9281 global-shutter frame camera.

**New term — "usable-bearing depth":** the maximum LOS angular rate (°/s) at
which the sensor still yields a target bearing good enough for pro-nav (LOS-rate
estimate error below the guidance tolerance). This is the single figure of merit
the whole bench exists to measure, for the event sensor **vs** a short-exposure
OV9281.

## 2. Why it might matter here (and the hard preconditions)

- **The wall it attacks is real and mono can't beat it.** Terminal LOS rate is
  **485–1870 °/s** (ADR-0023 row 7). Any global-shutter frame camera trades
  motion blur against light: a short enough exposure to freeze 1870 °/s
  (<~0.3 ms) starves a few-pixel, possibly shaded target of photons. The event
  sensor sidesteps the trade — no integration window at all.
- **Preconditions that gate it hard (do not skip):**
  1. **Pointing must be fixed first.** A terminal blur-killer does nothing if the
     target is not in frame during the dash (the #1 wall: 0.8 % in-flight
     detection). Validate loft-then-dive + wedge before any event-camera spend.
  2. **It only bites in the last few meters**, inside the window where LOS rate
     is highest and the frame camera is already blur/photon-limited.
  3. **Software levers first.** #1 (track-before-detect, noise-limited) and #2
     (IMU-deblur, blur-limited-but-above-noise) are zero-new-hardware and must be
     measured first — the event camera is the escalation if a residual remains.

## 3. Honesty caveats (state before any number)

- **New sensor path re-earns the no-cheat audit.** An event stream is a new
  measurement source into guidance; it reads only the scene (camera + own-state),
  never `gt_*`. Every bearing it produces must pass the numeric no-cheat audit,
  same as the mono seeker.
- **Ego-motion event segmentation is itself a research problem.** A violently
  pitching/rolling quad makes *the whole scene* generate events (every edge moves
  on the sensor). Isolating the target's events from self-induced background
  events — especially against desert/foliage clutter (cf. ADR-0078, color 3×
  worse on desert clutter) — is unsolved for this platform. Budget it as
  research, not a drop-in. IMU-driven event de-rotation (warp events by the
  measured body rate before clustering) is the leading approach and must be
  measured, not assumed.
- **Not Gazebo-scorable.** No number from this plan is a sim result; all are
  bench/HIL measurements, reported with the sensor + rig + lighting recorded.
- **Sim numbers stay upper bounds.** This plan does not change the standing rule
  that the sim (no blur) overstates the mono seeker's acquisition range; the
  bench is where the mono-vs-event comparison becomes real.

## 4. The bench measurements (exact, with pass/fail bars)

All measurements at **idle-load** on the bench, each repeated **n≥5** and
reported with spread. Rig: a target chip (3D-printed 2.5–3" quad silhouette,
matte, and a specular variant) on a **calibrated rate table / pan rig** whose
angular rate is set and logged; both sensors bore-sighted on the same target
through matched-FoV optics, co-triggered, same lighting.

### M-EV-1 — Usable-bearing depth vs angular rate (the headline)
- **Sweep** the pan rate across **{50, 150, 300, 485, 900, 1400, 1870} °/s**
  (mid-course → terminal, matching ADR-0023 row 7).
- At each rate, for **event sensor** and **short-exposure OV9281** (exposures
  {0.1, 0.3, 1, 5} ms), estimate the target **bearing** every ~5 ms of sim-
  equivalent tick and compute **bearing error** and **LOS-rate error** vs the
  rig's logged truth (rig angle is scoring-only truth, not fed to the estimator).
- **Report:** usable-bearing depth = the highest rate at which LOS-rate error
  stays below the guidance tolerance (derive the tolerance from the pro-nav gain:
  `a_cmd = N·Vc·λ̇`, so tolerate λ̇ error that keeps `a_cmd` error under, say,
  10 % of the terminal accel authority — compute the exact °/s from the terminal
  Vc and N used in `m4_intercept.py`).
- **PASS bar (go-signal for HIL):** event usable-bearing depth **≥ 2×** the best
  OV9281 exposure's, at the terminal rates (≥485 °/s), *and* at a bearing σ small
  enough to feed pro-nav. If the short-exposure OV9281 already covers the terminal
  rate band, the event camera is **not needed** — stop here.

### M-EV-2 — Photon/low-light floor
- Repeat M-EV-1 at **{full, 1/4, 1/16} scene illuminance** (dusk/overcast
  proxy). Short exposure + small pixels lose light fast (§3.6); the event sensor's
  >120 dB DR should hold.
- **Report:** the illuminance at which each sensor's usable-bearing depth halves.
- **PASS bar:** event advantage **grows** as light drops (else the win is
  daylight-only and marginal).

### M-EV-3 — Ego-motion segmentation load (the research-risk probe)
- Mount the event sensor on a **gimbal/shaker** reproducing the measured terminal
  body-rate profile (pitch +40°→−35° swing, roll from jink) while the target pans.
- Run **IMU-driven event de-rotation → target clustering** and measure the target
  detection/false-alarm rate **with** vs **without** de-rotation.
- **Report:** target-cluster recall and phantom-event rate vs body rate.
- **PASS bar:** de-rotated recall **≥ 80 %** with phantom rate low enough for the
  handoff cue gate (cf. the `--track --handoff-cue-gate 8` discipline that killed
  mono phantoms, ADR-0058). If segmentation collapses under real ego-motion,
  the dark horse is **dead for this platform** regardless of M-EV-1.

### M-EV-4 — Pi-5 event-processing budget
- Stream real terminal-rate events into the **Pi-5** (the deployment board;
  Hailo-8 deferred) and profile: **events/second** at terminal rates, CPU load of
  (de-rotation + clustering + bearing extraction), end-to-end **latency**, and
  headroom against the existing detect-then-track stack (~35 fps w/ Hailo, ADR-
  0015; CPU-only mono ~5–13 fps).
- **Report:** sustained event-rate the Pi-5 handles before the pipeline falls
  behind real time; latency budget.
- **PASS bar:** terminal-rate event processing sustains real time on the Pi-5 CPU
  with **≥30 %** headroom, or a clear Hailo/accelerator path. Event rates at
  1870 °/s against clutter can hit tens of M-events/s — if the Pi-5 can't keep
  up, the win is theoretical.

## 5. Cost / weight / integration (rough, confirm at order time)

| Item | Est. cost | Est. mass | Note |
|---|---|---|---|
| Prophesee GenX320 module | ~$150–250 dev / less at volume | ~few g bare sensor | 320×320; grams-class die, but carrier + lens add up |
| Lens (matched terminal FoV) | ~$20–60 | ~5–15 g | must not re-narrow FoV (ADR-0024 crosser-walkout lesson) |
| Pi-5 + eval/starter kit | already on the BoM | — | processing board; kit exists for GenX320 |
| Second-camera integration | — | mount + wiring + power | on a sacrificial ~950 g frame, every gram/W is scrutinized |

- **Weight discipline:** the frame is deliberately cheap/sacrificial; a second
  sensor + carrier + lens is real mass and power on a thrust-limited airframe.
  Justify only if M-EV-1..M-EV-3 clear their bars.
- **FoV rule:** the event lens must keep a **wide-enough** terminal FoV — the
  narrow-FoV crosser walkout (ADR-0024, 0/12) applies to any terminal sensor.

## 6. Decision flow (how the bench feeds the project)

```
pointing fixed & target framed?  --no-->  STOP (event camera helps nothing yet)
        | yes
software levers (#1 TBD, #2 deblur) leave a terminal-blur residual?  --no-->  STOP
        | yes
M-EV-1 event usable-depth >= 2x OV9281 at >=485 deg/s?  --no-->  DEAD (mono short-exposure suffices)
        | yes
M-EV-3 de-rotated segmentation recall >= 80% under ego-motion?  --no-->  DEAD (segmentation wall)
        | yes
M-EV-4 Pi-5 sustains terminal event rate?  --no-->  needs accelerator; re-gate on Hailo
        | yes
--> log an ADR; prototype the dual mono+event terminal seeker; re-earn no-cheat audit
```

## 7. What to record (so the result survives, per the context-loss rule)

For each measurement: sensor + firmware, lens/FoV, rig rate log, lighting
(lux), n and spread, the derived usable-bearing depth, and the raw event/frame
captures archived under `logs/bench/event_camera/…`. File an ADR-lite entry
(context / options / decision / why / date) in `docs/decisions.md` and update
`docs/project_state.json` the same turn the bench returns a verdict — an update
that isn't in the contract didn't happen.

## Sources
`docs/seeker_acquisition_range_note.md` §3.5 (motion-blur physics, 485–1870 °/s,
23 px smear); `docs/intercept_accuracy_levers.md` (event camera as a gated
hardware bet + the pointing-first ordering); ADR-0023 (terminal LOS rate),
ADR-0024 (narrow-FoV crosser walkout), ADR-0058 (cue-gated handoff phantom
control), ADR-0078 (desert-clutter difficulty); Prophesee GenX320 + Pi-5 kit;
event-camera drone detection (arXiv 2508.04564) and obstacle dodging (Science
Robotics `aaz9712`) cited in the levers note.
