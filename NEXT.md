# NEXT — the live queue

> **Canonical state → [`docs/project_state.json`](docs/project_state.json)** · view: [dashboard](docs/dashboard.html) · hosted: https://claude.ai/code/artifact/eb5e40d1-c12a-4b87-bca0-589ad5af96fc
> **Task board → https://github.com/users/Swampyemerson/projects/1** (issues in `Swampyemerson/interceptor-sim`)
> **History → [`docs/next_archive.md`](docs/next_archive.md)** — the full 883-line queue as it stood 2026-07-26, verbatim. Reasoning lives in [`docs/decisions.md`](docs/decisions.md) (the ADRs).
>
> *Rewritten 2026-07-26: this file had grown to 883 lines of stacked superseded banners — the "too much text" problem in miniature. It is now the SHORT live queue only. **Rule: items live here or on the board, not both; superseded blocks go to the archive the same turn they are superseded, never left in place with a strikethrough.***

## Where the project is

The **sim phase is closed** — it has told us what it can. The **physical build has started**
(target frame + Pixhawk 6C Mini + Pi 5 in hand). The next real milestone is **tripod day**, which
gates the ~$740 interceptor airframe order.

**The honest headline:** aiming the launch correctly matters more than the camera does. Sub-metre
interception is real at 10 mph (8/8 flights, median 0.73 m) but it is **ballistic** — camera off.
The camera has not beaten a well-aimed blind dash at any speed or aim error tested. **Nothing yet
lands inside the 0.35 m ram radius that defines a kill (0/16).** And every one of those numbers was
measured with the launch aim solved from the target's exactly-known path — now declared an explicit
project constraint, with the cue-error sweep as the deliverable that replaces it.

## Waiting on the builder

| | what | where |
|---|---|---|
| ✅ | **Launch-aim cue** — RULED 2026-07-26: declare it a project constraint; real interceptor launches off a GPS-derived cue **latched at trigger, link then dies**; inject error to find the tolerance | [#1](https://github.com/Swampyemerson/interceptor-sim/issues/1) |
| ⬜ | Retire or re-shoot the 3 stale demo assets (they read as a ram-kill claim they are not) | [#2](https://github.com/Swampyemerson/interceptor-sim/issues/2) |
| ⬜ | Fill `docs/regulatory_site_capture.md` (which regime, which site, mid-air permitted in writing?) | [#4](https://github.com/Swampyemerson/interceptor-sim/issues/4) |
| ⬜ | Confirm the 2nd RadioMaster Pocket TX isn't already ordered (ADR-0089) | [#5](https://github.com/Swampyemerson/interceptor-sim/issues/5) |

## Next work, in order

1. **Fix the past-CPA breakoff discriminator** — [#3](https://github.com/Swampyemerson/interceptor-sim/issues/3). **Blocks everything below it in sim.** The measured-range rise test carries no information (false rises median 0.175 m vs true 0.152 m); the dead-band is measurement-ruled-out. Needs a different signal + an A/B.
2. **The cue-error sensitivity sweep** — extend the aim-error arms to 20/25/30° and find the crossover where the seeker starts earning its place. That curve is the portfolio deliverable and the derived accuracy requirement for the cue. *(Blocked by 1 — wider-error arms trip the same false abort.)*
3. **Tripod day** — [#6](https://github.com/Swampyemerson/interceptor-sim/issues/6). All desk prep is closed; needs the print, a tripod, and a field afternoon.
4. **Build the target drone** — [#7](https://github.com/Swampyemerson/interceptor-sim/issues/7), in progress. Dry-fit only; no solder or power until the smoke stopper arrives.

## Standing cautions

- **No solder/power work** on the target until the smoke stopper + consumables land (tgt-04 gate).
- **LiPos:** store ~3.8 V/cell, in the bag, never charge unattended.
- **One sim at a time, at idle load.** Gates and batches only when the machine is quiet.
- **Pre-register** any arm that could change a belief — prediction, criterion, and what a null means — *before* it flies.
