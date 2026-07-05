# Perception design — the real interceptor's "eyes"

*Companion to ADR-0015 (decision record). Plain-language design for the perception
half of the counter-UAS interceptor: how a real system detects, tracks, and hands
off a small, fast, non-cooperative FPV drone — the problem the AprilTag stands in
for in the sim.*

## The core idea (and a correction to the naive version)

The interceptor works as a **two-sensor → one-sensor** system:
- A **ground station** (cameras on a fixed rig) watches the sky, detects the threat,
  and tracks it — this is the "smarter sensor" that cues the cheap interceptor.
- The **interceptor's own camera** takes over for the final seconds and finishes the
  intercept **on its own**, even if the radio link is jammed. That comms-denied
  finish is the whole headline capability.

The intuitive picture — "the ground gives *range*, the drone gives *bearing*, fuse
them" — is *almost* right but misleading in one way that matters for the math:
**both** sensors measure *angles* well and *distance* poorly. A camera (ground or
onboard) is great at "what direction is it" and weak at "how far." So the fusion's
real value is two different things:
1. The ground rig **covers the mid-course**, when the target is still too small and
   far for the onboard camera to see at all.
2. The **large distance between the ground rig and the drone** lets two angle
   measurements from very different viewpoints triangulate the range far better than
   either camera could alone.

And a subtle trap: once you translate the ground station's track into the drone's own
coordinate frame, the biggest error isn't the cameras — it's the **GPS offset and
clock difference between the two machines**. Two standard GPS units can disagree by
2–3 m, and if their clocks aren't synced, you match "where the target was" to "where
the drone was pointing" a fraction of a second off. The fixes are standard: a shared
**RTK GPS base** (gets both to ~0.3–0.5 m of each other) and **GPS-time-syncing both
clocks** (the real-world version of the sim-time fix we already learned the hard way).

## The hardware, in one table

| Piece | Choice | Why | ~Cost (2026) |
|---|---|---|---|
| **Onboard brain** | Raspberry Pi 5 **+ Hailo-8/8L AI hat** | Runs real ML detection at ~35 fps / 29 ms — fast enough for the terminal — **with no ROS 2**. Pi-CPU alone is too slow (~13 fps). | +$70–110 over the existing rig |
| **Onboard eye** | Global-shutter mono camera (already chosen) | No rolling-shutter skew while the seeker yaws hard at the end | (existing) |
| **Ground brain** | Jetson Orin NX | Heavy ML + sensor fusion; on the ground, power and weight are free | ~$400–600 |
| **Ground eyes** | 2× global-shutter cameras on a ~2 m baseline | Sub-meter *range* accuracy out to ~100 m | ~$600–1,200 |
| **Ground eyes (night/all-weather)** | + thermal (FLIR Boson 640-class) | Detects a small drone against cold sky day *or* night, and rejects birds | +$1,500–3,000 (staged) |
| **Link** | MANET / telemetry radio (track data only) | Sends a tiny "here's the target" message, not video; allowed to drop by design | ~$20–300 |

**The compute headline:** this overturns an earlier hardware decision (ADR-0012), which
picked a Pi with CPU-only vision and rejected the more powerful Jetson because its ML
path needed ROS 2 (which this project bans). That was right *for the AprilTag* (a CPU
job) but wrong for real ML. It turns out a **$70 Hailo accelerator runs real neural-net
detection on the Pi with no ROS 2 at all** — so we get real ML cheaply, keep every other
decision, and the Jetson just moves to the ground where its power draw doesn't matter.

## How it detects a real drone (not a printed tag)

Against a real FPV drone there's no fiducial to lock. The reliable method is
**"detect-then-track"** in layers:
1. **Motion** — spot tiny moving specks against the sky (cheap, catches things a
   detector misses).
2. **Classify** — a small neural net decides "drone vs bird" on those specks.
3. **Track** — a correlation tracker + Kalman filter *holds* the lock and coasts
   through the frames where detection blinks out.

A single-shot detector alone loses a small, fast, blurred target; the motion + tracking
layers are what keep the lock. (Published results: this approach lifts tiny-target
recall from ~40% to ~86%.)

One honest note on the threat: **radio-detection doesn't work against modern
fiber-optic / fully-autonomous FPV drones** — they emit nothing. That's exactly *why*
the camera-based ground rig plus an onboard seeker is the right answer: it doesn't rely
on the target broadcasting anything.

## The hard part (told straight)

The single biggest risk — and no amount of money fixes it — is **holding the onboard
lock on a small, jinking drone through motion blur and vibration in the last 1–2
seconds, with the radio jammed.** Everything else exists to deliver a clean lock into
that window and make it shorter. Our own sim already shows this: in a 20-flight batch,
*every single* miss came from the camera losing the target right at the end.

Two coupled sub-problems fall out of this:
- **Jamming decides when the link dies — not us.** So the interceptor needs to keep
  flying a best-guess course and *search* for the target with its own camera if the
  link is cut before it has locked on. If it can't find the target in time, it should
  break off rather than fly blind.
- **One camera can't do both jobs.** A long lens sees the target *sooner* (good for
  acquiring before jamming) but has a *narrow* view that loses the target as it whips
  past at the end; a wide lens is the opposite. The fix order: first spend the nearly
  free "let the drone yaw faster" fix we already identified, and if that's not enough,
  add a **second wide-angle camera** for the endgame (the Hailo chip can run both).

## What this means for the intercept math (the feedback loop)

Your instinct — "work out perception first, because it changes the data the guidance
sees" — is exactly right. The real sensors deliver a *worse* stream than the sim's clean
AprilTag: slower updates, noisier angles, much noisier range, variable lag, and
**dropouts that come in bursts** right when the target maneuvers. Before we finish tuning
the guidance, we feed those real numbers into the sim (see ADR-0015's data-constraints
table) and re-measure the hit rate. That's the next build step — and it's pure
simulation, no hardware needed yet.

## What to build/buy first (cheapest go/no-go)

1. **In the sim (this week, $0):** upgrade the mocked ground sensor to the *realistic*
   numbers above (bursty dropouts, noisy range, variable lag, jam-cutoff), add the
   "link cut before lock → coast and search" behavior, and re-run the hit-rate batch.
   This tells us if the architecture survives real data *before* spending a dollar.
2. **On the bench (~$250):** Pi 5 + Hailo + camera, pointed at a real (or proxy) fast
   drone outdoors, camera vibrating — measure the five numbers that decide everything:
   detection rate, how fast the target can cross before we lose it, angle noise, how
   long dropouts last, and total lag. Feed them into the sim.
3. **Only then** buy the ground rig and airframe.

## Staged capability (honesty)

- **Proof rig:** daylight, electro-optical only — a defensible *daytime* demonstration.
- **Fielded capability:** add thermal (night + bird rejection) and radar (all-weather
  cue). Each is disclosed, not baked silently into the headline numbers.

*See ADR-0015 in `docs/decisions.md` for the full decision record, dissents, the
complete data-constraints table, and sources.*
