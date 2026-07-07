# Interceptor Sim — Interviewer-Facing Core

*(Every number below traces to a gate script, an ADR in `docs/decisions.md`, or a CSV in `logs/`. Nothing here overclaims — the honest limitations are stated as part of the story.)*

---

## 1. Resume line (one line)

> **Implemented proportional-navigation guidance for autonomous camera-only drone intercept in PX4/Gazebo SITL (MAVSDK-Python); validated via Monte-Carlo miss-distance analysis, diagnosed the fast-target kinematic limit, and recovered sub-1.5 m intercepts with a two-stage ground-cue-to-camera handoff.**

---

## 2. Quantified resume bullets (lead with impact)

- **Proved the classic guidance win, on camera only:** built a monocular visual-intercept loop (AprilTag detection → line-of-sight rate → pro-nav acceleration commands) and showed **proportional navigation held a 4.6–7.6× tighter miss than pursuit** against a 2 m/s crossing target (**0.28–0.44 m vs. 2.0–2.5 m**), confirmed across three independent verifier-gated runs. *(M4, ADR-0009)*

- **Diagnosed the fast-target limit as kinematic, not perceptual** — the sophisticated finding — via a 41-flight forensic batch: **96% of the final miss was locked at handoff** (r²≈0.96), and the ~0.4 s terminal window's physical correction capacity (½·a·t_go² = **0.72 m**) sits far below the delivered error (**1.69 m**), so **a flawless terminal seeker would cut the miss only ~25%.** Result generalizes across 3–9 m/s. *(ADR-0023, replicated ADR-0027)*

- **Recovered fast-crosser performance by fixing the mid-course track, not the seeker:** a ground cue emitting a *filtered velocity* plus a dash-speed clamp fix cut miss distance to **1.19 m at 9 m/s and 1.48 m at 12 m/s under a realistic degraded cue** (down from 2.9–3.1 m), reached handoff on 6/6 flights, and **eliminated a 33% mid-course-failure mode** — beating even the earlier *idealized*-cue baseline. *(ADR-0030)*

- **Ran it like a flight program:** **30+ logged decision records** (including reversals where offline analysis was overruled by Gazebo), scripted pass/fail milestone gates, verifier sign-off, and a **static test enforcing a no-ground-truth-cheating boundary** — so every "camera-only" intercept number is independently auditable.

---

## 3. HOW IT WORKS (big bullet points, no specialist background needed)

- **The mission.** A small quadcopter that catches another drone. It uses only its own forward camera to see the target and fly itself into an intercept — a "counter-drone interceptor." Everything runs in simulation (PX4 flight stack + Gazebo physics, headless), and every run is logged so the numbers are reproducible.

- **How it "sees."** The target carries an **AprilTag** — a printed QR-like marker. From one camera image plus the lens calibration, the software recovers the direction and distance to the tag. That gives the interceptor a **bearing** (a pointing angle) to the target, ~14 times a second.

- **How it steers — proportional navigation.** Instead of chasing where the target *is* (which always lags a mover), it uses the guidance law real missiles use. *One-line idea:* if you hold your **line of sight** to the target at a steady angle while the range closes, you're on a collision course — so pro-nav simply turns *proportional to how fast that bearing is rotating* and drives the rotation to zero. Gain N = 5.

- **The headline architecture — cue → handoff → camera-only.** Real engagements start with an *external* sensor (a ground rig) giving a rough cue. Here a mocked ground cue steers a fast **mid-course dash** toward the target, then performs a **one-way handoff** to the onboard camera for the **terminal phase** — after which the ground link is structurally cut off. This is the whole point: **when the datalink is jammed or denied, the drone finishes the intercept on its own.**

- **Pro-nav beats pursuit — measured.** Against a slow crossing target, pro-nav was **4.6–7.6× more accurate** than naïve pursuit (sub-half-meter vs. ~2 m). That's the textbook result, demonstrated in a full physics sim on camera data alone.

- **The mature finding — the fast-target wall is geometry, not eyesight.** When the target moves fast, the miss doesn't come from a bad camera — it's baked in *before* the terminal phase even starts. Forensics showed **96% of the miss is set at handoff**, and the final seconds are physically too short to fix the delivered geometry (correction capacity 0.72 m vs. 1.69 m of error). So a better seeker barely helps; the real levers are **getting there earlier** and **how you define a "kill."**

- **The recovery — fix the dash, not the seeker.** Two changes to the *mid-course* solved it: launching from the ground with a longer, faster run-up (so a 12 m/s crosser goes from *uncatchable-from-hover* to catchable), and having the ground cue send a **filtered velocity** instead of raw position. Under a *realistic degraded cue* this reached **~1.2–1.5 m** miss and removed the catastrophic dropouts — and, notably, this was cross-checked to *beat* the earlier optimistic-cue baseline, not just match it.

- **Honest limitations, framed as engineering maturity.** The AprilTag is a **deliberate stand-in** for "a reliable target lock exists" — a real drone carries no marker, so the real seeker is the #1 unbuilt risk and is named as such. The ground cue is a **mocked** degraded-sensor stand-in, and "kill" is a lethal-radius closest-approach criterion, not a modeled collision. These are stated up front — knowing exactly where the sim ends and the real problem begins is itself part of the credibility.

---

## 4. 30-second verbal pitch

> "It's a simulated counter-drone interceptor — a quadcopter that catches another drone using only its own camera. It detects a marker on the target, turns that into a bearing, and steers with proportional navigation, the same guidance law missiles use. The headline is a two-stage engagement: a ground sensor cues a fast dash, then hands off to camera-only for the terminal phase — so if the datalink gets jammed, the drone finishes on its own. The interesting part wasn't just that pro-nav beat pursuit by about 5-to-7×; it's that when I pushed to fast targets and the intercepts stopped tightening, I ran a 41-flight forensic batch and *proved* the miss was kinematic — locked in before the terminal phase — not a perception problem. So instead of chasing a better camera, I fixed the mid-course track and got it back under a meter and a half. And I'm upfront that the marker and the ground cue are stand-ins — the real seeker is the honest next risk."
