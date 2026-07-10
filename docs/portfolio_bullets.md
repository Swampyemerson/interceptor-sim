# Interceptor Sim — Interviewer-Facing Core

*(Every number below traces to a gate script, an ADR in `docs/decisions.md`, or a
CSV in `logs/`. Nothing here overclaims — the honest limitations are stated as
part of the story. Updated 2026-07-09 for the post-M5 arcs: the markerless
seeker, the detect-then-track maneuvering terminal (ADR-0058), the real stereo
pipeline spine (ADR-0045..0053), and the sim-to-real design review; r²/ZEM
wording corrected 2026-07-10 (r² is variance explained, never "% of the miss" —
audit H5). Bullets that
depend on a claim the project itself has put on HOLD are explicitly marked
**[pending validation]** — do not use them unmarked.)*

---

## 1. Resume line (one line)

> **Implemented proportional-navigation guidance for autonomous camera-only
> drone intercept in PX4/Gazebo SITL (MAVSDK-Python); validated via
> Monte-Carlo miss-distance analysis (n=96), then removed the fiducial — a
> markerless neural-net seeker with a verified anti-phantom detect-then-track
> terminal holds camera-only intercepts against a 12 m/s weaving target.**

---

## 2. Quantified resume bullets (lead with impact)

- **Proved the classic guidance win, on camera only:** built a monocular
  visual-intercept loop (detection → line-of-sight rate → pro-nav acceleration
  commands) and showed **proportional navigation held a 4.6–7.6× tighter miss
  than pursuit** against a 2 m/s crossing target (**0.28–0.44 m vs. 2.0–2.5 m**),
  confirmed across three independent verifier-gated runs. *(M4, ADR-0009)*

- **Validated by Monte-Carlo at fleet scale:** a 96-flight final batch across
  pursuit/pro-nav × 6/9/12 m/s × straight/weave/jink/oblique target paths on
  the adopted deployment profile — **96.9% clean, mean miss 1.08 m, median
  0.93 m**, the whole 6–12 m/s FPV speed band catchable (12 m/s was uncatchable
  from a hover start), with the kill metric reported as a **full
  Pk-vs-lethal-radius curve**, never a single cherry-picked radius. *(M5,
  ADR-0036; `logs/mc_final_all.csv`; sensitivity: `docs/pk_vs_radius_note.md`)*

- **Diagnosed the fast-target limit as kinematic, not perceptual** via a
  41-flight forensic batch: **the final miss is a near-deterministic function
  of handoff geometry** (r²≈0.96 vs zero-effort-miss@handoff — variance
  explained, not a fraction of the miss), and the ~0.4 s terminal window's
  physical correction capacity (½·a·t_go² = **0.72 m**) sits far below the
  delivered error (**1.69 m**) — so a flawless terminal seeker would cut the
  miss only ~25%; ~75% is physically locked in.
  The fix that actually worked attacked the mid-course geometry, not the
  camera. *(ADR-0023, replicated ADR-0027; recovery ADR-0028/0030)*

- **Killed the fiducial, then fixed what broke — with a control:** replaced the
  AprilTag with a markerless neural-net seeker on a tag-less body; at 12 m/s +
  maneuver it threw sporadic high-confidence *phantom* detections (~18 m off
  target, confidence **inverted** — phantoms 0.34 vs real 0.17), causing 12/16
  false handoffs, while an AprilTag control at identical kinematics held 8/8 —
  isolating the failure to per-detection bearing quality, not guidance.
  *(ADR-0056/0057)*

- **Built the anti-phantom terminal that recovered it — detect-then-track:**
  the NN acquires once, a lightweight visual tracker (cv2 CSRT) follows
  frame-to-frame, and the NN re-validates every 8 frames. Post-handoff
  **camera-terminal Pk@2.5 m: weave 3/16 → 14/14, jink 1/8 → 14/15; phantom
  handoffs 12 → 0; 0 of 155 terminal detections false** in the pre-registered
  deployment-config validation arm — within ~0.4 m of the AprilTag perception
  ceiling (8/8 @ 1.64 m median). *(ADR-0058, commit `d10c9ca`,
  verifier-passed; `logs/mc_t21_trackgate_weave12_r2.csv`)*

- **Replaced the mocked ground sensor with a real computed pipeline:** a
  rendered two-camera stereo rig → NN detection → live triangulation → real
  UDP link, with the interceptor flying on the **genuinely computed** cue
  end-to-end (real cue tracks truth to **0.99 m median**) and the stereo range
  error validated against the σ_R ∝ R² physics model across 50–160 m (measured
  exponent 2.003). *(ADR-0045..0053, `check_t19.sh` exit 0)*

- **Ran it like a flight program:** ~60 logged decision records including
  **public retractions** (an n=2 "win" re-read as noise against a measured
  ~5 m run-to-run floor, ADR-0057), scripted pass/fail milestone gates with
  independent verifier sign-off, a statically-enforced
  no-ground-truth-cheating boundary (AST-pinned tests; the cue is
  *structurally unreadable* after handoff), and Pk reported at **three
  attribution levels** (whole-flight / post-handoff camera-terminal /
  real-handoff-conditioned) so the flattering pooled number is never quoted
  alone.

- **Audited my own headline before an interviewer could:** a multi-agent
  sim-to-real design review found the adopted anti-phantom config **fails
  closed under a mid-dash cue jam** (confirmed by independent code trace,
  ADR-0059) — so the comms-denied claim was placed on HOLD project-wide, a fix
  was built and unit/honesty-tested (86 tests), and a paired jam Monte-Carlo
  harness queued; the same review measured (from flight logs) that the dash
  pitch throws the target *above* the camera's field of view ~35% of the time,
  inverting the naive framing and driving a real camera-mount design decision.
  *(ADR-0059/0060, `docs/design_review_sim_to_real_2026-07-10.md`)*

- **[pending validation] Comms-denied terminal:** the architecture is built for
  it — a one-way handoff latch makes the ground link structurally unreadable
  once the camera terminal locks, and every "camera-only" number above is
  audited against that boundary. But a cue jammed *before* camera acquisition
  currently defeats the adopted config (ADR-0059), so **"works comms-denied /
  jam-resistant" is a held claim until the jam Monte-Carlo arm lands.** Use
  only as: *"designed a structurally comms-denied terminal; the mid-dash jam
  case is in validation."*

### Claims currently held — do not put on a resume

- "Works comms-denied / jam-resistant" (fails closed under pre-acquisition jam
  in the adopted config; fix built, not yet flown — ADR-0059).
- "Pk ≥ 95%" (14/14 observed → 95% Clopper-Pearson lower bound ~77%; the
  statistics are honest as "no failures observed," not as a Pk floor — design
  review G14).
- Anything implying the numbers transfer to real optics, real compute, wind,
  or a real airframe (every perception number is an upper bound from clean
  renders — design review §2, `docs/sim_to_real_gaps.md`).

---

## 3. HOW IT WORKS (big bullet points, no specialist background needed)

- **The mission.** A small quadcopter that catches another drone. It uses only
  its own forward camera to see the target and fly itself into an intercept —
  a "counter-drone interceptor." Everything runs in simulation (PX4 flight
  stack + Gazebo physics, headless), and every run is logged so the numbers
  are reproducible.

- **How it "sees" — two generations.** Originally the target carried an
  **AprilTag** (a printed QR-like marker) that gives a clean bearing and range
  from one camera image. That was always a disclosed stand-in. The current
  seeker is **markerless**: a small neural network finds the tag-less drone
  body in the frame, and the bearing comes from the detection box — the honest
  version of the problem.

- **How it steers — proportional navigation.** Instead of chasing where the
  target *is* (which always lags a mover), it uses the guidance law real
  missiles use. *One-line idea:* if your **line of sight** to the target stays
  at a steady angle while the range closes, you're on a collision course — so
  pro-nav turns *proportional to how fast that bearing is rotating* and drives
  the rotation to zero. Gain N = 5. It only needs an angle rate, which is why
  swapping the AprilTag for a neural net didn't change one line of guidance.

- **The headline architecture — cue → handoff → camera-only.** An external
  ground sensor (now a real computed stereo pipeline, not just a mock) steers
  a fast **mid-course dash**, then the system performs a **one-way handoff**
  to the onboard camera for the **terminal phase** — after which the ground
  link is structurally cut off, not just politely ignored. The design intent:
  when the datalink is jammed, the drone finishes the intercept on its own.
  **Status, honestly: the intent is proven for a jam *after* handoff; the
  mid-dash-jam case is in validation right now (ADR-0059)** — the project's
  own review caught a config that failed it, and the fix is built but not yet
  batch-proven.

- **The mature finding — the fast-target wall is geometry, not eyesight.**
  When the target moves fast, the miss is baked in *before* the terminal phase
  starts: **the final miss is almost entirely a function of the geometry at
  handoff** (statistically, handoff geometry explains ~96% of the run-to-run
  *spread* in miss — r²≈0.96 — which is not the same as 96% of any one miss),
  and the final seconds are physically too short to fix the delivered geometry
  (0.72 m of correction capacity vs 1.69 m of delivered error — even a perfect
  camera only cuts ~25%). So the levers are *getting there earlier with better
  geometry* — which is what the running-start + velocity-emitting-cue fix
  delivered; on the final adopted profile the whole 6–12 m/s band reads
  **mean 1.08 m / median 0.93 m over 96 flights** (ADR-0036 — the earlier
  ~1.2–1.5 m ADR-0030 figures were flown under the old, too-steep range-noise
  curve and are superseded).

- **The maneuvering fix — detect-then-track.** At 12 m/s with a weaving
  target, the neural seeker's failure wasn't "too slow" — it hallucinated
  *confident* detections ~18 m from the truth, and a control experiment with
  the AprilTag proved the guidance could catch the same target 8/8. The fix:
  **detect once, then track** — the NN identifies the target, a cheap visual
  tracker follows that same blob frame-to-frame (temporally consistent, so it
  ignores phantoms elsewhere in the frame), and the NN re-validates the track
  every 8 frames. Phantom handoffs went **12 → 0**; the camera-only terminal
  now lands 14/14 inside 2.5 m at 12 m/s weave.

- **Honest limitations, framed as engineering maturity.** The "kill" is a
  lethal-radius closest-approach criterion, not a modeled collision (and
  there's a published sensitivity note showing exactly how Pk moves with the
  assumed radius). Every perception number is an upper bound from clean
  rendered frames — no motion blur, no wind, no vibration. The comms-denied
  headline is HELD pending its jam batch. Knowing exactly where the sim ends
  and the real problem begins is itself part of the credibility.

- **A concrete, costed path to reality.** A staged plan from $0 desk
  experiments (synthetic motion blur through the exact seeker chain; the real
  detector against real-sky footage) through a ~$257 Raspberry Pi perception
  bench to a ~$1.05–1.25k full build — ordered by de-risk-per-dollar, with
  each stage gating the next purchase.

---

## 4. 30-second verbal pitch

> "It's a simulated counter-drone interceptor — a quadcopter that catches
> another drone using only its own camera. It steers with proportional
> navigation, the same guidance law missiles use, and the engagement is
> two-stage: a ground stereo sensor — a real computed pipeline, not a mock —
> cues a fast dash, then hands off to a camera-only terminal that's
> structurally cut off from the ground link. The part I'm proudest of isn't
> the 5-to-7× win over pursuit guidance — it's the discipline. When fast
> targets stopped tightening, I proved with a 41-flight forensic batch that
> the miss was kinematic, locked in at handoff, and fixed the mid-course
> instead of the camera. When I removed the target marker and the neural
> seeker started hallucinating at 12 m/s against a weaving target, I isolated
> it with an AprilTag control, built a detect-then-track terminal, and took
> phantom handoffs from twelve-in-sixteen to zero — fourteen-for-fourteen
> camera-only intercepts, reported at three honesty levels so the flattering
> number never stands alone. And when my own design review found that the
> anti-phantom config failed closed under a mid-dash jam, I put the
> jam-resistance claim on hold, built the fix, and it's in validation now —
> I'd rather show you the hole I caught than a number I can't defend."
