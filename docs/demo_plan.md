# Demo video + portfolio presentation plan

*Synthesized 2026-07-06 from a research+design workflow (portfolio presentation,
video-production, HUD-overlay, repo-capability, and four design agents). This is
the plan for the M5 demo and the overall portfolio packaging. TODO at the bottom.*

## Bottom line

The project's **substance is already strong and rare** for a student portfolio —
a pro-nav-vs-pursuit A/B with logged miss distances, a two-stage cue→handoff→
camera-only architecture, a kinematic-limit root-cause finding, 30+ ADRs with
dissent, verifier-gated milestones, and a README that traces every number to a
log. **The gap is packaging, not engineering.** The single highest-leverage
deliverable is a **2–3 minute produced demo video**, built as a data-driven HUD
composited beside the Gazebo GUI flight — because a working demo is worth 10× a
static image, and the closest comparable repo (emircansurucu/missile-guidance-sim,
which has similar rigor) notably has *no* video. That's the gap we beat.

**Hard gating rule:** everything below except drafts against already-validated
M3/M4/S2 numbers is **gated on the guidance being finalized** (Tier-2 concluded;
M5 Monte-Carlo pending). Do not film a "hero take" or publish numbers from a dev
log — the behavior and numbers are still moving. Build the *tooling* now; shoot
the *final* flight last.

## Part 1 — The demo video

### How it's built (the professional way: data-driven, not screen-only)
Render the flight in the Gazebo GUI (GPU, `scripts/sim_gui.sh` already boots a
3rd-person CameraTracking follow view + the onboard camera as a gz image topic),
capture it to video, and **composite a HUD panel generated offline from the
per-tick CSV** beside it. The HUD is *not* rendered in-sim — it's a
frame-per-tick image sequence from the logged columns, so every readout traces to
data. Tooling: matplotlib + OpenCV (already in the venv) + ffmpeg (needs install).

### The HUD panel (every widget → a real CSV column)
Aerospace glass-cockpit look (near-black, monospace, green/amber/red):
- **PHASE banner** ← `phase` (CUE_WAIT / DASH / ENGAGE / BREAKOFF).
- **SENSOR / guidance-mode lamp** — the headline. "EXTERNAL CUE (mocked ground
  sensor)" during DASH → flips to "CAMERA-ONLY" at handoff. *Gotcha (design agent
  caught this): HANDOFF is NOT a phase value — synthesize the flip from the tick
  where `ext_fresh` goes 1→0 (the cue channel blanking is the comms-denied proof).*
- **INTERCEPT SOLUTION: solving → converged** ← the `tgt_*_hat` track state (the
  "has it solved the end location yet" readout).
- **Closing speed** `vc_m_s`, **range** `r_hat_m`, **LOS rate** `lambda_dot_deg_s`
  (the live pro-nav signal), **law** (pursuit/pronav).
- **Top-down mini-map** (East-North): own ship + heading (`gt_cam_*`, `psi_deg`),
  target track, the intercept triangle, and at closest approach a **lethal-radius
  ring** (ADR-0025: ram ~0.5 m / net ~1.5 m) + closest-approach marker + miss.
- *Gotcha: NaN-parse blank cells (TAKEOFF/CUE_WAIT have blanks) — a 0-fill would
  draw a false "0 m range / LOCK".*

### Storyboard (9 beats, ~2–3 min)
1. **Title/context card** — one line: camera-only pro-nav counter-UAS intercept.
2. **Far approach** — the AprilTag target inbound at FPV speed; interceptor holding.
3. **Launch** — interceptor commits.
4. **External-cue DASH** — HUD shows EXTERNAL CUE mode + the intercept solution
   converging as the dash closes.
5. **HANDOFF (the headline beat)** — highlight the moment: SENSOR flips to
   CAMERA-ONLY, `ext_*` blanks on-screen = the comms-denied proof.
6. **Camera-only terminal pro-nav** — LOS-rate needle working, range collapsing.
7. **Maneuvering-target adaptation** — target jinks, interceptor re-solves. *(Needs
   the S3 mover built — see TODO. Flag clearly; do not stage from straight-line data.)*
8. **Intercept + honest kill** — lethal-radius ring + closest-approach marker +
   miss distance, LABELED "closest-approach < R_lethal criterion, not a physics
   collision" (there is no collision volume in the sim).
9. **Metrics outro** — cite only validated numbers (M3 0.018/0.035 m; M4 pro-nav
   vs pursuit 4.6–7.6× tighter; S2 camera-only handoff; M5 Pk-vs-radius when done).

Two cuts: a **SHIP-NOW cut** (beats 1–6, 8, 9) that needs no new mover, and a
**FULL-VISION cut** that adds beat 7 once S3 exists.

### Honest-depiction rules (non-negotiable — the field is primed to spot fakes)
On-screen text overlays for: the playback speed-ramp (the engagement is only ~4 s
of sim time; takeoff ~19 s — disclose any speedup); "external cue is a mocked
ground-sensor stand-in"; "kill = lethal-radius criterion, not a modeled collision";
and, for beat 7, that the maneuver is a built sim-time velocity schedule. In a
defense-adjacent project, disclosed honesty reads as credibility; a faked kill or
undisclosed mock is the fastest way to look naive.

## Part 2 — The portfolio package (5 parts, prioritized)

1. **Resume line + 3–4 quantified bullets** (single-sourced, mirrored everywhere):
   the pro-nav-vs-pursuit miss ratio, the two-stage camera-only handoff, the
   Monte-Carlo Pk validation. Defendable from a specific CSV in interview.
2. **GitHub repo + README** (the canonical hub): add a 5-line recruiter TL;DR at
   the top, an architecture diagram (DASH→HANDOFF→ENGAGE phase + data flow), keep
   the honesty section above the results, embed the video/GIF.
3. **The demo video** (above).
4. **Technical writeup** `docs/WRITEUP.md` — the guidance-ladder story + LOS-rate
   primer + frames + the two-stage architecture + results + honest limitations +
   sim-to-real gap. This is the interview-defense script (pre-answer "why N=5",
   "why ENU/FRD", "what does the tag hide", "what breaks on real hardware").
5. **One-page recruiter PDF** — resume line + bullets + hero image + honesty
   one-liner + three links.

Publish: repo public (scrub any parent-project proprietary detail), unlisted
video, cross-linked, one LinkedIn post.

## TODO (ordered; gated where noted)

**Demo polish — builder feedback (2026-07-07), do in the next capture pass:**
- [x] Target drone was oriented sideways (tag on its "front" while it crosses) →
      re-orient body to fly forward, tag on the -X side (detection preserved).
- [x] HUD track viz oversold the chaos → break the estimate line at real signal
      dropouts, plot smooth ground-truth as reference, label the degraded cue.
- [x] Target sat static until engage → make it incoming from the start of the shot.
- [x] Flat green ground → subtle grid/checker for a sense of speed (demo world only).
- [ ] **More drone-/OSD-like HUD info, but ONLY if genuinely useful — no cringe.**
      Recreate a real FPV/interceptor OSD feel with fields that actually carry meaning:
      candidates worth adding (each traces to a CSV column) — battery/flight-time
      proxy, altitude, a compact attitude/horizon indicator, ground speed, a
      time-to-intercept / t_go readout, a range-closure bar. Cut anything decorative
      that doesn't inform. Keep the aerospace-instrument look; legibility over density.
- [ ] **Final onboard frame = the closest-approach (CPA) frame, with a
      "PROXIMITY FUSE — DETONATE" overlay.** MUST be honest: pair the label with the
      real trigger number (e.g. "CPA 1.06 m < 1.5 m lethal radius") so it reads as the
      ratified proximity/lethal-radius criterion being met (ADR-0025), NOT a modeled
      detonation (the sim has no collision/blast — keep the README/WRITEUP disclosure).
      A brief stylized flash is fine if clearly the criterion, not a physics explosion.

**Now (ungated, safe to build against validated numbers/tooling):**
- [ ] Install `ffmpeg` (`sudo apt install -y ffmpeg` — apt is allowed; may prompt,
      so Emerson may need to run it via `! sudo apt install -y ffmpeg`). Blocks the
      whole capture/composite pipeline.
- [ ] Add a `t_sim` column to the per-tick CSV (`m4_intercept.py` CSV_HEADER +
      thread `sim_clock.t`; also log `handoff_t`). Needed for HUD↔video time-sync.
      Small, independent, low-risk. Re-run affects nothing but adds the column.
- [ ] Build `scripts/render_hud.py` (a.k.a. make_hud): CSV → RGBA HUD frames,
      glass-cockpit chrome (cv2/PIL) + matplotlib mini-map/tapes/LOS-needle, ZOH
      readouts, the mocked-cue→camera-only lamp keyed off `ext_fresh` 1→0, NaN-safe.
      Validate against an existing clean pronav log.
- [ ] Build the honest lethal-radius kill graphic + top-down closest-approach plot
      (ring at ADR-0025 radius + CPA marker + miss).
- [ ] Draft `docs/WRITEUP.md` skeleton by assembling existing docs + ADRs.
- [ ] Lock the resume line + quantified bullets; add the README recruiter TL;DR +
      architecture diagram.

**Gated on guidance finalized (Tier-2 done; M5 Monte-Carlo):**
- [ ] Build the **S3 maneuvering mover**: sim-time-scheduled *velocity* change in
      `m4_target_mover.py` (translation only — the board face is fixed, ADR-0010 #6).
      Enables beat 7. Medium.
- [ ] Build `scripts/demo_run.sh`: retry-until-clean scripted GUI flight
      (`sim_gui.sh` + `m4_intercept.py --fpv --handoff`, **`--early-handoff`** per
      the Tier-2 verdict) + screen/VideoRecorder capture + onboard-camera capture.
- [ ] Generate the canonical **hero-take log** (only after guidance + M5 freeze).
- [ ] Build `scripts/compose_demo.sh`: ffmpeg time-align (HANDOFF tick as sync
      anchor) + hstack flight video + HUD + kill graphic + a pursuit-vs-pronav A/B
      split → 2–3 min MP4 + README GIF. `.gitignore` the media.
- [ ] `scripts/check_demo.sh` (verifier): validate manifest/outputs/durations.
- [ ] Fill README M5 Pk-vs-radius numbers + plots; produce the recruiter PDF;
      publish + cross-link.

**Risks to watch:** ffmpeg absent (blocks pipeline); VideoRecorder under WSLg may
drop frames at ~0.5 RTF (fall back to capturing the onboard gz image topic to PNGs);
the full engagement is only ~4 s sim (disciplined disclosed speed-ramps needed);
and above all — **don't publish dev-log numbers; regenerate the hero take after
guidance freezes.**
