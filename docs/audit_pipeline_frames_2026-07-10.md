# Third Overnight Audit — Morning Report (2026-07-11)

**Scope:** stereo pipeline (T16–T19), fusion/EKF, coordinate frames, tonight's 4 commits + completeness. All findings below survived adversarial verification; false positives dropped, severities corrected, verifier-missed items folded in.

---

## 1. Headline

**Honesty held. Geometry didn't. Two real HIGHs in the code, two HIGHs in reproducibility.**

- **Both honesty audits came back CLEAN** (independently re-verified): no `gt_*` reaches the computed stereo cue (station.py's only gz subscribe is `/clock`; payload built exclusively from `triangulate()` output), and the post-latch cue guards in the EKF/guidance path are airtight (single latch site, guarded reads, counts-then-raises, AST mutation tests). The "genuinely computed stereo cue" claim stands with its disclosed caveats.
- **But the T19 headline number is softer than the ADR says.** The 0.99 m median cue error is an accident of cancellation: modeled latency alone (0.12 s floor + 0.1 s emit cadence + tick) predicts **1.3–2.2 m** at 9 m/s, so a nonzero, unmeasured station/mover clock-epoch skew is *demonstrably present* in both passing runs, partially canceling the latency lag. Same bug class ADR-0052 fixed, one level down. Clean provenance, conflated metric.
- **The frame audit found a structural error in the primary guidance signal:** bearing is yaw-compensated only — roll and pitch are never derotated (`lambda = psi + beta`, m4_intercept.py:1976; track_attitude discards everything but yaw at :1099-1100). At ADR-0060's measured 27–36° dash pitch that's a 12–24% azimuth gain error, plus sign-carrying roll cross-coupling under maneuvering, exactly in the FPV/maneuvering regime Emerson directed be fixed before T25. The `--bench` "proof" only ever spun level — the one geometry where the coupling is identically zero.
- **Tonight's 4 commits are sound** (no held-file leakage, every number reproduces), but the pre-registration package is still uncommitted with its attestation window closing, and the new `check_t21.sh` gate exits 1 on any fresh clone.

---

## 2. BLOCKERS / HIGH

### H1 — Roll/pitch never derotated from the LOS bearing (frames)
**Evidence:** m4_intercept.py:1976 (`lambda = psi + beta`), :1099-1100 (only `yaw_deg` kept), :2724; zero derotation hits repo-wide; run_bench (:2914+) validates level-hover only. Pitch inflates azimuth ~1/cos(θ) → 12–24% at ADR-0060's dash pitch; roll mixes vertical off-boresight into azimuth with roll-following sign under MPC_TILTMAX_AIR=60.
**Impact:** corrupts λ and λ̇ (the pro-nav input) hardest during dash handoffs and maneuvering terminals; plausibly contributes to the ADR-0056 maneuvering limit (causality unproven — the 12 m/s AprilTag weave still hit 1.64 m 8/8, so tolerable at *that* tilt profile).
**Fix:** **sim-gated.** Subscribe `attitude_quaternion` (own-state, honesty-legal), rotate the camera ray camera→body→NED, take `lambda = atan2(ray_e, ray_n)`. Validate with a pitched/rolled bench extension + paired-seed n≥8 A/B on the weave/jink arms. The same fix covers the missed camera-lever-arm and Euler-yaw-at-tilt items (§5).

### H2 — Station/mover co-start epoch skew: uncontrolled, unmeasured, and demonstrably nonzero (stereo)
**Evidence:** station t0 sampled after its own boot (station.py:1044); mover sim_t0 after gz import + pre-warm + clock-child spawn (m4_target_mover.py:~517); m4 co-starts them as two bare Popens with no shared epoch (m4_intercept.py:1876/1904). Every 0.1 s of skew = 0.9 m of systematic cue offset at 9 m/s. Latency-only prediction (1.3–2.2 m) exceeds the measured 0.99/1.03 m → canceling skew is *observed*, not hypothetical, and the error signature is degenerate (skew ~0.1 s and ~0.3 s both fit).
**Fix:** split. **Sim-free now:** the originally proposed log check is unimplementable (mover CSV logs *relative* sim_elapsed) — instead estimate skew from the existing flight CSVs via along-track error ≈ 9×(ext_age_s − skew), which simultaneously decomposes M1 below; instrument the mover to print absolute sim_t0 for future runs. **Sim-gated proper fix:** m4 samples its own sim clock at CUE_WAIT and passes `--epoch-t0` to both children.

### H3 — Pre-registration not committed; one attestation window already closed, one closing (commits)
**Evidence:** `eval_seeker_v3.py` +136 (the 4 pre-registered scorer fixes) still uncommitted while held decisions.md already contains ADR-0061 declaring the NULL landed — git can no longer prove patch-before-results; only train-daemon timestamps can. `check_jam_mc.py` untracked with a "written BEFORE any jam arm flew" header, and the jam MC is the next sim job. Verifier addition: `phase4_eval_v3.sh` (+30/−6) is part of the same package. **None of the three are held files.**
**Fix:** **sim-free NOW, time-critical.** Commit all three before any jam arm flies, citing daemon-log timestamps in the message as the attestation.

### H4 — check_t21.sh exits 1 on any fresh clone (commits)
**Evidence:** check_t21.sh:46-62 hard-fails if any of the 16 per-tick weave12_r2 CSVs is missing; all 16 (~1 MB) are still gitignored (`logs/*`, no re-include). The gate built to make the 14/14 headline disk-loss-durable dies on a clean clone — the direct residual of deep-audit H1's top blocker, disclosed in 2665bbb's message.
**Fix:** **sim-free now.** Commit the 16 CSVs (~1 MB vs ~574 KB already committed) or add an aggregate-only degraded mode.

---

## 3. MEDIUM / LOW (grouped)

### Stereo pipeline — gate & replay integrity (all sim-free-now)
- **M1 — assertion-4 conflates three error terms** (check_t19.sh:326-354 scores the cue at the read tick, not the cue's own t_sim): 0.99 m ≠ triangulation accuracy; ADR-0052's own wording invites the over-read. Fix: decompose from the flight CSV via `ext_age_s` (pure log math), document both numbers.
- **M2 — no trajectory-match check between cache and flown mover** (m4_intercept.py:1858-1863 checks env non-emptiness only; alignment lives in a check_t19.sh *comment*; `--dash-direction` never passed, station defaults to first index row). The cue is structurally open-loop — a mismatched dev/T21 flight chases a phantom with zero warning. *Folds in the cache-provenance gap:* `centroid_cache_meta.json` is written but consumed nowhere. Fix: m4 stereo branch loads both metas and fails loud on mismatch; gate spot-recomputes 2–3 cache rows.
- **M3 — triangulation has no geometric gates** (triangulate.py:199-201 rejects only disparity ≤ 1e-6; min_conf=0.5 is the sole other guard). The observed edge-clip outlier (seq=2, 43 m range error) was caught by *confidence*, a proxy. ADR-0052's text claims edge/small-disparity rejection that was never implemented; the T21 maneuvering caches inherit the gap. Fix: min-disparity floor, |v_l−v_r| epipolar tolerance, image-border margin — all offline-testable.
- **Low:** σ_R∝R² "validation" (ADR-0053) is near-tautological — the fit consumes zero rendered pixels, exponent 2.0 by construction; the real-detector exponent is **1.63** (ADR-0055). Route future σ_R consumers (T20 weights) to the real fit, tighten the citation. Rig pose hardcoded, never read back from sim (rig_snapshot_capture.py:132) — add a startup SDF-consistency check. Replay direction matched by convention only — derive `--dash-direction` from sign(vy) and hard-fail in station.py (sim-free; mc_batch *alternates* direction per flight, so the first stereo batch arm hits this immediately).
- **Gate hardening (verifier-missed, all sim-free):** check_t21/t19 never asserts the ADR-0048 pre-registered latency-floor condition (`n_floor_violations` is one grep away in the RUN_LOG — a floor-violating run passes today); no minimum cue-count (passes on n≥1) and the station's exit status is never checked; check_t19's "KNOWN OPEN QUESTION" header describes pre-ADR-0052 behavior (doc-rot).

### Fusion / EKF — inconsistent noise specs (accept-or-document / replay-first)
- **M4 — FusedTrack weight still uses the pre-ADR-0017 sigma model** (0.4/0.008 at m4_intercept.py:869-870 vs the corrected 4.45e-05 in ekf_tracker.py:139-140 — ~180x too steep, cue under-weighted ~6x at R=30 m). **Verifier-missed extension: FUSE_CAM_RANGE_FRAC=0.10 is also AprilTag-tier while the markerless spec is ~22%** — so ADR-0044's headline 8/8 L2 fusion win ran with a *doubly* under-weighted cue and is plausibly understated. Do NOT silently sync: the stale steep weight may be load-bearing for the WORST-bias 8/8 survival. ADR addendum now; paired A/B (nominal + WORST) before changing constants.
- **M5 — EKF measurement R is AprilTag-tier regardless of seeker** (EKFTracker() all-defaults at :1729; ML spec 1.5°/22% per decisions.md:355). Over-tight S is a plausible — not established — contributor to the 2/16 post-latch camera-gate rejections in ADR-0044 (real near-CPA dynamics also drive it; measured v2 bearing noise is 1.5–2x tag, not the spec's 3x). Fix: **offline replay of flight 33327 with widened R first** (sim-free), then a paired L1 re-fly.
- **Low (downgraded from filed medium):** the warm-seed skips the latch re-inflation floor when `_cue_called` is false — structural gap confirmed, but the filed NIS≈24 scenario is wrong: the seeded bearing is camera-owned, only attenuated along-LOS range crosses, first post-latch NIS ≈ 1. One-line flag + unit test, gate before adopting. **Verifier-missed sibling, arguably worse:** `seed_from_polar` stamps WARM_VEL_SIGMA=0.7 m/s even when the seeded velocity silently fell back to alpha-beta *differentiated* velocity (the documented ~7 m/s-noise PIP-killer) during a cue-stale window — up to ~100x understated velocity variance in exactly the comms-denied handoff the project centers on; only D3.2 backstops it. Also: D3.2 recovery protects the post-latch side only (pre-latch bias-lock — the 5/8 WORST family — has no in-filter recovery); cue R carries no latency-staleness term (~0.24–0.6 m unmodeled vs σ_pos ~0.53 m).
- **Wording/nits (sim-free):** "the cue never touches the angle" is overstated for the EKF polar split (along-LOS innovation rotates λ̂ through P's off-diagonals — legitimate Kalman behavior; tests only cover the zero-innovation case); "range_filter is camera-only — honesty verified" comment is false under `--tracker ekf --fuse-midcourse`; cold-init P, shared dt_since clock, vo-fallback-to-zero, silent singular-S — all nits, note-and-move-on.

### Coordinate frames — remaining mediums/lows
- **M6 — ψ(t_tick) paired with β(t_frame):** ~70–150 ms detection latency × yaw rate = LOS bias; constant-rate bias cancels in λ̇ (why the bench passes) but yaw *accelerations* differentiate straight into the pro-nav input. Fix: ring-buffer of (t_mono, ψ), interpolate at meas.t_mono — code + unit test sim-free; benefit claim sim-gated.
- **M7 — slant vs depth range convention:** AprilTag emits ‖pose_t‖ (slant), markerless emits fx·span/px (depth) — up to ~36% under-read at this camera's ~100° FOV edge, ~25% at the top-edge sliver where dash detections actually arrive (ADR-0060), biasing HANDOFF_RANGE_M and the range-filter seed. Verifier correction: the live cross-convention seam is **warm-handoff seeding** (cue horizontal-world range → corrected by markerless depth), not the ADR-0056 reject gate (single-seeker flights are self-consistent). One-line fix in `_geometry()` + unit test — genuinely sim-free.
- **Low:** ENU→NED mapping hard-assumes zero translation (spawn at world origin) — true today, disclosed in the docstring, but silently breaks the parked M-1..M-4 deployment arc; add a startup guard + one docstring line. Vertical channel is fully open-loop (correct while the mover holds z=0.5 m) — document the envelope boundary + a scoring-side tripwire. `cx, cy, cz` unpack shadowing intrinsics names — mechanical rename.

### Commits/docs mediums (all sim-free unless noted)
- Hero-demo CSV (0.632 m) still cited plainly at README:727 but absent from disk and git (downgraded to medium — secondary figure, ADR-of-record fallback exists). Note: the committed .gitignore exception list *omits* it, so even a successful recovery needs a second .gitignore edit.
- Committed docs cite ADR-0059/0060 that don't exist at HEAD; the blanket decisions.md hold now also traps ADR-0061 (no jam dependency) — **builder decision: split the commit or accept the dangling window.** Related: committed HELD boxes cite the held/untracked `mc_jam_arm.sh` by path.
- pk_vs_radius_note/README still say the evidence CSVs are "gitignored, not committed" — false since 2665bbb; project under-claims its own reproducibility. README also claims plots/ is checked in while the Pk/miss plots are ignored.
- Deep-audit item 2 residuals: two dead CSV cites at README:52-53; M4 glob misses 3 of 6 stamps; **the ADR-0009 selection disclosure (published 0.402/0.277/0.443 are the 3 tightest of 8 pronav runs; the other runs — 1.04–1.12 — fail the gate) is still recorded nowhere** — the sharpest honesty-adjacent item in this group; no pitch-probe summary CSV.
- Portfolio residuals: the 30-sec pitch juxtaposes "not a mock" with the 14/14 that flew the *mock* cue; jink 1/8→14/15 still unlogged in decisions.md and uncaveated; n=96 bullet never names the AprilTag seeker; the fusion capstone (the one clean p≈0.008 win) still absent — the agreed under-claim.
- ADR-0061 invalidates committed "retrain in progress / demo flies the best detector" surfaces (README:617/:735, storyboard, PROGRESS:19) — best detector is now known to be v2; the pre-registered-scorer-prevented-a-false-win story is a portfolio *positive* worth a bullet.
- Breakoff deadband/border-reject + frozen_vworld fallback: correctly deferred (held files mid-jam-validation) but the sequencing decision is recorded nowhere — one NEXT.md line.
- **Verifier-missed:** the `!logs/mc_t21_*.csv` exception swept in two degenerate CSVs with no marker — the documented-INVALID r1 arm (all miss=nan) and a header-only file; a fresh-clone reader can score r1 instead of r2 for the headline. Also `check_t21.sh` is referenced by zero committed docs (undiscoverable), and NEXT.md's hold ledger no longer matches the hold contents (ADR-0061 unlisted).

---

## 4. Commit-soundness verdict

**All four commits (309c24e, 1a14869, 7dad15e, 2665bbb) are SOUND — independently reproduced.** No held file leaked into any of them; check_t21.sh exits 0 with all 12 pins; mc_final_all.csv recomputes the pooled Pk curve exactly (9/51/76/88/96, mean 1.084, median 0.929); all six M4 gate CSVs byte-exact; M3 consistent with the published metric definition; pytest 86 passed / 2 skipped. The doc-honesty edits landed as claimed. Two bookkeeping corrections: 35 committed log CSVs (~574 KB), not "36 / 628 KB", and see the degenerate-CSV sweep + stale "not committed" doc claims above — residue *around* the commits, not in them.

---

## 5. Completeness — still unaudited or unfixed

**Never audited (new gaps the verifiers surfaced):**
1. **Wall-clock vs sim-time in the guidance measurement chain** — the filter/pro-nav loop runs on `time.monotonic` with fixed dt (m4_intercept.py:1907, :1681, :2865) while target motion and PX4 physics evolve in sim time; under RTF sag, effective pro-nav gain scales ~RTF². ADR-0009 fixed movers/holds but never this channel. Currently masked by the idle-load rule; **never quantified** — the biggest unexamined item on the board.
2. **Ground-station rig-extrinsics→world frame chain** — independently unverified; its correctness rests on check_t19 assertion 4 at one hand-matched geometry (n=1).
3. **Desk-experiment probes** (blur_replay.py, dash_pitch_probe.py) — single-run, zero-test scripts behind committed claims (blur band, ~35% dash ticks above FoV) feeding the #35 up-tilt decision; a pitch-sign error would invert the ADR-0060 story. One verifier pass + known-answer tests before #35 consumes it.
4. **Camera lever arm** (~0.25 m mount offset) never composed into the PIP track — ~0.12–0.20 m horizontal, attitude-correlated at dash pitch; same fix as H1.
5. Replay frames contain a parked interceptor → the two-drone confusion mode (ADR-0047) is structurally untestable in replay — add to the T23 gap audit.

**Known but still unfixed after tonight:** test wiring (no run_tests.sh, onnx pair never runs, untested pure helpers, degenerate-only geometry tests); deep-audit item 2 residuals (above); breakoff hardening (sequenced, needs the NEXT.md line); audit docs + night's scripts untracked; yolo11n.pt / scripts/experiments/logs/ gitignore holes.

---

## 6. Overnight fix order (sim-free first)

**Sim-free, tonight:**
1. **Commit the pre-registration package** — eval_seeker_v3.py + phase4_eval_v3.sh + check_jam_mc.py, daemon timestamps in the message. *Must land before the jam arm flies.* (H3)
2. **Commit the 16 per-tick r2 CSVs + both audit docs**; mark the two degenerate swept-in CSVs (logs README or check_t21 note); add check_t21.sh to README; .gitignore: hero-CSV exception, `/yolo11n.pt`, `scripts/experiments/logs/`. (H4 + residue)
3. **Skew/latency decomposition on the existing T19 flight CSVs** (along-track error vs ext_age_s — pure log math, resolves M1 and bounds H2's skew); instrument the mover to print absolute sim_t0; gate hardening: latency-floor grep, min cue-count, station exit-status, fix the stale header. (H2 sim-free half)
4. **Stereo replay integrity:** trajectory-match + cache-provenance checks in m4's stereo branch; pass `--dash-direction` derived from sign(vy); station hard-fail on absent direction; the three geometric gates in triangulate + offline regression on the committed capture. (M2, M3)
5. **Markerless slant-range one-liner + off-axis unit test.** (M7 — convention fix is sim-free; A/B re-run queued)
6. **Doc currency sweep, one pass:** ADR-0061 surfaces (v2 stays deployed, false-win-prevented as a positive); "not committed" claims; dead cites + M4 glob; portfolio residuals (mock-cue scope on the 14/14, jink caveat, AprilTag on n=96, fusion bullet); wording fixes (cue-angle claim, range_filter comment, ADR-0053 citation → real exponent 1.63); NEXT.md hold-ledger refresh + breakoff sequencing line; **surface the decisions.md hold-split question for Emerson** (ADR-0060/0061 + ADR-0059-finding could commit now — his call).
7. **Test wiring:** run_tests.sh (main venv + .venv-seeker onnx pair), COAST_STALE_S pin + nonzero-angle _cam_implied_ne in a NEW test file, pure-helper tests, M3/M4 evidence pins.
8. **Offline EKF replay** of flight 33327 with markerless-tier R (1.5°/22%) — sim-free evidence for whether M5 explains the gate-rejection quirk.

**Sim-gated queue (sequence with the jam MC; Emerson's call on priority):**
- **Quaternion derotation of the bearing** (H1 — also covers lever arm + Euler-at-tilt) + ψ(t_mono) time-alignment (M6): one attitude fix, bench extension, then paired-seed A/B on the weave/jink arms. This is the direct attack on the pre-T25 maneuvering directive.
- `--epoch-t0` shared-epoch co-start for station + mover, one gated re-fly. (H2 proper fix)
- FusedTrack/EKF noise-constant A/Bs (M4/M5), nominal + WORST arms, only after the ADR addendum documents what the current numbers were measured under.
- Quantify the wall-vs-sim-clock gain distortion (§5.1) — at minimum a logged-RTF sensitivity pass before any loaded-machine batch is ever trusted.