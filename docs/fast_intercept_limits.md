# What limits faster intercepts (2026-09-23 investigation)

Builder ask: keep working toward faster intercepts by identifying the
binding constraint, and answer whether a second onboard camera with a much
narrower FoV / higher resolution would help.

Every number below traces to a run; n=1 rows are labeled leads. The isim
rows use the fitted Gazebo-x500 vehicle model and the measured OV9281
camera/blur model; isim numbers are simulation and the vehicle fit's own
validity range is a stated caveat (section 5).

## 1. The chase concept's ceiling is OVERTAKE MARGIN (config, not physics)

Port arm (flight code in the loop), rear tag, n=50/cell, canonical crossing:

| target m/s | med miss | <=0.35 m | note |
|---|---|---|---|
| 9  | 0.081 | 100% | |
| 12 | 0.093 | 100% | |
| 15 | 0.341 | 54%  | chases run 2.5x longer (median 286 decodes) |
| 18 | 5.104 | 0%   | never catches: 8 decodes median |

Native concept with the `pursuit_v_max_ms` hook raised
(`scripts/vmax_sweep_chase.py`):

| v_max | target 15 | target 18 | target 21 |
|---|---|---|---|
| 16 m/s | 0% <=0.35 (med 1.52) | 0% | 0% |
| 20 m/s | **100% (med 0.112)** | 40% | 0% |
| 24 m/s | 100% (med 0.116) | **66% (med 0.131)** | 4% |

Mechanism: the chase closes from behind at v_max − v_target. At margin
~1 m/s the catch consumes the whole window; at zero margin it never
happens. **~4–5 m/s of overtake margin buys a clean catch.** The 16 m/s
cap is a configured constant (PursuitConfig / the sim's fitted top speed),
not perception and not the guidance law. The real 5-inch airframe's top
speed is one of the register's unmeasured givens (`dash-accel-profile`) —
the first real dash ULog measures it.

## 2. The tag-scenario terminal is capped by its own COMMAND ENVELOPE

Ladder scouting (n=1 leads) + `scripts/ladder_forensics.py` on the classic
M4 config (V_CLOSE_MAX 3.0 / V_PERP_MAX 3.0 / V_TOTAL_MAX 4.0 m/s — tuned
2026-07-04 for the original 2 m/s crosser):

| target | CPA | terminal det coverage | last det before CPA | v_perp at rail |
|---|---|---|---|---|
| 2 m/s | 0.413 | 76% | 0.51 m | 65% of ENGAGE |
| 3 m/s | 0.238 | 78% | 0.73 m | 74% |
| 4 m/s | 0.961 | 94% | 0.99 m (bearing 30 deg) | 29% |
| 6 m/s | 5.291 | 31% | **5.29 m** | 87% |

The signature: `v_perp` pegs at exactly 3.00 m/s, and at 6 m/s the vehicle
is commanded SLOWER than the target in every axis (total cap 4 < 6). The
LOS then walks the tag to the frame edge, detections stop at 5.3 m range,
and the CPA is that range verbatim. **The binding constraint is the
July-era command envelope; the FoV walkout is its symptom, not a camera
deficiency.** ZEM-vs-capacity (ADR-0023/0027) does not bind here — t_go at
handoff is long.

Mechanism-confirmation scout (envelope raised to 6/8/10 m/s via the new
`--v-perp-max/--v-close-max/--v-total-max` overrides; n=1 leads).
PREDICTION, registered before the flights were read: 4 m/s drops to
~<=0.5 m; 6 m/s completes with detections through the terminal (v_perp off
the 3.0 rail, last-detection range well under 1 m) and improves markedly;
8 m/s added as an exploration point.
RESULT (n=1 leads): 4 m/s 0.961 -> **0.140 m clean** (prediction held --
the envelope WAS the binding constraint at 4). 6 m/s 5.29 -> 3.996 (improved
but still a miss) and 8 m/s 5.48: forensics on the 6 m/s flight show v_perp
now railing at the NEW 8.0 m/s cap for 73% of ENGAGE and the tag lost at
45.4 deg bearing (frame edge) at 4.0 m -- the same signature one level up.
So above ~4 m/s the STANDING-START geometry itself binds: the classic M4
scenario acquires at ~7.5 m with the vehicle near hover, and no lateral cap
raise lets it match a 6+ m/s cross-speed from a standstill in the ~1.5 s
available. This is the sim-phase running-start lesson (ADR-0036, Pk 27% ->
100%) re-derived from the other side: faster targets need an engagement
that ARRIVES with speed (the dash entry handles 9 m/s; the chase handles
12+), not a hotter standing-start terminal.

## 3. Would a second narrower-FoV / higher-resolution camera help? NO for the terminal.

`scripts/fx_sweep_chase.py` — port chase arm, single camera swept from the
real 118-deg lens toward telephoto (same 1280 px sensor; the isim seeker
charges BOTH sides: more px on the tag AND a narrower frame, with the
measured-exposure blur model):

| HFOV | tgt 9: <=0.35 m | tgt 15: <=0.35 m |
|---|---|---|
| 118 deg (flying lens) | **100%** | 54% |
| 100 deg | 100% | 54% |
| 79 deg  | 68%  | 0% |
| 58 deg  | 38%  | 0% |
| 45 deg  | 24%  | 34% (mixed failure modes) |

Narrowing the FoV makes the chase WORSE, monotonically at 9 m/s. The
terminal's need is FIELD OF VIEW at close range (the tag subtends a huge
angle in the last metres and the vehicle maneuvers); its precision need is
already met — the KF closes to ~0.08 m on the wide lens, far inside the
0.35 m bar, so a sharper bearing buys nothing. This re-confirms the
dash-era narrow-lens rejection (ADR-0024, graveyard) in the chase regime
with fresh data. Since the SINGLE-narrow-camera sweep is an upper bound on
what a second narrow camera adds as a terminal lever (a real dual-camera
system still pays acquisition + cross-camera handoff + a second CSI/compute
stream on the measured Pi 5 budget), the answer to the terminal question is
no.

Where a long lens COULD still earn a place, honestly scoped: long-range
ACQUISITION/verification before launch — the launcher-side one-shot
spotter concept already evaluated in `docs/launch_mechanism_options.md`
(kept as the honesty-clean recast of option C), and the real-world decode
range that the tripod day measures. Neither touches the terminal miss, and
the in-flight acquisition problem the project actually measured (the
pointing wall) was about WHERE the camera looks, not how many pixels it
has (`docs/inview_probe_results.md`; foveated-crop/resolution rejected,
ADR-0076 add #18j-fix).

## 4. So the speed roadmap, in order of leverage

1. **Raise the vehicle's speed envelope** — both the chase v_max and the
   tag-terminal command envelope are configured constants sized for slower
   regimes. In sim this is a config sweep (done above); on hardware it is
   the first real dash ULog (`dash-accel-profile` register entry) plus a
   thrust/drag margin question, not a purchase.
2. **Close the Gazebo transfer gap** (median 1.51 m vs 0.122 predicted,
   registered FAIL 2026-09-23) — until the chase transfers to real
   physics, its isim speed numbers are upper bounds. Tick-trace diagnosis
   is the named next step (`docs/xcheck_gazebo_pursuit_prereg.md`).
3. **Engagement geometry** — a head-on or crossing chase entry converts
   target speed from an overtake problem into a closing-speed bonus; the
   isim scenario hooks (`head_on`, crossing geometry) can price this
   cheaply before any design change.
4. NOT a camera purchase: the narrow/high-res second camera does not move
   the terminal (section 3).

## 5. Caveats

- All isim; no wind, error-free launch cue, the register's given-perfect
  inputs apply. Gazebo cross-check currently FAILS its bar — see item 2.
- The fitted vehicle model was fit to flights that never exceeded ~16-18
  m/s commanded; v_max 20-24 rows extrapolate the fitted accel/drag beyond
  their data. Direction is trustworthy (margin helps), the specific
  percentages are not hardware claims.
- Ladder rows are n=1 scouting leads; no claim beyond mechanism
  identification may quote them without a paired batch.
