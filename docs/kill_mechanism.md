# Kill mechanism & honest success metric — what actually, cheaply kills a small FPV drone

**Purpose.** ADR-0014 established that our headline metric is a **Pk-vs-lethal-radius
curve**, not a single number, and that any lethal radius is a *disclosed narrative
assumption* (the sim target is a flat AprilTag board with no airframe collision
volume — nothing to physically hit). This lane asks the next question: **what kill
mechanism is cheapest to actually field on a 2.5–7 in FPV, what lethal radius does
its PHYSICS give, and does that reframe make our CURRENT miss distribution already
"good enough"?**

**Standing rules applied here:** cost-first; three tiers (BEST / EXPECTED /
WORST-CREDIBLE) where numbers vary; pessimistic on conflict reporting; every external
number cited to a URL; marketing flagged. **Honesty boundary (ADR-0014):** we do NOT
pick a radius to clear a Pk threshold. We let each mechanism's physics set the radius,
overlay it on the real miss data, and report where the curve lands — even when that is
"not good enough."

---

## 0. Our real miss distribution (the thing every radius gets overlaid on)

All internal, traced to logs / ADRs. No external source.

| Regime | Miss (closest approach) | Source |
|---|---|---|
| **M4 pro-nav, 2.0 m/s crossing** | 0.28 / 0.40 / 0.44 m (mean ~0.37 m) | docs/progress.md M4; `logs/m4_intercept_*_20260705T03*.csv` |
| M4 pursuit, 2.0 m/s | ~2.0–2.5 m | docs/progress.md M4 |
| S1 pure-PN, 3 m/s crosser, hover start | 0.94 m | docs/next.md S1; ADR-0011 |
| **S2 / FPV, 6 m/s crosser (N=20 MC)** | mean **2.19 m**, median 2.30, std 0.61, min **0.95**, max 3.43, p90 2.75, p95 2.91 | ADR-0014 addendum; `logs/mc_batch_20260705T225008Z.csv` |
| Realistic-cue lab batches (pooled) | mean ~**1.4–1.8 m**, Pk@2 m ~50–75% | ADR-0015; `logs/guidance_lab_adr0015_*.csv` |

**The measured 6 m/s Pk-vs-R curve (N=20, Wilson 95% CI), from ADR-0014 addendum:**

| R_lethal | 0.5 m | 1.0 m | 1.5 m | 2.0 m | 2.5 m | 3.0 m |
|---|---|---|---|---|---|---|
| **Pk** | ~0% | **5%** [1,24] | 15% [5,36] | 35% [18,57] | 70% [48,86] | 95% [76,99] |

Read that table as the spine of this whole document: **the kill mechanism you assume
IS the vertical line you drop on this curve.** A hit-to-kill mechanism drops the line
at 0.5 m (Pk ~0% at 6 m/s); a forgiving mechanism slides it right.

The universal 6 m/s failure mode was **terminal camera dropout inside ~1 m of CPA**
(20/20 flights, ADR-0014) — a *perception* limit, not a guidance-math limit. Hold
that thought; it comes back in the reframe, because a proximity fuze has the *same*
sensing problem.

---

## 1. Kinetic body-slam (ram / hit-to-kill, $0 payload) — the cheapest possible

**How it kills.** Two small multirotors collide; the interceptor's spinning props
chew into the target's props / arms / motor, or the prop-wash upset flips a
lightweight target. Real cheap interceptors do exactly this: the US Army **Bumblebee
V2** "physically intercepts... through drone-on-drone collision... rendering both
aircraft inoperable" (a $5.2 M award to Perennial Autonomy) [armyrecognition/embention
context]; the **Flying Sword** adds a *multi-bladed point* purely to "increase contact
surface area compared to a ram-style impact" and is pitched as something "any FPV
hobbyist could copy," explicitly non-explosive for low collateral + no need for
munitions [forbes]. In Jan 2026 engagements interceptor drones (mostly kinetic)
accounted for **~70% of drone kills** [armyrecognition].

**Lethal radius — set by the two drones' physical span, not an explosive.** A 7 in
quad's prop-tip-to-prop-tip footprint is ~0.40–0.45 m across (≈0.20–0.23 m half-span);
a small FPV target is similar. Disc overlap (a prop can strike something) begins when
the centroids pass within the *sum* of the half-spans, ≈0.40 m. But a glancing
edge-pass often doesn't disable — reliable prop-on-prop / prop-on-motor contact needs
real overlap. This is exactly ADR-0014's "0.5 m aggressive kinetic point
(two-small-quad body/prop half-span)."

- **BEST** ~0.5 m (full prop extent + prop-wash upset of a light target) — NOTE this
  BEST was a 7-inch-airframe figure; see the ratification below.
- **EXPECTED** ~0.30–0.35 m (disc-overlap, a prop reliably catches the target)
- **WORST-CREDIBLE** ~0.15 m (need a near-central strike; glancing passes miss). Note
  the FPV literature stresses the ram is the *least forgiving* option — attackers had
  to descend precisely onto the target's rotors, "sometimes resulting in mutual
  destruction" [forbes/webslingers].

> **⚖️ RATIFIED 2026-07-25 (ADR-0084) — the go-forward ram radius is 0.35 m, from the
> ORDERED 5-inch pair.** The BEST ~0.5 m above was the 7-inch heritage number (two 7"
> aircraft = 2 × 238.9 mm half-span = 0.48 m). Both ordered aircraft are 5-inch:
> interceptor Mark5 Pro half-span (226 mm wheelbase + 129.5 mm 5.1" prop)/2 = 177.8 mm;
> target Source One V6 half-span = 174.8 mm; **contact envelope = sum = 352.5 mm → 0.35 m**
> (rounded down). This lands exactly on the old "EXPECTED" tier — i.e. for the 5-inch
> pair the disc-overlap figure IS the whole-envelope figure. `field_score.py`
> `DEFAULT_LETHAL_RADIUS_M` is 0.35 m. The only physical way to widen it is a bigger
> airframe (a 7" would restore ~0.48 m, heavier/slower — a Tier-2 airframe-class call,
> NOT payload-driven: the 5-inch carries the full seeker with T/W ~6:1, ADR-0084).

**Cost $0 payload. Weight $0. Complexity: LOW payload, HIGHEST guidance burden** (all
the accuracy must come from the guidance loop — the mechanism forgives nothing).

**Implied Pk on our data:** 2 m/s regime (miss ~0.37 m) → **~95–100%** (a solved
problem). 6 m/s regime (miss ~2.2 m) → **~5%**. Realistic-cue (~1.4–1.8 m) → **~0–5%**.

**Verdict:** The correct *baseline / honest headline* mechanism for a cheap FPV, and
the one our slow-regime numbers already ace — but it is precisely the mechanism that
makes the fast regime look like a failure.

---

## 2. Net capture — forgiving radius, but heavy / one-shot / short-range

**How it kills.** A launched net expands to a wide square and snags the target's
rotors; the target drops or descends under a small parachute. Forgiveness is the whole
point: a net "offers a forgiving alternative to [the] precise strike requirement" of a
ram [forbes/webslingers].

**Two tiers of net exist:**

*(a) Cheap FPV-borne net* (directly portable to our platform). The Ukrainian
net-drone's net is **3 m × 3 m (10 ft square)** and "anything inside the 10-foot
reach... is likely to be brought down, if it has rotors that can be snagged"; the
launcher weighs **373 g**, effective range **up to ~10 m**, and the downed target is
often recovered/reused [forbes/webslingers]. Ukrainian net guns countering fiber-optic
FPVs expand to **3.5–4 m**, down drones within **~30 m**, and cost **< $200** [ts2.tech].

*(b) Purpose-built interceptor* — Fortem **DroneHunter F700**: radar-guided,
tow-net, **>4,500 captures**, only **~15% of targets evade the first shot** (~85%
first-shot), reload < 3 min [fortemtech — *vendor marketing, flag*]. No public
weight/price; it is a heavy dedicated airframe, not a 2.5 in payload.

**Lethal (capture) radius — half the net span:**

- **BEST** ~2.0 m (4 m net, target squarely in the sheet)
- **EXPECTED** ~1.5 m (3 m net, half-span)
- **WORST-CREDIBLE** ~0.8–1.0 m (target near net edge, or a rotor that doesn't snag;
  recoil "affects accuracy" [forbes])

**Cost ~$100–300 for the net payload. Weight ~370 g+ → forces a 7 in-class airframe**
(a 2.5 in can't lift it — see §7). **Complexity: MEDIUM** (launcher + one-shot,
short-range, aim into a 3–4 m basket).

**Implied Pk on our data (R≈1.5 m):** 2 m/s → ~100%; 6 m/s → **~15%** (rises to ~35%
with a 4 m net at R≈2.0); realistic-cue (~1.4–1.8 m) → **~40–55%** (→ ~50–75% at R≈2.0).

**Verdict:** The **cost-optimal *forgiveness* upgrade** — it roughly doubles the
tolerable miss (0.5 m ram → 1.5–2 m) with **no explosive** and no proximity-fuze
sensing problem (the net is a physical volume, it doesn't have to *detect* the target
at 1.5 m the way a fuze must). Best single non-explosive lever for the fast regime.

---

## 3. Small fragmentation / proximity charge — biggest radius, but wrong for this build

**⚠ Appropriateness first.** This is a **simulation portfolio piece**. Everything
below is *design-space analysis of published weapon data*, not a build recipe. A real
explosive charge on a hobby airframe is unsafe, likely illegal for a civilian to build,
and out of scope. It is included only because the reframe question demands we know
what radius a charge *would* buy — and to show why we still don't recommend it.

**How it kills.** A small charge detonates near the target; fragments cut a rotor,
motor, ESC, or battery. FPV interceptors already do this: Wild Hornets **Sting**
(~$2,500) "fire[s] a fragmentation charge into the engine or propeller," 80–90% hit
rate [thedefender/wildhornets — *vendor*]; "many interceptors carry a small
fragmentation charge that detonates near the target, significantly increasing the
chances... even if the final navigation is not perfect" [ynetnews].

**Lethal radius vs charge mass (published anchors — but read the caveat):**

- Anti-personnel: M67 grenade, **184 g** Comp B → **5 m** lethal / 15 m casualty
  [pica.army.mil / fas.org]; **70 g** TNT → ~7 m; **55 g** Comp B → ~20 m *casualty*
  radius [grokipedia/patent survey — treat as upper bound].
- Counter-UAS airburst: BarB-X loitering munition, **600 g** payload, proximity fuse →
  **~5 m** effective spread [bluebird-uav — *vendor*]; steel-case frag ~5–8 m
  [armyrecognition].

**The critical correction:** those radii are against a **soft human body**. A drone is
a *small, sparse* target — fragments must strike a *vital* part (motor/ESC/battery/
prop) and most fragments pass through empty air between the arms. So the *reliable
drone-kill* radius is **well inside** the anti-personnel number for the same charge,
even though the drone is fragile (one fragment through a motor = kill). Pessimistic
reading:

- **BEST** ~3–4 m (a few-hundred-gram charge; any fragment strike downs a fragile quad)
- **EXPECTED** ~1.5–2.5 m for a **tens-of-grams** FPV-appropriate charge (needs a
  vital-part hit)
- **WORST-CREDIBLE** ~0.8–1.2 m (small charge, small sparse target, fragments miss the
  vitals)

**Cost:** the charge itself is cheap (tens of $), but **the fuze is the expensive,
hard part** — see below. **Weight ~50–300 g. Complexity: HIGH + inappropriate to build.**

**The fuze is the same perception problem we already have.** A proximity fuze that
airbursts at R = 2 m must **detect the target at 2 m** (optical / laser / RF). That is
*exactly* the terminal detection ADR-0014 found us losing inside ~1 m of CPA. A frag
radius does **not** come for free — you have to buy the sensing you don't have. This is
the single most important reason a "just assume a 3 m frag radius" reframe is dishonest
for our system: **the radius is gated by a fuze we haven't demonstrated.**

**Implied Pk on our data (R≈2.5 m, EXPECTED–BEST, IF the fuze worked):** 6 m/s →
**~70%**; realistic-cue → ~90%+. **(R≈1.5 m, WORST-realistic small charge):** 6 m/s →
~15%. **Verdict:** physically the most forgiving, but **rejected for this project** on
appropriateness + the fuze-is-the-same-perception-gap grounds. Its only honest role
here is as the *upper bound* of the reframe curve.

---

## 4. Entanglement / streamers / drag-lines — cheap soft-kill, small radius

**How it kills.** Trailing streamers, threads, or a drag-line fouls the target's
rotors. Effectors are "engineered to consistently interrupt propeller thrust *if
delivered to a location where entanglement with a propeller is likely*" [droneshield /
soft-kill survey]. The frailty of the target helps: "to bring a quadcopter down, all
you need to do is break a rotor blade, and a fishing rod with a cord can do the job"
[ynetnews].

**Lethal radius — small, and directional.** The streamer must physically overlap the
rotor plane, usually from above/across. There is no wide "basket."

- **BEST** ~0.8 m (long deployed streamer, good geometry)
- **EXPECTED** ~0.3–0.5 m
- **WORST-CREDIBLE** ~0.1–0.2 m (must thread the rotor; low reliability)

**Cost ~$0–20. Weight ~20–100 g. Complexity: LOW hardware, but LOW reliability**
(hardest to make repeatable; poor against a maneuvering target).

**Implied Pk on our data:** essentially a ram-class radius (0.3–0.5 m) → clears the
2 m/s regime, **fails** 6 m/s (~5%). **Verdict:** a near-free curiosity; **no radius
advantage over the ram** to justify the reliability hit. Not recommended as primary.

---

## 5. THE REFRAME — overlay each radius on our ACTUAL miss, honestly

### Master table (radius set by physics, Pk read off our own curves)

*The "Pk @ realistic-cue (~1.4–1.8 m)" column is a LAB (`guidance_lab.py --adr0015`)
estimate, NOT a Gazebo Monte-Carlo — see the audit note on the net recommendation (§7).
The 2 m/s and 6 m/s columns trace to real Gazebo runs (M4 gate, ADR-0014 addendum batch).*

| Mechanism | Lethal R (BEST/EXP/WORST) | Payload cost | Weight | Pk @ 2 m/s (miss ~0.37 m) | Pk @ 6 m/s (miss ~2.2 m) | Pk @ realistic-cue (~1.4–1.8 m, LAB) | Verdict |
|---|---|---|---|---|---|---|---|
| **Kinetic ram** | 0.5 / 0.30 / 0.15 m | **$0** | **0 g** | **~95–100%** | ~5% | ~0–5% | Honest baseline; wins slow regime free |
| **Net (3–4 m)** | 2.0 / 1.5 / 0.9 m | ~$100–300 | ~370 g | ~100% | ~15–35% | ~40–75% | Best cheap *forgiveness* lever, no explosive |
| **Frag charge (small)** | 3–4 / 1.5–2.5 / 0.9 m | charge cheap; **fuze $$$** | 50–300 g | ~100% | ~70% (IF fuze works) | ~90% | Physically best; **rejected** (appropriateness + fuze = same perception gap) |
| **Streamer / entangle** | 0.8 / 0.4 / 0.15 m | ~$0–20 | 20–100 g | ~90–100% | ~5% | ~0–10% | No radius edge over ram; low reliability |

### The reframe, stated plainly

1. **The slow/mid regime is already solved under the STRICTEST mechanism.** M4 pro-nav
   at 2 m/s misses by ~0.37 m — inside even the WORST-credible ram radius. Under a
   **$0, 0 g kinetic ram**, that intercept is a **kill with Pk ~95–100%**. No net, no
   charge, no reframe needed. **Yes — at 2–3 m/s our current system already clears a
   defensible Pk under the cheapest possible mechanism.**

2. **The "sub-meter miss problem" is partly a mechanism-choice artifact.** docs/goals.md's
   "< 1 m" bar and ADR-0014's R = 0.5–1.0 m headline encode a **hit-to-kill** standard.
   That standard is only *required* by the ram — the one mechanism cheap/light enough
   for a 2.5 in and, not coincidentally, the *least forgiving*. Judged against a net
   (R ≈ 1.5 m) or a small frag charge (R ≈ 2–2.5 m), a 1–2 m miss is a **kill**, and
   the 6 m/s miss (mean 2.2 m) slides from Pk ~5% toward 35–70%. Real cheap
   interceptors know this — they **add a forgiving mechanism precisely because
   hit-to-kill on a small maneuvering quad is unreliable** [ynetnews: charge "even if
   the final navigation is not perfect"; forbes: net "forgiving alternative"].

3. **But the reframe does NOT hand us the fast regime for free — and here is where it
   reconciles with ADR-0014's honesty boundary.** The mechanisms that forgive 2–3 m
   are not free lunches:
   - **Frag radius is gated by a proximity fuze that must sense the target at 2–3 m —
     the exact detection we lose inside 1 m of CPA (ADR-0014's 20/20 dropout).** You
     cannot "assume R = 3 m" without assuming you solved the perception gap. The
     perception limiter doesn't vanish; it **moves from the guidance loop to the fuze.**
   - **Net radius is real and sensor-free** (a physical 3–4 m sheet), but it costs
     ~370 g and forces a 7 in airframe, is one-shot, and still needs you within
     ~1.5 m — at 6 m/s our p50 miss (2.3 m) is *outside* a 3 m net's half-span.
   - So the honest statement is: **do not invent a radius to clear 95%.** Publish the
     curve, name the mechanism next to it, and disclose that the flat-board sim has no
     collision volume so *every* radius here is a narrative assumption (ADR-0014).

4. **The single biggest reframe insight.** *You cannot buy your way out of the terminal
   perception problem with a bigger lethal radius.* The physical mechanisms that need
   **no** terminal sensing (ram, net) forgive only ~0.5–2 m — and our slow regime
   already clears the ram while our fast regime clears a net only ~15–35% of the time.
   The mechanism that *would* forgive our 6 m/s miss (a proximity-fuzed charge) needs
   to **detect the target at that same radius**, which is the very capability ADR-0014
   identified as the limiter. **The reframe converts "the guidance misses by 2 m" into
   "the FUZE/net must tolerate 2 m," and only the ram/net options do that without
   re-opening the perception gap** — which is why fixing the terminal detection (the
   perception-half redirect, ADR-0015) remains the real lever, not a bigger warhead.

---

## 6. Second-order economics (the questions §1–5 don't cover)

- **Payload-mass <-> guidance-accuracy tradeoff (self-defeating warheads).** A heavier
  forgiving payload (net 370 g, charge 50–300 g) *lowers* closing speed and agility,
  and can **worsen** the miss it was meant to forgive. S1 already found hover-start is
  **kinematically speed-limited** (~3 m/s cap); adding mass tightens that. Net: mass
  buys forgiveness and *spends* accuracy — they partly cancel. Cheapest net win is to
  keep the interceptor light (ram) *and* fast, and only add the minimum forgiving mass.

- **One-shot vs reusable economics.** Ram, net, and charge all consume the interceptor
  (one-flight-one-kill). That is *fine* and is the actual doctrine: Varta's shotgun
  payload runs **~$300/airframe, ~€6–7/shot**, deliberately "destroy hostile quadcopters
  cheaper than losing military equipment" [armyrecognition]. A $300–2,500 expendable
  vs a $3 M Patriot [ynetnews] is the whole cost thesis. Reusability is not the lever;
  cost-per-kill is.

- **Swarm / multi-interceptor Pk stacking — the cheapest path to a defensible number.**
  N independent shots at per-shot Pk *p* give **1 − (1 − p)^N**. On our 6 m/s data:
  - **Ram (p ≈ 5%):** N = 14 → 51%, N = 59 → 95% — impractical.
  - **Net R ≈ 2 m (p ≈ 35%):** N = 2 → 58%, N = 3 → 73%, **N = 5 → 88%, N = 7 → 95%**.
  - So **3–5 cheap net-FPVs stack to a defensible Pk** where a single hit-to-kill ram
    can't — for ~$900–1,500 total. Cost-first, *salvo forgiveness* beats chasing
    sub-meter single-shot guidance. (Caveat: assumes independent misses; correlated
    terminal-dropout would degrade the stack — another reason the perception fix is
    upstream of everything.)

---

## 7. Recommendation (cost-first)

1. **Headline mechanism = kinetic ram, R_lethal = 0.35 m (RATIFIED, ADR-0084 — supersedes this
   section's original "≈ 0.5 m (0.3 m expected)", which was the 7-inch heritage figure; see the
   ⚖️ banner in §2).** $0, 0 g,
   matches the committed 2.5–7 in airframe and docs/goals.md's "< 1 m" bar, and is what
   ~70% of real interceptor kills already do. **Report the ram Pk-vs-R curve as the
   honest headline.** Under it, **the 2–3 m/s intercept is already a validated kill
   (Pk ~95–100%)** and the 6 m/s intercept honestly is not (Pk ~5%) — publish both.

2. **Cost-optimal forgiveness upgrade = a small net (R ≈ 1.5 m, ~370 g, ~$100–200,
   7 in class).** It is the *only* cheap lever that roughly doubles tolerable miss
   **without an explosive and without needing terminal target-sensing at the kill
   radius.** It lifts the realistic-cue regime to ~40–75% *(LAB estimate, guidance_lab
   `--adr0015`, NOT Gazebo-confirmed — the one component later Gazebo-tested, velocity
   emission, came in at ~1/5 the lab effect; re-derive from the M5 Gazebo Monte-Carlo
   before headlining)* and, salvoed 3–5×, reaches a defensible Pk at 6 m/s. Recommend
   it explicitly **over** a fragmentation charge.

3. **Do NOT adopt a fragmentation/proximity-fuze radius to rescue the number.** It is
   inappropriate to build, and its radius is gated by the same terminal detection we
   lack — assuming it launders the perception gap. Keep it in the doc only as the
   *upper bound* of the design-space curve.

4. **The real lever stays perception (ADR-0014 / ADR-0015).** The reframe legitimately
   banks the slow regime and softens the fast one, but the 6 m/s kill still comes down
   to holding terminal lock in the last ~1 m — for the guidance loop *and* for any
   fuze. Fix that and every mechanism's Pk rises together.

---

## ADR-lite — kill mechanism & the honest metric

- **Context:** Terminal miss floors at ~1–2 m at FPV speed (6 m/s mean 2.19 m) and
  won't null. Real-missile answer: don't null the last meter — kill within a lethal
  radius. Which cheap mechanism, what radius, does it make the current system "enough"?
- **Options:** kinetic ram ($0, R~0.3–0.5 m) / small net (~$150, 370 g, R~1.5–2 m) /
  small frag charge (fuze-gated, R~1.5–2.5 m) / streamer (R~0.3–0.5 m).
- **Decision:** Headline the **kinetic ram (R≈0.5 m)**; recommend a **small net
  (R≈1.5 m)** as the cost-optimal, non-explosive *forgiveness* upgrade and salvo
  stacking (3–5×) for a defensible fast-regime Pk; **reject frag** for build
  (appropriateness) and reframe-rescue (fuze = the same perception gap).
- **Why:** Ram is $0 and already yields **Pk ~95–100% at 2–3 m/s** (miss ~0.37 m); at
  6 m/s only a forgiving mechanism helps, and only the net forgives 1.5–2 m without
  needing terminal sensing at the kill radius. Radii are set by physics (drone span /
  net span / grenade data), not reverse-engineered to a threshold — ADR-0014 boundary.
- **Honesty caveat:** Sim target is a flat board (no collision volume) — every radius
  is a disclosed narrative assumption. Frag's forgiving radius is gated by a proximity
  fuze that must detect the target at that radius: the exact perception limit
  (ADR-0014's 20/20 terminal dropout). The reframe moves the gap, it doesn't close it.
- **Biggest insight:** You can't buy out the terminal perception problem with a bigger
  warhead — a bigger radius needs bigger sensing. Sensor-free mechanisms (ram/net)
  forgive only ~0.5–2 m; the perception fix (ADR-0015) remains the real lever.
- **Date:** 2026-07-05. Analysis only (WebSearch/WebFetch); no sim run, no code change.

---

## Sources (external; internal numbers cite logs/ADRs inline above)

- Ukrainian interceptor scale, ram economics, ~70% of kills kinetic, Bumblebee V2 /
  Varta $300: https://www.armyrecognition.com/news/aerospace-news/2025/ukraine-debuts-modular-shotgun-drone-that-hunts-enemy-fpv-drones-at-ultra-close-range ,
  https://thedefensepost.com/2026/01/13/kinetic-c-uas-hits-battlefield/ ,
  https://www.embention.com/embention-uam-academy/lesson/counter-uas-interceptor-drones-defending-the-skies/
- Kinetic ram / Flying Sword (contact-kill, hobbyist-copyable, non-explosive rationale):
  https://www.forbes.com/sites/davidhambling/2026/04/16/fpvs-get-medieval-with-flying-sword-bladed-drone/
- Net-launching FPV (3 m net, 373 g, ~10 m, forgiving vs ram, reusable target):
  https://www.forbes.com/sites/davidhambling/2024/11/12/webslingers-how-net-launching-drones-are-downing-russian-quadcopters/
- Net guns 3.5–4 m / ~30 m / < $200 (anti-fiber-optic FPV):
  https://ts2.tech/en/unstoppable-unjammable-drones-how-fiber-optic-technology-is-revolutionizing-warfare-and-beyond/
- Fortem DroneHunter F700 (radar net-capture, 85% first-shot — *vendor marketing*):
  https://fortemtech.com/products/dronehunter-f700/ ,
  https://breakingdefense.com/2023/10/a-better-c-uas-option-capture-enemy-drones-in-a-net/
- SkyWall net launcher (>100 yd to a few ft, parachute recovery):
  https://openworksengineering.com/skywall-patrol/
- Frag charge vs mass — grenade anchors: https://www.pica.army.mil/pmccs/CombatMunitions/Grenades/LethalHand/M67Frag.html ,
  https://man.fas.org/dod-101/sys/land/m67.htm ; small-charge survey (upper bounds):
  https://grokipedia.com/page/Fragmentation_(weaponry)
- Counter-UAS airburst radii (BarB-X 600 g->~5 m; steel-case 5–8 m — *vendor*):
  https://bluebird-uav.com/barb-x-loitering-munition/ ,
  https://www.armyrecognition.com/news/aerospace-news/2026/u-s-air-force-approves-145m-dual-mode-apkws-ii-air-to-air-rocket-to-counter-drone-swarms
- FPV frag "even if navigation not perfect", cheap-quad-vs-$3M, fragility:
  https://www.ynetnews.com/tech-and-digital/article/sjqdemeqbe
- Wild Hornets Sting (~$2,500, frag into engine/prop, 80–90% — *vendor*):
  https://thedefender.media/en/2025/08/dyki-shershni-showcased-sting-315-km-god/ ,
  https://wildhornets.com/en/sting-interceptor
- Entanglement/streamer effectors + soft-kill survey:
  https://www.droneshield.com/blog/what-is-the-best-drone-defeat-technique-d6ldb-3724h ,
  https://norskluftvern.com/2026/03/23/soft-kill-non-kinetic-counter-drone-systems/
