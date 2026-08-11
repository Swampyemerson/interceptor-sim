# Regulatory + site capture — **CLOSED BY BUILDER RULING, 2026-08-10**

> ## RULING — this item is settled and is no longer a blocker
>
> **Builder, 2026-08-10, verbatim intent:** *"waive regulatory side issues, trust
> me it's sorted and legal — I own the land (50 acres) and everything."*
>
> **What that means for this project, operationally:**
>
> | | |
> |---|---|
> | **Site** | Private land the builder **owns**, ~**50 acres** — his own property, not a club field or a public site |
> | **Regulatory basis** | Settled by the builder. He has confirmed it directly and it is **his call and his responsibility**, not the head's |
> | **Two-aircraft mid-air** | Covered by the same ruling — it happens on his own land |
> | **Status** | **CLOSED.** Not a gate on tripod day, kill day, or the ~$740 order |
>
> **This ruling supersedes** the blank template below, GitHub issue #4, and the
> contract queue item `regulatory-site-capture`. `docs/tripod_test_protocol.md`
> §1/§9 and `docs/kill_day_protocol.md` §1.4/§9 no longer treat the regulatory
> line as UNVERIFIED — it is verified by builder attestation, recorded here.
>
> **The three contradicting contract lines** the 2026-07-25 review found (P0 task
> 7 "find a FRIA field", the hardware-stage "IN HAND" note, and the BOM-layer
> "STILL OPEN: the field choice") are resolved by this ruling in favour of:
> **owned private land, in hand, closed.** A FRIA field is not needed — that
> option existed only because the site was assumed to be public.
>
> **Scope of what is recorded here, stated plainly and once:** this is the
> builder's attestation, written down so the project stops re-asking. It is not
> a legal opinion and the head did not verify it independently, which is exactly
> right — it is his land and his call. The blanks below stay available if he ever
> wants the specifics on paper (an insurer or a visiting pilot might ask), but
> **filling them in is optional and blocks nothing.**

---

<details>
<summary><strong>Optional detail — the original blank template (no longer required)</strong></summary>

---

## 1. Operating regime

Which basis are you flying under? (Tick one per aircraft — they can differ, and
the answer can differ between the tripod day and the kill day.)

- [ ] **Recreational / limited exception for recreational flyers**
      — TRUST certificate number + date: `________________________`
- [ ] **Part 107 (remote pilot certificate)**
      — certificate number + expiry: `________________________`
- [ ] **Other / additional authorisation** (waiver, COA, club rule, private-land
      arrangement): `________________________________________________`

Community-based organisation / club (if the flight relies on one):
`____________________________________________`
CBO safety guidelines the flight is conducted under: `____________________`

Anything about the flight profile that the chosen regime constrains (altitude,
airspace class, over-people, beyond-visual-line-of-sight, night):
```
____________________________________________________________________
____________________________________________________________________
```

---

## 2. Aircraft registration — BOTH aircraft are >250 g

| | Interceptor (PX4 / 5-inch) | Target (ArduPilot / 5-inch) |
|---|---|---|
| Take-off mass (g), as flown | `______` | `______` |
| Registration required? (>250 g) | Y / N | Y / N |
| Registration number | `______________` | `______________` |
| Registered under | recreational / Part 107 | recreational / Part 107 |
| Expiry date | `__________` | `__________` |
| **Number physically displayed on the airframe?** | Y / N | Y / N |
| Re-marked after a rebuild/crash? | Y / N | Y / N |

> Note for the kill day: the interceptor's nose/camera boom is **sacrificial** —
> confirm the registration marking is not on a part that gets destroyed, and
> re-mark after every rebuild.

---

## 3. Remote ID compliance path — **one row per aircraft, both must be answered**

| | Interceptor | Target |
|---|---|---|
| Standard Remote ID built in? | Y / N | Y / N |
| Broadcast module fitted? (make/model/serial) | `____________` | `____________` |
| Or: flown at a **FRIA**? | Y / N | Y / N |
| FRIA name + FAA recognition reference | `____________` | `____________` |
| FRIA sponsoring organisation | `____________` | `____________` |
| **How compliance is demonstrated on the day** (module powered + confirmed, or FRIA boundary respected) | `____________` | `____________` |

This is the field that the contract currently leaves open ("FRIA field vs a
broadcast module"). Answer it **per aircraft** — a home-built target and a
home-built interceptor are two separate compliance problems, and a FRIA answer
only holds while you are inside that FRIA's boundary.

---

## 4. Site(s) — the tripod venue and the kill venue may differ

### 4.1 Tripod / capture day site (build_plan P2)

```
Site name:            _______________________________________________
Address / access:     _______________________________________________
Coordinates (lat, lon, decimal deg): __________ , __________
Landowner / manager:  ______________________  contact: ______________
Permission: verbal / WRITTEN / public land / club field     dated: ________
Airspace: class ______  nearest airport/heliport: ______  distance: ____ km
LAANC / authorisation needed?  Y / N     reference: ______________
Altitude ceiling flown (m AGL): ______   ceiling ALLOWED here (m AGL): ______
Known constraints (people, roads, livestock, structures, noise hours):
____________________________________________________________________
```

### 4.2 First-kill day site (build_plan P5) — **the two-aircraft mid-air**

```
Same as 4.1?  Y / N       If N:
Site name:            _______________________________________________
Address / access:     _______________________________________________
Coordinates (lat, lon, decimal deg): __________ , __________
Landowner / manager:  ______________________  contact: ______________
Permission: verbal / WRITTEN                          dated: ________
Airspace: class ______  nearest airport/heliport: ______  distance: ____ km
LAANC / authorisation needed?  Y / N     reference: ______________
Altitude ceiling flown (m AGL): ______   ceiling ALLOWED here (m AGL): ______
Dash corridor length needed (m): ______   Engagement box radius (m): ______
  (from docs/kill_day_protocol.md §5.1 — the corridor is the FULL dash bound,
   not the distance to the intercept point)
Is that corridor + box entirely inside the permitted area?   Y / N
Recovery area for two falling damaged aircraft — clear of people/road/water/dry
  fuel?  Y / N       described: ______________________________________
```

---

## 5. Written permission for a DELIBERATE two-aircraft mid-air

> This is the one the tripod day never needed and the kill day cannot skip. A site
> that is fine for flying two drones is not automatically a site where you may fly
> them **into each other on purpose** and let the wreckage fall.

```
Does the site owner / club / airfield manager KNOW the flight involves
  intentional aircraft-to-aircraft contact?              Y / N
Is that permission IN WRITING (email is fine)?           Y / N
  Where is it filed / who from / dated: ______________________________
Any conditions attached (times, area, spectators, notification, insurance):
____________________________________________________________________
____________________________________________________________________
Insurance in force for this flight profile? (club/CBO/other)   Y / N
  Policy / membership reference: ____________________________________
Does the CBO safety code or club rule permit deliberate contact?  Y / N / N-A
  If NO or unclear, what is the alternative site or authorisation:
____________________________________________________________________
Who else must be notified on the day (manager, neighbours, other flyers):
____________________________________________________________________
```

---

## 6. On-the-day compliance card (copy into the flight card, §11 of each protocol)

```
[ ] Registration numbers present and legible on BOTH aircraft
[ ] Remote ID path ACTIVE for BOTH aircraft (module powered + verified, or inside
    the FRIA boundary and it was confirmed today)
[ ] Flying under the regime in §1, and the flight profile stays inside it
[ ] Altitude ceiling briefed and respected
[ ] Site permission current; two-aircraft-contact permission current (kill day)
[ ] Any required notification made
[ ] Visual line of sight maintained on both aircraft, spotter briefed
```

---

## 7. Handoff back to the contract — what the head does with this

Once this file is filled in:

1. The head reconciles `docs/project_state.json` — the `build_plan` P0 task 7 line,
   the `hardware` stage note ("FAA regime + site IN HAND"), and the field-layer BOM
   note ("STILL OPEN: the field choice") — so all three agree with **this file**,
   and each points here instead of restating the facts.
2. If the three cannot be reconciled (e.g. registration is done but the Remote ID
   field choice genuinely is not), the disagreement is logged in the contract's
   **contradiction ledger** with this file as the current-truth pointer, rather than
   being silently smoothed over.
3. `docs/tripod_test_protocol.md` §9 and `docs/kill_day_protocol.md` §1.4/§9 stop
   being "unverified" and cite the filled sections here.

**Last filled by:** `____________`  **date:** `____________`
**Next review trigger:** any change of site, aircraft, mass, regime, or Remote ID
method — and before the first flight of each field day.

</details>
