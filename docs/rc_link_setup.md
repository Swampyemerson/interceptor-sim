# RC link setup — interceptor ELRS receiver (afr-03, RC side)

**Verified live over the RX's WiFi config mode, 2026-10-24 (bench session).**
Plain-English record of what the receiver is, how it is configured, and what
still has to happen on the transmitter side before the maiden.

## The receiver, as found on the bench LAN

| What | Value | Meaning |
|---|---|---|
| Hardware | RadioMaster **RP3 Diversity** 2.4 GHz | two antennas, picks the better one each packet |
| Firmware | ExpressLRS **3.3.1** (`e051b8`), target `UNIFIED_ESP8285_2400_RX` | 3.x — modern config API; **do not update** without matching the TX major version |
| Reached at | `elrs_rx.local` = 192.168.0.58 (STA mode on the bench WiFi) | WiFi config mode auto-starts **60 s after power-up if no TX connects**; a power-cycle leaves it |
| Serial protocol | **SBUS** (web API `serial-protocol: 2`) | ✅ exactly what the Pixhawk 6C Mini's RC IN pad needs — stock PX4 has no CRSF driver (`configs/px4_6cmini/README.md` §4). Already set; nothing was changed. |
| SBUS failsafe | **No pulses** (`sbus-failsafe: 0`) | ✅ correct for PX4: on link loss the FC *sees* the loss and runs its own failsafe (RTL at 30 m, `NAV_RCL_ACT=2`). "Hold last position" would mask the loss — do not change this. |
| Binding phrase | **Set** (`uidtype: Overridden`; UID deliberately not recorded — public repo, the UID is the bind credential) | TX must carry the **same phrase** to bind |
| Model match | **Off** (`modelid: 255`) | RX accepts any model slot from a phrase-matched TX. Fine for one aircraft; revisit when the 2nd TX / target drone arrives (ADR-0089) |
| Telemetry | `force-tlm: false`, domain ISM2G4 | stock |

One heads-up: the RX's config page shows the bench WiFi name and password in
plain text to anyone on the LAN — normal ELRS behavior, just never screenshot
it into this (public) repo.

## What still has to happen, in order (TX side + bench — your hands)

1. **TX binding phrase**: ~~must match the RX's~~ **DONE 2026-10-24** — the
   builder set the TX's phrase, and the RX's UID was rewritten over its WiFi
   page the same session to the matching value (old RX UID did NOT match, so
   without this write they would never have bound). The phrase/UID stay out of
   this public doc — they are the bind credential. With both sides matched,
   the RX connects (solid LED) within a few seconds of both being on.
2. **Power-cycle the RX out of WiFi mode** (it does no RC while configuring).
   After power-up, connect the TX within ~60 s or the RX drops back into
   WiFi mode and stops listening for the TX until the next power-cycle.
3. **EdgeTX mixer on the Pocket** — three switch assignments:
   - **Channel 5 = kill switch** (pick a 2-pos switch you can hit without
     looking; the FC already maps `RC_MAP_KILL_SW=5`).
   - **A 3-pos switch on a free channel** for flight modes (QGC's Flight
     Modes page will bind to whatever channel it sees moving).
   - **A GO switch on a free channel** for the autonomous mission later
     (bench item L3/L6 measures its channel + endpoints; not needed for the
     manual maiden).
4. **QGC radio calibration** (USB, props off), then flip the kill switch and
   confirm QGC logs *Kill engaged* / *Kill disengaged* — the protocol's
   "kill switch verified live" line (§10).

After step 4, the RC side of afr-03 is closed; what remains for the maiden is
the QGC safety-breaker flip + flight-mode switch assignment recorded in the
2026-10-24 bench session notes (contract `plain_log`).
