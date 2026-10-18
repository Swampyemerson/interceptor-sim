# Pi 5 battery-power stress test — 2026-10-17 (next.md 6a follow-up)

Setup: Pi 5 on the battery/BEC path (1000 µF cap on the BEC 5 V output, commit
27f257e), fresh cold boot (power-apply booted with NO button press — the 6a
experiment's battery arm). Load: 4× `sha256sum /dev/zero` (all cores, 100%),
180 s. Sampled every 5 s over SSH: `vcgencmd pmic_read_adc EXT5V_V` (the PMIC's
own measurement of the 5 V input), `vcgencmd get_throttled` (sticky
under-voltage/throttle flags since boot), temp, ARM clock.

## Result: PASS — no electrical evidence of a battery-power problem

| quantity | idle (baseline) | under load (36 samples / 180 s) |
|---|---|---|
| EXT5V_V | 5.317 V | **5.290–5.298 V** (sag ≈ 20 mV) |
| get_throttled | 0x0 | **0x0 every sample AND final** (no sticky bits) |
| ARM clock | 2400 MHz | 2400 MHz pinned (never throttled) |
| temp | 37.8 °C | plateau ≈ 55 °C |

The sticky `get_throttled` bits are latched in hardware for ANY under-voltage
event since boot (<~4.8 V even transiently), so "0x0 at the end" covers the
transients the 5 s ADC sampling could miss.

The builder's "LED looks dimmer/flickery on battery" observation is therefore
NOT corroborated at the Pi's power input. Honest limits: (i) CPU-only load —
no camera running (comparable CPU to the 96 fps AprilTag soak, but not
identical current profile); (ii) one state-of-charge point — the flight-length
endurance-under-load run on a draining pack is a separate, later gate; (iii)
Wi-Fi was active (SSH).

## 6a cold-cycle experiment (same session, pre-registered in next.md)

Both arms BOOTED with no button press: (a) wall PSU cold cycle → boots;
(b) battery/BEC cold cycle → boots. Per the pre-registration this REFUTES the
BEC-ramp theory and attributes the two standby incidents (2026-10-16,
suspected 2026-10-17) to carried-over halt/standby state — a software-halted
Pi 5 sits in standby, looking dead, until the button or a TRUE power pull.
EEPROM already ruled out on the live board (no POWER_OFF_ON_HALT, bootloader
2026-09-25). No setting was changed; boot-on-power works as shipped.
Standing rule: always pull power FULLY (and let it drain) instead of relying
on `halt`. Reopen only if a standby recurs from a verified true cold start.

## ⚠️ ADDENDUM, ~20 min after the test: the pack died — the caveats were load-bearing

The Pi dropped off the network and the builder found its lights fully OFF (total
5 V loss — a different signature from the solid-red standby). Most likely the
LiPo sagged below the BEC dropout / LVC; the 180 s all-core stress above will
have hastened the drain. Two honest corrections:
1. **The builder's "LED looks dimmer" observation is partially VINDICATED**, not
   refuted: this test measured the REGULATED side of the BEC, which holds 5 V
   rock-solid right up until its input collapses, then drops out abruptly. A
   sagging pack is invisible in EXT5V until the end. "Not corroborated at the
   Pi's input" was true but answered the wrong question.
2. **Method rule for every future battery-side test:** monitor PACK voltage via
   the FC's power module over MAVLink (battery_status) alongside EXT5V. The
   clean-5V verdict above stands only for "BEC regulation under CPU load at that
   state of charge"; it says nothing about endurance, which is the open gate.

```
  5 0x0 5.29702 39.5 2400
 10 0x0 5.29568 46.1 2400
 15 0x0 5.29702 47.7 2400
 20 0x0 5.29568 48.8 2400
 25 0x0 5.29568 49.4 2400
 30 0x0 5.29702 50.5 2400
 35 0x0 5.29434 51.0 2400
 40 0x0 5.29836 50.5 2400
 45 0x0 5.29568 51.0 2400
 50 0x0 5.29568 51.0 2400
 55 0x0 5.29568 50.5 2400
 60 0x0 5.29434 51.0 2400
 65 0x0 5.29166 51.6 2400
 70 0x0 5.29702 51.6 2400
 75 0x0 5.29434 51.6 2400
 80 0x0 5.29434 51.6 2400
 85 0x0 5.29568 52.7 2400
 90 0x0 5.29568 52.1 2400
 95 0x0 5.29166 52.1 2400
100 0x0 5.29568 52.1 2400
105 0x0 5.29300 52.7 2400
110 0x0 5.29434 53.2 2400
115 0x0 5.29434 52.7 2400
120 0x0 5.29568 52.7 2400
125 0x0 5.29568 53.8 2400
130 0x0 5.29568 54.9 2400
135 0x0 5.29836 53.8 2400
140 0x0 5.29434 53.2 2400
145 0x0 5.29568 55.4 2400
150 0x0 5.29166 54.3 2400
155 0x0 5.29568 54.3 2400
160 0x0 5.29166 53.8 2400
165 0x0 5.29166 54.3 2400
170 0x0 5.29434 53.8 2400
175 0x0 5.29032 54.9 2400
180 0x0 5.29568 54.9 2400
```
