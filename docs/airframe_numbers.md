# Airframe and propulsion numbers

The physical numbers for the 5-inch, 6S interceptor: weight, thrust, thrust-to-weight,
hover current and time, the battery, motor and prop constants, drag, air density, and the
carbon fibre frame. The target drone shares the same motors, props, frame and packs, so
most of this applies to it too.

**Status: nothing on the interceptor has been weighed or thrust-tested yet.** Every number
below is tagged `vendor` (manufacturer or retailer spec), `derived` (calculated here),
`estimate` (a typical value or assumption to replace with a measurement), or `bench`.
Every `derived` number comes out of **`scripts/airframe_numbers.py`** and is pinned by
`tests/test_airframe_numbers.py`. When the scale and thrust rig produce real numbers, re-run
the script with them (`--auw`, `--thrust`), update this doc and the test together, and update
the contract's `airframe-thrust-and-mass-unmeasured` assumption.

Interactive version (type in a weight, everything recomputes): `docs/airframe_calculator.html`,
hosted at https://claude.ai/artifact/VZ8qf3qmhdcEfrwZ74F5jk

---

## 1. Summary

| Quantity | Value | Tier |
|---|---|---|
| All-up weight (AUW), interceptor | ≈ 800 g (546.5 g dry + 254 g pack) | derived from estimates |
| Design ceiling used for sizing | 950 g | plan |
| Max static thrust | 1.5 kg per motor, 6.0 kg total (listings 1.4 to 2.0 kg) | estimate |
| Thrust-to-weight | 7.5 : 1 at 800 g; 6.3 : 1 at 950 g (rule: ≥ 5 : 1) | derived |
| Hover current | ≈ 7.5 A (5.5 to 7.5 A across the FM/efficiency band) | derived |
| Hover time | ≈ 10 min (9.6 to 13.1 min), 80% of a 1500 mAh pack | derived |
| Hover throttle | ≈ 37 % | derived |
| Target drone weight | 610.5 g bare, 803.5 g with placard | derived (`placard_mount.md` §4.2) |

---

## 2. Parts that set these numbers

| Part | Spec that matters | Tier |
|---|---|---|
| Motors: EMAX ECO II 2207 1900KV ×4 (+1 spare) | 1900 RPM/V. Listed 30.4 to 33.6 g without wire. Listed max thrust 1850 to 2020 g on 6S; peak 45 A, 1130 W. | vendor |
| Props: HQProp V2S 5x4.3x3 | 5.0 in (0.127 m) diameter, 4.3 in pitch, 3 blades. Disk area 0.01267 m² each. | vendor |
| Battery: CNHL Black 6S 1500 mAh 100C | 22.2 V nominal, 25.2 V full, 33.3 Wh. Listed 254 to 266 g with leads. | vendor |
| ESC: Skystars KO60II 60A 4-in-1 (AM32) | 60 A per motor, 3 to 6S | vendor |
| Frame: TBS Source One V6, 5 in | 240 mm wheelbase, 6 mm arms, 2 mm top plate, 136.5 g with hardware | vendor |
| FC + power: Pixhawk 6C Mini, PM02 V3 | PM02 logs pack voltage and current, which is how hover current gets measured | vendor |

Target drone: same motors, props, frame and packs; Kakute H7 + Tekko32 65A, ArduPilot.

---

## 3. Weight budget (interceptor)

| Item | g | Tier |
|---|---:|---|
| Frame, Source One V6 with hardware | 136.5 | vendor |
| 4 motors + 4 props | 146 | estimate |
| ESC, KO60II | 15 | estimate |
| Pixhawk 6C Mini | 35 | estimate |
| PM02 V3 | 30 | estimate |
| M10 GPS | 15 | estimate |
| RP3 receiver | 5 | estimate |
| Raspberry Pi 5 8GB | 46 | estimate |
| Pi active cooler | 25 | estimate |
| OV9281 + M12 lens | 15 | estimate |
| Matek 12S Pro BEC | 10 | estimate |
| Printed Pi tray + camera boom | 35 | estimate |
| Wiring, standoffs, XT60, strap | 30 | estimate |
| Buzzer | 3 | estimate |
| **Dry** | **546.5** | derived |
| CNHL 6S 1500 pack | 254 | vendor |
| **AUW** | **≈ 800** | derived |
| + AI HAT+ (neural-net phase) | +20 | estimate |

The earlier roll-up in `docs/decisions.md` (≈ 770 g, payload-feasibility entry) used the
first parts pick (Mark5 frame, XING2 motors). This table uses the parts actually ordered.
Both aircraft are over 250 g, so both need FAA registration and Remote ID outdoors.

---

## 4. Thrust and thrust-to-weight

**Where the thrust number comes from.**
- Planning value **1.5 kg per motor, 6.0 kg static** (`hardware_order_list.md` §3). It was
  first set for the XING2 1855KV; it is kept for the ECO II because it is the low end of
  what the listings claim.
- Retailer max for ECO II 2207 1900KV on 6S: **1850 g** and **2020 g** (vendor). The listings
  do not name the prop, so treat these as an upper bound.
- Similar-motor cross-check: FlashHobby K2207.5 1900KV, Gemfan 5040, about 23.6 V, full
  throttle: **1406 g, 28.4 A, 669 W, 2.10 g/W** (vendor).
- The DIY thrust rig (motor inverted on a kitchen scale, reading grams and amps at throttle
  steps) replaces all of these. It is on the must-add list.

```
T/W = (4 × thrust per motor) / AUW
  planning, 800 g:     6000 / 800  = 7.5 : 1
  listing max, 800 g:  8080 / 800  = 10.1 : 1
  design ceiling:      6000 / 950  = 6.3 : 1     (rule: ≥ 5 : 1)
```

**What T/W buys.**
- Steepest tilt that still holds altitude: θ = arccos(1 / (T/W)) ≈ 82° at 7.5 : 1.
- Max sideways acceleration at constant altitude: a = g √((T/W)² − 1) ≈ 73 m/s² on paper.
  In practice the autopilot tilt limit sets it: PX4's default MPC_TILTMAX_AIR = 45° gives
  g tan 45° = 9.8 m/s². The flight plan's dash acceleration constant is 10 m/s².
- Control margin. A quad at full collective cannot correct attitude because no motor can
  speed up further. High T/W keeps the dash and the terminal turn from saturating the motors.

---

## 5. Hover current and hover time

Momentum theory gives the ideal hover power; divide by the prop's figure of merit (FM) and
the motor plus ESC efficiency (η) for electrical power, then by pack voltage for current.

```
At 800 g, sea level, 5.0 in props
W        = 0.800 × 9.81                    = 7.85 N
T/rotor  = W / 4                           = 1.96 N   (200 g)
A        = π (0.127/2)²                    = 0.01267 m²
v_i      = √(T / 2ρA)                      = 7.95 m/s   induced velocity
P_ideal  = 4 T v_i  = (4 T^1.5) / √(2ρA)   = 62.5 W
P_elec   = P_ideal / (FM × η) = 62.5 / (0.50 × 0.75) = 167 W
I_hover  = P_elec / 22.2 V                 = 7.5 A
t_hover  = (0.80 × 1.5 Ah) / 7.5 A         = 9.6 min
throttle ≈ √(W / T_max) = √(1 / 7.5)       = 37 %
```

| FM | η | Hover current | Hover time |
|---|---|---|---|
| 0.60 | 0.85 | 5.5 A | 13.1 min |
| 0.55 | 0.80 | 6.4 A | 11.3 min |
| 0.50 | 0.75 | 7.5 A | 9.6 min |

- **Least certain inputs:** FM (small FPV props run about 0.4 to 0.6) and η at low throttle
  (about 0.75 to 0.85). Both are `estimate`.
- **80 % usable:** landing at about 3.5 V per cell under load leaves about 20 % in the pack.
- **Scaling:** hover power goes as weight^1.5. +10 % weight costs about +15 % power. That is
  why the target's placard (+32 % weight) is expected to cost it 25 to 35 % of its flight
  time (`placard_mount.md` §4.6).
- **Hover is the best case.** A full-throttle dash draws on the order of 100+ A (four motors
  at 28 to 45 A), which would use the usable pack in under a minute. The launch plan budgets
  30 to 60 s of standby hover (`launch_mechanism_plan.md`).
- **How it gets measured:** a 60 s hover, current from the PM02 in the ULog, and the learned
  hover throttle (PX4 MPC_THR_HOVER; ArduPilot MOT_THST_HOVER on the target).

---

## 6. The 6S pack

| Quantity | Value | How |
|---|---:|---|
| Nominal | 22.2 V | 6 × 3.7 V |
| Full | 25.2 V | 6 × 4.2 V |
| Storage | 22.8 V | 6 × 3.8 V |
| Land by, under load | ≈ 21.0 V | 6 × 3.5 V (rule of thumb) |
| Energy | 33.3 Wh | 1.5 Ah × 22.2 V |
| Specific energy | ≈ 131 Wh/kg | 33.3 Wh / 0.254 kg |
| Rated burst | 150 A | 100C × 1.5 Ah (marketing rating) |
| Standard charge | 1.5 A | 1C |

Why 6S: for the same power, higher voltage means lower current; I²R loss in pack, wires and
ESC drops with the square of current, and voltage sag on a dash is smaller. Both aircraft
also share one pack type and one charger.

---

## 7. Motor and prop constants

```
KV                    1900 RPM/V (no load)
no-load RPM           1900 × 25.2 = 47,880 (full) · 1900 × 22.2 = 42,180 (nominal)
loaded full RPM       ≈ 35,000   (estimate: ~80 % of KV × ~23 V sagged)
torque constant       Kt = 60 / (2π KV) = 0.00503 N·m/A;  τ ≈ Kt (I − I_noload)
tip speed at 35k      ≈ 232 m/s  (Mach 0.68)
pitch speed at 35k    4.3 in × 35,000 RPM ≈ 64 m/s
hover RPM             ≈ 35,000 × √(200 g / 1500 g) ≈ 12,800 RPM
                      = 213 Hz shaft, 638 Hz blade pass (3 blades)
```

- Prop coefficients, measured on the thrust rig: T = C_T ρ n² D⁴, P = C_P ρ n³ D⁵ (n in
  rev/s), FM = T^1.5 / (P √(2ρA)).
- Thrust falls toward zero as airspeed approaches the pitch speed, so static thrust
  overstates top speed.
- The hover shaft and blade-pass frequencies are what the gyro notch filter has to remove;
  AM32 ESCs can report RPM over bidirectional DShot for an RPM-tracking notch.
- Hot-motor fallback from `hardware_order_list.md` §3: 1755 KV and/or a lower-pitch prop.
  The 5x4.3x3 is already the cooler-running prop choice.

---

## 8. Drag, trim and air density

```
Cd·A, bare 5 in quad     ≈ 0.014 m²   (assumption, placard_mount.md §4.3:
                                       back-solved from ~30° trim at 20 m/s)
D = ½ ρ V² (Cd·A)        9 m/s: 0.69 N     16 m/s: 2.20 N
steady trim tan θ = D/W  9 m/s: 5.1°       16 m/s: 15.6°   (800 g; more while accelerating)
flat plate face-on Cd    ≈ 1.17 (Hoerner)
disk loading W/(4A)      155 N/m² at 800 g
```

Air density (ISA standard day): sea level 1.225, 975 m 1.114, 1655 m (Boulder) 1.042 kg/m³.
Max thrust scales with ρ (about −15 % at 1655 m); ideal hover power with 1/√ρ (about +8 %).
A T/W measured at sea level is optimistic at altitude, and hot days lower ρ further.

---

## 9. Carbon fibre

| What | Young's modulus | Tier |
|---|---:|---|
| Fibre alone (Toray T300 / T700S) | 230 GPa | vendor |
| Unidirectional laminate, along fibres (60 % Vf, epoxy) | 134 to 140 GPa | vendor |
| Same, across fibres (E2) | ≈ 8 to 10 GPa | estimate |
| Quasi-isotropic plate (frame plate, in-plane) | ≈ 56 GPa | derived |
| Aluminium 6061-T6 | 69 GPa | vendor |

```
E_QI ≈ 3/8 E1 + 5/8 E2 = 3/8 (135) + 5/8 (9) ≈ 56 GPa   (lamination-theory rule of thumb)
fibre strength: T300 3.53 GPa, T700S 4.90 GPa
density: CFRP ≈ 1.55 g/cm³, aluminium 2.70 g/cm³
specific stiffness E/ρ: CFRP ≈ 36, aluminium ≈ 26 MN·m/kg
```

- TBS does not publish the Source One plate layup or fibre grade, so 56 GPa is an estimate
  for this frame.
- **Carbon fibre conducts** (T300 resistivity about 1.7 × 10⁻⁵ Ω·m). A bare or over-stripped
  wire touching the frame can short to it; that is the likely contributor to target FC #1's
  dead I²C bus (`build_log.md`), and why a nicked motor winding shorts a phase to the frame
  (`decisions.md`, motor-screw note).
- Carbon is galvanically noble to aluminium: aluminium in contact with carbon corrodes in
  moisture.

**Arm check** (cantilever, motor at the tip). Thickness 6 mm is vendor; width 12 mm and free
length 90 mm are assumptions until measured with calipers.

```
I = b h³ / 12                        = 216 mm⁴
tip deflection δ = F L³ / 3EI         = 0.29 mm at 1.5 kgf
root stress σ = F L (h/2) / I         = 18 MPa   (far below laminate strength; crashes are the real load case)
first bending mode f = √(3EI / L³m) / 2π = 186 Hz  (36.5 g motor + prop, arm mass ignored, rigid root)
```

The arm's first mode lands near the hover shaft rate (≈ 213 Hz). With assumed dimensions this
is a reason to measure, not a finding: if confirmed, it shows up as gyro noise and is a
filter-tuning input.

---

## 10. Measurements that replace these estimates

| Measurement | Replaces | Where it lands |
|---|---|---|
| Kitchen scale, built aircraft | AUW roll-up | `--auw`, §1, §3 |
| Thrust rig, grams and amps per throttle step | 1.5 kg/motor, FM | `--thrust`, §4, §7 |
| 60 s hover, PM02 current + learned hover throttle | hover current, time, η·FM | §5 |
| First dash ULog, steady pitch at speed | Cd·A, trim | §8 (also sets camera tilt) |
| Calipers on an arm | arm width, length | §9 |

## Sources

- Repo: `hardware_order_list.md` (ordered parts, §3 sizing), `project_state.json` (`bom_tiers`),
  `placard_mount.md` §4 (target mass, Cd, drag), `launch_mechanism_plan.md` (standby budget),
  `decisions.md` ADR-0084 (contact radius).
- ECO II 2207 listings: [Zbotic](https://zbotic.in/product/ecoii-2207-1900kv-brushless-motor/)
  (2020 g on 6S), [Wind Catcher RC](https://windcatcherrc.com/product/eco-ii-2306-1700-1900-2400kv-emax-brushless-motor/)
  (1850 g on 6S), [RCFlight](https://www.rcflight.se/visaprodukt.aspx?id=13400&p=emax-eco-ii-2207-1900kv-6s)
  (45 A, 1130 W), [EMAX USA](https://shop.emax-usa.com/products/eco-ii-2207-brushless-motor-1700kv-1900kv-2400kv).
- Similar motor: [FlashHobby K2207.5 1900KV](https://www.flyingtech.co.uk/product/flashhobby-king-k2207-5-brushless-motor-1900kv-2500kv-2750kv/).
- Pack: [RaceDayQuads](https://racedayquads.com/products/cnhl-black-series-22-2v-6s-1500mah-100c-lipo-battery-xt60)
  (254 g), [GetFPV](https://www.getfpv.com/cnhl-black-series-100c-6s-lipo-battery-1500mah.html) (266 g).
- Carbon: [Toray T300](https://www.toraycma.com/wp-content/uploads/T300-Data-Sheet.pdf),
  [Toray T700S](https://www.toraycma.com/wp-content/uploads/T700S-Data-Sheet.pdf).
- Physics: momentum theory (Leishman, *Principles of Helicopter Aerodynamics*), ISA
  atmosphere, classical lamination theory.
