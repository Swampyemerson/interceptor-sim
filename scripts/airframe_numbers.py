#!/usr/bin/env python3
"""Airframe and propulsion numbers for the 5-inch 6S interceptor (and the target,
which shares motors, props, frame and packs).

Every DERIVED figure in docs/airframe_numbers.md comes out of this file, so the doc
traces to a derivation rather than to hand arithmetic. The INPUTS are vendor specs or
estimates (tagged below); nothing here has been weighed or thrust-tested yet. When the
bench numbers exist, pass them on the command line and re-run:

    python3 scripts/airframe_numbers.py                       # current estimates
    python3 scripts/airframe_numbers.py --auw 812 --thrust 1630   # after the scale + thrust rig
    python3 scripts/airframe_numbers.py --rho 1.042           # Boulder air (1655 m ISA)

Physics used (standard textbook results):
  momentum theory hover: v_i = sqrt(T / 2 rho A), P_ideal = T v_i = T^1.5 / sqrt(2 rho A)
  electrical power      : P_elec = P_ideal / (FM * eta_drive)
  hover throttle        : sqrt(W / T_max)  (thrust ~ RPM^2, RPM ~ throttle)
  level-flight limits   : max tilt = arccos(1 / TW), max lateral accel = g sqrt(TW^2 - 1)
  trim pitch            : tan(theta) = D / W,  D = 0.5 rho V^2 (Cd A)
  cantilever arm        : I = b h^3 / 12, delta = F L^3 / 3EI, f = sqrt(3EI / L^3 m) / 2 pi
  ISA troposphere       : T = 288.15 - 6.5 h_km, p = 101325 (T/288.15)^5.2559, rho = p / 287.05 T
"""
import argparse
import math

G = 9.81
RHO_SL = 1.225  # kg/m^3, ISA sea level

# ---- inputs (tier in the comment) -----------------------------------------------
PROP_DIAMETER_M = 0.127        # vendor: HQProp V2S 5x4.3x3, 5.0 in
PROP_PITCH_M = 4.3 * 0.0254    # vendor
PROP_BLADES = 3                # vendor
MOTOR_KV = 1900                # vendor: EMAX ECO II 2207 1900KV
THRUST_PER_MOTOR_G = 1500.0    # estimate: repo planning value (low end of 1406-2020 g listings)
PACK_MAH = 1500.0              # vendor: CNHL Black 6S 1500 100C
PACK_CELLS = 6
PACK_C_RATING = 100            # vendor (marketing burst rating)
PACK_MASS_G = 254.0            # vendor (254-266 g listed)
USABLE_FRACTION = 0.80         # estimate: land at ~3.5 V/cell under load
FIGURE_OF_MERIT = 0.50         # estimate: small FPV props ~0.4-0.6
DRIVE_EFFICIENCY = 0.75        # estimate: motor + ESC at low throttle ~0.75-0.85
LOADED_RPM_FRACTION = 0.80     # estimate: loaded full-throttle RPM vs Kv x V at sagged voltage
SAGGED_VOLTAGE_V = 23.0        # estimate: pack voltage under full-throttle load
CDA_M2 = 0.014                 # assumption: docs/placard_mount.md 4.3 (back-solved, ~30 deg trim at 20 m/s)
TILT_LIMIT_DEG = 45.0          # PX4 default MPC_TILTMAX_AIR

# interceptor component roll-up, grams: (value, tier)
INTERCEPTOR_PARTS = {
    "Frame, TBS Source One V6 with hardware": (136.5, "vendor"),
    "4 x ECO II 2207 motors + 4 x HQ V2S props": (146.0, "estimate"),
    "ESC, Skystars KO60II 60A": (15.0, "estimate"),
    "Pixhawk 6C Mini": (35.0, "estimate"),
    "PM02 V3 power module": (30.0, "estimate"),
    "M10 GPS": (15.0, "estimate"),
    "RP3 receiver": (5.0, "estimate"),
    "Raspberry Pi 5 8GB": (46.0, "estimate"),
    "Pi active cooler": (25.0, "estimate"),
    "OV9281 camera + M12 lens": (15.0, "estimate"),
    "Matek 12S Pro BEC": (10.0, "estimate"),
    "Printed Pi tray + camera boom": (35.0, "estimate"),
    "Wiring, standoffs, XT60, strap": (30.0, "estimate"),
    "Buzzer": (3.0, "estimate"),
}

# carbon fibre (GPa / g/cm^3)
E_FIBRE_GPA = 230.0            # vendor: Toray T300 / T700S data sheets
E1_UD_GPA = 135.0              # vendor: Toray composite data, 60% Vf epoxy (134-140)
E2_UD_GPA = 9.0                # estimate: typical carbon/epoxy transverse modulus (8-10)
RHO_CFRP = 1.55
E_AL6061_GPA = 69.0
RHO_AL = 2.70
# arm (Source One V6): thickness vendor, width and free length are ASSUMPTIONS until measured
ARM_WIDTH_MM = 12.0
ARM_THICK_MM = 6.0
ARM_FREE_LENGTH_MM = 90.0
TIP_MASS_G = 36.5              # one motor + one prop (from the 146 g line)


def disk_area():
    return math.pi * (PROP_DIAMETER_M / 2) ** 2


def isa_density(h_m):
    t = 288.15 - 0.0065 * h_m
    p = 101325.0 * (t / 288.15) ** 5.2559
    return p / (287.05 * t)


def dry_mass_g():
    return sum(v for v, _ in INTERCEPTOR_PARTS.values())


def hover(auw_g, thrust_g=THRUST_PER_MOTOR_G, rho=RHO_SL, fm=FIGURE_OF_MERIT,
          eta=DRIVE_EFFICIENCY, mah=PACK_MAH, usable=USABLE_FRACTION, v_pack=None):
    """Hover and thrust-margin figures for one all-up weight. thrust_g is the
    sea-level static thrust per motor; it is scaled by rho / RHO_SL."""
    if v_pack is None:
        v_pack = 3.7 * PACK_CELLS
    a = disk_area()
    w = auw_g / 1000 * G
    t_rotor = w / 4
    thrust_alt_g = thrust_g * rho / RHO_SL
    tw = 4 * thrust_alt_g / auw_g
    v_i = math.sqrt(t_rotor / (2 * rho * a))
    p_ideal = 4 * t_rotor * v_i
    p_elec = p_ideal / (fm * eta)
    i_hover = p_elec / v_pack
    t_min = (mah / 1000) * usable / i_hover * 60
    return {
        "auw_g": auw_g, "weight_N": w, "thrust_per_rotor_N": t_rotor,
        "thrust_per_motor_g_at_rho": thrust_alt_g, "tw": tw,
        "hover_throttle": math.sqrt(1 / tw),
        "induced_velocity_ms": v_i, "p_ideal_W": p_ideal, "p_elec_W": p_elec,
        "hover_current_A": i_hover, "hover_time_min": t_min,
        "g_per_W": auw_g / p_elec, "disk_loading_Nm2": w / (4 * a),
        "max_level_tilt_deg": math.degrees(math.acos(1 / tw)) if tw > 1 else float("nan"),
        "max_lateral_accel_ms2": G * math.sqrt(tw * tw - 1) if tw > 1 else float("nan"),
        "tilt_limited_accel_ms2": G * math.tan(math.radians(TILT_LIMIT_DEG)),
    }


def trim(auw_g, v_ms, rho=RHO_SL, cda=CDA_M2):
    w = auw_g / 1000 * G
    d = 0.5 * rho * v_ms ** 2 * cda
    return {"drag_N": d, "trim_deg": math.degrees(math.atan(d / w))}


def battery():
    wh = PACK_MAH / 1000 * 3.7 * PACK_CELLS
    return {
        "nominal_V": 3.7 * PACK_CELLS, "full_V": 4.2 * PACK_CELLS,
        "storage_V": 3.8 * PACK_CELLS, "land_V": 3.5 * PACK_CELLS,
        "energy_Wh": wh, "specific_energy_Whkg": wh / (PACK_MASS_G / 1000),
        "burst_A": PACK_C_RATING * PACK_MAH / 1000, "charge_1C_A": PACK_MAH / 1000,
    }


def motor(hover_thrust_g=None):
    rpm_noload_full = MOTOR_KV * 4.2 * PACK_CELLS
    rpm_noload_nom = MOTOR_KV * 3.7 * PACK_CELLS
    rpm_loaded = MOTOR_KV * SAGGED_VOLTAGE_V * LOADED_RPM_FRACTION
    tip = rpm_loaded / 60 * 2 * math.pi * PROP_DIAMETER_M / 2
    out = {
        "rpm_noload_full": rpm_noload_full, "rpm_noload_nominal": rpm_noload_nom,
        "rpm_loaded_full": rpm_loaded, "kt_NmA": 60 / (2 * math.pi * MOTOR_KV),
        "tip_speed_ms": tip, "tip_mach": tip / 343.0,
        "pitch_speed_ms": PROP_PITCH_M * rpm_loaded / 60,
    }
    if hover_thrust_g is not None:
        rpm_h = rpm_loaded * math.sqrt(hover_thrust_g / THRUST_PER_MOTOR_G)
        out.update(rpm_hover=rpm_h, shaft_hz_hover=rpm_h / 60,
                   blade_pass_hz_hover=rpm_h / 60 * PROP_BLADES)
    return out


def carbon():
    e_qi = 3 / 8 * E1_UD_GPA + 5 / 8 * E2_UD_GPA
    return {"E_quasi_iso_GPa": e_qi, "specific_E_cfrp": e_qi / RHO_CFRP,
            "specific_E_al": E_AL6061_GPA / RHO_AL}


def arm(e_gpa=None, width_mm=ARM_WIDTH_MM, thick_mm=ARM_THICK_MM,
        length_mm=ARM_FREE_LENGTH_MM, tip_force_g=THRUST_PER_MOTOR_G, tip_mass_g=TIP_MASS_G):
    e = (e_gpa if e_gpa is not None else carbon()["E_quasi_iso_GPa"]) * 1e9
    b, h, L = width_mm / 1000, thick_mm / 1000, length_mm / 1000
    i = b * h ** 3 / 12
    k = 3 * e * i / L ** 3
    f = tip_force_g / 1000 * G
    return {"I_mm4": i * 1e12, "tip_deflection_mm": f / k * 1000,
            "root_stress_MPa": f * L * (h / 2) / i / 1e6,
            "first_mode_Hz": math.sqrt(k / (tip_mass_g / 1000)) / (2 * math.pi)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--auw", type=float, default=None, help="all-up weight, g (default: roll-up)")
    ap.add_argument("--thrust", type=float, default=THRUST_PER_MOTOR_G, help="sea-level max thrust per motor, g")
    ap.add_argument("--rho", type=float, default=RHO_SL, help="air density, kg/m^3")
    ap.add_argument("--fm", type=float, default=FIGURE_OF_MERIT)
    ap.add_argument("--eta", type=float, default=DRIVE_EFFICIENCY)
    args = ap.parse_args(argv)

    dry = dry_mass_g()
    auw = args.auw if args.auw is not None else dry + PACK_MASS_G
    print(f"Interceptor dry {dry:.1f} g + pack {PACK_MASS_G:.0f} g = AUW {dry + PACK_MASS_G:.1f} g (roll-up)")
    print(f"Air density {args.rho:.3f} kg/m^3  (ISA 975 m {isa_density(975):.3f}, 1655 m {isa_density(1655):.3f})\n")

    cases = [("this AUW", auw), ("design ceiling", 950.0), ("target bare", 610.5), ("target + placard", 803.5)]
    for name, m in cases:
        h = hover(m, args.thrust, args.rho, args.fm, args.eta)
        print(f"[{name}, {m:.0f} g]  T/W {h['tw']:.2f}:1  hover {h['hover_current_A']:.1f} A, "
              f"{h['hover_time_min']:.1f} min, throttle {h['hover_throttle']*100:.0f}%  "
              f"P_ideal {h['p_ideal_W']:.1f} W -> P_elec {h['p_elec_W']:.0f} W  v_i {h['induced_velocity_ms']:.2f} m/s  "
              f"disk {h['disk_loading_Nm2']:.0f} N/m^2  max tilt {h['max_level_tilt_deg']:.0f} deg")
    for fm, eta in ((0.6, 0.85), (0.55, 0.80), (0.5, 0.75)):
        h = hover(auw, args.thrust, args.rho, fm, eta)
        print(f"  sensitivity FM {fm} eta {eta}: {h['hover_current_A']:.1f} A, {h['hover_time_min']:.1f} min")
    h = hover(auw, args.thrust, args.rho, args.fm, args.eta)
    print(f"  lateral accel: paper {h['max_lateral_accel_ms2']:.1f} m/s^2, at {TILT_LIMIT_DEG:.0f} deg tilt limit "
          f"{h['tilt_limited_accel_ms2']:.1f} m/s^2")
    for v in (9.0, 16.0):
        t = trim(auw, v, args.rho)
        print(f"  {v:.0f} m/s: drag {t['drag_N']:.2f} N, steady trim {t['trim_deg']:.1f} deg")

    b = battery()
    print(f"\nPack: {b['nominal_V']:.1f} V nominal / {b['full_V']:.1f} full / {b['storage_V']:.1f} storage / "
          f"land ~{b['land_V']:.1f}; {b['energy_Wh']:.1f} Wh, {b['specific_energy_Whkg']:.0f} Wh/kg, "
          f"burst {b['burst_A']:.0f} A, 1C {b['charge_1C_A']:.1f} A")
    mo = motor(hover_thrust_g=auw / 4)
    print(f"Motor: no-load {mo['rpm_noload_full']:.0f} RPM full / {mo['rpm_noload_nominal']:.0f} nominal, "
          f"loaded ~{mo['rpm_loaded_full']:.0f}; Kt {mo['kt_NmA']:.5f} N.m/A; tip {mo['tip_speed_ms']:.0f} m/s "
          f"(M {mo['tip_mach']:.2f}); pitch speed {mo['pitch_speed_ms']:.0f} m/s; hover ~{mo['rpm_hover']:.0f} RPM "
          f"= {mo['shaft_hz_hover']:.0f} Hz shaft, {mo['blade_pass_hz_hover']:.0f} Hz blade pass")
    c = carbon()
    print(f"Carbon: quasi-iso E ~{c['E_quasi_iso_GPa']:.0f} GPa; E/rho {c['specific_E_cfrp']:.0f} vs Al "
          f"{c['specific_E_al']:.0f} MN.m/kg")
    a = arm(tip_force_g=args.thrust)
    print(f"Arm (assumed {ARM_WIDTH_MM:.0f} x {ARM_THICK_MM:.0f} mm, L {ARM_FREE_LENGTH_MM:.0f} mm): "
          f"deflection {a['tip_deflection_mm']:.2f} mm, root stress {a['root_stress_MPa']:.0f} MPa, "
          f"first mode {a['first_mode_Hz']:.0f} Hz")


if __name__ == "__main__":
    main()
