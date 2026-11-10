"""Every arithmetic figure PRINTED in docs/airframe_numbers.md is re-derived here from
scripts/airframe_numbers.py and compared with the text.

tests/test_airframe_numbers.py pins the script's outputs; this file closes the other
half of the loop: the doc could drift from the script (a hand edit, a stale paste) and
that test would stay green. Each check FAILS if its pattern is not found in the doc --
a figure that silently disappears must not pass as "still correct" (no vacuous verdicts).
"""
import importlib.util
import json
import math
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location("airframe_numbers", ROOT / "scripts" / "airframe_numbers.py")
af = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(af)

DOC = (ROOT / "docs" / "airframe_numbers.md").read_text(encoding="utf-8")
AUW = af.dry_mass_g() + af.PACK_MASS_G      # 800.5 g: the doc's "800 g" figures are this roll-up
H = af.hover(AUW)
M = af.motor(hover_thrust_g=200.0)
B = af.battery()
C = af.carbon()
A = af.arm()
G = af.G


def _num(s):
    return float(s.replace(",", ""))


def _half_unit(s):
    """Rounding tolerance of a printed number: half of its last printed digit."""
    s = s.replace(",", "")
    return 0.5 * 10 ** (-len(s.split(".")[1])) if "." in s else 0.5


# (label, regex with ONE capture group around the printed number, derived value, extra relative slack)
# Extra slack is only for figures the doc prints with a "≈" and rounds to 2-3 significant digits.
CHECKS = [
    ("weight N", r"W\s+= 0\.800 × 9\.81\s+= ([\d.]+) N", 0.800 * G, 0),
    ("thrust per rotor N", r"T/rotor\s+= W / 4\s+= ([\d.]+) N", H["thrust_per_rotor_N"], 0.002),
    ("disk area", r"A\s+= π \(0\.127/2\)²\s+= ([\d.]+) m²", af.disk_area(), 0),
    ("induced velocity", r"v_i\s+= √\(T / 2ρA\)\s+= ([\d.]+) m/s", H["induced_velocity_ms"], 0),
    ("ideal power", r"P_ideal\s+= 4 T v_i.*= ([\d.]+) W", H["p_ideal_W"], 0),
    ("electrical power", r"62\.5 / \(0\.50 × 0\.75\) = ([\d.]+) W", H["p_elec_W"], 0),
    ("hover current", r"I_hover\s+= P_elec / 22\.2 V\s+= ([\d.]+) A", H["hover_current_A"], 0),
    ("hover time", r"t_hover\s+=.*= ([\d.]+) min", H["hover_time_min"], 0),
    ("hover throttle %", r"throttle ≈ √\(W / T_max\).*= (\d+) %", H["hover_throttle"] * 100, 0),
    ("T/W planning", r"planning, 800 g:\s+6000 / 800\s+= ([\d.]+) : 1", 6000 / 800, 0),
    ("T/W listing max", r"listing max, 800 g:\s+8080 / 800\s+= ([\d.]+) : 1", 4 * 2020 / 800, 0),
    ("T/W ceiling", r"design ceiling:\s+6000 / 950\s+= ([\d.]+) : 1", 6000 / 950, 0),
    ("max tilt", r"arccos\(1 / \(T/W\)\) ≈ (\d+)°", H["max_level_tilt_deg"], 0),
    ("paper lateral accel", r"≈ (\d+) m/s² on paper", H["max_lateral_accel_ms2"], 0),
    ("tilt-limited accel", r"g tan 45° = ([\d.]+) m/s²", H["tilt_limited_accel_ms2"], 0),
    ("sens 0.60/0.85 A", r"\| 0\.60 \| 0\.85 \| ([\d.]+) A", af.hover(AUW, fm=0.60, eta=0.85)["hover_current_A"], 0),
    ("sens 0.60/0.85 min", r"\| 0\.60 \| 0\.85 \| [\d.]+ A \| ([\d.]+) min", af.hover(AUW, fm=0.60, eta=0.85)["hover_time_min"], 0),
    ("sens 0.55/0.80 A", r"\| 0\.55 \| 0\.80 \| ([\d.]+) A", af.hover(AUW, fm=0.55, eta=0.80)["hover_current_A"], 0),
    ("sens 0.55/0.80 min", r"\| 0\.55 \| 0\.80 \| [\d.]+ A \| ([\d.]+) min", af.hover(AUW, fm=0.55, eta=0.80)["hover_time_min"], 0),
    ("pack nominal V", r"\| Nominal \| ([\d.]+) V", B["nominal_V"], 0),
    ("pack full V", r"\| Full \| ([\d.]+) V", B["full_V"], 0),
    ("pack storage V", r"\| Storage \| ([\d.]+) V", B["storage_V"], 0),
    ("pack land V", r"\| Land by, under load \| ≈ ([\d.]+) V", B["land_V"], 0),
    ("pack energy", r"\| Energy \| ([\d.]+) Wh", B["energy_Wh"], 0),
    ("specific energy", r"\| Specific energy \| ≈ (\d+) Wh/kg", B["specific_energy_Whkg"], 0),
    ("burst A", r"\| Rated burst \| (\d+) A", B["burst_A"], 0),
    ("no-load rpm full", r"1900 × 25\.2 = ([\d,]+) \(full\)", M["rpm_noload_full"], 0),
    ("no-load rpm nominal", r"1900 × 22\.2 = ([\d,]+) \(nominal\)", M["rpm_noload_nominal"], 0),
    ("loaded rpm", r"loaded full RPM\s+≈ ([\d,]+)", M["rpm_loaded_full"], 0.002),
    ("Kt", r"Kt = 60 / \(2π KV\) = ([\d.]+) N·m/A", M["kt_NmA"], 0),
    ("tip speed", r"tip speed at 35k\s+≈ (\d+) m/s", M["tip_speed_ms"], 0),
    ("tip Mach", r"\(Mach ([\d.]+)\)", M["tip_mach"], 0),
    ("pitch speed", r"4\.3 in × 35,000 RPM ≈ (\d+) m/s", M["pitch_speed_ms"], 0),
    ("hover rpm", r"√\(200 g / 1500 g\) ≈ ([\d,]+) RPM", M["rpm_hover"], 0.003),
    ("hover shaft Hz", r"= (\d+) Hz shaft", M["shaft_hz_hover"], 0),
    ("hover blade-pass Hz", r"shaft, (\d+) Hz blade pass", M["blade_pass_hz_hover"], 0),
    ("drag 9 m/s", r"9 m/s: ([\d.]+) N", af.trim(800.0, 9.0)["drag_N"], 0),
    ("drag 16 m/s", r"16 m/s: ([\d.]+) N", af.trim(800.0, 16.0)["drag_N"], 0),
    ("trim 9 m/s", r"tan θ = D/W\s+9 m/s: ([\d.]+)°", af.trim(800.0, 9.0)["trim_deg"], 0),
    ("trim 16 m/s", r"16 m/s: ([\d.]+)°", af.trim(800.0, 16.0)["trim_deg"], 0),
    ("disk loading", r"disk loading W/\(4A\)\s+(\d+) N/m²", H["disk_loading_Nm2"], 0),
    ("ISA 975 m", r"975 m ([\d.]+),", af.isa_density(975), 0),
    ("ISA 1655 m", r"1655 m \(Boulder\) ([\d.]+) kg/m³", af.isa_density(1655), 0),
    ("quasi-iso E", r"3/8 \(135\) \+ 5/8 \(9\) ≈ (\d+) GPa", C["E_quasi_iso_GPa"], 0),
    ("E/rho CFRP", r"specific stiffness E/ρ: CFRP ≈ (\d+),", C["specific_E_cfrp"], 0),
    ("E/rho Al", r"aluminium ≈ (\d+) MN·m/kg", C["specific_E_al"], 0),
    ("arm I", r"I = b h³ / 12\s+= (\d+) mm⁴", A["I_mm4"], 0),
    ("arm deflection", r"δ = F L³ / 3EI\s+= ([\d.]+) mm", A["tip_deflection_mm"], 0),
    ("arm stress", r"σ = F L \(h/2\) / I\s+= (\d+) MPa", A["root_stress_MPa"], 0),
    ("arm first mode", r"/ 2π = (\d+) Hz", A["first_mode_Hz"], 0),
    # section 5.1: hover command vs thrust-curve setting, and the lift-off stick position
    ("hover cmd, THR_MDL_FAC 0", r"so the derivation predicts \*\*([\d.]+)\*\*", af.hover_command(7.5, 0.0), 0),
    ("hover cmd, THR_MDL_FAC 0.3", r"same hover becomes \*\*([\d.]+)\*\*", af.hover_command(7.5, 0.3), 0),
    ("hover cmd, 2.02 kg motors", r"instead of 1\.5 gives ([\d.]+) \(and", af.hover_command(8080 / 800, 0.0), 0),
    ("hover cmd, 2.02 kg + curve", r"\(and ([\d.]+) with the 0\.3 curve\)", af.hover_command(8080 / 800, 0.3), 0),
    ("hover cmd, 950 g", r"950 g gives\s+([\d.]+)\.", af.hover_command(6000 / 950, 0.0), 0),
    ("hover cmd, uncalibrated ESC", r"\(([\d.]+) instead of 0\.365", af.hover_command(7.5, 0.0, 0.1, 0.9), 0),
    ("60 % stick thrust ratio", r"\(0\.60 / 0\.365\)² ≈ ([\d.]+) times", (0.60 / 0.365) ** 2, 0),
    ("lift-off stick %", r"about \*\*(\d+) % stick\*\*", af.stick_for_command(af.hover_command(7.5), 0.25) * 100, 0),
    ("lift-off stick % low", r"\((\d+) % to \d+ % across", af.stick_for_command(af.hover_command(8080 / 800), 0.25) * 100, 0),
    ("lift-off stick % high", r"\(\d+ % to (\d+) % across", af.stick_for_command(af.hover_command(6000 / 950), 0.25) * 100, 0),
]


@pytest.mark.parametrize("label,pattern,derived,slack", CHECKS, ids=[c[0] for c in CHECKS])
def test_doc_figure_matches_derivation(label, pattern, derived, slack):
    m = re.search(pattern, DOC)
    assert m, f"{label}: pattern not found in docs/airframe_numbers.md -- the doc changed; update this test"
    printed = _num(m.group(1))
    tol = _half_unit(m.group(1)) + slack * abs(derived) + 1e-9
    assert abs(printed - derived) <= tol, (
        f"{label}: doc prints {m.group(1)} but the derivation gives {derived:.5g} (tolerance {tol:.3g})")


def test_weight_table_adds_up():
    """The section 3 rows must sum to the printed Dry and to the script's roll-up."""
    block = DOC.split("## 3. Weight budget (interceptor)")[1].split("| **Dry**")[0]
    rows = re.findall(r"^\| [^|*]+ \| ([\d.]+) \| \w+ \|$", block, flags=re.M)
    assert len(rows) == len(af.INTERCEPTOR_PARTS) == 14, f"expected 14 part rows, parsed {len(rows)}"
    total = sum(float(r) for r in rows)
    printed_dry = float(re.search(r"\| \*\*Dry\*\* \| \*\*([\d.]+)\*\*", DOC).group(1))
    assert total == pytest.approx(printed_dry) == pytest.approx(af.dry_mass_g())


def test_placard_and_weight_scaling_claims():
    """'+10 % weight costs about +15 % power' and the placard's 25-35 % flight-time cost."""
    assert 1.10 ** 1.5 - 1 == pytest.approx(0.15, abs=0.01)
    ratio = 803.5 / 610.5
    assert ratio - 1 == pytest.approx(0.32, abs=0.005)
    time_loss = 1 - ratio ** -1.5
    assert 0.25 <= time_loss <= 0.35
    assert 1 - af.isa_density(1655) / af.RHO_SL == pytest.approx(0.15, abs=0.005)
    assert math.sqrt(af.RHO_SL / af.isa_density(1655)) - 1 == pytest.approx(0.08, abs=0.005)


def test_pod_mass_row_is_near_the_cad_report():
    """The pod row quotes hardware/avionics_pod/mass_report.json. The pod design is still
    changing, so this is a loose check (3 g): it catches a redesign that makes the row
    stale, not gram-level drift."""
    rep = json.loads((ROOT / "hardware" / "avionics_pod" / "mass_report.json").read_text())["printed_g"]
    m = re.search(r"Avionics pod, printed parts .*?\| ≈ \+(\d+) \|", DOC)
    assert m, "pod mass row not found in docs/airframe_numbers.md"
    assert float(m.group(1)) == pytest.approx(rep["dock"] + rep["pod_base"] + rep["pod_lid"], abs=3.0)
