"""The figures quoted in docs/airframe_numbers.md must stay what the derivation
script produces. If an input changes (a scale weight, a thrust-rig reading), update
the script, re-run it, and update the doc and these expected values together."""
import importlib.util
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location("airframe_numbers", ROOT / "scripts" / "airframe_numbers.py")
af = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(af)


def test_rollup_is_about_800_g():
    assert af.dry_mass_g() == pytest.approx(546.5)
    assert af.dry_mass_g() + af.PACK_MASS_G == pytest.approx(800.5)


def test_hover_and_tw_at_800_g():
    h = af.hover(800.0)
    assert h["tw"] == pytest.approx(7.5)
    assert h["hover_current_A"] == pytest.approx(7.5, abs=0.05)
    assert h["hover_time_min"] == pytest.approx(9.6, abs=0.05)
    assert h["hover_throttle"] == pytest.approx(math.sqrt(1 / 7.5))
    assert h["induced_velocity_ms"] == pytest.approx(7.95, abs=0.01)


def test_momentum_theory_identity():
    # P = T v_i must equal T^1.5 / sqrt(2 rho A) per rotor
    h = af.hover(800.0)
    t = h["thrust_per_rotor_N"]
    assert h["p_ideal_W"] == pytest.approx(4 * t ** 1.5 / math.sqrt(2 * af.RHO_SL * af.disk_area()))


def test_density_scales_thrust_and_power():
    sl, hi = af.hover(800.0), af.hover(800.0, rho=af.isa_density(1655))
    assert af.isa_density(1655) == pytest.approx(1.042, abs=0.002)
    assert hi["tw"] / sl["tw"] == pytest.approx(af.isa_density(1655) / af.RHO_SL)
    assert hi["p_ideal_W"] / sl["p_ideal_W"] == pytest.approx(math.sqrt(af.RHO_SL / af.isa_density(1655)))


def test_battery_motor_carbon():
    b = af.battery()
    assert (b["nominal_V"], b["full_V"]) == pytest.approx((22.2, 25.2))
    assert b["energy_Wh"] == pytest.approx(33.3)
    assert af.motor()["kt_NmA"] == pytest.approx(0.00503, abs=1e-5)
    assert af.carbon()["E_quasi_iso_GPa"] == pytest.approx(56.25)


def test_trim_pitch():
    assert af.trim(800.0, 9.0)["trim_deg"] == pytest.approx(5.1, abs=0.05)
    assert af.trim(800.0, 16.0)["trim_deg"] == pytest.approx(15.6, abs=0.05)
