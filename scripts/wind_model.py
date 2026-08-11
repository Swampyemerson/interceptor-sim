#!/usr/bin/env python3
"""wind_model.py -- the WIND + GUST disturbance model for the sim wind arm.

STATUS: PURE MODEL, NOT WIRED. This module boots no sim, imports no
gz-transport, reads no `gt_*`, and touches no flight code. It is the physics
half of the mitigation for the project-state assumption `zero-wind-environment`
("Wind/gust is unmodelled everywhere -- it pushes the open-loop dash off its
aim line AND rotates the airframe, moving where the camera wedge points").
The gz-transport process that APPLIES these forces (`scripts/wind_driver.py`,
a world-side disturbance in the mould of `scripts/m4_target_mover.py`) is a
separate build and imports this module; nothing here knows Gazebo exists.

WHY IT IS NOT IN `flight/`
--------------------------
`flight/` is the portable core that runs IDENTICALLY on the real Pixhawk/Pi
interceptor. Wind is a SIMULATOR-SIDE disturbance -- on the real vehicle the
weather supplies it. Putting a wind generator in `flight/` would put a thing
the real aircraft does not run inside the package whose whole point is that it
does. So it lives in `scripts/` beside the other pure, importable, sim-side
models (`scripts/ekf_tracker.py`, `scripts/stereo_model.py`).

HONESTY BOUNDARY (CLAUDE.md standing rule) -- READ THIS BEFORE WIRING
---------------------------------------------------------------------
Everything this module produces is on the `gt_*` side of the boundary:
SCORING / LOGGING / world-physics ONLY. The true wind vector must NEVER reach
guidance. The vehicle has no pitot, no airspeed sensor, and the datalink is
dead by the `no-datalink` constraint, so there is NO in-flight wind
measurement and therefore NO in-flight wind correction to build. Specifically
REFUSED (do not "helpfully" add them later):
  * feeding `WindSample.wind_n/wind_e` into the lead solve or the terminal law;
  * PX4 EKF2 drag-fusion wind estimate (EKF2_DRAG_CTRL) in SITL -- it would be
    tuned against this module's own drag law, i.e. a given-perfect input
    wearing a measured costume;
  * `resetWindToExternalObservation` / MAVSDK `telemetry.wind()`.
The only legitimate wind input to the vehicle is a PRE-FLIGHT operator reading
off a handheld anemometer (the same architectural category as the latched GPS
launch cue). That path is deliberately NOT implemented here: the designated
producer is `scripts/wind_anemometer_sim.py`, which is the ONE file allowed to
read the sim's true wind, adds instrument error, and writes a reading carrying
only what a real instrument would show; `scripts/wind_trim.py` consumes it.
There is intentionally only ONE noisy-anemometer implementation in the repo --
two of them would be two different rulers for the same quantity. If you find
yourself wanting `conditions.mean_mps` on the vehicle side, that is the
given-perfect loophole the contradiction ledger exists to close.

THE MODEL (two layers, both bench-measurable -- builder mandate)
---------------------------------------------------------------
1. STEADY wind: a horizontal vector, magnitude `mean_mps` from a handheld
   anemometer's 1-minute mean at launch height, direction `dir_from_deg` in the
   METEOROLOGICAL convention (degrees from true north that the wind blows
   FROM -- what a wind sock and a compass give you). Vertical component is
   DECLARED ZERO (`wind_d = 0.0`, always present in the sample so no consumer
   can mistake "absent" for "modelled").
2. GUST: the standard first-order Gauss-Markov (Ornstein-Uhlenbeck) discrete
   approximation to the Dryden low-altitude turbulence spectrum, MIL-F-8785C
   §3.7.2 low-altitude form, horizontal components only:
       sigma_u = sigma_v = 0.1 * V_mean / (0.177 + 0.000823*h_ft)^0.4
       L_u     = L_v     = h_ft / (0.177 + 0.000823*h_ft)^1.2
       tau     = L_u / V_mean
   stepped on a FIXED SIM-TIME GRID as
       g[k+1] = a*g[k] + sigma*sqrt(1 - a^2)*xi,   a = exp(-dt/tau), xi~N(0,1)
   which is stationary with std `sigma` and lag-1 autocorrelation `a`.
   `g_u` is along the mean flow, `g_v` is 90 deg to its right; both are
   rotated into (north, east).

   DECLARED APPROXIMATION: true Dryden makes the CROSS-wind component v a
   SECOND-order (-40 dB/dec) spectrum; a first-order Gauss-Markov gives it
   -20 dB/dec, so this model carries slightly MORE high-frequency cross-gust
   energy than true Dryden. For a disturbance arm that errs toward "worse than
   ideal" (CLAUDE.md), that is the safe direction. It is an approximation, not
   an accident -- do not quote this as "Dryden" without the qualifier.

3. FORCE on the airframe: PX4's own multicopter drag form (the one EKF2 uses
   for wind estimation, `src/modules/ekf2/params_drag.yaml`), driven by the
   RELATIVE AIR VELOCITY dv = v_wind - v_vehicle:
       a_drag = MCOEF*|dv| + rho*|dv|^2 / (2*BCOEF)      [m/s^2]
       F      = mass * a_drag * dv_hat                    [N]
   The linear term is propeller momentum drag (MCOEF, 1/s); the quadratic term
   is bluff-body drag (BCOEF, kg/m^2, = m/(Cd*A)). Using PX4's own form means
   the sim's disturbance and PX4's internal drag notion are at least the same
   shape.

WHERE IT ACTS -- BOTH HALVES OF THE ASSUMPTION
-----------------------------------------------
  * TRANSLATION: the force perturbs the open-loop dash. PX4's GPS/EKF velocity
    loop rejects most of the STEADY part in ground track (at the cost of tilt);
    gusts above the loop bandwidth pass through as track/CPA scatter.
  * ROTATION: to hold station against a steady wind the controller must tilt
    into it by `hover_tilt_deg()` = atan(a_drag/g). That rotates the camera
    wedge, and it propagates to guidance with ZERO new code because the
    bearing derotation already uses the full attitude quaternion and Gazebo
    renders from the true tilted pose. This is the half the assumption says
    nobody has ever tested.

SIM TIME, NEVER WALL TIME (ADR-0009)
------------------------------------
Every `t` here is SIM seconds. The gust realisation is advanced on a FIXED
sim-time grid (`sim_step_s`), so it is a pure function of (seed, sim_step_s,
grid index) and is INVARIANT to the caller's cadence jitter and to RTF sag.
That is what makes paired-seed A/B arms actually paired: two arms that sample
the same sim-time grid with the same seed see the SAME gusts, even if one arm
runs at RTF 0.9 and the other at 0.4. Time going backwards raises.

FAIL CLOSED (docs/error_handling_policy.md)
-------------------------------------------
Every quantity that a real session would MEASURE is a required argument with
no default: mean speed, direction, height, seed, sim step, mass, drag
coefficients, and a free-text `provenance` string saying where each number
came from. Omitting one raises `MissingMeasurementError` whose message states
the instrument and the units. `DragParams.px4_x500_mono_cam_sitl()` exists as
an explicit, self-labelling constructor for the SITL airframe -- the numbers
are PX4 defaults and the SDF mass, NOT measurements of the real interceptor,
and the provenance string says so in every log line that carries it.

TIERS (CLAUDE.md "simulate worse than ideal"; sources in `TIERS`)
-----------------------------------------------------------------
  BEST      1.5 m/s  top of Beaufort 1 "light air"    -> ~2.1 m/s peaks
  EXPECTED  4.5 m/s  mid Beaufort 3 "gentle breeze"   -> ~6.2 m/s peaks
  WORST     8.0 m/s  bottom of Beaufort 5 "fresh brz" -> ~11.1 m/s peaks
(peaks quoted at mean + 2*sigma with the h=6 m Dryden sigma coefficient
0.1930, i.e. a gust factor of 1.386 -- consistent with the WMO/Wieringa
open-terrain gust factor of ~1.4.)

USAGE
  .venv/bin/python scripts/wind_model.py --self-test
  .venv/bin/python scripts/wind_model.py --tiers --height 6.0
"""
from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import asdict, dataclass
from typing import Optional, Tuple

# --- physical constants (traceable) -----------------------------------------
G_M_S2 = 9.80665            # standard gravity, m/s^2 (CODATA/ISO 80000)
RHO_SEA_LEVEL = 1.225       # ISA sea-level air density, kg/m^3
M_PER_FT = 0.3048           # exact, by definition

# MIL-F-8785C low-altitude turbulence form is SPECIFIED for altitudes at and
# above 10 ft (3.048 m). Below that it still evaluates, but it is an
# extrapolation into the surface layer where real shear/obstacle wakes
# dominate -- so callers must acknowledge it explicitly.
DRYDEN_MIN_HEIGHT_M = 10.0 * M_PER_FT   # 3.048 m

# PX4 multicopter drag model defaults, src/modules/ekf2/params_drag.yaml
# (verified in-tree 2026-08-10): EKF2_MCOEF default 0.15 [1/s],
# EKF2_BCOEF_X/Y default 100.0 [kg/m^2].
PX4_MCOEF_DEFAULT = 0.15
PX4_BCOEF_DEFAULT = 100.0

# gz_x500_mono_cam SITL mass: sum of <mass> in
# PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf
# (2.0 body + 4 x 0.016076923 rotors = 2.064307692) plus mono_cam/model.sdf
# (0.050). Verified in-tree 2026-08-10.
X500_MONO_CAM_MASS_KG = 2.0 + 4 * 0.016076923076923075 + 0.050   # 2.114307692


class MissingMeasurementError(ValueError):
    """A quantity that a real session MEASURES was not supplied (or was NaN).

    Never substitute a default for a measured quantity (CLAUDE.md / error
    handling policy: fail closed) -- the invented value is exactly how a
    number that was never measured ends up quoted as a result.
    """


def _require(value, name: str, units: str, how_measured: str) -> float:
    """Fail-closed float coercion for a MEASURED quantity."""
    if value is None:
        raise MissingMeasurementError(
            f"{name} is required and has no default (units: {units}). "
            f"How to measure it: {how_measured}")
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise MissingMeasurementError(
            f"{name} must be a number, got {value!r} (units: {units}). "
            f"How to measure it: {how_measured}")
    if math.isnan(v) or math.isinf(v):
        raise MissingMeasurementError(
            f"{name} is {v} -- not a measurement (units: {units}). "
            f"How to measure it: {how_measured}")
    return v


def _require_provenance(text, what: str) -> str:
    if not isinstance(text, str) or not text.strip():
        raise MissingMeasurementError(
            f"provenance for {what} is required: a non-empty string saying "
            f"where these numbers came from (a run, a datasheet, an SDF, a "
            f"field session). Numbers trace to a run or a derivation.")
    return text.strip()


# ============================================================ Dryden geometry
def _dryden_denom(height_m: float) -> Tuple[float, float]:
    """(height in ft, the (0.177 + 0.000823*h_ft) group). h_ft is the MIL form's
    native unit -- the conversion is done here, once, on purpose."""
    h_ft = height_m / M_PER_FT
    return h_ft, 0.177 + 0.000823 * h_ft


def dryden_sigma_mps(mean_mps: float, height_m: float,
                     allow_below_10ft: bool = False) -> float:
    """Horizontal gust standard deviation [m/s], MIL-F-8785C low altitude.

        sigma_u = sigma_v = 0.1 * V_mean / (0.177 + 0.000823*h_ft)^0.4

    mean_mps  : 1-minute mean wind at `height_m` [m/s] -- handheld anemometer.
    height_m  : height above ground the wind is quoted/flown at [m] -- tape
                measure / the flight's own altitude reference.
    Bench cross-check: the anemometer's MAX-HOLD over the same minute gives a
    gust factor GF = peak/mean; `sigma_from_gust_factor` converts a MEASURED GF
    into sigma and should be preferred over this derivation when you have one.
    """
    v = _require(mean_mps, "mean_mps", "m/s",
                 "handheld anemometer, 1-minute mean at launch height")
    h = _require(height_m, "height_m", "m",
                 "tape measure to the anemometer / the flight altitude AGL")
    _check_height(h, allow_below_10ft)
    if v < 0.0:
        raise MissingMeasurementError("mean_mps must be >= 0 (it is a speed)")
    _, d = _dryden_denom(h)
    return 0.1 * v / d ** 0.4


def dryden_length_scale_m(height_m: float,
                          allow_below_10ft: bool = False) -> float:
    """Horizontal turbulence length scale L_u = L_v [m], MIL-F-8785C low
    altitude:  L_u = h_ft / (0.177 + 0.000823*h_ft)^1.2 , converted to metres.

    Not directly measurable with a handheld instrument; it is the DERIVED
    quantity that sets how long a gust lasts (`dryden_tau_s`). The measurable
    proxy is the timed period between gust peaks in an anemometer log.
    """
    h = _require(height_m, "height_m", "m",
                 "tape measure to the anemometer / the flight altitude AGL")
    _check_height(h, allow_below_10ft)
    h_ft, d = _dryden_denom(h)
    return (h_ft / d ** 1.2) * M_PER_FT


def dryden_tau_s(mean_mps: float, height_m: float,
                 allow_below_10ft: bool = False) -> float:
    """Gust correlation time tau = L_u / V_mean [SIM seconds].

    FAILS CLOSED at V_mean = 0: a still-air condition has no Dryden timescale.
    Build that arm with `WindConditions.still_air()`, which is explicit about
    being the drag-only control rather than "wind with a zero that fell out of
    a formula".
    """
    v = _require(mean_mps, "mean_mps", "m/s",
                 "handheld anemometer, 1-minute mean at launch height")
    if v <= 0.0:
        raise MissingMeasurementError(
            "dryden_tau_s is undefined at mean_mps <= 0 (tau = L/V). For the "
            "drag-only control arm use WindConditions.still_air().")
    return dryden_length_scale_m(height_m, allow_below_10ft) / v


def _check_height(h: float, allow_below_10ft: bool) -> None:
    if h <= 0.0:
        raise MissingMeasurementError(
            "height_m must be > 0 (height above ground level, metres)")
    if h < DRYDEN_MIN_HEIGHT_M and not allow_below_10ft:
        raise MissingMeasurementError(
            f"height_m={h:.3f} m is below the MIL-F-8785C low-altitude form's "
            f"stated floor of 10 ft ({DRYDEN_MIN_HEIGHT_M:.3f} m). The formula "
            f"still evaluates, but tau shrinks fast down there (h=6 m -> "
            f"tau=L/V with L=43.1 m; h=0.5 m -> L=3.96 m, a ~11x shorter "
            f"gust), which changes whether a gust looks like a bias or like "
            f"scatter inside a ~2 s engagement. If you mean it, pass "
            f"allow_below_10ft=True and say so in the arm's provenance.")


def sigma_from_gust_factor(mean_mps: float, gust_factor: float,
                           n_sigma: float = 2.0) -> float:
    """Convert a MEASURED gust factor into the gust sigma [m/s].

    gust_factor : GF = (anemometer MAX-HOLD peak) / (1-minute mean), unitless.
                  Open-terrain reference value ~1.4 (WMO / Wieringa).
    n_sigma     : how many sigma the max-hold peak is taken to represent.
                  2.0 by convention here; state it wherever you quote sigma.
    Derivation:  peak = mean * GF = mean + n_sigma*sigma  =>
                 sigma = mean*(GF - 1)/n_sigma.
    Prefer this over `dryden_sigma_mps` whenever a real GF has been measured --
    it replaces a textbook coefficient with a field number.
    """
    v = _require(mean_mps, "mean_mps", "m/s",
                 "handheld anemometer, 1-minute mean at launch height")
    gf = _require(gust_factor, "gust_factor", "unitless (peak/mean)",
                  "anemometer MAX-HOLD peak divided by the 1-minute mean over "
                  "the same minute")
    if gf < 1.0:
        raise MissingMeasurementError(
            f"gust_factor={gf} < 1: a peak below the mean is not a gust "
            "factor (check which reading is which)")
    if n_sigma <= 0.0:
        raise MissingMeasurementError("n_sigma must be > 0")
    return v * (gf - 1.0) / n_sigma


# ================================================================ drag params
@dataclass(frozen=True)
class DragParams:
    """The airframe's drag law -- PX4's multicopter form.

    mass_kg      : total vehicle mass [kg]. Bench: kitchen scale, flight-ready
                   (battery + payload fitted).
    mcoef_per_s  : propeller momentum drag coefficient [1/s] (PX4 EKF2_MCOEF).
                   Bench: hover in a MEASURED steady wind V and read the
                   settled tilt theta from the ULog `vehicle_attitude`; if the
                   quadratic term is small, k = g*tan(theta)/V.
    bcoef_kg_m2  : ballistic coefficient [kg/m^2] = mass/(Cd*A) (PX4
                   EKF2_BCOEF_X/Y). Bench: frontal area * drag coefficient, or
                   a coast-down fit from a ULog. PX4's sentinel 0.0 means
                   "bluff-body term OFF" and is honoured here.
    rho_kg_m3    : air density [kg/m^3]. Bench: 1.225 ISA, or compute from a
                   field thermometer + barometer if you want the real day.
    provenance   : REQUIRED free text -- where these four numbers came from.

    All four are quantities a real session measures, so none has a default;
    `px4_x500_mono_cam_sitl()` is the one explicit, self-labelling shortcut.
    """

    mass_kg: float
    mcoef_per_s: float
    bcoef_kg_m2: float
    rho_kg_m3: float
    provenance: str

    def __post_init__(self):
        m = _require(self.mass_kg, "mass_kg", "kg",
                     "kitchen scale, flight-ready (battery + payload fitted)")
        mc = _require(self.mcoef_per_s, "mcoef_per_s", "1/s",
                      "hover in measured steady wind V, k = g*tan(theta)/V "
                      "from the ULog attitude")
        bc = _require(self.bcoef_kg_m2, "bcoef_kg_m2", "kg/m^2",
                      "mass/(Cd*A) from frontal area, or a ULog coast-down fit")
        rho = _require(self.rho_kg_m3, "rho_kg_m3", "kg/m^3",
                       "1.225 (ISA sea level), or field thermometer+barometer")
        if m <= 0.0:
            raise MissingMeasurementError("mass_kg must be > 0")
        if mc < 0.0:
            raise MissingMeasurementError("mcoef_per_s must be >= 0")
        if bc < 0.0:
            raise MissingMeasurementError(
                "bcoef_kg_m2 must be >= 0 (0.0 = PX4's 'bluff-body drag OFF')")
        if rho <= 0.0:
            raise MissingMeasurementError("rho_kg_m3 must be > 0")
        object.__setattr__(self, "provenance",
                           _require_provenance(self.provenance, "DragParams"))

    @classmethod
    def px4_x500_mono_cam_sitl(cls) -> "DragParams":
        """The SITL airframe. NOT measurements of the real interceptor.

        These are PX4's PARAMETER DEFAULTS plus the model SDF mass, which is
        the right choice for a Gazebo disturbance arm (the sim's drag and
        PX4's internal drag notion then share a shape) and the WRONG basis for
        any sim-to-field claim about the real 5-inch airframe. The provenance
        string carries that warning into every log line.
        """
        return cls(
            mass_kg=X500_MONO_CAM_MASS_KG,
            mcoef_per_s=PX4_MCOEF_DEFAULT,
            bcoef_kg_m2=PX4_BCOEF_DEFAULT,
            rho_kg_m3=RHO_SEA_LEVEL,
            provenance=(
                "PX4 defaults + SDF mass, SITL ONLY: EKF2_MCOEF=0.15 1/s and "
                "EKF2_BCOEF_X/Y=100 kg/m^2 from src/modules/ekf2/"
                "params_drag.yaml; mass 2.114308 kg = x500_base/model.sdf "
                "(2.0 + 4*0.016076923) + mono_cam/model.sdf (0.050); "
                "rho 1.225 ISA. NOT measured on the real airframe -- do not "
                "quote a field number off these."))

    # -- the law ------------------------------------------------------------
    def drag_accel_m_s2(self, v_rel_mps: float) -> float:
        """a_drag = MCOEF*|dv| + rho*|dv|^2/(2*BCOEF)  [m/s^2], |dv| = relative
        air speed. Both terms are per unit mass, so this is what the airframe
        actually feels regardless of mass."""
        v = abs(_require(v_rel_mps, "v_rel_mps", "m/s",
                         "|v_wind - v_vehicle|, from the wind model and the "
                         "vehicle's own velocity"))
        bluff = 0.0 if self.bcoef_kg_m2 == 0.0 else \
            self.rho_kg_m3 * v * v / (2.0 * self.bcoef_kg_m2)
        return self.mcoef_per_s * v + bluff

    def drag_force_n(self, v_rel_mps: float) -> float:
        """|F| = mass * a_drag  [N]."""
        return self.mass_kg * self.drag_accel_m_s2(v_rel_mps)

    def hover_tilt_deg(self, v_rel_mps: float) -> float:
        """Steady tilt into the wind needed to hold station [deg].

        T*sin(theta) = m*a_drag and T*cos(theta) = m*g  =>  theta =
        atan(a_drag/g). This is the ROTATION half of the zero-wind assumption:
        the number of degrees the camera wedge is turned by a steady wind, and
        the Gate-0 mechanism-probe prediction.
        """
        return math.degrees(math.atan(self.drag_accel_m_s2(v_rel_mps) / G_M_S2))


# ============================================================== wind condition
@dataclass(frozen=True)
class WindConditions:
    """One session's measured wind, plus the derived gust parameters.

    mean_mps      : 1-minute mean wind speed [m/s] at `height_m`. Anemometer.
    dir_from_deg  : METEOROLOGICAL direction [deg from true north] the wind
                    blows FROM -- what a wind sock + compass give. 0 = a
                    northerly (air moving toward the south). Wind sock ±15 deg.
    height_m      : height AGL the wind is quoted at [m]. Tape measure.
    sigma_mps     : per-axis gust standard deviation [m/s]. Either derived
                    (Dryden) or measured (`sigma_from_gust_factor`).
    tau_s         : gust correlation time [SIM seconds] = L_u/V_mean. The
                    measurable proxy is the period between gust peaks.
    label         : short arm name for logs ("EXPECTED", "still-air-control").
    provenance    : REQUIRED -- where these numbers came from.

    Vertical wind is DECLARED ZERO (see module docstring); there is no w_z
    field to set, and `WindSample.wind_d` is always 0.0 so the omission is
    visible rather than silent.
    """

    mean_mps: float
    dir_from_deg: float
    height_m: float
    sigma_mps: float
    tau_s: float
    label: str
    provenance: str

    def __post_init__(self):
        v = _require(self.mean_mps, "mean_mps", "m/s",
                     "handheld anemometer, 1-minute mean at launch height")
        d = _require(self.dir_from_deg, "dir_from_deg", "deg from true N",
                     "wind sock + compass at launch, direction the wind blows "
                     "FROM, +/-15 deg")
        h = _require(self.height_m, "height_m", "m",
                     "tape measure to the anemometer / flight altitude AGL")
        s = _require(self.sigma_mps, "sigma_mps", "m/s",
                     "anemometer max-hold gust factor (preferred), else the "
                     "MIL-F-8785C Dryden derivation")
        t = _require(self.tau_s, "tau_s", "s (SIM seconds)",
                     "timed period between gust peaks in an anemometer log, "
                     "else L_u/V_mean")
        if v < 0.0:
            raise MissingMeasurementError("mean_mps must be >= 0")
        if h <= 0.0:
            raise MissingMeasurementError("height_m must be > 0")
        if s < 0.0:
            raise MissingMeasurementError("sigma_mps must be >= 0")
        if t <= 0.0:
            raise MissingMeasurementError(
                "tau_s must be > 0 (a zero correlation time is white noise, "
                "not a gust; use sigma_mps=0 for no gusts)")
        if not isinstance(self.label, str) or not self.label.strip():
            raise MissingMeasurementError("label is required (arm name for logs)")
        object.__setattr__(self, "dir_from_deg", d % 360.0)
        object.__setattr__(self, "provenance",
                           _require_provenance(self.provenance, "WindConditions"))

    # -- geometry -----------------------------------------------------------
    @property
    def flow_heading_deg(self) -> float:
        """Direction the air MOVES TOWARD [deg from true N] = dir_from + 180.

        The met convention trips everyone exactly once; it is converted here,
        in one place, and tested in both directions.
        """
        return (self.dir_from_deg + 180.0) % 360.0

    @property
    def steady_ne(self) -> Tuple[float, float]:
        """Steady wind as a FLOW vector (north, east) [m/s]."""
        psi = math.radians(self.flow_heading_deg)
        return self.mean_mps * math.cos(psi), self.mean_mps * math.sin(psi)

    def peak_mps(self, n_sigma: float = 2.0) -> float:
        """Quotable gust peak = mean + n_sigma*sigma [m/s]. State n_sigma."""
        return self.mean_mps + n_sigma * self.sigma_mps

    def gust_factor(self, n_sigma: float = 2.0) -> Optional[float]:
        """peak/mean, the anemometer-comparable number (None in still air)."""
        if self.mean_mps <= 0.0:
            return None
        return self.peak_mps(n_sigma) / self.mean_mps

    def to_dict(self) -> dict:
        return asdict(self)

    # -- constructors -------------------------------------------------------
    @classmethod
    def from_anemometer(cls, mean_mps, dir_from_deg, height_m, provenance,
                        label="anemometer", gust_factor=None, n_sigma=2.0,
                        tau_s=None, allow_below_10ft=False) -> "WindConditions":
        """Build from what a field session actually records.

        gust_factor : MEASURED peak/mean if you have it (preferred). If None,
                      sigma is DERIVED from the MIL-F-8785C Dryden form -- say
                      "derived" in the provenance when you do that.
        tau_s       : MEASURED gust period if you have it; else derived L_u/V.
        """
        if gust_factor is None:
            sigma = dryden_sigma_mps(mean_mps, height_m, allow_below_10ft)
        else:
            sigma = sigma_from_gust_factor(mean_mps, gust_factor, n_sigma)
        tau = dryden_tau_s(mean_mps, height_m, allow_below_10ft) \
            if tau_s is None else tau_s
        return cls(mean_mps=mean_mps, dir_from_deg=dir_from_deg,
                   height_m=height_m, sigma_mps=sigma, tau_s=tau,
                   label=label, provenance=provenance)

    @classmethod
    def still_air(cls, height_m, provenance, label="still-air-control") -> "WindConditions":
        """The Gate-1 DRAG-ONLY CONTROL: driver running, wind identically zero.

        This is not "wind with the numbers turned down" -- it is the mandatory
        control arm. Turning wind on also turns DRAG on, so a wind arm must be
        compared against THIS, never against a no-driver baseline; otherwise
        the two effects are conflated (the arm-asymmetric defect class).
        `tau_s` is set to 1.0 s and is unused because `sigma_mps` is 0.
        """
        return cls(mean_mps=0.0, dir_from_deg=0.0, height_m=height_m,
                   sigma_mps=0.0, tau_s=1.0, label=label, provenance=provenance)


# ---------------------------------------------------------------- the tiers
# CLAUDE.md: BEST / EXPECTED / WORST-credible; decisions must survive WORST.
# Speeds are Beaufort-scale anchors (WMO), NOT tuned numbers.
TIERS = {
    "BEST": (1.5, "top of Beaufort 1 'light air' (0.3-1.5 m/s): a calm-morning "
                  "session, the day you would choose"),
    "EXPECTED": (4.5, "mid Beaufort 3 'gentle breeze' (3.4-5.4 m/s): leaves in "
                      "constant motion. PX4's own reference windy.sdf uses "
                      "|(5,2,0)| = 5.39 m/s, an adjacent anchor"),
    "WORST": (8.0, "bottom of Beaufort 5 'fresh breeze' (8.0-10.7 m/s): small "
                   "trees sway. Consumer sub-250 g drones rate max operating "
                   "wind ~10.7 m/s, so 8 mean / ~11 gust is the worst day "
                   "anyone would credibly attempt"),
}


def tier(name: str, height_m, dir_from_deg, provenance=None,
         allow_below_10ft: bool = False) -> WindConditions:
    """A tiered WindConditions. `height_m` and `dir_from_deg` have NO defaults:
    they are properties of the site and the session, not of the tier."""
    key = str(name).upper()
    if key not in TIERS:
        raise MissingMeasurementError(
            f"unknown tier {name!r}; choose from {sorted(TIERS)}")
    mean, why = TIERS[key]
    prov = provenance or (
        f"tier {key}: {mean} m/s = {why}. sigma and tau DERIVED from the "
        f"MIL-F-8785C low-altitude Dryden form at h={float(height_m)} m "
        f"(not measured in the field).")
    return WindConditions.from_anemometer(
        mean_mps=mean, dir_from_deg=dir_from_deg, height_m=height_m,
        provenance=prov, label=key, allow_below_10ft=allow_below_10ft)


# ================================================================ the samples
@dataclass(frozen=True)
class WindSample:
    """The wind vector at one sim-time grid point. SCORING/PHYSICS ONLY."""

    t_sim_s: float          # sim seconds the caller asked for
    grid_index: int         # k = floor(t_sim / sim_step_s); the realisation index
    grid_t_sim_s: float     # k * sim_step_s -- when the gust actually stepped
    steady_n: float         # steady flow, north [m/s]
    steady_e: float         # steady flow, east  [m/s]
    gust_u: float           # gust along the mean flow [m/s]
    gust_v: float           # gust 90 deg right of the mean flow [m/s]
    wind_n: float           # total flow, north [m/s]
    wind_e: float           # total flow, east  [m/s]
    wind_d: float = 0.0     # DECLARED ZERO -- vertical gust is not modelled

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.wind_n, self.wind_e)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["speed_mps"] = self.speed_mps
        return d


@dataclass(frozen=True)
class WindForce:
    """The wrench the driver should apply to the vehicle's base_link."""

    f_n: float              # force, north [N]
    f_e: float              # force, east  [N]
    f_d: float = 0.0        # DECLARED ZERO (no vertical wind component)
    v_rel_n: float = 0.0    # relative air velocity, north [m/s]
    v_rel_e: float = 0.0    # relative air velocity, east  [m/s]
    v_rel_mps: float = 0.0  # |relative air velocity| [m/s]
    a_drag_m_s2: float = 0.0  # drag acceleration magnitude [m/s^2]

    @property
    def magnitude_n(self) -> float:
        return math.hypot(self.f_n, self.f_e)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["magnitude_n"] = self.magnitude_n
        return d


# ================================================================= the field
class WindField:
    """Seeded, sim-clocked wind generator: steady vector + Gauss-Markov gusts.

    conditions  : a WindConditions (all measured/derived inputs, with provenance)
    seed        : REQUIRED int. Paired-seed A/B arms are the project's evidence
                  standard (n>=8 paired seeds); an unseeded gust makes the pair
                  meaningless, so there is no default.
    sim_step_s  : REQUIRED. The FIXED sim-time step the gust process advances
                  on (e.g. 0.05 for a 20 Hz driver). The realisation is a pure
                  function of (seed, sim_step_s, grid index), so it is
                  identical across arms regardless of RTF or call jitter.
    max_catchup_steps : refuse to silently grind through an implausible time
                  jump (a broken clock); raises instead of hanging.
    """

    def __init__(self, conditions: WindConditions, seed, sim_step_s,
                 max_catchup_steps: int = 20000):
        if not isinstance(conditions, WindConditions):
            raise MissingMeasurementError(
                "conditions must be a WindConditions (built from measured "
                "inputs with provenance)")
        if seed is None:
            raise MissingMeasurementError(
                "seed is required and has no default: paired-seed A/B arms "
                "(n>=8) are this project's evidence standard, and an unseeded "
                "gust realisation cannot be paired across arms")
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise MissingMeasurementError(
                f"seed must be an int, got {seed!r}")
        step = _require(sim_step_s, "sim_step_s", "s (SIM seconds)",
                        "the wind driver's fixed tick period, e.g. 0.05 for "
                        "20 Hz -- a CONFIG value, identical across paired arms")
        if step <= 0.0:
            raise MissingMeasurementError("sim_step_s must be > 0")
        if max_catchup_steps < 1:
            raise MissingMeasurementError("max_catchup_steps must be >= 1")
        self.conditions = conditions
        self.seed = int(seed)
        self.sim_step_s = step
        self.max_catchup_steps = int(max_catchup_steps)
        # OU decay per fixed step. sigma*sqrt(1-a^2) keeps the process
        # stationary at std = sigma from the first sample onward.
        self._a = math.exp(-self.sim_step_s / conditions.tau_s)
        self._q = conditions.sigma_mps * math.sqrt(max(0.0, 1.0 - self._a ** 2))
        self.reset()

    def reset(self) -> None:
        """Re-arm at grid index -1 with a fresh, seed-determined realisation."""
        self._rng = random.Random(self.seed)
        # Start ON the stationary distribution so there is no warm-up transient
        # that a short (~2 s) engagement would sit entirely inside.
        s = self.conditions.sigma_mps
        self._gu = self._rng.gauss(0.0, s) if s > 0.0 else 0.0
        self._gv = self._rng.gauss(0.0, s) if s > 0.0 else 0.0
        self._k = 0
        self._last_t = None

    # -- generation ---------------------------------------------------------
    def _advance_to(self, k_target: int) -> None:
        if k_target < self._k:
            raise MissingMeasurementError(
                f"sim time went BACKWARDS (grid {self._k} -> {k_target}). The "
                "gust process is advanced on the sim clock only (ADR-0009); a "
                "backwards step means the clock source is wrong, and silently "
                "re-using the old sample would hide it.")
        n = k_target - self._k
        if n > self.max_catchup_steps:
            raise MissingMeasurementError(
                f"sim time jumped {n} steps ({n * self.sim_step_s:.1f} s) in "
                f"one call, over max_catchup_steps={self.max_catchup_steps}. "
                "That is a clock fault, not a gust; failing closed.")
        s = self.conditions.sigma_mps
        for _ in range(n):
            if s > 0.0:
                self._gu = self._a * self._gu + self._q * self._rng.gauss(0.0, 1.0)
                self._gv = self._a * self._gv + self._q * self._rng.gauss(0.0, 1.0)
            self._k += 1

    def _grid_index(self, t: float) -> int:
        """floor(t / sim_step_s), made robust to binary representation error.

        BUG FOUND BY THE SELF-TEST, 2026-08-10 (kept as a comment because it is
        exactly the instruments-are-evidence class): a plain
        floor(t / sim_step_s) puts t=2.15 s at grid 42 instead of 43, because
        2.15/0.05 evaluates to 42.99999999999999 in binary floating point. That
        is a MEASUREMENT defect -- the same sim instant would land on a
        different gust value depending on how the caller phrased the time,
        silently de-pairing two arms at grid boundaries. The epsilon snaps a
        quotient that is within 1e-9 of an integer up to that integer; 1e-9 of
        a grid step is ~5e-11 s of sim time, orders of magnitude below anything
        physical, and ~200x larger than the worst float error over a 1000 s run.
        """
        return int(math.floor(t / self.sim_step_s + 1e-9))

    def sample(self, t_sim_s) -> WindSample:
        """The wind at SIM time `t_sim_s` (zero-order hold on the grid)."""
        t = _require(t_sim_s, "t_sim_s", "s (SIM seconds)",
                     "the Gazebo /clock topic -- never wall time (ADR-0009)")
        if t < 0.0:
            raise MissingMeasurementError(
                f"t_sim_s={t} is negative; sim time starts at 0")
        self._advance_to(self._grid_index(t))
        self._last_t = t
        c = self.conditions
        sn, se = c.steady_ne
        psi = math.radians(c.flow_heading_deg)
        cu, su = math.cos(psi), math.sin(psi)
        # u along the flow, v 90 deg to its right (heading psi+90).
        gn = self._gu * cu + self._gv * (-su)
        ge = self._gu * su + self._gv * cu
        return WindSample(
            t_sim_s=t, grid_index=self._k,
            grid_t_sim_s=self._k * self.sim_step_s,
            steady_n=sn, steady_e=se, gust_u=self._gu, gust_v=self._gv,
            wind_n=sn + gn, wind_e=se + ge)

    # -- the force the driver applies ---------------------------------------
    def force(self, sample: WindSample, v_veh_n, v_veh_e,
              drag: DragParams) -> WindForce:
        """Wrench on the airframe from the RELATIVE AIR VELOCITY.

        v_veh_n / v_veh_e : the vehicle's WORLD velocity (north, east) [m/s],
        which the driver gets by finite-differencing the gz pose on SIM time.
        This is world physics, not a sensor: the vehicle never sees it.
        """
        vn = _require(v_veh_n, "v_veh_n", "m/s",
                      "vehicle world velocity north, finite-differenced from "
                      "the gz pose on SIM time")
        ve = _require(v_veh_e, "v_veh_e", "m/s",
                      "vehicle world velocity east, finite-differenced from "
                      "the gz pose on SIM time")
        if not isinstance(drag, DragParams):
            raise MissingMeasurementError(
                "drag must be a DragParams (with provenance)")
        dn, de = sample.wind_n - vn, sample.wind_e - ve
        v_rel = math.hypot(dn, de)
        a = drag.drag_accel_m_s2(v_rel)
        f = drag.mass_kg * a
        if v_rel > 0.0:
            fn, fe = f * dn / v_rel, f * de / v_rel
        else:
            fn = fe = 0.0
        return WindForce(f_n=fn, f_e=fe, v_rel_n=dn, v_rel_e=de,
                         v_rel_mps=v_rel, a_drag_m_s2=a)


# ==================================================== the pre-flight cue only
# DELIBERATELY EMPTY. The noisy pre-flight anemometer reading -- the only
# sanctioned path from wind toward the vehicle -- lives in
# scripts/wind_anemometer_sim.py (the producer, the one file allowed to read
# the sim's true wind) -> scripts/wind_trim.py (the flight-side consumer),
# whose honesty partition is a machine-checkable FILE BOUNDARY. A second
# implementation here would be a second ruler for the same quantity, with
# nothing to say which arm used which. Do not add one.


# ==================================================================== self-test
def _report_tiers(height_m: float, allow_below_10ft: bool = False) -> None:
    drag = DragParams.px4_x500_mono_cam_sitl()
    print(f"=== WIND TIERS at h={height_m} m "
          f"(drag: {drag.mass_kg:.6f} kg, MCOEF {drag.mcoef_per_s}, "
          f"BCOEF {drag.bcoef_kg_m2}) ===")
    print(f"{'tier':>9} {'mean':>6} {'sigma':>6} {'tau':>7} {'peak2s':>7} "
          f"{'GF':>5} {'a_drag':>7} {'tilt':>6}")
    print(f"{'':>9} {'m/s':>6} {'m/s':>6} {'s':>7} {'m/s':>7} "
          f"{'':>5} {'m/s^2':>7} {'deg':>6}")
    for name in ("BEST", "EXPECTED", "WORST"):
        c = tier(name, height_m=height_m, dir_from_deg=270.0,
                 allow_below_10ft=allow_below_10ft)
        print(f"{name:>9} {c.mean_mps:6.2f} {c.sigma_mps:6.3f} {c.tau_s:7.3f} "
              f"{c.peak_mps():7.2f} {c.gust_factor():5.3f} "
              f"{drag.drag_accel_m_s2(c.mean_mps):7.3f} "
              f"{drag.hover_tilt_deg(c.mean_mps):6.2f}")
    print(f"  L_u = {dryden_length_scale_m(height_m, allow_below_10ft):.3f} m; "
          f"sigma coefficient = "
          f"{dryden_sigma_mps(1.0, height_m, allow_below_10ft):.4f} * V_mean")


def _self_test() -> int:
    """Dependency-light smoke test. The REAL coverage is
    tests/test_wind_model.py (pytest, collected by scripts/run_tests.sh
    stage 1); this entry point exists so the module can be sanity-checked
    without pytest and so the tier table is printable evidence."""
    fails = []

    def check(name, cond, detail=""):
        if cond:
            print(f"PASS {name}")
        else:
            fails.append(name)
            print(f"FAIL {name}: {detail}")

    drag = DragParams.px4_x500_mono_cam_sitl()

    # 1. met-convention signs, all four cardinals.
    for d_from, exp in ((0.0, (-1.0, 0.0)), (90.0, (0.0, -1.0)),
                        (180.0, (1.0, 0.0)), (270.0, (0.0, 1.0))):
        c = WindConditions(mean_mps=5.0, dir_from_deg=d_from, height_m=6.0,
                           sigma_mps=0.0, tau_s=1.0, label="sign",
                           provenance="self-test")
        n, e = c.steady_ne
        check(f"met-convention FROM {d_from:.0f} deg",
              abs(n - 5.0 * exp[0]) < 1e-9 and abs(e - 5.0 * exp[1]) < 1e-9,
              f"got ({n:.3f},{e:.3f})")

    # 2. tiers, all three, BEST/EXPECTED/WORST.
    for name, mean, tilt in (("BEST", 1.5, 1.3948), ("EXPECTED", 4.5, 4.6581),
                             ("WORST", 8.0, 9.2209)):
        c = tier(name, height_m=6.0, dir_from_deg=270.0)
        check(f"tier {name} mean/sigma/tilt",
              abs(c.mean_mps - mean) < 1e-12
              and abs(c.sigma_mps - 0.193017 * mean) < 1e-4
              and abs(drag.hover_tilt_deg(mean) - tilt) < 1e-3,
              f"mean {c.mean_mps} sigma {c.sigma_mps} "
              f"tilt {drag.hover_tilt_deg(mean)}")

    # 3. gust statistics at EXPECTED.
    c = tier("EXPECTED", height_m=6.0, dir_from_deg=270.0)
    wf = WindField(c, seed=7, sim_step_s=0.05)
    gs = [wf.sample(i * 0.05).gust_u for i in range(200000)]
    m = sum(gs) / len(gs)
    sd = math.sqrt(sum((g - m) ** 2 for g in gs) / len(gs))
    check("gust std matches sigma (200k samples)",
          abs(sd - c.sigma_mps) / c.sigma_mps < 0.05, f"{sd:.4f} vs {c.sigma_mps:.4f}")
    check("gust mean ~ 0", abs(m) < 0.1 * c.sigma_mps, f"{m:.5f}")

    # 4. determinism + cadence invariance (paired-seed requirement).
    a = [WindField(c, seed=3, sim_step_s=0.05).sample(t * 0.05).wind_n
         for t in range(50)]
    b = [WindField(c, seed=3, sim_step_s=0.05).sample(t * 0.05).wind_n
         for t in range(50)]
    check("same seed -> identical realisation", a == b)
    jit = WindField(c, seed=3, sim_step_s=0.05)
    jitter = [jit.sample(t * 0.05 + 0.049).wind_n for t in range(50)]
    check("cadence jitter does not change the realisation", jitter == a)

    # 5. fail-closed.
    for name, fn in (
            ("missing mean", lambda: WindConditions(
                mean_mps=None, dir_from_deg=0.0, height_m=6.0, sigma_mps=0.1,
                tau_s=1.0, label="x", provenance="p")),
            ("missing provenance", lambda: DragParams(
                mass_kg=2.0, mcoef_per_s=0.15, bcoef_kg_m2=100.0,
                rho_kg_m3=1.225, provenance="")),
            ("missing seed", lambda: WindField(c, seed=None, sim_step_s=0.05)),
            ("backwards sim time", lambda: (
                lambda w: (w.sample(1.0), w.sample(0.1)))(
                    WindField(c, seed=1, sim_step_s=0.05))),
            ("sub-10ft height unacknowledged",
             lambda: tier("EXPECTED", height_m=0.5, dir_from_deg=270.0)),
    ):
        try:
            fn()
            check(f"fail-closed: {name}", False, "did NOT raise")
        except MissingMeasurementError:
            check(f"fail-closed: {name}", True)

    # 6. still-air control really is still, but still drags.
    sc = WindConditions.still_air(height_m=6.0, provenance="self-test Gate-1")
    sw = WindField(sc, seed=11, sim_step_s=0.05)
    s0 = sw.sample(3.7)
    f_still = sw.force(s0, 16.0, 0.0, drag)
    check("still-air control: zero wind",
          s0.wind_n == 0.0 and s0.wind_e == 0.0 and s0.speed_mps == 0.0)
    check("still-air control: nonzero drag on a moving vehicle",
          abs(f_still.f_n + drag.mass_kg * drag.drag_accel_m_s2(16.0)) < 1e-9,
          f"{f_still.f_n:.4f}")

    print()
    _report_tiers(6.0)
    print()
    print(f"{6 + 3 + 2 + 2 + 5 + 2 - len(fails)}/"
          f"{6 + 3 + 2 + 2 + 5 + 2} wind_model self-test checks passed")
    return 1 if fails else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true",
                    help="run the dependency-light smoke test (exit 0/1)")
    ap.add_argument("--tiers", action="store_true",
                    help="print the BEST/EXPECTED/WORST tier table")
    ap.add_argument("--height", type=float, default=None,
                    help="height AGL in metres for --tiers (REQUIRED for "
                         "--tiers; no default -- it is a site measurement)")
    ap.add_argument("--allow-below-10ft", action="store_true",
                    help="acknowledge extrapolating the MIL-F-8785C "
                         "low-altitude form below its 10 ft floor")
    a = ap.parse_args(argv)
    if a.self_test:
        return _self_test()
    if a.tiers:
        if a.height is None:
            print("--tiers requires --height (metres AGL): it is a site "
                  "measurement, not a default.", file=sys.stderr)
            return 2
        _report_tiers(a.height, a.allow_below_10ft)
        return 0
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
