#!/usr/bin/env python3
"""PRE-FLIGHT ANEMOMETER WIND TRIM -- the only wind "error correction" the
open-loop dash can honestly have (wind Workflow, design.correction layer 2).

==========================================================================
READ THIS FIRST: THE FINDING, STATED PLAINLY
==========================================================================
THE OPEN-LOOP DASH HAS NO IN-FLIGHT WIND MEASUREMENT, THEREFORE NO IN-FLIGHT
WIND CORRECTION IS POSSIBLE AND NONE IS BUILT HERE.

  * The airframe has no pitot / airspeed sensor (not in docs/hardware_order_list.md).
  * The guidance datalink is dead by project constraint `no-datalink`
    (docs/project_state.json constraints) -- nobody can uplink a wind vector.
  * PX4 EKF2's drag-fusion wind estimate exists in SITL, but consuming it here
    would be a given-perfect input wearing a measured costume: in SITL it is
    fitted to the simulator's OWN drag law, so it would "measure" the very
    thing the sim injects. REFUSED on honesty grounds, as is
    `resetWindToExternalObservation` (the pre-flight injection loophole the
    contradiction ledger exists to close).
  * The simulator's true wind vector is on the gt_* side of the honesty
    boundary: SCORING AND LOGGING ONLY. It never reaches this module. This
    file enforces that structurally (see THE BOUNDARY, below).

So the correction story has exactly three layers, and only ONE of them is
code that did not already exist:

  LAYER 1 -- PX4's own velocity loop, closed on own-state GPS/EKF. ALREADY
    EXISTS, costs nothing, and is the primary steady-wind rejection: the dash
    commands a ground-velocity NED setpoint, PX4 tilts into the wind and holds
    the track. Nothing to build. It must be OBSERVED, not assumed -- that is
    what the wind arms (Gate 1/Gate 2) measure, including its failure modes
    (tilt-authority saturation under a WORST-tier headwind, gusts above loop
    bandwidth).
  LAYER 2 -- THIS MODULE. The operator reads a handheld anemometer BEFORE
    launch and types two numbers. Architecturally identical to the latched
    GPS launch cue: a pre-flight constant, not a live read (builder ruling,
    assumption `launch-cue-error-free`). It corrects the one pre-flight-
    correctable term: the expected dash GROUND-SPEED SAG along-wind, which
    feeds the lead solve's time-of-flight.
  LAYER 3 -- the terminal camera phase IS the in-flight wind correction.
    Pro-nav closed on the camera LOS nulls accumulated trajectory error
    automatically, provided the wind-rotated camera wedge still holds the
    target in FOV. Nothing in the terminal path changes for wind; its wind
    response is MEASURED as-is (Gate 3). No code here.

==========================================================================
THE BOUNDARY -- how this file is kept honest, structurally
==========================================================================
1. This module imports STDLIB ONLY (json/math/dataclasses/typing). It cannot
   import the sim, the gz transport layer, the wind driver, the anemometer
   simulator, or any scorer. `tests/test_wind_trim_honesty.py` asserts the
   import set against an allowlist and proves the assertion fires by mutating
   a copy of this file.
2. The ONLY wind input it will accept is an `AnemometerReading` -- a value
   object with the operator's instrument reading and its declared sigma.
   `AnemometerReading.from_mapping()` REJECTS any key matching a
   ground-truth-ish pattern (gt_*, true_*, truth*, *_world_wind, sim_wind...).
   That makes the JSON handoff from the sim-side instrument model
   (scripts/wind_anemometer_sim.py, the PRODUCER) fail closed: if the producer
   ever leaked the simulator's true wind into the file, this CONSUMER refuses
   to load it rather than quietly using it.
3. No default anywhere substitutes for a number that must be MEASURED. There
   is NO baked-in sag table: enabling the trim without a measured
   `SagTable` raises `UnmeasuredSagError`. Today no measured table exists
   (the Gate-2 wind arms have not flown), so today the trim CANNOT be
   enabled -- that is the intended, loud state, not a bug.

==========================================================================
WHAT IT DELIBERATELY DOES NOT DO (limitations, in code not just prose)
==========================================================================
* NO HEADING TRIM. `WindTrimResult.heading_trim_deg` is ALWAYS 0.0 and a test
  pins it. Crosswind track-holding is Layer 1's job (PX4 closes on ground
  velocity); a pre-flight heading bias computed from a noisy anemometer would
  fight the controller that is already doing it. If the Gate-2 crosswind arm
  shows a systematic cross-track residual the velocity loop does NOT reject,
  that is a NEW finding and a new, separately pre-registered parameter -- it
  does not silently appear here.
* WHAT ACTUALLY CORRECTS FOR WIND, AND WHERE (builder question, 2026-08-12).
  In the TERMINAL phase, pro-nav corrects for wind with no wind-specific code at
  all: wind pushes the vehicle sideways, the line of sight to the target rotates,
  and pro-nav commands acceleration against that rotation -- it does not care
  WHY the geometry is changing. The airframe's lean-into-wind is absorbed too,
  because bearings are de-rotated through the full attitude quaternion
  (m4_intercept.py derotate_bearing_lambda).
  THE GAP IS EVERYTHING BEFORE THAT. The camera can only correct what is inside
  its wedge, and the dash is open-loop with the link dead, so wind spends the
  whole dash pushing the vehicle off its aim line. Push it far enough and the
  target never enters the wedge, the terminal phase never begins, and there is
  nothing to correct. THAT is the only thing this module defends: it moves the
  pre-flight AIM so the dash ends with the target still in view. Getting the
  target into the wedge is the wind problem; once it is there, pro-nav has it.

* NO VERTICAL TRIM -- AND THE REASON GIVEN HERE ORIGINALLY WAS WRONG
  (corrected 2026-08-10). This module was told, while it was being designed,
  that the published -0.37 m vertical bias was "~56% camera lens-arm artefact"
  and that a trim sized off it would overcorrect >2x. That is FALSE. The
  "+0.208 m lens arm" it rested on was `median(gt_cam_z - alt_m)`, and `alt_m`
  is MAVSDK `relative_altitude_m` -- height above the TAKEOFF POINT, not world
  z -- so the quantity measured the LANDING GEAR. The camera sits ~2 mm above
  the airframe datum. Re-measured over 168 flights at the corrected
  centre-to-centre CPA, the vertical bias is median **-0.374 m**: it stands
  (ADR-0095, docs/rescore_2026-08-10.md).
  THE DECISION IS UNCHANGED, for a better reason: a vertical trim is a
  GUIDANCE lever (the altitude channel was never closed -- the lead solve is
  2-D horizontal by construction), not a WIND correction. It belongs to its
  own pre-registered experiment, not bundled into a wind module where its
  effect could not be attributed. Nothing here depends on that constant either
  way -- which is why a wrong rationale produced no wrong code.
* IT CORRECTS THE AIM, NOT THE THROTTLE. The corrected speed is what the
  pre-flight LEAD SOLVE should ASSUME the vehicle will achieve, so the aim
  leads further ahead of a crossing target and expects a longer time of
  flight. The COMMANDED dash speed is untouched -- commanding less speed into
  a headwind would make the sag worse, not better. A caller that feeds
  `corrected_speed_mps` to a velocity setpoint has misread this module.
* NO ACCELERATION TRIM. Only the assumed achievable dash ground SPEED is
  corrected; the lead solve's accel term (`--dash-lead-accel-ms2`) is
  untouched. Wind's effect on the accel ramp is measured by Gate 1/2 and, if
  real, becomes its own pre-registered parameter.
* THE SAG TABLE IS KEYED ON THE HEADWIND COMPONENT ONLY (1-D). A pure
  crosswind still adds drag magnitude (|dv| includes the cross term), so the
  1-D key is a declared approximation. `WindTrimResult.crosswind_mps` is
  computed and logged so the arms can test whether 1-D suffices before
  anything is adopted.
* IT IS NOT WIRED INTO THE FLIGHT HARNESS BY THIS FILE. `scripts/m4_intercept.py`
  gains `--wind-trim-mps` / `--wind-trim-dir-deg` in a separate, isolated
  commit (design W4, sequenced after scoring_fix_plan.md #8/#9). Both default
  to 0.0 -> `WindTrim.disabled()` -> this module returns the nominal speed
  unchanged -> byte-identical to today's flights.

==========================================================================
UNITS, FRAMES, AND HOW EVERY KNOB IS BENCH-MEASURED (builder mandate)
==========================================================================
  heading_deg     compass azimuth of the dash, atan2(east, north) in DEGREES
                  (0 = north, 90 = east) -- the SAME convention as
                  flight/guidance.py:collision_lead_heading.
  dir_from_deg    METEOROLOGICAL wind direction: the compass bearing the wind
                  blows FROM, degrees, 0 = from north. Measured with a wind
                  sock / compass at the launch point, +/-15 deg.
  speed_mps       metres per second. Measured with the ~$20 handheld
                  anemometer named in assumption `zero-wind-environment`'s
                  test plan: 1-minute mean at launch height.
  sigma_*         the instrument's declared accuracy (spec sheet), carried
                  with the reading so a log never loses it. Graded
                  `given-noisy` in docs/project_state.json assumptions[].
  taken_at_sim_t  SIM seconds (never wall clock -- CLAUDE.md standing rule).
  sag_mps         m/s of dash ground speed LOST to a given headwind
                  component. MEASURED from the Gate-2 wind arms; there is no
                  derived default and none may be invented.

Sign conventions (all in the horizontal E/N plane):
  dash unit vector      u = (sin h, cos h)                       [east, north]
  wind velocity vector  w = (-V sin d, -V cos d)   (blows FROM d)
  tailwind  = w . u        = -V cos(d - h)      (>0 pushes you along)
  headwind  = -tailwind    = +V cos(d - h)      (>0 slows you down)
  crosswind = w . right    = -V sin(d - h)      (right = east of heading)

Run `python3 scripts/wind_trim.py --self-check` for a dependency-free
demonstration of the arithmetic on a synthetic table (prints, exits 0/1).
Tests: tests/test_wind_trim.py, tests/test_wind_trim_honesty.py,
tests/test_wind_trim_contract.py.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Callable, Mapping, Optional, Sequence, Tuple

__all__ = [
    "AnemometerReading",
    "AnemometerSpec",
    "PLACEHOLDER_ANEMOMETER_SPEC",
    "SagTable",
    "WindTrim",
    "WindTrimResult",
    "WindTrimError",
    "UnmeasuredSagError",
    "GroundTruthLeakError",
    "StaleReadingError",
    "WindTrimConvergenceError",
    "wind_components",
    "apply_wind_trim",
    "solve_wind_trimmed_lead",
    "load_reading_json",
    "READING_SCHEMA",
]

# Schema tag written by the PRODUCER (scripts/wind_anemometer_sim.py) and
# required by this CONSUMER. Bump both together; the contract test
# (tests/test_wind_trim_contract.py) builds its fixture from the producer's own
# writer, per CLAUDE.md's producer->consumer contract-test rule.
READING_SCHEMA = "anemometer_reading/1"

# Absolute float-dust dead-band on the sag table's measured domain, in m/s.
# 1e-9 m/s is nine orders of magnitude below the cheapest anemometer's 0.3 m/s
# sigma, so it can only ever absorb cos()/sin() rounding at a near-90 deg
# geometry -- never a real reading. See SagTable.sag_mps.
_DOMAIN_EPS = 1e-9

# Keys this module REFUSES to accept in a reading mapping. Anything matching
# these is, by construction, simulator ground truth or a scorer field trying
# to enter the correction path -- fail closed, never "ignore the extra key".
_FORBIDDEN_KEY_SUBSTRINGS = (
    "gt_",
    "ground_truth",
    "truth",
    "true_",
    "sim_wind",
    "world_wind",
    "driver_wind",
    "_actual",
    "cheat",
)

# The keys a reading legitimately carries. Unknown keys are refused too (an
# unknown key is an unreviewed input, and this is the one path where an
# unreviewed input is exactly the failure mode).
_ALLOWED_READING_KEYS = frozenset({
    "schema",
    "speed_mps",
    "dir_from_deg",
    "sigma_speed_mps",
    "sigma_dir_deg",
    "taken_at_sim_t",
    "provenance",
})


class WindTrimError(Exception):
    """Base for every fail-closed condition in the wind-trim path."""


class UnmeasuredSagError(WindTrimError):
    """Asked for a sag value that has never been MEASURED (no table, or a
    headwind outside the measured domain). Never substitute a default for a
    measured quantity -- CLAUDE.md fail-closed rule."""


class GroundTruthLeakError(WindTrimError):
    """A ground-truth-shaped input tried to enter the correction path."""


class StaleReadingError(WindTrimError):
    """The anemometer reading is older (in SIM seconds) than the configured
    maximum age, or a staleness check was requested with no clock."""


class WindTrimConvergenceError(WindTrimError):
    """The heading/speed fixed point did not converge inside the iteration
    budget. Fail closed: half a converged aim is not an aim."""


def _check_no_ground_truth(mapping: Mapping) -> None:
    for key in mapping:
        low = str(key).lower()
        for bad in _FORBIDDEN_KEY_SUBSTRINGS:
            if bad in low:
                raise GroundTruthLeakError(
                    f"anemometer reading contains the key {key!r}, which matches the "
                    f"forbidden ground-truth pattern {bad!r}. The wind trim is a "
                    "PRE-FLIGHT OPERATOR MEASUREMENT: it may consume the handheld "
                    "instrument's reading and nothing else. If the sim-side producer "
                    "(scripts/wind_anemometer_sim.py) started writing the simulator's "
                    "true wind into this file, fix the PRODUCER -- do not widen this "
                    "guard. See the honesty boundary in CLAUDE.md and the module "
                    "docstring of scripts/wind_trim.py."
                )


@dataclass(frozen=True)
class AnemometerSpec:
    """The instrument's declared error, from its data sheet. NO FIELD HAS A
    DEFAULT: a spec you did not state is a spec you did not measure.

    sigma_speed_mps / sigma_speed_frac: the accuracy statement, typically
      "+/-(0.3 m/s or 5% of reading, whichever is greater)" -- both halves are
      carried and the effective sigma is the max of the two at the read speed.
    sigma_dir_deg: direction reading accuracy (wind sock + compass, or the
      unit's vane spec), degrees.
    bias_speed_mps / bias_dir_deg: the PER-UNIT calibration offset. Zero here
      means "no bias has been measured", NOT "the unit is unbiased" -- see
      PLACEHOLDER_ANEMOMETER_SPEC's provenance string.
    provenance: free text naming the data sheet / measurement this came from.
      Required, and required non-empty.
    """
    sigma_speed_mps: float
    sigma_speed_frac: float
    sigma_dir_deg: float
    bias_speed_mps: float
    bias_dir_deg: float
    provenance: str

    def __post_init__(self) -> None:
        if not str(self.provenance).strip():
            raise WindTrimError(
                "AnemometerSpec.provenance must name the data sheet or bench "
                "measurement the sigmas came from (numbers trace to a run or a "
                "derivation -- CLAUDE.md).")
        for name in ("sigma_speed_mps", "sigma_speed_frac", "sigma_dir_deg"):
            if float(getattr(self, name)) < 0.0:
                raise WindTrimError(f"AnemometerSpec.{name} must be >= 0")

    def sigma_speed_at(self, speed_mps: float) -> float:
        """Effective 1-sigma speed error at a given reading (the 'whichever is
        greater' form that handheld anemometer sheets are written in)."""
        return max(float(self.sigma_speed_mps),
                   float(self.sigma_speed_frac) * abs(float(speed_mps)))


# PLACEHOLDER, and labelled as one everywhere it is used. The unit has not been
# bought (assumption `zero-wind-environment` test plan: "~$20 anemometer"), so
# these are the accuracy class typical of that price point, NOT a measured
# spec. The bias terms are 0.0 because NO bias has been measured -- when the
# unit arrives, a side-by-side against a reference (or the supplier's cal
# certificate) replaces this whole object, and any adoption decision taken
# with these numbers is re-checked against the real ones.
PLACEHOLDER_ANEMOMETER_SPEC = AnemometerSpec(
    sigma_speed_mps=0.3,
    sigma_speed_frac=0.05,
    sigma_dir_deg=15.0,
    bias_speed_mps=0.0,
    bias_dir_deg=0.0,
    provenance=("PLACEHOLDER 2026-08-10 -- typical +/-(0.3 m/s or 5%) handheld "
                "anemometer class; direction +/-15 deg from a wind sock + compass. "
                "NOT MEASURED: no unit purchased yet, and bias is UNMEASURED "
                "(0.0 means unknown, not unbiased). Replace with the purchased "
                "unit's data sheet before any adoption decision."),
)


@dataclass(frozen=True)
class AnemometerReading:
    """What the operator reads off the handheld instrument before launch.

    This is the ONLY wind input the correction path accepts. It carries no
    truth field and no simulator field -- see `from_mapping`.
    """
    speed_mps: float
    dir_from_deg: float
    sigma_speed_mps: float
    sigma_dir_deg: float
    taken_at_sim_t: Optional[float]
    provenance: str

    def __post_init__(self) -> None:
        if float(self.speed_mps) < 0.0:
            raise WindTrimError(
                f"anemometer speed {self.speed_mps} m/s is negative; a cup/vane "
                "anemometer cannot read a negative speed -- direction lives in "
                "dir_from_deg.")
        if not math.isfinite(float(self.speed_mps)) or not math.isfinite(float(self.dir_from_deg)):
            raise WindTrimError("anemometer reading must be finite")
        if float(self.sigma_speed_mps) < 0.0 or float(self.sigma_dir_deg) < 0.0:
            raise WindTrimError("anemometer sigmas must be >= 0")
        if not str(self.provenance).strip():
            raise WindTrimError(
                "AnemometerReading.provenance must say where the reading came from "
                "(field log line, or the sim instrument model + seed).")

    @staticmethod
    def from_mapping(data: Mapping) -> "AnemometerReading":
        """Build from a dict (e.g. parsed JSON), FAILING CLOSED on any
        ground-truth-shaped or unknown key. This is the structural gate that
        makes a leaky producer a loud error instead of a silent cheat."""
        _check_no_ground_truth(data)
        unknown = sorted(set(map(str, data.keys())) - _ALLOWED_READING_KEYS)
        if unknown:
            raise GroundTruthLeakError(
                f"anemometer reading has unknown key(s) {unknown}; the correction "
                f"path accepts exactly {sorted(_ALLOWED_READING_KEYS)}. An unreviewed "
                "input is how a truth value gets in -- refuse it.")
        schema = data.get("schema")
        if schema != READING_SCHEMA:
            raise WindTrimError(
                f"anemometer reading schema {schema!r} != expected {READING_SCHEMA!r}; "
                "producer and consumer are out of step (bump both together).")
        missing = sorted({"speed_mps", "dir_from_deg", "sigma_speed_mps",
                          "sigma_dir_deg", "provenance"} - set(map(str, data.keys())))
        if missing:
            raise WindTrimError(
                f"anemometer reading is missing required field(s) {missing}. There is "
                "no default for a MEASURED quantity or its declared sigma.")
        return AnemometerReading(
            speed_mps=float(data["speed_mps"]),
            dir_from_deg=float(data["dir_from_deg"]),
            sigma_speed_mps=float(data["sigma_speed_mps"]),
            sigma_dir_deg=float(data["sigma_dir_deg"]),
            taken_at_sim_t=(None if data.get("taken_at_sim_t") is None
                            else float(data["taken_at_sim_t"])),
            provenance=str(data["provenance"]),
        )

    def to_mapping(self) -> dict:
        """The wire form. Deliberately the same key set `from_mapping`
        allows, so a round trip is exact and a new field cannot appear on one
        side only."""
        return {
            "schema": READING_SCHEMA,
            "speed_mps": float(self.speed_mps),
            "dir_from_deg": float(self.dir_from_deg),
            "sigma_speed_mps": float(self.sigma_speed_mps),
            "sigma_dir_deg": float(self.sigma_dir_deg),
            "taken_at_sim_t": (None if self.taken_at_sim_t is None
                               else float(self.taken_at_sim_t)),
            "provenance": str(self.provenance),
        }


def load_reading_json(path: str) -> AnemometerReading:
    """CONSUMER side of the producer->consumer contract. Refuses a file that
    carries any ground-truth-shaped key (see `AnemometerReading.from_mapping`)."""
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    if not isinstance(data, dict):
        raise WindTrimError(f"{path}: expected a JSON object, got {type(data).__name__}")
    return AnemometerReading.from_mapping(data)


@dataclass(frozen=True)
class SagTable:
    """MEASURED dash ground-speed sag vs headwind component. There is no
    default table and none may be invented: every point comes from a flown
    wind arm (design Gate 2), and the object refuses to exist without a
    provenance string naming those runs.

    points: ascending (headwind_mps, sag_mps) pairs. sag > 0 means the dash
      ends up SLOWER than its nominal commanded speed.

    Two structural requirements, both deliberate:
      * the table MUST contain the point (0.0, 0.0) -- the Gate-1 drag-only
        control arm IS the zero-headwind reference, so sag is 0 there by
        definition of the reference. A table without it is measuring against
        an unstated baseline.
      * NO EXTRAPOLATION. A headwind outside [min, max] of the measured points
        raises UnmeasuredSagError. Guessing past the last flown tier is
        exactly the "threshold validated at one operating point" trap.
    """
    points: Tuple[Tuple[float, float], ...]
    provenance: str

    def __post_init__(self) -> None:
        pts = tuple((float(a), float(b)) for a, b in self.points)
        object.__setattr__(self, "points", pts)
        if not str(self.provenance).strip():
            raise WindTrimError(
                "SagTable.provenance must name the flown arms the points came from "
                "(numbers trace to a run -- CLAUDE.md).")
        if len(pts) < 2:
            raise WindTrimError(
                f"SagTable needs >= 2 measured points, got {len(pts)}. One point is "
                "not a curve.")
        xs = [p[0] for p in pts]
        if any(xs[i] >= xs[i + 1] for i in range(len(xs) - 1)):
            raise WindTrimError(
                f"SagTable points must be strictly ascending in headwind; got {xs}")
        zero = [p for p in pts if p[0] == 0.0]
        if not zero:
            raise WindTrimError(
                "SagTable must contain a point at headwind 0.0 -- the drag-only "
                "control arm is the zero-headwind reference. Without it the sag is "
                "measured against an unstated baseline.")
        if zero[0][1] != 0.0:
            raise WindTrimError(
                f"SagTable's headwind-0 point must have sag 0.0 (it IS the reference); "
                f"got {zero[0][1]}")

    @property
    def domain(self) -> Tuple[float, float]:
        return (self.points[0][0], self.points[-1][0])

    def sag_mps(self, headwind_mps: float) -> float:
        """Piecewise-linear interpolation of the MEASURED points. Raises
        outside the measured domain -- fail closed, never extrapolate.

        FLOAT-DUST DEAD-BAND (not a tolerance for extrapolation): a near-90 deg
        geometry gives a headwind of ~1e-16 m/s from cos() rounding, which is
        physically zero but numerically outside a domain starting at 0.0.
        Values within _DOMAIN_EPS (1e-9 m/s -- nine orders below the
        instrument's 0.3 m/s sigma, so it can never absorb a real reading) are
        SNAPPED to the boundary; anything beyond it still raises. This is the
        same relative-dead-band idiom scripts/m4_intercept.py's crossing_sign
        uses for exactly this class of dust.
        """
        h = float(headwind_mps)
        lo, hi = self.domain
        if not math.isfinite(h):
            raise UnmeasuredSagError(f"headwind component {h} is not finite")
        if lo - _DOMAIN_EPS <= h < lo:
            h = lo
        elif hi < h <= hi + _DOMAIN_EPS:
            h = hi
        if h < lo or h > hi:
            raise UnmeasuredSagError(
                f"headwind component {h:.3f} m/s is outside the MEASURED sag domain "
                f"[{lo:.3f}, {hi:.3f}] m/s ({self.provenance}). Refusing to "
                "extrapolate: a sag value that was never flown is not a measurement. "
                "Fly the arm at that headwind, or leave the trim off for this launch.")
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            if x0 <= h <= x1:
                if x1 == x0:
                    return y0
                w = (h - x0) / (x1 - x0)
                return y0 + w * (y1 - y0)
        raise UnmeasuredSagError(  # pragma: no cover -- domain check above covers it
            f"headwind {h} fell through the sag table bracket search")

    @staticmethod
    def from_json(path: str) -> "SagTable":
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        _check_no_ground_truth(data)
        return SagTable(points=tuple(tuple(p) for p in data["points"]),
                        provenance=str(data["provenance"]))


def wind_components(dash_heading_deg: float,
                    reading: AnemometerReading) -> Tuple[float, float, float]:
    """Resolve a met-convention reading into dash-frame components.

    Returns (headwind_mps, tailwind_mps, crosswind_mps):
      headwind  > 0  wind opposes the dash (slows it)
      tailwind        == -headwind, carried for readable logs
      crosswind > 0  wind pushes to the RIGHT of the dash heading (east of it)

    Derivation is in the module docstring's sign-convention block; this is a
    pure trig identity, not a fitted constant.
    """
    d = math.radians(float(reading.dir_from_deg))
    h = math.radians(float(dash_heading_deg))
    v = float(reading.speed_mps)
    headwind = v * math.cos(d - h)
    crosswind = -v * math.sin(d - h)
    return (headwind, -headwind, crosswind)


@dataclass(frozen=True)
class WindTrim:
    """Configuration for the pre-flight trim. DEFAULT IS DISABLED, and a
    disabled trim is an exact identity (the m4 flags default to 0.0, so a run
    that does not ask for wind trim is byte-identical to today's flights)."""
    enabled: bool = False
    reading: Optional[AnemometerReading] = None
    table: Optional[SagTable] = None
    # If set, the reading must be no older than this many SIM seconds at the
    # moment the trim is applied. If set, a clock MUST be supplied to
    # apply_wind_trim -- asking for a staleness check and silently skipping it
    # is the failure mode this fails closed on.
    max_reading_age_s: Optional[float] = None

    @staticmethod
    def disabled() -> "WindTrim":
        return WindTrim()

    @staticmethod
    def from_cli_values(wind_trim_mps: float,
                        wind_trim_dir_deg: float,
                        table: Optional[SagTable],
                        spec: AnemometerSpec = PLACEHOLDER_ANEMOMETER_SPEC,
                        taken_at_sim_t: Optional[float] = None,
                        max_reading_age_s: Optional[float] = None) -> "WindTrim":
        """Entry point for the (separately committed) m4 flags
        `--wind-trim-mps` / `--wind-trim-dir-deg`.

        `wind_trim_mps == 0.0` is the DISABLED sentinel -- it matches the
        aim-knob convention already in scripts/m4_intercept.py (a 0.0 knob is
        a no-op, byte-identical). An operator who genuinely measures dead calm
        has nothing to correct, so the sentinel loses no information.
        """
        if not float(wind_trim_mps):
            return WindTrim.disabled()
        reading = AnemometerReading(
            speed_mps=float(wind_trim_mps),
            dir_from_deg=float(wind_trim_dir_deg) % 360.0,
            sigma_speed_mps=spec.sigma_speed_at(float(wind_trim_mps)),
            sigma_dir_deg=float(spec.sigma_dir_deg),
            taken_at_sim_t=taken_at_sim_t,
            provenance=("operator handheld anemometer, typed at launch via "
                        f"--wind-trim-mps/--wind-trim-dir-deg; spec: {spec.provenance}"),
        )
        return WindTrim(enabled=True, reading=reading, table=table,
                        max_reading_age_s=max_reading_age_s)


@dataclass(frozen=True)
class WindTrimResult:
    """What the trim did, with enough provenance that a log read months later
    cannot confuse a MEASURED correction with an INJECTED error (the same
    reason --dash-aim-trim-deg is a separate flag from --dash-heading-err-deg
    in scripts/m4_intercept.py)."""
    enabled: bool
    nominal_speed_mps: float
    corrected_speed_mps: float
    headwind_mps: float
    crosswind_mps: float
    sag_mps: float
    # The operator's raw reading, echoed so a log line states the convention
    # (met: the bearing the wind blows FROM) instead of only the derived
    # components -- a frame/convention mix-up with the sim-side wind driver is
    # the most likely integration bug in this whole path.
    reading_speed_mps: float = 0.0
    reading_dir_from_deg: float = 0.0
    # ALWAYS 0.0 -- see the module docstring, "NO HEADING TRIM". Present as a
    # field (not absent) so the limitation is pinned by a test instead of
    # living only in prose.
    heading_trim_deg: float = 0.0
    reading_provenance: str = ""
    table_provenance: str = ""

    @property
    def is_noop(self) -> bool:
        return (self.corrected_speed_mps == self.nominal_speed_mps
                and self.heading_trim_deg == 0.0)

    @property
    def log_line(self) -> str:
        if not self.enabled:
            return ("[wind-trim] OFF -- lead solve uses the nominal dash speed "
                    f"{self.nominal_speed_mps:.2f} m/s (byte-identical to a no-wind run)")
        return (
            f"[wind-trim] PRE-FLIGHT anemometer trim ON: operator read "
            f"{self.reading_speed_mps:.2f} m/s FROM {self.reading_dir_from_deg:.1f} deg "
            f"(met convention) -> headwind {self.headwind_mps:+.2f} "
            f"m/s, crosswind {self.crosswind_mps:+.2f} m/s -> measured sag "
            f"{self.sag_mps:+.2f} m/s -> lead-solve dash speed "
            f"{self.nominal_speed_mps:.2f} -> {self.corrected_speed_mps:.2f} m/s; "
            f"heading trim {self.heading_trim_deg:+.2f} deg (always 0 by design). "
            f"OPERATOR MEASUREMENT, PRE-FLIGHT CONSTANT, no gt, no in-flight wind "
            f"sensing. reading: {self.reading_provenance} | table: {self.table_provenance}")


def apply_wind_trim(nominal_speed_mps: float,
                    dash_heading_deg: float,
                    trim: WindTrim,
                    now_sim_t: Optional[float] = None) -> WindTrimResult:
    """Correct the dash ground speed the LEAD SOLVE assumes, from the
    operator's pre-flight anemometer reading and a MEASURED sag table.

    Args:
      nominal_speed_mps: the dash speed the mission would have assumed with no
        wind information (m/s, > 0).
      dash_heading_deg: compass azimuth of the dash (deg, 0 = north, 90 = east).
      trim: configuration; `WindTrim.disabled()` -> exact identity.
      now_sim_t: current SIM time (seconds), required iff
        `trim.max_reading_age_s` is set.

    Returns a WindTrimResult. Raises (fail closed) if the trim is enabled but
    the sag is unmeasured, the reading is stale, or the correction would drive
    the assumed speed to <= 0.
    """
    v_nom = float(nominal_speed_mps)
    if not trim.enabled:
        return WindTrimResult(
            enabled=False, nominal_speed_mps=v_nom, corrected_speed_mps=v_nom,
            headwind_mps=0.0, crosswind_mps=0.0, sag_mps=0.0,
            reading_provenance="(trim off)", table_provenance="(trim off)")
    if v_nom <= 0.0:
        raise WindTrimError(f"nominal dash speed must be > 0, got {v_nom}")
    if trim.reading is None:
        raise WindTrimError(
            "wind trim is enabled with no AnemometerReading -- there is nothing to "
            "correct FROM. (An enabled-but-empty trim is exactly the inert-flag class "
            "of bug: it would log 'wind trim ON' and change nothing.)")
    if trim.table is None:
        raise UnmeasuredSagError(
            "wind trim is enabled but no MEASURED SagTable was supplied. There is no "
            "default sag: the table's points come from the Gate-2 wind arms, which "
            "have not flown yet. Until they do, the honest configuration is "
            "trim OFF -- see scripts/wind_trim.py's module docstring.")
    if trim.max_reading_age_s is not None:
        if now_sim_t is None:
            raise StaleReadingError(
                "max_reading_age_s is set but no now_sim_t was supplied; refusing to "
                "silently skip a staleness check that was explicitly requested. "
                "(SIM seconds -- never wall clock.)")
        if trim.reading.taken_at_sim_t is None:
            raise StaleReadingError(
                "max_reading_age_s is set but the reading carries no taken_at_sim_t; "
                "an undated reading cannot be checked for staleness.")
        age = float(now_sim_t) - float(trim.reading.taken_at_sim_t)
        if age < 0.0:
            raise StaleReadingError(
                f"anemometer reading is dated {age:.2f} s in the FUTURE relative to the "
                "sim clock -- clock basis mismatch, refusing to use it.")
        if age > float(trim.max_reading_age_s):
            raise StaleReadingError(
                f"anemometer reading is {age:.2f} sim-s old, older than the configured "
                f"{float(trim.max_reading_age_s):.2f} s. Take a fresh reading.")

    headwind, _tailwind, crosswind = wind_components(dash_heading_deg, trim.reading)
    sag = trim.table.sag_mps(headwind)
    corrected = v_nom - sag
    if corrected <= 0.0:
        raise WindTrimError(
            f"wind trim would set the assumed dash speed to {corrected:.2f} m/s "
            f"(nominal {v_nom:.2f} - measured sag {sag:.2f}); a non-positive assumed "
            "speed is not an intercept solution. This headwind is outside the regime "
            "the dash can fly -- do not launch, do not fudge the number.")
    return WindTrimResult(
        enabled=True, nominal_speed_mps=v_nom, corrected_speed_mps=corrected,
        headwind_mps=headwind, crosswind_mps=crosswind, sag_mps=sag,
        reading_speed_mps=float(trim.reading.speed_mps),
        reading_dir_from_deg=float(trim.reading.dir_from_deg),
        heading_trim_deg=0.0,
        reading_provenance=trim.reading.provenance,
        table_provenance=trim.table.provenance)


def solve_wind_trimmed_lead(lead_solver: Callable[..., Tuple[float, Optional[float]]],
                            target_pos: Sequence[float],
                            target_vel: Sequence[float],
                            nominal_speed_mps: float,
                            trim: WindTrim,
                            now_sim_t: Optional[float] = None,
                            max_iter: int = 8,
                            tol_deg: float = 1e-6):
    """Resolve the chicken-and-egg between the aim and the trim, ONCE,
    pre-flight.

    The headwind component depends on the dash HEADING, but the heading comes
    out of the lead solve, which needs the dash SPEED, which the trim adjusts.
    So iterate a bounded fixed point:

        h0 = lead_solver(..., v_nom)                  # today's answer
        v1 = v_nom - sag(headwind(h0));  h1 = lead_solver(..., v1)
        ... until |h_k - h_{k-1}| < tol_deg, or FAIL.

    Deterministic (no randomness, fixed budget) so the same launch inputs
    always give the same aim. If it does not converge inside `max_iter` the
    call RAISES -- returning a half-converged aim would be a silent wrong
    answer, the exact class this project keeps getting bitten by.

    `lead_solver` is injected rather than imported so this module stays
    stdlib-only and cannot reach into the flight harness (honesty boundary,
    enforced by tests/test_wind_trim_honesty.py). In use it is
    flight.guidance.collision_lead_heading (or ..._accel with its accel bound
    already partial-applied) -- both return (heading_deg, t_lead).

    Returns (heading_deg, t_lead, WindTrimResult).

    DISABLED TRIM => EXACTLY ONE lead_solver call, with the nominal speed, and
    the untouched heading: byte-identical to today.
    """
    heading_deg, t_lead = lead_solver(target_pos, target_vel, float(nominal_speed_mps))
    result = apply_wind_trim(float(nominal_speed_mps), heading_deg, trim,
                             now_sim_t=now_sim_t)
    if not trim.enabled:
        return heading_deg, t_lead, result
    if int(max_iter) < 1:
        raise WindTrimError("max_iter must be >= 1")
    for _ in range(int(max_iter)):
        new_heading, new_t_lead = lead_solver(
            target_pos, target_vel, result.corrected_speed_mps)
        delta = abs(_wrap_180(new_heading - heading_deg))
        heading_deg, t_lead = new_heading, new_t_lead
        result = apply_wind_trim(float(nominal_speed_mps), heading_deg, trim,
                                 now_sim_t=now_sim_t)
        if delta < float(tol_deg):
            return heading_deg, t_lead, result
    raise WindTrimConvergenceError(
        f"wind-trimmed lead solve did not converge within {int(max_iter)} iterations "
        f"(last heading step {delta:.6f} deg > tol {float(tol_deg):.6f}). Refusing to "
        "return a half-converged aim.")


def _wrap_180(deg: float) -> float:
    """Signed angle difference in (-180, 180]."""
    return (float(deg) + 180.0) % 360.0 - 180.0


# ==========================================================================
# Dependency-free demonstration (NOT a gate; the gates are the pytest files).
# ==========================================================================
def _self_check() -> int:
    ok = True

    def check(label, cond):
        nonlocal ok
        print(f"  [{'PASS' if cond else 'FAIL'}] {label}")
        ok = ok and bool(cond)

    print("wind_trim self-check (synthetic table -- NOT measured, illustration only)")
    # A synthetic table, clearly labelled: this is what a Gate-2 table would
    # LOOK like. It is not a measurement and must never be flown.
    table = SagTable(points=((0.0, 0.0), (4.5, 0.6), (8.0, 1.8)),
                     provenance="SYNTHETIC self-check illustration -- NOT MEASURED")
    reading = AnemometerReading(
        speed_mps=8.0, dir_from_deg=0.0, sigma_speed_mps=0.4, sigma_dir_deg=15.0,
        taken_at_sim_t=0.0, provenance="self-check")
    trim = WindTrim(enabled=True, reading=reading, table=table)

    off = apply_wind_trim(16.0, 0.0, WindTrim.disabled())
    check("disabled trim is an identity", off.is_noop and off.corrected_speed_mps == 16.0)
    print("    " + off.log_line)

    head = apply_wind_trim(16.0, 0.0, trim)   # dash north, wind from north
    check("pure headwind -> full sag", abs(head.headwind_mps - 8.0) < 1e-9
          and abs(head.corrected_speed_mps - 14.2) < 1e-9)
    print("    " + head.log_line)

    try:
        # dash south, wind from north -> headwind -8 m/s (a tailwind), which the
        # table never measured: refuse rather than extrapolate.
        apply_wind_trim(16.0, 180.0, trim)
        check("pure tailwind is REFUSED (outside measured domain)", False)
    except UnmeasuredSagError:
        check("pure tailwind is REFUSED (outside measured domain)", True)

    # Dash EAST, wind FROM north -> the wind blows toward the south, which is to
    # the RIGHT of an eastward heading, so crosswind is +8 (right-positive).
    cross = apply_wind_trim(16.0, 90.0, trim)
    check("pure crosswind -> zero headwind, zero sag, +8 m/s from the left",
          abs(cross.headwind_mps) < 1e-9 and abs(cross.sag_mps) < 1e-12
          and abs(cross.crosswind_mps - 8.0) < 1e-9)
    print("    " + cross.log_line)

    try:
        apply_wind_trim(16.0, 0.0, WindTrim(enabled=True, reading=reading, table=None))
        check("enabled with no measured table RAISES", False)
    except UnmeasuredSagError:
        check("enabled with no measured table RAISES", True)

    print("wind_trim self-check:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    import argparse
    import sys

    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--self-check", action="store_true",
                    help="run the dependency-free arithmetic demonstration")
    _a = ap.parse_args()
    if _a.self_check:
        sys.exit(_self_check())
    ap.print_help()
    sys.exit(0)
