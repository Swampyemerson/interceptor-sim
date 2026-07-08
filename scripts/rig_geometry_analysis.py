#!/usr/bin/env python3
r"""Ground stereo-rig GEOMETRY + RATE analysis pack (Phase 2, `docs/
phase2_sim_to_real_plan.md` Sec 5.6 items #5, #7, #8). PURE MATH -- no
Gazebo, no PX4, no sim runs. Reuses `scripts/stereo_model.py`'s constants and
physics (imported, not re-derived) plus real telemetry already in `logs/` for
the flown dash-profile geometry. Nothing here is committed by this script.

WHY THIS EXISTS. The Phase-2 adversarial review (ADR-0045) found three gaps
between the ratified plan and a buildable T16 (stereo rig world):
  #7  if RTF forces a rendered resolution below the 1920x1200 design point,
      f_px drops and the range-noise constant c = sigma_d/(b*f_px) RISES --
      validating against the fixed c=4.45e-05 would compare to the wrong
      model. Need: c and sigma_R at every credible resolution.
  #5  the rig sees a WEDGE (its 20-deg HFOV), not a dome. Nothing guarantees
      the ratified dash profile (`scripts/m4_target_mover.py`, `--x0 6.5
      --y0-mag 29.3`) crosses that wedge. Need: a rig-pose study against the
      REAL flown geometry, both dash directions.
  #8  the fusion arc assumed the cue emits sigma_v ~= 0.5 m/s
      (`s2_cue_mock.py --vel-sigma 0.5`) without ever deriving it from a
      track filter + measurement noise. Need: sigma_v vs. update rate,
      grounded in Kalata's tracking-index alpha-beta relations.

This script answers all three, in dependency order (1 feeds 3's resolution
sweep; 2 feeds 3's handoff-range input), and writes every table to
`logs/rig_geometry/*.csv` plus three plots to `plots/` (skip with
--no-plots). See `docs/rig_geometry_analysis.md` for the plain-language
write-up and the "Implications for the F1 council" bullets.

=======================================================================
ANALYSIS 1 -- RESOLUTION-SCALED sigma_R + DETECTION FLOOR
=======================================================================
Holds the LENS fixed (16 mm C-mount on the AR0234, HFOV = 0.349 rad ~= 20 deg
-- `docs/stereo_design.md`'s chosen design point) and varies only the
RENDERED PIXEL WIDTH (a resolution cut via binning/crop-scale/RTF-driven
render-resolution reduction -- NOT a different lens). At fixed HFOV,
    f_px = (W/2) / tan(HFOV/2)                                        (R1)
(this is literally `f_px = f_mm/pixel_pitch_mm` re-expressed for a FIXED
angle instead of a fixed focal length -- same physics, different free
variable). A resolution cut therefore shrinks f_px linearly with W, and
because `stereo_model.py` eq (4)/(D) sigma_R terms scale as 1/f_px, the
"scaled c" the task asks for,
    c(W, tier) = sigma_match_px(tier) / (b * f_px(W))                 (R2)
is the SAME functional form as `stereo_design.md`'s headline
sigma_R = a + c*R**2 model, but with matching noise ONLY (the dominant term,
~71% of the variance at the design point per `docs/stereo_design.md`'s error
budget). We report this simple c alongside the FULL 4-term RSS budget from
`stereo_model.sigma_R()` (reused unmodified, just called at the
resolution-scaled f_px) so a reader sees both the quick number the plan
asked for and the fuller, honest one.

Detection floor: `stereo_model.py`'s eq (10), R_detect = f_px*L/n90, evaluated
at the SPECIFIC (L=0.3 m, n90=10 px) point `docs/stereo_design.md` states as
"a 0.3 m drone needs >=10 px across" (not tiered -- a single, literal
instruction), plus the full tiered `stereo_model.detection_range_floor()` for
context (BEST/EXPECTED/WORST target-size + n90 assumptions).

=======================================================================
ANALYSIS 2 -- FoV-WEDGE COVERAGE vs. THE FLOWN DASH PROFILE
=======================================================================
Geometry (confirmed against `scripts/m4_target_mover.py` + `scripts/
mc_batch.sh`'s flight-plan generator + a real flown log, see the doc): the
interceptor launches near world origin; the target runs a STRAIGHT LINE at
FIXED world-X = x0 = 6.5 m, altitude z = 0.5 m, sweeping world-Y from
+/-y0_mag (29.3 m, +/- a small jitter) through Y=0 and beyond, at 9-16 m/s
(`mc_batch.sh --geometry standard`: start_x=x0, vx=0, vy=+/-speed). Two
mirror-image directions: "r2l" starts at Y=+y0_mag moving toward -Y; "l2r"
starts at Y=-y0_mag moving toward +Y.

The rig's shared stereo FoV is modelled as the INTERSECTION of two pinhole
cones, one per camera, each camera's boresight PARALLEL to the rig's own yaw
(a real stereo pair does not toe in -- both cameras point the same
direction; the "shared" part is the geometric overlap of two near-identical
cones offset by the baseline, which is what stereo matching actually
requires), half-angle HFOV/2 (~10 deg), cameras at +/-1.0 m along the rig's
LATERAL axis (perpendicular to its own boresight -- baseline is always
perpendicular to view direction for a fronto-parallel stereo rig). A target
point is "in the wedge" iff it is within BOTH cameras' half-angle cones.
Because the baseline (2 m) is two-plus orders of magnitude smaller than the
15-45 m engagement ranges here, the two cones are nearly coincident -- the
wedge is, to good approximation, just the single cone offset by the rig's
own boresight/position. We compute it exactly (both cameras) anyway, at
negligible cost, rather than assume the approximation.

The empirical HANDOFF point (target/interceptor coordinates at the moment
`m4_intercept.py`'s ADR-0010 #5 camera-handoff latch fires,
S2["HANDOFF_RANGE_M"]=10.0, `m4_intercept.py:384`) is read from a real
flown log, `logs/m4_intercept_pronav_20260708T214600Z.csv` (an r2l-direction
pronav flight): interceptor gt_cam=(1.863, 6.874), target gt_tag=
(6.500, 16.042), gt_range=10.276 at t_sim=23.88 s. The l2r direction is the
Y-mirror of this (guidance/geometry are symmetric under Y -> -Y for this
"standard" flight-plan geometry) -- an approximation, disclosed in the doc,
not a second measured flight.

HONESTY NOTE (explicit per the task): at these 16-45 m rig-to-target ranges
the rig is operating FAR BELOW its 60-160 m design envelope
(`docs/stereo_design.md`). We check this is not silently swept under the
rug: sigma_R at these short ranges is trivially small (R**2 scaling cuts
both ways), so the ranging accuracy is a non-issue at handoff -- the binding
question is purely the ANGULAR wedge-coverage geometry, which is what this
analysis actually computes.

=======================================================================
ANALYSIS 3 -- sigma_v-vs-RATE (alpha-beta / Kalata)
=======================================================================
Kalata's tracking-index gains (T.P. Kalata, 1984, "The Tracking Index: A
Generalized Parameter for alpha-beta and alpha-beta-gamma Target Trackers",
IEEE Trans. Aerosp. Electron. Syst. AES-20(2):174-182; independently
reproduced on Wikipedia's "Alpha beta filter" and in `scripts/
guidance_lab.py`'s `kalata_alpha_beta()`, which THIS module reuses
byte-for-byte via import so there is exactly one copy of this formula in the
repo):
    lambda_idx = sigma_w * T**2 / sigma_R                              (K1)
    r          = (4 + lambda_idx - sqrt(8*lambda_idx + lambda_idx**2))/4
    alpha      = 1 - r**2
    beta       = 2*(2-alpha) - 4*sqrt(1-alpha)                         (K2)
where sigma_w is the assumed target-maneuver-acceleration std (m/s^2),
sigma_R the position-measurement noise std (m), T the update interval (s).

The STEADY-STATE ESTIMATE-ERROR VARIANCES for this exact filter
mechanization (matching `guidance_lab.AlphaBetaFilter`/`m4_intercept.py`'s
predict: x += xdot*dt; correct: x += alpha*r, xdot += (beta/T)*r) are
DERIVED here from the discrete Riccati fixed point (shown in-line below) and
VERIFIED numerically against a 400k-step Monte-Carlo replica of the exact
filter code (see this task's report; residual < 0.5% at 3 tested lambda
values):
    Var(position error)  =  alpha * sigma_R**2                         (K3)
    Var(velocity error)  =  (sigma_R**2 / T**2)
                             * ( alpha*beta/(1-alpha) - lambda_idx**2/2 )   (K4)
    sigma_v = sqrt(Var(velocity error))
Steady-state LAG under a constant target acceleration a (the standard g-h
filter lag/bias result, ALSO verified numerically here, exact match to
machine precision in a noise-free constant-a simulation):
    lag_m = a * T**2 * (1 - alpha) / beta                               (K5)

APPROXIMATION disclosed: (K3)/(K4) assume a SCALAR position channel with
measurement noise sigma_R. The rig's actual triangulated position has an
anisotropic error ellipse (radial sigma_R >> cross-range sigma_cross, 1-2
orders of magnitude per `docs/stereo_design.md`). Using sigma_R (the WORSE,
dominant axis) as the Cartesian measurement-noise input to (K1) is a
deliberately PESSIMISTIC, worst-axis simplification -- consistent with this
project's "simulate worse than ideal" mandate -- not a claim that both x/y
channels are equally noisy.

Run:
    .venv/bin/python scripts/rig_geometry_analysis.py            # tables + plots
    .venv/bin/python scripts/rig_geometry_analysis.py --no-plots  # tables only
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
LOGS_OUT_DIR = os.path.join(REPO_ROOT, "logs", "rig_geometry")
PLOTS_DIR = os.path.join(REPO_ROOT, "plots")

sys.path.insert(0, SCRIPT_DIR)
import stereo_model as sm  # noqa: E402  (reuse constants/physics, not re-derive)
from guidance_lab import kalata_alpha_beta  # noqa: E402  (single-source Kalata gains)

TIERS = sm.TIERS  # ("best", "expected", "worst")

# ============================================================================
# ANALYSIS 1 -- resolution-scaled sigma_R + detection floor
# ============================================================================
HFOV_RAD = math.radians(20.0)  # docs/stereo_design.md: 16 mm lens -> ~20 deg
RESOLUTIONS = [(1920, 1200), (1280, 960), (960, 540), (640, 480)]
A1_RANGES_M = [15.0, 30.0, 60.0, 100.0, 150.0]
A1_SIGMA_GATE_M = 1.0          # "usable range" gate, matches stereo_design.md
A1_DETECT_L_M = 0.3            # "a 0.3 m drone needs >=10 px" -- literal ask
A1_DETECT_N90_PX = 10.0


def f_px_at_fixed_hfov(width_px, hfov_rad=HFOV_RAD):
    """Eq (R1): f_px = (W/2) / tan(HFOV/2) -- a resolution CUT at fixed lens
    FOV (binning / crop-scale / lower native resolution with a matched lens),
    the natural reading of ADR-0045 finding #7 ("if RTF forces a resolution
    below 1920x1200"). NOT `stereo_model.f_px_from_mm` (that holds SENSOR
    PITCH fixed and varies focal length -- a different lens, not a render cut)."""
    return (width_px / 2.0) / math.tan(hfov_rad / 2.0)


def scaled_c_matching_only(f_px, tier):
    """Eq (R2): c = sigma_match_px(tier) / (b * f_px) -- the SIMPLE,
    matching-noise-only scaled constant the plan's #7 literally asks for.
    Reuses `stereo_model.PARAMS["sigma_match_px"]` (same BEST/EXPECTED/WORST
    tiers docs/stereo_design.md cites: 0.1/0.4/1.0 px) and the CHOSEN
    baseline (2.0 m). This is a LOWER BOUND on the full budget (matching
    noise is ~71% of the variance at the design point per docs/
    stereo_design.md's error-budget table) -- see the full-model column for
    the honest total."""
    sigma_match = sm.PARAMS["sigma_match_px"][tier]
    return sigma_match / (sm.CHOSEN_BASELINE_M * f_px)


def max_range_simple_c(c, sigma_gate=A1_SIGMA_GATE_M):
    """Largest R with c*R**2 <= sigma_gate (a=0 in a+c*R**2, per stereo_design
    .md's note that stereo NOISE vanishes at short range)."""
    if c <= 0:
        return float("inf")
    return math.sqrt(sigma_gate / c)


def detection_floor_fixed(f_px, L_m=A1_DETECT_L_M, n90=A1_DETECT_N90_PX):
    """R_detect = f_px * L / n90 (stereo_model eq 10), at the LITERAL
    (0.3 m, 10 px) point docs/stereo_design.md states, not tiered."""
    return f_px * L_m / n90


def run_analysis_1(outdir):
    long_rows = []
    summary_rows = []
    for (w, h) in RESOLUTIONS:
        f_px = f_px_at_fixed_hfov(w)
        hfov_check_deg = math.degrees(2.0 * math.atan((w / 2.0) / f_px))
        detect_floor_m = detection_floor_fixed(f_px)
        for tier in TIERS:
            c = scaled_c_matching_only(f_px, tier)
            r_max_simple = max_range_simple_c(c, A1_SIGMA_GATE_M)
            r_max_full = sm.range_at_sigma(sm.CHOSEN_BASELINE_M, f_px, tier, A1_SIGMA_GATE_M)
            detect_floor_tiered = sm.detection_range_floor(f_px, tier)
            summary_rows.append({
                "width_px": w, "height_px": h, "f_px": round(f_px, 1),
                "hfov_deg_check": round(hfov_check_deg, 3), "tier": tier,
                "c_scaled_matching_only_per_m2": c,
                "max_range_sigmaR_le_1m_simple_c_m": round(r_max_simple, 2),
                "max_range_sigmaR_le_1m_full_model_m": round(r_max_full, 2),
                "detect_floor_0.3m_10px_m": round(detect_floor_m, 2),
                "detect_floor_tiered_m": round(detect_floor_tiered, 2),
            })
            for R in A1_RANGES_M:
                sigma_simple = c * R * R
                sigma_full = float(sm.sigma_R(np.array([R]), sm.CHOSEN_BASELINE_M, f_px, tier)[0])
                long_rows.append({
                    "width_px": w, "height_px": h, "f_px": round(f_px, 1),
                    "tier": tier, "R_m": R,
                    "sigma_R_simple_c_m": round(sigma_simple, 5),
                    "sigma_R_full_model_m": round(sigma_full, 5),
                    "detect_floor_0.3m_10px_m": round(detect_floor_m, 2),
                })

    _write_csv(os.path.join(outdir, "res_sigma_table.csv"), long_rows)
    _write_csv(os.path.join(outdir, "res_summary.csv"), summary_rows)
    return long_rows, summary_rows


# ============================================================================
# ANALYSIS 2 -- FoV-wedge coverage vs. the flown dash profile
# ============================================================================
X0_M = 6.5
Y0_MAG_M = 29.3
Z_TARGET_M = 0.5
Z_RIG_M = 1.5           # assumed mast/tripod height (docs/stereo_design.md); small vs 15-45 m range
HANDOFF_RANGE_M = 10.0   # m4_intercept.py S2["HANDOFF_RANGE_M"], confirmed by grep
RIG_BASELINE_HALF_M = sm.CHOSEN_BASELINE_M / 2.0  # 1.0 m each side of rig center
HALF_FOV_RAD = HFOV_RAD / 2.0
PATH_BOUNDARY_ABS_Y_M = 5.0   # "from start to y=+5" per the task -- stop just short of CPA
N_PATH_SAMPLES = 500
DESIGN_ENVELOPE_RANGES_M = (60.0, 160.0)

# Empirical handoff sample: logs/m4_intercept_pronav_20260708T214600Z.csv,
# r2l-direction pronav flight, first ENGAGE-phase row (the tick after the
# HANDOFF latch fired). l2r is the Y-mirror (documented approximation, see
# module docstring).
HANDOFF_LOG_SOURCE = "logs/m4_intercept_pronav_20260708T214600Z.csv"
HANDOFF_R2L_INTERCEPTOR_XY = (1.863, 6.874)
HANDOFF_R2L_TARGET_XY = (6.500, 16.042)
HANDOFF_R2L_GT_RANGE_M = 10.276


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rig_camera_positions(rx, ry, yaw_rad, baseline_half=RIG_BASELINE_HALF_M):
    """Both cameras' boresight is PARALLEL to yaw_rad (fronto-parallel
    stereo -- no toe-in); they are offset +/-baseline_half along the
    LATERAL axis (perpendicular to the boresight)."""
    lat = (-math.sin(yaw_rad), math.cos(yaw_rad))
    left = (rx - baseline_half * lat[0], ry - baseline_half * lat[1])
    right = (rx + baseline_half * lat[0], ry + baseline_half * lat[1])
    return left, right


def in_camera_cone(cam_xy, yaw_rad, point_xy, half_fov_rad=HALF_FOV_RAD):
    dx, dy = point_xy[0] - cam_xy[0], point_xy[1] - cam_xy[1]
    bearing = math.atan2(dy, dx)
    off = wrap_pi(bearing - yaw_rad)
    rng = math.hypot(dx, dy)
    return abs(off) <= half_fov_rad, rng


def point_in_wedge(rx, ry, yaw_rad, point_xy):
    """A point is in the SHARED wedge iff it is within BOTH cameras' cones.
    Returns (in_wedge, rig_center_range_m)."""
    left, right = rig_camera_positions(rx, ry, yaw_rad)
    ok_l, _ = in_camera_cone(left, yaw_rad, point_xy)
    ok_r, _ = in_camera_cone(right, yaw_rad, point_xy)
    rig_range = math.hypot(point_xy[0] - rx, point_xy[1] - ry)
    return (ok_l and ok_r), rig_range


def dash_path_samples(direction_sign, x0=X0_M, y0_mag=Y0_MAG_M,
                       boundary_abs_y=PATH_BOUNDARY_ABS_Y_M, n=N_PATH_SAMPLES):
    """direction_sign=+1: 'r2l', start Y=+y0_mag moving toward -Y (sub-path
    sampled from start down to Y=+boundary_abs_y, i.e. BEFORE crossing 0).
    direction_sign=-1: 'l2r', start Y=-y0_mag moving toward +Y (sub-path
    sampled from start up to Y=-boundary_abs_y). Both are the "from start to
    y=+/-5" span the task specifies -- CUE_WAIT/DASH through just past the
    10 m HANDOFF, stopping short of the noisy near-CPA tail."""
    start_y = direction_sign * y0_mag
    end_y = direction_sign * boundary_abs_y
    ys = np.linspace(start_y, end_y, n)
    xs = np.full(n, x0)
    return xs, ys


def handoff_point_for_direction(direction_sign):
    """r2l (direction_sign=+1): the measured log sample. l2r (-1): its
    Y-mirror (interceptor and target Y both negated; X unchanged -- see
    module docstring's disclosed approximation)."""
    if direction_sign > 0:
        return HANDOFF_R2L_INTERCEPTOR_XY, HANDOFF_R2L_TARGET_XY
    ix, iy = HANDOFF_R2L_INTERCEPTOR_XY
    tx, ty = HANDOFF_R2L_TARGET_XY
    return (ix, -iy), (tx, -ty)


RIG_POSES = [
    # (label, rig_xy, aim_xy, note)
    ("origin_bore", (0.0, 0.0), (X0_M, 0.0),
     "co-located with the interceptor launch pad, boresight at corridor CPA"),
    ("origin_setback", (-10.0, 0.0), (X0_M, 0.0),
     "10 m behind the pad on the same bore, wider standoff"),
    ("broadside_60m", (X0_M - 60.0, 0.0), (X0_M, 0.0),
     "design-envelope LOWER bound (60 m) standoff, broadside to the corridor"),
    ("broadside_100m", (X0_M - 100.0, 0.0), (X0_M, 0.0),
     "mid design-envelope (100 m) standoff, broadside -- tests whether "
     "standing FARTHER back (wider angular swath at fixed 10 deg half-angle) "
     "beats the closer candidates"),
    ("broadside_160m", (X0_M - 160.0, 0.0), (X0_M, 0.0),
     "design-envelope UPPER bound (160 m) standoff, broadside"),
    ("corridor_end_north", (X0_M, 45.0), (X0_M, 0.0),
     "on the corridor axis extension north of the target start -- COLLINEAR "
     "with the flight line (rx==x0): a real siting hazard despite the math"),
    ("quartering_handoff", (-5.0, 10.0), None,
     "offset pose aimed directly at the r2l empirical HANDOFF point (not "
     "CPA) -- tests whether optimizing for the pivotal moment costs general "
     "coverage, and whether it stays symmetric for l2r"),
]


def run_analysis_2(outdir):
    rows = []
    for label, (rx, ry), aim, note in RIG_POSES:
        if aim is None:
            aim = HANDOFF_R2L_TARGET_XY  # quartering_handoff's aim point
        yaw = math.atan2(aim[1] - ry, aim[0] - rx)
        collinear = abs(rx - X0_M) < 1e-6
        for dsign, dname in ((+1, "r2l"), (-1, "l2r")):
            xs, ys = dash_path_samples(dsign)
            in_wedge = np.zeros(len(xs), dtype=bool)
            rig_ranges = np.zeros(len(xs))
            for i in range(len(xs)):
                iw, rr = point_in_wedge(rx, ry, yaw, (xs[i], ys[i]))
                in_wedge[i] = iw
                rig_ranges[i] = rr
            frac_covered = float(np.mean(in_wedge))

            handoff_xy = handoff_point_for_direction(dsign)[1]
            handoff_y = handoff_xy[1]
            # pre-handoff sub-mask: points between start and the handoff Y
            # (chronologically BEFORE handoff, inclusive), in travel order.
            if dsign > 0:
                pre_mask = ys >= handoff_y
            else:
                pre_mask = ys <= handoff_y
            continuous_to_handoff = bool(np.all(in_wedge[pre_mask])) if pre_mask.any() else False

            handoff_in_wedge, range_at_handoff = point_in_wedge(rx, ry, yaw, handoff_xy)
            range_at_start = math.hypot(xs[0] - rx, ys[0] - ry)
            range_at_boundary = math.hypot(xs[-1] - rx, ys[-1] - ry)

            f_px_design = sm.f_px_from_mm(sm.CHOSEN_FOCAL_MM, sm.CHOSEN_SENSOR)
            sigma_R_expected_at_handoff = float(
                sm.sigma_R(np.array([range_at_handoff]), sm.CHOSEN_BASELINE_M, f_px_design, "expected")[0])
            sigma_R_worst_at_handoff = float(
                sm.sigma_R(np.array([range_at_handoff]), sm.CHOSEN_BASELINE_M, f_px_design, "worst")[0])

            in_60_160 = bool(
                DESIGN_ENVELOPE_RANGES_M[0] <= float(np.min(rig_ranges))
                and float(np.max(rig_ranges)) <= DESIGN_ENVELOPE_RANGES_M[1])

            rows.append({
                "pose": label, "rig_x": rx, "rig_y": ry, "yaw_deg": round(math.degrees(yaw), 2),
                "collinear_with_corridor": collinear, "direction": dname,
                "path_fraction_covered": round(frac_covered, 4),
                "continuous_cuewait_through_handoff": continuous_to_handoff,
                "handoff_point_in_wedge": bool(handoff_in_wedge),
                "range_at_handoff_m": round(range_at_handoff, 2),
                "range_at_path_start_m": round(range_at_start, 2),
                "range_at_boundary_y5_m": round(range_at_boundary, 2),
                "min_range_on_path_m": round(float(np.min(rig_ranges)), 2),
                "max_range_on_path_m": round(float(np.max(rig_ranges)), 2),
                "within_60_160m_design_envelope_whole_path": in_60_160,
                "sigma_R_expected_at_handoff_m": round(sigma_R_expected_at_handoff, 4),
                "sigma_R_worst_at_handoff_m": round(sigma_R_worst_at_handoff, 4),
                "note": note,
            })

    _write_csv(os.path.join(outdir, "coverage.csv"), rows)

    # Recommendation: maximize the WORSE-of-two-directions coverage fraction,
    # then prefer continuous pre-handoff coverage on BOTH directions. Scored
    # TWICE: "best_overall" (any pose) and "best_practical" (non-collinear
    # only -- a rig sitting ON the flight corridor, rx==x0, is a real
    # collision/siting hazard, not a genuine deployable recommendation, even
    # though its geometry scores perfectly; see the doc's honesty note).
    by_pose = {}
    for r in rows:
        by_pose.setdefault(r["pose"], []).append(r)
    scored = []
    for pose, prs in by_pose.items():
        worst_frac = min(p["path_fraction_covered"] for p in prs)
        both_continuous = all(p["continuous_cuewait_through_handoff"] for p in prs)
        collinear = prs[0]["collinear_with_corridor"]
        scored.append((pose, worst_frac, both_continuous, collinear, prs))
    scored.sort(key=lambda t: (-int(t[2]), -t[1]))
    best_overall = scored[0][0]
    practical = [s for s in scored if not s[3]]
    best_practical = practical[0][0] if practical else None
    return rows, best_overall, best_practical, scored


# ============================================================================
# ANALYSIS 3 -- sigma_v vs. update rate (alpha-beta / Kalata)
# ============================================================================
RATES_HZ = [5.0, 10.0, 15.0, 30.0]
ACCEL_CLASSES = [("gentle", 1.0), ("weave", 3.0), ("jink", 9.0)]  # m/s^2, T21 plan
SIGMA_V_TARGET_M_S = 0.5   # s2_cue_mock.py DEFAULT_VEL_SIGMA_M_S / --vel-sigma default


def alpha_beta_variances(sigma_w, sigma_R, T):
    """Eqs (K1)-(K4). Returns (alpha, beta, lambda_idx, var_pos, var_vel)."""
    alpha, beta = kalata_alpha_beta(sigma_w, sigma_R, T)
    lam = sigma_w * T * T / max(sigma_R, 1e-9)
    var_pos = alpha * sigma_R ** 2
    var_vel = (sigma_R ** 2 / (T * T)) * (alpha * beta / max(1.0 - alpha, 1e-9) - (lam ** 2) / 2.0)
    var_vel = max(var_vel, 0.0)
    return alpha, beta, lam, var_pos, var_vel


def filter_lag_m(a_mps2, T, alpha, beta):
    """Eq (K5): steady-state position lag under constant acceleration a."""
    return a_mps2 * T * T * (1.0 - alpha) / max(beta, 1e-9)


def run_analysis_3(outdir, res_summary_rows, a2_rows):
    """a2_rows: analysis-2's full coverage rows -- EVERY candidate pose's
    handoff range is swept here (not just the recommended one), so the
    coverage-vs-cue-quality tradeoff (closer poses => tiny sigma_R but poor
    wedge coverage; farther poses => wide coverage but large sigma_R) is
    directly traceable in the CSV, not asserted from a scratch calculation.

    sigma_R here uses the FULL 4-term `stereo_model.sigma_R()` RSS budget
    (match+cal+sync+dist) at each resolution's fixed-HFOV f_px, NOT the
    simple matching-only c from analysis 1 -- more rigorous, and it is what
    analysis 1's own cross-check column already validates. (Small, disclosed
    rounding note: f_px_at_fixed_hfov(1920)=5444 vs. the exact chosen-lens
    design f_px=5333 from stereo_model's 16 mm/AR0234 pitch -- both encode
    "~20 deg HFOV" per docs/stereo_design.md; the ~2% gap does not move any
    conclusion here.)"""
    range_points = [("design_envelope_100m", 100.0)]
    seen_poses = set()
    for row in a2_rows:
        if row["pose"] in seen_poses:
            continue
        seen_poses.add(row["pose"])
        range_points.append((f"handoff_{row['pose']}", None))  # value unused, per-direction below

    rows = []
    for w, h in RESOLUTIONS:
        f_px = f_px_at_fixed_hfov(w)
        c_expected = scaled_c_matching_only(f_px, "expected")  # kept in CSV for reference
        for range_label, fixed_R in range_points:
            if range_label == "design_envelope_100m":
                r_variants = [("both", fixed_R)]
            else:
                pose_name = range_label[len("handoff_"):]
                r_variants = [(row["direction"], row["range_at_handoff_m"])
                              for row in a2_rows if row["pose"] == pose_name]
            for dir_label, R in r_variants:
                sigma_R_expected = float(sm.sigma_R(np.array([R]), sm.CHOSEN_BASELINE_M, f_px, "expected")[0])
                for rate_hz in RATES_HZ:
                    T = 1.0 / rate_hz
                    for accel_label, a_mps2 in ACCEL_CLASSES:
                        alpha, beta, lam, var_pos, var_vel = alpha_beta_variances(
                            a_mps2, sigma_R_expected, T)
                        sigma_v = math.sqrt(var_vel)
                        lag = filter_lag_m(a_mps2, T, alpha, beta)
                        rows.append({
                            "width_px": w, "height_px": h, "f_px": round(f_px, 1),
                            "c_expected_per_m2": c_expected,
                            "range_label": range_label, "range_direction": dir_label,
                            "R_m": round(R, 2), "sigma_R_expected_m": round(sigma_R_expected, 5),
                            "rate_hz": rate_hz, "T_s": round(T, 4),
                            "accel_class": accel_label, "accel_mps2": a_mps2,
                            "lambda_idx": round(lam, 5),
                            "alpha": round(alpha, 5), "beta": round(beta, 5),
                            "sigma_v_mps": round(sigma_v, 4),
                            "lag_m": round(lag, 4),
                            "meets_0.5mps": sigma_v <= SIGMA_V_TARGET_M_S,
                        })
    _write_csv(os.path.join(outdir, "sigma_v_vs_rate.csv"), rows)
    return rows


# ============================================================================
# I/O helpers
# ============================================================================
def _write_csv(path, rows):
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"[rig-geometry] wrote {path} ({len(rows)} rows)")


# ============================================================================
# Plots (optional)
# ============================================================================
def make_plots(a1_long, a2_rows, a3_rows, best_overall, best_practical):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(PLOTS_DIR, exist_ok=True)

    # --- Plot 1: sigma_R (full model) vs range, one line per resolution ---
    fig, ax = plt.subplots(figsize=(8, 5.5))
    colors = plt.cm.viridis(np.linspace(0, 0.85, len(RESOLUTIONS)))
    for (w, h), color in zip(RESOLUTIONS, colors):
        for tier, ls in (("expected", "-"), ("worst", "--")):
            f_px = f_px_at_fixed_hfov(w)
            R = np.linspace(5, 200, 300)
            sig = sm.sigma_R(R, sm.CHOSEN_BASELINE_M, f_px, tier)
            ax.plot(R, sig, color=color, ls=ls, lw=1.6,
                    label=f"{w}x{h} ({tier})" if tier == "expected" else None)
    ax.axhline(A1_SIGMA_GATE_M, color="k", ls=":", lw=0.8, label="1 m gate")
    ax.set_xlabel("range R (m)")
    ax.set_ylabel("sigma_R, full model (m)")
    ax.set_ylim(0, 3)
    ax.set_title("Resolution-scaled sigma_R (solid=EXPECTED, dashed=WORST)\nfixed HFOV=20 deg, b=2.0 m")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    p1 = os.path.join(PLOTS_DIR, "rig_geometry_res_sigmaR.png")
    fig.savefig(p1, dpi=110)
    plt.close(fig)

    # --- Plot 2: top-down wedge-coverage map ---
    fig, ax = plt.subplots(figsize=(9, 9))
    for dsign, dname, color in ((+1, "r2l", "tab:red"), (-1, "l2r", "tab:blue")):
        xs, ys = dash_path_samples(dsign, n=200)
        ax.plot(xs, ys, color=color, lw=2.5, alpha=0.5, label=f"dash path ({dname})")
        _, tgt = handoff_point_for_direction(dsign)
        ax.plot(tgt[0], tgt[1], marker="x", color=color, ms=10, mew=2)
    for label, (rx, ry), aim, note in RIG_POSES:
        if aim is None:
            aim = HANDOFF_R2L_TARGET_XY
        yaw = math.atan2(aim[1] - ry, aim[0] - rx)
        if label == best_practical:
            marker_color = "tab:green"
        elif label == best_overall:
            marker_color = "tab:orange"
        else:
            marker_color = "0.4"
        ax.plot(rx, ry, marker="s", color=marker_color, ms=9)
        ax.annotate(label, (rx, ry), fontsize=7, xytext=(4, 4), textcoords="offset points")
        # draw the shared wedge boundary rays out to 50 m
        for sgn in (-1, 1):
            ang = yaw + sgn * HALF_FOV_RAD
            ax.plot([rx, rx + 55 * math.cos(ang)], [ry, ry + 55 * math.sin(ang)],
                     color=marker_color, lw=0.8, alpha=0.6)
    ax.plot(0, 0, marker="^", color="black", ms=10, label="interceptor launch (origin)")
    ax.plot([], [], marker="s", color="tab:green", ls="", label="best PRACTICAL (non-collinear)")
    ax.plot([], [], marker="s", color="tab:orange", ls="", label="best OVERALL (may be collinear/degenerate)")
    ax.set_xlabel("world X (m)")
    ax.set_ylabel("world Y (m)")
    ax.set_title("Rig-pose candidates vs. the flown dash corridor (x = 6.5 m)\n"
                 "squares = rig, thin rays = wedge half-angle bounds, x = HANDOFF point")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    p2 = os.path.join(PLOTS_DIR, "rig_geometry_wedge_coverage.png")
    fig.savefig(p2, dpi=110)
    plt.close(fig)

    # --- Plot 3: sigma_v vs rate, per accel class, design resolution, both ranges ---
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5), sharey=True)
    practical_label = f"handoff_{best_practical}"
    range_labels = [practical_label, "design_envelope_100m"]
    accel_colors = {"gentle": "tab:green", "weave": "tab:orange", "jink": "tab:red"}
    for ax, rlabel in zip(axes, range_labels):
        for accel_label in ("gentle", "weave", "jink"):
            sub = [r for r in a3_rows if r["range_label"] == rlabel
                   and r["accel_class"] == accel_label
                   and r["width_px"] == 1920]
            if rlabel == practical_label:
                # average across directions per rate for a single line
                by_rate = {}
                for r in sub:
                    by_rate.setdefault(r["rate_hz"], []).append(r["sigma_v_mps"])
                rates = sorted(by_rate)
                vals = [np.mean(by_rate[rt]) for rt in rates]
            else:
                sub.sort(key=lambda r: r["rate_hz"])
                rates = [r["rate_hz"] for r in sub]
                vals = [r["sigma_v_mps"] for r in sub]
            ax.plot(rates, vals, marker="o", color=accel_colors[accel_label], label=accel_label)
        ax.axhline(SIGMA_V_TARGET_M_S, color="k", ls="--", lw=0.8, label="0.5 m/s target")
        ax.set_xlabel("update rate (Hz)")
        ax.set_title(rlabel)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("sigma_v (m/s)")
    axes[0].legend(fontsize=8)
    fig.suptitle("sigma_v vs. update rate, 1920x1200 design resolution, EXPECTED sigma_R")
    fig.tight_layout()
    p3 = os.path.join(PLOTS_DIR, "rig_geometry_sigma_v_vs_rate.png")
    fig.savefig(p3, dpi=110)
    plt.close(fig)

    print(f"[rig-geometry] wrote plots:\n  {p1}\n  {p2}\n  {p3}")


# ============================================================================
# main
# ============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-plots", action="store_true", help="skip matplotlib output")
    ap.add_argument("--outdir", default=LOGS_OUT_DIR,
                    help=f"CSV output dir (default {LOGS_OUT_DIR})")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    print("=" * 78)
    print("ANALYSIS 1 -- resolution-scaled sigma_R + detection floor")
    print("=" * 78)
    a1_long, a1_summary = run_analysis_1(args.outdir)
    hdr = f"{'res':>10} {'f_px':>7} {'tier':>9} {'c(m/m^2)':>11} {'Rmax@1m(full)':>14} {'detect(0.3m/10px)':>18}"
    print(hdr)
    for row in a1_summary:
        print(f"{row['width_px']:>4}x{row['height_px']:<5} {row['f_px']:>7.0f} {row['tier']:>9} "
              f"{row['c_scaled_matching_only_per_m2']:>11.3e} "
              f"{row['max_range_sigmaR_le_1m_full_model_m']:>14.1f} "
              f"{row['detect_floor_0.3m_10px_m']:>18.1f}")

    print("\n" + "=" * 78)
    print("ANALYSIS 2 -- FoV-wedge coverage vs. the flown dash profile")
    print("=" * 78)
    a2_rows, best_overall, best_practical, scored = run_analysis_2(args.outdir)
    hdr2 = f"{'pose':>20} {'dir':>4} {'frac_cov':>9} {'cont.handoff':>13} {'R@handoff':>10} {'collinear':>10}"
    print(hdr2)
    for row in a2_rows:
        print(f"{row['pose']:>20} {row['direction']:>4} {row['path_fraction_covered']:>9.3f} "
              f"{str(row['continuous_cuewait_through_handoff']):>13} "
              f"{row['range_at_handoff_m']:>10.2f} {str(row['collinear_with_corridor']):>10}")
    print(f"\n[rig-geometry] BEST OVERALL pose (max worst-direction coverage, may be "
          f"collinear/degenerate -- see honesty note): {best_overall}")
    print(f"[rig-geometry] BEST PRACTICAL pose (non-collinear, genuinely siteable): "
          f"{best_practical}")

    print("\n" + "=" * 78)
    print("ANALYSIS 3 -- sigma_v vs. update rate (Kalata alpha-beta)")
    print("=" * 78)
    a3_rows = run_analysis_3(args.outdir, a1_summary, a2_rows)
    practical_label = f"handoff_{best_practical}"
    print(f"{'res':>10} {'rate_hz':>7} {'accel':>7} {'range_label':>26} {'sigma_R':>8} "
          f"{'alpha':>6} {'beta':>6} {'sigma_v':>8} {'meets0.5':>8}")
    for row in a3_rows:
        if row["width_px"] == 1920 and row["range_label"] in (
                practical_label, "design_envelope_100m") and row["range_direction"] != "l2r":
            print(f"{row['width_px']}x{row['height_px']:<5} {row['rate_hz']:>7.0f} "
                  f"{row['accel_class']:>7} {row['range_label']:>26} "
                  f"{row['sigma_R_expected_m']:>8.3f} {row['alpha']:>6.3f} {row['beta']:>6.3f} "
                  f"{row['sigma_v_mps']:>8.3f} {str(row['meets_0.5mps']):>8}")

    # Headline: minimum (rate, resolution) meeting 0.5 m/s for the WORST
    # (jink, 9 m/s^2) class at the BEST-PRACTICAL pose's handoff range --
    # the operationally binding case.
    jink_handoff = [r for r in a3_rows if r["accel_class"] == "jink"
                    and r["range_label"] == practical_label
                    and r["meets_0.5mps"]]
    best = None
    for r in sorted(jink_handoff, key=lambda r: (r["rate_hz"], -r["width_px"])):
        best = r
        break
    print("\n[rig-geometry] Minimum (rate, resolution) meeting sigma_v<=0.5 m/s "
          f"for JINK (9 m/s^2) at the best-practical pose's handoff range: "
          f"{best['rate_hz']:.0f} Hz @ {best['width_px']}x{best['height_px']}"
          if best else "\n[rig-geometry] NO (rate,resolution) combination tested meets "
                       f"sigma_v<=0.5 m/s for JINK at {best_practical}'s handoff range "
                       f"({[r['R_m'] for r in a3_rows if r['range_label']==practical_label][0]:.0f} m).")

    if not args.no_plots:
        make_plots(a1_long, a2_rows, a3_rows, best_overall, best_practical)

    print(f"\n[rig-geometry] all CSVs under {args.outdir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
