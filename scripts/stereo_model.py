#!/usr/bin/env python3
r"""Ground stereo-rig design model (ADR-0015 P-2): turn "~2 m baseline" into a
physics-derived design point with calibrated constants the sim can consume.

This is a PURE-MODEL script (no Gazebo, no PX4, no hardware). It answers the
P-2 question from docs/next.md: "camera baseline vs usable range vs 3D triangulation
accuracy vs cost -- the optimal distance to cover good range while remaining
accurate and affordable. Where's the cost/accuracy knee?" It exports the
range-error constants (a, c) for scripts/s2_cue_mock.py's sigma_R = a + c*R**2
model, DERIVED from stereo geometry + a term-by-term outdoor error budget,
rather than the hand-set 0.4 + 0.008*R**2 the mock ships with today.

METHODOLOGY (builder mandate, non-negotiable -- see docs/stereo_design.md):
every quantity is reported in THREE TIERS -- BEST (datasheet / lab bench),
EXPECTED (realistic field rig), WORST-CREDIBLE (sun-heated, wind-shaken,
imperfectly calibrated cheap outdoor rig). Design conclusions come from
EXPECTED; WORST is always shown alongside; BEST is never the headline. This
project has FOUR documented data points of lab models over-promising vs Gazebo
reality (ADR-0011/0014/0015), so we simulate worse-than-ideal on purpose. Every
model parameter maps to a physically MEASURABLE quantity so a real bench test
(ADR-0015 build-plan step 4: static stereo-vs-mono range accuracy at 50-150 m)
can replace it 1:1.

=======================================================================
DERIVATION 1 -- STEREO RANGE ERROR (the standard formula)
=======================================================================
A stereo pair with baseline b (camera separation, m) and focal length f_px
(pixels) images a point at range R (m). The two cameras see it shifted by the
DISPARITY d (pixels):

        d = b * f_px / R                                             (1)

Invert for range and differentiate to propagate a disparity error sigma_d into
a range error:

        R = b * f_px / d
        dR/dd = -b * f_px / d**2 = -R**2 / (b * f_px)               (2)

    =>  sigma_R = R**2 / (b * f_px) * sigma_d                       (3)

This is the classic result (Teledyne "Stereo Accuracy and Error Modeling":
dZ = Z**2/(B*f) * m). The R**2 is the whole story of stereo: error grows with
the SQUARE of range. Doubling the baseline or focal length halves the error;
doubling the range quadruples it. Sources:
  - Teledyne Vision Solutions, "Stereo Accuracy and Error Modeling"
    https://www.teledynevisionsolutions.com/support/support-center/application-note/iis/stereo-accuracy-and-error-modeling/
  - "Know Your Limits: Accuracy of Long Range Stereoscopic Object Measurements
    in Practice" (Springer, 2014)
    https://link.springer.com/chapter/10.1007/978-3-319-10605-2_7

=======================================================================
DERIVATION 2 -- THE ERROR BUDGET (what naive stereo analyses omit)
=======================================================================
sigma_d in (3) is NOT just sub-pixel matching noise. It is the RSS of four
physically distinct disparity-error sources. We propagate EACH through (3)
(or its analogue) so the reader sees which term dominates at range:

  (A) Sub-pixel matching noise, sigma_match [px]
      How precisely the correlator locates the same feature in both images.
      Lab: ~0.11 px (Teledyne). Outdoor, small/blurred/low-contrast target:
      0.3-0.5 px expected, up to 1.0 px worst.
          sigma_R_match(R) = R**2 * sigma_match / (b * f_px)         (4)

  (B) Calibration / extrinsic pointing drift, delta_theta [rad]
      A tiny CHANGE in the relative rotation (toe-in) of the two cameras
      shifts the whole disparity map by delta_theta * f_px pixels. On a 2 m
      OUTDOOR bar with no optical bench, thermal expansion (aluminium CTE
      ~23 ppm/degC), self-heating, and wind flex move the extrinsics between
      recalibrations. Propagating an equivalent disparity delta_theta*f_px:
          sigma_R_cal(R) = R**2 * (delta_theta * f_px) / (b * f_px)
                         = R**2 * delta_theta / b                    (5)
      *** KEY RESULT: the f_px CANCELS. Calibration/pointing drift produces a
      range error R**2 * delta_theta / b that NO focal length can fix -- only a
      wider baseline or a more rigid / more frequently recalibrated mount
      helps. This is the term naive "just use a long lens" analyses miss. ***
      Sources (thermal drift of stereo extrinsics):
        - "Modeling of systematic errors in stereo-DIC due to camera
          self-heating", Sci. Reports 2019
          https://www.nature.com/articles/s41598-019-43019-7
        - "Effect of camera temperature variations on stereo-DIC measurements"
          https://www.researchgate.net/publication/284515151
        - Teledyne (above): calibration pointing error p = 0.06-0.08 px
          (640x480) .. 0.10-0.15 px (1024x768), i.e. tens of microrad residual.

  (C) Camera-sync error, dt_sync [s], times target angular rate
      The two cameras must expose the SAME instant. If they are dt_sync apart,
      a target moving with transverse speed v_perp has moved v_perp*dt_sync
      between the frames, injecting an apparent disparity f_px*v_perp*dt_sync/R.
      Propagating:
          sigma_R_sync(R) = R * v_perp * dt_sync / b                 (6)
      (Note: LINEAR in R, not quadratic -- it fades at range for fixed v_perp.)
      With a HARDWARE GPIO trigger (dt_sync ~ 2-20 us) this term is
      negligible; with software/PTP-only sync (dt_sync ~ 1 ms) it can reach
      ~1 m at 100 m -- which is exactly why the design MANDATES a hardware
      trigger. Sources:
        - Arducam external-trigger (hardware-line, sub-frame sync)
          https://docs.arducam.com/Nvidia-Jetson-Camera/Global-Shutter-Camera/external-trigger/
        - GigE Vision PTP (IEEE-1588) gives us-level network sync.

  (D) Lens-distortion residual, sigma_dist [px]
      After calibration, residual radial/tangential distortion is ~0.1-0.3 px
      near frame CENTRE, rising to 1-2 px at the FOV EDGE with a cheap lens.
      Because the tracked target is kept near centre, we carry a modest central
      budget (0.05 / 0.2 / 0.5 px) and note the edge penalty (argues for a
      long-enough lens that the target stays central, and against relying on
      the frame corners). Propagated like (4).

  Total:  sigma_R(R) = sqrt( sigma_R_match**2 + sigma_R_cal**2
                             + sigma_R_sync**2 + sigma_R_dist**2 )   (7)

=======================================================================
DERIVATION 3 -- CROSS-RANGE (BEARING) ERROR
=======================================================================
The fused track needs BOTH range and cross-range. Cross-range (lateral, X/Y)
error does NOT carry the disparity R**2 penalty; it is set by how well we can
point-localise the target angularly (Teledyne: dX = Z/f * p). Combining the
calibration pointing error p with the target-centroid localisation error
sigma_centroid (both in px):

        sigma_cross(R) = R / f_px * sqrt(p**2 + sigma_centroid**2)   (8)

This grows only LINEARLY with R, so it is ~2 orders of magnitude smaller than
sigma_R at 100 m (cm-level vs ~1 m). Confirms ADR-0015's "both sensors are
angle-STRONG, range-WEAK". CAVEAT surfaced in the report: once the ground
track is expressed in the DRONE's own frame, cross-range is dominated NOT by
this cm-level stereo precision but by the cross-platform GPS DATUM offset
(0.5 m RTK .. 2.5 m standard GPS) -- ADR-0015's central correction. sigma_cross
here is the rig's INTRINSIC bearing precision; the datum floor is added on top.

=======================================================================
DERIVATION 4 -- PIXELS-ON-TARGET AND THE DETECTION FLOOR
=======================================================================
A rig that "ranges to 200 m" is dishonest if it cannot DETECT the drone past
80 m. A target of physical size L (m) subtends, at range R:

        n_px(R) = f_px * L / R   [pixels across]                     (9)

Detection probability is modelled as a logistic in n_px, anchored to Johnson's
criteria (line-pairs across a target for a task at ~50% probability, x2 for
pixels): DETECTION ~2 px, RECOGNITION ~8 px, IDENTIFICATION ~13 px. Because the
#1 fielded C-UAS failure is BIRD false-alarms (ADR-0015), a CONFIDENT, classified,
bird-rejected track needs recognition-class pixels, not bare detection. We
therefore require n_px >= n_90 (px for P_detect=0.9), tiered
5 / 10 / 18 px (BEST/EXPECTED/WORST). A pure single-frame YOLO is "nearly
undetectable" below ~32 px; DETECT-THEN-TRACK (motion proposals + tracker,
ADR-0015 unanimous #2) is what pulls the floor down to a few px, lifting
tiny-target recall 0.405 -> 0.861. Sources:
  - Johnson's criteria: https://en.wikipedia.org/wiki/Johnson%27s_criteria
  - YOLO sub-32px limit / VisDrone tiny-object stats (multiple 2025-2026 papers)
  - detect-then-track recall 0.405->0.861: docs/perception_design.md / ADR-0015 #2

The DETECTION-RANGE FLOOR is the largest R with n_px(R) >= n_90:
        R_detect = f_px * L / n_90                                  (10)

=======================================================================
DERIVATION 5 -- USABLE ENVELOPE and the FIT the sim consumes
=======================================================================
USABLE ENVELOPE = the range band where BOTH (a) sigma_R(R) <= a threshold
(default 1.0 m, the ADR-0015 "sub-meter" hope) AND (b) R <= R_detect. An honest
design point has R_detect comfortably >= R(sigma_R<=1m); if detection is the
binding constraint the rig is over-lensed for its eyes.

The sim (s2_cue_mock.py) consumes sigma_R as a + c*R**2. Since (4),(5),(D) are
all pure R**2 and (6) is a small linear term, a least-squares fit of a + c*R**2
to the modelled sigma_R over the usable envelope recovers c = the RSS of the
R**2 coefficients and a ~ 0 (stereo NOISE vanishes at short range). The GPS
DATUM offset (0.5 / 2.5 m) is a per-run CONSTANT BIAS, kept in the mock's
SEPARATE --datum-bias-m knob, NOT folded into c. See the report for the
reconciliation with the mock's current 0.4 + 0.008*R**2.

Run:
    .venv/bin/python scripts/stereo_model.py            # full study + plots
    .venv/bin/python scripts/stereo_model.py --no-plots # tables only
"""

import argparse
import math
import os

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLOTS_DIR = os.path.join(REPO_ROOT, "plots")

# ----------------------------------------------------------------------------
# SENSOR PRESETS. Pixel pitch and array width set f_px for a given lens.
#   f_px = f_mm / pitch_mm ;  HFOV = 2*atan(width_px * pitch / (2*f))
# AR0234: onsemi 1/2.6", 1920x1200, 3.0 um pixels -- the Arducam Jetson-native
#   global-shutter module (B0429), the natural choice for a Jetson Orin NX rig.
# IMX296: Sony 1/2.9", 1456x1088, 3.45 um pixels -- the Raspberry-Pi-class GS
#   module (Arducam/InnoMaker), cheaper, lower-res; carried for comparison.
# ----------------------------------------------------------------------------
SENSORS = {
    "AR0234": {"pitch_um": 3.0, "width_px": 1920, "height_px": 1200},
    "IMX296": {"pitch_um": 3.45, "width_px": 1456, "height_px": 1088},
}

# Candidate C-mount focal lengths (mm). 12=wide search, 25=medium, 50=narrow
# reach -- the "2-3 candidate focal lengths" of the P-2 trade table. 16 mm is
# the CHOSEN design point (printed separately); 8 shown as a wide extreme.
FOCALS_MM = [8, 12, 16, 25, 50]
TRADE_FOCALS_MM = [12, 25, 50]           # 3 candidates for the printed trade table
CHOSEN_FOCAL_MM = 16
CHOSEN_SENSOR = "AR0234"
CHOSEN_BASELINE_M = 2.0

BASELINES_M = [1, 2, 3, 4, 6]            # P-2 baseline sweep

# ----------------------------------------------------------------------------
# THREE-TIER ERROR-BUDGET PARAMETERS. Every value maps to a bench-measurable
# quantity (docs/stereo_design.md "bench-test procedure"). Sources in comments;
# where sources conflict we take the PESSIMISTIC value.
# ----------------------------------------------------------------------------
TIERS = ("best", "expected", "worst")

PARAMS = {
    # (A) sub-pixel matching noise [px]. Teledyne lab 0.11; outdoor small/blur
    # 0.3-0.5; "wrong by 1 px" worst. Measurable: static two-camera test vs a
    # surveyed target, std of disparity residuals.
    "sigma_match_px": {"best": 0.10, "expected": 0.40, "worst": 1.00},

    # (B) relative pointing / extrinsic stability at b=2 m, between recals [rad].
    # Teledyne calibration pointing p=0.06-0.15 px ~= 11-28 urad residual (lab).
    # Outdoor thermal self-heating + ambient + wind flex inflate it. Measurable:
    # re-run calibration over a day, watch the epipolar/extrinsic residual drift.
    "delta_theta_rad": {"best": 5e-6, "expected": 30e-6, "worst": 150e-6},

    # (B') baseline rigidity exponent: delta_theta(b) = delta_theta_ref*(b/2)^p.
    # 0 = rigidity scaled with length (CFRP/invar truss, temp control);
    # 1 = linear degradation (plain aluminium extrusion);
    # 2 = quadratic (thin bar, wind moment, sun gradient -- beam self-weight
    # end-slope scales ~L^3 for a fixed section; 2 is a mid-pessimistic bound).
    "rigidity_exp": {"best": 0.0, "expected": 1.0, "worst": 2.0},

    # (C) camera-sync error [s]. HW GPIO trigger 2 us; cheap-board readout skew
    # 20 us; software/PTP-only fallback 1 ms (the "forgot to hardware-trigger"
    # failure). Measurable: strobe both cameras at a fast LED, compare timestamps.
    "dt_sync_s": {"best": 2e-6, "expected": 2e-5, "worst": 1e-3},
    # transverse target speed used only for the sync term [m/s]: a fast crosser.
    "v_perp_ms": {"best": 10.0, "expected": 15.0, "worst": 25.0},

    # (D) lens-distortion residual near frame centre [px] (edge is 1-2 px worse).
    # Measurable: reprojection-error map after calibration.
    "sigma_dist_px": {"best": 0.05, "expected": 0.20, "worst": 0.50},

    # cross-range: calibration pointing p [px] (Teledyne) and target-centroid
    # localisation sigma_centroid [px]. Measurable: centroid jitter on a static
    # distant target.
    "p_point_px": {"best": 0.06, "expected": 0.12, "worst": 0.20},
    "sigma_centroid_px": {"best": 0.20, "expected": 0.50, "worst": 1.50},

    # detection: target physical size L [m] (5"-7" FPV, tip-to-tip; worst =
    # small/partly-visible) and n_90 [px across for P_detect=0.9], anchored to
    # Johnson (detection~2, recognition~8, identification~13 px).
    "target_L_m": {"best": 0.35, "expected": 0.30, "worst": 0.20},
    "n90_px": {"best": 5.0, "expected": 10.0, "worst": 18.0},

    # GPS/clock DATUM floor [m], REPORTED separately (mock --datum-bias-m knob),
    # NOT fitted into c. RTK+PPS 0.5 m; standard GPS 2.5 m (ADR-0015).
    "datum_floor_m": {"best": 0.3, "expected": 0.5, "worst": 2.5},
}

SIGMA_R_THRESHOLD_M = 1.0    # usable-envelope range-accuracy gate (ADR-0015 "sub-meter")
SIGMA_R_TIGHT_M = 0.5        # also report the tighter <=0.5 m range
P_DETECT_GATE = 0.90         # usable-envelope detection gate

# The constants s2_cue_mock.py ships with today, for the reconciliation printout.
SIM_CURRENT_A = 0.4
SIM_CURRENT_C = 0.008


# ============================================================================
# CORE PHYSICS FUNCTIONS
# ============================================================================
def f_px_from_mm(f_mm, sensor):
    """Focal length in pixels: f_px = f_mm / pixel_pitch_mm."""
    return f_mm / (SENSORS[sensor]["pitch_um"] * 1e-3)


def hfov_deg(f_mm, sensor):
    """Horizontal field of view (deg) = 2*atan(width_px * pitch / (2 f))."""
    s = SENSORS[sensor]
    half = s["width_px"] * (s["pitch_um"] * 1e-3) / (2.0 * f_mm)
    return math.degrees(2.0 * math.atan(half))


def delta_theta_of_b(b_m, tier):
    """Relative pointing stability at baseline b (rad): scales from the b=2 m
    reference by the tier's rigidity exponent (a longer bar is harder to hold)."""
    ref = PARAMS["delta_theta_rad"][tier]
    p = PARAMS["rigidity_exp"][tier]
    return ref * (b_m / 2.0) ** p


def sigma_R_terms(R, b_m, f_px, tier):
    """The four range-error terms (m) at range R, eqs (4),(5),(6),(D). Returns
    a dict so callers can see which dominates. R may be a numpy array."""
    sm = PARAMS["sigma_match_px"][tier]
    sd = PARAMS["sigma_dist_px"][tier]
    dtheta = delta_theta_of_b(b_m, tier)
    dt = PARAMS["dt_sync_s"][tier]
    vp = PARAMS["v_perp_ms"][tier]
    return {
        "match": R**2 * sm / (b_m * f_px),          # (4)
        "cal": R**2 * dtheta / b_m,                 # (5)  -- f_px cancels
        "sync": R * vp * dt / b_m,                  # (6)  -- linear in R
        "dist": R**2 * sd / (b_m * f_px),           # (D)
    }


def sigma_R(R, b_m, f_px, tier):
    """Total stereo range error (m), RSS of the four terms, eq (7)."""
    t = sigma_R_terms(R, b_m, f_px, tier)
    return np.sqrt(t["match"] ** 2 + t["cal"] ** 2 + t["sync"] ** 2 + t["dist"] ** 2)


def sigma_cross(R, f_px, tier):
    """Cross-range (bearing) error (m), eq (8). Rig-intrinsic; the drone-frame
    cross-range is dominated by the GPS datum floor on top of this."""
    p = PARAMS["p_point_px"][tier]
    sc = PARAMS["sigma_centroid_px"][tier]
    return R / f_px * math.sqrt(p**2 + sc**2)


def pixels_on_target(R, f_px, tier):
    """Pixels across the target, eq (9): n_px = f_px * L / R."""
    L = PARAMS["target_L_m"][tier]
    return f_px * L / R


def p_detect(R, f_px, tier):
    """Detection probability, logistic in pixels-on-target, anchored so
    P(n_90)=0.9 with a Johnson-like slope. n_50 = n_90 * 0.5 (recognition-class
    midpoint). k chosen so the logistic passes through (n_90, 0.9)."""
    n = pixels_on_target(R, f_px, tier)
    n90 = PARAMS["n90_px"][tier]
    n50 = 0.5 * n90
    # logistic P = 1/(1+exp(-k(n-n50))); solve k from P(n90)=0.9:
    #   ln(0.9/0.1) = k (n90-n50)  ->  k = ln(9)/(n90-n50)
    k = math.log(9.0) / (n90 - n50)
    return 1.0 / (1.0 + np.exp(-k * (n - n50)))


def detection_range_floor(f_px, tier):
    """Largest range (m) with n_px >= n_90 (confident classified track), eq (10)."""
    L = PARAMS["target_L_m"][tier]
    n90 = PARAMS["n90_px"][tier]
    return f_px * L / n90


def range_at_sigma(b_m, f_px, tier, sigma_max):
    """Largest range (m) with sigma_R(R) <= sigma_max. The sync term is linear so
    we solve robustly by dense sampling rather than a closed quadratic-in-R**2."""
    Rs = np.linspace(1.0, 1000.0, 200000)
    s = sigma_R(Rs, b_m, f_px, tier)
    ok = Rs[s <= sigma_max]
    return float(ok.max()) if ok.size else 0.0


def usable_envelope(b_m, f_px, tier, sigma_max=SIGMA_R_THRESHOLD_M):
    """Range band where BOTH sigma_R<=sigma_max AND P_detect>=gate. Returns
    (R_lo, R_hi, binding) where binding says which constraint caps R_hi."""
    r_sigma = range_at_sigma(b_m, f_px, tier, sigma_max)
    r_det = detection_range_floor(f_px, tier)
    r_hi = min(r_sigma, r_det)
    binding = "detection" if r_det < r_sigma else "range-accuracy"
    return 0.0, r_hi, binding


def fit_quadratic(R, sigma):
    """Least-squares fit sigma ~ a + c*R**2. Returns (a, c, rms_residual_m)."""
    A = np.column_stack([np.ones_like(R), R**2])
    coef, *_ = np.linalg.lstsq(A, sigma, rcond=None)
    a, c = float(coef[0]), float(coef[1])
    resid = sigma - (a + c * R**2)
    return a, c, float(np.sqrt(np.mean(resid**2)))


# ============================================================================
# CLI SECTIONS
# ============================================================================
def print_header(title):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def section_sensor_geometry():
    print_header("SENSOR + LENS GEOMETRY (f_px = f_mm / pitch; HFOV over the sensor)")
    for sensor in SENSORS:
        s = SENSORS[sensor]
        print(f"\n{sensor}: {s['width_px']}x{s['height_px']} px, {s['pitch_um']:.2f} um pitch")
        print(f"  {'f_mm':>6} {'f_px':>8} {'HFOV_deg':>9}")
        for f_mm in FOCALS_MM:
            print(f"  {f_mm:>6} {f_px_from_mm(f_mm, sensor):>8.0f} {hfov_deg(f_mm, sensor):>9.1f}")


def section_trade_table():
    print_header(
        "TRADE TABLE -- baseline b x focal x tier. R@1m = range where sigma_R<=1 m; "
        "R_det = detection floor (n_px>=n_90); Ruse = usable envelope; s100 = sigma_R at 100 m."
    )
    print(f"(sensor {CHOSEN_SENSOR}; sigma_R threshold {SIGMA_R_THRESHOLD_M:.1f} m; "
          f"P_detect gate {P_DETECT_GATE:.2f})\n")
    hdr = (f"{'b(m)':>4} {'f(mm)':>5} {'HFOV':>5} {'tier':>9} "
           f"{'R@1m':>7} {'R_det':>7} {'Ruse':>7} {'bind':>13} {'s@100':>7}")
    print(hdr)
    print("-" * len(hdr))
    for b in BASELINES_M:
        for f_mm in TRADE_FOCALS_MM:
            f_px = f_px_from_mm(f_mm, CHOSEN_SENSOR)
            fov = hfov_deg(f_mm, CHOSEN_SENSOR)
            for tier in TIERS:
                r1 = range_at_sigma(b, f_px, tier, SIGMA_R_THRESHOLD_M)
                rdet = detection_range_floor(f_px, tier)
                _, ruse, binding = usable_envelope(b, f_px, tier)
                s100 = float(sigma_R(np.array([100.0]), b, f_px, tier)[0])
                print(f"{b:>4} {f_mm:>5} {fov:>5.1f} {tier:>9} "
                      f"{r1:>7.0f} {rdet:>7.0f} {ruse:>7.0f} {binding:>13} {s100:>7.2f}")
        print("-" * len(hdr))


def section_knee():
    print_header("COST/ACCURACY KNEE -- why b~2 m (not 1, not 6)")
    f_px = f_px_from_mm(CHOSEN_FOCAL_MM, CHOSEN_SENSOR)
    print(f"(chosen {CHOSEN_SENSOR} + {CHOSEN_FOCAL_MM} mm, f_px={f_px:.0f}; "
          f"sigma_R at R=100 m vs baseline)\n")
    print(f"  {'b(m)':>4} {'best':>8} {'expected':>9} {'worst':>8}   marginal(EXPECTED)")
    prev = None
    for b in BASELINES_M:
        vals = {t: float(sigma_R(np.array([100.0]), b, f_px, t)[0]) for t in TIERS}
        marg = "" if prev is None else f"d={prev - vals['expected']:+.3f} m"
        print(f"  {b:>4} {vals['best']:>8.3f} {vals['expected']:>9.3f} {vals['worst']:>8.3f}   {marg}")
        prev = vals["expected"]

    # Interior optimum of the WORST tier (where a floppier longer bar starts to
    # LOSE): minimise (k_match/b)^2 + (k_cal*b)^2 -> b_opt = sqrt(k_match/k_cal).
    print("\nWORST-tier interior optimum (rigidity_exp=2 -> a longer bar gets floppier):")
    sm = PARAMS["sigma_match_px"]["worst"]
    dtheta_ref = PARAMS["delta_theta_rad"]["worst"]
    R0 = 100.0
    # sigma_R_cal(worst) = R^2 * dtheta_ref*(b/2)^2 / b = R^2 * dtheta_ref * b/4
    k_match = R0**2 * sm / f_px          # coefficient of 1/b
    k_cal = R0**2 * dtheta_ref / 4.0     # coefficient of b
    b_opt = math.sqrt(k_match / k_cal)
    print(f"  b_opt = sqrt(k_match/k_cal) = {b_opt:.2f} m  "
          f"(k_match={k_match:.3f}, k_cal={k_cal:.3f})")
    print("  => even in the pessimistic tier the accuracy optimum sits near 2 m; beyond it")
    print("     a longer, harder-to-stiffen outdoor bar trades ranging error for rigidity cost.")
    print("  EXPECTED tier (rigidity_exp=1): the calibration floor R^2*dtheta/b is")
    print("     baseline-INDEPENDENT, so widening past ~2 m buys ever less while the bar,")
    print("     the recal burden, and deployment cost all grow -> diminishing returns knee.")


def section_chosen_and_fit():
    print_header(f"CHOSEN DESIGN POINT: {CHOSEN_SENSOR} + {CHOSEN_FOCAL_MM} mm C-mount, "
                 f"b={CHOSEN_BASELINE_M} m, hardware GPIO trigger")
    f_px = f_px_from_mm(CHOSEN_FOCAL_MM, CHOSEN_SENSOR)
    fov = hfov_deg(CHOSEN_FOCAL_MM, CHOSEN_SENSOR)
    print(f"f_px={f_px:.0f}, HFOV={fov:.1f} deg, sensor {SENSORS[CHOSEN_SENSOR]['width_px']}"
          f"x{SENSORS[CHOSEN_SENSOR]['height_px']}\n")

    print(f"{'tier':>9} {'R@1m':>6} {'R@0.5m':>7} {'R_det':>6} {'Ruse':>6} {'bind':>13} "
          f"{'s@50':>6} {'s@100':>6} {'xr@100':>7}")
    print("-" * 84)
    for tier in TIERS:
        r1 = range_at_sigma(CHOSEN_BASELINE_M, f_px, tier, SIGMA_R_THRESHOLD_M)
        rtight = range_at_sigma(CHOSEN_BASELINE_M, f_px, tier, SIGMA_R_TIGHT_M)
        rdet = detection_range_floor(f_px, tier)
        _, ruse, binding = usable_envelope(CHOSEN_BASELINE_M, f_px, tier)
        s50 = float(sigma_R(np.array([50.0]), CHOSEN_BASELINE_M, f_px, tier)[0])
        s100 = float(sigma_R(np.array([100.0]), CHOSEN_BASELINE_M, f_px, tier)[0])
        xr100 = sigma_cross(100.0, f_px, tier)
        print(f"{tier:>9} {r1:>6.0f} {rtight:>7.0f} {rdet:>6.0f} {ruse:>6.0f} {binding:>13} "
              f"{s50:>6.2f} {s100:>6.2f} {xr100:>7.3f}")

    # Term-by-term budget at 100 m, EXPECTED. Shares are of the VARIANCE
    # (term**2 / total**2), which sum to 100% for an RSS budget.
    print("\nError budget at R=100 m, EXPECTED tier (share of variance, sums to 100%):")
    t = sigma_R_terms(np.array([100.0]), CHOSEN_BASELINE_M, f_px, "expected")
    tot = float(sigma_R(np.array([100.0]), CHOSEN_BASELINE_M, f_px, "expected")[0])
    for k in ("match", "cal", "sync", "dist"):
        v = float(t[k][0]) if hasattr(t[k], "__len__") else float(t[k])
        print(f"  sigma_R_{k:<5} = {v:6.3f} m   ({100*(v/tot)**2:4.1f}% of variance)")
    print(f"  TOTAL         = {tot:6.3f} m")

    # ---- THE FIT the sim consumes: a + c*R^2 over the usable envelope ----
    print_header("FIT TO THE SIM's sigma_R = a + c*R**2  (constants s2_cue_mock.py should consume)")
    print("Fitted over the usable envelope [20 m .. R_use], stereo NOISE only.")
    print("Datum/GPS offset (0.5 m RTK / 2.5 m standard GPS) stays in the SEPARATE")
    print("--datum-bias-m knob as a per-run CONSTANT bias -- it is NOT part of c.\n")
    print(f"{'tier':>9} {'a (m)':>10} {'c (m/m^2)':>12} {'rms_resid':>10} {'fit range':>14}   datum_bias")
    print("-" * 84)
    fits = {}
    for tier in TIERS:
        _, ruse, _ = usable_envelope(CHOSEN_BASELINE_M, f_px, tier)
        r_hi = max(40.0, ruse)   # fit over a meaningful span even if envelope is short
        R = np.linspace(20.0, r_hi, 400)
        s = sigma_R(R, CHOSEN_BASELINE_M, f_px, tier)
        a, c, rms = fit_quadratic(R, s)
        fits[tier] = (a, c, rms, r_hi)
        db = PARAMS["datum_floor_m"][tier]
        tag = 'RTK' if db <= 0.5 else 'std GPS' if db >= 2.0 else 'mid'
        print(f"{tier:>9} {a:>10.4f} {c:>12.3e} {rms:>10.4f} {f'20-{r_hi:.0f} m':>14}   "
              f"{db:.1f} m ({tag})")

    a_e, c_e, _, _ = fits["expected"]
    print("\nRECONCILIATION with the mock's CURRENT sigma_R = "
          f"{SIM_CURRENT_A} + {SIM_CURRENT_C}*R**2:")
    print(f"  - The mock's c={SIM_CURRENT_C} was hand-set for the SHORT handoff range "
          "(R~5-15 m):")
    for Rr in (5, 10, 15, 100):
        cur = SIM_CURRENT_A + SIM_CURRENT_C * Rr**2
        phys = a_e + c_e * Rr**2
        print(f"      R={Rr:>3} m: mock={cur:6.2f} m   physics(EXPECTED stereo)={phys:6.3f} m")
    print(f"  - Extrapolated to 100 m the mock gives {SIM_CURRENT_A + SIM_CURRENT_C*100**2:.0f} m "
          "-- absurd for a real 2 m rig.")
    print("  - Physics: a real 2 m stereo rig's NOISE is ~cm at handoff range and ~0.4 m at")
    print("    100 m. The 2-4 m cue error the mock models at handoff is REAL but is the GPS")
    print("    DATUM/clock offset (ADR-0015), which belongs in --datum-bias-m, NOT in c.")
    print(f"  - RECOMMENDED split for s2_cue_mock.py (EXPECTED): sigma_R(R) = {a_e:.3f} + "
          f"{c_e:.2e}*R**2 (stereo noise), plus --datum-bias-m 0.5 (RTK) or 2.5 (std GPS).")
    return fits


# ============================================================================
# PLOTS
# ============================================================================
def make_plots(fits):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(PLOTS_DIR, exist_ok=True)
    f_px = f_px_from_mm(CHOSEN_FOCAL_MM, CHOSEN_SENSOR)
    R = np.linspace(5, 200, 400)

    # --- Plot 1: sigma_R vs R, per baseline (EXPECTED) + per tier (chosen b) ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    for b in BASELINES_M:
        ax1.plot(R, sigma_R(R, b, f_px, "expected"), label=f"b={b} m")
    ax1.axhline(SIGMA_R_THRESHOLD_M, color="k", ls="--", lw=0.8, label="1 m gate")
    ax1.set_title(f"sigma_R vs range, per baseline\n(EXPECTED tier, {CHOSEN_SENSOR} {CHOSEN_FOCAL_MM} mm)")
    ax1.set_xlabel("range R (m)")
    ax1.set_ylabel("range error sigma_R (m)")
    ax1.set_ylim(0, 3)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8)

    colors = {"best": "tab:green", "expected": "tab:blue", "worst": "tab:red"}
    for tier in TIERS:
        ax2.plot(R, sigma_R(R, CHOSEN_BASELINE_M, f_px, tier), color=colors[tier], label=f"{tier}")
        a, c, _, _ = fits[tier]
        ax2.plot(R, a + c * R**2, color=colors[tier], ls=":", lw=1.0)
    ax2.axhline(SIGMA_R_THRESHOLD_M, color="k", ls="--", lw=0.8)
    ax2.set_title(f"sigma_R vs range, three tiers (b={CHOSEN_BASELINE_M} m)\n"
                  "solid=model, dotted=a+c*R^2 fit")
    ax2.set_xlabel("range R (m)")
    ax2.set_ylabel("range error sigma_R (m)")
    ax2.set_ylim(0, 3)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    p1 = os.path.join(PLOTS_DIR, "stereo_sigmaR_vs_range.png")
    fig.savefig(p1, dpi=110)
    plt.close(fig)

    # --- Plot 2: usable envelope -- sigma_R and P_detect, chosen design ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    for tier in TIERS:
        ax1.plot(R, sigma_R(R, CHOSEN_BASELINE_M, f_px, tier), color=colors[tier], label=f"sigma_R {tier}")
    ax1.axhline(SIGMA_R_THRESHOLD_M, color="k", ls="--", lw=0.8, label="1 m gate")
    ax1.set_title(f"Usable envelope: range accuracy\n({CHOSEN_SENSOR} {CHOSEN_FOCAL_MM} mm, b={CHOSEN_BASELINE_M} m)")
    ax1.set_xlabel("range R (m)")
    ax1.set_ylabel("sigma_R (m)")
    ax1.set_ylim(0, 3)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8)

    for tier in TIERS:
        ax2.plot(R, p_detect(R, f_px, tier), color=colors[tier], label=f"P_detect {tier}")
        rdet = detection_range_floor(f_px, tier)
        ax2.axvline(rdet, color=colors[tier], ls=":", lw=1.0)
    ax2.axhline(P_DETECT_GATE, color="k", ls="--", lw=0.8, label="0.9 gate")
    ax2.set_title("Usable envelope: detection\n(vertical dotted = detection floor R_det)")
    ax2.set_xlabel("range R (m)")
    ax2.set_ylabel("P_detect (confident classified track)")
    ax2.set_ylim(0, 1.05)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    p2 = os.path.join(PLOTS_DIR, "stereo_usable_envelope.png")
    fig.savefig(p2, dpi=110)
    plt.close(fig)

    # --- Plot 3: the knee -- sigma_R(100 m) vs baseline, three tiers ---
    fig, ax = plt.subplots(figsize=(7, 5))
    bs = np.linspace(0.5, 6.5, 200)
    for tier in TIERS:
        y = [float(sigma_R(np.array([100.0]), b, f_px, tier)[0]) for b in bs]
        ax.plot(bs, y, color=colors[tier], label=f"{tier}")
    ax.axvline(CHOSEN_BASELINE_M, color="k", ls="--", lw=0.8, label="chosen b=2 m")
    ax.set_title(f"Cost/accuracy knee: sigma_R at 100 m vs baseline\n({CHOSEN_SENSOR} {CHOSEN_FOCAL_MM} mm)")
    ax.set_xlabel("baseline b (m)")
    ax.set_ylabel("sigma_R at R=100 m (m)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    p3 = os.path.join(PLOTS_DIR, "stereo_baseline_knee.png")
    fig.savefig(p3, dpi=110)
    plt.close(fig)

    print(f"\n[plots] wrote:\n  {p1}\n  {p2}\n  {p3}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-plots", action="store_true", help="skip matplotlib output")
    ap.add_argument("--section", choices=["geometry", "trade", "knee", "chosen", "all"],
                    default="all", help="print only one section")
    args = ap.parse_args()

    if args.section in ("geometry", "all"):
        section_sensor_geometry()
    if args.section in ("trade", "all"):
        section_trade_table()
    if args.section in ("knee", "all"):
        section_knee()
    fits = None
    if args.section in ("chosen", "all"):
        fits = section_chosen_and_fit()

    if not args.no_plots:
        if fits is None:
            f_px = f_px_from_mm(CHOSEN_FOCAL_MM, CHOSEN_SENSOR)
            fits = {}
            for tier in TIERS:
                _, ruse, _ = usable_envelope(CHOSEN_BASELINE_M, f_px, tier)
                R = np.linspace(20.0, max(40.0, ruse), 400)
                fits[tier] = (*fit_quadratic(R, sigma_R(R, CHOSEN_BASELINE_M, f_px, tier)),
                              max(40.0, ruse))
        make_plots(fits)


if __name__ == "__main__":
    main()
