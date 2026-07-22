"""flight.range_fusion -- Phase C of the intercept-accuracy program
(docs/intercept_accuracy_levers.md): SHARPEN ZEM / t_go. A monocular
size-based range prior, R = fx * W / w_px from the detector bbox width
(known target span W ~= 0.3 m), plus a closing rate from its time
derivative, fused as a COARSE PRIOR into the existing alpha-beta/Kalata
range channel. t_go = R/Vc squares into the correction capacity
(0.5*a*t_go^2) that sets ~96% of the miss -- a sharper range/closing-rate
estimate is a t_go lever, NOT a resolution lever.

STATUS: PRE-CODED, OPT-IN, NOT SIM-VALIDATED. Default-off behind
--range-fusion (wiring: docs/phase_bcd_wiring.md). Phase A (pointing) must
land first: at 0.8% in-flight detection there is almost nothing to fuse.

THE PHYSICS:
  * Pinhole size-range: a target of true span W at range R images at
    w = fx*W/R pixels, so  R = fx*W/w  (the same relation the markerless
    seeker already uses for its coarse range, scripts/seeker/nn_seeker.py).
  * Closing rate WITHOUT differentiating range: run the alpha-beta (Kalata)
    filter on the WIDTH channel (w, wdot) -- the raw pixel quantity -- and
    differentiate the MODEL, not the measurement:
        R = fx*W/w,   Rdot = -fx*W*wdot/w^2,   tau = R/(-Rdot) = w/wdot.
    tau (time-to-contact from bbox growth, w/wdot) falls out for free and is
    what Phase D's coast arms on.
  * HONEST NOISE MODEL (stated, not hidden): the relative range error equals
    the relative width error, sigma_R/R ~= sigma_w/w. A +-1 px width error
    on a FEW-px box is a ~10-30% range error (1 px on 10 px = 10%; on 3 px
    = 33%) -- hence COARSE PRIOR, never the primary. Target bank/aspect
    foreshortens the projected span (apparent W' = W*cos(bank-projection),
    ~13% at 30 deg bank), read as a HIGH range bias; modeled as an extra
    relative sigma term, RSS'd:
        sigma_R = R * sqrt((sigma_w/w)^2 + foreshorten_frac^2).

THE FUSION (compose, don't replace): the existing guidance range filter
(flight.estimator.AlphaBetaFilter, camera/tag range as primary) is NOT
touched. The prior folds in through the filter's own audited fusion port,
`correct(meas, t, gain_scale=w)`, with an inverse-variance weight from the
declared sigmas -- the exact mechanization the fused-track cue range uses
(m4_intercept.FusedTrack.update_cue):  w = min(1, sigma_primary^2 /
sigma_prior^2). By the update_cue precedent, the coarse prior never DEFINES
the track: fusion is skipped until the primary filter is initialized
(override with allow_init=True for a markerless-early variant, and say so
in the A/B writeup).

HONESTY BOUNDARY (ADR-0008/0010): inputs are the detector bbox width
(camera pixels), the calibrated fx, and the KNOWN target span W (a
pre-briefed constant, same status as the AprilTag side length). No gt_*, no
datalink. NOTE: this adds a NEW pixel-into-loop path (bbox width steers the
range channel), so wiring it into guidance RE-EARNS the numeric no-cheat
audit before any result is claimed (CLAUDE.md honesty boundary).

SIM-TIME: every `t` here is SIM seconds from the caller's sim clock
(ADR-0009: sim time, never wall time).
"""

import math
from typing import Optional

from flight.estimator import AlphaBetaFilter

# Known target span (m): the fpv_quad_enemy body spans ~0.3 m
# (docs/intercept_accuracy_levers.md Phase C, W ~= 0.3 m known target).
TARGET_SPAN_M = 0.30

# Width-channel Kalata defaults (STARTING POINTS for the A/B, same role as
# flight.estimator's KALATA_* constants): measurement sigma ~1.5 px box-edge
# jitter; process sigma sized to the HYPERBOLIC width growth the constant-
# velocity filter must chase -- for w = fx*W/R the width acceleration is
# wddot = 2*fx*W*Vc^2/R^3. At the test nominal fx*W = 600*0.3 = 180 px*m and
# Vc = 4 m/s that is ~27 px/s^2 at R=6 m and ~213 px/s^2 at R=3 m; the FPV
# terminal closes faster (Vc up to ~8 m/s, wddot ~4x -> ~110/850), so 400 is
# deliberate HEADROOM across that band (derivation, not vibes). The rate
# channel must stay honest through the endgame where the tau clock is read; a
# too-small value lags wdot and reads tau HIGH (the failure the unit tests
# pin -- 40 was too small, 400 tracks).
WIDTH_KALATA_SIGMA_PROCESS_PX_S2 = 400.0
WIDTH_KALATA_SIGMA_MEAS_PX = 1.5

# Same-tick fusion guard (see SizeRangeEstimator.fuse_into): skip folding the
# coarse prior when the primary range filter was corrected more recently than
# this (sim seconds). Below a control tick (~0.05 s at 20 Hz) so a legitimate
# gap-fill on the NEXT tick still fuses, but a same-t fold (dt->1e-3) can't.
MIN_FUSE_DT_S = 0.02

# Detector width-measurement noise (px) for the sigma_R model.
WIDTH_PX_SIGMA_DEFAULT = 1.0
# Bank/aspect W-foreshortening as a relative range sigma (~30 deg bank).
FORESHORTEN_FRAC_DEFAULT = 0.13

# Primary (camera/tag) range sigma model -- MIRRORS m4_intercept.py's fused-
# track constants (FUSE_CAM_RANGE_FRAC=0.10, FUSE_RANGE_SIGMA_FLOOR_M=0.3)
# so the weight is computed on the same basis as the audited cue fusion.
PRIMARY_RANGE_FRAC = 0.10
PRIMARY_RANGE_SIGMA_FLOOR_M = 0.3


def size_range_from_width(w_px, fx, target_span_m=TARGET_SPAN_M):
    """R = fx * W / w_px (m). None for a missing/degenerate width (<= 0)."""
    if w_px is None or w_px <= 0.0:
        return None
    return float(fx) * float(target_span_m) / float(w_px)


def size_range_sigma(range_m, w_px, px_sigma=WIDTH_PX_SIGMA_DEFAULT,
                     foreshorten_frac=FORESHORTEN_FRAC_DEFAULT):
    """Honest 1-sigma of the size-based range (m):
    sigma_R = R * sqrt((sigma_w/w)^2 + foreshorten_frac^2). None if the
    range/width is missing or degenerate."""
    if range_m is None or w_px is None or w_px <= 0.0:
        return None
    rel_px = float(px_sigma) / float(w_px)
    return float(range_m) * math.sqrt(rel_px * rel_px
                                      + foreshorten_frac * foreshorten_frac)


def fusion_weight(sigma_primary, sigma_prior):
    """Inverse-variance gain scale for AlphaBetaFilter.correct(gain_scale=):
    min(1, sigma_primary^2 / sigma_prior^2) -- the update_cue mechanization.
    A prior noisier than the primary folds in weakly; one (implausibly)
    better is capped at the primary's full gain."""
    return min(1.0, (sigma_primary * sigma_primary)
               / max(1e-9, sigma_prior * sigma_prior))


class SizeRangeEstimator:
    """Bbox-width channel -> (R, Rdot, tau) size-based prior.

    Wraps ONE flight.estimator.AlphaBetaFilter on the WIDTH (px) channel in
    Kalata mode (adaptive gains at the real, possibly gappy, correction
    interval -- the detection stream is intermittent). Derived quantities
    come from the pinhole model applied to the FILTER state, so range is
    never differenced directly:
        range_m  = fx*W / w_hat
        rdot_m_s = -fx*W * wdot_hat / w_hat^2   (closing < 0)
        tau_s    = w_hat / wdot_hat             (closing only, else None)

    Call predict(dt) every control tick, correct(w_px, t) only on fresh
    detections (the AlphaBetaFilter contract). Pure Python + math."""

    def __init__(self, fx, target_span_m=TARGET_SPAN_M,
                 sigma_process_px_s2=WIDTH_KALATA_SIGMA_PROCESS_PX_S2,
                 sigma_meas_px=WIDTH_KALATA_SIGMA_MEAS_PX,
                 px_sigma=WIDTH_PX_SIGMA_DEFAULT,
                 foreshorten_frac=FORESHORTEN_FRAC_DEFAULT):
        self.fx = float(fx)
        self.target_span_m = float(target_span_m)
        self.px_sigma = float(px_sigma)
        self.foreshorten_frac = float(foreshorten_frac)
        self.width_f = AlphaBetaFilter(
            0.5, 0.15, angular=False,
            kalata_sigma_process=sigma_process_px_s2,
            kalata_sigma_meas=sigma_meas_px, label="size_w")

    @property
    def initialized(self) -> bool:
        return self.width_f.initialized

    def predict(self, dt: float) -> None:
        """Coast the width state forward one control tick (sim dt)."""
        self.width_f.predict(dt)
        # A coasted width must stay physical (> 0); clamp like a range floor.
        if self.width_f.x_hat is not None and self.width_f.x_hat < 1e-3:
            self.width_f.x_hat = 1e-3

    def correct(self, w_px, t: float) -> None:
        """Fold in a fresh detector bbox width (px) at sim time t. Missing /
        degenerate widths (None, <= 0) are ignored -- a no-detection tick
        must not touch the state."""
        if w_px is None or w_px <= 0.0:
            return
        self.width_f.correct(float(w_px), t)

    # -- derived pinhole-model quantities (None until initialized) ----------
    @property
    def width_px(self) -> Optional[float]:
        return self.width_f.x_hat

    @property
    def width_rate_px_s(self) -> Optional[float]:
        return self.width_f.xdot_hat if self.width_f.initialized else None

    @property
    def range_m(self) -> Optional[float]:
        return size_range_from_width(self.width_f.x_hat, self.fx,
                                     self.target_span_m)

    @property
    def rdot_m_s(self) -> Optional[float]:
        """Closing rate from the width-channel model (closing < 0)."""
        if not self.width_f.initialized:
            return None
        w = self.width_f.x_hat
        if w is None or w <= 0.0:
            return None
        return -self.fx * self.target_span_m * self.width_f.xdot_hat / (w * w)

    @property
    def tau_s(self) -> Optional[float]:
        """Time-to-contact from bbox growth: tau = w/wdot (= R/(-Rdot)).
        None unless the box is GROWING (wdot > 0, i.e. closing) -- an
        opening or static geometry has no contact time."""
        if not self.width_f.initialized:
            return None
        wdot = self.width_f.xdot_hat
        if wdot <= 1e-9:
            return None
        return self.width_f.x_hat / wdot

    @property
    def sigma_range_m(self) -> Optional[float]:
        return size_range_sigma(self.range_m, self.width_f.x_hat,
                                self.px_sigma, self.foreshorten_frac)

    # -- the fusion port ----------------------------------------------------
    def fuse_into(self, range_filter: AlphaBetaFilter, t: float,
                  primary_frac=PRIMARY_RANGE_FRAC,
                  primary_floor_m=PRIMARY_RANGE_SIGMA_FLOOR_M,
                  allow_init: bool = False, min_dt_s: float = MIN_FUSE_DT_S) -> float:
        """Fold the size-range prior into the CALLER'S range filter as a
        gain-scaled correction (compose, don't replace). Returns the applied
        weight (0.0 = nothing applied).

        Weight = fusion_weight(sigma_primary, sigma_prior) with
        sigma_primary = max(floor, frac * R_ref) evaluated at the primary
        filter's own estimate -- the update_cue mechanization. Skipped until
        the primary filter is initialized (a coarse prior must not DEFINE
        the track's initial state) unless allow_init=True is deliberately
        passed (and disclosed in the A/B).

        SAME-TICK GUARD (min_dt_s, safe-by-construction): the α-β rate update
        is `xdot_hat += beta*gain*residual/dt_since`. If this prior is folded
        in at the SAME sim time as the primary camera correction, dt_since
        collapses to AlphaBetaFilter's 1e-3 floor and even a small gain-scaled
        residual injects a huge Rdot -- which detonates vc = -Rdot and hence
        a_cmd = N*vc*λ̇ under the DEFAULT fixed-gain range filter (Kalata mode
        self-protects as α,β→0 with dt, fixed-gain does NOT). So fusion is
        SKIPPED when the primary was corrected < min_dt_s ago: the camera owns
        that tick and a coarse prior adds nothing on top of a fresh, better
        measurement. The prior's value is on the COASTING ticks (sparse
        detection) -- call fuse_into every ENGAGE tick and the guard makes it a
        no-op on fresh-camera ticks, active on the gaps. This is the module
        analogue of the wiring rule 'fuse on non-fresh ticks'."""
        r_prior = self.range_m
        sigma_prior = self.sigma_range_m
        if r_prior is None or sigma_prior is None:
            return 0.0
        if not range_filter.initialized:
            if not allow_init:
                return 0.0
            range_filter.correct(r_prior, t)  # first-meas init path
            return 1.0
        last_t = range_filter._last_correction_t
        if last_t is not None and (t - last_t) < min_dt_s:
            return 0.0  # camera just corrected -> don't detonate the rate channel
        r_ref = max(range_filter.x_hat, 1e-3)
        sigma_primary = max(primary_floor_m, primary_frac * r_ref)
        w = fusion_weight(sigma_primary, sigma_prior)
        range_filter.correct(r_prior, t, gain_scale=w)
        return w
