#!/usr/bin/env python3
"""box_hits_gt -- the ONE offline "did the detector hit the real target?" gate.

WHY THIS FILE EXISTS (ADR-0076 add #18k, constraint `box-hits-gt-scoring-debt`)
------------------------------------------------------------------------------
`box_hits_gt` used to live (copy-pasted) inside resolution_probe.py and
domain_gap_eval.py, and tripod_score.py imported the first. The frame-top sweep
(P0.1) exposed three real defects in that gate that FALSE-REJECTED honest,
confident, dead-centred detections -- a near-6th-mirage: real hits scored as
misses, which manufactured the frame-top/banked "0%" scare and corrupted
#18j-fix's in-view numbers. This module is the single, PURE (no cv2 / no
onnxruntime -- so `tests/` can exercise it under the main .venv), documented,
unit-tested home the copies now import from.

THE THREE DEFECTS AND THE FIX
-----------------------------
Empirical evidence (v2 on data/frametop_sweep/frametop, conf 0.25 -- run the
re-scorer to reproduce): a confident, image-centred detection on the real
target was scored a MISS purely on the size gate as range grew:

    range  gt_w(0.35)  det_w  det/gt   OLD verdict
     8 m     30 px    87.5    2.92     HIT
    12 m     20 px    62.0    3.10     miss   <- conf 0.70, dcx=dcy=0 -> REAL
    16 m     15 px    48.2    3.21     miss   <- conf 0.53, centred    -> REAL
    20 m     12 px    48.2    4.02     miss   <- conf 0.53, centred    -> REAL
    25 m      7.6px   (no boxes)       miss   <- genuine miss (stays a miss)

(1) UNIFIED EXTENT (`gt_scale`). The sweep labelled at --extent-m 0.35 while
    quad_approach labelled at 0.9 -- not comparable, and BOTH wrong for this
    model. `TARGET_EXTENT_M` below is the measured fpv_quad_enemy silhouette
    (0.52 m). A caller that captured at a different extent passes
    gt_scale = TARGET_EXTENT_M / capture_extent so the gt box is re-scaled to
    the one true extent before the ratio test. At 0.52 m the det/gt ratios
    above become 1.96 / 2.09 / 2.16 / 2.71 -- all safely inside the 3x gate,
    and the 25 m genuine miss stays a miss. This is the dominant fix.

(2) OFF-AXIS-AWARE gt (`offaxis_aware`). classify() (capture_flight_frames.py)
    sizes the gt box on-axis: half_w = f * (extent/2) / z. An off-boresight
    target subtends more pixels than that paraxial size -- the pinhole map
    u = cx + f*tan(theta) has d(u)/d(theta) = f*sec^2(theta), so a fixed
    angular target is sec^2(theta) WIDER off-axis. classify() omits that, so
    off-axis gt boxes are UNDER-sized (measured ~1.7-1.9x too small at the
    +38 deg frame-top elevation; sec^2(38 deg)=1.61 plus the look-up belly
    aspect). We widen the EFFECTIVE gt by sec^2(theta) per axis, theta read
    from the gt centre offset: tan(theta_x) = (gcx-cx)/fx. On-axis (centred)
    targets get sec^2(0)=1 -> unchanged, so this never touches the centred
    100%s -- it only ever recovers off-axis false-rejects.

(3) CENTRE-LAG tolerance (`centre_lag_px`) for MOVING captures. On a fast
    crosser a detection one frame stale sits where the target WAS, lagging the
    freshly-snapshotted gt centre by the target's cross-track pixel motion over
    one inter-frame interval. Static set-pose sweeps have zero motion so the
    default is 0.0 (nothing changes -- proves the fix does not inflate static
    numbers); a moving-capture caller passes `centre_lag_tol_px(...)`.

The `size_ratio` (default 3x) BLOB GUARD is preserved unchanged: it still
rejects a giant ground-shadow blob that merely contains the gt point (the
Fable round-3 case: a 376x368 blob vs a 60x60 gt target -> 6.3x -> reject,
and still rejected here even after the sec^2 widening).
"""

# ---- mono_cam intrinsics @1280x960 (gz_x500_mono_cam SDF; same as
#      render_sim_dataset.py / capture_flight_frames.py). Defaults for the
#      off-axis angle read; a caller scoring a CROP passes shifted cx/cy. ----
FX = FY = 539.936
CX, CY = 640.0, 480.0
W_FULL, H_FULL = 1280, 960

# ---- The ONE silhouette extent (m). Measured from models/fpv_quad_enemy:
#      * motor-to-motor arm-tip diagonal        = 0.492 m
#      * prop-envelope AABB (static mesh)        = 0.404 m (roll axis)
#                                                x 0.641 m (prop-blade axis)
#      * mean of the two silhouette axis spans   = 0.522 m
#      Cross-check: back-solving the v2 detector box on the frame-top centred
#      control gives ~1.3-1.4 m of "boxed extent"; at the detector's ~2.5x
#      YOLO over-box that is ~0.52-0.56 m -- it lands on the same number.
#      Supersedes the sweep's 0.35 (too small) and the 0.9 default (0.9 was
#      measured for the OLD fpv_target_markerless BILLBOARD, not this 3D quad
#      -- see label_rig_captures.py:31). Derivation is reproducible via
#      scripts/seeker/measure_target_extent.py.
TARGET_EXTENT_M = 0.52


def box_hits_gt(boxes, gcx, gcy, gw, gh, tol=15,
                fx=FX, fy=FY, cx=CX, cy=CY,
                gt_scale=1.0, offaxis_aware=True, size_ratio=3.0,
                centre_lag_px=0.0):
    """True iff some `boxes` row is a real hit on the gt target.

    A hit = a detected box whose centre lands inside the (effective) gt box
    (+ tol + centre_lag_px) AND whose size is within `size_ratio` of the
    effective gt extent both ways. See the module docstring for the geometry
    behind `gt_scale` (extent unification), `offaxis_aware` (sec^2 widening)
    and `centre_lag_px` (moving-capture tolerance).

    boxes: iterable of (score, u, v, bw, bh) in the SAME pixel frame the gt
           coords are in (full frame, or a crop with cx/cy shifted to match).
    gcx, gcy, gw, gh: gt box centre + size in px (as classify()/load_gt_box
           produced them, i.e. at the capture-time extent).
    """
    # (1) re-scale the gt box from its capture extent to the one true extent.
    gw *= gt_scale
    gh *= gt_scale
    # (2) off-axis silhouette growth: sec^2(theta) = 1 + tan^2(theta),
    #     tan(theta_x) = (gcx - cx)/fx. Centred target -> factor 1.0.
    if offaxis_aware:
        sx = 1.0 + ((gcx - cx) / fx) ** 2
        sy = 1.0 + ((gcy - cy) / fy) ** 2
    else:
        sx = sy = 1.0
    gw_eff = gw * sx
    gh_eff = gh * sy
    # (3) moving-capture centre lag folds into the centre tolerance only.
    ctol = tol + max(0.0, centre_lag_px)
    lo = 1.0 / size_ratio
    for _score, u, v, bw, bh in boxes:
        center_in = (abs(u - gcx) <= gw_eff / 2 + ctol and
                     abs(v - gcy) <= gh_eff / 2 + ctol)
        size_ok = (gw_eff * lo <= bw <= size_ratio * gw_eff and
                   gh_eff * lo <= bh <= size_ratio * gh_eff)
        if center_in and size_ok:
            return True
    return False


def gt_scale_for(capture_extent_m, unified_extent_m=TARGET_EXTENT_M):
    """gt_scale to pass to box_hits_gt for a dataset labelled at
    `capture_extent_m` (e.g. 0.35 for the frame-top sweep, 0.9 for
    quad_approach). Linear because the projected box is linear in extent."""
    if capture_extent_m <= 0:
        return 1.0
    return unified_extent_m / capture_extent_m


def centre_lag_tol_px(v_perp_mps, range_m, dt_s, f=FX):
    """Extra centre tolerance (px) for a MOVING capture: a detection one frame
    stale lags the freshly-snapshotted gt centre by the target's cross-track
    angular motion over one inter-frame interval.
        lag_px = f * (v_perp / range) * dt
    Conservative by construction -- pass the MEASURED cross-speed (0 for a
    static set-pose sweep). Returns 0 for a degenerate range."""
    if range_m <= 0 or dt_s <= 0:
        return 0.0
    return f * (abs(v_perp_mps) / range_m) * dt_s
