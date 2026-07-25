#!/usr/bin/env python3
"""Own-propeller phantom mask for the quad markerless seeker (ADR-0076 r2l fix,
lever A — the fixed-geometry alternative to negative-mining).

STATUS (2026-07-20, committed post-hoc): the spatial-separator arc this belongs to
CLOSED NULL — the props and the real target OVERLAP in image space (image-mask A/B
regressed l2r 7/8 -> 3/8; ADR-0076 "converged root cause"), and the committed
camera-forward mount removes the phantom by GEOMETRY instead. Kept default-OFF as
the documented dead lever; do NOT resurrect (see the project_state.json graveyard).

WHY: viewing a mined true-negative frame (scripts/seeker/data/quad_hard_negatives,
1436/1588 frames) shows the ~1.5m r2l phantom IS the interceptor's OWN propeller
blades — dark angular shapes intruding from the TOP-LEFT and TOP-RIGHT frame
corners, silhouetted against the light-gray sky. The quad seeker
(drone_finetuned_quad_v2, trained on ALL-POSITIVE renders, never shown the props
as background) fires a big "close target" box on a blade; its measured range reads
~1.5m while the true target is 15-20m; in r2l the blade is co-located (camera-left)
with the real crossing target, wins the handoff, and forces a premature BREAKOFF at
a true range of ~20m -> CPA stalls ~4-7m. l2r escapes because the real target
crosses camera-right, separated from the (also-present) blade, and wins the handoff.

The existing self-mask (finetuned_seeker.py) rejects detections >max_bearing_deg OR
within edge_margin_px of the L/R edge -- but the blade intrudes ~450px inward, so a
box on the blade's INNER TIP (bearing < 30 deg, off-edge) slips through. Negative-
mining the blade (drone_finetuned_quad_hardened) DOES kill the phantom but REGRESSES
real-target box precision (both directions ~3m, neither clean). This mask is the
alternative: keep the good-precision v2 seeker and reject the phantom by FIXED CAMERA
GEOMETRY instead of learned suppression.

KEY SEPARABILITY: the blades live HIGH in the frame (top corners, y < ~TOP_FRAC*H); a
real target at 15-20m sits near the HORIZON (image center, y ~ 0.5*H). So a top-corner
exclusion zone kills the blade without masking a horizon-level real target -- even in
r2l where blade and target share the camera-LEFT half, they differ VERTICALLY.

HONESTY (CLAUDE.md / ADR-0010): the prop zone is FIXED CAMERA/AIRFRAME GEOMETRY (the
props are rigidly mounted; they occupy the same image pixels every frame) -- the same
class of own-airframe knowledge the existing self-mask already uses. NO gt_* is read;
the mask is a function of the detection's pixel box and the frame size only.

CALIBRATION: TOP_FRAC / SIDE_FRAC default to the top-corner region measured from the
mined true-negative frames (calibrate_prop_zone() below, run offline over
quad_hard_negatives). Env overrides: MARKERLESS_PROP_MASK (1=on, default 0 = OFF,
byte-identical), MARKERLESS_PROP_MASK_TOP_FRAC, MARKERLESS_PROP_MASK_SIDE_FRAC.
"""
from __future__ import annotations

import os

# Defaults CALIBRATED post-batch from the mined true-negative prop frames
# (calibrate_prop_zone). Conservative placeholders until then: the blades sit in
# the top ~37% of the frame and within ~35% of each side edge (top corners).
DEF_TOP_FRAC = float(os.environ.get("MARKERLESS_PROP_MASK_TOP_FRAC", "0.37"))
DEF_SIDE_FRAC = float(os.environ.get("MARKERLESS_PROP_MASK_SIDE_FRAC", "0.35"))


def in_prop_zone(cx: float, cy: float, img_w: float, img_h: float,
                 top_frac: float = DEF_TOP_FRAC,
                 side_frac: float = DEF_SIDE_FRAC) -> bool:
    """True if a detection's box CENTER (cx,cy in pixels) falls in the top-left OR
    top-right prop corner: cy < top_frac*H AND (cx < side_frac*W OR
    cx > (1-side_frac)*W). Pure, no gt, unit-testable on plain numbers.

    A real 15-20m target near the horizon (cy ~ 0.5*H) is never in the top band;
    a blade tip in a top corner is. Center-based (not overlap) is deliberate: a real
    target whose box merely CLIPS the top band by a pixel is not rejected, only one
    whose CENTER is squarely in the corner."""
    if not (top_frac > 0.0 and side_frac > 0.0):
        return False
    in_top = cy < top_frac * img_h
    in_side = (cx < side_frac * img_w) or (cx > (1.0 - side_frac) * img_w)
    return in_top and in_side


def prop_mask_enabled() -> bool:
    return os.environ.get("MARKERLESS_PROP_MASK", "0") == "1"


# ===========================================================================
# Offline calibration: measure the prop zone from the mined true-negative frames
# (the dark-pixel union). Run AFTER the validation batch (light CPU).
#   .venv-seeker/bin/python scripts/seeker/prop_mask.py --calibrate \
#       scripts/seeker/data/quad_hard_negatives/images
# ===========================================================================
def calibrate_prop_zone(img_dir: str, dark_thresh: int = 70, sample: int = 120):
    """Over a sample of mined true-negative frames, find the dark (prop) pixels
    and report the bounding fractions of the top-corner dark mass -> suggested
    TOP_FRAC / SIDE_FRAC. Returns a dict; prints a summary."""
    import glob
    import cv2
    import numpy as np
    paths = sorted(glob.glob(os.path.join(img_dir, "*.png")))[:sample]
    if not paths:
        print(f"[prop-cal] no frames in {img_dir}")
        return {}
    acc = None
    H = W = None
    for p in paths:
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if im is None:
            continue
        H, W = im.shape
        dark = (im < dark_thresh).astype("uint32")
        acc = dark if acc is None else acc + dark
    if acc is None:
        print("[prop-cal] no readable frames")
        return {}
    # frequently-dark pixels = the props (present in most frames)
    freq = acc / len(paths)
    propmask = freq > 0.5           # dark in >50% of frames -> body-fixed prop
    ys, xs = np.where(propmask)
    if len(ys) == 0:
        print("[prop-cal] no body-fixed dark pixels found (raise dark_thresh?)")
        return {}
    top_frac = float(ys.max() + 1) / H
    # side extent: how far inward from each edge the prop dark mass reaches
    left_in = float(xs[xs < W / 2].max() + 1) / W if (xs < W / 2).any() else 0.0
    right_in = float(W - xs[xs > W / 2].min()) / W if (xs > W / 2).any() else 0.0
    side_frac = max(left_in, right_in)
    out = {"img_hw": (H, W), "n_frames": len(paths),
           "prop_pixels": int(len(ys)),
           "top_frac_measured": round(top_frac, 3),
           "side_frac_measured": round(side_frac, 3),
           "left_inward_frac": round(left_in, 3),
           "right_inward_frac": round(right_in, 3)}
    print(f"[prop-cal] {out}")
    print(f"[prop-cal] suggest TOP_FRAC~{top_frac:.2f} SIDE_FRAC~{side_frac:.2f} "
          "(then shrink slightly to protect horizon targets, and verify on a flight)")
    return out


def _self_test() -> int:
    ok = True
    W, H = 1280.0, 960.0
    # a real target near the horizon center is NOT masked
    ok &= not in_prop_zone(640, 480, W, H)
    # a real target a bit left of center at the horizon is NOT masked
    ok &= not in_prop_zone(420, 470, W, H)
    # a blade tip in the top-left corner IS masked
    ok &= in_prop_zone(200, 250, W, H)
    # a blade tip in the top-right corner IS masked
    ok &= in_prop_zone(1120, 250, W, H)
    # top-CENTER (between the corners) is NOT masked (a high target dead-ahead)
    ok &= not in_prop_zone(640, 200, W, H)
    # disabled (fracs 0) never masks -> byte-identical when off
    ok &= not in_prop_zone(200, 250, W, H, top_frac=0.0, side_frac=0.0)
    print(f"[prop-mask self-test] {'ALL PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--calibrate" in sys.argv:
        i = sys.argv.index("--calibrate")
        calibrate_prop_zone(sys.argv[i + 1])
    else:
        raise SystemExit(_self_test())
