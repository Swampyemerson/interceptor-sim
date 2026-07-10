#!/usr/bin/env python3
"""Generate UP-TILTED mono_cam model variants for the task #35 / ADR-0060
camera-mount A/B -- WITHOUT touching the canonical PX4 model.

WHY (ADR-0060, measured): the dash pitches the airframe nose-down 27-36 deg
(max ~52), throwing the horizon-level target to the TOP of the frame; ~35% of
dash ticks the target is entirely above the FoV and first in-range detection
threads a 0-6 deg sliver from the top edge. Remedy under test = a fixed
UP-TILTED camera mount (FPV convention), swept at +15/+25/+35 deg, plus a
+0 deg CONTROL variant that is functionally identical to the canonical model
(validates that the shadow-swap machinery itself introduces no delta).

WHAT IT DOES (canonical model is READ-ONLY -- verified by md5 before use):
  reads  ~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.{sdf,config}
  writes scripts/experiments/uptilt_mounts/up{00,15,25,35}/mono_cam/model.{sdf,config}
The ONLY functional change is the camera SENSOR pose inside camera_link:
  <pose>0 0 0 0 0 0</pose>  ->  <pose>0 0 0 0 -RAD 0</pose>
PITCH SIGN: SDF pose is x-forward/z-up; a POSITIVE pitch about +y rotates the
+x boresight DOWN (x_hat -> cos(t) x_hat - sin(t) z_hat), so an UP-tilt is a
NEGATIVE pitch. Tilting the SENSOR pose (not the link, not the joint) leaves
physics, inertia, the CameraJoint, and the /pose/info gt camera-link pose all
untouched -- so dash_pitch_probe-style body-frame analysis stays valid and the
mount angle is applied analytically (scripts/experiments/uptilt_ab_analyze.py).

HOW THE VARIANT IS SWAPPED IN AT RUN TIME (no PX4 edits):
  PX4's gz_env.sh.in composes GZ_SIM_RESOURCE_PATH as
      $GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS
  i.e. the USER path is searched FIRST, and mc_batch.sh launches the sim with
  GZ_SIM_RESOURCE_PATH=$REPO_ROOT/models. x500_mono_cam includes the camera
  via the URI `model://mono_cam`, resolved through that path -- so a
  models/mono_cam symlink pointing at a variant dir shadows the canonical
  model for that run only. scripts/uptilt_ab_arm.sh owns the symlink
  lifecycle (create -> fly -> ALWAYS remove).

Usage:  python3 scripts/uptilt_make_variants.py [--allow-hash-drift]
Safe by construction: never runs a sim, never writes outside
scripts/experiments/uptilt_mounts/.
"""
import hashlib
import math
import os
import sys

CANON_DIR = os.path.expanduser(
    "~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam")
# Canonical hashes recorded 2026-07-10 (PX4 v1.17.0 tree). If PX4 is updated
# the variants must be regenerated CONSCIOUSLY -- hence the hard check.
EXPECT_MD5 = {
    "model.sdf": "fb68cd22f30c7da2ba3e23c8cea2332a",
    "model.config": "9b1f52830c0038e1f05cf5c0eec4575c",
}
OUT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "experiments", "uptilt_mounts")
ANGLES_UP_DEG = [0, 15, 25, 35]

SENSOR_ANCHOR = '<sensor name="imager" type="camera">'
POSE_LINE = "<pose>0 0 0 0 0 0</pose>"


def main():
    allow_drift = "--allow-hash-drift" in sys.argv[1:]
    src = {}
    for name, want in EXPECT_MD5.items():
        p = os.path.join(CANON_DIR, name)
        with open(p, "rb") as f:
            data = f.read()
        got = hashlib.md5(data).hexdigest()
        if got != want:
            msg = (f"canonical {name} md5 {got} != recorded {want} "
                   "(PX4 tree changed?)")
            if not allow_drift:
                sys.exit(f"[uptilt-variants] FAIL: {msg} -- re-derive, then "
                         "rerun with --allow-hash-drift")
            print(f"[uptilt-variants] WARNING: {msg} -- proceeding (drift allowed)")
        src[name] = data.decode()

    sdf = src["model.sdf"]
    i_sensor = sdf.index(SENSOR_ANCHOR)
    i_pose = sdf.index(POSE_LINE, i_sensor)  # FIRST pose after the sensor tag
    for deg in ANGLES_UP_DEG:
        rad = math.radians(deg)
        if deg == 0:
            pose = POSE_LINE + "<!-- up00 CONTROL: pose unchanged -->"
        else:
            pose = (f"<pose>0 0 0 0 {-rad:.6f} 0</pose>"
                    f"<!-- UP-TILT +{deg} deg (task #35 / ADR-0060): negative "
                    "SDF pitch about +y tilts the sensor +x boresight UP -->")
        banner = (
            "<!-- GENERATED VARIANT (scripts/uptilt_make_variants.py) -- "
            f"mono_cam with a +{deg} deg UP-TILTED sensor mount for the "
            "task #35 / ADR-0060 camera-mount A/B. Shadows the canonical "
            "PX4 mono_cam via a models/mono_cam symlink + GZ_SIM_RESOURCE_PATH "
            "precedence (user path first, gz_env.sh.in). Only the imager "
            "sensor <pose> differs from canonical. -->\n")
        out_sdf = sdf[:i_pose] + pose + sdf[i_pose + len(POSE_LINE):]
        # banner right after the <sdf ...> open tag line
        k = out_sdf.index("\n", out_sdf.index("<sdf")) + 1
        out_sdf = out_sdf[:k] + banner + out_sdf[k:]

        d = os.path.join(OUT_ROOT, f"up{deg:02d}", "mono_cam")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "model.sdf"), "w") as f:
            f.write(out_sdf)
        with open(os.path.join(d, "model.config"), "w") as f:
            f.write(src["model.config"])  # verbatim; dir name drives model://
        print(f"[uptilt-variants] wrote {d}/model.sdf "
              f"(sensor pitch {-rad:+.6f} rad = +{deg} deg up)")
    print("[uptilt-variants] done. Canonical model untouched (read-only).")


if __name__ == "__main__":
    main()
