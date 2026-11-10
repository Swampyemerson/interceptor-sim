#!/usr/bin/env python3
"""Vibration check of the avionics pod: natural frequencies by finite elements.

WHAT IT IS: the printed pod base with the Pi, the HAT, the lid (and optionally the
camera cradle) stacked on it, meshed and solved for its lowest natural frequencies
with the pod floor held rigid (= a bolted dock). A natural frequency is the rate at
which a structure "wants" to shake; if a motor spins at that rate the shake grows.

WHAT IT IS NOT: a measurement. Every joint is modelled as glued (real screwed and
standoff joints are softer), boards are plain plates carrying their parts' weight,
and printed PETG is one uniform stiffness. Glued joints make the answer too HIGH,
so read the result as an upper bound and use the low-stiffness row as the working
number. The foam-tape rocking estimate at the end is a separate hand formula.

    <cad venv>/bin/python hardware/avionics_pod/fea_modal.py [--camera | --nose] [--epetg 1900]
Needs cadquery, gmsh, scikit-fem, scipy (scratch venv; not part of run_tests.sh).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pod as P  # noqa: E402
import cadquery as cq  # noqa: E402

# material: (Young's modulus MPa, Poisson, density tonne/mm^3)
BRASS = (100e3, 0.33, 8.5e-9)
STEEL = (200e3, 0.30, 7.85e-9)


def plate_density(mass_g, vol_mm3):
    return mass_g * 1e-6 / vol_mm3


def build(camera: bool, e_petg: float):
    """Return {name: (cq solid, material)}; solids touch but do not overlap."""
    petg = (e_petg, 0.38, P.PETG_G_PER_MM3 * 1e-6 * P.PRINT_FILL)
    hw = P.PI_W / 2
    parts = {"pod_base": (P.pod_base(), petg)}
    pi = P.box(-P.PI_L / 2, P.PI_L / 2, -hw, hw, P.Z_PI, P.Z_PI + P.PI_T)
    pi_mass = P.MASS["pi"] + P.MASS["cooler"]
    parts["pi_board"] = (pi, (22e3, 0.2, plate_density(pi_mass, P.PI_L * P.PI_W * P.PI_T)))
    hat = P.box(P.PI_X_SD, P.PI_X_SD + P.HAT_L, -hw, hw, P.Z_HAT, P.Z_HAT + P.PI_T)
    parts["hat_board"] = (hat, (22e3, 0.2, plate_density(P.MASS["hat"], P.HAT_L * P.PI_W * P.PI_T)))
    i = 0
    for hx in P.HOLE_XS:
        for hy in P.HOLE_YS:
            parts[f"so_a{i}"] = (P.cyl(hx, hy, P.Z_PI + P.PI_T, P.Z_HAT, 4.5), BRASS)
            parts[f"so_b{i}"] = (P.cyl(hx, hy, P.Z_HAT + P.PI_T, P.Z_LID, 4.5), BRASS)
            i += 1
    parts["lid"] = (P.pod_lid(camera=camera), petg)
    if camera:
        parts["cam_plate"] = (P.cam_plate(), petg)
        half = P.CAM_PLATE_HALF_W
        px, pz = P.cam_pivot()
        th = math.radians(P.CAM_TILT_SHOWN)
        j = 0
        for sy in (-1, 1):       # steel discs standing in for the pivot + lock screws
            for (x, z) in ((px, pz), (px - P.CAM_LOCK_R * math.cos(th),
                                      pz - P.CAM_LOCK_R * math.sin(th))):
                ya, yb = sorted((sy * half, sy * P.CHEEK_Y_IN))
                d = (cq.Workplane("XZ").circle(4.0).extrude(-(yb - ya))
                     .translate((x, ya, z)))
                parts[f"screw{j}"] = (d, STEEL)
                j += 1
        cam = P.cam_ghost()
        v = cam.val().Volume()
        parts["camera"] = (cam, (5e3, 0.3, plate_density(15.0, v)))
    return parts


def build_nose(e_petg: float):
    """The dock with its nose post, the tilting plate and the camera, on their own."""
    petg = (e_petg, 0.38, P.PETG_G_PER_MM3 * 1e-6 * P.PRINT_FILL)
    parts = {"dock_cam": (P.dock(nose_post=True), petg),
             "cam_plate_nose": (P.cam_plate_nose(), petg)}
    lx, lz = P.nose_lock_local()
    dx, dz = P._tilt(lx, lz, P.CAM_TILT_SHOWN)
    j = 0
    for sy in (-1, 1):           # steel discs standing in for the pivot + lock screws
        for (x, z) in ((P.NOSE_PIVOT_X, P.NOSE_PIVOT_Z),
                       (P.NOSE_PIVOT_X + dx, P.NOSE_PIVOT_Z + dz)):
            ya, yb = sorted((sy * P.NOSE_EAR_HALF, sy * P.NOSE_CHEEK_Y_IN))
            d = (cq.Workplane("XZ").center(x, z).circle(3.0).extrude(-(yb - ya))
                 .translate((0, ya, 0)))
            parts[f"screw{j}"] = (d, STEEL)
            j += 1
    cam = P.cam_ghost_nose()
    parts["camera"] = (cam, (5e3, 0.3, plate_density(15.0, cam.val().Volume())))
    return parts


def mesh_and_solve(parts, n_modes=6, size=4.0, clamp=None, z_top=None):
    import gmsh
    import skfem
    from skfem.models.elasticity import linear_elasticity, lame_parameters
    from scipy.sparse.linalg import eigsh

    tmp = tempfile.mkdtemp()
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    tags = {}
    for name, (wp, _) in parts.items():
        f = os.path.join(tmp, name + ".step")
        cq.exporters.export(wp, f)
        out = gmsh.model.occ.importShapes(f)
        tags[name] = [t for (d, t) in out if d == 3]
    allv = [(3, t) for ts in tags.values() for t in ts]
    _, omap = gmsh.model.occ.fragment(allv[:1], allv[1:])
    gmsh.model.occ.synchronize()
    k = 0
    for name, ts in tags.items():
        vols = []
        for _ in ts:
            vols += [t for (d, t) in omap[k] if d == 3]
            k += 1
        gmsh.model.addPhysicalGroup(3, sorted(set(vols)), name=name)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", size)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", 1.0)
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
    gmsh.model.mesh.generate(3)
    mf = os.path.join(tmp, "pod.msh")
    gmsh.write(mf)
    gmsh.finalize()

    m = skfem.Mesh.load(mf)
    e = skfem.ElementVector(skfem.ElementTetP2())
    full = skfem.Basis(m, e)
    K = None
    M = None

    @skfem.BilinearForm
    def massf(u, v, w):
        return w.rho * (u[0] * v[0] + u[1] * v[1] + u[2] * v[2])

    total = 0.0
    kparts = {}
    for name, (_, (E, nu, rho)) in parts.items():
        if name not in m.subdomains:
            raise SystemExit(f"FAIL: part {name} lost in meshing")
        b = skfem.Basis(m, e, elements=m.subdomains[name])
        lam, mu = lame_parameters(E, nu)
        k_ = linear_elasticity(lam, mu).assemble(b)
        m_ = massf.assemble(b, rho=rho)
        kparts[name] = k_
        K = k_ if K is None else K + k_
        M = m_ if M is None else M + m_
    ndof = K.shape[0]
    # rigid-body checks from the mass matrix
    X = full.doflocs
    tz = np.zeros(ndof)
    tz[full.nodal_dofs[2]] = 1.0
    tz[full.edge_dofs[2]] = 1.0 if full.edge_dofs.size else 0.0
    total = float(tz @ (M @ tz))
    if clamp is None:
        clamp = lambda x: x[2] < P.Z_DOCK + 0.05          # noqa: E731
    if z_top is None:
        z_top = P.Z_LID - 1.0
    fixed = full.get_dofs(clamp).all()
    if len(fixed) == 0:
        raise SystemExit("FAIL: no clamped nodes found")
    free = np.setdiff1d(np.arange(ndof), fixed)
    Kf = K[free][:, free].tocsc()
    Mf = M[free][:, free].tocsc()
    vals, vecs = eigsh(Kf, k=n_modes, M=Mf, sigma=0.0, which="LM")
    order = np.argsort(vals)
    freqs = np.sqrt(np.maximum(vals[order], 0)) / (2 * math.pi)
    modes = []
    for i in order:
        u = np.zeros(ndof)
        u[free] = vecs[:, i]
        comp = []
        for c in range(3):
            idx = np.concatenate([full.nodal_dofs[c], full.edge_dofs[c]])
            top = idx[X[2, idx] > z_top]
            comp.append(float(np.sqrt(np.mean(u[top] ** 2))))
        comp = np.array(comp) / (max(comp) + 1e-30)
        label = ("fore-aft", "side-to-side", "up-down")[int(np.argmax(comp))]
        tot = float(u @ (K @ u))
        grp = {}
        for name, k_ in kparts.items():
            key = "standoffs" if name.startswith("so_") else (
                "screws" if name.startswith("screw") else name)
            grp[key] = grp.get(key, 0.0) + float(u @ (k_ @ u)) / tot
        flex = {k2: round(v, 2) for k2, v in sorted(grp.items(), key=lambda kv: -kv[1])
                if v >= 0.05}
        modes.append((label, [round(float(c), 2) for c in comp], flex))
    return {"freqs_hz": [round(float(f), 1) for f in freqs], "modes": modes,
            "mass_g": round(total * 1e6, 1), "nodes": int(m.p.shape[1]),
            "tets": int(m.t.shape[1]), "clamped_dofs": int(len(fixed))}


def tape_rocking(mass_g, cg_z_mm, g_mpa, tape_t_mm=1.0):
    """Side-to-side rocking of the whole (rigid) pod on a foam-tape layer under
    the dock. Tape in tension/compression across the dock width; E taken as 3G."""
    L, W = (P.FLOOR_X_FRONT + 6.0 - P.TAB_X_REAR), P.DOCK_W
    e = 3.0 * g_mpa                                    # MPa = N/mm^2
    k_theta = e * (L * W ** 3 / 12.0) / tape_t_mm      # N*mm/rad
    m = mass_g * 1e-3                                  # kg
    inertia = m * ((cg_z_mm * 1e-3) ** 2 + (P.PI_W * 1e-3) ** 2 / 12.0)   # kg m^2
    return math.sqrt(k_theta * 1e-3 / inertia) / (2 * math.pi)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", action="store_true")
    ap.add_argument("--nose", action="store_true",
                    help="the dock's nose camera post instead of the pod tower")
    ap.add_argument("--epetg", type=float, default=1900.0, help="printed PETG modulus, MPa")
    ap.add_argument("--size", type=float, default=4.0, help="max element size, mm")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    if a.nose:
        # held where the dock lies on the top plate (29 mm wide there) = bolted flat
        r = mesh_and_solve(build_nose(a.epetg), size=a.size,
                           clamp=lambda x: (x[2] < 0.05) & (abs(x[1]) < 14.6),
                           z_top=P.NOSE_PIVOT_Z)
        r["nose_post"] = True
    else:
        r = mesh_and_solve(build(a.camera, a.epetg), size=a.size)
    r["camera"] = a.camera or a.nose
    r["e_petg_mpa"] = a.epetg
    print(json.dumps(r, indent=1))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(r, f, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
