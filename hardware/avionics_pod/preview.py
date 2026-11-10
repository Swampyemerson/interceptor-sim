#!/usr/bin/env python3
"""Render preview.png from the generated STLs (plus ghost boxes for the boards)."""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from stl import mesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pod as P

HERE = os.path.dirname(os.path.abspath(__file__))

def ghost(x0, x1, y0, y1, z0, z1):
    v = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])
    f = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    return [v[i] for i in f]

fig = plt.figure(figsize=(16, 8))
light = np.array([0.4, -0.5, 0.75]); light /= np.linalg.norm(light)
for k, (elev, azim, title) in enumerate([(22, -60, "front-right, assembled"),
                                         (18, 150, "rear-left: SD card, power button, release screw")]):
    ax = fig.add_subplot(1, 2, k + 1, projection="3d")
    for name, col in (("dock", (0.35, 0.35, 0.38)), ("pod_base", (0.95, 0.55, 0.15)),
                      ("pod_lid", (0.95, 0.75, 0.3))):
        m = mesh.Mesh.from_file(os.path.join(HERE, name + ".stl"))
        n = m.normals / (np.linalg.norm(m.normals, axis=1, keepdims=True) + 1e-12)
        shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
        ax.add_collection3d(Poly3DCollection(m.vectors, facecolors=np.outer(shade, col),
                                             edgecolors="none"))
    hw = P.PI_W / 2
    for (b, c) in (
        (ghost(P.FC_X0 - P.FC_L / 2, P.FC_X0 + P.FC_L / 2, -P.FC_W / 2, P.FC_W / 2,
               P.Z_FC0, P.Z_FC0 + P.FC_H), (0.15, 0.15, 0.15, 0.55)),
        (ghost(-P.PI_L / 2, P.PI_L / 2, -hw, hw, P.Z_PI, P.Z_PI + P.PI_T), (0.1, 0.6, 0.2, 0.55)),
        (ghost(P.PI_L / 2 - 21, P.PI_L / 2 + 2, -hw + 2, hw - 2, P.Z_PI + P.PI_T,
               P.Z_PI + 16), (0.75, 0.75, 0.78, 0.45)),
        (ghost(P.PI_X_SD, P.PI_X_SD + P.HAT_L, -hw, hw, P.Z_HAT, P.Z_HAT + P.PI_T),
         (0.1, 0.35, 0.7, 0.55)),
    ):
        ax.add_collection3d(Poly3DCollection(b, facecolors=c, edgecolors=(0, 0, 0, 0.4)))
    ax.set_xlim(-75, 45); ax.set_ylim(-52, 52); ax.set_zlim(0, 75)
    ax.set_box_aspect((120, 104, 75)); ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x  (nose +)"); ax.set_ylabel("y"); ax.set_title(title)
fig.suptitle("Avionics pod: grey = dock (stays on frame), orange = removable pod, "
             "ghosts = Pixhawk (black), Pi 5 (green), USB/Ethernet block (silver), AI HAT+ (blue)")
fig.tight_layout()
fig.savefig(os.path.join(HERE, "preview.png"), dpi=70)
