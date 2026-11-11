#!/usr/bin/env python3
"""Clearance checks for the dock's nose camera post (needs cadquery; not in run_tests.sh).

Prints, from the CAD itself: gaps between the nose hardware and the pod at each
tilt, how far the pod can be tipped nose-down while hinging in before it touches
the post, how close each part comes to the front prop circles in top view, and
how far outside the picture the props are. The pod datum is ASSUMED to sit at the
frame centre and the props level with the top plate; neither is measured.

    <cad venv>/bin/python hardware/avionics_pod/nose_post_check.py
"""
import math
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pod as P, cadquery as cq
import camera_on_pod_fov as C
hw=P.PI_W/2
dock=P.dock(nose_post=True)
podparts={"pod_base":P.pod_base(),
 "pi":P.box(-P.PI_L/2,P.PI_L/2,-hw,hw,P.Z_PI,P.Z_PI+P.PI_T),
 "usb":P.box(P.PI_L/2-21,P.PI_L/2+2,-hw+2,hw-2,P.Z_PI+P.PI_T,P.Z_PI+16),
 "hat":P.box(P.PI_X_SD,P.PI_X_SD+P.HAT_L,-hw,hw,P.Z_HAT,P.Z_HAT+P.PI_T),
 "lid":P.pod_lid()}
def vol(a,b):
    try: return a.intersect(b).val().Volume()
    except Exception: return 0.0
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
def dist(a,b):
    d=BRepExtrema_DistShapeShape(a.val().wrapped,b.val().wrapped); d.Perform(); return d.Value()
print("== static: gap (mm) between nose hardware and the pod, by tilt")
for t in (10,25,45):
    pl=P.cam_plate_nose(t); gh=P.cam_ghost_nose(t)
    row=[]
    for n,p in podparts.items():
        row.append(f"{n} {min(dist(pl,p),dist(gh,p),dist(dock.cut(P.dock()),p) if n!='pod_base' else 99):.1f}")
    print(t, "plate-vs-dock overlap mm3", round(vol(pl,dock),2), "ghost-vs-dock", round(vol(gh,dock),2), "ghost-dock gap", round(dist(gh,dock),1), "|", "  ".join(row))
print("== hinge-in: pod rotated nose-down about the hook (x=30,z=3)")
post=dock.cut(P.dock())
for ang in (2,4,6,8,10,12):
    for t in (10,25,45):
        pl=P.cam_plate_nose(t)
        g=99
        for n,p in podparts.items():
            q=p.rotate((P.FLOOR_X_FRONT,-1,P.Z_DOCK),(P.FLOOR_X_FRONT,1,P.Z_DOCK),ang)
            if ang==2 and t==10 and n=="usb":
                bb=q.val().BoundingBox(); chk=bb.xmax
            g=min(g,dist(q,post),dist(q,pl))
        print(f"  swing {ang:2d} deg, tilt {t}: min gap {g:.1f}", end="")
    print()
print("usb block xmax at 2 deg swing", round(chk,1), "(44.5 static; must be larger = nose-down)")
print("== prop circle clearance in top view (mm), pod datum = frame centre ASSUMED")
def plan(wp, zlo, zhi):
    v=wp.val().tessellate(0.3)[0]
    best=1e9
    for p in v:
        if zlo<=p.z<zhi:
            for sy in (-1,1):
                best=min(best, math.hypot(p.x-C.MOTOR_X,p.y-sy*C.MOTOR_Y)-C.PROP_R)
    return best
print(" dock_cam z<10:", round(plan(dock,-1,10),1), " z>=10:", round(plan(dock,10,200),1))
for t in (10,25,45):
    pl=P.cam_plate_nose(t); gh=P.cam_ghost_nose(t)
    zmin=min(p.z for p in pl.val().tessellate(0.3)[0])
    print(f" tilt {t}: plate {plan(pl,-1,200):.1f} ghost {plan(gh,-1,200):.1f}  lowest point of plate z={zmin:.1f}")
print("== prop margin outside the picture (deg), lens front as viewpoint")
for cname,(hf,asp) in C.CAMERAS.items():
    row=[]
    for t in (10,25,45):
        x,z=P.nose_lens_xz(t)
        row.append(f"{t}d: {C.margin_deg(x,z,t,hf,asp,0.0):+.1f} / {C.margin_deg(x,z,t,hf,asp,10.0):+.1f}")
    print(" ",cname," | ".join(row))
bad = []
if plan(dock, -1, 10) < 6.0:
    bad.append("low part of dock_cam closer than 6 mm to a prop circle")
for ang in (4, 6):
    for n, p in podparts.items():
        q = p.rotate((P.FLOOR_X_FRONT, -1, P.Z_DOCK), (P.FLOOR_X_FRONT, 1, P.Z_DOCK), ang)
        if dist(q, post) < 1.0:
            bad.append(f"pod part {n} within 1 mm of the post at {ang} deg of hinge-in")
for t in (10, 17, 25, 35, 45):
    # the plate's ears run 0.5 mm inside the cheeks by design; anything closer is a rub
    if min(dist(P.cam_plate_nose(t), dock), dist(P.cam_ghost_nose(t), dock)) < 0.45:
        bad.append(f"camera or plate touches the dock at {t} deg tilt")
print("FAIL: " + "; ".join(bad) if bad else "PASS")
sys.exit(1 if bad else 0)
