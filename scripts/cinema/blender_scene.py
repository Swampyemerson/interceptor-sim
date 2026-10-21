#!/usr/bin/env python3
"""Blender (4.5, Cycles/CUDA) scene builder + renderer for cinema takes.

Runs INSIDE Blender:
  ~/tools/blender45/blender -b --factory-startup -P scripts/cinema/blender_scene.py -- \
      --run logs/cinema/take2_run.json \
      --scene scripts/cinema/config/scene_field.json \
      --shots scripts/cinema/config/shots_default.json \
      --out renders/take2 [--shot NAME] [--still "NAME@12.5"] [--preview]

What it does:
  * builds an outdoor set (grass field, forest edge, bushes, sky sun) from
    the scene config (deterministic per `seed`);
  * builds the interceptor (x500-class dark quad, camera pod) and the target
    (FPV quad carrying the REAL 0.625 m AprilTag placard --
    models/apriltag_target/tag36h11_00000.png, Closest-filtered so the tag
    stays crisp) with the SAME body/tag layout as models/fpv_target;
  * animates both from the run.json pose log (ENU positions + quaternions,
    1 Blender unit = 1 m, world = ENU with +Z up);
  * renders the shot list: each shot = camera rig + sim-time window
    (event-relative expressions like "go-3", "cpa+0.5") + speed factor
    (0.2 = 5x slow motion), keyframed so Cycles motion blur is true to the
    retimed motion; writes shots/<name>/####.png and a manifest mapping
    every rendered frame to its sim time (hud_compose.py consumes it).

Pure visualization: reads logs, renders pixels. Nothing here feeds guidance.
"""
import argparse
import json
import math
import os
import random
import sys

import bpy
from mathutils import Euler, Quaternion, Vector

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TAG_PNG = os.path.join(REPO, "models", "apriltag_target", "tag36h11_00000.png")


# --------------------------------------------------------------------- helpers
def new_mat(name, base=(0.5, 0.5, 0.5, 1), rough=0.6, metal=0.0, emit=None):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = base
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if emit is not None:
        bsdf.inputs["Emission Color"].default_value = emit
        bsdf.inputs["Emission Strength"].default_value = 4.0
    return m


def link(obj, col):
    for c in obj.users_collection:
        c.objects.unlink(obj)
    col.objects.link(obj)


def prim(kind, name, col, mat=None, **kw):
    getattr(bpy.ops.mesh, f"primitive_{kind}")(**kw)   # kind includes "_add"
    o = bpy.context.active_object
    o.name = name
    if mat:
        o.data.materials.append(mat)
    link(o, col)
    return o


def shade_smooth(o):
    for p in o.data.polygons:
        p.use_smooth = True


def look_at_quat(cam_pos, target_pos, roll=0.0):
    d = (Vector(target_pos) - Vector(cam_pos)).normalized()
    q = d.to_track_quat("-Z", "Y")           # camera looks down -Z, +Y up
    if roll:
        q = q @ Quaternion((0, 0, 1), roll)
    return q


class Sampler:
    """Linear sampler over the run.json uniform timeline."""

    def __init__(self, run):
        self.t = run["t"]
        self.t0, self.hz, self.n = self.t[0], run["hz"], len(self.t)
        self.data = run

    def at(self, key, ts):
        a = self.data[key]
        x = (ts - self.t0) * self.hz
        i = max(0, min(self.n - 2, int(x)))
        f = max(0.0, min(1.0, x - i))
        va, vb = a[i], a[i + 1]
        if isinstance(va, list):
            return [va[k] * (1 - f) + vb[k] * f for k in range(len(va))]
        return va * (1 - f) + vb * f

    def quat(self, key, ts):
        q = self.at(key, ts)
        return Quaternion(q).normalized()


# ------------------------------------------------------------------ environment
def build_environment(scene_cfg, col):
    rng = random.Random(scene_cfg.get("seed", 7))

    # ground ----------------------------------------------------------------
    gmat = bpy.data.materials.new("ground")
    gmat.use_nodes = True
    nt = gmat.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = 0.035
    tex.inputs["Detail"].default_value = 6.0
    tex2 = nt.nodes.new("ShaderNodeTexNoise")
    tex2.inputs["Scale"].default_value = 2.4
    tex2.inputs["Detail"].default_value = 8.0
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].color = (0.03, 0.085, 0.015, 1)   # deep green
    ramp.color_ramp.elements[1].color = (0.13, 0.15, 0.035, 1)     # dry grass
    e = ramp.color_ramp.elements.new(0.55)
    e.color = (0.06, 0.115, 0.025, 1)
    # fac = 0.55*large_noise + 0.30*detail_noise (stays inside the ramp)
    m1 = nt.nodes.new("ShaderNodeMath")
    m1.operation = "MULTIPLY"
    nt.links.new(tex.outputs["Fac"], m1.inputs[0])
    m1.inputs[1].default_value = 0.55
    m2 = nt.nodes.new("ShaderNodeMath")
    m2.operation = "MULTIPLY_ADD"
    nt.links.new(tex2.outputs["Fac"], m2.inputs[0])
    m2.inputs[1].default_value = 0.30
    nt.links.new(m1.outputs[0], m2.inputs[2])
    nt.links.new(m2.outputs[0], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.35
    nt.links.new(tex2.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    bsdf.inputs["Roughness"].default_value = 0.95
    ground = prim("plane_add", "ground", col, gmat, size=600)

    # sky + sun ---------------------------------------------------------------
    sun_cfg = scene_cfg.get("sun", {})
    elev = math.radians(sun_cfg.get("elevation_deg", 22))
    azim = math.radians(sun_cfg.get("azimuth_deg", 160))
    w = bpy.context.scene.world or bpy.data.worlds.new("World")
    bpy.context.scene.world = w
    w.use_nodes = True
    wn = w.node_tree
    for n in list(wn.nodes):
        wn.nodes.remove(n)
    sky = wn.nodes.new("ShaderNodeTexSky")
    sky.sky_type = "NISHITA"
    sky.sun_elevation = elev
    sky.sun_rotation = azim
    sky.sun_intensity = sun_cfg.get("strength", 1.0)
    sky.altitude = 50
    sky.air_density = 1.2
    sky.dust_density = scene_cfg.get("haze", 2.0)
    # lighting sky + a BRIGHTER camera-visible sky (classic light-path split:
    # exposure is set for the sunlit ground, which would render the horizon
    # muddy; camera rays get a gain so the sky reads bright and clean)
    bg = wn.nodes.new("ShaderNodeBackground")
    bg_cam = wn.nodes.new("ShaderNodeBackground")
    gain = wn.nodes.new("ShaderNodeVectorMath")
    gain.operation = "SCALE"
    gain.inputs["Scale"].default_value = scene_cfg.get("sky_cam_gain", 2.2)
    lp = wn.nodes.new("ShaderNodeLightPath")
    mix = wn.nodes.new("ShaderNodeMixShader")
    out = wn.nodes.new("ShaderNodeOutputWorld")
    wn.links.new(sky.outputs["Color"], bg.inputs["Color"])
    wn.links.new(sky.outputs["Color"], gain.inputs[0])
    wn.links.new(gain.outputs["Vector"], bg_cam.inputs["Color"])
    wn.links.new(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
    wn.links.new(bg.outputs["Background"], mix.inputs[1])
    wn.links.new(bg_cam.outputs["Background"], mix.inputs[2])
    wn.links.new(mix.outputs["Shader"], out.inputs["Surface"])

    # trees -------------------------------------------------------------------
    trunk_m = new_mat("trunk", (0.035, 0.02, 0.012, 1), rough=0.95)
    leaf_ms = [new_mat(f"leaf{i}", c, rough=0.92) for i, c in enumerate([
        (0.022, 0.095, 0.012, 1), (0.035, 0.125, 0.015, 1), (0.015, 0.075, 0.015, 1)])]
    ctex = bpy.data.textures.new("canopy_noise", type="CLOUDS")
    ctex.noise_scale = 0.9

    def make_tree(name, x, y, s, variant):
        h = 5.0 * s
        t = prim("cone_add", f"{name}_trunk", col, trunk_m,
                 radius1=0.16 * s, radius2=0.07 * s, depth=h,
                 location=(x, y, h / 2))
        rng2 = random.Random(variant * 977 + int(x * 13) + int(y * 7))
        k = 3 + variant % 3
        for i in range(k):
            r = (1.6 - 0.28 * i) * s * rng2.uniform(0.85, 1.15)
            z = h * 0.62 + i * 1.05 * s
            o = prim("ico_sphere_add", f"{name}_c{i}", col,
                     leaf_ms[(variant + i) % 3], radius=r, subdivisions=2,
                     location=(x + rng2.uniform(-0.4, 0.4) * s,
                               y + rng2.uniform(-0.4, 0.4) * s, z))
            o.scale = (1.0, 1.0, rng2.uniform(0.7, 0.9))
            dm = o.modifiers.new("disp", "DISPLACE")
            dm.texture = ctex
            dm.strength = 0.45 * r
            shade_smooth(o)
        return t

    band = scene_cfg.get("tree_band", {})
    n_band = band.get("count", 120)
    for i in range(n_band):
        x = rng.uniform(band.get("x_min", -90), band.get("x_max", 90))
        y = rng.uniform(band.get("y_min", 10), band.get("y_max", 55))
        make_tree(f"treeB{i}", x, y, rng.uniform(0.9, 1.7), rng.randrange(6))
    sc = scene_cfg.get("scatter_trees", {})
    clear = sc.get("clear_radius", 10)
    for i in range(sc.get("count", 20)):
        for _try in range(20):
            x, y = rng.uniform(-80, 110), rng.uniform(-70, -18)
            if not (-14 < x < 10 and -30 < y < 6):
                break
        make_tree(f"treeS{i}", x, y, rng.uniform(0.8, 1.6), rng.randrange(6))

    # bushes ------------------------------------------------------------------
    bush_m = new_mat("bush", (0.025, 0.085, 0.015, 1), rough=0.9)
    for i in range(scene_cfg.get("bushes", {}).get("count", 180)):
        x, y = rng.uniform(-90, 90), rng.uniform(-70, 60)
        if abs(y + 5) < 2.5:
            continue
        r = rng.uniform(0.25, 0.9)
        o = prim("ico_sphere_add", f"bush{i}", col, bush_m, radius=r,
                 subdivisions=1, location=(x, y, r * 0.45))
        o.scale = (1, 1, rng.uniform(0.5, 0.75))
        o.rotation_euler = (0, 0, rng.uniform(0, 6.28))
        shade_smooth(o)

    # grass tufts (cheap cones, near the corridor for speed cues) -------------
    tuft_m = new_mat("tuft", (0.03, 0.10, 0.015, 1), rough=0.95)
    gcfg = scene_cfg.get("grass_patches", {})
    for i in range(gcfg.get("count", 600)):
        a = gcfg.get("area", [-35, 90, -26, 9])
        x, y = rng.uniform(a[0], a[1]), rng.uniform(a[2], a[3])
        h = rng.uniform(0.08, 0.3)
        o = prim("cone_add", f"tuft{i}", col, tuft_m, radius1=rng.uniform(0.04, 0.12),
                 radius2=0.0, depth=h, vertices=5, location=(x, y, h / 2))
        o.rotation_euler = (rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15),
                            rng.uniform(0, 6.28))
    return ground


# --------------------------------------------------------------------- drones
def build_interceptor(col):
    """x500-class quad, ~0.5 m wheelbase, dark + orange accents, camera pod."""
    root = bpy.data.objects.new("interceptor", None)
    col.objects.link(root)
    body_m = new_mat("ic_body", (0.018, 0.018, 0.02, 1), rough=0.45, metal=0.3)
    acc_m = new_mat("ic_accent", (0.75, 0.22, 0.02, 1), rough=0.5)
    pod_m = new_mat("ic_pod", (0.04, 0.04, 0.045, 1), rough=0.3)
    lens_m = new_mat("ic_lens", (0.01, 0.01, 0.012, 1), rough=0.05, metal=0.8)
    prop_m = bpy.data.materials.new("ic_prop")
    prop_m.use_nodes = True
    pb = prop_m.node_tree.nodes["Principled BSDF"]
    pb.inputs["Base Color"].default_value = (0.02, 0.02, 0.02, 1)
    pb.inputs["Alpha"].default_value = 0.22
    prop_m.blend_method = "BLEND"

    def P(o):
        o.parent = root
        return o

    b = P(prim("cube_add", "ic_core", col, body_m, size=1))
    b.scale = (0.16, 0.16, 0.062)
    top = P(prim("cube_add", "ic_top", col, acc_m, size=1,
                 location=(0, 0, 0.034)))
    top.scale = (0.10, 0.10, 0.022)
    arm_r, motor_z = 0.25, 0.028
    for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (1, -1), (-1, -1)]):
        ax, ay = sx * arm_r * 0.7071, sy * arm_r * 0.7071
        arm = P(prim("cylinder_add", f"ic_arm{i}", col, body_m, radius=0.012,
                     depth=arm_r, vertices=8))
        arm.location = (ax / 2, ay / 2, 0.0)
        arm.rotation_euler = Euler((0, math.pi / 2, math.atan2(ay, ax)), "XYZ")
        mot = P(prim("cylinder_add", f"ic_motor{i}", col, pod_m, radius=0.021,
                     depth=0.03, location=(ax, ay, motor_z)))
        disc = P(prim("cylinder_add", f"ic_prop{i}", col, prop_m, radius=0.165,
                      depth=0.0035, location=(ax, ay, motor_z + 0.02)))
        shade_smooth(mot), shade_smooth(disc)
    pod = P(prim("cube_add", "ic_campod", col, pod_m, size=1,
                 location=(0.13, 0, -0.01)))
    pod.scale = (0.035, 0.045, 0.04)
    lens = P(prim("cylinder_add", "ic_lensc", col, lens_m, radius=0.016,
                  depth=0.02, location=(0.168, 0, -0.01)))
    lens.rotation_euler = (0, math.pi / 2, 0)
    legs_m = body_m
    for sy in (-1, 1):
        leg = P(prim("cylinder_add", f"ic_leg{sy}", col, legs_m, radius=0.008,
                     depth=0.16, location=(0, sy * 0.09, -0.1)))
        foot = P(prim("cylinder_add", f"ic_foot{sy}", col, legs_m, radius=0.008,
                      depth=0.22, location=(0, sy * 0.12, -0.175)))
        foot.rotation_euler = (0, math.pi / 2, 0)
        leg.rotation_euler = (sy * 0.28, 0, 0)
    shade_smooth(b)
    return root


def build_target(col):
    """FPV quad + the REAL tag placard, laid out like models/fpv_target:
    placard centre at local origin FACING LOCAL -X, body 0.14 m at +X."""
    root = bpy.data.objects.new("target", None)
    col.objects.link(root)
    body_m = new_mat("tg_body", (0.09, 0.02, 0.02, 1), rough=0.5)
    pod_m = new_mat("tg_pod", (0.03, 0.03, 0.035, 1), rough=0.4)
    prop_m = bpy.data.materials.get("ic_prop")
    board_m = new_mat("tg_board", (0.82, 0.82, 0.8, 1), rough=0.7)

    tag_m = bpy.data.materials.new("tg_tag")
    tag_m.use_nodes = True
    nt = tag_m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    img = nt.nodes.new("ShaderNodeTexImage")
    img.image = bpy.data.images.load(TAG_PNG)
    img.interpolation = "Closest"
    nt.links.new(img.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.65

    def P(o):
        o.parent = root
        return o

    # placard: 0.625 m textured square facing local -X (normal = -X), thin
    board = P(prim("cube_add", "tg_boardback", col, board_m, size=1,
                   location=(0.014, 0, 0)))
    board.scale = (0.012, 0.68, 0.68)
    tag = P(prim("plane_add", "tg_tag", col, tag_m, size=0.625))
    tag.rotation_euler = Euler((0, -math.pi / 2, 0), "XYZ")
    tag.location = (-0.001, 0, 0)
    # body behind the placard at +X (hidden from a chaser, visible obliquely)
    b = P(prim("cube_add", "tg_core", col, body_m, size=1, location=(0.16, 0, 0)))
    b.scale = (0.13, 0.07, 0.038)
    cam = P(prim("cube_add", "tg_fpvcam", col, pod_m, size=1,
                 location=(0.25, 0, 0.012)))
    cam.scale = (0.018, 0.02, 0.018)
    cam.rotation_euler = (0, -0.4, 0)
    for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (1, -1), (-1, -1)]):
        ax, ay = 0.16 + sx * 0.08, sy * 0.08
        arm = P(prim("cube_add", f"tg_arm{i}", col, body_m, size=1))
        arm.location = ((0.16 + ax) / 2, ay / 2, -0.004)
        arm.scale = (0.055, 0.012, 0.006)
        arm.rotation_euler = (0, 0, math.atan2(ay, ax - 0.16))
        mot = P(prim("cylinder_add", f"tg_motor{i}", col, pod_m, radius=0.013,
                     depth=0.018, location=(ax, ay, 0.008)))
        disc = P(prim("cylinder_add", f"tg_prop{i}", col, prop_m, radius=0.0635,
                      depth=0.002, location=(ax, ay, 0.018)))
        shade_smooth(mot), shade_smooth(disc)
    return root


# ------------------------------------------------------------------- shot rigs
def resolve_t(expr, ev):
    """'go-3', 'first_det+1.5', 'cpa', 'end' -> sim seconds."""
    if isinstance(expr, (int, float)):
        return float(expr)
    base = {"go": ev["go_t"], "first_det": ev["first_det_t"],
            "last_det": ev["last_det_t"], "cpa": ev["t_cpa"]}
    for k, v in base.items():
        if expr.startswith(k):
            rest = expr[len(k):]
            return v + (float(rest) if rest else 0.0)
    return float(expr)


def camera_pose(rig, ts, smp, rng_shake):
    """-> (pos, quat). Rigs: side_track / chase / static / target_back /
    orbit_cpa. All read the drone states at sim time ts."""
    vp = Vector(smp.at("veh_pos_enu", ts))
    gp = Vector(smp.at("tgt_pos_enu", ts))
    kind = rig["type"]
    off = Vector(rig.get("offset", [0, -10, 2]))
    if kind == "static":
        pos = Vector(rig["pos"])
        aim = {"veh": vp, "tgt": gp, "mid": (vp + gp) / 2}[rig.get("aim", "mid")]
    elif kind == "static_cpa":
        pos = Vector(rig["_cpa_point"]) + off
        pos.z = max(pos.z, 0.3)
        aim = {"veh": vp, "tgt": gp, "mid": (vp + gp) / 2}[rig.get("aim", "mid")]
    elif kind == "side_track":
        w = rig.get("weight", 0.5)              # 0 = vehicle, 1 = target
        mid = vp * (1 - w) + gp * w
        pos = mid + off
        pos.z = max(pos.z, 0.4)
        aim = mid
    elif kind == "chase":
        head = Vector(smp.at("veh_pos_enu", ts + 0.25)) - vp
        head.z = 0
        head = head.normalized() if head.length > 1e-6 else Vector((1, 0, 0))
        left = Vector((-head.y, head.x, 0))
        pos = vp - head * off.x + left * off.y + Vector((0, 0, off.z))
        aim = gp if rig.get("aim", "tgt") == "tgt" else vp
    elif kind == "target_back":
        head = Vector(smp.at("tgt_pos_enu", ts + 0.25)) - gp
        head.z = 0
        head = head.normalized() if head.length > 1e-6 else Vector((1, 0, 0))
        left = Vector((-head.y, head.x, 0))
        pos = gp + head * off.x + left * off.y + Vector((0, 0, off.z))
        aim = {"veh": vp, "tgt": gp, "mid": (vp + gp) / 2}[rig.get("aim", "veh")]
    elif kind == "orbit_cpa":
        c = Vector(rig["_cpa_point"])
        r = rig.get("radius", 4.0)
        om = rig.get("omega_deg_s", 25.0)
        a0 = math.radians(rig.get("angle0_deg", 200.0))
        a = a0 + math.radians(om) * (ts - rig["_t0"])
        pos = c + Vector((r * math.cos(a), r * math.sin(a),
                          rig.get("height", 1.2)))
        aim = (vp + gp) / 2
    else:
        raise ValueError(f"unknown rig type {kind}")
    aim = aim + Vector(rig.get("aim_offset", [0, 0, 0]))
    sh = rig.get("shake", 0.0)
    roll = 0.0
    if sh:
        roll = sh * 0.01 * math.sin(ts * 1.7 + rng_shake)
        aim = aim + Vector((sh * 0.05 * math.sin(ts * 2.3 + rng_shake),
                            sh * 0.05 * math.sin(ts * 1.9 + 2 + rng_shake),
                            sh * 0.03 * math.sin(ts * 2.9 + 4 + rng_shake)))
    return pos, look_at_quat(pos, aim, roll)


# ----------------------------------------------------------------------- main
def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--shots", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--shot", default=None, help="render only this shot")
    ap.add_argument("--still", default=None, help="'SHOT@SIMT' single frame")
    ap.add_argument("--preview", action="store_true")
    args = ap.parse_args(argv)

    run = json.load(open(args.run))
    scene_cfg = json.load(open(args.scene))
    shots_cfg = json.load(open(args.shots))
    smp = Sampler(run)
    ev = run["events"]
    os.makedirs(args.out, exist_ok=True)

    # fresh scene
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    col = bpy.data.collections.new("set")
    sc.collection.children.link(col)

    rend = scene_cfg.get("render", {})
    fps = rend.get("fps", 30)
    sc.render.engine = "CYCLES"
    sc.cycles.device = "GPU"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "CUDA"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = d.type != "CPU"
    sc.cycles.samples = 24 if args.preview else rend.get("samples", 96)
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = rend.get("res", [1920, 1080])
    if args.preview:
        sc.render.resolution_percentage = 50
    sc.render.fps = fps
    sc.render.film_transparent = False
    sc.render.use_motion_blur = True
    sc.render.motion_blur_shutter = rend.get("motion_blur", 0.5)
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Punchy"
    sc.view_settings.exposure = scene_cfg.get("exposure", -0.6)

    build_environment(scene_cfg, col)
    veh = build_interceptor(col)
    tgt = build_target(col)
    veh.rotation_mode = tgt.rotation_mode = "QUATERNION"

    cam_data = bpy.data.cameras.new("cam")
    cam = bpy.data.objects.new("cam", cam_data)
    col.objects.link(cam)
    sc.camera = cam
    cam.rotation_mode = "QUATERNION"

    cpa_t = ev["t_cpa"]
    cpa_pt = [(a + b) / 2 for a, b in zip(smp.at("veh_pos_enu", cpa_t),
                                          smp.at("tgt_pos_enu", cpa_t))]

    manifest = {"fps": fps, "run": os.path.abspath(args.run), "shots": []}
    shots = shots_cfg["shots"]
    if args.still:
        wanted = []
        for spec in args.still.split(";"):
            name, ts = spec.split("@")
            hit = [dict(s, _still=float(resolve_t(ts, ev)), _sfx=ts)
                   for s in shots if s["name"] == name]
            if not hit:
                raise SystemExit(f"still: no shot named {name}")
            wanted += hit
        shots = wanted
    elif args.shot:
        shots = [s for s in shots if s["name"] == args.shot]

    for s in shots:
        rig = dict(s["camera"])
        rig["_cpa_point"] = cpa_pt
        t0 = resolve_t(s["t0"], ev)
        t1 = resolve_t(s["t1"], ev)
        rig["_t0"] = t0
        speed = s.get("speed", 1.0)
        nf = max(1, int(round((t1 - t0) / speed * fps)))
        cam_data.lens = s.get("lens_mm", 40)
        cam_data.dof.use_dof = bool(s.get("dof", False))
        if s.get("dof", False):
            cam_data.dof.focus_object = tgt
            cam_data.dof.aperture_fstop = s.get("fstop", 4.0)
        shake_seed = hash(s["name"]) % 100

        if "_still" in s:
            frames = [(1, s["_still"])]
        else:
            frames = [(i + 1, t0 + i * speed / fps) for i in range(nf)]

        # keyframe everything (motion blur needs animated values)
        for o in (veh, tgt, cam):
            o.animation_data_clear()
        for fr, ts in frames:
            veh.location = smp.at("veh_pos_enu", ts)
            veh.rotation_quaternion = smp.quat("veh_quat_wxyz", ts)
            tgt.location = smp.at("tgt_pos_enu", ts)
            tgt.rotation_quaternion = smp.quat("tgt_quat_wxyz", ts)
            cpos, cq = camera_pose(rig, ts, smp, shake_seed)
            cam.location, cam.rotation_quaternion = cpos, cq
            for o in (veh, tgt, cam):
                o.keyframe_insert("location", frame=fr)
                o.keyframe_insert("rotation_quaternion", frame=fr)

        sdir = os.path.join(args.out, "shots", s["name"])
        os.makedirs(sdir, exist_ok=True)
        sc.frame_start, sc.frame_end = 1, len(frames)
        sc.render.filepath = sdir + "/"
        sc.render.image_settings.file_format = "PNG"
        print(f"[blender] shot {s['name']}: {len(frames)} frames "
              f"(sim {t0:.2f}..{t1:.2f} s, speed {speed}x)")
        if "_still" in s:
            sc.frame_set(1)
            sc.render.filepath = os.path.join(
                sdir, f"still_{s.get('_sfx','x').replace('+','p')}.png")
            bpy.ops.render.render(write_still=True)
        else:
            bpy.ops.render.render(animation=True)
        manifest["shots"].append({
            "name": s["name"], "dir": sdir, "speed": speed,
            "frames": [[fr, round(ts, 4)] for fr, ts in frames]})

    with open(os.path.join(args.out, "manifest.json"), "w") as fh:
        json.dump(manifest, fh)
    print(f"[blender] done -> {args.out}/manifest.json")


if __name__ == "__main__":
    main()
