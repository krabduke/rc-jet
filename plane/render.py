"""Render the aircraft. Run under `blender --background`.

    blender -b build/rcjet.blend -P plane/render.py -- <mode> [samples]

Modes: hero, top, cutaway, exploded, all
"""

import math
import os
import sys

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "renders")

CORNERS = []


def meshes():
    return [o for o in bpy.data.objects if o.type == "MESH"]


def setup_render(samples=128, res=(1920, 1080)):
    s = bpy.context.scene
    s.render.engine = "CYCLES"
    s.cycles.samples = samples
    s.cycles.use_denoising = True
    s.cycles.max_bounces = 6
    s.render.resolution_x, s.render.resolution_y = res
    s.view_settings.view_transform = "AgX"
    s.view_settings.look = "AgX - Base Contrast"
    s.view_settings.exposure = -0.2
    try:
        pr = bpy.context.preferences.addons["cycles"].preferences
        pr.compute_device_type = "METAL"
        pr.get_devices()
        for d in pr.devices:
            d.use = True
        s.cycles.device = "GPU"
    except Exception as e:
        print("  (CPU render:", e, ")")


def setup_world(strength=0.55):
    w = bpy.data.worlds.get("World") or bpy.data.worlds.new("World")
    bpy.context.scene.world = w
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    grad = nt.nodes.new("ShaderNodeTexGradient")
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    tex = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Rotation"].default_value = (math.radians(90), 0, 0)
    ramp.color_ramp.elements[0].color = (0.035, 0.040, 0.048, 1)
    ramp.color_ramp.elements[1].color = (0.240, 0.265, 0.300, 1)
    nt.links.new(tex.outputs["Generated"], mp.inputs["Vector"])
    nt.links.new(mp.outputs["Vector"], grad.inputs["Vector"])
    nt.links.new(grad.outputs["Color"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bg.inputs["Color"])
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])
    bg.inputs["Strength"].default_value = strength


def area(name, loc, rot, energy, size):
    d = bpy.data.lights.new(name, type="AREA")
    d.energy = energy
    d.size = size
    o = bpy.data.objects.new(name, d)
    o.location = loc
    o.rotation_euler = rot
    bpy.context.scene.collection.objects.link(o)
    return o


def setup_lights():
    for o in [o for o in bpy.data.objects if o.type == "LIGHT"]:
        bpy.data.objects.remove(o, do_unlink=True)
    area("key",  (-0.30, -0.72, 0.66), (math.radians(46), 0, math.radians(-24)), 70, 0.9)
    area("fill", ( 0.34,  0.62,-0.22), (math.radians(-62), 0, math.radians(152)), 26, 1.1)
    area("rim",  ( 0.72,  0.30, 0.42), (math.radians(62), 0, math.radians(118)), 34, 0.5)
    area("bounce", (0.12, -0.10,-0.62), (math.radians(180), 0, 0), 14, 1.4)


def collect_corners():
    from mathutils import Vector as V
    xs, ys, zs = [], [], []
    for o in meshes():
        for c in o.bound_box:
            w = o.matrix_world @ V(c)
            xs.append(w.x); ys.append(w.y); zs.append(w.z)
    cen = V(((min(xs)+max(xs))/2, (min(ys)+max(ys))/2, (min(zs)+max(zs))/2))
    size = V((max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs)))
    CORNERS.clear()
    for sx in (-.5, .5):
        for sy in (-.5, .5):
            for sz in (-.5, .5):
                CORNERS.append(V((size.x*sx, size.y*sy, size.z*sz)))
    return cen


def fit_distance(cam, dirv, margin=1.06):
    """Smallest standoff that keeps every bounding-box corner in frame.

    Fitting the bounding sphere instead wastes most of the frame on something
    long and thin, which an aircraft very much is.
    """
    up = Vector((0, 0, 1))
    right = dirv.cross(up).normalized()
    camup = right.cross(dirv).normalized()
    sc = bpy.context.scene
    aspect = sc.render.resolution_x / sc.render.resolution_y
    th = math.tan(cam.data.angle / 2.0)      # sensor-fit axis is horizontal
    tv = th / aspect
    d = 0.0
    for c in CORNERS:
        depth = c.dot(dirv)
        d = max(d, abs(c.dot(right)) / th - depth,
                   abs(c.dot(camup)) / tv - depth)
    return d * margin


def setup_camera(dirv, centre, lens=70.0, ortho=False):
    cd = bpy.data.cameras.new("cam")
    cd.lens = lens
    if ortho:
        cd.type = "ORTHO"
    ob = bpy.data.objects.new("cam", cd)
    bpy.context.scene.collection.objects.link(ob)
    bpy.context.scene.camera = ob
    dirv = Vector(dirv).normalized()
    if ortho:
        span = max(max(abs(c.x), abs(c.y), abs(c.z)) for c in CORNERS) * 2.3
        cd.ortho_scale = span
        ob.location = centre - dirv * 2.0
    else:
        ob.location = centre - dirv * fit_distance(ob, dirv)
    ob.rotation_euler = dirv.to_track_quat("-Z", "Y").to_euler()
    return ob


def shoot(name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name + ".png")
    bpy.context.scene.render.filepath = p
    bpy.ops.render.render(write_still=True)
    print("  ->", p)


SHELL = ("fuselage_skin", "canopy_glass", "duct_inlet", "wing_l", "wing_r",
         "intake_lip")


def section(centre):
    """Peel the skin only, leaving the structure and systems whole -- the same
    reason the engine project sections its casings and not its rotor."""
    bpy.ops.mesh.primitive_cube_add(size=1)
    cut = bpy.context.active_object
    cut.name = "__section"
    cut.scale = (1.2, 0.5, 0.6)
    cut.location = (centre.x, centre.y - 0.25, centre.z)
    cut.hide_render = True
    for o in meshes():
        if o is cut or not any(o.name.startswith(s) for s in SHELL):
            continue
        m = o.modifiers.new("sec", "BOOLEAN")
        m.operation = "DIFFERENCE"
        m.solver = "FLOAT"
        m.object = cut


def mode_hero(s):
    setup_render(s); setup_world(); setup_lights()
    c = collect_corners()
    setup_camera((0.46, 0.80, -0.39), c, lens=76)
    shoot("01_hero")


def mode_top(s):
    setup_render(s, res=(1600, 1200)); setup_world(0.75); setup_lights()
    c = collect_corners()
    setup_camera((0.0, 0.0, -1.0), c, ortho=True)
    shoot("02_plan")


def mode_cutaway(s):
    setup_render(s); setup_world(0.5); setup_lights()
    c = collect_corners()
    section(c)
    area("bay", (0.0, -0.55, 0.16), (math.radians(74), 0, 0), 30, 0.8)
    setup_camera((0.30, 0.86, -0.42), c, lens=74)
    shoot("03_cutaway")


def mode_exploded(s):
    setup_render(s); setup_world(); setup_lights()
    moves = {
        "02 Wing": (0.0, 0.0, -0.070),
        "03 Tail": (0.055, 0.0, 0.034),
        "05 Canopy": (0.0, 0.0, 0.052),
        "06 Landing Gear": (0.0, 0.0, -0.046),
        "07 Engine": (0.080, 0.0, 0.0),
        "08 RC Systems": (-0.012, 0.0, 0.064),
        "09 Structure": (0.0, 0.0, 0.094),
        "04 Intake and Duct": (-0.034, 0.0, -0.034),
    }
    for cname, (dx, dy, dz) in moves.items():
        col = bpy.data.collections.get(cname)
        if not col:
            continue
        for o in col.objects:
            o.location.x += dx
            o.location.y += dy
            o.location.z += dz
    c = collect_corners()
    setup_camera((0.40, 0.82, -0.41), c, lens=72)
    shoot("04_exploded")


MODES = {"hero": mode_hero, "top": mode_top,
         "cutaway": mode_cutaway, "exploded": mode_exploded}

if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["hero"]
    mode = argv[0]
    samples = int(argv[1]) if len(argv) > 1 else 128
    if mode == "all":
        for m in ("hero", "top", "cutaway", "exploded"):
            MODES[m](samples)
    else:
        MODES[mode](samples)
