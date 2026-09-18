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
    """Every mesh that is part of the model.

    Not the helpers. The cutaway leaves its one-metre boolean cutter in the
    scene -- it has to stay, a boolean modifier needs its object -- and the
    exploded shot that runs after it was framing on that instead of on the
    aeroplane: camera six metres back for a 490 mm model, which came out a
    quarter of the width of the frame. Anything named with a leading double
    underscore is scaffolding.
    """
    return [o for o in bpy.data.objects
            if o.type == "MESH" and not o.name.startswith("__")]


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
    # Every part's own box, not one box round the whole model.
    #
    # The framing is fitted to these corners, so what they enclose is what
    # ends up filling the frame -- and a single box has corners that are
    # empty air a long way outside the model at any three-quarter angle, so
    # the camera backs off to keep that air in shot. The union of the parts'
    # own boxes still contains every vertex and hugs the shape instead.
    CORNERS.clear()
    for o in meshes():
        for c in o.bound_box:
            CORNERS.append((o.matrix_world @ V(c)) - cen)
    return cen


def corners_of(prefixes):
    """Same as `collect_corners`, over named parts only.

    A whole-aeroplane frame is no use for looking at one thing. This is for
    the inlet, which is the view that tells you whether the lip, the duct and
    the skin are one surface or three -- and for a long time they were three.
    """
    from mathutils import Vector as V
    sel = [o for o in meshes() if o.name.startswith(tuple(prefixes))]
    if not sel:
        return collect_corners()
    xs, ys, zs = [], [], []
    for o in sel:
        for c in o.bound_box:
            w = o.matrix_world @ V(c)
            xs.append(w.x); ys.append(w.y); zs.append(w.z)
    cen = V(((min(xs)+max(xs))/2, (min(ys)+max(ys))/2, (min(zs)+max(zs))/2))
    CORNERS.clear()
    for o in sel:
        for c in o.bound_box:
            CORNERS.append((o.matrix_world @ V(c)) - cen)
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



def fit_resolution(dirv, budget=1_920_000, lo=0.75, hi=2.6):
    """Choose a frame shape that suits what the shot is of.

    An elevation is fitted to the subject, so a fixed 4:3 frame decides how
    much of it is empty: the car is 4.93 m long and 2.0 m wide, and in plan on
    a 4:3 frame it filled half the height whatever the camera did. The frame
    follows the subject's own proportions instead, at a constant pixel count,
    clamped so nothing comes out a letterbox.

    Call after collect_corners() and before setup_camera().
    """
    dirv = Vector(dirv).normalized()
    up = Vector((0, 1, 0)) if abs(dirv.z) > 0.999 else Vector((0, 0, 1))
    right = dirv.cross(up).normalized()
    camup = right.cross(dirv).normalized()
    w = max(abs(c.dot(right)) for c in CORNERS) or 1.0
    h = max(abs(c.dot(camup)) for c in CORNERS) or 1.0
    a = min(hi, max(lo, w / h))
    rx = int(round(math.sqrt(budget * a) / 2) * 2)
    ry = int(round(math.sqrt(budget / a) / 2) * 2)
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y = rx, ry
    return rx, ry

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
        # Fit the orthographic frame to what the camera actually sees, not to
        # the model's largest dimension times a constant. ortho_scale is the
        # width, so the height it gives is width/aspect -- and on a 16:9 frame
        # that is a lot less. The old factor of 2.3 cut the top off the
        # plenum on the front elevation.
        # A plan view looks straight down, where world up and the view
        # direction are the same line and their cross product is zero. Pick
        # the other reference there, or the frame comes out zero wide and the
        # render is empty.
        up = Vector((0, 1, 0)) if abs(dirv.z) > 0.999 else Vector((0, 0, 1))
        right = dirv.cross(up).normalized()
        camup = right.cross(dirv).normalized()
        aspect = (bpy.context.scene.render.resolution_x
                  / bpy.context.scene.render.resolution_y)
        half_w = max(abs(c.dot(right)) for c in CORNERS)
        half_h = max(abs(c.dot(camup)) for c in CORNERS)
        cd.ortho_scale = max(2 * half_w, 2 * half_h * aspect) * 1.06
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
        # EXACT, not FLOAT. The fast solver gave up silently on the fuselage
        # skin once the cockpit aperture was cut into it, and a boolean that
        # does nothing looks exactly like a cutaway of a solid aeroplane.
        m.solver = "EXACT"
        m.object = cut


def mode_hero(s):
    setup_render(s); setup_world(); setup_lights()
    c = collect_corners()
    setup_camera((0.46, 0.80, -0.39), c, lens=76)
    shoot("01_hero")


DIRV_TOP = (0.0, 0.0, -1.0)


def mode_top(s):
    setup_render(s, res=(1600, 1200)); setup_world(0.75); setup_lights()
    c = collect_corners()
    fit_resolution(DIRV_TOP)
    setup_camera(DIRV_TOP, c, ortho=True)
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
        "05 Cockpit": (0.0, 0.0, 0.052),
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


def mode_intake(s):
    """Head-on at the inlet, close.

    Worth its own view: everything that has gone wrong with this intake has
    been invisible from every other angle. Two concentric lips, a ring of
    studs round the rim, a one-unit ledge where the lip met the duct, and a
    plywood former showing through the mouth -- all of it only reads looking
    straight down the throat.
    """
    import math
    setup_render(s, res=(1600, 1100)); setup_world(0.35)
    c = corners_of(("intake_lip",))
    # placed by hand rather than by the fitter: the fitter frames a bounding
    # box, and the thing worth seeing here is what is BEHIND the rim
    cd = bpy.data.cameras.new("cam_intake"); cd.lens = 80
    ob = bpy.data.objects.new("cam_intake", cd)
    bpy.context.scene.collection.objects.link(ob)
    bpy.context.scene.camera = ob
    ob.location = (c.x - 0.190, c.y - 0.034, c.z - 0.012)
    # Below the lip centre looking slightly up and aft, because the inlet is
    # under the nose and drooped: from anywhere above it the fuselage is in
    # the way and you photograph the canopy instead. Aimed with a track
    # quaternion rather than typed Euler angles -- the sign of the pitch is
    # not guessable and three renders went on finding that out.
    look = Vector((c.x + 0.030, c.y, c.z + 0.002)) - Vector(ob.location)
    ob.rotation_euler = look.to_track_quat("-Z", "Y").to_euler()
    for (dx, dy, dz, e) in ((-0.10, -0.09, 0.07, 3.2), (-0.08, 0.08, 0.05, 1.8),
                            (-0.05, 0.0, -0.10, 2.6), (-0.16, 0.0, 0.01, 4.0)):
        ld = bpy.data.lights.new("l_intake", "AREA")
        ld.energy, ld.size = e, 0.10
        lo = bpy.data.objects.new("l_intake", ld)
        lo.location = (c.x + dx, c.y + dy, c.z + dz)
        bpy.context.scene.collection.objects.link(lo)
        d = (-dx, -dy, -dz)
        lo.rotation_euler = (math.atan2(math.hypot(d[0], d[1]), d[2]), 0.0,
                             math.atan2(d[1], d[0]) + math.pi / 2)
    shoot("06_intake")


MODES = {"hero": mode_hero, "top": mode_top,
         "cutaway": mode_cutaway, "exploded": mode_exploded,
         "intake": mode_intake}

if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["hero"]
    mode = argv[0]
    samples = int(argv[1]) if len(argv) > 1 else 128
    if mode == "all":
        for m in ("hero", "top", "cutaway", "exploded"):
            MODES[m](samples)
    else:
        MODES[mode](samples)
