"""Build the aircraft in Blender. Run under `blender --background`."""

import csv
import math
import os
import sys
import time

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import spec              # noqa: E402
import mesh as meshlib   # noqa: E402
import materials         # noqa: E402
from parts import (fuselage, wing, tail, intake, canopy,   # noqa: E402
                   gear, internals, engine_mount)

MM = 0.001

MODULES = [
    ("fuselage", fuselage), ("wing", wing), ("tail", tail),
    ("intake", intake), ("canopy", canopy), ("gear", gear),
    ("internals", internals), ("engine", engine_mount),
]

COLLECTIONS = ["01 Fuselage", "02 Wing", "03 Tail", "04 Intake and Duct",
               "05 Canopy", "06 Landing Gear", "07 Engine",
               "08 RC Systems", "09 Structure"]


def collection_for(name):
    n = name.lower()
    if n.startswith("engine_"):
        return "07 Engine"
    if n.startswith(("lipo", "esc", "receiver", "servo", "wiring")):
        return "08 RC Systems"
    if n.startswith(("bhd_", "spar")):
        return "09 Structure"
    if n.startswith(("gear_", "wheel")):
        return "06 Landing Gear"
    if n.startswith("canopy"):
        return "05 Canopy"
    if n.startswith(("intake", "duct")):
        return "04 Intake and Duct"
    if n.startswith(("stabilator", "vtail", "rudder", "ventral")):
        return "03 Tail"
    if n.startswith(("wing", "flaperon")):
        return "02 Wing"
    return "01 Fuselage"


def material_for(name):
    n = name.lower()
    if n.startswith("engine_"):
        return "engine"
    best, best_len = spec.DEFAULT_MATERIAL, -1
    for key, mat in spec.MATERIAL_MAP.items():
        if key in n and len(key) > best_len:
            best, best_len = mat, len(key)
    return best


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"
    sc.unit_settings.length_unit = "MILLIMETERS"


def make_object(name, verts, faces, coll):
    me = bpy.data.meshes.new(name)
    me.from_pydata([(x * MM, y * MM, z * MM) for (x, y, z) in verts],
                   [], [list(f) for f in faces])
    me.validate(verbose=False)
    me.update()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def apply_cutters(obj, cv, cf):
    cutter = make_object(obj.name + "__cut", cv, cf, bpy.context.scene.collection)
    m = obj.modifiers.new("cut", "BOOLEAN")
    m.operation = "DIFFERENCE"
    m.solver = "EXACT"
    m.object = cutter
    bpy.context.view_layer.objects.active = obj
    ok = True
    try:
        bpy.ops.object.modifier_apply(modifier=m.name)
    except RuntimeError as e:
        print(f"    ! boolean failed on {obj.name}: {e}")
        obj.modifiers.remove(m)
        ok = False
    bpy.data.objects.remove(cutter, do_unlink=True)
    return ok


def array_rotational(obj, count, axis_x, axis_z):
    """Rotate copies about the engine's own axis, which is offset from the
    aircraft datum -- so shift to the axis, replicate, and shift back."""
    verts = [(v.co.x - axis_x, v.co.y, v.co.z - axis_z) for v in obj.data.vertices]
    faces = [tuple(p.vertices) for p in obj.data.polygons]
    nv, nf = meshlib.replicate(verts, faces, count)
    nv = [(x + axis_x, y, z + axis_z) for (x, y, z) in nv]
    me = bpy.data.meshes.new(obj.name + "_arr")
    me.from_pydata(nv, [], [list(f) for f in nf])
    me.validate(verbose=False)
    me.update()
    old = obj.data
    obj.data = me
    bpy.data.meshes.remove(old)


def recalc_normals(obj):
    """Make normals point outward. Every part here is a closed manifold, so
    Blender can resolve winding reliably -- far more robust than trying to get
    the face order right by hand for each panel orientation and deflection."""
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode="OBJECT")
    obj.select_set(False)


def shade(obj, angle_deg=34.0):
    """Smooth, with sharp edges marked from face angles. The operator form of
    this fails silently in background mode, so do it directly."""
    me = obj.data
    for p in me.polygons:
        p.use_smooth = True
    limit = math.cos(math.radians(angle_deg))
    normals = [tuple(p.normal) for p in me.polygons]
    by_edge = {}
    for pi, poly in enumerate(me.polygons):
        for ek in poly.edge_keys:
            by_edge.setdefault(ek, []).append(pi)
    edges = {e.key: e for e in me.edges}
    n = 0
    for ek, fs in by_edge.items():
        e = edges.get(ek)
        if e is None:
            continue
        if len(fs) != 2:
            e.use_edge_sharp = True
            n += 1
            continue
        a, b = normals[fs[0]], normals[fs[1]]
        if sum(p * q for p, q in zip(a, b)) < limit:
            e.use_edge_sharp = True
            n += 1
    return n


def main():
    t0 = time.time()
    clear_scene()
    mats = materials.build_all()
    cols = {}
    for c in COLLECTIONS:
        col = bpy.data.collections.new(c)
        bpy.context.scene.collection.children.link(col)
        cols[c] = col

    rows, n_bool, n_ok, n_sharp = [], 0, 0, 0
    eng_axis_x = spec.ENGINE_X * MM      # unused for rotation, kept for clarity
    for modname, module in MODULES:
        t1 = time.time()
        built = module.build()
        arrays = getattr(module, "ARRAYS", {})
        objects = {k: v for k, v in built.items() if not k.startswith("cut:")}
        cutters = {k[4:]: v for k, v in built.items() if k.startswith("cut:")}

        for name, (v, f) in sorted(objects.items()):
            cname = collection_for(name)
            ob = make_object(name, v, f, cols[cname])
            if name in cutters:
                n_bool += 1
                if apply_cutters(ob, *cutters[name]):
                    n_ok += 1
            if name in arrays:
                array_rotational(ob, arrays[name], 0.0, spec.ENGINE_Z * MM)
            if not name.startswith("engine_"):
                recalc_normals(ob)
            mname = material_for(name)
            ob.data.materials.append(mats[mname])
            n_sharp += shade(ob)
            bb = meshlib.bbox([tuple(x.co) for x in ob.data.vertices])
            rows.append({
                "name": name, "collection": cname, "material": mname,
                "verts": len(ob.data.vertices), "faces": len(ob.data.polygons),
                "x_min_mm": round(bb[0] / MM, 1), "x_max_mm": round(bb[3] / MM, 1),
                "y_min_mm": round(bb[1] / MM, 1), "y_max_mm": round(bb[4] / MM, 1),
                "z_min_mm": round(bb[2] / MM, 1), "z_max_mm": round(bb[5] / MM, 1),
                "count": arrays.get(name, ""),
            })
        print(f"  [{modname}] {len(objects)} objects in {time.time()-t1:.1f}s")

    os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
    p = os.path.join(ROOT, "build", "parts.csv")
    with open(p, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    tv = sum(r["verts"] for r in rows)
    tf = sum(r["faces"] for r in rows)
    print(f"\n{len(rows)} objects | {tv:,} verts | {tf:,} faces")
    print(f"booleans: {n_ok}/{n_bool} | sharp edges: {n_sharp:,}")
    print(f"parts.csv -> {p}")
    blend = os.path.join(ROOT, "build", "rcjet.blend")
    bpy.ops.wm.save_as_mainfile(filepath=blend)
    print(f"blend     -> {blend}")
    print(f"total {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
