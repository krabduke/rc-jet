"""Export the aircraft. Run under `blender --background`.

    blender -b build/rcjet.blend -P plane/export.py -- glb | stl | all

STL is written in millimetres, so parts drop straight into a slicer at the size
they were designed.
"""

import math
import os
import sys

import bpy

def flatten_pbr_for_gltf():
    """Unlink procedural inputs from every Principled BSDF before writing glTF.

    materials.py links a noise texture into Roughness (and, on the hot
    section, into Base Color) so large machined surfaces do not read as flat
    CG plastic under Cycles. glTF cannot express a procedural, so the
    exporter's answer is to write no factor at all -- and an absent
    roughnessFactor does not mean "keep what the .blend had", it means 1.0.

    Every metal in the file therefore arrived in the browser as metalness 1.0
    with roughness 1.0. A fully rough metal has no diffuse term, so wherever it
    fails to catch a light it goes black; on a cylinder that is the top and the
    bottom. That is the "black bands" this project spent a commit working
    around by abandoning PBR altogether and baking lambert shading into vertex
    colours.

    The BSDF's own default_value still holds the palette number, so simply
    dropping the link restores a correct factor. Cycles renders run from the
    unmodified .blend; only the export path sees this.
    """
    import bpy
    n = 0
    for mat in bpy.data.materials:
        if not mat.use_nodes or not mat.node_tree:
            continue
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf is None:
            continue
        for slot in ("Roughness", "Metallic", "Base Color"):
            sock = bsdf.inputs.get(slot)
            if sock is None:
                continue
            for link in list(sock.links):
                mat.node_tree.links.remove(link)
                n += 1
    return n


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BUILD = os.path.join(ROOT, "build")


def meshes():
    return [o for o in bpy.data.objects if o.type == "MESH"]


def deselect():
    for o in bpy.data.objects:
        o.select_set(False)


def split_sharp(angle_deg=32.0):
    """Bake sharp edges into the geometry before export.

    glTF does not carry Blender's split normals, so without this the exported
    shells arrive with their normals averaged across profile corners and shade
    wrong. Applying an Edge Split modifier puts the split in the geometry
    itself, where any exporter preserves it.
    """
    n = 0
    for o in meshes():
        m = o.modifiers.new("split", "EDGE_SPLIT")
        m.split_angle = math.radians(angle_deg)
        m.use_edge_angle = True
        m.use_edge_sharp = True
        bpy.context.view_layer.objects.active = o
        try:
            bpy.ops.object.modifier_apply(modifier=m.name)
            n += 1
        except RuntimeError:
            o.modifiers.remove(m)
    print(f"  edge-split applied to {n} objects")


def export_glb(name="rcjet.glb", draco=True):
    path = os.path.join(BUILD, name)
    deselect()
    kw = {}
    if draco:
        kw = {"export_draco_mesh_compression_enable": True,
              "export_draco_mesh_compression_level": 6}
    n_flat = flatten_pbr_for_gltf()
    print(f"  unlinked {n_flat} procedural material inputs for glTF")
    bpy.ops.export_scene.gltf(
        filepath=path, export_format="GLB", use_selection=False,
        export_apply=True, export_yup=True, export_materials="EXPORT", **kw)
    print(f"  -> {path}  ({os.path.getsize(path) / 1e6:.1f} MB)")


def export_stl():
    out = os.path.join(BUILD, "stl")
    os.makedirs(out, exist_ok=True)
    n = 0
    for o in meshes():
        deselect()
        o.select_set(True)
        bpy.context.view_layer.objects.active = o
        path = os.path.join(out, o.name + ".stl")
        try:
            bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True,
                                  global_scale=1000.0)
        except AttributeError:
            bpy.ops.export_mesh.stl(filepath=path, use_selection=True,
                                    global_scale=1000.0)
        n += 1
    print(f"  -> {n} STL files in {out}")


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["all"]
    mode = argv[0]
    os.makedirs(BUILD, exist_ok=True)
    split_sharp()
    if mode in ("glb", "all"):
        export_glb()
    if mode in ("stl", "all"):
        export_stl()
