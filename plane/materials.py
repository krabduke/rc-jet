"""PBR materials for the airframe. Requires bpy.

Colours read as a real small RC jet: matte EPO foam airframe in a low-vis grey,
darker control surfaces, tinted canopy, ply bulkheads, bare carbon spar, and
the engine keeping its own alloy look.
"""

import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spec  # noqa: E402

PALETTE = spec.PALETTE


def build_all():
    out = {}
    for name, (rgb, metallic, rough) in PALETTE.items():
        mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
        mat.use_nodes = True
        nt = mat.node_tree
        b = nt.nodes.get("Principled BSDF")
        if b is None:
            b = nt.nodes.new("ShaderNodeBsdfPrincipled")
            nt.links.new(b.outputs[0], nt.nodes["Material Output"].inputs[0])
        b.inputs["Base Color"].default_value = (*rgb, 1.0)
        b.inputs["Metallic"].default_value = metallic
        b.inputs["Roughness"].default_value = rough
        if name == "glass":
            if "Transmission Weight" in b.inputs:
                b.inputs["Transmission Weight"].default_value = 0.82
            b.inputs["IOR"].default_value = 1.49
        if name == "airframe":
            _matte_speckle(nt, b)
        out[name] = mat
    return out


def _matte_speckle(nt, bsdf):
    """Fine noise on the foam so large panels do not read as flat plastic."""
    coord = nt.nodes.new("ShaderNodeTexCoord")
    coord.location = (-900, -180)
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = 420.0
    tex.inputs["Detail"].default_value = 4.0
    tex.location = (-650, -180)
    nt.links.new(coord.outputs["Object"], tex.inputs["Vector"])
    rng = nt.nodes.new("ShaderNodeMapRange")
    rng.inputs["From Min"].default_value = 0.35
    rng.inputs["From Max"].default_value = 0.65
    rng.inputs["To Min"].default_value = 0.56
    rng.inputs["To Max"].default_value = 0.70
    rng.location = (-420, -180)
    nt.links.new(tex.outputs["Fac"], rng.inputs["Value"])
    nt.links.new(rng.outputs["Result"], bsdf.inputs["Roughness"])
