"""PBR materials. Requires bpy.

Colours and roughness are chosen to read as the actual alloys: titanium in the
cold section, nickel superalloy discoloured by heat in the hot section, and
pale ceramic thermal-barrier coating on the augmentor liner and nozzle flaps.
"""

import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spec  # noqa: E402

PALETTE = spec.PALETTE




def build_all():
    """Create every material once and return {name: bpy material}."""
    out = {}
    for name, (rgb, metallic, rough) in PALETTE.items():
        mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
        mat.use_nodes = True
        nt = mat.node_tree
        bsdf = nt.nodes.get("Principled BSDF")
        if bsdf is None:
            bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
            nt.links.new(bsdf.outputs[0], nt.nodes["Material Output"].inputs[0])
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Metallic"].default_value = metallic
        bsdf.inputs["Roughness"].default_value = rough
        if "Anisotropic" in bsdf.inputs and metallic > 0.5:
            bsdf.inputs["Anisotropic"].default_value = 0.35
        _add_wear(nt, bsdf, name)
        out[name] = mat
    return out


def _add_wear(nt, bsdf, name):
    """Break up the roughness with a fine noise so large machined surfaces do
    not read as flat CG plastic under a studio light."""
    scale = {"hot_nickel": 95.0, "thermal_barrier": 80.0}.get(name, 130.0)
    strength = {"hot_nickel": 0.09, "thermal_barrier": 0.09}.get(name, 0.05)

    coord = nt.nodes.new("ShaderNodeTexCoord")
    coord.location = (-1000, -200)
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = scale
    tex.inputs["Detail"].default_value = 5.0
    tex.location = (-700, -200)
    nt.links.new(coord.outputs["Object"], tex.inputs["Vector"])

    ramp = nt.nodes.new("ShaderNodeMapRange")
    ramp.inputs["From Min"].default_value = 0.30
    ramp.inputs["From Max"].default_value = 0.70
    base_rough = bsdf.inputs["Roughness"].default_value
    ramp.inputs["To Min"].default_value = max(0.02, base_rough - strength)
    ramp.inputs["To Max"].default_value = min(0.98, base_rough + strength)
    ramp.location = (-450, -200)

    nt.links.new(tex.outputs["Fac"], ramp.inputs["Value"])
    nt.links.new(ramp.outputs["Result"], bsdf.inputs["Roughness"])

    # heat discolouration on the hot section: tint base colour with the same noise
    if name == "hot_nickel":
        mix = nt.nodes.new("ShaderNodeMix")
        mix.data_type = "RGBA"
        mix.inputs["A"].default_value = (0.310, 0.246, 0.196, 1.0)
        mix.inputs["B"].default_value = (0.246, 0.214, 0.208, 1.0)
        mix.location = (-450, 100)
        nt.links.new(tex.outputs["Fac"], mix.inputs["Factor"])
        nt.links.new(mix.outputs["Result"], bsdf.inputs["Base Color"])
