"""The F110-GE-129 from the sibling project, scaled down and installed.

Nothing about the engine is re-modelled here. Its generators are imported as
they are, built at full size in their own millimetres, then scaled by
spec.ENGINE_SCALE and translated so the inlet flange lands on the firewall.

Tessellation is turned down first: at 140 mm long the engine's 96-segment
revolutions and 2044 individually lofted airfoils are far finer than anything
that can be seen, and would otherwise dominate the whole model's poly count.
"""

import sys, os

HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(HERE, "f110"))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import importlib

import spec                                    # the aircraft spec



def _load_engine():
    """Import the engine package with f110/ ahead on the path, so its modules
    resolve their own `spec` and not the aircraft's."""
    saved = {k: v for k, v in sys.modules.items()
             if k in ("spec", "mesh", "airfoil", "materials")
             or k.startswith("parts")}
    for k in list(saved):
        del sys.modules[k]

    f110_dir = os.path.join(HERE, "f110")
    sys.path.insert(0, f110_dir)
    try:
        espec = importlib.import_module("spec")
        # Coarsen the engine before anything is built.
        espec.RES.update({
            "airfoil_chord_pts": 14,
            "airfoil_span_pts": 4,
            "revolve_segments": 32,
            "small_revolve": 10,
            "pipe_segments": 8,
        })
        mods = [importlib.import_module(f"parts.{m}") for m in
                ("rotating", "statics", "combustor", "turbine",
                 "augmentor", "nozzle", "accessories")]
        built, arrays = {}, {}
        for m in mods:
            built.update(m.build())
            arrays.update(getattr(m, "ARRAYS", {}))
        return built, arrays, espec
    finally:
        sys.path.remove(f110_dir)
        for k in list(sys.modules):
            if k in ("spec", "mesh", "airfoil", "materials") or k.startswith("parts"):
                del sys.modules[k]
        sys.modules.update(saved)


ARRAYS = {}


def build():
    built, arrays, espec = _load_engine()

    s = spec.ENGINE_SCALE
    # place the engine's own inlet-lip station on the firewall
    dx = spec.ENGINE_X - espec.STATION["inlet_lip"] * s
    dz = spec.ENGINE_Z

    out = {}
    ARRAYS.clear()
    for name, (verts, faces) in built.items():
        key = name if name.startswith("cut:") else f"engine_{name}"
        if name.startswith("cut:"):
            key = f"cut:engine_{name[4:]}"
        out[key] = ([(x * s + dx, y * s, z * s + dz) for (x, y, z) in verts],
                    faces)
    for name, count in arrays.items():
        ARRAYS[f"engine_{name}"] = count

    return out
