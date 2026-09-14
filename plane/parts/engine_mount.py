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
        # Coarsened, but not by as much as it was: at 32 segments a casing
        # came out as a 128-vertex prism, which is a worse surface than
        # anything else on the airframe and the engine is the thing people
        # open the cutaway to look at. Two thirds of full resolution.
        espec.RES.update({
            "airfoil_chord_pts": 20,
            "airfoil_span_pts": 6,
            "revolve_segments": 64,
            "small_revolve": 16,
            "pipe_segments": 12,
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

# Parts of the full-size engine that this aircraft does not install.
#
# These four are airframe-mounted line-replaceable units: on a real
# installation the oil tank, the fuel/oil heat exchanger, the engine control
# and the ignition exciters live in the nacelle, outboard of the engine's own
# envelope, not on the engine as it leaves the stand. Scaled 1:33 into a
# 440 mm fuselage they reach 32-38 mm off the axis where the fuselage inner
# half-width is under 25 mm, so they come straight out through the skin.
#
# The aircraft has its own versions of all four at its own scale -- fuel_tank,
# fuel_pump, turbine_ecu, ecu_battery -- and fitting a scaled F110 oil tank
# beside a model fuel hopper would be the wrong object twice. So the airframe
# takes the engine and leaves the nacelle kit behind.
NOT_INSTALLED = ("oil_tank", "heat_exchanger", "engine_control",
                 "ignition_exciters")


def build():
    built, arrays, espec = _load_engine()

    s = spec.ENGINE_SCALE
    # place the engine's own inlet-lip station on the firewall
    dx = spec.ENGINE_X - espec.STATION["inlet_lip"] * s
    dz = spec.ENGINE_Z

    out = {}
    ARRAYS.clear()
    for name, (verts, faces) in built.items():
        # str.lstrip strips characters, not a prefix, so do it by hand:
        # "turbine_cooling_manifold".lstrip("cut:") is "rbine_..."
        bare = name[4:] if name.startswith("cut:") else name
        if bare in NOT_INSTALLED:
            continue
        key = name if name.startswith("cut:") else f"engine_{name}"
        if name.startswith("cut:"):
            key = f"cut:engine_{name[4:]}"
        out[key] = ([(x * s + dx, y * s, z * s + dz) for (x, y, z) in verts],
                    faces)
    for name, count in arrays.items():
        ARRAYS[f"engine_{name}"] = count

    return out


# Which of the engine's parts turn with which spool. The fan and the low
# turbine share the LP shaft; the compressor and the high turbine share the
# HP shaft, which turns faster and the other way. Everything else is static.
LP_PARTS = ("blades_fan_", "fan_disc_assembly", "spinner", "shaft_lp",
            "lpt_disc_assembly", "blades_lpt_r")
HP_PARTS = ("blades_hpc_", "hpc_drum", "hpc_front_cone", "hpc_rear_cone",
            "shaft_hp", "hpt_disc", "blades_hpt_r")


def pivots():
    """The rotating assemblies, about the engine's own centreline.

    The engine axis is x, at y = 0 and the thrust line in z. Giving each rotor
    that pivot means the viewer can spool the engine up without the parts
    orbiting the middle of the aeroplane.
    """
    built, _, espec = _load_engine()
    s = spec.ENGINE_SCALE
    dx = spec.ENGINE_X - espec.STATION["inlet_lip"] * s
    out = {}
    for name in built:
        if name.startswith("cut:"):
            continue
        lp = any(name.startswith(p) for p in LP_PARTS)
        hp = any(name.startswith(p) for p in HP_PARTS)
        if not (lp or hp):
            continue
        # pivot on the axis at the part's own mid-station, so the origin sits
        # inside the part rather than out at the inlet
        xs = [v[0] for v in built[name][0]]
        xm = (min(xs) + max(xs)) / 2 * s + dx
        out[f"engine_{name}"] = ((xm, 0.0, spec.ENGINE_Z), (1.0, 0.0, 0.0),
                                 1.0 if lp else -1.6, "spool_lp" if lp else "spool_hp")
    return out
