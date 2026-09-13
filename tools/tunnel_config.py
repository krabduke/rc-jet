"""Emit the wind-tunnel configuration the browser solver runs on.

The lattice the viewer solves is built from the same spec.py the geometry and
aero/analyse.py are built from, so the number on the web page and the number
from `make aero` describe the same aircraft. The browser uses a coarser
lattice -- it has to re-solve while a slider moves -- and tools/validate_js.py
checks what that costs.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "plane"))

import spec

MM = 0.001


# Lattice resolution is pinned, not chosen for speed.
#
# A swept low aspect ratio delta with a tailplane close behind it is a hard
# case for a lattice: at some spanwise panel counts a collocation point lands
# almost on a neighbouring bound vortex and the answer jumps. Sweeping the
# resolution against the validated Python solver (lift slope 3.533 /rad,
# neutral point 37.6 % MAC) shows n_span = 10 reproducing it to three figures,
# and n_span = 10 with two different chordwise counts agreeing with each other
# -- which is what convergence actually looks like. tools/validate_js.mjs
# asserts it, so changing these numbers fails the check rather than silently
# moving the aeroplane.
WING_NS, WING_NC = 10, 5
TAIL_NS, TAIL_NC = 6, 4


def config(n_span=WING_NS, n_chord=WING_NC):
    W, H, V, FL = spec.WING, spec.HTAIL, spec.VTAIL, spec.FLAPERON
    sweep = math.tan(math.radians(W["sweep_le"]))
    hsweep = math.tan(math.radians(H["sweep_le"]))
    hrise = math.tan(math.radians(H["anhedral"]))

    # One continuous wing. The flaperon is a band of strips inside it, not a
    # separate surface: two surfaces meeting at the flaperon's inboard end
    # shed coincident trailing filaments of different strength and the solve
    # blows up as soon as the control moves.
    surfaces = [
        {"name": "wing",
         "le_root": [W["x_root_le"] * MM, 0.0, W["z_root"] * MM],
         "c_root": W["root_chord"] * MM,
         "le_tip": [(W["x_root_le"] + W["semi_span"] * sweep) * MM,
                    W["semi_span"] * MM, W["z_root"] * MM],
         "c_tip": W["tip_chord"] * MM,
         "n_span": n_span, "n_chord": n_chord,
         "twist_root": W["incidence"],
         "twist_tip": W["incidence"] - W["washout"],
         "control": "flaperon", "control_chord": FL["chord_frac"],
         "control_span": [FL["span_in"], FL["span_out"]]},
        {"name": "stabilator",
         "le_root": [H["x_root_le"] * MM, 0.0, H["z_root"] * MM],
         "c_root": H["root_chord"] * MM,
         "le_tip": [(H["x_root_le"] + H["semi_span"] * hsweep) * MM,
                    H["semi_span"] * MM,
                    (H["z_root"] + H["semi_span"] * hrise) * MM],
         "c_tip": H["tip_chord"] * MM,
         "n_span": TAIL_NS, "n_chord": TAIL_NC,
         "twist_root": 0.0, "twist_tip": 0.0,
         # an all-moving surface is not a flap: the whole thing turns, so its
         # effectiveness is 1.0, not the thin-aerofoil flap factor
         "control": "stabilator", "control_tau": 1.0},
    ]

    return {
        "units": "m",
        "s_ref": spec.wing_area_mm2() * MM * MM,
        "c_ref": spec.mean_aero_chord() * MM,
        "b_ref": spec.SPAN * MM,
        "x_le_mac": spec.mac_leading_edge_x() * MM,
        "mass_kg": spec.total_mass_g() / 1000.0,
        "v_default": 22.0, "v_min": 8.0, "v_max": 45.0,
        "alpha_default": 4.0, "alpha_min": -6.0, "alpha_max": 16.0,
        "stall_alpha": 12.0,
        "ground": False,
        "cg_frac": spec.cg_frac_mac(),
        "cg_x": spec.cg_x() * MM,
        # `baked` is the deflection the geometry was BUILT at -- the model is
        # exported showing a control position, not a neutral one. The viewer
        # rotates each surface by (slider - baked) so that slider and mesh
        # agree; without it, sitting the slider at zero would still show the
        # surface at its modelled angle while the solver assumed neutral.
        "controls": [
            {"id": "flaperon", "label": "Flaperons", "unit": "deg",
             "min": -25.0, "max": 25.0, "value": 0.0,
             "baked": spec.FLAPERON["deflect"],
             "objects": ["flaperon_l", "flaperon_r"], "sign": [1.0, 1.0]},
            {"id": "stabilator", "label": "Stabilators", "unit": "deg",
             "min": -20.0, "max": 20.0, "value": 0.0,
             "baked": spec.HTAIL["deflect"],
             "objects": ["stabilator_l", "stabilator_r"], "sign": [1.0, 1.0]},
            {"id": "rudder", "label": "Rudder", "unit": "deg",
             "min": -25.0, "max": 25.0, "value": 0.0,
             "baked": spec.VTAIL["deflect"],
             "objects": ["rudder"], "sign": [1.0], "yaw_only": True},
        ],
        "surfaces": surfaces,
    }


if __name__ == "__main__":
    import json
    print(json.dumps(config(), indent=1))
