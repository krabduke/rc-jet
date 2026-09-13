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


# Lattice resolution is pinned, not chosen for speed, and it matches the
# resolution aero/analyse.py runs at.
#
# This wing is AR 1.81 and a lattice is sensitive at that aspect ratio: a
# resolution sweep gives a lift slope within 4 % of lifting-line theory at
# 8x4, 10x5 and 16x6, but a neutral point that only agrees between 8x4
# (32.5 %) and 16x6 (33.0 %) -- and 20x8 returns 6.9 per radian, which is
# nonsense. Pinned where two independent resolutions agree, and matched to
# the Python solver so the page and `make aero` describe one aeroplane.
# tools/validate_js.mjs asserts it.
WING_NS, WING_NC = 16, 6
TAIL_NS, TAIL_NC = 8, 5
FIN_NS, FIN_NC = 6, 4


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
         "planform": [[f, x * MM, c * MM] for (f, x, c) in spec.WING_PLANFORM],
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
        # The vertical fin. Its span runs in z, so it needs axis "z" and no
        # mirroring -- there is only one of it. Without the fin in the lattice
        # the rudder slider moved the geometry and changed nothing at all in
        # the solve, and sideslip had nothing to act on.
        {"name": "fin",
         "le_root": [V["x_root_le"] * MM, 0.0, V["z_root"] * MM],
         "c_root": V["root_chord"] * MM,
         "le_tip": [(V["x_root_le"] + V["height"]
                     * math.tan(math.radians(V["sweep_le"]))) * MM,
                    0.0, (V["z_root"] + V["height"]) * MM],
         "c_tip": V["tip_chord"] * MM,
         "axis": "z", "mirror": False,
         "n_span": FIN_NS, "n_chord": FIN_NC,
         "twist_root": 0.0, "twist_tip": 0.0,
         "control": "rudder", "control_chord": V["rudder_chord"],
         "control_span": [0.0, V["rudder_span"]]},
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
        "beta_default": 0.0, "beta_min": -15.0, "beta_max": 15.0,
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
            # +/- 12 degrees, which is real flaperon travel and also the
            # range over which this solver's answer stays monotonic. At AR
            # 1.81 a lattice is near its limit: past about 15 degrees the
            # lift it predicts starts falling as flap is added, which is a
            # property of the method rather than of the aeroplane.
            {"id": "flaperon", "label": "Flaperons", "unit": "deg",
             "min": -12.0, "max": 12.0, "value": 0.0,
             "baked": spec.FLAPERON["deflect"],
             "objects": ["flaperon_l", "flaperon_r"], "sign": [1.0, 1.0]},
            {"id": "stabilator", "label": "Stabilators", "unit": "deg",
             "min": -12.0, "max": 12.0, "value": 0.0,
             "baked": spec.HTAIL["deflect"],
             "objects": ["stabilator_l", "stabilator_r"], "sign": [1.0, 1.0]},
            {"id": "rudder", "label": "Rudder", "unit": "deg",
             "min": -25.0, "max": 25.0, "value": 0.0,
             "baked": spec.VTAIL["deflect"],
             "objects": ["rudder"], "sign": [1.0]},
        ],
        "surfaces": surfaces,
    }


if __name__ == "__main__":
    import json
    print(json.dumps(config(), indent=1))
