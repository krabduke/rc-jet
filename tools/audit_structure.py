"""Structural audit for the RC jet. See tools/_structure.py for the checks.

    python3 tools/audit_structure.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _structure as S

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CFG = {
    "gap_mm": 0.5,
    "exempt_attached": {},

    "mirror_tol_mm": 1.0,
    "exempt_mirror": {
        # ailerons are rigged with differential throw, so the two surfaces are
        # deliberately at different angles and cannot mirror
        "flaperon_": "differential aileron throw, by design",
    },

    "distinct_tol_mm": 0.5,
    "exempt_distinct": {
        # The turbofan's structural inlet case sits 10 mm inside its inlet
        # casing. Scaled 1:33 into this airframe that gap is 0.3 mm, under
        # the tolerance this test can meaningfully apply, so the two shells
        # land on the same box. They are genuinely different parts on the
        # full-size engine -- checked there, at full size, where they pass.
        "engine_inlet_case": "concentric with the inlet casing at 1:33",
    },
    "exempt_shape": {},

    # Things an aeroplane has exactly one of. It had two complete sets of
    # control pushrods -- `pushrods` in the fuselage module and
    # `pushrod_linkages` in the detail module, same span, same place -- and
    # two GPS aerials for one receiver.
    "singletons": {
        "control linkage set": (("pushrods", "pushrod_linkages"), 1),
        "GPS aerial": (("gps_puck", "gps_patch"), 1),
        "UHF aerial": (("antennas",), 1),
        "diversity aerials": (("antenna_a", "antenna_b"), 2),
        "receiver": (("receiver",), 1),
        "fuel tank": (("fuel_tank",), 1),
        "engine": (("engine_casing_fan",), 1),
        "fin": (("vtail_fin",), 1),
        "rudder": (("rudder",), 1),
        "canopy": (("canopy_glass",), 1),
        "nose gear leg": (("gear_nose_strut",), 1),
        "pitot": (("pitot",), 1),
    },

}

if __name__ == "__main__":
    n = S.report(os.path.join(ROOT, "build", "parts.csv"), CFG, "RC jet")
    sys.exit(0 if n == 0 else 1)
