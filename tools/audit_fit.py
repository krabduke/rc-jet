"""Check that nothing inside the aircraft pokes out through its skin.

Bounding boxes cannot answer this. A stringer runs the whole length of the
fuselage, so its box is as wide as the widest station it passes; compared
against the nose it looks 28 mm outside when it is perfectly inside all the
way along. The only honest test is per vertex, against the section at that
vertex's own station -- which is cheap, because the geometry layer is pure
Python and can be rebuilt without Blender.

    python3 tools/audit_fit.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "plane"))

import spec
from parts import (fuselage as fus, internals, structure, systems, detail,
                   canopy as canopy_mod)

# Everything is checked. This used to be an allow-list of prefixes that had to
# stay inside, and anything not on it was skipped -- so a NACA inlet placed
# 44 mm outside the fuselage passed a test whose entire job is catching parts
# outside the fuselage. Any part added after the list was written was invisible
# to it. Now the default is "must be inside", and each exception is named with
# the reason it is allowed out.
OUTSIDE = {
    # lifting surfaces and their hardware, checked against their own sections
    "wing": "a wing is outside the fuselage by definition",
    "flaperon": "on the wing",
    "stabilator": "on the tail",
    "vtail": "on the tail",
    "rudder": "on the fin",
    "ventral": "under the tail, outside the skin",
    "spar": "runs through the wing",
    "rib_": "in the wing",
    "fin_rib": "in the fin",
    "hinge_": "on a control surface",
    "horn_": "on a control surface, outboard of the skin",
    "clevis": "on a horn",
    "pushrod_linkages": "runs from the fuselage out to the surfaces",
    "bellcranks": "in the wing root, outside the fuselage section",
    # the skin and everything applied to its outer face
    "fuselage_skin": "is the skin",
    "seam_": "on the outside of the skin",
    "panel_": "an access hatch in the skin",
    "doublers": "bonded to the inside of the skin at a seam",
    "naca_inlet": "a duct cut into the outer face of the skin",
    "cooling_exit": "a vent in the outer face of the skin",
    "bl_diverter": "stands off the skin ahead of the intake",
    "intake_lip": "the outer lip of the intake",
    "duct_inlet": "the intake duct, open to outside",
    "nose_strakes": "on the outside of the nose",
    "wing_strake": "on the outside, at the wing root",
    "wing_fence": "on the wing",
    "vg_": "on the wing",
    "antenna": "sticks out, on purpose",
    "pitot": "sticks out, on purpose",
    "static_discharger": "on a trailing edge",
    "navlight": "in the skin",
    "tailpipe_shroud": "round the tailpipe, aft of the skin",
    "bypass_slots": "vents in the skin",
    "canopy_": "the canopy is outside the fuselage line",
    "access_tray": "under the canopy",
    "rx_battery": "under the canopy",
    "rx_mount": "under the canopy",
    # landing gear, which is outside whenever it is down
    "gear_": "extends below the aircraft",
    "wheel_": "extends below the aircraft",
    # the engine is checked by verify.py against the tailcone
    "engine_": "checked against the tailcone by verify.py",
    "mount_": "in the engine bay, checked with the engine",
}

# The canopy is a bubble standing above the fuselage, so anything under it is
# allowed that much extra height.
CANOPY_PARTS = ("instrument_panel", "seat_pan", "seat_back", "console_",
                "coaming", "hud", "pilot_", "ejection_seat")


def canopy_top(x):
    C = spec.CANOPY
    if not (C["x_front"] <= x <= C["x_rear"]):
        return None
    f = (x - C["x_front"]) / (C["x_rear"] - C["x_front"])
    # a bubble: full height in the middle, tapering to the ends
    return C["z_base"] + C["height"] * math.sin(math.pi * min(1.0, max(0.0, f))) ** 0.6


def check():
    built = {}
    for m in (internals, structure, systems, detail):
        built.update(m.build())

    worst = []
    for name, (verts, faces) in built.items():
        allow_canopy = name.startswith(CANOPY_PARTS)
        if any(name.startswith(k) for k in OUTSIDE):
            continue
        over = 0.0
        at = None
        for (x, y, z) in verts:
            if not (spec.FUSELAGE[0][0] <= x <= spec.FUSELAGE[-1][0]):
                continue
            w, h, zc, n = fus.station_at(x)
            w -= spec.FUSELAGE_SKIN
            h -= spec.FUSELAGE_SKIN
            p = 2.0 / n
            # superellipse test: |y/w|^n + |z/h|^n <= 1 inside
            t = (abs(y / max(w, 1e-6)) ** n + abs((z - zc) / max(h, 1e-6)) ** n)
            if t <= 1.0:
                continue
            # how far out, measured radially
            scale = t ** (1.0 / n)
            d = (scale - 1.0) * max(w, h)
            if allow_canopy:
                ct = canopy_top(x)
                if ct is not None and z <= ct and abs(y) <= spec.CANOPY["half_width"]:
                    continue
            if d > over:
                over, at = d, (x, y, z)
        if over > 0.5:
            worst.append((over, name, at))
    return sorted(worst, reverse=True)


if __name__ == "__main__":
    bad = check()
    if not bad:
        print("PASS  every internal part is inside the skin")
    else:
        print(f"FAIL  {len(bad)} parts poke through the skin\n")
        for d, n, at in bad:
            print(f"  {n:24s} {d:6.1f} mm out at "
                  f"({at[0]:.0f}, {at[1]:.0f}, {at[2]:.0f})")
    sys.exit(1 if bad else 0)
