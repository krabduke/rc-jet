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

# Things that live inside the fuselage and must stay there. Wing and tail
# parts are checked against their own surfaces elsewhere; the engine is
# checked by verify.py.
INSIDE = ("lipo", "esc_", "receiver", "servo", "wiring", "turbine_ecu",
          "ecu_battery", "kill_switch", "data_link", "avionics_tray",
          "fuel_tank", "fuel_hopper", "fuel_pump", "fuel_filter", "fuel_lines",
          "former_", "longeron_", "stringer_", "bhd_", "retract_",
          "gps_puck", "telemetry_sensor", "mount_ring", "mount_rails",
          "gear_door_actuator")

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
        if not (name.startswith(INSIDE) or allow_canopy):
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
