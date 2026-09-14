"""RC hardware and wiring -- the masses the CG solve is built on."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes


def build():
    out = {}
    for (name, x, y, z, l, w, h, _mass) in spec.HARDWARE:
        out[name] = shapes.rounded_box(x, y, z, l, w, h)
    out.update(_wiring())
    return out


def _wiring():
    """Servo leads and the motor phase wires, run along the inside of the
    fuselage past the bulkhead lightening holes."""
    runs = []
    esc = [h for h in spec.HARDWARE if h[0] == "esc_40a"][0]
    lipo = [h for h in spec.HARDWARE if h[0] == "lipo_3s_1300"][0]

    # battery to ESC
    runs.append(mesh.pipe([(lipo[1] - 20, 6.0, lipo[3] + 11),
                           (esc[1] + 10, 4.0, esc[3] - 4),
                           (esc[1], 0.0, esc[3] - 4)], 1.5, 16, subdiv=3))
    # ESC to the engine
    runs.append(mesh.pipe([(esc[1] + 20, -4.0, esc[3] - 3),
                           (280.0, -6.0, 4.0),
                           (spec.ENGINE_X + 6, -4.0, 2.0)], 1.4, 16, subdiv=3))
    # receiver out to each servo
    rx = [h for h in spec.HARDWARE if h[0] == "receiver"][0]
    for h in spec.HARDWARE:
        if not h[0].startswith("servo_"):
            continue
        runs.append(mesh.pipe([(rx[1], rx[2], rx[3]),
                               (rx[1] + 60, h[2] * 0.4, 6.0),
                               (h[1] - 8, h[2], h[3] + 8)], 0.8, 16, subdiv=3))
    return {"wiring": mesh.join(*runs)}


# The pushrods used to be built here as well as in
# detail.py -- two complete sets of control linkages, same span, same
# place, built by two modules that had never heard of each other. The
# surviving set lives with the horns it connects to, in detail.py.
