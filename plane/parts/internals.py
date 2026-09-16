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


# The one lane through this fuselage that is clear of everything: outboard of
# the avionics tray at y 18.6, above the saddle tanks that top out at z 14.2,
# and outside the intake duct, which is 41 mm across the middle of the body.
LOOM_Y, LOOM_Z = 20.0, 16.5


def _wiring():
    """Servo leads and the motor phase wires, run along the inside of the
    fuselage past the bulkhead lightening holes."""
    runs = []
    lipo = spec.equipment("lipo_3s_900")
    ecu = spec.equipment("turbine_ecu")
    eb = spec.equipment("ecu_battery")

    # flight pack back to the ECU, which is what runs the turbine
    runs.append(mesh.pipe([(lipo[0] + 24, 6.0, lipo[2] + 8),
                           (ecu[0] - 18, 4.0, ecu[2] + 4),
                           (ecu[0] - 8, 0.0, ecu[2] + 3)], 1.5, 16, subdiv=3))
    # ECU battery forward to the ECU, on its own circuit: losing the flight
    # pack should not stop the fuel pump mid-flameout
    runs.append(mesh.pipe([(eb[0] - 14, 5.0, eb[2] - 3),
                           (ecu[0] + 26, 3.0, ecu[2] + 4),
                           (ecu[0] + 14, 0.0, ecu[2] + 4)], 1.4, 16, subdiv=3))
    # ECU aft to the engine's starter and its sensors
    # ECU aft to the engine's starter and its sensors, down the same lane and
    # then down the engine's side. The engine's inlet is on the duct's axis,
    # so a wire that dives at it crosses the intake in its last 20 mm -- and
    # then the fan.
    runs.append(mesh.pipe([(ecu[0] + 16, -4.0, ecu[2] - 2),
                           (ecu[0] + 40, -LOOM_Y * 0.7, LOOM_Z),
                           (295.0, -LOOM_Y, LOOM_Z),
                           (spec.ENGINE_X + 12, -21.0, 6.0)], 1.4, 16,
                          subdiv=3))
    # receiver out to each servo
    rx = spec.equipment("receiver")
    for h in spec.HARDWARE:
        if not h[0].startswith("servo_"):
            continue
        # Along the loom line and then out to the servo.
        #
        # LOOM_Y / LOOM_Z is the one lane through this fuselage that is clear
        # of everything: outboard of the avionics tray, above the saddle tanks
        # and outside the intake duct. A run straight from the receiver to a
        # servo goes through all three.
        sgn = 1.0 if h[2] >= 0 else -1.0
        runs.append(mesh.pipe([(rx[0], rx[1], rx[2]),
                               (rx[0] + 14, sgn * LOOM_Y * 0.6, LOOM_Z),
                               (h[1] - 26, sgn * LOOM_Y, LOOM_Z),
                               # out along the lane and then straight DOWN on
                               # to the servo. Approaching from ahead of it
                               # runs the lead along the servo in front's own
                               # pushrod, which is where both live in a wing
                               # root that carries two of them.
                               (h[1], h[2], LOOM_Z),
                               (h[1], h[2], h[3] + 8)], 0.8, 16, subdiv=3))
    return {"wiring": mesh.join(*runs)}


# The pushrods used to be built here as well as in
# detail.py -- two complete sets of control linkages, same span, same
# place, built by two modules that had never heard of each other. The
# surviving set lives with the horns it connects to, in detail.py.
