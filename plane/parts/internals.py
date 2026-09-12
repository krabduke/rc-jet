"""RC hardware and wiring -- the masses the CG solve is built on."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh


def build():
    out = {}
    for (name, x, y, z, l, w, h, _mass) in spec.HARDWARE:
        out[name] = mesh.box(x, y, z, l, w, h)
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
                           (esc[1], 0.0, esc[3] - 4)], 1.5, 8))
    # ESC to the engine
    runs.append(mesh.pipe([(esc[1] + 20, -4.0, esc[3] - 3),
                           (280.0, -6.0, 4.0),
                           (spec.ENGINE_X + 6, -4.0, 2.0)], 1.4, 8))
    # receiver out to each servo
    rx = [h for h in spec.HARDWARE if h[0] == "receiver"][0]
    for h in spec.HARDWARE:
        if not h[0].startswith("servo_"):
            continue
        runs.append(mesh.pipe([(rx[1], rx[2], rx[3]),
                               (rx[1] + 60, h[2] * 0.4, 6.0),
                               (h[1] - 8, h[2], h[3] + 8)], 0.8, 6))
    return {"wiring": mesh.join(*runs), "pushrods": _pushrods()}


def _pushrods():
    """Carbon pushrods from the servo bay back to each control surface. The
    servos sit ahead of the firewall because the engine fills the tail, so the
    rods are long -- which is the real consequence of that packaging choice."""
    W, H, V = spec.WING, spec.HTAIL, spec.VTAIL
    rods = []
    servos = {h[0]: h for h in spec.HARDWARE}

    for tag, sgn in (("servo_ail_l", -1.0), ("servo_ail_r", 1.0)):
        s = servos[tag]
        f = (spec.FLAPERON["span_in"] + spec.FLAPERON["span_out"]) / 2
        y = sgn * W["semi_span"] * f
        chord = W["root_chord"] + (W["tip_chord"] - W["root_chord"]) * f
        x_le = W["x_root_le"] + abs(y) * math.tan(math.radians(W["sweep_le"]))
        x_h = x_le + chord * (1.0 - spec.FLAPERON["chord_frac"])
        rods.append(mesh.pipe([(s[1] + 12, s[2], s[3] + 8),
                               (x_h - 26, y * 0.62, W["z_root"] + 1.0),
                               (x_h - 3, y, W["z_root"] + 1.0)], 0.7, 6))

    s = servos["servo_stab"]
    x_h = H["x_root_le"] + H["root_chord"] * 0.25
    rods.append(mesh.pipe([(s[1] + 12, s[2], s[3]),
                           (340.0, -12.0, 6.0),
                           (x_h, -9.0, H["z_root"] + 3.0)], 0.7, 6))

    s = servos["servo_rudder"]
    x_r = V["x_root_le"] + V["root_chord"] * (1.0 - V["rudder_chord"])
    rods.append(mesh.pipe([(s[1] + 12, s[2], s[3]),
                           (350.0, 9.0, 14.0),
                           (x_r - 4, 3.0, 22.0)], 0.7, 6))
    return mesh.join(*rods)
