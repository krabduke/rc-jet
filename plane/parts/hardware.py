"""The engine's exhaust, and the hardware the airframe still needs.

The F110's own nozzle is built by the vendored engine at maximum augmented
power -- throat and exit both open. That is the wrong drawing for everything
but max reheat, and it is not the nozzle this airframe is seen with: the
aircraft sits on the ground and cruises dry, and what a nozzle looks like at
dry power is closed down to a convergent cone with a ring of external flaps
over it. So the nozzle that ships with the aeroplane is the engine's own
hardware re-posed: its installed hinge and exit geometry locate the flaps,
and the dry-power throat closes down inside the external petal envelope.

The old `thrust_tube` and `tailpipe_cone` were a model's jet pipe -- a plain
tube necking down to a cone on a spider, the exhaust of a ducted-fan airframe
rather than the exhaust of the turbofan installed behind it.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


import spec
import mesh
from parts import engine_mount


# these dry-power display proportions belong in spec.py; 1090 mm is the
# reference open exit diameter, not the smaller dry-power throat diameter.
NOZZLE_EXIT_DIAMETER = 1090.0 / spec.SCALE_TO_FULL
DRY_THROAT_RATIO = 0.65
DRY_EXIT_RATIO = 0.72
PETAL_OVERLAP = 1.04
NOZZLE_PETALS = 24

# hardware owns these installed meshes, otherwise the open engine nozzle
# remains superimposed on the dry-power one during assembly.
_NOZZLE_PARTS = ("nozzle_flaps_convergent", "nozzle_flaps_divergent",
                 "nozzle_seals", "nozzle_ext_flaps", "nozzle_actuator_ring",
                 "nozzle_actuators", "nozzle_links", "flameholder", "spraybars")
engine_mount.NOT_INSTALLED = tuple(dict.fromkeys(
    engine_mount.NOT_INSTALLED + _NOZZLE_PARTS))


def _nozzle_panel(x0, x1, r0, r1, z, angle, width, thickness,
                  n_arc=12, n_x=8):
    """Curved overlapping sectors keep the throat open without polygonal gaps.

    Six arc segments and two axial stations gave every flap 28 vertices, on a
    panel 570 mm long that is curved in both directions -- so it read as one
    flat facet, twelve times round, on the part of the aeroplane a viewer
    looks at first.

    The flap also bows. A convergent flap is not a straight cone: it is a
    shallow arc, which is what lets twelve of them close on the throat
    without binding on each other at the edges.
    """
    def radius(u):
        return (r0 + (r1 - r0) * u
                + abs(r1 - r0) * 0.06 * math.sin(math.pi * u))

    rows = []
    for i in range(n_x + 1):
        u = i / n_x
        x = x0 + (x1 - x0) * u
        r = radius(u)
        row = []
        for j in range(n_arc + 1):
            a = angle - width / 2 + width * j / n_arc
            sa, ca = math.sin(a), math.cos(a)
            row.append(((x, r * sa, z + r * ca),
                        (x, (r - thickness) * sa, z + (r - thickness) * ca)))
        rows.append(row)

    verts = [v for row in rows for pair in row for v in pair]
    nj = n_arc + 1

    def idx(i, j, inner):
        return (i * nj + j) * 2 + (1 if inner else 0)

    faces = []
    for i in range(n_x):
        for j in range(n_arc):
            faces.append((idx(i, j, False), idx(i, j + 1, False),
                          idx(i + 1, j + 1, False), idx(i + 1, j, False)))
            faces.append((idx(i, j, True), idx(i + 1, j, True),
                          idx(i + 1, j + 1, True), idx(i, j + 1, True)))
    for j in range(n_arc):                       # fore and aft edges
        faces.append((idx(0, j, False), idx(0, j, True),
                      idx(0, j + 1, True), idx(0, j + 1, False)))
        faces.append((idx(n_x, j, False), idx(n_x, j + 1, False),
                      idx(n_x, j + 1, True), idx(n_x, j, True)))
    for i in range(n_x):                          # the two side edges
        faces.append((idx(i, 0, False), idx(i + 1, 0, False),
                      idx(i + 1, 0, True), idx(i, 0, True)))
        faces.append((idx(i, n_arc, False), idx(i, n_arc, True),
                      idx(i + 1, n_arc, True), idx(i + 1, n_arc, False)))
    return verts, faces


def _nozzle():
    scale = spec.ENGINE_SCALE
    x = spec.ENGINE_X
    z = spec.ENGINE_Z
    out = {}
    count = 12
    pitch = 2 * math.pi / count
    for i in range(count):
        angle = i * pitch
        out["nozzle_flap_%02d" % i] = _nozzle_panel(
            x + 4060 * scale, x + 4630 * scale,
            428 * scale, 280 * scale, z, angle, pitch * 0.96,
            8 * scale)
        out["nozzle_seal_%02d" % i] = _nozzle_panel(
            x + 4060 * scale, x + 4630 * scale,
            431 * scale, 283 * scale, z, angle + pitch / 2,
            pitch * 0.20, 6 * scale)
        out["nozzle_external_flap_%02d" % i] = _nozzle_panel(
            x + 4050 * scale, x + 4620 * scale,
            452 * scale, 298 * scale, z, angle + pitch / 2,
            pitch - math.radians(2.4), 9 * scale)
        out["nozzle_link_%02d" % i] = mesh.pipe(
            [(x + 4014 * scale, 438 * scale * math.sin(angle),
              z + 438 * scale * math.cos(angle)),
             (x + 4156 * scale, 402 * scale * math.sin(angle),
              z + 402 * scale * math.cos(angle))],
            # 24 segments, not 10: at 10 this was a 40-vertex tube and the
            # geometry audit's floor is 180. There are twelve of them and
            # they are the linkage you can actually see working.
            14 * scale, segments=24, subdiv=3)
    verts, faces = mesh.ring_torus(x + 4010 * scale, 452 * scale,
                                    20 * scale, 64, 14)
    out["nozzle_unison_ring"] = (mesh.translate(verts, dz=z), faces)
    for i in range(6):
        angle = i * 2 * math.pi / 6
        y = 478 * scale * math.sin(angle)
        az = z + 478 * scale * math.cos(angle)
        out["nozzle_actuator_%02d" % i] = mesh.join(
            mesh.pipe([(x + 3770 * scale, y, az),
                       (x + 3928.4 * scale, y, az)],
                      34 * scale, segments=18),
            mesh.pipe([(x + 3914 * scale, y, az),
                       (x + 4010 * scale, y, az)],
                      14.28 * scale, segments=14))
    return out


def build():
    out = {}
    out.update(_nozzle())
    return out
