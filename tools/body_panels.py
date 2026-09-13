"""Panelise the solid parts of the aircraft for the flow solve.

The vortex lattice makes the lift; it is invisible to the air everywhere else,
so streamlines traced from it alone pass straight through the fuselage. Source
panels on the solid body fix that -- see the car project's copy of this file
for the method and why each piece of it is there.

Panels come from the same superellipse station table the fuselage is lofted
from, so the shape the air sees is the shape on screen.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "plane"))

import spec
from parts import fuselage as fus

MM = 0.001


def _quad(a, b, c, d):
    cx = (a[0] + b[0] + c[0] + d[0]) / 4.0
    cy = (a[1] + b[1] + c[1] + d[1]) / 4.0
    cz = (a[2] + b[2] + c[2] + d[2]) / 4.0
    ux, uy, uz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    vx, vy, vz = d[0] - b[0], d[1] - b[1], d[2] - b[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    if mag < 1e-12:
        return None
    return ([cx * MM, cy * MM, cz * MM], [nx / mag, ny / mag, nz / mag],
            0.5 * mag * MM * MM)


def _outward(panels, axis):
    out = []
    for (c, n, a) in panels:
        d = (c[0] - axis[0], c[1] - axis[1], c[2] - axis[2])
        if n[0] * d[0] + n[1] * d[1] + n[2] * d[2] < 0:
            n = [-n[0], -n[1], -n[2]]
        out.append((c, n, a))
    return out


def _cap(ring, centre, nrm):
    out = []
    n = len(ring)
    for j in range(n):
        q = _quad(ring[j], ring[(j + 1) % n], centre, centre)
        if q:
            out.append((q[0], list(nrm), q[2]))
    return out


def fuselage(n_x=20, n_theta=16):
    """The shelled body, plus the canopy bulge it carries."""
    x0, x1 = spec.FUSELAGE[0][0], spec.FUSELAGE[-1][0]
    rings = []
    for i in range(n_x):
        f = 0.5 * (1 - math.cos(math.pi * i / (n_x - 1)))
        x = x0 + (x1 - x0) * f
        rings.append(fus.section_ring(x, segments=n_theta))
    out = []
    for i in range(len(rings) - 1):
        xm = (rings[i][0][0] + rings[i + 1][0][0]) / 2
        w, h, zc, nn = fus.station_at(xm)
        out.extend(_outward(_loft(rings[i], rings[i + 1]),
                            (xm * MM, 0.0, zc * MM)))
    for (ring, nrm) in ((rings[0], (-1.0, 0.0, 0.0)),
                        (rings[-1], (1.0, 0.0, 0.0))):
        x = ring[0][0]
        w, h, zc, nn = fus.station_at(x)
        out.extend(_cap(ring, (x, 0.0, zc), nrm))
    return out


def _loft(r0, r1):
    out = []
    n = len(r0)
    for j in range(n):
        q = _quad(r0[j], r0[(j + 1) % n], r1[(j + 1) % n], r1[j])
        if q:
            out.append(q)
    return out


def build():
    panels = fuselage()
    return {
        "n": len(panels),
        "c": [v for (c, n, a) in panels for v in c],
        "n_": [v for (c, n, a) in panels for v in n],
        "a": [a for (c, n, a) in panels],
    }


if __name__ == "__main__":
    b = build()
    print(f"{b['n']} body panels, wetted area {sum(b['a']):.3f} m2")
