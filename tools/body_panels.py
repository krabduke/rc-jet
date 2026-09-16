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
    return [q for q in out if not _in_inlet(q)]


# The intake mouth is a hole, so the panels have to stop at its edge.
#
# The body was panelised as a closed shell, which meant the solver saw a
# solid nose: the boundary condition said no flow crosses the skin, including
# across the inlet. The air could not get in however hard the engine pulled,
# and the intake sink in the viewer was fighting a wall. Panels whose centre
# falls in the mouth are dropped, which leaves the aperture open and lets the
# capture streamtube form.
_LIP = spec.INTAKE_LIP if hasattr(spec, "INTAKE_LIP") else None


def _in_inlet(q):
    c = q[0]
    x, y, z = c[0] / MM, c[1] / MM, c[2] / MM
    # the lip sits low on the nose, around x 52-60
    if not (44.0 <= x <= 68.0):
        return False
    return (y * y) / (23.0 ** 2) + ((z + 17.0) ** 2) / (13.5 ** 2) <= 1.0


def _loft(r0, r1):
    out = []
    n = len(r0)
    for j in range(n):
        q = _quad(r0[j], r0[(j + 1) % n], r1[(j + 1) % n], r1[j])
        if q:
            out.append(q)
    return out


def from_mesh(verts, faces):
    """Source panels straight off a built surface.

    Any closed part can be a flow obstacle; it only has to hand over a
    centroid, an outward normal and an area per face. This is how the
    lifting surfaces get into the solve without re-deriving their geometry
    from a second set of numbers.
    """
    out = []
    for f in faces:
        pts = [verts[i] for i in f]
        for k in range(1, len(pts) - 1):
            a, b, c = pts[0], pts[k], pts[k + 1]
            cx = (a[0] + b[0] + c[0]) / 3.0
            cy = (a[1] + b[1] + c[1]) / 3.0
            cz = (a[2] + b[2] + c[2]) / 3.0
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            m = math.sqrt(nx * nx + ny * ny + nz * nz)
            if m < 1e-12:
                continue
            out.append(((cx * MM, cy * MM, cz * MM),
                        [nx / m, ny / m, nz / m],
                        0.5 * m * MM * MM))
    return out


# How far along the chord the obstacle stops.
#
# Every aerofoil section closes to a point at its trailing edge, so the last
# chordwise panel is a fraction of a millimetre tall with its opposite number
# the same distance away, facing the other way. That pair is near-singular in
# a source solve whatever size it is, and the strengths run away: the wingtip
# carried 1,628 against a 22 m/s freestream and the streamline that passed
# 2 mm from it read 295 m/s -- the fastest thing in the picture, so it set the
# colour scale for all of it.
#
# The obstacle stops before the section closes. What is left out is the last
# few per cent of the chord, which is a millimetre of trailing edge the air
# cannot tell is missing, and what is gained is a field near the surface that
# means something.
TE_TRIM = 0.97

# How finely each surface is panelised as an obstacle.
#
# Coarse, and refining it is wrong. A source panel models thickness, and a
# lifting surface is thin: halving the panels puts the upper and lower
# surfaces' panels closer together relative to their own size, the influence
# matrix gets worse rather than better, and the strongest source went from
# 8.5 x freestream to 632. tools/check_panels.mjs is the measurement.
NS_W, NC_W = 9, 12
NS_V, NC_V = 7, 10
NS_T, NC_T = 6, 9


def lifting_surfaces():
    """Wings, flaperons, stabilators, fin and ventrals as flow obstacles.

    Only the fuselage was panelised, so the source solve made the air go
    round the body and straight THROUGH everything else -- streamlines
    passed through the wings and the tail as if they were not there. The
    vortex lattice makes their lift, but a lattice is invisible to a
    streamline: it has no thickness for the air to flow around.

    Built coarse on purpose. The solve is a dense LU on the panel count, so
    these are the shape of the surface rather than its finish.
    """
    from parts import common as pc
    W, V = spec.WING, spec.VTAIL
    out = []

    for mir in (False, True):
        v, f = pc.panel(
            root_le=(W["x_root_le"], 0.0, W["z_root"]),
            root_chord=W["root_chord"], tip_chord=W["tip_chord"],
            semi_span=W["semi_span"], sweep_le=W["sweep_le"],
            dihedral=W["dihedral"], thickness=W["thickness"],
            planform=spec.WING_PLANFORM, thickness_tip=W["thickness_tip"],
            tip_cap=4, camber=W["camber"], twist_root=W["incidence"],
            twist_tip=W["incidence"] - W["washout"],
            u0=0.0, u1=TE_TRIM, n_span=NS_W, n_chord=NC_W, mirror=mir)
        out += from_mesh(v, f)

    v, f = pc.panel(
        root_le=(V["x_root_le"], 0.0, V["z_root"]),
        root_chord=V["root_chord"], tip_chord=V["tip_chord"],
        semi_span=V["height"], sweep_le=V["sweep_le"],
        thickness=V["thickness"], thickness_tip=V["thickness_tip"],
        u1=TE_TRIM, tip_cap=4, n_span=NS_V, n_chord=NC_V, vertical=True)
    out += from_mesh(v, f)

    T = spec.HTAIL
    for mir in (False, True):
        v, f = pc.panel(
            root_le=(T["x_root_le"], 0.0, T["z_root"]),
            root_chord=T["root_chord"], tip_chord=T["tip_chord"],
            semi_span=T["semi_span"], sweep_le=T["sweep_le"],
            dihedral=T["anhedral"], thickness=T["thickness"],
            thickness_tip=T["thickness_tip"], tip_cap=4,
            twist_root=T["deflect"], twist_tip=T["deflect"],
            u1=TE_TRIM, n_span=NS_T, n_chord=NC_T, mirror=mir)
        out += from_mesh(v, f)
    return out


def _no_slivers(panels, frac=0.30):
    """Drop panels too small for the solve to say anything about.

    A source panel's strength should come out the order of the freestream. Two
    panels a fraction of their own size apart facing opposite ways make the
    influence matrix near-singular, the strengths run away, and the field
    within a few millimetres of them is not flow but discretisation noise.

    A fifth of a millimetre of a 440 mm aeroplane is not a shape the air can
    tell is there. Dropping it changes the obstacle by nothing and changes the
    field beside it completely.
    """
    sides = sorted(math.sqrt(a) for (_c, _n, a) in panels)
    floor = sides[len(sides) // 2] * frac
    return [p for p in panels if math.sqrt(p[2]) >= floor]


def build():
    panels = _no_slivers(fuselage() + lifting_surfaces())
    return {
        "n": len(panels),
        "c": [v for (c, n, a) in panels for v in c],
        "n_": [v for (c, n, a) in panels for v in n],
        "a": [a for (c, n, a) in panels],
    }


if __name__ == "__main__":
    b = build()
    print(f"{b['n']} body panels, wetted area {sum(b['a']):.3f} m2")
