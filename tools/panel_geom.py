"""The RC jet as a closed quadrilateral surface, for the panel solve.

The machinery -- the quad book-keeping, the winding, the trimming, the
aerofoil sections, the control tagging -- is in _shared/tools/panelgeom.py
and is vendored in beside this file. What is here is this aeroplane: a lofted
superellipse body, a cropped delta running tip to tip through it, all-moving
stabilators and a fin.

Two decisions are this aeroplane's rather than the method's:

  * The intake mouth is CLOSED. The old source panelisation deliberately cut
    it open so air could get in; a Dirichlet interior condition needs an
    inside, and a body with a hole in the nose does not have one. An intake
    is a prescribed normal velocity on those panels, which is what an intake
    actually is, and it gets the exhaust right too, which a hole never could.

  * The wing runs TIP TO TIP through the fuselage rather than being capped at
    its side. Capped, it is a wing with two tips: its circulation has to fall
    to zero at the root and it sheds a root vortex the real aeroplane does
    not have. That cost 40 % of the lift slope. Run through and trimmed at
    the seam, the doublet distribution carries straight on into the body,
    which is what the circulation does.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "plane"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import spec                                    # noqa: E402
from parts import fuselage as fus              # noqa: E402
from panelgeom import (MM, Geom, orient, solid_angle, surface, trim,
                       control, _centroid, _size)   # noqa: E402

def fuselage(g, n_x=20, n_theta=16):
    """The body: a lofted superellipse tube with both ends capped.

    Closed, including across the intake mouth and the nozzle. Those panels are
    recorded in `inflow` so the solve can push air through them instead of
    leaving a hole in the boundary condition.
    """
    x0, x1 = spec.FUSELAGE[0][0], spec.FUSELAGE[-1][0]
    xs = [x0 + (x1 - x0) * 0.5 * (1 - math.cos(math.pi * i / (n_x - 1)))
          for i in range(n_x)]
    rings = [fus.section_ring(x, segments=n_theta) for x in xs]

    grid = []
    for i in range(n_x - 1):
        row = []
        for j in range(n_theta):
            k = (j + 1) % n_theta
            # wound so the normal points out of the body
            row.append([rings[i][j], rings[i][k], rings[i + 1][k], rings[i + 1][j]])
        grid.append(row)
    g.patch("fuselage", grid, n_x - 1, n_theta, wrap_j=True, body="fuselage")

    # end caps, as triangles written as quads with a repeated corner
    for (ring, x, sgn) in ((rings[0], xs[0], -1), (rings[-1], xs[-1], 1)):
        w, h, zc, nn = fus.station_at(x)
        apex = (x, 0.0, zc)
        row = []
        for j in range(n_theta):
            k = (j + 1) % n_theta
            q = ([ring[j], ring[k], apex, apex] if sgn > 0
                 else [ring[k], ring[j], apex, apex])
            row.append(q)
        g.patch("nose_cap" if sgn < 0 else "tail_cap", [row], 1, n_theta,
                wrap_j=True, body="fuselage")


# ---------------------------------------------------------- lifting surfaces

def _fuse_half_width(x, z):
    """Half-width of the body at station x and height z, from the same
    superellipse the fuselage is lofted from."""
    w, h, zc, n = fus.station_at(x)
    t = abs((z - zc) / h)
    if t >= 1.0:
        return 0.0
    return w * (1.0 - t ** n) ** (1.0 / n)


def _wing_stations(ns):
    """Spanwise stations off the planform table, tip to tip.

    Tip to tip, not root to tip, and the two halves are one surface with no
    cap between them. A wing capped at the fuselage side is a wing with two
    tips: its bound circulation has to fall to zero at the root and it sheds a
    root vortex there, which a real wing-body does not have because the
    circulation carries through the body. Capped at the side of this fuselage
    the lift slope came out at 1.07 per radian against the 1.78 an aspect
    ratio 1.81 delta should have.

    The price is that the inner wing passes through the fuselage, so those
    panels are buried inside another closed body. That is the ordinary
    wing-body compromise and the alternative -- trimming both surfaces along
    their intersection curve -- is a different program.
    """
    W, PL = spec.WING, spec.WING_PLANFORM
    out = []
    for i in range(ns + 1):
        f = -1.0 + 2.0 * (i / ns)
        # planform table is (fraction, x_le, chord)
        af = abs(f)
        for k in range(len(PL) - 1):
            if PL[k][0] <= af <= PL[k + 1][0]:
                a, b = PL[k], PL[k + 1]
                t = (af - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                x_le = a[1] + (b[1] - a[1]) * t
                chord = a[2] + (b[2] - a[2]) * t
                break
        else:
            x_le, chord = PL[-1][1], PL[-1][2]
        tc = W["thickness"] + (W["thickness_tip"] - W["thickness"]) * af
        tw = W["incidence"] - W["washout"] * af
        z = W["z_root"] + math.tan(math.radians(W["dihedral"])) * af * W["semi_span"]
        out.append((x_le, f * W["semi_span"], z, chord, tc, tw))
    return out


def _panel_stations(S, ns, span_key, sweep_key, dihedral_deg,
                    twist, z_key="z_root"):
    """Tip to tip, for the same reason the wing is."""
    out = []
    span = S[span_key]
    for i in range(ns + 1):
        f = -1.0 + 2.0 * (i / ns)
        y = f * span
        af = abs(f)
        x_le = S["x_root_le"] + abs(y) * math.tan(math.radians(S[sweep_key]))
        chord = S["root_chord"] + (S["tip_chord"] - S["root_chord"]) * af
        tc = S["thickness"] + (S["thickness_tip"] - S["thickness"]) * af
        z = S[z_key] + abs(y) * math.tan(math.radians(dihedral_deg))
        out.append((x_le, y, z, chord, tc, twist))
    return out


def _mirror(stations):
    return [(x, -y, z, c, t, w) for (x, y, z, c, t, w) in stations]


# The intake mouth, on the chin under the nose. Same superellipse the old
# source panelisation used to cut a HOLE with -- but a Dirichlet interior
# condition needs an inside, so the mouth stays closed and the engine becomes
# a prescribed normal velocity through those panels instead. That is what an
# intake is: a piece of skin the air goes through.
INTAKE = {"x0": 44.0, "x1": 68.0, "y": 23.0, "z": -17.0, "h": 13.5}


def engine_faces(g):
    """Tag the panels the engine breathes through.

    Ingest through the intake mouth, exhaust through the nozzle. Both are
    ordinary panels carrying an ordinary boundary condition; what makes them
    an engine is that the condition says air crosses them.
    """
    intake, exhaust = [], []
    for i, q in enumerate(g.quads):
        c = _centroid(q)
        if g.body[i] == "fuselage" and INTAKE["x0"] <= c[0] <= INTAKE["x1"]:
            if ((c[1] / INTAKE["y"]) ** 2
                    + ((c[2] - INTAKE["z"]) / INTAKE["h"]) ** 2) <= 1.0:
                intake.append(i)
    for (name, start, count) in g.parts:
        if name == "tail_cap":
            exhaust = list(range(start, start + count))
    for i in intake:
        g.inflow.append([i, -1])
    for i in exhaust:
        g.inflow.append([i, 1])


def build(n_x=20, n_theta=16, gap=None):
    """The whole aeroplane, trimmed into one surface."""
    g = Geom()
    fuselage(g, n_x, n_theta)

    # ---- wing, tip to tip through the body
    # 28 points a side round the section, which is more than it looks.
    #
    # The neutral point is the least forgiving thing this solve produces: it
    # is a ratio of two integrals of surface pressure, and at 12 chordwise
    # points it read 45 % of the mean chord measured between 0 and 4 degrees
    # and 40 % measured between 6 and 12 -- a number that depends on where you
    # measure it is not a number. At 28 it is 39.6 % at every pair, which is
    # what a converged linear solution looks like. Spanwise resolution moves
    # it far less; this is a chordwise pressure distribution being resolved.
    surface(g, "wing", _wing_stations(10), m=28)

    # ---- stabilators, all-moving, at their deflection
    T = spec.HTAIL
    # built UNDEFLECTED: the deflection is a boundary condition, not geometry
    stab_st = _panel_stations(T, 8, "semi_span", "sweep_le", T["anhedral"], 0.0)
    surface(g, "stab", stab_st, m=12)

    # ---- fin, rooted below the spine so it trims into the body
    V = spec.VTAIL
    ns = 5
    st = []
    for i in range(ns + 1):
        f = -0.12 + 1.12 * (i / ns)
        z = V["z_root"] + f * V["height"]
        af = max(f, 0.0)
        st.append((V["x_root_le"] + af * V["height"] * math.tan(math.radians(V["sweep_le"])),
                   z, 0.0,
                   V["root_chord"] + (V["tip_chord"] - V["root_chord"]) * af,
                   V["thickness"] + (V["thickness_tip"] - V["thickness"]) * af,
                   0.0))
    surface(g, "fin", st, m=10, vertical=True)

    wing_st = _wing_stations(10)
    control(g, "flaperon", "wing", wing_st, 0.74, 0.30, 0.92)
    # all-moving, and its pivot is across the aircraft rather than along the
    # swept leading edge
    control(g, "stabilator", "stab", stab_st, 0.0, 0.0, 1.0, axis=(0, 1, 0))
    control(g, "rudder", "fin", st, 1.0 - V["rudder_chord"], 0.0,
            V["rudder_span"], vertical=True)

    g.trimmed = trim(g)
    engine_faces(g)
    return g


def emit(**kw):
    return build(**kw).emit()


if __name__ == "__main__":
    import json
    d = emit()
    print(f"{d['n']} panels, {len(d['te'])} trailing edges, "
          f"{sum(1 for nb in d['nb'] for k in nb if k < 0)} open edges")
    for p in d["parts"]:
        print(f"   {p['name']:16s} {p['count']:4d} panels")
